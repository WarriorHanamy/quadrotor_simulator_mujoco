#include "quadrotor_sim/mujoco/sim_core.hpp"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <thread>

#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include "quadrotor_sim/shm/shm_layout.hpp"
#include "quadrotor_sim/shm/shm_backend.hpp"

namespace quadrotor_sim::mujoco {

using namespace quadrotor_sim::shm;

class SimCore::Shm {
public:
  Shm() {
    mkdir(kBaseDir, 0755);
    state = CreateStateShm();
    ctrl  = CreateCtrlShm();
    memset(state, 0, sizeof(StateWire));
    memset(ctrl, 0, sizeof(ControlWire));
  }
  ~Shm() {
    munmap(state, sizeof(StateWire));
    munmap(ctrl, sizeof(ControlWire));
  }
  StateWire* state;
  ControlWire* ctrl;
};

SimCore::SimCore(const std::string& model_path, const Params& params)
    : model_path_(model_path), params_(params) {
  LoadModel(model_path_);
  if (!m_) return;
  shm_ = new Shm();
  ctrlnoise_ = static_cast<mjtNum*>(calloc(m_->nu, sizeof(mjtNum)));
  CacheSensorIndices();
}

SimCore::~SimCore() {
  free(ctrlnoise_);
  delete shm_;
  if (d_) mj_deleteData(d_);
  if (m_) mj_deleteModel(m_);
}

void SimCore::LoadModel(const std::string& path) {
  char err[kErrorBufSize] = "";
  m_ = mj_loadXML(path.c_str(), nullptr, err, kErrorBufSize);
  if (!m_) { std::fprintf(stderr, "sim_core: load failed: %s\n", err); return; }
  d_ = mj_makeData(m_);
  if (d_) {
    mj_forward(m_, d_);
  } else {
    mj_deleteModel(m_);
    m_ = nullptr;
  }
}

void SimCore::CacheSensorIndices() {
  if (!m_) return;
  for (int i = 0; i < m_->nsensor; i++) {
    switch (m_->sensor_type[i]) {
      case mjSENS_VELOCIMETER:   sensor_vel_   = m_->sensor_adr[i]; break;
      case mjSENS_GYRO:          sensor_gyro_  = m_->sensor_adr[i]; break;
      case mjSENS_ACCELEROMETER: sensor_accel_ = m_->sensor_adr[i]; break;
    }
  }
}

void SimCore::ExtractState() {
  State s;
  s.time = d_->time;

  for (int i = 0; i < 3; i++) s.position[i]     = d_->qpos[i];
  for (int i = 0; i < 4; i++) s.orientation[i]   = d_->qpos[3 + i];

  if (sensor_vel_ >= 0)
    for (int i = 0; i < 3; i++) s.linear_velocity[i] = d_->sensordata[sensor_vel_ + i];
  if (sensor_gyro_ >= 0)
    for (int i = 0; i < 3; i++) s.angular_velocity[i] = d_->sensordata[sensor_gyro_ + i];
  if (sensor_accel_ >= 0)
    for (int i = 0; i < 3; i++) s.linear_acceleration[i] = d_->sensordata[sensor_accel_ + i];

  StateWire sw;
  ToWire(s, sw);
  WriteBegin(shm_->state->sequence);
  sw.sequence = shm_->state->sequence;
  *shm_->state = sw;
  WriteEnd(shm_->state->sequence);
}

void SimCore::ApplyControl() {
  if (!m_ || !d_ || !shm_) return;

  ControlWire cw;
  while (!ReadConsistent(*shm_->ctrl, cw)) {}

  Control c;
  FromWire(cw, c);

  d_->ctrl[0] = c.thrust;
  d_->ctrl[1] = c.torque[0];
  d_->ctrl[2] = c.torque[1];
  d_->ctrl[3] = c.torque[2];

  if (params_.ctrlnoise_std > 0 && ctrlnoise_) {
    mjtNum rate = std::exp(-m_->opt.timestep / std::max(params_.ctrlnoise_rate, mjMINVAL));
    mjtNum scale = params_.ctrlnoise_std * std::sqrt(1.0 - rate * rate);
    for (int i = 0; i < m_->nu; i++) {
      ctrlnoise_[i] = rate * ctrlnoise_[i] + scale * mju_standardNormal(nullptr);
      d_->ctrl[i] += ctrlnoise_[i];
    }
  }
}

void SimCore::Step() {
  mju_zero(d_->xfrc_applied, 6 * m_->nbody);
  ApplyControl();
  mj_step(m_, d_);
}

void SimCore::Run() {
  if (!valid()) return;
  std::fprintf(stderr, "sim_core: physics loop starting\n");
  sync_cpu_ = Clock::now();
  sync_sim_ = d_->time;

  while (!exit_.load(std::memory_order_relaxed)) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    double slowdown = params_.real_time_factor > 0.0 ? 1.0 / params_.real_time_factor : 1.0;
    auto start = Clock::now();
    double elapsed_cpu = std::chrono::duration<double>(start - sync_cpu_).count();
    double elapsed_sim = d_->time - sync_sim_;
    bool misaligned = std::abs(elapsed_cpu / slowdown - elapsed_sim) > kSyncMisalign;

    if (elapsed_sim < 0 || elapsed_cpu < 0 ||
        sync_cpu_.time_since_epoch().count() == 0 || misaligned || speed_changed_) {
      sync_cpu_ = start;
      sync_sim_ = d_->time;
      speed_changed_ = false;
      Step();
      ExtractState();
    } else {
      double refresh = kSimRefreshFraction / kRefreshRate;
      while (!exit_.load(std::memory_order_relaxed)) {
        auto now = Clock::now();
        double sim_ahead = d_->time - sync_sim_;
        double cpu_ahead = std::chrono::duration<double>(now - sync_cpu_).count();
        if (sim_ahead * slowdown >= cpu_ahead) break;
        if (std::chrono::duration<double>(now - start).count() >= refresh) break;
        Step();
      }
      ExtractState();
    }
  }
  std::fprintf(stderr, "sim_core: physics loop exited\n");
}

void SimCore::RequestShutdown() {
  exit_.store(true, std::memory_order_relaxed);
}

}  // namespace quadrotor_sim::mujoco
