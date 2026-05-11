#include "sim_core.h"

#include <cmath>
#include <cstring>
#include <cstdio>
#include <thread>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

// ---------------------------------------------------------------------------
// Constructor / Destructor
// ---------------------------------------------------------------------------

QuadrotorSimCore::QuadrotorSimCore(const std::string& model_path,
                                   const Params& params)
  : model_path_(model_path), params_(params) {
  load_model(model_path_);
  if (!m_) return;

  // Create shared memory directory and segments
  mkdir("/dev/shm/quadrotor_sim", 0755);
  shm_state_ = create_state_shm();
  shm_ctrl_  = create_ctrl_shm();
  zero_struct(shm_state_);
  zero_struct(shm_ctrl_);

  // Allocate control noise
  ctrlnoise_ = static_cast<mjtNum*>(calloc(m_->nu, sizeof(mjtNum)));

  cache_sensor_indices();
}

QuadrotorSimCore::~QuadrotorSimCore() {
  free(ctrlnoise_);
  if (shm_state_) munmap(shm_state_, sizeof(QuadrotorState));
  if (shm_ctrl_)  munmap(shm_ctrl_,  sizeof(QuadrotorControl));
  mj_deleteData(d_);
  mj_deleteModel(m_);
}

// ---------------------------------------------------------------------------
// Model loading
// ---------------------------------------------------------------------------

void QuadrotorSimCore::load_model(const std::string& model_path) {
  char loadError[kErrorBufSize] = "";
  m_ = mj_loadXML(model_path.c_str(), nullptr, loadError,
                  static_cast<int>(model_path.size()));
  if (!m_) {
    std::fprintf(stderr, "sim_core: cannot load model: %s\n", loadError);
    return;
  }
  d_ = mj_makeData(m_);
  if (d_) {
    mj_forward(m_, d_);
  } else {
    mj_deleteModel(m_);
    m_ = nullptr;
  }
}

void QuadrotorSimCore::cache_sensor_indices() {
  if (!m_) return;
  for (int i = 0; i < m_->nsensor; i++) {
    switch (m_->sensor_type[i]) {
      case mjSENS_VELOCIMETER:   sensor_adr_velocimeter_   = m_->sensor_adr[i]; break;
      case mjSENS_GYRO:          sensor_adr_gyro_          = m_->sensor_adr[i]; break;
      case mjSENS_ACCELEROMETER: sensor_adr_accelerometer_ = m_->sensor_adr[i]; break;
    }
  }
}

// ---------------------------------------------------------------------------
// Publish state to shared memory (seqlock-protected)
// ---------------------------------------------------------------------------

static uint64_t monotonic_ns() {
  return static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch()).count());
}

void QuadrotorSimCore::extract_state(QuadrotorState& out) const {
  // Write directly into the output struct, then bump seqlock
  out.time = d_->time;

  out.position[0] = d_->qpos[0];
  out.position[1] = d_->qpos[1];
  out.position[2] = d_->qpos[2];

  out.orientation[0] = d_->qpos[3];
  out.orientation[1] = d_->qpos[4];
  out.orientation[2] = d_->qpos[5];
  out.orientation[3] = d_->qpos[6];

  if (sensor_adr_velocimeter_ >= 0) {
    out.linear_velocity[0] = d_->sensordata[sensor_adr_velocimeter_    ];
    out.linear_velocity[1] = d_->sensordata[sensor_adr_velocimeter_ + 1];
    out.linear_velocity[2] = d_->sensordata[sensor_adr_velocimeter_ + 2];
  }

  if (sensor_adr_gyro_ >= 0) {
    out.angular_velocity[0] = d_->sensordata[sensor_adr_gyro_    ];
    out.angular_velocity[1] = d_->sensordata[sensor_adr_gyro_ + 1];
    out.angular_velocity[2] = d_->sensordata[sensor_adr_gyro_ + 2];
  }

  if (sensor_adr_accelerometer_ >= 0) {
    out.linear_acceleration[0] = d_->sensordata[sensor_adr_accelerometer_    ];
    out.linear_acceleration[1] = d_->sensordata[sensor_adr_accelerometer_ + 1];
    out.linear_acceleration[2] = d_->sensordata[sensor_adr_accelerometer_ + 2];
  }

  out.timestamp_ns = monotonic_ns();
}

// ---------------------------------------------------------------------------
// Control application
// ---------------------------------------------------------------------------

void QuadrotorSimCore::apply_control() {
  if (!m_ || !d_ || !shm_ctrl_) return;

  // Read control from shared memory with seqlock
  QuadrotorControl ctrl;
  while (!shm_read(*shm_ctrl_, ctrl)) {
    /* retry until consistent snapshot */
  }

  d_->ctrl[0] = ctrl.thrust;
  d_->ctrl[1] = ctrl.torque[0];
  d_->ctrl[2] = ctrl.torque[1];
  d_->ctrl[3] = ctrl.torque[2];

  // Apply control noise (Ornstein-Uhlenbeck process)
  if (params_.ctrlnoise_std > 0 && ctrlnoise_) {
    mjtNum rate  = std::exp(-m_->opt.timestep /
                            std::max(params_.ctrlnoise_rate, static_cast<double>(mjMINVAL)));
    mjtNum scale = params_.ctrlnoise_std * std::sqrt(1.0 - rate * rate);

    for (int i = 0; i < m_->nu; i++) {
      ctrlnoise_[i] = rate * ctrlnoise_[i] + scale * mju_standardNormal(nullptr);
      d_->ctrl[i] += ctrlnoise_[i];
    }
  }
}

// ---------------------------------------------------------------------------
// Publish state to shared memory (seqlock-protected)
// ---------------------------------------------------------------------------

void QuadrotorSimCore::publish_state() {
  // Temporarily use sequence to mark write-in-progress
  shm_write_begin(shm_state_->sequence);
  extract_state(*shm_state_);
  shm_write_end(shm_state_->sequence);
}

// ---------------------------------------------------------------------------
// Single physics step (with pre/post cleanup)
// ---------------------------------------------------------------------------

void QuadrotorSimCore::physics_step() {
  mju_zero(d_->xfrc_applied, 6 * m_->nbody);
  apply_control();
  mj_step(m_, d_);
}

// ---------------------------------------------------------------------------
// Main physics loop
// ---------------------------------------------------------------------------

void QuadrotorSimCore::run() {
  if (!valid()) return;

  std::printf("sim_core: model loaded, starting physics loop\n");

  // Initialize sync
  sync_cpu_ = Clock::now();
  sync_sim_ = d_->time;

  while (!exit_request_.load(std::memory_order_relaxed)) {
    // Yield or sleep to give other threads/processes CPU time
    std::this_thread::sleep_for(std::chrono::milliseconds(1));

    double slowdown = params_.real_time_factor > 0.0
                          ? 1.0 / params_.real_time_factor
                          : 1.0;

    // Record CPU time at start of iteration
    auto start_cpu = Clock::now();
    auto elapsed_cpu = start_cpu - sync_cpu_;
    double elapsed_cpu_s = std::chrono::duration<double>(elapsed_cpu).count();
    double elapsed_sim   = d_->time - sync_sim_;

    // Check misalignment
    bool misaligned = std::abs(elapsed_cpu_s / slowdown - elapsed_sim) > kSyncMisalign;

    if (elapsed_sim < 0 || elapsed_cpu_s < 0 ||
        sync_cpu_.time_since_epoch().count() == 0 ||
        misaligned || speed_changed_) {
      // Resync
      sync_cpu_ = start_cpu;
      sync_sim_ = d_->time;
      speed_changed_ = false;

      physics_step();

      // Publish state via seqlock
      publish_state();
    } else {
      // In-sync: step until ahead of CPU
      double refresh_time = kSimRefreshFraction / kRefreshRate;

      while (!exit_request_.load(std::memory_order_relaxed)) {
        auto now = Clock::now();
        double sim_ahead = d_->time - sync_sim_;
        double cpu_ahead = std::chrono::duration<double>(now - sync_cpu_).count();

        if (sim_ahead * slowdown >= cpu_ahead) break;
        if (std::chrono::duration<double>(now - start_cpu).count() >= refresh_time) break;

        physics_step();
      }

      // Write state after batch
      publish_state();
    }


  }

  std::printf("sim_core: physics loop exited\n");
}

void QuadrotorSimCore::request_shutdown() {
  exit_request_.store(true, std::memory_order_relaxed);
}
