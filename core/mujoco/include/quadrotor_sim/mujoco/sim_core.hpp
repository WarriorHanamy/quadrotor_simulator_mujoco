#pragma once

#include <atomic>
#include <chrono>
#include <string>

#include <mujoco/mujoco.h>

#include "quadrotor_sim/types.hpp"

namespace quadrotor_sim::mujoco {

/**
 * Headless MuJoCo physics engine for the quadrotor.
 *
 * Owns mjModel* and mjData*. Reads/writes quadrotor state and control
 * through shared memory via the shm:: protocol.
 */
class SimCore {
public:
  struct Params {
    double real_time_factor = 1.0;
    double ctrlnoise_std   = 0.0;
    double ctrlnoise_rate  = 0.0;
  };

  SimCore(const std::string& model_path, const Params& params);
  ~SimCore();

  SimCore(const SimCore&) = delete;
  SimCore& operator=(const SimCore&) = delete;

  bool valid() const { return m_ != nullptr && d_ != nullptr; }
  void Run();
  void RequestShutdown();

  /** Single physics step — for manual stepping (no Run loop). */
  void Step();
  double time() const { return d_ ? d_->time : 0.0; }

private:
  void LoadModel(const std::string& path);
  void CacheSensorIndices();
  void ExtractState();
  void ApplyControl();

  std::string model_path_;
  Params params_;

  mjModel* m_ = nullptr;
  mjData*  d_ = nullptr;

  int sensor_vel_   = -1;
  int sensor_gyro_  = -1;
  int sensor_accel_ = -1;

  mjtNum* ctrlnoise_ = nullptr;

  class Shm;  // PIMPL to hide SHM details from public header
  Shm* shm_ = nullptr;

  std::atomic<bool> exit_{false};

  // RT sync state
  using Clock = std::chrono::steady_clock;
  Clock::time_point sync_cpu_;
  mjtNum sync_sim_ = 0.0;
  bool speed_changed_ = true;

  static constexpr double kSyncMisalign       = 0.1;
  static constexpr double kSimRefreshFraction = 0.7;
  static constexpr int    kErrorBufSize       = 1024;
  static constexpr int    kRefreshRate        = 60;
};

}  // namespace quadrotor_sim::mujoco
