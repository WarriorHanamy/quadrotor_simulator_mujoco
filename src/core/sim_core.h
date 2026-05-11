#pragma once

#include <atomic>
#include <chrono>
#include <string>

#include <mujoco/mujoco.h>

#include "sim_schema.h"
#include "shm_backend.h"

/**
 * Headless MuJoCo quadrotor physics engine.
 *
 * Loads the drone.xml model, runs mj_step() in a loop,
 * writes QuadrotorState to shared memory, and reads QuadrotorControl from shm.
 *
 * Zero ROS / GLFW dependency.
 */
class QuadrotorSimCore {
public:
  struct Params {
    double real_time_factor = 1.0;   /**< 1.0 = real-time, 0.1 = 10x slow  */
    double ctrlnoise_std   = 0.0;    /**< Ornstein-Uhlenbeck noise std      */
    double ctrlnoise_rate  = 0.0;    /**< OU rate parameter                 */
  };

  /**
   * @param[in] model_path  Path to drone.xml or .mjb file
   * @param[in] params      Simulation parameters
   */
  QuadrotorSimCore(const std::string& model_path, const Params& params);
  ~QuadrotorSimCore();

  QuadrotorSimCore(const QuadrotorSimCore&) = delete;
  QuadrotorSimCore& operator=(const QuadrotorSimCore&) = delete;

  /** Start the physics loop. Blocks until shutdown. */
  void run();

  /** Request clean shutdown from another thread. */
  void request_shutdown();

  /** True if the simulator is in a valid state. */
  bool valid() const { return m_ != nullptr && d_ != nullptr; }

private:
  void load_model(const std::string& model_path);
  void cache_sensor_indices();
  void extract_state(QuadrotorState& out) const;
  void publish_state();
  void apply_control();
  void physics_step();

  std::string model_path_;
  Params params_;

  mjModel* m_ = nullptr;
  mjData* d_ = nullptr;

  // Sensor address caches
  int sensor_adr_velocimeter_    = -1;
  int sensor_adr_gyro_           = -1;
  int sensor_adr_accelerometer_  = -1;

  // Control noise
  mjtNum* ctrlnoise_ = nullptr;

  // Shared memory pointers (owned by this process)
  QuadrotorState*  shm_state_ = nullptr;
  QuadrotorControl* shm_ctrl_  = nullptr;

  // Threading
  std::atomic<bool> exit_request_{false};

  // Real-time sync state
  using Clock = std::chrono::steady_clock;
  Clock::time_point sync_cpu_;
  mjtNum sync_sim_ = 0.0;
  bool speed_changed_ = true;

  static constexpr double kSyncMisalign       = 0.1;     /**< [s] */
  static constexpr double kSimRefreshFraction = 0.7;
  static constexpr int    kErrorBufSize       = 1024;
  static constexpr int    kRefreshRate        = 60;       /**< [Hz] */
};
