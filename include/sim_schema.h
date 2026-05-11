#pragma once

#include <cstdint>

/**
 * @file sim_schema.h
 * ABI-stable shared-memory schema for the quadrotor simulator.
 *
 * The core simulator and all adapters (ROS, GLFW, Python, etc.) communicate
 * through three fixed-layout segments under /dev/shm/quadrotor_sim/:
 *   - state : QuadrotorState   (core writes, adapters read)
 *   - ctrl  : QuadrotorControl (adapters write, core reads)
 *   - image : ImageData        (core writes, adapters read)
 *
 * Synchronization: seqlock (monotonic sequence number + memory barriers).
 * All structs use #pragma pack(1) for ABI stability across compilers.
 */

#pragma pack(push, 1)

/* -------------------------------------------------------------------------- */
/* Shared memory path constants                                              */
/* -------------------------------------------------------------------------- */

static constexpr const char *SHM_BASE_DIR = "/dev/shm/quadrotor_sim";
static constexpr const char *SHM_STATE_FILE = "/dev/shm/quadrotor_sim/state";
static constexpr const char *SHM_CTRL_FILE  = "/dev/shm/quadrotor_sim/ctrl";
static constexpr const char *SHM_IMAGE_FILE = "/dev/shm/quadrotor_sim/image";

/* -------------------------------------------------------------------------- */
/* Physical limits (aligned with drone.xml)                                   */
/* -------------------------------------------------------------------------- */

static constexpr double SIM_GRAVITY_Z     = -9.81;    /**< [m/s²] */
static constexpr double SIM_TIMESTEP      = 0.001;    /**< [s]    */
static constexpr double SIM_DENSITY       = 1.225;    /**< [kg/m³]*/
static constexpr double SIM_VISCOSITY     = 1.8e-5;   /**< [Pa·s] */
static constexpr double SIM_THRUST_MIN    = 0.0;      /**< [N]    */
static constexpr double SIM_THRUST_MAX    = 42.0;     /**< [N]    */
static constexpr double SIM_TORQUE_MIN    = -0.5;     /**< [Nm]   */
static constexpr double SIM_TORQUE_MAX    = 0.5;      /**< [Nm]   */
static constexpr double SIM_TOTAL_MASS    = 1.0;      /**< [kg]    */

/* -------------------------------------------------------------------------- */
/* Image limits                                                               */
/* -------------------------------------------------------------------------- */

static constexpr uint32_t IMAGE_MAX_WIDTH  = 1280;
static constexpr uint32_t IMAGE_MAX_HEIGHT = 720;
static constexpr uint32_t IMAGE_MAX_BYTES  = IMAGE_MAX_WIDTH * IMAGE_MAX_HEIGHT * 3;  // rgb8

/* -------------------------------------------------------------------------- */
/* Control input (adapters → core)                                            */
/* -------------------------------------------------------------------------- */

struct QuadrotorControl {
  uint64_t sequence;        /**< seqlock counter (monotonic)       */
  double   thrust;          /**< body-frame Z force [N]            */
  double   torque[3];       /**< body-frame x/y/z moments [Nm]     */
  uint64_t timestamp_ns;    /**< CLOCK_MONOTONIC [ns]              */
  uint8_t  _pad[16];        /**< align to 64 bytes                 */
};
static_assert(sizeof(QuadrotorControl) == 64, "QuadrotorControl must be 64 bytes");

/* -------------------------------------------------------------------------- */
/* State output (core → adapters)                                             */
/* -------------------------------------------------------------------------- */

struct QuadrotorState {
  uint64_t sequence;                /**< seqlock counter (monotonic)          */
  double   time;                    /**< simulation time [s]                  */
  double   position[3];             /**< world-frame x, y, z [m]             */
  double   orientation[4];          /**< world-frame quaternion w, x, y, z   */
  double   linear_velocity[3];      /**< body-frame vx, vy, vz [m/s]         */
  double   angular_velocity[3];     /**< body-frame wx, wy, wz [rad/s]       */
  double   linear_acceleration[3];  /**< body-frame ax, ay, az [m/s²]        */
  uint64_t timestamp_ns;            /**< CLOCK_MONOTONIC [ns]                 */
  uint8_t  _pad[40];                /**< align to 192 bytes                  */
};
static_assert(sizeof(QuadrotorState) == 192, "QuadrotorState must be 192 bytes");

/* -------------------------------------------------------------------------- */
/* Image output (core → adapters, optional)                                   */
/* -------------------------------------------------------------------------- */

struct ImageData {
  uint64_t sequence;        /**< seqlock counter                           */
  uint32_t width;           /**< image width [px]                          */
  uint32_t height;          /**< image height [px]                         */
  uint32_t encoding;        /**< FourCC: 0x72363862 = 'rgb8'              */
  uint64_t timestamp_ns;    /**< CLOCK_MONOTONIC [ns]                      */
  uint8_t  _pad[36];        /**< pad header to 64 bytes                    */
  uint8_t  data[IMAGE_MAX_BYTES];  /**< pixel data, row-major, W*H*3 bytes */
};
static_assert(sizeof(ImageData) == 64 + IMAGE_MAX_BYTES, "ImageData size mismatch");

#pragma pack(pop)

/* -------------------------------------------------------------------------- */
/* Seqlock helpers                                                            */
/* -------------------------------------------------------------------------- */

/** Acquire seqlock for writing: bumps seq to odd. Call in pairs with shm_write_end. */
inline void shm_write_begin(uint64_t &seq) {
  seq++;
  __sync_synchronize();  // full memory barrier (store-store + store-load)
}

/** Release seqlock: bumps seq to even. Must follow shm_write_begin. */
inline void shm_write_end(uint64_t &seq) {
  __sync_synchronize();
  seq++;
}

/**
 * Read data under seqlock protection (volatile source).
 *
 * @return true if a consistent snapshot was read, false if retry needed.
 */
template<typename T>
inline bool shm_read_vol(const volatile T &src, T &dst) {
  uint64_t before = src.sequence;
  if (before & 1ULL) return false;        // writer is active
  __sync_synchronize();                    // acquire barrier
  dst = const_cast<const T &>(src);        // copy
  __sync_synchronize();                    // ensure copy finishes before re-reading seq
  uint64_t after = src.sequence;
  return before == after;
}

/**
 * Read data under seqlock protection (non-volatile source).
 *
 * @return true if a consistent snapshot was read, false if retry needed.
 */
template<typename T>
inline bool shm_read(const T &src, T &dst) {
  return shm_read_vol(
      *reinterpret_cast<const volatile T *>(&src), dst);
}
