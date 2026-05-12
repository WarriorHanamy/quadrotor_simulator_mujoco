#pragma once

#include <string>

#include "sim_schema.h"

/**
 * Reference state for the SE(3) geometric controller.
 *
 * Tracks a desired position and yaw angle in world ENU frame.
 */
struct Se3Setpoint {
  double position[3] = {0.0, 0.0, 2.0};   /**< world ENU [m]      */
  double yaw       = 0.0;                /**< desired yaw [rad]   */
};

/**
 * Geometric SE(3) quadrotor controller.
 *
 * Computes body-frame thrust and torques from a position + yaw setpoint
 * using a simplified version of the controller in Lee, Leok & McClamroch
 * (2010) "Geometric tracking control of a quadrotor UAV on SE(3)".
 *
 * Feed-forward acceleration, angular velocity, and inertial compensation
 * are omitted for simplicity.
 */
class Se3Controller {
public:
  struct Gains {
    double K_p[3] = {4.0, 4.0, 6.0};    /**< position       */
    double K_v[3] = {3.0, 3.0, 4.0};    /**< velocity       */
    double K_R[3] = {8.0, 8.0, 4.0};    /**< attitude SO(3) */
    double K_w[3] = {1.0, 1.0, 0.5};    /**< angular rate   */
  };

  Se3Controller() = default;
  explicit Se3Controller(const Gains& gains) : gains_(gains) {}

  /**
   * Load controller gains from a YAML file.
   *
   * Expected format (compatible with ROS 2 parameter YAML):
   *   K_p: [4.0, 4.0, 6.0]
   *   K_v: [3.0, 3.0, 4.0]
   *   K_R: [8.0, 8.0, 4.0]
   *   K_w: [1.0, 1.0, 0.5]
   *
   * @param[in]  path  Absolute or relative path to the YAML file
   * @return Gains parsed from file, or default gains on error
   */
  static Gains LoadGainsFromYAML(const std::string& path);

  /**
   * Compute control command from current state and desired setpoint.
   *
   * @param[in]  state  Current quadrotor state (from shared memory)
   * @param[in]  sp     Desired setpoint (position + yaw)
   * @param[out] ctrl   Computed thrust and body torques, clamped to limits
   */
  void compute(const QuadrotorState& state, const Se3Setpoint& sp,
               QuadrotorControl& ctrl) const;

private:
  Gains gains_;
};
