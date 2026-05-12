#pragma once

#include <string>

#include "quadrotor_sim/types.hpp"

namespace quadrotor_sim {

/**
 * Geometric SE(3) quadrotor controller.
 *
 * Computes body-frame thrust and torques from a position + yaw setpoint
 * using a simplified version of the controller in Lee, Leok & McClamroch
 * (2010) "Geometric tracking control of a quadrotor UAV on SE(3)".
 */
class Se3Controller {
public:
  Se3Controller() = default;
  explicit Se3Controller(const Se3Gains& gains) : gains_(gains) {}

  /** Load gains from a YAML file. Returns defaults on error. */
  static Se3Gains LoadGainsFromYAML(const std::string& path);

  /**
   * Compute control command from current state and desired setpoint.
   * @param[in]  state  Current quadrotor state
   * @param[in]  sp     Desired setpoint (position + yaw)
   * @param[out] ctrl   Computed thrust and body torques, clamped to limits
   */
  void Compute(const State& state, const Se3Setpoint& sp,
               Control& ctrl) const;

private:
  Se3Gains gains_;
};

}  // namespace quadrotor_sim
