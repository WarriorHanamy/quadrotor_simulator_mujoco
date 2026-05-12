#include "se3_controller.h"

#include <algorithm>
#include <cstdio>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <yaml-cpp/yaml.h>

Se3Controller::Gains Se3Controller::LoadGainsFromYAML(const std::string& path) {
  Gains g;
  try {
    YAML::Node doc = YAML::LoadFile(path);
    auto load_arr = [&](const char* key, double* out) {
      if (doc[key] && doc[key].IsSequence() && doc[key].size() == 3) {
        for (int i = 0; i < 3; i++) out[i] = doc[key][i].as<double>();
      }
    };
    load_arr("K_p", g.K_p);
    load_arr("K_v", g.K_v);
    load_arr("K_R", g.K_R);
    load_arr("K_w", g.K_w);
  } catch (const YAML::Exception& e) {
    std::fprintf(stderr, "se3_controller: YAML error in %s: %s — using defaults\n",
                 path.c_str(), e.what());
  }
  return g;
}

void Se3Controller::compute(const QuadrotorState& state,
                            const Se3Setpoint& sp,
                            QuadrotorControl& ctrl) const {
  // Current orientation as Eigen quaternion
  Eigen::Quaterniond q(state.orientation[0], state.orientation[1],
                       state.orientation[2], state.orientation[3]);
  Eigen::Matrix3d R = q.toRotationMatrix();

  // Position error in world frame
  Eigen::Vector3d e_p(state.position[0] - sp.position[0],
                      state.position[1] - sp.position[1],
                      state.position[2] - sp.position[2]);

  // Velocity in world frame (state gives body-frame velocity)
  Eigen::Vector3d v_world = R * Eigen::Vector3d(state.linear_velocity);

  // Desired force in world frame (gravity compensation + position/velocity feedback)
  Eigen::Vector3d F_des(
    -gains_.K_p[0] * e_p[0] - gains_.K_v[0] * v_world[0],
    -gains_.K_p[1] * e_p[1] - gains_.K_v[1] * v_world[1],
    -gains_.K_p[2] * e_p[2] - gains_.K_v[2] * v_world[2]
        - SIM_TOTAL_MASS * SIM_GRAVITY_Z
  );

  // Thrust = projection of desired force onto body Z axis (third column of R)
  double thrust_raw = F_des.dot(R.col(2));
  ctrl.thrust = std::clamp(thrust_raw, SIM_THRUST_MIN, SIM_THRUST_MAX);

  // Desired body Z axis in world frame
  Eigen::Vector3d z_b = F_des.normalized();

  // Desired rotation from Z axis and yaw
  Eigen::Vector3d x_c(std::cos(sp.yaw), std::sin(sp.yaw), 0.0);
  Eigen::Vector3d y_b = z_b.cross(x_c).normalized();
  Eigen::Vector3d x_b = y_b.cross(z_b);

  Eigen::Matrix3d R_des;
  R_des.col(0) = x_b;
  R_des.col(1) = y_b;
  R_des.col(2) = z_b;

  // Attitude error on SO(3): e_R = 0.5 * vee(R_des^T R - R^T R_des)
  Eigen::Matrix3d R_err = R_des.transpose() * R;
  Eigen::Matrix3d D = R_err - R_err.transpose();
  Eigen::Vector3d e_R(0.5 * D(2, 1), 0.5 * D(0, 2), 0.5 * D(1, 0));

  // Angular velocity error (zero desired angular velocity)
  const double* omega = state.angular_velocity;

  // Body-frame torques
  ctrl.torque[0] = std::clamp(-gains_.K_R[0] * e_R[0] - gains_.K_w[0] * omega[0],
                               SIM_TORQUE_MIN, SIM_TORQUE_MAX);
  ctrl.torque[1] = std::clamp(-gains_.K_R[1] * e_R[1] - gains_.K_w[1] * omega[1],
                               SIM_TORQUE_MIN, SIM_TORQUE_MAX);
  ctrl.torque[2] = std::clamp(-gains_.K_R[2] * e_R[2] - gains_.K_w[2] * omega[2],
                               SIM_TORQUE_MIN, SIM_TORQUE_MAX);
}
