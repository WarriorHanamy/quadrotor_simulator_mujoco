#include "quadrotor_sim/se3_controller.hpp"

#include <algorithm>
#include <cstdio>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <yaml-cpp/yaml.h>

namespace quadrotor_sim {

Se3Gains Se3Controller::LoadGainsFromYAML(const std::string& path) {
  Se3Gains g;
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

void Se3Controller::Compute(const State& state, const Se3Setpoint& sp,
                            Control& ctrl) const {
  using namespace Eigen;

  Quaterniond q(state.orientation[0], state.orientation[1],
                state.orientation[2], state.orientation[3]);
  Matrix3d R = q.toRotationMatrix();

  Vector3d e_p(state.position[0] - sp.position[0],
               state.position[1] - sp.position[1],
               state.position[2] - sp.position[2]);

  Vector3d v_world = R * Vector3d(state.linear_velocity);

  Vector3d F_des(
    -gains_.K_p[0] * e_p[0] - gains_.K_v[0] * v_world[0],
    -gains_.K_p[1] * e_p[1] - gains_.K_v[1] * v_world[1],
    -gains_.K_p[2] * e_p[2] - gains_.K_v[2] * v_world[2]
        - kTotalMass * kGravityZ
  );

  double thrust_raw = F_des.dot(R.col(2));
  ctrl.thrust = std::clamp(thrust_raw, kThrustMin, kThrustMax);

  Vector3d z_b = F_des.normalized();
  Vector3d x_c(std::cos(sp.yaw), std::sin(sp.yaw), 0.0);
  Vector3d y_b = z_b.cross(x_c).normalized();
  Vector3d x_b = y_b.cross(z_b);

  Matrix3d R_des;
  R_des.col(0) = x_b;
  R_des.col(1) = y_b;
  R_des.col(2) = z_b;

  Matrix3d R_err = R_des.transpose() * R;
  Matrix3d D = R_err - R_err.transpose();
  Vector3d e_R(0.5 * D(2, 1), 0.5 * D(0, 2), 0.5 * D(1, 0));

  const double* omega = state.angular_velocity;
  ctrl.torque[0] = std::clamp(-gains_.K_R[0] * e_R[0] - gains_.K_w[0] * omega[0],
                               kTorqueMin, kTorqueMax);
  ctrl.torque[1] = std::clamp(-gains_.K_R[1] * e_R[1] - gains_.K_w[1] * omega[1],
                               kTorqueMin, kTorqueMax);
  ctrl.torque[2] = std::clamp(-gains_.K_R[2] * e_R[2] - gains_.K_w[2] * omega[2],
                               kTorqueMin, kTorqueMax);
}

}  // namespace quadrotor_sim
