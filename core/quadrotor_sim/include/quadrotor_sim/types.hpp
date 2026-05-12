#pragma once

/**
 * Domain types and physical constants for the quadrotor simulator.
 *
 * Zero middleware dependency — pure C++ value types.
 * Coordinate frames:
 *   World:  ENU (X=East, Y=North, Z=Up)
 *   Body:   FLU (X=Front, Y=Left, Z=Up)
 */

namespace quadrotor_sim {

struct State {
  double time = 0.0;
  double position[3] = {0.0};
  double orientation[4] = {1.0, 0.0, 0.0, 0.0};   /**< quaternion w,x,y,z */
  double linear_velocity[3] = {0.0};                 /**< body frame FLU [m/s] */
  double angular_velocity[3] = {0.0};                /**< body frame FLU [rad/s] */
  double linear_acceleration[3] = {0.0};             /**< body frame FLU [m/s²] */
};

struct Control {
  double thrust = 0.0;            /**< body-frame Z force [N]      */
  double torque[3] = {0.0};       /**< body-frame x/y/z moments [Nm] */
};

struct Se3Setpoint {
  double position[3] = {0.0, 0.0, 2.0};   /**< world ENU [m]    */
  double yaw       = 0.0;                 /**< desired yaw [rad] */
};

struct Se3Gains {
  double K_p[3] = {4.0, 4.0, 6.0};    /**< position proportional      */
  double K_v[3] = {3.0, 3.0, 4.0};    /**< velocity proportional      */
  double K_R[3] = {8.0, 8.0, 4.0};    /**< attitude on SO(3)          */
  double K_w[3] = {1.0, 1.0, 0.5};    /**< angular rate damping       */
};

inline constexpr double kGravityZ  = -9.81;   /**< [m/s²]  */
inline constexpr double kTotalMass =  1.0;    /**< [kg]     */
inline constexpr double kThrustMin =  0.0;    /**< [N]      */
inline constexpr double kThrustMax =  42.0;   /**< [N]      */
inline constexpr double kTorqueMin = -0.5;    /**< [Nm]     */
inline constexpr double kTorqueMax =  0.5;    /**< [Nm]     */
inline constexpr double kTimestep  =  0.001;  /**< [s]       */

}  // namespace quadrotor_sim
