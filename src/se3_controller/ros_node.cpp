/**
 * ROS 2 backend for the SE(3) controller.
 *
 * Subscribes to /ns/odom (nav_msgs/Odometry) for current state and
 * /ns/se3_reference (geometry_msgs/PoseStamped) for the setpoint.
 * Publishes /ns/cmd (geometry_msgs/Wrench) — the existing ros_adapter
 * bridges this to shared memory for the core simulator.
 *
 * Zero MuJoCo / shm access — pure ROS topic I/O.
 */

#include <chrono>
#include <cmath>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include "se3_controller.h"

using namespace std::chrono_literals;

class Se3ControllerNode : public rclcpp::Node {
public:
  Se3ControllerNode()
    : Node("se3_controller") {
    this->declare_parameter("rate", 500.0);
    this->declare_parameter("gains_file", "");
    double rate = this->get_parameter("rate").as_double();

    std::string gf = this->get_parameter("gains_file").as_string();
    controller_ = gf.empty()
        ? Se3Controller()
        : Se3Controller(Se3Controller::LoadGainsFromYAML(gf));
    if (!gf.empty()) {
      RCLCPP_INFO(this->get_logger(), "Loaded gains from %s", gf.c_str());
    }

    // Subscribers
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", qos,
        std::bind(&Se3ControllerNode::odom_callback, this, std::placeholders::_1));
    ref_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "se3_reference", qos,
        std::bind(&Se3ControllerNode::ref_callback, this, std::placeholders::_1));

    // Publisher
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Wrench>("cmd", qos);

    // Control timer
    auto period = std::chrono::duration<double>(1.0 / rate);
    ctrl_timer_ = this->create_wall_timer(
        period, std::bind(&Se3ControllerNode::control_loop, this));

    RCLCPP_INFO(this->get_logger(), "SE(3) controller ready at %.0f Hz", rate);
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    latest_odom_ = msg;
  }

  void ref_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    latest_ref_ = msg;
  }

  void control_loop() {
    if (!latest_odom_ || !latest_ref_) return;

    // Convert odometry to QuadrotorState
    QuadrotorState state{};
    state.position[0] = latest_odom_->pose.pose.position.x;
    state.position[1] = latest_odom_->pose.pose.position.y;
    state.position[2] = latest_odom_->pose.pose.position.z;

    state.orientation[0] = latest_odom_->pose.pose.orientation.w;
    state.orientation[1] = latest_odom_->pose.pose.orientation.x;
    state.orientation[2] = latest_odom_->pose.pose.orientation.y;
    state.orientation[3] = latest_odom_->pose.pose.orientation.z;

    state.linear_velocity[0] = latest_odom_->twist.twist.linear.x;
    state.linear_velocity[1] = latest_odom_->twist.twist.linear.y;
    state.linear_velocity[2] = latest_odom_->twist.twist.linear.z;

    state.angular_velocity[0] = latest_odom_->twist.twist.angular.x;
    state.angular_velocity[1] = latest_odom_->twist.twist.angular.y;
    state.angular_velocity[2] = latest_odom_->twist.twist.angular.z;

    // Build setpoint from PoseStamped
    Se3Setpoint sp;
    sp.position[0] = latest_ref_->pose.position.x;
    sp.position[1] = latest_ref_->pose.position.y;
    sp.position[2] = latest_ref_->pose.position.z;

    // Extract yaw from quaternion
    double qw = latest_ref_->pose.orientation.w;
    double qx = latest_ref_->pose.orientation.x;
    double qy = latest_ref_->pose.orientation.y;
    double qz = latest_ref_->pose.orientation.z;
    sp.yaw = std::atan2(2.0 * (qw * qz + qx * qy),
                        1.0 - 2.0 * (qy * qy + qz * qz));

    // Compute control
    QuadrotorControl ctrl;
    controller_.compute(state, sp, ctrl);

    // Publish as Wrench
    auto msg = geometry_msgs::msg::Wrench();
    msg.force.z    = ctrl.thrust;
    msg.torque.x   = ctrl.torque[0];
    msg.torque.y   = ctrl.torque[1];
    msg.torque.z   = ctrl.torque[2];
    cmd_pub_->publish(msg);
  }

  Se3Controller controller_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr    odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr ref_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr    cmd_pub_;
  rclcpp::TimerBase::SharedPtr ctrl_timer_;

  nav_msgs::msg::Odometry::SharedPtr    latest_odom_;
  geometry_msgs::msg::PoseStamped::SharedPtr latest_ref_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Se3ControllerNode>());
  rclcpp::shutdown();
  return 0;
}
