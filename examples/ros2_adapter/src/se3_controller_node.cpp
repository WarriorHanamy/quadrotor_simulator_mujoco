/**
 * ROS 2 SE(3) controller node.
 *
 * Subscribes /ns/odom + /ns/se3_reference, publishes /ns/cmd (Wrench).
 * Zero shared-memory dependency — pure ROS topic I/O.
 */

#include <chrono>
#include <cmath>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include "quadrotor_sim/se3_controller.hpp"

using namespace std::chrono_literals;

class Se3ControllerNode : public rclcpp::Node {
public:
  Se3ControllerNode() : Node("se3_controller") {
    this->declare_parameter("rate", 500.0);
    this->declare_parameter("gains_file", "");
    double rate = this->get_parameter("rate").as_double();

    std::string gf = this->get_parameter("gains_file").as_string();
    controller_ = gf.empty()
        ? quadrotor_sim::Se3Controller()
        : quadrotor_sim::Se3Controller(
            quadrotor_sim::Se3Controller::LoadGainsFromYAML(gf));
    if (!gf.empty())
      RCLCPP_INFO(this->get_logger(), "Loaded gains from %s", gf.c_str());

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", qos,
        std::bind(&Se3ControllerNode::odom_callback, this, std::placeholders::_1));
    ref_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "se3_reference", qos,
        std::bind(&Se3ControllerNode::ref_callback, this, std::placeholders::_1));
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Wrench>("cmd", qos);

    ctrl_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / rate),
        std::bind(&Se3ControllerNode::control_loop, this));

    RCLCPP_INFO(this->get_logger(), "SE(3) controller ready at %.0f Hz", rate);
  }

private:
  quadrotor_sim::Se3Controller controller_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr        odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr ref_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr         cmd_pub_;
  rclcpp::TimerBase::SharedPtr ctrl_timer_;

  nav_msgs::msg::Odometry::SharedPtr        latest_odom_;
  geometry_msgs::msg::PoseStamped::SharedPtr latest_ref_;

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) { latest_odom_ = msg; }
  void ref_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) { latest_ref_ = msg; }

  void control_loop() {
    if (!latest_odom_ || !latest_ref_) return;

    quadrotor_sim::State st{};
    st.position[0] = latest_odom_->pose.pose.position.x;
    st.position[1] = latest_odom_->pose.pose.position.y;
    st.position[2] = latest_odom_->pose.pose.position.z;
    st.orientation[0] = latest_odom_->pose.pose.orientation.w;
    st.orientation[1] = latest_odom_->pose.pose.orientation.x;
    st.orientation[2] = latest_odom_->pose.pose.orientation.y;
    st.orientation[3] = latest_odom_->pose.pose.orientation.z;
    st.linear_velocity[0]  = latest_odom_->twist.twist.linear.x;
    st.linear_velocity[1]  = latest_odom_->twist.twist.linear.y;
    st.linear_velocity[2]  = latest_odom_->twist.twist.linear.z;
    st.angular_velocity[0] = latest_odom_->twist.twist.angular.x;
    st.angular_velocity[1] = latest_odom_->twist.twist.angular.y;
    st.angular_velocity[2] = latest_odom_->twist.twist.angular.z;

    quadrotor_sim::Se3Setpoint sp;
    sp.position[0] = latest_ref_->pose.position.x;
    sp.position[1] = latest_ref_->pose.position.y;
    sp.position[2] = latest_ref_->pose.position.z;

    double qw = latest_ref_->pose.orientation.w;
    double qx = latest_ref_->pose.orientation.x;
    double qy = latest_ref_->pose.orientation.y;
    double qz = latest_ref_->pose.orientation.z;
    sp.yaw = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));

    quadrotor_sim::Control ctrl;
    controller_.Compute(st, sp, ctrl);

    auto msg = geometry_msgs::msg::Wrench();
    msg.force.z    = ctrl.thrust;
    msg.torque.x   = ctrl.torque[0];
    msg.torque.y   = ctrl.torque[1];
    msg.torque.z   = ctrl.torque[2];
    cmd_pub_->publish(msg);
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Se3ControllerNode>());
  rclcpp::shutdown();
  return 0;
}
