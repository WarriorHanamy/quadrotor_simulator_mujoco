/**
 * ROS 2 adapter for the quadrotor simulator shared-memory protocol.
 *
 * Reads state from /dev/shm/quadrotor_sim/state
 * Writes control to /dev/shm/quadrotor_sim/ctrl
 *
 * Publishes:  /{ns}/odom, /{ns}/imu, /clock
 * Subscribes: /{ns}/cmd (geometry_msgs/Wrench)
 */

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

#include "quadrotor_sim/shm/shm_layout.hpp"
#include "quadrotor_sim/shm/shm_backend.hpp"

using namespace std::chrono_literals;
using namespace quadrotor_sim::shm;

class RosAdapter : public rclcpp::Node {
public:
  RosAdapter() : Node("quadrotor_sim_ros_adapter") {
    this->declare_parameter("rate_odom", 200.0);
    this->declare_parameter("rate_imu", 500.0);
    this->declare_parameter("world_frame_id", "world");
    this->declare_parameter("body_frame_id", "quadrotor");

    double rate_odom = this->get_parameter("rate_odom").as_double();
    double rate_imu  = this->get_parameter("rate_imu").as_double();
    world_frame_id_  = this->get_parameter("world_frame_id").as_string();
    body_frame_id_   = this->get_parameter("body_frame_id").as_string();

    shm_state_ = OpenStateShm();
    shm_ctrl_  = OpenCtrlShm(/*write=*/true);

    if (!shm_state_ || !shm_ctrl_) {
      RCLCPP_ERROR(this->get_logger(), "Cannot attach to shm. Is sim_core running?");
      ready_ = false;
      return;
    }

    auto qos = rclcpp::QoS(10);
    odom_pub_  = this->create_publisher<nav_msgs::msg::Odometry>("odom", qos);
    imu_pub_   = this->create_publisher<sensor_msgs::msg::Imu>("imu", qos);
    clock_pub_ = this->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 100);

    auto cmd_qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);
    cmd_sub_ = this->create_subscription<geometry_msgs::msg::Wrench>(
        "cmd", cmd_qos,
        std::bind(&RosAdapter::cmd_callback, this, std::placeholders::_1));

    odom_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / rate_odom),
        std::bind(&RosAdapter::odom_publish, this));
    imu_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / rate_imu),
        std::bind(&RosAdapter::imu_publish, this));
    clock_timer_ = this->create_wall_timer(1ms, std::bind(&RosAdapter::clock_publish, this));

    RCLCPP_INFO(this->get_logger(), "ROS adapter ready");
  }

  ~RosAdapter() {
    munmap(shm_state_, sizeof(StateWire));
    munmap(shm_ctrl_, sizeof(ControlWire));
  }

  bool is_ready() const { return ready_; }

private:
  StateWire*  shm_state_ = nullptr;
  ControlWire* shm_ctrl_  = nullptr;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Wrench>::SharedPtr cmd_sub_;
  rclcpp::TimerBase::SharedPtr odom_timer_, imu_timer_, clock_timer_;

  std::string world_frame_id_, body_frame_id_;
  bool ready_ = true;

  void cmd_callback(const geometry_msgs::msg::Wrench::SharedPtr msg) {
    quadrotor_sim::Control c;
    c.thrust    = msg->force.z;
    c.torque[0] = msg->torque.x;
    c.torque[1] = msg->torque.y;
    c.torque[2] = msg->torque.z;

    ControlWire cw;
    ToWire(c, cw);
    WriteBegin(shm_ctrl_->sequence);
    *shm_ctrl_ = cw;
    WriteEnd(shm_ctrl_->sequence);
  }

  void odom_publish() {
    StateWire sw;
    if (!ReadConsistent(*shm_state_, sw)) return;

    auto msg = nav_msgs::msg::Odometry();
    msg.header.stamp = this->now();
    msg.header.frame_id = world_frame_id_;
    msg.child_frame_id  = body_frame_id_;

    msg.pose.pose.position.x = sw.position[0];
    msg.pose.pose.position.y = sw.position[1];
    msg.pose.pose.position.z = sw.position[2];
    msg.pose.pose.orientation.w = sw.orientation[0];
    msg.pose.pose.orientation.x = sw.orientation[1];
    msg.pose.pose.orientation.y = sw.orientation[2];
    msg.pose.pose.orientation.z = sw.orientation[3];

    msg.twist.twist.linear.x  = sw.linear_velocity[0];
    msg.twist.twist.linear.y  = sw.linear_velocity[1];
    msg.twist.twist.linear.z  = sw.linear_velocity[2];
    msg.twist.twist.angular.x = sw.angular_velocity[0];
    msg.twist.twist.angular.y = sw.angular_velocity[1];
    msg.twist.twist.angular.z = sw.angular_velocity[2];

    odom_pub_->publish(msg);
  }

  void imu_publish() {
    StateWire sw;
    if (!ReadConsistent(*shm_state_, sw)) return;

    auto msg = sensor_msgs::msg::Imu();
    msg.header.stamp = this->now();
    msg.header.frame_id = body_frame_id_;

    msg.linear_acceleration.x = sw.linear_acceleration[0];
    msg.linear_acceleration.y = sw.linear_acceleration[1];
    msg.linear_acceleration.z = sw.linear_acceleration[2] - 9.81;

    msg.angular_velocity.x = sw.angular_velocity[0];
    msg.angular_velocity.y = sw.angular_velocity[1];
    msg.angular_velocity.z = sw.angular_velocity[2];

    msg.orientation.w = sw.orientation[0];
    msg.orientation.x = sw.orientation[1];
    msg.orientation.y = sw.orientation[2];
    msg.orientation.z = sw.orientation[3];

    imu_pub_->publish(msg);
  }

  void clock_publish() {
    StateWire sw;
    if (!ReadConsistent(*shm_state_, sw)) return;
    auto msg = rosgraph_msgs::msg::Clock();
    msg.clock = rclcpp::Time(static_cast<int64_t>(sw.time * 1e9));
    clock_pub_->publish(msg);
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RosAdapter>();
  if (!node->is_ready()) {
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
