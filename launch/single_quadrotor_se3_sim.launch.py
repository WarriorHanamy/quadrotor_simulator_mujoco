"""
Launch the SE(3) controller pipeline: ROS adapter + SE(3) controller.

Start core separately via `uv run sim run` before launching this file.

All nodes run under the same namespace (default 'quadrotor').

Topic layout under /ns:
  /ns/odom          — nav_msgs/Odometry (from ros_adapter)
  /ns/se3_reference — geometry_msgs/PoseStamped (setpoint, user publishes)
  /ns/cmd           — geometry_msgs/Wrench (SE3 controller → ros_adapter → shm)

Usage:
  # Terminal 1: start core simulator
  uv run sim run
  # Terminal 2: launch ROS pipeline
  ros2 launch quadrotor_simulator_mujoco single_quadrotor_se3_sim.launch.py
  # Terminal 3: publish a setpoint
  ros2 topic pub /quadrotor/se3_reference geometry_msgs/msg/PoseStamped \
    '{header: {frame_id: "world"}, pose: {position: {x: 0.0, y: 0.0, z: 2.0}}}'
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    launch_args = [
        DeclareLaunchArgument("name", default_value="quadrotor"),
        DeclareLaunchArgument("world_frame_id", default_value="world"),
        DeclareLaunchArgument("rate_odom", default_value="200.0"),
        DeclareLaunchArgument("rate_imu", default_value="500.0"),
        DeclareLaunchArgument("rate_se3", default_value="500.0"),
    ]

    name = LaunchConfiguration("name")
    world_frame_id = LaunchConfiguration("world_frame_id")
    rate_odom = LaunchConfiguration("rate_odom")
    rate_imu = LaunchConfiguration("rate_imu")
    rate_se3 = LaunchConfiguration("rate_se3")

    gains_file = PathJoinSubstitution(
        [
            get_package_share_directory("quadrotor_simulator_mujoco"),
            "config",
            "se3_gains.yaml",
        ]
    )

    launch_args.append(DeclareLaunchArgument("gains_file", default_value=gains_file))

    # ROS adapter (shm ↔ topics)
    ros_adapter_node = Node(
        package="quadrotor_simulator_mujoco",
        executable="quadrotor_sim_ros_adapter",
        namespace=name,
        output="screen",
        parameters=[
            {
                "rate_odom": rate_odom,
                "rate_imu": rate_imu,
                "world_frame_id": world_frame_id,
                "body_frame_id": name,
            }
        ],
    )

    # SE(3) controller node (odom + se3_reference → cmd)
    se3_node = Node(
        package="quadrotor_simulator_mujoco",
        executable="quadrotor_sim_se3_ros",
        namespace=name,
        output="screen",
        parameters=[
            {
                "rate": rate_se3,
                "gains_file": LaunchConfiguration("gains_file"),
            }
        ],
    )

    debug_print = LogInfo(
        msg=["[INFO] SE(3) gains file: ", LaunchConfiguration("gains_file")]
    )

    ld = LaunchDescription(launch_args)
    ld.add_action(debug_print)
    ld.add_action(
        GroupAction(
            actions=[
                ros_adapter_node,
                se3_node,
            ]
        )
    )
    return ld
