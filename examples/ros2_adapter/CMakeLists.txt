cmake_minimum_required(VERSION 3.8)
project(quadrotor_simulator_mujoco_ros2_adapter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)

# Path to core library (adjust to your build tree)
set(CORE_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../..")
set(CORE_BUILD "${CORE_ROOT}/build_standalone")

# ---------------------------------------------------------------------------
# ROS adapter (shm ↔ ROS topics)
# ---------------------------------------------------------------------------
add_executable(quadrotor_sim_ros_adapter src/ros_adapter_node.cpp)
target_include_directories(quadrotor_sim_ros_adapter PRIVATE
  ${CORE_ROOT}/core/quadrotor_sim/include
  ${CORE_ROOT}/core/shm/include
)
target_compile_features(quadrotor_sim_ros_adapter PRIVATE cxx_std_17)
ament_target_dependencies(quadrotor_sim_ros_adapter
  rclcpp sensor_msgs geometry_msgs nav_msgs rosgraph_msgs)
target_link_libraries(quadrotor_sim_ros_adapter
  ${CORE_BUILD}/libquadrotor_sim_shm.a
)

# ---------------------------------------------------------------------------
# SE(3) controller ROS node
# ---------------------------------------------------------------------------
add_executable(quadrotor_sim_se3_ros src/se3_controller_node.cpp)
target_include_directories(quadrotor_sim_se3_ros PRIVATE
  ${CORE_ROOT}/core/quadrotor_sim/include
  ${CORE_ROOT}/core/shim/include
)
target_compile_features(quadrotor_sim_se3_ros PRIVATE cxx_std_17)
ament_target_dependencies(quadrotor_sim_se3_ros
  rclcpp geometry_msgs nav_msgs)
target_link_libraries(quadrotor_sim_se3_ros
  ${CORE_BUILD}/libquadrotor_sim.a
  yaml-cpp::yaml-cpp
  Eigen3::Eigen
)

install(TARGETS
  quadrotor_sim_ros_adapter
  quadrotor_sim_se3_ros
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
