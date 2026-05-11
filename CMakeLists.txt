cmake_minimum_required(VERSION 3.8)
project(quadrotor_simulator_mujoco)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

list(APPEND ${PROJECT_NAME}_CONFIG_EXTRAS
        cmake/CheckAvxSupport.cmake
        cmake/FindOrFetch.cmake
        cmake/MujocoHarden.cmake
        cmake/MujocoLinkOptions.cmake
        cmake/MujocoMacOS.cmake
        cmake/SampleDependencies.cmake
        cmake/SampleOptions.cmake
        )

find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)

# Mujoco location
if(NOT TARGET mujoco)
  add_library(mujoco SHARED IMPORTED)
  set_target_properties(mujoco PROPERTIES IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/lib/libmujoco.so)
endif()

# ---------------------------------------------------------------------------
# Target 1: quadrotor_simulator (original, backward-compatible)
# ---------------------------------------------------------------------------
file(GLOB_RECURSE source_orig ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cc)
# Exclude new subdirectory sources from the original target
list(FILTER source_orig EXCLUDE REGEX "src/(core|ros_adapter|glfw_adapter)/")

add_executable(quadrotor_simulator ${source_orig})
target_link_libraries(quadrotor_simulator mujoco glfw pthread rt ${CMAKE_DL_LIBS})
target_include_directories(quadrotor_simulator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_options(quadrotor_simulator PUBLIC ${DEEPBREAK_CXX_FLAGS})
target_compile_features(quadrotor_simulator PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  quadrotor_simulator
  "ament_cmake"
  "sensor_msgs"
  "rclcpp"
  "geometry_msgs"
  "tf2"
  "tf2_ros"
  "nav_msgs"
)

# ---------------------------------------------------------------------------
# Target 2: quadrotor_sim_core (headless physics, zero ROS dependency)
# ---------------------------------------------------------------------------
file(GLOB source_core ${CMAKE_CURRENT_SOURCE_DIR}/src/core/*.cc)
add_executable(quadrotor_sim_core ${source_core})
target_link_libraries(quadrotor_sim_core mujoco pthread rt ${CMAKE_DL_LIBS})
target_include_directories(quadrotor_sim_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/core>
  $<INSTALL_INTERFACE:include>)
target_compile_features(quadrotor_sim_core PUBLIC cxx_std_17)

# ---------------------------------------------------------------------------
# Target 3: quadrotor_sim_ros_adapter (ROS 2 node, zero MuJoCo dependency)
# ---------------------------------------------------------------------------
file(GLOB source_ros_adapter ${CMAKE_CURRENT_SOURCE_DIR}/src/ros_adapter/*.cpp)
add_executable(quadrotor_sim_ros_adapter ${source_ros_adapter})
target_include_directories(quadrotor_sim_ros_adapter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/core>
  $<INSTALL_INTERFACE:include>)
target_compile_features(quadrotor_sim_ros_adapter PUBLIC cxx_std_17)
ament_target_dependencies(
  quadrotor_sim_ros_adapter
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  rosgraph_msgs
)

# ---------------------------------------------------------------------------
# Target 4: quadrotor_sim_glfw_adapter (render viewer)
# ---------------------------------------------------------------------------
file(GLOB source_glfw_adapter ${CMAKE_CURRENT_SOURCE_DIR}/src/glfw_adapter/*.cc)
add_executable(quadrotor_sim_glfw_adapter ${source_glfw_adapter})
target_include_directories(quadrotor_sim_glfw_adapter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/core>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(quadrotor_sim_glfw_adapter mujoco glfw pthread rt ${CMAKE_DL_LIBS})
target_compile_features(quadrotor_sim_glfw_adapter PUBLIC cxx_std_17)

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------
install(TARGETS
  quadrotor_simulator
  quadrotor_sim_core
  quadrotor_sim_ros_adapter
  quadrotor_sim_glfw_adapter
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
         DESTINATION share/${PROJECT_NAME})

install(DIRECTORY model
        DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
