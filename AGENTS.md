# AGENTS.md — quadrotor_simulator_mujoco

## Build & run

```bash
# Dependencies (system)
sudo apt install libglfw3-dev

# Build (from ROS 2 workspace root, using colcon)
colcon build --symlink-install
source install/setup.bash

# Run
ros2 launch quadrotor_simulator_mujoco single_quadrotor_sim.launch.py
```

- **ROS 2 / ament_cmake package** — not plain CMake. Always use `colcon build` from a ROS 2 workspace, not `cmake --build .` directly.
- MuJoCo 2.3.2 `.so` is bundled at `lib/libmujoco.so`. No system MuJoCo install needed.
- `CMakeLists.txt` uses `GLOB_RECURSE` for sources. New `.cc`/`.cpp` files in `src/` auto-include on the next `colcon` cmake reconfigure; CMake will not notice them on incremental builds alone.

## Architecture

- **Single executable** `quadrotor_simulator` built from all `.cc`/`.cpp` under `src/`.
- **Two threads**: physics thread (`PhysicsLoop` in `src/main.cc`) runs MuJoCo stepping; main thread runs GLFW render loop (`sim->renderloop()`). Access to `sim->d` is protected by `sim->mtx`.
- **ROS 2 node**: `MuJoCoMessageHandler` (its own `rclcpp::Node`, spun in a third thread). It publishes odom, IMU, images, and `/clock`; subscribes to `cmd` (`geometry_msgs::msg::Wrench`).

## Topic remapping under namespace

The launch file sets a node `namespace` (default `quadrotor`), so topic paths become:
- `/<ns>/cmd` — control input (`Wrench`: `force.z` → thrust, `torque.x/y/z` → body torques)
- `/<ns>/odom` — odometry
- `/<ns>/imu` — IMU
- `/<ns>/rgb_image` — RGB camera (flipped vertically before publishing, encoding `rgb8`)

## Image pipeline gotcha

`publish_image_from_render` in `MuJoCoMessageHandler.cpp` reads OpenGL pixels with `mjr_readPixels`, flips them vertically row-by-row, and publishes as `rgb8`. Skipping the flip or changing encoding will break downstream consumers.

## Model path

- The launch file resolves the MuJoCo XML model at `share/quadrotor_simulator_mujoco/model/mujoco/drone.xml` and passes it as the first CLI argument to the executable.
- The model defines 4 actuators (`body_thrust`, `x_moment`, `y_moment`, `z_moment`) in that order → mapped to `d->ctrl[0..3]` in `apply_ctrl()`.

## No tests / no CI

- There are no unit tests or CI workflows. The CMake `BUILD_TESTING` block exists but only declares linter packages; cpplint and copyright lint are explicitly skipped.
- No `.gitignore`, no `.clang-format`, no pre-commit hooks.
