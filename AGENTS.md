# AGENTS.md — quadrotor_simulator_mujoco

## Build & run

```bash
# Install Python deps (pydantic) and register the CLI
uv sync

# Build C++ binaries (core + GLFW adapter)
uv run sim build

# Run headless core simulator
uv run sim run
uv run sim run --model path/to/drone.xml --real-time-factor 0.5

# Run GLFW render viewer (requires display)
uv run sim glfw
uv run sim glfw --model path/to/drone.xml
```

- **uv** manages the Python environment and provides the `sim` CLI entrypoint.
- C++ binaries are built to `build_standalone/` via CMake (`CMakeLists_standalone.txt`).
- MuJoCo 2.3.2 `.so` is bundled at `deps/lib/libmujoco.so`. No system MuJoCo install needed.
- `LD_LIBRARY_PATH` is auto-set by `cli.py` to include `deps/lib/`.

## Architecture

- **Core simulator** `quadrotor_sim_core` (`src/core/`) — headless physics loop, writes `QuadrotorState` to `/dev/shm/quadrotor_sim/state`, reads `QuadrotorControl` from `/dev/shm/quadrotor_sim/ctrl`.
- **ROS 2 adapter** `quadrotor_sim_ros_adapter` (`src/ros_adapter/`) — independent `rclcpp::Node`, reads shm state → publishes odom/imu/clock, subscribes cmd → writes shm ctrl.
- **GLFW adapter** `quadrotor_sim_glfw_adapter` (`src/glfw_adapter/`) — reads shm state → renders MuJoCo scene.
- **Legacy** `quadrotor_simulator` (`src/_legacy/`) — original monolithic binary (GLFW + ROS in-process), kept for backward compatibility.
- **Schema** — `include/sim_schema.h` (C++), `python/quadrotor_sim/schema.py` (Pydantic), `python/quadrotor_sim/shm.py` (mmap I/O).
- **Model** — `deps/model/mujoco/drone.xml` defines 4 actuators (`body_thrust`, `x_moment`, `y_moment`, `z_moment`) → mapped to `d->ctrl[0..3]`.

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

## Simulator Interface Schema

The core simulator communicates with all adapters through shared memory under `/dev/shm/quadrotor_sim/`.

```
 HOST (Arch Linux)                         DOCKER Container
 ┌──────────────────────┐                  ┌──────────────────────┐
 │ quadrotor_sim_core    │  /dev/shm/       │ ROS 2 Adapter        │
 │ (headless MuJoCo)     │  quadrotor_sim/  │                      │
 │                        │                  │                      │
 │  mj_step() ──写──>─┼── state segment ──>── 读→ pub odom/imu/  │
 │                        │                  │           clock      │
 │  apply_ctrl() <──读──┼── ctrl segment ──<── 写 ← sub cmd(Wrench)│
 │                        │                  │                      │
 └──────────────────────┘                  └──────────────────────┘
```

### Shared memory layout

| Segment  | File path                           | Size            | Writer         | Readers        |
| -------- | ----------------------------------- | --------------- | -------------- | -------------- |
| `state`    | `/dev/shm/quadrotor_sim/state`      | 192 B           | core           | all adapters   |
| `ctrl`     | `/dev/shm/quadrotor_sim/ctrl`       | 64 B            | any controller | core           |
| `image`    | `/dev/shm/quadrotor_sim/image`      | 64 B + ~2.8 MB  | core (on-demand)| adapters      |

Synchronization: **seqlock** (monotonic `sequence` counter + memory barriers).

### Control input (`QuadrotorControl`, 64 B)

| Offset | Field        | Type     | Unit  | Range        | Description                  |
| ------ | ------------ | -------- | ----- | ------------ | ---------------------------- |
| 0      | `sequence`     | `uint64_t` | -     | monotonic    | seqlock counter              |
| 8      | `thrust`       | `double`   | N     | [0.0, 42.0]  | body-frame Z thrust          |
| 16     | `torque[0]`    | `double`   | Nm    | [-0.5, 0.5]  | body-frame X moment (roll)   |
| 24     | `torque[1]`    | `double`   | Nm    | [-0.5, 0.5]  | body-frame Y moment (pitch)  |
| 32     | `torque[2]`    | `double`   | Nm    | [-0.5, 0.5]  | body-frame Z moment (yaw)    |
| 40     | `timestamp_ns` | `uint64_t` | ns    | -            | CLOCK_MONOTONIC at write     |

### State output (`QuadrotorState`, 192 B)

| Offset | Field                  | Type       | Unit  | Frame | Description                      |
| ------ | ---------------------- | ---------- | ----- | ----- | -------------------------------- |
| 0      | `sequence`               | `uint64_t`   | -     | -     | seqlock counter                  |
| 8      | `time`                   | `double`     | s     | -     | simulation time                  |
| 16     | `position[0]`            | `double`     | m     | world | x                                |
| 24     | `position[1]`            | `double`     | m     | world | y                                |
| 32     | `position[2]`            | `double`     | m     | world | z                                |
| 40     | `orientation[0]`         | `double`     | -     | world | quaternion w                     |
| 48     | `orientation[1]`         | `double`     | -     | world | quaternion x                     |
| 56     | `orientation[2]`         | `double`     | -     | world | quaternion y                     |
| 64     | `orientation[3]`         | `double`     | -     | world | quaternion z                     |
| 72     | `linear_velocity[0]`     | `double`     | m/s   | body  | vx                               |
| 80     | `linear_velocity[1]`     | `double`     | m/s   | body  | vy                               |
| 88     | `linear_velocity[2]`     | `double`     | m/s   | body  | vz                               |
| 96     | `angular_velocity[0]`    | `double`     | rad/s | body  | wx                               |
| 104    | `angular_velocity[1]`    | `double`     | rad/s | body  | wy                               |
| 112    | `angular_velocity[2]`    | `double`     | rad/s | body  | wz                               |
| 120    | `linear_acceleration[0]` | `double`     | m/s²  | body  | ax                               |
| 128    | `linear_acceleration[1]` | `double`     | m/s²  | body  | ay                               |
| 136    | `linear_acceleration[2]` | `double`     | m/s²  | body  | az                               |
| 144    | `timestamp_ns`           | `uint64_t`   | ns    | -     | CLOCK_MONOTONIC at write         |

### Image output (`ImageData`, 64 B header + pixel data)

| Offset | Field        | Type       | Description                    |
| ------ | ------------ | ---------- | ------------------------------ |
| 0      | `sequence`     | `uint64_t`   | seqlock counter                |
| 8      | `width`        | `uint32_t`   | image width [px]               |
| 12     | `height`       | `uint32_t`   | image height [px]              |
| 16     | `encoding`     | `uint32_t`   | FourCC `0x72363862` = `'rgb8'` |
| 20     | `timestamp_ns` | `uint64_t`   | CLOCK_MONOTONIC at write       |
| 64     | `data[]`       | `uint8_t[]`  | pixel data, row-major, W×H×3 B |

### Semantics

- **Ranges**: Actuator limits enforced by `drone.xml` `ctrlrange`. The schema repeats them for validation.
- **Gravity**: Always `[0, 0, -9.81]` in world frame. IMU accelerometer reads include gravity; the ROS adapter subtracts `9.81` on Z (legacy convention).
- **IMU orientation**: The `orient` field is the world-frame quaternion. The ROS adapter reads frame-quaternion from MuJoCo sensors for the IMU message.
- **Scalar type**: `double` for Phy/Sim accuracy, `float` ranges are sufficient for all physical quantities.

### Adapter protocol

An adapter is any process that reads `/dev/shm/quadrotor_sim/state` and optionally writes `/dev/shm/quadrotor_sim/ctrl`. It:

1. Attaches to existing segments (does NOT create them — core owns creation)
2. Reads `state` via seqlock loop → converts to its output format
3. Writes `ctrl` via seqlock → core applies it on the next `mj_step`
4. Must NOT touch MuJoCo internal pointers (`mjModel*`, `mjData*`)

### Schema sources

| Language | File                                      | Role                           |
| -------- | ----------------------------------------- | ------------------------------ |
| C++      | `include/sim_schema.h`                    | ABI-stable structs + seqlock   |
| Python   | `python/quadrotor_sim/schema.py`          | Pydantic validation models     |
| Python   | `python/quadrotor_sim/shm.py`             | mmap+seqlock reader/writer     |
| —        | `AGENTS.md` (this section)                | Abstract reference contract    |

## No tests / no CI

- There are no unit tests or CI workflows. The CMake `BUILD_TESTING` block exists but only declares linter packages; cpplint and copyright lint are explicitly skipped.
- No `.gitignore`, no `.clang-format`, no pre-commit hooks.
