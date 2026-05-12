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
uv run sim render
uv run sim render --model path/to/drone.xml
```

- **uv** manages the Python environment and provides the `sim` CLI entrypoint.
- C++ binaries are built to `build_standalone/` via **xmake**; target definitions live in `xmake.lua`.
- MuJoCo 2.3.2 `.so` is bundled at `deps/lib/libmujoco.so`. No system MuJoCo install needed.
- `LD_LIBRARY_PATH` is auto-set by `cli.py` to include `deps/lib/`.

## Codemap

```
apps/                               # ○ executable entry points (main() functions)
  cli.py                             #   Python CLI `sim build|run|render|run-se3` — builds xmake, spawns subprocesses
  sim_core.cpp                       #   Headless core binary: instantiates SimCore, runs physics loop with SIGINT handler
  sim_render.cpp                     #   GLFW adapter binary: reads shm StateWire, loads MuJoCo model, renders mjvScene
  se3_direct.cpp                     #   Standalone SE(3) controller binary: reads shm StateWire → computes ControlWire via Se3Controller

core/                               # ◆ C++ static libraries
  quadrotor_sim/                     #   Domain types + SE(3) geometric controller (zero middleware deps)
    include/quadrotor_sim/
      types.hpp                       #     State, Control, Se3Setpoint, Se3Gains structs + physical constants
      se3_controller.hpp              #     Se3Controller class: Compute(state, setpoint) → (thrust, torque)
    src/se3_controller.cpp            #     SE(3) controller implementation (Eigen-based, Lee et al. 2010)
  shm/                               #   Shared memory IPC layer (mmap + seqlock)
    include/quadrotor_sim/shm/
      shm_layout.hpp                  #     StateWire (192 B), ControlWire (64 B) packed structs + seqlock ReadConsistent<T>()
      shm_backend.hpp                 #     Inline Create/Open shm functions (mmap wrappers)
    src/shm_backend.cpp               #     WriteBegin/WriteEnd + wire↔model conversion (ToWire/FromWire)
  mujoco/                            #   MuJoCo simulation (links libmujoco.so)
    include/quadrotor_sim/mujoco/
      sim_core.hpp                    #     SimCore class: physics engine with RT sync, control noise, SHM PIMPL
      render.hpp                      #     Renderer: offscreen mjvScene render → RGB pixel buffer
    src/sim_core.cpp                  #     Loads XML model, runs mj_step() loop, ExtractState/ApplyControl via shm
    src/render.cpp                    #     Renderer implementation
  glfw/                              #   GLFW window wrapper
    include/quadrotor_sim/glfw/
      viewer.hpp                      #     Viewer class: GLFW window + texture display
    src/viewer.cpp                    #     Window creation, Present() RGB texture, PollEvents()

python/quadrotor_sim/               # ◇ Python package (ctypes-based SHM protocol bindings)
  __init__.py                        #   Package init (version 0.1.0)
  schema.py                           #   Pydantic QuadrotorState, QuadrotorControl + physical constants
  shm.py                              #   ShmReader, ShmWriter: ctypes struct + mmap + seqlock

examples/ros2_adapter/               # △ ROS 2 adapter (separate ament_cmake project)
  CMakeLists.txt                      #   ROS 2 CMake: links to build_standalone/ static libs
  package.xml                         #   ROS 2 package manifest
  src/ros_adapter_node.cpp            #   ROS node: shm state → /odom, /imu, /clock publishers; /cmd subscriber → shm ctrl
  src/se3_controller_node.cpp         #   ROS node: /odom + /se3_reference subscriber → SE(3) control → /cmd publisher
  launch/single_quadrotor_se3_sim.launch.py  # Launch file (namespace `quadrotor`)

deps/                                # ◼ Vendored dependencies
  include/mujoco/                     #   MuJoCo 2.3.2 C headers (10 files)
  lib/libmujoco.so.2.3.2             #   Bundled MuJoCo shared library
  model/mujoco/drone.xml              #   1 kg quadrotor model: 4 actuators, 6 sensors, RK4 integrator, 0.001 s timestep

config/se3_gains.yaml                 # Default SE(3) gains (K_p, K_v, K_R, K_w)

docker/                               # Docker support for ROS adapter
  Dockerfile                          #   ros:humble-ros-core image + ROS adapter build
  docker-compose.yml                  #   Mounts host /dev/shm into container
  entrypoint.sh                       #   Sources ROS, runs adapter
  CMakeLists_ros_adapter.txt          #   Standalone CMake for colcon build in Docker
```

### C++ target dependency graph (xmake.lua)

```
quadrotor_sim_core  (binary)        quadrotor_sim_glfw_adapter (binary)      quadrotor_sim_se3_direct (binary)
       │                                     │                                       │
       ▼                                     ▼                                       ▼
quadrotor_sim_mujoco (.a)           quadrotor_sim_shm (.a)                quadrotor_sim_shm (.a)
       │                                     │                                       │
       ▼                                     ▼                                       ▼
 quadrotor_sim_shm (.a)             quadrotor_sim (.a)                    quadrotor_sim (.a)
       │
       ▼
 quadrotor_sim (.a)
       │
  ┌────┴────┐
  ▼         ▼
 eigen    yaml-cpp

quadrotor_sim_render (.a)            quadrotor_sim_glfw_viewer (.a)
  (static lib, used by               (static lib, used by
   GLFW adapter binary)                GLFW adapter binary)
```

The `sim build` command invokes `xmake build -w -j`. All output lands in `build_standalone/`.

### Legend

| Symbol | Meaning                                     |
| ------ | ------------------------------------------- |
| `○`    | Executable entry point (`main()`)           |
| `◆`    | C++ static library (`.a`)                   |
| `◇`    | Python package                              |
| `△`    | External adapter (separate build system)    |
| `◼`    | Vendored dependency                         |

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
| 8      | `thrust`       | `double`   | N     | [0.0, 42.0]  | body-frame (FLU) Z thrust      |
| 16     | `torque[0]`    | `double`   | Nm    | [-0.5, 0.5]  | body-frame (FLU) X moment (roll)   |
| 24     | `torque[1]`    | `double`   | Nm    | [-0.5, 0.5]  | body-frame (FLU) Y moment (pitch)  |
| 32     | `torque[2]`    | `double`   | Nm    | [-0.5, 0.5]  | body-frame (FLU) Z moment (yaw)    |
| 40     | `timestamp_ns` | `uint64_t` | ns    | -            | CLOCK_MONOTONIC at write     |

### State output (`QuadrotorState`, 192 B)

| Offset | Field                  | Type       | Unit  | Frame | Description                      |
| ------ | ---------------------- | ---------- | ----- | ----- | -------------------------------- |
| 0      | `sequence`               | `uint64_t`   | -     | -     | seqlock counter                  |
| 8      | `time`                   | `double`     | s     | -     | simulation time                  |
| 16     | `position[0]`            | `double`     | m     | world (ENU) | x                                |
| 24     | `position[1]`            | `double`     | m     | world (ENU) | y                                |
| 32     | `position[2]`            | `double`     | m     | world (ENU) | z                                |
| 40     | `orientation[0]`         | `double`     | -     | world (ENU) | quaternion w                     |
| 48     | `orientation[1]`         | `double`     | -     | world (ENU) | quaternion x                     |
| 56     | `orientation[2]`         | `double`     | -     | world (ENU) | quaternion y                     |
| 64     | `orientation[3]`         | `double`     | -     | world (ENU) | quaternion z                     |
| 72     | `linear_velocity[0]`     | `double`     | m/s   | body (FLU)  | vx                               |
| 80     | `linear_velocity[1]`     | `double`     | m/s   | body (FLU)  | vy                               |
| 88     | `linear_velocity[2]`     | `double`     | m/s   | body (FLU)  | vz                               |
| 96     | `angular_velocity[0]`    | `double`     | rad/s | body (FLU)  | wx                               |
| 104    | `angular_velocity[1]`    | `double`     | rad/s | body (FLU)  | wy                               |
| 112    | `angular_velocity[2]`    | `double`     | rad/s | body (FLU)  | wz                               |
| 120    | `linear_acceleration[0]` | `double`     | m/s²  | body (FLU)  | ax                               |
| 128    | `linear_acceleration[1]` | `double`     | m/s²  | body (FLU)  | ay                               |
| 136    | `linear_acceleration[2]` | `double`     | m/s²  | body (FLU)  | az                               |
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
- **Coordinate frames**: World is ENU (X=East, Y=North, Z=Up); body is FLU (X=Front, Y=Left, Z=Up).

### Adapter protocol

An adapter is any process that reads `/dev/shm/quadrotor_sim/state` and optionally writes `/dev/shm/quadrotor_sim/ctrl`. It:

1. Attaches to existing segments (does NOT create them — core owns creation)
2. Reads `state` via seqlock loop → converts to its output format
3. Writes `ctrl` via seqlock → core applies it on the next `mj_step`
4. Must NOT touch MuJoCo internal pointers (`mjModel*`, `mjData*`)

### Schema sources

| Language | File                                    | Role                           |
| -------- | --------------------------------------- | ------------------------------ |
| C++      | `core/shm/include/quadrotor_sim/shm/shm_layout.hpp` | ABI-stable structs + seqlock   |
| C++      | `core/quadrotor_sim/include/quadrotor_sim/types.hpp` | Domain structs + constants     |
| Python   | `python/quadrotor_sim/schema.py`        | Pydantic validation models     |
| Python   | `python/quadrotor_sim/shm.py`           | mmap+seqlock reader/writer     |
| —        | `AGENTS.md` (this section)              | Abstract reference contract    |

## No tests / no CI

- There are no unit tests or CI workflows. The CMake `BUILD_TESTING` block exists but only declares linter packages; cpplint and copyright lint are explicitly skipped.
- No `.gitignore`, no `.clang-format`, no pre-commit hooks.
