# Refactor: Decouple Core Simulator from ROS via Shared Memory Protocol

## Intent

将 MuJoCo 四旋翼仿真器的核心物理引擎与 ROS 耦合解耦。核心仿真以 headless 进程运行在 host（Arch Linux），通过 `/dev/shm/quadrotor_sim/` 下的 shared memory 暴露标准化的 I/O schema（`QuadrotorState` 输出、`QuadrotorControl` 输入）。ROS 2 作为其中一个 adapter，运行在 Docker 容器中，通过挂载同一段 shared memory 与核心通信，对外暴露 ROS 2 topic。GLFW 渲染也作为独立的 adapter。Schema 同时提供 C++ header 和 Python pydantic 定义，并在 `AGENTS.md` 中记录抽象契约。

## Feasibility

**feasible**

理由：
- `src/main.cc` 中已有的 `Control` struct 和 `actuator_cmds_ptr`（shared_ptr）已经是 ROS-free 的数据通道雏形
- `Simulate` class 本身零 ROS 依赖，`PhysicsLoop` 核心逻辑不依赖 ROS
- MuJoCo 支持 headless 运行（无需 `mjvScene` / `mjrContext` 即可调用 `mj_step`）
- `/dev/shm` 在 Linux 上为 tmpfs，零序列化开销，支持 host↔Docker 跨进程共享
- 现有代码结构清晰，耦合点集中在 `main.cc`（线程创建）、`MuJoCoMessageHandler.cpp`（直接读 `d->qpos`）、`PhysicsLoop` 中的 `rclcpp::ok()` 三处

## Task

将仿真器重构为 core + adapters 架构，以 shared memory 作为 adapter protocol。

## Context

- **当前状态**：`src/main.cc` 直接创建 ROS 线程 + 物理线程 + GLFW 渲染线程，三者共享全局 `mjModel* m`、`mjData* d`
- **关键耦合点**：
  1. `MuJoCoMessageHandler` 持有 `Simulate*` 裸指针，直接读取 `sim_->d->qpos[]`、`sim_->d->sensordata[]`
  2. `PhysicsLoop` 第 263 行调用 `rclcpp::ok()` 检查退出条件
  3. `main()` 硬编码 `rclcpp::init/spin/shutdown` 和线程创建顺序
- **物理模型**：`model/mujoco/drone.xml`，1.0 kg 四旋翼，4 个 actuator（thrust + 3 torque），5 个 sensor
- **目标**：host 跑 headless 核心 + GLFW 渲染（可选），Docker 跑 ROS 2 adapter

## Deliverables

1. **Schema 定义**：C++ header (`include/sim_schema.h`) + Python pydantic (`python/quadrotor_sim/schema.py`) + `AGENTS.md` 文档，三者字段和单位完全对齐
2. **核心仿真器**：headless 可执行文件 `quadrotor_sim_core`，支持 `--model`、`--timestep`、`--real-time-factor` 参数，通过 shm 通信
3. **ROS 2 Adapter**：独立编译的 ROS 2 节点，读 shm 发 topic，收 topic 写 shm
4. **GLFW Adapter**：独立编译的渲染进程，读 shm 渲染窗口
5. **Docker 基础设施**：`Dockerfile` + `docker-compose.yml`，启动 ROS adapter 并挂载 shm
6. **构建系统**：`CMakeLists.txt` 新增 `quadrotor_sim_core`、`quadrotor_sim_ros_adapter`、`quadrotor_sim_glfw_adapter` 三个 target
7. **向后兼容**：保留原有 `quadrotor_simulator` 单体可执行文件不变

## Child Tasks

### Child Task 1: Schema Definition

**Deliverable**: `include/sim_schema.h`、`python/quadrotor_sim/schema.py`、`python/quadrotor_sim/shm.py`、更新的 `AGENTS.md`

- 在 `include/sim_schema.h` 中定义 `QuadrotorControl`、`QuadrotorState`、`ImageData` 三个 struct，128 字节对齐，带 `static_assert` 校验
- 定义 seqlock 读写工具函数（`shm_write_begin/end`、`shm_read`）
- 定义 `/dev/shm/quadrotor_sim/` 路径常量和 segment 文件名常量
- 在 `python/quadrotor_sim/schema.py` 中定义对应的 Pydantic BaseModel，字段名和单位与 C++ 完全对齐
- 在 `python/quadrotor_sim/shm.py` 中实现 Python 侧 shm 读写（使用 `mmap` + `struct`），提供 `ShmWriter` / `ShmReader` 类
- 在 `AGENTS.md` 中新增 `## Simulator Interface Schema` 章节，包含：
  - 架构拓扑图（ASCII art）
  - 完整 I/O 字段表（名称、类型、单位、范围、坐标系）
  - Shared memory 布局和同步协议说明
  - Adapter protocol 描述（如何实现一个 adapter）

### Child Task 2: Core Headless Simulator

**Deliverable**: `src/core/sim_core.{h,cc}`、`src/core/shm_backend.{h,cc}`、`src/core/main.cc`

- `shm_backend`：封装 seqlock-based 的 shared memory 创建/读写/销毁，提供 `create_state_shm()`、`read_control_shm()` 等接口
- `sim_core`：
  - `init(model_path, params)` → 调用 `mj_loadXML`、`mj_makeData`
  - `step(ctrl)` → seqlock 保护下写 `QuadrotorState` 到 shm
  - `run()` → 主循环：从 shm 读最新 `QuadrotorControl` → `mj_step` → 写 `QuadrotorState` → sleep 至 real-time
  - 信号处理（SIGINT/SIGTERM）graceful shutdown
  - **不依赖** GLFW、ROS、`Simulate` class
- `main.cc`：
  - 解析 CLI 参数（`--model`、`--timestep`、`--real-time-factor`）
  - 创建 `QuadrotorSimCore`，调用 `run()`
  - 编译为 **非 ROS 目标**（plain `add_executable`，不 link ROS libs）

### Child Task 3: ROS 2 Adapter

**Deliverable**: `src/ros_adapter/ros_adapter.cpp`

- 独立的 `rclcpp::Node`，**不持有 `Simulate*`**，仅通过 `ShmReader` / `ShmWriter` 与核心通信
- **Publisher**（从 shm 读 `QuadrotorState`，转换为 ROS 消息）：
  - `/ns/odom` (`nav_msgs::Odometry`)：position + orientation → `pose`，lin_vel + ang_vel → `twist`
  - `/ns/imu` (`sensor_msgs::Imu`)：linear_accel → `linear_acceleration`，ang_vel → `angular_velocity`，orientation → `orientation`
  - `/ns/rgb_image` (`sensor_msgs::Image`)：读 shm image segment，封装为 `rgb8` 消息
  - `/clock` (`rosgraph_msgs::Clock`)：`time` → `clock`
- **Subscriber**（收 ROS 消息，写 `QuadrotorControl` 到 shm）：
  - `/ns/cmd` (`geometry_msgs::Wrench`)：`force.z` → `thrust`，`torque.x/y/z` → `torque[0/1/2]`
- 参数：`rate_odom`、`rate_imu`、`world_frame_id`、`body_frame_id`（与原 `MuJoCoMessageHandler` 一致）
- 需重新实现 **IMU 加速度的重力补偿**（`Z - 9.81`），与原实现保持一致
- `rclcpp::spin` 作为 main

### Child Task 4: GLFW Render Adapter

**Deliverable**: `src/glfw_adapter/glfw_adapter_main.cc`

- 从 shm 读取 `QuadrotorState`
- 创建 MuJoCo 渲染上下文（`mjvScene`、`mjvCamera`、`mjrContext`）
- 加载 `drone.xml` 模型（仅用于渲染，不调用 `mj_step`）
- 用 state 中的 `position` + `orientation` 更新 `d->qpos[0..6]`
- 调用 `mjv_updateScene` + `mjr_render` + `glfwSwapBuffers`
- 纯渲染进程，不跑物理

### Child Task 5: Docker Infrastructure

**Deliverable**: `docker/Dockerfile`、`docker/docker-compose.yml`

- `Dockerfile`：基于 `osrf/ros:humble-desktop`（或 `ros:humble-ros-core`），安装依赖，编译 ROS adapter
- `docker-compose.yml`：
  - 挂载 `/dev/shm/quadrotor_sim/` 到容器内
  - 网络模式：`host`（让 ROS 2 discovery 跨 host-container 工作）
  - 环境变量：`ROS_DOMAIN_ID` 可配置
  - 可选：同时启动 core 和 ros_adapter 两个 service
- 提供启动脚本或 compose profile 说明

### Child Task 6: Build System

**Deliverable**: 更新的 `CMakeLists.txt`

- 新增 target：
  - `quadrotor_sim_core`：`src/core/*.cc` + MuJoCo lib，**不 link ROS**
  - `quadrotor_sim_ros_adapter`：`src/ros_adapter/*.cpp` + shm 读写 + ROS 2 libs
  - `quadrotor_sim_glfw_adapter`：`src/glfw_adapter/*.cc` + GLFW + MuJoCo render libs
- 保留原有 `quadrotor_simulator` target 不变（`GLOB_RECURSE` 仍在 `src/` 下抓文件）
- 安装规则：core → `lib/${PROJECT_NAME}/`，ROS adapter → 同，GLFW → 同，模型文件保持不变

## Constraints

- **ABI 稳定**：`QuadrotorControl`、`QuadrotorState` 使用 `uint64_t`、`double`、固定大小 padding，确保 C++ 和 Python 侧 layout 一致
- **Seqlock 正确性**：x86 上 aligned 8-byte 读写是原子的，但必须加 `__sync_synchronize()` 或 `std::atomic_thread_fence` 保证顺序
- **向后兼容**：原 `quadrotor_simulator` 单体可执行文件不可删除、不可改变行为
- **MuJoCo 版本**：继续使用捆绑的 MuJoCo 2.3.2 (`lib/libmujoco.so`)，不升级
- **无 setup.py**：Python 代码不打包，仅作为源码目录提供给 adapter 使用

## Rules

- 核心仿真代码不引入 `rclcpp`、`sensor_msgs`、`geometry_msgs` 等 ROS header
- Shared memory 创建/销毁由 core 进程负责（owner），adapter 只做 attach/detach
- Core 进程启动时若 shm 已存在，报错退出（防止多实例竞争）
- 所有 struct 使用 `#pragma pack(1)` 或 `alignas(128)` + `static_assert` 确保 layout
- Python schema 字段名与 C++ 保持 snake_case 对齐
- 数字常量（如重力加速度 9.81、推力上限 42.0）定义在 schema header 中作为 `constexpr`

## Acceptance

1. `quadrotor_sim_core --model model/mujoco/drone.xml` 启动后，另一个进程能从 `/dev/shm/quadrotor_sim/state` 读到持续更新的 `QuadrotorState`
2. 向 `/dev/shm/quadrotor_sim/ctrl` 写入非零 `QuadrotorControl` 后，state 中的 position/velocity 发生可观测变化
3. ROS adapter 在 Docker 中运行时，host 上 `ros2 topic echo /quadrotor/odom` 能看到实时数据
4. `docker compose up` 一键启动 core + ROS adapter
5. 原 `quadrotor_simulator` 二进制构建和运行不受影响
6. Python `schema.py` 中每个字段名、类型、注释与 C++ `sim_schema.h` 一致

## Verification

- [ ] **Schema 一致性**：对比 `include/sim_schema.h` 和 `python/quadrotor_sim/schema.py`，逐字段确认名称/类型/单位一致；确认 `AGENTS.md` 中的字段表覆盖所有字段
- [ ] **Core headless 构建**：`colcon build --packages-select quadrotor_simulator_mujoco` 生成 `quadrotor_sim_core` 二进制，`ldd` 确认不链接 `rclcpp`
- [ ] **Core 运行**：启动 core 进程，检查 `/dev/shm/quadrotor_sim/state` 存在且持续更新（`stat` 查看 mtime 或 Python 脚本读 sequence 递增）
- [ ] **Control 通路**：向 shm ctrl 写入数据，确认 state 中时间推进且速度非零
- [ ] **ROS adapter 构建与运行**：ROS adapter 正常编译，启动后 `ros2 topic list` 包含 `/quadrotor/odom`、`/quadrotor/imu`、`/clock`、`/quadrotor/cmd`
- [ ] **ROS 闭环**：`ros2 topic pub /quadrotor/cmd` 发送推力命令后，`ros2 topic echo /quadrotor/odom` 观察到位置变化
- [ ] **Docker compose**：`docker compose up` 启动全部服务，无报错
- [ ] **向后兼容**：`quadrotor_simulator` 二进制正常启动且行为不变（原有 launch 文件启动成功）
- [ ] **Python shm 读取**：Python 脚本调用 `python/quadrotor_sim/shm.py` 能从 shm 读到与 C++ struct 一致的字段值
