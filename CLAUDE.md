# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

unitree_mujoco bridges Unitree's SDK2 DDS communication layer with the MuJoCo physics engine, enabling sim-to-real robotics development. Control programs written against Unitree SDK interfaces (`unitree_sdk2`, `unitree_ros2`, `unitree_sdk2_python`) work identically in simulation and on physical robots — only the DDS domain ID and network interface change.

## Build & Run Commands

### C++ Simulator (primary, `simulate/`)

```bash
# Dependencies
sudo apt install libyaml-cpp-dev libspdlog-dev libboost-all-dev libglfw3-dev
# unitree_sdk2 installed to /opt/unitree_robotics
# MuJoCo symlinked: simulate/mujoco -> ~/.mujoco/mujoco-3.3.6

cd simulate && mkdir build && cd build
cmake .. && make -j4

# Run (flags: -r robot, -s scene, -i domain_id, -n network_interface)
./unitree_mujoco -r go2 -s scene_terrain.xml

# Joystick diagnostic
./jstest
```

### Python Simulator (`simulate_python/`)

```bash
pip3 install mujoco pygame  # + unitree_sdk2_python from source
cd simulate_python
python3 ./unitree_mujoco.py
python3 ./test/test_unitree_sdk2.py  # test: sends 1Nm torque to all motors
```

### Examples

```bash
# C++ (sim mode: domain_id=1, interface=lo; real: pass NIC name)
cd example/cpp && mkdir build && cd build && cmake .. && make -j4
./stand_go2              # simulation
./stand_go2 enp3s0       # real robot

# Python
python3 ./example/python/stand_go2.py           # simulation
python3 ./example/python/stand_go2.py enp3s0     # real robot

# ROS2
source ~/unitree_ros2/setup.sh
cd example/ros2 && colcon build
source ~/unitree_ros2/setup_local.sh
export ROS_DOMAIN_ID=1
./install/stand_go2/bin/stand_go2
```

### Terrain Tool

```bash
pip3 install noise opencv-python numpy
cd terrain_tool && python3 ./terrain_generator.py
```

## Architecture

Two parallel simulator implementations, both bridging MuJoCo ↔ DDS:

**C++ (`simulate/src/`)** — `main.cc` runs physics in a background thread, renders via MuJoCo's GLFW viewer, and bridges sensor/actuator data over DDS in a separate thread. Uses `unitree_sdk2` C++ library. `param.h` handles YAML config + CLI args (boost_program_options). `physics_joystick.h` provides Xbox/Switch gamepad support.

**Python (`simulate_python/`)** — `unitree_mujoco.py` runs simulation and rendering in separate threads. Uses `mujoco` Python bindings + `unitree_sdk2py`.

### Bridge Design (`unitree_sdk2_bridge.h`)

`UnitreeSDK2BridgeBase` → `RobotBridge<LowCmd_t, LowState_t>` (template) → concrete bridges:
- **`Go2Bridge`** — for robots with ≤20 motors (Go2, B2, H1, B2w, Go2w) using `unitree_go` IDL
- **`G1Bridge`** — for robots with >20 motors (G1, H1-2) using `unitree_hg` IDL; adds secondary IMU and BMS state publishers

Motor control model applied per timestep: `tau + kp*(q_target - q_actual) + kd*(dq_target - dq_actual)`, matching the real robot's PD interface.

DDS topics published: `rt/lowstate` (motor + IMU), `rt/sportmodestate` (position/velocity), `rt/wirelesscontroller` (joystick). Subscribed: `rt/lowcmd` (motor commands).

### Robot Models (`unitree_robots/`)

Each robot directory contains: `<robot>.xml` (MJCF model), `scene.xml` (basic scene), `scene_terrain.xml` (terrain scene), `meshes/`, and textures. Robots: a2, b2, b2w, g1 (23-DOF and 29-DOF variants), go2, go2w, h1, h1_2, h2, r1.

## Key Configuration

`simulate/config.yaml` — robot name, scene file, domain_id, interface, joystick settings, elastic band toggle. CLI flags override YAML values.

Simulation defaults: domain_id=1, interface="lo" (loopback). Real robots: domain_id=0, interface=<robot NIC>.

Elastic band: virtual spring for humanoid harness simulation. Keyboard: `9` toggle, `7`/`8` lower/lift.

## Sim-to-Real Workflow

1. Write control code against Unitree SDK (any language with SDK bindings)
2. Run simulator with domain_id=1, interface="lo"
3. Run control code — it communicates with the simulator over DDS on localhost
4. Deploy same control code to real robot with domain_id=0, interface=<robot NIC>

## G1 arm_sdk 适配（进行中）

### 需求

让实机的 `csv_replay.cpp`（/home/james/Project/g1_motion_player）能直接在仿真中运行，无需修改代码。

### 已完成功能

1. **arm_sdk weight 机制** — G1Bridge 订阅 `rt/arm_sdk`，读取 `motor_cmd[29]` 的 weight 值，混合内建控制器和用户指令
2. **compute_ctrl() 重构** — RobotBridge 提取虚方法，G1Bridge 覆盖实现 weight 混合
3. **cmd_received_ 检测** — 收到指令前用内建 PD 控制器保持站立
4. **DDS domain_id** — config.yaml 从 1 改为 0（与 csv_replay 匹配）
5. **keyframe 姿态** — 改为 g1_stand.py 的弯曲膝盖姿态（膝关节 0.25 rad）
6. **PD 增益** — 按关节区分：膝=120, 髋=80, 踝=50, 臂=40
7. **脚部碰撞球** — 从 5mm 增大到 20mm，添加 condim=6, friction=0.4

### 待解决问题：机器人无法站立

所有上述修复已应用但机器人仍无法在仿真中站立。

#### 排查路径

| 尝试 | 问题 | 修复 | 结果 |
|------|------|------|------|
| 1 | keyframe 腿伸直（膝 0.03 rad） | 改为弯曲膝盖（0.25 rad） | 仍倒下 |
| 2 | PD 增益统一 kp=50 | 按关节区分（膝=120） | 仍倒下 |
| 3 | 脚部碰撞球 5mm | 增大到 20mm + friction | 仍倒下 |

#### 可能的剩余原因

1. **MuJoCo 仿真参数** — 没有 `<option>` 元素，可能需要调整 timestep、impratio、solver 参数
2. **ctrl 信号时序** — bridge 线程写入 ctrl，但物理线程的 mj_step 可能在 ctrl 更新前执行
3. **startup_hold 期间 ctrl 未应用** — hold 期间 mj_step 不执行，hold 结束后第一次 mj_step 使用的 ctrl 可能是旧值
4. **脚部接触面形状** — 球体可能不如盒体稳定，考虑用 box geom 替代
5. **keyframe 加载时序** — `mj_resetDataKeyframe` 在 `mj_makeData` 之后，但 `sim->Load()` 可能有异步操作覆盖 qpos

#### 下一步调查方向

- 添加调试输出：在 mj_step 前后打印 ctrl[] 和 qpos[] 的值
- 检查 startup_hold 结束时 ctrl 是否非零
- 尝试在 XML 中添加 `<option impratio="100" cone="elliptic" />`
- 尝试用 box geom 替代球体作为脚部接触面
- 检查是否需要在 bridge 的 compute_ctrl 中加锁 sim.mtx
