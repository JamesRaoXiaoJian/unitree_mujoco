# Unitree MuJoCo (Fork)

<p align="center">
  <a href="readme_zh.md">中文</a> · <a href="readme.md">English</a>
</p>

> **Fork 自 [unitreerobotics/unitree_mujoco](https://github.com/unitreerobotics/unitree_mujoco)**
> 本仓库在其基础上增加了 G1 人形机器人的仿真控制支持。

## Fork 改动

相比上游仓库，本 fork 新增了以下内容：

### G1 仿真支持

- **G1Bridge** (`simulate/src/unitree_sdk2_bridge.h`): 基于 `unitree_hg` IDL 的完整桥接，支持 29-DOF G1 模型
- **内置站立控制器**: keyframe 弯曲膝盖姿态 + 按关节区分的 PD 增益，下肢始终维持站立
- **arm_sdk weight 机制**: 订阅 `rt/arm_sdk`，上肢按 weight 混合用户控制和内置控制
- **二级 IMU / BMS 发布**: `rt/secondary_imu`、`rt/lf/bmsstate`

### G1 动作工具 (`tools/g1_motion/`)

| 工具 | 用途 |
|------|------|
| `csv_replay` | CSV 关键帧播放器，通过 DDS 发送到仿真或实机 |
| `headless_g1_sim` | 无头仿真，支持 bridge/direct/sensor 三种模式 |
| `state_recorder` | 全流程状态录制（idle→engage→replay→disengage→idle） |
| `check_g1_standing.py` | Python 站立诊断，支持 CSV 回放和平衡模式测试 |

### 仿真器改进

- **Startup hold**: 启动时冻结物理 ~4s，等待控制脚本连接
- **速度钳位**: Transition/Replay 阶段限制关节角速度，防止失稳
- **Monitor 线程**: 实时比较目标位置与实际关节误差

## 快速开始

### 依赖

```bash
sudo apt install libyaml-cpp-dev libspdlog-dev libboost-all-dev libglfw3-dev
# unitree_sdk2 → /opt/unitree_robotics
# MuJoCo 3.3.6 → ~/.mujoco/mujoco-3.3.6
```

### 构建

```bash
# C++ 仿真器
cmake -S simulate -B simulate/build -DCMAKE_BUILD_TYPE=Release
cmake --build simulate/build -j4

# G1 工具集
cmake -S tools/g1_motion -B tools/g1_motion/build -DCMAKE_BUILD_TYPE=Release
cmake --build tools/g1_motion/build -j4
```

### 运行

```bash
# GUI 仿真
cd simulate/build
./unitree_mujoco -r g1 -s scene_29dof.xml

# 另一个终端发送动作
tools/g1_motion/build/csv_replay tools/g1_motion/assets/wave.csv 60 lo

# Headless 验证
tools/g1_motion/build/headless_g1_sim unitree_robots/g1/scene_29dof.xml 8 lo 0 bridge
```

G1 站立、headless 验证、GUI/Xvfb 运行和 CSV 动作播放的详细流程见 [G1 运行手册](./docs/g1_motion_runbook_zh.md)。

## 原始文档

完整的原始项目文档请参考上游仓库: [unitreerobotics/unitree_mujoco](https://github.com/unitreerobotics/unitree_mujoco)

以下为原始文档的关键内容：

---

## 介绍

`unitree_mujoco` 是基于 `Unitree sdk2` 和 `mujoco` 开发的仿真器。用户使用 `Unitree_sdk2`、`unitree_ros2` 和 `unitree_sdk2_python` 开发的控制程序可以方便地接入该仿真器，实现仿真到实物的开发流程。

![](./doc/func.png)

## 目录结构

- `simulate`: C++ 仿真器（推荐）
- `simulate_python`: Python 仿真器
- `unitree_robots`: MJCF 模型文件（go2, b2, g1, h1, h2, r1 等）
- `terrain_tool`: 地形生成工具
- `example`: 例程（C++, Python, ROS2）
- `tools/g1_motion`: G1 上半身关键帧播放工具

## 支持的消息

**当前版本仅支持底层开发，主要用于控制器的 sim to real 验证**

- `LowCmd` / `LowState`: 电机控制指令和状态
- `SportModeState`: 机器人位置和速度
- `IMUState`: 胸部 IMU 数据（仅 G1，`rt/secondary_imu`）

**注：** Go2/B2/H1 使用 `unitree_go` IDL，G1/H1-2 使用 `unitree_hg` IDL。

## 相关链接

- [unitree_sdk2](https://github.com/unitreerobotics/unitree_sdk2)
- [unitree_sdk2_python](https://github.com/unitreerobotics/unitree_sdk2_python)
- [unitree_ros2](https://github.com/unitreerobotics/unitree_ros2)
- [Unitree 文档](https://support.unitree.com/home/zh/developer)
- [MuJoCo 文档](https://mujoco.readthedocs.io/en/stable/overview.html)
