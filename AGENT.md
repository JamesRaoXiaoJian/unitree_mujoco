# AGENT.md

## 当前任务状态

### G1 arm_sdk 仿真适配 — 进行中

**目标**：让实机的 `csv_replay.cpp` 能直接在仿真中运行

**已完成**：
- [x] G1Bridge 订阅 `rt/arm_sdk` DDS 话题
- [x] weight 混合机制（motor_cmd[29] 作为 weight）
- [x] compute_ctrl() 虚方法重构
- [x] cmd_received_ 检测（收到指令前保持站立）
- [x] DDS domain_id 改为 0（与 csv_replay 匹配）
- [x] keyframe 改为弯曲膝盖姿态
- [x] PD 增益按关节区分
- [x] 脚部碰撞球增大到 20mm

**待解决**：机器人无法在仿真中站立

---

## 问题排查记录

### 问题描述

机器人启动后倒下，无法保持站立姿态。

### 已尝试的修复

#### 1. keyframe 姿态（已修复）
- **问题**：CSV 首帧腿几乎伸直（膝关节 0.03 rad）
- **修复**：改为 g1_stand.py 的弯曲膝盖姿态（0.25 rad）
- **结果**：仍倒下

#### 2. PD 增益（已修复）
- **问题**：统一 kp=50，膝关节需要 kp=120
- **修复**：按关节区分增益（膝=120, 髋=80, 踝=50, 臂=40）
- **结果**：仍倒下

#### 3. 脚部碰撞球（已修复）
- **问题**：5mm 半径球太小
- **修复**：增大到 20mm，添加 condim=6, friction=0.4, margin=0.001
- **结果**：仍倒下

### 可能的剩余原因

1. **MuJoCo 仿真参数** — 没有 `<option>` 元素
2. **ctrl 信号时序** — bridge 线程和物理线程的竞态条件
3. **startup_hold 期间 ctrl 未应用** — hold 结束后第一次 mj_step 的 ctrl 可能是旧值
4. **脚部接触面形状** — 球体可能不如盒体稳定
5. **keyframe 加载时序** — sim->Load() 可能覆盖 qpos

### 下一步调查方向

- [ ] 添加调试输出：mj_step 前后打印 ctrl[] 和 qpos[]
- [ ] 检查 startup_hold 结束时 ctrl 是否非零
- [ ] 尝试添加 `<option impratio="100" cone="elliptic" />`
- [ ] 尝试用 box geom 替代球体
- [ ] 检查是否需要加锁 sim.mtx

---

## 关键文件

| 文件 | 用途 |
|------|------|
| `simulate/src/unitree_sdk2_bridge.h` | G1Bridge 实现（arm_sdk, weight, PD 控制） |
| `unitree_robots/g1/g1_29dof.xml` | 模型、keyframe、碰撞几何 |
| `simulate/src/main.cc` | 物理循环、startup hold |
| `simulate/config.yaml` | domain_id=0 |
| `example/g1_low_level/g1_stand.py` | 参考站立姿态和增益 |
| `tools/g1_motion/csv_replay.cpp` | 关键帧播放工具 |
| `tools/g1_motion/assets/*.csv` | 关键帧数据 |

## G1 Keyframe Playback Tool

**编译：**
```bash
cd tools/g1_motion && mkdir build && cd build
cmake .. && make -j4
```

**运行：**
```bash
# 仿真
./csv_replay ../assets/wave.csv lo
# 实物
./csv_replay ../assets/wave.csv enp3s0
```

**环境要求：**
- unitree_sdk2 安装到 `/opt/unitree_robotics/`
- config.yaml 中 domain_id=0（与 csv_replay 匹配）

**关键帧 CSV 格式：** 36 列/行
- 列 0-2: root 位置 (xyz)
- 列 3-6: root 四元数 (xyzw)
- 列 7-35: 29 个关节角（弧度）

**weight 机制：**
- motor_cmd[29].q() = weight
- weight=0: 内建 PD 控制器保持站立
- weight=1: 用户完全控制
- Engage(0→1) → Transition → Replay → Disengage(1→0)

---

## DDS 通信

| 话题 | 方向 | 用途 |
|------|------|------|
| `rt/arm_sdk` | 订阅 | csv_replay 发送指令（weight + 关节位置） |
| `rt/lowcmd` | 订阅 | 低层电机指令 |
| `rt/lowstate` | 发布 | 电机状态 + IMU |
| `rt/sportmodestate` | 发布 | 位置/速度 |

---

## 参考代码

**g1_stand.py 站立姿态**：
```python
stand_up_joint_pos = [
    -0.1, 0.0, 0.0, 0.25, -0.15, 0.0,   # left leg
    -0.1, 0.0, 0.0, 0.25, -0.15, 0.0,   # right leg
    0.0, 0.0, 0.0,                       # waist
    0.2, 0.2, 0.0, 0.8, 0.0, 0.0, 0.0,  # left arm
    0.2, -0.2, 0.0, 0.8, 0.0, 0.0, 0.0, # right arm
]
```

**g1_stand.py PD 增益**：
```python
Kp = [80, 80, 60, 120, 50, 50,   # left leg
      80, 80, 60, 120, 50, 50,   # right leg
      60, 40, 40,                 # waist
      40, 40, 40, 40, 40, 40, 40,  # left arm
      40, 40, 40, 40, 40, 40, 40]  # right arm

Kd = [2, 2, 2, 3, 1.5, 1.5,     # left leg
      2, 2, 2, 3, 1.5, 1.5,     # right leg
      2, 1.5, 1.5,               # waist
      1, 1, 1, 1, 1, 1, 1,      # left arm
      1, 1, 1, 1, 1, 1, 1]      # right arm
```
