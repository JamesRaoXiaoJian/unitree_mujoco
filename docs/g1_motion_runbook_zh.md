# G1 站立和上半身动作运行手册

本文记录当前仓库中 G1 站立控制、`arm_sdk` 上半身动作播放、headless 验证和 GUI/Xvfb 运行方式。

## 依赖检查

C++ 仿真和验证工具需要：

```bash
sudo apt-get install -y \
  libyaml-cpp-dev libspdlog-dev libboost-all-dev libglfw3-dev \
  xvfb mesa-utils libosmesa6 libosmesa6-dev python3-pip
```

Python 诊断脚本需要当前 Python 环境可导入 `mujoco`：

```bash
python3 -c 'import mujoco; print(mujoco.__version__)'
```

## 构建

```bash
cmake -S simulate -B simulate/build -DCMAKE_BUILD_TYPE=Release
cmake --build simulate/build -j4

cmake -S tools/g1_motion -B tools/g1_motion/build -DCMAKE_BUILD_TYPE=Release
cmake --build tools/g1_motion/build -j4
```

## GUI 运行

如果当前桌面 GLX 正常，可以直接运行：

```bash
cd simulate/build
./unitree_mujoco -r g1 -s scene_29dof.xml
```

如果出现类似 `GLX: Failed to create context: BadValue`，说明当前桌面 OpenGL context 创建失败。可以使用 Xvfb + llvmpipe：

```bash
cd simulate/build
xvfb-run -a ./unitree_mujoco -r g1 -s scene_29dof.xml
```

检查 Xvfb OpenGL：

```bash
xvfb-run -a glxinfo -B
```

正常时应看到 Mesa `llvmpipe` renderer。用 `timeout` 自动截断 GUI 程序时，退出码 `124` 表示程序已启动后被 `timeout` 停止，不代表 GLFW 失败。

## Headless 站立验证

Python 直接 MuJoCo 验证：

```bash
python3 tools/g1_motion/check_g1_standing.py --duration 8 --log-interval 4
```

C++ DDS bridge 验证：

```bash
tools/g1_motion/build/headless_g1_sim unitree_robots/g1/scene_29dof.xml 8 lo 0
```

通过条件：

- summary 中 `fell=false`
- `final_z` 约为 `0.802`
- `max_abs_tilt` 远小于 `0.8`
- `source=builtin`

## DDS 动作播放验证

启动 headless bridge：

```bash
tools/g1_motion/build/headless_g1_sim unitree_robots/g1/scene_29dof.xml 24 lo 0
```

另一个终端发送动作：

```bash
tools/g1_motion/build/csv_replay tools/g1_motion/assets/wave.csv 60 lo
tools/g1_motion/build/csv_replay tools/g1_motion/assets/zuoyi.csv 60 lo
```

通过条件：

- `csv_replay` 输出 `Robot returned to built-in control.`
- headless 日志中播放阶段 `source=arm_sdk`
- 结束后回到 `source=builtin`
- summary 中 `fell=false`

## 控制策略说明

G1 bridge 默认保持 keyframe 站立姿态。收到 `rt/arm_sdk` 后：

- 下肢 0-11 号关节继续使用内置站立 PD
- 腰和上肢 12-28 号关节按 `arm_sdk` 的 weight 机制混合控制
- `rt/arm_sdk` 和 `rt/lowcmd` 都会检查 DDS timeout，避免过期命令持续接管

`csv_replay` 只发送腰和上肢命令；下肢命令槽位保持 no-op，由仿真 bridge 负责站立稳定。
