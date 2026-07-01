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
