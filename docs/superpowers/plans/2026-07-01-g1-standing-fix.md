# G1 Standing Stability Fix Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the G1 robot stand stably in MuJoCo simulation on startup.

**Architecture:** Three changes: (1) use tuned standing pose with bent knees, (2) use joint-specific PD gains matching the example controller, (3) increase foot contact sphere size for ground stability.

**Tech Stack:** C++17, MuJoCo 3.3.6, MJCF XML

## Global Constraints

- Standing pose from `example/g1_low_level/g1_stand.py` (bent knees, tuned gains)
- PD gains: hip=80, knee=120, ankle=50, waist=40-60, arm=40
- Foot contact spheres: increase from 0.005 to 0.02 radius
- Must not break csv_replay workflow (transition phase handles pose difference)

---

### Task 1: Update keyframe to stable standing pose

**Files:**
- Modify: `unitree_robots/g1/g1_29dof.xml:526-528`

**Why:** The current keyframe uses CSV frame 0 (nearly straight legs, knee=0.03 rad). A stable humanoid standing pose requires bent knees (~0.25 rad) to lower the center of mass and give the PD controller room to correct perturbations. The example controller `g1_stand.py` uses this tuned pose.

- [ ] **Step 1: Update keyframe qpos**

Replace the keyframe in `unitree_robots/g1/g1_29dof.xml` with the example's standing pose. The qpos ordering is: root_xyz(3) + root_quat_wxyz(4) + joints(29).

Standing pose from `g1_stand.py`:
- Left leg:  [-0.1, 0.0, 0.0, 0.25, -0.15, 0.0]
- Right leg: [-0.1, 0.0, 0.0, 0.25, -0.15, 0.0]
- Waist:     [0.0, 0.0, 0.0]
- Left arm:  [0.2, 0.2, 0.0, 0.8, 0.0, 0.0, 0.0]
- Right arm: [0.2, -0.2, 0.0, 0.8, 0.0, 0.0, 0.0]

```xml
  <keyframe>
    <key name="standing" qpos="0 0 0.793 1 0 0 0 -0.1 0 0 0.25 -0.15 0 -0.1 0 0 0.25 -0.15 0 0 0 0 0.2 0.2 0 0.8 0 0 0 0.2 -0.2 0 0.8 0 0 0"/>
  </keyframe>
```

- [ ] **Step 2: Verify XML parses correctly**

```bash
cd /home/james/Project/unitree_mujoco/simulate/build
timeout 3 ./unitree_mujoco -r g1 -s scene_29dof.xml 2>&1 | grep "Loaded keyframe"
```

Expected: `Loaded keyframe 0, nq=36`

- [ ] **Step 3: Commit**

```bash
git add unitree_robots/g1/g1_29dof.xml
git commit -m "fix(g1): use bent-knee standing pose in keyframe"
```

---

### Task 2: Use joint-specific PD gains in builtin controller

**Files:**
- Modify: `simulate/src/unitree_sdk2_bridge.h:344-370`

**Why:** The uniform kp=50 is insufficient for leg joints. The example controller uses kp=80 for hips, kp=120 for knees, kp=50 for ankles. The builtin controller must match these gains to hold the standing pose against gravity.

Joint index to gain mapping (from `g1_stand.py`):
- 0-5 (left leg):   Kp=[80,80,60,120,50,50], Kd=[2,2,2,3,1.5,1.5]
- 6-11 (right leg):  Kp=[80,80,60,120,50,50], Kd=[2,2,2,3,1.5,1.5]
- 12-14 (waist):     Kp=[60,40,40], Kd=[2,1.5,1.5]
- 15-21 (left arm):  Kp=[40,40,40,40,40,40,40], Kd=[1,1,1,1,1,1,1]
- 22-28 (right arm): Kp=[40,40,40,40,40,40,40], Kd=[1,1,1,1,1,1,1]

- [ ] **Step 1: Add gain arrays to G1Bridge**

In `simulate/src/unitree_sdk2_bridge.h`, add as private members in G1Bridge (after `cmd_received_`):

```cpp
    // Tuned PD gains from g1_stand.py example
    const std::array<float, 29> builtin_kp_ = {
        80, 80, 60, 120, 50, 50,   // left leg
        80, 80, 60, 120, 50, 50,   // right leg
        60, 40, 40,                 // waist
        40, 40, 40, 40, 40, 40, 40,  // left arm
        40, 40, 40, 40, 40, 40, 40,  // right arm
    };
    const std::array<float, 29> builtin_kd_ = {
        2, 2, 2, 3, 1.5, 1.5,     // left leg
        2, 2, 2, 3, 1.5, 1.5,     // right leg
        2, 1.5, 1.5,               // waist
        1, 1, 1, 1, 1, 1, 1,      // left arm
        1, 1, 1, 1, 1, 1, 1,      // right arm
    };
```

- [ ] **Step 2: Update builtin_ctrl to use per-joint gains**

In `compute_ctrl()`, replace the hardcoded gains:

```cpp
                // Builtin: PD hold initial standing pose
                float builtin_ctrl =
                    builtin_kp_[i] * (builtin_pose_[i] - mj_data_->sensordata[i]) +
                    builtin_kd_[i] * (0.0f - mj_data_->sensordata[i + num_motor_]);
```

- [ ] **Step 3: Build**

```bash
cd /home/james/Project/unitree_mujoco/simulate/build
make -j4
```

Expected: Build succeeds.

- [ ] **Step 4: Commit**

```bash
git add simulate/src/unitree_sdk2_bridge.h
git commit -m "fix(g1): use joint-specific PD gains from g1_stand example"
```

---

### Task 3: Increase foot contact sphere size

**Files:**
- Modify: `unitree_robots/g1/g1_29dof.xml:124-127` (left foot)
- Modify: `unitree_robots/g1/g1_29dof.xml:184-187` (right foot)

**Why:** The current 5mm-radius spheres provide minimal contact area. Increasing to 20mm gives a much more stable base of support, similar to the go2 model's 22mm spheres.

- [ ] **Step 1: Update left foot contact spheres**

Replace lines 124-127:
```xml
                  <geom size="0.02" pos="-0.05 0.025 -0.03" condim="6" friction="0.4 0.02 0.01" margin="0.001" rgba="0.2 0.2 0.2 1" />
                  <geom size="0.02" pos="-0.05 -0.025 -0.03" condim="6" friction="0.4 0.02 0.01" margin="0.001" rgba="0.2 0.2 0.2 1" />
                  <geom size="0.02" pos="0.12 0.03 -0.03" condim="6" friction="0.4 0.02 0.01" margin="0.001" rgba="0.2 0.2 0.2 1" />
                  <geom size="0.02" pos="0.12 -0.03 -0.03" condim="6" friction="0.4 0.02 0.01" margin="0.001" rgba="0.2 0.2 0.2 1" />
```

- [ ] **Step 2: Update right foot contact spheres**

Replace lines 184-187 with the same pattern (same size, condim, friction, margin).

- [ ] **Step 3: Commit**

```bash
git add unitree_robots/g1/g1_29dof.xml
git commit -m "fix(g1): increase foot contact sphere size from 5mm to 20mm"
```

---

### Task 4: Build and verify standing

**Files:**
- No file changes

- [ ] **Step 1: Full rebuild**

```bash
cd /home/james/Project/unitree_mujoco/simulate
rm -rf build && mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release && make -j4
```

- [ ] **Step 2: Start simulator and verify standing**

```bash
./unitree_mujoco -r g1 -s scene_29dof.xml
```

Expected: Robot stands upright with bent knees, does not fall during startup hold or after physics starts.

- [ ] **Step 3: Verify csv_replay still works**

Terminal 2:
```bash
cd ~/Project/g1_motion_player/build
./csv_replay ../assets/wave.csv lo
```

Expected: csv_replay connects, robot transitions from standing pose to keyframe first frame, motion plays correctly.

- [ ] **Step 4: Final commit**

```bash
cd /home/james/Project/unitree_mujoco
git add -A
git commit -m "fix: G1 standing stability (pose + gains + foot contacts)"
```
