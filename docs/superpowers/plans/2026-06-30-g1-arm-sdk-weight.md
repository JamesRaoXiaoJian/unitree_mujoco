# G1 arm_sdk Weight Mechanism Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the real robot's `csv_replay.cpp` work directly in simulation by adding arm_sdk weight mechanism support to G1Bridge.

**Architecture:** G1Bridge subscribes to `rt/arm_sdk` DDS topic, reads the weight value from `motor_cmd[29]`, and blends between a built-in PD standing controller and user commands. No XML model changes needed — weight is read from the SDK message, not from MuJoCo ctrl.

**Tech Stack:** C++17, MuJoCo 3.3.6, unitree_sdk2 (DDS/CycloneDDS), yaml-cpp

## Global Constraints

- `num_motor_` stays at 29 (matches MuJoCo actuators and sensors)
- Weight read from SDK message index 29 (`kNotUsedJoint`), not from `ctrl[]`
- PD gains for built-in controller: kp=50.0, kd=3.0
- Bridge runs at 1000 Hz via `RecurrentThread`
- Follow existing code style: `snake_case` for variables, RAII for resources

---

### Task 1: Refactor RobotBridge — extract compute_ctrl()

**Files:**
- Modify: `simulate/src/unitree_sdk2_bridge.h:174-187`

**Interfaces:**
- Produces: `virtual void compute_ctrl()` — called by `run()`, can be overridden by subclasses

- [ ] **Step 1: Read current run() method**

Read `simulate/src/unitree_sdk2_bridge.h` lines 174-187 to confirm the ctrl computation code.

- [ ] **Step 2: Add compute_ctrl() virtual method to RobotBridge**

Add a protected virtual method after the `run()` method (around line 246):

```cpp
protected:
    virtual void compute_ctrl()
    {
        std::lock_guard<std::mutex> lock(lowcmd->mutex_);
        for(int i(0); i<num_motor_; i++) {
            auto & m = lowcmd->msg_.motor_cmd()[i];
            mj_data_->ctrl[i] = m.tau() +
                                m.kp() * (m.q() - mj_data_->sensordata[i]) +
                                m.kd() * (m.dq() - mj_data_->sensordata[i + num_motor_]);
        }
    }
```

- [ ] **Step 3: Replace inline ctrl code in run() with compute_ctrl() call**

In `run()` (line 174), replace lines 178-187:

```cpp
// Before (lines 178-187):
        // lowcmd
        {
            std::lock_guard<std::mutex> lock(lowcmd->mutex_);
            for(int i(0); i<num_motor_; i++) {
                auto & m = lowcmd->msg_.motor_cmd()[i];
                mj_data_->ctrl[i] = m.tau() +
                                    m.kp() * (m.q() - mj_data_->sensordata[i]) +
                                    m.kd() * (m.dq() - mj_data_->sensordata[i + num_motor_]);
            }
        }
```

Replace with:

```cpp
        // lowcmd / arm_sdk ctrl
        compute_ctrl();
```

- [ ] **Step 4: Verify Go2Bridge still works**

Go2Bridge is a typedef of `RobotBridge<...>` with no override. It will use the base `compute_ctrl()` — identical behavior to before.

Run: `cd simulate/build && cmake .. && make -j4`
Expected: Build succeeds with no errors.

- [ ] **Step 5: Commit**

```bash
git add simulate/src/unitree_sdk2_bridge.h
git commit -m "refactor: extract compute_ctrl() virtual method from RobotBridge::run()"
```

---

### Task 2: Add arm_sdk subscriber and weight state to G1Bridge

**Files:**
- Modify: `simulate/src/unitree_sdk2_bridge.h:259-323`

**Interfaces:**
- Consumes: `unitree::robot::g1::subscription::LowCmd` (same type as `lowcmd`)
- Produces: `arm_sdk` subscriber on `rt/arm_sdk`, `weight_` float, `builtin_pose_[]` array

- [ ] **Step 1: Add new member variables to G1Bridge**

In the G1Bridge class, add private members after `secondary_imustate` (line 322):

```cpp
private:
    // arm_sdk subscriber (same message type as lowcmd)
    std::shared_ptr<unitree::robot::g1::subscription::LowCmd> arm_sdk;

    // Weight mechanism state
    float weight_ = 0.0f;
    std::array<float, 29> builtin_pose_;
    bool pose_recorded_ = false;
```

- [ ] **Step 2: Subscribe to rt/arm_sdk in constructor**

In G1Bridge constructor (line 262), add after `secondary_imustate` initialization (line 275):

```cpp
        // Subscribe to arm_sdk for weight mechanism
        arm_sdk = std::make_shared<unitree::robot::g1::subscription::LowCmd>("rt/arm_sdk");
```

- [ ] **Step 3: Build to verify syntax**

Run: `cd simulate/build && cmake .. && make -j4`
Expected: Build succeeds. The new members and subscriber are declared but not yet used.

- [ ] **Step 4: Commit**

```bash
git add simulate/src/unitree_sdk2_bridge.h
git commit -m "feat(g1): add arm_sdk subscriber and weight state to G1Bridge"
```

---

### Task 3: Implement weight blending in G1Bridge::compute_ctrl()

**Files:**
- Modify: `simulate/src/unitree_sdk2_bridge.h:259-323` (G1Bridge class)

**Interfaces:**
- Consumes: `arm_sdk->msg_.motor_cmd()[29].q()` (weight), `arm_sdk->msg_.motor_cmd()[i]` (user commands)
- Consumes: `builtin_pose_[]` (initial standing pose)
- Produces: `mj_data_->ctrl[i]` (blended control output)

- [ ] **Step 1: Override compute_ctrl() in G1Bridge**

Add the override method to G1Bridge (after the `run()` override):

```cpp
    void compute_ctrl() override
    {
        // Read weight from arm_sdk message (kNotUsedJoint = 29)
        {
            std::lock_guard<std::mutex> lock(arm_sdk->mutex_);
            weight_ = arm_sdk->msg_.motor_cmd()[29].q();
        }

        // Record initial standing pose (first call)
        if (!pose_recorded_) {
            for (int i = 0; i < num_motor_; i++) {
                builtin_pose_[i] = mj_data_->sensordata[i];
            }
            pose_recorded_ = true;
        }

        if (weight_ > 0.0f) {
            // Weight blending mode: builtin + user cmd
            std::lock_guard<std::mutex> lock(arm_sdk->mutex_);
            for (int i = 0; i < num_motor_; i++) {
                auto& m = arm_sdk->msg_.motor_cmd()[i];
                float user_ctrl = m.tau() +
                    m.kp() * (m.q() - mj_data_->sensordata[i]) +
                    m.kd() * (m.dq() - mj_data_->sensordata[i + num_motor_]);

                // Builtin: PD hold initial standing pose
                float builtin_ctrl =
                    50.0f * (builtin_pose_[i] - mj_data_->sensordata[i]) +
                    3.0f * (0.0f - mj_data_->sensordata[i + num_motor_]);

                mj_data_->ctrl[i] = weight_ * user_ctrl + (1.0f - weight_) * builtin_ctrl;
            }
        } else {
            // Standard lowcmd mode
            RobotBridge::compute_ctrl();
        }
    }
```

- [ ] **Step 2: Build**

Run: `cd simulate/build && cmake .. && make -j4`
Expected: Build succeeds.

- [ ] **Step 3: Commit**

```bash
git add simulate/src/unitree_sdk2_bridge.h
git commit -m "feat(g1): implement weight blending in G1Bridge::compute_ctrl()"
```

---

### Task 4: Verify integration — build and basic smoke test

**Files:**
- No file changes

- [ ] **Step 1: Full rebuild**

```bash
cd /home/james/Project/unitree_mujoco/simulate
rm -rf build && mkdir build && cd build
cmake .. && make -j4
```

Expected: Build succeeds with no warnings related to our changes.

- [ ] **Step 2: Verify simulator starts with G1**

```bash
cd /home/james/Project/unitree_mujoco/simulate/build
timeout 5 ./unitree_mujoco -r g1 -s scene_29dof.xml 2>&1 || true
```

Expected: Simulator starts, prints scene information, no crash. (Will timeout after 5s — that's expected.)

- [ ] **Step 3: Verify arm_sdk topic is subscribed**

Check that the simulator output doesn't show errors about `rt/arm_sdk` subscription. The DDS subscriber initializes silently on success.

- [ ] **Step 4: Commit (if any fixups needed)**

```bash
git add -A
git commit -m "fix: address integration issues from smoke test"
```

---

### Task 5: End-to-end test with csv_replay

**Files:**
- No file changes

- [ ] **Step 1: Build csv_replay (if not already built)**

```bash
cd /home/james/Project/g1_motion_player
mkdir -p build && cd build
cmake .. && make -j4
```

Expected: Build succeeds.

- [ ] **Step 2: Start simulator in background**

```bash
cd /home/james/Project/unitree_mujoco/simulate/build
./unitree_mujoco -r g1 -s scene_29dof.xml &
SIM_PID=$!
sleep 3
```

Expected: Simulator window opens, G1 robot visible in standing pose.

- [ ] **Step 3: Run csv_replay with wave motion**

```bash
cd /home/james/Project/g1_motion_player/build
./csv_replay ../assets/wave.csv lo
```

Expected:
- "Connected." printed
- "Engaging (weight 0→1)..." printed
- "Transitioning..." printed
- "Replaying 600 frames..." printed
- Robot performs wave motion in simulator window
- "Disengaging..." printed
- "Robot returned to built-in control." printed

- [ ] **Step 4: Verify robot returns to standing after disengage**

After csv_replay finishes, the robot should return to its standing pose (weight back to 0, built-in controller holds pose).

- [ ] **Step 5: Clean up**

```bash
kill $SIM_PID 2>/dev/null || true
```

- [ ] **Step 6: Final commit**

```bash
cd /home/james/Project/unitree_mujoco
git add -A
git commit -m "feat: G1 arm_sdk weight mechanism for simulation adapter"
```
