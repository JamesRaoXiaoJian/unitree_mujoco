# G1 arm_sdk Weight Mechanism - Simulation Adapter Design

**Date:** 2026-06-30
**Goal:** Make the real robot's `csv_replay.cpp` work directly in simulation without modification.

## Background

The real G1 robot's `csv_replay.cpp` (at `/home/james/Project/g1_motion_player`) replays CSV keyframe animations on the robot. It uses:

- **Topic:** `rt/arm_sdk` (publishes `unitree_hg::msg::dds_::LowCmd_`)
- **Weight mechanism:** `motor_cmd[29].q()` controls blending between built-in controller and user commands
- **Formula:** `Motor_real = weight * User_Cmd + (1 - weight) * BuiltIn_Cmd`
- **Phases:** Engage (weight 0→1) → Transition → Replay → Disengage (weight 1→0)

The simulator currently only subscribes to `rt/lowcmd` and has no weight mechanism.

## Design

### Change 1: Add dummy actuators to G1 XML

**File:** `unitree_robots/g1/g1_29dof.xml`

The SDK expects 35 motor command slots (29 joints + weight + 5 unused). The MuJoCo model has only 29 actuators. Add 6 dummy actuators at the end of `<actuator>`:

```xml
<!-- Dummy actuators for SDK weight mechanism (indices 29-34) -->
<motor name="dummy_29" />
<motor name="dummy_30" />
<motor name="dummy_31" />
<motor name="dummy_32" />
<motor name="dummy_33" />
<motor name="dummy_34" />
```

This makes `mj_model_->nu = 35`, matching the SDK's 35 joint indices:
- Index 0-28: 29 physical joints
- Index 29 (`kNotUsedJoint`): weight channel
- Index 30-34: unused, stay at 0

Dummy actuators without a `joint` attribute produce no force/torque.

### Change 2: Refactor RobotBridge ctrl computation

**File:** `simulate/src/unitree_sdk2_bridge.h`

Extract the ctrl computation from `run()` into a virtual method:

```cpp
virtual void compute_ctrl() {
    std::lock_guard<std::mutex> lock(lowcmd->mutex_);
    for (int i = 0; i < num_motor_; i++) {
        auto& m = lowcmd->msg_.motor_cmd()[i];
        mj_data_->ctrl[i] = m.tau() +
            m.kp() * (m.q() - mj_data_->sensordata[i]) +
            m.kd() * (m.dq() - mj_data_->sensordata[i + num_motor_]);
    }
}
```

`run()` calls `compute_ctrl()` instead of inline code. No behavior change for Go2Bridge.

### Change 3: G1Bridge arm_sdk subscriber and weight blending

**File:** `simulate/src/unitree_sdk2_bridge.h`

New members in G1Bridge:

```cpp
// arm_sdk subscriber (same message type as lowcmd)
std::shared_ptr<unitree::robot::g1::subscription::LowCmd> arm_sdk;

// Weight mechanism state
float weight_ = 0.0f;
std::array<float, 35> builtin_pose_;
bool pose_recorded_ = false;
```

Constructor subscribes to `rt/arm_sdk`:

```cpp
arm_sdk = std::make_shared<unitree::robot::g1::subscription::LowCmd>("rt/arm_sdk");
```

Override `compute_ctrl()`:

```cpp
void compute_ctrl() override {
    // Read weight from arm_sdk
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
        // Weight blending mode
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

### Builtin Controller

When `weight=0`, the built-in controller maintains the standing pose via PD control:

- `kp_builtin = 50.0` (between csv_replay's leg kp=20 and arm kp=120)
- `kd_builtin = 3.0`
- Target: initial joint positions recorded at first `run()` call

This matches the real robot's behavior where the built-in controller maintains standing when the user has no control.

### Coexistence with lowcmd

- When `arm_sdk` weight > 0: G1Bridge uses weight blending
- When `arm_sdk` weight = 0: falls back to standard `lowcmd` processing
- The two modes are mutually exclusive

### Usage

```bash
# Start simulator (unchanged)
cd simulate/build && ./unitree_mujoco -r g1 -s scene_29dof.xml

# Run csv_replay (unchanged, same as real robot)
cd /home/james/Project/g1_motion_player/build
./csv_replay ../assets/wave.csv lo
```

No changes needed to csv_replay.cpp, main.cc, or config.yaml.

## Files Modified

| File | Change |
|------|--------|
| `unitree_robots/g1/g1_29dof.xml` | Add 6 dummy actuators |
| `simulate/src/unitree_sdk2_bridge.h` | Refactor RobotBridge + G1Bridge weight logic |

## Risks and Mitigations

1. **Dummy actuator physics impact:** Actuators without `joint` attribute produce no torque. Verified by MuJoCo semantics.
2. **Thread safety:** `arm_sdk->mutex_` protects the weight read and command read. Same pattern as existing `lowcmd`.
3. **Builtin PD gain tuning:** 50/3.0 is a reasonable default. Can be adjusted if standing pose drifts.
