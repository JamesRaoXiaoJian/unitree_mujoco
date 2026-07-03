#!/usr/bin/env python3
"""Headless MuJoCo check for the G1 standing pose and built-in PD gains."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

import mujoco
import numpy as np


G1_NUM_MOTOR = 29
ARM_JOINTS = np.array(
    [
        15,
        16,
        17,
        18,
        19,
        20,
        21,
        22,
        23,
        24,
        25,
        26,
        27,
        28,
        12,
        13,
        14,
    ],
    dtype=np.int64,
)

STAND_POSE = np.array(
    [
        -0.1,
        0.0,
        0.0,
        0.25,
        -0.15,
        0.0,
        -0.1,
        0.0,
        0.0,
        0.25,
        -0.15,
        0.0,
        0.0,
        0.0,
        0.0,
        0.2,
        0.2,
        0.0,
        0.8,
        0.0,
        0.0,
        0.0,
        0.2,
        -0.2,
        0.0,
        0.8,
        0.0,
        0.0,
        0.0,
    ],
    dtype=np.float64,
)

KP = np.array(
    [
        80,
        80,
        60,
        120,
        50,
        50,
        80,
        80,
        60,
        120,
        50,
        50,
        60,
        40,
        40,
        40,
        40,
        40,
        40,
        40,
        40,
        40,
        40,
        40,
        40,
        40,
        40,
        40,
        40,
    ],
    dtype=np.float64,
)

KD = np.array(
    [
        2,
        2,
        2,
        3,
        1.5,
        1.5,
        2,
        2,
        2,
        3,
        1.5,
        1.5,
        2,
        1.5,
        1.5,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
    ],
    dtype=np.float64,
)


def quat_wxyz_to_rpy(q: np.ndarray) -> tuple[float, float, float]:
    w, x, y, z = q
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch_arg = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(pitch_arg)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def summarize_contacts(model: mujoco.MjModel, data: mujoco.MjData) -> str:
    names = []
    for i in range(data.ncon):
        contact = data.contact[i]
        for geom_id in (contact.geom1, contact.geom2):
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_id)
            if name:
                names.append(name)
    return ",".join(sorted(set(names))) if names else "-"


def contact_bounds(data: mujoco.MjData) -> tuple[float, float, float, float] | None:
    if data.ncon == 0:
        return None

    points = np.array([data.contact[i].pos[:2] for i in range(data.ncon)])
    return (
        float(np.min(points[:, 0])),
        float(np.max(points[:, 0])),
        float(np.min(points[:, 1])),
        float(np.max(points[:, 1])),
    )


def load_csv_frames(path: Path) -> np.ndarray:
    frames = []
    with path.open(newline="") as f:
        for row in csv.reader(f):
            if not row:
                continue
            values = np.array([float(cell) for cell in row], dtype=np.float64)
            if len(values) == 36:
                frames.append(values[7 : 7 + G1_NUM_MOTOR])
    if not frames:
        raise RuntimeError(f"no 36-column frames in {path}")
    return np.vstack(frames)


def log_state(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    target_q: np.ndarray,
    next_log: float,
) -> float:
    roll, pitch, yaw = quat_wxyz_to_rpy(data.qpos[3:7])
    bounds = contact_bounds(data)
    bounds_text = (
        "x=[{:.3f},{:.3f}] y=[{:.3f},{:.3f}]".format(*bounds)
        if bounds is not None
        else "-"
    )
    q = data.qpos[7 : 7 + G1_NUM_MOTOR]
    print(
        "t={:.3f} z={:.3f} rpy=({:.3f},{:.3f},{:.3f}) "
        "com=({:.3f},{:.3f},{:.3f}) support={} "
        "joint_err={:.3f} ctrl_max={:.1f} ncon={} contacts={}".format(
            data.time,
            data.qpos[2],
            roll,
            pitch,
            yaw,
            data.subtree_com[0, 0],
            data.subtree_com[0, 1],
            data.subtree_com[0, 2],
            bounds_text,
            float(np.max(np.abs(target_q - q))),
            float(np.max(np.abs(data.ctrl))),
            data.ncon,
            summarize_contacts(model, data),
        )
    )
    return next_log


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--scene",
        type=Path,
        default=Path("unitree_robots/g1/scene_29dof.xml"),
    )
    parser.add_argument("--duration", type=float, default=8.0)
    parser.add_argument("--log-interval", type=float, default=0.5)
    parser.add_argument("--min-height", type=float, default=0.55)
    parser.add_argument("--max-tilt", type=float, default=0.8)
    parser.add_argument("--use-keyframe-target", action="store_true")
    parser.add_argument("--csv-first-frame", type=Path)
    parser.add_argument("--replay-csv", type=Path)
    parser.add_argument("--fps", type=float, default=60.0)
    parser.add_argument("--replay-kp", type=float, default=120.0)
    parser.add_argument("--replay-kd", type=float, default=3.0)
    parser.add_argument("--transition-max-vel", type=float, default=0.5)
    parser.add_argument("--replay-max-vel", type=float, default=0.8)
    parser.add_argument("--pitch-kp", type=float, default=0.0)
    parser.add_argument("--pitch-kd", type=float, default=0.0)
    parser.add_argument("--roll-kp", type=float, default=0.0)
    parser.add_argument("--roll-kd", type=float, default=0.0)
    parser.add_argument("--kp-scale", type=float, default=8.0)
    parser.add_argument("--kd-scale", type=float, default=8.0)
    parser.add_argument(
        "--balance-mode",
        choices=("none", "ankle", "ankle-hip"),
        default="none",
    )
    args = parser.parse_args()

    model = mujoco.MjModel.from_xml_path(str(args.scene))
    data = mujoco.MjData(model)

    if model.nu != G1_NUM_MOTOR:
        raise RuntimeError(f"expected {G1_NUM_MOTOR} actuators, got {model.nu}")
    if model.nkey <= 0:
        raise RuntimeError("model has no keyframes")

    mujoco.mj_resetDataKeyframe(model, data, 0)
    mujoco.mj_forward(model, data)

    if args.csv_first_frame is not None:
        with args.csv_first_frame.open(newline="") as f:
            row = next(csv.reader(f))
        values = np.array([float(cell) for cell in row], dtype=np.float64)
        if len(values) != 36:
            raise RuntimeError(f"expected 36 columns in CSV first frame, got {len(values)}")
        data.qpos[0:3] = values[0:3]
        data.qpos[3:7] = [values[6], values[3], values[4], values[5]]
        data.qpos[7 : 7 + G1_NUM_MOTOR] = values[7 : 7 + G1_NUM_MOTOR]
        data.qvel[:] = 0.0
        mujoco.mj_forward(model, data)
        target_q = values[7 : 7 + G1_NUM_MOTOR].copy()
    elif args.use_keyframe_target:
        target_q = data.qpos[7 : 7 + G1_NUM_MOTOR].copy()
    else:
        target_q = STAND_POSE

    next_log = 0.0
    fell = False
    max_abs_tilt = 0.0
    max_joint_err = 0.0
    max_ctrl = 0.0

    if args.replay_csv is not None:
        frames = load_csv_frames(args.replay_csv)
        initial_arm = data.qpos[7 + ARM_JOINTS].copy()
        cmd_arm = initial_arm.copy()
        replay_start = 3.0
        replay_end = replay_start + len(frames) / args.fps
        return_end = replay_end + 2.0
        total_duration = return_end + 2.0
        dt = model.opt.timestep

        while data.time < total_duration:
            q = data.qpos[7 : 7 + G1_NUM_MOTOR]
            dq = data.qvel[6 : 6 + G1_NUM_MOTOR]
            builtin_ctrl = (args.kp_scale * KP) * (target_q - q) + (args.kd_scale * KD) * (0.0 - dq)
            data.ctrl[:] = builtin_ctrl

            if data.time < 1.0:
                weight = data.time / 1.0
                arm_target = initial_arm
                max_delta = args.transition_max_vel * dt
            elif data.time < replay_start:
                weight = 1.0
                arm_target = frames[0, ARM_JOINTS]
                max_delta = args.transition_max_vel * dt
            elif data.time < replay_end:
                weight = 1.0
                frame_id = min(int((data.time - replay_start) * args.fps), len(frames) - 1)
                arm_target = frames[frame_id, ARM_JOINTS]
                max_delta = args.replay_max_vel * dt
            elif data.time < return_end:
                weight = 1.0
                arm_target = initial_arm
                max_delta = args.transition_max_vel * dt
            else:
                weight = max(0.0, 1.0 - (data.time - return_end) / 2.0)
                arm_target = initial_arm
                max_delta = args.transition_max_vel * dt

            cmd_arm += np.clip(arm_target - cmd_arm, -max_delta, max_delta)
            arm_q = q[ARM_JOINTS]
            arm_dq = dq[ARM_JOINTS]
            user_ctrl = args.replay_kp * (cmd_arm - arm_q) + args.replay_kd * (0.0 - arm_dq)
            data.ctrl[ARM_JOINTS] = weight * user_ctrl + (1.0 - weight) * builtin_ctrl[ARM_JOINTS]
            max_ctrl = max(max_ctrl, float(np.max(np.abs(data.ctrl))))

            mujoco.mj_step(model, data)

            roll, pitch, _ = quat_wxyz_to_rpy(data.qpos[3:7])
            tilt = max(abs(roll), abs(pitch))
            max_abs_tilt = max(max_abs_tilt, tilt)
            max_joint_err = max(max_joint_err, float(np.max(np.abs(target_q - q))))
            if data.qpos[2] < args.min_height or tilt > args.max_tilt:
                fell = True

            if data.time + 1e-12 >= next_log:
                log_state(model, data, target_q, next_log)
                next_log += args.log_interval

        print(
            "summary: replay={} fell={} final_z={:.3f} max_abs_tilt={:.3f} "
            "max_joint_err={:.3f} max_ctrl={:.1f}".format(
                args.replay_csv,
                fell,
                data.qpos[2],
                max_abs_tilt,
                max_joint_err,
                max_ctrl,
            )
        )
        return 1 if fell else 0

    while data.time < args.duration:
        q = data.qpos[7 : 7 + G1_NUM_MOTOR]
        dq = data.qvel[6 : 6 + G1_NUM_MOTOR]
        data.ctrl[:] = (args.kp_scale * KP) * (target_q - q) + (args.kd_scale * KD) * (0.0 - dq)
        roll, pitch, yaw = quat_wxyz_to_rpy(data.qpos[3:7])

        if args.balance_mode != "none":
            roll_feedback = args.roll_kp * roll + args.roll_kd * data.qvel[3]
            pitch_feedback = args.pitch_kp * pitch + args.pitch_kd * data.qvel[4]
            data.ctrl[4] += pitch_feedback
            data.ctrl[10] += pitch_feedback
            data.ctrl[5] += roll_feedback
            data.ctrl[11] += -roll_feedback
            if args.balance_mode == "ankle-hip":
                data.ctrl[0] += -0.5 * pitch_feedback
                data.ctrl[6] += -0.5 * pitch_feedback
                data.ctrl[1] += -0.5 * roll_feedback
                data.ctrl[7] += 0.5 * roll_feedback
        max_ctrl = max(max_ctrl, float(np.max(np.abs(data.ctrl))))

        mujoco.mj_step(model, data)

        roll, pitch, yaw = quat_wxyz_to_rpy(data.qpos[3:7])
        tilt = max(abs(roll), abs(pitch))
        max_abs_tilt = max(max_abs_tilt, tilt)
        max_joint_err = max(max_joint_err, float(np.max(np.abs(target_q - q))))

        if data.qpos[2] < args.min_height or tilt > args.max_tilt:
            fell = True

        if data.time + 1e-12 >= next_log:
            log_state(model, data, target_q, next_log)
            next_log += args.log_interval

    print(
        "summary: fell={} final_z={:.3f} max_abs_tilt={:.3f} "
        "max_joint_err={:.3f} max_ctrl={:.1f}".format(
            fell,
            data.qpos[2],
            max_abs_tilt,
            max_joint_err,
            max_ctrl,
        )
    )
    return 1 if fell else 0


if __name__ == "__main__":
    raise SystemExit(main())
