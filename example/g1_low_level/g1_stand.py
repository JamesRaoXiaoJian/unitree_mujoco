#!/home/james/miniconda3/bin/python
"""
G1 Stand Example for unitree_mujoco simulator (29-DOF)

Usage:
  ./g1_stand.py              (simulation: domain_id=1, interface=lo)
  ./g1_stand.py enp3s0       (real robot: domain_id=0, interface=enp3s0)
"""

import time
import sys
import numpy as np

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.utils.crc import CRC

G1_NUM_MOTOR = 29
dt = 0.002
crc = CRC()

# Standing pose: slight knee bend, arms at sides
stand_up_joint_pos = np.array([
    -0.1, 0.0, 0.0, 0.25, -0.15, 0.0,   # left leg
    -0.1, 0.0, 0.0, 0.25, -0.15, 0.0,   # right leg
    0.0, 0.0, 0.0,                       # waist
    0.2, 0.2, 0.0, 0.8, 0.0, 0.0, 0.0,  # left arm
    0.2, -0.2, 0.0, 0.8, 0.0, 0.0, 0.0, # right arm
], dtype=float)

# PD gains tuned for MuJoCo simulation
Kp = np.array([
    80, 80, 60, 120, 50, 50,   # left leg
    80, 80, 60, 120, 50, 50,   # right leg
    60, 40, 40,                # waist
    40, 40, 40, 40, 40, 40, 40,  # left arm
    40, 40, 40, 40, 40, 40, 40,  # right arm
], dtype=float)

Kd = np.array([
    2, 2, 2, 3, 1.5, 1.5,   # left leg
    2, 2, 2, 3, 1.5, 1.5,   # right leg
    2, 1.5, 1.5,             # waist
    1, 1, 1, 1, 1, 1, 1,    # left arm
    1, 1, 1, 1, 1, 1, 1,    # right arm
], dtype=float)


class G1Stand:
    def __init__(self, network="lo", domain_id=1):
        ChannelFactoryInitialize(domain_id, network)

        self.cmd_pub = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.cmd_pub.Init()

        self.state_sub = ChannelSubscriber("rt/lowstate", LowState_)
        self.state_sub.Init(self._state_handler, 10)

        self.current_q = np.zeros(G1_NUM_MOTOR)
        self.got_state = False
        self.time = 0.0
        self.counter = 0

    def _state_handler(self, msg: LowState_):
        for i in range(G1_NUM_MOTOR):
            self.current_q[i] = msg.motor_state[i].q
        self.got_state = True

        self.counter += 1
        if self.counter % 500 == 0:
            rpy = msg.imu_state.rpy
            print(f"[G1Stand] IMU rpy: {rpy[0]:.3f} {rpy[1]:.3f} {rpy[2]:.3f} | "
                  f"q[knee]={self.current_q[3]:.3f} {self.current_q[9]:.3f}")

    def run(self):
        print("[G1Stand] Waiting for motor state...")
        while not self.got_state:
            time.sleep(0.01)
        print("[G1Stand] Standing up over 3 seconds...")

        while True:
            step_start = time.perf_counter()
            self.time += dt

            phase = np.tanh(self.time / 1.2)

            cmd = unitree_hg_msg_dds__LowCmd_()
            cmd.mode_pr = 0
            cmd.mode_machine = 5  # G1 29-DOF

            for i in range(G1_NUM_MOTOR):
                cmd.motor_cmd[i].mode = 1
                cmd.motor_cmd[i].tau = 0.0
                cmd.motor_cmd[i].dq = 0.0
                cmd.motor_cmd[i].kp = float(Kp[i])
                cmd.motor_cmd[i].kd = float(Kd[i])
                cmd.motor_cmd[i].q = float(
                    phase * stand_up_joint_pos[i] + (1 - phase) * self.current_q[i]
                )

            cmd.crc = crc.Crc(cmd)
            self.cmd_pub.Write(cmd)

            time_until_next = dt - (time.perf_counter() - step_start)
            if time_until_next > 0:
                time.sleep(time_until_next)


if __name__ == "__main__":
    network = "lo"
    domain_id = 1
    if len(sys.argv) >= 2:
        network = sys.argv[1]
        domain_id = 0

    print(f"[G1Stand] domain_id={domain_id} interface={network}")
    G1Stand(network, domain_id).run()
