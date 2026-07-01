#!/home/james/miniconda3/bin/python
"""
G1 Stand Debug - 双进程调试版
增加详细调试输出，定位站立失败原因
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

# 站立目标姿态
stand_up_joint_pos = np.array([
    -0.1, 0.0, 0.0, 0.25, -0.15, 0.0,   # left leg
    -0.1, 0.0, 0.0, 0.25, -0.15, 0.0,   # right leg
    0.0, 0.0, 0.0,                       # waist
    0.2, 0.2, 0.0, 0.8, 0.0, 0.0, 0.0,  # left arm
    0.2, -0.2, 0.0, 0.8, 0.0, 0.0, 0.0, # right arm
], dtype=float)

# PD增益
Kp = np.array([
    80, 80, 60, 120, 50, 50,
    80, 80, 60, 120, 50, 50,
    60, 40, 40,
    40, 40, 40, 40, 40, 40, 40,
    40, 40, 40, 40, 40, 40, 40,
], dtype=float)

Kd = np.array([
    2, 2, 2, 3, 1.5, 1.5,
    2, 2, 2, 3, 1.5, 1.5,
    2, 1.5, 1.5,
    1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1,
], dtype=float)


class G1StandDebug:
    def __init__(self, network="lo", domain_id=1):
        ChannelFactoryInitialize(domain_id, network)

        self.cmd_pub = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.cmd_pub.Init()
        print("[DEBUG] Publisher initialized on rt/lowcmd")

        self.state_sub = ChannelSubscriber("rt/lowstate", LowState_)
        self.state_sub.Init(self._state_handler, 10)
        print("[DEBUG] Subscriber initialized on rt/lowstate")

        self.current_q = np.zeros(G1_NUM_MOTOR)
        self.current_dq = np.zeros(G1_NUM_MOTOR)
        self.got_state = False
        self.time = 0.0
        self.counter = 0
        self.last_state_time = 0

    def _state_handler(self, msg: LowState_):
        self.last_state_time = time.time()
        for i in range(G1_NUM_MOTOR):
            self.current_q[i] = msg.motor_state[i].q
            self.current_dq[i] = msg.motor_state[i].dq
        self.got_state = True

        self.counter += 1
        if self.counter <= 5 or self.counter % 250 == 0:
            rpy = msg.imu_state.rpy
            quat = msg.imu_state.quaternion
            print(f"[STATE #{self.counter}] IMU rpy: [{rpy[0]:.3f}, {rpy[1]:.3f}, {rpy[2]:.3f}]")
            print(f"  quat: [{quat[0]:.3f}, {quat[1]:.3f}, {quat[2]:.3f}, {quat[3]:.3f}]")
            print(f"  q[0:6] (L-leg):  [{', '.join(f'{x:.3f}' for x in self.current_q[0:6])}]")
            print(f"  q[6:12] (R-leg): [{', '.join(f'{x:.3f}' for x in self.current_q[6:12])}]")
            print(f"  pelvis_height(z): check frame_pos in sensor data")

    def run(self):
        print("[DEBUG] Waiting for motor state on rt/lowstate...")
        timeout_start = time.time()
        while not self.got_state:
            time.sleep(0.01)
            if time.time() - timeout_start > 5.0:
                print("[ERROR] No state received after 5s! DDS communication issue.")
                print("  Check: simulator running? domain_id match? interface match?")
                return

        print(f"[DEBUG] Got first state after {time.time()-timeout_start:.2f}s")
        print(f"[DEBUG] Initial q[0:6]: {self.current_q[0:6]}")
        print(f"[DEBUG] Starting stand-up sequence...")

        duration = 3.0
        log_interval = 100  # 每100步打印一次

        step = 0
        while True:
            step_start = time.perf_counter()
            self.time += dt

            # tanh平滑
            phase = np.tanh(self.time / 1.2)

            cmd = unitree_hg_msg_dds__LowCmd_()
            cmd.mode_pr = 0
            cmd.mode_machine = 5

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

            # 调试：打印实际发送的命令
            if step < 3 or step % log_interval == 0:
                q_cmd = [cmd.motor_cmd[i].q for i in range(6)]
                q_actual = list(self.current_q[0:6])
                err = [q_cmd[i] - q_actual[i] for i in range(6)]
                tau_est = [cmd.motor_cmd[i].kp * err[i] for i in range(6)]
                print(f"[CMD step={step} t={self.time:.3f} phase={phase:.3f}]")
                print(f"  q_cmd[0:6]:   [{', '.join(f'{x:.3f}' for x in q_cmd)}]")
                print(f"  q_actual[0:6]:[{', '.join(f'{x:.3f}' for x in q_actual)}]")
                print(f"  error[0:6]:   [{', '.join(f'{x:.3f}' for x in err)}]")
                print(f"  tau_est[0:6]: [{', '.join(f'{x:.1f}' for x in tau_est)}]")
                print(f"  mode_pr={cmd.mode_pr} mode_machine={cmd.mode_machine} crc={cmd.crc}")

            step += 1
            time_until_next = dt - (time.perf_counter() - step_start)
            if time_until_next > 0:
                time.sleep(time_until_next)


if __name__ == "__main__":
    network = "lo"
    domain_id = 1
    if len(sys.argv) >= 2:
        network = sys.argv[1]
        domain_id = 0

    print(f"[G1StandDebug] domain_id={domain_id} interface={network}")
    g1 = G1StandDebug(network, domain_id)
    g1.run()
