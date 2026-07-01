/*
 * G1 Stand Debug - 双进程调试版
 * 详细打印 DDS 通信、关节状态、命令发送情况
 *
 * Usage: ./g1_stand_debug [network_interface]
 *   无参数: simulation (domain_id=1, interface=lo)
 *   有参数: real robot (domain_id=0, interface=参数值)
 */

#include <cmath>
#include <iostream>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <chrono>

#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>

static const std::string HG_CMD_TOPIC = "rt/lowcmd";
static const std::string HG_STATE_TOPIC = "rt/lowstate";

using namespace unitree::common;
using namespace unitree::robot;
using namespace unitree_hg::msg::dds_;

const int G1_NUM_MOTOR = 29;

template <typename T>
class DataBuffer {
 public:
  void SetData(const T& newData) {
    std::unique_lock<std::shared_mutex> lock(mutex_);
    data_ = std::make_shared<T>(newData);
  }
  std::shared_ptr<const T> GetData() {
    std::shared_lock<std::shared_mutex> lock(mutex_);
    return data_;
  }
 private:
  std::shared_ptr<T> data_;
  std::shared_mutex mutex_;
};

struct MotorState {
  std::array<float, G1_NUM_MOTOR> q = {};
  std::array<float, G1_NUM_MOTOR> dq = {};
};

// 站立目标姿态 (弧度)
std::array<float, G1_NUM_MOTOR> stand_pose = {
    -0.1f, 0.0f, 0.0f, 0.25f, -0.15f, 0.0f,   // left leg
    -0.1f, 0.0f, 0.0f, 0.25f, -0.15f, 0.0f,   // right leg
    0.0f, 0.0f, 0.0f,                          // waist
    0.2f, 0.2f, 0.0f, 0.8f, 0.0f, 0.0f, 0.0f, // left arm
    0.2f, -0.2f, 0.0f, 0.8f, 0.0f, 0.0f, 0.0f // right arm
};

// PD增益 - 为MuJoCo仿真调高
std::array<float, G1_NUM_MOTOR> Kp = {
    80, 80, 60, 120, 50, 50,   // left leg
    80, 80, 60, 120, 50, 50,   // right leg
    60, 40, 40,                // waist
    40, 40, 40, 40, 40, 40, 40, // left arm
    40, 40, 40, 40, 40, 40, 40  // right arm
};

std::array<float, G1_NUM_MOTOR> Kd = {
    2, 2, 2, 3, 1.5f, 1.5f,   // left leg
    2, 2, 2, 3, 1.5f, 1.5f,   // right leg
    2, 1.5f, 1.5f,             // waist
    1, 1, 1, 1, 1, 1, 1,      // left arm
    1, 1, 1, 1, 1, 1, 1       // right arm
};

inline uint32_t Crc32Core(uint32_t* ptr, uint32_t len) {
  uint32_t xbit = 0;
  uint32_t data = 0;
  uint32_t CRC32 = 0xFFFFFFFF;
  const uint32_t dwPolynomial = 0x04c11db7;
  for (uint32_t i = 0; i < len; i++) {
    xbit = 1 << 31;
    data = ptr[i];
    for (uint32_t bits = 0; bits < 32; bits++) {
      if (CRC32 & 0x80000000) {
        CRC32 <<= 1;
        CRC32 ^= dwPolynomial;
      } else {
        CRC32 <<= 1;
      }
      if (data & xbit) CRC32 ^= dwPolynomial;
      xbit >>= 1;
    }
  }
  return CRC32;
}

class G1StandDebug {
 public:
  G1StandDebug(std::string network, int domain_id)
      : time_(0.0), control_dt_(0.002), duration_(3.0),
        counter_(0), got_state_(false) {
    std::cout << "[DEBUG] Initializing DDS with domain_id=" << domain_id
              << " interface=" << network << std::endl;

    ChannelFactory::Instance()->Init(domain_id, network);
    std::cout << "[DEBUG] ChannelFactory initialized" << std::endl;

    lowcmd_pub_.reset(new ChannelPublisher<LowCmd_>(HG_CMD_TOPIC));
    lowcmd_pub_->InitChannel();
    std::cout << "[DEBUG] Publisher on " << HG_CMD_TOPIC << std::endl;

    lowstate_sub_.reset(new ChannelSubscriber<LowState_>(HG_STATE_TOPIC));
    lowstate_sub_->InitChannel(
        std::bind(&G1StandDebug::LowStateHandler, this, std::placeholders::_1), 1);
    std::cout << "[DEBUG] Subscriber on " << HG_STATE_TOPIC << std::endl;

    control_thread_ = CreateRecurrentThreadEx("control", UT_CPU_ID_NONE, 2000,
                                               &G1StandDebug::Control, this);
    cmd_thread_ = CreateRecurrentThreadEx("cmd_writer", UT_CPU_ID_NONE, 2000,
                                           &G1StandDebug::WriteCommand, this);
    std::cout << "[DEBUG] Control threads started (2ms period)" << std::endl;
    std::cout << "[DEBUG] Waiting for state on " << HG_STATE_TOPIC << "..." << std::endl;
  }

 private:
  void LowStateHandler(const void* message) {
    LowState_ state = *(const LowState_*)message;

    MotorState ms;
    for (int i = 0; i < G1_NUM_MOTOR; ++i) {
      ms.q[i] = state.motor_state()[i].q();
      ms.dq[i] = state.motor_state()[i].dq();
    }
    motor_state_.SetData(ms);

    if (!got_state_) {
      got_state_ = true;
      std::cout << "[DEBUG] *** FIRST STATE RECEIVED ***" << std::endl;
      std::cout << "[DEBUG] mode_machine=" << (int)state.mode_machine() << std::endl;
      std::cout << "[DEBUG] Initial q[0:6]: ";
      for (int i = 0; i < 6; i++) std::cout << ms.q[i] << " ";
      std::cout << std::endl;
      std::cout << "[DEBUG] Initial q[6:12]: ";
      for (int i = 6; i < 12; i++) std::cout << ms.q[i] << " ";
      std::cout << std::endl;
    }

    if (++counter_ % 500 == 0) {
      auto& rpy = state.imu_state().rpy();
      printf("[STATE #%d] IMU rpy: %.3f %.3f %.3f | q[knee_L]=%.3f q[knee_R]=%.3f\n",
             counter_, rpy[0], rpy[1], rpy[2], ms.q[3], ms.q[9]);
    }
  }

  void Control() {
    auto ms = motor_state_.GetData();
    if (!ms) return;

    time_ += control_dt_;
    float phase = std::tanh(time_ / 1.2f);

    LowCmd_ cmd;
    cmd.mode_pr() = 0;  // PR mode
    cmd.mode_machine() = 5;  // G1 29-DOF

    for (int i = 0; i < G1_NUM_MOTOR; ++i) {
      cmd.motor_cmd()[i].mode() = 1;
      cmd.motor_cmd()[i].tau() = 0.0f;
      cmd.motor_cmd()[i].dq() = 0.0f;
      cmd.motor_cmd()[i].kp() = Kp[i];
      cmd.motor_cmd()[i].kd() = Kd[i];
      cmd.motor_cmd()[i].q() = phase * stand_pose[i] + (1.0f - phase) * ms->q[i];
    }

    cmd.crc() = Crc32Core((uint32_t*)&cmd, (sizeof(cmd) >> 2) - 1);
    motor_cmd_.SetData(cmd);

    // 调试输出
    static int step = 0;
    if (step < 5 || step % 500 == 0) {
      printf("[CMD step=%d t=%.3f phase=%.3f] mode_pr=%d mode_machine=%d\n",
             step, time_, phase, (int)cmd.mode_pr(), (int)cmd.mode_machine());
      printf("  q_cmd[0:6]:   ");
      for (int i = 0; i < 6; i++) printf("%.3f ", cmd.motor_cmd()[i].q());
      printf("\n  q_actual[0:6]:");
      for (int i = 0; i < 6; i++) printf("%.3f ", ms->q[i]);
      printf("\n  error[0:6]:   ");
      for (int i = 0; i < 6; i++) printf("%.3f ", cmd.motor_cmd()[i].q() - ms->q[i]);
      printf("\n  kp[0:6]:      ");
      for (int i = 0; i < 6; i++) printf("%.0f ", cmd.motor_cmd()[i].kp());
      printf("\n");
    }
    step++;
  }

  void WriteCommand() {
    auto mc = motor_cmd_.GetData();
    if (mc) {
      lowcmd_pub_->Write(*mc);
    }
  }

  double time_, control_dt_, duration_;
  int counter_;
  bool got_state_;
  DataBuffer<MotorState> motor_state_;
  DataBuffer<LowCmd_> motor_cmd_;
  ChannelPublisherPtr<LowCmd_> lowcmd_pub_;
  ChannelSubscriberPtr<LowState_> lowstate_sub_;
  ThreadPtr control_thread_, cmd_thread_;
};

int main(int argc, char const* argv[]) {
  std::string network = "lo";
  int domain_id = 1;

  if (argc >= 2) {
    network = argv[1];
    domain_id = 0;
  }

  std::cout << "[G1StandDebug] domain_id=" << domain_id
            << " interface=" << network << std::endl;

  G1StandDebug g1(network, domain_id);

  // 等待10秒收集调试信息
  for (int i = 0; i < 10; i++) {
    sleep(1);
    std::cout << "[DEBUG] Running... " << (i+1) << "s" << std::endl;
  }

  std::cout << "[DEBUG] Done." << std::endl;
  return 0;
}
