/*
 * G1 Stand Example for unitree_mujoco simulator (29-DOF)
 *
 * Usage:
 *   ./g1_stand              (simulation: domain_id=1, interface=lo)
 *   ./g1_stand enp3s0       (real robot: domain_id=0, interface=enp3s0)
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

// Standing pose: slight knee bend, arms at sides
std::array<float, G1_NUM_MOTOR> stand_pose = {
    -0.1f, 0.0f, 0.0f, 0.25f, -0.15f, 0.0f,
    -0.1f, 0.0f, 0.0f, 0.25f, -0.15f, 0.0f,
    0.0f, 0.0f, 0.0f,
    0.2f, 0.2f, 0.0f, 0.8f, 0.0f, 0.0f, 0.0f,
    0.2f, -0.2f, 0.0f, 0.8f, 0.0f, 0.0f, 0.0f
};

std::array<float, G1_NUM_MOTOR> Kp = {
    80, 80, 60, 120, 50, 50,
    80, 80, 60, 120, 50, 50,
    60, 40, 40,
    40, 40, 40, 40, 40, 40, 40,
    40, 40, 40, 40, 40, 40, 40
};

std::array<float, G1_NUM_MOTOR> Kd = {
    2, 2, 2, 3, 1.5f, 1.5f,
    2, 2, 2, 3, 1.5f, 1.5f,
    2, 1.5f, 1.5f,
    1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1
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

class G1Stand {
 public:
  G1Stand(std::string network, int domain_id)
      : time_(0.0), control_dt_(0.002), duration_(3.0),
        counter_(0), got_state_(false) {
    ChannelFactory::Instance()->Init(domain_id, network);

    lowcmd_pub_.reset(new ChannelPublisher<LowCmd_>(HG_CMD_TOPIC));
    lowcmd_pub_->InitChannel();

    lowstate_sub_.reset(new ChannelSubscriber<LowState_>(HG_STATE_TOPIC));
    lowstate_sub_->InitChannel(
        std::bind(&G1Stand::LowStateHandler, this, std::placeholders::_1), 1);

    control_thread_ = CreateRecurrentThreadEx("control", UT_CPU_ID_NONE, 2000,
                                               &G1Stand::Control, this);
    cmd_thread_ = CreateRecurrentThreadEx("cmd_writer", UT_CPU_ID_NONE, 2000,
                                           &G1Stand::WriteCommand, this);

    std::cout << "[G1Stand] Initialized. Standing up over "
              << duration_ << "s." << std::endl;
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
      std::cout << "[G1Stand] State OK. q[knee]=" << ms.q[3] << std::endl;
    }

    if (++counter_ % 500 == 0) {
      auto& rpy = state.imu_state().rpy();
      printf("[G1Stand] IMU: %.3f %.3f %.3f | knee: %.3f %.3f\n",
             rpy[0], rpy[1], rpy[2], ms.q[3], ms.q[9]);
    }
  }

  void Control() {
    auto ms = motor_state_.GetData();
    if (!ms) return;

    time_ += control_dt_;
    float phase = std::tanh(time_ / 1.2f);

    LowCmd_ cmd;
    cmd.mode_pr() = 0;
    cmd.mode_machine() = 5;

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

  std::cout << "[G1Stand] domain_id=" << domain_id
            << " interface=" << network << std::endl;

  G1Stand g1(network, domain_id);

  while (true) {
    sleep(10);
  }
  return 0;
}
