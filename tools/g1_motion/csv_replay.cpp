// G1 Upper-Body CSV Motion Replay
// Sends keyframes to robot via rt/arm_sdk with weight mechanism.
//
// Usage: ./csv_replay <csv> [fps] [net]
//        ./csv_replay <net> <csv> [fps]  (backward compatible)
//        default net is eno0

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstring>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <mutex>

static const std::string kTopicArmSDK = "rt/arm_sdk";
static const std::string kTopicState = "rt/lowstate";

// All joint indices for G1 (35 DOF)
enum JointIndex {
    // Left leg (0-5)
    kLeftHipPitch, kLeftHipRoll, kLeftHipYaw, kLeftKnee, kLeftAnkle, kLeftAnkleRoll,
    // Right leg (6-11)
    kRightHipPitch, kRightHipRoll, kRightHipYaw, kRightKnee, kRightAnkle, kRightAnkleRoll,
    // Waist (12-14)
    kWaistYaw, kWaistRoll, kWaistPitch,
    // Left arm (15-21)
    kLeftShoulderPitch, kLeftShoulderRoll, kLeftShoulderYaw, kLeftElbow,
    kLeftWristRoll, kLeftWristPitch, kLeftWristYaw,
    // Right arm (22-28)
    kRightShoulderPitch, kRightShoulderRoll, kRightShoulderYaw, kRightElbow,
    kRightWristRoll, kRightWristPitch, kRightWristYaw,
    // Weight slot + unused
    kNotUsedJoint, kNotUsedJoint1, kNotUsedJoint2, kNotUsedJoint3, kNotUsedJoint4, kNotUsedJoint5
};

// Upper-body joint indices we control (arms + waist = 17 DOF)
static const std::array<int, 17> kArmJoints = {
    15, 16, 17, 18, 19, 20, 21,  // left arm
    22, 23, 24, 25, 26, 27, 28,  // right arm
    12, 13, 14,                   // waist
};

// Leg joint indices we hold at initial position (12 DOF)
static const std::array<int, 12> kLegJoints = {
    0, 1, 2, 3, 4, 5,      // left leg
    6, 7, 8, 9, 10, 11,     // right leg
};

// PD gains — 提高以获得更精准的跟踪
static constexpr float kKp = 120.0f;
static constexpr float kKd = 3.0f;
// 速度钳位：Transition和Replay阶段都启用，确保稳定
static constexpr float kTransitionMaxVel = 0.5f;  // rad/s
static constexpr float kReplayMaxVel = 0.8f;      // rad/s，Replay阶段稍快

// weight=1.0，与官方 g1_arm7_sdk_dds_example 一致
static constexpr float kFinalWeightTarget = 1.0f;

struct CsvFrame {
    std::array<float, 29> joints;
};

std::vector<CsvFrame> LoadCsv(const std::string& path) {
    std::vector<CsvFrame> frames;
    std::ifstream file(path);
    if (!file.is_open()) {
        std::cerr << "ERROR: Cannot open " << path << std::endl;
        return frames;
    }
    std::string line;
    while (std::getline(file, line)) {
        if (line.empty()) continue;
        std::vector<float> vals;
        std::stringstream ss(line);
        std::string cell;
        while (std::getline(ss, cell, ',')) {
            try { vals.push_back(std::stof(cell)); } catch (...) { break; }
        }
        if (vals.size() != 36) continue;
        CsvFrame f;
        for (int i = 0; i < 29; i++) f.joints[i] = vals[7 + i];
        frames.push_back(f);
    }
    std::cout << "Loaded " << frames.size() << " frames ("
              << frames.size() / 60.0f << "s)" << std::endl;
    return frames;
}

int main(int argc, char const* argv[]) {
    auto is_csv_path = [](const std::string& s) {
        return s.size() >= 4 && s.substr(s.size() - 4) == ".csv";
    };

    if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " <csv> [fps] [net]" << std::endl;
        std::cout << "   or: " << argv[0] << " <net> <csv> [fps]" << std::endl;
        std::cout << "Default net: eno0" << std::endl;
        return 1;
    }

    std::string net = "lo";
    std::string csv_path;
    float fps = 60.0f;

    // New mode: <csv> [fps] [net]
    if (is_csv_path(argv[1])) {
        csv_path = argv[1];
        if (argc >= 3) {
            try {
                fps = std::stof(argv[2]);
                if (argc >= 4) net = argv[3];
            } catch (...) {
                net = argv[2];
            }
        }
    } else {
        // Backward-compatible mode: <net> <csv> [fps]
        if (argc < 3 || !is_csv_path(argv[2])) {
            std::cout << "Usage: " << argv[0] << " <csv> [fps] [net]" << std::endl;
            std::cout << "   or: " << argv[0] << " <net> <csv> [fps]" << std::endl;
            std::cout << "Default net: eno0" << std::endl;
            return 1;
        }
        net = argv[1];
        csv_path = argv[2];
        if (argc >= 4) {
            try { fps = std::stof(argv[3]); } catch (...) {}
        }
    }

    auto frames = LoadCsv(csv_path);
    if (frames.empty()) return 1;

    // DDS init
    std::cout << "Connecting via " << net << "..." << std::endl;
    unitree::robot::ChannelFactory::Instance()->Init(0, net);

    auto arm_pub = std::make_shared<unitree::robot::ChannelPublisher<unitree_hg::msg::dds_::LowCmd_>>(kTopicArmSDK);
    arm_pub->InitChannel();

    unitree_hg::msg::dds_::LowState_ state_msg;
    std::atomic<bool> ok{false};
    std::mutex state_mutex;
    auto state_sub = std::make_shared<unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::LowState_>>(kTopicState);
    state_sub->InitChannel([&](const void* msg) {
        std::lock_guard<std::mutex> lk(state_mutex);
        memcpy(&state_msg, msg, sizeof(unitree_hg::msg::dds_::LowState_));
        ok = true;
    }, 1);

    auto t0 = std::chrono::steady_clock::now();
    while (!ok) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        if (std::chrono::duration<float>(std::chrono::steady_clock::now() - t0).count() > 5.0f) {
            std::cerr << "ERROR: No state after 5s." << std::endl;
            return 1;
        }
    }
    std::cout << "Connected." << std::endl;

    // 读取所有关节当前位置
    std::array<float, 17> cur{};     // 上肢
    std::array<float, 12> leg_cur{}; // 下肢
    {
        std::lock_guard<std::mutex> lk(state_mutex);
        for (int i = 0; i < 17; i++) cur[i] = state_msg.motor_state().at(kArmJoints[i]).q();
        for (int i = 0; i < 12; i++) leg_cur[i] = state_msg.motor_state().at(kLegJoints[i]).q();
    }
    std::cout << "Initial leg positions: ";
    for (int i = 0; i < 12; i++) std::cout << leg_cur[i] << " ";
    std::cout << std::endl;

    unitree_hg::msg::dds_::LowCmd_ cmd;
    float dt = 1.0f / fps;
    float transition_max_delta = kTransitionMaxVel / fps;
    auto sleep_us = std::chrono::microseconds(static_cast<int>(dt * 1000000));
    std::atomic<float> weight{0.0f};
    int engage_steps = std::max(1, static_cast<int>(1.0f / dt));
    float dw_engage = kFinalWeightTarget / static_cast<float>(engage_steps);

    // Monitor thread: 比较 CSV 目标位置 vs 实际关节位置
    std::array<float, 17> csv_target{};
    std::mutex csv_target_mutex;
    std::atomic<bool> monitor_running{true};

    auto monitor = std::thread([&]() {
        using namespace std::chrono_literals;
        int monitor_count = 0;
        while (monitor_running.load()) {
            std::array<float, 17> state_pos{};
            std::array<float, 12> leg_state{};
            {
                std::lock_guard<std::mutex> lk(state_mutex);
                for (int i = 0; i < 17; i++) state_pos[i] = state_msg.motor_state().at(kArmJoints[i]).q();
                for (int i = 0; i < 12; i++) leg_state[i] = state_msg.motor_state().at(kLegJoints[i]).q();
            }
            std::array<float, 17> target_copy{};
            {
                std::lock_guard<std::mutex> lk(csv_target_mutex);
                target_copy = csv_target;
            }
            float w = weight.load();

            // 上肢误差
            float max_diff = 0.0f;
            int worst_idx = -1;
            for (int i = 0; i < 17; i++) {
                float d = std::abs(state_pos[i] - target_copy[i]);
                if (d > max_diff) { max_diff = d; worst_idx = i; }
            }

            // 下肢误差（相对初始位置）
            float leg_max_diff = 0.0f;
            int leg_worst = -1;
            for (int i = 0; i < 12; i++) {
                float d = std::abs(leg_state[i] - leg_cur[i]);
                if (d > leg_max_diff) { leg_max_diff = d; leg_worst = i; }
            }

            if (max_diff > 0.1f || leg_max_diff > 0.05f || monitor_count % 10 == 0) {
                std::cout << "[MONITOR] arm_err=" << max_diff
                          << " joint=" << (worst_idx >= 0 ? std::to_string(kArmJoints[worst_idx]) : "N/A")
                          << " | leg_err=" << leg_max_diff
                          << " leg_joint=" << (leg_worst >= 0 ? std::to_string(kLegJoints[leg_worst]) : "N/A")
                          << " w=" << w << std::endl;
            }
            monitor_count++;
            std::this_thread::sleep_for(200ms);
        }
    });

    // send: only command upper body. The simulator's G1Bridge keeps lower-body
    // standing control active while blending arm_sdk commands.
    auto send = [&](const std::array<float, 17>& pos) {
        float w = weight.load();
        cmd.motor_cmd().at(kNotUsedJoint).q(w);

        // 上肢关节：跟踪目标位置
        for (int j = 0; j < 17; j++) {
            cmd.motor_cmd().at(kArmJoints[j]).q(pos[j]);
            cmd.motor_cmd().at(kArmJoints[j]).dq(0);
            cmd.motor_cmd().at(kArmJoints[j]).kp(kKp);
            cmd.motor_cmd().at(kArmJoints[j]).kd(kKd);
            cmd.motor_cmd().at(kArmJoints[j]).tau(0);
        }

        // Lower-body slots are intentionally left as no-op arm_sdk commands.
        for (int j = 0; j < 12; j++) {
            cmd.motor_cmd().at(kLegJoints[j]).q(0);
            cmd.motor_cmd().at(kLegJoints[j]).dq(0);
            cmd.motor_cmd().at(kLegJoints[j]).kp(0);
            cmd.motor_cmd().at(kLegJoints[j]).kd(0);
            cmd.motor_cmd().at(kLegJoints[j]).tau(0);
        }

        arm_pub->Write(cmd);
    };

    // Phase 1: Engage — weight 0 → 1.0
    std::cout << "Engaging (weight 0→" << kFinalWeightTarget << ")..." << std::endl;
    for (int i = 0; i < engage_steps; i++) {
        weight = std::clamp(weight.load() + dw_engage, 0.0f, 1.0f);
        send(cur);
        std::this_thread::sleep_for(sleep_us);
    }

    // Phase 2: Transition — 速度钳位平滑过渡到 CSV 首帧
    std::cout << "Transitioning..." << std::endl;
    std::array<float, 17> target{};
    for (int i = 0; i < 17; i++) target[i] = frames[0].joints[kArmJoints[i]];
    std::array<float, 17> cmd_pos = cur;

    for (int i = 0; i < (int)(2.0f / dt); i++) {
        for (int j = 0; j < 17; j++) {
            float d = std::clamp(target[j] - cmd_pos[j], -transition_max_delta, transition_max_delta);
            cmd_pos[j] += d;
        }
        {
            std::lock_guard<std::mutex> lk(csv_target_mutex);
            csv_target = target;
        }
        send(cmd_pos);
        std::this_thread::sleep_for(sleep_us);
    }

    // Phase 3: Replay
    float replay_max_delta = kReplayMaxVel / fps;
    std::cout << "Replaying " << frames.size() << " frames (vel clamp=" << kReplayMaxVel << " rad/s)..." << std::endl;
    auto start = std::chrono::steady_clock::now();
    for (size_t fi = 0; fi < frames.size(); fi++) {
        auto fs = std::chrono::steady_clock::now();

        std::array<float, 17> frame_target{};
        for (int j = 0; j < 17; j++) {
            frame_target[j] = frames[fi].joints[kArmJoints[j]];
        }

        for (int j = 0; j < 17; j++) {
            float d = std::clamp(frame_target[j] - cmd_pos[j], -replay_max_delta, replay_max_delta);
            cmd_pos[j] += d;
        }

        {
            std::lock_guard<std::mutex> lk(csv_target_mutex);
            csv_target = frame_target;
        }
        send(cmd_pos);

        if (fi % 60 == 0) {
            float t = std::chrono::duration<float>(std::chrono::steady_clock::now() - start).count();
            std::cout << "  " << fi << "/" << frames.size() << " t=" << t << "s" << std::endl;
        }
        auto el = std::chrono::steady_clock::now() - fs;
        if (el < sleep_us) std::this_thread::sleep_for(sleep_us - el);
    }
    float total = std::chrono::duration<float>(std::chrono::steady_clock::now() - start).count();
    std::cout << "Done: " << frames.size() << " frames in " << total << "s" << std::endl;

    // Phase 4: Disengage
    std::cout << "Disengaging..." << std::endl;
    // Phase 4a: 平滑回到初始姿态
    std::cout << "Returning to initial posture..." << std::endl;
    for (int i = 0; i < (int)(2.0f / dt); i++) {
        for (int j = 0; j < 17; j++) {
            float d = std::clamp(cur[j] - cmd_pos[j], -transition_max_delta, transition_max_delta);
            cmd_pos[j] += d;
        }
        {
            std::lock_guard<std::mutex> lk(csv_target_mutex);
            csv_target = cur;
        }
        send(cmd_pos);
        std::this_thread::sleep_for(sleep_us);
    }

    // Phase 4b: weight 1.0 → 0
    std::cout << "Disengaging (ramp down) ..." << std::endl;
    int disengage_steps = std::max(1, static_cast<int>(2.0f / dt));
    float dw_disengage = kFinalWeightTarget / static_cast<float>(disengage_steps);
    for (int i = 0; i < disengage_steps; i++) {
        weight = std::clamp(weight.load() - dw_disengage, 0.0f, 1.0f);
        send(cmd_pos);
        std::this_thread::sleep_for(sleep_us);
    }
    weight = 0.0f;
    send(cmd_pos);

    // stop monitor thread and join
    monitor_running = false;
    if (monitor.joinable()) monitor.join();

    std::cout << "Robot returned to built-in control." << std::endl;
    return 0;
}
