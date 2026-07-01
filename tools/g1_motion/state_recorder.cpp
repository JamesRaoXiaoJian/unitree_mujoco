// G1 Full-Process State Recorder
// Records robot joint states through the entire replay cycle:
//   2s idle (initial state) → CSV replay → disengage → 2s idle (final state)
// Output: CSV in same 36-column format as input keyframes, with header.
//
// Usage: ./state_recorder <csv> [fps] [net]
//        default: 60fps, eno0

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

static const std::string kTopicArmSDK = "rt/arm_sdk";
static const std::string kTopicState = "rt/lowstate";

// G1 joint indices
enum JointIndex {
    kLeftHipPitch, kLeftHipRoll, kLeftHipYaw, kLeftKnee, kLeftAnkle, kLeftAnkleRoll,
    kRightHipPitch, kRightHipRoll, kRightHipYaw, kRightKnee, kRightAnkle, kRightAnkleRoll,
    kWaistYaw, kWaistRoll, kWaistPitch,
    kLeftShoulderPitch, kLeftShoulderRoll, kLeftShoulderYaw, kLeftElbow,
    kLeftWristRoll, kLeftWristPitch, kLeftWristYaw,
    kRightShoulderPitch, kRightShoulderRoll, kRightShoulderYaw, kRightElbow,
    kRightWristRoll, kRightWristPitch, kRightWristYaw,
    kNotUsedJoint, kNotUsedJoint1, kNotUsedJoint2, kNotUsedJoint3, kNotUsedJoint4, kNotUsedJoint5
};

static const std::array<int, 17> kArmJoints = {
    15, 16, 17, 18, 19, 20, 21,
    22, 23, 24, 25, 26, 27, 28,
    12, 13, 14,
};
static const std::array<int, 12> kLegJoints = {
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11,
};

static constexpr float kKp = 120.0f;
static constexpr float kKd = 3.0f;
static constexpr float kLegKp = 20.0f;
static constexpr float kLegKd = 1.0f;
static constexpr float kTransitionMaxVel = 0.5f;
static constexpr float kReplayMaxVel = 0.8f;
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
    return frames;
}

// Read all 29 joint angles from state_msg
std::array<float, 29> ReadJoints(const unitree_hg::msg::dds_::LowState_& s) {
    std::array<float, 29> j{};
    for (int i = 0; i < 29; i++) j[i] = s.motor_state().at(i).q();
    return j;
}

int main(int argc, char const* argv[]) {
    if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " <csv> [fps] [net]" << std::endl;
        std::cout << "Records full process: 2s idle → replay → disengage → 2s idle" << std::endl;
        return 1;
    }

    std::string csv_path = argv[1];
    float fps = 60.0f;
    std::string net = "eno0";
    if (argc >= 3) {
        try { fps = std::stof(argv[2]); } catch (...) { net = argv[2]; }
    }
    if (argc >= 4) net = argv[3];

    auto frames = LoadCsv(csv_path);
    if (frames.empty()) return 1;

    std::string out_path = csv_path;
    // Replace .csv with _recorded.csv
    auto pos = out_path.rfind(".csv");
    if (pos != std::string::npos) out_path.replace(pos, 4, "_recorded.csv");
    else out_path += "_recorded.csv";

    std::cout << "Input:  " << csv_path << " (" << frames.size() << " frames)" << std::endl;
    std::cout << "Output: " << out_path << std::endl;
    std::cout << "FPS: " << fps << ", Net: " << net << std::endl;

    // DDS init
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

    // Read initial positions
    std::array<float, 17> cur{};
    std::array<float, 12> leg_cur{};
    {
        std::lock_guard<std::mutex> lk(state_mutex);
        for (int i = 0; i < 17; i++) cur[i] = state_msg.motor_state().at(kArmJoints[i]).q();
        for (int i = 0; i < 12; i++) leg_cur[i] = state_msg.motor_state().at(kLegJoints[i]).q();
    }

    // Open output
    std::ofstream ofs(out_path);
    if (!ofs.is_open()) {
        std::cerr << "ERROR: Cannot open " << out_path << std::endl;
        return 1;
    }
    ofs << std::fixed << std::setprecision(7);

    // Header
    ofs << "root_pos_x,root_pos_y,root_pos_z,"
        << "root_quat_x,root_quat_y,root_quat_z,root_quat_w,"
        << "j_pelvis,j_spine1,j_spine2,j_spine3,j_left_hip,j_left_knee,"
        << "j_left_ankle,j_right_hip,j_right_knee,j_right_ankle,"
        << "j_chest,j_neck,j_head,"
        << "j_left_shoulder,j_left_arm,j_left_fore_arm,j_left_hand,"
        << "j_right_shoulder,j_right_arm,j_right_fore_arm,j_right_hand,"
        << "j_left_finger,j_right_finger,"
        << "j_left_toe,j_right_toe,"
        << "j_left_foot,j_right_foot,"
        << "j_waist_yaw,j_waist_roll,j_waist_pitch"
        << std::endl;

    // Helper: write one frame from state_msg
    auto write_frame = [&]() {
        std::array<float, 29> j{};
        {
            std::lock_guard<std::mutex> lk(state_mutex);
            j = ReadJoints(state_msg);
        }
        for (int c = 0; c < 7; c++) ofs << "0,";
        for (int i = 0; i < 29; i++) {
            ofs << j[i];
            if (i < 28) ofs << ",";
        }
        ofs << "\n";
    };

    float dt = 1.0f / fps;
    float transition_max_delta = kTransitionMaxVel / fps;
    float replay_max_delta = kReplayMaxVel / fps;
    auto sleep_us = std::chrono::microseconds(static_cast<int>(dt * 1000000));
    int idle_frames = static_cast<int>(2.0f * fps);  // 2s idle at start and end
    int engage_steps = std::max(1, static_cast<int>(1.0f / dt));
    float dw_engage = kFinalWeightTarget / static_cast<float>(engage_steps);

    unitree_hg::msg::dds_::LowCmd_ cmd;
    std::atomic<float> weight{0.0f};

    auto send = [&](const std::array<float, 17>& pos) {
        float w = weight.load();
        cmd.motor_cmd().at(kNotUsedJoint).q(w);
        for (int j = 0; j < 17; j++) {
            cmd.motor_cmd().at(kArmJoints[j]).q(pos[j]);
            cmd.motor_cmd().at(kArmJoints[j]).dq(0);
            cmd.motor_cmd().at(kArmJoints[j]).kp(kKp);
            cmd.motor_cmd().at(kArmJoints[j]).kd(kKd);
            cmd.motor_cmd().at(kArmJoints[j]).tau(0);
        }
        for (int j = 0; j < 12; j++) {
            cmd.motor_cmd().at(kLegJoints[j]).q(leg_cur[j]);
            cmd.motor_cmd().at(kLegJoints[j]).dq(0);
            cmd.motor_cmd().at(kLegJoints[j]).kp(kLegKp);
            cmd.motor_cmd().at(kLegJoints[j]).kd(kLegKd);
            cmd.motor_cmd().at(kLegJoints[j]).tau(0);
        }
        arm_pub->Write(cmd);
    };

    int total_frames = idle_frames                    // initial idle
                     + engage_steps                   // engage
                     + static_cast<int>(2.0f * fps)   // transition
                     + static_cast<int>(frames.size()) // replay
                     + static_cast<int>(2.0f * fps)   // return to init
                     + static_cast<int>(2.0f * fps)   // disengage
                     + idle_frames;                    // final idle

    std::cout << "Recording " << total_frames << " frames (" << total_frames / fps << "s)..." << std::endl;
    int frame_count = 0;
    auto record_start = std::chrono::steady_clock::now();

    // === Phase 0: Initial idle (2s) — just stand there ===
    std::cout << "[Phase 0] Initial idle (2s)..." << std::endl;
    for (int i = 0; i < idle_frames; i++) {
        write_frame();
        frame_count++;
        std::this_thread::sleep_for(sleep_us);
    }

    // === Phase 1: Engage — weight 0 → 1.0 ===
    std::cout << "[Phase 1] Engaging..." << std::endl;
    for (int i = 0; i < engage_steps; i++) {
        weight = std::clamp(weight.load() + dw_engage, 0.0f, 1.0f);
        send(cur);
        write_frame();
        frame_count++;
        std::this_thread::sleep_for(sleep_us);
    }

    // === Phase 2: Transition — smooth to CSV first frame ===
    std::cout << "[Phase 2] Transitioning..." << std::endl;
    std::array<float, 17> target{};
    for (int i = 0; i < 17; i++) target[i] = frames[0].joints[kArmJoints[i]];
    std::array<float, 17> cmd_pos = cur;

    int transition_frames = static_cast<int>(2.0f * fps);
    for (int i = 0; i < transition_frames; i++) {
        for (int j = 0; j < 17; j++) {
            float d = std::clamp(target[j] - cmd_pos[j], -transition_max_delta, transition_max_delta);
            cmd_pos[j] += d;
        }
        send(cmd_pos);
        write_frame();
        frame_count++;
        std::this_thread::sleep_for(sleep_us);
    }

    // === Phase 3: Replay ===
    std::cout << "[Phase 3] Replaying " << frames.size() << " frames..." << std::endl;
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
        send(cmd_pos);
        write_frame();
        frame_count++;
        if (fi % 120 == 0) std::cout << "  " << fi << "/" << frames.size() << std::endl;
        auto el = std::chrono::steady_clock::now() - fs;
        if (el < sleep_us) std::this_thread::sleep_for(sleep_us - el);
    }

    // === Phase 4a: Return to initial posture ===
    std::cout << "[Phase 4a] Returning to initial posture..." << std::endl;
    for (int i = 0; i < transition_frames; i++) {
        for (int j = 0; j < 17; j++) {
            float d = std::clamp(cur[j] - cmd_pos[j], -transition_max_delta, transition_max_delta);
            cmd_pos[j] += d;
        }
        send(cmd_pos);
        write_frame();
        frame_count++;
        std::this_thread::sleep_for(sleep_us);
    }

    // === Phase 4b: Disengage — weight 1.0 → 0 ===
    std::cout << "[Phase 4b] Disengaging..." << std::endl;
    int disengage_frames = static_cast<int>(2.0f * fps);
    float dw_disengage = kFinalWeightTarget / static_cast<float>(disengage_frames);
    for (int i = 0; i < disengage_frames; i++) {
        weight = std::clamp(weight.load() - dw_disengage, 0.0f, 1.0f);
        send(cmd_pos);
        write_frame();
        frame_count++;
        std::this_thread::sleep_for(sleep_us);
    }
    weight = 0.0f;
    send(cmd_pos);

    // === Phase 5: Final idle (2s) — record final resting state ===
    std::cout << "[Phase 5] Final idle (2s)..." << std::endl;
    for (int i = 0; i < idle_frames; i++) {
        write_frame();
        frame_count++;
        std::this_thread::sleep_for(sleep_us);
    }

    ofs.close();
    float total_time = std::chrono::duration<float>(std::chrono::steady_clock::now() - record_start).count();
    std::cout << "Done: " << frame_count << " frames in " << total_time << "s" << std::endl;
    std::cout << "Saved to " << out_path << std::endl;

    return 0;
}
