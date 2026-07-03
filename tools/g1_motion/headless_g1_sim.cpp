#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <filesystem>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include <mujoco/mujoco.h>
#include <unitree/robot/channel/channel_factory.hpp>

#include "param.h"
#include "unitree_sdk2_bridge.h"

namespace {

std::atomic<bool> g_stop{false};

void HandleSignal(int) {
    g_stop = true;
}

struct Rpy {
    double roll;
    double pitch;
    double yaw;
};

enum class ControlMode {
    Bridge,
    Direct,
    Sensor,
};

const char* ModeName(ControlMode mode) {
    switch (mode) {
        case ControlMode::Bridge:
            return "bridge";
        case ControlMode::Direct:
            return "direct";
        case ControlMode::Sensor:
            return "sensor";
    }
    return "unknown";
}

Rpy QuatToRpy(const mjtNum* q) {
    const double w = q[0];
    const double x = q[1];
    const double y = q[2];
    const double z = q[3];
    const double pitch_arg = std::clamp(2.0 * (w * y - z * x), -1.0, 1.0);
    return {
        std::atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y)),
        std::asin(pitch_arg),
        std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)),
    };
}

std::filesystem::path ResolveScenePath(const std::string& scene_arg) {
    std::filesystem::path scene(scene_arg);
    if (scene.is_absolute()) {
        return scene;
    }
    return std::filesystem::current_path() / scene;
}

}  // namespace

int main(int argc, char** argv) {
    std::string scene = "unitree_robots/g1/scene_29dof.xml";
    double duration = 20.0;
    std::string network = "lo";
    int domain_id = 0;
    ControlMode mode = ControlMode::Bridge;

    if (argc >= 2) scene = argv[1];
    if (argc >= 3) duration = std::stod(argv[2]);
    if (argc >= 4) network = argv[3];
    if (argc >= 5) domain_id = std::stoi(argv[4]);
    if (argc >= 6) {
        const std::string mode_arg = argv[5];
        if (mode_arg == "direct") {
            mode = ControlMode::Direct;
        } else if (mode_arg == "sensor") {
            mode = ControlMode::Sensor;
        }
    }

    std::signal(SIGINT, HandleSignal);
    std::signal(SIGTERM, HandleSignal);

    const auto scene_path = ResolveScenePath(scene);
    char load_error[1024] = "";
    mjModel* model = mj_loadXML(scene_path.c_str(), nullptr, load_error, sizeof(load_error));
    if (!model) {
        std::cerr << "[headless_g1_sim] Failed to load scene: " << scene_path << "\n"
                  << load_error << std::endl;
        return 1;
    }

    mjData* data = mj_makeData(model);
    if (!data) {
        std::cerr << "[headless_g1_sim] Failed to allocate mjData" << std::endl;
        mj_deleteModel(model);
        return 1;
    }

    if (model->nkey > 0) {
        mj_resetDataKeyframe(model, data, 0);
        std::cout << "[headless_g1_sim] Loaded keyframe 0, nq=" << model->nq << std::endl;
    }
    mj_forward(model, data);

    param::config.robot = "g1";
    param::config.robot_scene = scene_path;
    param::config.domain_id = domain_id;
    param::config.interface = network;
    param::config.use_joystick = 0;
    param::config.joystick_type = "xbox";
    param::config.joystick_device = "/dev/input/js0";
    param::config.joystick_bits = 16;
    param::config.print_scene_information = 0;
    param::config.enable_elastic_band = 0;

    std::unique_ptr<G1Bridge> bridge;
    if (mode == ControlMode::Bridge) {
        unitree::robot::ChannelFactory::Instance()->Init(domain_id, network);
        bridge = std::make_unique<G1Bridge>(model, data);
    }

    static constexpr std::array<double, 29> kKp = {
        80, 80, 60, 120, 50, 50,
        80, 80, 60, 120, 50, 50,
        60, 40, 40,
        40, 40, 40, 40, 40, 40, 40,
        40, 40, 40, 40, 40, 40, 40,
    };
    static constexpr std::array<double, 29> kKd = {
        2, 2, 2, 3, 1.5, 1.5,
        2, 2, 2, 3, 1.5, 1.5,
        2, 1.5, 1.5,
        1, 1, 1, 1, 1, 1, 1,
        1, 1, 1, 1, 1, 1, 1,
    };
    std::array<double, 29> target_q{};
    for (int i = 0; i < std::min(model->nu, 29); ++i) {
        target_q[i] = mode == ControlMode::Sensor ? data->sensordata[i] : data->qpos[7 + i];
    }

    std::cout << "[headless_g1_sim] Running scene=" << scene_path
              << " duration=" << duration
              << " domain_id=" << domain_id
              << " interface=" << network
              << " mode=" << ModeName(mode)
              << " timestep=" << model->opt.timestep << std::endl;

    bool fell = false;
    double max_abs_tilt = 0.0;
    double min_height = data->qpos[2];
    auto next_log = 0.0;
    const auto wall_start = std::chrono::steady_clock::now();

    while (!g_stop && data->time < duration) {
        if (mode == ControlMode::Direct) {
            for (int i = 0; i < std::min(model->nu, 29); ++i) {
                data->ctrl[i] = 8.0 * kKp[i] * (target_q[i] - data->qpos[7 + i]) +
                                8.0 * kKd[i] * (0.0 - data->qvel[6 + i]);
            }
        } else if (mode == ControlMode::Sensor) {
            for (int i = 0; i < std::min(model->nu, 29); ++i) {
                data->ctrl[i] = 8.0 * kKp[i] * (target_q[i] - data->sensordata[i]) +
                                8.0 * kKd[i] * (0.0 - data->sensordata[i + model->nu]);
            }
        } else {
            bridge->run();
        }
        mj_step(model, data);
        mj_forward(model, data);

        const auto rpy = QuatToRpy(&data->qpos[3]);
        const double tilt = std::max(std::abs(rpy.roll), std::abs(rpy.pitch));
        max_abs_tilt = std::max(max_abs_tilt, tilt);
        min_height = std::min(min_height, static_cast<double>(data->qpos[2]));
        if (data->qpos[2] < 0.55 || tilt > 0.8) {
            fell = true;
        }

        if (data->time >= next_log) {
            std::cout << "[headless_g1_sim] t=" << data->time
                      << " z=" << data->qpos[2]
                      << " rpy=(" << rpy.roll << "," << rpy.pitch << "," << rpy.yaw << ")"
                      << " ctrl0=" << (model->nu > 0 ? data->ctrl[0] : 0.0)
                      << " source=" << (mode == ControlMode::Bridge ? bridge->controlSourceName() : ModeName(mode))
                      << std::endl;
            next_log += 2.0;
        }

        const auto target_wall = wall_start + std::chrono::duration<double>(data->time);
        std::this_thread::sleep_until(target_wall);
    }

    const auto rpy = QuatToRpy(&data->qpos[3]);
    std::cout << "[headless_g1_sim] summary: fell=" << (fell ? "true" : "false")
              << " final_time=" << data->time
              << " final_z=" << data->qpos[2]
              << " min_z=" << min_height
              << " final_rpy=(" << rpy.roll << "," << rpy.pitch << "," << rpy.yaw << ")"
              << " max_abs_tilt=" << max_abs_tilt
              << std::endl;

    bridge.reset();
    mj_deleteData(data);
    mj_deleteModel(model);
    return fell ? 2 : 0;
}
