cmake_minimum_required(VERSION 3.10)
project(g1_motion)

set(CMAKE_CXX_STANDARD 17)

# unitree_sdk2 (system installed at /opt/unitree_robotics)
list(APPEND CMAKE_PREFIX_PATH "/opt/unitree_robotics/lib/cmake")
find_package(unitree_sdk2 REQUIRED)

# csv_replay: keyframe playback tool
add_executable(csv_replay csv_replay.cpp)
target_link_libraries(csv_replay unitree_sdk2)

# state_recorder: record robot state during playback
add_executable(state_recorder state_recorder.cpp)
target_link_libraries(state_recorder unitree_sdk2)
