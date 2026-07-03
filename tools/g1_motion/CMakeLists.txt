cmake_minimum_required(VERSION 3.10)
project(g1_motion)

set(CMAKE_CXX_STANDARD 17)

# unitree_sdk2 (system installed at /opt/unitree_robotics)
list(APPEND CMAKE_PREFIX_PATH "/opt/unitree_robotics/lib/cmake")
find_package(unitree_sdk2 REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options)

get_filename_component(REPO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
set(MUJOCO_DIR "${REPO_ROOT}/simulate/mujoco")

include_directories(
  "${REPO_ROOT}/simulate/src"
  "${MUJOCO_DIR}/include"
  "${MUJOCO_DIR}/simulate"
)
link_directories("${MUJOCO_DIR}/lib")

# csv_replay: keyframe playback tool
add_executable(csv_replay csv_replay.cpp)
target_link_libraries(csv_replay unitree_sdk2)

# state_recorder: record robot state during playback
add_executable(state_recorder state_recorder.cpp)
target_link_libraries(state_recorder unitree_sdk2)

# headless_g1_sim: run the same G1Bridge without the GLFW viewer for CI/DDS checks
add_executable(
  headless_g1_sim
  headless_g1_sim.cpp
  "${REPO_ROOT}/simulate/src/joystick/joystick.cc"
)
target_link_libraries(
  headless_g1_sim
  unitree_sdk2
  mujoco
  yaml-cpp
  Boost::program_options
  fmt
  pthread
)
set_target_properties(
  headless_g1_sim PROPERTIES
  BUILD_RPATH "${MUJOCO_DIR}/lib"
)
