cmake_minimum_required(VERSION 3.16)
project(g1_stand_example)

set(CMAKE_CXX_STANDARD 17)

list(APPEND CMAKE_PREFIX_PATH "/opt/unitree_robotics/lib/cmake")
find_package(unitree_sdk2 REQUIRED)

add_executable(g1_stand g1_stand.cpp)
target_link_libraries(g1_stand unitree_sdk2)

add_executable(g1_stand_debug g1_stand_debug.cpp)
target_link_libraries(g1_stand_debug unitree_sdk2)
