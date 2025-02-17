cmake_minimum_required(VERSION 3.15)
project(MujocoReplay)

find_package(mujoco REQUIRED)
find_package(glfw3 REQUIRED)

add_executable(replay src/replay.cpp)
target_link_libraries(replay PRIVATE glfw)
target_link_libraries(replay PRIVATE mujoco::mujoco)

set_target_properties(replay PROPERTIES RUNTIME_OUTPUT_DIRECTORY_RELEASE ../../bin)
