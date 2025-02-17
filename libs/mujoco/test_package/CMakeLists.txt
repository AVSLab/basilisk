cmake_minimum_required(VERSION 3.15)
project(PackageTest)

find_package(mujoco REQUIRED)

add_executable(test_mujoco src/main.cpp)
target_link_libraries(test_mujoco mujoco::mujoco)
