cmake_minimum_required(VERSION 3.15)
project(MujocoReplay)

find_package(glfw3 REQUIRED)

# Conan's generated CMake files use find_library() to locate Apple frameworks,
# and CMake caches that result. The replay binary can outlive Conan package
# revisions, so clear stale MuJoCo framework paths before resolving the target.
if(APPLE)
    unset(CONAN_FRAMEWORK_mujoco_FOUND CACHE)
endif()

find_package(mujoco REQUIRED)

add_executable(replay src/replay.cpp)
target_link_libraries(replay PRIVATE glfw)
target_link_libraries(replay PRIVATE mujoco::mujoco)

set_target_properties(replay PROPERTIES RUNTIME_OUTPUT_DIRECTORY_RELEASE ../../bin)

if(APPLE)
    set_target_properties(replay PROPERTIES
        BUILD_WITH_INSTALL_RPATH TRUE
        INSTALL_RPATH "@loader_path")

    add_custom_command(TARGET replay POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${CONAN_FRAMEWORK_mujoco_FOUND}"
            "$<TARGET_FILE_DIR:replay>/mujoco.framework")
endif()
