find_package(mujoco REQUIRED)

if (mujoco_FOUND)
  target_link_libraries(${TARGET_NAME} PRIVATE mujoco::mujoco)
else ()
  message(FATAL_ERROR "mujoco not found")
endif ()
