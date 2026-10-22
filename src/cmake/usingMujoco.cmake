if(APPLE AND NOT TARGET mujoco::mujoco)
  # Conan caches framework lookup independently of the selected package version.
  # Refresh it before creating the imported target so headers and binaries agree.
  unset(CONAN_FRAMEWORK_mujoco_FOUND CACHE)
endif()

find_package(mujoco REQUIRED)

if (mujoco_FOUND)
  if(TARGET_NAME)
    target_link_libraries(${TARGET_NAME} PRIVATE mujoco::mujoco)
  endif ()
  if(LIB_NAME)
    target_link_libraries(${LIB_NAME} PUBLIC mujoco::mujoco)
  endif ()
else ()
  message(FATAL_ERROR "mujoco not found")
endif ()
