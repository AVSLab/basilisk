if(BUILD_OPNAV)
  #  find_package(opencv CONFIG REQUIRED)
  #
  #  set(CUSTOM_CMAKE_BUILD_TARGETS opencv::opencv)
  #
  #  set(CMAKE_INCLUDE_CURRENT_DIR TRUE)
  #  if(APPLE)
  #    link_libraries("-framework OpenCL")
  #  endif()
  include(usingOpenCV)

else()
  MESSAGE("SKIPPED: ${TARGET_NAME}")
  set(CUSTOM_BUILD_HANDLED 1)
endif()
