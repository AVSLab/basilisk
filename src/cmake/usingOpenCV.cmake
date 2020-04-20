find_package(opencv CONFIG REQUIRED)

list(APPEND CUSTOM_DEPENDENCIES opencv::opencv)
set(CMAKE_INCLUDE_CURRENT_DIR TRUE)
if(APPLE)
  list(APPEND CUSTOM_DEPENDENCIES "-framework OpenCL")
endif()
