
  find_package(protobuf CONFIG REQUIRED)
  list(APPEND CUSTOM_CMAKE_BUILD_TARGETS protobuf::protobuf)
  set(CMAKE_INCLUDE_CURRENT_DIR TRUE)
