
  find_package(protobuf CONFIG REQUIRED)
  list(APPEND CUSTOM_DEPENDENCIES protobuf::protobuf)
  set(CMAKE_INCLUDE_CURRENT_DIR TRUE)
