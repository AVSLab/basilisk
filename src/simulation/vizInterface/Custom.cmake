if(BUILD_VIZINTERFACE)
  #find_package(protobuf CONFIG REQUIRED)
  #find_package(zmq CONFIG REQUIRED)

  #list(APPEND CUSTOM_CMAKE_BUILD_TARGETS zmq::zmq)
  #list(APPEND CUSTOM_CMAKE_BUILD_TARGETS protobuf::protobuf)

  #set(CMAKE_INCLUDE_CURRENT_DIR TRUE)

  include(usingZMQ)
  include(usingProtobuf)
else()
  MESSAGE("SKIPPED: ${TARGET_NAME}")
  set(CUSTOM_BUILD_HANDLED 1)
endif()
