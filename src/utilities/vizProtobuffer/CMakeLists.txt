cmake_minimum_required(VERSION 3.0)

if(BUILD_VIZINTERFACE)
    find_package(protobuf CONFIG REQUIRED)
    message(STATUS "Generating protobuffers")

    #file(GLOB PROTOBUF_DEFINITION_FILES "*.proto")
    #set(PROTOBUF_INPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/utilties/vizProto")
    #set(PROTOBUF_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/utilities/vizProtobuffer/")
    #foreach(file ${PROTOBUF_DEFINITION_FILES})
    #set(PROTOBUF_ARGUMENTS "protoc --proto_path=\"${PROTOBUF_INPUT_DIRECTORY}\" --cpp_out=\"${PROTOBUF_OUTPUT_DIRECTORY}\" \"${vizMessage.proto}\"")
    #execute_process(COMMAND ${PROTOBUF_OUTPUT_DIRECTORY}
    #endforeach()
    file(GLOB PROTO_SRC "vizMessage.pb.cc" "vizMessage.hpp")
    add_library(protocode ${PROTO_SRC})
    target_link_libraries(protocode protobuf::protobuf)

    set_target_properties(protocode PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/Basilisk")
    set_target_properties(protocode PROPERTIES LIBRARY_OUTPUT_DIRECTORY_DEBUG "${CMAKE_BINARY_DIR}/Basilisk")
    set_target_properties(protocode PROPERTIES LIBRARY_OUTPUT_DIRECTORY_RELEASE "${CMAKE_BINARY_DIR}/Basilisk")

    set_target_properties(protocode PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/Basilisk")
    set_target_properties(protocode PROPERTIES RUNTIME_OUTPUT_DIRECTORY_DEBUG "${CMAKE_BINARY_DIR}/Basilisk")
    set_target_properties(protocode PROPERTIES RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CMAKE_BINARY_DIR}/Basilisk")

    set_target_properties(protocode PROPERTIES ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/Basilisk")
    set_target_properties(protocode PROPERTIES ARCHIVE_OUTPUT_DIRECTORY_DEBUG "${CMAKE_BINARY_DIR}/Basilisk")
    set_target_properties(protocode PROPERTIES ARCHIVE_OUTPUT_DIRECTORY_RELEASE "${CMAKE_BINARY_DIR}/Basilisk")
endif()
