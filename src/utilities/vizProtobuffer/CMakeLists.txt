cmake_minimum_required(VERSION 2.8)

message(STATUS "Generating protobuffers")

message(STATUS "PROTOBUF_FOUND: ${PROTOBUF_FOUND}")

include_directories(${CMAKE_CURRENT_BINARY_DIR})

#protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS vizMessage.proto)



##file(GLOB PROTOBUF_DEFINITION_FILES "*.proto")
#set(PROTOBUF_INPUT_DIRECTORY "${PROJECT_SOURCE_DIR}")
#set(PROTOBUF_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/utilities/vizProtobuffer/")
##foreach(file ${PROTOBUF_DEFINITION_FILES})
#set(PROTOBUF_ARGUMENTS "protoc --proto_path=\"${PROTOBUF_INPUT_DIRECTORY}\" --cpp_out=\"${PROTOBUF_OUTPUT_DIRECTORY}\" \"${vizMessage.proto}\"")
#execute_process(COMMAND ${PROTOBUF_OUTPUT_DIRECTORY}
#WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
#RESULT_VARIABLE PROTOBUF_RESULT
#OUTPUT_VARIABLE PROTOBUF_OUTPUT_VARIABLE)
##endforeach()
#file(GLOB PROTOBUF_MODELS_INCLUDES "vizMessage.pb.cc" "vizMessage.hpp")
