
if(BUILD_VIZINTERFACE)
    find_package(protobuf CONFIG REQUIRED)
    message(STATUS "Scanning ${CMAKE_CURRENT_SOURCE_DIR} for protobuf interface definitions")

     find_program(PROTOC_EXE
         NAMES protoc
         PATHS ENV
         PATH NO_DEFAULT_PATH
         REQUIRED
     )

     file(GLOB PROTO_DEFS "${CMAKE_CURRENT_SOURCE_DIR}/*.proto")

     foreach(PROTO_DEF ${PROTO_DEFS})
         get_filename_component(PROTO_TARGET "${PROTO_DEF}" NAME_WE)

         add_custom_command(
             OUTPUT "${CMAKE_CURRENT_SOURCE_DIR}/${PROTO_TARGET}.pb.cc" "${CMAKE_CURRENT_SOURCE_DIR}/${PROTO_TARGET}.pb.h" "${CMAKE_CURRENT_SOURCE_DIR}/${PROTO_TARGET}_pb2.py"
             COMMAND "${PROTOC_EXE}" "--proto_path=${CMAKE_CURRENT_SOURCE_DIR}" "--cpp_out=${CMAKE_CURRENT_SOURCE_DIR}" "${PROTO_DEF}"
             COMMAND "${PROTOC_EXE}" "--proto_path=${CMAKE_CURRENT_SOURCE_DIR}" "--python_out=${CMAKE_CURRENT_SOURCE_DIR}" "${PROTO_DEF}"
             DEPENDS "${PROTO_DEF}"
             WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
             COMMENT "Generating protobuf (C++, Python): ${PROTO_TARGET}"
         )

         message(STATUS "Defining protobuf interface library target: ${PROTO_TARGET}")

         add_library(${PROTO_TARGET}
             "${CMAKE_CURRENT_SOURCE_DIR}/${PROTO_TARGET}.pb.cc"
         )

         target_include_directories(${PROTO_TARGET}
             PUBLIC
                 "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>"
                 "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
         )

         target_link_libraries(${PROTO_TARGET}
             PUBLIC
                 protobuf::protobuf
         )

         set_target_properties(${PROTO_TARGET} PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/Basilisk")
         set_target_properties(${PROTO_TARGET} PROPERTIES LIBRARY_OUTPUT_DIRECTORY_DEBUG "${CMAKE_BINARY_DIR}/Basilisk")
         set_target_properties(${PROTO_TARGET} PROPERTIES LIBRARY_OUTPUT_DIRECTORY_RELEASE "${CMAKE_BINARY_DIR}/Basilisk")

         set_target_properties(${PROTO_TARGET} PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/Basilisk")
         set_target_properties(${PROTO_TARGET} PROPERTIES RUNTIME_OUTPUT_DIRECTORY_DEBUG "${CMAKE_BINARY_DIR}/Basilisk")
         set_target_properties(${PROTO_TARGET} PROPERTIES RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CMAKE_BINARY_DIR}/Basilisk")

         set_target_properties(${PROTO_TARGET} PROPERTIES ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/Basilisk")
         set_target_properties(${PROTO_TARGET} PROPERTIES ARCHIVE_OUTPUT_DIRECTORY_DEBUG "${CMAKE_BINARY_DIR}/Basilisk")
         set_target_properties(${PROTO_TARGET} PROPERTIES ARCHIVE_OUTPUT_DIRECTORY_RELEASE "${CMAKE_BINARY_DIR}/Basilisk")
     endforeach()
endif()
