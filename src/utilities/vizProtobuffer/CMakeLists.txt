cmake_minimum_required(VERSION 3.14)

if(BUILD_VIZINTERFACE)
     find_package(Protobuf REQUIRED)
     message(STATUS "Scanning ${CMAKE_CURRENT_SOURCE_DIR} for protobuf interface definitions")

     add_custom_target(GeneratedProtobufMessages
        DEPENDS protobuf::protoc
     )

     get_property(PROTOC_EXE TARGET protobuf::protoc PROPERTY LOCATION)

     # XXX: Confirm that the protoc version found is correct. This is just a
     # sanity check due to some issues where the Conan protobuf recipe finds
     # the system-installed protoc instead of the Conan-provided protoc.
     # See https://github.com/conan-io/conan-center-index/issues/21737
     execute_process(COMMAND ${PROTOC_EXE} --version OUTPUT_VARIABLE REAL_PROTOC_VERSION_STRING OUTPUT_STRIP_TRAILING_WHITESPACE)
     string(REPLACE " " ";" REAL_PROTOC_VERSION_LIST ${REAL_PROTOC_VERSION_STRING})
     list(GET REAL_PROTOC_VERSION_LIST 1 REAL_PROTOC_VERSION)
     if(NOT ("${Protobuf_VERSION}" VERSION_EQUAL "${REAL_PROTOC_VERSION}"))
        message(FATAL_ERROR "Protobuf compiler found (${PROTOC_EXE}) is version ${REAL_PROTOC_VERSION}, but expected to find ${Protobuf_VERSION}. Please contact the maintainers.")
     endif()

     file(GLOB PROTO_DEFS "${CMAKE_CURRENT_SOURCE_DIR}/*.proto")

     foreach(PROTO_DEF ${PROTO_DEFS})
         get_filename_component(PROTO_TARGET "${PROTO_DEF}" NAME_WE)

         # XXX: In some cases on Windows, protoc does not work if paths are absolute, so make them relative!
         file(RELATIVE_PATH PROTO_DEF ${CMAKE_CURRENT_SOURCE_DIR} ${PROTO_DEF})

         add_custom_command(TARGET GeneratedProtobufMessages PRE_BUILD
             COMMAND "${PROTOC_EXE}" "--proto_path=." "--cpp_out=." "${PROTO_DEF}"
             BYPRODUCTS "${CMAKE_CURRENT_SOURCE_DIR}/${PROTO_TARGET}.pb.cc" "${CMAKE_CURRENT_SOURCE_DIR}/${PROTO_TARGET}.pb.h"
             WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
             COMMENT "Generating protobuf (C++): ${PROTO_TARGET}"
         )

         message(STATUS "Defining protobuf interface library target: ${PROTO_TARGET}")

         add_library(${PROTO_TARGET}
             "${CMAKE_CURRENT_SOURCE_DIR}/${PROTO_TARGET}.pb.cc"
         )
         add_dependencies(${PROTO_TARGET} GeneratedProtobufMessages)
         target_include_directories(${PROTO_TARGET}
             PUBLIC
                 "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>"
                 "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
         )

         target_link_libraries(${PROTO_TARGET}
             PUBLIC
                 protobuf::protobuf
         )

         set_target_properties(${PROTO_TARGET} PROPERTIES
             CXX_STANDARD 11
             POSITION_INDEPENDENT_CODE ON
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
