cmake_minimum_required(VERSION 2.8)

findAndCreateSwigModules("")

if (USE_PROTOBUFFERS)
	message(STATUS "Generating Protobuffers")
	add_subdirectory("vizProtobuffer")
endif()

