cmake_minimum_required(VERSION 2.8)

findAndCreateSwigModules("simulation/")
add_subdirectory("utilities")
add_subdirectory("architecture/messaging")

if (USE_PROTOBUFFERS)
	message(STATUS "Adding Viz Interface module with Protobuffers")
	add_subdirectory("viz_interface")
endif()
