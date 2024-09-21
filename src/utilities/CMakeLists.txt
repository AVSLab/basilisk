
#findAndCreateSwigModules("") Not confident what this was doing


if (BUILD_VIZINTERFACE)
	message(STATUS "Generating Protobuffers")
	add_subdirectory("vizProtobuffer")
endif()
