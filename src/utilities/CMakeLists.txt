
#findAndCreateSwigModules("") Not confident what this was doing


if (BUILD_VIZINTERFACE)
	message(VERBOSE "Generating Protobuf interfaces")
	add_subdirectory("vizProtobuffer")
endif()
