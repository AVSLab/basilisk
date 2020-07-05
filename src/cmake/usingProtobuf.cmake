
  find_package(protobuf CONFIG REQUIRED)

  # The following code is added to work around conan issues with Xcode not seeing the release
  # version in the IDE and giving false errors
  get_property(INCLUDE_DIRECTORY TARGET protobuf::protobuf
               PROPERTY INTERFACE_INCLUDE_DIRECTORIES)
  if(DEFINED protobuf_INCLUDE_DIRS_DEBUG)
      set(DIRECTORIES "$<$<CONFIG:Debug>:${protobuf_INCLUDE_DIRS_DEBUG}>"
                                       "$<$<CONFIG:Release>:${protobuf_INCLUDE_DIRS_DEBUG}>"
                                       "$<$<CONFIG:RelWithDebInfo>:${protobuf_INCLUDE_DIRS_DEBUG}>"
                                       "$<$<CONFIG:MinSizeRel>:${protobuf_INCLUDE_DIRS_DEBUG}>"
                                       )
  else()
      set(DIRECTORIES "$<$<CONFIG:Debug>:${protobuf_INCLUDE_DIRS_RELEASE}>"
                                       "$<$<CONFIG:Release>:${protobuf_INCLUDE_DIRS_RELEASE}>"
                                       "$<$<CONFIG:RelWithDebInfo>:${protobuf_INCLUDE_DIRS_RELEASE}>"
                                       "$<$<CONFIG:MinSizeRel>:${protobuf_INCLUDE_DIRS_RELEASE}>")
  endif()
  set_property(TARGET protobuf::protobuf PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${DIRECTORIES})

  list(APPEND CUSTOM_DEPENDENCIES protobuf::protobuf)
  set(CMAKE_INCLUDE_CURRENT_DIR TRUE)
