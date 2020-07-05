  find_package(zmq CONFIG REQUIRED)

  # The following code is added to work around conan issues with Xcode not seeing the release
  # version in the IDE and giving false errors
  get_property(INCLUDE_DIRECTORY TARGET zmq::zmq
               PROPERTY INTERFACE_INCLUDE_DIRECTORIES)
  if(DEFINED zmq_INCLUDE_DIRS_DEBUG)
      set(DIRECTORIES "$<$<CONFIG:Debug>:${zmq_INCLUDE_DIRS_DEBUG}>"
                                       "$<$<CONFIG:Release>:${zmq_INCLUDE_DIRS_DEBUG}>"
                                       "$<$<CONFIG:RelWithDebInfo>:${zmq_INCLUDE_DIRS_DEBUG}>"
                                       "$<$<CONFIG:MinSizeRel>:${zmq_INCLUDE_DIRS_DEBUG}>"
                                       )
  else()
      set(DIRECTORIES "$<$<CONFIG:Debug>:${zmq_INCLUDE_DIRS_RELEASE}>"
                                       "$<$<CONFIG:Release>:${zmq_INCLUDE_DIRS_RELEASE}>"
                                       "$<$<CONFIG:RelWithDebInfo>:${zmq_INCLUDE_DIRS_RELEASE}>"
                                       "$<$<CONFIG:MinSizeRel>:${zmq_INCLUDE_DIRS_RELEASE}>")
  endif()
  set_property(TARGET zmq::zmq PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${DIRECTORIES})

  list(APPEND CUSTOM_DEPENDENCIES zmq::zmq)
  set(CMAKE_INCLUDE_CURRENT_DIR TRUE)
