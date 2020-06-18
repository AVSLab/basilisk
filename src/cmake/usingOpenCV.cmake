find_package(opencv CONFIG REQUIRED)

# The following code is added to work around conan issues with Xcode not seeing the release
# version in the IDE and giving false errors
get_property(INCLUDE_DIRECTORY TARGET opencv::opencv
             PROPERTY INTERFACE_INCLUDE_DIRECTORIES)
if(DEFINED opencv_INCLUDE_DIRS_DEBUG)
    set(DIRECTORIES "$<$<CONFIG:Debug>:${opencv_INCLUDE_DIRS_DEBUG}>"
                                     "$<$<CONFIG:Release>:${opencv_INCLUDE_DIRS_DEBUG}>"
                                     "$<$<CONFIG:RelWithDebInfo>:${opencv_INCLUDE_DIRS_DEBUG}>"
                                     "$<$<CONFIG:MinSizeRel>:${opencv_INCLUDE_DIRS_DEBUG}>"
                                     )
else()
    set(DIRECTORIES "$<$<CONFIG:Debug>:${opencv_INCLUDE_DIRS_RELEASE}>"
                                     "$<$<CONFIG:Release>:${opencv_INCLUDE_DIRS_RELEASE}>"
                                     "$<$<CONFIG:RelWithDebInfo>:${opencv_INCLUDE_DIRS_RELEASE}>"
                                     "$<$<CONFIG:MinSizeRel>:${opencv_INCLUDE_DIRS_RELEASE}>")
endif()
set_property(TARGET opencv::opencv PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${DIRECTORIES})


list(APPEND CUSTOM_DEPENDENCIES opencv::opencv)

set(CMAKE_INCLUDE_CURRENT_DIR TRUE)
if(APPLE)
  list(APPEND CUSTOM_DEPENDENCIES "-framework OpenCL")
endif()
