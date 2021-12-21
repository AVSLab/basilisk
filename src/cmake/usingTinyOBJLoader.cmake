find_package(tinyobjloader CONFIG REQUIRED)

# The following code is added to work around conan issues with Xcode not seeing the release
# version in the IDE and giving false errors
get_property(INCLUDE_DIRECTORY TARGET tinyobjloader::tinyobjloader
             PROPERTY INTERFACE_INCLUDE_DIRECTORIES)
if(DEFINED tinyobjloader_INCLUDE_DIRS_DEBUG)
    set(DIRECTORIES "$<$<CONFIG:Debug>:${tinyobjloader_INCLUDE_DIRS_DEBUG}>"
                                     "$<$<CONFIG:Release>:${tinyobjloader_INCLUDE_DIRS_DEBUG}>"
                                     "$<$<CONFIG:RelWithDebInfo>:${tinyobjloader_INCLUDE_DIRS_DEBUG}>"
                                     "$<$<CONFIG:MinSizeRel>:${tinyobjloader_INCLUDE_DIRS_DEBUG}>"
                                     )
else()
    set(DIRECTORIES "$<$<CONFIG:Debug>:${tinyobjloader_INCLUDE_DIRS_RELEASE}>"
                                     "$<$<CONFIG:Release>:${tinyobjloader_INCLUDE_DIRS_RELEASE}>"
                                     "$<$<CONFIG:RelWithDebInfo>:${tinyobjloader_INCLUDE_DIRS_RELEASE}>"
                                     "$<$<CONFIG:MinSizeRel>:${tinyobjloader_INCLUDE_DIRS_RELEASE}>")
endif()
set_property(TARGET tinyobjloader::tinyobjloader PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${DIRECTORIES})


list(APPEND CUSTOM_DEPENDENCIES tinyobjloader::tinyobjloader)

set(CMAKE_INCLUDE_CURRENT_DIR TRUE)
