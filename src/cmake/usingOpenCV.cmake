find_package(OpenCV REQUIRED)
if (OpenCV_FOUND)
  set_property(TARGET opencv::opencv PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${DIRECTORIES})
  target_link_libraries(${TARGET_NAME} PRIVATE opencv::opencv)

  # The following code is added to work around conan issues with Xcode not seeing the release version in the IDE and
  # giving false errors
  get_property(
    INCLUDE_DIRECTORY
    TARGET opencv::opencv
    PROPERTY INTERFACE_INCLUDE_DIRECTORIES)
  if(DEFINED opencv_INCLUDE_DIRS_DEBUG)
    set(DIRECTORIES
            "$<$<CONFIG:Debug>:${opencv_INCLUDE_DIRS_DEBUG}>"
            "$<$<CONFIG:Release>:${opencv_INCLUDE_DIRS_DEBUG}>"
            "$<$<CONFIG:RelWithDebInfo>:${opencv_INCLUDE_DIRS_DEBUG}>"
            "$<$<CONFIG:MinSizeRel>:${opencv_INCLUDE_DIRS_DEBUG}>")
  else()
    set(DIRECTORIES
            "$<$<CONFIG:Debug>:${opencv_INCLUDE_DIRS_RELEASE}>"
            "$<$<CONFIG:Release>:${opencv_INCLUDE_DIRS_RELEASE}>"
            "$<$<CONFIG:RelWithDebInfo>:${opencv_INCLUDE_DIRS_RELEASE}>"
            "$<$<CONFIG:MinSizeRel>:${opencv_INCLUDE_DIRS_RELEASE}>")
  endif()
else ()
  message(FATAL_ERROR "opencv not found")
endif ()

if(APPLE)
  find_package(OpenCL REQUIRED)
  find_library(OPENCL_LIBRARY OpenCL REQUIRED)
  if (OPENCL_FOUND)
    include_directories(${OPENCL_INCLUDE_DIR})
    target_link_libraries(${TARGET_NAME} PUBLIC ${OPENCL_LIBRARY})
    # target_link_libraries(${TARGET_NAME} PUBLIC OpenCL::OpenCL)
  else ()
    message(FATAL_ERROR "OpenCL not found")
  endif ()
endif()
