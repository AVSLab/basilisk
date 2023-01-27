option(BUILD_VIZINTERFACE "Build VizInterface Module" ON)
if(NOT BUILD_VIZINTERFACE)
  list(APPEND EXCLUDED_BSK_TARGETS "vizInterface")
endif()

option(BUILD_OPNAV "Build OpNav Modules" OFF)
if(NOT BUILD_OPNAV)
  list(APPEND EXCLUDED_BSK_TARGETS "limbFinding" "centerRadiusCNN" "houghCircles" "camera")
endif()
