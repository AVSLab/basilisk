option(BUILD_VIZINTERFACE "Build VizInterface Module" ON)
if(NOT BUILD_VIZINTERFACE)
  list(APPEND EXCLUDED_BSK_TARGETS "vizInterface")
endif()

option(BUILD_OPNAV "Build OpNav Modules" OFF)
if(NOT BUILD_OPNAV)
  list(APPEND EXCLUDED_BSK_TARGETS "limbFinding" "centerRadiusCNN" "houghCircles" "camera")
endif()

option(BUILD_MUJOCO "Build MuJoCo Modules" OFF)
if(NOT BUILD_MUJOCO)
  list(APPEND EXCLUDED_BSK_LIBRARIES "mujocoDynamics")
endif()

# Rust module support is experimental (see
# docs/source/Learn/makingModules/rustModules.rst) and requires a Rust/Cargo
# toolchain that most Basilisk builds/CI images don't
# otherwise need. Off by default so no one is forced to install Rust just to
# build Basilisk; generate_rust_package_targets() (bskFindRustModules.cmake)
# is a no-op whenever this is OFF, regardless of what it discovers on disk.
option(BUILD_RUST_MODULES "Build Rust modules under src/ (requires cargo)" OFF)
