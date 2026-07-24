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
# docs/source/Learn/makingModules/rustModules.rst).
# and requires a Rust/Cargo toolchain that most Basilisk builds/CI images don't
# otherwise need. Off by default so no one is forced to install Rust just to
# build Basilisk; generate_rust_package_targets() (bskFindRustModules.cmake)
# is a no-op whenever this is OFF, regardless of what it discovers on disk.
option(BUILD_RUST_MODULES "Build Rust modules under src/ (requires cargo)" OFF)

# Evaluation path for replacing Basilisk's custom Cargo driver with Corrosion.
# It applies to discovered Rust modules and intentionally leaves the
# established path available for comparison and rollback.
option(BSK_RUST_USE_CORROSION
       "Build discovered Rust modules through the pinned Corrosion integration"
       OFF)
if(BSK_RUST_USE_CORROSION AND NOT BUILD_RUST_MODULES)
  message(FATAL_ERROR
    "BSK_RUST_USE_CORROSION requires BUILD_RUST_MODULES=ON")
endif()
