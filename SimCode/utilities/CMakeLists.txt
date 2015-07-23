cmake_minimum_required(VERSION 2.8)

file(GLOB basilisk_src
    "*.h"
    "*.cpp"
    "*.c"
)

add_library(SimUtilities STATIC ${basilisk_src})
target_compile_options(SimUtilities PUBLIC "-fPIC")
target_include_directories(SimUtilities PUBLIC "$ENV{SIMULATION_BASE}/SimCode")
