cmake_minimum_required(VERSION 2.8)

file(GLOB basilisk_src
    "*.h"
    "*.cpp"
    "*.c"
)

add_library(SimMessaging SHARED ${basilisk_src})
target_compile_options(SimMessaging PUBLIC "-fPIC")
target_include_directories(SimMessaging PUBLIC "$ENV{SIMULATION_BASE}/SimCode")
