cmake_minimum_required(VERSION 2.8)
cmake_policy(SET CMP0078 NEW)

findAndCreateSwigModules("simulation/")
add_subdirectory("utilities")
add_subdirectory("architecture/messaging")
