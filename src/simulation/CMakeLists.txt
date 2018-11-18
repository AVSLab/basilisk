cmake_minimum_required(VERSION 2.8)

findAndCreateSwigModules("simulation/")
add_subdirectory("utilities")
add_subdirectory("architecture/messaging")
