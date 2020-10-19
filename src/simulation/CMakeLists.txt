cmake_minimum_required(VERSION 3.14)

findAndCreateSwigModules("simulation/")
add_subdirectory("utilities")
add_subdirectory("architecture/messaging")
add_subdirectory("architecture/messaging/c_messages")
