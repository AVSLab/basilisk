cmake_minimum_required(VERSION 2.8)

message(STATUS "Adding Viz Interface module with Protobuffers")
findAndCreateSwigModules("simulation/viz_interface" TRUE)
