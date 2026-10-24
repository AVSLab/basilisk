find_package(cppzmq CONFIG REQUIRED QUIET)
target_link_libraries(${TARGET_NAME} PRIVATE cppzmq::cppzmq)
target_link_libraries(${TARGET_NAME} PRIVATE vizMessage)
