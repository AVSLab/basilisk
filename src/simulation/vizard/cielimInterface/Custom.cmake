find_package(cppzmq CONFIG REQUIRED)
target_link_libraries(${TARGET_NAME} PRIVATE cppzmq::cppzmq)
target_link_libraries(${TARGET_NAME} PRIVATE vizMessage)
