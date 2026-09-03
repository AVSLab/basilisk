# The fuelTank wrapper calls both thruster effector implementations. Give the
# package library ownership so all three wrappers link one compiled copy.
set(BSK_PACKAGE_LIBRARY_SOURCES
    "${CMAKE_CURRENT_LIST_DIR}/../Thrusters/thrusterDynamicEffector/thrusterDynamicEffector.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/../Thrusters/thrusterStateEffector/thrusterStateEffector.cpp")
