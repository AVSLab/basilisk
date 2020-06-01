cmake_minimum_required(VERSION 2.8)

#findAndCreateSwigModules("") # "" means install at top level
generate_package_targets("${CMAKE_SOURCE_DIR}/topLevelModules/pyswice/pyswice.i" "${ARCHITECTURE_LIBS}" "topLevelModules")