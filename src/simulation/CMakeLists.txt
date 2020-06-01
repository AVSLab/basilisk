cmake_minimum_required(VERSION 2.8)

#findAndCreateSwigModules("simulation/")
#add_subdirectory("utilities")
#add_subdirectory("architecture/messaging")

# The rationale for this is to explicitly specify packages that the user wants built.
# TODO: We might automate this, and leave the custom libraries to CMakeLists found within the automated search.
find_package_targets("${CMAKE_SOURCE_DIR}/simulation/dynamics" DYN_TARGETS) # Finds relative path of all .i files within the directory
generate_package_targets("${DYN_TARGETS}" "dynamicsLib;${ARCHITECTURE_LIBS}" "simulation") # The quotations around DYN_TARGETS informs function that it is a list

# TODO: Don't link cspice to all of the environment targets -- instead write a custom CMakeLists.txt file for spice_interface that includes it.
find_package_targets("${CMAKE_SOURCE_DIR}/simulation/environment" ENV_TARGETS) # Finds relative path of all .i files within the directory
generate_package_targets("${ENV_TARGETS}" "environmentLib;${ARCHITECTURE_LIBS};${library_dependencies}" "simulation") # Library Dependencies include cspice

find_package_targets("${CMAKE_SOURCE_DIR}/simulation/navigation" NAV_TARGETS)
generate_package_targets("${NAV_TARGETS}" "${ARCHITECTURE_LIBS};" "simulation")

find_package_targets("${CMAKE_SOURCE_DIR}/simulation/onboardDataHandling" DATA_TARGETS)
generate_package_targets("${DATA_TARGETS}" "onboardDataHandlingLib;${ARCHITECTURE_LIBS};" "simulation")

