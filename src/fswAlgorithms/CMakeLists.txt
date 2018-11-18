cmake_minimum_required(VERSION 2.8)
if(POLICY CMP0078)
  cmake_policy(SET CMP0078 NEW)
endif()

find_package(SWIG REQUIRED)
include(${SWIG_USE_FILE})
find_package(PythonLibs)
include_directories(${PYTHON_INCLUDE_PATH})

set(CMAKE_SWIG_FLAGS "")

file(GLOB sub_directories RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/*)

string(FIND ${CMAKE_CURRENT_SOURCE_DIR} "/" DIR_NAME_START REVERSE)
math(EXPR DIR_NAME_START "${DIR_NAME_START} + 1")
string(SUBSTRING ${CMAKE_CURRENT_SOURCE_DIR} ${DIR_NAME_START} -1 DIR_NAME_STRING)

project("${DIR_NAME_STRING}")

file(GLOB_RECURSE swig_modules RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "*.i")
get_property(LIBRARY_BUILD_LIST GLOBAL PROPERTY BUILT_LIB_LIST)
get_property(ALG_LIST GLOBAL PROPERTY ALG_LIB_LIST)
get_property(INTER_FILES GLOBAL PROPERTY SYS_INTER_FILES)

file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/Basilisk/fswAlgorithms")
file(WRITE "${CMAKE_BINARY_DIR}/Basilisk/fswAlgorithms/__init__.py" "#empty init file")

foreach(module ${swig_modules})
    get_filename_component(FileBase ${module} NAME_WE)
    get_filename_component(FileDir ${module} DIRECTORY)
	set(CMAKE_SWIG_OUTDIR "${CMAKE_BINARY_DIR}/Basilisk/fswAlgorithms/${FileBase}")

    file(GLOB gen_files
        "../simulation/_GeneralModuleFiles/*.cpp"
        "../simulation/_GeneralModuleFiles/*.h"
        "../simulation/_GeneralModuleFiles/*.c")
    file(GLOB impl_files
	    "${FileDir}/*.cpp"
	    "${FileDir}/*.c"
	    "${FileDir}/*.h"
	    "${FileDir}/../_GeneralModuleFiles/*.cpp"
	    "${FileDir}/../_GeneralModuleFiles/*.h"
	    "${FileDir}/../_GeneralModuleFiles/*.c")
	file(GLOB cpp_files
	    "${FileDir}/*.cpp"
		"${FileDir}/../_GeneralModuleFiles/*.cpp")

	# Make a python package directory and add an __init__.py fiel or
	# copy an existing custom __init__.py file.
	set(moduleFilePath "${CMAKE_BINARY_DIR}/Basilisk/fswAlgorithms/${FileBase}")
	make_python_package(${FileBase} ${FileDir} ${moduleFilePath})

	set_source_files_properties(${module} PROPERTIES CPLUSPLUS OFF)
	foreach(localFile ${cpp_files})
	    set_source_files_properties(${module} PROPERTIES CPLUSPLUS ON)
	endforeach()

    file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/${FileDir})
	set_property(SOURCE ${module} PROPERTY SWIG_FLAGS "-I${CMAKE_SOURCE_DIR}/simulation/_GeneralModuleFiles" "-I${CMAKE_CURRENT_SOURCE_DIR}/${FileDir}/")
	set(SWIG_MODULE_${FileBase}_EXTRA_DEPS ${impl_files} ${gen_files} ${INTER_FILES})
    MAKE_SWIG_MODULE(${FileBase} python ${module} ${impl_files} ${gen_files})
    SWIG_LINK_LIBRARIES(${FileBase} ${PYTHON_LIBRARIES})

	foreach(LibFile ${ALG_LIST})
  	    SWIG_LINK_LIBRARIES(${FileBase} ${LibFile})
    endforeach()
    target_include_directories(${SWIG_MODULE_${FileBase}_REAL_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/${FileDir})
    set_target_properties(${SWIG_MODULE_${FileBase}_REAL_NAME} PROPERTIES
	FOLDER "${DIR_NAME_STRING}/${FileDir}")
    set_target_properties(${SWIG_MODULE_${FileBase}_REAL_NAME} PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/Basilisk/fswAlgorithms/${FileBase}")
    set_target_properties(${SWIG_MODULE_${FileBase}_REAL_NAME} PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY_DEBUG "${CMAKE_BINARY_DIR}/Basilisk/fswAlgorithms/${FileBase}")
    set_target_properties(${SWIG_MODULE_${FileBase}_REAL_NAME} PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY_RELEASE "${CMAKE_BINARY_DIR}/Basilisk/fswAlgorithms/${FileBase}")
    set_target_properties(${SWIG_MODULE_${FileBase}_REAL_NAME} PROPERTIES
              COMPILE_FLAGS "-I${CMAKE_SOURCE_DIR}/fswAlgorithms")
endforeach()

add_subdirectory("messaging")
