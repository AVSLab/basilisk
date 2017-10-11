cmake_minimum_required(VERSION 2.8)

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

file(MAKE_DIRECTORY "${CMAKE_SOURCE_DIR}/../dist/Basilisk/simulation")
file(WRITE "${CMAKE_SOURCE_DIR}/../dist/Basilisk/simulation/__init__.py" "#empty init file")

foreach(module ${swig_modules})
    get_filename_component(FileBase ${module} NAME_WE)
    get_filename_component(FileDir ${module} DIRECTORY)
	set(CMAKE_SWIG_OUTDIR "${CMAKE_SOURCE_DIR}/../dist/Basilisk/simulation/${FileBase}")
	if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${FileDir}/CMakeLists.txt")
        continue()
    endif()
    file(GLOB impl_files
	    "${FileDir}/*.cpp"
	    "${FileDir}/*.c"
	    "${FileDir}/*.h"
	)
    set(gen_files "")
    set(swig_dep_files "")
    file(GLOB gen_files
        "_GeneralModuleFiles/*.cpp"
        "_GeneralModuleFiles/*.h"
        "_GeneralModuleFiles/*.c")
    file(GLOB swig_dep_files
        "_GeneralModuleFiles/*.i")
    string(REPLACE "/" ";" DirList ${FileDir})
    list(LENGTH DirList dirLength)
    set(currentDir "")
    set(endDir "")
    foreach(localDir ${DirList})
        set(currentDir "${currentDir}${localDir}/")
        file(GLOB gen_files_loc "${currentDir}_GeneralModuleFiles/*.cpp"
            "${currentDir}_GeneralModuleFiles/*.h"
            "${currentDir}_GeneralModuleFiles/*.c")
        file(GLOB swig_dep_loc "${currentDir}_GeneralModuleFiles/*.i")
        set(gen_files "${gen_files};${gen_files_loc}")
        set(swig_dep_files "${swig_dep_files};${swig_dep_loc}")
    set(endDir ${localDir})
    endforeach()
    if("${endDir}" STREQUAL "_GeneralModuleFiles")
        continue()
    endif()

	# Make a python package
	set(moduleFilePath "${CMAKE_SOURCE_DIR}/../dist/Basilisk/simulation/${FileBase}")
	make_python_package(${FileBase} ${FileDir} ${moduleFilePath})

    set_source_files_properties(${module} PROPERTIES CPLUSPLUS ON)
	set_property(SOURCE ${module} PROPERTY SWIG_FLAGS "-I${CMAKE_SOURCE_DIR}/SimCode/_GeneralModuleFiles" "-I${CMAKE_CURRENT_SOURCE_DIR}/${FileDir}/")
	# MESSAGE("${SWIG_FLAGS}")
    #set_source_files_properties(${module} PROPERTIES SWIG_FLAGS "-I${CMAKE_CURRENT_SOURCE_DIR}/${FileDir}/")
	set_source_files_properties( ${swig_generated_file_fullname}
               PROPERTIES COMPILE_FLAGS "-I${CMAKE_SOURCE_DIR}/SimCode")
    set(SWIG_MODULE_${FileBase}_EXTRA_DEPS ${impl_files} ${gen_files} ${swig_dep_files} ${INTER_FILES})
    SWIG_ADD_MODULE(${FileBase} python ${module} ${impl_files} ${gen_files})
    SWIG_LINK_LIBRARIES(${FileBase} ${PYTHON_LIBRARIES})

    foreach(LibFile ${LIBRARY_BUILD_LIST})
         SWIG_LINK_LIBRARIES(${FileBase} ${LibFile})
    endforeach()

    foreach(LibFile ${library_dependencies})
         SWIG_LINK_LIBRARIES(${FileBase} ${LibFile})
    endforeach()

    set_target_properties(${SWIG_MODULE_${FileBase}_REAL_NAME} PROPERTIES
		FOLDER "${DIR_NAME_STRING}/${FileDir}")
    target_include_directories(${SWIG_MODULE_${FileBase}_REAL_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/${FileDir})
	# set_target_properties(${SWIG_MODULE_${FileBase}_REAL_NAME} PROPERTIES
	#     CMAKE_SWIG_OUTDIR "${CMAKE_SOURCE_DIR}/../dist/Basilisk/${FileBase}")
    set_target_properties(${SWIG_MODULE_${FileBase}_REAL_NAME} PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/../dist/Basilisk/simulation/${FileBase}")
    set_target_properties(${SWIG_MODULE_${FileBase}_REAL_NAME} PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY_DEBUG "${CMAKE_SOURCE_DIR}/../dist/Basilisk/simulation/${FileBase}")
    set_target_properties(${SWIG_MODULE_${FileBase}_REAL_NAME} PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY_RELEASE "${CMAKE_SOURCE_DIR}/../dist/Basilisk/simulation/${FileBase}")

    set_target_properties(${SWIG_MODULE_${FileBase}_REAL_NAME} PROPERTIES
	    COMPILE_FLAGS "-I${CMAKE_CURRENT_SOURCE_DIR}")
endforeach()
