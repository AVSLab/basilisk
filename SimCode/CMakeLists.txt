cmake_minimum_required(VERSION 2.8)

function (create_symlinks destinationPath)
    # Do nothing if building in-source
    if (${CMAKE_CURRENT_BINARY_DIR} STREQUAL ${CMAKE_CURRENT_SOURCE_DIR})
        return()
    endif()
	# message(destinationPath: ${destinationPath})
	# message(ARGN: ${ARGN})
    foreach (filePath ${ARGN})
        get_filename_component(fileName ${filePath} NAME)
# 		message(${folder})
#         # Create REAL folder
#         file(MAKE_DIRECTORY "${destinationPath}/${folder}")
#message("directory created: ${destinationPath}/${folder}")
        # Delete symlink if it exists
        # file(REMOVE "${CMAKE_CURRENT_BINARY_DIR}/${path_file}")

        # Get OS dependent path to use in `execute_process`
        file(TO_NATIVE_PATH "${destinationPath}/${fileName}" link)
        file(TO_NATIVE_PATH "${filePath}" target)

        if (UNIX)
            set(command ln -s ${target} ${link})
        else()
            set(command cmd.exe /c mklink ${link} ${target})
        endif()

        execute_process(COMMAND ${command} 
                        RESULT_VARIABLE result
                        ERROR_VARIABLE output)

        if (NOT ${result} EQUAL 0)
            message(FATAL_ERROR "Could not create symbolic link for: ${target} --> ${output}")
        endif()

    endforeach(filePath)
endfunction(create_symlinks)

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

FOREACH(module ${swig_modules})
    get_filename_component(FileBase ${module} NAME_WE)
    get_filename_component(FileDir ${module} DIRECTORY)
	set(moduleFilePath "${CMAKE_SOURCE_DIR}/modules/${FileBase}")
	message("${CMAKE_SOURCE_DIR}/modules/${FileBase}")
    # Delete module folder and contents if it exists
	file(REMOVE_RECURSE "${CMAKE_SOURCE_DIR}/modules/${FileBase}")

	if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${FileDir}/CMakeLists.txt")
        continue()
    endif()
    file(GLOB impl_files
	    "${FileDir}/*.cpp"
	    "${FileDir}/*.c"
	    "${FileDir}/*.h"
	    "${FileDir}/../_GeneralModuleFiles/*.cpp"
	    "${FileDir}/../_GeneralModuleFiles/*.h"
	    "${FileDir}/../_GeneralModuleFiles/*.c"
	)
		
    file(MAKE_DIRECTORY "${CMAKE_SOURCE_DIR}/modules/${FileBase}")
	set(CMAKE_SWIG_OUTDIR "${CMAKE_SOURCE_DIR}/modules/${FileBase}")
	if(NOT EXISTS "${FileDir}/__init__.py")
		file(WRITE "__init__.py" "#This __init__.py file for the ${FileBase} package is intentionally empty.")
		file(COPY "__init__.py" DESTINATION ${CMAKE_SWIG_OUTDIR})
	else()
		configure_file(${CMAKE_CURRENT_SOURCE_DIR}/${FileDir}/__init__.py ${CMAKE_SWIG_OUTDIR}/__init__.py COPYONLY)
	endif()
	if (EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/__init__.py")
		file(REMOVE "__init__.py")
	endif()
	
	file(GLOB supporting_files ${CMAKE_CURRENT_SOURCE_DIR}/${FileDir}/*.xml ${CMAKE_CURRENT_SOURCE_DIR}/${FileDir}/*.py)
	list(REMOVE_ITEM supporting_files ${CMAKE_CURRENT_SOURCE_DIR}/${FileDir}/__init__.py)
 	# message("${CMAKE_CURRENT_SOURCE_DIR}/${FileDir}")
	# message("${supporting_files}")
	create_symlinks(${moduleFilePath} ${supporting_files})
	
    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/${FileDir})
    set_source_files_properties(${module} PROPERTIES CPLUSPLUS ON)
    #set_source_files_properties(${module} PROPERTIES SWIG_FLAGS "-includeall")
    #set_source_files_properties(${module} PROPERTIES SWIG_FLAGS "-I${CMAKE_SOURCE_DIR}/SimCode/utilities")
	set_property(SOURCE ${module} PROPERTY SWIG_FLAGS "-I${CMAKE_SOURCE_DIR}/SimCode/utilities" "-I${CMAKE_CURRENT_SOURCE_DIR}/${FileDir}/")
	# MESSAGE("${SWIG_FLAGS}")
    #set_source_files_properties(${module} PROPERTIES SWIG_FLAGS "-I${CMAKE_CURRENT_SOURCE_DIR}/${FileDir}/")
	set_source_files_properties( ${swig_generated_file_fullname}
               PROPERTIES COMPILE_FLAGS "-I${CMAKE_SOURCE_DIR}/SimCode")
    set(SWIG_MODULE_${FileBase}_EXTRA_DEPS ${impl_files})
    SWIG_ADD_MODULE(${FileBase} python ${module} ${impl_files})
    SWIG_LINK_LIBRARIES(${FileBase} ${PYTHON_LIBRARIES})
    foreach(LibFile ${LIBRARY_BUILD_LIST})
         SWIG_LINK_LIBRARIES(${FileBase} ${LibFile})
    endforeach()
    foreach(LibFile ${library_dependencies})
         SWIG_LINK_LIBRARIES(${FileBase} ${LibFile})
    endforeach()
    set_target_properties(${SWIG_MODULE_${FileBase}_REAL_NAME} PROPERTIES 
		FOLDER "${DIR_NAME_STRING}/${FileDir}")
    set_target_properties(${SWIG_MODULE_${FileBase}_REAL_NAME} PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/modules/${FileBase}")
    set_target_properties(${SWIG_MODULE_${FileBase}_REAL_NAME} PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY_DEBUG "${CMAKE_SOURCE_DIR}/modules/${FileBase}")
    set_target_properties(${SWIG_MODULE_${FileBase}_REAL_NAME} PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY_RELEASE "${CMAKE_SOURCE_DIR}/modules/${FileBase}")
    set_target_properties(${SWIG_MODULE_${FileBase}_REAL_NAME} PROPERTIES
	    COMPILE_FLAGS "-I${CMAKE_CURRENT_SOURCE_DIR}")
ENDFOREACH()
