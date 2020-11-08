if(BUILD_OPNAV)
  include(usingOpenCV)

  if(UNIX AND NOT APPLE)
    list(APPEND CUSTOM_DEPENDENCIES libmount/2.33.1@bincrafters/stable)
    list(APPEND CUSTOM_DEPENDENCIES libpng/1.6.37@bincrafters/stable)
    list(APPEND CUSTOM_DEPENDENCIES libtiff/4.0.9@bincrafters/stable)
    list(APPEND CUSTOM_DEPENDENCIES pcre/8.41@bincrafters/stable)
    list(APPEND CUSTOM_DEPENDENCIES freetype/2.10.0@bincrafters/stable)
  endif()

  if(APPLE)
    link_libraries("-march=haswell -mno-lzcnt")
  endif()


else()
  MESSAGE("SKIPPED: ${TARGET_NAME}")
  set(CUSTOM_DEPENDENCIES_HANDLED 1)
endif()
