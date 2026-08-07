# The sunSafeACS wrapper includes and calls the dvAttEffect implementation.
# Give the package library ownership so both wrappers can link one compiled copy.
set(BSK_PACKAGE_LIBRARY_SOURCES
    "${CMAKE_CURRENT_LIST_DIR}/../errorConversion/dvAttEffect.c")
