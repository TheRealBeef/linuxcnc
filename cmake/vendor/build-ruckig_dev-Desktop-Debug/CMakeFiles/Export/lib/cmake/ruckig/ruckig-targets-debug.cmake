#----------------------------------------------------------------
# Generated CMake target import file for configuration "Debug".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "ruckig::ruckig" for configuration "Debug"
set_property(TARGET ruckig::ruckig APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(ruckig::ruckig PROPERTIES
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/lib/libruckig.so"
  IMPORTED_SONAME_DEBUG "libruckig.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS ruckig::ruckig )
list(APPEND _IMPORT_CHECK_FILES_FOR_ruckig::ruckig "${_IMPORT_PREFIX}/lib/libruckig.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
