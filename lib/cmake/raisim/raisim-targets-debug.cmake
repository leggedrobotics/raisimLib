#----------------------------------------------------------------
# Generated CMake target import file for configuration "DEBUG".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "raisim::raisimODE" for configuration "DEBUG"
set_property(TARGET raisim::raisimODE APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(raisim::raisimODE PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_DEBUG "C;CXX"
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/lib/libraisimODE.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS raisim::raisimODE )
list(APPEND _IMPORT_CHECK_FILES_FOR_raisim::raisimODE "${_IMPORT_PREFIX}/lib/libraisimODE.a" )

# Import target "raisim::raisim" for configuration "DEBUG"
set_property(TARGET raisim::raisim APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(raisim::raisim PROPERTIES
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/lib/libraisim.so"
  IMPORTED_SONAME_DEBUG "libraisim.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS raisim::raisim )
list(APPEND _IMPORT_CHECK_FILES_FOR_raisim::raisim "${_IMPORT_PREFIX}/lib/libraisim.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
