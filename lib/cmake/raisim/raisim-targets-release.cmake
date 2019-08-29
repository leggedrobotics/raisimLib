#----------------------------------------------------------------
# Generated CMake target import file for configuration "RELEASE".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "raisim::raisimODE" for configuration "RELEASE"
set_property(TARGET raisim::raisimODE APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(raisim::raisimODE PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libraisimODE.so"
  IMPORTED_SONAME_RELEASE "libraisimODE.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS raisim::raisimODE )
list(APPEND _IMPORT_CHECK_FILES_FOR_raisim::raisimODE "${_IMPORT_PREFIX}/lib/libraisimODE.so" )

# Import target "raisim::raisim" for configuration "RELEASE"
set_property(TARGET raisim::raisim APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(raisim::raisim PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libraisim.so"
  IMPORTED_SONAME_RELEASE "libraisim.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS raisim::raisim )
list(APPEND _IMPORT_CHECK_FILES_FOR_raisim::raisim "${_IMPORT_PREFIX}/lib/libraisim.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
