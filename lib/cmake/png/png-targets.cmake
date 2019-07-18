# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Create imported target raisim::raisim
add_library(raisim::png SHARED IMPORTED)

set(_IMPORT_PREFIX ${CMAKE_CURRENT_LIST_DIR}/../../..)

set_target_properties(raisim::png PROPERTIES
  INTERFACE_INCLUDE_DIRECTORIES "${_IMPORT_PREFIX}/include"
  IMPORTED_LOCATION ${_IMPORT_PREFIX}/lib/libpng.so
)

list(APPEND _IMPORT_CHECK_TARGETS raisim::png)
list(APPEND _IMPORT_CHECK_FILES_FOR_raisim::png "${_IMPORT_PREFIX}/lib/libpng.so")
