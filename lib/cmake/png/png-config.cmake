if(NOT TARGET raisim::png)

  include(FindPackageHandleStandardArgs)

  unset(png_FOUND)

  include(${CMAKE_CURRENT_LIST_DIR}/png-targets.cmake)

  get_target_property(png_INCLUDE_DIRS raisim::png INTERFACE_INCLUDE_DIRECTORIES)
  get_target_property(png_LIBRARIES raisim::png IMPORTED_LOCATION)

  # NOTE: Checks need to happen at least once when the package is first imported
  find_package_handle_standard_args(raisim::png
    REQUIRED_VARS
      png_INCLUDE_DIRS
      png_LIBRARIES
  )

  if(assimp_FOUND)
      message(STATUS "raisim::png:")
      message(STATUS "  Includes: ${assimp_INCLUDE_DIRS}")
      message(STATUS "  Libraries: ${assimp_LIBRARIES}")
  endif()

endif()
