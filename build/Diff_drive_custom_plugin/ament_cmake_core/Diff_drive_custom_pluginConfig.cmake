# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_Diff_drive_custom_plugin_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED Diff_drive_custom_plugin_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(Diff_drive_custom_plugin_FOUND FALSE)
  elseif(NOT Diff_drive_custom_plugin_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(Diff_drive_custom_plugin_FOUND FALSE)
  endif()
  return()
endif()
set(_Diff_drive_custom_plugin_CONFIG_INCLUDED TRUE)

# output package information
if(NOT Diff_drive_custom_plugin_FIND_QUIETLY)
  message(STATUS "Found Diff_drive_custom_plugin: 0.0.0 (${Diff_drive_custom_plugin_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'Diff_drive_custom_plugin' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${Diff_drive_custom_plugin_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(Diff_drive_custom_plugin_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "ament_cmake_export_libraries-extras.cmake")
foreach(_extra ${_extras})
  include("${Diff_drive_custom_plugin_DIR}/${_extra}")
endforeach()
