# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_mavros_vicon_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED mavros_vicon_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(mavros_vicon_FOUND FALSE)
  elseif(NOT mavros_vicon_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(mavros_vicon_FOUND FALSE)
  endif()
  return()
endif()
set(_mavros_vicon_CONFIG_INCLUDED TRUE)

# output package information
if(NOT mavros_vicon_FIND_QUIETLY)
  message(STATUS "Found mavros_vicon: 0.0.0 (${mavros_vicon_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'mavros_vicon' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${mavros_vicon_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(mavros_vicon_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${mavros_vicon_DIR}/${_extra}")
endforeach()
