# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_robocup_vision25_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED robocup_vision25_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(robocup_vision25_FOUND FALSE)
  elseif(NOT robocup_vision25_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(robocup_vision25_FOUND FALSE)
  endif()
  return()
endif()
set(_robocup_vision25_CONFIG_INCLUDED TRUE)

# output package information
if(NOT robocup_vision25_FIND_QUIETLY)
  message(STATUS "Found robocup_vision25: 0.0.0 (${robocup_vision25_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'robocup_vision25' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${robocup_vision25_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(robocup_vision25_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${robocup_vision25_DIR}/${_extra}")
endforeach()
