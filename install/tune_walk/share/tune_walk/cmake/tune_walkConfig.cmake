# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_tune_walk_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED tune_walk_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(tune_walk_FOUND FALSE)
  elseif(NOT tune_walk_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(tune_walk_FOUND FALSE)
  endif()
  return()
endif()
set(_tune_walk_CONFIG_INCLUDED TRUE)

# output package information
if(NOT tune_walk_FIND_QUIETLY)
  message(STATUS "Found tune_walk: 0.1.0 (${tune_walk_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'tune_walk' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${tune_walk_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(tune_walk_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${tune_walk_DIR}/${_extra}")
endforeach()
