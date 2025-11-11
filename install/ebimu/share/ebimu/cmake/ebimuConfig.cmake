# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_ebimu_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED ebimu_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(ebimu_FOUND FALSE)
  elseif(NOT ebimu_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(ebimu_FOUND FALSE)
  endif()
  return()
endif()
set(_ebimu_CONFIG_INCLUDED TRUE)

# output package information
if(NOT ebimu_FIND_QUIETLY)
  message(STATUS "Found ebimu: 0.1.0 (${ebimu_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'ebimu' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${ebimu_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(ebimu_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${ebimu_DIR}/${_extra}")
endforeach()
