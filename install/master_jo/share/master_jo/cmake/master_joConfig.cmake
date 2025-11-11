# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_master_jo_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED master_jo_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(master_jo_FOUND FALSE)
  elseif(NOT master_jo_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(master_jo_FOUND FALSE)
  endif()
  return()
endif()
set(_master_jo_CONFIG_INCLUDED TRUE)

# output package information
if(NOT master_jo_FIND_QUIETLY)
  message(STATUS "Found master_jo: 0.0.0 (${master_jo_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'master_jo' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${master_jo_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(master_jo_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${master_jo_DIR}/${_extra}")
endforeach()
