# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_arithmetic_problem_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED arithmetic_problem_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(arithmetic_problem_FOUND FALSE)
  elseif(NOT arithmetic_problem_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(arithmetic_problem_FOUND FALSE)
  endif()
  return()
endif()
set(_arithmetic_problem_CONFIG_INCLUDED TRUE)

# output package information
if(NOT arithmetic_problem_FIND_QUIETLY)
  message(STATUS "Found arithmetic_problem: 0.0.0 (${arithmetic_problem_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'arithmetic_problem' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${arithmetic_problem_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(arithmetic_problem_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${arithmetic_problem_DIR}/${_extra}")
endforeach()
