project (GeographicLib-Fortran Fortran)

cmake_minimum_required (VERSION 3.1.0)

# Set a default build type for single-configuration cmake generators if
# no build type is set.
if (NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set (CMAKE_BUILD_TYPE Release)
endif ()

# Make the compiler more picky.
if (MSVC)
  set_target_properties (${TOOLS} PROPERTIES COMPILE_FLAGS "/W4")
else ()
  set_target_properties (${TOOLS} PROPERTIES COMPILE_FLAGS
    "-Wall -Wextra -pedantic -std=f95 -fimplicit-none -Wno-compare-reals")
endif ()

# Tell Intel compiler to do arithmetic accurately.  This is needed to
# stop the compiler from ignoring parentheses in expressions like
# (a + b) + c and from simplifying 0.0 + x to x (which is wrong if
# x = -0.0).
if (CMAKE_Fortran_COMPILER_ID STREQUAL "Intel")
  if (MSVC)
    set (CMAKE_Fortran_FLAGS
      "${CMAKE_Fortran_FLAGS} /fp:precise /Qdiag-disable:11074,11076")
    set (CMAKE_Fortan_FLAGS
      "${CMAKE_Fortran_FLAGS} -fp-model precise -diag-disable=11074,11076")
  endif ()
endif ()

if (CONVERT_WARNINGS_TO_ERRORS)
  if (MSVC)
    set (CMAKE_Fortran_FLAGS "${CMAKE_Fortran_FLAGS} /WX")
  else ()
    set (CMAKE_Fortran_FLAGS "${CMAKE_Fortran_FLAGS} -Werror")
  endif ()
endif ()

set (TOOLS geoddirect geodinverse planimeter geodtest)

foreach (TOOL ${TOOLS})
  add_executable (${TOOL} ${TOOL}.for geodesic.for geodesic.inc)
endforeach ()

# Work alikes for NGS geodesic tools.  This uses legacy code from NGS
# and so they trigger multiple errors and warnings if compiled with the
# compile flags above.
add_executable (ngsforward ngsforward.for ngscommon.for geodesic.for)
add_executable (ngsinverse ngsinverse.for ngscommon.for geodesic.for)

# Turn on testing
enable_testing ()

# Run the test suite
add_test (NAME geodtest COMMAND geodtest)
