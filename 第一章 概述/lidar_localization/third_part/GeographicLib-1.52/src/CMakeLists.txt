# Build the library...

# Include all the .cpp files in the library.
file (GLOB SOURCES [A-Za-z]*.cpp)
file (GLOB HEADERS
  ${PROJECT_BINARY_DIR}/include/GeographicLib/Config.h
  ../include/GeographicLib/[A-Za-z]*.hpp)

# Define the library and specify whether it is shared or not.
if (GEOGRAPHICLIB_SHARED_LIB)
  add_library (${PROJECT_SHARED_LIBRARIES} SHARED ${SOURCES} ${HEADERS})
  add_library (${PROJECT_NAME}::${PROJECT_SHARED_LIBRARIES}
    ALIAS ${PROJECT_SHARED_LIBRARIES})
endif ()
if (GEOGRAPHICLIB_STATIC_LIB)
  add_library (${PROJECT_STATIC_LIBRARIES} STATIC ${SOURCES} ${HEADERS})
  add_library (${PROJECT_NAME}::${PROJECT_STATIC_LIBRARIES}
    ALIAS ${PROJECT_STATIC_LIBRARIES})
endif ()

add_library (${PROJECT_INTERFACE_LIBRARIES} INTERFACE)
add_library (${PROJECT_NAME}::${PROJECT_INTERFACE_LIBRARIES}
  ALIAS ${PROJECT_INTERFACE_LIBRARIES})
target_link_libraries (${PROJECT_INTERFACE_LIBRARIES}
  INTERFACE ${PROJECT_LIBRARIES})

# Set the version number on the library
if (MSVC)
  if (GEOGRAPHICLIB_SHARED_LIB)
    set_target_properties (${PROJECT_SHARED_LIBRARIES} PROPERTIES
      VERSION "${LIBVERSION_BUILD}" OUTPUT_NAME ${LIBNAME}
      IMPORT_SUFFIX -i.lib)
    target_compile_definitions (${PROJECT_SHARED_LIBRARIES}
      PUBLIC GEOGRAPHICLIB_SHARED_LIB=1)
  endif ()
  if (GEOGRAPHICLIB_STATIC_LIB)
    set_target_properties (${PROJECT_STATIC_LIBRARIES} PROPERTIES
      VERSION "${LIBVERSION_BUILD}" OUTPUT_NAME ${LIBNAME})
    target_compile_definitions (${PROJECT_STATIC_LIBRARIES}
      PUBLIC GEOGRAPHICLIB_SHARED_LIB=0)
  endif ()
else ()
  set_target_properties (
    ${PROJECT_SHARED_LIBRARIES} ${PROJECT_STATIC_LIBRARIES} PROPERTIES
    VERSION "${LIBVERSION_BUILD}" SOVERSION "${LIBVERSION_API}"
    OUTPUT_NAME ${LIBNAME})
  if (APPLE AND GEOGRAPHICLIB_PRECISION EQUAL 5)
    if (GEOGRAPHICLIB_SHARED_LIB)
      target_link_libraries (${PROJECT_SHARED_LIBRARIES} ${HIGHPREC_LIBRARIES})
    endif ()
    if (GEOGRAPHICLIB_STATIC_LIB)
      target_link_libraries (${PROJECT_STATIC_LIBRARIES} ${HIGHPREC_LIBRARIES})
    endif ()
  endif ()
endif ()

if (GEOGRAPHICLIB_SHARED_LIB)
  target_include_directories (${PROJECT_SHARED_LIBRARIES} PUBLIC
    $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}/include>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
endif ()
if (GEOGRAPHICLIB_STATIC_LIB)
  target_include_directories (${PROJECT_STATIC_LIBRARIES} PUBLIC
    $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}/include>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
endif ()

# Specify where the library is installed, adding it to the export targets
install (TARGETS ${PROJECT_ALL_LIBRARIES}
  EXPORT targets
  # A potentially useful option.  However it's only supported in recent
  # versions of cmake (2.8.12 and later?).  So comment out for now.
  # INCLUDES DESTINATION include
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib${LIB_SUFFIX}
  ARCHIVE DESTINATION lib${LIB_SUFFIX})

if (MSVC AND PACKAGE_DEBUG_LIBS)
  if (GEOGRAPHICLIB_SHARED_LIB)
    install (FILES
      "${PROJECT_BINARY_DIR}/lib/Debug/${LIBNAME}${CMAKE_DEBUG_POSTFIX}-i.lib"
      DESTINATION lib${LIB_SUFFIX} CONFIGURATIONS Release)
    install (PROGRAMS
      "${PROJECT_BINARY_DIR}/bin/Debug/${LIBNAME}${CMAKE_DEBUG_POSTFIX}.dll"
      DESTINATION bin CONFIGURATIONS Release)
  endif ()
  if (GEOGRAPHICLIB_STATIC_LIB)
    install (FILES
      "${PROJECT_BINARY_DIR}/lib/Debug/${LIBNAME}${CMAKE_DEBUG_POSTFIX}.lib"
      DESTINATION lib${LIB_SUFFIX} CONFIGURATIONS Release)
  endif ()
endif ()

if (MSVC AND GEOGRAPHICLIB_SHARED_LIB)
  install (FILES $<TARGET_PDB_FILE:${PROJECT_SHARED_LIBRARIES}>
    DESTINATION bin OPTIONAL)
endif ()

# Put the library into a folder in the IDE
set_target_properties (
  ${PROJECT_SHARED_LIBRARIES} ${PROJECT_STATIC_LIBRARIES}
  PROPERTIES FOLDER library)
