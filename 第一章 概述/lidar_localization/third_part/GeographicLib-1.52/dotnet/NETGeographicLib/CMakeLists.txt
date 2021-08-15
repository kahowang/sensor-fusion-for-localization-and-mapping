# Build the .NET library...

# Include all the .cpp files in the library.
file (GLOB SOURCES [A-Za-z]*.cpp)
file (GLOB HEADERS [A-Za-z]*.h)

add_library (${NETGEOGRAPHICLIB_LIBRARIES} SHARED ${SOURCES} ${HEADERS})
if (CMAKE_VERSION VERSION_LESS 3.12)
  set_target_properties (${NETGEOGRAPHICLIB_LIBRARIES}
    PROPERTIES COMPILE_FLAGS "/clr")
else ()
  set_target_properties (${NETGEOGRAPHICLIB_LIBRARIES}
    PROPERTIES COMMON_LANGUAGE_RUNTIME "")
endif()

string (REPLACE "/RTC1" "" CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}")
string (REPLACE "/EHsc" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")

if (MSVC AND NOT MSVC_VERSION GREATER 1600)
  # Disable "already imported" and "unsupported default parameter"
  # warnings with VS 10
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd4945 /wd4564")
endif ()

add_definitions (${PROJECT_DEFINITIONS})
target_link_libraries (${NETGEOGRAPHICLIB_LIBRARIES} ${PROJECT_LIBRARIES})

# Set the version number on the library
set_target_properties (${NETGEOGRAPHICLIB_LIBRARIES} PROPERTIES
  VERSION "${LIBVERSIONFULL}" OUTPUT_NAME ${NETLIBNAME})

# Specify where the library is installed, adding it to the export targets
install (TARGETS ${NETGEOGRAPHICLIB_LIBRARIES}
  EXPORT targets
  RUNTIME DESTINATION bin)

if (PACKAGE_DEBUG_LIBS)
  install (PROGRAMS
    "${PROJECT_BINARY_DIR}/bin/Debug/${NETLIBNAME}${CMAKE_DEBUG_POSTFIX}.dll"
    DESTINATION bin CONFIGURATIONS Release)
endif ()

# Install pdb file.
install (FILES $<TARGET_PDB_FILE:${NETGEOGRAPHICLIB_LIBRARIES}>
  DESTINATION bin OPTIONAL)

# Put all the library into a folder in the IDE
set_property (TARGET ${NETGEOGRAPHICLIB_LIBRARIES} PROPERTY FOLDER library)
