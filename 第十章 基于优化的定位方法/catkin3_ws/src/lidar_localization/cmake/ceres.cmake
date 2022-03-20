find_package (Ceres REQUIRED)

include_directories(${CERES_INCLUDE_DIRS})
list(APPEND ALL_TARGET_LIBRARIES ${CERES_LIBRARIES})