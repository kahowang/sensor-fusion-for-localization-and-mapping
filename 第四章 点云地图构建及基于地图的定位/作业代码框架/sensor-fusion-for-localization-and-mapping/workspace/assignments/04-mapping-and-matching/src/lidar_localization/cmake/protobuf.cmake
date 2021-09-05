find_package (Protobuf REQUIRED)

include_directories(${Protobuf_INCLUDE_DIRS})
list(APPEND ALL_TARGET_LIBRARIES ${Protobuf_LIBRARIES})