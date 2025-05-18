cmake_minimum_required(VERSION 3.10)
project(hibah_groupsync)

set(CMAKE_CXX_STANDARD 11)

# Include headers
include_directories(${CMAKE_SOURCE_DIR}/dynamixel_sdk/include)

# Source files for Dynamixel SDK
set(DXLSDK_SOURCES
    ${CMAKE_SOURCE_DIR}/dynamixel_sdk/src/port_handler_windows.cpp
    ${CMAKE_SOURCE_DIR}/dynamixel_sdk/src/packet_handler.cpp
    ${CMAKE_SOURCE_DIR}/dynamixel_sdk/src/group_sync_write.cpp
    ${CMAKE_SOURCE_DIR}/dynamixel_sdk/src/group_sync_read.cpp
    ${CMAKE_SOURCE_DIR}/dynamixel_sdk/src/group_bulk_read.cpp
    ${CMAKE_SOURCE_DIR}/dynamixel_sdk/src/group_bulk_write.cpp
    ${CMAKE_SOURCE_DIR}/dynamixel_sdk/src/group_fast_bulk_read.cpp
    ${CMAKE_SOURCE_DIR}/dynamixel_sdk/src/group_fast_sync_read.cpp
    ${CMAKE_SOURCE_DIR}/dynamixel_sdk/src/group_handler.cpp
    ${CMAKE_SOURCE_DIR}/dynamixel_sdk/src/port_handler.cpp
    ${CMAKE_SOURCE_DIR}/dynamixel_sdk/src/protocol1_packet_handler.cpp
    ${CMAKE_SOURCE_DIR}/dynamixel_sdk/src/protocol2_packet_handler.cpp
)

# Build DLL
add_library(dynamixel_sdk SHARED ${DXLSDK_SOURCES})
target_compile_definitions(dynamixel_sdk PRIVATE DXLSDK_EXPORTS)
target_include_directories(dynamixel_sdk PUBLIC
    ${CMAKE_SOURCE_DIR}/dynamixel_sdk/include
)

# Executable
add_executable(main
    ${CMAKE_SOURCE_DIR}/program/main.cpp
)

target_link_libraries(main PRIVATE dynamixel_sdk)
