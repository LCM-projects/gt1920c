cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(gt1920c_driver)

set(CAMERA_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include)
set(CAMERA_LIB ${PROJECT_SOURCE_DIR}/lib/libPvAPI.so)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

find_package(OpenCV REQUIRED)

include_directories(
${CAMERA_INCLUDE_DIR}
${OpenCV_INCLUDE_DIRS}
)

add_definitions(-std=c++17 -g -O3 -fext-numeric-literals -Wall -D_LINUX -D_x64)

MESSAGE(STATUS "CAMERA_INCLUDE_DIR: " ${CAMERA_INCLUDE_DIR})
MESSAGE(STATUS "CAMERA_LIB: " ${CAMERA_LIB})

add_subdirectory(src)
