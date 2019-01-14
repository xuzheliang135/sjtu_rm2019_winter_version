cmake_minimum_required(VERSION 3.10)

project(sjtu_rm2019_winter_version)
set(CMAKE_CXX_STANDARD 11)
SET(CMAKE_BUILD_TYPE DEBUG)

FIND_PACKAGE(OpenCV 3 REQUIRED)

include_directories( ${PROJECT_SOURCE_DIR}/include)
include_directories( ${PROJECT_SOURCE_DIR}/src)
FILE(GLOB_RECURSE sourcefiles "src/*.cpp" )

add_executable(run main.cpp ${sourcefiles})

TARGET_LINK_LIBRARIES(run  ${OpenCV_LIBS})
TARGET_LINK_LIBRARIES(run  /lib/libMVSDK.so)


