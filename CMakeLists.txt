cmake_minimum_required(VERSION 3.10)

project(sjtu_rm2019_winter_version)
set(CMAKE_CXX_STANDARD 11)
SET(CMAKE_BUILD_TYPE DEBUG)


# 允许c++11标准、O3优化、多线程。match选项可避免一些cpu上的问题
set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread" )


FIND_PACKAGE(OpenCV 3 REQUIRED)
include_directories( ${OpenCV_DIRS} )

include_directories( ${PROJECT_SOURCE_DIR}/include)
include_directories( ${PROJECT_SOURCE_DIR}/tools)

FILE(GLOB_RECURSE sourcefiles "src/*.cpp" )

add_executable(run main.cpp ${sourcefiles})


TARGET_LINK_LIBRARIES(run  ${OpenCV_LIBS})
TARGET_LINK_LIBRARIES(run  /lib/libMVSDK.so)
