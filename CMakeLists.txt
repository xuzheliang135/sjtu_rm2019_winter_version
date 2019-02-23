cmake_minimum_required(VERSION 2.6)

project(sjtu_rm2019_winter_version)
set(CMAKE_CXX_STANDARD 11)
SET(CMAKE_BUILD_TYPE DEBUG)


find_package(OpenMP)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS} -std=c++11 -pthread")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
set(OpenMP_FLAGS ${OpenMP_CXX_FLAGS})
set(OpenMP_LIBS gomp)


FIND_PACKAGE(OpenCV 3 REQUIRED)
include_directories( ${OpenCV_DIRS} )


if(CMAKE_BUILD_TYPE STREQUAL Debug)
    message("Debug mode")
    ADD_DEFINITIONS(-D_DEBUG)
endif(CMAKE_BUILD_TYPE STREQUAL Debug)

if(CMAKE_BUILD_TYPE STREQUAL Release)
    message("Release mode")
endif(CMAKE_BUILD_TYPE STREQUAL Release)


include_directories( ${PROJECT_SOURCE_DIR}/include)
include_directories( ${PROJECT_SOURCE_DIR}/tools)

FILE(GLOB_RECURSE sourcefiles "src/*.cpp" )

add_executable(run main.cpp ${sourcefiles})

target_compile_options(run PRIVATE ${OpenMP_FLAGS})
TARGET_LINK_LIBRARIES(run  ${OpenCV_LIBS})
TARGET_LINK_LIBRARIES(run  /lib/libMVSDK.so)
TARGET_LINK_LIBRARIES(run  ${OpenMP_LIBS})