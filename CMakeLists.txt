cmake_minimum_required(VERSION 3.10)

project(sjtu_rm2019_winter_version)
set(CMAKE_CXX_STANDARD 11)
SET(CMAKE_BUILD_TYPE DEBUG)


# 添加cmake模块以使用ceres库
list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/extra_files/cmake_modules )

# 寻找G2O
find_package( G2O REQUIRED )
set(G2O_ROOT /usr/local/include/g2o)
include_directories(
        ${G2O_INCLUDE_DIRS}
        "/usr/include/eigen3"
)

FIND_PACKAGE(OpenCV 3 REQUIRED)
include_directories( ${OpenCV_DIRS} )

include_directories( ${PROJECT_SOURCE_DIR}/include)
include_directories( ${PROJECT_SOURCE_DIR}/tools)

FILE(GLOB_RECURSE sourcefiles "src/*.cpp" )

add_executable(run main.cpp ${sourcefiles})

target_link_libraries(run g2o_core g2o_stuff)
TARGET_LINK_LIBRARIES(run  ${OpenCV_LIBS})
TARGET_LINK_LIBRARIES(run  /lib/libMVSDK.so)
