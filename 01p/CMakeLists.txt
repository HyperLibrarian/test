cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(demo1)

set(CMAKE_CXX_STANDARD 14)

# set(OpenCV_DIR "/usr/include/opencv4")  # 可通过该命令指定OpenCV_DIR模块查找路径
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIR})
link_directories(${OpenCV_LIBRARY_DIRS})

find_package(PCL 1.8 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(demo1 SHARED demo1.cpp)
target_link_libraries (demo1 ${PCL_LIBRARIES})
target_link_libraries (demo1 ${OpenCV_LIBS})