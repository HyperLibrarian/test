cmake_minimum_required(VERSION 3.1.15)
project(demo2)

# obsensor sdk
set(OrbbecSDK_ROOT_DIR ${CMAKE_SOURCE_DIR}/../SDK) # 可通过该命令指定SDK查找路径
set(OrbbecSDK_LIBRARY_DIRS ${OrbbecSDK_ROOT_DIR}/lib)
set(OrbbecSDK_INCLUDE_DIR ${OrbbecSDK_ROOT_DIR}/include)
include_directories(${OrbbecSDK_INCLUDE_DIR})
link_directories(${OrbbecSDK_LIBRARY_DIRS})

# set(OpenCV_DIR "/usr/include/opencv4")  # 可通过该命令指定OpenCV_DIR模块查找路径
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIR})
link_directories(${OpenCV_LIBRARY_DIRS})

find_package(PCL 1.8 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (demo2 demo2.cpp)
target_link_libraries (demo2 OrbbecSDK ${OpenCV_LIBS} ${PCL_LIBRARIES})

