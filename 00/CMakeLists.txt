cmake_minimum_required(VERSION 3.10)
project(pcl_test)
#set(PCL_DIR /home/ls/softwares/lib/pcl1121/share/pcl-1.12)
find_package(PCL 1.8 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(pcl_test pcl_test.cpp)

target_link_libraries (pcl_test ${PCL_LIBRARIES})

install(TARGETS pcl_test RUNTIME DESTINATION bin)
