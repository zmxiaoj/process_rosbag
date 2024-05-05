cmake_minimum_required(VERSION 3.0 FATAL_ERROR)
project(pcl_pcd2txt)

find_package(PCL 1.8 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(pcl_pcd2txt pcl_pcd2txt.cpp)
target_link_libraries(pcl_pcd2txt ${PCL_LIBRARIES})