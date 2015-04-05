cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(IntrepidPCL)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_HOME_DIRECTORY}/bin)

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

add_executable (pcd_write intrepid_pcl.cpp)
target_link_libraries (pcd_write ${PCL_LIBRARIES})

