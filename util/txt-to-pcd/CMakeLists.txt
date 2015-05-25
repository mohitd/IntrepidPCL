cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(TXT-TO-PCD)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_HOME_DIRECTORY}/bin)

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

add_executable (ttp txt-to-pcd.cpp)
target_link_libraries (ttp ${PCL_LIBRARIES})

