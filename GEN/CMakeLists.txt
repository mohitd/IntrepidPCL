cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(GenSphere)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_HOME_DIRECTORY}/bin)

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

add_executable (gen_sphere gen_sphere.cpp)
target_link_libraries (gen_sphere ${PCL_LIBRARIES})

