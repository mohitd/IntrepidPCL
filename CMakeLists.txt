cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(IntrepidPCL)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_HOME_DIRECTORY}/bin)

find_package(PCL 1.8 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

# Main program
add_executable (intrepid src/intrepid.cpp)
target_link_libraries (intrepid ${PCL_LIBRARIES})

# Text to PCD
add_executable (ttp src/txt-to-pcd.cpp)
target_link_libraries (ttp ${PCL_LIBRARIES})

# PCD to VTK (via GreedyTriangulation)
add_executable (ptv src/pcd-to-vtk.cpp)
target_link_libraries (ptv ${PCL_LIBRARIES})

# Using Marching Cubes to reconstruct a mesh
add_executable (mcr src/mcr.cpp)
target_link_libraries (mcr ${PCL_LIBRARIES})

