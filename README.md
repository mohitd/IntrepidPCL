# Intrepid
This project uses the Point Cloud Library (PCL) to analyze 3D data points and produce a mesh.

## Formatting
The input data set is a `mx3` matrix where each column represents an x, y, and z coordinate respectively. The input file of points is formatted as follows

x_1   y_1   z_1

x_2   y_2   z_2

...

x_m   y_m   z_m

with a space between each coordinate of a point on a line.

## Subdirectories

`build`			– CMake build files 

`bin`			– Executables for Intrepid 

`data`			– 3D scans from the Project Tango tablet 

## Executables

`ttp` - Converts a .txt file to 2 .pcd files (with/without normals)

`ptv` - Uses the GreedyTriangulation algorithm to convert the pcd with normals to a vtk mesh

`mcr` - Uses the Marching Cubes algorithm (Hoppe or RBF) to generate a mesh from a pcd with normals.

`intrepid` - Takes in a .txt file, performs filtering algorithms, normal estimation, and generates a mesh using Greedy, Marching Cubes Hoppe, or Marching Cubes RBF algorithms. **It's the "main" program!** 

## Project Tango
Intrepid uses[Project Tango tablets](https://www.google.com/atap/project-tango/) to take 3D scans of environments. The [MeshBuilder](https://github.com/mohitd/tango-examples-unity) application was modified to save binary files of the scan data to `/sdcard/Android/data/com.projecttango.experiments.unitymeshbuilder/files/`, which can be accessible through connecting the tablet to a computer via USB.

In order to convert the binary file to a .txt file with only the depth data, use [TangoBinaryDecoder](https://github.com/mohitd/tangobinarydecoder), a Mono C# application.

## Marching Cubes 
This subproject takes in input .pcd file and generates a mesh that uses either a radial basis function (RBF) or Hoppe marching cubes as described in _Surface Reconstruction from Unorganized Points_ (Hoppe et al.)

## Copyrights
Copyright (c) 2015. Mohit Deshpande

Copyright (c) 2012, Willow Garage, Inc.
