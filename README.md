# Intrepid
This project uses the Point Cloud Library (PCL) to analyze 3D data points, examples are in the data folder.

## Formatting
The input data set is a `mx3` matrix where each column represents an x, y, and z coordinate respectively. The input file of points is formatted as follows

	x_1 y_1 z_1
	x_2 y_2 z_2
	...
	x_m y_m z_m

## Subdirectories

`build`			– CMake build files
`bin`			– Executables for Intrepid
`data`			– 3D scans from the Project Tango tablet
`util/gen-pts`	– Generate 3D point cloud data
`util/mcr`		– Marching cubes implementation

### Generate Point Cloud Data
This subproject generates all of the points that lie on a sphere of radius 3 using spherical coordinates (using $$\theta$$ and $$\phi$$)

### Marching Cubes 
This subproject takes in input .pcd file and generates a mesh that uses either a radial basis function (RBF) or Hoppe marching cubes as described in _Surface Reconstruction from Unorganized Points_ (Hoppe et al.)

## Copyrights
Copyright (c) 2015. Mohit Deshpande
Copyright (c) 2012, Willow Garage, Inc.
