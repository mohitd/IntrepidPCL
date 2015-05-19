# Intrepid
This project uses the Point Cloud Library (PCL) to analyze 3D data points, examples are in the data folder.

## Formatting
The input data set is a `mx3` matrix where each column represents an x, y, and z coordinate respectively. The input file of points is formatted as follows

	x_1 y_1 z_1
	x_2 y_2 z_2
	...
	x_m y_m z_m

## Marching Cubes
The marching cubes algorithm is present in the `MCR` subfolder with it's own Cmake file

## Copyrights
Copyright (c) 2015. Mohit Deshpande
Copyright (c) 2012, Willow Garage, Inc.
