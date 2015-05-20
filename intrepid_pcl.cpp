#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <stdlib.h> 
#include <ctime>
#include <thread>

#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

#include <pcl/io/obj_io.h>
#include <pcl/TextureMesh.h>
#include <pcl/surface/texture_mapping.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/grid_projection.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/organized_fast_mesh.h>
#include <pcl/surface/ear_clipping.h>
#include <pcl/surface/poisson.h>
#include <pcl/surface/marching_cubes_rbf.h>

#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

int default_mean_k = 200;
float default_stdev = 0.3f;
float default_leaf_size = 0.01f;

void print_help(char **argv)
{
	print_error("Syntax is: %s in.txt out.vtk <options>\n", argv[0]);
	
	print_info("\t\t-mean_k X\t= the number of neighbors to analyze per point (StatisticalOutlierRemoval) (default: ");
  	print_value("%d", default_mean_k); print_info(")\n");
  	
  	print_info("\t\t-stdev X\t= the threshold for standard deviation (StatisticalOutlierRemoval) (default: ");
  	print_value("%f", default_stdev); print_info(")\n");

	print_info("\t\t-leaf_size X\t= the resolution of the grid (VoxelGridFilter) (cubic grid) (default: ");
  	print_value("%f", default_leaf_size); print_info(")\n");
}

int main(int argc, char *argv[])
{
	if (argc < 3) 
	{
		print_help(argv);
		return -1;
	}

	// Parse the command line arguments for .pcd files
	std::vector<int> txt_file_indices;
	txt_file_indices = parse_file_extension_argument(argc, argv, ".txt");
	if (txt_file_indices.size () != 1)
	{
		print_error("Need one input text file and one output VTK file to continue.\n");
		return -1;
	}

	std::vector<int> vtk_file_indices = parse_file_extension_argument(argc, argv, ".vtk");
	if (vtk_file_indices.size () != 1)
	{
		print_error("Need one output VTK file to continue.\n");
		return -1;
	}

	int mean_k = default_mean_k;
	parse_argument(argc, argv, "-mean_k", mean_k);
	print_info("Setting a mean k of: "); print_value("%d\n", mean_k);

	float stdev = default_stdev;
	parse_argument(argc, argv, "-stdev", stdev);
	print_info("Setting a standard deviation of: "); print_value("%f\n", stdev);

	float leaf_size = default_leaf_size;
	parse_argument(argc, argv, "-leaf_size", leaf_size);
	print_info("Setting a leaf size of: "); print_value("%f\n", leaf_size);

	ifstream fin(argv[txt_file_indices[0]]);

	int num_data_pts = 0;
    std::string line;

    while (std::getline(fin, line)) ++num_data_pts;
    print_info("Num of data points: "); print_value("%d\n", num_data_pts);
    
    fin.clear();
    fin.seekg(0, ios::beg);
	
	PointCloud<PointXYZ> cloud;
	cloud.width = num_data_pts;
	cloud.height = 1;
	cloud.points.resize(cloud.width * cloud.height);

	for (int i = 0; i < cloud.width; ++i)
	{
		float x, y, z;
		fin >> x >> y >> z;

		cloud.points[i].x = x;
		cloud.points[i].y = y;
		cloud.points[i].z = z;
	}

	fin.close();
	savePCDFileASCII("out/pre-processed.pcd", cloud);
	print_highlight("Saving "); print_value("%s\n", "pre-processed.pcd");
	
	/** Statistical Outlier Removal */
	PointCloud<PointXYZ>::Ptr cloud2 (new PointCloud<PointXYZ>(cloud));
	PointCloud<PointXYZ>::Ptr cloud_filtered (new PointCloud<PointXYZ>());
	PointCloud<PointXYZ>::Ptr cloud_filtered2 (new PointCloud<PointXYZ>());
	
	// create the filtering object
	StatisticalOutlierRemoval<PointXYZ> sor;
  	sor.setInputCloud (cloud2);
  	sor.setMeanK(mean_k); // analyzes mean_k neighbors
  	sor.setStddevMulThresh(stdev); // remove all points who have a distance larger than stdev standard deviation of the mean distance

  	TicToc tt1;
  	tt1.tic();
  	print_highlight("Computing StatisticalOutlierRemoval ");
  	sor.filter(*cloud_filtered); // query point will be marked as outliers and removed
  	print_info("[done, "); print_value("%g", tt1.toc ()); print_info(" ms]\n");
  	
  	print_info("Stats filtered cloud is now "); print_value("%d\n", cloud_filtered->points.size());

  	VoxelGrid<PointXYZ> voxelFilter;
  	voxelFilter.setInputCloud(cloud_filtered);
  	voxelFilter.setLeafSize(leaf_size, leaf_size, leaf_size);	// create voxels of leaf_size

  	TicToc tt2;
  	tt2.tic();
  	print_highlight("Computing VoxelGridFilter ");
  	voxelFilter.filter(*cloud_filtered2);
  	print_info("[done, "); print_value("%g", tt2.toc()); print_info(" ms]\n");
	
	print_info("Voxel filtered cloud is now "); print_value("%d\n", cloud_filtered2->points.size());

	savePCDFileASCII("out/post-processed.pcd", *cloud_filtered2);
	print_highlight("Saving "); print_value("%s\n", "post-processed.pcd");

	/** Estimating the surface normals */
	NormalEstimationOMP<PointXYZ, Normal> ne;
	search::KdTree<PointXYZ>::Ptr tree (new search::KdTree<PointXYZ>);
	tree->setInputCloud(cloud_filtered2);

	unsigned int nthreads = 2;
	print_info("Running normal estimation on "); print_value("%d"); print_info(" threads\n");
	
	ne.setNumberOfThreads(nthreads);
  	ne.setInputCloud(cloud_filtered2);
  	ne.setSearchMethod(tree);
	ne.setKSearch(20);

	PointCloud<Normal>::Ptr cloud_normals (new PointCloud<Normal>);

	TicToc tt3;
	tt3.tic();
	print_highlight("Computing normals ");
	ne.compute (*cloud_normals);
	print_info("[done, "); print_value("%g", tt3.toc()); print_info(" ms]\n");

	/** Creating a mesh */
	// Concatenate the XYZ and normal fields*
  	PointCloud<PointNormal>::Ptr cloud_with_normals (new PointCloud<PointNormal>);
  	concatenateFields (*cloud_filtered2, *cloud_normals, *cloud_with_normals);
  	// cloud_with_normals = cloud + normals

  	savePCDFileASCII("out/post-processed_normals.pcd", *cloud_with_normals);
	print_highlight("Saving "); print_value("%s\n", "post-processed_normals.pcd");

  	// Create search tree
  	search::KdTree<PointNormal>::Ptr tree2 (new search::KdTree<PointNormal>);
  	tree2->setInputCloud (cloud_with_normals);
  	
  	
  	// Initialize objects
  	GreedyProjectionTriangulation<PointNormal> gp3;
  	PolygonMesh triangles;

  	// Set the maximum distance between connected points (maximum edge length)
  	gp3.setSearchRadius (0.025);

  	// Set typical values for the parameters
  	gp3.setMu (2.5);
  	gp3.setMaximumNearestNeighbors(500);
  	gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
  	gp3.setMinimumAngle(M_PI/18); // 10 degrees
  	gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
  	gp3.setNormalConsistency(false);

  	// Get result of triangulation
  	gp3.setInputCloud (cloud_with_normals);
  	gp3.setSearchMethod (tree2);

  	TicToc tt4;
  	tt4.tic();
  	print_highlight("Computing GreedyProjectionTriangulation ");
  	gp3.reconstruct (triangles);
  	print_info("[done, "); print_value("%g", tt4.toc()); print_info(" ms]\n");
	
  	std::string dir("out/");
  	std::string file(argv[vtk_file_indices[0]]);
	saveVTKFile (dir + file, triangles);
	print_highlight("Saving "); print_value("%s\n", argv[vtk_file_indices[0]]);
	return 0;
}
