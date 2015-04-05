#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <stdlib.h> 

#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>

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

int main(int argc, char const *argv[])
{
	ifstream fin("box.txt");
	
	int num_data_pts = 0;
    std::string line;

    while (std::getline(fin, line)) ++num_data_pts;
    std::cout << "Number of lines in text file: " << num_data_pts << std::endl;
    
    fin.clear();
    fin.seekg(0, ios::beg);
	
	pcl::PointCloud<pcl::PointXYZ> cloud;
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
	
	pcl::io::savePCDFileASCII("out/pre-processed.pcd", cloud);
	std::cout << "Saved input of " << cloud.points.size() << " data points to out/pre-processed" << std::endl;
	
	/** Statistical Outlier Removal */
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZ>(cloud));
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>());
	
	// Create the filtering object
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  	sor.setInputCloud (cloud2);
  	sor.setMeanK (50); // analyzes 100 neighbors
  	sor.setStddevMulThresh (1.0); // remove all points who have a distance larger than 1 standard deviation of the mean distance 							
  	sor.filter (*cloud_filtered); // query point will be marked as outliers and removed
  	
  	std::cout << "New filtered cloud is now " << cloud_filtered->points.size() << std::endl;
	
	/** Estimating the surface normals */
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  	ne.setInputCloud(cloud_filtered);
  	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  	ne.setSearchMethod (tree);
  	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
	ne.setRadiusSearch (0.03);
	ne.compute (*cloud_normals);
	
	/** Creating a mesh */
	// Concatenate the XYZ and normal fields*
  	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
  	pcl::concatenateFields (*cloud_filtered, *cloud_normals, *cloud_with_normals);
  	// cloud_with_normals = cloud + normals

  	// Create search tree
  	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
  	tree2->setInputCloud (cloud_with_normals);
  	
  	// Initialize objects
  	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
  	pcl::PolygonMesh triangles;

  	// Set the maximum distance between connected points (maximum edge length)
  	gp3.setSearchRadius (0.025);

  	// Set typical values for the parameters
  	gp3.setMu (2.5);
  	gp3.setMaximumNearestNeighbors (100);
  	gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
  	gp3.setMinimumAngle(M_PI/18); // 10 degrees
  	gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
  	gp3.setNormalConsistency(false);

  	// Get result
  	gp3.setInputCloud (cloud_with_normals);
  	gp3.setSearchMethod (tree2);
  	gp3.reconstruct (triangles);
	
	pcl::io::saveVTKFile ("out/mesh.vtk", triangles);
	
	return 0;
}
