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

using namespace std;
using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

int main(int argc, char *argv[])
{	
	PointCloud<PointNormal>::Ptr cloud_with_normals(new PointCloud<PointNormal>);
	loadPCDFile<PointNormal>(argv[1], *cloud_with_normals);

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
	
	saveVTKFile ("triangulation.vtk", triangles);
	print_highlight("Saving triangulation.vtk\n");

	return 0;
}
