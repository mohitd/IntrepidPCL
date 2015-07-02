#include <iostream>

#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>

#include <pcl/common/common_headers.h>

#include <pcl/surface/gp3.h>

#include <pcl/console/time.h>
#include <pcl/console/parse.h>

using namespace std;
using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

void print_help(char **argv)
{
	print_error("Syntax is: %s in.pcd <options>\n", argv[0]);
}

int main(int argc, char *argv[])
{
	if (argc < 2)
	{
		print_help(argv);
		return -1;
	}

	// Parse the command line arguments for .pcd files
	vector<int> txt_file_indices;
	txt_file_indices = parse_file_extension_argument(argc, argv, ".pcd");
	if (txt_file_indices.size () != 1)
	{
		print_error("Need one input PCD file to continue.\n");
		return -1;
	}

	string inputFileName = string(argv[txt_file_indices[0]]);
	size_t ext = inputFileName.find(".pcd");
	string outputFileName = inputFileName.substr(0, ext) + string(".vtk");

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

  	TicToc tt;
  	tt.tic();
  	print_highlight("Computing GreedyProjectionTriangulation ");
  	gp3.reconstruct (triangles);
  	print_info("[done, "); print_value("%g", tt.toc()); print_info(" ms]\n");
	
	saveVTKFile (outputFileName, triangles);
	print_highlight(string(string("Saving ") + outputFileName + string("\n")).c_str());

	return 0;
}
