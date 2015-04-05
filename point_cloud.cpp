
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

using namespace pcl;
using namespace pcl::io;
using namespace std;

int main(int argc, char const *argv[])
{
	ifstream fin("Points.txt");
	pcl::PointCloud<pcl::PointXYZ> cloud;
	cloud.width = 370600;
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

	

	return 0;
}
