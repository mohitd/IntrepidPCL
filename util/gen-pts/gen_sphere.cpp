#include <iostream>
#include <fstream>

#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/filter.h>

#define PI 3.141592653589793238462643383279502884

using namespace pcl;
using namespace pcl::io;

void gen_points(PointXYZ center, double r, std::vector<PointXYZ>& points)
{
	points.clear();
	for (double theta = 0; theta < 2*PI; theta += PI/10.)
	{
		for (double phi = 0; phi < PI; phi += PI/10.)
		{
			PointXYZ point;
			point.x = r * sin(phi) * cos(theta) + center.x;
			point.y = r * sin(phi) * sin(theta) + center.y;
			point.z = r            * cos(phi) + center.z;
			points.push_back(point);
		}
	}
}

int main(int argc, char *argv[])
{
	PointXYZ center;
	center.x = center.y = center.z = 1;
	std::vector<PointXYZ> spherePoints;

	gen_points(center, 3.0, spherePoints);

	PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ>);
	for (int i = 0; i < spherePoints.size(); i++)
	{
		PointXYZ point = spherePoints[i];
		cloud->push_back(point);
	}

	std::cout << "Point cloud is sized at " << cloud->size() << std::endl;
	savePCDFileASCII("out.pcd", *cloud);
	
	// Create the normal estimation class, and pass the input dataset to it
	NormalEstimation<PointXYZ, Normal> ne;
	search::KdTree<PointXYZ>::Ptr tree (new search::KdTree<PointXYZ>);
	tree->setInputCloud(cloud);
	ne.setInputCloud (cloud);
	ne.setSearchMethod (tree);
	ne.setKSearch(20);

	PointCloud<Normal>::Ptr cloud_normals (new PointCloud<Normal>);
	ne.compute (*cloud_normals);
  	
  	PointCloud<PointNormal>::Ptr cloud_with_normals (new PointCloud<PointNormal>);
  	concatenateFields (*cloud, *cloud_normals, *cloud_with_normals);

  	PointCloud<PointNormal>::Ptr final_cloud (new PointCloud<PointNormal>);
  	std::vector<int> indicies;
  	removeNaNNormalsFromPointCloud(*cloud_with_normals, *final_cloud, indicies);

	std::cout << "Point cloud with normals is sized at " << final_cloud->size() << std::endl;
	savePCDFileASCII("out_normals.pcd", *final_cloud);
}
