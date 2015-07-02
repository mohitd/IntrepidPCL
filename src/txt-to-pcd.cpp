#include <iostream>
#include <fstream>

#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/common/common_headers.h>

using namespace std;
using namespace pcl;
using namespace pcl::io;

int main(int argc, char *argv[])
{
	ifstream fin(argv[1]);

	uint32_t num_data_pts = 0;
    string line;

    while (getline(fin, line)) ++num_data_pts;
    cout << "Number of data points: " << num_data_pts << endl;
    
    fin.clear();
    fin.seekg(0, ios::beg);
	
	PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
	cloud->width = num_data_pts;
	cloud->height = 1;
	cloud->points.resize(cloud->width * cloud->height);

	for (int i = 0; i < cloud->width; ++i)
	{
		float x, y, z;
		fin >> x >> y >> z;

		cloud->points[i].x = x;
		cloud->points[i].y = y;
		cloud->points[i].z = z;
	}

	fin.close();
	savePCDFileASCII("out.pcd", *cloud);
	cout << "Saving out.pcd" << endl;

	/** Estimating the surface normals */
	NormalEstimationOMP<PointXYZ, Normal> ne;
	search::KdTree<PointXYZ>::Ptr tree (new search::KdTree<PointXYZ>);
	tree->setInputCloud(cloud);

	unsigned int nthreads = 2;
	cout << "Running normal estimation on " << nthreads << " threads" << endl;
	
	ne.setNumberOfThreads(nthreads);
  	ne.setInputCloud(cloud);
  	ne.setSearchMethod(tree);
	ne.setKSearch(20);

	PointCloud<Normal>::Ptr cloud_normals (new PointCloud<Normal>);

	ne.compute (*cloud_normals);

	/** Creating a mesh */
	// Concatenate the XYZ and normal fields*
  	PointCloud<PointNormal>::Ptr cloud_with_normals (new PointCloud<PointNormal>);
  	concatenateFields (*cloud, *cloud_normals, *cloud_with_normals);
  	// cloud_with_normals = cloud + normals

  	savePCDFileASCII("out-with-normals.pcd", *cloud_with_normals);
	cout << "Saving out-with-normals.pcd" << endl;

	return 0;
}
