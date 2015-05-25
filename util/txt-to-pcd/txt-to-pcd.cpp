#include <iostream>
#include <string>
#include <fstream>

#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

using namespace std;
using namespace pcl;
using namespace pcl::io;

int main(int argc, char *argv[])
{
	ifstream fin(argv[1]);

	int num_data_pts = 0;
    string line;

    while (getline(fin, line)) ++num_data_pts;
    cout << "Number of data points: " << num_data_pts << endl;
    
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
	savePCDFileASCII("out.pcd", cloud);
	cout << "Saving out.pcd" << endl;
	return 0;
}
