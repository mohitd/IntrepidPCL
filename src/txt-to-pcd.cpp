#include <iostream>
#include <fstream>

#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/common/common_headers.h>

#include <pcl/console/parse.h>

using namespace std;
using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

void print_help(char **argv)
{
	print_error("Syntax is: %s in.txt <options>\n", argv[0]);
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
	txt_file_indices = parse_file_extension_argument(argc, argv, ".txt");
	if (txt_file_indices.size () != 1)
	{
		print_error("Need one input TEXT file to continue.\n");
		return -1;
	}

	string inputFileName = string(argv[txt_file_indices[0]]);
	size_t ext = inputFileName.find(".txt");
	string outputFileNameCore = inputFileName.substr(0, ext);

	ifstream fin(argv[txt_file_indices[0]]);

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
	string outPCD = outputFileNameCore + string(".pcd");
	savePCDFileASCII(outPCD, *cloud);
	cout << "Saving " << outPCD << endl;

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

	string outPCDWithNormals = string("n-") + outputFileNameCore + string(".pcd");
  	savePCDFileASCII(outPCDWithNormals, *cloud_with_normals);
	cout << "Saving " << outPCDWithNormals << endl;

	return 0;
}
