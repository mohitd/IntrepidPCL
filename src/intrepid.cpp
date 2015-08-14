#include <iostream>
#include <fstream>

#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/features/normal_3d_omp.h>

#include <pcl/common/common_headers.h>

#include <pcl/surface/gp3.h>
#include <pcl/surface/marching_cubes_hoppe.h>
#include <pcl/surface/marching_cubes_rbf.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/impl/mls.hpp>

#include <pcl/console/time.h>
#include <pcl/console/parse.h>

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

int default_mean_k = 200;
float default_stdev = 0.1f;
float default_radius = 0.01f;
int default_min_neighbors = 100;
float default_leaf_size = 0.01f;
int default_norm_k = 100;
float default_off_surface_eps = 0.1f;
int default_grid_res = 50;
int default_algorithm = 1;
float default_search_radius = 0.03f;

void print_help(char **argv) {
    print_error("Syntax is: %s in.txt out.vtk <options>\n", argv[0]);

    print_info("        -mean_k X    = # of neighbors to analyze (StatisticalOutlierRemoval) (default: ");
    print_value("%d", default_mean_k);
    print_info(")\n");

    print_info("        -stdev X     = standard deviation threshold (StatisticalOutlierRemoval) (default: ");
    print_value("%.5f", default_stdev);
    print_info(")\n");

    print_info("        -leaf_size X = resolution of the cubic grid (VoxelGridFilter) (default: ");
    print_value("%.5f", default_leaf_size);
    print_info(")\n");

    print_info("        -radius_size X = radius of sphere to check (RadiusOutlierRemoval) (default: ");
    print_value("%.5f", default_radius);
    print_info(")\n");

    print_info("        -norm_k X    = # of neighbors for normal estimation (default: ");
    print_value("%d", default_norm_k);
    print_info(")\n");

    print_info("        -algorithm X = ID of the algorithm to run (default: ");
    print_value("%d", default_algorithm);
    print_info(")\n");
    print_info("            1 : GreedyTriangulation\n");
    print_info("            2 : MarchingCubesHoppe\n");
    print_info("            3 : MarchingCubesRBF\n");
}

/**
 * Read point cloud data from the given file and put the input cloud in 'cloud'
 */
void read_point_cloud(const std::string &fileName, PointCloud<PointXYZ> &cloud) {
    std::ifstream fin(fileName.c_str());

    uint32_t num_data_pts = 0;
    std::string line;

    while (std::getline(fin, line)) ++num_data_pts;
    print_info("Num of data points: ");
    print_value("%d\n\n", num_data_pts);

    fin.clear();
    fin.seekg(0, std::ios::beg);

    cloud.width = num_data_pts;
    cloud.height = 1;
    cloud.points.resize(cloud.width * cloud.height);
    cloud.is_dense = true;

    for (int i = 0; i < cloud.width; ++i) {
        float x, y, z;
        fin >> x >> y >> z;

        cloud.points[i].x = x;
        cloud.points[i].y = y;
        cloud.points[i].z = z;
    }

    fin.close();
}

/**
 * Performs statistical outlier removal
 * Overwrites 'cloud' with filtered cloud
 */
void filter_statistical_outlier(PointCloud<PointXYZ> &cloud, int k, float stdev) {
    // New cloud to hold filtered data
    PointCloud<PointXYZ>::Ptr cloudFiltered(new PointCloud<PointXYZ>());
    PointCloud<PointXYZ>::Ptr cloudCopy(new PointCloud<PointXYZ>(cloud));

    StatisticalOutlierRemoval<PointXYZ> sor;
    sor.setInputCloud(cloudCopy);
    sor.setMeanK(k);
    sor.setStddevMulThresh(stdev);

    TicToc tt;
    tt.tic();
    print_highlight("Computing StatisticalOutlierRemoval ");
    sor.filter(*cloudFiltered);
    print_info("[done, ");
    print_value("%g", tt.toc());
    print_info(" ms]\n");

    print_info("SOR filtered cloud is now ");
    print_value("%d\n\n", cloudFiltered->points.size());
    cloud = *cloudFiltered;
}

/**
 * Removes all points that don't have n number of neighbors in an r radius
 */
void filter_radius_outlier(PointCloud<PointXYZ> &cloud, float r, int n)
{
    PointCloud<PointXYZ>::Ptr cloudFiltered(new PointCloud<PointXYZ>());
    PointCloud<PointXYZ>::Ptr cloudCopy(new PointCloud<PointXYZ>(cloud));

    RadiusOutlierRemoval<PointXYZ> ror;
    ror.setInputCloud(cloudCopy);
    ror.setRadiusSearch(r);
    ror.setMinNeighborsInRadius(n);

    TicToc tt;
    tt.tic();
    print_highlight("Computing RadiusOutlierRemoval ");
    ror.filter(*cloudFiltered);
    print_info("[done, ");
    print_value("%g", tt.toc());
    print_info(" ms]\n");

    print_info("ROR filtered cloud is now ");
    print_value("%d\n\n", cloudFiltered->points.size());
    cloud = *cloudFiltered;
}

/**
 * Performs the voxel grid filter to downsample data
 * Overwrites 'cloud' with filtered cloud
 */
void filter_voxel_grid(PointCloud<PointXYZ> &cloud, float size) {
    PointCloud<PointXYZ>::Ptr cloudFiltered(new PointCloud<PointXYZ>());
    PointCloud<PointXYZ>::Ptr cloudCopy(new PointCloud<PointXYZ>(cloud));

    VoxelGrid<PointXYZ> voxelFilter;
    voxelFilter.setInputCloud(cloudCopy);
    voxelFilter.setLeafSize(size, size, size);

    TicToc tt;
    tt.tic();
    print_highlight("Computing VoxelGridFilter ");
    voxelFilter.filter(*cloudFiltered);
    print_info("[done, ");
    print_value("%g", tt.toc());
    print_info(" ms]\n");

    print_info("VGF filtered cloud is now ");
    print_value("%d\n\n", cloudFiltered->points.size());
    cloud = *cloudFiltered;
}

/**
 * Normal estimation from tangent planes using PCA (run on multiple threads with OpenMP)
 */
void normal_estimation(PointCloud<PointXYZ> &cloud, PointCloud<Normal> &cloudNormals, int k) {
    PointCloud<PointXYZ>::Ptr cloudCopy(new PointCloud<PointXYZ>(cloud));
    NormalEstimationOMP<PointXYZ, Normal> ne;

    search::KdTree<PointXYZ>::Ptr pointTree(new search::KdTree<PointXYZ>);
    pointTree->setInputCloud(cloudCopy);

    ne.setInputCloud(cloudCopy);
    ne.setSearchMethod(pointTree);
    ne.setKSearch(k);

    TicToc tt;
    tt.tic();
    print_highlight("Computing normals ");
    ne.compute(cloudNormals);
    print_info("[done, ");
    print_value("%g", tt.toc());
    print_info(" ms]\n\n");
}

/**
 * Smoothing of the point normals using moving least squares
 */
void normal_smoothing(PointCloud<PointNormal> &cloud, float r) {
    search::KdTree<PointNormal>::Ptr tree(new search::KdTree<PointNormal>);
    PointCloud<PointNormal>::Ptr cloudCopy(new PointCloud<PointNormal>(cloud));
    PointCloud<PointNormal> cloudSmoothed;

    MovingLeastSquares<PointNormal, PointNormal> mls;
    mls.setComputeNormals(false);
    mls.setPolynomialFit(true);
    mls.setInputCloud(cloudCopy);
    mls.setSearchMethod(tree);
    mls.setSearchRadius(r);

    TicToc tt;
    tt.tic();
    print_highlight("Smoothing normals ");
    mls.process(cloudSmoothed);
    print_info("[done, ");
    print_value("%g", tt.toc());
    print_info(" ms]\n");

    cloud = cloudSmoothed;
}

/**
 * Computes rho-bar of the point cloud.
 */
double cloud_resolution(const PointCloud<PointXYZ>::ConstPtr cloud) {
    double res = 0.0;
    int points = 0;
    int nres;
    std::vector<int> indices(2);
    std::vector<float> sq_dist (2);
    search::KdTree<PointXYZ> tree;
    tree.setInputCloud(cloud);

    for (size_t i = 0; i < cloud->size(); i++) {
        nres = tree.nearestKSearch(i, 2, indices, sq_dist);
        if (nres == 2) {
            res += sqrt(sq_dist[1]);
            points++;
        }
    }

    if (points != 0) {
        res /= points;
    }

    return res;
}

int main(int argc, char *argv[]) {
    if (argc < 3) {
        print_help(argv);
        return -1;
    }

    // Parse the command line arguments for .pcd files
    std::vector<int> txt_file_indices = parse_file_extension_argument(argc, argv, ".txt");
    if (txt_file_indices.size() != 1) {
        print_error("Need one input text file and one output VTK file to continue.\n");
        return -1;
    }

    std::vector<int> vtk_file_indices = parse_file_extension_argument(argc, argv, ".vtk");
    if (vtk_file_indices.size() != 1) {
        print_error("Need one output VTK file to continue.\n");
        return -1;
    }

    // k neighbors to search for in StatisticalOutlierRemoval
    int mean_k = default_mean_k;
    parse_argument(argc, argv, "-mean_k", mean_k);
    if (mean_k < 1) mean_k = default_mean_k;
    print_info("Setting a mean k of: "); print_value("%d\n", mean_k);

    // sigma for StatisticalOutlierRemoval
    float stdev = default_stdev;
    parse_argument(argc, argv, "-stdev", stdev);
    if (stdev < 0) stdev = default_stdev;
    print_info("Setting a standard deviation of: "); print_value("%f\n", stdev);

    // resolution for MarchingCubes
    int grid_res = default_grid_res;
    parse_argument(argc, argv, "-grid_res", grid_res);
    if (grid_res < 0) grid_res = default_grid_res;
    print_info("Setting grid resolution: "); print_value("%d\n", grid_res);

    // radius to check for RadiusOutlierRemoval
    float radius = default_radius;
    parse_argument(argc, argv, "-radius_size", radius);
    if (radius <= 0) radius = default_radius;
    print_info("Setting radius of: "); print_value("%f\n", radius);

    // minimum number of neighbors to check in radius for RadiusOutlierRemoval
    int min_neighbors = default_min_neighbors;
    parse_argument(argc, argv, "-min_neighbors", min_neighbors);
    if (min_neighbors < 1) min_neighbors = default_min_neighbors;
    print_info("Setting min neighbors: " ); print_value("%d\n", min_neighbors);

    // search radius for MLS normal smoothing
    float search_radius = default_search_radius;
    parse_argument(argc, argv, "-search_radius", search_radius);
    if (search_radius < 0) search_radius = default_search_radius;
    print_info("Setting search radius: "); print_value("%f\n", search_radius);

    // size of voxels to generate for VoxelGridFilter
    float leaf_size = default_leaf_size;
    parse_argument(argc, argv, "-leaf_size", leaf_size);
    if (leaf_size <= 0) leaf_size = default_leaf_size;
    print_info("Setting a leaf size of: "); print_value("%f\n", leaf_size);

    // k neighbors to search for NormalEstimation
    int norm_k = default_norm_k;
    parse_argument(argc, argv, "-norm_k", norm_k);
    if (norm_k < 1) norm_k = default_norm_k;
    print_info("Setting a norm k of: "); print_value("%d\n", norm_k);

    std::string str_algorithm("");
    int algorithm = default_algorithm;
    parse_argument(argc, argv, "-algorithm", algorithm);
    if (algorithm <= 1 || algorithm > 3) {
        str_algorithm = "GreedyTriangulation";
    } else if (algorithm == 2) {
        str_algorithm = "MarchingCubesHoppe";
    } else if (algorithm == 3) {
        str_algorithm = "MarchingCubesRBF";
    }
    print_info((std::string("Selected algorithm: ") + str_algorithm).c_str()); print_info("\n\n");


    PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>());
    read_point_cloud(argv[txt_file_indices[0]], *cloud);

    filter_statistical_outlier(*cloud, mean_k, stdev);
    filter_radius_outlier(*cloud, radius, min_neighbors);
    filter_voxel_grid(*cloud, leaf_size);

    print_info("Cloud \u03C1-bar: "); print_value("%e\n\n", cloud_resolution(cloud));

    PointCloud<Normal>::Ptr cloudNormals(new PointCloud<Normal>());
    normal_estimation(*cloud, *cloudNormals, norm_k);

    // Concatenate the XYZ and normal fields
    PointCloud<PointNormal>::Ptr cloudWithNormals(new PointCloud<PointNormal>());
    concatenateFields(*cloud, *cloudNormals, *cloudWithNormals);

    normal_smoothing(*cloudWithNormals, search_radius);

    // Create k-d search tree that has the points and point normals
    search::KdTree<PointNormal>::Ptr normalTree(new search::KdTree<PointNormal>);
    normalTree->setInputCloud(cloudWithNormals);

    PolygonMesh triangles;

    // Using OOP principle of polymorphism!
    PCLSurfaceBase<PointNormal> *reconstruction;
    switch (algorithm) {
        case 1: {
            reconstruction = new GreedyProjectionTriangulation<PointNormal>();
            GreedyProjectionTriangulation<PointNormal> *greedy = reinterpret_cast<GreedyProjectionTriangulation<PointNormal> *>(reconstruction);
            greedy->setSearchRadius(0.025);
            greedy->setMu(2.5);
            greedy->setMaximumNearestNeighbors(500);
            greedy->setMaximumSurfaceAngle(M_PI / 4);
            greedy->setMinimumAngle(M_PI / 18);
            greedy->setMaximumAngle(2 * M_PI / 3);
            greedy->setNormalConsistency(false);
            break;
        }
        case 2: {
            reconstruction = new MarchingCubesHoppe<PointNormal>();
            MarchingCubesHoppe<PointNormal> *hoppe = reinterpret_cast<MarchingCubesHoppe<PointNormal> *>(reconstruction);
            hoppe->setGridResolution(grid_res, grid_res, grid_res);
            hoppe->setIsoLevel(0);
            hoppe->setPercentageExtendGrid(0);
            break;
        }
        case 3: {
            reconstruction = new MarchingCubesRBF<PointNormal>();
            MarchingCubesRBF<PointNormal> *rbf = reinterpret_cast<MarchingCubesRBF<PointNormal> *>(reconstruction);
            rbf->setOffSurfaceDisplacement(default_off_surface_eps);
            rbf->setGridResolution(grid_res, grid_res, grid_res);
            rbf->setIsoLevel(0);
            rbf->setPercentageExtendGrid(0);
            break;
        }
        default:
            print_error("Unrecognized algorithm selection: %d", algorithm);
            return -1;
    }

    reconstruction->setInputCloud(cloudWithNormals);
    reconstruction->setSearchMethod(normalTree);

    TicToc tt;
    tt.tic();
    print_highlight((std::string("Computing ") + str_algorithm).c_str());
    reconstruction->reconstruct(triangles);
    print_info("[done, "); print_value("%g", tt.toc()); print_info(" ms]\n");

    print_info("# polygons: "); print_value("%d\n", triangles.polygons.size());

    delete reconstruction;

    saveVTKFile(argv[vtk_file_indices[0]], triangles);
    print_highlight("Saving ");
    print_value("%s\n", argv[vtk_file_indices[0]]);
    return 0;
}
