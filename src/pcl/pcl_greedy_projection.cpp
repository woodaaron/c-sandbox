#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include "surface_segmentation.h"
#include <iostream>

const bool REMOVE_LARGEST = false;
const int MIN_CLUSTER = 100;
const int MAX_CLUSTER = 100000;

int
main (int argc, char** argv)
{
  if(argc < 4)
  {
    std::cout << "Usage: " << argv[0] << "input.pcd output-dir name" << std::endl;
    return EXIT_FAILURE;
  }

  // Load input file into a PointCloud<T> with an appropriate type
  pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCLPointCloud2 cloud_blob;
  pcl::io::loadPCDFile (argv[1], cloud_blob);
  pcl::fromPCLPointCloud2 (cloud_blob, *in_cloud);
  //* the data should be available in cloud

  std::cout << "Input cloud has... " << std::to_string(in_cloud->points.size()) << " points" << std::endl;

  // Segment out surface ****************************************************************************************************

  // Segment the part into surface clusters using a "region growing" scheme
  std::cout << "Creating SS object\n";
  SurfaceSegmentation SS(in_cloud);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr region_colored_cloud_ptr = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
  std::vector <pcl::PointIndices> clusters = SS.computeSegments(region_colored_cloud_ptr);
  pcl::PointIndices::Ptr inliers_ptr(new pcl::PointIndices());
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> surface_clouds;

  if(clusters.size() > 0)
    std::cout << std::to_string(clusters.size()) << " clusters found" << std::endl;
  else
  {
    std::cout << "No clusters found" << std::endl;
    return EXIT_FAILURE;
  }

  std::cout << "Extracting surfaces\n";
  // Extract points from clusters into their own point clouds
  for (int i = 0; i < clusters.size(); i++)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr segment_cloud_ptr = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointIndices& indices = clusters[i];
    if (indices.indices.size() == 0)
      continue;

    if (indices.indices.size() >= MIN_CLUSTER)
    {
      inliers_ptr->indices.insert(inliers_ptr->indices.end(), indices.indices.begin(), indices.indices.end());

      pcl::copyPointCloud(*in_cloud, indices, *segment_cloud_ptr);
      std::vector<int> indices;
      pcl::removeNaNFromPointCloud (*segment_cloud_ptr, *segment_cloud_ptr, indices);
      surface_clouds.push_back(segment_cloud_ptr);
    }
  }

  if(surface_clouds.size() == 0)
  {
    std::cout << "No surfaces could be extracted" << std::endl;
    return EXIT_FAILURE;
  }

  // Remove largest cluster if appropriate
  if (REMOVE_LARGEST && surface_clouds.size() > 1)
  {
    std::cout << "Removing largest\n";
    int largest_index = 0;
    int largest_size = 0;
    for (int i = 0; i < surface_clouds.size(); i++)
    {
      if (surface_clouds[i]->points.size() > largest_size)
      {
        largest_size = surface_clouds[i]->points.size();
        largest_index = i;
      }
    }
    surface_clouds.erase(surface_clouds.begin() + largest_index);
  }

  //************************************************************************************************************************
  int counter = 0;

  std::string dir = argv[2];
  std::string prefix = argv[3];

  for(const auto& cloud : surface_clouds)
  {
    pcl::io::savePCDFileASCII(dir + "/" + prefix + "-surface-" + std::to_string(++counter) + ".pcd", *cloud);

    // Normal estimation*
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud);
    n.setInputCloud (cloud);
    n.setSearchMethod (tree);
    n.setKSearch (20);
    n.compute (*normals);
    //* normals should not contain the point normals + surface curvatures

    // Concatenate the XYZ and normal fields*
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
    //* cloud_with_normals = cloud + normals

    // Create search tree*
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

    // Additional vertex information
    std::vector<int> parts = gp3.getPartIDs();
    std::vector<int> states = gp3.getPointStates();

    pcl::io::saveVTKFile (dir + "/" + prefix + "-mesh-" + std::to_string(counter) + ".vtk", triangles);
  }

  // Finish
  return (0);
}