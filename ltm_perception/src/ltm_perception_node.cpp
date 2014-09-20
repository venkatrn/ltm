/**
 * @file ltm_perception_node.cpp
 * @brief LTM perception node
 * @author Venkatraman Narayanan
 * Carnegie Mellon University, 2014
 */

#include <ltm_perception/perception_utils.h>

#include <ros/ros.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <boost/lexical_cast.hpp>

using namespace std;
using namespace perception_utils;

//const string filename = "raw_0.pcd";
const string filename = "data/pointclouds/1404182828.986669753.pcd";



int main(int argc, char** argv)
{
  ros::init(argc, argv, "ltm_perception");

  
  PointCloudPtr cloud (new PointCloud);
  if (pcl::io::loadPCDFile<PointT> (filename, *cloud) == -1) //* load the file
  {
    printf("Couldn't read file %s \n", filename.c_str());
    return (-1);
  }
  PointCloudPtr original_cloud(new PointCloud);
  original_cloud = cloud;

  pcl::visualization::PCLVisualizer viewer;

  cloud = perception_utils::DownsamplePointCloud(cloud);

  vector<int> valid_indices;
  pcl::removeNaNFromPointCloud(*cloud, *cloud, valid_indices);
  printf("Number of valid indices: %d\n", valid_indices.size());
  
  if (!viewer.updatePointCloud(original_cloud, "input_cloud"))
  {
    viewer.addPointCloud(original_cloud, "input_cloud");
  }


  //Euclidean Clustering
  // PointCloudPtr cloud_cluster(new PointCloud);
  vector<PointCloudPtr> cluster_clouds;
  
  DoEuclideanClustering(cloud, &cluster_clouds);
  //perception_utils::DrawOrientedBoundingBox(viewer, cloud_cluster);
  for (int ii = 0; ii < cluster_clouds.size(); ++ii)
  {
    //perception_utils::GetEdges(cluster_clouds[ii]);
    string id = boost::lexical_cast<string>(ii);
    string box_id = "Box: " + id;
    DrawAxisAlignedBoundingBox(viewer, cluster_clouds[ii], box_id);

    PointCloudPtr plane_points(new PointCloud);
    pcl::ModelCoefficients::Ptr plane_coefficients(new pcl::ModelCoefficients);
    plane_coefficients = GetPlaneCoefficients(cluster_clouds[ii], plane_points);
    string plane_id = "Plane: " + id;
    pcl::visualization::PointCloudColorHandlerCustom<PointT> green(plane_points, 0, 255, 0);
    //viewer.addPlane(*line_coefficients, line_id);
    if (!viewer.updatePointCloud(plane_points, green, plane_id))
    {
      viewer.addPointCloud(plane_points, green, plane_id);
    }
  }

  viewer.spin();
  
  return 0;
}

