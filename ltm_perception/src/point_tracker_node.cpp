/**
 * @file point_tracker_node.cpp
 * @brief Point tracker node
 * @author Venkatraman Narayanan
 * Carnegie Mellon University, 2014
 */

#include <ltm_perception/point_tracker.h>

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
  
  PointTracker point_tracker;
  point_tracker.CloudCBInternal(cloud);
  point_tracker.PrintEdges();
  
  printf("Number of points: %d\n", point_tracker.tracked_points_.size());
  printf("Number of frames %d\n", point_tracker.tracked_points_[0].size());
  pcl::visualization::PCLVisualizer viewer = point_tracker.GetVisualizer();

  return 0;
}

