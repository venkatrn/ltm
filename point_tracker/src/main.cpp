/**
 * @file main.cpp
 * @brief Point tracker node
 * @author Venkatraman Narayanan
 * Carnegie Mellon University, 2014
 */

#include <point_tracker/point_tracker.h>


int main(int argc, char** argv)
{
  ros::init(argc, argv, "point_tracker");
  PointTracker point_tracker;
  PointCloudPtr cloud (new PointCloud);

  if (pcl::io::loadPCDFile<PointT> (string("raw_0.pcd"), *cloud) == -1) //* load the file
  {
    printf("Couldn't read file test_pcd.pcd \n");
    return (-1);
  }
  
  point_tracker.CloudCBInternal(cloud);
  //point_tracker.CloudCBInternal(cloud);
  //point_tracker.CloudCBInternal(cloud);
  point_tracker.PrintEdges();
  
  printf("Number of points: %d\n", point_tracker.tracked_points_.size());
  printf("Number of frames %d\n", point_tracker.tracked_points_[0].size());


  pcl::visualization::PCLVisualizer viewer = point_tracker.GetVisualizer();
  
  viewer.spin();
  /*
  while (!viewer.wasStopped ())
  {
    viewer.spinOnce ();
    usleep(100000);
  }
  */
  return 0;
}

