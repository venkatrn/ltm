
/**
 * @file perception_interface.h
 * @brief Interface for ltm perception
 * @author Venkatraman Narayanan
 * Carnegie Mellon University, 2014
 */

#include <ltm_perception/perception_interface.h>
#include <ltm_perception/perception_utils.h>
#include <ltm_msgs/PolygonArrayStamped.h>

#include <pcl/filters/filter.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_ros/transforms.h>

#include <boost/lexical_cast.hpp>

using namespace std;
using namespace perception_utils;


PerceptionInterface::PerceptionInterface(ros::NodeHandle nh) : nh_(nh)
{
  ros::NodeHandle private_nh("~");
  private_nh.param("pcl_visualization", pcl_visualization_, false);
  private_nh.param("reference_frame", reference_frame_, std::string("/base_link"));
  rectangle_pub_ = nh.advertise<ltm_msgs::PolygonArrayStamped>("rectangles", 1);
  cloud_sub_ = nh.subscribe("input_cloud", 1, &PerceptionInterface::CloudCB, this);
  if (pcl_visualization_)
  {
    viewer_= new pcl::visualization::PCLVisualizer("Articulation Viewer");

  }
}


void PerceptionInterface::CloudCB(const sensor_msgs::PointCloud2ConstPtr& sensor_cloud)
{
  PointCloudPtr pcl_cloud(new PointCloud);
  sensor_msgs::PointCloud2 ref_sensor_cloud;
  tf::StampedTransform transform;
  try{
    tf_listener_.lookupTransform(reference_frame_, sensor_cloud->header.frame_id,  
        ros::Time(0), transform);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
    //ros::Duration(1.0).sleep();
  }
  pcl_ros::transformPointCloud(reference_frame_, transform, *sensor_cloud, ref_sensor_cloud);

  // Fix up the "count" field of the PointCloud2 message because
  // transformLaserScanToPointCloud() does not set it to one which
  // is required by PCL since revision 5283.
  for(unsigned int i=0; i<ref_sensor_cloud.fields.size(); i++)
  {
    ref_sensor_cloud.fields[i].count = 1;
  }

  pcl::fromROSMsg (ref_sensor_cloud, *pcl_cloud);
  if (pcl_cloud == nullptr)
  {
    ROS_ERROR("[LTM Perception]: Error converting sensor cloud to pcl cloud");
    return;
  }
  ROS_DEBUG("[LTM Perception]: Converted sensor cloud to pcl cloud");
  CloudCBInternal(pcl_cloud);
  return;
}

void PerceptionInterface::CloudCBInternal(const string& pcd_file)
{
  PointCloudPtr cloud (new PointCloud);
  if (pcl::io::loadPCDFile<PointT> (pcd_file, *cloud) == -1) //* load the file
  {
    ROS_ERROR("Couldn't read file %s \n", pcd_file.c_str());
    return;
  }
  CloudCBInternal(cloud);
}

void PerceptionInterface::CloudCBInternal(const PointCloudPtr& original_cloud)
{
  if (pcl_visualization_)
  {
    viewer_->removeAllPointClouds();
    viewer_->removeAllShapes();
  }

  PointCloudPtr cloud(new PointCloud);
  cloud = original_cloud;

  if (pcl_visualization_ && original_cloud->size() != 0)
  {
    if (!viewer_->updatePointCloud(original_cloud, "input_cloud"))
    {
      viewer_->addPointCloud(original_cloud, "input_cloud");
    }
  }

  std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > > regions;
  OrganizedSegmentation(cloud, &regions);
  ROS_INFO("[LTM Perception]: Found %d planar regions", regions.size());
  if (pcl_visualization_)
  {
    DisplayPlanarRegions(*viewer_, regions);
  }

  ltm_msgs::PolygonArrayStamped poly_array_stamped;

  for (int ii = 0; ii < regions.size(); ++ii)
  {
    pcl::PointCloud<PointT>::Ptr contour (new pcl::PointCloud<PointT>);           contour->points = regions[ii].getContour();
    vector<PointT> corners;
    PointCloudPtr rectangle_proj_points(new PointCloud);
    vector<Eigen::Vector3f> axes;
    const bool found_rectangle = GetRectanglePoints(contour, rectangle_proj_points, &axes);
    if (!found_rectangle)
    {
      ROS_WARN("Couldn't find rectangle from contour. Skipping this contour");
      continue;
    }
    GetRectangleCorners(rectangle_proj_points, &corners, axes);
    if (!EvaluateRectangle(corners))
    {
      continue;
    }
    if (pcl_visualization_)
    {
      string id = boost::lexical_cast<string>(ii);
      /*
      pcl::visualization::PointCloudColorHandlerCustom <PointT> color (contour, 1.0, 0.0, 0.0);           
      if (!viewer_->updatePointCloud(rectangle_proj_points, color, string("rect_cloud_")+id))
      {
        viewer_->addPointCloud(rectangle_proj_points, color, string("rect_cloud")+id);
      }
      viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, string("rect_cloud")+id);                 
      */
      DrawRectangle(*viewer_, corners, id); 
    }

    // Publish the rectangle corners
    geometry_msgs::Polygon poly;
    geometry_msgs::Point32 p1, p2, p3, p4, p5;
    p1.x = corners[0].x; p1.y = corners[0].y; p1.z = corners[0].z;
    p2.x = corners[1].x; p2.y = corners[1].y; p2.z = corners[1].z;
    p3.x = corners[2].x; p3.y = corners[2].y; p3.z = corners[2].z;
    p4.x = corners[3].x; p4.y = corners[3].y; p4.z = corners[3].z;
    p5 = p1;
    poly.points.push_back(p1);
    poly.points.push_back(p2);
    poly.points.push_back(p3);
    poly.points.push_back(p4);
    poly.points.push_back(p5);
    poly_array_stamped.polygons.push_back(poly);
  }
  if (poly_array_stamped.polygons.size() > 0)
  {
    poly_array_stamped.header.frame_id = reference_frame_;
    poly_array_stamped.header.stamp = ros::Time::now();
    rectangle_pub_.publish(poly_array_stamped);
  }
}
