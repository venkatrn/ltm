/**
 * @file perception_interface.h
 * @brief Interface for ltm perception
 * @author Venkatraman Narayanan
 * Carnegie Mellon University, 2014
 */

#ifndef _LTM_PERCEPTION_PERCEPTION_INTERFACE_H_
#define _LTM_PERCEPTION_PERCEPTION_INTERFACE_H_

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>

#include <ltm_perception/pcl_typedefs.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/visualization/pcl_visualizer.h>

class PerceptionInterface
{
  public:
    PerceptionInterface(ros::NodeHandle nh);
    void CloudCB(const sensor_msgs::PointCloud2ConstPtr& sensor_cloud);
    void CloudCBInternal(const std::string& pcd_file);

    // Accessors
    const pcl::visualization::PCLVisualizer* viewer() const {return viewer_;}
    bool pcl_visualization() const {return pcl_visualization_;}
    // Mutators
    pcl::visualization::PCLVisualizer* mutable_viewer() const {return viewer_;}
    
  private:
    ros::NodeHandle nh_;
    pcl::visualization::PCLVisualizer* viewer_;

    //pcl::visualization::PCLVisualizer viewer_;
    bool pcl_visualization_;
    ros::Publisher rectangle_pub_;
    ros::Subscriber cloud_sub_;
    std::string reference_frame_;
    tf::TransformListener tf_listener_;

    // Does all the work
    void CloudCBInternal(const PointCloudPtr& original_cloud);
};

#endif /** _LTM_PERCEPTION_PERCEPTION_INTERFACE **/
