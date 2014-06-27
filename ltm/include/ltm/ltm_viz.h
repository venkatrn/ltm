/**
 * @file ltm_viz.h
 * @author Venkatraman Narayanan
 * Carnegie Mellon University, 2014
 */

#ifndef _LTM_LTM_VIZ_H_
#define _LTM_LTM_VIZ_H_

#include <ltm/d_model_structs.h>

#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/LinearMath/Vector3.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <string>

namespace ltm {
  struct RGBA
  {
    double r, g, b, a;
    RGBA()
    {
      r = g = b = 0.0;
      a = 1.0;
    }
    RGBA(double red, double green, double blue, double alpha)
    {
      r = red;
      g = green;
      b = blue;
      a = alpha;
    }
  };
}

class LTMViz
{
  public:
    explicit LTMViz(const std::string& ns);
    ~LTMViz();

    /* @brief set reference frame of visualization markers */
    void SetReferenceFrame(std::string frame);
    void PublishMarker(visualization_msgs::Marker& marker);
    void PublishMarkerArray(visualization_msgs::MarkerArray& marker_array);
    
    void VisualizePoints(const geometry_msgs::PoseArray& dmodel_points);
    void VisualizePoints(const geometry_msgs::PoseArray& dmodel_points, ltm::RGBA point_color);

    void VisualizeModel(const EdgeMap& edge_map, const geometry_msgs::PoseArray& dmodel_points);
    void VisualizeModel(const EdgeMap& edge_map, const geometry_msgs::PoseArray& dmodel_points, ltm::RGBA edge_color, ltm::RGBA point_color);

    void VisualizeForcePrim(const tf::Vector3 force, const geometry_msgs::Pose end_effector_pose);
    void VisualizeForcePrim(const tf::Vector3 force, const geometry_msgs::Pose end_effector_pose, const ltm::RGBA color);

    void VisualizeAxis(const geometry_msgs::Pose axis);
    void VisualizeTraj(const geometry_msgs::PoseArray traj);

  private:
    ros::NodeHandle nh_;
    ros::Publisher marker_array_publisher_;
    ros::Publisher marker_publisher_;
    std::string reference_frame_;
    visualization_msgs::MarkerArray marker_array_;
    visualization_msgs::Marker marker_;
};

#endif /* _LTM_LTM_VIZ_H_ */
