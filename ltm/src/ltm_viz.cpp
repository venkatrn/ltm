/**
 * @file ltm_viz.cpp
 * @author Venkatraman Narayanan
 * Carnegie Mellon University, 2014
 */

#include <ltm/ltm_viz.h>
#include <ltm/d_model_utils.h>

using namespace std;

LTMViz::LTMViz(const std::string& ns)
{
  if(ns.size() > 0)
  {
    std::stringstream ss;
    ss << "/" << ns << "/visualization_marker_array";
    marker_array_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>(ss.str(), 500);
    ss.str(std::string());
    ss << "/" << ns << "/visualization_marker";
    marker_publisher_ = nh_.advertise<visualization_msgs::Marker>(ss.str(), 1000);
  }
  else
  {
    marker_array_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 500);
    marker_publisher_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 1000);
  }
}

void LTMViz::SetReferenceFrame(string frame) 
{
  reference_frame_ = frame;
}


void LTMViz::VisualizePoints(const geometry_msgs::PoseArray poses)
{
  ltm::RGBA color(1.0, 0.0, 0.0, 1.0); 
  VisualizePoints(poses, color);
}

void LTMViz::VisualizePoints(const geometry_msgs::PoseArray poses, const ltm::RGBA color)
{
  visualization_msgs::Marker points;
  points.header.frame_id = reference_frame_;
  points.header.stamp = ros::Time::now();
  points.ns = "points";
  points.action = visualization_msgs::Marker::ADD;
  points.pose.orientation.x = points.pose.orientation.y = points.pose.orientation.z = 0;
  points.pose.orientation.w = 1;
  points.id = 0;
  points.type = visualization_msgs::Marker::SPHERE_LIST;
  points.scale.x = points.scale.y = points.scale.z = 0.05;
  points.color.r = color.r;
  points.color.g = color.g;
  points.color.b = color.b;
  points.color.a = color.a;
  for (size_t ii = 0; ii < poses.poses.size(); ++ii)
  {
    geometry_msgs::Pose p = poses.poses[ii];
    // TODO: Skip grasp points
    points.points.push_back(p.position);
  }
  // Publish points
  PublishMarker(points);
}

void LTMViz::VisualizeEdges(const EdgeMap& edge_map, const geometry_msgs::PoseArray& dmodel_points)
{
  ltm::RGBA color(0.0, 0.0, 1.0, 1.0);
  VisualizeEdges(edge_map, dmodel_points, color);
}

void LTMViz::VisualizeEdges(const EdgeMap& edge_map, const geometry_msgs::PoseArray& dmodel_points, const ltm::RGBA color)
{
  visualization_msgs::Marker edges;
  edges.header.frame_id = reference_frame_;
  edges.header.stamp = ros::Time::now();
  edges.ns = "edges";
  edges.action = visualization_msgs::Marker::ADD;
  edges.pose.orientation.x = edges.pose.orientation.y = edges.pose.orientation.z = 0;
  edges.pose.orientation.w = 1;
  edges.id = 0;
  edges.type = visualization_msgs::Marker::LINE_LIST;
  edges.color.r = color.r;
  edges.color.g = color.g;
  edges.color.b = color.b;
  edges.color.a = color.a;
  edges.scale.x = 0.02;

  for (auto it = edge_map.begin(); it != edge_map.end(); ++it)
  {
    // TODO: Skip grasp points
    edges.points.push_back(dmodel_points.poses[it->first.first].position);
    edges.points.push_back(dmodel_points.poses[it->first.second].position);
  }
  PublishMarker(edges);
}


void LTMViz::VisualizeForcePrim(const tf::Vector3 force, const geometry_msgs::Pose end_effector_pose)
{
  ltm::RGBA color(0.0, 1.0, 0.0, 1.0);
  VisualizeForcePrim(force, end_effector_pose, color);
}

void LTMViz::VisualizeForcePrim(const tf::Vector3 force, const geometry_msgs::Pose end_effector_pose, const ltm::RGBA color)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = reference_frame_;
  marker.header.stamp = ros::Time::now();
  marker.ns = "marker";
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation.x = marker.pose.orientation.y = marker.pose.orientation.z = 0;
  marker.pose.orientation.w = 1;
  marker.id = 0;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.scale.x = 0.04;
  marker.scale.y = 0.12;
  marker.scale.z = 0.0;
  marker.color.r = color.a;
  marker.color.g = color.g;
  marker.color.b = color.b;
  marker.color.a = color.a;
  geometry_msgs::Point start_point = end_effector_pose.position;
  geometry_msgs::Point end_point;
  const double normalizer = Norm(force);
  end_point.x = start_point.x + force.x()/normalizer;
  end_point.y = start_point.y + force.y()/normalizer;
  end_point.z = start_point.z + force.z()/normalizer;
  marker.points.push_back(start_point);
  marker.points.push_back(end_point);
  PublishMarker(marker);
}

void LTMViz::PublishMarker(visualization_msgs::Marker& marker)
{
  marker_publisher_.publish(marker);
}

void LTMViz::PublishMarkerArray(visualization_msgs::MarkerArray& marker_array)
{
  marker_array_publisher_.publish(marker_array);
  usleep(1000);
}
