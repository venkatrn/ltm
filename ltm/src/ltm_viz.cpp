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
    marker_array_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>(ss.str(), 500, true);
    ss.str(std::string());
    ss << "/" << ns << "/visualization_marker";
    marker_publisher_ = nh_.advertise<visualization_msgs::Marker>(ss.str(), 1000, true);
  }
  else
  {
    marker_array_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 500, true);
    marker_publisher_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 1000, true);
  }
}

void LTMViz::SetReferenceFrame(string frame) 
{
  reference_frame_ = frame;
}

void LTMViz::VisualizePoints(const geometry_msgs::PoseArray& dmodel_points)
{
  ltm::RGBA point_color(1.0, 0.0, 0.0, 1.0);
  VisualizePoints(dmodel_points, point_color);
}

void LTMViz::VisualizePoints(const geometry_msgs::PoseArray& dmodel_points, ltm::RGBA point_color)
{
  visualization_msgs::MarkerArray marker_array;

  visualization_msgs::Marker points_marker;
  points_marker.header.frame_id = reference_frame_;
  points_marker.header.stamp = ros::Time::now();
  points_marker.ns = "points";
  points_marker.action = visualization_msgs::Marker::ADD;
  points_marker.pose.orientation.x = points_marker.pose.orientation.y = points_marker.pose.orientation.z = 0;
  points_marker.pose.orientation.w = 1;
  points_marker.id = 0;
  points_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  points_marker.scale.x = points_marker.scale.y = points_marker.scale.z = 0.05;
  points_marker.color.r = point_color.r;
  points_marker.color.g = point_color.g;
  points_marker.color.b = point_color.b;
  points_marker.color.a = point_color.a;
  points_marker.lifetime = ros::Duration(1000.0);
  for (size_t ii = 0; ii < dmodel_points.poses.size(); ++ii)
  {
    geometry_msgs::Pose p = dmodel_points.poses[ii];
    // TODO: Skip grasp points
    points_marker.points.push_back(p.position);
  }
  marker_array.markers.push_back(points_marker);
  PublishMarkerArray(marker_array);
}

void LTMViz::VisualizeModel(const EdgeMap& edge_map, const geometry_msgs::PoseArray& dmodel_points)
{
  ltm::RGBA edge_color(0.0, 0.0, 1.0, 1.0);
  ltm::RGBA point_color(1.0, 0.0, 0.0, 1.0);
  VisualizeModel(edge_map, dmodel_points, edge_color, point_color);
}

void LTMViz::VisualizeModel(const EdgeMap& edge_map, const geometry_msgs::PoseArray& dmodel_points, ltm::RGBA edge_color, ltm::RGBA point_color)
{
  visualization_msgs::MarkerArray marker_array;

  visualization_msgs::Marker points_marker;
  points_marker.header.frame_id = reference_frame_;
  points_marker.header.stamp = ros::Time::now();
  points_marker.ns = "points";
  points_marker.action = visualization_msgs::Marker::ADD;
  points_marker.pose.orientation.x = points_marker.pose.orientation.y = points_marker.pose.orientation.z = 0;
  points_marker.pose.orientation.w = 1;
  points_marker.id = 0;
  points_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  points_marker.scale.x = points_marker.scale.y = points_marker.scale.z = 0.05;
  points_marker.color.r = point_color.r;
  points_marker.color.g = point_color.g;
  points_marker.color.b = point_color.b;
  points_marker.color.a = point_color.a;
  points_marker.lifetime = ros::Duration(1000.0);
  for (size_t ii = 0; ii < dmodel_points.poses.size(); ++ii)
  {
    geometry_msgs::Pose p = dmodel_points.poses[ii];
    // TODO: Skip grasp points
    points_marker.points.push_back(p.position);
  }
  marker_array.markers.push_back(points_marker);

  visualization_msgs::Marker edges_marker;
  edges_marker.header.frame_id = reference_frame_;
  edges_marker.header.stamp = ros::Time::now();
  edges_marker.ns = "edges_marker";
  edges_marker.action = visualization_msgs::Marker::ADD;
  edges_marker.pose.orientation.x = edges_marker.pose.orientation.y = edges_marker.pose.orientation.z = 0;
  edges_marker.pose.orientation.w = 1;
  edges_marker.id = 0;
  edges_marker.type = visualization_msgs::Marker::LINE_LIST;
  edges_marker.color.r = edge_color.r;
  edges_marker.color.g = edge_color.g;
  edges_marker.color.b = edge_color.b;
  edges_marker.color.a = edge_color.a;
  edges_marker.scale.x = 0.02;
  edges_marker.lifetime = ros::Duration(1000.0);

  for (auto it = edge_map.begin(); it != edge_map.end(); ++it)
  {
    // TODO: Skip grasp points
    edges_marker.points.push_back(dmodel_points.poses[it->first.first].position);
    edges_marker.points.push_back(dmodel_points.poses[it->first.second].position);
  }
  marker_array.markers.push_back(edges_marker);

  PublishMarkerArray(marker_array);
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
  marker.ns = "force";
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation.x = marker.pose.orientation.y = marker.pose.orientation.z = 0;
  marker.pose.orientation.w = 1;
  marker.id = 0;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.scale.x = 0.04;
  marker.scale.y = 0.12;
  marker.scale.z = 0.0;
  marker.color.r = color.r;
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

void LTMViz::VisualizeAxis(const geometry_msgs::Pose axis)
{
  ltm::RGBA color(1.0, 0.0, 0.0, 0.5);
  visualization_msgs::Marker marker;
  marker.header.frame_id = reference_frame_;
  marker.header.stamp = ros::Time::now();
  marker.ns = "articulation_axis";
  marker.action = visualization_msgs::Marker::ADD;
  marker.id = 0;
  marker.type = visualization_msgs::Marker::CYLINDER;
  marker.pose.position.x = axis.position.x;
  marker.pose.position.y = axis.position.y;
  marker.pose.position.z = axis.position.z;
  marker.pose.orientation.x = axis.orientation.x;
  marker.pose.orientation.y = axis.orientation.y;
  marker.pose.orientation.z = axis.orientation.z;
  marker.pose.orientation.w = axis.orientation.w;
  marker.scale.x = 0.05;
  marker.scale.y = 0.05;
  marker.scale.z = 0.8; 
  marker.color.r = color.r;
  marker.color.g = color.g;
  marker.color.b = color.b;
  marker.color.a = color.a;
  marker.lifetime = ros::Duration(100.0);
  PublishMarker(marker);
}

void LTMViz::VisualizeTraj(const geometry_msgs::PoseArray traj)
{
  ltm::RGBA color(0.0, 1.0, 0.0, 0.5);
  visualization_msgs::Marker marker;
  marker.header.frame_id = reference_frame_;
  marker.header.stamp = ros::Time::now();
  marker.ns = "trajectory";
  marker.action = visualization_msgs::Marker::ADD;
  marker.id = 0;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0;
  marker.pose.orientation.y = 0;
  marker.pose.orientation.z = 0;
  marker.pose.orientation.w = 1;
  marker.scale.x = 0.02;
  marker.scale.y = 0.02;
  marker.scale.z = 0.02; 
  marker.color.r = color.r;
  marker.color.g = color.g;
  marker.color.b = color.b;
  marker.color.a = color.a;
  marker.lifetime = ros::Duration(100.0);
  for (int ii = 0; ii < traj.poses.size(); ++ii)
  {
    marker.points.push_back(traj.poses[ii].position);
  }
  PublishMarker(marker);
}

void LTMViz::VisualizePolygon(const geometry_msgs::Polygon poly, std::string name)
{
  ltm::RGBA color(1.0, 0.0, 0.0, 1.0);
  visualization_msgs::Marker marker;
  marker.header.frame_id = reference_frame_;
  marker.header.stamp = ros::Time::now();
  marker.ns = name;
  marker.action = visualization_msgs::Marker::ADD;
  marker.id = 0;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  geometry_msgs::Point p;
  for (int ii = 0; ii < poly.points.size(); ++ii)
  {
    p.x = poly.points[ii].x;
    p.y = poly.points[ii].y;
    p.z = poly.points[ii].z;
    marker.points.push_back(p);
  }
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.01; 
  marker.color.r = color.r;
  marker.color.g = color.g;
  marker.color.b = color.b;
  marker.color.a = color.a;
  marker.lifetime = ros::Duration(100.0);
  PublishMarker(marker);
}

void LTMViz::PublishMarker(visualization_msgs::Marker& marker)
{
  marker_publisher_.publish(marker);
  //usleep(10000);
}

void LTMViz::PublishMarkerArray(visualization_msgs::MarkerArray& marker_array)
{
  marker_array_publisher_.publish(marker_array);
  //usleep(1000);
}
