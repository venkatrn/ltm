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

void LTMViz::VisualizeAxis(const tf::Vector3 axis, const tf::Vector3 axis_point)
{

  const double kArrowLength = 1.0;
  // Visualize the axis
  geometry_msgs::Point start_point, end_point;
  tf::Vector3 end = axis_point + kArrowLength*axis.normalized();
  start_point.x = axis_point.x();
  start_point.y = axis_point.y();
  start_point.z = axis_point.z();
  end_point.x = end.x();
  end_point.y = end.y();
  end_point.z = end.z();

  visualization_msgs::Marker axis_marker;
  axis_marker.header.frame_id = reference_frame_;
  axis_marker.header.stamp = ros::Time::now();
  axis_marker.ns = string("axis") + to_string(rand());
  axis_marker.action = visualization_msgs::Marker::ADD;
  axis_marker.id = 0;
  axis_marker.type = visualization_msgs::Marker::ARROW;
  axis_marker.points.push_back(start_point);
  axis_marker.points.push_back(end_point);
  axis_marker.scale.x = 0.015;
  axis_marker.scale.y = 0.04;
  axis_marker.scale.z = 0.04;
  axis_marker.color.r = 1.0;
  axis_marker.color.g = 0.0;
  axis_marker.color.b = 0.0;
  axis_marker.color.a = 1.0;
  axis_marker.lifetime = ros::Duration(100.0);
  PublishMarker(axis_marker);
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
  for (size_t ii = 0; ii < traj.poses.size(); ++ii)
  {
    marker.points.push_back(traj.poses[ii].position);
  }
  PublishMarker(marker);
}

void LTMViz::VisualizePolygon(const geometry_msgs::Polygon poly, std::string name)
{
  // Note: last point == first point
  if (poly.points.size() < 4)
  {
    ROS_INFO("[LTMViz]: Polygon has less than 3 points");
    return;
  }
  //ltm::RGBA normal_color(20.0/255, 73.0/255, 85.0/255, 1.0);
  ltm::RGBA normal_color(73.0/255, 0.0/255, 77.0/255, 1.0);
  ltm::RGBA color(241.0/255, 76.0/255, 56.0/255, 1.0);

  visualization_msgs::Marker marker;
  marker.header.frame_id = reference_frame_;
  marker.header.stamp = ros::Time::now();
  marker.ns = name;
  marker.action = visualization_msgs::Marker::ADD;
  marker.id = 0;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  geometry_msgs::Point p, centroid;
  centroid.x = 0; centroid.y = 0; centroid.z = 0;
  for (size_t ii = 0; ii < poly.points.size(); ++ii)
  {
    p.x = poly.points[ii].x;
    p.y = poly.points[ii].y;
    p.z = poly.points[ii].z;
    marker.points.push_back(p);
    if (ii != poly.points.size() - 1)
    {
      centroid.x = centroid.x + p.x;
      centroid.y = centroid.y + p.y;
      centroid.z = centroid.z + p.z;
    }
  }
  centroid.x = centroid.x / (poly.points.size()-1);
  centroid.y = centroid.y / (poly.points.size()-1);
  centroid.z = centroid.z / (poly.points.size()-1);
  marker.scale.x = 0.02;
  marker.scale.y = 0.02;
  marker.scale.z = 0.02; 
  marker.color.r = color.r;
  marker.color.g = color.g;
  marker.color.b = color.b;
  marker.color.a = color.a;
  marker.lifetime = ros::Duration(100.0);
  PublishMarker(marker);


  // Visualize the normal to the polygon
  geometry_msgs::Point normal, v1, v2;
  v1.x = centroid.x - poly.points[0].x;
  v1.y = centroid.y - poly.points[0].y;
  v1.z = centroid.z - poly.points[0].z;
  v2.x = centroid.x - poly.points[1].x;
  v2.y = centroid.y - poly.points[1].y;
  v2.z = centroid.z - poly.points[1].z;
  normal.x = v1.y*v2.z - v1.z*v2.y;
  normal.y = -v1.x*v2.z + v1.z*v2.x;
  normal.z = v1.x*v2.y - v1.y*v2.x;
  const double norm = Norm(normal);
  const double kArrowLength = max(norm, 0.25);
  geometry_msgs::Point end_point;
  end_point.x = centroid.x + kArrowLength*normal.x/max(1e-5, norm);
  end_point.y = centroid.y + kArrowLength*normal.y/max(1e-5, norm);
  end_point.z = centroid.z + kArrowLength*normal.z/max(1e-5, norm);

  visualization_msgs::Marker normal_marker;
  normal_marker.header.frame_id = reference_frame_;
  normal_marker.header.stamp = ros::Time::now();
  normal_marker.ns = name + "_normal";
  normal_marker.action = visualization_msgs::Marker::ADD;
  normal_marker.id = 0;
  normal_marker.type = visualization_msgs::Marker::ARROW;
  normal_marker.points.push_back(centroid);
  normal_marker.points.push_back(end_point);
  normal_marker.scale.x = 0.015;
  normal_marker.scale.y = 0.04;
  normal_marker.scale.z = 0.04;
  normal_marker.color.r = normal_color.r;
  normal_marker.color.g = normal_color.g;
  normal_marker.color.b = normal_color.b;
  normal_marker.color.a = normal_color.a;
  normal_marker.lifetime = ros::Duration(100.0);
  PublishMarker(normal_marker);

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
