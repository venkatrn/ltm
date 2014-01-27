/**
 * @file d_model.cpp
 * @author Venkatraman Narayanan
 * Carnegie Mellon University, 2013
 */

#include <ltm/d_model.h>

#include <Eigen/Core>

#include <cstdio>
#include <iostream>
#include <set>
#include <queue>

// Multiplier for edge costs to avoid working with floating point numbers.
const double kCostMultiplier = 1e3;
// Error tolerance for comparing goal point locations.
const double kGoalTolerance = 0.5;

using namespace std;

DModel::DModel(const string& reference_frame)
{
  reference_frame_ = reference_frame;
  edge_map_ = new unordered_map<Edge, EdgeParams, pair_hash>;

  points_pub_ = nh.advertise<visualization_msgs::Marker>("d_model_points", 5, true);
  edges_pub_ = nh.advertise<visualization_msgs::Marker>("d_model_edges", 5, true);
  force_pub_ = nh.advertise<visualization_msgs::Marker>("d_model_forces", 5);

  // Initialize env params
  env_cfg_.start_state_id = -1;
  env_cfg_.sim_time_step = 0.1;
}

DModel::DModel()
{
  DModel("/base_link");
}


DModel::~DModel() 
{
  if (edge_map_ != NULL)
  {
    delete edge_map_;
    edge_map_ = NULL;
  }
}

void DModel::InitFromFile(const char* dmodel_file)
{
  InitFromFile(dmodel_file, 0.0, 0.0, 0.0);
  return;
}

void DModel::InitFromFile(const char* dmodel_file, double shift_x, double shift_y, double shift_z)
{
  FILE* f_dmodel = fopen(dmodel_file,"r");
  int num_points, num_edges, idx1, idx2, j_t;
  float dir_x, dir_y, dir_z, rad;
  float x, y, z, o_x, o_y, o_z, o_w;
  geometry_msgs::Pose p;
  EdgeParams e_params;
  if (f_dmodel == NULL) 
  {
    printf("Unable to open dmodel file\n");
    return;
  }

  char s_temp[1024];
  if (fscanf(f_dmodel, "%s", s_temp) < 1)
  {
    printf("String length < 1\n");
  }
  if (strcmp(s_temp, "points:") !=0)
  {
    printf("Incorrect format for dmodel file\n");
  }
  if (fscanf(f_dmodel, "%s", s_temp) < 1)
  {
    printf("String length < 1\n");
  }
  num_points = atoi(s_temp);

  if (fscanf(f_dmodel, "%s", s_temp) < 1)
  {
    printf("String length < 1\n");
  }
  if (strcmp(s_temp, "edges:") !=0)
  {
    printf("Incorrect format for dmodel file\n");
  }
  if (fscanf(f_dmodel, "%s", s_temp) < 1)
  {
    printf("String length < 1\n");
  }
  num_edges = atoi(s_temp);
  printf("Reading model file with %d points and %d edges\n", num_points, num_edges);
  for (int ii = 0; ii < num_points; ++ii)
  {
    if (fscanf(f_dmodel, "%d %f %f %f %f %f %f %f\n", &idx1, &x, &y, &z,
        &o_x, &o_y, &o_z, &o_w) != 8) 
    {
      printf("Error reading points d-model file\n");
      return;
    }
    p.position.x = x + shift_x;
    p.position.y = y + shift_y;
    p.position.z = z + shift_z;
    p.orientation.x = o_x;
    p.orientation.y = o_y;
    p.orientation.z = o_z;
    p.orientation.w = o_w;
    
    AddPoint(p);
  }
  // Edges must be added only after all points have been added.
  for (int ii = 0; ii < num_edges; ++ii)
  {
    if (fscanf(f_dmodel, "%d %d %d %f %f %f %f\n", &idx1, &idx2, &j_t, &dir_x, &dir_y, &dir_z, &rad) != 7)
    {
      printf("Error reading edges d-model file\n");
      return;
    }
    e_params.joint = static_cast<JointType>(j_t);
    e_params.normal = tf::Vector3(dir_x, dir_y, dir_z);
    e_params.rad = rad;
    AddEdge(make_pair(idx1, idx2), e_params);
  } 
  fclose(f_dmodel);
  printf("Finished reading model file\n");
  TFCallback(points_);
  return;
}

void DModel::SetPoints(const geometry_msgs::PoseArray& points)
{
  points_ = points;
  return;
}

void DModel::InitForcePrimsFromFile(const char* fprims_file)
{
  FILE* f_fprims = fopen(fprims_file,"r");
  int num_fprims, idx;
  float f_x, f_y, f_z;
  geometry_msgs::Pose p;
  if (f_fprims == NULL) 
  {
    printf("Unable to open force primitives file\n");
    return;
  }

  char s_temp[1024];
  if (fscanf(f_fprims, "%s", s_temp) < 1)
  {
    printf("String length < 1\n");
  }
  if (strcmp(s_temp, "primitives:") !=0)
  {
    printf("Incorrect format for dmodel file\n");
  }
  if (fscanf(f_fprims, "%s", s_temp) < 1)
  {
    printf("String length < 1\n");
  }
  num_fprims = atoi(s_temp);

  printf("Reading force primitives file with %d primitives\n", num_fprims);

  for (int ii = 0; ii < num_fprims; ++ii)
  {
    if (fscanf(f_fprims, "%d %f %f %f\n", &idx, &f_x, &f_y, &f_z) != 4)
    {
      printf("Error reading force primitives file\n");
      return;
    }
    tf::Vector3 force(f_x, f_y, f_z);
    force_primitives_.push_back(force);
  } 
  fclose(f_fprims);
  printf("Finished reading force primitives file\n");
  return;
}

void DModel::AddEdge(Edge e, EdgeParams e_params) 
{
  if (adj_list_.size() == 0)
  {
    adj_list_.resize(points_.poses.size());
  }
  (*edge_map_)[e] = e_params;
  // TODO: Check for duplicates
  adj_list_[e.first].push_back(e.second);
  adj_list_[e.second].push_back(e.first);
  return;
}

void DModel::AddPoint(geometry_msgs::Pose p)
{
  points_.poses.push_back(p);
  return;
}

void DModel::TFTimedCallback(const ros::TimerEvent& event)
{
  TFCallback(points_);
  return;
}

void DModel::TFCallback(geometry_msgs::PoseArray dmodel_points)
{
  tf::Transform transform;

  visualization_msgs::Marker points;
  points.header.frame_id = reference_frame_;
  points.header.stamp = ros::Time::now();
  points.ns = "points";
  points.action = visualization_msgs::Marker::ADD;
  points.pose.orientation.x = points.pose.orientation.y = points.pose.orientation.z = 0;
  points.pose.orientation.w = 1;
  points.id = 0;
  points.type = visualization_msgs::Marker::SPHERE_LIST;
  points.scale.x = points.scale.y = points.scale.z = 0.1;
  points.color.g = points.color.b =  0.0;
  points.color.r = 1.0;
  points.color.a = 1.0;

  for (size_t ii = 0; ii < points_.poses.size(); ++ii)
  {
    static tf::TransformBroadcaster tf_br_;
    geometry_msgs::Pose p = dmodel_points.poses[ii];
    points.points.push_back(p.position);
    transform.setOrigin(tf::Vector3(p.position.x, p.position.y, p.position.z) );
    transform.setRotation(tf::Quaternion(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w));
    string child_frame_id = to_string(ii);
    tf_br_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), reference_frame_.c_str(), child_frame_id.c_str()));
  }

  // Publish points
  points_pub_.publish(points);

  // Visualize edges
  visualization_msgs::Marker edges;
  edges.header.frame_id = reference_frame_;
  edges.header.stamp = ros::Time::now();
  edges.ns = "edges";
  edges.action = visualization_msgs::Marker::ADD;
  edges.pose.orientation.x = edges.pose.orientation.y = edges.pose.orientation.z = 0;
  edges.pose.orientation.w = 1;
  edges.id = 0;
  edges.type = visualization_msgs::Marker::LINE_LIST;
  edges.scale.x = 0.02;
  edges.color.r = edges.color.g =  0.0;
  edges.color.b = 1.0;
  edges.color.a = 1.0;

  for (auto it = edge_map_->begin(); it != edge_map_->end(); ++it)
  {
    edges.points.push_back(dmodel_points.poses[it->first.first].position);
    edges.points.push_back(dmodel_points.poses[it->first.second].position);
  }
  edges_pub_.publish(edges);

  return;
}

void DModel::ApplyForce(int p_idx, tf::Vector3 force, double del_t)
{
  if (p_idx < 0 || p_idx >= int(points_.poses.size()))
  {
    printf("Invalid point index\n");
    return;
  }
  vector<int> component_idxs, sep_idxs;
  JointType joint_type;
  tf::Vector3 normal;
  ExtractIndices(p_idx, &component_idxs, &sep_idxs, &joint_type, &normal);
  /*
  printf("Rigid component has %d points. Joint type is %d and param vector is %0.2f %0.2f %0.2f\n",
      (int)component_idxs.size(), static_cast<int>(joint_type), normal.x(), 
      normal.y(), normal.z());
      */

  int closest_p_idx = -1;
  if (sep_idxs.size() != 0) 
  {
    // Find the separating point with the closest distance 
    geometry_msgs::Point p = points_.poses[p_idx].position;
    double min_dist = 100000;
    for (size_t ii = 0; ii < sep_idxs.size(); ++ii)
    {
      double distance = Dist(p, points_.poses[ii].position);
      if (distance < min_dist)
      {
        min_dist = distance;
        closest_p_idx = ii;
      }
    }
  }

  if (joint_type == RIGID)
  {
    //TODO: Assumes unit mass for all points
    double mass = 1.0;
    tf::Vector3 displacement = 0.5*Sqr(del_t)*force/mass;
    for (size_t ii = 0; ii < component_idxs.size(); ++ii)
    {
      points_.poses[component_idxs[ii]].position.x = points_.poses[component_idxs[ii]].position.x + displacement.x();
      points_.poses[component_idxs[ii]].position.y = points_.poses[component_idxs[ii]].position.y + displacement.y();
      points_.poses[component_idxs[ii]].position.z = points_.poses[component_idxs[ii]].position.z + displacement.z();
    }
  }
  else if (joint_type == PRISMATIC)
  {
    //TODO: Assumes unit mass for all points
    double mass = 1.0;
    tf::Stamped<tf::Vector3> transformed_force;
    tf::Stamped<tf::Vector3> ref_force(force, ros::Time::now(), reference_frame_);
    string target_frame = to_string(closest_p_idx);
    // Transform force in reference frame to target frame
    listener_.transformVector(target_frame.c_str(), ros::Time(0), ref_force, reference_frame_.c_str(), transformed_force);
    // Project the force onto the constraint direction
    double projection = normal.x()*transformed_force.x() + normal.y()*transformed_force.y() + normal.z()*transformed_force.z();
    tf::Vector3 projected_vector(projection*normal.x(), projection*normal.y(), projection*normal.z());
    // Transform back the projected force to reference frame
    tf::Stamped<tf::Vector3> projected_force(projected_vector, ros::Time::now(), target_frame);
    listener_.transformVector(reference_frame_.c_str(), ros::Time(0), projected_force, target_frame.c_str(), transformed_force);

    tf::Vector3 displacement = 0.5*Sqr(del_t)*transformed_force/mass;

    for (size_t ii = 0; ii < component_idxs.size(); ++ii)
    {
      points_.poses[component_idxs[ii]].position.x = points_.poses[component_idxs[ii]].position.x + displacement.x();
      points_.poses[component_idxs[ii]].position.y = points_.poses[component_idxs[ii]].position.y + displacement.y();
      points_.poses[component_idxs[ii]].position.z = points_.poses[component_idxs[ii]].position.z + displacement.z();
    }
  }
  else if (joint_type == REVOLUTE)
  {
    //TODO: Assumes unit inertia for the rigid body
    double inertia = 1.0;
    tf::Stamped<tf::Vector3> transformed_force;
    tf::Stamped<tf::Vector3> ref_force(force, ros::Time::now(), reference_frame_);
    string target_frame = to_string(closest_p_idx);
    // Transform force in reference frame to target frame
    bool transformed = false;
    while (!transformed)
    {
      try
      {
        listener_.transformVector(target_frame.c_str(), ros::Time(0), ref_force, reference_frame_.c_str(), transformed_force);
      }
      catch (tf::LookupException)
      { 
        continue;
      }
      transformed = true;
    }
    // Moment center and arm
    tf::Vector3 center(points_.poses[closest_p_idx].position.x, points_.poses[closest_p_idx].position.y, points_.poses[closest_p_idx].position.z);
    tf::Vector3 p_applied(points_.poses[p_idx].position.x, points_.poses[p_idx].position.y, points_.poses[p_idx].position.z);
    tf::Vector3 arm = p_applied - center;

    // Project the force onto the tangent direction
    tf::Vector3 tangent = Cross(normal, arm);
    double projection = tangent.x()*transformed_force.x() + tangent.y()*transformed_force.y() + tangent.z()*transformed_force.z();
    tf::Vector3 projected_vector(projection*tangent.x(), projection*tangent.y(), projection*tangent.z());
    tf::Vector3 torque = Cross(arm, projected_vector);
    double torque_norm = sqrt(Sqr(torque.x()) + Sqr(torque.y()) + Sqr(torque.z()));

    double theta = 0.5*Sqr(del_t)*torque_norm/inertia;
    tf::Quaternion quat;
    // Return
    if (torque_norm < 1e-5) 
    {
      return;
      printf("Applied torque is zero - dividing by zero\n");
    }
    quat.setRotation(torque/torque_norm, theta);
    tf::Transform tr = tf::Transform(quat);

    for (size_t ii = 0; ii < component_idxs.size(); ++ii)
    {
      geometry_msgs::Point p = points_.poses[component_idxs[ii]].position;
      geometry_msgs::Quaternion q = points_.poses[component_idxs[ii]].orientation;
      tf::Vector3 point(p.x, p.y, p.z);
      tf::Vector3 rotated_point = tr*(point - center) + center;
      points_.poses[component_idxs[ii]].position.x = rotated_point.x();
      points_.poses[component_idxs[ii]].position.y = rotated_point.y();
      points_.poses[component_idxs[ii]].position.z = rotated_point.z();
      tf::Quaternion original_quat(q.x, q.y, q.z, q.w);
      tf::Quaternion rotated_quat = quat * original_quat;
      points_.poses[component_idxs[ii]].orientation.x = double(rotated_quat.x());
      points_.poses[component_idxs[ii]].orientation.y = double(rotated_quat.y());
      points_.poses[component_idxs[ii]].orientation.z = double(rotated_quat.z());
      points_.poses[component_idxs[ii]].orientation.w = double(rotated_quat.w());
    }
  }
  TFCallback(points_);
  return;
}

void DModel::ExtractIndices(int p_idx, vector<int>* component_idxs, vector<int>* sep_idxs, JointType* joint_type,
    tf::Vector3* normal)
{
  component_idxs->clear();
  sep_idxs->clear();
  *joint_type = RIGID;
  *normal = tf::Vector3(0.0, 0.0, 0.0);

  queue<int> o_list;
  set<int> c_list, e_list;
  o_list.push(p_idx);
  while (o_list.size() != 0)
  {
    int idx = o_list.front();
    o_list.pop();
    e_list.erase(idx);
    c_list.insert(idx);
    component_idxs->push_back(idx);

    vector<int> adj_points;
    GetAdjPoints(idx, &adj_points);
    for (size_t ii = 0; ii < adj_points.size(); ++ii) 
    {
      // Skip if the successor is already in open list or closed list
      if (e_list.find(adj_points[ii]) != e_list.end() || 
          c_list.find(adj_points[ii]) != c_list.end())
        continue;
      EdgeParams e_params;
      if (edge_map_->find(make_pair(idx, adj_points[ii])) != edge_map_->end())
      {
        e_params = edge_map_->at(make_pair(idx, adj_points[ii]));
      }
      else
      {
        e_params = edge_map_->at(make_pair(adj_points[ii], idx));
      }
      // Add successor to separator list if not a rigid connection
      if (e_params.joint != RIGID) 
      {
        sep_idxs->push_back(adj_points[ii]);
        // TODO: Check for conflicting joint types
        *joint_type = e_params.joint;
        *normal = e_params.normal;
        continue;
      }
      // Add to open list and update the frontier list
      o_list.push(adj_points[ii]);
      e_list.insert(adj_points[ii]);
    }
  }
  return;
}

void DModel::GetAdjPoints(int p_idx, std::vector<int> *adj_points)
{
  adj_points->clear();
  for (size_t ii = 0; ii < adj_list_[p_idx].size(); ++ii)
  {
    adj_points->push_back(adj_list_[p_idx][ii]);
  }
  return;
}


void DModel::GetNextState(const geometry_msgs::PoseArray& in_points, int p_idx, tf::Vector3 force, double del_t, geometry_msgs::PoseArray *out_points)
{

  if (points_.poses.size() != in_points.poses.size())
  {
    printf("Number of points in current state and internal model do not match\n");
    return;
  }

  if (p_idx < 0 || p_idx >= int(points_.poses.size()))
  {
    printf("Invalid point index\n");
    return;
  }

  out_points->poses.clear();
  *out_points = in_points;
  vector<int> component_idxs, sep_idxs;
  JointType joint_type;
  tf::Vector3 normal;
  ExtractIndices(p_idx, &component_idxs, &sep_idxs, &joint_type, &normal);

  /*
  printf("Rigid component has %d points. Joint type is %d and param vector is %0.2f %0.2f %0.2f\n",
      (int)component_idxs.size(), static_cast<int>(joint_type), normal.x(), 
      normal.y(), normal.z());
      */

  int closest_p_idx = -1;
  if (sep_idxs.size() != 0) 
  {
    // Find the separating point with the closest distance 
    geometry_msgs::Point p = in_points.poses[p_idx].position;
    double min_dist = 100000;
    for (size_t ii = 0; ii < sep_idxs.size(); ++ii)
    {
      double distance = Dist(p, in_points.poses[ii].position);
      if (distance < min_dist)
      {
        min_dist = distance;
        closest_p_idx = ii;
      }
    }
  }

  if (joint_type == RIGID)
  {
    //TODO: Assumes unit mass for all points
    double mass = 1.0;
    tf::Vector3 displacement = 0.5*Sqr(del_t)*force/mass;
    for (size_t ii = 0; ii < component_idxs.size(); ++ii)
    {
      out_points->poses[component_idxs[ii]].position.x = out_points->poses[component_idxs[ii]].position.x + displacement.x();
      out_points->poses[component_idxs[ii]].position.y = out_points->poses[component_idxs[ii]].position.y + displacement.y();
      out_points->poses[component_idxs[ii]].position.z = out_points->poses[component_idxs[ii]].position.z + displacement.z();
    }
  }
  else if (joint_type == PRISMATIC)
  {
    //TODO: Assumes unit mass for all points
    double mass = 1.0;
    tf::Stamped<tf::Vector3> transformed_force;
    tf::Stamped<tf::Vector3> ref_force(force, ros::Time::now(), reference_frame_);
    string target_frame = to_string(closest_p_idx);
    // Transform force in reference frame to target frame
    listener_.transformVector(target_frame.c_str(), ros::Time(0), ref_force, reference_frame_.c_str(), transformed_force);
    // Project the force onto the constraint direction
    double projection = normal.x()*transformed_force.x() + normal.y()*transformed_force.y() + normal.z()*transformed_force.z();
    tf::Vector3 projected_vector(projection*normal.x(), projection*normal.y(), projection*normal.z());
    // Transform back the projected force to reference frame
    tf::Stamped<tf::Vector3> projected_force(projected_vector, ros::Time::now(), target_frame);
    listener_.transformVector(reference_frame_.c_str(), ros::Time(0), projected_force, target_frame.c_str(), transformed_force);

    tf::Vector3 displacement = 0.5*Sqr(del_t)*transformed_force/mass;

    for (size_t ii = 0; ii < component_idxs.size(); ++ii)
    {
      out_points->poses[component_idxs[ii]].position.x = out_points->poses[component_idxs[ii]].position.x + displacement.x();
      out_points->poses[component_idxs[ii]].position.y = out_points->poses[component_idxs[ii]].position.y + displacement.y();
      out_points->poses[component_idxs[ii]].position.z = out_points->poses[component_idxs[ii]].position.z + displacement.z();
    }
  }
  else if (joint_type == REVOLUTE)
  {
    //TODO: Assumes unit inertia for the rigid body
    double inertia = 1.0;
    tf::Stamped<tf::Vector3> transformed_force;
    tf::Stamped<tf::Vector3> ref_force(force, ros::Time::now(), reference_frame_);
    string target_frame = to_string(closest_p_idx);
    // Transform force in reference frame to target frame
    bool transformed = false;
    while (!transformed)
    {
      try
      {
        listener_.transformVector(target_frame.c_str(), ros::Time(0), ref_force, reference_frame_.c_str(), transformed_force);
      }
      catch (tf::LookupException)
      { 
        continue;
      }
      transformed = true;
    }
    // Moment center and arm
    tf::Vector3 center(out_points->poses[closest_p_idx].position.x, out_points->poses[closest_p_idx].position.y, out_points->poses[closest_p_idx].position.z);
    tf::Vector3 p_applied(out_points->poses[p_idx].position.x, out_points->poses[p_idx].position.y, out_points->poses[p_idx].position.z);
    tf::Vector3 arm = p_applied - center;

    // Project the force onto the tangent direction
    tf::Vector3 tangent = Cross(normal, arm);
    double projection = tangent.x()*transformed_force.x() + tangent.y()*transformed_force.y() + tangent.z()*transformed_force.z();
    tf::Vector3 projected_vector(projection*tangent.x(), projection*tangent.y(), projection*tangent.z());
    tf::Vector3 torque = Cross(arm, projected_vector);
    double torque_norm = sqrt(Sqr(torque.x()) + Sqr(torque.y()) + Sqr(torque.z()));

    double theta = 0.5*Sqr(del_t)*torque_norm/inertia;
    tf::Quaternion quat;
    // Return
    if (torque_norm < 1e-5) 
    {
      return;
      printf("Applied torque is zero - dividing by zero\n");
    }
    quat.setRotation(torque/torque_norm, theta);
    tf::Transform tr = tf::Transform(quat);

    for (size_t ii = 0; ii < component_idxs.size(); ++ii)
    {
      geometry_msgs::Point p = out_points->poses[component_idxs[ii]].position;
      geometry_msgs::Quaternion q = out_points->poses[component_idxs[ii]].orientation;
      tf::Vector3 point(p.x, p.y, p.z);
      tf::Vector3 rotated_point = tr*(point - center) + center;
      out_points->poses[component_idxs[ii]].position.x = rotated_point.x();
      out_points->poses[component_idxs[ii]].position.y = rotated_point.y();
      out_points->poses[component_idxs[ii]].position.z = rotated_point.z();

      // Update orientation of the point (the local coordinate frame) by passing it
      // through the same transformation.
      tf::Quaternion original_quat(q.x, q.y, q.z, q.w);
      tf::Quaternion rotated_quat = quat * original_quat;
      out_points->poses[component_idxs[ii]].orientation.x = double(rotated_quat.x());
      out_points->poses[component_idxs[ii]].orientation.y = double(rotated_quat.y());
      out_points->poses[component_idxs[ii]].orientation.z = double(rotated_quat.z());
      out_points->poses[component_idxs[ii]].orientation.w = double(rotated_quat.w());
    }
  }
  return;
}

void DModel::SimulatePlan(const vector<int>& fprim_ids)
{
  vector<tf::Vector3> forces;
  ConvertForcePrimIDsToForces(fprim_ids, &forces);
  SimulatePlan(forces);
  return;
}

void DModel::SimulatePlan(const vector<tf::Vector3>& forces)
{
  geometry_msgs::PoseArray in_points = points_;
  for (size_t ii = 0; ii < forces.size(); ++ii)
  {
    geometry_msgs::PoseArray out_points;
    GetNextState(in_points, force_idx_, forces[ii], env_cfg_.sim_time_step, &out_points);
    TFCallback(out_points);

    // Visualize force
    visualization_msgs::Marker force;
    force.header.frame_id = reference_frame_;
    force.header.stamp = ros::Time::now();
    force.ns = "force";
    force.action = visualization_msgs::Marker::ADD;
    force.pose.orientation.x = force.pose.orientation.y = force.pose.orientation.z = 0;
    force.pose.orientation.w = 1;
    force.id = 0;
    force.type = visualization_msgs::Marker::ARROW;
    force.scale.x = 0.04;
    force.scale.y = 0.12;
    force.scale.z = 0.0;
    force.color.r = force.color.b =  0.0;
    force.color.g = 1.0;
    force.color.a = 1.0;
    geometry_msgs::Point start_point = out_points.poses[force_idx_].position;
    geometry_msgs::Point end_point;
    const double normalizer = Norm(forces[ii]);
    end_point.x = start_point.x + forces[ii].x()/normalizer;
    end_point.y = start_point.y + forces[ii].y()/normalizer;
    end_point.z = start_point.z + forces[ii].z()/normalizer;
    force.points.push_back(start_point);
    force.points.push_back(end_point);
    force_pub_.publish(force);

    in_points = out_points;
    usleep(100000);
  }
  return;
}

void DModel::VisualizeState(int state_id)
{
  State_t s = StateIDToState(state_id);
  geometry_msgs::PoseArray points;
  for (size_t ii = 0; ii < points_.poses.size(); ++ii)
  {
    // If point has changed, then use the coordinates from state.
    auto it = find(s.changed_inds.begin(),
        s.changed_inds.end(),
        ii);
    if ( it != s.changed_inds.end())
    {
      int offset = distance(s.changed_inds.begin(), it);
      points.poses.push_back(s.changed_points.poses[offset]);
    }
    else
    {
      points.poses.push_back(points_.poses[ii]);
    }
  }
  TFCallback(points);
  return;
}

int DModel::StateToStateID(State_t& s)
{
  // If state has already been created, return ID from hash map
  for (auto it = StateMap.begin(); it != StateMap.end(); ++it)
  {
    if (s == it->second)
    {
      return it->first;
    }
  }
  // Otherwise, create state, add to hash map, and return ID
  int new_id = int(StateMap.size());
  StateMap[new_id] = s;
  //StateMap.insert(make_pair<int, State_t>(new_id, s));
  return new_id;
}

State_t DModel::StateIDToState(int state_id)
{
  auto it = StateMap.find(state_id);
  if (it != StateMap.end())
  {
    return it->second;
  }
  else
  {
    printf("Requested State ID does not exist\n");
  }
}

void DModel::GetSuccs(int source_state_id, vector<int>* succs, vector<double>* costs)
{
  succs->clear();
  costs->clear();

  // Goal state should be absorbing.
  if (IsGoalState(source_state_id))
  {
    return;
  }

  // VisualizeState(source_state_id);
  // usleep(100000);

  State_t source_state = StateIDToState(source_state_id);

  for (size_t jj = 0; jj < force_primitives_.size(); ++jj) {
    // Construct points in the state
    geometry_msgs::PoseArray in_points;
    for (size_t ii = 0; ii < points_.poses.size(); ++ii)
    {
      // If point has changed, then use the coordinates from state.
      auto it = find(source_state.changed_inds.begin(),
          source_state.changed_inds.end(),
          ii);
      if ( it != source_state.changed_inds.end())
      {
        int offset = distance(source_state.changed_inds.begin(), it);
        in_points.poses.push_back(source_state.changed_points.poses[offset]);
      }
      else
      {
        in_points.poses.push_back(points_.poses[ii]);
      }
    }
    geometry_msgs::PoseArray out_points;
    tf::Vector3 force = force_primitives_[jj];
    GetNextState(in_points, force_idx_, force, env_cfg_.sim_time_step, &out_points);

    // Determine points that have changed and generate succ state accordingly.
    State_t succ_state;
    for (size_t ii = 0; ii < points_.poses.size(); ++ii)
    {
      if (fabs(points_.poses[ii].position.x-out_points.poses[ii].position.x) >= kFPTolerance ||
          fabs(points_.poses[ii].position.y-out_points.poses[ii].position.y) >= kFPTolerance ||
          fabs(points_.poses[ii].position.z-out_points.poses[ii].position.z) >= kFPTolerance) 
      {
        succ_state.changed_points.poses.push_back(out_points.poses[ii]); 
        succ_state.changed_inds.push_back(ii); 
      }
    }

    // DEBUG
    // printf("Number of changed points: %d\n", int(succ_state.changed_inds.size()));

    int succ_id = StateToStateID(succ_state);

    succs->push_back(succ_id);
    // TODO(venkat): Compute costs
    // costs->push_back(1);
    // costs->push_back(int(kCostMultiplier * Norm(force)));
    // TODO(venkat): Compute distance traveled by end-effector, so that
    // we can compute power = force x velocity
    // Cost is time
    costs->push_back(int(kCostMultiplier * env_cfg_.sim_time_step));
  }
  return;
}

bool DModel::IsGoalState(int state_id)
{
  State_t s = StateIDToState(state_id);
  State_t goal_state = StateIDToState(env_cfg_.goal_state_id);

  for (size_t ii = 0; ii < goal_state.changed_inds.size(); ++ii)
  {
    auto it = std::find(s.changed_inds.begin(), s.changed_inds.end(), goal_state.changed_inds[ii]);
    if (it == s.changed_inds.end())
    {
      return false;
    }
    int offset = distance(s.changed_inds.begin(), it);


    if (fabs(goal_state.changed_points.poses[ii].position.x-s.changed_points.poses[offset].position.x) >= kGoalTolerance ||
        fabs(goal_state.changed_points.poses[ii].position.y-s.changed_points.poses[offset].position.y) >= kGoalTolerance ||
        fabs(goal_state.changed_points.poses[ii].position.z-s.changed_points.poses[offset].position.z) >= kGoalTolerance)
    {
      return false;
    }
    //TODO(venkat): Check orientations also
  }
  return true;
}

double DModel::GetGoalHeuristic(int state_id)
{
  //TODO: This heuristic is to be used only when the grasp point does not change throughout the plan
  State_t s = StateIDToState(state_id);
  geometry_msgs::Pose grasp_pose;
  // If position of end-eff has changed, then use the coordinates from state.
  auto it = find(s.changed_inds.begin(),
      s.changed_inds.end(),
      force_idx_);
  if ( it != s.changed_inds.end())
  {
    int offset = distance(s.changed_inds.begin(), it);
    grasp_pose = s.changed_points.poses[offset];
  }
  else
  {
    grasp_pose = points_.poses[force_idx_];
  }
  //printf("Goal grasp: %f %f %f\n", grasp_pose.position.x, 
   //   grasp_pose.position.y, goal_grasp_pose.position.z);
  double heuristic = kCostMultiplier * Dist(grasp_pose.position, env_cfg_.goal_grasp_pose.position) / env_cfg_.sim_time_step;
  return heuristic;
}

void DModel::SetForceIndex(int force_idx)
{
  if (force_idx >= int(points_.poses.size()))
  {
    printf("DModel: Invalid force index\n");
    return;
  }
  force_idx_ = force_idx;
  return;
}

void DModel::SetStartState(State_t start_state)
{
  int start_state_id = StateToStateID(start_state);
  env_cfg_.start_state_id = start_state_id;
  return; 
}

void DModel::SetGoalState(State_t goal_state)
{
  int goal_state_id = StateToStateID(goal_state);
  env_cfg_.goal_state_id = goal_state_id;
  // TODO: This is only for single grasp plans.
  if (int(goal_state.changed_inds.size()) != 0)
  {
    env_cfg_.goal_grasp_pose = goal_state.changed_points.poses[0];
  }
  return; 
}

int DModel::GetStartStateID()
{
  return env_cfg_.start_state_id;
}

void DModel::SetSimTimeStep(double del_t)
{
  env_cfg_.sim_time_step = del_t;
  return;
}

bool DModel::ConvertForcePrimIDsToForces(const std::vector<int>& fprim_ids, std::vector<tf::Vector3>* forces)
{
  forces->clear();
  // Skip the start state
  for (size_t ii = 1; ii < fprim_ids.size(); ++ii)
  {
    if (fprim_ids[ii] >= int(force_primitives_.size()))
    {
      printf("DModel: Invalid force primitive ID while reconstructing forces\n");
      return false;
    }
    forces->push_back(force_primitives_[fprim_ids[ii]]);
  }
  return true;
}

bool DModel::GetEndEffectorTrajFromStateIDs(const std::vector<int>& state_ids,
    geometry_msgs::PoseArray* traj)
{
  traj->poses.clear();
  for (size_t ii = 0; ii < state_ids.size(); ++ii)
  {
    State_t s = StateIDToState(state_ids[ii]);
    auto it = std::find(s.changed_inds.begin(), s.changed_inds.end(), force_idx_);
    if (it != s.changed_inds.end())
    {
      const int offset = distance(s.changed_inds.begin(), it);
      traj->poses.push_back(s.changed_points.poses[offset]);
    }
    else
    {
      traj->poses.push_back(points_.poses[force_idx_]);
    }
  }
  return true;
}


void DModel::LearnDModelParameters(const vector<geometry_msgs::PoseArray>& observations,
  const vector<Edge>& edges)
{
  const int num_edges = int(edges.size());
  const int num_obs = int(observations.size());
  const int num_points = int(observations[0].poses.size());

  if (num_obs == 0)
  {
    printf("DModel: No observations to learn model from\n");
    return;
  }

  // Gaussian mean for each edge, for constraint vector, and for distance
  vector<tf::Vector3> constraint_means;
  constraint_means.resize(num_edges);
  vector<double> dist_means;
  dist_means.resize(num_edges);

  // Gaussian covariance for each edge, for constraint vector, and for distance
  vector<Eigen::Matrix3d> constraint_cov;
  constraint_cov.resize(num_edges);
  vector<double> dist_cov;
  dist_cov.resize(num_edges);

  // MLE for means
  for (int ii = 0; ii < num_edges; ++ii)
  {
    constraint_means[ii] = tf::Vector3(0.0, 0.0, 0.0);
    dist_means[ii] = 0.0;

    for (int jj = 0; jj < num_obs; ++jj)
    {
      tf::Vector3 constraint_vector = GetLocalVector(observations[jj].poses[edges[ii].first], observations[jj].poses[edges[ii].second]);

      // DEBUG
      printf("Local vector: %f %f %f\n", constraint_vector.x(),
          constraint_vector.y(), constraint_vector.z());
      const double constraint_vector_norm = Norm(constraint_vector);
      constraint_vector = constraint_vector / constraint_vector_norm;

      constraint_means[ii] = constraint_means[ii] + constraint_vector;
      dist_means[ii] = dist_means[ii] + constraint_vector_norm;
    }
    constraint_means[ii] = constraint_means[ii] / num_obs;
    dist_means[ii] = dist_means[ii] / num_obs;
  }

  // MLE for covariances
  for (int ii = 0; ii < num_edges; ++ii)
  {
    constraint_cov[ii] = Eigen::Matrix3d::Zero();
    dist_cov[ii] = 0.0;

    for (int jj = 0; jj < num_obs; ++jj)
    {
      // TODO: Clean up this code to make it non-repetitive
      tf::Vector3 constraint_vector = GetLocalVector(observations[jj].poses[edges[ii].first], observations[jj].poses[edges[ii].second]);
      const double constraint_vector_norm = Norm(constraint_vector);
      constraint_vector = constraint_vector / constraint_vector_norm;
      tf::Vector3 centered_constraint_vector = constraint_vector - constraint_means[ii];

      Eigen::Vector3d c_vec(centered_constraint_vector.x(),
          centered_constraint_vector.y(),
          centered_constraint_vector.z());
      Eigen::Matrix3d cov = c_vec * c_vec.transpose();
      constraint_cov[ii] = constraint_cov[ii] + cov;
      dist_cov[ii] = dist_cov[ii] + Sqr(constraint_vector_norm - dist_means[ii]);
    }
    constraint_cov[ii] = constraint_cov[ii] / num_obs;
    dist_cov[ii] = dist_cov[ii] / num_obs;
  }

  printf("MLE Estimates:\n");
  for (int ii = 0; ii < num_edges; ++ii)
  {
    printf("Constraint mean: %f %f %f\n", constraint_means[ii].x(),
        constraint_means[ii].y(), constraint_means[ii].z());
    cout << "Constraint cov:\n" << constraint_cov[ii] << endl;
    printf("Dist mean: %f\n", dist_means[ii]);
    printf("Dist cov: %f\n\n", dist_cov[ii]);
  }

  // We need to compute edge params for all edges
  vector<EdgeParams> edge_params;
  edge_params.resize(num_edges);


  vector<double> p_rigid, p_prismatic, p_revolute;
  p_rigid.resize(num_edges);
  p_prismatic.resize(num_edges);
  p_revolute.resize(num_edges);

  /*
  // Bayesian inference to assign parameters
  for (int ii = 0; ii < num_edges; ++ii)
  {
    p_rigid[ii] = 1.0;
    p_prismatic[ii] = 1.0;
    p_revolute[ii] = 1.0;

    for (int jj = 0; jj < num_obs; ++jj)
    {
      tf::Vector3 constraint_vector = GetLocalVector(observations[jj].poses[edges[ii].first], observations[jj].poses[edges[ii].second]);
      const double constraint_vector_norm = Norm(constraint_vector);
      constraint_vector = constraint_vector / constraint_vector_norm;
      Eigen::Vector3d c_vec(constraint_vector.x(),
                            constraint_vector.y(),
                            constraint_vector.z());
      Eigen::Vector3d c_mu(constraint_means[ii].x(),
                           constraint_means[ii].y(),
                           constraint_means[ii].z());

      const double p_constraint_vec = MultivariateNormalPDF(c_vec,
          c_mu, constraint_cov[ii]);
      const double p_dist = NormalPDF(constraint_vector_norm, dist_means[ii], dist_cov[ii]);
      const double p_constraint_mean = MultivariateNormalPDF(c_mu, c_mu, constraint_cov[ii]);
      const double p_dist_mean = NormalPDF(dist_means[ii], dist_means[ii], dist_cov[ii]);
      printf("%f %f\n", p_constraint_vec, p_dist);
      printf("%f %f\n", p_constraint_mean, p_dist_mean);
      p_rigid[ii] *= p_constraint_vec/p_constraint_mean * p_dist/p_dist_mean;
      p_prismatic[ii] *= p_constraint_vec/p_constraint_mean * (1.0 - p_dist/p_dist_mean);
      p_revolute[ii] *= p_dist/p_dist_mean * (1.0  - p_constraint_vec/p_constraint_mean);
    }

    // Normalize the probabilties
    const double normalizer = p_rigid[ii] + p_prismatic[ii] + p_revolute[ii];
    if (normalizer < kFPTolerance)
    {
      printf("DModel: Normalizer is zero. Error in learning model parameters\n");
    }
    p_rigid[ii] /= normalizer;
    p_prismatic[ii] /= normalizer;
    p_revolute[ii] /= normalizer;
  }
  */

  // Decision tree based on variance
  // TODO: This must be refined
  const double constraint_thresh = 0.05;
  const double dist_thresh = 0.05;
  for (int ii = 0; ii < num_edges; ++ii)
  {
    double cov_det = constraint_cov[ii].determinant();
    if (cov_det < constraint_thresh && dist_cov[ii] < dist_thresh)
    {
      p_rigid[ii] = 1.0;
      p_prismatic[ii] = 0.0;
      p_revolute[ii] = 0.0;
    }
    else if (cov_det < constraint_thresh)
    {
      p_rigid[ii] = 0.0;
      p_prismatic[ii] = 1.0;
      p_revolute[ii] = 0.0;
    }
    else if (dist_cov[ii] < dist_thresh)
    {
      p_rigid[ii] = 0.0;
      p_prismatic[ii] = 0.0;
      p_revolute[ii] = 1.0;
    }
    else
    {
      p_rigid[ii] = 0.33;
      p_prismatic[ii] = 0.33;
      p_revolute[ii] = 0.33;
    }
  }
  
  // Display probabilities
  printf("DModel: Learnt probabilities\n");
  for (int ii = 0; ii < num_edges; ++ii)
  {
    printf("Edge (%d %d): %f %f %f\n", edges[ii].first, edges[ii].second, 
        p_rigid[ii], p_prismatic[ii], p_revolute[ii]);
  }
  
  // Initialize edges
  for (int ii = 0; ii < num_edges; ++ii)
  {
    // Determine edge type
    double max_p = p_rigid[ii];
    JointType jt = RIGID;
    if (p_prismatic[ii] > max_p)
    {
      max_p = p_prismatic[ii];
      jt = PRISMATIC;
    }
    if (p_revolute[ii] > max_p)
    {
      max_p = p_revolute[ii];
      jt = REVOLUTE;
    }
    
     // Add edge
    EdgeParams e_params;
    e_params.joint = jt;
    // Rad does not matter for now
    e_params.rad = 1.0; 
    if (jt == RIGID)
    {
      // No constraint vector for rigid joints
      e_params.normal = tf::Vector3(0.0, 0.0, 0.0);
    }
    else if (jt == PRISMATIC)
    {
      e_params.normal = constraint_means[ii];
    }
    else if (jt == PRISMATIC)
    {
      //TODO: I am assuming the first and last vectors are different
      tf::Vector3 c1 = GetLocalVector(observations[0].poses[edges[ii].first], observations[0].poses[edges[ii].second]);
      tf::Vector3 c2 = GetLocalVector(observations.back().poses[edges[ii].first], observations.back().poses[edges[ii].second]);
      e_params.normal = Cross(c1, c2);
    }
    AddEdge(edges[ii], e_params);
  }
  return;
}

tf::Vector3 DModel::GetLocalVector(const geometry_msgs::Pose p1, const geometry_msgs::Pose p2)
{
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(-p1.position.x, -p1.position.y, -p1.position.z));
  transform.setRotation(tf::Quaternion(p1.orientation.x, p1.orientation.y, p1.orientation.z, p1.orientation.w));
  return transform(tf::Vector3(p2.position.x, p2.position.y, p2.position.z));
}

// Debug utilties
void DModel::PrintPoints()
{
  printf("Points:\n");
  for (size_t ii = 0; ii < points_.poses.size(); ++ii)
  {
    geometry_msgs::Pose p = points_.poses[ii];
    printf("Point %d: (%f %f %f), (%f %f %f %f)\n", int(ii), p.position.x, p.position.y, p.position.z,
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w);
  }
  printf("\n");
  return;
}

void DModel::PrintEdges()
{
  printf("Edges:\n");
  for (auto ii = edge_map_->begin(); ii != edge_map_->end(); ++ii)
  {
    printf("(%d %d): (%d, (%f %f %f), %f)\n", ii->first.first, ii->first.second,
        ii->second.joint,
        ii->second.normal.x(), ii->second.normal.y(), ii->second.normal.z(),
        ii->second.rad);
  }
  printf("\n");
  return;
}
