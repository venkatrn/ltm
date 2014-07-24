/**
 * @file d_model_bank.cpp
 * @author Venkatraman Narayanan
 * Carnegie Mellon University, 2014
 */

#include <ltm/d_model_bank.h>
#include <Eigen/Core>
#include <ros/console.h>

#include <cstdio>
#include <iostream>
#include <set>
#include <queue>

// Multiplier for edge costs to avoid working with floating point numbers.
const double kCostMultiplier = 1e3;
// Error tolerance for comparing goal point locations.
const double kGoalTolerance = 0.25; //0.05, 0.01 for experiments

using namespace std;

DModelBank::DModelBank(const string& reference_frame, int num_models)
{
  reference_frame_ = reference_frame;
  num_models_ = num_models;
  for (int ii = 0; ii < num_models; ++ii)
  {
    EdgeMap* edge_map = new unordered_map<Edge, EdgeParams, pair_hash>();
    edge_maps_.push_back(edge_map);
  }

  viz_ = new LTMViz("d_model_viz");
  viz_->SetReferenceFrame(reference_frame);

  // Initialize env params
  env_cfg_.start_state_id = -1;
  env_cfg_.internal_start_state_id = -1;
  env_cfg_.sim_time_step = 0.1;
  visualize_dmodel_ = true;
  visualize_expansions_ = true;
}

DModelBank::DModelBank() 
{
  ROS_ERROR("Deprecated Constructor");
}

DModelBank::~DModelBank() 
{
  for (size_t ii = 0; ii < edge_maps_.size(); ++ii)
  {
    if (edge_maps_[ii] != nullptr)
    {
      delete edge_maps_[ii];
      edge_maps_[ii] = nullptr;
    }
  }
}

void DModelBank::InitFromFile(vector<string> dmodel_files)
{
  assert(num_models_ = int(dmodel_files.size()));
  for (int ii = 0; ii < num_models_; ++ii)
  {
    InitFromFile(ii, dmodel_files[ii].c_str(), 0.0, 0.0, 0.0);
  }
  return;
}

void DModelBank::InitFromFile(vector<string> dmodel_files, std::vector<double> shifts_x, std::vector<double> shifts_y, std::vector<double> shifts_z)
{
  assert(num_models_ = int(dmodel_files.size()));
  for (int ii = 0; ii < num_models_; ++ii)
  {
    ROS_DEBUG("Loading model file: %s", dmodel_files[ii].c_str());
    InitFromFile(ii, dmodel_files[ii].c_str(), shifts_x[ii], shifts_y[ii], shifts_z[ii]);
  }
  return;
}

void DModelBank::InitFromFile(int model_id, const char* dmodel_file)
{
  InitFromFile(model_id, dmodel_file, 0.0, 0.0, 0.0);
  return;
}

void DModelBank::InitFromFile(int model_id, const char* dmodel_file, double shift_x, double shift_y, double shift_z)
{
  FILE* f_dmodel = fopen(dmodel_file,"r");
  int num_points, num_edges, idx1, idx2, j_t;
  float dir_x, dir_y, dir_z, rad;
  float x, y, z, o_x, o_y, o_z, o_w;
  geometry_msgs::Pose p;
  EdgeParams e_params;
  if (f_dmodel == NULL) 
  {
    ROS_ERROR("Unable to open dmodel file\n");
    return;
  }

  char s_temp[1024];
  if (fscanf(f_dmodel, "%s", s_temp) < 1)
  {
    ROS_ERROR("Error reading dmodel file");
  }
  if (strcmp(s_temp, "points:") !=0)
  {
    ROS_ERROR("Incorrect format for dmodel file\n");
  }
  if (fscanf(f_dmodel, "%s", s_temp) < 1)
  {
    ROS_ERROR("Error reading dmodel file");
  }
  num_points = atoi(s_temp);

  if (fscanf(f_dmodel, "%s", s_temp) < 1)
  {
    ROS_ERROR("Error reading dmodel file");
  }
  if (strcmp(s_temp, "edges:") !=0)
  {
    ROS_ERROR("Incorrect format for dmodel file\n");
  }
  if (fscanf(f_dmodel, "%s", s_temp) < 1)
  {
    ROS_ERROR("Error reading dmodel file");
  }
  num_edges = atoi(s_temp);
  ROS_DEBUG("Reading model file with %d points and %d edges\n", num_points, num_edges);
  for (int ii = 0; ii < num_points; ++ii)
  {
    if (fscanf(f_dmodel, "%d %f %f %f %f %f %f %f\n", &idx1, &x, &y, &z,
          &o_x, &o_y, &o_z, &o_w) != 8) 
    {
      ROS_ERROR("Error reading points d-model file\n");
      return;
    }
    p.position.x = x + shift_x;
    p.position.y = y + shift_y;
    p.position.z = z + shift_z;
    p.orientation.x = o_x;
    p.orientation.y = o_y;
    p.orientation.z = o_z;
    p.orientation.w = o_w;
    // Add points only once. This should be refactored.
    if (model_id == 0)
    {
      AddPoint(p);
    }
  }
  // Edges must be added only after all points have been added.
  for (int ii = 0; ii < num_edges; ++ii)
  {
    if (fscanf(f_dmodel, "%d %d %d %f %f %f %f\n", &idx1, &idx2, &j_t, &dir_x, &dir_y, &dir_z, &rad) != 7)
    {
      ROS_ERROR("Error reading edges d-model file\n");
      return;
    }
    e_params.joint = static_cast<JointType>(j_t);
    e_params.normal = tf::Vector3(dir_x, dir_y, dir_z);
    e_params.rad = rad;
    AddEdge(model_id, make_pair(idx1, idx2), e_params);
  } 
  fclose(f_dmodel);
  ROS_DEBUG("Finished reading model file\n");
  TFCallback(points_);
  return;
}

void DModelBank::SetPoints(const geometry_msgs::PoseArray& points)
{
  points_.poses.clear();
  points_ = points;
  ROS_DEBUG("DModelBank: %d points have been set.\n", int(points_.poses.size())); 
  TFCallback(points_);
  return;
}

void DModelBank::InitForcePrimsFromFile(const char* fprims_file)
{
  FILE* f_fprims = fopen(fprims_file,"r");
  int num_fprims, idx;
  float f_x, f_y, f_z;
  geometry_msgs::Pose p;
  if (f_fprims == NULL) 
  {
    ROS_ERROR("Unable to open force primitives file\n");
    return;
  }

  char s_temp[1024];
  if (fscanf(f_fprims, "%s", s_temp) < 1)
  {
    ROS_ERROR("Error reading force primitives file");
  }
  if (strcmp(s_temp, "primitives:") !=0)
  {
    ROS_ERROR("Incorrect format for dmodel file");
  }
  if (fscanf(f_fprims, "%s", s_temp) < 1)
  {
    ROS_ERROR("Error reading force primitives file");
  }
  num_fprims = atoi(s_temp);

  ROS_DEBUG("Reading force primitives file with %d primitives\n", num_fprims);

  for (int ii = 0; ii < num_fprims; ++ii)
  {
    if (fscanf(f_fprims, "%d %f %f %f\n", &idx, &f_x, &f_y, &f_z) != 4)
    {
      ROS_ERROR("Error reading force primitives file\n");
      return;
    }
    tf::Vector3 force(f_x, f_y, f_z);
    force_primitives_.push_back(force);
  } 
  // TODO: Hack where the last force primitive is all zeroes (useful for switching grasp points)
  tf::Vector3 dummy_force(0.0, 0.0, 0.0);
  force_primitives_.push_back(dummy_force);

  fclose(f_fprims);
  ROS_DEBUG("Finished reading force primitives file\n");
  return;
}

void DModelBank::AddEdge(int model_id, Edge e, EdgeParams e_params) 
{
  assert(model_id < int(edge_maps_.size()));
  EdgeMap* edge_map = edge_maps_[model_id];
  if (adj_list_.size() == 0)
  {
    adj_list_.resize(points_.poses.size());
  }
  // TODO: Do this in a smarter way?
  if (find(adj_list_[e.first].begin(), adj_list_[e.first].end(), e.second) == adj_list_[e.first].end())
  {
    adj_list_[e.first].push_back(e.second);
    adj_list_[e.second].push_back(e.first);
    //printf("Size of edge map: %d\n",edge_map->size());
    (*edge_map)[e] = e_params;
    // By default, assume that the reverse connection is of the same type, unless otherwise provided
    (*edge_map)[make_pair(e.second, e.first)] = e_params;
  }
  else
  {
    // Overwrite the joint type
    (*edge_map)[e] = e_params;
  }
  /*
     else if (e_params.joint != RIGID)
     {
     (*edge_map)[e] = e_params;
     (*edge_map)[make_pair(e.second, e.first)] = e_params;
     }
     */

  return;
}

void DModelBank::AddPoint(geometry_msgs::Pose p)
{
  points_.poses.push_back(p);
  return;
}

void DModelBank::AddGraspPoint(geometry_msgs::Pose p)
{
  ROS_INFO("Adding grasp point");
  // This is a special case where a point can be added after the model is learnt.
  // Find the closest point
  double min_dist = 100000;
  int closest_p_idx = 1;
  for (size_t ii = 0; ii < points_.poses.size(); ++ii)
  {
    double distance = Dist(p.position, points_.poses[ii].position);
    if (distance < min_dist)
    {
      min_dist = distance;
      closest_p_idx = ii;
    }
  }
  int num_points = int(points_.poses.size());
  adj_list_.resize(num_points + 1);
  Edge e = make_pair(num_points, closest_p_idx);
  EdgeParams e_params(RIGID, tf::Vector3(0.0, 0.0, 0.0), 1.0);
  AddPoint(p);
  // Add edge in all models
  for (int ii = 0; ii < num_models_; ++ii)
  {
    AddEdge(ii, e, e_params);
  }
  AddGraspIdx(num_points);
  TFCallback(points_);
  return;
}

void DModelBank::TFTimedCallback(const ros::TimerEvent& event)
{
  TFCallback(points_);
  return;
}

void DModelBank::TFCallback(geometry_msgs::PoseArray dmodel_points)
{
  /*
  for (auto it = edge_map_->begin(); it != edge_map_->end(); ++it)
  {
    // Skip edges involving grasp point
    if (find(grasp_idxs_.begin(), grasp_idxs_.end(), it->first.first) != grasp_idxs_.end()
        || find(grasp_idxs_.begin(), grasp_idxs_.end(), it->first.second) != grasp_idxs_.end())
    {
      continue;
    }
    edges.points.push_back(dmodel_points.poses[it->first.first].position);
    edges.points.push_back(dmodel_points.poses[it->first.second].position);
  }
  */

  tf::Transform transform;

  for (size_t ii = 0; ii < points_.poses.size(); ++ii)
  {
    static tf::TransformBroadcaster tf_br_;
    geometry_msgs::Pose p = dmodel_points.poses[ii];
    transform.setOrigin(tf::Vector3(p.position.x, p.position.y, p.position.z) );
    transform.setRotation(tf::Quaternion(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w));
    string child_frame_id = to_string(ii);
    tf_br_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), reference_frame_.c_str(), child_frame_id.c_str()));

    // Skip the grasp points 
    if (find(grasp_idxs_.begin(), grasp_idxs_.end(), ii) != grasp_idxs_.end())
    {
      continue;
    }
    //points.points.push_back(p.position);
  }

  if (visualize_dmodel_)
  {
    // TODO: For now visualize all points, including the grasp points
    // Visualizing the 0th model by default
    assert(edge_maps_.size() >= 1);
    viz_->VisualizeModel(*(edge_maps_[0]), dmodel_points);
  }
  return;
}

void DModelBank::ExtractIndices(int model_id, int p_idx, vector<int>* component_idxs, vector<int>* sep_idxs, JointType* joint_type,
    tf::Vector3* normal)
{
  assert(model_id < int(edge_maps_.size()));
  EdgeMap* edge_map = edge_maps_[model_id];
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
      if (edge_map->find(make_pair(adj_points[ii], idx)) != edge_map->end())
      {
        e_params = edge_map->at(make_pair(adj_points[ii], idx));
      }
      else
      {
        ROS_ERROR("DModelBank: Adj list says neighbor exists, but edge map has no information\n");
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

void DModelBank::GetAdjPoints(int p_idx, std::vector<int> *adj_points)
{
  adj_points->clear();
  for (size_t ii = 0; ii < adj_list_[p_idx].size(); ++ii)
  {
    adj_points->push_back(adj_list_[p_idx][ii]);
  }
  return;
}


void DModelBank::GetNextState(int model_id, const geometry_msgs::PoseArray& in_points, int p_idx, tf::Vector3 force, double del_t, geometry_msgs::PoseArray *out_points)
{

  if (points_.poses.size() != in_points.poses.size())
  {
    ROS_ERROR("Number of points in current state and internal model do not match\n");
    return;
  }

  if (p_idx < 0 || p_idx >= int(points_.poses.size()))
  {
    ROS_ERROR("Invalid point index\n");
    return;
  }

  out_points->poses.clear();
  *out_points = in_points;
  vector<int> component_idxs, sep_idxs;
  JointType joint_type;
  tf::Vector3 normal;
  ExtractIndices(model_id, p_idx, &component_idxs, &sep_idxs, &joint_type, &normal);

  /*
     printf("Rigid component has %d points. Joint type is %d and param vector is %0.2f %0.2f %0.2f\n",
     (int)component_idxs.size(), static_cast<int>(joint_type), normal.x(), 
     normal.y(), normal.z());
     */

  // DEBUG
  /*
     printf("Cluster indices:\n");
     for (size_t ii = 0; ii < component_idxs.size(); ++ii)
     {
     printf("%d ", component_idxs[ii]);
     }
     printf("\n");
     printf("Sep indices:\n");
     for (size_t ii = 0; ii < sep_idxs.size(); ++ii)
     {
     printf("%d ", sep_idxs[ii]);
     }
     printf("\n");
     */

  int closest_p_idx = -1;
  if (sep_idxs.size() != 0) 
  {
    // Find the separating point with the closest distance 
    geometry_msgs::Point p = in_points.poses[p_idx].position;
    double min_dist = 100000;
    for (size_t ii = 0; ii < sep_idxs.size(); ++ii)
    {
      double distance = Dist(p, in_points.poses[sep_idxs[ii]].position);
      if (distance < min_dist)
      {
        min_dist = distance;
        closest_p_idx = sep_idxs[ii];
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
    // Do all computations in reference frame
    // Moment center and arm
    tf::Vector3 center(out_points->poses[closest_p_idx].position.x, out_points->poses[closest_p_idx].position.y, out_points->poses[closest_p_idx].position.z);
    tf::Vector3 p_applied(out_points->poses[p_idx].position.x, out_points->poses[p_idx].position.y, out_points->poses[p_idx].position.z);
    //TODO: This is a hack
    //tf::Vector3 arm = p_applied - center;
    tf::Vector3 arm = tf::Vector3(p_applied.x() - center.x(), p_applied.y() - center.y(), 0.0);

    // Project the force onto the tangent direction
    tf::Vector3 tangent = Cross(normal, arm);
    //double projection = tangent.x()*transformed_force.x() + tangent.y()*transformed_force.y() + tangent.z()*transformed_force.z();
    double projection = tangent.x()*ref_force.x() + tangent.y()*ref_force.y() + tangent.z()*ref_force.z();
    tangent = tangent / Norm(tangent);
    tf::Vector3 projected_vector(projection*tangent.x(), projection*tangent.y(), projection*tangent.z());
    tf::Vector3 torque = Cross(arm, projected_vector);
    double torque_norm = sqrt(Sqr(torque.x()) + Sqr(torque.y()) + Sqr(torque.z()));
    double theta = 0.5*Sqr(del_t)*torque_norm/inertia;

    /*
       printf("Arm: %f %f %f\n", arm.x(), arm.y(), arm.z());
       printf("Normal: %f %f %f\n", normal.x(), normal.y(), normal.z());
       printf("Tangent: %f %f %f\n", tangent.x(), tangent.y(), tangent.z());
       printf("Projected vector: %f %f %f\n", projected_vector.x(), projected_vector.y(), projected_vector.z());
       printf("Torque: %f %f %f\n", torque.x(), torque.y(), torque.z());
       printf("Theta: %f\n", theta);
       */

    tf::Quaternion quat;
    // Return
    if (torque_norm < 1e-5) 
    {
      return;
      ROS_ERROR("Applied torque is zero - dividing by zero\n");
    }
    quat.setRotation(torque/torque_norm, theta);
    tf::Transform tr = tf::Transform(quat);

    for (size_t ii = 0; ii < component_idxs.size(); ++ii)
    {
      geometry_msgs::Point p = out_points->poses[component_idxs[ii]].position;
      geometry_msgs::Quaternion q = out_points->poses[component_idxs[ii]].orientation;
      tf::Vector3 point(p.x, p.y, p.z);
      tf::Vector3 rotated_point = tr*(point-center) + center;
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
  else if (joint_type == SPHERICAL)
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
    // Do all computations in reference frame
    // Moment center and arm
    tf::Vector3 center(out_points->poses[closest_p_idx].position.x, out_points->poses[closest_p_idx].position.y, out_points->poses[closest_p_idx].position.z);
    tf::Vector3 p_applied(out_points->poses[p_idx].position.x, out_points->poses[p_idx].position.y, out_points->poses[p_idx].position.z);
    tf::Vector3 arm = p_applied - center;

    tf::Vector3 torque = Cross(arm, tf::Vector3(transformed_force.x(), transformed_force.y(), transformed_force.z()));
    tf::Vector3 omega = 0.5 * Sqr(del_t) * torque;
    double theta = sqrt(Sqr(omega.x()) + Sqr(omega.y()) + Sqr(omega.z()));

    /*
       printf("Arm: %f %f %f\n", arm.x(), arm.y(), arm.z());
       printf("Normal: %f %f %f\n", normal.x(), normal.y(), normal.z());
       printf("Torque: %f %f %f\n", torque.x(), torque.y(), torque.z());
       printf("Theta: %f\n", theta);
       */

    tf::Quaternion quat;
    // Return
    if (theta < 1e-5) 
    {
      return;
      ROS_ERROR("Omega norm is zero - dividing by zero\n");
    }
    quat.setRotation(omega/theta, theta);
    tf::Transform tr = tf::Transform(quat);

    for (size_t ii = 0; ii < component_idxs.size(); ++ii)
    {
      geometry_msgs::Point p = out_points->poses[component_idxs[ii]].position;
      geometry_msgs::Quaternion q = out_points->poses[component_idxs[ii]].orientation;
      tf::Vector3 point(p.x, p.y, p.z);
      tf::Vector3 rotated_point = tr*(point-center) + center;
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

void DModelBank::SimulatePlan(int model_id, const vector<int>& fprim_ids)
{
  assert(model_id < int(edge_maps_.size()));
  vector<tf::Vector3> forces;
  vector<int> grasp_points;
  ConvertForcePrimIDsToForcePrims(fprim_ids, &forces, &grasp_points);
  SimulatePlan(model_id, forces, grasp_points);
  return;
}

void DModelBank::SimulatePlan(int model_id, const vector<tf::Vector3>& forces, const vector<int>& grasp_points)
{
  assert(model_id < int(edge_maps_.size()));
  geometry_msgs::PoseArray in_points = points_;
  for (size_t ii = 0; ii < forces.size(); ++ii)
  {
    geometry_msgs::PoseArray out_points;
    GetNextState(model_id, in_points, grasp_points[ii], forces[ii], env_cfg_.sim_time_step, &out_points);
    // TODO: remove this
    // TFCallback(out_points);
    viz_->VisualizeModel(*(edge_maps_[model_id]), out_points);

    // Visualize force
    viz_->VisualizeForcePrim(forces[ii], out_points.poses[grasp_points[ii]]);

    in_points = out_points;
    usleep(100000);
  }
  return;
}

void DModelBank::VisualizeState(int model_id, int state_id)
{
  assert(model_id < edge_maps_.size());

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
  viz_->VisualizeModel(*(edge_maps_[model_id]), points);
  // TODO: remove this
  //TFCallback(points);
  return;
}

int DModelBank::StateToStateID(State_t& s)
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

State_t DModelBank::StateIDToState(int state_id)
{
  auto it = StateMap.find(state_id);
  if (it != StateMap.end())
  {
    return it->second;
  }
  else
  {
    ROS_ERROR("DModelBank: Error. Requested State ID does not exist. Will return empty state.\n");
  }
  State_t empty_state;
  return empty_state;
}

int DModelBank::BeliefStateToStateID(BeliefState_t& s)
{
  // If state has already been created, return ID from hash map
  for (auto it = BeliefStateMap.begin(); it != BeliefStateMap.end(); ++it)
  {
    if (s == it->second)
    {
      return it->first;
    }
  }
  // Otherwise, create state, add to hash map, and return ID
  int new_id = int(BeliefStateMap.size());
  BeliefStateMap[new_id] = s;
  return new_id;
}

BeliefState_t DModelBank::BeliefStateIDToState(int belief_state_id)
{
  auto it = BeliefStateMap.find(belief_state_id);
  if (it != BeliefStateMap.end())
  {
    return it->second;
  }
  else
  {
    ROS_ERROR("DModelBank: Error. Requested BeliefState ID does not exist. Will return empty state.\n");
  }
  BeliefState_t empty_state;
  return empty_state;
}

int DModelBank::FPrimToFPrimID(int grasp_idx, int force_idx)
{
  const int num_grasp_idxs = grasp_idxs_.size();
  return (force_idx * num_grasp_idxs) + grasp_idx;
}

void DModelBank::FPrimIDToFPrim(int fprim_id, int* grasp_idx, int* force_idx)
{
  const int num_grasp_idxs = grasp_idxs_.size();
  *grasp_idx = fprim_id % num_grasp_idxs;
  *force_idx = fprim_id / num_grasp_idxs;
}

void DModelBank::GetSuccs(int source_state_id, 
    vector<vector<int>>* succ_state_ids_map,
    vector<vector<double>>* succ_state_probabilities_map, 
    vector<int>* action_ids,
    vector<double>* action_costs)
{
  succ_state_ids_map->clear();
  succ_state_probabilities_map->clear();
  action_ids->clear();
  action_costs->clear();

  BeliefState_t source_belief_state = BeliefStateIDToState(source_state_id);
  const int internal_state_id = source_belief_state.internal_state_id;
  unordered_map<int, vector<int>> succ_map; //(fp_id, succs)
  unordered_map<int, vector<double>> probabilities_map; //(fp_id, probs)
  unordered_map<int, double> costs_map; //(fp_id, cost)
  for (int ii = 0; ii < num_models_; ++ii)
  {
    vector<int> succs;
    vector<int> edge_ids;
    vector<double> costs;
    GetSuccs(ii, internal_state_id, &succs, &edge_ids, &costs);
    for (int jj = 0; jj < int(succs.size()); ++jj)
    { 
      BeliefState_t s;
      s.internal_state_id = succs[jj];
      s.belief.resize(num_models_);
      // TODO: Incorporate noisy observation model
      s.belief[ii] = 1;
      int belief_state_id = BeliefStateToStateID(s);
      int action_id = edge_ids[jj];

      succ_map[action_id].push_back(belief_state_id);
      probabilities_map[action_id].push_back(source_belief_state.belief[ii]);
      costs_map[action_id] = costs[jj];
    }
  }
  const int num_actions = succ_map.size();
  int action_num = 0;
  succ_state_ids_map->resize(num_actions);
  succ_state_probabilities_map->resize(num_actions);
  for (auto it = succ_map.begin(); it != succ_map.end(); ++it)
  {
    int action_id = it->first;
    action_ids->push_back(action_id);
    action_costs->push_back(costs_map[action_id]);
    (*succ_state_ids_map)[action_num] = it->second;
    (*succ_state_probabilities_map)[action_num] = probabilities_map[action_id];
    action_num++;
  }
}

void DModelBank::GetSuccs(int model_id, int source_state_id, vector<int>* succs, vector<int>* edge_ids, vector<double>* costs)
{
  succs->clear();
  edge_ids->clear();
  costs->clear();

  // Goal state should be absorbing.
  if (IsInternalGoalState(source_state_id))
  {
    ROS_INFO("Asking successors for goal state!!");
    return;
  }

  if (visualize_expansions_)
  {
    VisualizeState(model_id, source_state_id);
    usleep(1000);
  }

  State_t source_state = StateIDToState(source_state_id);

  // TODO: Force primitive hack (-1)
  for (size_t jj = 0; jj < force_primitives_.size() - 1; ++jj) {
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
    GetNextState(model_id, in_points, source_state.grasp_idx, force, env_cfg_.sim_time_step, &out_points);

    // Determine points that have changed and generate succ state accordingly.
    State_t succ_state;
    succ_state.grasp_idx = source_state.grasp_idx;
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
    edge_ids->push_back(FPrimToFPrimID(source_state.grasp_idx, jj));
    // TODO(venkat): Compute costs
    // costs->push_back(1);
    // costs->push_back(int(kCostMultiplier * Norm(force)));
    // TODO(venkat): Compute distance traveled by end-effector, so that
    // we can compute power = force x velocity
    // Cost is time
    costs->push_back(int(kCostMultiplier * env_cfg_.sim_time_step));
  }

  // Successors for changing grasp point
  for (size_t ii = 0; ii < grasp_idxs_.size(); ++ii)
  {
    if (source_state.grasp_idx == grasp_idxs_[ii])
    {
      continue;
    }
    State_t succ_state = source_state;
    succ_state.grasp_idx = grasp_idxs_[ii];
    int succ_id = StateToStateID(succ_state);
    succs->push_back(succ_id);
    // TODO: This is a hack where the last force primitive represents the all zeroes force
    // for switching grasp points
    edge_ids->push_back(FPrimToFPrimID(succ_state.grasp_idx, force_primitives_.size() - 1));
    costs->push_back(int(1 * kCostMultiplier * env_cfg_.sim_time_step));
  }
  return;
}

bool DModelBank::IsGoalState(int belief_state_id)
{
  BeliefState_t s = BeliefStateIDToState(belief_state_id);
  int internal_state_id = s.internal_state_id;
  return IsInternalGoalState(internal_state_id);
}

bool DModelBank::IsInternalGoalState(int state_id)
{
  State_t s = StateIDToState(state_id);
  State_t goal_state = StateIDToState(env_cfg_.internal_goal_state_id);

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

double DModelBank::GetGoalHeuristic(int belief_state_id)
{
  BeliefState_t s = BeliefStateIDToState(belief_state_id);
  int internal_state_id = s.internal_state_id;
  return GetInternalGoalHeuristic(internal_state_id);
}

double DModelBank::GetInternalGoalHeuristic(int internal_state_id)
{

  State_t s = StateIDToState(internal_state_id);
  geometry_msgs::Pose grasp_pose;
  double total_dist = 0;

  geometry_msgs::Pose current_grasp_pose;
  double grasp_point_dist = 0;

  for (size_t ii = 0; ii < grasp_idxs_.size(); ++ii)
  {
    // If position of end-eff has changed, then use the coordinates from state.
    auto it = find(s.changed_inds.begin(),
        s.changed_inds.end(),
        grasp_idxs_[ii]);
    if ( it != s.changed_inds.end())
    {
      int offset = distance(s.changed_inds.begin(), it);
      grasp_pose = s.changed_points.poses[offset];
    }
    else
    {
      grasp_pose = points_.poses[grasp_idxs_[ii]];
    }
    if (ii >= env_cfg_.goal_grasp_poses.poses.size())
    {
      ROS_ERROR("DModelBank: Invalid goal grasp index\n");
      return 0;
    }
    total_dist += Dist(grasp_pose.position, env_cfg_.goal_grasp_poses.poses[ii].position);

    // Distance for current location of end-effector alone
    if (grasp_idxs_[ii] == s.grasp_idx)
    {
      grasp_point_dist = Dist(grasp_pose.position, env_cfg_.goal_grasp_poses.poses[ii].position);
    }
  }

  double heuristic = kCostMultiplier *  total_dist / env_cfg_.sim_time_step;

  if (grasp_point_dist > 0.2)
  {
    double end_eff_heuristic = kCostMultiplier * (total_dist) / env_cfg_.sim_time_step;
    return end_eff_heuristic;
  }
  return heuristic;
}

void DModelBank::AddGraspIdx(int grasp_idx)
{
  ROS_INFO("Adding grasp index");
  if (grasp_idx >= int(points_.poses.size()))
  {
    ROS_ERROR("DModelBank: Invalid force index\n");
    return;
  }
  grasp_idxs_.push_back(grasp_idx);
  return;
}

void DModelBank::SetStartState(BeliefState_t start_state)
{
  int start_state_id = BeliefStateToStateID(start_state);
  ROS_INFO("DModelBank: Setting start state: %d\n", start_state_id);
  env_cfg_.start_state_id = start_state_id;
  return; 
}

void DModelBank::SetInternalStartState(State_t start_state)
{
  int start_state_id = StateToStateID(start_state);
  ROS_INFO("DModelBank: Setting internal start state: %d\n", start_state_id);
  env_cfg_.internal_start_state_id = start_state_id;
  return; 
}

void DModelBank::SetGoalState(BeliefState_t goal_state)
{
  int goal_state_id = BeliefStateToStateID(goal_state);
  ROS_INFO("DModelBank: Setting goal state: %d\n",goal_state_id);
  env_cfg_.goal_state_id = goal_state_id;
  return; 
}

void DModelBank::SetInternalGoalState(State_t goal_state)
{
  int goal_state_id = StateToStateID(goal_state);
  ROS_INFO("DModelBank: Setting internal goal state: %d\n",goal_state_id);
  env_cfg_.internal_goal_state_id = goal_state_id;
  for (size_t ii = 0; ii < goal_state.changed_inds.size(); ++ii)
  {
    env_cfg_.goal_grasp_poses.poses = goal_state.changed_points.poses;
  }
  return; 
}

int DModelBank::GetStartStateID()
{
  return env_cfg_.start_state_id;
}

int DModelBank::GetInternalStartStateID()
{
  return env_cfg_.internal_start_state_id;
}

void DModelBank::SetSimTimeStep(double del_t)
{
  env_cfg_.sim_time_step = del_t;
  return;
}

bool DModelBank::ConvertForcePrimIDsToForcePrims(const vector<int>& fprim_ids, vector<tf::Vector3>* forces, 
    vector<int>* grasp_points)
{
  forces->clear();
  grasp_points->clear();
  // Skip the start state
  for (size_t ii = 1; ii < fprim_ids.size(); ++ii)
  {
    //TODO: Check this
    /*
       if (fprim_ids[ii] >= int(force_primitives_.size()))
       {
       printf("DModelBank: Invalid force primitive ID while reconstructing forces\n");
       return false;
       }
       */
    int grasp_idx;
    int force_idx;
    FPrimIDToFPrim(fprim_ids[ii], &grasp_idx, &force_idx);
    forces->push_back(force_primitives_[force_idx]);
    grasp_points->push_back(grasp_idxs_[grasp_idx]);
  }
  return true;
}

bool DModelBank::GetEndEffectorTrajFromStateIDs(const std::vector<int>& state_ids,
    geometry_msgs::PoseArray* traj)
{
  traj->poses.clear();
  for (size_t ii = 0; ii < state_ids.size(); ++ii)
  {
    State_t s = StateIDToState(state_ids[ii]);
    auto it = std::find(s.changed_inds.begin(), s.changed_inds.end(), s.grasp_idx);
    if (it != s.changed_inds.end())
    {
      const int offset = distance(s.changed_inds.begin(), it);
      traj->poses.push_back(s.changed_points.poses[offset]);
    }
    else
    {
      traj->poses.push_back(points_.poses[s.grasp_idx]);
    }
  }
  return true;
}


// Debug utilties
void DModelBank::PrintPoints()
{
  ROS_INFO("Points:");
  for (size_t ii = 0; ii < points_.poses.size(); ++ii)
  {
    geometry_msgs::Pose p = points_.poses[ii];
    ROS_INFO("Point %d: (%f %f %f), (%f %f %f %f)", int(ii), p.position.x, p.position.y, p.position.z,
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w);
  }
  return;
}

void DModelBank::PrintEdges(int model_id)
{
  ROS_INFO("Edges:");
  assert(model_id < int(edge_maps_.size()));
  for (auto ii = edge_maps_[model_id]->begin(); ii != edge_maps_[model_id]->end(); ++ii)
  {
    ROS_INFO("(%d %d): (%d, (%f %f %f), %f)", ii->first.first, ii->first.second,
        ii->second.joint,
        ii->second.normal.x(), ii->second.normal.y(), ii->second.normal.z(),
        ii->second.rad);
  }
  return;
}

void DModelBank::ResetStateMap()
{
  StateMap.clear();
}
