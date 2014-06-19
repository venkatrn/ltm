/**
 * @file d_model.cpp
 * @author Venkatraman Narayanan
 * Carnegie Mellon University, 2013
 */

#include <ltm/d_model.h>
#include <Eigen/Core>
#include <ros/console.h>

#include <cstdio>
#include <iostream>
#include <set>
#include <queue>

// Multiplier for edge costs to avoid working with floating point numbers.
const double kCostMultiplier = 1e3;
// Error tolerance for comparing goal point locations.
const double kGoalTolerance = 0.1; //0.05, 0.01 for experiments

using namespace std;

DModel::DModel(const string& reference_frame)
{
  reference_frame_ = reference_frame;
  edge_map_ = new unordered_map<Edge, EdgeParams, pair_hash>();

  viz_ = new LTMViz("d_model_viz");
  viz_->SetReferenceFrame(reference_frame);

  // Initialize env params
  env_cfg_.start_state_id = -1;
  env_cfg_.sim_time_step = 0.1;
  visualize_dmodel_ = true;
  visualize_expansions_ = true;
}

DModel::DModel() 
{
  ROS_ERROR("Deprecated Constructor");
}

DModel::~DModel() 
{
  if (edge_map_ != nullptr)
  {
    delete edge_map_;
    edge_map_ = nullptr;
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
    AddPoint(p);
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
    AddEdge(make_pair(idx1, idx2), e_params);
  } 
  fclose(f_dmodel);
  ROS_DEBUG("Finished reading model file\n");
  TFCallback(points_);
  return;
}

void DModel::SetPoints(const geometry_msgs::PoseArray& points)
{
  points_.poses.clear();
  points_ = points;
  ROS_DEBUG("DModel: %d points have been set.\n", int(points_.poses.size())); 
  TFCallback(points_);
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

void DModel::AddEdge(Edge e, EdgeParams e_params) 
{
  if (adj_list_.size() == 0)
  {
    adj_list_.resize(points_.poses.size());
  }
  // TODO: Do this in a smarter way?
  if (find(adj_list_[e.first].begin(), adj_list_[e.first].end(), e.second) == adj_list_[e.first].end())
  {
    adj_list_[e.first].push_back(e.second);
    adj_list_[e.second].push_back(e.first);
    //printf("Size of edge map: %d\n",edge_map_->size());
    (*edge_map_)[e] = e_params;
    // By default, assume that the reverse connection is of the same type, unless otherwise provided
    (*edge_map_)[make_pair(e.second, e.first)] = e_params;
  }
  else
  {
    // Overwrite the joint type
    (*edge_map_)[e] = e_params;
  }
  /*
     else if (e_params.joint != RIGID)
     {
     (*edge_map_)[e] = e_params;
     (*edge_map_)[make_pair(e.second, e.first)] = e_params;
     }
     */

  return;
}

void DModel::AddPoint(geometry_msgs::Pose p)
{
  points_.poses.push_back(p);
  return;
}

void DModel::AddGraspPoint(geometry_msgs::Pose p)
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
  AddEdge(e, e_params);
  AddGraspIdx(num_points);
  TFCallback(points_);
  return;
}

void DModel::TFTimedCallback(const ros::TimerEvent& event)
{
  TFCallback(points_);
  return;
}

void DModel::TFCallback(geometry_msgs::PoseArray dmodel_points)
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
    viz_->VisualizeModel(*edge_map_, dmodel_points);
  }
  return;
}

void DModel::ApplyForce(int p_idx, tf::Vector3 force, double del_t)
{
  if (p_idx < 0 || p_idx >= int(points_.poses.size()))
  {
    ROS_ERROR("ApplyForce: Invalid point index\n");
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
      ROS_ERROR("Applied torque is zero - dividing by zero\n");
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
      // TODO: This needs to be double checked
      /*
         if (edge_map_->find(make_pair(idx, adj_points[ii])) != edge_map_->end())
         {
         e_params = edge_map_->at(make_pair(idx, adj_points[ii]));
         }
         else
         {
         e_params = edge_map_->at(make_pair(adj_points[ii], idx));
         }
         */
      if (edge_map_->find(make_pair(adj_points[ii], idx)) != edge_map_->end())
      {
        e_params = edge_map_->at(make_pair(adj_points[ii], idx));
      }
      else
      {
        ROS_ERROR("DModel: Adj list says neighbor exists, but edge map has no information\n");
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
  ExtractIndices(p_idx, &component_idxs, &sep_idxs, &joint_type, &normal);

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

void DModel::SimulatePlan(const vector<int>& fprim_ids)
{
  vector<tf::Vector3> forces;
  vector<int> grasp_points;
  ConvertForcePrimIDsToForcePrims(fprim_ids, &forces, &grasp_points);
  SimulatePlan(forces, grasp_points);
  return;
}

void DModel::SimulatePlan(const vector<tf::Vector3>& forces, const vector<int>& grasp_points)
{
  geometry_msgs::PoseArray in_points = points_;
  for (size_t ii = 0; ii < forces.size(); ++ii)
  {
    geometry_msgs::PoseArray out_points;
    GetNextState(in_points, grasp_points[ii], forces[ii], env_cfg_.sim_time_step, &out_points);
    // TODO: remove this
    // TFCallback(out_points);
    viz_->VisualizeModel(*edge_map_, out_points);

    // Visualize force
    viz_->VisualizeForcePrim(forces[ii], out_points.poses[grasp_points[ii]]);

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
  viz_->VisualizeModel(*edge_map_, points);
  // TODO: remove this
  //TFCallback(points);
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
    ROS_ERROR("DModel: Error. Requested State ID does not exist. Will return empty state.\n");
  }
  State_t empty_state;
  return empty_state;
}

int DModel::FPrimToFPrimID(int grasp_idx, int force_idx)
{
  const int num_grasp_idxs = grasp_idxs_.size();
  return (force_idx * num_grasp_idxs) + grasp_idx;
}

void DModel::FPrimIDToFPrim(int fprim_id, int* grasp_idx, int* force_idx)
{
  const int num_grasp_idxs = grasp_idxs_.size();
  *grasp_idx = fprim_id % num_grasp_idxs;
  *force_idx = fprim_id / num_grasp_idxs;
}

void DModel::GetSuccs(int source_state_id, vector<int>* succs, vector<int>* edge_ids, vector<double>* costs)
{
  succs->clear();
  edge_ids->clear();
  costs->clear();

  // Goal state should be absorbing.
  if (IsGoalState(source_state_id))
  {
    return;
  }

  if (visualize_expansions_)
  {
    VisualizeState(source_state_id);
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
    GetNextState(in_points, source_state.grasp_idx, force, env_cfg_.sim_time_step, &out_points);

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

/*
   void DModel::GetSuccs(int source_state_id, vector<int>* succs, vector<int>* edge_ids, vector<double>* costs)
   {
   succs->clear();
   edge_ids->clear();
   costs->clear();

// Goal state should be absorbing.
if (IsGoalState(source_state_id))
{
return;
}

if (visualize_expansions_)
{
VisualizeState(source_state_id);
usleep(1000);
}

State_t source_state = StateIDToState(source_state_id);

for (size_t kk = 0; kk < grasp_idxs_.size(); ++kk)
{
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
GetNextState(in_points, grasp_idxs_[kk], force, env_cfg_.sim_time_step, &out_points);

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
edge_ids->push_back(FPrimToFPrimID(kk, jj));
// TODO(venkat): Compute costs
// costs->push_back(1);
// costs->push_back(int(kCostMultiplier * Norm(force)));
// TODO(venkat): Compute distance traveled by end-effector, so that
// we can compute power = force x velocity
// Cost is time
costs->push_back(int(kCostMultiplier * env_cfg_.sim_time_step));
}
}
return;
}
*/

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

  State_t s = StateIDToState(state_id);
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
      ROS_ERROR("DModel: Invalid goal grasp index\n");
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

void DModel::AddGraspIdx(int grasp_idx)
{
  ROS_INFO("Adding grasp index");
  if (grasp_idx >= int(points_.poses.size()))
  {
    ROS_ERROR("DModel: Invalid force index\n");
    return;
  }
  grasp_idxs_.push_back(grasp_idx);
  return;
}

void DModel::SetStartState(State_t start_state)
{
  int start_state_id = StateToStateID(start_state);
  ROS_INFO("DModel: Setting start state: %d\n", start_state_id);
  env_cfg_.start_state_id = start_state_id;
  return; 
}

void DModel::SetGoalState(State_t goal_state)
{
  int goal_state_id = StateToStateID(goal_state);
  ROS_INFO("DModel: Setting goal state: %d\n",goal_state_id);
  env_cfg_.goal_state_id = goal_state_id;
  for (size_t ii = 0; ii < goal_state.changed_inds.size(); ++ii)
  {
    env_cfg_.goal_grasp_poses.poses = goal_state.changed_points.poses;
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

bool DModel::ConvertForcePrimIDsToForcePrims(const vector<int>& fprim_ids, vector<tf::Vector3>* forces, 
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
       printf("DModel: Invalid force primitive ID while reconstructing forces\n");
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

bool DModel::GetEndEffectorTrajFromStateIDs(const std::vector<int>& state_ids,
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


void DModel::LearnDModelParameters(const vector<geometry_msgs::PoseArray>& observations,
    const vector<Edge>& edges)
{
  if (int(points_.poses.size()) == 0)
  {
    ROS_ERROR("DModel: Points have not been set. Cannot learn and initialize edges.\n");
    return;
  }
  const int num_edges = int(edges.size());
  if (edge_map_ != NULL)
  {
    edge_map_->clear();
  }
  const int num_obs = int(observations.size());

  if (num_obs < 2)
  {
    ROS_ERROR("DModel: Not enough observations to learn model from\n");
    return;
  }

  // Edges in local frame. For an edge (x1,x2), c = x2 expressed in frame of x1, and normalized.
  // Number of frames x Number of edges
  vector<vector<tf::Vector3>> local_vectors;
  vector<vector<double>> dists;
  local_vectors.resize(num_obs);
  dists.resize(num_obs);
  for (int ii = 0; ii < num_obs; ++ii)
  {
    local_vectors[ii].resize(num_edges);
    dists[ii].resize(num_edges);
    for (int jj = 0; jj < num_edges; ++jj)
    {
      tf::Vector3 c_vector = GetLocalVector(observations[ii].poses[edges[jj].first], observations[ii].poses[edges[jj].second]);
      const double c_vector_norm = Norm(c_vector);
      if (c_vector_norm > kFPTolerance)
      {
        local_vectors[ii][jj] = c_vector / c_vector_norm;
      }
      dists[ii][jj] = c_vector_norm;
    }
  }

  // Compute the axis vectors for edges in two consecutive frames
  vector<vector<tf::Vector3>> axis_vectors;
  axis_vectors.resize(num_obs - 1);
  vector<vector<tf::Vector3>> prismatic_vectors;
  prismatic_vectors.resize(num_obs - 1);
  for (int ii = 0; ii < num_obs - 1; ++ii)
  {
    axis_vectors[ii].resize(num_edges);
    prismatic_vectors[ii].resize(num_edges);
    for (int jj = 0; jj < num_edges; ++jj) 
    {
      axis_vectors[ii][jj] = Cross(local_vectors[ii][jj], local_vectors[ii + 1][jj]);
      const double axis_vector_norm = Norm(axis_vectors[ii][jj]);
      if (axis_vector_norm > 0.1)
      {
        axis_vectors[ii][jj] = axis_vectors[ii][jj] / axis_vector_norm;
      }
      else
      {
        axis_vectors[ii][jj] = 0*axis_vectors[ii][jj];
      }

      prismatic_vectors[ii][jj] = dists[ii + 1][jj]*local_vectors[ii + 1][jj] - dists[ii][jj] * local_vectors[ii][jj];
      //TODO: This is a hack to account for sign change. 
      if (prismatic_vectors[ii][jj].z() < 0)
      {
        prismatic_vectors[ii][jj] = -prismatic_vectors[ii][jj];
      }

      //if (fabs(dists[ii+1][jj]-dists[ii][jj]) < 0.01)
      if (fabs(dists[ii+1][jj]-dists[ii][jj]) < kFPTolerance)
      {
        prismatic_vectors[ii][jj] = tf::Vector3(0.0, 0.0, 0.0);
      }
      const double prismatic_vector_norm = Norm(prismatic_vectors[ii][jj]);
      if (prismatic_vector_norm > kFPTolerance)
      {
        prismatic_vectors[ii][jj] = prismatic_vectors[ii][jj] / prismatic_vector_norm;
      }
      else
      {
        prismatic_vectors[ii][jj] = 0*prismatic_vectors[ii][jj];
      }
    }
  }

  // DEBUG
  for (int jj = 0; jj < num_edges; ++jj)
  {
    for (int ii = 0; ii < num_obs - 1; ++ii)
    {
      //      printf("Axis vector: %d %d: %f %f %f\n", edges[jj].first, edges[jj].second, axis_vectors[ii][jj].x(),
      //          axis_vectors[ii][jj].y(), axis_vectors[ii][jj].z());
      ROS_DEBUG("Prismatic vector: %d %d: %f %f %f\n", edges[jj].first, edges[jj].second, prismatic_vectors[ii][jj].x(),
          prismatic_vectors[ii][jj].y(), prismatic_vectors[ii][jj].z());
      ROS_DEBUG("Local vector 1: %d %d: %f %f %f\n", edges[jj].first, edges[jj].second, local_vectors[ii][jj].x(),
          local_vectors[ii][jj].y(), local_vectors[ii][jj].z());
      ROS_DEBUG("Local vector 2: %d %d: %f %f %f\n", edges[jj].first, edges[jj].second, local_vectors[ii+1][jj].x(),
          local_vectors[ii+1][jj].y(), local_vectors[ii+1][jj].z());
      ROS_DEBUG("Dists 1: %f, 2: %f\n", dists[ii][jj], dists[ii+1][jj]);
    }
  }
  vector<tf::Vector3> constraint_means;
  constraint_means.resize(num_edges);
  vector<double> dist_means;
  dist_means.resize(num_edges);
  vector<tf::Vector3> axis_means;
  axis_means.resize(num_edges);
  vector<tf::Vector3> prismatic_means;
  prismatic_means.resize(num_edges);

  // Estimate parameters for all joint types
  for (int ii = 0; ii < num_edges; ++ii)
  {
    constraint_means[ii] = tf::Vector3(0.0, 0.0, 0.0);
    dist_means[ii] = 0.0;
    axis_means[ii] = tf::Vector3(0.0, 0.0, 0.0);
    prismatic_means[ii] = tf::Vector3(0.0, 0.0, 0.0);
    int num_unique_dist_vals = 1;
    for (int jj = 0; jj < num_obs - 1; ++jj)
    {
      constraint_means[ii] = constraint_means[ii] + local_vectors[jj][ii];
      if (fabs(dists[jj][ii] - dists[jj + 1][ii]) > 0.02)
      {
        dist_means[ii] = dist_means[ii] + dists[jj][ii];
        num_unique_dist_vals++;
      }
      axis_means[ii] = axis_means[ii] + axis_vectors[jj][ii];   
      prismatic_means[ii] = prismatic_means[ii] + prismatic_vectors[jj][ii];
    }
    constraint_means[ii] = constraint_means[ii] / (num_obs - 1);
    dist_means[ii] = dist_means[ii] / (num_unique_dist_vals);
    //dist_means[ii] = dist_means[ii] / (num_obs - 1);
    //axis_means[ii] = axis_means[ii] / (num_obs - 1);
    //prismatic_means[ii] = prismatic_means[ii] / (num_obs - 1);
    constraint_means[ii] = constraint_means[ii] / Norm(constraint_means[ii]);
    if (Norm(axis_means[ii]) > 0.001)
    {
      axis_means[ii] = axis_means[ii] / Norm(axis_means[ii]);
    }
    if (Norm(prismatic_means[ii]) > 0.001)
    {
      prismatic_means[ii] = prismatic_means[ii] / Norm(prismatic_means[ii]);
    }
  }

  ROS_DEBUG("Parameter estimates:\n");
  for (int ii = 0; ii < num_edges; ++ii)
  {
    ROS_DEBUG("Edge: %d %d\n", edges[ii].first, edges[ii].second);
    ROS_DEBUG("Dist mean: %f\n", dist_means[ii]);
    ROS_DEBUG("Constraint mean: %f %f %f\n", constraint_means[ii].x(),
        constraint_means[ii].y(), constraint_means[ii].z());
    ROS_DEBUG("Axis mean: %f %f %f\n", axis_means[ii].x(),
        axis_means[ii].y(), axis_means[ii].z());
    ROS_DEBUG("Prismatic mean: %f %f %f\n", prismatic_means[ii].x(),
        prismatic_means[ii].y(), prismatic_means[ii].z());
  }

  // Assign edge params for all edges
  vector<EdgeParams> edge_params;
  edge_params.resize(num_edges);

  vector<double> p_rigid, p_prismatic, p_revolute;
  p_rigid.resize(num_edges);
  p_prismatic.resize(num_edges);
  p_revolute.resize(num_edges);

  // Setup sensor model
  const double kDistVar = 0.009;
  const double kDirVar = 0.03; //0.3
  const double kCVecVar = 0.03; //0.3
  Eigen::Matrix3d cvec_covariance(kDirVar * Eigen::Matrix3d::Identity());
  Eigen::Matrix3d dir_covariance(kDirVar * Eigen::Matrix3d::Identity());

  // Priors
  const double rigid_prior = 0.35;
  const double prismatic_prior = 0.32;
  const double revolute_prior = 0.32;

  // Bayesian inference to assign parameters
  for (int ii = 0; ii < num_edges; ++ii)
  {
    p_rigid[ii] = 1.0;
    p_prismatic[ii] = 1.0;
    p_revolute[ii] = 1.0;

    for (int jj = 0; jj < num_obs - 1; ++jj)
    {
      Eigen::Vector3d c_vec(local_vectors[jj][ii].x(),
          local_vectors[jj][ii].y(),
          local_vectors[jj][ii].z());
      Eigen::Vector3d axis_vec(axis_vectors[jj][ii].x(),
          axis_vectors[jj][ii].y(),
          axis_vectors[jj][ii].z());
      Eigen::Vector3d p_vec(prismatic_vectors[jj][ii].x(),
          prismatic_vectors[jj][ii].y(),
          prismatic_vectors[jj][ii].z());
      Eigen::Vector3d c_mu(constraint_means[ii].x(),
          constraint_means[ii].y(),
          constraint_means[ii].z());
      Eigen::Vector3d p_mu(prismatic_means[ii].x(),
          prismatic_means[ii].y(),
          prismatic_means[ii].z());
      Eigen::Vector3d axis_mu(axis_means[ii].x(),
          axis_means[ii].y(),
          axis_means[ii].z());

      const double p_dist = NormalPDF(dists[jj][ii], dist_means[ii], kDistVar);
      double p_c_vec = MultivariateNormalPDF(c_vec, c_mu, cvec_covariance);
      double p_axis_vec;

      if (fabs(axis_means[ii].x()) < kFPTolerance &&
          fabs(axis_means[ii].y()) < kFPTolerance &&
          fabs(axis_means[ii].z()) < kFPTolerance)
      {
        //p_axis_vec = 1.0;
        p_axis_vec = 0.5;
      }
      else if (fabs(axis_vectors[jj][ii].x()) < kFPTolerance &&
          fabs(axis_vectors[jj][ii].y()) < kFPTolerance &&
          fabs(axis_vectors[jj][ii].z()) < kFPTolerance)
      {
        p_axis_vec = 1.0;
      }
      else
      {
        //p_axis_vec = MultivariateNormalPDF(axis_mu, axis_mu, dir_covariance);
        p_axis_vec = MultivariateNormalPDF(axis_vec, axis_mu, dir_covariance);
      }

      double p_prismatic_vec;

      if (fabs(prismatic_means[ii].x()) < kFPTolerance &&
          fabs(prismatic_means[ii].y()) < kFPTolerance &&
          fabs(prismatic_means[ii].z()) < kFPTolerance)
      {
        //p_prismatic_vec = 1.0;
        p_prismatic_vec = 0.5;
      }
      else if (fabs(prismatic_vectors[jj][ii].x()) < kFPTolerance &&
          fabs(prismatic_vectors[jj][ii].y()) < kFPTolerance &&
          fabs(prismatic_vectors[jj][ii].z()) < kFPTolerance)
      {
        p_prismatic_vec = 1.0;
      }
      else
      {
        // Account for sign change in direction vector
        p_prismatic_vec = MultivariateNormalPDF(p_vec, p_mu, cvec_covariance);
      }

      ROS_DEBUG("%f %f: %f %f %d %d\n", p_dist, p_c_vec, p_prismatic_vec, p_axis_vec, edges[ii].first, edges[ii].second);
      //p_rigid[ii] *= p_dist * p_c_vec;
      if (fabs(p_prismatic_vec - 1.0) < kFPTolerance || fabs(p_axis_vec - 1.0) < kFPTolerance)
      {
        continue;
      }
      p_rigid[ii] *= p_c_vec;
      p_prismatic[ii] *= p_prismatic_vec;
      p_revolute[ii] *= p_axis_vec;
      //p_revolute[ii] *= p_axis_vec * p_dist;
    }
    p_rigid[ii] *=  rigid_prior;
    p_prismatic[ii] *= prismatic_prior;
    p_revolute[ii] *= revolute_prior;

    // Normalize the probabilties
    const double normalizer = p_rigid[ii] + p_prismatic[ii] + p_revolute[ii];
    if (normalizer < kFPTolerance)
    {
      ROS_ERROR("DModel: Normalizer is zero. Error in learning model parameters\n");
    }
    p_rigid[ii] /= normalizer;
    p_prismatic[ii] /= normalizer;
    p_revolute[ii] /= normalizer;
  }

  // Decision tree based on variance
  /*
     const double constraint_thresh = 0.05;
     const double dist_thresh = 0.15; //0.05
     for (int ii = 0; ii < num_edges; ++ii)
     {
  // double cov_det = pow(constraint_cov[ii].determinant(), 0.33);
  double cov_det = constraint_cov[ii].trace();
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
  */

  // Display probabilities
  ROS_DEBUG("DModel: Learnt probabilities\n");
  for (int ii = 0; ii < num_edges; ++ii)
  {
    ROS_DEBUG("Edge (%d %d): %f %f %f\n", edges[ii].first, edges[ii].second, 
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
      e_params.normal = prismatic_means[ii];
    }
    else if (jt == REVOLUTE)
    {
      //e_params.normal = axis_means[ii];
      e_params.normal = tf::Vector3(0.0,0.0,1);
    }
    AddEdge(edges[ii], e_params);
  }
  PrintEdges();
  TFCallback(points_);
  return;
}

void DModel::LearnDModelParametersExperimental(const vector<geometry_msgs::PoseArray>& observations,
    const vector<Edge>& edges)
{
  if (int(points_.poses.size()) == 0)
  {
    ROS_ERROR("DModel: Points have not been set. Cannot learn and initialize edges.\n");
    return;
  }
  const int num_edges = int(edges.size());
  if (edge_map_ != NULL)
  {
    edge_map_->clear();
  }
  const int num_obs = int(observations.size());

  if (num_obs < 2)
  {
    ROS_ERROR("DModel: Not enough observations to learn model from\n");
    return;
  }

  // Edges in local frame. For an edge (x1,x2), c = x2 expressed in frame of x1, and normalized.
  // Number of frames x Number of edges
  vector<vector<tf::Vector3>> local_vectors;
  vector<vector<double>> dists;
  local_vectors.resize(num_obs);
  dists.resize(num_obs);
  for (int ii = 0; ii < num_obs; ++ii)
  {
    local_vectors[ii].resize(num_edges);
    dists[ii].resize(num_edges);
    for (int jj = 0; jj < num_edges; ++jj)
    {
      tf::Vector3 c_vector = GetLocalVector(observations[ii].poses[edges[jj].first], observations[ii].poses[edges[jj].second]);
      const double c_vector_norm = Norm(c_vector);
      if (c_vector_norm > kFPTolerance)
      {
        local_vectors[ii][jj] = c_vector / c_vector_norm;
      }
      dists[ii][jj] = c_vector_norm;
    }
  }

  // Compute the axis vectors for edges in two consecutive frames
  vector<vector<tf::Vector3>> axis_vectors;
  axis_vectors.resize(num_obs - 1);
  vector<vector<tf::Vector3>> prismatic_vectors;
  prismatic_vectors.resize(num_obs - 1);
  for (int ii = 0; ii < num_obs - 1; ++ii)
  {
    axis_vectors[ii].resize(num_edges);
    prismatic_vectors[ii].resize(num_edges);
    for (int jj = 0; jj < num_edges; ++jj) 
    {
      axis_vectors[ii][jj] = Cross(local_vectors[ii][jj], local_vectors[ii + 1][jj]);
      const double axis_vector_norm = Norm(axis_vectors[ii][jj]);
      if (axis_vector_norm > 0.1)
      {
        axis_vectors[ii][jj] = axis_vectors[ii][jj] / axis_vector_norm;
      }
      else
      {
        axis_vectors[ii][jj] = 0*axis_vectors[ii][jj];
      }

      prismatic_vectors[ii][jj] = dists[ii + 1][jj]*local_vectors[ii + 1][jj] - dists[ii][jj] * local_vectors[ii][jj];
      //TODO: This is a hack to account for sign change. 
      if (prismatic_vectors[ii][jj].z() < 0)
      {
        prismatic_vectors[ii][jj] = -prismatic_vectors[ii][jj];
      }

      //if (fabs(dists[ii+1][jj]-dists[ii][jj]) < 0.01)
      if (fabs(dists[ii+1][jj]-dists[ii][jj]) < kFPTolerance)
      {
        prismatic_vectors[ii][jj] = tf::Vector3(0.0, 0.0, 0.0);
      }
      const double prismatic_vector_norm = Norm(prismatic_vectors[ii][jj]);
      if (prismatic_vector_norm > kFPTolerance)
      {
        prismatic_vectors[ii][jj] = prismatic_vectors[ii][jj] / prismatic_vector_norm;
      }
      else
      {
        prismatic_vectors[ii][jj] = 0*prismatic_vectors[ii][jj];
      }
    }
  }

  // DEBUG
  for (int jj = 0; jj < num_edges; ++jj)
  {
    for (int ii = 0; ii < num_obs - 1; ++ii)
    {
      //      ROS_DEBUG("Axis vector: %d %d: %f %f %f\n", edges[jj].first, edges[jj].second, axis_vectors[ii][jj].x(),
      //          axis_vectors[ii][jj].y(), axis_vectors[ii][jj].z());
      ROS_DEBUG("Prismatic vector: %d %d: %f %f %f\n", edges[jj].first, edges[jj].second, prismatic_vectors[ii][jj].x(),
          prismatic_vectors[ii][jj].y(), prismatic_vectors[ii][jj].z());
      ROS_DEBUG("Local vector 1: %d %d: %f %f %f\n", edges[jj].first, edges[jj].second, local_vectors[ii][jj].x(),
          local_vectors[ii][jj].y(), local_vectors[ii][jj].z());
      ROS_DEBUG("Local vector 2: %d %d: %f %f %f\n", edges[jj].first, edges[jj].second, local_vectors[ii+1][jj].x(),
          local_vectors[ii+1][jj].y(), local_vectors[ii+1][jj].z());
      ROS_DEBUG("Dists 1: %f, 2: %f\n", dists[ii][jj], dists[ii+1][jj]);
    }
  }
  vector<tf::Vector3> constraint_means;
  constraint_means.resize(num_edges);
  vector<double> dist_means;
  dist_means.resize(num_edges);
  vector<tf::Vector3> axis_means;
  axis_means.resize(num_edges);
  vector<tf::Vector3> prismatic_means;
  prismatic_means.resize(num_edges);

  // Estimate parameters for all joint types
  for (int ii = 0; ii < num_edges; ++ii)
  {
    constraint_means[ii] = tf::Vector3(0.0, 0.0, 0.0);
    dist_means[ii] = 0.0;
    axis_means[ii] = tf::Vector3(0.0, 0.0, 0.0);
    prismatic_means[ii] = tf::Vector3(0.0, 0.0, 0.0);
    int num_unique_dist_vals = 1;
    for (int jj = 0; jj < num_obs - 1; ++jj)
    {
      constraint_means[ii] = constraint_means[ii] + local_vectors[jj][ii];
      if (fabs(dists[jj][ii] - dists[jj + 1][ii]) > 0.02)
      {
        dist_means[ii] = dist_means[ii] + dists[jj][ii];
        num_unique_dist_vals++;
      }
      axis_means[ii] = axis_means[ii] + axis_vectors[jj][ii];   
      prismatic_means[ii] = prismatic_means[ii] + prismatic_vectors[jj][ii];
    }
    constraint_means[ii] = constraint_means[ii] / (num_obs - 1);
    dist_means[ii] = dist_means[ii] / (num_unique_dist_vals);
    //dist_means[ii] = dist_means[ii] / (num_obs - 1);
    //axis_means[ii] = axis_means[ii] / (num_obs - 1);
    //prismatic_means[ii] = prismatic_means[ii] / (num_obs - 1);
    constraint_means[ii] = constraint_means[ii] / Norm(constraint_means[ii]);
    if (Norm(axis_means[ii]) > 0.001)
    {
      axis_means[ii] = axis_means[ii] / Norm(axis_means[ii]);
    }
    if (Norm(prismatic_means[ii]) > 0.001)
    {
      prismatic_means[ii] = prismatic_means[ii] / Norm(prismatic_means[ii]);
    }
  }

  ROS_DEBUG("Parameter estimates:\n");
  for (int ii = 0; ii < num_edges; ++ii)
  {
    ROS_DEBUG("Edge: %d %d\n", edges[ii].first, edges[ii].second);
    ROS_DEBUG("Dist mean: %f\n", dist_means[ii]);
    ROS_DEBUG("Constraint mean: %f %f %f\n", constraint_means[ii].x(),
        constraint_means[ii].y(), constraint_means[ii].z());
    ROS_DEBUG("Axis mean: %f %f %f\n", axis_means[ii].x(),
        axis_means[ii].y(), axis_means[ii].z());
    ROS_DEBUG("Prismatic mean: %f %f %f\n", prismatic_means[ii].x(),
        prismatic_means[ii].y(), prismatic_means[ii].z());
  }

  // Assign edge params for all edges
  vector<EdgeParams> edge_params;
  edge_params.resize(num_edges);

  vector<double> p_rigid, p_prismatic, p_revolute;
  p_rigid.resize(num_edges);
  p_prismatic.resize(num_edges);
  p_revolute.resize(num_edges);

  // Setup sensor model
  const double kDistVar = 0.009;
  const double kDirVar = 0.03; //0.3
  const double kCVecVar = 0.03; //0.3
  Eigen::Matrix3d cvec_covariance(kDirVar * Eigen::Matrix3d::Identity());
  Eigen::Matrix3d dir_covariance(kDirVar * Eigen::Matrix3d::Identity());

  // Priors
  const double rigid_prior = 0.35;
  const double prismatic_prior = 0.32;
  const double revolute_prior = 0.32;

  // Bayesian inference to assign parameters
  for (int ii = 0; ii < num_edges; ++ii)
  {
    p_rigid[ii] = 1.0;
    p_prismatic[ii] = 1.0;
    p_revolute[ii] = 1.0;

    for (int jj = 0; jj < num_obs - 1; ++jj)
    {
      Eigen::Vector3d c_vec(local_vectors[jj][ii].x(),
          local_vectors[jj][ii].y(),
          local_vectors[jj][ii].z());
      Eigen::Vector3d axis_vec(axis_vectors[jj][ii].x(),
          axis_vectors[jj][ii].y(),
          axis_vectors[jj][ii].z());
      Eigen::Vector3d p_vec(prismatic_vectors[jj][ii].x(),
          prismatic_vectors[jj][ii].y(),
          prismatic_vectors[jj][ii].z());
      Eigen::Vector3d c_mu(constraint_means[ii].x(),
          constraint_means[ii].y(),
          constraint_means[ii].z());
      Eigen::Vector3d p_mu(prismatic_means[ii].x(),
          prismatic_means[ii].y(),
          prismatic_means[ii].z());
      Eigen::Vector3d axis_mu(axis_means[ii].x(),
          axis_means[ii].y(),
          axis_means[ii].z());

      const double p_dist = NormalPDF(dists[jj][ii], dist_means[ii], kDistVar);
      double p_c_vec = MultivariateNormalPDF(c_vec, c_mu, cvec_covariance);
      double p_axis_vec;

      if (fabs(axis_means[ii].x()) < kFPTolerance &&
          fabs(axis_means[ii].y()) < kFPTolerance &&
          fabs(axis_means[ii].z()) < kFPTolerance)
      {
        //p_axis_vec = 1.0;
        p_axis_vec = 0.5;
      }
      else if (fabs(axis_vectors[jj][ii].x()) < kFPTolerance &&
          fabs(axis_vectors[jj][ii].y()) < kFPTolerance &&
          fabs(axis_vectors[jj][ii].z()) < kFPTolerance)
      {
        p_axis_vec = 1.0;
      }
      else
      {
        //p_axis_vec = MultivariateNormalPDF(axis_mu, axis_mu, dir_covariance);
        p_axis_vec = MultivariateNormalPDF(axis_vec, axis_mu, dir_covariance);
      }

      double p_prismatic_vec;

      if (fabs(prismatic_means[ii].x()) < kFPTolerance &&
          fabs(prismatic_means[ii].y()) < kFPTolerance &&
          fabs(prismatic_means[ii].z()) < kFPTolerance)
      {
        //p_prismatic_vec = 1.0;
        p_prismatic_vec = 0.5;
      }
      else if (fabs(prismatic_vectors[jj][ii].x()) < kFPTolerance &&
          fabs(prismatic_vectors[jj][ii].y()) < kFPTolerance &&
          fabs(prismatic_vectors[jj][ii].z()) < kFPTolerance)
      {
        p_prismatic_vec = 1.0;
      }
      else
      {
        // Account for sign change in direction vector
        p_prismatic_vec = MultivariateNormalPDF(p_vec, p_mu, cvec_covariance);
      }

      ROS_DEBUG("%f %f: %f %f %d %d\n", p_dist, p_c_vec, p_prismatic_vec, p_axis_vec, edges[ii].first, edges[ii].second);
      //p_rigid[ii] *= p_dist * p_c_vec;
      if (fabs(p_prismatic_vec - 1.0) < kFPTolerance || fabs(p_axis_vec - 1.0) < kFPTolerance)
      {
        continue;
      }
      p_rigid[ii] *= p_c_vec;
      p_prismatic[ii] *= p_prismatic_vec;
      p_revolute[ii] *= p_axis_vec;
      //p_revolute[ii] *= p_axis_vec * p_dist;
    }
    p_rigid[ii] *=  rigid_prior;
    p_prismatic[ii] *= prismatic_prior;
    p_revolute[ii] *= revolute_prior;

    // Normalize the probabilties
    const double normalizer = p_rigid[ii] + p_prismatic[ii] + p_revolute[ii];
    if (normalizer < kFPTolerance)
    {
      ROS_ERROR("DModel: Normalizer is zero. Error in learning model parameters\n");
    }
    p_rigid[ii] /= normalizer;
    p_prismatic[ii] /= normalizer;
    p_revolute[ii] /= normalizer;
  }

  // Decision tree based on variance
  /*
     const double constraint_thresh = 0.05;
     const double dist_thresh = 0.15; //0.05
     for (int ii = 0; ii < num_edges; ++ii)
     {
  // double cov_det = pow(constraint_cov[ii].determinant(), 0.33);
  double cov_det = constraint_cov[ii].trace();
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
  */

  // Display probabilities
  ROS_DEBUG("DModel: Learnt probabilities\n");
  for (int ii = 0; ii < num_edges; ++ii)
  {
    ROS_DEBUG("Edge (%d %d): %f %f %f\n", edges[ii].first, edges[ii].second, 
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
      e_params.normal = prismatic_means[ii];
    }
    else if (jt == REVOLUTE)
    {
      //e_params.normal = axis_means[ii];
      e_params.normal = tf::Vector3(0.0,0.0,1);
    }
    AddEdge(edges[ii], e_params);
  }
  PrintEdges();
  TFCallback(points_);
  return;
}


tf::Vector3 DModel::GetLocalVector(const geometry_msgs::Pose p1, const geometry_msgs::Pose p2)
{
  //tf::Transform transform;
  //transform.setOrigin(tf::Vector3(-p1.position.x, -p1.position.y, -p1.position.z));
  //transform.setRotation(tf::Quaternion(p1.orientation.x, p1.orientation.y, p1.orientation.z, p1.orientation.w));
  //return transform(tf::Vector3(p2.position.x, p2.position.y, p2.position.z));
  tf::Vector3 trans(p2.position.x - p1.position.x, p2.position.y - p1.position.y, p2.position.z - p1.position.z);
  tf::Quaternion rot(p1.orientation.x, p1.orientation.y, p1.orientation.z, p1.orientation.w);
  return tf::quatRotate(rot.inverse(), trans);
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

State_t DModel::GetStateFromStateID(int state_id)
{
  State_t s = StateIDToState(state_id);
  return s;
}

void DModel::ResetStateMap()
{
  StateMap.clear();
}
