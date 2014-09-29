/**
 * @file d_model_bank.cpp
 * @author Venkatraman Narayanan
 * Carnegie Mellon University, 2014
 */

#include <ltm/d_model_bank.h>
#include <ltm/kinematic_models/prismatic_model.h>
#include <ltm/kinematic_models/revolute_model.h>
#include <ltm/kinematic_models/spherical_model.h>
#include <Eigen/Core>
#include <ros/console.h>

#include <cstdio>
#include <iostream>
#include <set>
#include <queue>

#include <boost/lexical_cast.hpp>

// Multiplier for edge costs to avoid working with floating point numbers.
const double kCostMultiplier = 1e3;
// Error tolerance for comparing goal point locations.
const double kGoalTolerance = 0.2; //0.05, 0.01 for experiments //0.25
const double kGoalEndEffDisp = 0.6;
const bool kNoGoal = true;

using namespace std;

DModelBank::DModelBank(const string& reference_frame)
{
  reference_frame_ = reference_frame;
  num_models_ = 0;

  viz_ = new LTMViz("d_model_viz");
  viz_->SetReferenceFrame(reference_frame);

  // Initialize env params
  env_cfg_.start_state_id = -1;
  env_cfg_.internal_start_state_id = -1;
  env_cfg_.sim_time_step = 0.1;
  visualize_dmodel_ = false;
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

void DModelBank::InitEdgeMap(int num_models)
{
  assert(num_models_ == 0);
  num_models_ = num_models;
  adj_lists_.resize(num_models_);
  for (int ii = 0; ii < num_models_; ++ii)
  {
    EdgeMap* edge_map = new unordered_map<Edge, EdgeParams, pair_hash>();
    edge_maps_.push_back(edge_map);
  }
  viz_->VisualizeModel(*(edge_maps_[0]), points_);
}

void DModelBank::InitFromObs(vector<Edge> edges, vector<vector<EdgeParams>> edge_params)
{
  InitEdgeMap(edge_params.size());
  // Assume points have been set before through SetPoints
  for (int jj = 0; jj < num_models_; ++jj)
  {
    // Edges must be added only after all points have been added.
    for (int ii = 0; ii < edges.size(); ++ii)
    {
      AddEdge(jj, edges[ii], edge_params[jj][ii]);
    } 
  }
  TFCallback(points_);
}


void DModelBank::InitFromFile(vector<string> dmodel_files)
{
  const int num_models = dmodel_files.size();
  vector<double> empty_shifts;
  empty_shifts.resize(num_models, 0.0);
  InitFromFile(dmodel_files, empty_shifts, empty_shifts, empty_shifts);
  return;
}

void DModelBank::InitFromFile(vector<string> dmodel_files, std::vector<double> shifts_x, std::vector<double> shifts_y, std::vector<double> shifts_z)
{
  InitEdgeMap(dmodel_files.size());

  // Read models
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
  ROS_INFO("DModelBank: %d points have been set.\n", int(points_.poses.size())); 
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
  if (adj_lists_[model_id].size() == 0)
  {
    adj_lists_[model_id].resize(points_.poses.size());
  }
  // TODO: Do this in a smarter way?
  if (find(adj_lists_[model_id][e.first].begin(), adj_lists_[model_id][e.first].end(), e.second) == adj_lists_[model_id][e.first].end())
  {
    adj_lists_[model_id][e.first].push_back(e.second);
    adj_lists_[model_id][e.second].push_back(e.first);
    //printf("Size of edge map: %d\n",edge_map->size());
    (*edge_map)[e] = e_params;
    // By default, assume that the reverse connection is of the same type, unless otherwise provided
    EdgeParams reverse_e_params;
    if (e_params.joint != RIGID)
    {
    reverse_e_params.joint = e_params.joint;
    //string first_local = to_string(e.first);
    //string second_local = to_string(e.second);
    string first_local = reference_frame_;
    string second_local = reference_frame_;
    reverse_e_params.normal = TransformVector(e_params.normal, first_local, second_local);
    reverse_e_params.center = TransformPoint(tf::Point(e_params.center.x(), e_params.center.y(), e_params.center.z()), 
                              first_local, second_local);
    (*edge_map)[make_pair(e.second, e.first)] = reverse_e_params;
    }
    else
    {
    (*edge_map)[make_pair(e.second, e.first)] = e_params;
    }
  }
  else
  {
    // Overwrite the joint type
    (*edge_map)[e] = e_params;
  }

  return;
}

void DModelBank::AddPoint(geometry_msgs::Pose p)
{
  points_.poses.push_back(p);
  return;
}

int DModelBank::AddGraspPoint(geometry_msgs::Pose p)
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
  Edge e = make_pair(num_points, closest_p_idx);
  EdgeParams e_params(RIGID, tf::Vector3(0.0, 0.0, 0.0), 1.0);
  AddPoint(p);
  // Must set transform before adding edge
  TFCallback(points_);
  // Add edge in all models
  for (int ii = 0; ii < num_models_; ++ii)
  {
    adj_lists_[ii].resize(num_points + 1);
    AddEdge(ii, e, e_params);
  }
  AddGraspIdx(num_points);
  viz_->VisualizeModel(*(edge_maps_[0]), points_);
  return closest_p_idx;
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

  for (size_t ii = 0; ii < dmodel_points.poses.size(); ++ii)
  {
    static tf::TransformBroadcaster tf_br_;
    geometry_msgs::Pose p = dmodel_points.poses[ii];
    transform.setOrigin(tf::Vector3(p.position.x, p.position.y, p.position.z) );
    transform.setRotation(tf::Quaternion(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w));
    string child_frame_id = boost::lexical_cast<string>(ii);
    /*
    ROS_INFO("Child: %s", child_frame_id.c_str());
    ROS_INFO("Parent: %s", reference_frame_.c_str());
    ROS_INFO("Publishing transform from %s to %s", reference_frame_.c_str(), child_frame_id.c_str());
    ROS_INFO("Rotation: %f %f %f %f", transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z(), transform.getRotation().w());
    ROS_INFO("Translation: %f %f %f", transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
    */
    tf_br_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), reference_frame_.c_str(), child_frame_id.c_str()));

    // Skip the grasp points 
    if (find(grasp_idxs_.begin(), grasp_idxs_.end(), ii) != grasp_idxs_.end())
    {
      continue;
    }
    //points.points.push_back(p.position);
  }

  if (visualize_dmodel_ && edge_maps_.size() != 0)
  {
    // TODO: For now visualize all points, including the grasp points
    // Visualizing the 0th model by default
    // assert(edge_maps_.size() >= 1);
    viz_->VisualizeModel(*(edge_maps_[0]), dmodel_points);
  }
  return;
}

void DModelBank::ExtractIndices(int model_id, int p_idx, vector<int>* component_idxs, vector<int>* sep_idxs, JointType* joint_type,
    tf::Vector3* normal, tf::Vector3* center)
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
    GetAdjPoints(model_id, idx, &adj_points);
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
      // TODO: Don't add the same separator twice
      if (e_params.joint != RIGID) 
      {
        sep_idxs->push_back(adj_points[ii]);
        // TODO: Check for conflicting joint types
        *joint_type = e_params.joint;
        *normal = e_params.normal;
        *center = e_params.center;
        continue;
      }
      // Add to open list and update the frontier list
      o_list.push(adj_points[ii]);
      e_list.insert(adj_points[ii]);
    }
  }
  return;
}

void DModelBank::GetAdjPoints(int model_id, int p_idx, std::vector<int> *adj_points)
{
  adj_points->clear();
  for (size_t ii = 0; ii < adj_lists_[model_id][p_idx].size(); ++ii)
  {
    adj_points->push_back(adj_lists_[model_id][p_idx][ii]);
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
  tf::Vector3 normal, center;
  ExtractIndices(model_id, p_idx, &component_idxs, &sep_idxs, &joint_type, &normal, &center);


  // DEBUG
  /*
     printf("Rigid component has %d points. Joint type is %d and param vector is %0.2f %0.2f %0.2f\n",
     (int)component_idxs.size(), static_cast<int>(joint_type), normal.x(), 
     normal.y(), normal.z());
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

  // Must set up transforms for current state of points
  TFCallback(in_points);

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
  else
  {
    //string target_frame = to_string(closest_p_idx);
    string target_frame = reference_frame_;
    tf::Vector3 transformed_force = TransformVector(force, reference_frame_, target_frame);
    // TODO: The following three lines will go as soon as the EdgeParam is updated to hold the center and axis in local frames
    //center = tf::Vector3(out_points->poses[closest_p_idx].position.x, out_points->poses[closest_p_idx].position.y, out_points->poses[closest_p_idx].position.z);
    //center = TransformPoint(tf::Point(center.x(), center.y(), center.z()), reference_frame_, target_frame);
    //normal = TransformVector(normal, reference_frame_, target_frame);

    // Set up the kinematics model
    AbstractKinematicModel* kinematic_model;
    switch(joint_type)
    {
      case PRISMATIC:
        {
          kinematic_model = new PrismaticModel(target_frame, normal);
          break;
        }
      case REVOLUTE:
        {
          kinematic_model = new RevoluteModel(target_frame, normal, center);
          break;
        }
      case SPHERICAL:
        {
          kinematic_model = new SphericalModel(target_frame, center);
          break;
        }
      default:
        ROS_ERROR("Flow should have never got here");
    }
    assert(kinematic_model != nullptr);

    geometry_msgs::PoseArray transformed_poses;
    geometry_msgs::PoseArray in_poses, out_poses;
    for (size_t ii = 0; ii < component_idxs.size(); ++ii)
    {
      // Transform poses to target frame before applying the kinematics forward model
      geometry_msgs::Pose in_pose = out_points->poses[component_idxs[ii]];
      geometry_msgs::Pose transformed_pose = TransformPose(in_pose, reference_frame_, target_frame);
      in_poses.poses.push_back(transformed_pose);
    }
    geometry_msgs::Pose application_pose = out_points->poses[p_idx];
    kinematic_model->Transform(in_poses, application_pose, transformed_force, del_t, &out_poses);

    for (size_t ii = 0; ii < component_idxs.size(); ++ii)
    {
      // Transform back to reference frame
      out_points->poses[component_idxs[ii]] = TransformPose(out_poses.poses[ii], target_frame, reference_frame_);
    }
    delete kinematic_model;
  }
  return;
}

void DModelBank::GetObservationProbabilities(const State_t& initial_state, const State_t& final_state, const vector<int>& fprim_ids, vector<double>* obs_probs)
{
  const double kObsVar = 0.05; // 20 cm // 1 cm
  // Note: obs_probs doesn't need to normalize
  vector<tf::Vector3> forces;
  vector<int> grasp_points;
  ConvertForcePrimIDsToForcePrims(fprim_ids, &forces, &grasp_points);
  geometry_msgs::PoseArray final_poses, initial_poses;
  GetWorldPosesFromState(initial_state, &initial_poses);
  GetWorldPosesFromState(final_state, &final_poses);
  const int num_poses = int(final_poses.poses.size());
  //ROS_INFO("In poses: %d, Out poses: %d", initial_poses.poses.size(), final_poses.poses.size());
  ROS_INFO("Initial pose: %f %f %f", 
      initial_poses.poses[grasp_idxs_[0]].position.x,
      initial_poses.poses[grasp_idxs_[0]].position.y,
      initial_poses.poses[grasp_idxs_[0]].position.z);
  /*
  ROS_INFO("Executed forces");
  for (int ii = 0; ii < fprim_ids.size(); ++ii)
  {
    tf::Vector3 force = forces[ii];
    ROS_INFO("%f %f %f", force.x(), force.y(), force.z());
  }
  */
  geometry_msgs::PoseArray in_points;
  obs_probs->clear();
  obs_probs->resize(num_models_, 1.0);
  for (int ii = 0; ii < num_models_; ++ii)
  {
    in_points = initial_poses;
    geometry_msgs::PoseArray out_points = in_points;
    for (size_t jj = 0; jj < forces.size(); ++jj)
    {
      GetNextState(ii, in_points, grasp_points[jj], forces[jj], env_cfg_.sim_time_step, &out_points);
      
      // Compression
      /*
      State_t out_state;
      out_state.grasp_idx = grasp_points[jj];
      GetChangedStateFromWorldPoses(out_points, &out_state.changed_inds, &out_state.changed_points);
      int state_id = StateToStateID(out_state);
      out_state = StateIDToState(state_id);
      GetWorldPosesFromState(out_state, &out_points);
      */

      in_points = out_points;
    }
    for (int jj = 0; jj < num_poses; ++jj)
    {
      //ROS_INFO("There are %d points in out_poses", out_points.poses.size());
      // TODO: for now, I am computing probability only on the end-effector pose
      //if (jj != grasp_idxs_[0]) continue; 
      if (jj == grasp_idxs_[0])
      {
        ROS_INFO("Model %d, Obs: %f %f %f, Expected: %f %f %f", ii, 
            final_poses.poses[jj].position.x,
            final_poses.poses[jj].position.y,
            final_poses.poses[jj].position.z,
            out_points.poses[jj].position.x,
            out_points.poses[jj].position.y,
            out_points.poses[jj].position.z);
      }
      (*obs_probs)[ii] *= MultivariateNormalPDF(final_poses.poses[jj], out_points.poses[jj], kObsVar);
    }
  }
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

void DModelBank::VisualizeState(int model_id, int belief_state_id)
{
  // TODO: Create lookup between belief state id and internal state id
  BeliefState_t belief_state = BeliefStateIDToState(belief_state_id);
  int internal_state_id = belief_state.internal_state_id;
  VisualizeInternalState(model_id, internal_state_id);
}

void DModelBank::VisualizeInternalState(int model_id, int internal_state_id)
{
  assert(model_id < edge_maps_.size());

  State_t s = StateIDToState(internal_state_id);
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
  //TODO: harcoded colors for now
  vector<ltm::RGBA> model_colors;
  model_colors.push_back(ltm::RGBA(241.0/255,21.0/255,56.0/255,1.0));
  model_colors.push_back(ltm::RGBA(73.0/255,219.0/255,19.0/255,1.0));
  model_colors.push_back(ltm::RGBA(111.0/255,25.0/255,173.0/255,1.0));
  model_colors.push_back(ltm::RGBA(255.0/255,162.0/255,23.0/255,1.0));
  model_colors.push_back(ltm::RGBA(235.0/255,82.0/255,0.0/255,1.0));
  model_colors.push_back(ltm::RGBA(0.0/255,155.0/255,108.0/255,1.0));
  model_colors.push_back(ltm::RGBA(252.0/255,0.0/255,85.0/255,1.0));
  ltm::RGBA color = model_colors[model_id % model_colors.size()];
  viz_->VisualizeModel(*(edge_maps_[model_id]), points, color, color);
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

int DModelBank::FPrimToFPrimID(int grasp_id, int force_idx)
{
  const int num_grasp_idxs = grasp_idxs_.size();
  return (force_idx * num_grasp_idxs) + grasp_id;
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
    // Do not return a successor if model probability is negligible
    if (source_belief_state.belief[ii] < kFPTolerance)
    {
      continue;
    }
    vector<int> succs;
    vector<int> edge_ids;
    vector<double> costs;
    GetSuccs(ii, internal_state_id, &succs, &edge_ids, &costs);
    for (int jj = 0; jj < int(succs.size()); ++jj)
    { 
      BeliefState_t s;
      s.internal_state_id = succs[jj];
      s.belief.resize(num_models_, 0.0);
      // TODO: Incorporate noisy observation model
      //s.belief[ii] = 1.0/static_cast<double>(num_models_); //1.0
      s.belief[ii] = 1.0;
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

void DModelBank::GetWorldPosesFromState(const State_t& s, geometry_msgs::PoseArray* world_poses)
{
  world_poses->poses.clear();
  for (size_t ii = 0; ii < points_.poses.size(); ++ii)
  {
    // If point has changed, then use the coordinates from state.
    auto it = find(s.changed_inds.begin(),
        s.changed_inds.end(),
        ii);
    if ( it != s.changed_inds.end())
    {
      int offset = distance(s.changed_inds.begin(), it);
      world_poses->poses.push_back(s.changed_points.poses[offset]);
    }
    else
    {
      world_poses->poses.push_back(points_.poses[ii]);
    }
  }
}

void DModelBank::GetChangedStateFromWorldPoses(const geometry_msgs::PoseArray& world_poses, vector<int>* changed_inds, geometry_msgs::PoseArray* changed_points)
{
  // world_poses should not include the grasp poses
  changed_inds->clear();
  changed_points->poses.clear();
  if (world_poses.poses.size() != points_.poses.size())
  {
    ROS_ERROR("Number of world poses (%d) does not match internal number of poses used for initialization (%d)", world_poses.poses.size(), points_.poses.size());
    State_t empty_state;
    return;
  }
  for (size_t ii = 0; ii < points_.poses.size(); ++ii)
  {
    if (!PosesEqual(points_.poses[ii], world_poses.poses[ii]))
    {
      changed_inds->push_back(ii);
      changed_points->poses.push_back(world_poses.poses[ii]);
    }
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
    VisualizeInternalState(model_id, source_state_id);
    usleep(1000);//1000
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

    // DEBUG
    /*
    if (source_state_id == 0)
    {
      for (int kk = 0; kk < out_points.poses.size(); ++kk)
      {
        ROS_INFO("%f %f %f", out_points.poses[kk].position.x,
            out_points.poses[kk].position.y,
            out_points.poses[kk].position.z);
      }
    }
    */

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
    // ROS_INFO("Number of changed points: %d", int(succ_state.changed_inds.size()));

    int succ_id = StateToStateID(succ_state);

    succs->push_back(succ_id);
    // Find the actual grasp id (index of the grasp index)
    int grasp_id = -1;
    for (size_t ii = 0; ii < grasp_idxs_.size(); ++ii)
    {
      if (source_state.grasp_idx == grasp_idxs_[ii])
      {
        grasp_id = ii;
      }
    }
    assert(grasp_id != -1);
    edge_ids->push_back(FPrimToFPrimID(grasp_id, jj));
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
    edge_ids->push_back(FPrimToFPrimID(ii, force_primitives_.size() - 1));
    costs->push_back(int(1 * kCostMultiplier * env_cfg_.sim_time_step));
  }
  // Debug
  /*
  ROS_INFO("Successors for state %d:", source_state_id);
  for (int ii = 0; ii < succs->size(); ++ii)
  {
    ROS_INFO("    %d", (*succs)[ii]);
  }
  */
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

  //Experimental
  if (kNoGoal)
  {
    geometry_msgs::PoseArray state_poses;
    GetWorldPosesFromState(s, &state_poses);
    if (Dist(state_poses.poses[grasp_idxs_[0]].position, points_.poses[grasp_idxs_[0]].position)<kGoalEndEffDisp)
      return false;
    return true;
  }
  

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

bool DModelBank::IsInternalGoalState(State_t state)
{
  int state_id = StateToStateID(state);
  return IsInternalGoalState(state_id);
}

double DModelBank::GetGoalHeuristic(int belief_state_id)
{
  if (IsGoalState(belief_state_id))
  {
    return 0.0;
  }
  BeliefState_t s = BeliefStateIDToState(belief_state_id);
  int internal_state_id = s.internal_state_id;
  const double kHeurMultiplier = 50.0; //10
  return kHeurMultiplier*GetInternalGoalHeuristic(internal_state_id);
}

double DModelBank::GetInternalGoalHeuristic(int internal_state_id)
{

  State_t s = StateIDToState(internal_state_id);
  geometry_msgs::Pose grasp_pose;
  double total_dist = 0;

  geometry_msgs::Pose current_grasp_pose;
  double grasp_point_dist = 0;


  //Experimental
  if (kNoGoal)
  {
    geometry_msgs::PoseArray state_poses;
    GetWorldPosesFromState(s, &state_poses);
    return kCostMultiplier*max(0.0, kGoalEndEffDisp - Dist(state_poses.poses[grasp_idxs_[0]].position, points_.poses[grasp_idxs_[0]].position))/env_cfg_.sim_time_step;
  }

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
  if (start_state_id == env_cfg_.start_state_id)
  {
    ROS_INFO("DModelBank: Start belief state to be set is same as previous start belief state"); 
    return;
  }
  ROS_INFO("DModelBank: Setting start state: %d\n", start_state_id);
  env_cfg_.start_state_id = start_state_id;
  return; 
}

void DModelBank::SetInternalStartState(State_t start_state)
{
  int start_state_id = StateToStateID(start_state);
  if (start_state_id == env_cfg_.internal_start_state_id)
  {
    ROS_INFO("DModelBank: Start state to be set is same as previous start state"); 
    return;
  }
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
  //Note: Goal state must be set after start state always
  if (env_cfg_.internal_start_state_id == -1)
  {
    ROS_ERROR("[DModelBank: Cannot set goal state before start state has been set");
  }
  int goal_state_id = StateToStateID(goal_state);
  if (goal_state_id == env_cfg_.internal_goal_state_id)
  {
    ROS_DEBUG("DModelBank: Goal state to be set is same as previous goal state"); 
    return;
  }
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
  for (size_t ii = 0; ii < fprim_ids.size(); ++ii)
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
  traj->header.frame_id = reference_frame_;
  traj->header.stamp = ros::Time::now();
  for (size_t ii = 0; ii < state_ids.size(); ++ii)
  {
    BeliefState_t belief_state = BeliefStateIDToState(state_ids[ii]);
    int internal_state_id = belief_state.internal_state_id;
    State_t s = StateIDToState(internal_state_id);
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

// Transforms et al.

tf::StampedTransform DModelBank::GetTransform(std::string& from_frame, std::string& to_frame)
{
  tf::StampedTransform transform;
  try
  {
    //NOTE: For some reason, listener_.transformVector doesn't seem to work. This must be debugged later.
    listener_.lookupTransform(to_frame, from_frame, ros::Time(0), transform);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }
  return transform;
}

geometry_msgs::Pose DModelBank::TransformPose(const geometry_msgs::Pose& in_pose, std::string& from_frame, std::string& to_frame)
{
  geometry_msgs::PoseStamped stamped_in_pose, stamped_out_pose;
  stamped_in_pose.pose = in_pose;
  stamped_in_pose.header.frame_id = from_frame;
  stamped_in_pose.header.stamp = ros::Time(0);
  listener_.transformPose(to_frame, ros::Time(0), stamped_in_pose, from_frame, stamped_out_pose);
  return stamped_out_pose.pose;
}

tf::Vector3 DModelBank::TransformVector(const tf::Vector3& in_vec, std::string& from_frame, std::string& to_frame)
{
  tf::Vector3 out_vec;
  tf::Stamped<tf::Vector3> stamped_in_vec(in_vec, ros::Time(0), from_frame), stamped_out_vec;
  listener_.transformVector(to_frame, ros::Time(0), stamped_in_vec, from_frame, stamped_out_vec);
  out_vec = stamped_out_vec;
  return out_vec;
}

tf::Point DModelBank::TransformPoint(const tf::Point& in_point, std::string& from_frame, std::string& to_frame)
{
  tf::Point out_point;
  tf::Stamped<tf::Point> stamped_in_point(in_point, ros::Time(0), from_frame), stamped_out_point;
  listener_.transformPoint(to_frame, ros::Time(0), stamped_in_point, from_frame, stamped_out_point);
  out_point = stamped_out_point;
  return out_point;
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
