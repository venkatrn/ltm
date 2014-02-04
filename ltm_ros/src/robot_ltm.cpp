/**
 * @file real_ltm.cpp
 * @author Venkatraman Narayanan
 * Carnegie Mellon University, 2014
 */

#include <ltm_ros/robot_ltm.h>
#include <ltm/d_model.h>
#include <ltm/d_model_utils.h>

#include <pcl/kdtree/kdtree_flann.h>

#include <algorithm>

using namespace std;

RobotLTM::RobotLTM() : grasp_idx_(-1),
  ar_marker_tracking_(false)
{
  ros::NodeHandle private_nh("~");
  private_nh.param("use_model_file", use_model_file_, false);
  private_nh.param("model_file", model_file_, string(""));
  private_nh.param("fprims_file", fprims_file_, string(""));
  private_nh.param("reference_frame", reference_frame_, string("/map"));
  private_nh.param("sim_time_step", sim_time_step_, 0.1);
  private_nh.param("model_offset_x", model_offset_x_, 0.0);
  private_nh.param("model_offset_y", model_offset_y_, 0.0);
  private_nh.param("model_offset_z", model_offset_z_, 0.0);

  d_model_ = new DModel(reference_frame_);
  planner_ = new DModelPlanner;

  // Initialize force primitives
  d_model_->InitForcePrimsFromFile(fprims_file_.c_str());

  // Setup publsihers
  plan_pub_ = nh_.advertise<geometry_msgs::PoseArray>("ltm_plan", 1);

  // Setup subscribers
  if (!use_model_file_)
  {
    cloud_sub_ = nh_.subscribe ("d_model_structure", 1, &RobotLTM::ModelCB, this);
  }
  else
  {
    SetModelFromFile(model_file_.c_str());
    ROS_INFO("LTM Node: Initialized model from file\n");
  }
  goal_sub_ = nh_.subscribe ("goal_pose", 1, &RobotLTM::GoalCB, this);
  grasp_sub_ = nh_.subscribe ("gripper_pose", 1, &RobotLTM::GraspCB, this);
  traj_exec_sub_ = nh_.subscribe ("traj_exec", 1, &RobotLTM::TrajExecCB, this);
  learning_mode_sub_ = nh_.subscribe ("learning_mode", 1, &RobotLTM::LearnCB, this);
  ar_marker_sub_ = nh_.subscribe("ar_pose_marker", 1, &RobotLTM::ARMarkersCB, this);

  // Setup model and planner.
  planner_->SetModel(d_model_);

  // Initialize IK clients
  query_client_ = nh_.serviceClient<kinematics_msgs::GetKinematicSolverInfo>("pr2_right_arm_kinematics/get_ik_solver_info");
  ik_client_ = nh_.serviceClient<kinematics_msgs::GetPositionIK>("pr2_right_arm_kinematics/get_ik");
}

RobotLTM::~RobotLTM() 
{
  delete d_model_;
  delete planner_;
}

void RobotLTM::ModelCB(const ltm_msgs::DModel& d_model)
{
  // Update model (learn parameters)
  // Set start
  State_t start_state;
  d_model_->SetStartState(start_state);
}

void RobotLTM::LearnCB(const std_msgs::Int32ConstPtr& learning_mode)
{
  // Learning modes: 0-Learning not yet started, 1-Learning in progress, 2-Learning is complete
  if (learning_mode->data == 0)
  {
    ar_marker_tracking_ = false;
    return;
  }

  if (learning_mode->data == 1)
  {
    // Trigger AR Marker tracking
    ar_marker_tracking_ = true;
    // Reset observations and edges
    observations_.clear();
    edges_.clear();

    return;
  }

  if (learning_mode->data == 2)
  {
    // Stop AR Marker tracking, and pass observations to d_model
    ar_marker_tracking_ = false;
    
    if (int(observations_.size()) == 0)
    {
      ROS_INFO("LTM Node: Learning stopped before any observations have been received"
          "No model will be learnt");
      return;
    }

    ComputeEdges(observations_[0]);
    ROS_INFO("LTM Node: Number of edges in model: %d", int(edges_.size()));
    d_model_->LearnDModelParameters(observations_, edges_);
    return;
  }
}

void RobotLTM::GraspCB(const geometry_msgs::PoseStampedConstPtr& grasp_pose)
{
  ROS_INFO("LTM Node: Received grasp pose\n");

  // Transform goal pose to reference frame
  geometry_msgs::PoseStamped grasp_pose_ref_frame;
  tf_listener_.waitForTransform(grasp_pose->header.frame_id, reference_frame_, ros::Time::now(), ros::Duration(3.0));
  tf_listener_.transformPose(reference_frame_, *grasp_pose, grasp_pose_ref_frame);
  // Set the grasp pose, to transform the end effector trajectory later
  grasp_pose_ = grasp_pose_ref_frame.pose;
  d_model_->AddGraspPoint(grasp_pose_);

  // Set grasp index by finding closest pose in dmodel points.
  // TODO: This is redundant now.
  int closest_idx = 0;
  double min_dist = 1000;
  geometry_msgs::PoseArray points = d_model_->GetDModelPoints();
  for (size_t ii = 0; ii < points.poses.size(); ++ii)
  {
    //TODO: Include orientation
    const double dist = Dist(grasp_pose_ref_frame.pose.position, points.poses[ii].position);
    if (dist < min_dist)
    {
      min_dist = dist;
      closest_idx = int(ii);
    }
  }
  grasp_idx_ = closest_idx;
  ROS_INFO("LTM Node: The closest point for grasp pose is %d\n", grasp_idx_);
  d_model_->SetForceIndex(grasp_idx_);
  return;
}

void RobotLTM::GoalCB(const geometry_msgs::PoseStampedConstPtr& goal_pose)
{
  ROS_INFO("LTM Node: Received new goal");
  //TODO: Fake a start state for now. Should be set elsewhere
  State_t start_state;
  d_model_->SetStartState(start_state);
  d_model_->SetSimTimeStep(sim_time_step_);

  // Transform goal pose to reference frame
  geometry_msgs::PoseStamped goal_pose_ref_frame;
  tf_listener_.waitForTransform(goal_pose->header.frame_id, reference_frame_, ros::Time::now(), ros::Duration(3.0));
  tf_listener_.transformPose(reference_frame_, *goal_pose, goal_pose_ref_frame);

  // Set goal
  State_t goal_state;
  if (grasp_idx_ == -1)
  {
    ROS_INFO("LTM Node: Grasp point has not been set. Will attempt to take point nearest to goal pose as the grasp point.\n");
  }

  // Offset the effector goal pose by the position of the nearest point on D-Model.
  // This assumes a rigid connection between the end-effector location and the closest D-model point
  /*
  geometry_msgs::PoseArray points = d_model_->GetDModelPoints();
  geometry_msgs::Pose closest_point = points.poses[grasp_idx_];
  geometry_msgs::Point offset;
  offset.x = grasp_pose_.position.x - closest_point.position.x;
  offset.y = grasp_pose_.position.y - closest_point.position.y;
  offset.z = grasp_pose_.position.z - closest_point.position.z;
  goal_pose_ref_frame.pose.position.x -= offset.x; 
  goal_pose_ref_frame.pose.position.y -= offset.y; 
  goal_pose_ref_frame.pose.position.z -= offset.z; 
  */

  goal_state.changed_inds.push_back(grasp_idx_);
  goal_state.changed_points.poses.push_back(goal_pose_ref_frame.pose);
  ROS_INFO("LTM Node: New goal received: %f %f %f\n",
      goal_pose_ref_frame.pose.position.x, goal_pose_ref_frame.pose.position.y,
      goal_pose_ref_frame.pose.position.z);
  d_model_->SetGoalState(goal_state);


  // Plan
  planner_->SetStart(d_model_->GetStartStateID());
  planner_->SetEpsilon(1000.0);
  vector<int> state_ids, fprim_ids;
  if (!planner_->Plan(&state_ids, &fprim_ids))
  {
    printf("LTM Node: Failed to plan\n");
    return;
  }

  // Get forces from fprim_ids
  vector<tf::Vector3> forces;
  forces.clear();
  d_model_->ConvertForcePrimIDsToForces(fprim_ids, &forces);

  /*
  printf("Force Sequence:\n");
  for (size_t ii = 0; ii < forces.size(); ++ii)
  {
    printf("%f %f %f\n", forces[ii].x(), forces[ii].y(), forces[ii].z());
  }
  printf("\n");
  */

  // Print planner stats
  PlannerStats planner_stats = planner_->GetPlannerStats();
  ROS_INFO("Planner Stats:\nNum Expansions: %d, Planning Time: %f, Solution Cost: %d\n",
      planner_stats.expansions, planner_stats.time, planner_stats.cost);


  // Publish plan
  geometry_msgs::PoseArray traj;
  d_model_->GetEndEffectorTrajFromStateIDs(state_ids, &traj);

  // Transform end effector traj according to grasp pose
  /*
  tf::Quaternion q1 = tf::Quaternion(traj.poses[0].orientation.x, traj.poses[0].orientation.y, traj.poses[0].orientation.z, traj.poses[0].orientation.w);
tf::Quaternion q_grasp = tf::Quaternion(grasp_pose_.orientation.x, grasp_pose_.orientation.y, grasp_pose_.orientation.z, grasp_pose_.orientation.w);
  geometry_msgs::Point t1 = traj.poses[0].position;
  tf::Quaternion q_correction = q_grasp*q1.inverse();

  for (size_t ii = 0; ii < traj.poses.size(); ++ii)
  {
    geometry_msgs::Point offset; 
    offset.x = traj.poses[ii].position.x - t1.x;
    offset.y = traj.poses[ii].position.y - t1.y;
    offset.z = traj.poses[ii].position.z - t1.z;
    tf::Quaternion q2 = tf::Quaternion(traj.poses[ii].orientation.x, traj.poses[ii].orientation.y, traj.poses[ii].orientation.z, traj.poses[ii].orientation.w);
    //tf::Quaternion rotation = q2.inverse()*q1;
    //tf::Quaternion orientation = rotation * q_grasp;
    tf::Quaternion orientation = q_correction*q2;
    traj.poses[ii].position.x  = grasp_pose_.position.x + offset.x;
    traj.poses[ii].position.y  = grasp_pose_.position.y + offset.y;
    traj.poses[ii].position.z  = grasp_pose_.position.z + offset.z;
    traj.poses[ii].orientation.x = orientation.x();
    traj.poses[ii].orientation.y = orientation.y();
    traj.poses[ii].orientation.z = orientation.z();
    traj.poses[ii].orientation.w = orientation.w();
  }
  */

  plan_pub_.publish(traj);

  // Simulate plan 
  // d_model_->SimulatePlan(fprim_ids);
  SimulatePlan(traj, state_ids, forces);

  //Print end effector trajectory
  printf("End-Effector Trajectory:\n");
  for (size_t ii = 0; ii < traj.poses.size(); ++ii)
  {
    printf("%f %f %f %f %f %f %f\n", traj.poses[ii].position.x, traj.poses[ii].position.y, traj.poses[ii].position.z,
        traj.poses[ii].orientation.x, traj.poses[ii].orientation.y, traj.poses[ii].orientation.z, traj.poses[ii].orientation.w);
  }
  printf("\n");

  return;
}


void RobotLTM::TrajExecCB(const std_msgs::Int32ConstPtr& traj_idx_ptr)
{
  // d_model_->SimulatePlan(forces);
  /*
     int traj_idx = static_cast<int>(traj_idx_ptr->data) - 1;
     if (traj_idx >= int(forces.size()))
     {
     ROS_INFO("LTM Node: Trajectory index exceeds limits. Error simulating path.\n");
     }
     ROS_INFO("LTM Node: Simulating trajectory index %d\n", traj_idx);
     d_model_->ApplyForce(grasp_idx_, forces[traj_idx], sim_time_step_);
     usleep(10000);
     */
  return;
}

void RobotLTM::ARMarkersCB(const ar_track_alvar_msgs::AlvarMarkersConstPtr& ar_markers)
{
  // Do not store poses if not in learning mode
  if (!ar_marker_tracking_)
  {
    return;
  }
  
  const int num_tracked_markers = 3;
  // Ensure we have same number of markers in each frame
  const int num_markers = int(ar_markers->markers.size());
  if (num_markers != num_tracked_markers)
  {
    return;
  }
  if (int(observations_.size()) != 0)
  {
    // if (num_markers != int(observations_[0].poses.size()))
    if (num_markers != num_tracked_markers)
    {
      ROS_INFO("LTM Node: Number of markers ,%d in current frame does not match number being tracked %d. Skipping observation",
          num_markers, int(observations_[0].poses.size()));
      return;
    }
  }
  /*
  vector<int> id_mappings;
  for (int ii =0; ii < num_markers; ++ii)
  {
    id_mappings.push_back(ar_markers->markers[ii].id);
  }
  */

  // Update the d_model state
  geometry_msgs::PoseArray temp_pose_array, pose_array;
  pose_array.poses.resize(num_markers);
  for (int ii = 0; ii < num_markers; ++ii)
  {
    // Fill up valid header for AR marker poses
    geometry_msgs::PoseStamped ar_marker_pose = ar_markers->markers[ii].pose;
    ar_marker_pose.header.frame_id = ar_markers->markers[ii].header.frame_id;
    ar_marker_pose.header.stamp = ros::Time::now();
    // Transform goal pose to reference frame
    geometry_msgs::PoseStamped grasp_pose_ref_frame;
    tf_listener_.waitForTransform(ar_markers->markers[ii].header.frame_id, reference_frame_, ros::Time::now(), ros::Duration(3.0));
    tf_listener_.transformPose(reference_frame_, ar_marker_pose, grasp_pose_ref_frame);
    //auto id_map_it = find(id_mappings.begin(), id_mappings.end(), ar_markers->markers[ii].id);
    //int mapped_id = distance(id_mappings.begin(), id_map_it);
    // pose_array.poses[mapped_id] = grasp_pose_ref_frame.pose;
    temp_pose_array.poses.push_back(grasp_pose_ref_frame.pose);
  }

  if (observations_.size() != 0)
  {
  // Associate trackings
  const int kKNNSearchK = 1;

  // Create point cloud for first frame.
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  cloud->width = num_markers;
  cloud->height = 1;
  cloud->points.resize(cloud->width * cloud->height);
  for (size_t ii = 0; ii < num_markers; ++ii)
  {
    cloud->points[ii].x = temp_pose_array.poses[ii].position.x;
    cloud->points[ii].y = temp_pose_array.poses[ii].position.y;
    cloud->points[ii].z = temp_pose_array.poses[ii].position.z;
  }

  // Get nearest neighbors for each tracked point, in the first frame.
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud (cloud);

  for (size_t ii = 0; ii < num_markers; ++ii)
  {
    vector<int> idxs;
    vector<float> sqr_distances;
    pcl::PointXYZ search_point;
    search_point.x = observations_.back().poses[ii].position.x;
    search_point.y = observations_.back().poses[ii].position.y;
    search_point.z = observations_.back().poses[ii].position.z;
    kdtree.nearestKSearch(search_point, kKNNSearchK, idxs, sqr_distances);
    pose_array.poses[ii] = temp_pose_array.poses[idxs[0]];
  }
  }
  else
  {
    pose_array = temp_pose_array;
  }

  d_model_->SetPoints(pose_array);

  observations_.push_back(pose_array);
  //sleep(1);
  return;
}

void RobotLTM::SetModelFromFile(const char *model_file)
{
  d_model_->InitFromFile(model_file, model_offset_x_, model_offset_y_, model_offset_z_); 
  // Set start
  State_t start_state;
  d_model_->SetStartState(start_state);
}

void RobotLTM::SimulatePlan(const geometry_msgs::PoseArray& plan, const vector<int>& state_ids, const vector<tf::Vector3>& forces)
{
  // TODO: Check that plan and forces are of equal length
  
  vector<double> pose(7,0);
  vector<double> seed(7,0);
  vector<double> r_angles(7,0);
  vector<double> initial_r_angles(7,0);
  vector<double> l_angles(7,0);
  
  // Set default left-arm position
  l_angles[0] = 0.06024;
  l_angles[1] = 1.248526;
  l_angles[2] = 1.789070;
  l_angles[3] = -1.683386;
  l_angles[4] = -1.7343417;
  l_angles[5] = -0.0962141;
  l_angles[6] =  -0.0864407;

  kinematics_msgs::GetKinematicSolverInfo::Response response;
  kinematics_msgs::GetKinematicSolverInfo::Request request;
  query_client_.call(request, response);

  for (int ii = 0; ii < 7; ++ii)
  {
    seed[ii] = (response.kinematic_solver_info.limits[ii].min_position + response.kinematic_solver_info.limits[ii].max_position)/2.0;
  }

  vector<double> base_pos(6,0);
  const double torso_pos = 0.3;
  const double hue = 0.5;
  const string ns = "ltm_plan";
  const bool use_pr2_mesh = true;
  
  const double standoff = 0.0; //0.15

  for (size_t ii = 0; ii < forces.size(); ++ii)
  {
    // Get the IK for the end-effector
    pose[0] = plan.poses[ii].position.x - standoff;
    pose[1] = plan.poses[ii].position.y;
    pose[2] = plan.poses[ii].position.z;
    pose[3] = plan.poses[ii].orientation.w;
    pose[4] = plan.poses[ii].orientation.x;
    pose[5] = plan.poses[ii].orientation.y;
    pose[6] = plan.poses[ii].orientation.z;

    if (GetRightIK(pose, seed, &r_angles))
    {
      pviz_.visualizeRobot(r_angles,
          l_angles,
          base_pos,  // 0: x, 1: y,  2: theta,  3: head_pan,  4: head_tilt,  5: laser_tilt
          torso_pos, 
          hue,
          ns, 1, use_pr2_mesh);
    }
    else
    {
      ROS_ERROR("LTM Node: Could not get IK while simulating");
      //return;
    }

    if (ii == 0)
    {
      initial_r_angles = r_angles;
    }

    // Draw the model at next timestep
    // d_model_->ApplyForce(grasp_idx_, forces[ii], sim_time_step_);
    d_model_->VisualizeState(state_ids[ii]);
    
    seed.swap(r_angles);
    usleep(100000);
  }

  // Visualize the initial state after running through simulation
  sleep(2);
  d_model_->VisualizeState(state_ids[0]);
  pviz_.visualizeRobot(initial_r_angles,
      l_angles,
      base_pos,  // 0: x, 1: y,  2: theta,  3: head_pan,  4: head_tilt,  5: laser_tilt
      torso_pos, 
      hue,
      ns, 1, use_pr2_mesh);
  return;
}

bool RobotLTM::GetRightIK(const vector<double>& ik_pose, const vector<double>& seed, vector<double>* angles){

  // define the service messages
  kinematics_msgs::GetKinematicSolverInfo::Request request;
  kinematics_msgs::GetKinematicSolverInfo::Response response;

  if(query_client_.call(request,response))
  {
    for(unsigned int i=0; i< response.kinematic_solver_info.joint_names.size(); i++)
    {
      ROS_DEBUG("LTM Node: Joint: %d %s",i,response.kinematic_solver_info.joint_names[i].c_str());
    }
  }
  else
  {
    ROS_ERROR("LTM Node: Could not call query service");
    return false;
  }
  // define the service messages

  kinematics_msgs::GetPositionIK::Request  gpik_req;
  kinematics_msgs::GetPositionIK::Response gpik_res;
  gpik_req.timeout = ros::Duration(5.0);
  gpik_req.ik_request.ik_link_name = "r_wrist_roll_link";

  //gpik_req.ik_request.pose_stamped.header.frame_id = "torso_lift_link";
  gpik_req.ik_request.pose_stamped.header.frame_id = reference_frame_;
  gpik_req.ik_request.pose_stamped.pose.position.x = ik_pose[0];
  gpik_req.ik_request.pose_stamped.pose.position.y = ik_pose[1];
  gpik_req.ik_request.pose_stamped.pose.position.z = ik_pose[2];

  gpik_req.ik_request.pose_stamped.pose.orientation.w = ik_pose[3];
  gpik_req.ik_request.pose_stamped.pose.orientation.x = ik_pose[4];
  gpik_req.ik_request.pose_stamped.pose.orientation.y = ik_pose[5];
  gpik_req.ik_request.pose_stamped.pose.orientation.z = ik_pose[6];

  gpik_req.ik_request.ik_seed_state.joint_state.position.resize(response.kinematic_solver_info.joint_names.size());
  gpik_req.ik_request.ik_seed_state.joint_state.name = response.kinematic_solver_info.joint_names;

  for(unsigned int i=0; i< response.kinematic_solver_info.joint_names.size(); i++)
  {
    if ((int)seed.size() != 0)
    {
      gpik_req.ik_request.ik_seed_state.joint_state.position[i] = seed[i]; 
    }
    else
    {
      gpik_req.ik_request.ik_seed_state.joint_state.position[i] = (response.kinematic_solver_info.limits[i].min_position + response.kinematic_solver_info.limits[i].max_position)/2.0;
    }
  }

  if(ik_client_.call(gpik_req, gpik_res))
  {
    if(gpik_res.error_code.val == gpik_res.error_code.SUCCESS){
      ROS_DEBUG("LTM Node: Got IK:");
      for(unsigned int i=0; i < gpik_res.solution.joint_state.name.size(); i ++){
        ROS_DEBUG("LTM Node: Joint: %s %f",gpik_res.solution.joint_state.name[i].c_str(),gpik_res.solution.joint_state.position[i]);
        (*angles)[i] = gpik_res.solution.joint_state.position[i];
      }
      return true;
    } else{
      ROS_ERROR("LTM Node: Inverse kinematics to %.3f, %.3f, %.3f failed", ik_pose[0], ik_pose[1], ik_pose[2]);
      return false;
    }
  } else {
    ROS_ERROR("LTM Node: Inverse kinematics service call failed");
    return false; 
  }
  return false;
}

void RobotLTM::ComputeEdges(const geometry_msgs::PoseArray& pose_array)
{
  // Clear existing edges
  edges_.clear();

  const double kKNNSearchRadius = 0.5;
  const int kKNNSearchK = 2;
  
  const size_t num_points = pose_array.poses.size();

  // Create point cloud for first frame.
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  cloud->width = num_points;
  cloud->height = 1;
  cloud->points.resize(cloud->width * cloud->height);
  for (size_t ii = 0; ii < num_points; ++ii)
  {
    cloud->points[ii].x = pose_array.poses[ii].position.x;
    cloud->points[ii].y = pose_array.poses[ii].position.y;
    cloud->points[ii].z = pose_array.poses[ii].position.z;
  }

  // Get nearest neighbors for each tracked point, in the first frame.
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud (cloud);

  for (size_t ii = 0; ii < num_points; ++ii)
  {
    vector<int> idxs;
    vector<float> sqr_distances;
    kdtree.radiusSearch(cloud->points[ii], kKNNSearchRadius, idxs, sqr_distances);
    // kdtree.nearestKSearch(cloud->points[ii], kKNNSearchK, idxs, distances);
    for (size_t jj = 0; jj < idxs.size(); ++jj)
    {
      // TODO: Do distance checking if using knn search instead of radius search.
      // No self loops
      if (ii == idxs[jj])
      {
        continue;
      }
      edges_.push_back(make_pair(ii, idxs[jj]));
    }
  }
  return;
  
  return;
}
