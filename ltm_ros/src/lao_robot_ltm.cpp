/*
* @file lao_robot_ltm.cpp
 * @author Venkatraman Narayanan
 * Carnegie Mellon University, 2014
 */

#include <ltm_ros/lao_robot_ltm.h>
#include <ltm/d_model_bank.h>
#include <ltm/d_model_utils.h>
#include <ltm/kinematic_models/abstract_kinematic_model.h>

#include <pcl/kdtree/kdtree_flann.h>

#include <boost/lexical_cast.hpp>

#include <algorithm>

#include <ros/package.h>

using namespace std;

// Max joint velocity for PR2 arm
const double kMaxJointVel = 0.5;
// Home position for the right arm
//const vector<float> kRightArmHomeConfig = {0.564, 1.296, -0.042, -1.521, -3.273, -1.644, -40.441};
const bool run_experiments_ = false;
//const vector<float> kRightArmHomeConfig = {0.564, 1.296, -0.042, -1.521, -3.273, -1.644, -40.441};
//const vector<float> kRightArmHomeConfig = {0.260, -0.339, 0.301, -1.141, 2.966, -1.559, -9.704};//kitchen-top cabinet
const vector<float> kRightArmHomeConfig = {0.564, 0.441, -0.106, -2.117, -22.554, -1.823, -1.682}; //lab-door


double kGripperOffsetX = 0.15;
double kGripperOffsetY = -0.015; //-0.02
double kGripperOffsetZ = 0.1;


/*
kRightArmHomeConfig[0] = 0.564;
kRightArmHomeConfig[1] = 1.296;
kRightArmHomeConfig[2] = -0.042;
kRightArmHomeConfig[3] = -1.521;
kRightArmHomeConfig[4] = -3.273;
kRightArmHomeConfig[5] = -1.644;
kRightArmHomeConfig[6] = -40.441;
*/
string kPlannerStatsBase =  ros::package::getPath("ltm_ros") + "/stats/planner_stats";
string kBeliefStatsBase =  ros::package::getPath("ltm_ros") + "/stats/belief_stats";

LAORobotLTM::LAORobotLTM() : record_observations_(false),
                             record_bag_(false),
                             grasped_(false),
                             experiment_num_(0)
{
  ros::NodeHandle private_nh("~");
  private_nh.param("use_model_file", use_model_file_, false);
  private_nh.param("enforce_spatial_association", enforce_spatial_association_, true);
  vector<string> empty_model_files;
  private_nh.param("model_files", model_files_, empty_model_files);
  private_nh.param("fprims_file", fprims_file_, string(""));
  private_nh.param("obs_file", obs_file_, string("observations.txt"));
  private_nh.param("bag_file", bag_file_, string("observations.bag"));
  private_nh.param("reference_frame", reference_frame_, string("/map"));
  private_nh.param("sim_time_step", sim_time_step_, 0.1);
  private_nh.param("simulation", simulation_, true);
  private_nh.param("full_trajectory_execution", default_full_trajectory_execution_, true);
  private_nh.param("num_partial_traj_waypoints", num_partial_traj_waypoints_, 8);

  ResetStateMachine();
  grasp_idxs_.clear(); //grasp_idxs are not part of state machine--they are stored for good once received, unless deliberately cleared by the user
  received_handle_ = false; // and so is the handle

  d_model_bank_ = new DModelBank(reference_frame_);
  planner_ = new LAOPlanner();
  learner_ = new DModelLearner(reference_frame_);
  viz_ = new LTMViz("ltm_node_viz");
  viz_->SetReferenceFrame(reference_frame_);
  // Initialize force primitives and other dmodel_bank_ parameters
  d_model_bank_->InitForcePrimsFromFile(fprims_file_.c_str());
  d_model_bank_->SetSimTimeStep(sim_time_step_);
  planner_->SetModelBank(d_model_bank_);

  // Setup publsihers
  plan_pub_ = nh_.advertise<geometry_msgs::PoseArray>("ltm_plan", 1);

  // Setup subscribers
  if (!use_model_file_)
  {
    // TODO: Setup a subscriber to get the updated d-model structure
  }
  else
  {
    num_models_ = model_files_.size();
    current_belief_.clear();
    current_belief_.resize(num_models_, 1.0/static_cast<double>(num_models_));
    vector<double> empty_model_offsets;
    empty_model_offsets.resize(num_models_);
    private_nh.param("model_offsets_x", model_offsets_x_, empty_model_offsets);
    private_nh.param("model_offsets_y", model_offsets_y_, empty_model_offsets);
    private_nh.param("model_offsets_z", model_offsets_z_, empty_model_offsets);
    ROS_INFO("[LAO Robot LTM]: Number of models in bank: %d", num_models_);
    d_model_bank_->InitFromFile(model_files_, model_offsets_x_, model_offsets_y_, model_offsets_z_); 
    ROS_INFO("LTM Node: Initialized model from file\n");
    // Setup planner.
  }
  // TODO: Make all the topic names rosparams.
  point_cloud_sub_ = nh_.subscribe ("/kinect_head/depth_registered/points_throttle", 1, &LAORobotLTM::KinectCB, this);
  goal_sub_ = nh_.subscribe ("goal_pose", 1, &LAORobotLTM::GoalCB, this);
  grasp_sub_ = nh_.subscribe ("gripper_pose", 1, &LAORobotLTM::GraspCB, this);
  traj_exec_sub_ = nh_.subscribe ("traj_exec_mode", 1, &LAORobotLTM::TrajExecCB, this);
  learning_mode_sub_ = nh_.subscribe ("learning_mode", 1, &LAORobotLTM::LearnCB, this);
  ar_marker_sub_ = nh_.subscribe("ar_pose_marker", 1, &LAORobotLTM::ARMarkersCB, this);
  perception_sub_ = nh_.subscribe("rectangles", 1, &LAORobotLTM::PerceptionCB, this);
  handle_sub_ = nh_.subscribe("localization/handle_list", 1, &LAORobotLTM::HandleCB, this);

  // Initialize IK clients
  query_client_ = nh_.serviceClient<kinematics_msgs::GetKinematicSolverInfo>("pr2_right_arm_kinematics/get_ik_solver_info");
  ik_client_ = nh_.serviceClient<kinematics_msgs::GetPositionIK>("pr2_right_arm_kinematics/get_ik");
  // Initialize arm
  r_arm_ = new Arm("right");
}

LAORobotLTM::~LAORobotLTM() 
{
  delete d_model_bank_;
  delete planner_;
  delete learner_;
  if (r_arm_ != nullptr) delete r_arm_;
}

void LAORobotLTM::ResetStateMachine()
{
  num_goals_received_ = 0;
  grasped_ = false;
  observations_.clear();
  full_trajectory_execution_ = default_full_trajectory_execution_; //TODO: set to actual param
  replanning_ = false;
  executed_fprims_.clear();
  current_belief_.clear();
  if (num_models_ > 0) current_belief_.resize(num_models_, 1.0/static_cast<double>(num_models_));
  //last_observed_pose_.header.stamp = ros::Time::now() - ros::Duration(1000.0);
  goal_state_.changed_inds.clear();
  goal_state_.changed_points.poses.clear();
  goal_state_.grasp_idx = -1;
  previous_start_state_.changed_inds.clear();
  previous_start_state_.changed_points.poses.clear();
  previous_start_state_.grasp_idx = -1;
  //local_grasp_poses_.clear();
  //grasp_idxs_.clear();
  
}

void LAORobotLTM::LearnCB(const std_msgs::Int32ConstPtr& learning_mode)
{
  if (learning_mode->data == 1)
  {
    // Trigger AR Marker tracking
    // Reset observations and edges
    observations_.clear();
    // Open the rosbag file for recording data
    if (record_bag_)
    {
      ROS_INFO("[LTM Node]: Writing point cloud data to %s", bag_file_.c_str());
      bag_.open(bag_file_, rosbag::bagmode::Write);
    }
    return;
  }

  if (learning_mode->data == 2)
  {
    ROS_INFO("[LAO Robot LTM]: In learn CB");
    // Stop AR Marker tracking, and pass observations to d_model
    if (record_bag_)
    {
       bag_.close();
    }
    ROS_INFO("[LTM Node]: Finished recording %d frames", observations_.size());
    
    if (int(observations_.size()) == 0)
    {
      ROS_INFO("[LTM Node]: No observations have been received--cannot learn parameters and initialize the model");
      return;
    }
    if (record_observations_)
    {
      SaveObservationsToFile(obs_file_.c_str());
    }

    // For now, learn the prior and visualize immediately after recording
    // learner_->AddGraspIdx(0); //Deprecated--cannot add grasp idx before initializing models
    // Dummy force vector for visualization
    /*
    vector<tf::Vector3> forces;
    learner_->PlaybackObservations(observations_, forces);
    learner_->LearnPrior(observations_);
    */
    vector<Edge> edges;
    d_model_bank_->SetPoints(last_observed_pose_);
    learner_->SetModelBank(d_model_bank_);
    learner_->ComputeEdges(observations_[0], &edges);
    ROS_INFO("LTM Node: Number of edges in model: %d", int(edges.size()));
    vector<vector<EdgeParams>> edge_params;
    /*
    edge_params.resize(1);
    for (int jj = 0; jj < edge_params.size(); ++jj)
    {
      for (int ii = 0; ii < edges.size(); ++ii)
      {
        EdgeParams e_params;
        e_params.joint = RIGID;
        e_params.normal = tf::Vector3(1.0, 1.0, 1.0);
        e_params.rad = 1.0;
        edge_params[jj].push_back(e_params);
      }
    }
    */
    vector<AbstractKinematicModel*> kinematic_models;
    learner_->PlanesToKinematicModels(rectangles_, &kinematic_models);
    learner_->GenerateModelsMinCut(d_model_bank_->GetDModelPoints(), edges, kinematic_models, &edge_params);
    num_models_ = edge_params.size();

    current_belief_.clear();
    current_belief_.resize(num_models_, 1.0/static_cast<double>(num_models_));

    d_model_bank_->InitFromObs(edges, edge_params); 
    ROS_INFO("LTM Node: Initialized model from live observation\n");
    return;
  }
}

void LAORobotLTM::GraspCB(const geometry_msgs::PoseStampedConstPtr& grasp_pose)
{
  ROS_INFO("LTM Node: Received grasp pose\n");

  // Transform goal pose to reference frame
  geometry_msgs::PoseStamped grasp_pose_ref_frame;
  //tf_listener_.waitForTransform(grasp_pose->header.frame_id, reference_frame_, ros::Time(0), ros::Duration(3.0));
  tf_listener_.transformPose(reference_frame_, ros::Time(0), *grasp_pose, grasp_pose->header.frame_id, grasp_pose_ref_frame);
  // Set the grasp pose, to transform the end effector trajectory later
  grasp_pose_ = grasp_pose_ref_frame.pose;
  // Store the local grasp pose
  const int closest_p_idx = d_model_bank_->AddGraspPoint(grasp_pose_);
  ROS_INFO("[LTM Node]: Added new grasp point, attached to %d", closest_p_idx); 
  string local_frame = boost::lexical_cast<string>(closest_p_idx);
  geometry_msgs::Pose local_grasp = d_model_bank_->TransformPose(grasp_pose_, reference_frame_, local_frame);
  grasp_attached_idxs_.push_back(closest_p_idx);
  local_grasp_poses_.push_back(local_grasp);
  // TODO: Make this general
  geometry_msgs::PoseArray points = d_model_bank_->GetDModelPoints();
  grasp_idxs_.push_back(points.poses.size() - 1);
  return;
}

void LAORobotLTM::HandleCB(const handle_detector::HandleListMsgConstPtr& handle_list)
{
  if (received_handle_)
  {
    return;
  }
  // TODO: arbitrarily picking a grasp pose for now
  const int handle_idx = 0;
  const int cylinder_idx = 0;

  if (handle_list->handles.size() == 0)
  {
    ROS_WARN("No handles in handle list");
    return;
  }
  if (handle_list->handles[handle_idx].cylinders.size() == 0)
  {
    ROS_WARN("No cylinders in handle");
    return;
  }

  ROS_INFO("Num handles:  %d", handle_list->handles.size());
  ROS_INFO("Num cylinders: %d", handle_list->handles[handle_idx].cylinders.size());

  handle_detector::CylinderArrayMsg handle = handle_list->handles[handle_idx];
  handle_detector::CylinderMsg cylinder = handle.cylinders[cylinder_idx];
  geometry_msgs::PoseStampedPtr handle_pose(new geometry_msgs::PoseStamped);
  geometry_msgs::Pose axis_pose, door_mid_pose;

  // Fill in handle pose
  handle_pose->pose = cylinder.pose;


  handle_pose->header = handle.header;
  tf_listener_.waitForTransform(handle_pose->header.frame_id, reference_frame_, ros::Time::now(), ros::Duration(3.0));
  tf_listener_.transformPose(reference_frame_, ros::Time(0), *handle_pose, handle_pose->header.frame_id, *handle_pose);


  // Fill in axis pose
  // Take the rectangle corner furthest away from handle along y
  double max_y = 0;
  int farthest_idx = -1;
  if (rectangles_.polygons.size() == 0)
  {
    ROS_WARN("No rectangles have been received");
    return;
  }
  geometry_msgs::Polygon rect = rectangles_.polygons[0];
  for (int ii = 0; ii < rect.points.size(); ++ii)
  {
    double dist = fabs(rect.points[ii].y - handle_pose->pose.position.y);
    if (dist > max_y)
    {
      max_y = dist;
     farthest_idx = ii;
    }
  }
  axis_pose.position.x = rect.points[farthest_idx].x;
  axis_pose.position.y = rect.points[farthest_idx].y;
  axis_pose.position.z = rect.points[farthest_idx].z;
  axis_pose.orientation.x = 0.0;
  axis_pose.orientation.y = 0.0;
  axis_pose.orientation.z = 0.0;
  axis_pose.orientation.w = 1.0;

  door_mid_pose.position.x = (rect.points[0].x  + rect.points[2].x)/2.0;
  door_mid_pose.position.y = (rect.points[0].y  + rect.points[2].y)/2.0;
  door_mid_pose.position.z = (rect.points[0].z  + rect.points[2].z)/2.0;
  door_mid_pose.orientation.x = 0.0;
  door_mid_pose.orientation.y = 0.0;
  door_mid_pose.orientation.z = 0.0;
  door_mid_pose.orientation.w = 1.0;

  // Learn CB
  /*
  if (observations_.size() == 0)
  {
    ROS_WARN("No AR markers have been seen yet");
    return;
  }
  */

  ROS_INFO("Computing D-model edges");


  vector<Edge> edges;
  //learner_->SetModelBank(d_model_bank_);
  geometry_msgs::Point handle_point = handle_pose->pose.position;
  geometry_msgs::Point axis_point = axis_pose.position;
  const double handle_axis_offset = 0.5; //-0.5 for counterclockwise door


  edges.push_back(make_pair(0,1)); //handle-door mid
  edges.push_back(make_pair(1,2));//door mid-wall
  ROS_INFO("LTM Node: Number of edges in model: %d", int(edges.size()));
  vector<vector<EdgeParams>> edge_params;
  edge_params.resize(2); // 2 models--push and pull
  for (int jj = 0; jj < edge_params.size(); ++jj)
  {
    EdgeParams e_params;
    e_params.joint = REVOLUTE;
    e_params.normal = tf::Vector3(1.0, 0.0, 0.0); //handle axis
    e_params.center = tf::Vector3(handle_point.x, handle_point.y - handle_axis_offset, handle_point.z);
    e_params.rad = 1.0;
    edge_params[jj].push_back(e_params);
    e_params.joint = REVOLUTE;
    e_params.normal = tf::Vector3(0.0, 0.0, 1.0);//door axis
    e_params.center = tf::Vector3(axis_point.x, axis_point.y, axis_point.z);
    e_params.rad = 1.0;
    edge_params[jj].push_back(e_params);
  }
  num_models_ = edge_params.size();

  current_belief_.clear();
  current_belief_.resize(num_models_, 1.0/static_cast<double>(num_models_));
  current_belief_[0] = 0.99;
  current_belief_[1] = 0.01;


  // Set the grasp location to be same as handle pose
  // Hack to align gripper -- roll=pi/2,pitch=0,yaw=0
  tf::Quaternion quat(0.0, M_PI/2, 0.0);
  handle_pose->pose.orientation.x = quat.x();
  handle_pose->pose.orientation.y = quat.y();
  handle_pose->pose.orientation.z = quat.z();
  handle_pose->pose.orientation.w = quat.w();
  handle_pose->header.frame_id = reference_frame_;


  // Gripper offset
  handle_pose->pose.position.x = handle_pose->pose.position.x - kGripperOffsetX;
  handle_pose->pose.position.y = handle_pose->pose.position.y - kGripperOffsetY;
  handle_pose->pose.position.z = handle_pose->pose.position.z - kGripperOffsetZ;

  geometry_msgs::PoseArray door_poses;
  door_poses.poses.push_back(handle_pose->pose);
  door_poses.poses.push_back(door_mid_pose);
  door_poses.poses.push_back(axis_pose);
  d_model_bank_->SetPoints(door_poses);
  d_model_bank_->InitFromObs(edges, edge_params); 

  ROS_INFO("LTM Node: Initialized model from live observation\n");

  GraspCB(handle_pose);
  received_handle_ = true;
  return;
}

void LAORobotLTM::GoalCB(const geometry_msgs::PoseStampedConstPtr& goal_pose)
{
  // Experiments
  if (run_experiments_)
  {
    if (experiment_num_ != 0) fclose(planner_stats_file_);
    if (experiment_num_ != 0) fclose(belief_stats_file_);
    experiment_num_++;
    string planner_stats_name = kPlannerStatsBase + boost::lexical_cast<string>(experiment_num_) + ".csv";
    string belief_stats_name = kBeliefStatsBase + boost::lexical_cast<string>(experiment_num_) + ".csv";
    planner_stats_file_ = fopen(planner_stats_name.c_str(), "w");
    belief_stats_file_ = fopen(belief_stats_name.c_str(), "w");
  }

  ROS_INFO("LTM Node: Received goal %d", num_goals_received_);
  // Transform goal pose to reference frame
  geometry_msgs::PoseStamped goal_pose_ref_frame;
  tf_listener_.waitForTransform(goal_pose->header.frame_id, reference_frame_, ros::Time::now(), ros::Duration(3.0));
  tf_listener_.transformPose(reference_frame_, ros::Time(0), *goal_pose, goal_pose->header.frame_id, goal_pose_ref_frame);

  // Set goal
  if (grasp_idxs_.size() == 0 || num_goals_received_ >= grasp_idxs_.size())
  {
    ROS_INFO("LTM Node: Grasp points have not been set, or invalid goal num. Ignoring goal.\n");
    return;
  }

  goal_state_.changed_inds.push_back(grasp_idxs_[num_goals_received_]);
  goal_state_.changed_points.poses.push_back(goal_pose_ref_frame.pose);
  ROS_INFO("LTM Node: New goal received: %f %f %f\n",
      goal_pose_ref_frame.pose.position.x, goal_pose_ref_frame.pose.position.y,
      goal_pose_ref_frame.pose.position.z);

  // Wait until number of goal states is the same as number of grasp points
  if (num_goals_received_ != grasp_idxs_.size() - 1)
  {
    num_goals_received_++;
    return;
  }

  UpdateStartState();

  //TODO: Not setting belief goal state for now, since it depends on internal goal state
  //Note: Make sure to recompute goal state changed points when the start state changes
  d_model_bank_->SetInternalGoalState(goal_state_);

  PlanAndExecute();

  return;
}

void LAORobotLTM::PlanAndExecute()
{
  // Plan
  vector<int> state_ids, fprim_ids;
  planner_->SetStart(d_model_bank_->GetStartStateID());
  if (!planner_->Plan(&state_ids, &fprim_ids))
  {
    printf("[LTM Node]: Failed to plan\n");
    return;
  }
  // Print planner stats
  PlannerStats planner_stats = planner_->GetPlannerStats();
  ROS_INFO("Planner Stats:\nNum Expansions: %d, Planning Time: %f, Start State Value: %d\n",
      planner_stats.expansions, planner_stats.time, planner_stats.cost);
  if (run_experiments_)
  {
    fprintf(planner_stats_file_, "%d %f %d\n", planner_stats.expansions, planner_stats.time, planner_stats.cost);
  }

  // Convert state ids to end-effector trajectory
  geometry_msgs::PoseArray traj;
  GetExecutionTraj(state_ids, fprim_ids, &traj);

  // Publish end-effector trajectory
  // plan_pub_.publish(traj);
}

void LAORobotLTM::UpdateStartState()
{
  // Don't update the environment's start unless the grasp points have been set
  if (grasp_idxs_.size() == 0)
  {
    return;
  }
  start_state_.grasp_idx = grasp_idxs_[0];
  //assert(observations_.size() != 0);

  // Update the end-effector location if replanning, else set start to original start
  if (replanning_)
  {
    geometry_msgs::PoseArray appended_poses;
    geometry_msgs::PoseStamped end_eff_pose;
    r_arm_->getCurrentEndEffectorPose(end_eff_pose, reference_frame_);

    //Door-specific hack
    geometry_msgs::PoseArray poses;
    geometry_msgs::Pose axis_pose, door_mid_pose;
    //end_eff_pose.pose.position.x = end_eff_pose.pose.position.x + kGripperOffsetX;
    //end_eff_pose.pose.position.y = end_eff_pose.pose.position.y + kGripperOffsetY;
    //end_eff_pose.pose.position.z = end_eff_pose.pose.position.z + kGripperOffsetZ;

    // Fill in axis pose
    // Take the rectangle corner furthest away from handle along y
    double max_y = 0;
    int farthest_idx = -1;
    if (rectangles_.polygons.size() == 0)
    {
      ROS_WARN("No rectangles have been received");
      return;
    }
    geometry_msgs::Polygon rect = rectangles_.polygons[0];
    for (int ii = 0; ii < rect.points.size(); ++ii)
    {
      double dist = fabs(rect.points[ii].y - end_eff_pose.pose.position.y);
      if (dist > max_y)
      {
        max_y = dist;
        farthest_idx = ii;
      }
    }
    axis_pose.position.x = rect.points[farthest_idx].x;
    axis_pose.position.y = rect.points[farthest_idx].y;
    axis_pose.position.z = rect.points[farthest_idx].z;
    axis_pose.orientation.x = 0.0;
    axis_pose.orientation.y = 0.0;
    axis_pose.orientation.z = 0.0;
    axis_pose.orientation.w = 1.0;

    door_mid_pose.position.x = (rect.points[0].x  + rect.points[2].x)/2.0;
    door_mid_pose.position.y = (rect.points[0].y  + rect.points[2].y)/2.0;
    door_mid_pose.position.z = (rect.points[0].z  + rect.points[2].z)/2.0;
    door_mid_pose.orientation.x = 0.0;
    door_mid_pose.orientation.y = 0.0;
    door_mid_pose.orientation.z = 0.0;
    door_mid_pose.orientation.w = 1.0;

    poses.poses.push_back(end_eff_pose.pose);
    poses.poses.push_back(door_mid_pose);
    poses.poses.push_back(axis_pose);
    ObservationsToModelPoses(poses, &appended_poses);
    //ObservationsToModelPoses(last_observed_pose_, &appended_poses);

    d_model_bank_->GetChangedStateFromWorldPoses(appended_poses, &start_state_.changed_inds, &start_state_.changed_points);


    //Hack
    start_state_.changed_points.poses.clear();
    start_state_.changed_inds.clear();
    start_state_.changed_inds.push_back(0);
    start_state_.changed_inds.push_back(grasp_idxs_[0]);
    start_state_.changed_points.poses.push_back(end_eff_pose.pose);
    start_state_.changed_points.poses.push_back(end_eff_pose.pose);

    ROS_INFO("[LAO Robot LTM]: %d points have changed in the start state", start_state_.changed_inds.size());
    geometry_msgs::Pose p = appended_poses.poses[grasp_idxs_[0]];
    ROS_WARN("[LAO Robot LTM]: New start pose: %f %f %f", p.position.x, p.position.y, p.position.z);
    ROS_WARN("[LAO Robot LTM]: End-eff pose: %f %f %f", end_eff_pose.pose.position.x, end_eff_pose.pose.position.y, end_eff_pose.pose.position.z);
  }
  else
  {
    start_state_.changed_inds.clear();
    start_state_.changed_points.poses.clear();
  }


  d_model_bank_->SetInternalStartState(start_state_);

  BeliefState_t start_belief_state;
  start_belief_state.internal_state_id = d_model_bank_->GetInternalStartStateID();
  // Set model belief. TODO: Use DModelLearner to update model probabilities based on observed data
  GetStartBelief(&start_belief_state.belief);

  // Debug
  std::stringstream ss;
  for (int ii = 0; ii < num_models_; ++ii)
  {
    ss << start_belief_state.belief[ii] << " ";
  }
  string s = ss.str();
  ROS_WARN("New start state belief: %s", s.c_str());
  if (run_experiments_)
  {
    fprintf(belief_stats_file_, "%s\n", s.c_str());
  }

  //TODO: remove this
  //start_belief_state.belief.clear();
  //start_belief_state.belief.resize(num_models_, 1.0/static_cast<double>(num_models_));

  d_model_bank_->SetStartState(start_belief_state);
  previous_start_state_ = start_state_;
  current_belief_ = start_belief_state.belief;
  return;
}

void LAORobotLTM::GetStartBelief(vector<double>* belief)
{
  // Make sure start_state_ has been updated
  belief->clear();
  //ROS_INFO("Number of executed fprims: %d", executed_fprims_.size());
  //ROS_INFO("[LAO Robot LTM]: %d changed points in the start state", start_state_.changed_inds.size());
  //ROS_INFO("[LAO Robot LTM]: %d changed points in the prev start state", previous_start_state_.changed_inds.size());

  // Execute open-loop plan with current belief if we haven't seen the markers recently
  ros::Duration duration = ros::Time::now() - last_observed_pose_.header.stamp;
  //doorhack
  //const bool marker_recently_observed = (duration < ros::Duration(2.0));
  bool marker_recently_observed = true;
  if (executed_fprims_.size() == 0 || !marker_recently_observed)
  {
    *belief = current_belief_;
    // If the start state did not change, I am assuming that we dint see the AR markers.
    // So I ll run the full trajectory instead
    if (executed_fprims_.size() != 0)
    {
      full_trajectory_execution_ = true;
    }
  }
  else
  {
    vector<double> obs_probs;
    d_model_bank_->GetObservationProbabilities(previous_start_state_, start_state_, executed_fprims_, &obs_probs);
    double total_weight = 0;
    *belief = current_belief_;
    for (int ii = 0; ii < num_models_; ++ii)
    {
      (*belief)[ii] *= obs_probs[ii];
      ROS_INFO("Prob for %d: %f", ii, obs_probs[ii]);
      total_weight += (*belief)[ii];
    }
    if (total_weight < kFPTolerance)
    {
      ROS_ERROR("[LAO Robot LTM]: Total weight is 0, returning uniform belief");
      belief->clear();
      belief->resize(num_models_, 1.0/static_cast<double>(num_models_));
      return;
    }
    for (int ii = 0; ii < num_models_; ++ii)
    {
      (*belief)[ii] /= total_weight;
    }
  }
  return;
}

void LAORobotLTM::GetExecutionTraj(const vector<int>& all_state_ids, const vector<int>& all_fprim_ids, geometry_msgs::PoseArray* traj)
{
  std_msgs::Int32Ptr exec_mode_ptr(new std_msgs::Int32);
  exec_mode_ptr->data = 1;
  if (all_state_ids.size() < 2 || all_fprim_ids.size() == 0)
  {
    ROS_WARN("[LAO Robot LTM]: Trajectory has less than 2 waypoints. Assuming we are at the goal");
    traj->poses.clear();
    TrajExecCB(exec_mode_ptr);
    return;
  }
  // Truncate trajectory
  int num_points;
  if (full_trajectory_execution_)
  {
    ROS_INFO("[LAO Robot LTM]: Full trajectory execution");
    num_points = all_state_ids.size();
  }
  else 
  {
    num_points = num_partial_traj_waypoints_;
  }
  vector<int> state_ids, fprim_ids;
  assert(all_state_ids.size() == all_fprim_ids.size() + 1);
  if (num_points > all_state_ids.size())
  {
    state_ids = all_state_ids;
    fprim_ids = all_fprim_ids;
  }
  else
  {
    state_ids.assign(all_state_ids.begin(), all_state_ids.begin() + num_points);
    fprim_ids.assign(all_fprim_ids.begin(), all_fprim_ids.begin() + num_points - 1);
  }

  bool approach = false;
  if (executed_fprims_.size() == 0)
  {
    approach = true;
  }

  // Get forces from fprim_ids
  vector<tf::Vector3> forces;
  vector<int> grasp_points;
  forces.clear();
  grasp_points.clear();
  d_model_bank_->ConvertForcePrimIDsToForcePrims(fprim_ids, &forces, &grasp_points);

  /*
  ROS_INFO("Force Sequence:");
  for (size_t ii = 0; ii < forces.size(); ++ii)
  {
    ROS_INFO("%f %f %f", forces[ii].x(), forces[ii].y(), forces[ii].z());
  }
  */

  d_model_bank_->GetEndEffectorTrajFromStateIDs(state_ids, traj);

  // Simulate plan 
  // TODO: Hardcoded model id for visualization--doesn't matter since the actual model is not used for
  // visualization
  SimulatePlan(0, *traj, state_ids, forces, grasp_points);

  //Print end effector trajectory
  printf("End-Effector Trajectory:\n");
  for (size_t ii = 0; ii < (*traj).poses.size(); ++ii)
  {
    ROS_INFO("%f %f %f %f %f %f %f", (*traj).poses[ii].position.x, (*traj).poses[ii].position.y, (*traj).poses[ii].position.z,
        (*traj).poses[ii].orientation.x, (*traj).poses[ii].orientation.y, (*traj).poses[ii].orientation.z, (*traj).poses[ii].orientation.w);
  }


  if (!simulation_)
  {
  if ((*traj).poses.size() == 0)
  {
    ROS_WARN("Execution trajectory has no waypoints");
    return;
  }

  ROS_INFO("Executing trajectory");
  std::vector<double> current_angles;
  r_arm_->getCurrentArmConfiguration(current_angles);
  if (approach)
  {
    ROS_INFO("Beginning approach...");
    geometry_msgs::PoseStamped first_pose;
    first_pose.header = (*traj).header;
    first_pose.pose = (*traj).poses[0];
    r_arm_->openGripper();
    sleep(2.0); // wait for gripper to open
    //r_arm_->sendArmToPose(first_pose, current_angles, 4.0); 
    r_arm_->sendArmToPose(first_pose, current_angles);
    r_arm_->closeGripper();
    grasped_ = true;
    sleep(2.0); // wait for gripper to close
  }
  //std::vector<double> move_times;
  //move_times.resize((*traj).poses.size(), 3.0);

  geometry_msgs::PoseArray new_traj;
  new_traj.header = traj->header;
  new_traj.poses.assign(traj->poses.begin()+1,traj->poses.end());  
  r_arm_->sendArmToPoses(new_traj, current_angles);

  // Skip the first waypoint
  //Pose by pose
  /*
     for (int ii = 1; ii < (*traj).poses.size(); ++ii)
     {
     r_arm_->getCurrentArmConfiguration(current_angles);
     geometry_msgs::PoseStamped pose;
     pose.header = (*traj).header;
     pose.pose = (*traj).poses[ii];
     r_arm_->closeGripper();
  //if(!r_arm_->sendArmToPose(pose, current_angles, 1.0)) 
  if(!r_arm_->sendArmToPose(pose, current_angles)) 
  {
  ROS_ERROR("Could not get IK");
  }
  }
  */
  ROS_INFO("Finished executing trajectory");

  // Store the fprims, so that we can compute observation probabilities later
  executed_fprims_ = fprim_ids;
  TrajExecCB(exec_mode_ptr);
  }
  else
  {
    ResetStateMachine();
    ROS_INFO("[LTM Node]: Simulation complete");
  }
  return;
}

void LAORobotLTM::TrajExecCB(const std_msgs::Int32ConstPtr& exec_mode_ptr)
{
  sleep(2.0); //make sure AR marker poses have updated
  int exec_mode = static_cast<int>(exec_mode_ptr->data);
  State_t temp_goal_state;
  temp_goal_state.grasp_idx = grasp_idxs_[0]; //TODO: this will fail for multi-grasp points
  geometry_msgs::PoseArray appended_poses;


  //Door-specific hack
  geometry_msgs::PoseArray poses;
  geometry_msgs::Pose axis_pose, door_mid_pose;

  // Fill in axis pose
  // Take the rectangle corner furthest away from handle along y
  geometry_msgs::PoseStamped end_eff_pose;
  r_arm_->getCurrentEndEffectorPose(end_eff_pose, reference_frame_);
  //end_eff_pose.pose.position.x = end_eff_pose.pose.position.x + kGripperOffsetX;
  //end_eff_pose.pose.position.y = end_eff_pose.pose.position.y + kGripperOffsetY;
  //end_eff_pose.pose.position.z = end_eff_pose.pose.position.z + kGripperOffsetZ;
  double max_y = 0;
  int farthest_idx = -1;
  if (rectangles_.polygons.size() == 0)
  {
    ROS_WARN("No rectangles have been received");
    return;
  }
  geometry_msgs::Polygon rect = rectangles_.polygons[0];
  for (int ii = 0; ii < rect.points.size(); ++ii)
  {
    double dist = fabs(rect.points[ii].y - end_eff_pose.pose.position.y);
    if (dist > max_y)
    {
      max_y = dist;
      farthest_idx = ii;
    }
  }
  axis_pose.position.x = rect.points[farthest_idx].x;
  axis_pose.position.y = rect.points[farthest_idx].y;
  axis_pose.position.z = rect.points[farthest_idx].z;
  axis_pose.orientation.x = 0.0;
  axis_pose.orientation.y = 0.0;
  axis_pose.orientation.z = 0.0;
  axis_pose.orientation.w = 1.0;

  door_mid_pose.position.x = (rect.points[0].x  + rect.points[2].x)/2.0;
  door_mid_pose.position.y = (rect.points[0].y  + rect.points[2].y)/2.0;
  door_mid_pose.position.z = (rect.points[0].z  + rect.points[2].z)/2.0;
  door_mid_pose.orientation.x = 0.0;
  door_mid_pose.orientation.y = 0.0;
  door_mid_pose.orientation.z = 0.0;
  door_mid_pose.orientation.w = 1.0;

  poses.poses.push_back(end_eff_pose.pose);
  poses.poses.push_back(door_mid_pose);
  poses.poses.push_back(axis_pose);
  ObservationsToModelPoses(poses, &appended_poses);
  //ObservationsToModelPoses(last_observed_pose_, &appended_poses);





  d_model_bank_->GetChangedStateFromWorldPoses(appended_poses, &temp_goal_state.changed_inds, &temp_goal_state.changed_points);
  if (full_trajectory_execution_ || d_model_bank_->IsInternalGoalState(temp_goal_state)) 
  {
    // Success--reset goal state to be ready for next goal request
    r_arm_->openGripper();
    sleep(2.0); // wait for gripper to open
    r_arm_->backupGripper(0.2);
    r_arm_->sendArmToConfiguration(kRightArmHomeConfig, 3.0);
    ResetStateMachine();
    ROS_INFO("[LTM Node]: Task completed successfully, ready for new goal request");
  }
  else
  {
    // Replan and execute
    replanning_ = true;
    ROS_INFO("[LTM Node]: Partial trajectory executed, replanning to goal");
    UpdateStartState();
    PlanAndExecute();
  }
  return;
}

void LAORobotLTM::PerceptionCB(const ltm_msgs::PolygonArrayStamped& rectangles)
{
  rectangles_ = rectangles;
  for (int ii = 0; ii < rectangles.polygons.size(); ++ii)
  {
    string id = "polygon" + boost::lexical_cast<string>(ii);
    viz_->VisualizePolygon(rectangles.polygons[ii], id); 
  }
  return;
}

void LAORobotLTM::KinectCB(const sensor_msgs::PointCloud2& point_cloud)
{
  // Do not write to bag file if not in tracking mode
  if (!record_bag_)
  {
    return;
  }
  // Tranform to reference frame (not recording tf currently)
  sensor_msgs::PointCloud2 ref_point_cloud;
  pcl_ros::transformPointCloud(reference_frame_, point_cloud, ref_point_cloud, tf_listener_);
  bag_.write("kinect_point_cloud", point_cloud.header.stamp, ref_point_cloud);
}

void LAORobotLTM::ARMarkersCB(const ar_track_alvar_msgs::AlvarMarkersConstPtr& ar_markers)
{
  // Ensure we have same number of markers in each frame
  const int num_markers = int(ar_markers->markers.size());

  // Skip if we did not see any markers
  if (num_markers == 0)
  {
    return;
  }

  // This is a hack (assuming fixed number of markers)
  const int num_tracked_markers = 3;
  if (num_markers != num_tracked_markers)
  {
    return;
  }

  // We will assume that the first frame contains all the markers we want to track
  if (int(observations_.size()) != 0)
  {
    if (num_markers != int(observations_[0].poses.size()))
    // if (num_markers != num_tracked_markers) // Hack
    {
      ROS_WARN("LTM Node: Number of markers (%d) in current frame does not match number being tracked (%d). Dropping observation",
          num_markers, int(observations_[0].poses.size()));
      return;
    }
  }

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
    temp_pose_array.poses.push_back(grasp_pose_ref_frame.pose);
  }

  if (enforce_spatial_association_ && observations_.size() != 0)
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
      search_point.x = last_observed_pose_.poses[ii].position.x;
      search_point.y = last_observed_pose_.poses[ii].position.y;
      search_point.z = last_observed_pose_.poses[ii].position.z;
      kdtree.nearestKSearch(search_point, kKNNSearchK, idxs, sqr_distances);
      pose_array.poses[ii] = temp_pose_array.poses[idxs[0]];
    }
  }
  else
  {
    pose_array = temp_pose_array;
  }

  // Initialize the d_model only once
  if (observations_.size() == 0)
  {
    observations_.push_back(pose_array);
  }
  // Record the observations if needed
  if (record_observations_)
  {
    observations_.push_back(pose_array);
  }
  last_observed_pose_ = pose_array;
  last_observed_pose_.header.frame_id = reference_frame_;
  last_observed_pose_.header.stamp = ros::Time::now();

  return;
}

/*
void LAORobotLTM::SetModelBankFromFile(vector<string> model_files)
{
    d_model_bank_->InitFromFile(model_files, model_offsets_x_, model_offsets_y_, model_offsets_z_); 
}
*/
void LAORobotLTM::SaveObservationsToFile(const char* obs_file)
{

  FILE* f_obs = fopen(obs_file, "w+");
  if (f_obs == NULL)
  {
    ROS_ERROR("[LTM Node]: Could not open observations file");
    return;
  }
  const int num_frames = int(observations_.size());
  assert(num_frames > 0);
  const int num_points = int(observations_[0].poses.size());
  fprintf(f_obs, "points: %d\n", num_points);
  fprintf(f_obs, "frames: %d\n", num_frames);
  for (int ii = 0; ii < num_frames; ++ii)
  {
    for (int jj = 0; jj < num_points; ++jj)
    {
      geometry_msgs::Pose p = observations_[ii].poses[jj];
      fprintf(f_obs, "%f %f %f %f %f %f %f\n", p.position.x, p.position.y, p.position.z,
          p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w);
    }
    fprintf(f_obs, "\n");
  }
  fclose(f_obs);
  ROS_INFO("[LTM Node]: Saved observations to file %s", obs_file);
}

void LAORobotLTM::SimulatePlan(int model_id, const geometry_msgs::PoseArray& plan, const vector<int>& state_ids, const vector<tf::Vector3>& forces, const vector<int>& grasp_points)
{
  // TODO: Check that plan and forces are of equal length
  printf("Num States: %d, Num Forces: %d\n", state_ids.size(), forces.size());
  // TODO: This needs to reason about change of grsp point
  
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
  const double torso_pos = 0.3; //0.3
  const double hue = 0.5;
  const string ns = "ltm_plan";
  const bool use_pr2_mesh = true;
  
  const double standoff = 0.0; //0.15

  geometry_msgs::Pose last_grasp_pose, current_grasp_pose;
  for (size_t ii = 0; ii < state_ids.size(); ++ii)
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
    d_model_bank_->VisualizeState(model_id, state_ids[ii]);
    
    //seed.swap(r_angles);
    usleep(100000);
  }

  // Visualize the initial state after running through simulation
  sleep(2);
  d_model_bank_->VisualizeState(model_id, state_ids[0]);
  /*
  pviz_.visualizeRobot(initial_r_angles,
      l_angles,
      base_pos,  // 0: x, 1: y,  2: theta,  3: head_pan,  4: head_tilt,  5: laser_tilt
      torso_pos, 
      hue,
      ns, 1, use_pr2_mesh);
      */
  return;
}

bool LAORobotLTM::GetRightIK(const vector<double>& ik_pose, const vector<double>& seed, vector<double>* angles){

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

void LAORobotLTM::ObservationsToModelPoses(const geometry_msgs::PoseArray& observations, geometry_msgs::PoseArray* appended_poses)
{
  appended_poses->poses = observations.poses;
  // Must append the grasp poses to AR marker poses--need the TF's for the changed points first
  //TODO: smarter way to do this
  if (!grasped_)
  {
    tf::Transform transform;
    for (size_t ii = 0; ii < last_observed_pose_.poses.size(); ++ii)
    {
      static tf::TransformBroadcaster tf_br_;
      geometry_msgs::Pose p = last_observed_pose_.poses[ii];
      transform.setOrigin(tf::Vector3(p.position.x, p.position.y, p.position.z) );
      transform.setRotation(tf::Quaternion(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w));
      string child_frame_id = boost::lexical_cast<string>(ii); //assuming the idxs are same until the grasp idxx
      tf_br_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), reference_frame_.c_str(), child_frame_id.c_str()));
    }
    for (int ii = 0; ii < local_grasp_poses_.size(); ++ii)
    { 
      string local_frame = boost::lexical_cast<string>(grasp_attached_idxs_[ii]); //assuming the idxs are same until the grasp idxx
      appended_poses->poses.push_back(d_model_bank_->TransformPose(local_grasp_poses_[ii], local_frame, reference_frame_));
    }
  }
  else
  {
    // TODO: I am assuming there is only one grasp point for now and updating it live
    geometry_msgs::PoseStamped end_eff_pose;
    r_arm_->getCurrentEndEffectorPose(end_eff_pose, reference_frame_);
    appended_poses->poses.push_back(end_eff_pose.pose);
  }

}
