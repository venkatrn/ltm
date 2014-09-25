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

using namespace std;

LAORobotLTM::LAORobotLTM() : num_goals_received_(0),
                             ar_marker_tracking_(false)
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

  grasp_idxs_.clear();

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

  // Initialize IK clients
  query_client_ = nh_.serviceClient<kinematics_msgs::GetKinematicSolverInfo>("pr2_right_arm_kinematics/get_ik_solver_info");
  ik_client_ = nh_.serviceClient<kinematics_msgs::GetPositionIK>("pr2_right_arm_kinematics/get_ik");
}

LAORobotLTM::~LAORobotLTM() 
{
  delete d_model_bank_;
  delete planner_;
  delete learner_;
}

void LAORobotLTM::LearnCB(const std_msgs::Int32ConstPtr& learning_mode)
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
    // Open the rosbag file for recording data
    ROS_INFO("[LTM Node]: Writing point cloud data to %s", bag_file_.c_str());
    bag_.open(bag_file_, rosbag::bagmode::Write);

    return;
  }

  if (learning_mode->data == 2)
  {
    // Stop AR Marker tracking, and pass observations to d_model
    ar_marker_tracking_ = false;
    bag_.close();
    ROS_INFO("[LTM Node]: Finished recording %d frames", observations_.size());
    
    if (int(observations_.size()) == 0)
    {
      ROS_INFO("[LTM Node]: Learning stopped before any observations have been received. No model will be learnt");
      return;
    }
    SaveObservationsToFile(obs_file_.c_str());

    // For now, learn the prior and visualize immediately after recording
    // learner_->AddGraspIdx(0); //Deprecated--cannot add grasp idx before initializing models
    // Dummy force vector for visualization
    /*
    vector<tf::Vector3> forces;
    learner_->PlaybackObservations(observations_, forces);
    learner_->LearnPrior(observations_);
    */
    vector<Edge> edges;
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
    learner_->GenerateModels(d_model_bank_->GetDModelPoints(), edges, kinematic_models, &edge_params);
    num_models_ = edge_params.size();
    d_model_bank_->InitFromObs(edges, edge_params); 
    ROS_INFO("LTM Node: Initialized model from live observation\n");

    return;

    // TODO: Not using the old learning method currently
    // d_model_->LearnDModelParameters(observations_, edges_);
  }
}

void LAORobotLTM::GraspCB(const geometry_msgs::PoseStampedConstPtr& grasp_pose)
{
  ROS_INFO("LTM Node: Received grasp pose\n");

  // Transform goal pose to reference frame
  geometry_msgs::PoseStamped grasp_pose_ref_frame;
  tf_listener_.waitForTransform(grasp_pose->header.frame_id, reference_frame_, ros::Time::now(), ros::Duration(3.0));
  tf_listener_.transformPose(reference_frame_, *grasp_pose, grasp_pose_ref_frame);
  // Set the grasp pose, to transform the end effector trajectory later
  grasp_pose_ = grasp_pose_ref_frame.pose;
  d_model_bank_->AddGraspPoint(grasp_pose_);
  ROS_INFO("[LTM Node]: Added new grasp point"); 
  // TODO: Make this general
  geometry_msgs::PoseArray points = d_model_bank_->GetDModelPoints();
  grasp_idxs_.push_back(points.poses.size() - 1);
  return;
}

void LAORobotLTM::GoalCB(const geometry_msgs::PoseStampedConstPtr& goal_pose)
{
  ROS_INFO("LTM Node: Received goal %d", num_goals_received_);

  // Transform goal pose to reference frame
  geometry_msgs::PoseStamped goal_pose_ref_frame;
  tf_listener_.waitForTransform(goal_pose->header.frame_id, reference_frame_, ros::Time::now(), ros::Duration(3.0));
  tf_listener_.transformPose(reference_frame_, *goal_pose, goal_pose_ref_frame);

  // Set goal
  if (grasp_idxs_.size() == 0 || num_goals_received_ >= grasp_idxs_.size())
  {
    ROS_INFO("LTM Node: Grasp points have not been set, or invalid goal num\n");
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

  // Convert state ids to end-effector trajectory
  geometry_msgs::PoseArray traj;
  GetExecutionTraj(state_ids, fprim_ids, &traj);

  // Publish end-effector trajectory
  plan_pub_.publish(traj);
}

void LAORobotLTM::UpdateStartState()
{
  // Don't update the environment's start unless the grasp points have been set
  if (grasp_idxs_.size() == 0)
  {
    return;
  }
  start_state_.grasp_idx = grasp_idxs_[0];
  d_model_bank_->SetInternalStartState(start_state_);

  BeliefState_t start_belief_state;
  start_belief_state.internal_state_id = d_model_bank_->GetInternalStartStateID();
  // Set model belief. TODO: Use DModelLearner to update model probabilities based on observed data
  GetStartBelief(&start_belief_state.belief);
  d_model_bank_->SetStartState(start_belief_state);
  return;
}

void LAORobotLTM::GetStartBelief(vector<double>* belief)
{
  belief->clear();
  belief->resize(num_models_, 1.0/static_cast<double>(num_models_));
  return;
}

void LAORobotLTM::GetExecutionTraj(const vector<int>& state_ids, const vector<int>& fprim_ids, geometry_msgs::PoseArray* traj)
{
  // Get forces from fprim_ids
  vector<tf::Vector3> forces;
  vector<int> grasp_points;
  forces.clear();
  grasp_points.clear();
  d_model_bank_->ConvertForcePrimIDsToForcePrims(fprim_ids, &forces, &grasp_points);

  /*
     printf("Force Sequence:\n");
     for (size_t ii = 0; ii < forces.size(); ++ii)
     {
     printf("%f %f %f\n", forces[ii].x(), forces[ii].y(), forces[ii].z());
     }
     printf("\n");
     */

  d_model_bank_->GetEndEffectorTrajFromStateIDs(state_ids, traj);

  // Simulate plan 
  // d_model_->SimulatePlan(fprim_ids);
  // TODO: Hardcoded model id for visualization--doesn't matter since the actual model is not used for
  // visualization
  SimulatePlan(0, *traj, state_ids, forces, grasp_points);
  // TODO: Currently faking execution
  // std_msgs::Int32Ptr exec_mode_ptr(new std_msgs::Int32);
  // exec_mode_ptr->data = 1;
  // TrajExecCB(exec_mode_ptr);

  //Print end effector trajectory
  printf("End-Effector Trajectory:\n");
  for (size_t ii = 0; ii < (*traj).poses.size(); ++ii)
  {
    ROS_INFO("%f %f %f %f %f %f %f", (*traj).poses[ii].position.x, (*traj).poses[ii].position.y, (*traj).poses[ii].position.z,
        (*traj).poses[ii].orientation.x, (*traj).poses[ii].orientation.y, (*traj).poses[ii].orientation.z, (*traj).poses[ii].orientation.w);
  }

  return; 
}

void LAORobotLTM::TrajExecCB(const std_msgs::Int32ConstPtr& exec_mode_ptr)
{
  int exec_mode = static_cast<int>(exec_mode_ptr->data);
  // TODO: Check if actual goal has been reached
  if (exec_mode == 1)
  {
    // Success--reset goal state to be ready for next goal request
    ROS_INFO("[LTM Node]: Task completed successfully, ready for new goal request");
    num_goals_received_ = 0;
    goal_state_.changed_inds.clear();
    goal_state_.changed_points.poses.clear();
  }
  else
  {
    // Replan and execute
    ROS_INFO("[LTM Node]: Partial trajectory executed, replanning to goal");
    UpdateStartState();

    //TODO: Not setting belief goal state for now, since it depends on internal goal state
    //Note: Make sure to recompute goal state changed points when the start state changes
    d_model_bank_->SetInternalGoalState(goal_state_);
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
  if (!ar_marker_tracking_)
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
  // Do not store poses if not in learning mode
  if (!ar_marker_tracking_)
  {
    return;
  }
  
  // Ensure we have same number of markers in each frame
  const int num_markers = int(ar_markers->markers.size());

  // Skip if we did not see any markers
  if (num_markers == 0)
  {
    return;
  }

  // This is a hack (assuming fixed number of markers)
  /*
  const int num_tracked_markers = 3;
  if (num_markers != num_tracked_markers)
  {
    return;
  }
  */

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

  d_model_bank_->SetPoints(pose_array);

  observations_.push_back(pose_array);

  // Grab the kinect data and other stuff needed
  //sleep(1);
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

