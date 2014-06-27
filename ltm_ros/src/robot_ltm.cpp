/**
 * @file robot_ltm.cpp
 * @author Venkatraman Narayanan
 * Carnegie Mellon University, 2014
 */

#include <ltm_ros/robot_ltm.h>
#include <ltm/d_model.h>
#include <ltm/d_model_utils.h>

#include <pcl/kdtree/kdtree_flann.h>

#include <algorithm>

using namespace std;

RobotLTM::RobotLTM() : ar_marker_tracking_(false)
{
  ros::NodeHandle private_nh("~");
  private_nh.param("use_model_file", use_model_file_, false);
  private_nh.param("enforce_spatial_association", enforce_spatial_association_, true);
  private_nh.param("model_file", model_file_, string(""));
  private_nh.param("fprims_file", fprims_file_, string(""));
  private_nh.param("obs_file", obs_file_, string("observations.txt"));
  private_nh.param("reference_frame", reference_frame_, string("/map"));
  private_nh.param("sim_time_step", sim_time_step_, 0.1);
  private_nh.param("model_offset_x", model_offset_x_, 0.0);
  private_nh.param("model_offset_y", model_offset_y_, 0.0);
  private_nh.param("model_offset_z", model_offset_z_, 0.0);

  grasp_idxs_.clear();

  d_model_ = new DModel(reference_frame_);
  planner_ = new DModelPlanner;
  learner_ = new DModelLearner(reference_frame_);

  // Initialize force primitives
  d_model_->InitForcePrimsFromFile(fprims_file_.c_str());

  // Setup publsihers
  plan_pub_ = nh_.advertise<geometry_msgs::PoseArray>("ltm_plan", 1);
  mannequin_pub_ = nh_.advertise<sensor_msgs::JointState>("/robot2/mannequin_joints", 1000, true);

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

  /*
  for (int ii = 0; ii < 1000; ++ii) {
  sensor_msgs::JointState joints_msg;
  joints_msg.header.stamp = ros::Time::now();
  joints_msg.name.push_back(string("LShoulderYaw"));
  joints_msg.position.push_back(0.0); //1.07
  joints_msg.velocity.push_back(0.0);
  joints_msg.effort.push_back(0.0);
  mannequin_pub_.publish(joints_msg);
  usleep(10000);
  }
  */

}

RobotLTM::~RobotLTM() 
{
  delete d_model_;
  delete planner_;
  delete learner_;
}

void RobotLTM::ModelCB(const ltm_msgs::DModel& d_model)
{
  // Update model (learn parameters)
  // Set start
  State_t start_state;
  start_state.grasp_idx = 0;
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
    ROS_INFO("[LTM Node]: Finished recording %d frames", observations_.size());
    
    if (int(observations_.size()) == 0)
    {
      ROS_INFO("[LTM Node]: Learning stopped before any observations have been received. No model will be learnt");
      return;
    }
    SaveObservationsToFile(obs_file_.c_str());

    ComputeEdges(observations_[0]);
    ROS_INFO("LTM Node: Number of edges in model: %d", int(edges_.size()));
    // TODO: Not using the old learning method currently
    // d_model_->LearnDModelParameters(observations_, edges_);
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
  ROS_INFO("[LTM Node]: Added new grasp point"); 
  // TODO: Make this general
  geometry_msgs::PoseArray points = d_model_->GetDModelPoints();
  grasp_idxs_.push_back(points.poses.size() - 1);
  return;
}

void RobotLTM::GoalCB(const geometry_msgs::PoseStampedConstPtr& goal_pose)
{
  static int goal_num = 0;
  ROS_INFO("LTM Node: Received goal %d", goal_num);

  // Transform goal pose to reference frame
  geometry_msgs::PoseStamped goal_pose_ref_frame;
  tf_listener_.waitForTransform(goal_pose->header.frame_id, reference_frame_, ros::Time::now(), ros::Duration(3.0));
  tf_listener_.transformPose(reference_frame_, *goal_pose, goal_pose_ref_frame);

  // Set goal
  if (grasp_idxs_.size() == 0 || goal_num >= grasp_idxs_.size())
  {
    ROS_INFO("LTM Node: Grasp points have not been set, or invalid goal num\n");
  }

  goal_state_.changed_inds.push_back(grasp_idxs_[goal_num]);
  goal_state_.changed_points.poses.push_back(goal_pose_ref_frame.pose);
  ROS_INFO("LTM Node: New goal received: %f %f %f\n",
      goal_pose_ref_frame.pose.position.x, goal_pose_ref_frame.pose.position.y,
      goal_pose_ref_frame.pose.position.z);

  // Wait until number of goal states is the same as number of grasp points
  if (goal_num != grasp_idxs_.size() - 1)
  {
    goal_num++;
    return;
  }


  //TODO: Fake a start state for now. Should be set elsewhere
  State_t start_state;
  start_state.grasp_idx = grasp_idxs_[0];
  d_model_->SetStartState(start_state);
  d_model_->SetSimTimeStep(sim_time_step_);

  d_model_->SetGoalState(goal_state_);

  // Plan
  planner_->SetStart(d_model_->GetStartStateID());
  planner_->SetEpsilon(1000.0);//1000
  vector<int> state_ids, fprim_ids;
  if (!planner_->Plan(&state_ids, &fprim_ids))
  {
    printf("LTM Node: Failed to plan\n");
    return;
  }

  // Get forces from fprim_ids
  vector<tf::Vector3> forces;
  vector<int> grasp_points;
  forces.clear();
  grasp_points.clear();
  d_model_->ConvertForcePrimIDsToForcePrims(fprim_ids, &forces, &grasp_points);

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


  plan_pub_.publish(traj);

  // Simulate plan 
  // d_model_->SimulatePlan(fprim_ids);
  SimulatePlan(traj, state_ids, forces, grasp_points);

  //Print end effector trajectory
  printf("End-Effector Trajectory:\n");
  for (size_t ii = 0; ii < traj.poses.size(); ++ii)
  {
    printf("%f %f %f %f %f %f %f\n", traj.poses[ii].position.x, traj.poses[ii].position.y, traj.poses[ii].position.z,
        traj.poses[ii].orientation.x, traj.poses[ii].orientation.y, traj.poses[ii].orientation.z, traj.poses[ii].orientation.w);
  }
  printf("\n");

  // Reset goal state
  goal_num = 0;
  goal_state_.changed_inds.clear();
  goal_state_.changed_points.poses.clear();

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

  d_model_->SetPoints(pose_array);

  observations_.push_back(pose_array);
  //sleep(1);
  return;
}

void RobotLTM::SetModelFromFile(const char *model_file)
{
  d_model_->InitFromFile(model_file, model_offset_x_, model_offset_y_, model_offset_z_); 
  /*
  // Set start
  State_t start_state;
  start_state.grasp_idx = 0;
  d_model_->SetStartState(start_state);
  */
}

void RobotLTM::SaveObservationsToFile(const char* obs_file)
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

void RobotLTM::SimulatePlan(const geometry_msgs::PoseArray& plan, const vector<int>& state_ids, const vector<tf::Vector3>& forces, const vector<int>& grasp_points)
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

  int last_grasp_idx = -1, current_grasp_idx = -1;
  geometry_msgs::Pose last_grasp_pose, current_grasp_pose;
  for (size_t ii = 0; ii < state_ids.size(); ++ii)
  {
    // Get the IK for the end-effector
    pose[0] = plan.poses[ii].position.x - standoff;
    pose[1] = plan.poses[ii].position.y;
    pose[2] = plan.poses[ii].position.z;
    // TODO: Mannequin hack
    /*
    pose[3] = plan.poses[ii].orientation.w;
    pose[4] = plan.poses[ii].orientation.x;
    pose[5] = plan.poses[ii].orientation.y;
    pose[6] = plan.poses[ii].orientation.z;
    */
    pose[3] = 1.0;
    pose[4] = 0.0;
    pose[5] = 0.0;
    pose[6] = 0.0;

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
    
    // Mannequin stuff
    geometry_msgs::PoseArray points = d_model_->GetDModelPoints();
    State_t s = d_model_->GetStateFromStateID(state_ids[ii]);
    current_grasp_pose = plan.poses[ii];
    current_grasp_idx = s.grasp_idx;

    if (ii != 0 && current_grasp_idx != last_grasp_idx)
    {
      int timesteps = 100;
      for (int t = 1; t <= timesteps; ++t)
      {
        // Get the IK for the end-effector
        pose[0] = last_grasp_pose.position.x + t * (current_grasp_pose.position.x - last_grasp_pose.position.x) / timesteps;
        pose[1] = last_grasp_pose.position.y + t * (current_grasp_pose.position.y - last_grasp_pose.position.y) / timesteps;
        pose[2] = last_grasp_pose.position.z + t * (current_grasp_pose.position.z - last_grasp_pose.position.z) / timesteps;
        pose[3] = 1.0;
        pose[4] = 0.0;
        pose[5] = 0.0;
        pose[6] = 0.0;

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

      }
      usleep(1000000);
    }


    geometry_msgs::Pose l_elbow_pose, r_elbow_pose, l_wrist_pose;
    auto it = find(s.changed_inds.begin(),
        s.changed_inds.end(),   
        5);
    if ( it != s.changed_inds.end())
    { 
      int offset = distance(s.changed_inds.begin(), it);                                                               
      l_elbow_pose = s.changed_points.poses[offset];                                                                     
    } 
    else                                                                                                               
    { 
      l_elbow_pose = points.poses[5];
    }

    it = find(s.changed_inds.begin(),
        s.changed_inds.end(),   
        6);
    if ( it != s.changed_inds.end())
    { 
      int offset = distance(s.changed_inds.begin(), it);                                                               
      r_elbow_pose = s.changed_points.poses[offset];                                                                     
    } 
    else                                                                                                               
    { 
      r_elbow_pose = points.poses[6];
    }

    double ls_yaw, ls_pitch, le_yaw, le_pitch;
    double rs_yaw, rs_pitch, re_yaw, re_pitch;
    GetLShoulderAngles(l_elbow_pose, &ls_yaw, &ls_pitch);
    GetRShoulderAngles(r_elbow_pose, &rs_yaw, &rs_pitch);
    GetLElbowAngles(l_elbow_pose, l_wrist_pose, &le_yaw, &le_pitch);
    printf("%f %f\n", le_yaw, le_pitch); 
    for (int ii = 0; ii < 10; ++ii) {
      sensor_msgs::JointState joints_msg;
      joints_msg.header.stamp = ros::Time::now();

      joints_msg.name.push_back(string("LShoulderYaw"));
      joints_msg.position.push_back(ls_yaw); //1.07
      joints_msg.velocity.push_back(0.0);
      joints_msg.effort.push_back(0.0);

      joints_msg.name.push_back(string("LShoulderPitch"));
      joints_msg.position.push_back(0); //1.07
      joints_msg.velocity.push_back(0.0);
      joints_msg.effort.push_back(0.0);

      joints_msg.name.push_back(string("RShoulderYaw"));
      joints_msg.position.push_back(rs_yaw); //1.07
      joints_msg.velocity.push_back(0.0);
      joints_msg.effort.push_back(0.0);

      joints_msg.name.push_back(string("RShoulderPitch"));
      joints_msg.position.push_back(0); //1.07
      joints_msg.velocity.push_back(0.0);
      joints_msg.effort.push_back(0.0);

      mannequin_pub_.publish(joints_msg);
      usleep(10000);

    last_grasp_idx = s.grasp_idx;
    last_grasp_pose = current_grasp_pose;
    }


    // Draw the model at next timestep
    // d_model_->ApplyForce(grasp_idx_, forces[ii], sim_time_step_);
    d_model_->VisualizeState(state_ids[ii]);
    
    //seed.swap(r_angles);
    usleep(100000);
  }

  // Visualize the initial state after running through simulation
  sleep(2);
  d_model_->VisualizeState(state_ids[0]);
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


void RobotLTM::GetLShoulderAngles(geometry_msgs::Pose p, double* yaw, double* pitch)
{
  geometry_msgs::PoseArray points = d_model_->GetDModelPoints();
  geometry_msgs::Pose l_shoulder_pose = points.poses[3];
  geometry_msgs::Pose r_shoulder_pose = points.poses[4];
  geometry_msgs::Pose l_elbow_pose = points.poses[5];
  geometry_msgs::Pose r_elbow_pose = points.poses[6];
  double x = p.position.x - l_shoulder_pose.position.x;
  double y = p.position.y - l_shoulder_pose.position.y;
  double z = p.position.z - l_shoulder_pose.position.z;
  *yaw = atan(y/x);
  *pitch = atan(sqrt(Sqr(x) + Sqr(y))/z);
  return;
}

void RobotLTM::GetRShoulderAngles(geometry_msgs::Pose p, double* yaw, double* pitch)
{
  geometry_msgs::PoseArray points = d_model_->GetDModelPoints();
  geometry_msgs::Pose l_shoulder_pose = points.poses[3];
  geometry_msgs::Pose r_shoulder_pose = points.poses[4];
  geometry_msgs::Pose l_elbow_pose = points.poses[5];
  geometry_msgs::Pose r_elbow_pose = points.poses[6];
  double x = p.position.x - r_shoulder_pose.position.x;
  double y = p.position.y - r_shoulder_pose.position.y;
  double z = p.position.z - r_shoulder_pose.position.z;
  *yaw = atan(y/x);
  *pitch = atan(sqrt(Sqr(x) + Sqr(y))/z);
  return;
}

void RobotLTM::GetLElbowAngles(geometry_msgs::Pose l_elbow_pose, geometry_msgs::Pose p, double* yaw, double* pitch)
{
  geometry_msgs::PoseArray points = d_model_->GetDModelPoints();
  double x = p.position.x - l_elbow_pose.position.x;
  double y = p.position.y - l_elbow_pose.position.y;
  double z = p.position.z - l_elbow_pose.position.z;
  *yaw = atan(y/x);
  *pitch = atan(sqrt(Sqr(x) + Sqr(y))/z);
  return;
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
