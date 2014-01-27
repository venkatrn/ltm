/**
 * @file real_ltm.cpp
 * @author Venkatraman Narayanan
 * Carnegie Mellon University, 2014
 */

#include <ltm_ros/robot_ltm.h>
#include <ltm/d_model.h>
#include <ltm/d_model_utils.h>


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
  ar_marker_sub_ = nh_.subscribe("ar_pose_markers", 1, &RobotLTM::ARMarkersCB, this);

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
    return;
  }

  if (learning_mode->data == 2)
  {
    // Stop AR Marker tracking, and pass observations to d_model
    ar_marker_tracking_ = false;
    // TODO: Compute edges
    
    if (int(observations_.size()) == 0)
    {
      ROS_INFO("LTM Node: Learning stopped before any observations have been received"
          "No model will be learnt");
      return;
    }
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
  // Set grasp index by finding closest pose in dmodel points.
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
  goal_state.changed_inds.push_back(grasp_idx_);
  goal_state.changed_points.poses.push_back(goal_pose_ref_frame.pose);
  ROS_INFO("LTM Node: New goal received: %f %f %f\n",
      goal_pose_ref_frame.pose.position.x, goal_pose_ref_frame.pose.position.y,
      goal_pose_ref_frame.pose.position.z);
  d_model_->SetGoalState(goal_state);

  //TODO: Fake a start state for now. Should be set elsewhere
  State_t start_state;
  d_model_->SetStartState(start_state);
  d_model_->SetSimTimeStep(sim_time_step_);

  // Plan
  planner_->SetStart(d_model_->GetStartStateID());
  planner_->SetEpsilon(1000.0);
  vector<int> state_ids, fprim_ids;
  planner_->Plan(&state_ids, &fprim_ids);

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
  //plan_pub_.publish(traj);

  // Simulate plan 
  // d_model_->SimulatePlan(fprim_ids);
  SimulatePlan(traj, state_ids, forces);

  /*
  //Print end effector trajectory
  printf("End-Effector Trajectory:\n");
  for (size_t ii = 0; ii < traj.poses.size(); ++ii)
  {
    printf("%f %f %f %f %f %f %f\n", traj.poses[ii].position.x, traj.poses[ii].position.y, traj.poses[ii].position.z,
        traj.poses[ii].orientation.x, traj.poses[ii].orientation.y, traj.poses[ii].orientation.z, traj.poses[ii].orientation.w);
  }
  printf("\n");
  */

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

void RobotLTM::ARMarkersCB(const ar_pose::ARMarkersConstPtr& ar_markers)
{
  // Update the d_model state
  const int num_markers = int(ar_markers->markers.size());
  geometry_msgs::PoseArray pose_array;
  for (int ii = 0; ii < num_markers; ++ii)
  {
    pose_array.poses.push_back(ar_markers->markers[ii].pose.pose);
  }
  d_model_->SetPoints(pose_array);

  // Do not store poses if not in learning mode
  if (!ar_marker_tracking_)
  {
    return;
  }

  // Ensure we have same number of markers in each frame
  if (int(observations_.size()) != 0)
  {
    if (num_markers != int(observations_[0].poses.size()))
    {
      ROS_INFO("LTM Node: Error in AR Marker tracking. "
          "Number of markers ,%d in current frame does not match number being tracked %d",
          num_markers, int(observations_[0].poses.size()));
    }
  }
  observations_.push_back(pose_array);
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

  /*
  kinematics_msgs::GetKinematicSolverInfo::Response response;
  kinematics_msgs::GetKinematicSolverInfo::Request request;
  query_client_.call(request, response);

  for (int ii = 0; ii < 7; ++ii)
  {
    seed[ii] = (response.kinematic_solver_info.limits[ii].min_position + response.kinematic_solver_info.limits[ii].max_position)/2.0;
  }
  */

  vector<double> base_pos(6,0);
  const double torso_pos = 0.3;
  const double hue = 0.5;
  const string ns = "ltm_plan";
  const bool use_pr2_mesh = true;
  
  const double standoff = 0.15;

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
      return;
    }

    if (ii == 0)
    {
      initial_r_angles = r_angles;
    }

    // Draw the model at next timestep
    // d_model_->ApplyForce(grasp_idx_, forces[ii], sim_time_step_);
    d_model_->VisualizeState(state_ids[ii]);
    
    seed.swap(r_angles);
    usleep(10000);
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
