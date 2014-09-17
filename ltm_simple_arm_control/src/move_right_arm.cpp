#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/tf.h>

#include <arm_navigation_msgs/MoveArmAction.h>
#include <arm_navigation_msgs/utils.h>
#include <pr2_controllers_msgs/SingleJointPositionAction.h>
#include <pr2_controllers_msgs/Pr2GripperCommandAction.h>

#include <geometry_msgs/PoseArray.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <kinematics_msgs/GetKinematicSolverInfo.h>
#include <kinematics_msgs/GetPositionIK.h>

#include <std_msgs/Int32.h>

#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

#include <math.h>

#include <ltm_simple_arm_control/RobotArm.h>

using namespace visualization_msgs;

// %Tag(vars)%
boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
interactive_markers::MenuHandler menu_handler;
//actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction> *move_arm;
actionlib::SimpleActionClient<pr2_controllers_msgs::SingleJointPositionAction> *move_torso;
actionlib::SimpleActionClient<pr2_controllers_msgs::Pr2GripperCommandAction> *move_gripper;

ros::ServiceClient query_client;
ros::ServiceClient ik_client;

ros::Publisher goal_pub;
ros::Publisher gripper_pub;
ros::Publisher learning_mode_pub;
ros::Publisher traj_exec_mode_pub;

ros::Subscriber plan_sub;
bool execute_trajectory = false;

RobotRightArm* r_arm;

//ros::NodeHandle nh;
// %EndTag(vars)%

void moveArmTo(std::vector<double> angles){
  r_arm->startTrajectory(r_arm->armSinglePointTrajectory(angles));
  printf("Moving..."); fflush(stdout);
  // Wait for trajectory completion
  while(!r_arm->getState().isDone() && ros::ok())
  {
    printf("."); fflush(stdout);
    usleep(50000);
  }
  printf("done!\n"); fflush(stdout);
}

void moveArmAlong(std::vector<std::vector<double>> joint_trajectory){
  r_arm->startTrajectory(r_arm->armMultiplePointTrajectory(joint_trajectory));
  printf("Moving..."); fflush(stdout);
  // Wait for trajectory completion
  while(!r_arm->getState().isDone() && ros::ok())
  {
    printf("."); fflush(stdout);
    usleep(50000);
  }
  printf("done!\n"); fflush(stdout);
}


void getCurrentRightArm(std::vector<double> &angles){
	angles.resize(7);
	
	sensor_msgs::JointStateConstPtr state = ros::topic::waitForMessage < sensor_msgs::JointState > ("joint_states");
	for(int i = 0; i < (int)state->name.size(); i++){
		if(state->name[i].compare("r_shoulder_pan_joint")==0) angles[0] = state->position[i];
		if(state->name[i].compare("r_shoulder_lift_joint")==0) angles[1] = state->position[i];
		if(state->name[i].compare("r_upper_arm_roll_joint")==0) angles[2] = state->position[i];
		if(state->name[i].compare("r_elbow_flex_joint")==0) angles[3] = state->position[i];
		if(state->name[i].compare("r_forearm_roll_joint")==0) angles[4] = state->position[i];
		if(state->name[i].compare("r_wrist_flex_joint")==0) angles[5] = state->position[i];
		if(state->name[i].compare("r_wrist_roll_joint")==0) angles[6] = state->position[i];
	}
}

bool getRightIK(std::vector<double> ik_pose, std::vector<double> seed, std::vector<double> &angles){

  // define the service messages
  kinematics_msgs::GetKinematicSolverInfo::Request request;
  kinematics_msgs::GetKinematicSolverInfo::Response response;

  if(query_client.call(request,response))
  {
    for(unsigned int i=0; i< response.kinematic_solver_info.joint_names.size(); i++)
    {
      ROS_INFO("Joint: %d %s",i,response.kinematic_solver_info.joint_names[i].c_str());
    }
  }
  else
  {
    ROS_ERROR("Could not call query service");
    return false;
  }
  // define the service messages
  
  kinematics_msgs::GetPositionIK::Request  gpik_req;
  kinematics_msgs::GetPositionIK::Response gpik_res;
  gpik_req.timeout = ros::Duration(5.0);
  gpik_req.ik_request.ik_link_name = "r_wrist_roll_link";

  //gpik_req.ik_request.pose_stamped.header.frame_id = "torso_lift_link";
  gpik_req.ik_request.pose_stamped.header.frame_id = "base_link";
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
    gpik_req.ik_request.ik_seed_state.joint_state.position[i] = seed[i]; //(response.kinematic_solver_info.limits[i].min_position + response.kinematic_solver_info.limits[i].max_position)/2.0;
  }
  
  if(ik_client.call(gpik_req, gpik_res))
  {
    if(gpik_res.error_code.val == gpik_res.error_code.SUCCESS){
      ROS_INFO("Got IK:");
      for(unsigned int i=0; i < gpik_res.solution.joint_state.name.size(); i ++){
        ROS_INFO("Joint: %s %f",gpik_res.solution.joint_state.name[i].c_str(),gpik_res.solution.joint_state.position[i]);
        angles[i] = gpik_res.solution.joint_state.position[i];
      }
      return true;
    } else{
      ROS_ERROR("Inverse kinematics to %.3f, %.3f, %.3f failed", ik_pose[0], ik_pose[1], ik_pose[2]);
      return false;
    }
  } else {
    ROS_ERROR("Inverse kinematics service call failed");
    return false; 
  }
  return false;
}

// %Tag(Box)%
Marker makeBox( InteractiveMarker &msg )
{
  Marker marker;

  marker.type = Marker::CUBE;
  marker.scale.x = msg.scale * 0.45;
  marker.scale.y = msg.scale * 0.45;
  marker.scale.z = msg.scale * 0.45;
  marker.color.r = 0.5;
  marker.color.g = 0.5;
  marker.color.b = 0.5;
  marker.color.a = 1.0;

  return marker;
}

InteractiveMarkerControl& makeBoxControl( InteractiveMarker &msg )
{
  InteractiveMarkerControl control;
  control.always_visible = true;
  
  Marker marker_;
  
  std::vector<std::string> pr2_gripper_meshes_;
  pr2_gripper_meshes_.push_back("package://pr2_description/meshes/gripper_v0/upper_finger_r.stl");
  pr2_gripper_meshes_.push_back("package://pr2_description/meshes/gripper_v0/finger_tip_r.stl");
  pr2_gripper_meshes_.push_back("package://pr2_description/meshes/gripper_v0/upper_finger_l.stl");
  pr2_gripper_meshes_.push_back("package://pr2_description/meshes/gripper_v0/finger_tip_l.stl");

  for(int i = 0; i < (int)pr2_gripper_meshes_.size(); ++i)
  {
    marker_.header.stamp = ros::Time::now();
    //marker_.header.frame_id = chain_root_name_;
    marker_.ns = "goal_gripper";
    marker_.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker_.id = i;
    marker_.action = visualization_msgs::Marker::ADD;
    marker_.pose.position.x = 0.10;// = poses.at(i).pose;
    marker_.scale.x = 1.0;  
    marker_.scale.y = 1.0;
    marker_.scale.z = 1.0;

    marker_.color.r = 0.0;
    marker_.color.g = 1.0;
    marker_.color.b = 0.0;
    marker_.color.a = 1.0;
    //marker_array_.markers[i].lifetime = ros::Duration(120.0);
    marker_.mesh_resource = pr2_gripper_meshes_[i];
    control.markers.push_back( marker_ );
  }
  
  msg.controls.push_back( control );

  return msg.controls.back();
}
// %EndTag(Box)%
void OpenGripper()
{
  pr2_controllers_msgs::Pr2GripperCommandGoal open;
  open.command.position = 0.08;
  open.command.max_effort = -1.0;  // Do not limit effort (negative)
  move_gripper->sendGoal(open);
  move_gripper->waitForResult();
  return;
}

void CloseGripper()
{
  pr2_controllers_msgs::Pr2GripperCommandGoal close;
  close.command.position = 0.0;
  close.command.max_effort = 100.0;  // Close gently
  move_gripper->sendGoal(close);
  move_gripper->waitForResult();
  return;
}


void planCB(const geometry_msgs::PoseArrayConstPtr& plan)
{
  bool safe_execution = false;
  // Do nothin if execute trajectory is not on
  if (!execute_trajectory)
  {
    // Fake publish end of execution message
    std_msgs::Int32 traj_exec_mode;
    traj_exec_mode.data = 1;
    traj_exec_mode_pub.publish(traj_exec_mode);
    return;
  }
  // Reset execute trajectory to false
  execute_trajectory = false;

  if (plan->poses.size() == 0)
  {
    return;
  }

  std::vector<double> pose(7,0);
  std::vector<double> seed(7,0);
  std::vector<double> angles(7,0);

  /*
  // Move arm to home position
  pose[0] = 0.512524;
  pose[1] = 0.004654;
  pose[2] = -0.4272;
  pose[3] = 1.0;
  pose[4] = 0.0;
  pose[5] = 0.0;
  pose[6] = 0.0;
  getCurrentRightArm(seed);
  if(getRightIK(pose, seed, angles)){
    moveArmTo(angles);
  }
  */

  // Open the gripper
  OpenGripper();

  // Pre-grasp position 
  pose[0] = plan->poses[0].position.x;
  pose[1] = plan->poses[0].position.y;
  pose[2] = plan->poses[0].position.z;
  pose[3] = plan->poses[0].orientation.w;
  pose[4] = plan->poses[0].orientation.x;
  pose[5] = plan->poses[0].orientation.y;
  pose[6] = plan->poses[0].orientation.z;
  getCurrentRightArm(seed);
  if(getRightIK(pose, seed, angles)){
    moveArmTo(angles);
  }
  else if (safe_execution)
  {
    return;
  }

  // Close gripper after reaching grasp pose
  CloseGripper();
  // Wait for gripper to close fully
  sleep(2.0);

  std::vector<std::vector<double>> joint_trajectory;
  getCurrentRightArm(seed);
  for (size_t ii = 0; ii < plan->poses.size(); ++ii)
  {
    pose[0] = plan->poses[ii].position.x;
    pose[1] = plan->poses[ii].position.y;
    pose[2] = plan->poses[ii].position.z;
    pose[3] = plan->poses[ii].orientation.w;
    pose[4] = plan->poses[ii].orientation.x;
    pose[5] = plan->poses[ii].orientation.y;
    pose[6] = plan->poses[ii].orientation.z;
    std::vector<double> traj_point(7,0);
    if(getRightIK(pose, seed, traj_point)){
      // moveArmTo(angles); 
      joint_trajectory.push_back(traj_point);
    }
     seed.swap(angles);
  }
  
  printf("Joint Trajectory:\n");
  for (size_t ii = 0; ii < joint_trajectory.size(); ++ii)
  {
        printf("%f %f %f %f %f %f %f\n", joint_trajectory[ii][0],
            joint_trajectory[ii][1], 
            joint_trajectory[ii][2], 
            joint_trajectory[ii][3], 
            joint_trajectory[ii][4], 
            joint_trajectory[ii][5], 
            joint_trajectory[ii][6]);
  }

  moveArmAlong(joint_trajectory);
  
  // Open the gripper after executing trajectory
  OpenGripper();
  
  // Publish end of execution message
  std_msgs::Int32 traj_exec_mode;
  traj_exec_mode.data = 1;
  traj_exec_mode_pub.publish(traj_exec_mode);

  return;
}

// %Tag(processFeedback)%
void processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  std::ostringstream s;
  s << "Feedback from marker '" << feedback->marker_name << "' "
      << " / control '" << feedback->control_name << "'";

  std::ostringstream mouse_point_ss;
  if( feedback->mouse_point_valid )
  {
    mouse_point_ss << " at " << feedback->mouse_point.x
                   << ", " << feedback->mouse_point.y
                   << ", " << feedback->mouse_point.z
                   << " in frame " << feedback->header.frame_id;
  }
  std::vector<double> pose(7,0);
  std::vector<double> seed(7,0);
  std::vector<double> angles(7,0);
  
  geometry_msgs::PoseStamped goal_pose;
  geometry_msgs::PoseStamped gripper_pose;
  
  pr2_controllers_msgs::SingleJointPositionGoal torso_;

  switch ( feedback->event_type )
  {
    case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
      ROS_INFO_STREAM( s.str() << ": button click" << mouse_point_ss.str() << "." );
      break;

    case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
      ROS_INFO_STREAM( s.str() << ": menu item " << feedback->menu_entry_id << " clicked" << mouse_point_ss.str() << "." );
      if(feedback->menu_entry_id == 1){
        execute_trajectory = true;
      	goal_pose.pose.position.x = feedback->pose.position.x;
	      goal_pose.pose.position.y = feedback->pose.position.y;
	      goal_pose.pose.position.z = feedback->pose.position.z;
	      goal_pose.pose.orientation.w = feedback->pose.orientation.w;
	      goal_pose.pose.orientation.x = feedback->pose.orientation.x;
	      goal_pose.pose.orientation.y = feedback->pose.orientation.y;
	      goal_pose.pose.orientation.z = feedback->pose.orientation.z;
	      goal_pose.header.stamp = ros::Time::now();
	      goal_pose.header.frame_id = feedback->header.frame_id;
	      goal_pub.publish(goal_pose);
      }
      if(feedback->menu_entry_id == 2){
	      pose[0] = feedback->pose.position.x;
	      pose[1] = feedback->pose.position.y;
	      pose[2] = feedback->pose.position.z;
	      pose[3] = feedback->pose.orientation.w;
	      pose[4] = feedback->pose.orientation.x;
	      pose[5] = feedback->pose.orientation.y;
	      pose[6] = feedback->pose.orientation.z;
	      getCurrentRightArm(seed);
	      if(getRightIK(pose, seed, angles)){
	      	moveArmTo(angles);
	      }
      }
      if(feedback->menu_entry_id == 3){
      	      goal_pose.pose.position.x = feedback->pose.position.x;
	      goal_pose.pose.position.y = feedback->pose.position.y;
	      goal_pose.pose.position.z = feedback->pose.position.z;
	      goal_pose.pose.orientation.w = feedback->pose.orientation.w;
	      goal_pose.pose.orientation.x = feedback->pose.orientation.x;
	      goal_pose.pose.orientation.y = feedback->pose.orientation.y;
	      goal_pose.pose.orientation.z = feedback->pose.orientation.z;
	      goal_pose.header.stamp = ros::Time::now();
	      goal_pose.header.frame_id = feedback->header.frame_id;
	      goal_pub.publish(goal_pose);
      }
      if(feedback->menu_entry_id == 4){
      	      gripper_pose.pose.position.x = feedback->pose.position.x;
	      gripper_pose.pose.position.y = feedback->pose.position.y;
	      gripper_pose.pose.position.z = feedback->pose.position.z;
	      gripper_pose.pose.orientation.w = feedback->pose.orientation.w;
	      gripper_pose.pose.orientation.x = feedback->pose.orientation.x;
	      gripper_pose.pose.orientation.y = feedback->pose.orientation.y;
	      gripper_pose.pose.orientation.z = feedback->pose.orientation.z;
	      gripper_pose.header.stamp = ros::Time::now();
	      gripper_pose.header.frame_id = feedback->header.frame_id;
	      gripper_pub.publish(gripper_pose);
      }
      
      if(feedback->menu_entry_id == 5){ //Torso UP
		torso_.position = 0.30;  //all the way up is 0.2
		torso_.min_duration = ros::Duration(2.0);
		torso_.max_velocity = 1.0;     
		ROS_INFO("Moving the torso all the way up");
		move_torso->sendGoal(torso_);
		move_torso->waitForResult();
      }
      if(feedback->menu_entry_id == 6){ //Torso DOWN
		torso_.position = 0.0;  
		torso_.min_duration = ros::Duration(2.0);
		torso_.max_velocity = 1.0;     
		ROS_INFO("Moving the torso all the way down");
		move_torso->sendGoal(torso_);
		move_torso->waitForResult();
      }
      if(feedback->menu_entry_id == 7){ // IK to start
	      pose[0] = 0.512524;
	      pose[1] = 0.004654;
	      pose[2] = -0.4272;
	      pose[3] = 1.0;
	      pose[4] = 0.0;
	      pose[5] = 0.0;
	      pose[6] = 0.0;
	      getCurrentRightArm(seed);
	      if(getRightIK(pose, seed, angles)){
	      	moveArmTo(angles);
        }
      }
      if(feedback->menu_entry_id == 8){ // Plan to goal
        // Mannequin example
        execute_trajectory = false;
      	gripper_pose.pose.position.x = 0.55;
	      gripper_pose.pose.position.y = -0.2;
	      gripper_pose.pose.position.z = 0.2;
	      gripper_pose.pose.orientation.w =  1.0;
	      gripper_pose.pose.orientation.x = 0.0;
	      gripper_pose.pose.orientation.y = 0.0;
	      gripper_pose.pose.orientation.z =  0.0;
	      gripper_pose.header.stamp = ros::Time::now();
	      gripper_pose.header.frame_id = feedback->header.frame_id;
	      gripper_pub.publish(gripper_pose);
        sleep(1);
      	gripper_pose.pose.position.x = 0.55;
	      gripper_pose.pose.position.y = 0.2;
	      gripper_pose.pose.position.z = 0.2;
	      gripper_pose.pose.orientation.w =  1.0;
	      gripper_pose.pose.orientation.x = 0.0;
	      gripper_pose.pose.orientation.y = 0.0;
	      gripper_pose.pose.orientation.z =  0.0;
	      gripper_pose.header.stamp = ros::Time::now();
	      gripper_pose.header.frame_id = feedback->header.frame_id;
	      gripper_pub.publish(gripper_pose);
        sleep(1);
        /*
      	gripper_pose.pose.position.x = 0.35;
	      gripper_pose.pose.position.y = -0.2;
	      gripper_pose.pose.position.z = 0.2;
	      gripper_pose.pose.orientation.w =  1.0;
	      gripper_pose.pose.orientation.x = 0.0;
	      gripper_pose.pose.orientation.y = 0.0;
	      gripper_pose.pose.orientation.z =  0.0;
	      gripper_pose.header.stamp = ros::Time::now();
	      gripper_pose.header.frame_id = feedback->header.frame_id;
	      gripper_pub.publish(gripper_pose);
        sleep(1);
        */
      	goal_pose.pose.position.x = 0.75;
	      goal_pose.pose.position.y = -0.4;
	      goal_pose.pose.position.z = 0.2;
	      goal_pose.pose.orientation.w = 1.0;
	      goal_pose.pose.orientation.x = 0.0;
	      goal_pose.pose.orientation.y = 0.0;
	      goal_pose.pose.orientation.z = 0.0;
	      goal_pose.header.stamp = ros::Time::now();
	      goal_pose.header.frame_id = feedback->header.frame_id;
	      goal_pub.publish(goal_pose);
        sleep(1);
      	goal_pose.pose.position.x = 0.75;
	      goal_pose.pose.position.y = 0.4;
	      goal_pose.pose.position.z = 0.2;
	      goal_pose.pose.orientation.w = 1.0;
	      goal_pose.pose.orientation.x = 0.0;
	      goal_pose.pose.orientation.y = 0.0;
	      goal_pose.pose.orientation.z = 0.0;
	      goal_pose.header.stamp = ros::Time::now();
	      goal_pose.header.frame_id = feedback->header.frame_id;
	      goal_pub.publish(goal_pose);
        sleep(1);
        /*
      	goal_pose.pose.position.x = 0.75;
	      goal_pose.pose.position.y = -0.4; //
	      goal_pose.pose.position.z = -0.0; //-0.2
	      goal_pose.pose.orientation.w = 1.0;
	      goal_pose.pose.orientation.x = 0.0;
	      goal_pose.pose.orientation.y = 0.0;
	      goal_pose.pose.orientation.z = 0.0;
	      goal_pose.header.stamp = ros::Time::now();
	      goal_pose.header.frame_id = feedback->header.frame_id;
	      goal_pub.publish(goal_pose);
        */
      }
      if(feedback->menu_entry_id == 9){ // Start tracking
        std_msgs::Int32 learning_mode;
        learning_mode.data = 1;
	      learning_mode_pub.publish(learning_mode);
      }
      if(feedback->menu_entry_id == 10){ // Stop tracking
        std_msgs::Int32 learning_mode;
        learning_mode.data = 2;
	      learning_mode_pub.publish(learning_mode);
      }
      break;

    case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
      ROS_INFO_STREAM( s.str() << ": pose changed"
          << "\nposition = "
          << feedback->pose.position.x
          << ", " << feedback->pose.position.y
          << ", " << feedback->pose.position.z
          << "\norientation = "
          << feedback->pose.orientation.w
          << ", " << feedback->pose.orientation.x
          << ", " << feedback->pose.orientation.y
          << ", " << feedback->pose.orientation.z
          << "\nframe: " << feedback->header.frame_id
          << " time: " << feedback->header.stamp.sec << "sec, "
          << feedback->header.stamp.nsec << " nsec" );
      break;

    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
      ROS_INFO_STREAM( s.str() << ": mouse down" << mouse_point_ss.str() << "." );
      break;

    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
      ROS_INFO_STREAM( s.str() << ": mouse up" << mouse_point_ss.str() << "." );
      break;
  }

  server->applyChanges();
}
// %EndTag(processFeedback)%

// %Tag(alignMarker)%
void alignMarker( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  geometry_msgs::Pose pose = feedback->pose;

  pose.position.x = round(pose.position.x-0.5)+0.5;
  pose.position.y = round(pose.position.y-0.5)+0.5;

  ROS_INFO_STREAM( feedback->marker_name << ":"
      << " aligning position = "
      << feedback->pose.position.x
      << ", " << feedback->pose.position.y
      << ", " << feedback->pose.position.z
      << " to "
      << pose.position.x
      << ", " << pose.position.y
      << ", " << pose.position.z );

  server->setPose( feedback->marker_name, pose );
  server->applyChanges();
}
// %EndTag(alignMarker)%

double rand( double min, double max )
{
  double t = (double)rand() / (double)RAND_MAX;
  return min + t*(max-min);
}

void saveMarker( InteractiveMarker int_marker )
{
  server->insert(int_marker);
  server->setCallback(int_marker.name, &processFeedback);
}

////////////////////////////////////////////////////////////////////////////////////

// %Tag(6DOF)%
void make6DofMarker( bool fixed, unsigned int interaction_mode, const tf::Vector3& position, bool show_6dof )
{
  InteractiveMarker int_marker;
  int_marker.header.frame_id = "torso_lift_link";
  tf::pointTFToMsg(position, int_marker.pose.position);
  int_marker.scale = 0.5;

  int_marker.name = "right_gripper_6dof";
  int_marker.description = "LTM Control";

  // insert a box
  makeBoxControl(int_marker);
  int_marker.controls[0].interaction_mode = interaction_mode;

  InteractiveMarkerControl control;

  if ( fixed )
  {
    int_marker.name += "_fixed";
    int_marker.description += "\n(fixed orientation)";
    control.orientation_mode = InteractiveMarkerControl::FIXED;
  }

  /*if (interaction_mode != visualization_msgs::InteractiveMarkerControl::NONE)
  {
      std::string mode_text;
      if( interaction_mode == visualization_msgs::InteractiveMarkerControl::MOVE_AXIS )         mode_text = "MOVE_3D";
      if( interaction_mode == visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS )       mode_text = "ROTATE_3D";
      if( interaction_mode == visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE )  mode_text = "MOVE_ROTATE_3D";
      int_marker.name += "_" + mode_text;
      int_marker.description = std::string("3D Control") + (show_6dof ? " + 6-DOF controls" : "") + "\n" + mode_text;
  }*/

  if(show_6dof)
  {
    control.orientation.w = 1;
    control.orientation.x = 1;
    control.orientation.y = 0;
    control.orientation.z = 0;
    control.name = "rotate_x";
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_x";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.name = "rotate_z";
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_z";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 0;
    control.orientation.z = 1;
    control.name = "rotate_y";
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_y";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);
  }

  server->insert(int_marker);
  server->setCallback(int_marker.name, &processFeedback);
  if (interaction_mode != visualization_msgs::InteractiveMarkerControl::NONE)
    menu_handler.apply( *server, int_marker.name );
}
// %EndTag(6DOF)%

// %Tag(Menu)%
void makeMenuMarker( const tf::Vector3& position )
{
  InteractiveMarker int_marker;
  int_marker.header.frame_id = "base_link";
  tf::pointTFToMsg(position, int_marker.pose.position);
  int_marker.scale = 1;

  int_marker.name = "context_menu";
  int_marker.description = "Context Menu\n(Right Click)";

  InteractiveMarkerControl control;

  control.interaction_mode = InteractiveMarkerControl::MENU;
  control.name = "menu_only_control";

  Marker marker = makeBox( int_marker );
  control.markers.push_back( marker );
  control.always_visible = true;
  int_marker.controls.push_back(control);

  server->insert(int_marker);
  server->setCallback(int_marker.name, &processFeedback);
  menu_handler.apply( *server, int_marker.name );
}
// %EndTag(Menu)%

// %Tag(Button)%
void makeButtonMarker( const tf::Vector3& position )
{
  InteractiveMarker int_marker;
  int_marker.header.frame_id = "base_link";
  tf::pointTFToMsg(position, int_marker.pose.position);
  int_marker.scale = 1;

  int_marker.name = "button";
  int_marker.description = "Button\n(Left Click)";

  InteractiveMarkerControl control;

  control.interaction_mode = InteractiveMarkerControl::BUTTON;
  control.name = "button_control";

  Marker marker = makeBox( int_marker );
  control.markers.push_back( marker );
  control.always_visible = true;
  int_marker.controls.push_back(control);

  server->insert(int_marker);
  server->setCallback(int_marker.name, &processFeedback);
}
// %EndTag(Button)%

// %Tag(main)%
int main(int argc, char** argv)
{
  ros::init (argc, argv, "move_arm_pose_goal_test");
  ros::NodeHandle n;
  ros::NodeHandle ph("~");
  
  ros::service::waitForService("pr2_right_arm_kinematics/get_ik_solver_info");
  ros::service::waitForService("pr2_right_arm_kinematics/get_ik");
  
  query_client = n.serviceClient<kinematics_msgs::GetKinematicSolverInfo>("pr2_right_arm_kinematics/get_ik_solver_info");
  ik_client = n.serviceClient<kinematics_msgs::GetPositionIK>("pr2_right_arm_kinematics/get_ik");
  
  goal_pub = n.advertise<geometry_msgs::PoseStamped>("goal_pose", 1);
  gripper_pub = n.advertise<geometry_msgs::PoseStamped>("gripper_pose", 1);
  learning_mode_pub = n.advertise<std_msgs::Int32>("learning_mode", 1);
  traj_exec_mode_pub = n.advertise<std_msgs::Int32>("traj_exec_mode", 1);

  plan_sub = n.subscribe("ltm_plan", 1, &planCB);
  
  r_arm = new RobotRightArm();
  
  std::string arm_name;
  ROS_INFO("using right arm"); 
//  actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction> move_arm(nh,"move_arm");
//  move_arm = new actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction>("move_right_arm",true);
//  move_arm->waitForServer();
  ROS_INFO("Connected to move arm server");  
  
  move_torso = new actionlib::SimpleActionClient<pr2_controllers_msgs::SingleJointPositionAction>("torso_controller/position_joint_action", true);
  move_torso->waitForServer();
  ROS_INFO("Connected to move torso server");

  move_gripper = new actionlib::SimpleActionClient<pr2_controllers_msgs::Pr2GripperCommandAction>("r_gripper_controller/gripper_action", true);
  move_gripper->waitForServer();
  ROS_INFO("Connected to move torso server");


  // create a timer to update the published transforms
  //ros::Timer frame_timer = n.createTimer(ros::Duration(0.01), frameCallback);

  server.reset( new interactive_markers::InteractiveMarkerServer("basic_arm_controls","",false) );

  ros::Duration(0.1).sleep();

  menu_handler.insert( "Send Goal and Execute", &processFeedback );
  menu_handler.insert( "Move with IK", &processFeedback );
  menu_handler.insert( "Send Goal", &processFeedback );
  menu_handler.insert( "Send Gripper Pose", &processFeedback );
  menu_handler.insert( "Torso Up", &processFeedback );
  menu_handler.insert( "Torso Down", &processFeedback );
  menu_handler.insert( "IK to Start", &processFeedback );
  menu_handler.insert( "Plan to Goal", &processFeedback );
  menu_handler.insert( "Start tracking", &processFeedback );
  menu_handler.insert( "Stop tracking", &processFeedback );
  
  //menu_handler.insert( "Second Entry", &processFeedback );
  //interactive_markers::MenuHandler::EntryHandle sub_menu_handle = menu_handler.insert( "Submenu" );
  //menu_handler.insert( sub_menu_handle, "First Entry", &processFeedback );
  //menu_handler.insert( sub_menu_handle, "Second Entry", &processFeedback );

  tf::Vector3 position;
  
  position = tf::Vector3( 0, 0, 0);
  make6DofMarker( false, visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE, position, true );
  //makeMenuMarker( position );
  //position = tf::Vector3( 0,-9, 0);
  //makeButtonMarker( position );

  server->applyChanges();

  ROS_INFO("READY!");

  ros::spin();

  server.reset();
}
// %EndTag(main)%
