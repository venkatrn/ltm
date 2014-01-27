#include <ltm_interactive_control/interactive_control.h>

InteractiveControl::InteractiveControl() :
  server_ ("ltm_interactive_control")
{
  ros::NodeHandle nh;

  nh.param<std::string>("goal_pub_topic", goal_pose_topic_, "/goal_pose");
  nh.param<std::string>("gripper_pub_topic", gripper_pose_topic_, "/gripper_pose");
  nh.param<std::string>("goal_marker_name", goal_marker_name_, "ltm_interactive_control_marker");
  nh.param<std::string>("reference_frame", reference_frame_, "/base_link");

  goal_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>(goal_pose_topic_,1);
  gripper_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>(gripper_pose_topic_,1);

  int_marker_.header.frame_id = reference_frame_;
  int_marker_.name = goal_marker_name_;
  int_marker_.description = "";

  /*
  // create a grey box marker
  marker_.type = visualization_msgs::Marker::CUBE;
  marker_.scale.x = 0.5;
  marker_.scale.y = 0.5;
  marker_.scale.z = 0.5;
  marker_.color.r = 1;
  marker_.color.g = 0;
  marker_.color.b = 0;
  marker_.color.a = 1.0;
  */

  // create a non-interactive control which contains the box
  box_control_.always_visible = true;

  // PR2 end-effector marker
  std::vector<std::string> pr2_gripper_meshes;
  pr2_gripper_meshes.push_back("package://pr2_description/meshes/gripper_v0/upper_finger_r.stl");
  pr2_gripper_meshes.push_back("package://pr2_description/meshes/gripper_v0/finger_tip_r.stl");
  pr2_gripper_meshes.push_back("package://pr2_description/meshes/gripper_v0/upper_finger_l.stl");
  pr2_gripper_meshes.push_back("package://pr2_description/meshes/gripper_v0/finger_tip_l.stl");

  for(int i = 0; i < (int)pr2_gripper_meshes.size(); ++i)
  {
    marker_.header.stamp = ros::Time::now();
    marker_.header.frame_id = reference_frame_;
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
    marker_.mesh_resource = pr2_gripper_meshes[i];
    box_control_.markers.push_back( marker_ );
  }

    // add the control to the interactive marker
    int_marker_.controls.push_back( box_control_ );

    // Menu Stuff
    menu_handler_.insert( "Send Goal Pose", boost::bind(&InteractiveControl::ProcFeedback, this, _1) );
    menu_handler_.insert( "Send Gripper Pose", boost::bind(&InteractiveControl::ProcFeedback, this, _1) );
    menu_control_.interaction_mode = visualization_msgs::InteractiveMarkerControl::MENU;
    menu_control_.description="LTM Control Options";
    menu_control_.name = "menu_only_control";
    menu_control_.always_visible = true;
    int_marker_.controls.push_back(menu_control_);


    //TODO: these commands don't make sense
    control_.orientation.w = 1;
    control_.orientation.x = 1;
    control_.orientation.y = 0;
    control_.orientation.z = 0;
    control_.name = "move_x";
    control_.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker_.controls.push_back(control_);

    control_.orientation.w = 1;
    control_.orientation.x = 0;
    control_.orientation.y = 1;
    control_.orientation.z = 0;
    control_.name = "rotate_z";
    control_.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker_.controls.push_back(control_);
    control_.name = "move_z";
    control_.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker_.controls.push_back(control_);

    control_.orientation.w = 1;
    control_.orientation.x = 0;
    control_.orientation.y = 0;
    control_.orientation.z = 1;
    control_.name = "move_y";
    control_.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker_.controls.push_back(control_);


    // Add the interactive marker to our collection &
    // tell the server to call ProcFeedback() when feedback arrives for it
    server_.insert(int_marker_, boost::bind(&InteractiveControl::ProcFeedback, this, _1) );

    menu_handler_.apply(server_, int_marker_.name );
    // 'commit' changes and send to all clients
    server_.applyChanges();
  }

  InteractiveControl::~InteractiveControl() {

  }

  void InteractiveControl::ProcFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback )
  {
    ROS_INFO("In proc feedback\n");
    ROS_INFO_STREAM( feedback->marker_name << " is now at "
        << feedback->pose.position.x << ", " << feedback->pose.position.y
        << ", " << feedback->pose.position.z );

    if(feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT)
    {
      if(feedback->menu_entry_id == MENU_ENTRY_NEW_GOAL_POSE)
      {
        ROS_INFO("Interactive Control: Sending new goal pose");
        //geometry_msgs::PoseStamped goal_pose;
        goal_pose_.header.stamp = ros::Time::now();
        goal_pose_.header.frame_id = reference_frame_;
        goal_pose_.pose = feedback->pose;
        goal_pose_pub_.publish(goal_pose_);
      }
      else if(feedback->menu_entry_id == MENU_ENTRY_NEW_GRIPPER_POSE)
      {
        ROS_INFO("Interactive Control: Sending new gripper pose");
        gripper_pose_.header.stamp = ros::Time::now();
        gripper_pose_.header.frame_id = reference_frame_;
        gripper_pose_.pose = feedback->pose;
        gripper_pose_pub_.publish(gripper_pose_);
      }
    }

  }

  int main(int argc, char** argv)
  {
    ros::init(argc, argv, "ltm_interactive_control");
    InteractiveControl interactive_control;
    ros::spin();
    return 0;
  }

