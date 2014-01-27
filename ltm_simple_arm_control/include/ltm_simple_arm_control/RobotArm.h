#include <ros/ros.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <angles/angles.h>

typedef actionlib::SimpleActionClient< pr2_controllers_msgs::JointTrajectoryAction > TrajClient;

const double kMaxJointVel = 0.5;

class RobotRightArm
{
  private:
    // Action client for the joint trajectory action 
    // used to trigger the arm movement action
    TrajClient* traj_client_;

  public:
    //! Initialize the action client and wait for action server to come up
    RobotRightArm() 
    {
      // tell the action client that we want to spin a thread by default
      traj_client_ = new TrajClient("r_arm_controller/joint_trajectory_action", true);

      // wait for action server to come up
      while(!traj_client_->waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the joint_trajectory_action server");
      }
    }

    //! Clean up the action client
    ~RobotRightArm()
    {
      delete traj_client_;
    }

    //! Sends the command to start a given trajectory
    void startTrajectory(pr2_controllers_msgs::JointTrajectoryGoal goal)
    {
      // When to start the trajectory: 1s from now
      goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
      traj_client_->sendGoal(goal);
    }

    //! Generates a simple trajectory with two waypoints, used as an example
    /*! Note that this trajectory contains two waypoints, joined together
      as a single trajectory. Alternatively, each of these waypoints could
      be in its own trajectory - a trajectory can have one or more waypoints
      depending on the desired application.
      */
    pr2_controllers_msgs::JointTrajectoryGoal armSinglePointTrajectory(std::vector<double> angles)
    {
      //our goal variable
      pr2_controllers_msgs::JointTrajectoryGoal goal;
      // First, the joint names, which apply to all waypoints
      goal.trajectory.joint_names.push_back("r_shoulder_pan_joint");
      goal.trajectory.joint_names.push_back("r_shoulder_lift_joint");
      goal.trajectory.joint_names.push_back("r_upper_arm_roll_joint");
      goal.trajectory.joint_names.push_back("r_elbow_flex_joint");
      goal.trajectory.joint_names.push_back("r_forearm_roll_joint");
      goal.trajectory.joint_names.push_back("r_wrist_flex_joint");
      goal.trajectory.joint_names.push_back("r_wrist_roll_joint");
      goal.trajectory.points.resize(1);

      // Positions
      goal.trajectory.points[0].positions.resize(7);
      ros::NodeHandle ph("~");
      for(int i = 0; i < 7; i++){
        goal.trajectory.points[0].positions[i] = angles[i];
      }

      for(int i =0; i < int(goal.trajectory.points[0].positions.size()); i++)
      {
        goal.trajectory.points[0].positions[i] = angles::normalize_angle(goal.trajectory.points[0].positions[i]);
      }

      // Velocities
      goal.trajectory.points[0].velocities.resize(7);
      for (size_t j = 0; j < 7; ++j)
      {
        goal.trajectory.points[0].velocities[j] = 0.0;
      }
      // To be reached 1 second after starting along the trajectory
      goal.trajectory.points[0].time_from_start = ros::Duration(2.0);

      //we are done; return the goal
      return goal;
    }

    pr2_controllers_msgs::JointTrajectoryGoal armMultiplePointTrajectory(std::vector<double*> joint_trajectory)
    {
      pr2_controllers_msgs::JointTrajectoryGoal goal;
      goal.trajectory.joint_names.push_back("r_shoulder_pan_joint");
      goal.trajectory.joint_names.push_back("r_shoulder_lift_joint");
      goal.trajectory.joint_names.push_back("r_upper_arm_roll_joint");
      goal.trajectory.joint_names.push_back("r_elbow_flex_joint");
      goal.trajectory.joint_names.push_back("r_forearm_roll_joint");
      goal.trajectory.joint_names.push_back("r_wrist_flex_joint");
      goal.trajectory.joint_names.push_back("r_wrist_roll_joint");

      int i, j; 
      int trajectorylength = joint_trajectory.size();

      //fill the goal message with the desired joint trajectory
      goal.trajectory.points.resize(trajectorylength);

      //set the first trajectory point to the current position
      goal.trajectory.points[0].positions.resize(7);
      goal.trajectory.points[0].velocities.resize(7);
      for(j=0; j<7; j++){
        goal.trajectory.points[0].positions[j] = joint_trajectory[0][j];
        goal.trajectory.points[0].velocities[j] = 0.0; 
      }

      //make the first trajectory point start 0.25 seconds from when we run
      goal.trajectory.points[0].time_from_start = ros::Duration(0.25);     
      //goal.trajectory.points[0].time_from_start = ros::Duration(0.0);     

      //fill in the rest of the trajectory
      double time_from_start = 0.25;
      for(i=1; i<trajectorylength; i++){
        goal.trajectory.points[i].positions.resize(7);
        goal.trajectory.points[i].velocities.resize(7);

        //fill in the joint positions (velocities of 0 mean that the arm
        //will try to stop briefly at each waypoint)
        for(j=0; j<7; j++){
          goal.trajectory.points[i].positions[j] = joint_trajectory[i][j];
          goal.trajectory.points[i].velocities[j] = 0.0;
        }
        printf("%f %f %f %f %f %f %f\n", joint_trajectory[i][0],
            joint_trajectory[i][1], 
            joint_trajectory[i][2], 
            joint_trajectory[i][3], 
            joint_trajectory[i][4], 
            joint_trajectory[i][5], 
            joint_trajectory[i][6]);

        //compute a desired time for this trajectory point using a max 
        //joint velocity
        double max_joint_move = 0;
        for(j=0; j<7; j++){
          double joint_move = fabs(goal.trajectory.points[i].positions[j] 
              - goal.trajectory.points[i-1].positions[j]);
          if(joint_move > max_joint_move) max_joint_move = joint_move;
        }
        //double seconds = max_joint_move/kMaxJointVel;
        double seconds = 2.0;
        ROS_INFO("max_joint_move: %0.3f, seconds: %0.3f", max_joint_move, seconds);
        time_from_start += seconds;
        goal.trajectory.points[i].time_from_start = 
          ros::Duration(time_from_start);
      }

      //when to start the trajectory
      goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(0.25);

      //we are done; return the goal
      return goal;
    }

    //! Returns the current state of the action
    actionlib::SimpleClientGoalState getState()
    {
      return traj_client_->getState();
    }

};
