/**
 * @file lao_robot_ltm.h
 * @author Venkatraman Narayanan
 * Carnegie Mellon University, 2014
 */

#ifndef _LTM_ROS_LAO_ROBOT_LTM_H_
#define _LTM_ROS_LAO_ROBOT_LTM_H_

#include <ltm/d_model_bank.h>
#include <ltm/lao_planner.h>
#include <ltm/d_model_learner.h>

#include <ros/ros.h>
#include <ltm_msgs/DModel.h>

#include <kinematics_msgs/GetKinematicSolverInfo.h>
#include <kinematics_msgs/GetPositionIK.h>

#include <pcl_ros/transforms.h>
#include <rosbag/bag.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/PointCloud2.h>

#include <ar_track_alvar_msgs/AlvarMarkers.h>

#include <pviz/pviz.h>

#include <tf/transform_listener.h>

#include <std_msgs/Int32.h>

#include <string>

class LAORobotLTM
{
  public:
    LAORobotLTM();
    ~LAORobotLTM();


  private:
    ros::NodeHandle nh_;

    DModelBank* d_model_bank_;
    LAOPlanner* planner_;
    DModelLearner* learner_;

    PViz pviz_;
    tf::TransformListener tf_listener_;

    int num_models_;

    bool use_model_file_;
    std::vector<std::string> model_files_;
    std::string fprims_file_;
    std::string obs_file_;
    std::string bag_file_;
    std::string reference_frame_;
    double sim_time_step_;
    std::vector<double> model_offsets_x_, model_offsets_y_, model_offsets_z_;
    bool enforce_spatial_association_;

    int grasp_idx_;
    std::vector<int> grasp_idxs_;
    geometry_msgs::Pose grasp_pose_;
    State_t start_state_, goal_state_;
    int num_goals_received_; // Keep track of number of goals received--must match number of grasp poses.

    // Tracking points/ar markers
    bool ar_marker_tracking_;
    std::vector<geometry_msgs::PoseArray> observations_;
    std::vector<Edge> edges_;
    rosbag::Bag bag_;

    ros::ServiceClient query_client_;
    ros::ServiceClient ik_client_;

    // Subscribers
    ros::Subscriber point_cloud_sub_;
    ros::Subscriber goal_sub_;
    ros::Subscriber grasp_sub_;
    ros::Subscriber traj_exec_sub_;
    ros::Subscriber learning_mode_sub_;
    ros::Subscriber ar_marker_sub_;
 
    // Publishers
    ros::Publisher plan_pub_;

    // Callbacks
    void GraspCB(const geometry_msgs::PoseStampedConstPtr& grasp_pose);
    /**@brief For now, the goal state is defined only by the final position
    * of the end effector
    */
    void GoalCB(const geometry_msgs::PoseStampedConstPtr& goal_pose);
    /**@brief Helper for GoalCB. Note: goal_state must be set outside of this method**/
    void PlanAndExecute();
    void GetExecutionTraj(const std::vector<int>& state_ids, const std::vector<int>& fprim_ids, geometry_msgs::PoseArray* execution_traj);
    void GetStartBelief(std::vector<double>* belief);
    void UpdateStartState();

    /** CB triggered when part of trajectory has been executed--depending on completion status, we will
     * replan and execute, or declare success**/
    void TrajExecCB(const std_msgs::Int32ConstPtr& exec_mode_ptr);
    /**@brief Callback to trigger and end learning phase**/
    void LearnCB(const std_msgs::Int32ConstPtr& learning_mode);
    /**@ brief Recieve and store AR marker poses**/
    void ARMarkersCB(const ar_track_alvar_msgs::AlvarMarkersConstPtr& ar_markers);
    /**@brief This is temporary--will go away once the API for recording and stopping rosbags is ready**/
    void KinectCB(const sensor_msgs::PointCloud2& point_cloud);

    /**@brief Method to initialize DModel from file**/
    void SetModelBankFromFile(std::vector<std::string> model_files);

    /**@brief Save the observations to file**/
    void SaveObservationsToFile(const char* obs_file);

    /**@brief Simulate plan, with the PR2 robot--this is just a visualization of the states in the path**/
    void SimulatePlan(int model_id, const geometry_msgs::PoseArray& plan, const std::vector<int>& state_ids, const std::vector<tf::Vector3>& forces, const std::vector<int>& grasp_points);

    /**@brief Get inverse kinematics for the PR2 right arm**/
    bool GetRightIK(const std::vector<double>& ik_pose, const std::vector<double>& seed, std::vector<double>* angles);

    /**@brief Compute the edges between points, by doing nearest neighbor search**/
    void ComputeEdges(const geometry_msgs::PoseArray& pose_array);

};
#endif /* _LTM_ROS_LAO_ROBOT_LTM_H_ */

