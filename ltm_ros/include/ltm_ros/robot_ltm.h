/**
 * @file robot_ltm.h
 * @author Venkatraman Narayanan
 * Carnegie Mellon University, 2014
 */

#ifndef _LTM_ROS_ROBOT_LTM_H_
#define _LTM_ROS_ROBOT_LTM_H_

#include <ltm/d_model.h>
#include <ltm/d_model_planner.h>
#include <ltm/d_model_learner.h>

#include <ros/ros.h>
#include <ltm_msgs/DModel.h>

#include <kinematics_msgs/GetKinematicSolverInfo.h>
#include <kinematics_msgs/GetPositionIK.h>

#include <sensor_msgs/JointState.h>

#include <ar_track_alvar_msgs/AlvarMarkers.h>

#include <pviz/pviz.h>

#include <tf/transform_listener.h>

#include <std_msgs/Int32.h>

class RobotLTM
{
  public:
    RobotLTM();
    ~RobotLTM();


  private:
    ros::NodeHandle nh_;

    DModel* d_model_;
    DModelPlanner* planner_;
    DModelLearner* learner_;

    PViz pviz_;
    tf::TransformListener tf_listener_;

    bool use_model_file_;
    std::string model_file_;
    std::string fprims_file_;
    std::string obs_file_;
    std::string reference_frame_;
    double sim_time_step_;
    double model_offset_x_, model_offset_y_, model_offset_z_;
    bool enforce_spatial_association_;

    int grasp_idx_;
    std::vector<int> grasp_idxs_;
    geometry_msgs::Pose grasp_pose_;
    State_t goal_state_;

    // Tracking points/ar markers
    bool ar_marker_tracking_;
    std::vector<geometry_msgs::PoseArray> observations_;
    std::vector<Edge> edges_;

    ros::ServiceClient query_client_;
    ros::ServiceClient ik_client_;

    // Subscribers
    ros::Subscriber cloud_sub_;
    ros::Subscriber goal_sub_;
    ros::Subscriber grasp_sub_;
    ros::Subscriber traj_exec_sub_;
    ros::Subscriber learning_mode_sub_;
    ros::Subscriber ar_marker_sub_;
 
    // Publishers
    ros::Publisher plan_pub_;
    // TODO: This is only for mannequin
    ros::Publisher mannequin_pub_;

    // Callbacks
    void ModelCB(const ltm_msgs::DModel& d_model);
    void GraspCB(const geometry_msgs::PoseStampedConstPtr& grasp_pose);
    /**@brief For now, the goal state is defined only by the final position
    * of the end effector
    */
    void GoalCB(const geometry_msgs::PoseStampedConstPtr& goal_pose);
    void TrajExecCB(const std_msgs::Int32ConstPtr& traj_idx);
    /**@brief Callback to trigger and end learning phase**/
    void LearnCB(const std_msgs::Int32ConstPtr& learning_mode);
    /**@ brief Recieve and store AR marker poses**/
    void ARMarkersCB(const ar_track_alvar_msgs::AlvarMarkersConstPtr& ar_markers);

    /**@brief Method to initialize DModel from file**/
    void SetModelFromFile(const char* model_file);

    /**@brief Save the observations to file**/
    void SaveObservationsToFile(const char* obs_file);

    /**@brief Simulate plan, with the PR2 robot**/
    void SimulatePlan(const geometry_msgs::PoseArray& plan, const std::vector<int>& state_ids, const std::vector<tf::Vector3>& forces, const std::vector<int>& grasp_points);

    /**@brief Get inverse kinematics for the PR2 right arm**/
    bool GetRightIK(const std::vector<double>& ik_pose, const std::vector<double>& seed, std::vector<double>* angles);

    /**@brief Mannequin hacks**/
    void GetLShoulderAngles(geometry_msgs::Pose l_elbow_p, double* yaw, double* pitch);
    void GetRShoulderAngles(geometry_msgs::Pose l_elbow_p, double* yaw, double* pitch);
    void GetLElbowAngles(geometry_msgs::Pose l_elbow_p, geometry_msgs::Pose l_wrist_p, double* yaw, double* pitch);

    /**@brief Compute the edges between points, by doing nearest neighbor search**/
    void ComputeEdges(const geometry_msgs::PoseArray& pose_array);

};
#endif /* _LTM_ROS_ROBOT_LTM_H_ */

