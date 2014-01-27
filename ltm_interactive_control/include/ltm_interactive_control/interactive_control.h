#include <ros/ros.h>
#include <string>
#include <interactive_markers/menu_handler.h>
#include <interactive_markers/interactive_marker_server.h>
#include <geometry_msgs/PoseStamped.h>
#include "boost/thread/mutex.hpp"

#define MENU_ENTRY_NEW_GOAL_POSE 1
#define MENU_ENTRY_NEW_GRIPPER_POSE 2


class InteractiveControl {
  public:

    InteractiveControl();
    ~InteractiveControl();

  private:
    std::string goal_pose_topic_, gripper_pose_topic_, goal_marker_name_, reference_frame_;

    interactive_markers::MenuHandler menu_handler_;
    interactive_markers::InteractiveMarkerServer server_;

    visualization_msgs::InteractiveMarkerControl box_control_, control_, menu_control_;
    visualization_msgs::InteractiveMarker int_marker_;
    visualization_msgs::Marker marker_;

    geometry_msgs::PoseStamped goal_pose_, gripper_pose_;

    ros::Publisher goal_pose_pub_;
    ros::Publisher gripper_pose_pub_;

    void ProcFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
};
