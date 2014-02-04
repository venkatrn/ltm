#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <pr2_controllers_msgs/PointHeadAction.h>
#include <pr2_arm_utils/arm.h>
#include <vector>

typedef actionlib::SimpleActionClient<pr2_controllers_msgs::PointHeadAction> PointHeadClient;
//! Points the high-def camera frame at a point in a given frame  
void lookAt(std::string frame_id, double x, double y, double z, PointHeadClient* point_head_client_){
    //the goal message we will be sending
    pr2_controllers_msgs::PointHeadGoal goal;

    //the target point, expressed in the requested frame
    geometry_msgs::PointStamped point;
    point.header.frame_id = frame_id;
    point.point.x = x; point.point.y = y; point.point.z = z;
    goal.target = point;

    //we are pointing the high-def camera frame 
    //(pointing_axis defaults to X-axis)
    goal.pointing_frame = "high_def_frame";

    //take at least 0.5 seconds to get there
    goal.min_duration = ros::Duration(2);

    //and go no faster than 1 rad/s
    goal.max_velocity = 0.5;

    //send the goal
    point_head_client_->sendGoal(goal);

    //wait for it to get there (abort after 2 secs to prevent getting stuck)
    point_head_client_->waitForResult(ros::Duration(2));
    sleep(2);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "init_cabinet");
    Arm r_arm("right");
    Arm l_arm("left");
    r_arm.openGripper();
    l_arm.openGripper();
    std::vector<double> l_init(7,0);
    std::vector<double> r_init(7,0);

    l_init[0] = 0.43474247923335785;
    l_init[1] = 1.2952886686210827;
    l_init[2] = 0.30626353961552866;
    l_init[3] = -2.120429816472789;
    l_init[4] = -367.68569117708853;
    l_init[5] = -2.0067520269655006;
    l_init[6] = 66.00639888858403;
        
    r_init[0] =-0.23117967578398957;
    r_init[1] =1.0769730178922747;
    r_init[2] =0.019446379058501773;
    r_init[3] =-1.9621954289305932;
    r_init[4] =-2.8490612115504246;
    r_init[5] =-1.979940509361753;
    r_init[6] =-3.1167221313806674;

    r_arm.sendArmToConfiguration(&r_init[0],3);
    l_arm.sendArmToConfiguration(&l_init[0],3);

    /*
    PointHeadClient* point_head_client_;
    std::string point_head_srv("/head_traj_controller/point_head_action");
    point_head_client_ = new PointHeadClient(point_head_srv, true);

    //wait for head controller action server to come up 
    while(!point_head_client_->waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the point_head_action server to come up");
    }
    lookAt("base_link", 0.820, 0.095, 1.047, point_head_client_);
    */
}
