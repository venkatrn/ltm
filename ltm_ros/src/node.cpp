/**
 * @file node.cpp
 * @author Venkatraman Narayanan
 * Carnegie Mellon University, 2014
 */

#include <ltm_ros/lao_robot_ltm.h>
//#include <ltm_ros/robot_ltm.h>

#include <iostream>

using namespace std;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ltm_node");
  //RobotLTM robot_ltm;
  LAORobotLTM lao_robot_ltm;
  //ros::spin();
  ros::AsyncSpinner spinner(4); 
  spinner.start(); 
  ros::waitForShutdown();
  return 0;
}
