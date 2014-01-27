#include <ltm/d_model.h>

#include <tf/LinearMath/Vector3.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ltm");
  ros::NodeHandle nh;
  DModel d_model;
  //ros::Timer timer = nh.createTimer(ros::Duration(0.1), &DModel::TFTimedCallback, &d_model);

  d_model.InitFromFile("/usr0/home/venkatrn/groovy_workspace/sandbox/ltm/models/drawer.mdl");
  //d_model.InitFromFile("/usr0/home/venkatrn/groovy_workspace/sandbox/ltm/models/revolute.mdl");
  //d_model.InitFromFile("/usr0/home/venkatrn/groovy_workspace/sandbox/ltm/models/door.mdl");

  /*
  d_model.PrintPoints();
  d_model.PrintEdges();
  */

  geometry_msgs::PoseArray in_points = d_model.GetDModelPoints();
  // Apply force
  for (int ii =1; ii<300; ii++)
  {
  // Drawer
  tf::Vector3 force(1.0, 0.0, 0.0);
  //d_model.ApplyForce(134, force, 0.1);
  geometry_msgs::PoseArray out_points;
  d_model.GetNextState(in_points, 134, force, 0.1, &out_points);
  d_model.TFCallback(out_points);
  in_points = out_points;

  /*
  // Revolute
  tf::Vector3 force(0.2, 0.2, 0.0);
  d_model.ApplyForce(1, force, 0.1);
  */
    /*
  tf::Vector3 force(0.8, 0.2, 0.0);
  d_model.ApplyForce(13, force, 0.1);
  */

  usleep(10000);
  }

  /*
  d_model.PrintPoints();
  */

  return 0;
}
