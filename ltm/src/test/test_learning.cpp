#include <ltm/d_model.h>

#include <tf/LinearMath/Vector3.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>

#include <vector>

using namespace std;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ltm");
  ros::NodeHandle nh;

  /*
  // 3 points, 3 frames, prismatic
  geometry_msgs::Pose p11;
  p11.position.x = 0.0; p11.position.y = 0.0; p11.position.z = 0.0;
  p11.orientation.x = 0.0; p11.orientation.y = 0.0; p11.orientation.z = 0.0; p11.orientation.w = 1.0;

  geometry_msgs::Pose p21;
  p21.position.x = 1.0; p21.position.y = 0.0; p21.position.z = 0.0;
  p21.orientation.x = 0.0; p21.orientation.y = 0.0; p21.orientation.z = 0.0; p21.orientation.w = 1.0;

  geometry_msgs::Pose p31;
  p31.position.x = 2.0; p31.position.y = 0.0; p31.position.z = 0.0;
  p31.orientation.x = 0.0; p31.orientation.y = 0.0; p31.orientation.z = 0.0; p31.orientation.w = 1.0;

  geometry_msgs::PoseArray pa1;
  pa1.poses.push_back(p11); pa1.poses.push_back(p21); pa1.poses.push_back(p31);

  geometry_msgs::Pose p12;
  p12.position.x = 0.0; p12.position.y = 0.0; p12.position.z = 0.0;
  p12.orientation.x = 0.0; p12.orientation.y = 0.0; p12.orientation.z = 0.0; p12.orientation.w = 1.0;

  geometry_msgs::Pose p22;
  p22.position.x = 2.0; p22.position.y = 0.0; p22.position.z = 0.0;
  p22.orientation.x = 0.0; p22.orientation.y = 0.0; p22.orientation.z = 0.0; p22.orientation.w = 1.0;

  geometry_msgs::Pose p32;
  p32.position.x = 3.0; p32.position.y = 0.0; p32.position.z = 0.0;
  p32.orientation.x = 0.0; p32.orientation.y = 0.0; p32.orientation.z = 0.0; p32.orientation.w = 1.0;

  geometry_msgs::PoseArray pa2;
  pa2.poses.push_back(p12); pa2.poses.push_back(p22); pa2.poses.push_back(p32);

  geometry_msgs::Pose p13;
  p13.position.x = 0.0; p13.position.y = 0.0; p13.position.z = 0.0;
  p13.orientation.x = 0.0; p13.orientation.y = 0.0; p13.orientation.z = 0.0; p13.orientation.w = 1.0;

  geometry_msgs::Pose p23;
  p23.position.x = 3.0; p23.position.y = 0.0; p23.position.z = 0.0;
  p23.orientation.x = 0.0; p23.orientation.y = 0.0; p23.orientation.z = 0.0; p23.orientation.w = 1.0;

  geometry_msgs::Pose p33;
  p33.position.x = 4.0; p33.position.y = 0.0; p33.position.z = 0.0;
  p33.orientation.x = 0.0; p33.orientation.y = 0.0; p33.orientation.z = 0.0; p33.orientation.w = 1.0;

  geometry_msgs::PoseArray pa3;
  pa3.poses.push_back(p13); pa3.poses.push_back(p23); pa3.poses.push_back(p33);

  vector<geometry_msgs::PoseArray> observations;
  observations.push_back(pa1); observations.push_back(pa2); observations.push_back(pa3);

  vector<Edge> edges;
  edges.push_back(make_pair(0, 1));
  edges.push_back(make_pair(1, 2));
  */

  tf::Quaternion quat, new_quat;
  tf::Vector3 rotated_point;
  // 2 points, 3 frames, revolute
  geometry_msgs::Pose p11;
  p11.position.x = 0.0; p11.position.y = 0.0; p11.position.z = 0.0;
  p11.orientation.x = 0.0; p11.orientation.y = 0.0; p11.orientation.z = 0.0; p11.orientation.w = 1.0;

  geometry_msgs::Pose p21;
  p21.position.x = 1.0; p21.position.y = 0.0; p21.position.z = 0.0;
  p21.orientation.x = 0.0; p21.orientation.y = 0.0; p21.orientation.z = 0.0; p21.orientation.w = 1.0;

  geometry_msgs::PoseArray pa1;
  pa1.poses.push_back(p11); pa1.poses.push_back(p21); 

  geometry_msgs::Pose p12;
  p12.position.x = 0.0; p12.position.y = 0.0; p12.position.z = 0.0;
  p12.orientation.x = 0.0; p12.orientation.y = 0.0; p12.orientation.z = 0.0; p12.orientation.w = 1.0;

  quat.setRotation(tf::Vector3(0.0, 0.0, 1.0), M_PI/6);
  tf::Transform tr1(quat);
  rotated_point = tr1 * (tf::Vector3(p21.position.x, p21.position.y, p21.position.z));
  new_quat = quat * tf::Quaternion(p21.orientation.x, p21.orientation.y, p21.orientation.z, p21.orientation.z);
  geometry_msgs::Pose p22;
  p22.position.x = rotated_point.x(); p22.position.y = rotated_point.y(); p22.position.z = rotated_point.z();
  p22.orientation.x = new_quat.x(); p22.orientation.y = new_quat.y(); p22.orientation.z = new_quat.z(); p22.orientation.w = new_quat.w();

  geometry_msgs::PoseArray pa2;
  pa2.poses.push_back(p12); pa2.poses.push_back(p22);

  geometry_msgs::Pose p13;
  p13.position.x = 0.0; p13.position.y = 0.0; p13.position.z = 0.0;
  p13.orientation.x = 0.0; p13.orientation.y = 0.0; p13.orientation.z = 0.0; p13.orientation.w = 1.0;

  quat.setRotation(tf::Vector3(0.0, 0.0, 1.0), M_PI/3);
  tf::Transform tr2(quat);
  rotated_point = tr2 * (tf::Vector3(p21.position.x, p21.position.y, p21.position.z));
  new_quat = quat * tf::Quaternion(p21.orientation.x, p21.orientation.y, p21.orientation.z, p21.orientation.z);
  geometry_msgs::Pose p23;
  p23.position.x = rotated_point.x(); p23.position.y = rotated_point.y(); p23.position.z = rotated_point.z();
  p23.orientation.x = new_quat.x(); p23.orientation.y = new_quat.y(); p23.orientation.z = new_quat.z(); p23.orientation.w = new_quat.w();

  geometry_msgs::PoseArray pa3;
  pa3.poses.push_back(p13); pa3.poses.push_back(p23);

  vector<geometry_msgs::PoseArray> observations;
  observations.push_back(pa1); observations.push_back(pa2); observations.push_back(pa3);

  vector<Edge> edges;
  edges.push_back(make_pair(0, 1));

  DModel* d_model = new DModel;
  d_model->SetPoints(pa1);
  d_model->LearnDModelParameters(observations, edges);
  d_model->PrintEdges();

  delete d_model;
  return 0;
}
