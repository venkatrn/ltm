/**
 * @file spherical_model.cpp
 * @author Venkatraman Narayanan
 * Carnegie Mellon University, 2014
 */

#include <ltm/kinematic_models/spherical_model.h>
#include <ltm/d_model_utils.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Transform.h>

SphericalModel::SphericalModel(std::string reference_frame) :
  AbstractKinematicModel(reference_frame, SPHERICAL)
{
}

SphericalModel::SphericalModel(std::string reference_frame, tf::Vector3 center_point) :
  AbstractKinematicModel(reference_frame, SPHERICAL)
{
  SetParams(center_point);
}

void SphericalModel::SetParams(tf::Vector3 center_point)
{
  center_point_ = center_point;
}

geometry_msgs::Pose SphericalModel::Transform(geometry_msgs::Pose pose, tf::Vector3 force, double del_t)
{
  geometry_msgs::Point p = pose.position;
  geometry_msgs::Quaternion q = pose.orientation;
  tf::Quaternion original_quat(q.x, q.y, q.z, q.w);
  tf::Vector3 point(p.x, p.y, p.z);

  // Moment center and arm
  tf::Vector3 arm = point - center_point_;
  // r x F
  tf::Vector3 torque = arm.cross(force);
  //TODO: Assumes unit inertia for the rigid body
  double inertia = 1.0;
  tf::Vector3 omega = 0.5 * Sqr(del_t) * torque;
  double theta = omega.length();

  // DEBUG
  /*
   printf("Arm: %f %f %f\n", arm.x(), arm.y(), arm.z());
   printf("Torque: %f %f %f\n", torque.x(), torque.y(), torque.z());
   printf("Theta: %f\n", theta);
  */
  if (theta < 1e-5) 
  {
    printf("Applied torque is zero - dividing by zero\n");
    return pose;
  }

  tf::Quaternion quat;
  quat.setRotation(torque.normalized(), theta);
  tf::Transform tr = tf::Transform(quat);
  tf::Vector3 rotated_point = tr*(point-center_point_) + center_point_;
  tf::Quaternion rotated_quat = quat * original_quat;

  geometry_msgs::Pose new_pose;
  // TODO: use the tf conversions for all this stuff
  new_pose.position.x = double(rotated_point.x());
  new_pose.position.y = double(rotated_point.y());
  new_pose.position.z = double(rotated_point.z());
  new_pose.orientation.x = double(rotated_quat.x());
  new_pose.orientation.y = double(rotated_quat.y());
  new_pose.orientation.z = double(rotated_quat.z());
  new_pose.orientation.w = double(rotated_quat.w());
  return new_pose;
}

void SphericalModel::Transform(geometry_msgs::PoseArray in_poses, geometry_msgs::Pose application_pose, tf::Vector3 force, double del_t, geometry_msgs::PoseArray* out_poses)
{

  geometry_msgs::Point application_p = application_pose.position;
  geometry_msgs::Quaternion application_q = application_pose.orientation;
  tf::Vector3 application_point(application_p.x, application_p.y, application_p.z);

  // Moment center and arm
  tf::Vector3 arm = application_point - center_point_;
  // r x F
  tf::Vector3 torque = arm.cross(force);
  //TODO: Assumes unit inertia for the rigid body
  double inertia = 1.0;
  tf::Vector3 omega = 0.5 * Sqr(del_t) * torque;
  double theta = omega.length();

  // DEBUG
  /*
   printf("Arm: %f %f %f\n", arm.x(), arm.y(), arm.z());
   printf("Torque: %f %f %f\n", torque.x(), torque.y(), torque.z());
   printf("Theta: %f\n", theta);
  */
  if (theta < 1e-5) 
  {
    printf("Applied torque is zero - dividing by zero\n");
    (*out_poses) = in_poses;
    return;
  }

  tf::Quaternion quat;
  quat.setRotation(torque.normalized(), theta);
  tf::Transform tr = tf::Transform(quat);
  out_poses->poses.clear();
  for (int ii = 0; ii < in_poses.poses.size(); ++ii)
  {
    geometry_msgs::Point p = in_poses.poses[ii].position;
    geometry_msgs::Quaternion q = in_poses.poses[ii].orientation;
    tf::Quaternion original_quat(q.x, q.y, q.z, q.w);
    tf::Vector3 point(p.x, p.y, p.z);
    tf::Vector3 rotated_point = tr*(point-center_point_) + center_point_;
    tf::Quaternion rotated_quat = quat * original_quat;
    geometry_msgs::Pose new_pose;
    // TODO: use the tf conversions for all this stuff
    new_pose.position.x = double(rotated_point.x());
    new_pose.position.y = double(rotated_point.y());
    new_pose.position.z = double(rotated_point.z());
    new_pose.orientation.x = double(rotated_quat.x());
    new_pose.orientation.y = double(rotated_quat.y());
    new_pose.orientation.z = double(rotated_quat.z());
    new_pose.orientation.w = double(rotated_quat.w());
    out_poses->poses.push_back(new_pose);
  }
}
