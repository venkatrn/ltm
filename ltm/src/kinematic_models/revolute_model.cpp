/**
 * @file revolute_model.cpp
 * @author Venkatraman Narayanan
 * Carnegie Mellon University, 2014
 */

#include <ltm/kinematic_models/revolute_model.h>
#include <ltm/d_model_utils.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Transform.h>

RevoluteModel::RevoluteModel(std::string reference_frame) :
  AbstractKinematicModel(reference_frame, REVOLUTE)
{
}

RevoluteModel::RevoluteModel(std::string reference_frame, tf::Vector3 axis, tf::Vector3 axis_point) :
  AbstractKinematicModel(reference_frame, REVOLUTE)
{
  SetParams(axis, axis_point);
}

void RevoluteModel::SetParams(tf::Vector3 axis, tf::Vector3 axis_point)
{
  axis_ = axis.normalized();
  axis_point_ = axis_point;
}

geometry_msgs::Pose RevoluteModel::Transform(geometry_msgs::Pose pose, tf::Vector3 force, double del_t)
{
  geometry_msgs::Point p = pose.position;
  geometry_msgs::Quaternion q = pose.orientation;
  tf::Quaternion original_quat(q.x, q.y, q.z, q.w);
  tf::Vector3 point(p.x, p.y, p.z);

  // Moment center and arm
  tf::Vector3 projection = point.dot(axis_) * axis_;
  tf::Vector3 center = axis_point_ + projection;
  tf::Vector3 arm = point - center;
  // Project the force onto the tangent direction
  tf::Vector3 tangent = axis_.cross(arm);
  tangent.normalize();
  tf::Vector3 projected_force = tangent.dot(force) * tangent;
  // r x F
  tf::Vector3 torque = arm.cross(projected_force);
  double torque_norm = torque.length();
  //TODO: Assumes unit inertia for the rigid body
  double inertia = 1.0;
  double theta = 0.5*Sqr(del_t)*torque_norm/inertia;

  /*
     printf("Arm: %f %f %f\n", arm.x(), arm.y(), arm.z());
     printf("Normal: %f %f %f\n", normal.x(), normal.y(), normal.z());
     printf("Tangent: %f %f %f\n", tangent.x(), tangent.y(), tangent.z());
     printf("Projected vector: %f %f %f\n", projected_vector.x(), projected_vector.y(), projected_vector.z());
     printf("Torque: %f %f %f\n", torque.x(), torque.y(), torque.z());
     printf("Theta: %f\n", theta);
     */
  if (torque_norm < 1e-5) 
  {
    printf("Applied torque is zero - dividing by zero\n");
    return pose;
  }

  tf::Quaternion quat;
  quat.setRotation(torque/torque_norm, theta);
  tf::Transform tr = tf::Transform(quat);
  tf::Vector3 rotated_point = tr*(point-center) + center;
  //tf::Vector3 rotated_point = tr*(point);
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
