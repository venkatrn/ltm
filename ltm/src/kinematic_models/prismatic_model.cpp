/**
 * @file prismatic_model.cpp
 * @author Venkatraman Narayanan
 * Carnegie Mellon University, 2014
 */

#include <ltm/kinematic_models/prismatic_model.h>
#include <ltm/d_model_utils.h>

PrismaticModel::PrismaticModel(std::string reference_frame) :
                        AbstractKinematicModel(reference_frame, PRISMATIC)
{
}

PrismaticModel::PrismaticModel(std::string reference_frame, tf::Vector3 axis) :
                        AbstractKinematicModel(reference_frame, PRISMATIC)
{
  SetParams(axis);
}


void PrismaticModel::SetParams(tf::Vector3 axis)
{
  axis_ = axis;
}

geometry_msgs::Pose PrismaticModel::Transform(geometry_msgs::Pose pose, tf::Vector3 force, double del_t)
{
  // Force and pose need to be in reference_frame_
  tf::Vector3 projected_force = force.dot(axis_) * axis_;
  const double mass = 1.0;
  tf::Vector3 displacement = 0.5*Sqr(del_t)*projected_force/mass;
  geometry_msgs::Pose new_pose;
  new_pose.orientation = pose.orientation;
  new_pose.position.x = pose.position.x + displacement.x();
  new_pose.position.y = pose.position.y + displacement.y();
  new_pose.position.z = pose.position.z + displacement.z();
  return new_pose;
}
