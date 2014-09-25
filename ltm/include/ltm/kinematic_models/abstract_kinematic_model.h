/**
 * @file abstract_kinematic_model.h
 * @author Venkatraman Narayanan
 * Carnegie Mellon University, 2014
 */

#ifndef _LTM_DMODEL_ABSTRACT_KINEMATIC_MODEL_H_
#define _LTM_DMODEL_ABSTRACT_KINEMATIC_MODEL_H_

#include <ltm/d_model_structs.h>
#include <tf/LinearMath/Vector3.h>
#include <geometry_msgs/Pose.h>
#include <string>

class AbstractKinematicModel
{
  public:
    AbstractKinematicModel(std::string reference_frame, JointType joint);
    // Transform a point using the model. Point must be in reference_frame.
    virtual geometry_msgs::Pose Transform(geometry_msgs::Pose pose, tf::Vector3 force, double del_t) = 0;

    // Accessors
    const std::string& reference_frame() const {return reference_frame_;}
    JointType joint_type() const {return joint_;}
  protected:
    std::string reference_frame_;
    JointType joint_;
};

#endif /* _LTM_DMODEL_ABSTRACT_KINEMATIC_MODEL_H_ */
