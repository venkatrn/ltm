/**
 * @file prismatic_model.h
 * @author Venkatraman Narayanan
 * Carnegie Mellon University, 2014
 */

#ifndef _LTM_DMODEL_PRISMATIC_MODEL_H_
#define _LTM_DMODEL_PRISMATIC_MODEL_H_

#include <ltm/kinematic_models/abstract_kinematic_model.h>
#include <tf/LinearMath/Vector3.h>

class PrismaticModel : public AbstractKinematicModel
{
  public:
    PrismaticModel(std::string reference_frame);
    PrismaticModel(std::string reference_frame, tf::Vector3 axis);
    virtual geometry_msgs::Pose Transform(geometry_msgs::Pose pose, tf::Vector3 force, double del_t);
    /**@brief The axis of rotation, and a point on the axis**/
    void SetParams(tf::Vector3 axis);

    // Accesssors
    const tf::Vector3& axis() const {return axis_;}
  protected:
    tf::Vector3 axis_;
};

#endif /* _LTM_DMODEL_PRISMATIC_MODEL_H_ */
