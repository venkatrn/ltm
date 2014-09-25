/**
 * @file revolute_model.h
 * @author Venkatraman Narayanan
 * Carnegie Mellon University, 2014
 */

#ifndef _LTM_DMODEL_REVOLUTE_MODEL_H_
#define _LTM_DMODEL_REVOLUTE_MODEL_H_

#include <ltm/kinematic_models/abstract_kinematic_model.h>
#include <tf/LinearMath/Vector3.h>

class RevoluteModel : public AbstractKinematicModel
{
  public:
    RevoluteModel(std::string reference_frame);
    RevoluteModel(std::string reference_frame, tf::Vector3 axis, tf::Vector3 axis_point);
    virtual geometry_msgs::Pose Transform(geometry_msgs::Pose pose, tf::Vector3 force, double del_t);
    /**@brief The axis of rotation, and a point on the axis**/
    void SetParams(tf::Vector3 axis, tf::Vector3 axis_point);

    // Accesssors
    const tf::Vector3& axis() const {return axis_;}
    const tf::Vector3& axis_point() const {return axis_point_;}
  protected:
    tf::Vector3 axis_;
    tf::Vector3 axis_point_;
};

#endif /* _LTM_DMODEL_REVOLUTE_MODEL_H_ */
