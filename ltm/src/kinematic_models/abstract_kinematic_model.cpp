/**
 * @file abstract_kinematic_model.cpp
 * @author Venkatraman Narayanan
 * Carnegie Mellon University, 2014
 */

#include <ltm/kinematic_models/abstract_kinematic_model.h>

AbstractKinematicModel::AbstractKinematicModel(std::string reference_frame, 
    JointType joint) : reference_frame_(reference_frame),
                       joint_(joint)                      
{
}
