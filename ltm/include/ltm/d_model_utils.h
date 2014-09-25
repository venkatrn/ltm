/**
 * @file d_model_utils.h
 * @author Venkatraman Narayanan
 * Carnegie Mellon University, 2013
 */

#ifndef _LTM_DMODEL_UTILS_H_
#define _LTM_DMODEL_UTILS_H_

#include <geometry_msgs/PoseArray.h>
#include <tf/LinearMath/Vector3.h>
#include <Eigen/Core>
#include <Eigen/LU>

// Error tolerance for comparing floating point numbers.
const double kFPTolerance = 1e-3; //1e-3

geometry_msgs::Point Vector3ToPoint(tf::Vector3 v);

double Dist(geometry_msgs::Point p1, geometry_msgs::Point p2);
double SqrDist(geometry_msgs::Point p1, geometry_msgs::Point p2);
double Norm(geometry_msgs::Point p);
double Norm(tf::Vector3 v);
tf::Vector3 Cross(tf::Vector3 a, tf::Vector3 b);

double Sqr(double num);
double Max(double a, double b, double c);

double NormalPDF(double x, double mean, double var);
double MultivariateNormalPDF(Eigen::Vector3d x, Eigen::Vector3d mu, Eigen::Matrix3d sigma);

#endif /* _LTM_DMODEL_UTILS_H */
