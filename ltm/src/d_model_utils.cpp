/**
 * @file d_model_utils.cpp
 * @author Venkatraman Narayanan
 * Carnegie Mellon University, 2013
 */

#include <ltm/d_model_utils.h>
#include <iostream>


/** Type conversions**/

geometry_msgs::Point Vector3ToPoint(tf::Vector3 v)
{
  geometry_msgs::Point p;
  p.x = v.x(); p.y = v.y(); p.z = v.z();
  return p;
}

/**----------------------------------------------------------------------------------------------**/

/** Math**/

double Dist(geometry_msgs::Point p1, geometry_msgs::Point p2)
{
  return sqrt(Sqr(p1.x - p2.x) + Sqr(p1.y - p2.y) + Sqr(p1.z - p2.z));
}

double Norm(geometry_msgs::Point p)
{
  return sqrt(Sqr(p.x) + Sqr(p.y) + Sqr(p.z));
}

double Norm(tf::Vector3 v)
{
  return sqrt(Sqr(v.x()) + Sqr(v.y()) + Sqr(v.z()));
}

double Sqr(double num) 
{
  return num*num;
}

double Max(double a, double b, double c)
{
  return std::max(std::max(a,b), c);
}

tf::Vector3 Cross(tf::Vector3 a, tf::Vector3 b)
{
  tf::Vector3 cross_product(a.y()*b.z() - a.z()*b.y(), -a.x()*b.z() + a.z()*b.x(), 
      a.x()*b.y() - a.y()*b.x());
  return cross_product;
}

/** Linear Algebra **/

double NormalPDF(double x, double mu, double var)
{
  if (var < kFPTolerance)
  {
   printf("DModel Utils: NormalPDF error. Variance is 0");
   var = 1e-3;
  }
  return exp(-0.5 * Sqr((x-mu)) /var) / (sqrt(2 * var * M_PI));
}

double MultivariateNormalPDF(Eigen::Vector3d x, Eigen::Vector3d mu, Eigen::Matrix3d sigma)
{
  // Initialize inverse to identity, to avoid compiler warnings about using uninitialized matrix
  Eigen::Matrix3d sig_inv = Eigen::Matrix3d::Identity();
  double sig_det;
  bool invertible;
  sigma.computeInverseAndDetWithCheck(sig_inv, sig_det, invertible);

  if (!invertible)
  {
    printf("Matrix is not invertible\n");
    sig_inv = 1e3 * Eigen::Matrix3d::Identity();
    sig_det = 1e3;
  }

  Eigen::Vector3d y = (x-mu);
  double exponent = -0.5 * y.transpose() * sig_inv * y;
  return exp(exponent) / ((sqrt(pow(2 * M_PI, 3))) * sqrt(sig_det));
}
