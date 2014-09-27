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

bool PosesEqual(const geometry_msgs::Pose& p1, const geometry_msgs::Pose& p2)
{
  if (fabs(p1.position.x-p2.position.x) >= kFPTolerance ||
      fabs(p1.position.y-p2.position.y) >= kFPTolerance ||
      fabs(p1.position.z-p2.position.z) >= kFPTolerance)
  {
    return false;
  }
  return true;
}

/**----------------------------------------------------------------------------------------------**/

/** Math**/

double Dist(geometry_msgs::Point p1, geometry_msgs::Point p2)
{
  return sqrt(Sqr(p1.x - p2.x) + Sqr(p1.y - p2.y) + Sqr(p1.z - p2.z));
}

double SqrDist(geometry_msgs::Point p1, geometry_msgs::Point p2)
{
  return Sqr(p1.x - p2.x) + Sqr(p1.y - p2.y) + Sqr(p1.z - p2.z);
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

double MultivariateNormalPDF(const geometry_msgs::Pose& p, const geometry_msgs::Pose& p_mu, double var_scaling)
{
  // TODO: consider orientation also?
  Eigen::Vector3d x(p.position.x, p.position.y, p.position.z);
  Eigen::Vector3d mu(p_mu.position.x, p_mu.position.y, p_mu.position.z);
  Eigen::Matrix3d sigma = var_scaling*Eigen::Matrix3d::Identity();
  return MultivariateNormalPDF(x, mu, sigma);
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
