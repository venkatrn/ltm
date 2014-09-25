/** 
 * @file d_model.h
 * @author Venkatraman Narayanan
 * Carnegie Mellon University, 2013
 */
#ifndef _LTM_DMODEL_STRUCTS_H_
#define _LTM_DMODEL_STRUCTS_H_


#include <tf/LinearMath/Vector3.h>

#include <unordered_map>


enum JointType 
{
  FIXED = -1,
  RIGID,
  PRISMATIC, 
  REVOLUTE, 
  SPHERICAL
};

struct pair_hash
{
  size_t operator() (const std::pair<int, int>& p) const
  {return p.first+1000*p.second;}
};

struct EdgeParams
{
  JointType joint;
  tf::Vector3 normal;
  tf::Vector3 center;
  double rad;

  EdgeParams()
  {
    joint = RIGID;
    rad = 0.0;
  }
  // Prismatic
  EdgeParams(JointType jt, tf::Vector3 dir, double r)
  {
    joint = jt;
    normal = dir;
    rad = r;
    center = tf::Vector3(0.0, 0.0, 0.0);
  }
  // Revolute
  EdgeParams(JointType jt, tf::Vector3 dir, tf::Vector3 cen, double r)
  {
    joint = jt;
    normal = dir;
    center = cen;
    rad = r;
  }
};

typedef std::pair<int, int> Edge;
typedef std::unordered_map<Edge, EdgeParams, pair_hash> EdgeMap;


#endif /* _LTM_DMODEL_STRUCTS_H */
