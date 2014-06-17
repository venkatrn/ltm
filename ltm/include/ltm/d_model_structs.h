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
  double rad;

  EdgeParams()
  {
    joint = RIGID;
    rad = 0.0;
  }
  EdgeParams(JointType jt, tf::Vector3 dir, double r)
  {
    joint = jt;
    normal = dir;
    rad = r;
  }
};

typedef std::pair<int, int> Edge;
typedef std::unordered_map<Edge, EdgeParams, pair_hash> EdgeMap;


#endif /* _LTM_DMODEL_STRUCTS_H */
