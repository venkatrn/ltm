/**
 * @file ltm_viz.h
 * @author Venkatraman Narayanan
 * Carnegie Mellon University, 2014
 */

#ifndef _LTM_VIZ_LTM_VIZ_H_
#define _LTM_VIZ_LTM_VIZ_H_

#include <ltm/d_model.h>

#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace ltm {
  struct RGBA
  {
    double r, g, b, a;
    RGBA()
    {
      r = g = b = 0;
      a = 1;
    }
  };
}

class LTMViz
{
  public:
    LTMViz(const std::string& ns);
    ~LTMViz();
    
    void VisualizePoints(const geometry_msgs::PoseArray poses, const ltm::RGBA color);
    void VisualizePoints(const geometry_msgs::PoseArray poses);

    void VisualizeEdges(const EdgeMap& edge_map);

  private:
};

#endif /* _LTM_VIZ_LTM_VIZ_H_ */
