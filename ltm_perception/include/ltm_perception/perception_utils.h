/**
 * @file perception_utils.h
 * @brief This node tracks a set of 3D keypoints in the pointcloud and 
 * publishes their coordinates over time.
 * @author Venkatraman Narayanan
 * Carnegie Mellon University, 2014
 */

#ifndef _LTM_PERCEPTION_PERCEPTION_UTILS_H_
#define _LTM_PERCEPTION_PERCEPTION_UTILS_H_

#include <ltm_perception/pcl_typedefs.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/visualization/pcl_visualizer.h>

namespace perception_utils
{

  /**@brief Check if the cluster satisifies simple checks to be considered as an articulated object**/
  bool EvaluateCluster(PointCloudPtr cloud_cluster);
  /**@brief Obtain plane coefficients for segmentation**/
  pcl::ModelCoefficients::Ptr GetPlaneCoefficients(PointCloudPtr cloud);
  pcl::ModelCoefficients::Ptr GetPlaneCoefficients(PointCloudPtr cloud, PointCloudPtr inlier_points);
  /**@brief Ditto, but for lines**/
  pcl::ModelCoefficients::Ptr GetLineCoefficients(PointCloudPtr cloud, PointCloudPtr inlier_points);
  /**@brief 3D edge detection**/
  //void GetEdges(PointCloudPtr cloud);
  /**@brief Segment out the ground plane**/
  PointCloudPtr RemoveGroundPlane(PointCloudPtr cloud, pcl::ModelCoefficients::Ptr coefficients);
  /**@brief Get clusters from the point cloud**/ 
  void DoEuclideanClustering(PointCloudPtr cloud, std::vector<PointCloudPtr>* cluster_clouds);
  /**@brief Downsample point cloud**/
  PointCloudPtr DownsamplePointCloud(PointCloudPtr cloud);
  /**@brief Draw bounding box for point cloud**/
  void DrawOrientedBoundingBox(pcl::visualization::PCLVisualizer& viewer, PointCloudPtr cloud, std::string box_id);
  void DrawAxisAlignedBoundingBox(pcl::visualization::PCLVisualizer& viewer, PointCloudPtr cloud, std::string box_id);
} /** perception_utils **/

#endif /** _LTM_PERCEPTION_PERCEPTION_UTILS **/
