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
#include <pcl/segmentation/organized_multi_plane_segmentation.h>

namespace perception_utils
{

  void OrganizedSegmentation(PointCloudPtr cloud, std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT>>>* regions);

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

  /**@brief Project points onto model specified by coefficients--only planes for now**/
  PointCloudPtr ProjectOntoPlane(const pcl::ModelCoefficients::Ptr& coefficients, PointCloudPtr cloud);

  /**@brief Remove statistical outliers**/
  PointCloudPtr RemoveOutliers(PointCloudPtr cloud);

  /**@brief Passthrough filter**/
  PointCloudPtr PassthroughFilter(PointCloudPtr cloud);

  /**@brief Downsample point cloud**/
  PointCloudPtr DownsamplePointCloud(PointCloudPtr cloud);

  /**@brief Draw bounding box for point cloud**/
  void DrawOrientedBoundingBox(pcl::visualization::PCLVisualizer& viewer, PointCloudPtr cloud, std::string box_id);
  void DrawAxisAlignedBoundingBox(pcl::visualization::PCLVisualizer& viewer, PointCloudPtr cloud, std::string box_id);
  /**@brief Draw rectangle boundaries**/
  void DrawRectangle(pcl::visualization::PCLVisualizer& viewer, const std::vector<PointT>& corners, std::string rect_id);

  /**@brief Get planar convex hull vertices**/
  void GetPolygonVertices(PointCloudPtr cloud, std::vector<PointT>* poly_vertices);

  /**@brief Input: periphery points from planar segmentation. Output: Points projected onto a ransac rectangle (implemented by finding 4 lines), and also the two orthonormal axis vectors
   **/
  bool GetRectanglePoints(PointCloudPtr cloud, PointCloudPtr rectangle_points, std::vector<Eigen::Vector3f>* axes);


  /**@brief Get the corner points in a point cloud (of a rectangle boundary)**/
  void GetRectangleCorners(PointCloudPtr cloud, std::vector<PointT>* corners, const std::vector<Eigen::Vector3f>& axes);


  void DisplayPlanarRegions(pcl::visualization::PCLVisualizer& viewer, std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT>>> &regions);
} /** perception_utils **/

#endif /** _LTM_PERCEPTION_PERCEPTION_UTILS **/
