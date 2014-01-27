/**
 * @file point_tracker.h
 * @brief This node tracks a set of 3D keypoints in the pointcloud and 
 * publishes their coordinates over time.
 * @author Venkatraman Narayanan
 * Carnegie Mellon University, 2014
 */

#ifndef _POINT_TRACKER_POINT_TRACKER_H_
#define _POINT_TRACKER_POINT_TRACKER_H_

#include <point_tracker/pcl_typedefs.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/keypoints/sift_keypoint.h>
//#include <pcl/keypoints/iss_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud2.h>

typedef std::vector<std::vector<geometry_msgs::Point>> TrackedPoints;

class PointTracker
{
  public:
    PointTracker();
    ~PointTracker();

    void CloudCBInternal(const PointCloudPtr& pcl_cloud);

    /**@brief Coordinates of the tracked points (#points x #frames).**/
     TrackedPoints tracked_points_;

     /**@brief Indices of nearest neighbor for each point in the first frame**/
     vector<vector<int>> knn_indices_;

    inline pcl::visualization::PCLVisualizer GetVisualizer()
    {
      return vis_;
    }

    // Debug
    void PrintEdges();

  private:
    ros::Subscriber cloud_sub;
    ros::Publisher d_model_pub;

    pcl::visualization::PCLVisualizer vis_;


    LocalDescriptorsPtr recent_local_descriptors_;
    PointCloudPtr recent_keypoints_;


    /**@brief Callback to handle an incoming point cloud. This does
     * all the work--the keypoints are first detected, matched with the
     * previous set, and then added to the set of tracked points**/
    void CloudCB(const sensor_msgs::PointCloud2ConstPtr& sensor_cloud);

    /**@brief Downsample a point cloud**/
    PointCloudPtr DownsampleCloud(const PointCloudPtr& input, float l_x, float l_y, float l_z);

    /**@brief Compute surface normals for every point in the input point cloud
     * @param radius The size of the local neighborhood used to estimate the surface
     * @return Pointer to a surface normals point cloud
     */
    SurfaceNormalsPtr EstimateSurfaceNormals (const PointCloudPtr& input, float radius);

    /**@brief Detect SIFT Keypoints in a point cloud
     * @param points The input point cloud
     * @param normals Surface normals for the input point cloud
     * @param min_scale The smallest scale in the DoG scale-space
     * @param nr_octaves The number of times the scale doubles in the DoG scale-space
     * @param nr_scales_per_octave The number of scales computed for each doubling
     * @param min_contrast The minimum local contrast that must be present for a keypoint to be detected
     * @return Pointer to a point cloud of keypoints
     */
    PointCloudPtr DetectSIFTKeypoints(const PointCloudPtr& points, const SurfaceNormalsPtr& normals,
        float min_scale, int nr_octaves, int nr_scales_per_octave, float min_contrast);

    /**@brief ISS3D keypoint detector
    */
    // PointCloudPtr DetectISS3DKeypoints(const PointCloudPtr& points);

      /* @brief FPFHEstimation to compute local feature descriptors around each keypoint
       * @param points The input point cloud
       * @param normals Surface normals for the input point cloud
       * @param keypoints Keypoint for the input point cloud
       * @param feature_radius The size of the neighborhood from which the local descriptors will be computed 
       * @return A pointer to a LocalDescriptors (a cloud of LocalDescriptorT points)
       */
      LocalDescriptorsPtr ComputeLocalDescriptors (const PointCloudPtr& points, const SurfaceNormalsPtr & normals, const PointCloudPtr & keypoints, float feature_radius);

      /**@brief Find correspondences between two sets of feature point clouds
       */
      void FindCorrespondences(LocalDescriptorsPtr source, LocalDescriptorsPtr target, std::vector<int> *correspondences);

      /**@brief Filter correspondences**/
      void FilterCorrespondences(const vector<int>& source2target, const vector<int>& target2source, const PointCloudPtr& source_keypoints, const PointCloudPtr& target_keypoints, pcl::CorrespondencesPtr pcl_correspondences);

      /**@brief Learn model from tracked points**/
      void LearnModel();

};

#endif /** _POINT_TRACKER_POINT_TRACKER_H **/

