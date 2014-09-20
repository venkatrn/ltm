/**
 * @file point_tracker.cpp
 * @author Venkatraman Narayanan
 * Carnegie Mellon University, 2014
 */

#include <ltm_perception/point_tracker.h>
#include <ltm_msgs/DModel.h>

#include <pcl/ros/conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree_flann.h>

using namespace std;

const double kFeatureRadius = 0.1;
const double kSurfaceNormalRadius = 0.01;
const double kKNNSearchRadius = 10;
const int kKNNSearchK = 3;

PointTracker::PointTracker() :
  recent_local_descriptors_(new LocalDescriptors),
  recent_keypoints_(new PointCloud)
{
  ros::NodeHandle nh;
  cloud_sub = nh.subscribe("input_cloud", 1, &PointTracker::CloudCB, this);
  d_model_pub = nh.advertise<ltm_msgs::DModel>("d_model", 1);
}

PointTracker::~PointTracker()
{
}

void PointTracker::CloudCB(const sensor_msgs::PointCloud2ConstPtr& sensor_cloud)
{
  PointCloudPtr pcl_cloud;
  pcl::fromROSMsg (*sensor_cloud, *pcl_cloud);
  CloudCBInternal(pcl_cloud);
  return;
}


void PointTracker::CloudCBInternal(const PointCloudPtr& input_cloud)
{

  // Downsample point cloud
  printf("Filtering cloud\n");
  PointCloudPtr pcl_cloud = DownsampleCloud(input_cloud, 0.01, 0.01, 0.01);

  if (!vis_.updatePointCloud(pcl_cloud, "input_cloud"))
  {
    vis_.addPointCloud(pcl_cloud, "input_cloud");
  }
  // Compute and visualize surface normals
  printf("Computing normals\n");
  SurfaceNormalsPtr normals = EstimateSurfaceNormals(pcl_cloud, 0.01);
  //vis_.addPointCloudNormals<PointT, NormalT> (pcl_cloud, normals, 4, 0.02, "normals");

  // Compute keypoints
  printf("Computing keypoints\n");
  PointCloudPtr keypoints = DetectSIFTKeypoints(pcl_cloud, normals, 0.01, 2, 2, 1.0);
  //PointCloudPtr keypoints = DetectISS3DKeypoints(pcl_cloud);
  printf("Computed keypoints\n");
  pcl::visualization::PointCloudColorHandlerCustom<PointT> red (keypoints, 255, 0, 0);
  if (!vis_.updatePointCloud(keypoints, red, "keypoints"))
  {
    vis_.addPointCloud(keypoints, red, "keypoints");
  }
  vis_.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "keypoints");

  // Compute local descriptors at keypoints
  printf("Computing local descriptors\n");
  LocalDescriptorsPtr local_descriptors = ComputeLocalDescriptors(pcl_cloud, 
      normals, keypoints, kFeatureRadius);

  //*recent_local_descriptors_ = *local_descriptors;
  //*recent_keypoints_ = *keypoints;
  if (recent_keypoints_->width != 0)
  {
    // Find correspondences with last observed set of keypoints, in both directions.
    printf("Computing correspondences\n");
    vector<int> source2target, target2source;
    FindCorrespondences(local_descriptors, recent_local_descriptors_, &source2target);
    FindCorrespondences(recent_local_descriptors_, local_descriptors, &target2source);

    // Filter correspondences for robust matching
    printf("Filtering correspondences\n");
    pcl::CorrespondencesPtr pcl_correspondences(new pcl::Correspondences);
    FilterCorrespondences(source2target, target2source, keypoints, recent_keypoints_, pcl_correspondences);

    printf("Total features: %d\n", keypoints->width);
    printf("Number of correspondences: %d\n", int(pcl_correspondences->size()));


    for (size_t ii = 0; ii < target2source.size(); ++ii)
    {
      const int idx = target2source[ii];
      geometry_msgs::Point p;
      p.x = keypoints->points[idx].x;
      p.y = keypoints->points[idx].y;
      p.z = keypoints->points[idx].z;
      tracked_points_[ii].push_back(p);
    }
  }
  // First set of keypoints.
  else
  {
    tracked_points_.resize(keypoints->width);

    for (size_t ii = 0; ii < keypoints->width; ++ii)
    {
      geometry_msgs::Point p;
      p.x = keypoints->points[ii].x;
      p.y = keypoints->points[ii].y;
      p.z = keypoints->points[ii].z;
      tracked_points_[ii].push_back(p);
    }
  }


  // Update recent keypoints and descriptors
  *recent_keypoints_ = *keypoints;
  *recent_local_descriptors_ = *local_descriptors;

  printf("Done\n");
  LearnModel();


  return;
}

PointCloudPtr PointTracker::DownsampleCloud(const PointCloudPtr& input, float l_x, float l_y, float l_z)
{
  PointCloudPtr downsampled_cloud(new PointCloud);
  pcl::VoxelGrid<PointT> voxel_grid;
  voxel_grid.setInputCloud (input);
  voxel_grid.setLeafSize (l_x, l_y, l_z);
  voxel_grid.filter (*downsampled_cloud);
  return downsampled_cloud;
}

SurfaceNormalsPtr PointTracker::EstimateSurfaceNormals(const PointCloudPtr& input, float radius)
{
  pcl::NormalEstimation<PointT, NormalT> normal_estimation;
  normal_estimation.setSearchMethod (KdTreePtr (new KdTree()));
  normal_estimation.setRadiusSearch (radius);
  normal_estimation.setInputCloud (input);
  SurfaceNormalsPtr normals (new SurfaceNormals);
  normal_estimation.compute (*normals);
  return normals;
}

PointCloudPtr PointTracker::DetectSIFTKeypoints(const PointCloudPtr& points,
    const SurfaceNormalsPtr & normals, 
    float min_scale, int nr_octaves,
    int nr_scales_per_octave, float min_contrast)
{
  pcl::SIFTKeypoint<PointT, pcl::PointWithScale> sift_detect;
  sift_detect.setSearchMethod (KdTreePtr (new KdTree()));
  sift_detect.setScales (min_scale, nr_octaves, nr_scales_per_octave);
  sift_detect.setMinimumContrast (min_contrast);
  sift_detect.setInputCloud (points);
  pcl::PointCloud<pcl::PointWithScale> keypoints_temp;
  sift_detect.compute (keypoints_temp);
  PointCloudPtr keypoints (new PointCloud);
  pcl::copyPointCloud (keypoints_temp, *keypoints);
  return keypoints;
}

/*
   PointCloudPtr PointTracker::DetectISS3DKeypoints(const PointCloudPtr& points)
   {
   pcl::ISSKeypoint3D<PointT, PointT> iss_detector;

   double res = 0.01;
   iss_detector.setSalientRadius (6 * res);
   iss_detector.setNonMaxRadius (4 * res);
   iss_detector.setThreshold21 (0.975);
   iss_detector.setThreshold32 (0.975);
   iss_detector.setMinNeighbors (5);
   iss_detector.setNumberOfThreads (1);
   iss_detector.setInputCloud (points);
   PointCloudPtr keypoints (new PointCloud);
   iss_detector.compute ((*keypoints));
   return keypoints;
   }
   */
LocalDescriptorsPtr PointTracker::ComputeLocalDescriptors(
    const PointCloudPtr& points, const SurfaceNormalsPtr & normals,
    const PointCloudPtr & keypoints, float feature_radius)
{
  pcl::FPFHEstimation<PointT, NormalT, LocalDescriptorT> fpfh_estimation;
  fpfh_estimation.setSearchMethod (KdTreePtr (new KdTree()));
  fpfh_estimation.setRadiusSearch (feature_radius);
  fpfh_estimation.setSearchSurface (points);  
  fpfh_estimation.setInputNormals (normals);
  fpfh_estimation.setInputCloud (keypoints);
  LocalDescriptorsPtr local_descriptors (new LocalDescriptors);
  fpfh_estimation.compute (*local_descriptors);

  return local_descriptors;
}

void PointTracker::FindCorrespondences (LocalDescriptorsPtr source, LocalDescriptorsPtr target, std::vector<int> *correspondences)
{
  correspondences->resize (source->size());

  // Use a KdTree to search for the nearest matches in feature space
  pcl::KdTreeFLANN<LocalDescriptorT> descriptor_kdtree;
  descriptor_kdtree.setInputCloud(target);

  // Find the index of the best match for each keypoint, and store it in "correspondences_out"
  const int k = 1;
  std::vector<int> k_indices (k);
  std::vector<float> k_squared_distances (k);
  for (size_t i = 0; i < source->size (); ++i)
  {
    descriptor_kdtree.nearestKSearch (*source, i, k, k_indices, k_squared_distances);
    (*correspondences)[i] = k_indices[0];
  }
  return;
}

void PointTracker::FilterCorrespondences(const vector<int>& source2target, const vector<int>& target2source, const PointCloudPtr& source_keypoints, const PointCloudPtr& target_keypoints, pcl::CorrespondencesPtr pcl_correspondences)
{
  std::vector<pair<unsigned, unsigned> > correspondences;
  for (unsigned c_idx = 0; c_idx < source2target.size (); ++c_idx)
    if (target2source[source2target[c_idx]] == int(c_idx))
      correspondences.push_back(std::make_pair(c_idx, source2target[c_idx]));

  pcl_correspondences->resize (correspondences.size());
  for (unsigned c_idx = 0; c_idx < correspondences.size(); ++c_idx)
  {
    (*pcl_correspondences)[c_idx].index_query = correspondences[c_idx].first;
    (*pcl_correspondences)[c_idx].index_match = correspondences[c_idx].second;
  }

  // TODO: Following code rejects correspondences by computing a 4x4 homogeneous transform and discarding outliers. Do not use when objects being tracked are articulated.
  /*
     pcl::registration::CorrespondenceRejectorSampleConsensus<PointT> rejector;
     rejector.setInputCloud(source_keypoints);
     rejector.setTargetCloud(target_keypoints);
     rejector.setInputCorrespondences(pcl_correspondences);
     rejector.getCorrespondences(*pcl_correspondences);
     */
  return;
}

void PointTracker::LearnModel()
{
  // Create point cloud for first frame.
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  cloud->width = tracked_points_.size();
  cloud->height = 1;
  cloud->points.resize(cloud->width * cloud->height);
  for (size_t ii = 0; ii < tracked_points_.size(); ++ii)
  {
    cloud->points[ii].x = tracked_points_[ii][0].x;
    cloud->points[ii].y = tracked_points_[ii][0].y;
    cloud->points[ii].z = tracked_points_[ii][0].z;
  }
  knn_indices_.resize(tracked_points_.size());

  // Get nearest neighbors for each tracked point, in the first frame.
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud (cloud);

  for (size_t ii = 0; ii < tracked_points_.size(); ++ii)
  {
    vector<int> idxs;
    vector<float> distances;
    //kdtree.radiusSearch(cloud->points[ii], kKNNSearchRadius, idxs, distances);
    kdtree.nearestKSearch(cloud->points[ii], kKNNSearchK, idxs, distances);
    for (size_t jj = 0; jj < idxs.size(); ++jj)
    {
      // TODO: Do distance checking if using knn search instead of radius search.
      knn_indices_[ii].push_back(idxs[jj]);
    }
  }
  return;
}

// Debug

void PointTracker::PrintEdges()
{
  printf("KNN Neigbors:\n");
  for (size_t ii = 0; ii < knn_indices_.size(); ++ii)
  {
    printf("%d: ", ii);
    for (size_t jj = 0; jj < knn_indices_[ii].size(); ++jj)
    {
      printf("%d ", knn_indices_[ii][jj]);
    }
    printf("\n");
  }
  return;
}
