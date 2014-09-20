/**
 * @file perception_utils.cpp
 * @author Venkatraman Narayanan
 * Carnegie Mellon University, 2014
 */

#include <ltm_perception/perception_utils.h>

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
// Filtering
#include <pcl/filters/voxel_grid.h>
// Clustering
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
// PCL Plane Segmentation
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>

#include <boost/thread/thread.hpp>

// Euclidean cluster extraction params
const int kMaxClusterSize = 10000; //20000
const int kMinClusterSize = 100; //100
const double kClusterTolerance = 0.03; //0.2

// Downsampling params
const double kVoxelLeafSize = 0.02; //0.05

// Cluster evaluation params
const double kMinHeight = 0.01;
const double kMaxHeight = 1.5;
const double kMinLength = 0.2;
const double kMaxLength = 5;
const double kMinWidth = 0.2;
const double kMaxWidth = 5;

using namespace std;

pcl::ModelCoefficients::Ptr perception_utils::GetPlaneCoefficients(PointCloudPtr cloud)
{
  PointCloudPtr cloud_temp (new PointCloud);
  cloud_temp = cloud;

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);;
  // Create the segmentation object
  pcl::SACSegmentation<PointT> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.3); //0.01
  seg.setInputCloud (cloud_temp->makeShared ());
  seg.segment (*inliers, *coefficients);

  return coefficients; 
}

pcl::ModelCoefficients::Ptr perception_utils::GetPlaneCoefficients(PointCloudPtr cloud, PointCloudPtr plane_points)
{
  PointCloudPtr cloud_temp (new PointCloud);
  cloud_temp = cloud;

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);;
  // Create the segmentation object
  pcl::SACSegmentation<PointT> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_MLESAC);
  seg.setDistanceThreshold (0.01); //0.01
  seg.setInputCloud (cloud_temp->makeShared ());
  seg.segment (*inliers, *coefficients);

  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud (cloud);
  extract.setIndices (inliers);
  extract.setNegative (false);
  extract.filter(*plane_points);

  return coefficients; 
}

pcl::ModelCoefficients::Ptr perception_utils::GetLineCoefficients(PointCloudPtr cloud, PointCloudPtr line_points)
{
  PointCloudPtr cloud_temp (new PointCloud);
  cloud_temp = cloud;

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);;
  // Create the segmentation object
  pcl::SACSegmentation<PointT> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_LINE);
  seg.setMethodType (pcl::SAC_MLESAC);
  seg.setDistanceThreshold (0.01); //0.01
  seg.setInputCloud (cloud_temp->makeShared ());
  seg.segment (*inliers, *coefficients);

  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud (cloud);
  extract.setIndices (inliers);
  extract.setNegative (false);
  extract.filter(*line_points);

  // Publish the model coefficients
  // pub_plane.publish (*coefficients);
  /*
     std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
     << coefficients->values[1] << " "
     << coefficients->values[2] << " " 
     << coefficients->values[3] << std::endl;
     */
  return coefficients; 
}

/*
void perception_utils::GetEdges(PointCloudPtr cloud)
{
  pcl::OrganizedEdgeBase <PointT, pcl::Label> edgedetector;
  pcl::PointCloud <pcl::Label> label;
  std::vector <pcl::PointIndices> label_indices;

  edgedetector.setInputCloud(cloud);
  edgedetector.setDepthDisconThreshold(0.02f);
  edgedetector.setMaxSearchNeighbors(50);
  edgedetector.compute(label, label_indices);

  std::cout << edgedetector.getEdgeType() << std::endl;
  std::cout << label_indices.size() << std::endl;
  std::cout << label_indices[0].indices.size() << std::endl;
  std::cout << label_indices[1].indices.size() << std::endl;
  std::cout << label_indices[2].indices.size() << std::endl;
  std::cout << label_indices[3].indices.size() << std::endl;
  std::cout << label_indices[4].indices.size() << std::endl;
}
*/

PointCloudPtr perception_utils::RemoveGroundPlane(PointCloudPtr cloud, pcl::ModelCoefficients::Ptr coefficients)
{
  PointCloudPtr ground_removed_pcd (new PointCloud);

  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<PointT> seg;
  // Optional
  seg.setOptimizeCoefficients (false);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  // seg.setModelType (pcl::SACMODEL_PARALLEL_PLANE);
  //seg.setAxis (Eigen::Vector3f (0.0, 0.0, 1.0));
  //seg.setEpsAngle (15*3.14/180);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.1); //0.01
  seg.setInputCloud (cloud->makeShared ());
  seg.segment (*inliers, *coefficients);
  // std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
  // << coefficients->values[1] << " "
  // << coefficients->values[2] << " " 
  // << coefficients->values[3] << std::endl;

  // Remove the planar inliers from the input cloud
  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud (cloud);
  extract.setIndices (inliers);
  extract.setNegative (true);
  extract.filter(*ground_removed_pcd);
  return ground_removed_pcd;
}

void perception_utils::DoEuclideanClustering(PointCloudPtr cloud, vector<PointCloudPtr>* cluster_clouds)
{
  PointCloudPtr cluster_pcd (new PointCloud);
  cluster_clouds->clear();

  // Creating the KdTree object for the search method of the extraction
  // pcl::KdTreeFLANN<PointT>::Ptr tree (new pcl::KdTreeFLANN<PointT>);
  // tree->setInputCloud(cloud);
  pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
  tree->setInputCloud(cloud);

  vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<PointT> ec;
  ec.setClusterTolerance(kClusterTolerance); // 2cm
  ec.setMinClusterSize(kMinClusterSize); //100
  ec.setMaxClusterSize(kMaxClusterSize);
  ec.setSearchMethod(tree);
  //ec.setSearchMethod(KdTreePtr(new KdTree()));
  ec.setInputCloud(cloud);
  printf("Extracting..\n");
  ec.extract(cluster_indices);
  printf("Extracted..\n");

  int j = 0;
  printf("Number of clusters: %d\n", cluster_indices.size());
  for (vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
  {
    PointCloudPtr cloud_cluster (new PointCloud);
    for (vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      cloud_cluster->points.push_back(cloud->points[*pit]); 

    cloud_cluster->width = cloud_cluster->points.size();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;
    cloud_cluster->sensor_origin_ = cloud->sensor_origin_;
    cloud_cluster->sensor_orientation_ = cloud->sensor_orientation_;
    //printf("Size: %d\n",(int)cloud_cluster->points.size ());


    // Publish the cluster marker
    bool valid_cluster = perception_utils::EvaluateCluster(cloud_cluster);
    if(valid_cluster)
    {
      /*
         float r = 1, g = 0, b = 0;
         std::string ns = "base_link";
         publishClusterMarker(cloud_cluster,ns,1,r,g,b);

      // Publish the data
      sensor_msgs::PointCloud2 output_cloud; 
      pcl::toROSMsg(*cloud_cluster,output_cloud);
      output_cloud.header.frame_id = "openni_depth_optical_frame";
      pub_cluster.publish (output_cloud);
      */
      cluster_clouds->push_back(cloud_cluster);
      //return cloud_cluster;
    }
    j++;
  }
  //return cluster_pcd;
}

PointCloudPtr perception_utils::DownsamplePointCloud(PointCloudPtr cloud)
{
  // Perform the actual filtering
  PointCloudPtr downsampled_cloud(new PointCloud);
  pcl::VoxelGrid<PointT> sor;
  sor.setInputCloud(cloud);
  sor.setLeafSize(kVoxelLeafSize,kVoxelLeafSize,kVoxelLeafSize);
  sor.filter(*downsampled_cloud);
  return downsampled_cloud;
}

void perception_utils::DrawOrientedBoundingBox(pcl::visualization::PCLVisualizer&viewer, PointCloudPtr cloud, string box_id)
{
  // compute principal direction
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*cloud, centroid);
  Eigen::Matrix3f covariance;
  computeCovarianceMatrixNormalized(*cloud, centroid, covariance);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
  Eigen::Matrix3f eigDx = eigen_solver.eigenvectors();
  eigDx.col(2) = eigDx.col(0).cross(eigDx.col(1));

  // move the points to the that reference frame
  Eigen::Matrix4f p2w(Eigen::Matrix4f::Identity());
  p2w.block<3,3>(0,0) = eigDx.transpose();
  p2w.block<3,1>(0,3) = -1.f * (p2w.block<3,3>(0,0) * centroid.head<3>());
  pcl::PointCloud<PointT> cPoints;
  pcl::transformPointCloud(*cloud, cPoints, p2w);

  PointT min_pt, max_pt;
  pcl::getMinMax3D(cPoints, min_pt, max_pt);
  const Eigen::Vector3f mean_diag = 0.5f*(max_pt.getVector3fMap() + min_pt.getVector3fMap());

  // final transform
  const Eigen::Quaternionf qfinal(eigDx);
  const Eigen::Vector3f tfinal = eigDx*mean_diag + centroid.head<3>();

  // draw the cloud and the box
  viewer.addCube(tfinal, qfinal, max_pt.x - min_pt.x, max_pt.y - min_pt.y, max_pt.z - min_pt.z, box_id);
}

void perception_utils::DrawAxisAlignedBoundingBox(pcl::visualization::PCLVisualizer&viewer, PointCloudPtr cloud, string box_id)
{
  // compute principal direction
  Eigen::Vector4f min, max;
  pcl::getMinMax3D (*cloud, min, max);
  // draw the cloud and the box
  viewer.addCube(min[0], max[0], min[1], max[1], min[2], max[2], 1.0, 0.0, 0.0, box_id);
}

bool perception_utils::EvaluateCluster(PointCloudPtr cloud_cluster)
{
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*cloud_cluster, centroid);
  Eigen::Vector4f min, max;
  pcl::getMinMax3D (*cloud_cluster, min, max);
  const double length = max[0] - min[0];
  const double width = max[1] - min[1];
  const double height = max[2] - min[2];
  if(height < kMinHeight || height > kMaxHeight
      || width < kMinWidth || width > kMaxWidth
      || length < kMinLength || length > kMaxLength)
  {
    return false;
  }

  if (fabs(centroid[0]) > 2.0 || fabs(centroid[1] > 2.0))
  {
    return false;
  }

  return true;
}


