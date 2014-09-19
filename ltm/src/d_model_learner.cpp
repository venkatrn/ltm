/** 
 * @file d_model_learner.cpp
 * @author Venkatraman Narayanan
 * Carnegie Mellon University, 2014
 */

#include <ltm/d_model_learner.h>
#include <ros/console.h>
#include <cassert>
#include <string>

#include <pcl/kdtree/kdtree_flann.h>

#include <articulation_models/models/factory.h>
#include <articulation_msgs/ModelMsg.h>
#include <articulation_msgs/TrackMsg.h>
#include <articulation_msgs/ParamMsg.h>

using namespace std;
using namespace articulation_models;
using namespace articulation_msgs;

DModelLearner::DModelLearner(const string& reference_frame)
{
  num_hypotheses_ = 0;
  grasp_idx_ = -1;
  reference_frame_ = reference_frame;
  viz_ = new LTMViz("d_model_viz");
  viz_->SetReferenceFrame(reference_frame_);
}

DModelLearner::~DModelLearner()
{
  // DModelLearner does not own the model_bank
}

void DModelLearner::SetModelBank(DModelBank* const model_bank)
{
  candidate_model_bank_ = model_bank;
  num_hypotheses_ = model_bank->num_models();
  del_t_ = model_bank->sim_time_step();
}

void DModelLearner::AddGraspIdx(int grasp_idx)
{
  grasp_idx_ = grasp_idx;
}

void DModelLearner::PlaybackObservations(const std::string obs_file)
{
  vector<geometry_msgs::PoseArray> observations;
  vector<tf::Vector3> forces;
  ReadObservationsFromFile(obs_file, &observations, &forces);
  PlaybackObservations(observations, forces);
}

void DModelLearner::PlaybackObservations(const std::vector<geometry_msgs::PoseArray>& observations, const std::vector<tf::Vector3>& forces)
{
  const int num_observations = observations.size();
  if (num_observations <= 1)
  {
    return;
  }
  const bool forces_available = (observations.size() == (forces.size() + 1)) && (grasp_idx_ != -1);
  for (int ii = 0; ii < num_observations - 1; ++ii)
  {
    viz_->VisualizePoints(observations[ii]);
    if (forces_available)
    {
    viz_->VisualizeForcePrim(forces[ii], observations[ii].poses[grasp_idx_]);
    }
    usleep(100000);
  }
}

void DModelLearner::LearnTransitions(const std::string obs_file, std::vector<Transition>* transitions)
{
  assert(grasp_idx_ != -1);
  vector<geometry_msgs::PoseArray> observations;
  vector<tf::Vector3> forces;
  ReadObservationsFromFile(obs_file, &observations, &forces);
  LearnTransitions(observations, forces, transitions);
}

void DModelLearner::LearnTransitions(const vector<geometry_msgs::PoseArray>& observations, const vector<tf::Vector3> forces, vector<Transition>* transitions)
{
  transitions->clear();
  // Num actions should be one less than the number of observations
  assert(observations.size() == forces.size() + 1);
  const int num_observations = observations.size();
  if (num_observations <= 1)
  {
    return;
  }

  // Playback the observations and forces
  PlaybackObservations(observations, forces);

  /**DEBUG
  for (int ii = 0; ii < num_observations; ++ii)
  {
    for (int jj = 0; jj < observations[ii].poses.size(); ++jj)
    {
      printf("%f %f %f\n", observations[ii].poses[jj].position.x, observations[ii].poses[jj].position.y, observations[ii].poses[jj].position.z);
      }
    printf("\n");
  }
  **/

  vector<int> latent_variables;
  latent_variables.resize(num_observations);
  
  vector<double> model_errors;
  vector<double> model_probabilities;
  for (int ii = 0; ii < num_observations - 1; ++ii)
  {

    // Compute the squared error for each candidate model by forward simulating the points
    double max_error = 0;
    model_errors.clear();
    model_errors.resize(num_hypotheses_);
    for (int jj = 0; jj < num_hypotheses_; ++jj)
    {
      geometry_msgs::PoseArray simulated_points;
      candidate_model_bank_->GetNextState(jj, observations[ii], grasp_idx_, forces[ii], del_t_, &simulated_points);
      model_errors[jj] = ComputeSquaredError(observations[ii + 1], simulated_points);
      ROS_DEBUG("Model %d, error: %f\n", jj, model_errors[jj]);
      max_error = max(model_errors[jj], max_error);
    }

    // Convert the squared errors to probabilities and pick the maximizer
    model_probabilities.clear();
    model_probabilities.resize(num_hypotheses_);
    double max_prob = 0;
    int best_model_idx = 0;
    for (int jj = 0; jj < num_hypotheses_; ++jj)
    {
      model_probabilities[jj] = 1 - (model_errors[jj] / max_error);
      if (model_probabilities[jj] > max_prob)
      {
        max_prob = model_probabilities[jj];
        best_model_idx = jj;
      }
    }
    latent_variables[ii] = best_model_idx;
  }
  // Assume last observation takes the same latent variable as penultimate one
  latent_variables[num_observations - 1] = latent_variables[num_observations - 2];
  
  // DEBUG
  string inferred_variables = "";
  for (int ii = 0; ii < num_observations; ++ii)
  {
    inferred_variables += to_string(latent_variables[ii]) + " ";
  }
  ROS_INFO("Inferred latent variables: %s\n", inferred_variables.c_str());

  // Determine the latent variable transitions
  for (int ii = 0; ii < num_observations - 1; ++ii)
  {
    if (latent_variables[ii] != latent_variables[ii+1])
    {
      Transition transition;
      transition.latent_in = latent_variables[ii];
      transition.latent_out = latent_variables[ii+1];
      transition.action = forces[ii];
      transition.points_in = observations[ii];
      transitions->push_back(transition);
    }
  }
  return;
}

void DModelLearner::LearnPrior(const std::string obs_file)
{
  vector<geometry_msgs::PoseArray> observations;
  vector<tf::Vector3> forces;
  ReadObservationsFromFile(obs_file, &observations, &forces);
  LearnPrior(observations);
}

void DModelLearner::LearnPrior(const std::vector<geometry_msgs::PoseArray>& observations)
{
  assert(grasp_idx_ != -1);
  geometry_msgs::PoseArray end_eff_traj;
  const int num_observations = observations.size();
  if (num_observations <= 1)
  {
    ROS_WARN("[DModel Learner]: Not enough observations to compute a prior"); 
    return;
  }
  for (int ii = 0; ii < num_observations; ++ii)
  {
    end_eff_traj.poses.push_back(observations[ii].poses[grasp_idx_]);
  }
  viz_->VisualizeTraj(end_eff_traj);
  LearnPrior(end_eff_traj);
}

void DModelLearner::LearnPrior(const geometry_msgs::PoseArray& end_eff_traj)
{
  const int num_poses = int(end_eff_traj.poses.size());
  if (num_poses <= 1)
  {
    ROS_WARN("[DModel Learner]: Not enough poses in end-effector trajectory to compute a prior"); 
    return;
  }

  MultiModelFactory factory;
  ModelMsg model_msg;
  model_msg.name = "rotational";
  ParamMsg sigma_param;
  sigma_param.name = "sigma_position";
  sigma_param.value = 0.02;
  sigma_param.type = ParamMsg::PRIOR;
  model_msg.params.push_back(sigma_param);

  model_msg.track.header.stamp = ros::Time();
  model_msg.track.header.frame_id = reference_frame_;

  model_msg.track.pose = end_eff_traj.poses;
  /*
  for (int ii = 0; ii < num_poses; ++ii) {
    model_msg.track.pose.push_back(end_eff_traj.poses[ii]);
  }
  */
  GenericModelPtr model_instance = factory.restoreModel(model_msg);
  model_instance->fitModel();
  model_instance->evaluateModel();
  string model_class = model_instance->getModelName();
  double model_llh = model_instance->getLogLikelihood(true);

  ROS_INFO("Model Class: %s", model_class.c_str());
  ROS_INFO("Model Log LH: %f", model_llh);
  if (model_class.compare("rotational") == 0)
  {
    double rad = model_instance->getParam("rot_radius");
    double c_x = model_instance->getParam("rot_center.x");
    double c_y = model_instance->getParam("rot_center.y");
    double c_z = model_instance->getParam("rot_center.z");
    double o_x = model_instance->getParam("rot_axis.x");
    double o_y = model_instance->getParam("rot_axis.y");
    double o_z = model_instance->getParam("rot_axis.z");
    double o_w = model_instance->getParam("rot_axis.w");
    ROS_INFO("Center: %f %f %f, Radius: %f", c_x, c_y, c_z, rad);
    geometry_msgs::Pose axis;
    axis.position.x = c_x;
    axis.position.y = c_y;
    axis.position.z = c_z;
    axis.orientation.x = o_x;
    axis.orientation.y = o_y;
    axis.orientation.z = o_z;
    axis.orientation.w = o_w;
    viz_->VisualizeAxis(axis);
  }
  else if (model_class.compare("prismatic") == 0)
  {
  }
}

double DModelLearner::ComputeSquaredError(const geometry_msgs::PoseArray pts1, const geometry_msgs::PoseArray pts2)
{
  assert(pts1.poses.size() == pts2.poses.size());
  int num_points = pts1.poses.size();
  double sq_error = 0;
  for (int ii = 0; ii < num_points; ++ii)
  {
    sq_error += SqrDist(pts1.poses[ii].position, pts2.poses[ii].position);
  }
  return sq_error;
}

void DModelLearner::ReadObservationsFromFile(const string obs_file, vector<geometry_msgs::PoseArray>* observations, vector<tf::Vector3>* forces)
{
  observations->clear();
  forces->clear();

  FILE* f_obs = fopen(obs_file.c_str(),"r");
  int num_points, num_frames;
  float x, y, z, o_x, o_y, o_z, o_w;
  if (f_obs == NULL) 
  {
    ROS_ERROR("Unable to open observations file");
    return;
  }

  char s_temp[1024];
  if (fscanf(f_obs, "%s", s_temp) < 1)
  {
    ROS_ERROR("Error reading observations file");
  }
  if (strcmp(s_temp, "points:") !=0)
  {
    ROS_ERROR("Incorrect format for dmodel file\n");
  }
  if (fscanf(f_obs, "%s", s_temp) < 1)
  {
    ROS_ERROR("Error reading observations file");
  }
  num_points = atoi(s_temp);

  if (fscanf(f_obs, "%s", s_temp) < 1)
  {
    ROS_ERROR("Error reading observations file");
  }
  if (strcmp(s_temp, "frames:") !=0)
  {
    ROS_ERROR("Incorrect format for dmodel file\n");
  }
  if (fscanf(f_obs, "%s", s_temp) < 1)
  {
    ROS_ERROR("Error reading observations file");
  }
  num_frames = atoi(s_temp);
  ROS_DEBUG("Reading model file with %d points and %d frames\n", num_points, num_frames);

  for (int jj = 0; jj < num_frames; ++jj)
  {
    geometry_msgs::PoseArray pose_array;
    for (int ii = 0; ii < num_points; ++ii)
    {
      if (fscanf(f_obs, "%f %f %f %f %f %f %f\n", &x, &y, &z,
            &o_x, &o_y, &o_z, &o_w) != 7) 
      {
        ROS_ERROR("Error reading points in observations file\n");
        return;
      }
      geometry_msgs::Pose p;
      p.position.x = x;
      p.position.y = y;
      p.position.z = z;
      p.orientation.x = o_x;
      p.orientation.y = o_y;
      p.orientation.z = o_z;
      p.orientation.w = o_w;
      pose_array.poses.push_back(p);
    }
    observations->push_back(pose_array);
  }


  // Number of forces = Number of frames - 1
  for (int ii = 0; ii < num_frames - 1; ++ii)
  {
    if (fscanf(f_obs, "%f %f %f\n", &x, &y, &z) != 3)
    {
      ROS_WARN("Force data is missing/incorrect in observations file");
      break;
    }
    tf::Vector3 force(x, y, z);
    forces->push_back(force);
  } 
  fclose(f_obs);
}

void DModelLearner::ComputeEdges(const geometry_msgs::PoseArray& pose_array, vector<Edge>* edges)
{
  // Clear existing edges
  edges->clear();

  const double kKNNSearchRadius = 0.5;
  const int kKNNSearchK = 2;
  
  const size_t num_points = pose_array.poses.size();

  // Create point cloud for first frame.
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  cloud->width = num_points;
  cloud->height = 1;
  cloud->points.resize(cloud->width * cloud->height);
  for (size_t ii = 0; ii < num_points; ++ii)
  {
    cloud->points[ii].x = pose_array.poses[ii].position.x;
    cloud->points[ii].y = pose_array.poses[ii].position.y;
    cloud->points[ii].z = pose_array.poses[ii].position.z;
  }

  // Get nearest neighbors for each tracked point, in the first frame.
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud (cloud);

  for (size_t ii = 0; ii < num_points; ++ii)
  {
    vector<int> idxs;
    vector<float> sqr_distances;
    kdtree.radiusSearch(cloud->points[ii], kKNNSearchRadius, idxs, sqr_distances);
    // kdtree.nearestKSearch(cloud->points[ii], kKNNSearchK, idxs, distances);
    for (size_t jj = 0; jj < idxs.size(); ++jj)
    {
      // TODO: Do distance checking if using knn search instead of radius search.
      // No self loops
      if (ii == idxs[jj])
      {
        continue;
      }
      edges->push_back(make_pair(ii, idxs[jj]));
    }
  }
  return;
}
