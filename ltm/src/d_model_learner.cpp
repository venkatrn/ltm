/** 
 * @file d_model_learner.cpp
 * @author Venkatraman Narayanan
 * Carnegie Mellon University, 2014
 */

#include <ltm/d_model_learner.h>
#include <ros/console.h>
#include <cassert>
#include <string>

using namespace std;

DModelLearner::DModelLearner()
{
  num_hypotheses_ = 0;
}

DModelLearner::~DModelLearner()
{
  for (int ii = 0; ii < num_hypotheses_; ++ii)
  {
    if (candidate_models_[ii] != NULL)
    {
      delete candidate_models_[ii];
    }
  }
  candidate_models_.clear();
}

void DModelLearner::InitializeCandidateModels(const std::vector<std::string> dmodel_files, const std::string fprims_file, int grasp_idx, double del_t)
{
  // Models should be initialized before any of the other methods are called
  InitializeCandidateModels(dmodel_files);
  InitializeForcePrimsFromFile(fprims_file);
  AddGraspIdx(grasp_idx);
  SetSimTimeStep(del_t);
}

void DModelLearner::InitializeCandidateModels(const vector<std::string> dmodel_files)
{
  num_hypotheses_ = int(dmodel_files.size());
  for (int ii = 0; ii < num_hypotheses_; ++ii)
  {
    candidate_models_.push_back(new DModel("/base_link"));
    candidate_models_[ii]->InitFromFile(dmodel_files[ii].c_str());
  }
}

void DModelLearner::InitializeForcePrimsFromFile(const std::string fprims_file)
{
  for (int ii = 0; ii < num_hypotheses_; ++ii)
  {
    candidate_models_[ii]->InitForcePrimsFromFile(fprims_file.c_str());
  }
}

void DModelLearner::AddGraspIdx(int grasp_idx)
{
  grasp_idx_ = grasp_idx;
}

void DModelLearner::SetSimTimeStep(double del_t)
{
  del_t_ = del_t;
}

void DModelLearner::LearnTransitions(const std::string obs_file, std::vector<Transition>* transitions)
{
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
      candidate_models_[jj]->GetNextState(observations[ii], grasp_idx_, forces[ii], del_t_, &simulated_points);
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
      ROS_ERROR("Error reading forces in observations file");
      return;
    }
    tf::Vector3 force(x, y, z);
    forces->push_back(force);
  } 
  fclose(f_obs);
}
