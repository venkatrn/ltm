/** 
 * @file d_model_learner.h
 * @author Venkatraman Narayanan
 * Carnegie Mellon University, 2014
 */
#ifndef _LTM_DMODEL_LEARNER_H_
#define _LTM_DMODEL_LEARNER_H_

#include <ltm/abstract_model.h>
#include <ltm/d_model.h>
#include <ltm/d_model_utils.h>
#include <ltm/ltm_viz.h>


struct Transition
{
  int latent_in;
  int latent_out;
  tf::Vector3 action;
  geometry_msgs::PoseArray points_in;
  geometry_msgs::PoseArray points_out;
};

class DModelLearner
{
  public:
    /**@brief Constructor **/
    explicit DModelLearner(const std::string& reference_frame);

    /**@brief Destructor **/
    ~DModelLearner();

    /**@brief Initialize candidate models from files**/
    void InitializeCandidateModels(const std::vector<std::string> dmodel_files, const std::string fprims_file, int grasp_idx, double del_t);
    void InitializeCandidateModels(const std::vector<std::string> dmodel_files);

    /**@brief Initialize the force primitives **/
    void InitializeForcePrimsFromFile(const std::string fprims_file);

    /**@brief Set the grasp index for the candidate models**/
    void AddGraspIdx(int grasp_idx);

    /**@brief Set the timestep for quasi-static simulation used in computing successors**/
    void SetSimTimeStep(double del_t);

    /**@brief Learn the transitions from one static model to another
     * @param[in] observations Observed points 
     * @param[in] edges D-Model edge structure
     * @param[out] transitions vector of learnt transitions from one latent variable state to another
     **/
    void LearnTransitions(const std::vector<geometry_msgs::PoseArray>& observations, const std::vector<tf::Vector3> forces, std::vector<Transition>* transitions);
    /**@brief Ditto, but observation-action sequence is loaded from file**/
    void LearnTransitions(const std::string obs_file, std::vector<Transition>* transitions);

    /**@brief Playback observations**/
    void PlaybackObservations(const std::string obs_file);
    void PlaybackObservations(const std::vector<geometry_msgs::PoseArray>& observations, const std::vector<tf::Vector3>& forces);

    /**@brief Learn prior from observation**/
    void LearnPrior(const std::string obs_file);
    void LearnPrior(const std::vector<geometry_msgs::PoseArray>& observations);
    void LearnPrior(const geometry_msgs::PoseArray& end_eff_traj);

  private:
    int num_hypotheses_; ///< Number of candidate models 
    double del_t_; ///< Timestep for forward simulation of candidate models 
    int grasp_idx_; // TODO: This should be allowed to vary for each observation-action pair
    std::string reference_frame_;
    LTMViz* viz_;
    /// Candidate D-models. These are 'static' models--their edge parameters do not change.
    std::vector<DModel*> candidate_models_;
    /// Compute squared error between two point clouds
    double ComputeSquaredError(const geometry_msgs::PoseArray pts1, const geometry_msgs::PoseArray pts2);

    /// Utility to read observation-action sequence from text file
    void ReadObservationsFromFile(const std::string obs_file, std::vector<geometry_msgs::PoseArray>* observations, std::vector<tf::Vector3>* forces);
    void ReadObservationsFromFile(const std::string obs_file, std::vector<geometry_msgs::PoseArray>* observations);

};


#endif /*_LTM_DMODEL_LEARNER_H_ */
