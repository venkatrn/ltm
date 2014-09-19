/** 
 * @file d_model_learner.h
 * @author Venkatraman Narayanan
 * Carnegie Mellon University, 2014
 */
#ifndef _LTM_DMODEL_LEARNER_H_
#define _LTM_DMODEL_LEARNER_H_

#include <ltm/d_model_bank.h>
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

    void SetModelBank(DModelBank* const model_bank);
    void AddGraspIdx(int grasp_idx);

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

    /**@brief Compute edges for a point cloud**/
    void ComputeEdges(const geometry_msgs::PoseArray& points, std::vector<Edge>* edges);

  private:
    int num_hypotheses_; ///< Number of candidate models 
    int grasp_idx_; ///< Allow grasp_idx to vary for observations
    double del_t_; ///< Timestep for forward simulation of candidate models 
    std::string reference_frame_;
    LTMViz* viz_;
    /// Candidate D-models (DModelBank)
    DModelBank* candidate_model_bank_;
    /// Compute squared error between two point clouds
    double ComputeSquaredError(const geometry_msgs::PoseArray pts1, const geometry_msgs::PoseArray pts2);

    /// Utility to read observation-action sequence from text file
    void ReadObservationsFromFile(const std::string obs_file, std::vector<geometry_msgs::PoseArray>* observations, std::vector<tf::Vector3>* forces);
    void ReadObservationsFromFile(const std::string obs_file, std::vector<geometry_msgs::PoseArray>* observations);

};


#endif /*_LTM_DMODEL_LEARNER_H_ */
