/** 
 * @file d_model_bank.h
 * @brief Generalizes the d_model class to a collection/database of d_models, which ideally
 * come from prior demonstrations/training data
 * @author Venkatraman Narayanan
 * Carnegie Mellon University, 2014
 */
#ifndef _LTM_DMODEL_BANK_H_
#define _LTM_DMODEL_BANK_H_

#define DISALLOW_COPY_AND_ASSIGN(TypeName) \
  TypeName(const TypeName&);             \
void operator=(const TypeName&)

#include <ltm/abstract_model_bank.h>
#include <ltm/d_model_structs.h>
#include <ltm/d_model_utils.h>
#include <ltm/ltm_viz.h>

#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/Marker.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>

#include <unordered_map>
#include <vector>

// This is the internal state (set of points) for each individual d_model
struct State_t
{
  int grasp_idx; // Index of the point at which force is applied
  std::vector<int> changed_inds; // Indices of points that have changed
  geometry_msgs::PoseArray changed_points; // Coordinates of corresponding indices
  bool operator==(const State_t& s) const
  {
    // States are different if they have different grasp idxs. Otherwise, need to check position
    // of points.
    if (s.grasp_idx != grasp_idx)
    {
      return false;
    }

    if (changed_inds.size() != s.changed_inds.size())
    {
      return false;
    }

    for (size_t ii = 0; ii < changed_inds.size(); ++ii)
    {
      auto it = std::find(s.changed_inds.begin(), s.changed_inds.end(), changed_inds[ii]);
      if (it == s.changed_inds.end())
      {
        return false;
      }
      const int offset = distance(s.changed_inds.begin(), it);

      /** DEBUG
        printf("%f %f %f\n", changed_points.poses[ii].position.x, changed_points.poses[ii].position.y, 
        changed_points.poses[ii].position.z);
        printf("%f %f %f\n", s.changed_points.poses[offset].position.x, s.changed_points.poses[offset].position.y,
        s.changed_points.poses[offset].position.z);
       **/

      /**
      if (fabs(changed_points.poses[ii].position.x-s.changed_points.poses[offset].position.x) >= kFPTolerance ||
          fabs(changed_points.poses[ii].position.y-s.changed_points.poses[offset].position.y) >= kFPTolerance ||
          fabs(changed_points.poses[ii].position.z-s.changed_points.poses[offset].position.z) >= kFPTolerance)
      {
        return false;
      }
      **/
      if (!PosesEqual(changed_points.poses[ii], s.changed_points.poses[offset]))
      {
        return false;
      }
      //TODO(venkat): Check orientations also
    }
    return true;
  }
};

struct BeliefState_t
{
  int internal_state_id; //Internal state id (set of points)
  std::vector<double> belief; // Probability distribution over models
  bool operator==(const BeliefState_t& s) const
  {
    // States are different if they have different internal state_ids and/or different belief probabilities
    if (s.internal_state_id != internal_state_id)
    {
      return false;
    }

    if (belief.size() != s.belief.size())
    {
      return false;
    }
    for (size_t ii = 0; ii < belief.size(); ++ii)
    {
      if (fabs(belief[ii]-s.belief[ii]) >= kFPTolerance)
      {
        return false;
      }
    }
    return true;
  }
  BeliefState_t()
  {
    internal_state_id = -1;
    belief.clear();
  }
};

struct EnvCfg_t
{
  int start_state_id;
  int goal_state_id;

  int internal_start_state_id;
  int internal_goal_state_id;

  // Cache the goal poses for computing heuristics
  geometry_msgs::PoseArray goal_grasp_poses;

  // Timestep for quasi-static forward simulation that gives the next state of the model.
  // const double kSimTimeStep = 0.1;
  double sim_time_step;
};


class DModelBank : public AbstractModelBank
{
  public:
    /**@brief Constructor**/
    explicit DModelBank(const std::string& reference_frame);
    // TODO: This constructor is deprecated until I get the gcc 4.7 compiler to work
    DModelBank();

    /**@brief Destructor**/
    ~DModelBank();

    /**@brief Initialize the D-ModelBank from file. Refer to example file for the format
     * @param shift_x Offset along x-axis
     * @param shift_y Offset along y-axis
     * @param shift_z Offset along z-axis
     **/
    void InitFromFile(std::vector<std::string> dmodel_files, std::vector<double> shift_x, std::vector<double> shift_y, std::vector<double> shift_z);
    void InitFromFile(std::vector<std::string> dmodel_files);
    /**@brief Initialize the D-ModelBank directly from edges and edge params (typically computed by a learner from
     * AR marker trackings)
     **/
    void InitFromObs(std::vector<Edge> edges, std::vector<std::vector<EdgeParams>> edge_params);

    /**@brief Read the force primitives from file.**/
    void InitForcePrimsFromFile(const char* fprims_file);

    /**@brief Initialize points (only) directly**/
    void SetPoints(const geometry_msgs::PoseArray& points);

    void AddEdge(int model_id, Edge e, EdgeParams e_params);
    void AddPoint(geometry_msgs::Pose p);
    /**@brief Returns the index of the closest/attached point**/
    int AddGraspPoint(geometry_msgs::Pose p);

    /**@brief Simulate a plan by applying a sequence of forces.
     * Uses the already set force index and simulation timestep
     **/
    void SimulatePlan(int model_id, const std::vector<tf::Vector3>& forces, const std::vector<int>& grasp_points);
    void SimulatePlan(int model_id, const std::vector<int>& fprim_ids); 

    /**@brief Get observation probabilities, given two internal states, and a sequence of fprims executed**/
    void GetObservationProbabilities(const State_t& s_0, const State_t& s_1, const std::vector<int>& fprim_ids, std::vector<double>* obs_probs);

    /**@brief Visualize belief state**/
    void VisualizeState(int model_id, int belief_state_id);

    /**@brief Debug utilities**/
    void PrintPoints();
    void PrintEdges(int model_id);

    /**@brief TF publisher for points**/
    void TFTimedCallback(const ros::TimerEvent& event);

    /**@brief TF publisher**/
    void TFCallback(geometry_msgs::PoseArray);

    /**@brief Accessors and Mutators**/
    const geometry_msgs::PoseArray& GetDModelPoints()
    {
      return points_;
    }

    /**@brief Return successor states**/
    void GetSuccs(int source_state_id, std::vector<std::vector<int>>* succ_state_ids_map, 
        std::vector<std::vector<double>>* succ_state_probabilities_map, 
        std::vector<int>* action_ids, std::vector<double>* action_costs);
    /**@brief Check belief state goal**/
    bool IsGoalState(int state_id);
    /**@brief Returns true if internal state satisfies partial goal**/
    bool IsInternalGoalState(State_t state);
    /**@brief Return heuristic for a state**/
    double GetGoalHeuristic(int belief_state_id);

    /**@brief Return the forward simulated state of the d_model for the applied force.**/
    void GetNextState(int model_id, const geometry_msgs::PoseArray& in_points,
        int p_idx, tf::Vector3 force, double del_t,
        geometry_msgs::PoseArray *out_points);
    /**@brief Set the indices of the point where the forces can be applied**/
    void AddGraspIdx(int grasp_idx);
    /**@brief Set the start state.**/
    void SetStartState(BeliefState_t start_belief_state); 
    void SetInternalStartState(State_t start_state);
    /**@brief Set the goal state (can be partially defined).
     * The search will terminate when a state is found that satisfies the partial goal,
     * hence it is not necessary that the final goal state ID should be the same as the initialized
     * goal state ID.
     **/
    void SetGoalState(BeliefState_t goal_state);
    void SetInternalGoalState(State_t goal_state);
    /**@brief Return the start state ID**/
    int GetStartStateID();
    int GetInternalStartStateID();
    /**@brief Set the timestep for quasi-static simulation used in computing successors**/
    void SetSimTimeStep(double del_t);

    /**@brief Convert force primitive IDs to a sequence of forces**/
    bool ConvertForcePrimIDsToForcePrims(const std::vector<int>& fprim_ids, std::vector<tf::Vector3>* forces, 
        std::vector<int>* grasp_points);

    /**@brief Obtain end-effector poses from planner solution**/
    bool GetEndEffectorTrajFromStateIDs(const std::vector<int>& state_ids, geometry_msgs::PoseArray* traj);

    /**@brief Reset the state mappings**/
    void ResetStateMap();

    /**@brief Transformations**/
    tf::StampedTransform GetTransform(std::string& from_frame, std::string& to_frame);
    geometry_msgs::Pose TransformPose(const geometry_msgs::Pose& in_pose, std::string& from_frame, std::string& to_frame);
    tf::Vector3 TransformVector(const tf::Vector3& in_vec, std::string& from_frame, std::string& to_frame);
    tf::Point TransformPoint(const tf::Point& in_point, std::string& from_frame, std::string& to_frame);

    /**@brief Compute the changed inds and points for poses in the world frame, with respect to the inital poses set for initialization**/
    void GetChangedStateFromWorldPoses(const geometry_msgs::PoseArray& world_poses, std::vector<int>* changed_inds, geometry_msgs::PoseArray* changed_points);
    void GetWorldPosesFromState(const State_t& state, geometry_msgs::PoseArray* world_poses);

    /**@brief Accessors**/
    int num_models() const {return num_models_;}
    double sim_time_step() const {return env_cfg_.sim_time_step;}
    // TODO: this is arbitrary
    int grasp_idx() const {return (grasp_idxs_.size() == 0) ? -1 : grasp_idxs_[0];}

  private:
    std::string reference_frame_;
    geometry_msgs::PoseArray points_;
    int num_models_;
    /**@brief Mapping from (i,j) to edge(i,j)**/
    std::vector<EdgeMap*> edge_maps_;
    /**@brief Adjacency list, one for each model**/
    std::vector<std::vector<std::vector<int>>> adj_lists_;

    /**@brief Mapping from State to State ID**/
    std::unordered_map<int, State_t> StateMap;
    /**@brief Mapping from BeleifState to BeliefState ID**/
    std::unordered_map<int, BeliefState_t> BeliefStateMap;

    /**@brief DModel Visualizer**/
    LTMViz* viz_;

    tf::TransformListener listener_;

    bool visualize_dmodel_;
    bool visualize_expansions_;

    EnvCfg_t env_cfg_;
    // The force primitives that determine the successors of a state. 
    std::vector<tf::Vector3> force_primitives_;
    // Point index at which forces can be applied. 
    std::vector<int> grasp_idxs_;

    /**@brief Initialization helpers**/
    void InitEdgeMap(int num_models);

    /**@brief InitFromFile helpers**/
    void InitFromFile(int model_id, const char* dmodel_file, double shift_x, double shift_y, double shift_z);
    void InitFromFile(int model_id, const char* dmodel_file);

    /**@brief For a given point, find all points in the same rigid component and 
     * a set of separating points**/
    void ExtractIndices(int model_id,
        int p_idx,
        std::vector<int>* component_idxs,
        std::vector<int>* sep_idxs,
        JointType* joint_type,
        tf::Vector3* normal,
        tf::Vector3* center);

    /**@brief Return a set of successors for a given point*/
    void GetAdjPoints(int model_id, int p_idx, std::vector<int>* adj_points);

    /**@brief State to State ID mapping**/
    int StateToStateID(State_t& s);
    /**@brief State ID to State mapping**/
    State_t StateIDToState(int state_id);
    /**@brief BeliefState to BeliefState ID mapping**/
    int BeliefStateToStateID(BeliefState_t& s);
    /**@brief State ID to State mapping**/
    BeliefState_t BeliefStateIDToState(int belief_state_id);

    /**@brief Convert force primitive to fprim ID**/
    int FPrimToFPrimID(int grasp_idx, int force_idx);
    void FPrimIDToFPrim(int fprim_id, int* grasp_idx, int* force_idx);

    /**@brief Return true if valid state**/
    bool IsValidState(const State_t& s);

    /**@brief Return successor states for individual model**/
    void GetSuccs(int model_id, int source_state_id, std::vector<int>* succs, 
        std::vector<int>* edge_ids, std::vector<double>* costs);
    /**@brief Returns true if state ID satisfies partial goal**/
    bool IsInternalGoalState(int state_id);

    double GetInternalGoalHeuristic(int internal_state_id);

    /**@brief Visualize internal state**/
    void VisualizeInternalState(int model_id, int internal_state_id);


    DISALLOW_COPY_AND_ASSIGN(DModelBank);
};

#endif /* _LTM_DMODEL_BANK_H */
