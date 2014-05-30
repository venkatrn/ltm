/** 
 * @file d_model.h
 * @author Venkatraman Narayanan
 * Carnegie Mellon University, 2013
 */
#ifndef _LTM_DMODEL_H_
#define _LTM_DMODEL_H_

#define DISALLOW_COPY_AND_ASSIGN(TypeName) \
  TypeName(const TypeName&);             \
void operator=(const TypeName&)

#include <ltm/abstract_model.h>
#include <ltm/d_model_utils.h>

#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/Marker.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <unordered_map>
#include <vector>


enum JointType 
{
  FIXED = -1,
  RIGID,
  PRISMATIC, 
  REVOLUTE, 
  SPHERICAL
};

struct pair_hash {
  size_t operator() (const std::pair<int, int>& p) const
  {return p.first+1000*p.second;}
};

struct EdgeParams
{
  JointType joint;
  tf::Vector3 normal;
  double rad;

  EdgeParams()
  {
    joint = RIGID;
    rad = 0.0;
  }
  EdgeParams(JointType jt, tf::Vector3 dir, double r)
  {
    joint = jt;
    normal = dir;
    rad = r;
  }
};

typedef std::pair<int, int> Edge;
typedef std::unordered_map<Edge, EdgeParams, pair_hash> EdgeMap;

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

      if (fabs(changed_points.poses[ii].position.x-s.changed_points.poses[offset].position.x) >= kFPTolerance ||
          fabs(changed_points.poses[ii].position.y-s.changed_points.poses[offset].position.y) >= kFPTolerance ||
          fabs(changed_points.poses[ii].position.z-s.changed_points.poses[offset].position.z) >= kFPTolerance)
      {
        return false;
      }
      //TODO(venkat): Check orientations also
    }
    return true;
  }
};

struct EnvCfg_t
{
  int start_state_id;
  int goal_state_id;

  // Cache the goal poses for computing heuristics
  geometry_msgs::PoseArray goal_grasp_poses;

  // Timestep for quasi-static forward simulation that gives the next state of the model.
  // const double kSimTimeStep = 0.1;
  double sim_time_step;
};


class DModel : public AbstractModel
{
  public:
    /**@brief Constructor**/
    explicit DModel(const std::string& reference_frame);
    // TODO: This constructor is deprecated until I get the gcc 4.7 compiler to work
    DModel();

    /**@brief Destructor**/
    ~DModel();

    /**@brief Initialize the D-Model from file. Refer to example file for the format
     * @param shift_x Offset along x-axis
     * @param shift_y Offset along y-axis
     * @param shift_z Offset along z-axis
     **/
    void InitFromFile(const char* dmodel_file, double shift_x, double shift_y, double shift_z);
    void InitFromFile(const char* dmodel_file);

    /**@brief Read the force primitives from file.**/
    void InitForcePrimsFromFile(const char* fprims_file);

    /**@brief Initialize points (only) directly**/
    void SetPoints(const geometry_msgs::PoseArray& points);

    void AddEdge(Edge e, EdgeParams e_params);
    void AddPoint(geometry_msgs::Pose p);
    void AddGraspPoint(geometry_msgs::Pose p);

    /**@brief Simulate a plan by applying a sequence of forces.
     * Uses the already set force index and simulation timestep
     **/
    void SimulatePlan(const std::vector<tf::Vector3>& forces, const std::vector<int>& grasp_points);
    void SimulatePlan(const std::vector<int>& fprim_ids); 

    /**@brief Apply force at a point in the d_model**/
    void ApplyForce(int p_idx, tf::Vector3 force, double del_t);

    /**@brief Visualize a state**/
    void VisualizeState(int state_id);


    /**@brief Debug utilities**/
    void PrintPoints();
    void PrintEdges();

    /**@brief TF publisher for points**/
    void TFTimedCallback(const ros::TimerEvent& event);

    /**@brief TF publisher**/
    void TFCallback(geometry_msgs::PoseArray);

    /**@brief Accessors and Mutators**/
    const geometry_msgs::PoseArray& GetDModelPoints()
    {
      return points_;
    }

    /**@brief Return the forward simulated state of the d_model for the applied force.**/
    void GetNextState(const geometry_msgs::PoseArray& in_points,
        int p_idx, tf::Vector3 force, double del_t,
        geometry_msgs::PoseArray *out_points);
    /**@brief Set the indices of the point where the forces can be applied**/
    void AddGraspIdx(int grasp_idx);
    /**@brief Set the start state.**/
    void SetStartState(State_t start_state);
    /**@brief Set the goal state (can be partially defined).
     * The search will terminate when a state is found that satisfies the partial goal,
     * hence it is not necessary that the final goal state ID should be the same as the initialized
     * goal state ID.
     **/
    void SetGoalState(State_t goal_state);
    /**@brief Return the start state ID**/
    int GetStartStateID();
    /**@brief Set the timestep for quasi-static simulation used in computing successors**/
    void SetSimTimeStep(double del_t);

    /**@brief Convert force primitive IDs to a sequence of forces**/
    bool ConvertForcePrimIDsToForcePrims(const std::vector<int>& fprim_ids, std::vector<tf::Vector3>* forces, 
        std::vector<int>* grasp_points);

    /**@brief Obtain end-effector poses from planner solution**/
    bool GetEndEffectorTrajFromStateIDs(const std::vector<int>& state_ids, geometry_msgs::PoseArray* traj);

    /**@brief Methods for learning D-model parameters from observations**/
    void LearnDModelParameters(const std::vector<geometry_msgs::PoseArray>& observations,
        const std::vector<Edge>& edges);
    void LearnDModelParametersExperimental(const std::vector<geometry_msgs::PoseArray>& observations,
        const std::vector<Edge>& edges);

    /**@brief Reset the state mappings**/
    void ResetStateMap();

    /**@brief For Mannequin**/
    State_t GetStateFromStateID(int state_id);

  private:
    ros::NodeHandle nh;
    std::string reference_frame_;
    geometry_msgs::PoseArray points_;
    /**@brief Mapping from (i,j) to edge(i,j)**/
    EdgeMap* edge_map_;
    std::vector<std::vector<int>> adj_list_;

    /**@brief Mapping from State to State ID**/
    std::unordered_map<int, State_t> StateMap;

    ros::Publisher points_pub_;
    ros::Publisher edges_pub_;
    ros::Publisher force_pub_;
    tf::TransformListener listener_;

    bool visualize_dmodel_;
    bool visualize_expansions_;

    EnvCfg_t env_cfg_;
    // The force primitives that determine the successors of a state. 
    std::vector<tf::Vector3> force_primitives_;
    // Point index at which forces can be applied. 
    std::vector<int> grasp_idxs_;

    /**@brief For a given point, find all points in the same rigid component and 
     * a set of separating points**/
    void ExtractIndices(int p_idx,
        std::vector<int>* component_idxs,
        std::vector<int>* sep_idxs,
        JointType* joint_type,
        tf::Vector3* normal);

    /**@brief Return a set of successors for a given point*/
    void GetAdjPoints(int p_idx, std::vector<int>* adj_points);

    /**@brief State to State ID mapping**/
    int StateToStateID(State_t& s);
    /**@brief State ID to State mapping**/
    State_t StateIDToState(int state_id);

    /**@brief Convert force primitive to fprim ID**/
    int FPrimToFPrimID(int grasp_idx, int force_idx);
    void FPrimIDToFPrim(int fprim_id, int* grasp_idx, int* force_idx);

    /**@brief Return true if valid state**/
    bool IsValidState(const State_t& s);

    /**@brief Return successor states**/
    void GetSuccs(int source_state_id, std::vector<int>* succs, 
        std::vector<int>* edge_ids, std::vector<double>* costs);
    /**@brief Returns true if state ID satisfies partial goal**/
    bool IsGoalState(int state_id);

    /**@brief Return heuristic for a state**/
    double GetGoalHeuristic(int state_id);

    /**@brief Return a vector expressing p2's position in p1's frame**/
    tf::Vector3 GetLocalVector(const geometry_msgs::Pose p1, const geometry_msgs::Pose p2);

    DISALLOW_COPY_AND_ASSIGN(DModel);
};

#endif /* _LTM_DMODEL_H */
