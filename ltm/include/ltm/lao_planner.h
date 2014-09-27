/**
 * @file lao_planner.h
 * @brief Implements the algorithm in https://www.ics.uci.edu/~dechter/papers/paginated_binders/PART%25201.pdf
 * @author Venkatraman Narayanan
 * Carnegie Mellon University, 2014
 */

#ifndef _LTM_LAO_PLANNER_H_
#define _LTM_LAO_PLANNER_H_

#include <ltm/abstract_model_bank.h>

#include <cmath>
#include <unordered_map>

// Tolerance for comparing double numbers.
const double kDblTolerance = 1e-4;

struct PlannerState 
{
  int state_id;
  double v;
  bool expanded;

  // Successors indexed by action id
  std::vector<int> action_ids;
  std::vector<double> action_costs;
  std::vector<std::vector<int>> succ_state_ids_map;
  std::vector<std::vector<double>> succ_state_probabilities_map;
  int parent_state_id;
  int best_action_id;
  int best_vec_idx; //Cached for convenience -- action_ids[best_vec_idx] = best_action_id

  PlannerState()
  {
    state_id = -1;
    v = 0;
    expanded = false;
    parent_state_id = -1;
    best_action_id = -1;
    best_vec_idx = -1;
  }

  PlannerState(int s_id, int v_val, int parent_s_id)
  {
    state_id = s_id;
    v = v_val;
    expanded = false;
    parent_state_id = parent_s_id;
    best_action_id = -1;
    best_vec_idx = -1;
  }

};

struct StateHasher
{
  int operator() (const PlannerState& s) const
  {
    return s.state_id;
  }
};

struct StateEqual
{
  bool operator() (const PlannerState& s1, const PlannerState& s2) const
  {
    return (s1.state_id == s2.state_id);
  }
};

struct PlannerStats
{
  int expansions;
  double time;
  int cost;

  PlannerStats() : expansions(-1),
  time(-1.0),
  cost(-1)
  {}
};

class LAOPlanner
{
  public: 
    LAOPlanner();
    ~LAOPlanner();
    void SetStart(int start_state_id);
    void SetGoal(int goal_state_id);
    void SetModelBank(AbstractModelBank* model);
    bool Plan(std::vector<int>* state_ids, std::vector<int>* fprim_ids = nullptr);
    PlannerStats GetPlannerStats();
    
  private:
    // Planner takes passive ownership of the model (does not delete it when going out of scope)
    AbstractModelBank* model_bank_;
    int start_state_id_;
    //TODO: remove this
    int goal_state_id_;
    PlannerStats planner_stats_;

    /**@brief Return a postorder DFS traversal (state ids) of the best solution graph**/
    void DFSTraversal(std::vector<int>* traversal);
    /**@brief Reconstruct optimistic path (actions lead to successor with smallest V-value**/
    void ReconstructOptimisticPath(std::vector<int>* state_ids, std::vector<int>* fprim_ids);
    /**@brief Do value iteration on the best solution graph**/
    void SolutionValueIteration();

    /**@brief Planner hash table**/
    std::unordered_map<int, PlannerState> PlannerStateMap;
    PlannerState StateIDToState(int state_id);
    PlannerState* StateIDToMutableState(int state_id);

    /**@Debug**/
    void PrintPlannerStateMap();
};

#endif /* _LTM_LAO_PLANNER_H */
