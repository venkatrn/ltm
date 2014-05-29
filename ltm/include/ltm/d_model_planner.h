/**
 * @file d_model_planner.h
 * @author Venkatraman Narayanan
 * Carnegie Mellon University, 2013
 */

#ifndef _LTM_DMODEL_PLANNER_H_
#define _LTM_DMODEL_PLANNER_H_

#include <ltm/abstract_model.h>

#include <cmath>
#include <set>

// Tolerance for comparing double numbers.
const double kDblTolerance = 1e-4;

struct PlannerState 
{
  int state_id;
  double g;
  double h;

  int parent_state_id;
  // Store the motion primitive id to make reconstruction easier.
  int edge_id;

  PlannerState()
  {
    state_id = -1;
    g = 0;
    h = 0;
    parent_state_id = -1;
  }

  PlannerState(int s_id, int g_val, int h_val, int parent_s_id)
  {
    state_id = s_id;
    g = g_val;
    h = h_val;
    parent_state_id = parent_s_id;
  }

  double Key(double eps) const
  {
    return g + eps*h;
  }

};

struct StateCompare
{
  double eps;

  StateCompare(double epsilon)
  {
    eps = epsilon;
  }

  bool operator() (const PlannerState& s1, const PlannerState& s2) const
  {
    double key_1 = s1.Key(eps); 
    double key_2 = s2.Key(eps); 
    if (fabs(key_1 - key_2) <= kDblTolerance)
    {
      if (fabs(s1.h - s2.h) < kDblTolerance)
      {
        return false;
      }
      return (s1.h < s2.h);
    }
    return (key_1 < key_2);
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

class DModelPlanner
{
  public: 
    DModelPlanner();
    ~DModelPlanner();
    void SetStart(int start_state_id);
    void SetGoal(int goal_state_id);
    void SetModel(AbstractModel* model);
    void SetEpsilon(double eps);
    bool Plan(std::vector<int>* state_ids, std::vector<int>* fprim_ids = nullptr);
    PlannerStats GetPlannerStats();
    
  private:
    // Planner takes passive ownership of the model (does not delete it when going out of scope)
    AbstractModel* model_;
    int start_state_id_;
    int goal_state_id_;
    double eps_;
    PlannerStats planner_stats_;

    /**@brief Find a state in the open list (multiset) by state ID. 
     * Returns an iterator to the state if found, otherwise the end iterator
     **/
    std::multiset<PlannerState, StateCompare>::iterator FindStateInOpenList(
        std::multiset<PlannerState, StateCompare>& o_list, PlannerState state);

    // Debug Utils
    void PrintOpenList(std::multiset<PlannerState, StateCompare> o_list);
    
};

#endif /* _LTM_DMODEL_PLANNER_H */
