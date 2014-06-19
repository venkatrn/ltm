/**
 * @file d_model_planner.cpp
 * @author Venkatraman Narayanan
 * Carnegie Mellon University, 2013
 */

#include <ltm/d_model_planner.h>

#include <algorithm>
#include <cstdio>
#include <ctime>
#include <queue>
#include <set>
#include <unordered_set>

using namespace std;

const int kMaxPlannerExpansions = 5000;//1000

DModelPlanner::DModelPlanner()
{
  start_state_id_ = -1;
  goal_state_id_ = -1;
  model_ = nullptr;
  eps_ = 1;
}

DModelPlanner::~DModelPlanner()
{
}

void DModelPlanner::SetStart(int start_state_id)
{
  start_state_id_ = start_state_id;
  return;
}

void DModelPlanner::SetGoal(int goal_state_id)
{
  goal_state_id_ = goal_state_id;
  return;
}

void DModelPlanner::SetModel(AbstractModel* model)
{
  model_ = model;
  return;
}

void DModelPlanner::SetEpsilon(double eps)
{
  eps_ = eps;
  return;
}

bool DModelPlanner::Plan(vector<int>* state_ids, vector<int>* fprim_ids)
{
  state_ids->clear();
  if (fprim_ids != nullptr)
  {
    fprim_ids->clear();
  }

  if (model_ == nullptr)
  {
    printf("Planner: Environment is not defined\n");
    return false;
  }

  StateCompare state_compare(eps_);
  multiset<PlannerState, StateCompare> o_list(state_compare);
  unordered_set<PlannerState, StateHasher, StateEqual> c_list;
  PlannerState start_state(start_state_id_, 0, model_->GetGoalHeuristic(start_state_id_), -1);
  PlannerState goal_state(goal_state_id_, 0, 0, -1);

  o_list.insert(start_state);
  planner_stats_.expansions = 0;

  clock_t begin_time = clock();

  while (o_list.size() != 0)
  {
    if (planner_stats_.expansions > kMaxPlannerExpansions)
    {
      printf("Planner: Exceeded max expansions\n");
      return false;
    }

    // DEBUG
    // PrintOpenList(o_list);
    
    // Get state with best key value
    PlannerState s = *(o_list.begin());

    // DEBUG
    // printf("Expanding state %d with f-val: %f\n", s.state_id, s.Key(eps_));

    // Sanity check - we shouldn't be expanding states in the closed list.
    if (c_list.find(s) != c_list.end())
    {
      printf("Planner: Expanding a state in the closed list\n");
      printf("Expanding state %d in closed list with f-val: %f and g-val:\n", s.state_id, s.Key(eps_));
      return false;
    }
    // Search ends if state being expanded is a goal state 
    if (model_->IsGoalState(s.state_id)) 
    {
      printf("Planner: Goal state is found\n");
      goal_state_id_ = s.state_id;
      goal_state = s;
      break;
    }
    // Expand state
    vector<int> succ_ids, edge_ids;
    vector<double> costs;
    model_->GetSuccs(s.state_id, &succ_ids, &edge_ids, &costs);
    planner_stats_.expansions++;
    for (size_t ii = 0; ii < succ_ids.size(); ++ii)
    {
      PlannerState succ_state(succ_ids[ii], s.g + costs[ii], model_->GetGoalHeuristic(succ_ids[ii]), s.state_id);
      //TODO: This might not be the best way of storing force primitive number.
      succ_state.edge_id = edge_ids[ii];

      // Skip if already in closed list
      auto it_closed = c_list.find(succ_state);
      /*
      if (it_closed != c_list.end())
      {
        continue;
      }
      */
      // Skip if succ is already in open list and has a better key value
      auto it_open = FindStateInOpenList(o_list, succ_state);
      if (it_open != o_list.end())
      {
        if (succ_state.Key(eps_) >= it_open->Key(eps_))
        {
          continue;
        }
        else
        {
          // Update g value of succ_state otherwise
          o_list.erase(it_open);
          o_list.insert(succ_state);
        }
      }
      // Re-expand state from closed list if better path has been found
      else if (it_closed != c_list.end())
      {
        if (succ_state.Key(eps_) >= it_closed->Key(eps_))
        {
          continue;
        }
        else
        {
          // Update g value of succ_state otherwise
          c_list.erase(it_closed);
          o_list.insert(succ_state);
        }
      }
      else
      {
        o_list.insert(succ_state);
      }
    }
    // Mark state as expanded by removing from open list, and inserting into
    // closed list
    // Note: Cannot assume that the begin state is still the expanded state, after the newly inserted states
    auto it_expanded_state = FindStateInOpenList(o_list, s);
    o_list.erase(it_expanded_state);
    c_list.insert(s);
  }

  clock_t end_time = clock();
  planner_stats_.time = double(end_time - begin_time)/ CLOCKS_PER_SEC;
  planner_stats_.cost = goal_state.g;

  // Reconstruct path
  if (goal_state_id_ == -1)
  {
    printf("Planner: Open list is empty and goal was not found\n");
    return false;
  }

  state_ids->push_back(goal_state_id_);
  PlannerState current_state = goal_state;
  if (fprim_ids != nullptr)
  {
    fprim_ids->push_back(current_state.edge_id);
  }
  while (current_state.state_id != start_state_id_)
  {
    PlannerState prev_state(current_state.parent_state_id, -1, -1, -1);
    const double old_g = current_state.g;
    auto it = c_list.find(prev_state);
    if (it == c_list.end())
    {
      printf("Planner: Error reconstructing path. State not found in closed list\n");
      return false;
    }
    current_state = *it;
    const double new_g = current_state.g;
    if (new_g > old_g)
    {
      printf("Planner: Error reconstructing path. G-values are not non-increasing\n");
    }
    state_ids->push_back(current_state.state_id);
    if (fprim_ids != nullptr)
    {
      fprim_ids->push_back(current_state.edge_id);
    }
  }
  // Reverse path order to get start to goal state IDs.
  reverse(state_ids->begin(), state_ids->end());
  if (fprim_ids != nullptr)
  {
    reverse(fprim_ids->begin(), fprim_ids->end());
  }
  printf("Planner: Path reconstruction successful\n");
  return true;
}

PlannerStats DModelPlanner::GetPlannerStats()
{
  return planner_stats_; 
}

multiset<PlannerState, StateCompare>::iterator DModelPlanner::FindStateInOpenList(
    multiset<PlannerState, StateCompare>& o_list,
    PlannerState state)
{
  for (auto it = o_list.begin(); it != o_list.end(); ++it)
  {
    if (it->state_id == state.state_id)
    {
      return it;
    }
  }
  return o_list.end();
}

void DModelPlanner::PrintOpenList(multiset<PlannerState, StateCompare> o_list)
{
  for (auto it = o_list.begin(); it != o_list.end(); ++it)
  {
    printf("%d %f\n", it->state_id, it->Key(eps_));
  }
  printf("\n");
}
