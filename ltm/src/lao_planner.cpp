/**
 * @file lao_planner.cpp
 * @author Venkatraman Narayanan
 * Carnegie Mellon University, 2014
 */

#include <ltm/lao_planner.h>
#include <ros/console.h>

#include <algorithm>
#include <cassert>
#include <cstdio>
#include <ctime>
#include <queue>
#include <set>
#include <stack>
#include <unordered_set>

using namespace std;

const int kMaxPlannerExpansions = 5000;//1000

LAOPlanner::LAOPlanner()
{
  start_state_id_ = -1;
  goal_state_id_ = -1;
  model_bank_ = nullptr;
}

LAOPlanner::~LAOPlanner()
{
}

void LAOPlanner::SetStart(int start_state_id)
{
  start_state_id_ = start_state_id;
  return;
}

void LAOPlanner::SetGoal(int goal_state_id)
{
  goal_state_id_ = goal_state_id;
  return;
}

void LAOPlanner::SetModelBank(AbstractModelBank* model_bank)
{
  model_bank_ = model_bank;
  return;
}

void LAOPlanner::DFSTraversal(vector<int>* traversal)
{
  traversal->clear();
  stack<int> dfs_stack;
  vector<int> visited_states;
  dfs_stack.push(start_state_id_);
  visited_states.push_back(start_state_id_);
  bool terminal_state = false;
  while (dfs_stack.size() != 0)
  {
    while (!terminal_state)
    {
      PlannerState s = StateIDToState(dfs_stack.top());
      //ROS_INFO("Top state id: %d", s.state_id);
      int best_vec_idx = s.best_vec_idx;
      //ROS_INFO("Best action id: %d", s.best_action_id);
      if (best_vec_idx == -1 || model_bank_->IsGoalState(dfs_stack.top()))
      {
        terminal_state = true;
        break;
      }
      vector<int> best_action_succs = s.succ_state_ids_map[best_vec_idx];
      if (best_action_succs.size() == 0)
      {
        terminal_state = true;
        break;
      }
      // Declare terminal state if no successors are inserted
      terminal_state = true;
      for (size_t ii = 0; ii < best_action_succs.size(); ++ii)
      {
        // Avoid inserting duplicates (possible in a graph)
        auto it = find(visited_states.begin(), visited_states.end(), best_action_succs[ii]);
        if (it != visited_states.end())
        {
          continue;
        }
        dfs_stack.push(best_action_succs[ii]);
        visited_states.push_back(best_action_succs[ii]);
        terminal_state = false;
      }
    }
    traversal->push_back(dfs_stack.top());
    dfs_stack.pop();
    terminal_state = false;
  }
}

bool LAOPlanner::Plan(vector<int>* state_ids, vector<int>* fprim_ids)
{
  // Plan from scratch
  PlannerStateMap.clear();

  state_ids->clear();
  if (fprim_ids != nullptr)
  {
    fprim_ids->clear();
  }

  if (model_bank_ == nullptr)
  {
    printf("Planner: Environment is not defined\n");
    return false;
  }

  PlannerState start_state(start_state_id_, model_bank_->GetGoalHeuristic(start_state_id_), -1);
  PlannerState goal_state(goal_state_id_, 0, -1);

  planner_stats_.expansions = 0;


  PlannerStateMap[start_state_id_] = start_state;

  bool exists_non_terminal_states = true;
  clock_t begin_time = clock();

  while (exists_non_terminal_states)
  {
    /*
       if (planner_stats_.expansions > kMaxPlannerExpansions)
       {
       printf("Planner: Exceeded max expansions\n");
       return false;
       }
       */

    // Get the DFS traversal of the best partial solution graph
    vector<int> dfs_traversal;
    DFSTraversal(&dfs_traversal);

    // DEBUG
    /*
    ROS_INFO("DFS Traversal");
    for (int ii = 0; ii < dfs_traversal.size(); ++ii)
    {
      ROS_INFO("%d", dfs_traversal[ii]);
    }
    */
    // Iterate through and update V-values 
    for (size_t ii = 0; ii < dfs_traversal.size(); ++ii)
    {
      PlannerState s = StateIDToState(dfs_traversal[ii]);
      // TODO: Update this. Search ends if state being expanded is a goal state 
      if (model_bank_->IsGoalState(s.state_id)) 
      {
        ROS_INFO("[LAO Planner]: Goal state is found\n");
        exists_non_terminal_states = false;
        goal_state_id_ = s.state_id;
        goal_state = s;
        break;
      }

      // Expand the state if not already expanded
      if (!s.expanded)
      {
        ROS_INFO("Expanding state %d", s.state_id);
        vector<vector<int>> succ_state_ids_map;
        vector<vector<double>> succ_state_probabilities_map;
        vector<int> action_ids;
        vector<double> action_costs;
        model_bank_->GetSuccs(s.state_id, &succ_state_ids_map, &succ_state_probabilities_map,
            &action_ids, &action_costs);

        s.action_ids = action_ids;
        s.action_costs = action_costs;
        s.succ_state_ids_map = succ_state_ids_map;
        s.succ_state_probabilities_map = succ_state_probabilities_map;
        s.expanded = true;

        /*
        ROS_INFO("Succs:");
        for (int ii = 0; ii < succ_state_ids_map.size(); ++ii)
        {
          ROS_INFO("Edge: %d: %f", action_ids[ii], action_costs[ii]);
          for (int jj = 0; jj < succ_state_ids_map[ii].size(); ++jj)
          {
            ROS_INFO("State: %d: %f", succ_state_ids_map[ii][jj], succ_state_probabilities_map[ii][jj]);
          }
        }
        */

        for (size_t ii = 0; ii < action_ids.size(); ++ii)
        {
          for (size_t jj = 0; jj < succ_state_ids_map[ii].size(); ++jj)
          {
            const int succ_id = succ_state_ids_map[ii][jj];
            // Skip successors that have already been generated
            auto it = PlannerStateMap.find(succ_id);
            if (it != PlannerStateMap.end())
            {
              continue;
            }
            PlannerState succ_state(succ_state_ids_map[ii][jj], model_bank_->GetGoalHeuristic(succ_state_ids_map[ii][jj]), s.state_id);
            PlannerStateMap[succ_id] = succ_state;
          }
        }
      }

      // Update V-values: V(s) = min_{a\in A} c(s,a) + \sum_{s'} P(s'|s,a)*V(s') 
      const int num_actions = s.succ_state_ids_map.size();
      assert(num_actions == int(s.succ_state_probabilities_map.size()));
      assert(num_actions == int(s.action_ids.size()));
      double min_expected_cost = 1e10; //TODO: Set to Inf
      int best_idx = -1;
      for (int jj = 0; jj < num_actions; ++jj)
      {
        const int num_succs = s.succ_state_ids_map[jj].size();
        assert(num_succs == int(s.succ_state_probabilities_map[jj].size()));
        double expected_cost = s.action_costs[jj];
        for (int kk = 0; kk < num_succs; ++kk)
        {
          PlannerState succ_state = StateIDToState(s.succ_state_ids_map[jj][kk]);
          expected_cost += (succ_state.v * s.succ_state_probabilities_map[jj][kk]);
        }
        if (expected_cost < min_expected_cost)
        {
          min_expected_cost = expected_cost;
          best_idx = jj;
        }
      }
      s.best_action_id = s.action_ids[best_idx];
      s.best_vec_idx = best_idx;
      s.v = min_expected_cost;

      // Update the state in PlannerStateMap
      PlannerStateMap[s.state_id] = s;
    }
  }



  // Expand state
  /*
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
  */
clock_t end_time = clock();
planner_stats_.time = double(end_time - begin_time)/ CLOCKS_PER_SEC;
//planner_stats_.cost = start_state.g;

// Reconstruct path
/*
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
*/
ROS_INFO("[LAO Planner]: Finished planning");
PrintPlannerStateMap();
return true;
}

PlannerStats LAOPlanner::GetPlannerStats()
{
  return planner_stats_; 
}


PlannerState LAOPlanner::StateIDToState(int state_id)
{ 
  auto it = PlannerStateMap.find(state_id);
  if (it != PlannerStateMap.end())
  {                            
    return it->second;
  } 
  else
  { 
    ROS_ERROR("LAO Planner: Error. Requested State ID does not exist. Will return empty state.\n");
  }
  PlannerState empty_state;
  return empty_state;
}  

PlannerState* LAOPlanner::StateIDToMutableState(int state_id)
{ 
  auto it = PlannerStateMap.find(state_id);
  if (it != PlannerStateMap.end())
  {                            
    return &(it->second);
  } 
  else
  { 
    ROS_ERROR("LAO Planner: Error. Requested State ID does not exist. Will return empty state.\n");
  }
  return nullptr;
}  

void LAOPlanner::PrintPlannerStateMap()
{
  for (auto it = PlannerStateMap.begin(); it != PlannerStateMap.end(); ++it)
  {
    ROS_INFO("ID: %d | V: %f | E: %d", it->first, it->second.v, it->second.expanded); 
  }
}

