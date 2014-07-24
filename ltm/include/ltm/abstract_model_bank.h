/**
 * @file abstract_model_bank.h
 * @author Venkatraman Narayanan
 * Carnegie Mellon University, 2014
 */

#ifndef _LTM_DMODEL_ABSTRACT_MODEL_BANK_H_
#define _LTM_DMODEL_ABSTRACT_MODEL_BANK_H_

#include <vector>

class AbstractModelBank
{
  public:

    /**@brief Return successors, edge costs, edge ids, and probabilities for an expansion in an AND-OR graph**/
    virtual void GetSuccs(int source_state_id, std::vector<std::vector<int>>* succ_state_ids_map, 
        std::vector<std::vector<double>>* succ_state_probabilities_map, 
        std::vector<int>* action_ids, std::vector<double>* action_costs) = 0;
    /**@brief Return true if state is a goal state**/
    virtual bool IsGoalState(int state_id) = 0;
    /**@brief Heuristic for state**/
    virtual double GetGoalHeuristic(int state_id)
    {
      return 0;
    }
};

#endif /* _LTM_DMODEL_ABSTRACT_MODEL_BANK_H_ */
