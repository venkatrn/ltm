/**
 * @file abstract_model.h
 * @author Venkatraman Narayanan
 * Carnegie Mellon University, 2013
 */

#ifndef _LTM_DMODEL_ABSTRACT_MODEL_H_
#define _LTM_DMODEL_ABSTRACT_MODEL_H_

#include <vector>

class AbstractModel
{
  public:

    /**@brief Return all successors and corresponding edge costs for a state**/
    virtual void GetSuccs(int source_state_id, std::vector<int>* succs, 
        std::vector<int>* edge_ids, std::vector<double>* costs) = 0;
    /**@brief Return true if state is a goal state**/
    virtual bool IsGoalState(int state_id) = 0;
    /**@brief Heuristic for state**/
    virtual double GetGoalHeuristic(int state_id)
    {
      return 0;
    }
};

#endif /* _LTM_DMODEL_ABSTRACT_MODEL_H_ */
