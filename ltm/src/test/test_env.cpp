#include <ltm/d_model.h>
#include <ltm/d_model_planner.h>

#include <tf/LinearMath/Vector3.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>

using namespace std;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ltm");
  ros::NodeHandle nh;
  DModel* d_model = new DModel;
  //ros::Timer timer = nh.createTimer(ros::Duration(0.1), &DModel::TFTimedCallback, &d_model);

  d_model->InitFromFile("/usr0/home/venkatrn/groovy_workspace/sandbox/ltm_stack/ltm/models/drawer.mdl");
  d_model->InitForcePrimsFromFile("/usr0/home/venkatrn/groovy_workspace/sandbox/ltm_stack/ltm/models/two_forces.fprims");

  // Create and set start state.
  State_t start_state;
  d_model->SetForceIndex(132);
  d_model->SetStartState(start_state);

  // Create and set goal state.
  State_t goal_state;
  goal_state.changed_inds.push_back(130);
  geometry_msgs::Pose p;
  p.position.x = 1.68;
  p.position.y = -1;
  p.position.z = 1;
  goal_state.changed_points.poses.push_back(p);
  d_model->SetGoalState(goal_state);

  // Initialize the planner. 
  DModelPlanner* planner = new DModelPlanner;
  planner->SetModel(d_model);
  planner->SetStart(d_model->GetStartStateID());

  // Plan.
  vector<int> state_ids, fprim_ids;
  planner->Plan(&state_ids, &fprim_ids);

  // Print planner stats.
  PlannerStats planner_stats = planner->GetPlannerStats();
  printf("Planner Stats:\nNum Expansions: %d, Planning Time: %f, Solution Cost: %d\n",
      planner_stats.expansions, planner_stats.time, planner_stats.cost);

  // Print solution path.
  printf("Solution Path:\n");
  for (size_t ii = 0; ii < state_ids.size(); ++ii)
  {
    printf("state ID: %d, fprim ID: %d\n", state_ids[ii], fprim_ids[ii]);
  }
  printf("\n");

  // Print force sequence.
  vector<tf::Vector3> forces;
  d_model->ConvertForcePrimIDsToForces(fprim_ids, &forces);
  printf("Force Sequence:\n");
  for (size_t ii = 0; ii < forces.size(); ++ii)
  {
    printf("%f %f %f\n", forces[ii].x(), forces[ii].y(), forces[ii].z());
  }
  printf("\n");

  // Print end-effector trajectory
  geometry_msgs::PoseArray traj;
  d_model->GetEndEffectorTrajFromStateIDs(state_ids, &traj);
  printf("End-Effector Trajectory:\n");
  for (size_t ii = 0; ii < traj.poses.size(); ++ii)
  {
    printf("%f %f %f\n", traj.poses[ii].position.x, traj.poses[ii].position.y, traj.poses[ii].position.z);
  }
  printf("\n");
  
  
  // Simulate plan.
  d_model->SimulatePlan(forces);
  
  delete d_model;
  delete planner;
  return 0;
}
