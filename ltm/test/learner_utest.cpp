#include <ltm/d_model_learner.h>

#include <tf/LinearMath/Vector3.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <ros/package.h>

#include "gtest/gtest.h"

using namespace std;

string kTest1FixedModel = ros::package::getPath("ltm") + "/matlab/models/test1/fixed_model.mdl";
string kTest1PrismaticModel = ros::package::getPath("ltm") + "/matlab/models/test1/prismatic_model.mdl";

string kTest1FixedSequence = ros::package::getPath("ltm") + "/matlab/test_data/test1/fixed_obs_sequence.txt";
string kTest1PrismaticSequence = ros::package::getPath("ltm") + "/matlab/test_data/test1/prismatic_obs_sequence.txt";
string kTest1SwitchingSequence = ros::package::getPath("ltm") + "/matlab/test_data/test1/switching_obs_sequence.txt";

string kSphere20FPrims = ros::package::getPath("ltm") + "/matlab/fprims/sphere_20.fprim";

string kObservationsFile =  ros::package::getPath("ltm_ros") + "/data/observations.txt";

const string kReferenceFrame = "map";
const int kMaxPathLength = PATH_MAX;

class DModelLearnerTest : public testing::Test 
{
  protected:
    virtual void SetUp()
    {
      cout << kTest1FixedModel << endl;
    }

    /*
    virtual void TearDown()
    {
    }
    */

    DModelLearner* learner;
    vector<Transition> transitions;
    vector<string> candidate_models;

    // Setup full paths

};

/*
TEST_F(DModelLearnerTest, Test1FixedModel)
{
      learner = new DModelLearner(kReferenceFrame);
      transitions.clear();
      candidate_models.clear();

      candidate_models.push_back(kTest1FixedModel);
      candidate_models.push_back(kTest1PrismaticModel);

      learner->InitializeCandidateModels(candidate_models, kSphere20FPrims, 2, 0.5);
      learner->LearnTransitions(kTest1FixedSequence), &transitions;
      
      EXPECT_EQ(0, transitions.size());

      delete learner;
}

TEST_F(DModelLearnerTest, Test1PrismaticModel)
{
      learner = new DModelLearner(kReferenceFrame);
      transitions.clear();
      candidate_models.clear();

      candidate_models.push_back(kTest1FixedModel);
      candidate_models.push_back(kTest1PrismaticModel);

      learner->InitializeCandidateModels(candidate_models, kSphere20FPrims, 2, 0.5);
      learner->LearnTransitions(kTest1PrismaticSequence), &transitions;
      
      EXPECT_EQ(0, transitions.size());

      delete learner;
}

TEST_F(DModelLearnerTest, Test1SwitchingModel)
{
      learner = new DModelLearner(kReferenceFrame);
      transitions.clear();
      candidate_models.clear();

      candidate_models.push_back(kTest1FixedModel);
      candidate_models.push_back(kTest1PrismaticModel);

      learner->InitializeCandidateModels(candidate_models, kSphere20FPrims, 2, 0.5);
      learner->LearnTransitions(kTest1SwitchingSequence), &transitions;
      
      EXPECT_EQ(1, transitions.size());

      delete learner;
}
*/
TEST_F(DModelLearnerTest, TestObservations)
{
      learner = new DModelLearner(kReferenceFrame);
      transitions.clear();
      candidate_models.clear();

      candidate_models.push_back(kTest1FixedModel);
      candidate_models.push_back(kTest1PrismaticModel);

      learner->InitializeCandidateModels(candidate_models, kSphere20FPrims, 0, 0.5);
      learner->PlaybackObservations(kObservationsFile);
      learner->LearnPrior(kObservationsFile);

      delete learner;
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "ltm_learner_test");
  return RUN_ALL_TESTS();
}
