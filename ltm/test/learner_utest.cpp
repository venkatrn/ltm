#include <ltm/d_model_learner.h>

#include <tf/LinearMath/Vector3.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>

#include "gtest/gtest.h"

using namespace std;

const char kTest1FixedModel[] = "/usr0/home/venkatrn/groovy_workspace/sandbox/ltm_stack/ltm/matlab/models/test1/fixed_model.mdl";
const char kTest1PrismaticModel[] = "/usr0/home/venkatrn/groovy_workspace/sandbox/ltm_stack/ltm/matlab/models/test1/prismatic_model.mdl";

const char kTest1FixedSequence[] = "/usr0/home/venkatrn/groovy_workspace/sandbox/ltm_stack/ltm/matlab/test_data/test1/fixed_obs_sequence.txt";
const char kTest1PrismaticSequence[] = "/usr0/home/venkatrn/groovy_workspace/sandbox/ltm_stack/ltm/matlab/test_data/test1/prismatic_obs_sequence.txt";
const char kTest1SwitchingSequence[] = "/usr0/home/venkatrn/groovy_workspace/sandbox/ltm_stack/ltm/matlab/test_data/test1/switching_obs_sequence.txt";

const char kSphere20FPrims[] = "/usr0/home/venkatrn/groovy_workspace/sandbox/ltm_stack/ltm/matlab/fprims/sphere_20.fprim";

const string kReferenceFrame = "map";

class DModelLearnerTest : public testing::Test 
{
  protected:
    virtual void SetUp()
    {
    }

    /*
    virtual void TearDown()
    {
    }
    */

    DModelLearner* learner;
    vector<Transition> transitions;
    vector<string> candidate_models;

};

TEST_F(DModelLearnerTest, Test1FixedModel)
{
      learner = new DModelLearner(kReferenceFrame);
      transitions.clear();
      candidate_models.clear();

      candidate_models.push_back(string(kTest1FixedModel));
      candidate_models.push_back(string(kTest1PrismaticModel));

      learner->InitializeCandidateModels(candidate_models, kSphere20FPrims, 2, 0.5);
      learner->LearnTransitions(string(kTest1FixedSequence), &transitions);
      
      EXPECT_EQ(0, transitions.size());

      delete learner;
}

TEST_F(DModelLearnerTest, Test1PrismaticModel)
{
      learner = new DModelLearner(kReferenceFrame);
      transitions.clear();
      candidate_models.clear();

      candidate_models.push_back(string(kTest1FixedModel));
      candidate_models.push_back(string(kTest1PrismaticModel));

      learner->InitializeCandidateModels(candidate_models, kSphere20FPrims, 2, 0.5);
      learner->LearnTransitions(string(kTest1PrismaticSequence), &transitions);
      
      EXPECT_EQ(0, transitions.size());

      delete learner;
}

TEST_F(DModelLearnerTest, Test1SwitchingModel)
{
      learner = new DModelLearner(kReferenceFrame);
      transitions.clear();
      candidate_models.clear();

      candidate_models.push_back(string(kTest1FixedModel));
      candidate_models.push_back(string(kTest1PrismaticModel));

      learner->InitializeCandidateModels(candidate_models, kSphere20FPrims, 2, 0.5);
      learner->LearnTransitions(string(kTest1SwitchingSequence), &transitions);
      
      EXPECT_EQ(1, transitions.size());

      delete learner;
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "ltm_learner_test");
  return RUN_ALL_TESTS();
}
