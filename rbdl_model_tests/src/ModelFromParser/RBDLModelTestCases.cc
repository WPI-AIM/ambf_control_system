#include "rbdl_model_tests/ModelFromParser/RBDLModelFromAutomation.h"
#include "rbdl_model_tests/rbdl_tests.h"

RBDLModelFromAutomation* rbdlModel = nullptr;


TEST_CASE(__FILE__"_Initilize", "") 
{
  rbdlModel = new RBDLModelFromAutomation();
}


// TEST_CASE(__FILE__"_Body", "") 
// {
//   rbdlModel->PrintModelHierarchy();
// }

// TEST_CASE(__FILE__"_HomePose", "") 
// {
//   std::vector<std::string> controllableJoints = rbdlModel->ControlableJoints();

//   std::vector<double> desiredJointAngles(controllableJoints.size(), 0.0);
//   std::vector<t_w_nPtr> transformationsFromModels = rbdlModel->T_W_NfromModels(desiredJointAngles);
// }

TEST_CASE(__FILE__"_CleanUp", "") 
{
  delete rbdlModel;
}