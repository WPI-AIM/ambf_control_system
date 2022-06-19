#include "rbdl_model_tests/ModelFromParser/RBDLModelFromAutomation.h"
#include "rbdl_model_tests/rbdl_tests.h"

RBDLModelFromAutomation* rbdlModel = nullptr;


TEST_CASE(__FILE__"_Initilize", "") 
{
  rbdlModel = new RBDLModelFromAutomation();
}


TEST_CASE(__FILE__"_Body", "") 
{
  if(rbdlModel == nullptr) return;

  rbdlModel->PrintModelHierarchy();
}

// TEST_CASE(__FILE__"_HomePose", "") 
// {
//   if(rbdlModel == nullptr) return;

//   std::vector<std::string> controllableJoints = rbdlModel->ControlableJoints();

//   // Create Vector size of controllabel Joints
//   std::vector<double> desiredJointAngles(controllableJoints.size(), 0.0);
//   std::vector<t_w_nPtr> transformationsFromModels = rbdlModel->T_W_NfromModels(desiredJointAngles);

//   for(t_w_nPtr t_w_nptr : transformationsFromModels)
//   {
//     std::cout << "t_w_nptr->t_w_n_ambf" << std::endl << t_w_nptr->t_w_n_ambf.r << std::endl;
//     std::cout << "t_w_nptr->t_w_n_rbdl" << std::endl << t_w_nptr->t_w_n_rbdl.r << std::endl;
     
//     CHECK_THAT (t_w_nptr->t_w_n_ambf.r, 
//     AllCloseVector(t_w_nptr->t_w_n_rbdl.r, TEST_PREC, TEST_PREC));
//   }
// }

TEST_CASE(__FILE__"_CleanUp", "") 
{
  if(rbdlModel = nullptr) return;
  
  delete rbdlModel;
}