#include "rbdl_model_tests/ModelFromParser/RBDLModelFromAutomation.h"
#include "rbdl_model_tests/rbdl_tests.h"

RBDLModelFromAutomation* rbdlModelWrapper = nullptr;


TEST_CASE(__FILE__"_Initilize", "") 
{
  rbdlModelWrapper = new RBDLModelFromAutomation();
}


TEST_CASE(__FILE__"_Body", "") 
{
  if(rbdlModelWrapper == nullptr) return;

  // rbdlModelWrapper->PrintModelHierarchy();
}

// TEST_CASE(__FILE__"_HomePose", "") 
// {
//   if(rbdlModelWrapper == nullptr) return;
//   ClearLogOutput();

//   std::vector<std::string> controllableJoints = rbdlModelWrapper->ControlableJoints();

//   // Create Vector size of controllabel Joints
//   std::vector<double> desiredJointAngles(controllableJoints.size(), 0.0);

//   std::vector<t_w_nPtr> transformationsFromModels = rbdlModelWrapper->T_W_NfromModels(desiredJointAngles);

//   for(t_w_nPtr t_w_nptr : transformationsFromModels)
//   {
//     // std::cout << "t_w_nptr->t_w_n_ambf" << std::endl << t_w_nptr->t_w_n_ambf.r << std::endl;
//     // std::cout << "t_w_nptr->t_w_n_rbdl" << std::endl << t_w_nptr->t_w_n_rbdl.r << std::endl;
     
//     CHECK_THAT (t_w_nptr->t_w_n_ambf.r, 
//     AllCloseVector(t_w_nptr->t_w_n_rbdl.r, TEST_PREC, TEST_PREC));
//   }
// }

TEST_CASE(__FILE__"_RandomPose", "") 
{
  if(rbdlModelWrapper == nullptr) return;
  ClearLogOutput();

  std::vector<std::string> controllableJoints = rbdlModelWrapper->ControlableJoints();

  // Create Vector size of controllabel Joints
  std::vector<double> desiredJointAngles(controllableJoints.size(), 0.0);
    desiredJointAngles.at(0)  = M_PI_4;
    // desiredJointAngles.at(1)  = M_PI_4;
    desiredJointAngles.at(2)  = 0.25;
    desiredJointAngles.at(3)  = M_PI_4;

  std::vector<t_w_nPtr> transformationsFromModels = rbdlModelWrapper->T_W_NfromModels(desiredJointAngles);

  for(t_w_nPtr t_w_nptr : transformationsFromModels)
  {
    // std::cout << "t_w_nptr->t_w_n_ambf" << std::endl << t_w_nptr->t_w_n_ambf.r << std::endl;
    // std::cout << "t_w_nptr->t_w_n_rbdl" << std::endl << t_w_nptr->t_w_n_rbdl.r << std::endl;
     
    CHECK_THAT (t_w_nptr->t_w_n_ambf.r, 
    AllCloseVector(t_w_nptr->t_w_n_rbdl.r, TEST_PREC, TEST_PREC));
  }
}

TEST_CASE(__FILE__"_CleanUp", "") 
{
  if(rbdlModelWrapper == nullptr) return;
  rbdlModelWrapper->~RBDLModelFromAutomation();
  // delete rbdlModelWrapper;
}