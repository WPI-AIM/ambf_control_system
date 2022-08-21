#include "rbdl_model_tests/PS.h"
#include "rbdl_model_tests/rbdl_tests.h"
PS* ps = nullptr;

TEST_CASE(__FILE__"_Initilize", "") 
{
  ps = new PS();
}

// TEST_CASE_METHOD(ps, __FILE__"_TestpsBodyHierarchy", "") 
// {
//   if(ps == nullptr) return;
// }

TEST_CASE(__FILE__"_TestHomePose", "") 
{
  if(ps == nullptr) return;
  std::vector<t_w_nPtr> transformations = ps->HomePoseTransformation();
  // for(t_w_nPtr t_w_nptr : transformations)
  // {
  //   std::cout << "body Name: " << t_w_nptr->bodyName << std::endl;
    // std::cout << "t_w_nptr->r_w_n_ambf" << std::endl << t_w_nptr->r_w_n_ambf << std::endl;
    // std::cout << "t_w_nptr->r_w_n_rbdl" << std::endl << t_w_nptr->r_w_n_rbdl << std::endl;

    // std::cout << "t_w_nptr->p_w_n_ambf" << std::endl << t_w_nptr->p_w_n_ambf << std::endl;
    // std::cout << "t_w_nptr->p_w_n_rbdl" << std::endl << t_w_nptr->p_w_n_rbdl << std::endl;

    // // CHECK_THAT (t_w_nptr->r_w_n_ambf, AllCloseMatrix(t_w_nptr->r_w_n_rbdl, TEST_PREC, TEST_PREC));
    // CHECK_THAT (t_w_nptr->p_w_n_ambf, AllCloseVector(t_w_nptr->p_w_n_rbdl, TEST_PREC, TEST_PREC));
    // std::cout << "\n------------------------\n";
  // }
}

TEST_CASE(__FILE__"_RandomPose", "") 
{
  if(ps == nullptr) return;
  // std::vector<std::string> jointNames = ps->ControllableJointNames();
  

  // ps->JointAngleWithName(jointNames.at(0), M_PI_4);
  // ps->JointAngleWithName("world-l1", 0.0f);
  float qDesired = 0.1f;
  ps->JointAngleWithName("l1-l2", qDesired);
  ps->JointAngleWithName("l2-l3", -qDesired);
  ps->JointAngleWithName("l3-l4", qDesired);
  ps->JointAngleWithName("l1-l4", qDesired);
  // ps->JointAngleWithName("l1-l2", 0.3036194145679474);
  // ps->JointAngleWithName("l2-l3", -0.33735570311546326);
  // ps->JointAngleWithName("l3-l4", 0.37723666429519653);
  // ps->JointAngleWithName("l1-l4", 0.3036194145679474);

  // ps->JointAngleWithName("l1-l2", 0.3);
  // ps->JointAngleWithName("l2-l3", -0.33735570311546326);
  // ps->JointAngleWithName("l3-l4", 0.37723666429519653);
  // ps->JointAngleWithName("l1-l4", 0.3036194145679474);

  // VectorNd QTarget = ps->TargetJointAngles();
  // std::cout << "QTarget" << std::endl << QTarget << std::endl;

  ps->ExecutePose();
  // const std::string jointName = ;
  // t_w_nPtr t_w_nptr = ps->twnFromModels<std::string>("world-baselink");
  // CHECK_THAT (t_w_nptr->p_w_n_ambf, AllCloseVector(t_w_nptr->p_w_n_rbdl, TEST_PREC, TEST_PREC));


  unsigned int qSize = ps->RBDLModelJointSize();
  
  for(unsigned int qId = 1; qId <= qSize; qId++)
  {
    // const std::string jointName = "baselink-yawlink";
    // int qId = 6;
    t_w_nPtr t_w_nptr = ps->twnFromModels<unsigned int>(qId);

    std::cout << "jointId: " << qId << ", bodyName: " << t_w_nptr->bodyName << std::endl;
    // std::cout << "t_w_nptr->r_w_n_ambf: " << std::endl << t_w_nptr->r_w_n_ambf << std::endl;
    // std::cout << "t_w_nptr->r_w_n_rbdl: " << std::endl << t_w_nptr->r_w_n_rbdl << std::endl;

    std::cout << "t_w_nptr->p_w_n_ambf: " << std::endl << t_w_nptr->p_w_n_ambf << std::endl;
    std::cout << "t_w_nptr->p_w_n_rbdl: " << std::endl << t_w_nptr->p_w_n_rbdl << std::endl;
    
    // CHECK_THAT (t_w_nptr->r_w_n_ambf, AllCloseMatrix(t_w_nptr->r_w_n_rbdl, TEST_PREC, TEST_PREC));
    CHECK_THAT (t_w_nptr->p_w_n_ambf, AllCloseVector(t_w_nptr->p_w_n_rbdl, TEST_PREC, TEST_PREC));
    std::cout << "--------------------------\n";
  }
}

TEST_CASE(__FILE__"_Cleanup", "") 
{ 
  if(ps == nullptr) return;
  ps->CleanUp();
}