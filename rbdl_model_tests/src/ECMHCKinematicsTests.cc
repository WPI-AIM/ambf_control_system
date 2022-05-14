#include "rbdl_model_tests/RBDLTestPrep.h"
#include "rbdl_model_tests/ECM.h"
#include "rbdl_model_tests/EigenUtilities.h"

ECM* ecm = nullptr;

void ExecuteTestCases();

TEST_CASE(__FILE__"_Initilize", "") 
{
  ecm = new ECM();  
}

TEST_CASE(__FILE__"_TestHomePose", "") 
{
  std::vector<t_w_nPtr> transformations = ecm->HomePoseTransformation();
  for(t_w_nPtr t_w_nptr : transformations)
  {
    CHECK_THAT (t_w_nptr->r_w_n_ambf, AllCloseMatrix(t_w_nptr->r_w_n_rbdl, TEST_PREC, TEST_PREC));
    CHECK_THAT (t_w_nptr->p_w_n_ambf, AllCloseVector(t_w_nptr->p_w_n_rbdl, TEST_PREC, TEST_PREC));
  }
}

TEST_CASE(__FILE__"_Cleanup", "") 
{  
  ecm->CleanUp();
}
// TEST_CASE(__FILE__"_ECMFKTest", "") 
// {
//   // ecm->SetJointAngleWithName("baselink-yawlink", M_PI_2);
//   // ecm->SetJointAngleWithName("maininsertionlink-toollink", M_PI_2);
//   // for(int i = 0; i < Q.size(); i++)
//   // {
//   //   Q[i] = M_PI_4;
//   // }

//   // Q[0] = M_PI_4;
//   // Q[1] = 0.0f;
  
//   // if(!ecm->ExecutePose()) return;


//   // std::map< std::string, unsigned int > rbdlmBodyMap = rbdlModel->mBodyNameMap;
//   // std::map<std::string, unsigned int>::iterator rbdlmBodyMapItr;
// 	// for(rbdlmBodyMapItr = rbdlmBodyMap.begin(); rbdlmBodyMapItr != rbdlmBodyMap.end(); rbdlmBodyMapItr++)
//   // {
//   //   // std::string bodyName = "pitchbacklink-pitchbottomlink";
//   //   std::string bodyName = rbdlmBodyMapItr->first;
//   //   unsigned int bodyId = rbdlmBodyMapItr->second;
//   //   if(bodyId == 0) continue;

//   //   printf("bodyName: %s\n", bodyName.c_str());

//   //   t_w_nPtr t_w_nptr = ecm->twnFromModels(bodyName);
//   //   if(t_w_nptr == nullptr) continue;
    

//   //   // std::cout << std::endl << "r_w_n_ambf: " << std::endl << t_w_nptr->r_w_n_ambf << std::endl;
//   //   // std::cout << std::endl << "r_w_n_rbdl: " << std::endl << t_w_nptr->r_w_n_rbdl << std::endl;

//   //   // std::cout << std::endl << "p_w_n_ambf: " << std::endl << t_w_nptr->p_w_n_ambf << std::endl;
//   //   // std::cout << std::endl << "p_w_n_rbdl: " << std::endl << t_w_nptr->p_w_n_rbdl << std::endl;
    
//   //   CHECK_THAT (t_w_nptr->p_w_n_ambf, AllCloseVector(t_w_nptr->p_w_n_rbdl, TEST_PREC, TEST_PREC));
//   //   // std::cout << "---------------------------------\n";
//   // }
// }
