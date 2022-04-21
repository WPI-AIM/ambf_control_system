#include "rbdl_model_tests/RBDLTestPrep.h"
#include "rbdl_model_tests/KUKA.h"

TEST_CASE(__FILE__"_KUKAHomePoseTest", "") 
{
  KUKA* kuka = new KUKA();
  Model* rbdlModel = kuka->GetRBDLModel();

	VectorNd Q     = VectorNd::Constant ((size_t) rbdlModel->dof_count, 0.);
	VectorNd QDot  = VectorNd::Constant ((size_t) rbdlModel->dof_count, 0.);
	VectorNd QDDot = VectorNd::Constant ((size_t) rbdlModel->dof_count, 0.);
	VectorNd Tau   = VectorNd::Constant ((size_t) rbdlModel->dof_count, 0.);

  Q.setZero();
  for(int i = 0; i < Q.size(); i++)
  {
    Q[i] = M_PI_4;
  }

  kuka->ExecutePose(Q);


  std::map< std::string, unsigned int > rbdlmBodyMap = rbdlModel->mBodyNameMap;
  std::map<std::string, unsigned int>::iterator rbdlmBodyMapItr;
	for(rbdlmBodyMapItr = rbdlmBodyMap.begin(); rbdlmBodyMapItr != rbdlmBodyMap.end(); rbdlmBodyMapItr++)
  {
    std::string bodyName = rbdlmBodyMapItr->first;
    unsigned int bodyId = rbdlmBodyMapItr->second;
    if(bodyId == 0) continue;

    printf("bodyName: %s\n", bodyName.c_str());

    t_w_nPtr t_w_nptr = kuka->twnFromModels(bodyName);
    if(t_w_nptr == nullptr) continue;
    

    std::cout << std::endl << "r_w_n_ambf: " << std::endl << t_w_nptr->r_w_n_ambf << std::endl;
    std::cout << std::endl << "r_w_n_rbdl: " << std::endl << t_w_nptr->r_w_n_rbdl << std::endl;

    std::cout << std::endl << "p_w_n_ambf: " << std::endl << t_w_nptr->p_w_n_ambf << std::endl;
    std::cout << std::endl << "p_w_n_rbdl: " << std::endl << t_w_nptr->p_w_n_rbdl << std::endl;
    std::cout << "---------------------------------\n";
  }
  kuka->CleanUp();
}