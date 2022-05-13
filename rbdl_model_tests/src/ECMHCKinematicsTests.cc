#include "rbdl_model_tests/RBDLTestPrep.h"
#include "rbdl_model_tests/ECM.h"
#include "rbdl_model_tests/EigenUtilities.h"

// TEST_CASE(__FILE__"TestRotation", "") 
// {
//   EigenUtilities eu;
  
//   for(int i = 0; i < 10; i++)
//   {
//     Vector3d v1;
//     for(int i = 0; i < 3; i++)
//       v1(i) = eu.RandomNumber(0.0, M_2_PI);
//     std::cout << "v1" << std::endl << v1 << std::endl;

//     const Matrix3d m1 = eu.RPYToMatrix(v1);
//     const Vector3d v1_ = eu.MatrixToRPY(m1);

//     CHECK_THAT (v1, AllCloseVector(v1_, TEST_PREC, TEST_PREC));
//   }
// }


ECM* ecm = nullptr;
RBDLModelPtr rbdlModel = nullptr;

TEST_CASE(__FILE__"_Initilize", "") 
{
  ecm = new ECM();
  rbdlModel = ecm->GetRBDLModel();
}

TEST_CASE(__FILE__"_ECMFKTest", "") 
{
  // Eigen::Matrix3d m;
  // const double r = 0.5* M_PI;
  // const double p = 0.25* M_PI;
  // const double y = 0.75* M_PI;
  // printf("r: %lf, p: %lf, y: %lf\n", r, p, y);

  // m = Eigen::AngleAxisd(r, Vector3d::UnitX())
  //   * Eigen::AngleAxisd(p, Vector3d::UnitY())
  //   * Eigen::AngleAxisd(y, Vector3d::UnitZ());
  // std::cout << m << std::endl << "is unitary: " << m.isUnitary() << std::endl;

  if(rbdlModel == nullptr) return;
	VectorNd Q     = VectorNd::Constant ((size_t) rbdlModel->dof_count, 0.);
	VectorNd QDot  = VectorNd::Constant ((size_t) rbdlModel->dof_count, 0.);
	VectorNd QDDot = VectorNd::Constant ((size_t) rbdlModel->dof_count, 0.);
	VectorNd Tau   = VectorNd::Constant ((size_t) rbdlModel->dof_count, 0.);

  Q.setZero();
  // for(int i = 0; i < Q.size(); i++)
  // {
  //   Q[i] = M_PI_4;
  // }

  // Q[0] = M_PI_4;
  // Q[1] = 0.0f;
  
  if(!ecm->ExecutePose(Q)) return;


  std::map< std::string, unsigned int > rbdlmBodyMap = rbdlModel->mBodyNameMap;
  std::map<std::string, unsigned int>::iterator rbdlmBodyMapItr;
	for(rbdlmBodyMapItr = rbdlmBodyMap.begin(); rbdlmBodyMapItr != rbdlmBodyMap.end(); rbdlmBodyMapItr++)
  {
    // std::string bodyName = "pitchbacklink-pitchbottomlink";
    std::string bodyName = rbdlmBodyMapItr->first;
    unsigned int bodyId = rbdlmBodyMapItr->second;
    if(bodyId == 0) continue;

    printf("bodyName: %s\n", bodyName.c_str());

    t_w_nPtr t_w_nptr = ecm->twnFromModels(bodyName);
    if(t_w_nptr == nullptr) continue;
    

    std::cout << std::endl << "r_w_n_ambf: " << std::endl << t_w_nptr->r_w_n_ambf << std::endl;
    std::cout << std::endl << "r_w_n_rbdl: " << std::endl << t_w_nptr->r_w_n_rbdl << std::endl;

    std::cout << std::endl << "p_w_n_ambf: " << std::endl << t_w_nptr->p_w_n_ambf << std::endl;
    std::cout << std::endl << "p_w_n_rbdl: " << std::endl << t_w_nptr->p_w_n_rbdl << std::endl;
    
    CHECK_THAT (t_w_nptr->p_w_n_ambf, AllCloseVector(t_w_nptr->p_w_n_rbdl, TEST_PREC, TEST_PREC));
    std::cout << "---------------------------------\n";
  }
}

TEST_CASE(__FILE__"_Cleanup", "") 
{
  ecm->CleanUp();
}

// TEST_CASE(__FILE__"_TestMatrix", "") 
// {
//   Matrix3d r_w_m(0, 0, -1,  0, -1, 0, -1, 0, 0);
//   Matrix3d r_w_t(0, 1,  0, -1,  0, 0,  0, 0, 1);
  
//   Matrix3d rst_m_t = r_w_m.transpose() * r_w_t;
  
//   std::cout << "rst_m_t" << std::endl << rst_m_t << std::endl;
//   Matrix3d r_w_t_check = r_w_m * rst_m_t;
//   CHECK_THAT (r_w_t, AllCloseMatrix(r_w_t_check, TEST_PREC, TEST_PREC));

//   Matrix3d rRot(0, 0, 1, 0, 1, 0, -1, 0, 0);
//   Matrix3d rOff;
//   rOff = rst_m_t * rRot.transpose();
//   Matrix3d rst_m_t_check = rOff * rRot;
//   CHECK_THAT (rst_m_t, AllCloseMatrix(rst_m_t_check, TEST_PREC, TEST_PREC));
// }
