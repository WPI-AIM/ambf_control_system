#include "rbdl_model_tests/RBDLTestPrep.h"
#include "rbdl_model_tests/KUKA.h"
#include "rbdl_model_tests/EigenUtilities.h"

KUKA* kuka = nullptr;
Model* rbdlModel = nullptr;

TEST_CASE(__FILE__"_Initilize", "") 
{
  kuka = new KUKA();
  rbdlModel = kuka->GetRBDLModel();
}

TEST_CASE(__FILE__"_KUKAFKTest", "") 
{
  // KUKA* kuka = new KUKA();
  // Model* rbdlModel = kuka->GetRBDLModel();

	VectorNd Q     = VectorNd::Constant ((size_t) rbdlModel->dof_count, 0.);
	VectorNd QDot  = VectorNd::Constant ((size_t) rbdlModel->dof_count, 0.);
	VectorNd QDDot = VectorNd::Constant ((size_t) rbdlModel->dof_count, 0.);
	VectorNd Tau   = VectorNd::Constant ((size_t) rbdlModel->dof_count, 0.);

  Q.setZero();
  for(int i = 3; i < Q.size(); i++)
  {
    Q[i] = M_PI_2;
  }
  // Q[0] = M_PI; // Z
  // Q[1] = M_PI_2; // Y
  // Q[2] = -M_PI_2; // X
  // Q[3] = M_PI_2;
  // Q[4] = M_PI_2;
  // Q[4] = M_PI_2;
  // Q[5] = M_PI_2;
  std::cout << "Q" << std::endl << Q << std::endl;

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

  // CHECK_THAT (t_w_nptr->r_w_n_ambf, AllCloseMatrix(t_w_nptr->r_w_n_rbdl, TEST_PREC, TEST_PREC));  
  // CHECK_THAT (t_w_nptr->p_w_n_ambf, AllCloseVector(t_w_nptr->p_w_n_rbdl, TEST_PREC, TEST_PREC));
  }
}
// TEST_CASE(__FILE__"_KUKAFKTestMannual", "") 
// {
//   Vector3d b_l1PA = { 00.000, 00.000, 01.000 };
//   Vector3d b_l1CA = { 00.000, 00.000, 01.000 };

// 	Vector3d l1_l2PA = { 00.000, 01.000, 00.000 };
// 	Vector3d l1_l2CA = { 00.000, 00.000, 01.000 };

// 	Vector3d l2_l3PA = { 00.000, -1.000, 00.000 };
// 	Vector3d l2_l3CA = { 00.000, 00.000, 01.000 };

// 	Vector3d l3_lPA = { 00.000, -1.000, 00.000 };
// 	Vector3d l3_lCA = { 00.000, 00.000, 01.000 };

// 	Vector3d l4_l5PA = { 00.000, 01.000, 00.000 };
// 	Vector3d l4_l5CA = { 00.000, 00.000, 01.000 };
	
// 	Vector3d l5_link6PA = { 00.000, 01.000, 00.000 };
// 	Vector3d l5_link6CA = { 00.000, 00.000, 01.000 };
		
// 	Vector3d l6_l7PA = { 00.000, -1.000, 00.000 };
// 	Vector3d l6_l7CA = { 00.000, 00.000, 01.000 };

//   EigenUtilities eu;
//   Matrix3d r_w_b = eu.SetAlmostZeroToZero<Matrix3d>(eu.RPYToMatrix(Vector3d(0, 0, M_PI_2)));
//   std::cout << "r_w_b: " << std::endl << r_w_b << std::endl;

//   // Matrix3d r_w_b = eu.SetAlmostZeroToZero<Matrix3d>(eu.RPYToMatrix(Vector3d(-M_PI_2, 0, 0)).transpose());
//   // Vector3d p_w_b = Vector3d(0.1, 0.2, -0.3); 
//   // // ---------------------------- //
//   // Matrix3d r_b_l1 = Eigen::Matrix3d(Eigen::Quaterniond::FromTwoVectors(b_l1PA, b_l1CA));
//   // Vector3d p_b_l1 = Vector3d(0.0, 0.0, 0.103); 
//   // Matrix3d r_b_l1_body = eu.RPYToMatrix(Vector3d(0, M_PI_2, 0));
  
//   // Matrix3d r_w_l1 = eu.SetAlmostZeroToZero<Matrix3d>(
//   //   r_w_b * ( r_b_l1_body * r_b_l1).transpose()
//   // );
//   // Vector3d p_w_l1 = eu.SetAlmostZeroToZero<Vector3d>(p_w_b + 
//   //   (r_w_b * r_b_l1_body).transpose() * p_b_l1);
//   // // ---------------------------- //
//   // Matrix3d r_l1_l2 = Eigen::Matrix3d(Eigen::Quaterniond::FromTwoVectors(l1_l2PA, l1_l2CA));
//   // Matrix3d r_l1_l2_body = eu.RPYToMatrix(Vector3d(0, 0, M_PI_2));

//   // Matrix3d r_w_l2 = eu.SetAlmostZeroToZero<Matrix3d>(
//   //   r_w_l1 * eu.RPYToMatrix(Vector3d(0, 0, M_PI_2)) //* eu.RPYToMatrix(Vector3d(0, -M_PI_2, 0))
//   // );
//   // // * r_l1_l2.transpose())

//   // const Matrix3d r_w_b_ambf(1, 0, 0, 0, 0, -1, 0,  1,  0);
//   // CHECK_THAT (r_w_b_ambf, AllCloseMatrix(r_w_b, TEST_PREC, TEST_PREC));  
//   // CHECK_THAT (p_w_b, AllCloseVector(p_w_b, TEST_PREC, TEST_PREC));

//   // const Matrix3d r_w_l1_ambf(0, 0, -1.0, -1.0, 0, 0, 0, 1, 0);
//   // const Vector3d p_w_l1_ambf(0.1, 0.303, -0.3);

//   // CHECK_THAT (r_w_l1_ambf, AllCloseMatrix(r_w_l1, TEST_PREC, TEST_PREC));  
//   // CHECK_THAT (p_w_l1_ambf, AllCloseVector(p_w_l1, TEST_PREC, TEST_PREC));

//   // const Matrix3d r_w_l2_ambf(0, -1, 0, 0, 0, 1, -1, 0, 0);
//   // // const Vector3d p_w_l1_ambf(0.1, 0.303, -0.3);

//   // CHECK_THAT (r_w_l2_ambf, AllCloseMatrix(r_w_l2, TEST_PREC, TEST_PREC));  
//   // // CHECK_THAT (p_w_l1_ambf, AllCloseVector(p_w_l1, TEST_PREC, TEST_PREC));
// }
/*
TEST_CASE(__FILE__"_KUKAIKTest", "") 
{
	VectorNd Q     = VectorNd::Constant ((size_t) rbdlModel->dof_count, 0.);
	VectorNd QDot  = VectorNd::Constant ((size_t) rbdlModel->dof_count, 0.);
	VectorNd QDDot = VectorNd::Constant ((size_t) rbdlModel->dof_count, 0.);
	VectorNd Tau   = VectorNd::Constant ((size_t) rbdlModel->dof_count, 0.);

  std::vector<unsigned int> body_ids;
  std::vector<Vector3d> body_points;
  std::vector<Vector3d> target_pos;
  // Q.setZero();
  // Q[0] = 0.2;
  // Q[1] = 0.1;
  // Q[2] = 0.1;

  // Q[0] = M_PI_4;
  // Q[1] = M_PI_4;
  // Q[2] = 0.0;
  
  VectorNd Qres = VectorNd::Zero ((size_t) rbdlModel->dof_count);

  unsigned int base_link1Id = rbdlModel->GetBodyId("base-link1");
  unsigned int link1_link2Id = rbdlModel->GetBodyId("link1-link2");
  // unsigned int link2_link3Id = rbdlModel->GetBodyId("link2-link3");
  // unsigned int link3_link4Id = rbdlModel->GetBodyId("link3-link4");
  // unsigned int link4_link5Id = rbdlModel->GetBodyId("link4-link5");
  // unsigned int link5_link6Id = rbdlModel->GetBodyId("link5-link6");
  // unsigned int link6_link7Id = rbdlModel->GetBodyId("link6-link7");
  Vector3d body_point = Vector3d (0., 0., 0.);
  Vector3d target (-0.00919774, 0.0091977, -0.988082);

  body_ids.push_back (link1_link2Id);
  body_points.push_back (body_point);
  target_pos.push_back (target);

  ClearLogOutput();
  bool res = InverseKinematics (*rbdlModel, Q, body_ids, body_points,
                                target_pos, Qres, 1.0e-8, 0.9, 1000);
  
  // bool res = InverseKinematics (*model, Q, body_ids, body_points, target_pos,
  //                               Qres);
  std::cout << "Qres" << std::endl << Qres << std::endl;

  //	cout << LogOutput.str() << endl;
  CHECK (true == res);

  UpdateKinematicsCustom (*rbdlModel, &Qres, NULL, NULL);

  Vector3d effector;
  effector = CalcBodyToBaseCoordinates(*rbdlModel, Qres, link1_link2Id, body_point, false);

  Vector3d p_w_link1 = CalcBodyToBaseCoordinates(*rbdlModel,
                                          Q,
                                          base_link1Id,
                                          Vector3d (0., 0., 0.),
                                          true);
  
  Vector3d p_w_link2 = CalcBodyToBaseCoordinates(*rbdlModel,
                                          Q,
                                          link1_link2Id,
                                          Vector3d (0., 0., 0.),
                                          true);

  // Vector3d p_w_link3 = CalcBodyToBaseCoordinates(*rbdlModel,
  //                                         Q,
  //                                         link2_link3Id,
  //                                         Vector3d (0., 0., 0.),
  //                                         true);

  // Vector3d p_w_link4 = CalcBodyToBaseCoordinates(*rbdlModel,
  //                                         Q,
  //                                         link3_link4Id,
  //                                         Vector3d (0., 0., 0.),
  //                                         true);

  // Vector3d p_w_link5 = CalcBodyToBaseCoordinates(*rbdlModel,
  //                                         Q,
  //                                         link4_link5Id,
  //                                         Vector3d (0., 0., 0.),
  //                                         true);

  // Vector3d p_w_link6 = CalcBodyToBaseCoordinates(*rbdlModel,
  //                                         Q,
  //                                         link5_link6Id,
  //                                         Vector3d (0., 0., 0.),
  //                                         true);

  // Vector3d p_w_link7 = CalcBodyToBaseCoordinates(*rbdlModel,
  //                                         Q,
  //                                         link6_link7Id,
  //                                         Vector3d (0., 0., 0.),
  //                                         true);


  std::cout << "p_w_link1" << std::endl << p_w_link1 << std::endl << std::endl;
  std::cout << "p_w_link2" << std::endl << p_w_link2 << std::endl << std::endl;
  // std::cout << "p_w_link3" << std::endl << p_w_link3 << std::endl << std::endl;
  // std::cout << "p_w_link4" << std::endl << p_w_link4 << std::endl << std::endl;
  // std::cout << "p_w_link5" << std::endl << p_w_link5 << std::endl << std::endl;
  // std::cout << "p_w_link6" << std::endl << p_w_link6 << std::endl << std::endl;
  // std::cout << "p_w_link7" << std::endl << p_w_link7 << std::endl << std::endl;

  std::cout << "effector" << std::endl << effector << std::endl << std::endl;



}
*/
TEST_CASE(__FILE__"_Cleanup", "") 
{
  kuka->CleanUp();
}