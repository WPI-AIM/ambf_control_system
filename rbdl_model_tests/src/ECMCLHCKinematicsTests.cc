#include "rbdl_model_tests/ECMCL.h"
#include "rbdl_model_tests/rbdl_tests.h"
ECM* ecm = nullptr;

TEST_CASE(__FILE__"_Initilize", "") 
{
  ecm = new ECM();  
}

// TEST_CASE_METHOD(ECM, __FILE__"_TestECMBodyHierarchy", "") 
// {
//   if(ecm == nullptr) return;
// }

TEST_CASE(__FILE__"_TestHomePose", "") 
{
  if(ecm == nullptr) return;
  std::vector<t_w_nPtr> transformations = ecm->HomePoseTransformation();
  for(t_w_nPtr t_w_nptr : transformations)
  {
    std::cout << "body Name: " << t_w_nptr->bodyName << std::endl;
    std::cout << "t_w_nptr->p_w_n_ambf" << std::endl << t_w_nptr->p_w_n_ambf << std::endl;
    std::cout << "t_w_nptr->p_w_n_rbdl" << std::endl << t_w_nptr->p_w_n_rbdl << std::endl;

    // std::cout << "t_w_nptr->r_w_n_ambf" << std::endl << t_w_nptr->r_w_n_ambf << std::endl;
    // CHECK_THAT (t_w_nptr->r_w_n_ambf, AllCloseMatrix(t_w_nptr->r_w_n_rbdl, TEST_PREC, TEST_PREC));
    CHECK_THAT (t_w_nptr->p_w_n_ambf, AllCloseVector(t_w_nptr->p_w_n_rbdl, TEST_PREC, TEST_PREC));
    std::cout << "------------------------\n";
  }
}

TEST_CASE(__FILE__"_RandomPose", "") 
{
  if(ecm == nullptr) return;
  // std::vector<std::string> jointNames = ecm->ControllableJointNames();
  // for(std::string jointName : jointNames)
  //   std::cout << jointName + ", ";
  // std::cout << std::endl;

  // ecm->JointAngleWithName("baselink-yawlink", M_PI_4);
  // ecm->JointAngleWithName("yawlink-pitchbacklink", 1.0562572479248047);
  // ecm->JointAngleWithName("pitchbacklink-pitchbottomlink", -1.0471999645233154);
  // ecm->JointAngleWithName("yawlink-pitchfrontlink", 1.0569205284118652);
  // ecm->JointAngleWithName("pitchfrontlink-pitchtoplink", -1.04469895362854);
  // ecm->JointAngleWithName("pitchbottomlink-pitchendlink", 1.0532910823822021);
  // ecm->JointAngleWithName("pitchfrontlink-pitchbottomlink", -1.0478633642196655);
  // ecm->JointAngleWithName("pitchtoplink-pitchendlink", 1.0501266717910767);
  
  float qDesired = 1.0562572479248047;
  ecm->JointAngleWithName("yawlink-pitchbacklink", qDesired);
  ecm->JointAngleWithName("pitchbacklink-pitchbottomlink", -qDesired);
  ecm->JointAngleWithName("yawlink-pitchfrontlink", qDesired);
  ecm->JointAngleWithName("pitchfrontlink-pitchtoplink", -qDesired);
  ecm->JointAngleWithName("pitchbottomlink-pitchendlink", qDesired);
  ecm->JointAngleWithName("pitchfrontlink-pitchbottomlink", -qDesired);
  ecm->JointAngleWithName("pitchtoplink-pitchendlink", qDesired);

  // ecm->JointAngleWithName("pitchendlink-maininsertionlink", 0.25);
  // ecm->JointAngleWithName("maininsertionlink-toollink", M_PI_4);

  VectorNd QTarget = ecm->TargetJointAngles();
  std::cout << "QTarget" << std::endl << QTarget << std::endl;

  ecm->ExecutePose();
  t_w_nPtr t_w_nptr;
  // const std::string jointName = ;
  // t_w_nPtr t_w_nptr = ecm->twnFromModels<std::string>("world-baselink");
  // CHECK_THAT (t_w_nptr->p_w_n_ambf, AllCloseVector(t_w_nptr->p_w_n_rbdl, TEST_PREC, TEST_PREC));

  // t_w_nptr = ecm->twnFromModels<std::string>("baselink-yawlink");
  // CHECK_THAT (t_w_nptr->p_w_n_ambf, AllCloseVector(t_w_nptr->p_w_n_rbdl, TEST_PREC, TEST_PREC));

  // t_w_nptr = ecm->twnFromModels<std::string>("yawlink-pitchbacklink");
  // CHECK_THAT (t_w_nptr->p_w_n_ambf, AllCloseVector(t_w_nptr->p_w_n_rbdl, TEST_PREC, TEST_PREC));

  // t_w_nptr = ecm->twnFromModels<std::string>("yawlink-pitchfrontlink");
  // CHECK_THAT (t_w_nptr->p_w_n_ambf, AllCloseVector(t_w_nptr->p_w_n_rbdl, TEST_PREC, TEST_PREC));
  
  // t_w_nptr = ecm->twnFromModels<std::string>("pitchfrontlink-pitchtoplink");
  // CHECK_THAT (t_w_nptr->p_w_n_ambf, AllCloseVector(t_w_nptr->p_w_n_rbdl, TEST_PREC, TEST_PREC));
  // -----------------Test case above this line passes----------------------
  t_w_nptr = ecm->twnFromModels<std::string>("pitchbacklink-pitchbottomlink");
  CHECK_THAT (t_w_nptr->p_w_n_ambf, AllCloseVector(t_w_nptr->p_w_n_rbdl, TEST_PREC, TEST_PREC));
  
  // t_w_nptr = ecm->twnFromModels<std::string>("pitchtoplink-pitchendlink");
  // CHECK_THAT (t_w_nptr->p_w_n_ambf, AllCloseVector(t_w_nptr->p_w_n_rbdl, TEST_PREC, TEST_PREC));


  unsigned int qSize = ecm->RBDLModelJointSize();
  
  // for(unsigned int qId = 1; qId <= qSize; qId++)
  // {
    // const std::string jointName = "baselink-yawlink";
    // int qId = 6;
    // t_w_nPtr t_w_nptr = ecm->twnFromModels<unsigned int>(qId);

    // std::cout << "jointId: " << qId << ", bodyName: " << t_w_nptr->bodyName << std::endl;
    // std::cout << "t_w_nptr->r_w_n_ambf: " << std::endl << t_w_nptr->r_w_n_ambf << std::endl;
    // std::cout << "t_w_nptr->r_w_n_rbdl: " << std::endl << t_w_nptr->r_w_n_rbdl << std::endl;

    // std::cout << "t_w_nptr->p_w_n_ambf: " << std::endl << t_w_nptr->p_w_n_ambf << std::endl;
    // std::cout << "t_w_nptr->p_w_n_rbdl: " << std::endl << t_w_nptr->p_w_n_rbdl << std::endl;
    
    // CHECK_THAT (t_w_nptr->r_w_n_ambf, AllCloseMatrix(t_w_nptr->r_w_n_rbdl, TEST_PREC, TEST_PREC));
    // CHECK_THAT (t_w_nptr->p_w_n_ambf, AllCloseVector(t_w_nptr->p_w_n_rbdl, TEST_PREC, TEST_PREC));
    // std::cout << "--------------------------\n";
  // }
}

// // TEST_CASE(__FILE__"_ECMIK", "") 
// // {
// //   RBDLModelPtr rbdlModelPtr = ecm->RbdlModel();

// //   RigidBodyDynamics::Math::VectorNd q;
// //   RigidBodyDynamics::Math::VectorNd qdot;
// //   RigidBodyDynamics::Math::VectorNd qddot;
// //   RigidBodyDynamics::Math::VectorNd tau;
  
// //   q     = VectorNd::Zero (rbdlModelPtr->q_size);
// //   qdot  = VectorNd::Zero (rbdlModelPtr->qdot_size);
// //   qddot = VectorNd::Zero (rbdlModelPtr->qdot_size);
// //   tau   = VectorNd::Zero (rbdlModelPtr->qdot_size);

// //   unsigned int world_baselinkId                 = rbdlModelPtr->GetBodyId("world-baselink");
// //   unsigned int baselink_yawlinkId               = rbdlModelPtr->GetBodyId("baselink-yawlink");
// //   unsigned int yawlink_pitchbacklinkId          = rbdlModelPtr->GetBodyId("yawlink-pitchbacklink");
// //   unsigned int pitchbacklink_pitchbottomlinkId  = rbdlModelPtr->GetBodyId("pitchbacklink-pitchbottomlink");
// //   unsigned int pitchbottomlink_pitchendlinkId   = rbdlModelPtr->GetBodyId("pitchbottomlink-pitchendlink");
// //   unsigned int pitchendlink_maininsertionlinkId = rbdlModelPtr->GetBodyId("pitchendlink-maininsertionlink");
// //   unsigned int maininsertionlink_toollinkId     = rbdlModelPtr->GetBodyId("maininsertionlink-toollink");

// //   unsigned int body_id_emulated[baselink_yawlinkId];
  
// //   UpdateKinematicsCustom (*rbdlModelPtr, &q, NULL, NULL);
// //   InverseKinematicsConstraintSet cs;
// //   q.setZero();
// //   q[3] = M_PI_2;
// //   q[4] = 0.1;
// //   q[5] = 0.2;
// //   q[6] = M_PI_4;
// //   // std::cout << "q:" << std::endl << q << std::endl;
// //   Vector3d body_point(0., 0., 0.);

// // 	// Skip ROOT which has bodyId = 0
// // 	for(unsigned int bodyId = 1; bodyId < rbdlModelPtr->q_size - 1; bodyId++)
// //   {
// //     cs.AddFullConstraint(bodyId, body_point, 
// //     CalcBodyToBaseCoordinates(*rbdlModelPtr, q, bodyId, body_point, true), 
// //      CalcBodyWorldOrientation(*rbdlModelPtr, q, bodyId, false));
// //   }

// //   // cs.AddFullConstraint(1, body_point, 
// //   //   // Vector3d(0.4999, -0.3901, -0.599),
// //   //   CalcBodyToBaseCoordinates(*rbdlModelPtr, q, 1, body_point, true), 
// //   //    CalcBodyWorldOrientation(*rbdlModelPtr, q, 1, false));

// //   // cs.AddFullConstraint(2, body_point,  
// //   //   // Vector3d(0.499893, -0.927001, -0.599002), 
// //   //   CalcBodyToBaseCoordinates(*rbdlModelPtr, q, 2, body_point, true),
// //   //    CalcBodyWorldOrientation(*rbdlModelPtr, q, 2, false));

// //   // cs.AddFullConstraint(3, body_point, 
// //   //   // Vector3d(0.499924, -0.764606, -0.608803), 
// //   //   CalcBodyToBaseCoordinates(*rbdlModelPtr, q, 3, body_point, true),
// //   //    CalcBodyWorldOrientation(*rbdlModelPtr, q, 3, false));

// //   // cs.AddFullConstraint(4, body_point, 
// //   //   // Vector3d(0.499476, -0.831215, -0.312378), 
// //   //   CalcBodyToBaseCoordinates(*rbdlModelPtr, q, 4, body_point, true),
// //   //    CalcBodyWorldOrientation(*rbdlModelPtr, q, 4, false));

// //   // cs.AddFullConstraint(5, body_point, 
// //   //   // Vector3d(0.500136, -0.491126, -0.312304), 
// //   //   CalcBodyToBaseCoordinates(*rbdlModelPtr, q, 5, body_point, true),
// //   //    CalcBodyWorldOrientation(*rbdlModelPtr, q, 5, false));

// //   // cs.AddFullConstraint(6, body_point, 
// //   //   // Vector3d(0.500079, -0.450094, -0.228623), 
// //   //   CalcBodyToBaseCoordinates(*rbdlModelPtr, q, 6, body_point, true),
// //   //    CalcBodyWorldOrientation(*rbdlModelPtr, q, 6, false));

// //   // cs.AddFullConstraint(7, body_point, 
// //   //   // Vector3d(0.500289, -0.388191, -0.229572), 
// //   //   CalcBodyToBaseCoordinates(*rbdlModelPtr, q, 7, body_point, true),
// //   //    CalcBodyWorldOrientation(*rbdlModelPtr, q, 7, false));

// //   VectorNd qres (q);

// //   bool result = InverseKinematics (*rbdlModelPtr, q, cs, qres);

// //   CHECK (result);

// //   CHECK_THAT (0., IsClose(cs.error_norm, TEST_PREC, TEST_PREC));

// //   CHECK_THAT (q, AllCloseVector(qres, TEST_PREC, TEST_PREC));
// //   // std::cout << "qres:" << std::endl << qres << std::endl;
// // }

TEST_CASE(__FILE__"_Cleanup", "") 
{ 
  if(ecm == nullptr) return;
  ecm->CleanUp();
}

// double getAngle(Matrix3d r)
// {
//   return acos((r.trace() - 1) / 2.0);
// }

// Vector3d getAxis(Matrix3d r, double t)
// {
//   std::cout << "t: " << t << std::endl;
  
//   if(t == 0) return Vector3d(0, 0, 1);

//   Vector3d k(r(2, 1) - r(1, 2), r(0, 2) - r(2, 0), r(1, 0) - r(0, 1));
//   k *= (1.0 / (2.0 * sin(t)));
//   return k;
// }

// TEST_CASE(__FILE__"_RoationCheck", "") 
// {  
//   // Matrix3d r_world_base_world;
//   // r_world_base_world.setIdentity();

//   // double r_world_base_world_t = getAngle(r_world_base_world);
//   // Vector3d world_base_world_k = getAxis(r_world_base_world, r_world_base_world_t);
//   // std::cout << "world_base_world_k" << std::endl << world_base_world_k << std::endl;

//   Matrix3d r_base_yaw_world(-1, 0, 0,   0, 0, 1,  0, 1, 0);

//   double base_yaw_world_t = getAngle(r_base_yaw_world);
//   Vector3d base_yaw_world_k = getAxis(r_base_yaw_world, base_yaw_world_t);
//   std::cout << "base_yaw_world_k" << std::endl << base_yaw_world_k << std::endl;

// }

// TEST_CASE(__FILE__"_RoationCheck", "") 
// {  

//   Matrix3d r_w_b;
//   Matrix3d r_w_y;
//   Matrix3d r_w_pba;
//   Matrix3d r_w_pbo;
//   Matrix3d r_w_pend;
//   Matrix3d r_w_m;
//   Matrix3d r_w_tool;

//   Matrix3d r_b_yST;
//   Matrix3d r_y_pbaST;
//   Matrix3d r_pba_pboST;
//   Matrix3d r_pbo_pendST;
//   Matrix3d r_pend_mST;
//   Matrix3d r_m_toolST;

//   // 1 base
//   r_w_b.setIdentity();

  
//   Matrix3d b_yRot(
//     1, 0, 0, 
//     0, 0, -1,
//     0, 1, 0
//     );

//   Matrix3d b_yRotOffset(
//     -1, 0, 0, 
//     0,-1, 0,
//     0, 0, 1
//   );

//   r_b_yST = b_yRot.transpose() * b_yRotOffset;
//   r_w_y = r_w_b * r_b_yST;

//   Matrix3d r_w_y_(
//     -1, 0, 0, 
//     0, 0, 1,
//     0, 1, 0
//   );

//   CHECK_THAT (r_w_y_, AllCloseMatrix(r_w_y, TEST_PREC, TEST_PREC));

//   // 3 pitchback
//   Matrix3d y_pbaRot(
//       0, 0, -1,
//       0, 1, 0,
//       1, 0, 0
//   );

//   Matrix3d y_pbaRotOffset(
//     -1, 0, 0,
//     0,-1, 0,
//     0, 0, 1
//   );

//   r_y_pbaST = y_pbaRot.transpose() * y_pbaRotOffset;
  
//   // Matrix3d r_y_pbST_(
//   //   0, 0, 1,
//   //   0,-1, 0,
//   //   1, 0, 0
//   // );
//   // CHECK_THAT (r_y_pbST_, AllCloseMatrix(r_y_pbST, TEST_PREC, TEST_PREC));
//   r_w_pba = r_w_y * r_y_pbaST;

//   Matrix3d r_w_pba_(
//   0, 0, -1,
//   1, 0,  0,
//   0, -1, 0
//   );

//   CHECK_THAT (r_w_pba_, AllCloseMatrix(r_w_pba, TEST_PREC, TEST_PREC));

//   // 4 pitchbottom
//   Matrix3d pba_pboRot;
//   pba_pboRot.setIdentity();

//   Matrix3d pba_pboRotOffset;
//   pba_pboRotOffset.setIdentity();

//   r_pba_pboST = pba_pboRot.transpose() * pba_pboRotOffset;
  
//   r_w_pbo = r_w_pba * r_pba_pboST;

//   Matrix3d r_w_pbo_(
//     0, 0, -1,
//     1, 0, 0,
//     0, -1, 0
//   );

//   CHECK_THAT (r_w_pbo_, AllCloseMatrix(r_w_pbo, TEST_PREC, TEST_PREC));

//   // 5 pitchend
//   Matrix3d pbo_pendRot;
//   pbo_pendRot.setIdentity();

//   Matrix3d pbo_pendRotOffset;
//   pbo_pendRotOffset.setIdentity();

//   r_pbo_pendST = pbo_pendRot.transpose() * pbo_pendRotOffset;
  
//   r_w_pend = r_w_pbo * r_pbo_pendST;

//   Matrix3d r_w_end_(
//     0, 0, -1,
//     1, 0, 0,
//     0, -1, 0
//   );

//   CHECK_THAT (r_w_end_, AllCloseMatrix(r_w_pend, TEST_PREC, TEST_PREC));

//   // 6 maininsertion
//   Matrix3d pend_mRot(
//   0, 1, 0,
//   -1, 0, 0,
//   -0, 0, 1
//   );

//   Matrix3d pend_mRotOffset;
//   pend_mRotOffset.setIdentity();

//   r_pend_mST = pend_mRot.transpose() * pend_mRotOffset;
  
//   r_w_m = r_w_pend * r_pend_mST;

//   Matrix3d r_w_m_(
//   0, 0, -1,
//   0, -1, 0,
//   -1, 0, 0
//   );

//   CHECK_THAT (r_w_m_, AllCloseMatrix(r_w_m, TEST_PREC, TEST_PREC));

//   // pitchtool
//   Matrix3d m_toolRot(
//     0, 0, 1,
//     0, 1, 0,
//     -1, 0, 0
//   );

//   Matrix3d m_toolRotOffset(
//     0, -1, 0,
//     1,  0, 0,
//     0,  0, 1
//   );

//   r_m_toolST = m_toolRot.transpose() * m_toolRotOffset;

//   // Matrix3d r_m_toolST_(
//   //   0, 0, -1,
//   //   1, 0, 0,
//   //   0, -1, 0
//   // );
//   // CHECK_THAT (r_m_toolST_, AllCloseMatrix(r_m_toolST, TEST_PREC, TEST_PREC));

//   r_w_tool = r_w_m * r_m_toolST;

//   Matrix3d r_w_tool_(
//   0, 1, 0,
//   -1, 0, 0,
//    0, 0, 1
//   );

//   CHECK_THAT (r_w_tool_, AllCloseMatrix(r_w_tool, TEST_PREC, TEST_PREC));
// }
