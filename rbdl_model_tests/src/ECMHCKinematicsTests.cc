#include "rbdl_model_tests/RBDLTestPrep.h"
#include "rbdl_model_tests/ECM.h"
#include "rbdl_model_tests/EigenUtilities.h"

ECM* ecm = nullptr;


TEST_CASE(__FILE__"_Initilize", "") 
{
  ecm = new ECM();  
}

// TEST_CASE(__FILE__"_TestHomePose", "") 
// {
//   std::vector<t_w_nPtr> transformations = ecm->HomePoseTransformation();
//   for(t_w_nPtr t_w_nptr : transformations)
//   {
//     CHECK_THAT (t_w_nptr->r_w_n_ambf, AllCloseMatrix(t_w_nptr->r_w_n_rbdl, TEST_PREC, TEST_PREC));
//     CHECK_THAT (t_w_nptr->p_w_n_ambf, AllCloseVector(t_w_nptr->p_w_n_rbdl, TEST_PREC, TEST_PREC));
//   }
// }

// TEST_CASE(__FILE__"_RandomPose", "") 
// {
//   std::vector<std::string> jointNames = ecm->RBDLJointNames();
//   // for(std::string jointName : jointNames)
//   // {
//   //   ecm->JointAngleWithName(jointName, 1.0);
//   // }
//   ecm->JointAngleWithName(jointNames.at(0), 0.1);
//   ecm->JointAngleWithName(jointNames.at(1), 0.2);
//   ecm->JointAngleWithName(jointNames.at(2), 0.3);
//   ecm->JointAngleWithName(jointNames.at(3), 0.4);

//   VectorNd QTarget = ecm->TargetJointAngles();
//   std::cout << "QTarget" << std::endl << QTarget << std::endl;

//   ecm->ExecutePose();
// }


TEST_CASE(__FILE__"_ECMIK", "") 
{
  RBDLModelPtr rbdlModelPtr = ecm->RbdlModel();

  RigidBodyDynamics::Math::VectorNd q;
  RigidBodyDynamics::Math::VectorNd qdot;
  RigidBodyDynamics::Math::VectorNd qddot;
  RigidBodyDynamics::Math::VectorNd tau;
  
  q     = VectorNd::Zero (rbdlModelPtr->q_size);
  qdot  = VectorNd::Zero (rbdlModelPtr->qdot_size);
  qddot = VectorNd::Zero (rbdlModelPtr->qdot_size);
  tau   = VectorNd::Zero (rbdlModelPtr->qdot_size);

  unsigned int world_baselinkId                 = rbdlModelPtr->GetBodyId("world-baselink");
  unsigned int baselink_yawlinkId               = rbdlModelPtr->GetBodyId("baselink-yawlink");
  unsigned int yawlink_pitchbacklinkId          = rbdlModelPtr->GetBodyId("yawlink-pitchbacklink");
  unsigned int pitchbacklink_pitchbottomlinkId  = rbdlModelPtr->GetBodyId("pitchbacklink-pitchbottomlink");
  unsigned int pitchbottomlink_pitchendlinkId   = rbdlModelPtr->GetBodyId("pitchbottomlink-pitchendlink");
  unsigned int pitchendlink_maininsertionlinkId = rbdlModelPtr->GetBodyId("pitchendlink-maininsertionlink");
  unsigned int maininsertionlink_toollinkId     = rbdlModelPtr->GetBodyId("maininsertionlink-toollink");

  unsigned int body_id_emulated[baselink_yawlinkId];
  
  UpdateKinematicsCustom (*rbdlModelPtr, &q, NULL, NULL);
  InverseKinematicsConstraintSet cs;
  q.setZero();
  q[3] = M_PI_2;
  q[4] = 0.1;
  q[5] = 0.2;
  q[6] = M_PI_4;
  // std::cout << "q:" << std::endl << q << std::endl;
  Vector3d body_point(0., 0., 0.);

	// Skip ROOT which has bodyId = 0
	for(unsigned int bodyId = 1; bodyId < rbdlModelPtr->q_size - 1; bodyId++)
  {
    cs.AddFullConstraint(bodyId, body_point, 
    CalcBodyToBaseCoordinates(*rbdlModelPtr, q, bodyId, body_point, true), 
     CalcBodyWorldOrientation(*rbdlModelPtr, q, bodyId, false));

  }

  // cs.AddFullConstraint(1, body_point, 
  //   // Vector3d(0.4999, -0.3901, -0.599),
  //   CalcBodyToBaseCoordinates(*rbdlModelPtr, q, 1, body_point, true), 
  //    CalcBodyWorldOrientation(*rbdlModelPtr, q, 1, false));

  // cs.AddFullConstraint(2, body_point,  
  //   // Vector3d(0.499893, -0.927001, -0.599002), 
  //   CalcBodyToBaseCoordinates(*rbdlModelPtr, q, 2, body_point, true),
  //    CalcBodyWorldOrientation(*rbdlModelPtr, q, 2, false));

  // cs.AddFullConstraint(3, body_point, 
  //   // Vector3d(0.499924, -0.764606, -0.608803), 
  //   CalcBodyToBaseCoordinates(*rbdlModelPtr, q, 3, body_point, true),
  //    CalcBodyWorldOrientation(*rbdlModelPtr, q, 3, false));

  // cs.AddFullConstraint(4, body_point, 
  //   // Vector3d(0.499476, -0.831215, -0.312378), 
  //   CalcBodyToBaseCoordinates(*rbdlModelPtr, q, 4, body_point, true),
  //    CalcBodyWorldOrientation(*rbdlModelPtr, q, 4, false));

  // cs.AddFullConstraint(5, body_point, 
  //   // Vector3d(0.500136, -0.491126, -0.312304), 
  //   CalcBodyToBaseCoordinates(*rbdlModelPtr, q, 5, body_point, true),
  //    CalcBodyWorldOrientation(*rbdlModelPtr, q, 5, false));

  // cs.AddFullConstraint(6, body_point, 
  //   // Vector3d(0.500079, -0.450094, -0.228623), 
  //   CalcBodyToBaseCoordinates(*rbdlModelPtr, q, 6, body_point, true),
  //    CalcBodyWorldOrientation(*rbdlModelPtr, q, 6, false));

  // cs.AddFullConstraint(7, body_point, 
  //   // Vector3d(0.500289, -0.388191, -0.229572), 
  //   CalcBodyToBaseCoordinates(*rbdlModelPtr, q, 7, body_point, true),
  //    CalcBodyWorldOrientation(*rbdlModelPtr, q, 7, false));

  VectorNd qres (q);

  bool result = InverseKinematics (*rbdlModelPtr, q, cs, qres);

  CHECK (result);

  CHECK_THAT (0., IsClose(cs.error_norm, TEST_PREC, TEST_PREC));

  CHECK_THAT (q, AllCloseVector(qres, TEST_PREC, TEST_PREC));
  // std::cout << "qres:" << std::endl << qres << std::endl;
}

TEST_CASE(__FILE__"_Cleanup", "") 
{  
  ecm->CleanUp();
}

