#include "rbdl_model_tests/RBDLTestPrep.h"
#include "rbdl_model_tests/ECM.h"
#include "rbdl_model_tests/EigenUtilities.h"

// ECM* ecm = nullptr;


// TEST_CASE(__FILE__"_Initilize", "") 
// {
//   ecm = new ECM();  
// }

// // TEST_CASE(__FILE__"_TestHomePose", "") 
// // {
// //   std::vector<t_w_nPtr> transformations = ecm->HomePoseTransformation();
// //   for(t_w_nPtr t_w_nptr : transformations)
// //   {
// //     std::cout << "body Name: " << t_w_nptr->bodyName << std::endl;

// //     CHECK_THAT (t_w_nptr->r_w_n_ambf, AllCloseMatrix(t_w_nptr->r_w_n_rbdl, TEST_PREC, TEST_PREC));
// //     CHECK_THAT (t_w_nptr->p_w_n_ambf, AllCloseVector(t_w_nptr->p_w_n_rbdl, TEST_PREC, TEST_PREC));
// //   }
// // }

// TEST_CASE(__FILE__"_RandomPose", "") 
// {
//   std::vector<std::string> jointNames = ecm->ControllableJointNames();

//   ecm->JointAngleWithName(jointNames.at(0), 0.7872554063796997);
//   // ecm->JointAngleWithName(jointNames.at(1), M_PI_2);
//   ecm->JointAngleWithName(jointNames.at(2), 0.25);
//   ecm->JointAngleWithName(jointNames.at(3), 0.7872554063796997);

//   // VectorNd QTarget = ecm->TargetJointAngles();
//   // std::cout << "QTarget" << std::endl << QTarget << std::endl;

//   ecm->ExecutePose();
//   // const std::string jointName = ;
//   t_w_nPtr t_w_nptr = ecm->twnFromModels<std::string>("world-baselink");
//   CHECK_THAT (t_w_nptr->p_w_n_ambf, AllCloseVector(t_w_nptr->p_w_n_rbdl, TEST_PREC, TEST_PREC));


//   unsigned int qSize = ecm->RBDLModelJointSize();
  
//   for(unsigned int qId = 1; qId <= qSize; qId++)
//   {
//     // const std::string jointName = "baselink-yawlink";
//     // int qId = 6;
//     t_w_nPtr t_w_nptr = ecm->twnFromModels<unsigned int>(qId);

//     // std::cout << "jointId: " << qId << ", bodyName: " << t_w_nptr->bodyName << std::endl;
//     // std::cout << "t_w_nptr->r_w_n_ambf: " << std::endl << t_w_nptr->r_w_n_ambf << std::endl;
//     // std::cout << "t_w_nptr->r_w_n_rbdl: " << std::endl << t_w_nptr->r_w_n_rbdl << std::endl;

//     // std::cout << "t_w_nptr->p_w_n_ambf: " << std::endl << t_w_nptr->p_w_n_ambf << std::endl;
//     // std::cout << "t_w_nptr->p_w_n_rbdl: " << std::endl << t_w_nptr->p_w_n_rbdl << std::endl;
    
//     // CHECK_THAT (t_w_nptr->r_w_n_ambf, AllCloseMatrix(t_w_nptr->r_w_n_rbdl, TEST_PREC, TEST_PREC));
//     CHECK_THAT (t_w_nptr->p_w_n_ambf, AllCloseVector(t_w_nptr->p_w_n_rbdl, TEST_PREC, TEST_PREC));
//     // std::cout << "--------------------------\n";
//   }
// }


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

// TEST_CASE(__FILE__"_Cleanup", "") 
// {  
//   ecm->CleanUp();
// }

double getAngle(Matrix3d r)
{
  return acos((r.trace() - 1) / 2.0);
}

Vector3d getAxis(Matrix3d r, double t)
{
  std::cout << "t: " << t << std::endl;
  
  if(t == 0) return Vector3d(0, 0, 1);

  Vector3d k(r(2, 1) - r(1, 2), r(0, 2) - r(2, 0), r(1, 0) - r(0, 1));
  k *= (1.0 / (2.0 * sin(t)));
  return k;
}

TEST_CASE(__FILE__"_RoationCheck", "") 
{  
  // Matrix3d r_world_base_world;
  // r_world_base_world.setIdentity();

  // double r_world_base_world_t = getAngle(r_world_base_world);
  // Vector3d world_base_world_k = getAxis(r_world_base_world, r_world_base_world_t);
  // std::cout << "world_base_world_k" << std::endl << world_base_world_k << std::endl;

  Matrix3d r_base_yaw_world(-1, 0, 0,   0, 0, 1,  0, 1, 0);

  double base_yaw_world_t = getAngle(r_base_yaw_world);
  Vector3d base_yaw_world_k = getAxis(r_base_yaw_world, base_yaw_world_t);
  std::cout << "base_yaw_world_k" << std::endl << base_yaw_world_k << std::endl;

}
