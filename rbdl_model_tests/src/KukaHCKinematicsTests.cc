#include "rbdl_model_tests/KUKA.h"

TEST_CASE_METHOD(KUKA, __FILE__"_TestKUKAPositionNeutral", "") 
{
  // We call ForwardDynamics() as it updates the spatial transformation
  // matrices
  // ForwardDynamics(*rbdlModel, Q, QDot, Tau, QDDot);


  // Q.setZero();
  // Q[0] = M_PI_4;
  // Q[1] = M_PI_4;
  // Q[2] = M_PI_4;
  // Q[3] = M_PI_4;
  // Q[4] = M_PI_4;
  // Q[5] = M_PI_4;
  // Q[6] = M_PI_4;

  // std::vector<std::string> baseChildren = baseHandler->get_children_names();

  // std::map< std::string, unsigned int > mBodyNameMap = rbdlModel->mBodyNameMap;
  // std::map<std::string, unsigned int>::iterator mBodyNameMapItr;
  // // Itrerate through the body names. Skipp index 0 which is ROOT
  // for(mBodyNameMapItr = ++(mBodyNameMap.begin()); mBodyNameMapItr != mBodyNameMap.end(); 
  //                                                                           mBodyNameMapItr++)
  // {
  //   std::string jointNameRBDL = mBodyNameMapItr->first;
  //   unsigned int jointIDRBDL = mBodyNameMapItr->second;
  //   int jointIndexRBDL = jointIDRBDL - 1;

  //   // std::cout << jointName_rbdl << ", " << jointID_rbdl << std::endl;
  //   if(baseHandler->get_joint_pos<std::string>(jointNameRBDL) != FLT_MIN)
  //   {
  //     ActivationJoints AMBFactivationJoints = RBDL_AMBF_JOINT_MAP.at(jointNameRBDL);
  //     for(int i = 0; i < 5; i++)
  //     {
  //       baseHandler->set_joint_pos<std::string>
  //                                   (jointNameRBDL, Q[jointIndexRBDL]);
  //       usleep(250000);
  //     }
  //   }
  // }

  // for(mBodyNameMapItr = ++(mBodyNameMap.begin()); mBodyNameMapItr != mBodyNameMap.end(); 
  //                                                                           mBodyNameMapItr++)
  // {
  //   std::string jointNameRBDL = mBodyNameMapItr->first;
  //   unsigned int jointIDRBDL = mBodyNameMapItr->second;
  //   int jointIndexRBDL = jointIDRBDL - 1;

  //   // rigidBodyPtr rigidBodyHandler = clientPtr->getRigidBody(body, true);
  //   std::vector<std::string> rigidBodyNamesAMBF = clientPtr->getRigidBodyNames();
  //   ActivationJoints AMBFactivationJoints = RBDL_AMBF_JOINT_MAP.at(jointNameRBDL);

  //   std::string bodyNameAMBF = AMBFactivationJoints.bodyNameAMBF;
  //   // std::cout << "bodyNameAMBF: " << bodyNameAMBF << std::endl;
  //   if (!std::none_of(rigidBodyNamesAMBF.begin(), rigidBodyNamesAMBF.end(), 
  //                           EigenUtilities::compare(bodyNameAMBF))) 
  //   {
  //     // std::cout << "Element found" << std::endl;
  //     rigidBodyPtr rigidBodyHandler = clientPtr->getRigidBody(bodyNameAMBF, true);
  //     const tf::Quaternion quat_w_n_tf = rigidBodyHandler->get_rot();
  //     const tf::Vector3 P_w_n_tf = rigidBodyHandler->get_pos();

  //     const Eigen::Quaterniond quat_w_n = EigenUtilities::tfToEigenQuaternion(quat_w_n_tf);
  //     const Eigen::Matrix3d R_w_n = quat_w_n.toRotationMatrix();
  //     Eigen::VectorXd P_w_n = EigenUtilities::tfToEigenVector(P_w_n_tf);

  //     const Eigen::Matrix4d T_w_n = EigenUtilities::get_frame<Eigen::Matrix3d, 
  //                 Eigen::Vector3d, Eigen::Matrix4d>(R_w_n, P_w_n);

  //     // std::cout << "T_w_n" << std::endl << T_w_n << std::endl;
  //     const Eigen::Matrix4d T_0_n_AMBF = T_0_w * T_w_n;
  //     const Eigen::Vector3d P_0_n_AMBF = T_0_n_AMBF.block<3, 1>(0, 3);

  //     const RigidBodyDynamics::Math::Vector3d P_0_n_RBDL = 
  //     CalcBodyToBaseCoordinates(*rbdlModel, Q, jointIDRBDL, 
  //        RigidBodyDynamics::Math::Vector3d(0., 0., 0.),true);
  //     CHECK_THAT(P_0_n_RBDL, AllCloseMatrix(P_0_n_AMBF, TEST_PREC, TEST_PREC));
  //   }
  //   else 
  //   {
  //       std::cout << "Element not found" << std::endl;
  //   }
  // }
}