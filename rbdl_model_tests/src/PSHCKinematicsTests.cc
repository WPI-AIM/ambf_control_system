#include "rbdl_model_tests/ParallelStructure.h"

TEST_CASE_METHOD(ParallelStructure, __FILE__"_TestPSForwardKinematics", "") 
{
  const std::string activationJointName = "l3-l4";
  const float activationJointQdesired = M_PI_4;

  const int activationJointID = baseHandler->get_joint_idx_from_name(activationJointName.c_str());

  ForwardDynamics(*rbdlModel, Q, QDot, Tau, QDDot);
  Q.setZero();
  
  for(RBDL_AMBF_JOINT_MAP_itr = RBDL_AMBF_JOINT_MAP.begin();
      RBDL_AMBF_JOINT_MAP_itr != RBDL_AMBF_JOINT_MAP.end();
      RBDL_AMBF_JOINT_MAP_itr++)
    {
      std::string jointNameRBDL = RBDL_AMBF_JOINT_MAP_itr->first;
      ActivationJoints bodyAMBF = RBDL_AMBF_JOINT_MAP_itr->second;
      bodyAMBF.rigidBodyHandler = 
        clientPtr->getRigidBody(bodyAMBF.bodyNameAMBF, true); 
      RBDL_AMBF_JOINT_MAP[jointNameRBDL] = bodyAMBF;
    }

  usleep(1000000);

  std::map< std::string, unsigned int > mBodyNameMap = rbdlModel->mBodyNameMap;
  std::map<std::string, unsigned int>::iterator mBodyNameMapItr;

  // Check to be a valid joint
  if(baseHandler->is_joint_idx_valid(activationJointID))
  {
    for(int i = 0; i < 5; i++)
    {
      baseHandler->set_joint_pos<std::string>(activationJointName, activationJointQdesired);
      usleep(250000);

      for(mBodyNameMapItr = mBodyNameMap.begin(); 
          mBodyNameMapItr != mBodyNameMap.end(); 
          mBodyNameMapItr++)
      {
        std::string jointNameRBDL = mBodyNameMapItr->first;
        unsigned int jointIDRBDL = mBodyNameMapItr->second;

        // printf("jointNameRBDL: %s, jointIDRBDL: %d\n", jointNameRBDL.c_str(), jointIDRBDL);
        // Skip ROOT body and fixed joints
        if(jointNameRBDL == "ROOT" || jointIDRBDL > Q.size()) continue;

        RBDL_AMBF_JOINT_MAP_itr = RBDL_AMBF_JOINT_MAP.find(jointNameRBDL);
        // Not a vaild AMBF joint
        if(RBDL_AMBF_JOINT_MAP_itr == RBDL_AMBF_JOINT_MAP.end() && 
        baseHandler->get_joint_idx_from_name(jointNameRBDL) == -1) continue;

        float jointQdesired = baseHandler->get_joint_pos<std::string>(jointNameRBDL);        
        int jointIndexRBDL = jointIDRBDL - 1;
        Q[jointIndexRBDL] = jointQdesired;
      }
    }
  }

  for(mBodyNameMapItr = mBodyNameMap.begin(); 
      mBodyNameMapItr != mBodyNameMap.end(); 
      mBodyNameMapItr++)
  {
    std::string jointNameRBDL = mBodyNameMapItr->first;
    unsigned int jointIDRBDL  = mBodyNameMapItr->second;
    
    // Skip ROOT body and fixed joints
    if(jointNameRBDL == "ROOT" || jointIDRBDL > Q.size()) continue;

    // Not a vaild AMBF joint
    RBDL_AMBF_JOINT_MAP_itr = RBDL_AMBF_JOINT_MAP.find(jointNameRBDL);
    if(RBDL_AMBF_JOINT_MAP_itr == RBDL_AMBF_JOINT_MAP.end() && 
    baseHandler->get_joint_idx_from_name(jointNameRBDL) == -1) continue;

    ActivationJoints bodyAMBF = RBDL_AMBF_JOINT_MAP.at(jointNameRBDL);
    rigidBodyPtr rigidbodyAMBF = bodyAMBF.rigidBodyHandler;

    const tf::Quaternion quat_w_n_tf_ambf = rigidbodyAMBF->get_rot();
    const tf::Vector3 P_w_n_tf_ambf = rigidbodyAMBF->get_pos();
    
    const Quaternion quat_w_n_ambf = 
      EigenUtilities::TFtoEigenQuaternion(quat_w_n_tf_ambf);
      
    const Matrix3d R_w_n_ambf = quat_w_n_ambf.toMatrix();
    Eigen::VectorXd P_w_n_ambf = EigenUtilities::TFtoEigenVector(P_w_n_tf_ambf);

    const Eigen::Matrix4d T_w_n_ambf = 
      EigenUtilities::get_frame<Eigen::Matrix3d, 
      Eigen::Vector3d, Eigen::Matrix4d>(R_w_n_ambf, P_w_n_ambf);
    
    const RigidBodyDynamics::Math::Vector3d P_0_n_rbdl = 
      CalcBodyToBaseCoordinates(*rbdlModel, Q, jointIDRBDL, 
        RigidBodyDynamics::Math::Vector3d(0., 0., 0.),true);

    RigidBodyDynamics::Math::Vector4d P_w_n_rbdl_4d;
    P_w_n_rbdl_4d.setOnes();
    P_w_n_rbdl_4d(0) = P_0_n_rbdl(0);
    P_w_n_rbdl_4d(1) = P_0_n_rbdl(1);
    P_w_n_rbdl_4d(2) = P_0_n_rbdl(2);

    P_w_n_rbdl_4d = T_0_w * (P_w_n_rbdl_4d);
    RigidBodyDynamics::Math::Vector3d P_w_n_rbdl;
    P_w_n_rbdl = P_w_n_rbdl_4d.block<3,1>(0,0);

    CHECK_THAT(P_w_n_rbdl, AllCloseMatrix(P_w_n_ambf, TEST_PREC, TEST_PREC));
  }  
  
}
