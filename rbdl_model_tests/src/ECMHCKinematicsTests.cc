#include "rbdl_model_tests/RBDLTestPrep.h"
// #include "rbdl_model_tests/EigenUtilities.h"
#include "rbdl_model_tests/ECM.h"
#include <unordered_map>

//const double TEST_PREC = 1.0e-12;
// const double TEST_LAX = 1.0e-7;

TEST_CASE(__FILE__"_ECMHomePoseTest", "") 
{
  ECM* ecm = new ECM();
  ecm->~ECM();
}



/*
TEST_CASE_METHOD(ECM, __FILE__"_TestECMPositionNeutral", "") 
{
  // We call ForwardDynamics() as it updates the spatial transformation
  // matrices
  ForwardDynamics(*rbdlModel, Q, QDot, Tau, QDDot);

  std::vector<std::string> joints = baseHandler->get_joint_names();
  std::vector<std::string> baseChildren = baseHandler->get_children_names();


  // CHECK (baseChildren.size() == Q.size());

  for(int i = 0; i < 20; i++)
  {
    for(std::string joint : joints)
    {
      baseHandler->set_joint_pos<std::string>(joint, 0.0f);
    }

    usleep(250000);
  }

  Q.setZero();

  for(std::string body : baseChildren)
  {
    unsigned int rbdlBodyId = rbdlModel->GetBodyId(body.c_str());

    if(rbdlBodyId < rbdlModel->mBodies.size())
    {
      rigidBodyPtr rigidBodyHandler = clientPtr->getRigidBody(body, true);
      
      // n - respective body frame
      const tf::Vector3 P_n_w_tf = rigidBodyHandler->get_pos();

      const RigidBodyDynamics::Math::Vector3d P_n_0_rbd_rbdl = CalcBodyToBaseCoordinates(*rbdlModel, Q, rbdlBodyId, 
                                                              RigidBodyDynamics::Math::Vector3d(0., 0., 0.),true);

      RigidBodyDynamics::Math::Vector3d P_n_w_rbd_ambf;
      P_n_w_rbd_ambf.setZero();
      
      P_n_w_rbd_ambf(0) = P_n_w_tf[0];
      P_n_w_rbd_ambf(1) = P_n_w_tf[1];
      P_n_w_rbd_ambf(2) = P_n_w_tf[2];

      RigidBodyDynamics::Math::Vector4d P_n_w_rbd_rbdl_4d;
      P_n_w_rbd_rbdl_4d.setOnes();
      P_n_w_rbd_rbdl_4d(0) = P_n_0_rbd_rbdl(0);
      P_n_w_rbd_rbdl_4d(1) = P_n_0_rbd_rbdl(1);
      P_n_w_rbd_rbdl_4d(2) = P_n_0_rbd_rbdl(2);

      P_n_w_rbd_rbdl_4d = T_0_w * (P_n_w_rbd_rbdl_4d);

      RigidBodyDynamics::Math::Vector3d P_n_w_rbd_rbdl;
      P_n_w_rbd_rbdl = P_n_w_rbd_rbdl_4d.block<3,1>(0,0);
          
      CHECK_THAT(P_n_w_rbd_ambf, AllCloseMatrix(P_n_w_rbd_rbdl, TEST_PREC, TEST_PREC));
    }
  }
}


TEST_CASE_METHOD(ECM, __FILE__"_TestPSForwardKinematics", "") 
{
  ControllableBodyParams baseLinkControllableBodyParams = BODY_JOINTS_MAP[baseRigidBodyName];
  // rigidBodyPtr baseLinkHandler = baseLinkControllableBodyParams.rigidBodyHandler;
  rigidBodyPtr baseLinkHandler = BODY_JOINTS_MAP[baseRigidBodyName].rigidBodyHandler;
  usleep(1000000);

  std::vector<JointParams> controllableJoints = baseLinkControllableBodyParams.controllableJoints;


  useconds_t microsec = 250000;

  for(JointParams jointParam : controllableJoints)
  {
    std::string jointName = jointParam.jointName;
    for(int i = 0; i < 5; i++)
    {
      baseLinkHandler->set_joint_pos<std::string>(jointName, 0.0f);
      usleep(microsec);
    }
    std::cout << "jointName: " << jointName << baseLinkHandler->get_joint_pos<std::string>(jointName) << std::endl;
    std::cout << "--------------" << std::endl;
  }

  //   baseLinkHandler->set_joint_pos<std::string>(         "yawlink-pitchbacklink", 0.0f);
  //   usleep(microsec);

  //   baseLinkHandler->set_joint_pos<std::string>("pitchendlink-maininsertionlink", 0.0f);
  //   usleep(microsec);
    
  //   baseLinkHandler->set_joint_pos<std::string>(    "maininsertionlink-toollink", 0.0f);
  //   usleep(microsec);

  //   std::cout << "------------------" << std::endl;
  // }




  // usleep(550000);
  // for(BODY_JOINTS_MAP_itr  = BODY_JOINTS_MAP.begin(); BODY_JOINTS_MAP_itr != BODY_JOINTS_MAP.end(); 
  //     BODY_JOINTS_MAP_itr++)
  // {
  //   std::string rigidBodyName = BODY_JOINTS_MAP_itr->first;
  //   printf("rigidBodyName: %s\n", rigidBodyName.c_str());
  //   ControllableBodyParams controllableBodyParams = BODY_JOINTS_MAP_itr->second;
    
  //   rigidBodyPtr rigidBodyHandler = controllableBodyParams.rigidBodyHandler;
  //   std::vector<JointParams> childrenJoints = controllableBodyParams.childrenJoints;
  //   for(JointParams jointParam : childrenJoints)
  //   {
  //     std::string jointName = jointParam.jointName;
  //     float qDesired = rigidBodyHandler->get_joint_pos<std::string>(jointName);
      
  //     const tf::Quaternion quat_w_n_tf_ambf = rigidBodyHandler->get_rot();
  //     const tf::Vector3 P_w_n_tf_ambf = rigidBodyHandler->get_pos();
    
  //     const Eigen::Quaterniond quat_w_n_ambf = 
  //       EigenUtilities::tfToEigenQuaternion(quat_w_n_tf_ambf);
  //     const Eigen::Matrix3d R_w_n_ambf = quat_w_n_ambf.toRotationMatrix();
  //     Eigen::VectorXd P_w_n_ambf = EigenUtilities::tfToEigenVector(P_w_n_tf_ambf);

  //     const Eigen::Matrix4d T_w_n_ambf = 
  //       EigenUtilities::get_frame<Eigen::Matrix3d, 
  //       Eigen::Vector3d, Eigen::Matrix4d>(R_w_n_ambf, P_w_n_ambf);

  //     // printf("rigidBodyName: %s, qDesired: %f\n", jointName.c_str(), qDesired);
  //     // std::cout << "T_w_n_ambf" << std::endl << T_w_n_ambf << std::endl;
  //   }
  //   // std::cout << "------------------------" << std::endl;
  // }
  // usleep(250000);

  // usleep(1000000);

  // ForwardDynamics(*rbdlModel, Q, QDot, Tau, QDDot);
  // Q.setZero();

  // std::map< std::string, unsigned int > mBodyNameMap = rbdlModel->mBodyNameMap;
  // std::map<std::string, unsigned int>::iterator mBodyNameMapItr;

  // // Check to be a valid joint
  // if(baseHandler->is_joint_idx_valid(activationJointID))
  // {
  //   for(int i = 0; i < 5; i++)
  //   {
  //     baseHandler->set_joint_pos<std::string>(activationJointName, 
  //       activationJointQdesired);
  //     usleep(250000);

  //     for(mBodyNameMapItr = mBodyNameMap.begin(); 
  //         mBodyNameMapItr != mBodyNameMap.end(); 
  //         mBodyNameMapItr++)
  //     {
  //       std::string jointNameRBDL = mBodyNameMapItr->first;
  //       unsigned int jointIDRBDL = mBodyNameMapItr->second;

  //       // printf("jointNameRBDL: %s, jointIDRBDL: %d\n", jointNameRBDL.c_str(), jointIDRBDL);
  //       // Skip ROOT body and fixed joints
  //       if(jointNameRBDL == "ROOT" || jointIDRBDL > Q.size()) continue;

  //       RBDL_AMBF_JOINT_MAP_itr = RBDL_AMBF_JOINT_MAP.find(jointNameRBDL);
  //       // Not a vaild AMBF joint
  //       if(RBDL_AMBF_JOINT_MAP_itr == RBDL_AMBF_JOINT_MAP.end() && 
  //       baseHandler->get_joint_idx_from_name(jointNameRBDL) == -1) continue;

  //       float jointQdesired = baseHandler->get_joint_pos<std::string>(jointNameRBDL);        
  //       int jointIndexRBDL = jointIDRBDL - 1;
  //       Q[jointIndexRBDL] = jointQdesired;
  //     }
  //   }
  // }

  // for(mBodyNameMapItr = mBodyNameMap.begin(); 
  //     mBodyNameMapItr != mBodyNameMap.end(); 
  //     mBodyNameMapItr++)
  // {
  //   std::string jointNameRBDL = mBodyNameMapItr->first;
  //   unsigned int jointIDRBDL  = mBodyNameMapItr->second;
    
  //   // Skip ROOT body and fixed joints
  //   if(jointNameRBDL == "ROOT" || jointIDRBDL > Q.size()) continue;

  //   // Not a vaild AMBF joint
  //   RBDL_AMBF_JOINT_MAP_itr = RBDL_AMBF_JOINT_MAP.find(jointNameRBDL);
  //   if(RBDL_AMBF_JOINT_MAP_itr == RBDL_AMBF_JOINT_MAP.end() && 
  //   baseHandler->get_joint_idx_from_name(jointNameRBDL) == -1) continue;

  //   ActivationJoints bodyAMBF = RBDL_AMBF_JOINT_MAP.at(jointNameRBDL);
  //   rigidBodyPtr rigidbodyAMBF = bodyAMBF.rigidBodyHandler;

  //   const tf::Quaternion quat_w_n_tf_ambf = rigidbodyAMBF->get_rot();
  //   const tf::Vector3 P_w_n_tf_ambf = rigidbodyAMBF->get_pos();
    
  //   const Eigen::Quaterniond quat_w_n_ambf = 
  //     EigenUtilities::tfToEigenQuaternion(quat_w_n_tf_ambf);
  //   const Eigen::Matrix3d R_w_n_ambf = quat_w_n_ambf.toRotationMatrix();
  //   Eigen::VectorXd P_w_n_ambf = EigenUtilities::tfToEigenVector(P_w_n_tf_ambf);

  //   const Eigen::Matrix4d T_w_n_ambf = 
  //     EigenUtilities::get_frame<Eigen::Matrix3d, 
  //     Eigen::Vector3d, Eigen::Matrix4d>(R_w_n_ambf, P_w_n_ambf);
    
  //   const RigidBodyDynamics::Math::Vector3d P_0_n_rbdl = 
  //     CalcBodyToBaseCoordinates(*rbdlModel, Q, jointIDRBDL, 
  //       RigidBodyDynamics::Math::Vector3d(0., 0., 0.),true);

  //   RigidBodyDynamics::Math::Vector4d P_w_n_rbdl_4d;
  //   P_w_n_rbdl_4d.setOnes();
  //   P_w_n_rbdl_4d(0) = P_0_n_rbdl(0);
  //   P_w_n_rbdl_4d(1) = P_0_n_rbdl(1);
  //   P_w_n_rbdl_4d(2) = P_0_n_rbdl(2);

  //   P_w_n_rbdl_4d = T_0_w * (P_w_n_rbdl_4d);
  //   RigidBodyDynamics::Math::Vector3d P_w_n_rbdl;
  //   P_w_n_rbdl = P_w_n_rbdl_4d.block<3,1>(0,0);

  //   CHECK_THAT(P_w_n_rbdl, AllCloseMatrix(P_w_n_ambf, TEST_PREC, TEST_PREC));
  // }  
  
}
*/