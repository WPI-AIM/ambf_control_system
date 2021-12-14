#include "rbdl_model_tests/ECM.h"

/*
TEST_CASE_METHOD(ECM, __FILE__"_TestECMPositionNeutral", "") 
{
  //mBody in RBDL has AMBF Joints
  std::map< std::string, unsigned int > mBodyNameMap = rbdlECMModel->mBodyNameMap;
  std::map<std::string, unsigned int>::iterator mBodyNameMapItr;
  for(mBodyNameMapItr = mBodyNameMap.begin(); mBodyNameMapItr != mBodyNameMap.end(); mBodyNameMapItr++)
  {
    std::string bodyName = mBodyNameMapItr->first;
    unsigned int bodyId = mBodyNameMapItr->second;
    std::string parentName = rbdlECMModel->GetBodyName(rbdlECMModel->GetParentBodyId(bodyId));
    std::cout << parentName << ", " << bodyName << ", " << bodyId << std::endl;
    
  }

  ForwardDynamics(*rbdlECMModel, Q, QDot, Tau, QDDot);
  std::vector<std::string> joints = baseHandler->get_joint_names();
  std::vector<std::string> baseChildren = baseHandler->get_children_names();

  //CHECK (baseChildren.size() == Q.size());

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
    unsigned int rbdlBodyId = rbdlECMModel->GetBodyId(body.c_str());

    if(rbdlBodyId < rbdlECMModel->mBodies.size())
    {
      std::cout << "Executing Test case for body: "<< body << ", " << rbdlBodyId << std::endl;

      rigidBodyPtr rigidBodyHandler = clientPtr->getRigidBody(body, true);
      // n - respective body frame
      const tf::Quaternion quat_n_w_tf = rigidBodyHandler->get_rot();
      const tf::Vector3 P_n_w_tf = rigidBodyHandler->get_pos();

      RigidBodyDynamics::Math::Quaternion quat_n_w_ambf;
      quat_n_w_ambf(0) = quat_n_w_tf[0];
      quat_n_w_ambf(1) = quat_n_w_tf[1];
      quat_n_w_ambf(2) = quat_n_w_tf[2];
      quat_n_w_ambf(3) = quat_n_w_tf[3];

      const RigidBodyDynamics::Math::Matrix3d R_n_w_ambf = quat_n_w_ambf.toMatrix();

      RigidBodyDynamics::Math::Vector3d P_n_w_ambf;
      P_n_w_ambf.setZero();
      
      P_n_w_ambf(0) = P_n_w_tf[0];
      P_n_w_ambf(1) = P_n_w_tf[1];
      P_n_w_ambf(2) = P_n_w_tf[2];

      const RigidBodyDynamics::Math::Matrix3d R_n_w_rbdl =
                                        CalcBodyWorldOrientation(*rbdlECMModel, Q, rbdlBodyId, true);
      const RigidBodyDynamics::Math::Vector3d P_n_0_rbdl = 
                                        CalcBodyToBaseCoordinates(*rbdlECMModel, Q, rbdlBodyId, 
                                            RigidBodyDynamics::Math::Vector3d(0., 0., 0.), true);

      RigidBodyDynamics::Math::Vector4d P_n_0_rbdl_4d;
      P_n_0_rbdl_4d.setOnes();
      P_n_0_rbdl_4d(0) = P_n_0_rbdl(0);
      P_n_0_rbdl_4d(1) = P_n_0_rbdl(1);
      P_n_0_rbdl_4d(2) = P_n_0_rbdl(2);

      RigidBodyDynamics::Math::Vector4d P_n_w_rbdl_4d;
      P_n_w_rbdl_4d = T_0_w * (P_n_0_rbdl_4d);

      RigidBodyDynamics::Math::Vector3d P_n_w_rbdl;
      P_n_w_rbdl = P_n_w_rbdl_4d.block<3,1>(0,0);

      std::cout << std::endl << "diff: " << std::endl << R_n_w_ambf - R_n_w_rbdl << std::endl;

      CHECK_THAT(R_n_w_ambf, AllCloseMatrix(R_n_w_rbdl, TEST_PREC, TEST_PREC));
      CHECK_THAT(P_n_w_ambf, AllCloseMatrix(P_n_w_rbdl, TEST_PREC, TEST_PREC));
    }
  }
}
*/

TEST_CASE_METHOD(ECM, __FILE__"_TestECMPositionNeutral", "") 
{
  //mBody in RBDL has AMBF Joints
  std::map< std::string, unsigned int > mBodyNameMap = rbdlECMModel->mBodyNameMap;
  std::map<std::string, unsigned int>::iterator mBodyNameMapItr;
  for(mBodyNameMapItr = mBodyNameMap.begin(); mBodyNameMapItr != mBodyNameMap.end(); mBodyNameMapItr++)
  {
    std::string bodyName = mBodyNameMapItr->first;
    unsigned int bodyId = mBodyNameMapItr->second;
    std::string parentName = rbdlECMModel->GetBodyName(rbdlECMModel->GetParentBodyId(bodyId));
    std::cout << parentName << ", " << bodyName << ", " << bodyId << std::endl;
  }

  //std::vector<std::string> joints = baseHandler->get_joint_names();
  //std::vector<std::string> baseChildren = baseHandler->get_children_names();

  for(ActivationJoints cJoint : ecmControllableJoints)
  {
    int activation_joint_id = baseHandler->get_joint_idx_from_name(cJoint.name.c_str());
    if(!baseHandler->is_joint_idx_valid(activation_joint_id))
    {
      std::cout << "Controllable Joint " << cJoint.name 
                << " is invalid. Exiting test case execution." << std::endl;
      return; 
    }
  }

  std::unordered_map<std::string, float> q_desired;
  std::unordered_map<std::string, float>::iterator q_desiredItr;
  for(int nTest = 1; nTest <= 1; nTest++)
  {
    for(ActivationJoints cJoint : ecmControllableJoints)
    {
      //q_desired[cJoint.name] = EigenUtilities::get_random_between_range(cJoint.joint_lower_limit, cJoint.joint_upper_limit);
      q_desired[cJoint.name] = M_PI_4;
      usleep(1000000);
    }

    for(int i = 0; i < 1; i++)
    {
      for(q_desiredItr = q_desired.begin(); q_desiredItr != q_desired.end(); q_desiredItr++)
      {
        std::string cJointName = q_desiredItr->first;
        float jointAngle = q_desiredItr->second;
        baseHandler->set_joint_pos<std::string>(cJointName, jointAngle);
      }

      for(bodyJointsMapItr = bodyJointsMap.begin(); bodyJointsMapItr != bodyJointsMap.end(); bodyJointsMapItr++)
      {
        std::string parentBody = bodyJointsMapItr->first;
        std::vector<std::string> joints = bodyJointsMapItr->second;
      
        std::cout << "parentBody: " << parentBody << std::endl; 
        rigidBodyPtr parentHandler = clientPtr->getRigidBody(parentBody, true);
        usleep(1000000);

        for(std::string joint : joints)
        {
          std::cout << "joint: " << joint << std::endl;
          float joint_angle = parentHandler->get_joint_pos<std::string>(joint);
          q_desired[joint] = joint_angle;
        }
        std::cout << "-----------------------" << std::endl;
      }
      usleep(250000);
    }

    for(q_desiredItr = q_desired.begin(); q_desiredItr != q_desired.end(); q_desiredItr++)
    {
      std::string jointName = q_desiredItr->first;
      float jointAngle = q_desiredItr->second;

      std::cout << "jointName: " << jointName << ", jointAngle: " << jointAngle << std::endl;
    }
  }
  // ForwardDynamics(*rbdlECMModel, Q, QDot, Tau, QDDot);
  //CHECK (baseChildren.size() == Q.size());
  // Q.setZero();
  
  // for(std::string body : baseChildren)
  // {
  //   unsigned int rbdlBodyId = rbdlECMModel->GetBodyId(body.c_str());

  //   if(rbdlBodyId < rbdlECMModel->mBodies.size())
  //   {
  //     std::cout << "Executing Test case for body: "<< body << ", " << rbdlBodyId << std::endl;

  //     rigidBodyPtr rigidBodyHandler = clientPtr->getRigidBody(body, true);
  //     // n - respective body frame
  //     const tf::Quaternion quat_n_w_tf = rigidBodyHandler->get_rot();
  //     const tf::Vector3 P_n_w_tf = rigidBodyHandler->get_pos();

  //     RigidBodyDynamics::Math::Quaternion quat_n_w_ambf;
  //     quat_n_w_ambf(0) = quat_n_w_tf[0];
  //     quat_n_w_ambf(1) = quat_n_w_tf[1];
  //     quat_n_w_ambf(2) = quat_n_w_tf[2];
  //     quat_n_w_ambf(3) = quat_n_w_tf[3];

  //     const RigidBodyDynamics::Math::Matrix3d R_n_w_ambf = quat_n_w_ambf.toMatrix();

  //     RigidBodyDynamics::Math::Vector3d P_n_w_ambf;
  //     P_n_w_ambf.setZero();
      
  //     P_n_w_ambf(0) = P_n_w_tf[0];
  //     P_n_w_ambf(1) = P_n_w_tf[1];
  //     P_n_w_ambf(2) = P_n_w_tf[2];

  //     const RigidBodyDynamics::Math::Matrix3d R_n_w_rbdl =
  //                                       CalcBodyWorldOrientation(*rbdlECMModel, Q, rbdlBodyId, true);
  //     const RigidBodyDynamics::Math::Vector3d P_n_0_rbdl = 
  //                                       CalcBodyToBaseCoordinates(*rbdlECMModel, Q, rbdlBodyId, 
  //                                           RigidBodyDynamics::Math::Vector3d(0., 0., 0.), true);

  //     RigidBodyDynamics::Math::Vector4d P_n_0_rbdl_4d;
  //     P_n_0_rbdl_4d.setOnes();
  //     P_n_0_rbdl_4d(0) = P_n_0_rbdl(0);
  //     P_n_0_rbdl_4d(1) = P_n_0_rbdl(1);
  //     P_n_0_rbdl_4d(2) = P_n_0_rbdl(2);

  //     RigidBodyDynamics::Math::Vector4d P_n_w_rbdl_4d;
  //     P_n_w_rbdl_4d = T_0_w * (P_n_0_rbdl_4d);

  //     RigidBodyDynamics::Math::Vector3d P_n_w_rbdl;
  //     P_n_w_rbdl = P_n_w_rbdl_4d.block<3,1>(0,0);

  //     std::cout << std::endl << "diff: " << std::endl << R_n_w_ambf - R_n_w_rbdl << std::endl;

  //     CHECK_THAT(R_n_w_ambf, AllCloseMatrix(R_n_w_rbdl, TEST_PREC, TEST_PREC));
  //     CHECK_THAT(P_n_w_ambf, AllCloseMatrix(P_n_w_rbdl, TEST_PREC, TEST_PREC));
  //   }
  // }

}
