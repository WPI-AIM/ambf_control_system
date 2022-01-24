#include "rbdl_model_tests/ParallelStructure.h"

/*
TEST_CASE_METHOD(ParallelStructure, __FILE__"_TestPSNeutralPosition", "") 
{
  ForwardDynamics(*rbdlPSModel, Q, QDot, Tau, QDDot);
  std::vector<std::string> joints = baseHandler->get_joint_names();
  std::vector<std::string> baseChildren = baseHandler->get_children_names();

  CHECK (baseChildren.size() == Q.size());

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
    unsigned int rbdlBodyId = rbdlPSModel->GetBodyId(body.c_str());

    // AMBF body does not exist in RBDL model
    if(rbdlBodyId != std::numeric_limits<unsigned int>::max())
    // if(rbdlBodyId < rbdlPSModel->mBodies.size())
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
                                        CalcBodyWorldOrientation(*rbdlPSModel, Q, rbdlBodyId, true);
      const RigidBodyDynamics::Math::Vector3d P_n_0_rbdl = 
                                        CalcBodyToBaseCoordinates(*rbdlPSModel, Q, rbdlBodyId, 
                                            RigidBodyDynamics::Math::Vector3d(0., 0., 0.), true);

      const RigidBodyDynamics::Math::Vector3d P_n_w_rbdl = T_0_w.block<3, 3>(0, 0) * P_n_0_rbdl;

      CHECK_THAT(R_n_w_ambf, AllCloseMatrix(R_n_w_rbdl, TEST_PREC, TEST_PREC));
      CHECK_THAT(P_n_w_ambf, AllCloseMatrix(P_n_w_rbdl, TEST_PREC, TEST_PREC));
    }
  }
}

*/

TEST_CASE_METHOD(ParallelStructure, __FILE__"_TestPSPIbyFourPositionV2", "") 
{
  // mBody in RBDL has AMBF Joints
  std::map<std::string, unsigned int > mBodyNameMap = rbdlPSModel->mBodyNameMap;
  std::map<std::string, unsigned int>::iterator mBodyNameMapItr;
  for(mBodyNameMapItr = mBodyNameMap.begin(); mBodyNameMapItr != mBodyNameMap.end(); mBodyNameMapItr++)
  {
    std::string bodyName = mBodyNameMapItr->first;
    unsigned int bodyId = mBodyNameMapItr->second;
    std::string parentName = rbdlPSModel->GetBodyName(rbdlPSModel->GetParentBodyId(bodyId));
    std::cout << parentName << ", " << bodyName << ", " << bodyId << std::endl;
  }

  std::cout << "--------------------------" << std::endl;

  ForwardDynamics(*rbdlPSModel, Q, QDot, Tau, QDDot);
  std::vector<std::string> joints = baseHandler->get_joint_names();

  // for(std::string joint : joints)
  // {
  //   std::cout << joint << std::endl;
  // }

  //std::vector<std::string> baseChildren = baseHandler->get_children_names();

  CHECK (joints.size() == Q.size());

  const std::string activationJointName = "l3-l4";
  const int activationJointID = baseHandler->get_joint_idx_from_name(activationJointName.c_str());
  Q.setZero();

  std::unordered_map<std::string, float> q_desired;
  std::unordered_map<std::string, float>::iterator q_desiredItr;

  q_desired[activationJointName] = M_PI_4;

  // const int activationJointID = baseHandler->get_joint_idx_from_name(activationJointName.c_str());
            
  // Check to be a valid joint
  if(baseHandler->is_joint_idx_valid(activationJointID))
  {

    for(int i = 0; i < 20; i++)
    {

      std::cout << "activationJointID: "                << activationJointID << std::endl;
      std::cout << "activationJointName: "              << activationJointName 
                << ", q_desired[activationJointName]: " << q_desired[activationJointName]
                << std::endl;

      baseHandler->set_joint_pos<std::string>(activationJointName, q_desired[activationJointName]);
      usleep(250000);

      std::vector<float> joint_poses = baseHandler->get_all_joint_pos();
      
      for(int i = 0; i < joints.size(); i++)
      {
        std::string joint = joints.at(i);
        // Desired Angle of Activation joint keeps drifting to zero over iterations. So, do not update
        // desired pose to the map.
        if(joint.compare(activationJointName) != 0)
        {
          unsigned int rbdlBodyId = rbdlPSModel->GetBodyId(joint.c_str());
          float joint_angle = baseHandler->get_joint_pos<std::string>(joint);
          std::cout << "Updating pose for joint: " << joint << std::endl;
          q_desired[joint] = joint_angle;
        }
      }
    }

    for(q_desiredItr = q_desired.begin(); q_desiredItr!=q_desired.end(); ++q_desiredItr)
    {
      std::string joint = q_desiredItr->first;
      float joint_angle = q_desiredItr->second;

      unsigned int rbdlBodyId = rbdlPSModel->GetBodyId(joint.c_str());
      if(rbdlBodyId != std::numeric_limits<unsigned int>::max() && rbdlBodyId < rbdlPSModel->mBodies.size())
      {
        rbdlBodyId--;
        Q[rbdlBodyId] = joint_angle;
        std::cout << "joint: "         << joint 
                  << ", rbdlBodyId: "  << rbdlBodyId 
                  << ", joint_angle: " << joint_angle << std::endl;
      }
    }

    std::cout << "Q: " << std::endl << Q << std::endl;


    for(ambfRBDLBodyMapItr = ambfRBDLBodyMap.begin(); ambfRBDLBodyMapItr != ambfRBDLBodyMap.end(); 
                                                                              ambfRBDLBodyMapItr++)
      {
      // RBDL has only joint name no body name. RBDL uses joint names to refer rigid body poses.
      // However, AMBF has specific names to Rigid body as well as Joints. They use body names
      // to refer poses.
      const std::string ambfBodyName = ambfRBDLBodyMapItr->first;
      const std::string rbdlJointName = ambfRBDLBodyMapItr->second;


    // for(std::string body : baseChildren)
    // {
    //   unsigned int rbdlBodyId = rbdlPSModel->GetBodyId(body.c_str());
      unsigned int rbdlBodyId = rbdlPSModel->GetBodyId(rbdlJointName.c_str());

      // AMBF body does not exist in RBDL model
      if(rbdlBodyId != std::numeric_limits<unsigned int>::max())
      // if(rbdlBodyId < rbdlPSModel->mBodies.size())
      {
        std::cout << "Executing Test case for joint: "<< rbdlJointName << ", " << rbdlBodyId << std::endl;

        rigidBodyPtr rigidBodyHandler = clientPtr->getRigidBody(ambfBodyName.c_str(), true);
        
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
                                          CalcBodyWorldOrientation(*rbdlPSModel, Q, rbdlBodyId, true);
        const RigidBodyDynamics::Math::Vector3d P_n_0_rbdl = 
                                          CalcBodyToBaseCoordinates(*rbdlPSModel, Q, rbdlBodyId, 
                                              RigidBodyDynamics::Math::Vector3d(0., 0., 0.), true);

        const RigidBodyDynamics::Math::Vector3d P_n_w_rbdl = T_0_w.block<3, 3>(0, 0) * P_n_0_rbdl;

        // CHECK_THAT(R_n_w_ambf, AllCloseMatrix(R_n_w_rbdl, TEST_PREC, TEST_PREC));
        CHECK_THAT(P_n_w_ambf, AllCloseMatrix(P_n_w_rbdl, TEST_PREC, TEST_PREC));
      }
    }
  }
}


/*
TEST_CASE_METHOD(ParallelStructure, __FILE__"_TestPSPIbyFourPosition", "") 
{
  //mBody in RBDL has AMBF Joints
  std::map<std::string, unsigned int > mBodyNameMap = rbdlPSModel->mBodyNameMap;
  std::map<std::string, unsigned int>::iterator mBodyNameMapItr;
  for(mBodyNameMapItr = mBodyNameMap.begin(); mBodyNameMapItr != mBodyNameMap.end(); mBodyNameMapItr++)
  {
    std::string bodyName = mBodyNameMapItr->first;
    unsigned int bodyId = mBodyNameMapItr->second;
    std::string parentName = rbdlPSModel->GetBodyName(rbdlPSModel->GetParentBodyId(bodyId));
    std::cout << parentName << ", " << bodyName << ", " << bodyId << std::endl;
    
  }

  ForwardDynamics(*rbdlPSModel, Q, QDot, Tau, QDDot);
  std::vector<std::string> joints = baseHandler->get_joint_names();
  std::vector<std::string> baseChildren = baseHandler->get_children_names();

  CHECK (baseChildren.size() == Q.size());

  std::string activation_joint_name = "l3-l4";
  Q.setZero();

  std::unordered_map<std::string, float> q_desired;
  std::unordered_map<std::string, float>::iterator q_desiredItr;

  q_desired[activation_joint_name] = M_PI_4;

  int activation_joint_id = baseHandler->get_joint_idx_from_name(activation_joint_name.c_str());
  std::cout << "activation_joint_id: " << activation_joint_id << std::endl;

  if(baseHandler->is_joint_idx_valid(activation_joint_id))
  {
    for(int i = 0; i < 20; i++)
    {
      baseHandler->set_joint_pos<std::string>(activation_joint_name, q_desired[activation_joint_name]);
      usleep(250000);

      for(int i = 0; i < joints.size(); i++)
      {
        std::string joint = joints.at(i);
        std::string body = baseChildren.at(i);

        unsigned int rbdlBodyId = rbdlPSModel->GetBodyId(joint.c_str());
        float joint_angle = baseHandler->get_joint_pos<std::string>(joint);

        q_desired[body] = joint_angle;
      }
    }

    for(q_desiredItr = q_desired.begin(); q_desiredItr!=q_desired.end(); ++q_desiredItr)
    {
      std::string body = q_desiredItr->first;
      float joint_angle = q_desiredItr->second;

      std::cout << "body: " << body << ", joint_angle: " << joint_angle << std::endl;

      unsigned int rbdlBodyId = rbdlPSModel->GetBodyId(body.c_str());
      if(rbdlBodyId != std::numeric_limits<unsigned int>::max() && rbdlBodyId < rbdlPSModel->mBodies.size())
      {
        rbdlBodyId--;
        Q[rbdlBodyId] = joint_angle;
      }
    }

    // std::cout << "Q: " << std::endl << Q << std::endl;

    for(std::string body : baseChildren)
    {
      unsigned int rbdlBodyId = rbdlPSModel->GetBodyId(body.c_str());

      // AMBF body does not exist in RBDL model
      if(rbdlBodyId != std::numeric_limits<unsigned int>::max())
      // if(rbdlBodyId < rbdlPSModel->mBodies.size())
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
                                          CalcBodyWorldOrientation(*rbdlPSModel, Q, rbdlBodyId, true);
        const RigidBodyDynamics::Math::Vector3d P_n_0_rbdl = 
                                          CalcBodyToBaseCoordinates(*rbdlPSModel, Q, rbdlBodyId, 
                                              RigidBodyDynamics::Math::Vector3d(0., 0., 0.), true);

        const RigidBodyDynamics::Math::Vector3d P_n_w_rbdl = T_0_w.block<3, 3>(0, 0) * P_n_0_rbdl;

        CHECK_THAT(R_n_w_ambf, AllCloseMatrix(R_n_w_rbdl, TEST_PREC, TEST_PREC));
        CHECK_THAT(P_n_w_ambf, AllCloseMatrix(P_n_w_rbdl, TEST_PREC, TEST_PREC));
      }
    }
  }
}


TEST_CASE_METHOD(ParallelStructure, __FILE__"_TestPSRandomPosition", "") 
{
  //mBody in RBDL has AMBF Joints
  std::map<std::string, unsigned int > mBodyNameMap = rbdlPSModel->mBodyNameMap;
  std::map<std::string, unsigned int>::iterator mBodyNameMapItr;
  for(mBodyNameMapItr = mBodyNameMap.begin(); mBodyNameMapItr != mBodyNameMap.end(); mBodyNameMapItr++)
  {
    std::string bodyName = mBodyNameMapItr->first;
    unsigned int bodyId = mBodyNameMapItr->second;
    std::string parentName = rbdlPSModel->GetBodyName(rbdlPSModel->GetParentBodyId(bodyId));
    std::cout << parentName << ", " << bodyName << ", " << bodyId << std::endl;
    
  }

  ForwardDynamics(*rbdlPSModel, Q, QDot, Tau, QDDot);
  std::vector<std::string> joints = baseHandler->get_joint_names();
  std::vector<std::string> baseChildren = baseHandler->get_children_names();

  CHECK (baseChildren.size() == Q.size());

  std::string activation_joint_name = "l3-l4";
  Q.setZero();

  std::unordered_map<std::string, float> q_desired;
  std::unordered_map<std::string, float>::iterator q_desiredItr;

  int activation_joint_id = baseHandler->get_joint_idx_from_name(activation_joint_name.c_str());
  std::cout << "activation_joint_id: " << activation_joint_id << std::endl;

  if(baseHandler->is_joint_idx_valid(activation_joint_id))
  {
    for(int nTest = 1; nTest <= 5; nTest++)
    {
      q_desired.clear();

      std::vector<float> joint_limit = PS_JOINT_LIMITS[activation_joint_name];
      float low = joint_limit.at(0);
      float high = joint_limit.at(1);

      q_desired[activation_joint_name] = EigenUtilities::get_random_between_range(low, high);
      usleep(1000000);

      for(int i = 0; i < 20; i++)
      {
        baseHandler->set_joint_pos<std::string>(activation_joint_name, q_desired[activation_joint_name]);
        usleep(250000);

        for(int i = 0; i < joints.size(); i++)
        {
          std::string joint = joints.at(i);
          std::string body = baseChildren.at(i);

          unsigned int rbdlBodyId = rbdlPSModel->GetBodyId(joint.c_str());
          float joint_angle = baseHandler->get_joint_pos<std::string>(joint);

          q_desired[body] = joint_angle;

        }
      }

      for(int i = 0; i < joints.size(); i++)
      {
        std::string joint = joints.at(i);
        std::string body = baseChildren.at(i);

        float joint_angle = q_desired[body];
        std::cout << "joint: " << joint << ", desired joint_angle: " << joint_angle << std::endl;
      }

      for(q_desiredItr = q_desired.begin(); q_desiredItr!=q_desired.end(); ++q_desiredItr)
      {
        std::string body = q_desiredItr->first;
        float joint_angle = q_desiredItr->second;

        unsigned int rbdlBodyId = rbdlPSModel->GetBodyId(body.c_str());
        if(rbdlBodyId != std::numeric_limits<unsigned int>::max() && rbdlBodyId < rbdlPSModel->mBodies.size())
        {
          rbdlBodyId--;
          Q[rbdlBodyId] = joint_angle;
        }
      }

      for(std::string body : baseChildren)
      {
        unsigned int rbdlBodyId = rbdlPSModel->GetBodyId(body.c_str());

        // AMBF body does not exist in RBDL model
        if(rbdlBodyId != std::numeric_limits<unsigned int>::max())
        // if(rbdlBodyId < rbdlPSModel->mBodies.size())
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
                                            CalcBodyWorldOrientation(*rbdlPSModel, Q, rbdlBodyId, true);
          const RigidBodyDynamics::Math::Vector3d P_n_0_rbdl = 
                                            CalcBodyToBaseCoordinates(*rbdlPSModel, Q, rbdlBodyId, 
                                                RigidBodyDynamics::Math::Vector3d(0., 0., 0.), true);

          const RigidBodyDynamics::Math::Vector3d P_n_w_rbdl = T_0_w.block<3, 3>(0, 0) * P_n_0_rbdl;

          CHECK_THAT(R_n_w_ambf, AllCloseMatrix(R_n_w_rbdl, TEST_PREC, TEST_PREC));
          CHECK_THAT(P_n_w_ambf, AllCloseMatrix(P_n_w_rbdl, TEST_PREC, TEST_PREC));
        }
      }

      std::cout << "------- end of test: " << nTest << " ---------" << std::endl;
    }
  }
}
*/