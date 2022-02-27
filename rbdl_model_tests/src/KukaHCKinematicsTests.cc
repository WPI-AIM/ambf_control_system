#include "rbdl_model_tests/KUKA.h"

TEST_CASE_METHOD(KUKA, __FILE__"_TestKUKAPositionNeutral", "") 
{
  // We call ForwardDynamics() as it updates the spatial transformation
  // matrices
  ForwardDynamics(*rbdlModel, Q, QDot, Tau, QDDot);

  Q.setZero();
  Q[0] = M_PI_4;
  Q[1] = M_PI_4;
  Q[2] = M_PI_4;
  Q[3] = M_PI_4;
  Q[4] = M_PI_4;
  Q[5] = M_PI_4;
  Q[6] = M_PI_4;

  std::vector<std::string> baseChildren = baseHandler->get_children_names();

  std::map< std::string, unsigned int > mBodyNameMap = rbdlModel->mBodyNameMap;
  std::map<std::string, unsigned int>::iterator mBodyNameMapItr;
  // Itrerate through the body names. Skipp index 0 which is ROOT
  for(mBodyNameMapItr = ++(mBodyNameMap.begin()); mBodyNameMapItr != mBodyNameMap.end(); 
                                                                            mBodyNameMapItr++)
  {
    std::string jointNameRBDL = mBodyNameMapItr->first;
    unsigned int jointIDRBDL = mBodyNameMapItr->second;
    int jointIndexRBDL = jointIDRBDL - 1;

    // std::cout << jointName_rbdl << ", " << jointID_rbdl << std::endl;
    if(baseHandler->get_joint_pos<std::string>(jointNameRBDL) != FLT_MIN)
    {
      std::cout << "Executing Test case for body: "<< body << ", " << rbdlBodyId << std::endl;

      rigidBodyPtr rigidBodyHandler = clientPtr->getRigidBody(body, true);
      // TODO : it should be P_w_n not P_n_w. Change nameing convention
      // n - respective body frame
      const tf::Quaternion quat_n_w_tf = rigidBodyHandler->get_rot();
      const tf::Vector3 P_n_w_tf = rigidBodyHandler->get_pos();
      
      RigidBodyDynamics::Math::Quaternion quat_n_w_ambf;
      quat_n_w_ambf(0) = quat_n_w_tf[0];
      quat_n_w_ambf(1) = quat_n_w_tf[1];
      quat_n_w_ambf(2) = quat_n_w_tf[2];
      quat_n_w_ambf(3) = quat_n_w_tf[3];

      const RigidBodyDynamics::Math::Matrix3d R_n_w_ambf = quat_n_w_ambf.toMatrix();
      
      Eigen::Vector4d P_n_0_rbd;
      P_n_0_rbd[0] = P_n_w_tf[0];
      P_n_0_rbd[1] = P_n_w_tf[1];
      P_n_0_rbd[2] = P_n_w_tf[2];
      P_n_0_rbd[3] = 1.0;
      P_n_0_rbd = T_0_w.inverse() * P_n_0_rbd;

      RigidBodyDynamics::Math::Vector3d P_n_w_ambf;
      P_n_w_ambf.setZero();
      
      P_n_w_ambf(0) = P_n_w_tf[0];
      P_n_w_ambf(1) = P_n_w_tf[1];
      P_n_w_ambf(2) = P_n_w_tf[2];

      RigidBodyDynamics::Math::Vector3d P_n_w_rbd_ambf;
      P_n_w_rbd_ambf.setZero();
      
      P_n_w_rbd_ambf(0) = P_n_w_tf[0];
      P_n_w_rbd_ambf(1) = P_n_w_tf[1];
      P_n_w_rbd_ambf(2) = P_n_w_tf[2];

      const RigidBodyDynamics::Math::Matrix3d R_n_w_rbdl =
              CalcBodyWorldOrientation(*rbdlModel, Q, rbdlBodyId, true);
              
      const RigidBodyDynamics::Math::Vector3d P_n_0_rbd_rbdl = 
              CalcBodyToBaseCoordinates(*rbdlModel, Q, rbdlBodyId, 
                                    RigidBodyDynamics::Math::Vector3d(0., 0., 0.),true);

      RigidBodyDynamics::Math::Vector4d P_n_w_rbd_rbdl_4d;
      P_n_w_rbd_rbdl_4d.setOnes();
      P_n_w_rbd_rbdl_4d(0) = P_n_0_rbd_rbdl(0);
      P_n_w_rbd_rbdl_4d(1) = P_n_0_rbd_rbdl(1);
      P_n_w_rbd_rbdl_4d(2) = P_n_0_rbd_rbdl(2);

      P_n_w_rbd_rbdl_4d = T_0_w * (P_n_w_rbd_rbdl_4d);

      RigidBodyDynamics::Math::Vector3d P_n_w_rbd_rbdl;
      P_n_w_rbd_rbdl = P_n_w_rbd_rbdl_4d.block<3,1>(0,0);
      
      CHECK_THAT(R_n_w_ambf, AllCloseMatrix(R_n_w_rbdl, TEST_PREC, TEST_PREC));
      CHECK_THAT(P_n_w_rbd_ambf, AllCloseMatrix(P_n_w_rbd_rbdl, TEST_PREC, TEST_PREC));
    }
  }
}

TEST_CASE_METHOD(KUKA, __FILE__"_TestKUKARandomPosition", "") 
{
  // We call ForwardDynamics() as it updates the spatial transformation
  // matrices
  ForwardDynamics(*rbdlModel, Q, QDot, Tau, QDDot);

  std::vector<std::string> joints = baseHandler->get_joint_names();
  std::vector<std::string> baseChildren = baseHandler->get_children_names();

  // This will not be true for ECM
  const int nJoints = baseChildren.size();
  CHECK (nJoints == Q.size());

  float q_desired[nJoints];
  std::unordered_map<std::string, tf::Quaternion> Q_n_w_tf_ambf_map;
  std::unordered_map<std::string, tf::Vector3> P_n_w_tf_ambf_map;

  for(int i = 0; i < 5; i++)
  {
    P_n_w_tf_ambf_map.clear();

    for( std::string body : baseChildren)
    {
      ActivationJoints joinLimit = jointLimits[body];
      float low = joinLimit.joint_lower_limit;
      float high = joinLimit.joint_higher_limit;

      unsigned int rbdlBodyId = rbdlModel->GetBodyId(body.c_str());
      // body_id starts with 1 in RBDL Model. 
      // Storing corresponding Joint angles with 0 order index.
      q_desired[--rbdlBodyId] = EigenUtilities::get_random_between_range(low, high);
      usleep(1000000);
    }

    for(int j = 0; j < 10; j++)
    {
      for(std::string body : baseChildren)
      {
        baseHandler->set_joint_pos<std::string>
                                    (jointNameRBDL, Q[jointIndexRBDL]);
        usleep(250000);
      }
    }
  }

  for(mBodyNameMapItr = ++(mBodyNameMap.begin()); mBodyNameMapItr != mBodyNameMap.end(); 
                                                                            mBodyNameMapItr++)
  {
    std::string jointNameRBDL = mBodyNameMapItr->first;
    unsigned int jointIDRBDL = mBodyNameMapItr->second;
    int jointIndexRBDL = jointIDRBDL - 1;

    // rigidBodyPtr rigidBodyHandler = clientPtr->getRigidBody(body, true);
    std::vector<std::string> rigidBodyNamesAMBF = clientPtr->getRigidBodyNames();
    ActivationJoints AMBFactivationJoints = RBDL_AMBF_JOINT_MAP.at(jointNameRBDL);

    std::string bodyNameAMBF = AMBFactivationJoints.bodyNameAMBF;
    // std::cout << "bodyNameAMBF: " << bodyNameAMBF << std::endl;
    if (!std::none_of(rigidBodyNamesAMBF.begin(), rigidBodyNamesAMBF.end(), 
                            EigenUtilities::compare(bodyNameAMBF))) 
    {
      // std::cout << "Element found" << std::endl;
      rigidBodyPtr rigidBodyHandler = clientPtr->getRigidBody(bodyNameAMBF, true);
      const tf::Quaternion quat_w_n_tf = rigidBodyHandler->get_rot();
      const tf::Vector3 P_w_n_tf = rigidBodyHandler->get_pos();

      const Eigen::Quaterniond quat_w_n = EigenUtilities::tfToEigenQuaternion(quat_w_n_tf);
      const Eigen::Matrix3d R_w_n = quat_w_n.toRotationMatrix();
      Eigen::VectorXd P_w_n = EigenUtilities::tfToEigenVector(P_w_n_tf);

      const Eigen::Matrix4d T_w_n = EigenUtilities::get_frame<Eigen::Matrix3d, 
                  Eigen::Vector3d, Eigen::Matrix4d>(R_w_n, P_w_n);

      // std::cout << "T_w_n" << std::endl << T_w_n << std::endl;
      const Eigen::Matrix4d T_0_n_AMBF = T_0_w * T_w_n;
      const Eigen::Vector3d P_0_n_AMBF = T_0_n_AMBF.block<3, 1>(0, 3);

      const RigidBodyDynamics::Math::Vector3d P_0_n_RBDL = 
      CalcBodyToBaseCoordinates(*rbdlModel, Q, jointIDRBDL, 
         RigidBodyDynamics::Math::Vector3d(0., 0., 0.),true);
      CHECK_THAT(P_0_n_RBDL, AllCloseMatrix(P_0_n_AMBF, TEST_PREC, TEST_PREC));
    }
    else 
    {
        std::cout << "Element not found" << std::endl;
    }
  }
}