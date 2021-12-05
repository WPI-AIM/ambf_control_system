#include "rbdl_model_tests/ParallelStructure.h"

TEST_CASE_METHOD(ParallelStructure, __FILE__"_TestPSPositionNeutral", "") 
{
  ForwardDynamics(*rbdlPSModel, Q, QDot, Tau, QDDot);
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

      // std::cout << std::endl << "quat_n_w_rbdl.toMatrix()" << std::endl 
      //                        << quat_n_w_rbdl.toMatrix() << std::endl;

      const tf::Vector3 ryp_n_w_tf = rigidBodyHandler->get_rpy();

      const RigidBodyDynamics::Math::Matrix3d R_n_w_ambf = quat_n_w_ambf.toMatrix();
      // EigenUtilities::rotation_from_euler<Eigen::Matrix3d>(ryp_n_w_tf[0], ryp_n_w_tf[1], ryp_n_w_tf[2]);

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

      //const RigidBodyDynamics::Math::Matrix3d R_n_w_rbdl = T_0_w.block<3, 3>(0, 0) * R_n_0_rbdl;
      const RigidBodyDynamics::Math::Vector3d P_n_w_rbdl = T_0_w.block<3, 3>(0, 0) * P_n_0_rbdl;

      // const RigidBodyDynamics::Math::Matrix3d R_n_0_ambf = 
      //                           T_0_w.block<3, 3>(0, 0).transpose() * R_n_w_ambf;
      // std::cout << std::endl << "R_n_0_ambf: " << std::endl << R_n_0_ambf << std::endl;
      // std::cout << std::endl << "R_n_0_rbdl: " << std::endl << R_n_0_rbdl << std::endl;
      // std::cout << std::endl << "diff: " << std::endl << (R_n_0_ambf - R_n_0_rbdl) << std::endl;

      std::cout << std::endl << "R_n_w_ambf: " << std::endl << R_n_w_ambf << std::endl;
      std::cout << std::endl << "R_n_w_rbdl: " << std::endl << R_n_w_rbdl << std::endl;
      std::cout << std::endl << "diff: " << std::endl << (R_n_w_ambf - R_n_w_rbdl) << std::endl;

      CHECK_THAT(R_n_w_ambf, AllCloseMatrix(R_n_w_rbdl, TEST_PREC, TEST_PREC));
      // CHECK_THAT(P_n_w_ambf, AllCloseMatrix(P_n_w_rbdl, TEST_PREC, TEST_PREC));
    }
  }
}