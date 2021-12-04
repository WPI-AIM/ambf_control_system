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
      const tf::Vector3 P_n_w_tf = rigidBodyHandler->get_pos();
      Eigen::Vector3d P_n_0_ambf;
      Eigen::Vector3d P_n_w_ambf;
      P_n_w_ambf[0] = P_n_w_tf[0];
      P_n_w_ambf[1] = P_n_w_tf[1];
      P_n_w_ambf[2] = P_n_w_tf[2];
      // P_n_w_ambf[3] = 1.0;
      P_n_0_ambf = T_0_w.block<3, 3>(0, 0).transpose() * P_n_w_ambf;

      const RigidBodyDynamics::Math::Vector3d P_n_0_rbdl = CalcBodyToBaseCoordinates(*rbdlPSModel, Q, rbdlBodyId, 
                                                              RigidBodyDynamics::Math::Vector3d(0., 0., 0.),true);

      std::cout << body << ": " << "P_n_w: "      << P_n_w_tf[0]   << ", " << P_n_w_tf[1]   << ", " << P_n_w_tf[2]   << std::endl;
      std::cout << body << ": " << "P_n_0_ambf: " << P_n_0_ambf[0] << ", " << P_n_0_ambf[1] << ", " << P_n_0_ambf[2] << std::endl;
      std::cout << body << ": " << "P_n_0_rbdl: " << P_n_0_rbdl[0] << ", " << P_n_0_rbdl[1] << ", " << P_n_0_rbdl[2] << std::endl;

      RigidBodyDynamics::Math::Vector3d P_n_w_rbd_ambf;
      P_n_w_rbd_ambf.setZero();
      
      P_n_w_rbd_ambf(0) = P_n_w_tf[0];
      P_n_w_rbd_ambf(1) = P_n_w_tf[1];
      P_n_w_rbd_ambf(2) = P_n_w_tf[2];

      // std::cout << std::endl << "P_n_w_rbd_ambf" << std::endl << P_n_w_rbd_ambf << std::endl;
      // std::cout << std::endl << "P_n_0_rbd_rbdl" << std::endl << P_n_0_rbd_rbdl << std::endl;

      RigidBodyDynamics::Math::Vector4d P_n_w_rbd_rbdl_4d;
      P_n_w_rbd_rbdl_4d.setOnes();
      P_n_w_rbd_rbdl_4d(0) = P_n_0_rbdl(0);
      P_n_w_rbd_rbdl_4d(1) = P_n_0_rbdl(1);
      P_n_w_rbd_rbdl_4d(2) = P_n_0_rbdl(2);

      P_n_w_rbd_rbdl_4d = T_0_w * (P_n_w_rbd_rbdl_4d);

      RigidBodyDynamics::Math::Vector3d P_n_w_rbd_rbdl;
      P_n_w_rbd_rbdl = P_n_w_rbd_rbdl_4d.block<3,1>(0,0);
      
      std::cout << std::endl << "P_n_w_diff" << std::endl << P_n_w_rbd_ambf - P_n_w_rbd_rbdl << std::endl;
      CHECK_THAT(P_n_w_rbd_ambf, AllCloseMatrix(P_n_w_rbd_rbdl, TEST_PREC, TEST_PREC));
    }
  }
}