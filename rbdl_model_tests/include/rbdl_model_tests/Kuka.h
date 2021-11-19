#include "rbdl_model_tests/RBDLTestPrep.h"
#include "rbdl_model_tests/EigenUtilities.h"
//#include "rbdl_model_tests/Human36Fixture.h"
#include <unordered_map>

//const double TEST_PREC = 1.0e-12;
const double TEST_LAX = 1.0e-7;

struct Kuka {
  Kuka () {
    ClearLogOutput();
    
    clientPtr = RBDLTestPrep::getInstance()->getAMBFClientInstance();
    clientPtr->connect();


    baseHandler = clientPtr->getRigidBody(base_name, true);
    usleep(1000000);

    //base is rigid body name, not a joint. This is a hacky way to enable ros topics in the 
    //server side during first execution
    baseHandler->set_joint_pos(base_name, 0.0f); 

    tf::Vector3 P_0_w_tf = baseHandler->get_pos();
    Eigen::Vector3d P_0_w;
    P_0_w[0]= P_0_w_tf[0];
    P_0_w[1]= P_0_w_tf[1];
    P_0_w[2]= P_0_w_tf[2];
    
    tf::Vector3 R_0_w_tf = baseHandler->get_rpy();

    Eigen::Matrix3d R_0_w = EigenUtilities::rotation_from_euler<Eigen::Matrix3d>(R_0_w_tf[0], R_0_w_tf[1], R_0_w_tf[2]);

    T_0_w = EigenUtilities::get_frame<Eigen::Matrix3d, Eigen::Vector3d, Eigen::Matrix4d>(R_0_w, P_0_w);
    
    rbdlModel = new Model;

    base = Body(1., RigidBodyDynamics::Math::Vector3d (0.001, 0., 0.06), 
           RigidBodyDynamics::Math::Vector3d (0., 0., 0.));
    ROOT_base = Joint(JointTypeFixed);

    ROOT_baseST.E = RigidBodyDynamics::Math::Matrix3dIdentity;
    ROOT_baseST.r = RigidBodyDynamics::Math::Vector3dZero;
    base_id = rbdlModel->AddBody(0, ROOT_baseST, ROOT_base, base, "base");


    link1 = Body (1., RigidBodyDynamics::Math::Vector3d (0., -0.017, 0.134), 
          RigidBodyDynamics::Math::Vector3d (0.0452, 0.0446, 0.0041));
    base_link1 = Joint(JointTypeRevolute, Math::Vector3d(0.0, 0.0, 1.0));

    base_link1ST.E = RigidBodyDynamics::Math::Matrix3dIdentity;
    base_link1ST.r = RigidBodyDynamics::Math::Vector3d(0., 0.0, 0.103);
    link1_id = rbdlModel->AddBody(base_id, base_link1ST, base_link1, link1, "link1");


    link2 = Body (1., RigidBodyDynamics::Math::Vector3d (0., -0.074, 0.009), 
                  RigidBodyDynamics::Math::Vector3d (0.0227, 0.0037, 0.0224));
    link1_link2 = Joint(JointTypeRevolute, Math::Vector3d(0.0, 0.0, 1.0));

    link1_link2ST.E = RigidBodyDynamics::Math::Matrix3dZero;
    link1_link2ST.E(0, 0) = 1.0;
    link1_link2ST.E(1, 2) = -1.0;
    link1_link2ST.E(2, 1) = 1.0;
    link1_link2ST.r = RigidBodyDynamics::Math::Vector3d(0., 0.013, 0.209);

    link2_id = rbdlModel->AddBody(link1_id, link1_link2ST, link1_link2, link2, "link2");


    link3 = Body (1., RigidBodyDynamics::Math::Vector3d (0., 0.017, 0.134), 
                  RigidBodyDynamics::Math::Vector3d (0.0417, 0.0418, 0.0038));
    link2_link3 = Joint(JointTypeRevolute, Math::Vector3d(0.0, 0.0, 1.0));

    link2_link3ST.E = RigidBodyDynamics::Math::Matrix3dZero;
    link2_link3ST.E(0, 0) = 1.0;
    link2_link3ST.E(1, 2) = 1.0;
    link2_link3ST.E(2, 1) = -1.0;
    link2_link3ST.r = RigidBodyDynamics::Math::Vector3d(0., -0.194, -0.009);
    link3_id = rbdlModel->AddBody(link2_id, link2_link3ST, link2_link3, link3, "link3");


    link4 = Body (1., RigidBodyDynamics::Math::Vector3d (-0.001, 0.081, 0.008), 
                  RigidBodyDynamics::Math::Vector3d (0.0249, 0.0036, 0.0247));
    link3_link4 = Joint(JointTypeRevolute, Math::Vector3d(0.0, 0.0, 1.0));

    link3_link4ST.E = RigidBodyDynamics::Math::Matrix3dZero;
    link3_link4ST.E(0, 0) = 1.0;
    link3_link4ST.E(1, 2) = 1.0;
    link3_link4ST.E(2, 1) = -1.0;
    link3_link4ST.r = RigidBodyDynamics::Math::Vector3d(0., -0.013, 0.202);
    link4_id = rbdlModel->AddBody(link3_id, link3_link4ST, link3_link4, link4, "link4");

    link5 = Body (1., RigidBodyDynamics::Math::Vector3d (0.0, -0.017, 0.129), 
                  RigidBodyDynamics::Math::Vector3d (0.0363, 0.035, 0.0045));
    link4_link5 = Joint(JointTypeRevolute, Math::Vector3d(0.0, 0.0, 1.0));

    link4_link5ST.E = RigidBodyDynamics::Math::Matrix3dZero;
    link4_link5ST.E(0, 0) = 1.0;
    link4_link5ST.E(1, 2) = -1.0;
    link4_link5ST.E(2, 1) = 1.0;
    link4_link5ST.r = RigidBodyDynamics::Math::Vector3d(-0.002, 0.202, -0.008);
    link5_id = rbdlModel->AddBody(link4_id, link4_link5ST, link4_link5, link5, "link5");

    link6 = Body (1., RigidBodyDynamics::Math::Vector3d (0., 0.007, 0.068), 
              RigidBodyDynamics::Math::Vector3d (0.0114, 0.0116, 0.0037));
    link5_link6 = Joint(JointTypeRevolute, Math::Vector3d(0.0, 0.0, 1.0));

    link5_link6ST.E = RigidBodyDynamics::Math::Matrix3dZero;
    link5_link6ST.E(0, 0) = 1.0;
    link5_link6ST.E(1, 2) = -1.0;
    link5_link6ST.E(2, 1) = 1.0;
    link5_link6ST.r = RigidBodyDynamics::Math::Vector3d(0.002, -0.052, 0.204);
    link6_id = rbdlModel->AddBody(link5_id, link5_link6ST, link5_link6, link6, "link6");

    link7 = Body (1., RigidBodyDynamics::Math::Vector3d (0.006, 0., 0.015), 
              RigidBodyDynamics::Math::Vector3d (0.0012, 0.0013, 0.001));
    link6_link7 = Joint(JointTypeRevolute, Math::Vector3d(0.0, 0.0, 1.0));

    link6_link7ST.E = RigidBodyDynamics::Math::Matrix3dZero;
    link6_link7ST.E(0, 0) = 1.0;
    link6_link7ST.E(1, 2) = 1.0;
    link6_link7ST.E(2, 1) = -1.0;
    link6_link7ST.r = RigidBodyDynamics::Math::Vector3d(-0.003, -0.05, 0.053);
    link6_id = rbdlModel->AddBody(link6_id, link6_link7ST, link6_link7, link7, "link7");

    Q = VectorNd::Constant ((size_t) rbdlModel->dof_count, 0.);
    QDot = VectorNd::Constant ((size_t) rbdlModel->dof_count, 0.);
    QDDot = VectorNd::Constant ((size_t) rbdlModel->dof_count, 0.);
    Tau = VectorNd::Constant ((size_t) rbdlModel->dof_count, 0.);

    KUKA_JOINT_LIMITS[ "link1" ] = { -2.094f, 2.094f };
    KUKA_JOINT_LIMITS[ "link2" ] = { -2.094f, 2.094f };
    KUKA_JOINT_LIMITS[ "link3" ] = { -2.094f, 2.094f };
    KUKA_JOINT_LIMITS[ "link4" ] = { -2.094f, 2.094f }; 
    KUKA_JOINT_LIMITS[ "link5" ] = { -2.094f, 2.094f }; 
    KUKA_JOINT_LIMITS[ "link6" ] = { -2.094f, 2.094f }; 
    KUKA_JOINT_LIMITS[ "link7" ] = { -3.054f, 3.054f }; 

    ClearLogOutput();
  }

  ~Kuka () {
    delete rbdlModel;
    clientPtr->cleanUp();
  }

  Model *rbdlModel = nullptr;
  AMBFClientPtr clientPtr = nullptr;
  rigidBodyPtr baseHandler = nullptr;
  std::string base_name = "base";
  Eigen::Matrix4d T_0_w;
  // std::vector<std::vector<float>> KUKA_JOINT_LIMITS;
  std::unordered_map<std::string, std::vector<float>> KUKA_JOINT_LIMITS;
  std::unordered_map<std::string, std::vector<float>>::iterator KUKA_JOINT_LIMITS_itr;

  //EigenUtilities eigenUtilities;

  unsigned int base_id, link1_id, link2_id, link3_id, link4_id, link5_id, link6_id, link7_id;
  Body base, link1, link2, link3, link4, link5, link6, link7;
  Joint ROOT_base, base_link1, link1_link2, link2_link3, link3_link4, link4_link5, link5_link6, link6_link7;
  SpatialTransform ROOT_baseST, base_link1ST, link1_link2ST, link2_link3ST, link3_link4ST, 
                   link4_link5ST, link5_link6ST, link6_link7ST;

  VectorNd Q;
  VectorNd QDot;
  VectorNd QDDot;
  VectorNd Tau;
};