#include "rbdl_model_tests/RBDLTestPrep.h"
#include "rbdl_model_tests/EigenUtilities.h"
#include <unordered_map>
#include <Eigen/Geometry> 

//const double TEST_PREC = 1.0e-12;
const double TEST_LAX = 1.0e-7;
struct ActivationJoints
{
  std::string bodyNameAMBF;
  float jointLowerLimit;
  float jointHigherLimit;
};

struct KUKA {
  KUKA () {
    ClearLogOutput();

    clientPtr = RBDLTestPrep::getInstance()->getAMBFClientInstance();
    clientPtr->connect();

    baseHandler = clientPtr->getRigidBody(base_name, true);
    usleep(1000000);

    //base is rigid body name, not a joint. This is a hacky way to enable ros topics in the 
    //server side during first execution
    baseHandler->set_joint_pos(base_name, 0.0f); 

    const tf::Quaternion quat_w_0_tf = baseHandler->get_rot();
    const tf::Vector3 P_w_0_tf = baseHandler->get_pos();

    RigidBodyDynamics::Math::Quaternion quat_w_0;
    quat_w_0(0) = quat_w_0_tf[0];
    quat_w_0(1) = quat_w_0_tf[1];
    quat_w_0(2) = quat_w_0_tf[2];
    quat_w_0(3) = quat_w_0_tf[3];

    const RigidBodyDynamics::Math::Matrix3d R_w_0 = quat_w_0.toMatrix();

    RigidBodyDynamics::Math::Vector3d P_w_0;
    P_w_0.setZero();
    
    P_w_0(0) = P_w_0_tf[0];
    P_w_0(1) = P_w_0_tf[1];
    P_w_0(2) = P_w_0_tf[2];

    T_w_0 = EigenUtilities::get_frame<Eigen::Matrix3d, 
                Eigen::Vector3d, Eigen::Matrix4d>(R_w_0, P_w_0);
    
    T_0_w = EigenUtilities::get_frame<Eigen::Matrix3d, 
                Eigen::Vector3d, Eigen::Matrix4d>(R_w_0.transpose(), -R_w_0.transpose() * P_w_0);
    
    rbdlModel = new Model;
    rbdlModel->gravity = RigidBodyDynamics::Math::Vector3d(0., 0., -9.81);

    // mass, com - inertia offset, inertia
    baseBody  = Body (1., Math::Vector3d (00.001, 00.000, 0.060), Math::Vector3d (0.0000, 0.0000, 0.0000));
    link1Body = Body (1., Math::Vector3d (00.000, -0.017, 0.134), Math::Vector3d (0.0452, 0.0446, 0.0041));
    link2Body = Body (1., Math::Vector3d (00.000, -0.074, 0.009), Math::Vector3d (0.0227, 0.0037, 0.0224));
    link3Body = Body (1., Math::Vector3d (00.000, 00.017, 0.134), Math::Vector3d (0.0417, 0.0418, 0.0038));
    link4Body = Body (1., Math::Vector3d (-0.001, 00.081, 0.008), Math::Vector3d (0.0249, 0.0036, 0.0247));
    link5Body = Body (1., Math::Vector3d (0.0000, -0.017, 0.129), Math::Vector3d (0.0363, 0.0350, 0.0045));
    link6Body = Body (1., Math::Vector3d (0.0000, 00.007, 0.068), Math::Vector3d (0.0114, 0.0116, 0.0037));
    link7Body = Body (1., Math::Vector3d (0.0060, 00.000, 0.015), Math::Vector3d (0.0012, 0.0013, 0.0010));

    Eigen::Matrix3d base;
    base.setIdentity();
    //--------------------------------------------------------------------//
    Eigen::Vector3d base_link1PA = { 00.000, 00.000, 01.000 };
    Eigen::Vector3d base_link1CA = { 00.000, 00.000, 01.000 };
    Eigen::Vector3d base_link1PP = { 00.000, 00.000, 00.103 };
    Eigen::Vector3d base_link1CP = { 00.000, 00.000, 00.000 };
    base_link1PA.normalize();
    base_link1CA.normalize();

    Eigen::Matrix3d base_link1Rot = 
      Eigen::Matrix3d(Eigen::Quaterniond::FromTwoVectors(base_link1PA, 
                                                         base_link1CA));
    Eigen::Affine3d base_link1_offset(Eigen::AngleAxisd(base_link1Offset, 
      Eigen::Vector3d::UnitZ()));
    // Eigen::Matrix3d base_link1_offset = EigenUtilities::rotZ(base_link1Offset);

    base_link1ST.E = base_link1_offset.rotation() * base_link1Rot;
    base_link1ST.r = base_link1PP - (base_link1Rot.inverse() * base_link1CP);

    base_link1Joint = 
      RigidBodyDynamics::Joint(RigidBodyDynamics::JointType::JointTypeRevolute, 
      base_link1ST.E.transpose().block<3, 1>(0, 2).transpose());
    
    const Eigen::Vector3d P_base_link1_base =  base * base_link1ST.r;
    base_link1ID = rbdlModel->AddBody(0, 
      RigidBodyDynamics::Math::Xtrans(P_base_link1_base), 
      base_link1Joint, link1Body, "base-link1");
    //--------------------------------------------------------------------//
    Eigen::Vector3d link1_link2PA = { 00.000, 01.000, 00.000 };
    Eigen::Vector3d link1_link2CA = { 00.000, 00.000, 01.000 };
    Eigen::Vector3d link1_link2PP = { 00.000, 00.013, 00.209 };
    Eigen::Vector3d link1_link2CP = { 00.000, 00.000, 00.000 };
    link1_link2PA.normalize();
    link1_link2CA.normalize();

    Eigen::Matrix3d link1_link2Rot = 
      Eigen::Matrix3d(Eigen::Quaterniond::FromTwoVectors(link1_link2PA, 
      link1_link2CA));
    Eigen::Affine3d link1_link2_offset(Eigen::AngleAxisd(link1_link2Offset, 
      Eigen::Vector3d::UnitZ()));
    // Eigen::Matrix3d link1_link2_offset = 
    //   EigenUtilities::rotZ(link1_link2Offset);

    link1_link2ST.E = link1_link2_offset.rotation() * link1_link2Rot;
    link1_link2ST.r = 
      link1_link2PP - (link1_link2Rot.inverse() * link1_link2CP);
    
    const SpatialTransform base_link2ST = base_link1ST * link1_link2ST;

    link1_link2Joint = 
      RigidBodyDynamics::Joint(RigidBodyDynamics::JointType::JointTypeRevolute, 
      base_link2ST.E.transpose().block<3, 1>(0, 2).transpose());

    const Eigen::Vector3d P_base_link2_base = 
      base_link1ST.E.inverse() * link1_link2ST.r;
    link1_link2ID = rbdlModel->AddBody(base_link1ID, 
      RigidBodyDynamics::Math::Xtrans(P_base_link2_base), 
      link1_link2Joint, link2Body, "link1-link2");
    //--------------------------------------------------------------------//
    Eigen::Vector3d link2_link3PA = { 00.000, -1.000, 00.000 };
    Eigen::Vector3d link2_link3CA = { 00.000, 00.000, 01.000 };
    Eigen::Vector3d link2_link3PP = { 00.000, -0.194, -0.009 };
    Eigen::Vector3d link2_link3CP = { 00.000, 00.000, 00.000 };
    link2_link3PA.normalize();
    link2_link3CA.normalize();

    Eigen::Matrix3d link2_link3Rot = 
      Eigen::Matrix3d(Eigen::Quaterniond::FromTwoVectors(link2_link3PA, 
                                                          link2_link3CA));
    Eigen::Affine3d link2_link3_offset(Eigen::AngleAxisd(link2_link3Offset, 
      Eigen::Vector3d::UnitZ()));
    // Eigen::Matrix3d link2_link3_offset = 
    //   EigenUtilities::rotZ(link2_link3Offset);

    link2_link3ST.E = link2_link3_offset.rotation() * link2_link3Rot;
    link2_link3ST.r = 
      link2_link3PP - (link2_link3Rot.inverse() * link2_link3CP);   

    const SpatialTransform base_link3ST = base_link2ST * link2_link3ST;

    link2_link3Joint = 
      RigidBodyDynamics::Joint(RigidBodyDynamics::JointType::JointTypeRevolute, 
      base_link3ST.E.transpose().block<3, 1>(0, 2).transpose());

    const Eigen::Vector3d P_base_link3_base = 
      base_link2ST.E.inverse() * link2_link3ST.r;
    link2_link3ID = rbdlModel->AddBody(link1_link2ID, 
      RigidBodyDynamics::Math::Xtrans(P_base_link3_base), 
      link2_link3Joint, link3Body, "link2-link3");
    //--------------------------------------------------------------------//
    Eigen::Vector3d link3_link4PA = { 00.000, -1.000, 00.000 };
    Eigen::Vector3d link3_link4CA = { 00.000, 00.000, 01.000 };
    Eigen::Vector3d link3_link4PP = { 00.000, -0.013, 00.202 };
    Eigen::Vector3d link3_link4CP = { 00.000, 00.000, 00.000 };
    link3_link4PA.normalize();
    link3_link4CA.normalize();

    Eigen::Matrix3d link3_link4Rot = 
      Eigen::Matrix3d(Eigen::Quaterniond::FromTwoVectors(link3_link4PA, 
      link3_link4CA));
    Eigen::Affine3d link3_link4_offset(Eigen::AngleAxisd(link3_link4Offset, 
      Eigen::Vector3d::UnitZ()));
    // Eigen::Matrix3d link3_link4_offset = 
    //   EigenUtilities::rotZ(link3_link4Offset);

    link3_link4ST.E = link3_link4_offset.rotation() * link3_link4Rot;
    link3_link4ST.r = 
      link3_link4PP - (link3_link4Rot.inverse() * link3_link4CP);

    const SpatialTransform base_link4ST = base_link3ST * link3_link4ST;

    link3_link4Joint = 
      RigidBodyDynamics::Joint(RigidBodyDynamics::JointType::JointTypeRevolute, 
      base_link4ST.E.transpose().block<3, 1>(0, 2).transpose());

    const Eigen::Vector3d P_base_link4_base = 
      base_link3ST.E.inverse() * link3_link4ST.r;
    link3_link4ID = rbdlModel->AddBody(link2_link3ID, 
      RigidBodyDynamics::Math::Xtrans(P_base_link4_base), link3_link4Joint, 
      link4Body, "link3-link4");
    //--------------------------------------------------------------------//
    Eigen::Vector3d link4_link5PA = { 00.000, 01.000, 00.000 };
    Eigen::Vector3d link4_link5CA = { 00.000, 00.000, 01.000 };
    Eigen::Vector3d link4_link5PP = { -0.002, 00.202, -0.008 };
    Eigen::Vector3d link4_link5CP = { 00.000, 00.000, 00.000 };
    link4_link5PA.normalize();
    link4_link5CA.normalize();

    Eigen::Matrix3d link4_link5Rot = 
      Eigen::Matrix3d(Eigen::Quaterniond::FromTwoVectors(link4_link5PA, 
      link4_link5CA));
    Eigen::Affine3d link4_link5_offset(Eigen::AngleAxisd(link4_link5Offset, 
      Eigen::Vector3d::UnitZ()));
    // Eigen::Matrix3d link4_link5_offset = 
    //   EigenUtilities::rotZ(link5_link6Offset);

    link4_link5ST.E = link4_link5_offset.rotation() * link4_link5Rot;
    link4_link5ST.r = 
      link4_link5PP - (link4_link5Rot.inverse() * link4_link5CP);

    const SpatialTransform base_link5ST = base_link4ST * link4_link5ST;

    const Eigen::Vector3d P_base_link5_base = 
      base_link4ST.E.inverse() * link4_link5ST.r;

    link4_link5Joint = 
      RigidBodyDynamics::Joint(RigidBodyDynamics::JointType::JointTypeRevolute, 
      base_link5ST.E.transpose().block<3, 1>(0, 2).transpose());

    link4_link5ID = rbdlModel->AddBody(link3_link4ID, 
      RigidBodyDynamics::Math::Xtrans(P_base_link5_base), link4_link5Joint, 
      link5Body, "link4-link5");
    //--------------------------------------------------------------------//
    Eigen::Vector3d link5_link6PA = { 00.000, 01.000, 00.000 };
    Eigen::Vector3d link5_link6CA = { 00.000, 00.000, 01.000 };
    Eigen::Vector3d link5_link6PP = { 00.002, -0.052, 00.204 };
    Eigen::Vector3d link5_link6CP = { 00.000, 00.000, 00.000 };
    link5_link6PA.normalize();
    link5_link6CA.normalize();

    Eigen::Matrix3d link5_link6Rot = 
      Eigen::Matrix3d(Eigen::Quaterniond::FromTwoVectors(link5_link6PA, 
      link5_link6CA));
    Eigen::Affine3d link5_link6_offset(Eigen::AngleAxisd(link5_link6Offset, 
      Eigen::Vector3d::UnitZ()));
    // Eigen::Matrix3d link5_link6_offset = 
    //   EigenUtilities::rotZ(link5_link6Offset);

    link5_link6ST.E = link5_link6_offset.rotation() * link5_link6Rot;
    link5_link6ST.r = 
      link5_link6PP - (link5_link6Rot.inverse() * link5_link6CP);

    const SpatialTransform base_link6ST = base_link5ST * link5_link6ST;

    const Eigen::Vector3d P_base_link6_base = 
      base_link5ST.E.inverse() * link5_link6ST.r;

    link5_link6Joint = 
      RigidBodyDynamics::Joint(RigidBodyDynamics::JointType::JointTypeRevolute, 
      base_link6ST.E.transpose().block<3, 1>(0, 2).transpose());
    link5_link6ID = rbdlModel->AddBody(link4_link5ID, 
      RigidBodyDynamics::Math::Xtrans(P_base_link6_base), link5_link6Joint, 
                                                            link6Body, "link5-link6");
    //--------------------------------------------------------------------//
    Eigen::Vector3d link6_link7PA = { 00.000, -1.000, 00.000 };
    Eigen::Vector3d link6_link7CA = { 00.000, 00.000, 01.000 };
    Eigen::Vector3d link6_link7PP = { -0.003, -0.050, 00.053 };
    Eigen::Vector3d link6_link7CP = { 00.000, 00.000, 00.000 };
    link6_link7PA.normalize();
    link6_link7CA.normalize();

    Eigen::Matrix3d link6_link7Rot = 
      Eigen::Matrix3d(Eigen::Quaterniond::FromTwoVectors(link6_link7PA, 
      link6_link7CA));
    Eigen::Affine3d link6_link7_offset(Eigen::AngleAxisd(link6_link7Offset, 
      Eigen::Vector3d::UnitZ()));
    // Eigen::Matrix3d link6_link7_offset = 
    //   EigenUtilities::rotZ(link6_link7Offset);

    link6_link7ST.E = link6_link7_offset.rotation() * link6_link7Rot;
    link6_link7ST.r = 
      link6_link7PP - (link6_link7Rot.inverse() * link6_link7CP);

    const SpatialTransform base_link7ST = base_link6ST * link6_link7ST;

    const Eigen::Vector3d P_base_link7_base = 
      base_link6ST.E.inverse() * link6_link7ST.r;

    link6_link7Joint = 
      RigidBodyDynamics::Joint(RigidBodyDynamics::JointType::JointTypeRevolute, 
      base_link7ST.E.transpose().block<3, 1>(0, 2).transpose());

    link6_link7ID = rbdlModel->AddBody(link5_link6ID, 
      RigidBodyDynamics::Math::Xtrans(P_base_link7_base), link6_link7Joint, 
      link7Body, "link6-link7");
    //--------------------------------------------------------------------//
    Q     = VectorNd::Constant ((size_t) rbdlModel->dof_count, 0.);
    QDot  = VectorNd::Constant ((size_t) rbdlModel->dof_count, 0.);
    QDDot = VectorNd::Constant ((size_t) rbdlModel->dof_count, 0.);
    Tau   = VectorNd::Constant ((size_t) rbdlModel->dof_count, 0.); 

    ClearLogOutput();
  }

  ~KUKA () 
  {
    delete rbdlModel;
    clientPtr->cleanUp();
  }

  Model *rbdlModel = nullptr;

  AMBFClientPtr clientPtr = nullptr;
  rigidBodyPtr baseHandler = nullptr;
  std::string base_name = "base";

  Eigen::Matrix4d T_w_0;
  Eigen::Matrix4d T_0_w;

  // Child : Parent
  std::unordered_map<std::string, std::string> hierachyMap =
  {
    {  "ROOT", "ROOT" },
    {  "ROOT-base", "ROOT" },
    { "base-link1", "ROOT" },
    { "link1-link2", "base-link1" },
    { "link2-link3", "link1-link2" },
    { "link3-link4", "link2-link3" },
    { "link4-link5", "link3-link4" },
    { "link5-link6", "link4-link5" },
    { "link6-link7", "link5-link6" },
  };

  // AMBF joint name & RBDL Body Name, AMBF body name, limits. 
  // TBD limits to be taken from RBDL model  
  static const inline std::unordered_map<std::string, ActivationJoints> RBDL_AMBF_JOINT_MAP =
  {
    { 
      "base-link1", { "link1", -2.094f, 2.094f }
    },
    { 
      "link1-link2", { "link2", -2.094f, 2.094f }
    },
    { 
      "link2-link3", { "link3", -2.094f, 2.094f }
    },
    { 
      "link3-link4", { "link4", -2.094f, 2.094f }
    },
    { 
      "link4-link5", { "link5", -2.094f, 2.094f }
    },
    { 
      "link5-link6", { "link6", -2.094f, 2.094f }
    },
    { 
      "link6-link7", { "link7", -3.054f, 3.054f }
    }
  };

  Body baseBody, link1Body, link2Body, link3Body, link4Body, link5Body, link6Body, link7Body;

  unsigned int ROOT_baseID, base_link1ID, link1_link2ID, link2_link3ID, 
               link3_link4ID, link4_link5ID, link5_link6ID, link6_link7ID;

  Joint ROOT_baseJoint, base_link1Joint, link1_link2Joint, link2_link3Joint, 
        link3_link4Joint, link4_link5Joint, link5_link6Joint, link6_link7Joint;

  SpatialTransform ROOT_baseST, base_link1ST, link1_link2ST, link2_link3ST, link3_link4ST, 
                   link4_link5ST, link5_link6ST, link6_link7ST;

  const double  ROOT_baseOffset  = 0.0;
  const double  base_link1Offset = 0.0;
  const double link1_link2Offset = 0.0;
  const double link2_link3Offset = 0.0;
  const double link3_link4Offset = 0.0;
  const double link4_link5Offset = 0.0;
  const double link5_link6Offset = 0.0;
  const double link6_link7Offset = 0.0;

  VectorNd Q;
  VectorNd QDot;
  VectorNd QDDot;
  VectorNd Tau;
};