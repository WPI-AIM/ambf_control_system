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
  rigidBodyPtr rigidBodyHandler;
};

struct ParallelStructure {
  ParallelStructure () {
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
    const double mass                         = 1.0;
    RigidBodyDynamics::Math::Vector3d com     = {     0.0, -0.34415,     0.0 };
    RigidBodyDynamics::Math::Vector3d inertia = { 0.31777,  0.00961, 0.31777 };

    worldBody = Body(0.0, RigidBodyDynamics::Math::Vector3d (0.0, 0.0, 0.0), 
      RigidBodyDynamics::Math::Vector3d (0.0, 0.0, 0.0));
    l1Body = Body(mass, com, inertia);
    l2Body = Body(mass, com, inertia);
    l3Body = Body(mass, com, inertia);
    l4Body = Body(mass, com, inertia);
    
    Eigen::Matrix3d world;
    world.setIdentity();
    //--------------------------------------------------------------------//
    Eigen::Vector3d world_l1PA = {   1.0,      0.0,    0.0 };
    Eigen::Vector3d world_l1CA = {   0.0, -0.00011,    1.0 };
    Eigen::Vector3d world_l1PP = { 0.001,    -0.36, -0.222 };
    Eigen::Vector3d world_l1CP = {   0.0,      0.0,    0.0 };

    world_l1PA.normalize();
    world_l1CA.normalize();

    Eigen::Matrix3d world_l1Rot = 
      Eigen::Matrix3d(Eigen::Quaterniond::FromTwoVectors(world_l1PA, 
      world_l1CA));

    Eigen::Matrix3d wordl_l1_offset_unit_vec = world * world_l1Rot;

    Eigen::Affine3d world_l1_offset(Eigen::AngleAxisd(world_l1Offset, 
      Eigen::Vector3d::UnitZ()));

    world_l1ST.E = world_l1_offset.rotation() * world_l1Rot;
    world_l1ST.r = world_l1PP - (world_l1Rot.inverse() * world_l1CP);
    world_l1Joint = 
      RigidBodyDynamics::Joint(RigidBodyDynamics::JointType::JointTypeRevolute,
      world_l1ST.E.transpose().block<3, 1>(0, 2).transpose());

    const Eigen::Vector3d P_world_l1 =  world * world_l1ST.r;
    world_l1ID = rbdlModel->AddBody(0, 
      RigidBodyDynamics::Math::Xtrans(P_world_l1), world_l1Joint, 
      l1Body, "world-l1");

    //--------------------------------------------------------------------//
    Eigen::Vector3d l1_l2PA = {     0.0, 0.00016, 1.0 };
    Eigen::Vector3d l1_l2CA = { 0.00009, 0.00000, 1.0 };
    Eigen::Vector3d l1_l2PP = {   0.139,   0.138, 0.0 };
    Eigen::Vector3d l1_l2CP = {     0.0,     0.0, 0.0 };
    l1_l2PA.normalize();
    l1_l2CA.normalize();

    Eigen::Matrix3d l1_l2Rot = 
    Eigen::Matrix3d(Eigen::Quaterniond::FromTwoVectors(l1_l2PA, l1_l2CA));

    Eigen::Matrix3d l1_l2_offset_unit_vec = world_l1ST.E * l1_l2Rot;
    Eigen::Affine3d l1_l2_offset(Eigen::AngleAxisd(l1_l2Offset, 
      -Eigen::Vector3d::UnitX()));

    l1_l2ST.E = l1_l2_offset.rotation() * l1_l2Rot;
    l1_l2ST.r = l1_l2PP - (l1_l2Rot.inverse() * l1_l2CP);
    
    const SpatialTransform world_l2ST = world_l1ST * l1_l2ST;
    l1_l2Joint = 
      RigidBodyDynamics::Joint(RigidBodyDynamics::JointType::JointTypeRevolute, 
      world_l2ST.E.transpose().block<3, 1>(0, 2).transpose());
    
    const Eigen::Vector3d P_world_l2 = world_l1ST.E.inverse() * l1_l2ST.r;
    
    l1_l2ID = rbdlModel->AddBody(world_l1ID, 
      RigidBodyDynamics::Math::Xtrans(P_world_l2), l1_l2Joint, 
      l2Body, "l1-l2");
    //--------------------------------------------------------------------//
    Eigen::Vector3d l2_l3PA = {    0.0,      0.0, 1.0 };
    Eigen::Vector3d l2_l3CA = {    0.0, -0.00016, 1.0 };
    Eigen::Vector3d l2_l3PP = { -0.141,   -0.832, 0.0 };
    Eigen::Vector3d l2_l3CP = {    0.0,      0.0, 0.0 };
    l2_l3PA.normalize();
    l2_l3CA.normalize();

    Eigen::Matrix3d l2_l3Rot = 
      Eigen::Matrix3d(Eigen::Quaterniond::FromTwoVectors (l2_l3PA, l2_l3CA));

    Eigen::Affine3d l2_l3_offset(Eigen::AngleAxisd(l2_l3Offset, 
      -Eigen::Vector3d::UnitX()));

    l2_l3ST.E = l2_l3_offset.rotation() * l2_l3Rot;
    l2_l3ST.r = l2_l3PP - (l2_l3Rot.inverse() * l2_l3CP);

    const SpatialTransform world_l3ST = world_l2ST * l2_l3ST;
    
    l2_l3Joint = 
      RigidBodyDynamics::Joint(RigidBodyDynamics::JointType::JointTypeRevolute, 
      world_l3ST.E.transpose().block<3, 1>(0, 2).transpose());
    
    const Eigen::Vector3d P_world_l3 = world_l2ST.E.inverse() * l2_l3ST.r;
    
    l2_l3ID = rbdlModel->AddBody(l1_l2ID, 
      RigidBodyDynamics::Math::Xtrans(P_world_l3), l2_l3Joint, 
      l3Body, "l2-l3");
    //--------------------------------------------------------------------//
    Eigen::Vector3d l3_l4PA = {      0.0, 0.00035, 1.0 };
    Eigen::Vector3d l3_l4CA = { -0.00017,     0.0, 1.0 };
    Eigen::Vector3d l3_l4PP = {    -0.14,   -0.83, 0.0 };
    Eigen::Vector3d l3_l4CP = {      0.0,     0.0, 0.0 };
    l3_l4PA.normalize();
    l3_l4CA.normalize();

    Eigen::Matrix3d l3_l4Rot = 
      Eigen::Matrix3d(Eigen::Quaterniond::FromTwoVectors(l3_l4PA, l3_l4CA));

    Eigen::Affine3d l3_l4_offset(Eigen::AngleAxisd(l3_l4Offset, 
      Eigen::Vector3d::UnitY()));

    l3_l4ST.E = l3_l4_offset.rotation() * l3_l4Rot;
    l3_l4ST.r = l3_l4PP - (l3_l4Rot.inverse() * l3_l4CP);

    const SpatialTransform world_l4ST = world_l3ST * l3_l4ST;
    
    l3_l4Joint = 
      RigidBodyDynamics::Joint(RigidBodyDynamics::JointType::JointTypeRevolute, 
      world_l4ST.E.transpose().block<3, 1>(0, 2).transpose());
    
    const Eigen::Vector3d P_world_l4 = world_l3ST.E.inverse() * l3_l4ST.r;
    
    l3_l4ID = rbdlModel->AddBody(l2_l3ID, 
      RigidBodyDynamics::Math::Xtrans(P_world_l4), l3_l4Joint, 
      l4Body, "l3-l4");
    //--------------------------------------------------------------------//
    Eigen::Vector3d l1_l4PA = {      0.0, 0.00016,       1.0 };
    Eigen::Vector3d l1_l4CA = {  0.00024,     0.0,       1.0 };
    Eigen::Vector3d l1_l4PP = {     0.07,   -0.77,       0.0 };
    Eigen::Vector3d l1_l4CP = { -0.06239, -0.76135, -0.00018 };
    l1_l4PA.normalize();
    l1_l4CA.normalize();

    Eigen::Matrix3d l1_l4Rot = 
          Eigen::Matrix3d(Eigen::Quaterniond::FromTwoVectors(l1_l4PA, l1_l4CA));
        Eigen::Affine3d l1_l4_offset(Eigen::AngleAxisd(l1_l4Offset, 
      Eigen::Vector3d::UnitZ()));

    l1_l4ST.E = l1_l4_offset.rotation() * l1_l4Rot;
    l1_l4ST.r = l1_l4PP - (l1_l4Rot.inverse() * l1_l4CP);

    const SpatialTransform world_l4ST_ = world_l1ST * l1_l4ST;
    
    l1_l4Joint = RigidBodyDynamics::Joint(RigidBodyDynamics::JointType::JointTypeRevolute, 
                                    world_l4ST_.E.transpose().block<3, 1>(0, 2).transpose());
    
    const Eigen::Vector3d P_world_l4_ = world_l4ST_.E.inverse() * l1_l4ST.r;
    l1_l4ID = rbdlModel->AddBody(world_l1ID, 
      RigidBodyDynamics::Math::Xtrans(P_world_l4_), l1_l4Joint, 
                              l4Body, "l1-l4");
    //--------------------------------------------------------------------//
    Q     = VectorNd::Constant ((size_t) rbdlModel->dof_count, 0.);
    QDot  = VectorNd::Constant ((size_t) rbdlModel->dof_count, 0.);
    QDDot = VectorNd::Constant ((size_t) rbdlModel->dof_count, 0.);
    Tau   = VectorNd::Constant ((size_t) rbdlModel->dof_count, 0.); 

    ClearLogOutput();
  }

  ~ParallelStructure() 
  {
    delete rbdlModel;
    clientPtr->cleanUp();
  }

  Model *rbdlModel = nullptr;

  AMBFClientPtr clientPtr = nullptr;
  rigidBodyPtr baseHandler = nullptr;
  std::string base_name = "world";

  Eigen::Matrix4d T_w_0;
  Eigen::Matrix4d T_0_w;

  // Child : Parent,
  std::unordered_map<std::string, std::string> hierachyMap =
  {
    {     "ROOT", "ROOT"  },
    { "world-l1", "ROOT"  },
    {    "l1-l2", "ROOT"  },
    {    "l1-l4", "ROOT"  },
    {    "l2-l3", "l1-l2" },
    {    "l3-l4", "l2-l3" }
  };

  // AMBF joint name & RBDL Body Name, AMBF body name, limits. 
  // TBD limits to be taken from RBDL model  
  // static const inline 
  std::unordered_map<std::string, ActivationJoints> RBDL_AMBF_JOINT_MAP =
  {
    { 
      "world-l1", { "l1",    0.0f,   0.0f, nullptr }
    },
    { 
         "l1-l2", { "l2", -1.047f, 1.047f, nullptr }
    },
    { 
         "l2-l3", { "l3", -1.047f, 1.047f, nullptr }
    },
    { 
         "l3-l4", { "l4", -1.047f, 1.047f, nullptr }
    }
  };

  std::unordered_map<std::string, ActivationJoints>::const_iterator RBDL_AMBF_JOINT_MAP_itr;

  Body worldBody, l1Body, l2Body, l3Body, l4Body;
  unsigned int world_l1ID, l1_l2ID, l2_l3ID, l3_l4ID, l1_l4ID;
  Joint world_l1Joint, l1_l2Joint, l2_l3Joint, l3_l4Joint, l1_l4Joint;
  SpatialTransform world_l1ST, l1_l2ST, l2_l3ST, l3_l4ST, l1_l4ST;

  const double world_l1Offset =  3.14189;
  const double l1_l2Offset    =  1.575;
  const double l2_l3Offset    = -1.575;
  const double l3_l4Offset    = -1.567;
  const double l1_l4Offset    = -1.567;

  VectorNd Q;
  VectorNd QDot;
  VectorNd QDDot;
  VectorNd Tau;
};