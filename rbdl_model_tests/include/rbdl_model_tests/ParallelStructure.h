#include "rbdl_model_tests/RBDLTestPrep.h"
#include "rbdl_model_tests/EigenUtilities.h"
//#include "rbdl_model_tests/Human36Fixture.h"
#include <unordered_map>
#include <Eigen/Geometry> 

//const double TEST_PREC = 1.0e-12;
const double TEST_LAX = 1.0e-7;

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

    tf::Vector3 P_0_w_tf = baseHandler->get_pos();
    Eigen::Vector3d P_0_w;
    P_0_w[0]= P_0_w_tf[0];
    P_0_w[1]= P_0_w_tf[1];
    P_0_w[2]= P_0_w_tf[2];
    
    const tf::Quaternion rot_quat = baseHandler->get_rot();
    tf::Vector3 ryp_0_w_tf = baseHandler->get_rpy();
    Eigen::Matrix3d R_0_w = EigenUtilities::rotation_from_euler<Eigen::Matrix3d>(ryp_0_w_tf[0], ryp_0_w_tf[1], ryp_0_w_tf[2]);
    
    T_0_w = EigenUtilities::get_frame<Eigen::Matrix3d, Eigen::Vector3d, Eigen::Matrix4d>(R_0_w, P_0_w);

    rbdlPSModel = new Model;

    rbdlPSModel->gravity = RigidBodyDynamics::Math::Vector3d(0., 0., -9.81);
    // mass, com - inertia offset, inertia
    const double mass                         = 1.0;
    RigidBodyDynamics::Math::Vector3d com     = {     0.0, -0.34415,     0.0 };
    RigidBodyDynamics::Math::Vector3d inertia = { 0.31777,  0.00961, 0.31777 };

    world = Body(0.0, RigidBodyDynamics::Math::Vector3d (0.0, 0.0, 0.0), 
           RigidBodyDynamics::Math::Vector3d (0.0, 0.0, 0.0));
    l1 = Body(mass, com, inertia);
    l2 = Body(mass, com, inertia);
    l3 = Body(mass, com, inertia);
    l4 = Body(mass, com, inertia);
    //--------------------------------------------------------------------//
    ROOT_worldJoint = Joint(JointTypeFixed);
    ROOT_worldST.E = RigidBodyDynamics::Math::Matrix3dIdentity;
    ROOT_worldST.r = RigidBodyDynamics::Math::Vector3dZero;
    worldId = rbdlPSModel->AddBody(0, ROOT_worldST, ROOT_worldJoint, world, "world");
    //--------------------------------------------------------------------//
    Eigen::Vector3d world_l1PA = {   1.0,      0.0,    0.0 };
    Eigen::Vector3d world_l1CA = {   0.0, -0.00011,    1.0 };
    Eigen::Vector3d world_l1PP = { 0.001,    -0.36, -0.222 };
    Eigen::Vector3d world_l1CP = {   0.0,      0.0,    0.0 };
    world_l1PA.normalize();
    world_l1CA.normalize();

    Eigen::Matrix3d world_l1_Rot = 
                Eigen::Matrix3d(Eigen::Quaterniond::FromTwoVectors(world_l1PA, world_l1CA));
    Eigen::Matrix3d world_l1_offset = EigenUtilities::rotZ(3.14189);
    world_l1ST.E = world_l1_offset * world_l1_Rot ;
    world_l1ST.r = world_l1PP - (world_l1_Rot.inverse() * world_l1CP);

    world_l1Joint = Joint(JointTypeFixed);
    l1Id = rbdlPSModel->AddBody(worldId, world_l1ST, world_l1Joint, l1, "l1");
    //--------------------------------------------------------------------// 
    Eigen::Vector3d l1_l2PA = {   0.0, 0.00016, 1.0 };
    Eigen::Vector3d l1_l2CA = {   0.0, 0.00009, 1.0 };
    Eigen::Vector3d l1_l2PP = { 0.139,   0.138, 0.0 };
    Eigen::Vector3d l1_l2CP = {   0.0,     0.0, 0.0 };
    l1_l2PA.normalize();
    l1_l2CA.normalize();

    Eigen::Matrix3d l1_l2_Rot = Eigen::Matrix3d(Eigen::Quaterniond::FromTwoVectors(l1_l2PA, l1_l2CA));
    Eigen::Matrix3d l1_l2_offset = EigenUtilities::rotZ(1.575);
    l1_l2ST.E = l1_l2_offset * l1_l2_Rot;
    l1_l2ST.r = l1_l2PP - (l1_l2_Rot.inverse() * l1_l2CP);

    l1_l2Joint = Joint(JointTypeRevolute, Math::Vector3d(0.0, 0.0, 1.0));
    l2Id = rbdlPSModel->AddBody(l1Id, l1_l2ST, l1_l2Joint, l2, "l2");
    //--------------------------------------------------------------------//
    Eigen::Vector3d l2_l3PA = {    0.0,    0.0,      1.0 };
    Eigen::Vector3d l2_l3CA = {    0.0,    0.0, -0.00016 };
    Eigen::Vector3d l2_l3PP = { -0.141, -0.832,      0.0 };
    Eigen::Vector3d l2_l3CP = {   0.0,     0.0,      0.0 };
    l2_l3PA.normalize();
    l2_l3CA.normalize();

    Eigen::Matrix3d l2_l3_Rot = Eigen::Matrix3d(Eigen::Quaterniond::FromTwoVectors(l2_l3PA, l2_l3CA));
    Eigen::Matrix3d l2_l3_offset = EigenUtilities::rotZ(-1.575);
    l2_l3ST.E = l2_l3_offset * l2_l3_Rot;
    l2_l3ST.r = l2_l3PP - (l2_l3_Rot.inverse() * l2_l3CP);

    l2_l3Joint = Joint(JointTypeRevolute, Math::Vector3d(0.0, 0.0, 1.0));
    l3Id = rbdlPSModel->AddBody(l2Id, l2_l3ST, l2_l3Joint, l3, "l3");   
    //--------------------------------------------------------------------//
    Eigen::Vector3d l3_l4PA = {      0.0, 0.00035, 1.0 };
    Eigen::Vector3d l3_l4CA = { -0.00017,     0.0, 1.0 };
    // Eigen::Vector3d l3_l4PP = {    -0.14,   -0.83, 0.0 };
    Eigen::Vector3d l3_l4PP = {    -0.14+0.287025,   -0.83 , 0.0 };
    Eigen::Vector3d l3_l4CP = {      0.0,     0.0, 0.0 };
    l3_l4PA.normalize();
    l3_l4CA.normalize();

    Eigen::Matrix3d l3_l4_Rot = Eigen::Matrix3d(Eigen::Quaterniond::FromTwoVectors(l3_l4PA, l3_l4CA));
    Eigen::Matrix3d l3_l4_offset = EigenUtilities::rotZ(-1.567);
    l3_l4ST.E = l3_l4_offset * l3_l4_Rot;
    l3_l4ST.r = l3_l4PP - (l3_l4_Rot.inverse() * l3_l4CP);

    l3_l4Joint = Joint(JointTypeRevolute, Math::Vector3d(0.0, 0.0, 1.0));
    l4Id = rbdlPSModel->AddBody(l3Id, l3_l4ST, l3_l4Joint, l4, "l4");  
    //--------------------------------------------------------------------//
    Eigen::Vector3d l1_l4PA = {      0.0, 0.00016,       1.0 };
    Eigen::Vector3d l1_l4CA = {  0.00024,     0.0,       1.0 };
    Eigen::Vector3d l1_l4PP = {     0.07,   -0.77,       0.0 };
    Eigen::Vector3d l1_l4CP = { -0.06239, -0.76135, -0.00018 };
    l1_l4PA.normalize();
    l1_l4CA.normalize();

    Eigen::Matrix3d l1_l4_Rot = Eigen::Matrix3d(Eigen::Quaterniond::FromTwoVectors(l1_l4PA, l1_l4CA));
    Eigen::Matrix3d l1_l4_offset = EigenUtilities::rotZ(-1.567);
    l1_l4ST.E = l1_l4_offset * l1_l4_Rot;
    l1_l4ST.r = l1_l4PP - (l1_l4_Rot.inverse() * l1_l4CP);

    l1_l4Joint = Joint(JointTypeRevolute, Math::Vector3d(0.0, 0.0, 1.0));
    rbdlPSModel->AddBody(l1Id, l1_l4ST, l1_l4Joint, l4); 
    //--------------------------------------------------------------------//

    std::cout << "ROOT_worldST: " << std::endl << ROOT_worldST << std::endl;
    std::cout << "world_l1ST: "   << std::endl << world_l1ST   << std::endl;
    std::cout << "l1_l2ST: "      << std::endl << l1_l2ST      << std::endl;

    Q     = VectorNd::Constant ((size_t) rbdlPSModel->dof_count, 0.);
    QDot  = VectorNd::Constant ((size_t) rbdlPSModel->dof_count, 0.);
    QDDot = VectorNd::Constant ((size_t) rbdlPSModel->dof_count, 0.);
    Tau   = VectorNd::Constant ((size_t) rbdlPSModel->dof_count, 0.);

    JOINT_LIMITS[ "l1" ] = {    0.0f,   0.0f };
    JOINT_LIMITS[ "l2" ] = { -1.047f, 1.047f };
    JOINT_LIMITS[ "l3" ] = { -1.047f, 1.047f };
    JOINT_LIMITS[ "l4" ] = { -1.047f, 1.047f }; 

    // Child : Parent
    reference_hierachy_map[ "ROOT" ]  = "ROOT";
    reference_hierachy_map[ "world" ] = "ROOT";
    reference_hierachy_map[ "l1" ]    = "ROOT";
    reference_hierachy_map[ "l2" ]    = "ROOT";
    reference_hierachy_map[ "l3" ]    = "l2";
    reference_hierachy_map[ "l4" ]    = "l3";

    ClearLogOutput();
  }

  ~ParallelStructure () {
    delete rbdlPSModel;
    clientPtr->cleanUp();
  }

  Model *rbdlPSModel = nullptr;
  AMBFClientPtr clientPtr = nullptr;
  rigidBodyPtr baseHandler = nullptr;
  std::string base_name = "world";
  Eigen::Matrix4d T_0_w;
  std::unordered_map<std::string, std::vector<float>> JOINT_LIMITS;
  std::unordered_map<std::string, std::string> reference_hierachy_map;

  unsigned int worldId, l1Id, l2Id, l3Id, l4Id;
  Body world, l1, l2, l3, l4;
  Joint ROOT_worldJoint, world_l1Joint, l1_l2Joint, l2_l3Joint, l3_l4Joint, l1_l4Joint;
  SpatialTransform ROOT_worldST, world_l1ST, l1_l2ST, l2_l3ST, l3_l4ST, l1_l4ST;

  VectorNd Q;
  VectorNd QDot;
  VectorNd QDDot;
  VectorNd Tau;
};