#include "rbdl_model_tests/RBDLTestPrep.h"
#include "rbdl_model_tests/EigenUtilities.h"
//#include "rbdl_model_tests/Human36Fixture.h"
#include <unordered_map>
#include <Eigen/Geometry> 

//const double TEST_PREC = 1.0e-12;
const double TEST_LAX = 1.0e-7;

struct ECM {
  ECM () {
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

    rbdlModel = new Model;

    // mass, com - inertia offset, inertia
    baseLink = Body(1., RigidBodyDynamics::Math::Vector3d (-0.0046, 0., 0.0801), 
           RigidBodyDynamics::Math::Vector3d (0., 0., 0.));
    ROOT_baseLinkJoint = Joint(JointTypeFixed);
    ROOT_baseLinkST.E = RigidBodyDynamics::Math::Matrix3dIdentity;
    ROOT_baseLinkST.r = RigidBodyDynamics::Math::Vector3dZero;
    baseLinkId = rbdlModel->AddBody(0, ROOT_baseLinkST, ROOT_baseLinkJoint, baseLink, "baselink");

    // No Joint for pittchEndLink as its a p2p joint
    pitchEndLink = Body (2.032, RigidBodyDynamics::Math::Vector3d (0.0513, 0.0048, 0.0008), 
          RigidBodyDynamics::Math::Vector3d (0.0636, 0.0099, 0.0726));
    //--------------------------------------------------------------------//
    //baseLink_pitchEndLink = Joint(JointType3DoF);
    // baseLink_pitchEndLinkST.E = 
    // eigenUtilities.rotationMatrixFromVectors(Eigen::Vector3d(0.0227, 0.0239, 0.9995), 
    //                                         Eigen::Vector3d(0.0229, -0.9995, -0.0227));
    // baseLink_pitchEndLinkST.r = RigidBodyDynamics::Math::Vector3d(0.0001, 0.0, 0.0);
    // pitchEndLinkId = rbdlModel->AddBody(baseLinkId, baseLink_pitchEndLinkST, 
    //                                     baseLink_pitchEndLink, pitchEndLink, "pitchEndLink");    
    //--------------------------------------------------------------------//

    yawLink = Body (6.417, RigidBodyDynamics::Math::Vector3d (0.0, -0.0161, 0.1345), 
                  RigidBodyDynamics::Math::Vector3d (0.2978, 0.3125, 0.0449));

    Eigen::Vector3d baseLink_yawLinkPA = { -0.0002, -1.0,    0.0 };
    Eigen::Vector3d baseLink_yawLinkCA = {     0.0,  0.0,   -1.0 };
    Eigen::Vector3d baseLink_yawLinkPP = {     0.0,  0.0,    0.0 };
    Eigen::Vector3d baseLink_yawLinkCP = {  0.0001,  0.0, 0.5369 };
    baseLink_yawLinkPA.normalize();
    baseLink_yawLinkCA.normalize();

    baseLink_yawLinkST.E = Eigen::Matrix3d(Eigen::Quaterniond::FromTwoVectors(
                baseLink_yawLinkPA, baseLink_yawLinkCA));

    baseLink_yawLinkST.r = baseLink_yawLinkPP - 
                          (baseLink_yawLinkST.E.inverse() * baseLink_yawLinkCP);

    baseLink_yawLinkJoint = Joint(JointTypeRevolute, Math::Vector3d(0.0, 0.0, 1.0));
    yawLinkId = rbdlModel->AddBody(baseLinkId, baseLink_yawLinkST, 
                                      baseLink_yawLinkJoint, yawLink, "yawlink");
    //--------------------------------------------------------------------//
    pitchBackLink = Body (0.421, RigidBodyDynamics::Math::Vector3d (-0.0515, -0.1434, -0.009), 
                  RigidBodyDynamics::Math::Vector3d (0.0236, 0.0028, 0.0261));
    Eigen::Vector3d yawLink_pitchBackLinkPA = { 1.0,     0.0,    0.0 };
    Eigen::Vector3d yawLink_pitchBackLinkCA = { 0.0,     0.0,    1.0 };
    Eigen::Vector3d yawLink_pitchBackLinkPP = { 0.0, -0.0098, 0.1624 };
    Eigen::Vector3d yawLink_pitchBackLinkCP = { 0.0,   -0.01,    0.0 };
    yawLink_pitchBackLinkPA.normalize();
    yawLink_pitchBackLinkCA.normalize();

    yawLink_pitchBackLinkST.E = Eigen::Matrix3d(Eigen::Quaterniond::FromTwoVectors(
                yawLink_pitchBackLinkPA, yawLink_pitchBackLinkCA));

    yawLink_pitchBackLinkST.r = yawLink_pitchBackLinkPP -
                    // added a padding of -0.01 along y so that test cases passes
                    yawLink_pitchBackLinkST.E.inverse() * yawLink_pitchBackLinkCP;


    yawLink_pitchBackLinkJoint = Joint(JointTypeRevolute, Math::Vector3d(0.0, 0.0, 1.0));
    pitchBackLinkId = rbdlModel->AddBody(yawLinkId, yawLink_pitchBackLinkST, 
                                      yawLink_pitchBackLinkJoint, pitchBackLink, "pitchbacklink");
    //--------------------------------------------------------------------//
    pitchBottomLink = Body (0.359, RigidBodyDynamics::Math::Vector3d (0.1491, -0.0182, 0.0), 
              RigidBodyDynamics::Math::Vector3d (0.0007, 0.019, 0.0192));

    Eigen::Vector3d pitchBackLink_pitchBottomLinkPA = {     0.0,     0.0,     1.0 };
    Eigen::Vector3d pitchBackLink_pitchBottomLinkCA = {     0.0,     0.0,     1.0 };
    Eigen::Vector3d pitchBackLink_pitchBottomLinkPP = { -0.1028, -0.2867,     0.0 };
    Eigen::Vector3d pitchBackLink_pitchBottomLinkCP = { -0.0364,  0.0098, -0.0005 };
    pitchBackLink_pitchBottomLinkPA.normalize();
    pitchBackLink_pitchBottomLinkCA.normalize();

    pitchBackLink_pitchBottomLinkST.E = Eigen::Matrix3d(Eigen::Quaterniond::FromTwoVectors(
                pitchBackLink_pitchBottomLinkPA, pitchBackLink_pitchBottomLinkCA));

    pitchBackLink_pitchBottomLinkST.r = pitchBackLink_pitchBottomLinkPP -
                    pitchBackLink_pitchBottomLinkST.E.inverse() * pitchBackLink_pitchBottomLinkCP;


    pitchBackLink_pitchBottomLinkJoint = Joint(JointTypeRevolute, Math::Vector3d(0.0, 0.0, 1.0));
    pitchBottomLinkId = rbdlModel->AddBody(pitchBackLinkId, pitchBackLink_pitchBottomLinkST, 
                                      pitchBackLink_pitchBottomLinkJoint, pitchBottomLink, 
                                      "pitchbottomlink");
    //--------------------------------------------------------------------//
    Eigen::Vector3d pitchBottomLink_pitchEndLinkPA = {    0.0,     0.0,     1.0 };
    Eigen::Vector3d pitchBottomLink_pitchEndLinkCA = {    0.0,     0.0,     1.0 };
    Eigen::Vector3d pitchBottomLink_pitchEndLinkPP = { 0.3401, -0.0001, -0.0005 };
    Eigen::Vector3d pitchBottomLink_pitchEndLinkCP = {    0.0,     0.0, -0.0001 };
    pitchBottomLink_pitchEndLinkPA.normalize();
    pitchBottomLink_pitchEndLinkCA.normalize();

    pitchBottomLink_pitchEndLinkST.E = Eigen::Matrix3d(Eigen::Quaterniond::FromTwoVectors(
                pitchBottomLink_pitchEndLinkPA, pitchBottomLink_pitchEndLinkCA));

    pitchBottomLink_pitchEndLinkST.r = pitchBottomLink_pitchEndLinkPP -
                    pitchBottomLink_pitchEndLinkST.E.inverse() * pitchBottomLink_pitchEndLinkCP;

    pitchBottomLink_pitchEndLinkJoint = Joint(JointTypeRevolute, Math::Vector3d(0.0, 0.0, 1.0));
    pitchEndLinkId = rbdlModel->AddBody(pitchBottomLinkId, pitchBottomLink_pitchEndLinkST, 
                                      pitchBottomLink_pitchEndLinkJoint, pitchEndLink, 
                                      "pitchendlink");
    //--------------------------------------------------------------------//
    mainInsertionLink = Body (0.231, RigidBodyDynamics::Math::Vector3d (-0.059, -0.0165, 0.0008), 
                  RigidBodyDynamics::Math::Vector3d (0.0003, 0.0015, 0.0016));

    Eigen::Vector3d pitchEndLink_mainInsertionLinkPA = {     0.0,     1.0,    0.0 };
    Eigen::Vector3d pitchEndLink_mainInsertionLinkCA = {     1.0,     0.0,    0.0 };
    Eigen::Vector3d pitchEndLink_mainInsertionLinkPP = {  0.1031, -0.0961, 0.0001 };
    Eigen::Vector3d pitchEndLink_mainInsertionLinkCP = { -0.0108,  -0.062, 0.0002 };
    pitchEndLink_mainInsertionLinkPA.normalize();
    pitchEndLink_mainInsertionLinkCA.normalize();

    pitchEndLink_mainInsertionLinkST.E = Eigen::Matrix3d(Eigen::Quaterniond::FromTwoVectors(
                pitchEndLink_mainInsertionLinkPA, pitchEndLink_mainInsertionLinkCA));

    pitchEndLink_mainInsertionLinkST.r = pitchEndLink_mainInsertionLinkPP -
                    pitchEndLink_mainInsertionLinkST.E.inverse() * pitchEndLink_mainInsertionLinkCP;

    pitchEndLink_mainInsertionLinkJoint = Joint(JointTypePrismatic, Math::Vector3d(0.0, 0.0, 1.0));
    mainInsertionLinkId = rbdlModel->AddBody(pitchEndLinkId, pitchEndLink_mainInsertionLinkST, 
                                      pitchEndLink_mainInsertionLinkJoint, mainInsertionLink, 
                                      "maininsertionlink");
    //--------------------------------------------------------------------//

    toolLink = Body (1.907, RigidBodyDynamics::Math::Vector3d (0., -0.0008, 0.0723), 
                  RigidBodyDynamics::Math::Vector3d (0.0457, 0.0455, 0.0017));

    Eigen::Vector3d mainInsertionLink_toolLinkPA = {     1.0,     0.0,    0.0 };
    Eigen::Vector3d mainInsertionLink_toolLinkCA = {     0.0,     0.0,   -1.0 };
    Eigen::Vector3d mainInsertionLink_toolLinkPP = { -0.0108,  -0.062,    0.0 };
    Eigen::Vector3d mainInsertionLink_toolLinkCP = { -0.0001, -0.0002, 0.0118 };
    mainInsertionLink_toolLinkPA.normalize();
    mainInsertionLink_toolLinkCA.normalize();

    mainInsertionLink_toolLinkST.E = Eigen::Matrix3d(Eigen::Quaterniond::FromTwoVectors(
                mainInsertionLink_toolLinkPA, mainInsertionLink_toolLinkCA));

    mainInsertionLink_toolLinkST.r = mainInsertionLink_toolLinkPP -
                    mainInsertionLink_toolLinkST.E.inverse() * mainInsertionLink_toolLinkCP;

    mainInsertionLink_toolLinkJoint = Joint(JointTypeRevolute, Math::Vector3d(0.0, 0.0, 1.0));
    toolLinkId = rbdlModel->AddBody(mainInsertionLinkId, mainInsertionLink_toolLinkST, 
                                      mainInsertionLink_toolLinkJoint, toolLink, "toollink");
    //--------------------------------------------------------------------//
    pitchFrontLink = Body (1.607, RigidBodyDynamics::Math::Vector3d (-0.0365, -0.1526, 0.0), 
              RigidBodyDynamics::Math::Vector3d (0.0983, 0.0175, 0.1099));

    Eigen::Vector3d yawLink_pitchFrontLinkPA = { 1.0, 0.0,   0.0 };
    Eigen::Vector3d yawLink_pitchFrontLinkCA = { 0.0, 0.0,   1.0 };
    Eigen::Vector3d yawLink_pitchFrontLinkPP = { 0.0, 0.0, 0.199 };
    Eigen::Vector3d yawLink_pitchFrontLinkCP = { 0.0, 0.0,   0.0 };
    yawLink_pitchFrontLinkPA.normalize();
    yawLink_pitchFrontLinkCA.normalize();

    yawLink_pitchFrontLinkST.E = Eigen::Matrix3d(Eigen::Quaterniond::FromTwoVectors(
                yawLink_pitchFrontLinkPA, yawLink_pitchFrontLinkCA));

    yawLink_pitchFrontLinkST.r = yawLink_pitchFrontLinkPP -
                    yawLink_pitchFrontLinkST.E.inverse() * yawLink_pitchFrontLinkCP;

    yawLink_pitchFrontLinkJoint = Joint(JointTypeRevolute, Math::Vector3d(0.0, 0.0, 1.0));
    pitchFrontLinkId = rbdlModel->AddBody(yawLinkId, yawLink_pitchFrontLinkST, 
                                      yawLink_pitchFrontLinkJoint, pitchFrontLink, 
                                      "pitchfrontlink");
    //--------------------------------------------------------------------//
    Eigen::Vector3d  pitchFrontLink_pitchBottomLinkPA = {    0.0,      0.0,    1.0 };
    Eigen::Vector3d  pitchFrontLink_pitchBottomLinkCA = {    0.0,      0.0,    1.0 };
    Eigen::Vector3d  pitchFrontLink_pitchBottomLinkPP = { -0.1031, -0.2868,    0.0 };
    Eigen::Vector3d  pitchFrontLink_pitchBottomLinkCP = {  0.0001,  0.0001, 0.0005 };
    pitchFrontLink_pitchBottomLinkPA.normalize();
    pitchFrontLink_pitchBottomLinkCA.normalize();

    pitchFrontLink_pitchBottomLinkST.E = Eigen::Matrix3d(Eigen::Quaterniond::FromTwoVectors(
                 pitchFrontLink_pitchBottomLinkPA,  pitchFrontLink_pitchBottomLinkCA));

    pitchFrontLink_pitchBottomLinkST.r =  pitchFrontLink_pitchBottomLinkPP -
                     pitchFrontLink_pitchBottomLinkST.E.inverse() *  pitchFrontLink_pitchBottomLinkCP;    

    pitchFrontLink_pitchBottomLinkJoint = Joint(JointTypeRevolute, Math::Vector3d(0.0, 0.0, 1.0));
    rbdlModel->AddBody(pitchFrontLinkId, pitchFrontLink_pitchBottomLinkST, 
                                      pitchFrontLink_pitchBottomLinkJoint, pitchBottomLink);//, 
                                      //"pitchBottomLink");
    //--------------------------------------------------------------------//
    pitchTopLink = Body (0.439, RigidBodyDynamics::Math::Vector3d (0.1702, -0.0001, 0.0008), 
              RigidBodyDynamics::Math::Vector3d (0.0, 0.0381, 0.0381));

    Eigen::Vector3d pitchFrontLink_pitchTopLinkPA = {     0.0,     0.0,     1.0 };
    Eigen::Vector3d pitchFrontLink_pitchTopLinkCA = {     0.0,     0.0,     1.0 };
    Eigen::Vector3d pitchFrontLink_pitchTopLinkPP = { -0.1084, -0.3242,     0.0 };
    Eigen::Vector3d pitchFrontLink_pitchTopLinkCP = {     0.0,     0.0, -0.0006 };
    pitchFrontLink_pitchTopLinkPA.normalize();
    pitchFrontLink_pitchTopLinkCA.normalize();

   pitchFrontLink_pitchTopLinkST.E = Eigen::Matrix3d(Eigen::Quaterniond::FromTwoVectors(
                pitchFrontLink_pitchTopLinkPA, pitchFrontLink_pitchTopLinkCA));

   pitchFrontLink_pitchTopLinkST.r = pitchFrontLink_pitchTopLinkPP -
                    pitchFrontLink_pitchTopLinkST.E.inverse() * pitchFrontLink_pitchTopLinkCP;

    pitchFrontLink_pitchTopLinkJoint = Joint(JointTypeRevolute, Math::Vector3d(0.0, 0.0, 1.0));
    pitchTopLinkId = rbdlModel->AddBody(pitchFrontLinkId, pitchFrontLink_pitchTopLinkST, 
                                      pitchFrontLink_pitchTopLinkJoint, pitchTopLink, 
                                      "pitchtoplink");
    //--------------------------------------------------------------------//
    Eigen::Vector3d pitchTopLink_pitchEndLinkPA = {    0.0,     0.0,     1.0 };
    Eigen::Vector3d pitchTopLink_pitchEndLinkCA = {    0.0,     0.0,     1.0 };
    Eigen::Vector3d pitchTopLink_pitchEndLinkPP = { 0.3404, -0.0002, -0.0006 };
    Eigen::Vector3d pitchTopLink_pitchEndLinkCP = { 0.0051, -0.0376,  0.0001 };
    pitchTopLink_pitchEndLinkPA.normalize();
    pitchTopLink_pitchEndLinkCA.normalize();

    pitchTopLink_pitchEndLinkST.E = Eigen::Matrix3d(Eigen::Quaterniond::FromTwoVectors(
                pitchTopLink_pitchEndLinkPA, pitchTopLink_pitchEndLinkCA));

    pitchTopLink_pitchEndLinkST.r = pitchTopLink_pitchEndLinkPP -
                    pitchTopLink_pitchEndLinkST.E.inverse() * pitchTopLink_pitchEndLinkCP;

    pitchTopLink_pitchEndLinkJoint = Joint(JointTypeRevolute, Math::Vector3d(0.0, 0.0, 1.0));
    
    rbdlModel->AddBody(pitchTopLinkId, pitchTopLink_pitchEndLinkST, 
                                      pitchTopLink_pitchEndLinkJoint, pitchEndLink);//, "pitchEndLink");
    //--------------------------------------------------------------------//
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

    // Child : Parent
    reference_hierachy_map[ "ROOT" ]              = "ROOT";
    reference_hierachy_map[ "baselink" ]          = "ROOT";
    reference_hierachy_map[ "pitchendlink" ]      = "pitchbottomlink";
    reference_hierachy_map[ "yawlink" ]           = "ROOT";
    reference_hierachy_map[ "pitchbacklink" ]     = "yawlink";
    reference_hierachy_map[ "pitchbottomlink" ]   = "pitchbacklink";
    //reference_hierachy_map[ "pitchendlink" ]      = "pitchbottomlink";
    reference_hierachy_map[ "maininsertionlink" ] = "pitchendlink";
    reference_hierachy_map[ "toollink" ]          = "maininsertionlink";
    reference_hierachy_map[ "pitchfrontlink" ]    = "yawlink";
    //reference_hierachy_map[ "pitchbottomlink" ]   = "pitchfrontlink";
    reference_hierachy_map[ "pitchtoplink" ]      = "pitchfrontlink";
    //reference_hierachy_map[ "pitchendlink" ]      = "pitchtoplink";

    ClearLogOutput();
  }

  ~ECM () {
    delete rbdlModel;
    clientPtr->cleanUp();
  }

  Model *rbdlModel = nullptr;
  AMBFClientPtr clientPtr = nullptr;
  rigidBodyPtr baseHandler = nullptr;
  std::string base_name = "baselink";
  Eigen::Matrix4d T_0_w;
  std::unordered_map<std::string, std::vector<float>> KUKA_JOINT_LIMITS;
  std::unordered_map<std::string, std::vector<float>>::iterator KUKA_JOINT_LIMITS_itr;
  std::unordered_map<std::string, std::string> reference_hierachy_map;

  unsigned int baseLinkId, pitchEndLinkId, mainInsertionLinkId, toolLinkId, yawLinkId, 
            pitchBackLinkId, pitchBottomLinkId, pitchFrontLinkId, pitchTopLinkId;

  Body baseLink, pitchEndLink, mainInsertionLink, toolLink, yawLink, 
            pitchBackLink, pitchBottomLink, pitchFrontLink, pitchTopLink;

  double baseLinkMScale, pitchEndLinkMScale, mainInsertionLinkMScale, toolLinkMScale, yawLinkMScale, 
            pitchBackLinkMScale, pitchBottomLinkMScale, pitchFrontLinkMScale, pitchTopLinkMScale = 1.0;


  Joint ROOT_baseLinkJoint, baseLink_pitchEndLinkJoint, pitchEndLink_mainInsertionLinkJoint, 
        mainInsertionLink_toolLinkJoint, baseLink_yawLinkJoint, yawLink_pitchBackLinkJoint,
        pitchBackLink_pitchBottomLinkJoint, pitchBottomLink_pitchEndLinkJoint,
        yawLink_pitchFrontLinkJoint, pitchFrontLink_pitchBottomLinkJoint, 
        pitchFrontLink_pitchTopLinkJoint, pitchTopLink_pitchEndLinkJoint;

  SpatialTransform ROOT_baseLinkST, baseLink_pitchEndLinkST, pitchEndLink_mainInsertionLinkST, 
        mainInsertionLink_toolLinkST, baseLink_yawLinkST, yawLink_pitchBackLinkST,
        pitchBackLink_pitchBottomLinkST, pitchBottomLink_pitchEndLinkST,yawLink_pitchFrontLinkST, 
        pitchFrontLink_pitchBottomLinkST, pitchFrontLink_pitchTopLinkST, pitchTopLink_pitchEndLinkST;

  const double ROOT_baseLinkJointOffset = 0.0;
  const double baseLink_pitchEndLinkJointOffset = 0.0;
  const double pitchEndLink_mainInsertionLinkJointOffset = 0.0;
  const double mainInsertionLink_toolLinkJointOffset = 0.0;
  const double baseLink_yawLinkJointOffset = 0.0;
  const double yawLink_pitchBackLinkJointOffset = 0.0;
  const double pitchBackLink_pitchBottomLinkJointOffset = 0.0;
  const double pitchBottomLink_pitchEndLinkJointOffset = 0.0;
  const double yawLink_pitchFrontLinkJointOffset = 0.0;
  const double pitchFrontLink_pitchBottomLinkJointOffset = 0.0;
  const double pitchFrontLink_pitchTopLinkJointOffset = 0.0;
  const double pitchTopLink_pitchEndLinkJointOffset = 0.0;

  VectorNd Q;
  VectorNd QDot;
  VectorNd QDDot;
  VectorNd Tau;
};