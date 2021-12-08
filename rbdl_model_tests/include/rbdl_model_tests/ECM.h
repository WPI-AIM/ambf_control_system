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

    const tf::Quaternion quat_0_w_tf = baseHandler->get_rot();
    const tf::Vector3 P_0_w_tf = baseHandler->get_pos();

    RigidBodyDynamics::Math::Quaternion quat_0_w;
    quat_0_w(0) = quat_0_w_tf[0];
    quat_0_w(1) = quat_0_w_tf[1];
    quat_0_w(2) = quat_0_w_tf[2];
    quat_0_w(3) = quat_0_w_tf[3];

    const RigidBodyDynamics::Math::Matrix3d R_0_w = quat_0_w.toMatrix();

    RigidBodyDynamics::Math::Vector3d P_0_w;
    P_0_w.setZero();
    
    P_0_w(0) = P_0_w_tf[0];
    P_0_w(1) = P_0_w_tf[1];
    P_0_w(2) = P_0_w_tf[2];

    T_0_w = EigenUtilities::get_frame<Eigen::Matrix3d, Eigen::Vector3d, Eigen::Matrix4d>(R_0_w, P_0_w);

    rbdlECMModel = new Model;

    rbdlECMModel->gravity = RigidBodyDynamics::Math::Vector3d(0., 0., -9.81);

    // mass, com - inertia offset, inertia
    baseLink = Body(1., RigidBodyDynamics::Math::Vector3d (-0.0046, 0., 0.0801), 
           RigidBodyDynamics::Math::Vector3d (0., 0., 0.));
    yawLink = Body (6.417, RigidBodyDynamics::Math::Vector3d (0.0, -0.0161, 0.1345), 
                  RigidBodyDynamics::Math::Vector3d (0.2978, 0.3125, 0.0449));
    pitchBackLink = Body (0.421, RigidBodyDynamics::Math::Vector3d (-0.0515, -0.1434, -0.009), 
                  RigidBodyDynamics::Math::Vector3d (0.0236, 0.0028, 0.0261));
    pitchBottomLink = Body (0.359, RigidBodyDynamics::Math::Vector3d (0.1491, -0.0182, 0.0), 
              RigidBodyDynamics::Math::Vector3d (0.0007, 0.019, 0.0192));
    mainInsertionLink = Body (0.231, RigidBodyDynamics::Math::Vector3d (-0.059, -0.0165, 0.0008), 
                  RigidBodyDynamics::Math::Vector3d (0.0003, 0.0015, 0.0016));
    toolLink = Body (1.907, RigidBodyDynamics::Math::Vector3d (0., -0.0008, 0.0723), 
                  RigidBodyDynamics::Math::Vector3d (0.0457, 0.0455, 0.0017));
    pitchFrontLink = Body (1.607, RigidBodyDynamics::Math::Vector3d (-0.0365, -0.1526, 0.0), 
              RigidBodyDynamics::Math::Vector3d (0.0983, 0.0175, 0.1099));
    pitchTopLink = Body (0.439, RigidBodyDynamics::Math::Vector3d (0.1702, -0.0001, 0.0008), 
              RigidBodyDynamics::Math::Vector3d (0.0, 0.0381, 0.0381));
    //--------------------------------------------------------------------//
    ROOT_baseLinkJoint = Joint(JointTypeFixed);
    ROOT_baseLinkST.E = RigidBodyDynamics::Math::Matrix3dIdentity;
    ROOT_baseLinkST.r = RigidBodyDynamics::Math::Vector3dZero;
    baseLinkId = rbdlECMModel->AddBody(0, ROOT_baseLinkST, ROOT_baseLinkJoint, baseLink, "baselink");

    // No Joint for pittchEndLink as its a p2p joint
    pitchEndLink = Body (2.032, RigidBodyDynamics::Math::Vector3d (0.0513, 0.0048, 0.0008), 
          RigidBodyDynamics::Math::Vector3d (0.0636, 0.0099, 0.0726));
    //--------------------------------------------------------------------//
    //baseLink_pitchEndLink = Joint(JointType3DoF);
    // baseLink_pitchEndLinkST.E = 
    // eigenUtilities.rotationMatrixFromVectors(Eigen::Vector3d(0.0227, 0.0239, 0.9995), 
    //                                         Eigen::Vector3d(0.0229, -0.9995, -0.0227));
    // baseLink_pitchEndLinkST.r = RigidBodyDynamics::Math::Vector3d(0.0001, 0.0, 0.0);
    // pitchEndLinkId = rbdlECMModel->AddBody(baseLinkId, baseLink_pitchEndLinkST, 
    //                                     baseLink_pitchEndLink, pitchEndLink, "pitchEndLink");    
    //--------------------------------------------------------------------//
    Eigen::Vector3d baseLink_yawLinkPA = { -0.0002, -1.0,    0.0 };
    Eigen::Vector3d baseLink_yawLinkCA = {     0.0,  0.0,   -1.0 };
    Eigen::Vector3d baseLink_yawLinkPP = {     0.0,  0.0,    0.0 };
    Eigen::Vector3d baseLink_yawLinkCP = {  0.0001,  0.0, 0.5369 };
    baseLink_yawLinkPA.normalize();
    baseLink_yawLinkCA.normalize();

    Eigen::Matrix3d baseLink_yawLinkRot = 
          Eigen::Matrix3d(Eigen::Quaterniond::FromTwoVectors(baseLink_yawLinkPA, baseLink_yawLinkCA));
    Eigen::Matrix3d baseLink_yawLink_offset = EigenUtilities::rotZ(-3.1414);

    baseLink_yawLinkST.E = baseLink_yawLink_offset * baseLink_yawLinkRot;
    baseLink_yawLinkST.r = baseLink_yawLinkPP - 
                          (baseLink_yawLinkRot.inverse() * baseLink_yawLinkCP);

    baseLink_yawLinkJoint = Joint(JointTypeRevolute, Math::Vector3d(0.0, 0.0, 1.0));
    yawLinkId = rbdlECMModel->AddBody(baseLinkId, baseLink_yawLinkST, 
                                      baseLink_yawLinkJoint, yawLink, "yawlink");
    //--------------------------------------------------------------------//
    Eigen::Vector3d yawLink_pitchBackLinkPA = { 1.0,     0.0,    0.0 };
    Eigen::Vector3d yawLink_pitchBackLinkCA = { 0.0,     0.0,    1.0 };
    Eigen::Vector3d yawLink_pitchBackLinkPP = { 0.0, -0.0098, 0.1624 };
    Eigen::Vector3d yawLink_pitchBackLinkCP = { 0.0,     0.0,    0.0 };
    yawLink_pitchBackLinkPA.normalize();
    yawLink_pitchBackLinkCA.normalize();

    Eigen::Matrix3d yawLink_pitchBackLinkRot = 
          Eigen::Matrix3d(Eigen::Quaterniond::FromTwoVectors(yawLink_pitchBackLinkPA, yawLink_pitchBackLinkCA));
    Eigen::Matrix3d yawLink_pitchBackLink_offset = EigenUtilities::rotZ(3.1416);

    yawLink_pitchBackLinkST.E = yawLink_pitchBackLink_offset * yawLink_pitchBackLinkRot;
    yawLink_pitchBackLinkST.r = yawLink_pitchBackLinkPP - 
                          (yawLink_pitchBackLinkRot.inverse() * yawLink_pitchBackLinkCP);

    yawLink_pitchBackLinkJoint = Joint(JointTypeRevolute, Math::Vector3d(0.0, 0.0, 1.0));
    pitchBackLinkId = rbdlECMModel->AddBody(yawLinkId, yawLink_pitchBackLinkST, 
                                      yawLink_pitchBackLinkJoint, yawLink, "pitchbacklink");
    //--------------------------------------------------------------------//
    Eigen::Vector3d pitchBackLink_pitchBottomLinkPA = {     0.0,     0.0,     1.0 };
    Eigen::Vector3d pitchBackLink_pitchBottomLinkCA = {     0.0,     0.0,     1.0 };
    Eigen::Vector3d pitchBackLink_pitchBottomLinkPP = { -0.1028, -0.2867,     0.0 };
    Eigen::Vector3d pitchBackLink_pitchBottomLinkCP = { -0.0364,  0.0098, -0.0005 };
    pitchBackLink_pitchBottomLinkPA.normalize();
    pitchBackLink_pitchBottomLinkCA.normalize();

    Eigen::Matrix3d pitchBackLink_pitchBottomLinkRot = 
          Eigen::Matrix3d(Eigen::Quaterniond::FromTwoVectors(pitchBackLink_pitchBottomLinkPA, pitchBackLink_pitchBottomLinkCA));
    Eigen::Matrix3d pitchBackLink_pitchBottomLink_offset = RigidBodyDynamics::Math::Matrix3dIdentity;

    pitchBackLink_pitchBottomLinkST.E = pitchBackLink_pitchBottomLink_offset * pitchBackLink_pitchBottomLinkRot;
    pitchBackLink_pitchBottomLinkST.r = pitchBackLink_pitchBottomLinkPP - 
                          (pitchBackLink_pitchBottomLinkRot.inverse() * pitchBackLink_pitchBottomLinkCP);

    pitchBackLink_pitchBottomLinkJoint = Joint(JointTypeRevolute, Math::Vector3d(0.0, 0.0, 1.0));
    pitchBottomLinkId = rbdlECMModel->AddBody(pitchBackLinkId, pitchBackLink_pitchBottomLinkST, 
                                      pitchBackLink_pitchBottomLinkJoint, yawLink, "pitchbottomlink");
    //--------------------------------------------------------------------//
    Eigen::Vector3d pitchBottomLink_pitchEndLinkPA = {    0.0,     0.0,     1.0 };
    Eigen::Vector3d pitchBottomLink_pitchEndLinkCA = {    0.0,     0.0,     1.0 };
    Eigen::Vector3d pitchBottomLink_pitchEndLinkPP = { 0.3401, -0.0001, -0.0005 };
    Eigen::Vector3d pitchBottomLink_pitchEndLinkCP = {    0.0,     0.0,  0.0001 };
    pitchBottomLink_pitchEndLinkPA.normalize();
    pitchBottomLink_pitchEndLinkCA.normalize();

    pitchBottomLink_pitchEndLinkST.E = Eigen::Matrix3d(Eigen::Quaterniond::FromTwoVectors(
                pitchBottomLink_pitchEndLinkPA, pitchBottomLink_pitchEndLinkCA));

    pitchBottomLink_pitchEndLinkST.r = pitchBottomLink_pitchEndLinkPP -
                    pitchBottomLink_pitchEndLinkST.E.inverse() * pitchBottomLink_pitchEndLinkCP;

    pitchBottomLink_pitchEndLinkJoint = Joint(JointTypeRevolute, Math::Vector3d(0.0, 0.0, 1.0));
    pitchEndLinkId = rbdlECMModel->AddBody(pitchBottomLinkId, pitchBottomLink_pitchEndLinkST, 
                                      pitchBottomLink_pitchEndLinkJoint, pitchEndLink, 
                                      "pitchendlink");
    //--------------------------------------------------------------------//
    Eigen::Vector3d pitchEndLink_mainInsertionLinkPA = {     0.0,     1.0,    0.0 };
    Eigen::Vector3d pitchEndLink_mainInsertionLinkCA = {     1.0,     0.0,    0.0 };
    Eigen::Vector3d pitchEndLink_mainInsertionLinkPP = {  0.1031, -0.0961, 0.0001 };
    Eigen::Vector3d pitchEndLink_mainInsertionLinkCP = { -0.0108,  -0.062, 0.0002 };
    pitchEndLink_mainInsertionLinkPA.normalize();
    pitchEndLink_mainInsertionLinkCA.normalize();

    Eigen::Matrix3d pitchEndLink_mainInsertionLinkRot = 
          Eigen::Matrix3d(Eigen::Quaterniond::FromTwoVectors(pitchEndLink_mainInsertionLinkPA, pitchEndLink_mainInsertionLinkCA));
    Eigen::Matrix3d pitchEndLink_mainInsertionLink_offset = RigidBodyDynamics::Math::Matrix3dIdentity;

    pitchEndLink_mainInsertionLinkST.E = pitchEndLink_mainInsertionLink_offset * pitchEndLink_mainInsertionLinkRot;
    pitchEndLink_mainInsertionLinkST.r = pitchEndLink_mainInsertionLinkPP - 
                          (pitchEndLink_mainInsertionLinkRot.inverse() * pitchEndLink_mainInsertionLinkCP);

    pitchEndLink_mainInsertionLinkJoint = Joint(JointTypePrismatic, Math::Vector3d(0.0, 0.0, 1.0));
    mainInsertionLinkId = rbdlECMModel->AddBody(pitchEndLinkId, pitchEndLink_mainInsertionLinkST, 
                                      pitchEndLink_mainInsertionLinkJoint, mainInsertionLink, 
                                      "maininsertionlink");
    //--------------------------------------------------------------------//
    // Rotation fails
    // Eigen::Vector3d mainInsertionLink_toolLinkPA = {     1.0,     0.0,    0.0 };
    // Eigen::Vector3d mainInsertionLink_toolLinkCA = {     0.0,     0.0,   -1.0 };
    // Eigen::Vector3d mainInsertionLink_toolLinkPP = { -0.0108,  -0.062,    0.0 };
    // Eigen::Vector3d mainInsertionLink_toolLinkCP = { -0.0001, -0.0002, 0.0118 };
    // mainInsertionLink_toolLinkPA.normalize();
    // mainInsertionLink_toolLinkCA.normalize();

    // Eigen::Matrix3d mainInsertionLink_toolLinkRot = 
    //       Eigen::Matrix3d(Eigen::Quaterniond::FromTwoVectors(mainInsertionLink_toolLinkPA, mainInsertionLink_toolLinkCA));
    // Eigen::Matrix3d mainInsertionLink_toolLink_offset = EigenUtilities::rotZ(-1.5708);

    // mainInsertionLink_toolLinkST.E = mainInsertionLink_toolLink_offset * mainInsertionLink_toolLinkRot;
    // mainInsertionLink_toolLinkST.r = mainInsertionLink_toolLinkPP - 
    //                       (mainInsertionLink_toolLinkRot.inverse() * mainInsertionLink_toolLinkCP);

    // mainInsertionLink_toolLinkJoint = Joint(JointTypeRevolute, Math::Vector3d(0.0, 0.0, 1.0));
    // toolLinkId = rbdlECMModel->AddBody(mainInsertionLinkId, mainInsertionLink_toolLinkST, 
    //                                   mainInsertionLink_toolLinkJoint, toolLink, "toollink");
    //--------------------------------------------------------------------//
    Eigen::Vector3d yawLink_pitchFrontLinkPA = { 1.0, 0.0,   0.0 };
    Eigen::Vector3d yawLink_pitchFrontLinkCA = { 0.0, 0.0,   1.0 };
    Eigen::Vector3d yawLink_pitchFrontLinkPP = { 0.0, 0.0, 0.199 };
    Eigen::Vector3d yawLink_pitchFrontLinkCP = { 0.0, 0.0,   0.0 };
    yawLink_pitchFrontLinkPA.normalize();
    yawLink_pitchFrontLinkCA.normalize();

    Eigen::Matrix3d yawLink_pitchFrontLinkRot = 
          Eigen::Matrix3d(Eigen::Quaterniond::FromTwoVectors(yawLink_pitchFrontLinkPA, yawLink_pitchFrontLinkCA));
    Eigen::Matrix3d yawLink_pitchFrontLink_offset = EigenUtilities::rotZ(3.1416);

    yawLink_pitchFrontLinkST.E = yawLink_pitchFrontLink_offset * yawLink_pitchFrontLinkRot;
    yawLink_pitchFrontLinkST.r = yawLink_pitchFrontLinkPP - 
                          (yawLink_pitchFrontLinkRot.inverse() * yawLink_pitchFrontLinkCP);

    yawLink_pitchFrontLinkJoint = Joint(JointTypeRevolute, Math::Vector3d(0.0, 0.0, 1.0));
    pitchFrontLinkId = rbdlECMModel->AddBody(yawLinkId, yawLink_pitchFrontLinkST, 
                                      yawLink_pitchFrontLinkJoint, pitchFrontLink, 
                                      "pitchfrontlink");
    //--------------------------------------------------------------------//
    Eigen::Vector3d  pitchFrontLink_pitchBottomLinkPA = {    0.0,      0.0,     1.0 };
    Eigen::Vector3d  pitchFrontLink_pitchBottomLinkCA = {    0.0,      0.0,     1.0 };
    Eigen::Vector3d  pitchFrontLink_pitchBottomLinkPP = { -0.1031, -0.2868,     0.0 };
    Eigen::Vector3d  pitchFrontLink_pitchBottomLinkCP = { -0.0001, -0.0001, -0.0005 };
    pitchFrontLink_pitchBottomLinkPA.normalize();
    pitchFrontLink_pitchBottomLinkCA.normalize();

    Eigen::Matrix3d pitchFrontLink_pitchBottomLinkRot = 
          Eigen::Matrix3d(Eigen::Quaterniond::FromTwoVectors(pitchFrontLink_pitchBottomLinkPA, pitchFrontLink_pitchBottomLinkCA));
    Eigen::Matrix3d pitchFrontLink_pitchBottomLink_offset = RigidBodyDynamics::Math::Matrix3dIdentity;

    pitchFrontLink_pitchBottomLinkST.E = pitchFrontLink_pitchBottomLink_offset * pitchFrontLink_pitchBottomLinkRot;
    pitchFrontLink_pitchBottomLinkST.r = pitchFrontLink_pitchBottomLinkPP - 
                          (pitchFrontLink_pitchBottomLinkRot.inverse() * pitchFrontLink_pitchBottomLinkCP);

    pitchFrontLink_pitchBottomLinkJoint = Joint(JointTypeRevolute, Math::Vector3d(0.0, 0.0, 1.0));
    rbdlECMModel->AddBody(pitchFrontLinkId, pitchFrontLink_pitchBottomLinkST, 
                                      pitchFrontLink_pitchBottomLinkJoint, pitchBottomLink);
    //--------------------------------------------------------------------//
    Eigen::Vector3d pitchFrontLink_pitchTopLinkPA = {     0.0,     0.0,     1.0 };
    Eigen::Vector3d pitchFrontLink_pitchTopLinkCA = {     0.0,     0.0,     1.0 };
    Eigen::Vector3d pitchFrontLink_pitchTopLinkPP = { -0.1084, -0.3242,     0.0 };
    Eigen::Vector3d pitchFrontLink_pitchTopLinkCP = {     0.0,     0.0, -0.0006 };
    pitchFrontLink_pitchTopLinkPA.normalize();
    pitchFrontLink_pitchTopLinkCA.normalize();

    Eigen::Matrix3d pitchFrontLink_pitchTopLinkRot = 
          Eigen::Matrix3d(Eigen::Quaterniond::FromTwoVectors(pitchFrontLink_pitchTopLinkPA, pitchFrontLink_pitchTopLinkCA));
    Eigen::Matrix3d pitchFrontLink_pitchTopLink_offset = RigidBodyDynamics::Math::Matrix3dIdentity;

    pitchFrontLink_pitchTopLinkST.E = pitchFrontLink_pitchTopLink_offset * pitchFrontLink_pitchTopLinkRot;
    pitchFrontLink_pitchTopLinkST.r = pitchFrontLink_pitchTopLinkPP - 
                          (pitchFrontLink_pitchTopLinkRot.inverse() * pitchFrontLink_pitchTopLinkCP);

    pitchFrontLink_pitchTopLinkJoint = Joint(JointTypeRevolute, Math::Vector3d(0.0, 0.0, 1.0));
    pitchTopLinkId = rbdlECMModel->AddBody(pitchFrontLinkId, pitchFrontLink_pitchTopLinkST, 
                                      pitchFrontLink_pitchTopLinkJoint, pitchTopLink, 
                                      "pitchtoplink");
    //--------------------------------------------------------------------//
    Eigen::Vector3d pitchTopLink_pitchEndLinkPA = {     0.0,     0.0,     1.0 };
    Eigen::Vector3d pitchTopLink_pitchEndLinkCA = {     0.0,     0.0,     1.0 };
    Eigen::Vector3d pitchTopLink_pitchEndLinkPP = {  0.3404, -0.0002, -0.0006 };
    Eigen::Vector3d pitchTopLink_pitchEndLinkCP = { -0.0051, -0.0376,  0.0001 };
    pitchTopLink_pitchEndLinkPA.normalize();
    pitchTopLink_pitchEndLinkCA.normalize();

    Eigen::Matrix3d pitchTopLink_pitchEndLinkRot = 
          Eigen::Matrix3d(Eigen::Quaterniond::FromTwoVectors(pitchTopLink_pitchEndLinkPA, pitchTopLink_pitchEndLinkCA));
    Eigen::Matrix3d pitchTopLink_pitchEndLink_offset = RigidBodyDynamics::Math::Matrix3dIdentity;

    pitchTopLink_pitchEndLinkST.E = pitchTopLink_pitchEndLink_offset * pitchTopLink_pitchEndLinkRot;
    pitchTopLink_pitchEndLinkST.r = pitchTopLink_pitchEndLinkPP - 
                          (pitchTopLink_pitchEndLinkRot.inverse() * pitchTopLink_pitchEndLinkCP);

    pitchTopLink_pitchEndLinkJoint = Joint(JointTypeRevolute, Math::Vector3d(0.0, 0.0, 1.0));
    rbdlECMModel->AddBody(pitchTopLinkId, pitchTopLink_pitchEndLinkST, 
                                      pitchTopLink_pitchEndLinkJoint, pitchEndLink);
    //--------------------------------------------------------------------//
    // Rotation fails
    Eigen::Vector3d mainInsertionLink_toolLinkPA = {     1.0,     0.0,    0.0 };
    Eigen::Vector3d mainInsertionLink_toolLinkCA = {     0.0,     0.0,   -1.0 };
    Eigen::Vector3d mainInsertionLink_toolLinkPP = { -0.0108,  -0.062,    0.0 };
    Eigen::Vector3d mainInsertionLink_toolLinkCP = { -0.0001, -0.0002, 0.0118 };
    mainInsertionLink_toolLinkPA.normalize();
    mainInsertionLink_toolLinkCA.normalize();

    Eigen::Matrix3d mainInsertionLink_toolLinkRot = 
          Eigen::Matrix3d(Eigen::Quaterniond::FromTwoVectors(mainInsertionLink_toolLinkPA, mainInsertionLink_toolLinkCA));
    Eigen::Matrix3d mainInsertionLink_toolLink_offset = EigenUtilities::rotZ(-1.5708);

    mainInsertionLink_toolLinkST.E = mainInsertionLink_toolLink_offset * mainInsertionLink_toolLinkRot;
    mainInsertionLink_toolLinkST.r = mainInsertionLink_toolLinkPP - 
                          (mainInsertionLink_toolLinkRot.inverse() * mainInsertionLink_toolLinkCP);

    mainInsertionLink_toolLinkJoint = Joint(JointTypeRevolute, Math::Vector3d(0.0, 0.0, 1.0));
    toolLinkId = rbdlECMModel->AddBody(mainInsertionLinkId, mainInsertionLink_toolLinkST, 
                                      mainInsertionLink_toolLinkJoint, toolLink, "toollink");
    //--------------------------------------------------------------------//
    
    Q = VectorNd::Constant ((size_t) rbdlECMModel->dof_count, 0.);
    QDot = VectorNd::Constant ((size_t) rbdlECMModel->dof_count, 0.);
    QDDot = VectorNd::Constant ((size_t) rbdlECMModel->dof_count, 0.);
    Tau = VectorNd::Constant ((size_t) rbdlECMModel->dof_count, 0.);

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
    delete rbdlECMModel;
    clientPtr->cleanUp();
  }

  Model *rbdlECMModel = nullptr;
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