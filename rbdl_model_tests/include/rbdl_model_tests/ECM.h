#include "rbdl_model_tests/RBDLTestPrep.h"
#include "rbdl_model_tests/EigenUtilities.h"
#include <unordered_map>
#include <Eigen/Geometry> 

//const double TEST_PREC = 1.0e-12;
const double TEST_LAX = 1.0e-7;
struct JointParams
{
  std::string jointName;
  float jointLowerLimit;
  float jointHigherLimit;
};

struct ControllableBodyParams
{
  rigidBodyPtr rigidBodyHandler;
  std::vector<std::string> childrenJoints;
  std::vector<JointParams> controllableJoints;
};

struct ECM {
  ECM () {
    ClearLogOutput();

    // clientPtr = RBDLTestPrep::getInstance()->getAMBFClientInstance();
    // clientPtr->connect();
    // baseHandler = clientPtr->getRigidBody(base_name, true);
    // usleep(1000000);

    initializeHandlers();
    getBaseTransform();
    
    // //base is rigid body name, not a joint. This is a hacky way to enable ros topics in the 
    // //server side during first execution
    // baseHandler->set_joint_pos(base_name, 0.0f); 

    // const tf::Quaternion quat_w_0_tf = baseHandler->get_rot();
    // const tf::Vector3 P_w_0_tf = baseHandler->get_pos();

    // RigidBodyDynamics::Math::Quaternion quat_w_0;
    // quat_w_0(0) = quat_w_0_tf[0];
    // quat_w_0(1) = quat_w_0_tf[1];
    // quat_w_0(2) = quat_w_0_tf[2];
    // quat_w_0(3) = quat_w_0_tf[3];

    // const RigidBodyDynamics::Math::Matrix3d R_w_0 = quat_w_0.toMatrix();

    // RigidBodyDynamics::Math::Vector3d P_w_0;
    // P_w_0.setZero();
    
    // P_w_0(0) = P_w_0_tf[0];
    // P_w_0(1) = P_w_0_tf[1];
    // P_w_0(2) = P_w_0_tf[2];

    // T_w_0 = EigenUtilities::get_frame<Eigen::Matrix3d, 
    //             Eigen::Vector3d, Eigen::Matrix4d>(R_w_0, P_w_0);
    
    // T_0_w = EigenUtilities::get_frame<Eigen::Matrix3d, 
    //             Eigen::Vector3d, Eigen::Matrix4d>(R_w_0.transpose(), -R_w_0.transpose() * P_w_0);
    
    rbdlModel = new Model;
    rbdlModel->gravity = RigidBodyDynamics::Math::Vector3d(0., 0., -9.81);

    // mass, com - inertia offset, inertia
    baseLinkBody          = Body(1., RigidBodyDynamics::Math::Vector3d (-0.0046, 0., 0.0801), 
                            RigidBodyDynamics::Math::Vector3d (0., 0., 0.));
    yawLinkBody           = Body (6.417, RigidBodyDynamics::Math::Vector3d (0.0, -0.0161, 0.1345), 
                            RigidBodyDynamics::Math::Vector3d (0.2978, 0.3125, 0.0449));
    pitchBackLinkBody     = Body (0.421, RigidBodyDynamics::Math::Vector3d (-0.0515, -0.1434, -0.009), 
                            RigidBodyDynamics::Math::Vector3d (0.0236, 0.0028, 0.0261));
    pitchBottomLinkBody   = Body (0.359, RigidBodyDynamics::Math::Vector3d (0.1491, -0.0182, 0.0), 
                            RigidBodyDynamics::Math::Vector3d (0.0007, 0.019, 0.0192));
    mainInsertionLinkBody = Body (0.231, RigidBodyDynamics::Math::Vector3d (-0.059, -0.0165, 0.0008), 
                            RigidBodyDynamics::Math::Vector3d (0.0003, 0.0015, 0.0016));
    toolLinkBody          = Body (1.907, RigidBodyDynamics::Math::Vector3d (0., -0.0008, 0.0723), 
                            RigidBodyDynamics::Math::Vector3d (0.0457, 0.0455, 0.0017));
    pitchFrontLinkBody    = Body (1.607, RigidBodyDynamics::Math::Vector3d (-0.0365, -0.1526, 0.0), 
                            RigidBodyDynamics::Math::Vector3d (0.0983, 0.0175, 0.1099));
    pitchTopLinkBody      = Body (0.439, RigidBodyDynamics::Math::Vector3d (0.1702, -0.0001, 0.0008), 
                            RigidBodyDynamics::Math::Vector3d (0.0, 0.0381, 0.0381));
    // No Joint for pittchEndLink as its a p2p joint
    pitchEndLinkBody      = Body (2.032, RigidBodyDynamics::Math::Vector3d (0.0513, 0.0048, 0.0008), 
                            RigidBodyDynamics::Math::Vector3d (0.0636, 0.0099, 0.0726));

    Eigen::Matrix3d base;
    base.setIdentity();
    //--------------------------------------------------------------------//
    Eigen::Vector3d baseLink_yawLinkPA = { -0.0002, -1.0,    0.0 };
    Eigen::Vector3d baseLink_yawLinkCA = {     0.0,  0.0,   -1.0 };
    Eigen::Vector3d baseLink_yawLinkPP = {     0.0,  0.0,    0.0 };
    Eigen::Vector3d baseLink_yawLinkCP = {  0.0001,  0.0, 0.5369 };
    baseLink_yawLinkPA.normalize();
    baseLink_yawLinkCA.normalize();

    Eigen::Matrix3d baseLink_yawLinkRot = 
      Eigen::Matrix3d(Eigen::Quaterniond::FromTwoVectors(baseLink_yawLinkPA, baseLink_yawLinkCA));
    Eigen::Affine3d baseLink_yawLink_offset(Eigen::AngleAxisd(baseLink_yawLinkOffset, 
      Eigen::Vector3d::UnitZ()));
    // Eigen::Matrix3d base_link1_offset = EigenUtilities::rotZ(base_link1Offset);

    baseLink_yawLinkST.E = baseLink_yawLink_offset.rotation() * baseLink_yawLinkRot;
    baseLink_yawLinkST.r = baseLink_yawLinkPP - (baseLink_yawLinkRot.inverse() * baseLink_yawLinkCP);

    baseLink_yawLinkJoint = 
      RigidBodyDynamics::Joint(RigidBodyDynamics::JointType::JointTypeRevolute, 
      baseLink_yawLinkST.E.transpose().block<3, 1>(0, 2).transpose());
    
    const Eigen::Vector3d P_baseLink_yawLink_base =  base * baseLink_yawLinkST.r;
    baseLink_yawLinkID = rbdlModel->AddBody(0, 
      RigidBodyDynamics::Math::Xtrans(P_baseLink_yawLink_base), 
      baseLink_yawLinkJoint, baseLinkBody, "baseLink-yawLink");
    //--------------------------------------------------------------------//
    // Eigen::Vector3d link1_link2PA = { 00.000, 01.000, 00.000 };
    // Eigen::Vector3d link1_link2CA = { 00.000, 00.000, 01.000 };
    // Eigen::Vector3d link1_link2PP = { 00.000, 00.013, 00.209 };
    // Eigen::Vector3d link1_link2CP = { 00.000, 00.000, 00.000 };
    // link1_link2PA.normalize();
    // link1_link2CA.normalize();

    // Eigen::Matrix3d link1_link2Rot = 
    //   Eigen::Matrix3d(Eigen::Quaterniond::FromTwoVectors(link1_link2PA, 
    //   link1_link2CA));
    // Eigen::Affine3d link1_link2_offset(Eigen::AngleAxisd(link1_link2Offset, 
    //   Eigen::Vector3d::UnitZ()));
    // // Eigen::Matrix3d link1_link2_offset = 
    // //   EigenUtilities::rotZ(link1_link2Offset);

    // link1_link2ST.E = link1_link2_offset.rotation() * link1_link2Rot;
    // link1_link2ST.r = 
    //   link1_link2PP - (link1_link2Rot.inverse() * link1_link2CP);
    
    // const SpatialTransform base_link2ST = base_link1ST * link1_link2ST;

    // link1_link2Joint = 
    //   RigidBodyDynamics::Joint(RigidBodyDynamics::JointType::JointTypeRevolute, 
    //   base_link2ST.E.transpose().block<3, 1>(0, 2).transpose());

    // const Eigen::Vector3d P_base_link2_base = 
    //   base_link1ST.E.inverse() * link1_link2ST.r;
    // link1_link2ID = rbdlModel->AddBody(base_link1ID, 
    //   RigidBodyDynamics::Math::Xtrans(P_base_link2_base), 
    //   link1_link2Joint, link2Body, "link1-link2");
    // //--------------------------------------------------------------------//
    // Eigen::Vector3d link2_link3PA = { 00.000, -1.000, 00.000 };
    // Eigen::Vector3d link2_link3CA = { 00.000, 00.000, 01.000 };
    // Eigen::Vector3d link2_link3PP = { 00.000, -0.194, -0.009 };
    // Eigen::Vector3d link2_link3CP = { 00.000, 00.000, 00.000 };
    // link2_link3PA.normalize();
    // link2_link3CA.normalize();

    // Eigen::Matrix3d link2_link3Rot = 
    //   Eigen::Matrix3d(Eigen::Quaterniond::FromTwoVectors(link2_link3PA, 
    //                                                       link2_link3CA));
    // Eigen::Affine3d link2_link3_offset(Eigen::AngleAxisd(link2_link3Offset, 
    //   Eigen::Vector3d::UnitZ()));
    // // Eigen::Matrix3d link2_link3_offset = 
    // //   EigenUtilities::rotZ(link2_link3Offset);

    // link2_link3ST.E = link2_link3_offset.rotation() * link2_link3Rot;
    // link2_link3ST.r = 
    //   link2_link3PP - (link2_link3Rot.inverse() * link2_link3CP);   

    // const SpatialTransform base_link3ST = base_link2ST * link2_link3ST;

    // link2_link3Joint = 
    //   RigidBodyDynamics::Joint(RigidBodyDynamics::JointType::JointTypeRevolute, 
    //   base_link3ST.E.transpose().block<3, 1>(0, 2).transpose());

    // const Eigen::Vector3d P_base_link3_base = 
    //   base_link2ST.E.inverse() * link2_link3ST.r;
    // link2_link3ID = rbdlModel->AddBody(link1_link2ID, 
    //   RigidBodyDynamics::Math::Xtrans(P_base_link3_base), 
    //   link2_link3Joint, link3Body, "link2-link3");
    // //--------------------------------------------------------------------//
    // Eigen::Vector3d link3_link4PA = { 00.000, -1.000, 00.000 };
    // Eigen::Vector3d link3_link4CA = { 00.000, 00.000, 01.000 };
    // Eigen::Vector3d link3_link4PP = { 00.000, -0.013, 00.202 };
    // Eigen::Vector3d link3_link4CP = { 00.000, 00.000, 00.000 };
    // link3_link4PA.normalize();
    // link3_link4CA.normalize();

    // Eigen::Matrix3d link3_link4Rot = 
    //   Eigen::Matrix3d(Eigen::Quaterniond::FromTwoVectors(link3_link4PA, 
    //   link3_link4CA));
    // Eigen::Affine3d link3_link4_offset(Eigen::AngleAxisd(link3_link4Offset, 
    //   Eigen::Vector3d::UnitZ()));
    // // Eigen::Matrix3d link3_link4_offset = 
    // //   EigenUtilities::rotZ(link3_link4Offset);

    // link3_link4ST.E = link3_link4_offset.rotation() * link3_link4Rot;
    // link3_link4ST.r = 
    //   link3_link4PP - (link3_link4Rot.inverse() * link3_link4CP);

    // const SpatialTransform base_link4ST = base_link3ST * link3_link4ST;

    // link3_link4Joint = 
    //   RigidBodyDynamics::Joint(RigidBodyDynamics::JointType::JointTypeRevolute, 
    //   base_link4ST.E.transpose().block<3, 1>(0, 2).transpose());

    // const Eigen::Vector3d P_base_link4_base = 
    //   base_link3ST.E.inverse() * link3_link4ST.r;
    // link3_link4ID = rbdlModel->AddBody(link2_link3ID, 
    //   RigidBodyDynamics::Math::Xtrans(P_base_link4_base), link3_link4Joint, 
    //   link4Body, "link3-link4");
    // //--------------------------------------------------------------------//
    // Eigen::Vector3d link4_link5PA = { 00.000, 01.000, 00.000 };
    // Eigen::Vector3d link4_link5CA = { 00.000, 00.000, 01.000 };
    // Eigen::Vector3d link4_link5PP = { -0.002, 00.202, -0.008 };
    // Eigen::Vector3d link4_link5CP = { 00.000, 00.000, 00.000 };
    // link4_link5PA.normalize();
    // link4_link5CA.normalize();

    // Eigen::Matrix3d link4_link5Rot = 
    //   Eigen::Matrix3d(Eigen::Quaterniond::FromTwoVectors(link4_link5PA, 
    //   link4_link5CA));
    // Eigen::Affine3d link4_link5_offset(Eigen::AngleAxisd(link4_link5Offset, 
    //   Eigen::Vector3d::UnitZ()));
    // // Eigen::Matrix3d link4_link5_offset = 
    // //   EigenUtilities::rotZ(link5_link6Offset);

    // link4_link5ST.E = link4_link5_offset.rotation() * link4_link5Rot;
    // link4_link5ST.r = 
    //   link4_link5PP - (link4_link5Rot.inverse() * link4_link5CP);

    // const SpatialTransform base_link5ST = base_link4ST * link4_link5ST;

    // const Eigen::Vector3d P_base_link5_base = 
    //   base_link4ST.E.inverse() * link4_link5ST.r;

    // link4_link5Joint = 
    //   RigidBodyDynamics::Joint(RigidBodyDynamics::JointType::JointTypeRevolute, 
    //   base_link5ST.E.transpose().block<3, 1>(0, 2).transpose());

    // link4_link5ID = rbdlModel->AddBody(link3_link4ID, 
    //   RigidBodyDynamics::Math::Xtrans(P_base_link5_base), link4_link5Joint, 
    //   link5Body, "link4-link5");
    // //--------------------------------------------------------------------//
    // Eigen::Vector3d link5_link6PA = { 00.000, 01.000, 00.000 };
    // Eigen::Vector3d link5_link6CA = { 00.000, 00.000, 01.000 };
    // Eigen::Vector3d link5_link6PP = { 00.002, -0.052, 00.204 };
    // Eigen::Vector3d link5_link6CP = { 00.000, 00.000, 00.000 };
    // link5_link6PA.normalize();
    // link5_link6CA.normalize();

    // Eigen::Matrix3d link5_link6Rot = 
    //   Eigen::Matrix3d(Eigen::Quaterniond::FromTwoVectors(link5_link6PA, 
    //   link5_link6CA));
    // Eigen::Affine3d link5_link6_offset(Eigen::AngleAxisd(link5_link6Offset, 
    //   Eigen::Vector3d::UnitZ()));
    // // Eigen::Matrix3d link5_link6_offset = 
    // //   EigenUtilities::rotZ(link5_link6Offset);

    // link5_link6ST.E = link5_link6_offset.rotation() * link5_link6Rot;
    // link5_link6ST.r = 
    //   link5_link6PP - (link5_link6Rot.inverse() * link5_link6CP);

    // const SpatialTransform base_link6ST = base_link5ST * link5_link6ST;

    // const Eigen::Vector3d P_base_link6_base = 
    //   base_link5ST.E.inverse() * link5_link6ST.r;

    // link5_link6Joint = 
    //   RigidBodyDynamics::Joint(RigidBodyDynamics::JointType::JointTypeRevolute, 
    //   base_link6ST.E.transpose().block<3, 1>(0, 2).transpose());
    // link5_link6ID = rbdlModel->AddBody(link4_link5ID, 
    //   RigidBodyDynamics::Math::Xtrans(P_base_link6_base), link5_link6Joint, 
    //                                                         link6Body, "link5-link6");
    // //--------------------------------------------------------------------//
    // Eigen::Vector3d link6_link7PA = { 00.000, -1.000, 00.000 };
    // Eigen::Vector3d link6_link7CA = { 00.000, 00.000, 01.000 };
    // Eigen::Vector3d link6_link7PP = { -0.003, -0.050, 00.053 };
    // Eigen::Vector3d link6_link7CP = { 00.000, 00.000, 00.000 };
    // link6_link7PA.normalize();
    // link6_link7CA.normalize();

    // Eigen::Matrix3d link6_link7Rot = 
    //   Eigen::Matrix3d(Eigen::Quaterniond::FromTwoVectors(link6_link7PA, 
    //   link6_link7CA));
    // Eigen::Affine3d link6_link7_offset(Eigen::AngleAxisd(link6_link7Offset, 
    //   Eigen::Vector3d::UnitZ()));
    // // Eigen::Matrix3d link6_link7_offset = 
    // //   EigenUtilities::rotZ(link6_link7Offset);

    // link6_link7ST.E = link6_link7_offset.rotation() * link6_link7Rot;
    // link6_link7ST.r = 
    //   link6_link7PP - (link6_link7Rot.inverse() * link6_link7CP);

    // const SpatialTransform base_link7ST = base_link6ST * link6_link7ST;

    // const Eigen::Vector3d P_base_link7_base = 
    //   base_link6ST.E.inverse() * link6_link7ST.r;

    // link6_link7Joint = 
    //   RigidBodyDynamics::Joint(RigidBodyDynamics::JointType::JointTypeRevolute, 
    //   base_link7ST.E.transpose().block<3, 1>(0, 2).transpose());

    // link6_link7ID = rbdlModel->AddBody(link5_link6ID, 
    //   RigidBodyDynamics::Math::Xtrans(P_base_link7_base), link6_link7Joint, 
    //   link7Body, "link6-link7");
    // //--------------------------------------------------------------------//
    Q     = VectorNd::Constant ((size_t) rbdlModel->dof_count, 0.);
    QDot  = VectorNd::Constant ((size_t) rbdlModel->dof_count, 0.);
    QDDot = VectorNd::Constant ((size_t) rbdlModel->dof_count, 0.);
    Tau   = VectorNd::Constant ((size_t) rbdlModel->dof_count, 0.); 

    ClearLogOutput();
  }

  void initializeHandlers()
  {
    clientPtr = RBDLTestPrep::getInstance()->getAMBFClientInstance();
    clientPtr->connect();
    usleep(20000);

  for(BODY_JOINTS_MAP_itr  = BODY_JOINTS_MAP.begin(); BODY_JOINTS_MAP_itr != BODY_JOINTS_MAP.end();
      BODY_JOINTS_MAP_itr++)
    {
      std::string rigidBodyName = BODY_JOINTS_MAP_itr->first;
      ControllableBodyParams controllableBodyParams = BODY_JOINTS_MAP_itr->second;

      rigidBodyPtr rigidBodyHandler = clientPtr->getRigidBody(rigidBodyName, true);
      usleep(1000000);

      rigidBodyHandler->set_joint_pos(0, 0.0f);

      controllableBodyParams.rigidBodyHandler = rigidBodyHandler;
      BODY_JOINTS_MAP[rigidBodyName] = controllableBodyParams;
      printf("Creating handler for rigidbody: %s\n", rigidBodyName.c_str());
    }
  }

  void getBaseTransform()
  {
    // base is rigid body name, not a joint. This is a hacky way to enable ros topics in the 
    // server side during first execution

    rigidBodyPtr baseLinkHandler = BODY_JOINTS_MAP[baseRigidBodyName].rigidBodyHandler;
    // baseHandler->set_joint_pos(baseRigidBodyName, 0.0f);
    // baseHandler->set_joint_pos(0, 0.0f);

    const tf::Quaternion quat_w_0_tf = baseLinkHandler->get_rot();
    const tf::Vector3 P_w_0_tf = baseLinkHandler->get_pos();

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
    std::cout << "T_w_0" << std::endl << T_w_0 << std::endl;

    T_0_w = EigenUtilities::get_frame<Eigen::Matrix3d, 
                Eigen::Vector3d, Eigen::Matrix4d>(R_w_0.transpose(), -R_w_0.transpose() * P_w_0);
  }

  void cleanUp()
  {
    for(BODY_JOINTS_MAP_itr  = BODY_JOINTS_MAP.begin();
        BODY_JOINTS_MAP_itr != BODY_JOINTS_MAP.end();
        BODY_JOINTS_MAP_itr++)
    {
      std::string rigidBodyName = BODY_JOINTS_MAP_itr->first;
      printf("Cleaning handler for rigidbody: %s\n", rigidBodyName.c_str());

      ControllableBodyParams controllableBodyParams = BODY_JOINTS_MAP_itr->second;
      if(controllableBodyParams.rigidBodyHandler == nullptr)
        printf("nullptr for rigidBodyName: %s\n", rigidBodyName.c_str());
      controllableBodyParams.rigidBodyHandler->cleanUp();
    }
    
    clientPtr->cleanUp();
  }

  ~ECM() 
  {
    delete rbdlModel;
    cleanUp();
  }

  Model *rbdlModel = nullptr;

  AMBFClientPtr clientPtr = nullptr;
  std::string baseRigidBodyName = "ecm/baselink";

  Eigen::Matrix4d T_w_0;
  Eigen::Matrix4d T_0_w;

  // Child : Parent
  std::unordered_map<std::string, std::string> hierachyMap =
  {
    {               "ROOT", "ROOT"              },
    {           "baselink", "ROOT"              },
    {       "pitchendlink", "pitchbottomlink"   },
    {            "yawlink", "ROOT"              },
    {      "pitchbacklink", "yawlink"           },
    {    "pitchbottomlink", "pitchbacklink"     },
    {  "maininsertionlink", "pitchendlink"      },
    {           "toollink", "maininsertionlink" },
    {     "pitchfrontlink", "yawlink"           },
    // { "pitchbottomlink", "pitchfrontlink"    },
    {       "pitchtoplink", "pitchfrontlink"    },
    // {    "pitchendlink", { "pitchtoplink"    }
  };


  // static const inline 
  std::unordered_map<std::string, ControllableBodyParams> BODY_JOINTS_MAP =
  {
    { 
      "ecm/baselink", 
      {
        nullptr,
        { 
          "baselink-yawlink",
          "yawlink-pitchbacklink",
          "pitchendlink-maininsertionlink",
          "maininsertionlink-toollink",
          "pitchbacklink-pitchbottomlink", 
          "yawlink-pitchfrontlink",
          "pitchfrontlink-pitchtoplink" 
        },
        { 
          {               "baselink-yawlink", -1.595, 1.595 },
          {          "yawlink-pitchbacklink", -0.784, 1.158 },
          { "pitchendlink-maininsertionlink", 0.0000, 0.254 },
          {     "maininsertionlink-toollink", -1.553, 1.567 },
        }
      }
    },
    { 
      "ecm/yawlink", 
      {
        nullptr,
        { 
          "pitchbottomlink-pitchendlink"
        },
        {}
      }
    },
    // { 
    //   "ecm/pitchfrontlink", 
    //   {
    //     nullptr,
    //     { 
    //       "pitchfrontlink-pitchbottomlink"
    //     },
    //     {}
    //   }
    // },
    // { 
    //   "ecm/pitchtoplink", 
    //   {
    //     nullptr,
    //     { 
    //       "pitchtoplink-pitchendlink"
    //     },
    //     {}
    //   }
    // }
  };

  std::unordered_map<std::string, ControllableBodyParams>::const_iterator BODY_JOINTS_MAP_itr;

  unsigned int baseLink_yawLinkID, pitchEndLinkId, mainInsertionLinkId, toolLinkId, yawLinkId, 
            pitchBackLinkId, pitchBottomLinkId, pitchFrontLinkId, pitchTopLinkId;

  Body baseLinkBody, pitchEndLinkBody, mainInsertionLinkBody, toolLinkBody, yawLinkBody, 
            pitchBackLinkBody, pitchBottomLinkBody, pitchFrontLinkBody, pitchTopLinkBody;

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

  const double ROOT_baseLinkOffset                  = 0.0;
  const double baseLink_pitchEndLinkOffset          = 0.0;
  const double pitchEndLink_mainInsertionLinkOffset = 0.0;
  const double mainInsertionLink_toolLinkOffset     = 0.0;
  const double baseLink_yawLinkOffset               = 0.0;
  const double yawLink_pitchBackLinkOffset          = 0.0;
  const double pitchBackLink_pitchBottomLinkOffset  = 0.0;
  const double pitchBottomLink_pitchEndLinkOffset   = 0.0;
  const double yawLink_pitchFrontLinkOffset         = 0.0;
  const double pitchFrontLink_pitchBottomLinkOffset = 0.0;
  const double pitchFrontLink_pitchTopLinkOffset    = 0.0;
  const double pitchTopLink_pitchEndLinkOffset      = 0.0;

  VectorNd Q;
  VectorNd QDot;
  VectorNd QDDot;
  VectorNd Tau;
};