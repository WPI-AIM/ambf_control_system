#ifndef _PSM_
#define _PSM_

#include "Tests/Utilities.h"

const double TEST_PREC = 1.0e-3;
const double TEST_LAX = 1.0e-5;


struct PSM 
{
  PSM()
  {
    ClearLogOutput();
    model = new Model;
    model->gravity = Vector3d(0., 0., -9.81);

    Frames F_B_baselink_yawlink = 
      Utilities::Q_B_Frames(v_baselink_yawlink_ax_jINp, v_baselink_yawlink_ap, 
        v_baselink_yawlink_ac, q_baselink_yawlink_jointOffset, q_baselink_yawlink_childOffset);
    Frames F_B_yawlink_pitchbacklink = 
      Utilities::Q_B_Frames(v_yawlink_pitchbacklink_ax_jINp, v_yawlink_pitchbacklink_ap, 
        v_yawlink_pitchbacklink_ac, q_yawlink_pitchbacklink_jointOffset, q_yawlink_pitchbacklink_childOffset);
    // Frames F_B_pitchbacklink_pitchbottomlink = 
    //   Utilities::Q_B_Frames(v_pitchbacklink_pitchbottomlink_ax_jINp, v_pitchbacklink_pitchbottomlink_ap, 
    //     v_pitchbacklink_pitchbottomlink_ac, q_pitchbacklink_pitchbottomlink_jointOffset, q_pitchbacklink_pitchbottomlink_childOffset);
    // Frames F_B_pitchbottomlink_pitchendlink = 
    //   Utilities::Q_B_Frames(v_pitchbottomlink_pitchendlink_ax_jINp, v_pitchbottomlink_pitchendlink_ap, 
    //     v_pitchbottomlink_pitchendlink_ac, q_pitchbottomlink_pitchendlink_jointOffset, q_pitchbottomlink_pitchendlink_childOffset);
    // Frames F_B_yawlink_pitchfrontlink = 
    //   Utilities::Q_B_Frames(v_yawlink_pitchfrontlink_ax_jINp, v_yawlink_pitchfrontlink_ap, 
    //     v_yawlink_pitchfrontlink_ac, q_yawlink_pitchfrontlink_jointOffset, q_yawlink_pitchfrontlink_childOffset);
    // Frames F_B_pitchfrontlink_pitchbottomlink = 
    //   Utilities::Q_B_Frames(v_pitchfrontlink_pitchbottomlink_ax_jINp, v_pitchfrontlink_pitchbottomlink_ap, 
    //     v_pitchfrontlink_pitchbottomlink_ac, q_pitchfrontlink_pitchbottomlink_jointOffset, q_pitchfrontlink_pitchbottomlink_childOffset);
    // Frames F_B_pitchfrontlink_pitchtoplink = 
    //   Utilities::Q_B_Frames(v_pitchfrontlink_pitchtoplink_ax_jINp, v_pitchfrontlink_pitchtoplink_ap, 
    //     v_pitchfrontlink_pitchtoplink_ac, q_pitchfrontlink_pitchtoplink_jointOffset, q_pitchfrontlink_pitchtoplink_childOffset);
    // Frames F_B_pitchtoplink_pitchendlink = 
    //   Utilities::Q_B_Frames(v_pitchtoplink_pitchendlink_ax_jINp, v_pitchtoplink_pitchendlink_ap, 
    //     v_pitchtoplink_pitchendlink_ac, q_pitchtoplink_pitchendlink_jointOffset, q_pitchtoplink_pitchendlink_childOffset);
    // Frames F_B_pitchendlink_maininsertionlink = 
    //   Utilities::Q_B_Frames(v_pitchendlink_maininsertionlink_ax_jINp, v_pitchendlink_maininsertionlink_ap, 
    //     v_pitchendlink_maininsertionlink_ac, q_pitchendlink_maininsertionlink_jointOffset, q_pitchendlink_maininsertionlink_childOffset);
    // Frames F_B_maininsertionlink_toollink = 
    //   Utilities::Q_B_Frames(v_maininsertionlink_toollink_ax_jINp, v_maininsertionlink_toollink_ap, 
    //     v_maininsertionlink_toollink_ac, q_maininsertionlink_toollink_jointOffset, q_maininsertionlink_toollink_childOffset);

    //-----------------------------------------------------//
    Eigen::Matrix3d R_W_world_baselink = 
      Matrix3d(-1.0,  0.0,  0.0,
                0.0, -1.0,  0.0,
                0.0,  0.0,  1.0);
    Eigen::Quaterniond Q_W_world_baselink(R_W_world_baselink);
    //--------------------------//

    // std::cout << "F_B_baselink_yawlink.Q_W_FP: " << F_B_baselink_yawlink.Q_W_FP.toRotationMatrix() << std::endl;
    // std::cout << "F_B_baselink_yawlink.Q_W_FC: " << F_B_baselink_yawlink.Q_W_FC.toRotationMatrix() << std::endl;
    Eigen::Matrix3d R_W_world_yawlink;
    R_W_world_yawlink = 
      Matrix3d( 
        0.0,  1.0,  0.0,
        0.0,  0.0,  1.0,
        1.0,  0.0,  0.0
      );
    const Eigen::Quaterniond Q_W_world_yawlink = Eigen::Quaterniond(R_W_world_yawlink);

      // Q_W_world_baselink * F_B_baselink_yawlink.Q_W_FP * F_B_baselink_yawlink.Q_W_FC;
    CHECK_THAT (
      Matrix3d( 0.0,  1.0,  0.0,
                0.0,  0.0,  1.0,
                1.0,  0.0,  0.0), 
      AllCloseMatrix(Q_W_world_yawlink.toRotationMatrix(), TEST_PREC, TEST_PREC));
    //--------------------------//
    std::cout << "F_B_yawlink_pitchbacklink.Q_W_FP: \n" << F_B_yawlink_pitchbacklink.Q_W_FP.toRotationMatrix() << std::endl;
    std::cout << "F_B_yawlink_pitchbacklink.Q_W_FC: \n" << F_B_yawlink_pitchbacklink.Q_W_FC.toRotationMatrix() << std::endl;
    const Eigen::Quaterniond Q_W_world_pitchbacklink = 
      Q_W_world_yawlink * F_B_yawlink_pitchbacklink.Q_W_FP * F_B_yawlink_pitchbacklink.Q_W_FC;
      

    CHECK_THAT (
      Matrix3d( 0.0,       0.0,   1.0,
           0.283474,  -0.95898,   0.0,
            0.95898,   0.283474,  0.0), 
      AllCloseMatrix(Q_W_world_pitchbacklink.toRotationMatrix(), TEST_PREC, TEST_PREC));
    // //--------------------------//
    // const Eigen::Quaterniond Q_W_world_pitchbottomlink = 
    //   Q_W_world_pitchbacklink * F_B_pitchbacklink_pitchbottomlink.Q_W_FP * F_B_pitchbacklink_pitchbottomlink.Q_W_FC;
    // CHECK_THAT (
    //   Matrix3d( 0.0,  0.0, -1.0,
    //             1.0,  0.0,  0.0,
    //             0.0, -1.0,  0.0), 
    //   AllCloseMatrix(Q_W_world_pitchbottomlink.toRotationMatrix(), TEST_PREC, TEST_PREC));
    // //--------------------------//
    // const Eigen::Quaterniond Q_W_world_pitchendlink = 
    //   Q_W_world_pitchbottomlink * F_B_pitchbottomlink_pitchendlink.Q_W_FP * F_B_pitchbottomlink_pitchendlink.Q_W_FC;
    // CHECK_THAT (
    //   Matrix3d( 0.0,  0.0, -1.0,
    //             1.0,  0.0,  0.0,
    //             0.0, -1.0,  0.0), 
    //   AllCloseMatrix(Q_W_world_pitchendlink.toRotationMatrix(), TEST_PREC, TEST_PREC));  
    // //--------------------------//
    // const Eigen::Quaterniond Q_W_world_pitchfrontlink = 
    //   Q_W_world_yawlink * F_B_yawlink_pitchfrontlink.Q_W_FP * F_B_yawlink_pitchfrontlink.Q_W_FC;
    // CHECK_THAT (
    //   Matrix3d( 0.0,  0.0, -1.0,
    //             1.0,  0.0,  0.0,
    //             0.0, -1.0,  0.0), 
    //   AllCloseMatrix(Q_W_world_pitchfrontlink.toRotationMatrix(), TEST_PREC, TEST_PREC));  
    // //--------------------------//
    // const Eigen::Quaterniond Q_W_world_pitchbottomlink2 = 
    //   Q_W_world_pitchfrontlink * F_B_pitchfrontlink_pitchbottomlink.Q_W_FP * F_B_pitchfrontlink_pitchbottomlink.Q_W_FC;
    // CHECK_THAT (
    //   Matrix3d( 0.0,  0.0, -1.0,
    //             1.0,  0.0,  0.0,
    //             0.0, -1.0,  0.0), 
    //   AllCloseMatrix(Q_W_world_pitchbottomlink2.toRotationMatrix(), TEST_PREC, TEST_PREC));  
    // //--------------------------//
    // const Eigen::Quaterniond Q_W_world_pitchtoplink = 
    //   Q_W_world_pitchfrontlink * F_B_pitchfrontlink_pitchtoplink.Q_W_FP * F_B_pitchfrontlink_pitchtoplink.Q_W_FC;
    // CHECK_THAT (
    //   Matrix3d( 0.0,  0.0, -1.0,
    //             1.0,  0.0,  0.0,
    //             0.0, -1.0,  0.0), 
    //   AllCloseMatrix(Q_W_world_pitchtoplink.toRotationMatrix(), TEST_PREC, TEST_PREC));  
    // //--------------------------//
    // const Eigen::Quaterniond Q_W_world_pitchendlink2 = 
    //   Q_W_world_pitchtoplink * F_B_pitchtoplink_pitchendlink.Q_W_FP * F_B_pitchtoplink_pitchendlink.Q_W_FC;
    // CHECK_THAT (
    //   Matrix3d( 0.0,  0.0, -1.0,
    //             1.0,  0.0,  0.0,
    //             0.0, -1.0,  0.0), 
    //   AllCloseMatrix(Q_W_world_pitchendlink2.toRotationMatrix(), TEST_PREC, TEST_PREC));
    // //--------------------------//
    // const Eigen::Quaterniond Q_W_world_maininsertionlink = 
    //   Q_W_world_pitchendlink * F_B_pitchendlink_maininsertionlink.Q_W_FP * F_B_pitchendlink_maininsertionlink.Q_W_FC;
    // CHECK_THAT (
    //   Matrix3d( 0.0,  0.0, -1.0,
    //             0.0, -1.0,  0.0,
    //            -1.0,  0.0,  0.0), 
    //   AllCloseMatrix(Q_W_world_maininsertionlink.toRotationMatrix(), TEST_PREC, TEST_PREC));
    // //--------------------------//
    // const Eigen::Quaterniond Q_W_world_toollink = 
    //   Q_W_world_maininsertionlink * F_B_maininsertionlink_toollink.Q_W_FP * F_B_maininsertionlink_toollink.Q_W_FC;
    // CHECK_THAT (
    //   Matrix3d( 0.0,  1.0,  0.0,
    //            -1.0,  0.0,  0.0,
    //             0.0,  0.0,  1.0), 
    //   AllCloseMatrix(Q_W_world_toollink.toRotationMatrix(), TEST_PREC, TEST_PREC));
    //--------------------------------------------------------------------//
    // const Vector3d baselink_yawlinkJAxis = world_baselinkST.E * baselink_yawlink_ap;
    
    // const Vector3d yawlink_pitchbacklinkJAxis = world_yawlinkST.E * yawlink_pitchbacklink_ap;
    // const Vector3d pitchbacklink_pitchbottomlinkJAxis = world_pitchbacklinkST.E * pitchbacklink_pitchbottomlink_ap;
    // const Vector3d pitchbottomlink_pitchendlinkJAxis = world_pitchbottomlinkST.E * pitchbottomlink_pitchendlink_ap;
    
    // const Vector3d yawlink_pitchfrontlinkJAxis = world_yawlinkST.E * yawlink_pitchfrontlink_ap;
    // const Vector3d pitchfrontlink_pitchtoplinkJAxis = world_pitchfrontlinkST.E * pitchfrontlink_pitchtoplink_ap;
    // const Vector3d pitchtoplink_pitchendlinkJAxis = world_pitchtoplinkST.E * pitchtoplink_pitchendlink_ap;
    
    // const Vector3d pitchfrontlink_pitchbottomlinkJAxis = world_pitchfrontlinkST.E * pitchfrontlink_pitchbottomlink_ap;
    // const Vector3d pitchendlink_maininsertionlinkJAxis = world_pitchendlinkST.E * pitchendlink_maininsertionlink_ap;
    // const Vector3d maininsertionlink_toollinkJAxis = world_maininsertionlinkST.E * maininsertionlink_toollink_ap;
    //--------------------------------------------------------------------//

    // /*
    // Q Index
    // 0,               baselink-yawlink
    // 1,          yawlink-pitchbacklink
    // 2,  pitchbacklink-pitchbottomlink
    // 3,     pitchbacklink-pitchtoplink
    // 4,      pitchtoplink-pitchendlink
    // 5,         yawlink-pitchfrontlink
    // 6, pitchfrontlink-pitchbottomlink
    // 7,   pitchbottomlink-pitchendlink
    // 8, pitchendlink-maininsertionlink
    // 9, maininsertionlink-toolrolllink
    // 10,     toolrolllink-toolpitchlink
    // 11, toolpitchlink-toolgripper1link
    // 12, toolpitchlink-toolgripper2link

    // */




    Q     = VectorNd::Constant ((std::size_t) model->dof_count, 0.);
    QDot  = VectorNd::Constant ((std::size_t) model->dof_count, 0.);
    QDDot = VectorNd::Constant ((std::size_t) model->dof_count, 0.);
    Tau   = VectorNd::Constant ((std::size_t) model->dof_count, 0.);

    std::map<std::string, unsigned int> rbdlBodyMap;
    std::map<std::string, unsigned int>::iterator rbdlBodyMapItr;
    rbdlBodyMap = model->mBodyNameMap;

    ClearLogOutput();
    // for(rbdlBodyMapItr = rbdlBodyMap.begin(); 
    //     rbdlBodyMapItr != rbdlBodyMap.end(); 
    //     rbdlBodyMapItr++)
    // {
    //   std::string bodyName = rbdlBodyMapItr->first;
    //   unsigned int bodyId = rbdlBodyMapItr->second;

    //   std::string parentName = model->GetBodyName(model->GetParentBodyId(bodyId));
    //   bool isFixedBody = model->IsFixedBodyId(bodyId);
    //   std::cout << parentName << ", " << bodyName    << ", " 
    //             << bodyId     << ", " << isFixedBody << std::endl;
    //   // std::cout << --bodyId << ", " << bodyName << std::endl;
    // }
  }
  
  ~PSM()
  {
    delete model;
  }

public:
  // const Vector3d vector3d_zero = Eigen::Vector3d::Zero();
  // const Vector3d v_baselinkInertialOffset          = { -0.000010, -0.614520, -0.020880 };
  // const Vector3d v_yawlinkInertialOffset           = {  0.000000, -0.016140,  0.134470 };
  // const Vector3d v_pitchbacklinkInertialOffset     = { -0.051500, -0.143430, -0.00900 };
  // const Vector3d v_pitchbottomlinkInertialOffset   = {  0.149130, -0.018160,  0.000000 };
  // const Vector3d v_pitchendlinkInertialOffset      = {  0.051350,  0.004820,  0.000790 };
  // const Vector3d v_maininsertionlinkInertialOffset = { -0.059000, -0.016500,  0.000790 };
  // const Vector3d v_toollinkInertialOffset          = {  0.000000, -0.000810, -0.072320 };
  // const Vector3d v_pitchfrontlinkInertialOffset    = { -0.036490, -0.152610,  0.000000 };
  // const Vector3d v_pitchtoplinkInertialOffset      = {  0.170200, -0.000070,  0.000790 };


  //   // mass, com - inertia offset, inertia
  //   const Body virtualBody           = Body(0., vector3d_zero, vector3d_zero);
  //   const Body baselinkBody          = Body(00.001, v_baselinkInertialOffset, 
  //                                       Vector3d(00.00000, 00.00000, 00.00000));
  //   const Body yawlinkBody           = Body(06.417, v_yawlinkInertialOffset, 
  //                                       Vector3d(00.29778, 00.31243, 00.04495));
  //   const Body pitchbacklinkBody     = Body(00.421, v_pitchbacklinkInertialOffset, 
  //                                       Vector3d(00.02356, 00.00278, 00.02612));
  //   const Body pitchbottomlinkBody   = Body(00.359, v_pitchbottomlinkInertialOffset, 
  //                                       Vector3d(00.00065, 00.01897, 00.01923));
  //   const Body pitchendlinkBody      = Body(02.032, v_pitchendlinkInertialOffset, 
  //                                       Vector3d(00.06359, 00.00994, 00.07258));
  //   const Body maininsertionlinkBody = Body(00.231, v_maininsertionlinkInertialOffset, 
  //                                       Vector3d(00.00029, 00.00147, 00.00159));
  //   const Body toollinkBody          = Body(01.907, v_toollinkInertialOffset, 
  //                                       Vector3d(00.04569, 00.04553, 00.00169));
  //   const Body pitchfrontlinkBody    = Body(01.607, v_pitchfrontlinkInertialOffset, 
  //                                       Vector3d(00.09829, 00.01747, 00.10993));
  //   const Body pitchtoplinkBody      = Body(00.439, v_pitchtoplinkInertialOffset, 
  //                                       Vector3d(00.00030, 00.03813, 00.03812));
 

  const Vector3d v_baselink_yawlink_ax_jINp               = { 0.0, 0.0, 1.0 };
  const Vector3d v_yawlink_pitchbacklink_ax_jINp          = { 0.0, 0.0, 1.0 };
  const Vector3d v_pitchbacklink_pitchbottomlink_ax_jINp  = { 0.0, 0.0, 1.0 };
  const Vector3d v_pitchbacklink_pitchtoplink_ax_jINp     = { 0.0, 0.0, 1.0 };
  const Vector3d v_pitchtoplink_pitchendlink_ax_jINp      = { 0.0, 0.0, 1.0 };
  const Vector3d v_yawlink_pitchfrontlink_ax_jINp         = { 0.0, 0.0, 1.0 };
  const Vector3d v_pitchfrontlink_pitchbottomlink_ax_jINp = { 0.0, 0.0, 1.0 };
  const Vector3d v_pitchbottomlink_pitchendlink_ax_jINp   = { 0.0, 0.0, 1.0 };
  const Vector3d v_pitchendlink_maininsertionlink_ax_jINp = { 1.0, 0.0, 0.0 };
  const Vector3d v_maininsertionlink_toolrolllink_ax_jINp = { 0.0, 0.0, 1.0 };
  const Vector3d v_toolrolllink_toolpitchlink_ax_jINp     = { 0.0, 0.0, 1.0 };
  const Vector3d v_toolpitchlink_toolgripper1link_ax_jINp = { 0.0, 0.0, 1.0 };
  const Vector3d v_toolpitchlink_toolgripper2link_ax_jINp = { 0.0, 0.0, 1.0 };


  const Vector3d v_baselink_yawlink_ap               = { 00.0000, -1.0000, 00.0000 };
  const Vector3d v_baselink_yawlink_ac               = { 00.0000, 00.0000, 01.0000 };
  const Vector3d v_yawlink_pitchbacklink_ap          = { 00.0000, 01.0000, 00.0000 };
  const Vector3d v_yawlink_pitchbacklink_ac          = { 00.0000, 00.0000, 01.0000 };
  const Vector3d v_pitchbacklink_pitchbottomlink_ap  = { 00.0000, 00.0000, 01.0000 };
  const Vector3d v_pitchbacklink_pitchbottomlink_ac  = { 00.0000, 00.0000, 01.0000 };
  const Vector3d v_pitchbacklink_pitchtoplink_ap     = { 00.0000, 00.0000, 01.0000 };
  const Vector3d v_pitchbacklink_pitchtoplink_ac     = { 00.0000, 00.0000, 01.0000 };
  const Vector3d v_pitchtoplink_pitchendlink_ap      = { 00.0000, 00.0000, 01.0000 };
  const Vector3d v_pitchtoplink_pitchendlink_ac      = { 00.0000, 00.0000, 01.0000 };
  const Vector3d v_yawlink_pitchfrontlink_ap         = { 00.0000, 01.0000, 00.0000 };
  const Vector3d v_yawlink_pitchfrontlink_ac         = { 00.0000, 00.0000, 01.0000 };
  const Vector3d v_pitchfrontlink_pitchbottomlink_ap = { 00.0000, 00.0000, 01.0000 };
  const Vector3d v_pitchfrontlink_pitchbottomlink_ac = { 00.0000, 00.0000, 01.0000 };
  const Vector3d v_pitchbottomlink_pitchendlink_ap   = { 00.0000, 00.0000, 01.0000 };
  const Vector3d v_pitchbottomlink_pitchendlink_ac   = { 00.0000, 00.0000, 01.0000 };
  const Vector3d v_pitchendlink_maininsertionlink_ap = { 00.0000, 01.0000, 00.0000 };
  const Vector3d v_pitchendlink_maininsertionlink_ac = { 01.0000, 00.0000, 00.0000 };
  const Vector3d v_maininsertionlink_toolrolllink_ap = { 01.0000, 00.0000, 00.0000 };
  const Vector3d v_maininsertionlink_toolrolllink_ac = { 00.0000, 00.0000, 01.0000 };
  const Vector3d v_toolrolllink_toolpitchlink_ap     = { 00.0000, 01.0000, 00.0000 };
  const Vector3d v_toolrolllink_toolpitchlink_ac     = { 00.0000, 00.0000, 01.0000 };
  const Vector3d v_toolpitchlink_toolgripper1link_ap = { 00.0000, 01.0000, 00.0000 };
  const Vector3d v_toolpitchlink_toolgripper1link_ac = { 00.0000, 00.0000, 01.0000 };
  const Vector3d v_toolpitchlink_toolgripper2link_ap = { 00.0000, -1.0000, 00.0000 };
  const Vector3d v_toolpitchlink_toolgripper2link_ac = { 00.0000, 00.0000, -1.0000 };

  // const Vector3d v_baselink_yawlink_pp               = { 00.0000, -1.0000, 00.0000 };
  // const Vector3d v_baselink_yawlink_pc               = { 00.0000, 00.0000, 01.0000 };
  // const Vector3d v_yawlink_pitchbacklink_pp          = { 00.0000, 01.0000, 00.0000 };
  // const Vector3d v_yawlink_pitchbacklink_pc          = { 00.0000, 00.0000, 01.0000 };
  // const Vector3d v_pitchbacklink_pitchbottomlink_pp  = { 00.0000, 00.0000, 01.0000 };
  // const Vector3d v_pitchbacklink_pitchbottomlink_pc  = { 00.0000, 00.0000, 01.0000 };
  // const Vector3d v_pitchbacklink_pitchtoplink_pp     = { 00.0000, 00.0000, 01.0000 };
  // const Vector3d v_pitchbacklink_pitchtoplink_pc     = { 00.0000, 00.0000, 01.0000 };
  // const Vector3d v_pitchtoplink_pitchendlink_pp      = { 00.0000, 00.0000, 01.0000 };
  // const Vector3d v_pitchtoplink_pitchendlink_pc      = { 00.0000, 00.0000, 01.0000 };
  // const Vector3d v_yawlink_pitchfrontlink_pp         = { 00.0000, 01.0000, 00.0000 };
  // const Vector3d v_yawlink_pitchfrontlink_pc         = { 00.0000, 00.0000, 01.0000 };
  // const Vector3d v_pitchfrontlink_pitchbottomlink_pp = { 00.0000, 00.0000, 01.0000 };
  // const Vector3d v_pitchfrontlink_pitchbottomlink_pc = { 00.0000, 00.0000, 01.0000 };
  // const Vector3d v_pitchbottomlink_pitchendlink_pp   = { 00.0000, 00.0000, 01.0000 };
  // const Vector3d v_pitchbottomlink_pitchendlink_pc   = { 00.0000, 00.0000, 01.0000 };
  // const Vector3d v_pitchendlink_maininsertionlink_pp = { 00.0000, 01.0000, 00.0000 };
  // const Vector3d v_pitchendlink_maininsertionlink_pc = { 01.0000, 00.0000, 00.0000 };
  // const Vector3d v_maininsertionlink_toolrolllink_pp = { 01.0000, 00.0000, 00.0000 };
  // const Vector3d v_maininsertionlink_toolrolllink_pc = { 00.0000, 00.0000, 01.0000 };
  // const Vector3d v_toolrolllink_toolpitchlink_pp     = { 00.0000, 01.0000, 00.0000 };
  // const Vector3d v_toolrolllink_toolpitchlink_pc     = { 00.0000, 00.0000, 01.0000 };
  // const Vector3d v_toolpitchlink_toolgripper1link_pp = { 00.0000, 01.0000, 00.0000 };
  // const Vector3d v_toolpitchlink_toolgripper1link_pc = { 00.0000, 00.0000, 01.0000 };
  // const Vector3d v_toolpitchlink_toolgripper2link_pp = { 00.0000, -1.0000, 00.0000 };
  // const Vector3d v_toolpitchlink_toolgripper2link_pc = { 00.0000, 00.0000, -1.0000 };

  //-----------------------------------------------------//
  const double q_baselink_yawlink_childOffset{1.5726}, q_yawlink_pitchbacklink_childOffset{-0.291}, 
    q_pitchbacklink_pitchbottomlink_childOffset{1.8618}, q_pitchbacklink_pitchtoplink_childOffset{1.8618}, 
    q_pitchtoplink_pitchendlink_childOffset{0.0}, q_yawlink_pitchfrontlink_childOffset{-0.291}, 
    q_pitchfrontlink_pitchbottomlink_childOffset{1.8618}, q_pitchbottomlink_pitchendlink_childOffset{0.0}, 
    q_pitchendlink_maininsertionlink_childOffset{3.14409}, q_maininsertionlink_toolrolllink_childOffset{-1.5734}, 
    q_toolrolllink_toolpitchlink_childOffset{-1.5708}, q_toolpitchlink_toolgripper1link_childOffset{-1.58175}, 
    q_toolpitchlink_toolgripper2link_childOffset{1.55874};


  const double q_baselink_yawlink_jointOffset{0.0}, q_yawlink_pitchbacklink_jointOffset{0.0}, 
    q_pitchbacklink_pitchbottomlink_jointOffset{0.0}, q_pitchbacklink_pitchtoplink_jointOffset{0.0}, 
    q_pitchtoplink_pitchendlink_jointOffset{0.0}, q_yawlink_pitchfrontlink_jointOffset{0.0}, 
    q_pitchfrontlink_pitchbottomlink_jointOffset{0.0}, q_pitchbottomlink_pitchendlink_jointOffset{0.0}, 
    q_pitchendlink_maininsertionlink_jointOffset{0.0}, q_maininsertionlink_toolrolllink_jointOffset{0.0}, 
    q_toolrolllink_toolpitchlink_jointOffset{0.0}, q_toolpitchlink_toolgripper1link_jointOffset{0.0}, 
    q_toolpitchlink_toolgripper2link_jointOffset{0.0};

  const float baselink_scale{1.0f}, yawlink_scale{1.0f}, pitchbacklink_scale{1.0f}, pitchbottomlink_scale{1.0f}, 
    pitchendlink_scale{1.0f}, maininsertionlink_scale{1.0f}, toolrolllink_scale{1.0f}, toolpitchlink_scale{1.0f}, 
    toolgripper1link_scale{1.0f}, toolgripper2link_scale{1.0f}, pitchtoplink_scale{1.0f}, 
    pitchfrontlink_scale{1.0f}; 

  Vector3d v_world_baselink_jAxis, v_baselink_yawlink_jAxis, v_yawlink_pitchbacklink_jAxis, 
    v_pitchbacklink_pitchbottomlink_jAxis, v_pitchbottomlink_pitchendlink_jAxis, 
    v_pitchendlink_maininsertionlink_jAxis, v_maininsertionlink_toolrolllink_jAxis, 
    v_toolrolllink_toolpitchlink_jAxis, v_toolpitchlink_toolgripper1link_jAxis, 
    v_toolpitchlink_toolgripper2link_jAxis, v_pitchbacklink_pitchtoplink_jAxis, 
    v_yawlink_pitchfrontlink_jAxis;

  unsigned int 
    world_baselink_bodyId{0}, baselink_yawlink_bodyId{0}, yawlink_pitchbacklink_bodyId{0}, 
    pitchbacklink_pitchbottomlink_bodyId{0}, pitchbacklink_pitchtoplink_bodyId{0}, 
    pitchtoplink_pitchendlink_bodyId{0}, yawlink_pitchfrontlink_bodyId{0}, 
    pitchfrontlink_pitchbottomlink_bodyId{0}, pitchbottomlink_pitchendlink_bodyId{0}, 
    pitchendlink_maininsertionlink_bodyId{0}, maininsertionlink_toolrolllink_bodyId{0}, 
    toolrolllink_toolpitchlink_bodyId{0}, toolpitchlink_toolgripper1link_bodyId{0}, 
    toolpitchlink_toolgripper2link_bodyId{0};

  RigidBodyDynamics::Model *model;

  RigidBodyDynamics::Math::VectorNd Q;
  RigidBodyDynamics::Math::VectorNd QDot;
  RigidBodyDynamics::Math::VectorNd QDDot;
  RigidBodyDynamics::Math::VectorNd Tau;
};

#endif