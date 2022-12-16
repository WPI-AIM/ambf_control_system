#ifndef _ECM3_
#define _ECM3_

#include "Tests/Utilities.h"

const double TEST_PREC = 1.0e-3;
const double TEST_LAX = 1.0e-5;


struct ECM3 
{
  ECM3()
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
    Frames F_B_pitchbacklink_pitchbottomlink = 
      Utilities::Q_B_Frames(v_pitchbacklink_pitchbottomlink_ax_jINp, v_pitchbacklink_pitchbottomlink_ap, 
        v_pitchbacklink_pitchbottomlink_ac, q_pitchbacklink_pitchbottomlink_jointOffset, q_pitchbacklink_pitchbottomlink_childOffset);
    Frames F_B_pitchbottomlink_pitchendlink = 
      Utilities::Q_B_Frames(v_pitchbottomlink_pitchendlink_ax_jINp, v_pitchbottomlink_pitchendlink_ap, 
        v_pitchbottomlink_pitchendlink_ac, q_pitchbottomlink_pitchendlink_jointOffset, q_pitchbottomlink_pitchendlink_childOffset);
    Frames F_B_yawlink_pitchfrontlink = 
      Utilities::Q_B_Frames(v_yawlink_pitchfrontlink_ax_jINp, v_yawlink_pitchfrontlink_ap, 
        v_yawlink_pitchfrontlink_ac, q_yawlink_pitchfrontlink_jointOffset, q_yawlink_pitchfrontlink_childOffset);
    Frames F_B_pitchfrontlink_pitchbottomlink = 
      Utilities::Q_B_Frames(v_pitchfrontlink_pitchbottomlink_ax_jINp, v_pitchfrontlink_pitchbottomlink_ap, 
        v_pitchfrontlink_pitchbottomlink_ac, q_pitchfrontlink_pitchbottomlink_jointOffset, q_pitchfrontlink_pitchbottomlink_childOffset);
    Frames F_B_pitchfrontlink_pitchtoplink = 
      Utilities::Q_B_Frames(v_pitchfrontlink_pitchtoplink_ax_jINp, v_pitchfrontlink_pitchtoplink_ap, 
        v_pitchfrontlink_pitchtoplink_ac, q_pitchfrontlink_pitchtoplink_jointOffset, q_pitchfrontlink_pitchtoplink_childOffset);
    Frames F_B_pitchtoplink_pitchendlink = 
      Utilities::Q_B_Frames(v_pitchtoplink_pitchendlink_ax_jINp, v_pitchtoplink_pitchendlink_ap, 
        v_pitchtoplink_pitchendlink_ac, q_pitchtoplink_pitchendlink_jointOffset, q_pitchtoplink_pitchendlink_childOffset);
    Frames F_B_pitchendlink_maininsertionlink = 
      Utilities::Q_B_Frames(v_pitchendlink_maininsertionlink_ax_jINp, v_pitchendlink_maininsertionlink_ap, 
        v_pitchendlink_maininsertionlink_ac, q_pitchendlink_maininsertionlink_jointOffset, q_pitchendlink_maininsertionlink_childOffset);
    Frames F_B_maininsertionlink_toollink = 
      Utilities::Q_B_Frames(v_maininsertionlink_toollink_ax_jINp, v_maininsertionlink_toollink_ap, 
        v_maininsertionlink_toollink_ac, q_maininsertionlink_toollink_jointOffset, q_maininsertionlink_toollink_childOffset);

    //-----------------------------------------------------//
    Eigen::Quaterniond Q_W_world_baselink; Q_W_world_baselink.setIdentity();
    //--------------------------//

    const Eigen::Quaterniond Q_W_world_yawlink = 
      Q_W_world_baselink * F_B_baselink_yawlink.Q_W_FP * F_B_baselink_yawlink.Q_W_FC;
    CHECK_THAT (
      Matrix3d(-1.0,  0.0,  0.0,
                0.0,  0.0,  1.0,
                0.0,  1.0,  0.0), 
      AllCloseMatrix(Q_W_world_yawlink.toRotationMatrix(), TEST_PREC, TEST_PREC));
    //--------------------------//
    const Eigen::Quaterniond Q_W_world_pitchbacklink = 
      Q_W_world_yawlink * F_B_yawlink_pitchbacklink.Q_W_FP * F_B_yawlink_pitchbacklink.Q_W_FC;

    CHECK_THAT (
      Matrix3d( 0.0,  0.0, -1.0,
                1.0,  0.0,  0.0,
                0.0, -1.0,  0.0), 
      AllCloseMatrix(Q_W_world_pitchbacklink.toRotationMatrix(), TEST_PREC, TEST_PREC));
    //--------------------------//
    const Eigen::Quaterniond Q_W_world_pitchbottomlink = 
      Q_W_world_pitchbacklink * F_B_pitchbacklink_pitchbottomlink.Q_W_FP * F_B_pitchbacklink_pitchbottomlink.Q_W_FC;
    CHECK_THAT (
      Matrix3d( 0.0,  0.0, -1.0,
                1.0,  0.0,  0.0,
                0.0, -1.0,  0.0), 
      AllCloseMatrix(Q_W_world_pitchbottomlink.toRotationMatrix(), TEST_PREC, TEST_PREC));
    //--------------------------//
    const Eigen::Quaterniond Q_W_world_pitchendlink = 
      Q_W_world_pitchbottomlink * F_B_pitchbottomlink_pitchendlink.Q_W_FP * F_B_pitchbottomlink_pitchendlink.Q_W_FC;
    CHECK_THAT (
      Matrix3d( 0.0,  0.0, -1.0,
                1.0,  0.0,  0.0,
                0.0, -1.0,  0.0), 
      AllCloseMatrix(Q_W_world_pitchendlink.toRotationMatrix(), TEST_PREC, TEST_PREC));  
    //--------------------------//
    const Eigen::Quaterniond Q_W_world_pitchfrontlink = 
      Q_W_world_yawlink * F_B_yawlink_pitchfrontlink.Q_W_FP * F_B_yawlink_pitchfrontlink.Q_W_FC;
    CHECK_THAT (
      Matrix3d( 0.0,  0.0, -1.0,
                1.0,  0.0,  0.0,
                0.0, -1.0,  0.0), 
      AllCloseMatrix(Q_W_world_pitchfrontlink.toRotationMatrix(), TEST_PREC, TEST_PREC));  
    //--------------------------//
    const Eigen::Quaterniond Q_W_world_pitchbottomlink2 = 
      Q_W_world_pitchfrontlink * F_B_pitchfrontlink_pitchbottomlink.Q_W_FP * F_B_pitchfrontlink_pitchbottomlink.Q_W_FC;
    CHECK_THAT (
      Matrix3d( 0.0,  0.0, -1.0,
                1.0,  0.0,  0.0,
                0.0, -1.0,  0.0), 
      AllCloseMatrix(Q_W_world_pitchbottomlink2.toRotationMatrix(), TEST_PREC, TEST_PREC));  
    //--------------------------//
    const Eigen::Quaterniond Q_W_world_pitchtoplink = 
      Q_W_world_pitchfrontlink * F_B_pitchfrontlink_pitchtoplink.Q_W_FP * F_B_pitchfrontlink_pitchtoplink.Q_W_FC;
    CHECK_THAT (
      Matrix3d( 0.0,  0.0, -1.0,
                1.0,  0.0,  0.0,
                0.0, -1.0,  0.0), 
      AllCloseMatrix(Q_W_world_pitchtoplink.toRotationMatrix(), TEST_PREC, TEST_PREC));  
    //--------------------------//
    const Eigen::Quaterniond Q_W_world_pitchendlink2 = 
      Q_W_world_pitchtoplink * F_B_pitchtoplink_pitchendlink.Q_W_FP * F_B_pitchtoplink_pitchendlink.Q_W_FC;
    CHECK_THAT (
      Matrix3d( 0.0,  0.0, -1.0,
                1.0,  0.0,  0.0,
                0.0, -1.0,  0.0), 
      AllCloseMatrix(Q_W_world_pitchendlink2.toRotationMatrix(), TEST_PREC, TEST_PREC));
    //--------------------------//
    const Eigen::Quaterniond Q_W_world_maininsertionlink = 
      Q_W_world_pitchendlink * F_B_pitchendlink_maininsertionlink.Q_W_FP * F_B_pitchendlink_maininsertionlink.Q_W_FC;
    CHECK_THAT (
      Matrix3d( 0.0,  0.0, -1.0,
                0.0, -1.0,  0.0,
               -1.0,  0.0,  0.0), 
      AllCloseMatrix(Q_W_world_maininsertionlink.toRotationMatrix(), TEST_PREC, TEST_PREC));
    //--------------------------//
    const Eigen::Quaterniond Q_W_world_toollink = 
      Q_W_world_maininsertionlink * F_B_maininsertionlink_toollink.Q_W_FP * F_B_maininsertionlink_toollink.Q_W_FC;
    CHECK_THAT (
      Matrix3d( 0.0,  1.0,  0.0,
               -1.0,  0.0,  0.0,
                0.0,  0.0,  1.0), 
      AllCloseMatrix(Q_W_world_toollink.toRotationMatrix(), TEST_PREC, TEST_PREC));
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
    // 0, baselink-yawlink
    // 1, yawlink-pitchbacklink
    // 2, pitchbacklink-pitchbottomlink
    // 3, pitchbottomlink-pitchendlink
    // 4, yawlink-pitchfrontlink
    // 5, pitchfrontlink-pitchtoplink
    // 6, pitchtoplink-pitchendlink
    // 7, pitchendlink-maininsertionlink
    // 8, maininsertionlink-toollink
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
  
  ~ECM3()
  {
    delete model;
  }

public:
  const Vector3d vector3d_zero = Eigen::Vector3d::Zero();
  const Body virtualBody              = Body(0., vector3d_zero, vector3d_zero);

  const Vector3d v_baselinkInertialOffset           = { -0.000010, -0.614520, -0.020880 };
  const Vector3d v_yawlinkInertialOffset          = {  0.000000, -0.016140,  0.134470 };
  const Vector3d v_pitchbacklinkInertialOffset    = { -0.051500, -0.143430, -0.00900 };
  const Vector3d v_pitchbottomlinkInertialOffset   = {  0.149130, -0.018160,  0.000000 };
  const Vector3d v_pitchendlinkInertialOffset       = {  0.051350,  0.004820,  0.000790 };
  const Vector3d v_maininsertionlinkInertialOffset = { -0.059000, -0.016500,  0.000790 };
  const Vector3d v_toollinkInertialOffset           = {  0.000000, -0.000810, -0.072320 };
  const Vector3d v_pitchfrontlinkInertialOffset     = { -0.036490, -0.152610,  0.000000 };
  const Vector3d v_pitchtoplinkInertialOffset       = {  0.170200, -0.000070,  0.000790 };


    // mass, com - inertia offset, inertia
    const Body baselinkBody          = Body(00.001, v_baselinkInertialOffset, 
                                        Vector3d(00.00000, 00.00000, 00.00000));
    const Body yawlinkBody           = Body(06.417, v_yawlinkInertialOffset, 
                                        Vector3d(00.29778, 00.31243, 00.04495));
    const Body pitchbacklinkBody     = Body(00.421, v_pitchbacklinkInertialOffset, 
                                        Vector3d(00.02356, 00.00278, 00.02612));
    const Body pitchbottomlinkBody   = Body(00.359, v_pitchbottomlinkInertialOffset, 
                                        Vector3d(00.00065, 00.01897, 00.01923));
    const Body pitchendlinkBody      = Body(02.032, v_pitchendlinkInertialOffset, 
                                        Vector3d(00.06359, 00.00994, 00.07258));
    const Body maininsertionlinkBody = Body(00.231, v_maininsertionlinkInertialOffset, 
                                        Vector3d(00.00029, 00.00147, 00.00159));
    const Body toollinkBody          = Body(01.907, v_toollinkInertialOffset, 
                                        Vector3d(00.04569, 00.04553, 00.00169));
    const Body pitchfrontlinkBody    = Body(01.607, v_pitchfrontlinkInertialOffset, 
                                        Vector3d(00.09829, 00.01747, 00.10993));
    const Body pitchtoplinkBody      = Body(00.439, v_pitchtoplinkInertialOffset, 
                                        Vector3d(00.00030, 00.03813, 00.03812));
      
  const Vector3d v_baselink_yawlink_ax_jINp               = { 0.0, 0.0, 1.0 };
  const Vector3d v_yawlink_pitchbacklink_ax_jINp          = { 0.0, 0.0, 1.0 };
  const Vector3d v_pitchbacklink_pitchbottomlink_ax_jINp  = { 0.0, 0.0, 1.0 };
  const Vector3d v_pitchbottomlink_pitchendlink_ax_jINp   = { 0.0, 0.0, 1.0 };
  const Vector3d v_yawlink_pitchfrontlink_ax_jINp         = { 0.0, 0.0, 1.0 };
  const Vector3d v_pitchfrontlink_pitchbottomlink_ax_jINp = { 0.0, 0.0, 1.0 };
  const Vector3d v_pitchfrontlink_pitchtoplink_ax_jINp    = { 0.0, 0.0, 1.0 };
  const Vector3d v_pitchtoplink_pitchendlink_ax_jINp      = { 0.0, 0.0, 1.0 };
  const Vector3d v_pitchendlink_maininsertionlink_ax_jINp = { 1.0, 0.0, 0.0 };
  const Vector3d v_maininsertionlink_toollink_ax_jINp     = { 0.0, 0.0, 1.0 };
  //-----------------------------------------------------//
  Vector3d v_baselink_yawlink_ap               = { -0.0000, -1.0000, 00.0000 };
  Vector3d v_baselink_yawlink_ac               = { 00.0000, 00.0000, -1.0000 };
  Vector3d v_baselink_yawlinkPP               = { 00.0000, 00.0000, 00.0000 };
  Vector3d v_baselink_yawlinkCP               = { 00.0000, 00.0000, 00.5369 };
  //-----------------------------------------------------//
  Vector3d v_yawlink_pitchbacklink_ap          = { 1.0,     0.0,    0.0 };
  Vector3d v_yawlink_pitchbacklink_ac          = { 0.0,     0.0,    1.0 };
  Vector3d v_yawlink_pitchbacklinkPP          = { 0.0, -0.0000, 0.1624 };
  Vector3d v_yawlink_pitchbacklinkCP          = { 0.0,     0.0,    0.0 };

  Vector3d v_pitchbacklink_pitchbottomlink_ap  = {     0.0,     0.0,     1.0 };
  Vector3d v_pitchbacklink_pitchbottomlink_ac  = {     0.0,     0.0,     1.0 };
  Vector3d v_pitchbacklink_pitchbottomlinkPP  = { -0.1028, -0.2867,     0.0 };
  Vector3d v_pitchbacklink_pitchbottomlinkCP  = { -0.0364,  0.0000, -0.0000 };

  Vector3d v_pitchbottomlink_pitchendlink_ap   = {    0.0,     0.0,     1.0 };
  Vector3d v_pitchbottomlink_pitchendlink_ac   = {    0.0,     0.0,     1.0 };
  Vector3d v_pitchbottomlink_pitchendlinkPP   = { 0.3401, -0.0000, -0.0000 };
  Vector3d v_pitchbottomlink_pitchendlinkCP   = {    0.0,     0.0,  0.0000 };
  //-----------------------------------------------------//
  Vector3d v_yawlink_pitchfrontlink_ap           = { 1.0,     0.0,    0.0 };
  Vector3d v_yawlink_pitchfrontlink_ac           = { 0.0,     0.0,    1.0 };
  Vector3d v_yawlink_pitchfrontlinkPP           = { 0.0,     0.0,    0.2 };
  Vector3d v_yawlink_pitchfrontlinkCP           = { 0.0,     0.0,    0.0 };

  Vector3d v_pitchfrontlink_pitchbottomlink_ap   = {     0.0,     0.0,     1.0 };
  Vector3d v_pitchfrontlink_pitchbottomlink_ac   = {     0.0,     0.0,     1.0 };
  Vector3d v_pitchfrontlink_pitchbottomlinkPP   = { -0.1031, -0.2868,     0.0 };
  Vector3d v_pitchfrontlink_pitchbottomlinkCP   = { -0.0000, -0.0000, -0.0000 };

  Vector3d v_pitchfrontlink_pitchtoplink_ap  = {     0.0,     0.0,     1.0 };
  Vector3d v_pitchfrontlink_pitchtoplink_ac  = {     0.0,     0.0,     1.0 };
  Vector3d v_pitchfrontlink_pitchtoplinkPP  = { -0.1084, -0.3242,     0.0 };
  Vector3d v_pitchfrontlink_pitchtoplinkCP  = { -0.0000, -0.0000, -0.0000 };

  Vector3d v_pitchtoplink_pitchendlink_ap  = {     0.0,     0.0,     1.0 };
  Vector3d v_pitchtoplink_pitchendlink_ac  = {     0.0,     0.0,     1.0 };
  Vector3d v_pitchtoplink_pitchendlinkPP  = {  0.3404, -0.0000, -0.000 };
  Vector3d v_pitchtoplink_pitchendlinkCP  = { -0.0051, -0.0376,  0.000 };
  //-----------------------------------------------------//
  Vector3d v_pitchendlink_maininsertionlink_ap = {     0.0,     1.0,    0.0 };
  Vector3d v_pitchendlink_maininsertionlink_ac = {     1.0,     0.0,    0.0 };
  Vector3d v_pitchendlink_maininsertionlinkPP = {  0.1031, -0.0961, 0.0000 };
  Vector3d v_pitchendlink_maininsertionlinkCP = { -0.0108,  -0.062, 0.0000 };

  Vector3d v_maininsertionlink_toollink_ap     = {     1.0,     0.0,    0.0 };
  Vector3d v_maininsertionlink_toollink_ac     = {     0.0,     0.0,   -1.0 };
  Vector3d v_maininsertionlink_toollinkPP     = { -0.0108,  -0.062,    0.0 };
  Vector3d v_maininsertionlink_toollinkCP     = { -0.0000, -0.0000, 0.0118 };
  //-----------------------------------------------------//
  const double q_ROOT_baselink_childOffset                  = 0.0;
  const double q_baselink_yawlink_childOffset               = -3.1414;
  const double q_yawlink_pitchbacklink_childOffset          = 3.1416;
  const double q_pitchbacklink_pitchbottomlink_childOffset  = 0.0;
  const double q_baselink_pitchendlink_childOffset          = 1.56304;
  const double q_pitchendlink_maininsertionlink_childOffset = 0.0;
  const double q_maininsertionlink_toollink_childOffset     = -1.5708;
  const double q_pitchbottomlink_pitchendlink_childOffset   = 0.0;
  const double q_yawlink_pitchfrontlink_childOffset         = 3.1416;
  const double q_pitchfrontlink_pitchbottomlink_childOffset = 0.0;
  const double q_pitchfrontlink_pitchtoplink_childOffset    = 0.0;
  const double q_pitchtoplink_pitchendlink_childOffset      = 0.0;
  const double q_toollink_ee_childOffset                    = 0.0;


  const double q_ROOT_baselink_jointOffset                  = 0.0;
  const double q_baselink_yawlink_jointOffset               = 0.0;
  const double q_yawlink_pitchbacklink_jointOffset          = 0.0;
  const double q_pitchbacklink_pitchbottomlink_jointOffset  = 0.0;
  const double q_baselink_pitchendlink_jointOffset          = 0.0;
  const double q_pitchendlink_maininsertionlink_jointOffset = 0.0;
  const double q_maininsertionlink_toollink_jointOffset     = 0.0;
  const double q_pitchbottomlink_pitchendlink_jointOffset   = 0.0;
  const double q_yawlink_pitchfrontlink_jointOffset         = 0.0;
  const double q_pitchfrontlink_pitchbottomlink_jointOffset = 0.0;
  const double q_pitchfrontlink_pitchtoplink_jointOffset    = 0.0;
  const double q_pitchtoplink_pitchendlink_jointOffset      = 0.0;
  const double q_toollink_ee_jointOffset                    = 0.0;

  const float baselinkScale{1.0f}, yawlinkScale{1.0f}, pitchbacklinkScale{1.0f}, pitchbottomlinkScale{1.0f}, 
    pitchendlinkScale{1.0f}, maininsertionlinkScale{1.0f}, toollinkScale{1.0f}, pitchfrontlinkScale{1.0f}, 
    pitchtoplinkScale{1.0f};

  Vector3d v_baselink_yawlinkJAxis, v_yawlink_pitchbacklinkJAxis, 
    v_pitchbacklink_pitchbottomlinkJAxis, v_pitchbottomlink_pitchendlinkJAxis, 
    v_yawlink_pitchfrontlinkJAxis, v_pitchfrontlink_pitchtoplinkJAxis, 
    v_pitchtoplink_pitchendlinkJAxis, v_pitchfrontlink_pitchbottomlinkJAxis, 
    v_pitchendlink_maininsertionlinkJAxis, v_maininsertionlink_toollinkJAxis;

  unsigned int world_baselinkId{0}, baselink_yawlinkId{0}, yawlink_pitchbacklinkId{0}, yawlink_pitchfrontlinkId{0}, 
  pitchbacklink_pitchbottomlinkId{0}, pitchbacklink_pitchbottomlink_v_Id{0}, pitchbottomlink_pitchendlinkId{0}, 
  pitchendlink_maininsertionlinkId{0}, pitchfrontlink_pitchtoplinkId{0}, pitchfrontlink_pitchbottomlinkId{0}, 
  pitchtoplink_pitchendlinkId{0}, maininsertionlink_toollinkId{0};

  RigidBodyDynamics::Model *model;

  RigidBodyDynamics::Math::VectorNd Q;
  RigidBodyDynamics::Math::VectorNd QDot;
  RigidBodyDynamics::Math::VectorNd QDDot;
  RigidBodyDynamics::Math::VectorNd Tau;
};

#endif