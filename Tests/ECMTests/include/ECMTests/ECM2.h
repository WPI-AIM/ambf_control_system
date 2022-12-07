#include "rbdl/rbdl.h"
#include "Tests/rbdl_tests.h"

const double TEST_PREC = 1.0e-3;
const double TEST_LAX = 1.0e-5;


Quaternion EigenToRBDLQuaternion(const Eigen::Quaterniond &quat_eigen)
{
  Quaternion quat_rbdl;
  quat_rbdl.x() = quat_eigen.x();
  quat_rbdl.y() = quat_eigen.y();
  quat_rbdl.z() = quat_eigen.z();
  quat_rbdl.w() = quat_eigen.w();

  return quat_rbdl;
}

struct ECM 
{
  ECM()
  {
    /*
    Q Index
    0, baselink-yawlink
    1, yawlink-pitchbacklink
    2, pitchbacklink-pitchbottomlink
    3, pitchbottomlink-pitchendlink
    4, yawlink-pitchfrontlink
    5, pitchfrontlink-pitchtoplink
    6, pitchtoplink-pitchendlink
    7, pitchendlink-maininsertionlink
    8, maininsertionlink-toollink
    */

    Vector3d vector3d_zero = Vector3d::Zero();
    //0---------------------------------------------------------------------//
    T_W_world_baselinkST.E.setIdentity();
    T_W_world_baselinkST.r = Vector3d(0.5, -0.4, -0.6);
    //1--------------------------------------------------------------------//		
    baselink_yawlinkPP -= baselinkInertialOffsetPos;
    baselink_yawlinkCP -= yawlinkInertialOffsetPos;
    
    yawlink_pitchbacklinkPP -= yawlinkInertialOffsetPos;
    yawlink_pitchbacklinkCP -= pitchbacklinkInertialOffsetPos;

    pitchbacklink_pitchbottomlinkPP -= pitchbacklinkInertialOffsetPos;
    pitchbacklink_pitchbottomlinkCP -= pitchbottomlinkInertialOffsetPos;
  
    pitchbottomlink_pitchendlinkPP -= pitchbottomlinkInertialOffsetPos;
    pitchbottomlink_pitchendlinkCP -= pitchendlinkInertialOffsetPos;

    yawlink_pitchfrontlinkPP -= yawlinkInertialOffsetPos;
    yawlink_pitchfrontlinkCP -= pitchfrontlinkInertialOffsetPos;

    pitchfrontlink_pitchbottomlinkPP -= pitchfrontlinkInertialOffsetPos;
    pitchfrontlink_pitchbottomlinkCP -= pitchbottomlinkInertialOffsetPos;

    pitchfrontlink_pitchtoplinkPP -= pitchfrontlinkInertialOffsetPos;
    pitchfrontlink_pitchtoplinkCP -= pitchtoplinkInertialOffsetPos;

    pitchtoplink_pitchendlinkPP -= pitchtoplinkInertialOffsetPos;
    pitchtoplink_pitchendlinkCP -= pitchendlinkInertialOffsetPos;
    
    pitchendlink_maininsertionlinkPP -= pitchendlinkInertialOffsetPos;
    pitchendlink_maininsertionlinkCP -= maininsertionlinkInertialOffsetPos;

    maininsertionlink_toollinkPP -= maininsertionlinkInertialOffsetPos;
    maininsertionlink_toollinkCP -= toollinkInertialOffsetPos;
    
    // CHECK_THAT (Vector3d(0.000010, 0.614520, 0.020880), 
    //     AllCloseVector(baselink_yawlinkPP, TEST_PREC, TEST_PREC));
    // CHECK_THAT (Vector3d(0.000000, 0.016140, 0.402430),
    //     AllCloseVector(baselink_yawlinkCP, TEST_PREC, TEST_PREC));

    // CHECK_THAT (Vector3d(0.000000, 0.016140, 0.027930), 
    //     AllCloseVector(yawlink_pitchbacklinkPP, TEST_PREC, TEST_PREC));
    // CHECK_THAT (Vector3d(0.051500, 0.143430, 0.009000),
    //     AllCloseVector(yawlink_pitchbacklinkCP, TEST_PREC, TEST_PREC));

    // CHECK_THAT (Vector3d(-0.051300, -0.143270, 0.009000), 
    //     AllCloseVector(pitchbacklink_pitchbottomlinkPP, TEST_PREC, TEST_PREC));
    // CHECK_THAT (Vector3d(-0.185530, 0.018160, 0.000000),
    //     AllCloseVector(pitchbacklink_pitchbottomlinkCP, TEST_PREC, TEST_PREC));

    // CHECK_THAT (Vector3d(0.190970, 0.018160, 0.000000), 
    //     AllCloseVector(pitchbottomlink_pitchendlinkPP, TEST_PREC, TEST_PREC));
    // CHECK_THAT (Vector3d(-0.051350, -0.004820, -0.000790),
    //     AllCloseVector(pitchbottomlink_pitchendlinkCP, TEST_PREC, TEST_PREC));

    // CHECK_THAT (Vector3d(0.000000, 0.016140, 0.065530), 
    //     AllCloseVector(yawlink_pitchfrontlinkPP, TEST_PREC, TEST_PREC));
    // CHECK_THAT (Vector3d(0.036490, 0.152610, 0.000000),
    //     AllCloseVector(yawlink_pitchfrontlinkCP, TEST_PREC, TEST_PREC));

    // CHECK_THAT (Vector3d(-0.066610, -0.134190, 0.000000), 
    //     AllCloseVector(pitchfrontlink_pitchbottomlinkPP, TEST_PREC, TEST_PREC));
    // CHECK_THAT (Vector3d(-0.149130, 0.018160, 0.000000),
    //     AllCloseVector(pitchfrontlink_pitchbottomlinkCP, TEST_PREC, TEST_PREC));

    // CHECK_THAT (Vector3d(-0.071910, -0.171590, 0.000000), 
    //     AllCloseVector(pitchfrontlink_pitchtoplinkPP, TEST_PREC, TEST_PREC));
    // CHECK_THAT (Vector3d(-0.170200, 0.000070, -0.000790),
    //     AllCloseVector(pitchfrontlink_pitchtoplinkCP, TEST_PREC, TEST_PREC));
  
    // CHECK_THAT (Vector3d(0.170200, 0.000070, -0.000790), 
    //     AllCloseVector(pitchtoplink_pitchendlinkPP, TEST_PREC, TEST_PREC));
    // CHECK_THAT (Vector3d(-0.056450, -0.042420, -0.000790),
    //     AllCloseVector(pitchtoplink_pitchendlinkCP, TEST_PREC, TEST_PREC));
 
    // CHECK_THAT (Vector3d(0.051750, -0.100920, -0.000790), 
    //     AllCloseVector(pitchendlink_maininsertionlinkPP, TEST_PREC, TEST_PREC));
    // CHECK_THAT (Vector3d(0.048200, -0.045500, -0.000790),
    //     AllCloseVector(pitchendlink_maininsertionlinkCP, TEST_PREC, TEST_PREC)); 

    // CHECK_THAT (Vector3d(0.048200, -0.045500, -0.000790), 
    //     AllCloseVector(maininsertionlink_toollinkPP, TEST_PREC, TEST_PREC));
    // CHECK_THAT (Vector3d(0.000000, 0.000810, 0.084120),
    //     AllCloseVector(maininsertionlink_toollinkCP, TEST_PREC, TEST_PREC));
    //1--------------------------------------------------------------------//

    // CHECK_THAT (Vector3d(0.000000, -1.000000, -0.000000), 
    //     AllCloseVector(baselink_yawlinkPA, TEST_PREC, TEST_PREC));
    // CHECK_THAT (Vector3d(0.000000, -0.000000, -1.000000),
    //     AllCloseVector(baselink_yawlinkCA, TEST_PREC, TEST_PREC));

    // CHECK_THAT (Vector3d(1.000000, 0.000000, 0.000000), 
    //     AllCloseVector(yawlink_pitchbacklinkPA, TEST_PREC, TEST_PREC));
    // CHECK_THAT (Vector3d(0.000000, 0.000000, 1.000000),
    //     AllCloseVector(yawlink_pitchbacklinkCA, TEST_PREC, TEST_PREC));

    // CHECK_THAT (Vector3d(0.000000, 0.000000, 1.000000), 
    //     AllCloseVector(pitchbacklink_pitchbottomlinkPA, TEST_PREC, TEST_PREC));
    // CHECK_THAT (Vector3d(0.000000, 0.000000, 1.000000),
    //     AllCloseVector(pitchbacklink_pitchbottomlinkCA, TEST_PREC, TEST_PREC));

    // CHECK_THAT (Vector3d(0.000000, 0.000000, 1.000000), 
    //     AllCloseVector(pitchbottomlink_pitchendlinkPA, TEST_PREC, TEST_PREC));
    // CHECK_THAT (Vector3d(0.000000, 0.000000, 1.000000),
    //     AllCloseVector(pitchbottomlink_pitchendlinkCA, TEST_PREC, TEST_PREC));

    // CHECK_THAT (Vector3d(1.000000, 0.000000, 0.000000), 
    //     AllCloseVector(yawlink_pitchfrontlinkPA, TEST_PREC, TEST_PREC));
    // CHECK_THAT (Vector3d(0.000000, 0.000000, 1.000000),
    //     AllCloseVector(yawlink_pitchfrontlinkCA, TEST_PREC, TEST_PREC));

    // CHECK_THAT (Vector3d(0.000000, 0.000000, 1.000000), 
    //     AllCloseVector(pitchfrontlink_pitchbottomlinkPA, TEST_PREC, TEST_PREC));
    // CHECK_THAT (Vector3d(0.000000, 0.000000, 1.000000),
    //     AllCloseVector(pitchfrontlink_pitchbottomlinkCA, TEST_PREC, TEST_PREC));

    // CHECK_THAT (Vector3d(0.000000, 0.000000, 1.000000), 
    //     AllCloseVector(pitchfrontlink_pitchtoplinkPA, TEST_PREC, TEST_PREC));
    // CHECK_THAT (Vector3d(0.000000, 0.000000, 1.000000),
    //     AllCloseVector(pitchfrontlink_pitchtoplinkCA, TEST_PREC, TEST_PREC));
  
    // CHECK_THAT (Vector3d(0.000000, 0.000000, 1.000000), 
    //     AllCloseVector(pitchtoplink_pitchendlinkPA, TEST_PREC, TEST_PREC));
    // CHECK_THAT (Vector3d(0.000000, 0.000000, 1.000000),
    //     AllCloseVector(pitchtoplink_pitchendlinkCA, TEST_PREC, TEST_PREC));
 
    // CHECK_THAT (Vector3d(0.000000, 1.000000, 0.000000), 
    //     AllCloseVector(pitchendlink_maininsertionlinkPA, TEST_PREC, TEST_PREC));
    // CHECK_THAT (Vector3d(1.000000, 0.000000, 0.000000),
    //     AllCloseVector(pitchendlink_maininsertionlinkCA, TEST_PREC, TEST_PREC)); 

    // CHECK_THAT (Vector3d(1.000000, 0.000000, 0.000000), 
    //     AllCloseVector(maininsertionlink_toollinkPA, TEST_PREC, TEST_PREC));
    // CHECK_THAT (Vector3d(0.000000, 0.000000, -1.000000),
    //     AllCloseVector(maininsertionlink_toollinkCA, TEST_PREC, TEST_PREC));
    //1--------------------------------------------------------------------//
    const Vector3d baselink_yawlink_ax_jINp               = { 0.0, 0.0, 1.0 };
    const Vector3d yawlink_pitchbacklink_ax_jINp          = { 0.0, 0.0, 1.0 };
    const Vector3d pitchbacklink_pitchbottomlink_ax_jINp  = { 0.0, 0.0, 1.0 };
    const Vector3d pitchbottomlink_pitchendlink_ax_jINp   = { 0.0, 0.0, 1.0 };
    const Vector3d yawlink_pitchfrontlink_ax_jINp         = { 0.0, 0.0, 1.0 };
    const Vector3d pitchfrontlink_pitchbottomlink_ax_jINp = { 0.0, 0.0, 1.0 };
    const Vector3d pitchfrontlink_pitchtoplink_ax_jINp    = { 0.0, 0.0, 1.0 };
    const Vector3d pitchtoplink_pitchendlink_ax_jINp      = { 0.0, 0.0, 1.0 };
    const Vector3d pitchendlink_maininsertionlink_ax_jINp = { 1.0, 0.0, 0.0 };
    const Vector3d maininsertionlink_toollink_ax_jINp     = { 0.0, 0.0, 1.0 };


    Eigen::Quaternion baselink_yawlink_quat_jINp_eigen =
    Eigen::Quaterniond::FromTwoVectors(baselink_yawlink_ax_jINp, baselink_yawlinkPA);
    Quaternion baselink_yawlink_quat_jINp = EigenToRBDLQuaternion(baselink_yawlink_quat_jINp_eigen);

    Eigen::Quaternion yawlink_pitchbacklink_quat_jINp_eigen =
    Eigen::Quaterniond::FromTwoVectors(yawlink_pitchbacklink_ax_jINp, yawlink_pitchbacklinkPA);
    Quaternion yawlink_pitchbacklink_quat_jINp = EigenToRBDLQuaternion(yawlink_pitchbacklink_quat_jINp_eigen);

    Eigen::Quaternion pitchbacklink_pitchbottomlink_quat_jINp_eigen =
    Eigen::Quaterniond::FromTwoVectors(pitchbacklink_pitchbottomlink_ax_jINp, pitchbacklink_pitchbottomlinkPA);
    Quaternion pitchbacklink_pitchbottomlink_quat_jINp = EigenToRBDLQuaternion(pitchbacklink_pitchbottomlink_quat_jINp_eigen);

    Eigen::Quaternion pitchbottomlink_pitchendlink_quat_jINp_eigen =
    Eigen::Quaterniond::FromTwoVectors(pitchbottomlink_pitchendlink_ax_jINp, pitchbottomlink_pitchendlinkPA);
    Quaternion pitchbottomlink_pitchendlink_quat_jINp = 
      EigenToRBDLQuaternion(pitchbottomlink_pitchendlink_quat_jINp_eigen);

    Eigen::Quaternion yawlink_pitchfrontlink_quat_jINp_eigen =
    Eigen::Quaterniond::FromTwoVectors(yawlink_pitchfrontlink_ax_jINp, yawlink_pitchfrontlinkPA);
    Quaternion yawlink_pitchfrontlink_quat_jINp = 
      EigenToRBDLQuaternion(yawlink_pitchfrontlink_quat_jINp_eigen);

    Eigen::Quaternion pitchfrontlink_pitchbottomlink_quat_jINp_eigen =
    Eigen::Quaterniond::FromTwoVectors(pitchfrontlink_pitchbottomlink_ax_jINp, pitchfrontlink_pitchbottomlinkPA);
    Quaternion pitchfrontlink_pitchbottomlink_quat_jINp = 
      EigenToRBDLQuaternion(pitchfrontlink_pitchbottomlink_quat_jINp_eigen);

    Eigen::Quaternion pitchfrontlink_pitchtoplink_quat_jINp_eigen =
    Eigen::Quaterniond::FromTwoVectors(pitchfrontlink_pitchtoplink_ax_jINp, pitchfrontlink_pitchtoplinkPA);
    Quaternion pitchfrontlink_pitchtoplink_quat_jINp = 
      EigenToRBDLQuaternion(pitchfrontlink_pitchtoplink_quat_jINp_eigen);

    Eigen::Quaternion pitchtoplink_pitchendlink_quat_jINp_eigen =
    Eigen::Quaterniond::FromTwoVectors(pitchtoplink_pitchendlink_ax_jINp, pitchtoplink_pitchendlinkPA);
    Quaternion pitchtoplink_pitchendlink_quat_jINp = 
      EigenToRBDLQuaternion(pitchtoplink_pitchendlink_quat_jINp_eigen);

    Eigen::Quaternion pitchendlink_maininsertionlink_quat_jINp_eigen =
    Eigen::Quaterniond::FromTwoVectors(pitchendlink_maininsertionlink_ax_jINp, pitchendlink_maininsertionlinkPA);
    Quaternion pitchendlink_maininsertionlink_quat_jINp = 
      EigenToRBDLQuaternion(pitchendlink_maininsertionlink_quat_jINp_eigen);

    Eigen::Quaternion maininsertionlink_toollink_quat_jINp_eigen =
    Eigen::Quaterniond::FromTwoVectors(maininsertionlink_toollink_ax_jINp, maininsertionlink_toollinkPA);
    Quaternion maininsertionlink_toollink_quat_jINp = 
      EigenToRBDLQuaternion(maininsertionlink_toollink_quat_jINp_eigen);

    // CHECK_THAT (Quaternion(0.707107, 0.000000, -0.000000, 0.707107), 
    //   AllCloseVector(baselink_yawlink_quat_jINp, TEST_PREC, TEST_PREC));
    // CHECK_THAT (Quaternion(0.000000, 0.707107, 0.000000, 0.707107), 
    //   AllCloseVector(yawlink_pitchbacklink_quat_jINp, TEST_PREC, TEST_PREC));
    // CHECK_THAT (Quaternion(0.000000, 0.000000, 0.000000, 1.000000), 
    //   AllCloseVector(pitchbacklink_pitchbottomlink_quat_jINp, TEST_PREC, TEST_PREC));
    // CHECK_THAT (Quaternion(0.000000, 0.000000, 0.000000, 1.000000), 
    //   AllCloseVector(pitchbottomlink_pitchendlink_quat_jINp, TEST_PREC, TEST_PREC));
    // CHECK_THAT (Quaternion(0.000000, 0.707107, 0.000000, 0.707107), 
    //   AllCloseVector(yawlink_pitchfrontlink_quat_jINp, TEST_PREC, TEST_PREC));
    // CHECK_THAT (Quaternion(0.000000, 0.000000, 0.000000, 1.000000), 
    //   AllCloseVector(pitchfrontlink_pitchbottomlink_quat_jINp, TEST_PREC, TEST_PREC));
    // CHECK_THAT (Quaternion(0.000000, 0.000000, 0.000000, 1.000000), 
    //   AllCloseVector(pitchfrontlink_pitchtoplink_quat_jINp, TEST_PREC, TEST_PREC));
    // CHECK_THAT (Quaternion(0.000000, 0.000000, 0.000000, 1.000000), 
    //   AllCloseVector(pitchtoplink_pitchendlink_quat_jINp, TEST_PREC, TEST_PREC));
    // CHECK_THAT (Quaternion(0.000000, 0.000000, 0.707107, 0.707107), 
    //   AllCloseVector(pitchendlink_maininsertionlink_quat_jINp, TEST_PREC, TEST_PREC));
    // CHECK_THAT (Quaternion(0.000000, 0.707107, 0.000000, 0.707107), 
    //   AllCloseVector(maininsertionlink_toollink_quat_jINp, TEST_PREC, TEST_PREC));
    //1-------------------------------------
    
    // Eigen::Quaterniond baselink_yawlink_quat_jOffINp_eigen =
    // Eigen::Quaterniond{Eigen::AngleAxisd{baselink_yawlinkOffsetQ, baselink_yawlinkPA}};
    // Quaternion baselink_yawlink_quat_jOffINp = EigenToRBDLQuaternion(baselink_yawlink_quat_jOffINp_eigen);
    // CHECK_THAT (Quaternion((0.000000, 0.000000, 0.000000, 1.000000), 
    //   AllCloseVector(baselink_yawlink_quat_jOffINp, TEST_PREC, TEST_PREC));

    Eigen::Quaternion baselink_yawlink_quat_jOffINp_eigen =
    Eigen::Quaterniond(Eigen::AngleAxisd(baselink_yawlink_jointOffset, baselink_yawlinkPA));
    Quaternion baselink_yawlink_quat_jOffINp = EigenToRBDLQuaternion(baselink_yawlink_quat_jOffINp_eigen);

    Eigen::Quaternion yawlink_pitchbacklink_quat_jOffINp_eigen =
    Eigen::Quaterniond(Eigen::AngleAxisd(yawlink_pitchbacklink_jointOffset, yawlink_pitchbacklinkPA));
    Quaternion yawlink_pitchbacklink_quat_jOffINp = EigenToRBDLQuaternion(yawlink_pitchbacklink_quat_jOffINp_eigen);

    Eigen::Quaternion pitchbacklink_pitchbottomlink_quat_jOffINp_eigen =
    Eigen::Quaterniond(Eigen::AngleAxisd(pitchbacklink_pitchbottomlink_jointOffset, pitchbacklink_pitchbottomlinkPA));
    Quaternion pitchbacklink_pitchbottomlink_quat_jOffINp = EigenToRBDLQuaternion(pitchbacklink_pitchbottomlink_quat_jOffINp_eigen);

    Eigen::Quaternion pitchbottomlink_pitchendlink_quat_jOffINp_eigen =
    Eigen::Quaterniond(Eigen::AngleAxisd(pitchbottomlink_pitchendlink_jointOffset, pitchbottomlink_pitchendlinkPA));
    Quaternion pitchbottomlink_pitchendlink_quat_jOffINp = EigenToRBDLQuaternion(pitchbottomlink_pitchendlink_quat_jOffINp_eigen);

    Eigen::Quaternion maininsertionlink_toollink_quat_jOffINp_eigen =
    Eigen::Quaterniond(Eigen::AngleAxisd(maininsertionlink_toollink_jointOffset, maininsertionlink_toollinkPA));
    Quaternion maininsertionlink_toollink_quat_jOffINp = EigenToRBDLQuaternion(maininsertionlink_toollink_quat_jOffINp_eigen);

    Eigen::Quaternion yawlink_pitchfrontlink_quat_jOffINp_eigen =
    Eigen::Quaterniond(Eigen::AngleAxisd(yawlink_pitchfrontlink_jointOffset, yawlink_pitchfrontlinkPA));
    Quaternion yawlink_pitchfrontlink_quat_jOffINp = EigenToRBDLQuaternion(yawlink_pitchfrontlink_quat_jOffINp_eigen);

    Eigen::Quaternion pitchfrontlink_pitchbottomlink_quat_jOffINp_eigen =
    Eigen::Quaterniond(Eigen::AngleAxisd(pitchfrontlink_pitchbottomlink_jointOffset, pitchfrontlink_pitchbottomlinkPA));
    Quaternion pitchfrontlink_pitchbottomlink_quat_jOffINp = EigenToRBDLQuaternion(pitchfrontlink_pitchbottomlink_quat_jOffINp_eigen);

    Eigen::Quaternion pitchfrontlink_pitchtoplink_quat_jOffINp_eigen =
    Eigen::Quaterniond(Eigen::AngleAxisd(pitchfrontlink_pitchtoplink_jointOffset, pitchfrontlink_pitchtoplinkPA));
    Quaternion pitchfrontlink_pitchtoplink_quat_jOffINp = EigenToRBDLQuaternion(pitchfrontlink_pitchtoplink_quat_jOffINp_eigen);

    Eigen::Quaternion pitchtoplink_pitchendlink_quat_jOffINp_eigen =
    Eigen::Quaterniond(Eigen::AngleAxisd(pitchtoplink_pitchendlink_jointOffset, pitchtoplink_pitchendlinkPA));
    Quaternion pitchtoplink_pitchendlink_quat_jOffINp = EigenToRBDLQuaternion(pitchtoplink_pitchendlink_quat_jOffINp_eigen);

    Eigen::Quaternion pitchendlink_maininsertionlink_quat_jOffINp_eigen =
    Eigen::Quaterniond(Eigen::AngleAxisd(pitchendlink_maininsertionlink_jointOffset, pitchendlink_maininsertionlinkPA));
    Quaternion pitchendlink_maininsertionlink_quat_jOffINp = EigenToRBDLQuaternion(pitchendlink_maininsertionlink_quat_jOffINp_eigen);


    // CHECK_THAT (Quaternion(0.000000, 0.000000, 0.000000, 1.000000), 
    //   AllCloseVector(baselink_yawlink_quat_jOffINp, TEST_PREC, TEST_PREC));
    // CHECK_THAT (Quaternion(0.000000, 0.000000, 0.000000, 1.000000), 
    //   AllCloseVector(yawlink_pitchbacklink_quat_jOffINp, TEST_PREC, TEST_PREC));
    // CHECK_THAT (Quaternion(0.000000, 0.000000, 0.000000, 1.000000), 
    //   AllCloseVector(pitchbacklink_pitchbottomlink_quat_jOffINp, TEST_PREC, TEST_PREC));
    // CHECK_THAT (Quaternion(0.000000, 0.000000, 0.000000, 1.000000), 
    //   AllCloseVector(pitchbottomlink_pitchendlink_quat_jOffINp, TEST_PREC, TEST_PREC));
    // CHECK_THAT (Quaternion(0.000000, 0.000000, 0.000000, 1.000000), 
    //   AllCloseVector(yawlink_pitchfrontlink_quat_jOffINp, TEST_PREC, TEST_PREC));
    // CHECK_THAT (Quaternion(0.000000, 0.000000, 0.000000, 1.000000), 
    //   AllCloseVector(pitchfrontlink_pitchbottomlink_quat_jOffINp, TEST_PREC, TEST_PREC));
    // CHECK_THAT (Quaternion(0.000000, 0.000000, 0.000000, 1.000000), 
    //   AllCloseVector(pitchfrontlink_pitchtoplink_quat_jOffINp, TEST_PREC, TEST_PREC));
    // CHECK_THAT (Quaternion(0.000000, 0.000000, 0.000000, 1.000000), 
    //   AllCloseVector(pitchtoplink_pitchendlink_quat_jOffINp, TEST_PREC, TEST_PREC));
    // CHECK_THAT (Quaternion(0.000000, 0.000000, 0.000000, 1.000000), 
    //   AllCloseVector(pitchendlink_maininsertionlink_quat_jOffINp, TEST_PREC, TEST_PREC));
    // CHECK_THAT (Quaternion(0.000000, 0.000000, 0.000000, 1.000000), 
    //   AllCloseVector(maininsertionlink_toollink_quat_jOffINp, TEST_PREC, TEST_PREC));
    //1-------------------------------------
    Eigen::Quaternion baselink_yawlinkFP_quat_eigen = 
      baselink_yawlink_quat_jOffINp_eigen * baselink_yawlink_quat_jINp_eigen;
    Quaternion baselink_yawlinkFP_quat = EigenToRBDLQuaternion(baselink_yawlinkFP_quat_eigen);

    Eigen::Quaternion yawlink_pitchbacklinkFP_quat_eigen = 
      yawlink_pitchbacklink_quat_jOffINp_eigen * yawlink_pitchbacklink_quat_jINp_eigen;
    Quaternion yawlink_pitchbacklinkFP_quat = EigenToRBDLQuaternion(yawlink_pitchbacklinkFP_quat_eigen);

    Eigen::Quaternion pitchbacklink_pitchbottomlinkFP_quat_eigen = 
      pitchbacklink_pitchbottomlink_quat_jOffINp_eigen * pitchbacklink_pitchbottomlink_quat_jINp_eigen;
    Quaternion pitchbacklink_pitchbottomlinkFP_quat = EigenToRBDLQuaternion(pitchbacklink_pitchbottomlinkFP_quat_eigen);

    Eigen::Quaternion pitchbottomlink_pitchendlinkFP_quat_eigen = 
      pitchbottomlink_pitchendlink_quat_jOffINp_eigen * pitchbottomlink_pitchendlink_quat_jINp_eigen;
    Quaternion pitchbottomlink_pitchendlinkFP_quat = EigenToRBDLQuaternion(pitchbottomlink_pitchendlinkFP_quat_eigen);

    Eigen::Quaternion pitchendlink_maininsertionlinkFP_quat_eigen = 
      pitchendlink_maininsertionlink_quat_jOffINp_eigen * pitchendlink_maininsertionlink_quat_jINp_eigen;
    Quaternion pitchendlink_maininsertionlinkFP_quat = EigenToRBDLQuaternion(pitchendlink_maininsertionlinkFP_quat_eigen);

    Eigen::Quaternion maininsertionlink_toollinkFP_quat_eigen = 
      maininsertionlink_toollink_quat_jOffINp_eigen * maininsertionlink_toollink_quat_jINp_eigen;
    Quaternion maininsertionlink_toollinkFP_quat = EigenToRBDLQuaternion(maininsertionlink_toollinkFP_quat_eigen);

    Eigen::Quaternion yawlink_pitchfrontlinkFP_quat_eigen = 
      yawlink_pitchfrontlink_quat_jOffINp_eigen * yawlink_pitchfrontlink_quat_jINp_eigen;
    Quaternion yawlink_pitchfrontlinkFP_quat = EigenToRBDLQuaternion(yawlink_pitchfrontlinkFP_quat_eigen);

    Eigen::Quaternion pitchfrontlink_pitchbottomlinkFP_quat_eigen = 
      pitchfrontlink_pitchbottomlink_quat_jOffINp_eigen * pitchfrontlink_pitchbottomlink_quat_jINp_eigen;
    Quaternion pitchfrontlink_pitchbottomlinkFP_quat = EigenToRBDLQuaternion(pitchfrontlink_pitchbottomlinkFP_quat_eigen);

    Eigen::Quaternion pitchfrontlink_pitchtoplinkFP_quat_eigen = 
      pitchfrontlink_pitchtoplink_quat_jOffINp_eigen * pitchfrontlink_pitchtoplink_quat_jINp_eigen;
    Quaternion pitchfrontlink_pitchtoplinkFP_quat = EigenToRBDLQuaternion(pitchfrontlink_pitchtoplinkFP_quat_eigen);

    Eigen::Quaternion pitchtoplink_pitchendlinkFP_quat_eigen = 
      pitchtoplink_pitchendlink_quat_jOffINp_eigen * pitchtoplink_pitchendlink_quat_jINp_eigen;
    Quaternion pitchtoplink_pitchendlinkFP_quat = EigenToRBDLQuaternion(pitchtoplink_pitchendlinkFP_quat_eigen);

    // CHECK_THAT (Quaternion(0.707107, 0.000000, 0.000000, 0.707107), 
    //   AllCloseVector(baselink_yawlinkFP_quat, TEST_PREC, TEST_PREC));
    // CHECK_THAT (Quaternion(0.000000, 0.707107, 0.000000, 0.707107), 
    //   AllCloseVector(yawlink_pitchbacklinkFP_quat, TEST_PREC, TEST_PREC));
    // CHECK_THAT (Quaternion(0.000000, 0.000000, 0.000000, 1.000000), 
    //   AllCloseVector(pitchbacklink_pitchbottomlinkFP_quat, TEST_PREC, TEST_PREC));
    // CHECK_THAT (Quaternion(0.000000, 0.000000, 0.000000, 1.000000), 
    //   AllCloseVector(pitchbottomlink_pitchendlinkFP_quat, TEST_PREC, TEST_PREC));
    // CHECK_THAT (Quaternion(0.000000, 0.000000, 0.707107, 0.707107), 
    //   AllCloseVector(pitchendlink_maininsertionlinkFP_quat, TEST_PREC, TEST_PREC));
    // CHECK_THAT (Quaternion(0.000000, 0.707107, 0.000000, 0.707107), 
    //   AllCloseVector(maininsertionlink_toollinkFP_quat, TEST_PREC, TEST_PREC));
    // CHECK_THAT (Quaternion(0.000000, 0.707107, 0.000000, 0.707107), 
    //   AllCloseVector(yawlink_pitchfrontlinkFP_quat, TEST_PREC, TEST_PREC));
    // CHECK_THAT (Quaternion(0.000000, 0.000000, 0.000000, 1.000000), 
    //   AllCloseVector(pitchfrontlink_pitchbottomlinkFP_quat, TEST_PREC, TEST_PREC));
    // CHECK_THAT (Quaternion(0.000000, 0.000000, 0.000000, 1.000000), 
    //   AllCloseVector(pitchfrontlink_pitchtoplinkFP_quat, TEST_PREC, TEST_PREC));
    // CHECK_THAT (Quaternion(0.000000, 0.000000, 0.000000, 1.000000), 
    //   AllCloseVector(pitchtoplink_pitchendlinkFP_quat, TEST_PREC, TEST_PREC));

    Eigen::Quaternion baselink_yawlink_quat_cINp_eigen =
    Eigen::Quaterniond(Eigen::Quaterniond::FromTwoVectors(baselink_yawlinkCA, baselink_yawlinkPA));
    Quaternion baselink_yawlink_quat_cINp = EigenToRBDLQuaternion(baselink_yawlink_quat_cINp_eigen);

    Eigen::Quaternion yawlink_pitchbacklink_quat_cINp_eigen =
    Eigen::Quaterniond(Eigen::Quaterniond::FromTwoVectors(yawlink_pitchbacklinkCA, yawlink_pitchbacklinkPA));
    Quaternion yawlink_pitchbacklink_quat_cINp = EigenToRBDLQuaternion(yawlink_pitchbacklink_quat_cINp_eigen);

    Eigen::Quaternion pitchbacklink_pitchbottomlink_quat_cINp_eigen =
    Eigen::Quaterniond(Eigen::Quaterniond::FromTwoVectors(pitchbacklink_pitchbottomlinkCA, pitchbacklink_pitchbottomlinkPA));
    Quaternion pitchbacklink_pitchbottomlink_quat_cINp = EigenToRBDLQuaternion(pitchbacklink_pitchbottomlink_quat_cINp_eigen);

    Eigen::Quaternion pitchbottomlink_pitchendlink_quat_cINp_eigen =
    Eigen::Quaterniond(Eigen::Quaterniond::FromTwoVectors(pitchbottomlink_pitchendlinkCA, pitchbottomlink_pitchendlinkPA));
    Quaternion pitchbottomlink_pitchendlink_quat_cINp = EigenToRBDLQuaternion(pitchbottomlink_pitchendlink_quat_cINp_eigen);

    Eigen::Quaternion maininsertionlink_toollink_quat_cINp_eigen =
    Eigen::Quaterniond(Eigen::Quaterniond::FromTwoVectors(maininsertionlink_toollinkCA, maininsertionlink_toollinkPA));
    Quaternion maininsertionlink_toollink_quat_cINp = EigenToRBDLQuaternion(maininsertionlink_toollink_quat_cINp_eigen);

    Eigen::Quaternion yawlink_pitchfrontlink_quat_cINp_eigen =
    Eigen::Quaterniond(Eigen::Quaterniond::FromTwoVectors(yawlink_pitchfrontlinkCA, yawlink_pitchfrontlinkPA));
    Quaternion yawlink_pitchfrontlink_quat_cINp = EigenToRBDLQuaternion(yawlink_pitchfrontlink_quat_cINp_eigen);

    Eigen::Quaternion pitchfrontlink_pitchbottomlink_quat_cINp_eigen =
    Eigen::Quaterniond(Eigen::Quaterniond::FromTwoVectors(pitchfrontlink_pitchbottomlinkCA, pitchfrontlink_pitchbottomlinkPA));
    Quaternion pitchfrontlink_pitchbottomlink_quat_cINp = EigenToRBDLQuaternion(pitchfrontlink_pitchbottomlink_quat_cINp_eigen);

    Eigen::Quaternion pitchfrontlink_pitchtoplink_quat_cINp_eigen =
    Eigen::Quaterniond(Eigen::Quaterniond::FromTwoVectors(pitchfrontlink_pitchtoplinkCA, pitchfrontlink_pitchtoplinkPA));
    Quaternion pitchfrontlink_pitchtoplink_quat_cINp = EigenToRBDLQuaternion(pitchfrontlink_pitchtoplink_quat_cINp_eigen);

    Eigen::Quaternion pitchtoplink_pitchendlink_quat_cINp_eigen =
    Eigen::Quaterniond(Eigen::Quaterniond::FromTwoVectors(pitchtoplink_pitchendlinkCA, pitchtoplink_pitchendlinkPA));
    Quaternion pitchtoplink_pitchendlink_quat_cINp = EigenToRBDLQuaternion(pitchtoplink_pitchendlink_quat_cINp_eigen);

    Eigen::Quaternion pitchendlink_maininsertionlink_quat_cINp_eigen =
    Eigen::Quaterniond(Eigen::Quaterniond::FromTwoVectors(pitchendlink_maininsertionlinkCA, pitchendlink_maininsertionlinkPA));
    Quaternion pitchendlink_maininsertionlink_quat_cINp = EigenToRBDLQuaternion(pitchendlink_maininsertionlink_quat_cINp_eigen);

    // CHECK_THAT (Quaternion(-0.707107, 0.000000, 0.000000, 0.707107), 
    //   AllCloseVector(baselink_yawlink_quat_cINp, TEST_PREC, TEST_PREC));
    // CHECK_THAT (Quaternion(0.000000, 0.707107, 0.000000, 0.707107), 
    //   AllCloseVector(yawlink_pitchbacklink_quat_cINp, TEST_PREC, TEST_PREC));
    // CHECK_THAT (Quaternion(0.000000, 0.000000, 0.000000, 1.000000), 
    //   AllCloseVector(pitchbacklink_pitchbottomlink_quat_cINp, TEST_PREC, TEST_PREC));
    // CHECK_THAT (Quaternion(0.000000, 0.000000, 0.000000, 1.000000), 
    //   AllCloseVector(pitchbottomlink_pitchendlink_quat_cINp, TEST_PREC, TEST_PREC));
    // CHECK_THAT (Quaternion(0.000000, 0.707107, 0.000000, 0.707107), 
    //   AllCloseVector(yawlink_pitchfrontlink_quat_cINp, TEST_PREC, TEST_PREC));
    // CHECK_THAT (Quaternion(0.000000, 0.000000, 0.000000, 1.000000), 
    //   AllCloseVector(pitchfrontlink_pitchbottomlink_quat_cINp, TEST_PREC, TEST_PREC));
    // CHECK_THAT (Quaternion(0.000000, 0.000000, 0.000000, 1.000000), 
    //   AllCloseVector(pitchfrontlink_pitchtoplink_quat_cINp, TEST_PREC, TEST_PREC));
    // CHECK_THAT (Quaternion(0.000000, 0.000000, 0.000000, 1.000000), 
    //   AllCloseVector(pitchtoplink_pitchendlink_quat_cINp, TEST_PREC, TEST_PREC));
    // CHECK_THAT (Quaternion(0.000000, 0.000000, 0.707107, 0.707107), 
    //   AllCloseVector(pitchendlink_maininsertionlink_quat_cINp, TEST_PREC, TEST_PREC));
    // CHECK_THAT (Quaternion(0.000000, -0.707107, 0.000000, 0.707107), 
    //   AllCloseVector(maininsertionlink_toollink_quat_cINp, TEST_PREC, TEST_PREC));

    Eigen::Quaternion baselink_yawlink_quat_cOffINp_eigen =
    Eigen::Quaterniond(Eigen::AngleAxisd(baselink_yawlink_childOffset, baselink_yawlinkPA));
    Quaternion baselink_yawlink_quat_cOffINp = EigenToRBDLQuaternion(baselink_yawlink_quat_cOffINp_eigen);

    Eigen::Quaternion yawlink_pitchbacklink_quat_cOffINp_eigen =
    Eigen::Quaterniond(Eigen::AngleAxisd(yawlink_pitchbacklink_childOffset, yawlink_pitchbacklinkPA));
    Quaternion yawlink_pitchbacklink_quat_cOffINp = EigenToRBDLQuaternion(yawlink_pitchbacklink_quat_cOffINp_eigen);

    Eigen::Quaternion pitchbacklink_pitchbottomlink_quat_cOffINp_eigen =
    Eigen::Quaterniond(Eigen::AngleAxisd(pitchbacklink_pitchbottomlink_childOffset, pitchbacklink_pitchbottomlinkPA));
    Quaternion pitchbacklink_pitchbottomlink_quat_cOffINp = EigenToRBDLQuaternion(pitchbacklink_pitchbottomlink_quat_cOffINp_eigen);

    Eigen::Quaternion pitchbottomlink_pitchendlink_quat_cOffINp_eigen =
    Eigen::Quaterniond(Eigen::AngleAxisd(pitchbottomlink_pitchendlink_childOffset, pitchbottomlink_pitchendlinkPA));
    Quaternion pitchbottomlink_pitchendlink_quat_cOffINp = EigenToRBDLQuaternion(pitchbottomlink_pitchendlink_quat_cOffINp_eigen);

    Eigen::Quaternion maininsertionlink_toollink_quat_cOffINp_eigen =
    Eigen::Quaterniond(Eigen::AngleAxisd(maininsertionlink_toollink_childOffset, maininsertionlink_toollinkPA));
    Quaternion maininsertionlink_toollink_quat_cOffINp = EigenToRBDLQuaternion(maininsertionlink_toollink_quat_cOffINp_eigen);

    Eigen::Quaternion yawlink_pitchfrontlink_quat_cOffINp_eigen =
    Eigen::Quaterniond(Eigen::AngleAxisd(yawlink_pitchfrontlink_childOffset, yawlink_pitchfrontlinkPA));
    Quaternion yawlink_pitchfrontlink_quat_cOffINp = EigenToRBDLQuaternion(yawlink_pitchfrontlink_quat_cOffINp_eigen);

    Eigen::Quaternion pitchfrontlink_pitchbottomlink_quat_cOffINp_eigen =
    Eigen::Quaterniond(Eigen::AngleAxisd(pitchfrontlink_pitchbottomlink_childOffset, pitchfrontlink_pitchbottomlinkPA));
    Quaternion pitchfrontlink_pitchbottomlink_quat_cOffINp = EigenToRBDLQuaternion(pitchfrontlink_pitchbottomlink_quat_cOffINp_eigen);

    Eigen::Quaternion pitchfrontlink_pitchtoplink_quat_cOffINp_eigen =
    Eigen::Quaterniond(Eigen::AngleAxisd(pitchfrontlink_pitchtoplink_childOffset, pitchfrontlink_pitchtoplinkPA));
    Quaternion pitchfrontlink_pitchtoplink_quat_cOffINp = EigenToRBDLQuaternion(pitchfrontlink_pitchtoplink_quat_cOffINp_eigen);

    Eigen::Quaternion pitchtoplink_pitchendlink_quat_cOffINp_eigen =
    Eigen::Quaterniond(Eigen::AngleAxisd(pitchtoplink_pitchendlink_childOffset, pitchtoplink_pitchendlinkPA));
    Quaternion pitchtoplink_pitchendlink_quat_cOffINp = EigenToRBDLQuaternion(pitchtoplink_pitchendlink_quat_cOffINp_eigen);

    Eigen::Quaternion pitchendlink_maininsertionlink_quat_cOffINp_eigen =
    Eigen::Quaterniond(Eigen::AngleAxisd(pitchendlink_maininsertionlink_childOffset, pitchendlink_maininsertionlinkPA));
    Quaternion pitchendlink_maininsertionlink_quat_cOffINp = EigenToRBDLQuaternion(pitchendlink_maininsertionlink_quat_cOffINp_eigen);


    // CHECK_THAT (Quaternion(-0.000000, 1.000000, 0.000000, 0.000096), 
    //   AllCloseVector(baselink_yawlink_quat_cOffINp, TEST_PREC, TEST_PREC));
    // CHECK_THAT (Quaternion(1.000000, 0.000000, 0.000000, -0.000004), 
    //   AllCloseVector(yawlink_pitchbacklink_quat_cOffINp, TEST_PREC, TEST_PREC));
    // CHECK_THAT (Quaternion(0.000000, 0.000000, 0.000000, 1.000000), 
    //   AllCloseVector(pitchbacklink_pitchbottomlink_quat_cOffINp, TEST_PREC, TEST_PREC));
    // CHECK_THAT (Quaternion(0.000000, 0.000000, 0.000000, 1.000000), 
    //   AllCloseVector(pitchbottomlink_pitchendlink_quat_cOffINp, TEST_PREC, TEST_PREC));
    // CHECK_THAT (Quaternion(1.000000, 0.000000, 0.000000, -0.000004), 
    //   AllCloseVector(yawlink_pitchfrontlink_quat_cOffINp, TEST_PREC, TEST_PREC));
    // CHECK_THAT (Quaternion(0.000000, 0.000000, 0.000000, 1.000000), 
    //   AllCloseVector(pitchfrontlink_pitchbottomlink_quat_cOffINp, TEST_PREC, TEST_PREC));
    // CHECK_THAT (Quaternion(0.000000, 0.000000, 0.000000, 1.0000000), 
    //   AllCloseVector(pitchfrontlink_pitchtoplink_quat_cOffINp, TEST_PREC, TEST_PREC));
    // CHECK_THAT (Quaternion(0.000000, 0.000000, 0.000000, 1.000000), 
    //   AllCloseVector(pitchtoplink_pitchendlink_quat_cOffINp, TEST_PREC, TEST_PREC));
    // CHECK_THAT (Quaternion(0.000000, 0.000000, 0.000000, 1.000000), 
    //   AllCloseVector(pitchendlink_maininsertionlink_quat_cOffINp, TEST_PREC, TEST_PREC));
    // CHECK_THAT (Quaternion(-0.707108, -0.000000, -0.000000, 0.707105), 
    //   AllCloseVector(maininsertionlink_toollink_quat_cOffINp, TEST_PREC, TEST_PREC));

    Eigen::Quaternion baselink_yawlinkFC_quat_eigen = 
      baselink_yawlink_quat_cINp_eigen.inverse() * 
      baselink_yawlink_quat_cOffINp_eigen.inverse() *
      baselink_yawlink_quat_jOffINp_eigen * 
      baselink_yawlink_quat_jINp_eigen;
    Quaternion baselink_yawlinkFC_quat = EigenToRBDLQuaternion(baselink_yawlinkFC_quat_eigen); 

    Eigen::Quaternion yawlink_pitchbacklinkFC_quat_eigen = 
      yawlink_pitchbacklink_quat_cINp_eigen.inverse() * 
      yawlink_pitchbacklink_quat_cOffINp_eigen.inverse() *
      yawlink_pitchbacklink_quat_jOffINp_eigen * 
      yawlink_pitchbacklink_quat_jINp_eigen;
    Quaternion yawlink_pitchbacklinkFC_quat = EigenToRBDLQuaternion(yawlink_pitchbacklinkFC_quat_eigen); 

    Eigen::Quaternion pitchbacklink_pitchbottomlinkFC_quat_eigen = 
      pitchbacklink_pitchbottomlink_quat_cINp_eigen.inverse() * 
      pitchbacklink_pitchbottomlink_quat_cOffINp_eigen.inverse() *
      pitchbacklink_pitchbottomlink_quat_jOffINp_eigen * 
      pitchbacklink_pitchbottomlink_quat_jINp_eigen;
    Quaternion pitchbacklink_pitchbottomlinkFC_quat = EigenToRBDLQuaternion(pitchbacklink_pitchbottomlinkFC_quat_eigen); 

    Eigen::Quaternion pitchbottomlink_pitchendlinkFC_quat_eigen = 
      pitchbottomlink_pitchendlink_quat_cINp_eigen.inverse() * 
      pitchbottomlink_pitchendlink_quat_cOffINp_eigen.inverse() *
      pitchbottomlink_pitchendlink_quat_jOffINp_eigen * 
      pitchbottomlink_pitchendlink_quat_jINp_eigen;
    Quaternion pitchbottomlink_pitchendlinkFC_quat = EigenToRBDLQuaternion(pitchbottomlink_pitchendlinkFC_quat_eigen); 

    Eigen::Quaternion yawlink_pitchfrontlinkFC_quat_eigen = 
      yawlink_pitchfrontlink_quat_cINp_eigen.inverse() * 
      yawlink_pitchfrontlink_quat_cOffINp_eigen.inverse() *
      yawlink_pitchfrontlink_quat_jOffINp_eigen * 
      yawlink_pitchfrontlink_quat_jINp_eigen;
    Quaternion yawlink_pitchfrontlinkFC_quat = EigenToRBDLQuaternion(yawlink_pitchfrontlinkFC_quat_eigen); 

    Eigen::Quaternion pitchfrontlink_pitchbottomlinkFC_quat_eigen = 
      pitchfrontlink_pitchbottomlink_quat_cINp_eigen.inverse() * 
      pitchfrontlink_pitchbottomlink_quat_cOffINp_eigen.inverse() *
      pitchfrontlink_pitchbottomlink_quat_jOffINp_eigen * 
      pitchfrontlink_pitchbottomlink_quat_jINp_eigen;
    Quaternion pitchfrontlink_pitchbottomlinkFC_quat = EigenToRBDLQuaternion(pitchfrontlink_pitchbottomlinkFC_quat_eigen); 

    Eigen::Quaternion pitchfrontlink_pitchtoplinkFC_quat_eigen = 
      pitchfrontlink_pitchtoplink_quat_cINp_eigen.inverse() * 
      pitchfrontlink_pitchtoplink_quat_cOffINp_eigen.inverse() *
      pitchfrontlink_pitchtoplink_quat_jOffINp_eigen * 
      pitchfrontlink_pitchtoplink_quat_jINp_eigen;
    Quaternion pitchfrontlink_pitchtoplinkFC_quat = EigenToRBDLQuaternion(pitchfrontlink_pitchtoplinkFC_quat_eigen); 

    Eigen::Quaternion pitchtoplink_pitchendlinkFC_quat_eigen = 
      pitchtoplink_pitchendlink_quat_cINp_eigen.inverse() * 
      pitchtoplink_pitchendlink_quat_cOffINp_eigen.inverse() *
      pitchtoplink_pitchendlink_quat_jOffINp_eigen * 
      pitchtoplink_pitchendlink_quat_jINp_eigen;
    Quaternion pitchtoplink_pitchendlinkFC_quat = EigenToRBDLQuaternion(pitchtoplink_pitchendlinkFC_quat_eigen); 

    Eigen::Quaternion pitchendlink_maininsertionlinkFC_quat_eigen = 
      pitchendlink_maininsertionlink_quat_cINp_eigen.inverse() * 
      pitchendlink_maininsertionlink_quat_cOffINp_eigen.inverse() *
      pitchendlink_maininsertionlink_quat_jOffINp_eigen * 
      pitchendlink_maininsertionlink_quat_jINp_eigen;
    Quaternion pitchendlink_maininsertionlinkFC_quat = EigenToRBDLQuaternion(pitchendlink_maininsertionlinkFC_quat_eigen); 

    Eigen::Quaternion maininsertionlink_toollinkFC_quat_eigen = 
      maininsertionlink_toollink_quat_cINp_eigen.inverse() * 
      maininsertionlink_toollink_quat_cOffINp_eigen.inverse() *
      maininsertionlink_toollink_quat_jOffINp_eigen * 
      maininsertionlink_toollink_quat_jINp_eigen;
    Quaternion maininsertionlink_toollinkFC_quat = EigenToRBDLQuaternion(maininsertionlink_toollinkFC_quat_eigen); 



    // CHECK_THAT (Quaternion(0.000096, -1.000000, 0.000000, 0.000000), 
    //   AllCloseVector(baselink_yawlinkFC_quat, TEST_PREC, TEST_PREC));
    // CHECK_THAT (Quaternion(-0.000000, -0.000000, -1.000000, -0.000004), 
    //   AllCloseVector(yawlink_pitchbacklinkFC_quat, TEST_PREC, TEST_PREC));
    // CHECK_THAT (Quaternion(0.000000, 0.000000, 0.000000, 1.000000), 
    //   AllCloseVector(pitchbacklink_pitchbottomlinkFC_quat, TEST_PREC, TEST_PREC));
    // CHECK_THAT (Quaternion(0.000000, 0.000000, 0.000000, 1.000000), 
    //   AllCloseVector(pitchbottomlink_pitchendlinkFC_quat, TEST_PREC, TEST_PREC));
    // CHECK_THAT (Quaternion(-0.000000, -0.000000, -1.000000, -0.000004), 
    //   AllCloseVector(yawlink_pitchfrontlinkFC_quat, TEST_PREC, TEST_PREC));
    // CHECK_THAT (Quaternion(0.000000, 0.000000, 0.000000, 1.000000), 
    //   AllCloseVector(pitchfrontlink_pitchbottomlinkFC_quat, TEST_PREC, TEST_PREC));
    // CHECK_THAT (Quaternion(0.000000, 0.000000, 0.000000, 1.000000), 
    //   AllCloseVector(pitchfrontlink_pitchtoplinkFC_quat, TEST_PREC, TEST_PREC));
    // CHECK_THAT (Quaternion(0.000000, 0.000000, 0.000000, 1.000000), 
    //   AllCloseVector(pitchtoplink_pitchendlinkFC_quat, TEST_PREC, TEST_PREC));
    // CHECK_THAT (Quaternion(0.000000, 0.000000, -0.000000, 1.000000), 
    //   AllCloseVector(pitchendlink_maininsertionlinkFC_quat, TEST_PREC, TEST_PREC));
    // CHECK_THAT (Quaternion(0.707108, 0.707105, 0.000000, 0.000000), 
    //   AllCloseVector(maininsertionlink_toollinkFC_quat, TEST_PREC, TEST_PREC));
  }
  
  ~ECM()
  {
  }
public:
  const Vector3d baselinkInertialOffsetPos 				  = { -0.000010, -0.614520, -0.020880 };
  const Vector3d yawlinkInertialOffsetPos 				  = {  0.000000, -0.016140,  0.134470 };
  const Vector3d pitchbacklinkInertialOffsetPos 	  = { -0.051500, -0.143430, -0.00900 };
  const Vector3d pitchbottomlinkInertialOffsetPos   = {  0.149130, -0.018160,  0.000000 };
  const Vector3d pitchendlinkInertialOffsetPos 		  = {  0.051350,  0.004820,  0.000790 };
  const Vector3d maininsertionlinkInertialOffsetPos = { -0.059000, -0.016500,  0.000790 };
  const Vector3d toollinkInertialOffsetPos 				  = {  0.000000, -0.000810, -0.072320 };

  const Vector3d pitchfrontlinkInertialOffsetPos 	  = { -0.036490, -0.152610,  0.000000 };
  const Vector3d pitchtoplinkInertialOffsetPos 		  = {  0.170200, -0.000070,  0.000790 };
  //-----------------------------------------------------//
	Vector3d baselink_yawlinkPA 							= { -0.0000, -1.0000, 00.0000 };
	Vector3d baselink_yawlinkCA 							= { 00.0000, 00.0000, -1.0000 };
	Vector3d baselink_yawlinkPP               = { 00.0000, 00.0000, 00.0000 };
  Vector3d baselink_yawlinkCP               = { 00.0000, 00.0000, 00.5369 };
	//-----------------------------------------------------//
	Vector3d yawlink_pitchbacklinkPA 					= { 1.0,     0.0,    0.0 };
	Vector3d yawlink_pitchbacklinkCA 					= { 0.0,     0.0,    1.0 };
  Vector3d yawlink_pitchbacklinkPP 					= { 0.0, -0.0000, 0.1624 };
	Vector3d yawlink_pitchbacklinkCP 					= { 0.0,     0.0,    0.0 };

	Vector3d pitchbacklink_pitchbottomlinkPA 	= {     0.0,     0.0,     1.0 };
	Vector3d pitchbacklink_pitchbottomlinkCA 	= {     0.0,     0.0,     1.0 };
	Vector3d pitchbacklink_pitchbottomlinkPP 	= { -0.1028, -0.2867,     0.0 };
  Vector3d pitchbacklink_pitchbottomlinkCP 	= { -0.0364,  0.0000, -0.0000 };

	Vector3d pitchbottomlink_pitchendlinkPA 	= {    0.0,     0.0,     1.0 };
	Vector3d pitchbottomlink_pitchendlinkCA 	= {    0.0,     0.0,     1.0 };
  Vector3d pitchbottomlink_pitchendlinkPP 	= { 0.3401, -0.0000, -0.0000 };
	Vector3d pitchbottomlink_pitchendlinkCP 	= {    0.0,     0.0,  0.0000 };
	//-----------------------------------------------------//
	Vector3d yawlink_pitchfrontlinkPA 					= { 1.0,     0.0,    0.0 };
	Vector3d yawlink_pitchfrontlinkCA 					= { 0.0,     0.0,    1.0 };
  Vector3d yawlink_pitchfrontlinkPP 					= { 0.0, 		 0.0,    0.2 };
	Vector3d yawlink_pitchfrontlinkCP 					= { 0.0,     0.0,    0.0 };

	Vector3d pitchfrontlink_pitchbottomlinkPA 	= { 	 	0.0,     0.0,     1.0 };
	Vector3d pitchfrontlink_pitchbottomlinkCA 	= { 		0.0,     0.0,     1.0 };
	Vector3d pitchfrontlink_pitchbottomlinkPP 	= { -0.1031, -0.2868,  	  0.0 };
	Vector3d pitchfrontlink_pitchbottomlinkCP 	= { -0.0000, -0.0000, -0.0000 };

	Vector3d pitchfrontlink_pitchtoplinkPA 	= { 	 	0.0,     0.0,     1.0 };
	Vector3d pitchfrontlink_pitchtoplinkCA 	= { 		0.0,     0.0,     1.0 };
	Vector3d pitchfrontlink_pitchtoplinkPP 	= { -0.1084, -0.3242,  	  0.0 };
  Vector3d pitchfrontlink_pitchtoplinkCP 	= { -0.0000, -0.0000, -0.0000 };

	Vector3d pitchtoplink_pitchendlinkPA 	= {     0.0,     0.0,     1.0 };
	Vector3d pitchtoplink_pitchendlinkCA 	= {     0.0,     0.0,     1.0 };
  Vector3d pitchtoplink_pitchendlinkPP 	= {  0.3404, -0.0000, -0.000 };
	Vector3d pitchtoplink_pitchendlinkCP 	= { -0.0051, -0.0376,  0.000 };
	//-----------------------------------------------------//
	Vector3d pitchendlink_maininsertionlinkPA = {     0.0,     1.0,    0.0 };
	Vector3d pitchendlink_maininsertionlinkCA = {     1.0,     0.0,    0.0 };
  Vector3d pitchendlink_maininsertionlinkPP = {  0.1031, -0.0961, 0.0000 };
	Vector3d pitchendlink_maininsertionlinkCP = { -0.0108,  -0.062, 0.0000 };

	Vector3d maininsertionlink_toollinkPA 	  = {     1.0,     0.0,    0.0 };
	Vector3d maininsertionlink_toollinkCA 	  = {     0.0,     0.0,   -1.0 };
	Vector3d maininsertionlink_toollinkPP 	  = { -0.0108,  -0.062,    0.0 };
  Vector3d maininsertionlink_toollinkCP 	  = { -0.0000, -0.0000, 0.0118 };
	//-----------------------------------------------------//
  const double ROOT_baselink_childOffset                  = 0.0;
  const double baselink_yawlink_childOffset               = -3.1414;
  const double yawlink_pitchbacklink_childOffset          = 3.1416;
  const double pitchbacklink_pitchbottomlink_childOffset  = 0.0;
  const double baselink_pitchendlink_childOffset          = 1.56304;
  const double pitchendlink_maininsertionlink_childOffset = 0.0;
  const double maininsertionlink_toollink_childOffset     = -1.5708;
  const double pitchbottomlink_pitchendlink_childOffset   = 0.0;
  const double yawlink_pitchfrontlink_childOffset         = 3.1416;
  const double pitchfrontlink_pitchbottomlink_childOffset = 0.0;
  const double pitchfrontlink_pitchtoplink_childOffset    = 0.0;
  const double pitchtoplink_pitchendlink_childOffset      = 0.0;
	const double toollink_ee_childOffset     							  = 0.0;


  const double ROOT_baselink_jointOffset                  = 0.0;
  const double baselink_yawlink_jointOffset               = 0.0;
  const double yawlink_pitchbacklink_jointOffset          = 0.0;
  const double pitchbacklink_pitchbottomlink_jointOffset  = 0.0;
  const double baselink_pitchendlink_jointOffset          = 0.0;
  const double pitchendlink_maininsertionlink_jointOffset = 0.0;
  const double maininsertionlink_toollink_jointOffset     = 0.0;
  const double pitchbottomlink_pitchendlink_jointOffset   = 0.0;
  const double yawlink_pitchfrontlink_jointOffset         = 0.0;
  const double pitchfrontlink_pitchbottomlink_jointOffset = 0.0;
  const double pitchfrontlink_pitchtoplink_jointOffset    = 0.0;
  const double pitchtoplink_pitchendlink_jointOffset      = 0.0;
	const double toollink_ee_jointOffset      							= 0.0;

  const float baselinkScale{1.0f}, yawlinkScale{1.0f}, pitchbacklinkScale{1.0f}, pitchbottomlinkScale{1.0f}, 
    pitchendlinkScale{1.0f}, maininsertionlinkScale{1.0f}, toollinkScale{1.0f}, pitchfrontlinkScale{1.0f}, 
    pitchtoplinkScale{1.0f};
    
  Matrix3d baselink_yawlinkRot, yawlink_pitchbacklinkRot, pitchbacklink_pitchbottomlinkRot, 
    pitchbottomlink_pitchendlinkRot, yawlink_pitchfrontlinkRot, pitchfrontlink_pitchbottomlinkRot, 
    pitchfrontlink_pitchtoplinkRot, pitchtoplink_pitchendlinkRot, 
    pitchendlink_maininsertionlinkRot, maininsertionlink_toollinkRot;
  
  Matrix3d baselink_yawlinkRotOffset, yawlink_pitchbacklinkRotOffset, 
    pitchbacklink_pitchbottomlinkRotOffset, pitchbottomlink_pitchendlinkRotOffset, 
    yawlink_pitchfrontlinkRotOffset, pitchfrontlink_pitchbottomlinkRotOffset, 
    pitchfrontlink_pitchtoplinkRotOffset, pitchtoplink_pitchendlinkRotOffset, 
    pitchendlink_maininsertionlinkRotOffset, maininsertionlink_toollinkRotOffset;

  Vector3d baselink_yawlinkJAxis, yawlink_pitchbacklinkJAxis, 
    pitchbacklink_pitchbottomlinkJAxis, pitchbottomlink_pitchendlinkJAxis, 
    yawlink_pitchfrontlinkJAxis, pitchfrontlink_pitchtoplinkJAxis, 
    pitchtoplink_pitchendlinkJAxis, pitchfrontlink_pitchbottomlinkJAxis, 
    pitchendlink_maininsertionlinkJAxis, maininsertionlink_toollinkJAxis;

  SpatialTransform T_W_world_baselinkST;
};
