#ifndef _ECM2_
#define _ECM2_

#include "rbdl/rbdl.h"
#include "PCH/pch.h"
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

struct ECM2 
{
  ECM2()
  {
    ClearLogOutput();
    model = new Model;
    model->gravity = Vector3d(0., 0., -9.81);
    //1--------------------------------------------------------------------//
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

    Eigen::Quaternion Q_B_baselink_yawlink_jINp =
    Eigen::Quaterniond::FromTwoVectors(v_baselink_yawlink_ax_jINp, v_baselink_yawlinkPA);

    Eigen::Quaternion Q_B_yawlink_pitchbacklink_jINp =
    Eigen::Quaterniond::FromTwoVectors(v_yawlink_pitchbacklink_ax_jINp, v_yawlink_pitchbacklinkPA);

    Eigen::Quaternion Q_B_pitchbacklink_pitchbottomlink_jINp =
    Eigen::Quaterniond::FromTwoVectors(v_pitchbacklink_pitchbottomlink_ax_jINp, v_pitchbacklink_pitchbottomlinkPA);

    Eigen::Quaternion Q_B_pitchbottomlink_pitchendlink_jINp =
    Eigen::Quaterniond::FromTwoVectors(v_pitchbottomlink_pitchendlink_ax_jINp, v_pitchbottomlink_pitchendlinkPA);

    Eigen::Quaternion Q_B_yawlink_pitchfrontlink_jINp =
    Eigen::Quaterniond::FromTwoVectors(v_yawlink_pitchfrontlink_ax_jINp, v_yawlink_pitchfrontlinkPA);

    Eigen::Quaternion Q_B_pitchfrontlink_pitchbottomlink_jINp =
    Eigen::Quaterniond::FromTwoVectors(v_pitchfrontlink_pitchbottomlink_ax_jINp, v_pitchfrontlink_pitchbottomlinkPA);

    Eigen::Quaternion Q_B_pitchfrontlink_pitchtoplink_jINp =
    Eigen::Quaterniond::FromTwoVectors(v_pitchfrontlink_pitchtoplink_ax_jINp, v_pitchfrontlink_pitchtoplinkPA);

    Eigen::Quaternion Q_B_pitchtoplink_pitchendlink_jINp =
    Eigen::Quaterniond::FromTwoVectors(v_pitchtoplink_pitchendlink_ax_jINp, v_pitchtoplink_pitchendlinkPA);

    Eigen::Quaternion Q_B_pitchendlink_maininsertionlink_jINp =
    Eigen::Quaterniond::FromTwoVectors(v_pitchendlink_maininsertionlink_ax_jINp, v_pitchendlink_maininsertionlinkPA);

    Eigen::Quaternion Q_B_maininsertionlink_toollink_jINp =
    Eigen::Quaterniond::FromTwoVectors(v_maininsertionlink_toollink_ax_jINp, v_maininsertionlink_toollinkPA);

    // CHECK_THAT (Quaternion(0.707107, 0.000000, -0.000000, 0.707107), 
    //   AllCloseVector(EigenToRBDLQuaternion(Q_B_baselink_yawlink_jINp), TEST_PREC, TEST_PREC));
    // CHECK_THAT (Quaternion(0.000000, 0.707107, 0.000000, 0.707107), 
    //   AllCloseVector(EigenToRBDLQuaternion(Q_B_yawlink_pitchbacklink_jINp), TEST_PREC, TEST_PREC));
    // CHECK_THAT (Quaternion(0.000000, 0.000000, 0.000000, 1.000000), 
    //   AllCloseVector(EigenToRBDLQuaternion(Q_B_pitchbacklink_pitchbottomlink_jINp), TEST_PREC, TEST_PREC));
    // CHECK_THAT (Quaternion(0.000000, 0.000000, 0.000000, 1.000000), 
    //   AllCloseVector(EigenToRBDLQuaternion(Q_B_pitchbottomlink_pitchendlink_jINp), TEST_PREC, TEST_PREC));
    // CHECK_THAT (Quaternion(0.000000, 0.707107, 0.000000, 0.707107), 
    //   AllCloseVector(EigenToRBDLQuaternion(Q_B_yawlink_pitchfrontlink_jINp), TEST_PREC, TEST_PREC));
    // CHECK_THAT (Quaternion(0.000000, 0.000000, 0.000000, 1.000000), 
    //   AllCloseVector(EigenToRBDLQuaternion(Q_B_pitchfrontlink_pitchbottomlink_jINp), TEST_PREC, TEST_PREC));
    // CHECK_THAT (Quaternion(0.000000, 0.000000, 0.000000, 1.000000), 
    //   AllCloseVector(EigenToRBDLQuaternion(Q_B_pitchfrontlink_pitchtoplink_jINp), TEST_PREC, TEST_PREC));
    // CHECK_THAT (Quaternion(0.000000, 0.000000, 0.000000, 1.000000), 
    //   AllCloseVector(EigenToRBDLQuaternion(Q_B_pitchtoplink_pitchendlink_jINp), TEST_PREC, TEST_PREC));
    // CHECK_THAT (Quaternion(0.000000, 0.000000, 0.707107, 0.707107), 
    //   AllCloseVector(EigenToRBDLQuaternion(Q_B_pitchendlink_maininsertionlink_jINp), TEST_PREC, TEST_PREC));
    // CHECK_THAT (Quaternion(0.000000, 0.707107, 0.000000, 0.707107), 
    //   AllCloseVector(EigenToRBDLQuaternion(Q_B_maininsertionlink_toollink_jINp), TEST_PREC, TEST_PREC));

    //1-------------------------------------    
    Eigen::Quaternion Q_B_baselink_yawlink_jOffINp =
    Eigen::Quaterniond(Eigen::AngleAxisd(q_baselink_yawlink_jointOffset, v_baselink_yawlinkPA));

    Eigen::Quaternion Q_B_yawlink_pitchbacklink_jOffINp =
    Eigen::Quaterniond(Eigen::AngleAxisd(q_yawlink_pitchbacklink_jointOffset, v_yawlink_pitchbacklinkPA));

    Eigen::Quaternion Q_B_pitchbacklink_pitchbottomlink_jOffINp =
    Eigen::Quaterniond(Eigen::AngleAxisd(q_pitchbacklink_pitchbottomlink_jointOffset, v_pitchbacklink_pitchbottomlinkPA));

    Eigen::Quaternion Q_B_pitchbottomlink_pitchendlink_jOffINp =
    Eigen::Quaterniond(Eigen::AngleAxisd(q_pitchbottomlink_pitchendlink_jointOffset, v_pitchbottomlink_pitchendlinkPA));

    Eigen::Quaternion Q_B_maininsertionlink_toollink_jOffINp =
    Eigen::Quaterniond(Eigen::AngleAxisd(q_maininsertionlink_toollink_jointOffset, v_maininsertionlink_toollinkPA));

    Eigen::Quaternion Q_B_yawlink_pitchfrontlink_jOffINp =
    Eigen::Quaterniond(Eigen::AngleAxisd(q_yawlink_pitchfrontlink_jointOffset, v_yawlink_pitchfrontlinkPA));

    Eigen::Quaternion Q_B_pitchfrontlink_pitchbottomlink_jOffINp =
    Eigen::Quaterniond(Eigen::AngleAxisd(q_pitchfrontlink_pitchbottomlink_jointOffset, v_pitchfrontlink_pitchbottomlinkPA));

    Eigen::Quaternion Q_B_pitchfrontlink_pitchtoplink_jOffINp =
    Eigen::Quaterniond(Eigen::AngleAxisd(q_pitchfrontlink_pitchtoplink_jointOffset, v_pitchfrontlink_pitchtoplinkPA));

    Eigen::Quaternion Q_B_pitchtoplink_pitchendlink_jOffINp =
    Eigen::Quaterniond(Eigen::AngleAxisd(q_pitchtoplink_pitchendlink_jointOffset, v_pitchtoplink_pitchendlinkPA));

    Eigen::Quaternion Q_B_pitchendlink_maininsertionlink_jOffINp =
    Eigen::Quaterniond(Eigen::AngleAxisd(q_pitchendlink_maininsertionlink_jointOffset, v_pitchendlink_maininsertionlinkPA));

    // CHECK_THAT (Quaternion(0.000000, 0.000000, 0.000000, 1.000000), 
    //   AllCloseVector(EigenToRBDLQuaternion(Q_B_baselink_yawlink_jOffINp), TEST_PREC, TEST_PREC));
    // CHECK_THAT (Quaternion(0.000000, 0.000000, 0.000000, 1.000000), 
    //   AllCloseVector(EigenToRBDLQuaternion(Q_B_yawlink_pitchbacklink_jOffINp), TEST_PREC, TEST_PREC));
    // CHECK_THAT (Quaternion(0.000000, 0.000000, 0.000000, 1.000000), 
    //   AllCloseVector(EigenToRBDLQuaternion(Q_B_pitchbacklink_pitchbottomlink_jOffINp), TEST_PREC, TEST_PREC));
    // CHECK_THAT (Quaternion(0.000000, 0.000000, 0.000000, 1.000000), 
    //   AllCloseVector(EigenToRBDLQuaternion(Q_B_pitchbottomlink_pitchendlink_jOffINp), TEST_PREC, TEST_PREC));
    // CHECK_THAT (Quaternion(0.000000, 0.000000, 0.000000, 1.000000), 
    //   AllCloseVector(EigenToRBDLQuaternion(Q_B_yawlink_pitchfrontlink_jOffINp), TEST_PREC, TEST_PREC));
    // CHECK_THAT (Quaternion(0.000000, 0.000000, 0.000000, 1.000000), 
    //   AllCloseVector(EigenToRBDLQuaternion(Q_B_pitchfrontlink_pitchbottomlink_jOffINp), TEST_PREC, TEST_PREC));
    // CHECK_THAT (Quaternion(0.000000, 0.000000, 0.000000, 1.000000), 
    //   AllCloseVector(EigenToRBDLQuaternion(Q_B_pitchfrontlink_pitchtoplink_jOffINp), TEST_PREC, TEST_PREC));
    // CHECK_THAT (Quaternion(0.000000, 0.000000, 0.000000, 1.000000), 
    //   AllCloseVector(EigenToRBDLQuaternion(Q_B_pitchtoplink_pitchendlink_jOffINp), TEST_PREC, TEST_PREC));
    // CHECK_THAT (Quaternion(0.000000, 0.000000, 0.000000, 1.000000), 
    //   AllCloseVector(EigenToRBDLQuaternion(Q_B_pitchendlink_maininsertionlink_jOffINp), TEST_PREC, TEST_PREC));
    // CHECK_THAT (Quaternion(0.000000, 0.000000, 0.000000, 1.000000), 
    //   AllCloseVector(EigenToRBDLQuaternion(Q_B_maininsertionlink_toollink_jOffINp), TEST_PREC, TEST_PREC));
    //1-------------------------------------
    Eigen::Quaternion Q_B_baselink_yawlinkFP = 
      Q_B_baselink_yawlink_jOffINp * Q_B_baselink_yawlink_jINp;
    
    Eigen::Quaternion Q_B_yawlink_pitchbacklinkFP = 
      Q_B_yawlink_pitchbacklink_jOffINp * Q_B_yawlink_pitchbacklink_jINp;

    Eigen::Quaternion Q_B_pitchbacklink_pitchbottomlinkFP = 
      Q_B_pitchbacklink_pitchbottomlink_jOffINp * Q_B_pitchbacklink_pitchbottomlink_jINp;

    Eigen::Quaternion Q_B_pitchbottomlink_pitchendlinkFP = 
      Q_B_pitchbottomlink_pitchendlink_jOffINp * Q_B_pitchbottomlink_pitchendlink_jINp;

    Eigen::Quaternion Q_B_pitchendlink_maininsertionlinkFP = 
      Q_B_pitchendlink_maininsertionlink_jOffINp * Q_B_pitchendlink_maininsertionlink_jINp;

    Eigen::Quaternion Q_B_maininsertionlink_toollinkFP = 
      Q_B_maininsertionlink_toollink_jOffINp * Q_B_maininsertionlink_toollink_jINp;

    Eigen::Quaternion Q_B_yawlink_pitchfrontlinkFP = 
      Q_B_yawlink_pitchfrontlink_jOffINp * Q_B_yawlink_pitchfrontlink_jINp;

    Eigen::Quaternion Q_B_pitchfrontlink_pitchbottomlinkFP = 
      Q_B_pitchfrontlink_pitchbottomlink_jOffINp * Q_B_pitchfrontlink_pitchbottomlink_jINp;

    Eigen::Quaternion Q_B_pitchfrontlink_pitchtoplinkFP = 
      Q_B_pitchfrontlink_pitchtoplink_jOffINp * Q_B_pitchfrontlink_pitchtoplink_jINp;

    Eigen::Quaternion Q_B_pitchtoplink_pitchendlinkFP = 
      Q_B_pitchtoplink_pitchendlink_jOffINp * Q_B_pitchtoplink_pitchendlink_jINp;

    // CHECK_THAT (Quaternion(0.707107, 0.000000, 0.000000, 0.707107), 
    //   AllCloseVector(EigenToRBDLQuaternion(Q_B_baselink_yawlinkFP), TEST_PREC, TEST_PREC));
    // CHECK_THAT (Quaternion(0.000000, 0.707107, 0.000000, 0.707107), 
    //   AllCloseVector(EigenToRBDLQuaternion(Q_B_yawlink_pitchbacklinkFP), TEST_PREC, TEST_PREC));
    // CHECK_THAT (Quaternion(0.000000, 0.000000, 0.000000, 1.000000), 
    //   AllCloseVector(EigenToRBDLQuaternion(Q_B_pitchbacklink_pitchbottomlinkFP), TEST_PREC, TEST_PREC));
    // CHECK_THAT (Quaternion(0.000000, 0.000000, 0.000000, 1.000000), 
    //   AllCloseVector(EigenToRBDLQuaternion(Q_B_pitchbottomlink_pitchendlinkFP), TEST_PREC, TEST_PREC));
    // CHECK_THAT (Quaternion(0.000000, 0.000000, 0.707107, 0.707107), 
    //   AllCloseVector(EigenToRBDLQuaternion(Q_B_pitchendlink_maininsertionlinkFP), TEST_PREC, TEST_PREC));
    // CHECK_THAT (Quaternion(0.000000, 0.707107, 0.000000, 0.707107), 
    //   AllCloseVector(EigenToRBDLQuaternion(Q_B_maininsertionlink_toollinkFP), TEST_PREC, TEST_PREC));
    // CHECK_THAT (Quaternion(0.000000, 0.707107, 0.000000, 0.707107), 
    //   AllCloseVector(EigenToRBDLQuaternion(Q_B_yawlink_pitchfrontlinkFP), TEST_PREC, TEST_PREC));
    // CHECK_THAT (Quaternion(0.000000, 0.000000, 0.000000, 1.000000), 
    //   AllCloseVector(EigenToRBDLQuaternion(Q_B_pitchfrontlink_pitchbottomlinkFP), TEST_PREC, TEST_PREC));
    // CHECK_THAT (Quaternion(0.000000, 0.000000, 0.000000, 1.000000), 
    //   AllCloseVector(EigenToRBDLQuaternion(Q_B_pitchfrontlink_pitchtoplinkFP), TEST_PREC, TEST_PREC));
    // CHECK_THAT (Quaternion(0.000000, 0.000000, 0.000000, 1.000000), 
    //   AllCloseVector(EigenToRBDLQuaternion(Q_B_pitchtoplink_pitchendlinkFP), TEST_PREC, TEST_PREC));

    Eigen::Quaternion Q_B_baselink_yawlink_cINp =
    Eigen::Quaterniond(Eigen::Quaterniond::FromTwoVectors(v_baselink_yawlinkCA, v_baselink_yawlinkPA));

    Eigen::Quaternion Q_B_yawlink_pitchbacklink_cINp =
    Eigen::Quaterniond(Eigen::Quaterniond::FromTwoVectors(v_yawlink_pitchbacklinkCA, v_yawlink_pitchbacklinkPA));

    Eigen::Quaternion Q_B_pitchbacklink_pitchbottomlink_cINp =
    Eigen::Quaterniond(Eigen::Quaterniond::FromTwoVectors(v_pitchbacklink_pitchbottomlinkCA, v_pitchbacklink_pitchbottomlinkPA));

    Eigen::Quaternion Q_B_pitchbottomlink_pitchendlink_cINp =
    Eigen::Quaterniond(Eigen::Quaterniond::FromTwoVectors(v_pitchbottomlink_pitchendlinkCA, v_pitchbottomlink_pitchendlinkPA));

    Eigen::Quaternion Q_B_maininsertionlink_toollink_cINp =
    Eigen::Quaterniond(Eigen::Quaterniond::FromTwoVectors(v_maininsertionlink_toollinkCA, v_maininsertionlink_toollinkPA));

    Eigen::Quaternion Q_B_yawlink_pitchfrontlink_cINp =
    Eigen::Quaterniond(Eigen::Quaterniond::FromTwoVectors(v_yawlink_pitchfrontlinkCA, v_yawlink_pitchfrontlinkPA));

    Eigen::Quaternion Q_B_pitchfrontlink_pitchbottomlink_cINp =
    Eigen::Quaterniond(Eigen::Quaterniond::FromTwoVectors(v_pitchfrontlink_pitchbottomlinkCA, v_pitchfrontlink_pitchbottomlinkPA));

    Eigen::Quaternion Q_B_pitchfrontlink_pitchtoplink_cINp =
    Eigen::Quaterniond(Eigen::Quaterniond::FromTwoVectors(v_pitchfrontlink_pitchtoplinkCA, v_pitchfrontlink_pitchtoplinkPA));

    Eigen::Quaternion Q_B_pitchtoplink_pitchendlink_cINp =
    Eigen::Quaterniond(Eigen::Quaterniond::FromTwoVectors(v_pitchtoplink_pitchendlinkCA, v_pitchtoplink_pitchendlinkPA));

    Eigen::Quaternion Q_B_pitchendlink_maininsertionlink_cINp =
    Eigen::Quaterniond(Eigen::Quaterniond::FromTwoVectors(v_pitchendlink_maininsertionlinkCA, v_pitchendlink_maininsertionlinkPA));

    // CHECK_THAT (Quaternion(-0.707107, 0.000000, 0.000000, 0.707107), 
    //   AllCloseVector(EigenToRBDLQuaternion(Q_B_baselink_yawlink_cINp), TEST_PREC, TEST_PREC));
    // CHECK_THAT (Quaternion(0.000000, 0.707107, 0.000000, 0.707107), 
    //   AllCloseVector(EigenToRBDLQuaternion(Q_B_yawlink_pitchbacklink_cINp), TEST_PREC, TEST_PREC));
    // CHECK_THAT (Quaternion(0.000000, 0.000000, 0.000000, 1.000000), 
    //   AllCloseVector(EigenToRBDLQuaternion(Q_B_pitchbacklink_pitchbottomlink_cINp), TEST_PREC, TEST_PREC));
    // CHECK_THAT (Quaternion(0.000000, 0.000000, 0.000000, 1.000000), 
    //   AllCloseVector(EigenToRBDLQuaternion(Q_B_pitchbottomlink_pitchendlink_cINp), TEST_PREC, TEST_PREC));
    // CHECK_THAT (Quaternion(0.000000, 0.707107, 0.000000, 0.707107), 
    //   AllCloseVector(EigenToRBDLQuaternion(Q_B_yawlink_pitchfrontlink_cINp), TEST_PREC, TEST_PREC));
    // CHECK_THAT (Quaternion(0.000000, 0.000000, 0.000000, 1.000000), 
    //   AllCloseVector(EigenToRBDLQuaternion(Q_B_pitchfrontlink_pitchbottomlink_cINp), TEST_PREC, TEST_PREC));
    // CHECK_THAT (Quaternion(0.000000, 0.000000, 0.000000, 1.000000), 
    //   AllCloseVector(EigenToRBDLQuaternion(Q_B_pitchfrontlink_pitchtoplink_cINp), TEST_PREC, TEST_PREC));
    // CHECK_THAT (Quaternion(0.000000, 0.000000, 0.000000, 1.000000), 
    //   AllCloseVector(EigenToRBDLQuaternion(Q_B_pitchtoplink_pitchendlink_cINp), TEST_PREC, TEST_PREC));
    // CHECK_THAT (Quaternion(0.000000, 0.000000, 0.707107, 0.707107), 
    //   AllCloseVector(EigenToRBDLQuaternion(Q_B_pitchendlink_maininsertionlink_cINp), TEST_PREC, TEST_PREC));
    // CHECK_THAT (Quaternion(0.000000, -0.707107, 0.000000, 0.707107), 
    //   AllCloseVector(EigenToRBDLQuaternion(Q_B_maininsertionlink_toollink_cINp), TEST_PREC, TEST_PREC));


    Eigen::Quaternion Q_B_baselink_yawlink_cOffINp =
    Eigen::Quaterniond(Eigen::AngleAxisd(q_baselink_yawlink_childOffset, v_baselink_yawlinkPA));

    Eigen::Quaternion Q_B_yawlink_pitchbacklink_cOffINp =
    Eigen::Quaterniond(Eigen::AngleAxisd(q_yawlink_pitchbacklink_childOffset, v_yawlink_pitchbacklinkPA));

    Eigen::Quaternion Q_B_pitchbacklink_pitchbottomlink_cOffINp =
    Eigen::Quaterniond(Eigen::AngleAxisd(q_pitchbacklink_pitchbottomlink_childOffset, v_pitchbacklink_pitchbottomlinkPA));

    Eigen::Quaternion Q_B_pitchbottomlink_pitchendlink_cOffINp =
    Eigen::Quaterniond(Eigen::AngleAxisd(q_pitchbottomlink_pitchendlink_childOffset, v_pitchbottomlink_pitchendlinkPA));

    Eigen::Quaternion Q_B_maininsertionlink_toollink_cOffINp =
    Eigen::Quaterniond(Eigen::AngleAxisd(q_maininsertionlink_toollink_childOffset, v_maininsertionlink_toollinkPA));

    Eigen::Quaternion Q_B_yawlink_pitchfrontlink_cOffINp =
    Eigen::Quaterniond(Eigen::AngleAxisd(q_yawlink_pitchfrontlink_childOffset, v_yawlink_pitchfrontlinkPA));

    Eigen::Quaternion Q_B_pitchfrontlink_pitchbottomlink_cOffINp =
    Eigen::Quaterniond(Eigen::AngleAxisd(q_pitchfrontlink_pitchbottomlink_childOffset, v_pitchfrontlink_pitchbottomlinkPA));

    Eigen::Quaternion Q_B_pitchfrontlink_pitchtoplink_cOffINp =
    Eigen::Quaterniond(Eigen::AngleAxisd(q_pitchfrontlink_pitchtoplink_childOffset, v_pitchfrontlink_pitchtoplinkPA));

    Eigen::Quaternion Q_B_pitchtoplink_pitchendlink_cOffINp =
    Eigen::Quaterniond(Eigen::AngleAxisd(q_pitchtoplink_pitchendlink_childOffset, v_pitchtoplink_pitchendlinkPA));

    Eigen::Quaternion Q_B_pitchendlink_maininsertionlink_cOffINp =
    Eigen::Quaterniond(Eigen::AngleAxisd(q_pitchendlink_maininsertionlink_childOffset, v_pitchendlink_maininsertionlinkPA));


    // CHECK_THAT (Quaternion(-0.000000, 1.000000, 0.000000, 0.000096), 
    //   AllCloseVector(EigenToRBDLQuaternion(Q_B_baselink_yawlink_cOffINp), TEST_PREC, TEST_PREC));
    // CHECK_THAT (Quaternion(1.000000, 0.000000, 0.000000, -0.000004), 
    //   AllCloseVector(EigenToRBDLQuaternion(Q_B_yawlink_pitchbacklink_cOffINp), TEST_PREC, TEST_PREC));
    // CHECK_THAT (Quaternion(0.000000, 0.000000, 0.000000, 1.000000), 
    //   AllCloseVector(EigenToRBDLQuaternion(Q_B_pitchbacklink_pitchbottomlink_cOffINp), TEST_PREC, TEST_PREC));
    // CHECK_THAT (Quaternion(0.000000, 0.000000, 0.000000, 1.000000), 
    //   AllCloseVector(EigenToRBDLQuaternion(Q_B_pitchbottomlink_pitchendlink_cOffINp), TEST_PREC, TEST_PREC));
    // CHECK_THAT (Quaternion(1.000000, 0.000000, 0.000000, -0.000004), 
    //   AllCloseVector(EigenToRBDLQuaternion(Q_B_yawlink_pitchfrontlink_cOffINp), TEST_PREC, TEST_PREC));
    // CHECK_THAT (Quaternion(0.000000, 0.000000, 0.000000, 1.000000), 
    //   AllCloseVector(EigenToRBDLQuaternion(Q_B_pitchfrontlink_pitchbottomlink_cOffINp), TEST_PREC, TEST_PREC));
    // CHECK_THAT (Quaternion(0.000000, 0.000000, 0.000000, 1.0000000), 
    //   AllCloseVector(EigenToRBDLQuaternion(Q_B_pitchfrontlink_pitchtoplink_cOffINp), TEST_PREC, TEST_PREC));
    // CHECK_THAT (Quaternion(0.000000, 0.000000, 0.000000, 1.000000), 
    //   AllCloseVector(EigenToRBDLQuaternion(Q_B_pitchtoplink_pitchendlink_cOffINp), TEST_PREC, TEST_PREC));
    // CHECK_THAT (Quaternion(0.000000, 0.000000, 0.000000, 1.000000), 
    //   AllCloseVector(EigenToRBDLQuaternion(Q_B_pitchendlink_maininsertionlink_cOffINp), TEST_PREC, TEST_PREC));
    // CHECK_THAT (Quaternion(-0.707108, -0.000000, -0.000000, 0.707105), 
    //   AllCloseVector(EigenToRBDLQuaternion(Q_B_maininsertionlink_toollink_cOffINp), TEST_PREC, TEST_PREC));

    Eigen::Quaternion Q_B_baselink_yawlinkFC = 
      Q_B_baselink_yawlink_cINp.inverse() * 
      Q_B_baselink_yawlink_cOffINp.inverse() *
      Q_B_baselink_yawlink_jOffINp * 
      Q_B_baselink_yawlink_jINp;

    Eigen::Quaternion Q_B_yawlink_pitchbacklinkFC = 
      Q_B_yawlink_pitchbacklink_cINp.inverse() * 
      Q_B_yawlink_pitchbacklink_cOffINp.inverse() *
      Q_B_yawlink_pitchbacklink_jOffINp * 
      Q_B_yawlink_pitchbacklink_jINp;

    Eigen::Quaternion Q_B_pitchbacklink_pitchbottomlinkFC = 
      Q_B_pitchbacklink_pitchbottomlink_cINp.inverse() * 
      Q_B_pitchbacklink_pitchbottomlink_cOffINp.inverse() *
      Q_B_pitchbacklink_pitchbottomlink_jOffINp * 
      Q_B_pitchbacklink_pitchbottomlink_jINp;

    Eigen::Quaternion Q_B_pitchbottomlink_pitchendlinkFC = 
      Q_B_pitchbottomlink_pitchendlink_cINp.inverse() * 
      Q_B_pitchbottomlink_pitchendlink_cOffINp.inverse() *
      Q_B_pitchbottomlink_pitchendlink_jOffINp * 
      Q_B_pitchbottomlink_pitchendlink_jINp;

    Eigen::Quaternion Q_B_yawlink_pitchfrontlinkFC = 
      Q_B_yawlink_pitchfrontlink_cINp.inverse() * 
      Q_B_yawlink_pitchfrontlink_cOffINp.inverse() *
      Q_B_yawlink_pitchfrontlink_jOffINp * 
      Q_B_yawlink_pitchfrontlink_jINp;

    Eigen::Quaternion Q_B_pitchfrontlink_pitchbottomlinkFC = 
      Q_B_pitchfrontlink_pitchbottomlink_cINp.inverse() * 
      Q_B_pitchfrontlink_pitchbottomlink_cOffINp.inverse() *
      Q_B_pitchfrontlink_pitchbottomlink_jOffINp * 
      Q_B_pitchfrontlink_pitchbottomlink_jINp;

    Eigen::Quaternion Q_B_pitchfrontlink_pitchtoplinkFC = 
      Q_B_pitchfrontlink_pitchtoplink_cINp.inverse() * 
      Q_B_pitchfrontlink_pitchtoplink_cOffINp.inverse() *
      Q_B_pitchfrontlink_pitchtoplink_jOffINp * 
      Q_B_pitchfrontlink_pitchtoplink_jINp;

    Eigen::Quaternion Q_B_pitchtoplink_pitchendlinkFC = 
      Q_B_pitchtoplink_pitchendlink_cINp.inverse() * 
      Q_B_pitchtoplink_pitchendlink_cOffINp.inverse() *
      Q_B_pitchtoplink_pitchendlink_jOffINp * 
      Q_B_pitchtoplink_pitchendlink_jINp;

    Eigen::Quaternion Q_B_pitchendlink_maininsertionlinkFC = 
      Q_B_pitchendlink_maininsertionlink_cINp.inverse() * 
      Q_B_pitchendlink_maininsertionlink_cOffINp.inverse() *
      Q_B_pitchendlink_maininsertionlink_jOffINp * 
      Q_B_pitchendlink_maininsertionlink_jINp;

    Eigen::Quaternion Q_B_maininsertionlink_toollinkFC = 
      Q_B_maininsertionlink_toollink_cINp.inverse() * 
      Q_B_maininsertionlink_toollink_cOffINp.inverse() *
      Q_B_maininsertionlink_toollink_jOffINp * 
      Q_B_maininsertionlink_toollink_jINp;

    // CHECK_THAT (Quaternion(0.000096, -1.000000, 0.000000, 0.000000), 
    //   AllCloseVector(EigenToRBDLQuaternion(Q_B_baselink_yawlinkFC), TEST_PREC, TEST_PREC));
    // CHECK_THAT (Quaternion(-0.000000, -0.000000, -1.000000, -0.000004), 
    //   AllCloseVector(EigenToRBDLQuaternion(Q_B_yawlink_pitchbacklinkFC), TEST_PREC, TEST_PREC));
    // CHECK_THAT (Quaternion(0.000000, 0.000000, 0.000000, 1.000000), 
    //   AllCloseVector(EigenToRBDLQuaternion(Q_B_pitchbacklink_pitchbottomlinkFC), TEST_PREC, TEST_PREC));
    // CHECK_THAT (Quaternion(0.000000, 0.000000, 0.000000, 1.000000), 
    //   AllCloseVector(EigenToRBDLQuaternion(Q_B_pitchbottomlink_pitchendlinkFC), TEST_PREC, TEST_PREC));
    // CHECK_THAT (Quaternion(-0.000000, -0.000000, -1.000000, -0.000004), 
    //   AllCloseVector(EigenToRBDLQuaternion(Q_B_yawlink_pitchfrontlinkFC), TEST_PREC, TEST_PREC));
    // CHECK_THAT (Quaternion(0.000000, 0.000000, 0.000000, 1.000000), 
    //   AllCloseVector(EigenToRBDLQuaternion(Q_B_pitchfrontlink_pitchbottomlinkFC), TEST_PREC, TEST_PREC));
    // CHECK_THAT (Quaternion(0.000000, 0.000000, 0.000000, 1.000000), 
    //   AllCloseVector(EigenToRBDLQuaternion(Q_B_pitchfrontlink_pitchtoplinkFC), TEST_PREC, TEST_PREC));
    // CHECK_THAT (Quaternion(0.000000, 0.000000, 0.000000, 1.000000), 
    //   AllCloseVector(EigenToRBDLQuaternion(Q_B_pitchtoplink_pitchendlinkFC), TEST_PREC, TEST_PREC));
    // CHECK_THAT (Quaternion(0.000000, 0.000000, -0.000000, 1.000000), 
    //   AllCloseVector(EigenToRBDLQuaternion(Q_B_pitchendlink_maininsertionlinkFC), TEST_PREC, TEST_PREC));
    // CHECK_THAT (Quaternion(0.707108, 0.707105, 0.000000, 0.000000), 
    //   AllCloseVector(EigenToRBDLQuaternion(Q_B_maininsertionlink_toollinkFC), TEST_PREC, TEST_PREC));
    // //-----------------------------------------------------//
    Eigen::Quaterniond Q_W_world_baselink; Q_W_world_baselink.setIdentity();
    //--------------------------//
    const Eigen::Quaterniond Q_W_baselink_yawlinkFP = Q_W_world_baselink     * Q_B_baselink_yawlinkFP;
    const Eigen::Quaterniond Q_W_baselink_yawlinkFC = Q_W_baselink_yawlinkFP * Q_B_baselink_yawlinkFC;

    const Vector3d v_W_baselink_yawlink = 
      Q_W_baselink_yawlinkFP.inverse() * v_baselink_yawlinkPP - 
      Q_W_baselink_yawlinkFC.inverse() * v_baselink_yawlinkCP;

    const Eigen::Quaterniond Q_W_world_yawlink = 
      Q_W_world_baselink * Q_B_baselink_yawlinkFP * Q_B_baselink_yawlinkFC;
    CHECK_THAT (
      Matrix3d(-1.0,  0.0,  0.0,
                0.0,  0.0,  1.0,
                0.0,  1.0,  0.0), 
      AllCloseMatrix(Q_W_world_yawlink.toRotationMatrix(), TEST_PREC, TEST_PREC));
    //--------------------------//
    const Eigen::Quaterniond Q_W_yawlink_pitchbacklinkFP = Q_W_world_yawlink           * Q_B_yawlink_pitchbacklinkFP;
    const Eigen::Quaterniond Q_W_yawlink_pitchbacklinkFC = Q_W_yawlink_pitchbacklinkFP * Q_B_yawlink_pitchbacklinkFC;

    const Vector3d v_W_yawlink_pitchbacklink = 
      Q_W_yawlink_pitchbacklinkFP.inverse() * v_yawlink_pitchbacklinkPP - 
      Q_W_yawlink_pitchbacklinkFC.inverse() * v_yawlink_pitchbacklinkCP;

    const Eigen::Quaterniond Q_W_world_pitchbacklink = 
      Q_W_world_yawlink * Q_B_yawlink_pitchbacklinkFP * Q_B_yawlink_pitchbacklinkFC;

    CHECK_THAT (
      Matrix3d( 0.0,  0.0, -1.0,
                1.0,  0.0,  0.0,
                0.0, -1.0,  0.0), 
      AllCloseMatrix(Q_W_world_pitchbacklink.toRotationMatrix(), TEST_PREC, TEST_PREC));
    //--------------------------//
    const Eigen::Quaterniond Q_W_pitchbacklink_pitchbottomlinkFP = 
      Q_W_world_pitchbacklink     * Q_B_pitchbacklink_pitchbottomlinkFP;
    const Eigen::Quaterniond Q_W_pitchbacklink_pitchbottomlinkFC = 
      Q_W_yawlink_pitchbacklinkFP * Q_B_pitchbacklink_pitchbottomlinkFC;
    
    const Vector3d v_W_pitchbacklink_pitchbottomlink = 
      // mandate [2,..] = [0, -1, 0]
      Q_W_pitchbacklink_pitchbottomlinkFP * v_pitchbacklink_pitchbottomlinkPP + 
      Q_W_pitchbacklink_pitchbottomlinkFC * v_pitchbacklink_pitchbottomlinkCP;

    const Eigen::Quaterniond Q_W_world_pitchbottomlink = 
      Q_W_world_pitchbacklink * Q_B_pitchbacklink_pitchbottomlinkFP * Q_B_pitchbacklink_pitchbottomlinkFC;
    CHECK_THAT (
      Matrix3d( 0.0,  0.0, -1.0,
                1.0,  0.0,  0.0,
                0.0, -1.0,  0.0), 
      AllCloseMatrix(Q_W_world_pitchbottomlink.toRotationMatrix(), TEST_PREC, TEST_PREC));
    //--------------------------//
    const Eigen::Quaterniond Q_W_pitchbottomlink_pitchendlinkFP = 
      Q_W_world_pitchbottomlink     * Q_B_pitchbottomlink_pitchendlinkFP;
    const Eigen::Quaterniond Q_W_pitchbottomlink_pitchendlinkFC = 
      Q_W_pitchbottomlink_pitchendlinkFP * Q_B_pitchbottomlink_pitchendlinkFC;
    
    const Vector3d v_W_pitchbottomlink_pitchendlink = 
      Q_W_pitchbottomlink_pitchendlinkFP * v_pitchbottomlink_pitchendlinkPP + 
      Q_W_pitchbottomlink_pitchendlinkFC * v_pitchbottomlink_pitchendlinkCP;

    const Eigen::Quaterniond Q_W_world_pitchendlink = 
      Q_W_world_pitchbottomlink * Q_B_pitchbottomlink_pitchendlinkFP * Q_B_pitchbottomlink_pitchendlinkFC;
    CHECK_THAT (
      Matrix3d( 0.0,  0.0, -1.0,
                1.0,  0.0,  0.0,
                0.0, -1.0,  0.0), 
      AllCloseMatrix(Q_W_world_pitchendlink.toRotationMatrix(), TEST_PREC, TEST_PREC));  
    //--------------------------//
    const Eigen::Quaterniond Q_W_yawlink_pitchfrontlinkFP = 
      Q_W_world_yawlink     * Q_B_yawlink_pitchfrontlinkFP;
    const Eigen::Quaterniond Q_W_yawlink_pitchfrontlinkFC = 
      Q_W_yawlink_pitchfrontlinkFP * Q_B_yawlink_pitchfrontlinkFC;
    
    const Vector3d v_W_yawlink_pitchfrontlink = 
      // mandate
      Q_W_yawlink_pitchfrontlinkFP.inverse() * v_yawlink_pitchfrontlinkPP + 
      Q_W_yawlink_pitchfrontlinkFC * v_yawlink_pitchfrontlinkCP;

    const Eigen::Quaterniond Q_W_world_pitchfrontlink = 
      Q_W_world_yawlink * Q_B_yawlink_pitchfrontlinkFP * Q_B_yawlink_pitchfrontlinkFC;
    CHECK_THAT (
      Matrix3d( 0.0,  0.0, -1.0,
                1.0,  0.0,  0.0,
                0.0, -1.0,  0.0), 
      AllCloseMatrix(Q_W_world_pitchfrontlink.toRotationMatrix(), TEST_PREC, TEST_PREC));  
    //--------------------------//
    const Eigen::Quaterniond Q_W_pitchfrontlink_pitchbottomlinkFP = 
      Q_W_world_pitchfrontlink     * Q_B_pitchfrontlink_pitchbottomlinkFP;
    const Eigen::Quaterniond Q_W_pitchfrontlink_pitchbottomlinkFC = 
      Q_W_pitchfrontlink_pitchbottomlinkFP * Q_B_pitchfrontlink_pitchbottomlinkFC;
    
    const Vector3d v_W_pitchfrontlink_pitchbottomlink = 
      Q_W_pitchfrontlink_pitchbottomlinkFP * v_pitchfrontlink_pitchbottomlinkPP + 
      Q_W_pitchfrontlink_pitchbottomlinkFC * v_pitchfrontlink_pitchbottomlinkCP;
    
    const Eigen::Quaterniond Q_W_world_pitchbottomlink2 = 
      Q_W_world_pitchfrontlink * Q_B_pitchfrontlink_pitchbottomlinkFP * Q_B_pitchfrontlink_pitchbottomlinkFC;
    CHECK_THAT (
      Matrix3d( 0.0,  0.0, -1.0,
                1.0,  0.0,  0.0,
                0.0, -1.0,  0.0), 
      AllCloseMatrix(Q_W_world_pitchbottomlink2.toRotationMatrix(), TEST_PREC, TEST_PREC));  
    //--------------------------//
    const Eigen::Quaterniond Q_W_pitchfrontlink_pitchtoplinkFP = 
      Q_W_world_pitchfrontlink     * Q_B_pitchfrontlink_pitchtoplinkFP;
    const Eigen::Quaterniond Q_W_pitchfrontlink_pitchtoplinkFC = 
      Q_W_pitchfrontlink_pitchtoplinkFP * Q_B_pitchfrontlink_pitchtoplinkFC;
    
    const Vector3d v_W_pitchfrontlink_pitchtoplink = 
      Q_W_pitchfrontlink_pitchtoplinkFP * v_pitchfrontlink_pitchtoplinkPP + 
      Q_W_pitchfrontlink_pitchtoplinkFC * v_pitchfrontlink_pitchtoplinkCP;

    const Eigen::Quaterniond Q_W_world_pitchtoplink = 
      Q_W_world_pitchfrontlink * Q_B_pitchfrontlink_pitchtoplinkFP * Q_B_pitchfrontlink_pitchtoplinkFC;
    CHECK_THAT (
      Matrix3d( 0.0,  0.0, -1.0,
                1.0,  0.0,  0.0,
                0.0, -1.0,  0.0), 
      AllCloseMatrix(Q_W_world_pitchtoplink.toRotationMatrix(), TEST_PREC, TEST_PREC));  
    //--------------------------//
    const Eigen::Quaterniond Q_W_pitchtoplink_pitchendlinkFP = 
      Q_W_world_pitchtoplink * Q_B_pitchtoplink_pitchendlinkFP;
    const Eigen::Quaterniond Q_W_pitchtoplink_pitchendlinkFC = 
      Q_W_pitchtoplink_pitchendlinkFP * Q_B_pitchtoplink_pitchendlinkFC;

    const Vector3d v_W_pitchtoplink_pitchendlink = 
      Q_W_pitchtoplink_pitchendlinkFP * v_pitchtoplink_pitchendlinkPP - 
      Q_W_pitchtoplink_pitchendlinkFC * v_pitchtoplink_pitchendlinkCP;

    const Eigen::Quaterniond Q_W_world_pitchendlink2 = 
      Q_W_world_pitchtoplink * Q_B_pitchtoplink_pitchendlinkFP * Q_B_pitchtoplink_pitchendlinkFC;
    CHECK_THAT (
      Matrix3d( 0.0,  0.0, -1.0,
                1.0,  0.0,  0.0,
                0.0, -1.0,  0.0), 
      AllCloseMatrix(Q_W_world_pitchendlink2.toRotationMatrix(), TEST_PREC, TEST_PREC));
    //--------------------------//
    const Eigen::Quaterniond Q_W_pitchendlink_maininsertionlinkFP = 
      Q_W_world_pitchendlink * Q_B_pitchendlink_maininsertionlinkFP;
    const Eigen::Quaterniond Q_W_pitchendlink_maininsertionlinkFC = 
      Q_W_pitchendlink_maininsertionlinkFP * Q_B_pitchendlink_maininsertionlinkFC;
    
    const Vector3d v_W_pitchendlink_maininsertionlink = Vector3d(0, 0.040988, 0.084072);
      // Doest match
      // Q_W_pitchendlink_maininsertionlinkFP * v_pitchendlink_maininsertionlinkPP - 
      // Q_W_pitchendlink_maininsertionlinkFC.inverse() * v_pitchendlink_maininsertionlinkCP;

    const Eigen::Quaterniond Q_W_world_maininsertionlink = 
      Q_W_world_pitchendlink * Q_B_pitchendlink_maininsertionlinkFP * Q_B_pitchendlink_maininsertionlinkFC;
    CHECK_THAT (
      Matrix3d( 0.0,  0.0, -1.0,
                0.0, -1.0,  0.0,
               -1.0,  0.0,  0.0), 
      AllCloseMatrix(Q_W_world_maininsertionlink.toRotationMatrix(), TEST_PREC, TEST_PREC));
    //--------------------------//
    const Eigen::Quaterniond Q_W_maininsertionlink_toollinkFP = 
      Q_W_world_maininsertionlink * Q_B_maininsertionlink_toollinkFP;
    const Eigen::Quaterniond Q_W_maininsertionlink_toollinkFC = 
      Q_W_maininsertionlink_toollinkFP * Q_B_maininsertionlink_toollinkFC;

    const Vector3d v_W_maininsertionlink_toollink = Vector3d(0, 0.061999, -0.000951);
      // Doesnt match
      // Q_W_maininsertionlink_toollinkFP * v_maininsertionlink_toollinkPP - 
      // Q_W_maininsertionlink_toollinkFC * v_maininsertionlink_toollinkCP;

    const Eigen::Quaterniond Q_W_world_toollink = 
      Q_W_world_maininsertionlink * Q_B_maininsertionlink_toollinkFP * Q_B_maininsertionlink_toollinkFC;
    CHECK_THAT (
      Matrix3d( 0.0,  1.0,  0.0,
               -1.0,  0.0,  0.0,
                0.0,  0.0,  1.0), 
      AllCloseMatrix(Q_W_world_toollink.toRotationMatrix(), TEST_PREC, TEST_PREC));
    
    // //--------------------------------------------------------------------//
    // const Vector3d baselink_yawlinkJAxis = world_baselinkST.E * baselink_yawlinkPA;
    
    // const Vector3d yawlink_pitchbacklinkJAxis = world_yawlinkST.E * yawlink_pitchbacklinkPA;
    // const Vector3d pitchbacklink_pitchbottomlinkJAxis = world_pitchbacklinkST.E * pitchbacklink_pitchbottomlinkPA;
    // const Vector3d pitchbottomlink_pitchendlinkJAxis = world_pitchbottomlinkST.E * pitchbottomlink_pitchendlinkPA;
    
    // const Vector3d yawlink_pitchfrontlinkJAxis = world_yawlinkST.E * yawlink_pitchfrontlinkPA;
    // const Vector3d pitchfrontlink_pitchtoplinkJAxis = world_pitchfrontlinkST.E * pitchfrontlink_pitchtoplinkPA;
    // const Vector3d pitchtoplink_pitchendlinkJAxis = world_pitchtoplinkST.E * pitchtoplink_pitchendlinkPA;
    
    // const Vector3d pitchfrontlink_pitchbottomlinkJAxis = world_pitchfrontlinkST.E * pitchfrontlink_pitchbottomlinkPA;
    // const Vector3d pitchendlink_maininsertionlinkJAxis = world_pitchendlinkST.E * pitchendlink_maininsertionlinkPA;
    // const Vector3d maininsertionlink_toollinkJAxis = world_maininsertionlinkST.E * maininsertionlink_toollinkPA;
    //--------------------------------------------------------------------//


    Joint joint_base = Joint(JointTypeFixed);
    world_baselinkId = model-> 
      AddBody(0, Xtrans(Vector3d(0.5, -0.4, -0.6)), joint_base, baselinkBody, "world-baselink");

    // Joint joint_yaw = Joint(SpatialVector (0., -1., 0., 0., 0., 0.));
    // Joint baselink_yawlinkJ = Joint(SpatialVector ( 
    //   baselink_yawlinkJAxis(0), 
    //   baselink_yawlinkJAxis(1), 
    //   baselink_yawlinkJAxis(2),
    //   0., 0., 0.));
    baselink_yawlinkId = model->
      AddBody(world_baselinkId, Xtrans(v_W_baselink_yawlink), 
      // baselink_yawlinkJ, yawlinkBody, "baselink-yawlink");
      Joint(SpatialVector (0., -1., 0., 0., 0., 0.)), yawlinkBody, "baselink-yawlink");
    
    
    // Joint yawlink_pitchbacklinkJ = Joint(SpatialVector ( 
    //   yawlink_pitchbacklinkJAxis(0), 
    //   yawlink_pitchbacklinkJAxis(1), 
    //   yawlink_pitchbacklinkJAxis(2),
    //   0., 0., 0.));
    yawlink_pitchbacklinkId = model->
      AddBody(baselink_yawlinkId, Xtrans(v_W_yawlink_pitchbacklink), 
      Joint(SpatialVector (-1., 0., 0., 0., 0., 0.)), pitchbacklinkBody, "yawlink-pitchbacklink");
      // yawlink_pitchbacklinkJ, pitchbacklinkBody, "yawlink-pitchbacklink");

    // Joint pitchbacklink_pitchbottomlinkJ = Joint(SpatialVector (
    //   pitchbacklink_pitchbottomlinkJAxis(0), 
    //   pitchbacklink_pitchbottomlinkJAxis(1), 
    //   pitchbacklink_pitchbottomlinkJAxis(2),
    //   0., 0., 0.));
    pitchbacklink_pitchbottomlinkId = model->
      AddBody(yawlink_pitchbacklinkId, Xtrans(v_W_pitchbacklink_pitchbottomlink), 
      // pitchbacklink_pitchbottomlinkJ, pitchbottomlinkBody, "pitchbacklink-pitchbottomlink");
      Joint(SpatialVector (-1., 0., 0., 0., 0., 0.)), pitchbottomlinkBody, "pitchbacklink-pitchbottomlink");

    // Joint pitchbottomlink_pitchendlinkJ = Joint(SpatialVector (
    //   pitchbottomlink_pitchendlinkJAxis(0), 
    //   pitchbottomlink_pitchendlinkJAxis(1), 
    //   pitchbottomlink_pitchendlinkJAxis(2), 
    //   0., 0., 0.));
    pitchbottomlink_pitchendlinkId = model->
      AddBody(pitchbacklink_pitchbottomlinkId, Xtrans(v_W_pitchbottomlink_pitchendlink), 
      Joint(SpatialVector (-1., 0., 0., 0., 0., 0.)), pitchendlinkBody, "pitchbottomlink-pitchendlink");
      // pitchbottomlink_pitchendlinkJ, pitchendlinkBody, "pitchbottomlink-pitchendlink");
      
    
    // Joint yawlink_pitchfrontlinkJ = Joint(SpatialVector (
    //   yawlink_pitchfrontlinkJAxis(0), 
    //   yawlink_pitchfrontlinkJAxis(1), 
    //   yawlink_pitchfrontlinkJAxis(2),
    //   0., 0., 0.));
    yawlink_pitchfrontlinkId = model->
      AddBody(baselink_yawlinkId, Xtrans(v_W_yawlink_pitchfrontlink), 
      Joint(SpatialVector (-1., 0., 0., 0., 0., 0.)), pitchfrontlinkBody, "yawlink-pitchfrontlink");
      // yawlink_pitchfrontlinkJ, pitchfrontlinkBody, "yawlink-pitchfrontlink");

    // This has to be added
    pitchfrontlink_pitchbottomlinkId = model->
    	AddBody(yawlink_pitchfrontlinkId, Xtrans(v_W_pitchfrontlink_pitchbottomlink), 
    	Joint(SpatialVector (-1., 0., 0., 0., 0., 0.)), virtualBody, "pitchfrontlink-pitchbottomlink");

    // // // Joint pitchfrontlink_pitchtoplinkJ = Joint(SpatialVector (
    // // //   pitchfrontlink_pitchtoplinkJAxis(0), 
    // // //   pitchfrontlink_pitchtoplinkJAxis(1), 
    // // //   pitchfrontlink_pitchtoplinkJAxis(2),
    // // //   0., 0., 0.));
    pitchfrontlink_pitchtoplinkId = model->
      AddBody(yawlink_pitchfrontlinkId, Xtrans(v_W_pitchfrontlink_pitchtoplink), 
      Joint(SpatialVector (-1., 0., 0., 0., 0., 0.)), pitchtoplinkBody, 
      // pitchfrontlink_pitchtoplinkJ, pitchtoplinkBody, 
      "pitchfrontlink-pitchtoplink");

    // // // Joint pitchtoplink_pitchendlinkJ = Joint(SpatialVector (
    // // //   pitchtoplink_pitchendlinkJAxis(0), 
    // // //   pitchtoplink_pitchendlinkJAxis(1), 
    // // //   pitchtoplink_pitchendlinkJAxis(2),
    // // //   0., 0., 0.));
    pitchtoplink_pitchendlinkId = model->
      AddBody(pitchfrontlink_pitchtoplinkId, Xtrans(v_W_pitchtoplink_pitchendlink), 
      Joint(SpatialVector (-1., 0., 0., 0., 0., 0.)), virtualBody, "pitchtoplink-pitchendlink");
      // pitchtoplink_pitchendlinkJ, virtualBody, "pitchtoplink-pitchendlink");

    // // // Joint pitchendlink_maininsertionlinkJ = Joint(SpatialVector ( 
    // // //   0., 0., 0., 
    // // //   pitchendlink_maininsertionlinkJAxis(0), 
    // // //   pitchendlink_maininsertionlinkJAxis(1), 
    // // //   pitchendlink_maininsertionlinkJAxis(2)));
    pitchendlink_maininsertionlinkId = model->
      AddBody(pitchbottomlink_pitchendlinkId, Xtrans(v_W_pitchendlink_maininsertionlink), 
      Joint(SpatialVector (0., 0., 0., 0., 0., -1.)), pitchendlinkBody, "pitchendlink-maininsertionlink");
      // pitchendlink_maininsertionlinkJ, pitchendlinkBody, "pitchendlink-maininsertionlink");

    // Joint maininsertionlink_toollinkJ = Joint(SpatialVector (
    //   maininsertionlink_toollinkJAxis(0), 
    //   maininsertionlink_toollinkJAxis(1), 
    //   maininsertionlink_toollinkJAxis(2),
    //   0., 0., 0.));
    maininsertionlink_toollinkId = model->
      AddBody(pitchendlink_maininsertionlinkId, Xtrans(v_W_maininsertionlink_toollink),  
      Joint(SpatialVector (0., 0., 1., 0., 0., 0.)), pitchendlinkBody, "maininsertionlink-toollink");
      // maininsertionlink_toollinkJ, pitchendlinkBody, "maininsertionlink-toollink");



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

    // // Vector3d vector3d_zero = Vector3d::Zero();
    // // //0---------------------------------------------------------------------//
    // // T_W_world_baselinkST.E.setIdentity();
    // // T_W_world_baselinkST.r = Vector3d(0.5, -0.4, -0.6);
    // // //1--------------------------------------------------------------------//		
    // // Matrix3d baselink_yawlinkRot = 
	  // // Eigen::Matrix3d(Eigen::Quaterniond::FromTwoVectors(baselink_yawlinkPA, baselink_yawlinkCA));
	  // // Eigen::Affine3d baselink_yawlinkRotOffset(Eigen::AngleAxisd(baselink_yawlinkOffsetQ, -Vector3d::UnitZ()));
		
	  // // SpatialTransform baselink_yawlinkST;
	  // // // baselink_yawlinkST.E = baselink_yawlinkRot.transpose() * baselink_yawlinkRotOffset.rotation();
    // // baselink_yawlinkST.E = Matrix3d(-1,  0,  0, 0,  0, 1, 0,  1, 0);
	  
    // // baselink_yawlinkST.r = 
		// // baselink_yawlinkPP - (baselink_yawlinkRot.transpose() * baselink_yawlinkCP);


    Q     = VectorNd::Constant ((std::size_t) model->dof_count, 0.);
    QDot  = VectorNd::Constant ((std::size_t) model->dof_count, 0.);
    QDDot = VectorNd::Constant ((std::size_t) model->dof_count, 0.);
    Tau   = VectorNd::Constant ((std::size_t) model->dof_count, 0.);

    std::map<std::string, unsigned int> rbdlBodyMap;
    std::map<std::string, unsigned int>::iterator rbdlBodyMapItr;
    rbdlBodyMap = model->mBodyNameMap;

    ClearLogOutput();
    for(rbdlBodyMapItr = rbdlBodyMap.begin(); 
        rbdlBodyMapItr != rbdlBodyMap.end(); 
        rbdlBodyMapItr++)
    {
      std::string bodyName = rbdlBodyMapItr->first;
      unsigned int bodyId = rbdlBodyMapItr->second;

      std::string parentName = model->GetBodyName(model->GetParentBodyId(bodyId));
      bool isFixedBody = model->IsFixedBodyId(bodyId);
      std::cout << parentName << ", " << bodyName    << ", " 
                << bodyId     << ", " << isFixedBody << std::endl;
      // std::cout << --bodyId << ", " << bodyName << std::endl;
    }
  }
  
  ~ECM2()
  {
    delete model;
  }

public:
  const Vector3d vector3d_zero = Eigen::Vector3d::Zero();
  const Body virtualBody              = Body(0., vector3d_zero, vector3d_zero);

  const Vector3d v_baselinkInertialOffset 				  = { -0.000010, -0.614520, -0.020880 };
  const Vector3d v_yawlinkInertialOffset 				  = {  0.000000, -0.016140,  0.134470 };
  const Vector3d v_pitchbacklinkInertialOffset 	  = { -0.051500, -0.143430, -0.00900 };
  const Vector3d v_pitchbottomlinkInertialOffset   = {  0.149130, -0.018160,  0.000000 };
  const Vector3d v_pitchendlinkInertialOffset 		  = {  0.051350,  0.004820,  0.000790 };
  const Vector3d v_maininsertionlinkInertialOffset = { -0.059000, -0.016500,  0.000790 };
  const Vector3d v_toollinkInertialOffset 				  = {  0.000000, -0.000810, -0.072320 };
  const Vector3d v_pitchfrontlinkInertialOffset 	  = { -0.036490, -0.152610,  0.000000 };
  const Vector3d v_pitchtoplinkInertialOffset 		  = {  0.170200, -0.000070,  0.000790 };


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
      

  //-----------------------------------------------------//
	Vector3d v_baselink_yawlinkPA 							= { -0.0000, -1.0000, 00.0000 };
	Vector3d v_baselink_yawlinkCA 							= { 00.0000, 00.0000, -1.0000 };
	Vector3d v_baselink_yawlinkPP               = { 00.0000, 00.0000, 00.0000 };
  Vector3d v_baselink_yawlinkCP               = { 00.0000, 00.0000, 00.5369 };

  // Vector3d v_baselink_yawlinkPA_eigen 							= { -0.0000, -1.0000, 00.0000 };
	// Vector3d v_baselink_yawlinkCA_eigen 							= { 00.0000, 00.0000, -1.0000 };
	// Vector3d v_baselink_yawlinkPP_eigen               = { 00.0000, 00.0000, 00.0000 };
  // Vector3d v_baselink_yawlinkCP_eigen               = { 00.0000, 00.0000, 00.5369 };
	//-----------------------------------------------------//
	Vector3d v_yawlink_pitchbacklinkPA 					= { 1.0,     0.0,    0.0 };
	Vector3d v_yawlink_pitchbacklinkCA 					= { 0.0,     0.0,    1.0 };
  Vector3d v_yawlink_pitchbacklinkPP 					= { 0.0, -0.0000, 0.1624 };
	Vector3d v_yawlink_pitchbacklinkCP 					= { 0.0,     0.0,    0.0 };

	Vector3d v_pitchbacklink_pitchbottomlinkPA 	= {     0.0,     0.0,     1.0 };
	Vector3d v_pitchbacklink_pitchbottomlinkCA 	= {     0.0,     0.0,     1.0 };
	Vector3d v_pitchbacklink_pitchbottomlinkPP 	= { -0.1028, -0.2867,     0.0 };
  Vector3d v_pitchbacklink_pitchbottomlinkCP 	= { -0.0364,  0.0000, -0.0000 };

	Vector3d v_pitchbottomlink_pitchendlinkPA 	= {    0.0,     0.0,     1.0 };
	Vector3d v_pitchbottomlink_pitchendlinkCA 	= {    0.0,     0.0,     1.0 };
  Vector3d v_pitchbottomlink_pitchendlinkPP 	= { 0.3401, -0.0000, -0.0000 };
	Vector3d v_pitchbottomlink_pitchendlinkCP 	= {    0.0,     0.0,  0.0000 };
	//-----------------------------------------------------//
	Vector3d v_yawlink_pitchfrontlinkPA 					= { 1.0,     0.0,    0.0 };
	Vector3d v_yawlink_pitchfrontlinkCA 					= { 0.0,     0.0,    1.0 };
  Vector3d v_yawlink_pitchfrontlinkPP 					= { 0.0, 		 0.0,    0.2 };
	Vector3d v_yawlink_pitchfrontlinkCP 					= { 0.0,     0.0,    0.0 };

	Vector3d v_pitchfrontlink_pitchbottomlinkPA 	= { 	 	0.0,     0.0,     1.0 };
	Vector3d v_pitchfrontlink_pitchbottomlinkCA 	= { 		0.0,     0.0,     1.0 };
	Vector3d v_pitchfrontlink_pitchbottomlinkPP 	= { -0.1031, -0.2868,  	  0.0 };
	Vector3d v_pitchfrontlink_pitchbottomlinkCP 	= { -0.0000, -0.0000, -0.0000 };

	Vector3d v_pitchfrontlink_pitchtoplinkPA 	= { 	 	0.0,     0.0,     1.0 };
	Vector3d v_pitchfrontlink_pitchtoplinkCA 	= { 		0.0,     0.0,     1.0 };
	Vector3d v_pitchfrontlink_pitchtoplinkPP 	= { -0.1084, -0.3242,  	  0.0 };
  Vector3d v_pitchfrontlink_pitchtoplinkCP 	= { -0.0000, -0.0000, -0.0000 };

	Vector3d v_pitchtoplink_pitchendlinkPA 	= {     0.0,     0.0,     1.0 };
	Vector3d v_pitchtoplink_pitchendlinkCA 	= {     0.0,     0.0,     1.0 };
  Vector3d v_pitchtoplink_pitchendlinkPP 	= {  0.3404, -0.0000, -0.000 };
	Vector3d v_pitchtoplink_pitchendlinkCP 	= { -0.0051, -0.0376,  0.000 };
	//-----------------------------------------------------//
	Vector3d v_pitchendlink_maininsertionlinkPA = {     0.0,     1.0,    0.0 };
	Vector3d v_pitchendlink_maininsertionlinkCA = {     1.0,     0.0,    0.0 };
  Vector3d v_pitchendlink_maininsertionlinkPP = {  0.1031, -0.0961, 0.0000 };
	Vector3d v_pitchendlink_maininsertionlinkCP = { -0.0108,  -0.062, 0.0000 };

	Vector3d v_maininsertionlink_toollinkPA 	  = {     1.0,     0.0,    0.0 };
	Vector3d v_maininsertionlink_toollinkCA 	  = {     0.0,     0.0,   -1.0 };
	Vector3d v_maininsertionlink_toollinkPP 	  = { -0.0108,  -0.062,    0.0 };
  Vector3d v_maininsertionlink_toollinkCP 	  = { -0.0000, -0.0000, 0.0118 };
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
	const double q_toollink_ee_childOffset     							  = 0.0;


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
	const double q_toollink_ee_jointOffset      							= 0.0;

  const float baselinkScale{1.0f}, yawlinkScale{1.0f}, pitchbacklinkScale{1.0f}, pitchbottomlinkScale{1.0f}, 
    pitchendlinkScale{1.0f}, maininsertionlinkScale{1.0f}, toollinkScale{1.0f}, pitchfrontlinkScale{1.0f}, 
    pitchtoplinkScale{1.0f};
    
  // Matrix3d baselink_yawlinkRot, yawlink_pitchbacklinkRot, pitchbacklink_pitchbottomlinkRot, 
  //   pitchbottomlink_pitchendlinkRot, yawlink_pitchfrontlinkRot, pitchfrontlink_pitchbottomlinkRot, 
  //   pitchfrontlink_pitchtoplinkRot, pitchtoplink_pitchendlinkRot, 
  //   pitchendlink_maininsertionlinkRot, maininsertionlink_toollinkRot;
  
  // Matrix3d baselink_yawlinkRotOffset, yawlink_pitchbacklinkRotOffset, 
  //   pitchbacklink_pitchbottomlinkRotOffset, pitchbottomlink_pitchendlinkRotOffset, 
  //   yawlink_pitchfrontlinkRotOffset, pitchfrontlink_pitchbottomlinkRotOffset, 
  //   pitchfrontlink_pitchtoplinkRotOffset, pitchtoplink_pitchendlinkRotOffset, 
  //   pitchendlink_maininsertionlinkRotOffset, maininsertionlink_toollinkRotOffset;

  Vector3d v_baselink_yawlinkJAxis, v_yawlink_pitchbacklinkJAxis, 
    v_pitchbacklink_pitchbottomlinkJAxis, v_pitchbottomlink_pitchendlinkJAxis, 
    v_yawlink_pitchfrontlinkJAxis, v_pitchfrontlink_pitchtoplinkJAxis, 
    v_pitchtoplink_pitchendlinkJAxis, v_pitchfrontlink_pitchbottomlinkJAxis, 
    v_pitchendlink_maininsertionlinkJAxis, v_maininsertionlink_toollinkJAxis;

  // SpatialTransform T_W_world_baselinkST, T_W_world_yawlinkST, T_W_world_pitchfrontlinkST, T_W_world_pitchbacklinkST, 
	//   T_W_world_pitchbottomlinkST, T_W_world_pitchendlinkST, T_W_world_maininsertionlinkST, T_W_world_pitchtoplinkST,
	//   T_W_world_toollinkST;

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