#include "ECMTests/ECM2.h"


TEST_CASE_METHOD ( ECM, __FILE__"manual_HomePose", "") 
{
// 4294967295, ROOT
// 2147483646, world-baselink
// 0, baselink-yawlink
// 1, yawlink-pitchbacklink
// 2, pitchbacklink-pitchbottomlink
// 3, pitchbottomlink-pitchendlink
// 4, yawlink-pitchfrontlink
// 5, pitchfrontlink-pitchbottomlink
// 6, pitchfrontlink-pitchtoplink
// 7, pitchtoplink-pitchendlink
// 8, pitchendlink-maininsertionlink
// 9, maininsertionlink-toollink

  // Matrix3d r_w_baselink = world_baselinkST.E;
  // Vector3d p_w_baselink = W_world_baselinkST.r;
  // CHECK_THAT (Matrix3dIdentity, 
  //   AllCloseMatrix(W_world_baselinkST.E, TEST_PREC, TEST_PREC));
  // CHECK_THAT (Vector3d(0.5, -0.4, -0.6), 
  //   AllCloseVector(p_w_baselink, TEST_PREC, TEST_PREC));
  
  // CHECK_THAT (Matrix3d(-1, 0, 0, 0, 0, 1, 0, 1, 0), 
  //   AllCloseMatrix(W_world_yawlinkST.E, TEST_PREC, TEST_PREC));
  // Vector3d p_w_yawlink = p_w_baselink + W_world_baselinkST.E * B_baselink_yawlinkST.r;
  // CHECK_THAT (Vector3d(0.5,   -0.936899,   -0.600006), 
  //   AllCloseVector(p_w_yawlink, TEST_PREC, TEST_PREC));

  // CHECK_THAT (Matrix3d(0, 0, -1, 1, 0, 0, 0, -1, 0), 
  //   AllCloseMatrix(W_world_pitchbacklinkST.E, TEST_PREC, TEST_PREC));
  // Vector3d p_w_pitchbacklink = p_w_yawlink + W_world_yawlinkST.E * B_yawlink_pitchbacklinkST.r;
	// CHECK_THAT (Vector3d(0.500009,   -0.774492,   -0.600079), 
  //   AllCloseVector(p_w_pitchbacklink, TEST_PREC, TEST_PREC));

  // CHECK_THAT (Matrix3d(0, 0, -1, 1, 0, 0, 0, -1, 0), 
  //   AllCloseMatrix(W_world_pitchbottomlinkST.E, TEST_PREC, TEST_PREC));
  // Vector3d p_w_pitchbottomlink = p_w_pitchbacklink + 
  //   W_world_pitchbacklinkST.E * B_pitchbacklink_pitchbottomlinkST.r;
  // CHECK_THAT (Vector3d(0.499951,   -0.841289,   -0.313501), 
  //   AllCloseVector(p_w_pitchbottomlink, TEST_PREC, TEST_PREC));

  // CHECK_THAT (Matrix3d(0, 0, -1, 1, 0, 0, 0, -1, 0), 
  //   AllCloseMatrix(W_world_pitchendlinkST.E, TEST_PREC, TEST_PREC));
  // Vector3d p_w_pitchendlink = p_w_pitchbottomlink + 
  //   W_world_pitchbottomlinkST.E * B_pitchbottomlink_pitchendlinkST.r;
  // CHECK_THAT (Vector3d(0.499978,   -0.501171,   -0.313362), 
  //   AllCloseVector(p_w_pitchendlink, TEST_PREC, TEST_PREC));

  // CHECK_THAT (Matrix3d(0, 0, -1, 1, 0, 0, 0, -1, 0), 
  //   AllCloseMatrix(W_world_pitchfrontlinkST.E, TEST_PREC, TEST_PREC));
  // Vector3d p_w_pitchfrontlink = p_w_yawlink +
  //   W_world_yawlinkST.E * B_yawlink_pitchfrontlinkST.r;
  // CHECK_THAT (Vector3d(0.5,   -0.736909,    -0.59998), 
  //   AllCloseVector(p_w_pitchfrontlink, TEST_PREC, TEST_PREC));
  
  // CHECK_THAT (Matrix3d(0, 0, -1, 1, 0, 0, 0, -1, 0), 
  //   AllCloseMatrix(W_world_pitchtoplinkST.E, TEST_PREC, TEST_PREC));
  // Vector3d p_w_pitchtoplink = p_w_pitchfrontlink +
  //   W_world_pitchfrontlinkST.E * B_pitchfrontlink_pitchtoplinkST.r; 
  // CHECK_THAT (Vector3d(0.499951, -0.846723, -0.276256), 
  //   AllCloseVector(p_w_pitchtoplink, TEST_PREC, TEST_PREC));

  // p_w_pitchendlink = p_w_pitchtoplink + 
  //   W_world_pitchtoplinkST.E * B_pitchtoplink_pitchendlinkST.r;
  // CHECK_THAT (Vector3d(0.499978,   -0.501171,   -0.313362), 
  //   AllCloseVector(p_w_pitchendlink, TEST_PREC, TEST_PREC));

  // CHECK_THAT (Matrix3d(0, 0, -1, 0, -1, 0, -1, 0, 0), 
  //   AllCloseMatrix(W_world_maininsertionlinkST.E, TEST_PREC, TEST_PREC));
  // Vector3d p_w_maininsertionlink = p_w_pitchendlink +
  //   W_world_pitchendlinkST.E * B_pitchendlink_maininsertionlinkST.r;
  // CHECK_THAT (Vector3d(0.499918,   -0.460183,    -0.22929), 
  //   AllCloseVector(p_w_maininsertionlink, TEST_PREC, TEST_PREC));

  // CHECK_THAT (Matrix3d(0, 1, 0, -1, 0, 0, 0, 0, 1), 
  //   AllCloseMatrix(W_world_toollinkST.E, TEST_PREC, TEST_PREC));
  // Vector3d p_w_toollink = p_w_maininsertionlink +
  //   W_world_maininsertionlinkST.E * B_maininsertionlink_toollinkST.r;
  // CHECK_THAT (Vector3d(0.499922,   -0.398184,   -0.230241), 
  //   AllCloseVector(p_w_toollink, TEST_PREC, TEST_PREC));
}

/*
TEST_CASE_METHOD ( ECM, __FILE__"manual_base_yaw_piby2_needtobefixed", "") 
{
  // 0,               baselink-yawlink, 1.5751442909240723
  // 1,          yawlink-pitchbacklink, 2.6517625883570872e-05
  // 2,  pitchbacklink-pitchbottomlink, -0.00032555844518356025
  // 3,   pitchbottomlink-pitchendlink, 0.0004990961169824004
  // 4,         yawlink-pitchfrontlink, -0.002869951305910945
  // 5, pitchfrontlink-pitchbottomlink, 0.002570690819993615
  // 6,    pitchfrontlink-pitchtoplink, 0.0016482401406392455
  // 7,      pitchtoplink-pitchendlink, 0.0014168411726132035
  // 8, pitchendlink-maininsertionlink, -5.958145266049542e-07
  // 9,     maininsertionlink-toollink, -0.02941928431391716


  // Q[0] = 1.5751442909240723;
  // Q[1] = 2.6517625883570872e-05;
  // Q[2] = -0.00032555844518356025;
  // Q[3] = 0.0004990961169824004;
  // Q[4] = -0.002869951305910945;
  // Q[5] = 0.002570690819993615;
  // Q[6] = 0.0016482401406392455;
  // Q[7] = 0.0014168411726132035;
  // Q[8] = -5.958145266049542e-07;
  // Q[9] = -0.02941928431391716;



  SpatialTransform W_world_baselinkST_cal, W_world_yawlinkST_cal, W_world_pitchfrontlinkST_cal, 
  W_world_pitchbacklinkST_cal, W_world_pitchbottomlinkST_cal, W_world_pitchendlinkST_cal, 
  W_world_maininsertionlinkST_cal, W_world_pitchtoplinkST_cal, W_world_toollinkST_cal;


  //--------------------world-base--------------------//
  {
    W_world_baselinkST_cal.E = Matrix3dIdentity;
    W_world_baselinkST_cal.r = W_world_baselinkST.r;

    // Matrix3d r_w_baselink = world_baselinkST.E;
    // Vector3d p_w_baselink = world_baselinkST.r;
    CHECK_THAT (Matrix3dIdentity, 
      AllCloseMatrix(W_world_baselinkST_cal.E, TEST_PREC, TEST_PREC));
    CHECK_THAT (Vector3d(0.5, -0.4, -0.6), 
      AllCloseVector(W_world_baselinkST_cal.r, TEST_PREC, TEST_PREC));
  }
  //--------------------baselink-yawlink--------------------//
  {
    // Rotate by pi/2
    Eigen::Affine3d W_r_baselink_yawlink(
        Eigen::AngleAxisd(M_PI_2, R_W_P_C_Axis(R_W_P_C_AxisEnum::baselink_yawlink)));

    W_world_yawlinkST_cal.E = 
      W_r_baselink_yawlink.rotation() *
      W_world_baselinkST_cal.E * 
      B_baselink_yawlinkST.E;

    CHECK_THAT (
      Matrix3d(
        0, -1,  0, 
        0,  0,  1, 
      -1,  0,  0), 
      AllCloseMatrix(W_world_yawlinkST_cal.E, TEST_PREC, TEST_PREC));
  }
  //--------------------yawlink-pitchbacklink--------------------//
  {
    Eigen::Affine3d W_r_yawlink_pitchbacklink(
        Eigen::AngleAxisd(0, R_W_P_C_Axis(R_W_P_C_AxisEnum::yawlink_pitchbacklink)));

    W_world_pitchbacklinkST_cal.E = 
      W_r_yawlink_pitchbacklink.rotation() *
      W_world_yawlinkST_cal.E * 
      B_yawlink_pitchbacklinkST.E;

    CHECK_THAT (
      Matrix3d(
        0,  1,  0, 
        1,  0,  0, 
        0,  0, -1), 
      AllCloseMatrix(W_world_pitchbacklinkST_cal.E, TEST_PREC, TEST_PREC));
  }
  //--------------------pitchbacklink-pitchbottomlink--------------------//
  {
    Eigen::Affine3d W_r_pitchbacklink_pitchbottomlink(
        Eigen::AngleAxisd(0, R_W_P_C_Axis(R_W_P_C_AxisEnum::pitchbacklink_pitchbottomlink)));

    W_world_pitchbottomlinkST_cal.E = 
      W_r_pitchbacklink_pitchbottomlink.rotation() *
      W_world_pitchbacklinkST_cal.E * 
      B_pitchbacklink_pitchbottomlinkST.E;

    CHECK_THAT (
      Matrix3d(
        0,  1,  0, 
        1,  0,  0, 
        0,  0, -1), 
      AllCloseMatrix(W_world_pitchbottomlinkST_cal.E, TEST_PREC, TEST_PREC));
  }
  //--------------------pitchbottomlink-pitchendlink--------------------//
  {
    Eigen::Affine3d W_r_pitchbottomlink_pitchendlink(
        Eigen::AngleAxisd(0, R_W_P_C_Axis(R_W_P_C_AxisEnum::pitchbottomlink_pitchendlink)));

    W_world_pitchendlinkST_cal.E = 
      W_r_pitchbottomlink_pitchendlink.rotation() *
      W_world_pitchbottomlinkST_cal.E * 
      B_pitchbottomlink_pitchendlinkST.E;

    CHECK_THAT (
      Matrix3d(
        0,  1,  0, 
        1,  0,  0, 
        0,  0, -1), 
      AllCloseMatrix(W_world_pitchendlinkST_cal.E, TEST_PREC, TEST_PREC));
  }
  //--------------------yawlink_pitchfrontlink--------------------//
  {
    Eigen::Affine3d W_r_yawlink_pitchfrontlink(
        Eigen::AngleAxisd(0, R_W_P_C_Axis(R_W_P_C_AxisEnum::yawlink_pitchfrontlink)));

    W_world_pitchfrontlinkST_cal.E = 
      W_r_yawlink_pitchfrontlink.rotation() *
      W_world_yawlinkST_cal.E * 
      B_yawlink_pitchfrontlinkST.E;

    CHECK_THAT (
      Matrix3d(
        0,  1,  0, 
        1,  0,  0, 
        0,  0, -1), 
      AllCloseMatrix(W_world_pitchfrontlinkST_cal.E, TEST_PREC, TEST_PREC));
  }
  //--------------------pitchfrontlink_pitchbottomlink--------------------//
  {
    Eigen::Affine3d W_r_pitchfrontlink_pitchbottomlink(
        Eigen::AngleAxisd(0, R_W_P_C_Axis(R_W_P_C_AxisEnum::pitchfrontlink_pitchbottomlink)));

    W_world_pitchbottomlinkST_cal.E = 
      W_r_pitchfrontlink_pitchbottomlink.rotation() *
      W_world_pitchfrontlinkST_cal.E * 
      B_pitchfrontlink_pitchbottomlinkST.E;

    CHECK_THAT (
      Matrix3d(
        0,  1,  0, 
        1,  0,  0, 
        0,  0, -1), 
      AllCloseMatrix(W_world_pitchbottomlinkST_cal.E, TEST_PREC, TEST_PREC));
  }
  //--------------------pitchfrontlink-pitchtoplink--------------------//
  {
    Eigen::Affine3d W_r_pitchfrontlink_pitchtoplink(
        Eigen::AngleAxisd(0, R_W_P_C_Axis(R_W_P_C_AxisEnum::pitchfrontlink_pitchtoplink)));

    W_world_pitchtoplinkST_cal.E = 
      W_r_pitchfrontlink_pitchtoplink.rotation() *
      W_world_pitchfrontlinkST_cal.E * 
      B_pitchfrontlink_pitchtoplinkST.E;

    CHECK_THAT (
      Matrix3d(
        0,  1,  0, 
        1,  0,  0, 
        0,  0, -1), 
      AllCloseMatrix(W_world_pitchtoplinkST_cal.E, TEST_PREC, TEST_PREC));
  }
  //--------------------pitchtoplink_pitchendlink--------------------//
  {
    Eigen::Affine3d W_r_pitchtoplink_pitchendlink(
        Eigen::AngleAxisd(0, R_W_P_C_Axis(R_W_P_C_AxisEnum::pitchtoplink_pitchendlink)));

    W_world_pitchendlinkST_cal.E = 
      W_r_pitchtoplink_pitchendlink.rotation() *
      W_world_pitchtoplinkST_cal.E * 
      B_pitchtoplink_pitchendlinkST.E;

    CHECK_THAT (
      Matrix3d(
        0,  1,  0, 
        1,  0,  0, 
        0,  0, -1), 
      AllCloseMatrix(W_world_pitchendlinkST_cal.E, TEST_PREC, TEST_PREC));
  }
  //--------------------pitchendlink_maininsertionlink--------------------//
  {
    // Prismatic joint
    // Eigen::Affine3d W_r_pitchendlink_maininsertionlink(
    //     Eigen::AngleAxisd(0, -Vector3d::UnitZ()));

    W_world_maininsertionlinkST_cal.E = 
      // W_r_pitchendlink_maininsertionlink.rotation() *
      W_world_pitchendlinkST_cal.E * 
      B_pitchendlink_maininsertionlinkST.E;

    CHECK_THAT (
      Matrix3d(
        1,  0,  0, 
        0, -1,  0, 
        0,  0, -1), 
      AllCloseMatrix(W_world_maininsertionlinkST_cal.E, TEST_PREC, TEST_PREC));
  }
  //--------------------maininsertionlink-toollink--------------------//
  {
    Eigen::Affine3d W_r_maininsertionlink_toollink(
        Eigen::AngleAxisd(0, R_W_P_C_Axis(R_W_P_C_AxisEnum::maininsertionlink_toollink)));

    W_world_toollinkST_cal.E = 
      W_r_maininsertionlink_toollink.rotation() *
      W_world_maininsertionlinkST_cal.E * 
      B_maininsertionlink_toollinkST.E;

    CHECK_THAT (
      Matrix3d(
        0,  0, -1, 
       -1,  0,  0, 
        0,  1,  0), 
      AllCloseMatrix(W_world_toollinkST_cal.E, TEST_PREC, TEST_PREC));
  }
}
*/


/*
TEST_CASE_METHOD ( ECM, __FILE__"manual_base_yaw_piby2__main_tool_piby2", "") 
{
// 0,               baselink-yawlink, 1.5745775699615479
// 1,          yawlink-pitchbacklink, 2.5622757675591856e-05
// 2,  pitchbacklink-pitchbottomlink, -0.0003239346551708877
// 3,   pitchbottomlink-pitchendlink, 0.0004958109930157661
// 4,         yawlink-pitchfrontlink, -0.0028719608671963215
// 5, pitchfrontlink-pitchbottomlink, 0.00257349805906415
// 6,    pitchfrontlink-pitchtoplink, 0.0016522470396012068
// 7,      pitchtoplink-pitchendlink, 0.001413798308931291
// 8, pitchendlink-maininsertionlink, 0.09998920559883118
// 9,     maininsertionlink-toollink, 1.5367075204849243


  SpatialTransform W_world_baselinkST_cal, W_world_yawlinkST_cal, W_world_pitchfrontlinkST_cal, 
  W_world_pitchbacklinkST_cal, W_world_pitchbottomlinkST_cal, W_world_pitchendlinkST_cal, 
  W_world_maininsertionlinkST_cal, W_world_pitchtoplinkST_cal, W_world_toollinkST_cal;


  //--------------------world-base--------------------//
  {
    W_world_baselinkST_cal.E = Matrix3dIdentity;
    W_world_baselinkST_cal.r = W_world_baselinkST.r;

    // Matrix3d r_w_baselink = world_baselinkST.E;
    // Vector3d p_w_baselink = world_baselinkST.r;
    CHECK_THAT (Matrix3dIdentity, 
      AllCloseMatrix(W_world_baselinkST_cal.E, TEST_PREC, TEST_PREC));
    CHECK_THAT (Vector3d(0.5, -0.4, -0.6), 
      AllCloseVector(W_world_baselinkST_cal.r, TEST_PREC, TEST_PREC));
  }
  //--------------------baselink-yawlink--------------------//
  {
    // Rotate by pi/2
    Eigen::Affine3d W_r_baselink_yawlink(
        Eigen::AngleAxisd(M_PI_2, baselink_yawlinkJAxis));

    W_world_yawlinkST_cal.E = 
      W_world_baselinkST_cal.E * 
      W_r_baselink_yawlink.rotation() *
      baselink_yawlinkRotOffset *
      baselink_yawlinkRot;


    CHECK_THAT (
      Matrix3d(
        0, -1,  0, 
        0,  0,  1, 
       -1,  0,  0), 
      AllCloseMatrix(W_world_yawlinkST_cal.E, TEST_PREC, TEST_PREC));
    // std::cout << "#-----------------------------------\n";
  }
  //--------------------yawlink-pitchbacklink--------------------//
  {
    Eigen::Affine3d W_r_yawlink_pitchbacklink(
        Eigen::AngleAxisd(0, yawlink_pitchbacklinkJAxis));

    W_world_pitchbacklinkST_cal.E = 
      W_r_yawlink_pitchbacklink.rotation() *
      W_world_yawlinkST_cal.E * 
      B_yawlink_pitchbacklinkST.E;

    CHECK_THAT (
      Matrix3d(
        0,  1,  0, 
        1,  0,  0, 
        0,  0, -1), 
      AllCloseMatrix(W_world_pitchbacklinkST_cal.E, TEST_PREC, TEST_PREC));
  }
  //--------------------pitchbacklink-pitchbottomlink--------------------//
  {
    Eigen::Affine3d W_r_pitchbacklink_pitchbottomlink(
        Eigen::AngleAxisd(0, pitchbacklink_pitchbottomlinkJAxis));

    W_world_pitchbottomlinkST_cal.E = 
      W_r_pitchbacklink_pitchbottomlink.rotation() *
      W_world_pitchbacklinkST_cal.E * 
      B_pitchbacklink_pitchbottomlinkST.E;

    CHECK_THAT (
      Matrix3d(
        0,  1,  0, 
        1,  0,  0, 
        0,  0, -1), 
      AllCloseMatrix(W_world_pitchbottomlinkST_cal.E, TEST_PREC, TEST_PREC));
  }
  //--------------------pitchbottomlink-pitchendlink--------------------//
  {
    Eigen::Affine3d W_r_pitchbottomlink_pitchendlink(
        Eigen::AngleAxisd(0, pitchbottomlink_pitchendlinkJAxis));

    W_world_pitchendlinkST_cal.E = 
      W_r_pitchbottomlink_pitchendlink.rotation() *
      W_world_pitchbottomlinkST_cal.E * 
      B_pitchbottomlink_pitchendlinkST.E;

    CHECK_THAT (
      Matrix3d(
        0,  1,  0, 
        1,  0,  0, 
        0,  0, -1), 
      AllCloseMatrix(W_world_pitchendlinkST_cal.E, TEST_PREC, TEST_PREC));
  }
  // --------------------yawlink_pitchfrontlink--------------------//
  {
    Eigen::Affine3d W_r_yawlink_pitchfrontlink(
        Eigen::AngleAxisd(0, yawlink_pitchfrontlinkJAxis));

    W_world_pitchfrontlinkST_cal.E = 
      W_r_yawlink_pitchfrontlink.rotation() *
      W_world_yawlinkST_cal.E *
      yawlink_pitchfrontlinkRotOffset *
      yawlink_pitchfrontlinkRot;

    CHECK_THAT (
      Matrix3d(
        0,  1,  0, 
        1,  0,  0, 
        0,  0, -1), 
      AllCloseMatrix(W_world_pitchfrontlinkST_cal.E, TEST_PREC, TEST_PREC));
  }
  //--------------------pitchfrontlink_pitchbottomlink--------------------//
  
  {
    Eigen::Affine3d W_r_pitchfrontlink_pitchbottomlink(
        Eigen::AngleAxisd(0, pitchfrontlink_pitchbottomlinkJAxis));

    W_world_pitchbottomlinkST_cal.E = 
    W_r_pitchfrontlink_pitchbottomlink.rotation() *
    W_world_pitchfrontlinkST_cal.E *
    pitchfrontlink_pitchbottomlinkRotOffset *
    pitchfrontlink_pitchbottomlinkRot.transpose();


    CHECK_THAT (
      Matrix3d(
        0,  1,  0, 
        1,  0,  0, 
        0,  0, -1), 
      AllCloseMatrix(W_world_pitchbottomlinkST_cal.E, TEST_PREC, TEST_PREC));
  }
  //--------------------pitchfrontlink-pitchtoplink--------------------//
  {
    Eigen::Affine3d W_r_pitchfrontlink_pitchtoplink(
        Eigen::AngleAxisd(0, pitchfrontlink_pitchtoplinkJAxis));

    W_world_pitchtoplinkST_cal.E = 
    W_r_pitchfrontlink_pitchtoplink.rotation() *
    W_world_pitchfrontlinkST_cal.E *
    pitchfrontlink_pitchtoplinkRotOffset *
    pitchfrontlink_pitchtoplinkRot.transpose();

    CHECK_THAT (
      Matrix3d(
        0,  1,  0, 
        1,  0,  0, 
        0,  0, -1), 
      AllCloseMatrix(W_world_pitchtoplinkST_cal.E, TEST_PREC, TEST_PREC));
  }
  //--------------------pitchtoplink_pitchendlink--------------------//
  {
    Eigen::Affine3d W_r_pitchtoplink_pitchendlink(
        Eigen::AngleAxisd(0, pitchtoplink_pitchendlinkJAxis));

    W_world_pitchendlinkST_cal.E = 
      W_r_pitchtoplink_pitchendlink.rotation() *
      W_world_pitchtoplinkST_cal.E *
      pitchtoplink_pitchendlinkRotOffset *
      pitchtoplink_pitchendlinkRot.transpose();
 
    CHECK_THAT (
      Matrix3d(
        0,  1,  0, 
        1,  0,  0, 
        0,  0, -1), 
      AllCloseMatrix(W_world_pitchendlinkST_cal.E, TEST_PREC, TEST_PREC));
  }
  //--------------------pitchendlink_maininsertionlink--------------------//
  {
    // Prismatic joint
    W_world_maininsertionlinkST_cal.E = 
      // W_r_maininsertionlink_toollink.rotation() *
      W_world_pitchendlinkST_cal.E *
      pitchendlink_maininsertionlinkRotOffset *
      pitchendlink_maininsertionlinkRot.transpose();

    CHECK_THAT (
      Matrix3d(
        1,  0,  0, 
        0, -1,  0, 
        0,  0, -1), 
      AllCloseMatrix(W_world_maininsertionlinkST_cal.E, TEST_PREC, TEST_PREC));
  }
  //--------------------maininsertionlink-toollink--------------------//
  {
    Eigen::Affine3d W_r_maininsertionlink_toollink(
        Eigen::AngleAxisd(M_PI_2, maininsertionlink_toollinkJAxis));

    W_world_toollinkST_cal.E = 
      W_world_maininsertionlinkST_cal.E *
      W_r_maininsertionlink_toollink.rotation() *
      maininsertionlink_toollinkRotOffset *
      maininsertionlink_toollinkRot;


    // W_world_toollinkST_cal.E = 
    //   W_r_maininsertionlink_toollink.rotation() *
    //   W_world_maininsertionlinkST_cal.E * 
    //   B_maininsertionlink_toollinkST.E;

    // CHECK_THAT (
    //   Matrix3d(
    //     0,  0, -1, 
    //     0, -1,  0, 
    //    -1,  0,  0), 
    //   AllCloseMatrix(W_world_toollinkST_cal.E, TEST_PREC, TEST_PREC));
    // std::cout << "#-----------------------------------\n";
  }  
}
*/

/*
TEST_CASE_METHOD ( ECM, __FILE__"_manual_All_Controllable_Body_Rotate_PI_2", "") 
{
// 0,               baselink-yawlink, 1.5711669921875
// 1,          yawlink-pitchbacklink, 1.5707159042358398
// 2,  pitchbacklink-pitchbottomlink, -1.5399764776229858
// 3,   pitchbottomlink-pitchendlink, 1.5721415281295776
// 4,         yawlink-pitchfrontlink, 1.5738632678985596
// 5, pitchfrontlink-pitchbottomlink, -1.5431238412857056
// 6,    pitchfrontlink-pitchtoplink, -1.5386755466461182
// 7,      pitchtoplink-pitchendlink, 1.5676918029785156
// 8, pitchendlink-maininsertionlink, -3.4054405659844633e-06
// 9,     maininsertionlink-toollink, 1.5421557426452637


  SpatialTransform T_W_world_baselink_cal, T_W_world_yawlink_cal, T_W_world_pitchfrontlink_cal, 
  T_W_world_pitchbacklink_cal, T_W_world_pitchbottomlink_cal, T_W_world_pitchendlink_cal, 
  T_W_world_maininsertionlink_cal, T_W_world_pitchtoplink_cal, T_W_world_toollink_cal;


  //--------------------world-base--------------------//
  {
    T_W_world_baselink_cal.E = Matrix3dIdentity;
    T_W_world_baselink_cal.r = W_world_baselinkST.r;

    // Matrix3d r_w_baselink = world_baselinkST.E;
    // Vector3d p_w_baselink = world_baselinkST.r;
    CHECK_THAT (Matrix3dIdentity, 
      AllCloseMatrix(T_W_world_baselink_cal.E, TEST_PREC, TEST_PREC));
    CHECK_THAT (Vector3d(0.5, -0.4, -0.6), 
      AllCloseVector(T_W_world_baselink_cal.r, TEST_PREC, TEST_PREC));
  }
  //--------------------baselink-yawlink--------------------//
  {
    // // Rotate by pi/2
    // Eigen::Affine3d R_W_baselink_yawlink(
    //     Eigen::AngleAxisd(M_PI_2, baselink_yawlinkJAxis));

    // T_W_world_yawlink_cal.E = 
    //   R_W_baselink_yawlink.rotation() *
    //   T_W_world_baselink_cal.E *
    //   B_baselink_yawlinkST.E;

    Eigen::Affine3d R_B_baselink_yawlink(
        Eigen::AngleAxisd(M_PI_2, baselink_yawlinkPA));

    T_W_world_yawlink_cal.E = 
      T_W_world_baselink_cal.E *
      R_B_baselink_yawlink.rotation() *
      B_baselink_yawlinkST.E;

    T_W_world_yawlink_cal.r = T_W_world_baselink_cal.r +
      T_W_world_baselink_cal.E * B_baselink_yawlinkST.r;

    CHECK_THAT (
      Matrix3d(
        0, -1,  0, 
        0,  0,  1, 
       -1,  0,  0), 
      AllCloseMatrix(T_W_world_yawlink_cal.E, TEST_PREC, TEST_PREC));
    CHECK_THAT (Vector3d(0.499985,   -0.936864,   -0.599992), 
      AllCloseVector(T_W_world_yawlink_cal.r, TEST_PREC, TEST_PREC));
  }
  //--------------------yawlink-pitchbacklink--------------------//
  {
    // Eigen::Affine3d R_W_yawlink_pitchbacklink(
    //     Eigen::AngleAxisd(M_PI_2, yawlink_pitchbacklinkJAxis));
    Eigen::Affine3d R_B_yawlink_pitchbacklink(
        Eigen::AngleAxisd(M_PI_2, yawlink_pitchbacklinkPA));

    T_W_world_pitchbacklink_cal.E = 
      T_W_world_yawlink_cal.E *
      R_B_yawlink_pitchbacklink.rotation() *
      B_yawlink_pitchbacklinkST.E;

    T_W_world_pitchbacklink_cal.r = T_W_world_yawlink_cal.r +
      T_W_world_yawlink_cal.E * B_yawlink_pitchbacklinkST.r;

    CHECK_THAT (
      Matrix3d(
        1,  0,  0, 
        0, -1,  0, 
        0,  0, -1), 
      AllCloseMatrix(T_W_world_pitchbacklink_cal.E, TEST_PREC, TEST_PREC));
    
    CHECK_THAT (Vector3d(0.49979,   -0.773921,   -0.600031), 
      AllCloseVector(T_W_world_pitchbacklink_cal.r, TEST_PREC, TEST_PREC));
  }
  //--------------------pitchbacklink-pitchbottomlink--------------------//
  {
    Eigen::Affine3d R_B_pitchbacklink_pitchbottomlink(
        Eigen::AngleAxisd(-M_PI_2, pitchbacklink_pitchbottomlinkPA));

    T_W_world_pitchbottomlink_cal.E = 
      T_W_world_pitchbacklink_cal.E *
      R_B_pitchbacklink_pitchbottomlink.rotation() *
      B_pitchbacklink_pitchbottomlinkST.E;

    T_W_world_pitchbottomlink_cal.r = T_W_world_pitchbacklink_cal.r +
      T_W_world_pitchbacklink_cal.E * B_pitchbacklink_pitchbottomlinkST.r;

    CHECK_THAT (
      Matrix3d(
        0,  1,  0, 
        1,  0,  0, 
        0,  0, -1), 
      AllCloseMatrix(T_W_world_pitchbottomlink_cal.E, TEST_PREC, TEST_PREC));
    // CHECK_THAT (Vector3d(0.39818,   -0.450651,    -0.60011), 
    //   AllCloseVector(T_W_world_pitchbottomlink_cal.r, TEST_PREC, TEST_PREC));
  }
  //--------------------pitchbottomlink-pitchendlink--------------------//
  {
    Eigen::Affine3d R_B_pitchbottomlink_pitchendlink(
        Eigen::AngleAxisd(M_PI_2, pitchbottomlink_pitchendlinkPA));

    T_W_world_pitchendlink_cal.E = 
      T_W_world_pitchbottomlink_cal.E *
      R_B_pitchbottomlink_pitchendlink.rotation() *
      B_pitchbottomlink_pitchendlinkST.E;

    CHECK_THAT (
      Matrix3d(
        1,  0,  0, 
        0, -1,  0, 
        0,  0, -1), 
      AllCloseMatrix(T_W_world_pitchendlink_cal.E, TEST_PREC, TEST_PREC));
  }
  // --------------------yawlink_pitchfrontlink--------------------//
  {
    Eigen::Affine3d R_B_yawlink_pitchfrontlink(
      Eigen::AngleAxisd(M_PI_2, yawlink_pitchfrontlinkPA));

    T_W_world_pitchfrontlink_cal.E = 
      T_W_world_yawlink_cal.E *
      R_B_yawlink_pitchfrontlink.rotation() *
      B_yawlink_pitchfrontlinkST.E;

    T_W_world_pitchfrontlink_cal.r = T_W_world_yawlink_cal.r +
      T_W_world_yawlink_cal.E * B_yawlink_pitchfrontlinkST.r;

    CHECK_THAT (
      Matrix3d(
        1,  0,  0, 
        0, -1,  0, 
        0,  0, -1), 
      AllCloseMatrix(T_W_world_pitchfrontlink_cal.E, TEST_PREC, TEST_PREC));
    CHECK_THAT (Vector3d(0.500063,    -0.73706,   -0.600002), 
      AllCloseVector(T_W_world_pitchfrontlink_cal.r, TEST_PREC, TEST_PREC));
  }
  //--------------------pitchfrontlink_pitchbottomlink--------------------//
  
  {
    Eigen::Affine3d R_B_pitchfrontlink_pitchbottomlink(
        Eigen::AngleAxisd(-M_PI_2, pitchfrontlink_pitchbottomlinkPA));

    T_W_world_pitchbottomlink_cal.E = 
      T_W_world_pitchfrontlink_cal.E *
      R_B_pitchfrontlink_pitchbottomlink.rotation() *
      B_pitchfrontlink_pitchbottomlinkST.E;

    T_W_world_pitchbottomlink_cal.r = T_W_world_pitchfrontlink_cal.r +
      T_W_world_pitchfrontlink_cal.E * B_pitchfrontlink_pitchbottomlinkST.r;


    CHECK_THAT (
      Matrix3d(
        0,  1,  0, 
        1,  0,  0, 
        0,  0, -1),
      AllCloseMatrix(T_W_world_pitchbottomlink_cal.E, TEST_PREC, TEST_PREC));
  }
  //--------------------pitchfrontlink-pitchtoplink--------------------//
  {
    Eigen::Affine3d R_B_pitchfrontlink_pitchtoplink(
        Eigen::AngleAxisd(-M_PI_2, pitchfrontlink_pitchtoplinkPA));

    T_W_world_pitchtoplink_cal.E = 
      T_W_world_pitchfrontlink_cal.E *
      R_B_pitchfrontlink_pitchtoplink.rotation() *
      B_pitchfrontlink_pitchtoplinkST.E;


    CHECK_THAT (
      Matrix3d(
        0,  1,  0, 
        1,  0,  0, 
        0,  0, -1), 
      AllCloseMatrix(T_W_world_pitchtoplink_cal.E, TEST_PREC, TEST_PREC));
  }
  //--------------------pitchtoplink_pitchendlink--------------------//
  {
    Eigen::Affine3d R_B_pitchtoplink_pitchendlink(
        Eigen::AngleAxisd(M_PI_2, pitchtoplink_pitchendlinkPA));

    T_W_world_pitchendlink_cal.E = 
      T_W_world_pitchtoplink_cal.E *
      R_B_pitchtoplink_pitchendlink.rotation() *
      B_pitchtoplink_pitchendlinkST.E;
 
    CHECK_THAT (
      Matrix3d(
        1,  0,  0, 
        0, -1,  0, 
        0,  0, -1), 
      AllCloseMatrix(T_W_world_pitchendlink_cal.E, TEST_PREC, TEST_PREC));
  }
  //--------------------pitchendlink_maininsertionlink--------------------//
  {
    // Prismatic joint

    Eigen::Affine3d R_B_pitchendlink_maininsertionlink(
        Eigen::AngleAxisd(0, pitchendlink_maininsertionlinkPA));

    T_W_world_maininsertionlink_cal.E = 
      T_W_world_pitchendlink_cal.E *
      R_B_pitchendlink_maininsertionlink.rotation() *
      B_pitchendlink_maininsertionlinkST.E;

    CHECK_THAT (
      Matrix3d(
        0, -1,  0, 
       -1,  0,  0, 
        0,  0, -1), 
      AllCloseMatrix(T_W_world_maininsertionlink_cal.E, TEST_PREC, TEST_PREC));
  }
  //--------------------maininsertionlink-toollink--------------------//
  {
    // Had to use negative of this actual value to match with AMBF value
    Eigen::Affine3d R_B_maininsertionlink_toollink(
        Eigen::AngleAxisd(-M_PI_2, maininsertionlink_toollinkPA));

    T_W_world_toollink_cal.E = 
      T_W_world_maininsertionlink_cal.E *
      R_B_maininsertionlink_toollink.rotation() *
      B_maininsertionlink_toollinkST.E;

    CHECK_THAT (
      Matrix3d(
        0, -1,  0, 
        0,  0,  1, 
       -1,  0,  0), 
      AllCloseMatrix(T_W_world_toollink_cal.E, TEST_PREC, TEST_PREC));
  }  
}
*/

/*
TEST_CASE_METHOD ( ECM, __FILE__"_manual_All_Controllable_Body_Rotate_PI_4_AMBF_Reference", "") 
{
// 0,               baselink-yawlink, 0.7877469062805176
// 1,          yawlink-pitchbacklink, 0.788750946521759
// 2,  pitchbacklink-pitchbottomlink, -0.7828410863876343
// 3,   pitchbottomlink-pitchendlink, 0.7863313555717468
// 4,         yawlink-pitchfrontlink, 0.7869123220443726
// 5, pitchfrontlink-pitchbottomlink, -0.7810024619102478
// 6,    pitchfrontlink-pitchtoplink, -0.7796486020088196
// 7,      pitchtoplink-pitchendlink, 0.7849762439727783
// 8, pitchendlink-maininsertionlink, 0.10069068521261215
// 9,     maininsertionlink-toollink, 0.7648343443870544

  SpatialTransform T_W_world_baselink_cal, T_W_world_yawlink_cal, T_W_world_pitchfrontlink_cal, 
  T_W_world_pitchbacklink_cal, T_W_world_pitchbottomlink_cal, T_W_world_pitchendlink_cal, 
  T_W_world_maininsertionlink_cal, T_W_world_pitchtoplink_cal, T_W_world_toollink_cal;


  //--------------------world-base--------------------//
  {
    T_W_world_baselink_cal.E = Matrix3dIdentity;
    T_W_world_baselink_cal.r = W_world_baselinkST.r;

    // Matrix3d r_w_baselink = world_baselinkST.E;
    // Vector3d p_w_baselink = world_baselinkST.r;
    CHECK_THAT (Matrix3dIdentity, 
      AllCloseMatrix(T_W_world_baselink_cal.E, TEST_PREC, TEST_PREC));
    CHECK_THAT (Vector3d(0.5, -0.4, -0.6), 
      AllCloseVector(T_W_world_baselink_cal.r, TEST_PREC, TEST_PREC));
  }
  //--------------------baselink-yawlink--------------------//
  {
    Eigen::Affine3d R_B_baselink_yawlink(
        Eigen::AngleAxisd(0.788750946521759, baselink_yawlinkPA));

    T_W_world_yawlink_cal.E = 
      T_W_world_baselink_cal.E *
      R_B_baselink_yawlink.rotation() *
      B_baselink_yawlinkST.E;

    T_W_world_yawlink_cal.r = T_W_world_baselink_cal.r +
      T_W_world_baselink_cal.E * B_baselink_yawlinkST.r;

    CHECK_THAT (
      Matrix3d(
        -0.705307, -0.708902,     0,
                0,         0,     1,
        -0.708902,  0.705307,     0), 
      AllCloseMatrix(T_W_world_yawlink_cal.E, TEST_PREC, TEST_PREC));
    
    CHECK_THAT (Vector3d(0.499988,   -0.936889,   -0.599982), 
      AllCloseVector(T_W_world_yawlink_cal.r, TEST_PREC, TEST_PREC));
  }
  //--------------------yawlink-pitchbacklink--------------------//
  {
    Eigen::Affine3d R_B_yawlink_pitchbacklink(
        Eigen::AngleAxisd(0.788750946521759, yawlink_pitchbacklinkPA));


    T_W_world_pitchbacklink_cal.E = 
      T_W_world_yawlink_cal.E *
      R_B_yawlink_pitchbacklink.rotation() *
      B_yawlink_pitchbacklinkST.E;


    T_W_world_pitchbacklink_cal.r = T_W_world_yawlink_cal.r +
      T_W_world_yawlink_cal.E * B_yawlink_pitchbacklinkST.r;

    CHECK_THAT (
      Matrix3d(
        0.502623,    0.499695,    -0.705461,
        0.704721,   -0.709485, -0.000449693,
       -0.500739,   -0.496927,    -0.708748), 
      AllCloseMatrix(T_W_world_pitchbacklink_cal.E, TEST_PREC, TEST_PREC));
    
    CHECK_THAT (Vector3d(0.49979,   -0.774317,   -0.599829), 
      AllCloseVector(T_W_world_pitchbacklink_cal.r, TEST_PREC, TEST_PREC));
  }
  //--------------------pitchbacklink-pitchbottomlink--------------------//
  {
    Eigen::Affine3d R_B_pitchbacklink_pitchbottomlink(
        Eigen::AngleAxisd(-0.7828410863876343, pitchbacklink_pitchbottomlinkPA));

    T_W_world_pitchbottomlink_cal.E = 
      T_W_world_pitchbacklink_cal.E *
      R_B_pitchbacklink_pitchbottomlink.rotation() *
      B_pitchbacklink_pitchbottomlinkST.E;

    T_W_world_pitchbottomlink_cal.r = T_W_world_pitchbacklink_cal.r +
      T_W_world_pitchbacklink_cal.E * B_pitchbacklink_pitchbottomlinkST.r;

    CHECK_THAT (
      Matrix3d(
        0.00356559,    0.709154,    -0.705044,
          0.999982, -0.00592559, -0.000902961,
       -0.00481814,   -0.705029,    -0.709163), 

      AllCloseMatrix(T_W_world_pitchbottomlink_cal.E, TEST_PREC, TEST_PREC));
    CHECK_THAT (Vector3d(0.305009,   -0.606879,    -0.40612), 
      AllCloseVector(T_W_world_pitchbottomlink_cal.r, TEST_PREC, TEST_PREC));
  }
  //--------------------pitchbottomlink-pitchendlink--------------------//
  {
    Eigen::Affine3d R_B_pitchbottomlink_pitchendlink(
        Eigen::AngleAxisd(0.7863313555717468, pitchbottomlink_pitchendlinkPA));

    T_W_world_pitchendlink_cal.E = 
      T_W_world_pitchbottomlink_cal.E *
      R_B_pitchbottomlink_pitchendlink.rotation() *
      B_pitchbottomlink_pitchendlinkST.E;

    T_W_world_pitchendlink_cal.r = T_W_world_pitchbottomlink_cal.r +
      T_W_world_pitchbottomlink_cal.E * B_pitchbottomlink_pitchendlinkST.r;

    CHECK_THAT (
      Matrix3d(
        0.505005,    0.499067,   -0.704203,
        0.702241,   -0.711939, -0.000950852,
       -0.501824,    -0.49404,   -0.709998), 
      AllCloseMatrix(T_W_world_pitchendlink_cal.E, TEST_PREC, TEST_PREC));
    CHECK_THAT (Vector3d(0.306143,   -0.266613,   -0.407677), 
      AllCloseVector(T_W_world_pitchendlink_cal.r, TEST_PREC, TEST_PREC));
  }
  // --------------------yawlink_pitchfrontlink--------------------//
  {
    Eigen::Affine3d R_B_yawlink_pitchfrontlink(
      Eigen::AngleAxisd(0.7869123220443726, yawlink_pitchfrontlinkPA));

    T_W_world_pitchfrontlink_cal.E = 
      T_W_world_yawlink_cal.E *
      R_B_yawlink_pitchfrontlink.rotation() *
      B_yawlink_pitchfrontlinkST.E;

    T_W_world_pitchfrontlink_cal.r = T_W_world_yawlink_cal.r +
      T_W_world_yawlink_cal.E * B_yawlink_pitchfrontlinkST.r;

    CHECK_THAT (
      Matrix3d(
        0.501726,    0.500648,    -0.705424,
        0.706024,   -0.708188, -0.000457427,
       -0.499802,   -0.497817,    -0.708786), 
      AllCloseMatrix(T_W_world_pitchfrontlink_cal.E, TEST_PREC, TEST_PREC));
    CHECK_THAT (Vector3d(0.500063,    -0.73706,   -0.600002), 
      AllCloseVector(T_W_world_pitchfrontlink_cal.r, TEST_PREC, TEST_PREC));
    CHECK_THAT (Vector3d(0.500045,   -0.736933,   -0.600062), 
      AllCloseVector(T_W_world_pitchfrontlink_cal.r, TEST_PREC, TEST_PREC));
  }
  //--------------------pitchfrontlink_pitchbottomlink--------------------//
  
  {
    Eigen::Affine3d R_B_pitchfrontlink_pitchbottomlink(
        Eigen::AngleAxisd(-0.7810024619102478, pitchfrontlink_pitchbottomlinkPA));

    T_W_world_pitchbottomlink_cal.E = 
      T_W_world_pitchfrontlink_cal.E *
      R_B_pitchfrontlink_pitchbottomlink.rotation() *
      B_pitchfrontlink_pitchbottomlinkST.E;

    T_W_world_pitchbottomlink_cal.r = T_W_world_pitchfrontlink_cal.r +
      T_W_world_pitchfrontlink_cal.E * B_pitchfrontlink_pitchbottomlinkST.r;

    CHECK_THAT (
      Matrix3d(
        0.00356559,    0.709154,    -0.705044,
          0.999982, -0.00592559, -0.000902961,
       -0.00481814,   -0.705029,    -0.709163), 

      AllCloseMatrix(T_W_world_pitchbottomlink_cal.E, TEST_PREC, TEST_PREC));
    CHECK_THAT (Vector3d(0.305009,   -0.606879,    -0.40612), 
      AllCloseVector(T_W_world_pitchbottomlink_cal.r, TEST_PREC, TEST_PREC));
  }
  //--------------------pitchfrontlink-pitchtoplink--------------------//
  {
    Eigen::Affine3d R_B_pitchfrontlink_pitchtoplink(
        Eigen::AngleAxisd(-0.7796486020088196, pitchfrontlink_pitchtoplinkPA));

    T_W_world_pitchtoplink_cal.E = 
      T_W_world_pitchfrontlink_cal.E *
      R_B_pitchfrontlink_pitchtoplink.rotation() *
      B_pitchfrontlink_pitchtoplinkST.E;

    T_W_world_pitchtoplink_cal.r = T_W_world_pitchfrontlink_cal.r +
      T_W_world_pitchfrontlink_cal.E * B_pitchfrontlink_pitchtoplinkST.r;

    CHECK_THAT (
      Matrix3d(
        0.00428893,    0.710532,   -0.703652,
          0.999973, -0.00727851,  -0.0012546,
       -0.00601297,   -0.703627,   -0.710544), 
      AllCloseMatrix(T_W_world_pitchtoplink_cal.E, TEST_PREC, TEST_PREC));
    CHECK_THAT (Vector3d(0.283347,    -0.58387,   -0.384493), 
      AllCloseVector(T_W_world_pitchtoplink_cal.r, TEST_PREC, TEST_PREC));
  }
  //--------------------pitchtoplink_pitchendlink--------------------//
  {
    Eigen::Affine3d R_B_pitchtoplink_pitchendlink(
        Eigen::AngleAxisd(0.7849762439727783, pitchtoplink_pitchendlinkPA));

    T_W_world_pitchendlink_cal.E = 
      T_W_world_pitchtoplink_cal.E *
      R_B_pitchtoplink_pitchendlink.rotation() *
      B_pitchtoplink_pitchendlinkST.E;

    T_W_world_pitchendlink_cal.r = T_W_world_pitchtoplink_cal.r +
      T_W_world_pitchtoplink_cal.E * B_pitchtoplink_pitchendlinkST.r;

    CHECK_THAT (
      Matrix3d(
        0.505005,    0.499067,    -0.704203,
        0.702241,   -0.711939, -0.000950852,
      -0.501824,     -0.49404,    -0.709998), 
      AllCloseMatrix(T_W_world_pitchendlink_cal.E, TEST_PREC, TEST_PREC));
    CHECK_THAT (Vector3d(0.306143,   -0.266613,   -0.407677), 
      AllCloseVector(T_W_world_pitchendlink_cal.r, TEST_PREC, TEST_PREC));
  }
  //--------------------pitchendlink_maininsertionlink--------------------//
  {
    // Prismatic joint
    Eigen::Affine3d R_B_pitchendlink_maininsertionlink(
        Eigen::AngleAxisd(0, pitchendlink_maininsertionlinkPA));

    T_W_world_maininsertionlink_cal.E = 
      T_W_world_pitchendlink_cal.E *
      R_B_pitchendlink_maininsertionlink.rotation() *
      B_pitchendlink_maininsertionlinkST.E;

    T_W_world_maininsertionlink_cal.r = T_W_world_pitchendlink_cal.r +
      T_W_world_pitchendlink_cal.E * B_pitchendlink_maininsertionlinkST.r;

    CHECK_THAT (
      Matrix3d(
        0.499117,   -0.504929,   -0.704222,
       -0.711896,   -0.702284,  -0.0010158,
       -0.494051,     0.50184,   -0.709979), 
      AllCloseMatrix(T_W_world_maininsertionlink_cal.E, TEST_PREC, TEST_PREC));
    CHECK_THAT (Vector3d(0.334576,    -0.24871,   -0.435915), 
      AllCloseVector(T_W_world_maininsertionlink_cal.r, TEST_PREC, TEST_PREC));
  }
  //--------------------maininsertionlink-toollink--------------------//
  {
    // Had to use negative of this actual value to match with AMBF value
    Eigen::Affine3d R_B_maininsertionlink_toollink(
        Eigen::AngleAxisd(-M_PI_2, maininsertionlink_toollinkPA));

    T_W_world_toollink_cal.E = 
      T_W_world_maininsertionlink_cal.E *
      R_B_maininsertionlink_toollink.rotation() *
      B_maininsertionlink_toollinkST.E;

    T_W_world_toollink_cal.r = T_W_world_maininsertionlink_cal.r +
      T_W_world_maininsertionlink_cal.E * B_maininsertionlink_toollinkST.r;

    CHECK_THAT (
      Matrix3d(
        -0.852071,    0.159099,   -0.498661,
        -0.507187,   -0.486429,    0.711441,
        -0.129374,    0.859113,    0.495165), 
      AllCloseMatrix(T_W_world_toollink_cal.E, TEST_PREC, TEST_PREC));
    CHECK_THAT (Vector3d(0.366297,   -0.205856,   -0.467684), 
      AllCloseVector(T_W_world_toollink_cal.r, TEST_PREC, TEST_PREC));
  }  
}
*/

/*
TEST_CASE_METHOD ( ECM, __FILE__"_manual_All_Controllable_Body_Rotate_PI_4_RBDL_Reference", "") 
{
// 0,               baselink-yawlink, 0.7877469062805176
// 1,          yawlink-pitchbacklink, 0.788750946521759
// 2,  pitchbacklink-pitchbottomlink, -0.7828410863876343
// 3,   pitchbottomlink-pitchendlink, 0.7863313555717468
// 4,         yawlink-pitchfrontlink, 0.7869123220443726
// 5, pitchfrontlink-pitchbottomlink, -0.7810024619102478
// 6,    pitchfrontlink-pitchtoplink, -0.7796486020088196
// 7,      pitchtoplink-pitchendlink, 0.7849762439727783
// 8, pitchendlink-maininsertionlink, 0.10069068521261215
// 9,     maininsertionlink-toollink, 0.7648343443870544

  SpatialTransform T_W_world_baselink_cal, T_W_world_yawlink_cal, T_W_world_pitchfrontlink_cal, 
  T_W_world_pitchbacklink_cal, T_W_world_pitchbottomlink_cal, T_W_world_pitchendlink_cal, 
  T_W_world_maininsertionlink_cal, T_W_world_pitchtoplink_cal, T_W_world_toollink_cal;


  //--------------------world-base--------------------//
  {
    T_W_world_baselink_cal.E = Matrix3dIdentity;
    T_W_world_baselink_cal.r = W_world_baselinkST.r;

    // Matrix3d r_w_baselink = world_baselinkST.E;
    // Vector3d p_w_baselink = world_baselinkST.r;
    CHECK_THAT (Matrix3dIdentity, 
      AllCloseMatrix(T_W_world_baselink_cal.E, TEST_PREC, TEST_PREC));
    CHECK_THAT (Vector3d(0.5, -0.4, -0.6), 
      AllCloseVector(T_W_world_baselink_cal.r, TEST_PREC, TEST_PREC));
  }
  //--------------------baselink-yawlink--------------------//
  {
    Eigen::Affine3d R_B_baselink_yawlink(
        Eigen::AngleAxisd(0.788750946521759, baselink_yawlinkPA));

    T_W_world_yawlink_cal.E = 
      T_W_world_baselink_cal.E *
      R_B_baselink_yawlink.rotation() *
      B_baselink_yawlinkST.E;

    T_W_world_yawlink_cal.r = T_W_world_baselink_cal.r +
      T_W_world_baselink_cal.E * B_baselink_yawlinkST.r;

    // CHECK_THAT (
    //   Matrix3d(
    //     -0.705307, -0.708902,     0,
    //             0,         0,     1,
    //     -0.708902,  0.705307,     0), 
    //   AllCloseMatrix(T_W_world_yawlink_cal.E, TEST_PREC, TEST_PREC));
    
    CHECK_THAT (Vector3d(0.499988,   -0.936889,   -0.599982), 
      AllCloseVector(T_W_world_yawlink_cal.r, TEST_PREC, TEST_PREC));
  }
  //--------------------yawlink-pitchbacklink--------------------//
  {
    Eigen::Affine3d R_B_yawlink_pitchbacklink(
        Eigen::AngleAxisd(0.788750946521759, yawlink_pitchbacklinkPA));


    T_W_world_pitchbacklink_cal.E = 
      T_W_world_yawlink_cal.E *
      R_B_yawlink_pitchbacklink.rotation() *
      B_yawlink_pitchbacklinkST.E;


    T_W_world_pitchbacklink_cal.r = T_W_world_yawlink_cal.r +
      T_W_world_yawlink_cal.E * B_yawlink_pitchbacklinkST.r;

    // CHECK_THAT (
    //   Matrix3d(
    //     0.502623,    0.499695,    -0.705461,
    //     0.704721,   -0.709485, -0.000449693,
    //    -0.500739,   -0.496927,    -0.708748), 
    //   AllCloseMatrix(T_W_world_pitchbacklink_cal.E, TEST_PREC, TEST_PREC));
    
    CHECK_THAT (Vector3d(0.49979,   -0.774317,   -0.599829), 
      AllCloseVector(T_W_world_pitchbacklink_cal.r, TEST_PREC, TEST_PREC));
  }
  //--------------------pitchbacklink-pitchbottomlink--------------------//
  {
    Eigen::Affine3d R_B_pitchbacklink_pitchbottomlink(
        Eigen::AngleAxisd(-0.7828410863876343, pitchbacklink_pitchbottomlinkPA));

    T_W_world_pitchbottomlink_cal.E = 
      T_W_world_pitchbacklink_cal.E *
      R_B_pitchbacklink_pitchbottomlink.rotation() *
      B_pitchbacklink_pitchbottomlinkST.E;

    T_W_world_pitchbottomlink_cal.r = T_W_world_pitchbacklink_cal.r +
      T_W_world_pitchbacklink_cal.E * B_pitchbacklink_pitchbottomlinkST.r;

    // CHECK_THAT (
    //   Matrix3d(
    //     0.00356559,    0.709154,    -0.705044,
    //       0.999982, -0.00592559, -0.000902961,
    //    -0.00481814,   -0.705029,    -0.709163), 
      // AllCloseMatrix(T_W_world_pitchbottomlink_cal.E, TEST_PREC, TEST_PREC));
    
    CHECK_THAT (Vector3d(0.323407, -0.617888, -0.424235), 
      AllCloseVector(T_W_world_pitchbottomlink_cal.r, TEST_PREC, TEST_PREC));
  }
  //--------------------pitchbottomlink-pitchendlink--------------------//
  {
    Eigen::Affine3d R_B_pitchbottomlink_pitchendlink(
        Eigen::AngleAxisd(0.7863313555717468, pitchbottomlink_pitchendlinkPA));

    T_W_world_pitchendlink_cal.E = 
      T_W_world_pitchbottomlink_cal.E *
      R_B_pitchbottomlink_pitchendlink.rotation() *
      B_pitchbottomlink_pitchendlinkST.E;

    T_W_world_pitchendlink_cal.r = T_W_world_pitchbottomlink_cal.r +
      T_W_world_pitchbottomlink_cal.E * B_pitchbottomlink_pitchendlinkST.r;

    // CHECK_THAT (
    //   Matrix3d(
    //     0.505005,    0.499067,   -0.704203,
    //     0.702241,   -0.711939, -0.000950852,
    //    -0.501824,    -0.49404,   -0.709998), 
    //   AllCloseMatrix(T_W_world_pitchendlink_cal.E, TEST_PREC, TEST_PREC));

    CHECK_THAT (Vector3d(0.324832, -0.277794, -0.425652), 
      AllCloseVector(T_W_world_pitchendlink_cal.r, TEST_PREC, TEST_PREC));
  }
  // --------------------yawlink_pitchfrontlink--------------------//
  {
    Eigen::Affine3d R_B_yawlink_pitchfrontlink(
      Eigen::AngleAxisd(0.7869123220443726, yawlink_pitchfrontlinkPA));

    T_W_world_pitchfrontlink_cal.E = 
      T_W_world_yawlink_cal.E *
      R_B_yawlink_pitchfrontlink.rotation() *
      B_yawlink_pitchfrontlinkST.E;

    T_W_world_pitchfrontlink_cal.r = T_W_world_yawlink_cal.r +
      T_W_world_yawlink_cal.E * B_yawlink_pitchfrontlinkST.r;

    // CHECK_THAT (
    //   Matrix3d(
    //     0.501726,    0.500648,    -0.705424,
    //     0.706024,   -0.708188, -0.000457427,
    //    -0.499802,   -0.497817,    -0.708786), 
    //   AllCloseMatrix(T_W_world_pitchfrontlink_cal.E, TEST_PREC, TEST_PREC));
    CHECK_THAT (Vector3d(0.500045,   -0.736933,   -0.600062), 
      AllCloseVector(T_W_world_pitchfrontlink_cal.r, TEST_PREC, TEST_PREC));
  }
  //--------------------pitchfrontlink_pitchbottomlink--------------------//
  
  {
    Eigen::Affine3d R_B_pitchfrontlink_pitchbottomlink(
        Eigen::AngleAxisd(-0.7810024619102478, pitchfrontlink_pitchbottomlinkPA));

    T_W_world_pitchbottomlink_cal.E = 
      T_W_world_pitchfrontlink_cal.E *
      R_B_pitchfrontlink_pitchbottomlink.rotation() *
      B_pitchfrontlink_pitchbottomlinkST.E;

    T_W_world_pitchbottomlink_cal.r = T_W_world_pitchfrontlink_cal.r +
      T_W_world_pitchfrontlink_cal.E * B_pitchfrontlink_pitchbottomlinkST.r;

    // CHECK_THAT (
    //   Matrix3d(
    //     0.00356559,    0.709154,    -0.705044,
    //       0.999982, -0.00592559, -0.000902961,
    //    -0.00481814,   -0.705029,    -0.709163), 
      // AllCloseMatrix(T_W_world_pitchbottomlink_cal.E, TEST_PREC, TEST_PREC));
    // CHECK_THAT (Vector3d(0.323407, -0.617888, -0.424235), 
    //   AllCloseVector(T_W_world_pitchbottomlink_cal.r, TEST_PREC, TEST_PREC));
  }
  //--------------------pitchfrontlink-pitchtoplink--------------------//
  {
    Eigen::Affine3d R_B_pitchfrontlink_pitchtoplink(
        Eigen::AngleAxisd(-0.7796486020088196, pitchfrontlink_pitchtoplinkPA));

    T_W_world_pitchtoplink_cal.E = 
      T_W_world_pitchfrontlink_cal.E *
      R_B_pitchfrontlink_pitchtoplink.rotation() *
      B_pitchfrontlink_pitchtoplinkST.E;

    T_W_world_pitchtoplink_cal.r = T_W_world_pitchfrontlink_cal.r +
      T_W_world_pitchfrontlink_cal.E * B_pitchfrontlink_pitchtoplinkST.r;

    // CHECK_THAT (
    //   Matrix3d(
    //     0.00428893,    0.710532,   -0.703652,
    //       0.999973, -0.00727851,  -0.0012546,
    //    -0.00601297,   -0.703627,   -0.710544), 
    //   AllCloseMatrix(T_W_world_pitchtoplink_cal.E, TEST_PREC, TEST_PREC));
    CHECK_THAT (Vector3d(0.283347,    -0.58387,   -0.384493), 
      AllCloseVector(T_W_world_pitchtoplink_cal.r, TEST_PREC, TEST_PREC));
  }
  //--------------------pitchtoplink_pitchendlink--------------------//
  {
    Eigen::Affine3d R_B_pitchtoplink_pitchendlink(
        Eigen::AngleAxisd(0.7849762439727783, pitchtoplink_pitchendlinkPA));

    T_W_world_pitchendlink_cal.E = 
      T_W_world_pitchtoplink_cal.E *
      R_B_pitchtoplink_pitchendlink.rotation() *
      B_pitchtoplink_pitchendlinkST.E;

    T_W_world_pitchendlink_cal.r = T_W_world_pitchtoplink_cal.r +
      T_W_world_pitchtoplink_cal.E * B_pitchtoplink_pitchendlinkST.r;

  //   CHECK_THAT (
  //     Matrix3d(
  //       0.505005,    0.499067,    -0.704203,
  //       0.702241,   -0.711939, -0.000950852,
  //     -0.501824,     -0.49404,    -0.709998), 
  //     AllCloseMatrix(T_W_world_pitchendlink_cal.E, TEST_PREC, TEST_PREC));
    CHECK_THAT (Vector3d(0.324832, -0.277794, -0.425652), 
      AllCloseVector(T_W_world_pitchendlink_cal.r, TEST_PREC, TEST_PREC));
  }
  //--------------------pitchendlink_maininsertionlink--------------------//
  {
    // Prismatic joint
    Eigen::Affine3d R_B_pitchendlink_maininsertionlink(
        Eigen::AngleAxisd(0, pitchendlink_maininsertionlinkPA));

    T_W_world_maininsertionlink_cal.E = 
      T_W_world_pitchendlink_cal.E *
      R_B_pitchendlink_maininsertionlink.rotation() *
      B_pitchendlink_maininsertionlinkST.E;

    T_W_world_maininsertionlink_cal.r = T_W_world_pitchendlink_cal.r +
      T_W_world_pitchendlink_cal.E * B_pitchendlink_maininsertionlinkST.r;

  //   CHECK_THAT (
  //     Matrix3d(
  //       0.499117,   -0.504929,   -0.704222,
  //      -0.711896,   -0.702284,  -0.0010158,
  //      -0.494051,     0.50184,   -0.709979), 
      // AllCloseMatrix(T_W_world_maininsertionlink_cal.E, TEST_PREC, TEST_PREC));
    CHECK_THAT (Vector3d(0.353231, -0.259889, -0.453919), 
      AllCloseVector(T_W_world_maininsertionlink_cal.r, TEST_PREC, TEST_PREC));
  }
  //--------------------maininsertionlink-toollink--------------------//
  {
    // Had to use negative of this actual value to match with AMBF value
    Eigen::Affine3d R_B_maininsertionlink_toollink(
        Eigen::AngleAxisd(-M_PI_2, maininsertionlink_toollinkPA));

    T_W_world_toollink_cal.E = 
      T_W_world_maininsertionlink_cal.E *
      R_B_maininsertionlink_toollink.rotation() *
      B_maininsertionlink_toollinkST.E;

    T_W_world_toollink_cal.r = T_W_world_maininsertionlink_cal.r +
      T_W_world_maininsertionlink_cal.E * B_maininsertionlink_toollinkST.r;

    // CHECK_THAT (
    //   Matrix3d(
    //     -0.852071,    0.159099,   -0.498661,
    //     -0.507187,   -0.486429,    0.711441,
    //     -0.129374,    0.859113,    0.495165), 
    //   AllCloseMatrix(T_W_world_toollink_cal.E, TEST_PREC, TEST_PREC));
    CHECK_THAT (Vector3d(0.385013, -0.217061, -0.485552), 
      AllCloseVector(T_W_world_toollink_cal.r, TEST_PREC, TEST_PREC));
  }  
}
*/