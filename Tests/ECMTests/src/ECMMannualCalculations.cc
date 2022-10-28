#include "ECMTests/ECMMannualCalculations.h"

TEST_CASE_METHOD ( ECM, __FILE__"_HomePose", "") 
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
  Vector3d p_w_baselink = world_baselinkST.r;
  CHECK_THAT (Matrix3dIdentity, 
    AllCloseMatrix(world_baselinkST.E, TEST_PREC, TEST_PREC));
  CHECK_THAT (Vector3d(0.5, -0.4, -0.6), 
    AllCloseVector(p_w_baselink, TEST_PREC, TEST_PREC));
  
  CHECK_THAT (Matrix3d(-1, 0, 0, 0, 0, 1, 0, 1, 0), 
    AllCloseMatrix(world_yawlinkST.E, TEST_PREC, TEST_PREC));
  Vector3d p_w_yawlink = p_w_baselink + world_baselinkST.E * baselink_yawlinkST.r;
  CHECK_THAT (Vector3d(0.5,   -0.936899,   -0.600006), 
    AllCloseVector(p_w_yawlink, TEST_PREC, TEST_PREC));

  CHECK_THAT (Matrix3d(0, 0, -1, 1, 0, 0, 0, -1, 0), 
    AllCloseMatrix(world_pitchbacklinkST.E, TEST_PREC, TEST_PREC));
  Vector3d p_w_pitchbacklink = p_w_yawlink + world_yawlinkST.E * yawlink_pitchbacklinkST.r;
	CHECK_THAT (Vector3d(0.500009,   -0.774492,   -0.600079), 
    AllCloseVector(p_w_pitchbacklink, TEST_PREC, TEST_PREC));

  CHECK_THAT (Matrix3d(0, 0, -1, 1, 0, 0, 0, -1, 0), 
    AllCloseMatrix(world_pitchbottomlinkST.E, TEST_PREC, TEST_PREC));
  Vector3d p_w_pitchbottomlink = p_w_pitchbacklink + 
    world_pitchbacklinkST.E * pitchbacklink_pitchbottomlinkST.r;
  CHECK_THAT (Vector3d(0.499951,   -0.841289,   -0.313501), 
    AllCloseVector(p_w_pitchbottomlink, TEST_PREC, TEST_PREC));

  CHECK_THAT (Matrix3d(0, 0, -1, 1, 0, 0, 0, -1, 0), 
    AllCloseMatrix(world_pitchendlinkST.E, TEST_PREC, TEST_PREC));
  Vector3d p_w_pitchendlink = p_w_pitchbottomlink + 
    world_pitchbottomlinkST.E * pitchbottomlink_pitchendlinkST.r;
  CHECK_THAT (Vector3d(0.499978,   -0.501171,   -0.313362), 
    AllCloseVector(p_w_pitchendlink, TEST_PREC, TEST_PREC));

  CHECK_THAT (Matrix3d(0, 0, -1, 1, 0, 0, 0, -1, 0), 
    AllCloseMatrix(world_pitchfrontlinkST.E, TEST_PREC, TEST_PREC));
  Vector3d p_w_pitchfrontlink = p_w_yawlink +
    world_yawlinkST.E * yawlink_pitchfrontlinkST.r;
  CHECK_THAT (Vector3d(0.5,   -0.736909,    -0.59998), 
    AllCloseVector(p_w_pitchfrontlink, TEST_PREC, TEST_PREC));
  
  CHECK_THAT (Matrix3d(0, 0, -1, 1, 0, 0, 0, -1, 0), 
    AllCloseMatrix(world_pitchtoplinkST.E, TEST_PREC, TEST_PREC));
  Vector3d p_w_pitchtoplink = p_w_pitchfrontlink +
    world_pitchfrontlinkST.E * pitchfrontlink_pitchtoplinkST.r; 
  CHECK_THAT (Vector3d(0.499951, -0.846723, -0.276256), 
    AllCloseVector(p_w_pitchtoplink, TEST_PREC, TEST_PREC));

  p_w_pitchendlink = p_w_pitchtoplink + 
    world_pitchtoplinkST.E * pitchtoplink_pitchendlinkST.r;
  CHECK_THAT (Vector3d(0.499978,   -0.501171,   -0.313362), 
    AllCloseVector(p_w_pitchendlink, TEST_PREC, TEST_PREC));

  CHECK_THAT (Matrix3d(0, 0, -1, 0, -1, 0, -1, 0, 0), 
    AllCloseMatrix(world_maininsertionlinkST.E, TEST_PREC, TEST_PREC));
  Vector3d p_w_maininsertionlink = p_w_pitchendlink +
    world_pitchendlinkST.E * pitchendlink_maininsertionlinkST.r;
  CHECK_THAT (Vector3d(0.499918,   -0.460183,    -0.22929), 
    AllCloseVector(p_w_maininsertionlink, TEST_PREC, TEST_PREC));

  CHECK_THAT (Matrix3d(0, 1, 0, -1, 0, 0, 0, 0, 1), 
    AllCloseMatrix(world_toollinkST.E, TEST_PREC, TEST_PREC));
  Vector3d p_w_toollink = p_w_maininsertionlink +
    world_maininsertionlinkST.E * maininsertionlink_toollinkST.r;
  CHECK_THAT (Vector3d(0.499922,   -0.398184,   -0.230241), 
    AllCloseVector(p_w_toollink, TEST_PREC, TEST_PREC));
}