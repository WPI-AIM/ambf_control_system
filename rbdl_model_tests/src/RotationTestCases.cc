#include "rbdl_model_tests/rbdl_tests.h"
#include <rbdl/rbdl.h>
#include <rbdl/rbdl_math.h>
#include "application/Utilities.h"

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

Matrix3d R_W_N(Matrix3d r_w_p, Matrix3d bodyRot, Matrix3d bodyRotOffset)
{
  Matrix3d r_bodyST = bodyRot.transpose() * bodyRotOffset;
  Matrix3d r_w_c = r_w_p * r_bodyST;

  return r_w_c;
}

/*
TEST_CASE(__FILE__"_ECMHCRoationCheck", "") 
{  

  Matrix3d r_w_b;
  Matrix3d r_w_y;
  Matrix3d r_w_pba;
  Matrix3d r_w_pbo;
  Matrix3d r_w_pend;
  Matrix3d r_w_m;
  Matrix3d r_w_tool;

  Matrix3d r_b_yST;
  Matrix3d r_y_pbaST;
  Matrix3d r_pba_pboST;
  Matrix3d r_pbo_pendST;
  Matrix3d r_pend_mST;
  Matrix3d r_m_toolST;

  // 1 base
  r_w_b.setIdentity();

  
  Matrix3d b_yRot(
    1, 0, 0, 
    0, 0, -1,
    0, 1, 0
    );

  Matrix3d b_yRotOffset(
    -1, 0, 0, 
    0,-1, 0,
    0, 0, 1
  );

  // r_b_yST = b_yRot.transpose() * b_yRotOffset;
  // r_w_y = r_w_b * r_b_yST;

  r_w_y = R_W_N(r_w_b, b_yRot, b_yRotOffset);
  Matrix3d r_w_y_(
    -1, 0, 0, 
    0, 0, 1,
    0, 1, 0
  );

  CHECK_THAT (r_w_y_, AllCloseMatrix(r_w_y, TEST_PREC, TEST_PREC));

  // 3 pitchback
  Matrix3d y_pbaRot(
      0, 0, -1,
      0, 1, 0,
      1, 0, 0
  );

  Matrix3d y_pbaRotOffset(
    -1, 0, 0,
    0,-1, 0,
    0, 0, 1
  );

  // r_y_pbaST = y_pbaRot.transpose() * y_pbaRotOffset;
  
  // Matrix3d r_y_pbST_(
  //   0, 0, 1,
  //   0,-1, 0,
  //   1, 0, 0
  // );
  // CHECK_THAT (r_y_pbST_, AllCloseMatrix(r_y_pbST, TEST_PREC, TEST_PREC));
  // r_w_pba = r_w_y * r_y_pbaST;
  r_w_pba = R_W_N(r_w_y, y_pbaRot, y_pbaRotOffset);
  Matrix3d r_w_pba_(
  0, 0, -1,
  1, 0,  0,
  0, -1, 0
  );

  CHECK_THAT (r_w_pba_, AllCloseMatrix(r_w_pba, TEST_PREC, TEST_PREC));

  // 4 pitchbottom
  Matrix3d pba_pboRot;
  pba_pboRot.setIdentity();

  Matrix3d pba_pboRotOffset;
  pba_pboRotOffset.setIdentity();

  // r_pba_pboST = pba_pboRot.transpose() * pba_pboRotOffset;
  // r_w_pbo = r_w_pba * r_pba_pboST;

  r_w_pbo = R_W_N(r_w_pba, pba_pboRot, pba_pboRotOffset);

  Matrix3d r_w_pbo_(
    0, 0, -1,
    1, 0, 0,
    0, -1, 0
  );

  CHECK_THAT (r_w_pbo_, AllCloseMatrix(r_w_pbo, TEST_PREC, TEST_PREC));

  // 5 pitchend
  Matrix3d pbo_pendRot;
  pbo_pendRot.setIdentity();

  Matrix3d pbo_pendRotOffset;
  pbo_pendRotOffset.setIdentity();

  // r_pbo_pendST = pbo_pendRot.transpose() * pbo_pendRotOffset;
  // r_w_pend = r_w_pbo * r_pbo_pendST;

  r_w_pend = R_W_N(r_w_pbo, pbo_pendRot, pbo_pendRotOffset);
  Matrix3d r_w_end_(
    0, 0, -1,
    1, 0, 0,
    0, -1, 0
  );

  CHECK_THAT (r_w_end_, AllCloseMatrix(r_w_pend, TEST_PREC, TEST_PREC));

  // 6 maininsertion
  Matrix3d pend_mRot(
  0, 1, 0,
  -1, 0, 0,
  -0, 0, 1
  );

  Matrix3d pend_mRotOffset;
  pend_mRotOffset.setIdentity();

  // r_pend_mST = pend_mRot.transpose() * pend_mRotOffset;
  // r_w_m = r_w_pend * r_pend_mST;

  r_w_m = R_W_N(r_w_pend, pend_mRot, pend_mRotOffset);
  Matrix3d r_w_m_(
  0, 0, -1,
  0, -1, 0,
  -1, 0, 0
  );

  CHECK_THAT (r_w_m_, AllCloseMatrix(r_w_m, TEST_PREC, TEST_PREC));

  // pitchtool
  Matrix3d m_toolRot(
    0, 0, 1,
    0, 1, 0,
    -1, 0, 0
  );

  Matrix3d m_toolRotOffset(
    0, -1, 0,
    1,  0, 0,
    0,  0, 1
  );

  // r_m_toolST = m_toolRot.transpose() * m_toolRotOffset;

  // Matrix3d r_m_toolST_(
  //   0, 0, -1,
  //   1, 0, 0,
  //   0, -1, 0
  // );
  // CHECK_THAT (r_m_toolST_, AllCloseMatrix(r_m_toolST, TEST_PREC, TEST_PREC));

  // r_w_tool = r_w_m * r_m_toolST;

  r_w_m = R_W_N(r_w_pend, pend_mRot, pend_mRotOffset);

  Matrix3d r_w_tool_(
  0, 1, 0,
  -1, 0, 0,
   0, 0, 1
  );

  CHECK_THAT (r_w_tool_, AllCloseMatrix(r_w_tool, TEST_PREC, TEST_PREC));
}
*/

/*
TEST_CASE(__FILE__"_ECMRoationCheckV1", "") 
{  

  const double ROOT_baselinkOffsetQ                  = 0.0;
  const double baselink_yawlinkOffsetQ               = -3.1414;
  const double yawlink_pitchbacklinkOffsetQ          = 3.1416;
  const double pitchbacklink_pitchbottomlinkOffsetQ  = 0.0;
  const double baselink_pitchendlinkOffsetQ          = 1.56304;
  const double pitchendlink_maininsertionlinkOffsetQ = 0.0;
  const double maininsertionlink_toollinkOffsetQ     = -1.5708;
  const double pitchbottomlink_pitchendlinkOffsetQ   = 0.0;
  const double yawlink_pitchfrontlinkOffsetQ         = 3.1416;
  const double pitchfrontlink_pitchbottomlinkOffsetQ = 0.0;
  const double pitchfrontlink_pitchtoplinkOffsetQ    = 0.0;
  const double pitchtoplink_pitchendlinkOffsetQ      = 0.0;
	const double toollink_eeOffsetQ      							 = 0.0;

	Vector3d baselink_yawlinkPA 							= { -0.0002, -1.0000, 00.0000 };
	Vector3d baselink_yawlinkCA 							= { 00.0000, 00.0000, -1.0000 };
	Vector3d baselink_yawlinkPP 							= { 00.0000, 00.0000, 00.0000 };
	Vector3d baselink_yawlinkCP 							= { 00.0001, 00.0000, 00.5369 };

	Vector3d yawlink_pitchbacklinkPA 					= { 1.0,     0.0,    0.0 };
	Vector3d yawlink_pitchbacklinkCA 					= { 0.0,     0.0,    1.0 };
	Vector3d yawlink_pitchbacklinkPP 					= { 0.0, -0.0098, 0.1624 };
	Vector3d yawlink_pitchbacklinkCP 					= { 0.0,     0.0,    0.0 };

	Vector3d pitchbacklink_pitchbottomlinkPA 	= {     0.0,     0.0,     1.0 };
	Vector3d pitchbacklink_pitchbottomlinkCA 	= {     0.0,     0.0,     1.0 };
	Vector3d pitchbacklink_pitchbottomlinkPP 	= { -0.1028, -0.2867,     0.0 };
	Vector3d pitchbacklink_pitchbottomlinkCP 	= { -0.0364,  0.0098, -0.0005 };

	Vector3d pitchbottomlink_pitchendlinkPA 	= {    0.0,     0.0,     1.0 };
	Vector3d pitchbottomlink_pitchendlinkCA 	= {    0.0,     0.0,     1.0 };
	Vector3d pitchbottomlink_pitchendlinkPP 	= { 0.3401, -0.0001, -0.0005 };
	Vector3d pitchbottomlink_pitchendlinkCP 	= {    0.0,     0.0,  0.0001 };
	
	Vector3d pitchendlink_maininsertionlinkPA = {     0.0,     1.0,    0.0 };
	Vector3d pitchendlink_maininsertionlinkCA = {     1.0,     0.0,    0.0 };
	Vector3d pitchendlink_maininsertionlinkPP = {  0.1031, -0.0961, 0.0001 };
	Vector3d pitchendlink_maininsertionlinkCP = { -0.0108,  -0.062, 0.0002 };

	Vector3d maininsertionlink_toollinkPA 	  = {     1.0,     0.0,    0.0 };
	Vector3d maininsertionlink_toollinkCA 	  = {     0.0,     0.0,   -1.0 };
	Vector3d maininsertionlink_toollinkPP 	  = { -0.0108,  -0.062,    0.0 };
	Vector3d maininsertionlink_toollinkCP 	  = { -0.0001, -0.0002, 0.0118 };
	//1--------------------------------------------------------------------//
	baselink_yawlinkPA.normalize();
	baselink_yawlinkCA.normalize();

	yawlink_pitchbacklinkPA.normalize();
	yawlink_pitchbacklinkCA.normalize();

	pitchbacklink_pitchbottomlinkPA.normalize();
	pitchbacklink_pitchbottomlinkCA.normalize();

	pitchbottomlink_pitchendlinkPA.normalize();
	pitchbottomlink_pitchendlinkCA.normalize();

	pitchendlink_maininsertionlinkPA.normalize();
	pitchendlink_maininsertionlinkCA.normalize();
	//1--------------------------------------------------------------------//
	//1--------------------------------------------------------------------//
	Matrix3d baselink_yawlinkRot = 
	Eigen::Matrix3d(Eigen::Quaterniond::FromTwoVectors(baselink_yawlinkPA, baselink_yawlinkCA));
	Eigen::Affine3d baselink_yawlinkRotOffset(Eigen::AngleAxisd(baselink_yawlinkOffsetQ, -Vector3d::UnitZ()));
		
	SpatialTransform baselink_yawlinkST;
	baselink_yawlinkST.E = baselink_yawlinkRot.transpose() * baselink_yawlinkRotOffset.rotation();
	baselink_yawlinkST.r = 
		baselink_yawlinkPP - (baselink_yawlinkRot.transpose() * baselink_yawlinkCP);
	//--------------------------------------------------------------------//
	Matrix3d yawlink_pitchbacklinkRot = 
	Eigen::Matrix3d(Eigen::Quaterniond::FromTwoVectors(yawlink_pitchbacklinkPA, yawlink_pitchbacklinkCA));
	Eigen::Affine3d yawlink_pitchbacklinkRotOffset(
		Eigen::AngleAxisd(yawlink_pitchbacklinkOffsetQ, -Vector3d::UnitZ()));
	
	SpatialTransform yawlink_pitchbacklinkST;
	yawlink_pitchbacklinkST.E = yawlink_pitchbacklinkRot.transpose() * yawlink_pitchbacklinkRotOffset.rotation();
	yawlink_pitchbacklinkST.r = 
		yawlink_pitchbacklinkPP - (yawlink_pitchbacklinkRot.transpose() * yawlink_pitchbacklinkCP);
	//1--------------------------------------------------------------------//
	Matrix3d pitchbacklink_pitchbottomlinkRot = 
	Eigen::Matrix3d(
		Eigen::Quaterniond::FromTwoVectors(pitchbacklink_pitchbottomlinkPA, pitchbacklink_pitchbottomlinkCA));
	Eigen::Affine3d pitchbacklink_pitchbottomlinkRotOffset(
		Eigen::AngleAxisd(pitchbacklink_pitchbottomlinkOffsetQ, -Vector3d::UnitZ()));
		
	SpatialTransform pitchbacklink_pitchbottomlinkST;
	pitchbacklink_pitchbottomlinkST.E = 
		pitchbacklink_pitchbottomlinkRot.transpose() * pitchbacklink_pitchbottomlinkRotOffset.rotation();
	pitchbacklink_pitchbottomlinkST.r = 
		pitchbacklink_pitchbottomlinkPP - 
		(pitchbacklink_pitchbottomlinkRot.transpose() * pitchbacklink_pitchbottomlinkCP);
	//--------------------------------------------------------------------//
	Matrix3d pitchbottomlink_pitchendlinkRot = 
	Eigen::Matrix3d(
		Eigen::Quaterniond::FromTwoVectors(pitchbottomlink_pitchendlinkPA, pitchbottomlink_pitchendlinkCA));
	Eigen::Affine3d pitchbottomlink_pitchendlinkRotOffset(
		Eigen::AngleAxisd(pitchbottomlink_pitchendlinkOffsetQ, -Vector3d::UnitX()));
		
	SpatialTransform pitchbottomlink_pitchendlinkST;
	pitchbottomlink_pitchendlinkST.E = 
		pitchbottomlink_pitchendlinkRot.transpose() * pitchbottomlink_pitchendlinkRotOffset.rotation();
	pitchbottomlink_pitchendlinkST.r = 
		pitchbottomlink_pitchendlinkPP - 
		(pitchbottomlink_pitchendlinkRot.transpose() * pitchbottomlink_pitchendlinkCP);
	//--------------------------------------------------------------------//
		Matrix3d pitchendlink_maininsertionlinkRot = 
	Eigen::Matrix3d(
		Eigen::Quaterniond::FromTwoVectors(pitchendlink_maininsertionlinkPA, pitchendlink_maininsertionlinkCA));
	Eigen::Affine3d pitchendlink_maininsertionlinkRotOffset(
		Eigen::AngleAxisd(pitchendlink_maininsertionlinkOffsetQ, -Vector3d::UnitZ()));
	SpatialTransform pitchendlink_maininsertionlinkST;
	pitchendlink_maininsertionlinkST.E =
	pitchendlink_maininsertionlinkRot.transpose() * pitchendlink_maininsertionlinkRotOffset.rotation(); 
	pitchendlink_maininsertionlinkST.r = 
		pitchendlink_maininsertionlinkPP - 
		(pitchendlink_maininsertionlinkRot.transpose() * pitchendlink_maininsertionlinkCP);
	//--------------------------------------------------------------------//
	Matrix3d maininsertionlink_toollinkRot = 
	Eigen::Matrix3d(
		Eigen::Quaterniond::FromTwoVectors(maininsertionlink_toollinkPA, maininsertionlink_toollinkCA));
	Eigen::Affine3d maininsertionlink_toollinkRotOffset(
		Eigen::AngleAxisd(maininsertionlink_toollinkOffsetQ, -Vector3d::UnitZ()));
		
	SpatialTransform maininsertionlink_toollinkST;
	maininsertionlink_toollinkST.E = maininsertionlink_toollinkRot.transpose() * maininsertionlink_toollinkRotOffset.rotation();
	maininsertionlink_toollinkST.r = 
		maininsertionlink_toollinkPP - 
		(maininsertionlink_toollinkRot.transpose() * maininsertionlink_toollinkCP);
	//--------------------------------------------------------------------//
  SpatialTransform world_baselinkST;
  world_baselinkST.E.setIdentity();
  world_baselinkST.r = Vector3d(0.4999, -0.3901, -0.599);

	SpatialTransform world_yawlinkST = world_baselinkST * baselink_yawlinkST;
	SpatialTransform world_pitchbacklinkST = world_yawlinkST * yawlink_pitchbacklinkST;
	SpatialTransform world_pitchbottomlinkST = world_pitchbacklinkST * pitchbacklink_pitchbottomlinkST;
	SpatialTransform world_pitchendlinkST = world_pitchbottomlinkST * pitchbottomlink_pitchendlinkST;
	SpatialTransform world_maininsertionlinkST = world_pitchendlinkST * pitchendlink_maininsertionlinkST;
	SpatialTransform world_toollinkST = world_maininsertionlinkST * maininsertionlink_toollinkST;
  //--------------------------------------------------------------------//
  Matrix3d r_world_yawlink_(
    -1, 0, 0, 
    0, 0, 1,
    0, 1, 0
  );
  
  Matrix3d r_world_pitchbacklink_(
  0, 0, -1,
  1, 0,  0,
  0, -1, 0
  );

  Matrix3d r_world_pitchbottomlink_(
    0, 0, -1,
    1, 0, 0,
    0, -1, 0
  );

  Matrix3d r_world_pitchendlink_(
    0, 0, -1,
    1, 0, 0,
    0, -1, 0
  );

  Matrix3d r_world_maininsertionlink_(
  0, 0, -1,
  0, -1, 0,
  -1, 0, 0
  );

  Matrix3d r_world_toollink_(
  0, 1, 0,
  -1, 0, 0,
   0, 0, 1
  );

  //--------------------------------------------------------------------//
  CHECK_THAT (r_world_yawlink_, AllCloseMatrix(world_yawlinkST.E, TEST_PREC, TEST_PREC));
  CHECK_THAT (r_world_pitchbacklink_, AllCloseMatrix(world_pitchbacklinkST.E, TEST_PREC, TEST_PREC));
  CHECK_THAT (r_world_pitchbottomlink_, AllCloseMatrix(world_pitchbottomlinkST.E, TEST_PREC, TEST_PREC));
  CHECK_THAT (r_world_pitchendlink_, AllCloseMatrix(world_pitchendlinkST.E, TEST_PREC, TEST_PREC));
  CHECK_THAT (r_world_maininsertionlink_, AllCloseMatrix(world_maininsertionlinkST.E, TEST_PREC, TEST_PREC));
  CHECK_THAT (r_world_toollink_, AllCloseMatrix(world_toollinkST.E, TEST_PREC, TEST_PREC));
}
*/

/*
TEST_CASE(__FILE__"_ECMRoationCheckV2", "") 
{  

  const double ROOT_baselinkOffsetQ                  = 0.0;
  const double baselink_yawlinkOffsetQ               = -3.1414;
  const double yawlink_pitchbacklinkOffsetQ          = 3.1416;
  const double pitchbacklink_pitchbottomlinkOffsetQ  = 0.0;
  const double pitchbottomlink_pitchendlinkOffsetQ   = 0.0;
  const double pitchendlink_maininsertionlinkOffsetQ = 0.0;
  const double maininsertionlink_toollinkOffsetQ     = -1.5708;

	Vector3d baselink_yawlinkPA 							= { -0.0002, -1.0000, 00.0000 };
	Vector3d baselink_yawlinkCA 							= { 00.0000, 00.0000, -1.0000 };
	Vector3d baselink_yawlinkPP 							= { 00.0000, 00.0000, 00.0000 };
	Vector3d baselink_yawlinkCP 							= { 00.0001, 00.0000, 00.5369 };

	Vector3d yawlink_pitchbacklinkPA 					= { 1.0,     0.0,    0.0 };
	Vector3d yawlink_pitchbacklinkCA 					= { 0.0,     0.0,    1.0 };
	Vector3d yawlink_pitchbacklinkPP 					= { 0.0, -0.0098, 0.1624 };
	Vector3d yawlink_pitchbacklinkCP 					= { 0.0,     0.0,    0.0 };

	Vector3d pitchbacklink_pitchbottomlinkPA 	= {     0.0,     0.0,     1.0 };
	Vector3d pitchbacklink_pitchbottomlinkCA 	= {     0.0,     0.0,     1.0 };
	Vector3d pitchbacklink_pitchbottomlinkPP 	= { -0.1028, -0.2867,     0.0 };
	Vector3d pitchbacklink_pitchbottomlinkCP 	= { -0.0364,  0.0098, -0.0005 };

	Vector3d pitchbottomlink_pitchendlinkPA 	= {    0.0,     0.0,     1.0 };
	Vector3d pitchbottomlink_pitchendlinkCA 	= {    0.0,     0.0,     1.0 };
	Vector3d pitchbottomlink_pitchendlinkPP 	= { 0.3401, -0.0001, -0.0005 };
	Vector3d pitchbottomlink_pitchendlinkCP 	= {    0.0,     0.0,  0.0001 };
	
	Vector3d pitchendlink_maininsertionlinkPA = {     0.0,     1.0,    0.0 };
	Vector3d pitchendlink_maininsertionlinkCA = {     1.0,     0.0,    0.0 };
	Vector3d pitchendlink_maininsertionlinkPP = {  0.1031, -0.0961, 0.0001 };
	Vector3d pitchendlink_maininsertionlinkCP = { -0.0108,  -0.062, 0.0002 };

	Vector3d maininsertionlink_toollinkPA 	  = {     1.0,     0.0,    0.0 };
	Vector3d maininsertionlink_toollinkCA 	  = {     0.0,     0.0,   -1.0 };
	Vector3d maininsertionlink_toollinkPP 	  = { -0.0108,  -0.062,    0.0 };
	Vector3d maininsertionlink_toollinkCP 	  = { -0.0001, -0.0002, 0.0118 };
	//1--------------------------------------------------------------------//
	baselink_yawlinkPA.normalize();
	baselink_yawlinkCA.normalize();

	yawlink_pitchbacklinkPA.normalize();
	yawlink_pitchbacklinkCA.normalize();

	pitchbacklink_pitchbottomlinkPA.normalize();
	pitchbacklink_pitchbottomlinkCA.normalize();

	pitchbottomlink_pitchendlinkPA.normalize();
	pitchbottomlink_pitchendlinkCA.normalize();

	pitchendlink_maininsertionlinkPA.normalize();
	pitchendlink_maininsertionlinkCA.normalize();
	//1--------------------------------------------------------------------//
	Matrix3d baselink_yawlinkRot = 
	Eigen::Matrix3d(Eigen::Quaterniond::FromTwoVectors(baselink_yawlinkPA, baselink_yawlinkCA));
	// Eigen::Affine3d baselink_yawlinkRotOffset(
    // Eigen::AngleAxisd(baselink_yawlinkOffsetQ, -Vector3d::UnitZ()));
	Eigen::Affine3d baselink_yawlinkRotOffset(
    Eigen::AngleAxisd(baselink_yawlinkOffsetQ, baselink_yawlinkCA));
	
  SpatialTransform baselink_yawlinkST;
	//baselink_yawlinkST.E = baselink_yawlinkRot.transpose() * baselink_yawlinkRotOffset.rotation();
  baselink_yawlinkST.E = 
    baselink_yawlinkRot.transpose() * baselink_yawlinkRotOffset.rotation();
	baselink_yawlinkST.r = 
		baselink_yawlinkPP - (baselink_yawlinkRot.transpose() * baselink_yawlinkCP);
	//--------------------------------------------------------------------//
	Matrix3d yawlink_pitchbacklinkRot = 
	Eigen::Matrix3d(Eigen::Quaterniond::FromTwoVectors(yawlink_pitchbacklinkPA, yawlink_pitchbacklinkCA));
	// Eigen::Affine3d yawlink_pitchbacklinkRotOffset(
	// 	Eigen::AngleAxisd(yawlink_pitchbacklinkOffsetQ, -Vector3d::UnitZ()));
  Eigen::Affine3d yawlink_pitchbacklinkRotOffset(
  Eigen::AngleAxisd(yawlink_pitchbacklinkOffsetQ, yawlink_pitchbacklinkCA));
	
	SpatialTransform yawlink_pitchbacklinkST;
	//yawlink_pitchbacklinkST.E = yawlink_pitchbacklinkRot.transpose() * yawlink_pitchbacklinkRotOffset.rotation();
  yawlink_pitchbacklinkST.E = 
    yawlink_pitchbacklinkRot.transpose() * yawlink_pitchbacklinkRotOffset.rotation();
	yawlink_pitchbacklinkST.r = 
		yawlink_pitchbacklinkPP - (yawlink_pitchbacklinkRot.transpose() * yawlink_pitchbacklinkCP);
	//1--------------------------------------------------------------------//
	Matrix3d pitchbacklink_pitchbottomlinkRot = 
	Eigen::Matrix3d(
		Eigen::Quaterniond::FromTwoVectors(pitchbacklink_pitchbottomlinkPA, pitchbacklink_pitchbottomlinkCA));
	// Eigen::Affine3d pitchbacklink_pitchbottomlinkRotOffset(
	// 	Eigen::AngleAxisd(pitchbacklink_pitchbottomlinkOffsetQ, -Vector3d::UnitZ()));
  Eigen::Affine3d pitchbacklink_pitchbottomlinkRotOffset(
      Eigen::AngleAxisd(pitchbacklink_pitchbottomlinkOffsetQ, pitchbacklink_pitchbottomlinkCA));
		
	SpatialTransform pitchbacklink_pitchbottomlinkST;
	pitchbacklink_pitchbottomlinkST.E = 
		// pitchbacklink_pitchbottomlinkRot.transpose() * pitchbacklink_pitchbottomlinkRotOffset.rotation();
    pitchbacklink_pitchbottomlinkRot.transpose() * pitchbacklink_pitchbottomlinkRotOffset.rotation();
	pitchbacklink_pitchbottomlinkST.r = 
		pitchbacklink_pitchbottomlinkPP - 
		(pitchbacklink_pitchbottomlinkRot.transpose() * pitchbacklink_pitchbottomlinkCP);
	//--------------------------------------------------------------------//
	Matrix3d pitchbottomlink_pitchendlinkRot = 
	Eigen::Matrix3d(
		Eigen::Quaterniond::FromTwoVectors(pitchbottomlink_pitchendlinkPA, pitchbottomlink_pitchendlinkCA));
	// Eigen::Affine3d pitchbottomlink_pitchendlinkRotOffset(
	// 	Eigen::AngleAxisd(pitchbottomlink_pitchendlinkOffsetQ, -Vector3d::UnitX()));
	Eigen::Affine3d pitchbottomlink_pitchendlinkRotOffset(
		Eigen::AngleAxisd(pitchbottomlink_pitchendlinkOffsetQ, pitchbottomlink_pitchendlinkCA));
		
	SpatialTransform pitchbottomlink_pitchendlinkST;
	// pitchbottomlink_pitchendlinkST.E = 
	// 	pitchbottomlink_pitchendlinkRot.transpose() * pitchbottomlink_pitchendlinkRotOffset.rotation();
  pitchbottomlink_pitchendlinkST.E = 
		pitchbottomlink_pitchendlinkRot.transpose() * pitchbottomlink_pitchendlinkRotOffset.rotation();
	pitchbottomlink_pitchendlinkST.r = 
		pitchbottomlink_pitchendlinkPP - 
		(pitchbottomlink_pitchendlinkRot.transpose() * pitchbottomlink_pitchendlinkCP);
	//--------------------------------------------------------------------//
		Matrix3d pitchendlink_maininsertionlinkRot = 
	Eigen::Matrix3d(
		Eigen::Quaterniond::FromTwoVectors(pitchendlink_maininsertionlinkPA, pitchendlink_maininsertionlinkCA));
	// Eigen::Affine3d pitchendlink_maininsertionlinkRotOffset(
	// 	Eigen::AngleAxisd(pitchendlink_maininsertionlinkOffsetQ, -Vector3d::UnitZ()));
  Eigen::Affine3d pitchendlink_maininsertionlinkRotOffset(
		Eigen::AngleAxisd(pitchendlink_maininsertionlinkOffsetQ, pitchendlink_maininsertionlinkCA));

	SpatialTransform pitchendlink_maininsertionlinkST;
	pitchendlink_maininsertionlinkST.E =
	// pitchendlink_maininsertionlinkRot.transpose() * pitchendlink_maininsertionlinkRotOffset.rotation(); 
  pitchendlink_maininsertionlinkRot.transpose() * pitchendlink_maininsertionlinkRotOffset.rotation(); 
	pitchendlink_maininsertionlinkST.r = 
		pitchendlink_maininsertionlinkPP - 
		(pitchendlink_maininsertionlinkRot.transpose() * pitchendlink_maininsertionlinkCP);
	//--------------------------------------------------------------------//
	Matrix3d maininsertionlink_toollinkRot = 
	Eigen::Matrix3d(
		Eigen::Quaterniond::FromTwoVectors(maininsertionlink_toollinkPA, maininsertionlink_toollinkCA));
	Eigen::Affine3d maininsertionlink_toollinkRotOffset(
		// Eigen::AngleAxisd(maininsertionlink_toollinkOffsetQ, -Vector3d::UnitZ()));
    Eigen::AngleAxisd(maininsertionlink_toollinkOffsetQ, maininsertionlink_toollinkCA));
		
	SpatialTransform maininsertionlink_toollinkST;
	maininsertionlink_toollinkST.E = 
  // maininsertionlink_toollinkRot.transpose() * maininsertionlink_toollinkRotOffset.rotation();
	maininsertionlink_toollinkRot.transpose() *
  maininsertionlink_toollinkRotOffset.rotation(); 
  
  maininsertionlink_toollinkST.r = 
		maininsertionlink_toollinkPP - 
		(maininsertionlink_toollinkRot.transpose() * maininsertionlink_toollinkCP);
	//--------------------------------------------------------------------//
  SpatialTransform world_baselinkST;
  world_baselinkST.E.setIdentity();
  world_baselinkST.r = Vector3d(0.4999, -0.3901, -0.599);

	const SpatialTransform world_yawlinkST = world_baselinkST * baselink_yawlinkST;
	const SpatialTransform world_pitchbacklinkST = world_yawlinkST * yawlink_pitchbacklinkST;
	const SpatialTransform world_pitchbottomlinkST = world_pitchbacklinkST * pitchbacklink_pitchbottomlinkST;
	const SpatialTransform world_pitchendlinkST = world_pitchbottomlinkST * pitchbottomlink_pitchendlinkST;
	const SpatialTransform world_maininsertionlinkST = world_pitchendlinkST * pitchendlink_maininsertionlinkST;
	const SpatialTransform world_toollinkST = world_maininsertionlinkST * maininsertionlink_toollinkST;
  //--------------------------------------------------------------------//
  const Matrix3d r_world_yawlink_(
    -1, 0, 0, 
    0, 0, 1,
    0, 1, 0
  );
  
  const Matrix3d r_world_pitchbacklink_(
  0, 0, -1,
  1, 0,  0,
  0, -1, 0
  );

  const Matrix3d r_world_pitchbottomlink_(
    0, 0, -1,
    1, 0, 0,
    0, -1, 0
  );

  const Matrix3d r_world_pitchendlink_(
    0, 0, -1,
    1, 0, 0,
    0, -1, 0
  );

  const Matrix3d r_world_maininsertionlink_(
  0, 0, -1,
  0, -1, 0,
  -1, 0, 0
  );

  const Matrix3d r_world_toollink_(
  0, 1, 0,
  -1, 0, 0,
   0, 0, 1
  );

  //--------------------------------------------------------------------//
  CHECK_THAT (r_world_yawlink_, AllCloseMatrix(world_yawlinkST.E, TEST_PREC, TEST_PREC));
  CHECK_THAT (r_world_pitchbacklink_, AllCloseMatrix(world_pitchbacklinkST.E, TEST_PREC, TEST_PREC));
  CHECK_THAT (r_world_pitchbottomlink_, AllCloseMatrix(world_pitchbottomlinkST.E, TEST_PREC, TEST_PREC));
  CHECK_THAT (r_world_pitchendlink_, AllCloseMatrix(world_pitchendlinkST.E, TEST_PREC, TEST_PREC));
  CHECK_THAT (r_world_maininsertionlink_, AllCloseMatrix(world_maininsertionlinkST.E, TEST_PREC, TEST_PREC));
  CHECK_THAT (r_world_toollink_, AllCloseMatrix(world_toollinkST.E, TEST_PREC, TEST_PREC));
}
*/

/*
TEST_CASE(__FILE__"_ECMRoationCheckV3", "") 
{  

  const double ROOT_baselinkOffsetQ                  = 0.0;
  const double baselink_yawlinkOffsetQ               = -3.1414;
  const double yawlink_pitchbacklinkOffsetQ          = 3.1416;
  const double pitchbacklink_pitchbottomlinkOffsetQ  = 0.0;
  const double pitchbottomlink_pitchendlinkOffsetQ   = 0.0;
  const double pitchendlink_maininsertionlinkOffsetQ = 0.0;
  const double maininsertionlink_toollinkOffsetQ     = -1.5708;

	Vector3d baselink_yawlinkPA 							= { -0.0002, -1.0000, 00.0000 };
	Vector3d baselink_yawlinkCA 							= { 00.0000, 00.0000, -1.0000 };
	Vector3d baselink_yawlinkPP 							= { 00.0000, 00.0000, 00.0000 };
	Vector3d baselink_yawlinkCP 							= { 00.0001, 00.0000, 00.5369 };

	Vector3d yawlink_pitchbacklinkPA 					= { 1.0,     0.0,    0.0 };
	Vector3d yawlink_pitchbacklinkCA 					= { 0.0,     0.0,    1.0 };
	Vector3d yawlink_pitchbacklinkPP 					= { 0.0, -0.0098, 0.1624 };
	Vector3d yawlink_pitchbacklinkCP 					= { 0.0,     0.0,    0.0 };

	Vector3d pitchbacklink_pitchbottomlinkPA 	= {     0.0,     0.0,     1.0 };
	Vector3d pitchbacklink_pitchbottomlinkCA 	= {     0.0,     0.0,     1.0 };
	Vector3d pitchbacklink_pitchbottomlinkPP 	= { -0.1028, -0.2867,     0.0 };
	Vector3d pitchbacklink_pitchbottomlinkCP 	= { -0.0364,  0.0098, -0.0005 };

	Vector3d pitchbottomlink_pitchendlinkPA 	= {    0.0,     0.0,     1.0 };
	Vector3d pitchbottomlink_pitchendlinkCA 	= {    0.0,     0.0,     1.0 };
	Vector3d pitchbottomlink_pitchendlinkPP 	= { 0.3401, -0.0001, -0.0005 };
	Vector3d pitchbottomlink_pitchendlinkCP 	= {    0.0,     0.0,  0.0001 };
	
	Vector3d pitchendlink_maininsertionlinkPA = {     0.0,     1.0,    0.0 };
	Vector3d pitchendlink_maininsertionlinkCA = {     1.0,     0.0,    0.0 };
	Vector3d pitchendlink_maininsertionlinkPP = {  0.1031, -0.0961, 0.0001 };
	Vector3d pitchendlink_maininsertionlinkCP = { -0.0108,  -0.062, 0.0002 };

	Vector3d maininsertionlink_toollinkPA 	  = {     1.0,     0.0,    0.0 };
	Vector3d maininsertionlink_toollinkCA 	  = {     0.0,     0.0,   -1.0 };
	Vector3d maininsertionlink_toollinkPP 	  = { -0.0108,  -0.062,    0.0 };
	Vector3d maininsertionlink_toollinkCP 	  = { -0.0001, -0.0002, 0.0118 };
	//1--------------------------------------------------------------------//
	baselink_yawlinkPA.normalize();
	baselink_yawlinkCA.normalize();

	yawlink_pitchbacklinkPA.normalize();
	yawlink_pitchbacklinkCA.normalize();

	pitchbacklink_pitchbottomlinkPA.normalize();
	pitchbacklink_pitchbottomlinkCA.normalize();

	pitchbottomlink_pitchendlinkPA.normalize();
	pitchbottomlink_pitchendlinkCA.normalize();

	pitchendlink_maininsertionlinkPA.normalize();
	pitchendlink_maininsertionlinkCA.normalize();
	//1--------------------------------------------------------------------//
  const SpatialTransform baselink_yawlinkST =
  T_Parent_ChildST(baselink_yawlinkPP, baselink_yawlinkCP,
  baselink_yawlinkPA, baselink_yawlinkCA, baselink_yawlinkOffsetQ);
	//--------------------------------------------------------------------//
  const SpatialTransform yawlink_pitchbacklinkST =
  T_Parent_ChildST(yawlink_pitchbacklinkPP, yawlink_pitchbacklinkCP,
  yawlink_pitchbacklinkPA, yawlink_pitchbacklinkCA, yawlink_pitchbacklinkOffsetQ);
	//1--------------------------------------------------------------------//
  const SpatialTransform pitchbacklink_pitchbottomlinkST =
  T_Parent_ChildST(pitchbacklink_pitchbottomlinkPP, pitchbacklink_pitchbottomlinkCP,
  pitchbacklink_pitchbottomlinkPA, pitchbacklink_pitchbottomlinkCA, pitchbacklink_pitchbottomlinkOffsetQ);
	//--------------------------------------------------------------------//
  const SpatialTransform pitchbottomlink_pitchendlinkST =
  T_Parent_ChildST(pitchbottomlink_pitchendlinkPP, pitchbottomlink_pitchendlinkCP,
  pitchbottomlink_pitchendlinkPA, pitchbottomlink_pitchendlinkCA, pitchbottomlink_pitchendlinkOffsetQ);
	//--------------------------------------------------------------------//
  const SpatialTransform pitchendlink_maininsertionlinkST =
  T_Parent_ChildST(pitchendlink_maininsertionlinkPP, pitchendlink_maininsertionlinkCP,
  pitchendlink_maininsertionlinkPA, pitchendlink_maininsertionlinkCA, pitchendlink_maininsertionlinkOffsetQ);
	//--------------------------------------------------------------------//
  const SpatialTransform maininsertionlink_toollinkST =
  T_Parent_ChildST(maininsertionlink_toollinkPP, maininsertionlink_toollinkCP,
  maininsertionlink_toollinkPA, maininsertionlink_toollinkCA, maininsertionlink_toollinkOffsetQ);
	//--------------------------------------------------------------------//
  SpatialTransform world_baselinkST;
  world_baselinkST.E.setIdentity();
  world_baselinkST.r = Vector3d(0.4999, -0.3901, -0.599);

	const SpatialTransform world_yawlinkST = world_baselinkST * baselink_yawlinkST;
	const SpatialTransform world_pitchbacklinkST = world_yawlinkST * yawlink_pitchbacklinkST;
	const SpatialTransform world_pitchbottomlinkST = world_pitchbacklinkST * pitchbacklink_pitchbottomlinkST;
	const SpatialTransform world_pitchendlinkST = world_pitchbottomlinkST * pitchbottomlink_pitchendlinkST;
	const SpatialTransform world_maininsertionlinkST = world_pitchendlinkST * pitchendlink_maininsertionlinkST;
	const SpatialTransform world_toollinkST = world_maininsertionlinkST * maininsertionlink_toollinkST;
  //--------------------------------------------------------------------//
  const Matrix3d r_world_yawlink_(
    -1, 0, 0, 
    0, 0, 1,
    0, 1, 0
  );
  
  const Matrix3d r_world_pitchbacklink_(
  0, 0, -1,
  1, 0,  0,
  0, -1, 0
  );

  const Matrix3d r_world_pitchbottomlink_(
    0, 0, -1,
    1, 0, 0,
    0, -1, 0
  );

  const Matrix3d r_world_pitchendlink_(
    0, 0, -1,
    1, 0, 0,
    0, -1, 0
  );

  const Matrix3d r_world_maininsertionlink_(
  0, 0, -1,
  0, -1, 0,
  -1, 0, 0
  );

  const Matrix3d r_world_toollink_(
  0, 1, 0,
  -1, 0, 0,
   0, 0, 1
  );

  //--------------------------------------------------------------------//
  CHECK_THAT (r_world_yawlink_, AllCloseMatrix(world_yawlinkST.E, TEST_PREC, TEST_PREC));
  CHECK_THAT (r_world_pitchbacklink_, AllCloseMatrix(world_pitchbacklinkST.E, TEST_PREC, TEST_PREC));
  CHECK_THAT (r_world_pitchbottomlink_, AllCloseMatrix(world_pitchbottomlinkST.E, TEST_PREC, TEST_PREC));
  CHECK_THAT (r_world_pitchendlink_, AllCloseMatrix(world_pitchendlinkST.E, TEST_PREC, TEST_PREC));
  CHECK_THAT (r_world_maininsertionlink_, AllCloseMatrix(world_maininsertionlinkST.E, TEST_PREC, TEST_PREC));
  CHECK_THAT (r_world_toollink_, AllCloseMatrix(world_toollinkST.E, TEST_PREC, TEST_PREC));
}
*/

const SpatialTransform T_Parent_ChildST(const Vector3d pp, const Vector3d cp,
  const Vector3d pa, const Vector3d ca, const double offsetQ)
{
  Matrix3d p_cRot = 
	Eigen::Matrix3d(Eigen::Quaterniond::FromTwoVectors(pa, ca));

	Eigen::Affine3d p_cRotOffset(
    Eigen::AngleAxisd(offsetQ, ca));
	
  SpatialTransform parent_childST;
  parent_childST.E = 
    p_cRot.transpose() * p_cRotOffset.rotation();
	parent_childST.r = 
		pp - (p_cRot.transpose() * cp);
  
  return parent_childST;
}

void Round(Vector3d& inout, double threshold)
{
  inout = (threshold < inout.array().abs()).select(inout, 0.0);
}

void RoundVector(Vector3d& inout)
{
  int rows = inout.rows();
  int cols = inout.cols();

  for(int col = 0; col < cols; col++)
  {
    for(int row = 0; row < rows; row++)
    {
      double val = inout(row, col);
      if(val < -1.0 || val > 1.0) return;

      if(val < -0.5) inout(row, col) = -1.0;
      else if(val >= -0.5 && val <  0.0) inout(row, col) = 0;
      else if(val >   0.0 && val <= 0.5) inout(row, col) = 0;
      else if(val >   0.5) inout(row, col) = 1.0;
    }    
  }
}

void RoundQ(double &q)
{
  if(q < -M_PI || q > M_PI) return;

  if(q > -M_PI && q < -M_PI_2)
    q = (q <= -3.0 * M_PI_4) ? -M_PI : -M_PI_2;
  else if(q > -M_PI_2 && q < 0)
    q = (q <= -M_PI_4) ? -M_PI_2 : 0;
  else if(q > 0 && q < M_PI_2)
    q = (q <= M_PI_4) ? 0 : M_PI_2;
  else if(q > M_PI_2 && q < M_PI)
    q = (q < 3.0 * M_PI_4) ? M_PI_2 : M_PI;
}

void Round(Matrix3d& inout, double threshold)
{
  inout = (threshold < inout.array().abs()).select(inout, 0.0f);
}

void Round(SpatialTransform& inout, double threshold)
{
  inout.E = (threshold < inout.E.array().abs()).select(inout.E, 0.0);
  inout.r = (threshold < inout.r.array().abs()).select(inout.r, 0.0);
}


TEST_CASE(__FILE__"_PSMRoationCheck", "") 
{  
// baselink->yawlink->pitchbacklink->pitchbottomlink->pitchendlink->
// maininsertionlink->toolrolllink->toolpitchlink->toolgripper1link
  double ROOT_baselinkOffsetQ                  = 0.0;
  double baselink_yawlinkOffsetQ               = 1.5726;
  double yawlink_pitchbacklinkOffsetQ          = -0.291;
  double pitchbacklink_pitchbottomlinkOffsetQ  = 1.8618;
  double pitchbottomlink_pitchendlinkOffsetQ   = 0.0;
  double pitchendlink_maininsertionlinkOffsetQ = 3.14409;
  double maininsertionlink_toolrolllinkOffsetQ = -1.5734;
  double toolrolllink_toolpitchlinkOffsetQ     = -1.5708;
  double toolpitchlink_toolgripper1linkOffsetQ = -1.58175;
  double toolpitchlink_toolgripper2linkOffsetQ = 1.58175;
  double precision = 0.01;



  RoundQ(ROOT_baselinkOffsetQ);
  RoundQ(baselink_yawlinkOffsetQ);
  RoundQ(yawlink_pitchbacklinkOffsetQ);
  RoundQ(pitchbacklink_pitchbottomlinkOffsetQ);
  RoundQ(pitchbottomlink_pitchendlinkOffsetQ);
  RoundQ(pitchendlink_maininsertionlinkOffsetQ);
  RoundQ(maininsertionlink_toolrolllinkOffsetQ);
  RoundQ(toolrolllink_toolpitchlinkOffsetQ);
  RoundQ(toolpitchlink_toolgripper1linkOffsetQ);
  RoundQ(toolpitchlink_toolgripper2linkOffsetQ);

  // std::cout << "ROOT_baselinkOffsetQ: " << ROOT_baselinkOffsetQ << std::endl;
  // std::cout << "baselink_yawlinkOffsetQ: " << baselink_yawlinkOffsetQ << std::endl;
  // std::cout << "yawlink_pitchbacklinkOffsetQ: " << yawlink_pitchbacklinkOffsetQ << std::endl;
  // std::cout << "pitchbacklink_pitchbottomlinkOffsetQ: " << pitchbacklink_pitchbottomlinkOffsetQ << std::endl;
  // std::cout << "pitchbottomlink_pitchendlinkOffsetQ: " << pitchbottomlink_pitchendlinkOffsetQ << std::endl;
  // std::cout << "pitchendlink_maininsertionlinkOffsetQ: " << pitchendlink_maininsertionlinkOffsetQ << std::endl;
  // std::cout << "maininsertionlink_toolrolllinkOffsetQ: " << maininsertionlink_toolrolllinkOffsetQ << std::endl;
  // std::cout << "toolrolllink_toolpitchlinkOffsetQ: " << toolrolllink_toolpitchlinkOffsetQ << std::endl;
  // std::cout << "toolpitchlink_toolgripper1linkOffsetQ: " << toolpitchlink_toolgripper1linkOffsetQ << std::endl;
  // std::cout << "toolpitchlink_toolgripper2linkOffsetQ: " << toolpitchlink_toolgripper2linkOffsetQ << std::endl;

	Vector3d baselink_yawlinkPA 							= { 0.00160, -1.00000, 00.0000 };
	Vector3d baselink_yawlinkCA 							= { 00.0000, -0.00279, 01.0000 };
	Vector3d baselink_yawlinkPP 							= { 00.0000, 00.00000, 00.0000 };
	Vector3d baselink_yawlinkCP 							= { 00.0000, 00.00134, -0.4859 };

	Vector3d yawlink_pitchbacklinkPA 					= { 0.0, 0.99999, 0.0033 };
	Vector3d yawlink_pitchbacklinkCA 					= { 0.0, -0.0007,    1.0 };
	Vector3d yawlink_pitchbacklinkPP 					= { 0.0, -0.0001, 00.030 };
	Vector3d yawlink_pitchbacklinkCP 					= { 0.0,     0.0,    0.0 };

	Vector3d pitchbacklink_pitchbottomlinkPA 	= {      0.0,      0.0,     1.0 };
	Vector3d pitchbacklink_pitchbottomlinkCA 	= { -0.00051, -0.00032,     1.0 };
	Vector3d pitchbacklink_pitchbottomlinkPP 	= { 00.15000, -0.00000,     0.0 };
	Vector3d pitchbacklink_pitchbottomlinkCP 	= { -0.00000,  0.00000, 0.000 };

	Vector3d pitchbottomlink_pitchendlinkPA 	= {    0.0,     0.0,   1.0 };
	Vector3d pitchbottomlink_pitchendlinkCA 	= {    0.0,     0.0,   1.0 };
	Vector3d pitchbottomlink_pitchendlinkPP 	= { 0.5160, -0.0004, 0.000 };
	Vector3d pitchbottomlink_pitchendlinkCP 	= {    0.0,     0.0, 0.000 };
	
	Vector3d pitchendlink_maininsertionlinkPA = {     0.0,     1.0,    0.0 };
	Vector3d pitchendlink_maininsertionlinkCA = {     1.0,  0.0008,    0.0 };
	Vector3d pitchendlink_maininsertionlinkPP = {  0.0430, -0.2948, 0.000 };
	Vector3d pitchendlink_maininsertionlinkCP = { -0.0000,  -000.0, 0.000 };

	Vector3d maininsertionlink_toolrolllinkPA 	  = {     1.0, -0.0008,    0.00 };
	Vector3d maininsertionlink_toolrolllinkCA 	  = {     0.0,     0.0,    1.00 };
	Vector3d maininsertionlink_toolrolllinkPP 	  = { 00.0398,  0.0000,    0.00 };
	Vector3d maininsertionlink_toolrolllinkCP 	  = { -0.0000, -0.0000, -0.1912 };

  Vector3d toolrolllink_toolpitchlinkPA 	  = {     0.0, 01.00000,    0.00 };
	Vector3d toolrolllink_toolpitchlinkCA 	  = {     0.0, -0.00019,    1.00 };
	Vector3d toolrolllink_toolpitchlinkPP 	  = { 00.0000,  0.00000,  0.1850 };
	Vector3d toolrolllink_toolpitchlinkCP 	  = { -0.0000, -0.00000, -0.0000 };

  Vector3d toolpitchlink_toolgripper1linkPA 	  = {     0.0, 00.99919,  0.04034 };
	Vector3d toolpitchlink_toolgripper1linkCA 	  = { 00.0114, 00.00000,  0.99994 };
  // Vector3d toolpitchlink_toolgripper1linkPA 	  = {     0.0, 01.00000,  0.00000 };
	// Vector3d toolpitchlink_toolgripper1linkCA 	  = { 00.0000, 00.00000,  1.00000 };
	Vector3d toolpitchlink_toolgripper1linkPP 	  = { 00.0090,  0.00000,  0.00000 };
	Vector3d toolpitchlink_toolgripper1linkCP 	  = { -0.0000, -0.00000, -0.00000 };

  Vector3d toolpitchlink_toolgripper2linkPA 	  = {     0.0, -0.99971,  0.02414 };
	Vector3d toolpitchlink_toolgripper2linkCA 	  = { 0.01177, 00.00000,  -0.99993 };
  // Vector3d toolpitchlink_toolgripper2linkPA 	  = {     0.0, -1.00000,  0.00000 };
	// Vector3d toolpitchlink_toolgripper2linkCA 	  = { 0.00000, 00.00000,  -1.0000 };
	Vector3d toolpitchlink_toolgripper2linkPP 	  = { 00.0090,  0.00000,  0.00000 };
	Vector3d toolpitchlink_toolgripper2linkCP 	  = { -0.0000, -0.00000, -0.00000 };
	//1--------------------------------------------------------------------//
  RoundVector(baselink_yawlinkPA);
  RoundVector(baselink_yawlinkCA);

  RoundVector(baselink_yawlinkPA);
  RoundVector(baselink_yawlinkCA);

  RoundVector(yawlink_pitchbacklinkPA);
  RoundVector(yawlink_pitchbacklinkCA);

  RoundVector(pitchbacklink_pitchbottomlinkPA);
  RoundVector(pitchbacklink_pitchbottomlinkCA);

  RoundVector(pitchbottomlink_pitchendlinkPA);
  RoundVector(pitchbottomlink_pitchendlinkCA);

  RoundVector(pitchendlink_maininsertionlinkPA);
  RoundVector(pitchendlink_maininsertionlinkCA);

  RoundVector(maininsertionlink_toolrolllinkPA);
  RoundVector(maininsertionlink_toolrolllinkCA);

  RoundVector(toolrolllink_toolpitchlinkPA);
  RoundVector(toolrolllink_toolpitchlinkCA);

  RoundVector(toolpitchlink_toolgripper1linkPA);
  RoundVector(toolpitchlink_toolgripper1linkCA);

  RoundVector(toolpitchlink_toolgripper2linkPA);
  RoundVector(toolpitchlink_toolgripper2linkCA);
	//1--------------------------------------------------------------------//
  const SpatialTransform baselink_yawlinkST =
  T_Parent_ChildST(baselink_yawlinkPP, baselink_yawlinkCP,
  baselink_yawlinkPA, baselink_yawlinkCA, baselink_yawlinkOffsetQ);
	//--------------------------------------------------------------------//
  const SpatialTransform yawlink_pitchbacklinkST =
  T_Parent_ChildST(yawlink_pitchbacklinkPP, yawlink_pitchbacklinkCP,
  yawlink_pitchbacklinkPA, yawlink_pitchbacklinkCA, yawlink_pitchbacklinkOffsetQ);// yawlink_pitchbacklinkOffsetQ
	//1--------------------------------------------------------------------//
  const SpatialTransform pitchbacklink_pitchbottomlinkST =
  T_Parent_ChildST(pitchbacklink_pitchbottomlinkPP, pitchbacklink_pitchbottomlinkCP,
  pitchbacklink_pitchbottomlinkPA, pitchbacklink_pitchbottomlinkCA, pitchbacklink_pitchbottomlinkOffsetQ);
  // pitchbacklink_pitchbottomlinkOffsetQ
	//--------------------------------------------------------------------//
  const SpatialTransform pitchbottomlink_pitchendlinkST =
  T_Parent_ChildST(pitchbottomlink_pitchendlinkPP, pitchbottomlink_pitchendlinkCP,
  pitchbottomlink_pitchendlinkPA, pitchbottomlink_pitchendlinkCA, pitchbottomlink_pitchendlinkOffsetQ);
	//--------------------------------------------------------------------//
  const SpatialTransform pitchendlink_maininsertionlinkST =
  T_Parent_ChildST(pitchendlink_maininsertionlinkPP, pitchendlink_maininsertionlinkCP,
  pitchendlink_maininsertionlinkPA, pitchendlink_maininsertionlinkCA, pitchendlink_maininsertionlinkOffsetQ);
	//--------------------------------------------------------------------//
  const SpatialTransform maininsertionlink_toollinkST =
  T_Parent_ChildST(maininsertionlink_toolrolllinkPP, maininsertionlink_toolrolllinkCP,
  maininsertionlink_toolrolllinkPA, maininsertionlink_toolrolllinkCA, 
  maininsertionlink_toolrolllinkOffsetQ);
  //--------------------------------------------------------------------//
  const SpatialTransform toolrolllink_toolpitchlinkST =
  T_Parent_ChildST(toolrolllink_toolpitchlinkPP, toolrolllink_toolpitchlinkCP,
  toolrolllink_toolpitchlinkPA, toolrolllink_toolpitchlinkCA, 
  toolrolllink_toolpitchlinkOffsetQ);
	//--------------------------------------------------------------------//
  // std::cout << "toolpitchlink_toolgripper1linkPA" << std::endl << toolpitchlink_toolgripper1linkPA << std::endl;
  // std::cout << "toolpitchlink_toolgripper1linkCA" << std::endl << toolpitchlink_toolgripper1linkCA << std::endl;
  const SpatialTransform toolpitchlink_toolgripper1linkST =
  T_Parent_ChildST(toolpitchlink_toolgripper1linkPP, toolpitchlink_toolgripper1linkCP,
  toolpitchlink_toolgripper1linkPA, toolpitchlink_toolgripper1linkCA, 
  toolpitchlink_toolgripper1linkOffsetQ);
  // std::cout << "toolpitchlink_toolgripper1linkST" << std::endl << toolpitchlink_toolgripper1linkST << std::endl;
  // toolpitchlink_toolgripper1linkOffsetQ
  //--------------------------------------------------------------------//
  const SpatialTransform toolpitchlink_toolgripper2linkST =
  T_Parent_ChildST(toolpitchlink_toolgripper2linkPP, toolpitchlink_toolgripper2linkCP,
  toolpitchlink_toolgripper2linkPA, toolpitchlink_toolgripper2linkCA, 
  toolpitchlink_toolgripper2linkOffsetQ);
  // toolpitchlink_toolgripper2linkOffsetQ
  //--------------------------------------------------------------------//
  SpatialTransform world_baselinkST;
  world_baselinkST.E = Matrix3d(
    -1, 0,  0, 
     0, -1, 0,
     0, 0,  1 ); 

  world_baselinkST.r = Vector3d(0.00077, -0.57191, -0.0427);
  Round(         world_baselinkST, precision);

	SpatialTransform world_yawlinkST = world_baselinkST * baselink_yawlinkST;
  Round(          world_yawlinkST, precision);

	SpatialTransform world_pitchbacklinkST = world_yawlinkST * yawlink_pitchbacklinkST;
  Round(    world_pitchbacklinkST, precision);

	SpatialTransform world_pitchbottomlinkST = world_pitchbacklinkST * pitchbacklink_pitchbottomlinkST;
  Round(  world_pitchbottomlinkST, precision);
	
  SpatialTransform world_pitchendlinkST = world_pitchbottomlinkST * pitchbottomlink_pitchendlinkST;
  Round(     world_pitchendlinkST, precision);
	
  SpatialTransform world_maininsertionlinkST = world_pitchendlinkST * pitchendlink_maininsertionlinkST;
  Round(world_maininsertionlinkST, precision);
	
  SpatialTransform world_toolrolllinkST = world_maininsertionlinkST * maininsertionlink_toollinkST;
  Round(     world_toolrolllinkST, precision);
  
  SpatialTransform world_toolpitchlinkST = world_toolrolllinkST * toolrolllink_toolpitchlinkST;
  Round    (world_toolpitchlinkST, precision);
  
  SpatialTransform world_toolgripper1linkST = world_toolpitchlinkST * toolpitchlink_toolgripper1linkST;
  Round( world_toolgripper1linkST, precision);
  
  SpatialTransform world_toolgripper2linkST = world_toolpitchlinkST * toolpitchlink_toolgripper2linkST;
  Round( world_toolgripper2linkST, precision);



  Eigen::Vector3d v;
  Eigen::Matrix3d r_world_yawlink0 = world_yawlinkST.E;
  // Round(          r_world_yawlink0, precision);
  // r_world_yawlink0.col(0) = v.array().abs();
  // r_world_yawlink0.col(1) = v.array().abs();
  // r_world_yawlink0.col(2) = v.array().abs();


  // std::cout << "r_world_yawlink0" << std::endl << r_world_yawlink0 << std::endl;
  // std::cout << "world_yawlinkST" << std::endl << world_yawlinkST << std::endl;
  // std::cout << "world_toolpitchlinkST" << std::endl << world_toolpitchlinkST << std::endl;
  // std::cout << "toolpitchlink_toolgripper1linkST" << std::endl << toolpitchlink_toolgripper1linkST << std::endl;
  // std::cout << "world_toolgripper1linkST" << std::endl << world_toolgripper1linkST << std::endl;
  //--------------------------------------------------------------------//
  const Matrix3d r_world_baselink_(
    -1, 0,  0, 
     0, -1, 0,
     0, 0,  1
  );

  const Matrix3d r_world_yawlink_(
    0, 1, 0, 
    0, 0, 1,
    1, 0, 0
  );
  
  const Matrix3d r_world_pitchbacklink_(
  0, 0, 1,
  0, -1, 0,
  1, 0, 0
  );

  const Matrix3d r_world_pitchbottomlink_(
    0, 0, 1,
    -1, 0, 0,
    0, -1, 0
  );

  const Matrix3d r_world_pitchendlink_(
    0, 0, 1,
    -1, 0, 0,
    0, -1, 0
  );

  const Matrix3d r_world_maininsertionlink_(
  0, 0, -1,
  0, -1, 0,
  -1, 0, 0
  );

  const Matrix3d r_world_toolrolllink_(
  0, 1, 0,
  1, 0, 0,
   0, 0, -1
  );

  const Matrix3d r_world_toolpitchlink_(
  0, 0, 1,
  0, 1, 0,
  -1, 0, 0
  );

  const Matrix3d r_world_toolgripper1link_(
  1, 0, 0,
  0, 0, 1,
  0, -1, 0
  );

  const Matrix3d r_world_toolgripper2link_(
  1, 0, 0,
  0, 0, 1,
  0, -1, 0
  );
  //--------------------------------------------------------------------//
  CHECK_THAT (r_world_baselink_, AllCloseMatrix(world_baselinkST.E, TEST_PREC, TEST_PREC));

  // Matrix3d r_world_yawlink = world_yawlinkST.E;
  // Round(r_world_yawlink, 0.01);
  CHECK_THAT (r_world_yawlink_, AllCloseMatrix(world_yawlinkST.E, TEST_PREC, TEST_PREC));

  // Matrix3d r_world_pitchbacklink = world_pitchbacklinkST.E;
  // // Round(r_world_pitchbacklink, 0.01);
  CHECK_THAT (r_world_pitchbacklink_, AllCloseMatrix(world_pitchbacklinkST.E, TEST_PREC, TEST_PREC));
  
  // Matrix3d r_world_pitchbottomlink = world_pitchbottomlinkST.E;
  // // RoundM(r_world_pitchbottomlink, 0.01);
  CHECK_THAT (r_world_pitchbottomlink_, AllCloseMatrix(world_pitchbottomlinkST.E, TEST_PREC, TEST_PREC));

  // Matrix3d r_world_pitchendlink = world_pitchendlinkST.E;
  // // RoundM(r_world_pitchendlink, 0.01);
  CHECK_THAT (r_world_pitchendlink_, AllCloseMatrix(world_pitchendlinkST.E, TEST_PREC, TEST_PREC));

  Matrix3d r_world_maininsertionlink = world_maininsertionlinkST.E;
  // // RoundM(r_world_maininsertionlink, 0.01);
  CHECK_THAT (r_world_maininsertionlink_, AllCloseMatrix(world_maininsertionlinkST.E, TEST_PREC, TEST_PREC));
  
  // Matrix3d r_world_toolrolllink = world_toolrolllinkST.E;
  // // RoundM(r_world_toolrolllink, 0.01);
  CHECK_THAT (r_world_toolrolllink_, AllCloseMatrix(world_toolrolllinkST.E, 
  TEST_PREC, TEST_PREC));

  // Matrix3d r_world_toolpitchlink = world_toolpitchlinkST.E;
  // // RoundM(r_world_toolpitchlink, 0.01);
  CHECK_THAT (r_world_toolpitchlink_, AllCloseMatrix(world_toolpitchlinkST.E, 
  TEST_PREC, TEST_PREC));

  // Matrix3d r_world_toolgripper1link = world_toolgripper1linkST.E;
  // // RoundM(r_world_toolgripper1link, 0.0000000000001);
  CHECK_THAT (r_world_toolgripper1link_, AllCloseMatrix(world_toolgripper1linkST.E, 
  TEST_PREC, TEST_PREC));

  // Matrix3d r_world_toolgripper2link = world_toolgripper2linkST.E;
  // RoundM(r_world_toolgripper2link, 0.0000000000001);
  CHECK_THAT (r_world_toolgripper2link_, AllCloseMatrix(world_toolgripper2linkST.E, 
  TEST_PREC, TEST_PREC));
}
