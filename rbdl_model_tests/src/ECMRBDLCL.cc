#include <iostream>

#include "rbdl/rbdl_mathutils.h"
#include "rbdl/Logging.h"

#include "rbdl/Model.h"
#include "rbdl/Kinematics.h"
#include "rbdl/Dynamics.h"
#include "rbdl_model_tests/rbdl_tests.h"

using namespace std;
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

// const double TEST_PREC = 1.0e-2;
const double TEST_LAX = 1.0e-7;
typedef Model* RBDLModelPtr;

struct ECM {
  ECM () {
	// mass, com - inertia offset, inertia
	baselinkBody_          = Body(00.001, Vector3d(-0.00010, -0.61452, -0.02088), 
																				Vector3d(00.00000, 00.00000, 00.00000));
	yawlinkBody_           = Body(06.417, Vector3d(00.00000, -0.01614, 00.13447), 
																				Vector3d(00.29778, 00.31243, 00.04495));
	pitchbacklinkBody_     = Body(00.421, Vector3d(-0.05150, -0.14343, -0.00900), 
																				Vector3d(00.02356, 00.00278, 00.02612));
	pitchbottomlinkBody_   = Body(00.359, Vector3d(00.14913, -0.01816, 00.00000), 
																				Vector3d(00.00065, 00.01897, 00.01923));
	pitchendlinkBody_      = Body(02.032, Vector3d(00.05135, 00.00482, 00.00079), 
																				Vector3d(00.06359, 00.00994, 00.07258));
	maininsertionlinkBody_ = Body(00.231, Vector3d(-0.05900, -0.01650, 00.00079), 
																				Vector3d(00.00029, 00.00147, 00.00159));
	toollinkBody_          = Body(01.907, Vector3d(00.00000, -0.00081, 00.07232), 
																				Vector3d(00.04569, 00.04553, 00.00169));
	pitchfrontlinkBody_    = Body(01.607, Vector3d(-0.03649, -0.15261, 00.00000), 
																				Vector3d(00.09829, 00.01747, 00.10993));
	pitchtoplinkBody_      = Body(00.439, Vector3d(00.17020, -0.00070, 00.00079), 
																				Vector3d(00.00030, 00.03813, 00.03812));
	
	Vector3d vector3d_zero = Vector3d::Zero();
	virtualBody_ = Body(0., vector3d_zero, vector3d_zero);



  unsigned int world_baselinkId, baselink_yawlinkId, 
	yawlink_pitchbacklinkId, pitchbacklink_pitchbottomlinkId, pitchbottomlink_pitchendlinkId, 
	yawlink_pitchfrontlinkId, pitchfrontlink_pitchtoplinkId, pitchfrontlink_pitchbottomlinkId,
	pitchtoplink_pitchendlinkId, 
	pitchendlink_maininsertionlinkId,	maininsertionlink_toollinkId;

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

	Vector3d baselink_yawlinkPA 							= { -0.0000, -1.0000, 00.0000 };
	Vector3d baselink_yawlinkCA 							= { 00.0000, 00.0000, -1.0000 };
	Vector3d baselink_yawlinkPP 							= { 00.0000, 00.0000, 00.0000 };
	Vector3d baselink_yawlinkCP 							= { 00.0001, 00.0000, 00.5369 };

	//-----------------------------------------------------//
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
	//-----------------------------------------------------//
	Vector3d yawlink_pitchfrontlinkPA 					= { 1.0,     0.0,    0.0 };
	Vector3d yawlink_pitchfrontlinkCA 					= { 0.0,     0.0,    1.0 };
	Vector3d yawlink_pitchfrontlinkPP 					= { 0.0, 		 0.0,  0.199 };
	Vector3d yawlink_pitchfrontlinkCP 					= { 0.0,     0.0,    0.0 };

	Vector3d pitchfrontlink_pitchbottomlinkPA 	= { 	 	1.0,     0.0,     0.0 };
	Vector3d pitchfrontlink_pitchbottomlinkCA 	= { 		0.0,     0.0,     1.0 };
	Vector3d pitchfrontlink_pitchbottomlinkPP 	= { -0.1031, -0.2868,  	  0.0 };
	Vector3d pitchfrontlink_pitchbottomlinkCP 	= { -0.0001, -0.0001, -0.0005 };

	Vector3d pitchfrontlink_pitchtoplinkPA 	= { 	 	0.0,     0.0,     1.0 };
	Vector3d pitchfrontlink_pitchtoplinkCA 	= { 		0.0,     0.0,     1.0 };
	Vector3d pitchfrontlink_pitchtoplinkPP 	= { -0.1084, -0.3242,  	  0.0 };
	Vector3d pitchfrontlink_pitchtoplinkCP 	= { -0.0000, -0.0000, -0.0006 };

	Vector3d pitchtoplink_pitchendlinkPA 	= {     0.0,     0.0,     1.0 };
	Vector3d pitchtoplink_pitchendlinkCA 	= {     0.0,     0.0,     1.0 };
	Vector3d pitchtoplink_pitchendlinkPP 	= {  0.3404, -0.0002, -0.0006 };
	Vector3d pitchtoplink_pitchendlinkCP 	= { -0.0051, -0.0376,  0.0001 };
	//-----------------------------------------------------//
	Vector3d pitchendlink_maininsertionlinkPA = {     0.0,     1.0,    0.0 };
	Vector3d pitchendlink_maininsertionlinkCA = {     1.0,     0.0,    0.0 };
	Vector3d pitchendlink_maininsertionlinkPP = {  0.1031, -0.0961, 0.0001 };
	Vector3d pitchendlink_maininsertionlinkCP = { -0.0108,  -0.062, 0.00002 };

	Vector3d maininsertionlink_toollinkPA 	  = {     1.0,     0.0,    0.0 };
	Vector3d maininsertionlink_toollinkCA 	  = {     0.0,     0.0,   -1.0 };
	Vector3d maininsertionlink_toollinkPP 	  = { -0.0108,  -0.062,    0.0 };
	Vector3d maininsertionlink_toollinkCP 	  = { -0.0001, -0.0002, 0.0118 };

	rbdlModelPtr_ = new Model();
	rbdlModelPtr_->gravity = Vector3d(0., 0., -9.81);

	
	//0---------------------------------------------------------------------//
	world_baselinkST.E.setIdentity();
	world_baselinkST.r = Vector3d(0.4999, -0.3901, -0.599);

	// This is to handle initial World to body rotation.
	//1--------------------------------------------------------------------//
	// baselink_yawlinkPA.normalize();
	// baselink_yawlinkCA.normalize();

	// yawlink_pitchbacklinkPA.normalize();
	// yawlink_pitchbacklinkCA.normalize();

	// pitchbacklink_pitchbottomlinkPA.normalize();
	// pitchbacklink_pitchbottomlinkCA.normalize();

	// pitchbottomlink_pitchendlinkPA.normalize();
	// pitchbottomlink_pitchendlinkCA.normalize();

	// pitchendlink_maininsertionlinkPA.normalize();
	// pitchendlink_maininsertionlinkCA.normalize();

	// yawlink_pitchfrontlinkPA.normalize();
	// yawlink_pitchfrontlinkCA.normalize();

	// pitchfrontlink_pitchbottomlinkPA.normalize();
	// pitchfrontlink_pitchbottomlinkCA.normalize();

	// pitchfrontlink_pitchtoplinkPA.normalize();
	// pitchfrontlink_pitchtoplinkCA.normalize();

	// pitchtoplink_pitchendlinkPA.normalize();
	// pitchtoplink_pitchendlinkCA.normalize();
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
	Matrix3d yawlink_pitchfrontlinkRot = 
	Eigen::Matrix3d(Eigen::Quaterniond::FromTwoVectors(yawlink_pitchfrontlinkPA, yawlink_pitchfrontlinkCA));
	Eigen::Affine3d yawlink_pitchfrontlinkRotOffset(
		Eigen::AngleAxisd(yawlink_pitchfrontlinkOffsetQ, -Vector3d::UnitZ()));
	
	SpatialTransform yawlink_pitchfrontlinkST;
	yawlink_pitchfrontlinkST.E = yawlink_pitchfrontlinkRot.transpose() * yawlink_pitchfrontlinkRotOffset.rotation();
	yawlink_pitchfrontlinkST.r = 
		yawlink_pitchfrontlinkPP - (yawlink_pitchfrontlinkRot.transpose() * yawlink_pitchfrontlinkCP);
	//1--------------------------------------------------------------------//
	Matrix3d pitchfrontlink_pitchbottomlinkRot = 
	Eigen::Matrix3d(Eigen::Quaterniond::FromTwoVectors(pitchfrontlink_pitchbottomlinkPA, pitchfrontlink_pitchbottomlinkCA));
	Eigen::Affine3d pitchfrontlink_pitchbottomlinkRotOffset(
		Eigen::AngleAxisd(pitchfrontlink_pitchbottomlinkOffsetQ, -Vector3d::UnitZ()));
	
	SpatialTransform pitchfrontlink_pitchbottomlinkST;
	pitchfrontlink_pitchbottomlinkST.E = pitchfrontlink_pitchbottomlinkRot.transpose() * pitchfrontlink_pitchbottomlinkRotOffset.rotation();
	pitchfrontlink_pitchbottomlinkST.r = 
		pitchfrontlink_pitchbottomlinkPP - (pitchfrontlink_pitchbottomlinkRot.transpose() * pitchfrontlink_pitchbottomlinkCP);
	//1--------------------------------------------------------------------//
	Matrix3d pitchfrontlink_pitchtoplinkRot = 
	Eigen::Matrix3d(Eigen::Quaterniond::FromTwoVectors(pitchfrontlink_pitchtoplinkPA, pitchfrontlink_pitchtoplinkCA));
	Eigen::Affine3d pitchfrontlink_pitchtoplinkRotOffset(
		Eigen::AngleAxisd(pitchfrontlink_pitchtoplinkOffsetQ, -Vector3d::UnitZ()));
	
	SpatialTransform pitchfrontlink_pitchtoplinkST;
	pitchfrontlink_pitchtoplinkST.E = pitchfrontlink_pitchtoplinkRot.transpose() * pitchfrontlink_pitchtoplinkRotOffset.rotation();
	pitchfrontlink_pitchtoplinkST.r = 
		pitchfrontlink_pitchtoplinkPP - (pitchfrontlink_pitchtoplinkRot.transpose() * pitchfrontlink_pitchtoplinkCP);
	//1--------------------------------------------------------------------//
	Matrix3d pitchtoplink_pitchendlinkRot = 
	Eigen::Matrix3d(Eigen::Quaterniond::FromTwoVectors(pitchtoplink_pitchendlinkPA, pitchtoplink_pitchendlinkCA));
	Eigen::Affine3d pitchtoplink_pitchendlinkRotOffset(
		Eigen::AngleAxisd(pitchtoplink_pitchendlinkOffsetQ, -Vector3d::UnitZ()));
	
	SpatialTransform pitchtoplink_pitchendlinkST;
	pitchtoplink_pitchendlinkST.E = pitchtoplink_pitchendlinkRot.transpose() * pitchtoplink_pitchendlinkRotOffset.rotation();
	pitchtoplink_pitchendlinkST.r = 
		pitchtoplink_pitchendlinkPP - (pitchtoplink_pitchendlinkRot.transpose() * pitchtoplink_pitchendlinkCP);
	//1--------------------------------------------------------------------//
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
	// std::cout << "baselink_yawlinkRot" 			 					<< std::endl << baselink_yawlinkRot 							<< std::endl;
	// std::cout << "yawlink_pitchbacklinkRot" 		 			<< std::endl << yawlink_pitchbacklinkRot 					<< std::endl;
	// std::cout << "pitchbacklink_pitchbottomlinkRot" 	<< std::endl << pitchbacklink_pitchbottomlinkRot 	<< std::endl;
	// std::cout << "pitchbottomlink_pitchendlinkRot" 		<< std::endl << pitchbottomlink_pitchendlinkRot 	<< std::endl;
	// std::cout << "pitchendlink_maininsertionlinkRot"	<< std::endl << pitchendlink_maininsertionlinkRot << std::endl;
	// std::cout << "maininsertionlink_toollinkRot" 			<< std::endl << maininsertionlink_toollinkRot 		<< std::endl;

	// std::cout << "baselink_yawlinkRotOffset.rotation()" << std::endl << baselink_yawlinkRotOffset.rotation() 	<< std::endl;
	// std::cout << "yawlink_pitchbacklinkRotOffset.rotation()" 		 			<< std::endl << yawlink_pitchbacklinkRotOffset.rotation() 					<< std::endl;
	// std::cout << "pitchbacklink_pitchbottomlinkRotOffset.rotation()" 	<< std::endl << pitchbacklink_pitchbottomlinkRotOffset.rotation() 	<< std::endl;
	// std::cout << "pitchbottomlink_pitchendlinkRotOffset.rotation()" 		<< std::endl << pitchbottomlink_pitchendlinkRotOffset.rotation() 	<< std::endl;
	// std::cout << "pitchendlink_maininsertionlinkRotOffset.rotation()"	<< std::endl << pitchendlink_maininsertionlinkRotOffset.rotation() << std::endl;
	// std::cout << "maininsertionlink_toollinkRotOffset.rotation()" 			<< std::endl << maininsertionlink_toollinkRotOffset.rotation() 		<< std::endl;
	
  baselink_yawlinkST.E               = Matrix3d(-1,  0,  0, 0,  0, 1, 0,  1, 0);
  yawlink_pitchbacklinkST.E          = Matrix3d( 0,  0,  1, 0, -1, 0, 1,  0, 0);
  pitchendlink_maininsertionlinkST.E = Matrix3d( 0, -1,  0, 1,  0, 0, 0,  0, 1);
  maininsertionlink_toollinkST.E     = Matrix3d( 0,  0, -1, 1,  0, 0, 0, -1, 0);

	std::cout << "world_baselinkST" 			 					<< std::endl << world_baselinkST 								<< std::endl;
	std::cout << "baselink_yawlinkST" 		 					<< std::endl << baselink_yawlinkST 							<< std::endl;
	std::cout << "yawlink_pitchbacklinkST" 					<< std::endl << yawlink_pitchbacklinkST 				<< std::endl;
	std::cout << "pitchbacklink_pitchbottomlinkST" 	<< std::endl << pitchbacklink_pitchbottomlinkST << std::endl;
	std::cout << "pitchbottomlink_pitchendlinkST" 	<< std::endl << pitchbottomlink_pitchendlinkST 	<< std::endl;
	std::cout << "pitchendlink_maininsertionlinkST" << std::endl << pitchendlink_maininsertionlinkST << std::endl;
	std::cout << "maininsertionlink_toollinkST" 		<< std::endl << maininsertionlink_toollinkST 		<< std::endl;

	world_yawlinkST = world_baselinkST * baselink_yawlinkST;
	world_pitchbacklinkST = world_yawlinkST * yawlink_pitchbacklinkST;
	world_pitchbottomlinkST = world_pitchbacklinkST * pitchbacklink_pitchbottomlinkST;
	world_pitchendlinkST = world_pitchbottomlinkST * pitchbottomlink_pitchendlinkST;

	world_pitchfrontlinkST = world_yawlinkST * yawlink_pitchfrontlinkST;
	world_pitchbottomlink2ST = world_pitchfrontlinkST * pitchfrontlink_pitchbottomlinkST;
	world_pitchtoplinkST = world_pitchfrontlinkST * pitchfrontlink_pitchtoplinkST;
	world_pitchendlinkST = world_pitchtoplinkST * pitchtoplink_pitchendlinkST;

	world_maininsertionlinkST = world_pitchendlinkST * pitchendlink_maininsertionlinkST;
	world_toollinkST = world_maininsertionlinkST * maininsertionlink_toollinkST;

  world_pitchendlinkST.E      = Matrix3d( 0,  0, -1,  1,  0, 0,  0, -1, 0);
  world_pitchfrontlinkST.E    = Matrix3d( 0,  0, -1,  1,  0, 0,  0, -1, 0);
  world_pitchbottomlink2ST.E  = Matrix3d( 1,  0,  0,  0,  0, 1,  0, -1, 0);
  world_pitchtoplinkST.E      = Matrix3d( 0,  0, -1,  1,  0, 0,  0, -1, 0);
  world_maininsertionlinkST.E = Matrix3d( 0,  0, -1,  0, -1, 0, -1,  0, 0);
  world_toollinkST.E          = Matrix3d( 0,  1,  0, -1,  0, 0,  0,  0, 1);


  std::cout << "world_yawlinkST" << std::endl << world_yawlinkST << std::endl;
  std::cout << "world_pitchbacklinkST" << std::endl << world_pitchbacklinkST << std::endl;
  std::cout << "world_pitchbottomlinkST" << std::endl << world_pitchbottomlinkST << std::endl;
  std::cout << "world_pitchendlinkST" << std::endl << world_pitchendlinkST << std::endl;

  std::cout << "world_pitchfrontlinkST" << std::endl << world_pitchfrontlinkST << std::endl;
  std::cout << "world_pitchbottomlink2ST" << std::endl << world_pitchbottomlink2ST << std::endl;
  std::cout << "world_pitchtoplinkST" << std::endl << world_pitchtoplinkST << std::endl;
  std::cout << "world_pitchendlinkST" << std::endl << world_pitchendlinkST << std::endl;
  
  std::cout << "world_maininsertionlinkST" << std::endl << world_maininsertionlinkST << std::endl;
  std::cout << "world_toollinkST"          << std::endl << world_toollinkST          << std::endl;
  ////////////////
	Vector3d baselink_yawlinkJAxis = world_baselinkST.E * baselink_yawlinkPA;
  
	Vector3d yawlink_pitchbacklinkJAxis = world_yawlinkST.E * yawlink_pitchbacklinkPA;
	Vector3d pitchbacklink_pitchbottomlinkJAxis = world_pitchbacklinkST.E * pitchbacklink_pitchbottomlinkPA;
	Vector3d pitchbottomlink_pitchendlinkJAxis = world_pitchbottomlinkST.E * pitchbottomlink_pitchendlinkPA;
	
	Vector3d yawlink_pitchfrontlinkJAxis = world_yawlinkST.E * yawlink_pitchfrontlinkPA;
	Vector3d pitchfrontlink_pitchtoplinkJAxis = world_pitchfrontlinkST.E * pitchfrontlink_pitchtoplinkPA;
	Vector3d pitchtoplink_pitchendlinkJAxis = world_pitchtoplinkST.E * pitchtoplink_pitchendlinkPA;
	
	Vector3d pitchfrontlink_pitchbottomlinkJAxis = world_pitchfrontlinkST.E * pitchfrontlink_pitchbottomlinkPA;
  Vector3d pitchendlink_maininsertionlinkJAxis = world_pitchendlinkST.E * pitchendlink_maininsertionlinkPA;
  Vector3d maininsertionlink_toollinkJAxis = world_maininsertionlinkST.E * maininsertionlink_toollinkPA;
  
  std::cout << "baselink_yawlinkJAxis: "               << baselink_yawlinkJAxis.transpose()	<< std::endl;
  std::cout << "yawlink_pitchbacklinkJAxis: "          << yawlink_pitchbacklinkJAxis.transpose()	<< std::endl;
  std::cout << "pitchbacklink_pitchbottomlinkJAxis: "  << pitchbacklink_pitchbottomlinkJAxis.transpose()	<< std::endl;
  std::cout << "pitchbottomlink_pitchendlinkJAxis: "   << pitchbottomlink_pitchendlinkJAxis.transpose()	<< std::endl;

  std::cout << "yawlink_pitchfrontlinkJAxis: "         << yawlink_pitchfrontlinkJAxis.transpose()	<< std::endl;
  std::cout << "pitchfrontlink_pitchtoplinkJAxis: "    << pitchfrontlink_pitchtoplinkJAxis.transpose()	<< std::endl;
  std::cout << "pitchtoplink_pitchendlinkJAxis: "      << pitchtoplink_pitchendlinkJAxis.transpose() << std::endl;
  
  std::cout << "pitchfrontlink_pitchbottomlinkJAxis: " << pitchfrontlink_pitchbottomlinkJAxis.transpose()	<< std::endl;
  std::cout << "pitchendlink_maininsertionlinkJAxis: " << pitchendlink_maininsertionlinkJAxis.transpose()	<< std::endl;
  std::cout << "maininsertionlink_toollinkJAxis: "     << maininsertionlink_toollinkJAxis.transpose()	<< std::endl;

  ///////////////////
	// CHECK_THAT (yawlink_pitchbacklinkJ, 
	// 	AllCloseVector(yawlink_pitchfrontlinkJ, TEST_PREC, TEST_PREC));
	// CHECK_THAT (pitchbacklink_pitchbottomlinkJ, 
	// 	AllCloseVector(pitchfrontlink_pitchtoplinkJ, TEST_PREC, TEST_PREC));
	// CHECK_THAT (pitchbottomlink_pitchendlinkJ, 
	// 	AllCloseVector(pitchtoplink_pitchendlinkJ, TEST_PREC, TEST_PREC));

	// std::cout << "pitchfrontlink_pitchbottomlinkJ" << std::endl << pitchfrontlink_pitchbottomlinkJ << std::endl;

	// std::cout << "world_baselinkST" 				 << std::endl << world_baselinkST 				 << std::endl;
	// std::cout << "world_yawlinkST" 		 	 		 << std::endl << world_yawlinkST 					 << std::endl;
	// std::cout << "world_pitchbacklinkST" 	 	 << std::endl << world_pitchbacklinkST 		 << std::endl;
	// std::cout << "world_pitchbottomlinkST" 	 << std::endl << world_pitchbottomlinkST 	 << std::endl;
	// std::cout << "world_pitchendlinkST" 		 << std::endl << world_pitchendlinkST 		 << std::endl;
	// std::cout << "world_maininsertionlinkST" << std::endl << world_maininsertionlinkST << std::endl;
	// std::cout << "world_toollinkST" 				 << std::endl << world_toollinkST 				 << std::endl;
	//1--------------------------------------------------------------------//
	Vector3d p_baselink_yawlink_world 
		= world_baselinkST.E * baselink_yawlinkST.r;
	Vector3d p_yawlink_pitchbacklink_world 	
		= world_yawlinkST.E * yawlink_pitchbacklinkST.r;
	Vector3d p_pitchbacklink_pitchbottomlink_world  
		= world_pitchbacklinkST.E * pitchbacklink_pitchbottomlinkST.r;
	Vector3d p_pitchbottomlink_pitchendlink_world 
		= world_pitchbottomlinkST.E * pitchbottomlink_pitchendlinkST.r;

	Vector3d p_yawlink_pitchfrontlink_world 
		= world_yawlinkST.E * yawlink_pitchfrontlinkST.r;
	Vector3d p_pitchfrontlink_pitchbottomlink_world 
		= world_pitchfrontlinkST.E * pitchfrontlink_pitchbottomlinkST.r;
	Vector3d p_pitchfrontlink_pitchtoplink_world 
		= world_pitchfrontlinkST.E * pitchfrontlink_pitchtoplinkST.r;
	Vector3d p_pitchtoplink_pitchendlink_world 
		= world_pitchtoplinkST.E * pitchtoplink_pitchendlinkST.r;

	Vector3d p_pitchendlink_maininsertionlink_world = world_pitchendlinkST.E 		  * pitchendlink_maininsertionlinkST.r;
	Vector3d p_maininsertionlink_toollink_world     = world_maininsertionlinkST.E * maininsertionlink_toollinkST.r;
	//1--------------------------------------------------------------------//
	Joint joint_base = 
    Joint(JointTypeFixed);
	world_baselinkId = rbdlModelPtr_-> 
		AddBody(0, Xtrans(world_baselinkST.r), joint_base, baselinkBody_, "world-baselink");

	// Joint joint_yaw = Joint(SpatialVector (0., -1., 0., 0., 0., 0.));
  Joint baselink_yawlinkJ = Joint(SpatialVector ( 
    baselink_yawlinkJAxis(0), 
    baselink_yawlinkJAxis(1), 
    baselink_yawlinkJAxis(2),
    0., 0., 0.));
	baselink_yawlinkId = rbdlModelPtr_->
		AddBody(world_baselinkId, Xtrans(p_baselink_yawlink_world), 
		baselink_yawlinkJ, yawlinkBody_, "baselink-yawlink");
	
  Joint yawlink_pitchbacklinkJ = Joint(SpatialVector ( 
    yawlink_pitchbacklinkJAxis(0), 
    yawlink_pitchbacklinkJAxis(1), 
    yawlink_pitchbacklinkJAxis(2),
    0., 0., 0.));
	yawlink_pitchbacklinkId = rbdlModelPtr_->
		AddBody(baselink_yawlinkId, Xtrans(p_yawlink_pitchbacklink_world), 
		// Joint(SpatialVector (-1., 0., 0., 0., 0., 0.)), pitchbacklinkBody_, "yawlink-pitchbacklink");
    yawlink_pitchbacklinkJ, pitchbacklinkBody_, "yawlink-pitchbacklink");

  Joint pitchbacklink_pitchbottomlinkJ = Joint(SpatialVector (
    pitchbacklink_pitchbottomlinkJAxis(0), 
    pitchbacklink_pitchbottomlinkJAxis(1), 
    pitchbacklink_pitchbottomlinkJAxis(2),
    0., 0., 0.));
	pitchbacklink_pitchbottomlinkId = rbdlModelPtr_->
		AddBody(yawlink_pitchbacklinkId, Xtrans(p_pitchbacklink_pitchbottomlink_world), 
		// Joint(SpatialVector (-1., 0., 0., 0., 0., 0.)), pitchbottomlinkBody_, "pitchbacklink-pitchbottomlink");
    pitchbacklink_pitchbottomlinkJ, pitchbottomlinkBody_, "pitchbacklink-pitchbottomlink");

  Joint pitchbottomlink_pitchendlinkJ = Joint(SpatialVector (
    pitchbottomlink_pitchendlinkJAxis(0), 
    pitchbottomlink_pitchendlinkJAxis(1), 
    pitchbottomlink_pitchendlinkJAxis(2), 
    0., 0., 0.));
	pitchbottomlink_pitchendlinkId = rbdlModelPtr_->
		AddBody(pitchbacklink_pitchbottomlinkId, Xtrans(p_pitchbottomlink_pitchendlink_world), 
		// Joint(SpatialVector (-1., 0., 0., 0., 0., 0.)), pitchendlinkBody_, "pitchbottomlink-pitchendlink");
    pitchbottomlink_pitchendlinkJ, pitchendlinkBody_, "pitchbottomlink-pitchendlink");
	
  Joint yawlink_pitchfrontlinkJ = Joint(SpatialVector (
    yawlink_pitchfrontlinkJAxis(0), 
    yawlink_pitchfrontlinkJAxis(1), 
    yawlink_pitchfrontlinkJAxis(2),
    0., 0., 0.));
	yawlink_pitchfrontlinkId = rbdlModelPtr_->
		AddBody(baselink_yawlinkId, Xtrans(p_yawlink_pitchfrontlink_world), 
		// Joint(SpatialVector (-1., 0., 0., 0., 0., 0.)), pitchfrontlinkBody_, "yawlink-pitchfrontlink");
    yawlink_pitchfrontlinkJ, pitchfrontlinkBody_, "yawlink-pitchfrontlink");

	// pitchfrontlink_pitchbottomlinkId = rbdlModelPtr_->
	// 	AddBody(yawlink_pitchfrontlinkId, Xtrans(p_pitchfrontlink_pitchbottomlink_world), 
	// 	Joint(SpatialVector (0., 0., 0., 0., 0., 0.)), virtualBody_, "pitchfrontlink-pitchbottomlink");

  Joint pitchfrontlink_pitchtoplinkJ = Joint(SpatialVector (
    pitchfrontlink_pitchtoplinkJAxis(0), 
    pitchfrontlink_pitchtoplinkJAxis(1), 
    pitchfrontlink_pitchtoplinkJAxis(2),
    0., 0., 0.));
	pitchfrontlink_pitchtoplinkId = rbdlModelPtr_->
		AddBody(yawlink_pitchfrontlinkId, Xtrans(p_pitchfrontlink_pitchtoplink_world), 
		// Joint(SpatialVector (-1., 0., 0., 0., 0., 0.)), pitchtoplinkBody_, 
    pitchfrontlink_pitchtoplinkJ, pitchtoplinkBody_, 
		"pitchfrontlink-pitchtoplink");

  Joint pitchtoplink_pitchendlinkJ = Joint(SpatialVector (
    pitchtoplink_pitchendlinkJAxis(0), 
    pitchtoplink_pitchendlinkJAxis(1), 
    pitchtoplink_pitchendlinkJAxis(2),
    0., 0., 0.));
	pitchtoplink_pitchendlinkId = rbdlModelPtr_->
		AddBody(pitchfrontlink_pitchtoplinkId, Xtrans(p_pitchtoplink_pitchendlink_world), 
		// Joint(SpatialVector (-1., 0., 0., 0., 0., 0.)), virtualBody_, "pitchtoplink-pitchendlink");
    pitchtoplink_pitchendlinkJ, virtualBody_, "pitchtoplink-pitchendlink");

  Joint pitchendlink_maininsertionlinkJ = Joint(SpatialVector ( 
    0., 0., 0., 
    pitchendlink_maininsertionlinkJAxis(0), 
    pitchendlink_maininsertionlinkJAxis(1), 
    pitchendlink_maininsertionlinkJAxis(2)));
	pitchendlink_maininsertionlinkId = rbdlModelPtr_->
		AddBody(pitchbottomlink_pitchendlinkId, Xtrans(p_pitchendlink_maininsertionlink_world), 
		// Joint(SpatialVector (0., 0., 0., 0., 0., -1.)), pitchendlinkBody_, "pitchendlink-maininsertionlink");
    pitchendlink_maininsertionlinkJ, pitchendlinkBody_, "pitchendlink-maininsertionlink");

  Joint maininsertionlink_toollinkJ = Joint(SpatialVector (
    maininsertionlink_toollinkJAxis(0), 
    maininsertionlink_toollinkJAxis(1), 
    maininsertionlink_toollinkJAxis(2),
    0., 0., 0.));
	maininsertionlink_toollinkId = rbdlModelPtr_->
		AddBody(pitchendlink_maininsertionlinkId, Xtrans(p_maininsertionlink_toollink_world), 
		// Joint(SpatialVector (0., 0., 1., 0., 0., 0.)), pitchendlinkBody_, "maininsertionlink-toollink");
    maininsertionlink_toollinkJ, pitchendlinkBody_, "maininsertionlink-toollink");
	// --------------------------------------------------------------------//
	// unsigned int userDefinedId = 7;
	// cs_.AddLoopConstraint(yawlink_pitchfrontlinkId, pitchbacklink_pitchbottomlink_v_Id, X_p_, X_s_, 
	// 	SpatialVector(0, 0, 0, 1, 0, 0), bgStab_, 0.1, "LoopXY_Rz", userDefinedId);

	// //These two constraints below will be appended to the above
	// //constraint by default, and will assume its name and user defined id
	// cs_.AddLoopConstraint(yawlink_pitchfrontlinkId, pitchbacklink_pitchbottomlink_v_Id, X_p_, X_s_, 
	// 	SpatialVector(0, 0, 0, 0, 1, 0));
	// cs_.AddLoopConstraint(yawlink_pitchfrontlinkId, pitchbacklink_pitchbottomlink_v_Id, X_p_, X_s_, 
	// 	SpatialVector(0, 0, 1, 0, 0, 0));
	// cs_.Bind(*rbdlModelPtr_);

	Q_     = VectorNd::Constant ((size_t) rbdlModelPtr_->dof_count, 0.);
	QDot_  = VectorNd::Constant ((size_t) rbdlModelPtr_->dof_count, 0.);
	QDDot_ = VectorNd::Constant ((size_t) rbdlModelPtr_->dof_count, 0.);
	Tau_   = VectorNd::Constant ((size_t) rbdlModelPtr_->dof_count, 0.); 
	rbdlmBodyMap_ = rbdlModelPtr_->mBodyNameMap;
  
	ClearLogOutput();

	for(rbdlmBodyMapItr_ = rbdlmBodyMap_.begin(); rbdlmBodyMapItr_ != rbdlmBodyMap_.end(); rbdlmBodyMapItr_++)
  {
    std::string bodyName = rbdlmBodyMapItr_->first;
    unsigned int bodyId = rbdlmBodyMapItr_->second;
    std::string parentName = rbdlModelPtr_->GetBodyName(rbdlModelPtr_->GetParentBodyId(bodyId));
		bool isFixedBody = rbdlModelPtr_->IsFixedBodyId(bodyId);
    std::cout << parentName << ", " << bodyName << ", " << bodyId << ", " << isFixedBody << std::endl;
  }

  }

  ~ECM () {
    delete rbdlModelPtr_;

  }
  RBDLModelPtr rbdlModelPtr_{nullptr};
  // ConstraintSet cs_;


  std::map< std::string, unsigned int > rbdlmBodyMap;
  std::map<std::string, unsigned int>::iterator rbdlmBodyMapItr;

  // ConstraintSet cs_;
  bool bgStab_{true};
  SpatialTransform X_p_;
  SpatialTransform X_s_;
  Body baselinkBody_, pitchendlinkBody_, maininsertionlinkBody_, toollinkBody_, yawlinkBody_, 
    pitchbacklinkBody_, pitchbottomlinkBody_, pitchfrontlinkBody_, pitchtoplinkBody_, virtualBody_;

  SpatialTransform world_baselinkST, world_yawlinkST, world_pitchfrontlinkST, world_pitchbacklinkST, 
	world_pitchbottomlinkST, world_pitchbottomlink2ST, world_pitchendlinkST, world_maininsertionlinkST, world_pitchtoplinkST,
	world_toollinkST, world_eeST;
  // VectorNd err = VectorNd::Zero(cs_.size());

  VectorNd Q_;
  VectorNd QDot_;
  VectorNd QDDot_;
  VectorNd Tau_;

  std::map< std::string, unsigned int > rbdlmBodyMap_;
  std::map<std::string, unsigned int>::iterator rbdlmBodyMapItr_;
};

/*
TEST_CASE_METHOD ( ECM, __FILE__"_HomePose", "") 
{
  Q_.setZero();

  Vector3d p_w_baselink = CalcBodyToBaseCoordinates(*rbdlModelPtr_, Q_, 
    rbdlModelPtr_->GetBodyId("world-baselink"), Vector3d(0., 0., 0.), true);
  CHECK_THAT (Vector3d(0.4999, -0.3901, -0.599), 
    AllCloseVector(p_w_baselink, TEST_PREC, TEST_PREC));

  Vector3d p_w_yawlink = CalcBodyToBaseCoordinates(*rbdlModelPtr_, Q_, 
    rbdlModelPtr_->GetBodyId("baselink-yawlink"), Vector3d(0., 0., 0.), true);
  CHECK_THAT (Vector3d(0.49987, -0.927, -0.598924), 
    AllCloseVector(p_w_yawlink, TEST_PREC, TEST_PREC));

  Vector3d p_w_pitchbacklink = CalcBodyToBaseCoordinates(*rbdlModelPtr_, Q_, 
    rbdlModelPtr_->GetBodyId("yawlink-pitchbacklink"), Vector3d(0., 0., 0.), true);
	CHECK_THAT (Vector3d(0.5,   -0.764619,   -0.608807), 
    AllCloseVector(p_w_pitchbacklink, TEST_PREC, TEST_PREC));

  Vector3d p_w_pitchbottomlink = CalcBodyToBaseCoordinates(*rbdlModelPtr_, Q_, 
    rbdlModelPtr_->GetBodyId("pitchbacklink-pitchbottomlink"), Vector3d(0., 0., 0.), true);
  CHECK_THAT (Vector3d(0.5, -0.831042, -0.312319), 
    AllCloseVector(p_w_pitchbottomlink, TEST_PREC, TEST_PREC));

  Vector3d p_w_pitchendlink = CalcBodyToBaseCoordinates(*rbdlModelPtr_, Q_, 
    rbdlModelPtr_->GetBodyId("pitchbottomlink-pitchendlink"), Vector3d(0., 0., 0.), true);
  CHECK_THAT (Vector3d(0.500104, -0.490944, -0.31224), 
    AllCloseVector(p_w_pitchendlink, TEST_PREC, TEST_PREC));

  Vector3d p_w_pitchfrontlink = CalcBodyToBaseCoordinates(*rbdlModelPtr_, Q_, 
    rbdlModelPtr_->GetBodyId("yawlink-pitchfrontlink"), Vector3d(0., 0., 0.), true);
  CHECK_THAT (Vector3d(0.5, -0.728002, -0.599003), 
    AllCloseVector(p_w_pitchfrontlink, TEST_PREC, TEST_PREC));

  Vector3d p_w_pitchtoplink = CalcBodyToBaseCoordinates(*rbdlModelPtr_, Q_, 
    rbdlModelPtr_->GetBodyId("pitchfrontlink-pitchtoplink"), Vector3d(0., 0., 0.), true);
  CHECK_THAT (Vector3d(0.499403, -0.836886, -0.274964), 
    AllCloseVector(p_w_pitchtoplink, TEST_PREC, TEST_PREC));

  p_w_pitchendlink = CalcBodyToBaseCoordinates(*rbdlModelPtr_, Q_, 
    rbdlModelPtr_->GetBodyId("pitchtoplink-pitchendlink"), Vector3d(0., 0., 0.), true);
  CHECK_THAT (Vector3d(0.499902, -0.490902, -0.312198), 
    AllCloseVector(p_w_pitchendlink, TEST_PREC, TEST_PREC));

  Vector3d p_w_maininsertionlink = CalcBodyToBaseCoordinates(*rbdlModelPtr_, Q_, 
    rbdlModelPtr_->GetBodyId("pitchendlink-maininsertionlink"), Vector3d(0., 0., 0.), true);
  CHECK_THAT (Vector3d(0.500025, -0.449857, -0.226898), 
    AllCloseVector(p_w_maininsertionlink, TEST_PREC, TEST_PREC));

  Vector3d p_w_toollink = CalcBodyToBaseCoordinates(*rbdlModelPtr_, Q_, 
    rbdlModelPtr_->GetBodyId("maininsertionlink-toollink"), Vector3d(0., 0., 0.), true);
  CHECK_THAT (Vector3d(0.500025, -0.387959, -0.227233), 
	// CHECK_THAT (Vector3d(0.500219,   -0.388462,   -0.229241), 
    AllCloseVector(p_w_toollink, TEST_PREC, TEST_PREC));
}
*/



/*
TEST_CASE_METHOD ( ECM, __FILE__"YawlinkFK", "") 
{
  Q_.setZero();
  // Q_[rbdlModelPtr_->GetBodyId("baselink-yawlink") - 1]                = 0; // 0
  // Q_[rbdlModelPtr_->GetBodyId("yawlink-pitchbacklink") - 1]           = 0.1; // 1
  // Q_[rbdlModelPtr_->GetBodyId("pitchbacklink-pitchbottomlink") - 1]   = 0.2; // 2
  // Q_[rbdlModelPtr_->GetBodyId("pitchbottomlink-pitchendlink") - 1]    = 0.3; // 3
  // Q_[rbdlModelPtr_->GetBodyId("yawlink-pitchfrontlink") - 1]          = 0.4; // 4
  // Q_[rbdlModelPtr_->GetBodyId("pitchfrontlink-pitchbottomlink") - 1]  = 0.5; // 5
  // Q_[rbdlModelPtr_->GetBodyId("pitchfrontlink-pitchtoplink") - 1]     = 0.6; // 6
  // Q_[rbdlModelPtr_->GetBodyId("pitchtoplink-pitchendlink") - 1]       = 0.7; // 7
  // Q_[rbdlModelPtr_->GetBodyId("pitchendlink-maininsertionlink") - 1]  = 0.8; // 8
  // Q_[rbdlModelPtr_->GetBodyId("maininsertionlink-toollink") - 1]      = 0.9; // 9

  Q_[rbdlModelPtr_->GetBodyId("baselink-yawlink") - 1]                = 0; // 0
  Q_[rbdlModelPtr_->GetBodyId("yawlink-pitchbacklink") - 1]           = 0.; // 1
  // Q_[rbdlModelPtr_->GetBodyId("pitchbacklink-pitchbottomlink") - 1]   = -0.4190663993358612; // 2
  // Q_[rbdlModelPtr_->GetBodyId("pitchbottomlink-pitchendlink") - 1]    = 1.0532910823822021; // 3
  // Q_[rbdlModelPtr_->GetBodyId("yawlink-pitchfrontlink") - 1]          = 0.25388258695602417; // 4
  // Q_[rbdlModelPtr_->GetBodyId("pitchfrontlink-pitchbottomlink") - 1]  = -1.0478633642196655; // 5
  // Q_[rbdlModelPtr_->GetBodyId("pitchfrontlink-pitchtoplink") - 1]     = -1.04469895362854; // 6
  // Q_[rbdlModelPtr_->GetBodyId("pitchtoplink-pitchendlink") - 1]       = 1.0501266717910767; // 7
  // Q_[rbdlModelPtr_->GetBodyId("pitchendlink-maininsertionlink") - 1]  = 0.8; // 8
  // Q_[rbdlModelPtr_->GetBodyId("maininsertionlink-toollink") - 1]      = 0.9; // 9
  std::cout << std::endl << Q_ << std::endl;

	// No link can change this
  Vector3d p_w_baselink = CalcBodyToBaseCoordinates(*rbdlModelPtr_, Q_, 
    rbdlModelPtr_->GetBodyId("world-baselink"), Vector3d(0., 0., 0.), true);
  CHECK_THAT (Vector3d(0.4999, -0.3901, -0.599), 
    AllCloseVector(p_w_baselink, TEST_PREC, TEST_PREC));


	// No link can change this
  Vector3d p_w_yawlink = CalcBodyToBaseCoordinates(*rbdlModelPtr_, Q_, 
    rbdlModelPtr_->GetBodyId("baselink-yawlink"), Vector3d(0., 0., 0.), true);
  CHECK_THAT (Vector3d(0.49987, -0.927, -0.598924), 
    AllCloseVector(p_w_yawlink, TEST_PREC, TEST_PREC));

	// No link can change this
  Vector3d p_w_pitchbacklink = CalcBodyToBaseCoordinates(*rbdlModelPtr_, Q_, 
    rbdlModelPtr_->GetBodyId("yawlink-pitchbacklink"), Vector3d(0., 0., 0.), true);
  // CHECK_THAT (Vector3d(0.5, -0.764604, -0.605786), 
  CHECK_THAT (Vector3d(0.5, -0.764493, -0.608647), 
    AllCloseVector(p_w_pitchbacklink, TEST_PREC, TEST_PREC));

	// yawlink-pitchbacklink
  Vector3d p_w_pitchbottomlink = CalcBodyToBaseCoordinates(*rbdlModelPtr_, Q_, 
    rbdlModelPtr_->GetBodyId("pitchbacklink-pitchbottomlink"), Vector3d(0., 0., 0.), true);
	CHECK_THAT (Vector3d(0.499559,   -0.831125,   -0.314401), 
    AllCloseVector(p_w_pitchbottomlink, TEST_PREC, TEST_PREC));

  // Vector3d p_w_pitchendlink = CalcBodyToBaseCoordinates(*rbdlModelPtr_, Q_, 
  //   rbdlModelPtr_->GetBodyId("pitchbottomlink-pitchendlink"), Vector3d(0., 0., 0.), true);
  // CHECK_THAT (Vector3d(0.500104, -0.490944, -0.31224), 
  //   AllCloseVector(p_w_pitchendlink, TEST_PREC, TEST_PREC));

	// No link can change this
  // Vector3d p_w_pitchfrontlink = CalcBodyToBaseCoordinates(*rbdlModelPtr_, Q_, 
  //   rbdlModelPtr_->GetBodyId("yawlink-pitchfrontlink"), Vector3d(0., 0., 0.), true);
  // CHECK_THAT (Vector3d(0.5, -0.727978, -0.59897), 
  //   AllCloseVector(p_w_pitchfrontlink, TEST_PREC, TEST_PREC));

  // Vector3d p_w_pitchtoplink = CalcBodyToBaseCoordinates(*rbdlModelPtr_, Q_, 
  //   rbdlModelPtr_->GetBodyId("pitchfrontlink-pitchtoplink"), Vector3d(0., 0., 0.), true);
  // CHECK_THAT (Vector3d(0.499442,   -0.474463,   -0.370333), 
  //   AllCloseVector(p_w_pitchtoplink, TEST_PREC, TEST_PREC));
  // CHECK_THAT (Vector3d(0.499442, -0.474463, -0.370333), 
  //   AllCloseVector(p_w_pitchtoplink, TEST_PREC, TEST_PREC));

  // p_w_pitchendlink = CalcBodyToBaseCoordinates(*rbdlModelPtr_, Q_, 
  //   rbdlModelPtr_->GetBodyId("pitchtoplink-pitchendlink"), Vector3d(0., 0., 0.), true);
  // CHECK_THAT (Vector3d(0.499902, -0.490902, -0.312198), 
  //   AllCloseVector(p_w_pitchendlink, TEST_PREC, TEST_PREC));

  // Vector3d p_w_maininsertionlink = CalcBodyToBaseCoordinates(*rbdlModelPtr_, Q_, 
  //   rbdlModelPtr_->GetBodyId("pitchendlink-maininsertionlink"), Vector3d(0., 0., 0.), true);
  // // CHECK_THAT (Vector3d(0.500025, -0.449857, -0.228209), 
  // CHECK_THAT (Vector3d(0.500025, -0.449857, -0.226898), 
  //   AllCloseVector(p_w_maininsertionlink, TEST_PREC, TEST_PREC));

  // Vector3d p_w_toollink = CalcBodyToBaseCoordinates(*rbdlModelPtr_, Q_, 
  //   rbdlModelPtr_->GetBodyId("maininsertionlink-toollink"), Vector3d(0., 0., 0.), true);
  // CHECK_THAT (Vector3d(0.500195,  -0.0656258,   -0.421017), 
  //   AllCloseVector(p_w_toollink, TEST_PREC, TEST_PREC));
}
*/

/*

TEST_CASE_METHOD ( ECM, __FILE__"YawlinkFK", "") 
{
  Q_.setZero();
  // Q_[rbdlModelPtr_->GetBodyId("baselink-yawlink") - 1]                = 0; // 0
  // Q_[rbdlModelPtr_->GetBodyId("yawlink-pitchbacklink") - 1]           = 0.1; // 1
  // Q_[rbdlModelPtr_->GetBodyId("pitchbacklink-pitchbottomlink") - 1]   = 0.2; // 2
  // Q_[rbdlModelPtr_->GetBodyId("pitchbottomlink-pitchendlink") - 1]    = 0.3; // 3
  // Q_[rbdlModelPtr_->GetBodyId("yawlink-pitchfrontlink") - 1]          = 0.4; // 4
  // Q_[rbdlModelPtr_->GetBodyId("pitchfrontlink-pitchbottomlink") - 1]  = 0.5; // 5
  // Q_[rbdlModelPtr_->GetBodyId("pitchfrontlink-pitchtoplink") - 1]     = 0.6; // 6
  // Q_[rbdlModelPtr_->GetBodyId("pitchtoplink-pitchendlink") - 1]       = 0.7; // 7
  // Q_[rbdlModelPtr_->GetBodyId("pitchendlink-maininsertionlink") - 1]  = 0.8; // 8
  // Q_[rbdlModelPtr_->GetBodyId("maininsertionlink-toollink") - 1]      = 0.9; // 9

  Q_[rbdlModelPtr_->GetBodyId("baselink-yawlink") - 1]                = 0; // 0
  Q_[rbdlModelPtr_->GetBodyId("yawlink-pitchbacklink") - 1]           = 0.5032803416252136; // 1
  // Q_[rbdlModelPtr_->GetBodyId("pitchbacklink-pitchbottomlink") - 1]   = -1.0471999645233154; // 2
  // Q_[rbdlModelPtr_->GetBodyId("pitchbottomlink-pitchendlink") - 1]    = 1.0532910823822021; // 3
  // Q_[rbdlModelPtr_->GetBodyId("yawlink-pitchfrontlink") - 1]          = 1.15951; // 4
  // Q_[rbdlModelPtr_->GetBodyId("pitchfrontlink-pitchbottomlink") - 1]  = -1.0478633642196655; // 5
  // Q_[rbdlModelPtr_->GetBodyId("pitchfrontlink-pitchtoplink") - 1]     = -1.04469895362854; // 6
  // Q_[rbdlModelPtr_->GetBodyId("pitchtoplink-pitchendlink") - 1]       = 1.0501266717910767; // 7
  // Q_[rbdlModelPtr_->GetBodyId("pitchendlink-maininsertionlink") - 1]  = 0.8; // 8
  // Q_[rbdlModelPtr_->GetBodyId("maininsertionlink-toollink") - 1]      = 0.9; // 9
  std::cout << std::endl << Q_ << std::endl;

	// No link can change this
  Vector3d p_w_baselink = CalcBodyToBaseCoordinates(*rbdlModelPtr_, Q_, 
    rbdlModelPtr_->GetBodyId("world-baselink"), Vector3d(0., 0., 0.), true);
  CHECK_THAT (Vector3d(0.4999, -0.3901, -0.599), 
    AllCloseVector(p_w_baselink, TEST_PREC, TEST_PREC));


	// No link can change this
  Vector3d p_w_yawlink = CalcBodyToBaseCoordinates(*rbdlModelPtr_, Q_, 
    rbdlModelPtr_->GetBodyId("baselink-yawlink"), Vector3d(0., 0., 0.), true);
  CHECK_THAT (Vector3d(0.49987, -0.927, -0.598924), 
    AllCloseVector(p_w_yawlink, TEST_PREC, TEST_PREC));

	// No link can change this
  Vector3d p_w_pitchbacklink = CalcBodyToBaseCoordinates(*rbdlModelPtr_, Q_, 
    rbdlModelPtr_->GetBodyId("yawlink-pitchbacklink"), Vector3d(0., 0., 0.), true);
  // CHECK_THAT (Vector3d(0.5, -0.764604, -0.605786), 
  CHECK_THAT (Vector3d(0.5, -0.764493, -0.608647), 
    AllCloseVector(p_w_pitchbacklink, TEST_PREC, TEST_PREC));

	// yawlink-pitchbacklink
  Vector3d p_w_pitchbottomlink = CalcBodyToBaseCoordinates(*rbdlModelPtr_, Q_, 
    rbdlModelPtr_->GetBodyId("pitchbacklink-pitchbottomlink"), Vector3d(0., 0., 0.), true);
	// CHECK_THAT (Vector3d(0.499997,   -0.764536,   -0.608716), 
  //   AllCloseVector(p_w_pitchbottomlink, TEST_PREC, TEST_PREC));

  // Vector3d p_w_pitchendlink = CalcBodyToBaseCoordinates(*rbdlModelPtr_, Q_, 
  //   rbdlModelPtr_->GetBodyId("pitchbottomlink-pitchendlink"), Vector3d(0., 0., 0.), true);
  // CHECK_THAT (Vector3d(0.500104, -0.490944, -0.31224), 
  //   AllCloseVector(p_w_pitchendlink, TEST_PREC, TEST_PREC));

	// No link can change this
  // Vector3d p_w_pitchfrontlink = CalcBodyToBaseCoordinates(*rbdlModelPtr_, Q_, 
  //   rbdlModelPtr_->GetBodyId("yawlink-pitchfrontlink"), Vector3d(0., 0., 0.), true);
  // CHECK_THAT (Vector3d(0.5, -0.727978, -0.59897), 
  //   AllCloseVector(p_w_pitchfrontlink, TEST_PREC, TEST_PREC));

  // Vector3d p_w_pitchtoplink = CalcBodyToBaseCoordinates(*rbdlModelPtr_, Q_, 
  //   rbdlModelPtr_->GetBodyId("pitchfrontlink-pitchtoplink"), Vector3d(0., 0., 0.), true);
  // CHECK_THAT (Vector3d(0.499442,   -0.474463,   -0.370333), 
  //   AllCloseVector(p_w_pitchtoplink, TEST_PREC, TEST_PREC));
  // CHECK_THAT (Vector3d(0.499442, -0.474463, -0.370333), 
  //   AllCloseVector(p_w_pitchtoplink, TEST_PREC, TEST_PREC));

  // p_w_pitchendlink = CalcBodyToBaseCoordinates(*rbdlModelPtr_, Q_, 
  //   rbdlModelPtr_->GetBodyId("pitchtoplink-pitchendlink"), Vector3d(0., 0., 0.), true);
  // CHECK_THAT (Vector3d(0.499902, -0.490902, -0.312198), 
  //   AllCloseVector(p_w_pitchendlink, TEST_PREC, TEST_PREC));

  // Vector3d p_w_maininsertionlink = CalcBodyToBaseCoordinates(*rbdlModelPtr_, Q_, 
  //   rbdlModelPtr_->GetBodyId("pitchendlink-maininsertionlink"), Vector3d(0., 0., 0.), true);
  // // CHECK_THAT (Vector3d(0.500025, -0.449857, -0.228209), 
  // CHECK_THAT (Vector3d(0.500025, -0.449857, -0.226898), 
  //   AllCloseVector(p_w_maininsertionlink, TEST_PREC, TEST_PREC));

  // Vector3d p_w_toollink = CalcBodyToBaseCoordinates(*rbdlModelPtr_, Q_, 
  //   rbdlModelPtr_->GetBodyId("maininsertionlink-toollink"), Vector3d(0., 0., 0.), true);
  // CHECK_THAT (Vector3d(0.500195,  -0.0656258,   -0.421017), 
  //   AllCloseVector(p_w_toollink, TEST_PREC, TEST_PREC));
}
*/


TEST_CASE_METHOD ( ECM, __FILE__"YawlinkFK", "") 
{
  Q_.setZero();
  // Q_[rbdlModelPtr_->GetBodyId("baselink-yawlink") - 1]                = 0; // 0
  // Q_[rbdlModelPtr_->GetBodyId("yawlink-pitchbacklink") - 1]           = 0.1; // 1
  // Q_[rbdlModelPtr_->GetBodyId("pitchbacklink-pitchbottomlink") - 1]   = 0.2; // 2
  // Q_[rbdlModelPtr_->GetBodyId("pitchbottomlink-pitchendlink") - 1]    = 0.3; // 3
  // Q_[rbdlModelPtr_->GetBodyId("yawlink-pitchfrontlink") - 1]          = 0.4; // 4
  // Q_[rbdlModelPtr_->GetBodyId("pitchfrontlink-pitchbottomlink") - 1]  = 0.5; // 5
  // Q_[rbdlModelPtr_->GetBodyId("pitchfrontlink-pitchtoplink") - 1]     = 0.6; // 6
  // Q_[rbdlModelPtr_->GetBodyId("pitchtoplink-pitchendlink") - 1]       = 0.7; // 7
  // Q_[rbdlModelPtr_->GetBodyId("pitchendlink-maininsertionlink") - 1]  = 0.8; // 8
  // Q_[rbdlModelPtr_->GetBodyId("maininsertionlink-toollink") - 1]      = 0.9; // 9

  Q_[rbdlModelPtr_->GetBodyId("baselink-yawlink") - 1]                = 0; // 0
  Q_[rbdlModelPtr_->GetBodyId("yawlink-pitchbacklink") - 1]           = 0.5003815293312073; // 1
  Q_[rbdlModelPtr_->GetBodyId("pitchbacklink-pitchbottomlink") - 1]   = -0.49938082695007324; // 2
  // Q_[rbdlModelPtr_->GetBodyId("pitchbottomlink-pitchendlink") - 1]    = 1.0532910823822021; // 3
  // Q_[rbdlModelPtr_->GetBodyId("yawlink-pitchfrontlink") - 1]          = 1.15951; // 4
  // Q_[rbdlModelPtr_->GetBodyId("pitchfrontlink-pitchbottomlink") - 1]  = -1.0478633642196655; // 5
  // Q_[rbdlModelPtr_->GetBodyId("pitchfrontlink-pitchtoplink") - 1]     = -1.04469895362854; // 6
  // Q_[rbdlModelPtr_->GetBodyId("pitchtoplink-pitchendlink") - 1]       = 1.0501266717910767; // 7
  // Q_[rbdlModelPtr_->GetBodyId("pitchendlink-maininsertionlink") - 1]  = 0.8; // 8
  // Q_[rbdlModelPtr_->GetBodyId("maininsertionlink-toollink") - 1]      = 0.9; // 9
  std::cout << std::endl << Q_ << std::endl;

	// No link can change this
  Vector3d p_w_baselink = CalcBodyToBaseCoordinates(*rbdlModelPtr_, Q_, 
    rbdlModelPtr_->GetBodyId("world-baselink"), Vector3d(0., 0., 0.), true);
  CHECK_THAT (Vector3d(0.4999, -0.3901, -0.599), 
    AllCloseVector(p_w_baselink, TEST_PREC, TEST_PREC));

	// No link can change this
  Vector3d p_w_yawlink = CalcBodyToBaseCoordinates(*rbdlModelPtr_, Q_, 
    rbdlModelPtr_->GetBodyId("baselink-yawlink"), Vector3d(0., 0., 0.), true);
  CHECK_THAT (Vector3d(0.49987, -0.927, -0.598924), 
    AllCloseVector(p_w_yawlink, TEST_PREC, TEST_PREC));

	// No link can change this
  Vector3d p_w_pitchbacklink = CalcBodyToBaseCoordinates(*rbdlModelPtr_, Q_, 
    rbdlModelPtr_->GetBodyId("yawlink-pitchbacklink"), Vector3d(0., 0., 0.), true);
  CHECK_THAT (Vector3d(0.499999,   -0.764536,   -0.608716), 
    AllCloseVector(p_w_pitchbacklink, TEST_PREC, TEST_PREC));

	// yawlink-pitchbacklink
  Vector3d p_w_pitchbottomlink = CalcBodyToBaseCoordinates(*rbdlModelPtr_, Q_, 
    rbdlModelPtr_->GetBodyId("pitchbacklink-pitchbottomlink"), Vector3d(0., 0., 0.), true);
	CHECK_THAT (Vector3d(0.499444,   -0.680836,   -0.298169), 
    AllCloseVector(p_w_pitchbottomlink, TEST_PREC, TEST_PREC));

  // Vector3d p_w_pitchendlink = CalcBodyToBaseCoordinates(*rbdlModelPtr_, Q_, 
  //   rbdlModelPtr_->GetBodyId("pitchbottomlink-pitchendlink"), Vector3d(0., 0., 0.), true);
  // CHECK_THAT (Vector3d(0.500104, -0.490944, -0.31224), 
  //   AllCloseVector(p_w_pitchendlink, TEST_PREC, TEST_PREC));

	// No link can change this
  // Vector3d p_w_pitchfrontlink = CalcBodyToBaseCoordinates(*rbdlModelPtr_, Q_, 
  //   rbdlModelPtr_->GetBodyId("yawlink-pitchfrontlink"), Vector3d(0., 0., 0.), true);
  // CHECK_THAT (Vector3d(0.5, -0.727978, -0.59897), 
  //   AllCloseVector(p_w_pitchfrontlink, TEST_PREC, TEST_PREC));

  // Vector3d p_w_pitchtoplink = CalcBodyToBaseCoordinates(*rbdlModelPtr_, Q_, 
  //   rbdlModelPtr_->GetBodyId("pitchfrontlink-pitchtoplink"), Vector3d(0., 0., 0.), true);
  // CHECK_THAT (Vector3d(0.499442,   -0.474463,   -0.370333), 
  //   AllCloseVector(p_w_pitchtoplink, TEST_PREC, TEST_PREC));
  // CHECK_THAT (Vector3d(0.499442, -0.474463, -0.370333), 
  //   AllCloseVector(p_w_pitchtoplink, TEST_PREC, TEST_PREC));

  // p_w_pitchendlink = CalcBodyToBaseCoordinates(*rbdlModelPtr_, Q_, 
  //   rbdlModelPtr_->GetBodyId("pitchtoplink-pitchendlink"), Vector3d(0., 0., 0.), true);
  // CHECK_THAT (Vector3d(0.499902, -0.490902, -0.312198), 
  //   AllCloseVector(p_w_pitchendlink, TEST_PREC, TEST_PREC));

  // Vector3d p_w_maininsertionlink = CalcBodyToBaseCoordinates(*rbdlModelPtr_, Q_, 
  //   rbdlModelPtr_->GetBodyId("pitchendlink-maininsertionlink"), Vector3d(0., 0., 0.), true);
  // // CHECK_THAT (Vector3d(0.500025, -0.449857, -0.228209), 
  // CHECK_THAT (Vector3d(0.500025, -0.449857, -0.226898), 
  //   AllCloseVector(p_w_maininsertionlink, TEST_PREC, TEST_PREC));

  // Vector3d p_w_toollink = CalcBodyToBaseCoordinates(*rbdlModelPtr_, Q_, 
  //   rbdlModelPtr_->GetBodyId("maininsertionlink-toollink"), Vector3d(0., 0., 0.), true);
  // CHECK_THAT (Vector3d(0.500195,  -0.0656258,   -0.421017), 
  //   AllCloseVector(p_w_toollink, TEST_PREC, TEST_PREC));
}


/*
TEST_CASE_METHOD ( ECM, __FILE__"YawlinkIK", "") 
{
  Q_.setZero();

  Q_[rbdlModelPtr_->GetBodyId("baselink-yawlink") - 1]                = 0; // 0
  Q_[rbdlModelPtr_->GetBodyId("yawlink-pitchbacklink") - 1]           = 0.5032803416252136; // 1
  Q_[rbdlModelPtr_->GetBodyId("pitchbacklink-pitchbottomlink") - 1]   = -0.5001809597015381; // 2
  // // Q_[rbdlModelPtr_->GetBodyId("pitchbottomlink-pitchendlink") - 1]    = 0.3; // 3
  // Q_[rbdlModelPtr_->GetBodyId("yawlink-pitchfrontlink") - 1]          = 1.05457; // 4
  // Q_[rbdlModelPtr_->GetBodyId("pitchfrontlink-pitchbottomlink") - 1]  = 0.5; // 5
  // Q_[rbdlModelPtr_->GetBodyId("pitchfrontlink-pitchtoplink") - 1]     = 0.6; // 6
  // Q_[rbdlModelPtr_->GetBodyId("pitchtoplink-pitchendlink") - 1]       = 0.7; // 7
  Q_[rbdlModelPtr_->GetBodyId("pitchendlink-maininsertionlink") - 1]  = 0.0; // 8
  Q_[rbdlModelPtr_->GetBodyId("maininsertionlink-toollink") - 1]      = 0.; // 9
  std::cout << std::endl << Q_ << std::endl;

  // Vector3d p_w_baselink = CalcBodyToBaseCoordinates(*rbdlModelPtr_, Q_, 
  //   rbdlModelPtr_->GetBodyId("world-baselink"), Vector3d(0., 0., 0.), true);
  // CHECK_THAT (Vector3d(0.4999, -0.3901, -0.599), 
  //   AllCloseVector(p_w_baselink, TEST_PREC, TEST_PREC));

  // Vector3d p_w_yawlink = CalcBodyToBaseCoordinates(*rbdlModelPtr_, Q_, 
  //   rbdlModelPtr_->GetBodyId("baselink-yawlink"), Vector3d(0., 0., 0.), true);
  // CHECK_THAT (Vector3d(0.49987, -0.927, -0.598924), 
  //   AllCloseVector(p_w_yawlink, TEST_PREC, TEST_PREC));

  // Vector3d p_w_pitchbacklink = CalcBodyToBaseCoordinates(*rbdlModelPtr_, Q_, 
  //   rbdlModelPtr_->GetBodyId("yawlink-pitchbacklink"), Vector3d(0., 0., 0.), true);
  // // CHECK_THAT (Vector3d(0.5, -0.764604, -0.605786), 
  // CHECK_THAT (Vector3d(0.5, -0.764493, -0.608647), 
  //   AllCloseVector(p_w_pitchbacklink, TEST_PREC, TEST_PREC));

  // // Vector3d p_w_pitchbottomlink = CalcBodyToBaseCoordinates(*rbdlModelPtr_, Q_, 
  // //   rbdlModelPtr_->GetBodyId("pitchbacklink-pitchbottomlink"), Vector3d(0., 0., 0.), true);
  // // CHECK_THAT (Vector3d(0.5, -0.528961, -0.368644), 
  // //   AllCloseVector(p_w_pitchbottomlink, TEST_PREC, TEST_PREC));

  // Vector3d p_w_pitchfrontlink = CalcBodyToBaseCoordinates(*rbdlModelPtr_, Q_, 
  //   rbdlModelPtr_->GetBodyId("yawlink-pitchfrontlink"), Vector3d(0., 0., 0.), true);
  // CHECK_THAT (Vector3d(0.5, -0.728002, -0.599003), 
  //   AllCloseVector(p_w_pitchfrontlink, TEST_PREC, TEST_PREC));


  UpdateKinematicsCustom (*rbdlModelPtr_, &Q_, NULL, NULL);

  Matrix3d r_w_baselink = 
    CalcBodyWorldOrientation(*rbdlModelPtr_, Q_, 
			rbdlModelPtr_->GetBodyId("baselink-yawlink"), true);

  Matrix3d r_w_yawlink = 
    CalcBodyWorldOrientation(*rbdlModelPtr_, Q_, 
			rbdlModelPtr_->GetBodyId("yawlink-pitchbacklink"), true);

  Matrix3d r_w_pitchendlink = 
    CalcBodyWorldOrientation(*rbdlModelPtr_, Q_, 
			rbdlModelPtr_->GetBodyId("pitchendlink-maininsertionlink"), true);

	// Matrix3d r_w_pitchendlink = 
  //   CalcBodyWorldOrientation(*rbdlModelPtr_, Q_, 
	// 		rbdlModelPtr_->GetBodyId("pitchendlink-maininsertionlink"), true);
	
	Matrix3d r_w_toollink = 
    CalcBodyWorldOrientation(*rbdlModelPtr_, Q_, 
			rbdlModelPtr_->GetBodyId("maininsertionlink-toollink"), true);


	Vector3d p_w_baselink      = Vector3d(  0.4999,     -0.3901,      -0.599);
	Vector3d p_w_pitchbacklink = Vector3d(0.499999,   -0.764536,   -0.608716);
  Vector3d p_w_pitchbottomlink = Vector3d(0.499435,   -0.679858,    -0.29831);

	Vector3d p_w_pitchtoplink  = Vector3d(0.499325,   -0.666444,   -0.262765);
	Vector3d p_w_toollink      = Vector3d(0.500146,   -0.209318,   -0.276304);

  InverseKinematicsConstraintSet cs;

	// Constraints for baselink
  cs.AddOrientationConstraint (
    rbdlModelPtr_->GetBodyId("baselink-yawlink"),
    r_w_baselink,
		1.0f
  );

  cs.AddPointConstraint (
		rbdlModelPtr_->GetBodyId("world-baselink"), 
    Vector3d (0., 0., 0.), p_w_baselink,
		1.0f
  );

  // Constraints for yawlink
  cs.AddOrientationConstraint (
    rbdlModelPtr_->GetBodyId("yawlink-pitchbacklink"),
    r_w_yawlink,
		1.0f
  );
  
  // Constraints for pitchbacklink
  cs.AddOrientationConstraint (
    rbdlModelPtr_->GetBodyId("yawlink-pitchbacklink"),
    r_w_yawlink,
		1.0f
  );

  cs.AddPointConstraint (
		rbdlModelPtr_->GetBodyId("yawlink-pitchbacklink"), 
    Vector3d (0., 0., 0.), p_w_pitchbacklink,
		1.0f);
  
  // cs.AddPointConstraint (
	// 	rbdlModelPtr_->GetBodyId("pitchbacklink-pitchbottomlink"), 
  //   Vector3d (0., 0., 0.), p_w_pitchbottomlink,
	// 	1.0f);

  // Constraints for pitchendlink
  // cs.AddOrientationConstraint (
  //   rbdlModelPtr_->GetBodyId("pitchendlink-maininsertionlink"),
  //   r_w_pitchendlink,
	// 	1.0f
  // );

  // Constraints for pitchtoplink
	// cs.AddPointConstraint (
	// 	rbdlModelPtr_->GetBodyId("pitchfrontlink-pitchtoplink"), 
  //   Vector3d (0., 0., 0.), p_w_pitchtoplink,
	// 	1.0f);

	// Constraints for toollink
  // cs.AddOrientationConstraint (
  //   rbdlModelPtr_->GetBodyId("maininsertionlink-toollink"),
  //   r_w_toollink,
	// 	1.0f
  // );
  // cs.AddPointConstraint (
	// 	rbdlModelPtr_->GetBodyId("maininsertionlink-toollink"), 
  //   Vector3d (0., 0., 0.), p_w_toollink,
	// 	1.0f
  // );
  cs.step_tol = 1e-2;
  
  // q.setZero();

  VectorNd Qres (Q_);

  bool result = InverseKinematics (*rbdlModelPtr_, Q_, cs, Qres);

  CHECK (result);
  // CHECK_THAT ( 0.0, IsClose(cs.error_norm, cs.step_tol, cs.step_tol));
  std::cout << "Qres" << std::endl << Qres << std::endl;

  UpdateKinematicsCustom (*rbdlModelPtr_, &Qres, NULL, NULL);

  Matrix3d r_w_baselinkCal = CalcBodyWorldOrientation(*rbdlModelPtr_, Qres, 
		rbdlModelPtr_->GetBodyId("baselink-yawlink"), true);
  Vector3d p_w_baselinkCal = CalcBodyToBaseCoordinates(*rbdlModelPtr_, Qres, 
    rbdlModelPtr_->GetBodyId("world-baselink"), Vector3d(0., 0., 0.), true);
  CHECK_THAT (r_w_baselink, 
    AllCloseMatrix(r_w_baselinkCal, TEST_PREC, TEST_PREC));  
  CHECK_THAT (p_w_baselink, 
    AllCloseVector(p_w_baselinkCal, TEST_PREC, TEST_PREC));

  // Matrix3d r_w_yawlinkCal = CalcBodyWorldOrientation(*rbdlModelPtr_, Qres, 
	// 	rbdlModelPtr_->GetBodyId("yawlink-pitchbacklink"), true);
  // CHECK_THAT (r_w_yawlink, 
  //   AllCloseMatrix(r_w_yawlinkCal, TEST_PREC, TEST_PREC));
  
  Vector3d p_w_pitchbacklinkCal = CalcBodyToBaseCoordinates(*rbdlModelPtr_, Qres, 
    rbdlModelPtr_->GetBodyId("yawlink-pitchbacklink"), Vector3d(0., 0., 0.), true);
  CHECK_THAT (p_w_pitchbacklink, 
    AllCloseVector(p_w_pitchbacklinkCal, TEST_PREC, TEST_PREC));

  // Matrix3d r_w_pitchendlinkCal = CalcBodyWorldOrientation(*rbdlModelPtr_, Qres, 
	// 	rbdlModelPtr_->GetBodyId("pitchendlink-maininsertionlink"), true);
  // CHECK_THAT (r_w_pitchendlink, 
  //   AllCloseMatrix(r_w_pitchendlinkCal, TEST_PREC, TEST_PREC));

  // Vector3d p_w_pitchtoplinkCal = CalcBodyToBaseCoordinates(*rbdlModelPtr_, Qres, 
  //   rbdlModelPtr_->GetBodyId("pitchfrontlink-pitchtoplink"), Vector3d(0., 0., 0.), true);
  // CHECK_THAT (p_w_pitchtoplink, 
  //   AllCloseVector(p_w_pitchtoplinkCal, TEST_PREC, TEST_PREC));
}
*/


/*
TEST_CASE_METHOD ( ECM, __FILE__"YawlinkIK", "") 
{
  Q_.setZero();

  Q_[rbdlModelPtr_->GetBodyId("baselink-yawlink") - 1]                = 0; // 0
  Q_[rbdlModelPtr_->GetBodyId("yawlink-pitchbacklink") - 1]           = 1.0563281774520874; // 1
  // Q_[rbdlModelPtr_->GetBodyId("pitchbacklink-pitchbottomlink") - 1]   = -1.0471999645233154; // 2
  // // Q_[rbdlModelPtr_->GetBodyId("pitchbottomlink-pitchendlink") - 1]    = 0.3; // 3
  Q_[rbdlModelPtr_->GetBodyId("yawlink-pitchfrontlink") - 1]          = 1.05457; // 4
  // Q_[rbdlModelPtr_->GetBodyId("pitchfrontlink-pitchbottomlink") - 1]  = 0.5; // 5
  // Q_[rbdlModelPtr_->GetBodyId("pitchfrontlink-pitchtoplink") - 1]     = 0.6; // 6
  // Q_[rbdlModelPtr_->GetBodyId("pitchtoplink-pitchendlink") - 1]       = 0.7; // 7
  Q_[rbdlModelPtr_->GetBodyId("pitchendlink-maininsertionlink") - 1]  = 0.0; // 8
  // Q_[rbdlModelPtr_->GetBodyId("maininsertionlink-toollink") - 1]      = 0.9; // 9
  std::cout << std::endl << Q_ << std::endl;

  // Vector3d p_w_baselink = CalcBodyToBaseCoordinates(*rbdlModelPtr_, Q_, 
  //   rbdlModelPtr_->GetBodyId("world-baselink"), Vector3d(0., 0., 0.), true);
  // CHECK_THAT (Vector3d(0.4999, -0.3901, -0.599), 
  //   AllCloseVector(p_w_baselink, TEST_PREC, TEST_PREC));

  // Vector3d p_w_yawlink = CalcBodyToBaseCoordinates(*rbdlModelPtr_, Q_, 
  //   rbdlModelPtr_->GetBodyId("baselink-yawlink"), Vector3d(0., 0., 0.), true);
  // CHECK_THAT (Vector3d(0.49987, -0.927, -0.598924), 
  //   AllCloseVector(p_w_yawlink, TEST_PREC, TEST_PREC));

  // Vector3d p_w_pitchbacklink = CalcBodyToBaseCoordinates(*rbdlModelPtr_, Q_, 
  //   rbdlModelPtr_->GetBodyId("yawlink-pitchbacklink"), Vector3d(0., 0., 0.), true);
  // // CHECK_THAT (Vector3d(0.5, -0.764604, -0.605786), 
  // CHECK_THAT (Vector3d(0.5, -0.764493, -0.608647), 
  //   AllCloseVector(p_w_pitchbacklink, TEST_PREC, TEST_PREC));

  // // Vector3d p_w_pitchbottomlink = CalcBodyToBaseCoordinates(*rbdlModelPtr_, Q_, 
  // //   rbdlModelPtr_->GetBodyId("pitchbacklink-pitchbottomlink"), Vector3d(0., 0., 0.), true);
  // // CHECK_THAT (Vector3d(0.5, -0.528961, -0.368644), 
  // //   AllCloseVector(p_w_pitchbottomlink, TEST_PREC, TEST_PREC));

  // Vector3d p_w_pitchfrontlink = CalcBodyToBaseCoordinates(*rbdlModelPtr_, Q_, 
  //   rbdlModelPtr_->GetBodyId("yawlink-pitchfrontlink"), Vector3d(0., 0., 0.), true);
  // CHECK_THAT (Vector3d(0.5, -0.728002, -0.599003), 
  //   AllCloseVector(p_w_pitchfrontlink, TEST_PREC, TEST_PREC));


  UpdateKinematicsCustom (*rbdlModelPtr_, &Q_, NULL, NULL);

  Matrix3d r_w_baselink = 
    CalcBodyWorldOrientation(*rbdlModelPtr_, Q_, rbdlModelPtr_->GetBodyId("world-baselink"), true);
  // Vector3d p_w_baselink = 
  //   CalcBodyToBaseCoordinates(*rbdlModelPtr_, Q_, rbdlModelPtr_->GetBodyId("world-baselink"), 
  //     Vector3d (0., 0., 0.), true);

  Matrix3d r_w_yawlink = 
    CalcBodyWorldOrientation(*rbdlModelPtr_, Q_, rbdlModelPtr_->GetBodyId("baselink-yawlink"), true);
  Vector3d p_w_yawlink = 
    CalcBodyToBaseCoordinates(*rbdlModelPtr_, Q_, rbdlModelPtr_->GetBodyId("baselink-yawlink"), 
      Vector3d (0., 0., 0.), true);

  Matrix3d r_w_pitchbacklink = 
    CalcBodyWorldOrientation(*rbdlModelPtr_, Q_, rbdlModelPtr_->GetBodyId("yawlink-pitchbacklink"), 
      true);
  // Vector3d p_w_pitchbacklink = 
  //   CalcBodyToBaseCoordinates(*rbdlModelPtr_, Q_, rbdlModelPtr_->GetBodyId("yawlink-pitchbacklink"), 
  //     Vector3d (0., 0., 0.), true);

  Matrix3d r_w_pitchfrontlink = 
    CalcBodyWorldOrientation(*rbdlModelPtr_, Q_, rbdlModelPtr_->GetBodyId("yawlink-pitchfrontlink"), 
      true);
  Vector3d p_w_pitchfrontlink = 
    CalcBodyToBaseCoordinates(*rbdlModelPtr_, Q_, rbdlModelPtr_->GetBodyId("yawlink-pitchfrontlink"), 
      Vector3d (0., 0., 0.), true);

	Matrix3d r_w_pitchtoplink = 
    CalcBodyWorldOrientation(*rbdlModelPtr_, Q_, rbdlModelPtr_->GetBodyId("pitchfrontlink-pitchtoplink"), 
      true);


  Matrix3d r_w_maininsertionlink; r_w_maininsertionlink.setIdentity();
    // CalcBodyWorldOrientation(*rbdlModelPtr_, Q_, 
		// 	rbdlModelPtr_->GetBodyId("pitchendlink-maininsertionlink"), true);
  Vector3d p_w_maininsertionlink = 
    CalcBodyToBaseCoordinates(*rbdlModelPtr_, Q_, 
			rbdlModelPtr_->GetBodyId("pitchendlink-maininsertionlink"), Vector3d (0., 0., 0.), true);

	Matrix3d r_w_toollink = 
    CalcBodyWorldOrientation(*rbdlModelPtr_, Q_, 
			rbdlModelPtr_->GetBodyId("maininsertionlink-toollink"), true);

	Vector3d p_w_baselink = Vector3d(0.4999,     -0.3901,      -0.599);
	Vector3d p_w_pitchbacklink = Vector3d(0.499993,   -0.764493,   -0.608647);
  Vector3d p_w_pitchbottomlink = Vector3d (0.499487, -0.528961, -0.368644);
  // Vector3d p_w_pitchendlink = Vector3d (0.500076, -0.188766, -0.371571);
	Vector3d p_w_pitchtoplink = Vector3d(0.499382,     -0.4989,   -0.345218);
	Vector3d p_w_toollink = Vector3d(0.500195,  -0.0656258,   -0.421017);

  InverseKinematicsConstraintSet cs;
  cs.AddFullConstraint (
    rbdlModelPtr_->GetBodyId("world-baselink"),
    Vector3d (0., 0., 0.),
    p_w_baselink,
    r_w_baselink
  );
  cs.AddFullConstraint (
    rbdlModelPtr_->GetBodyId("baselink-yawlink"),
    Vector3d (0., 0., 0.),
    p_w_yawlink,
    r_w_yawlink
  );
  cs.AddFullConstraint (
    rbdlModelPtr_->GetBodyId("yawlink-pitchbacklink"),
    Vector3d (0., 0., 0.),
    p_w_pitchbacklink,
    r_w_pitchbacklink
  );

	cs.AddFullConstraint (
    rbdlModelPtr_->GetBodyId("yawlink-pitchfrontlink"),
    Vector3d (0., 0., 0.),
    p_w_pitchfrontlink,
    r_w_pitchfrontlink
  );
	cs.AddFullConstraint (
    rbdlModelPtr_->GetBodyId("pitchfrontlink-pitchtoplink"),
    Vector3d (0., 0., 0.),
    p_w_pitchtoplink,
    r_w_pitchtoplink
  );

	cs.AddFullConstraint (
    rbdlModelPtr_->GetBodyId("pitchendlink-maininsertionlink"),
    Vector3d (0., 0., 0.),
    p_w_maininsertionlink,
    r_w_maininsertionlink
  );

	cs.AddFullConstraint (
    rbdlModelPtr_->GetBodyId("maininsertionlink-toollink"),
    Vector3d (0., 0., 0.),
    p_w_toollink,
    r_w_toollink
  );
  cs.AddPointConstraint (rbdlModelPtr_->GetBodyId("pitchbacklink-pitchbottomlink"), 
    Vector3d (0., 0., 0.), p_w_pitchbottomlink);
  cs.AddPointConstraint (rbdlModelPtr_->GetBodyId("pitchfrontlink-pitchtoplink"), 
    Vector3d (0., 0., 0.), p_w_pitchtoplink);
	cs.AddPointConstraint (rbdlModelPtr_->GetBodyId("maininsertionlink-toollink"), 
    Vector3d (0., 0., 0.), p_w_toollink);

  cs.step_tol = 1e-2;

  // q.setZero();

  VectorNd Qres (Q_);

  bool result = InverseKinematics (*rbdlModelPtr_, Q_, cs, Qres);

  CHECK (result);
  CHECK_THAT ( 0.0, IsClose(cs.error_norm, cs.step_tol, cs.step_tol));
  std::cout << "Qres" << std::endl << Qres << std::endl;

  UpdateKinematicsCustom (*rbdlModelPtr_, &Qres, NULL, NULL);

  p_w_baselink = CalcBodyToBaseCoordinates(*rbdlModelPtr_, Qres, 
    rbdlModelPtr_->GetBodyId("world-baselink"), Vector3d(0., 0., 0.), true);
  CHECK_THAT (Vector3d(0.4999,     -0.3901,      -0.599), 
    AllCloseVector(p_w_baselink, TEST_PREC, TEST_PREC));

  p_w_pitchbacklink = CalcBodyToBaseCoordinates(*rbdlModelPtr_, Qres, 
    rbdlModelPtr_->GetBodyId("yawlink-pitchbacklink"), Vector3d(0., 0., 0.), true);
  CHECK_THAT (Vector3d(0.499975,   -0.764724,   -0.608522), 
    AllCloseVector(p_w_pitchbacklink, TEST_PREC, TEST_PREC));

  p_w_pitchtoplink = CalcBodyToBaseCoordinates(*rbdlModelPtr_, Qres, 
    rbdlModelPtr_->GetBodyId("pitchfrontlink-pitchtoplink"), Vector3d(0., 0., 0.), true);
  CHECK_THAT (Vector3d(0.499414,   -0.498396,    -0.34559), 
    AllCloseVector(p_w_pitchtoplink, TEST_PREC, TEST_PREC));

	p_w_toollink = CalcBodyToBaseCoordinates(*rbdlModelPtr_, Qres, 
    rbdlModelPtr_->GetBodyId("maininsertionlink-toollink"), Vector3d(0., 0., 0.), true);
  CHECK_THAT (Vector3d(0.500152,  -0.0658301,   -0.422452), 
    AllCloseVector(p_w_toollink, TEST_PREC, TEST_PREC));
}
*/