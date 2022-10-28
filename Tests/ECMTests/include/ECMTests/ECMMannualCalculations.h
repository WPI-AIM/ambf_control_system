#include "rbdl/rbdl.h"
#include "Tests/rbdl_tests.h"

const double TEST_PREC = 1.0e-3;
const double TEST_LAX = 1.0e-5;

struct ECM 
{
  ECM()
  {
    Vector3d vector3d_zero = Vector3d::Zero();
    //0---------------------------------------------------------------------//
    world_baselinkST.E.setIdentity();
    world_baselinkST.r = Vector3d(0.5, -0.4, -0.6);
    //1--------------------------------------------------------------------//
    Matrix3d baselink_yawlinkRot = 
	  Eigen::Matrix3d(Eigen::Quaterniond::FromTwoVectors(baselink_yawlinkPA, baselink_yawlinkCA));
	  Eigen::Affine3d baselink_yawlinkRotOffset(Eigen::AngleAxisd(baselink_yawlinkOffsetQ, -Vector3d::UnitZ()));
		
	  // baselink_yawlinkST.E = baselink_yawlinkRot.transpose() * baselink_yawlinkRotOffset.rotation();
    baselink_yawlinkST.E = Matrix3d(-1,  0,  0, 0,  0, 1, 0,  1, 0);
	  
    baselink_yawlinkST.r = 
		baselink_yawlinkPP - (baselink_yawlinkRot.transpose() * baselink_yawlinkCP);
    //--------------------------------------------------------------------//
    Matrix3d yawlink_pitchbacklinkRot = 
    Eigen::Matrix3d(Eigen::Quaterniond::FromTwoVectors(yawlink_pitchbacklinkPA, yawlink_pitchbacklinkCA));
    Eigen::Affine3d yawlink_pitchbacklinkRotOffset(
      Eigen::AngleAxisd(yawlink_pitchbacklinkOffsetQ, -Vector3d::UnitZ()));
    
    // yawlink_pitchbacklinkST.E = yawlink_pitchbacklinkRot.transpose() * 
    // yawlink_pitchbacklinkRotOffset.rotation();
    yawlink_pitchbacklinkST.E = Matrix3d( 0,  0,  1, 0, -1, 0, 1,  0, 0);
    
    yawlink_pitchbacklinkST.r = 
      yawlink_pitchbacklinkPP - (yawlink_pitchbacklinkRot.transpose() * yawlink_pitchbacklinkCP);
    //1--------------------------------------------------------------------//
    Matrix3d pitchbacklink_pitchbottomlinkRot = 
    Eigen::Matrix3d(
      Eigen::Quaterniond::FromTwoVectors(pitchbacklink_pitchbottomlinkPA, pitchbacklink_pitchbottomlinkCA));
    Eigen::Affine3d pitchbacklink_pitchbottomlinkRotOffset(
      Eigen::AngleAxisd(pitchbacklink_pitchbottomlinkOffsetQ, -Vector3d::UnitZ()));
      
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
    
    yawlink_pitchfrontlinkST.E = yawlink_pitchfrontlinkRot.transpose() * yawlink_pitchfrontlinkRotOffset.rotation();
    yawlink_pitchfrontlinkST.r = 
      yawlink_pitchfrontlinkPP - (yawlink_pitchfrontlinkRot.transpose() * yawlink_pitchfrontlinkCP);
    //1--------------------------------------------------------------------//
    Matrix3d pitchfrontlink_pitchbottomlinkRot = 
    Eigen::Matrix3d(Eigen::Quaterniond::FromTwoVectors(pitchfrontlink_pitchbottomlinkPA, pitchfrontlink_pitchbottomlinkCA));
    Eigen::Affine3d pitchfrontlink_pitchbottomlinkRotOffset(
      Eigen::AngleAxisd(pitchfrontlink_pitchbottomlinkOffsetQ, -Vector3d::UnitZ()));
    
    pitchfrontlink_pitchbottomlinkST.E = pitchfrontlink_pitchbottomlinkRot.transpose() * pitchfrontlink_pitchbottomlinkRotOffset.rotation();
    pitchfrontlink_pitchbottomlinkST.r = 
      pitchfrontlink_pitchbottomlinkPP - (pitchfrontlink_pitchbottomlinkRot.transpose() * pitchfrontlink_pitchbottomlinkCP);
    //1--------------------------------------------------------------------//
    Matrix3d pitchfrontlink_pitchtoplinkRot = 
    Eigen::Matrix3d(Eigen::Quaterniond::FromTwoVectors(pitchfrontlink_pitchtoplinkPA, pitchfrontlink_pitchtoplinkCA));
    Eigen::Affine3d pitchfrontlink_pitchtoplinkRotOffset(
      Eigen::AngleAxisd(pitchfrontlink_pitchtoplinkOffsetQ, -Vector3d::UnitZ()));
    
    pitchfrontlink_pitchtoplinkST.E = pitchfrontlink_pitchtoplinkRot.transpose() * pitchfrontlink_pitchtoplinkRotOffset.rotation();
    pitchfrontlink_pitchtoplinkST.r = 
      pitchfrontlink_pitchtoplinkPP - (pitchfrontlink_pitchtoplinkRot.transpose() * pitchfrontlink_pitchtoplinkCP);
    //1--------------------------------------------------------------------//
    Matrix3d pitchtoplink_pitchendlinkRot = 
    Eigen::Matrix3d(Eigen::Quaterniond::FromTwoVectors(pitchtoplink_pitchendlinkPA, pitchtoplink_pitchendlinkCA));
    Eigen::Affine3d pitchtoplink_pitchendlinkRotOffset(
      Eigen::AngleAxisd(pitchtoplink_pitchendlinkOffsetQ, -Vector3d::UnitZ()));
    
    pitchtoplink_pitchendlinkST.E = pitchtoplink_pitchendlinkRot.transpose() * pitchtoplink_pitchendlinkRotOffset.rotation();
    pitchtoplink_pitchendlinkST.r = 
      pitchtoplink_pitchendlinkPP - (pitchtoplink_pitchendlinkRot.transpose() * pitchtoplink_pitchendlinkCP);
    //1--------------------------------------------------------------------//
      Matrix3d pitchendlink_maininsertionlinkRot = 
    Eigen::Matrix3d(
      Eigen::Quaterniond::FromTwoVectors(pitchendlink_maininsertionlinkPA, pitchendlink_maininsertionlinkCA));
    Eigen::Affine3d pitchendlink_maininsertionlinkRotOffset(
      Eigen::AngleAxisd(pitchendlink_maininsertionlinkOffsetQ, -Vector3d::UnitZ()));
    // pitchendlink_maininsertionlinkST.E =
    // pitchendlink_maininsertionlinkRot.transpose() * 
    // pitchendlink_maininsertionlinkRotOffset.rotation(); 
    pitchendlink_maininsertionlinkST.E = Matrix3d( 0, -1,  0, 1,  0, 0, 0,  0, 1);

    pitchendlink_maininsertionlinkST.r = 
      pitchendlink_maininsertionlinkPP - 
      (pitchendlink_maininsertionlinkRot.transpose() * pitchendlink_maininsertionlinkCP);
    //--------------------------------------------------------------------//
    Matrix3d maininsertionlink_toollinkRot = 
    Eigen::Matrix3d(
      Eigen::Quaterniond::FromTwoVectors(maininsertionlink_toollinkPA, maininsertionlink_toollinkCA));
    Eigen::Affine3d maininsertionlink_toollinkRotOffset(
      Eigen::AngleAxisd(maininsertionlink_toollinkOffsetQ, -Vector3d::UnitZ()));
      
    // maininsertionlink_toollinkST.E = maininsertionlink_toollinkRot.transpose() * 
    //   maininsertionlink_toollinkRotOffset.rotation();
    maininsertionlink_toollinkST.E = Matrix3d( 0,  0, -1, 1,  0, 0, 0, -1, 0);

    maininsertionlink_toollinkST.r = 
      maininsertionlink_toollinkPP - 
      (maininsertionlink_toollinkRot.transpose() * maininsertionlink_toollinkCP);

    // std::cout << "world_baselinkST" 			 					<< std::endl << world_baselinkST 								<< std::endl;
    // std::cout << "baselink_yawlinkST" 		 					<< std::endl << baselink_yawlinkST 							<< std::endl;
    // std::cout << "yawlink_pitchbacklinkST" 					<< std::endl << yawlink_pitchbacklinkST 				<< std::endl;
    // std::cout << "pitchbacklink_pitchbottomlinkST" 	<< std::endl << pitchbacklink_pitchbottomlinkST << std::endl;
    // std::cout << "pitchbottomlink_pitchendlinkST" 	<< std::endl << pitchbottomlink_pitchendlinkST 	<< std::endl;
    // std::cout << "pitchendlink_maininsertionlinkST" << std::endl << pitchendlink_maininsertionlinkST << std::endl;
    // std::cout << "maininsertionlink_toollinkST" 		<< std::endl << maininsertionlink_toollinkST 		<< std::endl;
    //--------------------------------------------------------------------//
    world_yawlinkST           = world_baselinkST * baselink_yawlinkST;
    world_pitchbacklinkST     = world_yawlinkST * yawlink_pitchbacklinkST;
    world_pitchbottomlinkST   = world_pitchbacklinkST * pitchbacklink_pitchbottomlinkST;
    world_pitchendlinkST      = world_pitchbottomlinkST * pitchbottomlink_pitchendlinkST;

    world_pitchfrontlinkST    = world_yawlinkST * yawlink_pitchfrontlinkST;
    // world_pitchbottomlink2ST = world_pitchfrontlinkST * pitchfrontlink_pitchbottomlinkST;
    world_pitchtoplinkST      = world_pitchfrontlinkST * pitchfrontlink_pitchtoplinkST;
    world_pitchendlinkST      = world_pitchtoplinkST * pitchtoplink_pitchendlinkST;

    world_maininsertionlinkST = world_pitchendlinkST * pitchendlink_maininsertionlinkST;
    world_toollinkST          = world_maininsertionlinkST * maininsertionlink_toollinkST;

    world_pitchendlinkST.E      = Matrix3d( 0,  0, -1,  1,  0, 0,  0, -1, 0);
    world_pitchfrontlinkST.E    = Matrix3d( 0,  0, -1,  1,  0, 0,  0, -1, 0);
    // world_pitchbottomlink2ST.E  = Matrix3d( 1,  0,  0,  0,  0, 1,  0, -1, 0);
    world_pitchtoplinkST.E      = Matrix3d( 0,  0, -1,  1,  0, 0,  0, -1, 0);
    world_maininsertionlinkST.E = Matrix3d( 0,  0, -1,  0, -1, 0, -1,  0, 0);
    world_toollinkST.E          = Matrix3d( 0,  1,  0, -1,  0, 0,  0,  0, 1);


    // std::cout << "world_yawlinkST"          << std::endl << world_yawlinkST            << std::endl;
    // std::cout << "world_pitchbacklinkST"    << std::endl << world_pitchbacklinkST      << std::endl;
    // std::cout << "world_pitchbottomlinkST"  << std::endl << world_pitchbottomlinkST    << std::endl;
    // std::cout << "world_pitchendlinkST"     << std::endl << world_pitchendlinkST       << std::endl;

    // std::cout << "world_pitchfrontlinkST"    << std::endl << world_pitchfrontlinkST    << std::endl;
    // // std::cout << "world_pitchbottomlink2ST" << std::endl << world_pitchbottomlink2ST << std::endl;
    // std::cout << "world_pitchtoplinkST"      << std::endl << world_pitchtoplinkST      << std::endl;
    // std::cout << "world_pitchendlinkST"      << std::endl << world_pitchendlinkST      << std::endl;
    
    // std::cout << "world_maininsertionlinkST" << std::endl << world_maininsertionlinkST << std::endl;
    // std::cout << "world_toollinkST"          << std::endl << world_toollinkST          << std::endl;
    //--------------------------------------------------------------------//
    const Vector3d p_baselink_yawlink_world 
      = world_baselinkST.E * baselink_yawlinkST.r;
    const Vector3d p_yawlink_pitchbacklink_world 	
      = world_yawlinkST.E * yawlink_pitchbacklinkST.r;
    const Vector3d p_pitchbacklink_pitchbottomlink_world  
      = world_pitchbacklinkST.E * pitchbacklink_pitchbottomlinkST.r;
    const Vector3d p_pitchbottomlink_pitchendlink_world 
      = world_pitchbottomlinkST.E * pitchbottomlink_pitchendlinkST.r;

    const Vector3d p_yawlink_pitchfrontlink_world 
      = world_yawlinkST.E * yawlink_pitchfrontlinkST.r;
    const Vector3d p_pitchfrontlink_pitchbottomlink_world 
      = world_pitchfrontlinkST.E * pitchfrontlink_pitchbottomlinkST.r;
    const Vector3d p_pitchfrontlink_pitchtoplink_world 
      = world_pitchfrontlinkST.E * pitchfrontlink_pitchtoplinkST.r;
    const Vector3d p_pitchtoplink_pitchendlink_world 
      = world_pitchtoplinkST.E * pitchtoplink_pitchendlinkST.r;

    const Vector3d p_pitchendlink_maininsertionlink_world = world_pitchendlinkST.E 		  * pitchendlink_maininsertionlinkST.r;
    const Vector3d p_maininsertionlink_toollink_world     = world_maininsertionlinkST.E * maininsertionlink_toollinkST.r;
    //--------------------------------------------------------------------//
    const Vector3d baselink_yawlinkJAxis = world_baselinkST.E * baselink_yawlinkPA;
    
    const Vector3d yawlink_pitchbacklinkJAxis = world_yawlinkST.E * yawlink_pitchbacklinkPA;
    const Vector3d pitchbacklink_pitchbottomlinkJAxis = world_pitchbacklinkST.E * pitchbacklink_pitchbottomlinkPA;
    const Vector3d pitchbottomlink_pitchendlinkJAxis = world_pitchbottomlinkST.E * pitchbottomlink_pitchendlinkPA;
    
    const Vector3d yawlink_pitchfrontlinkJAxis = world_yawlinkST.E * yawlink_pitchfrontlinkPA;
    const Vector3d pitchfrontlink_pitchtoplinkJAxis = world_pitchfrontlinkST.E * pitchfrontlink_pitchtoplinkPA;
    const Vector3d pitchtoplink_pitchendlinkJAxis = world_pitchtoplinkST.E * pitchtoplink_pitchendlinkPA;
    
    const Vector3d pitchfrontlink_pitchbottomlinkJAxis = world_pitchfrontlinkST.E * pitchfrontlink_pitchbottomlinkPA;
    const Vector3d pitchendlink_maininsertionlinkJAxis = world_pitchendlinkST.E * pitchendlink_maininsertionlinkPA;
    const Vector3d maininsertionlink_toollinkJAxis = world_maininsertionlinkST.E * maininsertionlink_toollinkPA;
    //1--------------------------------------------------------------------//

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

  }
  
  ~ECM()
  {
  }

  SpatialTransform world_baselinkST, world_yawlinkST, world_pitchfrontlinkST, world_pitchbacklinkST, 
	world_pitchbottomlinkST, world_pitchendlinkST, world_maininsertionlinkST, world_pitchtoplinkST,
	world_toollinkST;

  SpatialTransform baselink_yawlinkST, yawlink_pitchbacklinkST, pitchbacklink_pitchbottomlinkST,
  pitchbottomlink_pitchendlinkST, yawlink_pitchfrontlinkST, pitchfrontlink_pitchbottomlinkST,
  pitchfrontlink_pitchtoplinkST, pitchtoplink_pitchendlinkST, pitchendlink_maininsertionlinkST,
  maininsertionlink_toollinkST;

	const Vector3d baselink_yawlinkPA 							= { -0.0000, -1.0000, 00.0000 };
	const Vector3d baselink_yawlinkCA 							= { 00.0000, 00.0000, -1.0000 };
	const Vector3d baselink_yawlinkPP 							= { 00.0000, 00.0000, 00.0000 };
	// const Vector3d baselink_yawlinkCP 							= { 00.0001, 00.0000, 00.5369 };
  const Vector3d baselink_yawlinkCP 							= { 00.0000, 00.0000, 00.5369 };

	//-----------------------------------------------------//
	const Vector3d yawlink_pitchbacklinkPA 					= { 1.0,     0.0,    0.0 };
	const Vector3d yawlink_pitchbacklinkCA 					= { 0.0,     0.0,    1.0 };
	// const Vector3d yawlink_pitchbacklinkPP 					= { 0.0, -0.0098, 0.1624 };
  const Vector3d yawlink_pitchbacklinkPP 					= { 0.0, -0.0000, 0.1624 };
	const Vector3d yawlink_pitchbacklinkCP 					= { 0.0,     0.0,    0.0 };

	const Vector3d pitchbacklink_pitchbottomlinkPA 	= {     0.0,     0.0,     1.0 };
	const Vector3d pitchbacklink_pitchbottomlinkCA 	= {     0.0,     0.0,     1.0 };
	const Vector3d pitchbacklink_pitchbottomlinkPP 	= { -0.1028, -0.2867,     0.0 };
	// const Vector3d pitchbacklink_pitchbottomlinkCP 	= { -0.0364,  0.0098, -0.0005 };
  const Vector3d pitchbacklink_pitchbottomlinkCP 	= { -0.0364,  0.0000, -0.0000 };

	const Vector3d pitchbottomlink_pitchendlinkPA 	= {    0.0,     0.0,     1.0 };
	const Vector3d pitchbottomlink_pitchendlinkCA 	= {    0.0,     0.0,     1.0 };
	// const Vector3d pitchbottomlink_pitchendlinkPP 	= { 0.3401, -0.0001, -0.0005 };
	// const Vector3d pitchbottomlink_pitchendlinkCP 	= {    0.0,     0.0,  0.0001 };
  const Vector3d pitchbottomlink_pitchendlinkPP 	= { 0.3401, -0.0000, -0.0000 };
	const Vector3d pitchbottomlink_pitchendlinkCP 	= {    0.0,     0.0,  0.0000 };
	//-----------------------------------------------------//
	const Vector3d yawlink_pitchfrontlinkPA 					= { 1.0,     0.0,    0.0 };
	const Vector3d yawlink_pitchfrontlinkCA 					= { 0.0,     0.0,    1.0 };
	// const Vector3d yawlink_pitchfrontlinkPP 					= { 0.0, 		 0.0,  0.199 };
  const Vector3d yawlink_pitchfrontlinkPP 					= { 0.0, 		 0.0,    0.2 };
	const Vector3d yawlink_pitchfrontlinkCP 					= { 0.0,     0.0,    0.0 };

	const Vector3d pitchfrontlink_pitchbottomlinkPA 	= { 	 	0.0,     0.0,     1.0 };
	const Vector3d pitchfrontlink_pitchbottomlinkCA 	= { 		0.0,     0.0,     1.0 };
	const Vector3d pitchfrontlink_pitchbottomlinkPP 	= { -0.1031, -0.2868,  	  0.0 };
  // const Vector3d pitchfrontlink_pitchbottomlinkCP 	= { -0.0001, -0.0001, -0.0005 };
	const Vector3d pitchfrontlink_pitchbottomlinkCP 	= { -0.0000, -0.0000, -0.0000 };

	const Vector3d pitchfrontlink_pitchtoplinkPA 	= { 	 	0.0,     0.0,     1.0 };
	const Vector3d pitchfrontlink_pitchtoplinkCA 	= { 		0.0,     0.0,     1.0 };
	const Vector3d pitchfrontlink_pitchtoplinkPP 	= { -0.1084, -0.3242,  	  0.0 };
	// const Vector3d pitchfrontlink_pitchtoplinkCP 	= { -0.0000, -0.0000, -0.0006 };
  const Vector3d pitchfrontlink_pitchtoplinkCP 	= { -0.0000, -0.0000, -0.0000 };

	const Vector3d pitchtoplink_pitchendlinkPA 	= {     0.0,     0.0,     1.0 };
	const Vector3d pitchtoplink_pitchendlinkCA 	= {     0.0,     0.0,     1.0 };
	// const Vector3d pitchtoplink_pitchendlinkPP 	= {  0.3404, -0.0002, -0.0006 };
	// const Vector3d pitchtoplink_pitchendlinkCP 	= { -0.0051, -0.0376,  0.0001 };
  const Vector3d pitchtoplink_pitchendlinkPP 	= {  0.3404, -0.0000, -0.000 };
	const Vector3d pitchtoplink_pitchendlinkCP 	= { -0.0051, -0.0376,  0.000 };
	//-----------------------------------------------------//
	const Vector3d pitchendlink_maininsertionlinkPA = {     0.0,     1.0,    0.0 };
	const Vector3d pitchendlink_maininsertionlinkCA = {     1.0,     0.0,    0.0 };
	// const Vector3d pitchendlink_maininsertionlinkPP = {  0.1031, -0.0961, 0.0001 };
	// const Vector3d pitchendlink_maininsertionlinkCP = { -0.0108,  -0.062, 0.00002 };
  const Vector3d pitchendlink_maininsertionlinkPP = {  0.1031, -0.0961, 0.0000 };
	const Vector3d pitchendlink_maininsertionlinkCP = { -0.0108,  -0.062, 0.0000 };

	const Vector3d maininsertionlink_toollinkPA 	  = {     1.0,     0.0,    0.0 };
	const Vector3d maininsertionlink_toollinkCA 	  = {     0.0,     0.0,   -1.0 };
	const Vector3d maininsertionlink_toollinkPP 	  = { -0.0108,  -0.062,    0.0 };
	// const Vector3d maininsertionlink_toollinkCP 	  = { -0.0001, -0.0002, 0.0118 };
  const Vector3d maininsertionlink_toollinkCP 	  = { -0.0000, -0.0000, 0.0118 };
  
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

};
