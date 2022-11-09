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
    W_world_baselinkST.E.setIdentity();
    W_world_baselinkST.r = Vector3d(0.5, -0.4, -0.6);
    //1--------------------------------------------------------------------//		
    {
      baselink_yawlinkRot = 
      Eigen::Matrix3d(Eigen::Quaterniond::FromTwoVectors
        (baselink_yawlinkPA, baselink_yawlinkCA));
      Eigen::Affine3d rotOffset(
        Eigen::AngleAxisd(baselink_yawlinkOffsetQ, baselink_yawlinkCA));

      baselink_yawlinkRotOffset = rotOffset.rotation();

      // B_baselink_yawlinkST.E = Matrix3d( 0,  0, -1, 1,  0, 0, 0, -1, 0);
      B_baselink_yawlinkST.E = 
        (baselink_yawlinkRotOffset * baselink_yawlinkRot).transpose();
      B_baselink_yawlinkST.r = 
        baselink_yawlinkPP - 
        (baselink_yawlinkRot.transpose() * baselink_yawlinkCP);
    }

    //--------------------------------------------------------------------//
    // yawlink_pitchbacklinkRot = 
    // Eigen::Matrix3d(Eigen::Quaterniond::FromTwoVectors(yawlink_pitchbacklinkPA, yawlink_pitchbacklinkCA));
    // Eigen::Affine3d yawlink_pitchbacklinkRotOffset(
    //   Eigen::AngleAxisd(yawlink_pitchbacklinkOffsetQ, yawlink_pitchbacklinkCA));
    
    // // yawlink_pitchbacklinkST.E = yawlink_pitchbacklinkRot.transpose() * 
    // // yawlink_pitchbacklinkRotOffset.rotation();
    // // B_yawlink_pitchbacklinkST.E = Matrix3d( 0,  0,  1, 0, -1, 0, 1,  0, 0);
    // B_yawlink_pitchbacklinkST.E = 
    //   (yawlink_pitchbacklinkRotOffset.rotation() * yawlink_pitchbacklinkRot).transpose();
    
    // B_yawlink_pitchbacklinkST.r = 
    //   yawlink_pitchbacklinkPP - (yawlink_pitchbacklinkRot.transpose() * yawlink_pitchbacklinkCP);
    {
      yawlink_pitchbacklinkRot = 
      Eigen::Matrix3d(Eigen::Quaterniond::FromTwoVectors
        (yawlink_pitchbacklinkPA, yawlink_pitchbacklinkCA));
      Eigen::Affine3d rotOffset(
        Eigen::AngleAxisd(yawlink_pitchbacklinkOffsetQ, yawlink_pitchbacklinkCA));
      yawlink_pitchbacklinkRotOffset = rotOffset.rotation();

      // B_yawlink_pitchbacklinkST.E = Matrix3d( 0,  0, -1, 1,  0, 0, 0, -1, 0);
      B_yawlink_pitchbacklinkST.E = 
        (yawlink_pitchbacklinkRotOffset * yawlink_pitchbacklinkRot).transpose();
      B_yawlink_pitchbacklinkST.r = 
        yawlink_pitchbacklinkPP - 
        (yawlink_pitchbacklinkRot.transpose() * yawlink_pitchbacklinkCP);
    }
    

    //1--------------------------------------------------------------------//
    // pitchbacklink_pitchbottomlinkRot = 
    // Eigen::Matrix3d(
    //   Eigen::Quaterniond::FromTwoVectors(pitchbacklink_pitchbottomlinkPA, pitchbacklink_pitchbottomlinkCA));
    // Eigen::Affine3d pitchbacklink_pitchbottomlinkRotOffset(
    //   Eigen::AngleAxisd(pitchbacklink_pitchbottomlinkOffsetQ, pitchbacklink_pitchbottomlinkCA));
      
    // // B_pitchbacklink_pitchbottomlinkST.E = 
    // //   pitchbacklink_pitchbottomlinkRot.transpose() * pitchbacklink_pitchbottomlinkRotOffset.rotation();
    // B_pitchbacklink_pitchbottomlinkST.E = 
    //   (pitchbacklink_pitchbottomlinkRotOffset.rotation() * pitchbacklink_pitchbottomlinkRot).transpose();
 
    // B_pitchbacklink_pitchbottomlinkST.r = 
    //   pitchbacklink_pitchbottomlinkPP - 
    //   (pitchbacklink_pitchbottomlinkRot.transpose() * pitchbacklink_pitchbottomlinkCP);
    {
      pitchbacklink_pitchbottomlinkRot = 
      Eigen::Matrix3d(Eigen::Quaterniond::FromTwoVectors
        (pitchbacklink_pitchbottomlinkPA, pitchbacklink_pitchbottomlinkCA));
      Eigen::Affine3d rotOffset(
        Eigen::AngleAxisd(pitchbacklink_pitchbottomlinkOffsetQ, pitchbacklink_pitchbottomlinkCA));
      pitchbacklink_pitchbottomlinkRotOffset = rotOffset.rotation();

      // B_pitchbacklink_pitchbottomlinkST.E = Matrix3d( 0,  0, -1, 1,  0, 0, 0, -1, 0);
      B_pitchbacklink_pitchbottomlinkST.E = 
        (pitchbacklink_pitchbottomlinkRotOffset * pitchbacklink_pitchbottomlinkRot).transpose();
      B_pitchbacklink_pitchbottomlinkST.r = 
        pitchbacklink_pitchbottomlinkPP - 
        (pitchbacklink_pitchbottomlinkRot.transpose() * pitchbacklink_pitchbottomlinkCP);
    }

    //--------------------------------------------------------------------//
    // pitchbottomlink_pitchendlinkRot = 
    // Eigen::Matrix3d(
    //   Eigen::Quaterniond::FromTwoVectors(pitchbottomlink_pitchendlinkPA, pitchbottomlink_pitchendlinkCA));
    // Eigen::Affine3d pitchbottomlink_pitchendlinkRotOffset(
    //   Eigen::AngleAxisd(pitchbottomlink_pitchendlinkOffsetQ, pitchbottomlink_pitchendlinkCA));
      
    // // B_pitchbottomlink_pitchendlinkST.E = 
    // //   pitchbottomlink_pitchendlinkRot.transpose() * pitchbottomlink_pitchendlinkRotOffset.rotation();
    // B_pitchbottomlink_pitchendlinkST.E = 
    //   (pitchbottomlink_pitchendlinkRotOffset.rotation() * pitchbottomlink_pitchendlinkRot).transpose();

    // B_pitchbottomlink_pitchendlinkST.r = 
    //   pitchbottomlink_pitchendlinkPP - 
    //   (pitchbottomlink_pitchendlinkRot.transpose() * pitchbottomlink_pitchendlinkCP);
    {
      pitchbottomlink_pitchendlinkRot = 
      Eigen::Matrix3d(Eigen::Quaterniond::FromTwoVectors
        (pitchbottomlink_pitchendlinkPA, pitchbottomlink_pitchendlinkCA));
      Eigen::Affine3d rotOffset(
        Eigen::AngleAxisd(pitchbottomlink_pitchendlinkOffsetQ, pitchbottomlink_pitchendlinkCA));
      pitchbottomlink_pitchendlinkRotOffset = rotOffset.rotation();

      // B_pitchbottomlink_pitchendlinkST.E = Matrix3d( 0,  0, -1, 1,  0, 0, 0, -1, 0);
      B_pitchbottomlink_pitchendlinkST.E = 
        (pitchbottomlink_pitchendlinkRotOffset * pitchbottomlink_pitchendlinkRot).transpose();
      B_pitchbottomlink_pitchendlinkST.r = 
        pitchbottomlink_pitchendlinkPP - 
        (pitchbottomlink_pitchendlinkRot.transpose() * pitchbottomlink_pitchendlinkCP);
    }
    //--------------------------------------------------------------------//
    // yawlink_pitchfrontlinkRot = 
    // Eigen::Matrix3d(Eigen::Quaterniond::FromTwoVectors(yawlink_pitchfrontlinkPA, yawlink_pitchfrontlinkCA));
    // Eigen::Affine3d yawlink_pitchfrontlinkRotOffset(
    //   Eigen::AngleAxisd(yawlink_pitchfrontlinkOffsetQ, Vector3d::UnitZ()));
    
    // // B_yawlink_pitchfrontlinkST.E = yawlink_pitchfrontlinkRot.transpose() * yawlink_pitchfrontlinkRotOffset.rotation();
    // B_yawlink_pitchfrontlinkST.E = 
    //   (yawlink_pitchfrontlinkRotOffset.rotation() * yawlink_pitchfrontlinkRot).transpose();

    // B_yawlink_pitchfrontlinkST.r = 
    //   yawlink_pitchfrontlinkPP - (yawlink_pitchfrontlinkRot.transpose() * yawlink_pitchfrontlinkCP);
    {
      yawlink_pitchfrontlinkRot = 
      Eigen::Matrix3d(Eigen::Quaterniond::FromTwoVectors
        (yawlink_pitchfrontlinkPA, yawlink_pitchfrontlinkCA));
      Eigen::Affine3d rotOffset(
        Eigen::AngleAxisd(yawlink_pitchfrontlinkOffsetQ, yawlink_pitchfrontlinkCA));
      yawlink_pitchfrontlinkRotOffset = rotOffset.rotation();

      // B_yawlink_pitchfrontlinkST.E = Matrix3d( 0,  0, -1, 1,  0, 0, 0, -1, 0);
      B_yawlink_pitchfrontlinkST.E = 
        (yawlink_pitchfrontlinkRotOffset * yawlink_pitchfrontlinkRot).transpose();
      B_yawlink_pitchfrontlinkST.r = 
        yawlink_pitchfrontlinkPP - 
        (yawlink_pitchfrontlinkRot.transpose() * yawlink_pitchfrontlinkCP);
    }
    //1--------------------------------------------------------------------//
    {
      pitchfrontlink_pitchbottomlinkRot = 
      Eigen::Matrix3d(Eigen::Quaterniond::FromTwoVectors
        (pitchfrontlink_pitchbottomlinkPA, pitchfrontlink_pitchbottomlinkCA));
      Eigen::Affine3d rotOffset(
        Eigen::AngleAxisd(pitchfrontlink_pitchbottomlinkOffsetQ, pitchfrontlink_pitchbottomlinkCA));
      pitchfrontlink_pitchbottomlinkRotOffset = rotOffset.rotation();

      // B_pitchfrontlink_pitchbottomlinkST.E = Matrix3d( 0,  0, -1, 1,  0, 0, 0, -1, 0);
      B_pitchfrontlink_pitchbottomlinkST.E = 
        (pitchfrontlink_pitchbottomlinkRotOffset * pitchfrontlink_pitchbottomlinkRot).transpose();
      B_pitchfrontlink_pitchbottomlinkST.r = 
        pitchfrontlink_pitchbottomlinkPP - 
        (pitchfrontlink_pitchbottomlinkRot.transpose() * pitchfrontlink_pitchbottomlinkCP);
    }
    //1--------------------------------------------------------------------//
    {
      pitchfrontlink_pitchtoplinkRot = 
      Eigen::Matrix3d(Eigen::Quaterniond::FromTwoVectors
        (pitchfrontlink_pitchtoplinkPA, pitchfrontlink_pitchtoplinkCA));
      Eigen::Affine3d rotOffset(
        Eigen::AngleAxisd(pitchfrontlink_pitchtoplinkOffsetQ, pitchfrontlink_pitchtoplinkCA));
      pitchfrontlink_pitchtoplinkRotOffset = rotOffset.rotation();

      // B_pitchfrontlink_pitchtoplinkST.E = Matrix3d( 0,  0, -1, 1,  0, 0, 0, -1, 0);
      B_pitchfrontlink_pitchtoplinkST.E = 
        (pitchfrontlink_pitchtoplinkRotOffset * pitchfrontlink_pitchtoplinkRot).transpose();
      B_pitchfrontlink_pitchtoplinkST.r = 
        pitchfrontlink_pitchtoplinkPP - 
        (pitchfrontlink_pitchtoplinkRot.transpose() * pitchfrontlink_pitchtoplinkCP);
    }
    //1--------------------------------------------------------------------//
    {
      pitchtoplink_pitchendlinkRot = 
      Eigen::Matrix3d(Eigen::Quaterniond::FromTwoVectors
        (pitchtoplink_pitchendlinkPA, pitchtoplink_pitchendlinkCA));
      Eigen::Affine3d rotOffset(
        Eigen::AngleAxisd(pitchtoplink_pitchendlinkOffsetQ, pitchtoplink_pitchendlinkCA));
      pitchtoplink_pitchendlinkRotOffset = rotOffset.rotation();

      // B_pitchtoplink_pitchendlinkST.E = Matrix3d( 0,  0, -1, 1,  0, 0, 0, -1, 0);
      B_pitchtoplink_pitchendlinkST.E = 
        (pitchtoplink_pitchendlinkRotOffset * pitchtoplink_pitchendlinkRot).transpose();
      B_pitchtoplink_pitchendlinkST.r = 
        pitchtoplink_pitchendlinkPP - 
        (pitchtoplink_pitchendlinkRot.transpose() * pitchtoplink_pitchendlinkCP);
    }
    //1--------------------------------------------------------------------//
    {
      pitchendlink_maininsertionlinkRot = 
      Eigen::Matrix3d(Eigen::Quaterniond::FromTwoVectors
        (pitchendlink_maininsertionlinkPA, pitchendlink_maininsertionlinkCA));
      Eigen::Affine3d rotOffset(
        Eigen::AngleAxisd(pitchendlink_maininsertionlinkOffsetQ, pitchendlink_maininsertionlinkCA));
      pitchendlink_maininsertionlinkRotOffset = rotOffset.rotation();

      // B_pitchendlink_maininsertionlinkST.E = Matrix3d( 0,  0, -1, 1,  0, 0, 0, -1, 0);
      B_pitchendlink_maininsertionlinkST.E = 
        (pitchendlink_maininsertionlinkRotOffset * pitchendlink_maininsertionlinkRot).transpose();
      B_pitchendlink_maininsertionlinkST.r = 
        pitchendlink_maininsertionlinkPP - 
        (pitchendlink_maininsertionlinkRot.transpose() * pitchendlink_maininsertionlinkCP);
    }
    //--------------------------------------------------------------------//
    {
      maininsertionlink_toollinkRot = 
      Eigen::Matrix3d(Eigen::Quaterniond::FromTwoVectors
        (maininsertionlink_toollinkPA, maininsertionlink_toollinkCA));
      Eigen::Affine3d rotOffset(
        Eigen::AngleAxisd(maininsertionlink_toollinkOffsetQ, maininsertionlink_toollinkCA));
      maininsertionlink_toollinkRotOffset = rotOffset.rotation();

      // B_maininsertionlink_toollinkST.E = Matrix3d( 0,  0, -1, 1,  0, 0, 0, -1, 0);
      B_maininsertionlink_toollinkST.E = 
        (maininsertionlink_toollinkRotOffset * maininsertionlink_toollinkRot).transpose();
      B_maininsertionlink_toollinkST.r = 
        maininsertionlink_toollinkPP - 
        (maininsertionlink_toollinkRot.transpose() * maininsertionlink_toollinkCP);
    }

    // std::cout << "world_baselinkST" 			 					<< std::endl << world_baselinkST 								<< std::endl;
    // std::cout << "baselink_yawlinkST" 		 					<< std::endl << baselink_yawlinkST 							<< std::endl;
    // std::cout << "yawlink_pitchbacklinkST" 					<< std::endl << yawlink_pitchbacklinkST 				<< std::endl;
    // std::cout << "pitchbacklink_pitchbottomlinkST" 	<< std::endl << pitchbacklink_pitchbottomlinkST << std::endl;
    // std::cout << "pitchbottomlink_pitchendlinkST" 	<< std::endl << pitchbottomlink_pitchendlinkST 	<< std::endl;
    // std::cout << "pitchendlink_maininsertionlinkST" << std::endl << pitchendlink_maininsertionlinkST << std::endl;
    // std::cout << "maininsertionlink_toollinkST" 		<< std::endl << maininsertionlink_toollinkST 		<< std::endl;

    //--------------------------------------------------------------------//
    W_world_yawlinkST           = W_world_baselinkST * B_baselink_yawlinkST;
    W_world_pitchbacklinkST     = W_world_yawlinkST * B_yawlink_pitchbacklinkST;
    W_world_pitchbottomlinkST   = W_world_pitchbacklinkST * B_pitchbacklink_pitchbottomlinkST;
    W_world_pitchendlinkST      = W_world_pitchbottomlinkST * B_pitchbottomlink_pitchendlinkST;

    W_world_pitchfrontlinkST    = W_world_yawlinkST * B_yawlink_pitchfrontlinkST;
    // W_world_pitchbottomlink2ST = W_world_pitchfrontlinkST * B_pitchfrontlink_pitchbottomlinkST;
    W_world_pitchtoplinkST      = W_world_pitchfrontlinkST * B_pitchfrontlink_pitchtoplinkST;
    W_world_pitchendlinkST      = W_world_pitchtoplinkST * B_pitchtoplink_pitchendlinkST;

    W_world_maininsertionlinkST = W_world_pitchendlinkST * B_pitchendlink_maininsertionlinkST;
    W_world_toollinkST          = W_world_maininsertionlinkST * B_maininsertionlink_toollinkST;

    W_world_pitchendlinkST.E      = Matrix3d( 0,  0, -1,  1,  0, 0,  0, -1, 0);
    W_world_pitchfrontlinkST.E    = Matrix3d( 0,  0, -1,  1,  0, 0,  0, -1, 0);
    // W_world_pitchbottomlink2ST.E  = Matrix3d( 1,  0,  0,  0,  0, 1,  0, -1, 0);
    W_world_pitchtoplinkST.E      = Matrix3d( 0,  0, -1,  1,  0, 0,  0, -1, 0);
    W_world_maininsertionlinkST.E = Matrix3d( 0,  0, -1,  0, -1, 0, -1,  0, 0);
    W_world_toollinkST.E          = Matrix3d( 0,  1,  0, -1,  0, 0,  0,  0, 1);


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
    baselink_yawlinkJAxis = 
    W_world_baselinkST.E * baselink_yawlinkPA;
    
    yawlink_pitchbacklinkJAxis = 
      W_world_yawlinkST.E * yawlink_pitchbacklinkPA;
    pitchbacklink_pitchbottomlinkJAxis = 
      W_world_pitchbacklinkST.E * pitchbacklink_pitchbottomlinkPA;
    pitchbottomlink_pitchendlinkJAxis = 
      W_world_pitchbottomlinkST.E * pitchbottomlink_pitchendlinkPA;
    
    yawlink_pitchfrontlinkJAxis = 
      W_world_yawlinkST.E * yawlink_pitchfrontlinkPA;
    pitchfrontlink_pitchtoplinkJAxis = 
      W_world_pitchfrontlinkST.E * pitchfrontlink_pitchtoplinkPA;
    pitchtoplink_pitchendlinkJAxis = 
      W_world_pitchtoplinkST.E * pitchtoplink_pitchendlinkPA;
    
    pitchfrontlink_pitchbottomlinkJAxis = 
      W_world_pitchfrontlinkST.E * pitchfrontlink_pitchbottomlinkPA;
    pitchendlink_maininsertionlinkJAxis = 
      W_world_pitchendlinkST.E * pitchendlink_maininsertionlinkPA;
    maininsertionlink_toollinkJAxis = 
      W_world_maininsertionlinkST.E * maininsertionlink_toollinkPA;

    // std::cout << "baselink_yawlinkJAxis"               << std::endl 
    //           << baselink_yawlinkJAxis                 << std::endl;
    // std::cout << "yawlink_pitchbacklinkJAxis"          << std::endl 
    //           << yawlink_pitchbacklinkJAxis            << std::endl;
    // std::cout << "pitchbacklink_pitchbottomlinkJAxis"  << std::endl 
    //           << pitchbacklink_pitchbottomlinkJAxis    << std::endl;
    // std::cout << "pitchbottomlink_pitchendlinkJAxis"   << std::endl 
    //           << pitchbottomlink_pitchendlinkJAxis     << std::endl;

    // std::cout << "yawlink_pitchfrontlinkJAxis"         << std::endl 
    //           << yawlink_pitchfrontlinkJAxis           << std::endl;
    // std::cout << "pitchfrontlink_pitchtoplinkJAxis"    << std::endl 
    //           << pitchfrontlink_pitchtoplinkJAxis      << std::endl;
    // std::cout << "pitchtoplink_pitchendlinkJAxis"      << std::endl 
    //           << pitchtoplink_pitchendlinkJAxis        << std::endl;
    // std::cout << "pitchfrontlink_pitchbottomlinkJAxis" << std::endl 
    //           << pitchfrontlink_pitchbottomlinkJAxis   << std::endl;
    
    // std::cout << "pitchendlink_maininsertionlinkJAxis" << std::endl 
    //           << pitchendlink_maininsertionlinkJAxis   << std::endl;
    // std::cout << "maininsertionlink_toollinkJAxis"     << std::endl 
    //           << maininsertionlink_toollinkJAxis       << std::endl;
    //1--------------------------------------------------------------------//
    // std::cout << "baselink_yawlinkRot"               << std::endl 
    //           << baselink_yawlinkRot                 << std::endl;
    // std::cout << "yawlink_pitchbacklinkRot"          << std::endl 
    //           << yawlink_pitchbacklinkRot            << std::endl;
    // std::cout << "pitchbacklink_pitchbottomlinkRot"  << std::endl 
    //           << pitchbacklink_pitchbottomlinkRot    << std::endl;
    // std::cout << "pitchbottomlink_pitchendlinkRot"   << std::endl 
    //           << pitchbottomlink_pitchendlinkRot     << std::endl;

    // std::cout << "yawlink_pitchfrontlinkRot"         << std::endl 
    //           << yawlink_pitchfrontlinkRot           << std::endl;
    // std::cout << "pitchfrontlink_pitchtoplinkRot"    << std::endl 
    //           << pitchfrontlink_pitchtoplinkRot      << std::endl;
    // std::cout << "pitchtoplink_pitchendlinkRot"      << std::endl 
    //           << pitchtoplink_pitchendlinkRot        << std::endl;
    // std::cout << "pitchfrontlink_pitchbottomlinkRot" << std::endl 
    //           << pitchfrontlink_pitchbottomlinkRot   << std::endl;
    
    // std::cout << "pitchendlink_maininsertionlinkRot" << std::endl 
    //           << pitchendlink_maininsertionlinkRot   << std::endl;
    // std::cout << "maininsertionlink_toollinkRot"     << std::endl 
    //           << maininsertionlink_toollinkRot       << std::endl;
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

  SpatialTransform W_world_baselinkST, W_world_yawlinkST, W_world_pitchfrontlinkST, 
  W_world_pitchbacklinkST, W_world_pitchbottomlinkST, W_world_pitchendlinkST, 
  W_world_maininsertionlinkST, W_world_pitchtoplinkST,
	W_world_toollinkST;

  SpatialTransform B_baselink_yawlinkST, B_yawlink_pitchbacklinkST, B_pitchbacklink_pitchbottomlinkST,
  B_pitchbottomlink_pitchendlinkST, B_yawlink_pitchfrontlinkST, B_pitchfrontlink_pitchbottomlinkST,
  B_pitchfrontlink_pitchtoplinkST, B_pitchtoplink_pitchendlinkST, B_pitchendlink_maininsertionlinkST,
  B_maininsertionlink_toollinkST;

  SpatialTransform B_baselink_yawlinkSTNew;

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

};
