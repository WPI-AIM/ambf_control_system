#include "rbdl_model_tests/ECM.h"

ECM::ECM() 
{
	// ConnectToAMBF();
	// SetAMBFParams();
	// ExecutePoseInAMBF();
  
	// PrintAMBFTransformation();

	SetBodyParams();
	CreateRBDLModel();
	CheckRBDLModel();
}

void ECM::ConnectToAMBF()
{
	ambfClientPtr_ = RBDLTestPrep::getInstance()->getAMBFClientInstance();
	ambfClientPtr_->connect();
	usleep(20000);
}

void ECM::SetAMBFParams()
{
	std::vector<std::string> rigidBodyNames = ambfClientPtr_->getRigidBodyNames();
	for(std::string rigidBodyName : rigidBodyNames)
	{
		std::cout << rigidBodyName << std::endl;
		ambfParamMap_[rigidBodyName] = new AMBFParams(
				rigidBodyName, 
				ambfClientPtr_->getRigidBody(rigidBodyName.c_str(), true), 
				std::vector<std::string>{},
				std::vector<ControllableJointConfig>{},
				Matrix3dZero,
				Vector3dZero
		);
	}

	ambfParamMap_[baselinkName_]->ControllableJointConfigs(
		std::vector<ControllableJointConfig> 
		{ 
			{               "baselink-yawlink", -1.595, 1.595 },
			{          "yawlink-pitchbacklink", -0.784, 1.158 },
			{ "pitchendlink-maininsertionlink", 0.0000, 0.254 },
			{     "maininsertionlink-toollink", -1.553, 1.567 },
		}
	);

	usleep(250000);
}

void ECM::ExecutePoseInAMBF()
{
	const useconds_t microsec = 250000;
	AMBFParamsPtr ambfRigidBodyParams = ambfParamMap_[baselinkName_];

	// const std::string parentBody = ambfRigidBodyParamsPtr->ParentBodyName();
	rigidBodyPtr ambfRigidBodyHandler = ambfRigidBodyParams->RididBodyHandler();
	std::vector<std::string> rigidBodyChildren = ambfRigidBodyParams->ChildrenJoints();
	std::vector<ControllableJointConfig> rigidBodyControllableJointConfigs = 
		ambfRigidBodyParams->ControllableJointConfigs();

	std::vector<std::string> rigidBodyNames = ambfClientPtr_->getRigidBodyNames();

	for(int i = 0; i < 1; i++)
	{
		for(ControllableJointConfig jointConfig : rigidBodyControllableJointConfigs)
		{
			ambfRigidBodyHandler->set_joint_pos<std::string>(jointConfig.jointName, 0.0f);
			usleep(microsec);
		}
	}

	for(ambfParamMapItr_ = ambfParamMap_.begin(); ambfParamMapItr_ != ambfParamMap_.end(); ambfParamMapItr_++)
	{
		const std::string parentBody = ambfParamMapItr_->first;
		AMBFParamsPtr ambfRigidBodyParams = ambfParamMap_[parentBody];

		rigidBodyPtr rigidBodyHandler = ambfParamMapItr_->second->RididBodyHandler();

		tf::Quaternion quat_w_n_tf = rigidBodyHandler->get_rot();
		Quaternion quat_w_n = EigenUtilities::TFtoEigenQuaternion(quat_w_n_tf);
		Matrix3d r_w_n = quat_w_n.toMatrix();

		tf::Vector3 p_w_n_tf = rigidBodyHandler->get_pos();
		Vector3d p_w_n = EigenUtilities::TFtoEigenVector(p_w_n_tf);

		ambfRigidBodyParams->RotationMatrix(r_w_n);
		ambfRigidBodyParams->TranslationVector(p_w_n);
		ambfParamMap_[parentBody] = ambfRigidBodyParams;
	}

	// Register Word to Base Transform
	ambfRigidBodyParams = ambfParamMap_[baselinkName_];

	t_w_0_.block<3, 3>(0, 0) = ambfRigidBodyParams->RotationMatrix();
	t_w_0_.block<3, 1>(0, 3) = ambfRigidBodyParams->TranslationVector();

	// std::cout << "t_w_0_" << std::endl << t_w_0_ << std::endl;
}

const Matrix3d ECM::PrintAMBFTransformation()
{
	t_0_w_.block<3, 3>(0, 0) = t_w_0_.block<3, 3>(0, 0).transpose();
	t_0_w_.block<3, 1>(0, 3) = -t_w_0_.block<3, 3>(0, 0).transpose() * t_w_0_.block<3, 1>(0, 3);
	std::cout << "t_0_w_" << std::endl << t_0_w_ << std::endl;

	for(ambfParamMapItr_ = ambfParamMap_.begin(); ambfParamMapItr_ != ambfParamMap_.end(); ambfParamMapItr_++)
	{
		const std::string parentBody = ambfParamMapItr_->first;
		AMBFParamsPtr ambfRigidBodyParams = ambfParamMap_[parentBody];

		MatrixNd t_w_n = MatrixNd::Identity(4, 4);
		t_w_n.block<3, 3>(0, 0) = ambfRigidBodyParams->RotationMatrix();
		t_w_n.block<3, 1>(0, 3) = ambfRigidBodyParams->TranslationVector();
		
		MatrixNd t_0_n = MatrixNd::Identity(4, 4);
		t_0_n = t_0_w_ * t_w_n;

		std::cout << "parentBody: " << parentBody << std::endl;
		std::cout << "t_w_n" << std::endl << t_w_n << std::endl;
		std::cout << "t_0_n" << std::endl << t_0_n << std::endl;
		std::cout << "---------------------------" << std::endl;
	}

	Matrix3d dummy;
	return dummy;
}

// const Vector3d TranslationVectorTF();

void ECM::SetBodyParams()
{
	// mass, com - inertia offset, inertia
	baselinkBody_          = Body(01.000, Vector3d(-0.0046, 00.0000, 00.0801), 
																				Vector3d(00.0000, 00.0000, 00.0000));
	yawlinkBody_           = Body(06.417, Vector3d(00.0000, -0.0161, 00.1345), 
																				Vector3d(00.2978, 00.3125, 00.0449));
	pitchbacklinkBody_     = Body(00.421, Vector3d(-0.0515, -0.1434, -0.0090), 
																				Vector3d(00.0236, 00.0028, 00.0261));
	pitchbottomlinkBody_   = Body(00.359, Vector3d(00.1491, -0.0182, 00.0000), 
																				Vector3d(00.0007, 00.0190, 00.0192));
	maininsertionlinkBody_ = Body(00.231, Vector3d(-0.0590, -0.0165, 00.0008), 
																				Vector3d(00.0003, 00.0015, 00.0016));
	toollinkBody_          = Body(01.907, Vector3d(00.0000, -0.0008, 00.0723), 
																				Vector3d(00.0457, 00.0455, 00.0017));
	pitchfrontlinkBody_    = Body(01.607, Vector3d(-0.0365, -0.1526, 00.0000), 
																				Vector3d(00.0983, 00.0175, 00.1099));
	pitchtoplinkBody_      = Body(00.439, Vector3d(00.1702, -0.0001, 00.0008), 
																				Vector3d(00.0000, 00.0381, 00.0381));
	// No Joint for pittchEndlink as its a p2p joint
	pitchendlinkBody_      = Body(02.032, Vector3d(00.0513, 00.0048, 00.0008), 
																				Vector3d(00.0636, 00.0099, 00.0726));
}

void ECM::CreateRBDLJoint(Vector3d& pa, Vector3d& ca, Vector3d& pp, Vector3d& cp, const double offsetQ, 
	Vector3d axis, unsigned int parentId, Vector3d& p_world_startingbody, Joint joint, SpatialTransform world_parentST, Body &body, 
	std::string bodyName, Vector3d& p_world_endingbody, unsigned int& newBodyId, SpatialTransform&	world_bodyST)
{
	pa.normalize();
	ca.normalize();

	Matrix3d bodyRot = Eigen::Matrix3d(Eigen::Quaterniond::FromTwoVectors(pa, ca));
	Eigen::Affine3d rotOffset(Eigen::AngleAxisd(offsetQ, axis));
		
	SpatialTransform bodyST;
	bodyST.E = rotOffset.rotation() * bodyRot;
	bodyST.r = pp - (bodyRot.inverse() * cp);

	//Vector3d roationAxis = world_parentST.E.transpose().block<3, 1>(0, 2).transpose();
	Vector3d roationAxis = world_parentST.E.block<3, 1>(0, 1).transpose();
	roationAxis.normalize();
	// Joint joint = Joint(jointType, 
	// 	world_parentST.E.transpose().block<3, 1>(0, 2).transpose());
	// std::cout << std::endl << "world_parentST" << std::endl << world_parentST << std::endl;
	// std::cout << std::endl << "roationAxis" << std::endl << roationAxis << std::endl;

	// Joint joint = Joint(jointType, roationAxis);

	// const Vector3d p_world_body =  world_parentST.E * bodyST.r;
	p_world_endingbody =  world_parentST.E * bodyST.r;
	// newBodyId = rbdlModel_->AddBody(parentId, Xtrans(p_world_body), joint, body, bodyName);
	newBodyId = rbdlModel_->AddBody(parentId, Xtrans(p_world_startingbody), joint, body, bodyName);
	world_bodyST = world_parentST * bodyST;

	printf("%s\n", bodyName.c_str());
}

void ECM::CreateRBDLModel()
{
  unsigned int baselink_yawlinkId_, yawlink_pitchbacklinkId_, yawlink_pitchfrontlinkId_, 
	pitchbacklink_pitchbottomlinkId_, pitchbottomlink_pitchendlinkId_, pitchendlink_maininsertionlinkId_,
	pitchfrontlink_pitchtoplinkId_, pitchfrontlink_pitchbottomlinkId_, pitchtoplink_pitchendlinkId_,
	maininsertionlink_toollinkId_;


  SpatialTransform world_baselinkST_, world_yawlinkST_, world_pitchfrontlinkST_, world_pitchbacklinkST_, 
	world_pitchbottomlinkST_, world_pitchendlinkST_, world_maininsertionlinkST_, world_pitchtoplinkST_,
	world_toollinkST_;

  const double ROOT_baselinkOffsetQ_                  = 0.0;
  const double baselink_yawlinkOffsetQ_               = -3.1414;
  const double yawlink_pitchbacklinkOffsetQ_          = 3.1416;
  const double pitchbacklink_pitchbottomlinkOffsetQ_  = 0.0;
  const double baselink_pitchendlinkOffsetQ_          = 1.56304;
  const double pitchendlink_maininsertionlinkOffsetQ_ = -1.5708;
  const double maininsertionlink_toollinkOffsetQ_     = -1.5708;
  const double pitchbottomlink_pitchendlinkOffsetQ_   = 0.0;
  const double yawlink_pitchfrontlinkOffsetQ_         = 3.1416;
  const double pitchfrontlink_pitchbottomlinkOffsetQ_ = 0.0;
  const double pitchfrontlink_pitchtoplinkOffsetQ_    = 0.0;
  const double pitchtoplink_pitchendlinkOffsetQ_      = 0.0;

	SetBodyParams();
	rbdlModel_ = new Model;
	rbdlModel_->gravity = Vector3d(0., 0., -9.81);

	// Modify this with World the base transformation
	world_baselinkST_.E.setIdentity();
	world_baselinkST_.r.setZero();

	std::cout << "-----------------------------" << std::endl;
	//1--------------------------------------------------------------------//
	Vector3d baselink_yawlinkPA = { -0.0002, -1.0000, 00.0000 };
	Vector3d baselink_yawlinkCA = { 00.0000, 00.0000, -1.0000 };
	Vector3d baselink_yawlinkPP = { 00.0000, 00.0000, 00.0000 };
	Vector3d baselink_yawlinkCP = { 00.0001, 00.0000, 00.5369 };

	Vector3d p_world_baselink   = { 00.0000, 00.0000, 00.0000 };
	Vector3d p_world_yawlink		= { 00.0000, 00.0000, 00.0000 };
	
	CreateRBDLJoint(baselink_yawlinkPA, baselink_yawlinkCA, baselink_yawlinkPP, baselink_yawlinkCP, 
	baselink_yawlinkOffsetQ_, Vector3d::UnitZ(), 0, p_world_baselink, Joint(SpatialVector (0., 1., 0., 0., 0., 0.)),
	world_baselinkST_, baselinkBody_, "baselink-yawlink", p_world_yawlink, baselink_yawlinkId_, 
	world_yawlinkST_);
	std::cout << "-----------------------------" << std::endl;
	//2 and 3 --------------------------------------------------------------------//
	Vector3d yawlink_pitchfrontlinkPA = { 1.0, 0.0,   0.0 };
	Vector3d yawlink_pitchfrontlinkCA = { 0.0, 0.0,   1.0 };
	Vector3d yawlink_pitchfrontlinkPP = { 0.0, 0.0, 0.199 };
	Vector3d yawlink_pitchfrontlinkCP = { 0.0, 0.0,   0.0 };

	Vector3d p_world_pitchfrontlink	 	= { 0.0, 0.0, 0.000 };

	CreateRBDLJoint(yawlink_pitchfrontlinkPA, yawlink_pitchfrontlinkCA, yawlink_pitchfrontlinkPP, 
	yawlink_pitchfrontlinkCP, yawlink_pitchfrontlinkOffsetQ_, Vector3d::UnitZ(), baselink_yawlinkId_, 
	p_world_yawlink, Joint(SpatialVector (-1., 0., 0., 0., 0., 0.)), world_yawlinkST_, yawlinkBody_, 
	"yawlink-pitchfrontlink", p_world_pitchfrontlink, yawlink_pitchfrontlinkId_, world_pitchfrontlinkST_);
	std::cout << "-----------------------------" << std::endl;
	//3--------------------------------------------------------------------//
	Vector3d yawlink_pitchbacklinkPA = { 1.0,     0.0,    0.0 };
	Vector3d yawlink_pitchbacklinkCA = { 0.0,     0.0,    1.0 };
	Vector3d yawlink_pitchbacklinkPP = { 0.0, -0.0098, 0.1624 };
	Vector3d yawlink_pitchbacklinkCP = { 0.0,     0.0,    0.0 };

	Vector3d p_world_pitchbacklink	 = { 0.0, 00.0000, 0.0000 };

	CreateRBDLJoint(yawlink_pitchbacklinkPA, yawlink_pitchbacklinkCA, yawlink_pitchbacklinkPP, 
	yawlink_pitchbacklinkCP, yawlink_pitchbacklinkOffsetQ_, Vector3d::UnitZ(), baselink_yawlinkId_, 
	p_world_yawlink, Joint(SpatialVector (-1., 0., 0., 0., 0., 0.)), world_yawlinkST_, yawlinkBody_, 
	"yawlink-pitchbacklink", p_world_pitchbacklink, yawlink_pitchbacklinkId_, world_pitchbacklinkST_);
	std::cout << "-----------------------------" << std::endl;
	//4-8-------------------------------------------------------------------//
	Vector3d pitchfrontlink_pitchtoplinkPA = {     0.0,     0.0,     1.0 };
	Vector3d pitchfrontlink_pitchtoplinkCA = {     0.0,     0.0,     1.0 };
	Vector3d pitchfrontlink_pitchtoplinkPP = { -0.1084, -0.3242,     0.0 };
	Vector3d pitchfrontlink_pitchtoplinkCP = {     0.0,     0.0, -0.0006 };

	Vector3d p_world_pitchtoplink	 = { 0.0, 00.0000, 0.0000 };

	CreateRBDLJoint(pitchfrontlink_pitchtoplinkPA, pitchfrontlink_pitchtoplinkCA, pitchfrontlink_pitchtoplinkPP, 
	pitchfrontlink_pitchtoplinkCP, pitchfrontlink_pitchtoplinkOffsetQ_, Vector3d::UnitZ(), yawlink_pitchfrontlinkId_, 
	p_world_pitchfrontlink, Joint(SpatialVector (-1., 0., 0., 0., 0., 0.)), world_pitchfrontlinkST_, pitchfrontlinkBody_, 
	"pitchfrontlink-pitchtoplink", p_world_pitchtoplink, pitchfrontlink_pitchtoplinkId_, world_pitchtoplinkST_);
	std::cout << "-----------------------------" << std::endl;
	//5--------------------------------------------------------------------//
	Vector3d pitchbacklink_pitchbottomlinkPA = {     0.0,     0.0,     1.0 };
	Vector3d pitchbacklink_pitchbottomlinkCA = {     0.0,     0.0,     1.0 };
	Vector3d pitchbacklink_pitchbottomlinkPP = { -0.1028, -0.2867,     0.0 };
	Vector3d pitchbacklink_pitchbottomlinkCP = { -0.0364,  0.0098, -0.0005 };

	Vector3d p_world_pitchbottomlink	 			 = { 00.0000, 00.0000, 00.0000 };

	CreateRBDLJoint(pitchbacklink_pitchbottomlinkPA, pitchbacklink_pitchbottomlinkCA, 
	pitchbacklink_pitchbottomlinkPP, pitchbacklink_pitchbottomlinkCP, pitchbacklink_pitchbottomlinkOffsetQ_, 
	Vector3d::UnitZ(), yawlink_pitchbacklinkId_, p_world_pitchbacklink, 
	Joint(SpatialVector (-1., 0., 0., 0., 0., 0.)), 
	world_pitchbacklinkST_, pitchbacklinkBody_, "pitchbacklink-pitchbottom", 
	p_world_pitchbottomlink, pitchbacklink_pitchbottomlinkId_, world_pitchbottomlinkST_);
	std::cout << "-----------------------------" << std::endl;
	//6--------------------------------------------------------------------//
	Vector3d  pitchfrontlink_pitchbottomlinkPA = {    0.0,      0.0,     1.0 };
	Vector3d  pitchfrontlink_pitchbottomlinkCA = {    0.0,      0.0,     1.0 };
	Vector3d  pitchfrontlink_pitchbottomlinkPP = { -0.1031, -0.2868,     0.0 };
	Vector3d  pitchfrontlink_pitchbottomlinkCP = { -0.0001, -0.0001, -0.0005 };

	Vector3d  p_world_pitchbottomlink_	 			 = { 00.0000, 00.0000, 00.0000 };

	CreateRBDLJoint(pitchfrontlink_pitchbottomlinkPA, pitchfrontlink_pitchbottomlinkCA, 
	pitchfrontlink_pitchbottomlinkPP, pitchfrontlink_pitchbottomlinkCP, pitchfrontlink_pitchbottomlinkOffsetQ_, 
	Vector3d::UnitZ(), yawlink_pitchfrontlinkId_, p_world_pitchfrontlink, 
	Joint(SpatialVector (-1., 0., 0., 0., 0., 0.)), 
	world_pitchfrontlinkST_, pitchfrontlinkBody_, "pitchfrontlink-pitchbottom", 
	p_world_pitchbottomlink_, pitchfrontlink_pitchbottomlinkId_, world_pitchbottomlinkST_);

	// std::cout << "p_world_pitchbottomlink" << std::endl << p_world_pitchbottomlink << std::endl;
	// std::cout << "p_world_pitchbottomlink_" << std::endl << p_world_pitchbottomlink_ << std::endl;

	std::cout << "-----------------------------" << std::endl;
	//7--------------------------------------------------------------------//
	Vector3d pitchtoplink_pitchendlinkPA = {     0.0,     0.0,     1.0 };
	Vector3d pitchtoplink_pitchendlinkCA = {     0.0,     0.0,     1.0 };
	Vector3d pitchtoplink_pitchendlinkPP = {  0.3404, -0.0002, -0.0006 };
	Vector3d pitchtoplink_pitchendlinkCP = { -0.0051, -0.0376,  0.0001 };
	Vector3d p_world_pitchendlink_	 	 	 = { 0.0000, 00.0000, 00.0000 };

	CreateRBDLJoint(pitchtoplink_pitchendlinkPA, pitchtoplink_pitchendlinkCA, 
	pitchtoplink_pitchendlinkPP, pitchtoplink_pitchendlinkCP, pitchtoplink_pitchendlinkOffsetQ_, 
	Vector3d::UnitZ(), pitchbacklink_pitchbottomlinkId_, p_world_pitchbottomlink, 
	Joint(SpatialVector (-1., 0., 0., 0., 0., 0.)), 
	world_pitchtoplinkST_, pitchtoplinkBody_, "pitchtoplink-pitchendlink", 
	p_world_pitchendlink_, pitchtoplink_pitchendlinkId_, world_pitchendlinkST_);
	std::cout << "-----------------------------" << std::endl;
	//8--------------------------------------------------------------------//
	Vector3d pitchbottomlink_pitchendlinkPA = {    0.0,     0.0,     1.0 };
	Vector3d pitchbottomlink_pitchendlinkCA = {    0.0,     0.0,     1.0 };
	Vector3d pitchbottomlink_pitchendlinkPP = { 0.3401, -0.0001, -0.0005 };
	Vector3d pitchbottomlink_pitchendlinkCP = {    0.0,     0.0,  0.0001 };

	Vector3d p_world_pitchendlink	 	 			 	= { 0.0000, 00.0000, 00.0000 };

	CreateRBDLJoint(pitchbottomlink_pitchendlinkPA, pitchbottomlink_pitchendlinkCA, 
	pitchbottomlink_pitchendlinkPP, pitchbottomlink_pitchendlinkCP, pitchbottomlink_pitchendlinkOffsetQ_, 
	Vector3d::UnitZ(), pitchbacklink_pitchbottomlinkId_, p_world_pitchbottomlink, 
	Joint(SpatialVector (-1., 0., 0., 0., 0., 0.)), 
	world_pitchbottomlinkST_, pitchbottomlinkBody_, "pitchbottomlink-pitchendlink", 
	p_world_pitchendlink, pitchbottomlink_pitchendlinkId_, world_pitchendlinkST_);
	std::cout << "-----------------------------" << std::endl;
	//9--------------------------------------------------------------------//
	Vector3d pitchendlink_maininsertionlinkPA = {     0.0,     1.0,    0.0 };
	Vector3d pitchendlink_maininsertionlinkCA = {     1.0,     0.0,    0.0 };
	Vector3d pitchendlink_maininsertionlinkPP = {  0.1031, -0.0961, 0.0001 };
	Vector3d pitchendlink_maininsertionlinkCP = { -0.0108,  -0.062, 0.0002 };

	Vector3d p_world_maininsertionlink	 		 	= { 00.0000, 00.0000, 0.0000 };

	CreateRBDLJoint(pitchendlink_maininsertionlinkPA, pitchendlink_maininsertionlinkCA, 
	pitchendlink_maininsertionlinkPP, pitchendlink_maininsertionlinkCP, pitchendlink_maininsertionlinkOffsetQ_, 
	Vector3d::UnitZ(), pitchbottomlink_pitchendlinkId_, p_world_pitchendlink, 
	Joint(SpatialVector (0., 0., 0., 0., 0., -1.)),
	world_pitchendlinkST_, maininsertionlinkBody_, "pitchendlink_maininsertionlink", 
	p_world_maininsertionlink, pitchendlink_maininsertionlinkId_, world_maininsertionlinkST_);
	std::cout << "-----------------------------" << std::endl;
	//10--------------------------------------------------------------------//
	Vector3d maininsertionlink_toollinkPA = {     1.0,     0.0,    0.0 };
	Vector3d maininsertionlink_toollinkCA = {     0.0,     0.0,   -1.0 };
	Vector3d maininsertionlink_toollinkPP = { -0.0108,  -0.062,    0.0 };
	Vector3d maininsertionlink_toollinkCP = { -0.0001, -0.0002, 0.0118 };
	Vector3d p_world_toollink	 		 			  = { 00.0000, 00.0000, 0.0000 };

	CreateRBDLJoint(maininsertionlink_toollinkPA, maininsertionlink_toollinkCA, 
	maininsertionlink_toollinkPP, maininsertionlink_toollinkCP, maininsertionlink_toollinkOffsetQ_, 
	Vector3d::UnitZ(), pitchendlink_maininsertionlinkId_, p_world_toollink, 
	Joint(SpatialVector (0., 0., 1., 0., 0., 0.)),
	world_maininsertionlinkST_, toollinkBody_, "maininsertionlink-toollink", 
	p_world_toollink, maininsertionlink_toollinkId_, world_toollinkST_);
	std::cout << "-----------------------------" << std::endl;
	//--------------------------------------------------------------------//
	Q_     = VectorNd::Constant ((size_t) rbdlModel_->dof_count, 0.);
	QDot_  = VectorNd::Constant ((size_t) rbdlModel_->dof_count, 0.);
	QDDot_ = VectorNd::Constant ((size_t) rbdlModel_->dof_count, 0.);
	Tau_   = VectorNd::Constant ((size_t) rbdlModel_->dof_count, 0.); 

	ClearLogOutput();
}

void ECM::CheckRBDLModel()
{
	std::map< std::string, unsigned int > mBodyNameMap = rbdlModel_->mBodyNameMap;
  std::map<std::string, unsigned int>::iterator mBodyNameMapItr;

  
  for(mBodyNameMapItr = mBodyNameMap.begin(); mBodyNameMapItr != mBodyNameMap.end(); mBodyNameMapItr++)
  {
    std::string jointNameRBDL = mBodyNameMapItr->first;
    unsigned int jointIDRBDL  = mBodyNameMapItr->second;
    
    const Vector3d P_0_n_rbdl = 
      CalcBodyToBaseCoordinates(*rbdlModel_, Q_, jointIDRBDL, Vector3d(0., 0., 0.), true);

		printf("jointNameRBDL: %s\n", jointNameRBDL.c_str());
		std::cout << "P_0_n_rbdl: " << std::endl << P_0_n_rbdl << std::endl;
		std::cout << "--------------------" << std::endl;
  }  
  
}
void ECM::CleanUp()
{
	for(ambfParamMapItr_ = ambfParamMap_.begin(); ambfParamMapItr_ != ambfParamMap_.end(); ambfParamMapItr_++)
	{
		const std::string parentBody = ambfParamMapItr_->first;
		AMBFParamsPtr ambfRigidBodyParamsPtr = ambfParamMapItr_->second;
		rigidBodyPtr ambfRigidBodyHandler = ambfRigidBodyParamsPtr->RididBodyHandler();

		std::cout << "Cleaning up Rigid body pointer for " << parentBody << std::endl;
		if(ambfRigidBodyHandler == nullptr)
			printf("nullptr for rigidBodyName: %s\n", parentBody.c_str());

		ambfRigidBodyHandler->cleanUp();
	}
}

ECM::~ECM() 
{
	CleanUp();
	ambfClientPtr_->cleanUp();
	delete rbdlModel_;
}

