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
  unsigned int baselink_yawlinkId, yawlink_pitchbacklinkId, yawlink_pitchfrontlinkId, 
	pitchbacklink_pitchbottomlink_v_Id, pitchbottomlink_pitchendlinkId, pitchendlink_maininsertionlinkId,
	pitchfrontlink_pitchtoplinkId, pitchfrontlink_pitchbottomlinkId, pitchtoplink_pitchendlinkId,
	maininsertionlink_toollinkId, toollink_eeId;

  SpatialTransform world_baselinkST, world_yawlinkST, world_pitchfrontlinkST, world_pitchbacklinkST, 
	world_pitchbottomlinkST, world_pitchendlinkST, world_maininsertionlinkST, world_pitchtoplinkST,
	world_toollinkST, world_eeST;

  const double ROOT_baselinkOffsetQ                  = 0.0;
  const double baselink_yawlinkOffsetQ               = -3.1414;
  const double yawlink_pitchbacklinkOffsetQ          = 3.1416;
  const double pitchbacklink_pitchbottomlinkOffsetQ  = 0.0;
  const double baselink_pitchendlinkOffsetQ          = 1.56304;
  const double pitchendlink_maininsertionlinkOffsetQ = -1.5708;
  const double maininsertionlink_toollinkOffsetQ     = -1.5708;
  const double pitchbottomlink_pitchendlinkOffsetQ   = 0.0;
  const double yawlink_pitchfrontlinkOffsetQ         = 3.1416;
  const double pitchfrontlink_pitchbottomlinkOffsetQ = 0.0;
  const double pitchfrontlink_pitchtoplinkOffsetQ    = 0.0;
  const double pitchtoplink_pitchendlinkOffsetQ      = 0.0;
	const double toollink_eeOffsetQ      							 = 0.0;

	SetBodyParams();
	rbdlModel_ = new Model;
	rbdlModel_->gravity = Vector3d(0., 0., -9.81);

	// Modify this with World the base transformation
	world_baselinkST.E.setIdentity();
	world_baselinkST.r.setZero();

	std::cout << "-----------------------------" << std::endl;
	//1--------------------------------------------------------------------//
	Vector3d baselink_yawlinkPA = { -0.0002, -1.0000, 00.0000 };
	Vector3d baselink_yawlinkCA = { 00.0000, 00.0000, -1.0000 };
	Vector3d baselink_yawlinkPP = { 00.0000, 00.0000, 00.0000 };
	Vector3d baselink_yawlinkCP = { 00.0001, 00.0000, 00.5369 };

	Vector3d p_world_baselink   = Vector3d::Zero();
	Vector3d p_world_yawlink		= Vector3d::Zero();
	
	CreateRBDLJoint(baselink_yawlinkPA, baselink_yawlinkCA, baselink_yawlinkPP, baselink_yawlinkCP, 
	baselink_yawlinkOffsetQ, Vector3d::UnitZ(), 0, p_world_baselink, Joint(SpatialVector (0., 1., 0., 0., 0., 0.)),
	world_baselinkST, baselinkBody_, "baselink-yawlink", p_world_yawlink, baselink_yawlinkId, 
	world_yawlinkST);
	std::cout << "-----------------------------" << std::endl;
	//2 and 3 --------------------------------------------------------------------//
	Vector3d yawlink_pitchfrontlinkPA = { 1.0, 0.0,   0.0 };
	Vector3d yawlink_pitchfrontlinkCA = { 0.0, 0.0,   1.0 };
	Vector3d yawlink_pitchfrontlinkPP = { 0.0, 0.0, 0.199 };
	Vector3d yawlink_pitchfrontlinkCP = { 0.0, 0.0,   0.0 };

	Vector3d p_world_pitchfrontlink	 	= Vector3d::Zero();

	CreateRBDLJoint(yawlink_pitchfrontlinkPA, yawlink_pitchfrontlinkCA, yawlink_pitchfrontlinkPP, 
	yawlink_pitchfrontlinkCP, yawlink_pitchfrontlinkOffsetQ, Vector3d::UnitZ(), baselink_yawlinkId, 
	p_world_yawlink, Joint(SpatialVector (-1., 0., 0., 0., 0., 0.)), world_yawlinkST, yawlinkBody_, 
	"yawlink-pitchfrontlink", p_world_pitchfrontlink, yawlink_pitchfrontlinkId, world_pitchfrontlinkST);
	std::cout << "-----------------------------" << std::endl;
	//3--------------------------------------------------------------------//
	Vector3d yawlink_pitchbacklinkPA = { 1.0,     0.0,    0.0 };
	Vector3d yawlink_pitchbacklinkCA = { 0.0,     0.0,    1.0 };
	Vector3d yawlink_pitchbacklinkPP = { 0.0, -0.0098, 0.1624 };
	Vector3d yawlink_pitchbacklinkCP = { 0.0,     0.0,    0.0 };

	Vector3d p_world_pitchbacklink	 = Vector3d::Zero();

	CreateRBDLJoint(yawlink_pitchbacklinkPA, yawlink_pitchbacklinkCA, yawlink_pitchbacklinkPP, 
	yawlink_pitchbacklinkCP, yawlink_pitchbacklinkOffsetQ, Vector3d::UnitZ(), baselink_yawlinkId, 
	p_world_yawlink, Joint(SpatialVector (-1., 0., 0., 0., 0., 0.)), world_yawlinkST, yawlinkBody_, 
	"yawlink-pitchbacklink", p_world_pitchbacklink, yawlink_pitchbacklinkId, world_pitchbacklinkST);
	std::cout << "-----------------------------" << std::endl;
  //--------------------------------------------------------------------//
	Vector3d pitchbacklink_pitchbottomlinkPA = {     0.0,     0.0,     1.0 };
	Vector3d pitchbacklink_pitchbottomlinkCA = {     0.0,     0.0,     1.0 };
	Vector3d pitchbacklink_pitchbottomlinkPP = { -0.1028, -0.2867,     0.0 };
	Vector3d pitchbacklink_pitchbottomlinkCP = { -0.0364,  0.0098, -0.0005 };

	Vector3d p_world_pitchbottomlink	 			 = Vector3d::Zero();

	CreateRBDLJoint(pitchbacklink_pitchbottomlinkPA, pitchbacklink_pitchbottomlinkCA, 
	pitchbacklink_pitchbottomlinkPP, pitchbacklink_pitchbottomlinkCP, pitchbacklink_pitchbottomlinkOffsetQ, 
	Vector3d::UnitZ(), yawlink_pitchbacklinkId, p_world_pitchbacklink, 
	Joint(SpatialVector (-1., 0., 0., 0., 0., 0.)), 
	world_pitchbacklinkST, virtualBody_, "pitchbacklink-pitchbottom-v", 
	p_world_pitchbottomlink, pitchbacklink_pitchbottomlink_v_Id, world_pitchbottomlinkST);
	std::cout << "-----------------------------" << std::endl;
	//8--------------------------------------------------------------------//
	Vector3d pitchbottomlink_pitchendlinkPA = {    0.0,     0.0,     1.0 };
	Vector3d pitchbottomlink_pitchendlinkCA = {    0.0,     0.0,     1.0 };
	Vector3d pitchbottomlink_pitchendlinkPP = { 0.3401, -0.0001, -0.0005 };
	Vector3d pitchbottomlink_pitchendlinkCP = {    0.0,     0.0,  0.0001 };

	Vector3d p_world_pitchendlink	 	 			 	= Vector3d::Zero();

	CreateRBDLJoint(pitchbottomlink_pitchendlinkPA, pitchbottomlink_pitchendlinkCA, 
	pitchbottomlink_pitchendlinkPP, pitchbottomlink_pitchendlinkCP, pitchbottomlink_pitchendlinkOffsetQ, 
	Vector3d::UnitZ(), pitchbacklink_pitchbottomlink_v_Id, p_world_pitchbottomlink, 
	Joint(SpatialVector (-1., 0., 0., 0., 0., 0.)), 
	world_pitchbottomlinkST, pitchbottomlinkBody_, "pitchbottomlink-pitchendlink", 
	p_world_pitchendlink, pitchbottomlink_pitchendlinkId, world_pitchendlinkST);
	std::cout << "-----------------------------" << std::endl;
	//9--------------------------------------------------------------------//
	Vector3d pitchendlink_maininsertionlinkPA = {     0.0,     1.0,    0.0 };
	Vector3d pitchendlink_maininsertionlinkCA = {     1.0,     0.0,    0.0 };
	Vector3d pitchendlink_maininsertionlinkPP = {  0.1031, -0.0961, 0.0001 };
	Vector3d pitchendlink_maininsertionlinkCP = { -0.0108,  -0.062, 0.0002 };
	Vector3d p_world_maininsertionlink	 		 	= Vector3d::Zero();

	CreateRBDLJoint(pitchendlink_maininsertionlinkPA, pitchendlink_maininsertionlinkCA, 
	pitchendlink_maininsertionlinkPP, pitchendlink_maininsertionlinkCP, pitchendlink_maininsertionlinkOffsetQ, 
	-Vector3d::UnitZ(), pitchbottomlink_pitchendlinkId, p_world_pitchendlink, 
	Joint(SpatialVector (0., 0., 0., 0., 0., -1.)),
	world_pitchendlinkST, pitchendlinkBody_, "pitchendlink-maininsertionlink", 
	p_world_maininsertionlink, pitchendlink_maininsertionlinkId, world_maininsertionlinkST);
	std::cout << "-----------------------------" << std::endl;
	//10--------------------------------------------------------------------//
	Vector3d maininsertionlink_toollinkPA = {     1.0,     0.0,    0.0 };
	Vector3d maininsertionlink_toollinkCA = {     0.0,     0.0,   -1.0 };
	Vector3d maininsertionlink_toollinkPP = { -0.0108,  -0.062,    0.0 };
	Vector3d maininsertionlink_toollinkCP = { -0.0001, -0.0002, 0.0118 };
	Vector3d p_world_toollink	 		 			  = Vector3d::Zero();

	CreateRBDLJoint(maininsertionlink_toollinkPA, maininsertionlink_toollinkCA, 
	maininsertionlink_toollinkPP, maininsertionlink_toollinkCP, maininsertionlink_toollinkOffsetQ, 
	Vector3d::UnitZ(), pitchendlink_maininsertionlinkId, p_world_maininsertionlink, 
	Joint(SpatialVector (0., 0., 1., 0., 0., 0.)),
	world_maininsertionlinkST, maininsertionlinkBody_, "maininsertionlink-toollink", 
	p_world_toollink, maininsertionlink_toollinkId, world_toollinkST);
	std::cout << "p_world_toollink" << std::endl << p_world_toollink << std::endl;
	std::cout << "world_toollinkST" << std::endl << world_toollinkST << std::endl;
	std::cout << "-----------------------------" << std::endl;
//11--------------------------------------------------------------------//
	Vector3d toollink_eePA = { 0.0, 0.0, 1.0 };
	Vector3d toollink_eeCA = { 0.0, 0.0, 1.0 };
	Vector3d toollink_eePP = Vector3d::Zero();
	Vector3d toollink_eeCP = Vector3d::Zero();
	Vector3d p_world_ee	 	 = Vector3d::Zero();

	CreateRBDLJoint(toollink_eePA, toollink_eeCA, 
	toollink_eePP, toollink_eeCP, toollink_eeOffsetQ, 
	Vector3d::UnitZ(), maininsertionlink_toollinkId, p_world_toollink, 
	Joint(SpatialVector (0., 0., 0., 0., 0., 0.)),
	world_toollinkST, toollinkBody_, "toollink_ee", 
	p_world_ee, toollink_eeId, world_eeST);
	std::cout << "-----------------------------" << std::endl;
	//--------------------------------------------------------------------//
	unsigned int userDefinedId = 7;
	cs_.AddLoopConstraint(yawlink_pitchfrontlinkId, pitchbacklink_pitchbottomlink_v_Id, X_p_, X_s_, 
		SpatialVector(0, 0, 0, 1, 0, 0), bgStab_, 0.1, "LoopXY_Rz", userDefinedId);

	//These two constraints below will be appended to the above
	//constraint by default, and will assume its name and user defined id
	cs_.AddLoopConstraint(yawlink_pitchfrontlinkId, pitchbacklink_pitchbottomlink_v_Id, X_p_, X_s_, 
		SpatialVector(0, 0, 0, 0, 1, 0));
	cs_.AddLoopConstraint(yawlink_pitchfrontlinkId, pitchbacklink_pitchbottomlink_v_Id, X_p_, X_s_, 
		SpatialVector(0, 0, 1, 0, 0, 0));
	cs_.Bind(*rbdlModel_);

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

