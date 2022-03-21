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

void ECM::CreateRBDLModel()
{
	SetBodyParams();
	rbdlModel_ = new Model;
	rbdlModel_->gravity = Vector3d(0., 0., -9.81);

	// Modify this with World the base transformation
	world_baselinkST_.E.setIdentity();
	world_baselinkST_.r.setZero();

	std::cout << "world_baselinkST_" << std::endl << world_baselinkST_ << std::endl;
	//--------------------------------------------------------------------//
	{
		Vector3d baselink_yawlinkPA = { -0.0002, -1.0000, 00.0000 };
		Vector3d baselink_yawlinkCA = { 00.0000, 00.0000, -1.0000 };
		Vector3d baselink_yawlinkPP = { 00.0000, 00.0000, 00.0000 };
		Vector3d baselink_yawlinkCP = { 00.0001, 00.0000, 00.5369 };
		baselink_yawlinkPA.normalize();
		baselink_yawlinkCA.normalize();

		Matrix3d baselink_yawlinkBodyRot = Eigen::Matrix3d(
			Eigen::Quaterniond::FromTwoVectors(baselink_yawlinkPA, baselink_yawlinkCA));
		std::cout << "baselink_yawlinkBodyRot" << std::endl << baselink_yawlinkBodyRot << std::endl;

		Eigen::Affine3d baselink_yawlinkRotOffset(Eigen::AngleAxisd(baselink_yawlinkOffsetQ_, Vector3d::UnitZ()));
		
		SpatialTransform baselink_yawlinkST;
		baselink_yawlinkST.E = baselink_yawlinkRotOffset.rotation() * baselink_yawlinkBodyRot;
		baselink_yawlinkST.r = baselink_yawlinkPP - (baselink_yawlinkBodyRot.inverse() * baselink_yawlinkCP);

		Joint baselink_yawlinkJoint = 
			RigidBodyDynamics::Joint(JointTypeRevolute, 
			world_baselinkST_.E.transpose().block<3, 1>(0, 2).transpose());

		const Vector3d p_world_yawlink =  world_baselinkST_.E * baselink_yawlinkST.r;
		std::cout << "p_world_yawlink" << std::endl << p_world_yawlink << std::endl;

		baselink_yawlinkId_ = rbdlModel_->AddBody(0, Xtrans(p_world_yawlink), baselink_yawlinkJoint, 
		baselinkBody_, "baselink-yawlink");

		world_yawlinkST_ = world_baselinkST_ * baselink_yawlinkST;
		std::cout << "world_yawlinkST_" << std::endl << world_yawlinkST_ << std::endl;
		std::cout << std::endl << "-----------------end of baselink_yawlink-----------------" << std::endl;
	}
	//--------------------------------------------------------------------//
	{
		Eigen::Vector3d yawlink_pitchbacklinkPA = { 1.0,     0.0,    0.0 };
		Eigen::Vector3d yawlink_pitchbacklinkCA = { 0.0,     0.0,    1.0 };
		Eigen::Vector3d yawlink_pitchbacklinkPP = { 0.0, -0.0098, 0.1624 };
		Eigen::Vector3d yawlink_pitchbacklinkCP = { 0.0,     0.0,    0.0 };
		yawlink_pitchbacklinkPA.normalize();
		yawlink_pitchbacklinkCA.normalize();

		Matrix3d yawlink_pitchbacklinkBodyRot = Eigen::Matrix3d(
			Eigen::Quaterniond::FromTwoVectors(yawlink_pitchbacklinkPA, yawlink_pitchbacklinkCA));
		Eigen::Affine3d yawlink_pitchbacklinkRotOffset(Eigen::AngleAxisd(yawlink_pitchbacklinkOffsetQ_, Vector3d::UnitZ()));
		std::cout << "yawlink_pitchbacklinkBodyRot" << std::endl << yawlink_pitchbacklinkBodyRot << std::endl;

		SpatialTransform yawlink_pitchbacklinkST;
		yawlink_pitchbacklinkST.E = yawlink_pitchbacklinkRotOffset.rotation() * yawlink_pitchbacklinkBodyRot;
		yawlink_pitchbacklinkST.r = yawlink_pitchbacklinkPP - 
			(yawlink_pitchbacklinkBodyRot.inverse() * yawlink_pitchbacklinkCP);

		Joint yawlink_pitchbacklinkJoint = 
			RigidBodyDynamics::Joint(JointTypeRevolute, 
			world_yawlinkST_.E.transpose().block<3, 1>(0, 2).transpose());

		const Vector3d p_world_patchbacklink =  world_yawlinkST_.E * yawlink_pitchbacklinkST.r;
		std::cout << "p_world_patchbacklink" << std::endl << p_world_patchbacklink << std::endl;

		yawlink_pitchbacklinkId_ = rbdlModel_->AddBody(baselink_yawlinkId_, 
		Xtrans(p_world_patchbacklink), yawlink_pitchbacklinkJoint, 
		yawlinkBody_, "yawlink-pitchbacklink");

		world_pitchbacklinkST_ = world_yawlinkST_ * yawlink_pitchbacklinkST;
		std::cout << "world_pitchbacklinkST_" << std::endl << world_pitchbacklinkST_ << std::endl;
		std::cout << std::endl << "-----------------end of yawlink-pitchbacklink-----------------" << std::endl;
	}
	// --------------------------------------------------------------------//
	{
    Eigen::Vector3d pitchbacklink_pitchbottomlinkPA = {     0.0,     0.0,     1.0 };
    Eigen::Vector3d pitchbacklink_pitchbottomlinkCA = {     0.0,     0.0,     1.0 };
    Eigen::Vector3d pitchbacklink_pitchbottomlinkPP = { -0.1028, -0.2867,     0.0 };
    Eigen::Vector3d pitchbacklink_pitchbottomlinkCP = { -0.0364,  0.0098, -0.0005 };
    pitchbacklink_pitchbottomlinkPA.normalize();
    pitchbacklink_pitchbottomlinkCA.normalize();

		Matrix3d pitchbacklink_pitchbottomlinkBodyRot = Eigen::Matrix3d(
			Eigen::Quaterniond::FromTwoVectors(pitchbacklink_pitchbottomlinkPA, pitchbacklink_pitchbottomlinkCA));
		Eigen::Affine3d pitchbacklink_pitchbottomlinkRotOffset(
			Eigen::AngleAxisd(pitchbacklink_pitchbottomlinkOffsetQ_, Vector3d::UnitZ()));
		std::cout << "pitchbacklink_pitchbottomlinkBodyRot" << std::endl << pitchbacklink_pitchbottomlinkBodyRot << std::endl;

		SpatialTransform pitchbacklink_pitchbottomlinkST;
		pitchbacklink_pitchbottomlinkST.E = pitchbacklink_pitchbottomlinkRotOffset.rotation() * 
			pitchbacklink_pitchbottomlinkBodyRot;
		pitchbacklink_pitchbottomlinkST.r = pitchbacklink_pitchbottomlinkPP - 
			(pitchbacklink_pitchbottomlinkBodyRot.inverse() * pitchbacklink_pitchbottomlinkCP);

		// world_pitchbottomlinkST_ = world_pitchbacklinkST_ * pitchbacklink_pitchbottomlinkST;
		Joint pitchbacklink_pitchbottomlinkJoint = 
			RigidBodyDynamics::Joint(JointTypeRevolute, 
			world_pitchbacklinkST_.E.transpose().block<3, 1>(0, 2).transpose());

		const Vector3d p_world_patchbottomlink =  world_pitchbacklinkST_.E * 
			pitchbacklink_pitchbottomlinkST.r;
		std::cout << "p_world_patchbottomlink" << std::endl << p_world_patchbottomlink << std::endl;

		pitchbacklink_pitchbottomlinkId_ = rbdlModel_->AddBody(yawlink_pitchbacklinkId_, 
		Xtrans(p_world_patchbottomlink), pitchbacklink_pitchbottomlinkJoint, 
		pitchbacklinkBody_, "pitchbacklink-pitchbottomlink");

		world_pitchbottomlinkST_ = world_pitchbacklinkST_ * pitchbacklink_pitchbottomlinkST;
		std::cout << std::endl << "-----------------end of pitchbacklink_pitchbottomlink-----------------" << std::endl;
	}
	//--------------------------------------------------------------------//
	// std::cout << "---------------------------" << std::endl;
	// std::cout << "baselink_yawlinkST_" << std::endl << baselink_yawlinkST_ << std::endl;
	// std::cout << "P_world_baselink" << std::endl << P_world_baselink << std::endl;
	// std::cout << "---------------------------" << std::endl;
	// std::cout << "yawlink_pitchbacklinkST_" << std::endl << baselink_yawlinkST_ << std::endl;
	// std::cout << "P_baselink_yawlink_base" << std::endl << P_baselink_yawlink_base << std::endl;
	// std::cout << "---------------------------" << std::endl;

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

