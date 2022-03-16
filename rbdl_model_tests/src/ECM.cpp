#include "rbdl_model_tests/ECM.h"
// #include "rbdl_model_tests/EigenUtilities.h"

ECM::ECM() 
{
	ConnectToAMBF();
	SetAMBFParams();
	ExecutePoseInAMBF();
  
	RotationQuaternionTF();
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
				tf::Quaternion(0., 0., 0., 0.),
				tf::Vector3(0., 0., 0.)
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
		AMBFParamsPtr ambfRigidBodyParams = ambfParamMap_[baselinkName_];

		rigidBodyPtr rigidBodyHandler = ambfParamMapItr_->second->RididBodyHandler();
		// const tf::Quaternion quat_w_n_tf_ambf = rigidBodyHandler->get_rot();
		// const tf::Vector3 P_w_n_tf_ambf = rigidBodyHandler->get_pos();
		ambfRigidBodyParams->RotationQuaternionTF(rigidBodyHandler->get_rot());
		ambfRigidBodyParams->TranslationVectorTF(rigidBodyHandler->get_pos());
	}
}


const Matrix3d ECM::RotationQuaternionTF()
{
	for(ambfParamMapItr_ = ambfParamMap_.begin(); ambfParamMapItr_ != ambfParamMap_.end(); ambfParamMapItr_++)
	{
		const std::string parentBody = ambfParamMapItr_->first;
		AMBFParamsPtr ambfRigidBodyParams = ambfParamMap_[baselinkName_];

		tf::Quaternion quat_w_n_tf = ambfRigidBodyParams->RotationQuaternionTF();
		Quaternion quat_w_n = EigenUtilities::TFtoEigenQuaternion(quat_w_n_tf);
		Matrix3d r_w_n = quat_w_n.toMatrix();

		tf::Vector3 p_w_n_tf = ambfRigidBodyParams->TranslationVectorTF();
		Vector3d p_w_n = EigenUtilities::TFtoEigenVector(p_w_n_tf);

		std::cout << "parentBody: " << parentBody << std::endl;
		std::cout << "r_w_n" << std::endl << r_w_n << std::endl;
		std::cout << "p_w_n" << std::endl << p_w_n << std::endl;

		std::cout << "---------------------------" << std::endl;
	}

	Matrix3d dummy;
	return dummy;
}


void ECM::SetBodyParams()
{
	// mass, com - inertia offset, inertia
	baseLinkBody_          = Body(01.000, Vector3d(-0.0046, 00.0000, 00.0801), 
																				Vector3d(00.0000, 00.0000, 00.0000));
	yawLinkBody_           = Body(06.417, Vector3d(00.0000, -0.0161, 00.1345), 
																				Vector3d(00.2978, 00.3125, 00.0449));
	pitchBackLinkBody_     = Body(00.421, Vector3d(-0.0515, -0.1434, -0.0090), 
																				Vector3d(00.0236, 00.0028, 00.0261));
	pitchBottomLinkBody_   = Body(00.359, Vector3d(00.1491, -0.0182, 00.0000), 
																				Vector3d(00.0007, 00.0190, 00.0192));
	mainInsertionLinkBody_ = Body(00.231, Vector3d(-0.0590, -0.0165, 00.0008), 
																				Vector3d(00.0003, 00.0015, 00.0016));
	toolLinkBody_          = Body(01.907, Vector3d(00.0000, -0.0008, 00.0723), 
																				Vector3d(00.0457, 00.0455, 00.0017));
	pitchFrontLinkBody_    = Body(01.607, Vector3d(-0.0365, -0.1526, 00.0000), 
																				Vector3d(00.0983, 00.0175, 00.1099));
	pitchTopLinkBody_      = Body(00.439, Vector3d(00.1702, -0.0001, 00.0008), 
																				Vector3d(00.0000, 00.0381, 00.0381));
	// No Joint for pittchEndLink as its a p2p joint
	pitchEndLinkBody_      = Body(02.032, Vector3d(00.0513, 00.0048, 00.0008), 
																				Vector3d(00.0636, 00.0099, 00.0726));

	Q_     = VectorNd::Constant ((size_t) rbdlModel_->dof_count, 0.);
	QDot_  = VectorNd::Constant ((size_t) rbdlModel_->dof_count, 0.);
	QDDot_ = VectorNd::Constant ((size_t) rbdlModel_->dof_count, 0.);
	Tau_   = VectorNd::Constant ((size_t) rbdlModel_->dof_count, 0.); 

	ClearLogOutput();
}

void ECM::CreateRBDLModel()
{
	SetBodyParams();
	rbdlModel_ = new Model;
	rbdlModel_->gravity = Vector3d(0., 0., -9.81);

	Matrix3d base;
	base.setIdentity();
	//--------------------------------------------------------------------//
	Vector3d baseLink_yawLinkPA = { -0.0002, -1.0000, 00.0000 };
	Vector3d baseLink_yawLinkCA = { 00.0000, 00.0000, -1.0000 };
	Vector3d baseLink_yawLinkPP = { 00.0000, 00.0000, 00.0000 };
	Vector3d baseLink_yawLinkCP = { 00.0001, 00.0000, 00.5369 };
	baseLink_yawLinkPA.normalize();
	baseLink_yawLinkCA.normalize();

	Matrix3d baseLink_yawLinkRot = Eigen::Matrix3d(
		Eigen::Quaterniond::FromTwoVectors(baseLink_yawLinkPA, baseLink_yawLinkCA));
	Eigen::Affine3d baseLink_yawLink_offset(Eigen::AngleAxisd(baseLink_yawLinkOffset_, Vector3d::UnitZ()));

	baseLink_yawLinkST_.E = baseLink_yawLink_offset.rotation() * baseLink_yawLinkRot;
	baseLink_yawLinkST_.r = baseLink_yawLinkPP - (baseLink_yawLinkRot.inverse() * baseLink_yawLinkCP);

	baseLink_yawLinkJoint_ = Joint(JointTypeRevolute, 
		baseLink_yawLinkST_.E.transpose().block<3, 1>(0, 2).transpose());
	
	const Vector3d P_baseLink_yawLink_base =  base * baseLink_yawLinkST_.r;
	baseLink_yawLinkID_ = rbdlModel_->AddBody(0, Xtrans(P_baseLink_yawLink_base), 
		baseLink_yawLinkJoint_, baseLinkBody_, "baseLink-yawLink");
}


// const Vector3d TranslationVectorTF();

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

