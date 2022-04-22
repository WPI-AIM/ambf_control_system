#include "rbdl_model_tests/ECM.h"

ECM::ECM() 
{
	ConnectToAMBF();
	SetAMBFParams();
	MapAMBFJointsToParent();
	SetBodyParams();
	CreateRBDLModel();
	ExecutePoseInAMBF();
  
	PrintAMBFTransformation();

	// SetRBDLPose();
	// CheckRBDLModel();
	
	// CleanUp();
}

void ECM::ConnectToAMBF()
{
	ambfClientPtr_ = RBDLTestPrep::getInstance()->getAMBFClientInstance();
	ambfClientPtr_->connect();
	usleep(1000000);
}

void ECM::SetAMBFParams()
{
	std::vector<std::string> rigidBodyNames = ambfClientPtr_->getRigidBodyNames();
	for(std::string rigidBodyName : rigidBodyNames)
	{
		ambfParamMap_.insert(AMBFParamPair(rigidBodyName, new AMBFParams(
				rigidBodyName, 
				ambfClientPtr_->getRigidBody(rigidBodyName.c_str(), true)))
				);
	}
	usleep(250000);
	// printf("---------------------------------\n");
	// Initialize all the handlers
	for(ambfParamMapItr_ = ambfParamMap_.begin(); ambfParamMapItr_ != ambfParamMap_.end(); ambfParamMapItr_++)
	{
		rigidBodyPtr handler = ambfParamMapItr_->second->RididBodyHandler();
				
		// Activate the rigid body if not active
		if(!handler->is_active())
		{
			handler->set_active();
		}
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
}

void ECM::MapJoints(const std::string& parentBody, rigidBodyPtr handler)
{
	std::vector<std::string> jointNames = handler->get_joint_names();
	for(std::string jointName : jointNames)
	{
		// AMBF parent has been identified already so skip updating it again
		// More than one AMBF rigid body can control the same joint.
		jointValuesMapItr_ = jointValuesMap_.find(jointName);
		if(jointValuesMapItr_ != jointValuesMap_.end()) continue;
		jointValuesMap_.insert(JointValuesPair(jointName, new JointValues{ parentBody, 0.0f }));
	}
}

void ECM::MapAMBFJointsToParent()
{
	// Iterate through all ambf handles and their joints. Find the ambf rigid body parent for all the 
	// RBDL joints. If no AMBF parent is found a given RBDL joint, its q desired cannot be found. 
	
	// Handle as many joints possible with baselink
	MapJoints(baselinkName_, ambfParamMap_[baselinkName_]->RididBodyHandler());

	for(ambfParamMapItr_ = ambfParamMap_.begin(); ambfParamMapItr_ != ambfParamMap_.end(); ambfParamMapItr_++)
	{
		const std::string parentBody = ambfParamMapItr_->first;
		rigidBodyPtr rigidBodyHandler = ambfParamMapItr_->second->RididBodyHandler();
		MapJoints(parentBody, rigidBodyHandler);
	}

	for(jointValuesMapItr_ = jointValuesMap_.begin(); 
			jointValuesMapItr_ != jointValuesMap_.end();
			jointValuesMapItr_++)
	{
		std::string jointName = jointValuesMapItr_->first;
		std::string parentName = jointValuesMapItr_->second->parent;

		printf("joint: %s, parent: %s\n", jointName.c_str(), parentName.c_str());
	}
}

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

	Vector3d roationAxis = world_parentST.E.block<3, 1>(0, 1).transpose();
	roationAxis.normalize();
	p_world_endingbody =  world_parentST.E * bodyST.r;
	newBodyId = rbdlModel_->AddBody(parentId, Xtrans(p_world_startingbody), joint, body, bodyName);
	world_bodyST = world_parentST * bodyST;

	// printf("%s\n", bodyName.c_str());
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
  //--------------------------------------------------------------------//
	// Virtual link
	Vector3d pitchbacklink_pitchbottomlinkPA = {     0.0,     0.0,     1.0 };
	Vector3d pitchbacklink_pitchbottomlinkCA = {     0.0,     0.0,     1.0 };
	Vector3d pitchbacklink_pitchbottomlinkPP = { -0.1028, -0.2867,     0.0 };
	Vector3d pitchbacklink_pitchbottomlinkCP = { -0.0364,  0.0098, -0.0005 };

	Vector3d p_world_pitchbottomlink	 			 = Vector3d::Zero();

	CreateRBDLJoint(pitchbacklink_pitchbottomlinkPA, pitchbacklink_pitchbottomlinkCA, 
	pitchbacklink_pitchbottomlinkPP, pitchbacklink_pitchbottomlinkCP, pitchbacklink_pitchbottomlinkOffsetQ, 
	Vector3d::UnitZ(), yawlink_pitchbacklinkId, p_world_pitchbacklink, 
	Joint(SpatialVector (-1., 0., 0., 0., 0., 0.)), 
	world_pitchbacklinkST, virtualBody_, "pitchbacklink-pitchbottomlink", 
	p_world_pitchbottomlink, pitchbacklink_pitchbottomlink_v_Id, world_pitchbottomlinkST);
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
	rbdlmBodyMap_ = rbdlModel_->mBodyNameMap;
  
	ClearLogOutput();

	for(rbdlmBodyMapItr_ = rbdlmBodyMap_.begin(); rbdlmBodyMapItr_ != rbdlmBodyMap_.end(); rbdlmBodyMapItr_++)
  {
    std::string bodyName = rbdlmBodyMapItr_->first;
    unsigned int bodyId = rbdlmBodyMapItr_->second;
    std::string parentName = rbdlModel_->GetBodyName(rbdlModel_->GetParentBodyId(bodyId));
    std::cout << parentName << ", " << bodyName << ", " << bodyId << std::endl;
  }
  std::cout << std::endl << "------------------" << std::endl;
}

void ECM::HelloThread()
{
	std::cout << "HelloThread\n";
}

void ECM::RegisterRigidBodysPose()
{
	for(ambfParamMapItr_ = ambfParamMap_.begin(); ambfParamMapItr_ != ambfParamMap_.end(); ambfParamMapItr_++)
	{
		const std::string parentBody = ambfParamMapItr_->first;
		AMBFParamsPtr rigidBodyParams = ambfParamMap_[parentBody];
		rigidBodyPtr rigidBodyHandler = ambfParamMapItr_->second->RididBodyHandler();

		tf::Quaternion quat_w_n_tf = rigidBodyHandler->get_rot();
		tf::Vector3 p_w_n_tf = rigidBodyHandler->get_pos();
		rigidBodyParams->QuaternionTF(quat_w_n_tf);
		rigidBodyParams->TranslationVectorTF(p_w_n_tf);
		ambfParamMap_[parentBody] = rigidBodyParams;
	}
}

void ECM::RegisterJointsAngle()
{
	for(jointValuesMapItr_ = jointValuesMap_.begin(); 
			jointValuesMapItr_ != jointValuesMap_.end(); 
			jointValuesMapItr_++)
	{
		const std::string jointName = jointValuesMapItr_->first;
		JointValuesPtr jointValuePtr = jointValuesMapItr_->second;

		const std::string parentName = jointValuesMap_[jointName]->parent;

		ambfParamMapItr_ = ambfParamMap_.find(parentName);
		if(ambfParamMapItr_ == ambfParamMap_.end()) continue;

		rigidBodyPtr handler = ambfParamMap_[parentName]->RididBodyHandler();
		jointValuePtr->qActual = handler->get_joint_pos<std::string>(jointName);

		jointValuesMap_[jointName] = jointValuePtr;
	}
}

void ECM::ExecutePoseInAMBF()
{
	AMBFParamsPtr baselinkParams = ambfParamMap_[baselinkName_];

	rigidBodyPtr baselinkHandler = baselinkParams->RididBodyHandler();
	// rigidBodyPtr baselinkHandler = ambfClientPtr_->getRigidBody(baselinkName_.c_str(), true);
	// usleep(250000);

	// std::string rigidBodyName = baselinkHandler->get_name();
	// std::cout << "Suppose to be base, check " << rigidBodyName << std::endl;

	std::vector<ControllableJointConfig> baselinkControllableJointConfigs = 
		baselinkParams->ControllableJointConfigs();

	int count = 0;
	float qDesired = M_PI_4;
	float sumOfAbsoluteDiff = 0.0f;
	do
	{
		/*** Running diff calculates the difference of desired and actual joint angles of all joints.
		 * Run the do while loop until all joints have reached the desired poses. That is the sume of
		 * absolute difference between actual and desired is zero for all joints. Absolute is used
		 * to make sure that positive difference dosent compensate for the negative difference. 
		 ***/  
		sumOfAbsoluteDiff = 1.0f;
		
		// To avoid infinte looping
		if(count > 20) return;

		// printf("sumOfAbsoluteDiff: %f\n", sumOfAbsoluteDiff);
		// for(ControllableJointConfig jointConfig : baselinkControllableJointConfigs)
		// {
		// 	baselinkHandler->set_joint_pos<std::string>(jointConfig.jointName.c_str(), qDesired);
		// }
		// usleep(sleepTime);
		baselinkHandler->set_joint_pos<std::string>(              "baselink-yawlink", qDesired);
		baselinkHandler->set_joint_pos<std::string>(         "yawlink-pitchbacklink", qDesired);
		baselinkHandler->set_joint_pos<std::string>("pitchendlink-maininsertionlink", 0.10);
		baselinkHandler->set_joint_pos<std::string>(    "maininsertionlink-toollink", qDesired);
		usleep(sleepTime);
		// // float qActual = baselinkHandler->get_joint_pos<std::string>(jointName.c_str());
		// sumOfAbsoluteDiff += abs(qDesired - qActual);
		// printf("jointName: %s, qActual: %f\n", jointName.c_str(), qActual);

		// float qAcutal1 = baselinkHandler->get_joint_pos<std::string>(              "baselink-yawlink");
		// float qAcutal2 = baselinkHandler->get_joint_pos<std::string>(         "yawlink-pitchbacklink");
		// float qAcutal3 = baselinkHandler->get_joint_pos<std::string>("pitchendlink-maininsertionlink");
		// float qAcutal4 = baselinkHandler->get_joint_pos<std::string>(    "maininsertionlink-toollink");
		// printf("qActual: %f, %f, %f, %f\n", qAcutal1, qAcutal2, qAcutal3, qAcutal4);
		// std::cout << sumOfAbsoluteDiff << "------------------" << std::endl;
		// RegisterRigidBodysPose();
		RegisterJointsAngle();
		count++;
	} while (sumOfAbsoluteDiff > 0.002);
	

/*
	// Get Joints rotation and pose
	// This is mearsured after the sleepTime completes. The robot pose could have changed by the
	// time below code gets executed. TODO: Explore multi theading to implement this. 
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
*/
}

const Matrix3d ECM::PrintAMBFTransformation()
{
	std::cout << "qActual from AMBF\n";
	for(jointValuesMapItr_ = jointValuesMap_.begin();
		  jointValuesMapItr_ != jointValuesMap_.end();
			jointValuesMapItr_++)
			{
				printf("%s, %f\n", jointValuesMapItr_->first.c_str(), jointValuesMapItr_->second->qActual);
			}
	std::cout << "-----------------\n";
	/*
	for(ambfParamMapItr_ = ambfParamMap_.begin(); ambfParamMapItr_ != ambfParamMap_.end(); ambfParamMapItr_++)
	{
		const std::string parentBody = ambfParamMapItr_->first;
		AMBFParamsPtr rigidBodyParams = ambfParamMap_[parentBody];
		rigidBodyPtr rigidBodyHandler = ambfParamMapItr_->second->RididBodyHandler();

		Matrix3d r_w_n_ambf = ambfParamMapItr_->second->RotationMatrix();
		Vector3d p_w_n_ambf = ambfParamMapItr_->second->TranslationVector();
		
		std::cout << "parentBody: " << parentBody << std::endl;
		std::cout << "r_w_n_ambf: " << std::endl << r_w_n_ambf << std::endl;
		std::cout << "p_w_n_ambf: " << std::endl << p_w_n_ambf << std::endl;
	}

	std::cout << "print q_Acutal from AMBF\n";
	for(jointValuesMapItr_ = jointValuesMap_.begin(); 
			jointValuesMapItr_ != jointValuesMap_.end(); 
			jointValuesMapItr_++)
	{
		const std::string jointName = jointValuesMapItr_->first;
		JointValuesPtr jointValuePtr = jointValuesMapItr_->second;

		printf("JointName: %s, qActualfromAMBF: %f\n", jointName.c_str(), jointValuePtr->qActual);
	}

	// t_0_w_.block<3, 3>(0, 0) = t_w_0_.block<3, 3>(0, 0).transpose();
	// t_0_w_.block<3, 1>(0, 3) = -t_w_0_.block<3, 3>(0, 0).transpose() * t_w_0_.block<3, 1>(0, 3);
	// std::cout << "t_0_w_" << std::endl << t_0_w_ << std::endl;
	*/

/*
	for(ambfParamMapItr_ = ambfParamMap_.begin(); ambfParamMapItr_ != ambfParamMap_.end(); ambfParamMapItr_++)
	{
		const std::string parentBody = ambfParamMapItr_->first;
		AMBFParamsPtr ambfRigidBodyParams = ambfParamMap_[parentBody];

		MatrixNd t_w_n = MatrixNd::Identity(4, 4);
		t_w_n.block<3, 3>(0, 0) = ambfRigidBodyParams->RotationMatrix();
		t_w_n.block<3, 1>(0, 3) = ambfRigidBodyParams->TranslationVector();
		
		MatrixNd t_0_n = MatrixNd::Identity(4, 4);
		t_0_n = t_0_w_ * t_w_n;

		// std::cout << "parentBody: " << parentBody << std::endl;
		// std::cout << "t_w_n" << std::endl << t_w_n << std::endl;
		// std::cout << "t_0_n" << std::endl << t_0_n << std::endl;
		// std::cout << "---------------------------" << std::endl;
	}
*/
	Matrix3d dummy;
	return dummy;
}

void ECM::SetRBDLPose()
{
	for(rbdlmBodyMapItr_ = rbdlmBodyMap_.begin(); rbdlmBodyMapItr_ != rbdlmBodyMap_.end(); rbdlmBodyMapItr_++)
	{
		std::string rbdlBodyName = rbdlmBodyMapItr_->first;
		unsigned int rbdlBodyId = rbdlmBodyMapItr_->second;

		printf("RBDL body: %s, RBDL Id: %d\n", rbdlBodyName.c_str(), rbdlBodyId);
	}
}

void ECM::CheckRBDLModel()
{
	// Iterate through RBDL body map, for each rbdl body, find the AMBF handler. Use the handler and find 
	// the q_desired in AMBF
  for(rbdlmBodyMapItr_ = rbdlmBodyMap_.begin(); rbdlmBodyMapItr_ != rbdlmBodyMap_.end(); rbdlmBodyMapItr_++)
  {
    std::string jointNameRBDL = rbdlmBodyMapItr_->first;
    unsigned int jointIDRBDL  = rbdlmBodyMapItr_->second;
    
    // const Vector3d P_0_n_rbdl = 
    //   CalcBodyToBaseCoordinates(*rbdlModel_, Q_, jointIDRBDL, Vector3d(0., 0., 0.), true);
		// std::cout << "P_0_n_rbdl: " << std::endl << P_0_n_rbdl << std::endl;
		// std::cout << "--------------------" << std::endl;

		// Skip the joint for which AMBF handler not found
		jointValuesMapItr_ = jointValuesMap_.find(jointNameRBDL);
		if(jointValuesMapItr_ == jointValuesMap_.end()) continue;
		std::string parentName = jointValuesMap_[jointNameRBDL]->parent;

		ambfParamMapItr_ = ambfParamMap_.find(parentName);
		if(ambfParamMapItr_ == ambfParamMap_.end()) continue;

		rigidBodyPtr rigidBodyHandler = ambfParamMapItr_->second->RididBodyHandler();

		float qDesiredambf = rigidBodyHandler->get_joint_pos<std::string>(jointNameRBDL);
		printf("jointNameRBDL: %s, qDesiredambf: %f\n", jointNameRBDL.c_str(), qDesiredambf);
  }  
  
}

void ECM::CleanUp()
{
	for(ambfParamMapItr_ = ambfParamMap_.begin(); ambfParamMapItr_ != ambfParamMap_.end(); ambfParamMapItr_++)
	{
		const std::string parentBody = ambfParamMapItr_->first;
		AMBFParamsPtr ambfRigidBodyParamsPtr = ambfParamMapItr_->second;
		rigidBodyPtr rigidBodyHandler = ambfRigidBodyParamsPtr->RididBodyHandler();

		std::cout << "Cleaning up Rigid body pointer for " << parentBody << std::endl;
		if(rigidBodyHandler == nullptr)
			printf("nullptr for rigidBodyName: %s\n", parentBody.c_str());

		rigidBodyHandler->cleanUp();
	}
}

ECM::~ECM() 
{
	CleanUp();
	ambfClientPtr_->cleanUp();
	delete rbdlModel_;
}