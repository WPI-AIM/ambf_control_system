#include "rbdl_model_tests/KUKA.h"

KUKA::KUKA() 
{
	ConnectToAMBF();
	SetAMBFParams();
	MapAMBFJointsToParent();
	SetBodyParams();
	CreateRBDLModel();
	// ExecutePoseInAMBF();
  
	// PrintAMBFTransformation();

	// SetRBDLPose();
	// CheckRBDLModel();
	
	CleanUp();
}

void KUKA::ConnectToAMBF()
{
	ambfClientPtr_ = RBDLTestPrep::getInstance()->getAMBFClientInstance();
	ambfClientPtr_->connect();
	usleep(1000000);

  rbdlModel_ = new Model;
  rbdlModel_->gravity = Vector3d(0., 0., -9.81);
}

void KUKA::RegisterBodyToWorldTransformation(const std::string parentBody)
{
	// const std::string parentBody = ambfParamMapItr_->first;
	AMBFParamsPtr rigidBodyParams = ambfParamMap_[parentBody];
	rigidBodyPtr rigidBodyHandler = rigidBodyParams->RididBodyHandler();

	tf::Quaternion quat_w_n_tf = rigidBodyHandler->get_rot();
	tf::Vector3 p_w_n_tf = rigidBodyHandler->get_pos();
	rigidBodyParams->QuaternionTF(quat_w_n_tf);
	rigidBodyParams->TranslationVectorTF(p_w_n_tf);
	ambfParamMap_[parentBody] = rigidBodyParams;
}

void KUKA::SetAMBFParams()
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
			{ "base", -1.595, 1.595 },
			{ "link1", -0.784, 1.158 },
			{ "link2", 0.0000, 0.254 },
			{ "link3", -1.553, 1.567 },
      { "link4", -1.553, 1.567 },
      { "link5", -1.553, 1.567 },
      { "link6", -1.553, 1.567 },
      { "link7", -1.553, 1.567 },
		}
	);

	RegisterBodyToWorldTransformation(baselinkName_);
}

void KUKA::MapJoints(const std::string& parentBody, rigidBodyPtr handler)
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

void KUKA::MapAMBFJointsToParent()
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

void KUKA::SetBodyParams()
{
  // mass, com - inertia offset, inertia
  baseBody_  = Body (1., Vector3d (00.001, 00.000, 0.060), Vector3d (0.0000, 0.0000, 0.0000));
  link1Body_ = Body (1., Vector3d (00.000, -0.017, 0.134), Vector3d (0.0452, 0.0446, 0.0041));
  link2Body_ = Body (1., Vector3d (00.000, -0.074, 0.009), Vector3d (0.0227, 0.0037, 0.0224));
  link3Body_ = Body (1., Vector3d (00.000, 00.017, 0.134), Vector3d (0.0417, 0.0418, 0.0038));
  link4Body_ = Body (1., Vector3d (-0.001, 00.081, 0.008), Vector3d (0.0249, 0.0036, 0.0247));
  link5Body_ = Body (1., Vector3d (0.0000, -0.017, 0.129), Vector3d (0.0363, 0.0350, 0.0045));
  link6Body_ = Body (1., Vector3d (0.0000, 00.007, 0.068), Vector3d (0.0114, 0.0116, 0.0037));
  link7Body_ = Body (1., Vector3d (0.0060, 00.000, 0.015), Vector3d (0.0012, 0.0013, 0.0010));
	
	Vector3d vector3d_zero = Vector3d::Zero();
}

void KUKA::CreateRBDLJoint(Vector3d& pa, Vector3d& ca, Vector3d& pp, Vector3d& cp, const double offsetQ, 
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

void KUKA::CreateRBDLModel()
{
  unsigned int base_link1Id, link1_link2Id, link2_link3Id, link3_link4Id, link4_link5Id, link5_link6Id, 
    link6_link7Id, link7_eeId;

  SpatialTransform world_baseST, world_link1ST, world_link2ST, world_link3ST, 
	world_link4ST, world_link5ST, world_link6ST, world_link7ST, world_eeST;

  const double ROOT_baseOffsetQ   = 0.0;
  const double base_link1OffsetQ  = 0.0;
  const double link1_link2OffsetQ = 0.0;
  const double link2_link3OffsetQ = 0.0;
  const double link3_link4OffsetQ = 0.0;
  const double link4_link5OffsetQ = 0.0;
  const double link5_link6OffsetQ = 0.0;
  const double link6_link7OffsetQ = 0.0;
	const double link7_eeOffsetQ    = 0.0;

	SetBodyParams();
	
	rbdlModel_->gravity = Vector3d(0., 0., -9.81);

	// Register Word to Base Transform
	AMBFParamsPtr ambfRigidBodyParams = ambfParamMap_[baselinkName_];

	world_baseST.E = ambfRigidBodyParams->RotationMatrix();
	world_baseST.r.setZero();
	Vector3d p_world_base  = ambfRigidBodyParams->TranslationVector();
	//1--------------------------------------------------------------------//
  Vector3d base_link1PA = { 00.000, 00.000, 01.000 };
  Vector3d base_link1CA = { 00.000, 00.000, 01.000 };
  Vector3d base_link1PP = { 00.000, 00.000, 00.103 };
  Vector3d base_link1CP = { 00.000, 00.000, 00.000 };

	Vector3d p_world_link1 = Vector3d::Zero();
	
	CreateRBDLJoint(base_link1PA, base_link1CA, base_link1PP, base_link1CP, 
	base_link1OffsetQ, Vector3d::UnitZ(), 0, p_world_base, Joint(SpatialVector (0., 0., 1., 0., 0., 0.)),
	world_baseST, baseBody_, "base-link1", p_world_link1, base_link1Id, 
	world_link1ST);
// 	//2 and 3 --------------------------------------------------------------------//
// 	Vector3d yawlink_pitchfrontlinkPA = { 1.0, 0.0,   0.0 };
// 	Vector3d yawlink_pitchfrontlinkCA = { 0.0, 0.0,   1.0 };
// 	Vector3d yawlink_pitchfrontlinkPP = { 0.0, 0.0, 0.199 };
// 	Vector3d yawlink_pitchfrontlinkCP = { 0.0, 0.0,   0.0 };

// 	Vector3d p_world_pitchfrontlink	 	= Vector3d::Zero();

// 	CreateRBDLJoint(yawlink_pitchfrontlinkPA, yawlink_pitchfrontlinkCA, yawlink_pitchfrontlinkPP, 
// 	yawlink_pitchfrontlinkCP, yawlink_pitchfrontlinkOffsetQ, Vector3d::UnitZ(), baselink_yawlinkId, 
// 	p_world_yawlink, Joint(SpatialVector (-1., 0., 0., 0., 0., 0.)), world_yawlinkST, yawlinkBody_, 
// 	"yawlink-pitchfrontlink", p_world_pitchfrontlink, yawlink_pitchfrontlinkId, world_pitchfrontlinkST);
// 	//3--------------------------------------------------------------------//
// 	Vector3d yawlink_pitchbacklinkPA = { 1.0,     0.0,    0.0 };
// 	Vector3d yawlink_pitchbacklinkCA = { 0.0,     0.0,    1.0 };
// 	Vector3d yawlink_pitchbacklinkPP = { 0.0, -0.0098, 0.1624 };
// 	Vector3d yawlink_pitchbacklinkCP = { 0.0,     0.0,    0.0 };

// 	Vector3d p_world_pitchbacklink	 = Vector3d::Zero();

// 	CreateRBDLJoint(yawlink_pitchbacklinkPA, yawlink_pitchbacklinkCA, yawlink_pitchbacklinkPP, 
// 	yawlink_pitchbacklinkCP, yawlink_pitchbacklinkOffsetQ, Vector3d::UnitZ(), baselink_yawlinkId, 
// 	p_world_yawlink, Joint(SpatialVector (-1., 0., 0., 0., 0., 0.)), world_yawlinkST, yawlinkBody_, 
// 	"yawlink-pitchbacklink", p_world_pitchbacklink, yawlink_pitchbacklinkId, world_pitchbacklinkST);
//   //--------------------------------------------------------------------//
// 	// Virtual link
// 	Vector3d pitchbacklink_pitchbottomlinkPA = {     0.0,     0.0,     1.0 };
// 	Vector3d pitchbacklink_pitchbottomlinkCA = {     0.0,     0.0,     1.0 };
// 	Vector3d pitchbacklink_pitchbottomlinkPP = { -0.1028, -0.2867,     0.0 };
// 	Vector3d pitchbacklink_pitchbottomlinkCP = { -0.0364,  0.0098, -0.0005 };

// 	Vector3d p_world_pitchbottomlink	 			 = Vector3d::Zero();

// 	CreateRBDLJoint(pitchbacklink_pitchbottomlinkPA, pitchbacklink_pitchbottomlinkCA, 
// 	pitchbacklink_pitchbottomlinkPP, pitchbacklink_pitchbottomlinkCP, pitchbacklink_pitchbottomlinkOffsetQ, 
// 	Vector3d::UnitZ(), yawlink_pitchbacklinkId, p_world_pitchbacklink, 
// 	Joint(SpatialVector (-1., 0., 0., 0., 0., 0.)), 
// 	world_pitchbacklinkST, virtualBody_, "pitchbacklink-pitchbottomlink", 
// 	p_world_pitchbottomlink, pitchbacklink_pitchbottomlink_v_Id, world_pitchbottomlinkST);
// 	//8--------------------------------------------------------------------//
// 	Vector3d pitchbottomlink_pitchendlinkPA = {    0.0,     0.0,     1.0 };
// 	Vector3d pitchbottomlink_pitchendlinkCA = {    0.0,     0.0,     1.0 };
// 	Vector3d pitchbottomlink_pitchendlinkPP = { 0.3401, -0.0001, -0.0005 };
// 	Vector3d pitchbottomlink_pitchendlinkCP = {    0.0,     0.0,  0.0001 };
// 	Vector3d p_world_pitchendlink	 	 			 	= Vector3d::Zero();

// 	CreateRBDLJoint(pitchbottomlink_pitchendlinkPA, pitchbottomlink_pitchendlinkCA, 
// 	pitchbottomlink_pitchendlinkPP, pitchbottomlink_pitchendlinkCP, pitchbottomlink_pitchendlinkOffsetQ, 
// 	Vector3d::UnitZ(), pitchbacklink_pitchbottomlink_v_Id, p_world_pitchbottomlink, 
// 	Joint(SpatialVector (-1., 0., 0., 0., 0., 0.)), 
// 	world_pitchbottomlinkST, pitchbottomlinkBody_, "pitchbottomlink-pitchendlink", 
// 	p_world_pitchendlink, pitchbottomlink_pitchendlinkId, world_pitchendlinkST);
// 	//9--------------------------------------------------------------------//
// 	Vector3d pitchendlink_maininsertionlinkPA = {     0.0,     1.0,    0.0 };
// 	Vector3d pitchendlink_maininsertionlinkCA = {     1.0,     0.0,    0.0 };
// 	Vector3d pitchendlink_maininsertionlinkPP = {  0.1031, -0.0961, 0.0001 };
// 	Vector3d pitchendlink_maininsertionlinkCP = { -0.0108,  -0.062, 0.0002 };
// 	Vector3d p_world_maininsertionlink	 		 	= Vector3d::Zero();

// 	CreateRBDLJoint(pitchendlink_maininsertionlinkPA, pitchendlink_maininsertionlinkCA, 
// 	pitchendlink_maininsertionlinkPP, pitchendlink_maininsertionlinkCP, pitchendlink_maininsertionlinkOffsetQ, 
// 	-Vector3d::UnitZ(), pitchbottomlink_pitchendlinkId, p_world_pitchendlink, 
// 	Joint(SpatialVector (0., 0., 0., 0., 0., -1.)),
// 	world_pitchendlinkST, pitchendlinkBody_, "pitchendlink-maininsertionlink", 
// 	p_world_maininsertionlink, pitchendlink_maininsertionlinkId, world_maininsertionlinkST);
// 	//10--------------------------------------------------------------------//
// 	Vector3d maininsertionlink_toollinkPA = {     1.0,     0.0,    0.0 };
// 	Vector3d maininsertionlink_toollinkCA = {     0.0,     0.0,   -1.0 };
// 	Vector3d maininsertionlink_toollinkPP = { -0.0108,  -0.062,    0.0 };
// 	Vector3d maininsertionlink_toollinkCP = { -0.0001, -0.0002, 0.0118 };
// 	Vector3d p_world_toollink	 		 			  = Vector3d::Zero();

// 	CreateRBDLJoint(maininsertionlink_toollinkPA, maininsertionlink_toollinkCA, 
// 	maininsertionlink_toollinkPP, maininsertionlink_toollinkCP, maininsertionlink_toollinkOffsetQ, 
// 	Vector3d::UnitZ(), pitchendlink_maininsertionlinkId, p_world_maininsertionlink, 
// 	Joint(SpatialVector (0., 0., 1., 0., 0., 0.)),
// 	world_maininsertionlinkST, maininsertionlinkBody_, "maininsertionlink-toollink", 
// 	p_world_toollink, maininsertionlink_toollinkId, world_toollinkST);
// //11--------------------------------------------------------------------//
// 	Vector3d toollink_eePA = { 0.0, 0.0, 1.0 };
// 	Vector3d toollink_eeCA = { 0.0, 0.0, 1.0 };
// 	Vector3d toollink_eePP = Vector3d::Zero();
// 	Vector3d toollink_eeCP = Vector3d::Zero();
// 	Vector3d p_world_ee	 	 = Vector3d::Zero();

// 	CreateRBDLJoint(toollink_eePA, toollink_eeCA, 
// 	toollink_eePP, toollink_eeCP, toollink_eeOffsetQ, 
// 	Vector3d::UnitZ(), maininsertionlink_toollinkId, p_world_toollink, 
// 	Joint(SpatialVector (0., 0., 0., 0., 0., 0.)),
// 	world_toollinkST, toollinkBody_, "toollink_ee", 
// 	p_world_ee, toollink_eeId, world_eeST);
	//--------------------------------------------------------------------//

	Q_     = VectorNd::Constant ((size_t) rbdlModel_->dof_count, 0.);
	QDot_  = VectorNd::Constant ((size_t) rbdlModel_->dof_count, 0.);
	QDDot_ = VectorNd::Constant ((size_t) rbdlModel_->dof_count, 0.);
	Tau_   = VectorNd::Constant ((size_t) rbdlModel_->dof_count, 0.); 
	rbdlmBodyMap_ = rbdlModel_->mBodyNameMap;
  
	ClearLogOutput();

  PrintRBDLModel();
}

void KUKA::PrintRBDLModel()
{
  std::cout << "Printing RBDL Model\n";

	for(rbdlmBodyMapItr_ = rbdlmBodyMap_.begin(); rbdlmBodyMapItr_ != rbdlmBodyMap_.end(); rbdlmBodyMapItr_++)
  {
    std::string bodyName = rbdlmBodyMapItr_->first;
    unsigned int bodyId = rbdlmBodyMapItr_->second;
    std::string parentName = rbdlModel_->GetBodyName(rbdlModel_->GetParentBodyId(bodyId));
    std::cout << parentName << ", " << bodyName << ", " << bodyId << std::endl;
  }
  
  std::cout << std::endl << "------------------" << std::endl;
}

void KUKA::HelloThread()
{
	std::cout << "HelloThread\n";
}

void KUKA::RegisterAllRigidBodyPose()
{
	for(ambfParamMapItr_ = ambfParamMap_.begin(); ambfParamMapItr_ != ambfParamMap_.end(); ambfParamMapItr_++)
	{
		const std::string parentBody = ambfParamMapItr_->first;
		RegisterBodyToWorldTransformation(parentBody);
	}
}

void KUKA::ExecutePoseInAMBF()
{
	AMBFParamsPtr baselinkParams = ambfParamMap_[baselinkName_];
	rigidBodyPtr baselinkHandler = baselinkParams->RididBodyHandler();
	// printf("Q_.size: %ld\n", Q_.size());

  for(int i = 0; i < 10; i++)
  {
    for(rbdlmBodyMapItr_ = rbdlmBodyMap_.begin(); rbdlmBodyMapItr_ != rbdlmBodyMap_.end(); rbdlmBodyMapItr_++)
    {
      std::string jointName = rbdlmBodyMapItr_->first;
      unsigned int jointId = rbdlmBodyMapItr_->second;
			// if(jointName.compare("ROOT") == 0) continue;
			if(jointId == 0) continue;
			// printf("jointName: %s, jointId: %d\n", jointName.c_str(), jointId);

      // Zero indexed
      float jointAngle = Q_[--jointId];
      baselinkHandler->set_joint_pos<std::string>( jointName.c_str(), jointAngle);
    }
    usleep(sleepTime);
    RegisterAllRigidBodyPose();
  }
}

void KUKA::ExecutePose(VectorNd Q)
{
	Q_ = Q;

	ExecutePoseInAMBF();
}

t_w_nPtr KUKA::twnFromModels(std::string jointName)
{
	jointValuesMapItr_ = jointValuesMap_.find(jointName);
	if(jointValuesMapItr_ == jointValuesMap_.end())
	{
		printf("Joint: %s not found in jointValuesMap\n", jointName.c_str());
		return nullptr;
	}

	// Get Parent name (rigidbody) of the joint.
	std::string parentName = jointValuesMap_[jointName]->parent;

	ambfParamMapItr_ = ambfParamMap_.find(parentName);
	if(ambfParamMapItr_ == ambfParamMap_.end())
	{
		printf("Rigid body: %s not found in ambfParamMap\n", parentName.c_str());
		return nullptr;
	}
	
	// Collect AMBF and RBDL Transformation Matrices
	t_w_nPtr t_w_nptr = new T_W_N();
	t_w_nptr->r_w_n_ambf = ambfParamMap_[parentName]->RotationMatrix();
	t_w_nptr->p_w_n_ambf = ambfParamMap_[parentName]->TranslationVector();
	
	ForwardDynamics(*rbdlModel_, Q_, QDot_, Tau_, QDDot_);
	unsigned int rbdlBodyId = rbdlModel_->GetBodyId(jointName.c_str());

	t_w_nptr->r_w_n_rbdl = CalcBodyWorldOrientation(*rbdlModel_, Q_, rbdlBodyId, true);
	t_w_nptr->p_w_n_rbdl = CalcBodyToBaseCoordinates(*rbdlModel_, Q_, rbdlBodyId, Vector3d(0., 0., 0.), true);
	return t_w_nptr;
}

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


const Matrix3d KUKA::PrintAMBFTransformation()
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

void KUKA::SetRBDLPose()
{
	for(rbdlmBodyMapItr_ = rbdlmBodyMap_.begin(); rbdlmBodyMapItr_ != rbdlmBodyMap_.end(); rbdlmBodyMapItr_++)
	{
		std::string rbdlBodyName = rbdlmBodyMapItr_->first;
		unsigned int rbdlBodyId = rbdlmBodyMapItr_->second;

		printf("RBDL body: %s, RBDL Id: %d\n", rbdlBodyName.c_str(), rbdlBodyId);
	}
}

void KUKA::CheckRBDLModel()
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

void KUKA::CleanUp()
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

KUKA::~KUKA() 
{
	CleanUp();
	ambfClientPtr_->cleanUp();
	delete rbdlModel_;
}