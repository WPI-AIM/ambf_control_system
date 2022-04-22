#include "rbdl_model_tests/KUKA.h"

KUKA::KUKA() 
{
	ConnectToAMBF();
	SetAMBFParams();
	MapAMBFJointsToParent();
	SetBodyParams();
	CreateRBDLModel();
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
	// printf("parent: %s, Translation: (%f, %f, %f)\n", parentBody.c_str(), p_w_n_tf[0], p_w_n_tf[1], p_w_n_tf[2]);
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
	t_w_0_.block<3, 3>(0, 0) = ambfParamMap_[baselinkName_]->RotationMatrix();
	t_w_0_.block<3, 1>(0, 3) = ambfParamMap_[baselinkName_]->TranslationVector();

	t_0_w_.block<3, 1>(0, 3) = -t_w_0_.block<3, 3>(0, 0).transpose() * t_w_0_.block<3, 1>(0, 3);
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

	// for(jointValuesMapItr_ = jointValuesMap_.begin(); 
	// 		jointValuesMapItr_ != jointValuesMap_.end();
	// 		jointValuesMapItr_++)
	// {
	// 	std::string jointName = jointValuesMapItr_->first;
	// 	std::string parentName = jointValuesMapItr_->second->parent;

	// 	printf("joint: %s, parent: %s\n", jointName.c_str(), parentName.c_str());
	// }
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

void KUKA::CreateRBDLJoint(Vector3d& pa, Vector3d& ca, const Vector3d& pp, const Vector3d& cp, const double offsetQ, 
	const Vector3d axis, const unsigned int parentId, const Joint joint, const SpatialTransform world_parentST, 
	const Body &body, const std::string bodyName, unsigned int& childId, SpatialTransform&	world_childST)
{
	pa.normalize();
	ca.normalize();

	Matrix3d bodyRot = Eigen::Matrix3d(Eigen::Quaterniond::FromTwoVectors(pa, ca));
	Eigen::Affine3d rotOffset(Eigen::AngleAxisd(offsetQ, axis));
		
	SpatialTransform bodyST;
	bodyST.E = rotOffset.rotation() * bodyRot;
	bodyST.r = pp - (bodyRot.inverse() * cp);

	world_childST = world_parentST * bodyST;

	Vector3d p_world_child =  world_parentST.E.inverse() * bodyST.r;
	if(parentId == 0) p_world_child = world_childST.r;

	childId = rbdlModel_->AddBody(parentId, Xtrans(p_world_child), joint, body, bodyName);
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

  Vector3d base_link1PA = { 00.000, 00.000, 01.000 };
  Vector3d base_link1CA = { 00.000, 00.000, 01.000 };
  Vector3d base_link1PP = { 00.000, 00.000, 00.103 };
  Vector3d base_link1CP = { 00.000, 00.000, 00.000 };

	Vector3d link1_link2PA = { 00.000, 01.000, 00.000 };
	Vector3d link1_link2CA = { 00.000, 00.000, 01.000 };
	Vector3d link1_link2PP = { 00.000, 00.013, 00.209 };
	Vector3d link1_link2CP = { 00.000, 00.000, 00.000 };

	Vector3d link2_link3PA = { 00.000, -1.000, 00.000 };
	Vector3d link2_link3CA = { 00.000, 00.000, 01.000 };
	Vector3d link2_link3PP = { 00.000, -0.194, -0.009 };
	Vector3d link2_link3CP = { 00.000, 00.000, 00.000 };

	Vector3d link3_link4PA = { 00.000, -1.000, 00.000 };
	Vector3d link3_link4CA = { 00.000, 00.000, 01.000 };
	Vector3d link3_link4PP = { 00.000, -0.013, 00.202 };
	Vector3d link3_link4CP = { 00.000, 00.000, 00.000 };

	Vector3d link4_link5PA = { 00.000, 01.000, 00.000 };
	Vector3d link4_link5CA = { 00.000, 00.000, 01.000 };
	Vector3d link4_link5PP = { -0.002, 00.202, -0.008 };
	Vector3d link4_link5CP = { 00.000, 00.000, 00.000 };
	
	Vector3d link5_link6PA = { 00.000, 01.000, 00.000 };
	Vector3d link5_link6CA = { 00.000, 00.000, 01.000 };
	Vector3d link5_link6PP = { 00.002, -0.052, 00.204 };
	Vector3d link5_link6CP = { 00.000, 00.000, 00.000 };
	
	Vector3d link6_link7PA = { 00.000, -1.000, 00.000 };
	Vector3d link6_link7CA = { 00.000, 00.000, 01.000 };
	Vector3d link6_link7PP = { -0.003, -0.050, 00.053 };
	Vector3d link6_link7CP = { 00.000, 00.000, 00.000 };

	Vector3d link7_eePA = { 0.0, 0.0, 1.0 };
	Vector3d link7_eeCA = { 0.0, 0.0, 1.0 };
	Vector3d link7_eePP = Vector3d::Zero();
	Vector3d link7_eeCP = Vector3d::Zero();
	
	Vector3d p_world_link1 = Vector3d::Zero();
	Vector3d p_world_link2 = Vector3d::Zero();
	Vector3d p_world_link3 = Vector3d::Zero();
	Vector3d p_world_link4 = Vector3d::Zero();
	Vector3d p_world_link5 = Vector3d::Zero();
	Vector3d p_world_link6 = Vector3d::Zero();
	Vector3d p_world_link7 = Vector3d::Zero();

	SetBodyParams();
	
	rbdlModel_->gravity = Vector3d(0., 0., -9.81);

	// Register Word to Base Transform
	AMBFParamsPtr ambfRigidBodyParams = ambfParamMap_[baselinkName_];

	world_baseST.E = ambfRigidBodyParams->RotationMatrix();
	world_baseST.r = ambfRigidBodyParams->TranslationVector();

	//1--------------------------------------------------------------------//
	CreateRBDLJoint(base_link1PA, base_link1CA, base_link1PP, base_link1CP, base_link1OffsetQ, 
	Vector3d::UnitZ(), 0, Joint(SpatialVector (0., 0., 1., 0., 0., 0.)), 
	world_baseST, baseBody_, "base-link1", base_link1Id, world_link1ST);
	//--------------------------------------------------------------------//
	CreateRBDLJoint(link1_link2PA, link1_link2CA, link1_link2PP, link1_link2CP, link1_link2OffsetQ, 
	Vector3d::UnitZ(), base_link1Id, Joint(SpatialVector (0., 1., 0., 0., 0., 0.)), 
	world_link1ST, link1Body_, "link1-link2", link1_link2Id, world_link2ST);
	// //--------------------------------------------------------------------//
	// CreateRBDLJoint(link2_link3PA, link2_link3CA, link2_link3PP, link2_link3CP, link2_link3OffsetQ, 
	// Vector3d::UnitZ(), link1_link2Id, Joint(SpatialVector (0., 0., 1., 0., 0., 0.)), 
	// world_link2ST, link2Body_, "link2-link3", link2_link3Id, world_link3ST);
	// //--------------------------------------------------------------------//
	// CreateRBDLJoint(link3_link4PA, link3_link4CA, link3_link4PP, link3_link4CP, link3_link4OffsetQ, 
	// Vector3d::UnitZ(), link2_link3Id, Joint(SpatialVector (0., -1., 0., 0., 0., 0.)), 
	// world_link3ST, link3Body_, "link3-link4", link3_link4Id, world_link4ST);
	// //--------------------------------------------------------------------//
	// CreateRBDLJoint(link4_link5PA, link4_link5CA, link4_link5PP, link4_link5CP, link4_link5OffsetQ, 
	// Vector3d::UnitZ(), link3_link4Id, Joint(SpatialVector (0., 0., 1., 0., 0., 0.)), 
	// world_link4ST, link4Body_, "link4-link5", link4_link5Id, world_link5ST);
	// //--------------------------------------------------------------------//
	// CreateRBDLJoint(link5_link6PA, link5_link6CA, link5_link6PP, link5_link6CP, link5_link6OffsetQ, 
	// Vector3d::UnitZ(), link4_link5Id, Joint(SpatialVector (0., 1., 0., 0., 0., 0.)), 
	// world_link5ST, link5Body_, "link5-link6", link5_link6Id, world_link6ST);
	// //--------------------------------------------------------------------//
	// CreateRBDLJoint(link6_link7PA, link6_link7CA, link6_link7PP, link6_link7CP, link6_link7OffsetQ, 
	// Vector3d::UnitZ(), link5_link6Id, Joint(SpatialVector (0., 0., 1., 0., 0., 0.)), 
	// world_link6ST, link6Body_, "link6-link7", link6_link7Id, world_link7ST);
	// //--------------------------------------------------------------------//

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
	// std::cout << "--------------\n";
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
			// ROOT or joint7-ee
			// if(jointId == 0 || jointId == Q_.size()) continue;
			if(jointId == 0) continue;
			// printf("jointName: %s, jointId: %d\n", jointName.c_str(), jointId);

      // Zero indexed
      float jointAngle = Q_[--jointId];
			// printf("ExecutePoseInAMBF(): jointName: %s, jointAngle: %f\n", jointName.c_str(), jointAngle);
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
	// std::cout << "jointValuesMap\n";
	// for(jointValuesMapItr_ = jointValuesMap_.begin(); jointValuesMapItr_ != jointValuesMap_.end();
	// jointValuesMapItr_++)
	// {
	// 	printf("joint: %s, parent: %s\n", 
	// 		jointValuesMapItr_->first.c_str(), jointValuesMapItr_->second->parent.c_str());
	// }
	// std::cout << "-------------\n";

	// jointValuesMapItr_ = jointValuesMap_.find(jointName);
	// if(jointValuesMapItr_ == jointValuesMap_.end())
	// {
	// 	printf("Joint: %s not found in jointValuesMap\n", jointName.c_str());
	// 	return nullptr;
	// }

	// // Get Parent name (rigidbody) of the joint.
	// std::string parentName = jointValuesMap_[jointName]->parent;

	// joint_ name: link1-link2, parentName: link1
	t_w_nPtr t_w_nptr = new T_W_N();
	size_t npos = jointName.find("-", 0);
	if(npos == -1) return t_w_nptr;

	const std::string parentName = jointName.substr(npos + 1, jointName.size());


	ambfParamMapItr_ = ambfParamMap_.find(parentName);
	if(ambfParamMapItr_ == ambfParamMap_.end())
	{
		printf("Rigid body: %s not found in ambfParamMap\n", parentName.c_str());
		return nullptr;
	}
	
	// Collect AMBF and RBDL Transformation Matrices
	t_w_nptr->r_w_n_ambf = ambfParamMap_[parentName]->RotationMatrix();
	t_w_nptr->p_w_n_ambf = ambfParamMap_[parentName]->TranslationVector();

	ForwardDynamics(*rbdlModel_, Q_, QDot_, Tau_, QDDot_);
	unsigned int rbdlBodyId = rbdlModel_->GetBodyId(jointName.c_str());

	t_w_nptr->r_w_n_rbdl = CalcBodyWorldOrientation(*rbdlModel_, Q_, rbdlBodyId, true);
	t_w_nptr->p_w_n_rbdl = CalcBodyToBaseCoordinates(*rbdlModel_, Q_, rbdlBodyId, Vector3d(0., 0., 0.), true);
	
	return t_w_nptr;
}

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

		if(rigidBodyHandler->is_active()) rigidBodyHandler->cleanUp();
	}
}

KUKA::~KUKA() 
{
	if(ambfClientPtr_ != nullptr) 
	{
		std::cout << "insideambfClientPtr_ != nullptr\n";
		CleanUp();
		ambfClientPtr_->cleanUp();
	}
	delete rbdlModel_;
}