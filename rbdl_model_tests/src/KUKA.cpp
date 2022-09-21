#include "rbdl_model_tests/KUKA.h"


#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <algorithm>

KUKA::KUKA() 
{
	try
	{
		if(!ConnectToAMBF()) throw "AMBF Inactive Exception";
	}
	catch(const char* e)
	{
		std::exit(EXIT_FAILURE); 		
	}
	
	SetAMBFParams();
	SetBodyParams();
	CreateRBDLModel();
}

bool KUKA::ConnectToAMBF()
{
	ambfClientPtr_ = new Client();
	
	if(!ambfClientPtr_->connect()) return false;
	usleep(1000000);

	return true;
}




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
		// handler->set_joint_pos<int>(0, 0.0f);
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
	worldBody_ = Body (0.0000001, Vector3d (0.0000, 0.0000, 0.000), Vector3d (0.0000, 0.0000, 0.0000));
  baseBody_  = Body (1., Vector3d (00.001, 00.000, 0.060), Vector3d (0.0000, 0.0000, 0.0000));
  link1Body_ = Body (1., Vector3d (00.000, -0.017, 0.134), Vector3d (0.0452, 0.0446, 0.0041));
  link2Body_ = Body (1., Vector3d (00.000, -0.074, 0.009), Vector3d (0.0227, 0.0037, 0.0224));
  link3Body_ = Body (1., Vector3d (00.000, 00.017, 0.134), Vector3d (0.0417, 0.0418, 0.0038));
  link4Body_ = Body (1., Vector3d (-0.001, 00.081, 0.008), Vector3d (0.0249, 0.0036, 0.0247));
  link5Body_ = Body (1., Vector3d (0.0000, -0.017, 0.129), Vector3d (0.0363, 0.0350, 0.0045));
  link6Body_ = Body (1., Vector3d (0.0000, 00.007, 0.068), Vector3d (0.0114, 0.0116, 0.0037));
  link7Body_ = Body (1., Vector3d (0.0060, 00.000, 0.015), Vector3d (0.0012, 0.0013, 0.0010));
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
  unsigned int world_base_rollId, world_base_pitchId, world_base_yawId, worldId, world_baseId, base_link1Id, 
	link1_link2Id, link2_link3Id, link3_link4Id, link4_link5Id, link5_link6Id, link6_link7Id, link7_eeId;

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

	SetBodyParams();
	
	rbdlModel_->gravity = Vector3d(0., 0., -9.81);

	// Register Word to Base Transform
	AMBFParamsPtr ambfRigidBodyParams = ambfParamMap_[baselinkName_];
	//0---------------------------------------------------------------------//
	world_baseST.E = ambfRigidBodyParams->RotationMatrix();
	world_baseST.r = ambfRigidBodyParams->TranslationVector();

	// This is to handle initial World to body rotation.
	world_baseId = rbdlModel_->
	AddBody(0, Xtrans(world_baseST.r), Joint(JointTypeEulerZYX), 
	baseBody_, "world-base");
	//1--------------------------------------------------------------------//
	base_link1PA.normalize();
	base_link1CA.normalize();

	Matrix3d base_link1Rot = Eigen::Matrix3d(Eigen::Quaterniond::FromTwoVectors(base_link1PA, base_link1CA));
	Eigen::Affine3d base_link1rotOffset(Eigen::AngleAxisd(base_link1OffsetQ, Vector3d::UnitZ()));
		
	SpatialTransform base_link1ST;
	base_link1ST.E = base_link1rotOffset.rotation() * base_link1Rot;
	base_link1ST.r = base_link1PP - (base_link1Rot.inverse() * base_link1CP);

	Vector3d pWorld_base_link1 = world_baseST.E.transpose() * base_link1ST.r;
	base_link1Id = rbdlModel_->
		AddBody(world_baseId, Xtrans(pWorld_base_link1), Joint(SpatialVector (0., 0., 1., 0., 0., 0.)), 
			link1Body_, "base-link1");

	world_link1ST = world_baseST * base_link1ST;
	// Works
	// CreateRBDLJoint(base_link1PA, base_link1CA, base_link1PP, base_link1CP, base_link1OffsetQ, 
	// Vector3d::UnitZ(), 0, Joint(SpatialVector (0., 0., 1., 0., 0., 0.)), 
	// world_baseST, baseBody_, "base-link1", base_link1Id, world_link1ST);
	//--------------------------------------------------------------------//
	link1_link2PA.normalize();
	link1_link2CA.normalize();

	Matrix3d link1_link2Rot = Eigen::Matrix3d(Eigen::Quaterniond::FromTwoVectors(link1_link2PA, link1_link2CA));
	Eigen::Affine3d link1_link2Offset(Eigen::AngleAxisd(link1_link2OffsetQ, Vector3d::UnitZ()));
		
	SpatialTransform link1_link2ST;
	link1_link2ST.E = link1_link2Offset.rotation() * link1_link2Rot;
	link1_link2ST.r = link1_link2PP - (link1_link2Rot.inverse() * link1_link2CP);

	Vector3d pWorld_link1_link2 =  world_link1ST.E.transpose() * link1_link2ST.r;
	
	link1_link2Id = rbdlModel_->AddBody(base_link1Id, Xtrans(pWorld_link1_link2), 
		Joint(SpatialVector (0., 1., 0., 0., 0., 0.)), link2Body_, "link1-link2");

	world_link2ST = world_link1ST * link1_link2ST;

	// CreateRBDLJoint(link1_link2PA, link1_link2CA, link1_link2PP, link1_link2CP, link1_link2OffsetQ, 
	// Vector3d::UnitZ(), base_link1Id, Joint(SpatialVector (0., 1., 0., 0., 0., 0.)), 
	// world_link1ST, link1Body_, "link1-link2", link1_link2Id, world_link2ST);
	//--------------------------------------------------------------------//
	link2_link3PA.normalize();
	link2_link3CA.normalize();

	Matrix3d link2_link3Rot = Eigen::Matrix3d(Eigen::Quaterniond::FromTwoVectors(link2_link3PA, link2_link3CA));
	Eigen::Affine3d link2_link3Offset(Eigen::AngleAxisd(link2_link3OffsetQ, Vector3d::UnitZ()));
		
	SpatialTransform link2_link3ST;
	link2_link3ST.E = link2_link3Offset.rotation() * link2_link3Rot;
	link2_link3ST.r = link2_link3PP - (link2_link3Rot.inverse() * link2_link3CP);

	Vector3d pWorld_lik2_link3 =  world_link2ST.E.transpose() * link2_link3ST.r;
	
	link2_link3Id = rbdlModel_->AddBody(link1_link2Id, Xtrans(pWorld_lik2_link3), 
		Joint(SpatialVector (0., 0., 1., 0., 0., 0.)), link3Body_, "link2-link3");
	
	world_link3ST = world_link2ST * link2_link3ST;
	// CreateRBDLJoint(link2_link3PA, link2_link3CA, link2_link3PP, link2_link3CP, link2_link3OffsetQ, 
	// Vector3d::UnitZ(), link1_link2Id, Joint(SpatialVector (0., 0., 1., 0., 0., 0.)), 
	// world_link2ST, link2Body_, "link2-link3", link2_link3Id, world_link3ST);
	// //--------------------------------------------------------------------//
	link3_link4PA.normalize();
	link3_link4CA.normalize();

	Matrix3d link3_link4Rot = Eigen::Matrix3d(Eigen::Quaterniond::FromTwoVectors(link3_link4PA, link3_link4CA));
	Eigen::Affine3d link3_link4Offset(Eigen::AngleAxisd(link3_link4OffsetQ, Vector3d::UnitZ()));
		
	SpatialTransform link3_link4ST;
	link3_link4ST.E = link3_link4Offset.rotation() * link3_link4Rot;
	link3_link4ST.r = link3_link4PP - (link3_link4Rot.inverse() * link3_link4CP);

	Vector3d pWorld_lik3_link4 =  world_link3ST.E.inverse() * link3_link4ST.r;
	
	link3_link4Id = rbdlModel_->AddBody(link2_link3Id, Xtrans(pWorld_lik3_link4), 
		Joint(SpatialVector (0., -1., 0., 0., 0., 0.)), link4Body_, "link3-link4");
	world_link4ST = world_link3ST * link3_link4ST;
	// CreateRBDLJoint(link3_link4PA, link3_link4CA, link3_link4PP, link3_link4CP, link3_link4OffsetQ, 
	// Vector3d::UnitZ(), link2_link3Id, Joint(SpatialVector (0., -1., 0., 0., 0., 0.)), 
	// world_link3ST, link3Body_, "link3-link4", link3_link4Id, world_link4ST);
	// // //--------------------------------------------------------------------//
	link4_link5PA.normalize();
	link4_link5CA.normalize();

	Matrix3d link4_link5Rot = Eigen::Matrix3d(Eigen::Quaterniond::FromTwoVectors(link4_link5PA, link4_link5CA));
	Eigen::Affine3d link4_link5Offset(Eigen::AngleAxisd(link4_link5OffsetQ, Vector3d::UnitZ()));
		
	SpatialTransform link4_link5ST;
	link4_link5ST.E = link4_link5Offset.rotation() * link4_link5Rot;
	link4_link5ST.r = link4_link5PP - (link4_link5Rot.inverse() * link4_link5CP);


	Vector3d pWorld_lik4_link5 =  world_link4ST.E.inverse() * link4_link5ST.r;
	
	link4_link5Id = rbdlModel_->AddBody(link3_link4Id, Xtrans(pWorld_lik4_link5), 
		Joint(SpatialVector (0., 0., 1., 0., 0., 0.)), link4Body_, "link4-link5");
	world_link5ST = world_link4ST * link4_link5ST;
	// CreateRBDLJoint(link4_link5PA, link4_link5CA, link4_link5PP, link4_link5CP, link4_link5OffsetQ, 
	// Vector3d::UnitZ(), link3_link4Id, Joint(SpatialVector (0., 0., 1., 0., 0., 0.)), 
	// world_link4ST, link4Body_, "link4-link5", link4_link5Id, world_link5ST);
	// // //--------------------------------------------------------------------//
	link5_link6PA.normalize();
	link5_link6CA.normalize();

	Matrix3d link5_link6Rot = Eigen::Matrix3d(Eigen::Quaterniond::FromTwoVectors(link5_link6PA, link5_link6CA));
	Eigen::Affine3d link5_link6Offset(Eigen::AngleAxisd(link5_link6OffsetQ, Vector3d::UnitZ()));
		
	SpatialTransform link5_link6ST;
	link5_link6ST.E = link5_link6Offset.rotation() * link5_link6Rot;
	link5_link6ST.r = link5_link6PP - (link5_link6Rot.inverse() * link5_link6CP);

	Vector3d pWorld_lik5_link6 =  world_link5ST.E.inverse() * link5_link6ST.r;
	
	link5_link6Id = rbdlModel_->AddBody(link4_link5Id, Xtrans(pWorld_lik5_link6), 
		Joint(SpatialVector (0., 1., 0., 0., 0., 0.)), link6Body_, "link5-link6");
	world_link6ST = world_link5ST * link5_link6ST;
	// CreateRBDLJoint(link5_link6PA, link5_link6CA, link5_link6PP, link5_link6CP, link5_link6OffsetQ, 
	// Vector3d::UnitZ(), link4_link5Id, Joint(SpatialVector (0., 1., 0., 0., 0., 0.)), 
	// world_link5ST, link5Body_, "link5-link6", link5_link6Id, world_link6ST);
	// // //--------------------------------------------------------------------//
	link6_link7PA.normalize();
	link6_link7CA.normalize();

	Matrix3d link6_link7Rot = Eigen::Matrix3d(Eigen::Quaterniond::FromTwoVectors(link6_link7PA, link6_link7CA));
	Eigen::Affine3d link6_link7Offset(Eigen::AngleAxisd(link6_link7OffsetQ, Vector3d::UnitZ()));
		
	SpatialTransform link6_link7ST;
	link6_link7ST.E = link6_link7Offset.rotation() * link6_link7Rot;
	link6_link7ST.r = link6_link7PP - (link6_link7Rot.inverse() * link6_link7CP);


	Vector3d pWorld_lik6_link7 =  world_link6ST.E.inverse() * link6_link7ST.r;	
	link6_link7Id = rbdlModel_->AddBody(link5_link6Id, Xtrans(pWorld_lik6_link7), 
		Joint(SpatialVector (0., 0., 1., 0., 0., 0.)), link7Body_, "link6-link7");
	world_link7ST = world_link6ST * link6_link7ST;
	// CreateRBDLJoint(link6_link7PA, link6_link7CA, link6_link7PP, link6_link7CP, link6_link7OffsetQ, 
	// Vector3d::UnitZ(), link5_link6Id, Joint(SpatialVector (0., 0., 1., 0., 0., 0.)), 
	// world_link6ST, link6Body_, "link6-link7", link6_link7Id, world_link7ST);
	// //--------------------------------------------------------------------//
	
	// std::cout << "base_link1ST" << std::endl << base_link1ST << std::endl;
	// std::cout << "link1_link2ST" << std::endl << link1_link2ST << std::endl;
	// std::cout << "link2_link3ST" << std::endl << link2_link3ST << std::endl;
	// std::cout << "link3_link4ST" << std::endl << link3_link4ST << std::endl;
	// std::cout << "link4_link5ST" << std::endl << link4_link5ST << std::endl;
	// std::cout << "link5_link6ST" << std::endl << link5_link6ST << std::endl;
	// std::cout << "link6_link7ST" << std::endl << link6_link7ST << std::endl;

	// std::cout << "world_baseST" << std::endl << world_baseST << std::endl;
	// std::cout << "world_link1ST" << std::endl << world_link1ST << std::endl;
	// std::cout << "world_link2ST" << std::endl << world_link2ST << std::endl;
	// std::cout << "world_link3ST" << std::endl << world_link3ST << std::endl;
	// std::cout << "world_link4ST" << std::endl << world_link4ST << std::endl;
	// std::cout << "world_link5ST" << std::endl << world_link5ST << std::endl;
	// std::cout << "world_link6ST" << std::endl << world_link6ST << std::endl;
	// std::cout << "world_link7ST" << std::endl << world_link7ST << std::endl;

	Q_     = VectorNd::Constant ((size_t) rbdlModel_->dof_count, 0.);
	QDot_  = VectorNd::Constant ((size_t) rbdlModel_->dof_count, 0.);
	QDDot_ = VectorNd::Constant ((size_t) rbdlModel_->dof_count, 0.);
	Tau_   = VectorNd::Constant ((size_t) rbdlModel_->dof_count, 0.); 
	rbdlmBodyMap_ = rbdlModel_->mBodyNameMap;

	// Q_[0] = world_base_yaw_; // Z
  // Q_[1] = world_base_pitch_; // Y
  // Q_[2] = world_base_roll_; // X

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
	std::cout << "Q_" << std::endl << Q_ << std::endl;
  for(int i = 0; i < 10; i++)
  {
    for(rbdlmBodyMapItr_ = rbdlmBodyMap_.begin(); rbdlmBodyMapItr_ != rbdlmBodyMap_.end(); rbdlmBodyMapItr_++)
    {
      std::string jointName = rbdlmBodyMapItr_->first;
      unsigned int jointId = rbdlmBodyMapItr_->second;

			// World to base has 3 controlable joints and Q is zero indexed.
			// So qIndex would be 1 more than jointId 
			int qIndex = jointId + 1; 
			if(qIndex < 3) continue;

			// printf("jointName: %s, jointId: %d\n", jointName.c_str(), jointId);
      float jointAngle = Q_[qIndex];

			// printf("ExecutePoseInAMBF(): jointName: %s, qIndex: %d, jointAngle: %f\n", 
			// 	jointName.c_str(), qIndex, jointAngle);

      baselinkHandler->set_joint_pos<std::string>( jointName.c_str(), jointAngle);
    }
    usleep(sleepTime);
    RegisterAllRigidBodyPose();
  }
}

void KUKA::ExecutePose(VectorNd Q)
{
	Q_ = Q;
	Q_[0] = world_base_yaw_; // Z
  Q_[1] = world_base_pitch_; // Y
  Q_[2] = world_base_roll_; // X
	// Q_[3] = M_PI_2;
	ExecutePoseInAMBF();
}

t_w_nPtr KUKA::twnFromModels(std::string jointName)
{
	t_w_nPtr t_w_nptr = new T_W_N();

	unsigned int jointId = rbdlModel_->GetBodyId(jointName.c_str());
	if(jointId == 0) return t_w_nptr;
	// joint_ name: link1-link2, parentName: link1
	size_t npos = jointName.find("-", 0);
	if(npos == -1) return t_w_nptr;

	// const std::string parentName = jointName.substr(0, npos);
	const std::string parentName = jointName.substr(npos + 1, jointName.size());
	std::cout << "parentName: " << parentName << std::endl;

	ambfParamMapItr_ = ambfParamMap_.find(parentName);
	if(ambfParamMapItr_ == ambfParamMap_.end())
	{
		printf("Rigid body: %s not found in ambfParamMap\n", parentName.c_str());
		return nullptr;
	}
	
	EigenUtilities eu;

	// Collect AMBF and RBDL Transformation Matrices
	t_w_nptr->r_w_n_ambf = eu.SetAlmostZeroToZero<Matrix3d>(ambfParamMap_[parentName]->RotationMatrix());
	t_w_nptr->p_w_n_ambf = eu.SetAlmostZeroToZero<Vector3d>(ambfParamMap_[parentName]->TranslationVector());

	ForwardDynamics(*rbdlModel_, Q_, QDot_, Tau_, QDDot_);
	unsigned int rbdlBodyId = rbdlModel_->GetBodyId(jointName.c_str());

	t_w_nptr->r_w_n_rbdl = eu.SetAlmostZeroToZero<Matrix3d>
		(CalcBodyWorldOrientation(*rbdlModel_, Q_, rbdlBodyId, true));
	t_w_nptr->p_w_n_rbdl = eu.SetAlmostZeroToZero<Vector3d>
		(CalcBodyToBaseCoordinates(*rbdlModel_, Q_, rbdlBodyId, Vector3d(0., 0., 0.), true));

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

		if(rigidBodyHandler != nullptr) rigidBodyHandler->cleanUp();
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