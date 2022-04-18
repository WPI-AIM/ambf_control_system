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
	
	// CleanUp();
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

void KUKA::CreateRBDLJoint(Vector3d& pa, Vector3d& ca, Vector3d& pp, Vector3d& cp, const double offsetQ, 
	Vector3d axis, unsigned int parentId, Vector3d& p_world_parent, Joint joint, SpatialTransform world_parentST, 
	Body &body, std::string bodyName, Vector3d& p_world_child, unsigned int& newBodyId, 
	SpatialTransform&	world_childST)
{
	pa.normalize();
	ca.normalize();

	Matrix3d bodyRot = Eigen::Matrix3d(Eigen::Quaterniond::FromTwoVectors(pa, ca));
	Eigen::Affine3d rotOffset(Eigen::AngleAxisd(offsetQ, axis));
		
	SpatialTransform bodyST;
	bodyST.E = rotOffset.rotation() * bodyRot;
	bodyST.r = pp - (bodyRot.inverse() * cp);

	// Vector3d roationAxis = world_parentST.E.block<3, 1>(0, 1).transpose();
	// roationAxis.normalize();
	// std::cout << "CreateRBDLJoint() " << bodyName << std::endl;
	// std::cout << "world_parentST" << std::endl << world_parentST << std::endl;
	// std::cout << "roationAxis" << std::endl << roationAxis << std::endl;
	// std::cout << "--------------------------\n";

	p_world_child =  world_parentST.E.inverse() * bodyST.r;
	newBodyId = rbdlModel_->AddBody(parentId, Xtrans(p_world_parent), joint, body, bodyName);
	
	world_childST = world_parentST * bodyST;
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
	
	// Vector3d p_world_link1 = Vector3d::Zero();
	// Vector3d p_world_link2 = Vector3d::Zero();
	// Vector3d p_world_link3 = Vector3d::Zero();
	// Vector3d p_world_link4 = Vector3d::Zero();
	// Vector3d p_world_link5 = Vector3d::Zero();
	// Vector3d p_world_link6 = Vector3d::Zero();
	// Vector3d p_world_link7 = Vector3d::Zero();
	// Vector3d p_world_ee	 	 = Vector3d::Zero();

	SetBodyParams();
	
	rbdlModel_->gravity = Vector3d(0., 0., -9.81);

	// Register Word to Base Transform
	AMBFParamsPtr ambfRigidBodyParams = ambfParamMap_[baselinkName_];

	// world_baseST.E = ambfRigidBodyParams->RotationMatrix();
	// world_baseST.r.setIdentity();
	// Vector3d p_world_base  = ambfRigidBodyParams->TranslationVector();
	Vector3d p_world_base;
	p_world_base.setZero();
	// std::cout << "world_baseST" << std::endl << world_baseST << std::endl;
	// std::cout << "p_world_base" << std::endl << p_world_base << std::endl;

	Matrix3d base;
	base.setIdentity();
	//1--------------------------------------------------------------------//
	// base_link1PA.normalize();
	// base_link1CA.normalize();

	// Matrix3d base_link1Rot = 
	// 	Eigen::Matrix3d(Eigen::Quaterniond::FromTwoVectors(base_link1PA, 
	// 																											base_link1CA));
	// Eigen::Affine3d base_link1_offset(Eigen::AngleAxisd(base_link1OffsetQ, 
	// 	Eigen::Vector3d::UnitZ()));
	// // Eigen::Matrix3d base_link1_offset = EigenUtilities::rotZ(base_link1Offset);

	// SpatialTransform base_link1ST;
	// base_link1ST.E = base_link1_offset.rotation() * base_link1Rot;
	// base_link1ST.r = base_link1PP - (base_link1Rot.inverse() * base_link1CP);

	Joint base_link1Joint = Joint(SpatialVector(0.0, 0.0, 1.0, 0.0, 0.0, 0.0));
	// RigidBodyDynamics::Joint(RigidBodyDynamics::JointType::JointTypeRevolute, 
	// base_link1ST.E.transpose().block<3, 1>(0, 2).transpose());
	
	// unsigned int base_link1ID = rbdlModel_->AddBody(0, 
	// 	RigidBodyDynamics::Math::Xtrans(p_world_base), 
	// 	base_link1Joint, link1Body_, "base-link1");

	// const Eigen::Vector3d P_base_link1_base =  base * base_link1ST.r;
	unsigned int base_link1ID = 
    rbdlModel_->AddBody(0, Xtrans(Vector3d(0., 0., 0.)), base_link1Joint, link1Body_, "base-link1");
	//--------------------------------------------------------------------//
	// link1_link2PA.normalize();
	// link1_link2CA.normalize();

	// Eigen::Matrix3d link1_link2Rot = 
	// 	Eigen::Matrix3d(Eigen::Quaterniond::FromTwoVectors(link1_link2PA, 
	// 	link1_link2CA));
	// Eigen::Affine3d link1_link2_offset(Eigen::AngleAxisd(link1_link2OffsetQ, 
	// 	Eigen::Vector3d::UnitZ()));
	// // Eigen::Matrix3d link1_link2_offset = 
	// //   EigenUtilities::rotZ(link1_link2Offset);
	// SpatialTransform link1_link2ST;
	// link1_link2ST.E = link1_link2_offset.rotation() * link1_link2Rot;
	// link1_link2ST.r = 
	// 	link1_link2PP - (link1_link2Rot.inverse() * link1_link2CP);
	
	// const SpatialTransform base_link2ST = base_link1ST * link1_link2ST;

	Joint link1_link2Joint = Joint(SpatialVector(0.0, 1.0, 0.0, 0.0, 0.0, 0.0));
		// RigidBodyDynamics::Joint(RigidBodyDynamics::JointType::JointTypeRevolute, 
		// base_link2ST.E.transpose().block<3, 1>(0, 2).transpose());

	// unsigned int link1_link2ID = rbdlModel_->AddBody(base_link1ID, 
	// 	RigidBodyDynamics::Math::Xtrans(P_base_link1_base), 
	// 	link1_link2Joint, link2Body_, "link1-link2");
  // Matrix3d r_world_link1( 
  //   1.0, 0.0, 0.0,
  //   0.0, 1.0, 0.0,
  //   0.0, 0.0, 1.0
  // );
  Matrix3d r_world_link1( 
    0.707037,     0.707177, -0.000297534,
   -0.707177,     0.707037, -1.45781e-05,
 0.000200058,  0.000220716,            1
  );
	const Vector3d p_world_link1 = r_world_link1 * base_link1PP;
	unsigned int link1_link2ID = 
    rbdlModel_->AddBody(base_link1ID, Xtrans(p_world_link1), link1_link2Joint, link2Body_, "link1-link2");
	//--------------------------------------------------------------------//
	// link2_link3PA.normalize();
	// link2_link3CA.normalize();

	// Eigen::Matrix3d link2_link3Rot = 
	// 	Eigen::Matrix3d(Eigen::Quaterniond::FromTwoVectors(link2_link3PA, 
	// 																											link2_link3CA));
	// Eigen::Affine3d link2_link3_offset(Eigen::AngleAxisd(link2_link3OffsetQ, 
	// 	Eigen::Vector3d::UnitZ()));
	// // Eigen::Matrix3d link2_link3_offset = 
	// //   EigenUtilities::rotZ(link2_link3Offset);
	// SpatialTransform link2_link3ST;
	// link2_link3ST.E = link2_link3_offset.rotation() * link2_link3Rot;
	// link2_link3ST.r = 
	// 	link2_link3PP - (link2_link3Rot.inverse() * link2_link3CP);   

	// const SpatialTransform base_link3ST = base_link2ST * link2_link3ST;

	Joint link2_link3Joint = Joint(SpatialVector(0.0, 0.0, 1.0, 0.0, 0.0, 0.0));
		// RigidBodyDynamics::Joint(RigidBodyDynamics::JointType::JointTypeRevolute, 
		// base_link3ST.E.transpose().block<3, 1>(0, 2).transpose());

	// unsigned int link2_link3ID = rbdlModel_->AddBody(link1_link2ID, 
	// 	RigidBodyDynamics::Math::Xtrans(P_base_link2_base), 
	// 	link2_link3Joint, link3Body_, "link2-link3");

	// const Eigen::Vector3d P_base_link3_base = 
	// 	base_link2ST.E.inverse() * link2_link3ST.r;
  // Matrix3d r_world_link2( 
  //   1.0, 0.0,  0.0,
  //   0.0, 0.0, -1.0,
  //   0.0, 1.0,  0.0
  // );
  Matrix3d r_world_link2( 
   0.489667,     0.48996,   -0.721226,
  -0.509947,    -0.51002,     -0.6927,
  -0.707234,    0.706979, 0.000114015
  );
  const Vector3d p_world_link2 = r_world_link1 * link1_link2PP;
	unsigned int link2_link3ID = rbdlModel_->AddBody(link1_link2ID, 
		RigidBodyDynamics::Math::Xtrans(p_world_link2), 
		link2_link3Joint, link3Body_, "link2-link3");
	//--------------------------------------------------------------------//
  /*
	link3_link4PA.normalize();
	link3_link4CA.normalize();

	Eigen::Matrix3d link3_link4Rot = 
		Eigen::Matrix3d(Eigen::Quaterniond::FromTwoVectors(link3_link4PA, 
		link3_link4CA));
	Eigen::Affine3d link3_link4_offset(Eigen::AngleAxisd(link3_link4OffsetQ, 
		Eigen::Vector3d::UnitZ()));
	// Eigen::Matrix3d link3_link4_offset = 
	//   EigenUtilities::rotZ(link3_link4Offset);

	SpatialTransform link3_link4ST;
	link3_link4ST.E = link3_link4_offset.rotation() * link3_link4Rot;
	link3_link4ST.r = 
		link3_link4PP - (link3_link4Rot.inverse() * link3_link4CP);

	const SpatialTransform base_link4ST = base_link3ST * link3_link4ST;

	Joint link3_link4Joint = 
		RigidBodyDynamics::Joint(RigidBodyDynamics::JointType::JointTypeRevolute, 
		base_link4ST.E.transpose().block<3, 1>(0, 2).transpose());

	// unsigned int link3_link4ID = rbdlModel_->AddBody(link2_link3ID, 
	// 	RigidBodyDynamics::Math::Xtrans(P_base_link3_base), link3_link4Joint, 
	// 	link4Body_, "link3-link4");
		
	const Eigen::Vector3d P_base_link4_base = 
		base_link3ST.E.inverse() * link3_link4ST.r;
	
	unsigned int link3_link4ID = rbdlModel_->AddBody(link2_link3ID, 
		RigidBodyDynamics::Math::Xtrans(P_base_link4_base), link3_link4Joint, 
		link4Body_, "link3-link4");
	//--------------------------------------------------------------------//
*/
  // std::cout << "base" << std::endl << base << std::endl;
  // std::cout << "p_world_base" << std::endl << p_world_base << std::endl;
  // std::cout << "-------------------------\n";
  // std::cout << "base_link1ST" << std::endl << base_link1ST << std::endl;
  // std::cout << "P_base_link1_base" << std::endl << P_base_link1_base << std::endl;
  // std::cout << "-------------------------\n";
  // std::cout << "link1_link2ST" << std::endl << link1_link2ST << std::endl;
  // std::cout << "P_base_link2_base" << std::endl << P_base_link2_base << std::endl;
  // std::cout << "-------------------------\n";


// CreateRBDLJoint(Vector3d& pa, Vector3d& ca, Vector3d& pp, Vector3d& cp, const double offsetQ, 
// 	Vector3d axis, unsigned int parentId, Vector3d& p_world_parent, Joint joint, SpatialTransform world_parentST, 
// 	Body &body, std::string bodyName, Vector3d& p_world_child, unsigned int& newBodyId, 
// 	SpatialTransform&	world_childST)

	// CreateRBDLJoint(base_link1PA, base_link1CA, base_link1PP, base_link1CP, base_link1OffsetQ, 
	// Vector3d::UnitZ(), 0, p_world_base, Joint(SpatialVector (0., 0., 1., 0., 0., 0.)), 
	// world_baseST, baseBody_, "base-link1", p_world_link1, base_link1Id, world_link1ST);
	
	// CreateRBDLJoint(link1_link2PA, link1_link2CA, link1_link2PP, link1_link2CP, link1_link2OffsetQ, 
	// Vector3d::UnitZ(), base_link1Id, p_world_link1, Joint(SpatialVector (0., 1., 0., 0., 0., 0.)), 
	// world_link1ST, link1Body_, "link1-link2", p_world_link2, link1_link2Id, world_link2ST);
	
	// CreateRBDLJoint(link2_link3PA, link2_link3CA, link2_link3PP, link2_link3CP, link2_link3OffsetQ, 
	// Vector3d::UnitZ(), link1_link2Id, p_world_link2, Joint(SpatialVector (0., 0., 1., 0., 0., 0.)), 
	// world_link2ST, link2Body_, "link2-link3", p_world_link3, link2_link3Id, world_link3ST);

	// CreateRBDLJoint(link3_link4PA, link3_link4CA, link3_link4PP, link3_link4CP, link3_link4OffsetQ, 
	// Vector3d::UnitZ(), link2_link3Id, p_world_link3, Joint(SpatialVector (0., -1., 0., 0., 0., 0.)), 
	// world_link3ST, link3Body_, "link3-link4", p_world_link4, link3_link4Id, world_link4ST);

	// CreateRBDLJoint(link4_link5PA, link4_link5CA, link4_link5PP, link4_link5CP, link4_link5OffsetQ, 
	// Vector3d::UnitZ(), link3_link4Id, p_world_link4, Joint(SpatialVector (0., 0., 1., 0., 0., 0.)), 
	// world_link4ST, link4Body_, "link4-link5", p_world_link5, link4_link5Id, world_link5ST);

	// CreateRBDLJoint(link5_link6PA, link5_link6CA, link5_link6PP, link5_link6CP, link5_link6OffsetQ, 
	// Vector3d::UnitZ(), link4_link5Id, p_world_link5, Joint(SpatialVector (0., 1., 0., 0., 0., 0.)), 
	// world_link5ST, link5Body_, "link5-link6", p_world_link6, link5_link6Id, world_link6ST);

	// CreateRBDLJoint(link6_link7PA, link6_link7CA, link6_link7PP, link6_link7CP, link6_link7OffsetQ, 
	// Vector3d::UnitZ(), link5_link6Id, p_world_link6, Joint(SpatialVector (0., 0., 1., 0., 0., 0.)), 
	// world_link6ST, link6Body_, "link6-link7", p_world_link7, link6_link7Id, world_link7ST);

	// CreateRBDLJoint(link7_eePA, link7_eeCA, link7_eePP, link7_eeCP, link7_eeOffsetQ, 
	// Vector3d::UnitZ(), link6_link7Id, p_world_link7, Joint(SpatialVector (0., 0., 0., 0., 0., 0.)), 
	// world_link7ST, link7Body_, "link7-ee", p_world_ee, link7_eeId, world_eeST);

	// std::cout << "world_baseST" << std::endl << world_baseST << std::endl;
	// std::cout << "p_world_base" << std::endl << p_world_base << std::endl;
	// std::cout << "---------------------\n";
	// std::cout << "world_link1ST" << std::endl << world_link1ST << std::endl;
	// std::cout << "p_world_link1" << std::endl << p_world_link1 << std::endl;
	// std::cout << "---------------------\n";
	// std::cout << "world_link2ST" << std::endl << world_link2ST << std::endl;
	// std::cout << "p_world_link2" << std::endl << p_world_link2 << std::endl;
	// std::cout << "---------------------\n";
	// std::cout << "world_link3ST" << std::endl << world_link3ST << std::endl;
	// std::cout << "p_world_link3" << std::endl << p_world_link3 << std::endl;
	// std::cout << "---------------------\n";
	// std::cout << "world_link4ST" << std::endl << world_link4ST << std::endl;
	// std::cout << "p_world_link4" << std::endl << p_world_link4 << std::endl;
	// std::cout << "---------------------\n";
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
			printf("ExecutePoseInAMBF(): jointName: %s, jointAngle: %f\n", jointName.c_str(), jointAngle);
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

	// const std::string parentName = jointName.substr(0, jointName.find("-", 0));
	const std::string parentName = jointName.substr(0, npos);
	// printf("joint_ name: %s, parentName: %s, npos: %ld\n", jointName.c_str(), parentName.c_str(), npos);
	//-------------------------------------------------------------------------//
	// std::cout << "ambfParamMap_\n";
	// for(ambfParamMapItr_ = ambfParamMap_.begin(); ambfParamMapItr_ != ambfParamMap_.end();
	// ambfParamMapItr_++)
	// {
	// 	printf("rigidbody: %s, parent: %s\n", 
	// 		ambfParamMapItr_->first.c_str() , ambfParamMapItr_->second->ParentBodyName().c_str());
	// }

	// std::cout << "-------------\n";

	ambfParamMapItr_ = ambfParamMap_.find(parentName);
	if(ambfParamMapItr_ == ambfParamMap_.end())
	{
		printf("Rigid body: %s not found in ambfParamMap\n", parentName.c_str());
		return nullptr;
	}
	
	// Collect AMBF and RBDL Transformation Matrices
	// t_w_nptr->r_w_n_ambf = ambfParamMap_[parentName]->RotationMatrix();
	// t_w_nptr->p_w_n_ambf = ambfParamMap_[parentName]->TranslationVector();

	// For troubleshooting	
	MatrixNd t_w_n_ambf = MatrixNd::Identity(4, 4);
	t_w_n_ambf.block<3, 3>(0, 0) = ambfParamMap_[parentName]->RotationMatrix();
	t_w_n_ambf.block<3, 1>(0, 3) = ambfParamMap_[parentName]->TranslationVector();
	
  // Updating pose w.r.t base and not w.r.t world for testing.
  // This has to chaned w.r.t world.
	MatrixNd t_0_n_ambf = MatrixNd::Identity(4, 4);
	t_0_n_ambf = t_0_w_ * t_w_n_ambf;
	t_w_nptr->r_w_n_ambf = t_0_n_ambf.block<3, 3>(0, 0);
	t_w_nptr->p_w_n_ambf = t_0_n_ambf.block<3, 1>(0, 3);

	// std::cout << "twnFromModels(): Q_" << std::endl << Q_ << std::endl;
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