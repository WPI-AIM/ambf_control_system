#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <algorithm>
#include "rbdl_model_tests/PS.h"


PS::PS() 
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

bool PS::ConnectToAMBF()
{
	ambfClientPtr_ = new Client();
	
	if(!ambfClientPtr_->connect()) return false;
	usleep(1000000);

	return true;
}

void PS::RegisterBodyToWorldTransformation(const std::string parentBody)
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

void PS::SetAMBFParams()
{
	std::vector<std::string> rigidBodyNames = ambfClientPtr_->getRigidBodyNames();

	const std::string modelName = "ps/";
	// Do not create handlers for Plane, target_fk, target_ik rigidbodies
	for(std::string rigidBodyName : rigidBodyNames)
	{
		if(rigidBodyName.find("Plane") 		 != std::string::npos ||
			 rigidBodyName.find("target_fk") != std::string::npos ||
			 rigidBodyName.find("target_ik") != std::string::npos) continue;
		
		// Search for the substring in string
    size_t pos = rigidBodyName.find(modelName);
    if (pos != std::string::npos)
    {
			// If found then erase it from string
			rigidBodyName.erase(pos, modelName.length());
    }

		ambfParamMap_.insert(AMBFParamPair(rigidBodyName, new AMBFParams(
			rigidBodyName, 
			ambfClientPtr_->getRigidBody(rigidBodyName.c_str(), true)))
		);
	}
	usleep(250000);
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

	RegisterBodyToWorldTransformation(baselinkName_);
	baselinkHandler_ = ambfParamMap_[baselinkName_]->RididBodyHandler();
	controlableJoints_ = baselinkHandler_->get_joint_names();
}

void PS::SetBodyParams()
{
	// mass, com - inertia offset, inertia
	const double mass                         = 1.0;
	RigidBodyDynamics::Math::Vector3d com     = {     0.0, -0.34415,     0.0 };
	RigidBodyDynamics::Math::Vector3d inertia = { 0.31777,  0.00961, 0.31777 };

	Vector3d vector3d_zero = Vector3d::Zero();

	worldBody_ = Body(0.0, vector3d_zero, vector3d_zero);
	tangibleBody_ = Body(mass, com, inertia);
	virtualBody_ = Body(0., vector3d_zero, vector3d_zero);
}

void PS::CreateRBDLModel()
{
  unsigned int world_l1Id, l1_l2Id, l2_l3Id, l3_l4Id, l1_l4Id, l3_vId;

  const double world_l1OffsetQ =  3.14189;
  const double l1_l2OffsetQ    =  1.575;
  const double l2_l3OffsetQ    = -1.575;
  const double l3_l4OffsetQ    = -1.567;
  const double l1_l4OffsetQ    = -1.567;

	Vector3d world_l1PA = {   1.0,      0.0,    0.0 };
	Vector3d world_l1CA = {   0.0, -0.00011,    1.0 };
	Vector3d world_l1PP = { 0.001,    -0.36, -0.222 };
	Vector3d world_l1CP = {   0.0,      0.0,    0.0 };

	Vector3d l1_l2PA = {     0.0, 0.00016, 1.0 };
	Vector3d l1_l2CA = { 0.00009, 0.00000, 1.0 };
	Vector3d l1_l2PP = {   0.139,   0.138, 0.0 };
	Vector3d l1_l2CP = {     0.0,     0.0, 0.0 };

	Vector3d l2_l3PA = {    0.0,      0.0, 1.0 };
	Vector3d l2_l3CA = {    0.0, -0.00016, 1.0 };
	Vector3d l2_l3PP = { -0.141,   -0.832, 0.0 };
	Vector3d l2_l3CP = {    0.0,      0.0, 0.0 };

	Vector3d l3_l4PA = {      0.0, 0.00035, 1.0 };
	Vector3d l3_l4CA = { -0.00017,     0.0, 1.0 };
	Vector3d l3_l4PP = {    -0.14,   -0.83, 0.0 };
	Vector3d l3_l4CP = {      0.0,     0.0, 0.0 };
	
	Vector3d l1_l4PA = {      0.0, 0.00016,       1.0 };
	Vector3d l1_l4CA = {  0.00024,     0.0,       1.0 };
	Vector3d l1_l4PP = {     0.07,   -0.77,       0.0 };
	Vector3d l1_l4CP = { -0.06239, -0.76135, -0.00018 };

	rbdlModelPtr_ = new Model();
	rbdlModelPtr_->gravity = Vector3d(0., 0., -9.81);

	// Register Word to Base Transform
	AMBFParamsPtr ambfRigidBodyParams = ambfParamMap_[baselinkName_];
	//0---------------------------------------------------------------------//
	world_l1ST.E = ambfRigidBodyParams->RotationMatrix();
	world_l1ST.r = ambfRigidBodyParams->TranslationVector();

	// This is to handle initial World to body rotation.
	//1--------------------------------------------------------------------//
	world_l1PA.normalize();
	world_l1CA.normalize();

	l1_l2PA.normalize();
	l1_l2CA.normalize();

	l2_l3PA.normalize();
	l2_l3CA.normalize();

	l3_l4PA.normalize();
	l3_l4CA.normalize();

	l1_l4PA.normalize();
	l1_l4CA.normalize();
	//1--------------------------------------------------------------------//
	Matrix3d world_l1Rot = 
	Eigen::Matrix3d(Eigen::Quaterniond::FromTwoVectors(world_l1PA, world_l1CA));
	Eigen::Affine3d world_l1RotOffset(Eigen::AngleAxisd(world_l1OffsetQ, Vector3d::UnitZ()));
		
	SpatialTransform world_l1ST;
	world_l1ST.E = world_l1Rot.transpose() * world_l1RotOffset.rotation();
	world_l1ST.r = world_l1PP - (world_l1Rot.transpose() * world_l1CP);
	//--------------------------------------------------------------------//
	Matrix3d l1_l2Rot = 
	Eigen::Matrix3d(Eigen::Quaterniond::FromTwoVectors(l1_l2PA, l1_l2CA));
	Eigen::Affine3d l1_l2RotOffset(
		Eigen::AngleAxisd(l1_l2OffsetQ, Vector3d::UnitZ()));
	
	SpatialTransform l1_l2ST;
	l1_l2ST.E = l1_l2Rot.transpose() * l1_l2RotOffset.rotation();
	l1_l2ST.r = l1_l2PP - (l1_l2Rot.transpose() * l1_l2CP);
	//1--------------------------------------------------------------------//
	Matrix3d l2_l3Rot = 
	Eigen::Matrix3d(
		Eigen::Quaterniond::FromTwoVectors(l2_l3PA, l2_l3CA));
	Eigen::Affine3d l2_l3RotOffset(
		Eigen::AngleAxisd(l2_l3OffsetQ, Vector3d::UnitZ()));
		
	SpatialTransform l2_l3ST;
	l2_l3ST.E = l2_l3Rot.transpose() * l2_l3RotOffset.rotation();
	l2_l3ST.r = l2_l3PP - (l2_l3Rot.transpose() * l2_l3CP);
	//--------------------------------------------------------------------//
	Matrix3d l3_l4Rot = 
	Eigen::Matrix3d(
		Eigen::Quaterniond::FromTwoVectors(l3_l4PA, l3_l4CA));
	Eigen::Affine3d l3_l4RotOffset(
		Eigen::AngleAxisd(l3_l4OffsetQ, Vector3d::UnitZ()));
		
	SpatialTransform l3_l4ST;
	l3_l4ST.E = l3_l4Rot.transpose() * l3_l4RotOffset.rotation();
	l3_l4ST.r = l3_l4PP - (l3_l4Rot.transpose() * l3_l4CP);
	//--------------------------------------------------------------------//
		Matrix3d l1_l4Rot = 
	Eigen::Matrix3d(
		Eigen::Quaterniond::FromTwoVectors(l1_l4PA, l1_l4CA));
	Eigen::Affine3d l1_l4RotOffset(
		Eigen::AngleAxisd(l1_l4OffsetQ, Vector3d::UnitZ()));
	SpatialTransform l1_l4ST;
	l1_l4ST.E = l1_l4Rot.transpose() * l1_l4RotOffset.rotation(); 
	l1_l4ST.r = l1_l4PP - (l1_l4Rot.transpose() * l1_l4CP);
	//--------------------------------------------------------------------//
	world_l2ST = world_l1ST * l1_l2ST;
	world_l3ST = world_l2ST * l2_l3ST;
	world_l4ST = world_l3ST * l3_l4ST;
	world_l4ST_ = world_l1ST * l1_l4ST;
	//1--------------------------------------------------------------------//
	// Vector3d p_world_l1_world = world_l1ST.E 				  * world_l1ST.r;
	Vector3d p_l1_l2_world 		= world_l1ST.E * l1_l2ST.r;
	Vector3d p_l2_l3_world    = world_l2ST.E * l2_l3ST.r;
	Vector3d p_l3_l4_world    = world_l3ST.E   * l3_l4ST.r;
	Vector3d p_l1_l4_world = world_l1ST.E 		  * l1_l4ST.r;
	//1--------------------------------------------------------------------//
	Joint joint_world = 
    Joint(JointTypeFixed);
	world_l1Id = rbdlModelPtr_-> 
		AddBody(0, Xtrans(world_l1ST.r), joint_world, tangibleBody_, "world-l1");

	Joint joint_l1 = Joint(SpatialVector (1., 0., 0., 0., 0., 0.));
	l1_l2Id = rbdlModelPtr_->
		AddBody(world_l1Id, Xtrans(p_l1_l2_world), 
		joint_l1, tangibleBody_, "l1-l2");

	l2_l3Id = rbdlModelPtr_->
		AddBody(l1_l2Id, Xtrans(p_l2_l3_world), 
		Joint(SpatialVector (-1., 0., 0., 0., 0., 0.)), tangibleBody_, "l2-l3");

	l3_l4Id = rbdlModelPtr_->
		AddBody(l2_l3Id, Xtrans(p_l3_l4_world), 
		Joint(SpatialVector (1., 0., 0., 0., 0., 0.)), tangibleBody_, "l3-l4");

	Vector3d l1bodyOffset(0.0, (0.470465 - (-0.36)), 0.222 -0.0830566);
	// std::cout << "p_l1_l4_world" << std::endl << p_l1_l4_world << std::endl; 
	// Vector3d l1bodyOffset(0.0, 0.0, 0.0);
	// p_l1_l4_world
	l1_l4Id = rbdlModelPtr_->
		AddBody(world_l1Id, Xtrans(l1bodyOffset), 
		Joint(SpatialVector (1., 0., 0., 0., 0., 0.)), tangibleBody_, "l1-l4");

	l3_vId = rbdlModelPtr_->
		AddBody(l2_l3Id, Xtrans(Vector3dZero), 
		Joint(SpatialVector (-1., 0., 0., 0., 0., 0.)), virtualBody_, "l2-l4");
	//--------------------------------------------------------------------//
	unsigned int userDefinedId = 7;
	cs_.AddLoopConstraint(l1_l2Id, l3_vId, X_p_, X_s_, 
		SpatialVector(0, 0, 0, 1, 0, 0), bgStab_, 0.1, "LoopXY_Rz", userDefinedId);

	//These two constraints below will be appended to the above
	//constraint by default, and will assume its name and user defined id
	cs_.AddLoopConstraint(l1_l2Id, l3_vId, X_p_, X_s_, SpatialVector(0, 0, 0, 0, 1, 0));
	cs_.AddLoopConstraint(l1_l2Id, l3_vId, X_p_, X_s_, SpatialVector(0, 0, 1, 0, 0, 0));
	cs_.Bind(*rbdlModelPtr_);

	Q_     = VectorNd::Constant ((size_t) rbdlModelPtr_->dof_count, 0.);
	QDot_  = VectorNd::Constant ((size_t) rbdlModelPtr_->dof_count, 0.);
	QDDot_ = VectorNd::Constant ((size_t) rbdlModelPtr_->dof_count, 0.);
	Tau_   = VectorNd::Constant ((size_t) rbdlModelPtr_->dof_count, 0.); 
	rbdlmBodyMap_ = rbdlModelPtr_->mBodyNameMap;
  
	ClearLogOutput();

	for(rbdlmBodyMapItr_ = rbdlmBodyMap_.begin(); rbdlmBodyMapItr_ != rbdlmBodyMap_.end(); rbdlmBodyMapItr_++)
  {
    std::string bodyName = rbdlmBodyMapItr_->first;
    unsigned int bodyId = rbdlmBodyMapItr_->second;
    std::string parentName = rbdlModelPtr_->GetBodyName(rbdlModelPtr_->GetParentBodyId(bodyId));
		bool isFixedBody = rbdlModelPtr_->IsFixedBodyId(bodyId);
    std::cout << parentName << ", " << bodyName << ", " << bodyId << ", " << isFixedBody << std::endl;
  }
}

void PS::PrintModelHierarchy()
{
	for(unsigned int bodyId = 0; bodyId < rbdlModelPtr_->q_size; bodyId++)
	{
    std::string bodyName = rbdlModelPtr_->GetBodyName(bodyId);
	  std::string parentName = rbdlModelPtr_->GetBodyName(rbdlModelPtr_->GetParentBodyId(bodyId));
    std::cout << parentName << ", " << bodyName << ", " << bodyId << std::endl;
  }
}

void PS::HelloThread()
{
	std::cout << "HelloThread\n";
}

void PS::RegisterRigidBodysPose()
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

int PS::QIndexFromJointName(const std::string jointName)
{
	unsigned int bodyId = rbdlModelPtr_->GetBodyId(jointName.c_str());
	if(bodyId < 1 || bodyId > rbdlModelPtr_->q_size) return -1;

	return --bodyId;
}

bool PS::JointAngleWithName(const std::string jointName, double qDesired)
{
	int qId = QIndexFromJointName(jointName);
	if(qId == -1) return false;
	Q_[qId] = qDesired;
	return true;
}

bool PS::ExecutePoseInAMBF()
{
	baselinkHandler_ = ambfParamMap_[baselinkName_]->RididBodyHandler();

	if(baselinkHandler_ == nullptr)
	{
		std::cout << "baselinkHandler is null, poses cannot be executed. Terminating execution\n";
		return false;
	}

	std::cout << "Q_" << std::endl << Q_ << std::endl;

	std::vector<std::string> controllableJoints = baselinkHandler_->get_joint_names();

	std::string jointName = "l1-l2";
	// std::cout << "ExecutePoseInAMBF() - jointName: " << jointName << QIndexFromJointName(jointName) << std::endl;
	int qIndex = QIndexFromJointName(jointName);
	float qDesired = Q_(qIndex);
	if (qIndex == -1) return false; 

	std::vector<float> qActulals;
  for(int i = 0; i < 10; i++)
  {
		// for(std::string jointName : controllableJoints)
		// {
		baselinkHandler_->
			set_joint_pos<std::string>(jointName, qDesired);
			// set_joint_pos<std::string>(jointName, 0.3f);
		// }
		qActulals = baselinkHandler_->get_all_joint_pos();
    usleep(sleepTime);
	
    RegisterRigidBodysPose();
  }

	std::cout << "qActulals: ";
	for(float q:qActulals)
	{
		printf("%f, ", q);
	} 
	std::cout << std::endl;

	// To be deleted
	// std::cout << "iterating ambfParamMap_\n";
	// for(ambfParamMapItr_ = ambfParamMap_.begin(); ambfParamMapItr_ != ambfParamMap_.end(); ambfParamMapItr_++)
  // {
  //   std::string bodyName = ambfParamMapItr_->first;

	// 	Matrix3d r_w_n_ambf = ambfParamMap_[bodyName]->RotationMatrix();
	// 	Vector3d p_w_n_ambf = ambfParamMap_[bodyName]->TranslationVector();

	// 	std::cout << "bodyName: " << bodyName << std::endl;
	// 	// std::cout << "q_w_n_ambf: " << std::endl << ambfParamMap_[bodyName]->GetQuternion() << std::endl << std::endl;
	// 	std::cout << "r_w_n_ambf: " << std::endl << r_w_n_ambf << std::endl << std::endl;
	// 	std::cout << "p_w_n_ambf: " << std::endl << p_w_n_ambf << std::endl << std::endl;
  // }
  // std::cout << std::endl << "------------------" << std::endl;


	return true;
}

bool PS::ExecutePose()
{
	return ExecutePoseInAMBF();
}

template <>
t_w_nPtr PS::twnFromModels(std::string jointName)
{
	// joint_ name: link1-link2, parentName: link1
	t_w_nPtr t_w_nptr = new T_W_N();
	size_t npos = jointName.find("-", 0);
	if(npos == -1) return t_w_nptr;

	const std::string parentName = jointName.substr(npos + 1, jointName.size());
	// std::cout << "parentName: " << parentName << std::endl;

	ambfParamMapItr_ = ambfParamMap_.find(parentName);
	int rbdlBodyId = rbdlModelPtr_->GetBodyId(jointName.c_str());
	if(ambfParamMapItr_ == ambfParamMap_.end())
	{
		printf("Rigid body: %s not found in ambfParamMap\n", parentName.c_str());
		return nullptr;
	}
	// std::cout << "jointName: " << jointName << " , rbdlBodyId: " << rbdlBodyId << std::endl;
	if(rbdlBodyId < 0)
	{
		printf("Joint: %s not found in RBDL Model\n", jointName.c_str());
		return nullptr;
	}
	
	t_w_nptr->bodyName = parentName;
	// Collect AMBF and RBDL Transformation Matrices
	t_w_nptr->r_w_n_ambf = ambfParamMap_[parentName]->RotationMatrix();
	t_w_nptr->p_w_n_ambf = ambfParamMap_[parentName]->TranslationVector();

	ForwardDynamics(*rbdlModelPtr_, Q_, QDot_, Tau_, QDDot_);

	// printf("Joint: %s, rbdlBodyId: %d\n", jointName.c_str(), rbdlBodyId);
	t_w_nptr->r_w_n_rbdl = CalcBodyWorldOrientation(*rbdlModelPtr_, Q_, rbdlBodyId, true);
	t_w_nptr->p_w_n_rbdl = CalcBodyToBaseCoordinates(*rbdlModelPtr_, Q_, rbdlBodyId, Vector3d(0., 0., 0.), true);
	
	return t_w_nptr;
}

template <>
t_w_nPtr PS::twnFromModels(unsigned int jointId)
{
	std::string jointName = rbdlModelPtr_->GetBodyName(jointId);
	return twnFromModels(jointName);
}

std::vector<t_w_nPtr> PS::HomePoseTransformation()
{
	Q_.setZero();

	ExecutePose();

	std::vector<t_w_nPtr> transformations;
	std::vector<t_w_nPtr>::iterator transformationsItr;

	// Skip ROOT which has bodyId = 0
	for(unsigned int bodyId = 1; bodyId < rbdlModelPtr_->q_size + 1; bodyId++)
  {
    std::string jointName = rbdlModelPtr_->GetBodyName(bodyId);
		transformations.emplace_back(twnFromModels(jointName));
  }

	// // This can be written over a loop if needed. Update RBDL home pose roation to be
	// // the values calcuated.
	// transformationsItr = std::find_if(transformations.begin(), transformations.end(),
	// 	[&](t_w_nPtr const & t_w_nptr) {return t_w_nptr->bodyName == "baselink";});
	// if ( transformationsItr != transformations.end() )
	// {
	// 	(*transformationsItr)->r_w_n_rbdl = world_baselinkST.E;
	// }

	// transformationsItr = std::find_if(transformations.begin(), transformations.end(),
	// 	[&](t_w_nPtr const & t_w_nptr) {return t_w_nptr->bodyName == "yawlink";});
	// if ( transformationsItr != transformations.end() )
	// {
	// 	(*transformationsItr)->r_w_n_rbdl = world_yawlinkST.E;
	// }

	// transformationsItr = std::find_if(transformations.begin(), transformations.end(),
	// 	[&](t_w_nPtr const & t_w_nptr) {return t_w_nptr->bodyName == "pitchbacklink";});
	// if ( transformationsItr != transformations.end() )
	// {
	// 	(*transformationsItr)->r_w_n_rbdl = world_pitchbacklinkST.E;
	// }

	// transformationsItr = std::find_if(transformations.begin(), transformations.end(),
	// 	[&](t_w_nPtr const & t_w_nptr) {return t_w_nptr->bodyName == "pitchbottomlink";});
	// if ( transformationsItr != transformations.end() )
	// {
	// 	(*transformationsItr)->r_w_n_rbdl = world_pitchbottomlinkST.E;
	// }

	// transformationsItr = std::find_if(transformations.begin(), transformations.end(),
	// 	[&](t_w_nPtr const & t_w_nptr) {return t_w_nptr->bodyName == "pitchendlink";});
	// if ( transformationsItr != transformations.end() )
	// {
	// 	(*transformationsItr)->r_w_n_rbdl = world_pitchendlinkST.E;
	// }

	// transformationsItr = std::find_if(transformations.begin(), transformations.end(),
	// 	[&](t_w_nPtr const & t_w_nptr) {return t_w_nptr->bodyName == "maininsertionlink";});
	// if ( transformationsItr != transformations.end() )
	// {
	// 	(*transformationsItr)->r_w_n_rbdl = world_maininsertionlinkST.E;
	// }

	// transformationsItr = std::find_if(transformations.begin(), transformations.end(),
	// 	[&](t_w_nPtr const & t_w_nptr) {return t_w_nptr->bodyName == "toollink";});
	// if ( transformationsItr != transformations.end() )
	// {
	// 	(*transformationsItr)->r_w_n_rbdl = world_toollinkST.E;
	// }

	return transformations;
}
void PS::CleanUp()
{
	this->~PS();
}

PS::~PS() 
{
	// CleanUp();

	if(baselinkHandler_ != nullptr) baselinkHandler_->cleanUp();
	if(ambfClientPtr_ != nullptr) ambfClientPtr_->cleanUp();
	if(rbdlModelPtr_ != nullptr) delete rbdlModelPtr_;
	if(ambfClientPtr_ != nullptr) delete ambfClientPtr_;
}