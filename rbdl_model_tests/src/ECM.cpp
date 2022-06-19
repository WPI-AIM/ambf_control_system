#include "rbdl_model_tests/ECM.h"


#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <algorithm>

ECM::ECM() 
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

bool ECM::ConnectToAMBF()
{
	ambfClientPtr_ = new Client();
	
	if(!ambfClientPtr_->connect()) return false;
	usleep(1000000);

	return true;
}

void ECM::RegisterBodyToWorldTransformation(const std::string parentBody)
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


void ECM::SetAMBFParams()
{
	std::vector<std::string> rigidBodyNames = ambfClientPtr_->getRigidBodyNames();

	const std::string modelName = "ecm/";
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

void ECM::CreateRBDLModel()
{
  unsigned int world_baselinkId, baselink_yawlinkId, yawlink_pitchbacklinkId, yawlink_pitchfrontlinkId, 
	pitchbacklink_pitchbottomlinkId, pitchbacklink_pitchbottomlink_v_Id, pitchbottomlink_pitchendlinkId, pitchendlink_maininsertionlinkId,
	pitchfrontlink_pitchtoplinkId, pitchfrontlink_pitchbottomlinkId, pitchtoplink_pitchendlinkId,
	maininsertionlink_toollinkId;

  const double ROOT_baselinkOffsetQ                  = 0.0;
  const double baselink_yawlinkOffsetQ               = -3.1414;
  const double yawlink_pitchbacklinkOffsetQ          = 3.1416;
  const double pitchbacklink_pitchbottomlinkOffsetQ  = 0.0;
  const double baselink_pitchendlinkOffsetQ          = 1.56304;
  const double pitchendlink_maininsertionlinkOffsetQ = 0.0;
  const double maininsertionlink_toollinkOffsetQ     = -1.5708;
  const double pitchbottomlink_pitchendlinkOffsetQ   = 0.0;
  const double yawlink_pitchfrontlinkOffsetQ         = 3.1416;
  const double pitchfrontlink_pitchbottomlinkOffsetQ = 0.0;
  const double pitchfrontlink_pitchtoplinkOffsetQ    = 0.0;
  const double pitchtoplink_pitchendlinkOffsetQ      = 0.0;
	const double toollink_eeOffsetQ      							 = 0.0;

	Vector3d baselink_yawlinkPA 							= { -0.0002, -1.0000, 00.0000 };
	Vector3d baselink_yawlinkCA 							= { 00.0000, 00.0000, -1.0000 };
	Vector3d baselink_yawlinkPP 							= { 00.0000, 00.0000, 00.0000 };
	Vector3d baselink_yawlinkCP 							= { 00.0001, 00.0000, 00.5369 };

	Vector3d yawlink_pitchbacklinkPA 					= { 1.0,     0.0,    0.0 };
	Vector3d yawlink_pitchbacklinkCA 					= { 0.0,     0.0,    1.0 };
	Vector3d yawlink_pitchbacklinkPP 					= { 0.0, -0.0098, 0.1624 };
	Vector3d yawlink_pitchbacklinkCP 					= { 0.0,     0.0,    0.0 };

	Vector3d pitchbacklink_pitchbottomlinkPA 	= {     0.0,     0.0,     1.0 };
	Vector3d pitchbacklink_pitchbottomlinkCA 	= {     0.0,     0.0,     1.0 };
	Vector3d pitchbacklink_pitchbottomlinkPP 	= { -0.1028, -0.2867,     0.0 };
	Vector3d pitchbacklink_pitchbottomlinkCP 	= { -0.0364,  0.0098, -0.0005 };

	Vector3d pitchbottomlink_pitchendlinkPA 	= {    0.0,     0.0,     1.0 };
	Vector3d pitchbottomlink_pitchendlinkCA 	= {    0.0,     0.0,     1.0 };
	Vector3d pitchbottomlink_pitchendlinkPP 	= { 0.3401, -0.0001, -0.0005 };
	Vector3d pitchbottomlink_pitchendlinkCP 	= {    0.0,     0.0,  0.0001 };
	
	Vector3d pitchendlink_maininsertionlinkPA = {     0.0,     1.0,    0.0 };
	Vector3d pitchendlink_maininsertionlinkCA = {     1.0,     0.0,    0.0 };
	Vector3d pitchendlink_maininsertionlinkPP = {  0.1031, -0.0961, 0.0001 };
	Vector3d pitchendlink_maininsertionlinkCP = { -0.0108,  -0.062, 0.0002 };

	Vector3d maininsertionlink_toollinkPA 	  = {     1.0,     0.0,    0.0 };
	Vector3d maininsertionlink_toollinkCA 	  = {     0.0,     0.0,   -1.0 };
	Vector3d maininsertionlink_toollinkPP 	  = { -0.0108,  -0.062,    0.0 };
	Vector3d maininsertionlink_toollinkCP 	  = { -0.0001, -0.0002, 0.0118 };

	rbdlModelPtr_ = new Model();
	rbdlModelPtr_->gravity = Vector3d(0., 0., -9.81);

	// Register Word to Base Transform
	AMBFParamsPtr ambfRigidBodyParams = ambfParamMap_[baselinkName_];
	//0---------------------------------------------------------------------//
	world_baselinkST.E = ambfRigidBodyParams->RotationMatrix();
	world_baselinkST.r = ambfRigidBodyParams->TranslationVector();

	// This is to handle initial World to body rotation.
	//1--------------------------------------------------------------------//
	baselink_yawlinkPA.normalize();
	baselink_yawlinkCA.normalize();

	yawlink_pitchbacklinkPA.normalize();
	yawlink_pitchbacklinkCA.normalize();

	pitchbacklink_pitchbottomlinkPA.normalize();
	pitchbacklink_pitchbottomlinkCA.normalize();

	pitchbottomlink_pitchendlinkPA.normalize();
	pitchbottomlink_pitchendlinkCA.normalize();

	pitchendlink_maininsertionlinkPA.normalize();
	pitchendlink_maininsertionlinkCA.normalize();
	//1--------------------------------------------------------------------//
	Matrix3d baselink_yawlinkRot = 
	Eigen::Matrix3d(Eigen::Quaterniond::FromTwoVectors(baselink_yawlinkPA, baselink_yawlinkCA));
	Eigen::Affine3d baselink_yawlinkRotOffset(Eigen::AngleAxisd(baselink_yawlinkOffsetQ, -Vector3d::UnitZ()));
		
	SpatialTransform baselink_yawlinkST;
	baselink_yawlinkST.E = baselink_yawlinkRot.transpose() * baselink_yawlinkRotOffset.rotation();
	baselink_yawlinkST.r = 
		baselink_yawlinkPP - (baselink_yawlinkRot.transpose() * baselink_yawlinkCP);
	//--------------------------------------------------------------------//
	Matrix3d yawlink_pitchbacklinkRot = 
	Eigen::Matrix3d(Eigen::Quaterniond::FromTwoVectors(yawlink_pitchbacklinkPA, yawlink_pitchbacklinkCA));
	Eigen::Affine3d yawlink_pitchbacklinkRotOffset(
		Eigen::AngleAxisd(yawlink_pitchbacklinkOffsetQ, -Vector3d::UnitZ()));
	
	SpatialTransform yawlink_pitchbacklinkST;
	yawlink_pitchbacklinkST.E = yawlink_pitchbacklinkRot.transpose() * yawlink_pitchbacklinkRotOffset.rotation();
	yawlink_pitchbacklinkST.r = 
		yawlink_pitchbacklinkPP - (yawlink_pitchbacklinkRot.transpose() * yawlink_pitchbacklinkCP);
	//1--------------------------------------------------------------------//
	Matrix3d pitchbacklink_pitchbottomlinkRot = 
	Eigen::Matrix3d(
		Eigen::Quaterniond::FromTwoVectors(pitchbacklink_pitchbottomlinkPA, pitchbacklink_pitchbottomlinkCA));
	Eigen::Affine3d pitchbacklink_pitchbottomlinkRotOffset(
		Eigen::AngleAxisd(pitchbacklink_pitchbottomlinkOffsetQ, -Vector3d::UnitZ()));
		
	SpatialTransform pitchbacklink_pitchbottomlinkST;
	pitchbacklink_pitchbottomlinkST.E = 
		pitchbacklink_pitchbottomlinkRot.transpose() * pitchbacklink_pitchbottomlinkRotOffset.rotation();
	pitchbacklink_pitchbottomlinkST.r = 
		pitchbacklink_pitchbottomlinkPP - 
		(pitchbacklink_pitchbottomlinkRot.transpose() * pitchbacklink_pitchbottomlinkCP);
	//--------------------------------------------------------------------//
	Matrix3d pitchbottomlink_pitchendlinkRot = 
	Eigen::Matrix3d(
		Eigen::Quaterniond::FromTwoVectors(pitchbottomlink_pitchendlinkPA, pitchbottomlink_pitchendlinkCA));
	Eigen::Affine3d pitchbottomlink_pitchendlinkRotOffset(
		Eigen::AngleAxisd(pitchbottomlink_pitchendlinkOffsetQ, -Vector3d::UnitX()));
		
	SpatialTransform pitchbottomlink_pitchendlinkST;
	pitchbottomlink_pitchendlinkST.E = 
		pitchbottomlink_pitchendlinkRot.transpose() * pitchbottomlink_pitchendlinkRotOffset.rotation();
	pitchbottomlink_pitchendlinkST.r = 
		pitchbottomlink_pitchendlinkPP - 
		(pitchbottomlink_pitchendlinkRot.transpose() * pitchbottomlink_pitchendlinkCP);
	//--------------------------------------------------------------------//
		Matrix3d pitchendlink_maininsertionlinkRot = 
	Eigen::Matrix3d(
		Eigen::Quaterniond::FromTwoVectors(pitchendlink_maininsertionlinkPA, pitchendlink_maininsertionlinkCA));
	Eigen::Affine3d pitchendlink_maininsertionlinkRotOffset(
		Eigen::AngleAxisd(pitchendlink_maininsertionlinkOffsetQ, -Vector3d::UnitZ()));
	SpatialTransform pitchendlink_maininsertionlinkST;
	pitchendlink_maininsertionlinkST.E =
	pitchendlink_maininsertionlinkRot.transpose() * pitchendlink_maininsertionlinkRotOffset.rotation(); 
	pitchendlink_maininsertionlinkST.r = 
		pitchendlink_maininsertionlinkPP - 
		(pitchendlink_maininsertionlinkRot.transpose() * pitchendlink_maininsertionlinkCP);
	//--------------------------------------------------------------------//
	Matrix3d maininsertionlink_toollinkRot = 
	Eigen::Matrix3d(
		Eigen::Quaterniond::FromTwoVectors(maininsertionlink_toollinkPA, maininsertionlink_toollinkCA));
	Eigen::Affine3d maininsertionlink_toollinkRotOffset(
		Eigen::AngleAxisd(maininsertionlink_toollinkOffsetQ, -Vector3d::UnitZ()));
		
	SpatialTransform maininsertionlink_toollinkST;
	maininsertionlink_toollinkST.E = maininsertionlink_toollinkRot.transpose() * maininsertionlink_toollinkRotOffset.rotation();
	maininsertionlink_toollinkST.r = 
		maininsertionlink_toollinkPP - 
		(maininsertionlink_toollinkRot.transpose() * maininsertionlink_toollinkCP);
	//--------------------------------------------------------------------//
	// std::cout << "world_baselinkST" 			 					<< std::endl << world_baselinkST 								<< std::endl;
	// std::cout << "baselink_yawlinkST" 		 					<< std::endl << baselink_yawlinkST 							<< std::endl;
	// std::cout << "yawlink_pitchbacklinkST" 					<< std::endl << yawlink_pitchbacklinkST 				<< std::endl;
	// std::cout << "pitchbacklink_pitchbottomlinkST" 	<< std::endl << pitchbacklink_pitchbottomlinkST << std::endl;
	// std::cout << "pitchbottomlink_pitchendlinkST" 	<< std::endl << pitchbottomlink_pitchendlinkST 	<< std::endl;
	// std::cout << "pitchendlink_maininsertionlinkST" << std::endl << pitchendlink_maininsertionlinkST << std::endl;
	// std::cout << "maininsertionlink_toollinkST" 		<< std::endl << maininsertionlink_toollinkST 		<< std::endl;

	world_yawlinkST = world_baselinkST * baselink_yawlinkST;
	world_pitchbacklinkST = world_yawlinkST * yawlink_pitchbacklinkST;
	world_pitchbottomlinkST = world_pitchbacklinkST * pitchbacklink_pitchbottomlinkST;
	world_pitchendlinkST = world_pitchbottomlinkST * pitchbottomlink_pitchendlinkST;
	world_maininsertionlinkST = world_pitchendlinkST * pitchendlink_maininsertionlinkST;
	world_toollinkST = world_maininsertionlinkST * maininsertionlink_toollinkST;

	// std::cout << "world_baselinkST" 				 << std::endl << world_baselinkST 				 << std::endl;
	// std::cout << "world_yawlinkST" 		 	 		 << std::endl << world_yawlinkST 					 << std::endl;
	// std::cout << "world_pitchbacklinkST" 	 	 << std::endl << world_pitchbacklinkST 		 << std::endl;
	// std::cout << "world_pitchbottomlinkST" 	 << std::endl << world_pitchbottomlinkST 	 << std::endl;
	// std::cout << "world_pitchendlinkST" 		 << std::endl << world_pitchendlinkST 		 << std::endl;
	// std::cout << "world_maininsertionlinkST" << std::endl << world_maininsertionlinkST << std::endl;
	// std::cout << "world_toollinkST" 				 << std::endl << world_toollinkST 				 << std::endl;
	//1--------------------------------------------------------------------//
	Vector3d p_baselink_yawlink_world 						  = world_baselinkST.E 				  * baselink_yawlinkST.r;
	Vector3d p_yawlink_pitchbacklink_world 				  = world_yawlinkST.E 				  * yawlink_pitchbacklinkST.r;
	Vector3d p_pitchbacklink_pitchbottomlink_world  = world_pitchbacklinkST.E 	  * pitchbacklink_pitchbottomlinkST.r;
	Vector3d p_pitchbottomlink_pitchendlink_world   = world_pitchbottomlinkST.E   * pitchbottomlink_pitchendlinkST.r;
	Vector3d p_pitchendlink_maininsertionlink_world = world_pitchendlinkST.E 		  * pitchendlink_maininsertionlinkST.r;
	Vector3d p_maininsertionlink_toollink_world     = world_maininsertionlinkST.E * maininsertionlink_toollinkST.r;
	//1--------------------------------------------------------------------//
	Joint joint_base = 
    Joint(JointTypeFixed);
	world_baselinkId = rbdlModelPtr_-> 
		// AddBody(0, Xtrans(world_baselinkST.r), Joint(JointTypeEulerZYX), baselinkBody_, "world-baselink");
		AddBody(0, Xtrans(world_baselinkST.r), joint_base, baselinkBody_, "world-baselink");
	
	baselink_yawlinkId = rbdlModelPtr_->
		AddBody(world_baselinkId, Xtrans(p_baselink_yawlink_world), 
		Joint(SpatialVector (0., -1., 0., 0., 0., 0.)), yawlinkBody_, "baselink-yawlink");
	
	yawlink_pitchbacklinkId = rbdlModelPtr_->
		AddBody(baselink_yawlinkId, Xtrans(p_yawlink_pitchbacklink_world), 
		Joint(SpatialVector (-1., 0., 0., 0., 0., 0.)), pitchbacklinkBody_, "yawlink-pitchbacklink");

	pitchbacklink_pitchbottomlink_v_Id = rbdlModelPtr_->
		AddBody(yawlink_pitchbacklinkId, Xtrans(p_pitchbacklink_pitchbottomlink_world), 
		Joint(SpatialVector (-1., 0., 0., 0., 0., 0.)), pitchbottomlinkBody_, "pitchbacklink-pitchbottomlink");

	pitchbottomlink_pitchendlinkId = rbdlModelPtr_->
		AddBody(pitchbacklink_pitchbottomlink_v_Id, Xtrans(p_pitchbottomlink_pitchendlink_world), 
		Joint(SpatialVector (-1., 0., 0., 0., 0., 0.)), pitchendlinkBody_, "pitchbottomlink-pitchendlink");
	
	pitchendlink_maininsertionlinkId = rbdlModelPtr_->
		AddBody(pitchbottomlink_pitchendlinkId, Xtrans(p_pitchendlink_maininsertionlink_world), 
		Joint(SpatialVector (0., 0., 0., 0., 0., -1.)), pitchendlinkBody_, "pitchendlink-maininsertionlink");

	maininsertionlink_toollinkId = rbdlModelPtr_->
		AddBody(pitchendlink_maininsertionlinkId, Xtrans(p_maininsertionlink_toollink_world), 
		Joint(SpatialVector (0., 0., 1., 0., 0., 0.)), pitchendlinkBody_, "maininsertionlink-toollink");
	//--------------------------------------------------------------------//
// 	unsigned int userDefinedId = 7;
// 	cs_.AddLoopConstraint(yawlink_pitchfrontlinkId, pitchbacklink_pitchbottomlink_v_Id, X_p_, X_s_, 
// 		SpatialVector(0, 0, 0, 1, 0, 0), bgStab_, 0.1, "LoopXY_Rz", userDefinedId);

// 	//These two constraints below will be appended to the above
// 	//constraint by default, and will assume its name and user defined id
// 	cs_.AddLoopConstraint(yawlink_pitchfrontlinkId, pitchbacklink_pitchbottomlink_v_Id, X_p_, X_s_, 
// 		SpatialVector(0, 0, 0, 0, 1, 0));
// 	cs_.AddLoopConstraint(yawlink_pitchfrontlinkId, pitchbacklink_pitchbottomlink_v_Id, X_p_, X_s_, 
// 		SpatialVector(0, 0, 1, 0, 0, 0));
// 	cs_.Bind(*rbdlModel_);

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
    std::cout << parentName << ", " << bodyName << ", " << bodyId << std::endl;
  }
}

void ECM::PrintModelHierarchy()
{
	for(unsigned int bodyId = 0; bodyId < rbdlModelPtr_->q_size; bodyId++)
	{
    std::string bodyName = rbdlModelPtr_->GetBodyName(bodyId);
	  std::string parentName = rbdlModelPtr_->GetBodyName(rbdlModelPtr_->GetParentBodyId(bodyId));
    std::cout << parentName << ", " << bodyName << ", " << bodyId << std::endl;
  }
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

int ECM::QIndexFromJointName(const std::string jointName)
{
	unsigned int bodyId = rbdlModelPtr_->GetBodyId(jointName.c_str());
	if(bodyId < 1 || bodyId > rbdlModelPtr_->q_size) return -1;

	return --bodyId;

}

bool ECM::JointAngleWithName(const std::string jointName, double qDesired)
{
	int qId = QIndexFromJointName(jointName);
	if(qId == -1) return false;
	Q_[qId] = qDesired;
	return true;
}

bool ECM::ExecutePoseInAMBF()
{
	baselinkHandler_ = ambfParamMap_[baselinkName_]->RididBodyHandler();

	if(baselinkHandler_ == nullptr)
	{
		std::cout << "baselinkHandler is null, poses cannot be executed. Terminating execution\n";
		return false;
	}

	std::cout << "Q_" << std::endl << Q_ << std::endl;

	std::vector<std::string> controllableJoints = baselinkHandler_->get_joint_names();
  for(int i = 0; i < 10; i++)
  {
		for(std::string jointName : controllableJoints)
		{
			// std::cout << "ExecutePoseInAMBF() - jointName: " << jointName << QIndexFromJointName(jointName) << std::endl;
			int qIndex = QIndexFromJointName(jointName);
			if (qIndex == -1) return false; 
			baselinkHandler_->
				set_joint_pos<std::string>(jointName, Q_(qIndex));
		}
    usleep(sleepTime);
	
    RegisterRigidBodysPose();
  }

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

bool ECM::ExecutePose()
{
	// Q_ = Q;
	// Q_[0] = world_base_yaw_; // Z
  // Q_[1] = world_base_pitch_; // Y
  // Q_[2] = world_base_roll_; // X

	return ExecutePoseInAMBF();
}


template <>
t_w_nPtr ECM::twnFromModels(std::string jointName)
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
t_w_nPtr ECM::twnFromModels(unsigned int jointId)
{
	std::string jointName = rbdlModelPtr_->GetBodyName(jointId);
	return twnFromModels(jointName);
}

std::vector<t_w_nPtr> ECM::HomePoseTransformation()
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

	// This can be written over a loop if needed. Update RBDL home pose roation to be
	// the values calcuated.
	transformationsItr = std::find_if(transformations.begin(), transformations.end(),
		[&](t_w_nPtr const & t_w_nptr) {return t_w_nptr->bodyName == "baselink";});
	if ( transformationsItr != transformations.end() )
	{
		(*transformationsItr)->r_w_n_rbdl = world_baselinkST.E;
	}

	transformationsItr = std::find_if(transformations.begin(), transformations.end(),
		[&](t_w_nPtr const & t_w_nptr) {return t_w_nptr->bodyName == "yawlink";});
	if ( transformationsItr != transformations.end() )
	{
		(*transformationsItr)->r_w_n_rbdl = world_yawlinkST.E;
	}

	transformationsItr = std::find_if(transformations.begin(), transformations.end(),
		[&](t_w_nPtr const & t_w_nptr) {return t_w_nptr->bodyName == "pitchbacklink";});
	if ( transformationsItr != transformations.end() )
	{
		(*transformationsItr)->r_w_n_rbdl = world_pitchbacklinkST.E;
	}

	transformationsItr = std::find_if(transformations.begin(), transformations.end(),
		[&](t_w_nPtr const & t_w_nptr) {return t_w_nptr->bodyName == "pitchbottomlink";});
	if ( transformationsItr != transformations.end() )
	{
		(*transformationsItr)->r_w_n_rbdl = world_pitchbottomlinkST.E;
	}

	transformationsItr = std::find_if(transformations.begin(), transformations.end(),
		[&](t_w_nPtr const & t_w_nptr) {return t_w_nptr->bodyName == "pitchendlink";});
	if ( transformationsItr != transformations.end() )
	{
		(*transformationsItr)->r_w_n_rbdl = world_pitchendlinkST.E;
	}

	transformationsItr = std::find_if(transformations.begin(), transformations.end(),
		[&](t_w_nPtr const & t_w_nptr) {return t_w_nptr->bodyName == "maininsertionlink";});
	if ( transformationsItr != transformations.end() )
	{
		(*transformationsItr)->r_w_n_rbdl = world_maininsertionlinkST.E;
	}

	transformationsItr = std::find_if(transformations.begin(), transformations.end(),
		[&](t_w_nPtr const & t_w_nptr) {return t_w_nptr->bodyName == "toollink";});
	if ( transformationsItr != transformations.end() )
	{
		(*transformationsItr)->r_w_n_rbdl = world_toollinkST.E;
	}

	return transformations;
}
void ECM::CleanUp()
{
	this->~ECM();
}

ECM::~ECM() 
{
	// CleanUp();

	if(baselinkHandler_ != nullptr) baselinkHandler_->cleanUp();
	if(ambfClientPtr_ != nullptr) ambfClientPtr_->cleanUp();
	if(rbdlModelPtr_ != nullptr) delete rbdlModelPtr_;
	if(ambfClientPtr_ != nullptr) delete ambfClientPtr_;
}