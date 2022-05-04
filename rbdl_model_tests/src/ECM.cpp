#include "rbdl_model_tests/ECM.h"
#include "rbdl_model_tests/EigenUtilities.h"

ECM::ECM() 
{
	if(!ConnectToAMBF()) return;
	SetAMBFParams();
	SetBodyParams();
	CreateRBDLModel();
}

bool ECM::ConnectToAMBF()
{
	ambfClientPtr_ = RBDLTestPrep::getInstance()->getAMBFClientInstance();
	
	if(!ambfClientPtr_->connect()) return false;
	usleep(1000000);

	// baselinkHandler_ = ambfClientPtr_->getRigidBody(baselinkName_.c_str(), true);
	// usleep(250000);

	// // Initialize handler
	// baselinkHandler_->set_joint_pos<int>(0, 0.0f);

	// controlableJoints = baselinkHandler_->get_joint_names();
	// baselinkChildren = baselinkHandler_->get_children_names();

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
	// // printf("---------------------------------\n");
	// // Initialize all the handlers
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
	// t_w_0_.block<3, 3>(0, 0) = ambfParamMap_[baselinkName_]->RotationMatrix();
	// t_w_0_.block<3, 1>(0, 3) = ambfParamMap_[baselinkName_]->TranslationVector();

	// t_0_w_.block<3, 1>(0, 3) = -t_w_0_.block<3, 3>(0, 0).transpose() * t_w_0_.block<3, 1>(0, 3);
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

	Vector3d baselink_yawlinkPA 							= { -0.0002, -1.0000, 00.0000 };
	Vector3d baselink_yawlinkCA 							= { 00.0000, 00.0000, -1.0000 };
	Vector3d baselink_yawlinkPP 							= { 00.0000, 00.0000, 00.0000 };
	Vector3d baselink_yawlinkCP 							= { 00.0001, 00.0000, 00.5369 };

	Vector3d yawlink_pitchfrontlinkPA 				= { 1.0, 0.0,   0.0 };
	Vector3d yawlink_pitchfrontlinkCA 				= { 0.0, 0.0,   1.0 };
	Vector3d yawlink_pitchfrontlinkPP 				= { 0.0, 0.0, 0.199 };
	Vector3d yawlink_pitchfrontlinkCP 				= { 0.0, 0.0,   0.0 };

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

	Vector3d p_world_baselink   				= Vector3d::Zero();
	Vector3d p_world_toollink	 		 			= Vector3d::Zero();
	Vector3d p_world_maininsertionlink	= Vector3d::Zero();
	Vector3d p_world_pitchendlink	 	 		= Vector3d::Zero();
	Vector3d p_world_pitchbottomlink	 	= Vector3d::Zero();
	Vector3d p_world_pitchbacklink	 		= Vector3d::Zero();
	Vector3d p_world_pitchfrontlink	 		= Vector3d::Zero();
	Vector3d p_world_yawlink						= Vector3d::Zero();

	rbdlModelPtr_ = new Model;
	rbdlModelPtr_->gravity = Vector3d(0., 0., -9.81);

// CreateRBDLJoint(RBDLModelPtr rbdlModelPtr, Vector3d& pa, Vector3d& ca, const Vector3d& pp, 
//     const Vector3d& cp, const double offsetQ, const Vector3d axis, const unsigned int parentId, const Joint joint, 
//     const SpatialTransform world_parentST, const Body &body, const std::string bodyName, unsigned int& childId, 
//     SpatialTransform&	world_childST)

	// Register Word to Base Transform
	AMBFParamsPtr ambfRigidBodyParams = ambfParamMap_[baselinkName_];

	world_baselinkST.E = ambfRigidBodyParams->RotationMatrix();
	world_baselinkST.r = ambfRigidBodyParams->TranslationVector();
	//1--------------------------------------------------------------------//
	// EigenUtilities::CreateRBDLJoint(rbdlModelPtr_, baselink_yawlinkPA, baselink_yawlinkCA, 
	// 	baselink_yawlinkPP, baselink_yawlinkCP, baselink_yawlinkOffsetQ, Vector3d::UnitZ(), 
	// 	0, Joint(SpatialVector (0., 1., 0., 0., 0., 0.)), world_baselinkST, baselinkBody_, 
	// 	"baselink-yawlink", baselink_yawlinkId, world_yawlinkST);
	baselink_yawlinkPA.normalize();
	baselink_yawlinkCA.normalize();


	Matrix3d baselink_yawlinkRot = 
	Eigen::Matrix3d(Eigen::Quaterniond::FromTwoVectors(baselink_yawlinkPA, baselink_yawlinkCA));
	Eigen::Affine3d baselink_yawlinkRotOffset(Eigen::AngleAxisd(baselink_yawlinkOffsetQ, Vector3d::UnitZ()));
		
	SpatialTransform baselink_yawlinkST;
	baselink_yawlinkST.E = baselink_yawlinkRotOffset.rotation() * baselink_yawlinkRot;
	baselink_yawlinkST.r = 
		baselink_yawlinkPP - (baselink_yawlinkRot.inverse() * baselink_yawlinkCP);

	world_yawlinkST = world_baselinkST * baselink_yawlinkST;

	p_world_yawlink =  (world_baselinkST * (world_baselinkST * baselink_yawlinkST)).r;
	// if(parentId == 0) p_world_child = world_childST.r;

	baselink_yawlinkId = rbdlModelPtr_->AddBody(0, Xtrans(p_world_yawlink), 
		Joint(SpatialVector (0., 1., 0., 0., 0., 0.)), 
		baselinkBody_, "baselink-yawlink");
	//--------------------------------------------------------------------//
	// EigenUtilities::CreateRBDLJoint(rbdlModelPtr_, yawlink_pitchfrontlinkPA, yawlink_pitchfrontlinkCA, 
	// 	yawlink_pitchfrontlinkPP, yawlink_pitchfrontlinkCP, yawlink_pitchfrontlinkOffsetQ, Vector3d::UnitY(), 
	// 	baselink_yawlinkId, Joint(SpatialVector (0., 0., 0., 0., 0., 0.)), world_yawlinkST, yawlinkBody_, 
	// 	"yawlink-pitchfrontlink", yawlink_pitchfrontlinkId, world_pitchfrontlinkST);
	//1--------------------------------------------------------------------//
	// yawlink_pitchbacklinkPA.normalize();
	// yawlink_pitchbacklinkCA.normalize();

	// Matrix3d yawlink_pitchbacklinkRot = 
	// Eigen::Matrix3d(Eigen::Quaterniond::FromTwoVectors(yawlink_pitchbacklinkPA, yawlink_pitchbacklinkCA));
	// Eigen::Affine3d yawlink_pitchbacklinkRotOffset(
	// 	Eigen::AngleAxisd(yawlink_pitchbacklinkOffsetQ, Vector3d::UnitX()));
		
	// SpatialTransform yawlink_pitchbacklinkST;
	// yawlink_pitchbacklinkST.E = yawlink_pitchbacklinkRotOffset.rotation() * yawlink_pitchbacklinkRot;
	// yawlink_pitchbacklinkST.r = 
	// 	yawlink_pitchbacklinkPP - (yawlink_pitchbacklinkRot.inverse() * yawlink_pitchbacklinkCP);

	// world_pitchbacklinkST = world_yawlinkST * yawlink_pitchbacklinkST;

	// p_world_pitchbacklink =  world_yawlinkST.E.inverse() * yawlink_pitchbacklinkST.r;
	// // if(parentId == 0) p_world_child = world_childST.r;

	// yawlink_pitchbacklinkId = rbdlModelPtr_->AddBody(baselink_yawlinkId, Xtrans(p_world_pitchbacklink), 
	// 	Joint(SpatialVector (0., 0., 1., 0., 0., 0.)), 
	// 	yawlinkBody_, "yawlink-pitchbacklink");

	std::cout << "world_baselinkST" << std::endl << world_baselinkST << std::endl << std::endl;
	std::cout << "baselink_yawlinkST" << std::endl << baselink_yawlinkST << std::endl << std::endl;
	std::cout << "world_yawlinkST" << std::endl << world_yawlinkST << std::endl << std::endl;
	// std::cout << "world_pitchbacklinkST" << std::endl << world_pitchbacklinkST << std::endl << std::endl;

	std::cout << "p_world_yawlink" << std::endl << p_world_yawlink << std::endl << std::endl;
	// std::cout << "p_world_pitchbacklink" << std::endl << p_world_pitchbacklink << std::endl << std::endl;

	// EigenUtilities::CreateRBDLJoint(rbdlModelPtr_, baselink_yawlinkPA, baselink_yawlinkCA, 
	// baselink_yawlinkPP, baselink_yawlinkCP, baselink_yawlinkOffsetQ, 
	// Vector3d::UnitZ(), 0, Joint(SpatialVector (0., 0., 1., 0., 0., 0.)), 
	// world_baselinkST, baselinkBody_, "baselink-yawlink", baselink_yawlinkId, world_baselinkST);

	// EigenUtilities::CreateRBDLJoint(rbdlModelPtr_, baselink_yawlinkPA, baselink_yawlinkCA, baselink_yawlinkPP, baselink_yawlinkCP, 
	// baselink_yawlinkOffsetQ, Vector3d::UnitZ(), 0, p_world_baselink, Joint(SpatialVector (0., 1., 0., 0., 0., 0.)),
	// world_baselinkST, baselinkBody_, "baselink-yawlink", p_world_yawlink, baselink_yawlinkId, 
	// world_yawlinkST);
	//2 and 3 --------------------------------------------------------------------//
	// EigenUtilities::CreateRBDLJoint(rbdlModelPtr_, yawlink_pitchfrontlinkPA, yawlink_pitchfrontlinkCA, 
	// yawlink_pitchfrontlinkPP, yawlink_pitchfrontlinkCP, yawlink_pitchfrontlinkOffsetQ, 
	// Vector3d::UnitZ(), 0, Joint(SpatialVector (-1., 0., 0., 0., 0., 0.)), 
	// world_yawlinkST, yawlinkBody_, "yawlink-pitchfrontlink", yawlink_pitchfrontlinkId, world_pitchfrontlinkST);
// 	CreateRBDLJoint(yawlink_pitchfrontlinkPA, yawlink_pitchfrontlinkCA, yawlink_pitchfrontlinkPP, 
// 	yawlink_pitchfrontlinkCP, yawlink_pitchfrontlinkOffsetQ, Vector3d::UnitZ(), baselink_yawlinkId, 
// 	p_world_yawlink, Joint(SpatialVector (-1., 0., 0., 0., 0., 0.)), world_yawlinkST, yawlinkBody_, 
// 	"yawlink-pitchfrontlink", p_world_pitchfrontlink, yawlink_pitchfrontlinkId, world_pitchfrontlinkST);
// 	//3--------------------------------------------------------------------//

// 	CreateRBDLJoint(yawlink_pitchbacklinkPA, yawlink_pitchbacklinkCA, yawlink_pitchbacklinkPP, 
// 	yawlink_pitchbacklinkCP, yawlink_pitchbacklinkOffsetQ, Vector3d::UnitZ(), baselink_yawlinkId, 
// 	p_world_yawlink, Joint(SpatialVector (-1., 0., 0., 0., 0., 0.)), world_yawlinkST, yawlinkBody_, 
// 	"yawlink-pitchbacklink", p_world_pitchbacklink, yawlink_pitchbacklinkId, world_pitchbacklinkST);
//   //--------------------------------------------------------------------//
// 	// Virtual link

// 	CreateRBDLJoint(pitchbacklink_pitchbottomlinkPA, pitchbacklink_pitchbottomlinkCA, 
// 	pitchbacklink_pitchbottomlinkPP, pitchbacklink_pitchbottomlinkCP, pitchbacklink_pitchbottomlinkOffsetQ, 
// 	Vector3d::UnitZ(), yawlink_pitchbacklinkId, p_world_pitchbacklink, 
// 	Joint(SpatialVector (-1., 0., 0., 0., 0., 0.)), 
// 	world_pitchbacklinkST, virtualBody_, "pitchbacklink-pitchbottomlink", 
// 	p_world_pitchbottomlink, pitchbacklink_pitchbottomlink_v_Id, world_pitchbottomlinkST);
// 	//8--------------------------------------------------------------------//

// 	CreateRBDLJoint(pitchbottomlink_pitchendlinkPA, pitchbottomlink_pitchendlinkCA, 
// 	pitchbottomlink_pitchendlinkPP, pitchbottomlink_pitchendlinkCP, pitchbottomlink_pitchendlinkOffsetQ, 
// 	Vector3d::UnitZ(), pitchbacklink_pitchbottomlink_v_Id, p_world_pitchbottomlink, 
// 	Joint(SpatialVector (-1., 0., 0., 0., 0., 0.)), 
// 	world_pitchbottomlinkST, pitchbottomlinkBody_, "pitchbottomlink-pitchendlink", 
// 	p_world_pitchendlink, pitchbottomlink_pitchendlinkId, world_pitchendlinkST);
// 	//9--------------------------------------------------------------------//

// 	CreateRBDLJoint(pitchendlink_maininsertionlinkPA, pitchendlink_maininsertionlinkCA, 
// 	pitchendlink_maininsertionlinkPP, pitchendlink_maininsertionlinkCP, pitchendlink_maininsertionlinkOffsetQ, 
// 	-Vector3d::UnitZ(), pitchbottomlink_pitchendlinkId, p_world_pitchendlink, 
// 	Joint(SpatialVector (0., 0., 0., 0., 0., -1.)),
// 	world_pitchendlinkST, pitchendlinkBody_, "pitchendlink-maininsertionlink", 
// 	p_world_maininsertionlink, pitchendlink_maininsertionlinkId, world_maininsertionlinkST);
// 	//10--------------------------------------------------------------------//

// 	CreateRBDLJoint(maininsertionlink_toollinkPA, maininsertionlink_toollinkCA, 
// 	maininsertionlink_toollinkPP, maininsertionlink_toollinkCP, maininsertionlink_toollinkOffsetQ, 
// 	Vector3d::UnitZ(), pitchendlink_maininsertionlinkId, p_world_maininsertionlink, 
// 	Joint(SpatialVector (0., 0., 1., 0., 0., 0.)),
// 	world_maininsertionlinkST, maininsertionlinkBody_, "maininsertionlink-toollink", 
// 	p_world_toollink, maininsertionlink_toollinkId, world_toollinkST);
// //11--------------------------------------------------------------------//
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

void ECM::SetActualJointsAngleToRBDL()
{

}

bool ECM::ExecutePoseInAMBF(VectorNd Q)
{
	baselinkHandler_ = ambfParamMap_[baselinkName_]->RididBodyHandler();

	if(baselinkHandler_ == nullptr)
	{
		std::cout << "baselinkHandler is null, poses cannot be executed. Terminating execution\n";
		return false;
	}

  for(int i = 0; i < 10; i++)
  {
		baselinkHandler_->set_joint_pos<std::string>(               "baselink-yawlink", 0.0f);
		baselinkHandler_->set_joint_pos<std::string>(     		 "yawlink-pitchbacklink", 0.0f);
		baselinkHandler_->set_joint_pos<std::string>( "pitchendlink-maininsertionlink", 0.0f);
		baselinkHandler_->set_joint_pos<std::string>( 		"maininsertionlink-toollink", 0.0f);
    usleep(sleepTime);
	
    RegisterRigidBodysPose();
  }

	// // To be deleted
	// for(ambfParamMapItr_ = ambfParamMap_.begin(); ambfParamMapItr_ != ambfParamMap_.end(); ambfParamMapItr_++)
  // {
  //   std::string bodyName = ambfParamMapItr_->first;

	// 	Matrix3d r_w_n_ambf = ambfParamMap_[bodyName]->RotationMatrix();
	// 	Vector3d p_w_n_ambf = ambfParamMap_[bodyName]->TranslationVector();

	// 	std::cout << "bodyName: " << bodyName << std::endl;
	// 	std::cout << "r_w_n_ambf: " << std::endl << r_w_n_ambf << std::endl << std::endl;
	// 	std::cout << "p_w_n_ambf: " << std::endl << p_w_n_ambf << std::endl << std::endl;
  // }
  // std::cout << std::endl << "------------------" << std::endl;
	return true;
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

t_w_nPtr ECM::twnFromModels(std::string jointName)
{
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

	ForwardDynamics(*rbdlModelPtr_, Q_, QDot_, Tau_, QDDot_);
	unsigned int rbdlBodyId = rbdlModelPtr_->GetBodyId(jointName.c_str());

	t_w_nptr->r_w_n_rbdl = CalcBodyWorldOrientation(*rbdlModelPtr_, Q_, rbdlBodyId, true);
	t_w_nptr->p_w_n_rbdl = CalcBodyToBaseCoordinates(*rbdlModelPtr_, Q_, rbdlBodyId, Vector3d(0., 0., 0.), true);
	
	return t_w_nptr;
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