#include "rbdl_model/BuildRBDLModel.h"
#include "rbdl_model/GraphEdge.h"

BuildRBDLModel::BuildRBDLModel(std::string actuator_config_file) 
{
	try
	{
		if(!ConnectToAMBF()) Utilities::ThrowAMBFInactiveException();
	}
	catch(const char* e)
	{
		std::exit(EXIT_FAILURE); 		
	}

  parseAdf_ = new ParseADF(actuator_config_file);
  baseRigidBodyName_ = parseAdf_->BaseName();
  endEffectorNodesName_ = parseAdf_->EndEffectorsName();
  paths_ = parseAdf_->Paths();

  this->SetAMBFParams();
  this->RegisterHomePoseTransformation();
  this->BuildModel();
}

bool BuildRBDLModel::ConnectToAMBF()
{
	ambfClientPtr_ = AMBFTestPrep::getInstance()->getAMBFClientInstance();
	
	if(!ambfClientPtr_->connect()) return false;
	usleep(1000000);

	return true;
}

AMBFParamsPtr BuildRBDLModel::FetchFromAMBFParamMap(const std::string parentBodyName) 
{
	ambfParamMapItr_ = ambfParamMap_.find(parentBodyName);
	if(ambfParamMapItr_ == ambfParamMap_.end())
    Utilities::ThrowKeyNotFoundException("ambfParamMapItr", parentBodyName);

	AMBFParamsPtr rigidBodyParams = ambfParamMap_[parentBodyName];

	return rigidBodyParams;
}

void BuildRBDLModel::RegisterBodyToWorldTransformation(const std::string parentBodyName)
{
	AMBFParamsPtr ambfRigidBodyParams = FetchFromAMBFParamMap(parentBodyName);
	rigidBodyPtr rigidBodyHandler = ambfRigidBodyParams->RididBodyHandler();

	tf::Quaternion quat_w_n_tf = rigidBodyHandler->get_rot();
	tf::Vector3 p_w_n_tf = rigidBodyHandler->get_pos();
	ambfRigidBodyParams->QuaternionTF(quat_w_n_tf);
	ambfRigidBodyParams->TranslationVectorTF(p_w_n_tf);
	ambfParamMap_[parentBodyName] = ambfRigidBodyParams;
	// printf("parent: %s, Translation: (%f, %f, %f)\n", parentBody.c_str(), p_w_n_tf[0], p_w_n_tf[1], p_w_n_tf[2]);
}

void BuildRBDLModel::SetAMBFParams()
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
			rigidBodyName, ambfClientPtr_->getRigidBody(rigidBodyName.c_str(), true)))
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
	RegisterBodyToWorldTransformation(baseRigidBodyName_);
	baselinkHandler_ = ambfParamMap_[baseRigidBodyName_]->RididBodyHandler();
	controlableJoints_ = baselinkHandler_->get_joint_names();
}

void BuildRBDLModel::RegisterRigidBodysPose()
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

void BuildRBDLModel::RegisterHomePoseTransformation()
{
 for(int i = 0; i < 10; i++)
  {
		for(std::string jointName : controlableJoints_)
		{
			baselinkHandler_->
				set_joint_pos<std::string>(jointName, 0.0f);
		}
    usleep(sleepTime);
	
    RegisterRigidBodysPose();
  }   
}



/*
 * Build RBDL Model
 * @return true after successful RBDL model creation
 */
bool BuildRBDLModel::BuildModel() 
{
	// std::cout << "print ambfParamMap_\n";
  // for(ambfParamMapItr_ = ambfParamMap_.begin(); ambfParamMapItr_ != ambfParamMap_.end(); ambfParamMapItr_++)
	// {
	// 	std::string rigidBodyName = ambfParamMapItr_->first;
	// 	printf("%s\n", rigidBodyName.c_str());
	// }
	
	rbdl_check_api_version(RBDL_API_VERSION);

  rbdlModelPtr_ = new Model();
  rbdlModelPtr_->gravity = Vector3d(0., 0., -9.81); // in my case should set in the Z direction

  // Joint jointType = Joint(JointTypeFixed);
  unsigned int parentBodyId = 0;

  std::vector<std::string> path = paths_.at(0);
	path.insert(path.begin(), "world");

  for(int i = 0; i < path.size() - 1; i++)
  {
		//--------------------------------------------------------------------//
		// Collect all data required for joint creation
    std::string parentRigidBodyName = path.at(i); 
    std::string childRigidBodyName = path.at(i + 1);
    std::string jointName = parentRigidBodyName + "-" + childRigidBodyName;

    // printf("parentName: %s, childName: %s, jointName: %s\n", 
    //   parentRigidBodyName.c_str(), childRigidBodyName.c_str(), jointName.c_str());

    bodyParamPtr childParamPtr = parseAdf_->BodyParams(childRigidBodyName);
    
    // mass, com - inertia offset, inertia
    Body childBody = 
      Body(childParamPtr->Mass(), childParamPtr->InertialOffsetPosition(), childParamPtr->Inertia());

		SpatialTransform world_parentST;
		SpatialTransform world_childST;
		SpatialTransform parent_childST;
		Vector3d p_parent_child_world;
		Joint jointType;
		// parent is world
		if(parentBodyId == 0)
		{
			AMBFParamsPtr ambfRigidBodyParams = FetchFromAMBFParamMap(childRigidBodyName);
			world_childST.E = ambfRigidBodyParams->RotationMatrix();
			world_childST.r = ambfRigidBodyParams->TranslationVector();

			jointType = Joint(JointTypeFixed);
			p_parent_child_world = world_childST.r;

			world_parentST = world_childST;
		}
		else
		{
    	jointParamPtr jointParamPtr = parseAdf_->JointParams(jointName);
			
			Vector3d parentAxis =	jointParamPtr->ParentAxis(); parentAxis.normalize();
			Vector3d childAxis = jointParamPtr->ChildAxis(); childAxis.normalize();
			Vector3d parentPivot = jointParamPtr->ParentPivot();
			Vector3d childPivot = jointParamPtr->ChildPivot();
			const double paret_childOffsetQ = jointParamPtr->Offset();

			// Parent to child body rotation calculations
			Matrix3d parent_childRot = 
			Eigen::Matrix3d(Eigen::Quaterniond::FromTwoVectors(parentAxis, childAxis));
			
			// TBD axis of rotation to be fould out
			Eigen::Affine3d 
				parent_childRotOffset(Eigen::AngleAxisd(paret_childOffsetQ, -Vector3d::UnitZ()));
				
			// SpatialTransform parent_childST;
			parent_childST.E = parent_childRot.transpose() * parent_childRotOffset.rotation();
			parent_childST.r = 
				parentPivot - (parent_childRot.transpose() * childPivot);


			// TBD: joint type to got fetched from world to body rotation
			jointType = Joint(JointTypeRevoluteX);
			p_parent_child_world = world_parentST.E * parent_childST.r;
		}

		// Joint Axis to be got from ration matrix
		unsigned int childBodyId = rbdlModelPtr_->AddBody(parentBodyId, Xtrans(p_parent_child_world), 
		Joint(SpatialVector (0., -1., 0., 0., 0., 0.)), childBody, jointName);

		world_childST = world_childST * parent_childST;
		parentBodyId = childBodyId;
  }
  return true;
}

/*
 * Get all body Names in RBDL model
 * @return vector of body names
 */
std::vector<std::string> BuildRBDLModel::GetAllBodyNames() {
  std::vector<std::string> bodyNames;
  std::transform (bodyParamObjectMap_.begin(), bodyParamObjectMap_.end(), back_inserter(bodyNames), [] (std::pair<std::string, bodyParamPtr> const & pair)
  {
  return pair.first;

  });

  return bodyNames;
}

/*
 * Clean up memory
 */
void BuildRBDLModel::CleanUp() {
  // std::unordered_map<std::string, bodyParamPtr>::iterator bodyParamMapIt;
  // for ( bodyParamMapIt = bodyParamObjectMap_.begin(); bodyParamMapIt != bodyParamObjectMap_.end(); ++bodyParamMapIt ) {
  //   bodyParamMapIt->second->~BodyParam();
  // }

  // std::unordered_map<std::string, std::unordered_map<std::string, jointParamPtr> >::iterator itr;
  // std::unordered_map<std::string, jointParamPtr>::iterator ptr;

  // for (itr = jointParamObjectMap_.begin(); itr != jointParamObjectMap_.end(); itr++) {
  //   for (ptr = itr->second.begin(); ptr != itr->second.end(); ptr++) {
  //     ptr->second->~JointParam();
  //   }
  // }

  // delete RBDLmodel_;
  std::cout << "RBDL Model deleted" << std::endl;
}

BuildRBDLModel::~BuildRBDLModel(void){
  delete rbdlModelPtr_;
}
