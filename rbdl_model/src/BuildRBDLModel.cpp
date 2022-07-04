#include "rbdl_model/BuildRBDLModel.h"
#include "rbdl_model/GraphEdge.h"


BuildRBDLModel::BuildRBDLModel(std::string actuator_config_file) 
{
  parseAdf_ = new ParseADF(actuator_config_file);
	modelName_ = parseAdf_->ModelName();
  baseRigidBodyName_ = parseAdf_->BaseName();
  endEffectorNodesName_ = parseAdf_->EndEffectorsName();
  paths_ = parseAdf_->Paths();
  this->BuildModel();
}


const SpatialTransform BuildRBDLModel::T_Parent_ChildST(const Vector3d pp, const Vector3d cp,
  const Vector3d pa, const Vector3d ca, const double offsetQ)
{
  Matrix3d p_cRot = 
	Eigen::Matrix3d(Eigen::Quaterniond::FromTwoVectors(pa, ca));
	// RBDLUtilities::Round<Matrix3d>(p_cRot);

	Eigen::Affine3d p_cRotOffset(
    Eigen::AngleAxisd(offsetQ, ca));
	
	Matrix3d p_cRotOffsetToMatrix = p_cRotOffset.rotation();
	// RBDLUtilities::Round<Matrix3d>(p_cRotOffsetToMatrix);

  SpatialTransform parent_childST;
  parent_childST.E = 
    p_cRot.transpose() * p_cRotOffsetToMatrix;
	parent_childST.r = 
		pp - (p_cRot.transpose() * cp);
  
  return parent_childST;
}


/*
 * Build RBDL Model
 * @return true after successful RBDL model creation
 */
bool BuildRBDLModel::BuildModel() 
{
	rbdl_check_api_version(RBDL_API_VERSION);

  rbdlModelPtr_ = new Model();
  rbdlModelPtr_->gravity = Vector3d(0., 0., -9.81); // in my case should set in the Z direction

  unsigned int parentBodyId = 0;

  std::vector<std::string> path = paths_.at(0);
	path.insert(path.begin(), "world");
	
	Matrix3d world_parent;
	Matrix3d world_child;

  for(int i = 0; i < path.size() - 1; i++)
  {
		//--------------------------------------------------------------------//
		// Collect all data required for joint creation
    std::string parentRigidBodyName = path.at(i); 
    std::string childRigidBodyName = path.at(i + 1);
    std::string jointName = parentRigidBodyName + "-" + childRigidBodyName;

    bodyParamPtr childParamPtr = parseAdf_->BodyParams(childRigidBodyName);
    
    // mass, com - inertia offset, inertia
    Body childBody = 
      Body(childParamPtr->Mass(), childParamPtr->InertialOffsetPosition(), childParamPtr->Inertia());

		SpatialTransform parent_childST;
		Vector3d p_parent_child_world;

		if(jointName.compare("pitchendlink-maininsertionlink") == 0)
			std::cout << "jointName\n";
		Joint joint;
		// parent is world
		if(parentBodyId == 0)
		{
			bodyParamPtr bodyParamPtr = parseAdf_->BodyParams(childRigidBodyName);
			world_child = RBDLUtilities::RPYToMatrix(bodyParamPtr->LocationOrientation());

			joint = Joint(JointTypeFixed);
			p_parent_child_world = bodyParamPtr->LocationPosition();
		}
		else
		{
    	jointParamPtr jointParamPtr = parseAdf_->JointParams(jointName);
			
			Vector3d parentAxis =	jointParamPtr->ParentAxis(); parentAxis.normalize();
			Vector3d childAxis = jointParamPtr->ChildAxis(); childAxis.normalize();
			Vector3d parentPivot = jointParamPtr->ParentPivot();
			Vector3d childPivot = jointParamPtr->ChildPivot();
			const double paret_childOffsetQ = jointParamPtr->Offset();
			std::string jointType = jointParamPtr->Type();

			parent_childST = T_Parent_ChildST(parentPivot, childPivot,
  			parentAxis, childAxis, paret_childOffsetQ);

			Vector3d jointAxis = world_parent * parentAxis;
			if(jointType.compare("revolute") == 0) 
				joint = Joint(SpatialVector (jointAxis(0), jointAxis(1), jointAxis(2), 0., 0., 0.));
			else if(jointType.compare("prismatic") == 0)
				joint = Joint(SpatialVector (0., 0., 0., jointAxis(0), jointAxis(1), jointAxis(2)));
			else
				RBDLUtilities::ThrowUnsupportedJointException(jointName, jointType);

			// std::cout << "jointName: " << jointName << std::endl << "jointAxis: " << std::endl << jointAxis << std::endl;
			p_parent_child_world = world_parent * parent_childST.r;
			world_child = world_parent * parent_childST.E;
		}
		// RBDLUtilities::Round<Matrix3d>(world_child);

		// Joint Axis to be got from ration matrix
		unsigned int childBodyId = rbdlModelPtr_->AddBody(parentBodyId, Xtrans(p_parent_child_world), 
		joint, childBody, jointName);
		printf("Added jointName: %s, parentBodyId: %d, childBodyId: %d\n", jointName.c_str(), parentBodyId, childBodyId);
		std::cout << "p_parent_child_world" << std::endl << p_parent_child_world << std::endl;
		std::cout << "world_child" << std::endl << world_child << std::endl;
		
		world_parent = world_child;
		parentBodyId = childBodyId;

		std::cout << "---------------------\n";
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
  std::unordered_map<std::string, bodyParamPtr>::iterator bodyParamMapIt;
  for ( bodyParamMapIt = bodyParamObjectMap_.begin(); bodyParamMapIt != bodyParamObjectMap_.end(); ++bodyParamMapIt ) {
    bodyParamMapIt->second->~BodyParam();
  }

  std::unordered_map<std::string, std::unordered_map<std::string, jointParamPtr> >::iterator itr;
  std::unordered_map<std::string, jointParamPtr>::iterator ptr;

  for (jointParamObjectMapItr_ = jointParamObjectMap_.begin(); jointParamObjectMapItr_ != jointParamObjectMap_.end(); jointParamObjectMapItr_++) {
    for (ptr = itr->second.begin(); ptr != itr->second.end(); ptr++) {
      ptr->second->~JointParam();
    }
  }

  if(rbdlModelPtr_ != nullptr) delete rbdlModelPtr_;
  std::cout << "RBDL Model deleted" << std::endl;
}

BuildRBDLModel::~BuildRBDLModel(void)
{
	// CleanUp();
}
