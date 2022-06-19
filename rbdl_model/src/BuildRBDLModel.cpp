#include "rbdl_model/BuildRBDLModel.h"
#include "rbdl_model/GraphEdge.h"
#include "application/Prep.h"

BuildRBDLModel::BuildRBDLModel(std::string actuator_config_file, AMBFWrapperPtr ambfWrapperPtr) 
{
	// ambfWrapperPtr_ = new AMBFWrapper();
	// ambfWrapperPtr_ = AMBFTestPrep::getInstance()->getAMBFWrapperInstance();
	ambfWrapperPtr_ = ambfWrapperPtr;
  parseAdf_ = new ParseADF(actuator_config_file);
  baseRigidBodyName_ = parseAdf_->BaseName();
  endEffectorNodesName_ = parseAdf_->EndEffectorsName();
  paths_ = parseAdf_->Paths();

	// const std::string modelName = "ecm/";
	const std::string modelName = parseAdf_->ModelName();
	ambfWrapperPtr_->ActivateAMBFHandlers(modelName.c_str(), baseRigidBodyName_.c_str());
	ambfWrapperPtr_->RegisterHomePoseTransformation();

  this->BuildModel();
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

	// std::cout << "PrintAMBFfParamMap() from BuildModel()\n";
	ambfWrapperPtr_->PrintAMBFfParamMap();

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

    printf("parentName: %s, childName: %s, jointName: %s\n", 
      parentRigidBodyName.c_str(), childRigidBodyName.c_str(), jointName.c_str());

    bodyParamPtr childParamPtr = parseAdf_->BodyParams(childRigidBodyName);
    
    // mass, com - inertia offset, inertia
    Body childBody = 
      Body(childParamPtr->Mass(), childParamPtr->InertialOffsetPosition(), childParamPtr->Inertia());

		SpatialTransform world_parentST;
		SpatialTransform world_childST;
		SpatialTransform parent_childST;
		Vector3d p_parent_child_world;
		Joint joint;
		// parent is world
		if(parentBodyId == 0)
		{
			world_childST = ambfWrapperPtr_->T_W_N(childRigidBodyName);
			joint = Joint(JointTypeFixed);
			p_parent_child_world = world_childST.r;

			// world_parentST = world_childST;
		}
		else
		{
			world_parentST = ambfWrapperPtr_->T_W_N(parentRigidBodyName);
    	jointParamPtr jointParamPtr = parseAdf_->JointParams(jointName);
			
			Vector3d parentAxis =	jointParamPtr->ParentAxis(); parentAxis.normalize();
			Vector3d childAxis = jointParamPtr->ChildAxis(); childAxis.normalize();
			Vector3d parentPivot = jointParamPtr->ParentPivot();
			Vector3d childPivot = jointParamPtr->ChildPivot();
			const double paret_childOffsetQ = jointParamPtr->Offset();
			std::string jointType = jointParamPtr->Type();

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

			// std::cout << "world_parentST.E" << std::endl << world_parentST.E << std::endl;
			// std::cout << "parentAxis" << std::endl << parentAxis << std::endl;
			Vector3d jointAxis = world_parentST.E * parentAxis;
			// std::cout << "jointAxis" << std::endl << jointAxis << std::endl;

			// As of now only prismatic and revolute joints are supported
			if(jointType.compare("revolute") == 0) 
				joint = Joint(SpatialVector (jointAxis(0), jointAxis(1), jointAxis(2), 0., 0., 0.));
			else if(jointType.compare("prismatic") == 0)
				joint = Joint(SpatialVector (0., 0., 0., jointAxis(0), jointAxis(1), jointAxis(2)));
			else
				Utilities::ThrowUnsupportedJointException(jointName, jointType);

			p_parent_child_world = world_parentST.E * parent_childST.r;
		}

		// Joint Axis to be got from ration matrix
		unsigned int childBodyId = rbdlModelPtr_->AddBody(parentBodyId, Xtrans(p_parent_child_world), 
		joint, childBody, jointName);
		printf("Added jointName: %s, parentBodyId: %d, childBodyId: %d\n", jointName.c_str(), parentBodyId, childBodyId);

		// world_childST = world_childST * parent_childST;
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
  // std::cout << "RBDL Model deleted" << std::endl;
}

BuildRBDLModel::~BuildRBDLModel(void)
{

}
