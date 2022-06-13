#include "application/AMBFParamWrapper.h"

AMBFParamWrapper::AMBFParamWrapper() {}

void AMBFParamWrapper::AMBFParameter(std::string rigidBodyName, AMBFParamsPtr ambfParamsPtr)
{
  ambfParamMap_.insert(AMBFParamPair(rigidBodyName, ambfParamsPtr));
}

void AMBFParamWrapper::ActivateAMBFHandlers()
{
	// Activate all the handlers
	for(ambfParamMapItr_ = ambfParamMap_.begin(); ambfParamMapItr_ != ambfParamMap_.end(); ambfParamMapItr_++)
	{
		rigidBodyPtr handler = ambfParamMapItr_->second->RididBodyHandler();
				
		// Activate the rigid body if not active
		if(!handler->is_active())
		{
			handler->set_active();
		}
	}
}

AMBFParamsPtr AMBFParamWrapper::FetchFromAMBFParamMap(const std::string parentBodyName) 
{
	ambfParamMapItr_ = ambfParamMap_.find(parentBodyName);
	if(ambfParamMapItr_ == ambfParamMap_.end())
    Utilities::ThrowKeyNotFoundException("ambfParamMapItr", parentBodyName);

	AMBFParamsPtr rigidBodyParams = ambfParamMap_[parentBodyName];

	return rigidBodyParams;
}


void AMBFParamWrapper::RegisterBodyToWorldTransformation(const std::string parentBodyName)
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

rigidBodyPtr AMBFParamWrapper::RigidBodyHandler(const std::string parentBodyName)
{
  AMBFParamsPtr ambfRigidBodyParams = FetchFromAMBFParamMap(parentBodyName);
  return ambfParamMap_[parentBodyName]->RididBodyHandler();
}

void AMBFParamWrapper::RegisterRigidBodysPose()
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

SpatialTransform AMBFParamWrapper::T_W_N(const std::string rigidBodyName)
{
	SpatialTransform t_w_nST;
	AMBFParamsPtr ambfRigidBodyParams = FetchFromAMBFParamMap(rigidBodyName);
	t_w_nST.E = ambfRigidBodyParams->RotationMatrix();
	t_w_nST.r = ambfRigidBodyParams->TranslationVector();

	return t_w_nST;
}