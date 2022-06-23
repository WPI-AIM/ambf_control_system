#include "application/AMBFWrapper.h"

AMBFWrapper::AMBFWrapper() 
{
	ConnectToAMBF();
}

void AMBFWrapper::ConnectToAMBF()
{
	try
	{
		ambfClientPtr_ = new Client();
		if(!ambfClientPtr_->connect()) Utilities::ThrowAMBFInactiveException();
	}
	catch(const char* e)
	{
		std::exit(EXIT_FAILURE); 		
	}
	usleep(1000000);
}

AMBFParamsPtr AMBFWrapper::FetchFromAMBFParamMap(const std::string parentBodyName) 
{
	ambfParamMapItr_ = ambfParamMap_.find(parentBodyName);
	if(ambfParamMapItr_ == ambfParamMap_.end())
    Utilities::ThrowKeyNotFoundException("ambfParamMapItr", parentBodyName);

	AMBFParamsPtr rigidBodyParams = ambfParamMap_[parentBodyName];

	return rigidBodyParams;
}

void AMBFWrapper::ActivateAMBFHandlers(const std::string modelName, std::string baseRigidBodyName)
{
	SetAMBFParams(modelName);
	ActivateHandlers();

	baseRigidBodyName_ = baseRigidBodyName;
	baselinkHandler_ = FetchFromAMBFParamMap(baseRigidBodyName_)->RididBodyHandler();
	controlableJoints_ = baselinkHandler_->get_joint_names();
}

void AMBFWrapper::SetAMBFParams(const std::string modelName)
{
	std::vector<std::string> rigidBodyNames = ambfClientPtr_->getRigidBodyNames();

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
			rigidBodyName, ambfClientPtr_->getRigidBody(rigidBodyName.c_str(), true))));
	}
	usleep(sleepTime);
}

void AMBFWrapper::ActivateHandlers()
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

void AMBFWrapper::RegisterBodyToWorldTransformation(const std::string parentBodyName)
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

rigidBodyPtr AMBFWrapper::RigidBodyHandler(const std::string parentBodyName)
{
  AMBFParamsPtr ambfRigidBodyParams = FetchFromAMBFParamMap(parentBodyName);
  return ambfParamMap_[parentBodyName]->RididBodyHandler();
}

void AMBFWrapper::RegisterRigidBodysPose()
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

void AMBFWrapper::RegisterHomePoseTransformation()
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

SpatialTransform AMBFWrapper::T_W_N(const std::string rigidBodyName)
{
	SpatialTransform t_w_nST;
	AMBFParamsPtr ambfRigidBodyParams = FetchFromAMBFParamMap(rigidBodyName);
	t_w_nST.E = ambfRigidBodyParams->RotationMatrix();
	t_w_nST.r = ambfRigidBodyParams->TranslationVector();

	return t_w_nST;
}

void AMBFWrapper::PoseWithJointAngles(std::vector<double> desiredJointAngles)
{
	assert(controlableJoints_.size() == desiredJointAngles.size());

	for(int i = 0; i < 10; i++)
  {
		for(int jIndex = 0; jIndex < desiredJointAngles.size(); jIndex++)
		{
			baselinkHandler_->
				set_joint_pos<int>(jIndex, desiredJointAngles.at(jIndex));
		}
    usleep(sleepTime);
	
    RegisterRigidBodysPose();
  } 
}

void AMBFWrapper::PrintAMBFfParamMap()
{
	for(ambfParamMapItr_ = ambfParamMap_.begin(); ambfParamMapItr_ != ambfParamMap_.end(); ambfParamMapItr_++)
	{
		std::string rigidBodyName = ambfParamMapItr_->first;
		AMBFParamsPtr ambfRigidBodyParams = ambfParamMapItr_->second;
		SpatialTransform t_w_n;
		t_w_n.E = ambfRigidBodyParams->RotationMatrix();
		t_w_n.r = ambfRigidBodyParams->TranslationVector();

		printf("rigidBodyName: %s\n", rigidBodyName.c_str());
		std::cout << t_w_n << std::endl;
		printf("--------------------------\n");
	}
}

AMBFWrapper::~AMBFWrapper()
{
	for(ambfParamMapItr_ = ambfParamMap_.begin(); ambfParamMapItr_ != ambfParamMap_.end(); ambfParamMapItr_++)
	{
		AMBFParamsPtr ambfRigidBodyParams = ambfParamMapItr_->second;
		delete ambfRigidBodyParams;
	}

	ambfClientPtr_->cleanUp();
	ambfClientPtr_->~Client();
}