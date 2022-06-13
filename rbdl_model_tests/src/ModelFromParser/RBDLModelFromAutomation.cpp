#include "rbdl_model_tests/ModelFromParser/RBDLModelFromAutomation.h"


RBDLModelFromAutomation::RBDLModelFromAutomation() 
{
	buildRBDLModelPtr_ = new BuildRBDLModel(AMBFTestPrep::ADFPath());
	rbdlModelPtr_ = buildRBDLModelPtr_->RBDLModel();

	Q_     = VectorNd::Constant ((size_t) rbdlModelPtr_->dof_count, 0.);
	QDot_  = VectorNd::Constant ((size_t) rbdlModelPtr_->dof_count, 0.);
	QDDot_ = VectorNd::Constant ((size_t) rbdlModelPtr_->dof_count, 0.);
	Tau_   = VectorNd::Constant ((size_t) rbdlModelPtr_->dof_count, 0.);
	rbdlmBodyMap_ = rbdlModelPtr_->mBodyNameMap;
}

void RBDLModelFromAutomation::PrintModelHierarchy()
{
	for(unsigned int bodyId = 0; bodyId < rbdlModelPtr_->q_size; bodyId++)
	{
    std::string bodyName = rbdlModelPtr_->GetBodyName(bodyId);
	  std::string parentName = rbdlModelPtr_->GetBodyName(rbdlModelPtr_->GetParentBodyId(bodyId));
    std::cout << parentName << ", " << bodyName << ", " << bodyId << std::endl;
  }
}

/*
template <>
t_w_nPtr RBDLModelFromAutomation::twnFromModels(std::string jointName)
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
t_w_nPtr RBDLModelFromAutomation::twnFromModels(unsigned int jointId)
{
	std::string jointName = rbdlModelPtr_->GetBodyName(jointId);
	return twnFromModels(jointName);
}
*/

RBDLModelFromAutomation::~RBDLModelFromAutomation() 
{
	// CleanUp();
	delete rbdlModelPtr_;
	delete buildRBDLModelPtr_;

}