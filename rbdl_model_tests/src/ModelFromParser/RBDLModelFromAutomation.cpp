#include "rbdl_model_tests/ModelFromParser/RBDLModelFromAutomation.h"
#include "application/Prep.h"

RBDLModelFromAutomation::RBDLModelFromAutomation() 
{
	ambfWrapperPtr_ = AMBFTestPrep::getInstance()->getAMBFWrapperInstance();

	buildRBDLModelPtr_ = new BuildRBDLModel(AMBFTestPrep::ADFPath().c_str(), ambfWrapperPtr_);
	rbdlModelPtr_ = buildRBDLModelPtr_->RBDLModel();

	Q_     = VectorNd::Constant ((size_t) rbdlModelPtr_->dof_count, 0.);
	QDot_  = VectorNd::Constant ((size_t) rbdlModelPtr_->dof_count, 0.);
	QDDot_ = VectorNd::Constant ((size_t) rbdlModelPtr_->dof_count, 0.);
	Tau_   = VectorNd::Constant ((size_t) rbdlModelPtr_->dof_count, 0.);
	rbdlmBodyMap_ = rbdlModelPtr_->mBodyNameMap;

	// std::cout << "PrintAMBFfParamMap() from RBDLModelFromAutomation\n";
	// ambfWrapperPtr_->PrintAMBFfParamMap();
	controlableJoints_ = ambfWrapperPtr_->ControlableJoints();
}

void RBDLModelFromAutomation::PrintModelHierarchy()
{
	for(unsigned int bodyId = 0; bodyId < rbdlModelPtr_->q_size; bodyId++)
	{
    std::string bodyName = rbdlModelPtr_->GetBodyName(bodyId);
		bool isFixedBody = rbdlModelPtr_->IsFixedBodyId(bodyId);
	  std::string parentName = rbdlModelPtr_->GetBodyName(rbdlModelPtr_->GetParentBodyId(bodyId));
    std::cout << parentName << ", " << bodyName << ", " << bodyId << ", " << isFixedBody << std::endl;
  }
}

unsigned int RBDLModelFromAutomation::QIndexFromName(const std::string jointName)
{
	unsigned int bodyId = rbdlModelPtr_->GetBodyId(jointName.c_str());
	if(bodyId < 1 || bodyId > rbdlModelPtr_->q_size) return -1;

	return --bodyId;
}

std::vector<t_w_nPtr> RBDLModelFromAutomation::T_W_NfromModels(std::vector<double> desiredJointAngles)
{
	std::vector<t_w_nPtr> transformationsFromModels;
	// std::cout << "controlableJoints_.size(): " << controlableJoints_.size() << std::endl;

	for(std::string jointName : controlableJoints_)
	{
		// std::string jointName = "maininsertionlink-toollink";
		unsigned int rbdlBodyId = rbdlModelPtr_->GetBodyId(jointName.c_str());
		unsigned int qIndex = QIndexFromName(jointName);

		// std::cout << "jointName: " << jointName << std::endl;
		// joint_ name: link1-link2, parentName: link1
		t_w_nPtr t_w_nptr = new T_W_N();
		size_t npos = jointName.find("-", 0);
		if(npos == -1) return std::vector<t_w_nPtr> {};

		const std::string parentName = jointName.substr(npos + 1, jointName.size());
		// std::cout << "parentName: " << parentName << std::endl;

		t_w_nptr->bodyName = parentName;
		// Collect AMBF and RBDL Transformation Matrices
		// t_w_nptr->r_w_n_ambf = ambfParamMap_[parentName]->RotationMatrix();
		// t_w_nptr->p_w_n_ambf = ambfParamMap_[parentName]->TranslationVector();
		t_w_nptr->t_w_n_ambf = ambfWrapperPtr_->T_W_N(parentName);

		ForwardDynamics(*rbdlModelPtr_, Q_, QDot_, Tau_, QDDot_);

		// // printf("Joint: %s, rbdlBodyId: %d\n", jointName.c_str(), rbdlBodyId);
		t_w_nptr->t_w_n_rbdl.E = CalcBodyWorldOrientation(*rbdlModelPtr_, Q_, rbdlBodyId, true);
		t_w_nptr->t_w_n_rbdl.r = CalcBodyToBaseCoordinates(*rbdlModelPtr_, Q_, rbdlBodyId, Vector3d(0., 0., 0.), true);

		transformationsFromModels.emplace_back(t_w_nptr);
	}

	return transformationsFromModels;
}



RBDLModelFromAutomation::~RBDLModelFromAutomation() 
{
	// CleanUp();
	ambfWrapperPtr_->~AMBFWrapper();
	delete rbdlModelPtr_;
	delete buildRBDLModelPtr_;
}