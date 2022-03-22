#include <unordered_map>

#include "rbdl_model_tests/RBDLTestPrep.h"
#include "rbdl_model_tests/EigenUtilities.h"
#include "rbdl_model_tests/AMBFParams.h"

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;
typedef AMBFParams* AMBFParamsPtr;

//const double TEST_PREC = 1.0e-12;
const double TEST_LAX = 1.0e-7;

class ECM {
public:
  ECM();
  ~ECM();
  const Matrix3d PrintAMBFTransformation();
  // const Vector3d TranslationVectorTF();

  inline Model* GetRBDLModel() { return rbdlModel_; }
  void CleanUp();
private:
  void ConnectToAMBF();
  void SetAMBFParams();
  void ExecutePoseInAMBF();

  void SetBodyParams();
  void CreateRBDLJoint(Vector3d& PA, Vector3d& CA, Vector3d& PP, Vector3d& CP, const double offsetQ, 
	Vector3d axis, unsigned int parentId, Joint joint, JointType jointType,  SpatialTransform	world_parentST, Body &body, 
  std::string bodyName, unsigned int& newBodyId, SpatialTransform&	world_bodyST);
  void CreateRBDLModel();
  void CheckRBDLModel();
private:
  AMBFClientPtr ambfClientPtr_ = nullptr;
  std::string baselinkName_ = "ecm/baselink";

  // Format of the map yet to be decided
  std::unordered_map<std::string, AMBFParamsPtr> ambfParamMap_;
  std::unordered_map<std::string, AMBFParamsPtr>::iterator ambfParamMapItr_;

  Model* rbdlModel_ = nullptr;
  ConstraintSet cs_;
  VectorNd Q_;
  VectorNd QDot_;
  VectorNd QDDot_;
  VectorNd Tau_;
  MatrixNd t_w_0_ = MatrixNd::Identity(4, 4);
  MatrixNd t_0_w_ = MatrixNd::Identity(4, 4);

  Body baselinkBody_, pitchendlinkBody_, maininsertionlinkBody_, toollinkBody_, yawlinkBody_, 
    pitchbacklinkBody_, pitchbottomlinkBody_, pitchfrontlinkBody_, pitchtoplinkBody_;
};