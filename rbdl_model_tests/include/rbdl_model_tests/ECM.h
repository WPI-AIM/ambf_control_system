#include <unordered_map>
#include <thread>

#include "rbdl_model_tests/RBDLTestPrep.h"
#include "rbdl_model_tests/AMBFParams.h"

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;
typedef AMBFParams* AMBFParamsPtr;
typedef std::unordered_map<std::string, AMBFParamsPtr> AMBFParamMap;
typedef std::pair<std::string, AMBFParamsPtr> AMBFParamPair;


//const double TEST_PREC = 1.0e-12;
const double TEST_LAX {1.0e-7};
const useconds_t sleepTime {250000};

struct JointValues
{
  std::string parent;
  float qActual;
};

typedef JointValues* JointValuesPtr;
typedef std::unordered_map<std::string, JointValuesPtr> JointValuesMap;
typedef std::pair<std::string, JointValuesPtr> JointValuesPair;

class ECM 
{
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
  void MapJoints(const std::string& parentBody, rigidBodyPtr rigidBodyHandler);
  void MapAMBFJointsToParent();
  void HelloThread();
  void RegisterRigidBodysPose();
  void RegisterJointsAngle();
  void ExecutePoseInAMBF();

  void SetBodyParams();
  void CreateRBDLJoint(Vector3d& PA, Vector3d& CA, Vector3d& PP, Vector3d& CP, const double offsetQ, 
	Vector3d axis, unsigned int parentId, Vector3d& p_world_startingbody, Joint joint, SpatialTransform	world_parentST, Body &body, 
  std::string bodyName, Vector3d& p_world_endingbody, unsigned int& newBodyId, SpatialTransform&	world_bodyST);
  void CreateRBDLModel();
  void SetRBDLPose();
  void CheckRBDLModel();

private:
  AMBFClientPtr ambfClientPtr_{nullptr};
  std::string baselinkName_{"ecm/baselink"};
  MatrixNd t_w_0_ = MatrixNd::Identity(4, 4);
  MatrixNd t_0_w_ = MatrixNd::Identity(4, 4);

  // Format of the map yet to be decided
  AMBFParamMap ambfParamMap_;
  AMBFParamMap::iterator ambfParamMapItr_;

  // RBDL use joint names to define a body. This corresponds to rigidbody in AMBF.
  // This give a need to map the RBDL joint name with the corresponing AMBF handler that would provied
  // the corresponding AMBF joint.
  // std::unordered_map<std::string, std::string> ambfJointToParentMap_;
  // std::unordered_map<std::string, std::string>::iterator ambfJointToParentMapItr_;
  JointValuesMap jointValuesMap_;
  JointValuesMap::iterator jointValuesMapItr_;

  Model* rbdlModel_{nullptr};
  ConstraintSet cs_;
  bool bgStab_{true};
  SpatialTransform X_p_;
  SpatialTransform X_s_;
  Body baselinkBody_, pitchendlinkBody_, maininsertionlinkBody_, toollinkBody_, yawlinkBody_, 
    pitchbacklinkBody_, pitchbottomlinkBody_, pitchfrontlinkBody_, pitchtoplinkBody_, virtualBody_;

  VectorNd Q_;
  VectorNd QDot_;
  VectorNd QDDot_;
  VectorNd Tau_;
  std::map< std::string, unsigned int > rbdlmBodyMap_;
  std::map<std::string, unsigned int>::iterator rbdlmBodyMapItr_;
};