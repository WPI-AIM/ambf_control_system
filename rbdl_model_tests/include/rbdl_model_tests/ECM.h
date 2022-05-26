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


struct T_W_N
{
  std::string bodyName;

  Matrix3d r_w_n_ambf;
  Vector3d p_w_n_ambf;

  Matrix3d r_w_n_rbdl;
  Vector3d p_w_n_rbdl;
};
typedef T_W_N* t_w_nPtr;

class ECM 
{
public:
  ECM();
  ~ECM();

  // To be deleted. Not to expose the model.
  // inline RBDLModelPtr RbdlModel() { return rbdlModelPtr_; }
  inline unsigned int RBDLModelJointSize() { return rbdlModelPtr_->q_size; }
  inline std::vector<std::string> ControllableJointNames() { return controlableJoints_; }
  bool JointAngleWithName(const std::string jointName, double qDesired);
  VectorNd inline TargetJointAngles() { return Q_; }
  bool ExecutePose();
  std::vector<t_w_nPtr> HomePoseTransformation();

  template<typename T>
  t_w_nPtr twnFromModels(T t);

  void CleanUp();
private:
  bool ConnectToAMBF();
  void RegisterBodyToWorldTransformation(const std::string parentBody);
  void SetAMBFParams();
  void MapJoints(const std::string& parentBody, rigidBodyPtr rigidBodyHandler);
  // void MapAMBFJointsToParent();
  void HelloThread();
  void RegisterRigidBodysPose();

  void SetBodyParams();
  // void CreateRBDLJoint(RBDLModelPtr rbdlModelPtr_, Vector3d& PA, Vector3d& CA, Vector3d& PP, Vector3d& CP, 
  // const double offsetQ, Vector3d axis, unsigned int parentId, Vector3d& p_world_startingbody, Joint joint, 
  // SpatialTransform	world_parentST, Body &body, std::string bodyName, Vector3d& p_world_endingbody, 
  // unsigned int& newBodyId, SpatialTransform&	world_bodyST);
  
  void CreateRBDLModel();
  int QIndexFromJointName(const std::string jointName);
  bool ExecutePoseInAMBF();
private:
  AMBFClientPtr ambfClientPtr_{nullptr};
  std::string baselinkName_{"baselink"};
  rigidBodyPtr baselinkHandler_{nullptr};
  std::vector<std::string> controlableJoints_;
  std::vector<std::string> baselinkChildren_;
  MatrixNd t_w_0_ = MatrixNd::Identity(4, 4);
  MatrixNd t_0_w_ = MatrixNd::Identity(4, 4);

  // Format of the map yet to be decided
  AMBFParamMap ambfParamMap_;
  AMBFParamMap::iterator ambfParamMapItr_;
  const double world_base_roll_{0.0};  // X
  const double world_base_pitch_{0.0}; // Y
  const double world_base_yaw_{0.0};   // Z
  RBDLModelPtr rbdlModelPtr_{nullptr};
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

  SpatialTransform world_baselinkST, world_yawlinkST, world_pitchfrontlinkST, world_pitchbacklinkST, 
	world_pitchbottomlinkST, world_pitchendlinkST, world_maininsertionlinkST, world_pitchtoplinkST,
	world_toollinkST, world_eeST;
};