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

  bool SetJointAngleWithName(const std::string jointName, double qDesired);
  float GetJointAngleWithName(const std::string jointName);
  bool ExecutePose();
  std::vector<t_w_nPtr> HomePoseTransformation();
  t_w_nPtr twnFromModels(std::string jointName);
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
  int GetQIndexFromJointName(const std::string jointName);
  bool ExecutePoseInAMBF();
private:
  AMBFClientPtr ambfClientPtr_{nullptr};
  std::string baselinkName_{"baselink"};
  rigidBodyPtr baselinkHandler_{nullptr};
  std::vector<std::string> controlableJoints;
  std::vector<std::string> baselinkChildren;
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