#include <unordered_map>
#include <thread>

#include "rbdl_model_tests/AMBFParams.h"
#include "ambf_client/ambf_client.h"
#include <rbdl/rbdl.h>
#include <rbdl/rbdl_math.h>

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

typedef Model* RBDLModelPtr;
typedef AMBFParams* AMBFParamsPtr;
typedef Client* AMBFClientPtr;
typedef std::unordered_map<std::string, AMBFParamsPtr> AMBFParamMap;
typedef std::pair<std::string, AMBFParamsPtr> AMBFParamPair;

//const double TEST_PREC = 1.0e-12;
const double TEST_LAX {1.0e-7};
const useconds_t sleepTime {250000};
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

struct T_W_N
{
  Matrix3d r_w_n_ambf;
  Vector3d p_w_n_ambf;

  Matrix3d r_w_n_rbdl;
  Vector3d p_w_n_rbdl;
};
typedef T_W_N* t_w_nPtr;

class KUKA 
{
public:
  KUKA();
  ~KUKA();
  const Matrix3d PrintAMBFTransformation();
  // const Vector3d TranslationVectorTF();

  inline Model* GetRBDLModel() { return rbdlModel_; }

  void ExecutePose(VectorNd Q);
  t_w_nPtr twnFromModels(std::string jointName);

  void CleanUp();
private:
  bool ConnectToAMBF();
  void RegisterBodyToWorldTransformation(const std::string parentBody);
  void SetAMBFParams();
  void MapJoints(const std::string& parentBody, rigidBodyPtr rigidBodyHandler);
  void MapAMBFJointsToParent();
  void HelloThread();
  void RegisterAllRigidBodyPose();

  void SetBodyParams();

  void CreateRBDLJoint(Vector3d& pa, Vector3d& ca, const Vector3d& pp, const Vector3d& cp, 
  const double offsetQ, 
	const Vector3d axis, const unsigned int parentId, const Joint joint, const SpatialTransform world_parentST, 
	const Body &body, const std::string bodyName, unsigned int& childId, SpatialTransform&	world_childST);


  void CreateRBDLModel();
  void PrintRBDLModel();
  void SetRBDLPose();
  void CheckRBDLModel();
  void ExecutePoseInAMBF();

private:
  AMBFClientPtr ambfClientPtr_{nullptr};
  std::string baselinkName_{"base"};
  MatrixNd t_w_0_ = MatrixNd::Identity(4, 4);
  MatrixNd t_0_w_ = MatrixNd::Identity(4, 4);
  Vector3d vector3d_zero_ = Vector3d::Zero();
  // Format of the map yet to be decided
  AMBFParamMap ambfParamMap_;
  AMBFParamMap::iterator ambfParamMapItr_;
  // const double world_base_roll_{-M_PI_2};  // X
  // const double world_base_pitch_{M_PI_2}; // Y
  // const double world_base_yaw_{M_PI};   // Z
  const double world_base_roll_{0.0};  // X
  const double world_base_pitch_{0.0}; // Y
  const double world_base_yaw_{0.0};   // Z
  // RBDL use joint names to define a body. This corresponds to rigidbody in AMBF.
  // This give a need to map the RBDL joint name with the corresponing AMBF handler that would provied
  // the corresponding AMBF joint.
  // std::unordered_map<std::string, std::string> ambfJointToParentMap_;
  // std::unordered_map<std::string, std::string>::iterator ambfJointToParentMapItr_;
  // JointValuesMap jointValuesMap_;
  // JointValuesMap::iterator jointValuesMapItr_;

  Model* rbdlModel_{nullptr};

  Body worldBody_, baseBody_, link1Body_, link2Body_, link3Body_, link4Body_, link5Body_, link6Body_, link7Body_;
  VectorNd Q_;
  VectorNd QDot_;
  VectorNd QDDot_;
  VectorNd Tau_;
  std::map< std::string, unsigned int > rbdlmBodyMap_;
  std::map<std::string, unsigned int>::iterator rbdlmBodyMapItr_;
};