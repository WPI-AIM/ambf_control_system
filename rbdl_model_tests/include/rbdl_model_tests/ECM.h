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

  unsigned int baselink_yawlinkId_, pitchendlinkId_, maininsertionlinkId_, toollinkId_, 
    yawlinkId_, yawlink_pitchbacklinkId_, pitchbacklink_pitchbottomlinkId_, pitchfrontlinkId_, pitchtoplinkId_;

  Body baselinkBody_, pitchendlinkBody_, maininsertionlinkBody_, toollinkBody_, yawlinkBody_, 
    pitchbacklinkBody_, pitchbottomlinkBody_, pitchfrontlinkBody_, pitchtoplinkBody_;

  double baselinkMScale_, pitchendlinkMScale_, maininsertionlinkMScale_, toollinkMScale_, 
    yawlinkMScale_, pitchbacklinkMScale_, pitchbottomlinkMScale_, pitchfrontlinkMScale_, 
    pitchtoplinkMScale_ = 1.0;

  // Joint baselink_pitchendlinkJoint_, pitchendlink_maininsertionlinkJoint_, 
  //   maininsertionlink_toollinkJoint_, baselink_yawlinkJoint_, yawlink_pitchbacklinkJoint_,
  //   pitchbacklink_pitchbottomlinkJoint_, pitchbottomlink_pitchendlinkJoint_,
  //   yawlink_pitchfrontlinkJoint_, pitchfrontlink_pitchbottomlinkJoint_, 
  //   pitchfrontlink_pitchtoplinkJoint_, pitchtoplink_pitchendlinkJoint_;

  // SpatialTransform pitchendlink_maininsertionlinkST_, maininsertionlink_toollinkST_, 
  //   baselink_yawlinkST_, yawlink_pitchbacklinkST_, pitchbacklink_pitchbottomlinkST_, 
  //   pitchbottomlink_pitchendlinkST_, yawlink_pitchfrontlinkST_, pitchfrontlink_pitchbottomlinkST_, 
  //   pitchfrontlink_pitchtoplinkST_, pitchyoplink_pitchendlinkST_;

  SpatialTransform world_baselinkST_, world_yawlinkST_, world_pitchbacklinkST_, world_pitchbottomlinkST_;


  const double ROOT_baselinkOffsetQ_                  = 0.0;
  const double baselink_yawlinkOffsetQ_               = -3.1414;
  const double yawlink_pitchbacklinkOffsetQ_          = 3.1416;
  const double pitchbacklink_pitchbottomlinkOffsetQ_  = 0.0;
  const double baselink_pitchendlinkOffsetQ_          = 1.56304;
  const double pitchendlink_maininsertionlinkOffsetQ_ = -1.5708;
  const double maininsertionlink_toollinkOffsetQ_     = -1.5708;
  const double pitchbottomlink_pitchendlinkOffsetQ_   = 0.0;
  const double yawlink_pitchfrontlinkOffsetQ_         = 3.1416;
  const double pitchfrontlink_pitchbottomlinkOffsetQ_ = 0.0;
  const double pitchfrontlink_pitchtoplinkOffsetQ_    = 0.0;
  const double pitchtoplink_pitchendlinkOffsetQ_      = 0.0;
};