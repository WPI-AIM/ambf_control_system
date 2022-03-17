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

  unsigned int baseLink_yawLinkID_, pitchEndLinkId_, mainInsertionLinkId_, toolLinkId_, 
    yawLinkId_, pitchBackLinkId_, pitchBottomLinkId_, pitchFrontLinkId_, pitchTopLinkId_;

  Body baseLinkBody_, pitchEndLinkBody_, mainInsertionLinkBody_, toolLinkBody_, yawLinkBody_, 
    pitchBackLinkBody_, pitchBottomLinkBody_, pitchFrontLinkBody_, pitchTopLinkBody_;

  double baseLinkMScale_, pitchEndLinkMScale_, mainInsertionLinkMScale_, toolLinkMScale_, 
    yawLinkMScale_, pitchBackLinkMScale_, pitchBottomLinkMScale_, pitchFrontLinkMScale_, 
    pitchTopLinkMScale_ = 1.0;

  Joint ROOT_baseLinkJoint_, baseLink_pitchEndLinkJoint_, pitchEndLink_mainInsertionLinkJoint_, 
    mainInsertionLink_toolLinkJoint_, baseLink_yawLinkJoint_, yawLink_pitchBackLinkJoint_,
    pitchBackLink_pitchBottomLinkJoint_, pitchBottomLink_pitchEndLinkJoint_,
    yawLink_pitchFrontLinkJoint_, pitchFrontLink_pitchBottomLinkJoint_, 
    pitchFrontLink_pitchTopLinkJoint_, pitchTopLink_pitchEndLinkJoint_;

  SpatialTransform ROOT_baseLinkST_, baseLink_pitchEndLinkST_, 
    pitchEndLink_mainInsertionLinkST_, mainInsertionLink_toolLinkST_, baseLink_yawLinkST_, 
    yawLink_pitchBackLinkST_, pitchBackLink_pitchBottomLinkST_, 
    pitchBottomLink_pitchEndLinkST_,yawLink_pitchFrontLinkST_, 
    pitchFrontLink_pitchBottomLinkST_, pitchFrontLink_pitchTopLinkST_, 
    pitchTopLink_pitchEndLinkST_;

  const double ROOT_baseLinkOffset_                  = 0.0;
  const double baseLink_pitchEndLinkOffset_          = 0.0;
  const double pitchEndLink_mainInsertionLinkOffset_ = 0.0;
  const double mainInsertionLink_toolLinkOffset_     = 0.0;
  const double baseLink_yawLinkOffset_               = 0.0;
  const double yawLink_pitchBackLinkOffset_          = 0.0;
  const double pitchBackLink_pitchBottomLinkOffset_  = 0.0;
  const double pitchBottomLink_pitchEndLinkOffset_   = 0.0;
  const double yawLink_pitchFrontLinkOffset_         = 0.0;
  const double pitchFrontLink_pitchBottomLinkOffset_ = 0.0;
  const double pitchFrontLink_pitchTopLinkOffset_    = 0.0;
  const double pitchTopLink_pitchEndLinkOffset_      = 0.0;
};