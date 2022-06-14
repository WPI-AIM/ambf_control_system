#include<iostream>
#include<unordered_map>
#include "application/AMBFParams.h"
#include "application/Utilities.h"
#include "ambf_client/ambf_client.h"

//------------------------------------------------------------------------------
typedef Client* AMBFClientPtr;
typedef AMBFParams* AMBFParamsPtr;
typedef std::unordered_map<std::string, AMBFParamsPtr> AMBFParamMap;
typedef std::pair<std::string, AMBFParamsPtr> AMBFParamPair;
//------------------------------------------------------------------------------
const useconds_t sleepTime {250000};

class AMBFWrapper
{
public:
  AMBFWrapper();
  ~AMBFWrapper();
  void ConnectToAMBF();

  void ActivateAMBFHandlers(const std::string modelName, std::string baseRigidBodyName);
  void RegisterBodyToWorldTransformation(const std::string parentBodyName);
  rigidBodyPtr RigidBodyHandler(const std::string parentBodyName);
  void RegisterRigidBodysPose();

  void RegisterHomePoseTransformation();
  // World to body Transformation
  SpatialTransform T_W_N(const std::string rigidBodyName);
  void PrintAMBFfParamMap();
private:
  void SetAMBFParams(const std::string modelName);
  void ActivateHandlers();
  AMBFParamsPtr FetchFromAMBFParamMap(std::string parentBodyName);
private:
  AMBFClientPtr ambfClientPtr_;
  std::string baseRigidBodyName_{""};
  rigidBodyPtr baselinkHandler_{nullptr};
  std::vector<std::string> controlableJoints_;


  AMBFParamMap ambfParamMap_;
  AMBFParamMap::iterator ambfParamMapItr_;
};