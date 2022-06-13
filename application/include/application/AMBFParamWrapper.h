#include<iostream>
#include<unordered_map>
#include "application/AMBFParams.h"
#include "application/Utilities.h"

//------------------------------------------------------------------------------
typedef AMBFParams* AMBFParamsPtr;
typedef std::unordered_map<std::string, AMBFParamsPtr> AMBFParamMap;
typedef std::pair<std::string, AMBFParamsPtr> AMBFParamPair;
//------------------------------------------------------------------------------

class AMBFParamWrapper
{
public:
  AMBFParamWrapper();
  ~AMBFParamWrapper();
  void AMBFParameter(std::string rigidBodyName, AMBFParamsPtr ambfParamsPtr);
  void ActivateAMBFHandlers();
  void RegisterBodyToWorldTransformation(const std::string parentBodyName);
  rigidBodyPtr RigidBodyHandler(const std::string parentBodyName);
  void RegisterRigidBodysPose();

  // World to body Transformation
  SpatialTransform T_W_N(const std::string rigidBodyName);
private:
  AMBFParamsPtr FetchFromAMBFParamMap(std::string parentBodyName);
private:
  AMBFParamMap ambfParamMap_;
  AMBFParamMap::iterator ambfParamMapItr_;

};