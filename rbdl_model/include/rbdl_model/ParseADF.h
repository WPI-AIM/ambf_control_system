#ifndef PARSEADF_H
#define PARSEADF_H

#include <boost/optional.hpp>
#include <boost/bimap.hpp>
#include "rbdl_model/BodyParam.h"
#include "rbdl_model/JointParam.h"
#include "rbdl_model/ModelGraph.h"
#include "application/Prep.h"
#include "application/Utilities.h"

//------------------------------------------------------------------------------
typedef BodyParam* bodyParamPtr;
typedef JointParam* jointParamPtr;
typedef boost::bimap<std::string, int> bmStrInt;
typedef bmStrInt::left_map::const_iterator bmLeftConstItr;
typedef bmStrInt::right_map::const_iterator bmRightConstItr;
//------------------------------------------------------------------------------

class ParseADF
{
public:
  ParseADF(const std::string actuator_config_file);

  void PrintBody();
  void PrintJoint();
  void CleanUp();


  std::string inline BaseName() const { return baseName_; }
  std::vector<std::string> inline EndEffectorsName() const { return endEffectorsName_; }
  std::vector<std::vector<std::string>> Paths() const { return paths_; }
  bodyParamPtr BodyParams(const std::string bodyName);
  jointParamPtr JointParams(const std::string jointName);
  ~ParseADF(void);

private:
  template< class MapType >
  void PrintMap(const MapType & map, const std::string & separator, std::ostream & os);
  
  template< class MapType, typename T, typename R>
  R GetValueFromMap(const MapType & map, T t, R r);

  bool Namespace();
  bool Bodies();
  bool Joints();
  bool BuildModelSequence();
private:
  std::string blender_namespace_;
  YAML::Node baseNode_;
  std::string actuator_config_file_;
  std::string baseName_{""};

  std::vector<std::string> endEffectorsName_;
  std::unordered_map<std::string, bodyParamPtr> bodyParamObjectMap_;
  std::unordered_map<std::string, bodyParamPtr>::iterator bodyParamObjectMapItr_;

  std::unordered_map<std::string, jointParamPtr> jointParamObjectMap_;
  std::unordered_map<std::string, jointParamPtr>::iterator jointParamObjectMapItr_;

  bmStrInt bodyNameHash_;
  std::vector<std::vector<std::string>> paths_;
};

#endif