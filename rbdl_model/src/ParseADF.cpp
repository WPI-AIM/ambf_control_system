#include "rbdl_model/ParseADF.h"
#include "rbdl_model/GraphEdge.h"

ParseADF::ParseADF(std::string actuator_config_file) 
{
  actuator_config_file_ = actuator_config_file;

  try 
  {
    baseNode_ = YAML::LoadFile(actuator_config_file_);
  } 
  catch (std::exception &e)
  {
    std::ostringstream errormsg;
    errormsg <<
      "Error: FAILED TO ACTUATOR CONFIG: " + actuator_config_file_ << std::endl;
    throw RBDLModel::ModelErrors::RBDLModelInvalidFilePathError(errormsg.str());
  }

  if (baseNode_.IsNull()) return;

  Namespace();
  if(!this->Bodies()) return;
  if(!this->Joints()) return;

  if(!this->BuildModelSequence()) return;
}

void ParseADF::Namespace() 
{

  YAML::Node blender_namespace = baseNode_["namespace"];
  if(!blender_namespace.IsDefined())
    RBDLUtilities::ThrowMissingFieldException("namespace");
    
  blender_namespace_ = 
    RBDLUtilities::TrimTrailingSpaces(blender_namespace);

  RBDLUtilities::EraseSubStr(blender_namespace_, "/ambf/env/");
  std::cout << "namespace: " << blender_namespace_ << std::endl;

  if(blender_namespace_.empty() || !RBDLUtilities::HasEnding(blender_namespace_, "/"))
    RBDLUtilities::ThrowInvalidNamespaceException();
}


template< class MapType >
void ParseADF::PrintMap(const MapType & map, const std::string & separator, std::ostream & os )
{
  typedef typename MapType::const_iterator const_iterator;

  for( const_iterator i = map.begin(), iend = map.end(); i != iend; ++i )
  {
    os << i->first << separator << i->second << std::endl;
  }
}

template< class MapType, typename T, typename R>
R ParseADF::GetValueFromMap(const MapType & map, T t, R r)
{
  if constexpr(std::is_same_v<T, std::string>)//char const*>)
  {
    bmLeftConstItr bmLeftItr = map.left.find(t);
    r = bmLeftItr->second;
    int id = r;
    assert(t == bmLeftItr->first);
    return id;
  }
  else if constexpr(std::is_same_v<T, int>)
  {
    bmRightConstItr bmRightItr = map.right.find(t);
    r = bmRightItr->second;
    std::string name = r;
    assert(t == bmRightItr->first);
    return name;
  }
  return r;
}


/*
 * Parse YAML file and create instances of Body class for each body of the model
 * @return true after successful parsing
 */
bool ParseADF::Bodies()
{

  YAML::Node rigidBodies = baseNode_["bodies"];
  if(!rigidBodies.IsDefined()) return false;

  int bodyIndex = 0;
  size_t totalRigidBodies = rigidBodies.size();
  for (size_t index = 0; index < totalRigidBodies; ++index) {
    std::string body_name_expanded = rigidBodies[index].as<std::string>();
    YAML::Node body_yaml = baseNode_[body_name_expanded];
    std::string bodyName;

    // Get the name of the body from YAML. Model cannot be processed if the body defined in the list
    // doenst have parameters defined in the YAML. TBD
    if(body_yaml.IsDefined())
      bodyName = RBDLUtilities::TrimTrailingSpaces(body_yaml["name"]);
    else
    {
      std::ostringstream errormsg;
      errormsg <<
        "Error: MISSING PARAMETER DEFINITION FOR " + body_name_expanded << std::endl;
      throw RBDLModel::ModelErrors::RBDLModelMissingParameterError(errormsg.str());
      return false;
    }

    if(bodyName.compare("target_ik") != 0 && bodyName.compare("target_fk") != 0)
    {
      bodyParamObjectMap_.insert(std::make_pair(bodyName, new BodyParam(baseNode_[body_name_expanded])));
      bodyNameHash_.insert(bmStrInt::value_type(bodyName, bodyIndex));
      bodyIndex++;
    }
  }
  // PrintMap(bodyNameHash_.right, ") ", std::cout );
  // PrintMap(bodyNameHash_.left, ") ", std::cout );
  return true;
}

/*
 * Parse YAML file and create instances of Joint class for each Joint of the model
 * @return true after successful parsing
 */
bool ParseADF::Joints()
{
  YAML::Node joints = baseNode_["joints"];
  if(!joints.IsDefined()) return false;

  size_t totalJoints = joints.size();
  for (size_t index = 0; index < totalJoints; ++index) {    
    std::string joint_name_expanded = joints[index].as<std::string>();
    YAML::Node joint_yaml = baseNode_[joint_name_expanded];
    

    std::string joint_name;
    if(joint_yaml.IsDefined()) joint_name = RBDLUtilities::TrimTrailingSpaces(joint_yaml["name"]);
    std::string parent_name;

    YAML::Node name = baseNode_[joint_name_expanded]["name"];
    if(!name.IsDefined()) RBDLUtilities::ThrowMissingFieldException("joint name: " + joint_name_expanded + ", name in Joint Params");

    YAML::Node parent = baseNode_[joint_name_expanded]["parent"];
    if(!parent.IsDefined()) RBDLUtilities::ThrowMissingFieldException("parent name of joint: " + joint_name_expanded + ", in Joint Params");
    parent_name = RBDLUtilities::TrimTrailingSpaces(parent);
    RBDLUtilities::EraseSubStr(parent_name, "BODY");

    // Ignore p2p joints
    YAML::Node type = baseNode_[joint_name_expanded]["type"];
    if(!type.IsDefined()) RBDLUtilities::ThrowMissingFieldException("joint name: " + joint_name_expanded + ", type in Joint Params");
    std::string joint_type = RBDLUtilities::TrimTrailingSpaces(type);
    if(joint_type.compare("p2p") == 0) continue;

    // Children for earch body sorted in a map
    // jointParamObjectMap_.insert(std::make_pair(parent_name, std::unordered_map<std::string, jointParamPtr>()));
    // jointParamObjectMap_[parent_name].insert(std::make_pair(joint_name, new JointParam(baseNode_[joint_name_expanded])));

    jointParamObjectMap_.insert(std::make_pair(joint_name, new JointParam(baseNode_[joint_name_expanded])));
  }

  return true;
}

bodyParamPtr ParseADF::BodyParams(const std::string bodyName)
{
  bodyParamObjectMapItr_ = bodyParamObjectMap_.find(bodyName);
  if(bodyParamObjectMapItr_ == bodyParamObjectMap_.end())
    RBDLUtilities::ThrowKeyNotFoundException("bodyParamObjectMapItr_", bodyName);

  return bodyParamObjectMap_[bodyName];
}

jointParamPtr ParseADF::JointParams(const std::string jointName)
{
  jointParamObjectMapItr_ = jointParamObjectMap_.find(jointName);
  if(jointParamObjectMapItr_ == jointParamObjectMap_.end())
    RBDLUtilities::ThrowKeyNotFoundException("jointParamObjectMapItr_", jointName);

  return jointParamObjectMap_[jointName];
}

bool ParseADF::BuildModelSequence()
{
  int V = bodyParamObjectMap_.size();  // Number of vertices in the graph
  int E = jointParamObjectMap_.size(); // Number of edges in the graph
  GraphEdge edges[E];
  bmLeftConstItr bmLeftItr;
  bmRightConstItr bmRightItr;

  int edgeIndex = 0;
  for(jointParamObjectMapItr_ = jointParamObjectMap_.begin(); 
    jointParamObjectMapItr_ != jointParamObjectMap_.end();
    jointParamObjectMapItr_++)
  {
    std::string jointName = jointParamObjectMapItr_->first;
    jointParamPtr ptr = jointParamObjectMapItr_->second;

    std::string parentName = ptr->Parent().c_str();
    int parentId = GetValueFromMap(bodyNameHash_, parentName, -1);

    std::string childName = ptr->Child().c_str();
    int childId = GetValueFromMap(bodyNameHash_, childName, -1);
    int weight = ptr->Weight();

    edges[edgeIndex] = { parentId, childId, weight };
    edgeIndex++;
  }

  assert(sizeof(edges)/sizeof(edges[0]) == E);

  ModelGraph modelGraph(edges, V, E);
 
  int baseNodeId = modelGraph.BaseNode();
  if(baseNodeId == -1) RBDLUtilities::ThrowBaseNotFoundException();

  // std::string baseNodeName;
  baseName_ = GetValueFromMap(bodyNameHash_, baseNodeId, baseName_);
  
  std::vector<int> endEffectorNodesId = modelGraph.EndEffectorNodes();
  for(int nodeId : endEffectorNodesId)
  {
    std::string rigidBodyName;
    rigidBodyName = GetValueFromMap(bodyNameHash_, nodeId, rigidBodyName);
    endEffectorsName_.emplace_back(rigidBodyName);
  }

  // Each end effector will have different path
  for(int eeId : endEffectorNodesId)
  {
    std::vector<int> pathById = modelGraph.ShortestPath(baseNodeId, eeId);
    std::vector<std::string> pathByName;
    // Add path by name to correspoinding Vector by Index
    for(int id : pathById)
    {
      std::string rigidBodyName;
      rigidBodyName = GetValueFromMap(bodyNameHash_, id, rigidBodyName);
      pathByName.emplace_back(rigidBodyName);
    }
    paths_.emplace_back(pathByName);
  }

  // Check that the rigid bodies is enabled for ROS output. 
  // Its need for RBDL Model creation.
  for(std::vector<std::string> path : paths_)
  {
    for(std::string bodyName : path)
    {
      bodyParamPtr bodyParam = BodyParams(bodyName);
      if(bodyParam->Passive())
        RBDLUtilities::ThrowDisabledForROS(bodyParam->Name().c_str());
    }
  }

  for(std::vector<std::string> path : paths_)
  {
    for(std::string name : path)
    {
      printf("%s->", name.c_str());
    }
    printf("\n");
  }

  return true;
}



ParseADF::~ParseADF(){}