#include "rbdl_model/BuildRBDLModel.h"
#include "rbdl_model/GraphEdge.h"

BuildRBDLModel::BuildRBDLModel(std::string actuator_config_file) 
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

  this->getNamespace();
  if(!this->getBodies()) return;
  if(!this->getJoints()) return;
  if(!this->buildModelSequence()) return;
  // if(!this->findBaseNode()) return;
  // this->addDummyBaseJoint();
  // this->buildModel();
}

void BuildRBDLModel::getNamespace() {
    Utilities utilities;

    YAML::Node blender_namespace = baseNode_["namespace"];
    if(blender_namespace.IsDefined())  blender_namespace_ = utilities.trimTrailingSpaces(blender_namespace);
}


template< class MapType >
void BuildRBDLModel::PrintMap(const MapType & map, const std::string & separator, std::ostream & os )
{
  typedef typename MapType::const_iterator const_iterator;

  for( const_iterator i = map.begin(), iend = map.end(); i != iend; ++i )
  {
    os << i->first << separator << i->second << std::endl;
  }
}

/*
 * Parse YAML file and create instances of Body class for each body of the model
 * @return true after successful parsing
 */
bool BuildRBDLModel::getBodies()
{
  Utilities utilities;

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
      bodyName = utilities.trimTrailingSpaces(body_yaml["name"]);
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

  PrintMap( bodyNameHash_.right, ") ", std::cout );
  // PrintMap( bodyNameHash_.left, " ends in position ", std::cout );
  return true;
}

/*
 * Parse YAML file and create instances of Joint class for each Joint of the model
 * @return true after successful parsing
 */
bool BuildRBDLModel::getJoints()
{
  YAML::Node joints = baseNode_["joints"];
  if(!joints.IsDefined()) return false;

  Utilities utilities;
  size_t totalJoints = joints.size();
  for (size_t index = 0; index < totalJoints; ++index) {    
    std::string joint_name_expanded = joints[index].as<std::string>();
    YAML::Node joint_yaml = baseNode_[joint_name_expanded];
    

    std::string joint_name;
    if(joint_yaml.IsDefined()) joint_name = utilities.trimTrailingSpaces(joint_yaml["name"]);
    std::string parent_name;

    YAML::Node name = baseNode_[joint_name_expanded]["name"];
    if(!name.IsDefined()) utilities.throwMissingFieldException("joint name: " + joint_name_expanded + ", name in Joint Params");

    YAML::Node parent = baseNode_[joint_name_expanded]["parent"];
    if(!parent.IsDefined()) utilities.throwMissingFieldException("parent name of joint: " + joint_name_expanded + ", in Joint Params");
    parent_name = utilities.trimTrailingSpaces(parent);
    utilities.eraseSubStr(parent_name, "BODY");

    // Ignore p2p joints
    YAML::Node type = baseNode_[joint_name_expanded]["type"];
    if(!type.IsDefined()) utilities.throwMissingFieldException("joint name: " + joint_name_expanded + ", type in Joint Params");
    std::string joint_type = utilities.trimTrailingSpaces(type);
    if(joint_type.compare("p2p") == 0) continue;

    // Children for earch body sorted in a map
    // jointParamObjectMap_.insert(std::make_pair(parent_name, std::unordered_map<std::string, jointParamPtr>()));
    // jointParamObjectMap_[parent_name].insert(std::make_pair(joint_name, new JointParam(baseNode_[joint_name_expanded])));

    jointParamObjectMap_.insert(std::make_pair(joint_name, new JointParam(baseNode_[joint_name_expanded])));
  }

  return true;
}

bool BuildRBDLModel::buildModelSequence()
{
  int V = bodyParamObjectMap_.size();  // Number of vertices in the graph
  int n = jointParamObjectMap_.size(); // Number of edges in the graph
  GraphEdge edges[n];
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
    bmLeftItr = bodyNameHash_.left.find(parentName);
    assert(parentName == bmLeftItr->first);
    int parentId = bmLeftItr->second;

    std::string childName = ptr->Child().c_str();
    bmLeftItr = bodyNameHash_.left.find(childName);
    assert(childName == bmLeftItr->first);
    int childId = bmLeftItr->second;

    int weight = ptr->Weight();
    // printf("%s,\t\t %s, %d,\t %s, %d,\t %d\n", 
    //   jointName.c_str(), parentName.c_str(), parentId, childName.c_str(), childId, weight);

    edges[edgeIndex] = { parentId, childId, weight };
    edgeIndex++;
  }

  // printf("V: %d, n: %d\n", V, n);
  // assert(sizeof(edges)/sizeof(edges[0]) == n);
  // printf("buildModelSequence() - edges\n");
  // for(int idx = 0; idx < n; idx++)
  // {
  //   GraphEdge graphEdge = edges[idx];
  //   printf("{ %d, %d, %d }\n", graphEdge.src, graphEdge.dest, graphEdge.weight);
  // }
  // graph edges array.
  // GraphEdge edges[] = {
  //   {0, 1, 4}, {0, 7, 8}, {1, 2, 8}, {1, 7, 11}, {2, 3, 7}, {2, 8, 2}, {2, 5, 4}, 
  //   {3, 4, 9}, {3, 5, 14},{4, 5, 10}, {5, 6, 2}, {6, 7, 1}, {6, 8, 6}, {7, 8, 7},
  // };
  // calculate number of edges
  // int n = sizeof(edges)/sizeof(edges[0]);
  // construct graph
  Utilities utilities;
  ModelGraph modelGraph(edges, V, n);
  // modelGraph.PrintAdjacencyMatrix();
  int baseNodeId = modelGraph.BaseNode();
  if(baseNodeId == -1) utilities.throwBaseNotFoundException();

  bmRightItr = bodyNameHash_.right.find(baseNodeId);
  // assert(modelGraph.BaseNode() == bmRightItr->first);
  std::string baseName = bmRightItr->second;
  printf("baseName: %s\n", baseName.c_str());
  
  bmLeftItr = bodyNameHash_.left.find(baseName);
  // assert(baseName == bmLeftItr->first);
  int baseId = bmLeftItr->second;

  std::vector<int> endEffectorNodesId = modelGraph.EndEffectorNodes();
  std::vector<std::string> endEffectorNodesName;

  for(int endEffectorNodeId : endEffectorNodesId)
  {
    bmRightItr = bodyNameHash_.right.find(endEffectorNodeId);
    // assert(modelGraph.BaseNode() == bmRightItr->first);
    endEffectorNodesName.emplace_back(bmRightItr->second);
  }

  for(std::string endEffectorNodeName: endEffectorNodesName)
  {
    printf("endEffectorNodeName: %s\n", endEffectorNodeName.c_str());
  }

  int dist[V];
  modelGraph.Dijkstra(baseId, dist);

  for(int v = 0; v < V; v++)
  {
    bmRightItr = bodyNameHash_.right.find(v);
    assert(v == bmRightItr->first);
    std::string bodyName = bmRightItr->second;

    printf("%d, %s, \t%d\n", v, bodyName.c_str(), dist[v]);
  }

  return true;
}
/*
 * Find the base of the model
 * @return true after successfully finding the base
 */
// bool BuildRBDLModel::findBaseNode() 
// {
//   // Holds parent of all the joints.
//   std::unordered_set<std::string> parentNodeSet;
//   // Holds children of all the joints
//   std::unordered_set<std::string> childNodeSet;

//   std::unordered_map<std::string, std::unordered_map<std::string, jointParamPtr> >::iterator itr;
//   std::unordered_map<std::string, jointParamPtr>::iterator ptr;

//   for (itr = jointParamObjectMap_.begin(); itr != jointParamObjectMap_.end(); itr++) {
//     for (ptr = itr->second.begin(); ptr != itr->second.end(); ptr++) {
//       jointParamPtr jointParamptr = ptr->second;

//       // Remove the body names from the parent set which are in child set
//       parentNodeSet.emplace(jointParamptr->Parent());
//       childNodeSet.emplace(jointParamptr->Child());
//     }
//   }

//   for (std::unordered_set<std::string>::iterator it=childNodeSet.begin(); it!=childNodeSet.end(); ++it) {
//     if(parentNodeSet.find(*it) != parentNodeSet.end()) parentNodeSet.erase(*it);
//   }

//   // In case multiple body names or body in remaining in the parent set, the root is undetermined.
//   if(parentNodeSet.size() < 1) {
//     std::cout << "No Base node found, invalid model." << std::endl;
//     return false;
//   }
//   if(parentNodeSet.size() > 1) {
//     std::cout << "Found more than one Base, make sure that model in YAML file has just one Base." << std::endl;
//     return false;
//   }

//   // If only one body name remains in the parent set, that body is set as the root.
//   std::unordered_set<std::string>::iterator it = parentNodeSet.begin();
//   baseRigidBody_ = *it;
//   return true;
// }

/*
 * Add a dummy parent to Base of the model just to handle RBDL model creation
 */
// void BuildRBDLModel::addDummyBaseJoint() {
//   // Create RBDL Joint for Base
//   std::string joint_type = "fixed";

//   // Create RBDL ID for Base
//   Vector3d parent_axis(0.0, 0.0, 0.0);
//   Vector3d parent_pivot(0.0, 0.0, 0.0);

//   Vector3d child_axis(0.0, 0.0, 0.0);
//   Vector3d child_pivot(0.0, 0.0, 0.0);

//   base_joint_name_ = base_parent_name_ + "-" + baseRigidBody_;
//   jointParamObjectMap_[base_parent_name_].insert(std::make_pair(base_joint_name_,
//     new JointParam(base_joint_name_, base_parent_name_, baseRigidBody_,
//                   parent_axis, parent_pivot, child_axis, child_pivot, joint_type)));
// }

/*
 * Build RBDL Model
 * @return true after successful RBDL model creation
 */
// bool BuildRBDLModel::buildModel() {
//   rbdl_check_api_version(RBDL_API_VERSION);

//   RBDLmodel_ = new Model();
//   RBDLmodel_->gravity = Vector3d(0., 0., -9.81); // in my case should set in the Z direction

//   std::unordered_map<std::string, std::unordered_map<std::string, jointParamPtr>>::iterator outter_map_itr;
//   std::unordered_map<std::string, jointParamPtr>::iterator inner_map_itr;

//   std::unordered_set<std::string> ancestry_set;
//   std::unordered_set<std::string>::iterator ancestry_set_itr;

//   // Add Base body to RBDL model
//   unsigned int base_id = addBodyToRBDL(base_parent_name_, 0, base_joint_name_, baseRigidBody_);
//   rbdlObjectMap_.insert(std::make_pair(baseRigidBody_, base_id));

//   // Add base body to ancestry_set
//   ancestry_set.emplace(baseRigidBody_);

//   // Iterate until ancestry_set is empty
//   while(!ancestry_set.empty()) {
//     //Get and remove a body from ancestry_set
//     ancestry_set_itr = ancestry_set.begin();
//     std::string parent_name = *ancestry_set_itr;
//     ancestry_set.erase(*ancestry_set_itr);

//     outter_map_itr = jointParamObjectMap_.find(parent_name);
//     if(outter_map_itr != jointParamObjectMap_.end()) {
//       // Get RBDL body_id of the Parent body
//       unsigned int parent_id = rbdlObjectMap_[parent_name];

//       // For every child of Parent body, estabilish parent-child relationship in RBDL Model
//       for (inner_map_itr = outter_map_itr->second.begin(); inner_map_itr != outter_map_itr->second.end(); inner_map_itr++) {
//         std::string child_name = inner_map_itr->second->Child();
//         std::string joint_name = inner_map_itr->first;

//         // Estabilish parent-child realationship in RBDL model and get RBDL body id of the child
//         unsigned int child_id = addBodyToRBDL(parent_name, parent_id, joint_name, child_name);

//         // Maintain the joint name and child id map. If the child_id doest already exist,
//         // add the child name to ancestry_set
//         joint_map.insert(std::make_pair(joint_name, child_id));
//         rbdl_object_map_itr_ = rbdlObjectMap_.find((child_name));
//         if(rbdl_object_map_itr_ == rbdlObjectMap_.end()) {
//           ancestry_set.emplace(child_name);
//           rbdlObjectMap_.insert(std::make_pair(child_name, child_id));
//         }
//       }
//     }
//   }
//   return true;
// }

/*
 * Different joint types suppored by RBDL
 * @param RBDL Joint type
 * @return String Joint type
 */
// const rbdlJointType BuildRBDLModel::getRBDLJointType(std::string joint_type) {
//     if (joint_type == "undefined") return JointTypeUndefined;
//     else if (joint_type == "revolute") return JointTypeRevolute;
//     else if (joint_type == "prismatic") return JointTypePrismatic;
//     else if (joint_type == "revoluteX") return JointTypeRevoluteX;
//     else if (joint_type == "revolutey") return JointTypeRevoluteY;
//     else if (joint_type == "revoluteZ") return JointTypeRevoluteZ;
//     else if (joint_type == "spherical") return JointTypeSpherical;
//     else if (joint_type == "eulerzyx") return JointTypeEulerZYX;
//     else if (joint_type == "eulerxyz") return JointTypeEulerXYZ;
//     else if (joint_type == "euleryxz") return JointTypeEulerYXZ;
//     else if (joint_type == "translationxyz") return JointTypeTranslationXYZ;
//     else if (joint_type == "base") return JointTypeFloatingBase;
//     else if (joint_type == "fixed") return JointTypeFixed;
//     else if (joint_type == "helical") return JointTypeHelical;
//     else if (joint_type == "1dof") return JointType1DoF;
//     else if (joint_type == "2dof") return JointType2DoF;
//     else if (joint_type == "p2p") return JointType3DoF;
//     else if (joint_type == "4dof") return JointType4DoF;
//     else if (joint_type == "5dof") return JointType5DoF;
//     else if (joint_type == "6dof") return JointType6DoF;
//     else if (joint_type == "custom") return JointTypeCustom;
//     return JointTypeUndefined;
// }


/*
 * Estabilish RBDL Joints between parent and child body
 * @param parent_name Parent name
 * @param parent_id   RBDL Parent ID
 * @param joint_name  Joint name
 * @param child_name  Child name
 * @return RBDL Child body id
 */
// unsigned int  BuildRBDLModel::addBodyToRBDL(std::string parent_name, unsigned int parent_id, std::string joint_name, std::string child_name) {
//   Utilities utilities;

//   // Get child body parameters
//   double mass = (bodyParamObjectMap_[child_name])->Mass();
//   Vector3d com = (bodyParamObjectMap_[child_name])->InertialOffsetPosition();
//   Math::Matrix3d inertia = (bodyParamObjectMap_[child_name])->Inertia();

//   // Create RBDL child instance
//   boost::optional<rbdlBody> child_body = Body(mass, com, inertia);

//   // Get child and parent positons for calculating Transformation between them
//   Vector3d parent_pivot = (jointParamObjectMap_[parent_name][joint_name])->ParentPivot();
//   Vector3d parent_axis = (jointParamObjectMap_[parent_name][joint_name])->ParentAxis();
//   Vector3d child_axis = (jointParamObjectMap_[parent_name][joint_name])->ChildAxis();

//   // Calculate Rotation between Parent and child
//   Math::Matrix3d rotation_matrix = utilities.rotationMatrixFromVectors(parent_axis, child_axis);

//   unsigned int child_id;
//   //Joint joint;
//   RigidBodyDynamics::JointType jointType;
//   // If base joint, make it be fixed with respect to World

//   if(parent_name.compare(base_parent_name_) == 0) {      
//     //joint = Joint (JointTypeFixed);
//     jointType = JointTypeFixed;
//   // Handle multiple Parents for a body. Just append ~ to body name.
//   // Otherwise RBDL would just throw duplicate body name exception
//   } else {
//     while(RBDLmodel_->GetBodyId(child_name.c_str()) != std::numeric_limits<unsigned int>::max()) {
//         child_name = child_name + "~";
//     }
//     //joint is not used. can it be removed?
//     //joint = Joint(SpatialVector (child_axis[0], child_axis[1], child_axis[2], 0.0, 0.0, 0.0));
//     jointType = JointTypeRevoluteZ;
//   }

//   // This map is just for tracking purpose and not used for RBDL model creation
//   rbdlBodyMap_.insert(std::make_pair(child_name, child_body));

//   // Crate RBDL Joint between parent and Child with parent_id, Transformation matrix, joint type, rbdl child body, child name
//   // SpatialTransform jointST;
//   // jointST.E = rotation_matrix;
//   // jointST.r = parent_pivot;

//   //child_id = RBDLmodel_->AddBody(parent_id, SpatialTransform (rotation_matrix, parent_pivot), joint, child_body.get(), child_name.c_str());
//   child_id = RBDLmodel_->AddBody(parent_id, SpatialTransform (rotation_matrix, parent_pivot), 
//                               jointType, child_body.get(), child_name.c_str());

//   return child_id;
// }

// Model* BuildRBDLModel::getRBDLModel()
// {
//   return RBDLmodel_;
// }

/*
 * Get all body Names in RBDL model
 * @return vector of body names
 */
std::vector<std::string> BuildRBDLModel::getAllBodyNames() {
  std::vector<std::string> bodyNames;
  std::transform (bodyParamObjectMap_.begin(), bodyParamObjectMap_.end(), back_inserter(bodyNames), [] (std::pair<std::string, bodyParamPtr> const & pair)
  {
  return pair.first;

  });

  return bodyNames;
}

/*
 * Get RBDL body id with body name
 * @param body name
 * @return RBDL body id
 */
// unsigned int BuildRBDLModel::getBodyId(const std::string bodyName) {
//   if (rbdlObjectMap_.find(bodyName) != rbdlObjectMap_.end()){
//       return rbdlObjectMap_[bodyName];
//   }
//   std::cout << "Body: " << bodyName << " not found in the model" << std::endl;

//   return -1;
// }

// /*
//  * Get RBDL body with body name
//  * @param body name
//  * @return RBDL body
//  */
// boost::optional<rbdlBody> BuildRBDLModel::getRBDLBody(const std::string bodyName) {
//   if(rbdlBodyMap_.find(bodyName) != rbdlBodyMap_.end()) {
//     return rbdlBodyMap_[bodyName];
//   }
//   std::cout << "Body: " << bodyName << " not found in the model" << std::endl;
//   return boost::none;
// }

// /*
//  * Get all children of a joint
//  * @param parent of the joint
//  * @return Joint name and children of the parent
//  */
// std::unordered_map<std::string, jointParamPtr> BuildRBDLModel::getJointChildren(std::string parent) {
//   if(jointParamObjectMap_.find(parent) != jointParamObjectMap_.end()) {
//     return jointParamObjectMap_[parent];
//   }

//   std::cout << "Parent: " << parent << " does not have any children in the model" << std::endl;
//   return {};
// }

// /*
//  * Print all Bodies of the Model
//  */
// void BuildRBDLModel::printBody() {
//   std::unordered_map<std::string, bodyParamPtr>::iterator body_map_itr;
//   for (body_map_itr = bodyParamObjectMap_.begin(); body_map_itr != bodyParamObjectMap_.end(); body_map_itr++) {
//     std::cout << body_map_itr->first << std::endl;
//   }
//   std::cout << std::endl;
// }

// /*
//  * Print all Joints of the Model
//  */
// void BuildRBDLModel::printJoint() 
// {
//   std::unordered_map<std::string, std::unordered_map<std::string, jointParamPtr>>::iterator outter_map_itr;
//   std::unordered_map<std::string, jointParamPtr>::iterator inner_map_itr;

//   for (outter_map_itr = jointParamObjectMap_.begin();
//    outter_map_itr != jointParamObjectMap_.end(); outter_map_itr++)
//   {
//     std::string parent_node_name = outter_map_itr->first;
//     std::cout << ", parent_node_name: " << parent_node_name << std::endl << "child_node_name: " ;

//     for (inner_map_itr = outter_map_itr->second.begin(); 
//       inner_map_itr != outter_map_itr->second.end(); inner_map_itr++)
//     {
//       std::string joint_name = inner_map_itr->first;
//       std::cout << "joint_name: " << joint_name;
//       jointParamPtr jointparamPtr = inner_map_itr->second;
//       std::cout << jointparamPtr->Child() << ", ";
//     }
//     std::cout << std::endl;
//   }
// }

/*
 * Get all Joint names of the model
 * @return Joint name as vector
 */

std::vector<std::string> BuildRBDLModel::getAllJointNames()
{
  // std::unordered_map<std::string, std::unordered_map<std::string, jointParamPtr>>::iterator outter_map_itr;
  // std::unordered_map<std::string, jointParamPtr>::iterator inner_map_itr;
  std::vector<std::string> names;

  // for (outter_map_itr = jointParamObjectMap_.begin(); outter_map_itr != jointParamObjectMap_.end(); outter_map_itr++) {
  //   for (inner_map_itr = outter_map_itr->second.begin(); inner_map_itr != outter_map_itr->second.end(); inner_map_itr++) {
  //     std::string joint_name = inner_map_itr->first;
  //     if (joint_name.find("world") == std::string::npos) 
  //     {
  //       names.push_back(joint_name);
  //     }
  //   }
  // }
  return names;
}


/*
 * Clean up memory
 */
void BuildRBDLModel::cleanUp() {
  // std::unordered_map<std::string, bodyParamPtr>::iterator bodyParamMapIt;
  // for ( bodyParamMapIt = bodyParamObjectMap_.begin(); bodyParamMapIt != bodyParamObjectMap_.end(); ++bodyParamMapIt ) {
  //   bodyParamMapIt->second->~BodyParam();
  // }

  // std::unordered_map<std::string, std::unordered_map<std::string, jointParamPtr> >::iterator itr;
  // std::unordered_map<std::string, jointParamPtr>::iterator ptr;

  // for (itr = jointParamObjectMap_.begin(); itr != jointParamObjectMap_.end(); itr++) {
  //   for (ptr = itr->second.begin(); ptr != itr->second.end(); ptr++) {
  //     ptr->second->~JointParam();
  //   }
  // }

  delete RBDLmodel_;
  std::cout << "RBDL Model deleted" << std::endl;
}

BuildRBDLModel::~BuildRBDLModel(void){
  delete RBDLmodel_;
}
