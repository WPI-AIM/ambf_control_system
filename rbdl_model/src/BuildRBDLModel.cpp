#include "rbdl_model/BuildRBDLModel.h"



BuildRBDLModel::BuildRBDLModel(std::string actuator_config_file) {
    actuator_config_file_ = actuator_config_file;

    try {
        baseNode_ = YAML::LoadFile(actuator_config_file_);

    } catch (std::exception &e){
//        std::cerr << "[Exception]: " << e.what() << std::endl;
//        std::cerr << "ERROR! FAILED TO ACTUATOR CONFIG: " << actuator_config_file_ << std::endl;
//        return;

        std::ostringstream errormsg;
        errormsg <<
                 "Error: FAILED TO ACTUATOR CONFIG: " + actuator_config_file_
                 << std::endl;
        throw RBDLModel::ModelErrors::RBDLModelInvalidFilePathError(errormsg.str());
    }

    if (baseNode_.IsNull()) return;

    this->getNamespace();
    if(!this->getBodies()) return;
    if(!this->getJoints()) return;
    if(!this->findBaseNode()) return;
    this->addDummyBaseJoint();
    this->buildModel();
}

void BuildRBDLModel::getNamespace() {
    Utilities utilities;

    YAML::Node blender_namespace = baseNode_["namespace"];
    if(blender_namespace.IsDefined())  blender_namespace_ = utilities.trimTrailingSpaces(blender_namespace);
}


bool BuildRBDLModel::getBodies()
{
    Utilities utilities;

    YAML::Node rigidBodies = baseNode_["bodies"];
    if(!rigidBodies.IsDefined()) return false;
    size_t totalRigidBodies = rigidBodies.size();
    for (size_t i = 0; i < totalRigidBodies; ++i) {
        std::string body_name_expanded = rigidBodies[i].as<std::string>();
        YAML::Node body_yaml = baseNode_[body_name_expanded];
        std::string body_name;
        if(body_yaml.IsDefined()) body_name = utilities.trimTrailingSpaces(body_yaml["name"]);
        bodyParamObjectMap_.insert(std::make_pair(body_name, new BodyParam(baseNode_[body_name_expanded])));
    }
    return true;
}

bool BuildRBDLModel::getJoints()
{
    YAML::Node joints = baseNode_["joints"];
    if(!joints.IsDefined()) return false;

    Utilities utilities;
    size_t totalJoints = joints.size();
    for (size_t i = 0; i < totalJoints; ++i) {
        std::string joint_name_expanded = joints[i].as<std::string>();
        YAML::Node joint_yaml = baseNode_[joint_name_expanded];
        std::string joint_name;
        if(joint_yaml.IsDefined()) joint_name = utilities.trimTrailingSpaces(joint_yaml["name"]);
//        std::cout << "joint_name: " << joint_name << std::endl;
        std::string parent_name;

        YAML::Node name = baseNode_[joint_name_expanded]["name"];

//        std::cout << "joint_name_expanded: getJoints()" << joint_name_expanded << std::endl;
//        if(!name.IsDefined()) utilities.throwExceptionMessage("joint name: " + name_ + ", name in Joint Params");
//        name_ = utilities.trimTrailingSpaces(name);


//        if(name.IsDefined()) {
//            YAML::Node parent = baseNode_[joint_name_expanded]["parent"];
//            if(parent.IsDefined()) parent_name = utilities.trimTrailingSpaces(parent);
//            utilities.eraseSubStr(parent_name, "BODY");
//        }

        if(!name.IsDefined()) utilities.throwExceptionMessage("joint name: " + joint_name_expanded + ", name in Joint Params");
        YAML::Node parent = baseNode_[joint_name_expanded]["parent"];
        if(!parent.IsDefined()) utilities.throwExceptionMessage("parent name of joint: " + joint_name_expanded + ", in Joint Params");
        parent_name = utilities.trimTrailingSpaces(parent);
        utilities.eraseSubStr(parent_name, "BODY");

        jointParamObjectMap_.insert(std::make_pair(parent_name, std::unordered_map<std::string, jointParamPtr>()));
        jointParamObjectMap_[parent_name].insert(std::make_pair(joint_name, new JointParam(baseNode_[joint_name_expanded])));
    }
    return true;
}


bool BuildRBDLModel::findBaseNode() {
    std::unordered_set<std::string> parentNodeSet;
    std::unordered_set<std::string> childNodeSet;

    std::unordered_map<std::string, std::unordered_map<std::string, jointParamPtr> >::iterator itr;
    std::unordered_map<std::string, jointParamPtr>::iterator ptr;

    for (itr = jointParamObjectMap_.begin(); itr != jointParamObjectMap_.end(); itr++) {
        for (ptr = itr->second.begin(); ptr != itr->second.end(); ptr++) {
            jointParamPtr jointParamptr = ptr->second;
            parentNodeSet.emplace(jointParamptr->Parent());
            childNodeSet.emplace(jointParamptr->Child());
        }
    }


    for (std::unordered_set<std::string>::iterator it=childNodeSet.begin(); it!=childNodeSet.end(); ++it) {
        if(parentNodeSet.find(*it) != parentNodeSet.end()) parentNodeSet.erase(*it);
    }

    if(parentNodeSet.size() < 1) {
        std::cout << "No Base node found, invalid model." << std::endl;
        return false;
    }
    if(parentNodeSet.size() > 1) {
        std::cout << "Found more than one Base, make sure that model in YAML file has just one Base." << std::endl;
        return false;
    }

    std::unordered_set<std::string>::iterator it = parentNodeSet.begin();
    baseRigidBody_ = *it;

//    std::cout << "Base node: " << *it << std::endl;
    return true;
}

void BuildRBDLModel::addDummyBaseJoint() {
    // Create RBDL Joint for Base
    std::string joint_type = "fixed";

    // Create RBDL ID for Base
    Vector3d parent_axis(0.0, 0.0, 0.0);
    Vector3d parent_pivot(0.0, 0.0, 0.0);

    base_joint_name_ = base_parent_name_ + "-" + baseRigidBody_;
    jointParamObjectMap_[base_parent_name_].insert(std::make_pair(base_joint_name_,
                                                                  new JointParam(base_joint_name_, base_parent_name_, baseRigidBody_, parent_axis, parent_pivot, joint_type)));
}

bool BuildRBDLModel::buildModel() {
    rbdl_check_api_version(RBDL_API_VERSION);

    RBDLmodel_ = new Model();
    RBDLmodel_->gravity = Vector3d(0., 0., -9.81); // in my case should set in the Z direction

    std::unordered_map<std::string, std::unordered_map<std::string, jointParamPtr>>::iterator outter_map_itr;
    std::unordered_map<std::string, jointParamPtr>::iterator inner_map_itr;

    std::unordered_set<std::string> ancestry_set;
    std::unordered_set<std::string>::iterator ancestry_set_itr;

    unsigned int base_id = addBodyToRBDL(base_parent_name_, 0, base_joint_name_, baseRigidBody_);
    rbdlObjectMap_.insert(std::make_pair(baseRigidBody_, base_id));

    ancestry_set.emplace(baseRigidBody_);
    while(!ancestry_set.empty()) {
        ancestry_set_itr = ancestry_set.begin();
        std::string parent_name = *ancestry_set_itr;
        ancestry_set.erase(*ancestry_set_itr);

        outter_map_itr = jointParamObjectMap_.find(parent_name);
        if(outter_map_itr != jointParamObjectMap_.end()) {
            unsigned int parent_id = rbdlObjectMap_[parent_name];
//            std::cout << "parent_name: " << parent_name << ", parent_id: " << parent_id << ", its children: ";
            for (inner_map_itr = outter_map_itr->second.begin(); inner_map_itr != outter_map_itr->second.end(); inner_map_itr++) {

                std::string child_name = inner_map_itr->second->Child();
//                std::cout << child_name << ", ";

                // Create RBDL Joint between parent and child
                std::string joint_name = inner_map_itr->first;
                unsigned int child_id = addBodyToRBDL(parent_name, parent_id, joint_name, child_name);


                rbdl_object_map_itr_ = rbdlObjectMap_.find((child_name));
                if(rbdl_object_map_itr_ == rbdlObjectMap_.end()) {
                    ancestry_set.emplace(child_name);
                    rbdlObjectMap_.insert(std::make_pair(child_name, child_id));
                }
            }
//            std::cout << std::endl;
        }

    }

//    std::cout << "rbdlObjectMap: " << std::endl;
//    for(rbdl_object_map_itr_ = rbdlObjectMap_.begin(); rbdl_object_map_itr_ != rbdlObjectMap_.end(); rbdl_object_map_itr_++) {
//        std::cout << rbdl_object_map_itr_->first << ": " <<rbdl_object_map_itr_->second << std::endl;

//    }
    return true;
}

unsigned int  BuildRBDLModel::addBodyToRBDL(std::string parent_name, unsigned int parent_id, std::string joint_name, std::string child_name) {
    // Create RBDL body for child joint
    double mass = (bodyParamObjectMap_[child_name])->Mass();
    Vector3d com = (bodyParamObjectMap_[child_name])->InertialOffsetPosition();
    Math::Matrix3d inertia = (bodyParamObjectMap_[child_name])->Inertia();

//    std::cout << "parent_name: " << parent_name << ", joint_name: " << joint_name << ", child_name: " << child_name << std::endl;
//    std::cout << "mass: " << mass << std::endl;
//    std::cout << "com: " << com[0] << ", " << com[1] << ", " << com[2] << std::endl;
//    std::cout << "inertia: " << inertia(0, 0) << ", " << inertia(1 , 1) << ", " << inertia(2, 2) << std::endl;


    boost::optional<rbdlBody> child_body = Body(mass, com, inertia);

    Vector3d parent_pivot = (jointParamObjectMap_[parent_name][joint_name])->ParentPivot();
    Vector3d parent_axis = (jointParamObjectMap_[parent_name][joint_name])->ParentAxis();
//    std::cout << "parent_pivot: " << parent_pivot[0] << ", " << parent_pivot[1] << ", " << parent_pivot[2] << std::endl;

    rbdlBodyMap_.insert(std::make_pair(child_name, child_body));

    // Create RBDL Joint between parent and child
    std::string joint_type_str = (jointParamObjectMap_[parent_name][joint_name])->Type();
    rbdlJointType joint_type = getRBDLJointType(joint_type_str);
//    std::cout << "child: " << child_name << ", joint_type_str: " << joint_type_str << ", joint_type: " << joint_type << ", parent: " << parent_name << std::endl;

    rbdlJoint joint_rot_z;

    if(joint_type == JointTypeFixed) {
        joint_rot_z = SpatialVector (0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    } else if (joint_type == JointTypeRevolute || joint_type == JointType3DoF) {
        joint_rot_z = SpatialVector (parent_axis[0], parent_axis[1], parent_axis[2], 0.0, 0.0, 0.0);
    } else if (joint_type == JointTypePrismatic) {
        joint_rot_z = SpatialVector (0.0, 0.0, 0.0, parent_axis[0], parent_axis[1], parent_axis[2]);
    }

    unsigned int child_id;
    // check if the body is base

    if(RBDLmodel_->GetBodyId(child_name.c_str()) != UINT_MAX) {
        if(parent_id == 0) {
            child_id = RBDLmodel_->AddBody(parent_id, Xtrans(parent_pivot), joint_rot_z, child_body.get(), child_name.c_str());
        } else {
            child_id = RBDLmodel_->AppendBody(Xtrans(parent_pivot), joint_rot_z, child_body.get(), child_name.c_str());
        }
    }

    return child_id;
}

const rbdlJointType BuildRBDLModel::getRBDLJointType(std::string joint_type) {
    if (joint_type == "undefined") return JointTypeUndefined; //Not supported
    else if (joint_type == "revolute") return JointTypeRevolute; //Tested with YAML file and works
    else if (joint_type == "prismatic") return JointTypePrismatic; //Tested with YAML file and works
    else if (joint_type == "revoluteX") return JointTypeRevoluteX; //Tested with hard coded values
    else if (joint_type == "revolutey") return JointTypeRevoluteY; //Tested with hard coded values
    else if (joint_type == "revoluteZ") return JointTypeRevoluteZ; //Tested with hard coded values
    else if (joint_type == "spherical") return JointTypeSpherical; //Tested with hard coded values
    else if (joint_type == "eulerzyx") return JointTypeEulerZYX; //Tested with hard coded values
    else if (joint_type == "eulerxyz") return JointTypeEulerXYZ; //Tested with hard coded values
    else if (joint_type == "euleryxz") return JointTypeEulerYXZ; //Tested with hard coded values
    else if (joint_type == "translationxyz") return JointTypeTranslationXYZ; //Tested with hard coded values
    else if (joint_type == "base") return JointTypeFloatingBase; //Tested with hard coded values
    else if (joint_type == "fixed") return JointTypeFixed; //Tested with hard coded values
    else if (joint_type == "helical") return JointTypeHelical; //Tested with hard coded values
    else if (joint_type == "1dof") return JointType1DoF; //Not supported
    else if (joint_type == "2dof") return JointType2DoF; //Tested with hard coded values
    else if (joint_type == "p2p") return JointType3DoF; //Tested with YAML file and works
    else if (joint_type == "4dof") return JointType4DoF; //Tested with hard coded values
    else if (joint_type == "5dof") return JointType5DoF; //Tested with hard coded values
    else if (joint_type == "6dof") return JointType6DoF; //Tested with hard coded values
    else if (joint_type == "custom") return JointTypeCustom; //Tested with hard coded values
}

std::vector<std::string> BuildRBDLModel::getAllBodyNames() {
    std::vector<std::string> bodyNames;
//    std::transform (objects_map_[msg_type].begin(), objects_map_[msg_type].end(),back_inserter(object_names), [] (std::pair<string, iBaseObjectPtr> const & pair)
    std::transform (bodyParamObjectMap_.begin(), bodyParamObjectMap_.end(), back_inserter(bodyNames), [] (std::pair<std::string, bodyParamPtr> const & pair)
    {
    return pair.first;

    });

    return bodyNames;
}

unsigned int BuildRBDLModel::getBodyId(const std::string bodyName) {
    if (rbdlObjectMap_.find(bodyName) != rbdlObjectMap_.end()){
        return rbdlObjectMap_[bodyName];
    }
    std::cout << "Body: " << bodyName << " not found in the model" << std::endl;

    return -1;
}

boost::optional<rbdlBody> BuildRBDLModel::getRBDLBody(const std::string bodyName) {
    if(rbdlBodyMap_.find(bodyName) != rbdlBodyMap_.end()) {
        return rbdlBodyMap_[bodyName];
    }
    std::cout << "Body: " << bodyName << " not found in the model" << std::endl;
    return boost::none;
}


std::unordered_map<std::string, jointParamPtr> BuildRBDLModel::getJointChildren(std::string parent) {
    if(jointParamObjectMap_.find(parent) != jointParamObjectMap_.end()) {
        return jointParamObjectMap_[parent];
    }

    std::cout << "Parent: " << parent << " does not have any children in the model" << std::endl;

//    return std::unordered_map<std::string, jointParamPtr>();
    return {};
}

void BuildRBDLModel::printBody() {
    std::unordered_map<std::string, bodyParamPtr>::iterator body_map_itr;
    for (body_map_itr = bodyParamObjectMap_.begin(); body_map_itr != bodyParamObjectMap_.end(); body_map_itr++) {
        std::cout << body_map_itr->first << std::endl;
    }
    std::cout << std::endl;
}

void BuildRBDLModel::printJoint() {
    std::unordered_map<std::string, std::unordered_map<std::string, jointParamPtr>>::iterator outter_map_itr;
    std::unordered_map<std::string, jointParamPtr>::iterator inner_map_itr;

    for (outter_map_itr = jointParamObjectMap_.begin(); outter_map_itr != jointParamObjectMap_.end(); outter_map_itr++) {
        std::string parent_node_name = outter_map_itr->first;
        std::cout << ", parent_node_name: " << parent_node_name << std::endl << "child_node_name: " ;
        for (inner_map_itr = outter_map_itr->second.begin(); inner_map_itr != outter_map_itr->second.end(); inner_map_itr++) {
            std::string joint_name = inner_map_itr->first;
            std::cout << "joint_name: " << joint_name;
            jointParamPtr jointparamPtr = inner_map_itr->second;
            std::cout << jointparamPtr->Child() << ", ";

        }
        std::cout << std::endl;
    }
}

void BuildRBDLModel::cleanUp() {
    std::unordered_map<std::string, bodyParamPtr>::iterator bodyParamMapIt;
    for ( bodyParamMapIt = bodyParamObjectMap_.begin(); bodyParamMapIt != bodyParamObjectMap_.end(); ++bodyParamMapIt ) {
        bodyParamMapIt->second->~BodyParam();
    }

    std::unordered_map<std::string, std::unordered_map<std::string, jointParamPtr> >::iterator itr;
    std::unordered_map<std::string, jointParamPtr>::iterator ptr;

    for (itr = jointParamObjectMap_.begin(); itr != jointParamObjectMap_.end(); itr++) {
        for (ptr = itr->second.begin(); ptr != itr->second.end(); ptr++) {
            ptr->second->~JointParam();
        }
    }

    delete RBDLmodel_;
    std::cout << "RBDL Model deleted" << std::endl;
}

BuildRBDLModel::~BuildRBDLModel(void){
    delete RBDLmodel_;
}
