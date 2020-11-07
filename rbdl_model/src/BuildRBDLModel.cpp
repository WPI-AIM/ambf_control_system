#include "rbdl_model/BuildRBDLModel.h"



BuildRBDLModel::BuildRBDLModel(std::string actuator_config_file) {
    actuator_config_file_ = actuator_config_file;

    try{
        baseNode_ = YAML::LoadFile(actuator_config_file_);

    }catch (std::exception &e){
        std::cerr << "[Exception]: " << e.what() << std::endl;
        std::cerr << "ERROR! FAILED TO ACTUATOR CONFIG: " << actuator_config_file_ << std::endl;
        return;
    }
    if (baseNode_.IsNull()) return;

    this->getNamespace();
    if(!this->getBodies()) return;
    if(!this->getJoints()) return;
    if(!this->findRootNode()) return;
    this->addDummyRootJoint();
    this->buildModel();
}

void BuildRBDLModel::getNamespace() {
    Utilities utilities;

    YAML::Node blender_namespace = baseNode_["namespace"];
    if(blender_namespace.IsDefined())  blender_namespace_ = utilities.trimTrailingSpaces(blender_namespace);

//    std::cout << blender_namespace_ << std::endl;
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
        if(name.IsDefined()) {
            YAML::Node parent = baseNode_[joint_name_expanded]["parent"];
            if(parent.IsDefined()) parent_name = utilities.trimTrailingSpaces(parent);
            utilities.eraseSubStr(parent_name, "BODY");
        }

        jointParamObjectMap_.insert(std::make_pair(parent_name, std::unordered_map<std::string, jointParamPtr>()));
        jointParamObjectMap_[parent_name].insert(std::make_pair(joint_name, new JointParam(baseNode_[joint_name_expanded])));
    }
    return true;
}


bool BuildRBDLModel::findRootNode() {
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
        std::cout << "No root node found, invalid model." << std::endl;
        return false;
    }
    if(parentNodeSet.size() > 1) {
        std::cout << "Found more than one root, make sure that model in YAML file has just one root." << std::endl;
        return false;
    }

    std::unordered_set<std::string>::iterator it = parentNodeSet.begin();
    rootRigidBody_ = *it;
    std::cout << "root node: " << *it << std::endl;

    return true;
}

void BuildRBDLModel::addDummyRootJoint() {

    // Create RBDL Joint for root
    std::string joint_type = "fixed";

    // Create RBDL ID for root
    Vector3d parent_axis(0., 0., 0.);
    Vector3d parent_pivot(0., 0., 0.);

    root_joint_name_ = root_parent_name_ + "-" + rootRigidBody_;


    jointParamObjectMap_[root_parent_name_].insert(std::make_pair(root_joint_name_, new JointParam(root_joint_name_, root_parent_name_, rootRigidBody_, parent_axis, parent_pivot, joint_type)));

}

bool BuildRBDLModel::buildModel() {
    rbdl_check_api_version(RBDL_API_VERSION);

    RBDLmodel_ = new Model();
    RBDLmodel_->gravity = Vector3d(0., 0., -9.81); // in my case should set in the Z direction

    std::unordered_map<std::string, std::unordered_map<std::string, jointParamPtr>>::iterator outter_map_itr;
    std::unordered_map<std::string, jointParamPtr>::iterator inner_map_itr;

    std::unordered_set<std::string> ancestry_set;
    std::unordered_set<std::string>::iterator ancestry_set_itr;

    unsigned int root_id = addBodyToRBDL(root_parent_name_, 0, root_joint_name_, rootRigidBody_);

    rbdlObjectMap_.insert(std::make_pair(rootRigidBody_, root_id));

    ancestry_set.emplace(rootRigidBody_);
    while(!ancestry_set.empty()) {
        ancestry_set_itr = ancestry_set.begin();
        std::string parent_name = *ancestry_set_itr;
        ancestry_set.erase(*ancestry_set_itr);

        outter_map_itr = jointParamObjectMap_.find(parent_name);
        if(outter_map_itr != jointParamObjectMap_.end()) {
            unsigned int parent_id = rbdlObjectMap_[parent_name];
            std::cout << "parent_name: " << parent_name << ", parent_id: " << parent_id << ", its children: ";
            for (inner_map_itr = outter_map_itr->second.begin(); inner_map_itr != outter_map_itr->second.end(); inner_map_itr++) {

                std::string child_name = inner_map_itr->second->Child();
                std::cout << child_name << ", ";

                // Create RBDL Joint between parent and child
                std::string joint_name = inner_map_itr->first;
                unsigned int child_id = addBodyToRBDL(parent_name, parent_id, joint_name, child_name);


                rbdl_object_map_itr_ = rbdlObjectMap_.find((child_name));
                if(rbdl_object_map_itr_ == rbdlObjectMap_.end()) {
                    ancestry_set.emplace(child_name);
                    rbdlObjectMap_.insert(std::make_pair(child_name, child_id));
                }
            }
            std::cout << std::endl;
        }

    }

    std::cout << "rbdlObjectMap: " << std::endl;
    for(rbdl_object_map_itr_ = rbdlObjectMap_.begin(); rbdl_object_map_itr_ != rbdlObjectMap_.end(); rbdl_object_map_itr_++) {
        std::cout << rbdl_object_map_itr_->first << ": " <<rbdl_object_map_itr_->second << std::endl;

    }
    return true;
}

unsigned int  BuildRBDLModel::addBodyToRBDL(std::string parent_name, unsigned int parent_id, std::string joint_name, std::string child_name) {
    // Create RBDL body for child joint
    double mass = (bodyParamObjectMap_[child_name])->Mass();
    Vector3d com = (bodyParamObjectMap_[child_name])->InertialOffsetOrientation();
    Vector3d inertia = (bodyParamObjectMap_[child_name])->Inertia();
//    std::cout << "child_name: " << child_name << ": inertia: " << inertia[0] << ", " << inertia[1] << ", " << inertia[2] << std::endl;

    Body child_body = Body(mass, com, inertia);

    // Create RBDL Joint between parent and child
    std::string joint_type_str = (jointParamObjectMap_[parent_name][joint_name])->Type();
    RigidBodyDynamics::JointType joint_type = getRBDLJointType(joint_type_str);
    std::cout << "joint_type_str: " << joint_type_str << ", joint_type: " << joint_type << std::endl;

//    Vector3d parent_axis = (jointParamObjectMap_[parent_name][joint_name])->ParentAxis();
//    std::cout << "parent_axis : " << parent_axis [0] << ", " << parent_axis[1] << ", " << parent_axis[2] << std::endl;
//    Joint rbdl_joint = Joint(joint_type, parent_axis);



//    joint_type = JointType6DoF;
//    std::cout << "joint_type: " << joint_type << std::endl;

    // Get Child transformation and rotation w.r.t parent
    Vector3d child_translation = (jointParamObjectMap_[parent_name][joint_name])->ParentPivot();
    Matrix3_t body_rotation = (jointParamObjectMap_[parent_name][joint_name])->BodyRotation();

    Joint rbdl_joint;
    if(joint_type == JointTypeUndefined) {
        rbdl_joint = Joint();
    } else if(joint_type == JointTypeRevoluteX || joint_type == JointTypeRevoluteY || joint_type == JointTypeRevoluteZ ||
              joint_type == JointTypeSpherical || joint_type == JointTypeEulerZYX  || joint_type == JointTypeEulerXYZ ||
              joint_type == JointTypeEulerYXZ  || joint_type == JointTypeFixed     || joint_type == JointTypeFloatingBase ||
              joint_type == JointTypeTranslationXYZ) {
        rbdl_joint = Joint (joint_type);
    } else if(joint_type == JointTypeCustom ) {
        int degreesOfFreedom = 0;
        rbdl_joint = Joint (joint_type, degreesOfFreedom);
    } else if(joint_type == JointTypeRevolute || joint_type == JointTypePrismatic) {
        Vector3d parent_axis = (jointParamObjectMap_[parent_name][joint_name])->ParentAxis();
        rbdl_joint = Joint(joint_type, parent_axis);
    } else if(joint_type == JointTypeRevoluteX || joint_type == JointTypeRevoluteY || joint_type == JointTypeRevoluteZ || joint_type == JointTypeHelical) {
        //Get this value from yaml. hardcoding it for testing
//        1 DoF joint with the given motion subspaces.
//        \f[ (r_x, r_y, r_z, t_x, t_y, t_z) \f]
        const Math::SpatialVector axis_0 = Math::SpatialVector(0., 0., 1., 0., 0., 0.);
        rbdl_joint = Joint(axis_0);
    } else if(joint_type == JointType2DoF) {
        //Get this value from yaml. hardcoding it for testing
//        2 DoF joint with the given motion subspaces.
//        \f[ (r_x, r_y, r_z, t_x, t_y, t_z) \f]
        const Math::SpatialVector axis_0 = Math::SpatialVector(0., 0., 1., 0., 0., 0.);
        const Math::SpatialVector axis_1 = Math::SpatialVector(0., 0., 1., 0., 0., 0.);

        rbdl_joint = Joint(axis_0, axis_1);
    } else if(joint_type == JointType3DoF) {
        //Get this value from yaml. hardcoding it for testing
//        3 DoF joint with the given motion subspaces.
//        \f[ (r_x, r_y, r_z, t_x, t_y, t_z) \f]
        //Body rotaion used to create create Joint as well as child_tf - check this to be right implementation
        const Math::SpatialVector axis_0 = Math::SpatialVector(body_rotation(0, 0), body_rotation(0, 1), body_rotation(0, 2), 0., 0., 0.);
        const Math::SpatialVector axis_1 = Math::SpatialVector(body_rotation(1, 0), body_rotation(1, 1), body_rotation(2, 2), 0., 0., 0.);
        const Math::SpatialVector axis_2 = Math::SpatialVector(body_rotation(2, 0), body_rotation(2, 1), body_rotation(2, 2), 0., 0., 0.);

        rbdl_joint = Joint(axis_0, axis_1, axis_2);
    } else if(joint_type == JointType4DoF) {
        //Get this value from yaml. hardcoding it for testing
//        2 DoF joint with the given motion subspaces.
//        \f[ (r_x, r_y, r_z, t_x, t_y, t_z) \f]
        const Math::SpatialVector axis_0 = Math::SpatialVector(0., 0., 1., 0., 0., 0.);
        const Math::SpatialVector axis_1 = Math::SpatialVector(0., 0., 1., 0., 0., 0.);
        const Math::SpatialVector axis_2 = Math::SpatialVector(0., 0., 1., 0., 0., 0.);
        const Math::SpatialVector axis_3 = Math::SpatialVector(0., 0., 1., 0., 0., 0.);

        rbdl_joint = Joint(axis_0, axis_1, axis_2, axis_3);
    } else if(joint_type == JointType5DoF) {
        //Get this value from yaml. hardcoding it for testing
//        2 DoF joint with the given motion subspaces.
//        \f[ (r_x, r_y, r_z, t_x, t_y, t_z) \f]
        const Math::SpatialVector axis_0 = Math::SpatialVector(0., 0., 1., 0., 0., 0.);
        const Math::SpatialVector axis_1 = Math::SpatialVector(0., 0., 1., 0., 0., 0.);
        const Math::SpatialVector axis_2 = Math::SpatialVector(0., 0., 1., 0., 0., 0.);
        const Math::SpatialVector axis_3 = Math::SpatialVector(0., 0., 1., 0., 0., 0.);
        const Math::SpatialVector axis_4 = Math::SpatialVector(0., 0., 1., 0., 0., 0.);

        rbdl_joint = Joint(axis_0, axis_1, axis_2, axis_3, axis_4);
    } else if(joint_type == JointType6DoF) {
        //Get this value from yaml. hardcoding it for testing
//        2 DoF joint with the given motion subspaces.
//        \f[ (r_x, r_y, r_z, t_x, t_y, t_z) \f]
        const Math::SpatialVector axis_0 = Math::SpatialVector(0., 0., 1., 0., 0., 0.);
        const Math::SpatialVector axis_1 = Math::SpatialVector(0., 0., 1., 0., 0., 0.);
        const Math::SpatialVector axis_2 = Math::SpatialVector(0., 0., 1., 0., 0., 0.);
        const Math::SpatialVector axis_3 = Math::SpatialVector(0., 0., 1., 0., 0., 0.);
        const Math::SpatialVector axis_4 = Math::SpatialVector(0., 0., 1., 0., 0., 0.);
        const Math::SpatialVector axis_5 = Math::SpatialVector(0., 0., 1., 0., 0., 0.);

        rbdl_joint = Joint(axis_0, axis_1, axis_2, axis_3, axis_4, axis_5);
    } else if(joint_type == JointTypeFloatingBase || joint_type == JointTypeFixed) {
//        std::cout << "inside else - joint_type: " << joint_type << std::endl;
        rbdl_joint = Joint(joint_type);
    } else {
        std::ostringstream errormsg;
        errormsg <<
                 "Error: Unsupported Joint type defined"
                 << std::endl;
        throw Errors::RBDLError(errormsg.str());
    }
//    std::cout << "joint_type: " << joint_type << ", rbdl_joint.mJointType: " << rbdl_joint.mJointType << std::endl;
    // Create RBDL ID for parent to child

    SpatialTransform child_tf(body_rotation, child_translation);

//    std::cout << "parent_id: " << parent_id << std::endl;
//    std::cout << "child_body: " << std::endl;
//    std::cout << child_translation(2) << std::endl;

    unsigned int child_id = RBDLmodel_->AddBody(parent_id, child_tf, rbdl_joint, child_body);

//    Matrix3_t body_rotation_trial;
//    body_rotation_trial <<  0., 0., 1.,
//                            0., -1., 0.,
//                            1., 0., 0.;

//    Vector3d child_translation_trial(0.,0.,0.);
//    SpatialTransform child_tf_trial(body_rotation_trial, child_translation_trial);

//    RBDLmodel_->AddBody(parent_id, child_tf_trial, rbdl_joint, child_body, "abc");

    return child_id;
}

const RigidBodyDynamics::JointType BuildRBDLModel::getRBDLJointType(std::string joint_type) {
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
}

BuildRBDLModel::~BuildRBDLModel(void){

}
