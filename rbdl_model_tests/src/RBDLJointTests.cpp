#include "rbdl_model_tests/RBDLTestPrep.h"

TEST ( TestRBBLJointToYAML ) {
    BuildRBDLModelPtr buildRBDLModelPtr = RBDLTestPrep::getInstance()->getRBDLModelInstance();

    std::vector<std::string> bodyNames = buildRBDLModelPtr->getAllBodyNames();
    for(std::string bodyName : bodyNames) {
        std::cout << bodyName << std::endl;
    }

    std::unordered_map<std::string, jointParamPtr> jointParamObjectMap = buildRBDLModelPtr->getJointChildren(bodyNames[4]);

    std::unordered_map<std::string, jointParamPtr> ::iterator jointParamObjectMap_itr;
    for(jointParamObjectMap_itr = jointParamObjectMap.begin(); jointParamObjectMap_itr != jointParamObjectMap.end(); jointParamObjectMap_itr++) {
        std::cout << jointParamObjectMap_itr->first << std::endl;
    }

    jointParamPtr jointParam = jointParamObjectMap["base-link1"];
    Vector3d joint_parent_axis = jointParam->ParentAxis();
    Vector3d joint_parent_pivot = jointParam->ParentPivot();

}
