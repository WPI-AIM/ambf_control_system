#include "rbdl_model_tests/RBDLTestPrep.h"

TEST ( TestRBBLBodyToYAML ) {
    BuildRBDLModelPtr buildRBDLModelPtr = RBDLTestPrep::getInstance()->getRBDLModelInstance();

    std::string baseRigidBody = buildRBDLModelPtr->getBaseRigidBody();

    std::vector<std::string> bodyNames = buildRBDLModelPtr->getAllBodyNames();

    for(std::string bodyName : bodyNames) {
        std::cout << bodyName << std::endl;
    }

    const std::string bodyName = "link1";
    std::cout << "ID for body: " << bodyName << ", is " << buildRBDLModelPtr->getBodyId(bodyName) << std::endl;
    boost::optional<rbdlBody> bodyBoostRbdlBody = buildRBDLModelPtr->getRBDLBody(bodyName);
    rbdlBody bodyRbdlBody =bodyBoostRbdlBody.get();

    const Math::Vector3d com_yaml(0.0, -0.0169, 0.1342);

    const Math::Matrix3d inertia_yaml (
    0.0453,     0.0,    0.0,
    0.0,        0.0446, 0.0,
    0.0,        0.0,    0.0041
    );

    CHECK_CLOSE(1.0, bodyRbdlBody.mMass, TEST_PREC);
    CHECK_ARRAY_CLOSE(inertia_yaml.data(), bodyRbdlBody.mInertia.data(), 9, TEST_PREC);
    CHECK_ARRAY_CLOSE(com_yaml.data(), bodyRbdlBody.mCenterOfMass.data(), 3, TEST_PREC);
    CHECK_ARRAY_CLOSE(inertia_yaml.data(), bodyRbdlBody.mInertia.data(), 9, TEST_PREC);
}


