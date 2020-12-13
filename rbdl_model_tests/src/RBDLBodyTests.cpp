#include "rbdl_model_tests/RBDLTestPrep.h"




/*
TEST ( TestRBBLBodyToYAML ) {
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
//    bodyParamPtr rootBodyParamPtr = buildRBDLModelPtr->getBodyParamPtr(bodyName);
//    double mass = rootBodyParamPtr->Mass();

//    std::cout << "mass: " << mass << std::endl;

//    std::unordered_map<std::string, jointParamPtr> jointChildrenMap = buildRBDLModelPtr->getJointChildren(bodyName);
//    std::unordered_map<std::string, jointParamPtr>::iterator itr;

//    if(!jointChildrenMap.empty()) {
//        for (itr = jointChildrenMap.begin(); itr != jointChildrenMap.end(); itr++) {
//            std::string jointName = itr->first;

//            std::cout << "jointName: " << jointName << std::endl;
//        }
//    }

    //    ClientPtr clientPtr = new Client();
    //    clientPtr->connect();

    //    std::cout << "baselink: " << baseRigidBody << "\n";
    //    rigidBodyPtr baselink_handler = clientPtr->getARigidBody(baseRigidBody, true);
    //    usleep(1000000);

    //    tf::Quaternion rot = baselink_handler->get_rot();
    //    std::cout << rot[0] << ", " << rot[1] << ", " << rot[2] << ", " << rot[3] << std::endl;
//    Body body(1.1, Vector3d (1.5, 1.2, 1.3), Vector3d (1.4, 2., 3.));

//    Matrix3d inertia_C (
//      1.4, 0., 0.,
//      0., 2., 0.,
//      0., 0., 3.);

//    SpatialMatrix reference_inertia (
//      4.843, -1.98, -2.145, 0, -1.43, 1.32,
//      -1.98, 6.334, -1.716, 1.43, 0, -1.65,
//      -2.145, -1.716, 7.059, -1.32, 1.65, 0,
//      0, 1.43, -1.32, 1.1, 0, 0,
//      -1.43, 0, 1.65, 0, 1.1, 0,
//      1.32, -1.65, 0, 0, 0, 1.1
//      );

    //	cout << LogOutput.str() << endl;

//    SpatialRigidBodyInertia body_rbi = SpatialRigidBodyInertia::createFromMassComInertiaC (body.mMass, body.mCenterOfMass, body.mInertia);

//    CHECK_ARRAY_CLOSE (reference_inertia.data(), body_rbi.toMatrix().data(), 36, TEST_PREC);
//    CHECK_ARRAY_CLOSE (inertia_C.data(), body.mInertia.data(), 9, TEST_PREC);

    //  buildRBDLModelPtr_->cleanUp();
    //  rbdlTestsPrep->~RbdlTestsPrep();

    buildRBDLModelPtr->~BuildRBDLModel();
    //  clientPtr->cleanUp();
}
*/

TEST ( TestTestTestModelBodyHierarchy ) {
    BuildRBDLModelPtr rbdlModelPtr = RBDLTestPrep::getInstance()->getRBDLModelInstance();
    Model model = rbdlModelPtr->getRBDLModel();

    const std::string base_body = rbdlModelPtr->getBaseRigidBody();
    const unsigned base_id = model.GetBodyId(base_body.c_str());
    const unsigned root_id = model.GetParentBodyId(base_id);
    const std::string root_body = model.GetBodyName(root_id);


    std::cout << "base_body: " << base_body << ", base_id: " << base_id
              << ", root_body: " << root_body << ", root_id: " << root_id
              << std::endl;


    std::cout << "model.q_size: " << model.q_size << std::endl;
    std::vector<std::string> bodyNames_from_model = rbdlModelPtr->getAllBodyNames();
//    for(const std::string body_name : bodyNames_from_model) {
//        unsigned int body_id = model.GetBodyId(body_name.c_str());
////        std::cout << bodyName << ", body_id: " << body_id << std::endl;
//        unsigned int parent_id = model.GetParentBodyId(body_id);
//        std::string parent_name = model.GetBodyName(parent_id);
//        std::cout << "body_name: " << body_name << ", body_id: " << body_id
//                  << ", parent_name: " << parent_name << ", parent_id: " << parent_id
//                  << std::endl;
//    }


//    std::cout << "-------------------------------------" << std::endl;

//    std::string body_name = "link7";
    std::string body_name = "ExoRightFoot";
    unsigned int body_id = model.GetBodyId(body_name.c_str());
    while(body_name != "ROOT") {
        body_name = model.GetBodyName(body_id);
        unsigned int parent_id = model.GetParentBodyId(body_id);
        std::string parent_name = model.GetBodyName(parent_id);

        std::cout << "body_name: " << body_name << ", body_id: " << body_id << ", parent_name: " << parent_name << ", parent_id: " << parent_id << std::endl;
        body_id = parent_id;
    }

//    body_name = "ExoRightThigh";
//    body_id = model.GetBodyId(body_name.c_str());
//    unsigned int parent_id = model.GetParentBodyId(body_id);
//    std::string parent_name = model.GetBodyName(parent_id);

//    std::cout << "body_name: " << model.GetBodyName(body_id) << ", body_id: " << body_id
//              << ", parent_name: " << parent_name << ", parent_id: " << parent_id
//              << std::endl;


    rbdlModelPtr->cleanUp();
}
