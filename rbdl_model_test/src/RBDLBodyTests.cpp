#include "rbdl_model_test/RBDLBodyTests.h"

//PSMBodyTests::PSMBodyTests()
//{



////    buildRBDLModel_(actuator_config_file);
//    //    buildRBDLModel.printBody();

//    //    buildRBDLModel.printJoint();
////        buildRBDLModel.cleanUp();
//    buildRBDLModelPtr_ = new BuildRBDLModel(actuator_config_file_);

////    client_.connect();

////    client_.printSummary();
////    string psm_baselink = "psm/baselink";
////    cout << "psm_baselink: " << psm_baselink << "\n";
////    psm_baselink_handler_ = client_.getARigidBody(psm_baselink, true);
////    usleep(1000000);

//}


//PSMBodyTests::~PSMBodyTests(void){
//    buildRBDLModelPtr_->cleanUp();
//}


//void PSMBodyTests::cleanUp(){
//    buildRBDLModelPtr_->cleanUp();
//}

/* Tests whether the spatial inertia matches the one specified by its
 * parameters
 */



void testPrep() {
    std::cout << "Inside test Prep" << std::endl;
}

TEST ( TestComputeSpatialInertiaFromAbsoluteRadiiGyration ) {
//    testPrep();
    const std::string actuator_config_file = "/localcodebase/ambfnags92/ambf/ambf_models/descriptions/multi-bodies/robots/blender-kuka.yaml";
    BuildRBDLModelPtr buildRBDLModelPtr = new BuildRBDLModel (actuator_config_file);

    std::string baseRigidBody = buildRBDLModelPtr->getBaseRigidBody();

//    ClientPtr clientPtr = new Client();
//    clientPtr->connect();

//    std::cout << "baselink: " << baseRigidBody << "\n";
//    rigidBodyPtr baselink_handler = clientPtr->getARigidBody(baseRigidBody, true);
//    usleep(1000000);

//    tf::Quaternion rot = baselink_handler->get_rot();
//    std::cout << rot[0] << ", " << rot[1] << ", " << rot[2] << ", " << rot[3] << std::endl;

  Body body(1.1, Vector3d (1.5, 1.2, 1.3), Vector3d (1.4, 2., 3.));

  Matrix3d inertia_C (
      1.4, 0., 0.,
      0., 2., 0.,
      0., 0., 3.);

  SpatialMatrix reference_inertia (
      4.843, -1.98, -2.145, 0, -1.43, 1.32,
      -1.98, 6.334, -1.716, 1.43, 0, -1.65,
      -2.145, -1.716, 7.059, -1.32, 1.65, 0,
      0, 1.43, -1.32, 1.1, 0, 0,
      -1.43, 0, 1.65, 0, 1.1, 0,
      1.32, -1.65, 0, 0, 0, 1.1
      );

  //	cout << LogOutput.str() << endl;

  SpatialRigidBodyInertia body_rbi = SpatialRigidBodyInertia::createFromMassComInertiaC (body.mMass, body.mCenterOfMass, body.mInertia);

  CHECK_ARRAY_CLOSE (reference_inertia.data(), body_rbi.toMatrix().data(), 36, TEST_PREC);
  CHECK_ARRAY_CLOSE (inertia_C.data(), body.mInertia.data(), 9, TEST_PREC);

//  buildRBDLModelPtr_->cleanUp();
//  rbdlTestsPrep->~RbdlTestsPrep();

  buildRBDLModelPtr->~BuildRBDLModel();
//  clientPtr->cleanUp();
}


int main(int argc, char **argv) {
    rbdl_check_api_version (RBDL_API_VERSION);

    if (argc > 1) {
      std::string arg (argv[1]);

      if (arg == "-v" || arg == "--version")
        rbdl_print_version();
    }

    return UnitTest::RunAllTests ();
}
