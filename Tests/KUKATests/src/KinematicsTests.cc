// #include <iostream>
// #include <limits>

// #include "rbdl/Logging.h"

// #include "rbdl/Model.h"
// #include "rbdl/Kinematics.h"
// #include "rbdl/Dynamics.h"
// #include "rbdl/Constraints.h"

// #include "Tests/rbdl_tests.h"

#include "KUKATests/KUKA.h"
// #include "rbdl/rbdl.h"

// using namespace std;
// using namespace RigidBodyDynamics;
// using namespace RigidBodyDynamics::Math;

// const double TEST_PREC = 1.0e-14;

TEST_CASE_METHOD(KUKA, __FILE__"_TestPositionNeutral", "") {
  // We call ForwardDynamics() as it updates the spatial transformation
  // matrices
  ForwardDynamics(*model, Q, QDot, Tau, QDDot);

  Vector3d body_position;

  CHECK_THAT (Vector3d (0., 0., 0.),
              AllCloseVector(
                CalcBodyToBaseCoordinates(*model,
                                          Q,
                                          world_baseId,
                                          Vector3d (0., 0., 0.),
                                          true),
                TEST_PREC, TEST_PREC)
  );
  // CHECK_THAT (Vector3d (1., 0., 0.),
  //             AllCloseVector(
  //               CalcBodyToBaseCoordinates(*model,
  //                                         Q,
  //                                         body_b_id,
  //                                         Vector3d (0., 0., 0.),
  //                                         true),
  //               TEST_PREC, TEST_PREC)
  // );
  // CHECK_THAT (Vector3d (1., 1., 0.),
  //             AllCloseVector(
  //               CalcBodyToBaseCoordinates(*model,
  //                                         Q,
  //                                         body_c_id,
  //                                         Vector3d (0., 0., 0.),
  //                                         true),
  //               TEST_PREC, TEST_PREC)
  // );
  // CHECK_THAT (Vector3d (1., 1., -1.),
  //             AllCloseVector(
  //               CalcBodyToBaseCoordinates(*model,
  //                                         Q,
  //                                         body_d_id,
  //                                         Vector3d (0., 0., 0.),
  //                                         true),
  //               TEST_PREC, TEST_PREC)
  // );
}