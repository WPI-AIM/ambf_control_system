#include "KUKATests/KUKA.h"

TEST_CASE_METHOD(KUKA, __FILE__"_TestPositionNeutral", "") 
{

  // We call ForwardDynamics() as it updates the spatial transformation
  // matrices
  // ForwardDynamics(*model, Q, QDot, Tau, QDDot);

  // Vector3d body_position;

  // CHECK_THAT (Vector3d (0., 0., 0.),
  //             AllCloseVector(
  //               CalcBodyToBaseCoordinates(*model,
  //                                         Q,
  //                                         world_baseId,
  //                                         Vector3d (0., 0., 0.),
  //                                         true),
  //               TEST_PREC, TEST_PREC)
  // );
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