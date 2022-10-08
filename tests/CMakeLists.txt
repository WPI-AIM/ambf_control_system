CMAKE_MINIMUM_REQUIRED (VERSION 3.0)

SET ( TESTS_SRCS
  main.cc
  MathTests.cc
  ErrorTests.cc
  SpatialAlgebraTests.cc
  MultiDofTests.cc
  KinematicsTests.cc
  BodyTests.cc
  ModelTests.cc
  FloatingBaseTests.cc
  CalcVelocitiesTests.cc
  CalcAccelerationsTests.cc
  DynamicsTests.cc
  InverseDynamicsTests.cc
  CompositeRigidBodyTests.cc
  ImpulsesTests.cc
  TwolegModelTests.cc
  ContactsTests.cc
  UtilsTests.cc
  SparseFactorizationTests.cc
  CustomJointSingleBodyTests.cc
  CustomJointMultiBodyTests.cc
  CustomConstraintsTests.cc
  InverseKinematicsTests.cc
  LoopConstraintsTests.cc
  ScrewJointTests.cc
  ForwardDynamicsConstraintsExternalForces.cc
  InverseDynamicsWithConstraintsTests.cc  
  )


ADD_EXECUTABLE ( rbdl_tests ${TESTS_SRCS} )

target_link_directories(rbdl_tests 
PRIVATE ${RBDL_DIR}
)

SET (RBDL_LIBRARY rbdl)
IF (RBDL_BUILD_STATIC)
  SET (RBDL_LIBRARY rbdl-static)
ENDIF (RBDL_BUILD_STATIC)

TARGET_LINK_LIBRARIES ( rbdl_tests
  PRIVATE Catch2::Catch2
  PRIVATE ${RBDL_LIBRARY}
  PRIVATE rbdl
  )


set_target_properties(rbdl_tests PROPERTIES
CXX_STANDARD 20
CXX_STANDARD_REQUIRED YES
)