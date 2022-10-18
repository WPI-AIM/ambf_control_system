cmake_minimum_required(VERSION 3.0.0)
project(Tests VERSION 2.0)

set(TESTS_HRD_DIR
  include
  ${Sway_SOURCE_DIR}
  ${spdlog_INCLUDE_DIRS_PS}
  ${PCH_INCLUDE_DIRS}
  ${Catch2_INCLUDE_DIRS_PS}
  ${Eigen3_INCLUDE_DIRS_PS}
  ${RBDL_CONFIG_INCLUDE_DIRS_PS}
  ${RBDL_INCLUDE_DIRS_PS}

  ${ROS_INCLUDE_DIRS_PS}
  ${AMBF_CLIENT_INCLUDE_DIRS_PS}
  ${AMBF_MSGS_INCLUDE_DIRS_PS}
  )

SET ( TESTS_SOURCE_FILES
  src/main.cc
  )


SET ( ReferenceTests_SOURCE_FILES
  ReferenceTests/src/BodyTests.cc
  ReferenceTests/src/MathTests.cc
  ReferenceTests/src/ErrorTests.cc
  ReferenceTests/src/SpatialAlgebraTests.cc
  ReferenceTests/src/MultiDofTests.cc
  ReferenceTests/src/KinematicsTests.cc
  ReferenceTests/src/BodyTests.cc
  ReferenceTests/src/ModelTests.cc
  ReferenceTests/src/FloatingBaseTests.cc
  ReferenceTests/src/CalcVelocitiesTests.cc
  ReferenceTests/src/CalcAccelerationsTests.cc
  ReferenceTests/src/DynamicsTests.cc
  ReferenceTests/src/InverseDynamicsTests.cc
  ReferenceTests/src/CompositeRigidBodyTests.cc
  ReferenceTests/src/ImpulsesTests.cc
  ReferenceTests/src/TwolegModelTests.cc
  ReferenceTests/src/ContactsTests.cc
  ReferenceTests/src/UtilsTests.cc
  ReferenceTests/src/SparseFactorizationTests.cc
  ReferenceTests/src/CustomJointSingleBodyTests.cc
  ReferenceTests/src/CustomJointMultiBodyTests.cc
  ReferenceTests/src/CustomConstraintsTests.cc
  ReferenceTests/src/InverseKinematicsTests.cc
  ReferenceTests/src/LoopConstraintsTests.cc
  ReferenceTests/src/ScrewJointTests.cc
  ReferenceTests/src/ForwardDynamicsConstraintsExternalForces.cc
  ReferenceTests/src/InverseDynamicsWithConstraintsTests.cc  
)

set(ReferenceTestsApp ReferenceTests_app)
ADD_EXECUTABLE ( ${ReferenceTestsApp} 
  ${TESTS_SOURCE_FILES} 
  ${ReferenceTests_SOURCE_FILES}
  )

target_include_directories(${ReferenceTestsApp}
PRIVATE ${TESTS_HRD_DIR} 
PRIVATE ReferenceTests/include
)


target_link_libraries(${ReferenceTestsApp} 
PRIVATE Catch2::Catch2
PRIVATE ${Sway_BINARY_DIR}/libSway.a
PRIVATE ${spdlog_BINARY_DIR}/libspdlog.a
PRIVATE ${RBDL_BINARY_DIR}/librbdl.so
PRIVATE ${AMBF_CLIENT_LIBRARIES_PS}
)


set_target_properties(${ReferenceTestsApp} PROPERTIES
CXX_STANDARD 20
CXX_STANDARD_REQUIRED YES
)