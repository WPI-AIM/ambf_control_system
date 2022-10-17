cmake_minimum_required(VERSION 3.0.0)
project(Tests VERSION 2.0)

set(TESTS_HRD_DIR
  include
)

SET ( TESTS_SOURCE_FILES
  src/main.cc
  # MathTests.cc
  # ErrorTests.cc
  # SpatialAlgebraTests.cc
  # MultiDofTests.cc
  # KinematicsTests.cc
  # BodyTests.cc
  # ModelTests.cc
  # FloatingBaseTests.cc
  # CalcVelocitiesTests.cc
  # CalcAccelerationsTests.cc
  # DynamicsTests.cc
  # InverseDynamicsTests.cc
  # CompositeRigidBodyTests.cc
  # ImpulsesTests.cc
  # TwolegModelTests.cc
  # ContactsTests.cc
  # UtilsTests.cc
  # SparseFactorizationTests.cc
  # CustomJointSingleBodyTests.cc
  # CustomJointMultiBodyTests.cc
  # CustomConstraintsTests.cc
  # InverseKinematicsTests.cc
  # LoopConstraintsTests.cc
  # ScrewJointTests.cc
  # ForwardDynamicsConstraintsExternalForces.cc
  # InverseDynamicsWithConstraintsTests.cc  
  )

set(ReferenceTests ReferenceTests_app)
ADD_EXECUTABLE ( ${ReferenceTests} ${TESTS_SOURCE_FILES} )

target_include_directories(${ReferenceTests}
# PRIVATE ${TESTS_HRD_DIR} 
PRIVATE ${Sway_SOURCE_DIR}
PRIVATE ${spdlog_INCLUDE_DIRS_PS}
PRIVATE ${PCH_INCLUDE_DIRS}
PRIVATE ${Catch2_INCLUDE_DIRS_PS}
# PRIVATE ${RBDL_INCLUDE_DIRS_PS}
)

target_link_libraries(${ReferenceTests} 
PRIVATE Catch2::Catch2
PRIVATE ${Sway_BINARY_DIR}/libSway.a
PRIVATE ${spdlog_BINARY_DIR}/libspdlog.a)
# PRIVATE ${RBDL_BINARY_DIR}/librbdl.so)
# target_link_directories(rbdl_tests 
# PRIVATE ${RBDL_DIR}
# )

# SET (RBDL_LIBRARY rbdl)
# IF (RBDL_BUILD_STATIC)
#   SET (RBDL_LIBRARY rbdl-static)
# ENDIF (RBDL_BUILD_STATIC)

# TARGET_LINK_LIBRARIES ( rbdl_tests
  # PRIVATE Catch2::Catch2
  # PRIVATE ${RBDL_LIBRARY}
  # PRIVATE rbdl
  # )


set_target_properties(${ReferenceTests} PROPERTIES
CXX_STANDARD 20
CXX_STANDARD_REQUIRED YES
)