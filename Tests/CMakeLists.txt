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


set(TESTS_LINK_LIBRARIES
  PRIVATE Catch2::Catch2
  PRIVATE ${Sway_BINARY_DIR}/libSway.a
  # PRIVATE ${spdlog_BINARY_DIR}/libspdlog.a
  # PRIVATE ${BOOST_LIBRARY_LIBRARY_PS}
  PRIVATE ${RBDL_BINARY_DIR}/librbdl.so
  # PRIVATE ${AMBF_CLIENT_LIBRARIES_PS}
  )

#####Beginning of Reference Test App#############
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
  ${TESTS_LINK_LIBRARIES}
)

set_target_properties(${ReferenceTestsApp} PROPERTIES
CXX_STANDARD 20
CXX_STANDARD_REQUIRED YES
)

#####End of Reference Test App#############

#####Beginning of KUKA Test App#############
SET ( KUKATests_SOURCE_FILES
  KUKATests/src/KUKAKinematicsTests.cc
)

set(KUKATestsApp KUKATests_app)
ADD_EXECUTABLE ( ${KUKATestsApp} 
  ${TESTS_SOURCE_FILES} 
  ${KUKATests_SOURCE_FILES}
  )

target_include_directories(${KUKATestsApp}
PRIVATE ${TESTS_HRD_DIR} 
PRIVATE KUKATests/include
)

target_link_libraries(${KUKATestsApp} 
  ${TESTS_LINK_LIBRARIES}
)

set_target_properties(${KUKATestsApp} PROPERTIES
CXX_STANDARD 20
CXX_STANDARD_REQUIRED YES
)

#####End of KUKA Test App#############

#####Beginning of ECM Test App#############
SET ( ECMTests_SOURCE_FILES
  ECMTests/src/ECMKinematicsTests.cc
)

set(ECMTestsApp ECMTests_app)
ADD_EXECUTABLE ( ${ECMTestsApp} 
  ${TESTS_SOURCE_FILES} 
  ${ECMTests_SOURCE_FILES}
  )

target_include_directories(${ECMTestsApp}
PRIVATE ${TESTS_HRD_DIR} 
PRIVATE ECMTests/include
)

target_link_libraries(${ECMTestsApp} 
  ${TESTS_LINK_LIBRARIES}
)

set_target_properties(${ECMTestsApp} PROPERTIES
CXX_STANDARD 20
CXX_STANDARD_REQUIRED YES
)
#####End of ECM Test App#############

#####Beginning of ECM Test App#############
SET ( ECMTests_SOURCE_FILES
  ECMTests/src/ECMMannualCalculations.cc
)

set(ECMManualCalculationsTests ECMManualCalculationsTests_app)
ADD_EXECUTABLE ( ${ECMManualCalculationsTests} 
  ${TESTS_SOURCE_FILES} 
  ${ECMTests_SOURCE_FILES}
  )

target_include_directories(${ECMManualCalculationsTests}
PRIVATE ${TESTS_HRD_DIR} 
PRIVATE ECMTests/include
)

target_link_libraries(${ECMManualCalculationsTests} 
  ${TESTS_LINK_LIBRARIES}
)

set_target_properties(${ECMManualCalculationsTests} PROPERTIES
CXX_STANDARD 20
CXX_STANDARD_REQUIRED YES
)
#####End of ECM Test App#############

#####Beginning of ECM Test App#############
SET ( ECM2Tests_SOURCE_FILES
  ECMTests/src/ECM2.cc
)

set(ECM2 ECM2_app)
ADD_EXECUTABLE ( ${ECM2} 
  ${TESTS_SOURCE_FILES} 
  ${ECM2Tests_SOURCE_FILES}
  )

target_include_directories(${ECM2}
PRIVATE ${TESTS_HRD_DIR} 
PRIVATE ECMTests/include
)

target_link_libraries(${ECM2} 
  ${TESTS_LINK_LIBRARIES}
)

set_target_properties(${ECM2} PROPERTIES
CXX_STANDARD 20
CXX_STANDARD_REQUIRED YES
)
#####End of ECM Test App#############