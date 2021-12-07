#include "rbdl_model_tests/Kuka.h"
#include "rbdl/Logging.h"

#include "rbdl/Model.h"
#include "rbdl/Dynamics.h"


#ifndef USE_SLOW_SPATIAL_ALGEBRA
TEST_CASE_METHOD(Kuka, __FILE__"_TestCalcDynamicPositionNeutral", "") 
{
    // We call ForwardDynamics() as it updates the spatial transformation
    // matrices
  // Initialization of the input vectors
  VectorNd Q      = VectorNd::Constant ((size_t) rbdlModel->dof_count, 0.);
  VectorNd QDot   = VectorNd::Constant ((size_t) rbdlModel->dof_count, 0.);
  VectorNd QDDot  = VectorNd::Constant ((size_t) rbdlModel->dof_count, 0.);
  VectorNd Tau    = VectorNd::Constant ((size_t) rbdlModel->dof_count, 0.);
  VectorNd TauInv = VectorNd::Constant ((size_t) rbdlModel->dof_count, 0.);

  ForwardDynamics(*rbdlModel, Q, QDot, Tau, QDDot);
  InverseDynamics(*rbdlModel, Q, QDot, QDDot, TauInv);

  
  CHECK_THAT (Tau,
              AllCloseVector(TauInv, TEST_PREC, TEST_PREC));
  std::vector<std::string> joints = baseHandler->get_joint_names();
  std::vector<std::string> baseChildren = baseHandler->get_children_names();

  CHECK (baseChildren.size() == Q.size());

  // Set to home pose
  for(int i = 0; i < 10; i++)
  {
    for(std::string joint : joints)
    {
      baseHandler->set_joint_pos<std::string>(joint, 0.0f);
      baseHandler->set_joint_effort<std::string>(joint, 0.0f);
      float joint_effort = baseHandler->get_joint_effort<std::string>(joint);
    }

    usleep(250000);
  }
}
#endif