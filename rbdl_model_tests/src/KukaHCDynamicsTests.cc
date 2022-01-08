#include "rbdl_model_tests/KUKA.h"
#include "rbdl/Logging.h"

#include "rbdl/Model.h"
#include "rbdl/Dynamics.h"

#include "rbdl_model_tests/DynamicTesting.h"

#ifndef USE_SLOW_SPATIAL_ALGEBRA
TEST_CASE_METHOD(KUKA, __FILE__"_TestCalcDynamicPositionNeutral", "") 
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



TEST_CASE_METHOD(KUKA, __FILE__"_TestCalcDynamicPosition", "") 
{
    // We call ForwardDynamics() as it updates the spatial transformation
    // matrices
  // Initialization of the input vectors
  int dof = (size_t)rbdlModel->dof_count;
  VectorNd Q      = VectorNd::Constant (dof, 0.);
  VectorNd QDot   = VectorNd::Constant (dof, 0.);
  VectorNd QDDot  = VectorNd::Constant (dof, 0.);
  VectorNd Tau    = VectorNd::Constant (dof, 0.);
  VectorNd TauInv = VectorNd::Constant (dof, 0.);

  VectorNd q   = VectorNd::Zero (dof);
  VectorNd qd  = VectorNd::Zero (dof);

  //problem specific constants
  int     nPts    = 100;
  double  t0      = 0;
  double  t1      = 3;


  double t        = 0;             //time
  double ts       = 0;            //scaled time
  double tp = 0;
  double dt   = (t1-t0)/((double)nPts);   

  //Integration settings
  double absTolVal   = 1e-10;
  double relTolVal   = 1e-6;
  double a_x = 1.0 , a_dxdt = 1.0;
  rbdlToBoost rbdlBoostModel(rbdlModel);

  // model setup 
  state_type xState(dof*2);
  int steps = 0;

  for(unsigned int i=0; i < 2*dof; ++i)
  {
    q[i] = 0.0;
    qd[i+dof] = 0.0;
    xState[i] = q[i];
    xState[i+dof] = q[i+dof];
  }



  controlled_stepper_type controlled_stepper(
                                             default_error_checker< double , range_algebra , default_operations >
                                             ( absTolVal , relTolVal , a_x , a_dxdt ) 
                                             );

        

  for(int i = 0; i < nPts; i++)
  {

      t = t0 + dt*i;

      //3h. Here we integrate forward in time between a series of nPts from
      //    t0 to t1
      integrate_adaptive( 
          controlled_stepper ,
          rbdlBoostModel , xState , tp , t , (t-tp)/10 );

      tp = t;


      for(unsigned int i=0; i < 2*dof; ++i)
      {
        q[i] = xState[i];
        qd[i+dof] = xState[i+dof];
      }


     
  }


}
