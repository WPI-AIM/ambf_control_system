#include "rbdl_model_tests/KUKA.h"
#include "rbdl/Logging.h"

#include "rbdl/Model.h"
#include "rbdl/Dynamics.h"
#include <ros/ros.h>
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
  ros::Rate loop_rate(1000);

  // hold some information
  VectorNd q   = VectorNd::Zero (dof);
  VectorNd qd  = VectorNd::Zero (dof);
  std::vector<std::vector< double > > trajData;
  std::vector<std::vector< float > > AMBFTrajData;
  std::vector<std::vector< float > > tauData;
  std::vector< double > rowData(2*dof+1);
  std::vector< float > tauRowData(dof);
  
  //problem specific constants
  int     nPts    = 1000;
  double  t0      = 0;
  double  t1      = 1.0;// 1s


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


  std::vector<float> joints_positions = baseHandler->get_all_joint_pos();
  std::vector<float> joints_velocity = baseHandler->get_all_joint_pos();

  for(unsigned int i=0; i < 2*dof; ++i)
  {
    q[i] = (double)joints_positions[i];
    qd[i+dof] = (double)joints_velocity[i];
    xState[i] = q[i];
    xState[i+dof] = q[i+dof];
  }


  controlled_stepper_type controlled_stepper(
                                             default_error_checker< double , range_algebra , default_operations >
                                             ( absTolVal , relTolVal , a_x , a_dxdt ) 
                                             );

        
  for(int i = 0; i < nPts; i++)
  {

      for(unsigned int j=0; j<dof; ++j)
      {
        Q[j] = xState[j];
        QDot[j] = xState[j+dof];
        QDDot[j] = 0.0;
        Tau[j] = 0.0;
      }
      // try to do gravity compenstation
      InverseDynamics(*rbdlModel,Q,QDot,QDDot,Tau);
      //update the torque somehow
      rbdlBoostModel.setTorque(Tau);
      
      //3h. Here we integrate forward in time between a series of nPts from
      //    t0 to t1
      t = t0 + dt*i;
      vector<double> times;
      vector<state_type> x_vec;
      
      integrate_adaptive(controlled_stepper , rbdlBoostModel , xState , tp , t , (t-tp)/10 , pushBackStateAndTime( x_vec , times ) );
      tp = t;

      // save the data
      rowData[0] = t;
      for(unsigned int z=0; z < 2*dof; z++){
          rowData[z+1] = xState[z];
      }
      trajData.push_back(rowData);

      //lets save the applied torque
      for(unsigned int z=0; z < dof; z++){
          tauRowData[z] = (float)Tau[i];
      }
      tauData.push_back(tauRowData);
     
     
  }

  //set the joint position to 0
  std::vector<float> init_pos(dof, 0);

  baseHandler->set_all_joint_pos(init_pos);
  int count = 0;
  //open loop control of the model
  while(ros::ok())
  {
    std::vector< float > current_tau = tauData[count];
    baseHandler->set_all_joint_effort(current_tau);
    std::vector<float> curr = baseHandler->get_all_joint_pos();
    AMBFTrajData.push_back(curr);
    loop_rate.sleep();
  }


}
 #endif
