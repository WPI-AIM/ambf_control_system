
#include "rbdl/Model.h"
#include "rbdl/Dynamics.h"

#include <string>
#include <iostream>
#include <stdio.h> 
#include <rbdl/rbdl.h>


#include <boost/numeric/odeint/stepper/runge_kutta_cash_karp54.hpp>
#include <boost/numeric/odeint/stepper/controlled_runge_kutta.hpp>
#include <boost/numeric/odeint/integrate/integrate_adaptive.hpp>
#include <boost/numeric/odeint/stepper/generation/make_controlled.hpp>
//using namespace std;
using namespace boost::numeric::odeint;
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;



//====================================================================
// Boost stuff
//====================================================================

typedef std::vector< double > state_type;

typedef runge_kutta_cash_karp54< state_type > error_stepper_type;
typedef controlled_runge_kutta< error_stepper_type > controlled_stepper_type;


class rbdlToBoost {

    public:
        rbdlToBoost(Model* model) : model(model) {
            q = VectorNd::Zero(model->dof_count);
            qd = VectorNd::Zero(model->dof_count);
            qdd = VectorNd::Zero(model->dof_count);
            tau = VectorNd::Zero(model->dof_count);

        }
        void operator() ( const state_type &x , state_type &dxdt , const double /* t */ )
        {
        dxdt[0] = x[1];
        dxdt[1] = -x[0] - 5*x[1];
        }
      
    private:
        Model* model;
        VectorNd q, qd, qdd, tau;
};

struct pushBackStateAndTime
{
    std::vector< state_type >& states;
    std::vector< double >& times;

    pushBackStateAndTime( std::vector< state_type > &states , 
                              std::vector< double > &times )
    : states( states ) , times( times ) { }

    void operator()( const state_type &x , double t )
    {
        states.push_back( x );
        times.push_back( t );
    }
};


struct write_state
{
    void operator()( const state_type &x ) const
    {
        std::cout << x[0] << "\t" << x[1] << "\n";
    }
};


// void f(const state_type &x, state_type &dxdt, const double t);

int main(int argc, char const *argv[])
{
  int     nPts    = 100;
  double  t0      = 0;
  double  t1      = 3;


  double t        = 0;             //time
  double ts       = 0;            //scaled time
  double dtsdt    = M_PI/(t1-t0);    //dertivative scaled time 
  double tp = 0;
  double dt   = (t1-t0)/((double)nPts);   

  //Integration settings
  double absTolVal   = 1e-10;
  double relTolVal   = 1e-6;
  double a_x = 1.0 , a_dxdt = 1.0;
  Model* model  = NULL;
  model         = new Model();
  rbdlToBoost rbdlBoostModel(model);
  state_type xState(2);
  int steps = 0;
  xState[0] = -M_PI/4.0;
  xState[1] = 0;
  controlled_stepper_type controlled_stepper;
// integrate_adaptive( 
//         controlled_stepper ,
//         model , xState , tp , t , (t-tp)/10 );

    integrate_adaptive( controlled_stepper , rbdlBoostModel , xState , 0.0 , 10.0 , 0.01 );

      tp = t;

  return 0;
}