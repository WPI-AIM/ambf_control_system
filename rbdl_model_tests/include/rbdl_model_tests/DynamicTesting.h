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

        void setTorque(const Math::VectorNd &my_tau)
        {
            //3e. Here we set the applied generalized forces to the value
            for(int i=0; i<model->dof_count; i++){                
                tau[i] = my_tau[i];
            }
        }

        //3c. Boost uses this 'operator()' function to evaluate the state
        //    derivative of the pendulum.
        void operator() (const state_type &x, 
                         state_type &dxdt, 
                         const double t){

            //3d. Here we split out q (generalized positions) and qd 
            //    (generalized velocities) from the x (state vector)
            //q
            int j = 0;            
            for(int i=0; i<model->dof_count; i++){                
                q[i] = (double)x[j];
                j++;
            }

            //qd
            for(int i=0; i<model->dof_count; i++){                
                qd[i] = (double)x[j];
                j++;
            }

            

            //3f. RBDL's ForwardDynamics function is used to evaluate
            //    qdd (generalized accelerations)
            ForwardDynamics (*model,q,qd,tau,qdd);

            //3g. Here qd, and qdd are used to populate dxdt 
            //(the state derivative)
            j = 0;
            for(int i = 0; i < model->dof_count; i++){
                dxdt[j] = (double)qd[i];
                j++;
            }            
            for(int i = 0; i < model->dof_count; i++){
                dxdt[j] = (double)qdd[i];
                j++;
            }


        }

    private:
        Model* model;
        VectorNd q, qd, qdd, tau;
        std::vector<float> my_tau;
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

void f(const state_type &x, state_type &dxdt, const double t);