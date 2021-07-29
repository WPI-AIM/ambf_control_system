#ifndef SMCCONTROLLER_H
#define SMCCONTROLLER_H
#include <Eigen/Core>
#include "controller_modules/ControllerBase.h"
#include <iostream>

class SMCController : public ControllerBase
{
     public:

          SMCController(const Eigen::MatrixXd&, const Eigen::MatrixXd&);
          SMCController();
          ~SMCController();
          bool setRho(const Eigen::MatrixXd&);
          bool setLambda(const Eigen::MatrixXd&);
          Eigen::MatrixXd getLambda();
          Eigen::MatrixXd getRho();
          void calculate_torque( const Eigen::VectorXd& e, const  Eigen::VectorXd& ed, Eigen::VectorXd& tau);
          void update(const trajectory_msgs::JointTrajectoryPoint&, const  trajectory_msgs::JointTrajectoryPoint&, std::vector<double>&);
          

     private:

          Eigen::MatrixXd lambda;
          Eigen::MatrixXd rho;
          int dimensions;
          std::vector<double> error;
          std::vector<double> tau;
          static double sign_func(double x);
          static Eigen::MatrixXd validateMat(const Eigen::MatrixXd&, const Eigen::MatrixXd&);

};


#endif
