#ifndef SMCCONTROLLER_H
#define SMCCONTROLLER_H
#include <Eigen/Core>
#include <iostream>
#include "ros/ros.h"
#include "controller_modules/ControllerBase.h"
#include "controller_modules/PDController.h"
#include "rbdl_server/RBDLInverseDynamics.h"

class SMCController : public ControllerBase
{
     public:
          SMCController(const std::string, ros::NodeHandle*, PDController*);
          ~SMCController();
          bool setRho(const Eigen::MatrixXd&);
          bool setLambda(const Eigen::MatrixXd&);
          Eigen::MatrixXd getLambda();
          Eigen::MatrixXd getRho();
          void calculate_torque(const trajectory_msgs::JointTrajectoryPoint& actual, const trajectory_msgs::JointTrajectoryPoint& desired, Eigen::VectorXd &torque);
          void update(const trajectory_msgs::JointTrajectoryPoint&, const  trajectory_msgs::JointTrajectoryPoint&, std::vector<double>&);
     private:
          std::string model_name;
          ros::NodeHandle nh;
          ros::ServiceClient client_ID;
          Eigen::MatrixXd rho;
          Eigen::MatrixXd lambda;
          int dimensions;
          std::vector<double> error;
          std::vector<double> tau;
          static Eigen::MatrixXd validateMat(const Eigen::MatrixXd&, const Eigen::MatrixXd&);
          
};


#endif
