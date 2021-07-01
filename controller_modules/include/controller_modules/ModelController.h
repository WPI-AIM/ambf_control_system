#ifndef MODELCONTROLLER_H
#define MODELCONTROLLER_H
#include <Eigen/Core>
#include <iostream>
#include "ros/ros.h"
#include "controller_modules/ControllerBase.h"
#include "controller_modules/PDController.h"
#include "rbdl_server/RBDLInverseDynamics.h"

class ModelController : public ControllerBase
{
     public:
          ModelController(const std::string, ros::NodeHandle*, PDController*);
          ~ModelController();
          void calculate_torque(const trajectory_msgs::JointTrajectoryPoint& actual, const trajectory_msgs::JointTrajectoryPoint& desired, Eigen::VectorXd &torque);
          void update(const trajectory_msgs::JointTrajectoryPoint&, const  trajectory_msgs::JointTrajectoryPoint&, std::vector<double>&);
     private:
          std::string model_name;
          PDController my_controller;
          ros::NodeHandle nh;
          ros::ServiceClient client_ID;
          
};


#endif
