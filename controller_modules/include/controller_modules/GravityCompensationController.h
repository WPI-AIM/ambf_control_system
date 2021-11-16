#ifndef GRAVITYCOMPENSATIONCONTROLLER_H
#define GRAVITYCOMPENSATIONCONTROLLER_H
#include <Eigen/Core>
#include <iostream>
#include "ros/ros.h"
#include "controller_modules/ControllerBase.h"

#include "rbdl_server/RBDLInverseDynamics.h"

class GravityCompensationController : public ControllerBase
{
     public:
          GravityCompensationController(const std::string, ros::NodeHandle*);
          ~GravityCompensationController();
          void calculate_torque(const trajectory_msgs::JointTrajectoryPoint& actual, const trajectory_msgs::JointTrajectoryPoint& desired, Eigen::VectorXd &torque);
          void update(const trajectory_msgs::JointTrajectoryPoint&, const  trajectory_msgs::JointTrajectoryPoint&, std::vector<double>&);
     private:
          std::string model_name;
     
          ros::NodeHandle nh;
          ros::ServiceClient client_ID;
          
};


#endif
