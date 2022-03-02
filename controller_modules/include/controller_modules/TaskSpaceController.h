#ifndef TASKSPACECONTROLLER_H
#define TASKSPACECONTROLLER_H

#include <Eigen/Core>
#include <Eigen/QR> 
#include "ros/ros.h"
#include "controller_modules/ControllerBase.h"
#include "controller_modules/PDController.h"
#include <iostream>
#include "rbdl_server/RBDLKinimatics.h"
#include "rbdl_server/RBDLPointVelocity.h"
#include "rbdl_server/RBDLJacobian.h"
#include "rbdl_server/RBDLInverseDynamics.h"


class TaskSpaceController : public ControllerBase {

     public:
          TaskSpaceController(const std::string, ros::NodeHandle*, PDController*);
          ~TaskSpaceController();

          void actual_to_task(const trajectory_msgs::JointTrajectoryPoint& actual, trajectory_msgs::JointTrajectoryPoint& actual_task_space);
          void get_Jacobian(const trajectory_msgs::JointTrajectoryPoint& actual, Eigen::MatrixXd &jacobian);
          void calculate_xdd(const trajectory_msgs::JointTrajectoryPoint& actual, const trajectory_msgs::JointTrajectoryPoint& desired, Eigen::VectorXd &xdd);
          void calculate_aq(const trajectory_msgs::JointTrajectoryPoint& actual, Eigen::VectorXd &xdd, Eigen::VectorXd &aq);

          void calculate_torque(const trajectory_msgs::JointTrajectoryPoint& actual, const trajectory_msgs::JointTrajectoryPoint& desired, Eigen::VectorXd &torque);
          void update(const trajectory_msgs::JointTrajectoryPoint&, const  trajectory_msgs::JointTrajectoryPoint&, std::vector<double>&);

     private:
          std::string model_name;

          std::string ee_body_name;
          geometry_msgs::Point ee_point;

          PDController my_controller;

          ros::NodeHandle nh;
          ros::ServiceClient client_fwdKin;
          ros::ServiceClient client_ptVel;
          ros::ServiceClient client_jacob;
          ros::ServiceClient client_invDyn;

          ros::Publisher xyz_actual_pub;

};


#endif