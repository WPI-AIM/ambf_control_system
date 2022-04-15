#ifndef TASKSPACECONTROLLER_H
#define TASKSPACECONTROLLER_H

#include <Eigen/Core>
#include <Eigen/QR> 
#include "ros/ros.h"
#include "controller_modules/ControllerBase.h"
#include "controller_modules/PDController.h"
#include <iostream>
#include "rbdl_server/RBDLTaskSpaceBody.h"
#include "rbdl_server/RBDLInverseDynamics.h"


class TaskSpaceController : public ControllerBase {

     public:
          TaskSpaceController(const std::string, const std::string, ros::NodeHandle*, PDController*);
          ~TaskSpaceController();

          void calculate_torque(const trajectory_msgs::JointTrajectoryPoint& actual, const trajectory_msgs::JointTrajectoryPoint& desired, Eigen::VectorXd &torque);
          void update(const trajectory_msgs::JointTrajectoryPoint&, const  trajectory_msgs::JointTrajectoryPoint&, std::vector<double>&);

     private:
          std::string model_name;
          std::string ee_body_name;

          PDController my_controller;

          ros::NodeHandle nh;

          ros::ServiceClient client_taskSpaceBody;
          ros::ServiceClient client_invDyn;

          ros::Publisher xyz_actual_pub;


          void actual_to_task(const trajectory_msgs::JointTrajectoryPoint& actual, trajectory_msgs::JointTrajectoryPoint& actual_task_space, Eigen::MatrixXd &jacobian);
          void calculate_xdd(const trajectory_msgs::JointTrajectoryPoint& actual, const trajectory_msgs::JointTrajectoryPoint& desired, Eigen::VectorXd &xdd);

};


#endif