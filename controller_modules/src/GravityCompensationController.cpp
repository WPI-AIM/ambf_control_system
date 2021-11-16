#include "controller_modules/GravityCompensationController.h"


GravityCompensationController::GravityCompensationController(const std::string name, ros::NodeHandle* n): nh(*n)
{
    model_name = name;
    client_ID = nh.serviceClient<rbdl_server::RBDLInverseDynamics>("InverseDynamics");
    
}


GravityCompensationController::~GravityCompensationController()
{

}


void GravityCompensationController::calculate_torque(const trajectory_msgs::JointTrajectoryPoint& actual, const trajectory_msgs::JointTrajectoryPoint& desired, Eigen::VectorXd &torque)
{
    rbdl_server::RBDLInverseDynamics dyn_msg;  

    int length = actual.positions.size();
    std::vector<double> qdd(length, 0.0);

    dyn_msg.request.q = actual.positions;
    dyn_msg.request.qd = actual.velocities;
    dyn_msg.request.qdd = qdd;

    dyn_msg.request.model_name = model_name;
    if(client_ID.call(dyn_msg))
    {
        ROS_INFO("Gravity Controller Calculated the dynamics");
        torque = VectToEigen(dyn_msg.response.tau);
    }
    else
    {
       ROS_ERROR("Dynamics Controller FAILED Calculated the dynamics");
    }
    

}

void GravityCompensationController::update(const trajectory_msgs::JointTrajectoryPoint& actual, const trajectory_msgs::JointTrajectoryPoint& desired, std::vector<double>& torque)
{

    //ROS_INFO("Dynamic server %d", VectToEigen(actual.positions).rows() );
    Eigen::VectorXd tau_( VectToEigen(actual.positions).rows());
    calculate_torque(actual, desired, tau_);
    std::vector<double> my_tau(&tau_[0], tau_.data()+tau_.cols()*tau_.rows());
    tau = my_tau;
    torque = my_tau;
    
}
