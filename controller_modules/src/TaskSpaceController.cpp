#include "controller_modules/TaskSpaceController.h"

/*
* Constructor for the TaskSpaceController
*
*
*/
TaskSpaceController::TaskSpaceController(const std::string name, const std::string body, ros::NodeHandle* n,  PDController* controller): my_controller(*controller), nh(*n) {

    model_name = name;
    ee_body_name = body;

    client_taskSpaceBody = nh.serviceClient<rbdl_server::RBDLTaskSpaceBody>("TaskSpaceBody");
    client_invDyn = nh.serviceClient<rbdl_server::RBDLInverseDynamics>("InverseDynamics");

    xyz_actual_pub = nh.advertise<std_msgs::Float64MultiArray>("xyz_actual", 1000);

}


TaskSpaceController::~TaskSpaceController() {

}


/**
*   Calculate the end-effector (ee) position and velocity of the model in the task space
*   @param actual is the message containing the joint angles/velocities of the model in the generalized space
*   @param actual_task_space is the resulting model ee position and velocity in the task space
**/
void TaskSpaceController::actual_to_task(const trajectory_msgs::JointTrajectoryPoint& actual, trajectory_msgs::JointTrajectoryPoint& actual_task_space, Eigen::MatrixXd &jacobian) {

    rbdl_server::RBDLTaskSpaceBody taskSpaceBody_msg;               // Task space body (pos, vel, and Jacobian of ee) service call message

    taskSpaceBody_msg.request.model_name = model_name;
    taskSpaceBody_msg.request.body_name = ee_body_name;
    taskSpaceBody_msg.request.q = actual.positions;
    taskSpaceBody_msg.request.qd = actual.velocities;

    if(client_taskSpaceBody.call(taskSpaceBody_msg)) {

        // Position of EE
        geometry_msgs::Point ee_point = taskSpaceBody_msg.response.point;
        std::vector<double> task_space_pos(3, 0);
        task_space_pos[0] = ee_point.x;
        task_space_pos[1] = ee_point.y;
        task_space_pos[2] = ee_point.z;
        actual_task_space.positions = task_space_pos;
        printf("EE point: %f, %f, %f\n", ee_point.x, ee_point.y, ee_point.z);

        // Velocity (linear part) of EE
        std::vector<double> ee_vel_full(taskSpaceBody_msg.response.velocity.begin(), taskSpaceBody_msg.response.velocity.end());
        std::vector<double> task_space_vel(3, 0);
        task_space_vel[0] = ee_vel_full[3];                                             // response is translational and rotational velocity (6 values)
        task_space_vel[1] = ee_vel_full[4];                                             // the first three elements are rotational and
        task_space_vel[2] = ee_vel_full[5];                                             // the last three elements are translational
        actual_task_space.velocities = task_space_vel;
        //printf("EE velocity: %f, %f, %f\n", ptVel_response[3], ptVel_response[4], ptVel_response[5]);

        // Jacobian (linear part) of EE
        std_msgs::Float64MultiArray ee_jacobian = taskSpaceBody_msg.response.jacobian;
        int nDoF = actual.positions.size();
        Eigen::MatrixXd jacobMatFull(Eigen::MatrixXd::Zero(6, nDoF));                    // initialize the ee Jacobian matrix (6 x nDoF)
        msgToEigenMat(ee_jacobian, jacobMatFull);                                        // Eigen matrix conversion from the message
        Eigen::MatrixXd jacobV = jacobMatFull.block(3, 0, 3, nDoF);                      // split the Jacobian (v and w) to use only v part
        jacobian = jacobV;
        //printf("EE jacobian: %f\n", ee_jacobian.data[1]);


        // Publishing
        std_msgs::Float64MultiArray xyz_actual;
        xyz_actual.data = task_space_pos; //std::vector<float>(xyzPosDesired.begin(), xyzPosDesired.end());
        xyz_actual_pub.publish(xyz_actual);

    }

}


/**
*   Function to calculate the desired acceleration for the end-effector in the task space
*   @param actual is the message containing the actual task space positions and velocities of the model end-effector
*   @param desired is the message containing the desired task space positions and velocities of the model end-effector
*   @param xdd is the resulting Eigen vector of joint accelerations calculated by the controller
**/
void TaskSpaceController::calculate_xdd(const trajectory_msgs::JointTrajectoryPoint& actual, const trajectory_msgs::JointTrajectoryPoint& desired, Eigen::VectorXd &xdd) {

    int xyzSize = desired.positions.size();                                                         // should be xyz points //+ pitch/roll/yaw orientations thus a size of 6

    // Calculate the actual and desired position and velocity errors
    Eigen::VectorXd e = VectToEigen(desired.positions) - VectToEigen(actual.positions);
    Eigen::VectorXd ed = VectToEigen(desired.velocities) - VectToEigen(actual.velocities);

    // Calclate the total desired accelerations
    Eigen::VectorXd xddDesired = VectToEigen(desired.accelerations);                                // accelerations in the task-space, xyz coordinates

    Eigen::VectorXd xddController(Eigen::VectorXd::Zero(xyzSize));                                  // init to zeros
    my_controller.calculate_torque(e, ed, xddController);
    
    xdd = xddDesired + xddController; 

}


/**
*   Function to calculate the resulting torque control input from the task space controller
*   @param actual is the message containing the joint angles/velocities of the model in the generalized space
*   @param desired is the message containing the desired task space positions and velocities of the model end-effector
*   @param torque is the resulting Eigen vector of joint torques calculated by the controller 
**/
void TaskSpaceController::calculate_torque(const trajectory_msgs::JointTrajectoryPoint& actual, const trajectory_msgs::JointTrajectoryPoint& desired, Eigen::VectorXd &torque) {
    
    int nDoF = actual.positions.size();                                                             // number of joints thus degrees of freedom (DoF) of the model

    // Calculate the task space position, velocity, and Jacobian
    trajectory_msgs::JointTrajectoryPoint actual_task;
    Eigen::MatrixXd jacobEE;
    actual_to_task(actual, actual_task, jacobEE);

    // Calculate the end-effector acceleration
    Eigen::VectorXd xdd(Eigen::VectorXd::Zero(3));
    calculate_xdd(actual_task, desired, xdd);

    // Calculate the aq from the controller
    Eigen::VectorXd aq(Eigen::VectorXd::Zero(nDoF));

    Eigen::MatrixXd invJacobMat = jacobEE.completeOrthogonalDecomposition().pseudoInverse();       // Eigen QR pseudo-inverse of the Jacobian matrix
    // TODO test using the Levenberg-Marquardt (damped least squares) algoritm to filter the Jacobian: J = J' * pinv(J * J' + lambda * I), lambda << 1
    aq = invJacobMat * xdd;
    std::vector<double> a_q = eigenToVect(aq);


    // Calculate the corresponding torque input using inverse dynamics
    rbdl_server::RBDLInverseDynamics invDyn_msg;

    invDyn_msg.request.model_name = model_name;
    invDyn_msg.request.q = actual.positions;
    invDyn_msg.request.qd = actual.velocities;
    invDyn_msg.request.qdd = a_q;

    if(client_invDyn.call(invDyn_msg)){

        torque = VectToEigen(invDyn_msg.response.tau);

    }

}


/**
*   Update function of the controller called by the controller manager to return a vector of joint torques
*   @param actual is the message containing the joint angles/velocities of the model in the generalized space
*   @param desired is the message containing the desired task space positions and velocities for the model end-effector
*   @param torque is the resulting std::vector of joint torques to pass to the model 
**/
void TaskSpaceController::update(const trajectory_msgs::JointTrajectoryPoint& actual, const trajectory_msgs::JointTrajectoryPoint& desired, std::vector<double>& torque) {

    Eigen::VectorXd tau_(VectToEigen(actual.positions).rows());

    // Calculate the torque vector to pass to the model
    calculate_torque(actual, desired, tau_);

    std::vector<double> my_tau = eigenToVect(tau_);

    tau = my_tau;
    torque = my_tau;

}