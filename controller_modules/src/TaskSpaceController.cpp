#include "controller_modules/TaskSpaceController.h"

/*
* Constructor for the TaskSpaceController
*
*
*/
TaskSpaceController::TaskSpaceController(const std::string name, ros::NodeHandle* n,  PDController* controller): my_controller(*controller), nh(*n) {

    model_name = name;

    client_fwdKin = nh.serviceClient<rbdl_server::RBDLKinimatics>("ForwardKinimatics");
    client_ptVel = nh.serviceClient<rbdl_server::RBDLPointVelocity>("PointVelocity");
    client_jacob = nh.serviceClient<rbdl_server::RBDLJacobian>("Jacobian");
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
void TaskSpaceController::actual_to_task(const trajectory_msgs::JointTrajectoryPoint& actual, trajectory_msgs::JointTrajectoryPoint& actual_task_space) {

    rbdl_server::RBDLKinimatics fwdKin_msg;                 // Forward kinematics (Base to body coordinates) RBDL server call message

    fwdKin_msg.request.model_name = model_name;
    fwdKin_msg.request.q = actual.positions;

    if(client_fwdKin.call(fwdKin_msg)){

        std::vector<geometry_msgs::Point> fwdKin_resp_points = fwdKin_msg.response.points;    // returns the points in 3D space at the base of the bodies that make up the model
        std::vector<std::string> fwdKin_resp_names = fwdKin_msg.response.names;

        // TODO don't hardcode this value!
        int bodyIndex = 0; // index 0 is "link7"

        ee_body_name = fwdKin_resp_names[bodyIndex];
        ee_point = fwdKin_resp_points[bodyIndex];

        std::vector<double> task_space_pos(3, 0);
        task_space_pos[0] = ee_point.x;
        task_space_pos[1] = ee_point.y;
        task_space_pos[2] = ee_point.z;

        // Publishing
        std_msgs::Float64MultiArray xyz_actual;
        xyz_actual.data = task_space_pos; //std::vector<float>(xyzPosDesired.begin(), xyzPosDesired.end());
        xyz_actual_pub.publish(xyz_actual);


        actual_task_space.positions = task_space_pos;

    }


    rbdl_server::RBDLPointVelocity ptVel_msg;               // Point velocity RBDL server call message

    ptVel_msg.request.model_name = model_name;
    ptVel_msg.request.body_name = ee_body_name;
    ptVel_msg.request.q = actual.positions;
    ptVel_msg.request.qd = actual.velocities;
    ptVel_msg.request.point = ee_point;

    if(client_ptVel.call(ptVel_msg)) {

        //std::vector<double> ptVel_response(ptVel_msg.response.velocity.begin(), ptVel_msg.response.velocity.end());
        //printf("Pt Vel response: %f, %f, %f\n", ptVel_response[3], ptVel_response[4], ptVel_response[5]);

        std::vector<double> task_space_vel(3, 0);
        task_space_vel[0] = ptVel_msg.response.velocity[3];     // response is translational and rotational velocity (6 values)
        task_space_vel[1] = ptVel_msg.response.velocity[4];     // the first three elements are rotational and
        task_space_vel[2] = ptVel_msg.response.velocity[5];     // the last three elements are translational    

        actual_task_space.velocities = task_space_vel;

    }


}


/**
*   Calculate the current Jacobian matrix for the end-effector given the current manipulator configuration
*   @param actual is the message containing the joint angles/velocities of the model in the generalized space
*   @param jacobian is the resulting Eigen matrix Jacobian of the current ee position
**/
void TaskSpaceController::get_Jacobian(const trajectory_msgs::JointTrajectoryPoint& actual, Eigen::MatrixXd &jacobian) {

    rbdl_server::RBDLJacobian jacob_msg;                                                // Jacobian RBDL server call message

    jacob_msg.request.model_name = model_name;
    jacob_msg.request.body_name = ee_body_name;
    jacob_msg.request.q = actual.positions;
    jacob_msg.request.point = ee_point;

    if(client_jacob.call(jacob_msg)){

        std_msgs::Float64MultiArray jacob_response = jacob_msg.response.jacobian;

        int nDoF = jacob_msg.request.q.size();
        
        Eigen::MatrixXd jacobMatFull(Eigen::MatrixXd::Zero(6, nDoF));                       // initialize the ee Jacobian matrix (6 x nDoF)
        msgToEigenMat(jacob_response, jacobMatFull);                                        // Eigen matrix conversion from the message

        // Get the linear velocity part of the Jacobian by removing the angular velocity part when not controlling manipulator orientation
        Eigen::MatrixXd jacobV = jacobMatFull.block(3, 0, 3, nDoF);                         // split the Jacobian (v and w) to use only v part
        //std::cout << "Jacob: " << std::endl << jacobV << std::endl;

        jacobian = jacobV;

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
*   Function to calculate the aq, the accelerations of the model joints transformed to the generalized space using the Jacobian
*   @param actual is the message containing the joint angles/velocities of the model in the generalized space
*   @param xdd is the Eigen vector of end-effector accelerations in the task space calculated by the PD controller
*   @param aq is the resulting Eigen vector of joint accelerations calculated by the controller
**/
void TaskSpaceController::calculate_aq(const trajectory_msgs::JointTrajectoryPoint& actual, Eigen::VectorXd &xdd, Eigen::VectorXd &aq) {

    int nDoF = actual.positions.size();                                                             // number of joints thus degrees of freedom (DoF) of the model

    // Calculate the inverse Jacobian matrix
    Eigen::MatrixXd jacobMat(Eigen::MatrixXd::Zero(3, nDoF));                                       // initialize the ee Jacobian matrix (3 x nDoF)
    get_Jacobian(actual, jacobMat);

    Eigen::MatrixXd invJacobMat = jacobMat.completeOrthogonalDecomposition().pseudoInverse();       // Eigen QR pseudo-inverse of the Jacobian matrix

    // TODO test using the Levenberg-Marquardt (damped least squares) algoritm to filter the Jacobian: J = J' * pinv(J * J' + lambda * I), lambda << 1

    // Calculate the aq
    aq = invJacobMat * xdd;

}


/**
*   Function to calculate the resulting torque control input from the task space controller
*   @param actual is the message containing the joint angles/velocities of the model in the generalized space
*   @param desired is the message containing the desired task space positions and velocities of the model end-effector
*   @param torque is the resulting Eigen vector of joint torques calculated by the controller 
**/
void TaskSpaceController::calculate_torque(const trajectory_msgs::JointTrajectoryPoint& actual, const trajectory_msgs::JointTrajectoryPoint& desired, Eigen::VectorXd &torque) {
    
    int nDoF = actual.positions.size();                                                         // number of joints thus degrees of freedom (DoF) of the model

    // Calculate the task space positions and velocities
    trajectory_msgs::JointTrajectoryPoint actual_task;
    actual_to_task(actual, actual_task);

    // Calculate the end-effector acceleration
    Eigen::VectorXd xdd(Eigen::VectorXd::Zero(3));
    calculate_xdd(actual_task, desired, xdd);

    // Calculate the aq from the controller
    Eigen::VectorXd aq(Eigen::VectorXd::Zero(nDoF));
    calculate_aq(actual, xdd, aq);
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


    printf("EE point: %f, %f, %f\n", ee_point.x, ee_point.y, ee_point.z);

    // printf("resp: %s\n", ee_body_name);
    //std::cout << "resp: " << ee_point.x << std::endl;

    //std::cout << "resp: " << actual_task.velocities[0] << std::endl;

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