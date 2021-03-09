#include "controller_modules/PDController.h"

/**
 * @brief Construct a new PDController::PDController object
 * 
 * @param _Kp matrix for the Kp gains
 * @param _Kd matrix forthe Kd gains
 */
PDController::PDController(const Eigen::MatrixXd& _Kp, const Eigen::MatrixXd& _Kd):
   Kp(validateMat(_Kp, _Kd)),
   Kd(validateMat(_Kd, _Kp)) //validate the size of the matrix
{
    dimensions = Kp.rows();
}

PDController::PDController()
{

}

PDController::~PDController()
{

}
/**
 * @brief initlize the Kp matrix
 * 
 * @param mat matrix to set the gains
 * @return true matrix is valid  
 * @return false matix is not valid
 */
bool PDController::setKp(const Eigen::MatrixXd& mat)
{
    int r = mat.rows();
    int c = mat.cols();

    if(r == dimensions && c == dimensions )
    {
        Kp = mat;
        return true;
    }
    else
    {
        return false;
    }

}

/**
 * @brief initlize the Kd matrix
 * 
 * @param mat matrix to set the gains 
 * @return true matrix is valid  
 * @return false matix is not valid
 */
bool PDController::setKd(const Eigen::MatrixXd& mat)
{

    int r = mat.rows();
    int c = mat.cols();

    if(r == dimensions && c == dimensions )
    {
        Kd = mat;
        return true;
    }
    else
    {
        return false;
    }

}

/**
 * @brief get the Kp matrix
 * 
 * @return Eigen::MatrixXd 
 */
Eigen::MatrixXd PDController::getKp()

{
    return Kp;
}
/**
 * @brief get the Kd matrix
 * 
 * @return Eigen::MatrixXd 
 */
Eigen::MatrixXd PDController::getKd()
{
    return Kd;
}

/**
 * @brief The matrix need to be the same shape
 * 
 * @param mat1 matrix 1 to compare
 * @param mat2 matrix 2 to compare
 * @return Eigen::MatrixXd 
 */
Eigen::MatrixXd PDController::validateMat(const Eigen::MatrixXd& mat1, const Eigen::MatrixXd& mat2)
{
    int r1 = mat1.rows();
    int c1 = mat1.cols();

    int r2 = mat2.rows();
    int c2 = mat2.cols();

    if( r1 == r2 && c1 == c2){ //make sure it is square and both are equal
       return mat1;
    }
    else{

         throw std::invalid_argument{"!"};
    }

}


/**
 * @brief calculate the torque for the cotnroller
 *  tau = Kq(q_d - q) + Kd(qd_d - q_d)
 * @param e vector of the position error
 * @param ed vector of the velocity error
 * @param torque output vector of the torque
 */
void PDController::calculate_torque(const Eigen::VectorXd &e, const Eigen::VectorXd &ed, Eigen::VectorXd &torque)
{
    torque = Kp*e + Kd*ed;
}

/**
 * @brief calls the main controller calculation function and puts the data into the required form
 * 
 * @param actual message with the current joint states
 * @param desired message with the desired jotni states
 * @param torque output of the torque requried 
 */
void PDController::update(const trajectory_msgs::JointTrajectoryPoint& actual, const trajectory_msgs::JointTrajectoryPoint& desired, std::vector<double>& torque)
{
    
    int dp_size = desired.positions.size();
    int ap_size = actual.positions.size();
    int pv_size = desired.velocities.size();
    int av_size = actual.velocities.size();


    if(dp_size == ap_size && dp_size == pv_size && dp_size == av_size )
    {
        Eigen::VectorXd e = VectToEigen(desired.positions) - VectToEigen(actual.positions);
        Eigen::VectorXd ed = VectToEigen(desired.velocities) - VectToEigen(actual.velocities); 
        Eigen::VectorXd tau_(e.rows());
        calculate_torque(e, ed, tau_);


        // tau = qdd +  Kq(q_d - q) + Kd(qd_d - q_d)
        if (desired.accelerations.size() == dp_size)
        {
            tau_ = tau_ +  VectToEigen(desired.accelerations); 
        }
        
        std::vector<double> my_tau(&tau_[0], tau_.data()+tau_.cols()*tau_.rows());
        tau = my_tau;
        torque = tau;

    }    

    
}
























