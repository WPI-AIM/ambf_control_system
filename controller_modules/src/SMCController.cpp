#include "controller_modules/SMCController.h"


SMCController::SMCController(const Eigen::MatrixXd& _lambda, const Eigen::MatrixXd& _rho):
   rho(validateMat(_lambda, _rho)),
   lambda(validateMat(_rho, _lambda)) //validate the size of the matrix
{
    dimensions = lambda.rows();
}

SMCController::SMCController()
{

}

SMCController::~SMCController()
{

}

bool SMCController::setRho(const Eigen::MatrixXd& mat)
{
    int r = mat.rows();
    int c = mat.cols();

    if(r == dimensions && c == dimensions )
    {
        rho = mat;
        return true;
    }
    else
    {
        return false;
    }

}

bool SMCController::setLambda(const Eigen::MatrixXd& mat)
{

    int r = mat.rows();
    int c = mat.cols();

    if(r == dimensions && c == dimensions )
    {
        lambda = mat;
        return true;
    }
    else
    {
        return false;
    }

}

Eigen::MatrixXd SMCController::getLambda()

{
    return lambda;
}

Eigen::MatrixXd SMCController::getRho()
{
    return rho;
}


Eigen::MatrixXd SMCController::validateMat(const Eigen::MatrixXd& mat1, const Eigen::MatrixXd& mat2)
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


double SMCController::sign_func(double x)
{
    if (x > 0)
        return +1.0;
    else if (x == 0)
        return 0.0;
    else
        return -1.0;
}


void SMCController::calculate_torque(const Eigen::VectorXd &e, const Eigen::VectorXd &ed, Eigen::VectorXd &torque)
{
    Eigen::VectorXd sigma =  e + lambda*ed;
    Eigen::VectorXd sigma_sign = sigma.unaryExpr(std::ptr_fun(sign_func));
    torque = -lambda*e - rho*sigma_sign;
}


void SMCController::update(const trajectory_msgs::JointTrajectoryPoint& actual, const trajectory_msgs::JointTrajectoryPoint& desired, std::vector<double>& torque)
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
    
        if (desired.accelerations.size() == dp_size)
        {
            tau_ = tau_ +  VectToEigen(desired.accelerations); 
        }
        
        std::vector<double> my_tau(&tau_[0], tau_.data()+tau_.cols()*tau_.rows());
        tau = my_tau;
        //error = std::vector<double>(&e[0], e.data()+e.cols()*e.rows());
        torque = tau;

    }    

    
}
























