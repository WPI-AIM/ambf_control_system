#ifndef CONTROLLERBASE_H
#define CONTROLLERBASE_H
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <Eigen/Core>
#include <ros/ros.h>
#include "std_msgs/Float64MultiArray.h"

class ControllerBase
{
   public:
    ControllerBase(){}
    ~ControllerBase(){}

    virtual void update(const trajectory_msgs::JointTrajectoryPoint&, const trajectory_msgs::JointTrajectoryPoint&, std::vector<double>&)=0;
    virtual std::vector<double> get_error(){return error;}
    virtual std::vector<double> get_tau(){return tau;}  


   protected:

      std::vector<double> error;
      std::vector<double> tau;
     
      template<typename T, typename A>
      Eigen::VectorXd VectToEigen(std::vector<T,A> const& msg )   
      {
         std::vector<double> vec(msg.begin(), msg.end());
         Eigen::VectorXd Q =  Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(vec.data(), vec.size());
         return Q;
      }

      /**
      *   Function for converting a Eigen VectorXd to a std::vector<double>
      *   @param e Eigen vector to parse and copy to the std::vector
      *   @return the std::vector<double> of e
      **/
      std::vector<double> eigenToVect(Eigen::VectorXd e) {

          std::vector<double> vec(&e[0], e.data() + e.cols()*e.rows());
          return vec;
      }

      /**
      *   Function for converting a std::vector<double> to an Eigen VectorXd
      *   @param msg std::vector<double> to map into an Eigen vector
      *   @return the Eigen vector of msg
      **/
      Eigen::VectorXd vectToEigen(const std::vector<double> &msg) {
          std::vector<double> vec(msg.begin(), msg.end());
          Eigen::VectorXd Q =  Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(vec.data(), vec.size());
          return Q;
      }

      /**
      *   Function for converting a std_msgs::Float64MultiArray to an Eigen MatrixXd
      *   ! Only tested on the 2-Dimensional (6xN) Jacobian matrix returned by RBDL server (m.layout.dim.size() = 2)
      *   @param m Float64Multiarray message to parse and copy to the Eigen matrix m
      *   @param e Eigen matrix to populate with data from m, should be initialized to the appropriate size
      **/
      void msgToEigenMat(std_msgs::Float64MultiArray &m, Eigen::MatrixXd &e) {

          if(m.layout.dim.size() != 2 || (int)m.data.size() != e.size()){
              return;
          }

          // TODO check or resize matrix row and column dimensions
          int matRows = m.layout.dim[0].size;
          int matCols = m.layout.dim[1].size;

          int ii = 0;

          for(int i = 0; i < matRows; i++){
              for(int j = 0; j < matCols; j++){
                  e(i, j) = m.data[ii++];
              }
          }

      }



};


#endif // CONTROLLERBASE_H
