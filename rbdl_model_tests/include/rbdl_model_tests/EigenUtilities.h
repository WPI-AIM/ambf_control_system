#ifndef EigenUtilities_H
#define EigenUtilities_H

#include<iostream>
#include<Eigen/Dense>
#include<Eigen/Eigen>
#include<Eigen/Core>
#include<cmath>
#include <stdlib.h>
#include <tf/LinearMath/Transform.h>
#include <algorithm>

using namespace Eigen;
// TODO: Use Quaternion instead of Matrix
class EigenUtilities
{
public:
    EigenUtilities() {}
    static float get_angle(Vector3f vec_a, Vector3f vec_b, Vector3f up_vector);
    static float get_random_between_range(float low, float high);
    static Eigen::Matrix3d rotationMatrixFromVectors(Eigen::Vector3d vec1, Eigen::Vector3d vec2);
    static Eigen::Matrix3d RodriguesRotationFormula(Eigen::Vector3d vec1, Eigen::Vector3d vec2);

    static Eigen::Matrix3d rotX(float theta);
    static Eigen::Matrix3d rotY(float theta);
    static Eigen::Matrix3d rotZ(float theta);

    template<typename R>
    static R rotation_from_euler(float roll, float pitch, float yaw) {
        // roll and pitch and yaw in radians
        float su = std::sin(roll);
        float cu = std::cos(roll);
        float sv = std::sin(pitch);
        float cv = std::cos(pitch);
        float sw = std::sin(yaw);
        float cw = std::cos(yaw);

        R r(3, 3);
        r(0, 0) = cv*cw;
        r(0, 1) = su*sv*cw - cu*sw;
        r(0, 2) = su*sw + cu*sv*cw;
        r(1, 0) = cv*sw;
        r(1, 1) = cu*cw + su*sv*sw;
        r(1, 2) = cu*sv*sw - su*cw;
        r(2, 0) = -sv;
        r(2, 1) = su*cv;
        r(2, 2) = cu*cv;

        return r;
    }

    static Eigen::Vector3f rpy_from_rotation(Eigen::Matrix3f R);

    template<typename R, typename P, typename T>
    static T get_frame(R r, P p){
        T Trans;

        // template disambiguator
        Trans.template setIdentity();

        Trans.template block<3,3>(0,0) = r;
        Trans.template block<3,1>(0,3) = p;

        return Trans;
    }

    static float MatrixRotationAngle(Eigen::Matrix3d R);
    static void MatrixAxisOfRotation(Eigen::Matrix3d R);
    static const Eigen::Vector3d tfToEigenVector(const tf::Vector3 vec_tf);
    static const Eigen::Quaterniond tfToEigenQuaternion(const tf::Quaternion quat_tf);

    // https://www.techiedelight.com/check-vector-contains-given-element-cpp/
    struct compare
    {
        std::string key;
        compare(std::string const &str): key(str){}

        bool operator()(std::string const &str)
        {
            return (str == key);
        }
    };

    /** 
     * https://stackoverflow.com/questions/15482498/how-to-resize-a-vector-in-eigen3
     * To be upgraded to c++ 20 to support double template. 
     * https://stackoverflow.com/questions/2183087/why-cant-i-use-float-value-as-a-template-parameter
     **/
    template <bool COND, int A, int B>
    struct IF
    {
    enum { val = A };
    };

    template <int A, int B>
    struct IF<false, A, B>
    {
    enum { val = B };
    };

    template <int A, int B>
    struct MIN : IF<A < B, A, B>
    {
    };

    template <typename T,int dim,int newDim>
    static Eigen::Matrix<T,newDim,1> to(Eigen::Matrix<T,dim,1> p)
    {
    Eigen::Matrix<int,newDim,1> newp =
        // Eigen::Matrix<T,newDim,1>::Zero();
        Eigen::Matrix<T,newDim,1>::Ones();;
    newp.template head< MIN<dim,newDim>::val >() =
        p.template head< MIN<dim,newDim>::val >();

    return newp;
    }

    // Eigen::Vector2i p_2i(1,2);
    // Eigen::Vector3i p_3i(3,4,5);

    // std::cout << EigenUtilities::to<int, 2, 3>(p_2i) << std::endl << std::endl;
    // std::cout << EigenUtilities::to<int, 3, 2>(p_3i) << std::endl << std::endl;


    ~EigenUtilities(void);
};

#endif // EigenUtilities_H
