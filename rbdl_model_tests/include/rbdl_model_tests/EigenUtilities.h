#ifndef EigenUtilities_H
#define EigenUtilities_H

#include<iostream>
#include<Eigen/Dense>
#include<Eigen/Eigen>
#include<Eigen/Core>
#include<cmath>
#include <stdlib.h>

using namespace Eigen;
class EigenUtilities
{
public:
    EigenUtilities();
    float get_angle(Vector3f vec_a, Vector3f vec_b, Vector3f up_vector);
    float get_random_between_range(float low, float high);
    Eigen::Matrix3d rotationMatrixFromVectors(Eigen::Vector3d vec1, Eigen::Vector3d vec2);

    template<typename R>
    R rotation_from_euler(float roll, float pitch, float yaw) {
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




    Eigen::Vector3f rpy_from_rotation(Eigen::Matrix3f R);


    template<typename R, typename P, typename T>
    T get_frame(R r, P p){
        T Trans;

        // template disambiguator
        Trans.template setIdentity();

        Trans.template block<3,3>(0,0) = r;
        Trans.template block<3,1>(0,3) = p;

        return Trans;
    }

    ~EigenUtilities(void);
};

#endif // EigenUtilities_H
