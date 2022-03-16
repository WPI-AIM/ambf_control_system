#ifndef EigenUtilities_H
#define EigenUtilities_H

#include<iostream>
#include<cmath>
#include <stdlib.h>
#include <tf/LinearMath/Transform.h>
#include <algorithm>
#include <math.h>
#include "rbdl/rbdl_math.h"
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

class EigenUtilities
{
public:
    EigenUtilities() {}
    static float get_random_between_range(float low, float high);
    static const Vector3d TFtoEigenVector(const tf::Vector3 vec_tf);
    static const Quaternion TFtoEigenQuaternion(const tf::Quaternion quat_tf);

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


    ~EigenUtilities(void);
};

#endif // EigenUtilities_H
