#ifndef EigenUtilities_H
#define EigenUtilities_H

#include<iostream>
#include<cmath>
#include <stdlib.h>
#include <tf/LinearMath/Transform.h>
#include <algorithm>
#include <math.h>
#include <string>
#include <algorithm>
#include <functional>

#include "rbdl/rbdl_math.h"
#include "rbdl/Joint.h"
#include "rbdl/Body.h"
#include <random>
#include <limits>
#include <chrono>

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

class EigenUtilities
{
public:
    EigenUtilities() {}
    static float get_random_between_range(float low, float high);
    static const Vector3d TFtoRBDLVector(const tf::Vector3 vec_tf);
    static const Quaternion TFtoRBDLQuaternion(const tf::Quaternion quat_tf);

    // static void CreateRBDLJoint(Vector3d& pa, Vector3d& ca, const Vector3d& pp, const Vector3d& cp, 
    // const double offsetQ, const Vector3d axis, const unsigned int parentId, const Joint joint, 
    // const SpatialTransform world_parentST, const Body &body, const std::string bodyName, 
    // unsigned int& childId, SpatialTransform& world_childST);
  //   static const void CreateRBDLJoint(RBDLModelPtr rbdlModelPtr, Vector3d& pa, Vector3d& ca, const Vector3d& pp, const Vector3d& cp, 
  //   const double offsetQ, 
	// const Vector3d axis, const unsigned int parentId, const Joint joint, const SpatialTransform world_parentST, 
	// const Body &body, const std::string bodyName, unsigned int& childId, SpatialTransform&	world_childST);
    void EraseSubStr(std::string & mainStr, const std::string & toErase);

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
    const Vector3d MatrixToRPY(Matrix3d r);
    const Matrix3d RPYToMatrix(Vector3d v);
    const double RandomNumber(double lowerbound, double upperbound);

    template<typename M>
    const M SetAlmostZeroToZero(M m)
    {
    return m.unaryExpr([](double x) {return (abs(x) < 1e-3) ? 0. : x; });
    }
    ~EigenUtilities(void);
};

#endif // EigenUtilities_H