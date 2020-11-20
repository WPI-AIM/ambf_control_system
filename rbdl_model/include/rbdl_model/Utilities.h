#ifndef UTILITIES_H
#define UTILITIES_H
#include <yaml-cpp/yaml.h>

#include <rbdl/rbdl.h>
#include <iostream>
#include "rbdl/rbdl_math.h"
#include <algorithm>
#include<rbdl_model/RBDLModelErrors.h>

using namespace RigidBodyDynamics;
using namespace Math;
using namespace RigidBodyDynamics::Math;

class Utilities
{
public:
    Utilities();

    Vector3d toXYZ(YAML::Node* node);
    Vector3d toRPY(YAML::Node* node);
    Vector3d toXYZInertia(YAML::Node* node);
    Math::Matrix3d vectorToMatrix3d(YAML::Node* node);
    Matrix3_t toRotation(YAML::Node* node);

    std::string trimTrailingSpaces(YAML::Node bodyNode);
    void eraseSubStr(std::string & mainStr, const std::string & toErase);
    void eraseAllSubStr(std::string & mainStr, const std::string & toErase);
    void throwExceptionMessage(const std::string message);
    ~Utilities(void);
};

#endif // UTILITIES_H
