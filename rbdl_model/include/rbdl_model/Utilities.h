#ifndef UTILITIES_H
#define UTILITIES_H
#include <yaml-cpp/yaml.h>

#include <rbdl/rbdl.h>
#include <iostream>
#include "rbdl/rbdl_math.h"
#include <algorithm>
#include<rbdl_model/RBDLModelErrors.h>
#include <string>
#include <boost/optional.hpp>
#include <boost/bimap.hpp>
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
  //const int toInt(YAML::Node* node, const std::string param);
  int toInt(YAML::Node bodyNode);
  Math::Matrix3d vectorToMatrix3d(YAML::Node* node);
  Math::Matrix3d rotationMatrixFromVectors(Vector3d vec1, Vector3d vec2);

  std::string trimTrailingSpaces(YAML::Node bodyNode);
  void eraseSubStr(std::string & mainStr, const std::string & toErase);
  void eraseAllSubStr(std::string & mainStr, const std::string & toErase);

  void throwInvalidFilePathException(const std::string message);
  void throwMissingFieldException(const std::string message);
  void throwInvalidValueException(const std::string message);
  void throwBaseNotFoundException();

  ~Utilities(void);
};

#endif // UTILITIES_H