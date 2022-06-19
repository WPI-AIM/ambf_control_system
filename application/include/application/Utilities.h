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
#include <yaml-cpp/yaml.h>
#include <random>
#include <limits>

#include "rbdl/rbdl_math.h"
#include "rbdl/Joint.h"
#include "rbdl/Body.h"
#include "application/RBDLModelErrors.h"

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

class Utilities
{
public:
  Utilities() {}
  static float get_random_between_range(float low, float high);
  static const Vector3d TFtoRBDLVector(const tf::Vector3 vec_tf);
  static const Quaternion TFtoRBDLQuaternion(const tf::Quaternion quat_tf);

  static void EraseSubStr(std::string & mainStr, const std::string & toErase);
  static bool HasEnding(std::string const &fullString, std::string const &ending);
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
  static const Vector3d MatrixToRPY(Matrix3d r);
  static const Matrix3d RPYToMatrix(Vector3d v);
  const double RandomNumber(double lowerbound, double upperbound);

  template<typename M>
  const M SetAlmostZeroToZero(M m)
  {
    return m.unaryExpr([](double x) {return (abs(x) < 1e-3) ? 0. : x; });
  }

  const Matrix3d QuaternionTFtoRBDL(const tf::Quaternion qtf);

  static const Vector3d ToXYZ(YAML::Node* node);
  static const Vector3d ToRPY(YAML::Node* node);
  static const Vector3d ToXYZInertia(YAML::Node* node);
  //const int toInt(YAML::Node* node, const std::string param);
  static const int ToInt(YAML::Node bodyNode);
  static const double ToDouble(YAML::Node node);
  static const bool ToBool(YAML::Node node);
  static const Math::Matrix3d VectorToMatrix3d(YAML::Node* node);

  static const std::string TrimTrailingSpaces(YAML::Node bodyNode);

  static void ThrowInvalidFilePathException(const std::string message);
  static void ThrowInvalidNamespaceException();
  static void ThrowMissingFieldException(const std::string message);
  static void ThrowInvalidValueException(const std::string m);
  static void ThrowAMBFInactiveException();
  static void ThrowBaseNotFoundException();
  static void ThrowKeyNotFoundException(const std::string mapName, const std::string key);
  static void ThrowDisabledForROS(const std::string message);
  static void ThrowUnsupportedJointException(const std::string jointName, const std::string jointType);
  ~Utilities(void);
};

#endif // EigenUtilities_H
