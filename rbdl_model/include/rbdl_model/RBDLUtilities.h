#ifndef EigenUtilities_H
#define EigenUtilities_H
#include<iostream>
#include<cmath>
#include <stdlib.h>
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
#include "rbdl_model/RBDLModelErrors.h"
#include "rbdl/rbdl_math.h"
#include "rbdl/Joint.h"

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

class RBDLUtilities
{
public:
  RBDLUtilities() {}
  static float get_random_between_range(float low, float high);

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


  static const Vector3d ToXYZ(YAML::Node* node);
  static const Vector3d ToRPY(YAML::Node* node);
  static const Vector3d ToXYZInertia(YAML::Node* node);
  //const int toInt(YAML::Node* node, const std::string param);
  static const int ToInt(YAML::Node bodyNode);
  static const double ToDouble(YAML::Node node);
  static const bool ToBool(YAML::Node node);
  static const Math::Matrix3d VectorToMatrix3d(YAML::Node* node);

  static const std::string TrimTrailingSpaces(YAML::Node bodyNode);
  static void Round(Matrix3d& inout, double threshold);
  
  template <typename T>
  static void Round(T& inout)
  {
    int rows = inout.rows();
    int cols = inout.cols();

    for(int col = 0; col < cols; col++)
    {
      for(int row = 0; row < rows; row++)
      {
        double val = inout(row, col);
        if(val < -1.5 || val > 1.5) return;

        if(val < -0.5) inout(row, col) = -1.0;
        else if(val >= -0.5 && val <  0.0) inout(row, col) = 0;
        else if(val >   0.0 && val <= 0.5) inout(row, col) = 0;
        else if(val >   0.5) inout(row, col) = 1.0;
      }    
    }
  }

  // static void RoundVector(Vector3d& inout);
  static void RoundQ(double &q);

  static void ThrowInvalidFilePathException(const std::string message);
  static void ThrowInvalidNamespaceException();
  static void ThrowMissingFieldException(const std::string message);
  static void ThrowInvalidValueException(const std::string m);
  
  static void ThrowKeyNotFoundException(const std::string mapName, const std::string key);
  static void ThrowBaseNotFoundException();
  static void ThrowDisabledForROS(const std::string message);
  static void ThrowUnsupportedJointException(const std::string jointName, const std::string jointType);
  ~RBDLUtilities(void);
};

#endif // RBDLUtilities_H
