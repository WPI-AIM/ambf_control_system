#include "application/Utilities.h"

float Utilities::get_random_between_range(float low, float high) {
  if(high < low) 
  {
    std::cout << "Make sure low is equal or less than high" << std::endl;
    return  std::numeric_limits<float>::min();
  }

  std::srand (time(NULL));
  float seed = ((double)rand()) / ((double)RAND_MAX) * high + low;

  return seed;
}

const Vector3d Utilities::TFtoRBDLVector(const tf::Vector3 vec_tf)
{
  Vector3d vec_rbdl;
  vec_rbdl(0) = vec_tf[0];
  vec_rbdl(1) = vec_tf[1];
  vec_rbdl(2) = vec_tf[2];

  return vec_rbdl;
}

const Quaternion Utilities::TFtoRBDLQuaternion(const tf::Quaternion quat_tf)
{
  Quaternion quat_rbdl;
  quat_rbdl.x() = quat_tf[0];
  quat_rbdl.y() = quat_tf[1];
  quat_rbdl.z() = quat_tf[2];
  quat_rbdl.w() = quat_tf[3];

  return quat_rbdl;
}


// https://thispointer.com/how-to-remove-substrings-from-a-string-in-c/
/*
 * Erase First Occurrence of given  substring from main string.
 */
void Utilities::EraseSubStr(std::string & mainStr, const std::string & toErase)
{
  // Search for the substring in string
  size_t pos = mainStr.find(toErase);
  if (pos != std::string::npos)
  {
    // If found then erase it from string
    mainStr.erase(pos, toErase.length());
  }
}


bool Utilities::HasEnding(std::string const &fullString, std::string const &ending)
{
  if (fullString.length() >= ending.length()) {
      return (0 == fullString.compare (fullString.length() - ending.length(), ending.length(), ending));
  } else {
      return false;
  }
}

const Vector3d Utilities::MatrixToRPY(Matrix3d r)
{
  // Note that r00 and r22 should be non-zero
  if(r(0, 0) == 0.0 || r(2, 2) == 0.0)
  {
    std::cout << "RPY cannot be calculated for the matrix\n";
    return Vector3dZero;
  }

  double roll  = std::atan2( r(2, 1), r(2, 2) );
  double pitch = std::atan2(-r(2, 0), std::pow(r(2, 1) * r(2, 1) + r(2, 2) * r(2,2), 0.5));
  double yaw   = std::atan2( r(1, 0), r(0, 0));

  return Vector3d(roll, pitch, yaw);
}

const Matrix3d Utilities::RPYToMatrix(Vector3d v)
{
  Eigen::AngleAxisd yawAngle  (v(2), Eigen::Vector3d::UnitZ());
  Eigen::AngleAxisd pitchAngle(v(1), Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd rollAngle (v(0), Eigen::Vector3d::UnitX());

  Eigen::Quaternion<double> q = yawAngle * pitchAngle * rollAngle;
  q.normalize();

  return q.toRotationMatrix(); 
}

void Utilities::ThrowAMBFInactiveException()
{
  throw std::runtime_error("Error: AMBF Inactive Exception. Terminating model creation!\n");
}

void Utilities::ThrowKeyNotFoundException(const std::string mapName, const std::string key)
{
  throw std::runtime_error(
    "Error: Key: " + key + " not found exception in map " + mapName + ". Terminating execution\n");
}

Utilities::~Utilities(void) {}
