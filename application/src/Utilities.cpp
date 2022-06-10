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


// TBD: Replease this with engine code
const double Utilities::RandomNumber(double lowerbound, double upperbound)
{
  // if(lowerbound > upperbound) return std::numeric_limits<double>::lowest();

  // // construct a trivial random generator engine from a time-based seed:
  // unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  // std::default_random_engine generator (seed);

  // std::uniform_real_distribution<double> distribution (lowerbound, upperbound);

  // return distribution(generator);
  return -1.0;
}


///
/// \brief toVector3d Inertia
/// \param node
/// \return
///
const Vector3d Utilities::ToXYZInertia(YAML::Node* node)
{
  Vector3d v;
  v(0) = (*node)["ix"].as<double>();
  v(1) = (*node)["iy"].as<double>();
  v(2) = (*node)["iz"].as<double>();
  return v;
}

const int Utilities::ToInt(YAML::Node node) {
  int val;
  if(node.IsDefined()) val = node.as<int>();
  return val;
}

///
/// \brief toVector3d
/// \param node
/// \return
///
const Vector3d Utilities::ToXYZ(YAML::Node* node)
{
  Vector3d v;

  v(0) = (*node)["x"].as<double>();
  v(1) = (*node)["y"].as<double>();
  v(2) = (*node)["z"].as<double>();
  return v;
}

const Math::Matrix3d Utilities::VectorToMatrix3d(YAML::Node* node) {
  Math::Matrix3d m;
  m = Math::Matrix3d::Zero(3,3);

  m(0, 0) = (*node)["ix"].as<double>();
//    m(0, 1) = 0.0;
//    m(0, 2) = 0.0;

//    m(1, 0) = 0.0;
  m(1, 1) = (*node)["iy"].as<double>();
//    m(1, 2) = 0.0;

//    m(2, 0) = 0.0;
//    m(2, 1) = 0.0;
  m(2, 2) = (*node)["iz"].as<double>();

  return m;
}


///
/// \brief toVector3d
/// \param node
/// \return
///
const Vector3d Utilities::ToRPY(YAML::Node* node)
{
  Vector3d v;

  v(0) = (*node)["r"].as<double>();
  v(1) = (*node)["p"].as<double>();
  v(2) = (*node)["y"].as<double>();
  return v;
}

const std::string Utilities::TrimTrailingSpaces(YAML::Node bodyNode) {
  std::string m_name;
  if(bodyNode.IsDefined()){
    m_name = bodyNode.as<std::string>();
    m_name.erase(std::remove(m_name.begin(), m_name.end(), ' '), m_name.end());
  }
  return m_name;
}

// /*
//  * Erase First Occurrence of given  substring from main string.
//  */
// void Utilities::eraseSubStr(std::string & mainStr, const std::string & toErase)
// {
//   // Search for the substring in string
//   size_t pos = mainStr.find(toErase);
//   if (pos != std::string::npos)
//   {
//     // If found then erase it from string
//     mainStr.erase(pos, toErase.length());
//   }
// }

// /*
//  * Erase all Occurrences of given substring from main string.
//  */
// void Utilities::eraseAllSubStr(std::string & mainStr, const std::string & toErase)
// {
//   size_t pos = std::string::npos;
//   // Search for the substring in string in a loop untill nothing is found
//   while ((pos  = mainStr.find(toErase) )!= std::string::npos)
//   {
//     // If found then erase it from string
//     mainStr.erase(pos, toErase.length());
//   }
// }

void Utilities::ThrowInvalidFilePathException(const std::string message)
{
  throw RBDLModel::ModelErrors::RBDLModelInvalidFilePathError("Error: Mention ADF file path not fould. Terminating model creation!\n");
}

void Utilities::ThrowMissingFieldException(const std::string message)
{
  throw RBDLModel::ModelErrors::RBDLModelMissingParameterError("Error: Missing " + message + " which is mandate field to build RBDL Model. Terminating model creation!\n");
}

void Utilities::ThrowInvalidValueException(const std::string message)
{
  throw std::runtime_error("Error: " + message + " . Terminating model creation!\n");
}

void Utilities::ThrowBaseNotFoundException()
{
  throw std::runtime_error("Error: Could not find base for the model. Terminating model creation!\n");
}

Utilities::~Utilities(void) {}
