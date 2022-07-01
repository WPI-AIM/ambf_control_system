#include "rbdl_model/RBDLUtilities.h"

float RBDLUtilities::get_random_between_range(float low, float high) {
  if(high < low) 
  {
    std::cout << "Make sure low is equal or less than high" << std::endl;
    return  std::numeric_limits<float>::min();
  }

  std::srand (time(NULL));
  float seed = ((double)rand()) / ((double)RAND_MAX) * high + low;

  return seed;
}


// https://thispointer.com/how-to-remove-substrings-from-a-string-in-c/
/*
 * Erase First Occurrence of given  substring from main string.
 */
void RBDLUtilities::EraseSubStr(std::string & mainStr, const std::string & toErase)
{
  // Search for the substring in string
  size_t pos = mainStr.find(toErase);
  if (pos != std::string::npos)
  {
    // If found then erase it from string
    mainStr.erase(pos, toErase.length());
  }
}


bool RBDLUtilities::HasEnding(std::string const &fullString, std::string const &ending)
{
  if (fullString.length() >= ending.length()) {
      return (0 == fullString.compare (fullString.length() - ending.length(), ending.length(), ending));
  } else {
      return false;
  }
}

const Vector3d RBDLUtilities::MatrixToRPY(Matrix3d r)
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

const Matrix3d RBDLUtilities::RPYToMatrix(Vector3d v)
{
  Eigen::AngleAxisd yawAngle  (v(2), Eigen::Vector3d::UnitZ());
  Eigen::AngleAxisd pitchAngle(v(1), Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd rollAngle (v(0), Eigen::Vector3d::UnitX());

  Eigen::Quaternion<double> q = yawAngle * pitchAngle * rollAngle;
  q.normalize();

  return q.toRotationMatrix(); 
}


// TBD: Replease this with engine code
const double RBDLUtilities::RandomNumber(double lowerbound, double upperbound)
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
const Vector3d RBDLUtilities::ToXYZInertia(YAML::Node* node)
{
  Vector3d v;
  v(0) = (*node)["ix"].as<double>();
  v(1) = (*node)["iy"].as<double>();
  v(2) = (*node)["iz"].as<double>();
  return v;
}

const int RBDLUtilities::ToInt(YAML::Node node) {
  int val;
  if(node.IsDefined()) val = node.as<int>();
  return val;
}

const double RBDLUtilities::ToDouble(YAML::Node node)
{
  double val;
  if(node.IsDefined()) val = node.as<double>();
  return val;
}

const bool RBDLUtilities::ToBool(YAML::Node node)
{
  bool val;
  if(node.IsDefined()) val = node.as<bool>();
  return val;
}

///
/// \brief toVector3d
/// \param node
/// \return
///
const Vector3d RBDLUtilities::ToXYZ(YAML::Node* node)
{
  Vector3d v;

  v(0) = (*node)["x"].as<double>();
  v(1) = (*node)["y"].as<double>();
  v(2) = (*node)["z"].as<double>();
  return v;
}

const Math::Matrix3d RBDLUtilities::VectorToMatrix3d(YAML::Node* node) {
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
const Vector3d RBDLUtilities::ToRPY(YAML::Node* node)
{
  Vector3d v;

  v(0) = (*node)["r"].as<double>();
  v(1) = (*node)["p"].as<double>();
  v(2) = (*node)["y"].as<double>();
  return v;
}

const std::string RBDLUtilities::TrimTrailingSpaces(YAML::Node bodyNode) {
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

void RBDLUtilities::Round(Matrix3d& inout, double threshold)
{
  inout = (threshold < inout.array().abs()).select(inout, 0.0f);
}

void RBDLUtilities::RoundQ(double &q)
{
  if(q < -M_PI || q > M_PI) return;

  if(q > -M_PI && q < -M_PI_2)
    q = (q <= -3.0 * M_PI_4) ? -M_PI : -M_PI_2;
  else if(q > -M_PI_2 && q < 0)
    q = (q <= -M_PI_4) ? -M_PI_2 : 0;
  else if(q > 0 && q < M_PI_2)
    q = (q <= M_PI_4) ? 0 : M_PI_2;
  else if(q > M_PI_2 && q < M_PI)
    q = (q < 3.0 * M_PI_4) ? M_PI_2 : M_PI;
}

void RBDLUtilities::ThrowInvalidFilePathException(const std::string message)
{
  throw RBDLModel::ModelErrors::RBDLModelInvalidFilePathError(
    "Error: Mention ADF file path not fould. Terminating model creation!\n");
}

void RBDLUtilities::ThrowInvalidNamespaceException()
{
  throw std::runtime_error(
    "Invalid namespace format in ADF. Valid format '/ambf/env/<modelName>/'");
}

void RBDLUtilities::ThrowMissingFieldException(const std::string message)
{
  throw RBDLModel::ModelErrors::RBDLModelMissingParameterError(
    "Error: Missing " + message + " which is mandate field to build RBDL Model. Terminating model creation!\n");
}

void RBDLUtilities::ThrowInvalidValueException(const std::string message)
{
  throw std::runtime_error("Error: " + message + " . Terminating model creation!\n");
}

void RBDLUtilities::ThrowBaseNotFoundException()
{
  throw std::runtime_error("Error: Could not find base for the model. Terminating model creation!\n");
}

void RBDLUtilities::ThrowKeyNotFoundException(const std::string mapName, const std::string key)
{
  throw std::runtime_error(
    "Error: Key: " + key + " not found exception in map " + mapName + ". Terminating execution\n");
}

void RBDLUtilities::ThrowDisabledForROS(const std::string message)
{
  throw std::runtime_error(
    "Error: " + message + " is disabled for ROS which is needed for RBDL model creation. Terminating execution\n");
}

void RBDLUtilities::ThrowUnsupportedJointException(const std::string jointName, const std::string jointType)
{
  throw std::runtime_error(
    "Error: Only Revoulte and Prismatic joints are supported. " + jointName + 
    " has joint Type " + jointType + " which is not supported. Terminating execution\n");
}

RBDLUtilities::~RBDLUtilities(void) {}
