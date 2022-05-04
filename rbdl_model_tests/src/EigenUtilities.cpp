#include "rbdl_model_tests/EigenUtilities.h"

float EigenUtilities::get_random_between_range(float low, float high) {
  if(high < low) {
    std::cout << "Make sure low is equal or less than high" << std::endl;
    return  std::numeric_limits<float>::min();
  }

  std::srand (time(NULL));
  float seed = ((double)rand()) / ((double)RAND_MAX) * high + low;

  return seed;
}

const Vector3d EigenUtilities::TFtoRBDLVector(const tf::Vector3 vec_tf)
{
  Vector3d vec_rbdl;
  vec_rbdl(0) = vec_tf[0];
  vec_rbdl(1) = vec_tf[1];
  vec_rbdl(2) = vec_tf[2];

  return vec_rbdl;
}

const Quaternion EigenUtilities::TFtoRBDLQuaternion(const tf::Quaternion quat_tf)
{
  Quaternion quat_rbdl;
  quat_rbdl.x() = quat_tf[0];
  quat_rbdl.y() = quat_tf[1];
  quat_rbdl.z() = quat_tf[2];
  quat_rbdl.w() = quat_tf[3];

  return quat_rbdl;
}


const void EigenUtilities::CreateRBDLJoint(RBDLModelPtr rbdlModelPtr, Vector3d& pa, Vector3d& ca, const Vector3d& pp, 
  const Vector3d& cp, const double offsetQ, const Vector3d axis, const unsigned int parentId, const Joint joint, 
  const SpatialTransform world_parentST, const Body &body, const std::string bodyName, unsigned int& childId, 
  SpatialTransform&	world_childST)
{
	pa.normalize();
	ca.normalize();

	Matrix3d bodyRot = Eigen::Matrix3d(Eigen::Quaterniond::FromTwoVectors(pa, ca));
	Eigen::Affine3d rotOffset(Eigen::AngleAxisd(offsetQ, axis));
		
	SpatialTransform bodyST;
	bodyST.E = rotOffset.rotation() * bodyRot;
	bodyST.r = pp - (bodyRot.inverse() * cp);

	world_childST = world_parentST * bodyST;

	Vector3d p_world_child =  world_parentST.E.inverse() * bodyST.r;
	if(parentId == 0) p_world_child = world_parentST.r;

	childId = rbdlModelPtr->AddBody(parentId, Xtrans(p_world_child), joint, body, bodyName);
}

// https://thispointer.com/how-to-remove-substrings-from-a-string-in-c/
/*
 * Erase First Occurrence of given  substring from main string.
 */
void EigenUtilities::EraseSubStr(std::string & mainStr, const std::string & toErase)
{
  // Search for the substring in string
  size_t pos = mainStr.find(toErase);
  if (pos != std::string::npos)
  {
    // If found then erase it from string
    mainStr.erase(pos, toErase.length());
  }
}

const Vector3d EigenUtilities::MatrixToRPY(Matrix3d r)
{
  // Note that r00 and r22 should be non-zero
  if(r(0, 0) == 0.0 || r(2, 2) == 0.0)
  {
    std::cout << "RPY cannot be calculated for the matrix\n";
    return Vector3dZero;
  }

  // roll(x)  : gamma=arctan(r21/r22)
  // pitch(y) : beta=arctan(-r20/sqrt( r21^2+r22^2 ) )
  // yaw(z)   : alpha=arctan(r10/r00)
  double roll  = std::atan2( r(2, 1), r(2, 2) );
  double pitch = std::atan2(-r(2, 0), std::pow(r(2, 1) * r(2, 1) + r(2, 2) * r(2,2), 0.5));
  double yaw   = std::atan2( r(1, 0), r(0, 0));

  return Vector3d(roll, pitch, yaw);
}

const Matrix3d EigenUtilities::RPYToMatrix(Vector3d v)
{
  Eigen::AngleAxisd yawAngle  (v(2), Eigen::Vector3d::UnitZ());
  Eigen::AngleAxisd pitchAngle(v(1), Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd rollAngle (v(0), Eigen::Vector3d::UnitX());

  Eigen::Quaternion<double> q = yawAngle * pitchAngle * rollAngle;
  q.normalize();

  return q.toRotationMatrix(); 
}

const double EigenUtilities::RandomNumber(double lowerbound, double upperbound)
{
  if(lowerbound > upperbound) return std::numeric_limits<double>::lowest();

  // construct a trivial random generator engine from a time-based seed:
  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  std::default_random_engine generator (seed);

  std::uniform_real_distribution<double> distribution (lowerbound, upperbound);

  return distribution(generator);
}



EigenUtilities::~EigenUtilities(void) {}
