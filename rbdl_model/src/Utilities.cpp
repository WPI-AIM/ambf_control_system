#include "rbdl_model/Utilities.h"

Utilities::Utilities()
{

}


///
/// \brief toVector3d Inertia
/// \param node
/// \return
///
Vector3d Utilities::toXYZInertia(YAML::Node* node){
    Vector3d v;
    v(0) = (*node)["ix"].as<double>();
    v(1) = (*node)["iy"].as<double>();
    v(2) = (*node)["iz"].as<double>();
    return v;
}



///
/// \brief toVector3d
/// \param node
/// \return
///
Vector3d Utilities::toXYZ(YAML::Node* node){
    Vector3d v;

    v(0) = (*node)["x"].as<double>();
    v(1) = (*node)["y"].as<double>();
    v(2) = (*node)["z"].as<double>();
    return v;
}

Math::Matrix3d Utilities::vectorToMatrix3d(YAML::Node* node) {
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


Math::Matrix3d Utilities::rotationMatrixFromVectors(Vector3d vec1, Vector3d vec2) {
    Math::Matrix3d m;
    m = Math::Matrix3d::Zero(3,3);

    Vector3d a = vec1 / vec1.norm();
    Vector3d b = vec2 / vec2.norm();

    Vector3d v = a.cross(b);
    double c = a.dot(b);
    double s = v.norm();

    Math::Matrix3d kmat;
    kmat = Math::Matrix3d::Zero(3,3);

    kmat(0, 1) = -v[2];
    kmat(0, 2) = -v[1];

    kmat(1, 0) = v[2];
    kmat(1, 2) = -v[0];

    kmat(2, 0) = -v[1];
    kmat(2, 1) = v[0];

    Math::Matrix3d eye;
    eye = Math::Matrix3d::Identity(3, 3);

    m = eye + kmat + (kmat * kmat) * ((1 - c) / std::pow(s, 2));

    if(m.hasNaN()) m = eye;

    return m;
}


///
/// \brief toVector3d
/// \param node
/// \return
///
Vector3d Utilities::toRPY(YAML::Node* node){
    Vector3d v;

    v(0) = (*node)["r"].as<double>();
    v(1) = (*node)["p"].as<double>();
    v(2) = (*node)["y"].as<double>();
    return v;
}

std::string Utilities::trimTrailingSpaces(YAML::Node bodyNode) {
    std::string m_name;
    if(bodyNode.IsDefined()){
        m_name = bodyNode.as<std::string>();
        m_name.erase(std::remove(m_name.begin(), m_name.end(), ' '), m_name.end());
    }
    return m_name;
}

/*
 * Erase First Occurrence of given  substring from main string.
 */
void Utilities::eraseSubStr(std::string & mainStr, const std::string & toErase)
{
    // Search for the substring in string
    size_t pos = mainStr.find(toErase);
    if (pos != std::string::npos)
    {
        // If found then erase it from string
        mainStr.erase(pos, toErase.length());
    }
}

/*
 * Erase all Occurrences of given substring from main string.
 */
void Utilities::eraseAllSubStr(std::string & mainStr, const std::string & toErase)
{
    size_t pos = std::string::npos;
    // Search for the substring in string in a loop untill nothing is found
    while ((pos  = mainStr.find(toErase) )!= std::string::npos)
    {
        // If found then erase it from string
        mainStr.erase(pos, toErase.length());
    }
}


void Utilities::throwExceptionMessage(const std::string message) {
    throw RBDLModel::ModelErrors::RBDLModelMissingParameterError("Error: Missing " + message + " which is mandate field to build RBDL Model. Terminating model creation!\n");
}

Utilities::~Utilities(void) {

}
