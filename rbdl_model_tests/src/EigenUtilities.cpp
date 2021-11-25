#include "rbdl_model_tests/EigenUtilities.h"

// EigenUtilities::EigenUtilities() {}

float EigenUtilities::get_angle(Vector3f vec_a, Vector3f vec_b, Vector3f up_vector) {
    float angle = 0.0;
    vec_a.normalize();
    vec_b.normalize();

    Vector3f cross_ab = vec_a.cross(vec_b);
    float vdot = vec_a.dot(vec_b);

//    # Check if the vectors are in the same direction
    if(1.0 - vdot < 0.000001) angle = 0.0;
    else if (1.0 + vdot < 0.000001) angle = M_PI;
    else angle = acos(vdot);

    if(up_vector(0) != std::numeric_limits<float>::min() &&
       up_vector(1) != std::numeric_limits<float>::min() &&
       up_vector(2) != std::numeric_limits<float>::min()) {
        float same_dir = cross_ab.dot(up_vector);

        if(same_dir < 0.0) angle = -angle;
    }

    return angle;
}

float EigenUtilities::get_random_between_range(float low, float high) {
    if(high < low) {
        std::cout << "Make sure low is equal or less than high" << std::endl;
        return  std::numeric_limits<float>::min();
    }

    std::srand (time(NULL));
    float seed = ((double)rand()) / ((double)RAND_MAX) * high + low;

    return seed;
}

Eigen::Matrix3d EigenUtilities::rotationMatrixFromVectors(Eigen::Vector3d vec1, Eigen::Vector3d vec2)
{
    Eigen::Matrix3d m;
    m = Eigen::Matrix3d::Zero(3,3);

    Vector3d a = vec1 / vec1.norm();
    Vector3d b = vec2 / vec2.norm();

    Vector3d v = a.cross(b);
    double c = a.dot(b);
    double s = v.norm();

    Eigen::Matrix3d kmat;
    kmat = Eigen::Matrix3d::Zero(3,3);

    kmat(0, 1) = -v[2];
    kmat(0, 2) = -v[1];

    kmat(1, 0) = v[2];
    kmat(1, 2) = -v[0];

    kmat(2, 0) = -v[1];
    kmat(2, 1) = v[0];

    Eigen::Matrix3d eye;
    eye = Eigen::Matrix3d::Identity(3, 3);

    m = eye + kmat + (kmat * kmat) * ((1 - c) / std::pow(s, 2));

    if(m.hasNaN()) m = eye;

    return m;
}

Eigen::Matrix3d RodriguesRotationFormula(Eigen::Vector3d vec1, Eigen::Vector3d vec2)
{
    vec1.normalize();
    vec2.normalize();
    
    Eigen::Matrix3d out;
    out.setIdentity();
    Eigen::Vector3d vCross = vec1.cross(vec2);
    double vDot = vec1.dot(vec2);

}

Eigen::Matrix3d EigenUtilities::rotX(float theta)
{
    Eigen::Matrix3d R;
    R.setZero();
    R(0, 0) = 1;

    R(1, 1) =  std::cos(theta);
    R(1, 2) = -std::sin(theta);
   
    R(2, 1) =  std::sin(theta);
    R(2, 2) =  std::cos(theta);

    return R;
}

Eigen::Matrix3d EigenUtilities::rotY(float theta)
{
    Eigen::Matrix3d R;
    R.setZero();
    R(0, 0) =  std::cos(theta);
    R(0, 2) = -std::sin(theta);

    R(1, 1) = 1;

    R(2, 1) =  std::sin(theta);
    R(2, 2) =  std::cos(theta);

    return R;
}

Eigen::Matrix3d EigenUtilities::rotZ(float theta)
{
    Eigen::Matrix3d R;
    R.setZero();
    R(0, 0) =  std::cos(theta);
    R(0, 1) =  std::sin(theta);
    
    R(1, 0) = -std::sin(theta);
    R(1, 1) =  std::cos(theta);

    R(2, 2) = 1;

    return R;
}

Eigen::Vector3f EigenUtilities::rpy_from_rotation(Eigen::Matrix3f R) {
    Eigen::Vector3f rpy;
    rpy[0] = std::atan2(R(2, 1), R(2, 2));
    rpy[1] = std::atan2(-R(2, 0), std::pow( R(2, 1)*R(2, 1) +R(2, 2)*R(2, 2), 0.5));
    rpy[2] = std::atan2( R(1, 0), R(0, 0));

    return rpy;
}

EigenUtilities::~EigenUtilities(void) {}
