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

const Vector3d EigenUtilities::TFtoEigenVector(const tf::Vector3 vec_tf)
{
    Vector3d vec_e;
    vec_e(0) = vec_tf[0];
    vec_e(1) = vec_tf[1];
    vec_e(2) = vec_tf[2];

    return vec_e;
}

const Quaternion EigenUtilities::TFtoEigenQuaternion(const tf::Quaternion quat_tf)
{
    Quaternion quat_e;
    quat_e.x() = quat_tf[0];
    quat_e.y() = quat_tf[1];
    quat_e.z() = quat_tf[2];
    quat_e.w() = quat_tf[3];

    return quat_e;
}

EigenUtilities::~EigenUtilities(void) {}
