#include "rbdl_model_tests/EigenUtilities.h"



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

Eigen::Matrix3d EigenUtilities::rotationMatrixFromVectors(Eigen::Vector3d vec1, 
                                                                Eigen::Vector3d vec2)
{
    Eigen::Matrix3d m;
    m = Eigen::Matrix3d::Zero(3,3);

    Eigen::Vector3d a = vec1 / vec1.norm();
    Eigen::Vector3d b = vec2 / vec2.norm();

    Eigen::Vector3d v = a.cross(b);
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

// By Cameron Buie's answer trace is equals to 1+2cos(θ) where θ is the angle of rotation. 
// θ can then be determined up to sign which will depend on the orientation of the 
// axis of rotation chosen.
float EigenUtilities::MatrixRotationAngle(Eigen::Matrix3d R)
{
    // TODO: Check if this is rotation matrix
    float rTrace = R.trace();
    float rAngleOfRoation = std::acos((rTrace - 1.0) / 2.0);

    return rAngleOfRoation;
}

// S=.5(R−RT)
// Then if S=(aij), the rotation axis with magnitude sinθ is (a21,a02,a10).
void EigenUtilities::MatrixAxisOfRotation(Eigen::Matrix3d R)
{
    const Eigen::Matrix3d S = 0.5 * (R - R.transpose());
    std::cout << "EigenUtilities - S" << std::endl << S << std::endl;
}

const Eigen::Vector3d EigenUtilities::tfToEigenVector(const tf::Vector3 vec_tf)
{
    Eigen::Vector3d vec_e;
    vec_e(0) = vec_tf[0];
    vec_e(1) = vec_tf[1];
    vec_e(2) = vec_tf[2];

    return vec_e;
}

const Eigen::Quaterniond EigenUtilities::tfToEigenQuaternion(const tf::Quaternion quat_tf)
{
    // Eigen::Quaternion quat_e;
    Eigen::Quaterniond quat_e;
    quat_e.x() = quat_tf[0];
    quat_e.y() = quat_tf[1];
    quat_e.z() = quat_tf[2];
    quat_e.w() = quat_tf[3];

    return quat_e;
}

// https://rip94550.wordpress.com/2008/05/20/axis-and-angle-of-rotation/
RigidBodyDynamics::Joint EigenUtilities::RotationMatrixToRBDLJoint(const Eigen::Matrix3d R, 
                                                        RigidBodyDynamics::JointType jointType)
{
    
    EigenSolver<Eigen::Matrix3d> es(R);
    double angle = acos((R.trace() - 1.0) * 0.5);
    RigidBodyDynamics::Math::Vector3d jointAxis;
    if(angle == 0.0)
    {
        // RBDL has Z axis as default.
        jointAxis = { 0.0, 0.0, 1.0 };
        // return RigidBodyDynamics::Joint(jointType, jointAxis);
        return RigidBodyDynamics::Joint(SpatialVector(0., 0., 1., 0., 0., 0.));
    }

    const Eigen::Vector3d eigenvalues = es.eigenvalues().real();
    const Eigen::Matrix3d eigenvectors = es.eigenvectors().real();

    // evMaxCoeff = 
    // VectorXf maxVal = eigenvalues.rowwise().maxCoeff();
    // const Eigen::Vector3d maxEigenVector = 
    

    std::cout << "angle: " << angle << std::endl;
    std::cout << "eigenvalues" << std::endl << eigenvalues << std::endl;
    std::cout << "eigenvectors" << std::endl << eigenvectors << std::endl;
    
    // MatrixXf::Index   maxIndex[3];
    // VectorXf maxVal(3);
    // for(int i=0;i<3;++i)
    //     maxVal(i) = eigenvectors.row(i).maxCoeff( &maxIndex[i] );

    int maxEigenValueIndex;
    double maxEigenValue;
    maxEigenValue = eigenvalues.col(0).maxCoeff(&maxEigenValueIndex);

    Eigen::Vector3d dominantEigenVector = eigenvectors.block<3, 1>(0, maxEigenValueIndex);
    dominantEigenVector.normalize();

    std::cout << "maxEigenValue" << std::endl << maxEigenValue << std::endl;
    std::cout << "maxEigenValueIndex" << std::endl << maxEigenValueIndex << std::endl;
    std::cout << "dominantEigenVector" << std::endl << dominantEigenVector << std::endl;

    // clockwise and anti-clockwise
    const std::vector<double> rotationDirections = 
                                { angle, -angle, std::numeric_limits<double>::min() };

    /***orientation
     * For a non-zero rotation first find the axis of rotation with eigen value.
     * The direction of rotation can be clockwise(cw) or counter clockwise(ccw).
     * To find this lets reconstruct the rotation matrix using Eigen vector 
     * corresponding to eigen value 1. We will have two Rotation matricies corresponding
     * to cw and ccw rotation. The calculated rotation matrix which matches the orgional 
     * rotaion matrix give the direction of rotation.
     * 
     * R = I + sin(theta) * N + (1 - cos(theta)) * N^2
     *  N = [
	 *        0,  c, -b,
	 *       -c,  0,  a,
	 *        b, -a,  0
     *      ]
     * where N is roation of base frame w.r.t body frame. 
     * We would need -N to find the rotation of body frame w.r.t base frame.
     ***/
    double a = dominantEigenVector(0);
    double b = dominantEigenVector(1);
    double c = dominantEigenVector(2);

    std::cout << a << ", " << b << ", " << c << std::endl;

    Eigen::Matrix3d N;
    N.setZero();
    // Rotation of base frame w.r.t body frame
    N << 
        0.0,   c,  -b, 
         -c, 0.0,   a, 
          b,  -a, 0.0;

    // std::cout << "N " << std::endl << N << std::endl;
    // Roatation of body frame w.r.t base frame
    N = -N;

    for( double theta : rotationDirections )
    {
        if(theta == std::numeric_limits<double>::min())
        {
            std::cout << "Rotation matrix could not be reconstruced. Invalid joint orientation."
                    << std::endl;
            return RigidBodyDynamics::JointTypeUndefined;
        }

        const Eigen::Matrix3d R_cal = 
            Eigen::Matrix3d::Identity() + sin(theta) * N + (1 - cos(theta)) * N * N;

        std::cout << "direction: " << theta << std::endl 
                  << "R_cal" << std::endl << R_cal << std::endl;
        if(R_cal.isApprox(R, 1e-5f))
        {// J = EigenUtilities::RotationMatrixToRBDLJoint(R, RigidBodyDynamics::JointType::JointTypeRevolute);
            // Found the rotation direction
            angle = theta;

            std::cout << "found match: " << angle << std::endl;

            break;
        }
        std::cout << "----------------------" << std::endl;
    }

    Math::Vector3d joint_axis = Math::Vector3d::Zero();
    joint_axis(maxEigenValueIndex) = 1.0;
    
    
    

    return RigidBodyDynamics::JointTypeRevoluteX;
    // return RigidBodyDynamics::Joint(jointType, joint_axis);
}

EigenUtilities::~EigenUtilities(void) {}
