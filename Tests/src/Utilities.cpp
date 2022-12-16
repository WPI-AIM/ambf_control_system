#include "Tests/Utilities.h"

Quaternion Utilities::EigenToRBDLQuaternion(const Eigen::Quaterniond &quat_eigen)
{
  Quaternion quat_rbdl;
  quat_rbdl.x() = quat_eigen.x();
  quat_rbdl.y() = quat_eigen.y();
  quat_rbdl.z() = quat_eigen.z();
  quat_rbdl.w() = quat_eigen.w();

  return quat_rbdl;
}


Frames Utilities::Q_B_Frames(const Vector3d& v_p_c_ax_jINp, const Vector3d& v_p_c_ap, const Vector3d& v_p_c_ac,
    const double& q_p_c_jOffset, const double& q_p_c_cOffset)
{
  Eigen::Quaternion Q_B_p_c_jINp =
    Eigen::Quaterniond::FromTwoVectors(v_p_c_ax_jINp, v_p_c_ap);
  Eigen::Quaternion Q_B_p_c_jOffINp =
    Eigen::Quaterniond(Eigen::AngleAxisd(q_p_c_jOffset, v_p_c_ap));

  Eigen::Quaternion Q_B_p_c_cINp =
    Eigen::Quaterniond(Eigen::Quaterniond::FromTwoVectors(v_p_c_ac, v_p_c_ap));
  Eigen::Quaternion Q_B_p_c_cOffINp =
    Eigen::Quaterniond(Eigen::AngleAxisd(q_p_c_cOffset, v_p_c_ap));

  Frames frames;
  frames.Q_W_FP = Q_B_p_c_jOffINp * Q_B_p_c_jINp;
  frames.Q_W_FC = Q_B_p_c_cINp.inverse() * Q_B_p_c_cOffINp.inverse() *
    Q_B_p_c_jOffINp * Q_B_p_c_jINp;

  return frames;
}