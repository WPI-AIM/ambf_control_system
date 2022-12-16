#ifndef _UTILITIES_
#define _UTILITIES_

#include "rbdl/rbdl.h"
#include "PCH/pch.h"
#include "Tests/rbdl_tests.h"

// Parent and Child frame w.r.t Parent frame
struct Frames
{
  Eigen::Quaterniond Q_W_FP;
  Eigen::Quaterniond Q_W_FC;
};

class Utilities
{
public:
  static Quaternion EigenToRBDLQuaternion(const Eigen::Quaterniond &quat_eigen);
  static Frames Q_B_Frames(const Vector3d& v_p_c_ax_jINp, const Vector3d& v_p_c_ap, const Vector3d& v_p_c_ac,
    const double& q_p_c_jOffset, const double& q_p_c_cOffset);

};
#endif