#ifndef TESTCASESTRUCTS_H
#define TESTCASESTRUCTS_H

#include <iostream>
#include "rbdl/rbdl_math.h"
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

struct T_W_N
{
  std::string bodyName;

  Matrix3d r_w_n_ambf;
  Vector3d p_w_n_ambf;

  Matrix3d r_w_n_rbdl;
  Vector3d p_w_n_rbdl;
};


#endif