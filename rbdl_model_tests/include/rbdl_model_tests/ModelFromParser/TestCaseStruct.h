#ifndef TESTCASESTRUCTS_H
#define TESTCASESTRUCTS_H

#include <iostream>
#include "rbdl/rbdl_math.h"
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

struct T_W_N
{
  std::string bodyName;

  SpatialTransform t_w_n_ambf;
  SpatialTransform t_w_n_rbdl;
};


#endif