#include <stdio.h>
#include <rbdl/Body.h>
#include <Eigen/Dense>
#include <iostream>
#include "catch2/catch.hpp"

int main()
{
  printf("Hello\n");

  Eigen::Matrix2d m;
  m.setRandom();
  std::cout << m << std::endl;

  RigidBodyDynamics::Math::Vector3d v;
  v.setIdentity();
  std::cout << v << std::endl;
  return 0;
}