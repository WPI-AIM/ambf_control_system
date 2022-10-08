#include "PCH/pch.h"
#include "ambf_client/ambf_client.h"

int main()
{
  printf("Hello\n");

  Eigen::Matrix2d m;
  m.setRandom();
  std::cout << m << std::endl;

  RigidBodyDynamics::Math::Vector3d v;
  v.setIdentity();
  std::cout << v << std::endl;

  Client client("ecm_ik_test"); 
  return 0;
}