#include "PCH/pch.h"
#include <ambf_control_system.h>

int main(int argc, char* argv[])
{
  std::cout << argv[0] << "Version " 
  << ambf_control_system_VERSION_MAJOR << "." <<
  ambf_control_system_VERSION_MINOR << std::endl;

  printf("Hello\n");

  Eigen::Matrix2d m;
  m.setRandom();
  std::cout << m << std::endl;

  RigidBodyDynamics::Math::Vector3d v;
  v.setIdentity();
  std::cout << v << std::endl;

  Client client("ecm");
  client.cleanUp(); 
  return 0;
}