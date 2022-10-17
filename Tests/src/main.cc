// #define CATCH_CONFIG_RUNNER
// #include "catch2/catch.hpp"

// #include <iostream>
// #include <string>

// #include <rbdl/rbdl.h>
// #include "Tests/rbdl_tests.h"
#include "PCH/pch.h"
// #include "Eigen/Dense"
int main (int argc, char *argv[])
{

  Eigen::Matrix2d m;
  m.setRandom();
  std::cout << m << std::endl;
  // rbdl_check_api_version (RBDL_API_VERSION);

  // if (argc > 1) {
  //   std::string arg (argv[1]);

  //   if (arg == "-v" || arg == "--version") {
  //     rbdl_print_version();
  //     return 0;
  //   }
  // }
  // return Catch::Session().run(argc, argv);


  return 0;
}
