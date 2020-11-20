#include<iostream>
#include "rbdl_model/BuildRBDLModel.h"
#include <UnitTest++/UnitTest++.h>

int main(int argc, char* argv[])
{
    rbdl_check_api_version (RBDL_API_VERSION);

    if (argc > 1) {
      std::string arg (argv[1]);

      if (arg == "-v" || arg == "--version")
        rbdl_print_version();
    }

    return UnitTest::RunAllTests ();
}
