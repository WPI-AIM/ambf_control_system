#include<iostream>
#include "rbdl_model/BuildRBDLModel.h"
#include <UnitTest++/UnitTest++.h>

int main(int argc, char* argv[])
{
//    const std::string actuator_config_file = "/localcodebase/ambfnags92/ambf/ambf_models/descriptions/multi-bodies/robots/blender-psm.yaml";
//    const std::string actuator_config_file = "/localcodebase/ambfnags92/ambf/ambf_models/descriptions/multi-bodies/robots/blender-kuka.yaml";

//    BuildRBDLModel buildRBDLModel(actuator_config_file);
//    buildRBDLModel.printBody();

//    buildRBDLModel.printJoint();
//    buildRBDLModel.cleanUp();

    rbdl_check_api_version (RBDL_API_VERSION);

    if (argc > 1) {
      std::string arg (argv[1]);

      if (arg == "-v" || arg == "--version")
        rbdl_print_version();
    }

    return UnitTest::RunAllTests ();
}
