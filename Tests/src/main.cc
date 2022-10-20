#define CATCH_CONFIG_RUNNER
#include "PCH/pch.h"
int main (int argc, char *argv[])
{
  rbdl_check_api_version (RBDL_API_VERSION);

  if (argc > 1) {
    std::string arg (argv[1]);

    if (arg == "-v" || arg == "--version") {
      rbdl_print_version();
      return 0;
    }
  }

  // Client client("ecm");
  // client.cleanUp(); 
  
  return Catch::Session().run(argc, argv);
}
