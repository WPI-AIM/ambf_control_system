#include "rbdl_model_tests/ModelFromParser/RBDLModelFromAutomation.h"
#include "rbdl_model_tests/rbdl_tests.h"

RBDLModelFromAutomation* rbdlModel = nullptr;


TEST_CASE(__FILE__"_Initilize", "") 
{
  rbdlModel = new RBDLModelFromAutomation();
}


TEST_CASE(__FILE__"_Body", "") 
{
  rbdlModel->PrintModelHierarchy();
}

TEST_CASE(__FILE__"_CleanUp", "") 
{
  // delete rbdlModel;
}