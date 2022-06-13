#include "rbdl_model_tests/RBDLTestPrep.h"
// #include "application/Prep.h"
#include "rbdl_model_tests/rbdl_tests.h"

TEST_CASE(__FILE__"_Initilize", "") 
{
 BuildRBDLModelPtr rbdlModelPtr = RBDLTestPrep::getInstance()->getRBDLModelInstance();
}