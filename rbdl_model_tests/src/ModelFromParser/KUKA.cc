#include "rbdl_model_tests/RBDLTestPrep.h"

TEST_CASE(__FILE__"_Initilize", "") 
{
 BuildRBDLModelPtr rbdlModelPtr = RBDLTestPrep::getInstance()->getRBDLModelInstance();
}