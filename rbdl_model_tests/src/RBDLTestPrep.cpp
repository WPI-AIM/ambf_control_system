#include "rbdl_model_tests/RBDLTestPrep.h"

std::atomic<RBDLTestPrep*> RBDLTestPrep::instance_;
std::mutex RBDLTestPrep::myMutex_;

std::chrono::duration<double> RBDLTestPrep::getTime()
{
  auto begin= std::chrono::system_clock::now();
  for ( size_t i= 0; i <= tenMill; ++i)
  {
    RBDLTestPrep::getInstance();
  }  
  return std::chrono::system_clock::now() - begin;
}

BuildRBDLModelPtr RBDLTestPrep::getRBDLModelInstance() 
{
  BuildRBDLModelPtr buildRBDLModelPtr = new BuildRBDLModel (AMBFTestPrep::ADFPath());

  return buildRBDLModelPtr;
}