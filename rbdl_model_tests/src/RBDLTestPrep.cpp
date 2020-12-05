#include "rbdl_model_tests/RBDLTestPrep.h"

std::atomic<RBDLTestPrep*> RBDLTestPrep::instance;
std::mutex RBDLTestPrep::myMutex;

std::chrono::duration<double> RBDLTestPrep::getTime(){

  auto begin= std::chrono::system_clock::now();
  for ( size_t i= 0; i <= tenMill; ++i){
       RBDLTestPrep::getInstance();
  }
  return std::chrono::system_clock::now() - begin;

}

//ClientPtr RBDLTestPrep::getAMBFClientInstance(){
//  ClientPtr clientPtr = new Client();
//  return clientPtr;
//}

BuildRBDLModelPtr RBDLTestPrep::getRBDLModelInstance() {
    BuildRBDLModelPtr buildRBDLModelPtr = new BuildRBDLModel (actuator_config_file_);

    return buildRBDLModelPtr;
}
