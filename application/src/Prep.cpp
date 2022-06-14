#include "application/Prep.h"

std::atomic<AMBFTestPrep*> AMBFTestPrep::instance_;
std::mutex AMBFTestPrep::myMutex_;

std::chrono::duration<double> AMBFTestPrep::getTime(){

  auto begin= std::chrono::system_clock::now();
  for ( size_t i= 0; i <= tenMill; ++i){
       AMBFTestPrep::getInstance();
  }
  return std::chrono::system_clock::now() - begin;
}

// AMBFClientPtr AMBFTestPrep::getAMBFClientInstance() {
//   AMBFClientPtr clientPtr = new Client();
//   return clientPtr;
// }

AMBFWrapperPtr AMBFTestPrep::getAMBFWrapperInstance() {
  AMBFWrapperPtr ambfWrapperPtr = new AMBFWrapper();
  return ambfWrapperPtr;
}
