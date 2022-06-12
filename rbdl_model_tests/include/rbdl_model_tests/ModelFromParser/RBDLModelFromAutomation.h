#include <unordered_map>
#include <thread>
#include "application/Utilities.h"
#include "application/Prep.h"
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <algorithm>
#include "rbdl_model/BuildRBDLModel.h"
#include "rbdl/rbdl_math.h"
#include "rbdl_model_tests/rbdl_tests.h"
#include "rbdl_model_tests/ModelFromParser/TestCaseStruct.h"

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

typedef BuildRBDLModel* BuildRBDLModelPtr;
typedef T_W_N* t_w_nPtr;

class RBDLModelFromAutomation 
{
public:
  RBDLModelFromAutomation();
  ~RBDLModelFromAutomation();

  void PrintModelHierarchy();


  void CleanUp();
private:
  RBDLModelPtr rbdlModelPtr_{nullptr};
  BuildRBDLModelPtr buildRBDLModelPtr_{nullptr};

  VectorNd Q_;
  VectorNd QDot_;
  VectorNd QDDot_;
  VectorNd Tau_;
  std::map< std::string, unsigned int > rbdlmBodyMap_;
  std::map<std::string, unsigned int>::iterator rbdlmBodyMapItr_;

  template<typename T>
  t_w_nPtr twnFromModels(T t);
};