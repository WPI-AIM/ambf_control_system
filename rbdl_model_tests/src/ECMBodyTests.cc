#include "rbdl_model_tests/ECM.h"
#include "rbdl_model_tests/rbdl_tests.h"

TEST_CASE_METHOD(ECM, __FILE__"_TestECMPositionHierarchy", "") 
{
  std::map< std::string, unsigned int > mBodyNameMap = rbdlModel->mBodyNameMap;
  std::map<std::string, unsigned int>::iterator mBodyNameMapItr;

  for(mBodyNameMapItr = mBodyNameMap.begin(); mBodyNameMapItr != mBodyNameMap.end(); mBodyNameMapItr++)
  {
    std::string bodyName = mBodyNameMapItr->first;
    unsigned int bodyId = mBodyNameMapItr->second;
    std::string parentName = rbdlModel->GetBodyName(rbdlModel->GetParentBodyId(bodyId));
    std::cout << parentName << ", " << bodyName << ", " << bodyId << std::endl;
  }
  std::cout << std::endl << "------------------" << std::endl;

  // for(mBodyNameMapItr = mBodyNameMap.begin(); mBodyNameMapItr != mBodyNameMap.end(); mBodyNameMapItr++)
  // {
  //   std::string bodyName = mBodyNameMapItr->first;
  //   unsigned int bodyId = mBodyNameMapItr->second;
  //   std::string parentName = rbdlModel->GetBodyName(rbdlModel->GetParentBodyId(bodyId));
    
  //   // std::cout << parentName << ", " << bodyName << ", " << bodyId << std::endl;

  //   // std::cout << hierachyMap[bodyName] << ": " << parentName 
  //   // << ", " << bodyName << bodyId << ", " << std::endl;
  //   // CHECK( parentName == hierachyMap[bodyName]);
  // }
}