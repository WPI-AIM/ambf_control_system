#include "rbdl_model_tests/ECM.h"

TEST_CASE_METHOD(ECM, __FILE__"_TestECMPositionNeutral", "") 
{
  std::map< std::string, unsigned int > mBodyNameMap = rbdlECMModel->mBodyNameMap;
  std::map<std::string, unsigned int>::iterator mBodyNameMapItr;

  // for(mBodyNameMapItr = mBodyNameMap.begin(); mBodyNameMapItr != mBodyNameMap.end(); mBodyNameMapItr++)
  // {
  //   std::string bodyName = mBodyNameMapItr->first;
  //   unsigned int bodyId = mBodyNameMapItr->second;
  //   std::string parentName = rbdlECMModel->GetBodyName(rbdlECMModel->GetParentBodyId(bodyId));
  //   std::cout << parentName << ", " << bodyName << ", " << bodyId << std::endl;
  // }

  for(mBodyNameMapItr = mBodyNameMap.begin(); mBodyNameMapItr != mBodyNameMap.end(); mBodyNameMapItr++)
  {
    std::string bodyName = mBodyNameMapItr->first;
    unsigned int bodyId = mBodyNameMapItr->second;
    std::string parentName = rbdlECMModel->GetBodyName(rbdlECMModel->GetParentBodyId(bodyId));
    
    // std::cout << bodyId << ", " << bodyName << ": " << parentName 
    // << ", " << hierachyMap[bodyName] << std::endl;
    CHECK( parentName == hierachyMap[bodyName]);
  }
}