#include "rbdl_model_tests/ParallelStructure.h"

TEST_CASE_METHOD(ParallelStructure, __FILE__"_TestPSPositionNeutral", "") 
{
  std::map< std::string, unsigned int > mBodyNameMap = rbdlPSModel->mBodyNameMap;
  std::map<std::string, unsigned int>::iterator mBodyNameMapItr;
  for(mBodyNameMapItr = mBodyNameMap.begin(); mBodyNameMapItr != mBodyNameMap.end(); mBodyNameMapItr++)
  {
    std::string bodyName = mBodyNameMapItr->first;
    unsigned int bodyId = mBodyNameMapItr->second;
    std::string parentName = rbdlPSModel->GetBodyName(rbdlPSModel->GetParentBodyId(bodyId));
    
    CHECK( parentName == reference_hierachy_map[bodyName]);
  }

}