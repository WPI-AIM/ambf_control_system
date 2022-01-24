#include "rbdl_model_tests/RBDLTestPrep.h"
#include "rbdl_model_tests/KUKA.h"


/*
TEST_CASE (__FILE__"_TestKukaBodyMass", "" )
{
    AMBFClientPtr clientPtr = RBDLTestPrep::getInstance()->getAMBFClientInstance();
    clientPtr->connect();

    BuildRBDLModelPtr rbdlModelPtr = RBDLTestPrep::getInstance()->getRBDLModelInstance();
    std::string baseRigidBody = rbdlModelPtr->getBaseRigidBody();
    struct RigidBodyDynamics::Model* rbdlModel = rbdlModelPtr->getRBDLModel();

    std::vector<std::string> rigidBodyNamesAmbf = clientPtr->getRigidBodyNames();

    for(std::string bodyAMBF : rigidBodyNamesAmbf)
    {
        unsigned int rbdlBodyId = rbdlModel->GetBodyId(bodyAMBF.c_str());

        if(rbdlBodyId && rbdlBodyId <rbdlModel->mBodies.size())
        {
            rigidBodyPtr rigidBodyHandlerAmbf = clientPtr->getRigidBody(bodyAMBF, true);
            usleep(1000000);
            double bodyMassAMBF = rigidBodyHandlerAmbf->get_mass();
            double bodyMassRBDL = rbdlModel->mBodies.at(rbdlBodyId).mMass;

            CHECK (bodyMassAMBF == bodyMassRBDL);
        }

    }
    rbdlModelPtr->~BuildRBDLModel();
    clientPtr->cleanUp();
}

TEST_CASE (__FILE__"_TestKukaBodyInertia", "" )
{
    AMBFClientPtr clientPtr = RBDLTestPrep::getInstance()->getAMBFClientInstance();
    clientPtr->connect();

    BuildRBDLModelPtr rbdlModelPtr = RBDLTestPrep::getInstance()->getRBDLModelInstance();
    std::string baseRigidBody = rbdlModelPtr->getBaseRigidBody();
    struct RigidBodyDynamics::Model* rbdlModel = rbdlModelPtr->getRBDLModel();

    std::vector<std::string> rigidBodyNamesAmbf = clientPtr->getRigidBodyNames();

    for(std::string bodyAMBF : rigidBodyNamesAmbf)
    {
        unsigned int rbdlBodyId = rbdlModel->GetBodyId(bodyAMBF.c_str());

        if(rbdlBodyId && rbdlBodyId <rbdlModel->mBodies.size())
        {
            rigidBodyPtr rigidBodyHandlerAmbf = clientPtr->getRigidBody(bodyAMBF, true);
            usleep(1000000);
            
            tf::Vector3 I = rigidBodyHandlerAmbf->get_inertia();
            RigidBodyDynamics::Math::Matrix3d bodyInertiaAMBF;
            bodyInertiaAMBF.setZero();

            bodyInertiaAMBF(0, 0) = I[0];
            bodyInertiaAMBF(1, 1) = I[1];
            bodyInertiaAMBF(2, 2) = I[2];

            RigidBodyDynamics::Math::Matrix3d bodyInertiaRBDL = rbdlModel->mBodies.at(rbdlBodyId).mInertia;

            CHECK_THAT(bodyInertiaAMBF, AllCloseMatrix(bodyInertiaRBDL,
                                               TEST_PREC, TEST_PREC));
        }

    }
    rbdlModelPtr->~BuildRBDLModel();
    clientPtr->cleanUp();
}
*/

TEST_CASE_METHOD(KUKA, __FILE__"_TestKUKAPositionNeutral", "") 
{
  std::map< std::string, unsigned int > mBodyNameMap = rbdlModel->mBodyNameMap;
  std::map<std::string, unsigned int>::iterator mBodyNameMapItr;

//   for(mBodyNameMapItr = mBodyNameMap.begin(); mBodyNameMapItr != mBodyNameMap.end(); mBodyNameMapItr++)
//   {
//     std::string bodyName = mBodyNameMapItr->first;
//     unsigned int bodyId = mBodyNameMapItr->second;
//     std::string parentName = rbdlModel->GetBodyName(rbdlModel->GetParentBodyId(bodyId));
//     std::cout << parentName << ", " << bodyName << ", " << bodyId << std::endl;
//   }

  for(mBodyNameMapItr = mBodyNameMap.begin(); mBodyNameMapItr != mBodyNameMap.end(); mBodyNameMapItr++)
  {
    std::string bodyName = mBodyNameMapItr->first;
    unsigned int bodyId = mBodyNameMapItr->second;
    std::string parentName = rbdlModel->GetBodyName(rbdlModel->GetParentBodyId(bodyId));
    
    // std::cout << bodyId << ", " << bodyName << ": " << parentName 
    // << ", " << hierachyMap[bodyName] << std::endl;
    CHECK( parentName == hierachyMap[bodyName]);
  }
}