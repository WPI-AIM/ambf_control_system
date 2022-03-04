#include "rbdl_model_tests/RBDLTestPrep.h"
#include "rbdl_model_tests/EigenUtilities.h"
//#include "rbdl_model_tests/Human36Fixture.h"
#include <unordered_map>

//const double TEST_PREC = 1.0e-12;
const double TEST_LAX = 1.0e-7;


struct Kuka {
    Kuka () {
        ClearLogOutput();

        BuildRBDLModelPtr rbdlModelPtr = RBDLTestPrep::getInstance()->getRBDLModelInstance();
        base = rbdlModelPtr->getBaseRigidBody();


        clientPtr = RBDLTestPrep::getInstance()->getAMBFClientInstance();
        clientPtr->connect();

        baseHandler = clientPtr->getRigidBody(base, true);
        usleep(1000000);

        //base is rigid body name, not a joint. This is a hacky way to enable ros topics in the 
        //server side during first execution
        baseHandler->set_joint_pos(base, 0.0f); 

        tf::Vector3 P_0_w_tf = baseHandler->get_pos();
        Eigen::Vector3d P_0_w;
        P_0_w[0]= P_0_w_tf[0];
        P_0_w[1]= P_0_w_tf[1];
        P_0_w[2]= P_0_w_tf[2];
        
        tf::Vector3 R_0_w_tf = baseHandler->get_rpy();

        Eigen::Matrix3d R_0_w = eigenUtilities.rotation_from_euler<Eigen::Matrix3d>(R_0_w_tf[0], R_0_w_tf[1], R_0_w_tf[2]);

        T_0_w = eigenUtilities.get_frame<Eigen::Matrix3d, Eigen::Vector3d, Eigen::Matrix4d>(R_0_w, P_0_w);

        rbdlModel = rbdlModelPtr->getRBDLModel();
        Q = VectorNd::Constant ((size_t) rbdlModel->dof_count, 0.);
        QDot = VectorNd::Constant ((size_t) rbdlModel->dof_count, 0.);
        QDDot = VectorNd::Constant ((size_t) rbdlModel->dof_count, 0.);
        Tau = VectorNd::Constant ((size_t) rbdlModel->dof_count, 0.);

        //TO DO: Should be set in rbdl model
        KUKA_JOINT_LIMITS[ "link1" ] = { -2.094f, 2.094f };
        KUKA_JOINT_LIMITS[ "link2" ] = { -2.094f, 2.094f };
        KUKA_JOINT_LIMITS[ "link3" ] = { -2.094f, 2.094f };
        KUKA_JOINT_LIMITS[ "link4" ] = { -2.094f, 2.094f }; 
        KUKA_JOINT_LIMITS[ "link5" ] = { -2.094f, 2.094f }; 
        KUKA_JOINT_LIMITS[ "link6" ] = { -2.094f, 2.094f }; 
        KUKA_JOINT_LIMITS[ "link7" ] = { -3.054f, 3.054f }; 
        ClearLogOutput();
    }

    ~Kuka () {
        delete rbdlModel;
        clientPtr->cleanUp();
    }

    Model *rbdlModel = nullptr;
    AMBFClientPtr clientPtr = nullptr;
    rigidBodyPtr baseHandler = nullptr;
    std::string base;
    Eigen::Matrix4d T_0_w;
    std::unordered_map<std::string, std::vector<float>> KUKA_JOINT_LIMITS;
    EigenUtilities eigenUtilities;

    VectorNd Q;
    VectorNd QDot;
    VectorNd QDDot;
    VectorNd Tau;
};



TEST_CASE_METHOD(Kuka, __FILE__"_TestKUKAPositionNeutral", "") 
{
  // We call ForwardDynamics() as it updates the spatial transformation
  // matrices
  ForwardDynamics(*rbdlModel, Q, QDot, Tau, QDDot);

  std::vector<std::string> joints = baseHandler->get_joint_names();
  std::vector<std::string> baseChildren = baseHandler->get_children_names();

//   std::map< std::string, unsigned int > mBodyNameMap = rbdlModel->mBodyNameMap;
//   std::map<std::string, unsigned int>::iterator mBodyNameMapItr;
//   for(mBodyNameMapItr = mBodyNameMap.begin(); mBodyNameMapItr != mBodyNameMap.end(); mBodyNameMapItr++)
//   {
//     std::cout << mBodyNameMapItr->first << ", " << mBodyNameMapItr->second << std::endl;
//   }


  CHECK (baseChildren.size() == Q.size());

  for(int i = 0; i < 20; i++)
  {
    for(std::string joint : joints)
    {
      baseHandler->set_joint_pos<std::string>(joint, 0.0f);
    }

    usleep(250000);
  }

  Q.setZero();

  for(std::string body : baseChildren)
  {
    unsigned int rbdlBodyId = rbdlModel->GetBodyId(body.c_str());

    if(rbdlBodyId < rbdlModel->mBodies.size())
    {
      rigidBodyPtr rigidBodyHandler = clientPtr->getRigidBody(body, true);
      
      // n - respective body frame
      const tf::Vector3 P_n_w_tf = rigidBodyHandler->get_pos();

      const RigidBodyDynamics::Math::Vector3d P_n_0_rbd_rbdl = CalcBodyToBaseCoordinates(*rbdlModel, Q, rbdlBodyId, 
                                                              RigidBodyDynamics::Math::Vector3d(0., 0., 0.),true);

      RigidBodyDynamics::Math::Vector3d P_n_w_rbd_ambf;
      P_n_w_rbd_ambf.setZero();
      
      P_n_w_rbd_ambf(0) = P_n_w_tf[0];
      P_n_w_rbd_ambf(1) = P_n_w_tf[1];
      P_n_w_rbd_ambf(2) = P_n_w_tf[2];

      RigidBodyDynamics::Math::Vector4d P_n_w_rbd_rbdl_4d;
      P_n_w_rbd_rbdl_4d.setOnes();
      P_n_w_rbd_rbdl_4d(0) = P_n_0_rbd_rbdl(0);
      P_n_w_rbd_rbdl_4d(1) = P_n_0_rbd_rbdl(1);
      P_n_w_rbd_rbdl_4d(2) = P_n_0_rbd_rbdl(2);

      P_n_w_rbd_rbdl_4d = T_0_w * (P_n_w_rbd_rbdl_4d);

      RigidBodyDynamics::Math::Vector3d P_n_w_rbd_rbdl;
      P_n_w_rbd_rbdl = P_n_w_rbd_rbdl_4d.block<3,1>(0,0);
          
      CHECK_THAT(P_n_w_rbd_ambf, AllCloseMatrix(P_n_w_rbd_rbdl, TEST_PREC, TEST_PREC));
    }
  }
}

