#include "rbdl_model_tests/RBDLTestPrep.h"
#include "rbdl_model_tests/EigenUtilities.h"
#include <unordered_map>
#include <Eigen/Geometry> 
#include <boost/range/combine.hpp>
#include <math.h>

//const double TEST_PREC = 1.0e-12;
const double TEST_LAX = 1.0e-7;
struct ActivationJoints
{
  std::string joint_name;
  float joint_lower_limit;
  float joint_higher_limit;
};

struct KUKA {
  KUKA () {
    ClearLogOutput();
    usleep(1000000);
    clientPtr = RBDLTestPrep::getInstance()->getAMBFClientInstance();
    clientPtr->connect();
    std::cout<<base_name<<"\n";
    vector<string> my_names = clientPtr->getRigidBodyNames();


    for(auto & name: my_names )
    {
      std::cout<<name<<"\n";
    }

    baseHandler = clientPtr->getRigidBody(base_name, false);
    usleep(1000000);

    // //base is rigid body name, not a joint. This is a hacky way to enable ros topics in the 
    // //server side during first execution
    baseHandler->set_joint_pos(base_name, 0.0f); 

    const tf::Quaternion quat_0_w_tf = baseHandler->get_rot();
    const tf::Vector3 P_0_w_tf = baseHandler->get_pos();

    RigidBodyDynamics::Math::Quaternion quat_0_w;
    quat_0_w(0) = quat_0_w_tf[0];
    quat_0_w(1) = quat_0_w_tf[1];
    quat_0_w(2) = quat_0_w_tf[2];
    quat_0_w(3) = quat_0_w_tf[3];

    const RigidBodyDynamics::Math::Matrix3d R_0_w = quat_0_w.toMatrix();

    RigidBodyDynamics::Math::Vector3d P_0_w;
    P_0_w.setZero();
    
    P_0_w(0) = P_0_w_tf[0];
    P_0_w(1) = P_0_w_tf[1];
    P_0_w(2) = P_0_w_tf[2];

    T_0_w = EigenUtilities::get_frame<Eigen::Matrix3d, Eigen::Vector3d, Eigen::Matrix4d>(R_0_w, P_0_w);

    rbdlModel = new Model;
    rbdlModel->gravity = RigidBodyDynamics::Math::Vector3d(0., 0., -9.81);

    // mass, com - inertia offset, inertia
    baseBody  = Body (1., Math::Vector3d (00.001, 00.000, 0.060), Math::Vector3d (0.0000, 0.0000, 0.0000));
    link1Body = Body (1., Math::Vector3d (00.000, -0.017, 0.134), Math::Vector3d (0.0452, 0.0446, 0.0041));
    link2Body = Body (1., Math::Vector3d (00.000, -0.074, 0.009), Math::Vector3d (0.0227, 0.0037, 0.0224));
    link3Body = Body (1., Math::Vector3d (00.000, 00.017, 0.134), Math::Vector3d (0.0417, 0.0418, 0.0038));
    link4Body = Body (1., Math::Vector3d (-0.001, 00.081, 0.008), Math::Vector3d (0.0249, 0.0036, 0.0247));
    link5Body = Body (1., Math::Vector3d (0.0000, -0.017, 0.129), Math::Vector3d (0.0363, 0.0350, 0.0045));
    link6Body = Body (1., Math::Vector3d (0.0000, 00.007, 0.068), Math::Vector3d (0.0114, 0.0116, 0.0037));
    link7Body = Body (1., Math::Vector3d (0.0060, 00.000, 0.015), Math::Vector3d (0.0012, 0.0013, 0.0010));
    //--------------------------------------------------------------------//
    ROOT_baseJoint = Joint(JointTypeFixed);
    ROOT_baseST.E = RigidBodyDynamics::Math::Matrix3dIdentity;
    ROOT_baseST.r = RigidBodyDynamics::Math::Vector3dZero;
    base_link1ID = rbdlModel->AddBody(0, ROOT_baseST, ROOT_baseJoint, baseBody, "base");
    //--------------------------------------------------------------------//
    Eigen::Vector3d base_link1PA = { 00.000, 00.000, 01.000 };
    Eigen::Vector3d base_link1CA = { 00.000, 00.000, 01.000 };
    Eigen::Vector3d base_link1PP = { 00.000, 00.000, 00.103 };
    Eigen::Vector3d base_link1CP = { 00.000, 00.000, 00.000 };
    base_link1PA.normalize();
    base_link1CA.normalize();

    Eigen::Matrix3d base_link1Rot = 
          Eigen::Matrix3d(Eigen::Quaterniond::FromTwoVectors(base_link1PA, base_link1CA));
    Eigen::Matrix3d base_link1_offset = EigenUtilities::rotZ(base_link1Offset);

    base_link1ST.E = base_link1_offset * base_link1Rot;
    base_link1ST.r = base_link1PP - (base_link1Rot.inverse() * base_link1CP);

    base_link1Joint = Joint(JointTypeRevolute, Math::Vector3d(0.0, 0.0, 1.0));
    base_link1ID = rbdlModel->AddBody(base_link1ID, base_link1ST, base_link1Joint, link1Body, "link1");
    //--------------------------------------------------------------------//
    Eigen::Vector3d link1_link2PA = { 00.000, 01.000, 00.000 };
    Eigen::Vector3d link1_link2CA = { 00.000, 00.000, 01.000 };
    Eigen::Vector3d link1_link2PP = { 00.000, 00.013, 00.209 };
    Eigen::Vector3d link1_link2CP = { 00.000, 00.000, 00.000 };
    link1_link2PA.normalize();
    link1_link2CA.normalize();

    Eigen::Matrix3d link1_link2Rot = 
          Eigen::Matrix3d(Eigen::Quaterniond::FromTwoVectors(link1_link2PA, link1_link2CA));
    Eigen::Matrix3d link1_link2_offset = EigenUtilities::rotZ(link1_link2Offset);

    link1_link2ST.E = link1_link2_offset * link1_link2Rot;
    link1_link2ST.r = link1_link2PP - (link1_link2Rot.inverse() * link1_link2CP);

    link1_link2Joint = Joint(JointTypeRevolute, Math::Vector3d(0.0, 0.0, 1.0));
    link1_link2ID = rbdlModel->AddBody(base_link1ID, link1_link2ST, link1_link2Joint, link2Body, "link2");
    //--------------------------------------------------------------------//
    Eigen::Vector3d link2_link3PA = { 00.000, -1.000, 00.000 };
    Eigen::Vector3d link2_link3CA = { 00.000, 00.000, 01.000 };
    Eigen::Vector3d link2_link3PP = { 00.000, -0.194, -0.009 };
    Eigen::Vector3d link2_link3CP = { 00.000, 00.000, 00.000 };
    link2_link3PA.normalize();
    link2_link3CA.normalize();

    Eigen::Matrix3d link2_link3Rot = 
          Eigen::Matrix3d(Eigen::Quaterniond::FromTwoVectors(link2_link3PA, link2_link3CA));
    Eigen::Matrix3d link2_link3_offset = EigenUtilities::rotZ(link2_link3Offset);

    link2_link3ST.E = link2_link3_offset * link2_link3Rot;
    link2_link3ST.r = link2_link3PP - (link2_link3Rot.inverse() * link2_link3CP);

    link2_link3Joint = Joint(JointTypeRevolute, Math::Vector3d(0.0, 0.0, 1.0));
    link2_link3ID = rbdlModel->AddBody(link1_link2ID, link2_link3ST, link2_link3Joint, link3Body, "link3");
    //--------------------------------------------------------------------//
    Eigen::Vector3d link3_link4PA = { 00.000, -1.000, 00.000 };
    Eigen::Vector3d link3_link4CA = { 00.000, 00.000, 01.000 };
    Eigen::Vector3d link3_link4PP = { 00.000, -0.013, 00.202 };
    Eigen::Vector3d link3_link4CP = { 00.000, 00.000, 00.000 };
    link3_link4PA.normalize();
    link3_link4CA.normalize();

    Eigen::Matrix3d link3_link4Rot = 
          Eigen::Matrix3d(Eigen::Quaterniond::FromTwoVectors(link3_link4PA, link3_link4CA));
    Eigen::Matrix3d link3_link4_offset = EigenUtilities::rotZ(link3_link4Offset);

    link3_link4ST.E = link3_link4_offset * link3_link4Rot;
    link3_link4ST.r = link3_link4PP - (link3_link4Rot.inverse() * link3_link4CP);

    link3_link4Joint = Joint(JointTypeRevolute, Math::Vector3d(0.0, 0.0, 1.0));
    link3_link4ID = rbdlModel->AddBody(link2_link3ID, link3_link4ST, link3_link4Joint, link4Body, "link4");
    //--------------------------------------------------------------------//
    Eigen::Vector3d link4_link5PA = { 00.000, 01.000, 00.000 };
    Eigen::Vector3d link4_link5CA = { 00.000, 00.000, 01.000 };
    Eigen::Vector3d link4_link5PP = { -0.002, 00.202, -0.008 };
    Eigen::Vector3d link4_link5CP = { 00.000, 00.000, 00.000 };
    link4_link5PA.normalize();
    link4_link5CA.normalize();

    Eigen::Matrix3d link4_link5Rot = 
          Eigen::Matrix3d(Eigen::Quaterniond::FromTwoVectors(link4_link5PA, link4_link5CA));
    Eigen::Matrix3d link4_link5_offset = EigenUtilities::rotZ(link5_link6Offset);

    link4_link5ST.E = link4_link5_offset * link4_link5Rot;
    link4_link5ST.r = link4_link5PP - (link4_link5Rot.inverse() * link4_link5CP);

    link4_link5Joint = Joint(JointTypeRevolute, Math::Vector3d(0.0, 0.0, 1.0));
    link4_link5ID = rbdlModel->AddBody(link3_link4ID, link4_link5ST, link4_link5Joint, link5Body, "link5");
    //--------------------------------------------------------------------//
    Eigen::Vector3d link5_link6PA = { 00.000, 01.000, 00.000 };
    Eigen::Vector3d link5_link6CA = { 00.000, 00.000, 01.000 };
    Eigen::Vector3d link5_link6PP = { 00.002, -0.052, 00.204 };
    Eigen::Vector3d link5_link6CP = { 00.000, 00.000, 00.000 };
    link5_link6PA.normalize();
    link5_link6CA.normalize();

    Eigen::Matrix3d link5_link6Rot = 
          Eigen::Matrix3d(Eigen::Quaterniond::FromTwoVectors(link5_link6PA, link5_link6CA));
    Eigen::Matrix3d link5_link6_offset = EigenUtilities::rotZ(link5_link6Offset);

    link5_link6ST.E = link5_link6_offset * link5_link6Rot;
    link5_link6ST.r = link5_link6PP - (link5_link6Rot.inverse() * link5_link6CP);

    link5_link6Joint = Joint(JointTypeRevolute, Math::Vector3d(0.0, 0.0, 1.0));
    link5_link6ID = rbdlModel->AddBody(link4_link5ID, link5_link6ST, link5_link6Joint, link6Body, "link6");
    //--------------------------------------------------------------------//
    Eigen::Vector3d link6_link7PA = { 00.000, -1.000, 00.000 };
    Eigen::Vector3d link6_link7CA = { 00.000, 00.000, 01.000 };
    Eigen::Vector3d link6_link7PP = { -0.003, -0.050, 00.053 };
    Eigen::Vector3d link6_link7CP = { 00.000, 00.000, 00.000 };
    link6_link7PA.normalize();
    link6_link7CA.normalize();

    Eigen::Matrix3d link6_link7Rot = 
          Eigen::Matrix3d(Eigen::Quaterniond::FromTwoVectors(link6_link7PA, link6_link7CA));
    Eigen::Matrix3d link6_link7_offset = EigenUtilities::rotZ(link6_link7Offset);

    link6_link7ST.E = link6_link7_offset * link6_link7Rot;
    link6_link7ST.r = link6_link7PP - (link6_link7Rot.inverse() * link6_link7CP);

    link6_link7Joint = Joint(JointTypeRevolute, Math::Vector3d(0.0, 0.0, 1.0));
    link6_link7ID = rbdlModel->AddBody(link5_link6ID, link6_link7ST, link6_link7Joint, link7Body, "link7");
    //--------------------------------------------------------------------//
    Q     = VectorNd::Constant ((size_t) rbdlModel->dof_count, 0.);
    QDot  = VectorNd::Constant ((size_t) rbdlModel->dof_count, 0.);
    QDDot = VectorNd::Constant ((size_t) rbdlModel->dof_count, 0.);
    Tau   = VectorNd::Constant ((size_t) rbdlModel->dof_count, 0.); 

    ClearLogOutput();
  }

  ~KUKA () 
  {
    delete rbdlModel;
    clientPtr->cleanUp();
  }

  Model *rbdlModel = nullptr;
  AMBFClientPtr clientPtr = nullptr;
  rigidBodyPtr baseHandler = nullptr;
  std::string base_name = "base";

  Eigen::Matrix4d T_0_w;
  // Child : Parent
  std::unordered_map<std::string, std::string> hierachyMap =
  {
    {  "ROOT", "ROOT" },
    {  "ROOT-base", "ROOT" },
    { "base-link1", "ROOT" },
    { "link1-link2", "base-link1" },
    { "link2-link3", "link1-link2" },
    { "link3-link4", "link2-link3" },
    { "link4-link5", "link3-link4" },
    { "link5-link6", "link4-link5" },
    { "link6-link7", "link5-link6" },
  };

  std::unordered_map<std::string, ActivationJoints> jointLimits =
  {
    { 
      "link1", { "link1", -2.094f, 2.094f }
    },
    { 
      "link2", { "link2", -2.094f, 2.094f }
    },
    { 
      "link3", { "link3", -2.094f, 2.094f }
    },
    { 
      "link4", { "link4", -2.094f, 2.094f }
    },
    { 
      "link5", { "link5", -2.094f, 2.094f }
    },
    { 
      "link6", { "link6", -2.094f, 2.094f }
    },
    { 
      "link7", { "link7", -3.054f, 3.054f }
    }
  };

  // const std::vector<ActivationJoints> controllableJoints =
  // {
  //   { "link1", -2.094f, 2.094f },
  //   { "link2", -2.094f, 2.094f },
  //   { "link3", -2.094f, 2.094f },
  //   { "link4", -2.094f, 2.094f },
  //   { "link5", -2.094f, 2.094f },
  //   { "link6", -2.094f, 2.094f },
  //   { "link7", -3.054f, 3.054f },
  // };

  // std::unordered_map<std::string, std::vector<std::string>> parentJointsMap =
  // {
  //   { 
  //     "base", 
  //     {
  //        "base-link1", "link1-link2", "link2-link3", "link3-link4", 
  //       "link4-link5", "link5-link6", "link6-link7"
  //     } 
  //   }
  // };

  // std::unordered_map<std::string, std::vector<std::string>>::iterator parentJointsMapItr;

  unsigned int base_link1ID, link1_link2ID, link2_link3ID, link3_link4ID, link4_link5ID, 
               link5_link6ID, link6_link7ID;
  Body baseBody, link1Body, link2Body, link3Body, link4Body, link5Body, link6Body, link7Body;
  Joint ROOT_baseJoint, base_link1Joint, link1_link2Joint, link2_link3Joint, link3_link4Joint, 
        link4_link5Joint, link5_link6Joint, link6_link7Joint;
  SpatialTransform ROOT_baseST, base_link1ST, link1_link2ST, link2_link3ST, link3_link4ST, 
                   link4_link5ST, link5_link6ST, link6_link7ST;

  const double  ROOT_baseOffset  = 0.0;
  const double  base_link1Offset = 0.0;
  const double link1_link2Offset = 0.0;
  const double link2_link3Offset = 0.0;
  const double link3_link4Offset = 0.0;
  const double link4_link5Offset = 0.0;
  const double link5_link6Offset = 0.0;
  const double link6_link7Offset = 0.0;

  VectorNd Q;
  VectorNd QDot;
  VectorNd QDDot;
  VectorNd Tau;
};

double calcEERMS(std::vector<std::vector< float > > ambf_position, std::vector<std::vector< double > > forward_sim_position  )
{

  int N = ambf_position.size();
  int M = forward_sim_position.size();
  double error[7] = {0,0,0,0,0,0,0};
  double rms[7] = {0,0,0,0,0,0,0};
  double totalerror = 0;

  std::cout<<N<<std::endl;
  std::cout<<M<<std::endl;
  // for(auto outer_vec: forward_sim_position)
  // {
  //   std::cout<<"lllllllllllllllllllllllllllllllllllllllllllllllllll"<<std::endl;
  //   for(auto innter_vec: outer_vec)
  //   {
  //     std::cout<<innter_vec << " , ";
  //   }
  //   std::cout<<std::endl;


  // }


  for( int i = 0; i<M; i++)
  {

    std::vector< float > current_ambf = ambf_position[i];
    std::vector< double > current_sim = forward_sim_position[i];
    std::vector< double > diff;
    
    
    for(int j = 0; j <7; j++)
    {
      double de = ((double)current_ambf[j] - current_sim[j]);
      diff.push_back( de*de  );
    }


    for(unsigned int k=0; k<7; ++k)
    {
      error[k]+=diff[k]/N;
    }

    for(unsigned int l=0; l<7; ++l)
    {
      rms[l]= std::sqrt(error[l]);
    }
    
  }


  for( unsigned int i= 0; i < 7; i++)
  {
    totalerror +=rms[i];
  }


  return totalerror;


}