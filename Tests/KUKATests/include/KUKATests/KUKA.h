#include "rbdl/rbdl.h"
#include "Tests/rbdl_tests.h"

const double TEST_PREC = 1.0e-14;
const double TEST_LAX = 1.0e-5;

struct KUKA 
{
  KUKA()
  {
    ClearLogOutput();
    model = new Model;
    model->gravity = Vector3d(0., 0., -9.81);

    // mass, com - inertia offset, inertia
    worldBody = Body (0.0000001, Vector3d (0.0000, 0.0000, 0.000), Vector3d (0.0000, 0.0000, 0.0000));
    baseBody  = Body (1., Vector3d (00.001, 00.000, 0.060), Vector3d (0.0000, 0.0000, 0.0000));
    Link1Body = Body (1., Vector3d (00.000, -0.017, 0.134), Vector3d (0.0452, 0.0446, 0.0041));
    Link2Body = Body (1., Vector3d (00.000, -0.074, 0.009), Vector3d (0.0227, 0.0037, 0.0224));
    Link3Body = Body (1., Vector3d (00.000, 00.017, 0.134), Vector3d (0.0417, 0.0418, 0.0038));
    Link4Body = Body (1., Vector3d (-0.001, 00.081, 0.008), Vector3d (0.0249, 0.0036, 0.0247));
    Link5Body = Body (1., Vector3d (0.0000, -0.017, 0.129), Vector3d (0.0363, 0.0350, 0.0045));
    Link6Body = Body (1., Vector3d (0.0000, 00.007, 0.068), Vector3d (0.0114, 0.0116, 0.0037));
    Link7Body = Body (1., Vector3d (0.0060, 00.000, 0.015), Vector3d (0.0012, 0.0013, 0.0010));

    //0---------------------------------------------------------------------//
    world_baseST.E.setIdentity();
    world_baseST.r = Vector3d(0.0, 0.0, -1.3);
    //1--------------------------------------------------------------------//
    Matrix3d base_Link1Rot = 
    Eigen::Matrix3d(Eigen::Quaterniond::FromTwoVectors(base_Link1PA, base_Link1CA));
    Eigen::Affine3d base_Link1RotOffset(Eigen::AngleAxisd(base_Link1OffsetQ, Vector3d::UnitZ()));
      
    SpatialTransform base_Link1ST;
    base_Link1ST.E = base_Link1Rot.transpose() * base_Link1RotOffset.rotation();
    base_Link1ST.r = 
      base_Link1PP - (base_Link1Rot.transpose() * base_Link1CP);
    //2--------------------------------------------------------------------//
    Matrix3d Link1_Link2Rot = 
    Eigen::Matrix3d(Eigen::Quaterniond::FromTwoVectors(Link1_Link2PA, Link1_Link2CA));
    Eigen::Affine3d Link1_Link2RotOffset(Eigen::AngleAxisd(Link1_Link2OffsetQ, Vector3d::UnitZ()));
      
    SpatialTransform Link1_Link2ST;
    Link1_Link2ST.E = Link1_Link2Rot.transpose() * Link1_Link2RotOffset.rotation();
    Link1_Link2ST.r = 
      Link1_Link2PP - (Link1_Link2Rot.transpose() * Link1_Link2CP);
    //3--------------------------------------------------------------------//
    Matrix3d Link2_Link3Rot = 
    Eigen::Matrix3d(Eigen::Quaterniond::FromTwoVectors(Link2_Link3PA, Link2_Link3CA));
    Eigen::Affine3d Link2_Link3RotOffset(Eigen::AngleAxisd(Link2_Link3OffsetQ, Vector3d::UnitZ()));
      
    SpatialTransform Link2_Link3ST;
    Link2_Link3ST.E = Link2_Link3Rot.transpose() * Link2_Link3RotOffset.rotation();
    Link2_Link3ST.r = 
      Link2_Link3PP - (Link2_Link3Rot.transpose() * Link2_Link3CP);
    //4--------------------------------------------------------------------//
    Matrix3d Link3_Link4Rot = 
    Eigen::Matrix3d(Eigen::Quaterniond::FromTwoVectors(Link3_Link4PA, Link3_Link4CA));
    Eigen::Affine3d Link3_Link4RotOffset(Eigen::AngleAxisd(Link3_Link4OffsetQ, Vector3d::UnitZ()));
      
    SpatialTransform Link3_Link4ST;
    Link3_Link4ST.E = Link3_Link4Rot.transpose() * Link3_Link4RotOffset.rotation();
    Link3_Link4ST.r = 
      Link3_Link4PP - (Link3_Link4Rot.transpose() * Link3_Link4CP);
    //5--------------------------------------------------------------------//
    Matrix3d Link4_Link5Rot = 
    Eigen::Matrix3d(Eigen::Quaterniond::FromTwoVectors(Link4_Link5PA, Link4_Link5CA));
    Eigen::Affine3d Link4_Link5RotOffset(Eigen::AngleAxisd(Link4_Link5OffsetQ, Vector3d::UnitZ()));
      
    SpatialTransform Link4_Link5ST;
    Link4_Link5ST.E = Link4_Link5Rot.transpose() * Link4_Link5RotOffset.rotation();
    Link4_Link5ST.r = 
      Link4_Link5PP - (Link4_Link5Rot.transpose() * Link4_Link5CP);
    //6--------------------------------------------------------------------//
    Matrix3d Link5_Link6Rot = 
    Eigen::Matrix3d(Eigen::Quaterniond::FromTwoVectors(Link5_Link6PA, Link5_Link6CA));
    Eigen::Affine3d Link5_Link6RotOffset(Eigen::AngleAxisd(Link5_Link6OffsetQ, Vector3d::UnitZ()));
      
    SpatialTransform Link5_Link6ST;
    Link5_Link6ST.E = Link5_Link6Rot.transpose() * Link5_Link6RotOffset.rotation();
    Link5_Link6ST.r = 
      Link5_Link6PP - (Link5_Link6Rot.transpose() * Link5_Link6CP);
    //7--------------------------------------------------------------------//
    Matrix3d Link6_Link7Rot = 
    Eigen::Matrix3d(Eigen::Quaterniond::FromTwoVectors(Link6_Link7PA, Link6_Link7CA));
    Eigen::Affine3d Link6_Link7RotOffset(Eigen::AngleAxisd(Link6_Link7OffsetQ, Vector3d::UnitZ()));
      
    SpatialTransform Link6_Link7ST;
    Link6_Link7ST.E = Link6_Link7Rot.transpose() * Link6_Link7RotOffset.rotation();
    Link6_Link7ST.r = 
      Link6_Link7PP - (Link6_Link7Rot.transpose() * Link6_Link7CP);
    //--------------------------------------------------------------------//
    world_Link1ST = world_baseST * base_Link1ST;
    world_Link2ST = world_Link1ST * Link1_Link2ST;
    world_Link3ST = world_Link2ST * Link2_Link3ST;
    world_Link4ST = world_Link3ST * Link3_Link4ST;
    world_Link5ST = world_Link4ST * Link4_Link5ST;
    world_Link6ST = world_Link5ST * Link5_Link6ST;
    world_Link7ST = world_Link6ST * Link6_Link7ST;
    //--------------------------------------------------------------------//
    const Vector3d p_base_Link1_world  = world_baseST.E * base_Link1ST.r;
    const Vector3d p_Link1_Link2_world = world_Link1ST.E * Link1_Link2ST.r;
    const Vector3d p_Link2_Link3_world = world_Link2ST.E * Link2_Link3ST.r;
    const Vector3d p_Link3_Link4_world = world_Link3ST.E * Link3_Link4ST.r;
    const Vector3d p_Link4_Link5_world = world_Link4ST.E * Link4_Link5ST.r;
    const Vector3d p_Link5_Link6_world = world_Link5ST.E * Link5_Link6ST.r;
    const Vector3d p_Link6_Link7_world = world_Link6ST.E * Link6_Link7ST.r;
    //--------------------------------------------------------------------//
    const Vector3d base_Link1JAxis = world_baseST.E * base_Link1PA;
    const Vector3d Link1_Link2JAxis = base_Link1ST.E * Link1_Link2PA;
    const Vector3d Link2_Link3JAxis = Link1_Link2ST.E * Link2_Link3PA;
    const Vector3d Link3_Link4JAxis = Link2_Link3ST.E * Link3_Link4PA;
    const Vector3d Link4_Link5JAxis = Link3_Link4ST.E * Link4_Link5PA;
    const Vector3d Link5_Link6JAxis = Link4_Link5ST.E * Link5_Link6PA;
    const Vector3d Link6_Link7JAxis = Link5_Link6ST.E * Link6_Link7PA;
    //--------------------------------------------------------------------//
    Joint joint_base = Joint(JointTypeFixed);
    
    Joint base_Link1J = Joint(SpatialVector ( 
    base_Link1JAxis(0), base_Link1JAxis(1), base_Link1JAxis(2), 0., 0., 0.));
    
    Joint Link1_Link2J = Joint(SpatialVector ( 
    Link1_Link2JAxis(0), Link1_Link2JAxis(1), Link1_Link2JAxis(2), 0., 0., 0.));
    
    Joint Link2_Link3J = Joint(SpatialVector ( 
    Link2_Link3JAxis(0), Link2_Link3JAxis(1), Link2_Link3JAxis(2), 0., 0., 0.));
    
    Joint Link3_Link4J = Joint(SpatialVector ( 
    Link3_Link4JAxis(0), Link3_Link4JAxis(1), Link3_Link4JAxis(2), 0., 0., 0.));
    
    Joint Link4_Link5J = Joint(SpatialVector ( 
    Link4_Link5JAxis(0), Link4_Link5JAxis(1), Link4_Link5JAxis(2), 0., 0., 0.));
    
    Joint Link5_Link6J = Joint(SpatialVector ( 
    Link5_Link6JAxis(0), Link5_Link6JAxis(1), Link5_Link6JAxis(2), 0., 0., 0.));
    
    Joint Link6_Link7J = Joint(SpatialVector ( 
    Link6_Link7JAxis(0), Link6_Link7JAxis(1), Link6_Link7JAxis(2), 0., 0., 0.));
    //--------------------------------------------------------------------//
	  world_baseId = model-> 
		AddBody(0, Xtrans(world_baseST.r), joint_base, baseBody, "world-base");

    base_Link1Id = model-> AddBody(world_baseId, Xtrans(p_base_Link1_world), 
		base_Link1J, Link1Body, "base_Link1");

    Link1_Link2Id = model-> AddBody(base_Link1Id, Xtrans(p_Link1_Link2_world), 
		Link1_Link2J, Link2Body, "Link1_Link2");

    Link2_Link3Id = model-> AddBody(Link1_Link2Id, Xtrans(p_Link2_Link3_world), 
		Link2_Link3J, Link3Body, "Link2_Link3");

    Link3_Link4Id = model-> AddBody(Link2_Link3Id, Xtrans(p_Link3_Link4_world), 
		Link3_Link4J, Link4Body, "Link3_Link4");

    Link4_Link5Id = model-> AddBody(Link3_Link4Id, Xtrans(p_Link4_Link5_world), 
		Link4_Link5J, Link5Body, "Link4_Link5");

    Link5_Link6Id = model-> AddBody(Link4_Link5Id, Xtrans(p_Link5_Link6_world), 
		Link5_Link6J, Link6Body, "Link5_Link6");

    Link6_Link7Id = model-> AddBody(Link5_Link6Id, Xtrans(p_Link6_Link7_world), 
		Link6_Link7J, Link7Body, "Link6_Link7");
    //--------------------------------------------------------------------//
    Q = VectorNd::Constant ((size_t) model->dof_count, 0.);
    QDot = VectorNd::Constant ((size_t) model->dof_count, 0.);
    QDDot = VectorNd::Constant ((size_t) model->dof_count, 0.);
    Tau = VectorNd::Constant ((size_t) model->dof_count, 0.);

    point_position = Vector3d::Zero (3);
    point_acceleration = Vector3d::Zero (3);

    std::map<std::string, unsigned int> rbdlBodyMap;
    std::map<std::string, unsigned int>::iterator rbdlBodyMapItr;
    rbdlBodyMap = model->mBodyNameMap;

    ClearLogOutput();
    for(rbdlBodyMapItr = rbdlBodyMap.begin(); 
        rbdlBodyMapItr != rbdlBodyMap.end(); 
        rbdlBodyMapItr++)
    {
      std::string bodyName = rbdlBodyMapItr->first;
      unsigned int bodyId = rbdlBodyMapItr->second;

      std::string parentName = model->GetBodyName(model->GetParentBodyId(bodyId));
      bool isFixedBody = model->IsFixedBodyId(bodyId);
      std::cout << parentName << ", " << bodyName    << ", " 
                << bodyId     << ", " << isFixedBody << std::endl;
    }
  }
  
  ~KUKA()
  {
    delete model;
  }

  RigidBodyDynamics::Model *model;
  Body worldBody, baseBody, Link1Body, Link2Body, Link3Body, 
    Link4Body, Link5Body, Link6Body, Link7Body;

  unsigned int worldId, world_baseId, 
  base_Link1Id, Link1_Link2Id, Link2_Link3Id, Link3_Link4Id, Link4_Link5Id, Link5_Link6Id, Link6_Link7Id, 
  Link7_eeId;

  SpatialTransform world_baseST, world_Link1ST, world_Link2ST, world_Link3ST, 
	world_Link4ST, world_Link5ST, world_Link6ST, world_Link7ST, world_eeST;

  const Vector3d base_Link1PA = { 00.000, 00.000, 01.000 };
  const Vector3d base_Link1CA = { 00.000, 00.000, 01.000 };
  const Vector3d base_Link1PP = { 00.000, 00.000, 00.103 };
  const Vector3d base_Link1CP = { 00.000, 00.000, 00.000 };

	const Vector3d Link1_Link2PA = { 00.000, 01.000, 00.000 };
	const Vector3d Link1_Link2CA = { 00.000, 00.000, 01.000 };
	const Vector3d Link1_Link2PP = { 00.000, 00.013, 00.209 };
	const Vector3d Link1_Link2CP = { 00.000, 00.000, 00.000 };

	const Vector3d Link2_Link3PA = { 00.000, -1.000, 00.000 };
	const Vector3d Link2_Link3CA = { 00.000, 00.000, 01.000 };
	const Vector3d Link2_Link3PP = { 00.000, -0.194, -0.009 };
	const Vector3d Link2_Link3CP = { 00.000, 00.000, 00.000 };

	const Vector3d Link3_Link4PA = { 00.000, -1.000, 00.000 };
	const Vector3d Link3_Link4CA = { 00.000, 00.000, 01.000 };
	const Vector3d Link3_Link4PP = { 00.000, -0.013, 00.202 };
	const Vector3d Link3_Link4CP = { 00.000, 00.000, 00.000 };

	const Vector3d Link4_Link5PA = { 00.000, 01.000, 00.000 };
	const Vector3d Link4_Link5CA = { 00.000, 00.000, 01.000 };
	const Vector3d Link4_Link5PP = { -0.002, 00.202, -0.008 };
	const Vector3d Link4_Link5CP = { 00.000, 00.000, 00.000 };
	
	const Vector3d Link5_Link6PA = { 00.000, 01.000, 00.000 };
	const Vector3d Link5_Link6CA = { 00.000, 00.000, 01.000 };
	const Vector3d Link5_Link6PP = { 00.002, -0.052, 00.204 };
	const Vector3d Link5_Link6CP = { 00.000, 00.000, 00.000 };
	
	const Vector3d Link6_Link7PA = { 00.000, -1.000, 00.000 };
	const Vector3d Link6_Link7CA = { 00.000, 00.000, 01.000 };
	const Vector3d Link6_Link7PP = { -0.003, -0.050, 00.053 };
	const Vector3d Link6_Link7CP = { 00.000, 00.000, 00.000 };

  const double ROOT_baseOffsetQ   = 0.0;
  const double base_Link1OffsetQ  = 0.0;
  const double Link1_Link2OffsetQ = 0.0;
  const double Link2_Link3OffsetQ = 0.0;
  const double Link3_Link4OffsetQ = 0.0;
  const double Link4_Link5OffsetQ = 0.0;
  const double Link5_Link6OffsetQ = 0.0;
  const double Link6_Link7OffsetQ = 0.0;
	const double Link7_eeOffsetQ    = 0.0;

  RigidBodyDynamics::Math::VectorNd Q;
  RigidBodyDynamics::Math::VectorNd QDot;
  RigidBodyDynamics::Math::VectorNd QDDot;
  RigidBodyDynamics::Math::VectorNd Tau;

  RigidBodyDynamics::Math::Vector3d point_position, point_acceleration;
};
