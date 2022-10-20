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
    link1Body = Body (1., Vector3d (00.000, -0.017, 0.134), Vector3d (0.0452, 0.0446, 0.0041));
    link2Body = Body (1., Vector3d (00.000, -0.074, 0.009), Vector3d (0.0227, 0.0037, 0.0224));
    link3Body = Body (1., Vector3d (00.000, 00.017, 0.134), Vector3d (0.0417, 0.0418, 0.0038));
    link4Body = Body (1., Vector3d (-0.001, 00.081, 0.008), Vector3d (0.0249, 0.0036, 0.0247));
    link5Body = Body (1., Vector3d (0.0000, -0.017, 0.129), Vector3d (0.0363, 0.0350, 0.0045));
    link6Body = Body (1., Vector3d (0.0000, 00.007, 0.068), Vector3d (0.0114, 0.0116, 0.0037));
    link7Body = Body (1., Vector3d (0.0060, 00.000, 0.015), Vector3d (0.0012, 0.0013, 0.0010));

    //0---------------------------------------------------------------------//
    world_baseST.E.setIdentity();
    world_baseST.r = Vector3d(0.0, 0.0, -1.3);
    //1--------------------------------------------------------------------//
    Matrix3d base_link1Rot = 
    Eigen::Matrix3d(Eigen::Quaterniond::FromTwoVectors(base_link1PA, base_link1CA));
    Eigen::Affine3d base_link1RotOffset(Eigen::AngleAxisd(base_link1OffsetQ, Vector3d::UnitZ()));
      
    SpatialTransform base_link1ST;
    base_link1ST.E = base_link1Rot.transpose() * base_link1RotOffset.rotation();
    base_link1ST.r = 
      base_link1PP - (base_link1Rot.transpose() * base_link1CP);
    //2--------------------------------------------------------------------//
    Matrix3d link1_link2Rot = 
    Eigen::Matrix3d(Eigen::Quaterniond::FromTwoVectors(link1_link2PA, link1_link2CA));
    Eigen::Affine3d link1_link2RotOffset(Eigen::AngleAxisd(link1_link2OffsetQ, Vector3d::UnitZ()));
      
    SpatialTransform link1_link2ST;
    link1_link2ST.E = link1_link2Rot.transpose() * link1_link2RotOffset.rotation();
    link1_link2ST.r = 
      link1_link2PP - (link1_link2Rot.transpose() * link1_link2CP);
    //3--------------------------------------------------------------------//
    Matrix3d link2_link3Rot = 
    Eigen::Matrix3d(Eigen::Quaterniond::FromTwoVectors(link2_link3PA, link2_link3CA));
    Eigen::Affine3d link2_link3RotOffset(Eigen::AngleAxisd(link2_link3OffsetQ, Vector3d::UnitZ()));
      
    SpatialTransform link2_link3ST;
    link2_link3ST.E = link2_link3Rot.transpose() * link2_link3RotOffset.rotation();
    link2_link3ST.r = 
      link2_link3PP - (link2_link3Rot.transpose() * link2_link3CP);
    //4--------------------------------------------------------------------//
    Matrix3d link3_link4Rot = 
    Eigen::Matrix3d(Eigen::Quaterniond::FromTwoVectors(link3_link4PA, link3_link4CA));
    Eigen::Affine3d link3_link4RotOffset(Eigen::AngleAxisd(link3_link4OffsetQ, Vector3d::UnitZ()));
      
    SpatialTransform link3_link4ST;
    link3_link4ST.E = link3_link4Rot.transpose() * link3_link4RotOffset.rotation();
    link3_link4ST.r = 
      link3_link4PP - (link3_link4Rot.transpose() * link3_link4CP);
    //5--------------------------------------------------------------------//
    Matrix3d link4_link5Rot = 
    Eigen::Matrix3d(Eigen::Quaterniond::FromTwoVectors(link4_link5PA, link4_link5CA));
    Eigen::Affine3d link4_link5RotOffset(Eigen::AngleAxisd(link4_link5OffsetQ, Vector3d::UnitZ()));
      
    SpatialTransform link4_link5ST;
    link4_link5ST.E = link4_link5Rot.transpose() * link4_link5RotOffset.rotation();
    link4_link5ST.r = 
      link4_link5PP - (link4_link5Rot.transpose() * link4_link5CP);
    //6--------------------------------------------------------------------//
    Matrix3d link5_link6Rot = 
    Eigen::Matrix3d(Eigen::Quaterniond::FromTwoVectors(link5_link6PA, link5_link6CA));
    Eigen::Affine3d link5_link6RotOffset(Eigen::AngleAxisd(link5_link6OffsetQ, Vector3d::UnitZ()));
      
    SpatialTransform link5_link6ST;
    link5_link6ST.E = link5_link6Rot.transpose() * link5_link6RotOffset.rotation();
    link5_link6ST.r = 
      link5_link6PP - (link5_link6Rot.transpose() * link5_link6CP);
    //7--------------------------------------------------------------------//
    Matrix3d link6_link7Rot = 
    Eigen::Matrix3d(Eigen::Quaterniond::FromTwoVectors(link6_link7PA, link6_link7CA));
    Eigen::Affine3d link6_link7RotOffset(Eigen::AngleAxisd(link6_link7OffsetQ, Vector3d::UnitZ()));
      
    SpatialTransform link6_link7ST;
    link6_link7ST.E = link6_link7Rot.transpose() * link6_link7RotOffset.rotation();
    link6_link7ST.r = 
      link6_link7PP - (link6_link7Rot.transpose() * link6_link7CP);
    //--------------------------------------------------------------------//
    world_link1ST = world_baseST * base_link1ST;
    world_link2ST = world_link1ST * link1_link2ST;
    world_link3ST = world_link2ST * link2_link3ST;
    world_link4ST = world_link3ST * link3_link4ST;
    world_link5ST = world_link4ST * link4_link5ST;
    world_link6ST = world_link5ST * link5_link6ST;
    world_link7ST = world_link6ST * link6_link7ST;
    //--------------------------------------------------------------------//
    Vector3d base_link1JAxis = world_baseST.E * base_link1PA;
    Vector3d link1_link2JAxis = base_link1ST.E * link1_link2PA;
    Vector3d link2_link3JAxis = link1_link2ST.E * link2_link3PA;
    Vector3d link3_link4JAxis = link2_link3ST.E * link3_link4PA;
    Vector3d link4_link5JAxis = link3_link4ST.E * link4_link5PA;
    Vector3d link5_link6JAxis = link4_link5ST.E * link5_link6PA;
    Vector3d link6_link7JAxis = link5_link6ST.E * link6_link7PA;
    
    
    Q = VectorNd::Constant ((size_t) model->dof_count, 0.);
    QDot = VectorNd::Constant ((size_t) model->dof_count, 0.);
    QDDot = VectorNd::Constant ((size_t) model->dof_count, 0.);
    Tau = VectorNd::Constant ((size_t) model->dof_count, 0.);

    point_position = Vector3d::Zero (3);
    point_acceleration = Vector3d::Zero (3);

    ClearLogOutput();
  }
  
  ~KUKA()
  {
    delete model;
  }

  RigidBodyDynamics::Model *model;
  Body worldBody, baseBody, link1Body, link2Body, link3Body, 
    link4Body, link5Body, link6Body, link7Body;

  unsigned int worldId, world_baseId, 
  base_link1Id, link1_link2Id, link2_link3Id, link3_link4Id, link4_link5Id, link5_link6Id, link6_link7Id, 
  link7_eeId;

  SpatialTransform world_baseST, world_link1ST, world_link2ST, world_link3ST, 
	world_link4ST, world_link5ST, world_link6ST, world_link7ST, world_eeST;

  const Vector3d base_link1PA = { 00.000, 00.000, 01.000 };
  const Vector3d base_link1CA = { 00.000, 00.000, 01.000 };
  const Vector3d base_link1PP = { 00.000, 00.000, 00.103 };
  const Vector3d base_link1CP = { 00.000, 00.000, 00.000 };

	const Vector3d link1_link2PA = { 00.000, 01.000, 00.000 };
	const Vector3d link1_link2CA = { 00.000, 00.000, 01.000 };
	const Vector3d link1_link2PP = { 00.000, 00.013, 00.209 };
	const Vector3d link1_link2CP = { 00.000, 00.000, 00.000 };

	const Vector3d link2_link3PA = { 00.000, -1.000, 00.000 };
	const Vector3d link2_link3CA = { 00.000, 00.000, 01.000 };
	const Vector3d link2_link3PP = { 00.000, -0.194, -0.009 };
	const Vector3d link2_link3CP = { 00.000, 00.000, 00.000 };

	const Vector3d link3_link4PA = { 00.000, -1.000, 00.000 };
	const Vector3d link3_link4CA = { 00.000, 00.000, 01.000 };
	const Vector3d link3_link4PP = { 00.000, -0.013, 00.202 };
	const Vector3d link3_link4CP = { 00.000, 00.000, 00.000 };

	const Vector3d link4_link5PA = { 00.000, 01.000, 00.000 };
	const Vector3d link4_link5CA = { 00.000, 00.000, 01.000 };
	const Vector3d link4_link5PP = { -0.002, 00.202, -0.008 };
	const Vector3d link4_link5CP = { 00.000, 00.000, 00.000 };
	
	const Vector3d link5_link6PA = { 00.000, 01.000, 00.000 };
	const Vector3d link5_link6CA = { 00.000, 00.000, 01.000 };
	const Vector3d link5_link6PP = { 00.002, -0.052, 00.204 };
	const Vector3d link5_link6CP = { 00.000, 00.000, 00.000 };
	
	const Vector3d link6_link7PA = { 00.000, -1.000, 00.000 };
	const Vector3d link6_link7CA = { 00.000, 00.000, 01.000 };
	const Vector3d link6_link7PP = { -0.003, -0.050, 00.053 };
	const Vector3d link6_link7CP = { 00.000, 00.000, 00.000 };

  const double ROOT_baseOffsetQ   = 0.0;
  const double base_link1OffsetQ  = 0.0;
  const double link1_link2OffsetQ = 0.0;
  const double link2_link3OffsetQ = 0.0;
  const double link3_link4OffsetQ = 0.0;
  const double link4_link5OffsetQ = 0.0;
  const double link5_link6OffsetQ = 0.0;
  const double link6_link7OffsetQ = 0.0;
	const double link7_eeOffsetQ    = 0.0;

  RigidBodyDynamics::Math::VectorNd Q;
  RigidBodyDynamics::Math::VectorNd QDot;
  RigidBodyDynamics::Math::VectorNd QDDot;
  RigidBodyDynamics::Math::VectorNd Tau;

  RigidBodyDynamics::Math::Vector3d point_position, point_acceleration;
};
