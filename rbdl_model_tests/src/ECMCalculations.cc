#include <iostream>

#include "rbdl/rbdl_mathutils.h"
#include "rbdl/Logging.h"

#include "rbdl/Model.h"
#include "rbdl/Kinematics.h"
#include "rbdl/Dynamics.h"
#include "rbdl_model_tests/rbdl_tests.h"

using namespace std;
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

// const double TEST_PREC = 1.0e-2;
const double TEST_LAX = 1.0e-7;

struct ECM {
  ECM () {
    ClearLogOutput();
    model = new Model;

    body = Body (1., Vector3d (1., 0., 0.), Vector3d (1., 1., 1.));
    
    // 0 - works
    Matrix3d r_w_base;
    r_w_base.setIdentity();
    Vector3d p_w_base(0.4999, -0.3901, -0.599);
    // Vector3d p_w_base(0, -0, -0); 
    Vector3d p_w_base_w = r_w_base * p_w_base;
    joint_base = 
    Joint(JointTypeFixed);
    // Joint( SpatialVector (0., 0., 0., 0., 0., 0.)); 
    // affects yawlink pose

    body_base_id = 
      model->AddBody(0, Xtrans(p_w_base_w), joint_base, body, "world-baselink");
    // Handles World to baselink rotation. Not a joint

    // 1 - works
    // Matrix3d r_w_yaw(0, 1, 0., 0., 0, 1, 0, 1, 0);
    Matrix3d r_w_yaw(-1, 0, 0, 0, 0, 1, 0, 1, 0);
    Vector3d p_base_yaw(0, -0.5369, 0);
    Vector3d p_base_yaw_w = r_w_base * p_base_yaw;
    joint_yaw = 
    Joint( SpatialVector (0., -1., 0., 0., 0., 0.)); 
    // affects pitchbacklink pose

    // Yaw-pitchback along word X
    body_yaw_id = 
      model->AddBody(body_base_id, Xtrans(p_base_yaw_w), joint_yaw, body, "baselink-yawlink");

    // 2- tbd
    Matrix3d r_w_pitchback(0., 0, -1, 1, 0, 0., 0, -1, 0);
    Vector3d p_yaw_pitchback(0, -0.0098, 0.1624);
    Vector3d p_yaw_pitchback_w = r_w_yaw * p_yaw_pitchback;
    joint_pitchback = Joint( SpatialVector (-1., 0., 0., 0., 0., 0.)); 
    // affects bottomlinklink pose

    body_pitchback_id = 
      model->AddBody(body_yaw_id, Xtrans(p_yaw_pitchback_w), 
        joint_pitchback, body, "yawlink-pitchbacklink");
    // pitchbacklink pose

    // 3 - tbd
    Matrix3d r_w_pitchbottom(0, 0, -1, 1, 0, 0, 0, -1, 0);
    Vector3d p_pitchback_pitchbottom(-0.0664, -0.2965, 0.0005);


    Vector3d p_pitchback_pitchbottom_w = r_w_pitchback * p_pitchback_pitchbottom;
    
    joint_pitchbottom = 
    Joint( SpatialVector (-1., 0., 0., 0., 0., 0.));
    // affects pitchendlink pose

    body_pitchbottom_id = 
      model->AddBody(body_pitchback_id, Xtrans(p_pitchback_pitchbottom_w), 
        joint_pitchbottom, body, "pitchbacklink-pitchbottomlink");

    // 4 - 
    Matrix3d r_w_pitchend(0, 0, -1, 1, 0, 0, 0, -1, 0);
    Vector3d p_pitchbottom_pitchend(0.3401, -0.0001, -0.0006);
    Vector3d p_pitchbottom_pitchend_w = r_w_pitchbottom * p_pitchbottom_pitchend;
    joint_pitchend = Joint( SpatialVector (-1., 0., 0., 0., 0., 0.));
    // affects yawlink pose

    body_pitchend_id = 
      model->AddBody(body_pitchbottom_id, Xtrans(p_pitchbottom_pitchend_w), 
        joint_pitchend, body, "pitchbottomlink-pitchendlink");

    // 5 - works
    Matrix3d r_w_maininsertion(0, 0, -1, 0, -1, 0, -1, 0, 0);
    Vector3d p_pitchend_maininsertion(0.0411, -0.0853, -0.0001);
    Vector3d p_pitchend_maininsertion_w = r_w_pitchend * p_pitchend_maininsertion;
    joint_maininsertion = Joint( SpatialVector (0., 0., 0., 0., 0., -1.));
    body_maininsertion_id = 
      model->AddBody(body_pitchend_id, Xtrans(p_pitchend_maininsertion_w), 
        joint_maininsertion, body, "pitchendlink-maininsertionlink");
    // affects maininsertionlink pose

    // 6 - 
    Matrix3d r_w_tool(0, 1, 0, -1, 0, 0, 0, 0, 1);
    Vector3d p_maininsertion_tool(0.001, -0.0618, 0.0001);
    Vector3d p_maininsertion_tool_w = r_w_maininsertion * p_maininsertion_tool;
    joint_tool = Joint( SpatialVector (0., 0., 1., 0., 0., 0.));
    body_tool_id = 
      model->AddBody(body_maininsertion_id, Xtrans(p_maininsertion_tool_w), 
        joint_tool, body, "maininsertionlink-toollink");
    // affects toollink pose

    // std::cout << "r_w_base"          << std::endl << r_w_base          << std::endl;
    // std::cout << "r_w_yaw"           << std::endl << r_w_yaw           << std::endl;
    // std::cout << "r_w_pitchback"     << std::endl << r_w_pitchback     << std::endl;
    // std::cout << "r_w_pitchbottom"   << std::endl << r_w_pitchbottom   << std::endl;
    // std::cout << "r_w_pitchend"      << std::endl << r_w_pitchend      << std::endl;
    // std::cout << "r_w_maininsertion" << std::endl << r_w_maininsertion << std::endl;
    // std::cout << "r_w_tool"          << std::endl << r_w_tool          << std::endl;
    // bool bgStab=true;
    // unsigned int userDefinedId = 7;
    // cs.AddLoopConstraint(idB2,idB5,X_p,X_s,SpatialVector(0,0,0,1,0,0),bgStab,0.1,
    //                      "LoopXY_Rz",userDefinedId);

    // //These two constraints below will be appended to the above
    // //constraint by default, and will assume its name and user defined id
    // cs.AddLoopConstraint(idB2,idB5,X_p,X_s,SpatialVector(0,0,0,0,1,0));
    // cs.AddLoopConstraint(idB2,idB5,X_p,X_s,SpatialVector(0,0,1,0,0,0));

    // cs.Bind(model);


    Q = VectorNd::Constant ((size_t) model->dof_count, 0.);
    QDot = VectorNd::Constant ((size_t) model->dof_count, 0.);
    QDDot = VectorNd::Constant ((size_t) model->dof_count, 0.);
    Tau = VectorNd::Constant ((size_t) model->dof_count, 0.);

    ClearLogOutput();
    Q.setZero();

    rbdlmBodyMap = model->mBodyNameMap;
    
    ClearLogOutput();

    for(unsigned int bodyId = 0; bodyId <= model->q_size; bodyId++)
    {
      std::string bodyName = model->GetBodyName(bodyId);
      // unsigned int bodyId = rbdlmBodyMapItr->second;
      std::string parentName = model->GetBodyName(model->GetParentBodyId(bodyId));
      std::cout << parentName << ", " << bodyName << ", " << bodyId << std::endl;
    }    

    // Q[0] = M_PI_4; // not activated
    

    // double qyawlink_pitchbacklink = M_PI_4;
    /***
     * Modifying yawlink_pitchbacklink affects angles of four links
     * Q[1] = M_PI_4; // yawlink-pitchbacklink
     * Q[2] = -M_PI_4; // pitchbacklink-pitchbottomlink
     * Q[3] = M_PI_4; // pitchbottomlink-pitchendlink
     * Q[4] = M_PI_4; // pitchendlink-maininsertionlink
     ***/
    Q[0] = 0.7872554063796997; // baselink-yawlink
    // Q[0] = M_PI_4;
    // Q[2] = M_PI_4; // yawlink-pitchbacklink
    // Q[3] = -0.7823343276977539; // pitchbacklink-pitchbottomlink
    // Q[4] = M_PI_4; // pitchbottomlink-pitchendlink

    // Q[5] = 0.25;  // pitchendlink-maininsertionlink
    // Q[4] = 0.25;  // pitchendlink-maininsertionlink
    // Q[6] = M_PI_4;  // maininsertionlink-toollink



  std::cout << "Q" << std::endl << Q << std::endl << std::endl;

  // Vector3d p_w_base_body(0, 0, 0);
  // Vector3d p_base_yaw_body(-0.00020738, -0.5369, 0);
  // Vector3d p_yaw_pitchback_body(0, -0.0098, 0.1624);
  // Vector3d p_pitchback_pitchbottom_body(-0.0664, -0.2965, 0.0005);

  // // Matrix3d r_w_base;
  // // r_w_base.setIdentity();

  // p_w_base_calc = p_w_base_body;

  // Eigen::Affine3d r_yaw(
  //   Eigen::AngleAxisd(Q[1], -Vector3d::UnitY()));

  // p_w_yaw_calc = 
  //   p_w_base_calc + r_yaw * r_w_base * p_base_yaw_body; 


  // p_w_pitchback_calc =  Vector3d(0,    -0.37436, -0.00970039);
  
  // Eigen::Affine3d r_pitchbottom_world(Eigen::AngleAxisd(M_PI_2, -Vector3d::UnitX()));


  // std::cout << "r_w_pitchback" << std::endl << r_w_pitchback << std::endl;
  // Matrix3d B = r_w_pitchback.transpose() * r_pitchbottom_world.rotation() * r_w_pitchback;
  // std::cout << "B" << std::endl << B << std::endl;


  // r_pitchbottom.rotation() * r_w_pitchback 
  // p_w_pitchbottom_calc = r_w_pitchback * p_pitchback_pitchbottom_body;
  // p_w_pitchbottom_calc = r_pitchbottom.rotation() * p_w_pitchbottom_calc;
  // p_w_pitchbottom_calc += p_w_pitchback_calc;

  // Vector3d p_w_pback_w(0,   -0.374506, -0.00980314);

  // // CHECK_THAT (p_w_pback_w, 
  // //   AllCloseVector(p_w_pback_w_cal, TEST_PREC, TEST_PREC));


  r_w_base_rbdl = CalcBodyWorldOrientation(*model, Q, body_base_id, true);
  p_w_base_rbdl = 
    CalcBodyToBaseCoordinates(*model, Q, body_base_id, Vector3d (0., 0., 0.), true);

  r_w_yaw_rbdl = CalcBodyWorldOrientation(*model, Q, body_yaw_id, true);
  p_w_yaw_rbdl = 
    CalcBodyToBaseCoordinates(*model, Q, body_yaw_id, Vector3d (0., 0., 0.), true);

  r_w_pitchback_rbdl = CalcBodyWorldOrientation(*model, Q, body_pitchback_id, true);
  p_w_pitchback_rbdl = 
    CalcBodyToBaseCoordinates(*model, Q, body_pitchback_id, Vector3d (0., 0., 0.), true);

  r_w_pitchbottom_rbdl = CalcBodyWorldOrientation(*model, Q, body_pitchbottom_id, true);
  p_w_pitchbottom_rbdl = 
    CalcBodyToBaseCoordinates(*model, Q, body_pitchbottom_id, Vector3d (0., 0., 0.), true);

  r_w_pitchend_rbdl = CalcBodyWorldOrientation(*model, Q, body_pitchend_id, true);
  p_w_pitchend_rbdl = 
    CalcBodyToBaseCoordinates(*model, Q, body_pitchend_id, Vector3d (0., 0., 0.), true);

  r_w_maininsertion_rbdl = CalcBodyWorldOrientation(*model, Q, body_maininsertion_id, true);
  p_w_maininsertion_rbdl = 
    CalcBodyToBaseCoordinates(*model, Q, body_maininsertion_id, Vector3d (0., 0., 0.), true);

  r_w_tool_rbdl = CalcBodyWorldOrientation(*model, Q, body_tool_id, true);
  p_w_tool_rbdl = 
    CalcBodyToBaseCoordinates(*model, Q, body_tool_id, Vector3d (0., 0., 0.), true);

  std::cout << "p_w_base_rbdl" << std::endl << p_w_base_rbdl << std::endl;
  std::cout << "p_w_yaw_rbdl" << std::endl << p_w_yaw_rbdl << std::endl;
  std::cout << "p_w_pitchback_rbdl" << std::endl << p_w_pitchback_rbdl << std::endl;
  std::cout << "p_w_pitchbottom_rbdl" << std::endl << p_w_pitchbottom_rbdl << std::endl;
  std::cout << "p_w_pitchend_rbdl" << std::endl << p_w_pitchend_rbdl << std::endl;
  std::cout << "p_w_maininsertion_rbdl" << std::endl << p_w_maininsertion_rbdl << std::endl;
  std::cout << "p_w_tool_rbdl" << std::endl << p_w_tool_rbdl << std::endl;
  }

  ~ECM () {
    delete model;
  }
  Model *model;
  // ConstraintSet cs;

  unsigned int body_world_id, body_base_id, body_yaw_id, body_pitchback_id, 
    body_pitchbottom_id, body_pitchend_id, body_maininsertion_id, body_tool_id;
  
  Body body;
  Joint joint_world, joint_base, joint_yaw, joint_pitchback, joint_pitchbottom, 
    joint_pitchend, joint_maininsertion, joint_tool;

  VectorNd Q;
  VectorNd QDot;
  VectorNd QDDot;
  VectorNd Tau;

  std::map< std::string, unsigned int > rbdlmBodyMap;
  std::map<std::string, unsigned int>::iterator rbdlmBodyMapItr;

  Matrix3d r_w_base_rbdl, r_w_yaw_rbdl, r_w_pitchback_rbdl, r_w_pitchbottom_rbdl,
    r_w_pitchend_rbdl, r_w_maininsertion_rbdl, r_w_tool_rbdl;
  Vector3d p_w_base_rbdl, p_w_yaw_rbdl, p_w_pitchback_rbdl, p_w_pitchbottom_rbdl,
    p_w_pitchend_rbdl, p_w_maininsertion_rbdl, p_w_tool_rbdl;
  Vector3d p_w_base_calc, p_w_yaw_calc, p_w_pitchback_calc, p_w_pitchbottom_calc;
};



TEST_CASE_METHOD ( ECM, __FILE__"_baselink_yawlink_roation", "") 
{
  // 1 - Test case for baselink-yawlink pi/4
  CHECK_THAT (Vector3d (0.4999,     -0.3901,      -0.599), 
    AllCloseVector(p_w_base_rbdl, TEST_PREC, TEST_PREC));

  CHECK_THAT (Vector3d (0.49987,      -0.927,   -0.598924), 
    AllCloseVector(p_w_yaw_rbdl, TEST_PREC, TEST_PREC));

  CHECK_THAT (Vector3d (0.506903,   -0.764604,   -0.605786), 
    AllCloseVector(p_w_pitchback_rbdl, TEST_PREC, TEST_PREC));

  CHECK_THAT (Vector3d (0.295787,   -0.831097,   -0.397694), 
    AllCloseVector(p_w_pitchbottom_rbdl, TEST_PREC, TEST_PREC));

  CHECK_THAT (Vector3d (0.295218,   -0.491005,    -0.39832), 
    AllCloseVector(p_w_pitchend_rbdl, TEST_PREC, TEST_PREC));

  CHECK_THAT (Vector3d (0.23481,   -0.449937,   -0.339737), 
    AllCloseVector(p_w_maininsertion_rbdl, TEST_PREC, TEST_PREC));

  CHECK_THAT (Vector3d (0.235425,   -0.388035,   -0.340503), 
    AllCloseVector(p_w_tool_rbdl, TEST_PREC, TEST_PREC));  

  // 1 - Test case for baselink-yawlink pi/4 - zero offset
//   CHECK_THAT (Vector3d (0, 0, 0), 
//     AllCloseVector(p_w_base_calc, TEST_PREC, TEST_PREC));

//   CHECK_THAT (Vector3d (0, -0.536901, 0.), 
//     AllCloseVector(p_w_yaw_calc, TEST_PREC, TEST_PREC));

//   CHECK_THAT (Vector3d (0.00692918,   -0.374517, -0.00685648), 
//     AllCloseVector(p_w_pitchback_calc, TEST_PREC, TEST_PREC));

//   CHECK_THAT (Vector3d (-0.203704,   -0.441149,    0.201671), 
//     AllCloseVector(p_w_pitchbottom_calc, TEST_PREC, TEST_PREC));

//   CHECK_THAT (Vector3d (0.295218,   -0.491005,    -0.39832), 
//     AllCloseVector(p_w_pitchend, TEST_PREC, TEST_PREC));

//   CHECK_THAT (Vector3d (0.23481,   -0.449937,   -0.339737), 
//     AllCloseVector(p_w_maininsertion, TEST_PREC, TEST_PREC));

//   CHECK_THAT (Vector3d (0.235425,   -0.388035,   -0.340503), 
//     AllCloseVector(p_w_tool, TEST_PREC, TEST_PREC));  
}


// TEST_CASE_METHOD(ECM, __FILE__"_Testyawlink-pitchbacklink1", "") {
//   // pi/4
  
//   // Test cases for yawlink-pitchbacklink pi/4
//   CHECK_THAT (Vector3d (0.4999, -0.3901, -0.599), 
//     AllCloseVector(p_w_base_rbdl, TEST_PREC, TEST_PREC));

//   CHECK_THAT (Vector3d (0.499893, -0.926996, -0.598989), 
//     AllCloseVector(p_w_yaw_rbdl, TEST_PREC, TEST_PREC));

//   CHECK_THAT (Vector3d (0.499923,   -0.764565,   -0.608806), 
//     AllCloseVector(p_w_pitchback_rbdl, TEST_PREC, TEST_PREC));

//   CHECK_THAT (Vector3d (0.499461,    -0.59706,   -0.324224), 
//     AllCloseVector(p_w_pitchbottom_rbdl, TEST_PREC, TEST_PREC));

//   CHECK_THAT (Vector3d (0.500149,   -0.256883,   -0.325803), 
//     AllCloseVector(p_w_pitchend_rbdl, TEST_PREC, TEST_PREC));

//   CHECK_THAT (Vector3d (0.500094,   -0.167828,   -0.295689), 
//     AllCloseVector(p_w_maininsertion_rbdl, TEST_PREC, TEST_PREC));

//   CHECK_THAT (Vector3d (0.500301,   -0.125064,   -0.340443), 
//     AllCloseVector(p_w_tool_rbdl, TEST_PREC, TEST_PREC));
  // #################################################
  
   
  // Test cases for yawlink-pitchbacklink pi/4 - 0 offset
  // CHECK_THAT (Vector3d (0, 0, 0), 
  //   AllCloseVector(p_w_base_calc, TEST_PREC, TEST_PREC));

  // CHECK_THAT (Vector3d (0,   -0.536901, 0), 
  //   AllCloseVector(p_w_yaw_calc, TEST_PREC, TEST_PREC));
  
  // // CHECK_THAT (Vector3d (0,   -0.374506, -0.00980314) 
  // CHECK_THAT (Vector3d (0,   -0.374479, -0.00974497), 
  //   AllCloseVector(p_w_pitchback_calc, TEST_PREC, TEST_PREC));

  // CHECK_THAT (Vector3d (-0.000444501,   -0.206259,    0.274546), 
  //   AllCloseVector(p_w_pitchbottom_calc, TEST_PREC, TEST_PREC));

  // CHECK_THAT (Vector3d (0.000247019,    0.133217,    0.273197), 
  //   AllCloseVector(p_w_pitchend, TEST_PREC, TEST_PREC));

  // CHECK_THAT (Vector3d (0.000191259,    0.222273,    0.303311), 
  //   AllCloseVector(p_w_maininsertion, TEST_PREC, TEST_PREC));

  // CHECK_THAT (Vector3d (0.000399107,    0.265037,    0.258557), 
    // AllCloseVector(p_w_tool, TEST_PREC, TEST_PREC)); 
  // #################################################
// }

// TEST_CASE_METHOD(ECM, __FILE__"_Testmaininsertionlink-toollink", "") {
//   // We call ForwardDynamics() as it updates the spatial transformation
//   // matrices
//   ForwardDynamics(*model, Q, QDot, Tau, QDDot);

//   // 3 - Test case for maininsertionlink-toollink 0.25 - matches
//   CHECK_THAT (Vector3d (0.4999,     -0.3901,      -0.599), 
//     AllCloseVector(p_w_base_rbdl, TEST_PREC, TEST_PREC));

//   CHECK_THAT (Vector3d (0.499893,   -0.927001,   -0.599002), 
//     AllCloseVector(p_w_yaw_rbdl, TEST_PREC, TEST_PREC));

//   CHECK_THAT (Vector3d (0.499926,   -0.764606,   -0.608803), 
//     AllCloseVector(p_w_pitchback_rbdl, TEST_PREC, TEST_PREC));

//   CHECK_THAT (Vector3d (0.499413,   -0.831198,   -0.312372), 
//     AllCloseVector(p_w_pitchbottom_rbdl, TEST_PREC, TEST_PREC));

//   CHECK_THAT (Vector3d (0.500074,   -0.491109,   -0.312298), 
//     AllCloseVector(p_w_pitchend_rbdl, TEST_PREC, TEST_PREC));

//   CHECK_THAT (Vector3d (0.500006,   -0.449879,   -0.478593), 
//     AllCloseVector(p_w_maininsertion_rbdl, TEST_PREC, TEST_PREC));

//   CHECK_THAT (Vector3d (0.500217,   -0.387976,   -0.479546), 
//     AllCloseVector(p_w_tool_rbdl, TEST_PREC, TEST_PREC));     
// }

// TEST_CASE_METHOD(ECM, __FILE__"_TestPositionNeutral", "") {
//   // We call ForwardDynamics() as it updates the spatial transformation
//   // matrices
//   ForwardDynamics(*model, Q, QDot, Tau, QDDot);


  
//   // 0 - Test case for home pose - Matches
//   CHECK_THAT (Vector3d (0.4999,     -0.3901,      -0.599), 
//     AllCloseVector(p_w_base, TEST_PREC, TEST_PREC));

//   CHECK_THAT (Vector3d (0.499893,   -0.927001,   -0.599002), 
//     AllCloseVector(p_w_yaw, TEST_PREC, TEST_PREC));

//   CHECK_THAT (Vector3d (0.499926,   -0.764606,   -0.608803), 
//     AllCloseVector(p_w_pitchback, TEST_PREC, TEST_PREC));

//   CHECK_THAT (Vector3d (0.499413,   -0.831198,   -0.312372), 
//     AllCloseVector(p_w_pitchbottom, TEST_PREC, TEST_PREC));

//   CHECK_THAT (Vector3d (0.500074,   -0.491109,   -0.312298), 
//     AllCloseVector(p_w_pitchend, TEST_PREC, TEST_PREC));

//   CHECK_THAT (Vector3d (0.499999,   -0.450071,   -0.228596), 
//     AllCloseVector(p_w_maininsertion, TEST_PREC, TEST_PREC));

//   CHECK_THAT (Vector3d (0.50021,   -0.388168,   -0.229549), 
//     AllCloseVector(p_w_tool, TEST_PREC, TEST_PREC));    
//   //#################################################
  
  
//   // 0.b - Test case for home pose - 0 offset - matches
//   CHECK_THAT (Vector3d (0, 0, 0), 
//     AllCloseVector(p_w_base_calc, TEST_PREC, TEST_PREC));

//   CHECK_THAT (Vector3d (0,   -0.536901, 0), 
//     AllCloseVector(p_w_yaw_calc, TEST_PREC, TEST_PREC));

//   CHECK_THAT (Vector3d (0,   -0.374506, -0.00980314), 
//     AllCloseVector(p_w_pitchback_calc, TEST_PREC, TEST_PREC));

//   CHECK_THAT (Vector3d (-0.000487329,   -0.441098,    0.286628), 
//     AllCloseVector(p_w_pitchbottom_calc, TEST_PREC, TEST_PREC));

//   // CHECK_THAT (Vector3d (0.000173091,   -0.101009,    0.286702), 
//   //   AllCloseVector(p_w_pitchend, TEST_PREC, TEST_PREC));

//   // CHECK_THAT (Vector3d (0,  -0.0599711,    0.370404), 
//   //   AllCloseVector(p_w_maininsertion, TEST_PREC, TEST_PREC));

//   // CHECK_THAT (Vector3d (0.000308897,  0.00193176,    0.369451), 
//   //   AllCloseVector(p_w_tool, TEST_PREC, TEST_PREC));    
//   // #################################################
// }



// TEST_CASE_METHOD ( ECM, __FILE__"_ManyBodyFullConstraints", "") 
// {
//   UpdateKinematicsCustom (*model, &Q, NULL, NULL);

//   Matrix3d r_w_base = CalcBodyWorldOrientation(*model, Q, body_base_id, true);
//   Vector3d p_w_base = 
//     CalcBodyToBaseCoordinates(*model, Q, body_base_id, Vector3d (0., 0., 0.), true);

//   Matrix3d r_w_yaw = CalcBodyWorldOrientation(*model, Q, body_yaw_id, true);
//   Vector3d p_w_yaw = 
//     CalcBodyToBaseCoordinates(*model, Q, body_yaw_id, Vector3d (0., 0., 0.), true);

//   Matrix3d r_w_pitchback = CalcBodyWorldOrientation(*model, Q, body_pitchback_id, true);
//   Vector3d p_w_pitchback = 
//     CalcBodyToBaseCoordinates(*model, Q, body_pitchback_id, Vector3d (0., 0., 0.), true);

//   // Matrix3d r_w_pitchbottom = CalcBodyWorldOrientation(*model, Q, body_pitchbottom_id, true);
//   Vector3d p_w_pitchbottom = Vector3d (-0.000470541,   -0.351093,    0.304278);
//     // CalcBodyToBaseCoordinates(*model, Q, body_pitchbottom_id, Vector3d (0., 0., 0.), true);

//   // std::cout << "p_w_base"        << std::endl << p_w_base        << std::endl;
//   // std::cout << "p_w_yaw"         << std::endl << p_w_yaw         << std::endl;
//   // std::cout << "p_w_pitchback"   << std::endl << p_w_pitchback   << std::endl;
//   // std::cout << "p_w_pitchbottom" << std::endl << p_w_pitchbottom << std::endl;

//   // std::cout << "r_w_base"        << std::endl << r_w_base        << std::endl;
//   // std::cout << "r_w_yaw"         << std::endl << r_w_yaw         << std::endl;
//   // std::cout << "r_w_pitchback"   << std::endl << r_w_pitchback   << std::endl;
//   // std::cout << "r_w_pitchbottom" << std::endl << r_w_pitchbottom << std::endl;
//   InverseKinematicsConstraintSet cs;
//   cs.AddFullConstraint (
//     body_base_id,
//     Vector3d (0., 0., 0.),
//     p_w_base,
//     r_w_base
//   );
//   cs.AddFullConstraint (
//     body_yaw_id,
//     Vector3d (0., 0., 0.),
//     p_w_yaw,
//     r_w_yaw
//   );
//   cs.AddFullConstraint (
//     body_pitchback_id,
//     Vector3d (0., 0., 0.),
//     p_w_pitchback,
//     r_w_pitchback
//   );
//   cs.AddPointConstraint (body_pitchbottom_id, Vector3d (0., 0., 0.),
//                          p_w_pitchbottom);
//   cs.step_tol = 1e-2;

//   // q.setZero();

//   VectorNd Qres (Q);

//   bool result = InverseKinematics (*model, Q, cs, Qres);

//   CHECK (result);
//   CHECK_THAT (0., IsClose(cs.error_norm, cs.step_tol, cs.step_tol));
//   std::cout << "Qres" << std::endl << Qres << std::endl;

//   // UpdateKinematicsCustom (*model, &qres, NULL, NULL);  

// }


// TEST_CASE_METHOD(ECM, __FILE__"_TestPositionNeutral", "") {
//   // We call ForwardDynamics() as it updates the spatial transformation
//   // matrices
//   ForwardDynamics(*model, Q, QDot, Tau, QDDot);

// //   Vector3d body_position;

//   Matrix3d r_w_base = CalcBodyWorldOrientation(*model, Q, body_base_id, true);
//   Vector3d p_w_base = 
//     CalcBodyToBaseCoordinates(*model, Q, body_base_id, Vector3d (0., 0., 0.), true);

//   Matrix3d r_w_yaw = CalcBodyWorldOrientation(*model, Q, body_yaw_id, true);
//   Vector3d p_w_yaw = 
//     CalcBodyToBaseCoordinates(*model, Q, body_yaw_id, Vector3d (0., 0., 0.), true);

//   Matrix3d r_w_pitchback = CalcBodyWorldOrientation(*model, Q, body_pitchback_id, true);
//   Vector3d p_w_pitchback = 
//     CalcBodyToBaseCoordinates(*model, Q, body_pitchback_id, Vector3d (0., 0., 0.), true);

//   Matrix3d r_w_pitchbottom = CalcBodyWorldOrientation(*model, Q, body_pitchbottom_id, true);
//   Vector3d p_w_pitchbottom = 
//     CalcBodyToBaseCoordinates(*model, Q, body_pitchbottom_id, Vector3d (0., 0., 0.), true);

//   // Matrix3d r_w_pitchend = CalcBodyWorldOrientation(*model, Q, body_pitchend_id, true);
//   // Vector3d p_w_pitchend = 
//   //   CalcBodyToBaseCoordinates(*model, Q, body_pitchend_id, Vector3d (0., 0., 0.), true);

//   // Matrix3d r_w_maininsertion = CalcBodyWorldOrientation(*model, Q, body_maininsertion_id, true);
//   // Vector3d p_w_maininsertion = 
//   //   CalcBodyToBaseCoordinates(*model, Q, body_maininsertion_id, Vector3d (0., 0., 0.), true);

//   // Matrix3d r_w_tool = CalcBodyWorldOrientation(*model, Q, body_tool_id, true);
//   // Vector3d p_w_tool = 
//   //   CalcBodyToBaseCoordinates(*model, Q, body_tool_id, Vector3d (0., 0., 0.), true);






  

/*
  // 4 - Test case for maininsertionlink-toollink pi/4 - 
  // postion matches which is same has home pose - check for orientation

  CHECK_THAT (Vector3d (0.4999,     -0.3901,      -0.599), 
    AllCloseVector(p_w_base, TEST_PREC, TEST_PREC));

  CHECK_THAT (Vector3d (0.499893,   -0.927001,   -0.599002), 
    AllCloseVector(p_w_yaw, TEST_PREC, TEST_PREC));

  CHECK_THAT (Vector3d (0.499926,   -0.764606,   -0.608803), 
    AllCloseVector(p_w_pitchback, TEST_PREC, TEST_PREC));

  CHECK_THAT (Vector3d (0.499414,   -0.831196,   -0.312371), 
    AllCloseVector(p_w_pitchbottom, TEST_PREC, TEST_PREC));

  CHECK_THAT (Vector3d (0.500074,   -0.491107,   -0.312298), 
    AllCloseVector(p_w_pitchend, TEST_PREC, TEST_PREC));

  CHECK_THAT (Vector3d (0.5,    -0.45007,   -0.228598),
  // CHECK_THAT (Vector3d (0.5001, -0.4498, -0.2269), 
    AllCloseVector(p_w_maininsertion, TEST_PREC, TEST_PREC));

  CHECK_THAT (Vector3d (0.500082,   -0.388279,   -0.229551), 
    AllCloseVector(p_w_tool, TEST_PREC, TEST_PREC)); 
  // #################################################
*/

  // std::cout << "baselink-yawlink\n"; 
  // std::cout << "r_w_base" << std::endl << r_w_base << std::endl;
  // std::cout << "p_w_base" << std::endl << p_w_base << std::endl;
  // std::cout << "-------------\n";

  // std::cout << "yawlink-pitchbacklink\n";
  // std::cout << "r_w_yaw" << std::endl << r_w_yaw << std::endl;
  // std::cout << "p_w_yaw" << std::endl << p_w_yaw << std::endl;
  // std::cout << "-------------\n";

  // std::cout << "pitchbacklink-pitchbottomlink\n";
  // std::cout << "r_w_pitchback" << std::endl << r_w_pitchback << std::endl;
  // std::cout << "p_w_pitchback" << std::endl << p_w_pitchback << std::endl;
  // std::cout << "-------------\n";

  // std::cout << "pitchbottomlink-pitchendlink\n";
  // std::cout << "r_w_pitchbottom" << std::endl << r_w_pitchbottom << std::endl;
  // std::cout << "p_w_pitchbottom" << std::endl << p_w_pitchbottom << std::endl;
  // std::cout << "-------------\n";

  // std::cout << "pitchendlink-maininsertionlink\n";
  // std::cout << "r_w_pitchend" << std::endl << r_w_pitchend << std::endl;
  // std::cout << "p_w_pitchend" << std::endl << p_w_pitchend << std::endl;
  // std::cout << "-------------\n";

  // std::cout << "maininsertionlink-toollink\n";
  // std::cout << "r_w_maininsertion" << std::endl << r_w_maininsertion << std::endl;
  // std::cout << "p_w_maininsertion" << std::endl << p_w_maininsertion << std::endl;
  // std::cout << "-------------\n";

  // std::cout << "toollink-ee\n";
  // std::cout << "r_w_tool" << std::endl << r_w_tool << std::endl;
  // std::cout << "p_w_tool" << std::endl << p_w_tool << std::endl;
  // std::cout << "-------------\n";
// }

// TEST_CASE(__FILE__"_ManualCalculation", "") 
// {
//   // Vector3d p_w_y_w(0, -0.5369, 0);
//   // Matrix3d r_w_y(-1,  0,  0, 0,  0,  1, 0,  1,  0);

//   // Eigen::Affine3d Rtheeta(
//   //   Eigen::AngleAxisd(M_PI_4, Vector3d::UnitY()));

//   // Matrix3d r_w_pback_w = Rtheeta.rotation() * r_w_y;  
  
//   // // std::cout << "r_w_pback_w" << std::endl << r_w_pback_w << std::endl;

//   // Vector3d p_w_pback(0, -0.0098, 0.1624);

//   // Vector3d p_w_pback_w_cal = p_w_y_w + r_w_pback_w * p_w_pback;

//   // Vector3d p_w_pback_w(0,   -0.374506, -0.00980314);

//   // CHECK_THAT (p_w_pback_w, 
//   //   AllCloseVector(p_w_pback_w_cal, TEST_PREC, TEST_PREC));

//   Vector3d p_w_pback_w(0,   -0.374506, -0.00980314);
//   Matrix3d r_w_pback(0, 0, -1, 1, 0, 0, 0, -1, 0);

//   Eigen::Affine3d Rtheeta_pback(
//     Eigen::AngleAxisd(0.6, -Vector3d::UnitX()));

//   Matrix3d r_w_pbottom_w = Rtheeta_pback.rotation() * r_w_pback;  
  
//   // std::cout << "r_w_pback_w" << std::endl << r_w_pback_w << std::endl;

//   Vector3d p_pback_pbottom(-0.0664, -0.2965, 0.0005);

//   Vector3d p_w_pbottom_w_cal = p_w_pback_w + r_w_pbottom_w * p_pback_pbottom;

//   // Home pose
//   // Vector3d p_w_pbottom_w(-0.000487329,   -0.441098,    0.286628);

//   // pi/4
//   Vector3d p_w_pbottom_w(-0.000441523,   -0.206959,    0.274776);
//   CHECK_THAT (p_w_pbottom_w, 
//     AllCloseVector(p_w_pbottom_w_cal, TEST_PREC, TEST_PREC));

// }