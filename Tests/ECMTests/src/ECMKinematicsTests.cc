#include "ECMTests/ECM.h"

/*
// All Test cases passes
TEST_CASE_METHOD ( ECM, __FILE__"_HomePose", "") 
{
// 4294967295, ROOT
// 2147483646, world-baselink
// 0, baselink-yawlink
// 1, yawlink-pitchbacklink
// 2, pitchbacklink-pitchbottomlink
// 3, pitchbottomlink-pitchendlink
// 4, yawlink-pitchfrontlink
// 5, pitchfrontlink-pitchbottomlink
// 6, pitchfrontlink-pitchtoplink
// 7, pitchtoplink-pitchendlink
// 8, pitchendlink-maininsertionlink
// 9, maininsertionlink-toollink


  Q.setZero();

  Vector3d p_w_baselink = CalcBodyToBaseCoordinates(*model, Q, 
    model->GetBodyId("world-baselink"), Vector3d(0., 0., 0.), true);
  CHECK_THAT (Vector3d(0.5, -0.4, -0.6), 
    AllCloseVector(p_w_baselink, TEST_PREC, TEST_PREC));
    
  Vector3d p_w_yawlink = CalcBodyToBaseCoordinates(*model, Q, 
    model->GetBodyId("baselink-yawlink"), Vector3d(0., 0., 0.), true);
  CHECK_THAT (Vector3d(0.5,   -0.936899,   -0.600006), 
    AllCloseVector(p_w_yawlink, TEST_PREC, TEST_PREC));

  Vector3d p_w_pitchbacklink = CalcBodyToBaseCoordinates(*model, Q, 
    model->GetBodyId("yawlink-pitchbacklink"), Vector3d(0., 0., 0.), true);
	CHECK_THAT (Vector3d(0.500009,   -0.774492,   -0.600079), 
    AllCloseVector(p_w_pitchbacklink, TEST_PREC, TEST_PREC));

  Vector3d p_w_pitchbottomlink = CalcBodyToBaseCoordinates(*model, Q, 
    model->GetBodyId("pitchbacklink-pitchbottomlink"), Vector3d(0., 0., 0.), true);
  CHECK_THAT (Vector3d(0.499951,   -0.841289,   -0.313501), 
    AllCloseVector(p_w_pitchbottomlink, TEST_PREC, TEST_PREC));
  Vector3d p_w_pitchendlink = CalcBodyToBaseCoordinates(*model, Q, 
    model->GetBodyId("pitchbottomlink-pitchendlink"), Vector3d(0., 0., 0.), true);
  CHECK_THAT (Vector3d(0.499978,   -0.501171,   -0.313362), 
    AllCloseVector(p_w_pitchendlink, TEST_PREC, TEST_PREC));

  Vector3d p_w_pitchfrontlink = CalcBodyToBaseCoordinates(*model, Q, 
    model->GetBodyId("yawlink-pitchfrontlink"), Vector3d(0., 0., 0.), true);
  CHECK_THAT (Vector3d(0.5,   -0.736909,    -0.59998), 
    AllCloseVector(p_w_pitchfrontlink, TEST_PREC, TEST_PREC));
  Vector3d p_w_pitchtoplink = CalcBodyToBaseCoordinates(*model, Q, 
    model->GetBodyId("pitchfrontlink-pitchtoplink"), Vector3d(0., 0., 0.), true);
  CHECK_THAT (Vector3d(0.499951, -0.846723, -0.276256), 
    AllCloseVector(p_w_pitchtoplink, TEST_PREC, TEST_PREC));
  Vector3d p_w_toollink = CalcBodyToBaseCoordinates(*model, Q, 
    model->GetBodyId("maininsertionlink-toollink"), Vector3d(0., 0., 0.), true);
  CHECK_THAT (Vector3d(0.49993, -0.398184, -0.230241), 
    AllCloseVector(p_w_toollink, TEST_PREC, TEST_PREC));
}
*/

/*
// All Test cases passes
TEST_CASE_METHOD ( ECM, __FILE__"_RandomPose_V0", "") 
{
  // 0               baselink-yawlink 1.5745776891708374
  // 1          yawlink-pitchbacklink 2.5686116714496166e-05
  // 2  pitchbacklink-pitchbottomlink -0.000324003747664392
  // 3   pitchbottomlink-pitchendlink 0.0004958789213560522
  // 4         yawlink-pitchfrontlink -0.0028719049878418446
  // 5 pitchfrontlink-pitchbottomlink 0.002573436824604869
  // 6    pitchfrontlink-pitchtoplink 0.0016521965153515339
  // 7      pitchtoplink-pitchendlink 0.0014138610567897558
  // 8 pitchendlink-maininsertionlink 0.09998920559883118
  // 9     maininsertionlink-toollink -0.029354527592658997

  Q.setZero();
  Q[0] = 1.5745776891708374;
  Q[1] = 2.5686116714496166e-05;
  Q[2] = -0.000324003747664392;
  Q[3] = 0.0004958789213560522;
  Q[4] = -0.0028719049878418446;
  Q[5] = 0.002573436824604869;
  Q[6] = 0.0016521965153515339;
  Q[7] = 0.0014138610567897558;
  Q[8] = 0.09998920559883118;
  Q[9] = -0.029354527592658997;

  // std::cout << "Q" << std::endl << Q << std::endl;

  Vector3d p_w_baselink = CalcBodyToBaseCoordinates(*model, Q, 
    model->GetBodyId("world-baselink"), Vector3d(0., 0., 0.), true);
  CHECK_THAT (Vector3d(0.5, -0.4, -0.6), 
    AllCloseVector(p_w_baselink, TEST_PREC, TEST_PREC));
    
  Vector3d p_w_yawlink = CalcBodyToBaseCoordinates(*model, Q, 
    model->GetBodyId("baselink-yawlink"), Vector3d(0., 0., 0.), true);
  CHECK_THAT (Vector3d(0.500003,   -0.936898,   -0.599996), 
    AllCloseVector(p_w_yawlink, TEST_PREC, TEST_PREC));


  Vector3d p_w_pitchbacklink = CalcBodyToBaseCoordinates(*model, Q, 
    model->GetBodyId("yawlink-pitchbacklink"), Vector3d(0., 0., 0.), true);
	CHECK_THAT (Vector3d(0.500068,   -0.774475,   -0.600029), 
    AllCloseVector(p_w_pitchbacklink, TEST_PREC, TEST_PREC));

  Vector3d p_w_pitchbottomlink = CalcBodyToBaseCoordinates(*model, Q, 
    model->GetBodyId("pitchbacklink-pitchbottomlink"), Vector3d(0., 0., 0.), true);
  CHECK_THAT (Vector3d(0.213359,   -0.840868,   -0.601266), 
    AllCloseVector(p_w_pitchbottomlink, TEST_PREC, TEST_PREC));
  Vector3d p_w_pitchendlink = CalcBodyToBaseCoordinates(*model, Q, 
    model->GetBodyId("pitchbottomlink-pitchendlink"), Vector3d(0., 0., 0.), true);
  CHECK_THAT (Vector3d(0.213287,   -0.500737,   -0.601691), 
    AllCloseVector(p_w_pitchendlink, TEST_PREC, TEST_PREC));
  Vector3d p_w_pitchfrontlink = CalcBodyToBaseCoordinates(*model, Q, 
    model->GetBodyId("yawlink-pitchfrontlink"), Vector3d(0., 0., 0.), true);
  CHECK_THAT (Vector3d(0.49998,   -0.736908,   -0.600024), 
    AllCloseVector(p_w_pitchfrontlink, TEST_PREC, TEST_PREC));
  
  Vector3d p_w_pitchtoplink = CalcBodyToBaseCoordinates(*model, Q, 
    model->GetBodyId("pitchfrontlink-pitchtoplink"), Vector3d(0., 0., 0.), true);
  CHECK_THAT (Vector3d(0.176094,   -0.846235,   -0.601304), 
    AllCloseVector(p_w_pitchtoplink, TEST_PREC, TEST_PREC));

  Vector3d p_w_maininsertionlink = CalcBodyToBaseCoordinates(*model, Q, 
    model->GetBodyId("pitchendlink-maininsertionlink"), Vector3d(0., 0., 0.), true);
  CHECK_THAT (Vector3d(0.227985,    -0.45964,   -0.601657), 
    AllCloseVector(p_w_maininsertionlink, TEST_PREC, TEST_PREC));

  Vector3d p_w_toollink = CalcBodyToBaseCoordinates(*model, Q, 
    model->GetBodyId("maininsertionlink-toollink"), Vector3d(0., 0., 0.), true);
  CHECK_THAT (Vector3d(0.228999,   -0.397638,    -0.60199), 
    AllCloseVector(p_w_toollink, TEST_PREC, TEST_PREC));
}
*/


/*
// All Test cases passes
TEST_CASE_METHOD ( ECM, __FILE__"_RandomPose_V1", "") 
{
  Q.setZero();

  // 0               baselink-yawlink 0.7880660891532898
  // 1          yawlink-pitchbacklink -0.0009518474689684808
  // 2  pitchbacklink-pitchbottomlink 0.0007073471788316965
  // 3   pitchbottomlink-pitchendlink -0.0006491024396382272
  // 4         yawlink-pitchfrontlink -0.003932825289666653
  // 5 pitchfrontlink-pitchbottomlink 0.0036882569547742605
  // 6    pitchfrontlink-pitchtoplink 0.002760454313829541
  // 7      pitchtoplink-pitchendlink 0.0002771636936813593
  // 8 pitchendlink-maininsertionlink 0.10089550912380219
  // 9     maininsertionlink-toollink -0.020769545808434486

  Q[0] = 0.7880660891532898;
  Q[1] = -0.0009518474689684808;
  Q[2] = 0.0007073471788316965;
  Q[3] = -0.0006491024396382272;
  Q[4] = -0.003932825289666653;
  Q[5] = 0.0036882569547742605;
  Q[6] = 0.002760454313829541;
  Q[7] = 0.0002771636936813593;
  Q[8] = 0.10089550912380219;
  Q[9] = -0.020769545808434486;
  std::cout << "Q" << std::endl << Q << std::endl;

  Vector3d p_w_baselink = CalcBodyToBaseCoordinates(*model, Q, 
    model->GetBodyId("world-baselink"), Vector3d(0., 0., 0.), true);
  CHECK_THAT (Vector3d(0.5, -0.4, -0.6), 
    AllCloseVector(p_w_baselink, TEST_PREC, TEST_PREC));
    
  Vector3d p_w_yawlink = CalcBodyToBaseCoordinates(*model, Q, 
    model->GetBodyId("baselink-yawlink"), Vector3d(0., 0., 0.), true);
  CHECK_THAT (Vector3d(0.500006,   -0.936899,   -0.600001), 
    AllCloseVector(p_w_yawlink, TEST_PREC, TEST_PREC));


  Vector3d p_w_pitchbacklink = CalcBodyToBaseCoordinates(*model, Q, 
    model->GetBodyId("yawlink-pitchbacklink"), Vector3d(0., 0., 0.), true);
	CHECK_THAT (Vector3d(0.50004,   -0.774488,   -0.600063), 
    AllCloseVector(p_w_pitchbacklink, TEST_PREC, TEST_PREC));

  Vector3d p_w_pitchbottomlink = CalcBodyToBaseCoordinates(*model, Q, 
    model->GetBodyId("pitchbacklink-pitchbottomlink"), Vector3d(0., 0., 0.), true);
  CHECK_THAT (Vector3d(0.296746,   -0.841165,   -0.398033), 
    AllCloseVector(p_w_pitchbottomlink, TEST_PREC, TEST_PREC));
  Vector3d p_w_pitchendlink = CalcBodyToBaseCoordinates(*model, Q, 
    model->GetBodyId("pitchbottomlink-pitchendlink"), Vector3d(0., 0., 0.), true);
  CHECK_THAT (Vector3d(0.296503,   -0.501044,   -0.398199), 
    AllCloseVector(p_w_pitchendlink, TEST_PREC, TEST_PREC));
  Vector3d p_w_pitchfrontlink = CalcBodyToBaseCoordinates(*model, Q, 
    model->GetBodyId("yawlink-pitchfrontlink"), Vector3d(0., 0., 0.), true);
  CHECK_THAT (Vector3d(0.499974,   -0.736908,   -0.599998), 
    AllCloseVector(p_w_pitchfrontlink, TEST_PREC, TEST_PREC));
  
  Vector3d p_w_pitchtoplink = CalcBodyToBaseCoordinates(*model, Q, 
    model->GetBodyId("pitchfrontlink-pitchtoplink"), Vector3d(0., 0., 0.), true);
  CHECK_THAT (Vector3d(0.270381,    -0.84658,   -0.371711), 
    AllCloseVector(p_w_pitchtoplink, TEST_PREC, TEST_PREC));

  Vector3d p_w_maininsertionlink = CalcBodyToBaseCoordinates(*model, Q, 
    model->GetBodyId("pitchendlink-maininsertionlink"), Vector3d(0., 0., 0.), true);
  CHECK_THAT (Vector3d(0.307533,   -0.459929,    -0.40918), 
    AllCloseVector(p_w_maininsertionlink, TEST_PREC, TEST_PREC));

  Vector3d p_w_toollink = CalcBodyToBaseCoordinates(*model, Q, 
    model->GetBodyId("maininsertionlink-toollink"), Vector3d(0., 0., 0.), true);
  CHECK_THAT (Vector3d(0.308055,   -0.397927,   -0.410032), 
    AllCloseVector(p_w_toollink, TEST_PREC, TEST_PREC));
}
*/


TEST_CASE_METHOD ( ECM, __FILE__"_RandomPose_V2", "") 
{
  Q.setZero();
  // //               baselink-yawlink 0.7880651950836182
  // //          yawlink-pitchbacklink -0.0009477213025093079
  // // pitchendlink-maininsertionlink 0.10089556127786636
  // //     maininsertionlink-toollink 0.7641677856445312

  // Q[0] = 0.7880660891532898;
  // Q[1] = -0.0009477213025093079;
  // Q[7] = 0.10089556127786636;
  // Q[8] = 0.7641677856445312;

  // 0,               baselink-yawlink, 0.7880651950836182
  // 1,          yawlink-pitchbacklink, -0.0009477213025093079
  // 2,  pitchbacklink-pitchbottomlink, 0.000703148078173399
  // 3,   pitchbottomlink-pitchendlink, -0.0006443223683163524
  // 4,         yawlink-pitchfrontlink, -0.003928348887711763
  // 5, pitchfrontlink-pitchbottomlink, 0.003683707909658551
  // 6,    pitchfrontlink-pitchtoplink, 0.0027559399604797363
  // 7,      pitchtoplink-pitchendlink, 0.0002819084911607206
  // 8, pitchendlink-maininsertionlink, 0.10089556127786636
  // 9,     maininsertionlink-toollink, 0.7641677856445312


  Q[0] = 0.7880651950836182;
  Q[1] = -0.0009477213025093079;
  Q[2] = 0.000703148078173399;
  Q[3] = -0.0006443223683163524;
  Q[4] = -0.003928348887711763;
  Q[5] = 0.003683707909658551;
  Q[6] = 0.0027559399604797363;
  Q[7] = 0.0002819084911607206;
  Q[8] = 0.10089556127786636;
  Q[9] = 0.7641677856445312;

  // std::cout << "Q" << std::endl << Q << std::endl;

  Vector3d p_w_baselink = CalcBodyToBaseCoordinates(*model, Q, 
    model->GetBodyId("world-baselink"), Vector3d(0., 0., 0.), true);
  CHECK_THAT (Vector3d(0.5, -0.4, -0.6), 
    AllCloseVector(p_w_baselink, TEST_PREC, TEST_PREC));
    
  Vector3d p_w_yawlink = CalcBodyToBaseCoordinates(*model, Q, 
    model->GetBodyId("baselink-yawlink"), Vector3d(0., 0., 0.), true);
  CHECK_THAT (Vector3d(0.500006,   -0.936899,   -0.600001), 
    AllCloseVector(p_w_yawlink, TEST_PREC, TEST_PREC));

  Vector3d p_w_pitchbacklink = CalcBodyToBaseCoordinates(*model, Q, 
    model->GetBodyId("yawlink-pitchbacklink"), Vector3d(0., 0., 0.), true);
	CHECK_THAT (Vector3d(0.50004,   -0.774488,   -0.600063), 
    AllCloseVector(p_w_pitchbacklink, TEST_PREC, TEST_PREC));

  Vector3d p_w_pitchbottomlink = CalcBodyToBaseCoordinates(*model, Q, 
    model->GetBodyId("pitchbacklink-pitchbottomlink"), Vector3d(0., 0., 0.), true);
  CHECK_THAT (Vector3d(0.296746,   -0.841163,   -0.398033), 
    AllCloseVector(p_w_pitchbottomlink, TEST_PREC, TEST_PREC));
  Vector3d p_w_pitchendlink = CalcBodyToBaseCoordinates(*model, Q, 
    model->GetBodyId("pitchbottomlink-pitchendlink"), Vector3d(0., 0., 0.), true);
  CHECK_THAT (Vector3d(0.296503,   -0.501043,   -0.398198), 
    AllCloseVector(p_w_pitchendlink, TEST_PREC, TEST_PREC));
  Vector3d p_w_pitchfrontlink = CalcBodyToBaseCoordinates(*model, Q, 
    model->GetBodyId("yawlink-pitchfrontlink"), Vector3d(0., 0., 0.), true);
  CHECK_THAT (Vector3d(0.499974,   -0.736908,   -0.599998), 
    AllCloseVector(p_w_pitchfrontlink, TEST_PREC, TEST_PREC));
  
  Vector3d p_w_pitchtoplink = CalcBodyToBaseCoordinates(*model, Q, 
    model->GetBodyId("pitchfrontlink-pitchtoplink"), Vector3d(0., 0., 0.), true);
  CHECK_THAT (Vector3d(0.270381,   -0.846579,    -0.37171), 
    AllCloseVector(p_w_pitchtoplink, TEST_PREC, TEST_PREC));

  p_w_pitchendlink = CalcBodyToBaseCoordinates(*model, Q, 
    model->GetBodyId("pitchtoplink-pitchendlink"), Vector3d(0., 0., 0.), true);
  CHECK_THAT (Vector3d(0.296503,   -0.501043,   -0.398198), 
    AllCloseVector(p_w_pitchendlink, TEST_PREC, TEST_PREC));
  Vector3d p_w_maininsertionlink = CalcBodyToBaseCoordinates(*model, Q, 
    model->GetBodyId("pitchendlink-maininsertionlink"), Vector3d(0., 0., 0.), true);
  CHECK_THAT (Vector3d(0.307534,   -0.459927,   -0.409179), 
    AllCloseVector(p_w_maininsertionlink, TEST_PREC, TEST_PREC));

  Vector3d p_w_toollink = CalcBodyToBaseCoordinates(*model, Q, 
    model->GetBodyId("maininsertionlink-toollink"), Vector3d(0., 0., 0.), true);
  CHECK_THAT (Vector3d(0.308056,   -0.397926,   -0.41003), 
    AllCloseVector(p_w_toollink, TEST_PREC, TEST_PREC));
}

/*
TEST_CASE_METHOD ( ECM, __FILE__"_RandomPose_V3", "") 
{
//               baselink-yawlink 9.513680561212823e-05 - 0
//          yawlink-pitchbacklink 0.791019082069397     - 1
//  pitchbacklink-pitchbottomlink -0.7852027416229248   - 2
//   pitchbottomlink-pitchendlink 0.7890188694000244    - 3
//         yawlink-pitchfrontlink 0.7893757224082947    - 4
//    pitchfrontlink-pitchtoplink -0.7822339534759521   - 5
//      pitchtoplink-pitchendlink 0.7876935005187988    - 6
// pitchendlink-maininsertionlink 0.0009716766071505845 - 7
//     maininsertionlink-toollink 0.000289801973849535  - 8
//          baselink-pitchendlink 0.0



  Q.setZero();
  Q[0] = 0.0;
  Q[1] = 0.791019082069397;
  Q[2] = -0.7852027416229248;
  Q[3] = 0.7890188694000244;
  Q[4] = 0.7893757224082947;
  Q[5] = -0.7822339534759521;
  Q[6] = 0.7876935005187988;
  Q[7] = 0.0;
  Q[8] = 0.0;

  // std::cout << "Q" << std::endl << Q << std::endl;

  Vector3d p_w_baselink = CalcBodyToBaseCoordinates(*model, Q, 
    model->GetBodyId("world-baselink"), Vector3d(0., 0., 0.), true);
  CHECK_THAT (Vector3d(0.5, -0.4, -0.6), 
    AllCloseVector(p_w_baselink, TEST_PREC, TEST_PREC));
    
  Vector3d p_w_yawlink = CalcBodyToBaseCoordinates(*model, Q, 
    model->GetBodyId("baselink-yawlink"), Vector3d(0., 0., 0.), true);
  CHECK_THAT (Vector3d(0.5,   -0.936888,   -0.599973), 
    AllCloseVector(p_w_yawlink, TEST_PREC, TEST_PREC));

  Vector3d p_w_pitchbacklink = CalcBodyToBaseCoordinates(*model, Q, 
    model->GetBodyId("yawlink-pitchbacklink"), Vector3d(0., 0., 0.), true);
	CHECK_THAT (Vector3d(0.499999,   -0.774282,   -0.599737), 
    AllCloseVector(p_w_pitchbacklink, TEST_PREC, TEST_PREC));

  Vector3d p_w_pitchbottomlink = CalcBodyToBaseCoordinates(*model, Q, 
    model->GetBodyId("pitchbacklink-pitchbottomlink"), Vector3d(0., 0., 0.), true);
  CHECK_THAT (Vector3d(0.499934,   -0.606216,   -0.325329), 
    AllCloseVector(p_w_pitchbottomlink, TEST_PREC, TEST_PREC));
  Vector3d p_w_pitchendlink = CalcBodyToBaseCoordinates(*model, Q, 
    model->GetBodyId("pitchbottomlink-pitchendlink"), Vector3d(0., 0., 0.), true);
  CHECK_THAT (Vector3d(0.499955,   -0.265919,   -0.327204), 
    AllCloseVector(p_w_pitchendlink, TEST_PREC, TEST_PREC));
  Vector3d p_w_pitchfrontlink = CalcBodyToBaseCoordinates(*model, Q, 
    model->GetBodyId("yawlink-pitchfrontlink"), Vector3d(0., 0., 0.), true);
  CHECK_THAT (Vector3d(0.500002,   -0.736932,    -0.60007), 
    AllCloseVector(p_w_pitchfrontlink, TEST_PREC, TEST_PREC));
  
  Vector3d p_w_pitchtoplink = CalcBodyToBaseCoordinates(*model, Q, 
    model->GetBodyId("pitchfrontlink-pitchtoplink"), Vector3d(0., 0., 0.), true);
  CHECK_THAT (Vector3d(0.499917,   -0.583113,    -0.29479), 
    AllCloseVector(p_w_pitchtoplink, TEST_PREC, TEST_PREC));

  Vector3d p_w_maininsertionlink = CalcBodyToBaseCoordinates(*model, Q, 
    model->GetBodyId("pitchendlink-maininsertionlink"), Vector3d(0., 0., 0.), true);
  CHECK_THAT (Vector3d(0.499931,   -0.176943,   -0.297481), 
    AllCloseVector(p_w_maininsertionlink, TEST_PREC, TEST_PREC));

  Vector3d p_w_toollink = CalcBodyToBaseCoordinates(*model, Q, 
    model->GetBodyId("maininsertionlink-toollink"), Vector3d(0., 0., 0.), true);
  CHECK_THAT (Vector3d(0.499971,   -0.134195,   -0.342508), 
    AllCloseVector(p_w_toollink, TEST_PREC, TEST_PREC));
}
*/

/*
// Remove pitchbottomlink - test cases fails at pitchendlink
TEST_CASE_METHOD ( ECM, __FILE__"_RandomPose_V4", "") 
{
// 4294967295, ROOT
// 0, baselink-yawlink
// 1, yawlink-pitchfrontlink
// 2, pitchfrontlink-pitchtoplink
// 3, pitchtoplink-pitchendlink
// 4, pitchendlink-maininsertionlink
// 5, maininsertionlink-toollink
// 2147483646, world-baselink

//               baselink-yawlink 0.0
//         yawlink-pitchfrontlink 0.7871469855308533
//    pitchfrontlink-pitchtoplink -0.7796755433082581
//      pitchtoplink-pitchendlink 0.7852902412414551
// pitchendlink-maininsertionlink 0.0
//     maininsertionlink-toollink 0.0

  Q.setZero();
  Q[0] = 0.0;
  Q[1] = 0.7871469855308533;
  Q[2] = -0.7796755433082581;
  Q[3] = 0.7852902412414551;
  Q[4] = 0.0;
  Q[5] = 0.0;
  // std::cout << "Q" << std::endl << Q << std::endl;

  Vector3d p_w_baselink = CalcBodyToBaseCoordinates(*model, Q, 
    model->GetBodyId("world-baselink"), Vector3d(0., 0., 0.), true);
  CHECK_THAT (Vector3d(0.5, -0.4, -0.6), 
    AllCloseVector(p_w_baselink, TEST_PREC, TEST_PREC));
    
  Vector3d p_w_yawlink = CalcBodyToBaseCoordinates(*model, Q, 
    model->GetBodyId("baselink-yawlink"), Vector3d(0., 0., 0.), true);
  CHECK_THAT (Vector3d(0.5,   -0.936888,   -0.599973), 
    AllCloseVector(p_w_yawlink, TEST_PREC, TEST_PREC));

  // Vector3d p_w_pitchbacklink = CalcBodyToBaseCoordinates(*model, Q, 
  //   model->GetBodyId("yawlink-pitchbacklink"), Vector3d(0., 0., 0.), true);
	// CHECK_THAT (Vector3d(0.499999,   -0.774282,   -0.599737), 
  //   AllCloseVector(p_w_pitchbacklink, TEST_PREC, TEST_PREC));

  // Vector3d p_w_pitchbottomlink = CalcBodyToBaseCoordinates(*model, Q, 
  //   model->GetBodyId("pitchbacklink-pitchbottomlink"), Vector3d(0., 0., 0.), true);
  // CHECK_THAT (Vector3d(0.499934,   -0.606216,   -0.325329), 
  //   AllCloseVector(p_w_pitchbottomlink, TEST_PREC, TEST_PREC));
  // Vector3d p_w_pitchendlink = CalcBodyToBaseCoordinates(*model, Q, 
  //   model->GetBodyId("pitchbottomlink-pitchendlink"), Vector3d(0., 0., 0.), true);
  // CHECK_THAT (Vector3d(0.499955,   -0.265919,   -0.327204), 
  //   AllCloseVector(p_w_pitchendlink, TEST_PREC, TEST_PREC));
  Vector3d p_w_pitchfrontlink = CalcBodyToBaseCoordinates(*model, Q, 
    model->GetBodyId("yawlink-pitchfrontlink"), Vector3d(0., 0., 0.), true);
  CHECK_THAT (Vector3d(0.5,   -0.736879,   -0.600001), 
    AllCloseVector(p_w_pitchfrontlink, TEST_PREC, TEST_PREC));
  
  Vector3d p_w_pitchendlink = CalcBodyToBaseCoordinates(*model, Q, 
    model->GetBodyId("pitchtoplink-pitchendlink"), Vector3d(0., 0., 0.), true);
  CHECK_THAT (Vector3d(0.499938,     -0.2665,   -0.326943), 
    AllCloseVector(p_w_pitchendlink, TEST_PREC, TEST_PREC));

  Vector3d p_w_pitchtoplink = CalcBodyToBaseCoordinates(*model, Q, 
    model->GetBodyId("pitchfrontlink-pitchtoplink"), Vector3d(0., 0., 0.), true);
  CHECK_THAT (Vector3d(0.499938,   -0.583697,   -0.294375), 
    AllCloseVector(p_w_pitchtoplink, TEST_PREC, TEST_PREC));

  Vector3d p_w_maininsertionlink = CalcBodyToBaseCoordinates(*model, Q, 
    model->GetBodyId("pitchendlink-maininsertionlink"), Vector3d(0., 0., 0.), true);
  CHECK_THAT (Vector3d(0.499901,   -0.177521,   -0.296968), 
    AllCloseVector(p_w_maininsertionlink, TEST_PREC, TEST_PREC));

  Vector3d p_w_toollink = CalcBodyToBaseCoordinates(*model, Q, 
    model->GetBodyId("maininsertionlink-toollink"), Vector3d(0., 0., 0.), true);
  CHECK_THAT (Vector3d(0.499935,   -0.134677,     -0.3419), 
    AllCloseVector(p_w_toollink, TEST_PREC, TEST_PREC));
}
*/

/*
TEST_CASE_METHOD ( ECM, __FILE__"_IK_RandomPose_V4", "") 
{
// 4294967295, ROOT
// 0, baselink-yawlink
// 1, yawlink-pitchfrontlink
// 2, pitchfrontlink-pitchtoplink
// 3, pitchtoplink-pitchendlink
// 4, pitchendlink-maininsertionlink
// 5, maininsertionlink-toollink
// 2147483646, world-baselink

//               baselink-yawlink 0.0
//         yawlink-pitchfrontlink 0.7871469855308533
//    pitchfrontlink-pitchtoplink -0.7796755433082581
//      pitchtoplink-pitchendlink 0.7852902412414551
// pitchendlink-maininsertionlink 0.0
//     maininsertionlink-toollink 0.0

  InverseKinematicsConstraintSet cs;
  Vector3d body_point(0., 0., 0.);
  Q.setZero();
  Q[0] = 0.0;
  Q[1] = 0.7871469855308533;
  Q[2] = -0.7796755433082581;
  // Q[3] = 0.7852902412414551;
  // Q[4] = 0.0;
  // Q[5] = 0.0;
  // std::cout << "Q" << std::endl << Q << std::endl;

  unsigned int bodyId;
  bodyId = model->GetBodyId("world-baselink");
  cs.AddFullConstraint(bodyId, body_point, 
    CalcBodyToBaseCoordinates(*model, Q, bodyId, body_point, true), 
     CalcBodyWorldOrientation(*model, Q, bodyId, true));

  bodyId = model->GetBodyId("baselink-yawlink");
  cs.AddFullConstraint(bodyId, body_point, 
    CalcBodyToBaseCoordinates(*model, Q, bodyId, body_point, true), 
     CalcBodyWorldOrientation(*model, Q, bodyId, true));

  bodyId = model->GetBodyId("yawlink-pitchfrontlink");
  cs.AddFullConstraint(bodyId, body_point, 
    CalcBodyToBaseCoordinates(*model, Q, bodyId, body_point, true), 
     CalcBodyWorldOrientation(*model, Q, bodyId, true));

  bodyId = model->GetBodyId("pitchfrontlink-pitchtoplink");
  cs.AddFullConstraint(bodyId, body_point, 
    CalcBodyToBaseCoordinates(*model, Q, bodyId, body_point, true), 
     CalcBodyWorldOrientation(*model, Q, bodyId, true));

  bodyId = model->GetBodyId("pitchtoplink-pitchendlink");
  cs.AddPointConstraint(bodyId, Vector3d(0.499938,     -0.2665,   -0.326943), 
    Vector3d(1., 1., 1.), 1.0F);
  
  VectorNd Qres (Q);
  bool result = InverseKinematics (*model, Q, cs, Qres);
  CHECK (result);
  CHECK_THAT (0., IsClose(cs.error_norm, TEST_PREC, TEST_PREC));

  std::cout << "Qres:" << std::endl << Qres << std::endl;
  Vector3d p_w_baselink = CalcBodyToBaseCoordinates(*model, Qres, 
    model->GetBodyId("world-baselink"), Vector3d(0., 0., 0.), true);
  CHECK_THAT (Vector3d(0.5, -0.4, -0.6), 
    AllCloseVector(p_w_baselink, TEST_PREC, TEST_PREC));
    
  Vector3d p_w_yawlink = CalcBodyToBaseCoordinates(*model, Qres, 
    model->GetBodyId("baselink-yawlink"), Vector3d(0., 0., 0.), true);
  CHECK_THAT (Vector3d(0.5,   -0.936888,   -0.599973), 
    AllCloseVector(p_w_yawlink, TEST_PREC, TEST_PREC));

  // Vector3d p_w_pitchbacklink = CalcBodyToBaseCoordinates(*model, Q, 
  //   model->GetBodyId("yawlink-pitchbacklink"), Vector3d(0., 0., 0.), true);
	// CHECK_THAT (Vector3d(0.499999,   -0.774282,   -0.599737), 
  //   AllCloseVector(p_w_pitchbacklink, TEST_PREC, TEST_PREC));

  // Vector3d p_w_pitchbottomlink = CalcBodyToBaseCoordinates(*model, Q, 
  //   model->GetBodyId("pitchbacklink-pitchbottomlink"), Vector3d(0., 0., 0.), true);
  // CHECK_THAT (Vector3d(0.499934,   -0.606216,   -0.325329), 
  //   AllCloseVector(p_w_pitchbottomlink, TEST_PREC, TEST_PREC));
  // Vector3d p_w_pitchendlink = CalcBodyToBaseCoordinates(*model, Q, 
  //   model->GetBodyId("pitchbottomlink-pitchendlink"), Vector3d(0., 0., 0.), true);
  // CHECK_THAT (Vector3d(0.499955,   -0.265919,   -0.327204), 
  //   AllCloseVector(p_w_pitchendlink, TEST_PREC, TEST_PREC));
  Vector3d p_w_pitchfrontlink = CalcBodyToBaseCoordinates(*model, Qres, 
    model->GetBodyId("yawlink-pitchfrontlink"), Vector3d(0., 0., 0.), true);
  CHECK_THAT (Vector3d(0.5,   -0.736879,   -0.600001), 
    AllCloseVector(p_w_pitchfrontlink, TEST_PREC, TEST_PREC));
  
  Vector3d p_w_pitchendlink = CalcBodyToBaseCoordinates(*model, Qres, 
    model->GetBodyId("pitchtoplink-pitchendlink"), Vector3d(0., 0., 0.), true);
  CHECK_THAT (Vector3d(0.499938,     -0.2665,   -0.326943), 
    AllCloseVector(p_w_pitchendlink, TEST_PREC, TEST_PREC));

  // Vector3d p_w_pitchtoplink = CalcBodyToBaseCoordinates(*model, Q, 
  //   model->GetBodyId("pitchfrontlink-pitchtoplink"), Vector3d(0., 0., 0.), true);
  // CHECK_THAT (Vector3d(0.499938,   -0.583697,   -0.294375), 
  //   AllCloseVector(p_w_pitchtoplink, TEST_PREC, TEST_PREC));

  // Vector3d p_w_maininsertionlink = CalcBodyToBaseCoordinates(*model, Q, 
  //   model->GetBodyId("pitchendlink-maininsertionlink"), Vector3d(0., 0., 0.), true);
  // CHECK_THAT (Vector3d(0.499901,   -0.177521,   -0.296968), 
  //   AllCloseVector(p_w_maininsertionlink, TEST_PREC, TEST_PREC));

  // Vector3d p_w_toollink = CalcBodyToBaseCoordinates(*model, Q, 
  //   model->GetBodyId("maininsertionlink-toollink"), Vector3d(0., 0., 0.), true);
  // CHECK_THAT (Vector3d(0.499935,   -0.134677,     -0.3419), 
  //   AllCloseVector(p_w_toollink, TEST_PREC, TEST_PREC));
}
*/
