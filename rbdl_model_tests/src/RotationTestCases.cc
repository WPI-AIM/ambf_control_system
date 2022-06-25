#include "rbdl_model_tests/rbdl_tests.h"
#include <rbdl/rbdl.h>
#include <rbdl/rbdl_math.h>

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

Matrix3d R_W_N(Matrix3d r_w_p, Matrix3d bodyRot, Matrix3d bodyRotOffset)
{
  Matrix3d r_bodyST = bodyRot.transpose() * bodyRotOffset;
  Matrix3d r_w_c = r_w_p * r_bodyST;

  return r_w_c;
}


TEST_CASE(__FILE__"_RoationCheck", "") 
{  

  Matrix3d r_w_b;
  Matrix3d r_w_y;
  Matrix3d r_w_pba;
  Matrix3d r_w_pbo;
  Matrix3d r_w_pend;
  Matrix3d r_w_m;
  Matrix3d r_w_tool;

  Matrix3d r_b_yST;
  Matrix3d r_y_pbaST;
  Matrix3d r_pba_pboST;
  Matrix3d r_pbo_pendST;
  Matrix3d r_pend_mST;
  Matrix3d r_m_toolST;

  // 1 base
  r_w_b.setIdentity();

  
  Matrix3d b_yRot(
    1, 0, 0, 
    0, 0, -1,
    0, 1, 0
    );

  Matrix3d b_yRotOffset(
    -1, 0, 0, 
    0,-1, 0,
    0, 0, 1
  );

  // r_b_yST = b_yRot.transpose() * b_yRotOffset;
  // r_w_y = r_w_b * r_b_yST;

  r_w_y = R_W_N(r_w_b, b_yRot, b_yRotOffset);
  Matrix3d r_w_y_(
    -1, 0, 0, 
    0, 0, 1,
    0, 1, 0
  );

  CHECK_THAT (r_w_y_, AllCloseMatrix(r_w_y, TEST_PREC, TEST_PREC));

  // 3 pitchback
  Matrix3d y_pbaRot(
      0, 0, -1,
      0, 1, 0,
      1, 0, 0
  );

  Matrix3d y_pbaRotOffset(
    -1, 0, 0,
    0,-1, 0,
    0, 0, 1
  );

  // r_y_pbaST = y_pbaRot.transpose() * y_pbaRotOffset;
  
  // Matrix3d r_y_pbST_(
  //   0, 0, 1,
  //   0,-1, 0,
  //   1, 0, 0
  // );
  // CHECK_THAT (r_y_pbST_, AllCloseMatrix(r_y_pbST, TEST_PREC, TEST_PREC));
  // r_w_pba = r_w_y * r_y_pbaST;
  r_w_pba = R_W_N(r_w_y, y_pbaRot, y_pbaRotOffset);
  Matrix3d r_w_pba_(
  0, 0, -1,
  1, 0,  0,
  0, -1, 0
  );

  CHECK_THAT (r_w_pba_, AllCloseMatrix(r_w_pba, TEST_PREC, TEST_PREC));

  // 4 pitchbottom
  Matrix3d pba_pboRot;
  pba_pboRot.setIdentity();

  Matrix3d pba_pboRotOffset;
  pba_pboRotOffset.setIdentity();

  // r_pba_pboST = pba_pboRot.transpose() * pba_pboRotOffset;
  // r_w_pbo = r_w_pba * r_pba_pboST;

  r_w_pbo = R_W_N(r_w_pba, pba_pboRot, pba_pboRotOffset);

  Matrix3d r_w_pbo_(
    0, 0, -1,
    1, 0, 0,
    0, -1, 0
  );

  CHECK_THAT (r_w_pbo_, AllCloseMatrix(r_w_pbo, TEST_PREC, TEST_PREC));

  // 5 pitchend
  Matrix3d pbo_pendRot;
  pbo_pendRot.setIdentity();

  Matrix3d pbo_pendRotOffset;
  pbo_pendRotOffset.setIdentity();

  // r_pbo_pendST = pbo_pendRot.transpose() * pbo_pendRotOffset;
  // r_w_pend = r_w_pbo * r_pbo_pendST;

  r_w_pend = R_W_N(r_w_pbo, pbo_pendRot, pbo_pendRotOffset);
  Matrix3d r_w_end_(
    0, 0, -1,
    1, 0, 0,
    0, -1, 0
  );

  CHECK_THAT (r_w_end_, AllCloseMatrix(r_w_pend, TEST_PREC, TEST_PREC));

  // 6 maininsertion
  Matrix3d pend_mRot(
  0, 1, 0,
  -1, 0, 0,
  -0, 0, 1
  );

  Matrix3d pend_mRotOffset;
  pend_mRotOffset.setIdentity();

  // r_pend_mST = pend_mRot.transpose() * pend_mRotOffset;
  // r_w_m = r_w_pend * r_pend_mST;

  r_w_m = R_W_N(r_w_pend, pend_mRot, pend_mRotOffset);
  Matrix3d r_w_m_(
  0, 0, -1,
  0, -1, 0,
  -1, 0, 0
  );

  CHECK_THAT (r_w_m_, AllCloseMatrix(r_w_m, TEST_PREC, TEST_PREC));

  // pitchtool
  Matrix3d m_toolRot(
    0, 0, 1,
    0, 1, 0,
    -1, 0, 0
  );

  Matrix3d m_toolRotOffset(
    0, -1, 0,
    1,  0, 0,
    0,  0, 1
  );

  // r_m_toolST = m_toolRot.transpose() * m_toolRotOffset;

  // Matrix3d r_m_toolST_(
  //   0, 0, -1,
  //   1, 0, 0,
  //   0, -1, 0
  // );
  // CHECK_THAT (r_m_toolST_, AllCloseMatrix(r_m_toolST, TEST_PREC, TEST_PREC));

  // r_w_tool = r_w_m * r_m_toolST;

  r_w_m = R_W_N(r_w_pend, pend_mRot, pend_mRotOffset);
  
  Matrix3d r_w_tool_(
  0, 1, 0,
  -1, 0, 0,
   0, 0, 1
  );

  CHECK_THAT (r_w_tool_, AllCloseMatrix(r_w_tool, TEST_PREC, TEST_PREC));
}