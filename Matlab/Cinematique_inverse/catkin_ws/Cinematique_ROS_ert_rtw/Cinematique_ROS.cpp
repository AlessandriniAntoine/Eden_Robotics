/*
 * Cinematique_ROS.cpp
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "Cinematique_ROS".
 *
 * Model version              : 1.46
 * Simulink Coder version : 9.4 (R2020b) 29-Jul-2020
 * C++ source code generated on : Thu Nov 25 10:49:18 2021
 *
 * Target selection: ert.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Generic->Unspecified (assume 32-bit Generic)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "Cinematique_ROS.h"
#include "Cinematique_ROS_private.h"

/* Block signals (default storage) */
B_Cinematique_ROS_T Cinematique_ROS_B;

/* Block states (default storage) */
DW_Cinematique_ROS_T Cinematique_ROS_DW;

/* Real-time model */
RT_MODEL_Cinematique_ROS_T Cinematique_ROS_M_ = RT_MODEL_Cinematique_ROS_T();
RT_MODEL_Cinematique_ROS_T *const Cinematique_ROS_M = &Cinematique_ROS_M_;

/* Forward declaration for local functions */
static void Cinematique_ROS_emxInit_char_T(emxArray_char_T_Cinematique_R_T
  **pEmxArray, int32_T numDimensions);
static void Cinematique_ROS_emxInit_real_T(emxArray_real_T_Cinematique_R_T
  **pEmxArray, int32_T numDimensions);
static void emxInitStruct_c_rigidBodyJoint(c_rigidBodyJoint_Cinematique__T
  *pStruct);
static void Cinemati_emxInit_unnamed_struct(emxArray_unnamed_struct_Cinem_T
  **pEmxArray, int32_T numDimensions);
static void emxInitStruct_m_robotics_manip_(m_robotics_manip_internal_Col_T
  *pStruct);
static void emxInitStruct_w_robotics_manip_(w_robotics_manip_internal_Rig_T
  *pStruct);
static void emxInitMatrix_w_robotics_manip_(w_robotics_manip_internal_Rig_T
  pMatrix[5]);
static void emxInitStruct_x_robotics_manip_(x_robotics_manip_internal_Rig_T
  *pStruct);
static void emxInitStruct_f_robotics_manip_(f_robotics_manip_internal_IKE_T
  *pStruct);
static void emxInitStruct_y_robotics_manip_(y_robotics_manip_internal_Rig_T
  *pStruct);
static void emxInitMatrix_y_robotics_manip_(y_robotics_manip_internal_Rig_T
  pMatrix[5]);
static void emxInitStruct_ab_robotics_manip(ab_robotics_manip_internal_Ri_T
  *pStruct);
static void emxInitMatrix_m_robotics_manip_(m_robotics_manip_internal_Col_T
  pMatrix[6]);
static void emxInitMatrix_c_rigidBodyJoint(c_rigidBodyJoint_Cinematique__T
  pMatrix[5]);
static void emxInitStruct_h_robotics_core_i(h_robotics_core_internal_Damp_T
  *pStruct);
static void emxInitStruct_b_inverseKinemati(b_inverseKinematics_Cinematiq_T
  *pStruct);
static void emxInitStruct_robotics_slmanip_(robotics_slmanip_internal_blo_T
  *pStruct);
static void Cinema_emxEnsureCapacity_char_T(emxArray_char_T_Cinematique_R_T
  *emxArray, int32_T oldNumel);
static void Cinema_emxEnsureCapacity_real_T(emxArray_real_T_Cinematique_R_T
  *emxArray, int32_T oldNumel);
static void emxEnsureCapacity_unnamed_struc(emxArray_unnamed_struct_Cinem_T
  *emxArray, int32_T oldNumel);
static void Cinemati_emxFree_unnamed_struct(emxArray_unnamed_struct_Cinem_T
  **pEmxArray);
static void Cinematique_ROS_emxFree_char_T(emxArray_char_T_Cinematique_R_T
  **pEmxArray);
static w_robotics_manip_internal_Rig_T *Cinematique_RigidBody_RigidBody
  (w_robotics_manip_internal_Rig_T *obj);
static w_robotics_manip_internal_Rig_T *Cinematiq_RigidBody_RigidBody_k
  (w_robotics_manip_internal_Rig_T *obj);
static w_robotics_manip_internal_Rig_T *Cinemati_RigidBody_RigidBody_k1
  (w_robotics_manip_internal_Rig_T *obj);
static w_robotics_manip_internal_Rig_T *Cinemat_RigidBody_RigidBody_k1e
  (w_robotics_manip_internal_Rig_T *obj);
static x_robotics_manip_internal_Rig_T *Cin_RigidBodyTree_RigidBodyTree
  (x_robotics_manip_internal_Rig_T *obj);
static void Cinemat_genrand_uint32_vector_k(uint32_T mt[625], uint32_T u[2]);
static boolean_T Cinematique_ROS_is_valid_state(const uint32_T mt[625]);
static void Cinematique__eml_rand_mt19937ar(const uint32_T state[625], uint32_T
  b_state[625], real_T *r);
static void Cinematique_ROS_rand(real_T r[5]);
static y_robotics_manip_internal_Rig_T *Cinema_RigidBody_RigidBody_k1e1
  (y_robotics_manip_internal_Rig_T *obj);
static y_robotics_manip_internal_Rig_T *Cinem_RigidBody_RigidBody_k1e1w
  (y_robotics_manip_internal_Rig_T *obj);
static y_robotics_manip_internal_Rig_T *Cine_RigidBody_RigidBody_k1e1wu
  (y_robotics_manip_internal_Rig_T *obj);
static void Ci_RigidBodyTree_clearAllBodies(ab_robotics_manip_internal_Ri_T *obj,
  y_robotics_manip_internal_Rig_T *iobj_0);
static boolean_T Cinematique_ROS_strcmp(const emxArray_char_T_Cinematique_R_T *a,
  const emxArray_char_T_Cinematique_R_T *b);
static m_robotics_manip_internal_Col_T *Cinematique_R_CollisionSet_copy
  (m_robotics_manip_internal_Col_T *obj, m_robotics_manip_internal_Col_T *iobj_0);
static real_T RigidBodyTree_findBodyIndexByNa(ab_robotics_manip_internal_Ri_T
  *obj, const emxArray_char_T_Cinematique_R_T *bodyname);
static void Cinematique_ROS_emxFree_real_T(emxArray_real_T_Cinematique_R_T
  **pEmxArray);
static c_rigidBodyJoint_Cinematique__T *Cinematique_rigidBodyJoint_copy(const
  c_rigidBodyJoint_Cinematique__T *obj, c_rigidBodyJoint_Cinematique__T *iobj_0);
static void Cinematiq_RigidBodyTree_addBody(ab_robotics_manip_internal_Ri_T *obj,
  w_robotics_manip_internal_Rig_T *bodyin, const emxArray_char_T_Cinematique_R_T
  *parentName, m_robotics_manip_internal_Col_T *iobj_0,
  c_rigidBodyJoint_Cinematique__T *iobj_1, y_robotics_manip_internal_Rig_T
  *iobj_2);
static void inverseKinematics_set_SolverAlg(b_inverseKinematics_Cinematiq_T *obj,
  h_robotics_core_internal_Damp_T *iobj_0);
static void Cinematique_RO_SystemCore_setup(robotics_slmanip_internal_blo_T *obj);
static void RigidBodyTree_get_JointPosition(ab_robotics_manip_internal_Ri_T *obj,
  emxArray_real_T_Cinematique_R_T *limits);
static void Cinematique_ROS_eml_find(const boolean_T x[4], int32_T i_data[],
  int32_T *i_size);
static void Cinematique_ROS_tic(real_T *tstart_tv_sec, real_T *tstart_tv_nsec);
static void C_RigidBodyTree_ancestorIndices(ab_robotics_manip_internal_Ri_T *obj,
  y_robotics_manip_internal_Rig_T *body, emxArray_real_T_Cinematique_R_T
  *indices);
static void RigidBodyTree_kinematicPathInte(ab_robotics_manip_internal_Ri_T *obj,
  y_robotics_manip_internal_Rig_T *body1, y_robotics_manip_internal_Rig_T *body2,
  emxArray_real_T_Cinematique_R_T *indices);
static void Cinematique_ROS_eye(real_T b_I[16]);
static void Ci_rigidBodyJoint_get_JointAxis(const
  c_rigidBodyJoint_Cinematique__T *obj, real_T ax[3]);
static void Cinematique_ROS_cat(real_T varargin_1, real_T varargin_2, real_T
  varargin_3, real_T varargin_4, real_T varargin_5, real_T varargin_6, real_T
  varargin_7, real_T varargin_8, real_T varargin_9, real_T y[9]);
static void Cinematique_ROS_mtimes(const real_T A[36], const
  emxArray_real_T_Cinematique_R_T *B, emxArray_real_T_Cinematique_R_T *C);
static void RigidBodyTree_efficientFKAndJac(ab_robotics_manip_internal_Ri_T *obj,
  const real_T qv[4], const emxArray_char_T_Cinematique_R_T *body1Name, real_T
  T_data[], int32_T T_size[2], emxArray_real_T_Cinematique_R_T *Jac);
static creal_T Cinematique_ROS_sqrt(const creal_T x);
static real_T Cinematique_ROS_xnrm2(int32_T n, const real_T x[9], int32_T ix0);
static real_T Cinematique_ROS_xdotc(int32_T n, const real_T x[9], int32_T ix0,
  const real_T y[9], int32_T iy0);
static void Cinematique_ROS_xaxpy(int32_T n, real_T a, int32_T ix0, const real_T
  y[9], int32_T iy0, real_T b_y[9]);
static real_T Cinematique_ROS_xnrm2_k(const real_T x[3], int32_T ix0);
static void Cinematique_ROS_xaxpy_k1e(int32_T n, real_T a, const real_T x[9],
  int32_T ix0, real_T y[3], int32_T iy0);
static void Cinematique_ROS_xaxpy_k1(int32_T n, real_T a, const real_T x[3],
  int32_T ix0, const real_T y[9], int32_T iy0, real_T b_y[9]);
static void Cinematique_ROS_xswap(const real_T x[9], int32_T ix0, int32_T iy0,
  real_T b_x[9]);
static void Cinematique_ROS_xrotg(real_T a, real_T b, real_T *b_a, real_T *b_b,
  real_T *c, real_T *s);
static void Cinematique_ROS_xrot(const real_T x[9], int32_T ix0, int32_T iy0,
  real_T c, real_T s, real_T b_x[9]);
static void Cinematique_ROS_svd(const real_T A[9], real_T U[9], real_T s[3],
  real_T V[9]);
static void Cinematiq_IKHelpers_computeCost(const real_T x[4],
  f_robotics_manip_internal_IKE_T *args, real_T *cost, real_T W[36],
  emxArray_real_T_Cinematique_R_T *Jac, f_robotics_manip_internal_IKE_T **b_args);
static void Cinematique_R_emxInit_boolean_T(emxArray_boolean_T_Cinematiqu_T
  **pEmxArray, int32_T numDimensions);
static void Cinematique_ROS_emxInit_int32_T(emxArray_int32_T_Cinematique__T
  **pEmxArray, int32_T numDimensions);
static void Cin_emxEnsureCapacity_boolean_T(emxArray_boolean_T_Cinematiqu_T
  *emxArray, int32_T oldNumel);
static void Cinematique_ROS_mtimes_k(const emxArray_real_T_Cinematique_R_T *A,
  const real_T B[4], emxArray_real_T_Cinematique_R_T *C);
static void Cinem_emxEnsureCapacity_int32_T(emxArray_int32_T_Cinematique__T
  *emxArray, int32_T oldNumel);
static void Cinematique_ROS_mtimes_k1(const real_T A[16], const
  emxArray_real_T_Cinematique_R_T *B, emxArray_real_T_Cinematique_R_T *C);
static real_T Cine_IKHelpers_evaluateSolution(const
  f_robotics_manip_internal_IKE_T *args);
static real_T Cinematique_ROS_toc(real_T tstart_tv_sec, real_T tstart_tv_nsec);
static void Cinematique_ROS_mtimes_k1e(const emxArray_real_T_Cinematique_R_T *A,
  const emxArray_real_T_Cinematique_R_T *B, emxArray_real_T_Cinematique_R_T *C);
static real_T Cinematique_ROS_xnrm2_k1(int32_T n, const
  emxArray_real_T_Cinematique_R_T *x, int32_T ix0);
static void Cinematique_ROS_emxFree_int32_T(emxArray_int32_T_Cinematique__T
  **pEmxArray);
static void Cinematique_ROS_xgeqp3(const emxArray_real_T_Cinematique_R_T *A,
  emxArray_real_T_Cinematique_R_T *b_A, emxArray_real_T_Cinematique_R_T *tau,
  emxArray_int32_T_Cinematique__T *jpvt);
static void Cinematique_ROS_xzgetrf(int32_T m, int32_T n, const
  emxArray_real_T_Cinematique_R_T *A, int32_T lda,
  emxArray_real_T_Cinematique_R_T *b_A, emxArray_int32_T_Cinematique__T *ipiv,
  int32_T *info);
static void Cinematique_ROS_xtrsm(int32_T m, int32_T n, const
  emxArray_real_T_Cinematique_R_T *A, int32_T lda, const
  emxArray_real_T_Cinematique_R_T *B, int32_T ldb,
  emxArray_real_T_Cinematique_R_T *b_B);
static void Cinematique_ROS_mldivide(const emxArray_real_T_Cinematique_R_T *A,
  const emxArray_real_T_Cinematique_R_T *B, emxArray_real_T_Cinematique_R_T *Y);
static real_T Cinematique_ROS_norm_k(const real_T x[4]);
static void Cinematique_R_emxFree_boolean_T(emxArray_boolean_T_Cinematiqu_T
  **pEmxArray);
static boolean_T DampedBFGSwGradientProjection_a(const
  h_robotics_core_internal_Damp_T *obj, const real_T Hg[4], const
  emxArray_real_T_Cinematique_R_T *alpha);
static void Cinematique_ROS_inv(const emxArray_real_T_Cinematique_R_T *x,
  emxArray_real_T_Cinematique_R_T *y);
static void Cinematique_ROS_diag(const emxArray_real_T_Cinematique_R_T *v,
  emxArray_real_T_Cinematique_R_T *d);
static void Cinematique_ROS_maximum(const emxArray_real_T_Cinematique_R_T *x,
  real_T *ex, int32_T *idx);
static boolean_T Cinematique_ROS_any(const emxArray_boolean_T_Cinematiqu_T *x);
static void Cinematique_ROS_eml_find_k(const emxArray_boolean_T_Cinematiqu_T *x,
  emxArray_int32_T_Cinematique__T *i);
static void Cinematique_ROS_minimum(const emxArray_real_T_Cinematique_R_T *x,
  real_T *ex, int32_T *idx);
static boolean_T Cinematique__isPositiveDefinite(const real_T B[16]);
static void Cinematique_ROS_mtimes_k1e1w(const emxArray_real_T_Cinematique_R_T
  *A, const real_T B[16], emxArray_real_T_Cinematique_R_T *C);
static void Cinematique_ROS_mtimes_k1e1(const emxArray_real_T_Cinematique_R_T *A,
  const emxArray_real_T_Cinematique_R_T *B, emxArray_real_T_Cinematique_R_T *C);
static void DampedBFGSwGradientProjection_s(h_robotics_core_internal_Damp_T *obj,
  real_T xSol[4], c_robotics_core_internal_NLPS_T *exitFlag, real_T *err, real_T
  *iter);
static real_T Cinematique_ROS_genrandu_k(uint32_T mt[625]);
static real_T Cinematiq_eml_rand_mt19937ar_k1(uint32_T state[625]);
static void Cinematique_ROS_randn(const real_T varargin_1[2],
  emxArray_real_T_Cinematique_R_T *r);
static void Cinematique_ROS_rand_k(real_T varargin_1,
  emxArray_real_T_Cinematique_R_T *r);
static void Cinema_NLPSolverInterface_solve(h_robotics_core_internal_Damp_T *obj,
  const real_T seed[4], real_T xSol[4], real_T *solutionInfo_Iterations, real_T *
  solutionInfo_RRAttempts, real_T *solutionInfo_Error, real_T
  *solutionInfo_ExitFlag, char_T solutionInfo_Status_data[], int32_T
  solutionInfo_Status_size[2]);
static void Cinem_inverseKinematics_solve_k(b_inverseKinematics_Cinematiq_T *obj,
  real_T initialGuess[4], real_T *solutionInfo_Iterations, real_T
  *solutionInfo_NumRandomRestarts, real_T *solutionInfo_PoseErrorNorm, real_T
  *solutionInfo_ExitFlag, char_T solutionInfo_Status_data[], int32_T
  solutionInfo_Status_size[2]);
static void Cine_inverseKinematics_stepImpl(b_inverseKinematics_Cinematiq_T *obj,
  const real_T tform[16], const real_T weights[6], const real_T initialGuess[4],
  real_T QSol[4], real_T *solutionInfo_Iterations, real_T
  *solutionInfo_PoseErrorNorm, real_T *solutionInfo_ExitFlag, char_T
  solutionInfo_Status_data[], int32_T solutionInfo_Status_size[2]);
static void emxFreeStruct_c_rigidBodyJoint(c_rigidBodyJoint_Cinematique__T
  *pStruct);
static void emxFreeStruct_m_robotics_manip_(m_robotics_manip_internal_Col_T
  *pStruct);
static void emxFreeStruct_w_robotics_manip_(w_robotics_manip_internal_Rig_T
  *pStruct);
static void emxFreeMatrix_w_robotics_manip_(w_robotics_manip_internal_Rig_T
  pMatrix[5]);
static void emxFreeStruct_x_robotics_manip_(x_robotics_manip_internal_Rig_T
  *pStruct);
static void emxFreeStruct_f_robotics_manip_(f_robotics_manip_internal_IKE_T
  *pStruct);
static void emxFreeStruct_y_robotics_manip_(y_robotics_manip_internal_Rig_T
  *pStruct);
static void emxFreeMatrix_y_robotics_manip_(y_robotics_manip_internal_Rig_T
  pMatrix[5]);
static void emxFreeStruct_ab_robotics_manip(ab_robotics_manip_internal_Ri_T
  *pStruct);
static void emxFreeMatrix_m_robotics_manip_(m_robotics_manip_internal_Col_T
  pMatrix[6]);
static void emxFreeMatrix_c_rigidBodyJoint(c_rigidBodyJoint_Cinematique__T
  pMatrix[5]);
static void emxFreeStruct_h_robotics_core_i(h_robotics_core_internal_Damp_T
  *pStruct);
static void emxFreeStruct_b_inverseKinemati(b_inverseKinematics_Cinematiq_T
  *pStruct);
static void emxFreeStruct_robotics_slmanip_(robotics_slmanip_internal_blo_T
  *pStruct);
int32_T div_s32_floor(int32_T numerator, int32_T denominator)
{
  int32_T quotient;
  uint32_T absDenominator;
  uint32_T absNumerator;
  uint32_T tempAbsQuotient;
  boolean_T quotientNeedsNegation;
  if (denominator == 0) {
    quotient = numerator >= 0 ? MAX_int32_T : MIN_int32_T;

    /* Divide by zero handler */
  } else {
    absNumerator = numerator < 0 ? ~static_cast<uint32_T>(numerator) + 1U :
      static_cast<uint32_T>(numerator);
    absDenominator = denominator < 0 ? ~static_cast<uint32_T>(denominator) + 1U :
      static_cast<uint32_T>(denominator);
    quotientNeedsNegation = ((numerator < 0) != (denominator < 0));
    tempAbsQuotient = absNumerator / absDenominator;
    if (quotientNeedsNegation) {
      absNumerator %= absDenominator;
      if (absNumerator > 0U) {
        tempAbsQuotient++;
      }
    }

    quotient = quotientNeedsNegation ? -static_cast<int32_T>(tempAbsQuotient) :
      static_cast<int32_T>(tempAbsQuotient);
  }

  return quotient;
}

static void Cinematique_ROS_emxInit_char_T(emxArray_char_T_Cinematique_R_T
  **pEmxArray, int32_T numDimensions)
{
  emxArray_char_T_Cinematique_R_T *emxArray;
  *pEmxArray = (emxArray_char_T_Cinematique_R_T *)malloc(sizeof
    (emxArray_char_T_Cinematique_R_T));
  emxArray = *pEmxArray;
  emxArray->data = (char_T *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int32_T *)malloc(sizeof(int32_T) * numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (Cinematique_ROS_B.i_m = 0; Cinematique_ROS_B.i_m < numDimensions;
       Cinematique_ROS_B.i_m++) {
    emxArray->size[Cinematique_ROS_B.i_m] = 0;
  }
}

static void Cinematique_ROS_emxInit_real_T(emxArray_real_T_Cinematique_R_T
  **pEmxArray, int32_T numDimensions)
{
  emxArray_real_T_Cinematique_R_T *emxArray;
  *pEmxArray = (emxArray_real_T_Cinematique_R_T *)malloc(sizeof
    (emxArray_real_T_Cinematique_R_T));
  emxArray = *pEmxArray;
  emxArray->data = (real_T *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int32_T *)malloc(sizeof(int32_T) * numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (Cinematique_ROS_B.i_f = 0; Cinematique_ROS_B.i_f < numDimensions;
       Cinematique_ROS_B.i_f++) {
    emxArray->size[Cinematique_ROS_B.i_f] = 0;
  }
}

static void emxInitStruct_c_rigidBodyJoint(c_rigidBodyJoint_Cinematique__T
  *pStruct)
{
  Cinematique_ROS_emxInit_char_T(&pStruct->Type, 2);
  Cinematique_ROS_emxInit_real_T(&pStruct->MotionSubspace, 2);
  Cinematique_ROS_emxInit_char_T(&pStruct->NameInternal, 2);
  Cinematique_ROS_emxInit_real_T(&pStruct->PositionLimitsInternal, 2);
  Cinematique_ROS_emxInit_real_T(&pStruct->HomePositionInternal, 1);
}

static void Cinemati_emxInit_unnamed_struct(emxArray_unnamed_struct_Cinem_T
  **pEmxArray, int32_T numDimensions)
{
  emxArray_unnamed_struct_Cinem_T *emxArray;
  int32_T i;
  *pEmxArray = (emxArray_unnamed_struct_Cinem_T *)malloc(sizeof
    (emxArray_unnamed_struct_Cinem_T));
  emxArray = *pEmxArray;
  emxArray->data = (l_robotics_manip_internal_Col_T **)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int32_T *)malloc(sizeof(int32_T) * numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

static void emxInitStruct_m_robotics_manip_(m_robotics_manip_internal_Col_T
  *pStruct)
{
  Cinemati_emxInit_unnamed_struct(&pStruct->CollisionGeometries, 2);
}

static void emxInitStruct_w_robotics_manip_(w_robotics_manip_internal_Rig_T
  *pStruct)
{
  Cinematique_ROS_emxInit_char_T(&pStruct->NameInternal, 2);
  emxInitStruct_c_rigidBodyJoint(&pStruct->JointInternal);
  emxInitStruct_m_robotics_manip_(&pStruct->CollisionsInternal);
}

static void emxInitMatrix_w_robotics_manip_(w_robotics_manip_internal_Rig_T
  pMatrix[5])
{
  int32_T i;
  for (i = 0; i < 5; i++) {
    emxInitStruct_w_robotics_manip_(&pMatrix[i]);
  }
}

static void emxInitStruct_x_robotics_manip_(x_robotics_manip_internal_Rig_T
  *pStruct)
{
  emxInitStruct_w_robotics_manip_(&pStruct->Base);
  emxInitMatrix_w_robotics_manip_(pStruct->_pobj0);
}

static void emxInitStruct_f_robotics_manip_(f_robotics_manip_internal_IKE_T
  *pStruct)
{
  Cinematique_ROS_emxInit_char_T(&pStruct->BodyName, 2);
  Cinematique_ROS_emxInit_real_T(&pStruct->ErrTemp, 1);
  Cinematique_ROS_emxInit_real_T(&pStruct->GradTemp, 1);
}

static void emxInitStruct_y_robotics_manip_(y_robotics_manip_internal_Rig_T
  *pStruct)
{
  Cinematique_ROS_emxInit_char_T(&pStruct->NameInternal, 2);
  emxInitStruct_m_robotics_manip_(&pStruct->_pobj0);
  emxInitStruct_c_rigidBodyJoint(&pStruct->_pobj1);
}

static void emxInitMatrix_y_robotics_manip_(y_robotics_manip_internal_Rig_T
  pMatrix[5])
{
  int32_T i;
  for (i = 0; i < 5; i++) {
    emxInitStruct_y_robotics_manip_(&pMatrix[i]);
  }
}

static void emxInitStruct_ab_robotics_manip(ab_robotics_manip_internal_Ri_T
  *pStruct)
{
  emxInitStruct_y_robotics_manip_(&pStruct->Base);
  emxInitMatrix_y_robotics_manip_(pStruct->_pobj0);
}

static void emxInitMatrix_m_robotics_manip_(m_robotics_manip_internal_Col_T
  pMatrix[6])
{
  int32_T i;
  for (i = 0; i < 6; i++) {
    emxInitStruct_m_robotics_manip_(&pMatrix[i]);
  }
}

static void emxInitMatrix_c_rigidBodyJoint(c_rigidBodyJoint_Cinematique__T
  pMatrix[5])
{
  int32_T i;
  for (i = 0; i < 5; i++) {
    emxInitStruct_c_rigidBodyJoint(&pMatrix[i]);
  }
}

static void emxInitStruct_h_robotics_core_i(h_robotics_core_internal_Damp_T
  *pStruct)
{
  Cinematique_ROS_emxInit_real_T(&pStruct->ConstraintMatrix, 2);
  Cinematique_ROS_emxInit_real_T(&pStruct->ConstraintBound, 1);
}

static void emxInitStruct_b_inverseKinemati(b_inverseKinematics_Cinematiq_T
  *pStruct)
{
  Cinematique_ROS_emxInit_real_T(&pStruct->Limits, 2);
  emxInitStruct_f_robotics_manip_(&pStruct->_pobj0);
  emxInitStruct_ab_robotics_manip(&pStruct->_pobj1);
  emxInitMatrix_m_robotics_manip_(pStruct->_pobj2);
  emxInitMatrix_c_rigidBodyJoint(pStruct->_pobj3);
  emxInitMatrix_y_robotics_manip_(pStruct->_pobj4);
  emxInitStruct_h_robotics_core_i(&pStruct->_pobj5);
}

static void emxInitStruct_robotics_slmanip_(robotics_slmanip_internal_blo_T
  *pStruct)
{
  emxInitStruct_x_robotics_manip_(&pStruct->TreeInternal);
  emxInitStruct_b_inverseKinemati(&pStruct->IKInternal);
}

static void Cinema_emxEnsureCapacity_char_T(emxArray_char_T_Cinematique_R_T
  *emxArray, int32_T oldNumel)
{
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  Cinematique_ROS_B.newNumel_m = 1;
  for (Cinematique_ROS_B.i_j = 0; Cinematique_ROS_B.i_j <
       emxArray->numDimensions; Cinematique_ROS_B.i_j++) {
    Cinematique_ROS_B.newNumel_m *= emxArray->size[Cinematique_ROS_B.i_j];
  }

  if (Cinematique_ROS_B.newNumel_m > emxArray->allocatedSize) {
    Cinematique_ROS_B.i_j = emxArray->allocatedSize;
    if (Cinematique_ROS_B.i_j < 16) {
      Cinematique_ROS_B.i_j = 16;
    }

    while (Cinematique_ROS_B.i_j < Cinematique_ROS_B.newNumel_m) {
      if (Cinematique_ROS_B.i_j > 1073741823) {
        Cinematique_ROS_B.i_j = MAX_int32_T;
      } else {
        Cinematique_ROS_B.i_j <<= 1;
      }
    }

    newData = calloc(static_cast<uint32_T>(Cinematique_ROS_B.i_j), sizeof(char_T));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(char_T) * oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }

    emxArray->data = (char_T *)newData;
    emxArray->allocatedSize = Cinematique_ROS_B.i_j;
    emxArray->canFreeData = true;
  }
}

static void Cinema_emxEnsureCapacity_real_T(emxArray_real_T_Cinematique_R_T
  *emxArray, int32_T oldNumel)
{
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  Cinematique_ROS_B.newNumel = 1;
  for (Cinematique_ROS_B.i_b = 0; Cinematique_ROS_B.i_b <
       emxArray->numDimensions; Cinematique_ROS_B.i_b++) {
    Cinematique_ROS_B.newNumel *= emxArray->size[Cinematique_ROS_B.i_b];
  }

  if (Cinematique_ROS_B.newNumel > emxArray->allocatedSize) {
    Cinematique_ROS_B.i_b = emxArray->allocatedSize;
    if (Cinematique_ROS_B.i_b < 16) {
      Cinematique_ROS_B.i_b = 16;
    }

    while (Cinematique_ROS_B.i_b < Cinematique_ROS_B.newNumel) {
      if (Cinematique_ROS_B.i_b > 1073741823) {
        Cinematique_ROS_B.i_b = MAX_int32_T;
      } else {
        Cinematique_ROS_B.i_b <<= 1;
      }
    }

    newData = calloc(static_cast<uint32_T>(Cinematique_ROS_B.i_b), sizeof(real_T));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(real_T) * oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }

    emxArray->data = (real_T *)newData;
    emxArray->allocatedSize = Cinematique_ROS_B.i_b;
    emxArray->canFreeData = true;
  }
}

static void emxEnsureCapacity_unnamed_struc(emxArray_unnamed_struct_Cinem_T
  *emxArray, int32_T oldNumel)
{
  int32_T i;
  int32_T newNumel;
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  newNumel = 1;
  for (i = 0; i < emxArray->numDimensions; i++) {
    newNumel *= emxArray->size[i];
  }

  if (newNumel > emxArray->allocatedSize) {
    i = emxArray->allocatedSize;
    if (i < 16) {
      i = 16;
    }

    while (i < newNumel) {
      if (i > 1073741823) {
        i = MAX_int32_T;
      } else {
        i <<= 1;
      }
    }

    newData = calloc(static_cast<uint32_T>(i), sizeof
                     (l_robotics_manip_internal_Col_T *));
    if (emxArray->data != NULL) {
      memcpy(newData, (void *)emxArray->data, sizeof
             (l_robotics_manip_internal_Col_T *) * oldNumel);
      if (emxArray->canFreeData) {
        free((void *)emxArray->data);
      }
    }

    emxArray->data = (l_robotics_manip_internal_Col_T **)newData;
    emxArray->allocatedSize = i;
    emxArray->canFreeData = true;
  }
}

static void Cinemati_emxFree_unnamed_struct(emxArray_unnamed_struct_Cinem_T
  **pEmxArray)
{
  if (*pEmxArray != (emxArray_unnamed_struct_Cinem_T *)NULL) {
    if (((*pEmxArray)->data != (l_robotics_manip_internal_Col_T **)NULL) &&
        (*pEmxArray)->canFreeData) {
      free((void *)(*pEmxArray)->data);
    }

    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = (emxArray_unnamed_struct_Cinem_T *)NULL;
  }
}

static void Cinematique_ROS_emxFree_char_T(emxArray_char_T_Cinematique_R_T
  **pEmxArray)
{
  if (*pEmxArray != (emxArray_char_T_Cinematique_R_T *)NULL) {
    if (((*pEmxArray)->data != (char_T *)NULL) && (*pEmxArray)->canFreeData) {
      free((*pEmxArray)->data);
    }

    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = (emxArray_char_T_Cinematique_R_T *)NULL;
  }
}

static w_robotics_manip_internal_Rig_T *Cinematique_RigidBody_RigidBody
  (w_robotics_manip_internal_Rig_T *obj)
{
  static const real_T tmp_2[36] = { 0.0027502023221753161,
    0.00013721164388727394, 0.0024884917125739987, 0.0, 0.028353516750173179,
    -0.0016865177898651368, 0.00013721164388727394, 0.0066511188638790578,
    -8.5354961040796825E-5, -0.028353516750173179, 0.0, -0.029695696707574566,
    0.0024884917125739987, -8.5354961040796825E-5, 0.0041201304643911571,
    0.0016865177898651368, 0.029695696707574566, 0.0, 0.0, -0.028353516750173179,
    0.0016865177898651368, 0.41420628802, 0.0, 0.0, 0.028353516750173179, 0.0,
    0.029695696707574566, 0.0, 0.41420628802, 0.0, -0.0016865177898651368,
    -0.029695696707574566, 0.0, 0.0, 0.0, 0.41420628802 };

  static const real_T tmp_9[36] = { 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  static const real_T tmp_7[16] = { 1.0, 0.0, -0.0, 0.0, -0.0,
    6.123233995736766E-17, -1.0, 0.0, 0.0, 1.0, 6.123233995736766E-17, 0.0, 0.0,
    0.05300000000000004, -0.13, 1.0 };

  static const real_T tmp_8[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  static const real_T tmp_1[9] = { 0.0027502023221753161, 0.00013721164388727394,
    0.0024884917125739987, 0.00013721164388727394, 0.0066511188638790578,
    -8.5354961040796825E-5, 0.0024884917125739987, -8.5354961040796825E-5,
    0.0041201304643911571 };

  static const char_T tmp_6[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const char_T tmp_5[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_4[6] = { 'J', 'o', 'i', 'n', 't', '2' };

  static const char_T tmp_0[5] = { 'B', 'o', 'd', 'y', '2' };

  static const int8_T tmp_3[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    1 };

  emxArray_char_T_Cinematique_R_T *switch_expression;
  emxArray_unnamed_struct_Cinem_T *f;
  m_robotics_manip_internal_Col_T *obj_0;
  w_robotics_manip_internal_Rig_T *b_obj;
  real_T poslim_data[12];
  real_T c;
  int32_T b_kstr;
  int32_T exitg1;
  int32_T loop_ub;
  char_T b_0[9];
  char_T b[8];
  int8_T msubspace_data[36];
  int8_T tmp[6];
  boolean_T b_bool;
  b_obj = obj;
  b_kstr = obj->NameInternal->size[0] * obj->NameInternal->size[1];
  obj->NameInternal->size[0] = 1;
  obj->NameInternal->size[1] = 5;
  Cinema_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp_0[b_kstr];
  }

  obj->ParentIndex = 1.0;
  obj->MassInternal = 0.41420628802;
  obj->CenterOfMassInternal[0] = 0.071693012796900629;
  obj->CenterOfMassInternal[1] = -0.0040716856277751706;
  obj->CenterOfMassInternal[2] = -0.068452646833802117;
  for (b_kstr = 0; b_kstr < 9; b_kstr++) {
    obj->InertiaInternal[b_kstr] = tmp_1[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 36; b_kstr++) {
    obj->SpatialInertia[b_kstr] = tmp_2[b_kstr];
  }

  obj->JointInternal.InTree = false;
  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.JointToParentTransform[b_kstr] = tmp_3[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.ChildToJointTransform[b_kstr] = tmp_3[b_kstr];
  }

  b_kstr = obj->JointInternal.NameInternal->size[0] *
    obj->JointInternal.NameInternal->size[1];
  obj->JointInternal.NameInternal->size[0] = 1;
  obj->JointInternal.NameInternal->size[1] = 6;
  Cinema_emxEnsureCapacity_char_T(obj->JointInternal.NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.NameInternal->data[b_kstr] = tmp_4[b_kstr];
  }

  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  Cinema_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_5[b_kstr];
  }

  Cinematique_ROS_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  Cinema_emxEnsureCapacity_char_T(switch_expression, b_kstr);
  loop_ub = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1]
    - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = obj->JointInternal.Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp_5[b_kstr];
  }

  b_bool = false;
  if (switch_expression->size[1] == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        if (switch_expression->data[b_kstr - 1] != b[b_kstr - 1]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (b_bool) {
    b_kstr = 0;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      b_0[b_kstr] = tmp_6[b_kstr];
    }

    b_bool = false;
    if (switch_expression->size[1] == 9) {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          if (switch_expression->data[b_kstr - 1] != b_0[b_kstr - 1]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (b_bool) {
      b_kstr = 1;
    } else {
      b_kstr = -1;
    }
  }

  Cinematique_ROS_emxFree_char_T(&switch_expression);
  switch (b_kstr) {
   case 0:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 1;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 0;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    poslim_data[0] = -3.1415926535897931;
    poslim_data[1] = 3.1415926535897931;
    obj->JointInternal.VelocityNumber = 1.0;
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 0;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 1;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    poslim_data[0] = -0.5;
    poslim_data[1] = 0.5;
    obj->JointInternal.VelocityNumber = 1.0;
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = 0;
    }

    poslim_data[0] = 0.0;
    poslim_data[1] = 0.0;
    obj->JointInternal.VelocityNumber = 0.0;
    obj->JointInternal.PositionNumber = 0.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  Cinema_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  b_kstr = obj->JointInternal.PositionLimitsInternal->size[0] *
    obj->JointInternal.PositionLimitsInternal->size[1];
  obj->JointInternal.PositionLimitsInternal->size[0] = 1;
  obj->JointInternal.PositionLimitsInternal->size[1] = 2;
  Cinema_emxEnsureCapacity_real_T(obj->JointInternal.PositionLimitsInternal,
    b_kstr);
  for (b_kstr = 0; b_kstr < 2; b_kstr++) {
    obj->JointInternal.PositionLimitsInternal->data[b_kstr] = poslim_data[b_kstr];
  }

  b_kstr = obj->JointInternal.HomePositionInternal->size[0];
  obj->JointInternal.HomePositionInternal->size[0] = 1;
  Cinema_emxEnsureCapacity_real_T(obj->JointInternal.HomePositionInternal,
    b_kstr);
  for (b_kstr = 0; b_kstr < 1; b_kstr++) {
    obj->JointInternal.HomePositionInternal->data[0] = 0.0;
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.JointToParentTransform[b_kstr] = tmp_7[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.ChildToJointTransform[b_kstr] = tmp_8[b_kstr];
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  Cinema_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = tmp_9[b_kstr];
  }

  obj->JointInternal.InTree = true;
  b_kstr = obj->JointInternal.PositionLimitsInternal->size[0] *
    obj->JointInternal.PositionLimitsInternal->size[1];
  obj->JointInternal.PositionLimitsInternal->size[0] = 1;
  obj->JointInternal.PositionLimitsInternal->size[1] = 2;
  Cinema_emxEnsureCapacity_real_T(obj->JointInternal.PositionLimitsInternal,
    b_kstr);
  obj->JointInternal.PositionLimitsInternal->data[0] = 1.0471975511965976;
  obj->JointInternal.PositionLimitsInternal->data
    [obj->JointInternal.PositionLimitsInternal->size[0]] = 3.1415926535897931;
  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 1.0;
  Cinemati_emxInit_unnamed_struct(&f, 2);
  b_kstr = obj->JointInternal.HomePositionInternal->size[0];
  obj->JointInternal.HomePositionInternal->size[0] = 1;
  Cinema_emxEnsureCapacity_real_T(obj->JointInternal.HomePositionInternal,
    b_kstr);
  obj->JointInternal.HomePositionInternal->data[0] = 2.0943951023931953;
  obj_0 = &obj->CollisionsInternal;
  obj->CollisionsInternal.Size = 0.0;
  obj->CollisionsInternal.MaxElements = 0.0;
  b_kstr = f->size[0] * f->size[1];
  f->size[0] = 1;
  f->size[1] = static_cast<int32_T>(obj->CollisionsInternal.MaxElements);
  emxEnsureCapacity_unnamed_struc(f, b_kstr);
  b_kstr = obj->CollisionsInternal.CollisionGeometries->size[0] *
    obj->CollisionsInternal.CollisionGeometries->size[1];
  obj->CollisionsInternal.CollisionGeometries->size[0] = f->size[0];
  obj->CollisionsInternal.CollisionGeometries->size[1] = f->size[1];
  emxEnsureCapacity_unnamed_struc(obj->CollisionsInternal.CollisionGeometries,
    b_kstr);
  collisioncodegen_makeBox(0.0, 0.0, 0.0);
  c = obj->CollisionsInternal.MaxElements;
  loop_ub = static_cast<int32_T>(c) - 1;
  Cinemati_emxFree_unnamed_struct(&f);
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    obj_0->CollisionGeometries->data[b_kstr] = &obj->CollisionsInternal._pobj0;
  }

  return b_obj;
}

static w_robotics_manip_internal_Rig_T *Cinematiq_RigidBody_RigidBody_k
  (w_robotics_manip_internal_Rig_T *obj)
{
  static const real_T tmp_2[36] = { 0.00092757091116331457,
    -6.9051472661594285E-5, 0.0012304241400885468, 0.0, 0.015260329314862712,
    0.00087956773259004477, -6.9051472661594285E-5, 0.0039846605904797893,
    3.1007932047924141E-5, -0.015260329314862712, 0.0, -0.023853969006140521,
    0.0012304241400885468, 3.1007932047924141E-5, 0.0031844743558810812,
    -0.00087956773259004477, 0.023853969006140521, 0.0, 0.0,
    -0.015260329314862712, -0.00087956773259004477, 0.33844006481999994, 0.0,
    0.0, 0.015260329314862712, 0.0, 0.023853969006140521, 0.0,
    0.33844006481999994, 0.0, 0.00087956773259004477, -0.023853969006140521, 0.0,
    0.0, 0.0, 0.33844006481999994 };

  static const real_T tmp_9[36] = { 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  static const real_T tmp_7[16] = { 1.0, 0.0, -0.0, 0.0, 0.0, -1.0,
    1.2246467991473532E-16, 0.0, 0.0, -1.2246467991473532E-16, -1.0, 0.0,
    0.16000000000000006, 5.8456999999956238E-5, -0.087999999999999967, 1.0 };

  static const real_T tmp_8[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  static const real_T tmp_1[9] = { 0.00092757091116331457,
    -6.9051472661594285E-5, 0.0012304241400885468, -6.9051472661594285E-5,
    0.0039846605904797893, 3.1007932047924141E-5, 0.0012304241400885468,
    3.1007932047924141E-5, 0.0031844743558810812 };

  static const char_T tmp_6[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const char_T tmp_5[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_4[6] = { 'J', 'o', 'i', 'n', 't', '3' };

  static const char_T tmp_0[5] = { 'B', 'o', 'd', 'y', '3' };

  static const int8_T tmp_3[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    1 };

  emxArray_char_T_Cinematique_R_T *switch_expression;
  emxArray_unnamed_struct_Cinem_T *f;
  m_robotics_manip_internal_Col_T *obj_0;
  w_robotics_manip_internal_Rig_T *b_obj;
  real_T poslim_data[12];
  real_T c;
  int32_T b_kstr;
  int32_T exitg1;
  int32_T loop_ub;
  char_T b_0[9];
  char_T b[8];
  int8_T msubspace_data[36];
  int8_T tmp[6];
  boolean_T b_bool;
  b_obj = obj;
  b_kstr = obj->NameInternal->size[0] * obj->NameInternal->size[1];
  obj->NameInternal->size[0] = 1;
  obj->NameInternal->size[1] = 5;
  Cinema_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp_0[b_kstr];
  }

  obj->ParentIndex = 2.0;
  obj->MassInternal = 0.33844006481999994;
  obj->CenterOfMassInternal[0] = 0.070482107426694016;
  obj->CenterOfMassInternal[1] = 0.0025988877323311135;
  obj->CenterOfMassInternal[2] = -0.045090197352901908;
  for (b_kstr = 0; b_kstr < 9; b_kstr++) {
    obj->InertiaInternal[b_kstr] = tmp_1[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 36; b_kstr++) {
    obj->SpatialInertia[b_kstr] = tmp_2[b_kstr];
  }

  obj->JointInternal.InTree = false;
  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.JointToParentTransform[b_kstr] = tmp_3[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.ChildToJointTransform[b_kstr] = tmp_3[b_kstr];
  }

  b_kstr = obj->JointInternal.NameInternal->size[0] *
    obj->JointInternal.NameInternal->size[1];
  obj->JointInternal.NameInternal->size[0] = 1;
  obj->JointInternal.NameInternal->size[1] = 6;
  Cinema_emxEnsureCapacity_char_T(obj->JointInternal.NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.NameInternal->data[b_kstr] = tmp_4[b_kstr];
  }

  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  Cinema_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_5[b_kstr];
  }

  Cinematique_ROS_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  Cinema_emxEnsureCapacity_char_T(switch_expression, b_kstr);
  loop_ub = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1]
    - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = obj->JointInternal.Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp_5[b_kstr];
  }

  b_bool = false;
  if (switch_expression->size[1] == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        if (switch_expression->data[b_kstr - 1] != b[b_kstr - 1]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (b_bool) {
    b_kstr = 0;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      b_0[b_kstr] = tmp_6[b_kstr];
    }

    b_bool = false;
    if (switch_expression->size[1] == 9) {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          if (switch_expression->data[b_kstr - 1] != b_0[b_kstr - 1]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (b_bool) {
      b_kstr = 1;
    } else {
      b_kstr = -1;
    }
  }

  Cinematique_ROS_emxFree_char_T(&switch_expression);
  switch (b_kstr) {
   case 0:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 1;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 0;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    poslim_data[0] = -3.1415926535897931;
    poslim_data[1] = 3.1415926535897931;
    obj->JointInternal.VelocityNumber = 1.0;
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 0;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 1;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    poslim_data[0] = -0.5;
    poslim_data[1] = 0.5;
    obj->JointInternal.VelocityNumber = 1.0;
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = 0;
    }

    poslim_data[0] = 0.0;
    poslim_data[1] = 0.0;
    obj->JointInternal.VelocityNumber = 0.0;
    obj->JointInternal.PositionNumber = 0.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  Cinema_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  b_kstr = obj->JointInternal.PositionLimitsInternal->size[0] *
    obj->JointInternal.PositionLimitsInternal->size[1];
  obj->JointInternal.PositionLimitsInternal->size[0] = 1;
  obj->JointInternal.PositionLimitsInternal->size[1] = 2;
  Cinema_emxEnsureCapacity_real_T(obj->JointInternal.PositionLimitsInternal,
    b_kstr);
  for (b_kstr = 0; b_kstr < 2; b_kstr++) {
    obj->JointInternal.PositionLimitsInternal->data[b_kstr] = poslim_data[b_kstr];
  }

  b_kstr = obj->JointInternal.HomePositionInternal->size[0];
  obj->JointInternal.HomePositionInternal->size[0] = 1;
  Cinema_emxEnsureCapacity_real_T(obj->JointInternal.HomePositionInternal,
    b_kstr);
  for (b_kstr = 0; b_kstr < 1; b_kstr++) {
    obj->JointInternal.HomePositionInternal->data[0] = 0.0;
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.JointToParentTransform[b_kstr] = tmp_7[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.ChildToJointTransform[b_kstr] = tmp_8[b_kstr];
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  Cinema_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = tmp_9[b_kstr];
  }

  obj->JointInternal.InTree = true;
  b_kstr = obj->JointInternal.PositionLimitsInternal->size[0] *
    obj->JointInternal.PositionLimitsInternal->size[1];
  obj->JointInternal.PositionLimitsInternal->size[0] = 1;
  obj->JointInternal.PositionLimitsInternal->size[1] = 2;
  Cinema_emxEnsureCapacity_real_T(obj->JointInternal.PositionLimitsInternal,
    b_kstr);
  obj->JointInternal.PositionLimitsInternal->data[0] = -1.9198621771937625;
  obj->JointInternal.PositionLimitsInternal->data
    [obj->JointInternal.PositionLimitsInternal->size[0]] = 0.0;
  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 1.0;
  Cinemati_emxInit_unnamed_struct(&f, 2);
  b_kstr = obj->JointInternal.HomePositionInternal->size[0];
  obj->JointInternal.HomePositionInternal->size[0] = 1;
  Cinema_emxEnsureCapacity_real_T(obj->JointInternal.HomePositionInternal,
    b_kstr);
  obj->JointInternal.HomePositionInternal->data[0] = -0.95993108859688125;
  obj_0 = &obj->CollisionsInternal;
  obj->CollisionsInternal.Size = 0.0;
  obj->CollisionsInternal.MaxElements = 0.0;
  b_kstr = f->size[0] * f->size[1];
  f->size[0] = 1;
  f->size[1] = static_cast<int32_T>(obj->CollisionsInternal.MaxElements);
  emxEnsureCapacity_unnamed_struc(f, b_kstr);
  b_kstr = obj->CollisionsInternal.CollisionGeometries->size[0] *
    obj->CollisionsInternal.CollisionGeometries->size[1];
  obj->CollisionsInternal.CollisionGeometries->size[0] = f->size[0];
  obj->CollisionsInternal.CollisionGeometries->size[1] = f->size[1];
  emxEnsureCapacity_unnamed_struc(obj->CollisionsInternal.CollisionGeometries,
    b_kstr);
  collisioncodegen_makeBox(0.0, 0.0, 0.0);
  c = obj->CollisionsInternal.MaxElements;
  loop_ub = static_cast<int32_T>(c) - 1;
  Cinemati_emxFree_unnamed_struct(&f);
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    obj_0->CollisionGeometries->data[b_kstr] = &obj->CollisionsInternal._pobj0;
  }

  return b_obj;
}

static w_robotics_manip_internal_Rig_T *Cinemati_RigidBody_RigidBody_k1
  (w_robotics_manip_internal_Rig_T *obj)
{
  static const real_T tmp_2[36] = { 0.0011140647394964876, 1.4367358751990404E-7,
    5.9358462211742927E-8, 0.0, 0.0053802774234008215, -0.00909661522722015,
    1.4367358751990404E-7, 0.00026893286506405407, -0.00039440410340773875,
    -0.0053802774234008215, 0.0, -1.7149169131106406E-6, 5.9358462211742927E-8,
    -0.00039440410340773875, 0.00087436792838748259, 0.00909661522722015,
    1.7149169131106406E-6, 0.0, 0.0, -0.0053802774234008215, 0.00909661522722015,
    0.125581734, 0.0, 0.0, 0.0053802774234008215, 0.0, 1.7149169131106406E-6,
    0.0, 0.125581734, 0.0, -0.00909661522722015, -1.7149169131106406E-6, 0.0,
    0.0, 0.0, 0.125581734 };

  static const real_T tmp_9[36] = { 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  static const real_T tmp_7[16] = { 1.0, 0.0, -0.0, 0.0, 0.0, -1.0,
    1.2246467991473532E-16, 0.0, 0.0, -1.2246467991473532E-16, -1.0, 0.0,
    0.16499999999999998, 0.0, -0.061000000000000026, 1.0 };

  static const real_T tmp_8[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  static const real_T tmp_1[9] = { 0.0011140647394964876, 1.4367358751990404E-7,
    5.9358462211742927E-8, 1.4367358751990404E-7, 0.00026893286506405407,
    -0.00039440410340773875, 5.9358462211742927E-8, -0.00039440410340773875,
    0.00087436792838748259 };

  static const char_T tmp_6[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const char_T tmp_5[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_4[6] = { 'J', 'o', 'i', 'n', 't', '4' };

  static const char_T tmp_0[5] = { 'B', 'o', 'd', 'y', '4' };

  static const int8_T tmp_3[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    1 };

  emxArray_char_T_Cinematique_R_T *switch_expression;
  emxArray_unnamed_struct_Cinem_T *f;
  m_robotics_manip_internal_Col_T *obj_0;
  w_robotics_manip_internal_Rig_T *b_obj;
  real_T poslim_data[12];
  real_T c;
  int32_T b_kstr;
  int32_T exitg1;
  int32_T loop_ub;
  char_T b_0[9];
  char_T b[8];
  int8_T msubspace_data[36];
  int8_T tmp[6];
  boolean_T b_bool;
  b_obj = obj;
  b_kstr = obj->NameInternal->size[0] * obj->NameInternal->size[1];
  obj->NameInternal->size[0] = 1;
  obj->NameInternal->size[1] = 5;
  Cinema_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp_0[b_kstr];
  }

  obj->ParentIndex = 3.0;
  obj->MassInternal = 0.125581734;
  obj->CenterOfMassInternal[0] = 1.3655783038563878E-5;
  obj->CenterOfMassInternal[1] = -0.0724358148074317;
  obj->CenterOfMassInternal[2] = -0.042842834320163323;
  for (b_kstr = 0; b_kstr < 9; b_kstr++) {
    obj->InertiaInternal[b_kstr] = tmp_1[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 36; b_kstr++) {
    obj->SpatialInertia[b_kstr] = tmp_2[b_kstr];
  }

  obj->JointInternal.InTree = false;
  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.JointToParentTransform[b_kstr] = tmp_3[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.ChildToJointTransform[b_kstr] = tmp_3[b_kstr];
  }

  b_kstr = obj->JointInternal.NameInternal->size[0] *
    obj->JointInternal.NameInternal->size[1];
  obj->JointInternal.NameInternal->size[0] = 1;
  obj->JointInternal.NameInternal->size[1] = 6;
  Cinema_emxEnsureCapacity_char_T(obj->JointInternal.NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.NameInternal->data[b_kstr] = tmp_4[b_kstr];
  }

  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  Cinema_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_5[b_kstr];
  }

  Cinematique_ROS_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  Cinema_emxEnsureCapacity_char_T(switch_expression, b_kstr);
  loop_ub = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1]
    - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = obj->JointInternal.Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp_5[b_kstr];
  }

  b_bool = false;
  if (switch_expression->size[1] == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        if (switch_expression->data[b_kstr - 1] != b[b_kstr - 1]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (b_bool) {
    b_kstr = 0;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      b_0[b_kstr] = tmp_6[b_kstr];
    }

    b_bool = false;
    if (switch_expression->size[1] == 9) {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          if (switch_expression->data[b_kstr - 1] != b_0[b_kstr - 1]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (b_bool) {
      b_kstr = 1;
    } else {
      b_kstr = -1;
    }
  }

  Cinematique_ROS_emxFree_char_T(&switch_expression);
  switch (b_kstr) {
   case 0:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 1;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 0;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    poslim_data[0] = -3.1415926535897931;
    poslim_data[1] = 3.1415926535897931;
    obj->JointInternal.VelocityNumber = 1.0;
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 0;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 1;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    poslim_data[0] = -0.5;
    poslim_data[1] = 0.5;
    obj->JointInternal.VelocityNumber = 1.0;
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = 0;
    }

    poslim_data[0] = 0.0;
    poslim_data[1] = 0.0;
    obj->JointInternal.VelocityNumber = 0.0;
    obj->JointInternal.PositionNumber = 0.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  Cinema_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  b_kstr = obj->JointInternal.PositionLimitsInternal->size[0] *
    obj->JointInternal.PositionLimitsInternal->size[1];
  obj->JointInternal.PositionLimitsInternal->size[0] = 1;
  obj->JointInternal.PositionLimitsInternal->size[1] = 2;
  Cinema_emxEnsureCapacity_real_T(obj->JointInternal.PositionLimitsInternal,
    b_kstr);
  for (b_kstr = 0; b_kstr < 2; b_kstr++) {
    obj->JointInternal.PositionLimitsInternal->data[b_kstr] = poslim_data[b_kstr];
  }

  b_kstr = obj->JointInternal.HomePositionInternal->size[0];
  obj->JointInternal.HomePositionInternal->size[0] = 1;
  Cinema_emxEnsureCapacity_real_T(obj->JointInternal.HomePositionInternal,
    b_kstr);
  for (b_kstr = 0; b_kstr < 1; b_kstr++) {
    obj->JointInternal.HomePositionInternal->data[0] = 0.0;
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.JointToParentTransform[b_kstr] = tmp_7[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.ChildToJointTransform[b_kstr] = tmp_8[b_kstr];
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  Cinema_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = tmp_9[b_kstr];
  }

  obj->JointInternal.InTree = true;
  b_kstr = obj->JointInternal.PositionLimitsInternal->size[0] *
    obj->JointInternal.PositionLimitsInternal->size[1];
  obj->JointInternal.PositionLimitsInternal->size[0] = 1;
  obj->JointInternal.PositionLimitsInternal->size[1] = 2;
  Cinema_emxEnsureCapacity_real_T(obj->JointInternal.PositionLimitsInternal,
    b_kstr);
  obj->JointInternal.PositionLimitsInternal->data[0] = 0.52359877559829882;
  obj->JointInternal.PositionLimitsInternal->data
    [obj->JointInternal.PositionLimitsInternal->size[0]] = 2.6179938779914944;
  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 1.0;
  Cinemati_emxInit_unnamed_struct(&f, 2);
  b_kstr = obj->JointInternal.HomePositionInternal->size[0];
  obj->JointInternal.HomePositionInternal->size[0] = 1;
  Cinema_emxEnsureCapacity_real_T(obj->JointInternal.HomePositionInternal,
    b_kstr);
  obj->JointInternal.HomePositionInternal->data[0] = 1.5428818882787143;
  obj_0 = &obj->CollisionsInternal;
  obj->CollisionsInternal.Size = 0.0;
  obj->CollisionsInternal.MaxElements = 0.0;
  b_kstr = f->size[0] * f->size[1];
  f->size[0] = 1;
  f->size[1] = static_cast<int32_T>(obj->CollisionsInternal.MaxElements);
  emxEnsureCapacity_unnamed_struc(f, b_kstr);
  b_kstr = obj->CollisionsInternal.CollisionGeometries->size[0] *
    obj->CollisionsInternal.CollisionGeometries->size[1];
  obj->CollisionsInternal.CollisionGeometries->size[0] = f->size[0];
  obj->CollisionsInternal.CollisionGeometries->size[1] = f->size[1];
  emxEnsureCapacity_unnamed_struc(obj->CollisionsInternal.CollisionGeometries,
    b_kstr);
  collisioncodegen_makeBox(0.0, 0.0, 0.0);
  c = obj->CollisionsInternal.MaxElements;
  loop_ub = static_cast<int32_T>(c) - 1;
  Cinemati_emxFree_unnamed_struct(&f);
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    obj_0->CollisionGeometries->data[b_kstr] = &obj->CollisionsInternal._pobj0;
  }

  return b_obj;
}

static w_robotics_manip_internal_Rig_T *Cinemat_RigidBody_RigidBody_k1e
  (w_robotics_manip_internal_Rig_T *obj)
{
  static const real_T tmp_2[36] = { 0.00012201830736582321,
    2.0700149769162018E-9, -4.4778421705104406E-7, 0.0, 0.0017479981049891046,
    1.4004577021897498E-7, 2.0700149769162018E-9, 0.00012202449746612746,
    -3.3295643968022648E-8, -0.0017479981049891046, 0.0, 8.5072875958285759E-6,
    -4.4778421705104406E-7, -3.3295643968022648E-8, 3.1221148522495581E-7,
    -1.4004577021897498E-7, -8.5072875958285759E-6, 0.0, 0.0,
    -0.0017479981049891046, -1.4004577021897498E-7, 0.03490034695, 0.0, 0.0,
    0.0017479981049891046, 0.0, -8.5072875958285759E-6, 0.0, 0.03490034695, 0.0,
    1.4004577021897498E-7, 8.5072875958285759E-6, 0.0, 0.0, 0.0, 0.03490034695 };

  static const real_T tmp_8[16] = { 6.123233995736766E-17, 0.0, 1.0, 0.0, 1.0,
    6.123233995736766E-17, -6.123233995736766E-17, 0.0, -6.123233995736766E-17,
    1.0, 3.749399456654644E-33, 0.0, 2.7105054312137611E-20,
    -0.049999999999999989, -0.043189205999999994, 1.0 };

  static const real_T tmp_9[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  static const real_T tmp_1[9] = { 0.00012201830736582321, 2.0700149769162018E-9,
    -4.4778421705104406E-7, 2.0700149769162018E-9, 0.00012202449746612746,
    -3.3295643968022648E-8, -4.4778421705104406E-7, -3.3295643968022648E-8,
    3.1221148522495581E-7 };

  static const char_T tmp_7[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const char_T tmp_6[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_4[6] = { 'J', 'o', 'i', 'n', 't', '5' };

  static const char_T tmp_0[5] = { 'B', 'o', 'd', 'y', '5' };

  static const char_T tmp_5[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const int8_T tmp_3[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    1 };

  emxArray_char_T_Cinematique_R_T *switch_expression;
  emxArray_unnamed_struct_Cinem_T *f;
  m_robotics_manip_internal_Col_T *obj_0;
  w_robotics_manip_internal_Rig_T *b_obj;
  real_T poslim_data[12];
  real_T c;
  int32_T b_kstr;
  int32_T exitg1;
  int32_T loop_ub;
  char_T b_0[9];
  char_T b[8];
  int8_T msubspace_data[36];
  int8_T tmp[6];
  boolean_T b_bool;
  b_obj = obj;
  b_kstr = obj->NameInternal->size[0] * obj->NameInternal->size[1];
  obj->NameInternal->size[0] = 1;
  obj->NameInternal->size[1] = 5;
  Cinema_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp_0[b_kstr];
  }

  obj->ParentIndex = 4.0;
  obj->MassInternal = 0.03490034695;
  obj->CenterOfMassInternal[0] = -0.00024375939895430112;
  obj->CenterOfMassInternal[1] = 4.012732894019983E-6;
  obj->CenterOfMassInternal[2] = -0.050085407674983144;
  for (b_kstr = 0; b_kstr < 9; b_kstr++) {
    obj->InertiaInternal[b_kstr] = tmp_1[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 36; b_kstr++) {
    obj->SpatialInertia[b_kstr] = tmp_2[b_kstr];
  }

  obj->JointInternal.InTree = false;
  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.JointToParentTransform[b_kstr] = tmp_3[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.ChildToJointTransform[b_kstr] = tmp_3[b_kstr];
  }

  b_kstr = obj->JointInternal.NameInternal->size[0] *
    obj->JointInternal.NameInternal->size[1];
  obj->JointInternal.NameInternal->size[0] = 1;
  obj->JointInternal.NameInternal->size[1] = 6;
  Cinema_emxEnsureCapacity_char_T(obj->JointInternal.NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.NameInternal->data[b_kstr] = tmp_4[b_kstr];
  }

  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 5;
  Cinema_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_5[b_kstr];
  }

  Cinematique_ROS_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  Cinema_emxEnsureCapacity_char_T(switch_expression, b_kstr);
  loop_ub = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1]
    - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = obj->JointInternal.Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp_6[b_kstr];
  }

  b_bool = false;
  if (switch_expression->size[1] == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        if (switch_expression->data[b_kstr - 1] != b[b_kstr - 1]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (b_bool) {
    b_kstr = 0;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      b_0[b_kstr] = tmp_7[b_kstr];
    }

    b_bool = false;
    if (switch_expression->size[1] == 9) {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          if (switch_expression->data[b_kstr - 1] != b_0[b_kstr - 1]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (b_bool) {
      b_kstr = 1;
    } else {
      b_kstr = -1;
    }
  }

  Cinematique_ROS_emxFree_char_T(&switch_expression);
  switch (b_kstr) {
   case 0:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 1;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 0;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    poslim_data[0] = -3.1415926535897931;
    poslim_data[1] = 3.1415926535897931;
    obj->JointInternal.VelocityNumber = 1.0;
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 0;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 1;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    poslim_data[0] = -0.5;
    poslim_data[1] = 0.5;
    obj->JointInternal.VelocityNumber = 1.0;
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = 0;
    }

    poslim_data[0] = 0.0;
    poslim_data[1] = 0.0;
    obj->JointInternal.VelocityNumber = 0.0;
    obj->JointInternal.PositionNumber = 0.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  Cinema_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  b_kstr = obj->JointInternal.PositionLimitsInternal->size[0] *
    obj->JointInternal.PositionLimitsInternal->size[1];
  obj->JointInternal.PositionLimitsInternal->size[0] = 1;
  obj->JointInternal.PositionLimitsInternal->size[1] = 2;
  Cinema_emxEnsureCapacity_real_T(obj->JointInternal.PositionLimitsInternal,
    b_kstr);
  for (b_kstr = 0; b_kstr < 2; b_kstr++) {
    obj->JointInternal.PositionLimitsInternal->data[b_kstr] = poslim_data[b_kstr];
  }

  b_kstr = obj->JointInternal.HomePositionInternal->size[0];
  obj->JointInternal.HomePositionInternal->size[0] = 1;
  Cinema_emxEnsureCapacity_real_T(obj->JointInternal.HomePositionInternal,
    b_kstr);
  for (b_kstr = 0; b_kstr < 1; b_kstr++) {
    obj->JointInternal.HomePositionInternal->data[0] = 0.0;
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.JointToParentTransform[b_kstr] = tmp_8[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.ChildToJointTransform[b_kstr] = tmp_9[b_kstr];
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  Cinema_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = 0.0;
  }

  obj->JointInternal.InTree = true;
  b_kstr = obj->JointInternal.PositionLimitsInternal->size[0] *
    obj->JointInternal.PositionLimitsInternal->size[1];
  obj->JointInternal.PositionLimitsInternal->size[0] = 1;
  obj->JointInternal.PositionLimitsInternal->size[1] = 2;
  Cinema_emxEnsureCapacity_real_T(obj->JointInternal.PositionLimitsInternal,
    b_kstr);
  obj->JointInternal.PositionLimitsInternal->data[0] = 0.0;
  obj->JointInternal.PositionLimitsInternal->data
    [obj->JointInternal.PositionLimitsInternal->size[0]] = 0.0;
  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 0.0;
  Cinemati_emxInit_unnamed_struct(&f, 2);
  b_kstr = obj->JointInternal.HomePositionInternal->size[0];
  obj->JointInternal.HomePositionInternal->size[0] = 1;
  Cinema_emxEnsureCapacity_real_T(obj->JointInternal.HomePositionInternal,
    b_kstr);
  obj->JointInternal.HomePositionInternal->data[0] = 0.0;
  obj_0 = &obj->CollisionsInternal;
  obj->CollisionsInternal.Size = 0.0;
  obj->CollisionsInternal.MaxElements = 0.0;
  b_kstr = f->size[0] * f->size[1];
  f->size[0] = 1;
  f->size[1] = static_cast<int32_T>(obj->CollisionsInternal.MaxElements);
  emxEnsureCapacity_unnamed_struc(f, b_kstr);
  b_kstr = obj->CollisionsInternal.CollisionGeometries->size[0] *
    obj->CollisionsInternal.CollisionGeometries->size[1];
  obj->CollisionsInternal.CollisionGeometries->size[0] = f->size[0];
  obj->CollisionsInternal.CollisionGeometries->size[1] = f->size[1];
  emxEnsureCapacity_unnamed_struc(obj->CollisionsInternal.CollisionGeometries,
    b_kstr);
  collisioncodegen_makeBox(0.0, 0.0, 0.0);
  c = obj->CollisionsInternal.MaxElements;
  loop_ub = static_cast<int32_T>(c) - 1;
  Cinemati_emxFree_unnamed_struct(&f);
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    obj_0->CollisionGeometries->data[b_kstr] = &obj->CollisionsInternal._pobj0;
  }

  return b_obj;
}

static x_robotics_manip_internal_Rig_T *Cin_RigidBodyTree_RigidBodyTree
  (x_robotics_manip_internal_Rig_T *obj)
{
  static const real_T tmp_2[36] = { 0.015069030676500643, -8.416429473559707E-20,
    2.1659198834136543E-20, 0.0, 0.10290947768970442, 0.04173545540596179,
    -8.416429473559707E-20, 0.012528566692678728, 0.0047464265993520747,
    -0.10290947768970442, 0.0, 0.0, 2.1659198834136543E-20,
    0.0047464265993520747, 0.0036477659838219162, -0.04173545540596179, -0.0,
    0.0, 0.0, -0.10290947768970442, -0.04173545540596179, 0.93990853550000009,
    0.0, 0.0, 0.10290947768970442, 0.0, -0.0, 0.0, 0.93990853550000009, 0.0,
    0.04173545540596179, 0.0, 0.0, 0.0, 0.0, 0.93990853550000009 };

  static const real_T tmp_9[36] = { 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  static const real_T tmp_7[16] = { 1.0, 0.0, -0.0, 0.0, 0.0, -1.0,
    1.2246467991473532E-16, 0.0, 0.0, -1.2246467991473532E-16, -1.0, 0.0,
    -0.017867237, -0.023828417, 0.120080891, 1.0 };

  static const real_T tmp_8[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  static const real_T tmp_1[9] = { 0.015069030676500643, -8.416429473559707E-20,
    2.1659198834136543E-20, -8.416429473559707E-20, 0.012528566692678728,
    0.0047464265993520747, 2.1659198834136543E-20, 0.0047464265993520747,
    0.0036477659838219162 };

  static const char_T tmp_6[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const char_T tmp_5[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_a[8] = { 'B', 'a', 's', 'e', '_', 'j', 'n', 't' };

  static const char_T tmp_4[6] = { 'J', 'o', 'i', 'n', 't', '1' };

  static const char_T tmp_0[5] = { 'B', 'o', 'd', 'y', '1' };

  static const char_T tmp_b[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const int8_T tmp_3[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    1 };

  emxArray_char_T_Cinematique_R_T *switch_expression;
  emxArray_unnamed_struct_Cinem_T *f;
  emxArray_unnamed_struct_Cinem_T *f_0;
  w_robotics_manip_internal_Rig_T *obj_0;
  x_robotics_manip_internal_Rig_T *b_obj;
  real_T poslim_data[12];
  real_T c;
  int32_T b_kstr;
  int32_T exitg1;
  int32_T loop_ub;
  char_T b_0[9];
  char_T b[8];
  int8_T msubspace_data[36];
  int8_T b_I[9];
  int8_T tmp[6];
  boolean_T b_bool;
  b_obj = obj;
  obj_0 = &obj->_pobj0[0];
  b_kstr = obj->_pobj0[0].NameInternal->size[0] * obj->_pobj0[0]
    .NameInternal->size[1];
  obj->_pobj0[0].NameInternal->size[0] = 1;
  obj->_pobj0[0].NameInternal->size[1] = 5;
  Cinema_emxEnsureCapacity_char_T(obj->_pobj0[0].NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    obj->_pobj0[0].NameInternal->data[b_kstr] = tmp_0[b_kstr];
  }

  obj->_pobj0[0].ParentIndex = 0.0;
  obj->_pobj0[0].MassInternal = 0.93990853550000009;
  obj->_pobj0[0].CenterOfMassInternal[0] = -0.0;
  obj->_pobj0[0].CenterOfMassInternal[1] = 0.044403741246758562;
  obj->_pobj0[0].CenterOfMassInternal[2] = -0.10948882130851169;
  for (b_kstr = 0; b_kstr < 9; b_kstr++) {
    obj->_pobj0[0].InertiaInternal[b_kstr] = tmp_1[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 36; b_kstr++) {
    obj->_pobj0[0].SpatialInertia[b_kstr] = tmp_2[b_kstr];
  }

  obj_0->JointInternal.InTree = false;
  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj_0->JointInternal.JointToParentTransform[b_kstr] = tmp_3[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj_0->JointInternal.ChildToJointTransform[b_kstr] = tmp_3[b_kstr];
  }

  b_kstr = obj_0->JointInternal.NameInternal->size[0] *
    obj_0->JointInternal.NameInternal->size[1];
  obj_0->JointInternal.NameInternal->size[0] = 1;
  obj_0->JointInternal.NameInternal->size[1] = 6;
  Cinema_emxEnsureCapacity_char_T(obj_0->JointInternal.NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj_0->JointInternal.NameInternal->data[b_kstr] = tmp_4[b_kstr];
  }

  b_kstr = obj_0->JointInternal.Type->size[0] * obj_0->JointInternal.Type->size
    [1];
  obj_0->JointInternal.Type->size[0] = 1;
  obj_0->JointInternal.Type->size[1] = 8;
  Cinema_emxEnsureCapacity_char_T(obj_0->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj_0->JointInternal.Type->data[b_kstr] = tmp_5[b_kstr];
  }

  Cinematique_ROS_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj_0->JointInternal.Type->size[1];
  Cinema_emxEnsureCapacity_char_T(switch_expression, b_kstr);
  loop_ub = obj_0->JointInternal.Type->size[0] * obj_0->JointInternal.Type->
    size[1] - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = obj_0->JointInternal.Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp_5[b_kstr];
  }

  b_bool = false;
  if (switch_expression->size[1] == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        if (switch_expression->data[b_kstr - 1] != b[b_kstr - 1]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (b_bool) {
    b_kstr = 0;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      b_0[b_kstr] = tmp_6[b_kstr];
    }

    b_bool = false;
    if (switch_expression->size[1] == 9) {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          if (switch_expression->data[b_kstr - 1] != b_0[b_kstr - 1]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (b_bool) {
      b_kstr = 1;
    } else {
      b_kstr = -1;
    }
  }

  switch (b_kstr) {
   case 0:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 1;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 0;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    poslim_data[0] = -3.1415926535897931;
    poslim_data[1] = 3.1415926535897931;
    obj_0->JointInternal.VelocityNumber = 1.0;
    obj_0->JointInternal.PositionNumber = 1.0;
    obj_0->JointInternal.JointAxisInternal[0] = 0.0;
    obj_0->JointInternal.JointAxisInternal[1] = 0.0;
    obj_0->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 0;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 1;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    poslim_data[0] = -0.5;
    poslim_data[1] = 0.5;
    obj_0->JointInternal.VelocityNumber = 1.0;
    obj_0->JointInternal.PositionNumber = 1.0;
    obj_0->JointInternal.JointAxisInternal[0] = 0.0;
    obj_0->JointInternal.JointAxisInternal[1] = 0.0;
    obj_0->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = 0;
    }

    poslim_data[0] = 0.0;
    poslim_data[1] = 0.0;
    obj_0->JointInternal.VelocityNumber = 0.0;
    obj_0->JointInternal.PositionNumber = 0.0;
    obj_0->JointInternal.JointAxisInternal[0] = 0.0;
    obj_0->JointInternal.JointAxisInternal[1] = 0.0;
    obj_0->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  b_kstr = obj_0->JointInternal.MotionSubspace->size[0] *
    obj_0->JointInternal.MotionSubspace->size[1];
  obj_0->JointInternal.MotionSubspace->size[0] = 6;
  obj_0->JointInternal.MotionSubspace->size[1] = 1;
  Cinema_emxEnsureCapacity_real_T(obj_0->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj_0->JointInternal.MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  b_kstr = obj_0->JointInternal.PositionLimitsInternal->size[0] *
    obj_0->JointInternal.PositionLimitsInternal->size[1];
  obj_0->JointInternal.PositionLimitsInternal->size[0] = 1;
  obj_0->JointInternal.PositionLimitsInternal->size[1] = 2;
  Cinema_emxEnsureCapacity_real_T(obj_0->JointInternal.PositionLimitsInternal,
    b_kstr);
  for (b_kstr = 0; b_kstr < 2; b_kstr++) {
    obj_0->JointInternal.PositionLimitsInternal->data[b_kstr] =
      poslim_data[b_kstr];
  }

  b_kstr = obj_0->JointInternal.HomePositionInternal->size[0];
  obj_0->JointInternal.HomePositionInternal->size[0] = 1;
  Cinema_emxEnsureCapacity_real_T(obj_0->JointInternal.HomePositionInternal,
    b_kstr);
  for (b_kstr = 0; b_kstr < 1; b_kstr++) {
    obj_0->JointInternal.HomePositionInternal->data[0] = 0.0;
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->_pobj0[0].JointInternal.JointToParentTransform[b_kstr] = tmp_7[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->_pobj0[0].JointInternal.ChildToJointTransform[b_kstr] = tmp_8[b_kstr];
  }

  b_kstr = obj->_pobj0[0].JointInternal.MotionSubspace->size[0] * obj->_pobj0[0]
    .JointInternal.MotionSubspace->size[1];
  obj->_pobj0[0].JointInternal.MotionSubspace->size[0] = 6;
  obj->_pobj0[0].JointInternal.MotionSubspace->size[1] = 1;
  Cinema_emxEnsureCapacity_real_T(obj->_pobj0[0].JointInternal.MotionSubspace,
    b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->_pobj0[0].JointInternal.MotionSubspace->data[b_kstr] = tmp_9[b_kstr];
  }

  obj->_pobj0[0].JointInternal.InTree = true;
  b_kstr = obj->_pobj0[0].JointInternal.PositionLimitsInternal->size[0] *
    obj->_pobj0[0].JointInternal.PositionLimitsInternal->size[1];
  obj->_pobj0[0].JointInternal.PositionLimitsInternal->size[0] = 1;
  obj->_pobj0[0].JointInternal.PositionLimitsInternal->size[1] = 2;
  Cinema_emxEnsureCapacity_real_T(obj->_pobj0[0].
    JointInternal.PositionLimitsInternal, b_kstr);
  obj->_pobj0[0].JointInternal.PositionLimitsInternal->data[0] =
    -1.9198621771937625;
  obj->_pobj0[0].JointInternal.PositionLimitsInternal->data[obj->_pobj0[0].
    JointInternal.PositionLimitsInternal->size[0]] = 1.9198621771937625;
  obj->_pobj0[0].JointInternal.JointAxisInternal[0] = 0.0;
  obj->_pobj0[0].JointInternal.JointAxisInternal[1] = 0.0;
  obj->_pobj0[0].JointInternal.JointAxisInternal[2] = 1.0;
  Cinemati_emxInit_unnamed_struct(&f, 2);
  b_kstr = obj->_pobj0[0].JointInternal.HomePositionInternal->size[0];
  obj->_pobj0[0].JointInternal.HomePositionInternal->size[0] = 1;
  Cinema_emxEnsureCapacity_real_T(obj->_pobj0[0].
    JointInternal.HomePositionInternal, b_kstr);
  obj->_pobj0[0].JointInternal.HomePositionInternal->data[0] = 0.0;
  obj_0->CollisionsInternal.Size = 0.0;
  obj_0->CollisionsInternal.MaxElements = 0.0;
  b_kstr = f->size[0] * f->size[1];
  f->size[0] = 1;
  f->size[1] = static_cast<int32_T>(obj_0->CollisionsInternal.MaxElements);
  emxEnsureCapacity_unnamed_struc(f, b_kstr);
  b_kstr = obj_0->CollisionsInternal.CollisionGeometries->size[0] *
    obj_0->CollisionsInternal.CollisionGeometries->size[1];
  obj_0->CollisionsInternal.CollisionGeometries->size[0] = f->size[0];
  obj_0->CollisionsInternal.CollisionGeometries->size[1] = f->size[1];
  emxEnsureCapacity_unnamed_struc(obj_0->CollisionsInternal.CollisionGeometries,
    b_kstr);
  collisioncodegen_makeBox(0.0, 0.0, 0.0);
  c = obj_0->CollisionsInternal.MaxElements;
  loop_ub = static_cast<int32_T>(c) - 1;
  Cinemati_emxFree_unnamed_struct(&f);
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    obj->_pobj0[0].CollisionsInternal.CollisionGeometries->data[b_kstr] =
      &obj_0->CollisionsInternal._pobj0;
  }

  obj->Bodies[0] = &obj->_pobj0[0];
  obj->Bodies[0]->Index = 1.0;
  obj->Bodies[1] = Cinematique_RigidBody_RigidBody(&obj->_pobj0[1]);
  obj->Bodies[1]->Index = 2.0;
  obj->Bodies[2] = Cinematiq_RigidBody_RigidBody_k(&obj->_pobj0[2]);
  obj->Bodies[2]->Index = 3.0;
  obj->Bodies[3] = Cinemati_RigidBody_RigidBody_k1(&obj->_pobj0[3]);
  obj->Bodies[3]->Index = 4.0;
  obj->Bodies[4] = Cinemat_RigidBody_RigidBody_k1e(&obj->_pobj0[4]);
  obj->Bodies[4]->Index = 5.0;
  obj->NumBodies = 5.0;
  obj->Gravity[0] = 0.0;
  obj->Gravity[1] = 0.0;
  obj->Gravity[2] = -9.80665;
  obj_0 = &obj->Base;
  b_kstr = obj->Base.NameInternal->size[0] * obj->Base.NameInternal->size[1];
  obj->Base.NameInternal->size[0] = 1;
  obj->Base.NameInternal->size[1] = 4;
  Cinema_emxEnsureCapacity_char_T(obj->Base.NameInternal, b_kstr);
  obj->Base.NameInternal->data[0] = 'B';
  obj->Base.NameInternal->data[1] = 'a';
  obj->Base.NameInternal->data[2] = 's';
  obj->Base.NameInternal->data[3] = 'e';
  obj->Base.JointInternal.InTree = false;
  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj_0->JointInternal.JointToParentTransform[b_kstr] = tmp_3[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj_0->JointInternal.ChildToJointTransform[b_kstr] = tmp_3[b_kstr];
  }

  b_kstr = obj->Base.JointInternal.NameInternal->size[0] *
    obj->Base.JointInternal.NameInternal->size[1];
  obj->Base.JointInternal.NameInternal->size[0] = 1;
  obj->Base.JointInternal.NameInternal->size[1] = 8;
  Cinema_emxEnsureCapacity_char_T(obj->Base.JointInternal.NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj_0->JointInternal.NameInternal->data[b_kstr] = tmp_a[b_kstr];
  }

  b_kstr = obj->Base.JointInternal.Type->size[0] * obj->
    Base.JointInternal.Type->size[1];
  obj->Base.JointInternal.Type->size[0] = 1;
  obj->Base.JointInternal.Type->size[1] = 5;
  Cinema_emxEnsureCapacity_char_T(obj->Base.JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    obj_0->JointInternal.Type->data[b_kstr] = tmp_b[b_kstr];
  }

  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->Base.JointInternal.Type->size[1];
  Cinema_emxEnsureCapacity_char_T(switch_expression, b_kstr);
  loop_ub = obj->Base.JointInternal.Type->size[0] * obj->
    Base.JointInternal.Type->size[1] - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = obj_0->JointInternal.Type->data[b_kstr];
  }

  b_bool = false;
  if (switch_expression->size[1] == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        if (switch_expression->data[b_kstr - 1] != b[b_kstr - 1]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (b_bool) {
    b_kstr = 0;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      b_0[b_kstr] = tmp_6[b_kstr];
    }

    b_bool = false;
    if (switch_expression->size[1] == 9) {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          if (switch_expression->data[b_kstr - 1] != b_0[b_kstr - 1]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (b_bool) {
      b_kstr = 1;
    } else {
      b_kstr = -1;
    }
  }

  Cinematique_ROS_emxFree_char_T(&switch_expression);
  switch (b_kstr) {
   case 0:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 1;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 0;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    poslim_data[0] = -3.1415926535897931;
    poslim_data[1] = 3.1415926535897931;
    obj->Base.JointInternal.VelocityNumber = 1.0;
    obj->Base.JointInternal.PositionNumber = 1.0;
    obj->Base.JointInternal.JointAxisInternal[0] = 0.0;
    obj->Base.JointInternal.JointAxisInternal[1] = 0.0;
    obj->Base.JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 0;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 1;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    poslim_data[0] = -0.5;
    poslim_data[1] = 0.5;
    obj->Base.JointInternal.VelocityNumber = 1.0;
    obj->Base.JointInternal.PositionNumber = 1.0;
    obj->Base.JointInternal.JointAxisInternal[0] = 0.0;
    obj->Base.JointInternal.JointAxisInternal[1] = 0.0;
    obj->Base.JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = 0;
    }

    poslim_data[0] = 0.0;
    poslim_data[1] = 0.0;
    obj->Base.JointInternal.VelocityNumber = 0.0;
    obj->Base.JointInternal.PositionNumber = 0.0;
    obj->Base.JointInternal.JointAxisInternal[0] = 0.0;
    obj->Base.JointInternal.JointAxisInternal[1] = 0.0;
    obj->Base.JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  b_kstr = obj->Base.JointInternal.MotionSubspace->size[0] *
    obj->Base.JointInternal.MotionSubspace->size[1];
  obj->Base.JointInternal.MotionSubspace->size[0] = 6;
  obj->Base.JointInternal.MotionSubspace->size[1] = 1;
  Cinema_emxEnsureCapacity_real_T(obj->Base.JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj_0->JointInternal.MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  b_kstr = obj->Base.JointInternal.PositionLimitsInternal->size[0] *
    obj->Base.JointInternal.PositionLimitsInternal->size[1];
  obj->Base.JointInternal.PositionLimitsInternal->size[0] = 1;
  obj->Base.JointInternal.PositionLimitsInternal->size[1] = 2;
  Cinema_emxEnsureCapacity_real_T(obj->Base.JointInternal.PositionLimitsInternal,
    b_kstr);
  for (b_kstr = 0; b_kstr < 2; b_kstr++) {
    obj_0->JointInternal.PositionLimitsInternal->data[b_kstr] =
      poslim_data[b_kstr];
  }

  b_kstr = obj->Base.JointInternal.HomePositionInternal->size[0];
  obj->Base.JointInternal.HomePositionInternal->size[0] = 1;
  Cinema_emxEnsureCapacity_real_T(obj->Base.JointInternal.HomePositionInternal,
    b_kstr);
  for (b_kstr = 0; b_kstr < 1; b_kstr++) {
    obj_0->JointInternal.HomePositionInternal->data[0] = 0.0;
  }

  obj->Base.Index = -1.0;
  obj->Base.ParentIndex = -1.0;
  obj->Base.MassInternal = 1.0;
  obj->Base.CenterOfMassInternal[0] = 0.0;
  obj->Base.CenterOfMassInternal[1] = 0.0;
  obj->Base.CenterOfMassInternal[2] = 0.0;
  for (b_kstr = 0; b_kstr < 9; b_kstr++) {
    b_I[b_kstr] = 0;
  }

  b_I[0] = 1;
  b_I[4] = 1;
  b_I[8] = 1;
  for (b_kstr = 0; b_kstr < 9; b_kstr++) {
    obj->Base.InertiaInternal[b_kstr] = b_I[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 36; b_kstr++) {
    msubspace_data[b_kstr] = 0;
  }

  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    msubspace_data[b_kstr + 6 * b_kstr] = 1;
  }

  for (b_kstr = 0; b_kstr < 36; b_kstr++) {
    obj->Base.SpatialInertia[b_kstr] = msubspace_data[b_kstr];
  }

  Cinemati_emxInit_unnamed_struct(&f_0, 2);
  obj_0->CollisionsInternal.Size = 0.0;
  obj_0->CollisionsInternal.MaxElements = 0.0;
  b_kstr = f_0->size[0] * f_0->size[1];
  f_0->size[0] = 1;
  f_0->size[1] = static_cast<int32_T>(obj_0->CollisionsInternal.MaxElements);
  emxEnsureCapacity_unnamed_struc(f_0, b_kstr);
  b_kstr = obj_0->CollisionsInternal.CollisionGeometries->size[0] *
    obj_0->CollisionsInternal.CollisionGeometries->size[1];
  obj_0->CollisionsInternal.CollisionGeometries->size[0] = f_0->size[0];
  obj_0->CollisionsInternal.CollisionGeometries->size[1] = f_0->size[1];
  emxEnsureCapacity_unnamed_struc(obj_0->CollisionsInternal.CollisionGeometries,
    b_kstr);
  collisioncodegen_makeBox(0.0, 0.0, 0.0);
  c = obj_0->CollisionsInternal.MaxElements;
  loop_ub = static_cast<int32_T>(c) - 1;
  Cinemati_emxFree_unnamed_struct(&f_0);
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    obj->Base.CollisionsInternal.CollisionGeometries->data[b_kstr] =
      &obj_0->CollisionsInternal._pobj0;
  }

  return b_obj;
}

static void Cinemat_genrand_uint32_vector_k(uint32_T mt[625], uint32_T u[2])
{
  for (Cinematique_ROS_B.b_j_a = 0; Cinematique_ROS_B.b_j_a < 2;
       Cinematique_ROS_B.b_j_a++) {
    Cinematique_ROS_B.mti = mt[624] + 1U;
    if (mt[624] + 1U >= 625U) {
      for (Cinematique_ROS_B.b_kk = 0; Cinematique_ROS_B.b_kk < 227;
           Cinematique_ROS_B.b_kk++) {
        Cinematique_ROS_B.y_m = (mt[Cinematique_ROS_B.b_kk + 1] & 2147483647U) |
          (mt[Cinematique_ROS_B.b_kk] & 2147483648U);
        if ((Cinematique_ROS_B.y_m & 1U) == 0U) {
          Cinematique_ROS_B.y_m >>= 1U;
        } else {
          Cinematique_ROS_B.y_m = Cinematique_ROS_B.y_m >> 1U ^ 2567483615U;
        }

        mt[Cinematique_ROS_B.b_kk] = mt[Cinematique_ROS_B.b_kk + 397] ^
          Cinematique_ROS_B.y_m;
      }

      for (Cinematique_ROS_B.b_kk = 0; Cinematique_ROS_B.b_kk < 396;
           Cinematique_ROS_B.b_kk++) {
        Cinematique_ROS_B.y_m = (mt[Cinematique_ROS_B.b_kk + 227] & 2147483648U)
          | (mt[Cinematique_ROS_B.b_kk + 228] & 2147483647U);
        if ((Cinematique_ROS_B.y_m & 1U) == 0U) {
          Cinematique_ROS_B.y_m >>= 1U;
        } else {
          Cinematique_ROS_B.y_m = Cinematique_ROS_B.y_m >> 1U ^ 2567483615U;
        }

        mt[Cinematique_ROS_B.b_kk + 227] = mt[Cinematique_ROS_B.b_kk] ^
          Cinematique_ROS_B.y_m;
      }

      Cinematique_ROS_B.y_m = (mt[623] & 2147483648U) | (mt[0] & 2147483647U);
      if ((Cinematique_ROS_B.y_m & 1U) == 0U) {
        Cinematique_ROS_B.y_m >>= 1U;
      } else {
        Cinematique_ROS_B.y_m = Cinematique_ROS_B.y_m >> 1U ^ 2567483615U;
      }

      mt[623] = mt[396] ^ Cinematique_ROS_B.y_m;
      Cinematique_ROS_B.mti = 1U;
    }

    Cinematique_ROS_B.y_m = mt[static_cast<int32_T>(Cinematique_ROS_B.mti) - 1];
    mt[624] = Cinematique_ROS_B.mti;
    Cinematique_ROS_B.y_m ^= Cinematique_ROS_B.y_m >> 11U;
    Cinematique_ROS_B.y_m ^= Cinematique_ROS_B.y_m << 7U & 2636928640U;
    Cinematique_ROS_B.y_m ^= Cinematique_ROS_B.y_m << 15U & 4022730752U;
    u[Cinematique_ROS_B.b_j_a] = Cinematique_ROS_B.y_m >> 18U ^
      Cinematique_ROS_B.y_m;
  }
}

static boolean_T Cinematique_ROS_is_valid_state(const uint32_T mt[625])
{
  boolean_T exitg1;
  boolean_T isvalid;
  if ((mt[624] >= 1U) && (mt[624] < 625U)) {
    isvalid = true;
  } else {
    isvalid = false;
  }

  if (isvalid) {
    isvalid = false;
    Cinematique_ROS_B.k = 0;
    exitg1 = false;
    while ((!exitg1) && (Cinematique_ROS_B.k + 1 < 625)) {
      if (mt[Cinematique_ROS_B.k] == 0U) {
        Cinematique_ROS_B.k++;
      } else {
        isvalid = true;
        exitg1 = true;
      }
    }
  }

  return isvalid;
}

static void Cinematique__eml_rand_mt19937ar(const uint32_T state[625], uint32_T
  b_state[625], real_T *r)
{
  int32_T exitg1;
  memcpy(&b_state[0], &state[0], 625U * sizeof(uint32_T));

  /* ========================= COPYRIGHT NOTICE ============================ */
  /*  This is a uniform (0,1) pseudorandom number generator based on:        */
  /*                                                                         */
  /*  A C-program for MT19937, with initialization improved 2002/1/26.       */
  /*  Coded by Takuji Nishimura and Makoto Matsumoto.                        */
  /*                                                                         */
  /*  Copyright (C) 1997 - 2002, Makoto Matsumoto and Takuji Nishimura,      */
  /*  All rights reserved.                                                   */
  /*                                                                         */
  /*  Redistribution and use in source and binary forms, with or without     */
  /*  modification, are permitted provided that the following conditions     */
  /*  are met:                                                               */
  /*                                                                         */
  /*    1. Redistributions of source code must retain the above copyright    */
  /*       notice, this list of conditions and the following disclaimer.     */
  /*                                                                         */
  /*    2. Redistributions in binary form must reproduce the above copyright */
  /*       notice, this list of conditions and the following disclaimer      */
  /*       in the documentation and/or other materials provided with the     */
  /*       distribution.                                                     */
  /*                                                                         */
  /*    3. The names of its contributors may not be used to endorse or       */
  /*       promote products derived from this software without specific      */
  /*       prior written permission.                                         */
  /*                                                                         */
  /*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS    */
  /*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT      */
  /*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR  */
  /*  A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT  */
  /*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,  */
  /*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT       */
  /*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,  */
  /*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY  */
  /*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT    */
  /*  (INCLUDING  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE */
  /*  OF THIS  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.  */
  /*                                                                         */
  /* =============================   END   ================================= */
  do {
    exitg1 = 0;
    Cinemat_genrand_uint32_vector_k(b_state, Cinematique_ROS_B.b_u);
    *r = (static_cast<real_T>(Cinematique_ROS_B.b_u[0] >> 5U) * 6.7108864E+7 +
          static_cast<real_T>(Cinematique_ROS_B.b_u[1] >> 6U)) *
      1.1102230246251565E-16;
    if (*r == 0.0) {
      if (!Cinematique_ROS_is_valid_state(b_state)) {
        Cinematique_ROS_B.r_k = 5489U;
        b_state[0] = 5489U;
        for (Cinematique_ROS_B.b_mti_c = 0; Cinematique_ROS_B.b_mti_c < 623;
             Cinematique_ROS_B.b_mti_c++) {
          Cinematique_ROS_B.r_k = ((Cinematique_ROS_B.r_k >> 30U ^
            Cinematique_ROS_B.r_k) * 1812433253U + Cinematique_ROS_B.b_mti_c) +
            1U;
          b_state[Cinematique_ROS_B.b_mti_c + 1] = Cinematique_ROS_B.r_k;
        }

        b_state[624] = 624U;
      }
    } else {
      exitg1 = 1;
    }
  } while (exitg1 == 0);
}

static void Cinematique_ROS_rand(real_T r[5])
{
  for (Cinematique_ROS_B.b_k_e = 0; Cinematique_ROS_B.b_k_e < 5;
       Cinematique_ROS_B.b_k_e++) {
    memcpy(&Cinematique_ROS_B.uv[0], &Cinematique_ROS_DW.state_l[0], 625U *
           sizeof(uint32_T));
    Cinematique__eml_rand_mt19937ar(Cinematique_ROS_B.uv,
      Cinematique_ROS_DW.state_l, &r[Cinematique_ROS_B.b_k_e]);
  }
}

static y_robotics_manip_internal_Rig_T *Cinema_RigidBody_RigidBody_k1e1
  (y_robotics_manip_internal_Rig_T *obj)
{
  static const char_T tmp_2[14] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '\x03', '_', 'j', 'n', 't' };

  static const char_T tmp_0[10] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '\x03' };

  static const char_T tmp_5[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const char_T tmp_4[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_3[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const int8_T tmp_1[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    1 };

  emxArray_char_T_Cinematique_R_T *switch_expression;
  emxArray_unnamed_struct_Cinem_T *f;
  m_robotics_manip_internal_Col_T *obj_0;
  y_robotics_manip_internal_Rig_T *b_obj;
  real_T poslim_data[12];
  real_T c;
  int32_T b_kstr;
  int32_T exitg1;
  int32_T loop_ub;
  char_T b_0[9];
  char_T b[8];
  int8_T msubspace_data[36];
  int8_T tmp[6];
  boolean_T b_bool;
  b_obj = obj;
  b_kstr = obj->NameInternal->size[0] * obj->NameInternal->size[1];
  obj->NameInternal->size[0] = 1;
  obj->NameInternal->size[1] = 10;
  Cinema_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 10; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp_0[b_kstr];
  }

  obj->_pobj1.InTree = false;
  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->_pobj1.JointToParentTransform[b_kstr] = tmp_1[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->_pobj1.ChildToJointTransform[b_kstr] = tmp_1[b_kstr];
  }

  b_kstr = obj->_pobj1.NameInternal->size[0] * obj->_pobj1.NameInternal->size[1];
  obj->_pobj1.NameInternal->size[0] = 1;
  obj->_pobj1.NameInternal->size[1] = 14;
  Cinema_emxEnsureCapacity_char_T(obj->_pobj1.NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 14; b_kstr++) {
    obj->_pobj1.NameInternal->data[b_kstr] = tmp_2[b_kstr];
  }

  b_kstr = obj->_pobj1.Type->size[0] * obj->_pobj1.Type->size[1];
  obj->_pobj1.Type->size[0] = 1;
  obj->_pobj1.Type->size[1] = 5;
  Cinema_emxEnsureCapacity_char_T(obj->_pobj1.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    obj->_pobj1.Type->data[b_kstr] = tmp_3[b_kstr];
  }

  Cinematique_ROS_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->_pobj1.Type->size[1];
  Cinema_emxEnsureCapacity_char_T(switch_expression, b_kstr);
  loop_ub = obj->_pobj1.Type->size[0] * obj->_pobj1.Type->size[1] - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = obj->_pobj1.Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp_4[b_kstr];
  }

  b_bool = false;
  if (switch_expression->size[1] == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        if (switch_expression->data[b_kstr - 1] != b[b_kstr - 1]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (b_bool) {
    b_kstr = 0;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      b_0[b_kstr] = tmp_5[b_kstr];
    }

    b_bool = false;
    if (switch_expression->size[1] == 9) {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          if (switch_expression->data[b_kstr - 1] != b_0[b_kstr - 1]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (b_bool) {
      b_kstr = 1;
    } else {
      b_kstr = -1;
    }
  }

  Cinematique_ROS_emxFree_char_T(&switch_expression);
  switch (b_kstr) {
   case 0:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 1;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 0;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    poslim_data[0] = -3.1415926535897931;
    poslim_data[1] = 3.1415926535897931;
    obj->_pobj1.VelocityNumber = 1.0;
    obj->_pobj1.PositionNumber = 1.0;
    obj->_pobj1.JointAxisInternal[0] = 0.0;
    obj->_pobj1.JointAxisInternal[1] = 0.0;
    obj->_pobj1.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 0;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 1;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    poslim_data[0] = -0.5;
    poslim_data[1] = 0.5;
    obj->_pobj1.VelocityNumber = 1.0;
    obj->_pobj1.PositionNumber = 1.0;
    obj->_pobj1.JointAxisInternal[0] = 0.0;
    obj->_pobj1.JointAxisInternal[1] = 0.0;
    obj->_pobj1.JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = 0;
    }

    poslim_data[0] = 0.0;
    poslim_data[1] = 0.0;
    obj->_pobj1.VelocityNumber = 0.0;
    obj->_pobj1.PositionNumber = 0.0;
    obj->_pobj1.JointAxisInternal[0] = 0.0;
    obj->_pobj1.JointAxisInternal[1] = 0.0;
    obj->_pobj1.JointAxisInternal[2] = 0.0;
    break;
  }

  b_kstr = obj->_pobj1.MotionSubspace->size[0] * obj->
    _pobj1.MotionSubspace->size[1];
  obj->_pobj1.MotionSubspace->size[0] = 6;
  obj->_pobj1.MotionSubspace->size[1] = 1;
  Cinema_emxEnsureCapacity_real_T(obj->_pobj1.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->_pobj1.MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  b_kstr = obj->_pobj1.PositionLimitsInternal->size[0] *
    obj->_pobj1.PositionLimitsInternal->size[1];
  obj->_pobj1.PositionLimitsInternal->size[0] = 1;
  obj->_pobj1.PositionLimitsInternal->size[1] = 2;
  Cinema_emxEnsureCapacity_real_T(obj->_pobj1.PositionLimitsInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 2; b_kstr++) {
    obj->_pobj1.PositionLimitsInternal->data[b_kstr] = poslim_data[b_kstr];
  }

  b_kstr = obj->_pobj1.HomePositionInternal->size[0];
  obj->_pobj1.HomePositionInternal->size[0] = 1;
  Cinema_emxEnsureCapacity_real_T(obj->_pobj1.HomePositionInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 1; b_kstr++) {
    obj->_pobj1.HomePositionInternal->data[0] = 0.0;
  }

  Cinemati_emxInit_unnamed_struct(&f, 2);
  obj->JointInternal = &obj->_pobj1;
  obj->Index = -1.0;
  obj->ParentIndex = -1.0;
  obj_0 = &obj->_pobj0;
  obj->_pobj0.Size = 0.0;
  obj->_pobj0.MaxElements = 0.0;
  b_kstr = f->size[0] * f->size[1];
  f->size[0] = 1;
  f->size[1] = static_cast<int32_T>(obj->_pobj0.MaxElements);
  emxEnsureCapacity_unnamed_struc(f, b_kstr);
  b_kstr = obj->_pobj0.CollisionGeometries->size[0] *
    obj->_pobj0.CollisionGeometries->size[1];
  obj->_pobj0.CollisionGeometries->size[0] = f->size[0];
  obj->_pobj0.CollisionGeometries->size[1] = f->size[1];
  emxEnsureCapacity_unnamed_struc(obj->_pobj0.CollisionGeometries, b_kstr);
  collisioncodegen_makeBox(0.0, 0.0, 0.0);
  c = obj->_pobj0.MaxElements;
  loop_ub = static_cast<int32_T>(c) - 1;
  Cinemati_emxFree_unnamed_struct(&f);
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    obj_0->CollisionGeometries->data[b_kstr] = &obj->_pobj0._pobj0;
  }

  obj->CollisionsInternal = &obj->_pobj0;
  return b_obj;
}

static y_robotics_manip_internal_Rig_T *Cinem_RigidBody_RigidBody_k1e1w
  (y_robotics_manip_internal_Rig_T *obj)
{
  static const char_T tmp_2[14] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '\x04', '_', 'j', 'n', 't' };

  static const char_T tmp_0[10] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '\x04' };

  static const char_T tmp_5[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const char_T tmp_4[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_3[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const int8_T tmp_1[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    1 };

  emxArray_char_T_Cinematique_R_T *switch_expression;
  emxArray_unnamed_struct_Cinem_T *f;
  m_robotics_manip_internal_Col_T *obj_0;
  y_robotics_manip_internal_Rig_T *b_obj;
  real_T poslim_data[12];
  real_T c;
  int32_T b_kstr;
  int32_T exitg1;
  int32_T loop_ub;
  char_T b_0[9];
  char_T b[8];
  int8_T msubspace_data[36];
  int8_T tmp[6];
  boolean_T b_bool;
  b_obj = obj;
  b_kstr = obj->NameInternal->size[0] * obj->NameInternal->size[1];
  obj->NameInternal->size[0] = 1;
  obj->NameInternal->size[1] = 10;
  Cinema_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 10; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp_0[b_kstr];
  }

  obj->_pobj1.InTree = false;
  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->_pobj1.JointToParentTransform[b_kstr] = tmp_1[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->_pobj1.ChildToJointTransform[b_kstr] = tmp_1[b_kstr];
  }

  b_kstr = obj->_pobj1.NameInternal->size[0] * obj->_pobj1.NameInternal->size[1];
  obj->_pobj1.NameInternal->size[0] = 1;
  obj->_pobj1.NameInternal->size[1] = 14;
  Cinema_emxEnsureCapacity_char_T(obj->_pobj1.NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 14; b_kstr++) {
    obj->_pobj1.NameInternal->data[b_kstr] = tmp_2[b_kstr];
  }

  b_kstr = obj->_pobj1.Type->size[0] * obj->_pobj1.Type->size[1];
  obj->_pobj1.Type->size[0] = 1;
  obj->_pobj1.Type->size[1] = 5;
  Cinema_emxEnsureCapacity_char_T(obj->_pobj1.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    obj->_pobj1.Type->data[b_kstr] = tmp_3[b_kstr];
  }

  Cinematique_ROS_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->_pobj1.Type->size[1];
  Cinema_emxEnsureCapacity_char_T(switch_expression, b_kstr);
  loop_ub = obj->_pobj1.Type->size[0] * obj->_pobj1.Type->size[1] - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = obj->_pobj1.Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp_4[b_kstr];
  }

  b_bool = false;
  if (switch_expression->size[1] == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        if (switch_expression->data[b_kstr - 1] != b[b_kstr - 1]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (b_bool) {
    b_kstr = 0;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      b_0[b_kstr] = tmp_5[b_kstr];
    }

    b_bool = false;
    if (switch_expression->size[1] == 9) {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          if (switch_expression->data[b_kstr - 1] != b_0[b_kstr - 1]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (b_bool) {
      b_kstr = 1;
    } else {
      b_kstr = -1;
    }
  }

  Cinematique_ROS_emxFree_char_T(&switch_expression);
  switch (b_kstr) {
   case 0:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 1;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 0;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    poslim_data[0] = -3.1415926535897931;
    poslim_data[1] = 3.1415926535897931;
    obj->_pobj1.VelocityNumber = 1.0;
    obj->_pobj1.PositionNumber = 1.0;
    obj->_pobj1.JointAxisInternal[0] = 0.0;
    obj->_pobj1.JointAxisInternal[1] = 0.0;
    obj->_pobj1.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 0;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 1;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    poslim_data[0] = -0.5;
    poslim_data[1] = 0.5;
    obj->_pobj1.VelocityNumber = 1.0;
    obj->_pobj1.PositionNumber = 1.0;
    obj->_pobj1.JointAxisInternal[0] = 0.0;
    obj->_pobj1.JointAxisInternal[1] = 0.0;
    obj->_pobj1.JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = 0;
    }

    poslim_data[0] = 0.0;
    poslim_data[1] = 0.0;
    obj->_pobj1.VelocityNumber = 0.0;
    obj->_pobj1.PositionNumber = 0.0;
    obj->_pobj1.JointAxisInternal[0] = 0.0;
    obj->_pobj1.JointAxisInternal[1] = 0.0;
    obj->_pobj1.JointAxisInternal[2] = 0.0;
    break;
  }

  b_kstr = obj->_pobj1.MotionSubspace->size[0] * obj->
    _pobj1.MotionSubspace->size[1];
  obj->_pobj1.MotionSubspace->size[0] = 6;
  obj->_pobj1.MotionSubspace->size[1] = 1;
  Cinema_emxEnsureCapacity_real_T(obj->_pobj1.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->_pobj1.MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  b_kstr = obj->_pobj1.PositionLimitsInternal->size[0] *
    obj->_pobj1.PositionLimitsInternal->size[1];
  obj->_pobj1.PositionLimitsInternal->size[0] = 1;
  obj->_pobj1.PositionLimitsInternal->size[1] = 2;
  Cinema_emxEnsureCapacity_real_T(obj->_pobj1.PositionLimitsInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 2; b_kstr++) {
    obj->_pobj1.PositionLimitsInternal->data[b_kstr] = poslim_data[b_kstr];
  }

  b_kstr = obj->_pobj1.HomePositionInternal->size[0];
  obj->_pobj1.HomePositionInternal->size[0] = 1;
  Cinema_emxEnsureCapacity_real_T(obj->_pobj1.HomePositionInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 1; b_kstr++) {
    obj->_pobj1.HomePositionInternal->data[0] = 0.0;
  }

  Cinemati_emxInit_unnamed_struct(&f, 2);
  obj->JointInternal = &obj->_pobj1;
  obj->Index = -1.0;
  obj->ParentIndex = -1.0;
  obj_0 = &obj->_pobj0;
  obj->_pobj0.Size = 0.0;
  obj->_pobj0.MaxElements = 0.0;
  b_kstr = f->size[0] * f->size[1];
  f->size[0] = 1;
  f->size[1] = static_cast<int32_T>(obj->_pobj0.MaxElements);
  emxEnsureCapacity_unnamed_struc(f, b_kstr);
  b_kstr = obj->_pobj0.CollisionGeometries->size[0] *
    obj->_pobj0.CollisionGeometries->size[1];
  obj->_pobj0.CollisionGeometries->size[0] = f->size[0];
  obj->_pobj0.CollisionGeometries->size[1] = f->size[1];
  emxEnsureCapacity_unnamed_struc(obj->_pobj0.CollisionGeometries, b_kstr);
  collisioncodegen_makeBox(0.0, 0.0, 0.0);
  c = obj->_pobj0.MaxElements;
  loop_ub = static_cast<int32_T>(c) - 1;
  Cinemati_emxFree_unnamed_struct(&f);
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    obj_0->CollisionGeometries->data[b_kstr] = &obj->_pobj0._pobj0;
  }

  obj->CollisionsInternal = &obj->_pobj0;
  return b_obj;
}

static y_robotics_manip_internal_Rig_T *Cine_RigidBody_RigidBody_k1e1wu
  (y_robotics_manip_internal_Rig_T *obj)
{
  static const char_T tmp_2[14] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '\x05', '_', 'j', 'n', 't' };

  static const char_T tmp_0[10] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '\x05' };

  static const char_T tmp_5[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const char_T tmp_4[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_3[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const int8_T tmp_1[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    1 };

  emxArray_char_T_Cinematique_R_T *switch_expression;
  emxArray_unnamed_struct_Cinem_T *f;
  m_robotics_manip_internal_Col_T *obj_0;
  y_robotics_manip_internal_Rig_T *b_obj;
  real_T poslim_data[12];
  real_T c;
  int32_T b_kstr;
  int32_T exitg1;
  int32_T loop_ub;
  char_T b_0[9];
  char_T b[8];
  int8_T msubspace_data[36];
  int8_T tmp[6];
  boolean_T b_bool;
  b_obj = obj;
  b_kstr = obj->NameInternal->size[0] * obj->NameInternal->size[1];
  obj->NameInternal->size[0] = 1;
  obj->NameInternal->size[1] = 10;
  Cinema_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 10; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp_0[b_kstr];
  }

  obj->_pobj1.InTree = false;
  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->_pobj1.JointToParentTransform[b_kstr] = tmp_1[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->_pobj1.ChildToJointTransform[b_kstr] = tmp_1[b_kstr];
  }

  b_kstr = obj->_pobj1.NameInternal->size[0] * obj->_pobj1.NameInternal->size[1];
  obj->_pobj1.NameInternal->size[0] = 1;
  obj->_pobj1.NameInternal->size[1] = 14;
  Cinema_emxEnsureCapacity_char_T(obj->_pobj1.NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 14; b_kstr++) {
    obj->_pobj1.NameInternal->data[b_kstr] = tmp_2[b_kstr];
  }

  b_kstr = obj->_pobj1.Type->size[0] * obj->_pobj1.Type->size[1];
  obj->_pobj1.Type->size[0] = 1;
  obj->_pobj1.Type->size[1] = 5;
  Cinema_emxEnsureCapacity_char_T(obj->_pobj1.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    obj->_pobj1.Type->data[b_kstr] = tmp_3[b_kstr];
  }

  Cinematique_ROS_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->_pobj1.Type->size[1];
  Cinema_emxEnsureCapacity_char_T(switch_expression, b_kstr);
  loop_ub = obj->_pobj1.Type->size[0] * obj->_pobj1.Type->size[1] - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = obj->_pobj1.Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp_4[b_kstr];
  }

  b_bool = false;
  if (switch_expression->size[1] == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        if (switch_expression->data[b_kstr - 1] != b[b_kstr - 1]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (b_bool) {
    b_kstr = 0;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      b_0[b_kstr] = tmp_5[b_kstr];
    }

    b_bool = false;
    if (switch_expression->size[1] == 9) {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          if (switch_expression->data[b_kstr - 1] != b_0[b_kstr - 1]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (b_bool) {
      b_kstr = 1;
    } else {
      b_kstr = -1;
    }
  }

  Cinematique_ROS_emxFree_char_T(&switch_expression);
  switch (b_kstr) {
   case 0:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 1;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 0;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    poslim_data[0] = -3.1415926535897931;
    poslim_data[1] = 3.1415926535897931;
    obj->_pobj1.VelocityNumber = 1.0;
    obj->_pobj1.PositionNumber = 1.0;
    obj->_pobj1.JointAxisInternal[0] = 0.0;
    obj->_pobj1.JointAxisInternal[1] = 0.0;
    obj->_pobj1.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 0;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 1;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    poslim_data[0] = -0.5;
    poslim_data[1] = 0.5;
    obj->_pobj1.VelocityNumber = 1.0;
    obj->_pobj1.PositionNumber = 1.0;
    obj->_pobj1.JointAxisInternal[0] = 0.0;
    obj->_pobj1.JointAxisInternal[1] = 0.0;
    obj->_pobj1.JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = 0;
    }

    poslim_data[0] = 0.0;
    poslim_data[1] = 0.0;
    obj->_pobj1.VelocityNumber = 0.0;
    obj->_pobj1.PositionNumber = 0.0;
    obj->_pobj1.JointAxisInternal[0] = 0.0;
    obj->_pobj1.JointAxisInternal[1] = 0.0;
    obj->_pobj1.JointAxisInternal[2] = 0.0;
    break;
  }

  b_kstr = obj->_pobj1.MotionSubspace->size[0] * obj->
    _pobj1.MotionSubspace->size[1];
  obj->_pobj1.MotionSubspace->size[0] = 6;
  obj->_pobj1.MotionSubspace->size[1] = 1;
  Cinema_emxEnsureCapacity_real_T(obj->_pobj1.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->_pobj1.MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  b_kstr = obj->_pobj1.PositionLimitsInternal->size[0] *
    obj->_pobj1.PositionLimitsInternal->size[1];
  obj->_pobj1.PositionLimitsInternal->size[0] = 1;
  obj->_pobj1.PositionLimitsInternal->size[1] = 2;
  Cinema_emxEnsureCapacity_real_T(obj->_pobj1.PositionLimitsInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 2; b_kstr++) {
    obj->_pobj1.PositionLimitsInternal->data[b_kstr] = poslim_data[b_kstr];
  }

  b_kstr = obj->_pobj1.HomePositionInternal->size[0];
  obj->_pobj1.HomePositionInternal->size[0] = 1;
  Cinema_emxEnsureCapacity_real_T(obj->_pobj1.HomePositionInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 1; b_kstr++) {
    obj->_pobj1.HomePositionInternal->data[0] = 0.0;
  }

  Cinemati_emxInit_unnamed_struct(&f, 2);
  obj->JointInternal = &obj->_pobj1;
  obj->Index = -1.0;
  obj->ParentIndex = -1.0;
  obj_0 = &obj->_pobj0;
  obj->_pobj0.Size = 0.0;
  obj->_pobj0.MaxElements = 0.0;
  b_kstr = f->size[0] * f->size[1];
  f->size[0] = 1;
  f->size[1] = static_cast<int32_T>(obj->_pobj0.MaxElements);
  emxEnsureCapacity_unnamed_struc(f, b_kstr);
  b_kstr = obj->_pobj0.CollisionGeometries->size[0] *
    obj->_pobj0.CollisionGeometries->size[1];
  obj->_pobj0.CollisionGeometries->size[0] = f->size[0];
  obj->_pobj0.CollisionGeometries->size[1] = f->size[1];
  emxEnsureCapacity_unnamed_struc(obj->_pobj0.CollisionGeometries, b_kstr);
  collisioncodegen_makeBox(0.0, 0.0, 0.0);
  c = obj->_pobj0.MaxElements;
  loop_ub = static_cast<int32_T>(c) - 1;
  Cinemati_emxFree_unnamed_struct(&f);
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    obj_0->CollisionGeometries->data[b_kstr] = &obj->_pobj0._pobj0;
  }

  obj->CollisionsInternal = &obj->_pobj0;
  return b_obj;
}

static void Ci_RigidBodyTree_clearAllBodies(ab_robotics_manip_internal_Ri_T *obj,
  y_robotics_manip_internal_Rig_T *iobj_0)
{
  static const char_T tmp_1[14] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '\x01', '_', 'j', 'n', 't' };

  static const char_T tmp_6[14] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '\x02', '_', 'j', 'n', 't' };

  static const char_T tmp[10] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '\x01' };

  static const char_T tmp_5[10] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '\x02' };

  static const char_T tmp_4[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const char_T tmp_3[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_2[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const int8_T tmp_0[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    1 };

  emxArray_char_T_Cinematique_R_T *switch_expression;
  emxArray_unnamed_struct_Cinem_T *f;
  emxArray_unnamed_struct_Cinem_T *f_0;
  y_robotics_manip_internal_Rig_T *obj_0;
  int32_T exitg1;
  obj_0 = &iobj_0[0];
  Cinematique_ROS_B.b_kstr_f = iobj_0[0].NameInternal->size[0] * iobj_0[0].
    NameInternal->size[1];
  iobj_0[0].NameInternal->size[0] = 1;
  iobj_0[0].NameInternal->size[1] = 10;
  Cinema_emxEnsureCapacity_char_T(iobj_0[0].NameInternal,
    Cinematique_ROS_B.b_kstr_f);
  for (Cinematique_ROS_B.b_kstr_f = 0; Cinematique_ROS_B.b_kstr_f < 10;
       Cinematique_ROS_B.b_kstr_f++) {
    iobj_0[0].NameInternal->data[Cinematique_ROS_B.b_kstr_f] =
      tmp[Cinematique_ROS_B.b_kstr_f];
  }

  obj_0->_pobj1.InTree = false;
  for (Cinematique_ROS_B.b_kstr_f = 0; Cinematique_ROS_B.b_kstr_f < 16;
       Cinematique_ROS_B.b_kstr_f++) {
    obj_0->_pobj1.JointToParentTransform[Cinematique_ROS_B.b_kstr_f] =
      tmp_0[Cinematique_ROS_B.b_kstr_f];
  }

  for (Cinematique_ROS_B.b_kstr_f = 0; Cinematique_ROS_B.b_kstr_f < 16;
       Cinematique_ROS_B.b_kstr_f++) {
    obj_0->_pobj1.ChildToJointTransform[Cinematique_ROS_B.b_kstr_f] =
      tmp_0[Cinematique_ROS_B.b_kstr_f];
  }

  Cinematique_ROS_B.b_kstr_f = obj_0->_pobj1.NameInternal->size[0] *
    obj_0->_pobj1.NameInternal->size[1];
  obj_0->_pobj1.NameInternal->size[0] = 1;
  obj_0->_pobj1.NameInternal->size[1] = 14;
  Cinema_emxEnsureCapacity_char_T(obj_0->_pobj1.NameInternal,
    Cinematique_ROS_B.b_kstr_f);
  for (Cinematique_ROS_B.b_kstr_f = 0; Cinematique_ROS_B.b_kstr_f < 14;
       Cinematique_ROS_B.b_kstr_f++) {
    obj_0->_pobj1.NameInternal->data[Cinematique_ROS_B.b_kstr_f] =
      tmp_1[Cinematique_ROS_B.b_kstr_f];
  }

  Cinematique_ROS_B.b_kstr_f = obj_0->_pobj1.Type->size[0] * obj_0->
    _pobj1.Type->size[1];
  obj_0->_pobj1.Type->size[0] = 1;
  obj_0->_pobj1.Type->size[1] = 5;
  Cinema_emxEnsureCapacity_char_T(obj_0->_pobj1.Type, Cinematique_ROS_B.b_kstr_f);
  for (Cinematique_ROS_B.b_kstr_f = 0; Cinematique_ROS_B.b_kstr_f < 5;
       Cinematique_ROS_B.b_kstr_f++) {
    obj_0->_pobj1.Type->data[Cinematique_ROS_B.b_kstr_f] =
      tmp_2[Cinematique_ROS_B.b_kstr_f];
  }

  Cinematique_ROS_emxInit_char_T(&switch_expression, 2);
  Cinematique_ROS_B.b_kstr_f = switch_expression->size[0] *
    switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj_0->_pobj1.Type->size[1];
  Cinema_emxEnsureCapacity_char_T(switch_expression, Cinematique_ROS_B.b_kstr_f);
  Cinematique_ROS_B.loop_ub_h = obj_0->_pobj1.Type->size[0] * obj_0->
    _pobj1.Type->size[1] - 1;
  for (Cinematique_ROS_B.b_kstr_f = 0; Cinematique_ROS_B.b_kstr_f <=
       Cinematique_ROS_B.loop_ub_h; Cinematique_ROS_B.b_kstr_f++) {
    switch_expression->data[Cinematique_ROS_B.b_kstr_f] = obj_0->
      _pobj1.Type->data[Cinematique_ROS_B.b_kstr_f];
  }

  for (Cinematique_ROS_B.b_kstr_f = 0; Cinematique_ROS_B.b_kstr_f < 8;
       Cinematique_ROS_B.b_kstr_f++) {
    Cinematique_ROS_B.b_bn[Cinematique_ROS_B.b_kstr_f] =
      tmp_3[Cinematique_ROS_B.b_kstr_f];
  }

  Cinematique_ROS_B.b_bool_l = false;
  if (switch_expression->size[1] == 8) {
    Cinematique_ROS_B.b_kstr_f = 1;
    do {
      exitg1 = 0;
      if (Cinematique_ROS_B.b_kstr_f - 1 < 8) {
        if (switch_expression->data[Cinematique_ROS_B.b_kstr_f - 1] !=
            Cinematique_ROS_B.b_bn[Cinematique_ROS_B.b_kstr_f - 1]) {
          exitg1 = 1;
        } else {
          Cinematique_ROS_B.b_kstr_f++;
        }
      } else {
        Cinematique_ROS_B.b_bool_l = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (Cinematique_ROS_B.b_bool_l) {
    Cinematique_ROS_B.b_kstr_f = 0;
  } else {
    for (Cinematique_ROS_B.b_kstr_f = 0; Cinematique_ROS_B.b_kstr_f < 9;
         Cinematique_ROS_B.b_kstr_f++) {
      Cinematique_ROS_B.b_l[Cinematique_ROS_B.b_kstr_f] =
        tmp_4[Cinematique_ROS_B.b_kstr_f];
    }

    Cinematique_ROS_B.b_bool_l = false;
    if (switch_expression->size[1] == 9) {
      Cinematique_ROS_B.b_kstr_f = 1;
      do {
        exitg1 = 0;
        if (Cinematique_ROS_B.b_kstr_f - 1 < 9) {
          if (switch_expression->data[Cinematique_ROS_B.b_kstr_f - 1] !=
              Cinematique_ROS_B.b_l[Cinematique_ROS_B.b_kstr_f - 1]) {
            exitg1 = 1;
          } else {
            Cinematique_ROS_B.b_kstr_f++;
          }
        } else {
          Cinematique_ROS_B.b_bool_l = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (Cinematique_ROS_B.b_bool_l) {
      Cinematique_ROS_B.b_kstr_f = 1;
    } else {
      Cinematique_ROS_B.b_kstr_f = -1;
    }
  }

  switch (Cinematique_ROS_B.b_kstr_f) {
   case 0:
    Cinematique_ROS_B.iv1[0] = 0;
    Cinematique_ROS_B.iv1[1] = 0;
    Cinematique_ROS_B.iv1[2] = 1;
    Cinematique_ROS_B.iv1[3] = 0;
    Cinematique_ROS_B.iv1[4] = 0;
    Cinematique_ROS_B.iv1[5] = 0;
    for (Cinematique_ROS_B.b_kstr_f = 0; Cinematique_ROS_B.b_kstr_f < 6;
         Cinematique_ROS_B.b_kstr_f++) {
      Cinematique_ROS_B.msubspace_data_n[Cinematique_ROS_B.b_kstr_f] =
        Cinematique_ROS_B.iv1[Cinematique_ROS_B.b_kstr_f];
    }

    Cinematique_ROS_B.poslim_data_c[0] = -3.1415926535897931;
    Cinematique_ROS_B.poslim_data_c[1] = 3.1415926535897931;
    obj_0->_pobj1.VelocityNumber = 1.0;
    obj_0->_pobj1.PositionNumber = 1.0;
    obj_0->_pobj1.JointAxisInternal[0] = 0.0;
    obj_0->_pobj1.JointAxisInternal[1] = 0.0;
    obj_0->_pobj1.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    Cinematique_ROS_B.iv1[0] = 0;
    Cinematique_ROS_B.iv1[1] = 0;
    Cinematique_ROS_B.iv1[2] = 0;
    Cinematique_ROS_B.iv1[3] = 0;
    Cinematique_ROS_B.iv1[4] = 0;
    Cinematique_ROS_B.iv1[5] = 1;
    for (Cinematique_ROS_B.b_kstr_f = 0; Cinematique_ROS_B.b_kstr_f < 6;
         Cinematique_ROS_B.b_kstr_f++) {
      Cinematique_ROS_B.msubspace_data_n[Cinematique_ROS_B.b_kstr_f] =
        Cinematique_ROS_B.iv1[Cinematique_ROS_B.b_kstr_f];
    }

    Cinematique_ROS_B.poslim_data_c[0] = -0.5;
    Cinematique_ROS_B.poslim_data_c[1] = 0.5;
    obj_0->_pobj1.VelocityNumber = 1.0;
    obj_0->_pobj1.PositionNumber = 1.0;
    obj_0->_pobj1.JointAxisInternal[0] = 0.0;
    obj_0->_pobj1.JointAxisInternal[1] = 0.0;
    obj_0->_pobj1.JointAxisInternal[2] = 1.0;
    break;

   default:
    for (Cinematique_ROS_B.b_kstr_f = 0; Cinematique_ROS_B.b_kstr_f < 6;
         Cinematique_ROS_B.b_kstr_f++) {
      Cinematique_ROS_B.msubspace_data_n[Cinematique_ROS_B.b_kstr_f] = 0;
    }

    Cinematique_ROS_B.poslim_data_c[0] = 0.0;
    Cinematique_ROS_B.poslim_data_c[1] = 0.0;
    obj_0->_pobj1.VelocityNumber = 0.0;
    obj_0->_pobj1.PositionNumber = 0.0;
    obj_0->_pobj1.JointAxisInternal[0] = 0.0;
    obj_0->_pobj1.JointAxisInternal[1] = 0.0;
    obj_0->_pobj1.JointAxisInternal[2] = 0.0;
    break;
  }

  Cinematique_ROS_B.b_kstr_f = obj_0->_pobj1.MotionSubspace->size[0] *
    obj_0->_pobj1.MotionSubspace->size[1];
  obj_0->_pobj1.MotionSubspace->size[0] = 6;
  obj_0->_pobj1.MotionSubspace->size[1] = 1;
  Cinema_emxEnsureCapacity_real_T(obj_0->_pobj1.MotionSubspace,
    Cinematique_ROS_B.b_kstr_f);
  for (Cinematique_ROS_B.b_kstr_f = 0; Cinematique_ROS_B.b_kstr_f < 6;
       Cinematique_ROS_B.b_kstr_f++) {
    obj_0->_pobj1.MotionSubspace->data[Cinematique_ROS_B.b_kstr_f] =
      Cinematique_ROS_B.msubspace_data_n[Cinematique_ROS_B.b_kstr_f];
  }

  Cinematique_ROS_B.b_kstr_f = obj_0->_pobj1.PositionLimitsInternal->size[0] *
    obj_0->_pobj1.PositionLimitsInternal->size[1];
  obj_0->_pobj1.PositionLimitsInternal->size[0] = 1;
  obj_0->_pobj1.PositionLimitsInternal->size[1] = 2;
  Cinema_emxEnsureCapacity_real_T(obj_0->_pobj1.PositionLimitsInternal,
    Cinematique_ROS_B.b_kstr_f);
  for (Cinematique_ROS_B.b_kstr_f = 0; Cinematique_ROS_B.b_kstr_f < 2;
       Cinematique_ROS_B.b_kstr_f++) {
    obj_0->_pobj1.PositionLimitsInternal->data[Cinematique_ROS_B.b_kstr_f] =
      Cinematique_ROS_B.poslim_data_c[Cinematique_ROS_B.b_kstr_f];
  }

  Cinematique_ROS_B.b_kstr_f = obj_0->_pobj1.HomePositionInternal->size[0];
  obj_0->_pobj1.HomePositionInternal->size[0] = 1;
  Cinema_emxEnsureCapacity_real_T(obj_0->_pobj1.HomePositionInternal,
    Cinematique_ROS_B.b_kstr_f);
  for (Cinematique_ROS_B.b_kstr_f = 0; Cinematique_ROS_B.b_kstr_f < 1;
       Cinematique_ROS_B.b_kstr_f++) {
    obj_0->_pobj1.HomePositionInternal->data[0] = 0.0;
  }

  Cinemati_emxInit_unnamed_struct(&f, 2);
  iobj_0[0].JointInternal = &obj_0->_pobj1;
  iobj_0[0].Index = -1.0;
  iobj_0[0].ParentIndex = -1.0;
  obj_0->_pobj0.Size = 0.0;
  obj_0->_pobj0.MaxElements = 0.0;
  Cinematique_ROS_B.b_kstr_f = f->size[0] * f->size[1];
  f->size[0] = 1;
  f->size[1] = static_cast<int32_T>(obj_0->_pobj0.MaxElements);
  emxEnsureCapacity_unnamed_struc(f, Cinematique_ROS_B.b_kstr_f);
  Cinematique_ROS_B.b_kstr_f = obj_0->_pobj0.CollisionGeometries->size[0] *
    obj_0->_pobj0.CollisionGeometries->size[1];
  obj_0->_pobj0.CollisionGeometries->size[0] = f->size[0];
  obj_0->_pobj0.CollisionGeometries->size[1] = f->size[1];
  emxEnsureCapacity_unnamed_struc(obj_0->_pobj0.CollisionGeometries,
    Cinematique_ROS_B.b_kstr_f);
  collisioncodegen_makeBox(0.0, 0.0, 0.0);
  Cinematique_ROS_B.c = obj_0->_pobj0.MaxElements;
  Cinematique_ROS_B.loop_ub_h = static_cast<int32_T>(Cinematique_ROS_B.c) - 1;
  Cinemati_emxFree_unnamed_struct(&f);
  for (Cinematique_ROS_B.b_kstr_f = 0; Cinematique_ROS_B.b_kstr_f <=
       Cinematique_ROS_B.loop_ub_h; Cinematique_ROS_B.b_kstr_f++) {
    iobj_0[0]._pobj0.CollisionGeometries->data[Cinematique_ROS_B.b_kstr_f] =
      &obj_0->_pobj0._pobj0;
  }

  iobj_0[0].CollisionsInternal = &obj_0->_pobj0;
  obj->Bodies[0] = &iobj_0[0];
  obj_0 = &iobj_0[1];
  Cinematique_ROS_B.b_kstr_f = iobj_0[1].NameInternal->size[0] * iobj_0[1].
    NameInternal->size[1];
  iobj_0[1].NameInternal->size[0] = 1;
  iobj_0[1].NameInternal->size[1] = 10;
  Cinema_emxEnsureCapacity_char_T(iobj_0[1].NameInternal,
    Cinematique_ROS_B.b_kstr_f);
  for (Cinematique_ROS_B.b_kstr_f = 0; Cinematique_ROS_B.b_kstr_f < 10;
       Cinematique_ROS_B.b_kstr_f++) {
    iobj_0[1].NameInternal->data[Cinematique_ROS_B.b_kstr_f] =
      tmp_5[Cinematique_ROS_B.b_kstr_f];
  }

  obj_0->_pobj1.InTree = false;
  for (Cinematique_ROS_B.b_kstr_f = 0; Cinematique_ROS_B.b_kstr_f < 16;
       Cinematique_ROS_B.b_kstr_f++) {
    obj_0->_pobj1.JointToParentTransform[Cinematique_ROS_B.b_kstr_f] =
      tmp_0[Cinematique_ROS_B.b_kstr_f];
  }

  for (Cinematique_ROS_B.b_kstr_f = 0; Cinematique_ROS_B.b_kstr_f < 16;
       Cinematique_ROS_B.b_kstr_f++) {
    obj_0->_pobj1.ChildToJointTransform[Cinematique_ROS_B.b_kstr_f] =
      tmp_0[Cinematique_ROS_B.b_kstr_f];
  }

  Cinematique_ROS_B.b_kstr_f = obj_0->_pobj1.NameInternal->size[0] *
    obj_0->_pobj1.NameInternal->size[1];
  obj_0->_pobj1.NameInternal->size[0] = 1;
  obj_0->_pobj1.NameInternal->size[1] = 14;
  Cinema_emxEnsureCapacity_char_T(obj_0->_pobj1.NameInternal,
    Cinematique_ROS_B.b_kstr_f);
  for (Cinematique_ROS_B.b_kstr_f = 0; Cinematique_ROS_B.b_kstr_f < 14;
       Cinematique_ROS_B.b_kstr_f++) {
    obj_0->_pobj1.NameInternal->data[Cinematique_ROS_B.b_kstr_f] =
      tmp_6[Cinematique_ROS_B.b_kstr_f];
  }

  Cinematique_ROS_B.b_kstr_f = obj_0->_pobj1.Type->size[0] * obj_0->
    _pobj1.Type->size[1];
  obj_0->_pobj1.Type->size[0] = 1;
  obj_0->_pobj1.Type->size[1] = 5;
  Cinema_emxEnsureCapacity_char_T(obj_0->_pobj1.Type, Cinematique_ROS_B.b_kstr_f);
  for (Cinematique_ROS_B.b_kstr_f = 0; Cinematique_ROS_B.b_kstr_f < 5;
       Cinematique_ROS_B.b_kstr_f++) {
    obj_0->_pobj1.Type->data[Cinematique_ROS_B.b_kstr_f] =
      tmp_2[Cinematique_ROS_B.b_kstr_f];
  }

  Cinematique_ROS_B.b_kstr_f = switch_expression->size[0] *
    switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj_0->_pobj1.Type->size[1];
  Cinema_emxEnsureCapacity_char_T(switch_expression, Cinematique_ROS_B.b_kstr_f);
  Cinematique_ROS_B.loop_ub_h = obj_0->_pobj1.Type->size[0] * obj_0->
    _pobj1.Type->size[1] - 1;
  for (Cinematique_ROS_B.b_kstr_f = 0; Cinematique_ROS_B.b_kstr_f <=
       Cinematique_ROS_B.loop_ub_h; Cinematique_ROS_B.b_kstr_f++) {
    switch_expression->data[Cinematique_ROS_B.b_kstr_f] = obj_0->
      _pobj1.Type->data[Cinematique_ROS_B.b_kstr_f];
  }

  Cinematique_ROS_B.b_bool_l = false;
  if (switch_expression->size[1] == 8) {
    Cinematique_ROS_B.b_kstr_f = 1;
    do {
      exitg1 = 0;
      if (Cinematique_ROS_B.b_kstr_f - 1 < 8) {
        if (switch_expression->data[Cinematique_ROS_B.b_kstr_f - 1] !=
            Cinematique_ROS_B.b_bn[Cinematique_ROS_B.b_kstr_f - 1]) {
          exitg1 = 1;
        } else {
          Cinematique_ROS_B.b_kstr_f++;
        }
      } else {
        Cinematique_ROS_B.b_bool_l = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (Cinematique_ROS_B.b_bool_l) {
    Cinematique_ROS_B.b_kstr_f = 0;
  } else {
    for (Cinematique_ROS_B.b_kstr_f = 0; Cinematique_ROS_B.b_kstr_f < 9;
         Cinematique_ROS_B.b_kstr_f++) {
      Cinematique_ROS_B.b_l[Cinematique_ROS_B.b_kstr_f] =
        tmp_4[Cinematique_ROS_B.b_kstr_f];
    }

    Cinematique_ROS_B.b_bool_l = false;
    if (switch_expression->size[1] == 9) {
      Cinematique_ROS_B.b_kstr_f = 1;
      do {
        exitg1 = 0;
        if (Cinematique_ROS_B.b_kstr_f - 1 < 9) {
          if (switch_expression->data[Cinematique_ROS_B.b_kstr_f - 1] !=
              Cinematique_ROS_B.b_l[Cinematique_ROS_B.b_kstr_f - 1]) {
            exitg1 = 1;
          } else {
            Cinematique_ROS_B.b_kstr_f++;
          }
        } else {
          Cinematique_ROS_B.b_bool_l = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (Cinematique_ROS_B.b_bool_l) {
      Cinematique_ROS_B.b_kstr_f = 1;
    } else {
      Cinematique_ROS_B.b_kstr_f = -1;
    }
  }

  Cinematique_ROS_emxFree_char_T(&switch_expression);
  switch (Cinematique_ROS_B.b_kstr_f) {
   case 0:
    Cinematique_ROS_B.iv1[0] = 0;
    Cinematique_ROS_B.iv1[1] = 0;
    Cinematique_ROS_B.iv1[2] = 1;
    Cinematique_ROS_B.iv1[3] = 0;
    Cinematique_ROS_B.iv1[4] = 0;
    Cinematique_ROS_B.iv1[5] = 0;
    for (Cinematique_ROS_B.b_kstr_f = 0; Cinematique_ROS_B.b_kstr_f < 6;
         Cinematique_ROS_B.b_kstr_f++) {
      Cinematique_ROS_B.msubspace_data_n[Cinematique_ROS_B.b_kstr_f] =
        Cinematique_ROS_B.iv1[Cinematique_ROS_B.b_kstr_f];
    }

    Cinematique_ROS_B.poslim_data_c[0] = -3.1415926535897931;
    Cinematique_ROS_B.poslim_data_c[1] = 3.1415926535897931;
    obj_0->_pobj1.VelocityNumber = 1.0;
    obj_0->_pobj1.PositionNumber = 1.0;
    obj_0->_pobj1.JointAxisInternal[0] = 0.0;
    obj_0->_pobj1.JointAxisInternal[1] = 0.0;
    obj_0->_pobj1.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    Cinematique_ROS_B.iv1[0] = 0;
    Cinematique_ROS_B.iv1[1] = 0;
    Cinematique_ROS_B.iv1[2] = 0;
    Cinematique_ROS_B.iv1[3] = 0;
    Cinematique_ROS_B.iv1[4] = 0;
    Cinematique_ROS_B.iv1[5] = 1;
    for (Cinematique_ROS_B.b_kstr_f = 0; Cinematique_ROS_B.b_kstr_f < 6;
         Cinematique_ROS_B.b_kstr_f++) {
      Cinematique_ROS_B.msubspace_data_n[Cinematique_ROS_B.b_kstr_f] =
        Cinematique_ROS_B.iv1[Cinematique_ROS_B.b_kstr_f];
    }

    Cinematique_ROS_B.poslim_data_c[0] = -0.5;
    Cinematique_ROS_B.poslim_data_c[1] = 0.5;
    obj_0->_pobj1.VelocityNumber = 1.0;
    obj_0->_pobj1.PositionNumber = 1.0;
    obj_0->_pobj1.JointAxisInternal[0] = 0.0;
    obj_0->_pobj1.JointAxisInternal[1] = 0.0;
    obj_0->_pobj1.JointAxisInternal[2] = 1.0;
    break;

   default:
    for (Cinematique_ROS_B.b_kstr_f = 0; Cinematique_ROS_B.b_kstr_f < 6;
         Cinematique_ROS_B.b_kstr_f++) {
      Cinematique_ROS_B.msubspace_data_n[Cinematique_ROS_B.b_kstr_f] = 0;
    }

    Cinematique_ROS_B.poslim_data_c[0] = 0.0;
    Cinematique_ROS_B.poslim_data_c[1] = 0.0;
    obj_0->_pobj1.VelocityNumber = 0.0;
    obj_0->_pobj1.PositionNumber = 0.0;
    obj_0->_pobj1.JointAxisInternal[0] = 0.0;
    obj_0->_pobj1.JointAxisInternal[1] = 0.0;
    obj_0->_pobj1.JointAxisInternal[2] = 0.0;
    break;
  }

  Cinematique_ROS_B.b_kstr_f = obj_0->_pobj1.MotionSubspace->size[0] *
    obj_0->_pobj1.MotionSubspace->size[1];
  obj_0->_pobj1.MotionSubspace->size[0] = 6;
  obj_0->_pobj1.MotionSubspace->size[1] = 1;
  Cinema_emxEnsureCapacity_real_T(obj_0->_pobj1.MotionSubspace,
    Cinematique_ROS_B.b_kstr_f);
  for (Cinematique_ROS_B.b_kstr_f = 0; Cinematique_ROS_B.b_kstr_f < 6;
       Cinematique_ROS_B.b_kstr_f++) {
    obj_0->_pobj1.MotionSubspace->data[Cinematique_ROS_B.b_kstr_f] =
      Cinematique_ROS_B.msubspace_data_n[Cinematique_ROS_B.b_kstr_f];
  }

  Cinematique_ROS_B.b_kstr_f = obj_0->_pobj1.PositionLimitsInternal->size[0] *
    obj_0->_pobj1.PositionLimitsInternal->size[1];
  obj_0->_pobj1.PositionLimitsInternal->size[0] = 1;
  obj_0->_pobj1.PositionLimitsInternal->size[1] = 2;
  Cinema_emxEnsureCapacity_real_T(obj_0->_pobj1.PositionLimitsInternal,
    Cinematique_ROS_B.b_kstr_f);
  for (Cinematique_ROS_B.b_kstr_f = 0; Cinematique_ROS_B.b_kstr_f < 2;
       Cinematique_ROS_B.b_kstr_f++) {
    obj_0->_pobj1.PositionLimitsInternal->data[Cinematique_ROS_B.b_kstr_f] =
      Cinematique_ROS_B.poslim_data_c[Cinematique_ROS_B.b_kstr_f];
  }

  Cinematique_ROS_B.b_kstr_f = obj_0->_pobj1.HomePositionInternal->size[0];
  obj_0->_pobj1.HomePositionInternal->size[0] = 1;
  Cinema_emxEnsureCapacity_real_T(obj_0->_pobj1.HomePositionInternal,
    Cinematique_ROS_B.b_kstr_f);
  for (Cinematique_ROS_B.b_kstr_f = 0; Cinematique_ROS_B.b_kstr_f < 1;
       Cinematique_ROS_B.b_kstr_f++) {
    obj_0->_pobj1.HomePositionInternal->data[0] = 0.0;
  }

  Cinemati_emxInit_unnamed_struct(&f_0, 2);
  iobj_0[1].JointInternal = &obj_0->_pobj1;
  iobj_0[1].Index = -1.0;
  iobj_0[1].ParentIndex = -1.0;
  obj_0->_pobj0.Size = 0.0;
  obj_0->_pobj0.MaxElements = 0.0;
  Cinematique_ROS_B.b_kstr_f = f_0->size[0] * f_0->size[1];
  f_0->size[0] = 1;
  f_0->size[1] = static_cast<int32_T>(obj_0->_pobj0.MaxElements);
  emxEnsureCapacity_unnamed_struc(f_0, Cinematique_ROS_B.b_kstr_f);
  Cinematique_ROS_B.b_kstr_f = obj_0->_pobj0.CollisionGeometries->size[0] *
    obj_0->_pobj0.CollisionGeometries->size[1];
  obj_0->_pobj0.CollisionGeometries->size[0] = f_0->size[0];
  obj_0->_pobj0.CollisionGeometries->size[1] = f_0->size[1];
  emxEnsureCapacity_unnamed_struc(obj_0->_pobj0.CollisionGeometries,
    Cinematique_ROS_B.b_kstr_f);
  collisioncodegen_makeBox(0.0, 0.0, 0.0);
  Cinematique_ROS_B.c = obj_0->_pobj0.MaxElements;
  Cinematique_ROS_B.loop_ub_h = static_cast<int32_T>(Cinematique_ROS_B.c) - 1;
  Cinemati_emxFree_unnamed_struct(&f_0);
  for (Cinematique_ROS_B.b_kstr_f = 0; Cinematique_ROS_B.b_kstr_f <=
       Cinematique_ROS_B.loop_ub_h; Cinematique_ROS_B.b_kstr_f++) {
    iobj_0[1]._pobj0.CollisionGeometries->data[Cinematique_ROS_B.b_kstr_f] =
      &obj_0->_pobj0._pobj0;
  }

  iobj_0[1].CollisionsInternal = &obj_0->_pobj0;
  obj->Bodies[1] = &iobj_0[1];
  obj->Bodies[2] = Cinema_RigidBody_RigidBody_k1e1(&iobj_0[2]);
  obj->Bodies[3] = Cinem_RigidBody_RigidBody_k1e1w(&iobj_0[3]);
  obj->Bodies[4] = Cine_RigidBody_RigidBody_k1e1wu(&iobj_0[4]);
  obj->NumBodies = 0.0;
  obj->NumNonFixedBodies = 0.0;
  obj->PositionNumber = 0.0;
  obj->VelocityNumber = 0.0;
  Cinematique_ROS_rand(Cinematique_ROS_B.unusedExpr_m);
  for (Cinematique_ROS_B.b_kstr_f = 0; Cinematique_ROS_B.b_kstr_f < 5;
       Cinematique_ROS_B.b_kstr_f++) {
    obj->PositionDoFMap[Cinematique_ROS_B.b_kstr_f] = 0.0;
  }

  for (Cinematique_ROS_B.b_kstr_f = 0; Cinematique_ROS_B.b_kstr_f < 5;
       Cinematique_ROS_B.b_kstr_f++) {
    obj->PositionDoFMap[Cinematique_ROS_B.b_kstr_f + 5] = -1.0;
  }

  for (Cinematique_ROS_B.b_kstr_f = 0; Cinematique_ROS_B.b_kstr_f < 5;
       Cinematique_ROS_B.b_kstr_f++) {
    obj->VelocityDoFMap[Cinematique_ROS_B.b_kstr_f] = 0.0;
  }

  for (Cinematique_ROS_B.b_kstr_f = 0; Cinematique_ROS_B.b_kstr_f < 5;
       Cinematique_ROS_B.b_kstr_f++) {
    obj->VelocityDoFMap[Cinematique_ROS_B.b_kstr_f + 5] = -1.0;
  }
}

static boolean_T Cinematique_ROS_strcmp(const emxArray_char_T_Cinematique_R_T *a,
  const emxArray_char_T_Cinematique_R_T *b)
{
  int32_T b_kstr;
  int32_T exitg1;
  boolean_T b_bool;
  boolean_T d;
  b_bool = false;
  d = (a->size[1] == 0);
  if (d && (b->size[1] == 0)) {
    b_bool = true;
  } else if (a->size[1] != b->size[1]) {
  } else {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 <= b->size[1] - 1) {
        if (a->data[b_kstr - 1] != b->data[b_kstr - 1]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  return b_bool;
}

static m_robotics_manip_internal_Col_T *Cinematique_R_CollisionSet_copy
  (m_robotics_manip_internal_Col_T *obj, m_robotics_manip_internal_Col_T *iobj_0)
{
  emxArray_unnamed_struct_Cinem_T *f;
  l_robotics_manip_internal_Col_T *obj_0;
  m_robotics_manip_internal_Col_T *newObj;
  real_T maxElements;
  int32_T b_i;
  int32_T d;
  Cinemati_emxInit_unnamed_struct(&f, 2);
  maxElements = obj->MaxElements;
  iobj_0->Size = 0.0;
  newObj = iobj_0;
  iobj_0->MaxElements = maxElements;
  b_i = f->size[0] * f->size[1];
  f->size[0] = 1;
  f->size[1] = static_cast<int32_T>(iobj_0->MaxElements);
  emxEnsureCapacity_unnamed_struc(f, b_i);
  b_i = iobj_0->CollisionGeometries->size[0] * iobj_0->CollisionGeometries->
    size[1];
  iobj_0->CollisionGeometries->size[0] = f->size[0];
  iobj_0->CollisionGeometries->size[1] = f->size[1];
  emxEnsureCapacity_unnamed_struc(iobj_0->CollisionGeometries, b_i);
  collisioncodegen_makeBox(0.0, 0.0, 0.0);
  maxElements = iobj_0->MaxElements;
  d = static_cast<int32_T>(maxElements) - 1;
  Cinemati_emxFree_unnamed_struct(&f);
  for (b_i = 0; b_i <= d; b_i++) {
    iobj_0->CollisionGeometries->data[b_i] = &iobj_0->_pobj0;
  }

  iobj_0->Size = obj->Size;
  maxElements = obj->Size;
  d = static_cast<int32_T>(maxElements) - 1;
  for (b_i = 0; b_i <= d; b_i++) {
    obj_0 = obj->CollisionGeometries->data[b_i];
    iobj_0->CollisionGeometries->data[b_i] = obj_0;
  }

  return newObj;
}

static real_T RigidBodyTree_findBodyIndexByNa(ab_robotics_manip_internal_Ri_T
  *obj, const emxArray_char_T_Cinematique_R_T *bodyname)
{
  emxArray_char_T_Cinematique_R_T *bname;
  y_robotics_manip_internal_Rig_T *obj_0;
  real_T bid;
  boolean_T exitg1;
  Cinematique_ROS_emxInit_char_T(&bname, 2);
  bid = -1.0;
  Cinematique_ROS_B.i2 = bname->size[0] * bname->size[1];
  bname->size[0] = 1;
  bname->size[1] = obj->Base.NameInternal->size[1];
  Cinema_emxEnsureCapacity_char_T(bname, Cinematique_ROS_B.i2);
  Cinematique_ROS_B.loop_ub_i = obj->Base.NameInternal->size[0] *
    obj->Base.NameInternal->size[1] - 1;
  for (Cinematique_ROS_B.i2 = 0; Cinematique_ROS_B.i2 <=
       Cinematique_ROS_B.loop_ub_i; Cinematique_ROS_B.i2++) {
    bname->data[Cinematique_ROS_B.i2] = obj->Base.NameInternal->
      data[Cinematique_ROS_B.i2];
  }

  if (Cinematique_ROS_strcmp(bname, bodyname)) {
    bid = 0.0;
  } else {
    Cinematique_ROS_B.b_a = obj->NumBodies;
    Cinematique_ROS_B.b_i_n = 0;
    exitg1 = false;
    while ((!exitg1) && (Cinematique_ROS_B.b_i_n <= static_cast<int32_T>
                         (Cinematique_ROS_B.b_a) - 1)) {
      obj_0 = obj->Bodies[Cinematique_ROS_B.b_i_n];
      Cinematique_ROS_B.i2 = bname->size[0] * bname->size[1];
      bname->size[0] = 1;
      bname->size[1] = obj_0->NameInternal->size[1];
      Cinema_emxEnsureCapacity_char_T(bname, Cinematique_ROS_B.i2);
      Cinematique_ROS_B.loop_ub_i = obj_0->NameInternal->size[0] *
        obj_0->NameInternal->size[1] - 1;
      for (Cinematique_ROS_B.i2 = 0; Cinematique_ROS_B.i2 <=
           Cinematique_ROS_B.loop_ub_i; Cinematique_ROS_B.i2++) {
        bname->data[Cinematique_ROS_B.i2] = obj_0->NameInternal->
          data[Cinematique_ROS_B.i2];
      }

      if (Cinematique_ROS_strcmp(bname, bodyname)) {
        bid = static_cast<real_T>(Cinematique_ROS_B.b_i_n) + 1.0;
        exitg1 = true;
      } else {
        Cinematique_ROS_B.b_i_n++;
      }
    }
  }

  Cinematique_ROS_emxFree_char_T(&bname);
  return bid;
}

static void Cinematique_ROS_emxFree_real_T(emxArray_real_T_Cinematique_R_T
  **pEmxArray)
{
  if (*pEmxArray != (emxArray_real_T_Cinematique_R_T *)NULL) {
    if (((*pEmxArray)->data != (real_T *)NULL) && (*pEmxArray)->canFreeData) {
      free((*pEmxArray)->data);
    }

    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = (emxArray_real_T_Cinematique_R_T *)NULL;
  }
}

static c_rigidBodyJoint_Cinematique__T *Cinematique_rigidBodyJoint_copy(const
  c_rigidBodyJoint_Cinematique__T *obj, c_rigidBodyJoint_Cinematique__T *iobj_0)
{
  static const char_T tmp_3[128] = { '\x00', '\x01', '\x02', '\x03', '\x04',
    '\x05', '\x06', '\x07', '\x08', '\x09', '\x0a', '\x0b', '\x0c', '\x0d',
    '\x0e', '\x0f', '\x10', '\x11', '\x12', '\x13', '\x14', '\x15', '\x16',
    '\x17', '\x18', '\x19', '\x1a', '\x1b', '\x1c', '\x1d', '\x1e', '\x1f', ' ',
    '!', '\"', '#', '$', '%', '&', '\'', '(', ')', '*', '+', ',', '-', '.', '/',
    '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', ':', ';', '<', '=', '>',
    '?', '@', 'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l', 'm',
    'n', 'o', 'p', 'q', 'r', 's', 't', 'u', 'v', 'w', 'x', 'y', 'z', '[', '\\',
    ']', '^', '_', '`', 'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k',
    'l', 'm', 'n', 'o', 'p', 'q', 'r', 's', 't', 'u', 'v', 'w', 'x', 'y', 'z',
    '{', '|', '}', '~', '\x7f' };

  static const char_T tmp_1[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const char_T tmp_0[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_2[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const int8_T tmp[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1
  };

  c_rigidBodyJoint_Cinematique__T *newjoint;
  emxArray_char_T_Cinematique_R_T *jname;
  emxArray_char_T_Cinematique_R_T *jtype;
  emxArray_real_T_Cinematique_R_T *obj_0;
  emxArray_real_T_Cinematique_R_T *obj_1;
  emxArray_real_T_Cinematique_R_T *obj_2;
  int32_T exitg1;
  boolean_T guard1 = false;
  boolean_T guard11 = false;
  boolean_T guard2 = false;
  boolean_T guard3 = false;
  boolean_T guard4 = false;
  Cinematique_ROS_emxInit_char_T(&jtype, 2);
  Cinematique_ROS_B.minnanb = jtype->size[0] * jtype->size[1];
  jtype->size[0] = 1;
  jtype->size[1] = obj->Type->size[1];
  Cinema_emxEnsureCapacity_char_T(jtype, Cinematique_ROS_B.minnanb);
  Cinematique_ROS_B.loop_ub_j = obj->Type->size[0] * obj->Type->size[1] - 1;
  for (Cinematique_ROS_B.minnanb = 0; Cinematique_ROS_B.minnanb <=
       Cinematique_ROS_B.loop_ub_j; Cinematique_ROS_B.minnanb++) {
    jtype->data[Cinematique_ROS_B.minnanb] = obj->Type->
      data[Cinematique_ROS_B.minnanb];
  }

  Cinematique_ROS_emxInit_char_T(&jname, 2);
  Cinematique_ROS_B.minnanb = jname->size[0] * jname->size[1];
  jname->size[0] = 1;
  jname->size[1] = obj->NameInternal->size[1];
  Cinema_emxEnsureCapacity_char_T(jname, Cinematique_ROS_B.minnanb);
  Cinematique_ROS_B.loop_ub_j = obj->NameInternal->size[0] * obj->
    NameInternal->size[1] - 1;
  for (Cinematique_ROS_B.minnanb = 0; Cinematique_ROS_B.minnanb <=
       Cinematique_ROS_B.loop_ub_j; Cinematique_ROS_B.minnanb++) {
    jname->data[Cinematique_ROS_B.minnanb] = obj->NameInternal->
      data[Cinematique_ROS_B.minnanb];
  }

  iobj_0->InTree = false;
  for (Cinematique_ROS_B.minnanb = 0; Cinematique_ROS_B.minnanb < 16;
       Cinematique_ROS_B.minnanb++) {
    iobj_0->JointToParentTransform[Cinematique_ROS_B.minnanb] =
      tmp[Cinematique_ROS_B.minnanb];
  }

  for (Cinematique_ROS_B.minnanb = 0; Cinematique_ROS_B.minnanb < 16;
       Cinematique_ROS_B.minnanb++) {
    iobj_0->ChildToJointTransform[Cinematique_ROS_B.minnanb] =
      tmp[Cinematique_ROS_B.minnanb];
  }

  newjoint = iobj_0;
  Cinematique_ROS_B.minnanb = iobj_0->NameInternal->size[0] *
    iobj_0->NameInternal->size[1];
  iobj_0->NameInternal->size[0] = 1;
  iobj_0->NameInternal->size[1] = jname->size[1];
  Cinema_emxEnsureCapacity_char_T(iobj_0->NameInternal,
    Cinematique_ROS_B.minnanb);
  Cinematique_ROS_B.loop_ub_j = jname->size[0] * jname->size[1] - 1;
  for (Cinematique_ROS_B.minnanb = 0; Cinematique_ROS_B.minnanb <=
       Cinematique_ROS_B.loop_ub_j; Cinematique_ROS_B.minnanb++) {
    iobj_0->NameInternal->data[Cinematique_ROS_B.minnanb] = jname->
      data[Cinematique_ROS_B.minnanb];
  }

  Cinematique_ROS_emxFree_char_T(&jname);
  Cinematique_ROS_B.partial_match_size_idx_1 = 0;
  for (Cinematique_ROS_B.minnanb = 0; Cinematique_ROS_B.minnanb < 8;
       Cinematique_ROS_B.minnanb++) {
    Cinematique_ROS_B.vstr[Cinematique_ROS_B.minnanb] =
      tmp_0[Cinematique_ROS_B.minnanb];
  }

  guard1 = false;
  guard2 = false;
  guard3 = false;
  guard4 = false;
  if (jtype->size[1] <= 8) {
    Cinematique_ROS_B.loop_ub_j = jtype->size[1];
    for (Cinematique_ROS_B.minnanb = 0; Cinematique_ROS_B.minnanb < 8;
         Cinematique_ROS_B.minnanb++) {
      Cinematique_ROS_B.b_bj[Cinematique_ROS_B.minnanb] =
        tmp_0[Cinematique_ROS_B.minnanb];
    }

    Cinematique_ROS_B.b_bool_j = false;
    Cinematique_ROS_B.minnanb = jtype->size[1];
    if (Cinematique_ROS_B.minnanb >= 8) {
      Cinematique_ROS_B.minnanb = 8;
    }

    guard11 = false;
    if (Cinematique_ROS_B.loop_ub_j <= Cinematique_ROS_B.minnanb) {
      if (Cinematique_ROS_B.minnanb < Cinematique_ROS_B.loop_ub_j) {
        Cinematique_ROS_B.loop_ub_j = Cinematique_ROS_B.minnanb;
      }

      Cinematique_ROS_B.minnanb = Cinematique_ROS_B.loop_ub_j - 1;
      guard11 = true;
    } else {
      if (jtype->size[1] == 8) {
        Cinematique_ROS_B.minnanb = 7;
        guard11 = true;
      }
    }

    if (guard11) {
      Cinematique_ROS_B.loop_ub_j = 1;
      do {
        exitg1 = 0;
        if (Cinematique_ROS_B.loop_ub_j - 1 <= Cinematique_ROS_B.minnanb) {
          if (tmp_3[static_cast<uint8_T>(jtype->data[Cinematique_ROS_B.loop_ub_j
               - 1]) & 127] != tmp_3[static_cast<int32_T>
              (Cinematique_ROS_B.b_bj[Cinematique_ROS_B.loop_ub_j - 1])]) {
            exitg1 = 1;
          } else {
            Cinematique_ROS_B.loop_ub_j++;
          }
        } else {
          Cinematique_ROS_B.b_bool_j = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (Cinematique_ROS_B.b_bool_j) {
      if (jtype->size[1] == 8) {
        Cinematique_ROS_B.nmatched = 1;
        Cinematique_ROS_B.partial_match_size_idx_1 = 8;
        for (Cinematique_ROS_B.minnanb = 0; Cinematique_ROS_B.minnanb < 8;
             Cinematique_ROS_B.minnanb++) {
          Cinematique_ROS_B.b_n[Cinematique_ROS_B.minnanb] =
            Cinematique_ROS_B.vstr[Cinematique_ROS_B.minnanb];
        }
      } else {
        Cinematique_ROS_B.partial_match_size_idx_1 = 8;
        for (Cinematique_ROS_B.minnanb = 0; Cinematique_ROS_B.minnanb < 8;
             Cinematique_ROS_B.minnanb++) {
          Cinematique_ROS_B.partial_match_data[Cinematique_ROS_B.minnanb] =
            Cinematique_ROS_B.vstr[Cinematique_ROS_B.minnanb];
        }

        Cinematique_ROS_B.matched = true;
        Cinematique_ROS_B.nmatched = 1;
        guard3 = true;
      }
    } else {
      guard4 = true;
    }
  } else {
    guard4 = true;
  }

  if (guard4) {
    Cinematique_ROS_B.matched = false;
    Cinematique_ROS_B.nmatched = 0;
    guard3 = true;
  }

  if (guard3) {
    for (Cinematique_ROS_B.minnanb = 0; Cinematique_ROS_B.minnanb < 9;
         Cinematique_ROS_B.minnanb++) {
      Cinematique_ROS_B.b_vstr[Cinematique_ROS_B.minnanb] =
        tmp_1[Cinematique_ROS_B.minnanb];
    }

    if (jtype->size[1] <= 9) {
      Cinematique_ROS_B.loop_ub_j = jtype->size[1];
      for (Cinematique_ROS_B.minnanb = 0; Cinematique_ROS_B.minnanb < 9;
           Cinematique_ROS_B.minnanb++) {
        Cinematique_ROS_B.b_n[Cinematique_ROS_B.minnanb] =
          tmp_1[Cinematique_ROS_B.minnanb];
      }

      Cinematique_ROS_B.b_bool_j = false;
      Cinematique_ROS_B.minnanb = jtype->size[1];
      if (Cinematique_ROS_B.minnanb >= 9) {
        Cinematique_ROS_B.minnanb = 9;
      }

      guard11 = false;
      if (Cinematique_ROS_B.loop_ub_j <= Cinematique_ROS_B.minnanb) {
        if (Cinematique_ROS_B.minnanb < Cinematique_ROS_B.loop_ub_j) {
          Cinematique_ROS_B.loop_ub_j = Cinematique_ROS_B.minnanb;
        }

        Cinematique_ROS_B.minnanb = Cinematique_ROS_B.loop_ub_j - 1;
        guard11 = true;
      } else {
        if (jtype->size[1] == 9) {
          Cinematique_ROS_B.minnanb = 8;
          guard11 = true;
        }
      }

      if (guard11) {
        Cinematique_ROS_B.loop_ub_j = 1;
        do {
          exitg1 = 0;
          if (Cinematique_ROS_B.loop_ub_j - 1 <= Cinematique_ROS_B.minnanb) {
            if (tmp_3[static_cast<uint8_T>(jtype->
                 data[Cinematique_ROS_B.loop_ub_j - 1]) & 127] != tmp_3[
                static_cast<int32_T>
                (Cinematique_ROS_B.b_n[Cinematique_ROS_B.loop_ub_j - 1])]) {
              exitg1 = 1;
            } else {
              Cinematique_ROS_B.loop_ub_j++;
            }
          } else {
            Cinematique_ROS_B.b_bool_j = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }

      if (Cinematique_ROS_B.b_bool_j) {
        if (jtype->size[1] == 9) {
          Cinematique_ROS_B.nmatched = 1;
          Cinematique_ROS_B.partial_match_size_idx_1 = 9;
          for (Cinematique_ROS_B.minnanb = 0; Cinematique_ROS_B.minnanb < 9;
               Cinematique_ROS_B.minnanb++) {
            Cinematique_ROS_B.b_n[Cinematique_ROS_B.minnanb] =
              Cinematique_ROS_B.b_vstr[Cinematique_ROS_B.minnanb];
          }
        } else {
          if (!Cinematique_ROS_B.matched) {
            Cinematique_ROS_B.partial_match_size_idx_1 = 9;
            for (Cinematique_ROS_B.minnanb = 0; Cinematique_ROS_B.minnanb < 9;
                 Cinematique_ROS_B.minnanb++) {
              Cinematique_ROS_B.partial_match_data[Cinematique_ROS_B.minnanb] =
                Cinematique_ROS_B.b_vstr[Cinematique_ROS_B.minnanb];
            }
          }

          Cinematique_ROS_B.matched = true;
          Cinematique_ROS_B.nmatched++;
          guard2 = true;
        }
      } else {
        guard2 = true;
      }
    } else {
      guard2 = true;
    }
  }

  if (guard2) {
    for (Cinematique_ROS_B.minnanb = 0; Cinematique_ROS_B.minnanb < 5;
         Cinematique_ROS_B.minnanb++) {
      Cinematique_ROS_B.c_vstr[Cinematique_ROS_B.minnanb] =
        tmp_2[Cinematique_ROS_B.minnanb];
    }

    if (jtype->size[1] <= 5) {
      Cinematique_ROS_B.loop_ub_j = jtype->size[1];
      for (Cinematique_ROS_B.minnanb = 0; Cinematique_ROS_B.minnanb < 5;
           Cinematique_ROS_B.minnanb++) {
        Cinematique_ROS_B.b_o3[Cinematique_ROS_B.minnanb] =
          tmp_2[Cinematique_ROS_B.minnanb];
      }

      Cinematique_ROS_B.b_bool_j = false;
      Cinematique_ROS_B.minnanb = jtype->size[1];
      if (Cinematique_ROS_B.minnanb >= 5) {
        Cinematique_ROS_B.minnanb = 5;
      }

      guard11 = false;
      if (Cinematique_ROS_B.loop_ub_j <= Cinematique_ROS_B.minnanb) {
        if (Cinematique_ROS_B.minnanb < Cinematique_ROS_B.loop_ub_j) {
          Cinematique_ROS_B.loop_ub_j = Cinematique_ROS_B.minnanb;
        }

        Cinematique_ROS_B.minnanb = Cinematique_ROS_B.loop_ub_j - 1;
        guard11 = true;
      } else {
        if (jtype->size[1] == 5) {
          Cinematique_ROS_B.minnanb = 4;
          guard11 = true;
        }
      }

      if (guard11) {
        Cinematique_ROS_B.loop_ub_j = 1;
        do {
          exitg1 = 0;
          if (Cinematique_ROS_B.loop_ub_j - 1 <= Cinematique_ROS_B.minnanb) {
            if (tmp_3[static_cast<uint8_T>(jtype->
                 data[Cinematique_ROS_B.loop_ub_j - 1]) & 127] != tmp_3[
                static_cast<int32_T>
                (Cinematique_ROS_B.b_o3[Cinematique_ROS_B.loop_ub_j - 1])]) {
              exitg1 = 1;
            } else {
              Cinematique_ROS_B.loop_ub_j++;
            }
          } else {
            Cinematique_ROS_B.b_bool_j = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }

      if (Cinematique_ROS_B.b_bool_j) {
        if (jtype->size[1] == 5) {
          Cinematique_ROS_B.nmatched = 1;
          Cinematique_ROS_B.partial_match_size_idx_1 = 5;
          for (Cinematique_ROS_B.minnanb = 0; Cinematique_ROS_B.minnanb < 5;
               Cinematique_ROS_B.minnanb++) {
            Cinematique_ROS_B.b_n[Cinematique_ROS_B.minnanb] =
              Cinematique_ROS_B.c_vstr[Cinematique_ROS_B.minnanb];
          }
        } else {
          if (!Cinematique_ROS_B.matched) {
            Cinematique_ROS_B.partial_match_size_idx_1 = 5;
            for (Cinematique_ROS_B.minnanb = 0; Cinematique_ROS_B.minnanb < 5;
                 Cinematique_ROS_B.minnanb++) {
              Cinematique_ROS_B.partial_match_data[Cinematique_ROS_B.minnanb] =
                Cinematique_ROS_B.c_vstr[Cinematique_ROS_B.minnanb];
            }
          }

          Cinematique_ROS_B.nmatched++;
          guard1 = true;
        }
      } else {
        guard1 = true;
      }
    } else {
      guard1 = true;
    }
  }

  if (guard1) {
    if (Cinematique_ROS_B.nmatched == 0) {
      Cinematique_ROS_B.partial_match_size_idx_1 = 0;
    } else {
      Cinematique_ROS_B.loop_ub_j = Cinematique_ROS_B.partial_match_size_idx_1 -
        1;
      if (0 <= Cinematique_ROS_B.loop_ub_j) {
        memcpy(&Cinematique_ROS_B.b_n[0], &Cinematique_ROS_B.partial_match_data
               [0], (Cinematique_ROS_B.loop_ub_j + 1) * sizeof(char_T));
      }
    }
  }

  if ((Cinematique_ROS_B.nmatched == 0) || ((jtype->size[1] == 0) !=
       (Cinematique_ROS_B.partial_match_size_idx_1 == 0))) {
    Cinematique_ROS_B.partial_match_size_idx_1 = 0;
  } else {
    Cinematique_ROS_B.loop_ub_j = Cinematique_ROS_B.partial_match_size_idx_1 - 1;
    if (0 <= Cinematique_ROS_B.loop_ub_j) {
      memcpy(&Cinematique_ROS_B.partial_match_data[0], &Cinematique_ROS_B.b_n[0],
             (Cinematique_ROS_B.loop_ub_j + 1) * sizeof(char_T));
    }
  }

  Cinematique_ROS_B.minnanb = iobj_0->Type->size[0] * iobj_0->Type->size[1];
  iobj_0->Type->size[0] = 1;
  iobj_0->Type->size[1] = Cinematique_ROS_B.partial_match_size_idx_1;
  Cinema_emxEnsureCapacity_char_T(iobj_0->Type, Cinematique_ROS_B.minnanb);
  Cinematique_ROS_B.loop_ub_j = Cinematique_ROS_B.partial_match_size_idx_1 - 1;
  for (Cinematique_ROS_B.minnanb = 0; Cinematique_ROS_B.minnanb <=
       Cinematique_ROS_B.loop_ub_j; Cinematique_ROS_B.minnanb++) {
    iobj_0->Type->data[Cinematique_ROS_B.minnanb] =
      Cinematique_ROS_B.partial_match_data[Cinematique_ROS_B.minnanb];
  }

  Cinematique_ROS_B.minnanb = jtype->size[0] * jtype->size[1];
  jtype->size[0] = 1;
  jtype->size[1] = iobj_0->Type->size[1];
  Cinema_emxEnsureCapacity_char_T(jtype, Cinematique_ROS_B.minnanb);
  Cinematique_ROS_B.loop_ub_j = iobj_0->Type->size[0] * iobj_0->Type->size[1] -
    1;
  for (Cinematique_ROS_B.minnanb = 0; Cinematique_ROS_B.minnanb <=
       Cinematique_ROS_B.loop_ub_j; Cinematique_ROS_B.minnanb++) {
    jtype->data[Cinematique_ROS_B.minnanb] = iobj_0->Type->
      data[Cinematique_ROS_B.minnanb];
  }

  for (Cinematique_ROS_B.minnanb = 0; Cinematique_ROS_B.minnanb < 8;
       Cinematique_ROS_B.minnanb++) {
    Cinematique_ROS_B.b_bj[Cinematique_ROS_B.minnanb] =
      tmp_0[Cinematique_ROS_B.minnanb];
  }

  Cinematique_ROS_B.b_bool_j = false;
  if (jtype->size[1] == 8) {
    Cinematique_ROS_B.loop_ub_j = 1;
    do {
      exitg1 = 0;
      if (Cinematique_ROS_B.loop_ub_j - 1 < 8) {
        if (jtype->data[Cinematique_ROS_B.loop_ub_j - 1] !=
            Cinematique_ROS_B.b_bj[Cinematique_ROS_B.loop_ub_j - 1]) {
          exitg1 = 1;
        } else {
          Cinematique_ROS_B.loop_ub_j++;
        }
      } else {
        Cinematique_ROS_B.b_bool_j = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (Cinematique_ROS_B.b_bool_j) {
    Cinematique_ROS_B.minnanb = 0;
  } else {
    for (Cinematique_ROS_B.minnanb = 0; Cinematique_ROS_B.minnanb < 9;
         Cinematique_ROS_B.minnanb++) {
      Cinematique_ROS_B.b_n[Cinematique_ROS_B.minnanb] =
        tmp_1[Cinematique_ROS_B.minnanb];
    }

    Cinematique_ROS_B.b_bool_j = false;
    if (jtype->size[1] == 9) {
      Cinematique_ROS_B.loop_ub_j = 1;
      do {
        exitg1 = 0;
        if (Cinematique_ROS_B.loop_ub_j - 1 < 9) {
          if (jtype->data[Cinematique_ROS_B.loop_ub_j - 1] !=
              Cinematique_ROS_B.b_n[Cinematique_ROS_B.loop_ub_j - 1]) {
            exitg1 = 1;
          } else {
            Cinematique_ROS_B.loop_ub_j++;
          }
        } else {
          Cinematique_ROS_B.b_bool_j = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (Cinematique_ROS_B.b_bool_j) {
      Cinematique_ROS_B.minnanb = 1;
    } else {
      Cinematique_ROS_B.minnanb = -1;
    }
  }

  switch (Cinematique_ROS_B.minnanb) {
   case 0:
    Cinematique_ROS_B.iv3[0] = 0;
    Cinematique_ROS_B.iv3[1] = 0;
    Cinematique_ROS_B.iv3[2] = 1;
    Cinematique_ROS_B.iv3[3] = 0;
    Cinematique_ROS_B.iv3[4] = 0;
    Cinematique_ROS_B.iv3[5] = 0;
    for (Cinematique_ROS_B.minnanb = 0; Cinematique_ROS_B.minnanb < 6;
         Cinematique_ROS_B.minnanb++) {
      Cinematique_ROS_B.msubspace_data_l[Cinematique_ROS_B.minnanb] =
        Cinematique_ROS_B.iv3[Cinematique_ROS_B.minnanb];
    }

    Cinematique_ROS_B.poslim_data_p[0] = -3.1415926535897931;
    Cinematique_ROS_B.poslim_data_p[1] = 3.1415926535897931;
    iobj_0->VelocityNumber = 1.0;
    iobj_0->PositionNumber = 1.0;
    iobj_0->JointAxisInternal[0] = 0.0;
    iobj_0->JointAxisInternal[1] = 0.0;
    iobj_0->JointAxisInternal[2] = 1.0;
    break;

   case 1:
    Cinematique_ROS_B.iv3[0] = 0;
    Cinematique_ROS_B.iv3[1] = 0;
    Cinematique_ROS_B.iv3[2] = 0;
    Cinematique_ROS_B.iv3[3] = 0;
    Cinematique_ROS_B.iv3[4] = 0;
    Cinematique_ROS_B.iv3[5] = 1;
    for (Cinematique_ROS_B.minnanb = 0; Cinematique_ROS_B.minnanb < 6;
         Cinematique_ROS_B.minnanb++) {
      Cinematique_ROS_B.msubspace_data_l[Cinematique_ROS_B.minnanb] =
        Cinematique_ROS_B.iv3[Cinematique_ROS_B.minnanb];
    }

    Cinematique_ROS_B.poslim_data_p[0] = -0.5;
    Cinematique_ROS_B.poslim_data_p[1] = 0.5;
    iobj_0->VelocityNumber = 1.0;
    iobj_0->PositionNumber = 1.0;
    iobj_0->JointAxisInternal[0] = 0.0;
    iobj_0->JointAxisInternal[1] = 0.0;
    iobj_0->JointAxisInternal[2] = 1.0;
    break;

   default:
    for (Cinematique_ROS_B.minnanb = 0; Cinematique_ROS_B.minnanb < 6;
         Cinematique_ROS_B.minnanb++) {
      Cinematique_ROS_B.msubspace_data_l[Cinematique_ROS_B.minnanb] = 0;
    }

    Cinematique_ROS_B.poslim_data_p[0] = 0.0;
    Cinematique_ROS_B.poslim_data_p[1] = 0.0;
    iobj_0->VelocityNumber = 0.0;
    iobj_0->PositionNumber = 0.0;
    iobj_0->JointAxisInternal[0] = 0.0;
    iobj_0->JointAxisInternal[1] = 0.0;
    iobj_0->JointAxisInternal[2] = 0.0;
    break;
  }

  Cinematique_ROS_B.minnanb = iobj_0->MotionSubspace->size[0] *
    iobj_0->MotionSubspace->size[1];
  iobj_0->MotionSubspace->size[0] = 6;
  iobj_0->MotionSubspace->size[1] = 1;
  Cinema_emxEnsureCapacity_real_T(iobj_0->MotionSubspace,
    Cinematique_ROS_B.minnanb);
  for (Cinematique_ROS_B.minnanb = 0; Cinematique_ROS_B.minnanb < 6;
       Cinematique_ROS_B.minnanb++) {
    iobj_0->MotionSubspace->data[Cinematique_ROS_B.minnanb] =
      Cinematique_ROS_B.msubspace_data_l[Cinematique_ROS_B.minnanb];
  }

  Cinematique_ROS_B.minnanb = iobj_0->PositionLimitsInternal->size[0] *
    iobj_0->PositionLimitsInternal->size[1];
  iobj_0->PositionLimitsInternal->size[0] = 1;
  iobj_0->PositionLimitsInternal->size[1] = 2;
  Cinema_emxEnsureCapacity_real_T(iobj_0->PositionLimitsInternal,
    Cinematique_ROS_B.minnanb);
  for (Cinematique_ROS_B.minnanb = 0; Cinematique_ROS_B.minnanb < 2;
       Cinematique_ROS_B.minnanb++) {
    iobj_0->PositionLimitsInternal->data[Cinematique_ROS_B.minnanb] =
      Cinematique_ROS_B.poslim_data_p[Cinematique_ROS_B.minnanb];
  }

  Cinematique_ROS_B.minnanb = iobj_0->HomePositionInternal->size[0];
  iobj_0->HomePositionInternal->size[0] = 1;
  Cinema_emxEnsureCapacity_real_T(iobj_0->HomePositionInternal,
    Cinematique_ROS_B.minnanb);
  for (Cinematique_ROS_B.minnanb = 0; Cinematique_ROS_B.minnanb < 1;
       Cinematique_ROS_B.minnanb++) {
    iobj_0->HomePositionInternal->data[0] = 0.0;
  }

  Cinematique_ROS_B.minnanb = jtype->size[0] * jtype->size[1];
  jtype->size[0] = 1;
  jtype->size[1] = obj->NameInternal->size[1];
  Cinema_emxEnsureCapacity_char_T(jtype, Cinematique_ROS_B.minnanb);
  Cinematique_ROS_B.loop_ub_j = obj->NameInternal->size[0] * obj->
    NameInternal->size[1] - 1;
  for (Cinematique_ROS_B.minnanb = 0; Cinematique_ROS_B.minnanb <=
       Cinematique_ROS_B.loop_ub_j; Cinematique_ROS_B.minnanb++) {
    jtype->data[Cinematique_ROS_B.minnanb] = obj->NameInternal->
      data[Cinematique_ROS_B.minnanb];
  }

  if (jtype->size[1] != 0) {
    Cinematique_ROS_B.minnanb = jtype->size[0] * jtype->size[1];
    jtype->size[0] = 1;
    jtype->size[1] = obj->NameInternal->size[1];
    Cinema_emxEnsureCapacity_char_T(jtype, Cinematique_ROS_B.minnanb);
    Cinematique_ROS_B.loop_ub_j = obj->NameInternal->size[0] * obj->
      NameInternal->size[1] - 1;
    for (Cinematique_ROS_B.minnanb = 0; Cinematique_ROS_B.minnanb <=
         Cinematique_ROS_B.loop_ub_j; Cinematique_ROS_B.minnanb++) {
      jtype->data[Cinematique_ROS_B.minnanb] = obj->NameInternal->
        data[Cinematique_ROS_B.minnanb];
    }

    if (!iobj_0->InTree) {
      Cinematique_ROS_B.minnanb = iobj_0->NameInternal->size[0] *
        iobj_0->NameInternal->size[1];
      iobj_0->NameInternal->size[0] = 1;
      iobj_0->NameInternal->size[1] = jtype->size[1];
      Cinema_emxEnsureCapacity_char_T(iobj_0->NameInternal,
        Cinematique_ROS_B.minnanb);
      Cinematique_ROS_B.loop_ub_j = jtype->size[0] * jtype->size[1] - 1;
      for (Cinematique_ROS_B.minnanb = 0; Cinematique_ROS_B.minnanb <=
           Cinematique_ROS_B.loop_ub_j; Cinematique_ROS_B.minnanb++) {
        iobj_0->NameInternal->data[Cinematique_ROS_B.minnanb] = jtype->
          data[Cinematique_ROS_B.minnanb];
      }
    }
  }

  Cinematique_ROS_emxFree_char_T(&jtype);
  Cinematique_ROS_emxInit_real_T(&obj_0, 1);
  Cinematique_ROS_B.loop_ub_j = obj->PositionLimitsInternal->size[0] *
    obj->PositionLimitsInternal->size[1];
  Cinematique_ROS_B.minnanb = iobj_0->PositionLimitsInternal->size[0] *
    iobj_0->PositionLimitsInternal->size[1];
  iobj_0->PositionLimitsInternal->size[0] = obj->PositionLimitsInternal->size[0];
  iobj_0->PositionLimitsInternal->size[1] = 2;
  Cinema_emxEnsureCapacity_real_T(iobj_0->PositionLimitsInternal,
    Cinematique_ROS_B.minnanb);
  Cinematique_ROS_B.minnanb = obj_0->size[0];
  obj_0->size[0] = Cinematique_ROS_B.loop_ub_j;
  Cinema_emxEnsureCapacity_real_T(obj_0, Cinematique_ROS_B.minnanb);
  for (Cinematique_ROS_B.minnanb = 0; Cinematique_ROS_B.minnanb <
       Cinematique_ROS_B.loop_ub_j; Cinematique_ROS_B.minnanb++) {
    obj_0->data[Cinematique_ROS_B.minnanb] = obj->PositionLimitsInternal->
      data[Cinematique_ROS_B.minnanb];
  }

  Cinematique_ROS_B.loop_ub_j = obj_0->size[0];
  for (Cinematique_ROS_B.minnanb = 0; Cinematique_ROS_B.minnanb <
       Cinematique_ROS_B.loop_ub_j; Cinematique_ROS_B.minnanb++) {
    iobj_0->PositionLimitsInternal->data[Cinematique_ROS_B.minnanb] =
      obj_0->data[Cinematique_ROS_B.minnanb];
  }

  Cinematique_ROS_emxFree_real_T(&obj_0);
  Cinematique_ROS_emxInit_real_T(&obj_1, 1);
  Cinematique_ROS_B.minnanb = obj_1->size[0];
  obj_1->size[0] = obj->HomePositionInternal->size[0];
  Cinema_emxEnsureCapacity_real_T(obj_1, Cinematique_ROS_B.minnanb);
  Cinematique_ROS_B.loop_ub_j = obj->HomePositionInternal->size[0];
  for (Cinematique_ROS_B.minnanb = 0; Cinematique_ROS_B.minnanb <
       Cinematique_ROS_B.loop_ub_j; Cinematique_ROS_B.minnanb++) {
    obj_1->data[Cinematique_ROS_B.minnanb] = obj->HomePositionInternal->
      data[Cinematique_ROS_B.minnanb];
  }

  Cinematique_ROS_B.minnanb = iobj_0->HomePositionInternal->size[0];
  iobj_0->HomePositionInternal->size[0] = obj_1->size[0];
  Cinema_emxEnsureCapacity_real_T(iobj_0->HomePositionInternal,
    Cinematique_ROS_B.minnanb);
  Cinematique_ROS_B.loop_ub_j = obj_1->size[0];
  for (Cinematique_ROS_B.minnanb = 0; Cinematique_ROS_B.minnanb <
       Cinematique_ROS_B.loop_ub_j; Cinematique_ROS_B.minnanb++) {
    iobj_0->HomePositionInternal->data[Cinematique_ROS_B.minnanb] = obj_1->
      data[Cinematique_ROS_B.minnanb];
  }

  Cinematique_ROS_emxFree_real_T(&obj_1);
  Cinematique_ROS_B.obj_idx_0 = obj->JointAxisInternal[0];
  Cinematique_ROS_B.obj_idx_1 = obj->JointAxisInternal[1];
  Cinematique_ROS_B.obj_idx_2 = obj->JointAxisInternal[2];
  iobj_0->JointAxisInternal[0] = Cinematique_ROS_B.obj_idx_0;
  iobj_0->JointAxisInternal[1] = Cinematique_ROS_B.obj_idx_1;
  iobj_0->JointAxisInternal[2] = Cinematique_ROS_B.obj_idx_2;
  Cinematique_ROS_emxInit_real_T(&obj_2, 1);
  Cinematique_ROS_B.loop_ub_j = obj->MotionSubspace->size[0] *
    obj->MotionSubspace->size[1];
  Cinematique_ROS_B.minnanb = iobj_0->MotionSubspace->size[0] *
    iobj_0->MotionSubspace->size[1];
  iobj_0->MotionSubspace->size[0] = 6;
  iobj_0->MotionSubspace->size[1] = obj->MotionSubspace->size[1];
  Cinema_emxEnsureCapacity_real_T(iobj_0->MotionSubspace,
    Cinematique_ROS_B.minnanb);
  Cinematique_ROS_B.minnanb = obj_2->size[0];
  obj_2->size[0] = Cinematique_ROS_B.loop_ub_j;
  Cinema_emxEnsureCapacity_real_T(obj_2, Cinematique_ROS_B.minnanb);
  for (Cinematique_ROS_B.minnanb = 0; Cinematique_ROS_B.minnanb <
       Cinematique_ROS_B.loop_ub_j; Cinematique_ROS_B.minnanb++) {
    obj_2->data[Cinematique_ROS_B.minnanb] = obj->MotionSubspace->
      data[Cinematique_ROS_B.minnanb];
  }

  Cinematique_ROS_B.loop_ub_j = obj_2->size[0];
  for (Cinematique_ROS_B.minnanb = 0; Cinematique_ROS_B.minnanb <
       Cinematique_ROS_B.loop_ub_j; Cinematique_ROS_B.minnanb++) {
    iobj_0->MotionSubspace->data[Cinematique_ROS_B.minnanb] = obj_2->
      data[Cinematique_ROS_B.minnanb];
  }

  Cinematique_ROS_emxFree_real_T(&obj_2);
  for (Cinematique_ROS_B.minnanb = 0; Cinematique_ROS_B.minnanb < 16;
       Cinematique_ROS_B.minnanb++) {
    Cinematique_ROS_B.obj[Cinematique_ROS_B.minnanb] =
      obj->JointToParentTransform[Cinematique_ROS_B.minnanb];
  }

  for (Cinematique_ROS_B.minnanb = 0; Cinematique_ROS_B.minnanb < 16;
       Cinematique_ROS_B.minnanb++) {
    iobj_0->JointToParentTransform[Cinematique_ROS_B.minnanb] =
      Cinematique_ROS_B.obj[Cinematique_ROS_B.minnanb];
  }

  for (Cinematique_ROS_B.minnanb = 0; Cinematique_ROS_B.minnanb < 16;
       Cinematique_ROS_B.minnanb++) {
    Cinematique_ROS_B.obj[Cinematique_ROS_B.minnanb] =
      obj->ChildToJointTransform[Cinematique_ROS_B.minnanb];
  }

  for (Cinematique_ROS_B.minnanb = 0; Cinematique_ROS_B.minnanb < 16;
       Cinematique_ROS_B.minnanb++) {
    iobj_0->ChildToJointTransform[Cinematique_ROS_B.minnanb] =
      Cinematique_ROS_B.obj[Cinematique_ROS_B.minnanb];
  }

  return newjoint;
}

static void Cinematiq_RigidBodyTree_addBody(ab_robotics_manip_internal_Ri_T *obj,
  w_robotics_manip_internal_Rig_T *bodyin, const emxArray_char_T_Cinematique_R_T
  *parentName, m_robotics_manip_internal_Col_T *iobj_0,
  c_rigidBodyJoint_Cinematique__T *iobj_1, y_robotics_manip_internal_Rig_T
  *iobj_2)
{
  static const char_T tmp_2[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const char_T tmp_1[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_0[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const int8_T tmp[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1
  };

  c_rigidBodyJoint_Cinematique__T *jnt;
  emxArray_char_T_Cinematique_R_T *bname;
  emxArray_unnamed_struct_Cinem_T *f;
  m_robotics_manip_internal_Col_T *obj_0;
  int32_T exitg1;
  Cinematique_ROS_emxInit_char_T(&bname, 2);
  Cinematique_ROS_B.b_kstr_c = bname->size[0] * bname->size[1];
  bname->size[0] = 1;
  bname->size[1] = bodyin->NameInternal->size[1];
  Cinema_emxEnsureCapacity_char_T(bname, Cinematique_ROS_B.b_kstr_c);
  Cinematique_ROS_B.loop_ub_m1 = bodyin->NameInternal->size[0] *
    bodyin->NameInternal->size[1] - 1;
  for (Cinematique_ROS_B.b_kstr_c = 0; Cinematique_ROS_B.b_kstr_c <=
       Cinematique_ROS_B.loop_ub_m1; Cinematique_ROS_B.b_kstr_c++) {
    bname->data[Cinematique_ROS_B.b_kstr_c] = bodyin->NameInternal->
      data[Cinematique_ROS_B.b_kstr_c];
  }

  RigidBodyTree_findBodyIndexByNa(obj, bname);
  Cinematique_ROS_B.pid = RigidBodyTree_findBodyIndexByNa(obj, parentName);
  Cinematique_ROS_B.b_index = obj->NumBodies + 1.0;
  Cinematique_ROS_B.b_kstr_c = bname->size[0] * bname->size[1];
  bname->size[0] = 1;
  bname->size[1] = bodyin->NameInternal->size[1];
  Cinema_emxEnsureCapacity_char_T(bname, Cinematique_ROS_B.b_kstr_c);
  Cinematique_ROS_B.loop_ub_m1 = bodyin->NameInternal->size[0] *
    bodyin->NameInternal->size[1] - 1;
  for (Cinematique_ROS_B.b_kstr_c = 0; Cinematique_ROS_B.b_kstr_c <=
       Cinematique_ROS_B.loop_ub_m1; Cinematique_ROS_B.b_kstr_c++) {
    bname->data[Cinematique_ROS_B.b_kstr_c] = bodyin->NameInternal->
      data[Cinematique_ROS_B.b_kstr_c];
  }

  Cinematique_ROS_B.b_kstr_c = iobj_2->NameInternal->size[0] *
    iobj_2->NameInternal->size[1];
  iobj_2->NameInternal->size[0] = 1;
  iobj_2->NameInternal->size[1] = bname->size[1];
  Cinema_emxEnsureCapacity_char_T(iobj_2->NameInternal,
    Cinematique_ROS_B.b_kstr_c);
  Cinematique_ROS_B.loop_ub_m1 = bname->size[0] * bname->size[1] - 1;
  for (Cinematique_ROS_B.b_kstr_c = 0; Cinematique_ROS_B.b_kstr_c <=
       Cinematique_ROS_B.loop_ub_m1; Cinematique_ROS_B.b_kstr_c++) {
    iobj_2->NameInternal->data[Cinematique_ROS_B.b_kstr_c] = bname->
      data[Cinematique_ROS_B.b_kstr_c];
  }

  iobj_2->_pobj1.InTree = false;
  for (Cinematique_ROS_B.b_kstr_c = 0; Cinematique_ROS_B.b_kstr_c < 16;
       Cinematique_ROS_B.b_kstr_c++) {
    iobj_2->_pobj1.JointToParentTransform[Cinematique_ROS_B.b_kstr_c] =
      tmp[Cinematique_ROS_B.b_kstr_c];
  }

  for (Cinematique_ROS_B.b_kstr_c = 0; Cinematique_ROS_B.b_kstr_c < 16;
       Cinematique_ROS_B.b_kstr_c++) {
    iobj_2->_pobj1.ChildToJointTransform[Cinematique_ROS_B.b_kstr_c] =
      tmp[Cinematique_ROS_B.b_kstr_c];
  }

  Cinematique_ROS_B.b_kstr_c = iobj_2->_pobj1.NameInternal->size[0] *
    iobj_2->_pobj1.NameInternal->size[1];
  iobj_2->_pobj1.NameInternal->size[0] = 1;
  iobj_2->_pobj1.NameInternal->size[1] = bname->size[1] + 4;
  Cinema_emxEnsureCapacity_char_T(iobj_2->_pobj1.NameInternal,
    Cinematique_ROS_B.b_kstr_c);
  Cinematique_ROS_B.loop_ub_m1 = bname->size[0] * bname->size[1];
  for (Cinematique_ROS_B.b_kstr_c = 0; Cinematique_ROS_B.b_kstr_c <
       Cinematique_ROS_B.loop_ub_m1; Cinematique_ROS_B.b_kstr_c++) {
    iobj_2->_pobj1.NameInternal->data[Cinematique_ROS_B.b_kstr_c] = bname->
      data[Cinematique_ROS_B.b_kstr_c];
  }

  iobj_2->_pobj1.NameInternal->data[Cinematique_ROS_B.loop_ub_m1] = '_';
  iobj_2->_pobj1.NameInternal->data[Cinematique_ROS_B.loop_ub_m1 + 1] = 'j';
  iobj_2->_pobj1.NameInternal->data[Cinematique_ROS_B.loop_ub_m1 + 2] = 'n';
  iobj_2->_pobj1.NameInternal->data[Cinematique_ROS_B.loop_ub_m1 + 3] = 't';
  Cinematique_ROS_B.b_kstr_c = iobj_2->_pobj1.Type->size[0] *
    iobj_2->_pobj1.Type->size[1];
  iobj_2->_pobj1.Type->size[0] = 1;
  iobj_2->_pobj1.Type->size[1] = 5;
  Cinema_emxEnsureCapacity_char_T(iobj_2->_pobj1.Type,
    Cinematique_ROS_B.b_kstr_c);
  for (Cinematique_ROS_B.b_kstr_c = 0; Cinematique_ROS_B.b_kstr_c < 5;
       Cinematique_ROS_B.b_kstr_c++) {
    iobj_2->_pobj1.Type->data[Cinematique_ROS_B.b_kstr_c] =
      tmp_0[Cinematique_ROS_B.b_kstr_c];
  }

  Cinematique_ROS_B.b_kstr_c = bname->size[0] * bname->size[1];
  bname->size[0] = 1;
  bname->size[1] = iobj_2->_pobj1.Type->size[1];
  Cinema_emxEnsureCapacity_char_T(bname, Cinematique_ROS_B.b_kstr_c);
  Cinematique_ROS_B.loop_ub_m1 = iobj_2->_pobj1.Type->size[0] *
    iobj_2->_pobj1.Type->size[1] - 1;
  for (Cinematique_ROS_B.b_kstr_c = 0; Cinematique_ROS_B.b_kstr_c <=
       Cinematique_ROS_B.loop_ub_m1; Cinematique_ROS_B.b_kstr_c++) {
    bname->data[Cinematique_ROS_B.b_kstr_c] = iobj_2->_pobj1.Type->
      data[Cinematique_ROS_B.b_kstr_c];
  }

  for (Cinematique_ROS_B.b_kstr_c = 0; Cinematique_ROS_B.b_kstr_c < 8;
       Cinematique_ROS_B.b_kstr_c++) {
    Cinematique_ROS_B.b_e[Cinematique_ROS_B.b_kstr_c] =
      tmp_1[Cinematique_ROS_B.b_kstr_c];
  }

  Cinematique_ROS_B.b_bool_d = false;
  if (bname->size[1] == 8) {
    Cinematique_ROS_B.b_kstr_c = 1;
    do {
      exitg1 = 0;
      if (Cinematique_ROS_B.b_kstr_c - 1 < 8) {
        if (bname->data[Cinematique_ROS_B.b_kstr_c - 1] !=
            Cinematique_ROS_B.b_e[Cinematique_ROS_B.b_kstr_c - 1]) {
          exitg1 = 1;
        } else {
          Cinematique_ROS_B.b_kstr_c++;
        }
      } else {
        Cinematique_ROS_B.b_bool_d = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (Cinematique_ROS_B.b_bool_d) {
    Cinematique_ROS_B.b_kstr_c = 0;
  } else {
    for (Cinematique_ROS_B.b_kstr_c = 0; Cinematique_ROS_B.b_kstr_c < 9;
         Cinematique_ROS_B.b_kstr_c++) {
      Cinematique_ROS_B.b_b[Cinematique_ROS_B.b_kstr_c] =
        tmp_2[Cinematique_ROS_B.b_kstr_c];
    }

    Cinematique_ROS_B.b_bool_d = false;
    if (bname->size[1] == 9) {
      Cinematique_ROS_B.b_kstr_c = 1;
      do {
        exitg1 = 0;
        if (Cinematique_ROS_B.b_kstr_c - 1 < 9) {
          if (bname->data[Cinematique_ROS_B.b_kstr_c - 1] !=
              Cinematique_ROS_B.b_b[Cinematique_ROS_B.b_kstr_c - 1]) {
            exitg1 = 1;
          } else {
            Cinematique_ROS_B.b_kstr_c++;
          }
        } else {
          Cinematique_ROS_B.b_bool_d = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (Cinematique_ROS_B.b_bool_d) {
      Cinematique_ROS_B.b_kstr_c = 1;
    } else {
      Cinematique_ROS_B.b_kstr_c = -1;
    }
  }

  switch (Cinematique_ROS_B.b_kstr_c) {
   case 0:
    Cinematique_ROS_B.iv2[0] = 0;
    Cinematique_ROS_B.iv2[1] = 0;
    Cinematique_ROS_B.iv2[2] = 1;
    Cinematique_ROS_B.iv2[3] = 0;
    Cinematique_ROS_B.iv2[4] = 0;
    Cinematique_ROS_B.iv2[5] = 0;
    for (Cinematique_ROS_B.b_kstr_c = 0; Cinematique_ROS_B.b_kstr_c < 6;
         Cinematique_ROS_B.b_kstr_c++) {
      Cinematique_ROS_B.msubspace_data_p[Cinematique_ROS_B.b_kstr_c] =
        Cinematique_ROS_B.iv2[Cinematique_ROS_B.b_kstr_c];
    }

    Cinematique_ROS_B.poslim_data_b[0] = -3.1415926535897931;
    Cinematique_ROS_B.poslim_data_b[1] = 3.1415926535897931;
    iobj_2->_pobj1.VelocityNumber = 1.0;
    iobj_2->_pobj1.PositionNumber = 1.0;
    iobj_2->_pobj1.JointAxisInternal[0] = 0.0;
    iobj_2->_pobj1.JointAxisInternal[1] = 0.0;
    iobj_2->_pobj1.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    Cinematique_ROS_B.iv2[0] = 0;
    Cinematique_ROS_B.iv2[1] = 0;
    Cinematique_ROS_B.iv2[2] = 0;
    Cinematique_ROS_B.iv2[3] = 0;
    Cinematique_ROS_B.iv2[4] = 0;
    Cinematique_ROS_B.iv2[5] = 1;
    for (Cinematique_ROS_B.b_kstr_c = 0; Cinematique_ROS_B.b_kstr_c < 6;
         Cinematique_ROS_B.b_kstr_c++) {
      Cinematique_ROS_B.msubspace_data_p[Cinematique_ROS_B.b_kstr_c] =
        Cinematique_ROS_B.iv2[Cinematique_ROS_B.b_kstr_c];
    }

    Cinematique_ROS_B.poslim_data_b[0] = -0.5;
    Cinematique_ROS_B.poslim_data_b[1] = 0.5;
    iobj_2->_pobj1.VelocityNumber = 1.0;
    iobj_2->_pobj1.PositionNumber = 1.0;
    iobj_2->_pobj1.JointAxisInternal[0] = 0.0;
    iobj_2->_pobj1.JointAxisInternal[1] = 0.0;
    iobj_2->_pobj1.JointAxisInternal[2] = 1.0;
    break;

   default:
    for (Cinematique_ROS_B.b_kstr_c = 0; Cinematique_ROS_B.b_kstr_c < 6;
         Cinematique_ROS_B.b_kstr_c++) {
      Cinematique_ROS_B.msubspace_data_p[Cinematique_ROS_B.b_kstr_c] = 0;
    }

    Cinematique_ROS_B.poslim_data_b[0] = 0.0;
    Cinematique_ROS_B.poslim_data_b[1] = 0.0;
    iobj_2->_pobj1.VelocityNumber = 0.0;
    iobj_2->_pobj1.PositionNumber = 0.0;
    iobj_2->_pobj1.JointAxisInternal[0] = 0.0;
    iobj_2->_pobj1.JointAxisInternal[1] = 0.0;
    iobj_2->_pobj1.JointAxisInternal[2] = 0.0;
    break;
  }

  Cinematique_ROS_B.b_kstr_c = iobj_2->_pobj1.MotionSubspace->size[0] *
    iobj_2->_pobj1.MotionSubspace->size[1];
  iobj_2->_pobj1.MotionSubspace->size[0] = 6;
  iobj_2->_pobj1.MotionSubspace->size[1] = 1;
  Cinema_emxEnsureCapacity_real_T(iobj_2->_pobj1.MotionSubspace,
    Cinematique_ROS_B.b_kstr_c);
  for (Cinematique_ROS_B.b_kstr_c = 0; Cinematique_ROS_B.b_kstr_c < 6;
       Cinematique_ROS_B.b_kstr_c++) {
    iobj_2->_pobj1.MotionSubspace->data[Cinematique_ROS_B.b_kstr_c] =
      Cinematique_ROS_B.msubspace_data_p[Cinematique_ROS_B.b_kstr_c];
  }

  Cinematique_ROS_B.b_kstr_c = iobj_2->_pobj1.PositionLimitsInternal->size[0] *
    iobj_2->_pobj1.PositionLimitsInternal->size[1];
  iobj_2->_pobj1.PositionLimitsInternal->size[0] = 1;
  iobj_2->_pobj1.PositionLimitsInternal->size[1] = 2;
  Cinema_emxEnsureCapacity_real_T(iobj_2->_pobj1.PositionLimitsInternal,
    Cinematique_ROS_B.b_kstr_c);
  for (Cinematique_ROS_B.b_kstr_c = 0; Cinematique_ROS_B.b_kstr_c < 2;
       Cinematique_ROS_B.b_kstr_c++) {
    iobj_2->_pobj1.PositionLimitsInternal->data[Cinematique_ROS_B.b_kstr_c] =
      Cinematique_ROS_B.poslim_data_b[Cinematique_ROS_B.b_kstr_c];
  }

  Cinematique_ROS_B.b_kstr_c = iobj_2->_pobj1.HomePositionInternal->size[0];
  iobj_2->_pobj1.HomePositionInternal->size[0] = 1;
  Cinema_emxEnsureCapacity_real_T(iobj_2->_pobj1.HomePositionInternal,
    Cinematique_ROS_B.b_kstr_c);
  for (Cinematique_ROS_B.b_kstr_c = 0; Cinematique_ROS_B.b_kstr_c < 1;
       Cinematique_ROS_B.b_kstr_c++) {
    iobj_2->_pobj1.HomePositionInternal->data[0] = 0.0;
  }

  Cinemati_emxInit_unnamed_struct(&f, 2);
  iobj_2->JointInternal = &iobj_2->_pobj1;
  iobj_2->Index = -1.0;
  iobj_2->ParentIndex = -1.0;
  obj_0 = &iobj_2->_pobj0;
  iobj_2->_pobj0.Size = 0.0;
  iobj_2->_pobj0.MaxElements = 0.0;
  Cinematique_ROS_B.b_kstr_c = f->size[0] * f->size[1];
  f->size[0] = 1;
  f->size[1] = static_cast<int32_T>(iobj_2->_pobj0.MaxElements);
  emxEnsureCapacity_unnamed_struc(f, Cinematique_ROS_B.b_kstr_c);
  Cinematique_ROS_B.b_kstr_c = iobj_2->_pobj0.CollisionGeometries->size[0] *
    iobj_2->_pobj0.CollisionGeometries->size[1];
  iobj_2->_pobj0.CollisionGeometries->size[0] = f->size[0];
  iobj_2->_pobj0.CollisionGeometries->size[1] = f->size[1];
  emxEnsureCapacity_unnamed_struc(iobj_2->_pobj0.CollisionGeometries,
    Cinematique_ROS_B.b_kstr_c);
  collisioncodegen_makeBox(0.0, 0.0, 0.0);
  Cinematique_ROS_B.c_o = iobj_2->_pobj0.MaxElements;
  Cinematique_ROS_B.loop_ub_m1 = static_cast<int32_T>(Cinematique_ROS_B.c_o) - 1;
  Cinemati_emxFree_unnamed_struct(&f);
  for (Cinematique_ROS_B.b_kstr_c = 0; Cinematique_ROS_B.b_kstr_c <=
       Cinematique_ROS_B.loop_ub_m1; Cinematique_ROS_B.b_kstr_c++) {
    obj_0->CollisionGeometries->data[Cinematique_ROS_B.b_kstr_c] =
      &iobj_2->_pobj0._pobj0;
  }

  iobj_2->CollisionsInternal = &iobj_2->_pobj0;
  iobj_2->JointInternal = Cinematique_rigidBodyJoint_copy(&bodyin->JointInternal,
    iobj_1);
  iobj_2->CollisionsInternal = Cinematique_R_CollisionSet_copy
    (&bodyin->CollisionsInternal, iobj_0);
  obj->Bodies[static_cast<int32_T>(Cinematique_ROS_B.b_index) - 1] = iobj_2;
  iobj_2->Index = Cinematique_ROS_B.b_index;
  iobj_2->ParentIndex = Cinematique_ROS_B.pid;
  iobj_2->JointInternal->InTree = true;
  obj->NumBodies++;
  jnt = iobj_2->JointInternal;
  Cinematique_ROS_B.b_kstr_c = bname->size[0] * bname->size[1];
  bname->size[0] = 1;
  bname->size[1] = jnt->Type->size[1];
  Cinema_emxEnsureCapacity_char_T(bname, Cinematique_ROS_B.b_kstr_c);
  Cinematique_ROS_B.loop_ub_m1 = jnt->Type->size[0] * jnt->Type->size[1] - 1;
  for (Cinematique_ROS_B.b_kstr_c = 0; Cinematique_ROS_B.b_kstr_c <=
       Cinematique_ROS_B.loop_ub_m1; Cinematique_ROS_B.b_kstr_c++) {
    bname->data[Cinematique_ROS_B.b_kstr_c] = jnt->Type->
      data[Cinematique_ROS_B.b_kstr_c];
  }

  for (Cinematique_ROS_B.b_kstr_c = 0; Cinematique_ROS_B.b_kstr_c < 5;
       Cinematique_ROS_B.b_kstr_c++) {
    Cinematique_ROS_B.b_c[Cinematique_ROS_B.b_kstr_c] =
      tmp_0[Cinematique_ROS_B.b_kstr_c];
  }

  Cinematique_ROS_B.b_bool_d = false;
  if (bname->size[1] == 5) {
    Cinematique_ROS_B.b_kstr_c = 1;
    do {
      exitg1 = 0;
      if (Cinematique_ROS_B.b_kstr_c - 1 < 5) {
        if (bname->data[Cinematique_ROS_B.b_kstr_c - 1] !=
            Cinematique_ROS_B.b_c[Cinematique_ROS_B.b_kstr_c - 1]) {
          exitg1 = 1;
        } else {
          Cinematique_ROS_B.b_kstr_c++;
        }
      } else {
        Cinematique_ROS_B.b_bool_d = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  Cinematique_ROS_emxFree_char_T(&bname);
  if (!Cinematique_ROS_B.b_bool_d) {
    obj->NumNonFixedBodies++;
    jnt = iobj_2->JointInternal;
    Cinematique_ROS_B.b_kstr_c = static_cast<int32_T>(iobj_2->Index) - 1;
    obj->PositionDoFMap[Cinematique_ROS_B.b_kstr_c] = obj->PositionNumber + 1.0;
    obj->PositionDoFMap[Cinematique_ROS_B.b_kstr_c + 5] = obj->PositionNumber +
      jnt->PositionNumber;
    jnt = iobj_2->JointInternal;
    Cinematique_ROS_B.b_kstr_c = static_cast<int32_T>(iobj_2->Index) - 1;
    obj->VelocityDoFMap[Cinematique_ROS_B.b_kstr_c] = obj->VelocityNumber + 1.0;
    obj->VelocityDoFMap[Cinematique_ROS_B.b_kstr_c + 5] = obj->VelocityNumber +
      jnt->VelocityNumber;
  } else {
    Cinematique_ROS_B.b_kstr_c = static_cast<int32_T>(iobj_2->Index);
    obj->PositionDoFMap[Cinematique_ROS_B.b_kstr_c - 1] = 0.0;
    obj->PositionDoFMap[Cinematique_ROS_B.b_kstr_c + 4] = -1.0;
    Cinematique_ROS_B.b_kstr_c = static_cast<int32_T>(iobj_2->Index);
    obj->VelocityDoFMap[Cinematique_ROS_B.b_kstr_c - 1] = 0.0;
    obj->VelocityDoFMap[Cinematique_ROS_B.b_kstr_c + 4] = -1.0;
  }

  jnt = iobj_2->JointInternal;
  obj->PositionNumber += jnt->PositionNumber;
  jnt = iobj_2->JointInternal;
  obj->VelocityNumber += jnt->VelocityNumber;
}

static void inverseKinematics_set_SolverAlg(b_inverseKinematics_Cinematiq_T *obj,
  h_robotics_core_internal_Damp_T *iobj_0)
{
  static const sdAmwXbnJnEmimT0NaJRtAD_Cinem_T tmp_0 = { 0.0,/* tv_sec */
    0.0                                /* tv_nsec */
  };

  static const char_T tmp[22] = { 'B', 'F', 'G', 'S', 'G', 'r', 'a', 'd', 'i',
    'e', 'n', 't', 'P', 'r', 'o', 'j', 'e', 'c', 't', 'i', 'o', 'n' };

  int32_T i;
  iobj_0->MaxNumIteration = 1500.0;
  iobj_0->MaxTime = 10.0;
  iobj_0->GradientTolerance = 1.0E-7;
  iobj_0->SolutionTolerance = 1.0E-6;
  iobj_0->ArmijoRuleBeta = 0.4;
  iobj_0->ArmijoRuleSigma = 1.0E-5;
  iobj_0->ConstraintsOn = true;
  iobj_0->RandomRestart = true;
  iobj_0->StepTolerance = 1.0E-14;
  for (i = 0; i < 22; i++) {
    iobj_0->Name[i] = tmp[i];
  }

  iobj_0->ConstraintMatrix->size[0] = 0;
  iobj_0->ConstraintMatrix->size[1] = 0;
  iobj_0->ConstraintBound->size[0] = 0;
  iobj_0->TimeObj.StartTime = tmp_0;
  iobj_0->TimeObjInternal.StartTime = tmp_0;
  obj->Solver = iobj_0;
}

static void Cinematique_RO_SystemCore_setup(robotics_slmanip_internal_blo_T *obj)
{
  static const char_T tmp_3[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const char_T tmp_0[8] = { 'b', 'a', 's', 'e', '_', 'j', 'n', 't' };

  static const char_T tmp_2[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_1[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const int8_T tmp[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1
  };

  ab_robotics_manip_internal_Ri_T *iobj_0;
  b_inverseKinematics_Cinematiq_T *obj_0;
  c_rigidBodyJoint_Cinematique__T *iobj_2;
  emxArray_char_T_Cinematique_R_T *bname;
  emxArray_char_T_Cinematique_R_T *switch_expression;
  emxArray_unnamed_struct_Cinem_T *f;
  h_robotics_core_internal_Damp_T *iobj_4;
  m_robotics_manip_internal_Col_T *iobj_1;
  w_robotics_manip_internal_Rig_T *body;
  w_robotics_manip_internal_Rig_T *parent;
  y_robotics_manip_internal_Rig_T *iobj_3;
  y_robotics_manip_internal_Rig_T *obj_1;
  int32_T exitg1;
  boolean_T exitg2;
  obj->isInitialized = 1;
  Cin_RigidBodyTree_RigidBodyTree(&obj->TreeInternal);
  obj_0 = &obj->IKInternal;
  obj->IKInternal.isInitialized = 0;
  iobj_0 = &obj->IKInternal._pobj1;
  iobj_1 = &obj->IKInternal._pobj2[0];
  iobj_2 = &obj->IKInternal._pobj3[0];
  iobj_3 = &obj->IKInternal._pobj4[0];
  iobj_4 = &obj->IKInternal._pobj5;
  obj_1 = &iobj_0->Base;
  Cinematique_ROS_B.d_a = iobj_0->Base.NameInternal->size[0] *
    iobj_0->Base.NameInternal->size[1];
  iobj_0->Base.NameInternal->size[0] = 1;
  iobj_0->Base.NameInternal->size[1] = 4;
  Cinema_emxEnsureCapacity_char_T(iobj_0->Base.NameInternal,
    Cinematique_ROS_B.d_a);
  iobj_0->Base.NameInternal->data[0] = 'b';
  iobj_0->Base.NameInternal->data[1] = 'a';
  iobj_0->Base.NameInternal->data[2] = 's';
  iobj_0->Base.NameInternal->data[3] = 'e';
  iobj_0->Base._pobj1.InTree = false;
  for (Cinematique_ROS_B.d_a = 0; Cinematique_ROS_B.d_a < 16;
       Cinematique_ROS_B.d_a++) {
    obj_1->_pobj1.JointToParentTransform[Cinematique_ROS_B.d_a] =
      tmp[Cinematique_ROS_B.d_a];
  }

  for (Cinematique_ROS_B.d_a = 0; Cinematique_ROS_B.d_a < 16;
       Cinematique_ROS_B.d_a++) {
    obj_1->_pobj1.ChildToJointTransform[Cinematique_ROS_B.d_a] =
      tmp[Cinematique_ROS_B.d_a];
  }

  Cinematique_ROS_B.d_a = iobj_0->Base._pobj1.NameInternal->size[0] *
    iobj_0->Base._pobj1.NameInternal->size[1];
  iobj_0->Base._pobj1.NameInternal->size[0] = 1;
  iobj_0->Base._pobj1.NameInternal->size[1] = 8;
  Cinema_emxEnsureCapacity_char_T(iobj_0->Base._pobj1.NameInternal,
    Cinematique_ROS_B.d_a);
  for (Cinematique_ROS_B.d_a = 0; Cinematique_ROS_B.d_a < 8;
       Cinematique_ROS_B.d_a++) {
    obj_1->_pobj1.NameInternal->data[Cinematique_ROS_B.d_a] =
      tmp_0[Cinematique_ROS_B.d_a];
  }

  Cinematique_ROS_B.d_a = iobj_0->Base._pobj1.Type->size[0] *
    iobj_0->Base._pobj1.Type->size[1];
  iobj_0->Base._pobj1.Type->size[0] = 1;
  iobj_0->Base._pobj1.Type->size[1] = 5;
  Cinema_emxEnsureCapacity_char_T(iobj_0->Base._pobj1.Type,
    Cinematique_ROS_B.d_a);
  for (Cinematique_ROS_B.d_a = 0; Cinematique_ROS_B.d_a < 5;
       Cinematique_ROS_B.d_a++) {
    obj_1->_pobj1.Type->data[Cinematique_ROS_B.d_a] =
      tmp_1[Cinematique_ROS_B.d_a];
  }

  Cinematique_ROS_emxInit_char_T(&switch_expression, 2);
  Cinematique_ROS_B.d_a = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = iobj_0->Base._pobj1.Type->size[1];
  Cinema_emxEnsureCapacity_char_T(switch_expression, Cinematique_ROS_B.d_a);
  Cinematique_ROS_B.loop_ub_e = iobj_0->Base._pobj1.Type->size[0] *
    iobj_0->Base._pobj1.Type->size[1] - 1;
  for (Cinematique_ROS_B.d_a = 0; Cinematique_ROS_B.d_a <=
       Cinematique_ROS_B.loop_ub_e; Cinematique_ROS_B.d_a++) {
    switch_expression->data[Cinematique_ROS_B.d_a] = obj_1->_pobj1.Type->
      data[Cinematique_ROS_B.d_a];
  }

  for (Cinematique_ROS_B.d_a = 0; Cinematique_ROS_B.d_a < 8;
       Cinematique_ROS_B.d_a++) {
    Cinematique_ROS_B.b_h[Cinematique_ROS_B.d_a] = tmp_2[Cinematique_ROS_B.d_a];
  }

  Cinematique_ROS_B.b_bool_g = false;
  if (switch_expression->size[1] == 8) {
    Cinematique_ROS_B.b_kstr_g = 1;
    do {
      exitg1 = 0;
      if (Cinematique_ROS_B.b_kstr_g - 1 < 8) {
        if (switch_expression->data[Cinematique_ROS_B.b_kstr_g - 1] !=
            Cinematique_ROS_B.b_h[Cinematique_ROS_B.b_kstr_g - 1]) {
          exitg1 = 1;
        } else {
          Cinematique_ROS_B.b_kstr_g++;
        }
      } else {
        Cinematique_ROS_B.b_bool_g = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (Cinematique_ROS_B.b_bool_g) {
    Cinematique_ROS_B.b_kstr_g = 0;
  } else {
    for (Cinematique_ROS_B.d_a = 0; Cinematique_ROS_B.d_a < 9;
         Cinematique_ROS_B.d_a++) {
      Cinematique_ROS_B.b_d[Cinematique_ROS_B.d_a] = tmp_3[Cinematique_ROS_B.d_a];
    }

    Cinematique_ROS_B.b_bool_g = false;
    if (switch_expression->size[1] == 9) {
      Cinematique_ROS_B.b_kstr_g = 1;
      do {
        exitg1 = 0;
        if (Cinematique_ROS_B.b_kstr_g - 1 < 9) {
          if (switch_expression->data[Cinematique_ROS_B.b_kstr_g - 1] !=
              Cinematique_ROS_B.b_d[Cinematique_ROS_B.b_kstr_g - 1]) {
            exitg1 = 1;
          } else {
            Cinematique_ROS_B.b_kstr_g++;
          }
        } else {
          Cinematique_ROS_B.b_bool_g = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (Cinematique_ROS_B.b_bool_g) {
      Cinematique_ROS_B.b_kstr_g = 1;
    } else {
      Cinematique_ROS_B.b_kstr_g = -1;
    }
  }

  switch (Cinematique_ROS_B.b_kstr_g) {
   case 0:
    Cinematique_ROS_B.iv[0] = 0;
    Cinematique_ROS_B.iv[1] = 0;
    Cinematique_ROS_B.iv[2] = 1;
    Cinematique_ROS_B.iv[3] = 0;
    Cinematique_ROS_B.iv[4] = 0;
    Cinematique_ROS_B.iv[5] = 0;
    for (Cinematique_ROS_B.d_a = 0; Cinematique_ROS_B.d_a < 6;
         Cinematique_ROS_B.d_a++) {
      Cinematique_ROS_B.msubspace_data[Cinematique_ROS_B.d_a] =
        Cinematique_ROS_B.iv[Cinematique_ROS_B.d_a];
    }

    Cinematique_ROS_B.poslim_data[0] = -3.1415926535897931;
    Cinematique_ROS_B.poslim_data[1] = 3.1415926535897931;
    iobj_0->Base._pobj1.VelocityNumber = 1.0;
    iobj_0->Base._pobj1.PositionNumber = 1.0;
    iobj_0->Base._pobj1.JointAxisInternal[0] = 0.0;
    iobj_0->Base._pobj1.JointAxisInternal[1] = 0.0;
    iobj_0->Base._pobj1.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    Cinematique_ROS_B.iv[0] = 0;
    Cinematique_ROS_B.iv[1] = 0;
    Cinematique_ROS_B.iv[2] = 0;
    Cinematique_ROS_B.iv[3] = 0;
    Cinematique_ROS_B.iv[4] = 0;
    Cinematique_ROS_B.iv[5] = 1;
    for (Cinematique_ROS_B.d_a = 0; Cinematique_ROS_B.d_a < 6;
         Cinematique_ROS_B.d_a++) {
      Cinematique_ROS_B.msubspace_data[Cinematique_ROS_B.d_a] =
        Cinematique_ROS_B.iv[Cinematique_ROS_B.d_a];
    }

    Cinematique_ROS_B.poslim_data[0] = -0.5;
    Cinematique_ROS_B.poslim_data[1] = 0.5;
    iobj_0->Base._pobj1.VelocityNumber = 1.0;
    iobj_0->Base._pobj1.PositionNumber = 1.0;
    iobj_0->Base._pobj1.JointAxisInternal[0] = 0.0;
    iobj_0->Base._pobj1.JointAxisInternal[1] = 0.0;
    iobj_0->Base._pobj1.JointAxisInternal[2] = 1.0;
    break;

   default:
    for (Cinematique_ROS_B.d_a = 0; Cinematique_ROS_B.d_a < 6;
         Cinematique_ROS_B.d_a++) {
      Cinematique_ROS_B.msubspace_data[Cinematique_ROS_B.d_a] = 0;
    }

    Cinematique_ROS_B.poslim_data[0] = 0.0;
    Cinematique_ROS_B.poslim_data[1] = 0.0;
    iobj_0->Base._pobj1.VelocityNumber = 0.0;
    iobj_0->Base._pobj1.PositionNumber = 0.0;
    iobj_0->Base._pobj1.JointAxisInternal[0] = 0.0;
    iobj_0->Base._pobj1.JointAxisInternal[1] = 0.0;
    iobj_0->Base._pobj1.JointAxisInternal[2] = 0.0;
    break;
  }

  Cinematique_ROS_B.d_a = iobj_0->Base._pobj1.MotionSubspace->size[0] *
    iobj_0->Base._pobj1.MotionSubspace->size[1];
  iobj_0->Base._pobj1.MotionSubspace->size[0] = 6;
  iobj_0->Base._pobj1.MotionSubspace->size[1] = 1;
  Cinema_emxEnsureCapacity_real_T(iobj_0->Base._pobj1.MotionSubspace,
    Cinematique_ROS_B.d_a);
  for (Cinematique_ROS_B.d_a = 0; Cinematique_ROS_B.d_a < 6;
       Cinematique_ROS_B.d_a++) {
    obj_1->_pobj1.MotionSubspace->data[Cinematique_ROS_B.d_a] =
      Cinematique_ROS_B.msubspace_data[Cinematique_ROS_B.d_a];
  }

  Cinematique_ROS_B.d_a = iobj_0->Base._pobj1.PositionLimitsInternal->size[0] *
    iobj_0->Base._pobj1.PositionLimitsInternal->size[1];
  iobj_0->Base._pobj1.PositionLimitsInternal->size[0] = 1;
  iobj_0->Base._pobj1.PositionLimitsInternal->size[1] = 2;
  Cinema_emxEnsureCapacity_real_T(iobj_0->Base._pobj1.PositionLimitsInternal,
    Cinematique_ROS_B.d_a);
  for (Cinematique_ROS_B.d_a = 0; Cinematique_ROS_B.d_a < 2;
       Cinematique_ROS_B.d_a++) {
    obj_1->_pobj1.PositionLimitsInternal->data[Cinematique_ROS_B.d_a] =
      Cinematique_ROS_B.poslim_data[Cinematique_ROS_B.d_a];
  }

  Cinematique_ROS_B.d_a = iobj_0->Base._pobj1.HomePositionInternal->size[0];
  iobj_0->Base._pobj1.HomePositionInternal->size[0] = 1;
  Cinema_emxEnsureCapacity_real_T(iobj_0->Base._pobj1.HomePositionInternal,
    Cinematique_ROS_B.d_a);
  for (Cinematique_ROS_B.d_a = 0; Cinematique_ROS_B.d_a < 1;
       Cinematique_ROS_B.d_a++) {
    obj_1->_pobj1.HomePositionInternal->data[0] = 0.0;
  }

  Cinemati_emxInit_unnamed_struct(&f, 2);
  iobj_0->Base.JointInternal = &iobj_0->Base._pobj1;
  iobj_0->Base.Index = -1.0;
  iobj_0->Base.ParentIndex = -1.0;
  obj_1->_pobj0.Size = 0.0;
  obj_1->_pobj0.MaxElements = 0.0;
  Cinematique_ROS_B.d_a = f->size[0] * f->size[1];
  f->size[0] = 1;
  f->size[1] = static_cast<int32_T>(obj_1->_pobj0.MaxElements);
  emxEnsureCapacity_unnamed_struc(f, Cinematique_ROS_B.d_a);
  Cinematique_ROS_B.d_a = obj_1->_pobj0.CollisionGeometries->size[0] *
    obj_1->_pobj0.CollisionGeometries->size[1];
  obj_1->_pobj0.CollisionGeometries->size[0] = f->size[0];
  obj_1->_pobj0.CollisionGeometries->size[1] = f->size[1];
  emxEnsureCapacity_unnamed_struc(obj_1->_pobj0.CollisionGeometries,
    Cinematique_ROS_B.d_a);
  collisioncodegen_makeBox(0.0, 0.0, 0.0);
  Cinematique_ROS_B.bid_n = obj_1->_pobj0.MaxElements;
  Cinematique_ROS_B.d_a = static_cast<int32_T>(Cinematique_ROS_B.bid_n) - 1;
  Cinemati_emxFree_unnamed_struct(&f);
  for (Cinematique_ROS_B.b_kstr_g = 0; Cinematique_ROS_B.b_kstr_g <=
       Cinematique_ROS_B.d_a; Cinematique_ROS_B.b_kstr_g++) {
    iobj_0->Base._pobj0.CollisionGeometries->data[Cinematique_ROS_B.b_kstr_g] =
      &obj_1->_pobj0._pobj0;
  }

  iobj_0->Base.CollisionsInternal = &obj_1->_pobj0;
  iobj_0->Base.Index = 0.0;
  Cinematique_ROS_rand(Cinematique_ROS_B.unusedExpr);
  Ci_RigidBodyTree_clearAllBodies(iobj_0, &iobj_0->_pobj0[0]);
  Cinematique_ROS_B.d_a = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->TreeInternal.Base.NameInternal->size[1];
  Cinema_emxEnsureCapacity_char_T(switch_expression, Cinematique_ROS_B.d_a);
  Cinematique_ROS_B.loop_ub_e = obj->TreeInternal.Base.NameInternal->size[0] *
    obj->TreeInternal.Base.NameInternal->size[1] - 1;
  for (Cinematique_ROS_B.d_a = 0; Cinematique_ROS_B.d_a <=
       Cinematique_ROS_B.loop_ub_e; Cinematique_ROS_B.d_a++) {
    switch_expression->data[Cinematique_ROS_B.d_a] =
      obj->TreeInternal.Base.NameInternal->data[Cinematique_ROS_B.d_a];
  }

  Cinematique_ROS_emxInit_char_T(&bname, 2);
  Cinematique_ROS_B.bid_n = -1.0;
  Cinematique_ROS_B.d_a = bname->size[0] * bname->size[1];
  bname->size[0] = 1;
  bname->size[1] = iobj_0->Base.NameInternal->size[1];
  Cinema_emxEnsureCapacity_char_T(bname, Cinematique_ROS_B.d_a);
  Cinematique_ROS_B.loop_ub_e = iobj_0->Base.NameInternal->size[0] *
    iobj_0->Base.NameInternal->size[1] - 1;
  for (Cinematique_ROS_B.d_a = 0; Cinematique_ROS_B.d_a <=
       Cinematique_ROS_B.loop_ub_e; Cinematique_ROS_B.d_a++) {
    bname->data[Cinematique_ROS_B.d_a] = iobj_0->Base.NameInternal->
      data[Cinematique_ROS_B.d_a];
  }

  if (Cinematique_ROS_strcmp(bname, switch_expression)) {
    Cinematique_ROS_B.bid_n = 0.0;
  } else {
    Cinematique_ROS_B.b_m = iobj_0->NumBodies;
    Cinematique_ROS_B.b_kstr_g = 0;
    exitg2 = false;
    while ((!exitg2) && (Cinematique_ROS_B.b_kstr_g <= static_cast<int32_T>
                         (Cinematique_ROS_B.b_m) - 1)) {
      obj_1 = iobj_0->Bodies[Cinematique_ROS_B.b_kstr_g];
      Cinematique_ROS_B.d_a = bname->size[0] * bname->size[1];
      bname->size[0] = 1;
      bname->size[1] = obj_1->NameInternal->size[1];
      Cinema_emxEnsureCapacity_char_T(bname, Cinematique_ROS_B.d_a);
      Cinematique_ROS_B.loop_ub_e = obj_1->NameInternal->size[0] *
        obj_1->NameInternal->size[1] - 1;
      for (Cinematique_ROS_B.d_a = 0; Cinematique_ROS_B.d_a <=
           Cinematique_ROS_B.loop_ub_e; Cinematique_ROS_B.d_a++) {
        bname->data[Cinematique_ROS_B.d_a] = obj_1->NameInternal->
          data[Cinematique_ROS_B.d_a];
      }

      if (Cinematique_ROS_strcmp(bname, switch_expression)) {
        Cinematique_ROS_B.bid_n = static_cast<real_T>(Cinematique_ROS_B.b_kstr_g)
          + 1.0;
        exitg2 = true;
      } else {
        Cinematique_ROS_B.b_kstr_g++;
      }
    }
  }

  if ((!(Cinematique_ROS_B.bid_n == 0.0)) && (Cinematique_ROS_B.bid_n < 0.0)) {
    Cinematique_ROS_B.d_a = iobj_0->Base.NameInternal->size[0] *
      iobj_0->Base.NameInternal->size[1];
    iobj_0->Base.NameInternal->size[0] = 1;
    iobj_0->Base.NameInternal->size[1] = switch_expression->size[1];
    Cinema_emxEnsureCapacity_char_T(iobj_0->Base.NameInternal,
      Cinematique_ROS_B.d_a);
    Cinematique_ROS_B.loop_ub_e = switch_expression->size[0] *
      switch_expression->size[1] - 1;
    for (Cinematique_ROS_B.d_a = 0; Cinematique_ROS_B.d_a <=
         Cinematique_ROS_B.loop_ub_e; Cinematique_ROS_B.d_a++) {
      iobj_0->Base.NameInternal->data[Cinematique_ROS_B.d_a] =
        switch_expression->data[Cinematique_ROS_B.d_a];
    }
  }

  Cinematique_ROS_emxFree_char_T(&switch_expression);
  iobj_0->Base.CollisionsInternal = Cinematique_R_CollisionSet_copy
    (&obj->TreeInternal.Base.CollisionsInternal, &(&(&(&(&iobj_1[0])[0])[0])[0])
     [0]);
  if (1.0 <= obj->TreeInternal.NumBodies) {
    body = obj->TreeInternal.Bodies[0];
    Cinematique_ROS_B.bid_n = body->ParentIndex;
    if (Cinematique_ROS_B.bid_n > 0.0) {
      parent = obj->TreeInternal.Bodies[static_cast<int32_T>
        (Cinematique_ROS_B.bid_n) - 1];
    } else {
      parent = &obj->TreeInternal.Base;
    }

    Cinematique_ROS_B.d_a = bname->size[0] * bname->size[1];
    bname->size[0] = 1;
    bname->size[1] = parent->NameInternal->size[1];
    Cinema_emxEnsureCapacity_char_T(bname, Cinematique_ROS_B.d_a);
    Cinematique_ROS_B.loop_ub_e = parent->NameInternal->size[0] *
      parent->NameInternal->size[1] - 1;
    for (Cinematique_ROS_B.d_a = 0; Cinematique_ROS_B.d_a <=
         Cinematique_ROS_B.loop_ub_e; Cinematique_ROS_B.d_a++) {
      bname->data[Cinematique_ROS_B.d_a] = parent->NameInternal->
        data[Cinematique_ROS_B.d_a];
    }

    Cinematiq_RigidBodyTree_addBody(iobj_0, body, bname, &(&(&(&(&iobj_1[0])[0])
      [0])[0])[1], &(&(&(&(&iobj_2[0])[0])[0])[0])[0], &(&(&(&(&iobj_3[0])[0])[0])
      [0])[0]);
  }

  if (2.0 <= obj->TreeInternal.NumBodies) {
    body = obj->TreeInternal.Bodies[1];
    Cinematique_ROS_B.bid_n = body->ParentIndex;
    if (Cinematique_ROS_B.bid_n > 0.0) {
      parent = obj->TreeInternal.Bodies[static_cast<int32_T>
        (Cinematique_ROS_B.bid_n) - 1];
    } else {
      parent = &obj->TreeInternal.Base;
    }

    Cinematique_ROS_B.d_a = bname->size[0] * bname->size[1];
    bname->size[0] = 1;
    bname->size[1] = parent->NameInternal->size[1];
    Cinema_emxEnsureCapacity_char_T(bname, Cinematique_ROS_B.d_a);
    Cinematique_ROS_B.loop_ub_e = parent->NameInternal->size[0] *
      parent->NameInternal->size[1] - 1;
    for (Cinematique_ROS_B.d_a = 0; Cinematique_ROS_B.d_a <=
         Cinematique_ROS_B.loop_ub_e; Cinematique_ROS_B.d_a++) {
      bname->data[Cinematique_ROS_B.d_a] = parent->NameInternal->
        data[Cinematique_ROS_B.d_a];
    }

    Cinematiq_RigidBodyTree_addBody(iobj_0, body, bname, &(&(&(&(&iobj_1[0])[0])
      [0])[0])[2], &(&(&(&(&iobj_2[0])[0])[0])[0])[1], &(&(&(&(&iobj_3[0])[0])[0])
      [0])[1]);
  }

  if (3.0 <= obj->TreeInternal.NumBodies) {
    body = obj->TreeInternal.Bodies[2];
    Cinematique_ROS_B.bid_n = body->ParentIndex;
    if (Cinematique_ROS_B.bid_n > 0.0) {
      parent = obj->TreeInternal.Bodies[static_cast<int32_T>
        (Cinematique_ROS_B.bid_n) - 1];
    } else {
      parent = &obj->TreeInternal.Base;
    }

    Cinematique_ROS_B.d_a = bname->size[0] * bname->size[1];
    bname->size[0] = 1;
    bname->size[1] = parent->NameInternal->size[1];
    Cinema_emxEnsureCapacity_char_T(bname, Cinematique_ROS_B.d_a);
    Cinematique_ROS_B.loop_ub_e = parent->NameInternal->size[0] *
      parent->NameInternal->size[1] - 1;
    for (Cinematique_ROS_B.d_a = 0; Cinematique_ROS_B.d_a <=
         Cinematique_ROS_B.loop_ub_e; Cinematique_ROS_B.d_a++) {
      bname->data[Cinematique_ROS_B.d_a] = parent->NameInternal->
        data[Cinematique_ROS_B.d_a];
    }

    Cinematiq_RigidBodyTree_addBody(iobj_0, body, bname, &(&(&(&(&iobj_1[0])[0])
      [0])[0])[3], &(&(&(&(&iobj_2[0])[0])[0])[0])[2], &(&(&(&(&iobj_3[0])[0])[0])
      [0])[2]);
  }

  if (4.0 <= obj->TreeInternal.NumBodies) {
    body = obj->TreeInternal.Bodies[3];
    Cinematique_ROS_B.bid_n = body->ParentIndex;
    if (Cinematique_ROS_B.bid_n > 0.0) {
      parent = obj->TreeInternal.Bodies[static_cast<int32_T>
        (Cinematique_ROS_B.bid_n) - 1];
    } else {
      parent = &obj->TreeInternal.Base;
    }

    Cinematique_ROS_B.d_a = bname->size[0] * bname->size[1];
    bname->size[0] = 1;
    bname->size[1] = parent->NameInternal->size[1];
    Cinema_emxEnsureCapacity_char_T(bname, Cinematique_ROS_B.d_a);
    Cinematique_ROS_B.loop_ub_e = parent->NameInternal->size[0] *
      parent->NameInternal->size[1] - 1;
    for (Cinematique_ROS_B.d_a = 0; Cinematique_ROS_B.d_a <=
         Cinematique_ROS_B.loop_ub_e; Cinematique_ROS_B.d_a++) {
      bname->data[Cinematique_ROS_B.d_a] = parent->NameInternal->
        data[Cinematique_ROS_B.d_a];
    }

    Cinematiq_RigidBodyTree_addBody(iobj_0, body, bname, &(&(&(&(&iobj_1[0])[0])
      [0])[0])[4], &(&(&(&(&iobj_2[0])[0])[0])[0])[3], &(&(&(&(&iobj_3[0])[0])[0])
      [0])[3]);
  }

  if (5.0 <= obj->TreeInternal.NumBodies) {
    body = obj->TreeInternal.Bodies[4];
    Cinematique_ROS_B.bid_n = body->ParentIndex;
    if (Cinematique_ROS_B.bid_n > 0.0) {
      parent = obj->TreeInternal.Bodies[static_cast<int32_T>
        (Cinematique_ROS_B.bid_n) - 1];
    } else {
      parent = &obj->TreeInternal.Base;
    }

    Cinematique_ROS_B.d_a = bname->size[0] * bname->size[1];
    bname->size[0] = 1;
    bname->size[1] = parent->NameInternal->size[1];
    Cinema_emxEnsureCapacity_char_T(bname, Cinematique_ROS_B.d_a);
    Cinematique_ROS_B.loop_ub_e = parent->NameInternal->size[0] *
      parent->NameInternal->size[1] - 1;
    for (Cinematique_ROS_B.d_a = 0; Cinematique_ROS_B.d_a <=
         Cinematique_ROS_B.loop_ub_e; Cinematique_ROS_B.d_a++) {
      bname->data[Cinematique_ROS_B.d_a] = parent->NameInternal->
        data[Cinematique_ROS_B.d_a];
    }

    Cinematiq_RigidBodyTree_addBody(iobj_0, body, bname, &(&(&(&(&iobj_1[0])[0])
      [0])[0])[5], &(&(&(&(&iobj_2[0])[0])[0])[0])[4], &(&(&(&(&iobj_3[0])[0])[0])
      [0])[4]);
  }

  Cinematique_ROS_emxFree_char_T(&bname);
  obj_0->RigidBodyTreeInternal = iobj_0;
  inverseKinematics_set_SolverAlg(obj_0, iobj_4);
  iobj_4 = obj_0->Solver;
  iobj_4->MaxNumIteration = 1500.0;
  iobj_4->MaxTime = 10.0;
  iobj_4->GradientTolerance = 1.0E-7;
  iobj_4->SolutionTolerance = 1.0E-6;
  iobj_4->ConstraintsOn = true;
  iobj_4->RandomRestart = false;
  iobj_4->StepTolerance = 1.0E-14;
  obj_0->matlabCodegenIsDeleted = false;
}

static void RigidBodyTree_get_JointPosition(ab_robotics_manip_internal_Ri_T *obj,
  emxArray_real_T_Cinematique_R_T *limits)
{
  static const char_T tmp[5] = { 'f', 'i', 'x', 'e', 'd' };

  c_rigidBodyJoint_Cinematique__T *obj_0;
  emxArray_char_T_Cinematique_R_T *a;
  y_robotics_manip_internal_Rig_T *body;
  real_T k;
  real_T pnum;
  int32_T b_i;
  int32_T b_kstr;
  int32_T c;
  int32_T exitg1;
  int32_T f;
  int32_T loop_ub;
  char_T b[5];
  boolean_T b_bool;
  b_kstr = limits->size[0] * limits->size[1];
  limits->size[0] = static_cast<int32_T>(obj->PositionNumber);
  limits->size[1] = 2;
  Cinema_emxEnsureCapacity_real_T(limits, b_kstr);
  loop_ub = (static_cast<int32_T>(obj->PositionNumber) << 1) - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    limits->data[b_kstr] = 0.0;
  }

  k = 1.0;
  pnum = obj->NumBodies;
  c = static_cast<int32_T>(pnum) - 1;
  Cinematique_ROS_emxInit_char_T(&a, 2);
  if (0 <= static_cast<int32_T>(pnum) - 1) {
    for (b_kstr = 0; b_kstr < 5; b_kstr++) {
      b[b_kstr] = tmp[b_kstr];
    }
  }

  for (b_i = 0; b_i <= c; b_i++) {
    body = obj->Bodies[b_i];
    b_kstr = a->size[0] * a->size[1];
    a->size[0] = 1;
    a->size[1] = body->JointInternal->Type->size[1];
    Cinema_emxEnsureCapacity_char_T(a, b_kstr);
    loop_ub = body->JointInternal->Type->size[0] * body->JointInternal->
      Type->size[1] - 1;
    for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
      a->data[b_kstr] = body->JointInternal->Type->data[b_kstr];
    }

    b_bool = false;
    if (a->size[1] == 5) {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 5) {
          if (a->data[b_kstr - 1] != b[b_kstr - 1]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (!b_bool) {
      pnum = body->JointInternal->PositionNumber;
      pnum += k;
      if (k > pnum - 1.0) {
        f = 0;
      } else {
        f = static_cast<int32_T>(k) - 1;
      }

      obj_0 = body->JointInternal;
      loop_ub = obj_0->PositionLimitsInternal->size[0];
      for (b_kstr = 0; b_kstr < loop_ub; b_kstr++) {
        limits->data[f + b_kstr] = obj_0->PositionLimitsInternal->data[b_kstr];
      }

      loop_ub = obj_0->PositionLimitsInternal->size[0];
      for (b_kstr = 0; b_kstr < loop_ub; b_kstr++) {
        limits->data[(f + b_kstr) + limits->size[0]] =
          obj_0->PositionLimitsInternal->data[b_kstr +
          obj_0->PositionLimitsInternal->size[0]];
      }

      k = pnum;
    }
  }

  Cinematique_ROS_emxFree_char_T(&a);
}

static void Cinematique_ROS_eml_find(const boolean_T x[4], int32_T i_data[],
  int32_T *i_size)
{
  int32_T b_ii;
  int32_T idx;
  boolean_T exitg1;
  idx = 0;
  b_ii = 1;
  exitg1 = false;
  while ((!exitg1) && (b_ii - 1 < 4)) {
    if (x[b_ii - 1]) {
      idx++;
      i_data[idx - 1] = b_ii;
      if (idx >= 4) {
        exitg1 = true;
      } else {
        b_ii++;
      }
    } else {
      b_ii++;
    }
  }

  if (1 > idx) {
    idx = 0;
  }

  *i_size = idx;
}

static void Cinematique_ROS_tic(real_T *tstart_tv_sec, real_T *tstart_tv_nsec)
{
  struct timespec b_timespec;
  clock_gettime(CLOCK_MONOTONIC, &b_timespec);
  *tstart_tv_sec = (real_T)b_timespec.tv_sec;
  *tstart_tv_nsec = (real_T)b_timespec.tv_nsec;
}

static void C_RigidBodyTree_ancestorIndices(ab_robotics_manip_internal_Ri_T *obj,
  y_robotics_manip_internal_Rig_T *body, emxArray_real_T_Cinematique_R_T
  *indices)
{
  emxArray_real_T_Cinematique_R_T *indices_0;
  Cinematique_ROS_B.i1 = indices->size[0] * indices->size[1];
  indices->size[0] = 1;
  indices->size[1] = static_cast<int32_T>(obj->NumBodies + 1.0);
  Cinema_emxEnsureCapacity_real_T(indices, Cinematique_ROS_B.i1);
  Cinematique_ROS_B.loop_ub_o = static_cast<int32_T>(obj->NumBodies + 1.0) - 1;
  for (Cinematique_ROS_B.i1 = 0; Cinematique_ROS_B.i1 <=
       Cinematique_ROS_B.loop_ub_o; Cinematique_ROS_B.i1++) {
    indices->data[Cinematique_ROS_B.i1] = 0.0;
  }

  Cinematique_ROS_B.i = 2.0;
  indices->data[0] = body->Index;
  while (body->ParentIndex > 0.0) {
    body = obj->Bodies[static_cast<int32_T>(body->ParentIndex) - 1];
    indices->data[static_cast<int32_T>(Cinematique_ROS_B.i) - 1] = body->Index;
    Cinematique_ROS_B.i++;
  }

  if (body->Index > 0.0) {
    indices->data[static_cast<int32_T>(Cinematique_ROS_B.i) - 1] =
      body->ParentIndex;
    Cinematique_ROS_B.i++;
  }

  Cinematique_ROS_emxInit_real_T(&indices_0, 2);
  Cinematique_ROS_B.loop_ub_o = static_cast<int32_T>(Cinematique_ROS_B.i - 1.0);
  Cinematique_ROS_B.i1 = indices_0->size[0] * indices_0->size[1];
  indices_0->size[0] = 1;
  indices_0->size[1] = static_cast<int32_T>(Cinematique_ROS_B.i - 1.0);
  Cinema_emxEnsureCapacity_real_T(indices_0, Cinematique_ROS_B.i1);
  for (Cinematique_ROS_B.i1 = 0; Cinematique_ROS_B.i1 <
       Cinematique_ROS_B.loop_ub_o; Cinematique_ROS_B.i1++) {
    indices_0->data[Cinematique_ROS_B.i1] = indices->data[Cinematique_ROS_B.i1];
  }

  Cinematique_ROS_B.i1 = indices->size[0] * indices->size[1];
  indices->size[0] = 1;
  indices->size[1] = indices_0->size[1];
  Cinema_emxEnsureCapacity_real_T(indices, Cinematique_ROS_B.i1);
  Cinematique_ROS_B.loop_ub_o = indices_0->size[0] * indices_0->size[1];
  for (Cinematique_ROS_B.i1 = 0; Cinematique_ROS_B.i1 <
       Cinematique_ROS_B.loop_ub_o; Cinematique_ROS_B.i1++) {
    indices->data[Cinematique_ROS_B.i1] = indices_0->data[Cinematique_ROS_B.i1];
  }

  Cinematique_ROS_emxFree_real_T(&indices_0);
}

static void RigidBodyTree_kinematicPathInte(ab_robotics_manip_internal_Ri_T *obj,
  y_robotics_manip_internal_Rig_T *body1, y_robotics_manip_internal_Rig_T *body2,
  emxArray_real_T_Cinematique_R_T *indices)
{
  emxArray_real_T_Cinematique_R_T *ancestorIndices1;
  emxArray_real_T_Cinematique_R_T *ancestorIndices2;
  boolean_T exitg1;
  Cinematique_ROS_emxInit_real_T(&ancestorIndices1, 2);
  Cinematique_ROS_emxInit_real_T(&ancestorIndices2, 2);
  C_RigidBodyTree_ancestorIndices(obj, body1, ancestorIndices1);
  C_RigidBodyTree_ancestorIndices(obj, body2, ancestorIndices2);
  Cinematique_ROS_B.numCommonAncestors = ancestorIndices1->size[1];
  Cinematique_ROS_B.u1_o = ancestorIndices2->size[1];
  if (Cinematique_ROS_B.numCommonAncestors < Cinematique_ROS_B.u1_o) {
    Cinematique_ROS_B.minPathLength = static_cast<int32_T>
      (Cinematique_ROS_B.numCommonAncestors);
  } else {
    Cinematique_ROS_B.minPathLength = static_cast<int32_T>
      (Cinematique_ROS_B.u1_o);
  }

  Cinematique_ROS_B.numCommonAncestors = Cinematique_ROS_B.minPathLength;
  Cinematique_ROS_B.b_i_a = 0;
  exitg1 = false;
  while ((!exitg1) && (Cinematique_ROS_B.b_i_a <=
                       Cinematique_ROS_B.minPathLength - 2)) {
    if (ancestorIndices1->data[static_cast<int32_T>(static_cast<real_T>
         (ancestorIndices1->size[1]) - (static_cast<real_T>
          (Cinematique_ROS_B.b_i_a) + 1.0)) - 1] != ancestorIndices2->data[
        static_cast<int32_T>(static_cast<real_T>(ancestorIndices2->size[1]) - (
          static_cast<real_T>(Cinematique_ROS_B.b_i_a) + 1.0)) - 1]) {
      Cinematique_ROS_B.numCommonAncestors = static_cast<real_T>
        (Cinematique_ROS_B.b_i_a) + 1.0;
      exitg1 = true;
    } else {
      Cinematique_ROS_B.b_i_a++;
    }
  }

  Cinematique_ROS_B.u1_o = static_cast<real_T>(ancestorIndices1->size[1]) -
    Cinematique_ROS_B.numCommonAncestors;
  if (1.0 > Cinematique_ROS_B.u1_o) {
    Cinematique_ROS_B.b_i_a = -1;
  } else {
    Cinematique_ROS_B.b_i_a = static_cast<int32_T>(Cinematique_ROS_B.u1_o) - 1;
  }

  Cinematique_ROS_B.u1_o = static_cast<real_T>(ancestorIndices2->size[1]) -
    Cinematique_ROS_B.numCommonAncestors;
  if (1.0 > Cinematique_ROS_B.u1_o) {
    Cinematique_ROS_B.j_e = 0;
    Cinematique_ROS_B.h = 1;
    Cinematique_ROS_B.g_j = -1;
  } else {
    Cinematique_ROS_B.j_e = static_cast<int32_T>(Cinematique_ROS_B.u1_o) - 1;
    Cinematique_ROS_B.h = -1;
    Cinematique_ROS_B.g_j = 0;
  }

  Cinematique_ROS_B.minPathLength = indices->size[0] * indices->size[1];
  indices->size[0] = 1;
  Cinematique_ROS_B.g_j = div_s32_floor(Cinematique_ROS_B.g_j -
    Cinematique_ROS_B.j_e, Cinematique_ROS_B.h);
  indices->size[1] = (Cinematique_ROS_B.g_j + Cinematique_ROS_B.b_i_a) + 3;
  Cinema_emxEnsureCapacity_real_T(indices, Cinematique_ROS_B.minPathLength);
  for (Cinematique_ROS_B.minPathLength = 0; Cinematique_ROS_B.minPathLength <=
       Cinematique_ROS_B.b_i_a; Cinematique_ROS_B.minPathLength++) {
    indices->data[Cinematique_ROS_B.minPathLength] = ancestorIndices1->
      data[Cinematique_ROS_B.minPathLength];
  }

  indices->data[Cinematique_ROS_B.b_i_a + 1] = ancestorIndices1->data[
    static_cast<int32_T>((static_cast<real_T>(ancestorIndices1->size[1]) -
    Cinematique_ROS_B.numCommonAncestors) + 1.0) - 1];
  Cinematique_ROS_emxFree_real_T(&ancestorIndices1);
  for (Cinematique_ROS_B.minPathLength = 0; Cinematique_ROS_B.minPathLength <=
       Cinematique_ROS_B.g_j; Cinematique_ROS_B.minPathLength++) {
    indices->data[(Cinematique_ROS_B.minPathLength + Cinematique_ROS_B.b_i_a) +
      2] = ancestorIndices2->data[Cinematique_ROS_B.h *
      Cinematique_ROS_B.minPathLength + Cinematique_ROS_B.j_e];
  }

  Cinematique_ROS_emxFree_real_T(&ancestorIndices2);
}

static void Cinematique_ROS_eye(real_T b_I[16])
{
  memset(&b_I[0], 0, sizeof(real_T) << 4U);
  b_I[0] = 1.0;
  b_I[5] = 1.0;
  b_I[10] = 1.0;
  b_I[15] = 1.0;
}

static void Ci_rigidBodyJoint_get_JointAxis(const
  c_rigidBodyJoint_Cinematique__T *obj, real_T ax[3])
{
  static const char_T tmp_0[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const char_T tmp[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  emxArray_char_T_Cinematique_R_T *a;
  int32_T exitg1;
  boolean_T guard1 = false;
  Cinematique_ROS_emxInit_char_T(&a, 2);
  Cinematique_ROS_B.b_kstr_e = a->size[0] * a->size[1];
  a->size[0] = 1;
  a->size[1] = obj->Type->size[1];
  Cinema_emxEnsureCapacity_char_T(a, Cinematique_ROS_B.b_kstr_e);
  Cinematique_ROS_B.loop_ub_m = obj->Type->size[0] * obj->Type->size[1] - 1;
  for (Cinematique_ROS_B.b_kstr_e = 0; Cinematique_ROS_B.b_kstr_e <=
       Cinematique_ROS_B.loop_ub_m; Cinematique_ROS_B.b_kstr_e++) {
    a->data[Cinematique_ROS_B.b_kstr_e] = obj->Type->
      data[Cinematique_ROS_B.b_kstr_e];
  }

  for (Cinematique_ROS_B.b_kstr_e = 0; Cinematique_ROS_B.b_kstr_e < 8;
       Cinematique_ROS_B.b_kstr_e++) {
    Cinematique_ROS_B.b_da[Cinematique_ROS_B.b_kstr_e] =
      tmp[Cinematique_ROS_B.b_kstr_e];
  }

  Cinematique_ROS_B.b_bool_f = false;
  if (a->size[1] == 8) {
    Cinematique_ROS_B.b_kstr_e = 1;
    do {
      exitg1 = 0;
      if (Cinematique_ROS_B.b_kstr_e - 1 < 8) {
        if (a->data[Cinematique_ROS_B.b_kstr_e - 1] !=
            Cinematique_ROS_B.b_da[Cinematique_ROS_B.b_kstr_e - 1]) {
          exitg1 = 1;
        } else {
          Cinematique_ROS_B.b_kstr_e++;
        }
      } else {
        Cinematique_ROS_B.b_bool_f = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  guard1 = false;
  if (Cinematique_ROS_B.b_bool_f) {
    guard1 = true;
  } else {
    Cinematique_ROS_B.b_kstr_e = a->size[0] * a->size[1];
    a->size[0] = 1;
    a->size[1] = obj->Type->size[1];
    Cinema_emxEnsureCapacity_char_T(a, Cinematique_ROS_B.b_kstr_e);
    Cinematique_ROS_B.loop_ub_m = obj->Type->size[0] * obj->Type->size[1] - 1;
    for (Cinematique_ROS_B.b_kstr_e = 0; Cinematique_ROS_B.b_kstr_e <=
         Cinematique_ROS_B.loop_ub_m; Cinematique_ROS_B.b_kstr_e++) {
      a->data[Cinematique_ROS_B.b_kstr_e] = obj->Type->
        data[Cinematique_ROS_B.b_kstr_e];
    }

    for (Cinematique_ROS_B.b_kstr_e = 0; Cinematique_ROS_B.b_kstr_e < 9;
         Cinematique_ROS_B.b_kstr_e++) {
      Cinematique_ROS_B.b_o[Cinematique_ROS_B.b_kstr_e] =
        tmp_0[Cinematique_ROS_B.b_kstr_e];
    }

    Cinematique_ROS_B.b_bool_f = false;
    if (a->size[1] == 9) {
      Cinematique_ROS_B.b_kstr_e = 1;
      do {
        exitg1 = 0;
        if (Cinematique_ROS_B.b_kstr_e - 1 < 9) {
          if (a->data[Cinematique_ROS_B.b_kstr_e - 1] !=
              Cinematique_ROS_B.b_o[Cinematique_ROS_B.b_kstr_e - 1]) {
            exitg1 = 1;
          } else {
            Cinematique_ROS_B.b_kstr_e++;
          }
        } else {
          Cinematique_ROS_B.b_bool_f = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (Cinematique_ROS_B.b_bool_f) {
      guard1 = true;
    } else {
      ax[0] = (rtNaN);
      ax[1] = (rtNaN);
      ax[2] = (rtNaN);
    }
  }

  if (guard1) {
    ax[0] = obj->JointAxisInternal[0];
    ax[1] = obj->JointAxisInternal[1];
    ax[2] = obj->JointAxisInternal[2];
  }

  Cinematique_ROS_emxFree_char_T(&a);
}

static void Cinematique_ROS_cat(real_T varargin_1, real_T varargin_2, real_T
  varargin_3, real_T varargin_4, real_T varargin_5, real_T varargin_6, real_T
  varargin_7, real_T varargin_8, real_T varargin_9, real_T y[9])
{
  y[0] = varargin_1;
  y[1] = varargin_2;
  y[2] = varargin_3;
  y[3] = varargin_4;
  y[4] = varargin_5;
  y[5] = varargin_6;
  y[6] = varargin_7;
  y[7] = varargin_8;
  y[8] = varargin_9;
}

static void Cinematique_ROS_mtimes(const real_T A[36], const
  emxArray_real_T_Cinematique_R_T *B, emxArray_real_T_Cinematique_R_T *C)
{
  Cinematique_ROS_B.n_n = B->size[1] - 1;
  Cinematique_ROS_B.b_j_o = C->size[0] * C->size[1];
  C->size[0] = 6;
  C->size[1] = B->size[1];
  Cinema_emxEnsureCapacity_real_T(C, Cinematique_ROS_B.b_j_o);
  for (Cinematique_ROS_B.b_j_o = 0; Cinematique_ROS_B.b_j_o <=
       Cinematique_ROS_B.n_n; Cinematique_ROS_B.b_j_o++) {
    Cinematique_ROS_B.coffset_tmp = Cinematique_ROS_B.b_j_o * 6 - 1;
    for (Cinematique_ROS_B.b_i_g = 0; Cinematique_ROS_B.b_i_g < 6;
         Cinematique_ROS_B.b_i_g++) {
      Cinematique_ROS_B.s_o = 0.0;
      for (Cinematique_ROS_B.b_k_c = 0; Cinematique_ROS_B.b_k_c < 6;
           Cinematique_ROS_B.b_k_c++) {
        Cinematique_ROS_B.s_o += A[Cinematique_ROS_B.b_k_c * 6 +
          Cinematique_ROS_B.b_i_g] * B->data[(Cinematique_ROS_B.coffset_tmp +
          Cinematique_ROS_B.b_k_c) + 1];
      }

      C->data[(Cinematique_ROS_B.coffset_tmp + Cinematique_ROS_B.b_i_g) + 1] =
        Cinematique_ROS_B.s_o;
    }
  }
}

static void RigidBodyTree_efficientFKAndJac(ab_robotics_manip_internal_Ri_T *obj,
  const real_T qv[4], const emxArray_char_T_Cinematique_R_T *body1Name, real_T
  T_data[], int32_T T_size[2], emxArray_real_T_Cinematique_R_T *Jac)
{
  static const char_T tmp_1[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_0[5] = { 'f', 'i', 'x', 'e', 'd' };

  c_rigidBodyJoint_Cinematique__T *joint;
  emxArray_char_T_Cinematique_R_T *body2Name;
  emxArray_real_T_Cinematique_R_T *Jac_0;
  emxArray_real_T_Cinematique_R_T *b;
  emxArray_real_T_Cinematique_R_T *kinematicPathIndices;
  emxArray_real_T_Cinematique_R_T *tmp;
  y_robotics_manip_internal_Rig_T *body1;
  y_robotics_manip_internal_Rig_T *body2;
  int32_T exitg1;
  Cinematique_ROS_emxInit_char_T(&body2Name, 2);
  Cinematique_ROS_B.b_kstr = body2Name->size[0] * body2Name->size[1];
  body2Name->size[0] = 1;
  body2Name->size[1] = obj->Base.NameInternal->size[1];
  Cinema_emxEnsureCapacity_char_T(body2Name, Cinematique_ROS_B.b_kstr);
  Cinematique_ROS_B.loop_ub_px = obj->Base.NameInternal->size[0] *
    obj->Base.NameInternal->size[1] - 1;
  for (Cinematique_ROS_B.b_kstr = 0; Cinematique_ROS_B.b_kstr <=
       Cinematique_ROS_B.loop_ub_px; Cinematique_ROS_B.b_kstr++) {
    body2Name->data[Cinematique_ROS_B.b_kstr] = obj->Base.NameInternal->
      data[Cinematique_ROS_B.b_kstr];
  }

  Cinematique_ROS_B.bid1 = RigidBodyTree_findBodyIndexByNa(obj, body1Name);
  Cinematique_ROS_B.bid2 = RigidBodyTree_findBodyIndexByNa(obj, body2Name);
  if (Cinematique_ROS_B.bid1 == 0.0) {
    body1 = &obj->Base;
  } else {
    body1 = obj->Bodies[static_cast<int32_T>(Cinematique_ROS_B.bid1) - 1];
  }

  if (Cinematique_ROS_B.bid2 == 0.0) {
    body2 = &obj->Base;
  } else {
    body2 = obj->Bodies[static_cast<int32_T>(Cinematique_ROS_B.bid2) - 1];
  }

  Cinematique_ROS_emxInit_real_T(&kinematicPathIndices, 2);
  RigidBodyTree_kinematicPathInte(obj, body1, body2, kinematicPathIndices);
  Cinematique_ROS_eye(Cinematique_ROS_B.T1);
  Cinematique_ROS_B.b_kstr = Jac->size[0] * Jac->size[1];
  Jac->size[0] = 6;
  Jac->size[1] = static_cast<int32_T>(obj->PositionNumber);
  Cinema_emxEnsureCapacity_real_T(Jac, Cinematique_ROS_B.b_kstr);
  Cinematique_ROS_B.loop_ub_px = 6 * static_cast<int32_T>(obj->PositionNumber) -
    1;
  for (Cinematique_ROS_B.b_kstr = 0; Cinematique_ROS_B.b_kstr <=
       Cinematique_ROS_B.loop_ub_px; Cinematique_ROS_B.b_kstr++) {
    Jac->data[Cinematique_ROS_B.b_kstr] = 0.0;
  }

  Cinematique_ROS_B.c_k = kinematicPathIndices->size[1] - 2;
  Cinematique_ROS_emxInit_real_T(&b, 2);
  Cinematique_ROS_emxInit_real_T(&tmp, 2);
  if (0 <= Cinematique_ROS_B.c_k) {
    for (Cinematique_ROS_B.b_kstr = 0; Cinematique_ROS_B.b_kstr < 5;
         Cinematique_ROS_B.b_kstr++) {
      Cinematique_ROS_B.b_g[Cinematique_ROS_B.b_kstr] =
        tmp_0[Cinematique_ROS_B.b_kstr];
    }
  }

  for (Cinematique_ROS_B.b_i_p = 0; Cinematique_ROS_B.b_i_p <=
       Cinematique_ROS_B.c_k; Cinematique_ROS_B.b_i_p++) {
    if (kinematicPathIndices->data[Cinematique_ROS_B.b_i_p] != 0.0) {
      body1 = obj->Bodies[static_cast<int32_T>(kinematicPathIndices->
        data[Cinematique_ROS_B.b_i_p]) - 1];
    } else {
      body1 = &obj->Base;
    }

    if (kinematicPathIndices->data[static_cast<int32_T>((static_cast<real_T>
          (Cinematique_ROS_B.b_i_p) + 1.0) + 1.0) - 1] != 0.0) {
      body2 = obj->Bodies[static_cast<int32_T>(kinematicPathIndices->data[
        static_cast<int32_T>((static_cast<real_T>(Cinematique_ROS_B.b_i_p) + 1.0)
        + 1.0) - 1]) - 1];
    } else {
      body2 = &obj->Base;
    }

    Cinematique_ROS_B.nextBodyIsParent = (body2->Index == body1->ParentIndex);
    if (Cinematique_ROS_B.nextBodyIsParent) {
      body2 = body1;
      Cinematique_ROS_B.jointSign = 1;
    } else {
      Cinematique_ROS_B.jointSign = -1;
    }

    joint = body2->JointInternal;
    Cinematique_ROS_B.b_kstr = body2Name->size[0] * body2Name->size[1];
    body2Name->size[0] = 1;
    body2Name->size[1] = joint->Type->size[1];
    Cinema_emxEnsureCapacity_char_T(body2Name, Cinematique_ROS_B.b_kstr);
    Cinematique_ROS_B.loop_ub_px = joint->Type->size[0] * joint->Type->size[1] -
      1;
    for (Cinematique_ROS_B.b_kstr = 0; Cinematique_ROS_B.b_kstr <=
         Cinematique_ROS_B.loop_ub_px; Cinematique_ROS_B.b_kstr++) {
      body2Name->data[Cinematique_ROS_B.b_kstr] = joint->Type->
        data[Cinematique_ROS_B.b_kstr];
    }

    Cinematique_ROS_B.b_bool = false;
    if (body2Name->size[1] == 5) {
      Cinematique_ROS_B.b_kstr = 1;
      do {
        exitg1 = 0;
        if (Cinematique_ROS_B.b_kstr - 1 < 5) {
          if (body2Name->data[Cinematique_ROS_B.b_kstr - 1] !=
              Cinematique_ROS_B.b_g[Cinematique_ROS_B.b_kstr - 1]) {
            exitg1 = 1;
          } else {
            Cinematique_ROS_B.b_kstr++;
          }
        } else {
          Cinematique_ROS_B.b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (Cinematique_ROS_B.b_bool) {
      for (Cinematique_ROS_B.b_kstr = 0; Cinematique_ROS_B.b_kstr < 16;
           Cinematique_ROS_B.b_kstr++) {
        Cinematique_ROS_B.Tj_c[Cinematique_ROS_B.b_kstr] =
          joint->JointToParentTransform[Cinematique_ROS_B.b_kstr];
      }

      Cinematique_ROS_B.b_kstr = body2Name->size[0] * body2Name->size[1];
      body2Name->size[0] = 1;
      body2Name->size[1] = joint->Type->size[1];
      Cinema_emxEnsureCapacity_char_T(body2Name, Cinematique_ROS_B.b_kstr);
      Cinematique_ROS_B.loop_ub_px = joint->Type->size[0] * joint->Type->size[1]
        - 1;
      for (Cinematique_ROS_B.b_kstr = 0; Cinematique_ROS_B.b_kstr <=
           Cinematique_ROS_B.loop_ub_px; Cinematique_ROS_B.b_kstr++) {
        body2Name->data[Cinematique_ROS_B.b_kstr] = joint->Type->
          data[Cinematique_ROS_B.b_kstr];
      }

      Cinematique_ROS_B.b_bool = false;
      if (body2Name->size[1] == 5) {
        Cinematique_ROS_B.b_kstr = 1;
        do {
          exitg1 = 0;
          if (Cinematique_ROS_B.b_kstr - 1 < 5) {
            if (body2Name->data[Cinematique_ROS_B.b_kstr - 1] !=
                Cinematique_ROS_B.b_g[Cinematique_ROS_B.b_kstr - 1]) {
              exitg1 = 1;
            } else {
              Cinematique_ROS_B.b_kstr++;
            }
          } else {
            Cinematique_ROS_B.b_bool = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }

      if (Cinematique_ROS_B.b_bool) {
        Cinematique_ROS_B.b_kstr = 0;
      } else {
        for (Cinematique_ROS_B.b_kstr = 0; Cinematique_ROS_B.b_kstr < 8;
             Cinematique_ROS_B.b_kstr++) {
          Cinematique_ROS_B.b_bs[Cinematique_ROS_B.b_kstr] =
            tmp_1[Cinematique_ROS_B.b_kstr];
        }

        Cinematique_ROS_B.b_bool = false;
        if (body2Name->size[1] == 8) {
          Cinematique_ROS_B.b_kstr = 1;
          do {
            exitg1 = 0;
            if (Cinematique_ROS_B.b_kstr - 1 < 8) {
              if (body2Name->data[Cinematique_ROS_B.b_kstr - 1] !=
                  Cinematique_ROS_B.b_bs[Cinematique_ROS_B.b_kstr - 1]) {
                exitg1 = 1;
              } else {
                Cinematique_ROS_B.b_kstr++;
              }
            } else {
              Cinematique_ROS_B.b_bool = true;
              exitg1 = 1;
            }
          } while (exitg1 == 0);
        }

        if (Cinematique_ROS_B.b_bool) {
          Cinematique_ROS_B.b_kstr = 1;
        } else {
          Cinematique_ROS_B.b_kstr = -1;
        }
      }

      switch (Cinematique_ROS_B.b_kstr) {
       case 0:
        memset(&Cinematique_ROS_B.T1j[0], 0, sizeof(real_T) << 4U);
        Cinematique_ROS_B.T1j[0] = 1.0;
        Cinematique_ROS_B.T1j[5] = 1.0;
        Cinematique_ROS_B.T1j[10] = 1.0;
        Cinematique_ROS_B.T1j[15] = 1.0;
        break;

       case 1:
        Ci_rigidBodyJoint_get_JointAxis(joint, Cinematique_ROS_B.v_j);
        Cinematique_ROS_B.bid2 = Cinematique_ROS_B.v_j[0];
        Cinematique_ROS_B.tempR_tmp = Cinematique_ROS_B.v_j[1];
        Cinematique_ROS_B.tempR_tmp_o = Cinematique_ROS_B.v_j[2];
        Cinematique_ROS_B.bid1 = 1.0 / sqrt((Cinematique_ROS_B.bid2 *
          Cinematique_ROS_B.bid2 + Cinematique_ROS_B.tempR_tmp *
          Cinematique_ROS_B.tempR_tmp) + Cinematique_ROS_B.tempR_tmp_o *
          Cinematique_ROS_B.tempR_tmp_o);
        Cinematique_ROS_B.v_j[0] = Cinematique_ROS_B.bid2 *
          Cinematique_ROS_B.bid1;
        Cinematique_ROS_B.v_j[1] = Cinematique_ROS_B.tempR_tmp *
          Cinematique_ROS_B.bid1;
        Cinematique_ROS_B.v_j[2] = Cinematique_ROS_B.tempR_tmp_o *
          Cinematique_ROS_B.bid1;
        Cinematique_ROS_B.bid2 = Cinematique_ROS_B.v_j[1] *
          Cinematique_ROS_B.v_j[0] * 0.0;
        Cinematique_ROS_B.tempR_tmp = Cinematique_ROS_B.v_j[2] *
          Cinematique_ROS_B.v_j[0] * 0.0;
        Cinematique_ROS_B.tempR_tmp_o = Cinematique_ROS_B.v_j[2] *
          Cinematique_ROS_B.v_j[1] * 0.0;
        Cinematique_ROS_cat(Cinematique_ROS_B.v_j[0] * Cinematique_ROS_B.v_j[0] *
                            0.0 + 1.0, Cinematique_ROS_B.bid2 -
                            Cinematique_ROS_B.v_j[2] * 0.0,
                            Cinematique_ROS_B.tempR_tmp + Cinematique_ROS_B.v_j
                            [1] * 0.0, Cinematique_ROS_B.bid2 +
                            Cinematique_ROS_B.v_j[2] * 0.0,
                            Cinematique_ROS_B.v_j[1] * Cinematique_ROS_B.v_j[1] *
                            0.0 + 1.0, Cinematique_ROS_B.tempR_tmp_o -
                            Cinematique_ROS_B.v_j[0] * 0.0,
                            Cinematique_ROS_B.tempR_tmp - Cinematique_ROS_B.v_j
                            [1] * 0.0, Cinematique_ROS_B.tempR_tmp_o +
                            Cinematique_ROS_B.v_j[0] * 0.0,
                            Cinematique_ROS_B.v_j[2] * Cinematique_ROS_B.v_j[2] *
                            0.0 + 1.0, Cinematique_ROS_B.tempR);
        for (Cinematique_ROS_B.b_kstr = 0; Cinematique_ROS_B.b_kstr < 3;
             Cinematique_ROS_B.b_kstr++) {
          Cinematique_ROS_B.R[Cinematique_ROS_B.b_kstr] =
            Cinematique_ROS_B.tempR[Cinematique_ROS_B.b_kstr * 3];
          Cinematique_ROS_B.R[Cinematique_ROS_B.b_kstr + 3] =
            Cinematique_ROS_B.tempR[Cinematique_ROS_B.b_kstr * 3 + 1];
          Cinematique_ROS_B.R[Cinematique_ROS_B.b_kstr + 6] =
            Cinematique_ROS_B.tempR[Cinematique_ROS_B.b_kstr * 3 + 2];
        }

        memset(&Cinematique_ROS_B.T1j[0], 0, sizeof(real_T) << 4U);
        for (Cinematique_ROS_B.b_kstr = 0; Cinematique_ROS_B.b_kstr < 3;
             Cinematique_ROS_B.b_kstr++) {
          Cinematique_ROS_B.f = Cinematique_ROS_B.b_kstr << 2;
          Cinematique_ROS_B.T1j[Cinematique_ROS_B.f] = Cinematique_ROS_B.R[3 *
            Cinematique_ROS_B.b_kstr];
          Cinematique_ROS_B.T1j[Cinematique_ROS_B.f + 1] = Cinematique_ROS_B.R[3
            * Cinematique_ROS_B.b_kstr + 1];
          Cinematique_ROS_B.T1j[Cinematique_ROS_B.f + 2] = Cinematique_ROS_B.R[3
            * Cinematique_ROS_B.b_kstr + 2];
        }

        Cinematique_ROS_B.T1j[15] = 1.0;
        break;

       default:
        Ci_rigidBodyJoint_get_JointAxis(joint, Cinematique_ROS_B.v_j);
        memset(&Cinematique_ROS_B.tempR[0], 0, 9U * sizeof(real_T));
        Cinematique_ROS_B.tempR[0] = 1.0;
        Cinematique_ROS_B.tempR[4] = 1.0;
        Cinematique_ROS_B.tempR[8] = 1.0;
        for (Cinematique_ROS_B.b_kstr = 0; Cinematique_ROS_B.b_kstr < 3;
             Cinematique_ROS_B.b_kstr++) {
          Cinematique_ROS_B.f = Cinematique_ROS_B.b_kstr << 2;
          Cinematique_ROS_B.T1j[Cinematique_ROS_B.f] = Cinematique_ROS_B.tempR[3
            * Cinematique_ROS_B.b_kstr];
          Cinematique_ROS_B.T1j[Cinematique_ROS_B.f + 1] =
            Cinematique_ROS_B.tempR[3 * Cinematique_ROS_B.b_kstr + 1];
          Cinematique_ROS_B.T1j[Cinematique_ROS_B.f + 2] =
            Cinematique_ROS_B.tempR[3 * Cinematique_ROS_B.b_kstr + 2];
          Cinematique_ROS_B.T1j[Cinematique_ROS_B.b_kstr + 12] =
            Cinematique_ROS_B.v_j[Cinematique_ROS_B.b_kstr] * 0.0;
        }

        Cinematique_ROS_B.T1j[3] = 0.0;
        Cinematique_ROS_B.T1j[7] = 0.0;
        Cinematique_ROS_B.T1j[11] = 0.0;
        Cinematique_ROS_B.T1j[15] = 1.0;
        break;
      }

      for (Cinematique_ROS_B.b_kstr = 0; Cinematique_ROS_B.b_kstr < 16;
           Cinematique_ROS_B.b_kstr++) {
        Cinematique_ROS_B.b[Cinematique_ROS_B.b_kstr] =
          joint->ChildToJointTransform[Cinematique_ROS_B.b_kstr];
      }

      for (Cinematique_ROS_B.b_kstr = 0; Cinematique_ROS_B.b_kstr < 4;
           Cinematique_ROS_B.b_kstr++) {
        for (Cinematique_ROS_B.g = 0; Cinematique_ROS_B.g < 4;
             Cinematique_ROS_B.g++) {
          Cinematique_ROS_B.f = Cinematique_ROS_B.g << 2;
          Cinematique_ROS_B.loop_ub_px = Cinematique_ROS_B.b_kstr +
            Cinematique_ROS_B.f;
          Cinematique_ROS_B.Tj_k[Cinematique_ROS_B.loop_ub_px] = 0.0;
          Cinematique_ROS_B.Tj_k[Cinematique_ROS_B.loop_ub_px] +=
            Cinematique_ROS_B.T1j[Cinematique_ROS_B.f] *
            Cinematique_ROS_B.Tj_c[Cinematique_ROS_B.b_kstr];
          Cinematique_ROS_B.Tj_k[Cinematique_ROS_B.loop_ub_px] +=
            Cinematique_ROS_B.T1j[Cinematique_ROS_B.f + 1] *
            Cinematique_ROS_B.Tj_c[Cinematique_ROS_B.b_kstr + 4];
          Cinematique_ROS_B.Tj_k[Cinematique_ROS_B.loop_ub_px] +=
            Cinematique_ROS_B.T1j[Cinematique_ROS_B.f + 2] *
            Cinematique_ROS_B.Tj_c[Cinematique_ROS_B.b_kstr + 8];
          Cinematique_ROS_B.Tj_k[Cinematique_ROS_B.loop_ub_px] +=
            Cinematique_ROS_B.T1j[Cinematique_ROS_B.f + 3] *
            Cinematique_ROS_B.Tj_c[Cinematique_ROS_B.b_kstr + 12];
        }

        for (Cinematique_ROS_B.g = 0; Cinematique_ROS_B.g < 4;
             Cinematique_ROS_B.g++) {
          Cinematique_ROS_B.f = Cinematique_ROS_B.g << 2;
          Cinematique_ROS_B.loop_ub_px = Cinematique_ROS_B.b_kstr +
            Cinematique_ROS_B.f;
          Cinematique_ROS_B.Tc2p[Cinematique_ROS_B.loop_ub_px] = 0.0;
          Cinematique_ROS_B.Tc2p[Cinematique_ROS_B.loop_ub_px] +=
            Cinematique_ROS_B.b[Cinematique_ROS_B.f] *
            Cinematique_ROS_B.Tj_k[Cinematique_ROS_B.b_kstr];
          Cinematique_ROS_B.Tc2p[Cinematique_ROS_B.loop_ub_px] +=
            Cinematique_ROS_B.b[Cinematique_ROS_B.f + 1] *
            Cinematique_ROS_B.Tj_k[Cinematique_ROS_B.b_kstr + 4];
          Cinematique_ROS_B.Tc2p[Cinematique_ROS_B.loop_ub_px] +=
            Cinematique_ROS_B.b[Cinematique_ROS_B.f + 2] *
            Cinematique_ROS_B.Tj_k[Cinematique_ROS_B.b_kstr + 8];
          Cinematique_ROS_B.Tc2p[Cinematique_ROS_B.loop_ub_px] +=
            Cinematique_ROS_B.b[Cinematique_ROS_B.f + 3] *
            Cinematique_ROS_B.Tj_k[Cinematique_ROS_B.b_kstr + 12];
        }
      }
    } else {
      Cinematique_ROS_B.b_kstr = static_cast<int32_T>(body2->Index);
      Cinematique_ROS_B.bid1 = obj->PositionDoFMap[Cinematique_ROS_B.b_kstr - 1];
      Cinematique_ROS_B.bid2 = obj->PositionDoFMap[Cinematique_ROS_B.b_kstr + 4];
      if (Cinematique_ROS_B.bid1 > Cinematique_ROS_B.bid2) {
        Cinematique_ROS_B.g = 0;
        Cinematique_ROS_B.f = 0;
      } else {
        Cinematique_ROS_B.g = static_cast<int32_T>(Cinematique_ROS_B.bid1) - 1;
        Cinematique_ROS_B.f = static_cast<int32_T>(Cinematique_ROS_B.bid2);
      }

      for (Cinematique_ROS_B.b_kstr = 0; Cinematique_ROS_B.b_kstr < 16;
           Cinematique_ROS_B.b_kstr++) {
        Cinematique_ROS_B.Tj_c[Cinematique_ROS_B.b_kstr] =
          joint->JointToParentTransform[Cinematique_ROS_B.b_kstr];
      }

      Cinematique_ROS_B.b_kstr = body2Name->size[0] * body2Name->size[1];
      body2Name->size[0] = 1;
      body2Name->size[1] = joint->Type->size[1];
      Cinema_emxEnsureCapacity_char_T(body2Name, Cinematique_ROS_B.b_kstr);
      Cinematique_ROS_B.loop_ub_px = joint->Type->size[0] * joint->Type->size[1]
        - 1;
      for (Cinematique_ROS_B.b_kstr = 0; Cinematique_ROS_B.b_kstr <=
           Cinematique_ROS_B.loop_ub_px; Cinematique_ROS_B.b_kstr++) {
        body2Name->data[Cinematique_ROS_B.b_kstr] = joint->Type->
          data[Cinematique_ROS_B.b_kstr];
      }

      Cinematique_ROS_B.b_bool = false;
      if (body2Name->size[1] == 5) {
        Cinematique_ROS_B.b_kstr = 1;
        do {
          exitg1 = 0;
          if (Cinematique_ROS_B.b_kstr - 1 < 5) {
            if (body2Name->data[Cinematique_ROS_B.b_kstr - 1] !=
                Cinematique_ROS_B.b_g[Cinematique_ROS_B.b_kstr - 1]) {
              exitg1 = 1;
            } else {
              Cinematique_ROS_B.b_kstr++;
            }
          } else {
            Cinematique_ROS_B.b_bool = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }

      if (Cinematique_ROS_B.b_bool) {
        Cinematique_ROS_B.b_kstr = 0;
      } else {
        for (Cinematique_ROS_B.b_kstr = 0; Cinematique_ROS_B.b_kstr < 8;
             Cinematique_ROS_B.b_kstr++) {
          Cinematique_ROS_B.b_bs[Cinematique_ROS_B.b_kstr] =
            tmp_1[Cinematique_ROS_B.b_kstr];
        }

        Cinematique_ROS_B.b_bool = false;
        if (body2Name->size[1] == 8) {
          Cinematique_ROS_B.b_kstr = 1;
          do {
            exitg1 = 0;
            if (Cinematique_ROS_B.b_kstr - 1 < 8) {
              if (body2Name->data[Cinematique_ROS_B.b_kstr - 1] !=
                  Cinematique_ROS_B.b_bs[Cinematique_ROS_B.b_kstr - 1]) {
                exitg1 = 1;
              } else {
                Cinematique_ROS_B.b_kstr++;
              }
            } else {
              Cinematique_ROS_B.b_bool = true;
              exitg1 = 1;
            }
          } while (exitg1 == 0);
        }

        if (Cinematique_ROS_B.b_bool) {
          Cinematique_ROS_B.b_kstr = 1;
        } else {
          Cinematique_ROS_B.b_kstr = -1;
        }
      }

      switch (Cinematique_ROS_B.b_kstr) {
       case 0:
        memset(&Cinematique_ROS_B.T1j[0], 0, sizeof(real_T) << 4U);
        Cinematique_ROS_B.T1j[0] = 1.0;
        Cinematique_ROS_B.T1j[5] = 1.0;
        Cinematique_ROS_B.T1j[10] = 1.0;
        Cinematique_ROS_B.T1j[15] = 1.0;
        break;

       case 1:
        Ci_rigidBodyJoint_get_JointAxis(joint, Cinematique_ROS_B.v_j);
        Cinematique_ROS_B.f -= Cinematique_ROS_B.g;
        for (Cinematique_ROS_B.b_kstr = 0; Cinematique_ROS_B.b_kstr <
             Cinematique_ROS_B.f; Cinematique_ROS_B.b_kstr++) {
          Cinematique_ROS_B.g_data[Cinematique_ROS_B.b_kstr] =
            Cinematique_ROS_B.g + Cinematique_ROS_B.b_kstr;
        }

        Cinematique_ROS_B.result_data[0] = Cinematique_ROS_B.v_j[0];
        Cinematique_ROS_B.result_data[1] = Cinematique_ROS_B.v_j[1];
        Cinematique_ROS_B.result_data[2] = Cinematique_ROS_B.v_j[2];
        if (0 <= (Cinematique_ROS_B.f != 0) - 1) {
          Cinematique_ROS_B.result_data[3] = qv[Cinematique_ROS_B.g_data[0]];
        }

        Cinematique_ROS_B.bid1 = 1.0 / sqrt((Cinematique_ROS_B.result_data[0] *
          Cinematique_ROS_B.result_data[0] + Cinematique_ROS_B.result_data[1] *
          Cinematique_ROS_B.result_data[1]) + Cinematique_ROS_B.result_data[2] *
          Cinematique_ROS_B.result_data[2]);
        Cinematique_ROS_B.v_j[0] = Cinematique_ROS_B.result_data[0] *
          Cinematique_ROS_B.bid1;
        Cinematique_ROS_B.v_j[1] = Cinematique_ROS_B.result_data[1] *
          Cinematique_ROS_B.bid1;
        Cinematique_ROS_B.v_j[2] = Cinematique_ROS_B.result_data[2] *
          Cinematique_ROS_B.bid1;
        Cinematique_ROS_B.bid1 = cos(Cinematique_ROS_B.result_data[3]);
        Cinematique_ROS_B.sth = sin(Cinematique_ROS_B.result_data[3]);
        Cinematique_ROS_B.bid2 = Cinematique_ROS_B.v_j[1] *
          Cinematique_ROS_B.v_j[0] * (1.0 - Cinematique_ROS_B.bid1);
        Cinematique_ROS_B.tempR_tmp = Cinematique_ROS_B.v_j[2] *
          Cinematique_ROS_B.sth;
        Cinematique_ROS_B.tempR_tmp_o = Cinematique_ROS_B.v_j[2] *
          Cinematique_ROS_B.v_j[0] * (1.0 - Cinematique_ROS_B.bid1);
        Cinematique_ROS_B.tempR_tmp_n = Cinematique_ROS_B.v_j[1] *
          Cinematique_ROS_B.sth;
        Cinematique_ROS_B.tempR_tmp_i = Cinematique_ROS_B.v_j[2] *
          Cinematique_ROS_B.v_j[1] * (1.0 - Cinematique_ROS_B.bid1);
        Cinematique_ROS_B.sth *= Cinematique_ROS_B.v_j[0];
        Cinematique_ROS_cat(Cinematique_ROS_B.v_j[0] * Cinematique_ROS_B.v_j[0] *
                            (1.0 - Cinematique_ROS_B.bid1) +
                            Cinematique_ROS_B.bid1, Cinematique_ROS_B.bid2 -
                            Cinematique_ROS_B.tempR_tmp,
                            Cinematique_ROS_B.tempR_tmp_o +
                            Cinematique_ROS_B.tempR_tmp_n,
                            Cinematique_ROS_B.bid2 + Cinematique_ROS_B.tempR_tmp,
                            Cinematique_ROS_B.v_j[1] * Cinematique_ROS_B.v_j[1] *
                            (1.0 - Cinematique_ROS_B.bid1) +
                            Cinematique_ROS_B.bid1,
                            Cinematique_ROS_B.tempR_tmp_i -
                            Cinematique_ROS_B.sth, Cinematique_ROS_B.tempR_tmp_o
                            - Cinematique_ROS_B.tempR_tmp_n,
                            Cinematique_ROS_B.tempR_tmp_i +
                            Cinematique_ROS_B.sth, Cinematique_ROS_B.v_j[2] *
                            Cinematique_ROS_B.v_j[2] * (1.0 -
          Cinematique_ROS_B.bid1) + Cinematique_ROS_B.bid1,
                            Cinematique_ROS_B.tempR);
        for (Cinematique_ROS_B.b_kstr = 0; Cinematique_ROS_B.b_kstr < 3;
             Cinematique_ROS_B.b_kstr++) {
          Cinematique_ROS_B.R[Cinematique_ROS_B.b_kstr] =
            Cinematique_ROS_B.tempR[Cinematique_ROS_B.b_kstr * 3];
          Cinematique_ROS_B.R[Cinematique_ROS_B.b_kstr + 3] =
            Cinematique_ROS_B.tempR[Cinematique_ROS_B.b_kstr * 3 + 1];
          Cinematique_ROS_B.R[Cinematique_ROS_B.b_kstr + 6] =
            Cinematique_ROS_B.tempR[Cinematique_ROS_B.b_kstr * 3 + 2];
        }

        memset(&Cinematique_ROS_B.T1j[0], 0, sizeof(real_T) << 4U);
        for (Cinematique_ROS_B.b_kstr = 0; Cinematique_ROS_B.b_kstr < 3;
             Cinematique_ROS_B.b_kstr++) {
          Cinematique_ROS_B.f = Cinematique_ROS_B.b_kstr << 2;
          Cinematique_ROS_B.T1j[Cinematique_ROS_B.f] = Cinematique_ROS_B.R[3 *
            Cinematique_ROS_B.b_kstr];
          Cinematique_ROS_B.T1j[Cinematique_ROS_B.f + 1] = Cinematique_ROS_B.R[3
            * Cinematique_ROS_B.b_kstr + 1];
          Cinematique_ROS_B.T1j[Cinematique_ROS_B.f + 2] = Cinematique_ROS_B.R[3
            * Cinematique_ROS_B.b_kstr + 2];
        }

        Cinematique_ROS_B.T1j[15] = 1.0;
        break;

       default:
        Ci_rigidBodyJoint_get_JointAxis(joint, Cinematique_ROS_B.v_j);
        memset(&Cinematique_ROS_B.tempR[0], 0, 9U * sizeof(real_T));
        Cinematique_ROS_B.tempR[0] = 1.0;
        Cinematique_ROS_B.tempR[4] = 1.0;
        Cinematique_ROS_B.tempR[8] = 1.0;
        Cinematique_ROS_B.bid1 = qv[Cinematique_ROS_B.g];
        for (Cinematique_ROS_B.b_kstr = 0; Cinematique_ROS_B.b_kstr < 3;
             Cinematique_ROS_B.b_kstr++) {
          Cinematique_ROS_B.f = Cinematique_ROS_B.b_kstr << 2;
          Cinematique_ROS_B.T1j[Cinematique_ROS_B.f] = Cinematique_ROS_B.tempR[3
            * Cinematique_ROS_B.b_kstr];
          Cinematique_ROS_B.T1j[Cinematique_ROS_B.f + 1] =
            Cinematique_ROS_B.tempR[3 * Cinematique_ROS_B.b_kstr + 1];
          Cinematique_ROS_B.T1j[Cinematique_ROS_B.f + 2] =
            Cinematique_ROS_B.tempR[3 * Cinematique_ROS_B.b_kstr + 2];
          Cinematique_ROS_B.T1j[Cinematique_ROS_B.b_kstr + 12] =
            Cinematique_ROS_B.v_j[Cinematique_ROS_B.b_kstr] *
            Cinematique_ROS_B.bid1;
        }

        Cinematique_ROS_B.T1j[3] = 0.0;
        Cinematique_ROS_B.T1j[7] = 0.0;
        Cinematique_ROS_B.T1j[11] = 0.0;
        Cinematique_ROS_B.T1j[15] = 1.0;
        break;
      }

      for (Cinematique_ROS_B.b_kstr = 0; Cinematique_ROS_B.b_kstr < 16;
           Cinematique_ROS_B.b_kstr++) {
        Cinematique_ROS_B.b[Cinematique_ROS_B.b_kstr] =
          joint->ChildToJointTransform[Cinematique_ROS_B.b_kstr];
      }

      for (Cinematique_ROS_B.b_kstr = 0; Cinematique_ROS_B.b_kstr < 4;
           Cinematique_ROS_B.b_kstr++) {
        for (Cinematique_ROS_B.g = 0; Cinematique_ROS_B.g < 4;
             Cinematique_ROS_B.g++) {
          Cinematique_ROS_B.f = Cinematique_ROS_B.g << 2;
          Cinematique_ROS_B.loop_ub_px = Cinematique_ROS_B.b_kstr +
            Cinematique_ROS_B.f;
          Cinematique_ROS_B.Tj_k[Cinematique_ROS_B.loop_ub_px] = 0.0;
          Cinematique_ROS_B.Tj_k[Cinematique_ROS_B.loop_ub_px] +=
            Cinematique_ROS_B.T1j[Cinematique_ROS_B.f] *
            Cinematique_ROS_B.Tj_c[Cinematique_ROS_B.b_kstr];
          Cinematique_ROS_B.Tj_k[Cinematique_ROS_B.loop_ub_px] +=
            Cinematique_ROS_B.T1j[Cinematique_ROS_B.f + 1] *
            Cinematique_ROS_B.Tj_c[Cinematique_ROS_B.b_kstr + 4];
          Cinematique_ROS_B.Tj_k[Cinematique_ROS_B.loop_ub_px] +=
            Cinematique_ROS_B.T1j[Cinematique_ROS_B.f + 2] *
            Cinematique_ROS_B.Tj_c[Cinematique_ROS_B.b_kstr + 8];
          Cinematique_ROS_B.Tj_k[Cinematique_ROS_B.loop_ub_px] +=
            Cinematique_ROS_B.T1j[Cinematique_ROS_B.f + 3] *
            Cinematique_ROS_B.Tj_c[Cinematique_ROS_B.b_kstr + 12];
        }

        for (Cinematique_ROS_B.g = 0; Cinematique_ROS_B.g < 4;
             Cinematique_ROS_B.g++) {
          Cinematique_ROS_B.f = Cinematique_ROS_B.g << 2;
          Cinematique_ROS_B.loop_ub_px = Cinematique_ROS_B.b_kstr +
            Cinematique_ROS_B.f;
          Cinematique_ROS_B.Tc2p[Cinematique_ROS_B.loop_ub_px] = 0.0;
          Cinematique_ROS_B.Tc2p[Cinematique_ROS_B.loop_ub_px] +=
            Cinematique_ROS_B.b[Cinematique_ROS_B.f] *
            Cinematique_ROS_B.Tj_k[Cinematique_ROS_B.b_kstr];
          Cinematique_ROS_B.Tc2p[Cinematique_ROS_B.loop_ub_px] +=
            Cinematique_ROS_B.b[Cinematique_ROS_B.f + 1] *
            Cinematique_ROS_B.Tj_k[Cinematique_ROS_B.b_kstr + 4];
          Cinematique_ROS_B.Tc2p[Cinematique_ROS_B.loop_ub_px] +=
            Cinematique_ROS_B.b[Cinematique_ROS_B.f + 2] *
            Cinematique_ROS_B.Tj_k[Cinematique_ROS_B.b_kstr + 8];
          Cinematique_ROS_B.Tc2p[Cinematique_ROS_B.loop_ub_px] +=
            Cinematique_ROS_B.b[Cinematique_ROS_B.f + 3] *
            Cinematique_ROS_B.Tj_k[Cinematique_ROS_B.b_kstr + 12];
        }
      }

      Cinematique_ROS_B.b_kstr = static_cast<int32_T>(body2->Index);
      Cinematique_ROS_B.bid1 = obj->VelocityDoFMap[Cinematique_ROS_B.b_kstr - 1];
      Cinematique_ROS_B.bid2 = obj->VelocityDoFMap[Cinematique_ROS_B.b_kstr + 4];
      if (Cinematique_ROS_B.nextBodyIsParent) {
        for (Cinematique_ROS_B.b_kstr = 0; Cinematique_ROS_B.b_kstr < 16;
             Cinematique_ROS_B.b_kstr++) {
          Cinematique_ROS_B.Tj_c[Cinematique_ROS_B.b_kstr] =
            joint->ChildToJointTransform[Cinematique_ROS_B.b_kstr];
        }
      } else {
        for (Cinematique_ROS_B.b_kstr = 0; Cinematique_ROS_B.b_kstr < 16;
             Cinematique_ROS_B.b_kstr++) {
          Cinematique_ROS_B.T1j[Cinematique_ROS_B.b_kstr] =
            joint->JointToParentTransform[Cinematique_ROS_B.b_kstr];
        }

        for (Cinematique_ROS_B.b_kstr = 0; Cinematique_ROS_B.b_kstr < 3;
             Cinematique_ROS_B.b_kstr++) {
          Cinematique_ROS_B.R[3 * Cinematique_ROS_B.b_kstr] =
            Cinematique_ROS_B.T1j[Cinematique_ROS_B.b_kstr];
          Cinematique_ROS_B.R[3 * Cinematique_ROS_B.b_kstr + 1] =
            Cinematique_ROS_B.T1j[Cinematique_ROS_B.b_kstr + 4];
          Cinematique_ROS_B.R[3 * Cinematique_ROS_B.b_kstr + 2] =
            Cinematique_ROS_B.T1j[Cinematique_ROS_B.b_kstr + 8];
        }

        for (Cinematique_ROS_B.b_kstr = 0; Cinematique_ROS_B.b_kstr < 9;
             Cinematique_ROS_B.b_kstr++) {
          Cinematique_ROS_B.tempR[Cinematique_ROS_B.b_kstr] =
            -Cinematique_ROS_B.R[Cinematique_ROS_B.b_kstr];
        }

        for (Cinematique_ROS_B.b_kstr = 0; Cinematique_ROS_B.b_kstr < 3;
             Cinematique_ROS_B.b_kstr++) {
          Cinematique_ROS_B.loop_ub_px = Cinematique_ROS_B.b_kstr << 2;
          Cinematique_ROS_B.Tj_c[Cinematique_ROS_B.loop_ub_px] =
            Cinematique_ROS_B.R[3 * Cinematique_ROS_B.b_kstr];
          Cinematique_ROS_B.Tj_c[Cinematique_ROS_B.loop_ub_px + 1] =
            Cinematique_ROS_B.R[3 * Cinematique_ROS_B.b_kstr + 1];
          Cinematique_ROS_B.Tj_c[Cinematique_ROS_B.loop_ub_px + 2] =
            Cinematique_ROS_B.R[3 * Cinematique_ROS_B.b_kstr + 2];
          Cinematique_ROS_B.Tj_c[Cinematique_ROS_B.b_kstr + 12] =
            Cinematique_ROS_B.tempR[Cinematique_ROS_B.b_kstr + 6] *
            Cinematique_ROS_B.T1j[14] +
            (Cinematique_ROS_B.tempR[Cinematique_ROS_B.b_kstr + 3] *
             Cinematique_ROS_B.T1j[13] +
             Cinematique_ROS_B.tempR[Cinematique_ROS_B.b_kstr] *
             Cinematique_ROS_B.T1j[12]);
        }

        Cinematique_ROS_B.Tj_c[3] = 0.0;
        Cinematique_ROS_B.Tj_c[7] = 0.0;
        Cinematique_ROS_B.Tj_c[11] = 0.0;
        Cinematique_ROS_B.Tj_c[15] = 1.0;
      }

      for (Cinematique_ROS_B.b_kstr = 0; Cinematique_ROS_B.b_kstr < 4;
           Cinematique_ROS_B.b_kstr++) {
        for (Cinematique_ROS_B.g = 0; Cinematique_ROS_B.g < 4;
             Cinematique_ROS_B.g++) {
          Cinematique_ROS_B.loop_ub_px = Cinematique_ROS_B.g << 2;
          Cinematique_ROS_B.f = Cinematique_ROS_B.b_kstr +
            Cinematique_ROS_B.loop_ub_px;
          Cinematique_ROS_B.T1j[Cinematique_ROS_B.f] = 0.0;
          Cinematique_ROS_B.T1j[Cinematique_ROS_B.f] +=
            Cinematique_ROS_B.T1[Cinematique_ROS_B.loop_ub_px] *
            Cinematique_ROS_B.Tj_c[Cinematique_ROS_B.b_kstr];
          Cinematique_ROS_B.T1j[Cinematique_ROS_B.f] +=
            Cinematique_ROS_B.T1[Cinematique_ROS_B.loop_ub_px + 1] *
            Cinematique_ROS_B.Tj_c[Cinematique_ROS_B.b_kstr + 4];
          Cinematique_ROS_B.T1j[Cinematique_ROS_B.f] +=
            Cinematique_ROS_B.T1[Cinematique_ROS_B.loop_ub_px + 2] *
            Cinematique_ROS_B.Tj_c[Cinematique_ROS_B.b_kstr + 8];
          Cinematique_ROS_B.T1j[Cinematique_ROS_B.f] +=
            Cinematique_ROS_B.T1[Cinematique_ROS_B.loop_ub_px + 3] *
            Cinematique_ROS_B.Tj_c[Cinematique_ROS_B.b_kstr + 12];
        }
      }

      for (Cinematique_ROS_B.b_kstr = 0; Cinematique_ROS_B.b_kstr < 3;
           Cinematique_ROS_B.b_kstr++) {
        Cinematique_ROS_B.R[3 * Cinematique_ROS_B.b_kstr] =
          Cinematique_ROS_B.T1j[Cinematique_ROS_B.b_kstr];
        Cinematique_ROS_B.R[3 * Cinematique_ROS_B.b_kstr + 1] =
          Cinematique_ROS_B.T1j[Cinematique_ROS_B.b_kstr + 4];
        Cinematique_ROS_B.R[3 * Cinematique_ROS_B.b_kstr + 2] =
          Cinematique_ROS_B.T1j[Cinematique_ROS_B.b_kstr + 8];
      }

      for (Cinematique_ROS_B.b_kstr = 0; Cinematique_ROS_B.b_kstr < 9;
           Cinematique_ROS_B.b_kstr++) {
        Cinematique_ROS_B.tempR[Cinematique_ROS_B.b_kstr] =
          -Cinematique_ROS_B.R[Cinematique_ROS_B.b_kstr];
      }

      for (Cinematique_ROS_B.b_kstr = 0; Cinematique_ROS_B.b_kstr < 3;
           Cinematique_ROS_B.b_kstr++) {
        Cinematique_ROS_B.loop_ub_px = Cinematique_ROS_B.b_kstr << 2;
        Cinematique_ROS_B.Tj_c[Cinematique_ROS_B.loop_ub_px] =
          Cinematique_ROS_B.R[3 * Cinematique_ROS_B.b_kstr];
        Cinematique_ROS_B.Tj_c[Cinematique_ROS_B.loop_ub_px + 1] =
          Cinematique_ROS_B.R[3 * Cinematique_ROS_B.b_kstr + 1];
        Cinematique_ROS_B.Tj_c[Cinematique_ROS_B.loop_ub_px + 2] =
          Cinematique_ROS_B.R[3 * Cinematique_ROS_B.b_kstr + 2];
        Cinematique_ROS_B.Tj_c[Cinematique_ROS_B.b_kstr + 12] =
          Cinematique_ROS_B.tempR[Cinematique_ROS_B.b_kstr + 6] *
          Cinematique_ROS_B.T1j[14] +
          (Cinematique_ROS_B.tempR[Cinematique_ROS_B.b_kstr + 3] *
           Cinematique_ROS_B.T1j[13] +
           Cinematique_ROS_B.tempR[Cinematique_ROS_B.b_kstr] *
           Cinematique_ROS_B.T1j[12]);
      }

      Cinematique_ROS_B.Tj_c[3] = 0.0;
      Cinematique_ROS_B.Tj_c[7] = 0.0;
      Cinematique_ROS_B.Tj_c[11] = 0.0;
      Cinematique_ROS_B.Tj_c[15] = 1.0;
      Cinematique_ROS_B.b_kstr = b->size[0] * b->size[1];
      b->size[0] = 6;
      b->size[1] = joint->MotionSubspace->size[1];
      Cinema_emxEnsureCapacity_real_T(b, Cinematique_ROS_B.b_kstr);
      Cinematique_ROS_B.loop_ub_px = joint->MotionSubspace->size[0] *
        joint->MotionSubspace->size[1] - 1;
      for (Cinematique_ROS_B.b_kstr = 0; Cinematique_ROS_B.b_kstr <=
           Cinematique_ROS_B.loop_ub_px; Cinematique_ROS_B.b_kstr++) {
        b->data[Cinematique_ROS_B.b_kstr] = joint->MotionSubspace->
          data[Cinematique_ROS_B.b_kstr];
      }

      if (Cinematique_ROS_B.bid1 > Cinematique_ROS_B.bid2) {
        Cinematique_ROS_B.f = 0;
      } else {
        Cinematique_ROS_B.f = static_cast<int32_T>(Cinematique_ROS_B.bid1) - 1;
      }

      Cinematique_ROS_B.R[0] = 0.0;
      Cinematique_ROS_B.R[3] = -Cinematique_ROS_B.Tj_c[14];
      Cinematique_ROS_B.R[6] = Cinematique_ROS_B.Tj_c[13];
      Cinematique_ROS_B.R[1] = Cinematique_ROS_B.Tj_c[14];
      Cinematique_ROS_B.R[4] = 0.0;
      Cinematique_ROS_B.R[7] = -Cinematique_ROS_B.Tj_c[12];
      Cinematique_ROS_B.R[2] = -Cinematique_ROS_B.Tj_c[13];
      Cinematique_ROS_B.R[5] = Cinematique_ROS_B.Tj_c[12];
      Cinematique_ROS_B.R[8] = 0.0;
      for (Cinematique_ROS_B.b_kstr = 0; Cinematique_ROS_B.b_kstr < 3;
           Cinematique_ROS_B.b_kstr++) {
        for (Cinematique_ROS_B.g = 0; Cinematique_ROS_B.g < 3;
             Cinematique_ROS_B.g++) {
          Cinematique_ROS_B.loop_ub_px = Cinematique_ROS_B.b_kstr + 3 *
            Cinematique_ROS_B.g;
          Cinematique_ROS_B.tempR[Cinematique_ROS_B.loop_ub_px] = 0.0;
          Cinematique_ROS_B.i_p = Cinematique_ROS_B.g << 2;
          Cinematique_ROS_B.tempR[Cinematique_ROS_B.loop_ub_px] +=
            Cinematique_ROS_B.Tj_c[Cinematique_ROS_B.i_p] *
            Cinematique_ROS_B.R[Cinematique_ROS_B.b_kstr];
          Cinematique_ROS_B.tempR[Cinematique_ROS_B.loop_ub_px] +=
            Cinematique_ROS_B.Tj_c[Cinematique_ROS_B.i_p + 1] *
            Cinematique_ROS_B.R[Cinematique_ROS_B.b_kstr + 3];
          Cinematique_ROS_B.tempR[Cinematique_ROS_B.loop_ub_px] +=
            Cinematique_ROS_B.Tj_c[Cinematique_ROS_B.i_p + 2] *
            Cinematique_ROS_B.R[Cinematique_ROS_B.b_kstr + 6];
          Cinematique_ROS_B.Tj[Cinematique_ROS_B.g + 6 *
            Cinematique_ROS_B.b_kstr] = Cinematique_ROS_B.Tj_c
            [(Cinematique_ROS_B.b_kstr << 2) + Cinematique_ROS_B.g];
          Cinematique_ROS_B.Tj[Cinematique_ROS_B.g + 6 *
            (Cinematique_ROS_B.b_kstr + 3)] = 0.0;
        }
      }

      for (Cinematique_ROS_B.b_kstr = 0; Cinematique_ROS_B.b_kstr < 3;
           Cinematique_ROS_B.b_kstr++) {
        Cinematique_ROS_B.Tj[6 * Cinematique_ROS_B.b_kstr + 3] =
          Cinematique_ROS_B.tempR[3 * Cinematique_ROS_B.b_kstr];
        Cinematique_ROS_B.loop_ub_px = Cinematique_ROS_B.b_kstr << 2;
        Cinematique_ROS_B.g = 6 * (Cinematique_ROS_B.b_kstr + 3);
        Cinematique_ROS_B.Tj[Cinematique_ROS_B.g + 3] =
          Cinematique_ROS_B.Tj_c[Cinematique_ROS_B.loop_ub_px];
        Cinematique_ROS_B.Tj[6 * Cinematique_ROS_B.b_kstr + 4] =
          Cinematique_ROS_B.tempR[3 * Cinematique_ROS_B.b_kstr + 1];
        Cinematique_ROS_B.Tj[Cinematique_ROS_B.g + 4] =
          Cinematique_ROS_B.Tj_c[Cinematique_ROS_B.loop_ub_px + 1];
        Cinematique_ROS_B.Tj[6 * Cinematique_ROS_B.b_kstr + 5] =
          Cinematique_ROS_B.tempR[3 * Cinematique_ROS_B.b_kstr + 2];
        Cinematique_ROS_B.Tj[Cinematique_ROS_B.g + 5] =
          Cinematique_ROS_B.Tj_c[Cinematique_ROS_B.loop_ub_px + 2];
      }

      Cinematique_ROS_mtimes(Cinematique_ROS_B.Tj, b, tmp);
      Cinematique_ROS_B.loop_ub_px = tmp->size[1];
      for (Cinematique_ROS_B.b_kstr = 0; Cinematique_ROS_B.b_kstr <
           Cinematique_ROS_B.loop_ub_px; Cinematique_ROS_B.b_kstr++) {
        for (Cinematique_ROS_B.g = 0; Cinematique_ROS_B.g < 6;
             Cinematique_ROS_B.g++) {
          Jac->data[Cinematique_ROS_B.g + 6 * (Cinematique_ROS_B.f +
            Cinematique_ROS_B.b_kstr)] = tmp->data[6 * Cinematique_ROS_B.b_kstr
            + Cinematique_ROS_B.g] * static_cast<real_T>
            (Cinematique_ROS_B.jointSign);
        }
      }
    }

    if (Cinematique_ROS_B.nextBodyIsParent) {
      for (Cinematique_ROS_B.b_kstr = 0; Cinematique_ROS_B.b_kstr < 4;
           Cinematique_ROS_B.b_kstr++) {
        for (Cinematique_ROS_B.g = 0; Cinematique_ROS_B.g < 4;
             Cinematique_ROS_B.g++) {
          Cinematique_ROS_B.f = Cinematique_ROS_B.g << 2;
          Cinematique_ROS_B.loop_ub_px = Cinematique_ROS_B.b_kstr +
            Cinematique_ROS_B.f;
          Cinematique_ROS_B.Tj_c[Cinematique_ROS_B.loop_ub_px] = 0.0;
          Cinematique_ROS_B.Tj_c[Cinematique_ROS_B.loop_ub_px] +=
            Cinematique_ROS_B.T1[Cinematique_ROS_B.f] *
            Cinematique_ROS_B.Tc2p[Cinematique_ROS_B.b_kstr];
          Cinematique_ROS_B.Tj_c[Cinematique_ROS_B.loop_ub_px] +=
            Cinematique_ROS_B.T1[Cinematique_ROS_B.f + 1] *
            Cinematique_ROS_B.Tc2p[Cinematique_ROS_B.b_kstr + 4];
          Cinematique_ROS_B.Tj_c[Cinematique_ROS_B.loop_ub_px] +=
            Cinematique_ROS_B.T1[Cinematique_ROS_B.f + 2] *
            Cinematique_ROS_B.Tc2p[Cinematique_ROS_B.b_kstr + 8];
          Cinematique_ROS_B.Tj_c[Cinematique_ROS_B.loop_ub_px] +=
            Cinematique_ROS_B.T1[Cinematique_ROS_B.f + 3] *
            Cinematique_ROS_B.Tc2p[Cinematique_ROS_B.b_kstr + 12];
        }
      }

      memcpy(&Cinematique_ROS_B.T1[0], &Cinematique_ROS_B.Tj_c[0], sizeof(real_T)
             << 4U);
    } else {
      for (Cinematique_ROS_B.b_kstr = 0; Cinematique_ROS_B.b_kstr < 3;
           Cinematique_ROS_B.b_kstr++) {
        Cinematique_ROS_B.R[3 * Cinematique_ROS_B.b_kstr] =
          Cinematique_ROS_B.Tc2p[Cinematique_ROS_B.b_kstr];
        Cinematique_ROS_B.R[3 * Cinematique_ROS_B.b_kstr + 1] =
          Cinematique_ROS_B.Tc2p[Cinematique_ROS_B.b_kstr + 4];
        Cinematique_ROS_B.R[3 * Cinematique_ROS_B.b_kstr + 2] =
          Cinematique_ROS_B.Tc2p[Cinematique_ROS_B.b_kstr + 8];
      }

      for (Cinematique_ROS_B.b_kstr = 0; Cinematique_ROS_B.b_kstr < 9;
           Cinematique_ROS_B.b_kstr++) {
        Cinematique_ROS_B.tempR[Cinematique_ROS_B.b_kstr] =
          -Cinematique_ROS_B.R[Cinematique_ROS_B.b_kstr];
      }

      for (Cinematique_ROS_B.b_kstr = 0; Cinematique_ROS_B.b_kstr < 3;
           Cinematique_ROS_B.b_kstr++) {
        Cinematique_ROS_B.jointSign = Cinematique_ROS_B.b_kstr << 2;
        Cinematique_ROS_B.Tj_c[Cinematique_ROS_B.jointSign] =
          Cinematique_ROS_B.R[3 * Cinematique_ROS_B.b_kstr];
        Cinematique_ROS_B.Tj_c[Cinematique_ROS_B.jointSign + 1] =
          Cinematique_ROS_B.R[3 * Cinematique_ROS_B.b_kstr + 1];
        Cinematique_ROS_B.Tj_c[Cinematique_ROS_B.jointSign + 2] =
          Cinematique_ROS_B.R[3 * Cinematique_ROS_B.b_kstr + 2];
        Cinematique_ROS_B.Tj_c[Cinematique_ROS_B.b_kstr + 12] =
          Cinematique_ROS_B.tempR[Cinematique_ROS_B.b_kstr + 6] *
          Cinematique_ROS_B.Tc2p[14] +
          (Cinematique_ROS_B.tempR[Cinematique_ROS_B.b_kstr + 3] *
           Cinematique_ROS_B.Tc2p[13] +
           Cinematique_ROS_B.tempR[Cinematique_ROS_B.b_kstr] *
           Cinematique_ROS_B.Tc2p[12]);
      }

      Cinematique_ROS_B.Tj_c[3] = 0.0;
      Cinematique_ROS_B.Tj_c[7] = 0.0;
      Cinematique_ROS_B.Tj_c[11] = 0.0;
      Cinematique_ROS_B.Tj_c[15] = 1.0;
      for (Cinematique_ROS_B.b_kstr = 0; Cinematique_ROS_B.b_kstr < 4;
           Cinematique_ROS_B.b_kstr++) {
        for (Cinematique_ROS_B.g = 0; Cinematique_ROS_B.g < 4;
             Cinematique_ROS_B.g++) {
          Cinematique_ROS_B.f = Cinematique_ROS_B.g << 2;
          Cinematique_ROS_B.jointSign = Cinematique_ROS_B.b_kstr +
            Cinematique_ROS_B.f;
          Cinematique_ROS_B.Tc2p[Cinematique_ROS_B.jointSign] = 0.0;
          Cinematique_ROS_B.Tc2p[Cinematique_ROS_B.jointSign] +=
            Cinematique_ROS_B.T1[Cinematique_ROS_B.f] *
            Cinematique_ROS_B.Tj_c[Cinematique_ROS_B.b_kstr];
          Cinematique_ROS_B.Tc2p[Cinematique_ROS_B.jointSign] +=
            Cinematique_ROS_B.T1[Cinematique_ROS_B.f + 1] *
            Cinematique_ROS_B.Tj_c[Cinematique_ROS_B.b_kstr + 4];
          Cinematique_ROS_B.Tc2p[Cinematique_ROS_B.jointSign] +=
            Cinematique_ROS_B.T1[Cinematique_ROS_B.f + 2] *
            Cinematique_ROS_B.Tj_c[Cinematique_ROS_B.b_kstr + 8];
          Cinematique_ROS_B.Tc2p[Cinematique_ROS_B.jointSign] +=
            Cinematique_ROS_B.T1[Cinematique_ROS_B.f + 3] *
            Cinematique_ROS_B.Tj_c[Cinematique_ROS_B.b_kstr + 12];
        }
      }

      memcpy(&Cinematique_ROS_B.T1[0], &Cinematique_ROS_B.Tc2p[0], sizeof(real_T)
             << 4U);
    }
  }

  Cinematique_ROS_emxFree_real_T(&tmp);
  Cinematique_ROS_emxFree_real_T(&b);
  Cinematique_ROS_emxFree_char_T(&body2Name);
  Cinematique_ROS_emxFree_real_T(&kinematicPathIndices);
  for (Cinematique_ROS_B.b_kstr = 0; Cinematique_ROS_B.b_kstr < 3;
       Cinematique_ROS_B.b_kstr++) {
    Cinematique_ROS_B.b_i_p = Cinematique_ROS_B.b_kstr << 2;
    Cinematique_ROS_B.bid1 = Cinematique_ROS_B.T1[Cinematique_ROS_B.b_i_p];
    Cinematique_ROS_B.Tj[6 * Cinematique_ROS_B.b_kstr] = Cinematique_ROS_B.bid1;
    Cinematique_ROS_B.g = 6 * (Cinematique_ROS_B.b_kstr + 3);
    Cinematique_ROS_B.Tj[Cinematique_ROS_B.g] = 0.0;
    Cinematique_ROS_B.Tj[6 * Cinematique_ROS_B.b_kstr + 3] = 0.0;
    Cinematique_ROS_B.Tj[Cinematique_ROS_B.g + 3] = Cinematique_ROS_B.bid1;
    Cinematique_ROS_B.bid1 = Cinematique_ROS_B.T1[Cinematique_ROS_B.b_i_p + 1];
    Cinematique_ROS_B.Tj[6 * Cinematique_ROS_B.b_kstr + 1] =
      Cinematique_ROS_B.bid1;
    Cinematique_ROS_B.Tj[Cinematique_ROS_B.g + 1] = 0.0;
    Cinematique_ROS_B.Tj[6 * Cinematique_ROS_B.b_kstr + 4] = 0.0;
    Cinematique_ROS_B.Tj[Cinematique_ROS_B.g + 4] = Cinematique_ROS_B.bid1;
    Cinematique_ROS_B.bid1 = Cinematique_ROS_B.T1[Cinematique_ROS_B.b_i_p + 2];
    Cinematique_ROS_B.Tj[6 * Cinematique_ROS_B.b_kstr + 2] =
      Cinematique_ROS_B.bid1;
    Cinematique_ROS_B.Tj[Cinematique_ROS_B.g + 2] = 0.0;
    Cinematique_ROS_B.Tj[6 * Cinematique_ROS_B.b_kstr + 5] = 0.0;
    Cinematique_ROS_B.Tj[Cinematique_ROS_B.g + 5] = Cinematique_ROS_B.bid1;
  }

  Cinematique_ROS_emxInit_real_T(&Jac_0, 2);
  Cinematique_ROS_B.b_kstr = Jac_0->size[0] * Jac_0->size[1];
  Jac_0->size[0] = 6;
  Jac_0->size[1] = Jac->size[1];
  Cinema_emxEnsureCapacity_real_T(Jac_0, Cinematique_ROS_B.b_kstr);
  Cinematique_ROS_B.loop_ub_px = Jac->size[0] * Jac->size[1];
  for (Cinematique_ROS_B.b_kstr = 0; Cinematique_ROS_B.b_kstr <
       Cinematique_ROS_B.loop_ub_px; Cinematique_ROS_B.b_kstr++) {
    Jac_0->data[Cinematique_ROS_B.b_kstr] = Jac->data[Cinematique_ROS_B.b_kstr];
  }

  Cinematique_ROS_mtimes(Cinematique_ROS_B.Tj, Jac_0, Jac);
  T_size[0] = 4;
  T_size[1] = 4;
  Cinematique_ROS_emxFree_real_T(&Jac_0);
  memcpy(&T_data[0], &Cinematique_ROS_B.T1[0], sizeof(real_T) << 4U);
}

real_T rt_hypotd_snf(real_T u0, real_T u1)
{
  real_T a;
  real_T y;
  a = fabs(u0);
  y = fabs(u1);
  if (a < y) {
    a /= y;
    y *= sqrt(a * a + 1.0);
  } else if (a > y) {
    y /= a;
    y = sqrt(y * y + 1.0) * a;
  } else {
    if (!rtIsNaN(y)) {
      y = a * 1.4142135623730951;
    }
  }

  return y;
}

static creal_T Cinematique_ROS_sqrt(const creal_T x)
{
  creal_T b_x;
  real_T absxi;
  real_T absxr;
  if (x.im == 0.0) {
    if (x.re < 0.0) {
      absxr = 0.0;
      absxi = sqrt(-x.re);
    } else {
      absxr = sqrt(x.re);
      absxi = 0.0;
    }
  } else if (x.re == 0.0) {
    if (x.im < 0.0) {
      absxr = sqrt(-x.im / 2.0);
      absxi = -absxr;
    } else {
      absxr = sqrt(x.im / 2.0);
      absxi = absxr;
    }
  } else if (rtIsNaN(x.re)) {
    absxr = x.re;
    absxi = x.re;
  } else if (rtIsNaN(x.im)) {
    absxr = x.im;
    absxi = x.im;
  } else if (rtIsInf(x.im)) {
    absxr = fabs(x.im);
    absxi = x.im;
  } else if (rtIsInf(x.re)) {
    if (x.re < 0.0) {
      absxr = 0.0;
      absxi = x.im * -x.re;
    } else {
      absxr = x.re;
      absxi = 0.0;
    }
  } else {
    absxr = fabs(x.re);
    absxi = fabs(x.im);
    if ((absxr > 4.4942328371557893E+307) || (absxi > 4.4942328371557893E+307))
    {
      absxr *= 0.5;
      absxi *= 0.5;
      absxi = rt_hypotd_snf(absxr, absxi);
      if (absxi > absxr) {
        absxr = sqrt(absxr / absxi + 1.0) * sqrt(absxi);
      } else {
        absxr = sqrt(absxi) * 1.4142135623730951;
      }
    } else {
      absxr = sqrt((rt_hypotd_snf(absxr, absxi) + absxr) * 0.5);
    }

    if (x.re > 0.0) {
      absxi = x.im / absxr * 0.5;
    } else {
      if (x.im < 0.0) {
        absxi = -absxr;
      } else {
        absxi = absxr;
      }

      absxr = x.im / absxi * 0.5;
    }
  }

  b_x.re = absxr;
  b_x.im = absxi;
  return b_x;
}

real_T rt_atan2d_snf(real_T u0, real_T u1)
{
  real_T y;
  int32_T u0_0;
  int32_T u1_0;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = (rtNaN);
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    if (u0 > 0.0) {
      u0_0 = 1;
    } else {
      u0_0 = -1;
    }

    if (u1 > 0.0) {
      u1_0 = 1;
    } else {
      u1_0 = -1;
    }

    y = atan2(static_cast<real_T>(u0_0), static_cast<real_T>(u1_0));
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(RT_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = atan2(u0, u1);
  }

  return y;
}

static real_T Cinematique_ROS_xnrm2(int32_T n, const real_T x[9], int32_T ix0)
{
  real_T absxk;
  real_T scale;
  real_T t;
  real_T y;
  int32_T k;
  int32_T kend;
  y = 0.0;
  scale = 3.3121686421112381E-170;
  kend = ix0 + n;
  for (k = ix0; k < kend; k++) {
    absxk = fabs(x[k - 1]);
    if (absxk > scale) {
      t = scale / absxk;
      y = y * t * t + 1.0;
      scale = absxk;
    } else {
      t = absxk / scale;
      y += t * t;
    }
  }

  return scale * sqrt(y);
}

static real_T Cinematique_ROS_xdotc(int32_T n, const real_T x[9], int32_T ix0,
  const real_T y[9], int32_T iy0)
{
  real_T d;
  int32_T ix;
  int32_T iy;
  int32_T k;
  d = 0.0;
  ix = ix0 - 1;
  iy = iy0 - 1;
  for (k = 0; k < n; k++) {
    d += x[ix] * y[iy];
    ix++;
    iy++;
  }

  return d;
}

static void Cinematique_ROS_xaxpy(int32_T n, real_T a, int32_T ix0, const real_T
  y[9], int32_T iy0, real_T b_y[9])
{
  int32_T ix;
  int32_T iy;
  int32_T k;
  memcpy(&b_y[0], &y[0], 9U * sizeof(real_T));
  if (!(a == 0.0)) {
    ix = ix0;
    iy = iy0 - 1;
    for (k = 0; k < n; k++) {
      b_y[iy] += b_y[ix - 1] * a;
      ix++;
      iy++;
    }
  }
}

static real_T Cinematique_ROS_xnrm2_k(const real_T x[3], int32_T ix0)
{
  real_T absxk;
  real_T scale;
  real_T t;
  real_T y;
  int32_T k;
  y = 0.0;
  scale = 3.3121686421112381E-170;
  for (k = ix0; k <= ix0 + 1; k++) {
    absxk = fabs(x[k - 1]);
    if (absxk > scale) {
      t = scale / absxk;
      y = y * t * t + 1.0;
      scale = absxk;
    } else {
      t = absxk / scale;
      y += t * t;
    }
  }

  return scale * sqrt(y);
}

static void Cinematique_ROS_xaxpy_k1e(int32_T n, real_T a, const real_T x[9],
  int32_T ix0, real_T y[3], int32_T iy0)
{
  int32_T ix;
  int32_T iy;
  int32_T k;
  if (!(a == 0.0)) {
    ix = ix0;
    iy = iy0 - 1;
    for (k = 0; k < n; k++) {
      y[iy] += x[ix - 1] * a;
      ix++;
      iy++;
    }
  }
}

static void Cinematique_ROS_xaxpy_k1(int32_T n, real_T a, const real_T x[3],
  int32_T ix0, const real_T y[9], int32_T iy0, real_T b_y[9])
{
  int32_T ix;
  int32_T iy;
  int32_T k;
  memcpy(&b_y[0], &y[0], 9U * sizeof(real_T));
  if (!(a == 0.0)) {
    ix = ix0;
    iy = iy0 - 1;
    for (k = 0; k < n; k++) {
      b_y[iy] += x[ix - 1] * a;
      ix++;
      iy++;
    }
  }
}

static void Cinematique_ROS_xswap(const real_T x[9], int32_T ix0, int32_T iy0,
  real_T b_x[9])
{
  real_T temp;
  memcpy(&b_x[0], &x[0], 9U * sizeof(real_T));
  temp = b_x[ix0 - 1];
  b_x[ix0 - 1] = b_x[iy0 - 1];
  b_x[iy0 - 1] = temp;
  temp = b_x[ix0];
  b_x[ix0] = b_x[iy0];
  b_x[iy0] = temp;
  temp = b_x[ix0 + 1];
  b_x[ix0 + 1] = b_x[iy0 + 1];
  b_x[iy0 + 1] = temp;
}

static void Cinematique_ROS_xrotg(real_T a, real_T b, real_T *b_a, real_T *b_b,
  real_T *c, real_T *s)
{
  Cinematique_ROS_B.roe = b;
  Cinematique_ROS_B.absa = fabs(a);
  Cinematique_ROS_B.absb = fabs(b);
  if (Cinematique_ROS_B.absa > Cinematique_ROS_B.absb) {
    Cinematique_ROS_B.roe = a;
  }

  Cinematique_ROS_B.scale = Cinematique_ROS_B.absa + Cinematique_ROS_B.absb;
  if (Cinematique_ROS_B.scale == 0.0) {
    *s = 0.0;
    *c = 1.0;
    *b_a = 0.0;
    *b_b = 0.0;
  } else {
    Cinematique_ROS_B.ads = Cinematique_ROS_B.absa / Cinematique_ROS_B.scale;
    Cinematique_ROS_B.bds = Cinematique_ROS_B.absb / Cinematique_ROS_B.scale;
    *b_a = sqrt(Cinematique_ROS_B.ads * Cinematique_ROS_B.ads +
                Cinematique_ROS_B.bds * Cinematique_ROS_B.bds) *
      Cinematique_ROS_B.scale;
    if (Cinematique_ROS_B.roe < 0.0) {
      *b_a = -*b_a;
    }

    *c = a / *b_a;
    *s = b / *b_a;
    if (Cinematique_ROS_B.absa > Cinematique_ROS_B.absb) {
      *b_b = *s;
    } else if (*c != 0.0) {
      *b_b = 1.0 / *c;
    } else {
      *b_b = 1.0;
    }
  }
}

static void Cinematique_ROS_xrot(const real_T x[9], int32_T ix0, int32_T iy0,
  real_T c, real_T s, real_T b_x[9])
{
  real_T temp;
  real_T temp_tmp;
  memcpy(&b_x[0], &x[0], 9U * sizeof(real_T));
  temp = b_x[iy0 - 1];
  temp_tmp = b_x[ix0 - 1];
  b_x[iy0 - 1] = temp * c - temp_tmp * s;
  b_x[ix0 - 1] = temp_tmp * c + temp * s;
  temp = b_x[ix0] * c + b_x[iy0] * s;
  b_x[iy0] = b_x[iy0] * c - b_x[ix0] * s;
  b_x[ix0] = temp;
  temp = b_x[iy0 + 1];
  temp_tmp = b_x[ix0 + 1];
  b_x[iy0 + 1] = temp * c - temp_tmp * s;
  b_x[ix0 + 1] = temp_tmp * c + temp * s;
}

static void Cinematique_ROS_svd(const real_T A[9], real_T U[9], real_T s[3],
  real_T V[9])
{
  boolean_T exitg1;
  Cinematique_ROS_B.e_d[0] = 0.0;
  Cinematique_ROS_B.work[0] = 0.0;
  Cinematique_ROS_B.e_d[1] = 0.0;
  Cinematique_ROS_B.work[1] = 0.0;
  Cinematique_ROS_B.e_d[2] = 0.0;
  Cinematique_ROS_B.work[2] = 0.0;
  for (Cinematique_ROS_B.m_l = 0; Cinematique_ROS_B.m_l < 9;
       Cinematique_ROS_B.m_l++) {
    Cinematique_ROS_B.A[Cinematique_ROS_B.m_l] = A[Cinematique_ROS_B.m_l];
    U[Cinematique_ROS_B.m_l] = 0.0;
    V[Cinematique_ROS_B.m_l] = 0.0;
  }

  Cinematique_ROS_B.apply_transform = false;
  Cinematique_ROS_B.nrm = Cinematique_ROS_xnrm2(3, Cinematique_ROS_B.A, 1);
  if (Cinematique_ROS_B.nrm > 0.0) {
    Cinematique_ROS_B.apply_transform = true;
    if (Cinematique_ROS_B.A[0] < 0.0) {
      Cinematique_ROS_B.s[0] = -Cinematique_ROS_B.nrm;
    } else {
      Cinematique_ROS_B.s[0] = Cinematique_ROS_B.nrm;
    }

    if (fabs(Cinematique_ROS_B.s[0]) >= 1.0020841800044864E-292) {
      Cinematique_ROS_B.nrm = 1.0 / Cinematique_ROS_B.s[0];
      for (Cinematique_ROS_B.qq = 1; Cinematique_ROS_B.qq < 4;
           Cinematique_ROS_B.qq++) {
        Cinematique_ROS_B.A[Cinematique_ROS_B.qq - 1] *= Cinematique_ROS_B.nrm;
      }
    } else {
      for (Cinematique_ROS_B.qq = 1; Cinematique_ROS_B.qq < 4;
           Cinematique_ROS_B.qq++) {
        Cinematique_ROS_B.A[Cinematique_ROS_B.qq - 1] /= Cinematique_ROS_B.s[0];
      }
    }

    Cinematique_ROS_B.A[0]++;
    Cinematique_ROS_B.s[0] = -Cinematique_ROS_B.s[0];
  } else {
    Cinematique_ROS_B.s[0] = 0.0;
  }

  for (Cinematique_ROS_B.m_l = 2; Cinematique_ROS_B.m_l < 4;
       Cinematique_ROS_B.m_l++) {
    Cinematique_ROS_B.qjj = (Cinematique_ROS_B.m_l - 1) * 3 + 1;
    if (Cinematique_ROS_B.apply_transform) {
      memcpy(&Cinematique_ROS_B.A_f[0], &Cinematique_ROS_B.A[0], 9U * sizeof
             (real_T));
      Cinematique_ROS_xaxpy(3, -(Cinematique_ROS_xdotc(3, Cinematique_ROS_B.A, 1,
        Cinematique_ROS_B.A, Cinematique_ROS_B.qjj) / Cinematique_ROS_B.A[0]), 1,
                            Cinematique_ROS_B.A_f, Cinematique_ROS_B.qjj,
                            Cinematique_ROS_B.A);
    }

    Cinematique_ROS_B.e_d[Cinematique_ROS_B.m_l - 1] =
      Cinematique_ROS_B.A[Cinematique_ROS_B.qjj - 1];
  }

  for (Cinematique_ROS_B.m_l = 1; Cinematique_ROS_B.m_l < 4;
       Cinematique_ROS_B.m_l++) {
    U[Cinematique_ROS_B.m_l - 1] = Cinematique_ROS_B.A[Cinematique_ROS_B.m_l - 1];
  }

  Cinematique_ROS_B.nrm = Cinematique_ROS_xnrm2_k(Cinematique_ROS_B.e_d, 2);
  if (Cinematique_ROS_B.nrm == 0.0) {
    Cinematique_ROS_B.e_d[0] = 0.0;
  } else {
    if (Cinematique_ROS_B.e_d[1] < 0.0) {
      Cinematique_ROS_B.rt = -Cinematique_ROS_B.nrm;
      Cinematique_ROS_B.e_d[0] = -Cinematique_ROS_B.nrm;
    } else {
      Cinematique_ROS_B.rt = Cinematique_ROS_B.nrm;
      Cinematique_ROS_B.e_d[0] = Cinematique_ROS_B.nrm;
    }

    if (fabs(Cinematique_ROS_B.rt) >= 1.0020841800044864E-292) {
      Cinematique_ROS_B.nrm = 1.0 / Cinematique_ROS_B.rt;
      for (Cinematique_ROS_B.qq = 2; Cinematique_ROS_B.qq < 4;
           Cinematique_ROS_B.qq++) {
        Cinematique_ROS_B.e_d[Cinematique_ROS_B.qq - 1] *= Cinematique_ROS_B.nrm;
      }
    } else {
      for (Cinematique_ROS_B.qq = 2; Cinematique_ROS_B.qq < 4;
           Cinematique_ROS_B.qq++) {
        Cinematique_ROS_B.e_d[Cinematique_ROS_B.qq - 1] /= Cinematique_ROS_B.rt;
      }
    }

    Cinematique_ROS_B.e_d[1]++;
    Cinematique_ROS_B.e_d[0] = -Cinematique_ROS_B.e_d[0];
    for (Cinematique_ROS_B.m_l = 2; Cinematique_ROS_B.m_l < 4;
         Cinematique_ROS_B.m_l++) {
      Cinematique_ROS_B.work[Cinematique_ROS_B.m_l - 1] = 0.0;
    }

    for (Cinematique_ROS_B.m_l = 2; Cinematique_ROS_B.m_l < 4;
         Cinematique_ROS_B.m_l++) {
      Cinematique_ROS_xaxpy_k1e(2, Cinematique_ROS_B.e_d[Cinematique_ROS_B.m_l -
        1], Cinematique_ROS_B.A, 3 * (Cinematique_ROS_B.m_l - 1) + 2,
        Cinematique_ROS_B.work, 2);
    }

    for (Cinematique_ROS_B.m_l = 2; Cinematique_ROS_B.m_l < 4;
         Cinematique_ROS_B.m_l++) {
      memcpy(&Cinematique_ROS_B.A_f[0], &Cinematique_ROS_B.A[0], 9U * sizeof
             (real_T));
      Cinematique_ROS_xaxpy_k1(2, -Cinematique_ROS_B.e_d[Cinematique_ROS_B.m_l -
        1] / Cinematique_ROS_B.e_d[1], Cinematique_ROS_B.work, 2,
        Cinematique_ROS_B.A_f, (Cinematique_ROS_B.m_l - 1) * 3 + 2,
        Cinematique_ROS_B.A);
    }
  }

  for (Cinematique_ROS_B.m_l = 2; Cinematique_ROS_B.m_l < 4;
       Cinematique_ROS_B.m_l++) {
    V[Cinematique_ROS_B.m_l - 1] = Cinematique_ROS_B.e_d[Cinematique_ROS_B.m_l -
      1];
  }

  Cinematique_ROS_B.apply_transform = false;
  Cinematique_ROS_B.nrm = Cinematique_ROS_xnrm2(2, Cinematique_ROS_B.A, 5);
  if (Cinematique_ROS_B.nrm > 0.0) {
    Cinematique_ROS_B.apply_transform = true;
    if (Cinematique_ROS_B.A[4] < 0.0) {
      Cinematique_ROS_B.s[1] = -Cinematique_ROS_B.nrm;
    } else {
      Cinematique_ROS_B.s[1] = Cinematique_ROS_B.nrm;
    }

    if (fabs(Cinematique_ROS_B.s[1]) >= 1.0020841800044864E-292) {
      Cinematique_ROS_B.nrm = 1.0 / Cinematique_ROS_B.s[1];
      for (Cinematique_ROS_B.qq = 5; Cinematique_ROS_B.qq < 7;
           Cinematique_ROS_B.qq++) {
        Cinematique_ROS_B.A[Cinematique_ROS_B.qq - 1] *= Cinematique_ROS_B.nrm;
      }
    } else {
      for (Cinematique_ROS_B.qq = 5; Cinematique_ROS_B.qq < 7;
           Cinematique_ROS_B.qq++) {
        Cinematique_ROS_B.A[Cinematique_ROS_B.qq - 1] /= Cinematique_ROS_B.s[1];
      }
    }

    Cinematique_ROS_B.A[4]++;
    Cinematique_ROS_B.s[1] = -Cinematique_ROS_B.s[1];
  } else {
    Cinematique_ROS_B.s[1] = 0.0;
  }

  if (Cinematique_ROS_B.apply_transform) {
    for (Cinematique_ROS_B.m_l = 3; Cinematique_ROS_B.m_l < 4;
         Cinematique_ROS_B.m_l++) {
      memcpy(&Cinematique_ROS_B.A_f[0], &Cinematique_ROS_B.A[0], 9U * sizeof
             (real_T));
      Cinematique_ROS_xaxpy(2, -(Cinematique_ROS_xdotc(2, Cinematique_ROS_B.A, 5,
        Cinematique_ROS_B.A, 8) / Cinematique_ROS_B.A[4]), 5,
                            Cinematique_ROS_B.A_f, 8, Cinematique_ROS_B.A);
    }
  }

  for (Cinematique_ROS_B.m_l = 2; Cinematique_ROS_B.m_l < 4;
       Cinematique_ROS_B.m_l++) {
    U[Cinematique_ROS_B.m_l + 2] = Cinematique_ROS_B.A[Cinematique_ROS_B.m_l + 2];
  }

  Cinematique_ROS_B.m_l = 2;
  Cinematique_ROS_B.s[2] = Cinematique_ROS_B.A[8];
  Cinematique_ROS_B.e_d[1] = Cinematique_ROS_B.A[7];
  Cinematique_ROS_B.e_d[2] = 0.0;
  U[6] = 0.0;
  U[7] = 0.0;
  U[8] = 1.0;
  for (Cinematique_ROS_B.c_q = 1; Cinematique_ROS_B.c_q >= 0;
       Cinematique_ROS_B.c_q--) {
    Cinematique_ROS_B.qq = 3 * Cinematique_ROS_B.c_q + Cinematique_ROS_B.c_q;
    if (Cinematique_ROS_B.s[Cinematique_ROS_B.c_q] != 0.0) {
      for (Cinematique_ROS_B.kase = Cinematique_ROS_B.c_q + 2;
           Cinematique_ROS_B.kase < 4; Cinematique_ROS_B.kase++) {
        Cinematique_ROS_B.qjj = ((Cinematique_ROS_B.kase - 1) * 3 +
          Cinematique_ROS_B.c_q) + 1;
        memcpy(&Cinematique_ROS_B.A[0], &U[0], 9U * sizeof(real_T));
        Cinematique_ROS_xaxpy(3 - Cinematique_ROS_B.c_q, -(Cinematique_ROS_xdotc
          (3 - Cinematique_ROS_B.c_q, U, Cinematique_ROS_B.qq + 1, U,
           Cinematique_ROS_B.qjj) / U[Cinematique_ROS_B.qq]),
                              Cinematique_ROS_B.qq + 1, Cinematique_ROS_B.A,
                              Cinematique_ROS_B.qjj, U);
      }

      for (Cinematique_ROS_B.qjj = Cinematique_ROS_B.c_q + 1;
           Cinematique_ROS_B.qjj < 4; Cinematique_ROS_B.qjj++) {
        Cinematique_ROS_B.kase = (3 * Cinematique_ROS_B.c_q +
          Cinematique_ROS_B.qjj) - 1;
        U[Cinematique_ROS_B.kase] = -U[Cinematique_ROS_B.kase];
      }

      U[Cinematique_ROS_B.qq]++;
      if (0 <= Cinematique_ROS_B.c_q - 1) {
        U[3 * Cinematique_ROS_B.c_q] = 0.0;
      }
    } else {
      U[3 * Cinematique_ROS_B.c_q] = 0.0;
      U[3 * Cinematique_ROS_B.c_q + 1] = 0.0;
      U[3 * Cinematique_ROS_B.c_q + 2] = 0.0;
      U[Cinematique_ROS_B.qq] = 1.0;
    }
  }

  for (Cinematique_ROS_B.c_q = 2; Cinematique_ROS_B.c_q >= 0;
       Cinematique_ROS_B.c_q--) {
    if ((Cinematique_ROS_B.c_q + 1 <= 1) && (Cinematique_ROS_B.e_d[0] != 0.0)) {
      memcpy(&Cinematique_ROS_B.A[0], &V[0], 9U * sizeof(real_T));
      Cinematique_ROS_xaxpy(2, -(Cinematique_ROS_xdotc(2, V, 2, V, 5) / V[1]), 2,
                            Cinematique_ROS_B.A, 5, V);
      memcpy(&Cinematique_ROS_B.A[0], &V[0], 9U * sizeof(real_T));
      Cinematique_ROS_xaxpy(2, -(Cinematique_ROS_xdotc(2, V, 2, V, 8) / V[1]), 2,
                            Cinematique_ROS_B.A, 8, V);
    }

    V[3 * Cinematique_ROS_B.c_q] = 0.0;
    V[3 * Cinematique_ROS_B.c_q + 1] = 0.0;
    V[3 * Cinematique_ROS_B.c_q + 2] = 0.0;
    V[Cinematique_ROS_B.c_q + 3 * Cinematique_ROS_B.c_q] = 1.0;
  }

  for (Cinematique_ROS_B.c_q = 0; Cinematique_ROS_B.c_q < 3;
       Cinematique_ROS_B.c_q++) {
    Cinematique_ROS_B.ztest = Cinematique_ROS_B.e_d[Cinematique_ROS_B.c_q];
    if (Cinematique_ROS_B.s[Cinematique_ROS_B.c_q] != 0.0) {
      Cinematique_ROS_B.rt = fabs(Cinematique_ROS_B.s[Cinematique_ROS_B.c_q]);
      Cinematique_ROS_B.nrm = Cinematique_ROS_B.s[Cinematique_ROS_B.c_q] /
        Cinematique_ROS_B.rt;
      Cinematique_ROS_B.s[Cinematique_ROS_B.c_q] = Cinematique_ROS_B.rt;
      if (Cinematique_ROS_B.c_q + 1 < 3) {
        Cinematique_ROS_B.ztest /= Cinematique_ROS_B.nrm;
      }

      Cinematique_ROS_B.qjj = 3 * Cinematique_ROS_B.c_q;
      for (Cinematique_ROS_B.qq = Cinematique_ROS_B.qjj + 1;
           Cinematique_ROS_B.qq <= Cinematique_ROS_B.qjj + 3;
           Cinematique_ROS_B.qq++) {
        U[Cinematique_ROS_B.qq - 1] *= Cinematique_ROS_B.nrm;
      }
    }

    if ((Cinematique_ROS_B.c_q + 1 < 3) && (Cinematique_ROS_B.ztest != 0.0)) {
      Cinematique_ROS_B.rt = fabs(Cinematique_ROS_B.ztest);
      Cinematique_ROS_B.nrm = Cinematique_ROS_B.rt / Cinematique_ROS_B.ztest;
      Cinematique_ROS_B.ztest = Cinematique_ROS_B.rt;
      Cinematique_ROS_B.s[Cinematique_ROS_B.c_q + 1] *= Cinematique_ROS_B.nrm;
      Cinematique_ROS_B.qjj = (Cinematique_ROS_B.c_q + 1) * 3;
      for (Cinematique_ROS_B.qq = Cinematique_ROS_B.qjj + 1;
           Cinematique_ROS_B.qq <= Cinematique_ROS_B.qjj + 3;
           Cinematique_ROS_B.qq++) {
        V[Cinematique_ROS_B.qq - 1] *= Cinematique_ROS_B.nrm;
      }
    }

    Cinematique_ROS_B.e_d[Cinematique_ROS_B.c_q] = Cinematique_ROS_B.ztest;
  }

  Cinematique_ROS_B.qq = 0;
  Cinematique_ROS_B.nrm = 0.0;
  Cinematique_ROS_B.ztest = fabs(Cinematique_ROS_B.s[0]);
  Cinematique_ROS_B.rt = fabs(Cinematique_ROS_B.e_d[0]);
  if ((Cinematique_ROS_B.ztest > Cinematique_ROS_B.rt) || rtIsNaN
      (Cinematique_ROS_B.rt)) {
    Cinematique_ROS_B.rt = Cinematique_ROS_B.ztest;
  }

  if (!rtIsNaN(Cinematique_ROS_B.rt)) {
    Cinematique_ROS_B.nrm = Cinematique_ROS_B.rt;
  }

  Cinematique_ROS_B.ztest = fabs(Cinematique_ROS_B.s[1]);
  Cinematique_ROS_B.rt = fabs(Cinematique_ROS_B.e_d[1]);
  if ((Cinematique_ROS_B.ztest > Cinematique_ROS_B.rt) || rtIsNaN
      (Cinematique_ROS_B.rt)) {
    Cinematique_ROS_B.rt = Cinematique_ROS_B.ztest;
  }

  if ((!(Cinematique_ROS_B.nrm > Cinematique_ROS_B.rt)) && (!rtIsNaN
       (Cinematique_ROS_B.rt))) {
    Cinematique_ROS_B.nrm = Cinematique_ROS_B.rt;
  }

  Cinematique_ROS_B.ztest = fabs(Cinematique_ROS_B.s[2]);
  Cinematique_ROS_B.rt = fabs(Cinematique_ROS_B.e_d[2]);
  if ((Cinematique_ROS_B.ztest > Cinematique_ROS_B.rt) || rtIsNaN
      (Cinematique_ROS_B.rt)) {
    Cinematique_ROS_B.rt = Cinematique_ROS_B.ztest;
  }

  if ((!(Cinematique_ROS_B.nrm > Cinematique_ROS_B.rt)) && (!rtIsNaN
       (Cinematique_ROS_B.rt))) {
    Cinematique_ROS_B.nrm = Cinematique_ROS_B.rt;
  }

  while ((Cinematique_ROS_B.m_l + 1 > 0) && (!(Cinematique_ROS_B.qq >= 75))) {
    Cinematique_ROS_B.c_q = Cinematique_ROS_B.m_l;
    Cinematique_ROS_B.qjj = Cinematique_ROS_B.m_l;
    exitg1 = false;
    while ((!exitg1) && (Cinematique_ROS_B.qjj > -1)) {
      Cinematique_ROS_B.c_q = Cinematique_ROS_B.qjj;
      if (Cinematique_ROS_B.qjj == 0) {
        exitg1 = true;
      } else {
        Cinematique_ROS_B.rt = fabs(Cinematique_ROS_B.e_d[Cinematique_ROS_B.qjj
          - 1]);
        if ((Cinematique_ROS_B.rt <= (fabs
              (Cinematique_ROS_B.s[Cinematique_ROS_B.qjj - 1]) + fabs
              (Cinematique_ROS_B.s[Cinematique_ROS_B.qjj])) *
             2.2204460492503131E-16) || (Cinematique_ROS_B.rt <=
             1.0020841800044864E-292) || ((Cinematique_ROS_B.qq > 20) &&
             (Cinematique_ROS_B.rt <= 2.2204460492503131E-16 *
              Cinematique_ROS_B.nrm))) {
          Cinematique_ROS_B.e_d[Cinematique_ROS_B.qjj - 1] = 0.0;
          exitg1 = true;
        } else {
          Cinematique_ROS_B.qjj--;
        }
      }
    }

    if (Cinematique_ROS_B.c_q == Cinematique_ROS_B.m_l) {
      Cinematique_ROS_B.kase = 4;
    } else {
      Cinematique_ROS_B.qjj = Cinematique_ROS_B.m_l + 1;
      Cinematique_ROS_B.kase = Cinematique_ROS_B.m_l + 1;
      exitg1 = false;
      while ((!exitg1) && (Cinematique_ROS_B.kase >= Cinematique_ROS_B.c_q)) {
        Cinematique_ROS_B.qjj = Cinematique_ROS_B.kase;
        if (Cinematique_ROS_B.kase == Cinematique_ROS_B.c_q) {
          exitg1 = true;
        } else {
          Cinematique_ROS_B.rt = 0.0;
          if (Cinematique_ROS_B.kase < Cinematique_ROS_B.m_l + 1) {
            Cinematique_ROS_B.rt = fabs
              (Cinematique_ROS_B.e_d[Cinematique_ROS_B.kase - 1]);
          }

          if (Cinematique_ROS_B.kase > Cinematique_ROS_B.c_q + 1) {
            Cinematique_ROS_B.rt += fabs
              (Cinematique_ROS_B.e_d[Cinematique_ROS_B.kase - 2]);
          }

          Cinematique_ROS_B.ztest = fabs
            (Cinematique_ROS_B.s[Cinematique_ROS_B.kase - 1]);
          if ((Cinematique_ROS_B.ztest <= 2.2204460492503131E-16 *
               Cinematique_ROS_B.rt) || (Cinematique_ROS_B.ztest <=
               1.0020841800044864E-292)) {
            Cinematique_ROS_B.s[Cinematique_ROS_B.kase - 1] = 0.0;
            exitg1 = true;
          } else {
            Cinematique_ROS_B.kase--;
          }
        }
      }

      if (Cinematique_ROS_B.qjj == Cinematique_ROS_B.c_q) {
        Cinematique_ROS_B.kase = 3;
      } else if (Cinematique_ROS_B.m_l + 1 == Cinematique_ROS_B.qjj) {
        Cinematique_ROS_B.kase = 1;
      } else {
        Cinematique_ROS_B.kase = 2;
        Cinematique_ROS_B.c_q = Cinematique_ROS_B.qjj;
      }
    }

    switch (Cinematique_ROS_B.kase) {
     case 1:
      Cinematique_ROS_B.rt = Cinematique_ROS_B.e_d[Cinematique_ROS_B.m_l - 1];
      Cinematique_ROS_B.e_d[Cinematique_ROS_B.m_l - 1] = 0.0;
      for (Cinematique_ROS_B.qjj = Cinematique_ROS_B.m_l; Cinematique_ROS_B.qjj >=
           Cinematique_ROS_B.c_q + 1; Cinematique_ROS_B.qjj--) {
        Cinematique_ROS_B.ztest = Cinematique_ROS_B.e_d[0];
        Cinematique_ROS_xrotg(Cinematique_ROS_B.s[Cinematique_ROS_B.qjj - 1],
                              Cinematique_ROS_B.rt,
                              &Cinematique_ROS_B.s[Cinematique_ROS_B.qjj - 1],
                              &Cinematique_ROS_B.rt, &Cinematique_ROS_B.sqds,
                              &Cinematique_ROS_B.b_md);
        if (Cinematique_ROS_B.qjj > Cinematique_ROS_B.c_q + 1) {
          Cinematique_ROS_B.rt = -Cinematique_ROS_B.b_md *
            Cinematique_ROS_B.e_d[0];
          Cinematique_ROS_B.ztest = Cinematique_ROS_B.e_d[0] *
            Cinematique_ROS_B.sqds;
        }

        memcpy(&Cinematique_ROS_B.A[0], &V[0], 9U * sizeof(real_T));
        Cinematique_ROS_xrot(Cinematique_ROS_B.A, (Cinematique_ROS_B.qjj - 1) *
                             3 + 1, 3 * Cinematique_ROS_B.m_l + 1,
                             Cinematique_ROS_B.sqds, Cinematique_ROS_B.b_md, V);
        Cinematique_ROS_B.e_d[0] = Cinematique_ROS_B.ztest;
      }
      break;

     case 2:
      Cinematique_ROS_B.rt = Cinematique_ROS_B.e_d[Cinematique_ROS_B.c_q - 1];
      Cinematique_ROS_B.e_d[Cinematique_ROS_B.c_q - 1] = 0.0;
      for (Cinematique_ROS_B.qjj = Cinematique_ROS_B.c_q + 1;
           Cinematique_ROS_B.qjj <= Cinematique_ROS_B.m_l + 1;
           Cinematique_ROS_B.qjj++) {
        Cinematique_ROS_xrotg(Cinematique_ROS_B.s[Cinematique_ROS_B.qjj - 1],
                              Cinematique_ROS_B.rt,
                              &Cinematique_ROS_B.s[Cinematique_ROS_B.qjj - 1],
                              &Cinematique_ROS_B.ztest, &Cinematique_ROS_B.sqds,
                              &Cinematique_ROS_B.b_md);
        Cinematique_ROS_B.ztest = Cinematique_ROS_B.e_d[Cinematique_ROS_B.qjj -
          1];
        Cinematique_ROS_B.rt = Cinematique_ROS_B.ztest * -Cinematique_ROS_B.b_md;
        Cinematique_ROS_B.e_d[Cinematique_ROS_B.qjj - 1] =
          Cinematique_ROS_B.ztest * Cinematique_ROS_B.sqds;
        memcpy(&Cinematique_ROS_B.A[0], &U[0], 9U * sizeof(real_T));
        Cinematique_ROS_xrot(Cinematique_ROS_B.A, (Cinematique_ROS_B.qjj - 1) *
                             3 + 1, (Cinematique_ROS_B.c_q - 1) * 3 + 1,
                             Cinematique_ROS_B.sqds, Cinematique_ROS_B.b_md, U);
      }
      break;

     case 3:
      Cinematique_ROS_B.ztest = fabs(Cinematique_ROS_B.s[Cinematique_ROS_B.m_l]);
      Cinematique_ROS_B.sqds = Cinematique_ROS_B.s[Cinematique_ROS_B.m_l - 1];
      Cinematique_ROS_B.rt = fabs(Cinematique_ROS_B.sqds);
      if ((Cinematique_ROS_B.ztest > Cinematique_ROS_B.rt) || rtIsNaN
          (Cinematique_ROS_B.rt)) {
        Cinematique_ROS_B.rt = Cinematique_ROS_B.ztest;
      }

      Cinematique_ROS_B.b_md = Cinematique_ROS_B.e_d[Cinematique_ROS_B.m_l - 1];
      Cinematique_ROS_B.ztest = fabs(Cinematique_ROS_B.b_md);
      if ((Cinematique_ROS_B.rt > Cinematique_ROS_B.ztest) || rtIsNaN
          (Cinematique_ROS_B.ztest)) {
        Cinematique_ROS_B.ztest = Cinematique_ROS_B.rt;
      }

      Cinematique_ROS_B.rt = fabs(Cinematique_ROS_B.s[Cinematique_ROS_B.c_q]);
      if ((Cinematique_ROS_B.ztest > Cinematique_ROS_B.rt) || rtIsNaN
          (Cinematique_ROS_B.rt)) {
        Cinematique_ROS_B.rt = Cinematique_ROS_B.ztest;
      }

      Cinematique_ROS_B.ztest = fabs(Cinematique_ROS_B.e_d[Cinematique_ROS_B.c_q]);
      if ((Cinematique_ROS_B.rt > Cinematique_ROS_B.ztest) || rtIsNaN
          (Cinematique_ROS_B.ztest)) {
        Cinematique_ROS_B.ztest = Cinematique_ROS_B.rt;
      }

      Cinematique_ROS_B.rt = Cinematique_ROS_B.s[Cinematique_ROS_B.m_l] /
        Cinematique_ROS_B.ztest;
      Cinematique_ROS_B.smm1 = Cinematique_ROS_B.sqds / Cinematique_ROS_B.ztest;
      Cinematique_ROS_B.emm1 = Cinematique_ROS_B.b_md / Cinematique_ROS_B.ztest;
      Cinematique_ROS_B.sqds = Cinematique_ROS_B.s[Cinematique_ROS_B.c_q] /
        Cinematique_ROS_B.ztest;
      Cinematique_ROS_B.b_md = ((Cinematique_ROS_B.smm1 + Cinematique_ROS_B.rt) *
        (Cinematique_ROS_B.smm1 - Cinematique_ROS_B.rt) + Cinematique_ROS_B.emm1
        * Cinematique_ROS_B.emm1) / 2.0;
      Cinematique_ROS_B.smm1 = Cinematique_ROS_B.rt * Cinematique_ROS_B.emm1;
      Cinematique_ROS_B.smm1 *= Cinematique_ROS_B.smm1;
      if ((Cinematique_ROS_B.b_md != 0.0) || (Cinematique_ROS_B.smm1 != 0.0)) {
        Cinematique_ROS_B.emm1 = sqrt(Cinematique_ROS_B.b_md *
          Cinematique_ROS_B.b_md + Cinematique_ROS_B.smm1);
        if (Cinematique_ROS_B.b_md < 0.0) {
          Cinematique_ROS_B.emm1 = -Cinematique_ROS_B.emm1;
        }

        Cinematique_ROS_B.emm1 = Cinematique_ROS_B.smm1 /
          (Cinematique_ROS_B.b_md + Cinematique_ROS_B.emm1);
      } else {
        Cinematique_ROS_B.emm1 = 0.0;
      }

      Cinematique_ROS_B.rt = (Cinematique_ROS_B.sqds + Cinematique_ROS_B.rt) *
        (Cinematique_ROS_B.sqds - Cinematique_ROS_B.rt) + Cinematique_ROS_B.emm1;
      Cinematique_ROS_B.sqds *= Cinematique_ROS_B.e_d[Cinematique_ROS_B.c_q] /
        Cinematique_ROS_B.ztest;
      for (Cinematique_ROS_B.d_k = Cinematique_ROS_B.c_q + 1;
           Cinematique_ROS_B.d_k <= Cinematique_ROS_B.m_l; Cinematique_ROS_B.d_k
           ++) {
        Cinematique_ROS_xrotg(Cinematique_ROS_B.rt, Cinematique_ROS_B.sqds,
                              &Cinematique_ROS_B.ztest, &Cinematique_ROS_B.emm1,
                              &Cinematique_ROS_B.b_md, &Cinematique_ROS_B.smm1);
        if (Cinematique_ROS_B.d_k > Cinematique_ROS_B.c_q + 1) {
          Cinematique_ROS_B.e_d[0] = Cinematique_ROS_B.ztest;
        }

        Cinematique_ROS_B.ztest = Cinematique_ROS_B.e_d[Cinematique_ROS_B.d_k -
          1];
        Cinematique_ROS_B.rt = Cinematique_ROS_B.s[Cinematique_ROS_B.d_k - 1];
        Cinematique_ROS_B.e_d[Cinematique_ROS_B.d_k - 1] =
          Cinematique_ROS_B.ztest * Cinematique_ROS_B.b_md -
          Cinematique_ROS_B.rt * Cinematique_ROS_B.smm1;
        Cinematique_ROS_B.sqds = Cinematique_ROS_B.smm1 *
          Cinematique_ROS_B.s[Cinematique_ROS_B.d_k];
        Cinematique_ROS_B.s[Cinematique_ROS_B.d_k] *= Cinematique_ROS_B.b_md;
        Cinematique_ROS_B.qjj = (Cinematique_ROS_B.d_k - 1) * 3 + 1;
        Cinematique_ROS_B.kase = 3 * Cinematique_ROS_B.d_k + 1;
        memcpy(&Cinematique_ROS_B.A[0], &V[0], 9U * sizeof(real_T));
        Cinematique_ROS_xrot(Cinematique_ROS_B.A, Cinematique_ROS_B.qjj,
                             Cinematique_ROS_B.kase, Cinematique_ROS_B.b_md,
                             Cinematique_ROS_B.smm1, V);
        Cinematique_ROS_xrotg(Cinematique_ROS_B.rt * Cinematique_ROS_B.b_md +
                              Cinematique_ROS_B.ztest * Cinematique_ROS_B.smm1,
                              Cinematique_ROS_B.sqds,
                              &Cinematique_ROS_B.s[Cinematique_ROS_B.d_k - 1],
                              &Cinematique_ROS_B.unusedU2,
                              &Cinematique_ROS_B.emm1, &Cinematique_ROS_B.d_sn);
        Cinematique_ROS_B.rt = Cinematique_ROS_B.e_d[Cinematique_ROS_B.d_k - 1] *
          Cinematique_ROS_B.emm1 + Cinematique_ROS_B.d_sn *
          Cinematique_ROS_B.s[Cinematique_ROS_B.d_k];
        Cinematique_ROS_B.s[Cinematique_ROS_B.d_k] =
          Cinematique_ROS_B.e_d[Cinematique_ROS_B.d_k - 1] *
          -Cinematique_ROS_B.d_sn + Cinematique_ROS_B.emm1 *
          Cinematique_ROS_B.s[Cinematique_ROS_B.d_k];
        Cinematique_ROS_B.sqds = Cinematique_ROS_B.d_sn *
          Cinematique_ROS_B.e_d[Cinematique_ROS_B.d_k];
        Cinematique_ROS_B.e_d[Cinematique_ROS_B.d_k] *= Cinematique_ROS_B.emm1;
        memcpy(&Cinematique_ROS_B.A[0], &U[0], 9U * sizeof(real_T));
        Cinematique_ROS_xrot(Cinematique_ROS_B.A, Cinematique_ROS_B.qjj,
                             Cinematique_ROS_B.kase, Cinematique_ROS_B.emm1,
                             Cinematique_ROS_B.d_sn, U);
      }

      Cinematique_ROS_B.e_d[Cinematique_ROS_B.m_l - 1] = Cinematique_ROS_B.rt;
      Cinematique_ROS_B.qq++;
      break;

     default:
      if (Cinematique_ROS_B.s[Cinematique_ROS_B.c_q] < 0.0) {
        Cinematique_ROS_B.s[Cinematique_ROS_B.c_q] =
          -Cinematique_ROS_B.s[Cinematique_ROS_B.c_q];
        Cinematique_ROS_B.qjj = 3 * Cinematique_ROS_B.c_q;
        for (Cinematique_ROS_B.qq = Cinematique_ROS_B.qjj + 1;
             Cinematique_ROS_B.qq <= Cinematique_ROS_B.qjj + 3;
             Cinematique_ROS_B.qq++) {
          V[Cinematique_ROS_B.qq - 1] = -V[Cinematique_ROS_B.qq - 1];
        }
      }

      Cinematique_ROS_B.qq = Cinematique_ROS_B.c_q + 1;
      while ((Cinematique_ROS_B.c_q + 1 < 3) &&
             (Cinematique_ROS_B.s[Cinematique_ROS_B.c_q] <
              Cinematique_ROS_B.s[Cinematique_ROS_B.qq])) {
        Cinematique_ROS_B.rt = Cinematique_ROS_B.s[Cinematique_ROS_B.c_q];
        Cinematique_ROS_B.s[Cinematique_ROS_B.c_q] =
          Cinematique_ROS_B.s[Cinematique_ROS_B.qq];
        Cinematique_ROS_B.s[Cinematique_ROS_B.qq] = Cinematique_ROS_B.rt;
        Cinematique_ROS_B.qjj = 3 * Cinematique_ROS_B.c_q + 1;
        Cinematique_ROS_B.kase = (Cinematique_ROS_B.c_q + 1) * 3 + 1;
        memcpy(&Cinematique_ROS_B.A[0], &V[0], 9U * sizeof(real_T));
        Cinematique_ROS_xswap(Cinematique_ROS_B.A, Cinematique_ROS_B.qjj,
                              Cinematique_ROS_B.kase, V);
        memcpy(&Cinematique_ROS_B.A[0], &U[0], 9U * sizeof(real_T));
        Cinematique_ROS_xswap(Cinematique_ROS_B.A, Cinematique_ROS_B.qjj,
                              Cinematique_ROS_B.kase, U);
        Cinematique_ROS_B.c_q = Cinematique_ROS_B.qq;
        Cinematique_ROS_B.qq++;
      }

      Cinematique_ROS_B.qq = 0;
      Cinematique_ROS_B.m_l--;
      break;
    }
  }

  s[0] = Cinematique_ROS_B.s[0];
  s[1] = Cinematique_ROS_B.s[1];
  s[2] = Cinematique_ROS_B.s[2];
}

static void Cinematiq_IKHelpers_computeCost(const real_T x[4],
  f_robotics_manip_internal_IKE_T *args, real_T *cost, real_T W[36],
  emxArray_real_T_Cinematique_R_T *Jac, f_robotics_manip_internal_IKE_T **b_args)
{
  ab_robotics_manip_internal_Ri_T *treeInternal;
  emxArray_char_T_Cinematique_R_T *bodyName;
  emxArray_real_T_Cinematique_R_T *J;
  emxArray_real_T_Cinematique_R_T *y;
  boolean_T exitg1;
  Cinematique_ROS_emxInit_char_T(&bodyName, 2);
  *b_args = args;
  treeInternal = args->Robot;
  Cinematique_ROS_B.b_k_h = bodyName->size[0] * bodyName->size[1];
  bodyName->size[0] = 1;
  bodyName->size[1] = args->BodyName->size[1];
  Cinema_emxEnsureCapacity_char_T(bodyName, Cinematique_ROS_B.b_k_h);
  Cinematique_ROS_B.loop_ub_cs = args->BodyName->size[0] * args->BodyName->size
    [1] - 1;
  for (Cinematique_ROS_B.b_k_h = 0; Cinematique_ROS_B.b_k_h <=
       Cinematique_ROS_B.loop_ub_cs; Cinematique_ROS_B.b_k_h++) {
    bodyName->data[Cinematique_ROS_B.b_k_h] = args->BodyName->
      data[Cinematique_ROS_B.b_k_h];
  }

  for (Cinematique_ROS_B.b_k_h = 0; Cinematique_ROS_B.b_k_h < 16;
       Cinematique_ROS_B.b_k_h++) {
    Cinematique_ROS_B.Td[Cinematique_ROS_B.b_k_h] = args->
      Tform[Cinematique_ROS_B.b_k_h];
  }

  for (Cinematique_ROS_B.b_k_h = 0; Cinematique_ROS_B.b_k_h < 36;
       Cinematique_ROS_B.b_k_h++) {
    W[Cinematique_ROS_B.b_k_h] = args->WeightMatrix[Cinematique_ROS_B.b_k_h];
  }

  Cinematique_ROS_emxInit_real_T(&J, 2);
  RigidBodyTree_efficientFKAndJac(treeInternal, x, bodyName,
    Cinematique_ROS_B.T_data, Cinematique_ROS_B.T_size, J);
  Cinematique_ROS_B.b_k_h = Jac->size[0] * Jac->size[1];
  Jac->size[0] = 6;
  Jac->size[1] = J->size[1];
  Cinema_emxEnsureCapacity_real_T(Jac, Cinematique_ROS_B.b_k_h);
  Cinematique_ROS_B.loop_ub_cs = J->size[0] * J->size[1] - 1;
  Cinematique_ROS_emxFree_char_T(&bodyName);
  for (Cinematique_ROS_B.b_k_h = 0; Cinematique_ROS_B.b_k_h <=
       Cinematique_ROS_B.loop_ub_cs; Cinematique_ROS_B.b_k_h++) {
    Jac->data[Cinematique_ROS_B.b_k_h] = -J->data[Cinematique_ROS_B.b_k_h];
  }

  Cinematique_ROS_emxFree_real_T(&J);
  Cinematique_ROS_B.y_tmp_d[0] = 1;
  Cinematique_ROS_B.y_tmp_d[1] = 2;
  Cinematique_ROS_B.y_tmp_d[2] = 3;
  for (Cinematique_ROS_B.b_k_h = 0; Cinematique_ROS_B.b_k_h < 3;
       Cinematique_ROS_B.b_k_h++) {
    Cinematique_ROS_B.y_tmp_g =
      Cinematique_ROS_B.y_tmp_d[Cinematique_ROS_B.b_k_h];
    Cinematique_ROS_B.T[3 * Cinematique_ROS_B.b_k_h] =
      Cinematique_ROS_B.T_data[Cinematique_ROS_B.y_tmp_g - 1];
    Cinematique_ROS_B.n_m = 3 * Cinematique_ROS_B.b_k_h + 1;
    Cinematique_ROS_B.T[Cinematique_ROS_B.n_m] = Cinematique_ROS_B.T_data
      [(Cinematique_ROS_B.y_tmp_g + Cinematique_ROS_B.T_size[0]) - 1];
    Cinematique_ROS_B.boffset = 3 * Cinematique_ROS_B.b_k_h + 2;
    Cinematique_ROS_B.T[Cinematique_ROS_B.boffset] = Cinematique_ROS_B.T_data
      [((Cinematique_ROS_B.T_size[0] << 1) + Cinematique_ROS_B.y_tmp_g) - 1];
    for (Cinematique_ROS_B.loop_ub_cs = 0; Cinematique_ROS_B.loop_ub_cs < 3;
         Cinematique_ROS_B.loop_ub_cs++) {
      Cinematique_ROS_B.y_tmp = Cinematique_ROS_B.loop_ub_cs + 3 *
        Cinematique_ROS_B.b_k_h;
      Cinematique_ROS_B.y[Cinematique_ROS_B.y_tmp] = 0.0;
      Cinematique_ROS_B.y[Cinematique_ROS_B.y_tmp] += Cinematique_ROS_B.T[3 *
        Cinematique_ROS_B.b_k_h] *
        Cinematique_ROS_B.Td[Cinematique_ROS_B.loop_ub_cs];
      Cinematique_ROS_B.y[Cinematique_ROS_B.y_tmp] +=
        Cinematique_ROS_B.T[Cinematique_ROS_B.n_m] *
        Cinematique_ROS_B.Td[Cinematique_ROS_B.loop_ub_cs + 4];
      Cinematique_ROS_B.y[Cinematique_ROS_B.y_tmp] +=
        Cinematique_ROS_B.T[Cinematique_ROS_B.boffset] *
        Cinematique_ROS_B.Td[Cinematique_ROS_B.loop_ub_cs + 8];
    }
  }

  Cinematique_ROS_B.u.re = (((Cinematique_ROS_B.y[0] + Cinematique_ROS_B.y[4]) +
    Cinematique_ROS_B.y[8]) - 1.0) * 0.5;
  if (!(fabs(Cinematique_ROS_B.u.re) > 1.0)) {
    Cinematique_ROS_B.v_g.re = acos(Cinematique_ROS_B.u.re);
  } else {
    Cinematique_ROS_B.u_l.re = Cinematique_ROS_B.u.re + 1.0;
    Cinematique_ROS_B.u_l.im = 0.0;
    Cinematique_ROS_B.dc.re = 1.0 - Cinematique_ROS_B.u.re;
    Cinematique_ROS_B.dc.im = 0.0;
    Cinematique_ROS_B.v_g.re = 2.0 * rt_atan2d_snf((Cinematique_ROS_sqrt
      (Cinematique_ROS_B.dc)).re, (Cinematique_ROS_sqrt(Cinematique_ROS_B.u_l)).
      re);
  }

  Cinematique_ROS_B.s_j = 2.0 * sin(Cinematique_ROS_B.v_g.re);
  Cinematique_ROS_B.v[0] = (Cinematique_ROS_B.y[5] - Cinematique_ROS_B.y[7]) /
    Cinematique_ROS_B.s_j;
  Cinematique_ROS_B.v[1] = (Cinematique_ROS_B.y[6] - Cinematique_ROS_B.y[2]) /
    Cinematique_ROS_B.s_j;
  Cinematique_ROS_B.v[2] = (Cinematique_ROS_B.y[1] - Cinematique_ROS_B.y[3]) /
    Cinematique_ROS_B.s_j;
  if (rtIsNaN(Cinematique_ROS_B.v_g.re) || rtIsInf(Cinematique_ROS_B.v_g.re)) {
    Cinematique_ROS_B.s_j = (rtNaN);
  } else if (Cinematique_ROS_B.v_g.re == 0.0) {
    Cinematique_ROS_B.s_j = 0.0;
  } else {
    Cinematique_ROS_B.s_j = fmod(Cinematique_ROS_B.v_g.re, 3.1415926535897931);
    Cinematique_ROS_B.rEQ0 = (Cinematique_ROS_B.s_j == 0.0);
    if (!Cinematique_ROS_B.rEQ0) {
      Cinematique_ROS_B.q = fabs(Cinematique_ROS_B.v_g.re / 3.1415926535897931);
      Cinematique_ROS_B.rEQ0 = !(fabs(Cinematique_ROS_B.q - floor
        (Cinematique_ROS_B.q + 0.5)) > 2.2204460492503131E-16 *
        Cinematique_ROS_B.q);
    }

    if (Cinematique_ROS_B.rEQ0) {
      Cinematique_ROS_B.s_j = 0.0;
    } else {
      if (Cinematique_ROS_B.v_g.re < 0.0) {
        Cinematique_ROS_B.s_j += 3.1415926535897931;
      }
    }
  }

  Cinematique_ROS_B.rEQ0 = (Cinematique_ROS_B.s_j == 0.0);
  Cinematique_ROS_B.e_i = true;
  Cinematique_ROS_B.b_k_h = 0;
  exitg1 = false;
  while ((!exitg1) && (Cinematique_ROS_B.b_k_h < 3)) {
    if (!(Cinematique_ROS_B.v[Cinematique_ROS_B.b_k_h] == 0.0)) {
      Cinematique_ROS_B.e_i = false;
      exitg1 = true;
    } else {
      Cinematique_ROS_B.b_k_h++;
    }
  }

  if (Cinematique_ROS_B.rEQ0 || Cinematique_ROS_B.e_i) {
    Cinematique_ROS_B.b_k_h = (Cinematique_ROS_B.rEQ0 || Cinematique_ROS_B.e_i);
    Cinematique_ROS_B.loop_ub_cs = Cinematique_ROS_B.b_k_h * 3 - 1;
    if (0 <= Cinematique_ROS_B.loop_ub_cs) {
      memset(&Cinematique_ROS_B.vspecial_data[0], 0,
             (Cinematique_ROS_B.loop_ub_cs + 1) * sizeof(real_T));
    }

    Cinematique_ROS_B.n_m = Cinematique_ROS_B.b_k_h - 1;
    for (Cinematique_ROS_B.loop_ub_cs = 0; Cinematique_ROS_B.loop_ub_cs <=
         Cinematique_ROS_B.n_m; Cinematique_ROS_B.loop_ub_cs++) {
      memset(&Cinematique_ROS_B.T[0], 0, 9U * sizeof(real_T));
      Cinematique_ROS_B.T[0] = 1.0;
      Cinematique_ROS_B.T[4] = 1.0;
      Cinematique_ROS_B.T[8] = 1.0;
      Cinematique_ROS_B.p_d = true;
      for (Cinematique_ROS_B.b_k_h = 0; Cinematique_ROS_B.b_k_h < 9;
           Cinematique_ROS_B.b_k_h++) {
        Cinematique_ROS_B.s_j = Cinematique_ROS_B.T[Cinematique_ROS_B.b_k_h] -
          Cinematique_ROS_B.y[Cinematique_ROS_B.b_k_h];
        if (Cinematique_ROS_B.p_d && ((!rtIsInf(Cinematique_ROS_B.s_j)) &&
             (!rtIsNaN(Cinematique_ROS_B.s_j)))) {
        } else {
          Cinematique_ROS_B.p_d = false;
        }

        Cinematique_ROS_B.T[Cinematique_ROS_B.b_k_h] = Cinematique_ROS_B.s_j;
      }

      if (Cinematique_ROS_B.p_d) {
        Cinematique_ROS_svd(Cinematique_ROS_B.T, Cinematique_ROS_B.b_U,
                            Cinematique_ROS_B.vspecial_data,
                            Cinematique_ROS_B.V_c);
      } else {
        for (Cinematique_ROS_B.b_k_h = 0; Cinematique_ROS_B.b_k_h < 9;
             Cinematique_ROS_B.b_k_h++) {
          Cinematique_ROS_B.V_c[Cinematique_ROS_B.b_k_h] = (rtNaN);
        }
      }

      Cinematique_ROS_B.vspecial_data[0] = Cinematique_ROS_B.V_c[6];
      Cinematique_ROS_B.vspecial_data[1] = Cinematique_ROS_B.V_c[7];
      Cinematique_ROS_B.vspecial_data[2] = Cinematique_ROS_B.V_c[8];
    }

    Cinematique_ROS_B.loop_ub_cs = 0;
    if (Cinematique_ROS_B.rEQ0 || Cinematique_ROS_B.e_i) {
      for (Cinematique_ROS_B.b_k_h = 0; Cinematique_ROS_B.b_k_h < 1;
           Cinematique_ROS_B.b_k_h++) {
        Cinematique_ROS_B.loop_ub_cs++;
      }
    }

    for (Cinematique_ROS_B.b_k_h = 0; Cinematique_ROS_B.b_k_h <
         Cinematique_ROS_B.loop_ub_cs; Cinematique_ROS_B.b_k_h++) {
      Cinematique_ROS_B.v[0] = Cinematique_ROS_B.vspecial_data[3 *
        Cinematique_ROS_B.b_k_h];
      Cinematique_ROS_B.v[1] = Cinematique_ROS_B.vspecial_data[3 *
        Cinematique_ROS_B.b_k_h + 1];
      Cinematique_ROS_B.v[2] = Cinematique_ROS_B.vspecial_data[3 *
        Cinematique_ROS_B.b_k_h + 2];
    }
  }

  Cinematique_ROS_B.s_j = 1.0 / sqrt((Cinematique_ROS_B.v[0] *
    Cinematique_ROS_B.v[0] + Cinematique_ROS_B.v[1] * Cinematique_ROS_B.v[1]) +
    Cinematique_ROS_B.v[2] * Cinematique_ROS_B.v[2]);
  Cinematique_ROS_B.v[0] *= Cinematique_ROS_B.s_j;
  Cinematique_ROS_B.v[1] *= Cinematique_ROS_B.s_j;
  Cinematique_ROS_B.e[0] = Cinematique_ROS_B.v_g.re * Cinematique_ROS_B.v[0];
  Cinematique_ROS_B.e[3] = Cinematique_ROS_B.Td[12] -
    Cinematique_ROS_B.T_data[Cinematique_ROS_B.T_size[0] * 3];
  Cinematique_ROS_B.e[1] = Cinematique_ROS_B.v_g.re * Cinematique_ROS_B.v[1];
  Cinematique_ROS_B.e[4] = Cinematique_ROS_B.Td[13] -
    Cinematique_ROS_B.T_data[Cinematique_ROS_B.T_size[0] * 3 + 1];
  Cinematique_ROS_B.e[2] = Cinematique_ROS_B.v[2] * Cinematique_ROS_B.s_j *
    Cinematique_ROS_B.v_g.re;
  Cinematique_ROS_B.e[5] = Cinematique_ROS_B.Td[14] -
    Cinematique_ROS_B.T_data[Cinematique_ROS_B.T_size[0] * 3 + 2];
  Cinematique_ROS_B.b_k_h = args->ErrTemp->size[0];
  args->ErrTemp->size[0] = 6;
  Cinema_emxEnsureCapacity_real_T(args->ErrTemp, Cinematique_ROS_B.b_k_h);
  for (Cinematique_ROS_B.b_k_h = 0; Cinematique_ROS_B.b_k_h < 6;
       Cinematique_ROS_B.b_k_h++) {
    args->ErrTemp->data[Cinematique_ROS_B.b_k_h] =
      Cinematique_ROS_B.e[Cinematique_ROS_B.b_k_h];
  }

  for (Cinematique_ROS_B.b_k_h = 0; Cinematique_ROS_B.b_k_h < 6;
       Cinematique_ROS_B.b_k_h++) {
    Cinematique_ROS_B.y_g[Cinematique_ROS_B.b_k_h] = 0.0;
    for (Cinematique_ROS_B.loop_ub_cs = 0; Cinematique_ROS_B.loop_ub_cs < 6;
         Cinematique_ROS_B.loop_ub_cs++) {
      Cinematique_ROS_B.s_j = W[6 * Cinematique_ROS_B.b_k_h +
        Cinematique_ROS_B.loop_ub_cs] * (0.5 *
        Cinematique_ROS_B.e[Cinematique_ROS_B.loop_ub_cs]) +
        Cinematique_ROS_B.y_g[Cinematique_ROS_B.b_k_h];
      Cinematique_ROS_B.y_g[Cinematique_ROS_B.b_k_h] = Cinematique_ROS_B.s_j;
    }
  }

  Cinematique_ROS_B.s_j = 0.0;
  for (Cinematique_ROS_B.b_k_h = 0; Cinematique_ROS_B.b_k_h < 6;
       Cinematique_ROS_B.b_k_h++) {
    Cinematique_ROS_B.s_j += Cinematique_ROS_B.y_g[Cinematique_ROS_B.b_k_h] *
      Cinematique_ROS_B.e[Cinematique_ROS_B.b_k_h];
  }

  args->CostTemp = Cinematique_ROS_B.s_j;
  for (Cinematique_ROS_B.b_k_h = 0; Cinematique_ROS_B.b_k_h < 6;
       Cinematique_ROS_B.b_k_h++) {
    Cinematique_ROS_B.y_g[Cinematique_ROS_B.b_k_h] = 0.0;
    for (Cinematique_ROS_B.loop_ub_cs = 0; Cinematique_ROS_B.loop_ub_cs < 6;
         Cinematique_ROS_B.loop_ub_cs++) {
      Cinematique_ROS_B.s_j = W[6 * Cinematique_ROS_B.b_k_h +
        Cinematique_ROS_B.loop_ub_cs] *
        Cinematique_ROS_B.e[Cinematique_ROS_B.loop_ub_cs] +
        Cinematique_ROS_B.y_g[Cinematique_ROS_B.b_k_h];
      Cinematique_ROS_B.y_g[Cinematique_ROS_B.b_k_h] = Cinematique_ROS_B.s_j;
    }
  }

  Cinematique_ROS_emxInit_real_T(&y, 2);
  Cinematique_ROS_B.n_m = Jac->size[1] - 1;
  Cinematique_ROS_B.b_k_h = y->size[0] * y->size[1];
  y->size[0] = 1;
  y->size[1] = Jac->size[1];
  Cinema_emxEnsureCapacity_real_T(y, Cinematique_ROS_B.b_k_h);
  for (Cinematique_ROS_B.loop_ub_cs = 0; Cinematique_ROS_B.loop_ub_cs <=
       Cinematique_ROS_B.n_m; Cinematique_ROS_B.loop_ub_cs++) {
    Cinematique_ROS_B.boffset = Cinematique_ROS_B.loop_ub_cs * 6 - 1;
    Cinematique_ROS_B.s_j = 0.0;
    for (Cinematique_ROS_B.b_k_h = 0; Cinematique_ROS_B.b_k_h < 6;
         Cinematique_ROS_B.b_k_h++) {
      Cinematique_ROS_B.s_j += Jac->data[(Cinematique_ROS_B.boffset +
        Cinematique_ROS_B.b_k_h) + 1] *
        Cinematique_ROS_B.y_g[Cinematique_ROS_B.b_k_h];
    }

    y->data[Cinematique_ROS_B.loop_ub_cs] = Cinematique_ROS_B.s_j;
  }

  Cinematique_ROS_B.b_k_h = args->GradTemp->size[0];
  args->GradTemp->size[0] = y->size[1];
  Cinema_emxEnsureCapacity_real_T(args->GradTemp, Cinematique_ROS_B.b_k_h);
  Cinematique_ROS_B.loop_ub_cs = y->size[1];
  for (Cinematique_ROS_B.b_k_h = 0; Cinematique_ROS_B.b_k_h <
       Cinematique_ROS_B.loop_ub_cs; Cinematique_ROS_B.b_k_h++) {
    args->GradTemp->data[Cinematique_ROS_B.b_k_h] = y->
      data[Cinematique_ROS_B.b_k_h];
  }

  Cinematique_ROS_emxFree_real_T(&y);
  Cinematique_ROS_B.s_j = args->CostTemp;
  *cost = Cinematique_ROS_B.s_j;
}

static void Cinematique_R_emxInit_boolean_T(emxArray_boolean_T_Cinematiqu_T
  **pEmxArray, int32_T numDimensions)
{
  emxArray_boolean_T_Cinematiqu_T *emxArray;
  int32_T i;
  *pEmxArray = (emxArray_boolean_T_Cinematiqu_T *)malloc(sizeof
    (emxArray_boolean_T_Cinematiqu_T));
  emxArray = *pEmxArray;
  emxArray->data = (boolean_T *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int32_T *)malloc(sizeof(int32_T) * numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

static void Cinematique_ROS_emxInit_int32_T(emxArray_int32_T_Cinematique__T
  **pEmxArray, int32_T numDimensions)
{
  emxArray_int32_T_Cinematique__T *emxArray;
  int32_T i;
  *pEmxArray = (emxArray_int32_T_Cinematique__T *)malloc(sizeof
    (emxArray_int32_T_Cinematique__T));
  emxArray = *pEmxArray;
  emxArray->data = (int32_T *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int32_T *)malloc(sizeof(int32_T) * numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

static void Cin_emxEnsureCapacity_boolean_T(emxArray_boolean_T_Cinematiqu_T
  *emxArray, int32_T oldNumel)
{
  int32_T i;
  int32_T newNumel;
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  newNumel = 1;
  for (i = 0; i < emxArray->numDimensions; i++) {
    newNumel *= emxArray->size[i];
  }

  if (newNumel > emxArray->allocatedSize) {
    i = emxArray->allocatedSize;
    if (i < 16) {
      i = 16;
    }

    while (i < newNumel) {
      if (i > 1073741823) {
        i = MAX_int32_T;
      } else {
        i <<= 1;
      }
    }

    newData = calloc(static_cast<uint32_T>(i), sizeof(boolean_T));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(boolean_T) * oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }

    emxArray->data = (boolean_T *)newData;
    emxArray->allocatedSize = i;
    emxArray->canFreeData = true;
  }
}

static void Cinematique_ROS_mtimes_k(const emxArray_real_T_Cinematique_R_T *A,
  const real_T B[4], emxArray_real_T_Cinematique_R_T *C)
{
  int32_T b_i;
  int32_T c_i;
  int32_T inner;
  int32_T m;
  m = A->size[1] - 1;
  inner = A->size[0] - 1;
  b_i = C->size[0];
  C->size[0] = A->size[1];
  Cinema_emxEnsureCapacity_real_T(C, b_i);
  for (b_i = 0; b_i <= m; b_i++) {
    C->data[b_i] = 0.0;
  }

  for (b_i = 0; b_i <= inner; b_i++) {
    for (c_i = 0; c_i <= m; c_i++) {
      C->data[c_i] += A->data[c_i * A->size[0] + b_i] * B[b_i];
    }
  }
}

static void Cinem_emxEnsureCapacity_int32_T(emxArray_int32_T_Cinematique__T
  *emxArray, int32_T oldNumel)
{
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  Cinematique_ROS_B.newNumel_n = 1;
  for (Cinematique_ROS_B.i_k = 0; Cinematique_ROS_B.i_k <
       emxArray->numDimensions; Cinematique_ROS_B.i_k++) {
    Cinematique_ROS_B.newNumel_n *= emxArray->size[Cinematique_ROS_B.i_k];
  }

  if (Cinematique_ROS_B.newNumel_n > emxArray->allocatedSize) {
    Cinematique_ROS_B.i_k = emxArray->allocatedSize;
    if (Cinematique_ROS_B.i_k < 16) {
      Cinematique_ROS_B.i_k = 16;
    }

    while (Cinematique_ROS_B.i_k < Cinematique_ROS_B.newNumel_n) {
      if (Cinematique_ROS_B.i_k > 1073741823) {
        Cinematique_ROS_B.i_k = MAX_int32_T;
      } else {
        Cinematique_ROS_B.i_k <<= 1;
      }
    }

    newData = calloc(static_cast<uint32_T>(Cinematique_ROS_B.i_k), sizeof
                     (int32_T));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(int32_T) * oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }

    emxArray->data = (int32_T *)newData;
    emxArray->allocatedSize = Cinematique_ROS_B.i_k;
    emxArray->canFreeData = true;
  }
}

static void Cinematique_ROS_mtimes_k1(const real_T A[16], const
  emxArray_real_T_Cinematique_R_T *B, emxArray_real_T_Cinematique_R_T *C)
{
  real_T s;
  int32_T b_i;
  int32_T b_j;
  int32_T boffset;
  int32_T coffset;
  int32_T n;
  n = B->size[1] - 1;
  b_j = C->size[0] * C->size[1];
  C->size[0] = 4;
  C->size[1] = B->size[1];
  Cinema_emxEnsureCapacity_real_T(C, b_j);
  for (b_j = 0; b_j <= n; b_j++) {
    coffset = (b_j << 2) - 1;
    boffset = b_j * B->size[0] - 1;
    for (b_i = 0; b_i < 4; b_i++) {
      s = B->data[boffset + 1] * A[b_i];
      s += A[b_i + 4] * B->data[boffset + 2];
      s += A[b_i + 8] * B->data[boffset + 3];
      s += A[b_i + 12] * B->data[boffset + 4];
      C->data[(coffset + b_i) + 1] = s;
    }
  }
}

static real_T Cine_IKHelpers_evaluateSolution(const
  f_robotics_manip_internal_IKE_T *args)
{
  emxArray_real_T_Cinematique_R_T *b;
  real_T en;
  for (Cinematique_ROS_B.b_k_p = 0; Cinematique_ROS_B.b_k_p < 36;
       Cinematique_ROS_B.b_k_p++) {
    Cinematique_ROS_B.a[Cinematique_ROS_B.b_k_p] = args->
      WeightMatrix[Cinematique_ROS_B.b_k_p];
  }

  Cinematique_ROS_emxInit_real_T(&b, 1);
  Cinematique_ROS_B.b_k_p = b->size[0];
  b->size[0] = args->ErrTemp->size[0];
  Cinema_emxEnsureCapacity_real_T(b, Cinematique_ROS_B.b_k_p);
  Cinematique_ROS_B.loop_ub_pt = args->ErrTemp->size[0];
  for (Cinematique_ROS_B.b_k_p = 0; Cinematique_ROS_B.b_k_p <
       Cinematique_ROS_B.loop_ub_pt; Cinematique_ROS_B.b_k_p++) {
    b->data[Cinematique_ROS_B.b_k_p] = args->ErrTemp->
      data[Cinematique_ROS_B.b_k_p];
  }

  for (Cinematique_ROS_B.b_k_p = 0; Cinematique_ROS_B.b_k_p < 6;
       Cinematique_ROS_B.b_k_p++) {
    Cinematique_ROS_B.y_g1[Cinematique_ROS_B.b_k_p] = 0.0;
    for (Cinematique_ROS_B.loop_ub_pt = 0; Cinematique_ROS_B.loop_ub_pt < 6;
         Cinematique_ROS_B.loop_ub_pt++) {
      Cinematique_ROS_B.scale_m = Cinematique_ROS_B.a[6 *
        Cinematique_ROS_B.loop_ub_pt + Cinematique_ROS_B.b_k_p] * b->
        data[Cinematique_ROS_B.loop_ub_pt] +
        Cinematique_ROS_B.y_g1[Cinematique_ROS_B.b_k_p];
      Cinematique_ROS_B.y_g1[Cinematique_ROS_B.b_k_p] =
        Cinematique_ROS_B.scale_m;
    }
  }

  Cinematique_ROS_emxFree_real_T(&b);
  en = 0.0;
  Cinematique_ROS_B.scale_m = 3.3121686421112381E-170;
  for (Cinematique_ROS_B.b_k_p = 0; Cinematique_ROS_B.b_k_p < 6;
       Cinematique_ROS_B.b_k_p++) {
    Cinematique_ROS_B.absxk = fabs
      (Cinematique_ROS_B.y_g1[Cinematique_ROS_B.b_k_p]);
    if (Cinematique_ROS_B.absxk > Cinematique_ROS_B.scale_m) {
      Cinematique_ROS_B.t_j = Cinematique_ROS_B.scale_m /
        Cinematique_ROS_B.absxk;
      en = en * Cinematique_ROS_B.t_j * Cinematique_ROS_B.t_j + 1.0;
      Cinematique_ROS_B.scale_m = Cinematique_ROS_B.absxk;
    } else {
      Cinematique_ROS_B.t_j = Cinematique_ROS_B.absxk /
        Cinematique_ROS_B.scale_m;
      en += Cinematique_ROS_B.t_j * Cinematique_ROS_B.t_j;
    }
  }

  return Cinematique_ROS_B.scale_m * sqrt(en);
}

static real_T Cinematique_ROS_toc(real_T tstart_tv_sec, real_T tstart_tv_nsec)
{
  struct timespec b_timespec;
  clock_gettime(CLOCK_MONOTONIC, &b_timespec);
  return ((real_T)b_timespec.tv_nsec - tstart_tv_nsec) / 1.0E+9 + ((real_T)
    b_timespec.tv_sec - tstart_tv_sec);
}

static void Cinematique_ROS_mtimes_k1e(const emxArray_real_T_Cinematique_R_T *A,
  const emxArray_real_T_Cinematique_R_T *B, emxArray_real_T_Cinematique_R_T *C)
{
  real_T bkj;
  int32_T b_i;
  int32_T b_j;
  int32_T boffset;
  int32_T c_i;
  int32_T coffset;
  int32_T inner;
  int32_T m;
  int32_T n;
  int32_T tmp;
  m = A->size[1] - 1;
  inner = A->size[0] - 1;
  n = B->size[1] - 1;
  tmp = C->size[0] * C->size[1];
  C->size[0] = A->size[1];
  C->size[1] = B->size[1];
  Cinema_emxEnsureCapacity_real_T(C, tmp);
  for (b_j = 0; b_j <= n; b_j++) {
    coffset = (m + 1) * b_j - 1;
    boffset = b_j * B->size[0] - 1;
    for (b_i = 0; b_i <= m; b_i++) {
      C->data[(coffset + b_i) + 1] = 0.0;
    }

    for (b_i = 0; b_i <= inner; b_i++) {
      bkj = B->data[(boffset + b_i) + 1];
      for (c_i = 0; c_i <= m; c_i++) {
        tmp = (coffset + c_i) + 1;
        C->data[tmp] += A->data[c_i * A->size[0] + b_i] * bkj;
      }
    }
  }
}

static real_T Cinematique_ROS_xnrm2_k1(int32_T n, const
  emxArray_real_T_Cinematique_R_T *x, int32_T ix0)
{
  real_T y;
  y = 0.0;
  if (n >= 1) {
    if (n == 1) {
      y = fabs(x->data[ix0 - 1]);
    } else {
      Cinematique_ROS_B.scale_p = 3.3121686421112381E-170;
      Cinematique_ROS_B.kend_h = ix0 + n;
      for (Cinematique_ROS_B.k_m = ix0; Cinematique_ROS_B.k_m <
           Cinematique_ROS_B.kend_h; Cinematique_ROS_B.k_m++) {
        Cinematique_ROS_B.absxk_a = fabs(x->data[Cinematique_ROS_B.k_m - 1]);
        if (Cinematique_ROS_B.absxk_a > Cinematique_ROS_B.scale_p) {
          Cinematique_ROS_B.t_e = Cinematique_ROS_B.scale_p /
            Cinematique_ROS_B.absxk_a;
          y = y * Cinematique_ROS_B.t_e * Cinematique_ROS_B.t_e + 1.0;
          Cinematique_ROS_B.scale_p = Cinematique_ROS_B.absxk_a;
        } else {
          Cinematique_ROS_B.t_e = Cinematique_ROS_B.absxk_a /
            Cinematique_ROS_B.scale_p;
          y += Cinematique_ROS_B.t_e * Cinematique_ROS_B.t_e;
        }
      }

      y = Cinematique_ROS_B.scale_p * sqrt(y);
    }
  }

  return y;
}

static void Cinematique_ROS_emxFree_int32_T(emxArray_int32_T_Cinematique__T
  **pEmxArray)
{
  if (*pEmxArray != (emxArray_int32_T_Cinematique__T *)NULL) {
    if (((*pEmxArray)->data != (int32_T *)NULL) && (*pEmxArray)->canFreeData) {
      free((*pEmxArray)->data);
    }

    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = (emxArray_int32_T_Cinematique__T *)NULL;
  }
}

static void Cinematique_ROS_xgeqp3(const emxArray_real_T_Cinematique_R_T *A,
  emxArray_real_T_Cinematique_R_T *b_A, emxArray_real_T_Cinematique_R_T *tau,
  emxArray_int32_T_Cinematique__T *jpvt)
{
  emxArray_int32_T_Cinematique__T *b_jpvt;
  emxArray_int32_T_Cinematique__T *b_jpvt_0;
  emxArray_real_T_Cinematique_R_T *c_x;
  emxArray_real_T_Cinematique_R_T *d_A;
  emxArray_real_T_Cinematique_R_T *vn1;
  emxArray_real_T_Cinematique_R_T *vn2;
  emxArray_real_T_Cinematique_R_T *work;
  int32_T exitg1;
  boolean_T exitg2;
  boolean_T guard1 = false;
  Cinematique_ROS_B.ma = A->size[0];
  Cinematique_ROS_B.na = A->size[1];
  Cinematique_ROS_B.itemp = b_A->size[0] * b_A->size[1];
  b_A->size[0] = A->size[0];
  b_A->size[1] = A->size[1];
  Cinema_emxEnsureCapacity_real_T(b_A, Cinematique_ROS_B.itemp);
  Cinematique_ROS_B.ix_o = A->size[0] * A->size[1] - 1;
  for (Cinematique_ROS_B.itemp = 0; Cinematique_ROS_B.itemp <=
       Cinematique_ROS_B.ix_o; Cinematique_ROS_B.itemp++) {
    b_A->data[Cinematique_ROS_B.itemp] = A->data[Cinematique_ROS_B.itemp];
  }

  Cinematique_ROS_B.u0_f = A->size[0];
  Cinematique_ROS_B.minmana = A->size[1];
  if (Cinematique_ROS_B.u0_f < Cinematique_ROS_B.minmana) {
    Cinematique_ROS_B.minmana = Cinematique_ROS_B.u0_f;
  }

  Cinematique_ROS_B.itemp = tau->size[0];
  tau->size[0] = Cinematique_ROS_B.minmana;
  Cinema_emxEnsureCapacity_real_T(tau, Cinematique_ROS_B.itemp);
  for (Cinematique_ROS_B.itemp = 0; Cinematique_ROS_B.itemp <
       Cinematique_ROS_B.minmana; Cinematique_ROS_B.itemp++) {
    tau->data[Cinematique_ROS_B.itemp] = 0.0;
  }

  Cinematique_ROS_emxInit_int32_T(&b_jpvt, 2);
  Cinematique_ROS_emxInit_real_T(&d_A, 2);
  Cinematique_ROS_emxInit_int32_T(&b_jpvt_0, 2);
  Cinematique_ROS_emxInit_real_T(&work, 1);
  Cinematique_ROS_emxInit_real_T(&vn1, 1);
  Cinematique_ROS_emxInit_real_T(&vn2, 1);
  Cinematique_ROS_emxInit_real_T(&c_x, 2);
  guard1 = false;
  if ((A->size[0] == 0) || (A->size[1] == 0)) {
    guard1 = true;
  } else {
    Cinematique_ROS_B.u0_f = A->size[0];
    Cinematique_ROS_B.minmn_i = A->size[1];
    if (Cinematique_ROS_B.u0_f < Cinematique_ROS_B.minmn_i) {
      Cinematique_ROS_B.minmn_i = Cinematique_ROS_B.u0_f;
    }

    if (Cinematique_ROS_B.minmn_i < 1) {
      guard1 = true;
    } else {
      Cinematique_ROS_B.itemp = b_jpvt->size[0] * b_jpvt->size[1];
      b_jpvt->size[0] = 1;
      b_jpvt->size[1] = A->size[1];
      Cinem_emxEnsureCapacity_int32_T(b_jpvt, Cinematique_ROS_B.itemp);
      Cinematique_ROS_B.ix_o = A->size[1] - 1;
      for (Cinematique_ROS_B.itemp = 0; Cinematique_ROS_B.itemp <=
           Cinematique_ROS_B.ix_o; Cinematique_ROS_B.itemp++) {
        b_jpvt->data[Cinematique_ROS_B.itemp] = 0;
      }

      for (Cinematique_ROS_B.u0_f = 0; Cinematique_ROS_B.u0_f <
           Cinematique_ROS_B.na; Cinematique_ROS_B.u0_f++) {
        b_jpvt->data[Cinematique_ROS_B.u0_f] = Cinematique_ROS_B.u0_f + 1;
      }

      Cinematique_ROS_B.itemp = b_jpvt_0->size[0] * b_jpvt_0->size[1];
      b_jpvt_0->size[0] = 1;
      b_jpvt_0->size[1] = b_jpvt->size[1];
      Cinem_emxEnsureCapacity_int32_T(b_jpvt_0, Cinematique_ROS_B.itemp);
      Cinematique_ROS_B.ix_o = b_jpvt->size[0] * b_jpvt->size[1] - 1;
      for (Cinematique_ROS_B.itemp = 0; Cinematique_ROS_B.itemp <=
           Cinematique_ROS_B.ix_o; Cinematique_ROS_B.itemp++) {
        b_jpvt_0->data[Cinematique_ROS_B.itemp] = b_jpvt->
          data[Cinematique_ROS_B.itemp];
      }

      Cinematique_ROS_B.itemp = d_A->size[0] * d_A->size[1];
      d_A->size[0] = A->size[0];
      d_A->size[1] = A->size[1];
      Cinema_emxEnsureCapacity_real_T(d_A, Cinematique_ROS_B.itemp);
      Cinematique_ROS_B.ix_o = A->size[0] * A->size[1] - 1;
      for (Cinematique_ROS_B.itemp = 0; Cinematique_ROS_B.itemp <=
           Cinematique_ROS_B.ix_o; Cinematique_ROS_B.itemp++) {
        d_A->data[Cinematique_ROS_B.itemp] = A->data[Cinematique_ROS_B.itemp];
      }

      Cinematique_ROS_B.minmana = A->size[0];
      Cinematique_ROS_B.u0_f = A->size[0];
      Cinematique_ROS_B.minmn_i = A->size[1];
      if (Cinematique_ROS_B.u0_f < Cinematique_ROS_B.minmn_i) {
        Cinematique_ROS_B.minmn_i = Cinematique_ROS_B.u0_f;
      }

      Cinematique_ROS_B.minmn_i--;
      Cinematique_ROS_B.itemp = work->size[0];
      work->size[0] = A->size[1];
      Cinema_emxEnsureCapacity_real_T(work, Cinematique_ROS_B.itemp);
      Cinematique_ROS_B.ix_o = A->size[1];
      for (Cinematique_ROS_B.itemp = 0; Cinematique_ROS_B.itemp <
           Cinematique_ROS_B.ix_o; Cinematique_ROS_B.itemp++) {
        work->data[Cinematique_ROS_B.itemp] = 0.0;
      }

      Cinematique_ROS_B.itemp = vn1->size[0];
      vn1->size[0] = A->size[1];
      Cinema_emxEnsureCapacity_real_T(vn1, Cinematique_ROS_B.itemp);
      Cinematique_ROS_B.ix_o = A->size[1];
      for (Cinematique_ROS_B.itemp = 0; Cinematique_ROS_B.itemp <
           Cinematique_ROS_B.ix_o; Cinematique_ROS_B.itemp++) {
        vn1->data[Cinematique_ROS_B.itemp] = 0.0;
      }

      Cinematique_ROS_B.itemp = vn2->size[0];
      vn2->size[0] = A->size[1];
      Cinema_emxEnsureCapacity_real_T(vn2, Cinematique_ROS_B.itemp);
      Cinematique_ROS_B.ix_o = A->size[1];
      for (Cinematique_ROS_B.itemp = 0; Cinematique_ROS_B.itemp <
           Cinematique_ROS_B.ix_o; Cinematique_ROS_B.itemp++) {
        vn2->data[Cinematique_ROS_B.itemp] = 0.0;
      }

      for (Cinematique_ROS_B.u0_f = 0; Cinematique_ROS_B.u0_f <
           Cinematique_ROS_B.na; Cinematique_ROS_B.u0_f++) {
        Cinematique_ROS_B.pvt = Cinematique_ROS_B.u0_f *
          Cinematique_ROS_B.minmana;
        Cinematique_ROS_B.smax = 0.0;
        if (Cinematique_ROS_B.ma >= 1) {
          if (Cinematique_ROS_B.ma == 1) {
            Cinematique_ROS_B.smax = fabs(A->data[Cinematique_ROS_B.pvt]);
          } else {
            Cinematique_ROS_B.scale_c = 3.3121686421112381E-170;
            Cinematique_ROS_B.kend = Cinematique_ROS_B.pvt +
              Cinematique_ROS_B.ma;
            for (Cinematique_ROS_B.itemp = Cinematique_ROS_B.pvt + 1;
                 Cinematique_ROS_B.itemp <= Cinematique_ROS_B.kend;
                 Cinematique_ROS_B.itemp++) {
              Cinematique_ROS_B.absxk_c = fabs(A->data[Cinematique_ROS_B.itemp -
                1]);
              if (Cinematique_ROS_B.absxk_c > Cinematique_ROS_B.scale_c) {
                Cinematique_ROS_B.t_p = Cinematique_ROS_B.scale_c /
                  Cinematique_ROS_B.absxk_c;
                Cinematique_ROS_B.smax = Cinematique_ROS_B.smax *
                  Cinematique_ROS_B.t_p * Cinematique_ROS_B.t_p + 1.0;
                Cinematique_ROS_B.scale_c = Cinematique_ROS_B.absxk_c;
              } else {
                Cinematique_ROS_B.t_p = Cinematique_ROS_B.absxk_c /
                  Cinematique_ROS_B.scale_c;
                Cinematique_ROS_B.smax += Cinematique_ROS_B.t_p *
                  Cinematique_ROS_B.t_p;
              }
            }

            Cinematique_ROS_B.smax = Cinematique_ROS_B.scale_c * sqrt
              (Cinematique_ROS_B.smax);
          }
        }

        vn1->data[Cinematique_ROS_B.u0_f] = Cinematique_ROS_B.smax;
        vn2->data[Cinematique_ROS_B.u0_f] = vn1->data[Cinematique_ROS_B.u0_f];
      }

      for (Cinematique_ROS_B.u0_f = 0; Cinematique_ROS_B.u0_f <=
           Cinematique_ROS_B.minmn_i; Cinematique_ROS_B.u0_f++) {
        Cinematique_ROS_B.iy = Cinematique_ROS_B.u0_f *
          Cinematique_ROS_B.minmana;
        Cinematique_ROS_B.ii = Cinematique_ROS_B.iy + Cinematique_ROS_B.u0_f;
        Cinematique_ROS_B.kend = Cinematique_ROS_B.na - Cinematique_ROS_B.u0_f;
        Cinematique_ROS_B.mmi = (Cinematique_ROS_B.ma - Cinematique_ROS_B.u0_f)
          - 1;
        if (Cinematique_ROS_B.kend < 1) {
          Cinematique_ROS_B.pvt = 0;
        } else {
          Cinematique_ROS_B.pvt = 1;
          if (Cinematique_ROS_B.kend > 1) {
            Cinematique_ROS_B.ix_o = Cinematique_ROS_B.u0_f;
            Cinematique_ROS_B.smax = fabs(vn1->data[Cinematique_ROS_B.u0_f]);
            for (Cinematique_ROS_B.itemp = 2; Cinematique_ROS_B.itemp <=
                 Cinematique_ROS_B.kend; Cinematique_ROS_B.itemp++) {
              Cinematique_ROS_B.ix_o++;
              Cinematique_ROS_B.scale_c = fabs(vn1->data[Cinematique_ROS_B.ix_o]);
              if (Cinematique_ROS_B.scale_c > Cinematique_ROS_B.smax) {
                Cinematique_ROS_B.pvt = Cinematique_ROS_B.itemp;
                Cinematique_ROS_B.smax = Cinematique_ROS_B.scale_c;
              }
            }
          }
        }

        Cinematique_ROS_B.pvt = (Cinematique_ROS_B.u0_f + Cinematique_ROS_B.pvt)
          - 1;
        if (Cinematique_ROS_B.pvt + 1 != Cinematique_ROS_B.u0_f + 1) {
          Cinematique_ROS_B.itemp = c_x->size[0] * c_x->size[1];
          c_x->size[0] = d_A->size[0];
          c_x->size[1] = d_A->size[1];
          Cinema_emxEnsureCapacity_real_T(c_x, Cinematique_ROS_B.itemp);
          Cinematique_ROS_B.ix_o = d_A->size[0] * d_A->size[1] - 1;
          for (Cinematique_ROS_B.itemp = 0; Cinematique_ROS_B.itemp <=
               Cinematique_ROS_B.ix_o; Cinematique_ROS_B.itemp++) {
            c_x->data[Cinematique_ROS_B.itemp] = d_A->
              data[Cinematique_ROS_B.itemp];
          }

          Cinematique_ROS_B.ix_o = Cinematique_ROS_B.pvt *
            Cinematique_ROS_B.minmana;
          for (Cinematique_ROS_B.itemp = 0; Cinematique_ROS_B.itemp <
               Cinematique_ROS_B.ma; Cinematique_ROS_B.itemp++) {
            Cinematique_ROS_B.scale_c = c_x->data[Cinematique_ROS_B.ix_o];
            c_x->data[Cinematique_ROS_B.ix_o] = c_x->data[Cinematique_ROS_B.iy];
            c_x->data[Cinematique_ROS_B.iy] = Cinematique_ROS_B.scale_c;
            Cinematique_ROS_B.ix_o++;
            Cinematique_ROS_B.iy++;
          }

          Cinematique_ROS_B.itemp = d_A->size[0] * d_A->size[1];
          d_A->size[0] = c_x->size[0];
          d_A->size[1] = c_x->size[1];
          Cinema_emxEnsureCapacity_real_T(d_A, Cinematique_ROS_B.itemp);
          Cinematique_ROS_B.ix_o = c_x->size[0] * c_x->size[1] - 1;
          for (Cinematique_ROS_B.itemp = 0; Cinematique_ROS_B.itemp <=
               Cinematique_ROS_B.ix_o; Cinematique_ROS_B.itemp++) {
            d_A->data[Cinematique_ROS_B.itemp] = c_x->
              data[Cinematique_ROS_B.itemp];
          }

          Cinematique_ROS_B.itemp = b_jpvt_0->data[Cinematique_ROS_B.pvt];
          b_jpvt_0->data[Cinematique_ROS_B.pvt] = b_jpvt_0->
            data[Cinematique_ROS_B.u0_f];
          b_jpvt_0->data[Cinematique_ROS_B.u0_f] = Cinematique_ROS_B.itemp;
          vn1->data[Cinematique_ROS_B.pvt] = vn1->data[Cinematique_ROS_B.u0_f];
          vn2->data[Cinematique_ROS_B.pvt] = vn2->data[Cinematique_ROS_B.u0_f];
        }

        if (Cinematique_ROS_B.u0_f + 1 < Cinematique_ROS_B.ma) {
          Cinematique_ROS_B.pvt = Cinematique_ROS_B.ii + 2;
          Cinematique_ROS_B.itemp = c_x->size[0] * c_x->size[1];
          c_x->size[0] = d_A->size[0];
          c_x->size[1] = d_A->size[1];
          Cinema_emxEnsureCapacity_real_T(c_x, Cinematique_ROS_B.itemp);
          Cinematique_ROS_B.ix_o = d_A->size[0] * d_A->size[1] - 1;
          for (Cinematique_ROS_B.itemp = 0; Cinematique_ROS_B.itemp <=
               Cinematique_ROS_B.ix_o; Cinematique_ROS_B.itemp++) {
            c_x->data[Cinematique_ROS_B.itemp] = d_A->
              data[Cinematique_ROS_B.itemp];
          }

          Cinematique_ROS_B.smax = d_A->data[Cinematique_ROS_B.ii];
          tau->data[Cinematique_ROS_B.u0_f] = 0.0;
          if (Cinematique_ROS_B.mmi + 1 > 0) {
            Cinematique_ROS_B.scale_c = Cinematique_ROS_xnrm2_k1
              (Cinematique_ROS_B.mmi, d_A, Cinematique_ROS_B.ii + 2);
            if (Cinematique_ROS_B.scale_c != 0.0) {
              Cinematique_ROS_B.scale_c = rt_hypotd_snf(d_A->
                data[Cinematique_ROS_B.ii], Cinematique_ROS_B.scale_c);
              if (d_A->data[Cinematique_ROS_B.ii] >= 0.0) {
                Cinematique_ROS_B.scale_c = -Cinematique_ROS_B.scale_c;
              }

              if (fabs(Cinematique_ROS_B.scale_c) < 1.0020841800044864E-292) {
                Cinematique_ROS_B.knt = -1;
                Cinematique_ROS_B.ix_o = (Cinematique_ROS_B.ii +
                  Cinematique_ROS_B.mmi) + 1;
                do {
                  Cinematique_ROS_B.knt++;
                  for (Cinematique_ROS_B.itemp = Cinematique_ROS_B.pvt;
                       Cinematique_ROS_B.itemp <= Cinematique_ROS_B.ix_o;
                       Cinematique_ROS_B.itemp++) {
                    c_x->data[Cinematique_ROS_B.itemp - 1] *=
                      9.9792015476736E+291;
                  }

                  Cinematique_ROS_B.scale_c *= 9.9792015476736E+291;
                  Cinematique_ROS_B.smax *= 9.9792015476736E+291;
                } while (!(fabs(Cinematique_ROS_B.scale_c) >=
                           1.0020841800044864E-292));

                Cinematique_ROS_B.scale_c = rt_hypotd_snf(Cinematique_ROS_B.smax,
                  Cinematique_ROS_xnrm2_k1(Cinematique_ROS_B.mmi, c_x,
                  Cinematique_ROS_B.ii + 2));
                if (Cinematique_ROS_B.smax >= 0.0) {
                  Cinematique_ROS_B.scale_c = -Cinematique_ROS_B.scale_c;
                }

                tau->data[Cinematique_ROS_B.u0_f] = (Cinematique_ROS_B.scale_c -
                  Cinematique_ROS_B.smax) / Cinematique_ROS_B.scale_c;
                Cinematique_ROS_B.smax = 1.0 / (Cinematique_ROS_B.smax -
                  Cinematique_ROS_B.scale_c);
                for (Cinematique_ROS_B.itemp = Cinematique_ROS_B.pvt;
                     Cinematique_ROS_B.itemp <= Cinematique_ROS_B.ix_o;
                     Cinematique_ROS_B.itemp++) {
                  c_x->data[Cinematique_ROS_B.itemp - 1] *=
                    Cinematique_ROS_B.smax;
                }

                for (Cinematique_ROS_B.itemp = 0; Cinematique_ROS_B.itemp <=
                     Cinematique_ROS_B.knt; Cinematique_ROS_B.itemp++) {
                  Cinematique_ROS_B.scale_c *= 1.0020841800044864E-292;
                }

                Cinematique_ROS_B.smax = Cinematique_ROS_B.scale_c;
              } else {
                tau->data[Cinematique_ROS_B.u0_f] = (Cinematique_ROS_B.scale_c -
                  d_A->data[Cinematique_ROS_B.ii]) / Cinematique_ROS_B.scale_c;
                Cinematique_ROS_B.smax = 1.0 / (d_A->data[Cinematique_ROS_B.ii]
                  - Cinematique_ROS_B.scale_c);
                Cinematique_ROS_B.itemp = c_x->size[0] * c_x->size[1];
                c_x->size[0] = d_A->size[0];
                c_x->size[1] = d_A->size[1];
                Cinema_emxEnsureCapacity_real_T(c_x, Cinematique_ROS_B.itemp);
                Cinematique_ROS_B.ix_o = d_A->size[0] * d_A->size[1] - 1;
                for (Cinematique_ROS_B.itemp = 0; Cinematique_ROS_B.itemp <=
                     Cinematique_ROS_B.ix_o; Cinematique_ROS_B.itemp++) {
                  c_x->data[Cinematique_ROS_B.itemp] = d_A->
                    data[Cinematique_ROS_B.itemp];
                }

                Cinematique_ROS_B.b_cu = (Cinematique_ROS_B.ii +
                  Cinematique_ROS_B.mmi) + 1;
                for (Cinematique_ROS_B.itemp = Cinematique_ROS_B.pvt;
                     Cinematique_ROS_B.itemp <= Cinematique_ROS_B.b_cu;
                     Cinematique_ROS_B.itemp++) {
                  c_x->data[Cinematique_ROS_B.itemp - 1] *=
                    Cinematique_ROS_B.smax;
                }

                Cinematique_ROS_B.smax = Cinematique_ROS_B.scale_c;
              }
            }
          }

          Cinematique_ROS_B.itemp = d_A->size[0] * d_A->size[1];
          d_A->size[0] = c_x->size[0];
          d_A->size[1] = c_x->size[1];
          Cinema_emxEnsureCapacity_real_T(d_A, Cinematique_ROS_B.itemp);
          Cinematique_ROS_B.ix_o = c_x->size[0] * c_x->size[1] - 1;
          for (Cinematique_ROS_B.itemp = 0; Cinematique_ROS_B.itemp <=
               Cinematique_ROS_B.ix_o; Cinematique_ROS_B.itemp++) {
            d_A->data[Cinematique_ROS_B.itemp] = c_x->
              data[Cinematique_ROS_B.itemp];
          }

          d_A->data[Cinematique_ROS_B.ii] = Cinematique_ROS_B.smax;
        } else {
          tau->data[Cinematique_ROS_B.u0_f] = 0.0;
        }

        if (Cinematique_ROS_B.u0_f + 1 < Cinematique_ROS_B.na) {
          Cinematique_ROS_B.smax = d_A->data[Cinematique_ROS_B.ii];
          d_A->data[Cinematique_ROS_B.ii] = 1.0;
          Cinematique_ROS_B.knt = (Cinematique_ROS_B.ii +
            Cinematique_ROS_B.minmana) + 1;
          Cinematique_ROS_B.itemp = c_x->size[0] * c_x->size[1];
          c_x->size[0] = d_A->size[0];
          c_x->size[1] = d_A->size[1];
          Cinema_emxEnsureCapacity_real_T(c_x, Cinematique_ROS_B.itemp);
          Cinematique_ROS_B.ix_o = d_A->size[0] * d_A->size[1] - 1;
          for (Cinematique_ROS_B.itemp = 0; Cinematique_ROS_B.itemp <=
               Cinematique_ROS_B.ix_o; Cinematique_ROS_B.itemp++) {
            c_x->data[Cinematique_ROS_B.itemp] = d_A->
              data[Cinematique_ROS_B.itemp];
          }

          if (tau->data[Cinematique_ROS_B.u0_f] != 0.0) {
            Cinematique_ROS_B.pvt = Cinematique_ROS_B.mmi;
            Cinematique_ROS_B.itemp = Cinematique_ROS_B.ii +
              Cinematique_ROS_B.mmi;
            while ((Cinematique_ROS_B.pvt + 1 > 0) && (d_A->
                    data[Cinematique_ROS_B.itemp] == 0.0)) {
              Cinematique_ROS_B.pvt--;
              Cinematique_ROS_B.itemp--;
            }

            Cinematique_ROS_B.kend--;
            exitg2 = false;
            while ((!exitg2) && (Cinematique_ROS_B.kend > 0)) {
              Cinematique_ROS_B.ix_o = (Cinematique_ROS_B.kend - 1) *
                Cinematique_ROS_B.minmana + Cinematique_ROS_B.knt;
              Cinematique_ROS_B.itemp = Cinematique_ROS_B.ix_o;
              do {
                exitg1 = 0;
                if (Cinematique_ROS_B.itemp <= Cinematique_ROS_B.ix_o +
                    Cinematique_ROS_B.pvt) {
                  if (d_A->data[Cinematique_ROS_B.itemp - 1] != 0.0) {
                    exitg1 = 1;
                  } else {
                    Cinematique_ROS_B.itemp++;
                  }
                } else {
                  Cinematique_ROS_B.kend--;
                  exitg1 = 2;
                }
              } while (exitg1 == 0);

              if (exitg1 == 1) {
                exitg2 = true;
              }
            }

            Cinematique_ROS_B.lastc = Cinematique_ROS_B.kend - 1;
            Cinematique_ROS_B.itemp = c_x->size[0] * c_x->size[1];
            c_x->size[0] = d_A->size[0];
            c_x->size[1] = d_A->size[1];
            Cinema_emxEnsureCapacity_real_T(c_x, Cinematique_ROS_B.itemp);
            Cinematique_ROS_B.ix_o = d_A->size[0] * d_A->size[1] - 1;
            for (Cinematique_ROS_B.itemp = 0; Cinematique_ROS_B.itemp <=
                 Cinematique_ROS_B.ix_o; Cinematique_ROS_B.itemp++) {
              c_x->data[Cinematique_ROS_B.itemp] = d_A->
                data[Cinematique_ROS_B.itemp];
            }
          } else {
            Cinematique_ROS_B.pvt = -1;
            Cinematique_ROS_B.lastc = -1;
          }

          if (Cinematique_ROS_B.pvt + 1 > 0) {
            if (Cinematique_ROS_B.lastc + 1 != 0) {
              for (Cinematique_ROS_B.itemp = 0; Cinematique_ROS_B.itemp <=
                   Cinematique_ROS_B.lastc; Cinematique_ROS_B.itemp++) {
                work->data[Cinematique_ROS_B.itemp] = 0.0;
              }

              Cinematique_ROS_B.iy = 0;
              Cinematique_ROS_B.b_cu = Cinematique_ROS_B.minmana *
                Cinematique_ROS_B.lastc + Cinematique_ROS_B.knt;
              for (Cinematique_ROS_B.kend = Cinematique_ROS_B.knt;
                   Cinematique_ROS_B.minmana < 0 ? Cinematique_ROS_B.kend >=
                   Cinematique_ROS_B.b_cu : Cinematique_ROS_B.kend <=
                   Cinematique_ROS_B.b_cu; Cinematique_ROS_B.kend +=
                   Cinematique_ROS_B.minmana) {
                Cinematique_ROS_B.ix_o = Cinematique_ROS_B.ii;
                Cinematique_ROS_B.scale_c = 0.0;
                Cinematique_ROS_B.d_m4 = Cinematique_ROS_B.kend +
                  Cinematique_ROS_B.pvt;
                for (Cinematique_ROS_B.itemp = Cinematique_ROS_B.kend;
                     Cinematique_ROS_B.itemp <= Cinematique_ROS_B.d_m4;
                     Cinematique_ROS_B.itemp++) {
                  Cinematique_ROS_B.scale_c += c_x->data[Cinematique_ROS_B.itemp
                    - 1] * c_x->data[Cinematique_ROS_B.ix_o];
                  Cinematique_ROS_B.ix_o++;
                }

                work->data[Cinematique_ROS_B.iy] += Cinematique_ROS_B.scale_c;
                Cinematique_ROS_B.iy++;
              }
            }

            if (!(-tau->data[Cinematique_ROS_B.u0_f] == 0.0)) {
              Cinematique_ROS_B.iy = 0;
              for (Cinematique_ROS_B.itemp = 0; Cinematique_ROS_B.itemp <=
                   Cinematique_ROS_B.lastc; Cinematique_ROS_B.itemp++) {
                if (work->data[Cinematique_ROS_B.iy] != 0.0) {
                  Cinematique_ROS_B.scale_c = work->data[Cinematique_ROS_B.iy] *
                    -tau->data[Cinematique_ROS_B.u0_f];
                  Cinematique_ROS_B.ix_o = Cinematique_ROS_B.ii;
                  Cinematique_ROS_B.b_cu = Cinematique_ROS_B.pvt +
                    Cinematique_ROS_B.knt;
                  for (Cinematique_ROS_B.kend = Cinematique_ROS_B.knt;
                       Cinematique_ROS_B.kend <= Cinematique_ROS_B.b_cu;
                       Cinematique_ROS_B.kend++) {
                    c_x->data[Cinematique_ROS_B.kend - 1] += c_x->
                      data[Cinematique_ROS_B.ix_o] * Cinematique_ROS_B.scale_c;
                    Cinematique_ROS_B.ix_o++;
                  }
                }

                Cinematique_ROS_B.iy++;
                Cinematique_ROS_B.knt += Cinematique_ROS_B.minmana;
              }
            }
          }

          Cinematique_ROS_B.itemp = d_A->size[0] * d_A->size[1];
          d_A->size[0] = c_x->size[0];
          d_A->size[1] = c_x->size[1];
          Cinema_emxEnsureCapacity_real_T(d_A, Cinematique_ROS_B.itemp);
          Cinematique_ROS_B.ix_o = c_x->size[0] * c_x->size[1] - 1;
          for (Cinematique_ROS_B.itemp = 0; Cinematique_ROS_B.itemp <=
               Cinematique_ROS_B.ix_o; Cinematique_ROS_B.itemp++) {
            d_A->data[Cinematique_ROS_B.itemp] = c_x->
              data[Cinematique_ROS_B.itemp];
          }

          d_A->data[Cinematique_ROS_B.ii] = Cinematique_ROS_B.smax;
        }

        for (Cinematique_ROS_B.ii = Cinematique_ROS_B.u0_f + 2;
             Cinematique_ROS_B.ii <= Cinematique_ROS_B.na; Cinematique_ROS_B.ii
             ++) {
          Cinematique_ROS_B.pvt = ((Cinematique_ROS_B.ii - 1) *
            Cinematique_ROS_B.minmana + Cinematique_ROS_B.u0_f) + 1;
          if (vn1->data[Cinematique_ROS_B.ii - 1] != 0.0) {
            Cinematique_ROS_B.smax = fabs(d_A->data[Cinematique_ROS_B.pvt - 1]) /
              vn1->data[Cinematique_ROS_B.ii - 1];
            Cinematique_ROS_B.smax = 1.0 - Cinematique_ROS_B.smax *
              Cinematique_ROS_B.smax;
            if (Cinematique_ROS_B.smax < 0.0) {
              Cinematique_ROS_B.smax = 0.0;
            }

            Cinematique_ROS_B.scale_c = vn1->data[Cinematique_ROS_B.ii - 1] /
              vn2->data[Cinematique_ROS_B.ii - 1];
            Cinematique_ROS_B.scale_c = Cinematique_ROS_B.scale_c *
              Cinematique_ROS_B.scale_c * Cinematique_ROS_B.smax;
            if (Cinematique_ROS_B.scale_c <= 1.4901161193847656E-8) {
              if (Cinematique_ROS_B.u0_f + 1 < Cinematique_ROS_B.ma) {
                Cinematique_ROS_B.smax = 0.0;
                if (Cinematique_ROS_B.mmi >= 1) {
                  if (Cinematique_ROS_B.mmi == 1) {
                    Cinematique_ROS_B.smax = fabs(d_A->
                      data[Cinematique_ROS_B.pvt]);
                  } else {
                    Cinematique_ROS_B.scale_c = 3.3121686421112381E-170;
                    Cinematique_ROS_B.kend = Cinematique_ROS_B.pvt +
                      Cinematique_ROS_B.mmi;
                    for (Cinematique_ROS_B.itemp = Cinematique_ROS_B.pvt + 1;
                         Cinematique_ROS_B.itemp <= Cinematique_ROS_B.kend;
                         Cinematique_ROS_B.itemp++) {
                      Cinematique_ROS_B.absxk_c = fabs(d_A->
                        data[Cinematique_ROS_B.itemp - 1]);
                      if (Cinematique_ROS_B.absxk_c > Cinematique_ROS_B.scale_c)
                      {
                        Cinematique_ROS_B.t_p = Cinematique_ROS_B.scale_c /
                          Cinematique_ROS_B.absxk_c;
                        Cinematique_ROS_B.smax = Cinematique_ROS_B.smax *
                          Cinematique_ROS_B.t_p * Cinematique_ROS_B.t_p + 1.0;
                        Cinematique_ROS_B.scale_c = Cinematique_ROS_B.absxk_c;
                      } else {
                        Cinematique_ROS_B.t_p = Cinematique_ROS_B.absxk_c /
                          Cinematique_ROS_B.scale_c;
                        Cinematique_ROS_B.smax += Cinematique_ROS_B.t_p *
                          Cinematique_ROS_B.t_p;
                      }
                    }

                    Cinematique_ROS_B.smax = Cinematique_ROS_B.scale_c * sqrt
                      (Cinematique_ROS_B.smax);
                  }
                }

                vn1->data[Cinematique_ROS_B.ii - 1] = Cinematique_ROS_B.smax;
                vn2->data[Cinematique_ROS_B.ii - 1] = vn1->
                  data[Cinematique_ROS_B.ii - 1];
              } else {
                vn1->data[Cinematique_ROS_B.ii - 1] = 0.0;
                vn2->data[Cinematique_ROS_B.ii - 1] = 0.0;
              }
            } else {
              vn1->data[Cinematique_ROS_B.ii - 1] *= sqrt(Cinematique_ROS_B.smax);
            }
          }
        }
      }

      Cinematique_ROS_B.itemp = b_A->size[0] * b_A->size[1];
      b_A->size[0] = d_A->size[0];
      b_A->size[1] = d_A->size[1];
      Cinema_emxEnsureCapacity_real_T(b_A, Cinematique_ROS_B.itemp);
      Cinematique_ROS_B.ix_o = d_A->size[0] * d_A->size[1] - 1;
      for (Cinematique_ROS_B.itemp = 0; Cinematique_ROS_B.itemp <=
           Cinematique_ROS_B.ix_o; Cinematique_ROS_B.itemp++) {
        b_A->data[Cinematique_ROS_B.itemp] = d_A->data[Cinematique_ROS_B.itemp];
      }

      Cinematique_ROS_B.itemp = b_jpvt->size[0] * b_jpvt->size[1];
      b_jpvt->size[0] = 1;
      b_jpvt->size[1] = b_jpvt_0->size[1];
      Cinem_emxEnsureCapacity_int32_T(b_jpvt, Cinematique_ROS_B.itemp);
      Cinematique_ROS_B.ix_o = b_jpvt_0->size[0] * b_jpvt_0->size[1] - 1;
      for (Cinematique_ROS_B.itemp = 0; Cinematique_ROS_B.itemp <=
           Cinematique_ROS_B.ix_o; Cinematique_ROS_B.itemp++) {
        b_jpvt->data[Cinematique_ROS_B.itemp] = b_jpvt_0->
          data[Cinematique_ROS_B.itemp];
      }
    }
  }

  if (guard1) {
    Cinematique_ROS_B.itemp = b_jpvt->size[0] * b_jpvt->size[1];
    b_jpvt->size[0] = 1;
    b_jpvt->size[1] = A->size[1];
    Cinem_emxEnsureCapacity_int32_T(b_jpvt, Cinematique_ROS_B.itemp);
    Cinematique_ROS_B.ix_o = A->size[1] - 1;
    for (Cinematique_ROS_B.itemp = 0; Cinematique_ROS_B.itemp <=
         Cinematique_ROS_B.ix_o; Cinematique_ROS_B.itemp++) {
      b_jpvt->data[Cinematique_ROS_B.itemp] = 0;
    }

    for (Cinematique_ROS_B.u0_f = 0; Cinematique_ROS_B.u0_f <
         Cinematique_ROS_B.na; Cinematique_ROS_B.u0_f++) {
      b_jpvt->data[Cinematique_ROS_B.u0_f] = Cinematique_ROS_B.u0_f + 1;
    }
  }

  Cinematique_ROS_emxFree_real_T(&c_x);
  Cinematique_ROS_emxFree_real_T(&vn2);
  Cinematique_ROS_emxFree_real_T(&vn1);
  Cinematique_ROS_emxFree_real_T(&work);
  Cinematique_ROS_emxFree_int32_T(&b_jpvt_0);
  Cinematique_ROS_emxFree_real_T(&d_A);
  Cinematique_ROS_B.itemp = jpvt->size[0] * jpvt->size[1];
  jpvt->size[0] = 1;
  jpvt->size[1] = b_jpvt->size[1];
  Cinem_emxEnsureCapacity_int32_T(jpvt, Cinematique_ROS_B.itemp);
  Cinematique_ROS_B.ix_o = b_jpvt->size[0] * b_jpvt->size[1] - 1;
  for (Cinematique_ROS_B.itemp = 0; Cinematique_ROS_B.itemp <=
       Cinematique_ROS_B.ix_o; Cinematique_ROS_B.itemp++) {
    jpvt->data[Cinematique_ROS_B.itemp] = b_jpvt->data[Cinematique_ROS_B.itemp];
  }

  Cinematique_ROS_emxFree_int32_T(&b_jpvt);
}

static void Cinematique_ROS_xzgetrf(int32_T m, int32_T n, const
  emxArray_real_T_Cinematique_R_T *A, int32_T lda,
  emxArray_real_T_Cinematique_R_T *b_A, emxArray_int32_T_Cinematique__T *ipiv,
  int32_T *info)
{
  emxArray_real_T_Cinematique_R_T *c_x;
  Cinematique_ROS_B.k_p = b_A->size[0] * b_A->size[1];
  b_A->size[0] = A->size[0];
  b_A->size[1] = A->size[1];
  Cinema_emxEnsureCapacity_real_T(b_A, Cinematique_ROS_B.k_p);
  Cinematique_ROS_B.ix_n = A->size[0] * A->size[1] - 1;
  for (Cinematique_ROS_B.k_p = 0; Cinematique_ROS_B.k_p <=
       Cinematique_ROS_B.ix_n; Cinematique_ROS_B.k_p++) {
    b_A->data[Cinematique_ROS_B.k_p] = A->data[Cinematique_ROS_B.k_p];
  }

  if (m < n) {
    Cinematique_ROS_B.n_f = m;
  } else {
    Cinematique_ROS_B.n_f = n;
  }

  if (Cinematique_ROS_B.n_f < 1) {
    Cinematique_ROS_B.n_f = 0;
  }

  Cinematique_ROS_B.k_p = ipiv->size[0] * ipiv->size[1];
  ipiv->size[0] = 1;
  ipiv->size[1] = Cinematique_ROS_B.n_f;
  Cinem_emxEnsureCapacity_int32_T(ipiv, Cinematique_ROS_B.k_p);
  if (Cinematique_ROS_B.n_f > 0) {
    ipiv->data[0] = 1;
    Cinematique_ROS_B.yk_p = 1;
    for (Cinematique_ROS_B.k_p = 2; Cinematique_ROS_B.k_p <=
         Cinematique_ROS_B.n_f; Cinematique_ROS_B.k_p++) {
      Cinematique_ROS_B.yk_p++;
      ipiv->data[Cinematique_ROS_B.k_p - 1] = Cinematique_ROS_B.yk_p;
    }
  }

  Cinematique_ROS_B.yk_p = 0;
  Cinematique_ROS_emxInit_real_T(&c_x, 2);
  if ((m < 1) || (n < 1)) {
  } else {
    if (m - 1 < n) {
      Cinematique_ROS_B.n_f = m - 1;
    } else {
      Cinematique_ROS_B.n_f = n;
    }

    Cinematique_ROS_B.b_gl = Cinematique_ROS_B.n_f - 1;
    for (Cinematique_ROS_B.n_f = 0; Cinematique_ROS_B.n_f <=
         Cinematique_ROS_B.b_gl; Cinematique_ROS_B.n_f++) {
      Cinematique_ROS_B.mmj = m - Cinematique_ROS_B.n_f;
      Cinematique_ROS_B.c_tmp = (lda + 1) * Cinematique_ROS_B.n_f;
      Cinematique_ROS_B.c_d = Cinematique_ROS_B.c_tmp + 2;
      if (Cinematique_ROS_B.mmj < 1) {
        Cinematique_ROS_B.iy_c = 0;
      } else {
        Cinematique_ROS_B.iy_c = 1;
        if (Cinematique_ROS_B.mmj > 1) {
          Cinematique_ROS_B.ix_n = Cinematique_ROS_B.c_tmp;
          Cinematique_ROS_B.smax_i = fabs(b_A->data[Cinematique_ROS_B.c_tmp]);
          for (Cinematique_ROS_B.k_p = 2; Cinematique_ROS_B.k_p <=
               Cinematique_ROS_B.mmj; Cinematique_ROS_B.k_p++) {
            Cinematique_ROS_B.ix_n++;
            Cinematique_ROS_B.s_l = fabs(b_A->data[Cinematique_ROS_B.ix_n]);
            if (Cinematique_ROS_B.s_l > Cinematique_ROS_B.smax_i) {
              Cinematique_ROS_B.iy_c = Cinematique_ROS_B.k_p;
              Cinematique_ROS_B.smax_i = Cinematique_ROS_B.s_l;
            }
          }
        }
      }

      if (b_A->data[(Cinematique_ROS_B.c_tmp + Cinematique_ROS_B.iy_c) - 1] !=
          0.0) {
        if (Cinematique_ROS_B.iy_c - 1 != 0) {
          Cinematique_ROS_B.k_p = Cinematique_ROS_B.n_f + Cinematique_ROS_B.iy_c;
          ipiv->data[Cinematique_ROS_B.n_f] = Cinematique_ROS_B.k_p;
          Cinematique_ROS_B.iy_c = c_x->size[0] * c_x->size[1];
          c_x->size[0] = b_A->size[0];
          c_x->size[1] = b_A->size[1];
          Cinema_emxEnsureCapacity_real_T(c_x, Cinematique_ROS_B.iy_c);
          Cinematique_ROS_B.ix_n = b_A->size[0] * b_A->size[1] - 1;
          for (Cinematique_ROS_B.iy_c = 0; Cinematique_ROS_B.iy_c <=
               Cinematique_ROS_B.ix_n; Cinematique_ROS_B.iy_c++) {
            c_x->data[Cinematique_ROS_B.iy_c] = b_A->data[Cinematique_ROS_B.iy_c];
          }

          Cinematique_ROS_B.ix_n = Cinematique_ROS_B.n_f;
          Cinematique_ROS_B.iy_c = Cinematique_ROS_B.k_p - 1;
          for (Cinematique_ROS_B.k_p = 0; Cinematique_ROS_B.k_p < n;
               Cinematique_ROS_B.k_p++) {
            Cinematique_ROS_B.smax_i = c_x->data[Cinematique_ROS_B.ix_n];
            c_x->data[Cinematique_ROS_B.ix_n] = c_x->data[Cinematique_ROS_B.iy_c];
            c_x->data[Cinematique_ROS_B.iy_c] = Cinematique_ROS_B.smax_i;
            Cinematique_ROS_B.ix_n += lda;
            Cinematique_ROS_B.iy_c += lda;
          }

          Cinematique_ROS_B.k_p = b_A->size[0] * b_A->size[1];
          b_A->size[0] = c_x->size[0];
          b_A->size[1] = c_x->size[1];
          Cinema_emxEnsureCapacity_real_T(b_A, Cinematique_ROS_B.k_p);
          Cinematique_ROS_B.ix_n = c_x->size[0] * c_x->size[1] - 1;
          for (Cinematique_ROS_B.k_p = 0; Cinematique_ROS_B.k_p <=
               Cinematique_ROS_B.ix_n; Cinematique_ROS_B.k_p++) {
            b_A->data[Cinematique_ROS_B.k_p] = c_x->data[Cinematique_ROS_B.k_p];
          }
        }

        Cinematique_ROS_B.c_n = (Cinematique_ROS_B.c_tmp + Cinematique_ROS_B.mmj)
          + 2;
        for (Cinematique_ROS_B.k_p = Cinematique_ROS_B.c_d;
             Cinematique_ROS_B.k_p <= Cinematique_ROS_B.c_n - 2;
             Cinematique_ROS_B.k_p++) {
          b_A->data[Cinematique_ROS_B.k_p - 1] /= b_A->
            data[Cinematique_ROS_B.c_tmp];
        }
      } else {
        Cinematique_ROS_B.yk_p = Cinematique_ROS_B.n_f + 1;
      }

      Cinematique_ROS_B.c_d = (n - Cinematique_ROS_B.n_f) - 2;
      Cinematique_ROS_B.jy = Cinematique_ROS_B.c_tmp + lda;
      Cinematique_ROS_B.jA = Cinematique_ROS_B.jy + 1;
      for (Cinematique_ROS_B.k_p = 0; Cinematique_ROS_B.k_p <=
           Cinematique_ROS_B.c_d; Cinematique_ROS_B.k_p++) {
        Cinematique_ROS_B.smax_i = b_A->data[Cinematique_ROS_B.jy];
        if (b_A->data[Cinematique_ROS_B.jy] != 0.0) {
          Cinematique_ROS_B.ix_n = Cinematique_ROS_B.c_tmp + 1;
          Cinematique_ROS_B.c_n = (Cinematique_ROS_B.mmj + Cinematique_ROS_B.jA)
            - 1;
          for (Cinematique_ROS_B.iy_c = Cinematique_ROS_B.jA + 1;
               Cinematique_ROS_B.iy_c <= Cinematique_ROS_B.c_n;
               Cinematique_ROS_B.iy_c++) {
            b_A->data[Cinematique_ROS_B.iy_c - 1] += b_A->
              data[Cinematique_ROS_B.ix_n] * -Cinematique_ROS_B.smax_i;
            Cinematique_ROS_B.ix_n++;
          }
        }

        Cinematique_ROS_B.jy += lda;
        Cinematique_ROS_B.jA += lda;
      }
    }

    if ((Cinematique_ROS_B.yk_p == 0) && (m <= n) && (!(b_A->data[((m - 1) *
           b_A->size[0] + m) - 1] != 0.0))) {
      Cinematique_ROS_B.yk_p = m;
    }
  }

  Cinematique_ROS_emxFree_real_T(&c_x);
  *info = Cinematique_ROS_B.yk_p;
}

static void Cinematique_ROS_xtrsm(int32_T m, int32_T n, const
  emxArray_real_T_Cinematique_R_T *A, int32_T lda, const
  emxArray_real_T_Cinematique_R_T *B, int32_T ldb,
  emxArray_real_T_Cinematique_R_T *b_B)
{
  int32_T b;
  int32_T b_i;
  int32_T i;
  int32_T jBcol;
  int32_T k;
  int32_T kAcol;
  int32_T loop_ub;
  int32_T tmp;
  i = b_B->size[0] * b_B->size[1];
  b_B->size[0] = B->size[0];
  b_B->size[1] = B->size[1];
  Cinema_emxEnsureCapacity_real_T(b_B, i);
  loop_ub = B->size[0] * B->size[1] - 1;
  for (i = 0; i <= loop_ub; i++) {
    b_B->data[i] = B->data[i];
  }

  if ((n == 0) || ((B->size[0] == 0) || (B->size[1] == 0))) {
  } else {
    for (loop_ub = 0; loop_ub < n; loop_ub++) {
      jBcol = ldb * loop_ub - 1;
      for (k = m; k >= 1; k--) {
        kAcol = (k - 1) * lda - 1;
        i = k + jBcol;
        if (b_B->data[i] != 0.0) {
          b_B->data[i] /= A->data[k + kAcol];
          b = k - 2;
          for (b_i = 0; b_i <= b; b_i++) {
            tmp = (b_i + jBcol) + 1;
            b_B->data[tmp] -= A->data[(b_i + kAcol) + 1] * b_B->data[i];
          }
        }
      }
    }
  }
}

static void Cinematique_ROS_mldivide(const emxArray_real_T_Cinematique_R_T *A,
  const emxArray_real_T_Cinematique_R_T *B, emxArray_real_T_Cinematique_R_T *Y)
{
  emxArray_int32_T_Cinematique__T *jpvt;
  emxArray_real_T_Cinematique_R_T *B_0;
  emxArray_real_T_Cinematique_R_T *b_A;
  emxArray_real_T_Cinematique_R_T *tau;
  Cinematique_ROS_emxInit_real_T(&b_A, 2);
  Cinematique_ROS_emxInit_real_T(&tau, 1);
  Cinematique_ROS_emxInit_int32_T(&jpvt, 2);
  Cinematique_ROS_emxInit_real_T(&B_0, 2);
  if ((A->size[0] == 0) || (A->size[1] == 0) || ((B->size[0] == 0) || (B->size[1]
        == 0))) {
    Cinematique_ROS_B.minmn = A->size[1];
    Cinematique_ROS_B.maxmn = B->size[1];
    Cinematique_ROS_B.b_i_k = Y->size[0] * Y->size[1];
    Y->size[0] = Cinematique_ROS_B.minmn;
    Y->size[1] = Cinematique_ROS_B.maxmn;
    Cinema_emxEnsureCapacity_real_T(Y, Cinematique_ROS_B.b_i_k);
    Cinematique_ROS_B.minmn = Cinematique_ROS_B.minmn * Cinematique_ROS_B.maxmn
      - 1;
    for (Cinematique_ROS_B.b_i_k = 0; Cinematique_ROS_B.b_i_k <=
         Cinematique_ROS_B.minmn; Cinematique_ROS_B.b_i_k++) {
      Y->data[Cinematique_ROS_B.b_i_k] = 0.0;
    }
  } else if (A->size[0] == A->size[1]) {
    Cinematique_ROS_B.minmn = A->size[0];
    Cinematique_ROS_B.rankA = A->size[1];
    if (Cinematique_ROS_B.minmn < Cinematique_ROS_B.rankA) {
      Cinematique_ROS_B.rankA = Cinematique_ROS_B.minmn;
    }

    Cinematique_ROS_B.minmn = B->size[0];
    if (Cinematique_ROS_B.minmn < Cinematique_ROS_B.rankA) {
      Cinematique_ROS_B.rankA = Cinematique_ROS_B.minmn;
    }

    Cinematique_ROS_B.nb = B->size[1] - 1;
    Cinematique_ROS_xzgetrf(Cinematique_ROS_B.rankA, Cinematique_ROS_B.rankA, A,
      A->size[0], b_A, jpvt, &Cinematique_ROS_B.minmn);
    Cinematique_ROS_B.b_i_k = B_0->size[0] * B_0->size[1];
    B_0->size[0] = B->size[0];
    B_0->size[1] = B->size[1];
    Cinema_emxEnsureCapacity_real_T(B_0, Cinematique_ROS_B.b_i_k);
    Cinematique_ROS_B.minmn = B->size[0] * B->size[1] - 1;
    for (Cinematique_ROS_B.b_i_k = 0; Cinematique_ROS_B.b_i_k <=
         Cinematique_ROS_B.minmn; Cinematique_ROS_B.b_i_k++) {
      B_0->data[Cinematique_ROS_B.b_i_k] = B->data[Cinematique_ROS_B.b_i_k];
    }

    Cinematique_ROS_B.minmn = Cinematique_ROS_B.rankA - 2;
    for (Cinematique_ROS_B.b_i_k = 0; Cinematique_ROS_B.b_i_k <=
         Cinematique_ROS_B.minmn; Cinematique_ROS_B.b_i_k++) {
      if (Cinematique_ROS_B.b_i_k + 1 != jpvt->data[Cinematique_ROS_B.b_i_k]) {
        Cinematique_ROS_B.ip = jpvt->data[Cinematique_ROS_B.b_i_k] - 1;
        for (Cinematique_ROS_B.maxmn = 0; Cinematique_ROS_B.maxmn <=
             Cinematique_ROS_B.nb; Cinematique_ROS_B.maxmn++) {
          Cinematique_ROS_B.tol_h = B_0->data[B_0->size[0] *
            Cinematique_ROS_B.maxmn + Cinematique_ROS_B.b_i_k];
          B_0->data[Cinematique_ROS_B.b_i_k + B_0->size[0] *
            Cinematique_ROS_B.maxmn] = B_0->data[B_0->size[0] *
            Cinematique_ROS_B.maxmn + Cinematique_ROS_B.ip];
          B_0->data[Cinematique_ROS_B.ip + B_0->size[0] *
            Cinematique_ROS_B.maxmn] = Cinematique_ROS_B.tol_h;
        }
      }
    }

    if ((B->size[1] == 0) || ((B_0->size[0] == 0) || (B_0->size[1] == 0))) {
    } else {
      for (Cinematique_ROS_B.maxmn = 0; Cinematique_ROS_B.maxmn <=
           Cinematique_ROS_B.nb; Cinematique_ROS_B.maxmn++) {
        Cinematique_ROS_B.m_i = B->size[0] * Cinematique_ROS_B.maxmn - 1;
        for (Cinematique_ROS_B.minmn = 0; Cinematique_ROS_B.minmn <
             Cinematique_ROS_B.rankA; Cinematique_ROS_B.minmn++) {
          Cinematique_ROS_B.nb_o = b_A->size[0] * Cinematique_ROS_B.minmn - 1;
          Cinematique_ROS_B.b_i_k = (Cinematique_ROS_B.minmn +
            Cinematique_ROS_B.m_i) + 1;
          if (B_0->data[Cinematique_ROS_B.b_i_k] != 0.0) {
            for (Cinematique_ROS_B.ip = Cinematique_ROS_B.minmn + 2;
                 Cinematique_ROS_B.ip <= Cinematique_ROS_B.rankA;
                 Cinematique_ROS_B.ip++) {
              Cinematique_ROS_B.mn = Cinematique_ROS_B.ip +
                Cinematique_ROS_B.m_i;
              B_0->data[Cinematique_ROS_B.mn] -= B_0->
                data[Cinematique_ROS_B.b_i_k] * b_A->data[Cinematique_ROS_B.ip +
                Cinematique_ROS_B.nb_o];
            }
          }
        }
      }
    }

    Cinematique_ROS_xtrsm(Cinematique_ROS_B.rankA, B->size[1], b_A, b_A->size[0],
                          B_0, B->size[0], Y);
  } else {
    Cinematique_ROS_xgeqp3(A, b_A, tau, jpvt);
    Cinematique_ROS_B.rankA = 0;
    if (b_A->size[0] < b_A->size[1]) {
      Cinematique_ROS_B.minmn = b_A->size[0];
      Cinematique_ROS_B.maxmn = b_A->size[1];
    } else {
      Cinematique_ROS_B.minmn = b_A->size[1];
      Cinematique_ROS_B.maxmn = b_A->size[0];
    }

    if (Cinematique_ROS_B.minmn > 0) {
      Cinematique_ROS_B.tol_h = 2.2204460492503131E-15 * static_cast<real_T>
        (Cinematique_ROS_B.maxmn);
      if (1.4901161193847656E-8 < Cinematique_ROS_B.tol_h) {
        Cinematique_ROS_B.tol_h = 1.4901161193847656E-8;
      }

      Cinematique_ROS_B.tol_h *= fabs(b_A->data[0]);
      while ((Cinematique_ROS_B.rankA < Cinematique_ROS_B.minmn) && (!(fabs
               (b_A->data[b_A->size[0] * Cinematique_ROS_B.rankA +
                Cinematique_ROS_B.rankA]) <= Cinematique_ROS_B.tol_h))) {
        Cinematique_ROS_B.rankA++;
      }
    }

    Cinematique_ROS_B.nb = B->size[1] - 1;
    Cinematique_ROS_B.minmn = b_A->size[1];
    Cinematique_ROS_B.maxmn = B->size[1];
    Cinematique_ROS_B.b_i_k = Y->size[0] * Y->size[1];
    Y->size[0] = Cinematique_ROS_B.minmn;
    Y->size[1] = Cinematique_ROS_B.maxmn;
    Cinema_emxEnsureCapacity_real_T(Y, Cinematique_ROS_B.b_i_k);
    Cinematique_ROS_B.minmn = Cinematique_ROS_B.minmn * Cinematique_ROS_B.maxmn
      - 1;
    for (Cinematique_ROS_B.b_i_k = 0; Cinematique_ROS_B.b_i_k <=
         Cinematique_ROS_B.minmn; Cinematique_ROS_B.b_i_k++) {
      Y->data[Cinematique_ROS_B.b_i_k] = 0.0;
    }

    Cinematique_ROS_B.b_i_k = B_0->size[0] * B_0->size[1];
    B_0->size[0] = B->size[0];
    B_0->size[1] = B->size[1];
    Cinema_emxEnsureCapacity_real_T(B_0, Cinematique_ROS_B.b_i_k);
    Cinematique_ROS_B.minmn = B->size[0] * B->size[1] - 1;
    for (Cinematique_ROS_B.b_i_k = 0; Cinematique_ROS_B.b_i_k <=
         Cinematique_ROS_B.minmn; Cinematique_ROS_B.b_i_k++) {
      B_0->data[Cinematique_ROS_B.b_i_k] = B->data[Cinematique_ROS_B.b_i_k];
    }

    Cinematique_ROS_B.m_i = b_A->size[0];
    Cinematique_ROS_B.nb_o = B->size[1] - 1;
    Cinematique_ROS_B.minmn = b_A->size[0];
    Cinematique_ROS_B.maxmn = b_A->size[1];
    if (Cinematique_ROS_B.minmn < Cinematique_ROS_B.maxmn) {
      Cinematique_ROS_B.maxmn = Cinematique_ROS_B.minmn;
    }

    Cinematique_ROS_B.mn = Cinematique_ROS_B.maxmn - 1;
    for (Cinematique_ROS_B.maxmn = 0; Cinematique_ROS_B.maxmn <=
         Cinematique_ROS_B.mn; Cinematique_ROS_B.maxmn++) {
      if (tau->data[Cinematique_ROS_B.maxmn] != 0.0) {
        for (Cinematique_ROS_B.minmn = 0; Cinematique_ROS_B.minmn <=
             Cinematique_ROS_B.nb_o; Cinematique_ROS_B.minmn++) {
          Cinematique_ROS_B.tol_h = B_0->data[B_0->size[0] *
            Cinematique_ROS_B.minmn + Cinematique_ROS_B.maxmn];
          for (Cinematique_ROS_B.ip = Cinematique_ROS_B.maxmn + 2;
               Cinematique_ROS_B.ip <= Cinematique_ROS_B.m_i;
               Cinematique_ROS_B.ip++) {
            Cinematique_ROS_B.tol_h += b_A->data[(b_A->size[0] *
              Cinematique_ROS_B.maxmn + Cinematique_ROS_B.ip) - 1] * B_0->data
              [(B_0->size[0] * Cinematique_ROS_B.minmn + Cinematique_ROS_B.ip) -
              1];
          }

          Cinematique_ROS_B.tol_h *= tau->data[Cinematique_ROS_B.maxmn];
          if (Cinematique_ROS_B.tol_h != 0.0) {
            B_0->data[Cinematique_ROS_B.maxmn + B_0->size[0] *
              Cinematique_ROS_B.minmn] -= Cinematique_ROS_B.tol_h;
            for (Cinematique_ROS_B.b_i_k = Cinematique_ROS_B.maxmn + 2;
                 Cinematique_ROS_B.b_i_k <= Cinematique_ROS_B.m_i;
                 Cinematique_ROS_B.b_i_k++) {
              B_0->data[(Cinematique_ROS_B.b_i_k + B_0->size[0] *
                         Cinematique_ROS_B.minmn) - 1] -= b_A->data[(b_A->size[0]
                * Cinematique_ROS_B.maxmn + Cinematique_ROS_B.b_i_k) - 1] *
                Cinematique_ROS_B.tol_h;
            }
          }
        }
      }
    }

    for (Cinematique_ROS_B.minmn = 0; Cinematique_ROS_B.minmn <=
         Cinematique_ROS_B.nb; Cinematique_ROS_B.minmn++) {
      for (Cinematique_ROS_B.b_i_k = 0; Cinematique_ROS_B.b_i_k <
           Cinematique_ROS_B.rankA; Cinematique_ROS_B.b_i_k++) {
        Y->data[(jpvt->data[Cinematique_ROS_B.b_i_k] + Y->size[0] *
                 Cinematique_ROS_B.minmn) - 1] = B_0->data[B_0->size[0] *
          Cinematique_ROS_B.minmn + Cinematique_ROS_B.b_i_k];
      }

      for (Cinematique_ROS_B.maxmn = Cinematique_ROS_B.rankA;
           Cinematique_ROS_B.maxmn >= 1; Cinematique_ROS_B.maxmn--) {
        Y->data[(jpvt->data[Cinematique_ROS_B.maxmn - 1] + Y->size[0] *
                 Cinematique_ROS_B.minmn) - 1] /= b_A->data
          [((Cinematique_ROS_B.maxmn - 1) * b_A->size[0] +
            Cinematique_ROS_B.maxmn) - 1];
        Cinematique_ROS_B.ip = Cinematique_ROS_B.maxmn - 2;
        for (Cinematique_ROS_B.b_i_k = 0; Cinematique_ROS_B.b_i_k <=
             Cinematique_ROS_B.ip; Cinematique_ROS_B.b_i_k++) {
          Y->data[(jpvt->data[Cinematique_ROS_B.b_i_k] + Y->size[0] *
                   Cinematique_ROS_B.minmn) - 1] -= Y->data[(jpvt->
            data[Cinematique_ROS_B.maxmn - 1] + Y->size[0] *
            Cinematique_ROS_B.minmn) - 1] * b_A->data[(Cinematique_ROS_B.maxmn -
            1) * b_A->size[0] + Cinematique_ROS_B.b_i_k];
        }
      }
    }
  }

  Cinematique_ROS_emxFree_real_T(&B_0);
  Cinematique_ROS_emxFree_int32_T(&jpvt);
  Cinematique_ROS_emxFree_real_T(&tau);
  Cinematique_ROS_emxFree_real_T(&b_A);
}

static real_T Cinematique_ROS_norm_k(const real_T x[4])
{
  real_T absxk;
  real_T scale;
  real_T t;
  real_T y;
  scale = 3.3121686421112381E-170;
  absxk = fabs(x[0]);
  if (absxk > 3.3121686421112381E-170) {
    y = 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    y = t * t;
  }

  absxk = fabs(x[1]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  absxk = fabs(x[2]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  absxk = fabs(x[3]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  return scale * sqrt(y);
}

static void Cinematique_R_emxFree_boolean_T(emxArray_boolean_T_Cinematiqu_T
  **pEmxArray)
{
  if (*pEmxArray != (emxArray_boolean_T_Cinematiqu_T *)NULL) {
    if (((*pEmxArray)->data != (boolean_T *)NULL) && (*pEmxArray)->canFreeData)
    {
      free((*pEmxArray)->data);
    }

    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = (emxArray_boolean_T_Cinematiqu_T *)NULL;
  }
}

static boolean_T DampedBFGSwGradientProjection_a(const
  h_robotics_core_internal_Damp_T *obj, const real_T Hg[4], const
  emxArray_real_T_Cinematique_R_T *alpha)
{
  emxArray_boolean_T_Cinematiqu_T *x;
  int32_T ix;
  int32_T loop_ub;
  boolean_T exitg1;
  boolean_T flag;
  boolean_T y;
  Cinematique_R_emxInit_boolean_T(&x, 1);
  if (Cinematique_ROS_norm_k(Hg) < obj->GradientTolerance) {
    ix = x->size[0];
    x->size[0] = alpha->size[0];
    Cin_emxEnsureCapacity_boolean_T(x, ix);
    loop_ub = alpha->size[0];
    for (ix = 0; ix < loop_ub; ix++) {
      x->data[ix] = (alpha->data[ix] <= 0.0);
    }

    y = true;
    ix = 0;
    exitg1 = false;
    while ((!exitg1) && (ix + 1 <= x->size[0])) {
      if (!x->data[ix]) {
        y = false;
        exitg1 = true;
      } else {
        ix++;
      }
    }

    if (y) {
      flag = true;
    } else {
      flag = false;
    }
  } else {
    flag = false;
  }

  Cinematique_R_emxFree_boolean_T(&x);
  return flag;
}

static void Cinematique_ROS_inv(const emxArray_real_T_Cinematique_R_T *x,
  emxArray_real_T_Cinematique_R_T *y)
{
  emxArray_int32_T_Cinematique__T *b_ipiv;
  emxArray_int32_T_Cinematique__T *p;
  emxArray_real_T_Cinematique_R_T *c_A;
  emxArray_real_T_Cinematique_R_T *y_0;
  if ((x->size[0] == 0) || (x->size[1] == 0)) {
    Cinematique_ROS_B.info = y->size[0] * y->size[1];
    y->size[0] = x->size[0];
    y->size[1] = x->size[1];
    Cinema_emxEnsureCapacity_real_T(y, Cinematique_ROS_B.info);
    Cinematique_ROS_B.n_a = x->size[0] * x->size[1] - 1;
    for (Cinematique_ROS_B.info = 0; Cinematique_ROS_B.info <=
         Cinematique_ROS_B.n_a; Cinematique_ROS_B.info++) {
      y->data[Cinematique_ROS_B.info] = x->data[Cinematique_ROS_B.info];
    }
  } else {
    Cinematique_ROS_B.n_j = x->size[0];
    Cinematique_ROS_B.info = y->size[0] * y->size[1];
    y->size[0] = x->size[0];
    y->size[1] = x->size[1];
    Cinema_emxEnsureCapacity_real_T(y, Cinematique_ROS_B.info);
    Cinematique_ROS_B.n_a = x->size[0] * x->size[1] - 1;
    for (Cinematique_ROS_B.info = 0; Cinematique_ROS_B.info <=
         Cinematique_ROS_B.n_a; Cinematique_ROS_B.info++) {
      y->data[Cinematique_ROS_B.info] = 0.0;
    }

    Cinematique_ROS_emxInit_int32_T(&p, 2);
    Cinematique_ROS_emxInit_real_T(&c_A, 2);
    Cinematique_ROS_emxInit_int32_T(&b_ipiv, 2);
    Cinematique_ROS_xzgetrf(x->size[0], x->size[0], x, x->size[0], c_A, b_ipiv,
      &Cinematique_ROS_B.info);
    if (x->size[0] < 1) {
      Cinematique_ROS_B.n_a = 0;
    } else {
      Cinematique_ROS_B.n_a = x->size[0];
    }

    Cinematique_ROS_B.info = p->size[0] * p->size[1];
    p->size[0] = 1;
    p->size[1] = Cinematique_ROS_B.n_a;
    Cinem_emxEnsureCapacity_int32_T(p, Cinematique_ROS_B.info);
    if (Cinematique_ROS_B.n_a > 0) {
      p->data[0] = 1;
      Cinematique_ROS_B.yk = 1;
      for (Cinematique_ROS_B.info = 2; Cinematique_ROS_B.info <=
           Cinematique_ROS_B.n_a; Cinematique_ROS_B.info++) {
        Cinematique_ROS_B.yk++;
        p->data[Cinematique_ROS_B.info - 1] = Cinematique_ROS_B.yk;
      }
    }

    Cinematique_ROS_B.n_a = b_ipiv->size[1] - 1;
    for (Cinematique_ROS_B.info = 0; Cinematique_ROS_B.info <=
         Cinematique_ROS_B.n_a; Cinematique_ROS_B.info++) {
      if (b_ipiv->data[Cinematique_ROS_B.info] > static_cast<real_T>
          (Cinematique_ROS_B.info) + 1.0) {
        Cinematique_ROS_B.yk = p->data[b_ipiv->data[Cinematique_ROS_B.info] - 1];
        p->data[b_ipiv->data[Cinematique_ROS_B.info] - 1] = p->
          data[Cinematique_ROS_B.info];
        p->data[Cinematique_ROS_B.info] = Cinematique_ROS_B.yk;
      }
    }

    Cinematique_ROS_emxFree_int32_T(&b_ipiv);
    for (Cinematique_ROS_B.info = 0; Cinematique_ROS_B.info <
         Cinematique_ROS_B.n_j; Cinematique_ROS_B.info++) {
      Cinematique_ROS_B.c_fo = p->data[Cinematique_ROS_B.info] - 1;
      y->data[Cinematique_ROS_B.info + y->size[0] * (p->
        data[Cinematique_ROS_B.info] - 1)] = 1.0;
      for (Cinematique_ROS_B.n_a = Cinematique_ROS_B.info + 1;
           Cinematique_ROS_B.n_a <= Cinematique_ROS_B.n_j; Cinematique_ROS_B.n_a
           ++) {
        if (y->data[(y->size[0] * Cinematique_ROS_B.c_fo + Cinematique_ROS_B.n_a)
            - 1] != 0.0) {
          for (Cinematique_ROS_B.yk = Cinematique_ROS_B.n_a + 1;
               Cinematique_ROS_B.yk <= Cinematique_ROS_B.n_j;
               Cinematique_ROS_B.yk++) {
            y->data[(Cinematique_ROS_B.yk + y->size[0] * Cinematique_ROS_B.c_fo)
              - 1] -= c_A->data[((Cinematique_ROS_B.n_a - 1) * c_A->size[0] +
                                 Cinematique_ROS_B.yk) - 1] * y->data[(y->size[0]
              * Cinematique_ROS_B.c_fo + Cinematique_ROS_B.n_a) - 1];
          }
        }
      }
    }

    Cinematique_ROS_emxFree_int32_T(&p);
    Cinematique_ROS_emxInit_real_T(&y_0, 2);
    Cinematique_ROS_B.info = y_0->size[0] * y_0->size[1];
    y_0->size[0] = y->size[0];
    y_0->size[1] = y->size[1];
    Cinema_emxEnsureCapacity_real_T(y_0, Cinematique_ROS_B.info);
    Cinematique_ROS_B.n_a = y->size[0] * y->size[1];
    for (Cinematique_ROS_B.info = 0; Cinematique_ROS_B.info <
         Cinematique_ROS_B.n_a; Cinematique_ROS_B.info++) {
      y_0->data[Cinematique_ROS_B.info] = y->data[Cinematique_ROS_B.info];
    }

    Cinematique_ROS_xtrsm(x->size[0], x->size[0], c_A, x->size[0], y_0, x->size
                          [0], y);
    Cinematique_ROS_emxFree_real_T(&y_0);
    Cinematique_ROS_emxFree_real_T(&c_A);
  }
}

static void Cinematique_ROS_diag(const emxArray_real_T_Cinematique_R_T *v,
  emxArray_real_T_Cinematique_R_T *d)
{
  int32_T u0;
  int32_T u1;
  if ((v->size[0] == 1) && (v->size[1] == 1)) {
    u0 = d->size[0];
    d->size[0] = 1;
    Cinema_emxEnsureCapacity_real_T(d, u0);
    d->data[0] = v->data[0];
  } else {
    if (0 < v->size[1]) {
      u0 = v->size[0];
      u1 = v->size[1];
      if (u0 < u1) {
        u1 = u0;
      }
    } else {
      u1 = 0;
    }

    u0 = d->size[0];
    d->size[0] = u1;
    Cinema_emxEnsureCapacity_real_T(d, u0);
    for (u0 = 0; u0 < u1; u0++) {
      d->data[u0] = v->data[v->size[0] * u0 + u0];
    }
  }
}

static void Cinematique_ROS_maximum(const emxArray_real_T_Cinematique_R_T *x,
  real_T *ex, int32_T *idx)
{
  int32_T b_idx;
  int32_T k;
  int32_T n;
  boolean_T exitg1;
  n = x->size[0];
  if (x->size[0] <= 2) {
    if (x->size[0] == 1) {
      *ex = x->data[0];
      *idx = 1;
    } else if ((x->data[0] < x->data[1]) || (rtIsNaN(x->data[0]) && (!rtIsNaN
                 (x->data[1])))) {
      *ex = x->data[1];
      *idx = 2;
    } else {
      *ex = x->data[0];
      *idx = 1;
    }
  } else {
    if (!rtIsNaN(x->data[0])) {
      b_idx = 1;
    } else {
      b_idx = 0;
      k = 2;
      exitg1 = false;
      while ((!exitg1) && (k <= x->size[0])) {
        if (!rtIsNaN(x->data[k - 1])) {
          b_idx = k;
          exitg1 = true;
        } else {
          k++;
        }
      }
    }

    if (b_idx == 0) {
      *ex = x->data[0];
      *idx = 1;
    } else {
      *ex = x->data[b_idx - 1];
      *idx = b_idx;
      for (k = b_idx + 1; k <= n; k++) {
        if (*ex < x->data[k - 1]) {
          *ex = x->data[k - 1];
          *idx = k;
        }
      }
    }
  }
}

static boolean_T Cinematique_ROS_any(const emxArray_boolean_T_Cinematiqu_T *x)
{
  int32_T ix;
  boolean_T exitg1;
  boolean_T y;
  y = false;
  ix = 0;
  exitg1 = false;
  while ((!exitg1) && (ix + 1 <= x->size[0])) {
    if (!x->data[ix]) {
      ix++;
    } else {
      y = true;
      exitg1 = true;
    }
  }

  return y;
}

static void Cinematique_ROS_eml_find_k(const emxArray_boolean_T_Cinematiqu_T *x,
  emxArray_int32_T_Cinematique__T *i)
{
  emxArray_int32_T_Cinematique__T *i_0;
  int32_T b_ii;
  int32_T idx;
  int32_T nx;
  boolean_T exitg1;
  nx = x->size[0];
  idx = 0;
  b_ii = i->size[0];
  i->size[0] = x->size[0];
  Cinem_emxEnsureCapacity_int32_T(i, b_ii);
  b_ii = 1;
  exitg1 = false;
  while ((!exitg1) && (b_ii - 1 <= nx - 1)) {
    if (x->data[b_ii - 1]) {
      idx++;
      i->data[idx - 1] = b_ii;
      if (idx >= nx) {
        exitg1 = true;
      } else {
        b_ii++;
      }
    } else {
      b_ii++;
    }
  }

  if (x->size[0] == 1) {
    if (idx == 0) {
      i->size[0] = 0;
    }
  } else {
    Cinematique_ROS_emxInit_int32_T(&i_0, 1);
    if (1 > idx) {
      idx = 0;
    }

    b_ii = i_0->size[0];
    i_0->size[0] = idx;
    Cinem_emxEnsureCapacity_int32_T(i_0, b_ii);
    for (b_ii = 0; b_ii < idx; b_ii++) {
      i_0->data[b_ii] = i->data[b_ii];
    }

    b_ii = i->size[0];
    i->size[0] = i_0->size[0];
    Cinem_emxEnsureCapacity_int32_T(i, b_ii);
    nx = i_0->size[0];
    for (b_ii = 0; b_ii < nx; b_ii++) {
      i->data[b_ii] = i_0->data[b_ii];
    }

    Cinematique_ROS_emxFree_int32_T(&i_0);
  }
}

static void Cinematique_ROS_minimum(const emxArray_real_T_Cinematique_R_T *x,
  real_T *ex, int32_T *idx)
{
  int32_T b_idx;
  int32_T k;
  int32_T n;
  boolean_T exitg1;
  n = x->size[0];
  if (x->size[0] <= 2) {
    if (x->size[0] == 1) {
      *ex = x->data[0];
      *idx = 1;
    } else if ((x->data[0] > x->data[1]) || (rtIsNaN(x->data[0]) && (!rtIsNaN
                 (x->data[1])))) {
      *ex = x->data[1];
      *idx = 2;
    } else {
      *ex = x->data[0];
      *idx = 1;
    }
  } else {
    if (!rtIsNaN(x->data[0])) {
      b_idx = 1;
    } else {
      b_idx = 0;
      k = 2;
      exitg1 = false;
      while ((!exitg1) && (k <= x->size[0])) {
        if (!rtIsNaN(x->data[k - 1])) {
          b_idx = k;
          exitg1 = true;
        } else {
          k++;
        }
      }
    }

    if (b_idx == 0) {
      *ex = x->data[0];
      *idx = 1;
    } else {
      *ex = x->data[b_idx - 1];
      *idx = b_idx;
      for (k = b_idx + 1; k <= n; k++) {
        if (*ex > x->data[k - 1]) {
          *ex = x->data[k - 1];
          *idx = k;
        }
      }
    }
  }
}

static boolean_T Cinematique__isPositiveDefinite(const real_T B[16])
{
  emxArray_real_T_Cinematique_R_T *b_x;
  boolean_T exitg1;
  Cinematique_ROS_B.c_A_size_idx_0 = 4;
  Cinematique_ROS_B.c_A_size_idx_1 = 4;
  memcpy(&Cinematique_ROS_B.c_A_data[0], &B[0], sizeof(real_T) << 4U);
  Cinematique_ROS_B.b_info = 0;
  Cinematique_ROS_B.b_j_ad = 1;
  Cinematique_ROS_emxInit_real_T(&b_x, 2);
  exitg1 = false;
  while ((!exitg1) && (Cinematique_ROS_B.b_j_ad - 1 < 4)) {
    Cinematique_ROS_B.jm1 = Cinematique_ROS_B.b_j_ad - 2;
    Cinematique_ROS_B.idxAjj = (((Cinematique_ROS_B.b_j_ad - 1) << 2) +
      Cinematique_ROS_B.b_j_ad) - 1;
    Cinematique_ROS_B.ssq = 0.0;
    if (Cinematique_ROS_B.b_j_ad - 1 >= 1) {
      Cinematique_ROS_B.ix_p = Cinematique_ROS_B.b_j_ad - 1;
      Cinematique_ROS_B.iy_b = Cinematique_ROS_B.b_j_ad - 1;
      for (Cinematique_ROS_B.k_c = 0; Cinematique_ROS_B.k_c <=
           Cinematique_ROS_B.jm1; Cinematique_ROS_B.k_c++) {
        Cinematique_ROS_B.ssq +=
          Cinematique_ROS_B.c_A_data[Cinematique_ROS_B.ix_p] *
          Cinematique_ROS_B.c_A_data[Cinematique_ROS_B.iy_b];
        Cinematique_ROS_B.ix_p += 4;
        Cinematique_ROS_B.iy_b += 4;
      }
    }

    Cinematique_ROS_B.ssq = Cinematique_ROS_B.c_A_data[Cinematique_ROS_B.idxAjj]
      - Cinematique_ROS_B.ssq;
    if (Cinematique_ROS_B.ssq > 0.0) {
      Cinematique_ROS_B.ssq = sqrt(Cinematique_ROS_B.ssq);
      Cinematique_ROS_B.c_A_data[Cinematique_ROS_B.idxAjj] =
        Cinematique_ROS_B.ssq;
      if (Cinematique_ROS_B.b_j_ad < 4) {
        if (Cinematique_ROS_B.b_j_ad - 1 != 0) {
          Cinematique_ROS_B.ix_p = Cinematique_ROS_B.b_j_ad - 1;
          Cinematique_ROS_B.jm1 = ((Cinematique_ROS_B.b_j_ad - 2) << 2) +
            Cinematique_ROS_B.b_j_ad;
          for (Cinematique_ROS_B.k_c = Cinematique_ROS_B.b_j_ad + 1;
               Cinematique_ROS_B.k_c <= Cinematique_ROS_B.jm1 + 1;
               Cinematique_ROS_B.k_c += 4) {
            Cinematique_ROS_B.c_a =
              -Cinematique_ROS_B.c_A_data[Cinematique_ROS_B.ix_p];
            Cinematique_ROS_B.iy_b = Cinematique_ROS_B.idxAjj + 1;
            Cinematique_ROS_B.d_kb = Cinematique_ROS_B.k_c -
              Cinematique_ROS_B.b_j_ad;
            for (Cinematique_ROS_B.ia = Cinematique_ROS_B.k_c;
                 Cinematique_ROS_B.ia <= Cinematique_ROS_B.d_kb + 3;
                 Cinematique_ROS_B.ia++) {
              Cinematique_ROS_B.c_A_data[Cinematique_ROS_B.iy_b] +=
                Cinematique_ROS_B.c_A_data[Cinematique_ROS_B.ia - 1] *
                Cinematique_ROS_B.c_a;
              Cinematique_ROS_B.iy_b++;
            }

            Cinematique_ROS_B.ix_p += 4;
          }
        }

        Cinematique_ROS_B.ssq = 1.0 / Cinematique_ROS_B.ssq;
        Cinematique_ROS_B.ix_p = b_x->size[0] * b_x->size[1];
        b_x->size[0] = 4;
        b_x->size[1] = 4;
        Cinema_emxEnsureCapacity_real_T(b_x, Cinematique_ROS_B.ix_p);
        Cinematique_ROS_B.c_A_size_idx_0 = Cinematique_ROS_B.c_A_size_idx_0 *
          Cinematique_ROS_B.c_A_size_idx_1 - 1;
        for (Cinematique_ROS_B.ix_p = 0; Cinematique_ROS_B.ix_p <=
             Cinematique_ROS_B.c_A_size_idx_0; Cinematique_ROS_B.ix_p++) {
          b_x->data[Cinematique_ROS_B.ix_p] =
            Cinematique_ROS_B.c_A_data[Cinematique_ROS_B.ix_p];
        }

        Cinematique_ROS_B.jm1 = Cinematique_ROS_B.idxAjj -
          Cinematique_ROS_B.b_j_ad;
        for (Cinematique_ROS_B.k_c = Cinematique_ROS_B.idxAjj + 2;
             Cinematique_ROS_B.k_c <= Cinematique_ROS_B.jm1 + 5;
             Cinematique_ROS_B.k_c++) {
          b_x->data[Cinematique_ROS_B.k_c - 1] *= Cinematique_ROS_B.ssq;
        }

        Cinematique_ROS_B.c_A_size_idx_0 = b_x->size[0];
        Cinematique_ROS_B.c_A_size_idx_1 = b_x->size[1];
        for (Cinematique_ROS_B.ix_p = 0; Cinematique_ROS_B.ix_p < 16;
             Cinematique_ROS_B.ix_p++) {
          Cinematique_ROS_B.c_A_data[Cinematique_ROS_B.ix_p] = b_x->
            data[Cinematique_ROS_B.ix_p];
        }
      }

      Cinematique_ROS_B.b_j_ad++;
    } else {
      Cinematique_ROS_B.b_info = Cinematique_ROS_B.b_j_ad;
      exitg1 = true;
    }
  }

  Cinematique_ROS_emxFree_real_T(&b_x);
  return Cinematique_ROS_B.b_info == 0;
}

static void Cinematique_ROS_mtimes_k1e1w(const emxArray_real_T_Cinematique_R_T
  *A, const real_T B[16], emxArray_real_T_Cinematique_R_T *C)
{
  real_T bkj;
  int32_T aoffset;
  int32_T b_i;
  int32_T c_i;
  int32_T coffset;
  int32_T inner;
  int32_T m;
  int32_T tmp;
  m = A->size[0] - 1;
  inner = A->size[1] - 1;
  tmp = C->size[0] * C->size[1];
  C->size[0] = A->size[0];
  C->size[1] = 4;
  Cinema_emxEnsureCapacity_real_T(C, tmp);
  for (b_i = 0; b_i <= m; b_i++) {
    C->data[b_i] = 0.0;
  }

  for (b_i = 0; b_i <= inner; b_i++) {
    aoffset = b_i * A->size[0] - 1;
    bkj = B[b_i];
    for (c_i = 0; c_i <= m; c_i++) {
      C->data[c_i] += A->data[(c_i + aoffset) + 1] * bkj;
    }
  }

  for (b_i = 0; b_i <= m; b_i++) {
    C->data[(m + b_i) + 1] = 0.0;
  }

  for (b_i = 0; b_i <= inner; b_i++) {
    aoffset = b_i * A->size[0] - 1;
    bkj = B[b_i + 4];
    for (c_i = 0; c_i <= m; c_i++) {
      tmp = (c_i + m) + 1;
      C->data[tmp] += A->data[(c_i + aoffset) + 1] * bkj;
    }
  }

  coffset = ((m + 1) << 1) - 1;
  for (b_i = 0; b_i <= m; b_i++) {
    C->data[(coffset + b_i) + 1] = 0.0;
  }

  for (b_i = 0; b_i <= inner; b_i++) {
    aoffset = b_i * A->size[0] - 1;
    bkj = B[b_i + 8];
    for (c_i = 0; c_i <= m; c_i++) {
      tmp = (c_i + coffset) + 1;
      C->data[tmp] += A->data[(c_i + aoffset) + 1] * bkj;
    }
  }

  coffset = (m + 1) * 3 - 1;
  for (b_i = 0; b_i <= m; b_i++) {
    C->data[(coffset + b_i) + 1] = 0.0;
  }

  for (b_i = 0; b_i <= inner; b_i++) {
    aoffset = b_i * A->size[0] - 1;
    bkj = B[b_i + 12];
    for (c_i = 0; c_i <= m; c_i++) {
      tmp = (c_i + coffset) + 1;
      C->data[tmp] += A->data[(c_i + aoffset) + 1] * bkj;
    }
  }
}

static void Cinematique_ROS_mtimes_k1e1(const emxArray_real_T_Cinematique_R_T *A,
  const emxArray_real_T_Cinematique_R_T *B, emxArray_real_T_Cinematique_R_T *C)
{
  real_T bkj;
  int32_T aoffset;
  int32_T b_i;
  int32_T b_j;
  int32_T boffset;
  int32_T c_i;
  int32_T coffset;
  int32_T inner;
  int32_T m;
  int32_T n;
  int32_T tmp;
  m = A->size[0] - 1;
  inner = A->size[1] - 1;
  n = B->size[1] - 1;
  tmp = C->size[0] * C->size[1];
  C->size[0] = A->size[0];
  C->size[1] = B->size[1];
  Cinema_emxEnsureCapacity_real_T(C, tmp);
  for (b_j = 0; b_j <= n; b_j++) {
    coffset = (m + 1) * b_j - 1;
    boffset = b_j * B->size[0] - 1;
    for (b_i = 0; b_i <= m; b_i++) {
      C->data[(coffset + b_i) + 1] = 0.0;
    }

    for (b_i = 0; b_i <= inner; b_i++) {
      aoffset = b_i * A->size[0] - 1;
      bkj = B->data[(boffset + b_i) + 1];
      for (c_i = 0; c_i <= m; c_i++) {
        tmp = (c_i + coffset) + 1;
        C->data[tmp] += A->data[(c_i + aoffset) + 1] * bkj;
      }
    }
  }
}

static void DampedBFGSwGradientProjection_s(h_robotics_core_internal_Damp_T *obj,
  real_T xSol[4], c_robotics_core_internal_NLPS_T *exitFlag, real_T *err, real_T
  *iter)
{
  static const int8_T tmp_6[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    1 };

  emxArray_boolean_T_Cinematiqu_T *L_0;
  emxArray_boolean_T_Cinematiqu_T *activeSet;
  emxArray_boolean_T_Cinematiqu_T *alpha_1;
  emxArray_boolean_T_Cinematiqu_T *tmp;
  emxArray_int32_T_Cinematique__T *bb;
  emxArray_int32_T_Cinematique__T *cb;
  emxArray_int32_T_Cinematique__T *db;
  emxArray_int32_T_Cinematique__T *eb;
  emxArray_int32_T_Cinematique__T *fb;
  emxArray_int32_T_Cinematique__T *gb;
  emxArray_real_T_Cinematique_R_T *A;
  emxArray_real_T_Cinematique_R_T *AIn;
  emxArray_real_T_Cinematique_R_T *A_0;
  emxArray_real_T_Cinematique_R_T *A_1;
  emxArray_real_T_Cinematique_R_T *A_2;
  emxArray_real_T_Cinematique_R_T *A_3;
  emxArray_real_T_Cinematique_R_T *A_4;
  emxArray_real_T_Cinematique_R_T *L;
  emxArray_real_T_Cinematique_R_T *alpha;
  emxArray_real_T_Cinematique_R_T *alpha_0;
  emxArray_real_T_Cinematique_R_T *alpha_2;
  emxArray_real_T_Cinematique_R_T *alpha_3;
  emxArray_real_T_Cinematique_R_T *alpha_4;
  emxArray_real_T_Cinematique_R_T *grad;
  emxArray_real_T_Cinematique_R_T *grad_0;
  emxArray_real_T_Cinematique_R_T *grad_1;
  emxArray_real_T_Cinematique_R_T *grad_2;
  emxArray_real_T_Cinematique_R_T *sNew;
  emxArray_real_T_Cinematique_R_T *sigma;
  emxArray_real_T_Cinematique_R_T *tmp_0;
  emxArray_real_T_Cinematique_R_T *tmp_1;
  emxArray_real_T_Cinematique_R_T *tmp_2;
  emxArray_real_T_Cinematique_R_T *tmp_3;
  emxArray_real_T_Cinematique_R_T *tmp_4;
  emxArray_real_T_Cinematique_R_T *tmp_5;
  emxArray_real_T_Cinematique_R_T *unusedU1;
  f_robotics_manip_internal_IKE_T *args;
  f_robotics_manip_internal_IKE_T *b;
  f_robotics_manip_internal_IKE_T *c;
  f_robotics_manip_internal_IKE_T *d;
  int32_T exitg1;
  int32_T exitg2;
  boolean_T guard1 = false;
  boolean_T guard2 = false;
  boolean_T guard3 = false;
  Cinematique_ROS_B.x[0] = obj->SeedInternal[0];
  Cinematique_ROS_B.x[1] = obj->SeedInternal[1];
  Cinematique_ROS_B.x[2] = obj->SeedInternal[2];
  Cinematique_ROS_B.x[3] = obj->SeedInternal[3];
  Cinematique_ROS_emxInit_real_T(&unusedU1, 2);
  Cinematique_ROS_emxInit_real_T(&grad, 1);
  Cinematique_ROS_tic(&obj->TimeObjInternal.StartTime.tv_sec,
                      &obj->TimeObjInternal.StartTime.tv_nsec);
  Cinematiq_IKHelpers_computeCost(Cinematique_ROS_B.x, obj->ExtraArgs,
    &Cinematique_ROS_B.cost, Cinematique_ROS_B.unusedU0, unusedU1, &b);
  obj->ExtraArgs = b;
  args = obj->ExtraArgs;
  Cinematique_ROS_B.nx_h = grad->size[0];
  grad->size[0] = args->GradTemp->size[0];
  Cinema_emxEnsureCapacity_real_T(grad, Cinematique_ROS_B.nx_h);
  Cinematique_ROS_B.inner = args->GradTemp->size[0];
  for (Cinematique_ROS_B.nx_h = 0; Cinematique_ROS_B.nx_h <
       Cinematique_ROS_B.inner; Cinematique_ROS_B.nx_h++) {
    grad->data[Cinematique_ROS_B.nx_h] = args->GradTemp->
      data[Cinematique_ROS_B.nx_h];
  }

  memset(&Cinematique_ROS_B.H[0], 0, sizeof(real_T) << 4U);
  Cinematique_ROS_B.H[0] = 1.0;
  Cinematique_ROS_B.H[5] = 1.0;
  Cinematique_ROS_B.H[10] = 1.0;
  Cinematique_ROS_B.H[15] = 1.0;
  Cinematique_R_emxInit_boolean_T(&activeSet, 1);
  Cinematique_ROS_emxInit_real_T(&A, 2);
  Cinematique_ROS_emxInit_real_T(&L, 1);
  Cinematique_ROS_emxInit_int32_T(&bb, 1);
  if (obj->ConstraintsOn) {
    Cinematique_ROS_B.nx_h = A->size[0] * A->size[1];
    A->size[0] = obj->ConstraintMatrix->size[0];
    A->size[1] = obj->ConstraintMatrix->size[1];
    Cinema_emxEnsureCapacity_real_T(A, Cinematique_ROS_B.nx_h);
    Cinematique_ROS_B.inner = obj->ConstraintMatrix->size[0] *
      obj->ConstraintMatrix->size[1] - 1;
    for (Cinematique_ROS_B.nx_h = 0; Cinematique_ROS_B.nx_h <=
         Cinematique_ROS_B.inner; Cinematique_ROS_B.nx_h++) {
      A->data[Cinematique_ROS_B.nx_h] = obj->ConstraintMatrix->
        data[Cinematique_ROS_B.nx_h];
    }

    Cinematique_ROS_mtimes_k(A, Cinematique_ROS_B.x, L);
    Cinematique_ROS_B.nx_h = activeSet->size[0];
    activeSet->size[0] = L->size[0];
    Cin_emxEnsureCapacity_boolean_T(activeSet, Cinematique_ROS_B.nx_h);
    Cinematique_ROS_B.inner = L->size[0];
    for (Cinematique_ROS_B.nx_h = 0; Cinematique_ROS_B.nx_h <
         Cinematique_ROS_B.inner; Cinematique_ROS_B.nx_h++) {
      activeSet->data[Cinematique_ROS_B.nx_h] = (L->data[Cinematique_ROS_B.nx_h]
        >= obj->ConstraintBound->data[Cinematique_ROS_B.nx_h]);
    }

    Cinematique_ROS_B.idxl = activeSet->size[0] - 1;
    Cinematique_ROS_B.m_me = 0;
    for (Cinematique_ROS_B.nx_h = 0; Cinematique_ROS_B.nx_h <=
         Cinematique_ROS_B.idxl; Cinematique_ROS_B.nx_h++) {
      if (activeSet->data[Cinematique_ROS_B.nx_h]) {
        Cinematique_ROS_B.m_me++;
      }
    }

    Cinematique_ROS_B.nx_h = bb->size[0];
    bb->size[0] = Cinematique_ROS_B.m_me;
    Cinem_emxEnsureCapacity_int32_T(bb, Cinematique_ROS_B.nx_h);
    Cinematique_ROS_B.m_me = 0;
    for (Cinematique_ROS_B.nx_h = 0; Cinematique_ROS_B.nx_h <=
         Cinematique_ROS_B.idxl; Cinematique_ROS_B.nx_h++) {
      if (activeSet->data[Cinematique_ROS_B.nx_h]) {
        bb->data[Cinematique_ROS_B.m_me] = Cinematique_ROS_B.nx_h + 1;
        Cinematique_ROS_B.m_me++;
      }
    }

    Cinematique_ROS_B.inner = obj->ConstraintMatrix->size[0];
    Cinematique_ROS_B.nx_h = A->size[0] * A->size[1];
    A->size[0] = Cinematique_ROS_B.inner;
    A->size[1] = bb->size[0];
    Cinema_emxEnsureCapacity_real_T(A, Cinematique_ROS_B.nx_h);
    Cinematique_ROS_B.aoffset = bb->size[0];
    for (Cinematique_ROS_B.nx_h = 0; Cinematique_ROS_B.nx_h <
         Cinematique_ROS_B.aoffset; Cinematique_ROS_B.nx_h++) {
      for (Cinematique_ROS_B.m_me = 0; Cinematique_ROS_B.m_me <
           Cinematique_ROS_B.inner; Cinematique_ROS_B.m_me++) {
        A->data[Cinematique_ROS_B.m_me + A->size[0] * Cinematique_ROS_B.nx_h] =
          obj->ConstraintMatrix->data[(bb->data[Cinematique_ROS_B.nx_h] - 1) *
          obj->ConstraintMatrix->size[0] + Cinematique_ROS_B.m_me];
      }
    }
  } else {
    Cinematique_ROS_B.idxl = obj->ConstraintBound->size[0];
    Cinematique_ROS_B.nx_h = activeSet->size[0];
    activeSet->size[0] = Cinematique_ROS_B.idxl;
    Cin_emxEnsureCapacity_boolean_T(activeSet, Cinematique_ROS_B.nx_h);
    for (Cinematique_ROS_B.nx_h = 0; Cinematique_ROS_B.nx_h <
         Cinematique_ROS_B.idxl; Cinematique_ROS_B.nx_h++) {
      activeSet->data[Cinematique_ROS_B.nx_h] = false;
    }

    A->size[0] = 4;
    A->size[1] = 0;
  }

  Cinematique_ROS_B.j_l = A->size[1] - 1;
  Cinematique_ROS_emxInit_real_T(&tmp_0, 2);
  Cinematique_ROS_emxInit_real_T(&A_0, 2);
  Cinematique_ROS_emxInit_real_T(&A_1, 1);
  Cinematique_ROS_emxInit_real_T(&A_2, 2);
  for (Cinematique_ROS_B.idxl = 0; Cinematique_ROS_B.idxl <=
       Cinematique_ROS_B.j_l; Cinematique_ROS_B.idxl++) {
    Cinematique_ROS_B.inner = A->size[0];
    Cinematique_ROS_B.nx_h = A_0->size[0] * A_0->size[1];
    A_0->size[0] = 1;
    A_0->size[1] = Cinematique_ROS_B.inner;
    Cinema_emxEnsureCapacity_real_T(A_0, Cinematique_ROS_B.nx_h);
    for (Cinematique_ROS_B.nx_h = 0; Cinematique_ROS_B.nx_h <
         Cinematique_ROS_B.inner; Cinematique_ROS_B.nx_h++) {
      A_0->data[Cinematique_ROS_B.nx_h] = A->data[A->size[0] *
        Cinematique_ROS_B.idxl + Cinematique_ROS_B.nx_h];
    }

    Cinematique_ROS_B.inner = A->size[0];
    Cinematique_ROS_B.nx_h = A_1->size[0];
    A_1->size[0] = Cinematique_ROS_B.inner;
    Cinema_emxEnsureCapacity_real_T(A_1, Cinematique_ROS_B.nx_h);
    for (Cinematique_ROS_B.nx_h = 0; Cinematique_ROS_B.nx_h <
         Cinematique_ROS_B.inner; Cinematique_ROS_B.nx_h++) {
      A_1->data[Cinematique_ROS_B.nx_h] = A->data[A->size[0] *
        Cinematique_ROS_B.idxl + Cinematique_ROS_B.nx_h];
    }

    Cinematique_ROS_B.A_a = 0.0;
    for (Cinematique_ROS_B.nx_h = 0; Cinematique_ROS_B.nx_h < 4;
         Cinematique_ROS_B.nx_h++) {
      Cinematique_ROS_B.m_me = Cinematique_ROS_B.nx_h << 2;
      Cinematique_ROS_B.beta = Cinematique_ROS_B.H[Cinematique_ROS_B.m_me] *
        A_0->data[0];
      Cinematique_ROS_B.beta += Cinematique_ROS_B.H[Cinematique_ROS_B.m_me + 1] *
        A_0->data[1];
      Cinematique_ROS_B.beta += Cinematique_ROS_B.H[Cinematique_ROS_B.m_me + 2] *
        A_0->data[2];
      Cinematique_ROS_B.beta += Cinematique_ROS_B.H[Cinematique_ROS_B.m_me + 3] *
        A_0->data[3];
      Cinematique_ROS_B.A_a += Cinematique_ROS_B.beta * A_1->
        data[Cinematique_ROS_B.nx_h];
    }

    Cinematique_ROS_B.theta = 1.0 / Cinematique_ROS_B.A_a;
    for (Cinematique_ROS_B.nx_h = 0; Cinematique_ROS_B.nx_h < 16;
         Cinematique_ROS_B.nx_h++) {
      Cinematique_ROS_B.V[Cinematique_ROS_B.nx_h] = Cinematique_ROS_B.theta *
        Cinematique_ROS_B.H[Cinematique_ROS_B.nx_h];
    }

    Cinematique_ROS_B.inner = A->size[0];
    Cinematique_ROS_B.aoffset = A->size[0];
    Cinematique_ROS_B.nx_h = A_2->size[0] * A_2->size[1];
    A_2->size[0] = Cinematique_ROS_B.inner;
    A_2->size[1] = Cinematique_ROS_B.aoffset;
    Cinema_emxEnsureCapacity_real_T(A_2, Cinematique_ROS_B.nx_h);
    for (Cinematique_ROS_B.nx_h = 0; Cinematique_ROS_B.nx_h <
         Cinematique_ROS_B.aoffset; Cinematique_ROS_B.nx_h++) {
      for (Cinematique_ROS_B.m_me = 0; Cinematique_ROS_B.m_me <
           Cinematique_ROS_B.inner; Cinematique_ROS_B.m_me++) {
        A_2->data[Cinematique_ROS_B.m_me + A_2->size[0] * Cinematique_ROS_B.nx_h]
          = A->data[A->size[0] * Cinematique_ROS_B.idxl + Cinematique_ROS_B.m_me]
          * A->data[A->size[0] * Cinematique_ROS_B.idxl + Cinematique_ROS_B.nx_h];
      }
    }

    Cinematique_ROS_mtimes_k1(Cinematique_ROS_B.V, A_2, tmp_0);
    for (Cinematique_ROS_B.nx_h = 0; Cinematique_ROS_B.nx_h < 4;
         Cinematique_ROS_B.nx_h++) {
      for (Cinematique_ROS_B.m_me = 0; Cinematique_ROS_B.m_me < 4;
           Cinematique_ROS_B.m_me++) {
        Cinematique_ROS_B.inner = Cinematique_ROS_B.m_me << 2;
        Cinematique_ROS_B.lambda = Cinematique_ROS_B.H[Cinematique_ROS_B.inner] *
          tmp_0->data[Cinematique_ROS_B.nx_h];
        Cinematique_ROS_B.lambda += Cinematique_ROS_B.H[Cinematique_ROS_B.inner
          + 1] * tmp_0->data[Cinematique_ROS_B.nx_h + 4];
        Cinematique_ROS_B.lambda += Cinematique_ROS_B.H[Cinematique_ROS_B.inner
          + 2] * tmp_0->data[Cinematique_ROS_B.nx_h + 8];
        Cinematique_ROS_B.lambda += Cinematique_ROS_B.H[Cinematique_ROS_B.inner
          + 3] * tmp_0->data[Cinematique_ROS_B.nx_h + 12];
        Cinematique_ROS_B.inner += Cinematique_ROS_B.nx_h;
        Cinematique_ROS_B.V[Cinematique_ROS_B.inner] =
          Cinematique_ROS_B.H[Cinematique_ROS_B.inner] -
          Cinematique_ROS_B.lambda;
      }
    }

    memcpy(&Cinematique_ROS_B.H[0], &Cinematique_ROS_B.V[0], sizeof(real_T) <<
           4U);
  }

  Cinematique_ROS_emxFree_real_T(&A_2);
  Cinematique_ROS_emxFree_real_T(&A_1);
  Cinematique_ROS_emxFree_real_T(&A_0);
  xSol[0] = Cinematique_ROS_B.x[0];
  xSol[1] = Cinematique_ROS_B.x[1];
  xSol[2] = Cinematique_ROS_B.x[2];
  xSol[3] = Cinematique_ROS_B.x[3];
  Cinematique_ROS_B.A_a = obj->MaxNumIterationInternal;
  Cinematique_ROS_B.j_l = 0;
  Cinematique_ROS_emxInit_real_T(&alpha, 1);
  Cinematique_ROS_emxInit_real_T(&AIn, 2);
  Cinematique_ROS_emxInit_int32_T(&cb, 1);
  Cinematique_ROS_emxInit_int32_T(&db, 1);
  Cinematique_ROS_emxInit_int32_T(&eb, 1);
  Cinematique_ROS_emxInit_int32_T(&fb, 1);
  Cinematique_ROS_emxInit_int32_T(&gb, 1);
  Cinematique_R_emxInit_boolean_T(&tmp, 1);
  Cinematique_ROS_emxInit_real_T(&tmp_1, 2);
  Cinematique_ROS_emxInit_real_T(&tmp_2, 1);
  Cinematique_ROS_emxInit_real_T(&tmp_3, 2);
  Cinematique_ROS_emxInit_real_T(&A_3, 2);
  Cinematique_ROS_emxInit_real_T(&alpha_0, 1);
  Cinematique_ROS_emxInit_real_T(&sigma, 2);
  Cinematique_ROS_emxInit_real_T(&tmp_4, 2);
  Cinematique_ROS_emxInit_real_T(&tmp_5, 2);
  Cinematique_ROS_emxInit_real_T(&grad_0, 2);
  Cinematique_ROS_emxInit_real_T(&A_4, 2);
  Cinematique_R_emxInit_boolean_T(&alpha_1, 1);
  Cinematique_ROS_emxInit_real_T(&alpha_2, 2);
  Cinematique_ROS_emxInit_real_T(&alpha_3, 2);
  Cinematique_ROS_emxInit_real_T(&sNew, 2);
  Cinematique_ROS_emxInit_real_T(&grad_1, 2);
  Cinematique_ROS_emxInit_real_T(&grad_2, 2);
  Cinematique_ROS_emxInit_real_T(&alpha_4, 1);
  Cinematique_R_emxInit_boolean_T(&L_0, 1);
  do {
    exitg2 = 0;
    if (Cinematique_ROS_B.j_l <= static_cast<int32_T>(Cinematique_ROS_B.A_a) - 1)
    {
      Cinematique_ROS_B.beta = Cinematique_ROS_toc
        (obj->TimeObjInternal.StartTime.tv_sec,
         obj->TimeObjInternal.StartTime.tv_nsec);
      Cinematique_ROS_B.flag = (Cinematique_ROS_B.beta > obj->MaxTimeInternal);
      if (Cinematique_ROS_B.flag) {
        *exitFlag = TimeLimitExceeded;
        *err = Cine_IKHelpers_evaluateSolution(obj->ExtraArgs);
        *iter = static_cast<real_T>(Cinematique_ROS_B.j_l) + 1.0;
        exitg2 = 1;
      } else {
        if ((A->size[0] == 0) || (A->size[1] == 0)) {
          Cinematique_ROS_B.nx_h = alpha->size[0];
          alpha->size[0] = 1;
          Cinema_emxEnsureCapacity_real_T(alpha, Cinematique_ROS_B.nx_h);
          alpha->data[0] = 0.0;
        } else {
          Cinematique_ROS_mtimes_k1e(A, A, tmp_1);
          Cinematique_ROS_B.nx_h = A_3->size[0] * A_3->size[1];
          A_3->size[0] = A->size[1];
          A_3->size[1] = A->size[0];
          Cinema_emxEnsureCapacity_real_T(A_3, Cinematique_ROS_B.nx_h);
          Cinematique_ROS_B.inner = A->size[0];
          for (Cinematique_ROS_B.nx_h = 0; Cinematique_ROS_B.nx_h <
               Cinematique_ROS_B.inner; Cinematique_ROS_B.nx_h++) {
            Cinematique_ROS_B.aoffset = A->size[1];
            for (Cinematique_ROS_B.m_me = 0; Cinematique_ROS_B.m_me <
                 Cinematique_ROS_B.aoffset; Cinematique_ROS_B.m_me++) {
              A_3->data[Cinematique_ROS_B.m_me + A_3->size[0] *
                Cinematique_ROS_B.nx_h] = A->data[A->size[0] *
                Cinematique_ROS_B.m_me + Cinematique_ROS_B.nx_h];
            }
          }

          Cinematique_ROS_mldivide(tmp_1, A_3, AIn);
          Cinematique_ROS_B.m_me = AIn->size[0] - 1;
          Cinematique_ROS_B.inner = AIn->size[1] - 1;
          Cinematique_ROS_B.nx_h = alpha->size[0];
          alpha->size[0] = AIn->size[0];
          Cinema_emxEnsureCapacity_real_T(alpha, Cinematique_ROS_B.nx_h);
          for (Cinematique_ROS_B.nx_h = 0; Cinematique_ROS_B.nx_h <=
               Cinematique_ROS_B.m_me; Cinematique_ROS_B.nx_h++) {
            alpha->data[Cinematique_ROS_B.nx_h] = 0.0;
          }

          for (Cinematique_ROS_B.idxl = 0; Cinematique_ROS_B.idxl <=
               Cinematique_ROS_B.inner; Cinematique_ROS_B.idxl++) {
            Cinematique_ROS_B.aoffset = Cinematique_ROS_B.idxl * AIn->size[0] -
              1;
            for (Cinematique_ROS_B.nx_h = 0; Cinematique_ROS_B.nx_h <=
                 Cinematique_ROS_B.m_me; Cinematique_ROS_B.nx_h++) {
              alpha->data[Cinematique_ROS_B.nx_h] += AIn->data
                [(Cinematique_ROS_B.aoffset + Cinematique_ROS_B.nx_h) + 1] *
                grad->data[Cinematique_ROS_B.idxl];
            }
          }
        }

        for (Cinematique_ROS_B.nx_h = 0; Cinematique_ROS_B.nx_h < 4;
             Cinematique_ROS_B.nx_h++) {
          Cinematique_ROS_B.theta = Cinematique_ROS_B.H[Cinematique_ROS_B.nx_h] *
            grad->data[0];
          Cinematique_ROS_B.theta += Cinematique_ROS_B.H[Cinematique_ROS_B.nx_h
            + 4] * grad->data[1];
          Cinematique_ROS_B.theta += Cinematique_ROS_B.H[Cinematique_ROS_B.nx_h
            + 8] * grad->data[2];
          Cinematique_ROS_B.theta += Cinematique_ROS_B.H[Cinematique_ROS_B.nx_h
            + 12] * grad->data[3];
          Cinematique_ROS_B.Hg[Cinematique_ROS_B.nx_h] = Cinematique_ROS_B.theta;
        }

        if (DampedBFGSwGradientProjection_a(obj, Cinematique_ROS_B.Hg, alpha)) {
          *exitFlag = LocalMinimumFound;
          *err = Cine_IKHelpers_evaluateSolution(obj->ExtraArgs);
          *iter = static_cast<real_T>(Cinematique_ROS_B.j_l) + 1.0;
          exitg2 = 1;
        } else {
          guard1 = false;
          guard2 = false;
          guard3 = false;
          if (obj->ConstraintsOn && ((A->size[0] != 0) && (A->size[1] != 0))) {
            Cinematique_ROS_mtimes_k1e(A, A, tmp_1);
            Cinematique_ROS_inv(tmp_1, AIn);
            Cinematique_ROS_diag(AIn, L);
            Cinematique_ROS_B.nx_h = L->size[0] - 1;
            for (Cinematique_ROS_B.idxl = 0; Cinematique_ROS_B.idxl <=
                 Cinematique_ROS_B.nx_h; Cinematique_ROS_B.idxl++) {
              L->data[Cinematique_ROS_B.idxl] = sqrt(L->
                data[Cinematique_ROS_B.idxl]);
            }

            Cinematique_ROS_B.nx_h = alpha_0->size[0];
            alpha_0->size[0] = alpha->size[0];
            Cinema_emxEnsureCapacity_real_T(alpha_0, Cinematique_ROS_B.nx_h);
            Cinematique_ROS_B.inner = alpha->size[0];
            for (Cinematique_ROS_B.nx_h = 0; Cinematique_ROS_B.nx_h <
                 Cinematique_ROS_B.inner; Cinematique_ROS_B.nx_h++) {
              alpha_0->data[Cinematique_ROS_B.nx_h] = alpha->
                data[Cinematique_ROS_B.nx_h] / L->data[Cinematique_ROS_B.nx_h];
            }

            Cinematique_ROS_maximum(alpha_0, &Cinematique_ROS_B.beta,
              &Cinematique_ROS_B.idxl);
            if (Cinematique_ROS_norm_k(Cinematique_ROS_B.Hg) < 0.5 *
                Cinematique_ROS_B.beta) {
              Cinematique_ROS_eml_find_k(activeSet, bb);
              Cinematique_ROS_B.nx_h = alpha->size[0];
              alpha->size[0] = bb->size[0];
              Cinema_emxEnsureCapacity_real_T(alpha, Cinematique_ROS_B.nx_h);
              Cinematique_ROS_B.inner = bb->size[0];
              for (Cinematique_ROS_B.nx_h = 0; Cinematique_ROS_B.nx_h <
                   Cinematique_ROS_B.inner; Cinematique_ROS_B.nx_h++) {
                alpha->data[Cinematique_ROS_B.nx_h] = bb->
                  data[Cinematique_ROS_B.nx_h];
              }

              activeSet->data[static_cast<int32_T>(alpha->
                data[Cinematique_ROS_B.idxl - 1]) - 1] = false;
              Cinematique_ROS_B.m_me = activeSet->size[0] - 1;
              Cinematique_ROS_B.inner = 0;
              for (Cinematique_ROS_B.nx_h = 0; Cinematique_ROS_B.nx_h <=
                   Cinematique_ROS_B.m_me; Cinematique_ROS_B.nx_h++) {
                if (activeSet->data[Cinematique_ROS_B.nx_h]) {
                  Cinematique_ROS_B.inner++;
                }
              }

              Cinematique_ROS_B.nx_h = eb->size[0];
              eb->size[0] = Cinematique_ROS_B.inner;
              Cinem_emxEnsureCapacity_int32_T(eb, Cinematique_ROS_B.nx_h);
              Cinematique_ROS_B.inner = 0;
              for (Cinematique_ROS_B.nx_h = 0; Cinematique_ROS_B.nx_h <=
                   Cinematique_ROS_B.m_me; Cinematique_ROS_B.nx_h++) {
                if (activeSet->data[Cinematique_ROS_B.nx_h]) {
                  eb->data[Cinematique_ROS_B.inner] = Cinematique_ROS_B.nx_h + 1;
                  Cinematique_ROS_B.inner++;
                }
              }

              Cinematique_ROS_B.inner = obj->ConstraintMatrix->size[0];
              Cinematique_ROS_B.nx_h = A->size[0] * A->size[1];
              A->size[0] = Cinematique_ROS_B.inner;
              A->size[1] = eb->size[0];
              Cinema_emxEnsureCapacity_real_T(A, Cinematique_ROS_B.nx_h);
              Cinematique_ROS_B.aoffset = eb->size[0];
              for (Cinematique_ROS_B.nx_h = 0; Cinematique_ROS_B.nx_h <
                   Cinematique_ROS_B.aoffset; Cinematique_ROS_B.nx_h++) {
                for (Cinematique_ROS_B.m_me = 0; Cinematique_ROS_B.m_me <
                     Cinematique_ROS_B.inner; Cinematique_ROS_B.m_me++) {
                  A->data[Cinematique_ROS_B.m_me + A->size[0] *
                    Cinematique_ROS_B.nx_h] = obj->ConstraintMatrix->data
                    [(eb->data[Cinematique_ROS_B.nx_h] - 1) *
                    obj->ConstraintMatrix->size[0] + Cinematique_ROS_B.m_me];
                }
              }

              memset(&Cinematique_ROS_B.P[0], 0, sizeof(real_T) << 4U);
              Cinematique_ROS_B.P[0] = 1.0;
              Cinematique_ROS_B.P[5] = 1.0;
              Cinematique_ROS_B.P[10] = 1.0;
              Cinematique_ROS_B.P[15] = 1.0;
              Cinematique_ROS_mtimes_k1e(A, A, tmp_1);
              Cinematique_ROS_B.nx_h = A_4->size[0] * A_4->size[1];
              A_4->size[0] = A->size[1];
              A_4->size[1] = A->size[0];
              Cinema_emxEnsureCapacity_real_T(A_4, Cinematique_ROS_B.nx_h);
              Cinematique_ROS_B.inner = A->size[0];
              for (Cinematique_ROS_B.nx_h = 0; Cinematique_ROS_B.nx_h <
                   Cinematique_ROS_B.inner; Cinematique_ROS_B.nx_h++) {
                Cinematique_ROS_B.aoffset = A->size[1];
                for (Cinematique_ROS_B.m_me = 0; Cinematique_ROS_B.m_me <
                     Cinematique_ROS_B.aoffset; Cinematique_ROS_B.m_me++) {
                  A_4->data[Cinematique_ROS_B.m_me + A_4->size[0] *
                    Cinematique_ROS_B.nx_h] = A->data[A->size[0] *
                    Cinematique_ROS_B.m_me + Cinematique_ROS_B.nx_h];
                }
              }

              Cinematique_ROS_mldivide(tmp_1, A_4, AIn);
              Cinematique_ROS_mtimes_k1e1(A, AIn, tmp_1);
              for (Cinematique_ROS_B.nx_h = 0; Cinematique_ROS_B.nx_h < 16;
                   Cinematique_ROS_B.nx_h++) {
                Cinematique_ROS_B.beta =
                  Cinematique_ROS_B.P[Cinematique_ROS_B.nx_h] - tmp_1->
                  data[Cinematique_ROS_B.nx_h];
                Cinematique_ROS_B.P[Cinematique_ROS_B.nx_h] =
                  Cinematique_ROS_B.beta;
              }

              Cinematique_ROS_B.beta = alpha->data[Cinematique_ROS_B.idxl - 1];
              Cinematique_ROS_B.inner = obj->ConstraintMatrix->size[0];
              Cinematique_ROS_B.nx_h = alpha->size[0];
              alpha->size[0] = Cinematique_ROS_B.inner;
              Cinema_emxEnsureCapacity_real_T(alpha, Cinematique_ROS_B.nx_h);
              for (Cinematique_ROS_B.nx_h = 0; Cinematique_ROS_B.nx_h <
                   Cinematique_ROS_B.inner; Cinematique_ROS_B.nx_h++) {
                alpha->data[Cinematique_ROS_B.nx_h] = obj->
                  ConstraintMatrix->data[(static_cast<int32_T>
                  (Cinematique_ROS_B.beta) - 1) * obj->ConstraintMatrix->size[0]
                  + Cinematique_ROS_B.nx_h];
              }

              Cinematique_ROS_B.nx_h = alpha_2->size[0] * alpha_2->size[1];
              alpha_2->size[0] = 1;
              alpha_2->size[1] = alpha->size[0];
              Cinema_emxEnsureCapacity_real_T(alpha_2, Cinematique_ROS_B.nx_h);
              Cinematique_ROS_B.inner = alpha->size[0];
              for (Cinematique_ROS_B.nx_h = 0; Cinematique_ROS_B.nx_h <
                   Cinematique_ROS_B.inner; Cinematique_ROS_B.nx_h++) {
                alpha_2->data[Cinematique_ROS_B.nx_h] = alpha->
                  data[Cinematique_ROS_B.nx_h];
              }

              Cinematique_ROS_B.beta = 0.0;
              for (Cinematique_ROS_B.nx_h = 0; Cinematique_ROS_B.nx_h < 4;
                   Cinematique_ROS_B.nx_h++) {
                Cinematique_ROS_B.idxl = Cinematique_ROS_B.nx_h << 2;
                Cinematique_ROS_B.theta =
                  Cinematique_ROS_B.P[Cinematique_ROS_B.idxl] * alpha_2->data[0];
                Cinematique_ROS_B.theta +=
                  Cinematique_ROS_B.P[Cinematique_ROS_B.idxl + 1] *
                  alpha_2->data[1];
                Cinematique_ROS_B.theta +=
                  Cinematique_ROS_B.P[Cinematique_ROS_B.idxl + 2] *
                  alpha_2->data[2];
                Cinematique_ROS_B.theta +=
                  Cinematique_ROS_B.P[Cinematique_ROS_B.idxl + 3] *
                  alpha_2->data[3];
                Cinematique_ROS_B.beta += Cinematique_ROS_B.theta * alpha->
                  data[Cinematique_ROS_B.nx_h];
              }

              Cinematique_ROS_B.theta = 1.0 / Cinematique_ROS_B.beta;
              for (Cinematique_ROS_B.nx_h = 0; Cinematique_ROS_B.nx_h < 16;
                   Cinematique_ROS_B.nx_h++) {
                Cinematique_ROS_B.V[Cinematique_ROS_B.nx_h] =
                  Cinematique_ROS_B.theta *
                  Cinematique_ROS_B.P[Cinematique_ROS_B.nx_h];
              }

              Cinematique_ROS_B.nx_h = alpha_3->size[0] * alpha_3->size[1];
              alpha_3->size[0] = alpha->size[0];
              alpha_3->size[1] = alpha->size[0];
              Cinema_emxEnsureCapacity_real_T(alpha_3, Cinematique_ROS_B.nx_h);
              Cinematique_ROS_B.inner = alpha->size[0];
              for (Cinematique_ROS_B.nx_h = 0; Cinematique_ROS_B.nx_h <
                   Cinematique_ROS_B.inner; Cinematique_ROS_B.nx_h++) {
                Cinematique_ROS_B.aoffset = alpha->size[0];
                for (Cinematique_ROS_B.m_me = 0; Cinematique_ROS_B.m_me <
                     Cinematique_ROS_B.aoffset; Cinematique_ROS_B.m_me++) {
                  alpha_3->data[Cinematique_ROS_B.m_me + alpha_3->size[0] *
                    Cinematique_ROS_B.nx_h] = alpha->data[Cinematique_ROS_B.m_me]
                    * alpha->data[Cinematique_ROS_B.nx_h];
                }
              }

              Cinematique_ROS_mtimes_k1(Cinematique_ROS_B.V, alpha_3, tmp_0);
              for (Cinematique_ROS_B.nx_h = 0; Cinematique_ROS_B.nx_h < 4;
                   Cinematique_ROS_B.nx_h++) {
                for (Cinematique_ROS_B.m_me = 0; Cinematique_ROS_B.m_me < 4;
                     Cinematique_ROS_B.m_me++) {
                  Cinematique_ROS_B.inner = Cinematique_ROS_B.m_me << 2;
                  Cinematique_ROS_B.lambda =
                    Cinematique_ROS_B.P[Cinematique_ROS_B.inner] * tmp_0->
                    data[Cinematique_ROS_B.nx_h];
                  Cinematique_ROS_B.lambda +=
                    Cinematique_ROS_B.P[Cinematique_ROS_B.inner + 1] *
                    tmp_0->data[Cinematique_ROS_B.nx_h + 4];
                  Cinematique_ROS_B.lambda +=
                    Cinematique_ROS_B.P[Cinematique_ROS_B.inner + 2] *
                    tmp_0->data[Cinematique_ROS_B.nx_h + 8];
                  Cinematique_ROS_B.lambda +=
                    Cinematique_ROS_B.P[Cinematique_ROS_B.inner + 3] *
                    tmp_0->data[Cinematique_ROS_B.nx_h + 12];
                  Cinematique_ROS_B.inner += Cinematique_ROS_B.nx_h;
                  Cinematique_ROS_B.H[Cinematique_ROS_B.inner] +=
                    Cinematique_ROS_B.lambda;
                }
              }

              Cinematique_ROS_B.j_l++;
            } else {
              guard3 = true;
            }
          } else {
            guard3 = true;
          }

          if (guard3) {
            Cinematique_ROS_B.Hg[0] = -Cinematique_ROS_B.Hg[0];
            Cinematique_ROS_B.Hg[1] = -Cinematique_ROS_B.Hg[1];
            Cinematique_ROS_B.Hg[2] = -Cinematique_ROS_B.Hg[2];
            Cinematique_ROS_B.theta = -Cinematique_ROS_B.Hg[3];
            Cinematique_ROS_B.Hg[3] = -Cinematique_ROS_B.Hg[3];
            Cinematique_ROS_B.idxl = -1;
            if (obj->ConstraintsOn) {
              Cinematique_ROS_B.nx_h = tmp->size[0];
              tmp->size[0] = activeSet->size[0];
              Cin_emxEnsureCapacity_boolean_T(tmp, Cinematique_ROS_B.nx_h);
              Cinematique_ROS_B.inner = activeSet->size[0];
              for (Cinematique_ROS_B.nx_h = 0; Cinematique_ROS_B.nx_h <
                   Cinematique_ROS_B.inner; Cinematique_ROS_B.nx_h++) {
                tmp->data[Cinematique_ROS_B.nx_h] = !activeSet->
                  data[Cinematique_ROS_B.nx_h];
              }

              if (Cinematique_ROS_any(tmp)) {
                Cinematique_ROS_B.m_me = activeSet->size[0] - 1;
                Cinematique_ROS_B.inner = 0;
                for (Cinematique_ROS_B.nx_h = 0; Cinematique_ROS_B.nx_h <=
                     Cinematique_ROS_B.m_me; Cinematique_ROS_B.nx_h++) {
                  if (!activeSet->data[Cinematique_ROS_B.nx_h]) {
                    Cinematique_ROS_B.inner++;
                  }
                }

                Cinematique_ROS_B.nx_h = cb->size[0];
                cb->size[0] = Cinematique_ROS_B.inner;
                Cinem_emxEnsureCapacity_int32_T(cb, Cinematique_ROS_B.nx_h);
                Cinematique_ROS_B.inner = 0;
                for (Cinematique_ROS_B.nx_h = 0; Cinematique_ROS_B.nx_h <=
                     Cinematique_ROS_B.m_me; Cinematique_ROS_B.nx_h++) {
                  if (!activeSet->data[Cinematique_ROS_B.nx_h]) {
                    cb->data[Cinematique_ROS_B.inner] = Cinematique_ROS_B.nx_h +
                      1;
                    Cinematique_ROS_B.inner++;
                  }
                }

                Cinematique_ROS_B.nx_h = alpha->size[0];
                alpha->size[0] = cb->size[0];
                Cinema_emxEnsureCapacity_real_T(alpha, Cinematique_ROS_B.nx_h);
                Cinematique_ROS_B.inner = cb->size[0];
                for (Cinematique_ROS_B.nx_h = 0; Cinematique_ROS_B.nx_h <
                     Cinematique_ROS_B.inner; Cinematique_ROS_B.nx_h++) {
                  alpha->data[Cinematique_ROS_B.nx_h] = obj->
                    ConstraintBound->data[cb->data[Cinematique_ROS_B.nx_h] - 1];
                }

                Cinematique_ROS_B.m_me = activeSet->size[0] - 1;
                Cinematique_ROS_B.inner = 0;
                for (Cinematique_ROS_B.nx_h = 0; Cinematique_ROS_B.nx_h <=
                     Cinematique_ROS_B.m_me; Cinematique_ROS_B.nx_h++) {
                  if (!activeSet->data[Cinematique_ROS_B.nx_h]) {
                    Cinematique_ROS_B.inner++;
                  }
                }

                Cinematique_ROS_B.nx_h = db->size[0];
                db->size[0] = Cinematique_ROS_B.inner;
                Cinem_emxEnsureCapacity_int32_T(db, Cinematique_ROS_B.nx_h);
                Cinematique_ROS_B.inner = 0;
                for (Cinematique_ROS_B.nx_h = 0; Cinematique_ROS_B.nx_h <=
                     Cinematique_ROS_B.m_me; Cinematique_ROS_B.nx_h++) {
                  if (!activeSet->data[Cinematique_ROS_B.nx_h]) {
                    db->data[Cinematique_ROS_B.inner] = Cinematique_ROS_B.nx_h +
                      1;
                    Cinematique_ROS_B.inner++;
                  }
                }

                Cinematique_ROS_B.inner = obj->ConstraintMatrix->size[0];
                Cinematique_ROS_B.nx_h = AIn->size[0] * AIn->size[1];
                AIn->size[0] = Cinematique_ROS_B.inner;
                AIn->size[1] = db->size[0];
                Cinema_emxEnsureCapacity_real_T(AIn, Cinematique_ROS_B.nx_h);
                Cinematique_ROS_B.aoffset = db->size[0];
                for (Cinematique_ROS_B.nx_h = 0; Cinematique_ROS_B.nx_h <
                     Cinematique_ROS_B.aoffset; Cinematique_ROS_B.nx_h++) {
                  for (Cinematique_ROS_B.m_me = 0; Cinematique_ROS_B.m_me <
                       Cinematique_ROS_B.inner; Cinematique_ROS_B.m_me++) {
                    AIn->data[Cinematique_ROS_B.m_me + AIn->size[0] *
                      Cinematique_ROS_B.nx_h] = obj->ConstraintMatrix->data
                      [(db->data[Cinematique_ROS_B.nx_h] - 1) *
                      obj->ConstraintMatrix->size[0] + Cinematique_ROS_B.m_me];
                  }
                }

                Cinematique_ROS_mtimes_k(AIn, Cinematique_ROS_B.x, L);
                Cinematique_ROS_mtimes_k(AIn, Cinematique_ROS_B.Hg, tmp_2);
                Cinematique_ROS_B.nx_h = alpha->size[0];
                Cinema_emxEnsureCapacity_real_T(alpha, Cinematique_ROS_B.nx_h);
                Cinematique_ROS_B.inner = alpha->size[0];
                for (Cinematique_ROS_B.nx_h = 0; Cinematique_ROS_B.nx_h <
                     Cinematique_ROS_B.inner; Cinematique_ROS_B.nx_h++) {
                  alpha->data[Cinematique_ROS_B.nx_h] = (alpha->
                    data[Cinematique_ROS_B.nx_h] - L->
                    data[Cinematique_ROS_B.nx_h]) / tmp_2->
                    data[Cinematique_ROS_B.nx_h];
                }

                Cinematique_ROS_B.nx_h = alpha_1->size[0];
                alpha_1->size[0] = alpha->size[0];
                Cin_emxEnsureCapacity_boolean_T(alpha_1, Cinematique_ROS_B.nx_h);
                Cinematique_ROS_B.inner = alpha->size[0];
                for (Cinematique_ROS_B.nx_h = 0; Cinematique_ROS_B.nx_h <
                     Cinematique_ROS_B.inner; Cinematique_ROS_B.nx_h++) {
                  alpha_1->data[Cinematique_ROS_B.nx_h] = (alpha->
                    data[Cinematique_ROS_B.nx_h] > 0.0);
                }

                Cinematique_ROS_eml_find_k(alpha_1, bb);
                Cinematique_ROS_B.nx_h = L->size[0];
                L->size[0] = bb->size[0];
                Cinema_emxEnsureCapacity_real_T(L, Cinematique_ROS_B.nx_h);
                Cinematique_ROS_B.inner = bb->size[0];
                for (Cinematique_ROS_B.nx_h = 0; Cinematique_ROS_B.nx_h <
                     Cinematique_ROS_B.inner; Cinematique_ROS_B.nx_h++) {
                  L->data[Cinematique_ROS_B.nx_h] = bb->
                    data[Cinematique_ROS_B.nx_h];
                }

                if (L->size[0] != 0) {
                  Cinematique_ROS_B.idxl = alpha->size[0] - 1;
                  Cinematique_ROS_B.m_me = 0;
                  for (Cinematique_ROS_B.nx_h = 0; Cinematique_ROS_B.nx_h <=
                       Cinematique_ROS_B.idxl; Cinematique_ROS_B.nx_h++) {
                    if (alpha->data[Cinematique_ROS_B.nx_h] > 0.0) {
                      Cinematique_ROS_B.m_me++;
                    }
                  }

                  Cinematique_ROS_B.nx_h = fb->size[0];
                  fb->size[0] = Cinematique_ROS_B.m_me;
                  Cinem_emxEnsureCapacity_int32_T(fb, Cinematique_ROS_B.nx_h);
                  Cinematique_ROS_B.m_me = 0;
                  for (Cinematique_ROS_B.nx_h = 0; Cinematique_ROS_B.nx_h <=
                       Cinematique_ROS_B.idxl; Cinematique_ROS_B.nx_h++) {
                    if (alpha->data[Cinematique_ROS_B.nx_h] > 0.0) {
                      fb->data[Cinematique_ROS_B.m_me] = Cinematique_ROS_B.nx_h
                        + 1;
                      Cinematique_ROS_B.m_me++;
                    }
                  }

                  Cinematique_ROS_B.nx_h = alpha_4->size[0];
                  alpha_4->size[0] = fb->size[0];
                  Cinema_emxEnsureCapacity_real_T(alpha_4,
                    Cinematique_ROS_B.nx_h);
                  Cinematique_ROS_B.inner = fb->size[0];
                  for (Cinematique_ROS_B.nx_h = 0; Cinematique_ROS_B.nx_h <
                       Cinematique_ROS_B.inner; Cinematique_ROS_B.nx_h++) {
                    alpha_4->data[Cinematique_ROS_B.nx_h] = alpha->data[fb->
                      data[Cinematique_ROS_B.nx_h] - 1];
                  }

                  Cinematique_ROS_minimum(alpha_4, &Cinematique_ROS_B.lambda,
                    &Cinematique_ROS_B.idxl);
                  Cinematique_ROS_eml_find_k(tmp, bb);
                  Cinematique_ROS_B.idxl = bb->data[static_cast<int32_T>(L->
                    data[Cinematique_ROS_B.idxl - 1]) - 1];
                } else {
                  Cinematique_ROS_B.lambda = 0.0;
                }
              } else {
                Cinematique_ROS_B.lambda = 0.0;
              }
            } else {
              Cinematique_ROS_B.lambda = 0.0;
            }

            if (Cinematique_ROS_B.lambda > 0.0) {
              if (1.0 < Cinematique_ROS_B.lambda) {
                Cinematique_ROS_B.b_gamma = 1.0;
              } else {
                Cinematique_ROS_B.b_gamma = Cinematique_ROS_B.lambda;
              }
            } else {
              Cinematique_ROS_B.b_gamma = 1.0;
            }

            Cinematique_ROS_B.beta = obj->ArmijoRuleBeta;
            Cinematique_ROS_B.sigma = obj->ArmijoRuleSigma;
            Cinematique_ROS_B.sNew[0] = Cinematique_ROS_B.b_gamma *
              Cinematique_ROS_B.Hg[0] + Cinematique_ROS_B.x[0];
            Cinematique_ROS_B.sNew[1] = Cinematique_ROS_B.b_gamma *
              Cinematique_ROS_B.Hg[1] + Cinematique_ROS_B.x[1];
            Cinematique_ROS_B.sNew[2] = Cinematique_ROS_B.b_gamma *
              Cinematique_ROS_B.Hg[2] + Cinematique_ROS_B.x[2];
            Cinematique_ROS_B.sNew[3] = Cinematique_ROS_B.b_gamma *
              Cinematique_ROS_B.theta + Cinematique_ROS_B.x[3];
            Cinematiq_IKHelpers_computeCost(Cinematique_ROS_B.sNew,
              obj->ExtraArgs, &Cinematique_ROS_B.costNew,
              Cinematique_ROS_B.unusedU0, unusedU1, &c);
            obj->ExtraArgs = c;
            Cinematique_ROS_B.m_f = 0.0;
            do {
              exitg1 = 0;
              xSol[0] = Cinematique_ROS_B.b_gamma * Cinematique_ROS_B.Hg[0];
              xSol[1] = Cinematique_ROS_B.b_gamma * Cinematique_ROS_B.Hg[1];
              xSol[2] = Cinematique_ROS_B.b_gamma * Cinematique_ROS_B.Hg[2];
              xSol[3] = Cinematique_ROS_B.b_gamma * Cinematique_ROS_B.theta;
              Cinematique_ROS_B.nx_h = sigma->size[0] * sigma->size[1];
              sigma->size[0] = 1;
              sigma->size[1] = grad->size[0];
              Cinema_emxEnsureCapacity_real_T(sigma, Cinematique_ROS_B.nx_h);
              Cinematique_ROS_B.inner = grad->size[0];
              for (Cinematique_ROS_B.nx_h = 0; Cinematique_ROS_B.nx_h <
                   Cinematique_ROS_B.inner; Cinematique_ROS_B.nx_h++) {
                sigma->data[Cinematique_ROS_B.nx_h] = -Cinematique_ROS_B.sigma *
                  grad->data[Cinematique_ROS_B.nx_h];
              }

              Cinematique_ROS_B.sigma_j = sigma->data[0] * xSol[0];
              Cinematique_ROS_B.sigma_j += sigma->data[1] * xSol[1];
              Cinematique_ROS_B.sigma_j += sigma->data[2] * xSol[2];
              Cinematique_ROS_B.sigma_j += sigma->data[3] * xSol[3];
              if (Cinematique_ROS_B.cost - Cinematique_ROS_B.costNew <
                  Cinematique_ROS_B.sigma_j) {
                Cinematique_ROS_B.flag = (Cinematique_ROS_B.b_gamma <
                  obj->StepTolerance);
                if (Cinematique_ROS_B.flag) {
                  xSol[0] = Cinematique_ROS_B.x[0];
                  xSol[1] = Cinematique_ROS_B.x[1];
                  xSol[2] = Cinematique_ROS_B.x[2];
                  xSol[3] = Cinematique_ROS_B.x[3];
                  *exitFlag = StepSizeBelowMinimum;
                  *err = Cine_IKHelpers_evaluateSolution(obj->ExtraArgs);
                  *iter = static_cast<real_T>(Cinematique_ROS_B.j_l) + 1.0;
                  exitg1 = 1;
                } else {
                  Cinematique_ROS_B.b_gamma *= Cinematique_ROS_B.beta;
                  Cinematique_ROS_B.m_f++;
                  Cinematique_ROS_B.sNew[0] = Cinematique_ROS_B.b_gamma *
                    Cinematique_ROS_B.Hg[0] + Cinematique_ROS_B.x[0];
                  Cinematique_ROS_B.sNew[1] = Cinematique_ROS_B.b_gamma *
                    Cinematique_ROS_B.Hg[1] + Cinematique_ROS_B.x[1];
                  Cinematique_ROS_B.sNew[2] = Cinematique_ROS_B.b_gamma *
                    Cinematique_ROS_B.Hg[2] + Cinematique_ROS_B.x[2];
                  Cinematique_ROS_B.sNew[3] = Cinematique_ROS_B.b_gamma *
                    Cinematique_ROS_B.theta + Cinematique_ROS_B.x[3];
                  Cinematiq_IKHelpers_computeCost(Cinematique_ROS_B.sNew,
                    obj->ExtraArgs, &Cinematique_ROS_B.costNew,
                    Cinematique_ROS_B.unusedU0, unusedU1, &d);
                  obj->ExtraArgs = d;
                }
              } else {
                xSol[0] += Cinematique_ROS_B.x[0];
                xSol[1] += Cinematique_ROS_B.x[1];
                xSol[2] += Cinematique_ROS_B.x[2];
                Cinematique_ROS_B.beta = Cinematique_ROS_B.x[3] + xSol[3];
                xSol[3] = Cinematique_ROS_B.beta;
                args = obj->ExtraArgs;
                Cinematique_ROS_B.nx_h = alpha->size[0];
                alpha->size[0] = args->GradTemp->size[0];
                Cinema_emxEnsureCapacity_real_T(alpha, Cinematique_ROS_B.nx_h);
                Cinematique_ROS_B.inner = args->GradTemp->size[0];
                for (Cinematique_ROS_B.nx_h = 0; Cinematique_ROS_B.nx_h <
                     Cinematique_ROS_B.inner; Cinematique_ROS_B.nx_h++) {
                  alpha->data[Cinematique_ROS_B.nx_h] = args->GradTemp->
                    data[Cinematique_ROS_B.nx_h];
                }

                exitg1 = 2;
              }
            } while (exitg1 == 0);

            if (exitg1 == 1) {
              exitg2 = 1;
            } else if ((Cinematique_ROS_B.m_f == 0.0) && (fabs
                        (Cinematique_ROS_B.b_gamma - Cinematique_ROS_B.lambda) <
                        1.4901161193847656E-8)) {
              Cinematique_ROS_B.inner = obj->ConstraintMatrix->size[0];
              Cinematique_ROS_B.nx_h = grad->size[0];
              grad->size[0] = Cinematique_ROS_B.inner;
              Cinema_emxEnsureCapacity_real_T(grad, Cinematique_ROS_B.nx_h);
              for (Cinematique_ROS_B.nx_h = 0; Cinematique_ROS_B.nx_h <
                   Cinematique_ROS_B.inner; Cinematique_ROS_B.nx_h++) {
                grad->data[Cinematique_ROS_B.nx_h] = obj->ConstraintMatrix->
                  data[(Cinematique_ROS_B.idxl - 1) * obj->
                  ConstraintMatrix->size[0] + Cinematique_ROS_B.nx_h];
              }

              activeSet->data[Cinematique_ROS_B.idxl - 1] = true;
              Cinematique_ROS_B.idxl = activeSet->size[0] - 1;
              Cinematique_ROS_B.m_me = 0;
              for (Cinematique_ROS_B.nx_h = 0; Cinematique_ROS_B.nx_h <=
                   Cinematique_ROS_B.idxl; Cinematique_ROS_B.nx_h++) {
                if (activeSet->data[Cinematique_ROS_B.nx_h]) {
                  Cinematique_ROS_B.m_me++;
                }
              }

              Cinematique_ROS_B.nx_h = gb->size[0];
              gb->size[0] = Cinematique_ROS_B.m_me;
              Cinem_emxEnsureCapacity_int32_T(gb, Cinematique_ROS_B.nx_h);
              Cinematique_ROS_B.m_me = 0;
              for (Cinematique_ROS_B.nx_h = 0; Cinematique_ROS_B.nx_h <=
                   Cinematique_ROS_B.idxl; Cinematique_ROS_B.nx_h++) {
                if (activeSet->data[Cinematique_ROS_B.nx_h]) {
                  gb->data[Cinematique_ROS_B.m_me] = Cinematique_ROS_B.nx_h + 1;
                  Cinematique_ROS_B.m_me++;
                }
              }

              Cinematique_ROS_B.inner = obj->ConstraintMatrix->size[0];
              Cinematique_ROS_B.nx_h = A->size[0] * A->size[1];
              A->size[0] = Cinematique_ROS_B.inner;
              A->size[1] = gb->size[0];
              Cinema_emxEnsureCapacity_real_T(A, Cinematique_ROS_B.nx_h);
              Cinematique_ROS_B.aoffset = gb->size[0];
              for (Cinematique_ROS_B.nx_h = 0; Cinematique_ROS_B.nx_h <
                   Cinematique_ROS_B.aoffset; Cinematique_ROS_B.nx_h++) {
                for (Cinematique_ROS_B.m_me = 0; Cinematique_ROS_B.m_me <
                     Cinematique_ROS_B.inner; Cinematique_ROS_B.m_me++) {
                  A->data[Cinematique_ROS_B.m_me + A->size[0] *
                    Cinematique_ROS_B.nx_h] = obj->ConstraintMatrix->data
                    [(gb->data[Cinematique_ROS_B.nx_h] - 1) *
                    obj->ConstraintMatrix->size[0] + Cinematique_ROS_B.m_me];
                }
              }

              Cinematique_ROS_B.nx_h = grad_1->size[0] * grad_1->size[1];
              grad_1->size[0] = 1;
              grad_1->size[1] = grad->size[0];
              Cinema_emxEnsureCapacity_real_T(grad_1, Cinematique_ROS_B.nx_h);
              Cinematique_ROS_B.inner = grad->size[0];
              for (Cinematique_ROS_B.nx_h = 0; Cinematique_ROS_B.nx_h <
                   Cinematique_ROS_B.inner; Cinematique_ROS_B.nx_h++) {
                grad_1->data[Cinematique_ROS_B.nx_h] = grad->
                  data[Cinematique_ROS_B.nx_h];
              }

              Cinematique_ROS_B.b_gamma = 0.0;
              for (Cinematique_ROS_B.nx_h = 0; Cinematique_ROS_B.nx_h < 4;
                   Cinematique_ROS_B.nx_h++) {
                Cinematique_ROS_B.idxl = Cinematique_ROS_B.nx_h << 2;
                Cinematique_ROS_B.m_f =
                  Cinematique_ROS_B.H[Cinematique_ROS_B.idxl] * grad_1->data[0];
                Cinematique_ROS_B.m_f +=
                  Cinematique_ROS_B.H[Cinematique_ROS_B.idxl + 1] * grad_1->
                  data[1];
                Cinematique_ROS_B.m_f +=
                  Cinematique_ROS_B.H[Cinematique_ROS_B.idxl + 2] * grad_1->
                  data[2];
                Cinematique_ROS_B.m_f +=
                  Cinematique_ROS_B.H[Cinematique_ROS_B.idxl + 3] * grad_1->
                  data[3];
                Cinematique_ROS_B.b_gamma += Cinematique_ROS_B.m_f * grad->
                  data[Cinematique_ROS_B.nx_h];
              }

              Cinematique_ROS_B.theta = 1.0 / Cinematique_ROS_B.b_gamma;
              Cinematique_ROS_B.nx_h = grad_2->size[0] * grad_2->size[1];
              grad_2->size[0] = grad->size[0];
              grad_2->size[1] = grad->size[0];
              Cinema_emxEnsureCapacity_real_T(grad_2, Cinematique_ROS_B.nx_h);
              Cinematique_ROS_B.inner = grad->size[0];
              for (Cinematique_ROS_B.nx_h = 0; Cinematique_ROS_B.nx_h <
                   Cinematique_ROS_B.inner; Cinematique_ROS_B.nx_h++) {
                Cinematique_ROS_B.aoffset = grad->size[0];
                for (Cinematique_ROS_B.m_me = 0; Cinematique_ROS_B.m_me <
                     Cinematique_ROS_B.aoffset; Cinematique_ROS_B.m_me++) {
                  grad_2->data[Cinematique_ROS_B.m_me + grad_2->size[0] *
                    Cinematique_ROS_B.nx_h] = grad->data[Cinematique_ROS_B.m_me]
                    * grad->data[Cinematique_ROS_B.nx_h];
                }
              }

              Cinematique_ROS_mtimes_k1e1w(grad_2, Cinematique_ROS_B.H, tmp_3);
              for (Cinematique_ROS_B.nx_h = 0; Cinematique_ROS_B.nx_h < 4;
                   Cinematique_ROS_B.nx_h++) {
                for (Cinematique_ROS_B.m_me = 0; Cinematique_ROS_B.m_me < 4;
                     Cinematique_ROS_B.m_me++) {
                  Cinematique_ROS_B.idxl = Cinematique_ROS_B.m_me << 2;
                  Cinematique_ROS_B.inner = Cinematique_ROS_B.nx_h +
                    Cinematique_ROS_B.idxl;
                  Cinematique_ROS_B.V[Cinematique_ROS_B.inner] = 0.0;
                  Cinematique_ROS_B.V[Cinematique_ROS_B.inner] += tmp_3->
                    data[Cinematique_ROS_B.idxl] *
                    Cinematique_ROS_B.H[Cinematique_ROS_B.nx_h];
                  Cinematique_ROS_B.V[Cinematique_ROS_B.inner] += tmp_3->
                    data[Cinematique_ROS_B.idxl + 1] *
                    Cinematique_ROS_B.H[Cinematique_ROS_B.nx_h + 4];
                  Cinematique_ROS_B.V[Cinematique_ROS_B.inner] += tmp_3->
                    data[Cinematique_ROS_B.idxl + 2] *
                    Cinematique_ROS_B.H[Cinematique_ROS_B.nx_h + 8];
                  Cinematique_ROS_B.V[Cinematique_ROS_B.inner] += tmp_3->
                    data[Cinematique_ROS_B.idxl + 3] *
                    Cinematique_ROS_B.H[Cinematique_ROS_B.nx_h + 12];
                }
              }

              for (Cinematique_ROS_B.nx_h = 0; Cinematique_ROS_B.nx_h < 16;
                   Cinematique_ROS_B.nx_h++) {
                Cinematique_ROS_B.H[Cinematique_ROS_B.nx_h] -=
                  Cinematique_ROS_B.theta *
                  Cinematique_ROS_B.V[Cinematique_ROS_B.nx_h];
              }

              guard2 = true;
            } else {
              Cinematique_ROS_B.nx_h = grad->size[0];
              grad->size[0] = alpha->size[0];
              Cinema_emxEnsureCapacity_real_T(grad, Cinematique_ROS_B.nx_h);
              Cinematique_ROS_B.inner = alpha->size[0];
              for (Cinematique_ROS_B.nx_h = 0; Cinematique_ROS_B.nx_h <
                   Cinematique_ROS_B.inner; Cinematique_ROS_B.nx_h++) {
                grad->data[Cinematique_ROS_B.nx_h] = alpha->
                  data[Cinematique_ROS_B.nx_h] - grad->
                  data[Cinematique_ROS_B.nx_h];
              }

              Cinematique_ROS_B.b_gamma = Cinematique_ROS_B.Hg[0] * grad->data[0];
              Cinematique_ROS_B.b_gamma += Cinematique_ROS_B.Hg[1] * grad->data
                [1];
              Cinematique_ROS_B.b_gamma += Cinematique_ROS_B.Hg[2] * grad->data
                [2];
              Cinematique_ROS_B.b_gamma += Cinematique_ROS_B.theta * grad->data
                [3];
              Cinematique_ROS_B.nx_h = tmp_4->size[0] * tmp_4->size[1];
              tmp_4->size[0] = 1;
              tmp_4->size[1] = grad->size[0];
              Cinema_emxEnsureCapacity_real_T(tmp_4, Cinematique_ROS_B.nx_h);
              Cinematique_ROS_B.inner = grad->size[0];
              for (Cinematique_ROS_B.nx_h = 0; Cinematique_ROS_B.nx_h <
                   Cinematique_ROS_B.inner; Cinematique_ROS_B.nx_h++) {
                tmp_4->data[Cinematique_ROS_B.nx_h] = 0.2 * grad->
                  data[Cinematique_ROS_B.nx_h];
              }

              Cinematique_ROS_B.lambda = 0.0;
              for (Cinematique_ROS_B.nx_h = 0; Cinematique_ROS_B.nx_h < 4;
                   Cinematique_ROS_B.nx_h++) {
                Cinematique_ROS_B.m_me = Cinematique_ROS_B.nx_h << 2;
                Cinematique_ROS_B.theta =
                  Cinematique_ROS_B.H[Cinematique_ROS_B.m_me] * tmp_4->data[0];
                Cinematique_ROS_B.theta +=
                  Cinematique_ROS_B.H[Cinematique_ROS_B.m_me + 1] * tmp_4->data
                  [1];
                Cinematique_ROS_B.theta +=
                  Cinematique_ROS_B.H[Cinematique_ROS_B.m_me + 2] * tmp_4->data
                  [2];
                Cinematique_ROS_B.theta +=
                  Cinematique_ROS_B.H[Cinematique_ROS_B.m_me + 3] * tmp_4->data
                  [3];
                Cinematique_ROS_B.lambda += Cinematique_ROS_B.theta * grad->
                  data[Cinematique_ROS_B.nx_h];
              }

              if (Cinematique_ROS_B.b_gamma < Cinematique_ROS_B.lambda) {
                Cinematique_ROS_B.nx_h = tmp_5->size[0] * tmp_5->size[1];
                tmp_5->size[0] = 1;
                tmp_5->size[1] = grad->size[0];
                Cinema_emxEnsureCapacity_real_T(tmp_5, Cinematique_ROS_B.nx_h);
                Cinematique_ROS_B.inner = grad->size[0];
                for (Cinematique_ROS_B.nx_h = 0; Cinematique_ROS_B.nx_h <
                     Cinematique_ROS_B.inner; Cinematique_ROS_B.nx_h++) {
                  tmp_5->data[Cinematique_ROS_B.nx_h] = 0.8 * grad->
                    data[Cinematique_ROS_B.nx_h];
                }

                Cinematique_ROS_B.lambda = 0.0;
                for (Cinematique_ROS_B.nx_h = 0; Cinematique_ROS_B.nx_h < 4;
                     Cinematique_ROS_B.nx_h++) {
                  Cinematique_ROS_B.m_me = Cinematique_ROS_B.nx_h << 2;
                  Cinematique_ROS_B.theta =
                    Cinematique_ROS_B.H[Cinematique_ROS_B.m_me] * tmp_5->data[0];
                  Cinematique_ROS_B.theta +=
                    Cinematique_ROS_B.H[Cinematique_ROS_B.m_me + 1] *
                    tmp_5->data[1];
                  Cinematique_ROS_B.theta +=
                    Cinematique_ROS_B.H[Cinematique_ROS_B.m_me + 2] *
                    tmp_5->data[2];
                  Cinematique_ROS_B.theta +=
                    Cinematique_ROS_B.H[Cinematique_ROS_B.m_me + 3] *
                    tmp_5->data[3];
                  Cinematique_ROS_B.lambda += Cinematique_ROS_B.theta *
                    grad->data[Cinematique_ROS_B.nx_h];
                }

                Cinematique_ROS_B.nx_h = grad_0->size[0] * grad_0->size[1];
                grad_0->size[0] = 1;
                grad_0->size[1] = grad->size[0];
                Cinema_emxEnsureCapacity_real_T(grad_0, Cinematique_ROS_B.nx_h);
                Cinematique_ROS_B.inner = grad->size[0];
                for (Cinematique_ROS_B.nx_h = 0; Cinematique_ROS_B.nx_h <
                     Cinematique_ROS_B.inner; Cinematique_ROS_B.nx_h++) {
                  grad_0->data[Cinematique_ROS_B.nx_h] = grad->
                    data[Cinematique_ROS_B.nx_h];
                }

                Cinematique_ROS_B.b_gamma = 0.0;
                Cinematique_ROS_B.theta = 0.0;
                for (Cinematique_ROS_B.nx_h = 0; Cinematique_ROS_B.nx_h < 4;
                     Cinematique_ROS_B.nx_h++) {
                  Cinematique_ROS_B.idxl = Cinematique_ROS_B.nx_h << 2;
                  Cinematique_ROS_B.m_f =
                    Cinematique_ROS_B.H[Cinematique_ROS_B.idxl] * grad_0->data[0];
                  Cinematique_ROS_B.m_f +=
                    Cinematique_ROS_B.H[Cinematique_ROS_B.idxl + 1] *
                    grad_0->data[1];
                  Cinematique_ROS_B.m_f +=
                    Cinematique_ROS_B.H[Cinematique_ROS_B.idxl + 2] *
                    grad_0->data[2];
                  Cinematique_ROS_B.m_f +=
                    Cinematique_ROS_B.H[Cinematique_ROS_B.idxl + 3] *
                    grad_0->data[3];
                  Cinematique_ROS_B.b_gamma += Cinematique_ROS_B.m_f *
                    grad->data[Cinematique_ROS_B.nx_h];
                  Cinematique_ROS_B.theta +=
                    Cinematique_ROS_B.Hg[Cinematique_ROS_B.nx_h] * grad->
                    data[Cinematique_ROS_B.nx_h];
                }

                Cinematique_ROS_B.theta = Cinematique_ROS_B.lambda /
                  (Cinematique_ROS_B.b_gamma - Cinematique_ROS_B.theta);
              } else {
                Cinematique_ROS_B.theta = 1.0;
              }

              Cinematique_ROS_B.b_gamma = 0.0;
              for (Cinematique_ROS_B.nx_h = 0; Cinematique_ROS_B.nx_h < 4;
                   Cinematique_ROS_B.nx_h++) {
                Cinematique_ROS_B.lambda = (1.0 - Cinematique_ROS_B.theta) *
                  Cinematique_ROS_B.H[Cinematique_ROS_B.nx_h] * grad->data[0];
                Cinematique_ROS_B.lambda += (1.0 - Cinematique_ROS_B.theta) *
                  Cinematique_ROS_B.H[Cinematique_ROS_B.nx_h + 4] * grad->data[1];
                Cinematique_ROS_B.lambda += (1.0 - Cinematique_ROS_B.theta) *
                  Cinematique_ROS_B.H[Cinematique_ROS_B.nx_h + 8] * grad->data[2];
                Cinematique_ROS_B.lambda += (1.0 - Cinematique_ROS_B.theta) *
                  Cinematique_ROS_B.H[Cinematique_ROS_B.nx_h + 12] * grad->data
                  [3];
                Cinematique_ROS_B.lambda += Cinematique_ROS_B.theta *
                  Cinematique_ROS_B.Hg[Cinematique_ROS_B.nx_h];
                Cinematique_ROS_B.b_gamma += Cinematique_ROS_B.lambda *
                  grad->data[Cinematique_ROS_B.nx_h];
                Cinematique_ROS_B.sNew[Cinematique_ROS_B.nx_h] =
                  Cinematique_ROS_B.lambda;
              }

              memset(&Cinematique_ROS_B.P[0], 0, sizeof(real_T) << 4U);
              Cinematique_ROS_B.P[0] = 1.0;
              Cinematique_ROS_B.P[5] = 1.0;
              Cinematique_ROS_B.P[10] = 1.0;
              Cinematique_ROS_B.P[15] = 1.0;
              Cinematique_ROS_B.nx_h = sNew->size[0] * sNew->size[1];
              sNew->size[0] = 4;
              sNew->size[1] = grad->size[0];
              Cinema_emxEnsureCapacity_real_T(sNew, Cinematique_ROS_B.nx_h);
              Cinematique_ROS_B.inner = grad->size[0];
              for (Cinematique_ROS_B.nx_h = 0; Cinematique_ROS_B.nx_h <
                   Cinematique_ROS_B.inner; Cinematique_ROS_B.nx_h++) {
                Cinematique_ROS_B.lambda = Cinematique_ROS_B.sNew[0] *
                  grad->data[Cinematique_ROS_B.nx_h];
                Cinematique_ROS_B.m_me = Cinematique_ROS_B.nx_h << 2;
                sNew->data[Cinematique_ROS_B.m_me] = Cinematique_ROS_B.lambda /
                  Cinematique_ROS_B.b_gamma;
                Cinematique_ROS_B.lambda = Cinematique_ROS_B.sNew[1] *
                  grad->data[Cinematique_ROS_B.nx_h];
                sNew->data[Cinematique_ROS_B.m_me + 1] =
                  Cinematique_ROS_B.lambda / Cinematique_ROS_B.b_gamma;
                Cinematique_ROS_B.lambda = Cinematique_ROS_B.sNew[2] *
                  grad->data[Cinematique_ROS_B.nx_h];
                sNew->data[Cinematique_ROS_B.m_me + 2] =
                  Cinematique_ROS_B.lambda / Cinematique_ROS_B.b_gamma;
                Cinematique_ROS_B.lambda = Cinematique_ROS_B.sNew[3] *
                  grad->data[Cinematique_ROS_B.nx_h];
                sNew->data[Cinematique_ROS_B.m_me + 3] =
                  Cinematique_ROS_B.lambda / Cinematique_ROS_B.b_gamma;
              }

              for (Cinematique_ROS_B.nx_h = 0; Cinematique_ROS_B.nx_h < 16;
                   Cinematique_ROS_B.nx_h++) {
                Cinematique_ROS_B.V[Cinematique_ROS_B.nx_h] =
                  Cinematique_ROS_B.P[Cinematique_ROS_B.nx_h] - sNew->
                  data[Cinematique_ROS_B.nx_h];
              }

              for (Cinematique_ROS_B.nx_h = 0; Cinematique_ROS_B.nx_h < 4;
                   Cinematique_ROS_B.nx_h++) {
                for (Cinematique_ROS_B.m_me = 0; Cinematique_ROS_B.m_me < 4;
                     Cinematique_ROS_B.m_me++) {
                  Cinematique_ROS_B.idxl = Cinematique_ROS_B.m_me << 2;
                  Cinematique_ROS_B.inner = Cinematique_ROS_B.nx_h +
                    Cinematique_ROS_B.idxl;
                  Cinematique_ROS_B.V_m[Cinematique_ROS_B.inner] = 0.0;
                  Cinematique_ROS_B.V_m[Cinematique_ROS_B.inner] +=
                    Cinematique_ROS_B.H[Cinematique_ROS_B.idxl] *
                    Cinematique_ROS_B.V[Cinematique_ROS_B.nx_h];
                  Cinematique_ROS_B.V_m[Cinematique_ROS_B.inner] +=
                    Cinematique_ROS_B.H[Cinematique_ROS_B.idxl + 1] *
                    Cinematique_ROS_B.V[Cinematique_ROS_B.nx_h + 4];
                  Cinematique_ROS_B.V_m[Cinematique_ROS_B.inner] +=
                    Cinematique_ROS_B.H[Cinematique_ROS_B.idxl + 2] *
                    Cinematique_ROS_B.V[Cinematique_ROS_B.nx_h + 8];
                  Cinematique_ROS_B.V_m[Cinematique_ROS_B.inner] +=
                    Cinematique_ROS_B.H[Cinematique_ROS_B.idxl + 3] *
                    Cinematique_ROS_B.V[Cinematique_ROS_B.nx_h + 12];
                }

                for (Cinematique_ROS_B.m_me = 0; Cinematique_ROS_B.m_me < 4;
                     Cinematique_ROS_B.m_me++) {
                  Cinematique_ROS_B.inner = Cinematique_ROS_B.nx_h +
                    (Cinematique_ROS_B.m_me << 2);
                  Cinematique_ROS_B.P[Cinematique_ROS_B.inner] = 0.0;
                  Cinematique_ROS_B.P[Cinematique_ROS_B.inner] +=
                    Cinematique_ROS_B.V_m[Cinematique_ROS_B.nx_h] *
                    Cinematique_ROS_B.V[Cinematique_ROS_B.m_me];
                  Cinematique_ROS_B.P[Cinematique_ROS_B.inner] +=
                    Cinematique_ROS_B.V_m[Cinematique_ROS_B.nx_h + 4] *
                    Cinematique_ROS_B.V[Cinematique_ROS_B.m_me + 4];
                  Cinematique_ROS_B.P[Cinematique_ROS_B.inner] +=
                    Cinematique_ROS_B.V_m[Cinematique_ROS_B.nx_h + 8] *
                    Cinematique_ROS_B.V[Cinematique_ROS_B.m_me + 8];
                  Cinematique_ROS_B.P[Cinematique_ROS_B.inner] +=
                    Cinematique_ROS_B.V_m[Cinematique_ROS_B.nx_h + 12] *
                    Cinematique_ROS_B.V[Cinematique_ROS_B.m_me + 12];
                }
              }

              for (Cinematique_ROS_B.nx_h = 0; Cinematique_ROS_B.nx_h < 4;
                   Cinematique_ROS_B.nx_h++) {
                Cinematique_ROS_B.idxl = Cinematique_ROS_B.nx_h << 2;
                Cinematique_ROS_B.V_m[Cinematique_ROS_B.idxl] =
                  Cinematique_ROS_B.sNew[0] *
                  Cinematique_ROS_B.sNew[Cinematique_ROS_B.nx_h] /
                  Cinematique_ROS_B.b_gamma;
                Cinematique_ROS_B.V_m[Cinematique_ROS_B.idxl + 1] =
                  Cinematique_ROS_B.sNew[1] *
                  Cinematique_ROS_B.sNew[Cinematique_ROS_B.nx_h] /
                  Cinematique_ROS_B.b_gamma;
                Cinematique_ROS_B.V_m[Cinematique_ROS_B.idxl + 2] =
                  Cinematique_ROS_B.sNew[2] *
                  Cinematique_ROS_B.sNew[Cinematique_ROS_B.nx_h] /
                  Cinematique_ROS_B.b_gamma;
                Cinematique_ROS_B.V_m[Cinematique_ROS_B.idxl + 3] =
                  Cinematique_ROS_B.sNew[3] *
                  Cinematique_ROS_B.sNew[Cinematique_ROS_B.nx_h] /
                  Cinematique_ROS_B.b_gamma;
              }

              for (Cinematique_ROS_B.nx_h = 0; Cinematique_ROS_B.nx_h < 16;
                   Cinematique_ROS_B.nx_h++) {
                Cinematique_ROS_B.theta =
                  Cinematique_ROS_B.P[Cinematique_ROS_B.nx_h] +
                  Cinematique_ROS_B.V_m[Cinematique_ROS_B.nx_h];
                Cinematique_ROS_B.V[Cinematique_ROS_B.nx_h] =
                  1.4901161193847656E-8 * static_cast<real_T>
                  (tmp_6[Cinematique_ROS_B.nx_h]) + Cinematique_ROS_B.theta;
                Cinematique_ROS_B.H[Cinematique_ROS_B.nx_h] =
                  Cinematique_ROS_B.theta;
              }

              if (!Cinematique__isPositiveDefinite(Cinematique_ROS_B.V)) {
                *exitFlag = HessianNotPositiveSemidefinite;
                *err = Cine_IKHelpers_evaluateSolution(obj->ExtraArgs);
                *iter = static_cast<real_T>(Cinematique_ROS_B.j_l) + 1.0;
                exitg2 = 1;
              } else {
                guard2 = true;
              }
            }
          }

          if (guard2) {
            if (obj->ConstraintsOn) {
              Cinematique_ROS_B.nx_h = AIn->size[0] * AIn->size[1];
              AIn->size[0] = obj->ConstraintMatrix->size[0];
              AIn->size[1] = obj->ConstraintMatrix->size[1];
              Cinema_emxEnsureCapacity_real_T(AIn, Cinematique_ROS_B.nx_h);
              Cinematique_ROS_B.inner = obj->ConstraintMatrix->size[0] *
                obj->ConstraintMatrix->size[1] - 1;
              for (Cinematique_ROS_B.nx_h = 0; Cinematique_ROS_B.nx_h <=
                   Cinematique_ROS_B.inner; Cinematique_ROS_B.nx_h++) {
                AIn->data[Cinematique_ROS_B.nx_h] = obj->ConstraintMatrix->
                  data[Cinematique_ROS_B.nx_h];
              }

              Cinematique_ROS_mtimes_k(AIn, xSol, L);
              Cinematique_ROS_B.nx_h = L_0->size[0];
              L_0->size[0] = L->size[0];
              Cin_emxEnsureCapacity_boolean_T(L_0, Cinematique_ROS_B.nx_h);
              Cinematique_ROS_B.inner = L->size[0];
              for (Cinematique_ROS_B.nx_h = 0; Cinematique_ROS_B.nx_h <
                   Cinematique_ROS_B.inner; Cinematique_ROS_B.nx_h++) {
                L_0->data[Cinematique_ROS_B.nx_h] = (L->
                  data[Cinematique_ROS_B.nx_h] - obj->ConstraintBound->
                  data[Cinematique_ROS_B.nx_h] > 1.4901161193847656E-8);
              }

              if (Cinematique_ROS_any(L_0)) {
                xSol[0] = Cinematique_ROS_B.x[0];
                xSol[1] = Cinematique_ROS_B.x[1];
                xSol[2] = Cinematique_ROS_B.x[2];
                xSol[3] = Cinematique_ROS_B.x[3];
                *exitFlag = SearchDirectionInvalid;
                *err = Cine_IKHelpers_evaluateSolution(obj->ExtraArgs);
                *iter = static_cast<real_T>(Cinematique_ROS_B.j_l) + 1.0;
                exitg2 = 1;
              } else {
                guard1 = true;
              }
            } else {
              guard1 = true;
            }
          }

          if (guard1) {
            Cinematique_ROS_B.x[0] = xSol[0];
            Cinematique_ROS_B.x[1] = xSol[1];
            Cinematique_ROS_B.x[2] = xSol[2];
            Cinematique_ROS_B.x[3] = Cinematique_ROS_B.beta;
            Cinematique_ROS_B.nx_h = grad->size[0];
            grad->size[0] = alpha->size[0];
            Cinema_emxEnsureCapacity_real_T(grad, Cinematique_ROS_B.nx_h);
            Cinematique_ROS_B.inner = alpha->size[0];
            for (Cinematique_ROS_B.nx_h = 0; Cinematique_ROS_B.nx_h <
                 Cinematique_ROS_B.inner; Cinematique_ROS_B.nx_h++) {
              grad->data[Cinematique_ROS_B.nx_h] = alpha->
                data[Cinematique_ROS_B.nx_h];
            }

            Cinematique_ROS_B.cost = Cinematique_ROS_B.costNew;
            Cinematique_ROS_B.j_l++;
          }
        }
      }
    } else {
      *exitFlag = IterationLimitExceeded;
      *err = Cine_IKHelpers_evaluateSolution(obj->ExtraArgs);
      *iter = obj->MaxNumIterationInternal;
      exitg2 = 1;
    }
  } while (exitg2 == 0);

  Cinematique_R_emxFree_boolean_T(&L_0);
  Cinematique_ROS_emxFree_real_T(&alpha_4);
  Cinematique_ROS_emxFree_real_T(&grad_2);
  Cinematique_ROS_emxFree_real_T(&grad_1);
  Cinematique_ROS_emxFree_real_T(&sNew);
  Cinematique_ROS_emxFree_real_T(&alpha_3);
  Cinematique_ROS_emxFree_real_T(&alpha_2);
  Cinematique_R_emxFree_boolean_T(&alpha_1);
  Cinematique_ROS_emxFree_real_T(&A_4);
  Cinematique_ROS_emxFree_real_T(&grad_0);
  Cinematique_ROS_emxFree_real_T(&tmp_5);
  Cinematique_ROS_emxFree_real_T(&tmp_4);
  Cinematique_ROS_emxFree_real_T(&sigma);
  Cinematique_ROS_emxFree_real_T(&alpha_0);
  Cinematique_ROS_emxFree_real_T(&A_3);
  Cinematique_ROS_emxFree_real_T(&tmp_3);
  Cinematique_ROS_emxFree_real_T(&tmp_2);
  Cinematique_ROS_emxFree_real_T(&tmp_1);
  Cinematique_ROS_emxFree_real_T(&tmp_0);
  Cinematique_R_emxFree_boolean_T(&tmp);
  Cinematique_ROS_emxFree_int32_T(&gb);
  Cinematique_ROS_emxFree_int32_T(&fb);
  Cinematique_ROS_emxFree_int32_T(&eb);
  Cinematique_ROS_emxFree_int32_T(&db);
  Cinematique_ROS_emxFree_int32_T(&cb);
  Cinematique_ROS_emxFree_int32_T(&bb);
  Cinematique_ROS_emxFree_real_T(&L);
  Cinematique_ROS_emxFree_real_T(&AIn);
  Cinematique_ROS_emxFree_real_T(&alpha);
  Cinematique_ROS_emxFree_real_T(&A);
  Cinematique_R_emxFree_boolean_T(&activeSet);
  Cinematique_ROS_emxFree_real_T(&grad);
  Cinematique_ROS_emxFree_real_T(&unusedU1);
}

static real_T Cinematique_ROS_genrandu_k(uint32_T mt[625])
{
  real_T r;
  int32_T exitg1;

  /* ========================= COPYRIGHT NOTICE ============================ */
  /*  This is a uniform (0,1) pseudorandom number generator based on:        */
  /*                                                                         */
  /*  A C-program for MT19937, with initialization improved 2002/1/26.       */
  /*  Coded by Takuji Nishimura and Makoto Matsumoto.                        */
  /*                                                                         */
  /*  Copyright (C) 1997 - 2002, Makoto Matsumoto and Takuji Nishimura,      */
  /*  All rights reserved.                                                   */
  /*                                                                         */
  /*  Redistribution and use in source and binary forms, with or without     */
  /*  modification, are permitted provided that the following conditions     */
  /*  are met:                                                               */
  /*                                                                         */
  /*    1. Redistributions of source code must retain the above copyright    */
  /*       notice, this list of conditions and the following disclaimer.     */
  /*                                                                         */
  /*    2. Redistributions in binary form must reproduce the above copyright */
  /*       notice, this list of conditions and the following disclaimer      */
  /*       in the documentation and/or other materials provided with the     */
  /*       distribution.                                                     */
  /*                                                                         */
  /*    3. The names of its contributors may not be used to endorse or       */
  /*       promote products derived from this software without specific      */
  /*       prior written permission.                                         */
  /*                                                                         */
  /*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS    */
  /*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT      */
  /*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR  */
  /*  A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT  */
  /*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,  */
  /*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT       */
  /*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,  */
  /*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY  */
  /*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT    */
  /*  (INCLUDING  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE */
  /*  OF THIS  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.  */
  /*                                                                         */
  /* =============================   END   ================================= */
  do {
    exitg1 = 0;
    Cinemat_genrand_uint32_vector_k(mt, Cinematique_ROS_B.b_u_i);
    r = (static_cast<real_T>(Cinematique_ROS_B.b_u_i[0] >> 5U) * 6.7108864E+7 +
         static_cast<real_T>(Cinematique_ROS_B.b_u_i[1] >> 6U)) *
      1.1102230246251565E-16;
    if (r == 0.0) {
      if (!Cinematique_ROS_is_valid_state(mt)) {
        Cinematique_ROS_B.r_p = 5489U;
        mt[0] = 5489U;
        for (Cinematique_ROS_B.b_mti_n = 0; Cinematique_ROS_B.b_mti_n < 623;
             Cinematique_ROS_B.b_mti_n++) {
          Cinematique_ROS_B.r_p = ((Cinematique_ROS_B.r_p >> 30U ^
            Cinematique_ROS_B.r_p) * 1812433253U + Cinematique_ROS_B.b_mti_n) +
            1U;
          mt[Cinematique_ROS_B.b_mti_n + 1] = Cinematique_ROS_B.r_p;
        }

        mt[624] = 624U;
      }
    } else {
      exitg1 = 1;
    }
  } while (exitg1 == 0);

  return r;
}

static real_T Cinematiq_eml_rand_mt19937ar_k1(uint32_T state[625])
{
  static const real_T tmp[257] = { 1.0, 0.977101701267673, 0.959879091800108,
    0.9451989534423, 0.932060075959231, 0.919991505039348, 0.908726440052131,
    0.898095921898344, 0.887984660755834, 0.878309655808918, 0.869008688036857,
    0.860033621196332, 0.851346258458678, 0.842915653112205, 0.834716292986884,
    0.826726833946222, 0.818929191603703, 0.811307874312656, 0.803849483170964,
    0.796542330422959, 0.789376143566025, 0.782341832654803, 0.775431304981187,
    0.768637315798486, 0.761953346836795, 0.755373506507096, 0.748892447219157,
    0.742505296340151, 0.736207598126863, 0.729995264561476, 0.72386453346863,
    0.717811932630722, 0.711834248878248, 0.705928501332754, 0.700091918136512,
    0.694321916126117, 0.688616083004672, 0.682972161644995, 0.677388036218774,
    0.671861719897082, 0.66639134390875, 0.660975147776663, 0.655611470579697,
    0.650298743110817, 0.645035480820822, 0.639820277453057, 0.634651799287624,
    0.629528779924837, 0.624450015547027, 0.619414360605834, 0.614420723888914,
    0.609468064925773, 0.604555390697468, 0.599681752619125, 0.594846243767987,
    0.590047996332826, 0.585286179263371, 0.580559996100791, 0.575868682972354,
    0.571211506735253, 0.566587763256165, 0.561996775814525, 0.557437893618766,
    0.552910490425833, 0.548413963255266, 0.543947731190026, 0.539511234256952,
    0.535103932380458, 0.530725304403662, 0.526374847171684, 0.522052074672322,
    0.517756517229756, 0.513487720747327, 0.509245245995748, 0.505028667943468,
    0.500837575126149, 0.49667156905249, 0.492530263643869, 0.488413284705458,
    0.484320269426683, 0.480250865909047, 0.476204732719506, 0.47218153846773,
    0.468180961405694, 0.464202689048174, 0.460246417812843, 0.456311852678716,
    0.452398706861849, 0.448506701507203, 0.444635565395739, 0.440785034665804,
    0.436954852547985, 0.433144769112652, 0.429354541029442, 0.425583931338022,
    0.421832709229496, 0.418100649837848, 0.414387534040891, 0.410693148270188,
    0.407017284329473, 0.403359739221114, 0.399720314980197, 0.396098818515832,
    0.392495061459315, 0.388908860018789, 0.385340034840077, 0.381788410873393,
    0.378253817245619, 0.374736087137891, 0.371235057668239, 0.367750569779032,
    0.364282468129004, 0.360830600989648, 0.357394820145781, 0.353974980800077,
    0.350570941481406, 0.347182563956794, 0.343809713146851, 0.340452257044522,
    0.337110066637006, 0.333783015830718, 0.330470981379163, 0.327173842813601,
    0.323891482376391, 0.320623784956905, 0.317370638029914, 0.314131931596337,
    0.310907558126286, 0.307697412504292, 0.30450139197665, 0.301319396100803,
    0.298151326696685, 0.294997087799962, 0.291856585617095, 0.288729728482183,
    0.285616426815502, 0.282516593083708, 0.279430141761638, 0.276356989295668,
    0.273297054068577, 0.270250256365875, 0.267216518343561, 0.264195763997261,
    0.261187919132721, 0.258192911337619, 0.255210669954662, 0.252241126055942,
    0.249284212418529, 0.246339863501264, 0.24340801542275, 0.240488605940501,
    0.237581574431238, 0.23468686187233, 0.231804410824339, 0.228934165414681,
    0.226076071322381, 0.223230075763918, 0.220396127480152, 0.217574176724331,
    0.214764175251174, 0.211966076307031, 0.209179834621125, 0.206405406397881,
    0.203642749310335, 0.200891822494657, 0.198152586545776, 0.195425003514135,
    0.192709036903589, 0.190004651670465, 0.187311814223801, 0.1846304924268,
    0.181960655599523, 0.179302274522848, 0.176655321443735, 0.174019770081839,
    0.171395595637506, 0.168782774801212, 0.166181285764482, 0.163591108232366,
    0.161012223437511, 0.158444614155925, 0.15588826472448, 0.153343161060263,
    0.150809290681846, 0.148286642732575, 0.145775208005994, 0.143274978973514,
    0.140785949814445, 0.138308116448551, 0.135841476571254, 0.133386029691669,
    0.130941777173644, 0.12850872228, 0.126086870220186, 0.123676228201597,
    0.12127680548479, 0.11888861344291, 0.116511665625611, 0.114145977827839,
    0.111791568163838, 0.109448457146812, 0.107116667774684, 0.104796225622487,
    0.102487158941935, 0.10018949876881, 0.0979032790388625, 0.095628536713009,
    0.093365311912691, 0.0911136480663738, 0.0888735920682759,
    0.0866451944505581, 0.0844285095703535, 0.082223595813203,
    0.0800305158146631, 0.0778493367020961, 0.0756801303589272,
    0.0735229737139814, 0.0713779490588905, 0.0692451443970068,
    0.0671246538277886, 0.065016577971243, 0.0629210244377582, 0.06083810834954,
    0.0587679529209339, 0.0567106901062031, 0.0546664613248891,
    0.0526354182767924, 0.0506177238609479, 0.0486135532158687,
    0.0466230949019305, 0.0446465522512946, 0.0426841449164746,
    0.0407361106559411, 0.0388027074045262, 0.0368842156885674,
    0.0349809414617162, 0.0330932194585786, 0.0312214171919203,
    0.0293659397581334, 0.0275272356696031, 0.0257058040085489,
    0.0239022033057959, 0.0221170627073089, 0.0203510962300445,
    0.0186051212757247, 0.0168800831525432, 0.0151770883079353,
    0.0134974506017399, 0.0118427578579079, 0.0102149714397015,
    0.00861658276939875, 0.00705087547137324, 0.00552240329925101,
    0.00403797259336304, 0.00260907274610216, 0.0012602859304986,
    0.000477467764609386 };

  const real_T *fitab;
  real_T r;
  int32_T exitg1;
  Cinematique_ROS_B.xi[0] = 0.0;
  Cinematique_ROS_B.xi[1] = 0.215241895984875;
  Cinematique_ROS_B.xi[2] = 0.286174591792068;
  Cinematique_ROS_B.xi[3] = 0.335737519214422;
  Cinematique_ROS_B.xi[4] = 0.375121332878378;
  Cinematique_ROS_B.xi[5] = 0.408389134611989;
  Cinematique_ROS_B.xi[6] = 0.43751840220787;
  Cinematique_ROS_B.xi[7] = 0.46363433679088;
  Cinematique_ROS_B.xi[8] = 0.487443966139235;
  Cinematique_ROS_B.xi[9] = 0.50942332960209;
  Cinematique_ROS_B.xi[10] = 0.529909720661557;
  Cinematique_ROS_B.xi[11] = 0.549151702327164;
  Cinematique_ROS_B.xi[12] = 0.567338257053817;
  Cinematique_ROS_B.xi[13] = 0.584616766106378;
  Cinematique_ROS_B.xi[14] = 0.601104617755991;
  Cinematique_ROS_B.xi[15] = 0.61689699000775;
  Cinematique_ROS_B.xi[16] = 0.63207223638606;
  Cinematique_ROS_B.xi[17] = 0.646695714894993;
  Cinematique_ROS_B.xi[18] = 0.660822574244419;
  Cinematique_ROS_B.xi[19] = 0.674499822837293;
  Cinematique_ROS_B.xi[20] = 0.687767892795788;
  Cinematique_ROS_B.xi[21] = 0.700661841106814;
  Cinematique_ROS_B.xi[22] = 0.713212285190975;
  Cinematique_ROS_B.xi[23] = 0.725446140909999;
  Cinematique_ROS_B.xi[24] = 0.737387211434295;
  Cinematique_ROS_B.xi[25] = 0.749056662017815;
  Cinematique_ROS_B.xi[26] = 0.760473406430107;
  Cinematique_ROS_B.xi[27] = 0.771654424224568;
  Cinematique_ROS_B.xi[28] = 0.782615023307232;
  Cinematique_ROS_B.xi[29] = 0.793369058840623;
  Cinematique_ROS_B.xi[30] = 0.80392911698997;
  Cinematique_ROS_B.xi[31] = 0.814306670135215;
  Cinematique_ROS_B.xi[32] = 0.824512208752291;
  Cinematique_ROS_B.xi[33] = 0.834555354086381;
  Cinematique_ROS_B.xi[34] = 0.844444954909153;
  Cinematique_ROS_B.xi[35] = 0.854189171008163;
  Cinematique_ROS_B.xi[36] = 0.863795545553308;
  Cinematique_ROS_B.xi[37] = 0.87327106808886;
  Cinematique_ROS_B.xi[38] = 0.882622229585165;
  Cinematique_ROS_B.xi[39] = 0.891855070732941;
  Cinematique_ROS_B.xi[40] = 0.900975224461221;
  Cinematique_ROS_B.xi[41] = 0.909987953496718;
  Cinematique_ROS_B.xi[42] = 0.91889818364959;
  Cinematique_ROS_B.xi[43] = 0.927710533401999;
  Cinematique_ROS_B.xi[44] = 0.936429340286575;
  Cinematique_ROS_B.xi[45] = 0.945058684468165;
  Cinematique_ROS_B.xi[46] = 0.953602409881086;
  Cinematique_ROS_B.xi[47] = 0.96206414322304;
  Cinematique_ROS_B.xi[48] = 0.970447311064224;
  Cinematique_ROS_B.xi[49] = 0.978755155294224;
  Cinematique_ROS_B.xi[50] = 0.986990747099062;
  Cinematique_ROS_B.xi[51] = 0.99515699963509;
  Cinematique_ROS_B.xi[52] = 1.00325667954467;
  Cinematique_ROS_B.xi[53] = 1.01129241744;
  Cinematique_ROS_B.xi[54] = 1.01926671746548;
  Cinematique_ROS_B.xi[55] = 1.02718196603564;
  Cinematique_ROS_B.xi[56] = 1.03504043983344;
  Cinematique_ROS_B.xi[57] = 1.04284431314415;
  Cinematique_ROS_B.xi[58] = 1.05059566459093;
  Cinematique_ROS_B.xi[59] = 1.05829648333067;
  Cinematique_ROS_B.xi[60] = 1.06594867476212;
  Cinematique_ROS_B.xi[61] = 1.07355406579244;
  Cinematique_ROS_B.xi[62] = 1.0811144097034;
  Cinematique_ROS_B.xi[63] = 1.08863139065398;
  Cinematique_ROS_B.xi[64] = 1.09610662785202;
  Cinematique_ROS_B.xi[65] = 1.10354167942464;
  Cinematique_ROS_B.xi[66] = 1.11093804601357;
  Cinematique_ROS_B.xi[67] = 1.11829717411934;
  Cinematique_ROS_B.xi[68] = 1.12562045921553;
  Cinematique_ROS_B.xi[69] = 1.13290924865253;
  Cinematique_ROS_B.xi[70] = 1.14016484436815;
  Cinematique_ROS_B.xi[71] = 1.14738850542085;
  Cinematique_ROS_B.xi[72] = 1.15458145035993;
  Cinematique_ROS_B.xi[73] = 1.16174485944561;
  Cinematique_ROS_B.xi[74] = 1.16887987673083;
  Cinematique_ROS_B.xi[75] = 1.17598761201545;
  Cinematique_ROS_B.xi[76] = 1.18306914268269;
  Cinematique_ROS_B.xi[77] = 1.19012551542669;
  Cinematique_ROS_B.xi[78] = 1.19715774787944;
  Cinematique_ROS_B.xi[79] = 1.20416683014438;
  Cinematique_ROS_B.xi[80] = 1.2111537262437;
  Cinematique_ROS_B.xi[81] = 1.21811937548548;
  Cinematique_ROS_B.xi[82] = 1.22506469375653;
  Cinematique_ROS_B.xi[83] = 1.23199057474614;
  Cinematique_ROS_B.xi[84] = 1.23889789110569;
  Cinematique_ROS_B.xi[85] = 1.24578749554863;
  Cinematique_ROS_B.xi[86] = 1.2526602218949;
  Cinematique_ROS_B.xi[87] = 1.25951688606371;
  Cinematique_ROS_B.xi[88] = 1.26635828701823;
  Cinematique_ROS_B.xi[89] = 1.27318520766536;
  Cinematique_ROS_B.xi[90] = 1.27999841571382;
  Cinematique_ROS_B.xi[91] = 1.28679866449324;
  Cinematique_ROS_B.xi[92] = 1.29358669373695;
  Cinematique_ROS_B.xi[93] = 1.30036323033084;
  Cinematique_ROS_B.xi[94] = 1.30712898903073;
  Cinematique_ROS_B.xi[95] = 1.31388467315022;
  Cinematique_ROS_B.xi[96] = 1.32063097522106;
  Cinematique_ROS_B.xi[97] = 1.32736857762793;
  Cinematique_ROS_B.xi[98] = 1.33409815321936;
  Cinematique_ROS_B.xi[99] = 1.3408203658964;
  Cinematique_ROS_B.xi[100] = 1.34753587118059;
  Cinematique_ROS_B.xi[101] = 1.35424531676263;
  Cinematique_ROS_B.xi[102] = 1.36094934303328;
  Cinematique_ROS_B.xi[103] = 1.36764858359748;
  Cinematique_ROS_B.xi[104] = 1.37434366577317;
  Cinematique_ROS_B.xi[105] = 1.38103521107586;
  Cinematique_ROS_B.xi[106] = 1.38772383568998;
  Cinematique_ROS_B.xi[107] = 1.39441015092814;
  Cinematique_ROS_B.xi[108] = 1.40109476367925;
  Cinematique_ROS_B.xi[109] = 1.4077782768464;
  Cinematique_ROS_B.xi[110] = 1.41446128977547;
  Cinematique_ROS_B.xi[111] = 1.42114439867531;
  Cinematique_ROS_B.xi[112] = 1.42782819703026;
  Cinematique_ROS_B.xi[113] = 1.43451327600589;
  Cinematique_ROS_B.xi[114] = 1.44120022484872;
  Cinematique_ROS_B.xi[115] = 1.44788963128058;
  Cinematique_ROS_B.xi[116] = 1.45458208188841;
  Cinematique_ROS_B.xi[117] = 1.46127816251028;
  Cinematique_ROS_B.xi[118] = 1.46797845861808;
  Cinematique_ROS_B.xi[119] = 1.47468355569786;
  Cinematique_ROS_B.xi[120] = 1.48139403962819;
  Cinematique_ROS_B.xi[121] = 1.48811049705745;
  Cinematique_ROS_B.xi[122] = 1.49483351578049;
  Cinematique_ROS_B.xi[123] = 1.50156368511546;
  Cinematique_ROS_B.xi[124] = 1.50830159628131;
  Cinematique_ROS_B.xi[125] = 1.51504784277671;
  Cinematique_ROS_B.xi[126] = 1.521803020761;
  Cinematique_ROS_B.xi[127] = 1.52856772943771;
  Cinematique_ROS_B.xi[128] = 1.53534257144151;
  Cinematique_ROS_B.xi[129] = 1.542128153229;
  Cinematique_ROS_B.xi[130] = 1.54892508547417;
  Cinematique_ROS_B.xi[131] = 1.55573398346918;
  Cinematique_ROS_B.xi[132] = 1.56255546753104;
  Cinematique_ROS_B.xi[133] = 1.56939016341512;
  Cinematique_ROS_B.xi[134] = 1.57623870273591;
  Cinematique_ROS_B.xi[135] = 1.58310172339603;
  Cinematique_ROS_B.xi[136] = 1.58997987002419;
  Cinematique_ROS_B.xi[137] = 1.59687379442279;
  Cinematique_ROS_B.xi[138] = 1.60378415602609;
  Cinematique_ROS_B.xi[139] = 1.61071162236983;
  Cinematique_ROS_B.xi[140] = 1.61765686957301;
  Cinematique_ROS_B.xi[141] = 1.62462058283303;
  Cinematique_ROS_B.xi[142] = 1.63160345693487;
  Cinematique_ROS_B.xi[143] = 1.63860619677555;
  Cinematique_ROS_B.xi[144] = 1.64562951790478;
  Cinematique_ROS_B.xi[145] = 1.65267414708306;
  Cinematique_ROS_B.xi[146] = 1.65974082285818;
  Cinematique_ROS_B.xi[147] = 1.66683029616166;
  Cinematique_ROS_B.xi[148] = 1.67394333092612;
  Cinematique_ROS_B.xi[149] = 1.68108070472517;
  Cinematique_ROS_B.xi[150] = 1.68824320943719;
  Cinematique_ROS_B.xi[151] = 1.69543165193456;
  Cinematique_ROS_B.xi[152] = 1.70264685479992;
  Cinematique_ROS_B.xi[153] = 1.7098896570713;
  Cinematique_ROS_B.xi[154] = 1.71716091501782;
  Cinematique_ROS_B.xi[155] = 1.72446150294804;
  Cinematique_ROS_B.xi[156] = 1.73179231405296;
  Cinematique_ROS_B.xi[157] = 1.73915426128591;
  Cinematique_ROS_B.xi[158] = 1.74654827828172;
  Cinematique_ROS_B.xi[159] = 1.75397532031767;
  Cinematique_ROS_B.xi[160] = 1.76143636531891;
  Cinematique_ROS_B.xi[161] = 1.76893241491127;
  Cinematique_ROS_B.xi[162] = 1.77646449552452;
  Cinematique_ROS_B.xi[163] = 1.78403365954944;
  Cinematique_ROS_B.xi[164] = 1.79164098655216;
  Cinematique_ROS_B.xi[165] = 1.79928758454972;
  Cinematique_ROS_B.xi[166] = 1.80697459135082;
  Cinematique_ROS_B.xi[167] = 1.81470317596628;
  Cinematique_ROS_B.xi[168] = 1.82247454009388;
  Cinematique_ROS_B.xi[169] = 1.83028991968276;
  Cinematique_ROS_B.xi[170] = 1.83815058658281;
  Cinematique_ROS_B.xi[171] = 1.84605785028518;
  Cinematique_ROS_B.xi[172] = 1.8540130597602;
  Cinematique_ROS_B.xi[173] = 1.86201760539967;
  Cinematique_ROS_B.xi[174] = 1.87007292107127;
  Cinematique_ROS_B.xi[175] = 1.878180486293;
  Cinematique_ROS_B.xi[176] = 1.88634182853678;
  Cinematique_ROS_B.xi[177] = 1.8945585256707;
  Cinematique_ROS_B.xi[178] = 1.90283220855043;
  Cinematique_ROS_B.xi[179] = 1.91116456377125;
  Cinematique_ROS_B.xi[180] = 1.91955733659319;
  Cinematique_ROS_B.xi[181] = 1.92801233405266;
  Cinematique_ROS_B.xi[182] = 1.93653142827569;
  Cinematique_ROS_B.xi[183] = 1.94511656000868;
  Cinematique_ROS_B.xi[184] = 1.95376974238465;
  Cinematique_ROS_B.xi[185] = 1.96249306494436;
  Cinematique_ROS_B.xi[186] = 1.97128869793366;
  Cinematique_ROS_B.xi[187] = 1.98015889690048;
  Cinematique_ROS_B.xi[188] = 1.98910600761744;
  Cinematique_ROS_B.xi[189] = 1.99813247135842;
  Cinematique_ROS_B.xi[190] = 2.00724083056053;
  Cinematique_ROS_B.xi[191] = 2.0164337349062;
  Cinematique_ROS_B.xi[192] = 2.02571394786385;
  Cinematique_ROS_B.xi[193] = 2.03508435372962;
  Cinematique_ROS_B.xi[194] = 2.04454796521753;
  Cinematique_ROS_B.xi[195] = 2.05410793165065;
  Cinematique_ROS_B.xi[196] = 2.06376754781173;
  Cinematique_ROS_B.xi[197] = 2.07353026351874;
  Cinematique_ROS_B.xi[198] = 2.0833996939983;
  Cinematique_ROS_B.xi[199] = 2.09337963113879;
  Cinematique_ROS_B.xi[200] = 2.10347405571488;
  Cinematique_ROS_B.xi[201] = 2.11368715068665;
  Cinematique_ROS_B.xi[202] = 2.12402331568952;
  Cinematique_ROS_B.xi[203] = 2.13448718284602;
  Cinematique_ROS_B.xi[204] = 2.14508363404789;
  Cinematique_ROS_B.xi[205] = 2.15581781987674;
  Cinematique_ROS_B.xi[206] = 2.16669518035431;
  Cinematique_ROS_B.xi[207] = 2.17772146774029;
  Cinematique_ROS_B.xi[208] = 2.18890277162636;
  Cinematique_ROS_B.xi[209] = 2.20024554661128;
  Cinematique_ROS_B.xi[210] = 2.21175664288416;
  Cinematique_ROS_B.xi[211] = 2.22344334009251;
  Cinematique_ROS_B.xi[212] = 2.23531338492992;
  Cinematique_ROS_B.xi[213] = 2.24737503294739;
  Cinematique_ROS_B.xi[214] = 2.25963709517379;
  Cinematique_ROS_B.xi[215] = 2.27210899022838;
  Cinematique_ROS_B.xi[216] = 2.28480080272449;
  Cinematique_ROS_B.xi[217] = 2.29772334890286;
  Cinematique_ROS_B.xi[218] = 2.31088825060137;
  Cinematique_ROS_B.xi[219] = 2.32430801887113;
  Cinematique_ROS_B.xi[220] = 2.33799614879653;
  Cinematique_ROS_B.xi[221] = 2.35196722737914;
  Cinematique_ROS_B.xi[222] = 2.36623705671729;
  Cinematique_ROS_B.xi[223] = 2.38082279517208;
  Cinematique_ROS_B.xi[224] = 2.39574311978193;
  Cinematique_ROS_B.xi[225] = 2.41101841390112;
  Cinematique_ROS_B.xi[226] = 2.42667098493715;
  Cinematique_ROS_B.xi[227] = 2.44272531820036;
  Cinematique_ROS_B.xi[228] = 2.4592083743347;
  Cinematique_ROS_B.xi[229] = 2.47614993967052;
  Cinematique_ROS_B.xi[230] = 2.49358304127105;
  Cinematique_ROS_B.xi[231] = 2.51154444162669;
  Cinematique_ROS_B.xi[232] = 2.53007523215985;
  Cinematique_ROS_B.xi[233] = 2.54922155032478;
  Cinematique_ROS_B.xi[234] = 2.56903545268184;
  Cinematique_ROS_B.xi[235] = 2.58957598670829;
  Cinematique_ROS_B.xi[236] = 2.61091051848882;
  Cinematique_ROS_B.xi[237] = 2.63311639363158;
  Cinematique_ROS_B.xi[238] = 2.65628303757674;
  Cinematique_ROS_B.xi[239] = 2.68051464328574;
  Cinematique_ROS_B.xi[240] = 2.70593365612306;
  Cinematique_ROS_B.xi[241] = 2.73268535904401;
  Cinematique_ROS_B.xi[242] = 2.76094400527999;
  Cinematique_ROS_B.xi[243] = 2.79092117400193;
  Cinematique_ROS_B.xi[244] = 2.82287739682644;
  Cinematique_ROS_B.xi[245] = 2.85713873087322;
  Cinematique_ROS_B.xi[246] = 2.89412105361341;
  Cinematique_ROS_B.xi[247] = 2.93436686720889;
  Cinematique_ROS_B.xi[248] = 2.97860327988184;
  Cinematique_ROS_B.xi[249] = 3.02783779176959;
  Cinematique_ROS_B.xi[250] = 3.08352613200214;
  Cinematique_ROS_B.xi[251] = 3.147889289518;
  Cinematique_ROS_B.xi[252] = 3.2245750520478;
  Cinematique_ROS_B.xi[253] = 3.32024473383983;
  Cinematique_ROS_B.xi[254] = 3.44927829856143;
  Cinematique_ROS_B.xi[255] = 3.65415288536101;
  Cinematique_ROS_B.xi[256] = 3.91075795952492;
  fitab = &tmp[0];
  do {
    exitg1 = 0;
    Cinemat_genrand_uint32_vector_k(state, Cinematique_ROS_B.u32);
    Cinematique_ROS_B.i_o = static_cast<int32_T>((Cinematique_ROS_B.u32[1] >>
      24U) + 1U);
    r = ((static_cast<real_T>(Cinematique_ROS_B.u32[0] >> 3U) * 1.6777216E+7 +
          static_cast<real_T>(static_cast<int32_T>(Cinematique_ROS_B.u32[1]) &
           16777215)) * 2.2204460492503131E-16 - 1.0) *
      Cinematique_ROS_B.xi[Cinematique_ROS_B.i_o];
    if (fabs(r) <= Cinematique_ROS_B.xi[Cinematique_ROS_B.i_o - 1]) {
      exitg1 = 1;
    } else if (Cinematique_ROS_B.i_o < 256) {
      Cinematique_ROS_B.x_c = Cinematique_ROS_genrandu_k(state);
      if ((fitab[Cinematique_ROS_B.i_o - 1] - fitab[Cinematique_ROS_B.i_o]) *
          Cinematique_ROS_B.x_c + fitab[Cinematique_ROS_B.i_o] < exp(-0.5 * r *
           r)) {
        exitg1 = 1;
      }
    } else {
      do {
        Cinematique_ROS_B.x_c = Cinematique_ROS_genrandu_k(state);
        Cinematique_ROS_B.x_c = log(Cinematique_ROS_B.x_c) * 0.273661237329758;
        Cinematique_ROS_B.d_u = Cinematique_ROS_genrandu_k(state);
      } while (!(-2.0 * log(Cinematique_ROS_B.d_u) > Cinematique_ROS_B.x_c *
                 Cinematique_ROS_B.x_c));

      if (r < 0.0) {
        r = Cinematique_ROS_B.x_c - 3.65415288536101;
      } else {
        r = 3.65415288536101 - Cinematique_ROS_B.x_c;
      }

      exitg1 = 1;
    }
  } while (exitg1 == 0);

  return r;
}

static void Cinematique_ROS_randn(const real_T varargin_1[2],
  emxArray_real_T_Cinematique_R_T *r)
{
  Cinematique_ROS_B.b_k_m = r->size[0] * r->size[1];
  r->size[0] = static_cast<int32_T>(varargin_1[0]);
  r->size[1] = 1;
  Cinema_emxEnsureCapacity_real_T(r, Cinematique_ROS_B.b_k_m);
  Cinematique_ROS_B.d_p = r->size[0] - 1;
  for (Cinematique_ROS_B.b_k_m = 0; Cinematique_ROS_B.b_k_m <=
       Cinematique_ROS_B.d_p; Cinematique_ROS_B.b_k_m++) {
    r->data[Cinematique_ROS_B.b_k_m] = Cinematiq_eml_rand_mt19937ar_k1
      (Cinematique_ROS_DW.state_l);
  }
}

static void Cinematique_ROS_rand_k(real_T varargin_1,
  emxArray_real_T_Cinematique_R_T *r)
{
  Cinematique_ROS_B.b_k_a = r->size[0];
  r->size[0] = static_cast<int32_T>(varargin_1);
  Cinema_emxEnsureCapacity_real_T(r, Cinematique_ROS_B.b_k_a);
  Cinematique_ROS_B.d_d = static_cast<int32_T>(varargin_1) - 1;
  for (Cinematique_ROS_B.b_k_a = 0; Cinematique_ROS_B.b_k_a <=
       Cinematique_ROS_B.d_d; Cinematique_ROS_B.b_k_a++) {
    memcpy(&Cinematique_ROS_B.uv1[0], &Cinematique_ROS_DW.state_l[0], 625U *
           sizeof(uint32_T));
    Cinematique__eml_rand_mt19937ar(Cinematique_ROS_B.uv1,
      Cinematique_ROS_DW.state_l, &r->data[Cinematique_ROS_B.b_k_a]);
  }
}

static void Cinema_NLPSolverInterface_solve(h_robotics_core_internal_Damp_T *obj,
  const real_T seed[4], real_T xSol[4], real_T *solutionInfo_Iterations, real_T *
  solutionInfo_RRAttempts, real_T *solutionInfo_Error, real_T
  *solutionInfo_ExitFlag, char_T solutionInfo_Status_data[], int32_T
  solutionInfo_Status_size[2])
{
  static const char_T tmp_2[14] = { 'b', 'e', 's', 't', ' ', 'a', 'v', 'a', 'i',
    'l', 'a', 'b', 'l', 'e' };

  static const char_T tmp_3[7] = { 's', 'u', 'c', 'c', 'e', 's', 's' };

  ab_robotics_manip_internal_Ri_T *obj_0;
  c_rigidBodyJoint_Cinematique__T *obj_1;
  emxArray_boolean_T_Cinematiqu_T *b;
  emxArray_boolean_T_Cinematiqu_T *tmp;
  emxArray_boolean_T_Cinematiqu_T *tmp_0;
  emxArray_boolean_T_Cinematiqu_T *tmp_1;
  emxArray_real_T_Cinematique_R_T *e;
  emxArray_real_T_Cinematique_R_T *lb;
  emxArray_real_T_Cinematique_R_T *newseed;
  emxArray_real_T_Cinematique_R_T *qi;
  emxArray_real_T_Cinematique_R_T *rn;
  emxArray_real_T_Cinematique_R_T *ub;
  f_robotics_manip_internal_IKE_T *args;
  boolean_T exitg1;
  boolean_T exitg2;
  boolean_T guard1 = false;
  boolean_T guard2 = false;
  boolean_T guard3 = false;
  obj->MaxNumIterationInternal = obj->MaxNumIteration;
  obj->MaxTimeInternal = obj->MaxTime;
  obj->SeedInternal[0] = seed[0];
  obj->SeedInternal[1] = seed[1];
  obj->SeedInternal[2] = seed[2];
  obj->SeedInternal[3] = seed[3];
  Cinematique_ROS_B.tol = obj->SolutionTolerance;
  Cinematique_ROS_tic(&obj->TimeObj.StartTime.tv_sec,
                      &obj->TimeObj.StartTime.tv_nsec);
  DampedBFGSwGradientProjection_s(obj, xSol, &Cinematique_ROS_B.exitFlag,
    &Cinematique_ROS_B.err, &Cinematique_ROS_B.iter);
  *solutionInfo_RRAttempts = 0.0;
  *solutionInfo_Iterations = Cinematique_ROS_B.iter;
  *solutionInfo_Error = Cinematique_ROS_B.err;
  Cinematique_ROS_B.exitFlagPrev = Cinematique_ROS_B.exitFlag;
  Cinematique_ROS_emxInit_real_T(&newseed, 1);
  Cinematique_ROS_emxInit_real_T(&qi, 2);
  Cinematique_ROS_emxInit_real_T(&ub, 1);
  Cinematique_ROS_emxInit_real_T(&lb, 1);
  Cinematique_ROS_emxInit_real_T(&rn, 1);
  Cinematique_ROS_emxInit_real_T(&e, 2);
  Cinematique_R_emxInit_boolean_T(&b, 1);
  Cinematique_R_emxInit_boolean_T(&tmp, 1);
  Cinematique_R_emxInit_boolean_T(&tmp_0, 1);
  Cinematique_R_emxInit_boolean_T(&tmp_1, 1);
  exitg1 = false;
  while ((!exitg1) && (obj->RandomRestart && (Cinematique_ROS_B.err >
           Cinematique_ROS_B.tol))) {
    obj->MaxNumIterationInternal -= Cinematique_ROS_B.iter;
    Cinematique_ROS_B.err = Cinematique_ROS_toc(obj->TimeObj.StartTime.tv_sec,
      obj->TimeObj.StartTime.tv_nsec);
    obj->MaxTimeInternal = obj->MaxTime - Cinematique_ROS_B.err;
    if (obj->MaxNumIterationInternal <= 0.0) {
      Cinematique_ROS_B.exitFlag = IterationLimitExceeded;
    }

    if ((Cinematique_ROS_B.exitFlag == IterationLimitExceeded) ||
        (Cinematique_ROS_B.exitFlag == TimeLimitExceeded)) {
      Cinematique_ROS_B.exitFlagPrev = Cinematique_ROS_B.exitFlag;
      exitg1 = true;
    } else {
      args = obj->ExtraArgs;
      obj_0 = args->Robot;
      Cinematique_ROS_B.ix = newseed->size[0];
      newseed->size[0] = static_cast<int32_T>(obj_0->PositionNumber);
      Cinema_emxEnsureCapacity_real_T(newseed, Cinematique_ROS_B.ix);
      Cinematique_ROS_B.nx = static_cast<int32_T>(obj_0->PositionNumber);
      for (Cinematique_ROS_B.ix = 0; Cinematique_ROS_B.ix < Cinematique_ROS_B.nx;
           Cinematique_ROS_B.ix++) {
        newseed->data[Cinematique_ROS_B.ix] = 0.0;
      }

      Cinematique_ROS_B.err = obj_0->NumBodies;
      Cinematique_ROS_B.c_h = static_cast<int32_T>(Cinematique_ROS_B.err) - 1;
      for (Cinematique_ROS_B.b_i = 0; Cinematique_ROS_B.b_i <=
           Cinematique_ROS_B.c_h; Cinematique_ROS_B.b_i++) {
        Cinematique_ROS_B.err = obj_0->PositionDoFMap[Cinematique_ROS_B.b_i];
        Cinematique_ROS_B.iter = obj_0->PositionDoFMap[Cinematique_ROS_B.b_i + 5];
        if (Cinematique_ROS_B.err <= Cinematique_ROS_B.iter) {
          obj_1 = obj_0->Bodies[Cinematique_ROS_B.b_i]->JointInternal;
          if (static_cast<int32_T>(obj_1->PositionNumber) == 0) {
            Cinematique_ROS_B.ix = qi->size[0] * qi->size[1];
            qi->size[0] = 1;
            qi->size[1] = 1;
            Cinema_emxEnsureCapacity_real_T(qi, Cinematique_ROS_B.ix);
            qi->data[0] = (rtNaN);
          } else {
            Cinematique_ROS_B.nx = obj_1->PositionLimitsInternal->size[0];
            Cinematique_ROS_B.ix = ub->size[0];
            ub->size[0] = Cinematique_ROS_B.nx;
            Cinema_emxEnsureCapacity_real_T(ub, Cinematique_ROS_B.ix);
            for (Cinematique_ROS_B.ix = 0; Cinematique_ROS_B.ix <
                 Cinematique_ROS_B.nx; Cinematique_ROS_B.ix++) {
              ub->data[Cinematique_ROS_B.ix] = obj_1->
                PositionLimitsInternal->data[Cinematique_ROS_B.ix +
                obj_1->PositionLimitsInternal->size[0]];
            }

            Cinematique_ROS_B.nx = obj_1->PositionLimitsInternal->size[0];
            Cinematique_ROS_B.ix = lb->size[0];
            lb->size[0] = Cinematique_ROS_B.nx;
            Cinema_emxEnsureCapacity_real_T(lb, Cinematique_ROS_B.ix);
            for (Cinematique_ROS_B.ix = 0; Cinematique_ROS_B.ix <
                 Cinematique_ROS_B.nx; Cinematique_ROS_B.ix++) {
              lb->data[Cinematique_ROS_B.ix] = obj_1->
                PositionLimitsInternal->data[Cinematique_ROS_B.ix];
            }

            Cinematique_ROS_B.ix = b->size[0];
            b->size[0] = lb->size[0];
            Cin_emxEnsureCapacity_boolean_T(b, Cinematique_ROS_B.ix);
            Cinematique_ROS_B.nx = lb->size[0];
            for (Cinematique_ROS_B.ix = 0; Cinematique_ROS_B.ix <
                 Cinematique_ROS_B.nx; Cinematique_ROS_B.ix++) {
              b->data[Cinematique_ROS_B.ix] = rtIsInf(lb->
                data[Cinematique_ROS_B.ix]);
            }

            Cinematique_ROS_B.ix = tmp->size[0];
            tmp->size[0] = lb->size[0];
            Cin_emxEnsureCapacity_boolean_T(tmp, Cinematique_ROS_B.ix);
            Cinematique_ROS_B.nx = lb->size[0];
            for (Cinematique_ROS_B.ix = 0; Cinematique_ROS_B.ix <
                 Cinematique_ROS_B.nx; Cinematique_ROS_B.ix++) {
              tmp->data[Cinematique_ROS_B.ix] = rtIsNaN(lb->
                data[Cinematique_ROS_B.ix]);
            }

            Cinematique_ROS_B.ix = b->size[0];
            Cin_emxEnsureCapacity_boolean_T(b, Cinematique_ROS_B.ix);
            Cinematique_ROS_B.nx = b->size[0];
            for (Cinematique_ROS_B.ix = 0; Cinematique_ROS_B.ix <
                 Cinematique_ROS_B.nx; Cinematique_ROS_B.ix++) {
              b->data[Cinematique_ROS_B.ix] = ((!b->data[Cinematique_ROS_B.ix]) &&
                (!tmp->data[Cinematique_ROS_B.ix]));
            }

            Cinematique_ROS_B.y_cx = true;
            Cinematique_ROS_B.ix = 0;
            exitg2 = false;
            while ((!exitg2) && (Cinematique_ROS_B.ix + 1 <= b->size[0])) {
              if (!b->data[Cinematique_ROS_B.ix]) {
                Cinematique_ROS_B.y_cx = false;
                exitg2 = true;
              } else {
                Cinematique_ROS_B.ix++;
              }
            }

            guard1 = false;
            guard2 = false;
            guard3 = false;
            if (Cinematique_ROS_B.y_cx) {
              Cinematique_ROS_B.ix = b->size[0];
              b->size[0] = ub->size[0];
              Cin_emxEnsureCapacity_boolean_T(b, Cinematique_ROS_B.ix);
              Cinematique_ROS_B.nx = ub->size[0];
              for (Cinematique_ROS_B.ix = 0; Cinematique_ROS_B.ix <
                   Cinematique_ROS_B.nx; Cinematique_ROS_B.ix++) {
                b->data[Cinematique_ROS_B.ix] = rtIsInf(ub->
                  data[Cinematique_ROS_B.ix]);
              }

              Cinematique_ROS_B.ix = tmp->size[0];
              tmp->size[0] = ub->size[0];
              Cin_emxEnsureCapacity_boolean_T(tmp, Cinematique_ROS_B.ix);
              Cinematique_ROS_B.nx = ub->size[0];
              for (Cinematique_ROS_B.ix = 0; Cinematique_ROS_B.ix <
                   Cinematique_ROS_B.nx; Cinematique_ROS_B.ix++) {
                tmp->data[Cinematique_ROS_B.ix] = rtIsNaN(ub->
                  data[Cinematique_ROS_B.ix]);
              }

              Cinematique_ROS_B.ix = b->size[0];
              Cin_emxEnsureCapacity_boolean_T(b, Cinematique_ROS_B.ix);
              Cinematique_ROS_B.nx = b->size[0];
              for (Cinematique_ROS_B.ix = 0; Cinematique_ROS_B.ix <
                   Cinematique_ROS_B.nx; Cinematique_ROS_B.ix++) {
                b->data[Cinematique_ROS_B.ix] = ((!b->data[Cinematique_ROS_B.ix])
                  && (!tmp->data[Cinematique_ROS_B.ix]));
              }

              Cinematique_ROS_B.y_cx = true;
              Cinematique_ROS_B.ix = 0;
              exitg2 = false;
              while ((!exitg2) && (Cinematique_ROS_B.ix + 1 <= b->size[0])) {
                if (!b->data[Cinematique_ROS_B.ix]) {
                  Cinematique_ROS_B.y_cx = false;
                  exitg2 = true;
                } else {
                  Cinematique_ROS_B.ix++;
                }
              }

              if (Cinematique_ROS_B.y_cx) {
                Cinematique_ROS_rand_k(obj_1->PositionNumber, rn);
                Cinematique_ROS_B.ix = qi->size[0] * qi->size[1];
                qi->size[0] = lb->size[0];
                qi->size[1] = 1;
                Cinema_emxEnsureCapacity_real_T(qi, Cinematique_ROS_B.ix);
                Cinematique_ROS_B.nx = lb->size[0] - 1;
                for (Cinematique_ROS_B.ix = 0; Cinematique_ROS_B.ix <=
                     Cinematique_ROS_B.nx; Cinematique_ROS_B.ix++) {
                  qi->data[Cinematique_ROS_B.ix] = (ub->
                    data[Cinematique_ROS_B.ix] - lb->data[Cinematique_ROS_B.ix])
                    * rn->data[Cinematique_ROS_B.ix] + lb->
                    data[Cinematique_ROS_B.ix];
                }
              } else {
                guard3 = true;
              }
            } else {
              guard3 = true;
            }

            if (guard3) {
              Cinematique_ROS_B.ix = b->size[0];
              b->size[0] = lb->size[0];
              Cin_emxEnsureCapacity_boolean_T(b, Cinematique_ROS_B.ix);
              Cinematique_ROS_B.nx = lb->size[0];
              for (Cinematique_ROS_B.ix = 0; Cinematique_ROS_B.ix <
                   Cinematique_ROS_B.nx; Cinematique_ROS_B.ix++) {
                b->data[Cinematique_ROS_B.ix] = rtIsInf(lb->
                  data[Cinematique_ROS_B.ix]);
              }

              Cinematique_ROS_B.ix = tmp->size[0];
              tmp->size[0] = lb->size[0];
              Cin_emxEnsureCapacity_boolean_T(tmp, Cinematique_ROS_B.ix);
              Cinematique_ROS_B.nx = lb->size[0];
              for (Cinematique_ROS_B.ix = 0; Cinematique_ROS_B.ix <
                   Cinematique_ROS_B.nx; Cinematique_ROS_B.ix++) {
                tmp->data[Cinematique_ROS_B.ix] = rtIsNaN(lb->
                  data[Cinematique_ROS_B.ix]);
              }

              Cinematique_ROS_B.ix = b->size[0];
              Cin_emxEnsureCapacity_boolean_T(b, Cinematique_ROS_B.ix);
              Cinematique_ROS_B.nx = b->size[0];
              for (Cinematique_ROS_B.ix = 0; Cinematique_ROS_B.ix <
                   Cinematique_ROS_B.nx; Cinematique_ROS_B.ix++) {
                b->data[Cinematique_ROS_B.ix] = ((!b->data[Cinematique_ROS_B.ix])
                  && (!tmp->data[Cinematique_ROS_B.ix]));
              }

              Cinematique_ROS_B.y_cx = true;
              Cinematique_ROS_B.ix = 0;
              exitg2 = false;
              while ((!exitg2) && (Cinematique_ROS_B.ix + 1 <= b->size[0])) {
                if (!b->data[Cinematique_ROS_B.ix]) {
                  Cinematique_ROS_B.y_cx = false;
                  exitg2 = true;
                } else {
                  Cinematique_ROS_B.ix++;
                }
              }

              if (Cinematique_ROS_B.y_cx) {
                Cinematique_ROS_B.ix = tmp->size[0];
                tmp->size[0] = ub->size[0];
                Cin_emxEnsureCapacity_boolean_T(tmp, Cinematique_ROS_B.ix);
                Cinematique_ROS_B.nx = ub->size[0];
                for (Cinematique_ROS_B.ix = 0; Cinematique_ROS_B.ix <
                     Cinematique_ROS_B.nx; Cinematique_ROS_B.ix++) {
                  tmp->data[Cinematique_ROS_B.ix] = rtIsInf(ub->
                    data[Cinematique_ROS_B.ix]);
                }

                Cinematique_ROS_B.ix = b->size[0];
                b->size[0] = ub->size[0];
                Cin_emxEnsureCapacity_boolean_T(b, Cinematique_ROS_B.ix);
                Cinematique_ROS_B.nx = ub->size[0];
                for (Cinematique_ROS_B.ix = 0; Cinematique_ROS_B.ix <
                     Cinematique_ROS_B.nx; Cinematique_ROS_B.ix++) {
                  b->data[Cinematique_ROS_B.ix] = rtIsNaN(ub->
                    data[Cinematique_ROS_B.ix]);
                }

                Cinematique_ROS_B.ix = tmp_0->size[0];
                tmp_0->size[0] = tmp->size[0];
                Cin_emxEnsureCapacity_boolean_T(tmp_0, Cinematique_ROS_B.ix);
                Cinematique_ROS_B.nx = tmp->size[0];
                for (Cinematique_ROS_B.ix = 0; Cinematique_ROS_B.ix <
                     Cinematique_ROS_B.nx; Cinematique_ROS_B.ix++) {
                  tmp_0->data[Cinematique_ROS_B.ix] = (tmp->
                    data[Cinematique_ROS_B.ix] || b->data[Cinematique_ROS_B.ix]);
                }

                if (Cinematique_ROS_any(tmp_0)) {
                  Cinematique_ROS_B.ub[0] = lb->size[0];
                  Cinematique_ROS_B.ub[1] = 1.0;
                  Cinematique_ROS_randn(Cinematique_ROS_B.ub, qi);
                  Cinematique_ROS_B.nx = qi->size[0] - 1;
                  Cinematique_ROS_B.ix = e->size[0] * e->size[1];
                  e->size[0] = qi->size[0];
                  e->size[1] = 1;
                  Cinema_emxEnsureCapacity_real_T(e, Cinematique_ROS_B.ix);
                  for (Cinematique_ROS_B.ix = 0; Cinematique_ROS_B.ix <=
                       Cinematique_ROS_B.nx; Cinematique_ROS_B.ix++) {
                    e->data[Cinematique_ROS_B.ix] = fabs(qi->
                      data[Cinematique_ROS_B.ix]);
                  }

                  Cinematique_ROS_B.ix = qi->size[0] * qi->size[1];
                  qi->size[0] = lb->size[0];
                  qi->size[1] = 1;
                  Cinema_emxEnsureCapacity_real_T(qi, Cinematique_ROS_B.ix);
                  Cinematique_ROS_B.nx = lb->size[0] - 1;
                  for (Cinematique_ROS_B.ix = 0; Cinematique_ROS_B.ix <=
                       Cinematique_ROS_B.nx; Cinematique_ROS_B.ix++) {
                    qi->data[Cinematique_ROS_B.ix] = lb->
                      data[Cinematique_ROS_B.ix] + e->data[Cinematique_ROS_B.ix];
                  }
                } else {
                  guard2 = true;
                }
              } else {
                guard2 = true;
              }
            }

            if (guard2) {
              Cinematique_ROS_B.ix = tmp->size[0];
              tmp->size[0] = lb->size[0];
              Cin_emxEnsureCapacity_boolean_T(tmp, Cinematique_ROS_B.ix);
              Cinematique_ROS_B.nx = lb->size[0];
              for (Cinematique_ROS_B.ix = 0; Cinematique_ROS_B.ix <
                   Cinematique_ROS_B.nx; Cinematique_ROS_B.ix++) {
                tmp->data[Cinematique_ROS_B.ix] = rtIsInf(lb->
                  data[Cinematique_ROS_B.ix]);
              }

              Cinematique_ROS_B.ix = b->size[0];
              b->size[0] = lb->size[0];
              Cin_emxEnsureCapacity_boolean_T(b, Cinematique_ROS_B.ix);
              Cinematique_ROS_B.nx = lb->size[0];
              for (Cinematique_ROS_B.ix = 0; Cinematique_ROS_B.ix <
                   Cinematique_ROS_B.nx; Cinematique_ROS_B.ix++) {
                b->data[Cinematique_ROS_B.ix] = rtIsNaN(lb->
                  data[Cinematique_ROS_B.ix]);
              }

              Cinematique_ROS_B.ix = tmp_1->size[0];
              tmp_1->size[0] = tmp->size[0];
              Cin_emxEnsureCapacity_boolean_T(tmp_1, Cinematique_ROS_B.ix);
              Cinematique_ROS_B.nx = tmp->size[0];
              for (Cinematique_ROS_B.ix = 0; Cinematique_ROS_B.ix <
                   Cinematique_ROS_B.nx; Cinematique_ROS_B.ix++) {
                tmp_1->data[Cinematique_ROS_B.ix] = (tmp->
                  data[Cinematique_ROS_B.ix] || b->data[Cinematique_ROS_B.ix]);
              }

              if (Cinematique_ROS_any(tmp_1)) {
                Cinematique_ROS_B.ix = b->size[0];
                b->size[0] = ub->size[0];
                Cin_emxEnsureCapacity_boolean_T(b, Cinematique_ROS_B.ix);
                Cinematique_ROS_B.nx = ub->size[0];
                for (Cinematique_ROS_B.ix = 0; Cinematique_ROS_B.ix <
                     Cinematique_ROS_B.nx; Cinematique_ROS_B.ix++) {
                  b->data[Cinematique_ROS_B.ix] = rtIsInf(ub->
                    data[Cinematique_ROS_B.ix]);
                }

                Cinematique_ROS_B.ix = tmp->size[0];
                tmp->size[0] = ub->size[0];
                Cin_emxEnsureCapacity_boolean_T(tmp, Cinematique_ROS_B.ix);
                Cinematique_ROS_B.nx = ub->size[0];
                for (Cinematique_ROS_B.ix = 0; Cinematique_ROS_B.ix <
                     Cinematique_ROS_B.nx; Cinematique_ROS_B.ix++) {
                  tmp->data[Cinematique_ROS_B.ix] = rtIsNaN(ub->
                    data[Cinematique_ROS_B.ix]);
                }

                Cinematique_ROS_B.ix = b->size[0];
                Cin_emxEnsureCapacity_boolean_T(b, Cinematique_ROS_B.ix);
                Cinematique_ROS_B.nx = b->size[0];
                for (Cinematique_ROS_B.ix = 0; Cinematique_ROS_B.ix <
                     Cinematique_ROS_B.nx; Cinematique_ROS_B.ix++) {
                  b->data[Cinematique_ROS_B.ix] = ((!b->
                    data[Cinematique_ROS_B.ix]) && (!tmp->
                    data[Cinematique_ROS_B.ix]));
                }

                Cinematique_ROS_B.y_cx = true;
                Cinematique_ROS_B.ix = 0;
                exitg2 = false;
                while ((!exitg2) && (Cinematique_ROS_B.ix + 1 <= b->size[0])) {
                  if (!b->data[Cinematique_ROS_B.ix]) {
                    Cinematique_ROS_B.y_cx = false;
                    exitg2 = true;
                  } else {
                    Cinematique_ROS_B.ix++;
                  }
                }

                if (Cinematique_ROS_B.y_cx) {
                  Cinematique_ROS_B.ub[0] = ub->size[0];
                  Cinematique_ROS_B.ub[1] = 1.0;
                  Cinematique_ROS_randn(Cinematique_ROS_B.ub, qi);
                  Cinematique_ROS_B.nx = qi->size[0] - 1;
                  Cinematique_ROS_B.ix = e->size[0] * e->size[1];
                  e->size[0] = qi->size[0];
                  e->size[1] = 1;
                  Cinema_emxEnsureCapacity_real_T(e, Cinematique_ROS_B.ix);
                  for (Cinematique_ROS_B.ix = 0; Cinematique_ROS_B.ix <=
                       Cinematique_ROS_B.nx; Cinematique_ROS_B.ix++) {
                    e->data[Cinematique_ROS_B.ix] = fabs(qi->
                      data[Cinematique_ROS_B.ix]);
                  }

                  Cinematique_ROS_B.ix = qi->size[0] * qi->size[1];
                  qi->size[0] = ub->size[0];
                  qi->size[1] = 1;
                  Cinema_emxEnsureCapacity_real_T(qi, Cinematique_ROS_B.ix);
                  Cinematique_ROS_B.nx = ub->size[0] - 1;
                  for (Cinematique_ROS_B.ix = 0; Cinematique_ROS_B.ix <=
                       Cinematique_ROS_B.nx; Cinematique_ROS_B.ix++) {
                    qi->data[Cinematique_ROS_B.ix] = ub->
                      data[Cinematique_ROS_B.ix] - e->data[Cinematique_ROS_B.ix];
                  }
                } else {
                  guard1 = true;
                }
              } else {
                guard1 = true;
              }
            }

            if (guard1) {
              Cinematique_ROS_B.ub[0] = ub->size[0];
              Cinematique_ROS_B.ub[1] = 1.0;
              Cinematique_ROS_randn(Cinematique_ROS_B.ub, qi);
            }
          }

          if (Cinematique_ROS_B.err > Cinematique_ROS_B.iter) {
            Cinematique_ROS_B.nx = 0;
            Cinematique_ROS_B.ix = 0;
          } else {
            Cinematique_ROS_B.nx = static_cast<int32_T>(Cinematique_ROS_B.err) -
              1;
            Cinematique_ROS_B.ix = static_cast<int32_T>(Cinematique_ROS_B.iter);
          }

          Cinematique_ROS_B.unnamed_idx_1 = Cinematique_ROS_B.ix -
            Cinematique_ROS_B.nx;
          for (Cinematique_ROS_B.ix = 0; Cinematique_ROS_B.ix <
               Cinematique_ROS_B.unnamed_idx_1; Cinematique_ROS_B.ix++) {
            newseed->data[Cinematique_ROS_B.nx + Cinematique_ROS_B.ix] =
              qi->data[Cinematique_ROS_B.ix];
          }
        }
      }

      obj->SeedInternal[0] = newseed->data[0];
      obj->SeedInternal[1] = newseed->data[1];
      obj->SeedInternal[2] = newseed->data[2];
      obj->SeedInternal[3] = newseed->data[3];
      DampedBFGSwGradientProjection_s(obj, Cinematique_ROS_B.c_xSol,
        &Cinematique_ROS_B.exitFlag, &Cinematique_ROS_B.err,
        &Cinematique_ROS_B.iter);
      if (Cinematique_ROS_B.err < *solutionInfo_Error) {
        xSol[0] = Cinematique_ROS_B.c_xSol[0];
        xSol[1] = Cinematique_ROS_B.c_xSol[1];
        xSol[2] = Cinematique_ROS_B.c_xSol[2];
        xSol[3] = Cinematique_ROS_B.c_xSol[3];
        *solutionInfo_Error = Cinematique_ROS_B.err;
        Cinematique_ROS_B.exitFlagPrev = Cinematique_ROS_B.exitFlag;
      }

      (*solutionInfo_RRAttempts)++;
      *solutionInfo_Iterations += Cinematique_ROS_B.iter;
    }
  }

  Cinematique_R_emxFree_boolean_T(&tmp_1);
  Cinematique_R_emxFree_boolean_T(&tmp_0);
  Cinematique_R_emxFree_boolean_T(&tmp);
  Cinematique_R_emxFree_boolean_T(&b);
  Cinematique_ROS_emxFree_real_T(&e);
  Cinematique_ROS_emxFree_real_T(&rn);
  Cinematique_ROS_emxFree_real_T(&lb);
  Cinematique_ROS_emxFree_real_T(&ub);
  Cinematique_ROS_emxFree_real_T(&qi);
  Cinematique_ROS_emxFree_real_T(&newseed);
  *solutionInfo_ExitFlag = Cinematique_ROS_B.exitFlagPrev;
  if (*solutionInfo_Error < Cinematique_ROS_B.tol) {
    solutionInfo_Status_size[0] = 1;
    solutionInfo_Status_size[1] = 7;
    for (Cinematique_ROS_B.ix = 0; Cinematique_ROS_B.ix < 7;
         Cinematique_ROS_B.ix++) {
      solutionInfo_Status_data[Cinematique_ROS_B.ix] =
        tmp_3[Cinematique_ROS_B.ix];
    }
  } else {
    solutionInfo_Status_size[0] = 1;
    solutionInfo_Status_size[1] = 14;
    for (Cinematique_ROS_B.ix = 0; Cinematique_ROS_B.ix < 14;
         Cinematique_ROS_B.ix++) {
      solutionInfo_Status_data[Cinematique_ROS_B.ix] =
        tmp_2[Cinematique_ROS_B.ix];
    }
  }
}

static void Cinem_inverseKinematics_solve_k(b_inverseKinematics_Cinematiq_T *obj,
  real_T initialGuess[4], real_T *solutionInfo_Iterations, real_T
  *solutionInfo_NumRandomRestarts, real_T *solutionInfo_PoseErrorNorm, real_T
  *solutionInfo_ExitFlag, char_T solutionInfo_Status_data[], int32_T
  solutionInfo_Status_size[2])
{
  ab_robotics_manip_internal_Ri_T *obj_0;
  emxArray_char_T_Cinematique_R_T *endEffectorName;
  emxArray_int32_T_Cinematique__T *h;
  emxArray_real_T_Cinematique_R_T *bodyIndices;
  emxArray_real_T_Cinematique_R_T *bodyIndices_0;
  emxArray_real_T_Cinematique_R_T *e;
  emxArray_real_T_Cinematique_R_T *limits;
  emxArray_real_T_Cinematique_R_T *limits_0;
  emxArray_real_T_Cinematique_R_T *limits_1;
  emxArray_real_T_Cinematique_R_T *positionIndices;
  emxArray_real_T_Cinematique_R_T *y;
  y_robotics_manip_internal_Rig_T *body;
  boolean_T exitg1;
  boolean_T guard1 = false;
  Cinematique_ROS_emxInit_real_T(&limits, 2);
  Cinematique_ROS_emxInit_real_T(&limits_0, 1);
  obj_0 = obj->RigidBodyTreeInternal;
  RigidBodyTree_get_JointPosition(obj_0, limits);
  Cinematique_ROS_B.loop_ub_p = limits->size[0];
  Cinematique_ROS_B.b_k = limits_0->size[0];
  limits_0->size[0] = Cinematique_ROS_B.loop_ub_p;
  Cinema_emxEnsureCapacity_real_T(limits_0, Cinematique_ROS_B.b_k);
  for (Cinematique_ROS_B.b_k = 0; Cinematique_ROS_B.b_k <
       Cinematique_ROS_B.loop_ub_p; Cinematique_ROS_B.b_k++) {
    limits_0->data[Cinematique_ROS_B.b_k] = limits->data[Cinematique_ROS_B.b_k +
      limits->size[0]] + 4.4408920985006262E-16;
  }

  Cinematique_ROS_B.ubOK[0] = (initialGuess[0] <= limits_0->data[0]);
  Cinematique_ROS_B.ubOK[1] = (initialGuess[1] <= limits_0->data[1]);
  Cinematique_ROS_B.ubOK[2] = (initialGuess[2] <= limits_0->data[2]);
  Cinematique_ROS_B.ubOK[3] = (initialGuess[3] <= limits_0->data[3]);
  Cinematique_ROS_emxFree_real_T(&limits_0);
  Cinematique_ROS_emxInit_real_T(&limits_1, 1);
  Cinematique_ROS_B.loop_ub_p = limits->size[0];
  Cinematique_ROS_B.b_k = limits_1->size[0];
  limits_1->size[0] = Cinematique_ROS_B.loop_ub_p;
  Cinema_emxEnsureCapacity_real_T(limits_1, Cinematique_ROS_B.b_k);
  for (Cinematique_ROS_B.b_k = 0; Cinematique_ROS_B.b_k <
       Cinematique_ROS_B.loop_ub_p; Cinematique_ROS_B.b_k++) {
    limits_1->data[Cinematique_ROS_B.b_k] = limits->data[Cinematique_ROS_B.b_k]
      - 4.4408920985006262E-16;
  }

  Cinematique_ROS_B.lbOK[0] = (initialGuess[0] >= limits_1->data[0]);
  Cinematique_ROS_B.lbOK[1] = (initialGuess[1] >= limits_1->data[1]);
  Cinematique_ROS_B.lbOK[2] = (initialGuess[2] >= limits_1->data[2]);
  Cinematique_ROS_B.lbOK[3] = (initialGuess[3] >= limits_1->data[3]);
  Cinematique_ROS_emxFree_real_T(&limits_1);
  Cinematique_ROS_B.y_c = true;
  Cinematique_ROS_B.b_k = 0;
  exitg1 = false;
  while ((!exitg1) && (Cinematique_ROS_B.b_k < 4)) {
    if (!Cinematique_ROS_B.ubOK[Cinematique_ROS_B.b_k]) {
      Cinematique_ROS_B.y_c = false;
      exitg1 = true;
    } else {
      Cinematique_ROS_B.b_k++;
    }
  }

  guard1 = false;
  if (Cinematique_ROS_B.y_c) {
    Cinematique_ROS_B.y_c = true;
    Cinematique_ROS_B.b_k = 0;
    exitg1 = false;
    while ((!exitg1) && (Cinematique_ROS_B.b_k < 4)) {
      if (!Cinematique_ROS_B.lbOK[Cinematique_ROS_B.b_k]) {
        Cinematique_ROS_B.y_c = false;
        exitg1 = true;
      } else {
        Cinematique_ROS_B.b_k++;
      }
    }

    if (Cinematique_ROS_B.y_c) {
    } else {
      guard1 = true;
    }
  } else {
    guard1 = true;
  }

  if (guard1) {
    Cinematique_ROS_B.ubOK_l[0] = !Cinematique_ROS_B.ubOK[0];
    Cinematique_ROS_B.ubOK_l[1] = !Cinematique_ROS_B.ubOK[1];
    Cinematique_ROS_B.ubOK_l[2] = !Cinematique_ROS_B.ubOK[2];
    Cinematique_ROS_B.ubOK_l[3] = !Cinematique_ROS_B.ubOK[3];
    Cinematique_ROS_eml_find(Cinematique_ROS_B.ubOK_l,
      Cinematique_ROS_B.tmp_data, &Cinematique_ROS_B.tmp_size);
    Cinematique_ROS_B.indicesUpperBoundViolation_size =
      Cinematique_ROS_B.tmp_size;
    Cinematique_ROS_B.loop_ub_p = Cinematique_ROS_B.tmp_size;
    if (0 <= Cinematique_ROS_B.loop_ub_p - 1) {
      memcpy(&Cinematique_ROS_B.indicesUpperBoundViolation_data[0],
             &Cinematique_ROS_B.tmp_data[0], Cinematique_ROS_B.loop_ub_p *
             sizeof(int32_T));
    }

    for (Cinematique_ROS_B.b_k = 0; Cinematique_ROS_B.b_k <
         Cinematique_ROS_B.indicesUpperBoundViolation_size;
         Cinematique_ROS_B.b_k++) {
      Cinematique_ROS_B.indicesUpperBoundViolation_da_e =
        Cinematique_ROS_B.indicesUpperBoundViolation_data[Cinematique_ROS_B.b_k];
      initialGuess[Cinematique_ROS_B.indicesUpperBoundViolation_da_e - 1] =
        limits->data[(Cinematique_ROS_B.indicesUpperBoundViolation_da_e +
                      limits->size[0]) - 1];
    }

    Cinematique_ROS_B.ubOK[0] = !Cinematique_ROS_B.lbOK[0];
    Cinematique_ROS_B.ubOK[1] = !Cinematique_ROS_B.lbOK[1];
    Cinematique_ROS_B.ubOK[2] = !Cinematique_ROS_B.lbOK[2];
    Cinematique_ROS_B.ubOK[3] = !Cinematique_ROS_B.lbOK[3];
    Cinematique_ROS_eml_find(Cinematique_ROS_B.ubOK, Cinematique_ROS_B.tmp_data,
      &Cinematique_ROS_B.tmp_size);
    Cinematique_ROS_B.indicesUpperBoundViolation_size =
      Cinematique_ROS_B.tmp_size;
    Cinematique_ROS_B.loop_ub_p = Cinematique_ROS_B.tmp_size;
    if (0 <= Cinematique_ROS_B.loop_ub_p - 1) {
      memcpy(&Cinematique_ROS_B.indicesUpperBoundViolation_data[0],
             &Cinematique_ROS_B.tmp_data[0], Cinematique_ROS_B.loop_ub_p *
             sizeof(int32_T));
    }

    for (Cinematique_ROS_B.b_k = 0; Cinematique_ROS_B.b_k <
         Cinematique_ROS_B.indicesUpperBoundViolation_size;
         Cinematique_ROS_B.b_k++) {
      Cinematique_ROS_B.indicesUpperBoundViolation_da_e =
        Cinematique_ROS_B.indicesUpperBoundViolation_data[Cinematique_ROS_B.b_k];
      initialGuess[Cinematique_ROS_B.indicesUpperBoundViolation_da_e - 1] =
        limits->data[Cinematique_ROS_B.indicesUpperBoundViolation_da_e - 1];
    }
  }

  Cinematique_ROS_emxInit_char_T(&endEffectorName, 2);
  Cinema_NLPSolverInterface_solve(obj->Solver, initialGuess,
    Cinematique_ROS_B.qvSolRaw, &Cinematique_ROS_B.d3, &Cinematique_ROS_B.d4,
    &Cinematique_ROS_B.d5, &Cinematique_ROS_B.d6, solutionInfo_Status_data,
    solutionInfo_Status_size);
  *solutionInfo_ExitFlag = Cinematique_ROS_B.d6;
  *solutionInfo_PoseErrorNorm = Cinematique_ROS_B.d5;
  *solutionInfo_NumRandomRestarts = Cinematique_ROS_B.d4;
  *solutionInfo_Iterations = Cinematique_ROS_B.d3;
  obj_0 = obj->RigidBodyTreeInternal;
  Cinematique_ROS_B.b_k = endEffectorName->size[0] * endEffectorName->size[1];
  endEffectorName->size[0] = 1;
  endEffectorName->size[1] = obj->Solver->ExtraArgs->BodyName->size[1];
  Cinema_emxEnsureCapacity_char_T(endEffectorName, Cinematique_ROS_B.b_k);
  Cinematique_ROS_B.loop_ub_p = obj->Solver->ExtraArgs->BodyName->size[0] *
    obj->Solver->ExtraArgs->BodyName->size[1] - 1;
  for (Cinematique_ROS_B.b_k = 0; Cinematique_ROS_B.b_k <=
       Cinematique_ROS_B.loop_ub_p; Cinematique_ROS_B.b_k++) {
    endEffectorName->data[Cinematique_ROS_B.b_k] = obj->Solver->
      ExtraArgs->BodyName->data[Cinematique_ROS_B.b_k];
  }

  Cinematique_ROS_emxInit_real_T(&bodyIndices, 1);
  Cinematique_ROS_B.b_k = bodyIndices->size[0];
  bodyIndices->size[0] = static_cast<int32_T>(obj_0->NumBodies);
  Cinema_emxEnsureCapacity_real_T(bodyIndices, Cinematique_ROS_B.b_k);
  Cinematique_ROS_B.loop_ub_p = static_cast<int32_T>(obj_0->NumBodies);
  for (Cinematique_ROS_B.b_k = 0; Cinematique_ROS_B.b_k <
       Cinematique_ROS_B.loop_ub_p; Cinematique_ROS_B.b_k++) {
    bodyIndices->data[Cinematique_ROS_B.b_k] = 0.0;
  }

  Cinematique_ROS_B.bid = RigidBodyTree_findBodyIndexByNa(obj_0, endEffectorName);
  Cinematique_ROS_emxFree_char_T(&endEffectorName);
  if (Cinematique_ROS_B.bid == 0.0) {
    Cinematique_ROS_B.b_k = bodyIndices->size[0];
    bodyIndices->size[0] = 1;
    Cinema_emxEnsureCapacity_real_T(bodyIndices, Cinematique_ROS_B.b_k);
    bodyIndices->data[0] = 0.0;
  } else {
    body = obj_0->Bodies[static_cast<int32_T>(Cinematique_ROS_B.bid) - 1];
    Cinematique_ROS_B.bid = 1.0;
    while (body->ParentIndex != 0.0) {
      bodyIndices->data[static_cast<int32_T>(Cinematique_ROS_B.bid) - 1] =
        body->Index;
      body = obj_0->Bodies[static_cast<int32_T>(body->ParentIndex) - 1];
      Cinematique_ROS_B.bid++;
    }

    if (1.0 > Cinematique_ROS_B.bid - 1.0) {
      Cinematique_ROS_B.indicesUpperBoundViolation_da_e = -1;
    } else {
      Cinematique_ROS_B.indicesUpperBoundViolation_da_e = static_cast<int32_T>
        (Cinematique_ROS_B.bid - 1.0) - 1;
    }

    Cinematique_ROS_emxInit_real_T(&bodyIndices_0, 1);
    Cinematique_ROS_B.b_k = bodyIndices_0->size[0];
    bodyIndices_0->size[0] = Cinematique_ROS_B.indicesUpperBoundViolation_da_e +
      3;
    Cinema_emxEnsureCapacity_real_T(bodyIndices_0, Cinematique_ROS_B.b_k);
    for (Cinematique_ROS_B.b_k = 0; Cinematique_ROS_B.b_k <=
         Cinematique_ROS_B.indicesUpperBoundViolation_da_e;
         Cinematique_ROS_B.b_k++) {
      bodyIndices_0->data[Cinematique_ROS_B.b_k] = bodyIndices->
        data[Cinematique_ROS_B.b_k];
    }

    bodyIndices_0->data[Cinematique_ROS_B.indicesUpperBoundViolation_da_e + 1] =
      body->Index;
    bodyIndices_0->data[Cinematique_ROS_B.indicesUpperBoundViolation_da_e + 2] =
      0.0;
    Cinematique_ROS_B.b_k = bodyIndices->size[0];
    bodyIndices->size[0] = bodyIndices_0->size[0];
    Cinema_emxEnsureCapacity_real_T(bodyIndices, Cinematique_ROS_B.b_k);
    Cinematique_ROS_B.loop_ub_p = bodyIndices_0->size[0];
    for (Cinematique_ROS_B.b_k = 0; Cinematique_ROS_B.b_k <
         Cinematique_ROS_B.loop_ub_p; Cinematique_ROS_B.b_k++) {
      bodyIndices->data[Cinematique_ROS_B.b_k] = bodyIndices_0->
        data[Cinematique_ROS_B.b_k];
    }

    Cinematique_ROS_emxFree_real_T(&bodyIndices_0);
  }

  obj_0 = obj->RigidBodyTreeInternal;
  Cinematique_ROS_B.indicesUpperBoundViolation_size = bodyIndices->size[0] - 1;
  Cinematique_ROS_B.indicesUpperBoundViolation_da_e = 0;
  for (Cinematique_ROS_B.b_k = 0; Cinematique_ROS_B.b_k <=
       Cinematique_ROS_B.indicesUpperBoundViolation_size; Cinematique_ROS_B.b_k
       ++) {
    if (bodyIndices->data[Cinematique_ROS_B.b_k] != 0.0) {
      Cinematique_ROS_B.indicesUpperBoundViolation_da_e++;
    }
  }

  Cinematique_ROS_emxInit_int32_T(&h, 1);
  Cinematique_ROS_B.b_k = h->size[0];
  h->size[0] = Cinematique_ROS_B.indicesUpperBoundViolation_da_e;
  Cinem_emxEnsureCapacity_int32_T(h, Cinematique_ROS_B.b_k);
  Cinematique_ROS_B.indicesUpperBoundViolation_da_e = 0;
  for (Cinematique_ROS_B.b_k = 0; Cinematique_ROS_B.b_k <=
       Cinematique_ROS_B.indicesUpperBoundViolation_size; Cinematique_ROS_B.b_k
       ++) {
    if (bodyIndices->data[Cinematique_ROS_B.b_k] != 0.0) {
      h->data[Cinematique_ROS_B.indicesUpperBoundViolation_da_e] =
        Cinematique_ROS_B.b_k + 1;
      Cinematique_ROS_B.indicesUpperBoundViolation_da_e++;
    }
  }

  Cinematique_ROS_B.b_k = limits->size[0] * limits->size[1];
  limits->size[0] = h->size[0];
  limits->size[1] = 2;
  Cinema_emxEnsureCapacity_real_T(limits, Cinematique_ROS_B.b_k);
  Cinematique_ROS_B.loop_ub_p = h->size[0];
  for (Cinematique_ROS_B.b_k = 0; Cinematique_ROS_B.b_k <
       Cinematique_ROS_B.loop_ub_p; Cinematique_ROS_B.b_k++) {
    limits->data[Cinematique_ROS_B.b_k] = obj_0->PositionDoFMap
      [static_cast<int32_T>(bodyIndices->data[h->data[Cinematique_ROS_B.b_k] - 1])
      - 1];
  }

  Cinematique_ROS_B.loop_ub_p = h->size[0];
  for (Cinematique_ROS_B.b_k = 0; Cinematique_ROS_B.b_k <
       Cinematique_ROS_B.loop_ub_p; Cinematique_ROS_B.b_k++) {
    limits->data[Cinematique_ROS_B.b_k + limits->size[0]] =
      obj_0->PositionDoFMap[static_cast<int32_T>(bodyIndices->data[h->
      data[Cinematique_ROS_B.b_k] - 1]) + 4];
  }

  Cinematique_ROS_emxFree_int32_T(&h);
  Cinematique_ROS_emxFree_real_T(&bodyIndices);
  Cinematique_ROS_emxInit_real_T(&positionIndices, 2);
  Cinematique_ROS_B.b_k = positionIndices->size[0] * positionIndices->size[1];
  positionIndices->size[0] = 1;
  positionIndices->size[1] = static_cast<int32_T>(obj_0->PositionNumber);
  Cinema_emxEnsureCapacity_real_T(positionIndices, Cinematique_ROS_B.b_k);
  Cinematique_ROS_B.loop_ub_p = static_cast<int32_T>(obj_0->PositionNumber) - 1;
  for (Cinematique_ROS_B.b_k = 0; Cinematique_ROS_B.b_k <=
       Cinematique_ROS_B.loop_ub_p; Cinematique_ROS_B.b_k++) {
    positionIndices->data[Cinematique_ROS_B.b_k] = 0.0;
  }

  Cinematique_ROS_B.bid = 0.0;
  Cinematique_ROS_B.indicesUpperBoundViolation_da_e = limits->size[0] - 1;
  Cinematique_ROS_emxInit_real_T(&e, 2);
  Cinematique_ROS_emxInit_real_T(&y, 2);
  for (Cinematique_ROS_B.indicesUpperBoundViolation_size = 0;
       Cinematique_ROS_B.indicesUpperBoundViolation_size <=
       Cinematique_ROS_B.indicesUpperBoundViolation_da_e;
       Cinematique_ROS_B.indicesUpperBoundViolation_size++) {
    Cinematique_ROS_B.numPositions = (limits->
      data[Cinematique_ROS_B.indicesUpperBoundViolation_size + limits->size[0]]
      - limits->data[Cinematique_ROS_B.indicesUpperBoundViolation_size]) + 1.0;
    if (Cinematique_ROS_B.numPositions > 0.0) {
      if (Cinematique_ROS_B.numPositions < 1.0) {
        y->size[0] = 1;
        y->size[1] = 0;
      } else if (rtIsInf(Cinematique_ROS_B.numPositions) && (1.0 ==
                  Cinematique_ROS_B.numPositions)) {
        Cinematique_ROS_B.b_k = y->size[0] * y->size[1];
        y->size[0] = 1;
        y->size[1] = 1;
        Cinema_emxEnsureCapacity_real_T(y, Cinematique_ROS_B.b_k);
        y->data[0] = (rtNaN);
      } else {
        Cinematique_ROS_B.b_k = y->size[0] * y->size[1];
        y->size[0] = 1;
        Cinematique_ROS_B.loop_ub_p = static_cast<int32_T>(floor
          (Cinematique_ROS_B.numPositions - 1.0));
        y->size[1] = Cinematique_ROS_B.loop_ub_p + 1;
        Cinema_emxEnsureCapacity_real_T(y, Cinematique_ROS_B.b_k);
        for (Cinematique_ROS_B.b_k = 0; Cinematique_ROS_B.b_k <=
             Cinematique_ROS_B.loop_ub_p; Cinematique_ROS_B.b_k++) {
          y->data[Cinematique_ROS_B.b_k] = static_cast<real_T>
            (Cinematique_ROS_B.b_k) + 1.0;
        }
      }

      if (rtIsNaN(limits->data[Cinematique_ROS_B.indicesUpperBoundViolation_size])
          || rtIsNaN(limits->
                     data[Cinematique_ROS_B.indicesUpperBoundViolation_size +
                     limits->size[0]])) {
        Cinematique_ROS_B.b_k = e->size[0] * e->size[1];
        e->size[0] = 1;
        e->size[1] = 1;
        Cinema_emxEnsureCapacity_real_T(e, Cinematique_ROS_B.b_k);
        e->data[0] = (rtNaN);
      } else if (limits->data[Cinematique_ROS_B.indicesUpperBoundViolation_size
                 + limits->size[0]] < limits->
                 data[Cinematique_ROS_B.indicesUpperBoundViolation_size]) {
        e->size[0] = 1;
        e->size[1] = 0;
      } else if ((rtIsInf(limits->
                          data[Cinematique_ROS_B.indicesUpperBoundViolation_size])
                  || rtIsInf(limits->
                             data[Cinematique_ROS_B.indicesUpperBoundViolation_size
        + limits->size[0]])) && (limits->
                  data[Cinematique_ROS_B.indicesUpperBoundViolation_size +
                  limits->size[0]] == limits->
                  data[Cinematique_ROS_B.indicesUpperBoundViolation_size])) {
        Cinematique_ROS_B.b_k = e->size[0] * e->size[1];
        e->size[0] = 1;
        e->size[1] = 1;
        Cinema_emxEnsureCapacity_real_T(e, Cinematique_ROS_B.b_k);
        e->data[0] = (rtNaN);
      } else if (floor(limits->
                       data[Cinematique_ROS_B.indicesUpperBoundViolation_size]) ==
                 limits->data[Cinematique_ROS_B.indicesUpperBoundViolation_size])
      {
        Cinematique_ROS_B.b_k = e->size[0] * e->size[1];
        e->size[0] = 1;
        e->size[1] = static_cast<int32_T>(floor(limits->
          data[Cinematique_ROS_B.indicesUpperBoundViolation_size + limits->size
          [0]] - limits->data[Cinematique_ROS_B.indicesUpperBoundViolation_size]))
          + 1;
        Cinema_emxEnsureCapacity_real_T(e, Cinematique_ROS_B.b_k);
        Cinematique_ROS_B.loop_ub_p = static_cast<int32_T>(floor(limits->
          data[Cinematique_ROS_B.indicesUpperBoundViolation_size + limits->size
          [0]] - limits->data[Cinematique_ROS_B.indicesUpperBoundViolation_size]));
        for (Cinematique_ROS_B.b_k = 0; Cinematique_ROS_B.b_k <=
             Cinematique_ROS_B.loop_ub_p; Cinematique_ROS_B.b_k++) {
          e->data[Cinematique_ROS_B.b_k] = limits->
            data[Cinematique_ROS_B.indicesUpperBoundViolation_size] +
            static_cast<real_T>(Cinematique_ROS_B.b_k);
        }
      } else {
        Cinematique_ROS_B.ndbl = floor((limits->
          data[Cinematique_ROS_B.indicesUpperBoundViolation_size + limits->size
          [0]] - limits->data[Cinematique_ROS_B.indicesUpperBoundViolation_size])
          + 0.5);
        Cinematique_ROS_B.apnd = limits->
          data[Cinematique_ROS_B.indicesUpperBoundViolation_size] +
          Cinematique_ROS_B.ndbl;
        Cinematique_ROS_B.cdiff = Cinematique_ROS_B.apnd - limits->
          data[Cinematique_ROS_B.indicesUpperBoundViolation_size + limits->size
          [0]];
        Cinematique_ROS_B.u0 = fabs(limits->
          data[Cinematique_ROS_B.indicesUpperBoundViolation_size]);
        Cinematique_ROS_B.u1 = fabs(limits->
          data[Cinematique_ROS_B.indicesUpperBoundViolation_size + limits->size
          [0]]);
        if ((Cinematique_ROS_B.u0 > Cinematique_ROS_B.u1) || rtIsNaN
            (Cinematique_ROS_B.u1)) {
          Cinematique_ROS_B.u1 = Cinematique_ROS_B.u0;
        }

        if (fabs(Cinematique_ROS_B.cdiff) < 4.4408920985006262E-16 *
            Cinematique_ROS_B.u1) {
          Cinematique_ROS_B.ndbl++;
          Cinematique_ROS_B.apnd = limits->
            data[Cinematique_ROS_B.indicesUpperBoundViolation_size +
            limits->size[0]];
        } else if (Cinematique_ROS_B.cdiff > 0.0) {
          Cinematique_ROS_B.apnd = (Cinematique_ROS_B.ndbl - 1.0) + limits->
            data[Cinematique_ROS_B.indicesUpperBoundViolation_size];
        } else {
          Cinematique_ROS_B.ndbl++;
        }

        if (Cinematique_ROS_B.ndbl >= 0.0) {
          Cinematique_ROS_B.loop_ub_p = static_cast<int32_T>
            (Cinematique_ROS_B.ndbl);
        } else {
          Cinematique_ROS_B.loop_ub_p = 0;
        }

        Cinematique_ROS_B.b_k = e->size[0] * e->size[1];
        e->size[0] = 1;
        e->size[1] = Cinematique_ROS_B.loop_ub_p;
        Cinema_emxEnsureCapacity_real_T(e, Cinematique_ROS_B.b_k);
        if (Cinematique_ROS_B.loop_ub_p > 0) {
          e->data[0] = limits->
            data[Cinematique_ROS_B.indicesUpperBoundViolation_size];
          if (Cinematique_ROS_B.loop_ub_p > 1) {
            e->data[Cinematique_ROS_B.loop_ub_p - 1] = Cinematique_ROS_B.apnd;
            Cinematique_ROS_B.nm1d2 = (((Cinematique_ROS_B.loop_ub_p - 1 < 0) +
              Cinematique_ROS_B.loop_ub_p) - 1) >> 1;
            Cinematique_ROS_B.c_f = Cinematique_ROS_B.nm1d2 - 2;
            for (Cinematique_ROS_B.b_k = 0; Cinematique_ROS_B.b_k <=
                 Cinematique_ROS_B.c_f; Cinematique_ROS_B.b_k++) {
              e->data[Cinematique_ROS_B.b_k + 1] = static_cast<real_T>
                (Cinematique_ROS_B.b_k + 1) + limits->
                data[Cinematique_ROS_B.indicesUpperBoundViolation_size];
              e->data[(Cinematique_ROS_B.loop_ub_p - Cinematique_ROS_B.b_k) - 2]
                = Cinematique_ROS_B.apnd - static_cast<real_T>
                (Cinematique_ROS_B.b_k + 1);
            }

            if (Cinematique_ROS_B.nm1d2 << 1 == Cinematique_ROS_B.loop_ub_p - 1)
            {
              e->data[Cinematique_ROS_B.nm1d2] = (limits->
                data[Cinematique_ROS_B.indicesUpperBoundViolation_size] +
                Cinematique_ROS_B.apnd) / 2.0;
            } else {
              e->data[Cinematique_ROS_B.nm1d2] = limits->
                data[Cinematique_ROS_B.indicesUpperBoundViolation_size] +
                static_cast<real_T>(Cinematique_ROS_B.nm1d2);
              e->data[Cinematique_ROS_B.nm1d2 + 1] = Cinematique_ROS_B.apnd -
                static_cast<real_T>(Cinematique_ROS_B.nm1d2);
            }
          }
        }
      }

      Cinematique_ROS_B.b_k = e->size[0] * e->size[1];
      Cinematique_ROS_B.loop_ub_p = Cinematique_ROS_B.b_k - 1;
      for (Cinematique_ROS_B.b_k = 0; Cinematique_ROS_B.b_k <=
           Cinematique_ROS_B.loop_ub_p; Cinematique_ROS_B.b_k++) {
        positionIndices->data[static_cast<int32_T>(Cinematique_ROS_B.bid +
          y->data[Cinematique_ROS_B.b_k]) - 1] = e->data[Cinematique_ROS_B.b_k];
      }

      Cinematique_ROS_B.bid += Cinematique_ROS_B.numPositions;
    }
  }

  Cinematique_ROS_emxFree_real_T(&y);
  Cinematique_ROS_emxFree_real_T(&e);
  Cinematique_ROS_emxFree_real_T(&limits);
  if (1.0 > Cinematique_ROS_B.bid) {
    positionIndices->size[1] = 0;
  } else {
    Cinematique_ROS_B.b_k = positionIndices->size[0] * positionIndices->size[1];
    positionIndices->size[1] = static_cast<int32_T>(Cinematique_ROS_B.bid);
    Cinema_emxEnsureCapacity_real_T(positionIndices, Cinematique_ROS_B.b_k);
  }

  Cinematique_ROS_B.loop_ub_p = positionIndices->size[0] * positionIndices->
    size[1];
  for (Cinematique_ROS_B.b_k = 0; Cinematique_ROS_B.b_k <
       Cinematique_ROS_B.loop_ub_p; Cinematique_ROS_B.b_k++) {
    initialGuess[static_cast<int32_T>(positionIndices->
      data[Cinematique_ROS_B.b_k]) - 1] = Cinematique_ROS_B.qvSolRaw[
      static_cast<int32_T>(positionIndices->data[Cinematique_ROS_B.b_k]) - 1];
  }

  Cinematique_ROS_emxFree_real_T(&positionIndices);
}

static void Cine_inverseKinematics_stepImpl(b_inverseKinematics_Cinematiq_T *obj,
  const real_T tform[16], const real_T weights[6], const real_T initialGuess[4],
  real_T QSol[4], real_T *solutionInfo_Iterations, real_T
  *solutionInfo_PoseErrorNorm, real_T *solutionInfo_ExitFlag, char_T
  solutionInfo_Status_data[], int32_T solutionInfo_Status_size[2])
{
  static const char_T tmp[5] = { 'B', 'o', 'd', 'y', '5' };

  f_robotics_manip_internal_IKE_T *args;
  memset(&Cinematique_ROS_B.weightMatrix[0], 0, 36U * sizeof(real_T));
  for (Cinematique_ROS_B.b_j = 0; Cinematique_ROS_B.b_j < 6;
       Cinematique_ROS_B.b_j++) {
    Cinematique_ROS_B.weightMatrix[Cinematique_ROS_B.b_j + 6 *
      Cinematique_ROS_B.b_j] = weights[Cinematique_ROS_B.b_j];
  }

  args = obj->Solver->ExtraArgs;
  for (Cinematique_ROS_B.b_j = 0; Cinematique_ROS_B.b_j < 36;
       Cinematique_ROS_B.b_j++) {
    args->WeightMatrix[Cinematique_ROS_B.b_j] =
      Cinematique_ROS_B.weightMatrix[Cinematique_ROS_B.b_j];
  }

  Cinematique_ROS_B.b_j = args->BodyName->size[0] * args->BodyName->size[1];
  args->BodyName->size[0] = 1;
  args->BodyName->size[1] = 5;
  Cinema_emxEnsureCapacity_char_T(args->BodyName, Cinematique_ROS_B.b_j);
  for (Cinematique_ROS_B.b_j = 0; Cinematique_ROS_B.b_j < 5;
       Cinematique_ROS_B.b_j++) {
    args->BodyName->data[Cinematique_ROS_B.b_j] = tmp[Cinematique_ROS_B.b_j];
  }

  for (Cinematique_ROS_B.b_j = 0; Cinematique_ROS_B.b_j < 16;
       Cinematique_ROS_B.b_j++) {
    args->Tform[Cinematique_ROS_B.b_j] = tform[Cinematique_ROS_B.b_j];
  }

  QSol[0] = initialGuess[0];
  QSol[1] = initialGuess[1];
  QSol[2] = initialGuess[2];
  QSol[3] = initialGuess[3];
  Cinem_inverseKinematics_solve_k(obj, QSol, &Cinematique_ROS_B.d,
    &Cinematique_ROS_B.expl_temp, &Cinematique_ROS_B.d1, &Cinematique_ROS_B.d2,
    solutionInfo_Status_data, solutionInfo_Status_size);
  *solutionInfo_ExitFlag = Cinematique_ROS_B.d2;
  *solutionInfo_PoseErrorNorm = Cinematique_ROS_B.d1;
  *solutionInfo_Iterations = Cinematique_ROS_B.d;
}

real_T rt_roundd_snf(real_T u)
{
  real_T y;
  if (fabs(u) < 4.503599627370496E+15) {
    if (u >= 0.5) {
      y = floor(u + 0.5);
    } else if (u > -0.5) {
      y = u * 0.0;
    } else {
      y = ceil(u - 0.5);
    }
  } else {
    y = u;
  }

  return y;
}

static void emxFreeStruct_c_rigidBodyJoint(c_rigidBodyJoint_Cinematique__T
  *pStruct)
{
  Cinematique_ROS_emxFree_char_T(&pStruct->Type);
  Cinematique_ROS_emxFree_real_T(&pStruct->MotionSubspace);
  Cinematique_ROS_emxFree_char_T(&pStruct->NameInternal);
  Cinematique_ROS_emxFree_real_T(&pStruct->PositionLimitsInternal);
  Cinematique_ROS_emxFree_real_T(&pStruct->HomePositionInternal);
}

static void emxFreeStruct_m_robotics_manip_(m_robotics_manip_internal_Col_T
  *pStruct)
{
  Cinemati_emxFree_unnamed_struct(&pStruct->CollisionGeometries);
}

static void emxFreeStruct_w_robotics_manip_(w_robotics_manip_internal_Rig_T
  *pStruct)
{
  Cinematique_ROS_emxFree_char_T(&pStruct->NameInternal);
  emxFreeStruct_c_rigidBodyJoint(&pStruct->JointInternal);
  emxFreeStruct_m_robotics_manip_(&pStruct->CollisionsInternal);
}

static void emxFreeMatrix_w_robotics_manip_(w_robotics_manip_internal_Rig_T
  pMatrix[5])
{
  int32_T i;
  for (i = 0; i < 5; i++) {
    emxFreeStruct_w_robotics_manip_(&pMatrix[i]);
  }
}

static void emxFreeStruct_x_robotics_manip_(x_robotics_manip_internal_Rig_T
  *pStruct)
{
  emxFreeStruct_w_robotics_manip_(&pStruct->Base);
  emxFreeMatrix_w_robotics_manip_(pStruct->_pobj0);
}

static void emxFreeStruct_f_robotics_manip_(f_robotics_manip_internal_IKE_T
  *pStruct)
{
  Cinematique_ROS_emxFree_char_T(&pStruct->BodyName);
  Cinematique_ROS_emxFree_real_T(&pStruct->ErrTemp);
  Cinematique_ROS_emxFree_real_T(&pStruct->GradTemp);
}

static void emxFreeStruct_y_robotics_manip_(y_robotics_manip_internal_Rig_T
  *pStruct)
{
  Cinematique_ROS_emxFree_char_T(&pStruct->NameInternal);
  emxFreeStruct_m_robotics_manip_(&pStruct->_pobj0);
  emxFreeStruct_c_rigidBodyJoint(&pStruct->_pobj1);
}

static void emxFreeMatrix_y_robotics_manip_(y_robotics_manip_internal_Rig_T
  pMatrix[5])
{
  int32_T i;
  for (i = 0; i < 5; i++) {
    emxFreeStruct_y_robotics_manip_(&pMatrix[i]);
  }
}

static void emxFreeStruct_ab_robotics_manip(ab_robotics_manip_internal_Ri_T
  *pStruct)
{
  emxFreeStruct_y_robotics_manip_(&pStruct->Base);
  emxFreeMatrix_y_robotics_manip_(pStruct->_pobj0);
}

static void emxFreeMatrix_m_robotics_manip_(m_robotics_manip_internal_Col_T
  pMatrix[6])
{
  int32_T i;
  for (i = 0; i < 6; i++) {
    emxFreeStruct_m_robotics_manip_(&pMatrix[i]);
  }
}

static void emxFreeMatrix_c_rigidBodyJoint(c_rigidBodyJoint_Cinematique__T
  pMatrix[5])
{
  int32_T i;
  for (i = 0; i < 5; i++) {
    emxFreeStruct_c_rigidBodyJoint(&pMatrix[i]);
  }
}

static void emxFreeStruct_h_robotics_core_i(h_robotics_core_internal_Damp_T
  *pStruct)
{
  Cinematique_ROS_emxFree_real_T(&pStruct->ConstraintMatrix);
  Cinematique_ROS_emxFree_real_T(&pStruct->ConstraintBound);
}

static void emxFreeStruct_b_inverseKinemati(b_inverseKinematics_Cinematiq_T
  *pStruct)
{
  Cinematique_ROS_emxFree_real_T(&pStruct->Limits);
  emxFreeStruct_f_robotics_manip_(&pStruct->_pobj0);
  emxFreeStruct_ab_robotics_manip(&pStruct->_pobj1);
  emxFreeMatrix_m_robotics_manip_(pStruct->_pobj2);
  emxFreeMatrix_c_rigidBodyJoint(pStruct->_pobj3);
  emxFreeMatrix_y_robotics_manip_(pStruct->_pobj4);
  emxFreeStruct_h_robotics_core_i(&pStruct->_pobj5);
}

static void emxFreeStruct_robotics_slmanip_(robotics_slmanip_internal_blo_T
  *pStruct)
{
  emxFreeStruct_x_robotics_manip_(&pStruct->TreeInternal);
  emxFreeStruct_b_inverseKinemati(&pStruct->IKInternal);
}

/* Model step function */
void Cinematique_ROS_step(void)
{
  static const char_T tmp[7] = { 's', 'u', 'c', 'c', 'e', 's', 's' };

  static const char_T tmp_0[5] = { 'f', 'i', 'x', 'e', 'd' };

  b_inverseKinematics_Cinematiq_T *obj_0;
  c_rigidBodyJoint_Cinematique__T *joint;
  emxArray_char_T_Cinematique_R_T *a;
  emxArray_real_T_Cinematique_R_T *A;
  emxArray_real_T_Cinematique_R_T *b;
  emxArray_real_T_Cinematique_R_T *e;
  emxArray_real_T_Cinematique_R_T *s;
  robotics_slmanip_internal_blo_T *obj;
  y_robotics_manip_internal_Rig_T *obj_1;
  int32_T exitg1;

  /* Outputs for Atomic SubSystem: '<Root>/Subscribe' */
  /* MATLABSystem: '<S3>/SourceBlock' */
  Cinematique_ROS_B.b_varargout_1 = Sub_Cinematique_ROS_1224.getLatestMessage
    (&Cinematique_ROS_B.BusAssignment);

  /* Outputs for Enabled SubSystem: '<Root>/Subsystem1' incorporates:
   *  EnablePort: '<S4>/Enable'
   */
  /* Outputs for Enabled SubSystem: '<S3>/Enabled Subsystem' incorporates:
   *  EnablePort: '<S17>/Enable'
   */
  if (Cinematique_ROS_B.b_varargout_1) {
    /* MATLABSystem: '<S4>/Coordinate Transformation Conversion1' */
    for (Cinematique_ROS_B.iacol_tmp = 0; Cinematique_ROS_B.iacol_tmp < 16;
         Cinematique_ROS_B.iacol_tmp++) {
      Cinematique_ROS_B.b_I[Cinematique_ROS_B.iacol_tmp] = 0;
    }

    Cinematique_ROS_B.b_I[0] = 1;
    Cinematique_ROS_B.b_I[5] = 1;
    Cinematique_ROS_B.b_I[10] = 1;
    Cinematique_ROS_B.b_I[15] = 1;
    for (Cinematique_ROS_B.b_jcol = 0; Cinematique_ROS_B.b_jcol < 4;
         Cinematique_ROS_B.b_jcol++) {
      Cinematique_ROS_B.iacol_tmp = (Cinematique_ROS_B.b_jcol << 2) - 1;
      Cinematique_ROS_B.out[Cinematique_ROS_B.iacol_tmp + 1] =
        Cinematique_ROS_B.b_I[Cinematique_ROS_B.iacol_tmp + 1];
      Cinematique_ROS_B.out[Cinematique_ROS_B.iacol_tmp + 2] =
        Cinematique_ROS_B.b_I[Cinematique_ROS_B.iacol_tmp + 2];
      Cinematique_ROS_B.out[Cinematique_ROS_B.iacol_tmp + 3] =
        Cinematique_ROS_B.b_I[Cinematique_ROS_B.iacol_tmp + 3];
      Cinematique_ROS_B.out[Cinematique_ROS_B.iacol_tmp + 4] =
        Cinematique_ROS_B.b_I[Cinematique_ROS_B.iacol_tmp + 4];
    }

    Cinematique_ROS_B.out[12] = Cinematique_ROS_B.BusAssignment.X;
    Cinematique_ROS_B.out[13] = Cinematique_ROS_B.BusAssignment.Y;
    Cinematique_ROS_B.out[14] = Cinematique_ROS_B.BusAssignment.Z;

    /* MATLABSystem: '<S18>/MATLAB System' */
    obj = &Cinematique_ROS_DW.obj;
    obj_0 = &Cinematique_ROS_DW.obj.IKInternal;
    if (Cinematique_ROS_DW.obj.IKInternal.isInitialized != 1) {
      Cinematique_ROS_emxInit_real_T(&A, 2);
      Cinematique_ROS_DW.obj.IKInternal.isSetupComplete = false;
      Cinematique_ROS_DW.obj.IKInternal.isInitialized = 1;
      Cinematique_ROS_B.n =
        Cinematique_ROS_DW.obj.IKInternal.RigidBodyTreeInternal->PositionNumber;
      Cinematique_ROS_B.iacol_tmp = A->size[0] * A->size[1];
      A->size[0] = static_cast<int32_T>(Cinematique_ROS_B.n);
      Cinematique_ROS_B.loop_ub_c = static_cast<int32_T>(2.0 *
        Cinematique_ROS_B.n);
      A->size[1] = Cinematique_ROS_B.loop_ub_c;
      Cinema_emxEnsureCapacity_real_T(A, Cinematique_ROS_B.iacol_tmp);
      Cinematique_ROS_B.loop_ub = Cinematique_ROS_B.loop_ub_c *
        static_cast<int32_T>(Cinematique_ROS_B.n) - 1;
      for (Cinematique_ROS_B.iacol_tmp = 0; Cinematique_ROS_B.iacol_tmp <=
           Cinematique_ROS_B.loop_ub; Cinematique_ROS_B.iacol_tmp++) {
        A->data[Cinematique_ROS_B.iacol_tmp] = 0.0;
      }

      Cinematique_ROS_emxInit_real_T(&b, 1);
      Cinematique_ROS_B.iacol_tmp = b->size[0];
      b->size[0] = Cinematique_ROS_B.loop_ub_c;
      Cinema_emxEnsureCapacity_real_T(b, Cinematique_ROS_B.iacol_tmp);
      for (Cinematique_ROS_B.iacol_tmp = 0; Cinematique_ROS_B.iacol_tmp <
           Cinematique_ROS_B.loop_ub_c; Cinematique_ROS_B.iacol_tmp++) {
        b->data[Cinematique_ROS_B.iacol_tmp] = 0.0;
      }

      Cinematique_ROS_B.n = 1.0;
      Cinematique_ROS_B.m = 1.0;
      Cinematique_ROS_B.pnum =
        Cinematique_ROS_DW.obj.IKInternal.RigidBodyTreeInternal->NumBodies;
      Cinematique_ROS_B.d_m = static_cast<int32_T>(Cinematique_ROS_B.pnum) - 1;
      Cinematique_ROS_emxInit_real_T(&e, 2);
      Cinematique_ROS_emxInit_real_T(&s, 2);
      Cinematique_ROS_emxInit_char_T(&a, 2);
      if (0 <= static_cast<int32_T>(Cinematique_ROS_B.pnum) - 1) {
        for (Cinematique_ROS_B.iacol_tmp = 0; Cinematique_ROS_B.iacol_tmp < 5;
             Cinematique_ROS_B.iacol_tmp++) {
          Cinematique_ROS_B.b_ff[Cinematique_ROS_B.iacol_tmp] =
            tmp_0[Cinematique_ROS_B.iacol_tmp];
        }
      }

      for (Cinematique_ROS_B.b_jcol = 0; Cinematique_ROS_B.b_jcol <=
           Cinematique_ROS_B.d_m; Cinematique_ROS_B.b_jcol++) {
        obj_1 = obj_0->RigidBodyTreeInternal->Bodies[Cinematique_ROS_B.b_jcol];
        joint = obj_1->JointInternal;
        Cinematique_ROS_B.pnum = joint->PositionNumber;
        Cinematique_ROS_B.iacol_tmp = a->size[0] * a->size[1];
        a->size[0] = 1;
        a->size[1] = joint->Type->size[1];
        Cinema_emxEnsureCapacity_char_T(a, Cinematique_ROS_B.iacol_tmp);
        Cinematique_ROS_B.loop_ub_c = joint->Type->size[0] * joint->Type->size[1]
          - 1;
        for (Cinematique_ROS_B.iacol_tmp = 0; Cinematique_ROS_B.iacol_tmp <=
             Cinematique_ROS_B.loop_ub_c; Cinematique_ROS_B.iacol_tmp++) {
          a->data[Cinematique_ROS_B.iacol_tmp] = joint->Type->
            data[Cinematique_ROS_B.iacol_tmp];
        }

        Cinematique_ROS_B.b_varargout_1 = false;
        if (a->size[1] == 5) {
          Cinematique_ROS_B.iacol_tmp = 1;
          do {
            exitg1 = 0;
            if (Cinematique_ROS_B.iacol_tmp - 1 < 5) {
              if (a->data[Cinematique_ROS_B.iacol_tmp - 1] !=
                  Cinematique_ROS_B.b_ff[Cinematique_ROS_B.iacol_tmp - 1]) {
                exitg1 = 1;
              } else {
                Cinematique_ROS_B.iacol_tmp++;
              }
            } else {
              Cinematique_ROS_B.b_varargout_1 = true;
              exitg1 = 1;
            }
          } while (exitg1 == 0);
        }

        if (!Cinematique_ROS_B.b_varargout_1) {
          Cinematique_ROS_B.fallValLimit = (Cinematique_ROS_B.n +
            Cinematique_ROS_B.pnum) - 1.0;
          if (Cinematique_ROS_B.n > Cinematique_ROS_B.fallValLimit) {
            Cinematique_ROS_B.j = 0;
          } else {
            Cinematique_ROS_B.j = static_cast<int32_T>(Cinematique_ROS_B.n) - 1;
          }

          Cinematique_ROS_B.w = Cinematique_ROS_B.m + Cinematique_ROS_B.pnum;
          if (Cinematique_ROS_B.m > Cinematique_ROS_B.w - 1.0) {
            Cinematique_ROS_B.p = 0;
          } else {
            Cinematique_ROS_B.p = static_cast<int32_T>(Cinematique_ROS_B.m) - 1;
          }

          if (Cinematique_ROS_B.pnum < 0.0) {
            Cinematique_ROS_B.t = 0.0;
            Cinematique_ROS_B.pnum_j = 0.0;
          } else {
            Cinematique_ROS_B.t = Cinematique_ROS_B.pnum;
            Cinematique_ROS_B.pnum_j = Cinematique_ROS_B.pnum;
          }

          Cinematique_ROS_B.m_m = static_cast<int32_T>(Cinematique_ROS_B.pnum_j)
            - 1;
          Cinematique_ROS_B.iacol_tmp = s->size[0] * s->size[1];
          s->size[0] = static_cast<int32_T>(Cinematique_ROS_B.t);
          s->size[1] = static_cast<int32_T>(Cinematique_ROS_B.t);
          Cinema_emxEnsureCapacity_real_T(s, Cinematique_ROS_B.iacol_tmp);
          Cinematique_ROS_B.loop_ub_c = static_cast<int32_T>(Cinematique_ROS_B.t)
            * static_cast<int32_T>(Cinematique_ROS_B.t) - 1;
          for (Cinematique_ROS_B.iacol_tmp = 0; Cinematique_ROS_B.iacol_tmp <=
               Cinematique_ROS_B.loop_ub_c; Cinematique_ROS_B.iacol_tmp++) {
            s->data[Cinematique_ROS_B.iacol_tmp] = 0.0;
          }

          if (static_cast<int32_T>(Cinematique_ROS_B.t) > 0) {
            for (Cinematique_ROS_B.iacol_tmp = 0; Cinematique_ROS_B.iacol_tmp <=
                 Cinematique_ROS_B.m_m; Cinematique_ROS_B.iacol_tmp++) {
              s->data[Cinematique_ROS_B.iacol_tmp + s->size[0] *
                Cinematique_ROS_B.iacol_tmp] = 1.0;
            }
          }

          Cinematique_ROS_B.loop_ub_c = s->size[1];
          for (Cinematique_ROS_B.iacol_tmp = 0; Cinematique_ROS_B.iacol_tmp <
               Cinematique_ROS_B.loop_ub_c; Cinematique_ROS_B.iacol_tmp++) {
            Cinematique_ROS_B.loop_ub = s->size[0];
            for (Cinematique_ROS_B.m_m = 0; Cinematique_ROS_B.m_m <
                 Cinematique_ROS_B.loop_ub; Cinematique_ROS_B.m_m++) {
              A->data[(Cinematique_ROS_B.j + Cinematique_ROS_B.m_m) + A->size[0]
                * (Cinematique_ROS_B.p + Cinematique_ROS_B.iacol_tmp)] = s->
                data[s->size[0] * Cinematique_ROS_B.iacol_tmp +
                Cinematique_ROS_B.m_m];
            }
          }

          if (Cinematique_ROS_B.n > Cinematique_ROS_B.fallValLimit) {
            Cinematique_ROS_B.j = 0;
          } else {
            Cinematique_ROS_B.j = static_cast<int32_T>(Cinematique_ROS_B.n) - 1;
          }

          Cinematique_ROS_B.fallValLimit = 2.0 * Cinematique_ROS_B.pnum +
            Cinematique_ROS_B.m;
          if (Cinematique_ROS_B.w > Cinematique_ROS_B.fallValLimit - 1.0) {
            Cinematique_ROS_B.p = 0;
          } else {
            Cinematique_ROS_B.p = static_cast<int32_T>(Cinematique_ROS_B.w) - 1;
          }

          if (Cinematique_ROS_B.pnum < 0.0) {
            Cinematique_ROS_B.t = 0.0;
            Cinematique_ROS_B.pnum_j = 0.0;
          } else {
            Cinematique_ROS_B.t = Cinematique_ROS_B.pnum;
            Cinematique_ROS_B.pnum_j = Cinematique_ROS_B.pnum;
          }

          Cinematique_ROS_B.m_m = static_cast<int32_T>(Cinematique_ROS_B.pnum_j)
            - 1;
          Cinematique_ROS_B.iacol_tmp = s->size[0] * s->size[1];
          s->size[0] = static_cast<int32_T>(Cinematique_ROS_B.t);
          s->size[1] = static_cast<int32_T>(Cinematique_ROS_B.t);
          Cinema_emxEnsureCapacity_real_T(s, Cinematique_ROS_B.iacol_tmp);
          Cinematique_ROS_B.loop_ub_c = static_cast<int32_T>(Cinematique_ROS_B.t)
            * static_cast<int32_T>(Cinematique_ROS_B.t) - 1;
          for (Cinematique_ROS_B.iacol_tmp = 0; Cinematique_ROS_B.iacol_tmp <=
               Cinematique_ROS_B.loop_ub_c; Cinematique_ROS_B.iacol_tmp++) {
            s->data[Cinematique_ROS_B.iacol_tmp] = 0.0;
          }

          if (static_cast<int32_T>(Cinematique_ROS_B.t) > 0) {
            for (Cinematique_ROS_B.iacol_tmp = 0; Cinematique_ROS_B.iacol_tmp <=
                 Cinematique_ROS_B.m_m; Cinematique_ROS_B.iacol_tmp++) {
              s->data[Cinematique_ROS_B.iacol_tmp + s->size[0] *
                Cinematique_ROS_B.iacol_tmp] = 1.0;
            }
          }

          Cinematique_ROS_B.loop_ub_c = s->size[1];
          for (Cinematique_ROS_B.iacol_tmp = 0; Cinematique_ROS_B.iacol_tmp <
               Cinematique_ROS_B.loop_ub_c; Cinematique_ROS_B.iacol_tmp++) {
            Cinematique_ROS_B.loop_ub = s->size[0];
            for (Cinematique_ROS_B.m_m = 0; Cinematique_ROS_B.m_m <
                 Cinematique_ROS_B.loop_ub; Cinematique_ROS_B.m_m++) {
              A->data[(Cinematique_ROS_B.j + Cinematique_ROS_B.m_m) + A->size[0]
                * (Cinematique_ROS_B.p + Cinematique_ROS_B.iacol_tmp)] =
                -s->data[s->size[0] * Cinematique_ROS_B.iacol_tmp +
                Cinematique_ROS_B.m_m];
            }
          }

          Cinematique_ROS_B.iacol_tmp = e->size[0] * e->size[1];
          e->size[0] = joint->PositionLimitsInternal->size[0];
          e->size[1] = 2;
          Cinema_emxEnsureCapacity_real_T(e, Cinematique_ROS_B.iacol_tmp);
          Cinematique_ROS_B.loop_ub_c = joint->PositionLimitsInternal->size[0] *
            joint->PositionLimitsInternal->size[1] - 1;
          for (Cinematique_ROS_B.iacol_tmp = 0; Cinematique_ROS_B.iacol_tmp <=
               Cinematique_ROS_B.loop_ub_c; Cinematique_ROS_B.iacol_tmp++) {
            e->data[Cinematique_ROS_B.iacol_tmp] = joint->
              PositionLimitsInternal->data[Cinematique_ROS_B.iacol_tmp];
          }

          b->data[static_cast<int32_T>(Cinematique_ROS_B.m) - 1] = e->data[1];
          Cinematique_ROS_B.iacol_tmp = e->size[0] * e->size[1];
          e->size[0] = joint->PositionLimitsInternal->size[0];
          e->size[1] = 2;
          Cinema_emxEnsureCapacity_real_T(e, Cinematique_ROS_B.iacol_tmp);
          Cinematique_ROS_B.loop_ub_c = joint->PositionLimitsInternal->size[0] *
            joint->PositionLimitsInternal->size[1] - 1;
          for (Cinematique_ROS_B.iacol_tmp = 0; Cinematique_ROS_B.iacol_tmp <=
               Cinematique_ROS_B.loop_ub_c; Cinematique_ROS_B.iacol_tmp++) {
            e->data[Cinematique_ROS_B.iacol_tmp] = joint->
              PositionLimitsInternal->data[Cinematique_ROS_B.iacol_tmp];
          }

          b->data[static_cast<int32_T>(Cinematique_ROS_B.m + 1.0) - 1] =
            -e->data[0];
          Cinematique_ROS_B.m = Cinematique_ROS_B.fallValLimit;
        }

        Cinematique_ROS_B.n += Cinematique_ROS_B.pnum;
      }

      Cinematique_ROS_emxFree_char_T(&a);
      Cinematique_ROS_emxFree_real_T(&s);
      Cinematique_ROS_emxFree_real_T(&e);
      Cinematique_ROS_B.iacol_tmp = A->size[0] * A->size[1];
      Cinematique_ROS_B.m_m = obj->IKInternal.Solver->ConstraintMatrix->size[0] *
        obj->IKInternal.Solver->ConstraintMatrix->size[1];
      obj->IKInternal.Solver->ConstraintMatrix->size[0] = A->size[0];
      obj->IKInternal.Solver->ConstraintMatrix->size[1] = A->size[1];
      Cinema_emxEnsureCapacity_real_T(obj->IKInternal.Solver->ConstraintMatrix,
        Cinematique_ROS_B.m_m);
      Cinematique_ROS_B.loop_ub_c = Cinematique_ROS_B.iacol_tmp - 1;
      for (Cinematique_ROS_B.iacol_tmp = 0; Cinematique_ROS_B.iacol_tmp <=
           Cinematique_ROS_B.loop_ub_c; Cinematique_ROS_B.iacol_tmp++) {
        obj->IKInternal.Solver->ConstraintMatrix->
          data[Cinematique_ROS_B.iacol_tmp] = A->
          data[Cinematique_ROS_B.iacol_tmp];
      }

      Cinematique_ROS_emxFree_real_T(&A);
      Cinematique_ROS_B.iacol_tmp = obj->IKInternal.Solver->
        ConstraintBound->size[0];
      obj->IKInternal.Solver->ConstraintBound->size[0] = b->size[0];
      Cinema_emxEnsureCapacity_real_T(obj->IKInternal.Solver->ConstraintBound,
        Cinematique_ROS_B.iacol_tmp);
      Cinematique_ROS_B.loop_ub_c = b->size[0];
      for (Cinematique_ROS_B.iacol_tmp = 0; Cinematique_ROS_B.iacol_tmp <
           Cinematique_ROS_B.loop_ub_c; Cinematique_ROS_B.iacol_tmp++) {
        obj->IKInternal.Solver->ConstraintBound->
          data[Cinematique_ROS_B.iacol_tmp] = b->
          data[Cinematique_ROS_B.iacol_tmp];
      }

      RigidBodyTree_get_JointPosition(obj->IKInternal.RigidBodyTreeInternal,
        obj->IKInternal.Limits);
      obj->IKInternal.Solver->ExtraArgs = &obj_0->_pobj0;
      for (Cinematique_ROS_B.iacol_tmp = 0; Cinematique_ROS_B.iacol_tmp < 36;
           Cinematique_ROS_B.iacol_tmp++) {
        obj->IKInternal.Solver->ExtraArgs->
          WeightMatrix[Cinematique_ROS_B.iacol_tmp] = 0.0;
      }

      obj->IKInternal.Solver->ExtraArgs->Robot =
        obj->IKInternal.RigidBodyTreeInternal;
      for (Cinematique_ROS_B.iacol_tmp = 0; Cinematique_ROS_B.iacol_tmp < 16;
           Cinematique_ROS_B.iacol_tmp++) {
        Cinematique_ROS_B.b_I[Cinematique_ROS_B.iacol_tmp] = 0;
      }

      Cinematique_ROS_B.b_I[0] = 1;
      Cinematique_ROS_B.b_I[5] = 1;
      Cinematique_ROS_B.b_I[10] = 1;
      Cinematique_ROS_B.b_I[15] = 1;
      for (Cinematique_ROS_B.iacol_tmp = 0; Cinematique_ROS_B.iacol_tmp < 16;
           Cinematique_ROS_B.iacol_tmp++) {
        obj->IKInternal.Solver->ExtraArgs->Tform[Cinematique_ROS_B.iacol_tmp] =
          Cinematique_ROS_B.b_I[Cinematique_ROS_B.iacol_tmp];
      }

      obj->IKInternal.Solver->ExtraArgs->BodyName->size[0] = 1;
      obj->IKInternal.Solver->ExtraArgs->BodyName->size[1] = 0;
      Cinematique_ROS_B.iacol_tmp = obj->IKInternal.Solver->ExtraArgs->
        ErrTemp->size[0];
      obj->IKInternal.Solver->ExtraArgs->ErrTemp->size[0] = 6;
      Cinema_emxEnsureCapacity_real_T(obj->IKInternal.Solver->ExtraArgs->ErrTemp,
        Cinematique_ROS_B.iacol_tmp);
      for (Cinematique_ROS_B.iacol_tmp = 0; Cinematique_ROS_B.iacol_tmp < 6;
           Cinematique_ROS_B.iacol_tmp++) {
        obj->IKInternal.Solver->ExtraArgs->ErrTemp->
          data[Cinematique_ROS_B.iacol_tmp] = 0.0;
      }

      obj->IKInternal.Solver->ExtraArgs->CostTemp = 0.0;
      Cinematique_ROS_B.iacol_tmp = b->size[0];
      b->size[0] = static_cast<int32_T>(obj->
        IKInternal.RigidBodyTreeInternal->PositionNumber);
      Cinema_emxEnsureCapacity_real_T(b, Cinematique_ROS_B.iacol_tmp);
      Cinematique_ROS_B.loop_ub_c = static_cast<int32_T>
        (obj->IKInternal.RigidBodyTreeInternal->PositionNumber);
      for (Cinematique_ROS_B.iacol_tmp = 0; Cinematique_ROS_B.iacol_tmp <
           Cinematique_ROS_B.loop_ub_c; Cinematique_ROS_B.iacol_tmp++) {
        b->data[Cinematique_ROS_B.iacol_tmp] = 0.0;
      }

      Cinematique_ROS_B.iacol_tmp = obj->IKInternal.Solver->ExtraArgs->
        GradTemp->size[0];
      obj->IKInternal.Solver->ExtraArgs->GradTemp->size[0] = b->size[0];
      Cinema_emxEnsureCapacity_real_T(obj->IKInternal.Solver->
        ExtraArgs->GradTemp, Cinematique_ROS_B.iacol_tmp);
      Cinematique_ROS_B.loop_ub_c = b->size[0];
      for (Cinematique_ROS_B.iacol_tmp = 0; Cinematique_ROS_B.iacol_tmp <
           Cinematique_ROS_B.loop_ub_c; Cinematique_ROS_B.iacol_tmp++) {
        obj->IKInternal.Solver->ExtraArgs->GradTemp->
          data[Cinematique_ROS_B.iacol_tmp] = b->
          data[Cinematique_ROS_B.iacol_tmp];
      }

      Cinematique_ROS_emxFree_real_T(&b);
      obj->IKInternal.isSetupComplete = true;
    }

    /* MATLABSystem: '<S18>/MATLAB System' incorporates:
     *  Constant: '<S4>/Constant1'
     *  Delay: '<S4>/Delay'
     *  MATLABSystem: '<S4>/Coordinate Transformation Conversion1'
     */
    Cine_inverseKinematics_stepImpl(&obj->IKInternal, Cinematique_ROS_B.out,
      Cinematique_ROS_P.Constant1_Value, Cinematique_ROS_DW.Delay_DSTATE,
      Cinematique_ROS_B.MATLABSystem_o1,
      &Cinematique_ROS_B.MATLABSystem_o2.Iterations,
      &Cinematique_ROS_B.MATLABSystem_o2.PoseErrorNorm, &Cinematique_ROS_B.n,
      Cinematique_ROS_B.b_varargout_2_Status_data,
      Cinematique_ROS_B.b_varargout_2_Status_size);

    /* MATLABSystem: '<S18>/MATLAB System' */
    for (Cinematique_ROS_B.iacol_tmp = 0; Cinematique_ROS_B.iacol_tmp < 7;
         Cinematique_ROS_B.iacol_tmp++) {
      Cinematique_ROS_B.b_f[Cinematique_ROS_B.iacol_tmp] =
        tmp[Cinematique_ROS_B.iacol_tmp];
    }

    Cinematique_ROS_B.b_varargout_1 = false;
    if (Cinematique_ROS_B.b_varargout_2_Status_size[1] == 7) {
      Cinematique_ROS_B.iacol_tmp = 1;
      do {
        exitg1 = 0;
        if (Cinematique_ROS_B.iacol_tmp - 1 < 7) {
          if (Cinematique_ROS_B.b_varargout_2_Status_data[Cinematique_ROS_B.iacol_tmp
              - 1] != Cinematique_ROS_B.b_f[Cinematique_ROS_B.iacol_tmp - 1]) {
            exitg1 = 1;
          } else {
            Cinematique_ROS_B.iacol_tmp++;
          }
        } else {
          Cinematique_ROS_B.b_varargout_1 = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    Cinematique_ROS_B.fallValLimit = rt_roundd_snf(Cinematique_ROS_B.n);
    if (Cinematique_ROS_B.fallValLimit < 65536.0) {
      if (Cinematique_ROS_B.fallValLimit >= 0.0) {
        /* MATLABSystem: '<S18>/MATLAB System' */
        Cinematique_ROS_B.MATLABSystem_o2.ExitFlag = static_cast<uint16_T>
          (Cinematique_ROS_B.fallValLimit);
      } else {
        /* MATLABSystem: '<S18>/MATLAB System' */
        Cinematique_ROS_B.MATLABSystem_o2.ExitFlag = 0U;
      }
    } else {
      /* MATLABSystem: '<S18>/MATLAB System' */
      Cinematique_ROS_B.MATLABSystem_o2.ExitFlag = MAX_uint16_T;
    }

    if (Cinematique_ROS_B.b_varargout_1) {
      /* MATLABSystem: '<S18>/MATLAB System' */
      Cinematique_ROS_B.MATLABSystem_o2.Status = 1U;
    } else {
      /* MATLABSystem: '<S18>/MATLAB System' */
      Cinematique_ROS_B.MATLABSystem_o2.Status = 2U;
    }

    /* Update for Delay: '<S4>/Delay' incorporates:
     *  MATLABSystem: '<S18>/MATLAB System'
     */
    Cinematique_ROS_DW.Delay_DSTATE[0] = Cinematique_ROS_B.MATLABSystem_o1[0];
    Cinematique_ROS_DW.Delay_DSTATE[1] = Cinematique_ROS_B.MATLABSystem_o1[1];
    Cinematique_ROS_DW.Delay_DSTATE[2] = Cinematique_ROS_B.MATLABSystem_o1[2];
    Cinematique_ROS_DW.Delay_DSTATE[3] = Cinematique_ROS_B.MATLABSystem_o1[3];
  }

  /* End of MATLABSystem: '<S3>/SourceBlock' */
  /* End of Outputs for SubSystem: '<S3>/Enabled Subsystem' */
  /* End of Outputs for SubSystem: '<Root>/Subsystem1' */
  /* End of Outputs for SubSystem: '<Root>/Subscribe' */

  /* Gain: '<Root>/Gain' incorporates:
   *  MATLABSystem: '<S18>/MATLAB System'
   */
  Cinematique_ROS_B.Gain[0] = Cinematique_ROS_P.Gain_Gain *
    Cinematique_ROS_B.MATLABSystem_o1[0];
  Cinematique_ROS_B.Gain[1] = Cinematique_ROS_P.Gain_Gain *
    Cinematique_ROS_B.MATLABSystem_o1[1];
  Cinematique_ROS_B.Gain[2] = Cinematique_ROS_P.Gain_Gain *
    Cinematique_ROS_B.MATLABSystem_o1[2];
  Cinematique_ROS_B.Gain[3] = Cinematique_ROS_P.Gain_Gain *
    Cinematique_ROS_B.MATLABSystem_o1[3];

  /* RateLimiter: '<Root>/Rate Limiter' incorporates:
   *  Gain: '<Root>/Gain'
   */
  if (Cinematique_ROS_DW.LastMajorTime == (rtInf)) {
    /* RateLimiter: '<Root>/Rate Limiter' incorporates:
     *  Gain: '<Root>/Gain'
     */
    Cinematique_ROS_B.RateLimiter[0] = Cinematique_ROS_B.Gain[0];
    Cinematique_ROS_B.RateLimiter[1] = Cinematique_ROS_B.Gain[1];
    Cinematique_ROS_B.RateLimiter[2] = Cinematique_ROS_B.Gain[2];
    Cinematique_ROS_B.RateLimiter[3] = Cinematique_ROS_B.Gain[3];
  } else {
    Cinematique_ROS_B.n = Cinematique_ROS_M->Timing.t[0] -
      Cinematique_ROS_DW.LastMajorTime;
    Cinematique_ROS_B.m = Cinematique_ROS_B.n *
      Cinematique_ROS_P.RateLimiter_RisingLim;
    Cinematique_ROS_B.pnum = Cinematique_ROS_B.Gain[0] -
      Cinematique_ROS_DW.PrevY[0];
    if (Cinematique_ROS_B.pnum > Cinematique_ROS_B.m) {
      /* RateLimiter: '<Root>/Rate Limiter' */
      Cinematique_ROS_B.RateLimiter[0] = Cinematique_ROS_DW.PrevY[0] +
        Cinematique_ROS_B.m;
    } else {
      Cinematique_ROS_B.fallValLimit = Cinematique_ROS_B.n *
        Cinematique_ROS_P.RateLimiter_FallingLim;
      if (Cinematique_ROS_B.pnum < Cinematique_ROS_B.fallValLimit) {
        /* RateLimiter: '<Root>/Rate Limiter' */
        Cinematique_ROS_B.RateLimiter[0] = Cinematique_ROS_DW.PrevY[0] +
          Cinematique_ROS_B.fallValLimit;
      } else {
        /* RateLimiter: '<Root>/Rate Limiter' */
        Cinematique_ROS_B.RateLimiter[0] = Cinematique_ROS_B.Gain[0];
      }
    }

    Cinematique_ROS_B.pnum = Cinematique_ROS_B.Gain[1] -
      Cinematique_ROS_DW.PrevY[1];
    if (Cinematique_ROS_B.pnum > Cinematique_ROS_B.m) {
      /* RateLimiter: '<Root>/Rate Limiter' */
      Cinematique_ROS_B.RateLimiter[1] = Cinematique_ROS_DW.PrevY[1] +
        Cinematique_ROS_B.m;
    } else {
      Cinematique_ROS_B.fallValLimit = Cinematique_ROS_B.n *
        Cinematique_ROS_P.RateLimiter_FallingLim;
      if (Cinematique_ROS_B.pnum < Cinematique_ROS_B.fallValLimit) {
        /* RateLimiter: '<Root>/Rate Limiter' */
        Cinematique_ROS_B.RateLimiter[1] = Cinematique_ROS_DW.PrevY[1] +
          Cinematique_ROS_B.fallValLimit;
      } else {
        /* RateLimiter: '<Root>/Rate Limiter' */
        Cinematique_ROS_B.RateLimiter[1] = Cinematique_ROS_B.Gain[1];
      }
    }

    Cinematique_ROS_B.pnum = Cinematique_ROS_B.Gain[2] -
      Cinematique_ROS_DW.PrevY[2];
    if (Cinematique_ROS_B.pnum > Cinematique_ROS_B.m) {
      /* RateLimiter: '<Root>/Rate Limiter' */
      Cinematique_ROS_B.RateLimiter[2] = Cinematique_ROS_DW.PrevY[2] +
        Cinematique_ROS_B.m;
    } else {
      Cinematique_ROS_B.fallValLimit = Cinematique_ROS_B.n *
        Cinematique_ROS_P.RateLimiter_FallingLim;
      if (Cinematique_ROS_B.pnum < Cinematique_ROS_B.fallValLimit) {
        /* RateLimiter: '<Root>/Rate Limiter' */
        Cinematique_ROS_B.RateLimiter[2] = Cinematique_ROS_DW.PrevY[2] +
          Cinematique_ROS_B.fallValLimit;
      } else {
        /* RateLimiter: '<Root>/Rate Limiter' */
        Cinematique_ROS_B.RateLimiter[2] = Cinematique_ROS_B.Gain[2];
      }
    }

    Cinematique_ROS_B.pnum = Cinematique_ROS_B.Gain[3] -
      Cinematique_ROS_DW.PrevY[3];
    if (Cinematique_ROS_B.pnum > Cinematique_ROS_B.m) {
      /* RateLimiter: '<Root>/Rate Limiter' */
      Cinematique_ROS_B.RateLimiter[3] = Cinematique_ROS_DW.PrevY[3] +
        Cinematique_ROS_B.m;
    } else {
      Cinematique_ROS_B.fallValLimit = Cinematique_ROS_B.n *
        Cinematique_ROS_P.RateLimiter_FallingLim;
      if (Cinematique_ROS_B.pnum < Cinematique_ROS_B.fallValLimit) {
        /* RateLimiter: '<Root>/Rate Limiter' */
        Cinematique_ROS_B.RateLimiter[3] = Cinematique_ROS_DW.PrevY[3] +
          Cinematique_ROS_B.fallValLimit;
      } else {
        /* RateLimiter: '<Root>/Rate Limiter' */
        Cinematique_ROS_B.RateLimiter[3] = Cinematique_ROS_B.Gain[3];
      }
    }
  }

  /* End of RateLimiter: '<Root>/Rate Limiter' */

  /* RelationalOperator: '<S1>/Compare' incorporates:
   *  Constant: '<S1>/Constant'
   */
  Cinematique_ROS_B.Compare = (Cinematique_ROS_B.MATLABSystem_o2.ExitFlag ==
    Cinematique_ROS_P.CompareToConstant_const);

  /* Outputs for Enabled SubSystem: '<Root>/Enabled Subsystem' incorporates:
   *  EnablePort: '<S2>/Enable'
   */
  Cinematique_ROS_DW.EnabledSubsystem_MODE = Cinematique_ROS_B.Compare;
  if (Cinematique_ROS_DW.EnabledSubsystem_MODE) {
    /* BusAssignment: '<S5>/Bus Assignment' incorporates:
     *  Constant: '<S9>/Constant'
     */
    Cinematique_ROS_B.BusAssignment = Cinematique_ROS_P.Constant_Value;
    Cinematique_ROS_B.BusAssignment.X = Cinematique_ROS_B.RateLimiter[0];

    /* Outputs for Atomic SubSystem: '<S5>/Publish' */
    /* MATLABSystem: '<S10>/SinkBlock' */
    Pub_Cinematique_ROS_1271.publish(&Cinematique_ROS_B.BusAssignment);

    /* End of Outputs for SubSystem: '<S5>/Publish' */

    /* BusAssignment: '<S6>/Bus Assignment' incorporates:
     *  Constant: '<S11>/Constant'
     */
    Cinematique_ROS_B.BusAssignment = Cinematique_ROS_P.Constant_Value_l;
    Cinematique_ROS_B.BusAssignment.X = Cinematique_ROS_B.RateLimiter[2];

    /* Outputs for Atomic SubSystem: '<S6>/Publish' */
    /* MATLABSystem: '<S12>/SinkBlock' */
    Pub_Cinematique_ROS_1319.publish(&Cinematique_ROS_B.BusAssignment);

    /* End of Outputs for SubSystem: '<S6>/Publish' */

    /* BusAssignment: '<S7>/Bus Assignment' incorporates:
     *  Constant: '<S13>/Constant'
     */
    Cinematique_ROS_B.BusAssignment = Cinematique_ROS_P.Constant_Value_lm;
    Cinematique_ROS_B.BusAssignment.X = Cinematique_ROS_B.RateLimiter[1];

    /* Outputs for Atomic SubSystem: '<S7>/Publish' */
    /* MATLABSystem: '<S14>/SinkBlock' */
    Pub_Cinematique_ROS_1314.publish(&Cinematique_ROS_B.BusAssignment);

    /* End of Outputs for SubSystem: '<S7>/Publish' */

    /* BusAssignment: '<S8>/Bus Assignment' incorporates:
     *  Constant: '<S15>/Constant'
     */
    Cinematique_ROS_B.BusAssignment = Cinematique_ROS_P.Constant_Value_n;
    Cinematique_ROS_B.BusAssignment.X = Cinematique_ROS_B.RateLimiter[3];

    /* Outputs for Atomic SubSystem: '<S8>/Publish' */
    /* MATLABSystem: '<S16>/SinkBlock' */
    Pub_Cinematique_ROS_1324.publish(&Cinematique_ROS_B.BusAssignment);

    /* End of Outputs for SubSystem: '<S8>/Publish' */
  }

  /* End of Outputs for SubSystem: '<Root>/Enabled Subsystem' */

  /* Update for RateLimiter: '<Root>/Rate Limiter' */
  Cinematique_ROS_DW.PrevY[0] = Cinematique_ROS_B.RateLimiter[0];
  Cinematique_ROS_DW.PrevY[1] = Cinematique_ROS_B.RateLimiter[1];
  Cinematique_ROS_DW.PrevY[2] = Cinematique_ROS_B.RateLimiter[2];
  Cinematique_ROS_DW.PrevY[3] = Cinematique_ROS_B.RateLimiter[3];
  Cinematique_ROS_DW.LastMajorTime = Cinematique_ROS_M->Timing.t[0];

  /* Update absolute time for base rate */
  /* The "clockTick0" counts the number of times the code of this task has
   * been executed. The absolute time is the multiplication of "clockTick0"
   * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
   * overflow during the application lifespan selected.
   * Timer of this task consists of two 32 bit unsigned integers.
   * The two integers represent the low bits Timing.clockTick0 and the high bits
   * Timing.clockTickH0. When the low bit overflows to 0, the high bits increment.
   */
  if (!(++Cinematique_ROS_M->Timing.clockTick0)) {
    ++Cinematique_ROS_M->Timing.clockTickH0;
  }

  Cinematique_ROS_M->Timing.t[0] = Cinematique_ROS_M->Timing.clockTick0 *
    Cinematique_ROS_M->Timing.stepSize0 + Cinematique_ROS_M->Timing.clockTickH0 *
    Cinematique_ROS_M->Timing.stepSize0 * 4294967296.0;

  {
    /* Update absolute timer for sample time: [0.05s, 0.0s] */
    /* The "clockTick1" counts the number of times the code of this task has
     * been executed. The resolution of this integer timer is 0.05, which is the step size
     * of the task. Size of "clockTick1" ensures timer will not overflow during the
     * application lifespan selected.
     * Timer of this task consists of two 32 bit unsigned integers.
     * The two integers represent the low bits Timing.clockTick1 and the high bits
     * Timing.clockTickH1. When the low bit overflows to 0, the high bits increment.
     */
    Cinematique_ROS_M->Timing.clockTick1++;
    if (!Cinematique_ROS_M->Timing.clockTick1) {
      Cinematique_ROS_M->Timing.clockTickH1++;
    }
  }
}

/* Model initialize function */
void Cinematique_ROS_initialize(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  {
    /* Setup solver object */
    rtsiSetSimTimeStepPtr(&Cinematique_ROS_M->solverInfo,
                          &Cinematique_ROS_M->Timing.simTimeStep);
    rtsiSetTPtr(&Cinematique_ROS_M->solverInfo, &rtmGetTPtr(Cinematique_ROS_M));
    rtsiSetStepSizePtr(&Cinematique_ROS_M->solverInfo,
                       &Cinematique_ROS_M->Timing.stepSize0);
    rtsiSetErrorStatusPtr(&Cinematique_ROS_M->solverInfo, (&rtmGetErrorStatus
      (Cinematique_ROS_M)));
    rtsiSetRTModelPtr(&Cinematique_ROS_M->solverInfo, Cinematique_ROS_M);
  }

  rtsiSetSimTimeStep(&Cinematique_ROS_M->solverInfo, MAJOR_TIME_STEP);
  rtsiSetSolverName(&Cinematique_ROS_M->solverInfo,"FixedStepDiscrete");
  rtmSetTPtr(Cinematique_ROS_M, &Cinematique_ROS_M->Timing.tArray[0]);
  Cinematique_ROS_M->Timing.stepSize0 = 0.05;

  /* block I/O */
  (void) memset((static_cast<void *>(&Cinematique_ROS_B)), 0,
                sizeof(B_Cinematique_ROS_T));

  /* states (dwork) */
  (void) memset(static_cast<void *>(&Cinematique_ROS_DW), 0,
                sizeof(DW_Cinematique_ROS_T));

  {
    static const char_T tmp[9] = { '/', 'P', 'o', 's', 'i', 't', 'i', 'o', 'n' };

    static const char_T tmp_3[8] = { '/', 'P', 'o', 'i', 'g', 'n', 'e', 't' };

    static const char_T tmp_0[7] = { '/', 'B', 'a', 's', 's', 'i', 'n' };

    static const char_T tmp_2[7] = { '/', 'E', 'p', 'a', 'u', 'l', 'e' };

    static const char_T tmp_1[6] = { '/', 'C', 'o', 'u', 'd', 'e' };

    /* Start for Atomic SubSystem: '<Root>/Subscribe' */
    /* Start for MATLABSystem: '<S3>/SourceBlock' */
    Cinematique_ROS_DW.obj_ez.matlabCodegenIsDeleted = false;
    Cinematique_ROS_DW.objisempty_f = true;
    Cinematique_ROS_DW.obj_ez.isInitialized = 1;
    for (Cinematique_ROS_B.b_mti = 0; Cinematique_ROS_B.b_mti < 9;
         Cinematique_ROS_B.b_mti++) {
      Cinematique_ROS_B.b_zeroDelimTopic[Cinematique_ROS_B.b_mti] =
        tmp[Cinematique_ROS_B.b_mti];
    }

    Cinematique_ROS_B.b_zeroDelimTopic[9] = '\x00';
    Sub_Cinematique_ROS_1224.createSubscriber
      (&Cinematique_ROS_B.b_zeroDelimTopic[0], 1);
    Cinematique_ROS_DW.obj_ez.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<S3>/SourceBlock' */
    /* End of Start for SubSystem: '<Root>/Subscribe' */

    /* Start for Enabled SubSystem: '<Root>/Subsystem1' */
    /* Start for MATLABSystem: '<S4>/Coordinate Transformation Conversion1' */
    Cinematique_ROS_DW.objisempty_k = true;
    Cinematique_ROS_DW.obj_g.isInitialized = 1;

    /* End of Start for SubSystem: '<Root>/Subsystem1' */
    emxInitStruct_robotics_slmanip_(&Cinematique_ROS_DW.obj);

    /* Start for Enabled SubSystem: '<Root>/Subsystem1' */
    /* Start for MATLABSystem: '<S18>/MATLAB System' */
    Cinematique_ROS_DW.obj.IKInternal.matlabCodegenIsDeleted = true;
    Cinematique_ROS_DW.obj.matlabCodegenIsDeleted = true;
    Cinematique_ROS_DW.method_o = 7U;
    Cinematique_ROS_DW.method_not_empty = true;
    Cinematique_ROS_DW.state = 1144108930U;
    Cinematique_ROS_DW.state_not_empty = true;
    Cinematique_ROS_DW.state_f[0] = 362436069U;
    Cinematique_ROS_DW.state_f[1] = 521288629U;
    Cinematique_ROS_DW.state_not_empty_ml = true;
    memset(&Cinematique_ROS_DW.state_l[0], 0, 625U * sizeof(uint32_T));
    Cinematique_ROS_B.r = 5489U;
    Cinematique_ROS_DW.state_l[0] = 5489U;
    for (Cinematique_ROS_B.b_mti = 0; Cinematique_ROS_B.b_mti < 623;
         Cinematique_ROS_B.b_mti++) {
      Cinematique_ROS_B.r = ((Cinematique_ROS_B.r >> 30U ^ Cinematique_ROS_B.r) *
        1812433253U + Cinematique_ROS_B.b_mti) + 1U;
      Cinematique_ROS_DW.state_l[Cinematique_ROS_B.b_mti + 1] =
        Cinematique_ROS_B.r;
    }

    Cinematique_ROS_DW.state_l[624] = 624U;
    Cinematique_ROS_DW.method_not_empty_i = true;
    Cinematique_ROS_DW.method = 0U;
    Cinematique_ROS_DW.state_not_empty_m = true;
    Cinematique_ROS_DW.state_o[0] = 362436069U;
    Cinematique_ROS_DW.state_o[1] = 521288629U;
    Cinematique_ROS_DW.state_not_empty_j = true;
    Cinematique_ROS_DW.obj.isInitialized = 0;
    Cinematique_ROS_DW.obj.matlabCodegenIsDeleted = false;
    Cinematique_ROS_DW.objisempty = true;
    Cinematique_RO_SystemCore_setup(&Cinematique_ROS_DW.obj);

    /* End of Start for MATLABSystem: '<S18>/MATLAB System' */
    /* End of Start for SubSystem: '<Root>/Subsystem1' */

    /* Start for Enabled SubSystem: '<Root>/Enabled Subsystem' */
    Cinematique_ROS_DW.EnabledSubsystem_MODE = false;

    /* Start for Atomic SubSystem: '<S5>/Publish' */
    /* Start for MATLABSystem: '<S10>/SinkBlock' */
    Cinematique_ROS_DW.obj_e.matlabCodegenIsDeleted = false;
    Cinematique_ROS_DW.objisempty_g = true;
    Cinematique_ROS_DW.obj_e.isInitialized = 1;
    for (Cinematique_ROS_B.b_mti = 0; Cinematique_ROS_B.b_mti < 7;
         Cinematique_ROS_B.b_mti++) {
      Cinematique_ROS_B.b_zeroDelimTopic_l[Cinematique_ROS_B.b_mti] =
        tmp_0[Cinematique_ROS_B.b_mti];
    }

    Cinematique_ROS_B.b_zeroDelimTopic_l[7] = '\x00';
    Pub_Cinematique_ROS_1271.createPublisher
      (&Cinematique_ROS_B.b_zeroDelimTopic_l[0], 1);
    Cinematique_ROS_DW.obj_e.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<S10>/SinkBlock' */
    /* End of Start for SubSystem: '<S5>/Publish' */

    /* Start for Atomic SubSystem: '<S6>/Publish' */
    /* Start for MATLABSystem: '<S12>/SinkBlock' */
    Cinematique_ROS_DW.obj_n.matlabCodegenIsDeleted = false;
    Cinematique_ROS_DW.objisempty_fv = true;
    Cinematique_ROS_DW.obj_n.isInitialized = 1;
    for (Cinematique_ROS_B.b_mti = 0; Cinematique_ROS_B.b_mti < 6;
         Cinematique_ROS_B.b_mti++) {
      Cinematique_ROS_B.b_zeroDelimTopic_i[Cinematique_ROS_B.b_mti] =
        tmp_1[Cinematique_ROS_B.b_mti];
    }

    Cinematique_ROS_B.b_zeroDelimTopic_i[6] = '\x00';
    Pub_Cinematique_ROS_1319.createPublisher
      (&Cinematique_ROS_B.b_zeroDelimTopic_i[0], 1);
    Cinematique_ROS_DW.obj_n.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<S12>/SinkBlock' */
    /* End of Start for SubSystem: '<S6>/Publish' */

    /* Start for Atomic SubSystem: '<S7>/Publish' */
    /* Start for MATLABSystem: '<S14>/SinkBlock' */
    Cinematique_ROS_DW.obj_k.matlabCodegenIsDeleted = false;
    Cinematique_ROS_DW.objisempty_m = true;
    Cinematique_ROS_DW.obj_k.isInitialized = 1;
    for (Cinematique_ROS_B.b_mti = 0; Cinematique_ROS_B.b_mti < 7;
         Cinematique_ROS_B.b_mti++) {
      Cinematique_ROS_B.b_zeroDelimTopic_l[Cinematique_ROS_B.b_mti] =
        tmp_2[Cinematique_ROS_B.b_mti];
    }

    Cinematique_ROS_B.b_zeroDelimTopic_l[7] = '\x00';
    Pub_Cinematique_ROS_1314.createPublisher
      (&Cinematique_ROS_B.b_zeroDelimTopic_l[0], 1);
    Cinematique_ROS_DW.obj_k.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<S14>/SinkBlock' */
    /* End of Start for SubSystem: '<S7>/Publish' */

    /* Start for Atomic SubSystem: '<S8>/Publish' */
    /* Start for MATLABSystem: '<S16>/SinkBlock' */
    Cinematique_ROS_DW.obj_o.matlabCodegenIsDeleted = false;
    Cinematique_ROS_DW.objisempty_h = true;
    Cinematique_ROS_DW.obj_o.isInitialized = 1;
    for (Cinematique_ROS_B.b_mti = 0; Cinematique_ROS_B.b_mti < 8;
         Cinematique_ROS_B.b_mti++) {
      Cinematique_ROS_B.b_zeroDelimTopic_d[Cinematique_ROS_B.b_mti] =
        tmp_3[Cinematique_ROS_B.b_mti];
    }

    Cinematique_ROS_B.b_zeroDelimTopic_d[8] = '\x00';
    Pub_Cinematique_ROS_1324.createPublisher
      (&Cinematique_ROS_B.b_zeroDelimTopic_d[0], 1);
    Cinematique_ROS_DW.obj_o.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<S16>/SinkBlock' */
    /* End of Start for SubSystem: '<S8>/Publish' */
    /* End of Start for SubSystem: '<Root>/Enabled Subsystem' */
  }

  /* InitializeConditions for RateLimiter: '<Root>/Rate Limiter' */
  Cinematique_ROS_DW.LastMajorTime = (rtInf);

  /* SystemInitialize for Enabled SubSystem: '<Root>/Subsystem1' */
  /* InitializeConditions for Delay: '<S4>/Delay' */
  Cinematique_ROS_DW.Delay_DSTATE[0] = Cinematique_ROS_P.Delay_InitialCondition
    [0];

  /* SystemInitialize for MATLABSystem: '<S18>/MATLAB System' incorporates:
   *  Outport: '<S4>/Config'
   */
  Cinematique_ROS_B.MATLABSystem_o1[0] = Cinematique_ROS_P.Config_Y0;

  /* InitializeConditions for Delay: '<S4>/Delay' */
  Cinematique_ROS_DW.Delay_DSTATE[1] = Cinematique_ROS_P.Delay_InitialCondition
    [1];

  /* SystemInitialize for MATLABSystem: '<S18>/MATLAB System' incorporates:
   *  Outport: '<S4>/Config'
   */
  Cinematique_ROS_B.MATLABSystem_o1[1] = Cinematique_ROS_P.Config_Y0;

  /* InitializeConditions for Delay: '<S4>/Delay' */
  Cinematique_ROS_DW.Delay_DSTATE[2] = Cinematique_ROS_P.Delay_InitialCondition
    [2];

  /* SystemInitialize for MATLABSystem: '<S18>/MATLAB System' incorporates:
   *  Outport: '<S4>/Config'
   */
  Cinematique_ROS_B.MATLABSystem_o1[2] = Cinematique_ROS_P.Config_Y0;

  /* InitializeConditions for Delay: '<S4>/Delay' */
  Cinematique_ROS_DW.Delay_DSTATE[3] = Cinematique_ROS_P.Delay_InitialCondition
    [3];

  /* SystemInitialize for MATLABSystem: '<S18>/MATLAB System' incorporates:
   *  Outport: '<S4>/Config'
   */
  Cinematique_ROS_B.MATLABSystem_o1[3] = Cinematique_ROS_P.Config_Y0;

  /* SystemInitialize for Outport: '<S4>/Flag' */
  Cinematique_ROS_B.MATLABSystem_o2.ExitFlag = Cinematique_ROS_P.Flag_Y0;

  /* End of SystemInitialize for SubSystem: '<Root>/Subsystem1' */
}

/* Model terminate function */
void Cinematique_ROS_terminate(void)
{
  b_inverseKinematics_Cinematiq_T *obj;

  /* Terminate for Atomic SubSystem: '<Root>/Subscribe' */
  /* Terminate for MATLABSystem: '<S3>/SourceBlock' */
  if (!Cinematique_ROS_DW.obj_ez.matlabCodegenIsDeleted) {
    Cinematique_ROS_DW.obj_ez.matlabCodegenIsDeleted = true;
  }

  /* End of Terminate for MATLABSystem: '<S3>/SourceBlock' */
  /* End of Terminate for SubSystem: '<Root>/Subscribe' */

  /* Terminate for Enabled SubSystem: '<Root>/Subsystem1' */
  /* Terminate for MATLABSystem: '<S18>/MATLAB System' */
  if (!Cinematique_ROS_DW.obj.matlabCodegenIsDeleted) {
    Cinematique_ROS_DW.obj.matlabCodegenIsDeleted = true;
  }

  obj = &Cinematique_ROS_DW.obj.IKInternal;
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
    if (obj->isInitialized == 1) {
      obj->isInitialized = 2;
    }
  }

  /* End of Terminate for MATLABSystem: '<S18>/MATLAB System' */
  /* End of Terminate for SubSystem: '<Root>/Subsystem1' */
  emxFreeStruct_robotics_slmanip_(&Cinematique_ROS_DW.obj);

  /* Terminate for Enabled SubSystem: '<Root>/Enabled Subsystem' */
  /* Terminate for Atomic SubSystem: '<S5>/Publish' */
  /* Terminate for MATLABSystem: '<S10>/SinkBlock' */
  if (!Cinematique_ROS_DW.obj_e.matlabCodegenIsDeleted) {
    Cinematique_ROS_DW.obj_e.matlabCodegenIsDeleted = true;
  }

  /* End of Terminate for MATLABSystem: '<S10>/SinkBlock' */
  /* End of Terminate for SubSystem: '<S5>/Publish' */

  /* Terminate for Atomic SubSystem: '<S6>/Publish' */
  /* Terminate for MATLABSystem: '<S12>/SinkBlock' */
  if (!Cinematique_ROS_DW.obj_n.matlabCodegenIsDeleted) {
    Cinematique_ROS_DW.obj_n.matlabCodegenIsDeleted = true;
  }

  /* End of Terminate for MATLABSystem: '<S12>/SinkBlock' */
  /* End of Terminate for SubSystem: '<S6>/Publish' */

  /* Terminate for Atomic SubSystem: '<S7>/Publish' */
  /* Terminate for MATLABSystem: '<S14>/SinkBlock' */
  if (!Cinematique_ROS_DW.obj_k.matlabCodegenIsDeleted) {
    Cinematique_ROS_DW.obj_k.matlabCodegenIsDeleted = true;
  }

  /* End of Terminate for MATLABSystem: '<S14>/SinkBlock' */
  /* End of Terminate for SubSystem: '<S7>/Publish' */

  /* Terminate for Atomic SubSystem: '<S8>/Publish' */
  /* Terminate for MATLABSystem: '<S16>/SinkBlock' */
  if (!Cinematique_ROS_DW.obj_o.matlabCodegenIsDeleted) {
    Cinematique_ROS_DW.obj_o.matlabCodegenIsDeleted = true;
  }

  /* End of Terminate for MATLABSystem: '<S16>/SinkBlock' */
  /* End of Terminate for SubSystem: '<S8>/Publish' */
  /* End of Terminate for SubSystem: '<Root>/Enabled Subsystem' */
}
