/*
 * inverse_kinematics.cpp
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "inverse_kinematics".
 *
 * Model version              : 4.51
 * Simulink Coder version : 9.7 (R2022a) 13-Nov-2021
 * C++ source code generated on : Sun Oct 16 00:22:54 2022
 *
 * Target selection: ert.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Generic->Unspecified (assume 32-bit Generic)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "inverse_kinematics.h"
#include "inverse_kinematics_types.h"
#include "rtwtypes.h"
#include <string.h>

extern "C" {

#include "rt_nonfinite.h"

}
#include <math.h>
#include "coder_posix_time.h"
#include "inverse_kinematics_private.h"
#include "rt_defines.h"
#include <stdlib.h>
#include <stddef.h>

/* Block signals (default storage) */
B_inverse_kinematics_T inverse_kinematics_B;

/* Block states (default storage) */
DW_inverse_kinematics_T inverse_kinematics_DW;

/* Real-time model */
RT_MODEL_inverse_kinematics_T inverse_kinematics_M_ =
  RT_MODEL_inverse_kinematics_T();
RT_MODEL_inverse_kinematics_T *const inverse_kinematics_M =
  &inverse_kinematics_M_;

/* Forward declaration for local functions */
static void inverse_kinemati_emxInit_char_T(emxArray_char_T_inverse_kinem_T
  **pEmxArray, int32_T numDimensions);
static void emxInitStruct_t_robotics_manip_(t_robotics_manip_internal_Rig_T
  *pStruct);
static void inverse__emxInit_unnamed_struct(emxArray_unnamed_struct_inver_T
  **pEmxArray, int32_T numDimensions);
static void emxInitStruct_l_robotics_manip_(l_robotics_manip_internal_Col_T
  *pStruct);
static void emxInitMatrix_l_robotics_manip_(l_robotics_manip_internal_Col_T
  pMatrix[15]);
static void inverse_kinemati_emxInit_real_T(emxArray_real_T_inverse_kinem_T
  **pEmxArray, int32_T numDimensions);
static void emxInitStruct_c_rigidBodyJoint(c_rigidBodyJoint_inverse_kine_T
  *pStruct);
static void emxInitMatrix_c_rigidBodyJoint(c_rigidBodyJoint_inverse_kine_T
  pMatrix[15]);
static void emxInitMatrix_t_robotics_manip_(t_robotics_manip_internal_Rig_T
  pMatrix[14]);
static void emxInitStruct_u_robotics_manip_(u_robotics_manip_internal_Rig_T
  *pStruct);
static void emxInitStruct_f_robotics_manip_(f_robotics_manip_internal_IKE_T
  *pStruct);
static void emxInitMatrix_c_rigidBodyJoint1(c_rigidBodyJoint_inverse_kine_T
  pMatrix[14]);
static void emxInitMatrix_t_robotics_mani_g(t_robotics_manip_internal_Rig_T
  pMatrix[7]);
static void emxInitMatrix_l_robotics_mani_g(l_robotics_manip_internal_Col_T
  pMatrix[8]);
static void emxInitMatrix_c_rigidBodyJoint2(c_rigidBodyJoint_inverse_kine_T
  pMatrix[8]);
static void emxInitStruct_v_robotics_manip_(v_robotics_manip_internal_Rig_T
  *pStruct);
static void emxInitStruct_b_inverseKinemati(b_inverseKinematics_inverse_k_T
  *pStruct);
static void emxInitStruct_robotics_slmanip_(robotics_slmanip_internal_blo_T
  *pStruct);
static void invers_emxEnsureCapacity_char_T(emxArray_char_T_inverse_kinem_T
  *emxArray, int32_T oldNumel);
static void inverse_kinemati_emxFree_char_T(emxArray_char_T_inverse_kinem_T
  **pEmxArray);
static void invers_emxEnsureCapacity_real_T(emxArray_real_T_inverse_kinem_T
  *emxArray, int32_T oldNumel);
static void emxEnsureCapacity_unnamed_struc(emxArray_unnamed_struct_inver_T
  *emxArray, int32_T oldNumel);
static void inverse__emxFree_unnamed_struct(emxArray_unnamed_struct_inver_T
  **pEmxArray);
static l_robotics_manip_internal_Col_T *inver_CollisionSet_CollisionSet
  (l_robotics_manip_internal_Col_T *obj, real_T maxElements);
static t_robotics_manip_internal_Rig_T *inverse_kin_RigidBody_RigidBody
  (t_robotics_manip_internal_Rig_T *obj, l_robotics_manip_internal_Col_T *iobj_0,
   c_rigidBodyJoint_inverse_kine_T *iobj_1);
static t_robotics_manip_internal_Rig_T *inverse_k_RigidBody_RigidBody_g
  (t_robotics_manip_internal_Rig_T *obj, l_robotics_manip_internal_Col_T *iobj_0,
   c_rigidBodyJoint_inverse_kine_T *iobj_1);
static t_robotics_manip_internal_Rig_T *inverse__RigidBody_RigidBody_ga
  (t_robotics_manip_internal_Rig_T *obj, l_robotics_manip_internal_Col_T *iobj_0,
   c_rigidBodyJoint_inverse_kine_T *iobj_1);
static t_robotics_manip_internal_Rig_T *inverse_RigidBody_RigidBody_gaq
  (t_robotics_manip_internal_Rig_T *obj, l_robotics_manip_internal_Col_T *iobj_0,
   c_rigidBodyJoint_inverse_kine_T *iobj_1);
static t_robotics_manip_internal_Rig_T *invers_RigidBody_RigidBody_gaqy
  (t_robotics_manip_internal_Rig_T *obj, l_robotics_manip_internal_Col_T *iobj_0,
   c_rigidBodyJoint_inverse_kine_T *iobj_1);
static t_robotics_manip_internal_Rig_T *inver_RigidBody_RigidBody_gaqyy
  (t_robotics_manip_internal_Rig_T *obj, l_robotics_manip_internal_Col_T *iobj_0,
   c_rigidBodyJoint_inverse_kine_T *iobj_1);
static t_robotics_manip_internal_Rig_T *inve_RigidBody_RigidBody_gaqyy1
  (t_robotics_manip_internal_Rig_T *obj, l_robotics_manip_internal_Col_T *iobj_0,
   c_rigidBodyJoint_inverse_kine_T *iobj_1);
static t_robotics_manip_internal_Rig_T *inv_RigidBody_RigidBody_gaqyy1n
  (t_robotics_manip_internal_Rig_T *obj, l_robotics_manip_internal_Col_T *iobj_0,
   c_rigidBodyJoint_inverse_kine_T *iobj_1);
static t_robotics_manip_internal_Rig_T *in_RigidBody_RigidBody_gaqyy1no
  (t_robotics_manip_internal_Rig_T *obj, l_robotics_manip_internal_Col_T *iobj_0,
   c_rigidBodyJoint_inverse_kine_T *iobj_1);
static t_robotics_manip_internal_Rig_T *i_RigidBody_RigidBody_gaqyy1no5
  (t_robotics_manip_internal_Rig_T *obj, l_robotics_manip_internal_Col_T *iobj_0,
   c_rigidBodyJoint_inverse_kine_T *iobj_1);
static t_robotics_manip_internal_Rig_T *RigidBody_RigidBody_gaqyy1no5h
  (t_robotics_manip_internal_Rig_T *obj, l_robotics_manip_internal_Col_T *iobj_0,
   c_rigidBodyJoint_inverse_kine_T *iobj_1);
static t_robotics_manip_internal_Rig_T *RigidBody_RigidBody_gaqyy1no5hz
  (t_robotics_manip_internal_Rig_T *obj, l_robotics_manip_internal_Col_T *iobj_0,
   c_rigidBodyJoint_inverse_kine_T *iobj_1);
static c_rigidBodyJoint_inverse_kine_T *i_rigidBodyJoint_rigidBodyJoint
  (c_rigidBodyJoint_inverse_kine_T *obj, const emxArray_char_T_inverse_kinem_T
   *jname);
static u_robotics_manip_internal_Rig_T *inv_RigidBodyTree_RigidBodyTree
  (u_robotics_manip_internal_Rig_T *obj);
static void inverse_genrand_uint32_vector_g(uint32_T mt[625], uint32_T u[2]);
static boolean_T inverse_kinemati_is_valid_state(const uint32_T mt[625]);
static void inverse_kinematics_rand(real_T r[5]);
static boolean_T inverse_kinematics_strcmp(const emxArray_char_T_inverse_kinem_T
  *a, const emxArray_char_T_inverse_kinem_T *b);
static real_T RigidBodyTree_findBodyIndexByNa(v_robotics_manip_internal_Rig_T
  *obj, const emxArray_char_T_inverse_kinem_T *bodyname);
static void inverse_kinemati_emxFree_real_T(emxArray_real_T_inverse_kinem_T
  **pEmxArray);
static t_robotics_manip_internal_Rig_T *inverse_kinemati_RigidBody_copy
  (t_robotics_manip_internal_Rig_T *obj, l_robotics_manip_internal_Col_T *iobj_0,
   c_rigidBodyJoint_inverse_kine_T *iobj_1, t_robotics_manip_internal_Rig_T
   *iobj_2);
static void inverse_k_RigidBodyTree_addBody(v_robotics_manip_internal_Rig_T *obj,
  t_robotics_manip_internal_Rig_T *bodyin, const emxArray_char_T_inverse_kinem_T
  *parentName, c_rigidBodyJoint_inverse_kine_T *iobj_0,
  t_robotics_manip_internal_Rig_T *iobj_1, l_robotics_manip_internal_Col_T
  *iobj_2);
static void inverseKinematics_set_RigidBody(b_inverseKinematics_inverse_k_T *obj,
  u_robotics_manip_internal_Rig_T *rigidbodytree,
  c_rigidBodyJoint_inverse_kine_T *iobj_0, t_robotics_manip_internal_Rig_T
  *iobj_1, l_robotics_manip_internal_Col_T *iobj_2,
  v_robotics_manip_internal_Rig_T *iobj_3);
static void inverse_kinema_SystemCore_setup(robotics_slmanip_internal_blo_T *obj);
static void RigidBodyTree_get_JointPosition(v_robotics_manip_internal_Rig_T *obj,
  emxArray_real_T_inverse_kinem_T *limits);
static void inverse_kinemati_emxInit_int8_T(emxArray_int8_T_inverse_kinem_T
  **pEmxArray, int32_T numDimensions);
static void invers_emxEnsureCapacity_int8_T(emxArray_int8_T_inverse_kinem_T
  *emxArray, int32_T oldNumel);
static void inverse_kinemati_emxFree_int8_T(emxArray_int8_T_inverse_kinem_T
  **pEmxArray);
static void inverse_ki_binary_expand_op_gaq(boolean_T in1[4], const real_T in2[4],
  const emxArray_real_T_inverse_kinem_T *in3);
static void inverse_kin_binary_expand_op_ga(boolean_T in1[4], const real_T in2[4],
  const emxArray_real_T_inverse_kinem_T *in3);
static void inverse_kinematics_eml_find(const boolean_T x[4], int32_T i_data[],
  int32_T *i_size);
static void inverse_kinematics_tic(real_T *tstart_tv_sec, real_T *tstart_tv_nsec);
static void i_RigidBodyTree_ancestorIndices(v_robotics_manip_internal_Rig_T *obj,
  t_robotics_manip_internal_Rig_T *body, emxArray_real_T_inverse_kinem_T
  *indices);
static void RigidBodyTree_kinematicPathInte(v_robotics_manip_internal_Rig_T *obj,
  t_robotics_manip_internal_Rig_T *body1, t_robotics_manip_internal_Rig_T *body2,
  emxArray_real_T_inverse_kinem_T *indices);
static void in_rigidBodyJoint_get_JointAxis(const
  c_rigidBodyJoint_inverse_kine_T *obj, real_T ax[3]);
static void inverse_kinematics_cat(real_T varargin_1, real_T varargin_2, real_T
  varargin_3, real_T varargin_4, real_T varargin_5, real_T varargin_6, real_T
  varargin_7, real_T varargin_8, real_T varargin_9, real_T y[9]);
static void inverse_kinematics_mtimes(const real_T A[36], const
  emxArray_real_T_inverse_kinem_T *B, emxArray_real_T_inverse_kinem_T *C);
static void RigidBodyTree_efficientFKAndJac(v_robotics_manip_internal_Rig_T *obj,
  const real_T qv[4], const emxArray_char_T_inverse_kinem_T *body1Name, real_T
  T_data[], int32_T T_size[2], emxArray_real_T_inverse_kinem_T *Jac);
static creal_T inverse_kinematics_sqrt(const creal_T x);
static real_T inverse_kinematics_xnrm2(int32_T n, const real_T x[9], int32_T ix0);
static real_T inverse_kinematics_xdotc(int32_T n, const real_T x[9], int32_T ix0,
  const real_T y[9], int32_T iy0);
static void inverse_kinematics_xaxpy(int32_T n, real_T a, int32_T ix0, const
  real_T y[9], int32_T iy0, real_T b_y[9]);
static real_T inverse_kinematics_xnrm2_g(const real_T x[3], int32_T ix0);
static void inverse_kinematics_xaxpy_gaq(int32_T n, real_T a, const real_T x[9],
  int32_T ix0, real_T y[3], int32_T iy0);
static void inverse_kinematics_xaxpy_ga(int32_T n, real_T a, const real_T x[3],
  int32_T ix0, const real_T y[9], int32_T iy0, real_T b_y[9]);
static void inverse_kinematics_xswap(const real_T x[9], int32_T ix0, int32_T iy0,
  real_T b_x[9]);
static void inverse_kinematics_xrotg(real_T a, real_T b, real_T *b_a, real_T
  *b_b, real_T *c, real_T *s);
static void inverse_kinematics_xrot(const real_T x[9], int32_T ix0, int32_T iy0,
  real_T c, real_T s, real_T b_x[9]);
static void inverse_kinematics_svd(const real_T A[9], real_T U[9], real_T s[3],
  real_T V[9]);
static void inverse_kin_IKHelpers_poseError(const real_T Td[16], const real_T
  T_data[], const int32_T T_size[2], real_T errorvec[6]);
static void inverse_kinematics_mtimes_g(const real_T A[6], const
  emxArray_real_T_inverse_kinem_T *B, emxArray_real_T_inverse_kinem_T *C);
static void inverse_kinem_emxInit_boolean_T(emxArray_boolean_T_inverse_ki_T
  **pEmxArray, int32_T numDimensions);
static real_T inverse_kinematics_norm_g(const real_T x[6]);
static void inverse_kinematics_minus_g(emxArray_real_T_inverse_kinem_T *in1,
  const emxArray_real_T_inverse_kinem_T *in2);
static void inv_emxEnsureCapacity_boolean_T(emxArray_boolean_T_inverse_ki_T
  *emxArray, int32_T oldNumel);
static real_T inverse_kinematics_toc(real_T tstart_tv_sec, real_T tstart_tv_nsec);
static void inverse_kinematics_mldivide(const real_T A[16], const
  emxArray_real_T_inverse_kinem_T *B, real_T Y_data[], int32_T *Y_size);
static void inverse_kine_binary_expand_op_g(real_T in1_data[], int32_T *in1_size,
  const emxArray_real_T_inverse_kinem_T *in2, real_T in3, const real_T in4[16],
  const emxArray_real_T_inverse_kinem_T *in5);
static void inverse_kinematics_expand_max(const emxArray_real_T_inverse_kinem_T *
  a, const real_T b[4], real_T c[4]);
static void inverse_kinematics_expand_min(const emxArray_real_T_inverse_kinem_T *
  a, const real_T b[4], real_T c[4]);
static void inverse_kinem_emxFree_boolean_T(emxArray_boolean_T_inverse_ki_T
  **pEmxArray);
static void ErrorDampedLevenbergMarquardt_s(h_robotics_core_internal_Erro_T *obj,
  real_T xSol[4], c_robotics_core_internal_NLPS_T *exitFlag, real_T *en, real_T *
  iter);
static boolean_T inverse_kinematics_any(const emxArray_boolean_T_inverse_ki_T *x);
static void inverse_kinematics_randn(const real_T varargin_1[2],
  emxArray_real_T_inverse_kinem_T *r);
static void inverse_kinematics_minus(emxArray_real_T_inverse_kinem_T *in1, const
  emxArray_real_T_inverse_kinem_T *in2);
static void inverse_kinematics_plus(emxArray_real_T_inverse_kinem_T *in1, const
  emxArray_real_T_inverse_kinem_T *in2);
static void inverse_kinematics_rand_g(real_T varargin_1,
  emxArray_real_T_inverse_kinem_T *r);
static void inverse_kinema_binary_expand_op(emxArray_real_T_inverse_kinem_T *in1,
  const emxArray_real_T_inverse_kinem_T *in2, const
  emxArray_real_T_inverse_kinem_T *in3);
static void invers_NLPSolverInterface_solve(h_robotics_core_internal_Erro_T *obj,
  const real_T seed[4], real_T xSol[4], real_T *solutionInfo_Iterations, real_T *
  solutionInfo_RRAttempts, real_T *solutionInfo_Error, real_T
  *solutionInfo_ExitFlag, char_T solutionInfo_Status_data[], int32_T
  solutionInfo_Status_size[2]);
static void inverse_kinemat_emxInit_int32_T(emxArray_int32_T_inverse_kine_T
  **pEmxArray, int32_T numDimensions);
static void inver_emxEnsureCapacity_int32_T(emxArray_int32_T_inverse_kine_T
  *emxArray, int32_T oldNumel);
static void inverse_kinemat_emxFree_int32_T(emxArray_int32_T_inverse_kine_T
  **pEmxArray);
static void inverse_kinema_emxInit_uint32_T(emxArray_uint32_T_inverse_kin_T
  **pEmxArray, int32_T numDimensions);
static void inve_emxEnsureCapacity_uint32_T(emxArray_uint32_T_inverse_kin_T
  *emxArray, int32_T oldNumel);
static void inverse_kinema_emxFree_uint32_T(emxArray_uint32_T_inverse_kin_T
  **pEmxArray);
static void inver_inverseKinematics_solve_g(b_inverseKinematics_inverse_k_T *obj,
  real_T initialGuess[4], real_T *solutionInfo_Iterations, real_T
  *solutionInfo_NumRandomRestarts, real_T *solutionInfo_PoseErrorNorm, real_T
  *solutionInfo_ExitFlag, char_T solutionInfo_Status_data[], int32_T
  solutionInfo_Status_size[2]);
static void inve_inverseKinematics_stepImpl(b_inverseKinematics_inverse_k_T *obj,
  const real_T tform[16], const real_T weights[6], const real_T initialGuess[4],
  real_T QSol[4]);
static void emxFreeStruct_t_robotics_manip_(t_robotics_manip_internal_Rig_T
  *pStruct);
static void emxFreeStruct_l_robotics_manip_(l_robotics_manip_internal_Col_T
  *pStruct);
static void emxFreeMatrix_l_robotics_manip_(l_robotics_manip_internal_Col_T
  pMatrix[15]);
static void emxFreeStruct_c_rigidBodyJoint(c_rigidBodyJoint_inverse_kine_T
  *pStruct);
static void emxFreeMatrix_c_rigidBodyJoint(c_rigidBodyJoint_inverse_kine_T
  pMatrix[15]);
static void emxFreeMatrix_t_robotics_manip_(t_robotics_manip_internal_Rig_T
  pMatrix[14]);
static void emxFreeStruct_u_robotics_manip_(u_robotics_manip_internal_Rig_T
  *pStruct);
static void emxFreeStruct_f_robotics_manip_(f_robotics_manip_internal_IKE_T
  *pStruct);
static void emxFreeMatrix_c_rigidBodyJoint1(c_rigidBodyJoint_inverse_kine_T
  pMatrix[14]);
static void emxFreeMatrix_t_robotics_mani_g(t_robotics_manip_internal_Rig_T
  pMatrix[7]);
static void emxFreeMatrix_l_robotics_mani_g(l_robotics_manip_internal_Col_T
  pMatrix[8]);
static void emxFreeMatrix_c_rigidBodyJoint2(c_rigidBodyJoint_inverse_kine_T
  pMatrix[8]);
static void emxFreeStruct_v_robotics_manip_(v_robotics_manip_internal_Rig_T
  *pStruct);
static void emxFreeStruct_b_inverseKinemati(b_inverseKinematics_inverse_k_T
  *pStruct);
static void emxFreeStruct_robotics_slmanip_(robotics_slmanip_internal_blo_T
  *pStruct);
int32_T div_s32(int32_T numerator, int32_T denominator)
{
  int32_T quotient;
  if (denominator == 0) {
    quotient = numerator >= 0 ? MAX_int32_T : MIN_int32_T;

    /* Divide by zero handler */
  } else {
    uint32_T tempAbsQuotient;
    tempAbsQuotient = (numerator < 0 ? ~static_cast<uint32_T>(numerator) + 1U :
                       static_cast<uint32_T>(numerator)) / (denominator < 0 ? ~
      static_cast<uint32_T>(denominator) + 1U : static_cast<uint32_T>
      (denominator));
    quotient = (numerator < 0) != (denominator < 0) ? -static_cast<int32_T>
      (tempAbsQuotient) : static_cast<int32_T>(tempAbsQuotient);
  }

  return quotient;
}

static void inverse_kinemati_emxInit_char_T(emxArray_char_T_inverse_kinem_T
  **pEmxArray, int32_T numDimensions)
{
  emxArray_char_T_inverse_kinem_T *emxArray;
  *pEmxArray = static_cast<emxArray_char_T_inverse_kinem_T *>(malloc(sizeof
    (emxArray_char_T_inverse_kinem_T)));
  emxArray = *pEmxArray;
  emxArray->data = static_cast<char_T *>(NULL);
  emxArray->numDimensions = numDimensions;
  emxArray->size = static_cast<int32_T *>(malloc(sizeof(int32_T) * numDimensions));
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (inverse_kinematics_B.i_pt = 0; inverse_kinematics_B.i_pt < numDimensions;
       inverse_kinematics_B.i_pt++) {
    emxArray->size[inverse_kinematics_B.i_pt] = 0;
  }
}

static void emxInitStruct_t_robotics_manip_(t_robotics_manip_internal_Rig_T
  *pStruct)
{
  inverse_kinemati_emxInit_char_T(&pStruct->NameInternal, 2);
}

static void inverse__emxInit_unnamed_struct(emxArray_unnamed_struct_inver_T
  **pEmxArray, int32_T numDimensions)
{
  emxArray_unnamed_struct_inver_T *emxArray;
  *pEmxArray = static_cast<emxArray_unnamed_struct_inver_T *>(malloc(sizeof
    (emxArray_unnamed_struct_inver_T)));
  emxArray = *pEmxArray;
  emxArray->data = (k_robotics_manip_internal_Col_T **)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = static_cast<int32_T *>(malloc(sizeof(int32_T) * numDimensions));
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (int32_T i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

static void emxInitStruct_l_robotics_manip_(l_robotics_manip_internal_Col_T
  *pStruct)
{
  inverse__emxInit_unnamed_struct(&pStruct->CollisionGeometries, 2);
}

static void emxInitMatrix_l_robotics_manip_(l_robotics_manip_internal_Col_T
  pMatrix[15])
{
  for (int32_T i = 0; i < 15; i++) {
    emxInitStruct_l_robotics_manip_(&pMatrix[i]);
  }
}

static void inverse_kinemati_emxInit_real_T(emxArray_real_T_inverse_kinem_T
  **pEmxArray, int32_T numDimensions)
{
  emxArray_real_T_inverse_kinem_T *emxArray;
  *pEmxArray = static_cast<emxArray_real_T_inverse_kinem_T *>(malloc(sizeof
    (emxArray_real_T_inverse_kinem_T)));
  emxArray = *pEmxArray;
  emxArray->data = static_cast<real_T *>(NULL);
  emxArray->numDimensions = numDimensions;
  emxArray->size = static_cast<int32_T *>(malloc(sizeof(int32_T) * numDimensions));
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (inverse_kinematics_B.i_k = 0; inverse_kinematics_B.i_k < numDimensions;
       inverse_kinematics_B.i_k++) {
    emxArray->size[inverse_kinematics_B.i_k] = 0;
  }
}

static void emxInitStruct_c_rigidBodyJoint(c_rigidBodyJoint_inverse_kine_T
  *pStruct)
{
  inverse_kinemati_emxInit_char_T(&pStruct->Type, 2);
  inverse_kinemati_emxInit_real_T(&pStruct->MotionSubspace, 2);
  inverse_kinemati_emxInit_char_T(&pStruct->NameInternal, 2);
  inverse_kinemati_emxInit_real_T(&pStruct->PositionLimitsInternal, 2);
  inverse_kinemati_emxInit_real_T(&pStruct->HomePositionInternal, 1);
}

static void emxInitMatrix_c_rigidBodyJoint(c_rigidBodyJoint_inverse_kine_T
  pMatrix[15])
{
  for (int32_T i = 0; i < 15; i++) {
    emxInitStruct_c_rigidBodyJoint(&pMatrix[i]);
  }
}

static void emxInitMatrix_t_robotics_manip_(t_robotics_manip_internal_Rig_T
  pMatrix[14])
{
  for (int32_T i = 0; i < 14; i++) {
    emxInitStruct_t_robotics_manip_(&pMatrix[i]);
  }
}

static void emxInitStruct_u_robotics_manip_(u_robotics_manip_internal_Rig_T
  *pStruct)
{
  emxInitStruct_t_robotics_manip_(&pStruct->Base);
  emxInitMatrix_l_robotics_manip_(pStruct->_pobj0);
  emxInitMatrix_c_rigidBodyJoint(pStruct->_pobj1);
  emxInitMatrix_t_robotics_manip_(pStruct->_pobj2);
}

static void emxInitStruct_f_robotics_manip_(f_robotics_manip_internal_IKE_T
  *pStruct)
{
  inverse_kinemati_emxInit_real_T(&pStruct->Limits, 2);
  inverse_kinemati_emxInit_char_T(&pStruct->BodyName, 2);
  inverse_kinemati_emxInit_real_T(&pStruct->ErrTemp, 1);
  inverse_kinemati_emxInit_real_T(&pStruct->GradTemp, 1);
}

static void emxInitMatrix_c_rigidBodyJoint1(c_rigidBodyJoint_inverse_kine_T
  pMatrix[14])
{
  for (int32_T i = 0; i < 14; i++) {
    emxInitStruct_c_rigidBodyJoint(&pMatrix[i]);
  }
}

static void emxInitMatrix_t_robotics_mani_g(t_robotics_manip_internal_Rig_T
  pMatrix[7])
{
  for (int32_T i = 0; i < 7; i++) {
    emxInitStruct_t_robotics_manip_(&pMatrix[i]);
  }
}

static void emxInitMatrix_l_robotics_mani_g(l_robotics_manip_internal_Col_T
  pMatrix[8])
{
  for (int32_T i = 0; i < 8; i++) {
    emxInitStruct_l_robotics_manip_(&pMatrix[i]);
  }
}

static void emxInitMatrix_c_rigidBodyJoint2(c_rigidBodyJoint_inverse_kine_T
  pMatrix[8])
{
  for (int32_T i = 0; i < 8; i++) {
    emxInitStruct_c_rigidBodyJoint(&pMatrix[i]);
  }
}

static void emxInitStruct_v_robotics_manip_(v_robotics_manip_internal_Rig_T
  *pStruct)
{
  emxInitStruct_t_robotics_manip_(&pStruct->Base);
  emxInitMatrix_t_robotics_mani_g(pStruct->_pobj0);
  emxInitMatrix_l_robotics_mani_g(pStruct->_pobj1);
  emxInitMatrix_c_rigidBodyJoint2(pStruct->_pobj2);
}

static void emxInitStruct_b_inverseKinemati(b_inverseKinematics_inverse_k_T
  *pStruct)
{
  inverse_kinemati_emxInit_real_T(&pStruct->Limits, 2);
  emxInitStruct_f_robotics_manip_(&pStruct->_pobj0);
  emxInitMatrix_c_rigidBodyJoint1(pStruct->_pobj1);
  emxInitMatrix_t_robotics_mani_g(pStruct->_pobj2);
  emxInitMatrix_l_robotics_manip_(pStruct->_pobj3);
  emxInitStruct_v_robotics_manip_(&pStruct->_pobj4);
}

static void emxInitStruct_robotics_slmanip_(robotics_slmanip_internal_blo_T
  *pStruct)
{
  emxInitStruct_u_robotics_manip_(&pStruct->TreeInternal);
  emxInitStruct_b_inverseKinemati(&pStruct->IKInternal);
}

static void invers_emxEnsureCapacity_char_T(emxArray_char_T_inverse_kinem_T
  *emxArray, int32_T oldNumel)
{
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  inverse_kinematics_B.newNumel_p = 1;
  for (inverse_kinematics_B.i_a = 0; inverse_kinematics_B.i_a <
       emxArray->numDimensions; inverse_kinematics_B.i_a++) {
    inverse_kinematics_B.newNumel_p *= emxArray->size[inverse_kinematics_B.i_a];
  }

  if (inverse_kinematics_B.newNumel_p > emxArray->allocatedSize) {
    inverse_kinematics_B.i_a = emxArray->allocatedSize;
    if (inverse_kinematics_B.i_a < 16) {
      inverse_kinematics_B.i_a = 16;
    }

    while (inverse_kinematics_B.i_a < inverse_kinematics_B.newNumel_p) {
      if (inverse_kinematics_B.i_a > 1073741823) {
        inverse_kinematics_B.i_a = MAX_int32_T;
      } else {
        inverse_kinematics_B.i_a <<= 1;
      }
    }

    newData = calloc(static_cast<uint32_T>(inverse_kinematics_B.i_a), sizeof
                     (char_T));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(char_T) * oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }

    emxArray->data = static_cast<char_T *>(newData);
    emxArray->allocatedSize = inverse_kinematics_B.i_a;
    emxArray->canFreeData = true;
  }
}

static void inverse_kinemati_emxFree_char_T(emxArray_char_T_inverse_kinem_T
  **pEmxArray)
{
  if (*pEmxArray != static_cast<emxArray_char_T_inverse_kinem_T *>(NULL)) {
    if (((*pEmxArray)->data != static_cast<char_T *>(NULL)) && (*pEmxArray)
        ->canFreeData) {
      free((*pEmxArray)->data);
    }

    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = static_cast<emxArray_char_T_inverse_kinem_T *>(NULL);
  }
}

static void invers_emxEnsureCapacity_real_T(emxArray_real_T_inverse_kinem_T
  *emxArray, int32_T oldNumel)
{
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  inverse_kinematics_B.newNumel = 1;
  for (inverse_kinematics_B.i_o4 = 0; inverse_kinematics_B.i_o4 <
       emxArray->numDimensions; inverse_kinematics_B.i_o4++) {
    inverse_kinematics_B.newNumel *= emxArray->size[inverse_kinematics_B.i_o4];
  }

  if (inverse_kinematics_B.newNumel > emxArray->allocatedSize) {
    inverse_kinematics_B.i_o4 = emxArray->allocatedSize;
    if (inverse_kinematics_B.i_o4 < 16) {
      inverse_kinematics_B.i_o4 = 16;
    }

    while (inverse_kinematics_B.i_o4 < inverse_kinematics_B.newNumel) {
      if (inverse_kinematics_B.i_o4 > 1073741823) {
        inverse_kinematics_B.i_o4 = MAX_int32_T;
      } else {
        inverse_kinematics_B.i_o4 <<= 1;
      }
    }

    newData = calloc(static_cast<uint32_T>(inverse_kinematics_B.i_o4), sizeof
                     (real_T));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(real_T) * oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }

    emxArray->data = static_cast<real_T *>(newData);
    emxArray->allocatedSize = inverse_kinematics_B.i_o4;
    emxArray->canFreeData = true;
  }
}

static void emxEnsureCapacity_unnamed_struc(emxArray_unnamed_struct_inver_T
  *emxArray, int32_T oldNumel)
{
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  inverse_kinematics_B.newNumel_o = 1;
  for (inverse_kinematics_B.i_n = 0; inverse_kinematics_B.i_n <
       emxArray->numDimensions; inverse_kinematics_B.i_n++) {
    inverse_kinematics_B.newNumel_o *= emxArray->size[inverse_kinematics_B.i_n];
  }

  if (inverse_kinematics_B.newNumel_o > emxArray->allocatedSize) {
    inverse_kinematics_B.i_n = emxArray->allocatedSize;
    if (inverse_kinematics_B.i_n < 16) {
      inverse_kinematics_B.i_n = 16;
    }

    while (inverse_kinematics_B.i_n < inverse_kinematics_B.newNumel_o) {
      if (inverse_kinematics_B.i_n > 1073741823) {
        inverse_kinematics_B.i_n = MAX_int32_T;
      } else {
        inverse_kinematics_B.i_n <<= 1;
      }
    }

    newData = calloc(static_cast<uint32_T>(inverse_kinematics_B.i_n), sizeof
                     (k_robotics_manip_internal_Col_T *));
    if (emxArray->data != NULL) {
      memcpy(newData, (void *)emxArray->data, sizeof
             (k_robotics_manip_internal_Col_T *) * oldNumel);
      if (emxArray->canFreeData) {
        free((void *)emxArray->data);
      }
    }

    emxArray->data = (k_robotics_manip_internal_Col_T **)newData;
    emxArray->allocatedSize = inverse_kinematics_B.i_n;
    emxArray->canFreeData = true;
  }
}

static void inverse__emxFree_unnamed_struct(emxArray_unnamed_struct_inver_T
  **pEmxArray)
{
  if (*pEmxArray != static_cast<emxArray_unnamed_struct_inver_T *>(NULL)) {
    if (((*pEmxArray)->data != (k_robotics_manip_internal_Col_T **)NULL) &&
        (*pEmxArray)->canFreeData) {
      free((void *)(*pEmxArray)->data);
    }

    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = static_cast<emxArray_unnamed_struct_inver_T *>(NULL);
  }
}

static l_robotics_manip_internal_Col_T *inver_CollisionSet_CollisionSet
  (l_robotics_manip_internal_Col_T *obj, real_T maxElements)
{
  emxArray_unnamed_struct_inver_T *e;
  k_robotics_manip_internal_Col_T *obj_0;
  l_robotics_manip_internal_Col_T *b_obj;
  inverse__emxInit_unnamed_struct(&e, 2);
  obj->Size = 0.0;
  b_obj = obj;
  obj->MaxElements = maxElements;
  inverse_kinematics_B.b_i_ms = e->size[0] * e->size[1];
  e->size[1] = static_cast<int32_T>(obj->MaxElements);
  emxEnsureCapacity_unnamed_struc(e, inverse_kinematics_B.b_i_ms);
  inverse_kinematics_B.b_i_ms = obj->CollisionGeometries->size[0] *
    obj->CollisionGeometries->size[1];
  obj->CollisionGeometries->size[0] = 1;
  obj->CollisionGeometries->size[1] = e->size[1];
  emxEnsureCapacity_unnamed_struc(obj->CollisionGeometries,
    inverse_kinematics_B.b_i_ms);
  inverse_kinematics_B.defaultCollisionObj_GeometryInt = 0;
  obj_0 = &obj->_pobj0;
  obj->_pobj0.CollisionPrimitive =
    inverse_kinematics_B.defaultCollisionObj_GeometryInt;
  obj->_pobj0.matlabCodegenIsDeleted = false;
  inverse_kinematics_B.c = obj->MaxElements;
  inverse_kinematics_B.d_p = static_cast<int32_T>(inverse_kinematics_B.c) - 1;
  inverse__emxFree_unnamed_struct(&e);
  for (inverse_kinematics_B.b_i_ms = 0; inverse_kinematics_B.b_i_ms <=
       inverse_kinematics_B.d_p; inverse_kinematics_B.b_i_ms++) {
    obj->CollisionGeometries->data[inverse_kinematics_B.b_i_ms] = obj_0;
  }

  return b_obj;
}

static t_robotics_manip_internal_Rig_T *inverse_kin_RigidBody_RigidBody
  (t_robotics_manip_internal_Rig_T *obj, l_robotics_manip_internal_Col_T *iobj_0,
   c_rigidBodyJoint_inverse_kine_T *iobj_1)
{
  emxArray_char_T_inverse_kinem_T *switch_expression;
  t_robotics_manip_internal_Rig_T *b_obj;
  real_T poslim_data[12];
  int32_T b_kstr;
  int32_T loop_ub;
  char_T b_0[9];
  char_T b[8];
  int8_T msubspace_data[36];
  int8_T b_I[9];
  int8_T tmp[6];
  boolean_T b_bool;
  static const char_T tmp_0[10] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '1' };

  static const int8_T tmp_1[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    1 };

  static const char_T tmp_2[14] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '1', '_', 'j', 'n', 't' };

  static const char_T tmp_3[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_4[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_5[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  int32_T exitg1;
  b_obj = obj;
  b_kstr = obj->NameInternal->size[0] * obj->NameInternal->size[1];
  obj->NameInternal->size[0] = 1;
  obj->NameInternal->size[1] = 10;
  invers_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 10; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp_0[b_kstr];
  }

  iobj_1->InTree = false;
  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    iobj_1->JointToParentTransform[b_kstr] = tmp_1[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    iobj_1->ChildToJointTransform[b_kstr] = tmp_1[b_kstr];
  }

  b_kstr = iobj_1->NameInternal->size[0] * iobj_1->NameInternal->size[1];
  iobj_1->NameInternal->size[0] = 1;
  iobj_1->NameInternal->size[1] = 14;
  invers_emxEnsureCapacity_char_T(iobj_1->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 14; b_kstr++) {
    iobj_1->NameInternal->data[b_kstr] = tmp_2[b_kstr];
  }

  b_kstr = iobj_1->Type->size[0] * iobj_1->Type->size[1];
  iobj_1->Type->size[0] = 1;
  iobj_1->Type->size[1] = 5;
  invers_emxEnsureCapacity_char_T(iobj_1->Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    iobj_1->Type->data[b_kstr] = tmp_3[b_kstr];
  }

  inverse_kinemati_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = iobj_1->Type->size[1];
  invers_emxEnsureCapacity_char_T(switch_expression, b_kstr);
  loop_ub = iobj_1->Type->size[1] - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = iobj_1->Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp_4[b_kstr];
  }

  b_bool = false;
  if (switch_expression->size[1] != 8) {
  } else {
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
    if (switch_expression->size[1] != 9) {
    } else {
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

  inverse_kinemati_emxFree_char_T(&switch_expression);
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
    iobj_1->VelocityNumber = 1.0;
    iobj_1->PositionNumber = 1.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 1.0;
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
    iobj_1->VelocityNumber = 1.0;
    iobj_1->PositionNumber = 1.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = 0;
    }

    poslim_data[0] = 0.0;
    poslim_data[1] = 0.0;
    iobj_1->VelocityNumber = 0.0;
    iobj_1->PositionNumber = 0.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 0.0;
    break;
  }

  b_kstr = iobj_1->MotionSubspace->size[0] * iobj_1->MotionSubspace->size[1];
  iobj_1->MotionSubspace->size[0] = 6;
  iobj_1->MotionSubspace->size[1] = 1;
  invers_emxEnsureCapacity_real_T(iobj_1->MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    iobj_1->MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  b_kstr = iobj_1->PositionLimitsInternal->size[0] *
    iobj_1->PositionLimitsInternal->size[1];
  iobj_1->PositionLimitsInternal->size[0] = 1;
  iobj_1->PositionLimitsInternal->size[1] = 2;
  invers_emxEnsureCapacity_real_T(iobj_1->PositionLimitsInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 2; b_kstr++) {
    iobj_1->PositionLimitsInternal->data[b_kstr] = poslim_data[b_kstr];
  }

  b_kstr = iobj_1->HomePositionInternal->size[0];
  iobj_1->HomePositionInternal->size[0] = 1;
  invers_emxEnsureCapacity_real_T(iobj_1->HomePositionInternal, b_kstr);
  iobj_1->HomePositionInternal->data[0] = 0.0;
  obj->JointInternal = iobj_1;
  obj->Index = -1.0;
  obj->ParentIndex = -1.0;
  obj->MassInternal = 1.0;
  obj->CenterOfMassInternal[0] = 0.0;
  obj->CenterOfMassInternal[1] = 0.0;
  obj->CenterOfMassInternal[2] = 0.0;
  for (b_kstr = 0; b_kstr < 9; b_kstr++) {
    b_I[b_kstr] = 0;
  }

  b_I[0] = 1;
  b_I[4] = 1;
  b_I[8] = 1;
  for (b_kstr = 0; b_kstr < 9; b_kstr++) {
    obj->InertiaInternal[b_kstr] = b_I[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 36; b_kstr++) {
    msubspace_data[b_kstr] = 0;
  }

  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    msubspace_data[b_kstr + 6 * b_kstr] = 1;
  }

  for (b_kstr = 0; b_kstr < 36; b_kstr++) {
    obj->SpatialInertia[b_kstr] = msubspace_data[b_kstr];
  }

  obj->CollisionsInternal = inver_CollisionSet_CollisionSet(iobj_0, 0.0);
  return b_obj;
}

static t_robotics_manip_internal_Rig_T *inverse_k_RigidBody_RigidBody_g
  (t_robotics_manip_internal_Rig_T *obj, l_robotics_manip_internal_Col_T *iobj_0,
   c_rigidBodyJoint_inverse_kine_T *iobj_1)
{
  emxArray_char_T_inverse_kinem_T *switch_expression;
  t_robotics_manip_internal_Rig_T *b_obj;
  real_T poslim_data[12];
  int32_T b_kstr;
  int32_T loop_ub;
  char_T b_0[9];
  char_T b[8];
  int8_T msubspace_data[36];
  int8_T b_I[9];
  int8_T tmp[6];
  boolean_T b_bool;
  static const char_T tmp_0[10] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '2' };

  static const int8_T tmp_1[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    1 };

  static const char_T tmp_2[14] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '2', '_', 'j', 'n', 't' };

  static const char_T tmp_3[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_4[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_5[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  int32_T exitg1;
  b_obj = obj;
  b_kstr = obj->NameInternal->size[0] * obj->NameInternal->size[1];
  obj->NameInternal->size[0] = 1;
  obj->NameInternal->size[1] = 10;
  invers_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 10; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp_0[b_kstr];
  }

  iobj_1->InTree = false;
  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    iobj_1->JointToParentTransform[b_kstr] = tmp_1[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    iobj_1->ChildToJointTransform[b_kstr] = tmp_1[b_kstr];
  }

  b_kstr = iobj_1->NameInternal->size[0] * iobj_1->NameInternal->size[1];
  iobj_1->NameInternal->size[0] = 1;
  iobj_1->NameInternal->size[1] = 14;
  invers_emxEnsureCapacity_char_T(iobj_1->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 14; b_kstr++) {
    iobj_1->NameInternal->data[b_kstr] = tmp_2[b_kstr];
  }

  b_kstr = iobj_1->Type->size[0] * iobj_1->Type->size[1];
  iobj_1->Type->size[0] = 1;
  iobj_1->Type->size[1] = 5;
  invers_emxEnsureCapacity_char_T(iobj_1->Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    iobj_1->Type->data[b_kstr] = tmp_3[b_kstr];
  }

  inverse_kinemati_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = iobj_1->Type->size[1];
  invers_emxEnsureCapacity_char_T(switch_expression, b_kstr);
  loop_ub = iobj_1->Type->size[1] - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = iobj_1->Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp_4[b_kstr];
  }

  b_bool = false;
  if (switch_expression->size[1] != 8) {
  } else {
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
    if (switch_expression->size[1] != 9) {
    } else {
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

  inverse_kinemati_emxFree_char_T(&switch_expression);
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
    iobj_1->VelocityNumber = 1.0;
    iobj_1->PositionNumber = 1.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 1.0;
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
    iobj_1->VelocityNumber = 1.0;
    iobj_1->PositionNumber = 1.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = 0;
    }

    poslim_data[0] = 0.0;
    poslim_data[1] = 0.0;
    iobj_1->VelocityNumber = 0.0;
    iobj_1->PositionNumber = 0.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 0.0;
    break;
  }

  b_kstr = iobj_1->MotionSubspace->size[0] * iobj_1->MotionSubspace->size[1];
  iobj_1->MotionSubspace->size[0] = 6;
  iobj_1->MotionSubspace->size[1] = 1;
  invers_emxEnsureCapacity_real_T(iobj_1->MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    iobj_1->MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  b_kstr = iobj_1->PositionLimitsInternal->size[0] *
    iobj_1->PositionLimitsInternal->size[1];
  iobj_1->PositionLimitsInternal->size[0] = 1;
  iobj_1->PositionLimitsInternal->size[1] = 2;
  invers_emxEnsureCapacity_real_T(iobj_1->PositionLimitsInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 2; b_kstr++) {
    iobj_1->PositionLimitsInternal->data[b_kstr] = poslim_data[b_kstr];
  }

  b_kstr = iobj_1->HomePositionInternal->size[0];
  iobj_1->HomePositionInternal->size[0] = 1;
  invers_emxEnsureCapacity_real_T(iobj_1->HomePositionInternal, b_kstr);
  iobj_1->HomePositionInternal->data[0] = 0.0;
  obj->JointInternal = iobj_1;
  obj->Index = -1.0;
  obj->ParentIndex = -1.0;
  obj->MassInternal = 1.0;
  obj->CenterOfMassInternal[0] = 0.0;
  obj->CenterOfMassInternal[1] = 0.0;
  obj->CenterOfMassInternal[2] = 0.0;
  for (b_kstr = 0; b_kstr < 9; b_kstr++) {
    b_I[b_kstr] = 0;
  }

  b_I[0] = 1;
  b_I[4] = 1;
  b_I[8] = 1;
  for (b_kstr = 0; b_kstr < 9; b_kstr++) {
    obj->InertiaInternal[b_kstr] = b_I[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 36; b_kstr++) {
    msubspace_data[b_kstr] = 0;
  }

  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    msubspace_data[b_kstr + 6 * b_kstr] = 1;
  }

  for (b_kstr = 0; b_kstr < 36; b_kstr++) {
    obj->SpatialInertia[b_kstr] = msubspace_data[b_kstr];
  }

  obj->CollisionsInternal = inver_CollisionSet_CollisionSet(iobj_0, 0.0);
  return b_obj;
}

static t_robotics_manip_internal_Rig_T *inverse__RigidBody_RigidBody_ga
  (t_robotics_manip_internal_Rig_T *obj, l_robotics_manip_internal_Col_T *iobj_0,
   c_rigidBodyJoint_inverse_kine_T *iobj_1)
{
  emxArray_char_T_inverse_kinem_T *switch_expression;
  t_robotics_manip_internal_Rig_T *b_obj;
  real_T poslim_data[12];
  int32_T b_kstr;
  int32_T loop_ub;
  char_T b_0[9];
  char_T b[8];
  int8_T msubspace_data[36];
  int8_T b_I[9];
  int8_T tmp[6];
  boolean_T b_bool;
  static const char_T tmp_0[10] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '3' };

  static const int8_T tmp_1[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    1 };

  static const char_T tmp_2[14] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '3', '_', 'j', 'n', 't' };

  static const char_T tmp_3[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_4[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_5[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  int32_T exitg1;
  b_obj = obj;
  b_kstr = obj->NameInternal->size[0] * obj->NameInternal->size[1];
  obj->NameInternal->size[0] = 1;
  obj->NameInternal->size[1] = 10;
  invers_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 10; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp_0[b_kstr];
  }

  iobj_1->InTree = false;
  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    iobj_1->JointToParentTransform[b_kstr] = tmp_1[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    iobj_1->ChildToJointTransform[b_kstr] = tmp_1[b_kstr];
  }

  b_kstr = iobj_1->NameInternal->size[0] * iobj_1->NameInternal->size[1];
  iobj_1->NameInternal->size[0] = 1;
  iobj_1->NameInternal->size[1] = 14;
  invers_emxEnsureCapacity_char_T(iobj_1->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 14; b_kstr++) {
    iobj_1->NameInternal->data[b_kstr] = tmp_2[b_kstr];
  }

  b_kstr = iobj_1->Type->size[0] * iobj_1->Type->size[1];
  iobj_1->Type->size[0] = 1;
  iobj_1->Type->size[1] = 5;
  invers_emxEnsureCapacity_char_T(iobj_1->Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    iobj_1->Type->data[b_kstr] = tmp_3[b_kstr];
  }

  inverse_kinemati_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = iobj_1->Type->size[1];
  invers_emxEnsureCapacity_char_T(switch_expression, b_kstr);
  loop_ub = iobj_1->Type->size[1] - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = iobj_1->Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp_4[b_kstr];
  }

  b_bool = false;
  if (switch_expression->size[1] != 8) {
  } else {
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
    if (switch_expression->size[1] != 9) {
    } else {
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

  inverse_kinemati_emxFree_char_T(&switch_expression);
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
    iobj_1->VelocityNumber = 1.0;
    iobj_1->PositionNumber = 1.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 1.0;
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
    iobj_1->VelocityNumber = 1.0;
    iobj_1->PositionNumber = 1.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = 0;
    }

    poslim_data[0] = 0.0;
    poslim_data[1] = 0.0;
    iobj_1->VelocityNumber = 0.0;
    iobj_1->PositionNumber = 0.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 0.0;
    break;
  }

  b_kstr = iobj_1->MotionSubspace->size[0] * iobj_1->MotionSubspace->size[1];
  iobj_1->MotionSubspace->size[0] = 6;
  iobj_1->MotionSubspace->size[1] = 1;
  invers_emxEnsureCapacity_real_T(iobj_1->MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    iobj_1->MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  b_kstr = iobj_1->PositionLimitsInternal->size[0] *
    iobj_1->PositionLimitsInternal->size[1];
  iobj_1->PositionLimitsInternal->size[0] = 1;
  iobj_1->PositionLimitsInternal->size[1] = 2;
  invers_emxEnsureCapacity_real_T(iobj_1->PositionLimitsInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 2; b_kstr++) {
    iobj_1->PositionLimitsInternal->data[b_kstr] = poslim_data[b_kstr];
  }

  b_kstr = iobj_1->HomePositionInternal->size[0];
  iobj_1->HomePositionInternal->size[0] = 1;
  invers_emxEnsureCapacity_real_T(iobj_1->HomePositionInternal, b_kstr);
  iobj_1->HomePositionInternal->data[0] = 0.0;
  obj->JointInternal = iobj_1;
  obj->Index = -1.0;
  obj->ParentIndex = -1.0;
  obj->MassInternal = 1.0;
  obj->CenterOfMassInternal[0] = 0.0;
  obj->CenterOfMassInternal[1] = 0.0;
  obj->CenterOfMassInternal[2] = 0.0;
  for (b_kstr = 0; b_kstr < 9; b_kstr++) {
    b_I[b_kstr] = 0;
  }

  b_I[0] = 1;
  b_I[4] = 1;
  b_I[8] = 1;
  for (b_kstr = 0; b_kstr < 9; b_kstr++) {
    obj->InertiaInternal[b_kstr] = b_I[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 36; b_kstr++) {
    msubspace_data[b_kstr] = 0;
  }

  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    msubspace_data[b_kstr + 6 * b_kstr] = 1;
  }

  for (b_kstr = 0; b_kstr < 36; b_kstr++) {
    obj->SpatialInertia[b_kstr] = msubspace_data[b_kstr];
  }

  obj->CollisionsInternal = inver_CollisionSet_CollisionSet(iobj_0, 0.0);
  return b_obj;
}

static t_robotics_manip_internal_Rig_T *inverse_RigidBody_RigidBody_gaq
  (t_robotics_manip_internal_Rig_T *obj, l_robotics_manip_internal_Col_T *iobj_0,
   c_rigidBodyJoint_inverse_kine_T *iobj_1)
{
  emxArray_char_T_inverse_kinem_T *switch_expression;
  t_robotics_manip_internal_Rig_T *b_obj;
  real_T poslim_data[12];
  int32_T b_kstr;
  int32_T loop_ub;
  char_T b_0[9];
  char_T b[8];
  int8_T msubspace_data[36];
  int8_T b_I[9];
  int8_T tmp[6];
  boolean_T b_bool;
  static const char_T tmp_0[10] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '4' };

  static const int8_T tmp_1[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    1 };

  static const char_T tmp_2[14] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '4', '_', 'j', 'n', 't' };

  static const char_T tmp_3[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_4[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_5[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  int32_T exitg1;
  b_obj = obj;
  b_kstr = obj->NameInternal->size[0] * obj->NameInternal->size[1];
  obj->NameInternal->size[0] = 1;
  obj->NameInternal->size[1] = 10;
  invers_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 10; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp_0[b_kstr];
  }

  iobj_1->InTree = false;
  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    iobj_1->JointToParentTransform[b_kstr] = tmp_1[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    iobj_1->ChildToJointTransform[b_kstr] = tmp_1[b_kstr];
  }

  b_kstr = iobj_1->NameInternal->size[0] * iobj_1->NameInternal->size[1];
  iobj_1->NameInternal->size[0] = 1;
  iobj_1->NameInternal->size[1] = 14;
  invers_emxEnsureCapacity_char_T(iobj_1->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 14; b_kstr++) {
    iobj_1->NameInternal->data[b_kstr] = tmp_2[b_kstr];
  }

  b_kstr = iobj_1->Type->size[0] * iobj_1->Type->size[1];
  iobj_1->Type->size[0] = 1;
  iobj_1->Type->size[1] = 5;
  invers_emxEnsureCapacity_char_T(iobj_1->Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    iobj_1->Type->data[b_kstr] = tmp_3[b_kstr];
  }

  inverse_kinemati_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = iobj_1->Type->size[1];
  invers_emxEnsureCapacity_char_T(switch_expression, b_kstr);
  loop_ub = iobj_1->Type->size[1] - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = iobj_1->Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp_4[b_kstr];
  }

  b_bool = false;
  if (switch_expression->size[1] != 8) {
  } else {
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
    if (switch_expression->size[1] != 9) {
    } else {
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

  inverse_kinemati_emxFree_char_T(&switch_expression);
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
    iobj_1->VelocityNumber = 1.0;
    iobj_1->PositionNumber = 1.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 1.0;
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
    iobj_1->VelocityNumber = 1.0;
    iobj_1->PositionNumber = 1.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = 0;
    }

    poslim_data[0] = 0.0;
    poslim_data[1] = 0.0;
    iobj_1->VelocityNumber = 0.0;
    iobj_1->PositionNumber = 0.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 0.0;
    break;
  }

  b_kstr = iobj_1->MotionSubspace->size[0] * iobj_1->MotionSubspace->size[1];
  iobj_1->MotionSubspace->size[0] = 6;
  iobj_1->MotionSubspace->size[1] = 1;
  invers_emxEnsureCapacity_real_T(iobj_1->MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    iobj_1->MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  b_kstr = iobj_1->PositionLimitsInternal->size[0] *
    iobj_1->PositionLimitsInternal->size[1];
  iobj_1->PositionLimitsInternal->size[0] = 1;
  iobj_1->PositionLimitsInternal->size[1] = 2;
  invers_emxEnsureCapacity_real_T(iobj_1->PositionLimitsInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 2; b_kstr++) {
    iobj_1->PositionLimitsInternal->data[b_kstr] = poslim_data[b_kstr];
  }

  b_kstr = iobj_1->HomePositionInternal->size[0];
  iobj_1->HomePositionInternal->size[0] = 1;
  invers_emxEnsureCapacity_real_T(iobj_1->HomePositionInternal, b_kstr);
  iobj_1->HomePositionInternal->data[0] = 0.0;
  obj->JointInternal = iobj_1;
  obj->Index = -1.0;
  obj->ParentIndex = -1.0;
  obj->MassInternal = 1.0;
  obj->CenterOfMassInternal[0] = 0.0;
  obj->CenterOfMassInternal[1] = 0.0;
  obj->CenterOfMassInternal[2] = 0.0;
  for (b_kstr = 0; b_kstr < 9; b_kstr++) {
    b_I[b_kstr] = 0;
  }

  b_I[0] = 1;
  b_I[4] = 1;
  b_I[8] = 1;
  for (b_kstr = 0; b_kstr < 9; b_kstr++) {
    obj->InertiaInternal[b_kstr] = b_I[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 36; b_kstr++) {
    msubspace_data[b_kstr] = 0;
  }

  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    msubspace_data[b_kstr + 6 * b_kstr] = 1;
  }

  for (b_kstr = 0; b_kstr < 36; b_kstr++) {
    obj->SpatialInertia[b_kstr] = msubspace_data[b_kstr];
  }

  obj->CollisionsInternal = inver_CollisionSet_CollisionSet(iobj_0, 0.0);
  return b_obj;
}

static t_robotics_manip_internal_Rig_T *invers_RigidBody_RigidBody_gaqy
  (t_robotics_manip_internal_Rig_T *obj, l_robotics_manip_internal_Col_T *iobj_0,
   c_rigidBodyJoint_inverse_kine_T *iobj_1)
{
  emxArray_char_T_inverse_kinem_T *switch_expression;
  t_robotics_manip_internal_Rig_T *b_obj;
  real_T poslim_data[12];
  int32_T b_kstr;
  int32_T loop_ub;
  char_T b_0[9];
  char_T b[8];
  int8_T msubspace_data[36];
  int8_T b_I[9];
  int8_T tmp[6];
  boolean_T b_bool;
  static const char_T tmp_0[10] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '5' };

  static const int8_T tmp_1[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    1 };

  static const char_T tmp_2[14] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '5', '_', 'j', 'n', 't' };

  static const char_T tmp_3[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_4[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_5[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  int32_T exitg1;
  b_obj = obj;
  b_kstr = obj->NameInternal->size[0] * obj->NameInternal->size[1];
  obj->NameInternal->size[0] = 1;
  obj->NameInternal->size[1] = 10;
  invers_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 10; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp_0[b_kstr];
  }

  iobj_1->InTree = false;
  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    iobj_1->JointToParentTransform[b_kstr] = tmp_1[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    iobj_1->ChildToJointTransform[b_kstr] = tmp_1[b_kstr];
  }

  b_kstr = iobj_1->NameInternal->size[0] * iobj_1->NameInternal->size[1];
  iobj_1->NameInternal->size[0] = 1;
  iobj_1->NameInternal->size[1] = 14;
  invers_emxEnsureCapacity_char_T(iobj_1->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 14; b_kstr++) {
    iobj_1->NameInternal->data[b_kstr] = tmp_2[b_kstr];
  }

  b_kstr = iobj_1->Type->size[0] * iobj_1->Type->size[1];
  iobj_1->Type->size[0] = 1;
  iobj_1->Type->size[1] = 5;
  invers_emxEnsureCapacity_char_T(iobj_1->Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    iobj_1->Type->data[b_kstr] = tmp_3[b_kstr];
  }

  inverse_kinemati_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = iobj_1->Type->size[1];
  invers_emxEnsureCapacity_char_T(switch_expression, b_kstr);
  loop_ub = iobj_1->Type->size[1] - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = iobj_1->Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp_4[b_kstr];
  }

  b_bool = false;
  if (switch_expression->size[1] != 8) {
  } else {
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
    if (switch_expression->size[1] != 9) {
    } else {
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

  inverse_kinemati_emxFree_char_T(&switch_expression);
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
    iobj_1->VelocityNumber = 1.0;
    iobj_1->PositionNumber = 1.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 1.0;
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
    iobj_1->VelocityNumber = 1.0;
    iobj_1->PositionNumber = 1.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = 0;
    }

    poslim_data[0] = 0.0;
    poslim_data[1] = 0.0;
    iobj_1->VelocityNumber = 0.0;
    iobj_1->PositionNumber = 0.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 0.0;
    break;
  }

  b_kstr = iobj_1->MotionSubspace->size[0] * iobj_1->MotionSubspace->size[1];
  iobj_1->MotionSubspace->size[0] = 6;
  iobj_1->MotionSubspace->size[1] = 1;
  invers_emxEnsureCapacity_real_T(iobj_1->MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    iobj_1->MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  b_kstr = iobj_1->PositionLimitsInternal->size[0] *
    iobj_1->PositionLimitsInternal->size[1];
  iobj_1->PositionLimitsInternal->size[0] = 1;
  iobj_1->PositionLimitsInternal->size[1] = 2;
  invers_emxEnsureCapacity_real_T(iobj_1->PositionLimitsInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 2; b_kstr++) {
    iobj_1->PositionLimitsInternal->data[b_kstr] = poslim_data[b_kstr];
  }

  b_kstr = iobj_1->HomePositionInternal->size[0];
  iobj_1->HomePositionInternal->size[0] = 1;
  invers_emxEnsureCapacity_real_T(iobj_1->HomePositionInternal, b_kstr);
  iobj_1->HomePositionInternal->data[0] = 0.0;
  obj->JointInternal = iobj_1;
  obj->Index = -1.0;
  obj->ParentIndex = -1.0;
  obj->MassInternal = 1.0;
  obj->CenterOfMassInternal[0] = 0.0;
  obj->CenterOfMassInternal[1] = 0.0;
  obj->CenterOfMassInternal[2] = 0.0;
  for (b_kstr = 0; b_kstr < 9; b_kstr++) {
    b_I[b_kstr] = 0;
  }

  b_I[0] = 1;
  b_I[4] = 1;
  b_I[8] = 1;
  for (b_kstr = 0; b_kstr < 9; b_kstr++) {
    obj->InertiaInternal[b_kstr] = b_I[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 36; b_kstr++) {
    msubspace_data[b_kstr] = 0;
  }

  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    msubspace_data[b_kstr + 6 * b_kstr] = 1;
  }

  for (b_kstr = 0; b_kstr < 36; b_kstr++) {
    obj->SpatialInertia[b_kstr] = msubspace_data[b_kstr];
  }

  obj->CollisionsInternal = inver_CollisionSet_CollisionSet(iobj_0, 0.0);
  return b_obj;
}

static t_robotics_manip_internal_Rig_T *inver_RigidBody_RigidBody_gaqyy
  (t_robotics_manip_internal_Rig_T *obj, l_robotics_manip_internal_Col_T *iobj_0,
   c_rigidBodyJoint_inverse_kine_T *iobj_1)
{
  emxArray_char_T_inverse_kinem_T *switch_expression;
  t_robotics_manip_internal_Rig_T *b_obj;
  real_T poslim_data[12];
  int32_T b_kstr;
  int32_T loop_ub;
  char_T b_0[9];
  char_T b[8];
  int8_T msubspace_data[36];
  int8_T b_I[9];
  int8_T tmp[6];
  boolean_T b_bool;
  static const char_T tmp_0[10] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '6' };

  static const int8_T tmp_1[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    1 };

  static const char_T tmp_2[14] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '6', '_', 'j', 'n', 't' };

  static const char_T tmp_3[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_4[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_5[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  int32_T exitg1;
  b_obj = obj;
  b_kstr = obj->NameInternal->size[0] * obj->NameInternal->size[1];
  obj->NameInternal->size[0] = 1;
  obj->NameInternal->size[1] = 10;
  invers_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 10; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp_0[b_kstr];
  }

  iobj_1->InTree = false;
  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    iobj_1->JointToParentTransform[b_kstr] = tmp_1[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    iobj_1->ChildToJointTransform[b_kstr] = tmp_1[b_kstr];
  }

  b_kstr = iobj_1->NameInternal->size[0] * iobj_1->NameInternal->size[1];
  iobj_1->NameInternal->size[0] = 1;
  iobj_1->NameInternal->size[1] = 14;
  invers_emxEnsureCapacity_char_T(iobj_1->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 14; b_kstr++) {
    iobj_1->NameInternal->data[b_kstr] = tmp_2[b_kstr];
  }

  b_kstr = iobj_1->Type->size[0] * iobj_1->Type->size[1];
  iobj_1->Type->size[0] = 1;
  iobj_1->Type->size[1] = 5;
  invers_emxEnsureCapacity_char_T(iobj_1->Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    iobj_1->Type->data[b_kstr] = tmp_3[b_kstr];
  }

  inverse_kinemati_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = iobj_1->Type->size[1];
  invers_emxEnsureCapacity_char_T(switch_expression, b_kstr);
  loop_ub = iobj_1->Type->size[1] - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = iobj_1->Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp_4[b_kstr];
  }

  b_bool = false;
  if (switch_expression->size[1] != 8) {
  } else {
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
    if (switch_expression->size[1] != 9) {
    } else {
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

  inverse_kinemati_emxFree_char_T(&switch_expression);
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
    iobj_1->VelocityNumber = 1.0;
    iobj_1->PositionNumber = 1.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 1.0;
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
    iobj_1->VelocityNumber = 1.0;
    iobj_1->PositionNumber = 1.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = 0;
    }

    poslim_data[0] = 0.0;
    poslim_data[1] = 0.0;
    iobj_1->VelocityNumber = 0.0;
    iobj_1->PositionNumber = 0.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 0.0;
    break;
  }

  b_kstr = iobj_1->MotionSubspace->size[0] * iobj_1->MotionSubspace->size[1];
  iobj_1->MotionSubspace->size[0] = 6;
  iobj_1->MotionSubspace->size[1] = 1;
  invers_emxEnsureCapacity_real_T(iobj_1->MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    iobj_1->MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  b_kstr = iobj_1->PositionLimitsInternal->size[0] *
    iobj_1->PositionLimitsInternal->size[1];
  iobj_1->PositionLimitsInternal->size[0] = 1;
  iobj_1->PositionLimitsInternal->size[1] = 2;
  invers_emxEnsureCapacity_real_T(iobj_1->PositionLimitsInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 2; b_kstr++) {
    iobj_1->PositionLimitsInternal->data[b_kstr] = poslim_data[b_kstr];
  }

  b_kstr = iobj_1->HomePositionInternal->size[0];
  iobj_1->HomePositionInternal->size[0] = 1;
  invers_emxEnsureCapacity_real_T(iobj_1->HomePositionInternal, b_kstr);
  iobj_1->HomePositionInternal->data[0] = 0.0;
  obj->JointInternal = iobj_1;
  obj->Index = -1.0;
  obj->ParentIndex = -1.0;
  obj->MassInternal = 1.0;
  obj->CenterOfMassInternal[0] = 0.0;
  obj->CenterOfMassInternal[1] = 0.0;
  obj->CenterOfMassInternal[2] = 0.0;
  for (b_kstr = 0; b_kstr < 9; b_kstr++) {
    b_I[b_kstr] = 0;
  }

  b_I[0] = 1;
  b_I[4] = 1;
  b_I[8] = 1;
  for (b_kstr = 0; b_kstr < 9; b_kstr++) {
    obj->InertiaInternal[b_kstr] = b_I[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 36; b_kstr++) {
    msubspace_data[b_kstr] = 0;
  }

  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    msubspace_data[b_kstr + 6 * b_kstr] = 1;
  }

  for (b_kstr = 0; b_kstr < 36; b_kstr++) {
    obj->SpatialInertia[b_kstr] = msubspace_data[b_kstr];
  }

  obj->CollisionsInternal = inver_CollisionSet_CollisionSet(iobj_0, 0.0);
  return b_obj;
}

static t_robotics_manip_internal_Rig_T *inve_RigidBody_RigidBody_gaqyy1
  (t_robotics_manip_internal_Rig_T *obj, l_robotics_manip_internal_Col_T *iobj_0,
   c_rigidBodyJoint_inverse_kine_T *iobj_1)
{
  emxArray_char_T_inverse_kinem_T *switch_expression;
  t_robotics_manip_internal_Rig_T *b_obj;
  static const char_T tmp[10] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '7' };

  static const int8_T tmp_0[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    1 };

  static const char_T tmp_1[14] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '7', '_', 'j', 'n', 't' };

  static const char_T tmp_2[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_3[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_4[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  int32_T exitg1;
  b_obj = obj;
  inverse_kinematics_B.i6 = obj->NameInternal->size[0] * obj->NameInternal->
    size[1];
  obj->NameInternal->size[0] = 1;
  obj->NameInternal->size[1] = 10;
  invers_emxEnsureCapacity_char_T(obj->NameInternal, inverse_kinematics_B.i6);
  for (inverse_kinematics_B.b_kstr_d = 0; inverse_kinematics_B.b_kstr_d < 10;
       inverse_kinematics_B.b_kstr_d++) {
    obj->NameInternal->data[inverse_kinematics_B.b_kstr_d] =
      tmp[inverse_kinematics_B.b_kstr_d];
  }

  iobj_1->InTree = false;
  for (inverse_kinematics_B.b_kstr_d = 0; inverse_kinematics_B.b_kstr_d < 16;
       inverse_kinematics_B.b_kstr_d++) {
    iobj_1->JointToParentTransform[inverse_kinematics_B.b_kstr_d] =
      tmp_0[inverse_kinematics_B.b_kstr_d];
  }

  for (inverse_kinematics_B.b_kstr_d = 0; inverse_kinematics_B.b_kstr_d < 16;
       inverse_kinematics_B.b_kstr_d++) {
    iobj_1->ChildToJointTransform[inverse_kinematics_B.b_kstr_d] =
      tmp_0[inverse_kinematics_B.b_kstr_d];
  }

  inverse_kinematics_B.i6 = iobj_1->NameInternal->size[0] * iobj_1->
    NameInternal->size[1];
  iobj_1->NameInternal->size[0] = 1;
  iobj_1->NameInternal->size[1] = 14;
  invers_emxEnsureCapacity_char_T(iobj_1->NameInternal, inverse_kinematics_B.i6);
  for (inverse_kinematics_B.b_kstr_d = 0; inverse_kinematics_B.b_kstr_d < 14;
       inverse_kinematics_B.b_kstr_d++) {
    iobj_1->NameInternal->data[inverse_kinematics_B.b_kstr_d] =
      tmp_1[inverse_kinematics_B.b_kstr_d];
  }

  inverse_kinematics_B.i6 = iobj_1->Type->size[0] * iobj_1->Type->size[1];
  iobj_1->Type->size[0] = 1;
  iobj_1->Type->size[1] = 5;
  invers_emxEnsureCapacity_char_T(iobj_1->Type, inverse_kinematics_B.i6);
  for (inverse_kinematics_B.b_kstr_d = 0; inverse_kinematics_B.b_kstr_d < 5;
       inverse_kinematics_B.b_kstr_d++) {
    iobj_1->Type->data[inverse_kinematics_B.b_kstr_d] =
      tmp_2[inverse_kinematics_B.b_kstr_d];
  }

  inverse_kinemati_emxInit_char_T(&switch_expression, 2);
  inverse_kinematics_B.i6 = switch_expression->size[0] * switch_expression->
    size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = iobj_1->Type->size[1];
  invers_emxEnsureCapacity_char_T(switch_expression, inverse_kinematics_B.i6);
  inverse_kinematics_B.loop_ub_af = iobj_1->Type->size[1] - 1;
  for (inverse_kinematics_B.b_kstr_d = 0; inverse_kinematics_B.b_kstr_d <=
       inverse_kinematics_B.loop_ub_af; inverse_kinematics_B.b_kstr_d++) {
    inverse_kinematics_B.i6 = inverse_kinematics_B.b_kstr_d;
    switch_expression->data[inverse_kinematics_B.i6] = iobj_1->Type->
      data[inverse_kinematics_B.i6];
  }

  for (inverse_kinematics_B.b_kstr_d = 0; inverse_kinematics_B.b_kstr_d < 8;
       inverse_kinematics_B.b_kstr_d++) {
    inverse_kinematics_B.b_n[inverse_kinematics_B.b_kstr_d] =
      tmp_3[inverse_kinematics_B.b_kstr_d];
  }

  inverse_kinematics_B.b_bool_i = false;
  if (switch_expression->size[1] != 8) {
  } else {
    inverse_kinematics_B.b_kstr_d = 1;
    do {
      exitg1 = 0;
      if (inverse_kinematics_B.b_kstr_d - 1 < 8) {
        if (switch_expression->data[inverse_kinematics_B.b_kstr_d - 1] !=
            inverse_kinematics_B.b_n[inverse_kinematics_B.b_kstr_d - 1]) {
          exitg1 = 1;
        } else {
          inverse_kinematics_B.b_kstr_d++;
        }
      } else {
        inverse_kinematics_B.b_bool_i = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (inverse_kinematics_B.b_bool_i) {
    inverse_kinematics_B.b_kstr_d = 0;
  } else {
    for (inverse_kinematics_B.b_kstr_d = 0; inverse_kinematics_B.b_kstr_d < 9;
         inverse_kinematics_B.b_kstr_d++) {
      inverse_kinematics_B.b_j[inverse_kinematics_B.b_kstr_d] =
        tmp_4[inverse_kinematics_B.b_kstr_d];
    }

    inverse_kinematics_B.b_bool_i = false;
    if (switch_expression->size[1] != 9) {
    } else {
      inverse_kinematics_B.b_kstr_d = 1;
      do {
        exitg1 = 0;
        if (inverse_kinematics_B.b_kstr_d - 1 < 9) {
          if (switch_expression->data[inverse_kinematics_B.b_kstr_d - 1] !=
              inverse_kinematics_B.b_j[inverse_kinematics_B.b_kstr_d - 1]) {
            exitg1 = 1;
          } else {
            inverse_kinematics_B.b_kstr_d++;
          }
        } else {
          inverse_kinematics_B.b_bool_i = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (inverse_kinematics_B.b_bool_i) {
      inverse_kinematics_B.b_kstr_d = 1;
    } else {
      inverse_kinematics_B.b_kstr_d = -1;
    }
  }

  inverse_kinemati_emxFree_char_T(&switch_expression);
  switch (inverse_kinematics_B.b_kstr_d) {
   case 0:
    inverse_kinematics_B.iv4[0] = 0;
    inverse_kinematics_B.iv4[1] = 0;
    inverse_kinematics_B.iv4[2] = 1;
    inverse_kinematics_B.iv4[3] = 0;
    inverse_kinematics_B.iv4[4] = 0;
    inverse_kinematics_B.iv4[5] = 0;
    for (inverse_kinematics_B.b_kstr_d = 0; inverse_kinematics_B.b_kstr_d < 6;
         inverse_kinematics_B.b_kstr_d++) {
      inverse_kinematics_B.msubspace_data_d[inverse_kinematics_B.b_kstr_d] =
        inverse_kinematics_B.iv4[inverse_kinematics_B.b_kstr_d];
    }

    inverse_kinematics_B.poslim_data_g[0] = -3.1415926535897931;
    inverse_kinematics_B.poslim_data_g[1] = 3.1415926535897931;
    iobj_1->VelocityNumber = 1.0;
    iobj_1->PositionNumber = 1.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 1.0;
    break;

   case 1:
    inverse_kinematics_B.iv4[0] = 0;
    inverse_kinematics_B.iv4[1] = 0;
    inverse_kinematics_B.iv4[2] = 0;
    inverse_kinematics_B.iv4[3] = 0;
    inverse_kinematics_B.iv4[4] = 0;
    inverse_kinematics_B.iv4[5] = 1;
    for (inverse_kinematics_B.b_kstr_d = 0; inverse_kinematics_B.b_kstr_d < 6;
         inverse_kinematics_B.b_kstr_d++) {
      inverse_kinematics_B.msubspace_data_d[inverse_kinematics_B.b_kstr_d] =
        inverse_kinematics_B.iv4[inverse_kinematics_B.b_kstr_d];
    }

    inverse_kinematics_B.poslim_data_g[0] = -0.5;
    inverse_kinematics_B.poslim_data_g[1] = 0.5;
    iobj_1->VelocityNumber = 1.0;
    iobj_1->PositionNumber = 1.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 1.0;
    break;

   default:
    for (inverse_kinematics_B.b_kstr_d = 0; inverse_kinematics_B.b_kstr_d < 6;
         inverse_kinematics_B.b_kstr_d++) {
      inverse_kinematics_B.msubspace_data_d[inverse_kinematics_B.b_kstr_d] = 0;
    }

    inverse_kinematics_B.poslim_data_g[0] = 0.0;
    inverse_kinematics_B.poslim_data_g[1] = 0.0;
    iobj_1->VelocityNumber = 0.0;
    iobj_1->PositionNumber = 0.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 0.0;
    break;
  }

  inverse_kinematics_B.i6 = iobj_1->MotionSubspace->size[0] *
    iobj_1->MotionSubspace->size[1];
  iobj_1->MotionSubspace->size[0] = 6;
  iobj_1->MotionSubspace->size[1] = 1;
  invers_emxEnsureCapacity_real_T(iobj_1->MotionSubspace,
    inverse_kinematics_B.i6);
  for (inverse_kinematics_B.b_kstr_d = 0; inverse_kinematics_B.b_kstr_d < 6;
       inverse_kinematics_B.b_kstr_d++) {
    iobj_1->MotionSubspace->data[inverse_kinematics_B.b_kstr_d] =
      inverse_kinematics_B.msubspace_data_d[inverse_kinematics_B.b_kstr_d];
  }

  inverse_kinematics_B.i6 = iobj_1->PositionLimitsInternal->size[0] *
    iobj_1->PositionLimitsInternal->size[1];
  iobj_1->PositionLimitsInternal->size[0] = 1;
  iobj_1->PositionLimitsInternal->size[1] = 2;
  invers_emxEnsureCapacity_real_T(iobj_1->PositionLimitsInternal,
    inverse_kinematics_B.i6);
  for (inverse_kinematics_B.b_kstr_d = 0; inverse_kinematics_B.b_kstr_d < 2;
       inverse_kinematics_B.b_kstr_d++) {
    iobj_1->PositionLimitsInternal->data[inverse_kinematics_B.b_kstr_d] =
      inverse_kinematics_B.poslim_data_g[inverse_kinematics_B.b_kstr_d];
  }

  inverse_kinematics_B.i6 = iobj_1->HomePositionInternal->size[0];
  iobj_1->HomePositionInternal->size[0] = 1;
  invers_emxEnsureCapacity_real_T(iobj_1->HomePositionInternal,
    inverse_kinematics_B.i6);
  iobj_1->HomePositionInternal->data[0] = 0.0;
  obj->JointInternal = iobj_1;
  obj->Index = -1.0;
  obj->ParentIndex = -1.0;
  obj->MassInternal = 1.0;
  obj->CenterOfMassInternal[0] = 0.0;
  obj->CenterOfMassInternal[1] = 0.0;
  obj->CenterOfMassInternal[2] = 0.0;
  for (inverse_kinematics_B.b_kstr_d = 0; inverse_kinematics_B.b_kstr_d < 9;
       inverse_kinematics_B.b_kstr_d++) {
    inverse_kinematics_B.b_I_bj[inverse_kinematics_B.b_kstr_d] = 0;
  }

  inverse_kinematics_B.b_I_bj[0] = 1;
  inverse_kinematics_B.b_I_bj[4] = 1;
  inverse_kinematics_B.b_I_bj[8] = 1;
  for (inverse_kinematics_B.b_kstr_d = 0; inverse_kinematics_B.b_kstr_d < 9;
       inverse_kinematics_B.b_kstr_d++) {
    obj->InertiaInternal[inverse_kinematics_B.b_kstr_d] =
      inverse_kinematics_B.b_I_bj[inverse_kinematics_B.b_kstr_d];
  }

  for (inverse_kinematics_B.b_kstr_d = 0; inverse_kinematics_B.b_kstr_d < 36;
       inverse_kinematics_B.b_kstr_d++) {
    inverse_kinematics_B.msubspace_data_d[inverse_kinematics_B.b_kstr_d] = 0;
  }

  for (inverse_kinematics_B.b_kstr_d = 0; inverse_kinematics_B.b_kstr_d < 6;
       inverse_kinematics_B.b_kstr_d++) {
    inverse_kinematics_B.msubspace_data_d[inverse_kinematics_B.b_kstr_d + 6 *
      inverse_kinematics_B.b_kstr_d] = 1;
  }

  for (inverse_kinematics_B.b_kstr_d = 0; inverse_kinematics_B.b_kstr_d < 36;
       inverse_kinematics_B.b_kstr_d++) {
    obj->SpatialInertia[inverse_kinematics_B.b_kstr_d] =
      inverse_kinematics_B.msubspace_data_d[inverse_kinematics_B.b_kstr_d];
  }

  obj->CollisionsInternal = inver_CollisionSet_CollisionSet(iobj_0, 0.0);
  return b_obj;
}

static t_robotics_manip_internal_Rig_T *inv_RigidBody_RigidBody_gaqyy1n
  (t_robotics_manip_internal_Rig_T *obj, l_robotics_manip_internal_Col_T *iobj_0,
   c_rigidBodyJoint_inverse_kine_T *iobj_1)
{
  emxArray_char_T_inverse_kinem_T *switch_expression;
  t_robotics_manip_internal_Rig_T *b_obj;
  real_T poslim_data[12];
  int32_T b_kstr;
  int32_T loop_ub;
  char_T b_0[9];
  char_T b[8];
  int8_T msubspace_data[36];
  int8_T tmp[6];
  boolean_T b_bool;
  static const char_T tmp_0[5] = { 'B', 'o', 'd', 'y', '3' };

  static const real_T tmp_1[9] = { 0.0046677770146054676, 6.0325713613339561E-11,
    -1.4423351048665928E-11, 6.0325713613339561E-11, 0.00056414137071986029,
    0.00066778964460177833, -1.4423351048665928E-11, 0.00066778964460177833,
    0.0043215016438855995 };

  static const real_T tmp_2[36] = { 0.0046677770146054676,
    6.0325713613339561E-11, -1.4423351048665928E-11, 0.0, 0.007515489174200001,
    0.0314335584052, 6.0325713613339561E-11, 0.00056414137071986029,
    0.00066778964460177833, -0.007515489174200001, 0.0, 7.876920876999998E-10,
    -1.4423351048665928E-11, 0.00066778964460177833, 0.0043215016438855995,
    -0.0314335584052, -7.876920876999998E-10, 0.0, 0.0, -0.007515489174200001,
    -0.0314335584052, 0.410438, 0.0, 0.0, 0.007515489174200001, 0.0,
    -7.876920876999998E-10, 0.0, 0.410438, 0.0, 0.0314335584052,
    7.876920876999998E-10, 0.0, 0.0, 0.0, 0.410438 };

  static const int8_T tmp_3[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    1 };

  static const char_T tmp_4[6] = { 'J', 'o', 'i', 'n', 't', '3' };

  static const char_T tmp_5[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_6[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const real_T tmp_7[16] = { -3.6732051031245293E-6, 0.99999999999325373,
    -0.0, 0.0, 3.673205103099749E-6, 1.3492435729620085E-11,
    -0.99999999999325373, 0.0, -0.99999999998650746, -3.673205103099749E-6,
    -3.6732051031245293E-6, 0.0, -0.018, -1.0432626984524518E-14, -0.116, 1.0 };

  static const real_T tmp_8[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  static const real_T tmp_9[36] = { 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  int32_T exitg1;
  b_obj = obj;
  b_kstr = obj->NameInternal->size[0] * obj->NameInternal->size[1];
  obj->NameInternal->size[0] = 1;
  obj->NameInternal->size[1] = 5;
  invers_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp_0[b_kstr];
  }

  obj->ParentIndex = 2.0;
  obj->MassInternal = 0.410438;
  obj->CenterOfMassInternal[0] = -1.9191499999999995E-9;
  obj->CenterOfMassInternal[1] = 0.0765854;
  obj->CenterOfMassInternal[2] = -0.0183109;
  for (b_kstr = 0; b_kstr < 9; b_kstr++) {
    obj->InertiaInternal[b_kstr] = tmp_1[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 36; b_kstr++) {
    obj->SpatialInertia[b_kstr] = tmp_2[b_kstr];
  }

  iobj_1->InTree = false;
  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    iobj_1->JointToParentTransform[b_kstr] = tmp_3[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    iobj_1->ChildToJointTransform[b_kstr] = tmp_3[b_kstr];
  }

  b_kstr = iobj_1->NameInternal->size[0] * iobj_1->NameInternal->size[1];
  iobj_1->NameInternal->size[0] = 1;
  iobj_1->NameInternal->size[1] = 6;
  invers_emxEnsureCapacity_char_T(iobj_1->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    iobj_1->NameInternal->data[b_kstr] = tmp_4[b_kstr];
  }

  b_kstr = iobj_1->Type->size[0] * iobj_1->Type->size[1];
  iobj_1->Type->size[0] = 1;
  iobj_1->Type->size[1] = 8;
  invers_emxEnsureCapacity_char_T(iobj_1->Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    iobj_1->Type->data[b_kstr] = tmp_5[b_kstr];
  }

  inverse_kinemati_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = iobj_1->Type->size[1];
  invers_emxEnsureCapacity_char_T(switch_expression, b_kstr);
  loop_ub = iobj_1->Type->size[1] - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = iobj_1->Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp_5[b_kstr];
  }

  b_bool = false;
  if (switch_expression->size[1] != 8) {
  } else {
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
    if (switch_expression->size[1] != 9) {
    } else {
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

  inverse_kinemati_emxFree_char_T(&switch_expression);
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
    iobj_1->VelocityNumber = 1.0;
    iobj_1->PositionNumber = 1.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 1.0;
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
    iobj_1->VelocityNumber = 1.0;
    iobj_1->PositionNumber = 1.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = 0;
    }

    poslim_data[0] = 0.0;
    poslim_data[1] = 0.0;
    iobj_1->VelocityNumber = 0.0;
    iobj_1->PositionNumber = 0.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 0.0;
    break;
  }

  b_kstr = iobj_1->MotionSubspace->size[0] * iobj_1->MotionSubspace->size[1];
  iobj_1->MotionSubspace->size[0] = 6;
  iobj_1->MotionSubspace->size[1] = 1;
  invers_emxEnsureCapacity_real_T(iobj_1->MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    iobj_1->MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  b_kstr = iobj_1->PositionLimitsInternal->size[0] *
    iobj_1->PositionLimitsInternal->size[1];
  iobj_1->PositionLimitsInternal->size[0] = 1;
  iobj_1->PositionLimitsInternal->size[1] = 2;
  invers_emxEnsureCapacity_real_T(iobj_1->PositionLimitsInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 2; b_kstr++) {
    iobj_1->PositionLimitsInternal->data[b_kstr] = poslim_data[b_kstr];
  }

  b_kstr = iobj_1->HomePositionInternal->size[0];
  iobj_1->HomePositionInternal->size[0] = 1;
  invers_emxEnsureCapacity_real_T(iobj_1->HomePositionInternal, b_kstr);
  iobj_1->HomePositionInternal->data[0] = 0.0;
  obj->JointInternal = iobj_1;
  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal->JointToParentTransform[b_kstr] = tmp_7[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal->ChildToJointTransform[b_kstr] = tmp_8[b_kstr];
  }

  b_kstr = obj->JointInternal->MotionSubspace->size[0] * obj->
    JointInternal->MotionSubspace->size[1];
  obj->JointInternal->MotionSubspace->size[0] = 6;
  obj->JointInternal->MotionSubspace->size[1] = 1;
  invers_emxEnsureCapacity_real_T(obj->JointInternal->MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal->MotionSubspace->data[b_kstr] = tmp_9[b_kstr];
  }

  obj->JointInternal->InTree = true;
  b_kstr = obj->JointInternal->PositionLimitsInternal->size[0] *
    obj->JointInternal->PositionLimitsInternal->size[1];
  obj->JointInternal->PositionLimitsInternal->size[0] = 1;
  obj->JointInternal->PositionLimitsInternal->size[1] = 2;
  invers_emxEnsureCapacity_real_T(obj->JointInternal->PositionLimitsInternal,
    b_kstr);
  obj->JointInternal->PositionLimitsInternal->data[0] = -1.5708;
  obj->JointInternal->PositionLimitsInternal->data[obj->
    JointInternal->PositionLimitsInternal->size[0]] = 0.698132;
  obj->JointInternal->JointAxisInternal[0] = 0.0;
  obj->JointInternal->JointAxisInternal[1] = 0.0;
  obj->JointInternal->JointAxisInternal[2] = 1.0;
  b_kstr = obj->JointInternal->HomePositionInternal->size[0];
  obj->JointInternal->HomePositionInternal->size[0] = 1;
  invers_emxEnsureCapacity_real_T(obj->JointInternal->HomePositionInternal,
    b_kstr);
  obj->JointInternal->HomePositionInternal->data[0] = 0.0;
  obj->CollisionsInternal = inver_CollisionSet_CollisionSet(iobj_0, 0.0);
  return b_obj;
}

static t_robotics_manip_internal_Rig_T *in_RigidBody_RigidBody_gaqyy1no
  (t_robotics_manip_internal_Rig_T *obj, l_robotics_manip_internal_Col_T *iobj_0,
   c_rigidBodyJoint_inverse_kine_T *iobj_1)
{
  emxArray_char_T_inverse_kinem_T *switch_expression;
  t_robotics_manip_internal_Rig_T *b_obj;
  real_T poslim_data[12];
  int32_T b_kstr;
  int32_T loop_ub;
  char_T b_0[9];
  char_T b[8];
  int8_T msubspace_data[36];
  int8_T tmp[6];
  boolean_T b_bool;
  static const char_T tmp_0[5] = { 'B', 'o', 'd', 'y', '4' };

  static const real_T tmp_1[9] = { 0.0045638174922413618, 4.8717513500972173E-11,
    -1.3910421065015089E-11, 4.8717513500972173E-11, 0.00051036521293464988,
    0.00086169248657773053, -1.3910421065015089E-11, 0.00086169248657773053,
    0.0042490582793066993 };

  static const real_T tmp_2[36] = { 0.0045638174922413618,
    4.8717513500972173E-11, -1.3910421065015089E-11, 0.0, 0.0076468943736000012,
    0.0313626438672, 4.8717513500972173E-11, 0.00051036521293464988,
    0.00086169248657773053, -0.0076468943736000012, 0.0, 1.0059843920400007E-9,
    -1.3910421065015089E-11, 0.00086169248657773053, 0.0042490582793066993,
    -0.0313626438672, -1.0059843920400007E-9, 0.0, 0.0, -0.0076468943736000012,
    -0.0313626438672, 0.416076, 0.0, 0.0, 0.0076468943736000012, 0.0,
    -1.0059843920400007E-9, 0.0, 0.416076, 0.0, 0.0313626438672,
    1.0059843920400007E-9, 0.0, 0.0, 0.0, 0.416076 };

  static const int8_T tmp_3[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    1 };

  static const char_T tmp_4[6] = { 'J', 'o', 'i', 'n', 't', '4' };

  static const char_T tmp_5[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_6[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const real_T tmp_7[16] = { -3.6732051040127077E-6, 0.99999999999325373,
    -0.0, 0.0, 0.999999999989733, 3.6732051039997752E-6, -2.65358979335273E-6,
    0.0, -2.6535897933348279E-6, -9.7471795728992745E-12, -0.99999999999647926,
    0.0, 1.443290000172386E-14, 0.16, -0.042, 1.0 };

  static const real_T tmp_8[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  static const real_T tmp_9[36] = { 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  int32_T exitg1;
  b_obj = obj;
  b_kstr = obj->NameInternal->size[0] * obj->NameInternal->size[1];
  obj->NameInternal->size[0] = 1;
  obj->NameInternal->size[1] = 5;
  invers_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp_0[b_kstr];
  }

  obj->ParentIndex = 3.0;
  obj->MassInternal = 0.416076;
  obj->CenterOfMassInternal[0] = -2.4177900000000014E-9;
  obj->CenterOfMassInternal[1] = 0.0753772;
  obj->CenterOfMassInternal[2] = -0.018378600000000002;
  for (b_kstr = 0; b_kstr < 9; b_kstr++) {
    obj->InertiaInternal[b_kstr] = tmp_1[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 36; b_kstr++) {
    obj->SpatialInertia[b_kstr] = tmp_2[b_kstr];
  }

  iobj_1->InTree = false;
  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    iobj_1->JointToParentTransform[b_kstr] = tmp_3[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    iobj_1->ChildToJointTransform[b_kstr] = tmp_3[b_kstr];
  }

  b_kstr = iobj_1->NameInternal->size[0] * iobj_1->NameInternal->size[1];
  iobj_1->NameInternal->size[0] = 1;
  iobj_1->NameInternal->size[1] = 6;
  invers_emxEnsureCapacity_char_T(iobj_1->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    iobj_1->NameInternal->data[b_kstr] = tmp_4[b_kstr];
  }

  b_kstr = iobj_1->Type->size[0] * iobj_1->Type->size[1];
  iobj_1->Type->size[0] = 1;
  iobj_1->Type->size[1] = 8;
  invers_emxEnsureCapacity_char_T(iobj_1->Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    iobj_1->Type->data[b_kstr] = tmp_5[b_kstr];
  }

  inverse_kinemati_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = iobj_1->Type->size[1];
  invers_emxEnsureCapacity_char_T(switch_expression, b_kstr);
  loop_ub = iobj_1->Type->size[1] - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = iobj_1->Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp_5[b_kstr];
  }

  b_bool = false;
  if (switch_expression->size[1] != 8) {
  } else {
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
    if (switch_expression->size[1] != 9) {
    } else {
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

  inverse_kinemati_emxFree_char_T(&switch_expression);
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
    iobj_1->VelocityNumber = 1.0;
    iobj_1->PositionNumber = 1.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 1.0;
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
    iobj_1->VelocityNumber = 1.0;
    iobj_1->PositionNumber = 1.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = 0;
    }

    poslim_data[0] = 0.0;
    poslim_data[1] = 0.0;
    iobj_1->VelocityNumber = 0.0;
    iobj_1->PositionNumber = 0.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 0.0;
    break;
  }

  b_kstr = iobj_1->MotionSubspace->size[0] * iobj_1->MotionSubspace->size[1];
  iobj_1->MotionSubspace->size[0] = 6;
  iobj_1->MotionSubspace->size[1] = 1;
  invers_emxEnsureCapacity_real_T(iobj_1->MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    iobj_1->MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  b_kstr = iobj_1->PositionLimitsInternal->size[0] *
    iobj_1->PositionLimitsInternal->size[1];
  iobj_1->PositionLimitsInternal->size[0] = 1;
  iobj_1->PositionLimitsInternal->size[1] = 2;
  invers_emxEnsureCapacity_real_T(iobj_1->PositionLimitsInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 2; b_kstr++) {
    iobj_1->PositionLimitsInternal->data[b_kstr] = poslim_data[b_kstr];
  }

  b_kstr = iobj_1->HomePositionInternal->size[0];
  iobj_1->HomePositionInternal->size[0] = 1;
  invers_emxEnsureCapacity_real_T(iobj_1->HomePositionInternal, b_kstr);
  iobj_1->HomePositionInternal->data[0] = 0.0;
  obj->JointInternal = iobj_1;
  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal->JointToParentTransform[b_kstr] = tmp_7[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal->ChildToJointTransform[b_kstr] = tmp_8[b_kstr];
  }

  b_kstr = obj->JointInternal->MotionSubspace->size[0] * obj->
    JointInternal->MotionSubspace->size[1];
  obj->JointInternal->MotionSubspace->size[0] = 6;
  obj->JointInternal->MotionSubspace->size[1] = 1;
  invers_emxEnsureCapacity_real_T(obj->JointInternal->MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal->MotionSubspace->data[b_kstr] = tmp_9[b_kstr];
  }

  obj->JointInternal->InTree = true;
  b_kstr = obj->JointInternal->PositionLimitsInternal->size[0] *
    obj->JointInternal->PositionLimitsInternal->size[1];
  obj->JointInternal->PositionLimitsInternal->size[0] = 1;
  obj->JointInternal->PositionLimitsInternal->size[1] = 2;
  invers_emxEnsureCapacity_real_T(obj->JointInternal->PositionLimitsInternal,
    b_kstr);
  obj->JointInternal->PositionLimitsInternal->data[0] = -1.5708;
  obj->JointInternal->PositionLimitsInternal->data[obj->
    JointInternal->PositionLimitsInternal->size[0]] = 1.0472;
  obj->JointInternal->JointAxisInternal[0] = 0.0;
  obj->JointInternal->JointAxisInternal[1] = 0.0;
  obj->JointInternal->JointAxisInternal[2] = 1.0;
  b_kstr = obj->JointInternal->HomePositionInternal->size[0];
  obj->JointInternal->HomePositionInternal->size[0] = 1;
  invers_emxEnsureCapacity_real_T(obj->JointInternal->HomePositionInternal,
    b_kstr);
  obj->JointInternal->HomePositionInternal->data[0] = 0.0;
  obj->CollisionsInternal = inver_CollisionSet_CollisionSet(iobj_0, 0.0);
  return b_obj;
}

static t_robotics_manip_internal_Rig_T *i_RigidBody_RigidBody_gaqyy1no5
  (t_robotics_manip_internal_Rig_T *obj, l_robotics_manip_internal_Col_T *iobj_0,
   c_rigidBodyJoint_inverse_kine_T *iobj_1)
{
  emxArray_char_T_inverse_kinem_T *switch_expression;
  t_robotics_manip_internal_Rig_T *b_obj;
  real_T poslim_data[12];
  int32_T b_kstr;
  int32_T loop_ub;
  char_T b_0[9];
  char_T b[8];
  int8_T msubspace_data[36];
  int8_T tmp[6];
  boolean_T b_bool;
  static const char_T tmp_0[5] = { 'B', 'o', 'd', 'y', '5' };

  static const real_T tmp_1[9] = { 0.002508757765738561, -6.47781598373829E-5,
    7.8309648045720952E-6, -6.47781598373829E-5, 0.00019068014109471763,
    0.00023564436615893973, 7.8309648045720952E-6, 0.00023564436615893973,
    0.0024880552586374742 };

  static const real_T tmp_2[36] = { 0.002508757765738561, -6.47781598373829E-5,
    7.8309648045720952E-6, 0.0, 0.0035755966797, 0.0224274819866,
    -6.47781598373829E-5, 0.00019068014109471763, 0.00023564436615893973,
    -0.0035755966797, 0.0, -0.00074801093844999991, 7.8309648045720952E-6,
    0.00023564436615893973, 0.0024880552586374742, -0.0224274819866,
    0.00074801093844999991, 0.0, 0.0, -0.0035755966797, -0.0224274819866,
    0.300847, 0.0, 0.0, 0.0035755966797, 0.0, 0.00074801093844999991, 0.0,
    0.300847, 0.0, 0.0224274819866, -0.00074801093844999991, 0.0, 0.0, 0.0,
    0.300847 };

  static const int8_T tmp_3[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    1 };

  static const char_T tmp_4[6] = { 'J', 'o', 'i', 'n', 't', '5' };

  static const char_T tmp_5[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_6[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const real_T tmp_7[16] = { 1.0, 0.0, -0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 4.4408899995298218E-15, 0.155, -0.014125000000000002, 1.0 };

  static const real_T tmp_8[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  static const real_T tmp_9[36] = { 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  int32_T exitg1;
  b_obj = obj;
  b_kstr = obj->NameInternal->size[0] * obj->NameInternal->size[1];
  obj->NameInternal->size[0] = 1;
  obj->NameInternal->size[1] = 5;
  invers_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp_0[b_kstr];
  }

  obj->ParentIndex = 4.0;
  obj->MassInternal = 0.300847;
  obj->CenterOfMassInternal[0] = 0.00248635;
  obj->CenterOfMassInternal[1] = 0.0745478;
  obj->CenterOfMassInternal[2] = -0.011885100000000001;
  for (b_kstr = 0; b_kstr < 9; b_kstr++) {
    obj->InertiaInternal[b_kstr] = tmp_1[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 36; b_kstr++) {
    obj->SpatialInertia[b_kstr] = tmp_2[b_kstr];
  }

  iobj_1->InTree = false;
  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    iobj_1->JointToParentTransform[b_kstr] = tmp_3[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    iobj_1->ChildToJointTransform[b_kstr] = tmp_3[b_kstr];
  }

  b_kstr = iobj_1->NameInternal->size[0] * iobj_1->NameInternal->size[1];
  iobj_1->NameInternal->size[0] = 1;
  iobj_1->NameInternal->size[1] = 6;
  invers_emxEnsureCapacity_char_T(iobj_1->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    iobj_1->NameInternal->data[b_kstr] = tmp_4[b_kstr];
  }

  b_kstr = iobj_1->Type->size[0] * iobj_1->Type->size[1];
  iobj_1->Type->size[0] = 1;
  iobj_1->Type->size[1] = 8;
  invers_emxEnsureCapacity_char_T(iobj_1->Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    iobj_1->Type->data[b_kstr] = tmp_5[b_kstr];
  }

  inverse_kinemati_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = iobj_1->Type->size[1];
  invers_emxEnsureCapacity_char_T(switch_expression, b_kstr);
  loop_ub = iobj_1->Type->size[1] - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = iobj_1->Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp_5[b_kstr];
  }

  b_bool = false;
  if (switch_expression->size[1] != 8) {
  } else {
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
    if (switch_expression->size[1] != 9) {
    } else {
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

  inverse_kinemati_emxFree_char_T(&switch_expression);
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
    iobj_1->VelocityNumber = 1.0;
    iobj_1->PositionNumber = 1.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 1.0;
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
    iobj_1->VelocityNumber = 1.0;
    iobj_1->PositionNumber = 1.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = 0;
    }

    poslim_data[0] = 0.0;
    poslim_data[1] = 0.0;
    iobj_1->VelocityNumber = 0.0;
    iobj_1->PositionNumber = 0.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 0.0;
    break;
  }

  b_kstr = iobj_1->MotionSubspace->size[0] * iobj_1->MotionSubspace->size[1];
  iobj_1->MotionSubspace->size[0] = 6;
  iobj_1->MotionSubspace->size[1] = 1;
  invers_emxEnsureCapacity_real_T(iobj_1->MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    iobj_1->MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  b_kstr = iobj_1->PositionLimitsInternal->size[0] *
    iobj_1->PositionLimitsInternal->size[1];
  iobj_1->PositionLimitsInternal->size[0] = 1;
  iobj_1->PositionLimitsInternal->size[1] = 2;
  invers_emxEnsureCapacity_real_T(iobj_1->PositionLimitsInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 2; b_kstr++) {
    iobj_1->PositionLimitsInternal->data[b_kstr] = poslim_data[b_kstr];
  }

  b_kstr = iobj_1->HomePositionInternal->size[0];
  iobj_1->HomePositionInternal->size[0] = 1;
  invers_emxEnsureCapacity_real_T(iobj_1->HomePositionInternal, b_kstr);
  iobj_1->HomePositionInternal->data[0] = 0.0;
  obj->JointInternal = iobj_1;
  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal->JointToParentTransform[b_kstr] = tmp_7[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal->ChildToJointTransform[b_kstr] = tmp_8[b_kstr];
  }

  b_kstr = obj->JointInternal->MotionSubspace->size[0] * obj->
    JointInternal->MotionSubspace->size[1];
  obj->JointInternal->MotionSubspace->size[0] = 6;
  obj->JointInternal->MotionSubspace->size[1] = 1;
  invers_emxEnsureCapacity_real_T(obj->JointInternal->MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal->MotionSubspace->data[b_kstr] = tmp_9[b_kstr];
  }

  obj->JointInternal->InTree = true;
  b_kstr = obj->JointInternal->PositionLimitsInternal->size[0] *
    obj->JointInternal->PositionLimitsInternal->size[1];
  obj->JointInternal->PositionLimitsInternal->size[0] = 1;
  obj->JointInternal->PositionLimitsInternal->size[1] = 2;
  invers_emxEnsureCapacity_real_T(obj->JointInternal->PositionLimitsInternal,
    b_kstr);
  obj->JointInternal->PositionLimitsInternal->data[0] = 0.0;
  obj->JointInternal->PositionLimitsInternal->data[obj->
    JointInternal->PositionLimitsInternal->size[0]] = 1.5708;
  obj->JointInternal->JointAxisInternal[0] = 0.0;
  obj->JointInternal->JointAxisInternal[1] = 0.0;
  obj->JointInternal->JointAxisInternal[2] = 1.0;
  b_kstr = obj->JointInternal->HomePositionInternal->size[0];
  obj->JointInternal->HomePositionInternal->size[0] = 1;
  invers_emxEnsureCapacity_real_T(obj->JointInternal->HomePositionInternal,
    b_kstr);
  obj->JointInternal->HomePositionInternal->data[0] = 0.0;
  obj->CollisionsInternal = inver_CollisionSet_CollisionSet(iobj_0, 0.0);
  return b_obj;
}

static t_robotics_manip_internal_Rig_T *RigidBody_RigidBody_gaqyy1no5h
  (t_robotics_manip_internal_Rig_T *obj, l_robotics_manip_internal_Col_T *iobj_0,
   c_rigidBodyJoint_inverse_kine_T *iobj_1)
{
  emxArray_char_T_inverse_kinem_T *switch_expression;
  t_robotics_manip_internal_Rig_T *b_obj;
  real_T poslim_data[12];
  int32_T b_kstr;
  int32_T loop_ub;
  char_T b_0[9];
  char_T b[8];
  int8_T msubspace_data[36];
  int8_T tmp[6];
  boolean_T b_bool;
  static const char_T tmp_0[5] = { 'B', 'o', 'd', 'y', '6' };

  static const real_T tmp_1[9] = { 8.1E-10, 0.0, -6.075E-10, 0.0, 2.025E-9, 0.0,
    -6.075E-10, 0.0, 2.025E-9 };

  static const real_T tmp_2[36] = { 8.1E-10, 0.0, -6.075E-10, 0.0,
    2.0250000000000002E-7, -0.0, 0.0, 2.025E-9, 0.0, -2.0250000000000002E-7, 0.0,
    4.0500000000000004E-7, -6.075E-10, 0.0, 2.025E-9, 0.0,
    -4.0500000000000004E-7, 0.0, 0.0, -2.0250000000000002E-7, 0.0, 0.000135, 0.0,
    0.0, 2.0250000000000002E-7, 0.0, -4.0500000000000004E-7, 0.0, 0.000135, 0.0,
    -0.0, 4.0500000000000004E-7, 0.0, 0.0, 0.0, 0.000135 };

  static const int8_T tmp_3[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    1 };

  static const char_T tmp_4[6] = { 'J', 'o', 'i', 'n', 't', '6' };

  static const char_T tmp_5[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_6[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_7[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const real_T tmp_8[16] = { 1.0, 0.0, -0.0, 0.0, 0.0,
    -3.6732051040127077E-6, 0.99999999999325373, 0.0, 0.0, -0.99999999999325373,
    -3.6732051040127077E-6, 0.0, 0.056000400000000006, 0.079999999999999988,
    -0.010500000000000002, 1.0 };

  static const real_T tmp_9[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  int32_T exitg1;
  b_obj = obj;
  b_kstr = obj->NameInternal->size[0] * obj->NameInternal->size[1];
  obj->NameInternal->size[0] = 1;
  obj->NameInternal->size[1] = 5;
  invers_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp_0[b_kstr];
  }

  obj->ParentIndex = 5.0;
  obj->MassInternal = 0.000135;
  obj->CenterOfMassInternal[0] = -0.003;
  obj->CenterOfMassInternal[1] = -0.0;
  obj->CenterOfMassInternal[2] = -0.0015;
  for (b_kstr = 0; b_kstr < 9; b_kstr++) {
    obj->InertiaInternal[b_kstr] = tmp_1[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 36; b_kstr++) {
    obj->SpatialInertia[b_kstr] = tmp_2[b_kstr];
  }

  iobj_1->InTree = false;
  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    iobj_1->JointToParentTransform[b_kstr] = tmp_3[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    iobj_1->ChildToJointTransform[b_kstr] = tmp_3[b_kstr];
  }

  b_kstr = iobj_1->NameInternal->size[0] * iobj_1->NameInternal->size[1];
  iobj_1->NameInternal->size[0] = 1;
  iobj_1->NameInternal->size[1] = 6;
  invers_emxEnsureCapacity_char_T(iobj_1->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    iobj_1->NameInternal->data[b_kstr] = tmp_4[b_kstr];
  }

  b_kstr = iobj_1->Type->size[0] * iobj_1->Type->size[1];
  iobj_1->Type->size[0] = 1;
  iobj_1->Type->size[1] = 5;
  invers_emxEnsureCapacity_char_T(iobj_1->Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    iobj_1->Type->data[b_kstr] = tmp_5[b_kstr];
  }

  inverse_kinemati_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = iobj_1->Type->size[1];
  invers_emxEnsureCapacity_char_T(switch_expression, b_kstr);
  loop_ub = iobj_1->Type->size[1] - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = iobj_1->Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp_6[b_kstr];
  }

  b_bool = false;
  if (switch_expression->size[1] != 8) {
  } else {
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
    if (switch_expression->size[1] != 9) {
    } else {
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

  inverse_kinemati_emxFree_char_T(&switch_expression);
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
    iobj_1->VelocityNumber = 1.0;
    iobj_1->PositionNumber = 1.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 1.0;
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
    iobj_1->VelocityNumber = 1.0;
    iobj_1->PositionNumber = 1.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = 0;
    }

    poslim_data[0] = 0.0;
    poslim_data[1] = 0.0;
    iobj_1->VelocityNumber = 0.0;
    iobj_1->PositionNumber = 0.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 0.0;
    break;
  }

  b_kstr = iobj_1->MotionSubspace->size[0] * iobj_1->MotionSubspace->size[1];
  iobj_1->MotionSubspace->size[0] = 6;
  iobj_1->MotionSubspace->size[1] = 1;
  invers_emxEnsureCapacity_real_T(iobj_1->MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    iobj_1->MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  b_kstr = iobj_1->PositionLimitsInternal->size[0] *
    iobj_1->PositionLimitsInternal->size[1];
  iobj_1->PositionLimitsInternal->size[0] = 1;
  iobj_1->PositionLimitsInternal->size[1] = 2;
  invers_emxEnsureCapacity_real_T(iobj_1->PositionLimitsInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 2; b_kstr++) {
    iobj_1->PositionLimitsInternal->data[b_kstr] = poslim_data[b_kstr];
  }

  b_kstr = iobj_1->HomePositionInternal->size[0];
  iobj_1->HomePositionInternal->size[0] = 1;
  invers_emxEnsureCapacity_real_T(iobj_1->HomePositionInternal, b_kstr);
  iobj_1->HomePositionInternal->data[0] = 0.0;
  obj->JointInternal = iobj_1;
  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal->JointToParentTransform[b_kstr] = tmp_8[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal->ChildToJointTransform[b_kstr] = tmp_9[b_kstr];
  }

  b_kstr = obj->JointInternal->MotionSubspace->size[0] * obj->
    JointInternal->MotionSubspace->size[1];
  obj->JointInternal->MotionSubspace->size[0] = 6;
  obj->JointInternal->MotionSubspace->size[1] = 1;
  invers_emxEnsureCapacity_real_T(obj->JointInternal->MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal->MotionSubspace->data[b_kstr] = 0.0;
  }

  obj->JointInternal->InTree = true;
  b_kstr = obj->JointInternal->PositionLimitsInternal->size[0] *
    obj->JointInternal->PositionLimitsInternal->size[1];
  obj->JointInternal->PositionLimitsInternal->size[0] = 1;
  obj->JointInternal->PositionLimitsInternal->size[1] = 2;
  invers_emxEnsureCapacity_real_T(obj->JointInternal->PositionLimitsInternal,
    b_kstr);
  obj->JointInternal->PositionLimitsInternal->data[0] = 0.0;
  obj->JointInternal->PositionLimitsInternal->data[obj->
    JointInternal->PositionLimitsInternal->size[0]] = 0.0;
  obj->JointInternal->JointAxisInternal[0] = 0.0;
  obj->JointInternal->JointAxisInternal[1] = 0.0;
  obj->JointInternal->JointAxisInternal[2] = 0.0;
  b_kstr = obj->JointInternal->HomePositionInternal->size[0];
  obj->JointInternal->HomePositionInternal->size[0] = 1;
  invers_emxEnsureCapacity_real_T(obj->JointInternal->HomePositionInternal,
    b_kstr);
  obj->JointInternal->HomePositionInternal->data[0] = 0.0;
  obj->CollisionsInternal = inver_CollisionSet_CollisionSet(iobj_0, 0.0);
  return b_obj;
}

static t_robotics_manip_internal_Rig_T *RigidBody_RigidBody_gaqyy1no5hz
  (t_robotics_manip_internal_Rig_T *obj, l_robotics_manip_internal_Col_T *iobj_0,
   c_rigidBodyJoint_inverse_kine_T *iobj_1)
{
  emxArray_char_T_inverse_kinem_T *switch_expression;
  t_robotics_manip_internal_Rig_T *b_obj;
  real_T poslim_data[12];
  int32_T b_kstr;
  int32_T loop_ub;
  char_T b_0[9];
  char_T b[8];
  int8_T msubspace_data[36];
  int8_T tmp[6];
  boolean_T b_bool;
  static const char_T tmp_0[5] = { 'B', 'o', 'd', 'y', '7' };

  static const real_T tmp_1[9] = { 1.265625E-10, 0.0, 0.0, 0.0, 1.265625E-10,
    0.0, 0.0, 0.0, 5.0625E-11 };

  static const real_T tmp_2[36] = { 1.265625E-10, 0.0, 0.0, 0.0,
    5.0625000000000005E-8, -0.0, 0.0, 1.265625E-10, 0.0, -5.0625000000000005E-8,
    0.0, 0.0, 0.0, 0.0, 5.0625E-11, 0.0, -0.0, 0.0, 0.0, -5.0625000000000005E-8,
    0.0, 3.375E-5, 0.0, 0.0, 5.0625000000000005E-8, 0.0, -0.0, 0.0, 3.375E-5,
    0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 3.375E-5 };

  static const int8_T tmp_3[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    1 };

  static const char_T tmp_4[6] = { 'J', 'o', 'i', 'n', 't', '7' };

  static const char_T tmp_5[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_6[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_7[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const real_T tmp_8[16] = { -3.6730077737609104E-6,
    4.4981372133796444E-22, -0.99999999999325451, 0.0, 0.99999999999325451,
    -1.8369701987127688E-16, -3.6730077737609104E-6, 0.0,
    -1.8369701987168995E-16, -1.0, 2.2490686066898222E-22, 0.0,
    0.00050041600000000049, 0.23623999999999998, -0.0105, 1.0 };

  static const real_T tmp_9[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  int32_T exitg1;
  b_obj = obj;
  b_kstr = obj->NameInternal->size[0] * obj->NameInternal->size[1];
  obj->NameInternal->size[0] = 1;
  obj->NameInternal->size[1] = 5;
  invers_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp_0[b_kstr];
  }

  obj->ParentIndex = 5.0;
  obj->MassInternal = 3.375E-5;
  obj->CenterOfMassInternal[0] = -0.0;
  obj->CenterOfMassInternal[1] = -0.0;
  obj->CenterOfMassInternal[2] = -0.0015;
  for (b_kstr = 0; b_kstr < 9; b_kstr++) {
    obj->InertiaInternal[b_kstr] = tmp_1[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 36; b_kstr++) {
    obj->SpatialInertia[b_kstr] = tmp_2[b_kstr];
  }

  iobj_1->InTree = false;
  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    iobj_1->JointToParentTransform[b_kstr] = tmp_3[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    iobj_1->ChildToJointTransform[b_kstr] = tmp_3[b_kstr];
  }

  b_kstr = iobj_1->NameInternal->size[0] * iobj_1->NameInternal->size[1];
  iobj_1->NameInternal->size[0] = 1;
  iobj_1->NameInternal->size[1] = 6;
  invers_emxEnsureCapacity_char_T(iobj_1->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    iobj_1->NameInternal->data[b_kstr] = tmp_4[b_kstr];
  }

  b_kstr = iobj_1->Type->size[0] * iobj_1->Type->size[1];
  iobj_1->Type->size[0] = 1;
  iobj_1->Type->size[1] = 5;
  invers_emxEnsureCapacity_char_T(iobj_1->Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    iobj_1->Type->data[b_kstr] = tmp_5[b_kstr];
  }

  inverse_kinemati_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = iobj_1->Type->size[1];
  invers_emxEnsureCapacity_char_T(switch_expression, b_kstr);
  loop_ub = iobj_1->Type->size[1] - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = iobj_1->Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp_6[b_kstr];
  }

  b_bool = false;
  if (switch_expression->size[1] != 8) {
  } else {
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
    if (switch_expression->size[1] != 9) {
    } else {
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

  inverse_kinemati_emxFree_char_T(&switch_expression);
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
    iobj_1->VelocityNumber = 1.0;
    iobj_1->PositionNumber = 1.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 1.0;
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
    iobj_1->VelocityNumber = 1.0;
    iobj_1->PositionNumber = 1.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = 0;
    }

    poslim_data[0] = 0.0;
    poslim_data[1] = 0.0;
    iobj_1->VelocityNumber = 0.0;
    iobj_1->PositionNumber = 0.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 0.0;
    break;
  }

  b_kstr = iobj_1->MotionSubspace->size[0] * iobj_1->MotionSubspace->size[1];
  iobj_1->MotionSubspace->size[0] = 6;
  iobj_1->MotionSubspace->size[1] = 1;
  invers_emxEnsureCapacity_real_T(iobj_1->MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    iobj_1->MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  b_kstr = iobj_1->PositionLimitsInternal->size[0] *
    iobj_1->PositionLimitsInternal->size[1];
  iobj_1->PositionLimitsInternal->size[0] = 1;
  iobj_1->PositionLimitsInternal->size[1] = 2;
  invers_emxEnsureCapacity_real_T(iobj_1->PositionLimitsInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 2; b_kstr++) {
    iobj_1->PositionLimitsInternal->data[b_kstr] = poslim_data[b_kstr];
  }

  b_kstr = iobj_1->HomePositionInternal->size[0];
  iobj_1->HomePositionInternal->size[0] = 1;
  invers_emxEnsureCapacity_real_T(iobj_1->HomePositionInternal, b_kstr);
  iobj_1->HomePositionInternal->data[0] = 0.0;
  obj->JointInternal = iobj_1;
  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal->JointToParentTransform[b_kstr] = tmp_8[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal->ChildToJointTransform[b_kstr] = tmp_9[b_kstr];
  }

  b_kstr = obj->JointInternal->MotionSubspace->size[0] * obj->
    JointInternal->MotionSubspace->size[1];
  obj->JointInternal->MotionSubspace->size[0] = 6;
  obj->JointInternal->MotionSubspace->size[1] = 1;
  invers_emxEnsureCapacity_real_T(obj->JointInternal->MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal->MotionSubspace->data[b_kstr] = 0.0;
  }

  obj->JointInternal->InTree = true;
  b_kstr = obj->JointInternal->PositionLimitsInternal->size[0] *
    obj->JointInternal->PositionLimitsInternal->size[1];
  obj->JointInternal->PositionLimitsInternal->size[0] = 1;
  obj->JointInternal->PositionLimitsInternal->size[1] = 2;
  invers_emxEnsureCapacity_real_T(obj->JointInternal->PositionLimitsInternal,
    b_kstr);
  obj->JointInternal->PositionLimitsInternal->data[0] = 0.0;
  obj->JointInternal->PositionLimitsInternal->data[obj->
    JointInternal->PositionLimitsInternal->size[0]] = 0.0;
  obj->JointInternal->JointAxisInternal[0] = 0.0;
  obj->JointInternal->JointAxisInternal[1] = 0.0;
  obj->JointInternal->JointAxisInternal[2] = 0.0;
  b_kstr = obj->JointInternal->HomePositionInternal->size[0];
  obj->JointInternal->HomePositionInternal->size[0] = 1;
  invers_emxEnsureCapacity_real_T(obj->JointInternal->HomePositionInternal,
    b_kstr);
  obj->JointInternal->HomePositionInternal->data[0] = 0.0;
  obj->CollisionsInternal = inver_CollisionSet_CollisionSet(iobj_0, 0.0);
  return b_obj;
}

static c_rigidBodyJoint_inverse_kine_T *i_rigidBodyJoint_rigidBodyJoint
  (c_rigidBodyJoint_inverse_kine_T *obj, const emxArray_char_T_inverse_kinem_T
   *jname)
{
  c_rigidBodyJoint_inverse_kine_T *b_obj;
  emxArray_char_T_inverse_kinem_T *switch_expression;
  static const int8_T tmp[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1
  };

  static const char_T tmp_0[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_1[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_2[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  int32_T exitg1;
  obj->InTree = false;
  for (inverse_kinematics_B.b_kstr_p = 0; inverse_kinematics_B.b_kstr_p < 16;
       inverse_kinematics_B.b_kstr_p++) {
    obj->JointToParentTransform[inverse_kinematics_B.b_kstr_p] =
      tmp[inverse_kinematics_B.b_kstr_p];
  }

  for (inverse_kinematics_B.b_kstr_p = 0; inverse_kinematics_B.b_kstr_p < 16;
       inverse_kinematics_B.b_kstr_p++) {
    obj->ChildToJointTransform[inverse_kinematics_B.b_kstr_p] =
      tmp[inverse_kinematics_B.b_kstr_p];
  }

  b_obj = obj;
  inverse_kinematics_B.i2 = obj->NameInternal->size[0] * obj->NameInternal->
    size[1];
  obj->NameInternal->size[0] = 1;
  obj->NameInternal->size[1] = jname->size[1];
  invers_emxEnsureCapacity_char_T(obj->NameInternal, inverse_kinematics_B.i2);
  inverse_kinematics_B.loop_ub_p = jname->size[1] - 1;
  for (inverse_kinematics_B.b_kstr_p = 0; inverse_kinematics_B.b_kstr_p <=
       inverse_kinematics_B.loop_ub_p; inverse_kinematics_B.b_kstr_p++) {
    inverse_kinematics_B.i2 = inverse_kinematics_B.b_kstr_p;
    obj->NameInternal->data[inverse_kinematics_B.i2] = jname->
      data[inverse_kinematics_B.i2];
  }

  inverse_kinematics_B.i2 = obj->Type->size[0] * obj->Type->size[1];
  obj->Type->size[0] = 1;
  obj->Type->size[1] = 5;
  invers_emxEnsureCapacity_char_T(obj->Type, inverse_kinematics_B.i2);
  for (inverse_kinematics_B.b_kstr_p = 0; inverse_kinematics_B.b_kstr_p < 5;
       inverse_kinematics_B.b_kstr_p++) {
    obj->Type->data[inverse_kinematics_B.b_kstr_p] =
      tmp_0[inverse_kinematics_B.b_kstr_p];
  }

  inverse_kinemati_emxInit_char_T(&switch_expression, 2);
  inverse_kinematics_B.i2 = switch_expression->size[0] * switch_expression->
    size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->Type->size[1];
  invers_emxEnsureCapacity_char_T(switch_expression, inverse_kinematics_B.i2);
  inverse_kinematics_B.loop_ub_p = obj->Type->size[1] - 1;
  for (inverse_kinematics_B.b_kstr_p = 0; inverse_kinematics_B.b_kstr_p <=
       inverse_kinematics_B.loop_ub_p; inverse_kinematics_B.b_kstr_p++) {
    inverse_kinematics_B.i2 = inverse_kinematics_B.b_kstr_p;
    switch_expression->data[inverse_kinematics_B.i2] = obj->Type->
      data[inverse_kinematics_B.i2];
  }

  for (inverse_kinematics_B.b_kstr_p = 0; inverse_kinematics_B.b_kstr_p < 8;
       inverse_kinematics_B.b_kstr_p++) {
    inverse_kinematics_B.b_jz[inverse_kinematics_B.b_kstr_p] =
      tmp_1[inverse_kinematics_B.b_kstr_p];
  }

  inverse_kinematics_B.b_bool_p = false;
  if (switch_expression->size[1] != 8) {
  } else {
    inverse_kinematics_B.b_kstr_p = 1;
    do {
      exitg1 = 0;
      if (inverse_kinematics_B.b_kstr_p - 1 < 8) {
        if (switch_expression->data[inverse_kinematics_B.b_kstr_p - 1] !=
            inverse_kinematics_B.b_jz[inverse_kinematics_B.b_kstr_p - 1]) {
          exitg1 = 1;
        } else {
          inverse_kinematics_B.b_kstr_p++;
        }
      } else {
        inverse_kinematics_B.b_bool_p = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (inverse_kinematics_B.b_bool_p) {
    inverse_kinematics_B.b_kstr_p = 0;
  } else {
    for (inverse_kinematics_B.b_kstr_p = 0; inverse_kinematics_B.b_kstr_p < 9;
         inverse_kinematics_B.b_kstr_p++) {
      inverse_kinematics_B.b_d[inverse_kinematics_B.b_kstr_p] =
        tmp_2[inverse_kinematics_B.b_kstr_p];
    }

    inverse_kinematics_B.b_bool_p = false;
    if (switch_expression->size[1] != 9) {
    } else {
      inverse_kinematics_B.b_kstr_p = 1;
      do {
        exitg1 = 0;
        if (inverse_kinematics_B.b_kstr_p - 1 < 9) {
          if (switch_expression->data[inverse_kinematics_B.b_kstr_p - 1] !=
              inverse_kinematics_B.b_d[inverse_kinematics_B.b_kstr_p - 1]) {
            exitg1 = 1;
          } else {
            inverse_kinematics_B.b_kstr_p++;
          }
        } else {
          inverse_kinematics_B.b_bool_p = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (inverse_kinematics_B.b_bool_p) {
      inverse_kinematics_B.b_kstr_p = 1;
    } else {
      inverse_kinematics_B.b_kstr_p = -1;
    }
  }

  inverse_kinemati_emxFree_char_T(&switch_expression);
  switch (inverse_kinematics_B.b_kstr_p) {
   case 0:
    inverse_kinematics_B.iv2[0] = 0;
    inverse_kinematics_B.iv2[1] = 0;
    inverse_kinematics_B.iv2[2] = 1;
    inverse_kinematics_B.iv2[3] = 0;
    inverse_kinematics_B.iv2[4] = 0;
    inverse_kinematics_B.iv2[5] = 0;
    for (inverse_kinematics_B.b_kstr_p = 0; inverse_kinematics_B.b_kstr_p < 6;
         inverse_kinematics_B.b_kstr_p++) {
      inverse_kinematics_B.msubspace_data_l[inverse_kinematics_B.b_kstr_p] =
        inverse_kinematics_B.iv2[inverse_kinematics_B.b_kstr_p];
    }

    inverse_kinematics_B.poslim_data_c[0] = -3.1415926535897931;
    inverse_kinematics_B.poslim_data_c[1] = 3.1415926535897931;
    obj->VelocityNumber = 1.0;
    obj->PositionNumber = 1.0;
    obj->JointAxisInternal[0] = 0.0;
    obj->JointAxisInternal[1] = 0.0;
    obj->JointAxisInternal[2] = 1.0;
    break;

   case 1:
    inverse_kinematics_B.iv2[0] = 0;
    inverse_kinematics_B.iv2[1] = 0;
    inverse_kinematics_B.iv2[2] = 0;
    inverse_kinematics_B.iv2[3] = 0;
    inverse_kinematics_B.iv2[4] = 0;
    inverse_kinematics_B.iv2[5] = 1;
    for (inverse_kinematics_B.b_kstr_p = 0; inverse_kinematics_B.b_kstr_p < 6;
         inverse_kinematics_B.b_kstr_p++) {
      inverse_kinematics_B.msubspace_data_l[inverse_kinematics_B.b_kstr_p] =
        inverse_kinematics_B.iv2[inverse_kinematics_B.b_kstr_p];
    }

    inverse_kinematics_B.poslim_data_c[0] = -0.5;
    inverse_kinematics_B.poslim_data_c[1] = 0.5;
    obj->VelocityNumber = 1.0;
    obj->PositionNumber = 1.0;
    obj->JointAxisInternal[0] = 0.0;
    obj->JointAxisInternal[1] = 0.0;
    obj->JointAxisInternal[2] = 1.0;
    break;

   default:
    for (inverse_kinematics_B.b_kstr_p = 0; inverse_kinematics_B.b_kstr_p < 6;
         inverse_kinematics_B.b_kstr_p++) {
      inverse_kinematics_B.msubspace_data_l[inverse_kinematics_B.b_kstr_p] = 0;
    }

    inverse_kinematics_B.poslim_data_c[0] = 0.0;
    inverse_kinematics_B.poslim_data_c[1] = 0.0;
    obj->VelocityNumber = 0.0;
    obj->PositionNumber = 0.0;
    obj->JointAxisInternal[0] = 0.0;
    obj->JointAxisInternal[1] = 0.0;
    obj->JointAxisInternal[2] = 0.0;
    break;
  }

  inverse_kinematics_B.i2 = obj->MotionSubspace->size[0] * obj->
    MotionSubspace->size[1];
  obj->MotionSubspace->size[0] = 6;
  obj->MotionSubspace->size[1] = 1;
  invers_emxEnsureCapacity_real_T(obj->MotionSubspace, inverse_kinematics_B.i2);
  for (inverse_kinematics_B.b_kstr_p = 0; inverse_kinematics_B.b_kstr_p < 6;
       inverse_kinematics_B.b_kstr_p++) {
    obj->MotionSubspace->data[inverse_kinematics_B.b_kstr_p] =
      inverse_kinematics_B.msubspace_data_l[inverse_kinematics_B.b_kstr_p];
  }

  inverse_kinematics_B.i2 = obj->PositionLimitsInternal->size[0] *
    obj->PositionLimitsInternal->size[1];
  obj->PositionLimitsInternal->size[0] = 1;
  obj->PositionLimitsInternal->size[1] = 2;
  invers_emxEnsureCapacity_real_T(obj->PositionLimitsInternal,
    inverse_kinematics_B.i2);
  for (inverse_kinematics_B.b_kstr_p = 0; inverse_kinematics_B.b_kstr_p < 2;
       inverse_kinematics_B.b_kstr_p++) {
    obj->PositionLimitsInternal->data[inverse_kinematics_B.b_kstr_p] =
      inverse_kinematics_B.poslim_data_c[inverse_kinematics_B.b_kstr_p];
  }

  inverse_kinematics_B.i2 = obj->HomePositionInternal->size[0];
  obj->HomePositionInternal->size[0] = 1;
  invers_emxEnsureCapacity_real_T(obj->HomePositionInternal,
    inverse_kinematics_B.i2);
  obj->HomePositionInternal->data[0] = 0.0;
  return b_obj;
}

static u_robotics_manip_internal_Rig_T *inv_RigidBodyTree_RigidBodyTree
  (u_robotics_manip_internal_Rig_T *obj)
{
  c_rigidBodyJoint_inverse_kine_T *iobj_1;
  emxArray_char_T_inverse_kinem_T *jname;
  l_robotics_manip_internal_Col_T *iobj_0;
  t_robotics_manip_internal_Rig_T *iobj_2;
  u_robotics_manip_internal_Rig_T *b_obj;
  static const char_T tmp[5] = { 'B', 'o', 'd', 'y', '1' };

  static const real_T tmp_0[9] = { 0.00544722127026208, 1.7565828776431849E-8,
    0.0035546154800648435, 1.7565828776431849E-8, 0.011752147632167451,
    2.7954603751052958E-7, 0.0035546154800648435, 2.7954603751052958E-7,
    0.010105286379255111 };

  static const real_T tmp_1[36] = { 0.00544722127026208, 1.7565828776431849E-8,
    0.0035546154800648435, 0.0, -0.07105704914, -3.9101186964999986E-6,
    1.7565828776431849E-8, 0.011752147632167451, 2.7954603751052958E-7,
    0.07105704914, 0.0, 0.08887083627, 0.0035546154800648435,
    2.7954603751052958E-7, 0.010105286379255111, 3.9101186964999986E-6,
    -0.08887083627, 0.0, 0.0, 0.07105704914, 3.9101186964999986E-6, 1.76245, 0.0,
    0.0, -0.07105704914, 0.0, -0.08887083627, 0.0, 1.76245, 0.0,
    -3.9101186964999986E-6, 0.08887083627, 0.0, 0.0, 0.0, 1.76245 };

  static const int8_T tmp_2[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    1 };

  static const char_T tmp_3[6] = { 'J', 'o', 'i', 'n', 't', '1' };

  static const char_T tmp_4[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_5[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_6[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const real_T tmp_7[16] = { 1.0, 0.0, -0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  static const real_T tmp_8[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  static const char_T tmp_9[5] = { 'B', 'o', 'd', 'y', '2' };

  static const real_T tmp_a[9] = { 0.0047129683656382641, 2.3217908522253995E-5,
    5.2307023667153062E-5, 2.3217908522253995E-5, 0.0044177468163980006,
    -0.000836298840095305, 5.2307023667153062E-5, -0.000836298840095305,
    0.0009022836543377406 };

  static const real_T tmp_b[36] = { 0.0047129683656382641, 2.3217908522253995E-5,
    5.2307023667153062E-5, 0.0, 0.0362568255036, -0.0074843294297999973,
    2.3217908522253995E-5, 0.0044177468163980006, -0.000836298840095305,
    -0.0362568255036, 0.0, -0.00042583748634299976, 5.2307023667153062E-5,
    -0.000836298840095305, 0.0009022836543377406, 0.0074843294297999973,
    0.00042583748634299976, 0.0, 0.0, -0.0362568255036, 0.0074843294297999973,
    0.680307, 0.0, 0.0, 0.0362568255036, 0.0, 0.00042583748634299976, 0.0,
    0.680307, 0.0, -0.0074843294297999973, -0.00042583748634299976, 0.0, 0.0,
    0.0, 0.680307 };

  static const char_T tmp_c[6] = { 'J', 'o', 'i', 'n', 't', '2' };

  static const real_T tmp_d[16] = { -3.6732051042347524E-6, -0.99999999999325373,
    -0.0, 0.0, -0.999999999989733, 3.67320510422182E-6, -2.65358979335273E-6,
    0.0, 2.6535897933348279E-6, -9.7471795734884888E-12, -0.99999999999647926,
    0.0, -0.06, 4.3385027558365263E-18, 0.069, 1.0 };

  static const real_T tmp_e[36] = { 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  int32_T exitg1;
  b_obj = obj;
  obj->NumBodies = 7.0;
  iobj_0 = &obj->_pobj0[0];
  iobj_1 = &obj->_pobj1[0];
  iobj_2 = &obj->_pobj2[0];
  obj->Bodies[0] = inverse_kin_RigidBody_RigidBody(&(&obj->_pobj2[0])[0],
    &iobj_0[0], &iobj_1[0]);
  obj->Bodies[1] = inverse_k_RigidBody_RigidBody_g(&iobj_2[1], &iobj_0[1],
    &iobj_1[1]);
  obj->Bodies[2] = inverse__RigidBody_RigidBody_ga(&iobj_2[2], &iobj_0[2],
    &iobj_1[2]);
  obj->Bodies[3] = inverse_RigidBody_RigidBody_gaq(&iobj_2[3], &iobj_0[3],
    &iobj_1[3]);
  obj->Bodies[4] = invers_RigidBody_RigidBody_gaqy(&iobj_2[4], &iobj_0[4],
    &iobj_1[4]);
  obj->Bodies[5] = inver_RigidBody_RigidBody_gaqyy(&iobj_2[5], &iobj_0[5],
    &iobj_1[5]);
  obj->Bodies[6] = inve_RigidBody_RigidBody_gaqyy1(&iobj_2[6], &iobj_0[6],
    &iobj_1[6]);
  iobj_2 = &obj->_pobj2[7];
  iobj_0 = &obj->_pobj0[7];
  inverse_kinematics_B.i5 = iobj_2->NameInternal->size[0] * iobj_2->
    NameInternal->size[1];
  iobj_2->NameInternal->size[0] = 1;
  iobj_2->NameInternal->size[1] = 5;
  invers_emxEnsureCapacity_char_T(iobj_2->NameInternal, inverse_kinematics_B.i5);
  for (inverse_kinematics_B.b_kstr_c = 0; inverse_kinematics_B.b_kstr_c < 5;
       inverse_kinematics_B.b_kstr_c++) {
    iobj_2->NameInternal->data[inverse_kinematics_B.b_kstr_c] =
      tmp[inverse_kinematics_B.b_kstr_c];
  }

  iobj_2->ParentIndex = 0.0;
  iobj_2->MassInternal = 1.76245;
  iobj_2->CenterOfMassInternal[0] = -0.0504246;
  iobj_2->CenterOfMassInternal[1] = -2.218569999999999E-6;
  iobj_2->CenterOfMassInternal[2] = 0.0403172;
  for (inverse_kinematics_B.b_kstr_c = 0; inverse_kinematics_B.b_kstr_c < 9;
       inverse_kinematics_B.b_kstr_c++) {
    iobj_2->InertiaInternal[inverse_kinematics_B.b_kstr_c] =
      tmp_0[inverse_kinematics_B.b_kstr_c];
  }

  for (inverse_kinematics_B.b_kstr_c = 0; inverse_kinematics_B.b_kstr_c < 36;
       inverse_kinematics_B.b_kstr_c++) {
    iobj_2->SpatialInertia[inverse_kinematics_B.b_kstr_c] =
      tmp_1[inverse_kinematics_B.b_kstr_c];
  }

  obj->_pobj1[7].InTree = false;
  for (inverse_kinematics_B.b_kstr_c = 0; inverse_kinematics_B.b_kstr_c < 16;
       inverse_kinematics_B.b_kstr_c++) {
    obj->_pobj1[7].JointToParentTransform[inverse_kinematics_B.b_kstr_c] =
      tmp_2[inverse_kinematics_B.b_kstr_c];
  }

  for (inverse_kinematics_B.b_kstr_c = 0; inverse_kinematics_B.b_kstr_c < 16;
       inverse_kinematics_B.b_kstr_c++) {
    obj->_pobj1[7].ChildToJointTransform[inverse_kinematics_B.b_kstr_c] =
      tmp_2[inverse_kinematics_B.b_kstr_c];
  }

  inverse_kinematics_B.i5 = obj->_pobj1[7].NameInternal->size[0] * obj->_pobj1[7]
    .NameInternal->size[1];
  obj->_pobj1[7].NameInternal->size[0] = 1;
  obj->_pobj1[7].NameInternal->size[1] = 6;
  invers_emxEnsureCapacity_char_T(obj->_pobj1[7].NameInternal,
    inverse_kinematics_B.i5);
  for (inverse_kinematics_B.b_kstr_c = 0; inverse_kinematics_B.b_kstr_c < 6;
       inverse_kinematics_B.b_kstr_c++) {
    obj->_pobj1[7].NameInternal->data[inverse_kinematics_B.b_kstr_c] =
      tmp_3[inverse_kinematics_B.b_kstr_c];
  }

  inverse_kinematics_B.i5 = obj->_pobj1[7].Type->size[0] * obj->_pobj1[7]
    .Type->size[1];
  obj->_pobj1[7].Type->size[0] = 1;
  obj->_pobj1[7].Type->size[1] = 5;
  invers_emxEnsureCapacity_char_T(obj->_pobj1[7].Type, inverse_kinematics_B.i5);
  for (inverse_kinematics_B.b_kstr_c = 0; inverse_kinematics_B.b_kstr_c < 5;
       inverse_kinematics_B.b_kstr_c++) {
    obj->_pobj1[7].Type->data[inverse_kinematics_B.b_kstr_c] =
      tmp_4[inverse_kinematics_B.b_kstr_c];
  }

  inverse_kinemati_emxInit_char_T(&jname, 2);
  inverse_kinematics_B.i5 = jname->size[0] * jname->size[1];
  jname->size[0] = 1;
  jname->size[1] = obj->_pobj1[7].Type->size[1];
  invers_emxEnsureCapacity_char_T(jname, inverse_kinematics_B.i5);
  inverse_kinematics_B.loop_ub_a = obj->_pobj1[7].Type->size[1] - 1;
  for (inverse_kinematics_B.b_kstr_c = 0; inverse_kinematics_B.b_kstr_c <=
       inverse_kinematics_B.loop_ub_a; inverse_kinematics_B.b_kstr_c++) {
    inverse_kinematics_B.i5 = inverse_kinematics_B.b_kstr_c;
    jname->data[inverse_kinematics_B.i5] = obj->_pobj1[7].Type->
      data[inverse_kinematics_B.i5];
  }

  for (inverse_kinematics_B.b_kstr_c = 0; inverse_kinematics_B.b_kstr_c < 8;
       inverse_kinematics_B.b_kstr_c++) {
    inverse_kinematics_B.b_o[inverse_kinematics_B.b_kstr_c] =
      tmp_5[inverse_kinematics_B.b_kstr_c];
  }

  inverse_kinematics_B.b_bool_n = false;
  if (jname->size[1] != 8) {
  } else {
    inverse_kinematics_B.b_kstr_c = 1;
    do {
      exitg1 = 0;
      if (inverse_kinematics_B.b_kstr_c - 1 < 8) {
        if (jname->data[inverse_kinematics_B.b_kstr_c - 1] !=
            inverse_kinematics_B.b_o[inverse_kinematics_B.b_kstr_c - 1]) {
          exitg1 = 1;
        } else {
          inverse_kinematics_B.b_kstr_c++;
        }
      } else {
        inverse_kinematics_B.b_bool_n = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (inverse_kinematics_B.b_bool_n) {
    inverse_kinematics_B.b_kstr_c = 0;
  } else {
    for (inverse_kinematics_B.b_kstr_c = 0; inverse_kinematics_B.b_kstr_c < 9;
         inverse_kinematics_B.b_kstr_c++) {
      inverse_kinematics_B.b_e[inverse_kinematics_B.b_kstr_c] =
        tmp_6[inverse_kinematics_B.b_kstr_c];
    }

    inverse_kinematics_B.b_bool_n = false;
    if (jname->size[1] != 9) {
    } else {
      inverse_kinematics_B.b_kstr_c = 1;
      do {
        exitg1 = 0;
        if (inverse_kinematics_B.b_kstr_c - 1 < 9) {
          if (jname->data[inverse_kinematics_B.b_kstr_c - 1] !=
              inverse_kinematics_B.b_e[inverse_kinematics_B.b_kstr_c - 1]) {
            exitg1 = 1;
          } else {
            inverse_kinematics_B.b_kstr_c++;
          }
        } else {
          inverse_kinematics_B.b_bool_n = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (inverse_kinematics_B.b_bool_n) {
      inverse_kinematics_B.b_kstr_c = 1;
    } else {
      inverse_kinematics_B.b_kstr_c = -1;
    }
  }

  switch (inverse_kinematics_B.b_kstr_c) {
   case 0:
    inverse_kinematics_B.iv3[0] = 0;
    inverse_kinematics_B.iv3[1] = 0;
    inverse_kinematics_B.iv3[2] = 1;
    inverse_kinematics_B.iv3[3] = 0;
    inverse_kinematics_B.iv3[4] = 0;
    inverse_kinematics_B.iv3[5] = 0;
    for (inverse_kinematics_B.b_kstr_c = 0; inverse_kinematics_B.b_kstr_c < 6;
         inverse_kinematics_B.b_kstr_c++) {
      inverse_kinematics_B.msubspace_data_j[inverse_kinematics_B.b_kstr_c] =
        inverse_kinematics_B.iv3[inverse_kinematics_B.b_kstr_c];
    }

    inverse_kinematics_B.poslim_data_f[0] = -3.1415926535897931;
    inverse_kinematics_B.poslim_data_f[1] = 3.1415926535897931;
    obj->_pobj1[7].VelocityNumber = 1.0;
    obj->_pobj1[7].PositionNumber = 1.0;
    obj->_pobj1[7].JointAxisInternal[0] = 0.0;
    obj->_pobj1[7].JointAxisInternal[1] = 0.0;
    obj->_pobj1[7].JointAxisInternal[2] = 1.0;
    break;

   case 1:
    inverse_kinematics_B.iv3[0] = 0;
    inverse_kinematics_B.iv3[1] = 0;
    inverse_kinematics_B.iv3[2] = 0;
    inverse_kinematics_B.iv3[3] = 0;
    inverse_kinematics_B.iv3[4] = 0;
    inverse_kinematics_B.iv3[5] = 1;
    for (inverse_kinematics_B.b_kstr_c = 0; inverse_kinematics_B.b_kstr_c < 6;
         inverse_kinematics_B.b_kstr_c++) {
      inverse_kinematics_B.msubspace_data_j[inverse_kinematics_B.b_kstr_c] =
        inverse_kinematics_B.iv3[inverse_kinematics_B.b_kstr_c];
    }

    inverse_kinematics_B.poslim_data_f[0] = -0.5;
    inverse_kinematics_B.poslim_data_f[1] = 0.5;
    obj->_pobj1[7].VelocityNumber = 1.0;
    obj->_pobj1[7].PositionNumber = 1.0;
    obj->_pobj1[7].JointAxisInternal[0] = 0.0;
    obj->_pobj1[7].JointAxisInternal[1] = 0.0;
    obj->_pobj1[7].JointAxisInternal[2] = 1.0;
    break;

   default:
    for (inverse_kinematics_B.b_kstr_c = 0; inverse_kinematics_B.b_kstr_c < 6;
         inverse_kinematics_B.b_kstr_c++) {
      inverse_kinematics_B.msubspace_data_j[inverse_kinematics_B.b_kstr_c] = 0;
    }

    inverse_kinematics_B.poslim_data_f[0] = 0.0;
    inverse_kinematics_B.poslim_data_f[1] = 0.0;
    obj->_pobj1[7].VelocityNumber = 0.0;
    obj->_pobj1[7].PositionNumber = 0.0;
    obj->_pobj1[7].JointAxisInternal[0] = 0.0;
    obj->_pobj1[7].JointAxisInternal[1] = 0.0;
    obj->_pobj1[7].JointAxisInternal[2] = 0.0;
    break;
  }

  inverse_kinematics_B.i5 = obj->_pobj1[7].MotionSubspace->size[0] * obj->
    _pobj1[7].MotionSubspace->size[1];
  obj->_pobj1[7].MotionSubspace->size[0] = 6;
  obj->_pobj1[7].MotionSubspace->size[1] = 1;
  invers_emxEnsureCapacity_real_T(obj->_pobj1[7].MotionSubspace,
    inverse_kinematics_B.i5);
  for (inverse_kinematics_B.b_kstr_c = 0; inverse_kinematics_B.b_kstr_c < 6;
       inverse_kinematics_B.b_kstr_c++) {
    obj->_pobj1[7].MotionSubspace->data[inverse_kinematics_B.b_kstr_c] =
      inverse_kinematics_B.msubspace_data_j[inverse_kinematics_B.b_kstr_c];
  }

  inverse_kinematics_B.i5 = obj->_pobj1[7].PositionLimitsInternal->size[0] *
    obj->_pobj1[7].PositionLimitsInternal->size[1];
  obj->_pobj1[7].PositionLimitsInternal->size[0] = 1;
  obj->_pobj1[7].PositionLimitsInternal->size[1] = 2;
  invers_emxEnsureCapacity_real_T(obj->_pobj1[7].PositionLimitsInternal,
    inverse_kinematics_B.i5);
  for (inverse_kinematics_B.b_kstr_c = 0; inverse_kinematics_B.b_kstr_c < 2;
       inverse_kinematics_B.b_kstr_c++) {
    obj->_pobj1[7].PositionLimitsInternal->data[inverse_kinematics_B.b_kstr_c] =
      inverse_kinematics_B.poslim_data_f[inverse_kinematics_B.b_kstr_c];
  }

  inverse_kinematics_B.i5 = obj->_pobj1[7].HomePositionInternal->size[0];
  obj->_pobj1[7].HomePositionInternal->size[0] = 1;
  invers_emxEnsureCapacity_real_T(obj->_pobj1[7].HomePositionInternal,
    inverse_kinematics_B.i5);
  obj->_pobj1[7].HomePositionInternal->data[0] = 0.0;
  iobj_2->JointInternal = &obj->_pobj1[7];
  for (inverse_kinematics_B.b_kstr_c = 0; inverse_kinematics_B.b_kstr_c < 16;
       inverse_kinematics_B.b_kstr_c++) {
    iobj_2->JointInternal->JointToParentTransform[inverse_kinematics_B.b_kstr_c]
      = tmp_7[inverse_kinematics_B.b_kstr_c];
  }

  for (inverse_kinematics_B.b_kstr_c = 0; inverse_kinematics_B.b_kstr_c < 16;
       inverse_kinematics_B.b_kstr_c++) {
    iobj_2->JointInternal->ChildToJointTransform[inverse_kinematics_B.b_kstr_c] =
      tmp_8[inverse_kinematics_B.b_kstr_c];
  }

  inverse_kinematics_B.i5 = iobj_2->JointInternal->MotionSubspace->size[0] *
    iobj_2->JointInternal->MotionSubspace->size[1];
  iobj_2->JointInternal->MotionSubspace->size[0] = 6;
  iobj_2->JointInternal->MotionSubspace->size[1] = 1;
  invers_emxEnsureCapacity_real_T(iobj_2->JointInternal->MotionSubspace,
    inverse_kinematics_B.i5);
  for (inverse_kinematics_B.b_kstr_c = 0; inverse_kinematics_B.b_kstr_c < 6;
       inverse_kinematics_B.b_kstr_c++) {
    iobj_2->JointInternal->MotionSubspace->data[inverse_kinematics_B.b_kstr_c] =
      0.0;
  }

  iobj_2->JointInternal->InTree = true;
  inverse_kinematics_B.i5 = iobj_2->JointInternal->PositionLimitsInternal->size
    [0] * iobj_2->JointInternal->PositionLimitsInternal->size[1];
  iobj_2->JointInternal->PositionLimitsInternal->size[0] = 1;
  iobj_2->JointInternal->PositionLimitsInternal->size[1] = 2;
  invers_emxEnsureCapacity_real_T(iobj_2->JointInternal->PositionLimitsInternal,
    inverse_kinematics_B.i5);
  iobj_2->JointInternal->PositionLimitsInternal->data[0] = 0.0;
  iobj_2->JointInternal->PositionLimitsInternal->data[iobj_2->
    JointInternal->PositionLimitsInternal->size[0]] = 0.0;
  iobj_2->JointInternal->JointAxisInternal[0] = 0.0;
  iobj_2->JointInternal->JointAxisInternal[1] = 0.0;
  iobj_2->JointInternal->JointAxisInternal[2] = 0.0;
  inverse_kinematics_B.i5 = iobj_2->JointInternal->HomePositionInternal->size[0];
  iobj_2->JointInternal->HomePositionInternal->size[0] = 1;
  invers_emxEnsureCapacity_real_T(iobj_2->JointInternal->HomePositionInternal,
    inverse_kinematics_B.i5);
  iobj_2->JointInternal->HomePositionInternal->data[0] = 0.0;
  iobj_2->CollisionsInternal = inver_CollisionSet_CollisionSet(iobj_0, 0.0);
  obj->Bodies[0] = iobj_2;
  obj->Bodies[0]->Index = 1.0;
  iobj_2 = &obj->_pobj2[8];
  iobj_0 = &obj->_pobj0[8];
  inverse_kinematics_B.i5 = iobj_2->NameInternal->size[0] * iobj_2->
    NameInternal->size[1];
  iobj_2->NameInternal->size[0] = 1;
  iobj_2->NameInternal->size[1] = 5;
  invers_emxEnsureCapacity_char_T(iobj_2->NameInternal, inverse_kinematics_B.i5);
  for (inverse_kinematics_B.b_kstr_c = 0; inverse_kinematics_B.b_kstr_c < 5;
       inverse_kinematics_B.b_kstr_c++) {
    iobj_2->NameInternal->data[inverse_kinematics_B.b_kstr_c] =
      tmp_9[inverse_kinematics_B.b_kstr_c];
  }

  iobj_2->ParentIndex = 1.0;
  iobj_2->MassInternal = 0.680307;
  iobj_2->CenterOfMassInternal[0] = 0.00062594899999999967;
  iobj_2->CenterOfMassInternal[1] = -0.011001399999999996;
  iobj_2->CenterOfMassInternal[2] = -0.0532948;
  for (inverse_kinematics_B.b_kstr_c = 0; inverse_kinematics_B.b_kstr_c < 9;
       inverse_kinematics_B.b_kstr_c++) {
    iobj_2->InertiaInternal[inverse_kinematics_B.b_kstr_c] =
      tmp_a[inverse_kinematics_B.b_kstr_c];
  }

  for (inverse_kinematics_B.b_kstr_c = 0; inverse_kinematics_B.b_kstr_c < 36;
       inverse_kinematics_B.b_kstr_c++) {
    iobj_2->SpatialInertia[inverse_kinematics_B.b_kstr_c] =
      tmp_b[inverse_kinematics_B.b_kstr_c];
  }

  obj->_pobj1[8].InTree = false;
  for (inverse_kinematics_B.b_kstr_c = 0; inverse_kinematics_B.b_kstr_c < 16;
       inverse_kinematics_B.b_kstr_c++) {
    obj->_pobj1[8].JointToParentTransform[inverse_kinematics_B.b_kstr_c] =
      tmp_2[inverse_kinematics_B.b_kstr_c];
  }

  for (inverse_kinematics_B.b_kstr_c = 0; inverse_kinematics_B.b_kstr_c < 16;
       inverse_kinematics_B.b_kstr_c++) {
    obj->_pobj1[8].ChildToJointTransform[inverse_kinematics_B.b_kstr_c] =
      tmp_2[inverse_kinematics_B.b_kstr_c];
  }

  inverse_kinematics_B.i5 = obj->_pobj1[8].NameInternal->size[0] * obj->_pobj1[8]
    .NameInternal->size[1];
  obj->_pobj1[8].NameInternal->size[0] = 1;
  obj->_pobj1[8].NameInternal->size[1] = 6;
  invers_emxEnsureCapacity_char_T(obj->_pobj1[8].NameInternal,
    inverse_kinematics_B.i5);
  for (inverse_kinematics_B.b_kstr_c = 0; inverse_kinematics_B.b_kstr_c < 6;
       inverse_kinematics_B.b_kstr_c++) {
    obj->_pobj1[8].NameInternal->data[inverse_kinematics_B.b_kstr_c] =
      tmp_c[inverse_kinematics_B.b_kstr_c];
  }

  inverse_kinematics_B.i5 = obj->_pobj1[8].Type->size[0] * obj->_pobj1[8]
    .Type->size[1];
  obj->_pobj1[8].Type->size[0] = 1;
  obj->_pobj1[8].Type->size[1] = 8;
  invers_emxEnsureCapacity_char_T(obj->_pobj1[8].Type, inverse_kinematics_B.i5);
  for (inverse_kinematics_B.b_kstr_c = 0; inverse_kinematics_B.b_kstr_c < 8;
       inverse_kinematics_B.b_kstr_c++) {
    obj->_pobj1[8].Type->data[inverse_kinematics_B.b_kstr_c] =
      tmp_5[inverse_kinematics_B.b_kstr_c];
  }

  inverse_kinematics_B.i5 = jname->size[0] * jname->size[1];
  jname->size[0] = 1;
  jname->size[1] = obj->_pobj1[8].Type->size[1];
  invers_emxEnsureCapacity_char_T(jname, inverse_kinematics_B.i5);
  inverse_kinematics_B.loop_ub_a = obj->_pobj1[8].Type->size[1] - 1;
  for (inverse_kinematics_B.b_kstr_c = 0; inverse_kinematics_B.b_kstr_c <=
       inverse_kinematics_B.loop_ub_a; inverse_kinematics_B.b_kstr_c++) {
    inverse_kinematics_B.i5 = inverse_kinematics_B.b_kstr_c;
    jname->data[inverse_kinematics_B.i5] = obj->_pobj1[8].Type->
      data[inverse_kinematics_B.i5];
  }

  inverse_kinematics_B.b_bool_n = false;
  if (jname->size[1] != 8) {
  } else {
    inverse_kinematics_B.b_kstr_c = 1;
    do {
      exitg1 = 0;
      if (inverse_kinematics_B.b_kstr_c - 1 < 8) {
        if (jname->data[inverse_kinematics_B.b_kstr_c - 1] !=
            inverse_kinematics_B.b_o[inverse_kinematics_B.b_kstr_c - 1]) {
          exitg1 = 1;
        } else {
          inverse_kinematics_B.b_kstr_c++;
        }
      } else {
        inverse_kinematics_B.b_bool_n = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (inverse_kinematics_B.b_bool_n) {
    inverse_kinematics_B.b_kstr_c = 0;
  } else {
    for (inverse_kinematics_B.b_kstr_c = 0; inverse_kinematics_B.b_kstr_c < 9;
         inverse_kinematics_B.b_kstr_c++) {
      inverse_kinematics_B.b_e[inverse_kinematics_B.b_kstr_c] =
        tmp_6[inverse_kinematics_B.b_kstr_c];
    }

    inverse_kinematics_B.b_bool_n = false;
    if (jname->size[1] != 9) {
    } else {
      inverse_kinematics_B.b_kstr_c = 1;
      do {
        exitg1 = 0;
        if (inverse_kinematics_B.b_kstr_c - 1 < 9) {
          if (jname->data[inverse_kinematics_B.b_kstr_c - 1] !=
              inverse_kinematics_B.b_e[inverse_kinematics_B.b_kstr_c - 1]) {
            exitg1 = 1;
          } else {
            inverse_kinematics_B.b_kstr_c++;
          }
        } else {
          inverse_kinematics_B.b_bool_n = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (inverse_kinematics_B.b_bool_n) {
      inverse_kinematics_B.b_kstr_c = 1;
    } else {
      inverse_kinematics_B.b_kstr_c = -1;
    }
  }

  switch (inverse_kinematics_B.b_kstr_c) {
   case 0:
    inverse_kinematics_B.iv3[0] = 0;
    inverse_kinematics_B.iv3[1] = 0;
    inverse_kinematics_B.iv3[2] = 1;
    inverse_kinematics_B.iv3[3] = 0;
    inverse_kinematics_B.iv3[4] = 0;
    inverse_kinematics_B.iv3[5] = 0;
    for (inverse_kinematics_B.b_kstr_c = 0; inverse_kinematics_B.b_kstr_c < 6;
         inverse_kinematics_B.b_kstr_c++) {
      inverse_kinematics_B.msubspace_data_j[inverse_kinematics_B.b_kstr_c] =
        inverse_kinematics_B.iv3[inverse_kinematics_B.b_kstr_c];
    }

    inverse_kinematics_B.poslim_data_f[0] = -3.1415926535897931;
    inverse_kinematics_B.poslim_data_f[1] = 3.1415926535897931;
    obj->_pobj1[8].VelocityNumber = 1.0;
    obj->_pobj1[8].PositionNumber = 1.0;
    obj->_pobj1[8].JointAxisInternal[0] = 0.0;
    obj->_pobj1[8].JointAxisInternal[1] = 0.0;
    obj->_pobj1[8].JointAxisInternal[2] = 1.0;
    break;

   case 1:
    inverse_kinematics_B.iv3[0] = 0;
    inverse_kinematics_B.iv3[1] = 0;
    inverse_kinematics_B.iv3[2] = 0;
    inverse_kinematics_B.iv3[3] = 0;
    inverse_kinematics_B.iv3[4] = 0;
    inverse_kinematics_B.iv3[5] = 1;
    for (inverse_kinematics_B.b_kstr_c = 0; inverse_kinematics_B.b_kstr_c < 6;
         inverse_kinematics_B.b_kstr_c++) {
      inverse_kinematics_B.msubspace_data_j[inverse_kinematics_B.b_kstr_c] =
        inverse_kinematics_B.iv3[inverse_kinematics_B.b_kstr_c];
    }

    inverse_kinematics_B.poslim_data_f[0] = -0.5;
    inverse_kinematics_B.poslim_data_f[1] = 0.5;
    obj->_pobj1[8].VelocityNumber = 1.0;
    obj->_pobj1[8].PositionNumber = 1.0;
    obj->_pobj1[8].JointAxisInternal[0] = 0.0;
    obj->_pobj1[8].JointAxisInternal[1] = 0.0;
    obj->_pobj1[8].JointAxisInternal[2] = 1.0;
    break;

   default:
    for (inverse_kinematics_B.b_kstr_c = 0; inverse_kinematics_B.b_kstr_c < 6;
         inverse_kinematics_B.b_kstr_c++) {
      inverse_kinematics_B.msubspace_data_j[inverse_kinematics_B.b_kstr_c] = 0;
    }

    inverse_kinematics_B.poslim_data_f[0] = 0.0;
    inverse_kinematics_B.poslim_data_f[1] = 0.0;
    obj->_pobj1[8].VelocityNumber = 0.0;
    obj->_pobj1[8].PositionNumber = 0.0;
    obj->_pobj1[8].JointAxisInternal[0] = 0.0;
    obj->_pobj1[8].JointAxisInternal[1] = 0.0;
    obj->_pobj1[8].JointAxisInternal[2] = 0.0;
    break;
  }

  inverse_kinematics_B.i5 = obj->_pobj1[8].MotionSubspace->size[0] * obj->
    _pobj1[8].MotionSubspace->size[1];
  obj->_pobj1[8].MotionSubspace->size[0] = 6;
  obj->_pobj1[8].MotionSubspace->size[1] = 1;
  invers_emxEnsureCapacity_real_T(obj->_pobj1[8].MotionSubspace,
    inverse_kinematics_B.i5);
  for (inverse_kinematics_B.b_kstr_c = 0; inverse_kinematics_B.b_kstr_c < 6;
       inverse_kinematics_B.b_kstr_c++) {
    obj->_pobj1[8].MotionSubspace->data[inverse_kinematics_B.b_kstr_c] =
      inverse_kinematics_B.msubspace_data_j[inverse_kinematics_B.b_kstr_c];
  }

  inverse_kinematics_B.i5 = obj->_pobj1[8].PositionLimitsInternal->size[0] *
    obj->_pobj1[8].PositionLimitsInternal->size[1];
  obj->_pobj1[8].PositionLimitsInternal->size[0] = 1;
  obj->_pobj1[8].PositionLimitsInternal->size[1] = 2;
  invers_emxEnsureCapacity_real_T(obj->_pobj1[8].PositionLimitsInternal,
    inverse_kinematics_B.i5);
  for (inverse_kinematics_B.b_kstr_c = 0; inverse_kinematics_B.b_kstr_c < 2;
       inverse_kinematics_B.b_kstr_c++) {
    obj->_pobj1[8].PositionLimitsInternal->data[inverse_kinematics_B.b_kstr_c] =
      inverse_kinematics_B.poslim_data_f[inverse_kinematics_B.b_kstr_c];
  }

  inverse_kinematics_B.i5 = obj->_pobj1[8].HomePositionInternal->size[0];
  obj->_pobj1[8].HomePositionInternal->size[0] = 1;
  invers_emxEnsureCapacity_real_T(obj->_pobj1[8].HomePositionInternal,
    inverse_kinematics_B.i5);
  obj->_pobj1[8].HomePositionInternal->data[0] = 0.0;
  iobj_2->JointInternal = &obj->_pobj1[8];
  for (inverse_kinematics_B.b_kstr_c = 0; inverse_kinematics_B.b_kstr_c < 16;
       inverse_kinematics_B.b_kstr_c++) {
    iobj_2->JointInternal->JointToParentTransform[inverse_kinematics_B.b_kstr_c]
      = tmp_d[inverse_kinematics_B.b_kstr_c];
  }

  for (inverse_kinematics_B.b_kstr_c = 0; inverse_kinematics_B.b_kstr_c < 16;
       inverse_kinematics_B.b_kstr_c++) {
    iobj_2->JointInternal->ChildToJointTransform[inverse_kinematics_B.b_kstr_c] =
      tmp_8[inverse_kinematics_B.b_kstr_c];
  }

  inverse_kinematics_B.i5 = iobj_2->JointInternal->MotionSubspace->size[0] *
    iobj_2->JointInternal->MotionSubspace->size[1];
  iobj_2->JointInternal->MotionSubspace->size[0] = 6;
  iobj_2->JointInternal->MotionSubspace->size[1] = 1;
  invers_emxEnsureCapacity_real_T(iobj_2->JointInternal->MotionSubspace,
    inverse_kinematics_B.i5);
  for (inverse_kinematics_B.b_kstr_c = 0; inverse_kinematics_B.b_kstr_c < 6;
       inverse_kinematics_B.b_kstr_c++) {
    iobj_2->JointInternal->MotionSubspace->data[inverse_kinematics_B.b_kstr_c] =
      tmp_e[inverse_kinematics_B.b_kstr_c];
  }

  iobj_2->JointInternal->InTree = true;
  inverse_kinematics_B.i5 = iobj_2->JointInternal->PositionLimitsInternal->size
    [0] * iobj_2->JointInternal->PositionLimitsInternal->size[1];
  iobj_2->JointInternal->PositionLimitsInternal->size[0] = 1;
  iobj_2->JointInternal->PositionLimitsInternal->size[1] = 2;
  invers_emxEnsureCapacity_real_T(iobj_2->JointInternal->PositionLimitsInternal,
    inverse_kinematics_B.i5);
  iobj_2->JointInternal->PositionLimitsInternal->data[0] = -1.5708;
  iobj_2->JointInternal->PositionLimitsInternal->data[iobj_2->
    JointInternal->PositionLimitsInternal->size[0]] = 1.5708;
  iobj_2->JointInternal->JointAxisInternal[0] = 0.0;
  iobj_2->JointInternal->JointAxisInternal[1] = 0.0;
  iobj_2->JointInternal->JointAxisInternal[2] = 1.0;
  inverse_kinematics_B.i5 = iobj_2->JointInternal->HomePositionInternal->size[0];
  iobj_2->JointInternal->HomePositionInternal->size[0] = 1;
  invers_emxEnsureCapacity_real_T(iobj_2->JointInternal->HomePositionInternal,
    inverse_kinematics_B.i5);
  iobj_2->JointInternal->HomePositionInternal->data[0] = 0.0;
  iobj_2->CollisionsInternal = inver_CollisionSet_CollisionSet(iobj_0, 0.0);
  obj->Bodies[1] = iobj_2;
  obj->Bodies[1]->Index = 2.0;
  obj->Bodies[2] = inv_RigidBody_RigidBody_gaqyy1n(&obj->_pobj2[9], &obj->
    _pobj0[9], &obj->_pobj1[9]);
  obj->Bodies[2]->Index = 3.0;
  obj->Bodies[3] = in_RigidBody_RigidBody_gaqyy1no(&obj->_pobj2[10],
    &obj->_pobj0[10], &obj->_pobj1[10]);
  obj->Bodies[3]->Index = 4.0;
  obj->Bodies[4] = i_RigidBody_RigidBody_gaqyy1no5(&obj->_pobj2[11],
    &obj->_pobj0[11], &obj->_pobj1[11]);
  obj->Bodies[4]->Index = 5.0;
  obj->Bodies[5] = RigidBody_RigidBody_gaqyy1no5h(&obj->_pobj2[12], &obj->
    _pobj0[12], &obj->_pobj1[12]);
  obj->Bodies[5]->Index = 6.0;
  obj->Bodies[6] = RigidBody_RigidBody_gaqyy1no5hz(&obj->_pobj2[13],
    &obj->_pobj0[13], &obj->_pobj1[13]);
  obj->Bodies[6]->Index = 7.0;
  obj->Gravity[0] = 0.0;
  obj->Gravity[1] = 0.0;
  obj->Gravity[2] = -9.80665;
  iobj_2 = &obj->Base;
  iobj_0 = &obj->_pobj0[14];
  inverse_kinematics_B.i5 = iobj_2->NameInternal->size[0] * iobj_2->
    NameInternal->size[1];
  iobj_2->NameInternal->size[0] = 1;
  iobj_2->NameInternal->size[1] = 4;
  invers_emxEnsureCapacity_char_T(iobj_2->NameInternal, inverse_kinematics_B.i5);
  iobj_2->NameInternal->data[0] = 'B';
  iobj_2->NameInternal->data[1] = 'a';
  iobj_2->NameInternal->data[2] = 's';
  iobj_2->NameInternal->data[3] = 'e';
  iobj_2->ParentIndex = -1.0;
  iobj_2->MassInternal = 0.0;
  iobj_2->CenterOfMassInternal[0] = 0.0;
  iobj_2->CenterOfMassInternal[1] = 0.0;
  iobj_2->CenterOfMassInternal[2] = 0.0;
  for (inverse_kinematics_B.b_kstr_c = 0; inverse_kinematics_B.b_kstr_c < 9;
       inverse_kinematics_B.b_kstr_c++) {
    iobj_2->InertiaInternal[inverse_kinematics_B.b_kstr_c] = 0.0;
  }

  for (inverse_kinematics_B.b_kstr_c = 0; inverse_kinematics_B.b_kstr_c < 36;
       inverse_kinematics_B.b_kstr_c++) {
    iobj_2->SpatialInertia[inverse_kinematics_B.b_kstr_c] = 0.0;
  }

  inverse_kinematics_B.i5 = jname->size[0] * jname->size[1];
  jname->size[0] = 1;
  jname->size[1] = iobj_2->NameInternal->size[1] + 4;
  invers_emxEnsureCapacity_char_T(jname, inverse_kinematics_B.i5);
  inverse_kinematics_B.loop_ub_a = iobj_2->NameInternal->size[1];
  for (inverse_kinematics_B.b_kstr_c = 0; inverse_kinematics_B.b_kstr_c <
       inverse_kinematics_B.loop_ub_a; inverse_kinematics_B.b_kstr_c++) {
    inverse_kinematics_B.i5 = inverse_kinematics_B.b_kstr_c;
    jname->data[inverse_kinematics_B.i5] = iobj_2->NameInternal->
      data[inverse_kinematics_B.i5];
  }

  jname->data[inverse_kinematics_B.loop_ub_a] = '_';
  jname->data[inverse_kinematics_B.loop_ub_a + 1] = 'j';
  jname->data[inverse_kinematics_B.loop_ub_a + 2] = 'n';
  jname->data[inverse_kinematics_B.loop_ub_a + 3] = 't';
  iobj_2->JointInternal = i_rigidBodyJoint_rigidBodyJoint(&obj->_pobj1[14],
    jname);
  iobj_2->CollisionsInternal = inver_CollisionSet_CollisionSet(iobj_0, 0.0);
  obj->Base.Index = 0.0;
  inverse_kinemati_emxFree_char_T(&jname);
  return b_obj;
}

static void inverse_genrand_uint32_vector_g(uint32_T mt[625], uint32_T u[2])
{
  for (inverse_kinematics_B.b_j_h = 0; inverse_kinematics_B.b_j_h < 2;
       inverse_kinematics_B.b_j_h++) {
    inverse_kinematics_B.mti = mt[624] + 1U;
    if (mt[624] + 1U >= 625U) {
      for (inverse_kinematics_B.b_kk = 0; inverse_kinematics_B.b_kk < 227;
           inverse_kinematics_B.b_kk++) {
        inverse_kinematics_B.y_o = (mt[inverse_kinematics_B.b_kk + 1] &
          2147483647U) | (mt[inverse_kinematics_B.b_kk] & 2147483648U);
        if ((inverse_kinematics_B.y_o & 1U) == 0U) {
          inverse_kinematics_B.y_o >>= 1U;
        } else {
          inverse_kinematics_B.y_o = inverse_kinematics_B.y_o >> 1U ^
            2567483615U;
        }

        mt[inverse_kinematics_B.b_kk] = mt[inverse_kinematics_B.b_kk + 397] ^
          inverse_kinematics_B.y_o;
      }

      for (inverse_kinematics_B.b_kk = 0; inverse_kinematics_B.b_kk < 396;
           inverse_kinematics_B.b_kk++) {
        inverse_kinematics_B.y_o = (mt[inverse_kinematics_B.b_kk + 227] &
          2147483648U) | (mt[inverse_kinematics_B.b_kk + 228] & 2147483647U);
        if ((inverse_kinematics_B.y_o & 1U) == 0U) {
          inverse_kinematics_B.y_o >>= 1U;
        } else {
          inverse_kinematics_B.y_o = inverse_kinematics_B.y_o >> 1U ^
            2567483615U;
        }

        mt[inverse_kinematics_B.b_kk + 227] = mt[inverse_kinematics_B.b_kk] ^
          inverse_kinematics_B.y_o;
      }

      inverse_kinematics_B.y_o = (mt[623] & 2147483648U) | (mt[0] & 2147483647U);
      if ((inverse_kinematics_B.y_o & 1U) == 0U) {
        inverse_kinematics_B.y_o >>= 1U;
      } else {
        inverse_kinematics_B.y_o = inverse_kinematics_B.y_o >> 1U ^ 2567483615U;
      }

      mt[623] = mt[396] ^ inverse_kinematics_B.y_o;
      inverse_kinematics_B.mti = 1U;
    }

    inverse_kinematics_B.y_o = mt[static_cast<int32_T>(inverse_kinematics_B.mti)
      - 1];
    mt[624] = inverse_kinematics_B.mti;
    inverse_kinematics_B.y_o ^= inverse_kinematics_B.y_o >> 11U;
    inverse_kinematics_B.y_o ^= inverse_kinematics_B.y_o << 7U & 2636928640U;
    inverse_kinematics_B.y_o ^= inverse_kinematics_B.y_o << 15U & 4022730752U;
    u[inverse_kinematics_B.b_j_h] = inverse_kinematics_B.y_o >> 18U ^
      inverse_kinematics_B.y_o;
  }
}

static boolean_T inverse_kinemati_is_valid_state(const uint32_T mt[625])
{
  boolean_T isvalid;
  if ((mt[624] >= 1U) && (mt[624] < 625U)) {
    isvalid = true;
  } else {
    isvalid = false;
  }

  if (isvalid) {
    int32_T k;
    boolean_T exitg1;
    isvalid = false;
    k = 0;
    exitg1 = false;
    while ((!exitg1) && (k + 1 < 625)) {
      if (mt[k] == 0U) {
        k++;
      } else {
        isvalid = true;
        exitg1 = true;
      }
    }
  }

  return isvalid;
}

static void inverse_kinematics_rand(real_T r[5])
{
  uint32_T b_u[2];
  for (int32_T b_k = 0; b_k < 5; b_k++) {
    real_T b_r;

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
    int32_T exitg1;
    do {
      exitg1 = 0;
      inverse_genrand_uint32_vector_g(inverse_kinematics_DW.state_c, b_u);
      b_r = (static_cast<real_T>(b_u[0] >> 5U) * 6.7108864E+7 +
             static_cast<real_T>(b_u[1] >> 6U)) * 1.1102230246251565E-16;
      if (b_r == 0.0) {
        if (!inverse_kinemati_is_valid_state(inverse_kinematics_DW.state_c)) {
          inverse_kinematics_DW.state_c[0] = 5489U;
          inverse_kinematics_DW.state_c[624] = 624U;
        }
      } else {
        exitg1 = 1;
      }
    } while (exitg1 == 0);

    r[b_k] = b_r;
  }
}

static boolean_T inverse_kinematics_strcmp(const emxArray_char_T_inverse_kinem_T
  *a, const emxArray_char_T_inverse_kinem_T *b)
{
  boolean_T b_bool;
  b_bool = false;
  inverse_kinematics_B.d_m = (a->size[1] == 0);
  if (inverse_kinematics_B.d_m && (b->size[1] == 0)) {
    b_bool = true;
  } else if (a->size[1] != b->size[1]) {
  } else {
    inverse_kinematics_B.b_kstr_i = 1;
    int32_T exitg1;
    do {
      exitg1 = 0;
      if (inverse_kinematics_B.b_kstr_i - 1 <= b->size[1] - 1) {
        inverse_kinematics_B.i9 = inverse_kinematics_B.b_kstr_i - 1;
        if (a->data[inverse_kinematics_B.i9] != b->data[inverse_kinematics_B.i9])
        {
          exitg1 = 1;
        } else {
          inverse_kinematics_B.b_kstr_i++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  return b_bool;
}

static real_T RigidBodyTree_findBodyIndexByNa(v_robotics_manip_internal_Rig_T
  *obj, const emxArray_char_T_inverse_kinem_T *bodyname)
{
  emxArray_char_T_inverse_kinem_T *bname;
  t_robotics_manip_internal_Rig_T *obj_0;
  real_T bid;
  inverse_kinemati_emxInit_char_T(&bname, 2);
  bid = -1.0;
  inverse_kinematics_B.i7 = bname->size[0] * bname->size[1];
  bname->size[0] = 1;
  bname->size[1] = obj->Base.NameInternal->size[1];
  invers_emxEnsureCapacity_char_T(bname, inverse_kinematics_B.i7);
  inverse_kinematics_B.loop_ub_pe = obj->Base.NameInternal->size[1] - 1;
  for (inverse_kinematics_B.i7 = 0; inverse_kinematics_B.i7 <=
       inverse_kinematics_B.loop_ub_pe; inverse_kinematics_B.i7++) {
    inverse_kinematics_B.i8 = inverse_kinematics_B.i7;
    bname->data[inverse_kinematics_B.i8] = obj->Base.NameInternal->
      data[inverse_kinematics_B.i8];
  }

  if (inverse_kinematics_strcmp(bname, bodyname)) {
    bid = 0.0;
  } else {
    boolean_T exitg1;
    inverse_kinematics_B.b_p = obj->NumBodies;
    inverse_kinematics_B.b_i_l = 0;
    exitg1 = false;
    while ((!exitg1) && (inverse_kinematics_B.b_i_l <= static_cast<int32_T>
                         (inverse_kinematics_B.b_p) - 1)) {
      obj_0 = obj->Bodies[inverse_kinematics_B.b_i_l];
      inverse_kinematics_B.i7 = bname->size[0] * bname->size[1];
      bname->size[0] = 1;
      bname->size[1] = obj_0->NameInternal->size[1];
      invers_emxEnsureCapacity_char_T(bname, inverse_kinematics_B.i7);
      inverse_kinematics_B.loop_ub_pe = obj_0->NameInternal->size[1] - 1;
      for (inverse_kinematics_B.i7 = 0; inverse_kinematics_B.i7 <=
           inverse_kinematics_B.loop_ub_pe; inverse_kinematics_B.i7++) {
        inverse_kinematics_B.i8 = inverse_kinematics_B.i7;
        bname->data[inverse_kinematics_B.i8] = obj_0->NameInternal->
          data[inverse_kinematics_B.i8];
      }

      if (inverse_kinematics_strcmp(bname, bodyname)) {
        bid = static_cast<real_T>(inverse_kinematics_B.b_i_l) + 1.0;
        exitg1 = true;
      } else {
        inverse_kinematics_B.b_i_l++;
      }
    }
  }

  inverse_kinemati_emxFree_char_T(&bname);
  return bid;
}

static void inverse_kinemati_emxFree_real_T(emxArray_real_T_inverse_kinem_T
  **pEmxArray)
{
  if (*pEmxArray != static_cast<emxArray_real_T_inverse_kinem_T *>(NULL)) {
    if (((*pEmxArray)->data != static_cast<real_T *>(NULL)) && (*pEmxArray)
        ->canFreeData) {
      free((*pEmxArray)->data);
    }

    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = static_cast<emxArray_real_T_inverse_kinem_T *>(NULL);
  }
}

static t_robotics_manip_internal_Rig_T *inverse_kinemati_RigidBody_copy
  (t_robotics_manip_internal_Rig_T *obj, l_robotics_manip_internal_Col_T *iobj_0,
   c_rigidBodyJoint_inverse_kine_T *iobj_1, t_robotics_manip_internal_Rig_T
   *iobj_2)
{
  c_rigidBodyJoint_inverse_kine_T *obj_0;
  emxArray_char_T_inverse_kinem_T *jname;
  emxArray_char_T_inverse_kinem_T *jtype;
  emxArray_real_T_inverse_kinem_T *obj_3;
  k_robotics_manip_internal_Col_T *obj_2;
  l_robotics_manip_internal_Col_T *newObj;
  l_robotics_manip_internal_Col_T *obj_1;
  t_robotics_manip_internal_Rig_T *newbody;
  static const int8_T tmp[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1
  };

  static const char_T tmp_0[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_1[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const char_T tmp_2[5] = { 'f', 'i', 'x', 'e', 'd' };

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

  int32_T exitg1;
  boolean_T guard1 = false;
  boolean_T guard11 = false;
  boolean_T guard2 = false;
  boolean_T guard3 = false;
  inverse_kinemati_emxInit_char_T(&jtype, 2);
  inverse_kinematics_B.nmatched = jtype->size[0] * jtype->size[1];
  jtype->size[0] = 1;
  jtype->size[1] = obj->NameInternal->size[1];
  invers_emxEnsureCapacity_char_T(jtype, inverse_kinematics_B.nmatched);
  inverse_kinematics_B.loop_ub_k = obj->NameInternal->size[1] - 1;
  for (inverse_kinematics_B.minnanb = 0; inverse_kinematics_B.minnanb <=
       inverse_kinematics_B.loop_ub_k; inverse_kinematics_B.minnanb++) {
    inverse_kinematics_B.nmatched = inverse_kinematics_B.minnanb;
    jtype->data[inverse_kinematics_B.nmatched] = obj->NameInternal->
      data[inverse_kinematics_B.nmatched];
  }

  newbody = iobj_2;
  inverse_kinematics_B.nmatched = iobj_2->NameInternal->size[0] *
    iobj_2->NameInternal->size[1];
  iobj_2->NameInternal->size[0] = 1;
  iobj_2->NameInternal->size[1] = jtype->size[1];
  invers_emxEnsureCapacity_char_T(iobj_2->NameInternal,
    inverse_kinematics_B.nmatched);
  inverse_kinematics_B.loop_ub_k = jtype->size[1] - 1;
  for (inverse_kinematics_B.minnanb = 0; inverse_kinematics_B.minnanb <=
       inverse_kinematics_B.loop_ub_k; inverse_kinematics_B.minnanb++) {
    inverse_kinematics_B.nmatched = inverse_kinematics_B.minnanb;
    iobj_2->NameInternal->data[inverse_kinematics_B.nmatched] = jtype->
      data[inverse_kinematics_B.nmatched];
  }

  inverse_kinemati_emxInit_char_T(&jname, 2);
  inverse_kinematics_B.nmatched = jname->size[0] * jname->size[1];
  jname->size[0] = 1;
  jname->size[1] = jtype->size[1] + 4;
  invers_emxEnsureCapacity_char_T(jname, inverse_kinematics_B.nmatched);
  inverse_kinematics_B.loop_ub_k = jtype->size[1];
  if (inverse_kinematics_B.loop_ub_k - 1 >= 0) {
    memcpy(&jname->data[0], &jtype->data[0], inverse_kinematics_B.loop_ub_k *
           sizeof(char_T));
  }

  jname->data[jtype->size[1]] = '_';
  jname->data[jtype->size[1] + 1] = 'j';
  jname->data[jtype->size[1] + 2] = 'n';
  jname->data[jtype->size[1] + 3] = 't';
  iobj_2->JointInternal = i_rigidBodyJoint_rigidBodyJoint(&iobj_1[0], jname);
  iobj_2->Index = -1.0;
  iobj_2->ParentIndex = -1.0;
  iobj_2->MassInternal = 1.0;
  iobj_2->CenterOfMassInternal[0] = 0.0;
  iobj_2->CenterOfMassInternal[1] = 0.0;
  iobj_2->CenterOfMassInternal[2] = 0.0;
  for (inverse_kinematics_B.minnanb = 0; inverse_kinematics_B.minnanb < 9;
       inverse_kinematics_B.minnanb++) {
    inverse_kinematics_B.b_I_h[inverse_kinematics_B.minnanb] = 0;
  }

  inverse_kinematics_B.b_I_h[0] = 1;
  inverse_kinematics_B.b_I_h[4] = 1;
  inverse_kinematics_B.b_I_h[8] = 1;
  for (inverse_kinematics_B.minnanb = 0; inverse_kinematics_B.minnanb < 9;
       inverse_kinematics_B.minnanb++) {
    iobj_2->InertiaInternal[inverse_kinematics_B.minnanb] =
      inverse_kinematics_B.b_I_h[inverse_kinematics_B.minnanb];
  }

  for (inverse_kinematics_B.minnanb = 0; inverse_kinematics_B.minnanb < 36;
       inverse_kinematics_B.minnanb++) {
    inverse_kinematics_B.msubspace_data_p[inverse_kinematics_B.minnanb] = 0;
  }

  for (inverse_kinematics_B.minnanb = 0; inverse_kinematics_B.minnanb < 6;
       inverse_kinematics_B.minnanb++) {
    inverse_kinematics_B.msubspace_data_p[inverse_kinematics_B.minnanb + 6 *
      inverse_kinematics_B.minnanb] = 1;
  }

  for (inverse_kinematics_B.minnanb = 0; inverse_kinematics_B.minnanb < 36;
       inverse_kinematics_B.minnanb++) {
    iobj_2->SpatialInertia[inverse_kinematics_B.minnanb] =
      inverse_kinematics_B.msubspace_data_p[inverse_kinematics_B.minnanb];
  }

  iobj_2->CollisionsInternal = inver_CollisionSet_CollisionSet(&iobj_0[0], 0.0);
  obj_0 = obj->JointInternal;
  inverse_kinematics_B.nmatched = jtype->size[0] * jtype->size[1];
  jtype->size[0] = 1;
  jtype->size[1] = obj_0->Type->size[1];
  invers_emxEnsureCapacity_char_T(jtype, inverse_kinematics_B.nmatched);
  inverse_kinematics_B.loop_ub_k = obj_0->Type->size[1] - 1;
  for (inverse_kinematics_B.minnanb = 0; inverse_kinematics_B.minnanb <=
       inverse_kinematics_B.loop_ub_k; inverse_kinematics_B.minnanb++) {
    inverse_kinematics_B.nmatched = inverse_kinematics_B.minnanb;
    jtype->data[inverse_kinematics_B.nmatched] = obj_0->Type->
      data[inverse_kinematics_B.nmatched];
  }

  inverse_kinematics_B.nmatched = jname->size[0] * jname->size[1];
  jname->size[0] = 1;
  jname->size[1] = obj_0->NameInternal->size[1];
  invers_emxEnsureCapacity_char_T(jname, inverse_kinematics_B.nmatched);
  inverse_kinematics_B.loop_ub_k = obj_0->NameInternal->size[1] - 1;
  for (inverse_kinematics_B.minnanb = 0; inverse_kinematics_B.minnanb <=
       inverse_kinematics_B.loop_ub_k; inverse_kinematics_B.minnanb++) {
    inverse_kinematics_B.nmatched = inverse_kinematics_B.minnanb;
    jname->data[inverse_kinematics_B.nmatched] = obj_0->NameInternal->
      data[inverse_kinematics_B.nmatched];
  }

  iobj_1[1].InTree = false;
  for (inverse_kinematics_B.minnanb = 0; inverse_kinematics_B.minnanb < 16;
       inverse_kinematics_B.minnanb++) {
    iobj_1[1].JointToParentTransform[inverse_kinematics_B.minnanb] =
      tmp[inverse_kinematics_B.minnanb];
  }

  for (inverse_kinematics_B.minnanb = 0; inverse_kinematics_B.minnanb < 16;
       inverse_kinematics_B.minnanb++) {
    iobj_1[1].ChildToJointTransform[inverse_kinematics_B.minnanb] =
      tmp[inverse_kinematics_B.minnanb];
  }

  inverse_kinematics_B.nmatched = iobj_1[1].NameInternal->size[0] * iobj_1[1].
    NameInternal->size[1];
  iobj_1[1].NameInternal->size[0] = 1;
  iobj_1[1].NameInternal->size[1] = jname->size[1];
  invers_emxEnsureCapacity_char_T(iobj_1[1].NameInternal,
    inverse_kinematics_B.nmatched);
  inverse_kinematics_B.loop_ub_k = jname->size[1] - 1;
  for (inverse_kinematics_B.minnanb = 0; inverse_kinematics_B.minnanb <=
       inverse_kinematics_B.loop_ub_k; inverse_kinematics_B.minnanb++) {
    inverse_kinematics_B.nmatched = inverse_kinematics_B.minnanb;
    iobj_1[1].NameInternal->data[inverse_kinematics_B.nmatched] = jname->
      data[inverse_kinematics_B.nmatched];
  }

  inverse_kinemati_emxFree_char_T(&jname);
  inverse_kinematics_B.partial_match_size_idx_1 = 8;
  inverse_kinematics_B.nmatched = 0;
  inverse_kinematics_B.matched = false;
  for (inverse_kinematics_B.minnanb = 0; inverse_kinematics_B.minnanb < 8;
       inverse_kinematics_B.minnanb++) {
    inverse_kinematics_B.partial_match_data[inverse_kinematics_B.minnanb] = ' ';
    inverse_kinematics_B.vstr[inverse_kinematics_B.minnanb] =
      tmp_0[inverse_kinematics_B.minnanb];
  }

  guard1 = false;
  guard2 = false;
  guard3 = false;
  if (jtype->size[1] <= 8) {
    inverse_kinematics_B.loop_ub_k = jtype->size[1];
    for (inverse_kinematics_B.minnanb = 0; inverse_kinematics_B.minnanb < 8;
         inverse_kinematics_B.minnanb++) {
      inverse_kinematics_B.b_ju[inverse_kinematics_B.minnanb] =
        tmp_0[inverse_kinematics_B.minnanb];
    }

    inverse_kinematics_B.b_bool_k = false;
    inverse_kinematics_B.minnanb = jtype->size[1];
    guard11 = false;
    if (inverse_kinematics_B.loop_ub_k <= inverse_kinematics_B.minnanb) {
      if (inverse_kinematics_B.minnanb <= inverse_kinematics_B.loop_ub_k) {
        inverse_kinematics_B.loop_ub_k = inverse_kinematics_B.minnanb;
      }

      inverse_kinematics_B.minnanb = inverse_kinematics_B.loop_ub_k - 1;
      guard11 = true;
    } else if (jtype->size[1] == 8) {
      inverse_kinematics_B.minnanb = 7;
      guard11 = true;
    }

    if (guard11) {
      inverse_kinematics_B.loop_ub_k = 1;
      do {
        exitg1 = 0;
        if (inverse_kinematics_B.loop_ub_k - 1 <= inverse_kinematics_B.minnanb)
        {
          if (tmp_3[static_cast<uint8_T>(jtype->
               data[inverse_kinematics_B.loop_ub_k - 1]) & 127] != tmp_3[
              static_cast<int32_T>
              (inverse_kinematics_B.b_ju[inverse_kinematics_B.loop_ub_k - 1])])
          {
            exitg1 = 1;
          } else {
            inverse_kinematics_B.loop_ub_k++;
          }
        } else {
          inverse_kinematics_B.b_bool_k = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (inverse_kinematics_B.b_bool_k) {
      if (jtype->size[1] == 8) {
        inverse_kinematics_B.nmatched = 1;
        inverse_kinematics_B.partial_match_size_idx_1 = 8;
        for (inverse_kinematics_B.minnanb = 0; inverse_kinematics_B.minnanb < 8;
             inverse_kinematics_B.minnanb++) {
          inverse_kinematics_B.b_b[inverse_kinematics_B.minnanb] =
            inverse_kinematics_B.vstr[inverse_kinematics_B.minnanb];
        }
      } else {
        inverse_kinematics_B.partial_match_size_idx_1 = 8;
        for (inverse_kinematics_B.minnanb = 0; inverse_kinematics_B.minnanb < 8;
             inverse_kinematics_B.minnanb++) {
          inverse_kinematics_B.partial_match_data[inverse_kinematics_B.minnanb] =
            inverse_kinematics_B.vstr[inverse_kinematics_B.minnanb];
        }

        inverse_kinematics_B.matched = true;
        inverse_kinematics_B.nmatched = 1;
        guard3 = true;
      }
    } else {
      guard3 = true;
    }
  } else {
    guard3 = true;
  }

  if (guard3) {
    for (inverse_kinematics_B.minnanb = 0; inverse_kinematics_B.minnanb < 9;
         inverse_kinematics_B.minnanb++) {
      inverse_kinematics_B.b_vstr[inverse_kinematics_B.minnanb] =
        tmp_1[inverse_kinematics_B.minnanb];
    }

    if (jtype->size[1] <= 9) {
      inverse_kinematics_B.loop_ub_k = jtype->size[1];
      for (inverse_kinematics_B.minnanb = 0; inverse_kinematics_B.minnanb < 9;
           inverse_kinematics_B.minnanb++) {
        inverse_kinematics_B.b_b[inverse_kinematics_B.minnanb] =
          tmp_1[inverse_kinematics_B.minnanb];
      }

      inverse_kinematics_B.b_bool_k = false;
      inverse_kinematics_B.minnanb = jtype->size[1];
      guard11 = false;
      if (inverse_kinematics_B.loop_ub_k <= inverse_kinematics_B.minnanb) {
        if (inverse_kinematics_B.minnanb <= inverse_kinematics_B.loop_ub_k) {
          inverse_kinematics_B.loop_ub_k = inverse_kinematics_B.minnanb;
        }

        inverse_kinematics_B.minnanb = inverse_kinematics_B.loop_ub_k - 1;
        guard11 = true;
      } else if (jtype->size[1] == 9) {
        inverse_kinematics_B.minnanb = 8;
        guard11 = true;
      }

      if (guard11) {
        inverse_kinematics_B.loop_ub_k = 1;
        do {
          exitg1 = 0;
          if (inverse_kinematics_B.loop_ub_k - 1 <= inverse_kinematics_B.minnanb)
          {
            if (tmp_3[static_cast<uint8_T>(jtype->
                 data[inverse_kinematics_B.loop_ub_k - 1]) & 127] != tmp_3[
                static_cast<int32_T>
                (inverse_kinematics_B.b_b[inverse_kinematics_B.loop_ub_k - 1])])
            {
              exitg1 = 1;
            } else {
              inverse_kinematics_B.loop_ub_k++;
            }
          } else {
            inverse_kinematics_B.b_bool_k = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }

      if (inverse_kinematics_B.b_bool_k) {
        if (jtype->size[1] == 9) {
          inverse_kinematics_B.nmatched = 1;
          inverse_kinematics_B.partial_match_size_idx_1 = 9;
          for (inverse_kinematics_B.minnanb = 0; inverse_kinematics_B.minnanb <
               9; inverse_kinematics_B.minnanb++) {
            inverse_kinematics_B.b_b[inverse_kinematics_B.minnanb] =
              inverse_kinematics_B.b_vstr[inverse_kinematics_B.minnanb];
          }
        } else {
          if (!inverse_kinematics_B.matched) {
            inverse_kinematics_B.partial_match_size_idx_1 = 9;
            for (inverse_kinematics_B.minnanb = 0; inverse_kinematics_B.minnanb <
                 9; inverse_kinematics_B.minnanb++) {
              inverse_kinematics_B.partial_match_data[inverse_kinematics_B.minnanb]
                = inverse_kinematics_B.b_vstr[inverse_kinematics_B.minnanb];
            }
          }

          inverse_kinematics_B.matched = true;
          inverse_kinematics_B.nmatched++;
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
    for (inverse_kinematics_B.minnanb = 0; inverse_kinematics_B.minnanb < 5;
         inverse_kinematics_B.minnanb++) {
      inverse_kinematics_B.c_vstr[inverse_kinematics_B.minnanb] =
        tmp_2[inverse_kinematics_B.minnanb];
    }

    if (jtype->size[1] <= 5) {
      inverse_kinematics_B.loop_ub_k = jtype->size[1];
      for (inverse_kinematics_B.minnanb = 0; inverse_kinematics_B.minnanb < 5;
           inverse_kinematics_B.minnanb++) {
        inverse_kinematics_B.b_as[inverse_kinematics_B.minnanb] =
          tmp_2[inverse_kinematics_B.minnanb];
      }

      inverse_kinematics_B.b_bool_k = false;
      inverse_kinematics_B.minnanb = jtype->size[1];
      guard11 = false;
      if (inverse_kinematics_B.loop_ub_k <= inverse_kinematics_B.minnanb) {
        if (inverse_kinematics_B.minnanb <= inverse_kinematics_B.loop_ub_k) {
          inverse_kinematics_B.loop_ub_k = inverse_kinematics_B.minnanb;
        }

        inverse_kinematics_B.minnanb = inverse_kinematics_B.loop_ub_k - 1;
        guard11 = true;
      } else if (jtype->size[1] == 5) {
        inverse_kinematics_B.minnanb = 4;
        guard11 = true;
      }

      if (guard11) {
        inverse_kinematics_B.loop_ub_k = 1;
        do {
          exitg1 = 0;
          if (inverse_kinematics_B.loop_ub_k - 1 <= inverse_kinematics_B.minnanb)
          {
            if (tmp_3[static_cast<uint8_T>(jtype->
                 data[inverse_kinematics_B.loop_ub_k - 1]) & 127] != tmp_3[
                static_cast<int32_T>
                (inverse_kinematics_B.b_as[inverse_kinematics_B.loop_ub_k - 1])])
            {
              exitg1 = 1;
            } else {
              inverse_kinematics_B.loop_ub_k++;
            }
          } else {
            inverse_kinematics_B.b_bool_k = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }

      if (inverse_kinematics_B.b_bool_k) {
        if (jtype->size[1] == 5) {
          inverse_kinematics_B.nmatched = 1;
          inverse_kinematics_B.partial_match_size_idx_1 = 5;
          for (inverse_kinematics_B.minnanb = 0; inverse_kinematics_B.minnanb <
               5; inverse_kinematics_B.minnanb++) {
            inverse_kinematics_B.b_b[inverse_kinematics_B.minnanb] =
              inverse_kinematics_B.c_vstr[inverse_kinematics_B.minnanb];
          }
        } else {
          if (!inverse_kinematics_B.matched) {
            inverse_kinematics_B.partial_match_size_idx_1 = 5;
            for (inverse_kinematics_B.minnanb = 0; inverse_kinematics_B.minnanb <
                 5; inverse_kinematics_B.minnanb++) {
              inverse_kinematics_B.partial_match_data[inverse_kinematics_B.minnanb]
                = inverse_kinematics_B.c_vstr[inverse_kinematics_B.minnanb];
            }
          }

          inverse_kinematics_B.nmatched++;
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
    if (inverse_kinematics_B.nmatched == 0) {
      inverse_kinematics_B.partial_match_size_idx_1 = 8;
      for (inverse_kinematics_B.minnanb = 0; inverse_kinematics_B.minnanb < 8;
           inverse_kinematics_B.minnanb++) {
        inverse_kinematics_B.b_b[inverse_kinematics_B.minnanb] = ' ';
      }
    } else {
      inverse_kinematics_B.loop_ub_k =
        inverse_kinematics_B.partial_match_size_idx_1 - 1;
      memcpy(&inverse_kinematics_B.b_b[0],
             &inverse_kinematics_B.partial_match_data[0],
             (inverse_kinematics_B.loop_ub_k + 1) * sizeof(char_T));
    }
  }

  if ((inverse_kinematics_B.nmatched == 0) || (jtype->size[1] == 0)) {
    inverse_kinematics_B.partial_match_size_idx_1 = 8;
    for (inverse_kinematics_B.minnanb = 0; inverse_kinematics_B.minnanb < 8;
         inverse_kinematics_B.minnanb++) {
      inverse_kinematics_B.partial_match_data[inverse_kinematics_B.minnanb] =
        ' ';
    }
  } else {
    inverse_kinematics_B.loop_ub_k =
      inverse_kinematics_B.partial_match_size_idx_1 - 1;
    memcpy(&inverse_kinematics_B.partial_match_data[0],
           &inverse_kinematics_B.b_b[0], (inverse_kinematics_B.loop_ub_k + 1) *
           sizeof(char_T));
  }

  inverse_kinematics_B.nmatched = iobj_1[1].Type->size[0] * iobj_1[1].Type->
    size[1];
  iobj_1[1].Type->size[0] = 1;
  iobj_1[1].Type->size[1] = inverse_kinematics_B.partial_match_size_idx_1;
  invers_emxEnsureCapacity_char_T(iobj_1[1].Type, inverse_kinematics_B.nmatched);
  inverse_kinematics_B.loop_ub_k = inverse_kinematics_B.partial_match_size_idx_1
    - 1;
  for (inverse_kinematics_B.minnanb = 0; inverse_kinematics_B.minnanb <=
       inverse_kinematics_B.loop_ub_k; inverse_kinematics_B.minnanb++) {
    inverse_kinematics_B.nmatched = inverse_kinematics_B.minnanb;
    iobj_1[1].Type->data[inverse_kinematics_B.nmatched] =
      inverse_kinematics_B.partial_match_data[inverse_kinematics_B.nmatched];
  }

  inverse_kinematics_B.nmatched = jtype->size[0] * jtype->size[1];
  jtype->size[0] = 1;
  jtype->size[1] = iobj_1[1].Type->size[1];
  invers_emxEnsureCapacity_char_T(jtype, inverse_kinematics_B.nmatched);
  inverse_kinematics_B.loop_ub_k = iobj_1[1].Type->size[1] - 1;
  for (inverse_kinematics_B.minnanb = 0; inverse_kinematics_B.minnanb <=
       inverse_kinematics_B.loop_ub_k; inverse_kinematics_B.minnanb++) {
    inverse_kinematics_B.nmatched = inverse_kinematics_B.minnanb;
    jtype->data[inverse_kinematics_B.nmatched] = iobj_1[1].Type->
      data[inverse_kinematics_B.nmatched];
  }

  for (inverse_kinematics_B.minnanb = 0; inverse_kinematics_B.minnanb < 8;
       inverse_kinematics_B.minnanb++) {
    inverse_kinematics_B.b_ju[inverse_kinematics_B.minnanb] =
      tmp_0[inverse_kinematics_B.minnanb];
  }

  inverse_kinematics_B.b_bool_k = false;
  if (jtype->size[1] != 8) {
  } else {
    inverse_kinematics_B.loop_ub_k = 1;
    do {
      exitg1 = 0;
      if (inverse_kinematics_B.loop_ub_k - 1 < 8) {
        if (jtype->data[inverse_kinematics_B.loop_ub_k - 1] !=
            inverse_kinematics_B.b_ju[inverse_kinematics_B.loop_ub_k - 1]) {
          exitg1 = 1;
        } else {
          inverse_kinematics_B.loop_ub_k++;
        }
      } else {
        inverse_kinematics_B.b_bool_k = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (inverse_kinematics_B.b_bool_k) {
    inverse_kinematics_B.minnanb = 0;
  } else {
    for (inverse_kinematics_B.minnanb = 0; inverse_kinematics_B.minnanb < 9;
         inverse_kinematics_B.minnanb++) {
      inverse_kinematics_B.b_b[inverse_kinematics_B.minnanb] =
        tmp_1[inverse_kinematics_B.minnanb];
    }

    inverse_kinematics_B.b_bool_k = false;
    if (jtype->size[1] != 9) {
    } else {
      inverse_kinematics_B.loop_ub_k = 1;
      do {
        exitg1 = 0;
        if (inverse_kinematics_B.loop_ub_k - 1 < 9) {
          if (jtype->data[inverse_kinematics_B.loop_ub_k - 1] !=
              inverse_kinematics_B.b_b[inverse_kinematics_B.loop_ub_k - 1]) {
            exitg1 = 1;
          } else {
            inverse_kinematics_B.loop_ub_k++;
          }
        } else {
          inverse_kinematics_B.b_bool_k = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (inverse_kinematics_B.b_bool_k) {
      inverse_kinematics_B.minnanb = 1;
    } else {
      inverse_kinematics_B.minnanb = -1;
    }
  }

  switch (inverse_kinematics_B.minnanb) {
   case 0:
    inverse_kinematics_B.iv1[0] = 0;
    inverse_kinematics_B.iv1[1] = 0;
    inverse_kinematics_B.iv1[2] = 1;
    inverse_kinematics_B.iv1[3] = 0;
    inverse_kinematics_B.iv1[4] = 0;
    inverse_kinematics_B.iv1[5] = 0;
    for (inverse_kinematics_B.minnanb = 0; inverse_kinematics_B.minnanb < 6;
         inverse_kinematics_B.minnanb++) {
      inverse_kinematics_B.msubspace_data_p[inverse_kinematics_B.minnanb] =
        inverse_kinematics_B.iv1[inverse_kinematics_B.minnanb];
    }

    inverse_kinematics_B.poslim_data_p[0] = -3.1415926535897931;
    inverse_kinematics_B.poslim_data_p[1] = 3.1415926535897931;
    iobj_1[1].VelocityNumber = 1.0;
    iobj_1[1].PositionNumber = 1.0;
    iobj_1[1].JointAxisInternal[0] = 0.0;
    iobj_1[1].JointAxisInternal[1] = 0.0;
    iobj_1[1].JointAxisInternal[2] = 1.0;
    break;

   case 1:
    inverse_kinematics_B.iv1[0] = 0;
    inverse_kinematics_B.iv1[1] = 0;
    inverse_kinematics_B.iv1[2] = 0;
    inverse_kinematics_B.iv1[3] = 0;
    inverse_kinematics_B.iv1[4] = 0;
    inverse_kinematics_B.iv1[5] = 1;
    for (inverse_kinematics_B.minnanb = 0; inverse_kinematics_B.minnanb < 6;
         inverse_kinematics_B.minnanb++) {
      inverse_kinematics_B.msubspace_data_p[inverse_kinematics_B.minnanb] =
        inverse_kinematics_B.iv1[inverse_kinematics_B.minnanb];
    }

    inverse_kinematics_B.poslim_data_p[0] = -0.5;
    inverse_kinematics_B.poslim_data_p[1] = 0.5;
    iobj_1[1].VelocityNumber = 1.0;
    iobj_1[1].PositionNumber = 1.0;
    iobj_1[1].JointAxisInternal[0] = 0.0;
    iobj_1[1].JointAxisInternal[1] = 0.0;
    iobj_1[1].JointAxisInternal[2] = 1.0;
    break;

   default:
    for (inverse_kinematics_B.minnanb = 0; inverse_kinematics_B.minnanb < 6;
         inverse_kinematics_B.minnanb++) {
      inverse_kinematics_B.msubspace_data_p[inverse_kinematics_B.minnanb] = 0;
    }

    inverse_kinematics_B.poslim_data_p[0] = 0.0;
    inverse_kinematics_B.poslim_data_p[1] = 0.0;
    iobj_1[1].VelocityNumber = 0.0;
    iobj_1[1].PositionNumber = 0.0;
    iobj_1[1].JointAxisInternal[0] = 0.0;
    iobj_1[1].JointAxisInternal[1] = 0.0;
    iobj_1[1].JointAxisInternal[2] = 0.0;
    break;
  }

  inverse_kinematics_B.nmatched = iobj_1[1].MotionSubspace->size[0] * iobj_1[1].
    MotionSubspace->size[1];
  iobj_1[1].MotionSubspace->size[0] = 6;
  iobj_1[1].MotionSubspace->size[1] = 1;
  invers_emxEnsureCapacity_real_T(iobj_1[1].MotionSubspace,
    inverse_kinematics_B.nmatched);
  for (inverse_kinematics_B.minnanb = 0; inverse_kinematics_B.minnanb < 6;
       inverse_kinematics_B.minnanb++) {
    iobj_1[1].MotionSubspace->data[inverse_kinematics_B.minnanb] =
      inverse_kinematics_B.msubspace_data_p[inverse_kinematics_B.minnanb];
  }

  inverse_kinematics_B.nmatched = iobj_1[1].PositionLimitsInternal->size[0] *
    iobj_1[1].PositionLimitsInternal->size[1];
  iobj_1[1].PositionLimitsInternal->size[0] = 1;
  iobj_1[1].PositionLimitsInternal->size[1] = 2;
  invers_emxEnsureCapacity_real_T(iobj_1[1].PositionLimitsInternal,
    inverse_kinematics_B.nmatched);
  for (inverse_kinematics_B.minnanb = 0; inverse_kinematics_B.minnanb < 2;
       inverse_kinematics_B.minnanb++) {
    iobj_1[1].PositionLimitsInternal->data[inverse_kinematics_B.minnanb] =
      inverse_kinematics_B.poslim_data_p[inverse_kinematics_B.minnanb];
  }

  inverse_kinematics_B.nmatched = iobj_1[1].HomePositionInternal->size[0];
  iobj_1[1].HomePositionInternal->size[0] = 1;
  invers_emxEnsureCapacity_real_T(iobj_1[1].HomePositionInternal,
    inverse_kinematics_B.nmatched);
  iobj_1[1].HomePositionInternal->data[0] = 0.0;
  inverse_kinematics_B.nmatched = jtype->size[0] * jtype->size[1];
  jtype->size[0] = 1;
  jtype->size[1] = obj_0->NameInternal->size[1];
  invers_emxEnsureCapacity_char_T(jtype, inverse_kinematics_B.nmatched);
  inverse_kinematics_B.loop_ub_k = obj_0->NameInternal->size[1] - 1;
  for (inverse_kinematics_B.minnanb = 0; inverse_kinematics_B.minnanb <=
       inverse_kinematics_B.loop_ub_k; inverse_kinematics_B.minnanb++) {
    inverse_kinematics_B.nmatched = inverse_kinematics_B.minnanb;
    jtype->data[inverse_kinematics_B.nmatched] = obj_0->NameInternal->
      data[inverse_kinematics_B.nmatched];
  }

  if (jtype->size[1] != 0) {
    inverse_kinematics_B.nmatched = jtype->size[0] * jtype->size[1];
    jtype->size[0] = 1;
    jtype->size[1] = obj_0->NameInternal->size[1];
    invers_emxEnsureCapacity_char_T(jtype, inverse_kinematics_B.nmatched);
    inverse_kinematics_B.loop_ub_k = obj_0->NameInternal->size[1] - 1;
    for (inverse_kinematics_B.minnanb = 0; inverse_kinematics_B.minnanb <=
         inverse_kinematics_B.loop_ub_k; inverse_kinematics_B.minnanb++) {
      inverse_kinematics_B.nmatched = inverse_kinematics_B.minnanb;
      jtype->data[inverse_kinematics_B.nmatched] = obj_0->NameInternal->
        data[inverse_kinematics_B.nmatched];
    }

    if (!iobj_1[1].InTree) {
      inverse_kinematics_B.nmatched = iobj_1[1].NameInternal->size[0] * iobj_1[1]
        .NameInternal->size[1];
      iobj_1[1].NameInternal->size[0] = 1;
      iobj_1[1].NameInternal->size[1] = jtype->size[1];
      invers_emxEnsureCapacity_char_T(iobj_1[1].NameInternal,
        inverse_kinematics_B.nmatched);
      inverse_kinematics_B.loop_ub_k = jtype->size[1] - 1;
      for (inverse_kinematics_B.minnanb = 0; inverse_kinematics_B.minnanb <=
           inverse_kinematics_B.loop_ub_k; inverse_kinematics_B.minnanb++) {
        inverse_kinematics_B.nmatched = inverse_kinematics_B.minnanb;
        iobj_1[1].NameInternal->data[inverse_kinematics_B.nmatched] =
          jtype->data[inverse_kinematics_B.nmatched];
      }
    }
  }

  inverse_kinemati_emxFree_char_T(&jtype);
  inverse_kinemati_emxInit_real_T(&obj_3, 1);
  inverse_kinematics_B.loop_ub_k = obj_0->PositionLimitsInternal->size[0] << 1;
  inverse_kinematics_B.nmatched = iobj_1[1].PositionLimitsInternal->size[0] *
    iobj_1[1].PositionLimitsInternal->size[1];
  iobj_1[1].PositionLimitsInternal->size[0] = obj_0->
    PositionLimitsInternal->size[0];
  iobj_1[1].PositionLimitsInternal->size[1] = 2;
  invers_emxEnsureCapacity_real_T(iobj_1[1].PositionLimitsInternal,
    inverse_kinematics_B.nmatched);
  inverse_kinematics_B.nmatched = obj_3->size[0];
  obj_3->size[0] = inverse_kinematics_B.loop_ub_k;
  invers_emxEnsureCapacity_real_T(obj_3, inverse_kinematics_B.nmatched);
  for (inverse_kinematics_B.minnanb = 0; inverse_kinematics_B.minnanb <
       inverse_kinematics_B.loop_ub_k; inverse_kinematics_B.minnanb++) {
    obj_3->data[inverse_kinematics_B.minnanb] = obj_0->
      PositionLimitsInternal->data[inverse_kinematics_B.minnanb];
  }

  inverse_kinematics_B.loop_ub_k = obj_3->size[0];
  for (inverse_kinematics_B.minnanb = 0; inverse_kinematics_B.minnanb <
       inverse_kinematics_B.loop_ub_k; inverse_kinematics_B.minnanb++) {
    iobj_1[1].PositionLimitsInternal->data[inverse_kinematics_B.minnanb] =
      obj_3->data[inverse_kinematics_B.minnanb];
  }

  inverse_kinematics_B.nmatched = obj_3->size[0];
  obj_3->size[0] = obj_0->HomePositionInternal->size[0];
  invers_emxEnsureCapacity_real_T(obj_3, inverse_kinematics_B.nmatched);
  inverse_kinematics_B.loop_ub_k = obj_0->HomePositionInternal->size[0];
  for (inverse_kinematics_B.minnanb = 0; inverse_kinematics_B.minnanb <
       inverse_kinematics_B.loop_ub_k; inverse_kinematics_B.minnanb++) {
    obj_3->data[inverse_kinematics_B.minnanb] = obj_0->
      HomePositionInternal->data[inverse_kinematics_B.minnanb];
  }

  inverse_kinematics_B.nmatched = iobj_1[1].HomePositionInternal->size[0];
  iobj_1[1].HomePositionInternal->size[0] = obj_3->size[0];
  invers_emxEnsureCapacity_real_T(iobj_1[1].HomePositionInternal,
    inverse_kinematics_B.nmatched);
  inverse_kinematics_B.loop_ub_k = obj_3->size[0];
  for (inverse_kinematics_B.minnanb = 0; inverse_kinematics_B.minnanb <
       inverse_kinematics_B.loop_ub_k; inverse_kinematics_B.minnanb++) {
    iobj_1[1].HomePositionInternal->data[inverse_kinematics_B.minnanb] =
      obj_3->data[inverse_kinematics_B.minnanb];
  }

  inverse_kinematics_B.obj_idx_0 = obj_0->JointAxisInternal[0];
  inverse_kinematics_B.obj_idx_1 = obj_0->JointAxisInternal[1];
  inverse_kinematics_B.obj_idx_2 = obj_0->JointAxisInternal[2];
  iobj_1[1].JointAxisInternal[0] = inverse_kinematics_B.obj_idx_0;
  iobj_1[1].JointAxisInternal[1] = inverse_kinematics_B.obj_idx_1;
  iobj_1[1].JointAxisInternal[2] = inverse_kinematics_B.obj_idx_2;
  inverse_kinematics_B.loop_ub_k = 6 * obj_0->MotionSubspace->size[1];
  inverse_kinematics_B.nmatched = iobj_1[1].MotionSubspace->size[0] * iobj_1[1].
    MotionSubspace->size[1];
  iobj_1[1].MotionSubspace->size[0] = 6;
  iobj_1[1].MotionSubspace->size[1] = obj_0->MotionSubspace->size[1];
  invers_emxEnsureCapacity_real_T(iobj_1[1].MotionSubspace,
    inverse_kinematics_B.nmatched);
  inverse_kinematics_B.nmatched = obj_3->size[0];
  obj_3->size[0] = inverse_kinematics_B.loop_ub_k;
  invers_emxEnsureCapacity_real_T(obj_3, inverse_kinematics_B.nmatched);
  for (inverse_kinematics_B.minnanb = 0; inverse_kinematics_B.minnanb <
       inverse_kinematics_B.loop_ub_k; inverse_kinematics_B.minnanb++) {
    obj_3->data[inverse_kinematics_B.minnanb] = obj_0->MotionSubspace->
      data[inverse_kinematics_B.minnanb];
  }

  inverse_kinematics_B.loop_ub_k = obj_3->size[0];
  for (inverse_kinematics_B.minnanb = 0; inverse_kinematics_B.minnanb <
       inverse_kinematics_B.loop_ub_k; inverse_kinematics_B.minnanb++) {
    iobj_1[1].MotionSubspace->data[inverse_kinematics_B.minnanb] = obj_3->
      data[inverse_kinematics_B.minnanb];
  }

  inverse_kinemati_emxFree_real_T(&obj_3);
  for (inverse_kinematics_B.minnanb = 0; inverse_kinematics_B.minnanb < 16;
       inverse_kinematics_B.minnanb++) {
    inverse_kinematics_B.obj_b[inverse_kinematics_B.minnanb] =
      obj_0->JointToParentTransform[inverse_kinematics_B.minnanb];
  }

  for (inverse_kinematics_B.minnanb = 0; inverse_kinematics_B.minnanb < 16;
       inverse_kinematics_B.minnanb++) {
    iobj_1[1].JointToParentTransform[inverse_kinematics_B.minnanb] =
      inverse_kinematics_B.obj_b[inverse_kinematics_B.minnanb];
  }

  for (inverse_kinematics_B.minnanb = 0; inverse_kinematics_B.minnanb < 16;
       inverse_kinematics_B.minnanb++) {
    inverse_kinematics_B.obj_b[inverse_kinematics_B.minnanb] =
      obj_0->ChildToJointTransform[inverse_kinematics_B.minnanb];
  }

  for (inverse_kinematics_B.minnanb = 0; inverse_kinematics_B.minnanb < 16;
       inverse_kinematics_B.minnanb++) {
    iobj_1[1].ChildToJointTransform[inverse_kinematics_B.minnanb] =
      inverse_kinematics_B.obj_b[inverse_kinematics_B.minnanb];
  }

  iobj_2->JointInternal = &iobj_1[1];
  iobj_2->MassInternal = obj->MassInternal;
  inverse_kinematics_B.obj_idx_0 = obj->CenterOfMassInternal[0];
  inverse_kinematics_B.obj_idx_1 = obj->CenterOfMassInternal[1];
  inverse_kinematics_B.obj_idx_2 = obj->CenterOfMassInternal[2];
  iobj_2->CenterOfMassInternal[0] = inverse_kinematics_B.obj_idx_0;
  iobj_2->CenterOfMassInternal[1] = inverse_kinematics_B.obj_idx_1;
  iobj_2->CenterOfMassInternal[2] = inverse_kinematics_B.obj_idx_2;
  for (inverse_kinematics_B.minnanb = 0; inverse_kinematics_B.minnanb < 9;
       inverse_kinematics_B.minnanb++) {
    inverse_kinematics_B.obj_g[inverse_kinematics_B.minnanb] =
      obj->InertiaInternal[inverse_kinematics_B.minnanb];
  }

  for (inverse_kinematics_B.minnanb = 0; inverse_kinematics_B.minnanb < 9;
       inverse_kinematics_B.minnanb++) {
    iobj_2->InertiaInternal[inverse_kinematics_B.minnanb] =
      inverse_kinematics_B.obj_g[inverse_kinematics_B.minnanb];
  }

  for (inverse_kinematics_B.minnanb = 0; inverse_kinematics_B.minnanb < 36;
       inverse_kinematics_B.minnanb++) {
    inverse_kinematics_B.obj[inverse_kinematics_B.minnanb] = obj->
      SpatialInertia[inverse_kinematics_B.minnanb];
  }

  for (inverse_kinematics_B.minnanb = 0; inverse_kinematics_B.minnanb < 36;
       inverse_kinematics_B.minnanb++) {
    iobj_2->SpatialInertia[inverse_kinematics_B.minnanb] =
      inverse_kinematics_B.obj[inverse_kinematics_B.minnanb];
  }

  obj_1 = obj->CollisionsInternal;
  newObj = inver_CollisionSet_CollisionSet(&iobj_0[1], obj_1->MaxElements);
  newObj->Size = obj_1->Size;
  inverse_kinematics_B.obj_idx_0 = obj_1->Size;
  inverse_kinematics_B.nmatched = static_cast<int32_T>
    (inverse_kinematics_B.obj_idx_0) - 1;
  for (inverse_kinematics_B.minnanb = 0; inverse_kinematics_B.minnanb <=
       inverse_kinematics_B.nmatched; inverse_kinematics_B.minnanb++) {
    inverse_kinematics_B.loop_ub_k = inverse_kinematics_B.minnanb;
    obj_2 = obj_1->CollisionGeometries->data[inverse_kinematics_B.loop_ub_k];
    newObj->CollisionGeometries->data[inverse_kinematics_B.loop_ub_k] = obj_2;
  }

  iobj_2->CollisionsInternal = newObj;
  return newbody;
}

static void inverse_k_RigidBodyTree_addBody(v_robotics_manip_internal_Rig_T *obj,
  t_robotics_manip_internal_Rig_T *bodyin, const emxArray_char_T_inverse_kinem_T
  *parentName, c_rigidBodyJoint_inverse_kine_T *iobj_0,
  t_robotics_manip_internal_Rig_T *iobj_1, l_robotics_manip_internal_Col_T
  *iobj_2)
{
  c_rigidBodyJoint_inverse_kine_T *jnt;
  emxArray_char_T_inverse_kinem_T *bname;
  t_robotics_manip_internal_Rig_T *body;
  static const char_T tmp[5] = { 'f', 'i', 'x', 'e', 'd' };

  inverse_kinemati_emxInit_char_T(&bname, 2);
  inverse_kinematics_B.i1 = bname->size[0] * bname->size[1];
  bname->size[0] = 1;
  bname->size[1] = bodyin->NameInternal->size[1];
  invers_emxEnsureCapacity_char_T(bname, inverse_kinematics_B.i1);
  inverse_kinematics_B.loop_ub_c = bodyin->NameInternal->size[1] - 1;
  for (inverse_kinematics_B.b_kstr_h = 0; inverse_kinematics_B.b_kstr_h <=
       inverse_kinematics_B.loop_ub_c; inverse_kinematics_B.b_kstr_h++) {
    inverse_kinematics_B.i1 = inverse_kinematics_B.b_kstr_h;
    bname->data[inverse_kinematics_B.i1] = bodyin->NameInternal->
      data[inverse_kinematics_B.i1];
  }

  RigidBodyTree_findBodyIndexByNa(obj, bname);
  inverse_kinematics_B.pid = RigidBodyTree_findBodyIndexByNa(obj, parentName);
  inverse_kinematics_B.b_index = obj->NumBodies + 1.0;
  body = inverse_kinemati_RigidBody_copy(bodyin, &iobj_2[0], &iobj_0[0], iobj_1);
  obj->Bodies[static_cast<int32_T>(inverse_kinematics_B.b_index) - 1] = body;
  body->Index = inverse_kinematics_B.b_index;
  body->ParentIndex = inverse_kinematics_B.pid;
  body->JointInternal->InTree = true;
  obj->NumBodies++;
  jnt = body->JointInternal;
  inverse_kinematics_B.i1 = bname->size[0] * bname->size[1];
  bname->size[0] = 1;
  bname->size[1] = jnt->Type->size[1];
  invers_emxEnsureCapacity_char_T(bname, inverse_kinematics_B.i1);
  inverse_kinematics_B.loop_ub_c = jnt->Type->size[1] - 1;
  for (inverse_kinematics_B.b_kstr_h = 0; inverse_kinematics_B.b_kstr_h <=
       inverse_kinematics_B.loop_ub_c; inverse_kinematics_B.b_kstr_h++) {
    inverse_kinematics_B.i1 = inverse_kinematics_B.b_kstr_h;
    bname->data[inverse_kinematics_B.i1] = jnt->Type->
      data[inverse_kinematics_B.i1];
  }

  for (inverse_kinematics_B.b_kstr_h = 0; inverse_kinematics_B.b_kstr_h < 5;
       inverse_kinematics_B.b_kstr_h++) {
    inverse_kinematics_B.b_ax[inverse_kinematics_B.b_kstr_h] =
      tmp[inverse_kinematics_B.b_kstr_h];
  }

  inverse_kinematics_B.b_bool_a = false;
  if (bname->size[1] != 5) {
  } else {
    inverse_kinematics_B.b_kstr_h = 1;
    int32_T exitg1;
    do {
      exitg1 = 0;
      if (inverse_kinematics_B.b_kstr_h - 1 < 5) {
        if (bname->data[inverse_kinematics_B.b_kstr_h - 1] !=
            inverse_kinematics_B.b_ax[inverse_kinematics_B.b_kstr_h - 1]) {
          exitg1 = 1;
        } else {
          inverse_kinematics_B.b_kstr_h++;
        }
      } else {
        inverse_kinematics_B.b_bool_a = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  inverse_kinemati_emxFree_char_T(&bname);
  if (!inverse_kinematics_B.b_bool_a) {
    obj->NumNonFixedBodies++;
    jnt = body->JointInternal;
    inverse_kinematics_B.b_kstr_h = static_cast<int32_T>(body->Index) - 1;
    obj->PositionDoFMap[inverse_kinematics_B.b_kstr_h] = obj->PositionNumber +
      1.0;
    obj->PositionDoFMap[inverse_kinematics_B.b_kstr_h + 7] = obj->PositionNumber
      + jnt->PositionNumber;
    jnt = body->JointInternal;
    inverse_kinematics_B.b_kstr_h = static_cast<int32_T>(body->Index) - 1;
    obj->VelocityDoFMap[inverse_kinematics_B.b_kstr_h] = obj->VelocityNumber +
      1.0;
    obj->VelocityDoFMap[inverse_kinematics_B.b_kstr_h + 7] = obj->VelocityNumber
      + jnt->VelocityNumber;
  } else {
    inverse_kinematics_B.b_kstr_h = static_cast<int32_T>(body->Index);
    obj->PositionDoFMap[inverse_kinematics_B.b_kstr_h - 1] = 0.0;
    obj->PositionDoFMap[inverse_kinematics_B.b_kstr_h + 6] = -1.0;
    inverse_kinematics_B.b_kstr_h = static_cast<int32_T>(body->Index);
    obj->VelocityDoFMap[inverse_kinematics_B.b_kstr_h - 1] = 0.0;
    obj->VelocityDoFMap[inverse_kinematics_B.b_kstr_h + 6] = -1.0;
  }

  jnt = body->JointInternal;
  obj->PositionNumber += jnt->PositionNumber;
  jnt = body->JointInternal;
  obj->VelocityNumber += jnt->VelocityNumber;
}

static void inverseKinematics_set_RigidBody(b_inverseKinematics_inverse_k_T *obj,
  u_robotics_manip_internal_Rig_T *rigidbodytree,
  c_rigidBodyJoint_inverse_kine_T *iobj_0, t_robotics_manip_internal_Rig_T
  *iobj_1, l_robotics_manip_internal_Col_T *iobj_2,
  v_robotics_manip_internal_Rig_T *iobj_3)
{
  c_rigidBodyJoint_inverse_kine_T *iobj_1_0;
  emxArray_char_T_inverse_kinem_T *bname;
  emxArray_char_T_inverse_kinem_T *switch_expression;
  k_robotics_manip_internal_Col_T *obj_0;
  l_robotics_manip_internal_Col_T *iobj_0_0;
  l_robotics_manip_internal_Col_T *newObj;
  t_robotics_manip_internal_Rig_T *body;
  t_robotics_manip_internal_Rig_T *parent;
  static const int8_T tmp[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1
  };

  static const char_T tmp_0[8] = { 'b', 'a', 's', 'e', '_', 'j', 'n', 't' };

  static const char_T tmp_1[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_2[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_3[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  int32_T exitg1;
  body = &iobj_3->Base;
  iobj_0_0 = &iobj_3->_pobj1[0];
  inverse_kinematics_B.obj_tmp = body->NameInternal->size[0] *
    body->NameInternal->size[1];
  body->NameInternal->size[0] = 1;
  body->NameInternal->size[1] = 4;
  invers_emxEnsureCapacity_char_T(body->NameInternal,
    inverse_kinematics_B.obj_tmp);
  body->NameInternal->data[0] = 'b';
  body->NameInternal->data[1] = 'a';
  body->NameInternal->data[2] = 's';
  body->NameInternal->data[3] = 'e';
  iobj_3->_pobj2[0].InTree = false;
  for (inverse_kinematics_B.b_kstr = 0; inverse_kinematics_B.b_kstr < 16;
       inverse_kinematics_B.b_kstr++) {
    iobj_3->_pobj2[0].JointToParentTransform[inverse_kinematics_B.b_kstr] =
      tmp[inverse_kinematics_B.b_kstr];
  }

  for (inverse_kinematics_B.b_kstr = 0; inverse_kinematics_B.b_kstr < 16;
       inverse_kinematics_B.b_kstr++) {
    iobj_3->_pobj2[0].ChildToJointTransform[inverse_kinematics_B.b_kstr] =
      tmp[inverse_kinematics_B.b_kstr];
  }

  inverse_kinematics_B.obj_tmp = iobj_3->_pobj2[0].NameInternal->size[0] *
    iobj_3->_pobj2[0].NameInternal->size[1];
  iobj_3->_pobj2[0].NameInternal->size[0] = 1;
  iobj_3->_pobj2[0].NameInternal->size[1] = 8;
  invers_emxEnsureCapacity_char_T(iobj_3->_pobj2[0].NameInternal,
    inverse_kinematics_B.obj_tmp);
  for (inverse_kinematics_B.b_kstr = 0; inverse_kinematics_B.b_kstr < 8;
       inverse_kinematics_B.b_kstr++) {
    iobj_3->_pobj2[0].NameInternal->data[inverse_kinematics_B.b_kstr] =
      tmp_0[inverse_kinematics_B.b_kstr];
  }

  inverse_kinematics_B.obj_tmp = iobj_3->_pobj2[0].Type->size[0] *
    iobj_3->_pobj2[0].Type->size[1];
  iobj_3->_pobj2[0].Type->size[0] = 1;
  iobj_3->_pobj2[0].Type->size[1] = 5;
  invers_emxEnsureCapacity_char_T(iobj_3->_pobj2[0].Type,
    inverse_kinematics_B.obj_tmp);
  for (inverse_kinematics_B.b_kstr = 0; inverse_kinematics_B.b_kstr < 5;
       inverse_kinematics_B.b_kstr++) {
    iobj_3->_pobj2[0].Type->data[inverse_kinematics_B.b_kstr] =
      tmp_1[inverse_kinematics_B.b_kstr];
  }

  inverse_kinemati_emxInit_char_T(&switch_expression, 2);
  inverse_kinematics_B.obj_tmp = switch_expression->size[0] *
    switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = iobj_3->_pobj2[0].Type->size[1];
  invers_emxEnsureCapacity_char_T(switch_expression,
    inverse_kinematics_B.obj_tmp);
  inverse_kinematics_B.loop_ub_mc = iobj_3->_pobj2[0].Type->size[1] - 1;
  for (inverse_kinematics_B.b_kstr = 0; inverse_kinematics_B.b_kstr <=
       inverse_kinematics_B.loop_ub_mc; inverse_kinematics_B.b_kstr++) {
    inverse_kinematics_B.obj_tmp = inverse_kinematics_B.b_kstr;
    switch_expression->data[inverse_kinematics_B.obj_tmp] = iobj_3->_pobj2[0].
      Type->data[inverse_kinematics_B.obj_tmp];
  }

  for (inverse_kinematics_B.b_kstr = 0; inverse_kinematics_B.b_kstr < 8;
       inverse_kinematics_B.b_kstr++) {
    inverse_kinematics_B.b_a[inverse_kinematics_B.b_kstr] =
      tmp_2[inverse_kinematics_B.b_kstr];
  }

  inverse_kinematics_B.b_bool_m = false;
  if (switch_expression->size[1] != 8) {
  } else {
    inverse_kinematics_B.b_kstr = 1;
    do {
      exitg1 = 0;
      if (inverse_kinematics_B.b_kstr - 1 < 8) {
        if (switch_expression->data[inverse_kinematics_B.b_kstr - 1] !=
            inverse_kinematics_B.b_a[inverse_kinematics_B.b_kstr - 1]) {
          exitg1 = 1;
        } else {
          inverse_kinematics_B.b_kstr++;
        }
      } else {
        inverse_kinematics_B.b_bool_m = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (inverse_kinematics_B.b_bool_m) {
    inverse_kinematics_B.b_kstr = 0;
  } else {
    for (inverse_kinematics_B.b_kstr = 0; inverse_kinematics_B.b_kstr < 9;
         inverse_kinematics_B.b_kstr++) {
      inverse_kinematics_B.b_ln[inverse_kinematics_B.b_kstr] =
        tmp_3[inverse_kinematics_B.b_kstr];
    }

    inverse_kinematics_B.b_bool_m = false;
    if (switch_expression->size[1] != 9) {
    } else {
      inverse_kinematics_B.b_kstr = 1;
      do {
        exitg1 = 0;
        if (inverse_kinematics_B.b_kstr - 1 < 9) {
          if (switch_expression->data[inverse_kinematics_B.b_kstr - 1] !=
              inverse_kinematics_B.b_ln[inverse_kinematics_B.b_kstr - 1]) {
            exitg1 = 1;
          } else {
            inverse_kinematics_B.b_kstr++;
          }
        } else {
          inverse_kinematics_B.b_bool_m = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (inverse_kinematics_B.b_bool_m) {
      inverse_kinematics_B.b_kstr = 1;
    } else {
      inverse_kinematics_B.b_kstr = -1;
    }
  }

  switch (inverse_kinematics_B.b_kstr) {
   case 0:
    inverse_kinematics_B.iv[0] = 0;
    inverse_kinematics_B.iv[1] = 0;
    inverse_kinematics_B.iv[2] = 1;
    inverse_kinematics_B.iv[3] = 0;
    inverse_kinematics_B.iv[4] = 0;
    inverse_kinematics_B.iv[5] = 0;
    for (inverse_kinematics_B.b_kstr = 0; inverse_kinematics_B.b_kstr < 6;
         inverse_kinematics_B.b_kstr++) {
      inverse_kinematics_B.msubspace_data[inverse_kinematics_B.b_kstr] =
        inverse_kinematics_B.iv[inverse_kinematics_B.b_kstr];
    }

    inverse_kinematics_B.poslim_data[0] = -3.1415926535897931;
    inverse_kinematics_B.poslim_data[1] = 3.1415926535897931;
    iobj_3->_pobj2[0].VelocityNumber = 1.0;
    iobj_3->_pobj2[0].PositionNumber = 1.0;
    iobj_3->_pobj2[0].JointAxisInternal[0] = 0.0;
    iobj_3->_pobj2[0].JointAxisInternal[1] = 0.0;
    iobj_3->_pobj2[0].JointAxisInternal[2] = 1.0;
    break;

   case 1:
    inverse_kinematics_B.iv[0] = 0;
    inverse_kinematics_B.iv[1] = 0;
    inverse_kinematics_B.iv[2] = 0;
    inverse_kinematics_B.iv[3] = 0;
    inverse_kinematics_B.iv[4] = 0;
    inverse_kinematics_B.iv[5] = 1;
    for (inverse_kinematics_B.b_kstr = 0; inverse_kinematics_B.b_kstr < 6;
         inverse_kinematics_B.b_kstr++) {
      inverse_kinematics_B.msubspace_data[inverse_kinematics_B.b_kstr] =
        inverse_kinematics_B.iv[inverse_kinematics_B.b_kstr];
    }

    inverse_kinematics_B.poslim_data[0] = -0.5;
    inverse_kinematics_B.poslim_data[1] = 0.5;
    iobj_3->_pobj2[0].VelocityNumber = 1.0;
    iobj_3->_pobj2[0].PositionNumber = 1.0;
    iobj_3->_pobj2[0].JointAxisInternal[0] = 0.0;
    iobj_3->_pobj2[0].JointAxisInternal[1] = 0.0;
    iobj_3->_pobj2[0].JointAxisInternal[2] = 1.0;
    break;

   default:
    for (inverse_kinematics_B.b_kstr = 0; inverse_kinematics_B.b_kstr < 6;
         inverse_kinematics_B.b_kstr++) {
      inverse_kinematics_B.msubspace_data[inverse_kinematics_B.b_kstr] = 0;
    }

    inverse_kinematics_B.poslim_data[0] = 0.0;
    inverse_kinematics_B.poslim_data[1] = 0.0;
    iobj_3->_pobj2[0].VelocityNumber = 0.0;
    iobj_3->_pobj2[0].PositionNumber = 0.0;
    iobj_3->_pobj2[0].JointAxisInternal[0] = 0.0;
    iobj_3->_pobj2[0].JointAxisInternal[1] = 0.0;
    iobj_3->_pobj2[0].JointAxisInternal[2] = 0.0;
    break;
  }

  inverse_kinematics_B.obj_tmp = iobj_3->_pobj2[0].MotionSubspace->size[0] *
    iobj_3->_pobj2[0].MotionSubspace->size[1];
  iobj_3->_pobj2[0].MotionSubspace->size[0] = 6;
  iobj_3->_pobj2[0].MotionSubspace->size[1] = 1;
  invers_emxEnsureCapacity_real_T(iobj_3->_pobj2[0].MotionSubspace,
    inverse_kinematics_B.obj_tmp);
  for (inverse_kinematics_B.b_kstr = 0; inverse_kinematics_B.b_kstr < 6;
       inverse_kinematics_B.b_kstr++) {
    iobj_3->_pobj2[0].MotionSubspace->data[inverse_kinematics_B.b_kstr] =
      inverse_kinematics_B.msubspace_data[inverse_kinematics_B.b_kstr];
  }

  inverse_kinematics_B.obj_tmp = iobj_3->_pobj2[0].PositionLimitsInternal->size
    [0] * iobj_3->_pobj2[0].PositionLimitsInternal->size[1];
  iobj_3->_pobj2[0].PositionLimitsInternal->size[0] = 1;
  iobj_3->_pobj2[0].PositionLimitsInternal->size[1] = 2;
  invers_emxEnsureCapacity_real_T(iobj_3->_pobj2[0].PositionLimitsInternal,
    inverse_kinematics_B.obj_tmp);
  for (inverse_kinematics_B.b_kstr = 0; inverse_kinematics_B.b_kstr < 2;
       inverse_kinematics_B.b_kstr++) {
    iobj_3->_pobj2[0].PositionLimitsInternal->data[inverse_kinematics_B.b_kstr] =
      inverse_kinematics_B.poslim_data[inverse_kinematics_B.b_kstr];
  }

  inverse_kinematics_B.obj_tmp = iobj_3->_pobj2[0].HomePositionInternal->size[0];
  iobj_3->_pobj2[0].HomePositionInternal->size[0] = 1;
  invers_emxEnsureCapacity_real_T(iobj_3->_pobj2[0].HomePositionInternal,
    inverse_kinematics_B.obj_tmp);
  iobj_3->_pobj2[0].HomePositionInternal->data[0] = 0.0;
  body->JointInternal = &iobj_3->_pobj2[0];
  body->Index = -1.0;
  body->ParentIndex = -1.0;
  body->MassInternal = 1.0;
  body->CenterOfMassInternal[0] = 0.0;
  body->CenterOfMassInternal[1] = 0.0;
  body->CenterOfMassInternal[2] = 0.0;
  for (inverse_kinematics_B.b_kstr = 0; inverse_kinematics_B.b_kstr < 9;
       inverse_kinematics_B.b_kstr++) {
    inverse_kinematics_B.b_I_b[inverse_kinematics_B.b_kstr] = 0;
  }

  inverse_kinematics_B.b_I_b[0] = 1;
  inverse_kinematics_B.b_I_b[4] = 1;
  inverse_kinematics_B.b_I_b[8] = 1;
  for (inverse_kinematics_B.b_kstr = 0; inverse_kinematics_B.b_kstr < 9;
       inverse_kinematics_B.b_kstr++) {
    body->InertiaInternal[inverse_kinematics_B.b_kstr] =
      inverse_kinematics_B.b_I_b[inverse_kinematics_B.b_kstr];
  }

  for (inverse_kinematics_B.b_kstr = 0; inverse_kinematics_B.b_kstr < 36;
       inverse_kinematics_B.b_kstr++) {
    inverse_kinematics_B.msubspace_data[inverse_kinematics_B.b_kstr] = 0;
  }

  for (inverse_kinematics_B.b_kstr = 0; inverse_kinematics_B.b_kstr < 6;
       inverse_kinematics_B.b_kstr++) {
    inverse_kinematics_B.msubspace_data[inverse_kinematics_B.b_kstr + 6 *
      inverse_kinematics_B.b_kstr] = 1;
  }

  for (inverse_kinematics_B.b_kstr = 0; inverse_kinematics_B.b_kstr < 36;
       inverse_kinematics_B.b_kstr++) {
    body->SpatialInertia[inverse_kinematics_B.b_kstr] =
      inverse_kinematics_B.msubspace_data[inverse_kinematics_B.b_kstr];
  }

  body->CollisionsInternal = inver_CollisionSet_CollisionSet(iobj_0_0, 0.0);
  iobj_3->Base.Index = 0.0;
  inverse_kinematics_rand(inverse_kinematics_B.unusedExpr);
  iobj_0_0 = &iobj_3->_pobj1[1];
  iobj_1_0 = &iobj_3->_pobj2[1];
  body = &iobj_3->_pobj0[0];
  iobj_3->Bodies[0] = inverse_kin_RigidBody_RigidBody(&(&(&iobj_3->_pobj0[0])[0])
    [0], &(&iobj_0_0[0])[0], &(&iobj_1_0[0])[0]);
  iobj_3->Bodies[1] = inverse_k_RigidBody_RigidBody_g(&(&body[0])[1],
    &(&iobj_0_0[0])[1], &(&iobj_1_0[0])[1]);
  iobj_3->Bodies[2] = inverse__RigidBody_RigidBody_ga(&(&body[0])[2],
    &(&iobj_0_0[0])[2], &(&iobj_1_0[0])[2]);
  iobj_3->Bodies[3] = inverse_RigidBody_RigidBody_gaq(&(&body[0])[3],
    &(&iobj_0_0[0])[3], &(&iobj_1_0[0])[3]);
  iobj_3->Bodies[4] = invers_RigidBody_RigidBody_gaqy(&(&body[0])[4],
    &(&iobj_0_0[0])[4], &(&iobj_1_0[0])[4]);
  iobj_3->Bodies[5] = inver_RigidBody_RigidBody_gaqyy(&(&body[0])[5],
    &(&iobj_0_0[0])[5], &(&iobj_1_0[0])[5]);
  iobj_3->Bodies[6] = inve_RigidBody_RigidBody_gaqyy1(&(&body[0])[6],
    &(&iobj_0_0[0])[6], &(&iobj_1_0[0])[6]);
  iobj_3->NumBodies = 0.0;
  iobj_3->NumNonFixedBodies = 0.0;
  iobj_3->PositionNumber = 0.0;
  iobj_3->VelocityNumber = 0.0;
  inverse_kinematics_rand(inverse_kinematics_B.unusedExpr);
  for (inverse_kinematics_B.b_kstr = 0; inverse_kinematics_B.b_kstr < 7;
       inverse_kinematics_B.b_kstr++) {
    iobj_3->PositionDoFMap[inverse_kinematics_B.b_kstr] = 0.0;
  }

  for (inverse_kinematics_B.b_kstr = 0; inverse_kinematics_B.b_kstr < 7;
       inverse_kinematics_B.b_kstr++) {
    iobj_3->PositionDoFMap[inverse_kinematics_B.b_kstr + 7] = -1.0;
  }

  for (inverse_kinematics_B.b_kstr = 0; inverse_kinematics_B.b_kstr < 7;
       inverse_kinematics_B.b_kstr++) {
    iobj_3->VelocityDoFMap[inverse_kinematics_B.b_kstr] = 0.0;
  }

  for (inverse_kinematics_B.b_kstr = 0; inverse_kinematics_B.b_kstr < 7;
       inverse_kinematics_B.b_kstr++) {
    iobj_3->VelocityDoFMap[inverse_kinematics_B.b_kstr + 7] = -1.0;
  }

  inverse_kinematics_B.obj_tmp = switch_expression->size[0] *
    switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = rigidbodytree->Base.NameInternal->size[1];
  invers_emxEnsureCapacity_char_T(switch_expression,
    inverse_kinematics_B.obj_tmp);
  inverse_kinematics_B.loop_ub_mc = rigidbodytree->Base.NameInternal->size[1] -
    1;
  for (inverse_kinematics_B.b_kstr = 0; inverse_kinematics_B.b_kstr <=
       inverse_kinematics_B.loop_ub_mc; inverse_kinematics_B.b_kstr++) {
    inverse_kinematics_B.obj_tmp = inverse_kinematics_B.b_kstr;
    switch_expression->data[inverse_kinematics_B.obj_tmp] =
      rigidbodytree->Base.NameInternal->data[inverse_kinematics_B.obj_tmp];
  }

  inverse_kinemati_emxInit_char_T(&bname, 2);
  inverse_kinematics_B.bid_m = -1.0;
  inverse_kinematics_B.obj_tmp = bname->size[0] * bname->size[1];
  bname->size[0] = 1;
  bname->size[1] = iobj_3->Base.NameInternal->size[1];
  invers_emxEnsureCapacity_char_T(bname, inverse_kinematics_B.obj_tmp);
  inverse_kinematics_B.loop_ub_mc = iobj_3->Base.NameInternal->size[1] - 1;
  for (inverse_kinematics_B.b_kstr = 0; inverse_kinematics_B.b_kstr <=
       inverse_kinematics_B.loop_ub_mc; inverse_kinematics_B.b_kstr++) {
    inverse_kinematics_B.obj_tmp = inverse_kinematics_B.b_kstr;
    bname->data[inverse_kinematics_B.obj_tmp] = iobj_3->Base.NameInternal->
      data[inverse_kinematics_B.obj_tmp];
  }

  if (inverse_kinematics_strcmp(bname, switch_expression)) {
    inverse_kinematics_B.bid_m = 0.0;
  } else {
    boolean_T exitg2;
    inverse_kinematics_B.b_ja = iobj_3->NumBodies;
    inverse_kinematics_B.iobj_3 = 0;
    exitg2 = false;
    while ((!exitg2) && (inverse_kinematics_B.iobj_3 <= static_cast<int32_T>
                         (inverse_kinematics_B.b_ja) - 1)) {
      body = iobj_3->Bodies[inverse_kinematics_B.iobj_3];
      inverse_kinematics_B.obj_tmp = bname->size[0] * bname->size[1];
      bname->size[0] = 1;
      bname->size[1] = body->NameInternal->size[1];
      invers_emxEnsureCapacity_char_T(bname, inverse_kinematics_B.obj_tmp);
      inverse_kinematics_B.loop_ub_mc = body->NameInternal->size[1] - 1;
      for (inverse_kinematics_B.b_kstr = 0; inverse_kinematics_B.b_kstr <=
           inverse_kinematics_B.loop_ub_mc; inverse_kinematics_B.b_kstr++) {
        inverse_kinematics_B.obj_tmp = inverse_kinematics_B.b_kstr;
        bname->data[inverse_kinematics_B.obj_tmp] = body->NameInternal->
          data[inverse_kinematics_B.obj_tmp];
      }

      if (inverse_kinematics_strcmp(bname, switch_expression)) {
        inverse_kinematics_B.bid_m = static_cast<real_T>
          (inverse_kinematics_B.iobj_3) + 1.0;
        exitg2 = true;
      } else {
        inverse_kinematics_B.iobj_3++;
      }
    }
  }

  if ((!(inverse_kinematics_B.bid_m == 0.0)) && (inverse_kinematics_B.bid_m <
       0.0)) {
    inverse_kinematics_B.obj_tmp = iobj_3->Base.NameInternal->size[0] *
      iobj_3->Base.NameInternal->size[1];
    iobj_3->Base.NameInternal->size[0] = 1;
    iobj_3->Base.NameInternal->size[1] = switch_expression->size[1];
    invers_emxEnsureCapacity_char_T(iobj_3->Base.NameInternal,
      inverse_kinematics_B.obj_tmp);
    inverse_kinematics_B.loop_ub_mc = switch_expression->size[1] - 1;
    for (inverse_kinematics_B.b_kstr = 0; inverse_kinematics_B.b_kstr <=
         inverse_kinematics_B.loop_ub_mc; inverse_kinematics_B.b_kstr++) {
      inverse_kinematics_B.obj_tmp = inverse_kinematics_B.b_kstr;
      iobj_3->Base.NameInternal->data[inverse_kinematics_B.obj_tmp] =
        switch_expression->data[inverse_kinematics_B.obj_tmp];
    }
  }

  inverse_kinemati_emxFree_char_T(&switch_expression);
  iobj_0_0 = rigidbodytree->Base.CollisionsInternal;
  newObj = inver_CollisionSet_CollisionSet(&(&iobj_2[0])[0],
    iobj_0_0->MaxElements);
  newObj->Size = iobj_0_0->Size;
  inverse_kinematics_B.b_ja = iobj_0_0->Size;
  inverse_kinematics_B.b_kstr = static_cast<int32_T>(inverse_kinematics_B.b_ja)
    - 1;
  for (inverse_kinematics_B.iobj_3 = 0; inverse_kinematics_B.iobj_3 <=
       inverse_kinematics_B.b_kstr; inverse_kinematics_B.iobj_3++) {
    inverse_kinematics_B.obj_tmp = inverse_kinematics_B.iobj_3;
    obj_0 = iobj_0_0->CollisionGeometries->data[inverse_kinematics_B.obj_tmp];
    newObj->CollisionGeometries->data[inverse_kinematics_B.obj_tmp] = obj_0;
  }

  iobj_3->Base.CollisionsInternal = newObj;
  if (rigidbodytree->NumBodies >= 1.0) {
    body = rigidbodytree->Bodies[0];
    inverse_kinematics_B.bid_m = body->ParentIndex;
    if (inverse_kinematics_B.bid_m > 0.0) {
      parent = rigidbodytree->Bodies[static_cast<int32_T>
        (inverse_kinematics_B.bid_m) - 1];
    } else {
      parent = &rigidbodytree->Base;
    }

    inverse_kinematics_B.obj_tmp = bname->size[0] * bname->size[1];
    bname->size[0] = 1;
    bname->size[1] = parent->NameInternal->size[1];
    invers_emxEnsureCapacity_char_T(bname, inverse_kinematics_B.obj_tmp);
    inverse_kinematics_B.loop_ub_mc = parent->NameInternal->size[1] - 1;
    for (inverse_kinematics_B.b_kstr = 0; inverse_kinematics_B.b_kstr <=
         inverse_kinematics_B.loop_ub_mc; inverse_kinematics_B.b_kstr++) {
      inverse_kinematics_B.obj_tmp = inverse_kinematics_B.b_kstr;
      bname->data[inverse_kinematics_B.obj_tmp] = parent->NameInternal->
        data[inverse_kinematics_B.obj_tmp];
    }

    inverse_k_RigidBodyTree_addBody(iobj_3, body, bname, &(&iobj_0[0])[0],
      &(&iobj_1[0])[0], &(&iobj_2[0])[1]);
  }

  if (rigidbodytree->NumBodies >= 2.0) {
    body = rigidbodytree->Bodies[1];
    inverse_kinematics_B.bid_m = body->ParentIndex;
    if (inverse_kinematics_B.bid_m > 0.0) {
      parent = rigidbodytree->Bodies[static_cast<int32_T>
        (inverse_kinematics_B.bid_m) - 1];
    } else {
      parent = &rigidbodytree->Base;
    }

    inverse_kinematics_B.obj_tmp = bname->size[0] * bname->size[1];
    bname->size[0] = 1;
    bname->size[1] = parent->NameInternal->size[1];
    invers_emxEnsureCapacity_char_T(bname, inverse_kinematics_B.obj_tmp);
    inverse_kinematics_B.loop_ub_mc = parent->NameInternal->size[1] - 1;
    for (inverse_kinematics_B.b_kstr = 0; inverse_kinematics_B.b_kstr <=
         inverse_kinematics_B.loop_ub_mc; inverse_kinematics_B.b_kstr++) {
      inverse_kinematics_B.obj_tmp = inverse_kinematics_B.b_kstr;
      bname->data[inverse_kinematics_B.obj_tmp] = parent->NameInternal->
        data[inverse_kinematics_B.obj_tmp];
    }

    inverse_k_RigidBodyTree_addBody(iobj_3, body, bname, &(&iobj_0[0])[2],
      &(&iobj_1[0])[1], &(&iobj_2[0])[3]);
  }

  if (rigidbodytree->NumBodies >= 3.0) {
    body = rigidbodytree->Bodies[2];
    inverse_kinematics_B.bid_m = body->ParentIndex;
    if (inverse_kinematics_B.bid_m > 0.0) {
      parent = rigidbodytree->Bodies[static_cast<int32_T>
        (inverse_kinematics_B.bid_m) - 1];
    } else {
      parent = &rigidbodytree->Base;
    }

    inverse_kinematics_B.obj_tmp = bname->size[0] * bname->size[1];
    bname->size[0] = 1;
    bname->size[1] = parent->NameInternal->size[1];
    invers_emxEnsureCapacity_char_T(bname, inverse_kinematics_B.obj_tmp);
    inverse_kinematics_B.loop_ub_mc = parent->NameInternal->size[1] - 1;
    for (inverse_kinematics_B.b_kstr = 0; inverse_kinematics_B.b_kstr <=
         inverse_kinematics_B.loop_ub_mc; inverse_kinematics_B.b_kstr++) {
      inverse_kinematics_B.obj_tmp = inverse_kinematics_B.b_kstr;
      bname->data[inverse_kinematics_B.obj_tmp] = parent->NameInternal->
        data[inverse_kinematics_B.obj_tmp];
    }

    inverse_k_RigidBodyTree_addBody(iobj_3, body, bname, &(&iobj_0[0])[4],
      &(&iobj_1[0])[2], &(&iobj_2[0])[5]);
  }

  if (rigidbodytree->NumBodies >= 4.0) {
    body = rigidbodytree->Bodies[3];
    inverse_kinematics_B.bid_m = body->ParentIndex;
    if (inverse_kinematics_B.bid_m > 0.0) {
      parent = rigidbodytree->Bodies[static_cast<int32_T>
        (inverse_kinematics_B.bid_m) - 1];
    } else {
      parent = &rigidbodytree->Base;
    }

    inverse_kinematics_B.obj_tmp = bname->size[0] * bname->size[1];
    bname->size[0] = 1;
    bname->size[1] = parent->NameInternal->size[1];
    invers_emxEnsureCapacity_char_T(bname, inverse_kinematics_B.obj_tmp);
    inverse_kinematics_B.loop_ub_mc = parent->NameInternal->size[1] - 1;
    for (inverse_kinematics_B.b_kstr = 0; inverse_kinematics_B.b_kstr <=
         inverse_kinematics_B.loop_ub_mc; inverse_kinematics_B.b_kstr++) {
      inverse_kinematics_B.obj_tmp = inverse_kinematics_B.b_kstr;
      bname->data[inverse_kinematics_B.obj_tmp] = parent->NameInternal->
        data[inverse_kinematics_B.obj_tmp];
    }

    inverse_k_RigidBodyTree_addBody(iobj_3, body, bname, &(&iobj_0[0])[6],
      &(&iobj_1[0])[3], &(&iobj_2[0])[7]);
  }

  if (rigidbodytree->NumBodies >= 5.0) {
    body = rigidbodytree->Bodies[4];
    inverse_kinematics_B.bid_m = body->ParentIndex;
    if (inverse_kinematics_B.bid_m > 0.0) {
      parent = rigidbodytree->Bodies[static_cast<int32_T>
        (inverse_kinematics_B.bid_m) - 1];
    } else {
      parent = &rigidbodytree->Base;
    }

    inverse_kinematics_B.obj_tmp = bname->size[0] * bname->size[1];
    bname->size[0] = 1;
    bname->size[1] = parent->NameInternal->size[1];
    invers_emxEnsureCapacity_char_T(bname, inverse_kinematics_B.obj_tmp);
    inverse_kinematics_B.loop_ub_mc = parent->NameInternal->size[1] - 1;
    for (inverse_kinematics_B.b_kstr = 0; inverse_kinematics_B.b_kstr <=
         inverse_kinematics_B.loop_ub_mc; inverse_kinematics_B.b_kstr++) {
      inverse_kinematics_B.obj_tmp = inverse_kinematics_B.b_kstr;
      bname->data[inverse_kinematics_B.obj_tmp] = parent->NameInternal->
        data[inverse_kinematics_B.obj_tmp];
    }

    inverse_k_RigidBodyTree_addBody(iobj_3, body, bname, &(&iobj_0[0])[8],
      &(&iobj_1[0])[4], &(&iobj_2[0])[9]);
  }

  if (rigidbodytree->NumBodies >= 6.0) {
    body = rigidbodytree->Bodies[5];
    inverse_kinematics_B.bid_m = body->ParentIndex;
    if (inverse_kinematics_B.bid_m > 0.0) {
      parent = rigidbodytree->Bodies[static_cast<int32_T>
        (inverse_kinematics_B.bid_m) - 1];
    } else {
      parent = &rigidbodytree->Base;
    }

    inverse_kinematics_B.obj_tmp = bname->size[0] * bname->size[1];
    bname->size[0] = 1;
    bname->size[1] = parent->NameInternal->size[1];
    invers_emxEnsureCapacity_char_T(bname, inverse_kinematics_B.obj_tmp);
    inverse_kinematics_B.loop_ub_mc = parent->NameInternal->size[1] - 1;
    for (inverse_kinematics_B.b_kstr = 0; inverse_kinematics_B.b_kstr <=
         inverse_kinematics_B.loop_ub_mc; inverse_kinematics_B.b_kstr++) {
      inverse_kinematics_B.obj_tmp = inverse_kinematics_B.b_kstr;
      bname->data[inverse_kinematics_B.obj_tmp] = parent->NameInternal->
        data[inverse_kinematics_B.obj_tmp];
    }

    inverse_k_RigidBodyTree_addBody(iobj_3, body, bname, &(&iobj_0[0])[10],
      &(&iobj_1[0])[5], &(&iobj_2[0])[11]);
  }

  if (rigidbodytree->NumBodies >= 7.0) {
    body = rigidbodytree->Bodies[6];
    inverse_kinematics_B.bid_m = body->ParentIndex;
    if (inverse_kinematics_B.bid_m > 0.0) {
      parent = rigidbodytree->Bodies[static_cast<int32_T>
        (inverse_kinematics_B.bid_m) - 1];
    } else {
      parent = &rigidbodytree->Base;
    }

    inverse_kinematics_B.obj_tmp = bname->size[0] * bname->size[1];
    bname->size[0] = 1;
    bname->size[1] = parent->NameInternal->size[1];
    invers_emxEnsureCapacity_char_T(bname, inverse_kinematics_B.obj_tmp);
    inverse_kinematics_B.loop_ub_mc = parent->NameInternal->size[1] - 1;
    for (inverse_kinematics_B.b_kstr = 0; inverse_kinematics_B.b_kstr <=
         inverse_kinematics_B.loop_ub_mc; inverse_kinematics_B.b_kstr++) {
      inverse_kinematics_B.obj_tmp = inverse_kinematics_B.b_kstr;
      bname->data[inverse_kinematics_B.obj_tmp] = parent->NameInternal->
        data[inverse_kinematics_B.obj_tmp];
    }

    inverse_k_RigidBodyTree_addBody(iobj_3, body, bname, &(&iobj_0[0])[12],
      &(&iobj_1[0])[6], &(&iobj_2[0])[13]);
  }

  inverse_kinemati_emxFree_char_T(&bname);
  obj->RigidBodyTreeInternal = iobj_3;
}

static void inverse_kinema_SystemCore_setup(robotics_slmanip_internal_blo_T *obj)
{
  b_inverseKinematics_inverse_k_T *obj_0;
  c_rigidBodyJoint_inverse_kine_T *iobj_0;
  h_robotics_core_internal_Erro_T *iobj_4;
  l_robotics_manip_internal_Col_T *iobj_2;
  t_robotics_manip_internal_Rig_T *iobj_1;
  v_robotics_manip_internal_Rig_T *iobj_3;
  static const sdAmwXbnJnEmimT0NaJRtAD_inver_T tmp = { 0.0,/* tv_sec */
    0.0                                /* tv_nsec */
  };

  static const char_T tmp_0[18] = { 'L', 'e', 'v', 'e', 'n', 'b', 'e', 'r', 'g',
    'M', 'a', 'r', 'q', 'u', 'a', 'r', 'd', 't' };

  obj->isInitialized = 1;
  inv_RigidBodyTree_RigidBodyTree(&obj->TreeInternal);
  obj_0 = &obj->IKInternal;
  obj->IKInternal.isInitialized = 0;
  iobj_0 = &obj->IKInternal._pobj1[0];
  iobj_1 = &obj->IKInternal._pobj2[0];
  iobj_2 = &obj->IKInternal._pobj3[0];
  iobj_3 = &obj->IKInternal._pobj4;
  iobj_4 = &obj->IKInternal._pobj5;
  inverseKinematics_set_RigidBody(&obj->IKInternal, &obj->TreeInternal,
    &(&(&iobj_0[0])[0])[0], &(&(&iobj_1[0])[0])[0], &(&(&iobj_2[0])[0])[0],
    iobj_3);
  iobj_4->MaxNumIteration = 1500.0;
  iobj_4->MaxTime = 10.0;
  iobj_4->SolutionTolerance = 1.0E-6;
  iobj_4->ConstraintsOn = true;
  iobj_4->RandomRestart = true;
  iobj_4->StepTolerance = 1.0E-12;
  iobj_4->GradientTolerance = 5.0E-9;
  iobj_4->ErrorChangeTolerance = 1.0E-12;
  iobj_4->DampingBias = 0.0025;
  iobj_4->UseErrorDamping = true;
  for (inverse_kinematics_B.ret = 0; inverse_kinematics_B.ret < 18;
       inverse_kinematics_B.ret++) {
    iobj_4->Name[inverse_kinematics_B.ret] = tmp_0[inverse_kinematics_B.ret];
  }

  iobj_4->TimeObj.StartTime = tmp;
  iobj_4->TimeObjInternal.StartTime = tmp;
  obj_0->Solver = iobj_4;
  iobj_4 = obj_0->Solver;
  inverse_kinematics_B.params_ErrorChangeTolerance =
    iobj_4->ErrorChangeTolerance;
  inverse_kinematics_B.params_DampingBias = iobj_4->DampingBias;
  inverse_kinematics_B.params_UseErrorDamping = iobj_4->UseErrorDamping;
  for (inverse_kinematics_B.ret = 0; inverse_kinematics_B.ret < 18;
       inverse_kinematics_B.ret++) {
    inverse_kinematics_B.switch_expression[inverse_kinematics_B.ret] =
      obj_0->Solver->Name[inverse_kinematics_B.ret];
  }

  for (inverse_kinematics_B.ret = 0; inverse_kinematics_B.ret < 18;
       inverse_kinematics_B.ret++) {
    inverse_kinematics_B.b_l[inverse_kinematics_B.ret] =
      tmp_0[inverse_kinematics_B.ret];
  }

  inverse_kinematics_B.ret = memcmp(&inverse_kinematics_B.switch_expression[0],
    &inverse_kinematics_B.b_l[0], 18);
  if (inverse_kinematics_B.ret == 0) {
    inverse_kinematics_B.params_ErrorChangeTolerance = 1.0E-12;
    inverse_kinematics_B.params_DampingBias = 0.0025;
    inverse_kinematics_B.params_UseErrorDamping = true;
  }

  iobj_4 = obj_0->Solver;
  iobj_4->MaxNumIteration = 1500.0;
  iobj_4->MaxTime = 10.0;
  iobj_4->GradientTolerance = 1.0E-7;
  iobj_4->SolutionTolerance = 1.0E-6;
  iobj_4->ConstraintsOn = true;
  iobj_4->RandomRestart = false;
  iobj_4->StepTolerance = 1.0E-14;
  iobj_4->ErrorChangeTolerance =
    inverse_kinematics_B.params_ErrorChangeTolerance;
  iobj_4->DampingBias = inverse_kinematics_B.params_DampingBias;
  iobj_4->UseErrorDamping = inverse_kinematics_B.params_UseErrorDamping;
  obj_0->matlabCodegenIsDeleted = false;
}

static void RigidBodyTree_get_JointPosition(v_robotics_manip_internal_Rig_T *obj,
  emxArray_real_T_inverse_kinem_T *limits)
{
  c_rigidBodyJoint_inverse_kine_T *obj_0;
  emxArray_char_T_inverse_kinem_T *a;
  t_robotics_manip_internal_Rig_T *body;
  real_T k;
  real_T pnum;
  int32_T b_kstr;
  int32_T loop_ub;
  char_T b[5];
  static const char_T tmp[5] = { 'f', 'i', 'x', 'e', 'd' };

  int32_T c_tmp;
  int32_T i;
  i = limits->size[0] * limits->size[1];
  limits->size[0] = static_cast<int32_T>(obj->PositionNumber);
  limits->size[1] = 2;
  invers_emxEnsureCapacity_real_T(limits, i);
  loop_ub = (static_cast<int32_T>(obj->PositionNumber) << 1) - 1;
  if (loop_ub >= 0) {
    memset(&limits->data[0], 0, (loop_ub + 1) * sizeof(real_T));
  }

  k = 1.0;
  pnum = obj->NumBodies;
  c_tmp = static_cast<int32_T>(pnum) - 1;
  if (static_cast<int32_T>(pnum) - 1 >= 0) {
    for (b_kstr = 0; b_kstr < 5; b_kstr++) {
      b[b_kstr] = tmp[b_kstr];
    }
  }

  inverse_kinemati_emxInit_char_T(&a, 2);
  for (int32_T limits_0 = 0; limits_0 <= c_tmp; limits_0++) {
    boolean_T b_bool;
    body = obj->Bodies[limits_0];
    i = a->size[0] * a->size[1];
    a->size[0] = 1;
    a->size[1] = body->JointInternal->Type->size[1];
    invers_emxEnsureCapacity_char_T(a, i);
    loop_ub = body->JointInternal->Type->size[1] - 1;
    for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
      a->data[b_kstr] = body->JointInternal->Type->data[b_kstr];
    }

    b_bool = false;
    if (a->size[1] != 5) {
    } else {
      b_kstr = 1;
      int32_T exitg1;
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
      int32_T f;
      pnum = body->JointInternal->PositionNumber;
      pnum += k;
      if (k > pnum - 1.0) {
        f = 0;
      } else {
        f = static_cast<int32_T>(k) - 1;
      }

      obj_0 = body->JointInternal;
      loop_ub = obj_0->PositionLimitsInternal->size[0];
      for (b_kstr = 0; b_kstr < 2; b_kstr++) {
        for (i = 0; i < loop_ub; i++) {
          limits->data[(f + i) + limits->size[0] * b_kstr] =
            obj_0->PositionLimitsInternal->data[obj_0->
            PositionLimitsInternal->size[0] * b_kstr + i];
        }
      }

      k = pnum;
    }
  }

  inverse_kinemati_emxFree_char_T(&a);
}

static void inverse_kinemati_emxInit_int8_T(emxArray_int8_T_inverse_kinem_T
  **pEmxArray, int32_T numDimensions)
{
  emxArray_int8_T_inverse_kinem_T *emxArray;
  *pEmxArray = static_cast<emxArray_int8_T_inverse_kinem_T *>(malloc(sizeof
    (emxArray_int8_T_inverse_kinem_T)));
  emxArray = *pEmxArray;
  emxArray->data = static_cast<int8_T *>(NULL);
  emxArray->numDimensions = numDimensions;
  emxArray->size = static_cast<int32_T *>(malloc(sizeof(int32_T) * numDimensions));
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (int32_T i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

static void invers_emxEnsureCapacity_int8_T(emxArray_int8_T_inverse_kinem_T
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

    newData = calloc(static_cast<uint32_T>(i), sizeof(int8_T));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(int8_T) * oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }

    emxArray->data = static_cast<int8_T *>(newData);
    emxArray->allocatedSize = i;
    emxArray->canFreeData = true;
  }
}

static void inverse_kinemati_emxFree_int8_T(emxArray_int8_T_inverse_kinem_T
  **pEmxArray)
{
  if (*pEmxArray != static_cast<emxArray_int8_T_inverse_kinem_T *>(NULL)) {
    if (((*pEmxArray)->data != static_cast<int8_T *>(NULL)) && (*pEmxArray)
        ->canFreeData) {
      free((*pEmxArray)->data);
    }

    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = static_cast<emxArray_int8_T_inverse_kinem_T *>(NULL);
  }
}

static void inverse_ki_binary_expand_op_gaq(boolean_T in1[4], const real_T in2[4],
  const emxArray_real_T_inverse_kinem_T *in3)
{
  int32_T stride_0_0;
  stride_0_0 = (in3->size[0] != 1);
  in1[0] = (in2[0] <= in3->data[in3->size[0]] + 4.4408920985006262E-16);
  in1[1] = (in2[1] <= in3->data[stride_0_0 + in3->size[0]] +
            4.4408920985006262E-16);
  in1[2] = (in2[2] <= in3->data[(stride_0_0 << 1) + in3->size[0]] +
            4.4408920985006262E-16);
  in1[3] = (in2[3] <= in3->data[3 * stride_0_0 + in3->size[0]] +
            4.4408920985006262E-16);
}

static void inverse_kin_binary_expand_op_ga(boolean_T in1[4], const real_T in2[4],
  const emxArray_real_T_inverse_kinem_T *in3)
{
  int32_T stride_0_0;
  stride_0_0 = (in3->size[0] != 1);
  in1[0] = (in2[0] >= in3->data[0] - 4.4408920985006262E-16);
  in1[1] = (in2[1] >= in3->data[stride_0_0] - 4.4408920985006262E-16);
  in1[2] = (in2[2] >= in3->data[stride_0_0 << 1] - 4.4408920985006262E-16);
  in1[3] = (in2[3] >= in3->data[3 * stride_0_0] - 4.4408920985006262E-16);
}

static void inverse_kinematics_eml_find(const boolean_T x[4], int32_T i_data[],
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

  if (idx < 1) {
    idx = 0;
  }

  *i_size = idx;
}

static void inverse_kinematics_tic(real_T *tstart_tv_sec, real_T *tstart_tv_nsec)
{
  coderTimespec b_timespec;
  if (!inverse_kinematics_DW.method_not_empty) {
    inverse_kinematics_DW.method_not_empty = true;
    coderInitTimeFunctions(&inverse_kinematics_DW.freq);
  }

  coderTimeClockGettimeMonotonic(&b_timespec, inverse_kinematics_DW.freq);
  *tstart_tv_sec = b_timespec.tv_sec;
  *tstart_tv_nsec = b_timespec.tv_nsec;
}

static void i_RigidBodyTree_ancestorIndices(v_robotics_manip_internal_Rig_T *obj,
  t_robotics_manip_internal_Rig_T *body, emxArray_real_T_inverse_kinem_T
  *indices)
{
  inverse_kinematics_B.loop_ub_e = indices->size[0] * indices->size[1];
  indices->size[0] = 1;
  indices->size[1] = static_cast<int32_T>(obj->NumBodies + 1.0);
  invers_emxEnsureCapacity_real_T(indices, inverse_kinematics_B.loop_ub_e);
  inverse_kinematics_B.loop_ub_e = static_cast<int32_T>(obj->NumBodies + 1.0) -
    1;
  if (inverse_kinematics_B.loop_ub_e >= 0) {
    memset(&indices->data[0], 0, (inverse_kinematics_B.loop_ub_e + 1) * sizeof
           (real_T));
  }

  inverse_kinematics_B.i = 2.0;
  indices->data[0] = body->Index;
  while (body->ParentIndex > 0.0) {
    body = obj->Bodies[static_cast<int32_T>(body->ParentIndex) - 1];
    indices->data[static_cast<int32_T>(inverse_kinematics_B.i) - 1] =
      body->Index;
    inverse_kinematics_B.i++;
  }

  if (body->Index > 0.0) {
    indices->data[static_cast<int32_T>(inverse_kinematics_B.i) - 1] =
      body->ParentIndex;
    inverse_kinematics_B.i++;
  }

  inverse_kinematics_B.loop_ub_tmp = static_cast<int32_T>(inverse_kinematics_B.i
    - 1.0);
  for (inverse_kinematics_B.loop_ub_e = 0; inverse_kinematics_B.loop_ub_e <
       inverse_kinematics_B.loop_ub_tmp; inverse_kinematics_B.loop_ub_e++) {
  }

  inverse_kinematics_B.loop_ub_e = indices->size[0] * indices->size[1];
  indices->size[0] = 1;
  indices->size[1] = static_cast<int32_T>(inverse_kinematics_B.i - 1.0);
  invers_emxEnsureCapacity_real_T(indices, inverse_kinematics_B.loop_ub_e);
}

static void RigidBodyTree_kinematicPathInte(v_robotics_manip_internal_Rig_T *obj,
  t_robotics_manip_internal_Rig_T *body1, t_robotics_manip_internal_Rig_T *body2,
  emxArray_real_T_inverse_kinem_T *indices)
{
  emxArray_real_T_inverse_kinem_T *ancestorIndices1;
  emxArray_real_T_inverse_kinem_T *ancestorIndices2;
  boolean_T exitg1;
  inverse_kinemati_emxInit_real_T(&ancestorIndices1, 2);
  inverse_kinemati_emxInit_real_T(&ancestorIndices2, 2);
  i_RigidBodyTree_ancestorIndices(obj, body1, ancestorIndices1);
  i_RigidBodyTree_ancestorIndices(obj, body2, ancestorIndices2);
  if (static_cast<real_T>(ancestorIndices1->size[1]) <= ancestorIndices2->size[1])
  {
    inverse_kinematics_B.minPathLength = ancestorIndices1->size[1];
  } else {
    inverse_kinematics_B.minPathLength = ancestorIndices2->size[1];
  }

  inverse_kinematics_B.b_i_m = 0;
  exitg1 = false;
  while ((!exitg1) && (inverse_kinematics_B.b_i_m <=
                       inverse_kinematics_B.minPathLength - 2)) {
    if (ancestorIndices1->data[(ancestorIndices1->size[1] -
         inverse_kinematics_B.b_i_m) - 2] != ancestorIndices2->data
        [(ancestorIndices2->size[1] - inverse_kinematics_B.b_i_m) - 2]) {
      inverse_kinematics_B.minPathLength = inverse_kinematics_B.b_i_m + 1;
      exitg1 = true;
    } else {
      inverse_kinematics_B.b_i_m++;
    }
  }

  inverse_kinematics_B.b_i_m = ancestorIndices1->size[1] -
    inverse_kinematics_B.minPathLength;
  if (inverse_kinematics_B.b_i_m < 1) {
    inverse_kinematics_B.e_c = -1;
  } else {
    inverse_kinematics_B.e_c = inverse_kinematics_B.b_i_m - 1;
  }

  inverse_kinematics_B.b_i_m = ancestorIndices2->size[1] -
    inverse_kinematics_B.minPathLength;
  if (inverse_kinematics_B.b_i_m < 1) {
    inverse_kinematics_B.j = 0;
    inverse_kinematics_B.h = 1;
    inverse_kinematics_B.b_i_m = -1;
  } else {
    inverse_kinematics_B.j = inverse_kinematics_B.b_i_m - 1;
    inverse_kinematics_B.h = -1;
    inverse_kinematics_B.b_i_m = 0;
  }

  inverse_kinematics_B.i_p = indices->size[0] * indices->size[1];
  indices->size[0] = 1;
  inverse_kinematics_B.loop_ub_fm = div_s32(inverse_kinematics_B.b_i_m -
    inverse_kinematics_B.j, inverse_kinematics_B.h);
  indices->size[1] = (inverse_kinematics_B.loop_ub_fm + inverse_kinematics_B.e_c)
    + 3;
  invers_emxEnsureCapacity_real_T(indices, inverse_kinematics_B.i_p);
  if (inverse_kinematics_B.e_c >= 0) {
    memcpy(&indices->data[0], &ancestorIndices1->data[0],
           (inverse_kinematics_B.e_c + 1) * sizeof(real_T));
  }

  indices->data[inverse_kinematics_B.e_c + 1] = ancestorIndices1->
    data[ancestorIndices1->size[1] - inverse_kinematics_B.minPathLength];
  inverse_kinemati_emxFree_real_T(&ancestorIndices1);
  for (inverse_kinematics_B.b_i_m = 0; inverse_kinematics_B.b_i_m <=
       inverse_kinematics_B.loop_ub_fm; inverse_kinematics_B.b_i_m++) {
    indices->data[(inverse_kinematics_B.b_i_m + inverse_kinematics_B.e_c) + 2] =
      ancestorIndices2->data[inverse_kinematics_B.h * inverse_kinematics_B.b_i_m
      + inverse_kinematics_B.j];
  }

  inverse_kinemati_emxFree_real_T(&ancestorIndices2);
}

static void in_rigidBodyJoint_get_JointAxis(const
  c_rigidBodyJoint_inverse_kine_T *obj, real_T ax[3])
{
  int32_T b_kstr;
  char_T b_0[9];
  char_T b[8];
  boolean_T b_bool;
  static const char_T tmp[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_0[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  int32_T exitg1;
  boolean_T guard1 = false;
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp[b_kstr];
  }

  b_bool = false;
  if (obj->Type->size[1] != 8) {
  } else {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        if (obj->Type->data[b_kstr - 1] != b[b_kstr - 1]) {
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

  guard1 = false;
  if (b_bool) {
    guard1 = true;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      b_0[b_kstr] = tmp_0[b_kstr];
    }

    b_bool = false;
    if (obj->Type->size[1] != 9) {
    } else {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          if (obj->Type->data[b_kstr - 1] != b_0[b_kstr - 1]) {
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
}

static void inverse_kinematics_cat(real_T varargin_1, real_T varargin_2, real_T
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

static void inverse_kinematics_mtimes(const real_T A[36], const
  emxArray_real_T_inverse_kinem_T *B, emxArray_real_T_inverse_kinem_T *C)
{
  inverse_kinematics_B.n = B->size[1] - 1;
  inverse_kinematics_B.b_j_f = C->size[0] * C->size[1];
  C->size[0] = 6;
  C->size[1] = B->size[1];
  invers_emxEnsureCapacity_real_T(C, inverse_kinematics_B.b_j_f);
  for (inverse_kinematics_B.b_j_f = 0; inverse_kinematics_B.b_j_f <=
       inverse_kinematics_B.n; inverse_kinematics_B.b_j_f++) {
    inverse_kinematics_B.coffset_tmp = inverse_kinematics_B.b_j_f * 6 - 1;
    for (inverse_kinematics_B.b_i_i = 0; inverse_kinematics_B.b_i_i < 6;
         inverse_kinematics_B.b_i_i++) {
      inverse_kinematics_B.s_a = 0.0;
      for (inverse_kinematics_B.b_k_o = 0; inverse_kinematics_B.b_k_o < 6;
           inverse_kinematics_B.b_k_o++) {
        inverse_kinematics_B.s_a += A[inverse_kinematics_B.b_k_o * 6 +
          inverse_kinematics_B.b_i_i] * B->data
          [(inverse_kinematics_B.coffset_tmp + inverse_kinematics_B.b_k_o) + 1];
      }

      C->data[(inverse_kinematics_B.coffset_tmp + inverse_kinematics_B.b_i_i) +
        1] = inverse_kinematics_B.s_a;
    }
  }
}

static void RigidBodyTree_efficientFKAndJac(v_robotics_manip_internal_Rig_T *obj,
  const real_T qv[4], const emxArray_char_T_inverse_kinem_T *body1Name, real_T
  T_data[], int32_T T_size[2], emxArray_real_T_inverse_kinem_T *Jac)
{
  c_rigidBodyJoint_inverse_kine_T *joint;
  emxArray_char_T_inverse_kinem_T *body2Name;
  emxArray_real_T_inverse_kinem_T *b;
  emxArray_real_T_inverse_kinem_T *kinematicPathIndices;
  emxArray_real_T_inverse_kinem_T *tmp;
  t_robotics_manip_internal_Rig_T *body1;
  t_robotics_manip_internal_Rig_T *body2;
  static const char_T tmp_0[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_1[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  emxArray_real_T_inverse_kinem_T *Jac_0;
  inverse_kinemati_emxInit_char_T(&body2Name, 2);
  inverse_kinematics_B.result_data_tmp = body2Name->size[0] * body2Name->size[1];
  body2Name->size[0] = 1;
  body2Name->size[1] = obj->Base.NameInternal->size[1];
  invers_emxEnsureCapacity_char_T(body2Name,
    inverse_kinematics_B.result_data_tmp);
  inverse_kinematics_B.loop_ub_m = obj->Base.NameInternal->size[1] - 1;
  for (inverse_kinematics_B.i_l = 0; inverse_kinematics_B.i_l <=
       inverse_kinematics_B.loop_ub_m; inverse_kinematics_B.i_l++) {
    inverse_kinematics_B.result_data_tmp = inverse_kinematics_B.i_l;
    body2Name->data[inverse_kinematics_B.result_data_tmp] =
      obj->Base.NameInternal->data[inverse_kinematics_B.result_data_tmp];
  }

  inverse_kinematics_B.bid1 = RigidBodyTree_findBodyIndexByNa(obj, body1Name);
  inverse_kinematics_B.bid2 = RigidBodyTree_findBodyIndexByNa(obj, body2Name);
  if (inverse_kinematics_B.bid1 == 0.0) {
    body1 = &obj->Base;
  } else {
    body1 = obj->Bodies[static_cast<int32_T>(inverse_kinematics_B.bid1) - 1];
  }

  if (inverse_kinematics_B.bid2 == 0.0) {
    body2 = &obj->Base;
  } else {
    body2 = obj->Bodies[static_cast<int32_T>(inverse_kinematics_B.bid2) - 1];
  }

  inverse_kinemati_emxInit_real_T(&kinematicPathIndices, 2);
  RigidBodyTree_kinematicPathInte(obj, body1, body2, kinematicPathIndices);
  memset(&inverse_kinematics_B.T1[0], 0, sizeof(real_T) << 4U);
  inverse_kinematics_B.T1[0] = 1.0;
  inverse_kinematics_B.T1[5] = 1.0;
  inverse_kinematics_B.T1[10] = 1.0;
  inverse_kinematics_B.T1[15] = 1.0;
  inverse_kinematics_B.result_data_tmp = Jac->size[0] * Jac->size[1];
  Jac->size[0] = 6;
  Jac->size[1] = static_cast<int32_T>(obj->PositionNumber);
  invers_emxEnsureCapacity_real_T(Jac, inverse_kinematics_B.result_data_tmp);
  inverse_kinematics_B.loop_ub_m = 6 * static_cast<int32_T>(obj->PositionNumber)
    - 1;
  if (inverse_kinematics_B.loop_ub_m >= 0) {
    memset(&Jac->data[0], 0, (inverse_kinematics_B.loop_ub_m + 1) * sizeof
           (real_T));
  }

  inverse_kinematics_B.c_o = kinematicPathIndices->size[1] - 2;
  if (kinematicPathIndices->size[1] - 2 >= 0) {
    for (inverse_kinematics_B.i_l = 0; inverse_kinematics_B.i_l < 5;
         inverse_kinematics_B.i_l++) {
      inverse_kinematics_B.b_ev[inverse_kinematics_B.i_l] =
        tmp_0[inverse_kinematics_B.i_l];
    }
  }

  inverse_kinemati_emxInit_real_T(&b, 2);
  inverse_kinemati_emxInit_real_T(&tmp, 2);
  for (inverse_kinematics_B.Jac = 0; inverse_kinematics_B.Jac <=
       inverse_kinematics_B.c_o; inverse_kinematics_B.Jac++) {
    int32_T exitg1;
    inverse_kinematics_B.result_data_tmp = inverse_kinematics_B.Jac;
    if (kinematicPathIndices->data[inverse_kinematics_B.result_data_tmp] != 0.0)
    {
      body1 = obj->Bodies[static_cast<int32_T>(kinematicPathIndices->
        data[inverse_kinematics_B.result_data_tmp]) - 1];
    } else {
      body1 = &obj->Base;
    }

    inverse_kinematics_B.bid1 = kinematicPathIndices->
      data[inverse_kinematics_B.Jac + 1];
    if (inverse_kinematics_B.bid1 != 0.0) {
      body2 = obj->Bodies[static_cast<int32_T>(inverse_kinematics_B.bid1) - 1];
    } else {
      body2 = &obj->Base;
    }

    inverse_kinematics_B.nextBodyIsParent = (body2->Index == body1->ParentIndex);
    if (inverse_kinematics_B.nextBodyIsParent) {
      body2 = body1;
      inverse_kinematics_B.jointSign = 1;
    } else {
      inverse_kinematics_B.jointSign = -1;
    }

    joint = body2->JointInternal;
    inverse_kinematics_B.result_data_tmp = body2Name->size[0] * body2Name->size
      [1];
    body2Name->size[0] = 1;
    body2Name->size[1] = joint->Type->size[1];
    invers_emxEnsureCapacity_char_T(body2Name,
      inverse_kinematics_B.result_data_tmp);
    inverse_kinematics_B.loop_ub_m = joint->Type->size[1] - 1;
    for (inverse_kinematics_B.i_l = 0; inverse_kinematics_B.i_l <=
         inverse_kinematics_B.loop_ub_m; inverse_kinematics_B.i_l++) {
      inverse_kinematics_B.result_data_tmp = inverse_kinematics_B.i_l;
      body2Name->data[inverse_kinematics_B.result_data_tmp] = joint->Type->
        data[inverse_kinematics_B.result_data_tmp];
    }

    inverse_kinematics_B.b_bool = false;
    if (body2Name->size[1] != 5) {
    } else {
      inverse_kinematics_B.i_l = 1;
      do {
        exitg1 = 0;
        if (inverse_kinematics_B.i_l - 1 < 5) {
          if (body2Name->data[inverse_kinematics_B.i_l - 1] !=
              inverse_kinematics_B.b_ev[inverse_kinematics_B.i_l - 1]) {
            exitg1 = 1;
          } else {
            inverse_kinematics_B.i_l++;
          }
        } else {
          inverse_kinematics_B.b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (inverse_kinematics_B.b_bool) {
      for (inverse_kinematics_B.i_l = 0; inverse_kinematics_B.i_l < 16;
           inverse_kinematics_B.i_l++) {
        inverse_kinematics_B.Tj_k[inverse_kinematics_B.i_l] =
          joint->JointToParentTransform[inverse_kinematics_B.i_l];
      }

      inverse_kinematics_B.result_data_tmp = body2Name->size[0] *
        body2Name->size[1];
      body2Name->size[0] = 1;
      body2Name->size[1] = joint->Type->size[1];
      invers_emxEnsureCapacity_char_T(body2Name,
        inverse_kinematics_B.result_data_tmp);
      inverse_kinematics_B.loop_ub_m = joint->Type->size[1] - 1;
      for (inverse_kinematics_B.i_l = 0; inverse_kinematics_B.i_l <=
           inverse_kinematics_B.loop_ub_m; inverse_kinematics_B.i_l++) {
        inverse_kinematics_B.result_data_tmp = inverse_kinematics_B.i_l;
        body2Name->data[inverse_kinematics_B.result_data_tmp] = joint->
          Type->data[inverse_kinematics_B.result_data_tmp];
      }

      inverse_kinematics_B.b_bool = false;
      if (body2Name->size[1] != 5) {
      } else {
        inverse_kinematics_B.i_l = 1;
        do {
          exitg1 = 0;
          if (inverse_kinematics_B.i_l - 1 < 5) {
            if (body2Name->data[inverse_kinematics_B.i_l - 1] !=
                inverse_kinematics_B.b_ev[inverse_kinematics_B.i_l - 1]) {
              exitg1 = 1;
            } else {
              inverse_kinematics_B.i_l++;
            }
          } else {
            inverse_kinematics_B.b_bool = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }

      if (inverse_kinematics_B.b_bool) {
        inverse_kinematics_B.i_l = 0;
      } else {
        for (inverse_kinematics_B.i_l = 0; inverse_kinematics_B.i_l < 8;
             inverse_kinematics_B.i_l++) {
          inverse_kinematics_B.b_f[inverse_kinematics_B.i_l] =
            tmp_1[inverse_kinematics_B.i_l];
        }

        inverse_kinematics_B.b_bool = false;
        if (body2Name->size[1] != 8) {
        } else {
          inverse_kinematics_B.i_l = 1;
          do {
            exitg1 = 0;
            if (inverse_kinematics_B.i_l - 1 < 8) {
              if (body2Name->data[inverse_kinematics_B.i_l - 1] !=
                  inverse_kinematics_B.b_f[inverse_kinematics_B.i_l - 1]) {
                exitg1 = 1;
              } else {
                inverse_kinematics_B.i_l++;
              }
            } else {
              inverse_kinematics_B.b_bool = true;
              exitg1 = 1;
            }
          } while (exitg1 == 0);
        }

        if (inverse_kinematics_B.b_bool) {
          inverse_kinematics_B.i_l = 1;
        } else {
          inverse_kinematics_B.i_l = -1;
        }
      }

      switch (inverse_kinematics_B.i_l) {
       case 0:
        memset(&inverse_kinematics_B.T1j[0], 0, sizeof(real_T) << 4U);
        inverse_kinematics_B.T1j[0] = 1.0;
        inverse_kinematics_B.T1j[5] = 1.0;
        inverse_kinematics_B.T1j[10] = 1.0;
        inverse_kinematics_B.T1j[15] = 1.0;
        break;

       case 1:
        in_rigidBodyJoint_get_JointAxis(joint, inverse_kinematics_B.v);
        inverse_kinematics_B.bid2 = inverse_kinematics_B.v[0];
        inverse_kinematics_B.bid1_tmp = inverse_kinematics_B.v[1];
        inverse_kinematics_B.tempR_tmp = inverse_kinematics_B.v[2];
        inverse_kinematics_B.bid1 = 1.0 / sqrt((inverse_kinematics_B.bid2 *
          inverse_kinematics_B.bid2 + inverse_kinematics_B.bid1_tmp *
          inverse_kinematics_B.bid1_tmp) + inverse_kinematics_B.tempR_tmp *
          inverse_kinematics_B.tempR_tmp);
        inverse_kinematics_B.v[0] = inverse_kinematics_B.bid2 *
          inverse_kinematics_B.bid1;
        inverse_kinematics_B.v[1] = inverse_kinematics_B.bid1_tmp *
          inverse_kinematics_B.bid1;
        inverse_kinematics_B.v[2] = inverse_kinematics_B.tempR_tmp *
          inverse_kinematics_B.bid1;
        inverse_kinematics_B.bid2 = inverse_kinematics_B.v[0] *
          inverse_kinematics_B.v[1] * 0.0;
        inverse_kinematics_B.bid1_tmp = inverse_kinematics_B.v[0] *
          inverse_kinematics_B.v[2] * 0.0;
        inverse_kinematics_B.tempR_tmp = inverse_kinematics_B.v[1] *
          inverse_kinematics_B.v[2] * 0.0;
        inverse_kinematics_cat(inverse_kinematics_B.v[0] *
          inverse_kinematics_B.v[0] * 0.0 + 1.0, inverse_kinematics_B.bid2 -
          inverse_kinematics_B.v[2] * 0.0, inverse_kinematics_B.bid1_tmp +
          inverse_kinematics_B.v[1] * 0.0, inverse_kinematics_B.bid2 +
          inverse_kinematics_B.v[2] * 0.0, inverse_kinematics_B.v[1] *
          inverse_kinematics_B.v[1] * 0.0 + 1.0, inverse_kinematics_B.tempR_tmp
          - inverse_kinematics_B.v[0] * 0.0, inverse_kinematics_B.bid1_tmp -
          inverse_kinematics_B.v[1] * 0.0, inverse_kinematics_B.tempR_tmp +
          inverse_kinematics_B.v[0] * 0.0, inverse_kinematics_B.v[2] *
          inverse_kinematics_B.v[2] * 0.0 + 1.0, inverse_kinematics_B.tempR);
        for (inverse_kinematics_B.i_l = 0; inverse_kinematics_B.i_l < 3;
             inverse_kinematics_B.i_l++) {
          inverse_kinematics_B.R[inverse_kinematics_B.i_l] =
            inverse_kinematics_B.tempR[inverse_kinematics_B.i_l * 3];
          inverse_kinematics_B.R[inverse_kinematics_B.i_l + 3] =
            inverse_kinematics_B.tempR[inverse_kinematics_B.i_l * 3 + 1];
          inverse_kinematics_B.R[inverse_kinematics_B.i_l + 6] =
            inverse_kinematics_B.tempR[inverse_kinematics_B.i_l * 3 + 2];
        }

        memset(&inverse_kinematics_B.T1j[0], 0, sizeof(real_T) << 4U);
        for (inverse_kinematics_B.i_l = 0; inverse_kinematics_B.i_l < 3;
             inverse_kinematics_B.i_l++) {
          inverse_kinematics_B.loop_ub_m = inverse_kinematics_B.i_l << 2;
          inverse_kinematics_B.T1j[inverse_kinematics_B.loop_ub_m] =
            inverse_kinematics_B.R[3 * inverse_kinematics_B.i_l];
          inverse_kinematics_B.T1j[inverse_kinematics_B.loop_ub_m + 1] =
            inverse_kinematics_B.R[3 * inverse_kinematics_B.i_l + 1];
          inverse_kinematics_B.T1j[inverse_kinematics_B.loop_ub_m + 2] =
            inverse_kinematics_B.R[3 * inverse_kinematics_B.i_l + 2];
        }

        inverse_kinematics_B.T1j[15] = 1.0;
        break;

       default:
        in_rigidBodyJoint_get_JointAxis(joint, inverse_kinematics_B.v);
        memset(&inverse_kinematics_B.tempR[0], 0, 9U * sizeof(real_T));
        inverse_kinematics_B.tempR[0] = 1.0;
        inverse_kinematics_B.tempR[4] = 1.0;
        inverse_kinematics_B.tempR[8] = 1.0;
        for (inverse_kinematics_B.i_l = 0; inverse_kinematics_B.i_l < 3;
             inverse_kinematics_B.i_l++) {
          inverse_kinematics_B.loop_ub_m = inverse_kinematics_B.i_l << 2;
          inverse_kinematics_B.T1j[inverse_kinematics_B.loop_ub_m] =
            inverse_kinematics_B.tempR[3 * inverse_kinematics_B.i_l];
          inverse_kinematics_B.T1j[inverse_kinematics_B.loop_ub_m + 1] =
            inverse_kinematics_B.tempR[3 * inverse_kinematics_B.i_l + 1];
          inverse_kinematics_B.T1j[inverse_kinematics_B.loop_ub_m + 2] =
            inverse_kinematics_B.tempR[3 * inverse_kinematics_B.i_l + 2];
          inverse_kinematics_B.T1j[inverse_kinematics_B.i_l + 12] =
            inverse_kinematics_B.v[inverse_kinematics_B.i_l] * 0.0;
        }

        inverse_kinematics_B.T1j[3] = 0.0;
        inverse_kinematics_B.T1j[7] = 0.0;
        inverse_kinematics_B.T1j[11] = 0.0;
        inverse_kinematics_B.T1j[15] = 1.0;
        break;
      }

      for (inverse_kinematics_B.i_l = 0; inverse_kinematics_B.i_l < 16;
           inverse_kinematics_B.i_l++) {
        inverse_kinematics_B.b[inverse_kinematics_B.i_l] =
          joint->ChildToJointTransform[inverse_kinematics_B.i_l];
      }

      for (inverse_kinematics_B.i_l = 0; inverse_kinematics_B.i_l < 4;
           inverse_kinematics_B.i_l++) {
        for (inverse_kinematics_B.g = 0; inverse_kinematics_B.g < 4;
             inverse_kinematics_B.g++) {
          inverse_kinematics_B.f = inverse_kinematics_B.g << 2;
          inverse_kinematics_B.loop_ub_m = inverse_kinematics_B.i_l +
            inverse_kinematics_B.f;
          inverse_kinematics_B.Tj_c[inverse_kinematics_B.loop_ub_m] = 0.0;
          inverse_kinematics_B.Tj_c[inverse_kinematics_B.loop_ub_m] +=
            inverse_kinematics_B.T1j[inverse_kinematics_B.f] *
            inverse_kinematics_B.Tj_k[inverse_kinematics_B.i_l];
          inverse_kinematics_B.Tj_c[inverse_kinematics_B.loop_ub_m] +=
            inverse_kinematics_B.T1j[inverse_kinematics_B.f + 1] *
            inverse_kinematics_B.Tj_k[inverse_kinematics_B.i_l + 4];
          inverse_kinematics_B.Tj_c[inverse_kinematics_B.loop_ub_m] +=
            inverse_kinematics_B.T1j[inverse_kinematics_B.f + 2] *
            inverse_kinematics_B.Tj_k[inverse_kinematics_B.i_l + 8];
          inverse_kinematics_B.Tj_c[inverse_kinematics_B.loop_ub_m] +=
            inverse_kinematics_B.T1j[inverse_kinematics_B.f + 3] *
            inverse_kinematics_B.Tj_k[inverse_kinematics_B.i_l + 12];
        }

        for (inverse_kinematics_B.g = 0; inverse_kinematics_B.g < 4;
             inverse_kinematics_B.g++) {
          inverse_kinematics_B.loop_ub_m = inverse_kinematics_B.g << 2;
          inverse_kinematics_B.f = inverse_kinematics_B.i_l +
            inverse_kinematics_B.loop_ub_m;
          inverse_kinematics_B.Tc2p[inverse_kinematics_B.f] = 0.0;
          inverse_kinematics_B.Tc2p[inverse_kinematics_B.f] +=
            inverse_kinematics_B.b[inverse_kinematics_B.loop_ub_m] *
            inverse_kinematics_B.Tj_c[inverse_kinematics_B.i_l];
          inverse_kinematics_B.Tc2p[inverse_kinematics_B.f] +=
            inverse_kinematics_B.b[inverse_kinematics_B.loop_ub_m + 1] *
            inverse_kinematics_B.Tj_c[inverse_kinematics_B.i_l + 4];
          inverse_kinematics_B.Tc2p[inverse_kinematics_B.f] +=
            inverse_kinematics_B.b[inverse_kinematics_B.loop_ub_m + 2] *
            inverse_kinematics_B.Tj_c[inverse_kinematics_B.i_l + 8];
          inverse_kinematics_B.Tc2p[inverse_kinematics_B.f] +=
            inverse_kinematics_B.b[inverse_kinematics_B.loop_ub_m + 3] *
            inverse_kinematics_B.Tj_c[inverse_kinematics_B.i_l + 12];
        }
      }
    } else {
      inverse_kinematics_B.i_l = static_cast<int32_T>(body2->Index);
      inverse_kinematics_B.bid1 = obj->PositionDoFMap[inverse_kinematics_B.i_l -
        1];
      inverse_kinematics_B.bid2 = obj->PositionDoFMap[inverse_kinematics_B.i_l +
        6];
      if (inverse_kinematics_B.bid1 > inverse_kinematics_B.bid2) {
        inverse_kinematics_B.g = 0;
        inverse_kinematics_B.f = 0;
      } else {
        inverse_kinematics_B.g = static_cast<int32_T>(inverse_kinematics_B.bid1)
          - 1;
        inverse_kinematics_B.f = static_cast<int32_T>(inverse_kinematics_B.bid2);
      }

      for (inverse_kinematics_B.i_l = 0; inverse_kinematics_B.i_l < 16;
           inverse_kinematics_B.i_l++) {
        inverse_kinematics_B.Tj_k[inverse_kinematics_B.i_l] =
          joint->JointToParentTransform[inverse_kinematics_B.i_l];
      }

      inverse_kinematics_B.result_data_tmp = body2Name->size[0] *
        body2Name->size[1];
      body2Name->size[0] = 1;
      body2Name->size[1] = joint->Type->size[1];
      invers_emxEnsureCapacity_char_T(body2Name,
        inverse_kinematics_B.result_data_tmp);
      inverse_kinematics_B.loop_ub_m = joint->Type->size[1] - 1;
      for (inverse_kinematics_B.i_l = 0; inverse_kinematics_B.i_l <=
           inverse_kinematics_B.loop_ub_m; inverse_kinematics_B.i_l++) {
        inverse_kinematics_B.result_data_tmp = inverse_kinematics_B.i_l;
        body2Name->data[inverse_kinematics_B.result_data_tmp] = joint->
          Type->data[inverse_kinematics_B.result_data_tmp];
      }

      inverse_kinematics_B.b_bool = false;
      if (body2Name->size[1] != 5) {
      } else {
        inverse_kinematics_B.i_l = 1;
        do {
          exitg1 = 0;
          if (inverse_kinematics_B.i_l - 1 < 5) {
            if (body2Name->data[inverse_kinematics_B.i_l - 1] !=
                inverse_kinematics_B.b_ev[inverse_kinematics_B.i_l - 1]) {
              exitg1 = 1;
            } else {
              inverse_kinematics_B.i_l++;
            }
          } else {
            inverse_kinematics_B.b_bool = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }

      if (inverse_kinematics_B.b_bool) {
        inverse_kinematics_B.i_l = 0;
      } else {
        for (inverse_kinematics_B.i_l = 0; inverse_kinematics_B.i_l < 8;
             inverse_kinematics_B.i_l++) {
          inverse_kinematics_B.b_f[inverse_kinematics_B.i_l] =
            tmp_1[inverse_kinematics_B.i_l];
        }

        inverse_kinematics_B.b_bool = false;
        if (body2Name->size[1] != 8) {
        } else {
          inverse_kinematics_B.i_l = 1;
          do {
            exitg1 = 0;
            if (inverse_kinematics_B.i_l - 1 < 8) {
              if (body2Name->data[inverse_kinematics_B.i_l - 1] !=
                  inverse_kinematics_B.b_f[inverse_kinematics_B.i_l - 1]) {
                exitg1 = 1;
              } else {
                inverse_kinematics_B.i_l++;
              }
            } else {
              inverse_kinematics_B.b_bool = true;
              exitg1 = 1;
            }
          } while (exitg1 == 0);
        }

        if (inverse_kinematics_B.b_bool) {
          inverse_kinematics_B.i_l = 1;
        } else {
          inverse_kinematics_B.i_l = -1;
        }
      }

      switch (inverse_kinematics_B.i_l) {
       case 0:
        memset(&inverse_kinematics_B.T1j[0], 0, sizeof(real_T) << 4U);
        inverse_kinematics_B.T1j[0] = 1.0;
        inverse_kinematics_B.T1j[5] = 1.0;
        inverse_kinematics_B.T1j[10] = 1.0;
        inverse_kinematics_B.T1j[15] = 1.0;
        break;

       case 1:
        in_rigidBodyJoint_get_JointAxis(joint, inverse_kinematics_B.v);
        inverse_kinematics_B.i_l = 0;
        inverse_kinematics_B.result_data[inverse_kinematics_B.i_l] =
          inverse_kinematics_B.v[0];
        inverse_kinematics_B.result_data_tmp = 1;
        inverse_kinematics_B.result_data[inverse_kinematics_B.result_data_tmp] =
          inverse_kinematics_B.v[1];
        inverse_kinematics_B.loop_ub_m = 2;
        inverse_kinematics_B.result_data[inverse_kinematics_B.loop_ub_m] =
          inverse_kinematics_B.v[2];
        if ((inverse_kinematics_B.f - inverse_kinematics_B.g != 0) - 1 >= 0) {
          inverse_kinematics_B.result_data[3] = qv[inverse_kinematics_B.g];
        }

        inverse_kinematics_B.bid2 =
          inverse_kinematics_B.result_data[inverse_kinematics_B.i_l];
        inverse_kinematics_B.sth =
          inverse_kinematics_B.result_data[inverse_kinematics_B.result_data_tmp];
        inverse_kinematics_B.bid1_tmp =
          inverse_kinematics_B.result_data[inverse_kinematics_B.loop_ub_m];
        inverse_kinematics_B.bid1 = 1.0 / sqrt((inverse_kinematics_B.bid2 *
          inverse_kinematics_B.bid2 + inverse_kinematics_B.sth *
          inverse_kinematics_B.sth) + inverse_kinematics_B.bid1_tmp *
          inverse_kinematics_B.bid1_tmp);
        inverse_kinematics_B.v[0] = inverse_kinematics_B.bid2 *
          inverse_kinematics_B.bid1;
        inverse_kinematics_B.v[1] = inverse_kinematics_B.sth *
          inverse_kinematics_B.bid1;
        inverse_kinematics_B.v[2] = inverse_kinematics_B.bid1_tmp *
          inverse_kinematics_B.bid1;
        inverse_kinematics_B.bid2 = inverse_kinematics_B.result_data[3];
        inverse_kinematics_B.bid1 = cos(inverse_kinematics_B.bid2);
        inverse_kinematics_B.sth = sin(inverse_kinematics_B.bid2);
        inverse_kinematics_B.bid2 = inverse_kinematics_B.v[0] *
          inverse_kinematics_B.v[1] * (1.0 - inverse_kinematics_B.bid1);
        inverse_kinematics_B.bid1_tmp = inverse_kinematics_B.v[2] *
          inverse_kinematics_B.sth;
        inverse_kinematics_B.tempR_tmp = inverse_kinematics_B.v[0] *
          inverse_kinematics_B.v[2] * (1.0 - inverse_kinematics_B.bid1);
        inverse_kinematics_B.tempR_tmp_c = inverse_kinematics_B.v[1] *
          inverse_kinematics_B.sth;
        inverse_kinematics_B.tempR_tmp_m = inverse_kinematics_B.v[1] *
          inverse_kinematics_B.v[2] * (1.0 - inverse_kinematics_B.bid1);
        inverse_kinematics_B.sth *= inverse_kinematics_B.v[0];
        inverse_kinematics_cat(inverse_kinematics_B.v[0] *
          inverse_kinematics_B.v[0] * (1.0 - inverse_kinematics_B.bid1) +
          inverse_kinematics_B.bid1, inverse_kinematics_B.bid2 -
          inverse_kinematics_B.bid1_tmp, inverse_kinematics_B.tempR_tmp +
          inverse_kinematics_B.tempR_tmp_c, inverse_kinematics_B.bid2 +
          inverse_kinematics_B.bid1_tmp, inverse_kinematics_B.v[1] *
          inverse_kinematics_B.v[1] * (1.0 - inverse_kinematics_B.bid1) +
          inverse_kinematics_B.bid1, inverse_kinematics_B.tempR_tmp_m -
          inverse_kinematics_B.sth, inverse_kinematics_B.tempR_tmp -
          inverse_kinematics_B.tempR_tmp_c, inverse_kinematics_B.tempR_tmp_m +
          inverse_kinematics_B.sth, inverse_kinematics_B.v[2] *
          inverse_kinematics_B.v[2] * (1.0 - inverse_kinematics_B.bid1) +
          inverse_kinematics_B.bid1, inverse_kinematics_B.tempR);
        for (inverse_kinematics_B.i_l = 0; inverse_kinematics_B.i_l < 3;
             inverse_kinematics_B.i_l++) {
          inverse_kinematics_B.R[inverse_kinematics_B.i_l] =
            inverse_kinematics_B.tempR[inverse_kinematics_B.i_l * 3];
          inverse_kinematics_B.R[inverse_kinematics_B.i_l + 3] =
            inverse_kinematics_B.tempR[inverse_kinematics_B.i_l * 3 + 1];
          inverse_kinematics_B.R[inverse_kinematics_B.i_l + 6] =
            inverse_kinematics_B.tempR[inverse_kinematics_B.i_l * 3 + 2];
        }

        memset(&inverse_kinematics_B.T1j[0], 0, sizeof(real_T) << 4U);
        for (inverse_kinematics_B.i_l = 0; inverse_kinematics_B.i_l < 3;
             inverse_kinematics_B.i_l++) {
          inverse_kinematics_B.loop_ub_m = inverse_kinematics_B.i_l << 2;
          inverse_kinematics_B.T1j[inverse_kinematics_B.loop_ub_m] =
            inverse_kinematics_B.R[3 * inverse_kinematics_B.i_l];
          inverse_kinematics_B.T1j[inverse_kinematics_B.loop_ub_m + 1] =
            inverse_kinematics_B.R[3 * inverse_kinematics_B.i_l + 1];
          inverse_kinematics_B.T1j[inverse_kinematics_B.loop_ub_m + 2] =
            inverse_kinematics_B.R[3 * inverse_kinematics_B.i_l + 2];
        }

        inverse_kinematics_B.T1j[15] = 1.0;
        break;

       default:
        in_rigidBodyJoint_get_JointAxis(joint, inverse_kinematics_B.v);
        memset(&inverse_kinematics_B.tempR[0], 0, 9U * sizeof(real_T));
        inverse_kinematics_B.tempR[0] = 1.0;
        inverse_kinematics_B.tempR[4] = 1.0;
        inverse_kinematics_B.tempR[8] = 1.0;
        inverse_kinematics_B.bid1 = qv[inverse_kinematics_B.g];
        for (inverse_kinematics_B.i_l = 0; inverse_kinematics_B.i_l < 3;
             inverse_kinematics_B.i_l++) {
          inverse_kinematics_B.loop_ub_m = inverse_kinematics_B.i_l << 2;
          inverse_kinematics_B.T1j[inverse_kinematics_B.loop_ub_m] =
            inverse_kinematics_B.tempR[3 * inverse_kinematics_B.i_l];
          inverse_kinematics_B.T1j[inverse_kinematics_B.loop_ub_m + 1] =
            inverse_kinematics_B.tempR[3 * inverse_kinematics_B.i_l + 1];
          inverse_kinematics_B.T1j[inverse_kinematics_B.loop_ub_m + 2] =
            inverse_kinematics_B.tempR[3 * inverse_kinematics_B.i_l + 2];
          inverse_kinematics_B.T1j[inverse_kinematics_B.i_l + 12] =
            inverse_kinematics_B.v[inverse_kinematics_B.i_l] *
            inverse_kinematics_B.bid1;
        }

        inverse_kinematics_B.T1j[3] = 0.0;
        inverse_kinematics_B.T1j[7] = 0.0;
        inverse_kinematics_B.T1j[11] = 0.0;
        inverse_kinematics_B.T1j[15] = 1.0;
        break;
      }

      for (inverse_kinematics_B.i_l = 0; inverse_kinematics_B.i_l < 16;
           inverse_kinematics_B.i_l++) {
        inverse_kinematics_B.b[inverse_kinematics_B.i_l] =
          joint->ChildToJointTransform[inverse_kinematics_B.i_l];
      }

      for (inverse_kinematics_B.i_l = 0; inverse_kinematics_B.i_l < 4;
           inverse_kinematics_B.i_l++) {
        for (inverse_kinematics_B.g = 0; inverse_kinematics_B.g < 4;
             inverse_kinematics_B.g++) {
          inverse_kinematics_B.f = inverse_kinematics_B.g << 2;
          inverse_kinematics_B.loop_ub_m = inverse_kinematics_B.i_l +
            inverse_kinematics_B.f;
          inverse_kinematics_B.Tj_c[inverse_kinematics_B.loop_ub_m] = 0.0;
          inverse_kinematics_B.Tj_c[inverse_kinematics_B.loop_ub_m] +=
            inverse_kinematics_B.T1j[inverse_kinematics_B.f] *
            inverse_kinematics_B.Tj_k[inverse_kinematics_B.i_l];
          inverse_kinematics_B.Tj_c[inverse_kinematics_B.loop_ub_m] +=
            inverse_kinematics_B.T1j[inverse_kinematics_B.f + 1] *
            inverse_kinematics_B.Tj_k[inverse_kinematics_B.i_l + 4];
          inverse_kinematics_B.Tj_c[inverse_kinematics_B.loop_ub_m] +=
            inverse_kinematics_B.T1j[inverse_kinematics_B.f + 2] *
            inverse_kinematics_B.Tj_k[inverse_kinematics_B.i_l + 8];
          inverse_kinematics_B.Tj_c[inverse_kinematics_B.loop_ub_m] +=
            inverse_kinematics_B.T1j[inverse_kinematics_B.f + 3] *
            inverse_kinematics_B.Tj_k[inverse_kinematics_B.i_l + 12];
        }

        for (inverse_kinematics_B.g = 0; inverse_kinematics_B.g < 4;
             inverse_kinematics_B.g++) {
          inverse_kinematics_B.loop_ub_m = inverse_kinematics_B.g << 2;
          inverse_kinematics_B.f = inverse_kinematics_B.i_l +
            inverse_kinematics_B.loop_ub_m;
          inverse_kinematics_B.Tc2p[inverse_kinematics_B.f] = 0.0;
          inverse_kinematics_B.Tc2p[inverse_kinematics_B.f] +=
            inverse_kinematics_B.b[inverse_kinematics_B.loop_ub_m] *
            inverse_kinematics_B.Tj_c[inverse_kinematics_B.i_l];
          inverse_kinematics_B.Tc2p[inverse_kinematics_B.f] +=
            inverse_kinematics_B.b[inverse_kinematics_B.loop_ub_m + 1] *
            inverse_kinematics_B.Tj_c[inverse_kinematics_B.i_l + 4];
          inverse_kinematics_B.Tc2p[inverse_kinematics_B.f] +=
            inverse_kinematics_B.b[inverse_kinematics_B.loop_ub_m + 2] *
            inverse_kinematics_B.Tj_c[inverse_kinematics_B.i_l + 8];
          inverse_kinematics_B.Tc2p[inverse_kinematics_B.f] +=
            inverse_kinematics_B.b[inverse_kinematics_B.loop_ub_m + 3] *
            inverse_kinematics_B.Tj_c[inverse_kinematics_B.i_l + 12];
        }
      }

      inverse_kinematics_B.i_l = static_cast<int32_T>(body2->Index);
      inverse_kinematics_B.bid1 = obj->VelocityDoFMap[inverse_kinematics_B.i_l -
        1];
      inverse_kinematics_B.bid2 = obj->VelocityDoFMap[inverse_kinematics_B.i_l +
        6];
      if (inverse_kinematics_B.nextBodyIsParent) {
        for (inverse_kinematics_B.i_l = 0; inverse_kinematics_B.i_l < 16;
             inverse_kinematics_B.i_l++) {
          inverse_kinematics_B.Tj_k[inverse_kinematics_B.i_l] =
            joint->ChildToJointTransform[inverse_kinematics_B.i_l];
        }
      } else {
        for (inverse_kinematics_B.i_l = 0; inverse_kinematics_B.i_l < 16;
             inverse_kinematics_B.i_l++) {
          inverse_kinematics_B.T1j[inverse_kinematics_B.i_l] =
            joint->JointToParentTransform[inverse_kinematics_B.i_l];
        }

        for (inverse_kinematics_B.i_l = 0; inverse_kinematics_B.i_l < 3;
             inverse_kinematics_B.i_l++) {
          inverse_kinematics_B.R[3 * inverse_kinematics_B.i_l] =
            inverse_kinematics_B.T1j[inverse_kinematics_B.i_l];
          inverse_kinematics_B.R[3 * inverse_kinematics_B.i_l + 1] =
            inverse_kinematics_B.T1j[inverse_kinematics_B.i_l + 4];
          inverse_kinematics_B.R[3 * inverse_kinematics_B.i_l + 2] =
            inverse_kinematics_B.T1j[inverse_kinematics_B.i_l + 8];
        }

        for (inverse_kinematics_B.i_l = 0; inverse_kinematics_B.i_l < 9;
             inverse_kinematics_B.i_l++) {
          inverse_kinematics_B.tempR[inverse_kinematics_B.i_l] =
            -inverse_kinematics_B.R[inverse_kinematics_B.i_l];
        }

        for (inverse_kinematics_B.i_l = 0; inverse_kinematics_B.i_l < 3;
             inverse_kinematics_B.i_l++) {
          inverse_kinematics_B.loop_ub_m = inverse_kinematics_B.i_l << 2;
          inverse_kinematics_B.Tj_k[inverse_kinematics_B.loop_ub_m] =
            inverse_kinematics_B.R[3 * inverse_kinematics_B.i_l];
          inverse_kinematics_B.Tj_k[inverse_kinematics_B.loop_ub_m + 1] =
            inverse_kinematics_B.R[3 * inverse_kinematics_B.i_l + 1];
          inverse_kinematics_B.Tj_k[inverse_kinematics_B.loop_ub_m + 2] =
            inverse_kinematics_B.R[3 * inverse_kinematics_B.i_l + 2];
          inverse_kinematics_B.Tj_k[inverse_kinematics_B.i_l + 12] =
            (inverse_kinematics_B.tempR[inverse_kinematics_B.i_l + 3] *
             inverse_kinematics_B.T1j[13] +
             inverse_kinematics_B.tempR[inverse_kinematics_B.i_l] *
             inverse_kinematics_B.T1j[12]) +
            inverse_kinematics_B.tempR[inverse_kinematics_B.i_l + 6] *
            inverse_kinematics_B.T1j[14];
        }

        inverse_kinematics_B.Tj_k[3] = 0.0;
        inverse_kinematics_B.Tj_k[7] = 0.0;
        inverse_kinematics_B.Tj_k[11] = 0.0;
        inverse_kinematics_B.Tj_k[15] = 1.0;
      }

      for (inverse_kinematics_B.i_l = 0; inverse_kinematics_B.i_l < 4;
           inverse_kinematics_B.i_l++) {
        for (inverse_kinematics_B.g = 0; inverse_kinematics_B.g < 4;
             inverse_kinematics_B.g++) {
          inverse_kinematics_B.f = inverse_kinematics_B.g << 2;
          inverse_kinematics_B.loop_ub_m = inverse_kinematics_B.i_l +
            inverse_kinematics_B.f;
          inverse_kinematics_B.T1j[inverse_kinematics_B.loop_ub_m] = 0.0;
          inverse_kinematics_B.T1j[inverse_kinematics_B.loop_ub_m] +=
            inverse_kinematics_B.T1[inverse_kinematics_B.f] *
            inverse_kinematics_B.Tj_k[inverse_kinematics_B.i_l];
          inverse_kinematics_B.T1j[inverse_kinematics_B.loop_ub_m] +=
            inverse_kinematics_B.T1[inverse_kinematics_B.f + 1] *
            inverse_kinematics_B.Tj_k[inverse_kinematics_B.i_l + 4];
          inverse_kinematics_B.T1j[inverse_kinematics_B.loop_ub_m] +=
            inverse_kinematics_B.T1[inverse_kinematics_B.f + 2] *
            inverse_kinematics_B.Tj_k[inverse_kinematics_B.i_l + 8];
          inverse_kinematics_B.T1j[inverse_kinematics_B.loop_ub_m] +=
            inverse_kinematics_B.T1[inverse_kinematics_B.f + 3] *
            inverse_kinematics_B.Tj_k[inverse_kinematics_B.i_l + 12];
        }
      }

      for (inverse_kinematics_B.i_l = 0; inverse_kinematics_B.i_l < 3;
           inverse_kinematics_B.i_l++) {
        inverse_kinematics_B.R[3 * inverse_kinematics_B.i_l] =
          inverse_kinematics_B.T1j[inverse_kinematics_B.i_l];
        inverse_kinematics_B.R[3 * inverse_kinematics_B.i_l + 1] =
          inverse_kinematics_B.T1j[inverse_kinematics_B.i_l + 4];
        inverse_kinematics_B.R[3 * inverse_kinematics_B.i_l + 2] =
          inverse_kinematics_B.T1j[inverse_kinematics_B.i_l + 8];
      }

      for (inverse_kinematics_B.i_l = 0; inverse_kinematics_B.i_l < 9;
           inverse_kinematics_B.i_l++) {
        inverse_kinematics_B.tempR[inverse_kinematics_B.i_l] =
          -inverse_kinematics_B.R[inverse_kinematics_B.i_l];
      }

      for (inverse_kinematics_B.i_l = 0; inverse_kinematics_B.i_l < 3;
           inverse_kinematics_B.i_l++) {
        inverse_kinematics_B.loop_ub_m = inverse_kinematics_B.i_l << 2;
        inverse_kinematics_B.Tj_k[inverse_kinematics_B.loop_ub_m] =
          inverse_kinematics_B.R[3 * inverse_kinematics_B.i_l];
        inverse_kinematics_B.Tj_k[inverse_kinematics_B.loop_ub_m + 1] =
          inverse_kinematics_B.R[3 * inverse_kinematics_B.i_l + 1];
        inverse_kinematics_B.Tj_k[inverse_kinematics_B.loop_ub_m + 2] =
          inverse_kinematics_B.R[3 * inverse_kinematics_B.i_l + 2];
        inverse_kinematics_B.Tj_k[inverse_kinematics_B.i_l + 12] =
          (inverse_kinematics_B.tempR[inverse_kinematics_B.i_l + 3] *
           inverse_kinematics_B.T1j[13] +
           inverse_kinematics_B.tempR[inverse_kinematics_B.i_l] *
           inverse_kinematics_B.T1j[12]) +
          inverse_kinematics_B.tempR[inverse_kinematics_B.i_l + 6] *
          inverse_kinematics_B.T1j[14];
      }

      inverse_kinematics_B.Tj_k[3] = 0.0;
      inverse_kinematics_B.Tj_k[7] = 0.0;
      inverse_kinematics_B.Tj_k[11] = 0.0;
      inverse_kinematics_B.Tj_k[15] = 1.0;
      inverse_kinematics_B.result_data_tmp = b->size[0] * b->size[1];
      b->size[0] = 6;
      b->size[1] = joint->MotionSubspace->size[1];
      invers_emxEnsureCapacity_real_T(b, inverse_kinematics_B.result_data_tmp);
      inverse_kinematics_B.loop_ub_m = 6 * joint->MotionSubspace->size[1];
      for (inverse_kinematics_B.i_l = 0; inverse_kinematics_B.i_l <
           inverse_kinematics_B.loop_ub_m; inverse_kinematics_B.i_l++) {
        b->data[inverse_kinematics_B.i_l] = joint->MotionSubspace->
          data[inverse_kinematics_B.i_l];
      }

      if (inverse_kinematics_B.bid1 > inverse_kinematics_B.bid2) {
        inverse_kinematics_B.f = 0;
      } else {
        inverse_kinematics_B.f = static_cast<int32_T>(inverse_kinematics_B.bid1)
          - 1;
      }

      inverse_kinematics_B.R[0] = 0.0;
      inverse_kinematics_B.R[3] = -inverse_kinematics_B.Tj_k[14];
      inverse_kinematics_B.R[6] = inverse_kinematics_B.Tj_k[13];
      inverse_kinematics_B.R[1] = inverse_kinematics_B.Tj_k[14];
      inverse_kinematics_B.R[4] = 0.0;
      inverse_kinematics_B.R[7] = -inverse_kinematics_B.Tj_k[12];
      inverse_kinematics_B.R[2] = -inverse_kinematics_B.Tj_k[13];
      inverse_kinematics_B.R[5] = inverse_kinematics_B.Tj_k[12];
      inverse_kinematics_B.R[8] = 0.0;
      for (inverse_kinematics_B.i_l = 0; inverse_kinematics_B.i_l < 3;
           inverse_kinematics_B.i_l++) {
        for (inverse_kinematics_B.g = 0; inverse_kinematics_B.g < 3;
             inverse_kinematics_B.g++) {
          inverse_kinematics_B.loop_ub_m = 3 * inverse_kinematics_B.g +
            inverse_kinematics_B.i_l;
          inverse_kinematics_B.tempR[inverse_kinematics_B.loop_ub_m] = 0.0;
          inverse_kinematics_B.result_data_tmp = inverse_kinematics_B.g << 2;
          inverse_kinematics_B.tempR[inverse_kinematics_B.loop_ub_m] +=
            inverse_kinematics_B.Tj_k[inverse_kinematics_B.result_data_tmp] *
            inverse_kinematics_B.R[inverse_kinematics_B.i_l];
          inverse_kinematics_B.tempR[inverse_kinematics_B.loop_ub_m] +=
            inverse_kinematics_B.Tj_k[inverse_kinematics_B.result_data_tmp + 1] *
            inverse_kinematics_B.R[inverse_kinematics_B.i_l + 3];
          inverse_kinematics_B.tempR[inverse_kinematics_B.loop_ub_m] +=
            inverse_kinematics_B.Tj_k[inverse_kinematics_B.result_data_tmp + 2] *
            inverse_kinematics_B.R[inverse_kinematics_B.i_l + 6];
          inverse_kinematics_B.Tj[inverse_kinematics_B.g + 6 *
            inverse_kinematics_B.i_l] = inverse_kinematics_B.Tj_k
            [(inverse_kinematics_B.i_l << 2) + inverse_kinematics_B.g];
          inverse_kinematics_B.Tj[inverse_kinematics_B.g + 6 *
            (inverse_kinematics_B.i_l + 3)] = 0.0;
        }
      }

      for (inverse_kinematics_B.i_l = 0; inverse_kinematics_B.i_l < 3;
           inverse_kinematics_B.i_l++) {
        inverse_kinematics_B.Tj[6 * inverse_kinematics_B.i_l + 3] =
          inverse_kinematics_B.tempR[3 * inverse_kinematics_B.i_l];
        inverse_kinematics_B.loop_ub_m = inverse_kinematics_B.i_l << 2;
        inverse_kinematics_B.g = (inverse_kinematics_B.i_l + 3) * 6;
        inverse_kinematics_B.Tj[inverse_kinematics_B.g + 3] =
          inverse_kinematics_B.Tj_k[inverse_kinematics_B.loop_ub_m];
        inverse_kinematics_B.Tj[6 * inverse_kinematics_B.i_l + 4] =
          inverse_kinematics_B.tempR[3 * inverse_kinematics_B.i_l + 1];
        inverse_kinematics_B.Tj[inverse_kinematics_B.g + 4] =
          inverse_kinematics_B.Tj_k[inverse_kinematics_B.loop_ub_m + 1];
        inverse_kinematics_B.Tj[6 * inverse_kinematics_B.i_l + 5] =
          inverse_kinematics_B.tempR[3 * inverse_kinematics_B.i_l + 2];
        inverse_kinematics_B.Tj[inverse_kinematics_B.g + 5] =
          inverse_kinematics_B.Tj_k[inverse_kinematics_B.loop_ub_m + 2];
      }

      inverse_kinematics_mtimes(inverse_kinematics_B.Tj, b, tmp);
      inverse_kinematics_B.loop_ub_m = tmp->size[1];
      for (inverse_kinematics_B.i_l = 0; inverse_kinematics_B.i_l <
           inverse_kinematics_B.loop_ub_m; inverse_kinematics_B.i_l++) {
        for (inverse_kinematics_B.g = 0; inverse_kinematics_B.g < 6;
             inverse_kinematics_B.g++) {
          Jac->data[inverse_kinematics_B.g + 6 * (inverse_kinematics_B.f +
            inverse_kinematics_B.i_l)] = tmp->data[6 * inverse_kinematics_B.i_l
            + inverse_kinematics_B.g] * static_cast<real_T>
            (inverse_kinematics_B.jointSign);
        }
      }
    }

    if (inverse_kinematics_B.nextBodyIsParent) {
      for (inverse_kinematics_B.i_l = 0; inverse_kinematics_B.i_l < 4;
           inverse_kinematics_B.i_l++) {
        for (inverse_kinematics_B.g = 0; inverse_kinematics_B.g < 4;
             inverse_kinematics_B.g++) {
          inverse_kinematics_B.loop_ub_m = inverse_kinematics_B.g << 2;
          inverse_kinematics_B.f = inverse_kinematics_B.i_l +
            inverse_kinematics_B.loop_ub_m;
          inverse_kinematics_B.Tj_k[inverse_kinematics_B.f] = 0.0;
          inverse_kinematics_B.Tj_k[inverse_kinematics_B.f] +=
            inverse_kinematics_B.T1[inverse_kinematics_B.loop_ub_m] *
            inverse_kinematics_B.Tc2p[inverse_kinematics_B.i_l];
          inverse_kinematics_B.Tj_k[inverse_kinematics_B.f] +=
            inverse_kinematics_B.T1[inverse_kinematics_B.loop_ub_m + 1] *
            inverse_kinematics_B.Tc2p[inverse_kinematics_B.i_l + 4];
          inverse_kinematics_B.Tj_k[inverse_kinematics_B.f] +=
            inverse_kinematics_B.T1[inverse_kinematics_B.loop_ub_m + 2] *
            inverse_kinematics_B.Tc2p[inverse_kinematics_B.i_l + 8];
          inverse_kinematics_B.Tj_k[inverse_kinematics_B.f] +=
            inverse_kinematics_B.T1[inverse_kinematics_B.loop_ub_m + 3] *
            inverse_kinematics_B.Tc2p[inverse_kinematics_B.i_l + 12];
        }
      }

      memcpy(&inverse_kinematics_B.T1[0], &inverse_kinematics_B.Tj_k[0], sizeof
             (real_T) << 4U);
    } else {
      for (inverse_kinematics_B.i_l = 0; inverse_kinematics_B.i_l < 3;
           inverse_kinematics_B.i_l++) {
        inverse_kinematics_B.R[3 * inverse_kinematics_B.i_l] =
          inverse_kinematics_B.Tc2p[inverse_kinematics_B.i_l];
        inverse_kinematics_B.R[3 * inverse_kinematics_B.i_l + 1] =
          inverse_kinematics_B.Tc2p[inverse_kinematics_B.i_l + 4];
        inverse_kinematics_B.R[3 * inverse_kinematics_B.i_l + 2] =
          inverse_kinematics_B.Tc2p[inverse_kinematics_B.i_l + 8];
      }

      for (inverse_kinematics_B.i_l = 0; inverse_kinematics_B.i_l < 9;
           inverse_kinematics_B.i_l++) {
        inverse_kinematics_B.tempR[inverse_kinematics_B.i_l] =
          -inverse_kinematics_B.R[inverse_kinematics_B.i_l];
      }

      for (inverse_kinematics_B.i_l = 0; inverse_kinematics_B.i_l < 3;
           inverse_kinematics_B.i_l++) {
        inverse_kinematics_B.jointSign = inverse_kinematics_B.i_l << 2;
        inverse_kinematics_B.Tj_k[inverse_kinematics_B.jointSign] =
          inverse_kinematics_B.R[3 * inverse_kinematics_B.i_l];
        inverse_kinematics_B.Tj_k[inverse_kinematics_B.jointSign + 1] =
          inverse_kinematics_B.R[3 * inverse_kinematics_B.i_l + 1];
        inverse_kinematics_B.Tj_k[inverse_kinematics_B.jointSign + 2] =
          inverse_kinematics_B.R[3 * inverse_kinematics_B.i_l + 2];
        inverse_kinematics_B.Tj_k[inverse_kinematics_B.i_l + 12] =
          (inverse_kinematics_B.tempR[inverse_kinematics_B.i_l + 3] *
           inverse_kinematics_B.Tc2p[13] +
           inverse_kinematics_B.tempR[inverse_kinematics_B.i_l] *
           inverse_kinematics_B.Tc2p[12]) +
          inverse_kinematics_B.tempR[inverse_kinematics_B.i_l + 6] *
          inverse_kinematics_B.Tc2p[14];
      }

      inverse_kinematics_B.Tj_k[3] = 0.0;
      inverse_kinematics_B.Tj_k[7] = 0.0;
      inverse_kinematics_B.Tj_k[11] = 0.0;
      inverse_kinematics_B.Tj_k[15] = 1.0;
      for (inverse_kinematics_B.i_l = 0; inverse_kinematics_B.i_l < 4;
           inverse_kinematics_B.i_l++) {
        for (inverse_kinematics_B.g = 0; inverse_kinematics_B.g < 4;
             inverse_kinematics_B.g++) {
          inverse_kinematics_B.loop_ub_m = inverse_kinematics_B.g << 2;
          inverse_kinematics_B.jointSign = inverse_kinematics_B.i_l +
            inverse_kinematics_B.loop_ub_m;
          inverse_kinematics_B.Tc2p[inverse_kinematics_B.jointSign] = 0.0;
          inverse_kinematics_B.Tc2p[inverse_kinematics_B.jointSign] +=
            inverse_kinematics_B.T1[inverse_kinematics_B.loop_ub_m] *
            inverse_kinematics_B.Tj_k[inverse_kinematics_B.i_l];
          inverse_kinematics_B.Tc2p[inverse_kinematics_B.jointSign] +=
            inverse_kinematics_B.T1[inverse_kinematics_B.loop_ub_m + 1] *
            inverse_kinematics_B.Tj_k[inverse_kinematics_B.i_l + 4];
          inverse_kinematics_B.Tc2p[inverse_kinematics_B.jointSign] +=
            inverse_kinematics_B.T1[inverse_kinematics_B.loop_ub_m + 2] *
            inverse_kinematics_B.Tj_k[inverse_kinematics_B.i_l + 8];
          inverse_kinematics_B.Tc2p[inverse_kinematics_B.jointSign] +=
            inverse_kinematics_B.T1[inverse_kinematics_B.loop_ub_m + 3] *
            inverse_kinematics_B.Tj_k[inverse_kinematics_B.i_l + 12];
        }
      }

      memcpy(&inverse_kinematics_B.T1[0], &inverse_kinematics_B.Tc2p[0], sizeof
             (real_T) << 4U);
    }
  }

  inverse_kinemati_emxFree_real_T(&tmp);
  inverse_kinemati_emxFree_real_T(&b);
  inverse_kinemati_emxFree_char_T(&body2Name);
  inverse_kinemati_emxFree_real_T(&kinematicPathIndices);
  for (inverse_kinematics_B.i_l = 0; inverse_kinematics_B.i_l < 3;
       inverse_kinematics_B.i_l++) {
    inverse_kinematics_B.Jac = inverse_kinematics_B.i_l << 2;
    inverse_kinematics_B.bid1 = inverse_kinematics_B.T1[inverse_kinematics_B.Jac];
    inverse_kinematics_B.Tj[6 * inverse_kinematics_B.i_l] =
      inverse_kinematics_B.bid1;
    inverse_kinematics_B.c_o = (inverse_kinematics_B.i_l + 3) * 6;
    inverse_kinematics_B.Tj[inverse_kinematics_B.c_o] = 0.0;
    inverse_kinematics_B.Tj[6 * inverse_kinematics_B.i_l + 3] = 0.0;
    inverse_kinematics_B.Tj[inverse_kinematics_B.c_o + 3] =
      inverse_kinematics_B.bid1;
    inverse_kinematics_B.bid1 = inverse_kinematics_B.T1[inverse_kinematics_B.Jac
      + 1];
    inverse_kinematics_B.Tj[6 * inverse_kinematics_B.i_l + 1] =
      inverse_kinematics_B.bid1;
    inverse_kinematics_B.Tj[inverse_kinematics_B.c_o + 1] = 0.0;
    inverse_kinematics_B.Tj[6 * inverse_kinematics_B.i_l + 4] = 0.0;
    inverse_kinematics_B.Tj[inverse_kinematics_B.c_o + 4] =
      inverse_kinematics_B.bid1;
    inverse_kinematics_B.bid1 = inverse_kinematics_B.T1[inverse_kinematics_B.Jac
      + 2];
    inverse_kinematics_B.Tj[6 * inverse_kinematics_B.i_l + 2] =
      inverse_kinematics_B.bid1;
    inverse_kinematics_B.Tj[inverse_kinematics_B.c_o + 2] = 0.0;
    inverse_kinematics_B.Tj[6 * inverse_kinematics_B.i_l + 5] = 0.0;
    inverse_kinematics_B.Tj[inverse_kinematics_B.c_o + 5] =
      inverse_kinematics_B.bid1;
  }

  inverse_kinemati_emxInit_real_T(&Jac_0, 2);
  inverse_kinematics_B.result_data_tmp = Jac_0->size[0] * Jac_0->size[1];
  Jac_0->size[0] = 6;
  Jac_0->size[1] = Jac->size[1];
  invers_emxEnsureCapacity_real_T(Jac_0, inverse_kinematics_B.result_data_tmp);
  inverse_kinematics_B.loop_ub_m = Jac->size[0] * Jac->size[1] - 1;
  if (inverse_kinematics_B.loop_ub_m >= 0) {
    memcpy(&Jac_0->data[0], &Jac->data[0], (inverse_kinematics_B.loop_ub_m + 1) *
           sizeof(real_T));
  }

  inverse_kinematics_mtimes(inverse_kinematics_B.Tj, Jac_0, Jac);
  T_size[0] = 4;
  T_size[1] = 4;
  inverse_kinemati_emxFree_real_T(&Jac_0);
  memcpy(&T_data[0], &inverse_kinematics_B.T1[0], sizeof(real_T) << 4U);
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
  } else if (!rtIsNaN(y)) {
    y = a * 1.4142135623730951;
  }

  return y;
}

static creal_T inverse_kinematics_sqrt(const creal_T x)
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
      absxi = rt_hypotd_snf(absxr, absxi * 0.5);
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
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = (rtNaN);
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    int32_T u0_0;
    int32_T u1_0;
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

static real_T inverse_kinematics_xnrm2(int32_T n, const real_T x[9], int32_T ix0)
{
  real_T scale;
  real_T y;
  int32_T kend;
  y = 0.0;
  scale = 3.3121686421112381E-170;
  kend = ix0 + n;
  for (int32_T k = ix0; k < kend; k++) {
    real_T absxk;
    absxk = fabs(x[k - 1]);
    if (absxk > scale) {
      real_T t;
      t = scale / absxk;
      y = y * t * t + 1.0;
      scale = absxk;
    } else {
      real_T t;
      t = absxk / scale;
      y += t * t;
    }
  }

  return scale * sqrt(y);
}

static real_T inverse_kinematics_xdotc(int32_T n, const real_T x[9], int32_T ix0,
  const real_T y[9], int32_T iy0)
{
  real_T d;
  d = 0.0;
  for (int32_T k = 0; k < n; k++) {
    d += x[(ix0 + k) - 1] * y[(iy0 + k) - 1];
  }

  return d;
}

static void inverse_kinematics_xaxpy(int32_T n, real_T a, int32_T ix0, const
  real_T y[9], int32_T iy0, real_T b_y[9])
{
  memcpy(&b_y[0], &y[0], 9U * sizeof(real_T));
  if (!(a == 0.0)) {
    for (int32_T k = 0; k < n; k++) {
      int32_T b_y_tmp;
      b_y_tmp = (iy0 + k) - 1;
      b_y[b_y_tmp] += b_y[(ix0 + k) - 1] * a;
    }
  }
}

static real_T inverse_kinematics_xnrm2_g(const real_T x[3], int32_T ix0)
{
  real_T scale;
  real_T y;
  y = 0.0;
  scale = 3.3121686421112381E-170;
  for (int32_T k = ix0; k <= ix0 + 1; k++) {
    real_T absxk;
    absxk = fabs(x[k - 1]);
    if (absxk > scale) {
      real_T t;
      t = scale / absxk;
      y = y * t * t + 1.0;
      scale = absxk;
    } else {
      real_T t;
      t = absxk / scale;
      y += t * t;
    }
  }

  return scale * sqrt(y);
}

static void inverse_kinematics_xaxpy_gaq(int32_T n, real_T a, const real_T x[9],
  int32_T ix0, real_T y[3], int32_T iy0)
{
  if (!(a == 0.0)) {
    for (int32_T k = 0; k < n; k++) {
      int32_T tmp;
      tmp = (iy0 + k) - 1;
      y[tmp] += x[(ix0 + k) - 1] * a;
    }
  }
}

static void inverse_kinematics_xaxpy_ga(int32_T n, real_T a, const real_T x[3],
  int32_T ix0, const real_T y[9], int32_T iy0, real_T b_y[9])
{
  memcpy(&b_y[0], &y[0], 9U * sizeof(real_T));
  if (!(a == 0.0)) {
    for (int32_T k = 0; k < n; k++) {
      int32_T b_y_tmp;
      b_y_tmp = (iy0 + k) - 1;
      b_y[b_y_tmp] += x[(ix0 + k) - 1] * a;
    }
  }
}

static void inverse_kinematics_xswap(const real_T x[9], int32_T ix0, int32_T iy0,
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

static void inverse_kinematics_xrotg(real_T a, real_T b, real_T *b_a, real_T
  *b_b, real_T *c, real_T *s)
{
  inverse_kinematics_B.roe = b;
  inverse_kinematics_B.absa = fabs(a);
  inverse_kinematics_B.absb = fabs(b);
  if (inverse_kinematics_B.absa > inverse_kinematics_B.absb) {
    inverse_kinematics_B.roe = a;
  }

  inverse_kinematics_B.scale_c = inverse_kinematics_B.absa +
    inverse_kinematics_B.absb;
  if (inverse_kinematics_B.scale_c == 0.0) {
    *s = 0.0;
    *c = 1.0;
    *b_a = 0.0;
    *b_b = 0.0;
  } else {
    inverse_kinematics_B.ads = inverse_kinematics_B.absa /
      inverse_kinematics_B.scale_c;
    inverse_kinematics_B.bds = inverse_kinematics_B.absb /
      inverse_kinematics_B.scale_c;
    *b_a = sqrt(inverse_kinematics_B.ads * inverse_kinematics_B.ads +
                inverse_kinematics_B.bds * inverse_kinematics_B.bds) *
      inverse_kinematics_B.scale_c;
    if (inverse_kinematics_B.roe < 0.0) {
      *b_a = -*b_a;
    }

    *c = a / *b_a;
    *s = b / *b_a;
    if (inverse_kinematics_B.absa > inverse_kinematics_B.absb) {
      *b_b = *s;
    } else if (*c != 0.0) {
      *b_b = 1.0 / *c;
    } else {
      *b_b = 1.0;
    }
  }
}

static void inverse_kinematics_xrot(const real_T x[9], int32_T ix0, int32_T iy0,
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

static void inverse_kinematics_svd(const real_T A[9], real_T U[9], real_T s[3],
  real_T V[9])
{
  inverse_kinematics_B.s[0] = 0.0;
  inverse_kinematics_B.e_d[0] = 0.0;
  inverse_kinematics_B.work[0] = 0.0;
  inverse_kinematics_B.s[1] = 0.0;
  inverse_kinematics_B.e_d[1] = 0.0;
  inverse_kinematics_B.work[1] = 0.0;
  inverse_kinematics_B.s[2] = 0.0;
  inverse_kinematics_B.e_d[2] = 0.0;
  inverse_kinematics_B.work[2] = 0.0;
  for (inverse_kinematics_B.m_b = 0; inverse_kinematics_B.m_b < 9;
       inverse_kinematics_B.m_b++) {
    inverse_kinematics_B.A[inverse_kinematics_B.m_b] =
      A[inverse_kinematics_B.m_b];
    U[inverse_kinematics_B.m_b] = 0.0;
    V[inverse_kinematics_B.m_b] = 0.0;
  }

  for (inverse_kinematics_B.m_b = 0; inverse_kinematics_B.m_b < 2;
       inverse_kinematics_B.m_b++) {
    inverse_kinematics_B.q_o = inverse_kinematics_B.m_b + 1;
    inverse_kinematics_B.qp1 = inverse_kinematics_B.m_b + 2;
    inverse_kinematics_B.qq_tmp = inverse_kinematics_B.m_b * 3 +
      inverse_kinematics_B.m_b;
    inverse_kinematics_B.qq = inverse_kinematics_B.qq_tmp + 1;
    inverse_kinematics_B.apply_transform = false;
    inverse_kinematics_B.nrm = inverse_kinematics_xnrm2(3 -
      inverse_kinematics_B.m_b, inverse_kinematics_B.A,
      inverse_kinematics_B.qq_tmp + 1);
    if (inverse_kinematics_B.nrm > 0.0) {
      inverse_kinematics_B.apply_transform = true;
      if (inverse_kinematics_B.A[inverse_kinematics_B.qq_tmp] < 0.0) {
        inverse_kinematics_B.s[inverse_kinematics_B.m_b] =
          -inverse_kinematics_B.nrm;
      } else {
        inverse_kinematics_B.s[inverse_kinematics_B.m_b] =
          inverse_kinematics_B.nrm;
      }

      if (fabs(inverse_kinematics_B.s[inverse_kinematics_B.m_b]) >=
          1.0020841800044864E-292) {
        inverse_kinematics_B.nrm = 1.0 /
          inverse_kinematics_B.s[inverse_kinematics_B.m_b];
        inverse_kinematics_B.b_ao = inverse_kinematics_B.qq_tmp -
          inverse_kinematics_B.m_b;
        for (inverse_kinematics_B.qjj = inverse_kinematics_B.qq;
             inverse_kinematics_B.qjj <= inverse_kinematics_B.b_ao + 3;
             inverse_kinematics_B.qjj++) {
          inverse_kinematics_B.A[inverse_kinematics_B.qjj - 1] *=
            inverse_kinematics_B.nrm;
        }
      } else {
        inverse_kinematics_B.b_ao = inverse_kinematics_B.qq_tmp -
          inverse_kinematics_B.m_b;
        for (inverse_kinematics_B.qjj = inverse_kinematics_B.qq;
             inverse_kinematics_B.qjj <= inverse_kinematics_B.b_ao + 3;
             inverse_kinematics_B.qjj++) {
          inverse_kinematics_B.A[inverse_kinematics_B.qjj - 1] /=
            inverse_kinematics_B.s[inverse_kinematics_B.m_b];
        }
      }

      inverse_kinematics_B.A[inverse_kinematics_B.qq_tmp]++;
      inverse_kinematics_B.s[inverse_kinematics_B.m_b] =
        -inverse_kinematics_B.s[inverse_kinematics_B.m_b];
    } else {
      inverse_kinematics_B.s[inverse_kinematics_B.m_b] = 0.0;
    }

    for (inverse_kinematics_B.qq = inverse_kinematics_B.qp1;
         inverse_kinematics_B.qq < 4; inverse_kinematics_B.qq++) {
      inverse_kinematics_B.qjj = ((inverse_kinematics_B.qq - 1) * 3 +
        inverse_kinematics_B.m_b) + 1;
      if (inverse_kinematics_B.apply_transform) {
        memcpy(&inverse_kinematics_B.A_m[0], &inverse_kinematics_B.A[0], 9U *
               sizeof(real_T));
        inverse_kinematics_xaxpy(3 - inverse_kinematics_B.m_b,
          -(inverse_kinematics_xdotc(3 - inverse_kinematics_B.m_b,
          inverse_kinematics_B.A, inverse_kinematics_B.qq_tmp + 1,
          inverse_kinematics_B.A, inverse_kinematics_B.qjj) /
            inverse_kinematics_B.A[inverse_kinematics_B.qq_tmp]),
          inverse_kinematics_B.qq_tmp + 1, inverse_kinematics_B.A_m,
          inverse_kinematics_B.qjj, inverse_kinematics_B.A);
      }

      inverse_kinematics_B.e_d[inverse_kinematics_B.qq - 1] =
        inverse_kinematics_B.A[inverse_kinematics_B.qjj - 1];
    }

    memcpy(&U[(inverse_kinematics_B.m_b * 3 + inverse_kinematics_B.q_o) + -1],
           &inverse_kinematics_B.A[(inverse_kinematics_B.m_b * 3 +
            inverse_kinematics_B.q_o) + -1], (-inverse_kinematics_B.q_o + 4) *
           sizeof(real_T));
    if (inverse_kinematics_B.m_b + 1 <= 1) {
      inverse_kinematics_B.nrm = inverse_kinematics_xnrm2_g
        (inverse_kinematics_B.e_d, 2);
      if (inverse_kinematics_B.nrm == 0.0) {
        inverse_kinematics_B.e_d[0] = 0.0;
      } else {
        if (inverse_kinematics_B.e_d[1] < 0.0) {
          inverse_kinematics_B.rt = -inverse_kinematics_B.nrm;
          inverse_kinematics_B.e_d[0] = -inverse_kinematics_B.nrm;
        } else {
          inverse_kinematics_B.rt = inverse_kinematics_B.nrm;
          inverse_kinematics_B.e_d[0] = inverse_kinematics_B.nrm;
        }

        if (fabs(inverse_kinematics_B.rt) >= 1.0020841800044864E-292) {
          inverse_kinematics_B.nrm = 1.0 / inverse_kinematics_B.rt;
          for (inverse_kinematics_B.qjj = inverse_kinematics_B.qp1;
               inverse_kinematics_B.qjj < 4; inverse_kinematics_B.qjj++) {
            inverse_kinematics_B.e_d[inverse_kinematics_B.qjj - 1] *=
              inverse_kinematics_B.nrm;
          }
        } else {
          for (inverse_kinematics_B.qjj = inverse_kinematics_B.qp1;
               inverse_kinematics_B.qjj < 4; inverse_kinematics_B.qjj++) {
            inverse_kinematics_B.e_d[inverse_kinematics_B.qjj - 1] /=
              inverse_kinematics_B.rt;
          }
        }

        inverse_kinematics_B.e_d[1]++;
        inverse_kinematics_B.e_d[0] = -inverse_kinematics_B.e_d[0];
        for (inverse_kinematics_B.q_o = inverse_kinematics_B.qp1;
             inverse_kinematics_B.q_o < 4; inverse_kinematics_B.q_o++) {
          inverse_kinematics_B.work[inverse_kinematics_B.q_o - 1] = 0.0;
        }

        for (inverse_kinematics_B.q_o = inverse_kinematics_B.qp1;
             inverse_kinematics_B.q_o < 4; inverse_kinematics_B.q_o++) {
          inverse_kinematics_xaxpy_gaq(2,
            inverse_kinematics_B.e_d[inverse_kinematics_B.q_o - 1],
            inverse_kinematics_B.A, 3 * (inverse_kinematics_B.q_o - 1) + 2,
            inverse_kinematics_B.work, 2);
        }

        for (inverse_kinematics_B.q_o = inverse_kinematics_B.qp1;
             inverse_kinematics_B.q_o < 4; inverse_kinematics_B.q_o++) {
          memcpy(&inverse_kinematics_B.A_m[0], &inverse_kinematics_B.A[0], 9U *
                 sizeof(real_T));
          inverse_kinematics_xaxpy_ga(2,
            -inverse_kinematics_B.e_d[inverse_kinematics_B.q_o - 1] /
            inverse_kinematics_B.e_d[1], inverse_kinematics_B.work, 2,
            inverse_kinematics_B.A_m, (inverse_kinematics_B.q_o - 1) * 3 + 2,
            inverse_kinematics_B.A);
        }
      }

      for (inverse_kinematics_B.q_o = inverse_kinematics_B.qp1;
           inverse_kinematics_B.q_o < 4; inverse_kinematics_B.q_o++) {
        V[inverse_kinematics_B.q_o - 1] =
          inverse_kinematics_B.e_d[inverse_kinematics_B.q_o - 1];
      }
    }
  }

  inverse_kinematics_B.m_b = 2;
  inverse_kinematics_B.s[2] = inverse_kinematics_B.A[8];
  inverse_kinematics_B.e_d[1] = inverse_kinematics_B.A[7];
  inverse_kinematics_B.e_d[2] = 0.0;
  U[6] = 0.0;
  U[7] = 0.0;
  U[8] = 1.0;
  for (inverse_kinematics_B.q_o = 1; inverse_kinematics_B.q_o >= 0;
       inverse_kinematics_B.q_o--) {
    inverse_kinematics_B.qq = 3 * inverse_kinematics_B.q_o +
      inverse_kinematics_B.q_o;
    if (inverse_kinematics_B.s[inverse_kinematics_B.q_o] != 0.0) {
      for (inverse_kinematics_B.qp1 = inverse_kinematics_B.q_o + 2;
           inverse_kinematics_B.qp1 < 4; inverse_kinematics_B.qp1++) {
        inverse_kinematics_B.qjj = ((inverse_kinematics_B.qp1 - 1) * 3 +
          inverse_kinematics_B.q_o) + 1;
        memcpy(&inverse_kinematics_B.A[0], &U[0], 9U * sizeof(real_T));
        inverse_kinematics_xaxpy(3 - inverse_kinematics_B.q_o,
          -(inverse_kinematics_xdotc(3 - inverse_kinematics_B.q_o, U,
          inverse_kinematics_B.qq + 1, U, inverse_kinematics_B.qjj) /
            U[inverse_kinematics_B.qq]), inverse_kinematics_B.qq + 1,
          inverse_kinematics_B.A, inverse_kinematics_B.qjj, U);
      }

      for (inverse_kinematics_B.qp1 = inverse_kinematics_B.q_o + 1;
           inverse_kinematics_B.qp1 < 4; inverse_kinematics_B.qp1++) {
        inverse_kinematics_B.qjj = (3 * inverse_kinematics_B.q_o +
          inverse_kinematics_B.qp1) - 1;
        U[inverse_kinematics_B.qjj] = -U[inverse_kinematics_B.qjj];
      }

      U[inverse_kinematics_B.qq]++;
      if (inverse_kinematics_B.q_o - 1 >= 0) {
        U[3 * inverse_kinematics_B.q_o] = 0.0;
      }
    } else {
      U[3 * inverse_kinematics_B.q_o] = 0.0;
      U[3 * inverse_kinematics_B.q_o + 1] = 0.0;
      U[3 * inverse_kinematics_B.q_o + 2] = 0.0;
      U[inverse_kinematics_B.qq] = 1.0;
    }
  }

  for (inverse_kinematics_B.q_o = 2; inverse_kinematics_B.q_o >= 0;
       inverse_kinematics_B.q_o--) {
    if ((inverse_kinematics_B.q_o + 1 <= 1) && (inverse_kinematics_B.e_d[0] !=
         0.0)) {
      memcpy(&inverse_kinematics_B.A[0], &V[0], 9U * sizeof(real_T));
      inverse_kinematics_xaxpy(2, -(inverse_kinematics_xdotc(2, V, 2, V, 5) / V
        [1]), 2, inverse_kinematics_B.A, 5, V);
      memcpy(&inverse_kinematics_B.A[0], &V[0], 9U * sizeof(real_T));
      inverse_kinematics_xaxpy(2, -(inverse_kinematics_xdotc(2, V, 2, V, 8) / V
        [1]), 2, inverse_kinematics_B.A, 8, V);
    }

    V[3 * inverse_kinematics_B.q_o] = 0.0;
    V[3 * inverse_kinematics_B.q_o + 1] = 0.0;
    V[3 * inverse_kinematics_B.q_o + 2] = 0.0;
    V[inverse_kinematics_B.q_o + 3 * inverse_kinematics_B.q_o] = 1.0;
  }

  for (inverse_kinematics_B.q_o = 0; inverse_kinematics_B.q_o < 3;
       inverse_kinematics_B.q_o++) {
    inverse_kinematics_B.ztest =
      inverse_kinematics_B.e_d[inverse_kinematics_B.q_o];
    if (inverse_kinematics_B.s[inverse_kinematics_B.q_o] != 0.0) {
      inverse_kinematics_B.rt = fabs
        (inverse_kinematics_B.s[inverse_kinematics_B.q_o]);
      inverse_kinematics_B.nrm = inverse_kinematics_B.s[inverse_kinematics_B.q_o]
        / inverse_kinematics_B.rt;
      inverse_kinematics_B.s[inverse_kinematics_B.q_o] = inverse_kinematics_B.rt;
      if (inverse_kinematics_B.q_o + 1 < 3) {
        inverse_kinematics_B.ztest /= inverse_kinematics_B.nrm;
      }

      inverse_kinematics_B.qp1 = 3 * inverse_kinematics_B.q_o;
      for (inverse_kinematics_B.qjj = inverse_kinematics_B.qp1 + 1;
           inverse_kinematics_B.qjj <= inverse_kinematics_B.qp1 + 3;
           inverse_kinematics_B.qjj++) {
        U[inverse_kinematics_B.qjj - 1] *= inverse_kinematics_B.nrm;
      }
    }

    if ((inverse_kinematics_B.q_o + 1 < 3) && (inverse_kinematics_B.ztest != 0.0))
    {
      inverse_kinematics_B.rt = fabs(inverse_kinematics_B.ztest);
      inverse_kinematics_B.nrm = inverse_kinematics_B.rt /
        inverse_kinematics_B.ztest;
      inverse_kinematics_B.ztest = inverse_kinematics_B.rt;
      inverse_kinematics_B.s[inverse_kinematics_B.q_o + 1] *=
        inverse_kinematics_B.nrm;
      inverse_kinematics_B.qp1 = (inverse_kinematics_B.q_o + 1) * 3;
      for (inverse_kinematics_B.qjj = inverse_kinematics_B.qp1 + 1;
           inverse_kinematics_B.qjj <= inverse_kinematics_B.qp1 + 3;
           inverse_kinematics_B.qjj++) {
        V[inverse_kinematics_B.qjj - 1] *= inverse_kinematics_B.nrm;
      }
    }

    inverse_kinematics_B.e_d[inverse_kinematics_B.q_o] =
      inverse_kinematics_B.ztest;
  }

  inverse_kinematics_B.qp1 = 0;
  inverse_kinematics_B.nrm = 0.0;
  inverse_kinematics_B.ztest = fabs(inverse_kinematics_B.s[0]);
  inverse_kinematics_B.rt = fabs(inverse_kinematics_B.e_d[0]);
  if ((inverse_kinematics_B.ztest >= inverse_kinematics_B.rt) || rtIsNaN
      (inverse_kinematics_B.rt)) {
    inverse_kinematics_B.rt = inverse_kinematics_B.ztest;
  }

  if ((!(inverse_kinematics_B.rt <= 0.0)) && (!rtIsNaN(inverse_kinematics_B.rt)))
  {
    inverse_kinematics_B.nrm = inverse_kinematics_B.rt;
  }

  inverse_kinematics_B.ztest = fabs(inverse_kinematics_B.s[1]);
  inverse_kinematics_B.rt = fabs(inverse_kinematics_B.e_d[1]);
  if ((inverse_kinematics_B.ztest >= inverse_kinematics_B.rt) || rtIsNaN
      (inverse_kinematics_B.rt)) {
    inverse_kinematics_B.rt = inverse_kinematics_B.ztest;
  }

  if ((!(inverse_kinematics_B.nrm >= inverse_kinematics_B.rt)) && (!rtIsNaN
       (inverse_kinematics_B.rt))) {
    inverse_kinematics_B.nrm = inverse_kinematics_B.rt;
  }

  inverse_kinematics_B.ztest = fabs(inverse_kinematics_B.s[2]);
  inverse_kinematics_B.rt = fabs(inverse_kinematics_B.e_d[2]);
  if ((inverse_kinematics_B.ztest >= inverse_kinematics_B.rt) || rtIsNaN
      (inverse_kinematics_B.rt)) {
    inverse_kinematics_B.rt = inverse_kinematics_B.ztest;
  }

  if ((!(inverse_kinematics_B.nrm >= inverse_kinematics_B.rt)) && (!rtIsNaN
       (inverse_kinematics_B.rt))) {
    inverse_kinematics_B.nrm = inverse_kinematics_B.rt;
  }

  while ((inverse_kinematics_B.m_b + 1 > 0) && (!(inverse_kinematics_B.qp1 >= 75)))
  {
    boolean_T exitg1;
    inverse_kinematics_B.q_o = inverse_kinematics_B.m_b;
    inverse_kinematics_B.qq = inverse_kinematics_B.m_b;
    exitg1 = false;
    while ((!exitg1) && (inverse_kinematics_B.qq > -1)) {
      inverse_kinematics_B.q_o = inverse_kinematics_B.qq;
      if (inverse_kinematics_B.qq == 0) {
        exitg1 = true;
      } else {
        inverse_kinematics_B.rt = fabs
          (inverse_kinematics_B.e_d[inverse_kinematics_B.qq - 1]);
        if ((inverse_kinematics_B.rt <= (fabs
              (inverse_kinematics_B.s[inverse_kinematics_B.qq - 1]) + fabs
              (inverse_kinematics_B.s[inverse_kinematics_B.qq])) *
             2.2204460492503131E-16) || (inverse_kinematics_B.rt <=
             1.0020841800044864E-292) || ((inverse_kinematics_B.qp1 > 20) &&
             (inverse_kinematics_B.rt <= 2.2204460492503131E-16 *
              inverse_kinematics_B.nrm))) {
          inverse_kinematics_B.e_d[inverse_kinematics_B.qq - 1] = 0.0;
          exitg1 = true;
        } else {
          inverse_kinematics_B.qq--;
        }
      }
    }

    if (inverse_kinematics_B.q_o == inverse_kinematics_B.m_b) {
      inverse_kinematics_B.qjj = 4;
    } else {
      inverse_kinematics_B.qq = inverse_kinematics_B.m_b + 1;
      inverse_kinematics_B.qjj = inverse_kinematics_B.m_b + 1;
      exitg1 = false;
      while ((!exitg1) && (inverse_kinematics_B.qjj >= inverse_kinematics_B.q_o))
      {
        inverse_kinematics_B.qq = inverse_kinematics_B.qjj;
        if (inverse_kinematics_B.qjj == inverse_kinematics_B.q_o) {
          exitg1 = true;
        } else {
          inverse_kinematics_B.rt = 0.0;
          if (inverse_kinematics_B.qjj < inverse_kinematics_B.m_b + 1) {
            inverse_kinematics_B.rt = fabs
              (inverse_kinematics_B.e_d[inverse_kinematics_B.qjj - 1]);
          }

          if (inverse_kinematics_B.qjj > inverse_kinematics_B.q_o + 1) {
            inverse_kinematics_B.rt += fabs
              (inverse_kinematics_B.e_d[inverse_kinematics_B.qjj - 2]);
          }

          inverse_kinematics_B.ztest = fabs
            (inverse_kinematics_B.s[inverse_kinematics_B.qjj - 1]);
          if ((inverse_kinematics_B.ztest <= 2.2204460492503131E-16 *
               inverse_kinematics_B.rt) || (inverse_kinematics_B.ztest <=
               1.0020841800044864E-292)) {
            inverse_kinematics_B.s[inverse_kinematics_B.qjj - 1] = 0.0;
            exitg1 = true;
          } else {
            inverse_kinematics_B.qjj--;
          }
        }
      }

      if (inverse_kinematics_B.qq == inverse_kinematics_B.q_o) {
        inverse_kinematics_B.qjj = 3;
      } else if (inverse_kinematics_B.m_b + 1 == inverse_kinematics_B.qq) {
        inverse_kinematics_B.qjj = 1;
      } else {
        inverse_kinematics_B.qjj = 2;
        inverse_kinematics_B.q_o = inverse_kinematics_B.qq;
      }
    }

    switch (inverse_kinematics_B.qjj) {
     case 1:
      inverse_kinematics_B.rt =
        inverse_kinematics_B.e_d[inverse_kinematics_B.m_b - 1];
      inverse_kinematics_B.e_d[inverse_kinematics_B.m_b - 1] = 0.0;
      for (inverse_kinematics_B.qq = inverse_kinematics_B.m_b;
           inverse_kinematics_B.qq >= inverse_kinematics_B.q_o + 1;
           inverse_kinematics_B.qq--) {
        inverse_kinematics_B.ztest = inverse_kinematics_B.e_d[0];
        inverse_kinematics_xrotg(inverse_kinematics_B.s[inverse_kinematics_B.qq
          - 1], inverse_kinematics_B.rt,
          &inverse_kinematics_B.s[inverse_kinematics_B.qq - 1],
          &inverse_kinematics_B.rt, &inverse_kinematics_B.sqds,
          &inverse_kinematics_B.b_c);
        if (inverse_kinematics_B.qq > inverse_kinematics_B.q_o + 1) {
          inverse_kinematics_B.rt = -inverse_kinematics_B.b_c *
            inverse_kinematics_B.e_d[0];
          inverse_kinematics_B.ztest = inverse_kinematics_B.e_d[0] *
            inverse_kinematics_B.sqds;
        }

        memcpy(&inverse_kinematics_B.A[0], &V[0], 9U * sizeof(real_T));
        inverse_kinematics_xrot(inverse_kinematics_B.A, (inverse_kinematics_B.qq
          - 1) * 3 + 1, 3 * inverse_kinematics_B.m_b + 1,
          inverse_kinematics_B.sqds, inverse_kinematics_B.b_c, V);
        inverse_kinematics_B.e_d[0] = inverse_kinematics_B.ztest;
      }
      break;

     case 2:
      inverse_kinematics_B.rt =
        inverse_kinematics_B.e_d[inverse_kinematics_B.q_o - 1];
      inverse_kinematics_B.e_d[inverse_kinematics_B.q_o - 1] = 0.0;
      for (inverse_kinematics_B.qq = inverse_kinematics_B.q_o + 1;
           inverse_kinematics_B.qq <= inverse_kinematics_B.m_b + 1;
           inverse_kinematics_B.qq++) {
        inverse_kinematics_xrotg(inverse_kinematics_B.s[inverse_kinematics_B.qq
          - 1], inverse_kinematics_B.rt,
          &inverse_kinematics_B.s[inverse_kinematics_B.qq - 1],
          &inverse_kinematics_B.ztest, &inverse_kinematics_B.sqds,
          &inverse_kinematics_B.b_c);
        inverse_kinematics_B.ztest =
          inverse_kinematics_B.e_d[inverse_kinematics_B.qq - 1];
        inverse_kinematics_B.rt = inverse_kinematics_B.ztest *
          -inverse_kinematics_B.b_c;
        inverse_kinematics_B.e_d[inverse_kinematics_B.qq - 1] =
          inverse_kinematics_B.ztest * inverse_kinematics_B.sqds;
        memcpy(&inverse_kinematics_B.A[0], &U[0], 9U * sizeof(real_T));
        inverse_kinematics_xrot(inverse_kinematics_B.A, (inverse_kinematics_B.qq
          - 1) * 3 + 1, (inverse_kinematics_B.q_o - 1) * 3 + 1,
          inverse_kinematics_B.sqds, inverse_kinematics_B.b_c, U);
      }
      break;

     case 3:
      inverse_kinematics_B.ztest = fabs
        (inverse_kinematics_B.s[inverse_kinematics_B.m_b]);
      inverse_kinematics_B.sqds =
        inverse_kinematics_B.s[inverse_kinematics_B.m_b - 1];
      inverse_kinematics_B.rt = fabs(inverse_kinematics_B.sqds);
      if ((inverse_kinematics_B.ztest >= inverse_kinematics_B.rt) || rtIsNaN
          (inverse_kinematics_B.rt)) {
        inverse_kinematics_B.rt = inverse_kinematics_B.ztest;
      }

      inverse_kinematics_B.b_c =
        inverse_kinematics_B.e_d[inverse_kinematics_B.m_b - 1];
      inverse_kinematics_B.ztest = fabs(inverse_kinematics_B.b_c);
      if ((inverse_kinematics_B.rt >= inverse_kinematics_B.ztest) || rtIsNaN
          (inverse_kinematics_B.ztest)) {
        inverse_kinematics_B.ztest = inverse_kinematics_B.rt;
      }

      inverse_kinematics_B.rt = fabs
        (inverse_kinematics_B.s[inverse_kinematics_B.q_o]);
      if ((inverse_kinematics_B.ztest >= inverse_kinematics_B.rt) || rtIsNaN
          (inverse_kinematics_B.rt)) {
        inverse_kinematics_B.rt = inverse_kinematics_B.ztest;
      }

      inverse_kinematics_B.ztest = fabs
        (inverse_kinematics_B.e_d[inverse_kinematics_B.q_o]);
      if ((inverse_kinematics_B.rt >= inverse_kinematics_B.ztest) || rtIsNaN
          (inverse_kinematics_B.ztest)) {
        inverse_kinematics_B.ztest = inverse_kinematics_B.rt;
      }

      inverse_kinematics_B.rt = inverse_kinematics_B.s[inverse_kinematics_B.m_b]
        / inverse_kinematics_B.ztest;
      inverse_kinematics_B.smm1 = inverse_kinematics_B.sqds /
        inverse_kinematics_B.ztest;
      inverse_kinematics_B.emm1 = inverse_kinematics_B.b_c /
        inverse_kinematics_B.ztest;
      inverse_kinematics_B.sqds =
        inverse_kinematics_B.s[inverse_kinematics_B.q_o] /
        inverse_kinematics_B.ztest;
      inverse_kinematics_B.b_c = ((inverse_kinematics_B.smm1 +
        inverse_kinematics_B.rt) * (inverse_kinematics_B.smm1 -
        inverse_kinematics_B.rt) + inverse_kinematics_B.emm1 *
        inverse_kinematics_B.emm1) / 2.0;
      inverse_kinematics_B.smm1 = inverse_kinematics_B.rt *
        inverse_kinematics_B.emm1;
      inverse_kinematics_B.smm1 *= inverse_kinematics_B.smm1;
      if ((inverse_kinematics_B.b_c != 0.0) || (inverse_kinematics_B.smm1 != 0.0))
      {
        inverse_kinematics_B.emm1 = sqrt(inverse_kinematics_B.b_c *
          inverse_kinematics_B.b_c + inverse_kinematics_B.smm1);
        if (inverse_kinematics_B.b_c < 0.0) {
          inverse_kinematics_B.emm1 = -inverse_kinematics_B.emm1;
        }

        inverse_kinematics_B.emm1 = inverse_kinematics_B.smm1 /
          (inverse_kinematics_B.b_c + inverse_kinematics_B.emm1);
      } else {
        inverse_kinematics_B.emm1 = 0.0;
      }

      inverse_kinematics_B.rt = (inverse_kinematics_B.sqds +
        inverse_kinematics_B.rt) * (inverse_kinematics_B.sqds -
        inverse_kinematics_B.rt) + inverse_kinematics_B.emm1;
      inverse_kinematics_B.sqds *=
        inverse_kinematics_B.e_d[inverse_kinematics_B.q_o] /
        inverse_kinematics_B.ztest;
      for (inverse_kinematics_B.qq_tmp = inverse_kinematics_B.q_o + 1;
           inverse_kinematics_B.qq_tmp <= inverse_kinematics_B.m_b;
           inverse_kinematics_B.qq_tmp++) {
        inverse_kinematics_xrotg(inverse_kinematics_B.rt,
          inverse_kinematics_B.sqds, &inverse_kinematics_B.ztest,
          &inverse_kinematics_B.emm1, &inverse_kinematics_B.b_c,
          &inverse_kinematics_B.smm1);
        if (inverse_kinematics_B.qq_tmp > inverse_kinematics_B.q_o + 1) {
          inverse_kinematics_B.e_d[0] = inverse_kinematics_B.ztest;
        }

        inverse_kinematics_B.ztest =
          inverse_kinematics_B.e_d[inverse_kinematics_B.qq_tmp - 1];
        inverse_kinematics_B.rt =
          inverse_kinematics_B.s[inverse_kinematics_B.qq_tmp - 1];
        inverse_kinematics_B.e_d[inverse_kinematics_B.qq_tmp - 1] =
          inverse_kinematics_B.ztest * inverse_kinematics_B.b_c -
          inverse_kinematics_B.rt * inverse_kinematics_B.smm1;
        inverse_kinematics_B.sqds = inverse_kinematics_B.smm1 *
          inverse_kinematics_B.s[inverse_kinematics_B.qq_tmp];
        inverse_kinematics_B.s[inverse_kinematics_B.qq_tmp] *=
          inverse_kinematics_B.b_c;
        inverse_kinematics_B.qq = (inverse_kinematics_B.qq_tmp - 1) * 3 + 1;
        inverse_kinematics_B.qjj = 3 * inverse_kinematics_B.qq_tmp + 1;
        memcpy(&inverse_kinematics_B.A[0], &V[0], 9U * sizeof(real_T));
        inverse_kinematics_xrot(inverse_kinematics_B.A, inverse_kinematics_B.qq,
          inverse_kinematics_B.qjj, inverse_kinematics_B.b_c,
          inverse_kinematics_B.smm1, V);
        inverse_kinematics_xrotg(inverse_kinematics_B.rt *
          inverse_kinematics_B.b_c + inverse_kinematics_B.ztest *
          inverse_kinematics_B.smm1, inverse_kinematics_B.sqds,
          &inverse_kinematics_B.s[inverse_kinematics_B.qq_tmp - 1],
          &inverse_kinematics_B.a__3, &inverse_kinematics_B.emm1,
          &inverse_kinematics_B.d_sn);
        inverse_kinematics_B.rt =
          inverse_kinematics_B.e_d[inverse_kinematics_B.qq_tmp - 1] *
          inverse_kinematics_B.emm1 + inverse_kinematics_B.d_sn *
          inverse_kinematics_B.s[inverse_kinematics_B.qq_tmp];
        inverse_kinematics_B.s[inverse_kinematics_B.qq_tmp] =
          inverse_kinematics_B.e_d[inverse_kinematics_B.qq_tmp - 1] *
          -inverse_kinematics_B.d_sn + inverse_kinematics_B.emm1 *
          inverse_kinematics_B.s[inverse_kinematics_B.qq_tmp];
        inverse_kinematics_B.sqds = inverse_kinematics_B.d_sn *
          inverse_kinematics_B.e_d[inverse_kinematics_B.qq_tmp];
        inverse_kinematics_B.e_d[inverse_kinematics_B.qq_tmp] *=
          inverse_kinematics_B.emm1;
        memcpy(&inverse_kinematics_B.A[0], &U[0], 9U * sizeof(real_T));
        inverse_kinematics_xrot(inverse_kinematics_B.A, inverse_kinematics_B.qq,
          inverse_kinematics_B.qjj, inverse_kinematics_B.emm1,
          inverse_kinematics_B.d_sn, U);
      }

      inverse_kinematics_B.e_d[inverse_kinematics_B.m_b - 1] =
        inverse_kinematics_B.rt;
      inverse_kinematics_B.qp1++;
      break;

     default:
      if (inverse_kinematics_B.s[inverse_kinematics_B.q_o] < 0.0) {
        inverse_kinematics_B.s[inverse_kinematics_B.q_o] =
          -inverse_kinematics_B.s[inverse_kinematics_B.q_o];
        inverse_kinematics_B.qp1 = 3 * inverse_kinematics_B.q_o;
        for (inverse_kinematics_B.qjj = inverse_kinematics_B.qp1 + 1;
             inverse_kinematics_B.qjj <= inverse_kinematics_B.qp1 + 3;
             inverse_kinematics_B.qjj++) {
          V[inverse_kinematics_B.qjj - 1] = -V[inverse_kinematics_B.qjj - 1];
        }
      }

      inverse_kinematics_B.qp1 = inverse_kinematics_B.q_o + 1;
      while ((inverse_kinematics_B.q_o + 1 < 3) &&
             (inverse_kinematics_B.s[inverse_kinematics_B.q_o] <
              inverse_kinematics_B.s[inverse_kinematics_B.qp1])) {
        inverse_kinematics_B.rt =
          inverse_kinematics_B.s[inverse_kinematics_B.q_o];
        inverse_kinematics_B.s[inverse_kinematics_B.q_o] =
          inverse_kinematics_B.s[inverse_kinematics_B.qp1];
        inverse_kinematics_B.s[inverse_kinematics_B.qp1] =
          inverse_kinematics_B.rt;
        inverse_kinematics_B.qq = 3 * inverse_kinematics_B.q_o + 1;
        inverse_kinematics_B.qjj = (inverse_kinematics_B.q_o + 1) * 3 + 1;
        memcpy(&inverse_kinematics_B.A[0], &V[0], 9U * sizeof(real_T));
        inverse_kinematics_xswap(inverse_kinematics_B.A, inverse_kinematics_B.qq,
          inverse_kinematics_B.qjj, V);
        memcpy(&inverse_kinematics_B.A[0], &U[0], 9U * sizeof(real_T));
        inverse_kinematics_xswap(inverse_kinematics_B.A, inverse_kinematics_B.qq,
          inverse_kinematics_B.qjj, U);
        inverse_kinematics_B.q_o = inverse_kinematics_B.qp1;
        inverse_kinematics_B.qp1++;
      }

      inverse_kinematics_B.qp1 = 0;
      inverse_kinematics_B.m_b--;
      break;
    }
  }

  s[0] = inverse_kinematics_B.s[0];
  s[1] = inverse_kinematics_B.s[1];
  s[2] = inverse_kinematics_B.s[2];
}

static void inverse_kin_IKHelpers_poseError(const real_T Td[16], const real_T
  T_data[], const int32_T T_size[2], real_T errorvec[6])
{
  boolean_T exitg1;
  inverse_kinematics_B.y_tmp_m[0] = 1;
  inverse_kinematics_B.y_tmp_m[1] = 2;
  inverse_kinematics_B.y_tmp_m[2] = 3;
  for (inverse_kinematics_B.b_k_j = 0; inverse_kinematics_B.b_k_j < 3;
       inverse_kinematics_B.b_k_j++) {
    inverse_kinematics_B.y_tmp_c =
      inverse_kinematics_B.y_tmp_m[inverse_kinematics_B.b_k_j];
    inverse_kinematics_B.T[3 * inverse_kinematics_B.b_k_j] =
      T_data[inverse_kinematics_B.y_tmp_c - 1];
    inverse_kinematics_B.T_tmp = 3 * inverse_kinematics_B.b_k_j + 1;
    inverse_kinematics_B.T[inverse_kinematics_B.T_tmp] = T_data
      [(inverse_kinematics_B.y_tmp_c + T_size[0]) - 1];
    inverse_kinematics_B.T_tmp_e = 3 * inverse_kinematics_B.b_k_j + 2;
    inverse_kinematics_B.T[inverse_kinematics_B.T_tmp_e] = T_data[((T_size[0] <<
      1) + inverse_kinematics_B.y_tmp_c) - 1];
    for (inverse_kinematics_B.i3 = 0; inverse_kinematics_B.i3 < 3;
         inverse_kinematics_B.i3++) {
      inverse_kinematics_B.y_tmp = 3 * inverse_kinematics_B.b_k_j +
        inverse_kinematics_B.i3;
      inverse_kinematics_B.y[inverse_kinematics_B.y_tmp] = 0.0;
      inverse_kinematics_B.y[inverse_kinematics_B.y_tmp] +=
        inverse_kinematics_B.T[3 * inverse_kinematics_B.b_k_j] *
        Td[inverse_kinematics_B.i3];
      inverse_kinematics_B.y[inverse_kinematics_B.y_tmp] +=
        Td[inverse_kinematics_B.i3 + 4] *
        inverse_kinematics_B.T[inverse_kinematics_B.T_tmp];
      inverse_kinematics_B.y[inverse_kinematics_B.y_tmp] +=
        Td[inverse_kinematics_B.i3 + 8] *
        inverse_kinematics_B.T[inverse_kinematics_B.T_tmp_e];
    }
  }

  inverse_kinematics_B.u.re = (((inverse_kinematics_B.y[0] +
    inverse_kinematics_B.y[4]) + inverse_kinematics_B.y[8]) - 1.0) * 0.5;
  if (!(fabs(inverse_kinematics_B.u.re) > 1.0)) {
    inverse_kinematics_B.v_o.re = acos(inverse_kinematics_B.u.re);
  } else {
    inverse_kinematics_B.u_b.re = inverse_kinematics_B.u.re + 1.0;
    inverse_kinematics_B.u_b.im = 0.0;
    inverse_kinematics_B.dc.re = 1.0 - inverse_kinematics_B.u.re;
    inverse_kinematics_B.dc.im = 0.0;
    inverse_kinematics_B.v_o.re = 2.0 * rt_atan2d_snf((inverse_kinematics_sqrt
      (inverse_kinematics_B.dc)).re, (inverse_kinematics_sqrt
      (inverse_kinematics_B.u_b)).re);
  }

  inverse_kinematics_B.a_h = 2.0 * sin(inverse_kinematics_B.v_o.re);
  inverse_kinematics_B.v_l[0] = (inverse_kinematics_B.y[5] -
    inverse_kinematics_B.y[7]) / inverse_kinematics_B.a_h;
  inverse_kinematics_B.v_l[1] = (inverse_kinematics_B.y[6] -
    inverse_kinematics_B.y[2]) / inverse_kinematics_B.a_h;
  inverse_kinematics_B.v_l[2] = (inverse_kinematics_B.y[1] -
    inverse_kinematics_B.y[3]) / inverse_kinematics_B.a_h;
  if (rtIsNaN(inverse_kinematics_B.v_o.re) || rtIsInf
      (inverse_kinematics_B.v_o.re)) {
    inverse_kinematics_B.a_h = (rtNaN);
  } else if (inverse_kinematics_B.v_o.re == 0.0) {
    inverse_kinematics_B.a_h = 0.0;
  } else {
    inverse_kinematics_B.a_h = fmod(inverse_kinematics_B.v_o.re,
      3.1415926535897931);
    inverse_kinematics_B.rEQ0 = (inverse_kinematics_B.a_h == 0.0);
    if (!inverse_kinematics_B.rEQ0) {
      inverse_kinematics_B.q = fabs(inverse_kinematics_B.v_o.re /
        3.1415926535897931);
      inverse_kinematics_B.rEQ0 = !(fabs(inverse_kinematics_B.q - floor
        (inverse_kinematics_B.q + 0.5)) > 2.2204460492503131E-16 *
        inverse_kinematics_B.q);
    }

    if (inverse_kinematics_B.rEQ0) {
      inverse_kinematics_B.a_h = 0.0;
    } else if (inverse_kinematics_B.v_o.re < 0.0) {
      inverse_kinematics_B.a_h += 3.1415926535897931;
    }
  }

  inverse_kinematics_B.rEQ0 = (inverse_kinematics_B.a_h == 0.0);
  inverse_kinematics_B.e_b = true;
  inverse_kinematics_B.b_k_j = 0;
  exitg1 = false;
  while ((!exitg1) && (inverse_kinematics_B.b_k_j < 3)) {
    if (!(inverse_kinematics_B.v_l[inverse_kinematics_B.b_k_j] == 0.0)) {
      inverse_kinematics_B.e_b = false;
      exitg1 = true;
    } else {
      inverse_kinematics_B.b_k_j++;
    }
  }

  if (inverse_kinematics_B.rEQ0 || inverse_kinematics_B.e_b) {
    inverse_kinematics_B.T_tmp = (inverse_kinematics_B.rEQ0 ||
      inverse_kinematics_B.e_b) * 3 - 1;
    if (inverse_kinematics_B.T_tmp >= 0) {
      memset(&inverse_kinematics_B.vspecial_data[0], 0,
             (inverse_kinematics_B.T_tmp + 1) * sizeof(real_T));
    }

    inverse_kinematics_B.T_tmp = (inverse_kinematics_B.rEQ0 ||
      inverse_kinematics_B.e_b) - 1;
    for (inverse_kinematics_B.T_tmp_e = 0; inverse_kinematics_B.T_tmp_e <=
         inverse_kinematics_B.T_tmp; inverse_kinematics_B.T_tmp_e++) {
      memset(&inverse_kinematics_B.T[0], 0, 9U * sizeof(real_T));
      inverse_kinematics_B.T[0] = 1.0;
      inverse_kinematics_B.T[4] = 1.0;
      inverse_kinematics_B.T[8] = 1.0;
      for (inverse_kinematics_B.b_k_j = 0; inverse_kinematics_B.b_k_j < 9;
           inverse_kinematics_B.b_k_j++) {
        inverse_kinematics_B.T[inverse_kinematics_B.b_k_j] -=
          inverse_kinematics_B.y[inverse_kinematics_B.b_k_j];
      }

      inverse_kinematics_B.x_c = true;
      for (inverse_kinematics_B.b_k_j = 0; inverse_kinematics_B.b_k_j < 9;
           inverse_kinematics_B.b_k_j++) {
        if (inverse_kinematics_B.x_c) {
          inverse_kinematics_B.a_h =
            inverse_kinematics_B.T[inverse_kinematics_B.b_k_j];
          if ((!rtIsInf(inverse_kinematics_B.a_h)) && (!rtIsNaN
               (inverse_kinematics_B.a_h))) {
          } else {
            inverse_kinematics_B.x_c = false;
          }
        } else {
          inverse_kinematics_B.x_c = false;
        }
      }

      if (inverse_kinematics_B.x_c) {
        inverse_kinematics_svd(inverse_kinematics_B.T, inverse_kinematics_B.b_U,
          inverse_kinematics_B.v_d, inverse_kinematics_B.V);
      } else {
        for (inverse_kinematics_B.b_k_j = 0; inverse_kinematics_B.b_k_j < 9;
             inverse_kinematics_B.b_k_j++) {
          inverse_kinematics_B.V[inverse_kinematics_B.b_k_j] = (rtNaN);
        }
      }

      inverse_kinematics_B.vspecial_data[0] = inverse_kinematics_B.V[6];
      inverse_kinematics_B.vspecial_data[1] = inverse_kinematics_B.V[7];
      inverse_kinematics_B.vspecial_data[2] = inverse_kinematics_B.V[8];
    }

    inverse_kinematics_B.b_k_j = 0;
    if (inverse_kinematics_B.rEQ0 || inverse_kinematics_B.e_b) {
      for (inverse_kinematics_B.T_tmp = 0; inverse_kinematics_B.T_tmp < 1;
           inverse_kinematics_B.T_tmp++) {
        inverse_kinematics_B.b_k_j++;
      }
    }

    if (inverse_kinematics_B.b_k_j - 1 >= 0) {
      inverse_kinematics_B.v_l[0] = inverse_kinematics_B.vspecial_data[0];
      inverse_kinematics_B.v_l[1] = inverse_kinematics_B.vspecial_data[1];
      inverse_kinematics_B.v_l[2] = inverse_kinematics_B.vspecial_data[2];
    }
  }

  inverse_kinematics_B.v_d[0] = inverse_kinematics_B.v_l[0];
  inverse_kinematics_B.v_d[1] = inverse_kinematics_B.v_l[1];
  inverse_kinematics_B.v_d[2] = inverse_kinematics_B.v_l[2];
  inverse_kinematics_B.a_h = 1.0 / sqrt((inverse_kinematics_B.v_l[0] *
    inverse_kinematics_B.v_l[0] + inverse_kinematics_B.v_l[1] *
    inverse_kinematics_B.v_l[1]) + inverse_kinematics_B.v_l[2] *
    inverse_kinematics_B.v_l[2]);
  errorvec[0] = inverse_kinematics_B.v_d[0] * inverse_kinematics_B.a_h *
    inverse_kinematics_B.v_o.re;
  errorvec[3] = Td[12] - T_data[T_size[0] * 3];
  errorvec[1] = inverse_kinematics_B.v_d[1] * inverse_kinematics_B.a_h *
    inverse_kinematics_B.v_o.re;
  errorvec[4] = Td[13] - T_data[T_size[0] * 3 + 1];
  errorvec[2] = inverse_kinematics_B.v_d[2] * inverse_kinematics_B.a_h *
    inverse_kinematics_B.v_o.re;
  errorvec[5] = Td[14] - T_data[T_size[0] * 3 + 2];
}

static void inverse_kinematics_mtimes_g(const real_T A[6], const
  emxArray_real_T_inverse_kinem_T *B, emxArray_real_T_inverse_kinem_T *C)
{
  int32_T b_j;
  int32_T n;
  n = B->size[1] - 1;
  b_j = C->size[0] * C->size[1];
  C->size[0] = 1;
  C->size[1] = B->size[1];
  invers_emxEnsureCapacity_real_T(C, b_j);
  for (b_j = 0; b_j <= n; b_j++) {
    real_T s;
    int32_T boffset;
    boffset = b_j * 6 - 1;
    s = 0.0;
    for (int32_T b_k = 0; b_k < 6; b_k++) {
      s += B->data[(boffset + b_k) + 1] * A[b_k];
    }

    C->data[b_j] = s;
  }
}

static void inverse_kinem_emxInit_boolean_T(emxArray_boolean_T_inverse_ki_T
  **pEmxArray, int32_T numDimensions)
{
  emxArray_boolean_T_inverse_ki_T *emxArray;
  *pEmxArray = static_cast<emxArray_boolean_T_inverse_ki_T *>(malloc(sizeof
    (emxArray_boolean_T_inverse_ki_T)));
  emxArray = *pEmxArray;
  emxArray->data = static_cast<boolean_T *>(NULL);
  emxArray->numDimensions = numDimensions;
  emxArray->size = static_cast<int32_T *>(malloc(sizeof(int32_T) * numDimensions));
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (int32_T i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

static real_T inverse_kinematics_norm_g(const real_T x[6])
{
  real_T scale;
  real_T y;
  y = 0.0;
  scale = 3.3121686421112381E-170;
  for (int32_T b_k = 0; b_k < 6; b_k++) {
    real_T absxk;
    absxk = fabs(x[b_k]);
    if (absxk > scale) {
      real_T t;
      t = scale / absxk;
      y = y * t * t + 1.0;
      scale = absxk;
    } else {
      real_T t;
      t = absxk / scale;
      y += t * t;
    }
  }

  return scale * sqrt(y);
}

static void inverse_kinematics_minus_g(emxArray_real_T_inverse_kinem_T *in1,
  const emxArray_real_T_inverse_kinem_T *in2)
{
  emxArray_real_T_inverse_kinem_T *in2_0;
  int32_T i;
  int32_T loop_ub;
  int32_T stride_0_0;
  int32_T stride_1_0;
  inverse_kinemati_emxInit_real_T(&in2_0, 1);
  i = in2_0->size[0];
  in2_0->size[0] = in1->size[0] == 1 ? in2->size[0] : in1->size[0];
  invers_emxEnsureCapacity_real_T(in2_0, i);
  stride_0_0 = (in2->size[0] != 1);
  stride_1_0 = (in1->size[0] != 1);
  loop_ub = in1->size[0] == 1 ? in2->size[0] : in1->size[0];
  for (i = 0; i < loop_ub; i++) {
    in2_0->data[i] = in2->data[i * stride_0_0] - in1->data[i * stride_1_0];
  }

  i = in1->size[0];
  in1->size[0] = in2_0->size[0];
  invers_emxEnsureCapacity_real_T(in1, i);
  loop_ub = in2_0->size[0];
  if (loop_ub - 1 >= 0) {
    memcpy(&in1->data[0], &in2_0->data[0], loop_ub * sizeof(real_T));
  }

  inverse_kinemati_emxFree_real_T(&in2_0);
}

static void inv_emxEnsureCapacity_boolean_T(emxArray_boolean_T_inverse_ki_T
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

    emxArray->data = static_cast<boolean_T *>(newData);
    emxArray->allocatedSize = i;
    emxArray->canFreeData = true;
  }
}

static real_T inverse_kinematics_toc(real_T tstart_tv_sec, real_T tstart_tv_nsec)
{
  coderTimespec b_timespec;
  if (!inverse_kinematics_DW.method_not_empty) {
    inverse_kinematics_DW.method_not_empty = true;
    coderInitTimeFunctions(&inverse_kinematics_DW.freq);
  }

  coderTimeClockGettimeMonotonic(&b_timespec, inverse_kinematics_DW.freq);
  return (b_timespec.tv_nsec - tstart_tv_nsec) / 1.0E+9 + (b_timespec.tv_sec -
    tstart_tv_sec);
}

static void inverse_kinematics_mldivide(const real_T A[16], const
  emxArray_real_T_inverse_kinem_T *B, real_T Y_data[], int32_T *Y_size)
{
  memcpy(&inverse_kinematics_B.c_x[0], &A[0], sizeof(real_T) << 4U);
  inverse_kinematics_B.b_ipiv[0] = 1;
  inverse_kinematics_B.b_ipiv[1] = 2;
  inverse_kinematics_B.b_ipiv[2] = 3;
  for (inverse_kinematics_B.kAcol = 0; inverse_kinematics_B.kAcol < 3;
       inverse_kinematics_B.kAcol++) {
    inverse_kinematics_B.c_g = inverse_kinematics_B.kAcol * 5 + 2;
    inverse_kinematics_B.jj = inverse_kinematics_B.kAcol * 5;
    inverse_kinematics_B.c_e = 4 - inverse_kinematics_B.kAcol;
    inverse_kinematics_B.a_f = 1;
    inverse_kinematics_B.smax = fabs
      (inverse_kinematics_B.c_x[inverse_kinematics_B.jj]);
    for (inverse_kinematics_B.jA = 2; inverse_kinematics_B.jA <=
         inverse_kinematics_B.c_e; inverse_kinematics_B.jA++) {
      inverse_kinematics_B.s_p = fabs(inverse_kinematics_B.c_x
        [(inverse_kinematics_B.c_g + inverse_kinematics_B.jA) - 3]);
      if (inverse_kinematics_B.s_p > inverse_kinematics_B.smax) {
        inverse_kinematics_B.a_f = inverse_kinematics_B.jA;
        inverse_kinematics_B.smax = inverse_kinematics_B.s_p;
      }
    }

    if (inverse_kinematics_B.c_x[(inverse_kinematics_B.c_g +
         inverse_kinematics_B.a_f) - 3] != 0.0) {
      if (inverse_kinematics_B.a_f - 1 != 0) {
        inverse_kinematics_B.a_f += inverse_kinematics_B.kAcol;
        inverse_kinematics_B.b_ipiv[inverse_kinematics_B.kAcol] =
          static_cast<int8_T>(inverse_kinematics_B.a_f);
        inverse_kinematics_B.smax =
          inverse_kinematics_B.c_x[inverse_kinematics_B.kAcol];
        inverse_kinematics_B.c_x[inverse_kinematics_B.kAcol] =
          inverse_kinematics_B.c_x[inverse_kinematics_B.a_f - 1];
        inverse_kinematics_B.c_x[inverse_kinematics_B.a_f - 1] =
          inverse_kinematics_B.smax;
        inverse_kinematics_B.smax =
          inverse_kinematics_B.c_x[inverse_kinematics_B.kAcol + 4];
        inverse_kinematics_B.c_x[inverse_kinematics_B.kAcol + 4] =
          inverse_kinematics_B.c_x[inverse_kinematics_B.a_f + 3];
        inverse_kinematics_B.c_x[inverse_kinematics_B.a_f + 3] =
          inverse_kinematics_B.smax;
        inverse_kinematics_B.smax =
          inverse_kinematics_B.c_x[inverse_kinematics_B.kAcol + 8];
        inverse_kinematics_B.c_x[inverse_kinematics_B.kAcol + 8] =
          inverse_kinematics_B.c_x[inverse_kinematics_B.a_f + 7];
        inverse_kinematics_B.c_x[inverse_kinematics_B.a_f + 7] =
          inverse_kinematics_B.smax;
        inverse_kinematics_B.smax =
          inverse_kinematics_B.c_x[inverse_kinematics_B.kAcol + 12];
        inverse_kinematics_B.c_x[inverse_kinematics_B.kAcol + 12] =
          inverse_kinematics_B.c_x[inverse_kinematics_B.a_f + 11];
        inverse_kinematics_B.c_x[inverse_kinematics_B.a_f + 11] =
          inverse_kinematics_B.smax;
      }

      inverse_kinematics_B.a_f = inverse_kinematics_B.c_g -
        inverse_kinematics_B.kAcol;
      for (inverse_kinematics_B.c_e = inverse_kinematics_B.c_g;
           inverse_kinematics_B.c_e <= inverse_kinematics_B.a_f + 2;
           inverse_kinematics_B.c_e++) {
        inverse_kinematics_B.c_x[inverse_kinematics_B.c_e - 1] /=
          inverse_kinematics_B.c_x[inverse_kinematics_B.jj];
      }
    }

    inverse_kinematics_B.c_e = 3 - inverse_kinematics_B.kAcol;
    inverse_kinematics_B.jA = inverse_kinematics_B.jj;
    for (inverse_kinematics_B.j_h = 0; inverse_kinematics_B.j_h <
         inverse_kinematics_B.c_e; inverse_kinematics_B.j_h++) {
      inverse_kinematics_B.smax = inverse_kinematics_B.c_x
        [((inverse_kinematics_B.j_h << 2) + inverse_kinematics_B.jj) + 4];
      if (inverse_kinematics_B.smax != 0.0) {
        inverse_kinematics_B.a_f = inverse_kinematics_B.jA + 6;
        inverse_kinematics_B.c_ei = inverse_kinematics_B.jA -
          inverse_kinematics_B.kAcol;
        for (inverse_kinematics_B.ijA = inverse_kinematics_B.a_f;
             inverse_kinematics_B.ijA <= inverse_kinematics_B.c_ei + 8;
             inverse_kinematics_B.ijA++) {
          inverse_kinematics_B.c_x[inverse_kinematics_B.ijA - 1] +=
            inverse_kinematics_B.c_x[((inverse_kinematics_B.c_g +
            inverse_kinematics_B.ijA) - inverse_kinematics_B.jA) - 7] *
            -inverse_kinematics_B.smax;
        }
      }

      inverse_kinematics_B.jA += 4;
    }
  }

  *Y_size = B->size[0];
  inverse_kinematics_B.c_g = B->size[0];
  if (inverse_kinematics_B.c_g - 1 >= 0) {
    memcpy(&Y_data[0], &B->data[0], inverse_kinematics_B.c_g * sizeof(real_T));
  }

  if (inverse_kinematics_B.b_ipiv[0] != 1) {
    inverse_kinematics_B.smax = Y_data[0];
    Y_data[0] = Y_data[inverse_kinematics_B.b_ipiv[0] - 1];
    Y_data[inverse_kinematics_B.b_ipiv[0] - 1] = inverse_kinematics_B.smax;
  }

  if (inverse_kinematics_B.b_ipiv[1] != 2) {
    inverse_kinematics_B.smax = Y_data[1];
    Y_data[1] = Y_data[inverse_kinematics_B.b_ipiv[1] - 1];
    Y_data[inverse_kinematics_B.b_ipiv[1] - 1] = inverse_kinematics_B.smax;
  }

  if (inverse_kinematics_B.b_ipiv[2] != 3) {
    inverse_kinematics_B.smax = Y_data[2];
    Y_data[2] = Y_data[inverse_kinematics_B.b_ipiv[2] - 1];
    Y_data[inverse_kinematics_B.b_ipiv[2] - 1] = inverse_kinematics_B.smax;
  }

  for (inverse_kinematics_B.c_g = 0; inverse_kinematics_B.c_g < 4;
       inverse_kinematics_B.c_g++) {
    inverse_kinematics_B.kAcol = (inverse_kinematics_B.c_g << 2) - 1;
    if (Y_data[inverse_kinematics_B.c_g] != 0.0) {
      for (inverse_kinematics_B.c_e = inverse_kinematics_B.c_g + 2;
           inverse_kinematics_B.c_e < 5; inverse_kinematics_B.c_e++) {
        Y_data[inverse_kinematics_B.c_e - 1] -=
          inverse_kinematics_B.c_x[inverse_kinematics_B.c_e +
          inverse_kinematics_B.kAcol] * Y_data[inverse_kinematics_B.c_g];
      }
    }
  }

  for (inverse_kinematics_B.jA = 3; inverse_kinematics_B.jA >= 0;
       inverse_kinematics_B.jA--) {
    inverse_kinematics_B.kAcol = inverse_kinematics_B.jA << 2;
    if (Y_data[inverse_kinematics_B.jA] != 0.0) {
      Y_data[inverse_kinematics_B.jA] /=
        inverse_kinematics_B.c_x[inverse_kinematics_B.jA +
        inverse_kinematics_B.kAcol];
      inverse_kinematics_B.a_f = inverse_kinematics_B.jA - 1;
      for (inverse_kinematics_B.c_g = 0; inverse_kinematics_B.c_g <=
           inverse_kinematics_B.a_f; inverse_kinematics_B.c_g++) {
        Y_data[inverse_kinematics_B.c_g] -=
          inverse_kinematics_B.c_x[inverse_kinematics_B.c_g +
          inverse_kinematics_B.kAcol] * Y_data[inverse_kinematics_B.jA];
      }
    }
  }
}

static void inverse_kine_binary_expand_op_g(real_T in1_data[], int32_T *in1_size,
  const emxArray_real_T_inverse_kinem_T *in2, real_T in3, const real_T in4[16],
  const emxArray_real_T_inverse_kinem_T *in5)
{
  inverse_kinematics_B.stride_0_0 = (in2->size[0] != 1);
  inverse_kinematics_B.stride_0_1 = (in2->size[1] != 1);
  inverse_kinematics_B.aux_0_1 = 0;
  for (inverse_kinematics_B.i4 = 0; inverse_kinematics_B.i4 < 4;
       inverse_kinematics_B.i4++) {
    inverse_kinematics_B.in2_tmp = inverse_kinematics_B.i4 << 2;
    inverse_kinematics_B.in2[inverse_kinematics_B.in2_tmp] =
      -(in4[inverse_kinematics_B.in2_tmp] * in3 + in2->data[in2->size[0] *
        inverse_kinematics_B.aux_0_1]);
    inverse_kinematics_B.in2[inverse_kinematics_B.in2_tmp + 1] =
      -(in4[inverse_kinematics_B.in2_tmp + 1] * in3 + in2->data[in2->size[0] *
        inverse_kinematics_B.aux_0_1 + inverse_kinematics_B.stride_0_0]);
    inverse_kinematics_B.in2[inverse_kinematics_B.in2_tmp + 2] =
      -(in4[inverse_kinematics_B.in2_tmp + 2] * in3 + in2->data
        [(inverse_kinematics_B.stride_0_0 << 1) + in2->size[0] *
        inverse_kinematics_B.aux_0_1]);
    inverse_kinematics_B.in2[inverse_kinematics_B.in2_tmp + 3] =
      -(in4[inverse_kinematics_B.in2_tmp + 3] * in3 + in2->data[3 *
        inverse_kinematics_B.stride_0_0 + in2->size[0] *
        inverse_kinematics_B.aux_0_1]);
    inverse_kinematics_B.aux_0_1 += inverse_kinematics_B.stride_0_1;
  }

  inverse_kinematics_mldivide(inverse_kinematics_B.in2, in5, in1_data, in1_size);
}

static void inverse_kinematics_expand_max(const emxArray_real_T_inverse_kinem_T *
  a, const real_T b[4], real_T c[4])
{
  real_T u0;
  int32_T acoef;
  acoef = (a->size[0] != 1);
  if ((a->data[0] >= b[0]) || rtIsNaN(b[0])) {
    c[0] = a->data[0];
  } else {
    c[0] = b[0];
  }

  u0 = a->data[acoef];
  if ((u0 >= b[1]) || rtIsNaN(b[1])) {
    c[1] = u0;
  } else {
    c[1] = b[1];
  }

  u0 = a->data[acoef << 1];
  if ((u0 >= b[2]) || rtIsNaN(b[2])) {
    c[2] = u0;
  } else {
    c[2] = b[2];
  }

  u0 = a->data[acoef * 3];
  if ((u0 >= b[3]) || rtIsNaN(b[3])) {
    c[3] = u0;
  } else {
    c[3] = b[3];
  }
}

static void inverse_kinematics_expand_min(const emxArray_real_T_inverse_kinem_T *
  a, const real_T b[4], real_T c[4])
{
  real_T u0;
  int32_T acoef;
  acoef = (a->size[0] != 1);
  if ((a->data[0] <= b[0]) || rtIsNaN(b[0])) {
    c[0] = a->data[0];
  } else {
    c[0] = b[0];
  }

  u0 = a->data[acoef];
  if ((u0 <= b[1]) || rtIsNaN(b[1])) {
    c[1] = u0;
  } else {
    c[1] = b[1];
  }

  u0 = a->data[acoef << 1];
  if ((u0 <= b[2]) || rtIsNaN(b[2])) {
    c[2] = u0;
  } else {
    c[2] = b[2];
  }

  u0 = a->data[acoef * 3];
  if ((u0 <= b[3]) || rtIsNaN(b[3])) {
    c[3] = u0;
  } else {
    c[3] = b[3];
  }
}

static void inverse_kinem_emxFree_boolean_T(emxArray_boolean_T_inverse_ki_T
  **pEmxArray)
{
  if (*pEmxArray != static_cast<emxArray_boolean_T_inverse_ki_T *>(NULL)) {
    if (((*pEmxArray)->data != static_cast<boolean_T *>(NULL)) && (*pEmxArray)
        ->canFreeData) {
      free((*pEmxArray)->data);
    }

    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = static_cast<emxArray_boolean_T_inverse_ki_T *>(NULL);
  }
}

static void ErrorDampedLevenbergMarquardt_s(h_robotics_core_internal_Erro_T *obj,
  real_T xSol[4], c_robotics_core_internal_NLPS_T *exitFlag, real_T *en, real_T *
  iter)
{
  emxArray_boolean_T_inverse_ki_T *x;
  emxArray_char_T_inverse_kinem_T *bodyName;
  emxArray_real_T_inverse_kinem_T *H0;
  emxArray_real_T_inverse_kinem_T *J;
  emxArray_real_T_inverse_kinem_T *b;
  emxArray_real_T_inverse_kinem_T *ev;
  emxArray_real_T_inverse_kinem_T *evprev;
  emxArray_real_T_inverse_kinem_T *grad;
  emxArray_real_T_inverse_kinem_T *tmp;
  emxArray_real_T_inverse_kinem_T *y;
  f_robotics_manip_internal_IKE_T *args;
  v_robotics_manip_internal_Rig_T *treeInternal;
  static const real_T tmp_0[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  emxArray_real_T_inverse_kinem_T *J_0;
  inverse_kinemati_emxInit_char_T(&bodyName, 2);
  xSol[0] = obj->SeedInternal[0];
  xSol[1] = obj->SeedInternal[1];
  xSol[2] = obj->SeedInternal[2];
  xSol[3] = obj->SeedInternal[3];
  inverse_kinematics_tic(&obj->TimeObjInternal.StartTime.tv_sec,
    &obj->TimeObjInternal.StartTime.tv_nsec);
  inverse_kinematics_B.xprev_idx_0 = xSol[0];
  inverse_kinematics_B.xprev_idx_1 = xSol[1];
  inverse_kinematics_B.xprev_idx_2 = xSol[2];
  inverse_kinematics_B.xprev_idx_3 = xSol[3];
  args = obj->ExtraArgs;
  treeInternal = args->Robot;
  inverse_kinematics_B.J = bodyName->size[0] * bodyName->size[1];
  bodyName->size[0] = 1;
  bodyName->size[1] = args->BodyName->size[1];
  invers_emxEnsureCapacity_char_T(bodyName, inverse_kinematics_B.J);
  inverse_kinematics_B.kend = args->BodyName->size[1] - 1;
  for (inverse_kinematics_B.b_k_c = 0; inverse_kinematics_B.b_k_c <=
       inverse_kinematics_B.kend; inverse_kinematics_B.b_k_c++) {
    inverse_kinematics_B.J = inverse_kinematics_B.b_k_c;
    bodyName->data[inverse_kinematics_B.J] = args->BodyName->
      data[inverse_kinematics_B.J];
  }

  for (inverse_kinematics_B.b_k_c = 0; inverse_kinematics_B.b_k_c < 16;
       inverse_kinematics_B.b_k_c++) {
    inverse_kinematics_B.Td[inverse_kinematics_B.b_k_c] = args->
      Tform[inverse_kinematics_B.b_k_c];
  }

  for (inverse_kinematics_B.b_k_c = 0; inverse_kinematics_B.b_k_c < 36;
       inverse_kinematics_B.b_k_c++) {
    inverse_kinematics_B.weightMatrix_m[inverse_kinematics_B.b_k_c] =
      args->WeightMatrix[inverse_kinematics_B.b_k_c];
  }

  inverse_kinemati_emxInit_real_T(&J, 2);
  RigidBodyTree_efficientFKAndJac(treeInternal, xSol, bodyName,
    inverse_kinematics_B.T_data, inverse_kinematics_B.T_size, J);
  inverse_kin_IKHelpers_poseError(inverse_kinematics_B.Td,
    inverse_kinematics_B.T_data, inverse_kinematics_B.T_size,
    inverse_kinematics_B.e);
  inverse_kinematics_B.J = args->ErrTemp->size[0];
  args->ErrTemp->size[0] = 6;
  invers_emxEnsureCapacity_real_T(args->ErrTemp, inverse_kinematics_B.J);
  for (inverse_kinematics_B.b_k_c = 0; inverse_kinematics_B.b_k_c < 6;
       inverse_kinematics_B.b_k_c++) {
    args->ErrTemp->data[inverse_kinematics_B.b_k_c] =
      inverse_kinematics_B.e[inverse_kinematics_B.b_k_c];
  }

  inverse_kinematics_B.absxk = 0.0;
  for (inverse_kinematics_B.b_k_c = 0; inverse_kinematics_B.b_k_c < 6;
       inverse_kinematics_B.b_k_c++) {
    inverse_kinematics_B.e_n[inverse_kinematics_B.b_k_c] = 0.0;
    for (inverse_kinematics_B.kend = 0; inverse_kinematics_B.kend < 6;
         inverse_kinematics_B.kend++) {
      inverse_kinematics_B.e_n[inverse_kinematics_B.b_k_c] +=
        inverse_kinematics_B.weightMatrix_m[6 * inverse_kinematics_B.b_k_c +
        inverse_kinematics_B.kend] * (0.5 *
        inverse_kinematics_B.e[inverse_kinematics_B.kend]);
    }

    inverse_kinematics_B.absxk +=
      inverse_kinematics_B.e_n[inverse_kinematics_B.b_k_c] *
      inverse_kinematics_B.e[inverse_kinematics_B.b_k_c];
  }

  args->CostTemp = inverse_kinematics_B.absxk;
  for (inverse_kinematics_B.b_k_c = 0; inverse_kinematics_B.b_k_c < 6;
       inverse_kinematics_B.b_k_c++) {
    inverse_kinematics_B.e_n[inverse_kinematics_B.b_k_c] = 0.0;
    for (inverse_kinematics_B.kend = 0; inverse_kinematics_B.kend < 6;
         inverse_kinematics_B.kend++) {
      inverse_kinematics_B.e_n[inverse_kinematics_B.b_k_c] +=
        inverse_kinematics_B.weightMatrix_m[6 * inverse_kinematics_B.b_k_c +
        inverse_kinematics_B.kend] *
        inverse_kinematics_B.e[inverse_kinematics_B.kend];
    }
  }

  inverse_kinemati_emxInit_real_T(&J_0, 2);
  inverse_kinematics_B.J = J_0->size[0] * J_0->size[1];
  J_0->size[0] = 6;
  J_0->size[1] = J->size[1];
  invers_emxEnsureCapacity_real_T(J_0, inverse_kinematics_B.J);
  inverse_kinematics_B.kend = 6 * J->size[1];
  for (inverse_kinematics_B.b_k_c = 0; inverse_kinematics_B.b_k_c <
       inverse_kinematics_B.kend; inverse_kinematics_B.b_k_c++) {
    J_0->data[inverse_kinematics_B.b_k_c] = -J->data[inverse_kinematics_B.b_k_c];
  }

  inverse_kinemati_emxInit_real_T(&tmp, 2);
  inverse_kinematics_mtimes_g(inverse_kinematics_B.e_n, J_0, tmp);
  inverse_kinematics_B.J = args->GradTemp->size[0];
  args->GradTemp->size[0] = tmp->size[1];
  invers_emxEnsureCapacity_real_T(args->GradTemp, inverse_kinematics_B.J);
  inverse_kinematics_B.kend = tmp->size[1];
  for (inverse_kinematics_B.b_k_c = 0; inverse_kinematics_B.b_k_c <
       inverse_kinematics_B.kend; inverse_kinematics_B.b_k_c++) {
    args->GradTemp->data[inverse_kinematics_B.b_k_c] = tmp->
      data[inverse_kinematics_B.b_k_c];
  }

  inverse_kinemati_emxInit_real_T(&evprev, 1);
  obj->ExtraArgs = args;
  args = obj->ExtraArgs;
  inverse_kinematics_B.J = evprev->size[0];
  evprev->size[0] = args->ErrTemp->size[0];
  invers_emxEnsureCapacity_real_T(evprev, inverse_kinematics_B.J);
  inverse_kinematics_B.kend = args->ErrTemp->size[0];
  for (inverse_kinematics_B.b_k_c = 0; inverse_kinematics_B.b_k_c <
       inverse_kinematics_B.kend; inverse_kinematics_B.b_k_c++) {
    evprev->data[inverse_kinematics_B.b_k_c] = args->ErrTemp->
      data[inverse_kinematics_B.b_k_c];
  }

  inverse_kinematics_B.d = obj->MaxNumIterationInternal;
  inverse_kinematics_B.b_i_g = 0;
  inverse_kinemati_emxInit_real_T(&grad, 1);
  inverse_kinemati_emxInit_real_T(&H0, 2);
  inverse_kinemati_emxInit_real_T(&ev, 1);
  inverse_kinemati_emxInit_real_T(&y, 2);
  inverse_kinemati_emxInit_real_T(&b, 1);
  inverse_kinem_emxInit_boolean_T(&x, 1);
  int32_T exitg1;
  do {
    exitg1 = 0;
    if (inverse_kinematics_B.b_i_g <= static_cast<int32_T>
        (inverse_kinematics_B.d) - 1) {
      args = obj->ExtraArgs;
      treeInternal = args->Robot;
      inverse_kinematics_B.J = bodyName->size[0] * bodyName->size[1];
      bodyName->size[0] = 1;
      bodyName->size[1] = args->BodyName->size[1];
      invers_emxEnsureCapacity_char_T(bodyName, inverse_kinematics_B.J);
      inverse_kinematics_B.kend = args->BodyName->size[1] - 1;
      for (inverse_kinematics_B.b_k_c = 0; inverse_kinematics_B.b_k_c <=
           inverse_kinematics_B.kend; inverse_kinematics_B.b_k_c++) {
        inverse_kinematics_B.J = inverse_kinematics_B.b_k_c;
        bodyName->data[inverse_kinematics_B.J] = args->BodyName->
          data[inverse_kinematics_B.J];
      }

      for (inverse_kinematics_B.b_k_c = 0; inverse_kinematics_B.b_k_c < 16;
           inverse_kinematics_B.b_k_c++) {
        inverse_kinematics_B.Td[inverse_kinematics_B.b_k_c] = args->
          Tform[inverse_kinematics_B.b_k_c];
      }

      for (inverse_kinematics_B.b_k_c = 0; inverse_kinematics_B.b_k_c < 36;
           inverse_kinematics_B.b_k_c++) {
        inverse_kinematics_B.weightMatrix_m[inverse_kinematics_B.b_k_c] =
          args->WeightMatrix[inverse_kinematics_B.b_k_c];
      }

      RigidBodyTree_efficientFKAndJac(treeInternal, xSol, bodyName,
        inverse_kinematics_B.T_data, inverse_kinematics_B.T_size, J);
      inverse_kinematics_B.J = J_0->size[0] * J_0->size[1];
      J_0->size[0] = 6;
      J_0->size[1] = J->size[1];
      invers_emxEnsureCapacity_real_T(J_0, inverse_kinematics_B.J);
      inverse_kinematics_B.kend = 6 * J->size[1];
      for (inverse_kinematics_B.b_k_c = 0; inverse_kinematics_B.b_k_c <
           inverse_kinematics_B.kend; inverse_kinematics_B.b_k_c++) {
        J_0->data[inverse_kinematics_B.b_k_c] = -J->
          data[inverse_kinematics_B.b_k_c];
      }

      inverse_kin_IKHelpers_poseError(inverse_kinematics_B.Td,
        inverse_kinematics_B.T_data, inverse_kinematics_B.T_size,
        inverse_kinematics_B.e);
      inverse_kinematics_B.J = args->ErrTemp->size[0];
      args->ErrTemp->size[0] = 6;
      invers_emxEnsureCapacity_real_T(args->ErrTemp, inverse_kinematics_B.J);
      for (inverse_kinematics_B.b_k_c = 0; inverse_kinematics_B.b_k_c < 6;
           inverse_kinematics_B.b_k_c++) {
        args->ErrTemp->data[inverse_kinematics_B.b_k_c] =
          inverse_kinematics_B.e[inverse_kinematics_B.b_k_c];
      }

      inverse_kinematics_B.absxk = 0.0;
      for (inverse_kinematics_B.b_k_c = 0; inverse_kinematics_B.b_k_c < 6;
           inverse_kinematics_B.b_k_c++) {
        inverse_kinematics_B.e_n[inverse_kinematics_B.b_k_c] = 0.0;
        for (inverse_kinematics_B.kend = 0; inverse_kinematics_B.kend < 6;
             inverse_kinematics_B.kend++) {
          inverse_kinematics_B.e_n[inverse_kinematics_B.b_k_c] +=
            inverse_kinematics_B.weightMatrix_m[6 * inverse_kinematics_B.b_k_c +
            inverse_kinematics_B.kend] * (0.5 *
            inverse_kinematics_B.e[inverse_kinematics_B.kend]);
        }

        inverse_kinematics_B.absxk +=
          inverse_kinematics_B.e_n[inverse_kinematics_B.b_k_c] *
          inverse_kinematics_B.e[inverse_kinematics_B.b_k_c];
      }

      args->CostTemp = inverse_kinematics_B.absxk;
      for (inverse_kinematics_B.b_k_c = 0; inverse_kinematics_B.b_k_c < 6;
           inverse_kinematics_B.b_k_c++) {
        inverse_kinematics_B.e_n[inverse_kinematics_B.b_k_c] = 0.0;
        for (inverse_kinematics_B.kend = 0; inverse_kinematics_B.kend < 6;
             inverse_kinematics_B.kend++) {
          inverse_kinematics_B.e_n[inverse_kinematics_B.b_k_c] +=
            inverse_kinematics_B.weightMatrix_m[6 * inverse_kinematics_B.b_k_c +
            inverse_kinematics_B.kend] *
            inverse_kinematics_B.e[inverse_kinematics_B.kend];
        }
      }

      inverse_kinematics_mtimes_g(inverse_kinematics_B.e_n, J_0, tmp);
      inverse_kinematics_B.J = args->GradTemp->size[0];
      args->GradTemp->size[0] = tmp->size[1];
      invers_emxEnsureCapacity_real_T(args->GradTemp, inverse_kinematics_B.J);
      inverse_kinematics_B.kend = tmp->size[1];
      for (inverse_kinematics_B.b_k_c = 0; inverse_kinematics_B.b_k_c <
           inverse_kinematics_B.kend; inverse_kinematics_B.b_k_c++) {
        args->GradTemp->data[inverse_kinematics_B.b_k_c] = tmp->
          data[inverse_kinematics_B.b_k_c];
      }

      inverse_kinematics_B.cost = args->CostTemp;
      obj->ExtraArgs = args;
      args = obj->ExtraArgs;
      inverse_kinematics_B.J = grad->size[0];
      grad->size[0] = args->GradTemp->size[0];
      invers_emxEnsureCapacity_real_T(grad, inverse_kinematics_B.J);
      inverse_kinematics_B.kend = args->GradTemp->size[0];
      for (inverse_kinematics_B.b_k_c = 0; inverse_kinematics_B.b_k_c <
           inverse_kinematics_B.kend; inverse_kinematics_B.b_k_c++) {
        grad->data[inverse_kinematics_B.b_k_c] = args->GradTemp->
          data[inverse_kinematics_B.b_k_c];
      }

      args = obj->ExtraArgs;
      for (inverse_kinematics_B.b_k_c = 0; inverse_kinematics_B.b_k_c < 36;
           inverse_kinematics_B.b_k_c++) {
        inverse_kinematics_B.a[inverse_kinematics_B.b_k_c] = args->
          WeightMatrix[inverse_kinematics_B.b_k_c];
      }

      inverse_kinematics_B.J = b->size[0];
      b->size[0] = args->ErrTemp->size[0];
      invers_emxEnsureCapacity_real_T(b, inverse_kinematics_B.J);
      inverse_kinematics_B.kend = args->ErrTemp->size[0];
      for (inverse_kinematics_B.b_k_c = 0; inverse_kinematics_B.b_k_c <
           inverse_kinematics_B.kend; inverse_kinematics_B.b_k_c++) {
        b->data[inverse_kinematics_B.b_k_c] = args->ErrTemp->
          data[inverse_kinematics_B.b_k_c];
      }

      inverse_kinematics_B.J = ev->size[0];
      ev->size[0] = args->ErrTemp->size[0];
      invers_emxEnsureCapacity_real_T(ev, inverse_kinematics_B.J);
      inverse_kinematics_B.kend = args->ErrTemp->size[0];
      for (inverse_kinematics_B.b_k_c = 0; inverse_kinematics_B.b_k_c <
           inverse_kinematics_B.kend; inverse_kinematics_B.b_k_c++) {
        ev->data[inverse_kinematics_B.b_k_c] = args->ErrTemp->
          data[inverse_kinematics_B.b_k_c];
      }

      for (inverse_kinematics_B.b_k_c = 0; inverse_kinematics_B.b_k_c < 6;
           inverse_kinematics_B.b_k_c++) {
        inverse_kinematics_B.e[inverse_kinematics_B.b_k_c] = 0.0;
        for (inverse_kinematics_B.kend = 0; inverse_kinematics_B.kend < 6;
             inverse_kinematics_B.kend++) {
          inverse_kinematics_B.e[inverse_kinematics_B.b_k_c] +=
            inverse_kinematics_B.a[6 * inverse_kinematics_B.kend +
            inverse_kinematics_B.b_k_c] * b->data[inverse_kinematics_B.kend];
        }
      }

      *en = inverse_kinematics_norm_g(inverse_kinematics_B.e);
      *iter = static_cast<real_T>(inverse_kinematics_B.b_i_g) + 1.0;
      if (grad->size[0] == 0) {
        inverse_kinematics_B.cc = 0.0;
      } else {
        inverse_kinematics_B.cc = 0.0;
        if (grad->size[0] == 1) {
          inverse_kinematics_B.cc = fabs(grad->data[0]);
        } else {
          inverse_kinematics_B.scale = 3.3121686421112381E-170;
          inverse_kinematics_B.kend = grad->size[0] - 1;
          for (inverse_kinematics_B.b_k_c = 0; inverse_kinematics_B.b_k_c <=
               inverse_kinematics_B.kend; inverse_kinematics_B.b_k_c++) {
            inverse_kinematics_B.absxk = fabs(grad->
              data[inverse_kinematics_B.b_k_c]);
            if (inverse_kinematics_B.absxk > inverse_kinematics_B.scale) {
              inverse_kinematics_B.t = inverse_kinematics_B.scale /
                inverse_kinematics_B.absxk;
              inverse_kinematics_B.cc = inverse_kinematics_B.cc *
                inverse_kinematics_B.t * inverse_kinematics_B.t + 1.0;
              inverse_kinematics_B.scale = inverse_kinematics_B.absxk;
            } else {
              inverse_kinematics_B.t = inverse_kinematics_B.absxk /
                inverse_kinematics_B.scale;
              inverse_kinematics_B.cc += inverse_kinematics_B.t *
                inverse_kinematics_B.t;
            }
          }

          inverse_kinematics_B.cc = inverse_kinematics_B.scale * sqrt
            (inverse_kinematics_B.cc);
        }
      }

      inverse_kinematics_B.flag = (inverse_kinematics_B.cc <
        obj->GradientTolerance);
      if (inverse_kinematics_B.flag) {
        *exitFlag = LocalMinimumFound;
        exitg1 = 1;
      } else {
        boolean_T exitg2;
        boolean_T guard1 = false;
        boolean_T guard2 = false;
        guard1 = false;
        guard2 = false;
        if (static_cast<real_T>(inverse_kinematics_B.b_i_g) + 1.0 > 1.0) {
          inverse_kinematics_B.x_l[0] = (fabs(xSol[0] -
            inverse_kinematics_B.xprev_idx_0) < obj->StepTolerance);
          inverse_kinematics_B.x_l[1] = (fabs(xSol[1] -
            inverse_kinematics_B.xprev_idx_1) < obj->StepTolerance);
          inverse_kinematics_B.x_l[2] = (fabs(xSol[2] -
            inverse_kinematics_B.xprev_idx_2) < obj->StepTolerance);
          inverse_kinematics_B.x_l[3] = (fabs(xSol[3] -
            inverse_kinematics_B.xprev_idx_3) < obj->StepTolerance);
          inverse_kinematics_B.flag = true;
          inverse_kinematics_B.b_k_c = 0;
          exitg2 = false;
          while ((!exitg2) && (inverse_kinematics_B.b_k_c < 4)) {
            if (!inverse_kinematics_B.x_l[inverse_kinematics_B.b_k_c]) {
              inverse_kinematics_B.flag = false;
              exitg2 = true;
            } else {
              inverse_kinematics_B.b_k_c++;
            }
          }

          if (inverse_kinematics_B.flag) {
            *exitFlag = StepSizeBelowMinimum;
            exitg1 = 1;
          } else {
            guard2 = true;
          }
        } else {
          guard2 = true;
        }

        if (guard2) {
          if (static_cast<real_T>(inverse_kinematics_B.b_i_g) + 1.0 > 1.0) {
            if (ev->size[0] == evprev->size[0]) {
              inverse_kinematics_B.J = evprev->size[0];
              evprev->size[0] = ev->size[0];
              invers_emxEnsureCapacity_real_T(evprev, inverse_kinematics_B.J);
              inverse_kinematics_B.kend = ev->size[0];
              for (inverse_kinematics_B.b_k_c = 0; inverse_kinematics_B.b_k_c <
                   inverse_kinematics_B.kend; inverse_kinematics_B.b_k_c++) {
                evprev->data[inverse_kinematics_B.b_k_c] = ev->
                  data[inverse_kinematics_B.b_k_c] - evprev->
                  data[inverse_kinematics_B.b_k_c];
              }
            } else {
              inverse_kinematics_minus_g(evprev, ev);
            }

            inverse_kinematics_B.kend = evprev->size[0] - 1;
            inverse_kinematics_B.J = b->size[0];
            b->size[0] = evprev->size[0];
            invers_emxEnsureCapacity_real_T(b, inverse_kinematics_B.J);
            for (inverse_kinematics_B.b_k_c = 0; inverse_kinematics_B.b_k_c <=
                 inverse_kinematics_B.kend; inverse_kinematics_B.b_k_c++) {
              b->data[inverse_kinematics_B.b_k_c] = fabs(evprev->
                data[inverse_kinematics_B.b_k_c]);
            }

            inverse_kinematics_B.J = x->size[0];
            x->size[0] = b->size[0];
            inv_emxEnsureCapacity_boolean_T(x, inverse_kinematics_B.J);
            inverse_kinematics_B.kend = b->size[0];
            for (inverse_kinematics_B.b_k_c = 0; inverse_kinematics_B.b_k_c <
                 inverse_kinematics_B.kend; inverse_kinematics_B.b_k_c++) {
              x->data[inverse_kinematics_B.b_k_c] = (b->
                data[inverse_kinematics_B.b_k_c] < obj->ErrorChangeTolerance);
            }

            inverse_kinematics_B.flag = true;
            inverse_kinematics_B.b_k_c = 0;
            exitg2 = false;
            while ((!exitg2) && (inverse_kinematics_B.b_k_c + 1 <= x->size[0]))
            {
              if (!x->data[inverse_kinematics_B.b_k_c]) {
                inverse_kinematics_B.flag = false;
                exitg2 = true;
              } else {
                inverse_kinematics_B.b_k_c++;
              }
            }

            if (inverse_kinematics_B.flag) {
              *exitFlag = ChangeInErrorBelowMinimum;
              exitg1 = 1;
            } else {
              guard1 = true;
            }
          } else {
            guard1 = true;
          }
        }

        if (guard1) {
          inverse_kinematics_B.xprev_idx_0 = inverse_kinematics_toc
            (obj->TimeObjInternal.StartTime.tv_sec,
             obj->TimeObjInternal.StartTime.tv_nsec);
          inverse_kinematics_B.flag = (inverse_kinematics_B.xprev_idx_0 >
            obj->MaxTimeInternal);
          if (inverse_kinematics_B.flag) {
            *exitFlag = TimeLimitExceeded;
            exitg1 = 1;
          } else {
            inverse_kinematics_B.J = evprev->size[0];
            evprev->size[0] = ev->size[0];
            invers_emxEnsureCapacity_real_T(evprev, inverse_kinematics_B.J);
            inverse_kinematics_B.kend = ev->size[0];
            if (inverse_kinematics_B.kend - 1 >= 0) {
              memcpy(&evprev->data[0], &ev->data[0], inverse_kinematics_B.kend *
                     sizeof(real_T));
            }

            inverse_kinematics_B.xprev_idx_0 = xSol[0];
            inverse_kinematics_B.xprev_idx_1 = xSol[1];
            inverse_kinematics_B.xprev_idx_2 = xSol[2];
            inverse_kinematics_B.xprev_idx_3 = xSol[3];
            inverse_kinematics_B.flag = obj->UseErrorDamping;
            inverse_kinematics_B.cc = static_cast<real_T>
              (inverse_kinematics_B.flag) * inverse_kinematics_B.cost;
            inverse_kinematics_B.absxk = inverse_kinematics_B.cc +
              obj->DampingBias;
            inverse_kinematics_B.m = J_0->size[1];
            inverse_kinematics_B.J = y->size[0] * y->size[1];
            y->size[0] = J_0->size[1];
            y->size[1] = 6;
            invers_emxEnsureCapacity_real_T(y, inverse_kinematics_B.J);
            for (inverse_kinematics_B.kend = 0; inverse_kinematics_B.kend < 6;
                 inverse_kinematics_B.kend++) {
              inverse_kinematics_B.coffset = inverse_kinematics_B.kend *
                inverse_kinematics_B.m - 1;
              inverse_kinematics_B.boffset = inverse_kinematics_B.kend * 6 - 1;
              for (inverse_kinematics_B.J = 0; inverse_kinematics_B.J <
                   inverse_kinematics_B.m; inverse_kinematics_B.J++) {
                inverse_kinematics_B.aoffset = inverse_kinematics_B.J * 6 - 1;
                inverse_kinematics_B.scale = 0.0;
                for (inverse_kinematics_B.b_k_c = 0; inverse_kinematics_B.b_k_c <
                     6; inverse_kinematics_B.b_k_c++) {
                  inverse_kinematics_B.scale += J_0->data
                    [(inverse_kinematics_B.b_k_c + inverse_kinematics_B.aoffset)
                    + 1] * inverse_kinematics_B.weightMatrix_m
                    [(inverse_kinematics_B.b_k_c + inverse_kinematics_B.boffset)
                    + 1];
                }

                y->data[(inverse_kinematics_B.coffset + inverse_kinematics_B.J)
                  + 1] = inverse_kinematics_B.scale;
              }
            }

            inverse_kinematics_B.m = y->size[0];
            inverse_kinematics_B.aoffset = J_0->size[1] - 1;
            inverse_kinematics_B.J = H0->size[0] * H0->size[1];
            H0->size[0] = y->size[0];
            H0->size[1] = J_0->size[1];
            invers_emxEnsureCapacity_real_T(H0, inverse_kinematics_B.J);
            for (inverse_kinematics_B.kend = 0; inverse_kinematics_B.kend <=
                 inverse_kinematics_B.aoffset; inverse_kinematics_B.kend++) {
              inverse_kinematics_B.coffset = inverse_kinematics_B.kend *
                inverse_kinematics_B.m - 1;
              inverse_kinematics_B.boffset = inverse_kinematics_B.kend * 6 - 1;
              for (inverse_kinematics_B.J = 0; inverse_kinematics_B.J <
                   inverse_kinematics_B.m; inverse_kinematics_B.J++) {
                inverse_kinematics_B.scale = 0.0;
                for (inverse_kinematics_B.b_k_c = 0; inverse_kinematics_B.b_k_c <
                     6; inverse_kinematics_B.b_k_c++) {
                  inverse_kinematics_B.scale += y->
                    data[inverse_kinematics_B.b_k_c * y->size[0] +
                    inverse_kinematics_B.J] * J_0->data
                    [(inverse_kinematics_B.boffset + inverse_kinematics_B.b_k_c)
                    + 1];
                }

                H0->data[(inverse_kinematics_B.coffset + inverse_kinematics_B.J)
                  + 1] = inverse_kinematics_B.scale;
              }
            }

            if ((H0->size[0] == 4) && (H0->size[1] == 4)) {
              for (inverse_kinematics_B.b_k_c = 0; inverse_kinematics_B.b_k_c <
                   4; inverse_kinematics_B.b_k_c++) {
                for (inverse_kinematics_B.kend = 0; inverse_kinematics_B.kend <
                     4; inverse_kinematics_B.kend++) {
                  inverse_kinematics_B.J = (inverse_kinematics_B.b_k_c << 2) +
                    inverse_kinematics_B.kend;
                  inverse_kinematics_B.Td[inverse_kinematics_B.J] = -(H0->
                    data[H0->size[0] * inverse_kinematics_B.b_k_c +
                    inverse_kinematics_B.kend] + tmp_0[inverse_kinematics_B.J] *
                    inverse_kinematics_B.absxk);
                }
              }

              inverse_kinematics_mldivide(inverse_kinematics_B.Td, grad,
                inverse_kinematics_B.step_data, &inverse_kinematics_B.step_size);
            } else {
              inverse_kine_binary_expand_op_g(inverse_kinematics_B.step_data,
                &inverse_kinematics_B.step_size, H0, inverse_kinematics_B.absxk,
                tmp_0, grad);
            }

            args = obj->ExtraArgs;
            treeInternal = args->Robot;
            inverse_kinematics_B.J = bodyName->size[0] * bodyName->size[1];
            bodyName->size[0] = 1;
            bodyName->size[1] = args->BodyName->size[1];
            invers_emxEnsureCapacity_char_T(bodyName, inverse_kinematics_B.J);
            inverse_kinematics_B.kend = args->BodyName->size[1] - 1;
            for (inverse_kinematics_B.b_k_c = 0; inverse_kinematics_B.b_k_c <=
                 inverse_kinematics_B.kend; inverse_kinematics_B.b_k_c++) {
              inverse_kinematics_B.J = inverse_kinematics_B.b_k_c;
              bodyName->data[inverse_kinematics_B.J] = args->BodyName->
                data[inverse_kinematics_B.J];
            }

            for (inverse_kinematics_B.b_k_c = 0; inverse_kinematics_B.b_k_c < 16;
                 inverse_kinematics_B.b_k_c++) {
              inverse_kinematics_B.Td[inverse_kinematics_B.b_k_c] = args->
                Tform[inverse_kinematics_B.b_k_c];
            }

            for (inverse_kinematics_B.b_k_c = 0; inverse_kinematics_B.b_k_c < 36;
                 inverse_kinematics_B.b_k_c++) {
              inverse_kinematics_B.weightMatrix_m[inverse_kinematics_B.b_k_c] =
                args->WeightMatrix[inverse_kinematics_B.b_k_c];
            }

            inverse_kinematics_B.y_g[0] = xSol[0] +
              inverse_kinematics_B.step_data[0];
            inverse_kinematics_B.y_g[1] = xSol[1] +
              inverse_kinematics_B.step_data[1];
            inverse_kinematics_B.y_g[2] = xSol[2] +
              inverse_kinematics_B.step_data[2];
            inverse_kinematics_B.y_g[3] = xSol[3] +
              inverse_kinematics_B.step_data[3];
            RigidBodyTree_efficientFKAndJac(treeInternal,
              inverse_kinematics_B.y_g, bodyName, inverse_kinematics_B.T_data,
              inverse_kinematics_B.T_size, J);
            inverse_kin_IKHelpers_poseError(inverse_kinematics_B.Td,
              inverse_kinematics_B.T_data, inverse_kinematics_B.T_size,
              inverse_kinematics_B.e);
            inverse_kinematics_B.J = args->ErrTemp->size[0];
            args->ErrTemp->size[0] = 6;
            invers_emxEnsureCapacity_real_T(args->ErrTemp,
              inverse_kinematics_B.J);
            for (inverse_kinematics_B.b_k_c = 0; inverse_kinematics_B.b_k_c < 6;
                 inverse_kinematics_B.b_k_c++) {
              args->ErrTemp->data[inverse_kinematics_B.b_k_c] =
                inverse_kinematics_B.e[inverse_kinematics_B.b_k_c];
            }

            inverse_kinematics_B.absxk = 0.0;
            for (inverse_kinematics_B.b_k_c = 0; inverse_kinematics_B.b_k_c < 6;
                 inverse_kinematics_B.b_k_c++) {
              inverse_kinematics_B.e_n[inverse_kinematics_B.b_k_c] = 0.0;
              for (inverse_kinematics_B.kend = 0; inverse_kinematics_B.kend < 6;
                   inverse_kinematics_B.kend++) {
                inverse_kinematics_B.e_n[inverse_kinematics_B.b_k_c] +=
                  inverse_kinematics_B.weightMatrix_m[6 *
                  inverse_kinematics_B.b_k_c + inverse_kinematics_B.kend] * (0.5
                  * inverse_kinematics_B.e[inverse_kinematics_B.kend]);
              }

              inverse_kinematics_B.absxk +=
                inverse_kinematics_B.e_n[inverse_kinematics_B.b_k_c] *
                inverse_kinematics_B.e[inverse_kinematics_B.b_k_c];
            }

            args->CostTemp = inverse_kinematics_B.absxk;
            for (inverse_kinematics_B.b_k_c = 0; inverse_kinematics_B.b_k_c < 6;
                 inverse_kinematics_B.b_k_c++) {
              inverse_kinematics_B.e_n[inverse_kinematics_B.b_k_c] = 0.0;
              for (inverse_kinematics_B.kend = 0; inverse_kinematics_B.kend < 6;
                   inverse_kinematics_B.kend++) {
                inverse_kinematics_B.e_n[inverse_kinematics_B.b_k_c] +=
                  inverse_kinematics_B.weightMatrix_m[6 *
                  inverse_kinematics_B.b_k_c + inverse_kinematics_B.kend] *
                  inverse_kinematics_B.e[inverse_kinematics_B.kend];
              }
            }

            inverse_kinematics_B.J = J_0->size[0] * J_0->size[1];
            J_0->size[0] = 6;
            J_0->size[1] = J->size[1];
            invers_emxEnsureCapacity_real_T(J_0, inverse_kinematics_B.J);
            inverse_kinematics_B.kend = 6 * J->size[1];
            for (inverse_kinematics_B.b_k_c = 0; inverse_kinematics_B.b_k_c <
                 inverse_kinematics_B.kend; inverse_kinematics_B.b_k_c++) {
              J_0->data[inverse_kinematics_B.b_k_c] = -J->
                data[inverse_kinematics_B.b_k_c];
            }

            inverse_kinematics_mtimes_g(inverse_kinematics_B.e_n, J_0, tmp);
            inverse_kinematics_B.J = args->GradTemp->size[0];
            args->GradTemp->size[0] = tmp->size[1];
            invers_emxEnsureCapacity_real_T(args->GradTemp,
              inverse_kinematics_B.J);
            inverse_kinematics_B.kend = tmp->size[1];
            for (inverse_kinematics_B.b_k_c = 0; inverse_kinematics_B.b_k_c <
                 inverse_kinematics_B.kend; inverse_kinematics_B.b_k_c++) {
              args->GradTemp->data[inverse_kinematics_B.b_k_c] = tmp->
                data[inverse_kinematics_B.b_k_c];
            }

            inverse_kinematics_B.absxk = args->CostTemp;
            inverse_kinematics_B.scale = 1.0;
            while (inverse_kinematics_B.absxk > inverse_kinematics_B.cost) {
              inverse_kinematics_B.scale *= 2.5;
              inverse_kinematics_B.absxk = inverse_kinematics_B.scale *
                obj->DampingBias + inverse_kinematics_B.cc;
              if ((H0->size[0] == 4) && (H0->size[1] == 4)) {
                for (inverse_kinematics_B.b_k_c = 0; inverse_kinematics_B.b_k_c <
                     4; inverse_kinematics_B.b_k_c++) {
                  for (inverse_kinematics_B.kend = 0; inverse_kinematics_B.kend <
                       4; inverse_kinematics_B.kend++) {
                    inverse_kinematics_B.J = (inverse_kinematics_B.b_k_c << 2) +
                      inverse_kinematics_B.kend;
                    inverse_kinematics_B.Td[inverse_kinematics_B.J] = -(H0->
                      data[H0->size[0] * inverse_kinematics_B.b_k_c +
                      inverse_kinematics_B.kend] + tmp_0[inverse_kinematics_B.J]
                      * inverse_kinematics_B.absxk);
                  }
                }

                inverse_kinematics_mldivide(inverse_kinematics_B.Td, grad,
                  inverse_kinematics_B.step_data,
                  &inverse_kinematics_B.step_size);
              } else {
                inverse_kine_binary_expand_op_g(inverse_kinematics_B.step_data,
                  &inverse_kinematics_B.step_size, H0,
                  inverse_kinematics_B.absxk, tmp_0, grad);
              }

              args = obj->ExtraArgs;
              treeInternal = args->Robot;
              inverse_kinematics_B.J = bodyName->size[0] * bodyName->size[1];
              bodyName->size[0] = 1;
              bodyName->size[1] = args->BodyName->size[1];
              invers_emxEnsureCapacity_char_T(bodyName, inverse_kinematics_B.J);
              inverse_kinematics_B.kend = args->BodyName->size[1] - 1;
              for (inverse_kinematics_B.b_k_c = 0; inverse_kinematics_B.b_k_c <=
                   inverse_kinematics_B.kend; inverse_kinematics_B.b_k_c++) {
                inverse_kinematics_B.J = inverse_kinematics_B.b_k_c;
                bodyName->data[inverse_kinematics_B.J] = args->BodyName->
                  data[inverse_kinematics_B.J];
              }

              for (inverse_kinematics_B.b_k_c = 0; inverse_kinematics_B.b_k_c <
                   16; inverse_kinematics_B.b_k_c++) {
                inverse_kinematics_B.Td[inverse_kinematics_B.b_k_c] =
                  args->Tform[inverse_kinematics_B.b_k_c];
              }

              for (inverse_kinematics_B.b_k_c = 0; inverse_kinematics_B.b_k_c <
                   36; inverse_kinematics_B.b_k_c++) {
                inverse_kinematics_B.weightMatrix_m[inverse_kinematics_B.b_k_c] =
                  args->WeightMatrix[inverse_kinematics_B.b_k_c];
              }

              inverse_kinematics_B.y_g[0] = xSol[0] +
                inverse_kinematics_B.step_data[0];
              inverse_kinematics_B.y_g[1] = xSol[1] +
                inverse_kinematics_B.step_data[1];
              inverse_kinematics_B.y_g[2] = xSol[2] +
                inverse_kinematics_B.step_data[2];
              inverse_kinematics_B.y_g[3] = xSol[3] +
                inverse_kinematics_B.step_data[3];
              RigidBodyTree_efficientFKAndJac(treeInternal,
                inverse_kinematics_B.y_g, bodyName, inverse_kinematics_B.T_data,
                inverse_kinematics_B.T_size, J);
              inverse_kin_IKHelpers_poseError(inverse_kinematics_B.Td,
                inverse_kinematics_B.T_data, inverse_kinematics_B.T_size,
                inverse_kinematics_B.e);
              inverse_kinematics_B.J = args->ErrTemp->size[0];
              args->ErrTemp->size[0] = 6;
              invers_emxEnsureCapacity_real_T(args->ErrTemp,
                inverse_kinematics_B.J);
              for (inverse_kinematics_B.b_k_c = 0; inverse_kinematics_B.b_k_c <
                   6; inverse_kinematics_B.b_k_c++) {
                args->ErrTemp->data[inverse_kinematics_B.b_k_c] =
                  inverse_kinematics_B.e[inverse_kinematics_B.b_k_c];
              }

              inverse_kinematics_B.absxk = 0.0;
              for (inverse_kinematics_B.b_k_c = 0; inverse_kinematics_B.b_k_c <
                   6; inverse_kinematics_B.b_k_c++) {
                inverse_kinematics_B.e_n[inverse_kinematics_B.b_k_c] = 0.0;
                for (inverse_kinematics_B.kend = 0; inverse_kinematics_B.kend <
                     6; inverse_kinematics_B.kend++) {
                  inverse_kinematics_B.e_n[inverse_kinematics_B.b_k_c] +=
                    inverse_kinematics_B.weightMatrix_m[6 *
                    inverse_kinematics_B.b_k_c + inverse_kinematics_B.kend] *
                    (0.5 * inverse_kinematics_B.e[inverse_kinematics_B.kend]);
                }

                inverse_kinematics_B.absxk +=
                  inverse_kinematics_B.e_n[inverse_kinematics_B.b_k_c] *
                  inverse_kinematics_B.e[inverse_kinematics_B.b_k_c];
              }

              args->CostTemp = inverse_kinematics_B.absxk;
              for (inverse_kinematics_B.b_k_c = 0; inverse_kinematics_B.b_k_c <
                   6; inverse_kinematics_B.b_k_c++) {
                inverse_kinematics_B.e_n[inverse_kinematics_B.b_k_c] = 0.0;
                for (inverse_kinematics_B.kend = 0; inverse_kinematics_B.kend <
                     6; inverse_kinematics_B.kend++) {
                  inverse_kinematics_B.e_n[inverse_kinematics_B.b_k_c] +=
                    inverse_kinematics_B.weightMatrix_m[6 *
                    inverse_kinematics_B.b_k_c + inverse_kinematics_B.kend] *
                    inverse_kinematics_B.e[inverse_kinematics_B.kend];
                }
              }

              inverse_kinematics_B.J = J_0->size[0] * J_0->size[1];
              J_0->size[0] = 6;
              J_0->size[1] = J->size[1];
              invers_emxEnsureCapacity_real_T(J_0, inverse_kinematics_B.J);
              inverse_kinematics_B.kend = 6 * J->size[1];
              for (inverse_kinematics_B.b_k_c = 0; inverse_kinematics_B.b_k_c <
                   inverse_kinematics_B.kend; inverse_kinematics_B.b_k_c++) {
                J_0->data[inverse_kinematics_B.b_k_c] = -J->
                  data[inverse_kinematics_B.b_k_c];
              }

              inverse_kinematics_mtimes_g(inverse_kinematics_B.e_n, J_0, tmp);
              inverse_kinematics_B.J = args->GradTemp->size[0];
              args->GradTemp->size[0] = tmp->size[1];
              invers_emxEnsureCapacity_real_T(args->GradTemp,
                inverse_kinematics_B.J);
              inverse_kinematics_B.kend = tmp->size[1];
              for (inverse_kinematics_B.b_k_c = 0; inverse_kinematics_B.b_k_c <
                   inverse_kinematics_B.kend; inverse_kinematics_B.b_k_c++) {
                args->GradTemp->data[inverse_kinematics_B.b_k_c] = tmp->
                  data[inverse_kinematics_B.b_k_c];
              }

              inverse_kinematics_B.absxk = args->CostTemp;
            }

            inverse_kinematics_B.cost = xSol[0] +
              inverse_kinematics_B.step_data[0];
            xSol[0] += inverse_kinematics_B.step_data[0];
            inverse_kinematics_B.absxk = xSol[1] +
              inverse_kinematics_B.step_data[1];
            xSol[1] += inverse_kinematics_B.step_data[1];
            inverse_kinematics_B.cc = xSol[2] + inverse_kinematics_B.step_data[2];
            xSol[2] += inverse_kinematics_B.step_data[2];
            inverse_kinematics_B.scale = xSol[3] +
              inverse_kinematics_B.step_data[3];
            xSol[3] += inverse_kinematics_B.step_data[3];
            if (obj->ConstraintsOn) {
              args = obj->ExtraArgs;
              inverse_kinematics_B.kend = args->Limits->size[0];
              inverse_kinematics_B.J = grad->size[0];
              grad->size[0] = inverse_kinematics_B.kend;
              invers_emxEnsureCapacity_real_T(grad, inverse_kinematics_B.J);
              for (inverse_kinematics_B.b_k_c = 0; inverse_kinematics_B.b_k_c <
                   inverse_kinematics_B.kend; inverse_kinematics_B.b_k_c++) {
                grad->data[inverse_kinematics_B.b_k_c] = args->Limits->
                  data[inverse_kinematics_B.b_k_c];
              }

              if (grad->size[0] == 4) {
                if ((grad->data[0] >= inverse_kinematics_B.cost) || rtIsNaN
                    (inverse_kinematics_B.cost)) {
                  inverse_kinematics_B.y_g[0] = grad->data[0];
                } else {
                  inverse_kinematics_B.y_g[0] = inverse_kinematics_B.cost;
                }

                if ((grad->data[1] >= inverse_kinematics_B.absxk) || rtIsNaN
                    (inverse_kinematics_B.absxk)) {
                  inverse_kinematics_B.y_g[1] = grad->data[1];
                } else {
                  inverse_kinematics_B.y_g[1] = inverse_kinematics_B.absxk;
                }

                if ((grad->data[2] >= inverse_kinematics_B.cc) || rtIsNaN
                    (inverse_kinematics_B.cc)) {
                  inverse_kinematics_B.y_g[2] = grad->data[2];
                } else {
                  inverse_kinematics_B.y_g[2] = inverse_kinematics_B.cc;
                }

                if ((grad->data[3] >= inverse_kinematics_B.scale) || rtIsNaN
                    (inverse_kinematics_B.scale)) {
                  inverse_kinematics_B.y_g[3] = grad->data[3];
                } else {
                  inverse_kinematics_B.y_g[3] = inverse_kinematics_B.scale;
                }
              } else {
                inverse_kinematics_expand_max(grad, xSol,
                  inverse_kinematics_B.y_g);
              }

              inverse_kinematics_B.kend = args->Limits->size[0];
              inverse_kinematics_B.J = grad->size[0];
              grad->size[0] = inverse_kinematics_B.kend;
              invers_emxEnsureCapacity_real_T(grad, inverse_kinematics_B.J);
              for (inverse_kinematics_B.b_k_c = 0; inverse_kinematics_B.b_k_c <
                   inverse_kinematics_B.kend; inverse_kinematics_B.b_k_c++) {
                grad->data[inverse_kinematics_B.b_k_c] = args->Limits->
                  data[inverse_kinematics_B.b_k_c + args->Limits->size[0]];
              }

              if (grad->size[0] == 4) {
                if ((grad->data[0] <= inverse_kinematics_B.y_g[0]) || rtIsNaN
                    (inverse_kinematics_B.y_g[0])) {
                  xSol[0] = grad->data[0];
                } else {
                  xSol[0] = inverse_kinematics_B.y_g[0];
                }

                if ((grad->data[1] <= inverse_kinematics_B.y_g[1]) || rtIsNaN
                    (inverse_kinematics_B.y_g[1])) {
                  xSol[1] = grad->data[1];
                } else {
                  xSol[1] = inverse_kinematics_B.y_g[1];
                }

                if ((grad->data[2] <= inverse_kinematics_B.y_g[2]) || rtIsNaN
                    (inverse_kinematics_B.y_g[2])) {
                  xSol[2] = grad->data[2];
                } else {
                  xSol[2] = inverse_kinematics_B.y_g[2];
                }

                if ((grad->data[3] <= inverse_kinematics_B.y_g[3]) || rtIsNaN
                    (inverse_kinematics_B.y_g[3])) {
                  xSol[3] = grad->data[3];
                } else {
                  xSol[3] = inverse_kinematics_B.y_g[3];
                }
              } else {
                inverse_kinematics_expand_min(grad, inverse_kinematics_B.y_g,
                  xSol);
              }
            }

            inverse_kinematics_B.b_i_g++;
          }
        }
      }
    } else {
      args = obj->ExtraArgs;
      for (inverse_kinematics_B.b_k_c = 0; inverse_kinematics_B.b_k_c < 36;
           inverse_kinematics_B.b_k_c++) {
        inverse_kinematics_B.a[inverse_kinematics_B.b_k_c] = args->
          WeightMatrix[inverse_kinematics_B.b_k_c];
      }

      inverse_kinematics_B.J = b->size[0];
      b->size[0] = args->ErrTemp->size[0];
      invers_emxEnsureCapacity_real_T(b, inverse_kinematics_B.J);
      inverse_kinematics_B.kend = args->ErrTemp->size[0];
      for (inverse_kinematics_B.b_k_c = 0; inverse_kinematics_B.b_k_c <
           inverse_kinematics_B.kend; inverse_kinematics_B.b_k_c++) {
        b->data[inverse_kinematics_B.b_k_c] = args->ErrTemp->
          data[inverse_kinematics_B.b_k_c];
      }

      for (inverse_kinematics_B.b_k_c = 0; inverse_kinematics_B.b_k_c < 6;
           inverse_kinematics_B.b_k_c++) {
        inverse_kinematics_B.e[inverse_kinematics_B.b_k_c] = 0.0;
        for (inverse_kinematics_B.kend = 0; inverse_kinematics_B.kend < 6;
             inverse_kinematics_B.kend++) {
          inverse_kinematics_B.e[inverse_kinematics_B.b_k_c] +=
            inverse_kinematics_B.a[6 * inverse_kinematics_B.kend +
            inverse_kinematics_B.b_k_c] * b->data[inverse_kinematics_B.kend];
        }
      }

      *en = inverse_kinematics_norm_g(inverse_kinematics_B.e);
      *iter = obj->MaxNumIterationInternal;
      *exitFlag = IterationLimitExceeded;
      exitg1 = 1;
    }
  } while (exitg1 == 0);

  inverse_kinemati_emxFree_real_T(&J_0);
  inverse_kinemati_emxFree_real_T(&tmp);
  inverse_kinem_emxFree_boolean_T(&x);
  inverse_kinemati_emxFree_real_T(&b);
  inverse_kinemati_emxFree_real_T(&J);
  inverse_kinemati_emxFree_char_T(&bodyName);
  inverse_kinemati_emxFree_real_T(&y);
  inverse_kinemati_emxFree_real_T(&ev);
  inverse_kinemati_emxFree_real_T(&H0);
  inverse_kinemati_emxFree_real_T(&grad);
  inverse_kinemati_emxFree_real_T(&evprev);
}

static boolean_T inverse_kinematics_any(const emxArray_boolean_T_inverse_ki_T *x)
{
  int32_T ix;
  boolean_T exitg1;
  boolean_T y;
  y = false;
  ix = 0;
  exitg1 = false;
  while ((!exitg1) && (ix + 1 <= x->size[0])) {
    if (x->data[ix]) {
      y = true;
      exitg1 = true;
    } else {
      ix++;
    }
  }

  return y;
}

static void inverse_kinematics_randn(const real_T varargin_1[2],
  emxArray_real_T_inverse_kinem_T *r)
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
  inverse_kinematics_B.b_k_h = r->size[0];
  r->size[0] = static_cast<int32_T>(varargin_1[0]);
  invers_emxEnsureCapacity_real_T(r, inverse_kinematics_B.b_k_h);
  inverse_kinematics_B.d_tmp = static_cast<int32_T>(varargin_1[0]) - 1;
  if (static_cast<int32_T>(varargin_1[0]) - 1 >= 0) {
    inverse_kinematics_B.xi[0] = 0.0;
    inverse_kinematics_B.xi[1] = 0.215241895984875;
    inverse_kinematics_B.xi[2] = 0.286174591792068;
    inverse_kinematics_B.xi[3] = 0.335737519214422;
    inverse_kinematics_B.xi[4] = 0.375121332878378;
    inverse_kinematics_B.xi[5] = 0.408389134611989;
    inverse_kinematics_B.xi[6] = 0.43751840220787;
    inverse_kinematics_B.xi[7] = 0.46363433679088;
    inverse_kinematics_B.xi[8] = 0.487443966139235;
    inverse_kinematics_B.xi[9] = 0.50942332960209;
    inverse_kinematics_B.xi[10] = 0.529909720661557;
    inverse_kinematics_B.xi[11] = 0.549151702327164;
    inverse_kinematics_B.xi[12] = 0.567338257053817;
    inverse_kinematics_B.xi[13] = 0.584616766106378;
    inverse_kinematics_B.xi[14] = 0.601104617755991;
    inverse_kinematics_B.xi[15] = 0.61689699000775;
    inverse_kinematics_B.xi[16] = 0.63207223638606;
    inverse_kinematics_B.xi[17] = 0.646695714894993;
    inverse_kinematics_B.xi[18] = 0.660822574244419;
    inverse_kinematics_B.xi[19] = 0.674499822837293;
    inverse_kinematics_B.xi[20] = 0.687767892795788;
    inverse_kinematics_B.xi[21] = 0.700661841106814;
    inverse_kinematics_B.xi[22] = 0.713212285190975;
    inverse_kinematics_B.xi[23] = 0.725446140909999;
    inverse_kinematics_B.xi[24] = 0.737387211434295;
    inverse_kinematics_B.xi[25] = 0.749056662017815;
    inverse_kinematics_B.xi[26] = 0.760473406430107;
    inverse_kinematics_B.xi[27] = 0.771654424224568;
    inverse_kinematics_B.xi[28] = 0.782615023307232;
    inverse_kinematics_B.xi[29] = 0.793369058840623;
    inverse_kinematics_B.xi[30] = 0.80392911698997;
    inverse_kinematics_B.xi[31] = 0.814306670135215;
    inverse_kinematics_B.xi[32] = 0.824512208752291;
    inverse_kinematics_B.xi[33] = 0.834555354086381;
    inverse_kinematics_B.xi[34] = 0.844444954909153;
    inverse_kinematics_B.xi[35] = 0.854189171008163;
    inverse_kinematics_B.xi[36] = 0.863795545553308;
    inverse_kinematics_B.xi[37] = 0.87327106808886;
    inverse_kinematics_B.xi[38] = 0.882622229585165;
    inverse_kinematics_B.xi[39] = 0.891855070732941;
    inverse_kinematics_B.xi[40] = 0.900975224461221;
    inverse_kinematics_B.xi[41] = 0.909987953496718;
    inverse_kinematics_B.xi[42] = 0.91889818364959;
    inverse_kinematics_B.xi[43] = 0.927710533401999;
    inverse_kinematics_B.xi[44] = 0.936429340286575;
    inverse_kinematics_B.xi[45] = 0.945058684468165;
    inverse_kinematics_B.xi[46] = 0.953602409881086;
    inverse_kinematics_B.xi[47] = 0.96206414322304;
    inverse_kinematics_B.xi[48] = 0.970447311064224;
    inverse_kinematics_B.xi[49] = 0.978755155294224;
    inverse_kinematics_B.xi[50] = 0.986990747099062;
    inverse_kinematics_B.xi[51] = 0.99515699963509;
    inverse_kinematics_B.xi[52] = 1.00325667954467;
    inverse_kinematics_B.xi[53] = 1.01129241744;
    inverse_kinematics_B.xi[54] = 1.01926671746548;
    inverse_kinematics_B.xi[55] = 1.02718196603564;
    inverse_kinematics_B.xi[56] = 1.03504043983344;
    inverse_kinematics_B.xi[57] = 1.04284431314415;
    inverse_kinematics_B.xi[58] = 1.05059566459093;
    inverse_kinematics_B.xi[59] = 1.05829648333067;
    inverse_kinematics_B.xi[60] = 1.06594867476212;
    inverse_kinematics_B.xi[61] = 1.07355406579244;
    inverse_kinematics_B.xi[62] = 1.0811144097034;
    inverse_kinematics_B.xi[63] = 1.08863139065398;
    inverse_kinematics_B.xi[64] = 1.09610662785202;
    inverse_kinematics_B.xi[65] = 1.10354167942464;
    inverse_kinematics_B.xi[66] = 1.11093804601357;
    inverse_kinematics_B.xi[67] = 1.11829717411934;
    inverse_kinematics_B.xi[68] = 1.12562045921553;
    inverse_kinematics_B.xi[69] = 1.13290924865253;
    inverse_kinematics_B.xi[70] = 1.14016484436815;
    inverse_kinematics_B.xi[71] = 1.14738850542085;
    inverse_kinematics_B.xi[72] = 1.15458145035993;
    inverse_kinematics_B.xi[73] = 1.16174485944561;
    inverse_kinematics_B.xi[74] = 1.16887987673083;
    inverse_kinematics_B.xi[75] = 1.17598761201545;
    inverse_kinematics_B.xi[76] = 1.18306914268269;
    inverse_kinematics_B.xi[77] = 1.19012551542669;
    inverse_kinematics_B.xi[78] = 1.19715774787944;
    inverse_kinematics_B.xi[79] = 1.20416683014438;
    inverse_kinematics_B.xi[80] = 1.2111537262437;
    inverse_kinematics_B.xi[81] = 1.21811937548548;
    inverse_kinematics_B.xi[82] = 1.22506469375653;
    inverse_kinematics_B.xi[83] = 1.23199057474614;
    inverse_kinematics_B.xi[84] = 1.23889789110569;
    inverse_kinematics_B.xi[85] = 1.24578749554863;
    inverse_kinematics_B.xi[86] = 1.2526602218949;
    inverse_kinematics_B.xi[87] = 1.25951688606371;
    inverse_kinematics_B.xi[88] = 1.26635828701823;
    inverse_kinematics_B.xi[89] = 1.27318520766536;
    inverse_kinematics_B.xi[90] = 1.27999841571382;
    inverse_kinematics_B.xi[91] = 1.28679866449324;
    inverse_kinematics_B.xi[92] = 1.29358669373695;
    inverse_kinematics_B.xi[93] = 1.30036323033084;
    inverse_kinematics_B.xi[94] = 1.30712898903073;
    inverse_kinematics_B.xi[95] = 1.31388467315022;
    inverse_kinematics_B.xi[96] = 1.32063097522106;
    inverse_kinematics_B.xi[97] = 1.32736857762793;
    inverse_kinematics_B.xi[98] = 1.33409815321936;
    inverse_kinematics_B.xi[99] = 1.3408203658964;
    inverse_kinematics_B.xi[100] = 1.34753587118059;
    inverse_kinematics_B.xi[101] = 1.35424531676263;
    inverse_kinematics_B.xi[102] = 1.36094934303328;
    inverse_kinematics_B.xi[103] = 1.36764858359748;
    inverse_kinematics_B.xi[104] = 1.37434366577317;
    inverse_kinematics_B.xi[105] = 1.38103521107586;
    inverse_kinematics_B.xi[106] = 1.38772383568998;
    inverse_kinematics_B.xi[107] = 1.39441015092814;
    inverse_kinematics_B.xi[108] = 1.40109476367925;
    inverse_kinematics_B.xi[109] = 1.4077782768464;
    inverse_kinematics_B.xi[110] = 1.41446128977547;
    inverse_kinematics_B.xi[111] = 1.42114439867531;
    inverse_kinematics_B.xi[112] = 1.42782819703026;
    inverse_kinematics_B.xi[113] = 1.43451327600589;
    inverse_kinematics_B.xi[114] = 1.44120022484872;
    inverse_kinematics_B.xi[115] = 1.44788963128058;
    inverse_kinematics_B.xi[116] = 1.45458208188841;
    inverse_kinematics_B.xi[117] = 1.46127816251028;
    inverse_kinematics_B.xi[118] = 1.46797845861808;
    inverse_kinematics_B.xi[119] = 1.47468355569786;
    inverse_kinematics_B.xi[120] = 1.48139403962819;
    inverse_kinematics_B.xi[121] = 1.48811049705745;
    inverse_kinematics_B.xi[122] = 1.49483351578049;
    inverse_kinematics_B.xi[123] = 1.50156368511546;
    inverse_kinematics_B.xi[124] = 1.50830159628131;
    inverse_kinematics_B.xi[125] = 1.51504784277671;
    inverse_kinematics_B.xi[126] = 1.521803020761;
    inverse_kinematics_B.xi[127] = 1.52856772943771;
    inverse_kinematics_B.xi[128] = 1.53534257144151;
    inverse_kinematics_B.xi[129] = 1.542128153229;
    inverse_kinematics_B.xi[130] = 1.54892508547417;
    inverse_kinematics_B.xi[131] = 1.55573398346918;
    inverse_kinematics_B.xi[132] = 1.56255546753104;
    inverse_kinematics_B.xi[133] = 1.56939016341512;
    inverse_kinematics_B.xi[134] = 1.57623870273591;
    inverse_kinematics_B.xi[135] = 1.58310172339603;
    inverse_kinematics_B.xi[136] = 1.58997987002419;
    inverse_kinematics_B.xi[137] = 1.59687379442279;
    inverse_kinematics_B.xi[138] = 1.60378415602609;
    inverse_kinematics_B.xi[139] = 1.61071162236983;
    inverse_kinematics_B.xi[140] = 1.61765686957301;
    inverse_kinematics_B.xi[141] = 1.62462058283303;
    inverse_kinematics_B.xi[142] = 1.63160345693487;
    inverse_kinematics_B.xi[143] = 1.63860619677555;
    inverse_kinematics_B.xi[144] = 1.64562951790478;
    inverse_kinematics_B.xi[145] = 1.65267414708306;
    inverse_kinematics_B.xi[146] = 1.65974082285818;
    inverse_kinematics_B.xi[147] = 1.66683029616166;
    inverse_kinematics_B.xi[148] = 1.67394333092612;
    inverse_kinematics_B.xi[149] = 1.68108070472517;
    inverse_kinematics_B.xi[150] = 1.68824320943719;
    inverse_kinematics_B.xi[151] = 1.69543165193456;
    inverse_kinematics_B.xi[152] = 1.70264685479992;
    inverse_kinematics_B.xi[153] = 1.7098896570713;
    inverse_kinematics_B.xi[154] = 1.71716091501782;
    inverse_kinematics_B.xi[155] = 1.72446150294804;
    inverse_kinematics_B.xi[156] = 1.73179231405296;
    inverse_kinematics_B.xi[157] = 1.73915426128591;
    inverse_kinematics_B.xi[158] = 1.74654827828172;
    inverse_kinematics_B.xi[159] = 1.75397532031767;
    inverse_kinematics_B.xi[160] = 1.76143636531891;
    inverse_kinematics_B.xi[161] = 1.76893241491127;
    inverse_kinematics_B.xi[162] = 1.77646449552452;
    inverse_kinematics_B.xi[163] = 1.78403365954944;
    inverse_kinematics_B.xi[164] = 1.79164098655216;
    inverse_kinematics_B.xi[165] = 1.79928758454972;
    inverse_kinematics_B.xi[166] = 1.80697459135082;
    inverse_kinematics_B.xi[167] = 1.81470317596628;
    inverse_kinematics_B.xi[168] = 1.82247454009388;
    inverse_kinematics_B.xi[169] = 1.83028991968276;
    inverse_kinematics_B.xi[170] = 1.83815058658281;
    inverse_kinematics_B.xi[171] = 1.84605785028518;
    inverse_kinematics_B.xi[172] = 1.8540130597602;
    inverse_kinematics_B.xi[173] = 1.86201760539967;
    inverse_kinematics_B.xi[174] = 1.87007292107127;
    inverse_kinematics_B.xi[175] = 1.878180486293;
    inverse_kinematics_B.xi[176] = 1.88634182853678;
    inverse_kinematics_B.xi[177] = 1.8945585256707;
    inverse_kinematics_B.xi[178] = 1.90283220855043;
    inverse_kinematics_B.xi[179] = 1.91116456377125;
    inverse_kinematics_B.xi[180] = 1.91955733659319;
    inverse_kinematics_B.xi[181] = 1.92801233405266;
    inverse_kinematics_B.xi[182] = 1.93653142827569;
    inverse_kinematics_B.xi[183] = 1.94511656000868;
    inverse_kinematics_B.xi[184] = 1.95376974238465;
    inverse_kinematics_B.xi[185] = 1.96249306494436;
    inverse_kinematics_B.xi[186] = 1.97128869793366;
    inverse_kinematics_B.xi[187] = 1.98015889690048;
    inverse_kinematics_B.xi[188] = 1.98910600761744;
    inverse_kinematics_B.xi[189] = 1.99813247135842;
    inverse_kinematics_B.xi[190] = 2.00724083056053;
    inverse_kinematics_B.xi[191] = 2.0164337349062;
    inverse_kinematics_B.xi[192] = 2.02571394786385;
    inverse_kinematics_B.xi[193] = 2.03508435372962;
    inverse_kinematics_B.xi[194] = 2.04454796521753;
    inverse_kinematics_B.xi[195] = 2.05410793165065;
    inverse_kinematics_B.xi[196] = 2.06376754781173;
    inverse_kinematics_B.xi[197] = 2.07353026351874;
    inverse_kinematics_B.xi[198] = 2.0833996939983;
    inverse_kinematics_B.xi[199] = 2.09337963113879;
    inverse_kinematics_B.xi[200] = 2.10347405571488;
    inverse_kinematics_B.xi[201] = 2.11368715068665;
    inverse_kinematics_B.xi[202] = 2.12402331568952;
    inverse_kinematics_B.xi[203] = 2.13448718284602;
    inverse_kinematics_B.xi[204] = 2.14508363404789;
    inverse_kinematics_B.xi[205] = 2.15581781987674;
    inverse_kinematics_B.xi[206] = 2.16669518035431;
    inverse_kinematics_B.xi[207] = 2.17772146774029;
    inverse_kinematics_B.xi[208] = 2.18890277162636;
    inverse_kinematics_B.xi[209] = 2.20024554661128;
    inverse_kinematics_B.xi[210] = 2.21175664288416;
    inverse_kinematics_B.xi[211] = 2.22344334009251;
    inverse_kinematics_B.xi[212] = 2.23531338492992;
    inverse_kinematics_B.xi[213] = 2.24737503294739;
    inverse_kinematics_B.xi[214] = 2.25963709517379;
    inverse_kinematics_B.xi[215] = 2.27210899022838;
    inverse_kinematics_B.xi[216] = 2.28480080272449;
    inverse_kinematics_B.xi[217] = 2.29772334890286;
    inverse_kinematics_B.xi[218] = 2.31088825060137;
    inverse_kinematics_B.xi[219] = 2.32430801887113;
    inverse_kinematics_B.xi[220] = 2.33799614879653;
    inverse_kinematics_B.xi[221] = 2.35196722737914;
    inverse_kinematics_B.xi[222] = 2.36623705671729;
    inverse_kinematics_B.xi[223] = 2.38082279517208;
    inverse_kinematics_B.xi[224] = 2.39574311978193;
    inverse_kinematics_B.xi[225] = 2.41101841390112;
    inverse_kinematics_B.xi[226] = 2.42667098493715;
    inverse_kinematics_B.xi[227] = 2.44272531820036;
    inverse_kinematics_B.xi[228] = 2.4592083743347;
    inverse_kinematics_B.xi[229] = 2.47614993967052;
    inverse_kinematics_B.xi[230] = 2.49358304127105;
    inverse_kinematics_B.xi[231] = 2.51154444162669;
    inverse_kinematics_B.xi[232] = 2.53007523215985;
    inverse_kinematics_B.xi[233] = 2.54922155032478;
    inverse_kinematics_B.xi[234] = 2.56903545268184;
    inverse_kinematics_B.xi[235] = 2.58957598670829;
    inverse_kinematics_B.xi[236] = 2.61091051848882;
    inverse_kinematics_B.xi[237] = 2.63311639363158;
    inverse_kinematics_B.xi[238] = 2.65628303757674;
    inverse_kinematics_B.xi[239] = 2.68051464328574;
    inverse_kinematics_B.xi[240] = 2.70593365612306;
    inverse_kinematics_B.xi[241] = 2.73268535904401;
    inverse_kinematics_B.xi[242] = 2.76094400527999;
    inverse_kinematics_B.xi[243] = 2.79092117400193;
    inverse_kinematics_B.xi[244] = 2.82287739682644;
    inverse_kinematics_B.xi[245] = 2.85713873087322;
    inverse_kinematics_B.xi[246] = 2.89412105361341;
    inverse_kinematics_B.xi[247] = 2.93436686720889;
    inverse_kinematics_B.xi[248] = 2.97860327988184;
    inverse_kinematics_B.xi[249] = 3.02783779176959;
    inverse_kinematics_B.xi[250] = 3.08352613200214;
    inverse_kinematics_B.xi[251] = 3.147889289518;
    inverse_kinematics_B.xi[252] = 3.2245750520478;
    inverse_kinematics_B.xi[253] = 3.32024473383983;
    inverse_kinematics_B.xi[254] = 3.44927829856143;
    inverse_kinematics_B.xi[255] = 3.65415288536101;
    inverse_kinematics_B.xi[256] = 3.91075795952492;
    fitab = &tmp[0];
  }

  for (inverse_kinematics_B.b_k_h = 0; inverse_kinematics_B.b_k_h <=
       inverse_kinematics_B.d_tmp; inverse_kinematics_B.b_k_h++) {
    int32_T exitg1;
    do {
      exitg1 = 0;
      inverse_genrand_uint32_vector_g(inverse_kinematics_DW.state_c,
        inverse_kinematics_B.u32);
      inverse_kinematics_B.i_l5 = static_cast<int32_T>
        ((inverse_kinematics_B.u32[1] >> 24U) + 1U);
      inverse_kinematics_B.b_r = ((static_cast<real_T>(inverse_kinematics_B.u32
        [0] >> 3U) * 1.6777216E+7 + static_cast<real_T>(static_cast<int32_T>
        (inverse_kinematics_B.u32[1]) & 16777215)) * 2.2204460492503131E-16 -
        1.0) * inverse_kinematics_B.xi[inverse_kinematics_B.i_l5];
      if (fabs(inverse_kinematics_B.b_r) <=
          inverse_kinematics_B.xi[inverse_kinematics_B.i_l5 - 1]) {
        exitg1 = 1;
      } else if (inverse_kinematics_B.i_l5 < 256) {
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
        int32_T exitg2;
        do {
          exitg2 = 0;
          inverse_genrand_uint32_vector_g(inverse_kinematics_DW.state_c,
            inverse_kinematics_B.u32);
          inverse_kinematics_B.x = (static_cast<real_T>
            (inverse_kinematics_B.u32[0] >> 5U) * 6.7108864E+7 +
            static_cast<real_T>(inverse_kinematics_B.u32[1] >> 6U)) *
            1.1102230246251565E-16;
          if (inverse_kinematics_B.x == 0.0) {
            if (!inverse_kinemati_is_valid_state(inverse_kinematics_DW.state_c))
            {
              inverse_kinematics_DW.state_c[0] = 5489U;
              inverse_kinematics_DW.state_c[624] = 624U;
            }
          } else {
            exitg2 = 1;
          }
        } while (exitg2 == 0);

        if ((fitab[inverse_kinematics_B.i_l5 - 1] -
             fitab[inverse_kinematics_B.i_l5]) * inverse_kinematics_B.x +
            fitab[inverse_kinematics_B.i_l5] < exp(-0.5 *
             inverse_kinematics_B.b_r * inverse_kinematics_B.b_r)) {
          exitg1 = 1;
        }
      } else {
        do {
          int32_T exitg2;

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
            exitg2 = 0;
            inverse_genrand_uint32_vector_g(inverse_kinematics_DW.state_c,
              inverse_kinematics_B.u32);
            inverse_kinematics_B.x = (static_cast<real_T>
              (inverse_kinematics_B.u32[0] >> 5U) * 6.7108864E+7 +
              static_cast<real_T>(inverse_kinematics_B.u32[1] >> 6U)) *
              1.1102230246251565E-16;
            if (inverse_kinematics_B.x == 0.0) {
              if (!inverse_kinemati_is_valid_state(inverse_kinematics_DW.state_c))
              {
                inverse_kinematics_DW.state_c[0] = 5489U;
                inverse_kinematics_DW.state_c[624] = 624U;
              }
            } else {
              exitg2 = 1;
            }
          } while (exitg2 == 0);

          inverse_kinematics_B.x = log(inverse_kinematics_B.x) *
            0.273661237329758;

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
            exitg2 = 0;
            inverse_genrand_uint32_vector_g(inverse_kinematics_DW.state_c,
              inverse_kinematics_B.u32);
            inverse_kinematics_B.d_u = (static_cast<real_T>
              (inverse_kinematics_B.u32[0] >> 5U) * 6.7108864E+7 +
              static_cast<real_T>(inverse_kinematics_B.u32[1] >> 6U)) *
              1.1102230246251565E-16;
            if (inverse_kinematics_B.d_u == 0.0) {
              if (!inverse_kinemati_is_valid_state(inverse_kinematics_DW.state_c))
              {
                inverse_kinematics_DW.state_c[0] = 5489U;
                inverse_kinematics_DW.state_c[624] = 624U;
              }
            } else {
              exitg2 = 1;
            }
          } while (exitg2 == 0);
        } while (!(-2.0 * log(inverse_kinematics_B.d_u) > inverse_kinematics_B.x
                   * inverse_kinematics_B.x));

        if (inverse_kinematics_B.b_r < 0.0) {
          inverse_kinematics_B.b_r = inverse_kinematics_B.x - 3.65415288536101;
        } else {
          inverse_kinematics_B.b_r = 3.65415288536101 - inverse_kinematics_B.x;
        }

        exitg1 = 1;
      }
    } while (exitg1 == 0);

    r->data[inverse_kinematics_B.b_k_h] = inverse_kinematics_B.b_r;
  }
}

static void inverse_kinematics_minus(emxArray_real_T_inverse_kinem_T *in1, const
  emxArray_real_T_inverse_kinem_T *in2)
{
  emxArray_real_T_inverse_kinem_T *in1_0;
  int32_T i;
  int32_T loop_ub;
  int32_T stride_0_0;
  int32_T stride_1_0;
  inverse_kinemati_emxInit_real_T(&in1_0, 1);
  i = in1_0->size[0];
  in1_0->size[0] = in2->size[0] == 1 ? in1->size[0] : in2->size[0];
  invers_emxEnsureCapacity_real_T(in1_0, i);
  stride_0_0 = (in1->size[0] != 1);
  stride_1_0 = (in2->size[0] != 1);
  loop_ub = in2->size[0] == 1 ? in1->size[0] : in2->size[0];
  for (i = 0; i < loop_ub; i++) {
    in1_0->data[i] = in1->data[i * stride_0_0] - in2->data[i * stride_1_0];
  }

  i = in1->size[0];
  in1->size[0] = in1_0->size[0];
  invers_emxEnsureCapacity_real_T(in1, i);
  loop_ub = in1_0->size[0];
  if (loop_ub - 1 >= 0) {
    memcpy(&in1->data[0], &in1_0->data[0], loop_ub * sizeof(real_T));
  }

  inverse_kinemati_emxFree_real_T(&in1_0);
}

static void inverse_kinematics_plus(emxArray_real_T_inverse_kinem_T *in1, const
  emxArray_real_T_inverse_kinem_T *in2)
{
  emxArray_real_T_inverse_kinem_T *in2_0;
  int32_T i;
  int32_T loop_ub;
  int32_T stride_0_0;
  int32_T stride_1_0;
  inverse_kinemati_emxInit_real_T(&in2_0, 1);
  i = in2_0->size[0];
  in2_0->size[0] = in1->size[0] == 1 ? in2->size[0] : in1->size[0];
  invers_emxEnsureCapacity_real_T(in2_0, i);
  stride_0_0 = (in2->size[0] != 1);
  stride_1_0 = (in1->size[0] != 1);
  loop_ub = in1->size[0] == 1 ? in2->size[0] : in1->size[0];
  for (i = 0; i < loop_ub; i++) {
    in2_0->data[i] = in2->data[i * stride_0_0] + in1->data[i * stride_1_0];
  }

  i = in1->size[0];
  in1->size[0] = in2_0->size[0];
  invers_emxEnsureCapacity_real_T(in1, i);
  loop_ub = in2_0->size[0];
  if (loop_ub - 1 >= 0) {
    memcpy(&in1->data[0], &in2_0->data[0], loop_ub * sizeof(real_T));
  }

  inverse_kinemati_emxFree_real_T(&in2_0);
}

static void inverse_kinematics_rand_g(real_T varargin_1,
  emxArray_real_T_inverse_kinem_T *r)
{
  int32_T b_k;
  int32_T d;
  uint32_T b_u[2];
  b_k = r->size[0];
  r->size[0] = static_cast<int32_T>(varargin_1);
  invers_emxEnsureCapacity_real_T(r, b_k);
  d = static_cast<int32_T>(varargin_1) - 1;
  for (b_k = 0; b_k <= d; b_k++) {
    real_T b_r;

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
    int32_T exitg1;
    do {
      exitg1 = 0;
      inverse_genrand_uint32_vector_g(inverse_kinematics_DW.state_c, b_u);
      b_r = (static_cast<real_T>(b_u[0] >> 5U) * 6.7108864E+7 + static_cast<
             real_T>(b_u[1] >> 6U)) * 1.1102230246251565E-16;
      if (b_r == 0.0) {
        if (!inverse_kinemati_is_valid_state(inverse_kinematics_DW.state_c)) {
          inverse_kinematics_DW.state_c[0] = 5489U;
          inverse_kinematics_DW.state_c[624] = 624U;
        }
      } else {
        exitg1 = 1;
      }
    } while (exitg1 == 0);

    r->data[b_k] = b_r;
  }
}

static void inverse_kinema_binary_expand_op(emxArray_real_T_inverse_kinem_T *in1,
  const emxArray_real_T_inverse_kinem_T *in2, const
  emxArray_real_T_inverse_kinem_T *in3)
{
  emxArray_real_T_inverse_kinem_T *in2_0;
  int32_T i;
  int32_T loop_ub;
  int32_T stride_0_0;
  int32_T stride_1_0;
  int32_T stride_2_0;
  int32_T stride_3_0;
  inverse_kinemati_emxInit_real_T(&in2_0, 1);
  i = in2_0->size[0];
  in2_0->size[0] = ((in2->size[0] == 1 ? in1->size[0] : in2->size[0]) == 1 ?
                    in3->size[0] : in2->size[0] == 1 ? in1->size[0] : in2->size
                    [0]) == 1 ? in2->size[0] : (in2->size[0] == 1 ? in1->size[0]
    : in2->size[0]) == 1 ? in3->size[0] : in2->size[0] == 1 ? in1->size[0] :
    in2->size[0];
  invers_emxEnsureCapacity_real_T(in2_0, i);
  stride_0_0 = (in2->size[0] != 1);
  stride_1_0 = (in3->size[0] != 1);
  stride_2_0 = (in1->size[0] != 1);
  stride_3_0 = (in2->size[0] != 1);
  loop_ub = ((in2->size[0] == 1 ? in1->size[0] : in2->size[0]) == 1 ? in3->size
             [0] : in2->size[0] == 1 ? in1->size[0] : in2->size[0]) == 1 ?
    in2->size[0] : (in2->size[0] == 1 ? in1->size[0] : in2->size[0]) == 1 ?
    in3->size[0] : in2->size[0] == 1 ? in1->size[0] : in2->size[0];
  for (i = 0; i < loop_ub; i++) {
    in2_0->data[i] = (in1->data[i * stride_2_0] - in2->data[i * stride_3_0]) *
      in3->data[i * stride_1_0] + in2->data[i * stride_0_0];
  }

  i = in1->size[0];
  in1->size[0] = in2_0->size[0];
  invers_emxEnsureCapacity_real_T(in1, i);
  loop_ub = in2_0->size[0];
  if (loop_ub - 1 >= 0) {
    memcpy(&in1->data[0], &in2_0->data[0], loop_ub * sizeof(real_T));
  }

  inverse_kinemati_emxFree_real_T(&in2_0);
}

static void invers_NLPSolverInterface_solve(h_robotics_core_internal_Erro_T *obj,
  const real_T seed[4], real_T xSol[4], real_T *solutionInfo_Iterations, real_T *
  solutionInfo_RRAttempts, real_T *solutionInfo_Error, real_T
  *solutionInfo_ExitFlag, char_T solutionInfo_Status_data[], int32_T
  solutionInfo_Status_size[2])
{
  c_rigidBodyJoint_inverse_kine_T *obj_1;
  emxArray_boolean_T_inverse_ki_T *b;
  emxArray_boolean_T_inverse_ki_T *tmp;
  emxArray_boolean_T_inverse_ki_T *tmp_0;
  emxArray_real_T_inverse_kinem_T *lb;
  emxArray_real_T_inverse_kinem_T *newseed;
  emxArray_real_T_inverse_kinem_T *rn;
  emxArray_real_T_inverse_kinem_T *ub;
  f_robotics_manip_internal_IKE_T *args;
  v_robotics_manip_internal_Rig_T *obj_0;
  static const char_T tmp_1[14] = { 'b', 'e', 's', 't', ' ', 'a', 'v', 'a', 'i',
    'l', 'a', 'b', 'l', 'e' };

  static const char_T tmp_2[7] = { 's', 'u', 'c', 'c', 'e', 's', 's' };

  boolean_T exitg1;
  obj->MaxNumIterationInternal = obj->MaxNumIteration;
  obj->MaxTimeInternal = obj->MaxTime;
  obj->SeedInternal[0] = seed[0];
  obj->SeedInternal[1] = seed[1];
  obj->SeedInternal[2] = seed[2];
  obj->SeedInternal[3] = seed[3];
  inverse_kinematics_B.tol = obj->SolutionTolerance;
  inverse_kinematics_tic(&obj->TimeObj.StartTime.tv_sec,
    &obj->TimeObj.StartTime.tv_nsec);
  ErrorDampedLevenbergMarquardt_s(obj, xSol, &inverse_kinematics_B.exitFlag,
    &inverse_kinematics_B.err, &inverse_kinematics_B.iter);
  *solutionInfo_RRAttempts = 0.0;
  *solutionInfo_Iterations = inverse_kinematics_B.iter;
  *solutionInfo_Error = inverse_kinematics_B.err;
  inverse_kinematics_B.exitFlagPrev = inverse_kinematics_B.exitFlag;
  inverse_kinemati_emxInit_real_T(&newseed, 1);
  inverse_kinemati_emxInit_real_T(&ub, 1);
  inverse_kinemati_emxInit_real_T(&lb, 1);
  inverse_kinemati_emxInit_real_T(&rn, 1);
  inverse_kinem_emxInit_boolean_T(&b, 1);
  inverse_kinem_emxInit_boolean_T(&tmp, 1);
  inverse_kinem_emxInit_boolean_T(&tmp_0, 1);
  exitg1 = false;
  while ((!exitg1) && (obj->RandomRestart && (inverse_kinematics_B.err >
           inverse_kinematics_B.tol))) {
    obj->MaxNumIterationInternal -= inverse_kinematics_B.iter;
    inverse_kinematics_B.err = inverse_kinematics_toc
      (obj->TimeObj.StartTime.tv_sec, obj->TimeObj.StartTime.tv_nsec);
    obj->MaxTimeInternal = obj->MaxTime - inverse_kinematics_B.err;
    if (obj->MaxNumIterationInternal <= 0.0) {
      inverse_kinematics_B.exitFlag = IterationLimitExceeded;
    }

    if ((inverse_kinematics_B.exitFlag == IterationLimitExceeded) ||
        (inverse_kinematics_B.exitFlag == TimeLimitExceeded)) {
      inverse_kinematics_B.exitFlagPrev = inverse_kinematics_B.exitFlag;
      exitg1 = true;
    } else {
      args = obj->ExtraArgs;
      obj_0 = args->Robot;
      inverse_kinematics_B.ix = newseed->size[0];
      newseed->size[0] = static_cast<int32_T>(obj_0->PositionNumber);
      invers_emxEnsureCapacity_real_T(newseed, inverse_kinematics_B.ix);
      inverse_kinematics_B.nx = static_cast<int32_T>(obj_0->PositionNumber);
      if (inverse_kinematics_B.nx - 1 >= 0) {
        memset(&newseed->data[0], 0, inverse_kinematics_B.nx * sizeof(real_T));
      }

      inverse_kinematics_B.err = obj_0->NumBodies;
      inverse_kinematics_B.c_f = static_cast<int32_T>(inverse_kinematics_B.err)
        - 1;
      for (inverse_kinematics_B.b_i = 0; inverse_kinematics_B.b_i <=
           inverse_kinematics_B.c_f; inverse_kinematics_B.b_i++) {
        inverse_kinematics_B.err = obj_0->
          PositionDoFMap[inverse_kinematics_B.b_i];
        inverse_kinematics_B.iter = obj_0->
          PositionDoFMap[inverse_kinematics_B.b_i + 7];
        if (inverse_kinematics_B.err <= inverse_kinematics_B.iter) {
          obj_1 = obj_0->Bodies[inverse_kinematics_B.b_i]->JointInternal;
          if (static_cast<int32_T>(obj_1->PositionNumber) == 0) {
            inverse_kinematics_B.ix = ub->size[0];
            ub->size[0] = 1;
            invers_emxEnsureCapacity_real_T(ub, inverse_kinematics_B.ix);
            ub->data[0] = (rtNaN);
          } else {
            boolean_T exitg2;
            boolean_T guard1 = false;
            boolean_T guard2 = false;
            boolean_T guard3 = false;
            inverse_kinematics_B.nx = obj_1->PositionLimitsInternal->size[0];
            inverse_kinematics_B.ix = ub->size[0];
            ub->size[0] = inverse_kinematics_B.nx;
            invers_emxEnsureCapacity_real_T(ub, inverse_kinematics_B.ix);
            for (inverse_kinematics_B.ix = 0; inverse_kinematics_B.ix <
                 inverse_kinematics_B.nx; inverse_kinematics_B.ix++) {
              ub->data[inverse_kinematics_B.ix] = obj_1->
                PositionLimitsInternal->data[inverse_kinematics_B.ix +
                obj_1->PositionLimitsInternal->size[0]];
            }

            inverse_kinematics_B.nx = obj_1->PositionLimitsInternal->size[0];
            inverse_kinematics_B.ix = lb->size[0];
            lb->size[0] = inverse_kinematics_B.nx;
            invers_emxEnsureCapacity_real_T(lb, inverse_kinematics_B.ix);
            for (inverse_kinematics_B.ix = 0; inverse_kinematics_B.ix <
                 inverse_kinematics_B.nx; inverse_kinematics_B.ix++) {
              lb->data[inverse_kinematics_B.ix] = obj_1->
                PositionLimitsInternal->data[inverse_kinematics_B.ix];
            }

            inverse_kinematics_B.ix = b->size[0];
            b->size[0] = lb->size[0];
            inv_emxEnsureCapacity_boolean_T(b, inverse_kinematics_B.ix);
            inverse_kinematics_B.nx = lb->size[0];
            for (inverse_kinematics_B.ix = 0; inverse_kinematics_B.ix <
                 inverse_kinematics_B.nx; inverse_kinematics_B.ix++) {
              b->data[inverse_kinematics_B.ix] = rtIsInf(lb->
                data[inverse_kinematics_B.ix]);
            }

            inverse_kinematics_B.ix = tmp->size[0];
            tmp->size[0] = lb->size[0];
            inv_emxEnsureCapacity_boolean_T(tmp, inverse_kinematics_B.ix);
            inverse_kinematics_B.nx = lb->size[0];
            for (inverse_kinematics_B.ix = 0; inverse_kinematics_B.ix <
                 inverse_kinematics_B.nx; inverse_kinematics_B.ix++) {
              tmp->data[inverse_kinematics_B.ix] = rtIsNaN(lb->
                data[inverse_kinematics_B.ix]);
            }

            inverse_kinematics_B.nx = b->size[0];
            for (inverse_kinematics_B.ix = 0; inverse_kinematics_B.ix <
                 inverse_kinematics_B.nx; inverse_kinematics_B.ix++) {
              b->data[inverse_kinematics_B.ix] = ((!b->
                data[inverse_kinematics_B.ix]) && (!tmp->
                data[inverse_kinematics_B.ix]));
            }

            inverse_kinematics_B.y_h = true;
            inverse_kinematics_B.ix = 0;
            exitg2 = false;
            while ((!exitg2) && (inverse_kinematics_B.ix + 1 <= b->size[0])) {
              if (!b->data[inverse_kinematics_B.ix]) {
                inverse_kinematics_B.y_h = false;
                exitg2 = true;
              } else {
                inverse_kinematics_B.ix++;
              }
            }

            guard1 = false;
            guard2 = false;
            guard3 = false;
            if (inverse_kinematics_B.y_h) {
              inverse_kinematics_B.ix = b->size[0];
              b->size[0] = ub->size[0];
              inv_emxEnsureCapacity_boolean_T(b, inverse_kinematics_B.ix);
              inverse_kinematics_B.nx = ub->size[0];
              for (inverse_kinematics_B.ix = 0; inverse_kinematics_B.ix <
                   inverse_kinematics_B.nx; inverse_kinematics_B.ix++) {
                b->data[inverse_kinematics_B.ix] = rtIsInf(ub->
                  data[inverse_kinematics_B.ix]);
              }

              inverse_kinematics_B.ix = tmp->size[0];
              tmp->size[0] = ub->size[0];
              inv_emxEnsureCapacity_boolean_T(tmp, inverse_kinematics_B.ix);
              inverse_kinematics_B.nx = ub->size[0];
              for (inverse_kinematics_B.ix = 0; inverse_kinematics_B.ix <
                   inverse_kinematics_B.nx; inverse_kinematics_B.ix++) {
                tmp->data[inverse_kinematics_B.ix] = rtIsNaN(ub->
                  data[inverse_kinematics_B.ix]);
              }

              inverse_kinematics_B.nx = b->size[0];
              for (inverse_kinematics_B.ix = 0; inverse_kinematics_B.ix <
                   inverse_kinematics_B.nx; inverse_kinematics_B.ix++) {
                b->data[inverse_kinematics_B.ix] = ((!b->
                  data[inverse_kinematics_B.ix]) && (!tmp->
                  data[inverse_kinematics_B.ix]));
              }

              inverse_kinematics_B.y_h = true;
              inverse_kinematics_B.ix = 0;
              exitg2 = false;
              while ((!exitg2) && (inverse_kinematics_B.ix + 1 <= b->size[0])) {
                if (!b->data[inverse_kinematics_B.ix]) {
                  inverse_kinematics_B.y_h = false;
                  exitg2 = true;
                } else {
                  inverse_kinematics_B.ix++;
                }
              }

              if (inverse_kinematics_B.y_h) {
                inverse_kinematics_rand_g(obj_1->PositionNumber, rn);
                if ((ub->size[0] == lb->size[0]) && ((ub->size[0] == 1 ?
                      lb->size[0] : ub->size[0]) == rn->size[0]) && ((rn->size[0]
                      == 1 ? ub->size[0] == 1 ? lb->size[0] : ub->size[0] :
                      rn->size[0]) == lb->size[0])) {
                  inverse_kinematics_B.ix = ub->size[0];
                  ub->size[0] = lb->size[0];
                  invers_emxEnsureCapacity_real_T(ub, inverse_kinematics_B.ix);
                  inverse_kinematics_B.nx = lb->size[0];
                  for (inverse_kinematics_B.ix = 0; inverse_kinematics_B.ix <
                       inverse_kinematics_B.nx; inverse_kinematics_B.ix++) {
                    inverse_kinematics_B.lb = lb->data[inverse_kinematics_B.ix];
                    ub->data[inverse_kinematics_B.ix] = (ub->
                      data[inverse_kinematics_B.ix] - inverse_kinematics_B.lb) *
                      rn->data[inverse_kinematics_B.ix] +
                      inverse_kinematics_B.lb;
                  }
                } else {
                  inverse_kinema_binary_expand_op(ub, lb, rn);
                }
              } else {
                guard3 = true;
              }
            } else {
              guard3 = true;
            }

            if (guard3) {
              inverse_kinematics_B.ix = b->size[0];
              b->size[0] = lb->size[0];
              inv_emxEnsureCapacity_boolean_T(b, inverse_kinematics_B.ix);
              inverse_kinematics_B.nx = lb->size[0];
              for (inverse_kinematics_B.ix = 0; inverse_kinematics_B.ix <
                   inverse_kinematics_B.nx; inverse_kinematics_B.ix++) {
                b->data[inverse_kinematics_B.ix] = rtIsInf(lb->
                  data[inverse_kinematics_B.ix]);
              }

              inverse_kinematics_B.ix = tmp->size[0];
              tmp->size[0] = lb->size[0];
              inv_emxEnsureCapacity_boolean_T(tmp, inverse_kinematics_B.ix);
              inverse_kinematics_B.nx = lb->size[0];
              for (inverse_kinematics_B.ix = 0; inverse_kinematics_B.ix <
                   inverse_kinematics_B.nx; inverse_kinematics_B.ix++) {
                tmp->data[inverse_kinematics_B.ix] = rtIsNaN(lb->
                  data[inverse_kinematics_B.ix]);
              }

              inverse_kinematics_B.nx = b->size[0];
              for (inverse_kinematics_B.ix = 0; inverse_kinematics_B.ix <
                   inverse_kinematics_B.nx; inverse_kinematics_B.ix++) {
                b->data[inverse_kinematics_B.ix] = ((!b->
                  data[inverse_kinematics_B.ix]) && (!tmp->
                  data[inverse_kinematics_B.ix]));
              }

              inverse_kinematics_B.y_h = true;
              inverse_kinematics_B.ix = 0;
              exitg2 = false;
              while ((!exitg2) && (inverse_kinematics_B.ix + 1 <= b->size[0])) {
                if (!b->data[inverse_kinematics_B.ix]) {
                  inverse_kinematics_B.y_h = false;
                  exitg2 = true;
                } else {
                  inverse_kinematics_B.ix++;
                }
              }

              if (inverse_kinematics_B.y_h) {
                inverse_kinematics_B.ix = tmp->size[0];
                tmp->size[0] = ub->size[0];
                inv_emxEnsureCapacity_boolean_T(tmp, inverse_kinematics_B.ix);
                inverse_kinematics_B.nx = ub->size[0];
                for (inverse_kinematics_B.ix = 0; inverse_kinematics_B.ix <
                     inverse_kinematics_B.nx; inverse_kinematics_B.ix++) {
                  tmp->data[inverse_kinematics_B.ix] = rtIsInf(ub->
                    data[inverse_kinematics_B.ix]);
                }

                inverse_kinematics_B.ix = b->size[0];
                b->size[0] = ub->size[0];
                inv_emxEnsureCapacity_boolean_T(b, inverse_kinematics_B.ix);
                inverse_kinematics_B.nx = ub->size[0];
                for (inverse_kinematics_B.ix = 0; inverse_kinematics_B.ix <
                     inverse_kinematics_B.nx; inverse_kinematics_B.ix++) {
                  b->data[inverse_kinematics_B.ix] = rtIsNaN(ub->
                    data[inverse_kinematics_B.ix]);
                }

                inverse_kinematics_B.ix = tmp_0->size[0];
                tmp_0->size[0] = tmp->size[0];
                inv_emxEnsureCapacity_boolean_T(tmp_0, inverse_kinematics_B.ix);
                inverse_kinematics_B.nx = tmp->size[0];
                for (inverse_kinematics_B.ix = 0; inverse_kinematics_B.ix <
                     inverse_kinematics_B.nx; inverse_kinematics_B.ix++) {
                  tmp_0->data[inverse_kinematics_B.ix] = (tmp->
                    data[inverse_kinematics_B.ix] || b->
                    data[inverse_kinematics_B.ix]);
                }

                if (inverse_kinematics_any(tmp_0)) {
                  inverse_kinematics_B.ub[0] = lb->size[0];
                  inverse_kinematics_B.ub[1] = 1.0;
                  inverse_kinematics_randn(inverse_kinematics_B.ub, rn);
                  inverse_kinematics_B.nx = rn->size[0] - 1;
                  inverse_kinematics_B.ix = ub->size[0];
                  ub->size[0] = rn->size[0];
                  invers_emxEnsureCapacity_real_T(ub, inverse_kinematics_B.ix);
                  for (inverse_kinematics_B.ix = 0; inverse_kinematics_B.ix <=
                       inverse_kinematics_B.nx; inverse_kinematics_B.ix++) {
                    ub->data[inverse_kinematics_B.ix] = fabs(rn->
                      data[inverse_kinematics_B.ix]);
                  }

                  if (lb->size[0] == ub->size[0]) {
                    inverse_kinematics_B.ix = ub->size[0];
                    ub->size[0] = lb->size[0];
                    invers_emxEnsureCapacity_real_T(ub, inverse_kinematics_B.ix);
                    inverse_kinematics_B.nx = lb->size[0];
                    for (inverse_kinematics_B.ix = 0; inverse_kinematics_B.ix <
                         inverse_kinematics_B.nx; inverse_kinematics_B.ix++) {
                      ub->data[inverse_kinematics_B.ix] += lb->
                        data[inverse_kinematics_B.ix];
                    }
                  } else {
                    inverse_kinematics_plus(ub, lb);
                  }
                } else {
                  guard2 = true;
                }
              } else {
                guard2 = true;
              }
            }

            if (guard2) {
              inverse_kinematics_B.ix = tmp->size[0];
              tmp->size[0] = lb->size[0];
              inv_emxEnsureCapacity_boolean_T(tmp, inverse_kinematics_B.ix);
              inverse_kinematics_B.nx = lb->size[0];
              for (inverse_kinematics_B.ix = 0; inverse_kinematics_B.ix <
                   inverse_kinematics_B.nx; inverse_kinematics_B.ix++) {
                tmp->data[inverse_kinematics_B.ix] = rtIsInf(lb->
                  data[inverse_kinematics_B.ix]);
              }

              inverse_kinematics_B.ix = b->size[0];
              b->size[0] = lb->size[0];
              inv_emxEnsureCapacity_boolean_T(b, inverse_kinematics_B.ix);
              inverse_kinematics_B.nx = lb->size[0];
              for (inverse_kinematics_B.ix = 0; inverse_kinematics_B.ix <
                   inverse_kinematics_B.nx; inverse_kinematics_B.ix++) {
                b->data[inverse_kinematics_B.ix] = rtIsNaN(lb->
                  data[inverse_kinematics_B.ix]);
              }

              inverse_kinematics_B.ix = tmp_0->size[0];
              tmp_0->size[0] = tmp->size[0];
              inv_emxEnsureCapacity_boolean_T(tmp_0, inverse_kinematics_B.ix);
              inverse_kinematics_B.nx = tmp->size[0];
              for (inverse_kinematics_B.ix = 0; inverse_kinematics_B.ix <
                   inverse_kinematics_B.nx; inverse_kinematics_B.ix++) {
                tmp_0->data[inverse_kinematics_B.ix] = (tmp->
                  data[inverse_kinematics_B.ix] || b->
                  data[inverse_kinematics_B.ix]);
              }

              if (inverse_kinematics_any(tmp_0)) {
                inverse_kinematics_B.ix = b->size[0];
                b->size[0] = ub->size[0];
                inv_emxEnsureCapacity_boolean_T(b, inverse_kinematics_B.ix);
                inverse_kinematics_B.nx = ub->size[0];
                for (inverse_kinematics_B.ix = 0; inverse_kinematics_B.ix <
                     inverse_kinematics_B.nx; inverse_kinematics_B.ix++) {
                  b->data[inverse_kinematics_B.ix] = rtIsInf(ub->
                    data[inverse_kinematics_B.ix]);
                }

                inverse_kinematics_B.ix = tmp->size[0];
                tmp->size[0] = ub->size[0];
                inv_emxEnsureCapacity_boolean_T(tmp, inverse_kinematics_B.ix);
                inverse_kinematics_B.nx = ub->size[0];
                for (inverse_kinematics_B.ix = 0; inverse_kinematics_B.ix <
                     inverse_kinematics_B.nx; inverse_kinematics_B.ix++) {
                  tmp->data[inverse_kinematics_B.ix] = rtIsNaN(ub->
                    data[inverse_kinematics_B.ix]);
                }

                inverse_kinematics_B.nx = b->size[0];
                for (inverse_kinematics_B.ix = 0; inverse_kinematics_B.ix <
                     inverse_kinematics_B.nx; inverse_kinematics_B.ix++) {
                  b->data[inverse_kinematics_B.ix] = ((!b->
                    data[inverse_kinematics_B.ix]) && (!tmp->
                    data[inverse_kinematics_B.ix]));
                }

                inverse_kinematics_B.y_h = true;
                inverse_kinematics_B.ix = 0;
                exitg2 = false;
                while ((!exitg2) && (inverse_kinematics_B.ix + 1 <= b->size[0]))
                {
                  if (!b->data[inverse_kinematics_B.ix]) {
                    inverse_kinematics_B.y_h = false;
                    exitg2 = true;
                  } else {
                    inverse_kinematics_B.ix++;
                  }
                }

                if (inverse_kinematics_B.y_h) {
                  inverse_kinematics_B.ub[0] = ub->size[0];
                  inverse_kinematics_B.ub[1] = 1.0;
                  inverse_kinematics_randn(inverse_kinematics_B.ub, rn);
                  inverse_kinematics_B.nx = rn->size[0] - 1;
                  inverse_kinematics_B.ix = lb->size[0];
                  lb->size[0] = rn->size[0];
                  invers_emxEnsureCapacity_real_T(lb, inverse_kinematics_B.ix);
                  for (inverse_kinematics_B.ix = 0; inverse_kinematics_B.ix <=
                       inverse_kinematics_B.nx; inverse_kinematics_B.ix++) {
                    lb->data[inverse_kinematics_B.ix] = fabs(rn->
                      data[inverse_kinematics_B.ix]);
                  }

                  if (ub->size[0] == lb->size[0]) {
                    inverse_kinematics_B.nx = ub->size[0];
                    for (inverse_kinematics_B.ix = 0; inverse_kinematics_B.ix <
                         inverse_kinematics_B.nx; inverse_kinematics_B.ix++) {
                      ub->data[inverse_kinematics_B.ix] -= lb->
                        data[inverse_kinematics_B.ix];
                    }
                  } else {
                    inverse_kinematics_minus(ub, lb);
                  }
                } else {
                  guard1 = true;
                }
              } else {
                guard1 = true;
              }
            }

            if (guard1) {
              inverse_kinematics_B.ub[0] = ub->size[0];
              inverse_kinematics_B.ub[1] = 1.0;
              inverse_kinematics_randn(inverse_kinematics_B.ub, ub);
            }
          }

          if (inverse_kinematics_B.err > inverse_kinematics_B.iter) {
            inverse_kinematics_B.nx = 0;
            inverse_kinematics_B.ix = 0;
          } else {
            inverse_kinematics_B.nx = static_cast<int32_T>
              (inverse_kinematics_B.err) - 1;
            inverse_kinematics_B.ix = static_cast<int32_T>
              (inverse_kinematics_B.iter);
          }

          inverse_kinematics_B.unnamed_idx_1 = inverse_kinematics_B.ix -
            inverse_kinematics_B.nx;
          for (inverse_kinematics_B.ix = 0; inverse_kinematics_B.ix <
               inverse_kinematics_B.unnamed_idx_1; inverse_kinematics_B.ix++) {
            newseed->data[inverse_kinematics_B.nx + inverse_kinematics_B.ix] =
              ub->data[inverse_kinematics_B.ix];
          }
        }
      }

      obj->SeedInternal[0] = newseed->data[0];
      obj->SeedInternal[1] = newseed->data[1];
      obj->SeedInternal[2] = newseed->data[2];
      obj->SeedInternal[3] = newseed->data[3];
      ErrorDampedLevenbergMarquardt_s(obj, inverse_kinematics_B.c_xSol,
        &inverse_kinematics_B.exitFlag, &inverse_kinematics_B.err,
        &inverse_kinematics_B.iter);
      if (inverse_kinematics_B.err < *solutionInfo_Error) {
        xSol[0] = inverse_kinematics_B.c_xSol[0];
        xSol[1] = inverse_kinematics_B.c_xSol[1];
        xSol[2] = inverse_kinematics_B.c_xSol[2];
        xSol[3] = inverse_kinematics_B.c_xSol[3];
        *solutionInfo_Error = inverse_kinematics_B.err;
        inverse_kinematics_B.exitFlagPrev = inverse_kinematics_B.exitFlag;
      }

      (*solutionInfo_RRAttempts)++;
      *solutionInfo_Iterations += inverse_kinematics_B.iter;
    }
  }

  inverse_kinem_emxFree_boolean_T(&tmp_0);
  inverse_kinem_emxFree_boolean_T(&tmp);
  inverse_kinem_emxFree_boolean_T(&b);
  inverse_kinemati_emxFree_real_T(&rn);
  inverse_kinemati_emxFree_real_T(&lb);
  inverse_kinemati_emxFree_real_T(&ub);
  inverse_kinemati_emxFree_real_T(&newseed);
  *solutionInfo_ExitFlag = inverse_kinematics_B.exitFlagPrev;
  if (*solutionInfo_Error < inverse_kinematics_B.tol) {
    solutionInfo_Status_size[0] = 1;
    solutionInfo_Status_size[1] = 7;
    for (inverse_kinematics_B.ix = 0; inverse_kinematics_B.ix < 7;
         inverse_kinematics_B.ix++) {
      solutionInfo_Status_data[inverse_kinematics_B.ix] =
        tmp_2[inverse_kinematics_B.ix];
    }
  } else {
    solutionInfo_Status_size[0] = 1;
    solutionInfo_Status_size[1] = 14;
    for (inverse_kinematics_B.ix = 0; inverse_kinematics_B.ix < 14;
         inverse_kinematics_B.ix++) {
      solutionInfo_Status_data[inverse_kinematics_B.ix] =
        tmp_1[inverse_kinematics_B.ix];
    }
  }
}

static void inverse_kinemat_emxInit_int32_T(emxArray_int32_T_inverse_kine_T
  **pEmxArray, int32_T numDimensions)
{
  emxArray_int32_T_inverse_kine_T *emxArray;
  *pEmxArray = static_cast<emxArray_int32_T_inverse_kine_T *>(malloc(sizeof
    (emxArray_int32_T_inverse_kine_T)));
  emxArray = *pEmxArray;
  emxArray->data = static_cast<int32_T *>(NULL);
  emxArray->numDimensions = numDimensions;
  emxArray->size = static_cast<int32_T *>(malloc(sizeof(int32_T) * numDimensions));
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (int32_T i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

static void inver_emxEnsureCapacity_int32_T(emxArray_int32_T_inverse_kine_T
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

    newData = calloc(static_cast<uint32_T>(i), sizeof(int32_T));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(int32_T) * oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }

    emxArray->data = static_cast<int32_T *>(newData);
    emxArray->allocatedSize = i;
    emxArray->canFreeData = true;
  }
}

static void inverse_kinemat_emxFree_int32_T(emxArray_int32_T_inverse_kine_T
  **pEmxArray)
{
  if (*pEmxArray != static_cast<emxArray_int32_T_inverse_kine_T *>(NULL)) {
    if (((*pEmxArray)->data != static_cast<int32_T *>(NULL)) && (*pEmxArray)
        ->canFreeData) {
      free((*pEmxArray)->data);
    }

    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = static_cast<emxArray_int32_T_inverse_kine_T *>(NULL);
  }
}

static void inverse_kinema_emxInit_uint32_T(emxArray_uint32_T_inverse_kin_T
  **pEmxArray, int32_T numDimensions)
{
  emxArray_uint32_T_inverse_kin_T *emxArray;
  *pEmxArray = static_cast<emxArray_uint32_T_inverse_kin_T *>(malloc(sizeof
    (emxArray_uint32_T_inverse_kin_T)));
  emxArray = *pEmxArray;
  emxArray->data = static_cast<uint32_T *>(NULL);
  emxArray->numDimensions = numDimensions;
  emxArray->size = static_cast<int32_T *>(malloc(sizeof(int32_T) * numDimensions));
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (int32_T i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

static void inve_emxEnsureCapacity_uint32_T(emxArray_uint32_T_inverse_kin_T
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

    newData = calloc(static_cast<uint32_T>(i), sizeof(uint32_T));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(uint32_T) * oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }

    emxArray->data = static_cast<uint32_T *>(newData);
    emxArray->allocatedSize = i;
    emxArray->canFreeData = true;
  }
}

static void inverse_kinema_emxFree_uint32_T(emxArray_uint32_T_inverse_kin_T
  **pEmxArray)
{
  if (*pEmxArray != static_cast<emxArray_uint32_T_inverse_kin_T *>(NULL)) {
    if (((*pEmxArray)->data != static_cast<uint32_T *>(NULL)) && (*pEmxArray)
        ->canFreeData) {
      free((*pEmxArray)->data);
    }

    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = static_cast<emxArray_uint32_T_inverse_kin_T *>(NULL);
  }
}

static void inver_inverseKinematics_solve_g(b_inverseKinematics_inverse_k_T *obj,
  real_T initialGuess[4], real_T *solutionInfo_Iterations, real_T
  *solutionInfo_NumRandomRestarts, real_T *solutionInfo_PoseErrorNorm, real_T
  *solutionInfo_ExitFlag, char_T solutionInfo_Status_data[], int32_T
  solutionInfo_Status_size[2])
{
  emxArray_char_T_inverse_kinem_T *endEffectorName;
  emxArray_int32_T_inverse_kine_T *h;
  emxArray_real_T_inverse_kinem_T *bodyIndices;
  emxArray_real_T_inverse_kinem_T *e;
  emxArray_real_T_inverse_kinem_T *limits;
  emxArray_real_T_inverse_kinem_T *positionIndices;
  emxArray_uint32_T_inverse_kin_T *y;
  t_robotics_manip_internal_Rig_T *body;
  v_robotics_manip_internal_Rig_T *obj_0;
  boolean_T exitg1;
  boolean_T guard1 = false;
  inverse_kinemati_emxInit_real_T(&limits, 2);
  obj_0 = obj->RigidBodyTreeInternal;
  RigidBodyTree_get_JointPosition(obj_0, limits);
  if (limits->size[0] == 4) {
    inverse_kinematics_B.ubOK[0] = (initialGuess[0] <= limits->data[limits->
      size[0]] + 4.4408920985006262E-16);
    inverse_kinematics_B.ubOK[1] = (initialGuess[1] <= limits->data[1 +
      limits->size[0]] + 4.4408920985006262E-16);
    inverse_kinematics_B.ubOK[2] = (initialGuess[2] <= limits->data[2 +
      limits->size[0]] + 4.4408920985006262E-16);
    inverse_kinematics_B.ubOK[3] = (initialGuess[3] <= limits->data[3 +
      limits->size[0]] + 4.4408920985006262E-16);
  } else {
    inverse_ki_binary_expand_op_gaq(inverse_kinematics_B.ubOK, initialGuess,
      limits);
  }

  if (limits->size[0] == 4) {
    inverse_kinematics_B.lbOK[0] = (initialGuess[0] >= limits->data[0] -
      4.4408920985006262E-16);
    inverse_kinematics_B.lbOK[1] = (initialGuess[1] >= limits->data[1] -
      4.4408920985006262E-16);
    inverse_kinematics_B.lbOK[2] = (initialGuess[2] >= limits->data[2] -
      4.4408920985006262E-16);
    inverse_kinematics_B.lbOK[3] = (initialGuess[3] >= limits->data[3] -
      4.4408920985006262E-16);
  } else {
    inverse_kin_binary_expand_op_ga(inverse_kinematics_B.lbOK, initialGuess,
      limits);
  }

  inverse_kinematics_B.y_f = true;
  inverse_kinematics_B.b_k = 0;
  exitg1 = false;
  while ((!exitg1) && (inverse_kinematics_B.b_k < 4)) {
    if (!inverse_kinematics_B.ubOK[inverse_kinematics_B.b_k]) {
      inverse_kinematics_B.y_f = false;
      exitg1 = true;
    } else {
      inverse_kinematics_B.b_k++;
    }
  }

  guard1 = false;
  if (inverse_kinematics_B.y_f) {
    inverse_kinematics_B.y_f = true;
    inverse_kinematics_B.b_k = 0;
    exitg1 = false;
    while ((!exitg1) && (inverse_kinematics_B.b_k < 4)) {
      if (!inverse_kinematics_B.lbOK[inverse_kinematics_B.b_k]) {
        inverse_kinematics_B.y_f = false;
        exitg1 = true;
      } else {
        inverse_kinematics_B.b_k++;
      }
    }

    if (inverse_kinematics_B.y_f) {
    } else {
      guard1 = true;
    }
  } else {
    guard1 = true;
  }

  if (guard1) {
    inverse_kinematics_B.ubOK_i[0] = !inverse_kinematics_B.ubOK[0];
    inverse_kinematics_B.ubOK_i[1] = !inverse_kinematics_B.ubOK[1];
    inverse_kinematics_B.ubOK_i[2] = !inverse_kinematics_B.ubOK[2];
    inverse_kinematics_B.ubOK_i[3] = !inverse_kinematics_B.ubOK[3];
    inverse_kinematics_eml_find(inverse_kinematics_B.ubOK_i,
      inverse_kinematics_B.tmp_data, &inverse_kinematics_B.tmp_size);
    inverse_kinematics_B.indicesUpperBoundViolation_size =
      inverse_kinematics_B.tmp_size;
    inverse_kinematics_B.loop_ub_f = inverse_kinematics_B.tmp_size;
    if (inverse_kinematics_B.loop_ub_f - 1 >= 0) {
      memcpy(&inverse_kinematics_B.indicesUpperBoundViolation_data[0],
             &inverse_kinematics_B.tmp_data[0], inverse_kinematics_B.loop_ub_f *
             sizeof(int32_T));
    }

    for (inverse_kinematics_B.b_k = 0; inverse_kinematics_B.b_k <
         inverse_kinematics_B.indicesUpperBoundViolation_size;
         inverse_kinematics_B.b_k++) {
      inverse_kinematics_B.indicesUpperBoundViolation =
        inverse_kinematics_B.indicesUpperBoundViolation_data[inverse_kinematics_B.b_k];
      initialGuess[inverse_kinematics_B.indicesUpperBoundViolation - 1] =
        limits->data[(inverse_kinematics_B.indicesUpperBoundViolation +
                      limits->size[0]) - 1];
    }

    inverse_kinematics_B.ubOK[0] = !inverse_kinematics_B.lbOK[0];
    inverse_kinematics_B.ubOK[1] = !inverse_kinematics_B.lbOK[1];
    inverse_kinematics_B.ubOK[2] = !inverse_kinematics_B.lbOK[2];
    inverse_kinematics_B.ubOK[3] = !inverse_kinematics_B.lbOK[3];
    inverse_kinematics_eml_find(inverse_kinematics_B.ubOK,
      inverse_kinematics_B.tmp_data, &inverse_kinematics_B.tmp_size);
    inverse_kinematics_B.indicesUpperBoundViolation_size =
      inverse_kinematics_B.tmp_size;
    inverse_kinematics_B.loop_ub_f = inverse_kinematics_B.tmp_size;
    if (inverse_kinematics_B.loop_ub_f - 1 >= 0) {
      memcpy(&inverse_kinematics_B.indicesUpperBoundViolation_data[0],
             &inverse_kinematics_B.tmp_data[0], inverse_kinematics_B.loop_ub_f *
             sizeof(int32_T));
    }

    for (inverse_kinematics_B.b_k = 0; inverse_kinematics_B.b_k <
         inverse_kinematics_B.indicesUpperBoundViolation_size;
         inverse_kinematics_B.b_k++) {
      inverse_kinematics_B.indicesUpperBoundViolation =
        inverse_kinematics_B.indicesUpperBoundViolation_data[inverse_kinematics_B.b_k];
      initialGuess[inverse_kinematics_B.indicesUpperBoundViolation - 1] =
        limits->data[inverse_kinematics_B.indicesUpperBoundViolation - 1];
    }
  }

  inverse_kinemati_emxInit_char_T(&endEffectorName, 2);
  invers_NLPSolverInterface_solve(obj->Solver, initialGuess,
    inverse_kinematics_B.qvSolRaw, solutionInfo_Iterations,
    solutionInfo_NumRandomRestarts, solutionInfo_PoseErrorNorm,
    solutionInfo_ExitFlag, solutionInfo_Status_data, solutionInfo_Status_size);
  obj_0 = obj->RigidBodyTreeInternal;
  inverse_kinematics_B.nm1d2 = endEffectorName->size[0] * endEffectorName->size
    [1];
  endEffectorName->size[0] = 1;
  endEffectorName->size[1] = obj->Solver->ExtraArgs->BodyName->size[1];
  invers_emxEnsureCapacity_char_T(endEffectorName, inverse_kinematics_B.nm1d2);
  inverse_kinematics_B.loop_ub_f = obj->Solver->ExtraArgs->BodyName->size[1] - 1;
  for (inverse_kinematics_B.b_k = 0; inverse_kinematics_B.b_k <=
       inverse_kinematics_B.loop_ub_f; inverse_kinematics_B.b_k++) {
    inverse_kinematics_B.nm1d2 = inverse_kinematics_B.b_k;
    endEffectorName->data[inverse_kinematics_B.nm1d2] = obj->Solver->
      ExtraArgs->BodyName->data[inverse_kinematics_B.nm1d2];
  }

  inverse_kinemati_emxInit_real_T(&bodyIndices, 1);
  inverse_kinematics_B.nm1d2 = bodyIndices->size[0];
  bodyIndices->size[0] = static_cast<int32_T>(obj_0->NumBodies);
  invers_emxEnsureCapacity_real_T(bodyIndices, inverse_kinematics_B.nm1d2);
  inverse_kinematics_B.loop_ub_f = static_cast<int32_T>(obj_0->NumBodies);
  if (inverse_kinematics_B.loop_ub_f - 1 >= 0) {
    memset(&bodyIndices->data[0], 0, inverse_kinematics_B.loop_ub_f * sizeof
           (real_T));
  }

  inverse_kinematics_B.bid = RigidBodyTree_findBodyIndexByNa(obj_0,
    endEffectorName);
  inverse_kinemati_emxFree_char_T(&endEffectorName);
  if (inverse_kinematics_B.bid == 0.0) {
    inverse_kinematics_B.nm1d2 = bodyIndices->size[0];
    bodyIndices->size[0] = 1;
    invers_emxEnsureCapacity_real_T(bodyIndices, inverse_kinematics_B.nm1d2);
    bodyIndices->data[0] = 0.0;
  } else {
    body = obj_0->Bodies[static_cast<int32_T>(inverse_kinematics_B.bid) - 1];
    inverse_kinematics_B.bid = 1.0;
    while (body->ParentIndex != 0.0) {
      bodyIndices->data[static_cast<int32_T>(inverse_kinematics_B.bid) - 1] =
        body->Index;
      body = obj_0->Bodies[static_cast<int32_T>(body->ParentIndex) - 1];
      inverse_kinematics_B.bid++;
    }

    if (inverse_kinematics_B.bid - 1.0 < 1.0) {
      inverse_kinematics_B.indicesUpperBoundViolation_size = -1;
    } else {
      inverse_kinematics_B.indicesUpperBoundViolation_size = static_cast<int32_T>
        (inverse_kinematics_B.bid - 1.0) - 1;
    }

    inverse_kinematics_B.nm1d2 = bodyIndices->size[0];
    bodyIndices->size[0] = inverse_kinematics_B.indicesUpperBoundViolation_size
      + 3;
    invers_emxEnsureCapacity_real_T(bodyIndices, inverse_kinematics_B.nm1d2);
    bodyIndices->data[inverse_kinematics_B.indicesUpperBoundViolation_size + 1] =
      body->Index;
    bodyIndices->data[inverse_kinematics_B.indicesUpperBoundViolation_size + 2] =
      0.0;
  }

  obj_0 = obj->RigidBodyTreeInternal;
  inverse_kinematics_B.b_k = bodyIndices->size[0] - 1;
  inverse_kinematics_B.indicesUpperBoundViolation_size = 0;
  for (inverse_kinematics_B.indicesUpperBoundViolation = 0;
       inverse_kinematics_B.indicesUpperBoundViolation <=
       inverse_kinematics_B.b_k; inverse_kinematics_B.indicesUpperBoundViolation
       ++) {
    if (bodyIndices->data[inverse_kinematics_B.indicesUpperBoundViolation] !=
        0.0) {
      inverse_kinematics_B.indicesUpperBoundViolation_size++;
    }
  }

  inverse_kinemat_emxInit_int32_T(&h, 1);
  inverse_kinematics_B.nm1d2 = h->size[0];
  h->size[0] = inverse_kinematics_B.indicesUpperBoundViolation_size;
  inver_emxEnsureCapacity_int32_T(h, inverse_kinematics_B.nm1d2);
  inverse_kinematics_B.indicesUpperBoundViolation_size = 0;
  for (inverse_kinematics_B.indicesUpperBoundViolation = 0;
       inverse_kinematics_B.indicesUpperBoundViolation <=
       inverse_kinematics_B.b_k; inverse_kinematics_B.indicesUpperBoundViolation
       ++) {
    if (bodyIndices->data[inverse_kinematics_B.indicesUpperBoundViolation] !=
        0.0) {
      h->data[inverse_kinematics_B.indicesUpperBoundViolation_size] =
        inverse_kinematics_B.indicesUpperBoundViolation + 1;
      inverse_kinematics_B.indicesUpperBoundViolation_size++;
    }
  }

  inverse_kinematics_B.nm1d2 = limits->size[0] * limits->size[1];
  limits->size[0] = h->size[0];
  limits->size[1] = 2;
  invers_emxEnsureCapacity_real_T(limits, inverse_kinematics_B.nm1d2);
  inverse_kinematics_B.loop_ub_f = h->size[0];
  for (inverse_kinematics_B.b_k = 0; inverse_kinematics_B.b_k < 2;
       inverse_kinematics_B.b_k++) {
    for (inverse_kinematics_B.indicesUpperBoundViolation = 0;
         inverse_kinematics_B.indicesUpperBoundViolation <
         inverse_kinematics_B.loop_ub_f;
         inverse_kinematics_B.indicesUpperBoundViolation++) {
      limits->data[inverse_kinematics_B.indicesUpperBoundViolation +
        limits->size[0] * inverse_kinematics_B.b_k] = obj_0->PositionDoFMap[(
        static_cast<int32_T>(bodyIndices->data[h->
        data[inverse_kinematics_B.indicesUpperBoundViolation] - 1]) + 7 *
        inverse_kinematics_B.b_k) - 1];
    }
  }

  inverse_kinemat_emxFree_int32_T(&h);
  inverse_kinemati_emxFree_real_T(&bodyIndices);
  inverse_kinemati_emxInit_real_T(&positionIndices, 2);
  inverse_kinematics_B.nm1d2 = positionIndices->size[0] * positionIndices->size
    [1];
  positionIndices->size[0] = 1;
  positionIndices->size[1] = static_cast<int32_T>(obj_0->PositionNumber);
  invers_emxEnsureCapacity_real_T(positionIndices, inverse_kinematics_B.nm1d2);
  inverse_kinematics_B.loop_ub_f = static_cast<int32_T>(obj_0->PositionNumber) -
    1;
  if (inverse_kinematics_B.loop_ub_f >= 0) {
    memset(&positionIndices->data[0], 0, (inverse_kinematics_B.loop_ub_f + 1) *
           sizeof(real_T));
  }

  inverse_kinematics_B.bid = 0.0;
  inverse_kinematics_B.indicesUpperBoundViolation_size = limits->size[0] - 1;
  inverse_kinemati_emxInit_real_T(&e, 2);
  inverse_kinema_emxInit_uint32_T(&y, 2);
  for (inverse_kinematics_B.indicesUpperBoundViolation = 0;
       inverse_kinematics_B.indicesUpperBoundViolation <=
       inverse_kinematics_B.indicesUpperBoundViolation_size;
       inverse_kinematics_B.indicesUpperBoundViolation++) {
    inverse_kinematics_B.numPositions_tmp = limits->
      data[inverse_kinematics_B.indicesUpperBoundViolation + limits->size[0]] -
      limits->data[inverse_kinematics_B.indicesUpperBoundViolation];
    if (inverse_kinematics_B.numPositions_tmp + 1.0 > 0.0) {
      if (inverse_kinematics_B.numPositions_tmp + 1.0 < 1.0) {
        y->size[0] = 1;
        y->size[1] = 0;
      } else {
        inverse_kinematics_B.nm1d2 = y->size[0] * y->size[1];
        y->size[0] = 1;
        y->size[1] = static_cast<int32_T>((inverse_kinematics_B.numPositions_tmp
          + 1.0) - 1.0) + 1;
        inve_emxEnsureCapacity_uint32_T(y, inverse_kinematics_B.nm1d2);
        inverse_kinematics_B.loop_ub_f = static_cast<int32_T>
          ((inverse_kinematics_B.numPositions_tmp + 1.0) - 1.0);
        for (inverse_kinematics_B.b_k = 0; inverse_kinematics_B.b_k <=
             inverse_kinematics_B.loop_ub_f; inverse_kinematics_B.b_k++) {
          y->data[inverse_kinematics_B.b_k] = inverse_kinematics_B.b_k + 1U;
        }
      }

      if (rtIsNaN(limits->data[inverse_kinematics_B.indicesUpperBoundViolation])
          || rtIsNaN(limits->
                     data[inverse_kinematics_B.indicesUpperBoundViolation +
                     limits->size[0]])) {
        inverse_kinematics_B.nm1d2 = e->size[0] * e->size[1];
        e->size[0] = 1;
        e->size[1] = 1;
        invers_emxEnsureCapacity_real_T(e, inverse_kinematics_B.nm1d2);
        e->data[0] = (rtNaN);
      } else if (limits->data[inverse_kinematics_B.indicesUpperBoundViolation +
                 limits->size[0]] < limits->
                 data[inverse_kinematics_B.indicesUpperBoundViolation]) {
        e->size[0] = 1;
        e->size[1] = 0;
      } else if ((rtIsInf(limits->
                          data[inverse_kinematics_B.indicesUpperBoundViolation])
                  || rtIsInf(limits->
                             data[inverse_kinematics_B.indicesUpperBoundViolation
        + limits->size[0]])) && (limits->
                  data[inverse_kinematics_B.indicesUpperBoundViolation +
                  limits->size[0]] == limits->
                  data[inverse_kinematics_B.indicesUpperBoundViolation])) {
        inverse_kinematics_B.nm1d2 = e->size[0] * e->size[1];
        e->size[0] = 1;
        e->size[1] = 1;
        invers_emxEnsureCapacity_real_T(e, inverse_kinematics_B.nm1d2);
        e->data[0] = (rtNaN);
      } else if (floor(limits->
                       data[inverse_kinematics_B.indicesUpperBoundViolation]) ==
                 limits->data[inverse_kinematics_B.indicesUpperBoundViolation])
      {
        inverse_kinematics_B.nm1d2 = e->size[0] * e->size[1];
        e->size[0] = 1;
        e->size[1] = static_cast<int32_T>(inverse_kinematics_B.numPositions_tmp)
          + 1;
        invers_emxEnsureCapacity_real_T(e, inverse_kinematics_B.nm1d2);
        inverse_kinematics_B.loop_ub_f = static_cast<int32_T>(limits->
          data[inverse_kinematics_B.indicesUpperBoundViolation + limits->size[0]]
          - limits->data[inverse_kinematics_B.indicesUpperBoundViolation]);
        for (inverse_kinematics_B.b_k = 0; inverse_kinematics_B.b_k <=
             inverse_kinematics_B.loop_ub_f; inverse_kinematics_B.b_k++) {
          e->data[inverse_kinematics_B.b_k] = limits->
            data[inverse_kinematics_B.indicesUpperBoundViolation] + static_cast<
            real_T>(inverse_kinematics_B.b_k);
        }
      } else {
        inverse_kinematics_B.ndbl = floor(inverse_kinematics_B.numPositions_tmp
          + 0.5);
        inverse_kinematics_B.apnd = limits->
          data[inverse_kinematics_B.indicesUpperBoundViolation] +
          inverse_kinematics_B.ndbl;
        inverse_kinematics_B.cdiff = inverse_kinematics_B.apnd - limits->
          data[inverse_kinematics_B.indicesUpperBoundViolation + limits->size[0]];
        inverse_kinematics_B.u0 = fabs(limits->
          data[inverse_kinematics_B.indicesUpperBoundViolation]);
        inverse_kinematics_B.u1 = fabs(limits->
          data[inverse_kinematics_B.indicesUpperBoundViolation + limits->size[0]]);
        if ((inverse_kinematics_B.u0 >= inverse_kinematics_B.u1) || rtIsNaN
            (inverse_kinematics_B.u1)) {
          inverse_kinematics_B.u1 = inverse_kinematics_B.u0;
        }

        if (fabs(inverse_kinematics_B.cdiff) < 4.4408920985006262E-16 *
            inverse_kinematics_B.u1) {
          inverse_kinematics_B.ndbl++;
          inverse_kinematics_B.apnd = limits->
            data[inverse_kinematics_B.indicesUpperBoundViolation + limits->size
            [0]];
        } else if (inverse_kinematics_B.cdiff > 0.0) {
          inverse_kinematics_B.apnd = (inverse_kinematics_B.ndbl - 1.0) +
            limits->data[inverse_kinematics_B.indicesUpperBoundViolation];
        } else {
          inverse_kinematics_B.ndbl++;
        }

        if (inverse_kinematics_B.ndbl >= 0.0) {
          inverse_kinematics_B.loop_ub_f = static_cast<int32_T>
            (inverse_kinematics_B.ndbl);
        } else {
          inverse_kinematics_B.loop_ub_f = 0;
        }

        inverse_kinematics_B.nm1d2 = e->size[0] * e->size[1];
        e->size[0] = 1;
        e->size[1] = inverse_kinematics_B.loop_ub_f;
        invers_emxEnsureCapacity_real_T(e, inverse_kinematics_B.nm1d2);
        if (inverse_kinematics_B.loop_ub_f > 0) {
          e->data[0] = limits->
            data[inverse_kinematics_B.indicesUpperBoundViolation];
          if (inverse_kinematics_B.loop_ub_f > 1) {
            e->data[inverse_kinematics_B.loop_ub_f - 1] =
              inverse_kinematics_B.apnd;
            inverse_kinematics_B.nm1d2 = (((inverse_kinematics_B.loop_ub_f - 1 <
              0) + inverse_kinematics_B.loop_ub_f) - 1) >> 1;
            inverse_kinematics_B.c_i = inverse_kinematics_B.nm1d2 - 2;
            for (inverse_kinematics_B.b_k = 0; inverse_kinematics_B.b_k <=
                 inverse_kinematics_B.c_i; inverse_kinematics_B.b_k++) {
              e->data[inverse_kinematics_B.b_k + 1] = static_cast<real_T>
                (inverse_kinematics_B.b_k + 1) + limits->
                data[inverse_kinematics_B.indicesUpperBoundViolation];
              e->data[(inverse_kinematics_B.loop_ub_f - inverse_kinematics_B.b_k)
                - 2] = inverse_kinematics_B.apnd - static_cast<real_T>
                (inverse_kinematics_B.b_k + 1);
            }

            if (inverse_kinematics_B.nm1d2 << 1 ==
                inverse_kinematics_B.loop_ub_f - 1) {
              e->data[inverse_kinematics_B.nm1d2] = (limits->
                data[inverse_kinematics_B.indicesUpperBoundViolation] +
                inverse_kinematics_B.apnd) / 2.0;
            } else {
              e->data[inverse_kinematics_B.nm1d2] = limits->
                data[inverse_kinematics_B.indicesUpperBoundViolation] +
                static_cast<real_T>(inverse_kinematics_B.nm1d2);
              e->data[inverse_kinematics_B.nm1d2 + 1] =
                inverse_kinematics_B.apnd - static_cast<real_T>
                (inverse_kinematics_B.nm1d2);
            }
          }
        }
      }

      inverse_kinematics_B.loop_ub_f = e->size[1] - 1;
      for (inverse_kinematics_B.b_k = 0; inverse_kinematics_B.b_k <=
           inverse_kinematics_B.loop_ub_f; inverse_kinematics_B.b_k++) {
        inverse_kinematics_B.nm1d2 = inverse_kinematics_B.b_k;
        positionIndices->data[static_cast<int32_T>(inverse_kinematics_B.bid +
          static_cast<real_T>(y->data[inverse_kinematics_B.nm1d2])) - 1] =
          e->data[inverse_kinematics_B.nm1d2];
      }

      inverse_kinematics_B.bid += inverse_kinematics_B.numPositions_tmp + 1.0;
    }
  }

  inverse_kinema_emxFree_uint32_T(&y);
  inverse_kinemati_emxFree_real_T(&e);
  inverse_kinemati_emxFree_real_T(&limits);
  if (inverse_kinematics_B.bid < 1.0) {
    inverse_kinematics_B.indicesUpperBoundViolation = -1;
  } else {
    inverse_kinematics_B.indicesUpperBoundViolation = static_cast<int32_T>
      (inverse_kinematics_B.bid) - 1;
  }

  for (inverse_kinematics_B.b_k = 0; inverse_kinematics_B.b_k <=
       inverse_kinematics_B.indicesUpperBoundViolation; inverse_kinematics_B.b_k
       ++) {
    inverse_kinematics_B.bid = positionIndices->data[inverse_kinematics_B.b_k];
    initialGuess[static_cast<int32_T>(inverse_kinematics_B.bid) - 1] =
      inverse_kinematics_B.qvSolRaw[static_cast<int32_T>
      (inverse_kinematics_B.bid) - 1];
  }

  inverse_kinemati_emxFree_real_T(&positionIndices);
}

static void inve_inverseKinematics_stepImpl(b_inverseKinematics_inverse_k_T *obj,
  const real_T tform[16], const real_T weights[6], const real_T initialGuess[4],
  real_T QSol[4])
{
  f_robotics_manip_internal_IKE_T *args;
  static const char_T tmp[5] = { 'B', 'o', 'd', 'y', '7' };

  memset(&inverse_kinematics_B.weightMatrix[0], 0, 36U * sizeof(real_T));
  for (inverse_kinematics_B.b_j_o = 0; inverse_kinematics_B.b_j_o < 6;
       inverse_kinematics_B.b_j_o++) {
    inverse_kinematics_B.weightMatrix[inverse_kinematics_B.b_j_o + 6 *
      inverse_kinematics_B.b_j_o] = weights[inverse_kinematics_B.b_j_o];
  }

  args = obj->Solver->ExtraArgs;
  for (inverse_kinematics_B.b_j_o = 0; inverse_kinematics_B.b_j_o < 36;
       inverse_kinematics_B.b_j_o++) {
    args->WeightMatrix[inverse_kinematics_B.b_j_o] =
      inverse_kinematics_B.weightMatrix[inverse_kinematics_B.b_j_o];
  }

  inverse_kinematics_B.b_j_o = args->BodyName->size[0] * args->BodyName->size[1];
  args->BodyName->size[0] = 1;
  args->BodyName->size[1] = 5;
  invers_emxEnsureCapacity_char_T(args->BodyName, inverse_kinematics_B.b_j_o);
  for (inverse_kinematics_B.b_j_o = 0; inverse_kinematics_B.b_j_o < 5;
       inverse_kinematics_B.b_j_o++) {
    args->BodyName->data[inverse_kinematics_B.b_j_o] =
      tmp[inverse_kinematics_B.b_j_o];
  }

  for (inverse_kinematics_B.b_j_o = 0; inverse_kinematics_B.b_j_o < 16;
       inverse_kinematics_B.b_j_o++) {
    args->Tform[inverse_kinematics_B.b_j_o] = tform[inverse_kinematics_B.b_j_o];
  }

  QSol[0] = initialGuess[0];
  QSol[1] = initialGuess[1];
  QSol[2] = initialGuess[2];
  QSol[3] = initialGuess[3];
  inver_inverseKinematics_solve_g(obj, QSol, &inverse_kinematics_B.expl_temp,
    &inverse_kinematics_B.expl_temp_o, &inverse_kinematics_B.expl_temp_n,
    &inverse_kinematics_B.expl_temp_m, inverse_kinematics_B.expl_temp_data,
    inverse_kinematics_B.expl_temp_size);
}

static void emxFreeStruct_t_robotics_manip_(t_robotics_manip_internal_Rig_T
  *pStruct)
{
  inverse_kinemati_emxFree_char_T(&pStruct->NameInternal);
}

static void emxFreeStruct_l_robotics_manip_(l_robotics_manip_internal_Col_T
  *pStruct)
{
  inverse__emxFree_unnamed_struct(&pStruct->CollisionGeometries);
}

static void emxFreeMatrix_l_robotics_manip_(l_robotics_manip_internal_Col_T
  pMatrix[15])
{
  for (int32_T i = 0; i < 15; i++) {
    emxFreeStruct_l_robotics_manip_(&pMatrix[i]);
  }
}

static void emxFreeStruct_c_rigidBodyJoint(c_rigidBodyJoint_inverse_kine_T
  *pStruct)
{
  inverse_kinemati_emxFree_char_T(&pStruct->Type);
  inverse_kinemati_emxFree_real_T(&pStruct->MotionSubspace);
  inverse_kinemati_emxFree_char_T(&pStruct->NameInternal);
  inverse_kinemati_emxFree_real_T(&pStruct->PositionLimitsInternal);
  inverse_kinemati_emxFree_real_T(&pStruct->HomePositionInternal);
}

static void emxFreeMatrix_c_rigidBodyJoint(c_rigidBodyJoint_inverse_kine_T
  pMatrix[15])
{
  for (int32_T i = 0; i < 15; i++) {
    emxFreeStruct_c_rigidBodyJoint(&pMatrix[i]);
  }
}

static void emxFreeMatrix_t_robotics_manip_(t_robotics_manip_internal_Rig_T
  pMatrix[14])
{
  for (int32_T i = 0; i < 14; i++) {
    emxFreeStruct_t_robotics_manip_(&pMatrix[i]);
  }
}

static void emxFreeStruct_u_robotics_manip_(u_robotics_manip_internal_Rig_T
  *pStruct)
{
  emxFreeStruct_t_robotics_manip_(&pStruct->Base);
  emxFreeMatrix_l_robotics_manip_(pStruct->_pobj0);
  emxFreeMatrix_c_rigidBodyJoint(pStruct->_pobj1);
  emxFreeMatrix_t_robotics_manip_(pStruct->_pobj2);
}

static void emxFreeStruct_f_robotics_manip_(f_robotics_manip_internal_IKE_T
  *pStruct)
{
  inverse_kinemati_emxFree_real_T(&pStruct->Limits);
  inverse_kinemati_emxFree_char_T(&pStruct->BodyName);
  inverse_kinemati_emxFree_real_T(&pStruct->ErrTemp);
  inverse_kinemati_emxFree_real_T(&pStruct->GradTemp);
}

static void emxFreeMatrix_c_rigidBodyJoint1(c_rigidBodyJoint_inverse_kine_T
  pMatrix[14])
{
  for (int32_T i = 0; i < 14; i++) {
    emxFreeStruct_c_rigidBodyJoint(&pMatrix[i]);
  }
}

static void emxFreeMatrix_t_robotics_mani_g(t_robotics_manip_internal_Rig_T
  pMatrix[7])
{
  for (int32_T i = 0; i < 7; i++) {
    emxFreeStruct_t_robotics_manip_(&pMatrix[i]);
  }
}

static void emxFreeMatrix_l_robotics_mani_g(l_robotics_manip_internal_Col_T
  pMatrix[8])
{
  for (int32_T i = 0; i < 8; i++) {
    emxFreeStruct_l_robotics_manip_(&pMatrix[i]);
  }
}

static void emxFreeMatrix_c_rigidBodyJoint2(c_rigidBodyJoint_inverse_kine_T
  pMatrix[8])
{
  for (int32_T i = 0; i < 8; i++) {
    emxFreeStruct_c_rigidBodyJoint(&pMatrix[i]);
  }
}

static void emxFreeStruct_v_robotics_manip_(v_robotics_manip_internal_Rig_T
  *pStruct)
{
  emxFreeStruct_t_robotics_manip_(&pStruct->Base);
  emxFreeMatrix_t_robotics_mani_g(pStruct->_pobj0);
  emxFreeMatrix_l_robotics_mani_g(pStruct->_pobj1);
  emxFreeMatrix_c_rigidBodyJoint2(pStruct->_pobj2);
}

static void emxFreeStruct_b_inverseKinemati(b_inverseKinematics_inverse_k_T
  *pStruct)
{
  inverse_kinemati_emxFree_real_T(&pStruct->Limits);
  emxFreeStruct_f_robotics_manip_(&pStruct->_pobj0);
  emxFreeMatrix_c_rigidBodyJoint1(pStruct->_pobj1);
  emxFreeMatrix_t_robotics_mani_g(pStruct->_pobj2);
  emxFreeMatrix_l_robotics_manip_(pStruct->_pobj3);
  emxFreeStruct_v_robotics_manip_(&pStruct->_pobj4);
}

static void emxFreeStruct_robotics_slmanip_(robotics_slmanip_internal_blo_T
  *pStruct)
{
  emxFreeStruct_u_robotics_manip_(&pStruct->TreeInternal);
  emxFreeStruct_b_inverseKinemati(&pStruct->IKInternal);
}

/* Model step function */
void inverse_kinematics_step(void)
{
  b_inverseKinematics_inverse_k_T *obj;
  emxArray_int8_T_inverse_kinem_T *b_gradTmp;
  emxArray_real_T_inverse_kinem_T *tmp;
  static const uint8_T b[6] = { 112U, 101U, 108U, 118U, 105U, 115U };

  static const uint8_T c[8] = { 115U, 104U, 111U, 117U, 108U, 100U, 101U, 114U };

  static const uint8_T d[5] = { 101U, 108U, 98U, 111U, 119U };

  static const uint8_T e[5] = { 119U, 114U, 105U, 115U, 116U };

  /* Outputs for Atomic SubSystem: '<S1>/Subscribe' */
  /* MATLABSystem: '<S4>/SourceBlock' */
  inverse_kinematics_B.b_varargout_1 =
    Sub_inverse_kinematics_482.getLatestMessage
    (&inverse_kinematics_B.b_varargout_2);

  /* Outputs for Enabled SubSystem: '<Root>/Inverse Kinematics' incorporates:
   *  EnablePort: '<S2>/Enable'
   */
  /* Outputs for Enabled SubSystem: '<S4>/Enabled Subsystem' incorporates:
   *  EnablePort: '<S5>/Enable'
   */
  if (inverse_kinematics_B.b_varargout_1) {
    /* MATLABSystem: '<S2>/Coordinate Transformation Conversion1' */
    for (inverse_kinematics_B.i_o = 0; inverse_kinematics_B.i_o < 16;
         inverse_kinematics_B.i_o++) {
      inverse_kinematics_B.b_I[inverse_kinematics_B.i_o] = 0;
    }

    inverse_kinematics_B.b_I[0] = 1;
    inverse_kinematics_B.b_I[5] = 1;
    inverse_kinematics_B.b_I[10] = 1;
    inverse_kinematics_B.b_I[15] = 1;
    for (inverse_kinematics_B.i_o = 0; inverse_kinematics_B.i_o < 4;
         inverse_kinematics_B.i_o++) {
      inverse_kinematics_B.iacol_tmp = (inverse_kinematics_B.i_o << 2) - 1;
      inverse_kinematics_B.out[inverse_kinematics_B.iacol_tmp + 1] =
        inverse_kinematics_B.b_I[inverse_kinematics_B.iacol_tmp + 1];
      inverse_kinematics_B.out[inverse_kinematics_B.iacol_tmp + 2] =
        inverse_kinematics_B.b_I[inverse_kinematics_B.iacol_tmp + 2];
      inverse_kinematics_B.out[inverse_kinematics_B.iacol_tmp + 3] =
        inverse_kinematics_B.b_I[inverse_kinematics_B.iacol_tmp + 3];
      inverse_kinematics_B.out[inverse_kinematics_B.iacol_tmp + 4] =
        inverse_kinematics_B.b_I[inverse_kinematics_B.iacol_tmp + 4];
    }

    inverse_kinematics_B.out[12] = inverse_kinematics_B.b_varargout_2.X;
    inverse_kinematics_B.out[13] = inverse_kinematics_B.b_varargout_2.Y;
    inverse_kinematics_B.out[14] = inverse_kinematics_B.b_varargout_2.Z;

    /* MATLABSystem: '<S6>/MATLAB System' incorporates:
     *  Constant: '<S2>/weight'
     *  Delay: '<S2>/Delay'
     *  MATLABSystem: '<S2>/Coordinate Transformation Conversion1'
     */
    obj = &inverse_kinematics_DW.obj.IKInternal;
    if (inverse_kinematics_DW.obj.IKInternal.isInitialized != 1) {
      inverse_kinematics_DW.obj.IKInternal.isSetupComplete = false;
      inverse_kinematics_DW.obj.IKInternal.isInitialized = 1;
      RigidBodyTree_get_JointPosition
        (inverse_kinematics_DW.obj.IKInternal.RigidBodyTreeInternal,
         inverse_kinematics_DW.obj.IKInternal.Limits);
      inverse_kinematics_DW.obj.IKInternal.Solver->ExtraArgs = &obj->_pobj0;
      for (inverse_kinematics_B.i_o = 0; inverse_kinematics_B.i_o < 36;
           inverse_kinematics_B.i_o++) {
        inverse_kinematics_DW.obj.IKInternal.Solver->ExtraArgs->
          WeightMatrix[inverse_kinematics_B.i_o] = 0.0;
      }

      inverse_kinemati_emxInit_real_T(&tmp, 1);
      inverse_kinematics_DW.obj.IKInternal.Solver->ExtraArgs->Robot =
        inverse_kinematics_DW.obj.IKInternal.RigidBodyTreeInternal;
      inverse_kinematics_B.loop_ub =
        inverse_kinematics_DW.obj.IKInternal.Limits->size[0] << 1;
      inverse_kinematics_B.i_o =
        inverse_kinematics_DW.obj.IKInternal.Solver->ExtraArgs->Limits->size[0] *
        inverse_kinematics_DW.obj.IKInternal.Solver->ExtraArgs->Limits->size[1];
      inverse_kinematics_DW.obj.IKInternal.Solver->ExtraArgs->Limits->size[0] =
        inverse_kinematics_DW.obj.IKInternal.Limits->size[0];
      inverse_kinematics_DW.obj.IKInternal.Solver->ExtraArgs->Limits->size[1] =
        2;
      invers_emxEnsureCapacity_real_T
        (inverse_kinematics_DW.obj.IKInternal.Solver->ExtraArgs->Limits,
         inverse_kinematics_B.i_o);
      inverse_kinematics_B.i_o = tmp->size[0];
      tmp->size[0] = inverse_kinematics_B.loop_ub;
      invers_emxEnsureCapacity_real_T(tmp, inverse_kinematics_B.i_o);
      for (inverse_kinematics_B.iacol_tmp = 0; inverse_kinematics_B.iacol_tmp <
           inverse_kinematics_B.loop_ub; inverse_kinematics_B.iacol_tmp++) {
        tmp->data[inverse_kinematics_B.iacol_tmp] =
          inverse_kinematics_DW.obj.IKInternal.Limits->
          data[inverse_kinematics_B.iacol_tmp];
      }

      inverse_kinematics_B.loop_ub = tmp->size[0];
      for (inverse_kinematics_B.iacol_tmp = 0; inverse_kinematics_B.iacol_tmp <
           inverse_kinematics_B.loop_ub; inverse_kinematics_B.iacol_tmp++) {
        inverse_kinematics_DW.obj.IKInternal.Solver->ExtraArgs->Limits->
          data[inverse_kinematics_B.iacol_tmp] = tmp->
          data[inverse_kinematics_B.iacol_tmp];
      }

      inverse_kinemati_emxFree_real_T(&tmp);
      for (inverse_kinematics_B.i_o = 0; inverse_kinematics_B.i_o < 16;
           inverse_kinematics_B.i_o++) {
        inverse_kinematics_B.b_I[inverse_kinematics_B.i_o] = 0;
      }

      inverse_kinematics_B.b_I[0] = 1;
      inverse_kinematics_B.b_I[5] = 1;
      inverse_kinematics_B.b_I[10] = 1;
      inverse_kinematics_B.b_I[15] = 1;
      for (inverse_kinematics_B.i_o = 0; inverse_kinematics_B.i_o < 16;
           inverse_kinematics_B.i_o++) {
        inverse_kinematics_DW.obj.IKInternal.Solver->ExtraArgs->
          Tform[inverse_kinematics_B.i_o] =
          inverse_kinematics_B.b_I[inverse_kinematics_B.i_o];
      }

      inverse_kinematics_DW.obj.IKInternal.Solver->ExtraArgs->BodyName->size[0] =
        1;
      inverse_kinematics_DW.obj.IKInternal.Solver->ExtraArgs->BodyName->size[1] =
        0;
      inverse_kinematics_B.i_o =
        inverse_kinematics_DW.obj.IKInternal.Solver->ExtraArgs->ErrTemp->size[0];
      inverse_kinematics_DW.obj.IKInternal.Solver->ExtraArgs->ErrTemp->size[0] =
        6;
      invers_emxEnsureCapacity_real_T
        (inverse_kinematics_DW.obj.IKInternal.Solver->ExtraArgs->ErrTemp,
         inverse_kinematics_B.i_o);
      for (inverse_kinematics_B.i_o = 0; inverse_kinematics_B.i_o < 6;
           inverse_kinematics_B.i_o++) {
        inverse_kinematics_DW.obj.IKInternal.Solver->ExtraArgs->ErrTemp->
          data[inverse_kinematics_B.i_o] = 0.0;
      }

      inverse_kinemati_emxInit_int8_T(&b_gradTmp, 1);
      inverse_kinematics_DW.obj.IKInternal.Solver->ExtraArgs->CostTemp = 0.0;
      inverse_kinematics_B.i_o = b_gradTmp->size[0];
      b_gradTmp->size[0] = static_cast<int32_T>
        (inverse_kinematics_DW.obj.IKInternal.RigidBodyTreeInternal->PositionNumber);
      invers_emxEnsureCapacity_int8_T(b_gradTmp, inverse_kinematics_B.i_o);
      inverse_kinematics_B.loop_ub = static_cast<int32_T>
        (inverse_kinematics_DW.obj.IKInternal.RigidBodyTreeInternal->PositionNumber);
      if (inverse_kinematics_B.loop_ub - 1 >= 0) {
        memset(&b_gradTmp->data[0], 0, inverse_kinematics_B.loop_ub * sizeof
               (int8_T));
      }

      inverse_kinematics_B.i_o =
        inverse_kinematics_DW.obj.IKInternal.Solver->ExtraArgs->GradTemp->size[0];
      inverse_kinematics_DW.obj.IKInternal.Solver->ExtraArgs->GradTemp->size[0] =
        b_gradTmp->size[0];
      invers_emxEnsureCapacity_real_T
        (inverse_kinematics_DW.obj.IKInternal.Solver->ExtraArgs->GradTemp,
         inverse_kinematics_B.i_o);
      inverse_kinematics_B.loop_ub = b_gradTmp->size[0];
      inverse_kinemati_emxFree_int8_T(&b_gradTmp);
      for (inverse_kinematics_B.i_o = 0; inverse_kinematics_B.i_o <
           inverse_kinematics_B.loop_ub; inverse_kinematics_B.i_o++) {
        inverse_kinematics_DW.obj.IKInternal.Solver->ExtraArgs->GradTemp->
          data[inverse_kinematics_B.i_o] = 0.0;
      }

      inverse_kinematics_DW.obj.IKInternal.isSetupComplete = true;
    }

    inve_inverseKinematics_stepImpl(&inverse_kinematics_DW.obj.IKInternal,
      inverse_kinematics_B.out, inverse_kinematics_P.weight_Value,
      inverse_kinematics_DW.Delay_DSTATE, inverse_kinematics_B.MATLABSystem_o1);

    /* End of MATLABSystem: '<S6>/MATLAB System' */

    /* Update for Delay: '<S2>/Delay' incorporates:
     *  MATLABSystem: '<S6>/MATLAB System'
     */
    inverse_kinematics_DW.Delay_DSTATE[0] =
      inverse_kinematics_B.MATLABSystem_o1[0];
    inverse_kinematics_DW.Delay_DSTATE[1] =
      inverse_kinematics_B.MATLABSystem_o1[1];
    inverse_kinematics_DW.Delay_DSTATE[2] =
      inverse_kinematics_B.MATLABSystem_o1[2];
    inverse_kinematics_DW.Delay_DSTATE[3] =
      inverse_kinematics_B.MATLABSystem_o1[3];
  }

  /* End of MATLABSystem: '<S4>/SourceBlock' */
  /* End of Outputs for SubSystem: '<S4>/Enabled Subsystem' */
  /* End of Outputs for SubSystem: '<Root>/Inverse Kinematics' */
  /* End of Outputs for SubSystem: '<S1>/Subscribe' */

  /* Outputs for Atomic SubSystem: '<S3>/Header Assignment' */
  /* ASCIIToString: '<S8>/ASCII to String' incorporates:
   *  Constant: '<S7>/Constant'
   */
  inverse_kinematics_B.rtb_ASCIItoString_c[0] = static_cast<int8_T>
    (inverse_kinematics_P.Constant_Value.Header.FrameId[0]);
  inverse_kinematics_B.rtb_ASCIItoString_c[1] = static_cast<int8_T>
    (inverse_kinematics_P.Constant_Value.Header.FrameId[1]);
  inverse_kinematics_B.rtb_ASCIItoString_c[2] = static_cast<int8_T>
    (inverse_kinematics_P.Constant_Value.Header.FrameId[2]);
  inverse_kinematics_B.rtb_ASCIItoString_c[3] = static_cast<int8_T>
    (inverse_kinematics_P.Constant_Value.Header.FrameId[3]);
  memset(&inverse_kinematics_B.rtb_ASCIItoString_c[4], 0, 252U * sizeof(char_T));

  /* Switch: '<S8>/Switch1' incorporates:
   *  ASCIIToString: '<S8>/ASCII to String'
   *  Constant: '<S8>/Constant1'
   *  StringConstant: '<S8>/String Constant1'
   */
  if (inverse_kinematics_P.Constant1_Value != 0.0) {
    strncpy(&inverse_kinematics_B.Switch1[0],
            &inverse_kinematics_P.StringConstant1_String[0], 255U);
    inverse_kinematics_B.Switch1[255] = '\x00';
  } else {
    strncpy(&inverse_kinematics_B.Switch1[0],
            &inverse_kinematics_B.rtb_ASCIItoString_c[0], 255U);
    inverse_kinematics_B.Switch1[255] = '\x00';
  }

  /* End of Switch: '<S8>/Switch1' */

  /* StringToASCII: '<S8>/String To ASCII' */
  strncpy(&inverse_kinematics_B.cv[0], &inverse_kinematics_B.Switch1[0], 4U);

  /* MATLABSystem: '<S8>/Current Time' */
  currentROSTimeBus(&inverse_kinematics_B.msg.Header.Stamp);

  /* Switch: '<S8>/Switch' incorporates:
   *  Constant: '<S8>/Constant'
   */
  if (!(inverse_kinematics_P.Constant_Value_k != 0.0)) {
    /* MATLAB Function: '<S3>/MATLAB Function' incorporates:
     *  Constant: '<S7>/Constant'
     */
    inverse_kinematics_B.msg.Header.Stamp =
      inverse_kinematics_P.Constant_Value.Header.Stamp;
  }

  /* End of Switch: '<S8>/Switch' */

  /* MATLAB Function: '<S3>/MATLAB Function' incorporates:
   *  Constant: '<S7>/Constant'
   *  MATLABSystem: '<S6>/MATLAB System'
   *  SignalConversion generated from: '<S8>/HeaderAssign'
   *  StringLength: '<S8>/String Length'
   *  StringToASCII: '<S8>/String To ASCII'
   */
  inverse_kinematics_B.msg.Header.Seq =
    inverse_kinematics_P.Constant_Value.Header.Seq;
  inverse_kinematics_B.msg.Header.FrameId_SL_Info.CurrentLength = strlen
    (&inverse_kinematics_B.Switch1[0]);
  inverse_kinematics_B.msg.Header.FrameId_SL_Info.ReceivedLength =
    inverse_kinematics_P.Constant_Value.Header.FrameId_SL_Info.ReceivedLength;
  inverse_kinematics_B.msg.Name_SL_Info.ReceivedLength =
    inverse_kinematics_P.Constant_Value.Name_SL_Info.ReceivedLength;
  inverse_kinematics_B.msg.Position_SL_Info.ReceivedLength =
    inverse_kinematics_P.Constant_Value.Position_SL_Info.ReceivedLength;
  inverse_kinematics_B.msg.Velocity_SL_Info =
    inverse_kinematics_P.Constant_Value.Velocity_SL_Info;
  inverse_kinematics_B.msg.Effort_SL_Info =
    inverse_kinematics_P.Constant_Value.Effort_SL_Info;

  /* End of Outputs for SubSystem: '<S3>/Header Assignment' */
  inverse_kinematics_B.msg.Name_SL_Info.CurrentLength = 4U;
  inverse_kinematics_B.msg.Position_SL_Info.CurrentLength = 4U;

  /* Outputs for Atomic SubSystem: '<S3>/Header Assignment' */
  inverse_kinematics_B.msg.Header.FrameId[0] = static_cast<uint8_T>
    (inverse_kinematics_B.cv[0]);
  inverse_kinematics_B.msg.Name[0] = inverse_kinematics_P.Constant_Value.Name[0];
  inverse_kinematics_B.msg.Velocity[0] =
    inverse_kinematics_P.Constant_Value.Velocity[0];
  inverse_kinematics_B.msg.Effort[0] =
    inverse_kinematics_P.Constant_Value.Effort[0];

  /* End of Outputs for SubSystem: '<S3>/Header Assignment' */
  inverse_kinematics_B.msg.Position[0] = inverse_kinematics_B.MATLABSystem_o1[0];

  /* Outputs for Atomic SubSystem: '<S3>/Header Assignment' */
  inverse_kinematics_B.msg.Header.FrameId[1] = static_cast<uint8_T>
    (inverse_kinematics_B.cv[1]);
  inverse_kinematics_B.msg.Name[1] = inverse_kinematics_P.Constant_Value.Name[1];
  inverse_kinematics_B.msg.Velocity[1] =
    inverse_kinematics_P.Constant_Value.Velocity[1];
  inverse_kinematics_B.msg.Effort[1] =
    inverse_kinematics_P.Constant_Value.Effort[1];

  /* End of Outputs for SubSystem: '<S3>/Header Assignment' */
  inverse_kinematics_B.msg.Position[1] = inverse_kinematics_B.MATLABSystem_o1[1];

  /* Outputs for Atomic SubSystem: '<S3>/Header Assignment' */
  inverse_kinematics_B.msg.Header.FrameId[2] = static_cast<uint8_T>
    (inverse_kinematics_B.cv[2]);
  inverse_kinematics_B.msg.Name[2] = inverse_kinematics_P.Constant_Value.Name[2];
  inverse_kinematics_B.msg.Velocity[2] =
    inverse_kinematics_P.Constant_Value.Velocity[2];
  inverse_kinematics_B.msg.Effort[2] =
    inverse_kinematics_P.Constant_Value.Effort[2];

  /* End of Outputs for SubSystem: '<S3>/Header Assignment' */
  inverse_kinematics_B.msg.Position[2] = inverse_kinematics_B.MATLABSystem_o1[2];

  /* Outputs for Atomic SubSystem: '<S3>/Header Assignment' */
  inverse_kinematics_B.msg.Header.FrameId[3] = static_cast<uint8_T>
    (inverse_kinematics_B.cv[3]);
  inverse_kinematics_B.msg.Name[3] = inverse_kinematics_P.Constant_Value.Name[3];
  inverse_kinematics_B.msg.Velocity[3] =
    inverse_kinematics_P.Constant_Value.Velocity[3];
  inverse_kinematics_B.msg.Effort[3] =
    inverse_kinematics_P.Constant_Value.Effort[3];

  /* End of Outputs for SubSystem: '<S3>/Header Assignment' */
  inverse_kinematics_B.msg.Position[3] = inverse_kinematics_B.MATLABSystem_o1[3];
  for (inverse_kinematics_B.i_o = 0; inverse_kinematics_B.i_o < 6;
       inverse_kinematics_B.i_o++) {
    inverse_kinematics_B.msg.Name[0].Data[inverse_kinematics_B.i_o] =
      b[inverse_kinematics_B.i_o];
  }

  inverse_kinematics_B.msg.Name[0].Data_SL_Info.CurrentLength = 6U;
  for (inverse_kinematics_B.i_o = 0; inverse_kinematics_B.i_o < 8;
       inverse_kinematics_B.i_o++) {
    inverse_kinematics_B.msg.Name[1].Data[inverse_kinematics_B.i_o] =
      c[inverse_kinematics_B.i_o];
  }

  inverse_kinematics_B.msg.Name[1].Data_SL_Info.CurrentLength = 8U;
  inverse_kinematics_B.msg.Name[2].Data_SL_Info.CurrentLength = 5U;
  for (inverse_kinematics_B.i_o = 0; inverse_kinematics_B.i_o < 5;
       inverse_kinematics_B.i_o++) {
    inverse_kinematics_B.msg.Name[2].Data[inverse_kinematics_B.i_o] =
      d[inverse_kinematics_B.i_o];
    inverse_kinematics_B.msg.Name[3].Data[inverse_kinematics_B.i_o] =
      e[inverse_kinematics_B.i_o];
  }

  inverse_kinematics_B.msg.Name[3].Data_SL_Info.CurrentLength = 5U;

  /* Outputs for Atomic SubSystem: '<S3>/Publish' */
  /* MATLABSystem: '<S10>/SinkBlock' */
  Pub_inverse_kinematics_487.publish(&inverse_kinematics_B.msg);

  /* End of Outputs for SubSystem: '<S3>/Publish' */
}

/* Model initialize function */
void inverse_kinematics_initialize(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* block I/O */
  (void) memset((static_cast<void *>(&inverse_kinematics_B)), 0,
                sizeof(B_inverse_kinematics_T));

  /* states (dwork) */
  (void) memset(static_cast<void *>(&inverse_kinematics_DW), 0,
                sizeof(DW_inverse_kinematics_T));

  {
    static const char_T tmp[8] = { '/', 'C', 'o', 'm', 'm', 'a', 'n', 'd' };

    static const uint32_T tmp_0[625] = { 5489U, 1301868182U, 2938499221U,
      2950281878U, 1875628136U, 751856242U, 944701696U, 2243192071U, 694061057U,
      219885934U, 2066767472U, 3182869408U, 485472502U, 2336857883U, 1071588843U,
      3418470598U, 951210697U, 3693558366U, 2923482051U, 1793174584U,
      2982310801U, 1586906132U, 1951078751U, 1808158765U, 1733897588U,
      431328322U, 4202539044U, 530658942U, 1714810322U, 3025256284U, 3342585396U,
      1937033938U, 2640572511U, 1654299090U, 3692403553U, 4233871309U,
      3497650794U, 862629010U, 2943236032U, 2426458545U, 1603307207U,
      1133453895U, 3099196360U, 2208657629U, 2747653927U, 931059398U, 761573964U,
      3157853227U, 785880413U, 730313442U, 124945756U, 2937117055U, 3295982469U,
      1724353043U, 3021675344U, 3884886417U, 4010150098U, 4056961966U,
      699635835U, 2681338818U, 1339167484U, 720757518U, 2800161476U, 2376097373U,
      1532957371U, 3902664099U, 1238982754U, 3725394514U, 3449176889U,
      3570962471U, 4287636090U, 4087307012U, 3603343627U, 202242161U,
      2995682783U, 1620962684U, 3704723357U, 371613603U, 2814834333U,
      2111005706U, 624778151U, 2094172212U, 4284947003U, 1211977835U, 991917094U,
      1570449747U, 2962370480U, 1259410321U, 170182696U, 146300961U, 2836829791U,
      619452428U, 2723670296U, 1881399711U, 1161269684U, 1675188680U,
      4132175277U, 780088327U, 3409462821U, 1036518241U, 1834958505U,
      3048448173U, 161811569U, 618488316U, 44795092U, 3918322701U, 1924681712U,
      3239478144U, 383254043U, 4042306580U, 2146983041U, 3992780527U,
      3518029708U, 3545545436U, 3901231469U, 1896136409U, 2028528556U,
      2339662006U, 501326714U, 2060962201U, 2502746480U, 561575027U, 581893337U,
      3393774360U, 1778912547U, 3626131687U, 2175155826U, 319853231U, 986875531U,
      819755096U, 2915734330U, 2688355739U, 3482074849U, 2736559U, 2296975761U,
      1029741190U, 2876812646U, 690154749U, 579200347U, 4027461746U, 1285330465U,
      2701024045U, 4117700889U, 759495121U, 3332270341U, 2313004527U,
      2277067795U, 4131855432U, 2722057515U, 1264804546U, 3848622725U,
      2211267957U, 4100593547U, 959123777U, 2130745407U, 3194437393U, 486673947U,
      1377371204U, 17472727U, 352317554U, 3955548058U, 159652094U, 1232063192U,
      3835177280U, 49423123U, 3083993636U, 733092U, 2120519771U, 2573409834U,
      1112952433U, 3239502554U, 761045320U, 1087580692U, 2540165110U, 641058802U,
      1792435497U, 2261799288U, 1579184083U, 627146892U, 2165744623U,
      2200142389U, 2167590760U, 2381418376U, 1793358889U, 3081659520U,
      1663384067U, 2009658756U, 2689600308U, 739136266U, 2304581039U,
      3529067263U, 591360555U, 525209271U, 3131882996U, 294230224U, 2076220115U,
      3113580446U, 1245621585U, 1386885462U, 3203270426U, 123512128U, 12350217U,
      354956375U, 4282398238U, 3356876605U, 3888857667U, 157639694U, 2616064085U,
      1563068963U, 2762125883U, 4045394511U, 4180452559U, 3294769488U,
      1684529556U, 1002945951U, 3181438866U, 22506664U, 691783457U, 2685221343U,
      171579916U, 3878728600U, 2475806724U, 2030324028U, 3331164912U,
      1708711359U, 1970023127U, 2859691344U, 2588476477U, 2748146879U,
      136111222U, 2967685492U, 909517429U, 2835297809U, 3206906216U, 3186870716U,
      341264097U, 2542035121U, 3353277068U, 548223577U, 3170936588U, 1678403446U,
      297435620U, 2337555430U, 466603495U, 1132321815U, 1208589219U, 696392160U,
      894244439U, 2562678859U, 470224582U, 3306867480U, 201364898U, 2075966438U,
      1767227936U, 2929737987U, 3674877796U, 2654196643U, 3692734598U,
      3528895099U, 2796780123U, 3048728353U, 842329300U, 191554730U, 2922459673U,
      3489020079U, 3979110629U, 1022523848U, 2202932467U, 3583655201U,
      3565113719U, 587085778U, 4176046313U, 3013713762U, 950944241U, 396426791U,
      3784844662U, 3477431613U, 3594592395U, 2782043838U, 3392093507U,
      3106564952U, 2829419931U, 1358665591U, 2206918825U, 3170783123U, 31522386U,
      2988194168U, 1782249537U, 1105080928U, 843500134U, 1225290080U,
      1521001832U, 3605886097U, 2802786495U, 2728923319U, 3996284304U,
      903417639U, 1171249804U, 1020374987U, 2824535874U, 423621996U, 1988534473U,
      2493544470U, 1008604435U, 1756003503U, 1488867287U, 1386808992U,
      732088248U, 1780630732U, 2482101014U, 976561178U, 1543448953U, 2602866064U,
      2021139923U, 1952599828U, 2360242564U, 2117959962U, 2753061860U,
      2388623612U, 4138193781U, 2962920654U, 2284970429U, 766920861U,
      3457264692U, 2879611383U, 815055854U, 2332929068U, 1254853997U,
      3740375268U, 3799380844U, 4091048725U, 2006331129U, 1982546212U,
      686850534U, 1907447564U, 2682801776U, 2780821066U, 998290361U, 1342433871U,
      4195430425U, 607905174U, 3902331779U, 2454067926U, 1708133115U,
      1170874362U, 2008609376U, 3260320415U, 2211196135U, 433538229U,
      2728786374U, 2189520818U, 262554063U, 1182318347U, 3710237267U,
      1221022450U, 715966018U, 2417068910U, 2591870721U, 2870691989U,
      3418190842U, 4238214053U, 1540704231U, 1575580968U, 2095917976U,
      4078310857U, 2313532447U, 2110690783U, 4056346629U, 4061784526U,
      1123218514U, 551538993U, 597148360U, 4120175196U, 3581618160U, 3181170517U,
      422862282U, 3227524138U, 1713114790U, 662317149U, 1230418732U, 928171837U,
      1324564878U, 1928816105U, 1786535431U, 2878099422U, 3290185549U,
      539474248U, 1657512683U, 552370646U, 1671741683U, 3655312128U, 1552739510U,
      2605208763U, 1441755014U, 181878989U, 3124053868U, 1447103986U,
      3183906156U, 1728556020U, 3502241336U, 3055466967U, 1013272474U,
      818402132U, 1715099063U, 2900113506U, 397254517U, 4194863039U, 1009068739U,
      232864647U, 2540223708U, 2608288560U, 2415367765U, 478404847U, 3455100648U,
      3182600021U, 2115988978U, 434269567U, 4117179324U, 3461774077U, 887256537U,
      3545801025U, 286388911U, 3451742129U, 1981164769U, 786667016U, 3310123729U,
      3097811076U, 2224235657U, 2959658883U, 3370969234U, 2514770915U,
      3345656436U, 2677010851U, 2206236470U, 271648054U, 2342188545U,
      4292848611U, 3646533909U, 3754009956U, 3803931226U, 4160647125U,
      1477814055U, 4043852216U, 1876372354U, 3133294443U, 3871104810U,
      3177020907U, 2074304428U, 3479393793U, 759562891U, 164128153U, 1839069216U,
      2114162633U, 3989947309U, 3611054956U, 1333547922U, 835429831U, 494987340U,
      171987910U, 1252001001U, 370809172U, 3508925425U, 2535703112U, 1276855041U,
      1922855120U, 835673414U, 3030664304U, 613287117U, 171219893U, 3423096126U,
      3376881639U, 2287770315U, 1658692645U, 1262815245U, 3957234326U,
      1168096164U, 2968737525U, 2655813712U, 2132313144U, 3976047964U,
      326516571U, 353088456U, 3679188938U, 3205649712U, 2654036126U, 1249024881U,
      880166166U, 691800469U, 2229503665U, 1673458056U, 4032208375U, 1851778863U,
      2563757330U, 376742205U, 1794655231U, 340247333U, 1505873033U, 396524441U,
      879666767U, 3335579166U, 3260764261U, 3335999539U, 506221798U, 4214658741U,
      975887814U, 2080536343U, 3360539560U, 571586418U, 138896374U, 4234352651U,
      2737620262U, 3928362291U, 1516365296U, 38056726U, 3599462320U, 3585007266U,
      3850961033U, 471667319U, 1536883193U, 2310166751U, 1861637689U,
      2530999841U, 4139843801U, 2710569485U, 827578615U, 2012334720U,
      2907369459U, 3029312804U, 2820112398U, 1965028045U, 35518606U, 2478379033U,
      643747771U, 1924139484U, 4123405127U, 3811735531U, 3429660832U,
      3285177704U, 1948416081U, 1311525291U, 1183517742U, 1739192232U,
      3979815115U, 2567840007U, 4116821529U, 213304419U, 4125718577U,
      1473064925U, 2442436592U, 1893310111U, 4195361916U, 3747569474U,
      828465101U, 2991227658U, 750582866U, 1205170309U, 1409813056U, 678418130U,
      1171531016U, 3821236156U, 354504587U, 4202874632U, 3882511497U,
      1893248677U, 1903078632U, 26340130U, 2069166240U, 3657122492U, 3725758099U,
      831344905U, 811453383U, 3447711422U, 2434543565U, 4166886888U, 3358210805U,
      4142984013U, 2988152326U, 3527824853U, 982082992U, 2809155763U, 190157081U,
      3340214818U, 2365432395U, 2548636180U, 2894533366U, 3474657421U,
      2372634704U, 2845748389U, 43024175U, 2774226648U, 1987702864U, 3186502468U,
      453610222U, 4204736567U, 1392892630U, 2471323686U, 2470534280U,
      3541393095U, 4269885866U, 3909911300U, 759132955U, 1482612480U, 667715263U,
      1795580598U, 2337923983U, 3390586366U, 581426223U, 1515718634U, 476374295U,
      705213300U, 363062054U, 2084697697U, 2407503428U, 2292957699U, 2426213835U,
      2199989172U, 1987356470U, 4026755612U, 2147252133U, 270400031U,
      1367820199U, 2369854699U, 2844269403U, 79981964U, 624U };

    static const char_T tmp_1[16] = { '/', 'R', 'o', 'b', 'o', 't', '/', 'r',
      'e', 'f', '/', 'j', 'o', 'i', 'n', 't' };

    /* Start for Atomic SubSystem: '<S1>/Subscribe' */
    /* Start for MATLABSystem: '<S4>/SourceBlock' */
    inverse_kinematics_DW.obj_a.matlabCodegenIsDeleted = true;
    inverse_kinematics_DW.obj_a.isInitialized = 0;
    inverse_kinematics_DW.obj_a.matlabCodegenIsDeleted = false;
    inverse_kinematics_DW.objisempty_i = true;
    inverse_kinematics_DW.obj_a.isSetupComplete = false;
    inverse_kinematics_DW.obj_a.isInitialized = 1;
    for (inverse_kinematics_B.i_m = 0; inverse_kinematics_B.i_m < 8;
         inverse_kinematics_B.i_m++) {
      inverse_kinematics_B.b_zeroDelimTopic_n[inverse_kinematics_B.i_m] =
        tmp[inverse_kinematics_B.i_m];
    }

    inverse_kinematics_B.b_zeroDelimTopic_n[8] = '\x00';
    Sub_inverse_kinematics_482.createSubscriber
      (&inverse_kinematics_B.b_zeroDelimTopic_n[0], 1);
    inverse_kinematics_DW.obj_a.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<S4>/SourceBlock' */
    /* End of Start for SubSystem: '<S1>/Subscribe' */

    /* Start for Enabled SubSystem: '<Root>/Inverse Kinematics' */
    /* Start for MATLABSystem: '<S2>/Coordinate Transformation Conversion1' */
    inverse_kinematics_DW.obj_jv.isInitialized = 0;
    inverse_kinematics_DW.objisempty_an = true;
    inverse_kinematics_DW.obj_jv.isInitialized = 1;

    /* End of Start for SubSystem: '<Root>/Inverse Kinematics' */
    emxInitStruct_robotics_slmanip_(&inverse_kinematics_DW.obj);

    /* Start for Enabled SubSystem: '<Root>/Inverse Kinematics' */
    /* Start for MATLABSystem: '<S6>/MATLAB System' */
    for (inverse_kinematics_B.i_m = 0; inverse_kinematics_B.i_m < 15;
         inverse_kinematics_B.i_m++) {
      inverse_kinematics_DW.obj.TreeInternal._pobj0[inverse_kinematics_B.i_m].
        _pobj0.matlabCodegenIsDeleted = true;
    }

    for (inverse_kinematics_B.i_m = 0; inverse_kinematics_B.i_m < 8;
         inverse_kinematics_B.i_m++) {
      inverse_kinematics_DW.obj.IKInternal._pobj4._pobj1[inverse_kinematics_B.i_m]
        ._pobj0.matlabCodegenIsDeleted = true;
    }

    for (inverse_kinematics_B.i_m = 0; inverse_kinematics_B.i_m < 15;
         inverse_kinematics_B.i_m++) {
      inverse_kinematics_DW.obj.IKInternal._pobj3[inverse_kinematics_B.i_m].
        _pobj0.matlabCodegenIsDeleted = true;
    }

    inverse_kinematics_DW.obj.IKInternal.matlabCodegenIsDeleted = true;
    inverse_kinematics_DW.obj.matlabCodegenIsDeleted = true;
    inverse_kinematics_DW.method_a = 7U;
    inverse_kinematics_DW.freq_not_empty = true;
    inverse_kinematics_DW.state = 1144108930U;
    inverse_kinematics_DW.state_not_empty = true;
    inverse_kinematics_DW.state_p[0] = 362436069U;
    inverse_kinematics_DW.state_p[1] = 521288629U;
    inverse_kinematics_DW.state_not_empty_l = true;
    memcpy(&inverse_kinematics_DW.state_c[0], &tmp_0[0], 625U * sizeof(uint32_T));
    inverse_kinematics_DW.method_not_empty_f = true;
    inverse_kinematics_DW.method = 0U;
    inverse_kinematics_DW.state_not_empty_c = true;
    inverse_kinematics_DW.state_pz[0] = 362436069U;
    inverse_kinematics_DW.state_pz[1] = 521288629U;
    inverse_kinematics_DW.state_not_empty_b = true;
    inverse_kinematics_DW.obj.isInitialized = 0;
    inverse_kinematics_DW.obj.matlabCodegenIsDeleted = false;
    inverse_kinematics_DW.objisempty_a = true;
    inverse_kinema_SystemCore_setup(&inverse_kinematics_DW.obj);

    /* End of Start for MATLABSystem: '<S6>/MATLAB System' */
    /* End of Start for SubSystem: '<Root>/Inverse Kinematics' */

    /* Start for Atomic SubSystem: '<S3>/Header Assignment' */
    /* Start for MATLABSystem: '<S8>/Current Time' */
    inverse_kinematics_DW.obj_m.matlabCodegenIsDeleted = true;
    inverse_kinematics_DW.obj_m.isInitialized = 0;
    inverse_kinematics_DW.obj_m.matlabCodegenIsDeleted = false;
    inverse_kinematics_DW.objisempty_c = true;
    inverse_kinematics_DW.obj_m.isSetupComplete = false;
    inverse_kinematics_DW.obj_m.isInitialized = 1;
    inverse_kinematics_DW.obj_m.isSetupComplete = true;

    /* End of Start for SubSystem: '<S3>/Header Assignment' */

    /* Start for Atomic SubSystem: '<S3>/Publish' */
    /* Start for MATLABSystem: '<S10>/SinkBlock' */
    inverse_kinematics_DW.obj_j.matlabCodegenIsDeleted = true;
    inverse_kinematics_DW.obj_j.isInitialized = 0;
    inverse_kinematics_DW.obj_j.matlabCodegenIsDeleted = false;
    inverse_kinematics_DW.objisempty = true;
    inverse_kinematics_DW.obj_j.isSetupComplete = false;
    inverse_kinematics_DW.obj_j.isInitialized = 1;
    for (inverse_kinematics_B.i_m = 0; inverse_kinematics_B.i_m < 16;
         inverse_kinematics_B.i_m++) {
      inverse_kinematics_B.b_zeroDelimTopic[inverse_kinematics_B.i_m] =
        tmp_1[inverse_kinematics_B.i_m];
    }

    inverse_kinematics_B.b_zeroDelimTopic[16] = '\x00';
    Pub_inverse_kinematics_487.createPublisher
      (&inverse_kinematics_B.b_zeroDelimTopic[0], 1);
    inverse_kinematics_DW.obj_j.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<S10>/SinkBlock' */
    /* End of Start for SubSystem: '<S3>/Publish' */
  }

  /* SystemInitialize for Enabled SubSystem: '<Root>/Inverse Kinematics' */
  /* InitializeConditions for Delay: '<S2>/Delay' */
  inverse_kinematics_DW.Delay_DSTATE[0] =
    inverse_kinematics_P.Delay_InitialCondition[0];

  /* SystemInitialize for MATLABSystem: '<S6>/MATLAB System' incorporates:
   *  Outport: '<S2>/Config'
   */
  inverse_kinematics_B.MATLABSystem_o1[0] = inverse_kinematics_P.Config_Y0;

  /* InitializeConditions for Delay: '<S2>/Delay' */
  inverse_kinematics_DW.Delay_DSTATE[1] =
    inverse_kinematics_P.Delay_InitialCondition[1];

  /* SystemInitialize for MATLABSystem: '<S6>/MATLAB System' incorporates:
   *  Outport: '<S2>/Config'
   */
  inverse_kinematics_B.MATLABSystem_o1[1] = inverse_kinematics_P.Config_Y0;

  /* InitializeConditions for Delay: '<S2>/Delay' */
  inverse_kinematics_DW.Delay_DSTATE[2] =
    inverse_kinematics_P.Delay_InitialCondition[2];

  /* SystemInitialize for MATLABSystem: '<S6>/MATLAB System' incorporates:
   *  Outport: '<S2>/Config'
   */
  inverse_kinematics_B.MATLABSystem_o1[2] = inverse_kinematics_P.Config_Y0;

  /* InitializeConditions for Delay: '<S2>/Delay' */
  inverse_kinematics_DW.Delay_DSTATE[3] =
    inverse_kinematics_P.Delay_InitialCondition[3];

  /* SystemInitialize for MATLABSystem: '<S6>/MATLAB System' incorporates:
   *  Outport: '<S2>/Config'
   */
  inverse_kinematics_B.MATLABSystem_o1[3] = inverse_kinematics_P.Config_Y0;

  /* End of SystemInitialize for SubSystem: '<Root>/Inverse Kinematics' */
}

/* Model terminate function */
void inverse_kinematics_terminate(void)
{
  void* geometryInternal;
  b_inverseKinematics_inverse_k_T *obj;
  k_robotics_manip_internal_Col_T *obj_0;

  /* Terminate for Atomic SubSystem: '<S1>/Subscribe' */
  /* Terminate for MATLABSystem: '<S4>/SourceBlock' */
  if (!inverse_kinematics_DW.obj_a.matlabCodegenIsDeleted) {
    inverse_kinematics_DW.obj_a.matlabCodegenIsDeleted = true;
  }

  /* End of Terminate for MATLABSystem: '<S4>/SourceBlock' */
  /* End of Terminate for SubSystem: '<S1>/Subscribe' */

  /* Terminate for Enabled SubSystem: '<Root>/Inverse Kinematics' */
  /* Terminate for MATLABSystem: '<S6>/MATLAB System' */
  if (!inverse_kinematics_DW.obj.matlabCodegenIsDeleted) {
    inverse_kinematics_DW.obj.matlabCodegenIsDeleted = true;
  }

  obj = &inverse_kinematics_DW.obj.IKInternal;
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
    if (obj->isInitialized == 1) {
      obj->isInitialized = 2;
    }
  }

  for (int32_T b = 0; b < 15; b++) {
    obj_0 = &inverse_kinematics_DW.obj.IKInternal._pobj3[b]._pobj0;
    if (!obj_0->matlabCodegenIsDeleted) {
      obj_0->matlabCodegenIsDeleted = true;
      geometryInternal = obj_0->CollisionPrimitive;
      collisioncodegen_destructGeometry(&geometryInternal);
    }
  }

  for (int32_T b = 0; b < 8; b++) {
    obj_0 = &inverse_kinematics_DW.obj.IKInternal._pobj4._pobj1[b]._pobj0;
    if (!obj_0->matlabCodegenIsDeleted) {
      obj_0->matlabCodegenIsDeleted = true;
      geometryInternal = obj_0->CollisionPrimitive;
      collisioncodegen_destructGeometry(&geometryInternal);
    }
  }

  for (int32_T b = 0; b < 15; b++) {
    obj_0 = &inverse_kinematics_DW.obj.TreeInternal._pobj0[b]._pobj0;
    if (!obj_0->matlabCodegenIsDeleted) {
      obj_0->matlabCodegenIsDeleted = true;
      geometryInternal = obj_0->CollisionPrimitive;
      collisioncodegen_destructGeometry(&geometryInternal);
    }
  }

  /* End of Terminate for MATLABSystem: '<S6>/MATLAB System' */
  /* End of Terminate for SubSystem: '<Root>/Inverse Kinematics' */
  emxFreeStruct_robotics_slmanip_(&inverse_kinematics_DW.obj);

  /* Terminate for Atomic SubSystem: '<S3>/Header Assignment' */
  /* Terminate for MATLABSystem: '<S8>/Current Time' */
  if (!inverse_kinematics_DW.obj_m.matlabCodegenIsDeleted) {
    inverse_kinematics_DW.obj_m.matlabCodegenIsDeleted = true;
  }

  /* End of Terminate for MATLABSystem: '<S8>/Current Time' */
  /* End of Terminate for SubSystem: '<S3>/Header Assignment' */

  /* Terminate for Atomic SubSystem: '<S3>/Publish' */
  /* Terminate for MATLABSystem: '<S10>/SinkBlock' */
  if (!inverse_kinematics_DW.obj_j.matlabCodegenIsDeleted) {
    inverse_kinematics_DW.obj_j.matlabCodegenIsDeleted = true;
  }

  /* End of Terminate for MATLABSystem: '<S10>/SinkBlock' */
  /* End of Terminate for SubSystem: '<S3>/Publish' */
}
