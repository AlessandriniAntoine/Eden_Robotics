/*
 * InverseKinematics.h
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "InverseKinematics".
 *
 * Model version              : 4.21
 * Simulink Coder version : 9.7 (R2022a) 13-Nov-2021
 * C++ source code generated on : Fri Jul 29 16:24:00 2022
 *
 * Target selection: ert.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Intel->x86-64 (Linux 64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef RTW_HEADER_InverseKinematics_h_
#define RTW_HEADER_InverseKinematics_h_
#include <string.h>
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "coder_posix_time.h"
#include "collisioncodegen_api.hpp"
#include "InverseKinematics_pubsub_common.h"
#include "InverseKinematics_types.h"

extern "C" {

#include "rtGetNaN.h"

}
  extern "C"
{

#include "rt_nonfinite.h"

}

extern "C" {

#include "rtGetInf.h"

}
#include <stddef.h>

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

/* Block signals (default storage) */
struct B_InverseKinematics_T {
  real_T xi[257];
  real_T weightMatrix[36];
  real_T weightMatrix_m[36];
  real_T a[36];
  real_T Tj[36];
  real_T obj[36];
  real_T out[16];
  real_T Td[16];
  real_T T_data[16];
  real_T T1[16];
  real_T Tc2p[16];
  real_T Tj_c[16];
  real_T T1j[16];
  real_T b[16];
  real_T Tj_k[16];
  real_T obj_c[16];
  real_T in2[16];
  real_T c_x[16];
  real_T poslim_data[12];
  real_T poslim_data_b[12];
  real_T poslim_data_p[12];
  real_T poslim_data_c[12];
  real_T poslim_data_f[12];
  real_T R[9];
  real_T tempR[9];
  real_T obj_g[9];
  real_T y[9];
  real_T V[9];
  real_T b_U[9];
  real_T T[9];
  real_T A[9];
  real_T A_g[9];
  real_T e[6];
  real_T e_m[6];
  real_T unusedExpr[5];
  int8_T msubspace_data[36];
  int8_T msubspace_data_n[36];
  int8_T msubspace_data_p[36];
  int8_T msubspace_data_l[36];
  int8_T msubspace_data_j[36];
  SL_Bus_geometry_msgs_Quaternion BusAssignment;/* '<S2>/Bus Assignment' */
  real_T b_varargout_1[4];
  real_T qvSolRaw[4];
  real_T c_xSol[4];
  real_T step_data[4];
  real_T y_d[4];
  real_T result_data[4];
  SL_Bus_geometry_msgs_Point In1;      /* '<S7>/In1' */
  SL_Bus_geometry_msgs_Point b_varargout_2;
  real_T v[3];
  real_T v_g[3];
  real_T vspecial_data[3];
  real_T v_l[3];
  real_T s[3];
  real_T e_d[3];
  real_T work[3];
  char_T switch_expression[18];
  char_T b_d[18];
  int8_T b_I[16];
  int32_T indicesUpperBoundViolation_data[4];
  int32_T tmp_data[4];
  creal_T v_lx;
  creal_T u;
  creal_T u_o;
  creal_T dc;
  real_T ub[2];
  char_T expl_temp_data[14];
  int8_T b_I_b[9];
  char_T b_n[9];
  int8_T b_I_bs[9];
  char_T partial_match_data[9];
  char_T b_vstr[9];
  char_T b_l[9];
  char_T b_h[9];
  char_T b_b[9];
  int8_T b_I_d[9];
  char_T b_e[9];
  char_T b_bj[8];
  char_T b_j[8];
  char_T vstr[8];
  char_T b_f[8];
  char_T b_a[8];
  char_T b_ju[8];
  char_T b_jz[8];
  real_T expl_temp;
  real_T expl_temp_n;
  real_T expl_temp_i;
  real_T expl_temp_o;
  real_T bid;
  real_T ndbl;
  real_T apnd;
  real_T cdiff;
  real_T u0;
  real_T u1;
  real_T numPositions_tmp;
  real_T tol;
  real_T err;
  real_T iter;
  real_T lb;
  real_T cc;
  real_T cost;
  real_T d;
  real_T scale;
  real_T absxk;
  real_T t;
  real_T xprev_idx_0;
  real_T xprev_idx_1;
  real_T xprev_idx_2;
  real_T xprev_idx_3;
  real_T bid1;
  real_T bid2;
  real_T sth;
  real_T tempR_tmp;
  real_T tempR_tmp_n;
  real_T tempR_tmp_m;
  real_T bid1_tmp;
  real_T i;
  real_T b_r;
  real_T x;
  real_T d_u;
  real_T params_ErrorChangeTolerance;
  real_T params_DampingBias;
  real_T bid_c;
  real_T b_m;
  real_T pid;
  real_T b_index;
  real_T obj_idx_0;
  real_T obj_idx_1;
  real_T obj_idx_2;
  real_T a_m;
  real_T q;
  real_T nrm;
  real_T rt;
  real_T ztest;
  real_T smm1;
  real_T emm1;
  real_T sqds;
  real_T b_ja;
  real_T a__3;
  real_T d_sn;
  real_T roe;
  real_T absa;
  real_T absb;
  real_T scale_h;
  real_T ads;
  real_T bds;
  real_T smax;
  real_T s_c;
  real_T b_c;
  real_T c;
  real_T s_p;
  uint32_T u32[2];
  void* defaultCollisionObj_GeometryInt;
  int32_T expl_temp_size[2];
  int32_T T_size[2];
  int8_T iv[6];
  int8_T iv1[6];
  int8_T iv2[6];
  int8_T iv3[6];
  int8_T iv4[6];
  char_T b_p[5];
  char_T b_af[5];
  char_T c_vstr[5];
  char_T b_ev[5];
  int8_T b_ipiv[4];
  int32_T b_jcol;
  int32_T loop_ub;
  int32_T iacol_tmp;
  int32_T b_j_i;
  int32_T nm1d2;
  int32_T c_l;
  int32_T b_k;
  int32_T loop_ub_o;
  int32_T indicesUpperBoundViolation;
  int32_T tmp_size;
  int32_T indicesUpperBoundViolation_size;
  int32_T c_i;
  int32_T b_i;
  int32_T ix;
  int32_T nx;
  int32_T scalarLB;
  int32_T vectorUB;
  int32_T b_i_f;
  int32_T m;
  int32_T coffset;
  int32_T boffset;
  int32_T aoffset;
  int32_T kend;
  int32_T b_k_i;
  int32_T J;
  int32_T step_size;
  int32_T jointSign;
  int32_T c_f;
  int32_T f;
  int32_T g;
  int32_T i_g;
  int32_T loop_ub_c;
  int32_T Jac;
  int32_T result_data_tmp;
  int32_T minPathLength;
  int32_T b_i_o;
  int32_T e_l;
  int32_T h;
  int32_T j;
  int32_T loop_ub_m;
  int32_T i_m;
  int32_T loop_ub_cn;
  int32_T loop_ub_tmp;
  int32_T newNumel;
  int32_T i_f;
  int32_T b_k_p;
  int32_T i_e;
  int32_T d_tmp;
  int32_T b_j_o;
  int32_T b_kk;
  int32_T i_h;
  int32_T ret;
  int32_T b_kstr;
  int32_T loop_ub_l;
  int32_T iobj_3;
  int32_T obj_tmp;
  int32_T b_kstr_h;
  int32_T loop_ub_me;
  int32_T i1;
  int32_T nmatched;
  int32_T minnanb;
  int32_T loop_ub_mc;
  int32_T partial_match_size_idx_1;
  int32_T b_kstr_h3;
  int32_T loop_ub_cs;
  int32_T i2;
  int32_T newNumel_k;
  int32_T i_p;
  int32_T b_k_px;
  int32_T i3;
  int32_T y_tmp;
  int32_T T_tmp;
  int32_T T_tmp_p;
  int32_T q_a;
  int32_T qp1;
  int32_T qq;
  int32_T qjj;
  int32_T m_j;
  int32_T b_ek;
  int32_T k;
  int32_T vectorUB_o;
  int32_T qq_tmp;
  int32_T i4;
  int32_T aux_0_1;
  int32_T stride_0_0;
  int32_T stride_0_1;
  int32_T in2_tmp;
  int32_T jj;
  int32_T c_b;
  int32_T c_a;
  int32_T a_g;
  int32_T kAcol;
  int32_T jA;
  int32_T j_e;
  int32_T c_fi;
  int32_T ijA;
  int32_T b_i_h;
  int32_T loop_ub_e;
  int32_T i5;
  int32_T i6;
  int32_T i_c;
  int32_T b_kstr_a;
  int32_T loop_ub_d;
  int32_T i7;
  int32_T b_kstr_af;
  int32_T loop_ub_p;
  int32_T i8;
  int32_T d_m;
  int32_T b_i_o3;
  int32_T newNumel_n;
  int32_T i_l;
  int32_T n;
  int32_T b_j_p;
  int32_T b_i_p;
  int32_T b_k_f;
  int32_T coffset_tmp;
  int32_T i_i;
  int32_T b_kstr_o;
  int32_T i9;
  uint32_T mti;
  uint32_T y_k;
  boolean_T ubOK[4];
  boolean_T lbOK[4];
  boolean_T ubOK_a[4];
  boolean_T x_a[4];
  c_robotics_core_internal_NLPS_T exitFlag;
  c_robotics_core_internal_NLPS_T exitFlagPrev;
  int8_T y_tmp_i[3];
  int8_T y_tmp_o;
  boolean_T b_varargout_1_m;
  boolean_T y_c;
  boolean_T y_f;
  boolean_T flag;
  boolean_T nextBodyIsParent;
  boolean_T b_bool;
  boolean_T params_UseErrorDamping;
  boolean_T b_bool_h;
  boolean_T b_bool_m;
  boolean_T matched;
  boolean_T b_bool_a;
  boolean_T b_bool_k;
  boolean_T e_p;
  boolean_T x_b;
  boolean_T rEQ0;
  boolean_T apply_transform;
  boolean_T b_bool_c;
  boolean_T b_bool_n;
  boolean_T d_i;
};

/* Block states (default storage) for system '<Root>' */
struct DW_InverseKinematics_T {
  robotics_slmanip_internal_blo_T obj; /* '<S4>/MATLAB System' */
  ros_slros2_internal_block_Pub_T obj_k;/* '<S6>/SinkBlock' */
  ros_slros2_internal_block_Sub_T obj_n;/* '<S3>/SourceBlock' */
  real_T Delay_DSTATE[4];              /* '<S1>/Delay' */
  real_T freq;                         /* '<S4>/MATLAB System' */
  uint32_T state;                      /* '<S4>/MATLAB System' */
  uint32_T state_p[2];                 /* '<S4>/MATLAB System' */
  uint32_T state_c[625];               /* '<S4>/MATLAB System' */
  uint32_T method;                     /* '<S4>/MATLAB System' */
  uint32_T method_a;                   /* '<S4>/MATLAB System' */
  uint32_T state_pz[2];                /* '<S4>/MATLAB System' */
  robotics_slcore_internal_bloc_T obj_j;
                              /* '<S1>/Coordinate Transformation Conversion1' */
  boolean_T objisempty;                /* '<S3>/SourceBlock' */
  boolean_T objisempty_b;              /* '<S6>/SinkBlock' */
  boolean_T objisempty_a;              /* '<S4>/MATLAB System' */
  boolean_T state_not_empty;           /* '<S4>/MATLAB System' */
  boolean_T state_not_empty_b;         /* '<S4>/MATLAB System' */
  boolean_T state_not_empty_c;         /* '<S4>/MATLAB System' */
  boolean_T method_not_empty;          /* '<S4>/MATLAB System' */
  boolean_T freq_not_empty;            /* '<S4>/MATLAB System' */
  boolean_T method_not_empty_f;        /* '<S4>/MATLAB System' */
  boolean_T state_not_empty_l;         /* '<S4>/MATLAB System' */
  boolean_T objisempty_an;    /* '<S1>/Coordinate Transformation Conversion1' */
};

/* Parameters (default storage) */
struct P_InverseKinematics_T_ {
  SL_Bus_geometry_msgs_Quaternion Constant_Value;/* Computed Parameter: Constant_Value
                                                  * Referenced by: '<S5>/Constant'
                                                  */
  SL_Bus_geometry_msgs_Point Out1_Y0;  /* Computed Parameter: Out1_Y0
                                        * Referenced by: '<S7>/Out1'
                                        */
  SL_Bus_geometry_msgs_Point Constant_Value_k;/* Computed Parameter: Constant_Value_k
                                               * Referenced by: '<S3>/Constant'
                                               */
  real_T Constant1_Value[6];           /* Expression: [0 0 0 1 1 1]
                                        * Referenced by: '<S1>/Constant1'
                                        */
  real_T Delay_InitialCondition[4];    /* Expression: ([0 -10 0 90]*pi/180)'
                                        * Referenced by: '<S1>/Delay'
                                        */
};

/* Real-time Model Data Structure */
struct tag_RTM_InverseKinematics_T {
  const char_T *errorStatus;
};

/* Class declaration for model InverseKinematics */
class InverseKinematics
{
  /* public data and function members */
 public:
  /* Real-Time Model get method */
  RT_MODEL_InverseKinematics_T * getRTM();

  /* model start function */
  void start();

  /* Initial conditions function */
  void initialize();

  /* model step function */
  void step();

  /* model terminate function */
  void terminate();

  /* Constructor */
  InverseKinematics();

  /* Destructor */
  ~InverseKinematics();

  /* private data and function members */
 private:
  /* Block signals */
  B_InverseKinematics_T InverseKinematics_B;

  /* Block states */
  DW_InverseKinematics_T InverseKinematics_DW;

  /* Tunable parameters */
  static P_InverseKinematics_T InverseKinematics_P;

  /* private member function(s) for subsystem '<Root>'*/
  void InverseKine_SystemCore_setup_nl(ros_slros2_internal_block_Sub_T *obj);
  void InverseKinematic_emxInit_char_T(emxArray_char_T_InverseKinema_T
    **pEmxArray, int32_T numDimensions);
  void emxInitStruct_t_robotics_manip_(t_robotics_manip_internal_Rig_T *pStruct);
  void InverseK_emxInit_unnamed_struct(emxArray_unnamed_struct_Inver_T
    **pEmxArray, int32_T numDimensions);
  void emxInitStruct_l_robotics_manip_(l_robotics_manip_internal_Col_T *pStruct);
  void emxInitMatrix_l_robotics_manip_(l_robotics_manip_internal_Col_T pMatrix
    [13]);
  void InverseKinematic_emxInit_real_T(emxArray_real_T_InverseKinema_T
    **pEmxArray, int32_T numDimensions);
  void emxInitStruct_c_rigidBodyJoint(c_rigidBodyJoint_InverseKinem_T *pStruct);
  void emxInitMatrix_c_rigidBodyJoint(c_rigidBodyJoint_InverseKinem_T pMatrix[13]);
  void emxInitMatrix_t_robotics_manip_(t_robotics_manip_internal_Rig_T pMatrix
    [12]);
  void emxInitStruct_u_robotics_manip_(u_robotics_manip_internal_Rig_T *pStruct);
  void emxInitStruct_f_robotics_manip_(f_robotics_manip_internal_IKE_T *pStruct);
  void emxInitMatrix_c_rigidBodyJoint1(c_rigidBodyJoint_InverseKinem_T pMatrix
    [12]);
  void emxInitMatrix_t_robotics_mani_n(t_robotics_manip_internal_Rig_T pMatrix[6]);
  void emxInitMatrix_l_robotics_mani_n(l_robotics_manip_internal_Col_T pMatrix[7]);
  void emxInitMatrix_c_rigidBodyJoint2(c_rigidBodyJoint_InverseKinem_T pMatrix[7]);
  void emxInitStruct_v_robotics_manip_(v_robotics_manip_internal_Rig_T *pStruct);
  void emxInitStruct_b_inverseKinemati(b_inverseKinematics_InverseKi_T *pStruct);
  void emxInitStruct_robotics_slmanip_(robotics_slmanip_internal_blo_T *pStruct);
  void Invers_emxEnsureCapacity_char_T(emxArray_char_T_InverseKinema_T *emxArray,
    int32_T oldNumel);
  void InverseKinematic_emxFree_char_T(emxArray_char_T_InverseKinema_T
    **pEmxArray);
  void Invers_emxEnsureCapacity_real_T(emxArray_real_T_InverseKinema_T *emxArray,
    int32_T oldNumel);
  void emxEnsureCapacity_unnamed_struc(emxArray_unnamed_struct_Inver_T *emxArray,
    int32_T oldNumel);
  void InverseK_emxFree_unnamed_struct(emxArray_unnamed_struct_Inver_T
    **pEmxArray);
  l_robotics_manip_internal_Col_T *Inver_CollisionSet_CollisionSet
    (l_robotics_manip_internal_Col_T *obj, real_T maxElements);
  t_robotics_manip_internal_Rig_T *InverseKine_RigidBody_RigidBody
    (t_robotics_manip_internal_Rig_T *obj, l_robotics_manip_internal_Col_T
     *iobj_0, c_rigidBodyJoint_InverseKinem_T *iobj_1);
  t_robotics_manip_internal_Rig_T *InverseKi_RigidBody_RigidBody_n
    (t_robotics_manip_internal_Rig_T *obj, l_robotics_manip_internal_Col_T
     *iobj_0, c_rigidBodyJoint_InverseKinem_T *iobj_1);
  t_robotics_manip_internal_Rig_T *InverseK_RigidBody_RigidBody_nl
    (t_robotics_manip_internal_Rig_T *obj, l_robotics_manip_internal_Col_T
     *iobj_0, c_rigidBodyJoint_InverseKinem_T *iobj_1);
  t_robotics_manip_internal_Rig_T *Inverse_RigidBody_RigidBody_nly
    (t_robotics_manip_internal_Rig_T *obj, l_robotics_manip_internal_Col_T
     *iobj_0, c_rigidBodyJoint_InverseKinem_T *iobj_1);
  t_robotics_manip_internal_Rig_T *Invers_RigidBody_RigidBody_nlyk
    (t_robotics_manip_internal_Rig_T *obj, l_robotics_manip_internal_Col_T
     *iobj_0, c_rigidBodyJoint_InverseKinem_T *iobj_1);
  t_robotics_manip_internal_Rig_T *Inver_RigidBody_RigidBody_nlyk3
    (t_robotics_manip_internal_Rig_T *obj, l_robotics_manip_internal_Col_T
     *iobj_0, c_rigidBodyJoint_InverseKinem_T *iobj_1);
  t_robotics_manip_internal_Rig_T *Inve_RigidBody_RigidBody_nlyk3r
    (t_robotics_manip_internal_Rig_T *obj, l_robotics_manip_internal_Col_T
     *iobj_0, c_rigidBodyJoint_InverseKinem_T *iobj_1);
  t_robotics_manip_internal_Rig_T *Inv_RigidBody_RigidBody_nlyk3r3
    (t_robotics_manip_internal_Rig_T *obj, l_robotics_manip_internal_Col_T
     *iobj_0, c_rigidBodyJoint_InverseKinem_T *iobj_1);
  t_robotics_manip_internal_Rig_T *In_RigidBody_RigidBody_nlyk3r3m
    (t_robotics_manip_internal_Rig_T *obj, l_robotics_manip_internal_Col_T
     *iobj_0, c_rigidBodyJoint_InverseKinem_T *iobj_1);
  t_robotics_manip_internal_Rig_T *I_RigidBody_RigidBody_nlyk3r3my
    (t_robotics_manip_internal_Rig_T *obj, l_robotics_manip_internal_Col_T
     *iobj_0, c_rigidBodyJoint_InverseKinem_T *iobj_1);
  c_rigidBodyJoint_InverseKinem_T *I_rigidBodyJoint_rigidBodyJoint
    (c_rigidBodyJoint_InverseKinem_T *obj, const emxArray_char_T_InverseKinema_T
     *jname);
  u_robotics_manip_internal_Rig_T *Inv_RigidBodyTree_RigidBodyTree
    (u_robotics_manip_internal_Rig_T *obj);
  void Inverse_genrand_uint32_vector_n(uint32_T mt[625], uint32_T u[2]);
  boolean_T InverseKinematic_is_valid_state(const uint32_T mt[625]);
  void InverseKinematics_rand(real_T r[5]);
  boolean_T InverseKinematics_strcmp(const emxArray_char_T_InverseKinema_T *a,
    const emxArray_char_T_InverseKinema_T *b);
  real_T RigidBodyTree_findBodyIndexByNa(v_robotics_manip_internal_Rig_T *obj,
    const emxArray_char_T_InverseKinema_T *bodyname);
  void InverseKinematic_emxFree_real_T(emxArray_real_T_InverseKinema_T
    **pEmxArray);
  t_robotics_manip_internal_Rig_T *InverseKinematic_RigidBody_copy
    (t_robotics_manip_internal_Rig_T *obj, l_robotics_manip_internal_Col_T
     *iobj_0, c_rigidBodyJoint_InverseKinem_T *iobj_1,
     t_robotics_manip_internal_Rig_T *iobj_2);
  void InverseKi_RigidBodyTree_addBody(v_robotics_manip_internal_Rig_T *obj,
    t_robotics_manip_internal_Rig_T *bodyin, const
    emxArray_char_T_InverseKinema_T *parentName, c_rigidBodyJoint_InverseKinem_T
    *iobj_0, t_robotics_manip_internal_Rig_T *iobj_1,
    l_robotics_manip_internal_Col_T *iobj_2);
  void inverseKinematics_set_RigidBody(b_inverseKinematics_InverseKi_T *obj,
    u_robotics_manip_internal_Rig_T *rigidbodytree,
    c_rigidBodyJoint_InverseKinem_T *iobj_0, t_robotics_manip_internal_Rig_T
    *iobj_1, l_robotics_manip_internal_Col_T *iobj_2,
    v_robotics_manip_internal_Rig_T *iobj_3);
  void InverseKinemat_SystemCore_setup(robotics_slmanip_internal_blo_T *obj);
  void InverseKinem_SystemCore_setup_n(ros_slros2_internal_block_Pub_T *obj);
  void RigidBodyTree_get_JointPosition(v_robotics_manip_internal_Rig_T *obj,
    emxArray_real_T_InverseKinema_T *limits);
  void InverseKinematic_emxInit_int8_T(emxArray_int8_T_InverseKinema_T
    **pEmxArray, int32_T numDimensions);
  void Invers_emxEnsureCapacity_int8_T(emxArray_int8_T_InverseKinema_T *emxArray,
    int32_T oldNumel);
  void InverseKinematic_emxFree_int8_T(emxArray_int8_T_InverseKinema_T
    **pEmxArray);
  void InverseKin_binary_expand_op_nly(boolean_T in1[4], const real_T in2[4],
    const emxArray_real_T_InverseKinema_T *in3);
  void InverseKine_binary_expand_op_nl(boolean_T in1[4], const real_T in2[4],
    const emxArray_real_T_InverseKinema_T *in3);
  void InverseKinematics_eml_find(const boolean_T x[4], int32_T i_data[],
    int32_T *i_size);
  void InverseKinematics_tic(real_T *tstart_tv_sec, real_T *tstart_tv_nsec);
  void I_RigidBodyTree_ancestorIndices(v_robotics_manip_internal_Rig_T *obj,
    t_robotics_manip_internal_Rig_T *body, emxArray_real_T_InverseKinema_T
    *indices);
  void RigidBodyTree_kinematicPathInte(v_robotics_manip_internal_Rig_T *obj,
    t_robotics_manip_internal_Rig_T *body1, t_robotics_manip_internal_Rig_T
    *body2, emxArray_real_T_InverseKinema_T *indices);
  void In_rigidBodyJoint_get_JointAxis(const c_rigidBodyJoint_InverseKinem_T
    *obj, real_T ax[3]);
  void InverseKinematics_cat(real_T varargin_1, real_T varargin_2, real_T
    varargin_3, real_T varargin_4, real_T varargin_5, real_T varargin_6, real_T
    varargin_7, real_T varargin_8, real_T varargin_9, real_T y[9]);
  void InverseKinematics_mtimes(const real_T A[36], const
    emxArray_real_T_InverseKinema_T *B, emxArray_real_T_InverseKinema_T *C);
  void RigidBodyTree_efficientFKAndJac(v_robotics_manip_internal_Rig_T *obj,
    const real_T qv[4], const emxArray_char_T_InverseKinema_T *body1Name, real_T
    T_data[], int32_T T_size[2], emxArray_real_T_InverseKinema_T *Jac);
  creal_T InverseKinematics_sqrt(const creal_T x);
  real_T InverseKinematics_xnrm2(int32_T n, const real_T x[9], int32_T ix0);
  real_T InverseKinematics_xdotc(int32_T n, const real_T x[9], int32_T ix0,
    const real_T y[9], int32_T iy0);
  void InverseKinematics_xaxpy(int32_T n, real_T a, int32_T ix0, const real_T y
    [9], int32_T iy0, real_T b_y[9]);
  real_T InverseKinematics_xnrm2_n(const real_T x[3], int32_T ix0);
  void InverseKinematics_xaxpy_nly(int32_T n, real_T a, const real_T x[9],
    int32_T ix0, real_T y[3], int32_T iy0);
  void InverseKinematics_xaxpy_nl(int32_T n, real_T a, const real_T x[3],
    int32_T ix0, const real_T y[9], int32_T iy0, real_T b_y[9]);
  void InverseKinematics_xswap(const real_T x[9], int32_T ix0, int32_T iy0,
    real_T b_x[9]);
  void InverseKinematics_xrotg(real_T a, real_T b, real_T *b_a, real_T *b_b,
    real_T *c, real_T *s);
  void InverseKinematics_xrot(const real_T x[9], int32_T ix0, int32_T iy0,
    real_T c, real_T s, real_T b_x[9]);
  void InverseKinematics_svd(const real_T A[9], real_T U[9], real_T s[3], real_T
    V[9]);
  void InverseKine_IKHelpers_poseError(const real_T Td[16], const real_T T_data[],
    const int32_T T_size[2], real_T errorvec[6]);
  void InverseKinematics_mtimes_n(const real_T A[6], const
    emxArray_real_T_InverseKinema_T *B, emxArray_real_T_InverseKinema_T *C);
  void InverseKinema_emxInit_boolean_T(emxArray_boolean_T_InverseKin_T
    **pEmxArray, int32_T numDimensions);
  real_T InverseKinematics_norm_n(const real_T x[6]);
  void InverseKinematics_minus_n(emxArray_real_T_InverseKinema_T *in1, const
    emxArray_real_T_InverseKinema_T *in2);
  void Inv_emxEnsureCapacity_boolean_T(emxArray_boolean_T_InverseKin_T *emxArray,
    int32_T oldNumel);
  real_T InverseKinematics_toc(real_T tstart_tv_sec, real_T tstart_tv_nsec);
  void InverseKinematics_mldivide(const real_T A[16], const
    emxArray_real_T_InverseKinema_T *B, real_T Y_data[], int32_T *Y_size);
  void InverseKinem_binary_expand_op_n(real_T in1_data[], int32_T *in1_size,
    const emxArray_real_T_InverseKinema_T *in2, real_T in3, const real_T in4[16],
    const emxArray_real_T_InverseKinema_T *in5);
  void InverseKinematics_expand_max(const emxArray_real_T_InverseKinema_T *a,
    const real_T b[4], real_T c[4]);
  void InverseKinematics_expand_min(const emxArray_real_T_InverseKinema_T *a,
    const real_T b[4], real_T c[4]);
  void InverseKinema_emxFree_boolean_T(emxArray_boolean_T_InverseKin_T
    **pEmxArray);
  void ErrorDampedLevenbergMarquardt_s(h_robotics_core_internal_Erro_T *obj,
    real_T xSol[4], c_robotics_core_internal_NLPS_T *exitFlag, real_T *en,
    real_T *iter);
  boolean_T InverseKinematics_any(const emxArray_boolean_T_InverseKin_T *x);
  void InverseKinematics_randn(const real_T varargin_1[2],
    emxArray_real_T_InverseKinema_T *r);
  void InverseKinematics_minus(emxArray_real_T_InverseKinema_T *in1, const
    emxArray_real_T_InverseKinema_T *in2);
  void InverseKinematics_plus(emxArray_real_T_InverseKinema_T *in1, const
    emxArray_real_T_InverseKinema_T *in2);
  void InverseKinematics_rand_n(real_T varargin_1,
    emxArray_real_T_InverseKinema_T *r);
  void InverseKinemat_binary_expand_op(emxArray_real_T_InverseKinema_T *in1,
    const emxArray_real_T_InverseKinema_T *in2, const
    emxArray_real_T_InverseKinema_T *in3);
  void Invers_NLPSolverInterface_solve(h_robotics_core_internal_Erro_T *obj,
    const real_T seed[4], real_T xSol[4], real_T *solutionInfo_Iterations,
    real_T *solutionInfo_RRAttempts, real_T *solutionInfo_Error, real_T
    *solutionInfo_ExitFlag, char_T solutionInfo_Status_data[], int32_T
    solutionInfo_Status_size[2]);
  void InverseKinemati_emxInit_int32_T(emxArray_int32_T_InverseKinem_T
    **pEmxArray, int32_T numDimensions);
  void Inver_emxEnsureCapacity_int32_T(emxArray_int32_T_InverseKinem_T *emxArray,
    int32_T oldNumel);
  void InverseKinemati_emxFree_int32_T(emxArray_int32_T_InverseKinem_T
    **pEmxArray);
  void InverseKinemat_emxInit_uint32_T(emxArray_uint32_T_InverseKine_T
    **pEmxArray, int32_T numDimensions);
  void Inve_emxEnsureCapacity_uint32_T(emxArray_uint32_T_InverseKine_T *emxArray,
    int32_T oldNumel);
  void InverseKinemat_emxFree_uint32_T(emxArray_uint32_T_InverseKine_T
    **pEmxArray);
  void Inver_inverseKinematics_solve_n(b_inverseKinematics_InverseKi_T *obj,
    real_T initialGuess[4], real_T *solutionInfo_Iterations, real_T
    *solutionInfo_NumRandomRestarts, real_T *solutionInfo_PoseErrorNorm, real_T *
    solutionInfo_ExitFlag, char_T solutionInfo_Status_data[], int32_T
    solutionInfo_Status_size[2]);
  void Inve_inverseKinematics_stepImpl(b_inverseKinematics_InverseKi_T *obj,
    const real_T tform[16], const real_T weights[6], const real_T initialGuess[4],
    real_T QSol[4]);
  void emxFreeStruct_t_robotics_manip_(t_robotics_manip_internal_Rig_T *pStruct);
  void emxFreeStruct_l_robotics_manip_(l_robotics_manip_internal_Col_T *pStruct);
  void emxFreeMatrix_l_robotics_manip_(l_robotics_manip_internal_Col_T pMatrix
    [13]);
  void emxFreeStruct_c_rigidBodyJoint(c_rigidBodyJoint_InverseKinem_T *pStruct);
  void emxFreeMatrix_c_rigidBodyJoint(c_rigidBodyJoint_InverseKinem_T pMatrix[13]);
  void emxFreeMatrix_t_robotics_manip_(t_robotics_manip_internal_Rig_T pMatrix
    [12]);
  void emxFreeStruct_u_robotics_manip_(u_robotics_manip_internal_Rig_T *pStruct);
  void emxFreeStruct_f_robotics_manip_(f_robotics_manip_internal_IKE_T *pStruct);
  void emxFreeMatrix_c_rigidBodyJoint1(c_rigidBodyJoint_InverseKinem_T pMatrix
    [12]);
  void emxFreeMatrix_t_robotics_mani_n(t_robotics_manip_internal_Rig_T pMatrix[6]);
  void emxFreeMatrix_l_robotics_mani_n(l_robotics_manip_internal_Col_T pMatrix[7]);
  void emxFreeMatrix_c_rigidBodyJoint2(c_rigidBodyJoint_InverseKinem_T pMatrix[7]);
  void emxFreeStruct_v_robotics_manip_(v_robotics_manip_internal_Rig_T *pStruct);
  void emxFreeStruct_b_inverseKinemati(b_inverseKinematics_InverseKi_T *pStruct);
  void emxFreeStruct_robotics_slmanip_(robotics_slmanip_internal_blo_T *pStruct);

  /* Real-Time Model */
  RT_MODEL_InverseKinematics_T InverseKinematics_M;
};

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'InverseKinematics'
 * '<S1>'   : 'InverseKinematics/Inverse Kinematics'
 * '<S2>'   : 'InverseKinematics/Publisher'
 * '<S3>'   : 'InverseKinematics/Subscribe'
 * '<S4>'   : 'InverseKinematics/Inverse Kinematics/Inverse Kinematics'
 * '<S5>'   : 'InverseKinematics/Publisher/Blank Message'
 * '<S6>'   : 'InverseKinematics/Publisher/Publish'
 * '<S7>'   : 'InverseKinematics/Subscribe/Enabled Subsystem'
 */
#endif                                 /* RTW_HEADER_InverseKinematics_h_ */
