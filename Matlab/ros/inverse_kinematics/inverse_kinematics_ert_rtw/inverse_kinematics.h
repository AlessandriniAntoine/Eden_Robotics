/*
 * inverse_kinematics.h
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "inverse_kinematics".
 *
 * Model version              : 4.7
 * Simulink Coder version : 9.7 (R2022a) 13-Nov-2021
 * C++ source code generated on : Sun Oct  2 23:17:33 2022
 *
 * Target selection: ert.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Generic->Unspecified (assume 32-bit Generic)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef RTW_HEADER_inverse_kinematics_h_
#define RTW_HEADER_inverse_kinematics_h_
#include <string.h>
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "coder_posix_time.h"
#include "collisioncodegen_api.hpp"
#include "slros_initialize.h"
#include "inverse_kinematics_types.h"

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
struct B_inverse_kinematics_T {
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
  SL_Bus_inverse_kinematics_geometry_msgs_Quaternion BusAssignment;/* '<S2>/Bus Assignment' */
  real_T MATLABSystem_o1[4];           /* '<S4>/MATLAB System' */
  real_T qvSolRaw[4];
  real_T c_xSol[4];
  real_T step_data[4];
  real_T y_d[4];
  real_T result_data[4];
  SL_Bus_inverse_kinematics_geometry_msgs_Point b_varargout_2;
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
  char_T b_zeroDelimTopic[12];
  char_T b_zeroDelimTopic_b[9];
  int8_T b_I_n[9];
  char_T b_b[9];
  int8_T b_I_l[9];
  char_T partial_match_data[9];
  char_T b_vstr[9];
  char_T b_h[9];
  char_T b_bn[9];
  char_T b_da[9];
  int8_T b_I_e[9];
  char_T b_bj[9];
  char_T b_j[8];
  char_T b_f[8];
  char_T vstr[8];
  char_T b_a[8];
  char_T b_ju[8];
  char_T b_jz[8];
  char_T b_o[8];
  real_T expl_temp;
  real_T expl_temp_i;
  real_T expl_temp_o;
  real_T expl_temp_n;
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
  real_T tempR_tmp_m;
  real_T tempR_tmp_c;
  real_T bid1_tmp;
  real_T i;
  real_T b_r;
  real_T x;
  real_T d_u;
  real_T params_ErrorChangeTolerance;
  real_T params_DampingBias;
  real_T bid_m;
  real_T b_m;
  real_T pid;
  real_T b_index;
  real_T obj_idx_0;
  real_T obj_idx_1;
  real_T obj_idx_2;
  real_T a_j;
  real_T q;
  real_T nrm;
  real_T rt;
  real_T ztest;
  real_T smm1;
  real_T emm1;
  real_T sqds;
  real_T b_h5;
  real_T a__3;
  real_T d_sn;
  real_T roe;
  real_T absa;
  real_T absb;
  real_T scale_c;
  real_T ads;
  real_T bds;
  real_T smax;
  real_T s_c;
  real_T c;
  real_T b_p;
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
  char_T b_af[5];
  char_T b_e[5];
  char_T c_vstr[5];
  char_T b_ax[5];
  int8_T b_ipiv[4];
  int32_T b_jcol;
  int32_T loop_ub;
  int32_T iacol_tmp;
  int32_T b_j_l;
  int32_T nm1d2;
  int32_T c_o;
  int32_T b_k;
  int32_T loop_ub_o;
  int32_T indicesUpperBoundViolation;
  int32_T tmp_size;
  int32_T indicesUpperBoundViolation_size;
  int32_T c_f;
  int32_T b_i;
  int32_T ix;
  int32_T nx;
  int32_T unnamed_idx_1;
  int32_T b_i_i;
  int32_T m;
  int32_T coffset;
  int32_T boffset;
  int32_T aoffset;
  int32_T kend;
  int32_T b_k_f;
  int32_T J;
  int32_T step_size;
  int32_T jointSign;
  int32_T c_g;
  int32_T f;
  int32_T g;
  int32_T i_c;
  int32_T loop_ub_o3;
  int32_T Jac;
  int32_T result_data_tmp;
  int32_T minPathLength;
  int32_T b_i_l;
  int32_T e_mv;
  int32_T h;
  int32_T j;
  int32_T loop_ub_m;
  int32_T i_cn;
  int32_T loop_ub_f;
  int32_T loop_ub_tmp;
  int32_T newNumel;
  int32_T i_p;
  int32_T b_k_e;
  int32_T i_o;
  int32_T d_tmp;
  int32_T b_j_h;
  int32_T b_kk;
  int32_T i_l;
  int32_T ret;
  int32_T b_kstr;
  int32_T loop_ub_h;
  int32_T iobj_3;
  int32_T obj_tmp;
  int32_T b_kstr_m;
  int32_T loop_ub_mc;
  int32_T i1;
  int32_T nmatched;
  int32_T minnanb;
  int32_T loop_ub_h3;
  int32_T partial_match_size_idx_1;
  int32_T b_kstr_c;
  int32_T loop_ub_k;
  int32_T i2;
  int32_T newNumel_p;
  int32_T i_px;
  int32_T b_k_p;
  int32_T i3;
  int32_T y_tmp;
  int32_T T_tmp;
  int32_T T_tmp_a;
  int32_T q_j;
  int32_T qp1;
  int32_T qq;
  int32_T qjj;
  int32_T m_e;
  int32_T b_ol;
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
  int32_T b_kstr_h;
  int32_T loop_ub_e;
  int32_T i5;
  int32_T b_kstr_ch;
  int32_T loop_ub_a;
  int32_T i6;
  int32_T d_d;
  int32_T b_i_a;
  int32_T newNumel_pb;
  int32_T i_m;
  int32_T b_i_o;
  int32_T loop_ub_n;
  int32_T i7;
  int32_T i8;
  int32_T i_lu;
  int32_T n;
  int32_T b_j_p;
  int32_T b_i_p;
  int32_T b_k_ft;
  int32_T coffset_tmp;
  int32_T i_i;
  int32_T b_kstr_o;
  int32_T i9;
  uint32_T mti;
  uint32_T y_k;
  boolean_T ubOK[4];
  boolean_T lbOK[4];
  boolean_T ubOK_a[4];
  boolean_T x_i[4];
  c_robotics_core_internal_NLPS_T exitFlag;
  c_robotics_core_internal_NLPS_T exitFlagPrev;
  int8_T y_tmp_i[3];
  int8_T y_tmp_o;
  boolean_T b_varargout_1;
  boolean_T y_m;
  boolean_T y_c;
  boolean_T flag;
  boolean_T nextBodyIsParent;
  boolean_T b_bool;
  boolean_T params_UseErrorDamping;
  boolean_T b_bool_f;
  boolean_T b_bool_h;
  boolean_T matched;
  boolean_T b_bool_m;
  boolean_T b_bool_a;
  boolean_T e_k;
  boolean_T x_p;
  boolean_T rEQ0;
  boolean_T apply_transform;
  boolean_T b_bool_b;
  boolean_T b_bool_c;
  boolean_T d_n;
};

/* Block states (default storage) for system '<Root>' */
struct DW_inverse_kinematics_T {
  robotics_slmanip_internal_blo_T obj; /* '<S4>/MATLAB System' */
  ros_slroscpp_internal_block_P_T obj_j;/* '<S6>/SinkBlock' */
  ros_slroscpp_internal_block_S_T obj_a;/* '<S7>/SourceBlock' */
  real_T Delay_DSTATE[4];              /* '<S1>/Delay' */
  real_T freq;                         /* '<S4>/MATLAB System' */
  uint32_T state;                      /* '<S4>/MATLAB System' */
  uint32_T state_p[2];                 /* '<S4>/MATLAB System' */
  uint32_T state_c[625];               /* '<S4>/MATLAB System' */
  uint32_T method;                     /* '<S4>/MATLAB System' */
  uint32_T method_a;                   /* '<S4>/MATLAB System' */
  uint32_T state_pz[2];                /* '<S4>/MATLAB System' */
  robotics_slcore_internal_bloc_T obj_jv;
                              /* '<S1>/Coordinate Transformation Conversion1' */
  boolean_T objisempty;                /* '<S7>/SourceBlock' */
  boolean_T objisempty_j;              /* '<S6>/SinkBlock' */
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
struct P_inverse_kinematics_T_ {
  SL_Bus_inverse_kinematics_geometry_msgs_Quaternion Constant_Value;/* Computed Parameter: Constant_Value
                                                                     * Referenced by: '<S5>/Constant'
                                                                     */
  SL_Bus_inverse_kinematics_geometry_msgs_Point Out1_Y0;/* Computed Parameter: Out1_Y0
                                                         * Referenced by: '<S8>/Out1'
                                                         */
  SL_Bus_inverse_kinematics_geometry_msgs_Point Constant_Value_b;/* Computed Parameter: Constant_Value_b
                                                                  * Referenced by: '<S7>/Constant'
                                                                  */
  real_T Config_Y0;                    /* Computed Parameter: Config_Y0
                                        * Referenced by: '<S1>/Config'
                                        */
  real_T Constant1_Value[6];           /* Expression: [0 0 0 1 1 1]
                                        * Referenced by: '<S1>/Constant1'
                                        */
  real_T Delay_InitialCondition[4];    /* Expression: ([0 -10 0 90]*pi/180)'
                                        * Referenced by: '<S1>/Delay'
                                        */
};

/* Real-time Model Data Structure */
struct tag_RTM_inverse_kinematics_T {
  const char_T *errorStatus;
};

/* Block parameters (default storage) */
#ifdef __cplusplus

extern "C" {

#endif

  extern P_inverse_kinematics_T inverse_kinematics_P;

#ifdef __cplusplus

}
#endif

/* Block signals (default storage) */
#ifdef __cplusplus

extern "C" {

#endif

  extern struct B_inverse_kinematics_T inverse_kinematics_B;

#ifdef __cplusplus

}
#endif

/* Block states (default storage) */
extern struct DW_inverse_kinematics_T inverse_kinematics_DW;

#ifdef __cplusplus

extern "C" {

#endif

  /* Model entry point functions */
  extern void inverse_kinematics_initialize(void);
  extern void inverse_kinematics_step(void);
  extern void inverse_kinematics_terminate(void);

#ifdef __cplusplus

}
#endif

/* Real-time Model object */
#ifdef __cplusplus

extern "C" {

#endif

  extern RT_MODEL_inverse_kinematics_T *const inverse_kinematics_M;

#ifdef __cplusplus

}
#endif

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
 * '<Root>' : 'inverse_kinematics'
 * '<S1>'   : 'inverse_kinematics/Inverse Kinematics'
 * '<S2>'   : 'inverse_kinematics/Publisher'
 * '<S3>'   : 'inverse_kinematics/Subscriber'
 * '<S4>'   : 'inverse_kinematics/Inverse Kinematics/Inverse Kinematics'
 * '<S5>'   : 'inverse_kinematics/Publisher/Blank Message1'
 * '<S6>'   : 'inverse_kinematics/Publisher/Publish'
 * '<S7>'   : 'inverse_kinematics/Subscriber/Subscribe'
 * '<S8>'   : 'inverse_kinematics/Subscriber/Subscribe/Enabled Subsystem'
 */
#endif                                 /* RTW_HEADER_inverse_kinematics_h_ */
