/*
 * Cinematique_ROS.h
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

#ifndef RTW_HEADER_Cinematique_ROS_h_
#define RTW_HEADER_Cinematique_ROS_h_
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stddef.h>
#include <string.h>
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "slros_initialize.h"
#include "collisioncodegen_api.hpp"
#include "Cinematique_ROS_types.h"

/* Shared type includes */
#include "multiword_types.h"
#include "rtGetNaN.h"
#include "rt_nonfinite.h"
#include "rtGetInf.h"
#include "rt_defines.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

#ifndef rtmGetT
#define rtmGetT(rtm)                   (rtmGetTPtr((rtm))[0])
#endif

#ifndef rtmGetTPtr
#define rtmGetTPtr(rtm)                ((rtm)->Timing.t)
#endif

/* Block signals (default storage) */
typedef struct {
  uint32_T uv[625];
  uint32_T uv1[625];
  real_T xi[257];
  real_T weightMatrix[36];
  real_T unusedU0[36];
  real_T Tj[36];
  real_T a[36];
  real_T out[16];
  real_T H[16];
  real_T P[16];
  real_T V[16];
  real_T V_m[16];
  real_T Td[16];
  real_T T_data[16];
  real_T T1[16];
  real_T Tc2p[16];
  real_T Tj_c[16];
  real_T T1j[16];
  real_T b[16];
  real_T Tj_k[16];
  real_T c_A_data[16];
  real_T obj[16];
  real_T poslim_data[12];
  real_T poslim_data_c[12];
  real_T poslim_data_b[12];
  real_T poslim_data_p[12];
  real_T y[9];
  real_T V_c[9];
  real_T b_U[9];
  real_T T[9];
  real_T R[9];
  real_T tempR[9];
  real_T A[9];
  real_T A_f[9];
  real_T e[6];
  real_T y_g[6];
  real_T y_g1[6];
  real_T unusedExpr[5];
  real_T unusedExpr_m[5];
  int8_T msubspace_data[36];
  int8_T msubspace_data_n[36];
  int8_T msubspace_data_p[36];
  int8_T msubspace_data_l[36];
  real_T RateLimiter[4];               /* '<Root>/Rate Limiter' */
  real_T MATLABSystem_o1[4];           /* '<S18>/MATLAB System' */
  real_T qvSolRaw[4];
  real_T c_xSol[4];
  real_T x[4];
  real_T Hg[4];
  real_T sNew[4];
  real_T result_data[4];
  SL_Bus_Cinematique_ROS_geometry_msgs_Point BusAssignment;/* '<S5>/Bus Assignment' */
  ikblock_info_bus MATLABSystem_o2;    /* '<S18>/MATLAB System' */
  real_T Gain[4];                      /* '<Root>/Gain' */
  real_T v[3];
  real_T vspecial_data[3];
  real_T v_j[3];
  real_T s[3];
  real_T e_d[3];
  real_T work[3];
  int8_T b_I[16];
  int32_T indicesUpperBoundViolation_data[4];
  int32_T tmp_data[4];
  int32_T g_data[4];
  creal_T v_g;
  creal_T u;
  creal_T u_l;
  creal_T dc;
  real_T ub[2];
  char_T b_varargout_2_Status_data[14];
  char_T b_zeroDelimTopic[10];
  char_T b_zeroDelimTopic_d[9];
  char_T b_d[9];
  char_T b_l[9];
  char_T b_o[9];
  char_T b_b[9];
  char_T partial_match_data[9];
  char_T b_vstr[9];
  char_T b_n[9];
  char_T b_bs[8];
  char_T b_zeroDelimTopic_l[8];
  char_T b_h[8];
  char_T b_bn[8];
  char_T b_da[8];
  char_T b_e[8];
  char_T vstr[8];
  char_T b_bj[8];
  real_T fallValLimit;
  real_T n;
  real_T m;
  real_T pnum;
  real_T t;
  real_T pnum_j;
  real_T w;
  real_T expl_temp;
  real_T d;
  real_T d1;
  real_T d2;
  real_T bid;
  real_T numPositions;
  real_T ndbl;
  real_T apnd;
  real_T cdiff;
  real_T u0;
  real_T u1;
  real_T d3;
  real_T d4;
  real_T d5;
  real_T d6;
  real_T tol;
  real_T err;
  real_T iter;
  real_T cost;
  real_T lambda;
  real_T b_gamma;
  real_T beta;
  real_T sigma;
  real_T costNew;
  real_T m_f;
  real_T theta;
  real_T A_a;
  real_T sigma_j;
  real_T s_j;
  real_T q;
  real_T bid1;
  real_T bid2;
  real_T sth;
  real_T tempR_tmp;
  real_T tempR_tmp_o;
  real_T tempR_tmp_n;
  real_T tempR_tmp_i;
  real_T numCommonAncestors;
  real_T u1_o;
  real_T i;
  real_T bid_n;
  real_T b_m;
  real_T c;
  real_T x_c;
  real_T d_u;
  real_T nrm;
  real_T rt;
  real_T ztest;
  real_T smm1;
  real_T emm1;
  real_T sqds;
  real_T b_md;
  real_T unusedU2;
  real_T d_sn;
  real_T roe;
  real_T absa;
  real_T absb;
  real_T scale;
  real_T ads;
  real_T bds;
  real_T scale_m;
  real_T absxk;
  real_T t_j;
  real_T tol_h;
  real_T scale_c;
  real_T absxk_c;
  real_T t_p;
  real_T smax;
  real_T scale_p;
  real_T absxk_a;
  real_T t_e;
  real_T ssq;
  real_T c_a;
  real_T b_a;
  real_T smax_i;
  real_T s_l;
  real_T s_o;
  real_T pid;
  real_T b_index;
  real_T c_o;
  real_T obj_idx_0;
  real_T obj_idx_1;
  real_T obj_idx_2;
  uint32_T b_u[2];
  uint32_T u32[2];
  uint32_T b_u_i[2];
  int32_T b_varargout_2_Status_size[2];
  int32_T T_size[2];
  char_T b_f[7];
  char_T b_zeroDelimTopic_i[7];
  int8_T iv[6];
  int8_T iv1[6];
  int8_T iv2[6];
  int8_T iv3[6];
  char_T b_ff[5];
  char_T b_g[5];
  char_T b_c[5];
  char_T c_vstr[5];
  char_T b_o3[5];
  int32_T b_jcol;
  int32_T d_m;
  int32_T j;
  int32_T p;
  int32_T m_m;
  int32_T loop_ub;
  int32_T iacol_tmp;
  int32_T loop_ub_c;
  int32_T b_j;
  int32_T nm1d2;
  int32_T c_f;
  int32_T b_k;
  int32_T loop_ub_p;
  int32_T tmp_size;
  int32_T indicesUpperBoundViolation_da_e;
  int32_T indicesUpperBoundViolation_size;
  int32_T c_h;
  int32_T b_i;
  int32_T ix;
  int32_T nx;
  int32_T unnamed_idx_1;
  int32_T idxl;
  int32_T j_l;
  int32_T nx_h;
  int32_T m_me;
  int32_T inner;
  int32_T aoffset;
  int32_T n_m;
  int32_T boffset;
  int32_T b_k_h;
  int32_T loop_ub_cs;
  int32_T y_tmp;
  int32_T jointSign;
  int32_T c_k;
  int32_T b_i_p;
  int32_T f;
  int32_T g;
  int32_T b_kstr;
  int32_T loop_ub_px;
  int32_T i_p;
  int32_T minPathLength;
  int32_T b_i_a;
  int32_T g_j;
  int32_T h;
  int32_T j_e;
  int32_T loop_ub_o;
  int32_T i1;
  int32_T newNumel;
  int32_T i_b;
  int32_T b_mti;
  int32_T d_a;
  int32_T b_kstr_g;
  int32_T loop_ub_e;
  int32_T b_kstr_f;
  int32_T loop_ub_h;
  int32_T b_k_e;
  int32_T b_mti_c;
  int32_T b_j_a;
  int32_T b_kk;
  int32_T d_d;
  int32_T b_k_a;
  int32_T d_p;
  int32_T b_k_m;
  int32_T i_o;
  int32_T b_mti_n;
  int32_T k;
  int32_T qq;
  int32_T qjj;
  int32_T m_l;
  int32_T kase;
  int32_T c_q;
  int32_T d_k;
  int32_T b_k_p;
  int32_T loop_ub_pt;
  int32_T i_f;
  int32_T rankA;
  int32_T minmn;
  int32_T maxmn;
  int32_T nb;
  int32_T m_i;
  int32_T nb_o;
  int32_T mn;
  int32_T b_i_k;
  int32_T ip;
  int32_T ma;
  int32_T na;
  int32_T minmana;
  int32_T minmn_i;
  int32_T ii;
  int32_T mmi;
  int32_T pvt;
  int32_T itemp;
  int32_T kend;
  int32_T ix_o;
  int32_T iy;
  int32_T knt;
  int32_T lastc;
  int32_T d_m4;
  int32_T b_cu;
  int32_T u0_f;
  int32_T kend_h;
  int32_T k_m;
  int32_T b_info;
  int32_T jm1;
  int32_T idxAjj;
  int32_T b_j_ad;
  int32_T d_kb;
  int32_T ia;
  int32_T ix_p;
  int32_T iy_b;
  int32_T k_c;
  int32_T c_A_size_idx_0;
  int32_T c_A_size_idx_1;
  int32_T b_i_n;
  int32_T loop_ub_i;
  int32_T i2;
  int32_T newNumel_m;
  int32_T i_j;
  int32_T b_kstr_e;
  int32_T loop_ub_m;
  int32_T i_m;
  int32_T n_j;
  int32_T c_fo;
  int32_T info;
  int32_T n_a;
  int32_T yk;
  int32_T mmj;
  int32_T b_gl;
  int32_T c_n;
  int32_T c_d;
  int32_T ix_n;
  int32_T iy_c;
  int32_T n_f;
  int32_T yk_p;
  int32_T k_p;
  int32_T jA;
  int32_T jy;
  int32_T c_tmp;
  int32_T newNumel_n;
  int32_T i_k;
  int32_T n_n;
  int32_T b_j_o;
  int32_T b_i_g;
  int32_T b_k_c;
  int32_T coffset_tmp;
  int32_T b_kstr_c;
  int32_T loop_ub_m1;
  int32_T nmatched;
  int32_T minnanb;
  int32_T loop_ub_j;
  int32_T partial_match_size_idx_1;
  uint32_T r;
  uint32_T r_k;
  uint32_T mti;
  uint32_T y_m;
  uint32_T r_p;
  boolean_T ubOK[4];
  boolean_T lbOK[4];
  boolean_T ubOK_l[4];
  c_robotics_core_internal_NLPS_T exitFlag;
  c_robotics_core_internal_NLPS_T exitFlagPrev;
  int8_T y_tmp_d[3];
  int8_T y_tmp_g;
  boolean_T Compare;                   /* '<S1>/Compare' */
  boolean_T b_varargout_1;
  boolean_T y_c;
  boolean_T y_cx;
  boolean_T flag;
  boolean_T e_i;
  boolean_T rEQ0;
  boolean_T p_d;
  boolean_T nextBodyIsParent;
  boolean_T b_bool;
  boolean_T b_bool_g;
  boolean_T b_bool_l;
  boolean_T apply_transform;
  boolean_T b_bool_f;
  boolean_T b_bool_d;
  boolean_T matched;
  boolean_T b_bool_j;
} B_Cinematique_ROS_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  robotics_slmanip_internal_blo_T obj; /* '<S18>/MATLAB System' */
  ros_slroscpp_internal_block_P_T obj_o;/* '<S16>/SinkBlock' */
  ros_slroscpp_internal_block_P_T obj_k;/* '<S14>/SinkBlock' */
  ros_slroscpp_internal_block_P_T obj_n;/* '<S12>/SinkBlock' */
  ros_slroscpp_internal_block_P_T obj_e;/* '<S10>/SinkBlock' */
  ros_slroscpp_internal_block_S_T obj_ez;/* '<S3>/SourceBlock' */
  real_T Delay_DSTATE[4];              /* '<S4>/Delay' */
  real_T PrevY[4];                     /* '<Root>/Rate Limiter' */
  real_T LastMajorTime;                /* '<Root>/Rate Limiter' */
  uint32_T state;                      /* '<S18>/MATLAB System' */
  uint32_T state_f[2];                 /* '<S18>/MATLAB System' */
  uint32_T state_l[625];               /* '<S18>/MATLAB System' */
  uint32_T method;                     /* '<S18>/MATLAB System' */
  uint32_T method_o;                   /* '<S18>/MATLAB System' */
  uint32_T state_o[2];                 /* '<S18>/MATLAB System' */
  robotics_slcore_internal_bloc_T obj_g;
                              /* '<S4>/Coordinate Transformation Conversion1' */
  boolean_T objisempty;                /* '<S18>/MATLAB System' */
  boolean_T state_not_empty;           /* '<S18>/MATLAB System' */
  boolean_T state_not_empty_j;         /* '<S18>/MATLAB System' */
  boolean_T state_not_empty_m;         /* '<S18>/MATLAB System' */
  boolean_T method_not_empty;          /* '<S18>/MATLAB System' */
  boolean_T method_not_empty_i;        /* '<S18>/MATLAB System' */
  boolean_T state_not_empty_ml;        /* '<S18>/MATLAB System' */
  boolean_T objisempty_k;     /* '<S4>/Coordinate Transformation Conversion1' */
  boolean_T objisempty_f;              /* '<S3>/SourceBlock' */
  boolean_T objisempty_h;              /* '<S16>/SinkBlock' */
  boolean_T objisempty_m;              /* '<S14>/SinkBlock' */
  boolean_T objisempty_fv;             /* '<S12>/SinkBlock' */
  boolean_T objisempty_g;              /* '<S10>/SinkBlock' */
  boolean_T EnabledSubsystem_MODE;     /* '<Root>/Enabled Subsystem' */
} DW_Cinematique_ROS_T;

/* Parameters (default storage) */
struct P_Cinematique_ROS_T_ {
  uint16_T CompareToConstant_const;   /* Mask Parameter: CompareToConstant_const
                                       * Referenced by: '<S1>/Constant'
                                       */
  SL_Bus_Cinematique_ROS_geometry_msgs_Point Constant_Value;/* Computed Parameter: Constant_Value
                                                             * Referenced by: '<S9>/Constant'
                                                             */
  SL_Bus_Cinematique_ROS_geometry_msgs_Point Constant_Value_l;/* Computed Parameter: Constant_Value_l
                                                               * Referenced by: '<S11>/Constant'
                                                               */
  SL_Bus_Cinematique_ROS_geometry_msgs_Point Constant_Value_lm;/* Computed Parameter: Constant_Value_lm
                                                                * Referenced by: '<S13>/Constant'
                                                                */
  SL_Bus_Cinematique_ROS_geometry_msgs_Point Constant_Value_n;/* Computed Parameter: Constant_Value_n
                                                               * Referenced by: '<S15>/Constant'
                                                               */
  SL_Bus_Cinematique_ROS_geometry_msgs_Point Out1_Y0;/* Computed Parameter: Out1_Y0
                                                      * Referenced by: '<S17>/Out1'
                                                      */
  SL_Bus_Cinematique_ROS_geometry_msgs_Point Constant_Value_f;/* Computed Parameter: Constant_Value_f
                                                               * Referenced by: '<S3>/Constant'
                                                               */
  real_T Config_Y0;                    /* Computed Parameter: Config_Y0
                                        * Referenced by: '<S4>/Config'
                                        */
  real_T Constant1_Value[6];           /* Expression: [0 0 0 1 1 1]
                                        * Referenced by: '<S4>/Constant1'
                                        */
  real_T Delay_InitialCondition[4];    /* Expression: ([90 140 -50 70]*pi/180)'
                                        * Referenced by: '<S4>/Delay'
                                        */
  real_T Gain_Gain;                    /* Expression: 180/pi
                                        * Referenced by: '<Root>/Gain'
                                        */
  real_T RateLimiter_RisingLim;        /* Expression: 20
                                        * Referenced by: '<Root>/Rate Limiter'
                                        */
  real_T RateLimiter_FallingLim;       /* Expression: -20
                                        * Referenced by: '<Root>/Rate Limiter'
                                        */
  uint16_T Flag_Y0;                    /* Computed Parameter: Flag_Y0
                                        * Referenced by: '<S4>/Flag'
                                        */
};

/* Real-time Model Data Structure */
struct tag_RTM_Cinematique_ROS_T {
  const char_T *errorStatus;
  RTWSolverInfo solverInfo;

  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    uint32_T clockTick0;
    uint32_T clockTickH0;
    time_T stepSize0;
    uint32_T clockTick1;
    uint32_T clockTickH1;
    SimTimeStep simTimeStep;
    time_T *t;
    time_T tArray[2];
  } Timing;
};

/* Block parameters (default storage) */
#ifdef __cplusplus

extern "C" {

#endif

  extern P_Cinematique_ROS_T Cinematique_ROS_P;

#ifdef __cplusplus

}
#endif

/* Block signals (default storage) */
#ifdef __cplusplus

extern "C" {

#endif

  extern B_Cinematique_ROS_T Cinematique_ROS_B;

#ifdef __cplusplus

}
#endif

/* Block states (default storage) */
extern DW_Cinematique_ROS_T Cinematique_ROS_DW;

#ifdef __cplusplus

extern "C" {

#endif

  /* Model entry point functions */
  extern void Cinematique_ROS_initialize(void);
  extern void Cinematique_ROS_step(void);
  extern void Cinematique_ROS_terminate(void);

#ifdef __cplusplus

}
#endif

/* Real-time Model object */
#ifdef __cplusplus

extern "C" {

#endif

  extern RT_MODEL_Cinematique_ROS_T *const Cinematique_ROS_M;

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
 * '<Root>' : 'Cinematique_ROS'
 * '<S1>'   : 'Cinematique_ROS/Compare To Constant'
 * '<S2>'   : 'Cinematique_ROS/Enabled Subsystem'
 * '<S3>'   : 'Cinematique_ROS/Subscribe'
 * '<S4>'   : 'Cinematique_ROS/Subsystem1'
 * '<S5>'   : 'Cinematique_ROS/Enabled Subsystem/Bassin'
 * '<S6>'   : 'Cinematique_ROS/Enabled Subsystem/Coude'
 * '<S7>'   : 'Cinematique_ROS/Enabled Subsystem/Epaule'
 * '<S8>'   : 'Cinematique_ROS/Enabled Subsystem/Poignet'
 * '<S9>'   : 'Cinematique_ROS/Enabled Subsystem/Bassin/Blank Message'
 * '<S10>'  : 'Cinematique_ROS/Enabled Subsystem/Bassin/Publish'
 * '<S11>'  : 'Cinematique_ROS/Enabled Subsystem/Coude/Blank Message'
 * '<S12>'  : 'Cinematique_ROS/Enabled Subsystem/Coude/Publish'
 * '<S13>'  : 'Cinematique_ROS/Enabled Subsystem/Epaule/Blank Message'
 * '<S14>'  : 'Cinematique_ROS/Enabled Subsystem/Epaule/Publish'
 * '<S15>'  : 'Cinematique_ROS/Enabled Subsystem/Poignet/Blank Message'
 * '<S16>'  : 'Cinematique_ROS/Enabled Subsystem/Poignet/Publish'
 * '<S17>'  : 'Cinematique_ROS/Subscribe/Enabled Subsystem'
 * '<S18>'  : 'Cinematique_ROS/Subsystem1/Inverse Kinematics'
 */
#endif                                 /* RTW_HEADER_Cinematique_ROS_h_ */
