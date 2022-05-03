/*
 * axes.h
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "axes".
 *
 * Model version              : 1.8
 * Simulink Coder version : 9.6 (R2021b) 14-May-2021
 * C++ source code generated on : Tue Feb  8 03:17:57 2022
 *
 * Target selection: ert.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Generic->Unspecified (assume 32-bit Generic)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef RTW_HEADER_axes_h_
#define RTW_HEADER_axes_h_
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stddef.h>
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "collisioncodegen_api.hpp"
#include "slros_initialize.h"
#include "axes_types.h"

/* Shared type includes */
#include "multiword_types.h"
#include "rtGetNaN.h"
#include "rt_nonfinite.h"
#include "rtGetInf.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

/* Block signals (default storage) */
struct B_axes_T {
  SL_Bus_axes_sensor_msgs_Joy In1;     /* '<S12>/In1' */
  SL_Bus_axes_sensor_msgs_Joy b_varargout_2;
  real_T T1[16];
  real_T T2[16];
  real_T b[16];
  real_T T1_m[16];
  d_cell_wrap_axes_T expl_temp;
  real_T R[9];
  real_T tempR[9];
  SL_Bus_axes_geometry_msgs_Quaternion b_varargout_2_c;
  real_T result_data[4];
  real_T TmpSignalConversionAtMATLAB[4];
  SL_Bus_axes_geometry_msgs_Point BusAssignment;/* '<S6>/Bus Assignment' */
  real_T v[3];
  int32_T e_data[4];
  char_T b_zeroDelimTopic[16];
  char_T b_zeroDelimTopic_k[14];
  char_T b_c[9];
  char_T b_b[9];
  char_T b_p[9];
  char_T b_cv[9];
  char_T b_f[9];
  char_T b_g[8];
  char_T b_g1[8];
  char_T b_m[8];
  char_T b_n[8];
  char_T b_l[8];
  char_T b_j[8];
  real_T X;
  real_T Y;
  real_T Z;
  real_T W;
  real_T n;
  real_T k;
  real_T theta;
  real_T tempR_tmp;
  real_T tempR_tmp_d;
  real_T k_tmp;
  real_T k_tmp_g;
  real_T c;
  SL_Bus_axes_std_msgs_Int16 In1_h;    /* '<S18>/In1' */
  void* defaultCollisionObj_GeometryInt;
  char_T b_zeroDelimTopic_l[6];
  char_T b_d[5];
  char_T b_zeroDelimTopic_d[5];
  char_T b_lx[4];
  int32_T d;
  int32_T e;
  int32_T ntilecols;
  int32_T b_o;
  int32_T b_kstr;
  int32_T loop_ub;
  int32_T i;
  int32_T b_kstr_b;
  int32_T loop_ub_n;
  int32_T i1;
  int32_T d_b;
  int32_T b_kstr_l;
  int32_T loop_ub_h;
  int32_T i2;
  int32_T b_kstr_bn;
  int32_T loop_ub_d;
  int32_T i3;
  int32_T b_jtilecol;
};

/* Block states (default storage) for system '<Root>' */
struct DW_axes_T {
  robotics_slmanip_internal_blo_T obj; /* '<S9>/MATLAB System' */
  ros_slroscpp_internal_block_S_T obj_g;/* '<S3>/SourceBlock' */
  ros_slroscpp_internal_block_S_T obj_k;/* '<S13>/SourceBlock' */
  ros_slroscpp_internal_block_S_T obj_n;/* '<S10>/SourceBlock' */
  ros_slroscpp_internal_block_P_T obj_p;/* '<S17>/SinkBlock' */
  int8_T If_ActiveSubsystem;           /* '<Root>/If' */
  int8_T If_ActiveSubsystem_d;         /* '<S1>/If' */
  boolean_T objisempty;                /* '<S3>/SourceBlock' */
  boolean_T objisempty_i;              /* '<S17>/SinkBlock' */
  boolean_T objisempty_b;              /* '<S13>/SourceBlock' */
  boolean_T objisempty_l;              /* '<S10>/SourceBlock' */
  boolean_T objisempty_f;              /* '<S9>/MATLAB System' */
};

/* Parameters for system: '<S4>/poser' */
struct P_poser_axes_T_ {
  real_T Constant_Value[3];            /* Expression: [0.06058 0.3063 0.1321]
                                        * Referenced by: '<S7>/Constant'
                                        */
};

/* Parameters (default storage) */
struct P_axes_T_ {
  SL_Bus_axes_sensor_msgs_Joy Out1_Y0; /* Computed Parameter: Out1_Y0
                                        * Referenced by: '<S12>/Out1'
                                        */
  SL_Bus_axes_sensor_msgs_Joy Constant_Value;/* Computed Parameter: Constant_Value
                                              * Referenced by: '<S10>/Constant'
                                              */
  SL_Bus_axes_geometry_msgs_Quaternion Out1_Y0_i;/* Computed Parameter: Out1_Y0_i
                                                  * Referenced by: '<S15>/Out1'
                                                  */
  SL_Bus_axes_geometry_msgs_Quaternion Constant_Value_f;/* Computed Parameter: Constant_Value_f
                                                         * Referenced by: '<S13>/Constant'
                                                         */
  SL_Bus_axes_geometry_msgs_Point Constant_Value_l;/* Computed Parameter: Constant_Value_l
                                                    * Referenced by: '<S16>/Constant'
                                                    */
  SL_Bus_axes_std_msgs_Int16 Out1_Y0_g;/* Computed Parameter: Out1_Y0_g
                                        * Referenced by: '<S18>/Out1'
                                        */
  SL_Bus_axes_std_msgs_Int16 Constant_Value_b;/* Computed Parameter: Constant_Value_b
                                               * Referenced by: '<S3>/Constant'
                                               */
  real_T Out1_Y0_l;                    /* Computed Parameter: Out1_Y0_l
                                        * Referenced by: '<S14>/Out1'
                                        */
  real_T Constant2_Value;              /* Expression: 0
                                        * Referenced by: '<S5>/Constant2'
                                        */
  real_T Constant_Value_j;             /* Expression: 0.1555
                                        * Referenced by: '<S5>/Constant'
                                        */
  real_T Constant1_Value;              /* Expression: 0.05513
                                        * Referenced by: '<S5>/Constant1'
                                        */
  real_T Constant3_Value;              /* Expression: 1
                                        * Referenced by: '<S5>/Constant3'
                                        */
  real_T Gain3_Gain;                   /* Expression: 1
                                        * Referenced by: '<S5>/Gain3'
                                        */
  real_T Gain4_Gain;                   /* Expression: 1
                                        * Referenced by: '<S5>/Gain4'
                                        */
  real32_T Gain1_Gain;                 /* Computed Parameter: Gain1_Gain
                                        * Referenced by: '<S5>/Gain1'
                                        */
  real32_T Gain_Gain;                  /* Computed Parameter: Gain_Gain
                                        * Referenced by: '<S5>/Gain'
                                        */
  real32_T Gain2_Gain;                 /* Computed Parameter: Gain2_Gain
                                        * Referenced by: '<S5>/Gain2'
                                        */
  P_poser_axes_T zero;                 /* '<S4>/zero' */
  P_poser_axes_T poser;                /* '<S4>/poser' */
};

/* Real-time Model Data Structure */
struct tag_RTM_axes_T {
  const char_T *errorStatus;
};

/* Block parameters (default storage) */
#ifdef __cplusplus

extern "C" {

#endif

  extern P_axes_T axes_P;

#ifdef __cplusplus

}
#endif

/* Block signals (default storage) */
#ifdef __cplusplus

extern "C" {

#endif

  extern struct B_axes_T axes_B;

#ifdef __cplusplus

}
#endif

/* Block states (default storage) */
extern struct DW_axes_T axes_DW;

#ifdef __cplusplus

extern "C" {

#endif

  /* Model entry point functions */
  extern void axes_initialize(void);
  extern void axes_step(void);
  extern void axes_terminate(void);

#ifdef __cplusplus

}
#endif

/* Real-time Model object */
#ifdef __cplusplus

extern "C" {

#endif

  extern RT_MODEL_axes_T *const axes_M;

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
 * '<Root>' : 'axes'
 * '<S1>'   : 'axes/1 2'
 * '<S2>'   : 'axes/Rien'
 * '<S3>'   : 'axes/Subscribe1'
 * '<S4>'   : 'axes/1 2/3 4 5'
 * '<S5>'   : 'axes/1 2/Deplacement'
 * '<S6>'   : 'axes/1 2/publisher_position_robot'
 * '<S7>'   : 'axes/1 2/3 4 5/poser'
 * '<S8>'   : 'axes/1 2/3 4 5/zero'
 * '<S9>'   : 'axes/1 2/Deplacement/Get Transform'
 * '<S10>'  : 'axes/1 2/Deplacement/Subscribe'
 * '<S11>'  : 'axes/1 2/Deplacement/angles_reference'
 * '<S12>'  : 'axes/1 2/Deplacement/Subscribe/Enabled Subsystem'
 * '<S13>'  : 'axes/1 2/Deplacement/angles_reference/Subscribe'
 * '<S14>'  : 'axes/1 2/Deplacement/angles_reference/Subsystem'
 * '<S15>'  : 'axes/1 2/Deplacement/angles_reference/Subscribe/Enabled Subsystem'
 * '<S16>'  : 'axes/1 2/publisher_position_robot/Blank Message1'
 * '<S17>'  : 'axes/1 2/publisher_position_robot/Publish'
 * '<S18>'  : 'axes/Subscribe1/Enabled Subsystem'
 */
#endif                                 /* RTW_HEADER_axes_h_ */
