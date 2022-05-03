/*
 * Cinematique_ROS_data.cpp
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

/* Block parameters (default storage) */
P_Cinematique_ROS_T Cinematique_ROS_P = {
  /* Mask Parameter: CompareToConstant_const
   * Referenced by: '<S1>/Constant'
   */
  1U,

  /* Computed Parameter: Constant_Value
   * Referenced by: '<S9>/Constant'
   */
  {
    0.0,                               /* X */
    0.0,                               /* Y */
    0.0                                /* Z */
  },

  /* Computed Parameter: Constant_Value_l
   * Referenced by: '<S11>/Constant'
   */
  {
    0.0,                               /* X */
    0.0,                               /* Y */
    0.0                                /* Z */
  },

  /* Computed Parameter: Constant_Value_lm
   * Referenced by: '<S13>/Constant'
   */
  {
    0.0,                               /* X */
    0.0,                               /* Y */
    0.0                                /* Z */
  },

  /* Computed Parameter: Constant_Value_n
   * Referenced by: '<S15>/Constant'
   */
  {
    0.0,                               /* X */
    0.0,                               /* Y */
    0.0                                /* Z */
  },

  /* Computed Parameter: Out1_Y0
   * Referenced by: '<S17>/Out1'
   */
  {
    0.0,                               /* X */
    0.0,                               /* Y */
    0.0                                /* Z */
  },

  /* Computed Parameter: Constant_Value_f
   * Referenced by: '<S3>/Constant'
   */
  {
    0.0,                               /* X */
    0.0,                               /* Y */
    0.0                                /* Z */
  },

  /* Computed Parameter: Config_Y0
   * Referenced by: '<S4>/Config'
   */
  0.0,

  /* Expression: [0 0 0 1 1 1]
   * Referenced by: '<S4>/Constant1'
   */
  { 0.0, 0.0, 0.0, 1.0, 1.0, 1.0 },

  /* Expression: ([90 140 -50 70]*pi/180)'
   * Referenced by: '<S4>/Delay'
   */
  { 1.5707963267948966, 2.4434609527920612, -0.87266462599716477,
    1.2217304763960306 },

  /* Expression: 180/pi
   * Referenced by: '<Root>/Gain'
   */
  57.295779513082323,

  /* Expression: 20
   * Referenced by: '<Root>/Rate Limiter'
   */
  20.0,

  /* Expression: -20
   * Referenced by: '<Root>/Rate Limiter'
   */
  -20.0,

  /* Computed Parameter: Flag_Y0
   * Referenced by: '<S4>/Flag'
   */
  0U
};
