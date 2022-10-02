/*
 * InverseKinematics_data.cpp
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "InverseKinematics".
 *
 * Model version              : 4.20
 * Simulink Coder version : 9.7 (R2022a) 13-Nov-2021
 * C++ source code generated on : Wed Jul 27 23:36:07 2022
 *
 * Target selection: ert.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Intel->x86-64 (Linux 64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "InverseKinematics.h"

/* Block parameters (default storage) */
P_InverseKinematics_T InverseKinematics::InverseKinematics_P = {
  /* Computed Parameter: Constant_Value
   * Referenced by: '<S5>/Constant'
   */
  {
    0.0,                               /* x */
    0.0,                               /* y */
    0.0,                               /* z */
    0.0                                /* w */
  },

  /* Computed Parameter: Out1_Y0
   * Referenced by: '<S7>/Out1'
   */
  {
    0.0,                               /* x */
    0.0,                               /* y */
    0.0                                /* z */
  },

  /* Computed Parameter: Constant_Value_k
   * Referenced by: '<S3>/Constant'
   */
  {
    0.0,                               /* x */
    0.0,                               /* y */
    0.0                                /* z */
  },

  /* Expression: [0 0 0 1 1 1]
   * Referenced by: '<S1>/Constant1'
   */
  { 0.0, 0.0, 0.0, 1.0, 1.0, 1.0 },

  /* Expression: ([0 -10 0 90]*pi/180)'
   * Referenced by: '<S1>/Delay'
   */
  { 0.0, -0.17453292519943295, 0.0, 1.5707963267948966 }
};
