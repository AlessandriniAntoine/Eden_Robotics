/*
 * inverse_kinematics_data.cpp
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

#include "inverse_kinematics.h"

/* Block parameters (default storage) */
P_inverse_kinematics_T inverse_kinematics_P = {
  /* Computed Parameter: Constant_Value
   * Referenced by: '<S5>/Constant'
   */
  {
    0.0,                               /* X */
    0.0,                               /* Y */
    0.0,                               /* Z */
    0.0                                /* W */
  },

  /* Computed Parameter: Out1_Y0
   * Referenced by: '<S8>/Out1'
   */
  {
    0.0,                               /* X */
    0.0,                               /* Y */
    0.0                                /* Z */
  },

  /* Computed Parameter: Constant_Value_b
   * Referenced by: '<S7>/Constant'
   */
  {
    0.0,                               /* X */
    0.0,                               /* Y */
    0.0                                /* Z */
  },

  /* Computed Parameter: Config_Y0
   * Referenced by: '<S1>/Config'
   */
  0.0,

  /* Expression: [0 0 0 1 1 1]
   * Referenced by: '<S1>/Constant1'
   */
  { 0.0, 0.0, 0.0, 1.0, 1.0, 1.0 },

  /* Expression: ([0 -10 0 90]*pi/180)'
   * Referenced by: '<S1>/Delay'
   */
  { 0.0, -0.17453292519943295, 0.0, 1.5707963267948966 }
};
