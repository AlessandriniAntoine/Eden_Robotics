/*
 * Cinematique_ROS_types.h
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

#ifndef RTW_HEADER_Cinematique_ROS_types_h_
#define RTW_HEADER_Cinematique_ROS_types_h_
#include "rtwtypes.h"
#include "multiword_types.h"

/* Model Code Variants */
#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_Cinematique_ROS_geometry_msgs_Point_
#define DEFINED_TYPEDEF_FOR_SL_Bus_Cinematique_ROS_geometry_msgs_Point_

typedef struct {
  real_T X;
  real_T Y;
  real_T Z;
} SL_Bus_Cinematique_ROS_geometry_msgs_Point;

#endif

#ifndef DEFINED_TYPEDEF_FOR_ikblock_info_bus_
#define DEFINED_TYPEDEF_FOR_ikblock_info_bus_

typedef struct {
  real_T Iterations;
  real_T PoseErrorNorm;
  uint16_T ExitFlag;
  uint8_T Status;
} ikblock_info_bus;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_fGIjNeTd7SKHdd1pi8YJrG_
#define DEFINED_TYPEDEF_FOR_struct_fGIjNeTd7SKHdd1pi8YJrG_

typedef struct {
  real_T NameLength;
  uint8_T Name[5];
  real_T ParentIndex;
  real_T NumChildren;
  real_T ChildrenIndices[5];
  real_T Mass;
  real_T CenterOfMass[3];
  real_T Inertia[9];
  real_T SpatialInertia[36];
} struct_fGIjNeTd7SKHdd1pi8YJrG;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_Rse6wCtwGVc5lcXFy76SOB_
#define DEFINED_TYPEDEF_FOR_struct_Rse6wCtwGVc5lcXFy76SOB_

typedef struct {
  uint8_T Type;
  real_T NameLength;
  uint8_T Name[6];
  real_T VelocityNumber;
  real_T PositionNumber;
  real_T MotionSubspace[36];
  real_T JointAxis[3];
  real_T PositionLimits[14];
  real_T HomePosition[7];
  real_T JointToParentTransform[16];
  real_T ChildToJointTransform[16];
} struct_Rse6wCtwGVc5lcXFy76SOB;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_PNeGoHTm5CEERDPrIHt9vH_
#define DEFINED_TYPEDEF_FOR_struct_PNeGoHTm5CEERDPrIHt9vH_

typedef struct {
  real_T NumBodies;
  real_T Gravity[3];
  real_T NumNonFixedBodies;
  real_T PositionNumber;
  real_T VelocityNumber;
  real_T PositionDoFMap[10];
  real_T VelocityDoFMap[10];
  real_T MaxNameLength;
  real_T MaxJointPositionNumber;
  uint8_T DataFormat;
  real_T JointPositionLimits[8];
  struct_fGIjNeTd7SKHdd1pi8YJrG Bodies[6];
  struct_Rse6wCtwGVc5lcXFy76SOB Joints[6];
} struct_PNeGoHTm5CEERDPrIHt9vH;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_MH1icwhz7mkMseeLmnjXIE_
#define DEFINED_TYPEDEF_FOR_struct_MH1icwhz7mkMseeLmnjXIE_

typedef struct {
  boolean_T EnforceJointLimits;
  boolean_T AllowRandomRestart;
  real_T MaxIterations;
  real_T MaxTime;
  real_T GradientTolerance;
  real_T SolutionTolerance;
  real_T StepTolerance;
} struct_MH1icwhz7mkMseeLmnjXIE;

#endif

#ifndef struct_tag_D6QYIVJDjoKN0blLVEpT8F
#define struct_tag_D6QYIVJDjoKN0blLVEpT8F

struct tag_D6QYIVJDjoKN0blLVEpT8F
{
  boolean_T matlabCodegenIsDeleted;
  int32_T isInitialized;
  boolean_T isSetupComplete;
};

#endif                                 /*struct_tag_D6QYIVJDjoKN0blLVEpT8F*/

#ifndef typedef_ros_slroscpp_internal_block_P_T
#define typedef_ros_slroscpp_internal_block_P_T

typedef tag_D6QYIVJDjoKN0blLVEpT8F ros_slroscpp_internal_block_P_T;

#endif                               /*typedef_ros_slroscpp_internal_block_P_T*/

#ifndef struct_tag_eFCXAaC7vLdwjksE0MwgOD
#define struct_tag_eFCXAaC7vLdwjksE0MwgOD

struct tag_eFCXAaC7vLdwjksE0MwgOD
{
  boolean_T matlabCodegenIsDeleted;
  int32_T isInitialized;
  boolean_T isSetupComplete;
};

#endif                                 /*struct_tag_eFCXAaC7vLdwjksE0MwgOD*/

#ifndef typedef_ros_slroscpp_internal_block_S_T
#define typedef_ros_slroscpp_internal_block_S_T

typedef tag_eFCXAaC7vLdwjksE0MwgOD ros_slroscpp_internal_block_S_T;

#endif                               /*typedef_ros_slroscpp_internal_block_S_T*/

#ifndef struct_tag_vxHWSOYrO9xtYchIOe7EKG
#define struct_tag_vxHWSOYrO9xtYchIOe7EKG

struct tag_vxHWSOYrO9xtYchIOe7EKG
{
  int32_T isInitialized;
};

#endif                                 /*struct_tag_vxHWSOYrO9xtYchIOe7EKG*/

#ifndef typedef_robotics_slcore_internal_bloc_T
#define typedef_robotics_slcore_internal_bloc_T

typedef tag_vxHWSOYrO9xtYchIOe7EKG robotics_slcore_internal_bloc_T;

#endif                               /*typedef_robotics_slcore_internal_bloc_T*/

#ifndef struct_tag_sdAmwXbnJnEmimT0NaJRtAD
#define struct_tag_sdAmwXbnJnEmimT0NaJRtAD

struct tag_sdAmwXbnJnEmimT0NaJRtAD
{
  real_T tv_sec;
  real_T tv_nsec;
};

#endif                                 /*struct_tag_sdAmwXbnJnEmimT0NaJRtAD*/

#ifndef typedef_sdAmwXbnJnEmimT0NaJRtAD_Cinem_T
#define typedef_sdAmwXbnJnEmimT0NaJRtAD_Cinem_T

typedef tag_sdAmwXbnJnEmimT0NaJRtAD sdAmwXbnJnEmimT0NaJRtAD_Cinem_T;

#endif                               /*typedef_sdAmwXbnJnEmimT0NaJRtAD_Cinem_T*/

/* Custom Type definition for MATLABSystem: '<S18>/MATLAB System' */
#include "coder_posix_time.h"
#include "coder_posix_time.h"
#ifndef struct_tag_qN5Oiz2OtHsE7f06wrZAJC
#define struct_tag_qN5Oiz2OtHsE7f06wrZAJC

struct tag_qN5Oiz2OtHsE7f06wrZAJC
{
  int32_T __dummy;
};

#endif                                 /*struct_tag_qN5Oiz2OtHsE7f06wrZAJC*/

#ifndef typedef_l_robotics_manip_internal_Col_T
#define typedef_l_robotics_manip_internal_Col_T

typedef tag_qN5Oiz2OtHsE7f06wrZAJC l_robotics_manip_internal_Col_T;

#endif                               /*typedef_l_robotics_manip_internal_Col_T*/

#ifndef struct_tag_KIm0luGquH6Gx0epdGQpnH
#define struct_tag_KIm0luGquH6Gx0epdGQpnH

struct tag_KIm0luGquH6Gx0epdGQpnH
{
  sdAmwXbnJnEmimT0NaJRtAD_Cinem_T StartTime;
};

#endif                                 /*struct_tag_KIm0luGquH6Gx0epdGQpnH*/

#ifndef typedef_f_robotics_core_internal_Syst_T
#define typedef_f_robotics_core_internal_Syst_T

typedef tag_KIm0luGquH6Gx0epdGQpnH f_robotics_core_internal_Syst_T;

#endif                               /*typedef_f_robotics_core_internal_Syst_T*/

#ifndef struct_emxArray_char_T
#define struct_emxArray_char_T

struct emxArray_char_T
{
  char_T *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
};

#endif                                 /*struct_emxArray_char_T*/

#ifndef typedef_emxArray_char_T_Cinematique_R_T
#define typedef_emxArray_char_T_Cinematique_R_T

typedef emxArray_char_T emxArray_char_T_Cinematique_R_T;

#endif                               /*typedef_emxArray_char_T_Cinematique_R_T*/

#ifndef struct_emxArray_real_T
#define struct_emxArray_real_T

struct emxArray_real_T
{
  real_T *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
};

#endif                                 /*struct_emxArray_real_T*/

#ifndef typedef_emxArray_real_T_Cinematique_R_T
#define typedef_emxArray_real_T_Cinematique_R_T

typedef emxArray_real_T emxArray_real_T_Cinematique_R_T;

#endif                               /*typedef_emxArray_real_T_Cinematique_R_T*/

#ifndef typedef_emxArray_unnamed_struct_Cinem_T
#define typedef_emxArray_unnamed_struct_Cinem_T

typedef struct {
  l_robotics_manip_internal_Col_T **data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
} emxArray_unnamed_struct_Cinem_T;

#endif                               /*typedef_emxArray_unnamed_struct_Cinem_T*/

#ifndef struct_emxArray_boolean_T
#define struct_emxArray_boolean_T

struct emxArray_boolean_T
{
  boolean_T *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
};

#endif                                 /*struct_emxArray_boolean_T*/

#ifndef typedef_emxArray_boolean_T_Cinematiqu_T
#define typedef_emxArray_boolean_T_Cinematiqu_T

typedef emxArray_boolean_T emxArray_boolean_T_Cinematiqu_T;

#endif                               /*typedef_emxArray_boolean_T_Cinematiqu_T*/

#ifndef struct_emxArray_int32_T
#define struct_emxArray_int32_T

struct emxArray_int32_T
{
  int32_T *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
};

#endif                                 /*struct_emxArray_int32_T*/

#ifndef typedef_emxArray_int32_T_Cinematique__T
#define typedef_emxArray_int32_T_Cinematique__T

typedef emxArray_int32_T emxArray_int32_T_Cinematique__T;

#endif                               /*typedef_emxArray_int32_T_Cinematique__T*/

#ifndef struct_tag_rAoU6BZpwiLAkT4vFxI5fF
#define struct_tag_rAoU6BZpwiLAkT4vFxI5fF

struct tag_rAoU6BZpwiLAkT4vFxI5fF
{
  emxArray_unnamed_struct_Cinem_T *CollisionGeometries;
  real_T MaxElements;
  real_T Size;
  l_robotics_manip_internal_Col_T _pobj0;
};

#endif                                 /*struct_tag_rAoU6BZpwiLAkT4vFxI5fF*/

#ifndef typedef_m_robotics_manip_internal_Col_T
#define typedef_m_robotics_manip_internal_Col_T

typedef tag_rAoU6BZpwiLAkT4vFxI5fF m_robotics_manip_internal_Col_T;

#endif                               /*typedef_m_robotics_manip_internal_Col_T*/

#ifndef struct_tag_s2AnghWSiQbdEs37ST6kiUG
#define struct_tag_s2AnghWSiQbdEs37ST6kiUG

struct tag_s2AnghWSiQbdEs37ST6kiUG
{
  real_T NameLength;
  uint8_T Name[5];
  real_T ParentIndex;
  real_T NumChildren;
  real_T ChildrenIndices[5];
  real_T Mass;
  real_T CenterOfMass[3];
  real_T Inertia[9];
  real_T SpatialInertia[36];
};

#endif                                 /*struct_tag_s2AnghWSiQbdEs37ST6kiUG*/

#ifndef typedef_s2AnghWSiQbdEs37ST6kiUG_Cinem_T
#define typedef_s2AnghWSiQbdEs37ST6kiUG_Cinem_T

typedef tag_s2AnghWSiQbdEs37ST6kiUG s2AnghWSiQbdEs37ST6kiUG_Cinem_T;

#endif                               /*typedef_s2AnghWSiQbdEs37ST6kiUG_Cinem_T*/

#ifndef struct_tag_swZ9yUSx6BPXU2zxjUKEQzD
#define struct_tag_swZ9yUSx6BPXU2zxjUKEQzD

struct tag_swZ9yUSx6BPXU2zxjUKEQzD
{
  uint8_T Type;
  real_T NameLength;
  uint8_T Name[6];
  real_T VelocityNumber;
  real_T PositionNumber;
  real_T MotionSubspace[36];
  real_T JointAxis[3];
  real_T PositionLimits[14];
  real_T HomePosition[7];
  real_T JointToParentTransform[16];
  real_T ChildToJointTransform[16];
};

#endif                                 /*struct_tag_swZ9yUSx6BPXU2zxjUKEQzD*/

#ifndef typedef_swZ9yUSx6BPXU2zxjUKEQzD_Cinem_T
#define typedef_swZ9yUSx6BPXU2zxjUKEQzD_Cinem_T

typedef tag_swZ9yUSx6BPXU2zxjUKEQzD swZ9yUSx6BPXU2zxjUKEQzD_Cinem_T;

#endif                               /*typedef_swZ9yUSx6BPXU2zxjUKEQzD_Cinem_T*/

#ifndef struct_tag_a9w8IowFHzogdOiVBZxs7
#define struct_tag_a9w8IowFHzogdOiVBZxs7

struct tag_a9w8IowFHzogdOiVBZxs7
{
  emxArray_char_T_Cinematique_R_T *Type;
  real_T VelocityNumber;
  real_T PositionNumber;
  emxArray_real_T_Cinematique_R_T *MotionSubspace;
  boolean_T InTree;
  real_T JointToParentTransform[16];
  real_T ChildToJointTransform[16];
  emxArray_char_T_Cinematique_R_T *NameInternal;
  emxArray_real_T_Cinematique_R_T *PositionLimitsInternal;
  emxArray_real_T_Cinematique_R_T *HomePositionInternal;
  real_T JointAxisInternal[3];
};

#endif                                 /*struct_tag_a9w8IowFHzogdOiVBZxs7*/

#ifndef typedef_c_rigidBodyJoint_Cinematique__T
#define typedef_c_rigidBodyJoint_Cinematique__T

typedef tag_a9w8IowFHzogdOiVBZxs7 c_rigidBodyJoint_Cinematique__T;

#endif                               /*typedef_c_rigidBodyJoint_Cinematique__T*/

#ifndef struct_tag_VQgqBkzYvS7jqEE6bcy6pF
#define struct_tag_VQgqBkzYvS7jqEE6bcy6pF

struct tag_VQgqBkzYvS7jqEE6bcy6pF
{
  real_T Index;
  emxArray_char_T_Cinematique_R_T *NameInternal;
  c_rigidBodyJoint_Cinematique__T JointInternal;
  real_T ParentIndex;
  real_T MassInternal;
  real_T CenterOfMassInternal[3];
  real_T InertiaInternal[9];
  real_T SpatialInertia[36];
  m_robotics_manip_internal_Col_T CollisionsInternal;
};

#endif                                 /*struct_tag_VQgqBkzYvS7jqEE6bcy6pF*/

#ifndef typedef_w_robotics_manip_internal_Rig_T
#define typedef_w_robotics_manip_internal_Rig_T

typedef tag_VQgqBkzYvS7jqEE6bcy6pF w_robotics_manip_internal_Rig_T;

#endif                               /*typedef_w_robotics_manip_internal_Rig_T*/

#ifndef struct_tag_KL8CUG9EguQykeBm4UBr4B
#define struct_tag_KL8CUG9EguQykeBm4UBr4B

struct tag_KL8CUG9EguQykeBm4UBr4B
{
  real_T NumBodies;
  w_robotics_manip_internal_Rig_T Base;
  real_T Gravity[3];
  w_robotics_manip_internal_Rig_T *Bodies[5];
  w_robotics_manip_internal_Rig_T _pobj0[5];
};

#endif                                 /*struct_tag_KL8CUG9EguQykeBm4UBr4B*/

#ifndef typedef_x_robotics_manip_internal_Rig_T
#define typedef_x_robotics_manip_internal_Rig_T

typedef tag_KL8CUG9EguQykeBm4UBr4B x_robotics_manip_internal_Rig_T;

#endif                               /*typedef_x_robotics_manip_internal_Rig_T*/

#ifndef struct_tag_g9J8khxeEsksT7SWBjkmU
#define struct_tag_g9J8khxeEsksT7SWBjkmU

struct tag_g9J8khxeEsksT7SWBjkmU
{
  real_T Index;
  emxArray_char_T_Cinematique_R_T *NameInternal;
  c_rigidBodyJoint_Cinematique__T *JointInternal;
  real_T ParentIndex;
  m_robotics_manip_internal_Col_T *CollisionsInternal;
  m_robotics_manip_internal_Col_T _pobj0;
  c_rigidBodyJoint_Cinematique__T _pobj1;
};

#endif                                 /*struct_tag_g9J8khxeEsksT7SWBjkmU*/

#ifndef typedef_y_robotics_manip_internal_Rig_T
#define typedef_y_robotics_manip_internal_Rig_T

typedef tag_g9J8khxeEsksT7SWBjkmU y_robotics_manip_internal_Rig_T;

#endif                               /*typedef_y_robotics_manip_internal_Rig_T*/

#ifndef struct_tag_BtddnDEKDrJdxzrbF0533G
#define struct_tag_BtddnDEKDrJdxzrbF0533G

struct tag_BtddnDEKDrJdxzrbF0533G
{
  real_T NumBodies;
  y_robotics_manip_internal_Rig_T Base;
  y_robotics_manip_internal_Rig_T *Bodies[5];
  real_T NumNonFixedBodies;
  real_T PositionNumber;
  real_T VelocityNumber;
  real_T PositionDoFMap[10];
  real_T VelocityDoFMap[10];
  y_robotics_manip_internal_Rig_T _pobj0[5];
};

#endif                                 /*struct_tag_BtddnDEKDrJdxzrbF0533G*/

#ifndef typedef_ab_robotics_manip_internal_Ri_T
#define typedef_ab_robotics_manip_internal_Ri_T

typedef tag_BtddnDEKDrJdxzrbF0533G ab_robotics_manip_internal_Ri_T;

#endif                               /*typedef_ab_robotics_manip_internal_Ri_T*/

#ifndef struct_tag_3YGdBhGoaUa7QJvgMkgODE
#define struct_tag_3YGdBhGoaUa7QJvgMkgODE

struct tag_3YGdBhGoaUa7QJvgMkgODE
{
  ab_robotics_manip_internal_Ri_T *Robot;
  real_T WeightMatrix[36];
  emxArray_char_T_Cinematique_R_T *BodyName;
  real_T Tform[16];
  emxArray_real_T_Cinematique_R_T *ErrTemp;
  real_T CostTemp;
  emxArray_real_T_Cinematique_R_T *GradTemp;
};

#endif                                 /*struct_tag_3YGdBhGoaUa7QJvgMkgODE*/

#ifndef typedef_f_robotics_manip_internal_IKE_T
#define typedef_f_robotics_manip_internal_IKE_T

typedef tag_3YGdBhGoaUa7QJvgMkgODE f_robotics_manip_internal_IKE_T;

#endif                               /*typedef_f_robotics_manip_internal_IKE_T*/

#ifndef struct_tag_tyYxWS48xPbiLg7Xr99XME
#define struct_tag_tyYxWS48xPbiLg7Xr99XME

struct tag_tyYxWS48xPbiLg7Xr99XME
{
  char_T Name[22];
  emxArray_real_T_Cinematique_R_T *ConstraintMatrix;
  emxArray_real_T_Cinematique_R_T *ConstraintBound;
  boolean_T ConstraintsOn;
  real_T SolutionTolerance;
  boolean_T RandomRestart;
  f_robotics_manip_internal_IKE_T *ExtraArgs;
  real_T MaxNumIteration;
  real_T MaxTime;
  real_T SeedInternal[4];
  real_T MaxTimeInternal;
  real_T MaxNumIterationInternal;
  real_T StepTolerance;
  f_robotics_core_internal_Syst_T TimeObj;
  real_T GradientTolerance;
  real_T ArmijoRuleBeta;
  real_T ArmijoRuleSigma;
  f_robotics_core_internal_Syst_T TimeObjInternal;
};

#endif                                 /*struct_tag_tyYxWS48xPbiLg7Xr99XME*/

#ifndef typedef_h_robotics_core_internal_Damp_T
#define typedef_h_robotics_core_internal_Damp_T

typedef tag_tyYxWS48xPbiLg7Xr99XME h_robotics_core_internal_Damp_T;

#endif                               /*typedef_h_robotics_core_internal_Damp_T*/

#ifndef struct_tag_hyhcJZKGSAx6gWytM6ZpLE
#define struct_tag_hyhcJZKGSAx6gWytM6ZpLE

struct tag_hyhcJZKGSAx6gWytM6ZpLE
{
  boolean_T matlabCodegenIsDeleted;
  int32_T isInitialized;
  boolean_T isSetupComplete;
  h_robotics_core_internal_Damp_T *Solver;
  emxArray_real_T_Cinematique_R_T *Limits;
  ab_robotics_manip_internal_Ri_T *RigidBodyTreeInternal;
  f_robotics_manip_internal_IKE_T _pobj0;
  ab_robotics_manip_internal_Ri_T _pobj1;
  m_robotics_manip_internal_Col_T _pobj2[6];
  c_rigidBodyJoint_Cinematique__T _pobj3[5];
  y_robotics_manip_internal_Rig_T _pobj4[5];
  h_robotics_core_internal_Damp_T _pobj5;
};

#endif                                 /*struct_tag_hyhcJZKGSAx6gWytM6ZpLE*/

#ifndef typedef_b_inverseKinematics_Cinematiq_T
#define typedef_b_inverseKinematics_Cinematiq_T

typedef tag_hyhcJZKGSAx6gWytM6ZpLE b_inverseKinematics_Cinematiq_T;

#endif                               /*typedef_b_inverseKinematics_Cinematiq_T*/

#ifndef struct_tag_6YCN3LS3P0WgfETiBZdjsE
#define struct_tag_6YCN3LS3P0WgfETiBZdjsE

struct tag_6YCN3LS3P0WgfETiBZdjsE
{
  boolean_T matlabCodegenIsDeleted;
  int32_T isInitialized;
  x_robotics_manip_internal_Rig_T TreeInternal;
  b_inverseKinematics_Cinematiq_T IKInternal;
};

#endif                                 /*struct_tag_6YCN3LS3P0WgfETiBZdjsE*/

#ifndef typedef_robotics_slmanip_internal_blo_T
#define typedef_robotics_slmanip_internal_blo_T

typedef tag_6YCN3LS3P0WgfETiBZdjsE robotics_slmanip_internal_blo_T;

#endif                               /*typedef_robotics_slmanip_internal_blo_T*/

#ifndef typedef_c_robotics_core_internal_NLPS_T
#define typedef_c_robotics_core_internal_NLPS_T

typedef int32_T c_robotics_core_internal_NLPS_T;

#endif                               /*typedef_c_robotics_core_internal_NLPS_T*/

#ifndef robotics_core_internal_NLPSolverExitFlags_constants
#define robotics_core_internal_NLPSolverExitFlags_constants

/* enum robotics_core_internal_NLPSolverExitFlags */
const c_robotics_core_internal_NLPS_T LocalMinimumFound = 1;
const c_robotics_core_internal_NLPS_T IterationLimitExceeded = 2;
const c_robotics_core_internal_NLPS_T TimeLimitExceeded = 3;
const c_robotics_core_internal_NLPS_T StepSizeBelowMinimum = 4;
const c_robotics_core_internal_NLPS_T ChangeInErrorBelowMinimum = 5;
const c_robotics_core_internal_NLPS_T SearchDirectionInvalid = 6;
const c_robotics_core_internal_NLPS_T HessianNotPositiveSemidefinite = 7;
const c_robotics_core_internal_NLPS_T TrustRegionRadiusBelowMinimum = 8;

#endif                   /*robotics_core_internal_NLPSolverExitFlags_constants*/

/* Parameters (default storage) */
typedef struct P_Cinematique_ROS_T_ P_Cinematique_ROS_T;

/* Forward declaration for rtModel */
typedef struct tag_RTM_Cinematique_ROS_T RT_MODEL_Cinematique_ROS_T;

#endif                                 /* RTW_HEADER_Cinematique_ROS_types_h_ */
