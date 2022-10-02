/*
 * InverseKinematics_types.h
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

#ifndef RTW_HEADER_InverseKinematics_types_h_
#define RTW_HEADER_InverseKinematics_types_h_
#include "rtwtypes.h"

/* Model Code Variants */
#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_geometry_msgs_Quaternion_
#define DEFINED_TYPEDEF_FOR_SL_Bus_geometry_msgs_Quaternion_

struct SL_Bus_geometry_msgs_Quaternion
{
  real_T x;
  real_T y;
  real_T z;
  real_T w;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_geometry_msgs_Point_
#define DEFINED_TYPEDEF_FOR_SL_Bus_geometry_msgs_Point_

struct SL_Bus_geometry_msgs_Point
{
  real_T x;
  real_T y;
  real_T z;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_ikblock_info_bus_
#define DEFINED_TYPEDEF_FOR_ikblock_info_bus_

struct ikblock_info_bus
{
  real_T Iterations;
  real_T PoseErrorNorm;
  uint16_T ExitFlag;
  uint8_T Status;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_j29BDD3GtugYMsepf4x9iH_
#define DEFINED_TYPEDEF_FOR_struct_j29BDD3GtugYMsepf4x9iH_

struct struct_j29BDD3GtugYMsepf4x9iH
{
  boolean_T EnforceJointLimits;
  boolean_T AllowRandomRestart;
  real_T MaxIterations;
  real_T MaxTime;
  real_T GradientTolerance;
  real_T SolutionTolerance;
  real_T StepTolerance;
  real_T ErrorChangeTolerance;
  boolean_T UseErrorDamping;
  real_T DampingBias;
};

#endif

#ifndef struct_robotics_slcore_internal_bloc_T
#define struct_robotics_slcore_internal_bloc_T

struct robotics_slcore_internal_bloc_T
{
  int32_T isInitialized;
};

#endif                              /* struct_robotics_slcore_internal_bloc_T */

#ifndef struct_sdAmwXbnJnEmimT0NaJRtAD_Inver_T
#define struct_sdAmwXbnJnEmimT0NaJRtAD_Inver_T

struct sdAmwXbnJnEmimT0NaJRtAD_Inver_T
{
  real_T tv_sec;
  real_T tv_nsec;
};

#endif                              /* struct_sdAmwXbnJnEmimT0NaJRtAD_Inver_T */

/* Custom Type definition for MATLABSystem: '<S4>/MATLAB System' */
#include "coder_posix_time.h"
#ifndef struct_k_robotics_manip_internal_Col_T
#define struct_k_robotics_manip_internal_Col_T

struct k_robotics_manip_internal_Col_T
{
  boolean_T matlabCodegenIsDeleted;
  void* CollisionPrimitive;
};

#endif                              /* struct_k_robotics_manip_internal_Col_T */

#ifndef struct_f_robotics_manip_internal_Fas_T
#define struct_f_robotics_manip_internal_Fas_T

struct f_robotics_manip_internal_Fas_T
{
  int32_T __dummy;
};

#endif                              /* struct_f_robotics_manip_internal_Fas_T */

#ifndef struct_f_robotics_core_internal_Syst_T
#define struct_f_robotics_core_internal_Syst_T

struct f_robotics_core_internal_Syst_T
{
  sdAmwXbnJnEmimT0NaJRtAD_Inver_T StartTime;
};

#endif                              /* struct_f_robotics_core_internal_Syst_T */

/* Custom Type definition for MATLABSystem: '<S3>/SourceBlock' */
#include "rmw/qos_profiles.h"
#include "rmw/types.h"
#include "rmw/types.h"
#include "rmw/types.h"
#ifndef struct_ros_slros2_internal_block_Pub_T
#define struct_ros_slros2_internal_block_Pub_T

struct ros_slros2_internal_block_Pub_T
{
  boolean_T matlabCodegenIsDeleted;
  int32_T isInitialized;
  boolean_T isSetupComplete;
};

#endif                              /* struct_ros_slros2_internal_block_Pub_T */

#ifndef struct_ros_slros2_internal_block_Sub_T
#define struct_ros_slros2_internal_block_Sub_T

struct ros_slros2_internal_block_Sub_T
{
  boolean_T matlabCodegenIsDeleted;
  int32_T isInitialized;
  boolean_T isSetupComplete;
};

#endif                              /* struct_ros_slros2_internal_block_Sub_T */

#ifndef struct_emxArray_char_T_InverseKinema_T
#define struct_emxArray_char_T_InverseKinema_T

struct emxArray_char_T_InverseKinema_T
{
  char_T *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
};

#endif                              /* struct_emxArray_char_T_InverseKinema_T */

#ifndef struct_emxArray_real_T_InverseKinema_T
#define struct_emxArray_real_T_InverseKinema_T

struct emxArray_real_T_InverseKinema_T
{
  real_T *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
};

#endif                              /* struct_emxArray_real_T_InverseKinema_T */

#ifndef struct_emxArray_unnamed_struct_Inver_T
#define struct_emxArray_unnamed_struct_Inver_T

struct emxArray_unnamed_struct_Inver_T
{
  k_robotics_manip_internal_Col_T **data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
};

#endif                              /* struct_emxArray_unnamed_struct_Inver_T */

#ifndef struct_c_rigidBodyJoint_InverseKinem_T
#define struct_c_rigidBodyJoint_InverseKinem_T

struct c_rigidBodyJoint_InverseKinem_T
{
  emxArray_char_T_InverseKinema_T *Type;
  real_T VelocityNumber;
  real_T PositionNumber;
  emxArray_real_T_InverseKinema_T *MotionSubspace;
  boolean_T InTree;
  real_T JointToParentTransform[16];
  real_T ChildToJointTransform[16];
  emxArray_char_T_InverseKinema_T *NameInternal;
  emxArray_real_T_InverseKinema_T *PositionLimitsInternal;
  emxArray_real_T_InverseKinema_T *HomePositionInternal;
  real_T JointAxisInternal[3];
};

#endif                              /* struct_c_rigidBodyJoint_InverseKinem_T */

#ifndef struct_emxArray_int8_T_InverseKinema_T
#define struct_emxArray_int8_T_InverseKinema_T

struct emxArray_int8_T_InverseKinema_T
{
  int8_T *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
};

#endif                              /* struct_emxArray_int8_T_InverseKinema_T */

#ifndef struct_emxArray_boolean_T_InverseKin_T
#define struct_emxArray_boolean_T_InverseKin_T

struct emxArray_boolean_T_InverseKin_T
{
  boolean_T *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
};

#endif                              /* struct_emxArray_boolean_T_InverseKin_T */

#ifndef struct_emxArray_int32_T_InverseKinem_T
#define struct_emxArray_int32_T_InverseKinem_T

struct emxArray_int32_T_InverseKinem_T
{
  int32_T *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
};

#endif                              /* struct_emxArray_int32_T_InverseKinem_T */

#ifndef struct_emxArray_uint32_T_InverseKine_T
#define struct_emxArray_uint32_T_InverseKine_T

struct emxArray_uint32_T_InverseKine_T
{
  uint32_T *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
};

#endif                              /* struct_emxArray_uint32_T_InverseKine_T */

#ifndef struct_l_robotics_manip_internal_Col_T
#define struct_l_robotics_manip_internal_Col_T

struct l_robotics_manip_internal_Col_T
{
  emxArray_unnamed_struct_Inver_T *CollisionGeometries;
  real_T MaxElements;
  real_T Size;
  k_robotics_manip_internal_Col_T _pobj0;
};

#endif                              /* struct_l_robotics_manip_internal_Col_T */

#ifndef struct_t_robotics_manip_internal_Rig_T
#define struct_t_robotics_manip_internal_Rig_T

struct t_robotics_manip_internal_Rig_T
{
  real_T Index;
  emxArray_char_T_InverseKinema_T *NameInternal;
  c_rigidBodyJoint_InverseKinem_T *JointInternal;
  real_T ParentIndex;
  real_T MassInternal;
  real_T CenterOfMassInternal[3];
  real_T InertiaInternal[9];
  real_T SpatialInertia[36];
  l_robotics_manip_internal_Col_T *CollisionsInternal;
};

#endif                              /* struct_t_robotics_manip_internal_Rig_T */

#ifndef struct_v_robotics_manip_internal_Rig_T
#define struct_v_robotics_manip_internal_Rig_T

struct v_robotics_manip_internal_Rig_T
{
  real_T NumBodies;
  t_robotics_manip_internal_Rig_T Base;
  f_robotics_manip_internal_Fas_T FastVisualizationHelper;
  t_robotics_manip_internal_Rig_T *Bodies[6];
  real_T NumNonFixedBodies;
  real_T PositionNumber;
  real_T VelocityNumber;
  real_T PositionDoFMap[12];
  real_T VelocityDoFMap[12];
  t_robotics_manip_internal_Rig_T _pobj0[6];
  l_robotics_manip_internal_Col_T _pobj1[7];
  c_rigidBodyJoint_InverseKinem_T _pobj2[7];
};

#endif                              /* struct_v_robotics_manip_internal_Rig_T */

#ifndef struct_u_robotics_manip_internal_Rig_T
#define struct_u_robotics_manip_internal_Rig_T

struct u_robotics_manip_internal_Rig_T
{
  real_T NumBodies;
  t_robotics_manip_internal_Rig_T Base;
  real_T Gravity[3];
  t_robotics_manip_internal_Rig_T *Bodies[6];
  l_robotics_manip_internal_Col_T _pobj0[13];
  c_rigidBodyJoint_InverseKinem_T _pobj1[13];
  t_robotics_manip_internal_Rig_T _pobj2[12];
};

#endif                              /* struct_u_robotics_manip_internal_Rig_T */

#ifndef struct_f_robotics_manip_internal_IKE_T
#define struct_f_robotics_manip_internal_IKE_T

struct f_robotics_manip_internal_IKE_T
{
  v_robotics_manip_internal_Rig_T *Robot;
  real_T WeightMatrix[36];
  emxArray_real_T_InverseKinema_T *Limits;
  emxArray_char_T_InverseKinema_T *BodyName;
  real_T Tform[16];
  emxArray_real_T_InverseKinema_T *ErrTemp;
  real_T CostTemp;
  emxArray_real_T_InverseKinema_T *GradTemp;
};

#endif                              /* struct_f_robotics_manip_internal_IKE_T */

#ifndef struct_h_robotics_core_internal_Erro_T
#define struct_h_robotics_core_internal_Erro_T

struct h_robotics_core_internal_Erro_T
{
  char_T Name[18];
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
  real_T ErrorChangeTolerance;
  real_T DampingBias;
  boolean_T UseErrorDamping;
  f_robotics_core_internal_Syst_T TimeObjInternal;
};

#endif                              /* struct_h_robotics_core_internal_Erro_T */

#ifndef struct_b_inverseKinematics_InverseKi_T
#define struct_b_inverseKinematics_InverseKi_T

struct b_inverseKinematics_InverseKi_T
{
  boolean_T matlabCodegenIsDeleted;
  int32_T isInitialized;
  boolean_T isSetupComplete;
  h_robotics_core_internal_Erro_T *Solver;
  emxArray_real_T_InverseKinema_T *Limits;
  v_robotics_manip_internal_Rig_T *RigidBodyTreeInternal;
  f_robotics_manip_internal_IKE_T _pobj0;
  c_rigidBodyJoint_InverseKinem_T _pobj1[12];
  t_robotics_manip_internal_Rig_T _pobj2[6];
  l_robotics_manip_internal_Col_T _pobj3[13];
  v_robotics_manip_internal_Rig_T _pobj4;
  h_robotics_core_internal_Erro_T _pobj5;
};

#endif                              /* struct_b_inverseKinematics_InverseKi_T */

#ifndef struct_robotics_slmanip_internal_blo_T
#define struct_robotics_slmanip_internal_blo_T

struct robotics_slmanip_internal_blo_T
{
  boolean_T matlabCodegenIsDeleted;
  int32_T isInitialized;
  u_robotics_manip_internal_Rig_T TreeInternal;
  b_inverseKinematics_InverseKi_T IKInternal;
};

#endif                              /* struct_robotics_slmanip_internal_blo_T */

#ifndef typedef_c_robotics_core_internal_NLPS_T
#define typedef_c_robotics_core_internal_NLPS_T

typedef int32_T c_robotics_core_internal_NLPS_T;

#endif                             /* typedef_c_robotics_core_internal_NLPS_T */

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

#endif                 /* robotics_core_internal_NLPSolverExitFlags_constants */

/* Parameters (default storage) */
typedef struct P_InverseKinematics_T_ P_InverseKinematics_T;

/* Forward declaration for rtModel */
typedef struct tag_RTM_InverseKinematics_T RT_MODEL_InverseKinematics_T;

#endif                               /* RTW_HEADER_InverseKinematics_types_h_ */
