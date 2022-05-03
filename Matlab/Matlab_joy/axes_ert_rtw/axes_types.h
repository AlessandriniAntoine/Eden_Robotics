/*
 * axes_types.h
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

#ifndef RTW_HEADER_axes_types_h_
#define RTW_HEADER_axes_types_h_
#include "rtwtypes.h"
#include "multiword_types.h"

/* Model Code Variants */
#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_axes_ros_time_Time_
#define DEFINED_TYPEDEF_FOR_SL_Bus_axes_ros_time_Time_

struct SL_Bus_axes_ros_time_Time
{
  real_T Sec;
  real_T Nsec;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_ROSVariableLengthArrayInfo_
#define DEFINED_TYPEDEF_FOR_SL_Bus_ROSVariableLengthArrayInfo_

struct SL_Bus_ROSVariableLengthArrayInfo
{
  uint32_T CurrentLength;
  uint32_T ReceivedLength;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_axes_std_msgs_Header_
#define DEFINED_TYPEDEF_FOR_SL_Bus_axes_std_msgs_Header_

struct SL_Bus_axes_std_msgs_Header
{
  uint32_T Seq;
  SL_Bus_axes_ros_time_Time Stamp;
  uint8_T FrameId[128];
  SL_Bus_ROSVariableLengthArrayInfo FrameId_SL_Info;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_axes_sensor_msgs_Joy_
#define DEFINED_TYPEDEF_FOR_SL_Bus_axes_sensor_msgs_Joy_

struct SL_Bus_axes_sensor_msgs_Joy
{
  SL_Bus_axes_std_msgs_Header Header;
  real32_T Axes[128];
  SL_Bus_ROSVariableLengthArrayInfo Axes_SL_Info;
  int32_T Buttons[128];
  SL_Bus_ROSVariableLengthArrayInfo Buttons_SL_Info;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_axes_geometry_msgs_Quaternion_
#define DEFINED_TYPEDEF_FOR_SL_Bus_axes_geometry_msgs_Quaternion_

struct SL_Bus_axes_geometry_msgs_Quaternion
{
  real_T X;
  real_T Y;
  real_T Z;
  real_T W;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_axes_geometry_msgs_Point_
#define DEFINED_TYPEDEF_FOR_SL_Bus_axes_geometry_msgs_Point_

struct SL_Bus_axes_geometry_msgs_Point
{
  real_T X;
  real_T Y;
  real_T Z;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_axes_std_msgs_Int16_
#define DEFINED_TYPEDEF_FOR_SL_Bus_axes_std_msgs_Int16_

struct SL_Bus_axes_std_msgs_Int16
{
  int16_T Data;
};

#endif

#ifndef struct_i_robotics_manip_internal_Col_T
#define struct_i_robotics_manip_internal_Col_T

struct i_robotics_manip_internal_Col_T
{
  boolean_T matlabCodegenIsDeleted;
  void* CollisionPrimitive;
};

#endif                              /* struct_i_robotics_manip_internal_Col_T */

#ifndef struct_ros_slroscpp_internal_block_S_T
#define struct_ros_slroscpp_internal_block_S_T

struct ros_slroscpp_internal_block_S_T
{
  boolean_T matlabCodegenIsDeleted;
  int32_T isInitialized;
  boolean_T isSetupComplete;
};

#endif                              /* struct_ros_slroscpp_internal_block_S_T */

#ifndef struct_ros_slroscpp_internal_block_P_T
#define struct_ros_slroscpp_internal_block_P_T

struct ros_slroscpp_internal_block_P_T
{
  boolean_T matlabCodegenIsDeleted;
  int32_T isInitialized;
  boolean_T isSetupComplete;
};

#endif                              /* struct_ros_slroscpp_internal_block_P_T */

#ifndef struct_d_cell_wrap_axes_T
#define struct_d_cell_wrap_axes_T

struct d_cell_wrap_axes_T
{
  real_T f1[16];
};

#endif                                 /* struct_d_cell_wrap_axes_T */

#ifndef struct_emxArray_char_T_axes_T
#define struct_emxArray_char_T_axes_T

struct emxArray_char_T_axes_T
{
  char_T *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
};

#endif                                 /* struct_emxArray_char_T_axes_T */

#ifndef struct_emxArray_unnamed_struct_axes_T
#define struct_emxArray_unnamed_struct_axes_T

struct emxArray_unnamed_struct_axes_T
{
  i_robotics_manip_internal_Col_T **data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
};

#endif                               /* struct_emxArray_unnamed_struct_axes_T */

#ifndef struct_c_rigidBodyJoint_axes_T
#define struct_c_rigidBodyJoint_axes_T

struct c_rigidBodyJoint_axes_T
{
  emxArray_char_T_axes_T *Type;
  real_T PositionNumber;
  real_T JointToParentTransform[16];
  real_T ChildToJointTransform[16];
  real_T JointAxisInternal[3];
};

#endif                                 /* struct_c_rigidBodyJoint_axes_T */

#ifndef struct_j_robotics_manip_internal_Col_T
#define struct_j_robotics_manip_internal_Col_T

struct j_robotics_manip_internal_Col_T
{
  emxArray_unnamed_struct_axes_T *CollisionGeometries;
  real_T MaxElements;
  i_robotics_manip_internal_Col_T _pobj0;
};

#endif                              /* struct_j_robotics_manip_internal_Col_T */

#ifndef struct_k_robotics_manip_internal_Rig_T
#define struct_k_robotics_manip_internal_Rig_T

struct k_robotics_manip_internal_Rig_T
{
  emxArray_char_T_axes_T *NameInternal;
  c_rigidBodyJoint_axes_T JointInternal;
  real_T ParentIndex;
  j_robotics_manip_internal_Col_T CollisionsInternal;
};

#endif                              /* struct_k_robotics_manip_internal_Rig_T */

#ifndef struct_l_robotics_manip_internal_Rig_T
#define struct_l_robotics_manip_internal_Rig_T

struct l_robotics_manip_internal_Rig_T
{
  real_T NumBodies;
  k_robotics_manip_internal_Rig_T Base;
  k_robotics_manip_internal_Rig_T *Bodies[6];
  real_T PositionNumber;
  k_robotics_manip_internal_Rig_T _pobj0[6];
};

#endif                              /* struct_l_robotics_manip_internal_Rig_T */

#ifndef struct_robotics_slmanip_internal_blo_T
#define struct_robotics_slmanip_internal_blo_T

struct robotics_slmanip_internal_blo_T
{
  int32_T isInitialized;
  l_robotics_manip_internal_Rig_T TreeInternal;
};

#endif                              /* struct_robotics_slmanip_internal_blo_T */

#ifndef struct_emxArray_d_cell_wrap_axes_T
#define struct_emxArray_d_cell_wrap_axes_T

struct emxArray_d_cell_wrap_axes_T
{
  d_cell_wrap_axes_T *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
};

#endif                                 /* struct_emxArray_d_cell_wrap_axes_T */

/* Parameters for system: '<S4>/poser' */
typedef struct P_poser_axes_T_ P_poser_axes_T;

/* Parameters (default storage) */
typedef struct P_axes_T_ P_axes_T;

/* Forward declaration for rtModel */
typedef struct tag_RTM_axes_T RT_MODEL_axes_T;

#endif                                 /* RTW_HEADER_axes_types_h_ */
