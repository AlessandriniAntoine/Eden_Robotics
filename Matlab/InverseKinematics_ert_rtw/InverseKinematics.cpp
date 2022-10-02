/*
 * InverseKinematics.cpp
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

#include "InverseKinematics.h"
#include "InverseKinematics_types.h"
#include "rtwtypes.h"
#include <string.h>

extern "C" {

#include "rt_nonfinite.h"

}
#include <emmintrin.h>
#include <math.h>
#include "coder_posix_time.h"
#include "InverseKinematics_private.h"
#include "rmw/qos_profiles.h"
#include "rmw/types.h"
#include <stddef.h>
#include "rt_defines.h"
#include <stdlib.h>

int32_T div_s32(int32_T numerator, int32_T denominator)
{
  int32_T quotient;
  if (denominator == 0) {
    quotient = numerator >= 0 ? MAX_int32_T : MIN_int32_T;

    /* Divide by zero handler */
  } else {
    uint32_T tempAbsQuotient;
    tempAbsQuotient = (numerator < 0 ? ~static_cast<uint32_T>(numerator) + 1U :
                       static_cast<uint32_T>(numerator)) / (denominator < 0 ? ~
      static_cast<uint32_T>(denominator) + 1U : static_cast<uint32_T>
      (denominator));
    quotient = (numerator < 0) != (denominator < 0) ? -static_cast<int32_T>
      (tempAbsQuotient) : static_cast<int32_T>(tempAbsQuotient);
  }

  return quotient;
}

void InverseKinematics::InverseKine_SystemCore_setup_nl
  (ros_slros2_internal_block_Sub_T *obj)
{
  rmw_qos_durability_policy_t durability;
  rmw_qos_history_policy_t history;
  rmw_qos_profile_t qos_profile;
  rmw_qos_reliability_policy_t reliability;
  char_T b_zeroDelimTopic[11];
  static const char_T tmp[10] = { '/', 'R', 'o', 'b', 'o', 't', '/', 'R', 'e',
    'f' };

  obj->isSetupComplete = false;
  obj->isInitialized = 1;
  qos_profile = rmw_qos_profile_default;
  history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
  reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
  SET_QOS_VALUES(qos_profile, history, (size_t)1.0, durability, reliability);
  for (int32_T i = 0; i < 10; i++) {
    b_zeroDelimTopic[i] = tmp[i];
  }

  b_zeroDelimTopic[10] = '\x00';
  ros2::matlab::create_Sub_InverseKinematics_562(&b_zeroDelimTopic[0],
    qos_profile);
  obj->isSetupComplete = true;
}

void InverseKinematics::InverseKinematic_emxInit_char_T
  (emxArray_char_T_InverseKinema_T **pEmxArray, int32_T numDimensions)
{
  emxArray_char_T_InverseKinema_T *emxArray;
  *pEmxArray = static_cast<emxArray_char_T_InverseKinema_T *>(malloc(sizeof
    (emxArray_char_T_InverseKinema_T)));
  emxArray = *pEmxArray;
  emxArray->data = static_cast<char_T *>(NULL);
  emxArray->numDimensions = numDimensions;
  emxArray->size = static_cast<int32_T *>(malloc(sizeof(int32_T) * numDimensions));
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (InverseKinematics_B.i_c = 0; InverseKinematics_B.i_c < numDimensions;
       InverseKinematics_B.i_c++) {
    emxArray->size[InverseKinematics_B.i_c] = 0;
  }
}

void InverseKinematics::emxInitStruct_t_robotics_manip_
  (t_robotics_manip_internal_Rig_T *pStruct)
{
  InverseKinematic_emxInit_char_T(&pStruct->NameInternal, 2);
}

void InverseKinematics::InverseK_emxInit_unnamed_struct
  (emxArray_unnamed_struct_Inver_T **pEmxArray, int32_T numDimensions)
{
  emxArray_unnamed_struct_Inver_T *emxArray;
  *pEmxArray = static_cast<emxArray_unnamed_struct_Inver_T *>(malloc(sizeof
    (emxArray_unnamed_struct_Inver_T)));
  emxArray = *pEmxArray;
  emxArray->data = (k_robotics_manip_internal_Col_T **)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = static_cast<int32_T *>(malloc(sizeof(int32_T) * numDimensions));
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (int32_T i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

void InverseKinematics::emxInitStruct_l_robotics_manip_
  (l_robotics_manip_internal_Col_T *pStruct)
{
  InverseK_emxInit_unnamed_struct(&pStruct->CollisionGeometries, 2);
}

void InverseKinematics::emxInitMatrix_l_robotics_manip_
  (l_robotics_manip_internal_Col_T pMatrix[13])
{
  for (int32_T i = 0; i < 13; i++) {
    emxInitStruct_l_robotics_manip_(&pMatrix[i]);
  }
}

void InverseKinematics::InverseKinematic_emxInit_real_T
  (emxArray_real_T_InverseKinema_T **pEmxArray, int32_T numDimensions)
{
  emxArray_real_T_InverseKinema_T *emxArray;
  *pEmxArray = static_cast<emxArray_real_T_InverseKinema_T *>(malloc(sizeof
    (emxArray_real_T_InverseKinema_T)));
  emxArray = *pEmxArray;
  emxArray->data = static_cast<real_T *>(NULL);
  emxArray->numDimensions = numDimensions;
  emxArray->size = static_cast<int32_T *>(malloc(sizeof(int32_T) * numDimensions));
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (InverseKinematics_B.i_i = 0; InverseKinematics_B.i_i < numDimensions;
       InverseKinematics_B.i_i++) {
    emxArray->size[InverseKinematics_B.i_i] = 0;
  }
}

void InverseKinematics::emxInitStruct_c_rigidBodyJoint
  (c_rigidBodyJoint_InverseKinem_T *pStruct)
{
  InverseKinematic_emxInit_char_T(&pStruct->Type, 2);
  InverseKinematic_emxInit_real_T(&pStruct->MotionSubspace, 2);
  InverseKinematic_emxInit_char_T(&pStruct->NameInternal, 2);
  InverseKinematic_emxInit_real_T(&pStruct->PositionLimitsInternal, 2);
  InverseKinematic_emxInit_real_T(&pStruct->HomePositionInternal, 1);
}

void InverseKinematics::emxInitMatrix_c_rigidBodyJoint
  (c_rigidBodyJoint_InverseKinem_T pMatrix[13])
{
  for (int32_T i = 0; i < 13; i++) {
    emxInitStruct_c_rigidBodyJoint(&pMatrix[i]);
  }
}

void InverseKinematics::emxInitMatrix_t_robotics_manip_
  (t_robotics_manip_internal_Rig_T pMatrix[12])
{
  for (int32_T i = 0; i < 12; i++) {
    emxInitStruct_t_robotics_manip_(&pMatrix[i]);
  }
}

void InverseKinematics::emxInitStruct_u_robotics_manip_
  (u_robotics_manip_internal_Rig_T *pStruct)
{
  emxInitStruct_t_robotics_manip_(&pStruct->Base);
  emxInitMatrix_l_robotics_manip_(pStruct->_pobj0);
  emxInitMatrix_c_rigidBodyJoint(pStruct->_pobj1);
  emxInitMatrix_t_robotics_manip_(pStruct->_pobj2);
}

void InverseKinematics::emxInitStruct_f_robotics_manip_
  (f_robotics_manip_internal_IKE_T *pStruct)
{
  InverseKinematic_emxInit_real_T(&pStruct->Limits, 2);
  InverseKinematic_emxInit_char_T(&pStruct->BodyName, 2);
  InverseKinematic_emxInit_real_T(&pStruct->ErrTemp, 1);
  InverseKinematic_emxInit_real_T(&pStruct->GradTemp, 1);
}

void InverseKinematics::emxInitMatrix_c_rigidBodyJoint1
  (c_rigidBodyJoint_InverseKinem_T pMatrix[12])
{
  for (int32_T i = 0; i < 12; i++) {
    emxInitStruct_c_rigidBodyJoint(&pMatrix[i]);
  }
}

void InverseKinematics::emxInitMatrix_t_robotics_mani_n
  (t_robotics_manip_internal_Rig_T pMatrix[6])
{
  for (int32_T i = 0; i < 6; i++) {
    emxInitStruct_t_robotics_manip_(&pMatrix[i]);
  }
}

void InverseKinematics::emxInitMatrix_l_robotics_mani_n
  (l_robotics_manip_internal_Col_T pMatrix[7])
{
  for (int32_T i = 0; i < 7; i++) {
    emxInitStruct_l_robotics_manip_(&pMatrix[i]);
  }
}

void InverseKinematics::emxInitMatrix_c_rigidBodyJoint2
  (c_rigidBodyJoint_InverseKinem_T pMatrix[7])
{
  for (int32_T i = 0; i < 7; i++) {
    emxInitStruct_c_rigidBodyJoint(&pMatrix[i]);
  }
}

void InverseKinematics::emxInitStruct_v_robotics_manip_
  (v_robotics_manip_internal_Rig_T *pStruct)
{
  emxInitStruct_t_robotics_manip_(&pStruct->Base);
  emxInitMatrix_t_robotics_mani_n(pStruct->_pobj0);
  emxInitMatrix_l_robotics_mani_n(pStruct->_pobj1);
  emxInitMatrix_c_rigidBodyJoint2(pStruct->_pobj2);
}

void InverseKinematics::emxInitStruct_b_inverseKinemati
  (b_inverseKinematics_InverseKi_T *pStruct)
{
  InverseKinematic_emxInit_real_T(&pStruct->Limits, 2);
  emxInitStruct_f_robotics_manip_(&pStruct->_pobj0);
  emxInitMatrix_c_rigidBodyJoint1(pStruct->_pobj1);
  emxInitMatrix_t_robotics_mani_n(pStruct->_pobj2);
  emxInitMatrix_l_robotics_manip_(pStruct->_pobj3);
  emxInitStruct_v_robotics_manip_(&pStruct->_pobj4);
}

void InverseKinematics::emxInitStruct_robotics_slmanip_
  (robotics_slmanip_internal_blo_T *pStruct)
{
  emxInitStruct_u_robotics_manip_(&pStruct->TreeInternal);
  emxInitStruct_b_inverseKinemati(&pStruct->IKInternal);
}

void InverseKinematics::Invers_emxEnsureCapacity_char_T
  (emxArray_char_T_InverseKinema_T *emxArray, int32_T oldNumel)
{
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  InverseKinematics_B.newNumel_k = 1;
  for (InverseKinematics_B.i_p = 0; InverseKinematics_B.i_p <
       emxArray->numDimensions; InverseKinematics_B.i_p++) {
    InverseKinematics_B.newNumel_k *= emxArray->size[InverseKinematics_B.i_p];
  }

  if (InverseKinematics_B.newNumel_k > emxArray->allocatedSize) {
    InverseKinematics_B.i_p = emxArray->allocatedSize;
    if (InverseKinematics_B.i_p < 16) {
      InverseKinematics_B.i_p = 16;
    }

    while (InverseKinematics_B.i_p < InverseKinematics_B.newNumel_k) {
      if (InverseKinematics_B.i_p > 1073741823) {
        InverseKinematics_B.i_p = MAX_int32_T;
      } else {
        InverseKinematics_B.i_p <<= 1;
      }
    }

    newData = calloc(static_cast<uint32_T>(InverseKinematics_B.i_p), sizeof
                     (char_T));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(char_T) * oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }

    emxArray->data = static_cast<char_T *>(newData);
    emxArray->allocatedSize = InverseKinematics_B.i_p;
    emxArray->canFreeData = true;
  }
}

void InverseKinematics::InverseKinematic_emxFree_char_T
  (emxArray_char_T_InverseKinema_T **pEmxArray)
{
  if (*pEmxArray != static_cast<emxArray_char_T_InverseKinema_T *>(NULL)) {
    if (((*pEmxArray)->data != static_cast<char_T *>(NULL)) && (*pEmxArray)
        ->canFreeData) {
      free((*pEmxArray)->data);
    }

    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = static_cast<emxArray_char_T_InverseKinema_T *>(NULL);
  }
}

void InverseKinematics::Invers_emxEnsureCapacity_real_T
  (emxArray_real_T_InverseKinema_T *emxArray, int32_T oldNumel)
{
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  InverseKinematics_B.newNumel = 1;
  for (InverseKinematics_B.i_f = 0; InverseKinematics_B.i_f <
       emxArray->numDimensions; InverseKinematics_B.i_f++) {
    InverseKinematics_B.newNumel *= emxArray->size[InverseKinematics_B.i_f];
  }

  if (InverseKinematics_B.newNumel > emxArray->allocatedSize) {
    InverseKinematics_B.i_f = emxArray->allocatedSize;
    if (InverseKinematics_B.i_f < 16) {
      InverseKinematics_B.i_f = 16;
    }

    while (InverseKinematics_B.i_f < InverseKinematics_B.newNumel) {
      if (InverseKinematics_B.i_f > 1073741823) {
        InverseKinematics_B.i_f = MAX_int32_T;
      } else {
        InverseKinematics_B.i_f <<= 1;
      }
    }

    newData = calloc(static_cast<uint32_T>(InverseKinematics_B.i_f), sizeof
                     (real_T));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(real_T) * oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }

    emxArray->data = static_cast<real_T *>(newData);
    emxArray->allocatedSize = InverseKinematics_B.i_f;
    emxArray->canFreeData = true;
  }
}

void InverseKinematics::emxEnsureCapacity_unnamed_struc
  (emxArray_unnamed_struct_Inver_T *emxArray, int32_T oldNumel)
{
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  InverseKinematics_B.newNumel_n = 1;
  for (InverseKinematics_B.i_l = 0; InverseKinematics_B.i_l <
       emxArray->numDimensions; InverseKinematics_B.i_l++) {
    InverseKinematics_B.newNumel_n *= emxArray->size[InverseKinematics_B.i_l];
  }

  if (InverseKinematics_B.newNumel_n > emxArray->allocatedSize) {
    InverseKinematics_B.i_l = emxArray->allocatedSize;
    if (InverseKinematics_B.i_l < 16) {
      InverseKinematics_B.i_l = 16;
    }

    while (InverseKinematics_B.i_l < InverseKinematics_B.newNumel_n) {
      if (InverseKinematics_B.i_l > 1073741823) {
        InverseKinematics_B.i_l = MAX_int32_T;
      } else {
        InverseKinematics_B.i_l <<= 1;
      }
    }

    newData = calloc(static_cast<uint32_T>(InverseKinematics_B.i_l), sizeof
                     (k_robotics_manip_internal_Col_T *));
    if (emxArray->data != NULL) {
      memcpy(newData, (void *)emxArray->data, sizeof
             (k_robotics_manip_internal_Col_T *) * oldNumel);
      if (emxArray->canFreeData) {
        free((void *)emxArray->data);
      }
    }

    emxArray->data = (k_robotics_manip_internal_Col_T **)newData;
    emxArray->allocatedSize = InverseKinematics_B.i_l;
    emxArray->canFreeData = true;
  }
}

void InverseKinematics::InverseK_emxFree_unnamed_struct
  (emxArray_unnamed_struct_Inver_T **pEmxArray)
{
  if (*pEmxArray != static_cast<emxArray_unnamed_struct_Inver_T *>(NULL)) {
    if (((*pEmxArray)->data != (k_robotics_manip_internal_Col_T **)NULL) &&
        (*pEmxArray)->canFreeData) {
      free((void *)(*pEmxArray)->data);
    }

    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = static_cast<emxArray_unnamed_struct_Inver_T *>(NULL);
  }
}

l_robotics_manip_internal_Col_T *InverseKinematics::
  Inver_CollisionSet_CollisionSet(l_robotics_manip_internal_Col_T *obj, real_T
  maxElements)
{
  emxArray_unnamed_struct_Inver_T *e;
  k_robotics_manip_internal_Col_T *obj_0;
  l_robotics_manip_internal_Col_T *b_obj;
  InverseK_emxInit_unnamed_struct(&e, 2);
  obj->Size = 0.0;
  b_obj = obj;
  obj->MaxElements = maxElements;
  InverseKinematics_B.b_i_o3 = e->size[0] * e->size[1];
  e->size[1] = static_cast<int32_T>(obj->MaxElements);
  emxEnsureCapacity_unnamed_struc(e, InverseKinematics_B.b_i_o3);
  InverseKinematics_B.b_i_o3 = obj->CollisionGeometries->size[0] *
    obj->CollisionGeometries->size[1];
  obj->CollisionGeometries->size[0] = 1;
  obj->CollisionGeometries->size[1] = e->size[1];
  emxEnsureCapacity_unnamed_struc(obj->CollisionGeometries,
    InverseKinematics_B.b_i_o3);
  InverseKinematics_B.defaultCollisionObj_GeometryInt = 0;
  obj_0 = &obj->_pobj0;
  obj->_pobj0.CollisionPrimitive =
    InverseKinematics_B.defaultCollisionObj_GeometryInt;
  obj->_pobj0.matlabCodegenIsDeleted = false;
  InverseKinematics_B.c = obj->MaxElements;
  InverseKinematics_B.d_m = static_cast<int32_T>(InverseKinematics_B.c) - 1;
  InverseK_emxFree_unnamed_struct(&e);
  for (InverseKinematics_B.b_i_o3 = 0; InverseKinematics_B.b_i_o3 <=
       InverseKinematics_B.d_m; InverseKinematics_B.b_i_o3++) {
    obj->CollisionGeometries->data[InverseKinematics_B.b_i_o3] = obj_0;
  }

  return b_obj;
}

t_robotics_manip_internal_Rig_T *InverseKinematics::
  InverseKine_RigidBody_RigidBody(t_robotics_manip_internal_Rig_T *obj,
  l_robotics_manip_internal_Col_T *iobj_0, c_rigidBodyJoint_InverseKinem_T
  *iobj_1)
{
  emxArray_char_T_InverseKinema_T *switch_expression;
  t_robotics_manip_internal_Rig_T *b_obj;
  real_T poslim_data[12];
  int32_T b_kstr;
  int32_T loop_ub;
  char_T b_0[9];
  char_T b[8];
  int8_T msubspace_data[36];
  int8_T b_I[9];
  int8_T tmp[6];
  boolean_T b_bool;
  static const char_T tmp_0[10] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '1' };

  static const int8_T tmp_1[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    1 };

  static const char_T tmp_2[14] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '1', '_', 'j', 'n', 't' };

  static const char_T tmp_3[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_4[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_5[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  int32_T exitg1;
  b_obj = obj;
  b_kstr = obj->NameInternal->size[0] * obj->NameInternal->size[1];
  obj->NameInternal->size[0] = 1;
  obj->NameInternal->size[1] = 10;
  Invers_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 10; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp_0[b_kstr];
  }

  iobj_1->InTree = false;
  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    iobj_1->JointToParentTransform[b_kstr] = tmp_1[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    iobj_1->ChildToJointTransform[b_kstr] = tmp_1[b_kstr];
  }

  b_kstr = iobj_1->NameInternal->size[0] * iobj_1->NameInternal->size[1];
  iobj_1->NameInternal->size[0] = 1;
  iobj_1->NameInternal->size[1] = 14;
  Invers_emxEnsureCapacity_char_T(iobj_1->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 14; b_kstr++) {
    iobj_1->NameInternal->data[b_kstr] = tmp_2[b_kstr];
  }

  b_kstr = iobj_1->Type->size[0] * iobj_1->Type->size[1];
  iobj_1->Type->size[0] = 1;
  iobj_1->Type->size[1] = 5;
  Invers_emxEnsureCapacity_char_T(iobj_1->Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    iobj_1->Type->data[b_kstr] = tmp_3[b_kstr];
  }

  InverseKinematic_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = iobj_1->Type->size[1];
  Invers_emxEnsureCapacity_char_T(switch_expression, b_kstr);
  loop_ub = iobj_1->Type->size[1] - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = iobj_1->Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp_4[b_kstr];
  }

  b_bool = false;
  if (switch_expression->size[1] != 8) {
  } else {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        if (switch_expression->data[b_kstr - 1] != b[b_kstr - 1]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (b_bool) {
    b_kstr = 0;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      b_0[b_kstr] = tmp_5[b_kstr];
    }

    b_bool = false;
    if (switch_expression->size[1] != 9) {
    } else {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          if (switch_expression->data[b_kstr - 1] != b_0[b_kstr - 1]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (b_bool) {
      b_kstr = 1;
    } else {
      b_kstr = -1;
    }
  }

  InverseKinematic_emxFree_char_T(&switch_expression);
  switch (b_kstr) {
   case 0:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 1;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 0;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    poslim_data[0] = -3.1415926535897931;
    poslim_data[1] = 3.1415926535897931;
    iobj_1->VelocityNumber = 1.0;
    iobj_1->PositionNumber = 1.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 1.0;
    break;

   case 1:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 0;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 1;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    poslim_data[0] = -0.5;
    poslim_data[1] = 0.5;
    iobj_1->VelocityNumber = 1.0;
    iobj_1->PositionNumber = 1.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = 0;
    }

    poslim_data[0] = 0.0;
    poslim_data[1] = 0.0;
    iobj_1->VelocityNumber = 0.0;
    iobj_1->PositionNumber = 0.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 0.0;
    break;
  }

  b_kstr = iobj_1->MotionSubspace->size[0] * iobj_1->MotionSubspace->size[1];
  iobj_1->MotionSubspace->size[0] = 6;
  iobj_1->MotionSubspace->size[1] = 1;
  Invers_emxEnsureCapacity_real_T(iobj_1->MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    iobj_1->MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  b_kstr = iobj_1->PositionLimitsInternal->size[0] *
    iobj_1->PositionLimitsInternal->size[1];
  iobj_1->PositionLimitsInternal->size[0] = 1;
  iobj_1->PositionLimitsInternal->size[1] = 2;
  Invers_emxEnsureCapacity_real_T(iobj_1->PositionLimitsInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 2; b_kstr++) {
    iobj_1->PositionLimitsInternal->data[b_kstr] = poslim_data[b_kstr];
  }

  b_kstr = iobj_1->HomePositionInternal->size[0];
  iobj_1->HomePositionInternal->size[0] = 1;
  Invers_emxEnsureCapacity_real_T(iobj_1->HomePositionInternal, b_kstr);
  iobj_1->HomePositionInternal->data[0] = 0.0;
  obj->JointInternal = iobj_1;
  obj->Index = -1.0;
  obj->ParentIndex = -1.0;
  obj->MassInternal = 1.0;
  obj->CenterOfMassInternal[0] = 0.0;
  obj->CenterOfMassInternal[1] = 0.0;
  obj->CenterOfMassInternal[2] = 0.0;
  for (b_kstr = 0; b_kstr < 9; b_kstr++) {
    b_I[b_kstr] = 0;
  }

  b_I[0] = 1;
  b_I[4] = 1;
  b_I[8] = 1;
  for (b_kstr = 0; b_kstr < 9; b_kstr++) {
    obj->InertiaInternal[b_kstr] = b_I[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 36; b_kstr++) {
    msubspace_data[b_kstr] = 0;
  }

  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    msubspace_data[b_kstr + 6 * b_kstr] = 1;
  }

  for (b_kstr = 0; b_kstr < 36; b_kstr++) {
    obj->SpatialInertia[b_kstr] = msubspace_data[b_kstr];
  }

  obj->CollisionsInternal = Inver_CollisionSet_CollisionSet(iobj_0, 0.0);
  return b_obj;
}

t_robotics_manip_internal_Rig_T *InverseKinematics::
  InverseKi_RigidBody_RigidBody_n(t_robotics_manip_internal_Rig_T *obj,
  l_robotics_manip_internal_Col_T *iobj_0, c_rigidBodyJoint_InverseKinem_T
  *iobj_1)
{
  emxArray_char_T_InverseKinema_T *switch_expression;
  t_robotics_manip_internal_Rig_T *b_obj;
  real_T poslim_data[12];
  int32_T b_kstr;
  int32_T loop_ub;
  char_T b_0[9];
  char_T b[8];
  int8_T msubspace_data[36];
  int8_T b_I[9];
  int8_T tmp[6];
  boolean_T b_bool;
  static const char_T tmp_0[10] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '2' };

  static const int8_T tmp_1[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    1 };

  static const char_T tmp_2[14] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '2', '_', 'j', 'n', 't' };

  static const char_T tmp_3[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_4[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_5[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  int32_T exitg1;
  b_obj = obj;
  b_kstr = obj->NameInternal->size[0] * obj->NameInternal->size[1];
  obj->NameInternal->size[0] = 1;
  obj->NameInternal->size[1] = 10;
  Invers_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 10; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp_0[b_kstr];
  }

  iobj_1->InTree = false;
  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    iobj_1->JointToParentTransform[b_kstr] = tmp_1[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    iobj_1->ChildToJointTransform[b_kstr] = tmp_1[b_kstr];
  }

  b_kstr = iobj_1->NameInternal->size[0] * iobj_1->NameInternal->size[1];
  iobj_1->NameInternal->size[0] = 1;
  iobj_1->NameInternal->size[1] = 14;
  Invers_emxEnsureCapacity_char_T(iobj_1->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 14; b_kstr++) {
    iobj_1->NameInternal->data[b_kstr] = tmp_2[b_kstr];
  }

  b_kstr = iobj_1->Type->size[0] * iobj_1->Type->size[1];
  iobj_1->Type->size[0] = 1;
  iobj_1->Type->size[1] = 5;
  Invers_emxEnsureCapacity_char_T(iobj_1->Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    iobj_1->Type->data[b_kstr] = tmp_3[b_kstr];
  }

  InverseKinematic_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = iobj_1->Type->size[1];
  Invers_emxEnsureCapacity_char_T(switch_expression, b_kstr);
  loop_ub = iobj_1->Type->size[1] - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = iobj_1->Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp_4[b_kstr];
  }

  b_bool = false;
  if (switch_expression->size[1] != 8) {
  } else {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        if (switch_expression->data[b_kstr - 1] != b[b_kstr - 1]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (b_bool) {
    b_kstr = 0;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      b_0[b_kstr] = tmp_5[b_kstr];
    }

    b_bool = false;
    if (switch_expression->size[1] != 9) {
    } else {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          if (switch_expression->data[b_kstr - 1] != b_0[b_kstr - 1]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (b_bool) {
      b_kstr = 1;
    } else {
      b_kstr = -1;
    }
  }

  InverseKinematic_emxFree_char_T(&switch_expression);
  switch (b_kstr) {
   case 0:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 1;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 0;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    poslim_data[0] = -3.1415926535897931;
    poslim_data[1] = 3.1415926535897931;
    iobj_1->VelocityNumber = 1.0;
    iobj_1->PositionNumber = 1.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 1.0;
    break;

   case 1:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 0;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 1;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    poslim_data[0] = -0.5;
    poslim_data[1] = 0.5;
    iobj_1->VelocityNumber = 1.0;
    iobj_1->PositionNumber = 1.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = 0;
    }

    poslim_data[0] = 0.0;
    poslim_data[1] = 0.0;
    iobj_1->VelocityNumber = 0.0;
    iobj_1->PositionNumber = 0.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 0.0;
    break;
  }

  b_kstr = iobj_1->MotionSubspace->size[0] * iobj_1->MotionSubspace->size[1];
  iobj_1->MotionSubspace->size[0] = 6;
  iobj_1->MotionSubspace->size[1] = 1;
  Invers_emxEnsureCapacity_real_T(iobj_1->MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    iobj_1->MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  b_kstr = iobj_1->PositionLimitsInternal->size[0] *
    iobj_1->PositionLimitsInternal->size[1];
  iobj_1->PositionLimitsInternal->size[0] = 1;
  iobj_1->PositionLimitsInternal->size[1] = 2;
  Invers_emxEnsureCapacity_real_T(iobj_1->PositionLimitsInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 2; b_kstr++) {
    iobj_1->PositionLimitsInternal->data[b_kstr] = poslim_data[b_kstr];
  }

  b_kstr = iobj_1->HomePositionInternal->size[0];
  iobj_1->HomePositionInternal->size[0] = 1;
  Invers_emxEnsureCapacity_real_T(iobj_1->HomePositionInternal, b_kstr);
  iobj_1->HomePositionInternal->data[0] = 0.0;
  obj->JointInternal = iobj_1;
  obj->Index = -1.0;
  obj->ParentIndex = -1.0;
  obj->MassInternal = 1.0;
  obj->CenterOfMassInternal[0] = 0.0;
  obj->CenterOfMassInternal[1] = 0.0;
  obj->CenterOfMassInternal[2] = 0.0;
  for (b_kstr = 0; b_kstr < 9; b_kstr++) {
    b_I[b_kstr] = 0;
  }

  b_I[0] = 1;
  b_I[4] = 1;
  b_I[8] = 1;
  for (b_kstr = 0; b_kstr < 9; b_kstr++) {
    obj->InertiaInternal[b_kstr] = b_I[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 36; b_kstr++) {
    msubspace_data[b_kstr] = 0;
  }

  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    msubspace_data[b_kstr + 6 * b_kstr] = 1;
  }

  for (b_kstr = 0; b_kstr < 36; b_kstr++) {
    obj->SpatialInertia[b_kstr] = msubspace_data[b_kstr];
  }

  obj->CollisionsInternal = Inver_CollisionSet_CollisionSet(iobj_0, 0.0);
  return b_obj;
}

t_robotics_manip_internal_Rig_T *InverseKinematics::
  InverseK_RigidBody_RigidBody_nl(t_robotics_manip_internal_Rig_T *obj,
  l_robotics_manip_internal_Col_T *iobj_0, c_rigidBodyJoint_InverseKinem_T
  *iobj_1)
{
  emxArray_char_T_InverseKinema_T *switch_expression;
  t_robotics_manip_internal_Rig_T *b_obj;
  real_T poslim_data[12];
  int32_T b_kstr;
  int32_T loop_ub;
  char_T b_0[9];
  char_T b[8];
  int8_T msubspace_data[36];
  int8_T b_I[9];
  int8_T tmp[6];
  boolean_T b_bool;
  static const char_T tmp_0[10] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '3' };

  static const int8_T tmp_1[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    1 };

  static const char_T tmp_2[14] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '3', '_', 'j', 'n', 't' };

  static const char_T tmp_3[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_4[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_5[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  int32_T exitg1;
  b_obj = obj;
  b_kstr = obj->NameInternal->size[0] * obj->NameInternal->size[1];
  obj->NameInternal->size[0] = 1;
  obj->NameInternal->size[1] = 10;
  Invers_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 10; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp_0[b_kstr];
  }

  iobj_1->InTree = false;
  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    iobj_1->JointToParentTransform[b_kstr] = tmp_1[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    iobj_1->ChildToJointTransform[b_kstr] = tmp_1[b_kstr];
  }

  b_kstr = iobj_1->NameInternal->size[0] * iobj_1->NameInternal->size[1];
  iobj_1->NameInternal->size[0] = 1;
  iobj_1->NameInternal->size[1] = 14;
  Invers_emxEnsureCapacity_char_T(iobj_1->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 14; b_kstr++) {
    iobj_1->NameInternal->data[b_kstr] = tmp_2[b_kstr];
  }

  b_kstr = iobj_1->Type->size[0] * iobj_1->Type->size[1];
  iobj_1->Type->size[0] = 1;
  iobj_1->Type->size[1] = 5;
  Invers_emxEnsureCapacity_char_T(iobj_1->Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    iobj_1->Type->data[b_kstr] = tmp_3[b_kstr];
  }

  InverseKinematic_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = iobj_1->Type->size[1];
  Invers_emxEnsureCapacity_char_T(switch_expression, b_kstr);
  loop_ub = iobj_1->Type->size[1] - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = iobj_1->Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp_4[b_kstr];
  }

  b_bool = false;
  if (switch_expression->size[1] != 8) {
  } else {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        if (switch_expression->data[b_kstr - 1] != b[b_kstr - 1]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (b_bool) {
    b_kstr = 0;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      b_0[b_kstr] = tmp_5[b_kstr];
    }

    b_bool = false;
    if (switch_expression->size[1] != 9) {
    } else {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          if (switch_expression->data[b_kstr - 1] != b_0[b_kstr - 1]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (b_bool) {
      b_kstr = 1;
    } else {
      b_kstr = -1;
    }
  }

  InverseKinematic_emxFree_char_T(&switch_expression);
  switch (b_kstr) {
   case 0:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 1;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 0;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    poslim_data[0] = -3.1415926535897931;
    poslim_data[1] = 3.1415926535897931;
    iobj_1->VelocityNumber = 1.0;
    iobj_1->PositionNumber = 1.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 1.0;
    break;

   case 1:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 0;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 1;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    poslim_data[0] = -0.5;
    poslim_data[1] = 0.5;
    iobj_1->VelocityNumber = 1.0;
    iobj_1->PositionNumber = 1.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = 0;
    }

    poslim_data[0] = 0.0;
    poslim_data[1] = 0.0;
    iobj_1->VelocityNumber = 0.0;
    iobj_1->PositionNumber = 0.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 0.0;
    break;
  }

  b_kstr = iobj_1->MotionSubspace->size[0] * iobj_1->MotionSubspace->size[1];
  iobj_1->MotionSubspace->size[0] = 6;
  iobj_1->MotionSubspace->size[1] = 1;
  Invers_emxEnsureCapacity_real_T(iobj_1->MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    iobj_1->MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  b_kstr = iobj_1->PositionLimitsInternal->size[0] *
    iobj_1->PositionLimitsInternal->size[1];
  iobj_1->PositionLimitsInternal->size[0] = 1;
  iobj_1->PositionLimitsInternal->size[1] = 2;
  Invers_emxEnsureCapacity_real_T(iobj_1->PositionLimitsInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 2; b_kstr++) {
    iobj_1->PositionLimitsInternal->data[b_kstr] = poslim_data[b_kstr];
  }

  b_kstr = iobj_1->HomePositionInternal->size[0];
  iobj_1->HomePositionInternal->size[0] = 1;
  Invers_emxEnsureCapacity_real_T(iobj_1->HomePositionInternal, b_kstr);
  iobj_1->HomePositionInternal->data[0] = 0.0;
  obj->JointInternal = iobj_1;
  obj->Index = -1.0;
  obj->ParentIndex = -1.0;
  obj->MassInternal = 1.0;
  obj->CenterOfMassInternal[0] = 0.0;
  obj->CenterOfMassInternal[1] = 0.0;
  obj->CenterOfMassInternal[2] = 0.0;
  for (b_kstr = 0; b_kstr < 9; b_kstr++) {
    b_I[b_kstr] = 0;
  }

  b_I[0] = 1;
  b_I[4] = 1;
  b_I[8] = 1;
  for (b_kstr = 0; b_kstr < 9; b_kstr++) {
    obj->InertiaInternal[b_kstr] = b_I[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 36; b_kstr++) {
    msubspace_data[b_kstr] = 0;
  }

  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    msubspace_data[b_kstr + 6 * b_kstr] = 1;
  }

  for (b_kstr = 0; b_kstr < 36; b_kstr++) {
    obj->SpatialInertia[b_kstr] = msubspace_data[b_kstr];
  }

  obj->CollisionsInternal = Inver_CollisionSet_CollisionSet(iobj_0, 0.0);
  return b_obj;
}

t_robotics_manip_internal_Rig_T *InverseKinematics::
  Inverse_RigidBody_RigidBody_nly(t_robotics_manip_internal_Rig_T *obj,
  l_robotics_manip_internal_Col_T *iobj_0, c_rigidBodyJoint_InverseKinem_T
  *iobj_1)
{
  emxArray_char_T_InverseKinema_T *switch_expression;
  t_robotics_manip_internal_Rig_T *b_obj;
  real_T poslim_data[12];
  int32_T b_kstr;
  int32_T loop_ub;
  char_T b_0[9];
  char_T b[8];
  int8_T msubspace_data[36];
  int8_T b_I[9];
  int8_T tmp[6];
  boolean_T b_bool;
  static const char_T tmp_0[10] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '4' };

  static const int8_T tmp_1[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    1 };

  static const char_T tmp_2[14] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '4', '_', 'j', 'n', 't' };

  static const char_T tmp_3[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_4[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_5[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  int32_T exitg1;
  b_obj = obj;
  b_kstr = obj->NameInternal->size[0] * obj->NameInternal->size[1];
  obj->NameInternal->size[0] = 1;
  obj->NameInternal->size[1] = 10;
  Invers_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 10; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp_0[b_kstr];
  }

  iobj_1->InTree = false;
  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    iobj_1->JointToParentTransform[b_kstr] = tmp_1[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    iobj_1->ChildToJointTransform[b_kstr] = tmp_1[b_kstr];
  }

  b_kstr = iobj_1->NameInternal->size[0] * iobj_1->NameInternal->size[1];
  iobj_1->NameInternal->size[0] = 1;
  iobj_1->NameInternal->size[1] = 14;
  Invers_emxEnsureCapacity_char_T(iobj_1->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 14; b_kstr++) {
    iobj_1->NameInternal->data[b_kstr] = tmp_2[b_kstr];
  }

  b_kstr = iobj_1->Type->size[0] * iobj_1->Type->size[1];
  iobj_1->Type->size[0] = 1;
  iobj_1->Type->size[1] = 5;
  Invers_emxEnsureCapacity_char_T(iobj_1->Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    iobj_1->Type->data[b_kstr] = tmp_3[b_kstr];
  }

  InverseKinematic_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = iobj_1->Type->size[1];
  Invers_emxEnsureCapacity_char_T(switch_expression, b_kstr);
  loop_ub = iobj_1->Type->size[1] - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = iobj_1->Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp_4[b_kstr];
  }

  b_bool = false;
  if (switch_expression->size[1] != 8) {
  } else {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        if (switch_expression->data[b_kstr - 1] != b[b_kstr - 1]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (b_bool) {
    b_kstr = 0;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      b_0[b_kstr] = tmp_5[b_kstr];
    }

    b_bool = false;
    if (switch_expression->size[1] != 9) {
    } else {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          if (switch_expression->data[b_kstr - 1] != b_0[b_kstr - 1]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (b_bool) {
      b_kstr = 1;
    } else {
      b_kstr = -1;
    }
  }

  InverseKinematic_emxFree_char_T(&switch_expression);
  switch (b_kstr) {
   case 0:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 1;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 0;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    poslim_data[0] = -3.1415926535897931;
    poslim_data[1] = 3.1415926535897931;
    iobj_1->VelocityNumber = 1.0;
    iobj_1->PositionNumber = 1.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 1.0;
    break;

   case 1:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 0;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 1;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    poslim_data[0] = -0.5;
    poslim_data[1] = 0.5;
    iobj_1->VelocityNumber = 1.0;
    iobj_1->PositionNumber = 1.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = 0;
    }

    poslim_data[0] = 0.0;
    poslim_data[1] = 0.0;
    iobj_1->VelocityNumber = 0.0;
    iobj_1->PositionNumber = 0.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 0.0;
    break;
  }

  b_kstr = iobj_1->MotionSubspace->size[0] * iobj_1->MotionSubspace->size[1];
  iobj_1->MotionSubspace->size[0] = 6;
  iobj_1->MotionSubspace->size[1] = 1;
  Invers_emxEnsureCapacity_real_T(iobj_1->MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    iobj_1->MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  b_kstr = iobj_1->PositionLimitsInternal->size[0] *
    iobj_1->PositionLimitsInternal->size[1];
  iobj_1->PositionLimitsInternal->size[0] = 1;
  iobj_1->PositionLimitsInternal->size[1] = 2;
  Invers_emxEnsureCapacity_real_T(iobj_1->PositionLimitsInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 2; b_kstr++) {
    iobj_1->PositionLimitsInternal->data[b_kstr] = poslim_data[b_kstr];
  }

  b_kstr = iobj_1->HomePositionInternal->size[0];
  iobj_1->HomePositionInternal->size[0] = 1;
  Invers_emxEnsureCapacity_real_T(iobj_1->HomePositionInternal, b_kstr);
  iobj_1->HomePositionInternal->data[0] = 0.0;
  obj->JointInternal = iobj_1;
  obj->Index = -1.0;
  obj->ParentIndex = -1.0;
  obj->MassInternal = 1.0;
  obj->CenterOfMassInternal[0] = 0.0;
  obj->CenterOfMassInternal[1] = 0.0;
  obj->CenterOfMassInternal[2] = 0.0;
  for (b_kstr = 0; b_kstr < 9; b_kstr++) {
    b_I[b_kstr] = 0;
  }

  b_I[0] = 1;
  b_I[4] = 1;
  b_I[8] = 1;
  for (b_kstr = 0; b_kstr < 9; b_kstr++) {
    obj->InertiaInternal[b_kstr] = b_I[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 36; b_kstr++) {
    msubspace_data[b_kstr] = 0;
  }

  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    msubspace_data[b_kstr + 6 * b_kstr] = 1;
  }

  for (b_kstr = 0; b_kstr < 36; b_kstr++) {
    obj->SpatialInertia[b_kstr] = msubspace_data[b_kstr];
  }

  obj->CollisionsInternal = Inver_CollisionSet_CollisionSet(iobj_0, 0.0);
  return b_obj;
}

t_robotics_manip_internal_Rig_T *InverseKinematics::
  Invers_RigidBody_RigidBody_nlyk(t_robotics_manip_internal_Rig_T *obj,
  l_robotics_manip_internal_Col_T *iobj_0, c_rigidBodyJoint_InverseKinem_T
  *iobj_1)
{
  emxArray_char_T_InverseKinema_T *switch_expression;
  t_robotics_manip_internal_Rig_T *b_obj;
  real_T poslim_data[12];
  int32_T b_kstr;
  int32_T loop_ub;
  char_T b_0[9];
  char_T b[8];
  int8_T msubspace_data[36];
  int8_T b_I[9];
  int8_T tmp[6];
  boolean_T b_bool;
  static const char_T tmp_0[10] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '5' };

  static const int8_T tmp_1[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    1 };

  static const char_T tmp_2[14] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '5', '_', 'j', 'n', 't' };

  static const char_T tmp_3[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_4[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_5[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  int32_T exitg1;
  b_obj = obj;
  b_kstr = obj->NameInternal->size[0] * obj->NameInternal->size[1];
  obj->NameInternal->size[0] = 1;
  obj->NameInternal->size[1] = 10;
  Invers_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 10; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp_0[b_kstr];
  }

  iobj_1->InTree = false;
  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    iobj_1->JointToParentTransform[b_kstr] = tmp_1[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    iobj_1->ChildToJointTransform[b_kstr] = tmp_1[b_kstr];
  }

  b_kstr = iobj_1->NameInternal->size[0] * iobj_1->NameInternal->size[1];
  iobj_1->NameInternal->size[0] = 1;
  iobj_1->NameInternal->size[1] = 14;
  Invers_emxEnsureCapacity_char_T(iobj_1->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 14; b_kstr++) {
    iobj_1->NameInternal->data[b_kstr] = tmp_2[b_kstr];
  }

  b_kstr = iobj_1->Type->size[0] * iobj_1->Type->size[1];
  iobj_1->Type->size[0] = 1;
  iobj_1->Type->size[1] = 5;
  Invers_emxEnsureCapacity_char_T(iobj_1->Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    iobj_1->Type->data[b_kstr] = tmp_3[b_kstr];
  }

  InverseKinematic_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = iobj_1->Type->size[1];
  Invers_emxEnsureCapacity_char_T(switch_expression, b_kstr);
  loop_ub = iobj_1->Type->size[1] - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = iobj_1->Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp_4[b_kstr];
  }

  b_bool = false;
  if (switch_expression->size[1] != 8) {
  } else {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        if (switch_expression->data[b_kstr - 1] != b[b_kstr - 1]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (b_bool) {
    b_kstr = 0;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      b_0[b_kstr] = tmp_5[b_kstr];
    }

    b_bool = false;
    if (switch_expression->size[1] != 9) {
    } else {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          if (switch_expression->data[b_kstr - 1] != b_0[b_kstr - 1]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (b_bool) {
      b_kstr = 1;
    } else {
      b_kstr = -1;
    }
  }

  InverseKinematic_emxFree_char_T(&switch_expression);
  switch (b_kstr) {
   case 0:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 1;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 0;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    poslim_data[0] = -3.1415926535897931;
    poslim_data[1] = 3.1415926535897931;
    iobj_1->VelocityNumber = 1.0;
    iobj_1->PositionNumber = 1.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 1.0;
    break;

   case 1:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 0;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 1;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    poslim_data[0] = -0.5;
    poslim_data[1] = 0.5;
    iobj_1->VelocityNumber = 1.0;
    iobj_1->PositionNumber = 1.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = 0;
    }

    poslim_data[0] = 0.0;
    poslim_data[1] = 0.0;
    iobj_1->VelocityNumber = 0.0;
    iobj_1->PositionNumber = 0.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 0.0;
    break;
  }

  b_kstr = iobj_1->MotionSubspace->size[0] * iobj_1->MotionSubspace->size[1];
  iobj_1->MotionSubspace->size[0] = 6;
  iobj_1->MotionSubspace->size[1] = 1;
  Invers_emxEnsureCapacity_real_T(iobj_1->MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    iobj_1->MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  b_kstr = iobj_1->PositionLimitsInternal->size[0] *
    iobj_1->PositionLimitsInternal->size[1];
  iobj_1->PositionLimitsInternal->size[0] = 1;
  iobj_1->PositionLimitsInternal->size[1] = 2;
  Invers_emxEnsureCapacity_real_T(iobj_1->PositionLimitsInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 2; b_kstr++) {
    iobj_1->PositionLimitsInternal->data[b_kstr] = poslim_data[b_kstr];
  }

  b_kstr = iobj_1->HomePositionInternal->size[0];
  iobj_1->HomePositionInternal->size[0] = 1;
  Invers_emxEnsureCapacity_real_T(iobj_1->HomePositionInternal, b_kstr);
  iobj_1->HomePositionInternal->data[0] = 0.0;
  obj->JointInternal = iobj_1;
  obj->Index = -1.0;
  obj->ParentIndex = -1.0;
  obj->MassInternal = 1.0;
  obj->CenterOfMassInternal[0] = 0.0;
  obj->CenterOfMassInternal[1] = 0.0;
  obj->CenterOfMassInternal[2] = 0.0;
  for (b_kstr = 0; b_kstr < 9; b_kstr++) {
    b_I[b_kstr] = 0;
  }

  b_I[0] = 1;
  b_I[4] = 1;
  b_I[8] = 1;
  for (b_kstr = 0; b_kstr < 9; b_kstr++) {
    obj->InertiaInternal[b_kstr] = b_I[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 36; b_kstr++) {
    msubspace_data[b_kstr] = 0;
  }

  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    msubspace_data[b_kstr + 6 * b_kstr] = 1;
  }

  for (b_kstr = 0; b_kstr < 36; b_kstr++) {
    obj->SpatialInertia[b_kstr] = msubspace_data[b_kstr];
  }

  obj->CollisionsInternal = Inver_CollisionSet_CollisionSet(iobj_0, 0.0);
  return b_obj;
}

t_robotics_manip_internal_Rig_T *InverseKinematics::
  Inver_RigidBody_RigidBody_nlyk3(t_robotics_manip_internal_Rig_T *obj,
  l_robotics_manip_internal_Col_T *iobj_0, c_rigidBodyJoint_InverseKinem_T
  *iobj_1)
{
  emxArray_char_T_InverseKinema_T *switch_expression;
  t_robotics_manip_internal_Rig_T *b_obj;
  static const char_T tmp[10] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '6' };

  static const int8_T tmp_0[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    1 };

  static const char_T tmp_1[14] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '6', '_', 'j', 'n', 't' };

  static const char_T tmp_2[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_3[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_4[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  int32_T exitg1;
  b_obj = obj;
  InverseKinematics_B.i8 = obj->NameInternal->size[0] * obj->NameInternal->size
    [1];
  obj->NameInternal->size[0] = 1;
  obj->NameInternal->size[1] = 10;
  Invers_emxEnsureCapacity_char_T(obj->NameInternal, InverseKinematics_B.i8);
  for (InverseKinematics_B.b_kstr_af = 0; InverseKinematics_B.b_kstr_af < 10;
       InverseKinematics_B.b_kstr_af++) {
    obj->NameInternal->data[InverseKinematics_B.b_kstr_af] =
      tmp[InverseKinematics_B.b_kstr_af];
  }

  iobj_1->InTree = false;
  for (InverseKinematics_B.b_kstr_af = 0; InverseKinematics_B.b_kstr_af < 16;
       InverseKinematics_B.b_kstr_af++) {
    iobj_1->JointToParentTransform[InverseKinematics_B.b_kstr_af] =
      tmp_0[InverseKinematics_B.b_kstr_af];
  }

  for (InverseKinematics_B.b_kstr_af = 0; InverseKinematics_B.b_kstr_af < 16;
       InverseKinematics_B.b_kstr_af++) {
    iobj_1->ChildToJointTransform[InverseKinematics_B.b_kstr_af] =
      tmp_0[InverseKinematics_B.b_kstr_af];
  }

  InverseKinematics_B.i8 = iobj_1->NameInternal->size[0] * iobj_1->
    NameInternal->size[1];
  iobj_1->NameInternal->size[0] = 1;
  iobj_1->NameInternal->size[1] = 14;
  Invers_emxEnsureCapacity_char_T(iobj_1->NameInternal, InverseKinematics_B.i8);
  for (InverseKinematics_B.b_kstr_af = 0; InverseKinematics_B.b_kstr_af < 14;
       InverseKinematics_B.b_kstr_af++) {
    iobj_1->NameInternal->data[InverseKinematics_B.b_kstr_af] =
      tmp_1[InverseKinematics_B.b_kstr_af];
  }

  InverseKinematics_B.i8 = iobj_1->Type->size[0] * iobj_1->Type->size[1];
  iobj_1->Type->size[0] = 1;
  iobj_1->Type->size[1] = 5;
  Invers_emxEnsureCapacity_char_T(iobj_1->Type, InverseKinematics_B.i8);
  for (InverseKinematics_B.b_kstr_af = 0; InverseKinematics_B.b_kstr_af < 5;
       InverseKinematics_B.b_kstr_af++) {
    iobj_1->Type->data[InverseKinematics_B.b_kstr_af] =
      tmp_2[InverseKinematics_B.b_kstr_af];
  }

  InverseKinematic_emxInit_char_T(&switch_expression, 2);
  InverseKinematics_B.i8 = switch_expression->size[0] * switch_expression->size
    [1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = iobj_1->Type->size[1];
  Invers_emxEnsureCapacity_char_T(switch_expression, InverseKinematics_B.i8);
  InverseKinematics_B.loop_ub_p = iobj_1->Type->size[1] - 1;
  for (InverseKinematics_B.b_kstr_af = 0; InverseKinematics_B.b_kstr_af <=
       InverseKinematics_B.loop_ub_p; InverseKinematics_B.b_kstr_af++) {
    InverseKinematics_B.i8 = InverseKinematics_B.b_kstr_af;
    switch_expression->data[InverseKinematics_B.i8] = iobj_1->Type->
      data[InverseKinematics_B.i8];
  }

  for (InverseKinematics_B.b_kstr_af = 0; InverseKinematics_B.b_kstr_af < 8;
       InverseKinematics_B.b_kstr_af++) {
    InverseKinematics_B.b_jz[InverseKinematics_B.b_kstr_af] =
      tmp_3[InverseKinematics_B.b_kstr_af];
  }

  InverseKinematics_B.b_bool_n = false;
  if (switch_expression->size[1] != 8) {
  } else {
    InverseKinematics_B.b_kstr_af = 1;
    do {
      exitg1 = 0;
      if (InverseKinematics_B.b_kstr_af - 1 < 8) {
        if (switch_expression->data[InverseKinematics_B.b_kstr_af - 1] !=
            InverseKinematics_B.b_jz[InverseKinematics_B.b_kstr_af - 1]) {
          exitg1 = 1;
        } else {
          InverseKinematics_B.b_kstr_af++;
        }
      } else {
        InverseKinematics_B.b_bool_n = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (InverseKinematics_B.b_bool_n) {
    InverseKinematics_B.b_kstr_af = 0;
  } else {
    for (InverseKinematics_B.b_kstr_af = 0; InverseKinematics_B.b_kstr_af < 9;
         InverseKinematics_B.b_kstr_af++) {
      InverseKinematics_B.b_e[InverseKinematics_B.b_kstr_af] =
        tmp_4[InverseKinematics_B.b_kstr_af];
    }

    InverseKinematics_B.b_bool_n = false;
    if (switch_expression->size[1] != 9) {
    } else {
      InverseKinematics_B.b_kstr_af = 1;
      do {
        exitg1 = 0;
        if (InverseKinematics_B.b_kstr_af - 1 < 9) {
          if (switch_expression->data[InverseKinematics_B.b_kstr_af - 1] !=
              InverseKinematics_B.b_e[InverseKinematics_B.b_kstr_af - 1]) {
            exitg1 = 1;
          } else {
            InverseKinematics_B.b_kstr_af++;
          }
        } else {
          InverseKinematics_B.b_bool_n = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (InverseKinematics_B.b_bool_n) {
      InverseKinematics_B.b_kstr_af = 1;
    } else {
      InverseKinematics_B.b_kstr_af = -1;
    }
  }

  InverseKinematic_emxFree_char_T(&switch_expression);
  switch (InverseKinematics_B.b_kstr_af) {
   case 0:
    InverseKinematics_B.iv4[0] = 0;
    InverseKinematics_B.iv4[1] = 0;
    InverseKinematics_B.iv4[2] = 1;
    InverseKinematics_B.iv4[3] = 0;
    InverseKinematics_B.iv4[4] = 0;
    InverseKinematics_B.iv4[5] = 0;
    for (InverseKinematics_B.b_kstr_af = 0; InverseKinematics_B.b_kstr_af < 6;
         InverseKinematics_B.b_kstr_af++) {
      InverseKinematics_B.msubspace_data_j[InverseKinematics_B.b_kstr_af] =
        InverseKinematics_B.iv4[InverseKinematics_B.b_kstr_af];
    }

    InverseKinematics_B.poslim_data_f[0] = -3.1415926535897931;
    InverseKinematics_B.poslim_data_f[1] = 3.1415926535897931;
    iobj_1->VelocityNumber = 1.0;
    iobj_1->PositionNumber = 1.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 1.0;
    break;

   case 1:
    InverseKinematics_B.iv4[0] = 0;
    InverseKinematics_B.iv4[1] = 0;
    InverseKinematics_B.iv4[2] = 0;
    InverseKinematics_B.iv4[3] = 0;
    InverseKinematics_B.iv4[4] = 0;
    InverseKinematics_B.iv4[5] = 1;
    for (InverseKinematics_B.b_kstr_af = 0; InverseKinematics_B.b_kstr_af < 6;
         InverseKinematics_B.b_kstr_af++) {
      InverseKinematics_B.msubspace_data_j[InverseKinematics_B.b_kstr_af] =
        InverseKinematics_B.iv4[InverseKinematics_B.b_kstr_af];
    }

    InverseKinematics_B.poslim_data_f[0] = -0.5;
    InverseKinematics_B.poslim_data_f[1] = 0.5;
    iobj_1->VelocityNumber = 1.0;
    iobj_1->PositionNumber = 1.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 1.0;
    break;

   default:
    for (InverseKinematics_B.b_kstr_af = 0; InverseKinematics_B.b_kstr_af < 6;
         InverseKinematics_B.b_kstr_af++) {
      InverseKinematics_B.msubspace_data_j[InverseKinematics_B.b_kstr_af] = 0;
    }

    InverseKinematics_B.poslim_data_f[0] = 0.0;
    InverseKinematics_B.poslim_data_f[1] = 0.0;
    iobj_1->VelocityNumber = 0.0;
    iobj_1->PositionNumber = 0.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 0.0;
    break;
  }

  InverseKinematics_B.i8 = iobj_1->MotionSubspace->size[0] *
    iobj_1->MotionSubspace->size[1];
  iobj_1->MotionSubspace->size[0] = 6;
  iobj_1->MotionSubspace->size[1] = 1;
  Invers_emxEnsureCapacity_real_T(iobj_1->MotionSubspace, InverseKinematics_B.i8);
  for (InverseKinematics_B.b_kstr_af = 0; InverseKinematics_B.b_kstr_af < 6;
       InverseKinematics_B.b_kstr_af++) {
    iobj_1->MotionSubspace->data[InverseKinematics_B.b_kstr_af] =
      InverseKinematics_B.msubspace_data_j[InverseKinematics_B.b_kstr_af];
  }

  InverseKinematics_B.i8 = iobj_1->PositionLimitsInternal->size[0] *
    iobj_1->PositionLimitsInternal->size[1];
  iobj_1->PositionLimitsInternal->size[0] = 1;
  iobj_1->PositionLimitsInternal->size[1] = 2;
  Invers_emxEnsureCapacity_real_T(iobj_1->PositionLimitsInternal,
    InverseKinematics_B.i8);
  for (InverseKinematics_B.b_kstr_af = 0; InverseKinematics_B.b_kstr_af < 2;
       InverseKinematics_B.b_kstr_af++) {
    iobj_1->PositionLimitsInternal->data[InverseKinematics_B.b_kstr_af] =
      InverseKinematics_B.poslim_data_f[InverseKinematics_B.b_kstr_af];
  }

  InverseKinematics_B.i8 = iobj_1->HomePositionInternal->size[0];
  iobj_1->HomePositionInternal->size[0] = 1;
  Invers_emxEnsureCapacity_real_T(iobj_1->HomePositionInternal,
    InverseKinematics_B.i8);
  iobj_1->HomePositionInternal->data[0] = 0.0;
  obj->JointInternal = iobj_1;
  obj->Index = -1.0;
  obj->ParentIndex = -1.0;
  obj->MassInternal = 1.0;
  obj->CenterOfMassInternal[0] = 0.0;
  obj->CenterOfMassInternal[1] = 0.0;
  obj->CenterOfMassInternal[2] = 0.0;
  for (InverseKinematics_B.b_kstr_af = 0; InverseKinematics_B.b_kstr_af < 9;
       InverseKinematics_B.b_kstr_af++) {
    InverseKinematics_B.b_I_d[InverseKinematics_B.b_kstr_af] = 0;
  }

  InverseKinematics_B.b_I_d[0] = 1;
  InverseKinematics_B.b_I_d[4] = 1;
  InverseKinematics_B.b_I_d[8] = 1;
  for (InverseKinematics_B.b_kstr_af = 0; InverseKinematics_B.b_kstr_af < 9;
       InverseKinematics_B.b_kstr_af++) {
    obj->InertiaInternal[InverseKinematics_B.b_kstr_af] =
      InverseKinematics_B.b_I_d[InverseKinematics_B.b_kstr_af];
  }

  for (InverseKinematics_B.b_kstr_af = 0; InverseKinematics_B.b_kstr_af < 36;
       InverseKinematics_B.b_kstr_af++) {
    InverseKinematics_B.msubspace_data_j[InverseKinematics_B.b_kstr_af] = 0;
  }

  for (InverseKinematics_B.b_kstr_af = 0; InverseKinematics_B.b_kstr_af < 6;
       InverseKinematics_B.b_kstr_af++) {
    InverseKinematics_B.msubspace_data_j[InverseKinematics_B.b_kstr_af + 6 *
      InverseKinematics_B.b_kstr_af] = 1;
  }

  for (InverseKinematics_B.b_kstr_af = 0; InverseKinematics_B.b_kstr_af < 36;
       InverseKinematics_B.b_kstr_af++) {
    obj->SpatialInertia[InverseKinematics_B.b_kstr_af] =
      InverseKinematics_B.msubspace_data_j[InverseKinematics_B.b_kstr_af];
  }

  obj->CollisionsInternal = Inver_CollisionSet_CollisionSet(iobj_0, 0.0);
  return b_obj;
}

t_robotics_manip_internal_Rig_T *InverseKinematics::
  Inve_RigidBody_RigidBody_nlyk3r(t_robotics_manip_internal_Rig_T *obj,
  l_robotics_manip_internal_Col_T *iobj_0, c_rigidBodyJoint_InverseKinem_T
  *iobj_1)
{
  emxArray_char_T_InverseKinema_T *switch_expression;
  t_robotics_manip_internal_Rig_T *b_obj;
  real_T poslim_data[12];
  int32_T b_kstr;
  int32_T loop_ub;
  char_T b_0[9];
  char_T b[8];
  int8_T msubspace_data[36];
  int8_T tmp[6];
  boolean_T b_bool;
  static const char_T tmp_0[5] = { 'B', 'o', 'd', 'y', '3' };

  static const real_T tmp_1[9] = { 0.004214759646674851, -3.0676517226065562E-8,
    2.1821908339314224E-8, -3.0676517226065562E-8, 0.00037231560073204209,
    -0.00079485808764448288, 2.1821908339314224E-8, -0.00079485808764448288,
    0.00396761410886888 };

  static const real_T tmp_2[36] = { 0.004214759646674851, -3.0676517226065562E-8,
    2.1821908339314224E-8, 0.0, 0.0065367341731804784, -0.029400227748421107,
    -3.0676517226065562E-8, 0.00037231560073204209, -0.00079485808764448288,
    -0.0065367341731804784, 0.0, 1.0391384923362643E-6, 2.1821908339314224E-8,
    -0.00079485808764448288, 0.00396761410886888, 0.029400227748421107,
    -1.0391384923362643E-6, 0.0, 0.0, -0.0065367341731804784,
    0.029400227748421107, 0.30177243138, 0.0, 0.0, 0.0065367341731804784, 0.0,
    -1.0391384923362643E-6, 0.0, 0.30177243138, 0.0, -0.029400227748421107,
    1.0391384923362643E-6, 0.0, 0.0, 0.0, 0.30177243138 };

  static const int8_T tmp_3[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    1 };

  static const char_T tmp_4[6] = { 'J', 'o', 'i', 'n', 't', '3' };

  static const char_T tmp_5[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_6[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const real_T tmp_7[16] = { 6.123233995736766E-17, -1.0, -0.0, 0.0, -1.0,
    -6.123233995736766E-17, 1.2246467991473532E-16, 0.0, -1.2246467991473532E-16,
    -7.498798913309288E-33, -1.0, 0.0, -3.5971225173156E-17, 0.16,
    -0.042000000000000058, 1.0 };

  static const real_T tmp_8[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  static const real_T tmp_9[36] = { 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  int32_T exitg1;
  b_obj = obj;
  b_kstr = obj->NameInternal->size[0] * obj->NameInternal->size[1];
  obj->NameInternal->size[0] = 1;
  obj->NameInternal->size[1] = 5;
  Invers_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp_0[b_kstr];
  }

  obj->ParentIndex = 2.0;
  obj->MassInternal = 0.30177243138;
  obj->CenterOfMassInternal[0] = -3.4434507074894226E-6;
  obj->CenterOfMassInternal[1] = -0.097425161118841724;
  obj->CenterOfMassInternal[2] = -0.021661137643647927;
  for (b_kstr = 0; b_kstr < 9; b_kstr++) {
    obj->InertiaInternal[b_kstr] = tmp_1[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 36; b_kstr++) {
    obj->SpatialInertia[b_kstr] = tmp_2[b_kstr];
  }

  iobj_1->InTree = false;
  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    iobj_1->JointToParentTransform[b_kstr] = tmp_3[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    iobj_1->ChildToJointTransform[b_kstr] = tmp_3[b_kstr];
  }

  b_kstr = iobj_1->NameInternal->size[0] * iobj_1->NameInternal->size[1];
  iobj_1->NameInternal->size[0] = 1;
  iobj_1->NameInternal->size[1] = 6;
  Invers_emxEnsureCapacity_char_T(iobj_1->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    iobj_1->NameInternal->data[b_kstr] = tmp_4[b_kstr];
  }

  b_kstr = iobj_1->Type->size[0] * iobj_1->Type->size[1];
  iobj_1->Type->size[0] = 1;
  iobj_1->Type->size[1] = 8;
  Invers_emxEnsureCapacity_char_T(iobj_1->Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    iobj_1->Type->data[b_kstr] = tmp_5[b_kstr];
  }

  InverseKinematic_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = iobj_1->Type->size[1];
  Invers_emxEnsureCapacity_char_T(switch_expression, b_kstr);
  loop_ub = iobj_1->Type->size[1] - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = iobj_1->Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp_5[b_kstr];
  }

  b_bool = false;
  if (switch_expression->size[1] != 8) {
  } else {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        if (switch_expression->data[b_kstr - 1] != b[b_kstr - 1]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (b_bool) {
    b_kstr = 0;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      b_0[b_kstr] = tmp_6[b_kstr];
    }

    b_bool = false;
    if (switch_expression->size[1] != 9) {
    } else {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          if (switch_expression->data[b_kstr - 1] != b_0[b_kstr - 1]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (b_bool) {
      b_kstr = 1;
    } else {
      b_kstr = -1;
    }
  }

  InverseKinematic_emxFree_char_T(&switch_expression);
  switch (b_kstr) {
   case 0:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 1;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 0;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    poslim_data[0] = -3.1415926535897931;
    poslim_data[1] = 3.1415926535897931;
    iobj_1->VelocityNumber = 1.0;
    iobj_1->PositionNumber = 1.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 1.0;
    break;

   case 1:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 0;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 1;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    poslim_data[0] = -0.5;
    poslim_data[1] = 0.5;
    iobj_1->VelocityNumber = 1.0;
    iobj_1->PositionNumber = 1.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = 0;
    }

    poslim_data[0] = 0.0;
    poslim_data[1] = 0.0;
    iobj_1->VelocityNumber = 0.0;
    iobj_1->PositionNumber = 0.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 0.0;
    break;
  }

  b_kstr = iobj_1->MotionSubspace->size[0] * iobj_1->MotionSubspace->size[1];
  iobj_1->MotionSubspace->size[0] = 6;
  iobj_1->MotionSubspace->size[1] = 1;
  Invers_emxEnsureCapacity_real_T(iobj_1->MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    iobj_1->MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  b_kstr = iobj_1->PositionLimitsInternal->size[0] *
    iobj_1->PositionLimitsInternal->size[1];
  iobj_1->PositionLimitsInternal->size[0] = 1;
  iobj_1->PositionLimitsInternal->size[1] = 2;
  Invers_emxEnsureCapacity_real_T(iobj_1->PositionLimitsInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 2; b_kstr++) {
    iobj_1->PositionLimitsInternal->data[b_kstr] = poslim_data[b_kstr];
  }

  b_kstr = iobj_1->HomePositionInternal->size[0];
  iobj_1->HomePositionInternal->size[0] = 1;
  Invers_emxEnsureCapacity_real_T(iobj_1->HomePositionInternal, b_kstr);
  iobj_1->HomePositionInternal->data[0] = 0.0;
  obj->JointInternal = iobj_1;
  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal->JointToParentTransform[b_kstr] = tmp_7[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal->ChildToJointTransform[b_kstr] = tmp_8[b_kstr];
  }

  b_kstr = obj->JointInternal->MotionSubspace->size[0] * obj->
    JointInternal->MotionSubspace->size[1];
  obj->JointInternal->MotionSubspace->size[0] = 6;
  obj->JointInternal->MotionSubspace->size[1] = 1;
  Invers_emxEnsureCapacity_real_T(obj->JointInternal->MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal->MotionSubspace->data[b_kstr] = tmp_9[b_kstr];
  }

  obj->JointInternal->InTree = true;
  b_kstr = obj->JointInternal->PositionLimitsInternal->size[0] *
    obj->JointInternal->PositionLimitsInternal->size[1];
  obj->JointInternal->PositionLimitsInternal->size[0] = 1;
  obj->JointInternal->PositionLimitsInternal->size[1] = 2;
  Invers_emxEnsureCapacity_real_T(obj->JointInternal->PositionLimitsInternal,
    b_kstr);
  obj->JointInternal->PositionLimitsInternal->data[0] = -1.5707963267948966;
  obj->JointInternal->PositionLimitsInternal->data[obj->
    JointInternal->PositionLimitsInternal->size[0]] = 0.3490658503988659;
  obj->JointInternal->JointAxisInternal[0] = 0.0;
  obj->JointInternal->JointAxisInternal[1] = 0.0;
  obj->JointInternal->JointAxisInternal[2] = 1.0;
  b_kstr = obj->JointInternal->HomePositionInternal->size[0];
  obj->JointInternal->HomePositionInternal->size[0] = 1;
  Invers_emxEnsureCapacity_real_T(obj->JointInternal->HomePositionInternal,
    b_kstr);
  obj->JointInternal->HomePositionInternal->data[0] = -2.2999999516049027E-8;
  obj->CollisionsInternal = Inver_CollisionSet_CollisionSet(iobj_0, 0.0);
  return b_obj;
}

t_robotics_manip_internal_Rig_T *InverseKinematics::
  Inv_RigidBody_RigidBody_nlyk3r3(t_robotics_manip_internal_Rig_T *obj,
  l_robotics_manip_internal_Col_T *iobj_0, c_rigidBodyJoint_InverseKinem_T
  *iobj_1)
{
  emxArray_char_T_InverseKinema_T *switch_expression;
  t_robotics_manip_internal_Rig_T *b_obj;
  real_T poslim_data[12];
  int32_T b_kstr;
  int32_T loop_ub;
  char_T b_0[9];
  char_T b[8];
  int8_T msubspace_data[36];
  int8_T tmp[6];
  boolean_T b_bool;
  static const char_T tmp_0[5] = { 'B', 'o', 'd', 'y', '4' };

  static const real_T tmp_1[9] = { 0.0025850396977390622, 8.0307310648899469E-5,
    1.1770697894580795E-5, 8.0307310648899469E-5, 0.00015209383720762718,
    -0.00023074969363202281, 1.1770697894580795E-5, -0.00023074969363202281,
    0.0025807399030087002 };

  static const real_T tmp_2[36] = { 0.0025850396977390622, 8.0307310648899469E-5,
    1.1770697894580795E-5, 0.0, -0.0029407217431669705, 0.021997815546504242,
    8.0307310648899469E-5, 0.00015209383720762718, -0.00023074969363202281,
    0.0029407217431669705, 0.0, 0.0011226397295655513, 1.1770697894580795E-5,
    -0.00023074969363202281, 0.0025807399030087002, -0.021997815546504242,
    -0.0011226397295655513, 0.0, 0.0, 0.0029407217431669705,
    -0.021997815546504242, 0.24806822598, 0.0, 0.0, -0.0029407217431669705, 0.0,
    -0.0011226397295655513, 0.0, 0.24806822598, 0.0, 0.021997815546504242,
    0.0011226397295655513, 0.0, 0.0, 0.0, 0.24806822598 };

  static const int8_T tmp_3[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    1 };

  static const char_T tmp_4[6] = { 'J', 'o', 'i', 'n', 't', '4' };

  static const char_T tmp_5[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_6[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const real_T tmp_7[16] = { 1.0, 0.0, -0.0, 0.0, 0.0, -1.0,
    1.2246467991473532E-16, 0.0, 0.0, -1.2246467991473532E-16, -1.0, 0.0,
    1.7725858487190743E-17, -0.15500000000000003, -0.014125000000000037, 1.0 };

  static const real_T tmp_8[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  static const real_T tmp_9[36] = { 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  int32_T exitg1;
  b_obj = obj;
  b_kstr = obj->NameInternal->size[0] * obj->NameInternal->size[1];
  obj->NameInternal->size[0] = 1;
  obj->NameInternal->size[1] = 5;
  Invers_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp_0[b_kstr];
  }

  obj->ParentIndex = 3.0;
  obj->MassInternal = 0.24806822598;
  obj->CenterOfMassInternal[0] = -0.0045255281087714224;
  obj->CenterOfMassInternal[1] = 0.088676473819253948;
  obj->CenterOfMassInternal[2] = 0.011854487738401694;
  for (b_kstr = 0; b_kstr < 9; b_kstr++) {
    obj->InertiaInternal[b_kstr] = tmp_1[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 36; b_kstr++) {
    obj->SpatialInertia[b_kstr] = tmp_2[b_kstr];
  }

  iobj_1->InTree = false;
  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    iobj_1->JointToParentTransform[b_kstr] = tmp_3[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    iobj_1->ChildToJointTransform[b_kstr] = tmp_3[b_kstr];
  }

  b_kstr = iobj_1->NameInternal->size[0] * iobj_1->NameInternal->size[1];
  iobj_1->NameInternal->size[0] = 1;
  iobj_1->NameInternal->size[1] = 6;
  Invers_emxEnsureCapacity_char_T(iobj_1->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    iobj_1->NameInternal->data[b_kstr] = tmp_4[b_kstr];
  }

  b_kstr = iobj_1->Type->size[0] * iobj_1->Type->size[1];
  iobj_1->Type->size[0] = 1;
  iobj_1->Type->size[1] = 8;
  Invers_emxEnsureCapacity_char_T(iobj_1->Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    iobj_1->Type->data[b_kstr] = tmp_5[b_kstr];
  }

  InverseKinematic_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = iobj_1->Type->size[1];
  Invers_emxEnsureCapacity_char_T(switch_expression, b_kstr);
  loop_ub = iobj_1->Type->size[1] - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = iobj_1->Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp_5[b_kstr];
  }

  b_bool = false;
  if (switch_expression->size[1] != 8) {
  } else {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        if (switch_expression->data[b_kstr - 1] != b[b_kstr - 1]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (b_bool) {
    b_kstr = 0;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      b_0[b_kstr] = tmp_6[b_kstr];
    }

    b_bool = false;
    if (switch_expression->size[1] != 9) {
    } else {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          if (switch_expression->data[b_kstr - 1] != b_0[b_kstr - 1]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (b_bool) {
      b_kstr = 1;
    } else {
      b_kstr = -1;
    }
  }

  InverseKinematic_emxFree_char_T(&switch_expression);
  switch (b_kstr) {
   case 0:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 1;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 0;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    poslim_data[0] = -3.1415926535897931;
    poslim_data[1] = 3.1415926535897931;
    iobj_1->VelocityNumber = 1.0;
    iobj_1->PositionNumber = 1.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 1.0;
    break;

   case 1:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 0;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 1;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    poslim_data[0] = -0.5;
    poslim_data[1] = 0.5;
    iobj_1->VelocityNumber = 1.0;
    iobj_1->PositionNumber = 1.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = 0;
    }

    poslim_data[0] = 0.0;
    poslim_data[1] = 0.0;
    iobj_1->VelocityNumber = 0.0;
    iobj_1->PositionNumber = 0.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 0.0;
    break;
  }

  b_kstr = iobj_1->MotionSubspace->size[0] * iobj_1->MotionSubspace->size[1];
  iobj_1->MotionSubspace->size[0] = 6;
  iobj_1->MotionSubspace->size[1] = 1;
  Invers_emxEnsureCapacity_real_T(iobj_1->MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    iobj_1->MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  b_kstr = iobj_1->PositionLimitsInternal->size[0] *
    iobj_1->PositionLimitsInternal->size[1];
  iobj_1->PositionLimitsInternal->size[0] = 1;
  iobj_1->PositionLimitsInternal->size[1] = 2;
  Invers_emxEnsureCapacity_real_T(iobj_1->PositionLimitsInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 2; b_kstr++) {
    iobj_1->PositionLimitsInternal->data[b_kstr] = poslim_data[b_kstr];
  }

  b_kstr = iobj_1->HomePositionInternal->size[0];
  iobj_1->HomePositionInternal->size[0] = 1;
  Invers_emxEnsureCapacity_real_T(iobj_1->HomePositionInternal, b_kstr);
  iobj_1->HomePositionInternal->data[0] = 0.0;
  obj->JointInternal = iobj_1;
  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal->JointToParentTransform[b_kstr] = tmp_7[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal->ChildToJointTransform[b_kstr] = tmp_8[b_kstr];
  }

  b_kstr = obj->JointInternal->MotionSubspace->size[0] * obj->
    JointInternal->MotionSubspace->size[1];
  obj->JointInternal->MotionSubspace->size[0] = 6;
  obj->JointInternal->MotionSubspace->size[1] = 1;
  Invers_emxEnsureCapacity_real_T(obj->JointInternal->MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal->MotionSubspace->data[b_kstr] = tmp_9[b_kstr];
  }

  obj->JointInternal->InTree = true;
  b_kstr = obj->JointInternal->PositionLimitsInternal->size[0] *
    obj->JointInternal->PositionLimitsInternal->size[1];
  obj->JointInternal->PositionLimitsInternal->size[0] = 1;
  obj->JointInternal->PositionLimitsInternal->size[1] = 2;
  Invers_emxEnsureCapacity_real_T(obj->JointInternal->PositionLimitsInternal,
    b_kstr);
  obj->JointInternal->PositionLimitsInternal->data[0] = -1.0471975511965976;
  obj->JointInternal->PositionLimitsInternal->data[obj->
    JointInternal->PositionLimitsInternal->size[0]] = 0.0;
  obj->JointInternal->JointAxisInternal[0] = 0.0;
  obj->JointInternal->JointAxisInternal[1] = 0.0;
  obj->JointInternal->JointAxisInternal[2] = 1.0;
  b_kstr = obj->JointInternal->HomePositionInternal->size[0];
  obj->JointInternal->HomePositionInternal->size[0] = 1;
  Invers_emxEnsureCapacity_real_T(obj->JointInternal->HomePositionInternal,
    b_kstr);
  obj->JointInternal->HomePositionInternal->data[0] = -0.52359877559829882;
  obj->CollisionsInternal = Inver_CollisionSet_CollisionSet(iobj_0, 0.0);
  return b_obj;
}

t_robotics_manip_internal_Rig_T *InverseKinematics::
  In_RigidBody_RigidBody_nlyk3r3m(t_robotics_manip_internal_Rig_T *obj,
  l_robotics_manip_internal_Col_T *iobj_0, c_rigidBodyJoint_InverseKinem_T
  *iobj_1)
{
  emxArray_char_T_InverseKinema_T *switch_expression;
  t_robotics_manip_internal_Rig_T *b_obj;
  real_T poslim_data[12];
  int32_T b_kstr;
  int32_T loop_ub;
  char_T b_0[9];
  char_T b[8];
  int8_T msubspace_data[36];
  int8_T tmp[6];
  boolean_T b_bool;
  static const char_T tmp_0[5] = { 'B', 'o', 'd', 'y', '5' };

  static const real_T tmp_1[36] = { 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0,
    -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, -0.0, 0.0, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  static const int8_T tmp_2[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    1 };

  static const char_T tmp_3[6] = { 'J', 'o', 'i', 'n', 't', '5' };

  static const char_T tmp_4[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_5[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_6[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const real_T tmp_7[16] = { 6.123233995736766E-17, 1.0, -0.0, 0.0, 1.0,
    -6.123233995736766E-17, 1.2246467991473532E-16, 0.0, 1.2246467991473532E-16,
    -7.498798913309288E-33, -1.0, 0.0, -0.0011000000000000749,
    0.23624000000000006, 0.010499999999999877, 1.0 };

  static const real_T tmp_8[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  int32_T exitg1;
  b_obj = obj;
  b_kstr = obj->NameInternal->size[0] * obj->NameInternal->size[1];
  obj->NameInternal->size[0] = 1;
  obj->NameInternal->size[1] = 5;
  Invers_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp_0[b_kstr];
  }

  obj->ParentIndex = 4.0;
  obj->MassInternal = 0.0;
  obj->CenterOfMassInternal[0] = -0.0;
  obj->CenterOfMassInternal[1] = -0.0;
  obj->CenterOfMassInternal[2] = -0.0;
  for (b_kstr = 0; b_kstr < 9; b_kstr++) {
    obj->InertiaInternal[b_kstr] = 0.0;
  }

  for (b_kstr = 0; b_kstr < 36; b_kstr++) {
    obj->SpatialInertia[b_kstr] = tmp_1[b_kstr];
  }

  iobj_1->InTree = false;
  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    iobj_1->JointToParentTransform[b_kstr] = tmp_2[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    iobj_1->ChildToJointTransform[b_kstr] = tmp_2[b_kstr];
  }

  b_kstr = iobj_1->NameInternal->size[0] * iobj_1->NameInternal->size[1];
  iobj_1->NameInternal->size[0] = 1;
  iobj_1->NameInternal->size[1] = 6;
  Invers_emxEnsureCapacity_char_T(iobj_1->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    iobj_1->NameInternal->data[b_kstr] = tmp_3[b_kstr];
  }

  b_kstr = iobj_1->Type->size[0] * iobj_1->Type->size[1];
  iobj_1->Type->size[0] = 1;
  iobj_1->Type->size[1] = 5;
  Invers_emxEnsureCapacity_char_T(iobj_1->Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    iobj_1->Type->data[b_kstr] = tmp_4[b_kstr];
  }

  InverseKinematic_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = iobj_1->Type->size[1];
  Invers_emxEnsureCapacity_char_T(switch_expression, b_kstr);
  loop_ub = iobj_1->Type->size[1] - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = iobj_1->Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp_5[b_kstr];
  }

  b_bool = false;
  if (switch_expression->size[1] != 8) {
  } else {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        if (switch_expression->data[b_kstr - 1] != b[b_kstr - 1]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (b_bool) {
    b_kstr = 0;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      b_0[b_kstr] = tmp_6[b_kstr];
    }

    b_bool = false;
    if (switch_expression->size[1] != 9) {
    } else {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          if (switch_expression->data[b_kstr - 1] != b_0[b_kstr - 1]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (b_bool) {
      b_kstr = 1;
    } else {
      b_kstr = -1;
    }
  }

  InverseKinematic_emxFree_char_T(&switch_expression);
  switch (b_kstr) {
   case 0:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 1;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 0;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    poslim_data[0] = -3.1415926535897931;
    poslim_data[1] = 3.1415926535897931;
    iobj_1->VelocityNumber = 1.0;
    iobj_1->PositionNumber = 1.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 1.0;
    break;

   case 1:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 0;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 1;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    poslim_data[0] = -0.5;
    poslim_data[1] = 0.5;
    iobj_1->VelocityNumber = 1.0;
    iobj_1->PositionNumber = 1.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = 0;
    }

    poslim_data[0] = 0.0;
    poslim_data[1] = 0.0;
    iobj_1->VelocityNumber = 0.0;
    iobj_1->PositionNumber = 0.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 0.0;
    break;
  }

  b_kstr = iobj_1->MotionSubspace->size[0] * iobj_1->MotionSubspace->size[1];
  iobj_1->MotionSubspace->size[0] = 6;
  iobj_1->MotionSubspace->size[1] = 1;
  Invers_emxEnsureCapacity_real_T(iobj_1->MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    iobj_1->MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  b_kstr = iobj_1->PositionLimitsInternal->size[0] *
    iobj_1->PositionLimitsInternal->size[1];
  iobj_1->PositionLimitsInternal->size[0] = 1;
  iobj_1->PositionLimitsInternal->size[1] = 2;
  Invers_emxEnsureCapacity_real_T(iobj_1->PositionLimitsInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 2; b_kstr++) {
    iobj_1->PositionLimitsInternal->data[b_kstr] = poslim_data[b_kstr];
  }

  b_kstr = iobj_1->HomePositionInternal->size[0];
  iobj_1->HomePositionInternal->size[0] = 1;
  Invers_emxEnsureCapacity_real_T(iobj_1->HomePositionInternal, b_kstr);
  iobj_1->HomePositionInternal->data[0] = 0.0;
  obj->JointInternal = iobj_1;
  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal->JointToParentTransform[b_kstr] = tmp_7[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal->ChildToJointTransform[b_kstr] = tmp_8[b_kstr];
  }

  b_kstr = obj->JointInternal->MotionSubspace->size[0] * obj->
    JointInternal->MotionSubspace->size[1];
  obj->JointInternal->MotionSubspace->size[0] = 6;
  obj->JointInternal->MotionSubspace->size[1] = 1;
  Invers_emxEnsureCapacity_real_T(obj->JointInternal->MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal->MotionSubspace->data[b_kstr] = 0.0;
  }

  obj->JointInternal->InTree = true;
  b_kstr = obj->JointInternal->PositionLimitsInternal->size[0] *
    obj->JointInternal->PositionLimitsInternal->size[1];
  obj->JointInternal->PositionLimitsInternal->size[0] = 1;
  obj->JointInternal->PositionLimitsInternal->size[1] = 2;
  Invers_emxEnsureCapacity_real_T(obj->JointInternal->PositionLimitsInternal,
    b_kstr);
  obj->JointInternal->PositionLimitsInternal->data[0] = 0.0;
  obj->JointInternal->PositionLimitsInternal->data[obj->
    JointInternal->PositionLimitsInternal->size[0]] = 0.0;
  obj->JointInternal->JointAxisInternal[0] = 0.0;
  obj->JointInternal->JointAxisInternal[1] = 0.0;
  obj->JointInternal->JointAxisInternal[2] = 0.0;
  b_kstr = obj->JointInternal->HomePositionInternal->size[0];
  obj->JointInternal->HomePositionInternal->size[0] = 1;
  Invers_emxEnsureCapacity_real_T(obj->JointInternal->HomePositionInternal,
    b_kstr);
  obj->JointInternal->HomePositionInternal->data[0] = 0.0;
  obj->CollisionsInternal = Inver_CollisionSet_CollisionSet(iobj_0, 0.0);
  return b_obj;
}

t_robotics_manip_internal_Rig_T *InverseKinematics::
  I_RigidBody_RigidBody_nlyk3r3my(t_robotics_manip_internal_Rig_T *obj,
  l_robotics_manip_internal_Col_T *iobj_0, c_rigidBodyJoint_InverseKinem_T
  *iobj_1)
{
  emxArray_char_T_InverseKinema_T *switch_expression;
  t_robotics_manip_internal_Rig_T *b_obj;
  real_T poslim_data[12];
  int32_T b_kstr;
  int32_T loop_ub;
  char_T b_0[9];
  char_T b[8];
  int8_T msubspace_data[36];
  int8_T tmp[6];
  boolean_T b_bool;
  static const char_T tmp_0[5] = { 'B', 'o', 'd', 'y', '6' };

  static const real_T tmp_1[36] = { 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0,
    -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, -0.0, 0.0, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  static const int8_T tmp_2[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    1 };

  static const char_T tmp_3[6] = { 'J', 'o', 'i', 'n', 't', '6' };

  static const char_T tmp_4[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_5[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_6[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const real_T tmp_7[16] = { 2.1073424178067666E-8, 0.0,
    0.99999999999999978, 0.0, 0.99999999999999978, 2.83276944882399E-16,
    -2.1073424178067666E-8, 0.0, -2.8327694488239893E-16, 1.0,
    5.9696152193738883E-24, 0.0, -0.055500000000000022, 0.08049999999999996,
    0.010499999999999971, 1.0 };

  static const real_T tmp_8[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  int32_T exitg1;
  b_obj = obj;
  b_kstr = obj->NameInternal->size[0] * obj->NameInternal->size[1];
  obj->NameInternal->size[0] = 1;
  obj->NameInternal->size[1] = 5;
  Invers_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp_0[b_kstr];
  }

  obj->ParentIndex = 4.0;
  obj->MassInternal = 0.0;
  obj->CenterOfMassInternal[0] = -0.0;
  obj->CenterOfMassInternal[1] = -0.0;
  obj->CenterOfMassInternal[2] = -0.0;
  for (b_kstr = 0; b_kstr < 9; b_kstr++) {
    obj->InertiaInternal[b_kstr] = 0.0;
  }

  for (b_kstr = 0; b_kstr < 36; b_kstr++) {
    obj->SpatialInertia[b_kstr] = tmp_1[b_kstr];
  }

  iobj_1->InTree = false;
  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    iobj_1->JointToParentTransform[b_kstr] = tmp_2[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    iobj_1->ChildToJointTransform[b_kstr] = tmp_2[b_kstr];
  }

  b_kstr = iobj_1->NameInternal->size[0] * iobj_1->NameInternal->size[1];
  iobj_1->NameInternal->size[0] = 1;
  iobj_1->NameInternal->size[1] = 6;
  Invers_emxEnsureCapacity_char_T(iobj_1->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    iobj_1->NameInternal->data[b_kstr] = tmp_3[b_kstr];
  }

  b_kstr = iobj_1->Type->size[0] * iobj_1->Type->size[1];
  iobj_1->Type->size[0] = 1;
  iobj_1->Type->size[1] = 5;
  Invers_emxEnsureCapacity_char_T(iobj_1->Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    iobj_1->Type->data[b_kstr] = tmp_4[b_kstr];
  }

  InverseKinematic_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = iobj_1->Type->size[1];
  Invers_emxEnsureCapacity_char_T(switch_expression, b_kstr);
  loop_ub = iobj_1->Type->size[1] - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = iobj_1->Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp_5[b_kstr];
  }

  b_bool = false;
  if (switch_expression->size[1] != 8) {
  } else {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        if (switch_expression->data[b_kstr - 1] != b[b_kstr - 1]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (b_bool) {
    b_kstr = 0;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      b_0[b_kstr] = tmp_6[b_kstr];
    }

    b_bool = false;
    if (switch_expression->size[1] != 9) {
    } else {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          if (switch_expression->data[b_kstr - 1] != b_0[b_kstr - 1]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (b_bool) {
      b_kstr = 1;
    } else {
      b_kstr = -1;
    }
  }

  InverseKinematic_emxFree_char_T(&switch_expression);
  switch (b_kstr) {
   case 0:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 1;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 0;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    poslim_data[0] = -3.1415926535897931;
    poslim_data[1] = 3.1415926535897931;
    iobj_1->VelocityNumber = 1.0;
    iobj_1->PositionNumber = 1.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 1.0;
    break;

   case 1:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 0;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 1;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    poslim_data[0] = -0.5;
    poslim_data[1] = 0.5;
    iobj_1->VelocityNumber = 1.0;
    iobj_1->PositionNumber = 1.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = 0;
    }

    poslim_data[0] = 0.0;
    poslim_data[1] = 0.0;
    iobj_1->VelocityNumber = 0.0;
    iobj_1->PositionNumber = 0.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 0.0;
    break;
  }

  b_kstr = iobj_1->MotionSubspace->size[0] * iobj_1->MotionSubspace->size[1];
  iobj_1->MotionSubspace->size[0] = 6;
  iobj_1->MotionSubspace->size[1] = 1;
  Invers_emxEnsureCapacity_real_T(iobj_1->MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    iobj_1->MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  b_kstr = iobj_1->PositionLimitsInternal->size[0] *
    iobj_1->PositionLimitsInternal->size[1];
  iobj_1->PositionLimitsInternal->size[0] = 1;
  iobj_1->PositionLimitsInternal->size[1] = 2;
  Invers_emxEnsureCapacity_real_T(iobj_1->PositionLimitsInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 2; b_kstr++) {
    iobj_1->PositionLimitsInternal->data[b_kstr] = poslim_data[b_kstr];
  }

  b_kstr = iobj_1->HomePositionInternal->size[0];
  iobj_1->HomePositionInternal->size[0] = 1;
  Invers_emxEnsureCapacity_real_T(iobj_1->HomePositionInternal, b_kstr);
  iobj_1->HomePositionInternal->data[0] = 0.0;
  obj->JointInternal = iobj_1;
  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal->JointToParentTransform[b_kstr] = tmp_7[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal->ChildToJointTransform[b_kstr] = tmp_8[b_kstr];
  }

  b_kstr = obj->JointInternal->MotionSubspace->size[0] * obj->
    JointInternal->MotionSubspace->size[1];
  obj->JointInternal->MotionSubspace->size[0] = 6;
  obj->JointInternal->MotionSubspace->size[1] = 1;
  Invers_emxEnsureCapacity_real_T(obj->JointInternal->MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal->MotionSubspace->data[b_kstr] = 0.0;
  }

  obj->JointInternal->InTree = true;
  b_kstr = obj->JointInternal->PositionLimitsInternal->size[0] *
    obj->JointInternal->PositionLimitsInternal->size[1];
  obj->JointInternal->PositionLimitsInternal->size[0] = 1;
  obj->JointInternal->PositionLimitsInternal->size[1] = 2;
  Invers_emxEnsureCapacity_real_T(obj->JointInternal->PositionLimitsInternal,
    b_kstr);
  obj->JointInternal->PositionLimitsInternal->data[0] = 0.0;
  obj->JointInternal->PositionLimitsInternal->data[obj->
    JointInternal->PositionLimitsInternal->size[0]] = 0.0;
  obj->JointInternal->JointAxisInternal[0] = 0.0;
  obj->JointInternal->JointAxisInternal[1] = 0.0;
  obj->JointInternal->JointAxisInternal[2] = 0.0;
  b_kstr = obj->JointInternal->HomePositionInternal->size[0];
  obj->JointInternal->HomePositionInternal->size[0] = 1;
  Invers_emxEnsureCapacity_real_T(obj->JointInternal->HomePositionInternal,
    b_kstr);
  obj->JointInternal->HomePositionInternal->data[0] = 0.0;
  obj->CollisionsInternal = Inver_CollisionSet_CollisionSet(iobj_0, 0.0);
  return b_obj;
}

c_rigidBodyJoint_InverseKinem_T *InverseKinematics::
  I_rigidBodyJoint_rigidBodyJoint(c_rigidBodyJoint_InverseKinem_T *obj, const
  emxArray_char_T_InverseKinema_T *jname)
{
  c_rigidBodyJoint_InverseKinem_T *b_obj;
  emxArray_char_T_InverseKinema_T *switch_expression;
  static const int8_T tmp[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1
  };

  static const char_T tmp_0[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_1[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_2[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  int32_T exitg1;
  obj->InTree = false;
  for (InverseKinematics_B.b_kstr_h3 = 0; InverseKinematics_B.b_kstr_h3 < 16;
       InverseKinematics_B.b_kstr_h3++) {
    obj->JointToParentTransform[InverseKinematics_B.b_kstr_h3] =
      tmp[InverseKinematics_B.b_kstr_h3];
  }

  for (InverseKinematics_B.b_kstr_h3 = 0; InverseKinematics_B.b_kstr_h3 < 16;
       InverseKinematics_B.b_kstr_h3++) {
    obj->ChildToJointTransform[InverseKinematics_B.b_kstr_h3] =
      tmp[InverseKinematics_B.b_kstr_h3];
  }

  b_obj = obj;
  InverseKinematics_B.i2 = obj->NameInternal->size[0] * obj->NameInternal->size
    [1];
  obj->NameInternal->size[0] = 1;
  obj->NameInternal->size[1] = jname->size[1];
  Invers_emxEnsureCapacity_char_T(obj->NameInternal, InverseKinematics_B.i2);
  InverseKinematics_B.loop_ub_cs = jname->size[1] - 1;
  for (InverseKinematics_B.b_kstr_h3 = 0; InverseKinematics_B.b_kstr_h3 <=
       InverseKinematics_B.loop_ub_cs; InverseKinematics_B.b_kstr_h3++) {
    InverseKinematics_B.i2 = InverseKinematics_B.b_kstr_h3;
    obj->NameInternal->data[InverseKinematics_B.i2] = jname->
      data[InverseKinematics_B.i2];
  }

  InverseKinematics_B.i2 = obj->Type->size[0] * obj->Type->size[1];
  obj->Type->size[0] = 1;
  obj->Type->size[1] = 5;
  Invers_emxEnsureCapacity_char_T(obj->Type, InverseKinematics_B.i2);
  for (InverseKinematics_B.b_kstr_h3 = 0; InverseKinematics_B.b_kstr_h3 < 5;
       InverseKinematics_B.b_kstr_h3++) {
    obj->Type->data[InverseKinematics_B.b_kstr_h3] =
      tmp_0[InverseKinematics_B.b_kstr_h3];
  }

  InverseKinematic_emxInit_char_T(&switch_expression, 2);
  InverseKinematics_B.i2 = switch_expression->size[0] * switch_expression->size
    [1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->Type->size[1];
  Invers_emxEnsureCapacity_char_T(switch_expression, InverseKinematics_B.i2);
  InverseKinematics_B.loop_ub_cs = obj->Type->size[1] - 1;
  for (InverseKinematics_B.b_kstr_h3 = 0; InverseKinematics_B.b_kstr_h3 <=
       InverseKinematics_B.loop_ub_cs; InverseKinematics_B.b_kstr_h3++) {
    InverseKinematics_B.i2 = InverseKinematics_B.b_kstr_h3;
    switch_expression->data[InverseKinematics_B.i2] = obj->Type->
      data[InverseKinematics_B.i2];
  }

  for (InverseKinematics_B.b_kstr_h3 = 0; InverseKinematics_B.b_kstr_h3 < 8;
       InverseKinematics_B.b_kstr_h3++) {
    InverseKinematics_B.b_a[InverseKinematics_B.b_kstr_h3] =
      tmp_1[InverseKinematics_B.b_kstr_h3];
  }

  InverseKinematics_B.b_bool_k = false;
  if (switch_expression->size[1] != 8) {
  } else {
    InverseKinematics_B.b_kstr_h3 = 1;
    do {
      exitg1 = 0;
      if (InverseKinematics_B.b_kstr_h3 - 1 < 8) {
        if (switch_expression->data[InverseKinematics_B.b_kstr_h3 - 1] !=
            InverseKinematics_B.b_a[InverseKinematics_B.b_kstr_h3 - 1]) {
          exitg1 = 1;
        } else {
          InverseKinematics_B.b_kstr_h3++;
        }
      } else {
        InverseKinematics_B.b_bool_k = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (InverseKinematics_B.b_bool_k) {
    InverseKinematics_B.b_kstr_h3 = 0;
  } else {
    for (InverseKinematics_B.b_kstr_h3 = 0; InverseKinematics_B.b_kstr_h3 < 9;
         InverseKinematics_B.b_kstr_h3++) {
      InverseKinematics_B.b_h[InverseKinematics_B.b_kstr_h3] =
        tmp_2[InverseKinematics_B.b_kstr_h3];
    }

    InverseKinematics_B.b_bool_k = false;
    if (switch_expression->size[1] != 9) {
    } else {
      InverseKinematics_B.b_kstr_h3 = 1;
      do {
        exitg1 = 0;
        if (InverseKinematics_B.b_kstr_h3 - 1 < 9) {
          if (switch_expression->data[InverseKinematics_B.b_kstr_h3 - 1] !=
              InverseKinematics_B.b_h[InverseKinematics_B.b_kstr_h3 - 1]) {
            exitg1 = 1;
          } else {
            InverseKinematics_B.b_kstr_h3++;
          }
        } else {
          InverseKinematics_B.b_bool_k = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (InverseKinematics_B.b_bool_k) {
      InverseKinematics_B.b_kstr_h3 = 1;
    } else {
      InverseKinematics_B.b_kstr_h3 = -1;
    }
  }

  InverseKinematic_emxFree_char_T(&switch_expression);
  switch (InverseKinematics_B.b_kstr_h3) {
   case 0:
    InverseKinematics_B.iv2[0] = 0;
    InverseKinematics_B.iv2[1] = 0;
    InverseKinematics_B.iv2[2] = 1;
    InverseKinematics_B.iv2[3] = 0;
    InverseKinematics_B.iv2[4] = 0;
    InverseKinematics_B.iv2[5] = 0;
    for (InverseKinematics_B.b_kstr_h3 = 0; InverseKinematics_B.b_kstr_h3 < 6;
         InverseKinematics_B.b_kstr_h3++) {
      InverseKinematics_B.msubspace_data_p[InverseKinematics_B.b_kstr_h3] =
        InverseKinematics_B.iv2[InverseKinematics_B.b_kstr_h3];
    }

    InverseKinematics_B.poslim_data_p[0] = -3.1415926535897931;
    InverseKinematics_B.poslim_data_p[1] = 3.1415926535897931;
    obj->VelocityNumber = 1.0;
    obj->PositionNumber = 1.0;
    obj->JointAxisInternal[0] = 0.0;
    obj->JointAxisInternal[1] = 0.0;
    obj->JointAxisInternal[2] = 1.0;
    break;

   case 1:
    InverseKinematics_B.iv2[0] = 0;
    InverseKinematics_B.iv2[1] = 0;
    InverseKinematics_B.iv2[2] = 0;
    InverseKinematics_B.iv2[3] = 0;
    InverseKinematics_B.iv2[4] = 0;
    InverseKinematics_B.iv2[5] = 1;
    for (InverseKinematics_B.b_kstr_h3 = 0; InverseKinematics_B.b_kstr_h3 < 6;
         InverseKinematics_B.b_kstr_h3++) {
      InverseKinematics_B.msubspace_data_p[InverseKinematics_B.b_kstr_h3] =
        InverseKinematics_B.iv2[InverseKinematics_B.b_kstr_h3];
    }

    InverseKinematics_B.poslim_data_p[0] = -0.5;
    InverseKinematics_B.poslim_data_p[1] = 0.5;
    obj->VelocityNumber = 1.0;
    obj->PositionNumber = 1.0;
    obj->JointAxisInternal[0] = 0.0;
    obj->JointAxisInternal[1] = 0.0;
    obj->JointAxisInternal[2] = 1.0;
    break;

   default:
    for (InverseKinematics_B.b_kstr_h3 = 0; InverseKinematics_B.b_kstr_h3 < 6;
         InverseKinematics_B.b_kstr_h3++) {
      InverseKinematics_B.msubspace_data_p[InverseKinematics_B.b_kstr_h3] = 0;
    }

    InverseKinematics_B.poslim_data_p[0] = 0.0;
    InverseKinematics_B.poslim_data_p[1] = 0.0;
    obj->VelocityNumber = 0.0;
    obj->PositionNumber = 0.0;
    obj->JointAxisInternal[0] = 0.0;
    obj->JointAxisInternal[1] = 0.0;
    obj->JointAxisInternal[2] = 0.0;
    break;
  }

  InverseKinematics_B.i2 = obj->MotionSubspace->size[0] * obj->
    MotionSubspace->size[1];
  obj->MotionSubspace->size[0] = 6;
  obj->MotionSubspace->size[1] = 1;
  Invers_emxEnsureCapacity_real_T(obj->MotionSubspace, InverseKinematics_B.i2);
  for (InverseKinematics_B.b_kstr_h3 = 0; InverseKinematics_B.b_kstr_h3 < 6;
       InverseKinematics_B.b_kstr_h3++) {
    obj->MotionSubspace->data[InverseKinematics_B.b_kstr_h3] =
      InverseKinematics_B.msubspace_data_p[InverseKinematics_B.b_kstr_h3];
  }

  InverseKinematics_B.i2 = obj->PositionLimitsInternal->size[0] *
    obj->PositionLimitsInternal->size[1];
  obj->PositionLimitsInternal->size[0] = 1;
  obj->PositionLimitsInternal->size[1] = 2;
  Invers_emxEnsureCapacity_real_T(obj->PositionLimitsInternal,
    InverseKinematics_B.i2);
  for (InverseKinematics_B.b_kstr_h3 = 0; InverseKinematics_B.b_kstr_h3 < 2;
       InverseKinematics_B.b_kstr_h3++) {
    obj->PositionLimitsInternal->data[InverseKinematics_B.b_kstr_h3] =
      InverseKinematics_B.poslim_data_p[InverseKinematics_B.b_kstr_h3];
  }

  InverseKinematics_B.i2 = obj->HomePositionInternal->size[0];
  obj->HomePositionInternal->size[0] = 1;
  Invers_emxEnsureCapacity_real_T(obj->HomePositionInternal,
    InverseKinematics_B.i2);
  obj->HomePositionInternal->data[0] = 0.0;
  return b_obj;
}

u_robotics_manip_internal_Rig_T *InverseKinematics::
  Inv_RigidBodyTree_RigidBodyTree(u_robotics_manip_internal_Rig_T *obj)
{
  c_rigidBodyJoint_InverseKinem_T *iobj_1;
  emxArray_char_T_InverseKinema_T *jname;
  l_robotics_manip_internal_Col_T *iobj_0;
  t_robotics_manip_internal_Rig_T *iobj_2;
  u_robotics_manip_internal_Rig_T *b_obj;
  static const char_T tmp[5] = { 'B', 'o', 'd', 'y', '1' };

  static const real_T tmp_0[9] = { 0.0037363941296150649, -2.3660843211813849E-5,
    0.00073505788570426926, -2.3660843211813849E-5, 0.0039968621121805256,
    5.1910119838100993E-5, 0.00073505788570426926, 5.1910119838100993E-5,
    0.00070548220277786677 };

  static const real_T tmp_1[36] = { 0.0037363941296150649,
    -2.3660843211813849E-5, 0.00073505788570426926, 0.0, 0.030307931352254961,
    0.00043638286578455231, -2.3660843211813849E-5, 0.0039968621121805256,
    5.1910119838100993E-5, -0.030307931352254961, 0.0, -0.006598642785242542,
    0.00073505788570426926, 5.1910119838100993E-5, 0.00070548220277786677,
    -0.00043638286578455231, 0.006598642785242542, 0.0, 0.0,
    -0.030307931352254961, -0.00043638286578455231, 0.42596811552, 0.0, 0.0,
    0.030307931352254961, 0.0, 0.006598642785242542, 0.0, 0.42596811552, 0.0,
    0.00043638286578455231, -0.006598642785242542, 0.0, 0.0, 0.0, 0.42596811552
  };

  static const int8_T tmp_2[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    1 };

  static const char_T tmp_3[6] = { 'J', 'o', 'i', 'n', 't', '1' };

  static const char_T tmp_4[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_5[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const real_T tmp_6[16] = { 1.0, 0.0, -0.0, 0.0, 0.0, -1.0,
    1.2246467991473532E-16, 0.0, 0.0, -1.2246467991473532E-16, -1.0, 0.0,
    0.060164888, 0.059081195000000031, 0.069308911, 1.0 };

  static const real_T tmp_7[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  static const real_T tmp_8[36] = { 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  static const char_T tmp_9[5] = { 'B', 'o', 'd', 'y', '2' };

  static const real_T tmp_a[9] = { 0.0044702657880398922, 7.792531032363201E-12,
    -1.5591817920808506E-12, 7.792531032363201E-12, 0.00048451650591669959,
    0.00058763430807934293, -1.5591817920808506E-12, 0.00058763430807934293,
    0.0041701532821232018 };

  static const real_T tmp_b[36] = { 0.0044702657880398922, 7.792531032363201E-12,
    -1.5591817920808506E-12, 0.0, 0.0060878808750251465, 0.030426215636229687,
    7.792531032363201E-12, 0.00048451650591669959, 0.00058763430807934293,
    -0.0060878808750251465, 0.0, 9.30825333481194E-11, -1.5591817920808506E-12,
    0.00058763430807934293, 0.0041701532821232018, -0.030426215636229687,
    -9.30825333481194E-11, 0.0, 0.0, -0.0060878808750251465,
    -0.030426215636229687, 0.36344406661, 0.0, 0.0, 0.0060878808750251465, 0.0,
    -9.30825333481194E-11, 0.0, 0.36344406661, 0.0, 0.030426215636229687,
    9.30825333481194E-11, 0.0, 0.0, 0.0, 0.36344406661 };

  static const char_T tmp_c[6] = { 'J', 'o', 'i', 'n', 't', '2' };

  static const real_T tmp_d[16] = { -1.0, 1.2246467991473532E-16, -0.0, 0.0,
    -7.498798913309288E-33, -6.123233995736766E-17, -1.0, 0.0,
    -1.2246467991473532E-16, -1.0, 6.123233995736766E-17, 0.0,
    -1.3877787807814457E-17, -0.018000000000000044, -0.11600000000000002, 1.0 };

  int32_T exitg1;
  b_obj = obj;
  obj->NumBodies = 6.0;
  iobj_0 = &obj->_pobj0[0];
  iobj_1 = &obj->_pobj1[0];
  iobj_2 = &obj->_pobj2[0];
  obj->Bodies[0] = InverseKine_RigidBody_RigidBody(&(&obj->_pobj2[0])[0],
    &iobj_0[0], &iobj_1[0]);
  obj->Bodies[1] = InverseKi_RigidBody_RigidBody_n(&iobj_2[1], &iobj_0[1],
    &iobj_1[1]);
  obj->Bodies[2] = InverseK_RigidBody_RigidBody_nl(&iobj_2[2], &iobj_0[2],
    &iobj_1[2]);
  obj->Bodies[3] = Inverse_RigidBody_RigidBody_nly(&iobj_2[3], &iobj_0[3],
    &iobj_1[3]);
  obj->Bodies[4] = Invers_RigidBody_RigidBody_nlyk(&iobj_2[4], &iobj_0[4],
    &iobj_1[4]);
  obj->Bodies[5] = Inver_RigidBody_RigidBody_nlyk3(&iobj_2[5], &iobj_0[5],
    &iobj_1[5]);
  iobj_2 = &obj->_pobj2[6];
  iobj_0 = &obj->_pobj0[6];
  InverseKinematics_B.i7 = iobj_2->NameInternal->size[0] * iobj_2->
    NameInternal->size[1];
  iobj_2->NameInternal->size[0] = 1;
  iobj_2->NameInternal->size[1] = 5;
  Invers_emxEnsureCapacity_char_T(iobj_2->NameInternal, InverseKinematics_B.i7);
  for (InverseKinematics_B.b_kstr_a = 0; InverseKinematics_B.b_kstr_a < 5;
       InverseKinematics_B.b_kstr_a++) {
    iobj_2->NameInternal->data[InverseKinematics_B.b_kstr_a] =
      tmp[InverseKinematics_B.b_kstr_a];
  }

  iobj_2->ParentIndex = 0.0;
  iobj_2->MassInternal = 0.42596811552;
  iobj_2->CenterOfMassInternal[0] = 0.015490931233637657;
  iobj_2->CenterOfMassInternal[1] = 0.0010244496005336889;
  iobj_2->CenterOfMassInternal[2] = -0.071150704120792219;
  for (InverseKinematics_B.b_kstr_a = 0; InverseKinematics_B.b_kstr_a < 9;
       InverseKinematics_B.b_kstr_a++) {
    iobj_2->InertiaInternal[InverseKinematics_B.b_kstr_a] =
      tmp_0[InverseKinematics_B.b_kstr_a];
  }

  for (InverseKinematics_B.b_kstr_a = 0; InverseKinematics_B.b_kstr_a < 36;
       InverseKinematics_B.b_kstr_a++) {
    iobj_2->SpatialInertia[InverseKinematics_B.b_kstr_a] =
      tmp_1[InverseKinematics_B.b_kstr_a];
  }

  obj->_pobj1[6].InTree = false;
  for (InverseKinematics_B.b_kstr_a = 0; InverseKinematics_B.b_kstr_a < 16;
       InverseKinematics_B.b_kstr_a++) {
    obj->_pobj1[6].JointToParentTransform[InverseKinematics_B.b_kstr_a] =
      tmp_2[InverseKinematics_B.b_kstr_a];
  }

  for (InverseKinematics_B.b_kstr_a = 0; InverseKinematics_B.b_kstr_a < 16;
       InverseKinematics_B.b_kstr_a++) {
    obj->_pobj1[6].ChildToJointTransform[InverseKinematics_B.b_kstr_a] =
      tmp_2[InverseKinematics_B.b_kstr_a];
  }

  InverseKinematics_B.i7 = obj->_pobj1[6].NameInternal->size[0] * obj->_pobj1[6]
    .NameInternal->size[1];
  obj->_pobj1[6].NameInternal->size[0] = 1;
  obj->_pobj1[6].NameInternal->size[1] = 6;
  Invers_emxEnsureCapacity_char_T(obj->_pobj1[6].NameInternal,
    InverseKinematics_B.i7);
  for (InverseKinematics_B.b_kstr_a = 0; InverseKinematics_B.b_kstr_a < 6;
       InverseKinematics_B.b_kstr_a++) {
    obj->_pobj1[6].NameInternal->data[InverseKinematics_B.b_kstr_a] =
      tmp_3[InverseKinematics_B.b_kstr_a];
  }

  InverseKinematics_B.i7 = obj->_pobj1[6].Type->size[0] * obj->_pobj1[6]
    .Type->size[1];
  obj->_pobj1[6].Type->size[0] = 1;
  obj->_pobj1[6].Type->size[1] = 8;
  Invers_emxEnsureCapacity_char_T(obj->_pobj1[6].Type, InverseKinematics_B.i7);
  for (InverseKinematics_B.b_kstr_a = 0; InverseKinematics_B.b_kstr_a < 8;
       InverseKinematics_B.b_kstr_a++) {
    obj->_pobj1[6].Type->data[InverseKinematics_B.b_kstr_a] =
      tmp_4[InverseKinematics_B.b_kstr_a];
  }

  InverseKinematic_emxInit_char_T(&jname, 2);
  InverseKinematics_B.i7 = jname->size[0] * jname->size[1];
  jname->size[0] = 1;
  jname->size[1] = obj->_pobj1[6].Type->size[1];
  Invers_emxEnsureCapacity_char_T(jname, InverseKinematics_B.i7);
  InverseKinematics_B.loop_ub_d = obj->_pobj1[6].Type->size[1] - 1;
  for (InverseKinematics_B.b_kstr_a = 0; InverseKinematics_B.b_kstr_a <=
       InverseKinematics_B.loop_ub_d; InverseKinematics_B.b_kstr_a++) {
    InverseKinematics_B.i7 = InverseKinematics_B.b_kstr_a;
    jname->data[InverseKinematics_B.i7] = obj->_pobj1[6].Type->
      data[InverseKinematics_B.i7];
  }

  for (InverseKinematics_B.b_kstr_a = 0; InverseKinematics_B.b_kstr_a < 8;
       InverseKinematics_B.b_kstr_a++) {
    InverseKinematics_B.b_ju[InverseKinematics_B.b_kstr_a] =
      tmp_4[InverseKinematics_B.b_kstr_a];
  }

  InverseKinematics_B.b_bool_c = false;
  if (jname->size[1] != 8) {
  } else {
    InverseKinematics_B.b_kstr_a = 1;
    do {
      exitg1 = 0;
      if (InverseKinematics_B.b_kstr_a - 1 < 8) {
        if (jname->data[InverseKinematics_B.b_kstr_a - 1] !=
            InverseKinematics_B.b_ju[InverseKinematics_B.b_kstr_a - 1]) {
          exitg1 = 1;
        } else {
          InverseKinematics_B.b_kstr_a++;
        }
      } else {
        InverseKinematics_B.b_bool_c = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (InverseKinematics_B.b_bool_c) {
    InverseKinematics_B.b_kstr_a = 0;
  } else {
    for (InverseKinematics_B.b_kstr_a = 0; InverseKinematics_B.b_kstr_a < 9;
         InverseKinematics_B.b_kstr_a++) {
      InverseKinematics_B.b_b[InverseKinematics_B.b_kstr_a] =
        tmp_5[InverseKinematics_B.b_kstr_a];
    }

    InverseKinematics_B.b_bool_c = false;
    if (jname->size[1] != 9) {
    } else {
      InverseKinematics_B.b_kstr_a = 1;
      do {
        exitg1 = 0;
        if (InverseKinematics_B.b_kstr_a - 1 < 9) {
          if (jname->data[InverseKinematics_B.b_kstr_a - 1] !=
              InverseKinematics_B.b_b[InverseKinematics_B.b_kstr_a - 1]) {
            exitg1 = 1;
          } else {
            InverseKinematics_B.b_kstr_a++;
          }
        } else {
          InverseKinematics_B.b_bool_c = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (InverseKinematics_B.b_bool_c) {
      InverseKinematics_B.b_kstr_a = 1;
    } else {
      InverseKinematics_B.b_kstr_a = -1;
    }
  }

  switch (InverseKinematics_B.b_kstr_a) {
   case 0:
    InverseKinematics_B.iv3[0] = 0;
    InverseKinematics_B.iv3[1] = 0;
    InverseKinematics_B.iv3[2] = 1;
    InverseKinematics_B.iv3[3] = 0;
    InverseKinematics_B.iv3[4] = 0;
    InverseKinematics_B.iv3[5] = 0;
    for (InverseKinematics_B.b_kstr_a = 0; InverseKinematics_B.b_kstr_a < 6;
         InverseKinematics_B.b_kstr_a++) {
      InverseKinematics_B.msubspace_data_l[InverseKinematics_B.b_kstr_a] =
        InverseKinematics_B.iv3[InverseKinematics_B.b_kstr_a];
    }

    InverseKinematics_B.poslim_data_c[0] = -3.1415926535897931;
    InverseKinematics_B.poslim_data_c[1] = 3.1415926535897931;
    obj->_pobj1[6].VelocityNumber = 1.0;
    obj->_pobj1[6].PositionNumber = 1.0;
    obj->_pobj1[6].JointAxisInternal[0] = 0.0;
    obj->_pobj1[6].JointAxisInternal[1] = 0.0;
    obj->_pobj1[6].JointAxisInternal[2] = 1.0;
    break;

   case 1:
    InverseKinematics_B.iv3[0] = 0;
    InverseKinematics_B.iv3[1] = 0;
    InverseKinematics_B.iv3[2] = 0;
    InverseKinematics_B.iv3[3] = 0;
    InverseKinematics_B.iv3[4] = 0;
    InverseKinematics_B.iv3[5] = 1;
    for (InverseKinematics_B.b_kstr_a = 0; InverseKinematics_B.b_kstr_a < 6;
         InverseKinematics_B.b_kstr_a++) {
      InverseKinematics_B.msubspace_data_l[InverseKinematics_B.b_kstr_a] =
        InverseKinematics_B.iv3[InverseKinematics_B.b_kstr_a];
    }

    InverseKinematics_B.poslim_data_c[0] = -0.5;
    InverseKinematics_B.poslim_data_c[1] = 0.5;
    obj->_pobj1[6].VelocityNumber = 1.0;
    obj->_pobj1[6].PositionNumber = 1.0;
    obj->_pobj1[6].JointAxisInternal[0] = 0.0;
    obj->_pobj1[6].JointAxisInternal[1] = 0.0;
    obj->_pobj1[6].JointAxisInternal[2] = 1.0;
    break;

   default:
    for (InverseKinematics_B.b_kstr_a = 0; InverseKinematics_B.b_kstr_a < 6;
         InverseKinematics_B.b_kstr_a++) {
      InverseKinematics_B.msubspace_data_l[InverseKinematics_B.b_kstr_a] = 0;
    }

    InverseKinematics_B.poslim_data_c[0] = 0.0;
    InverseKinematics_B.poslim_data_c[1] = 0.0;
    obj->_pobj1[6].VelocityNumber = 0.0;
    obj->_pobj1[6].PositionNumber = 0.0;
    obj->_pobj1[6].JointAxisInternal[0] = 0.0;
    obj->_pobj1[6].JointAxisInternal[1] = 0.0;
    obj->_pobj1[6].JointAxisInternal[2] = 0.0;
    break;
  }

  InverseKinematics_B.i7 = obj->_pobj1[6].MotionSubspace->size[0] * obj->_pobj1
    [6].MotionSubspace->size[1];
  obj->_pobj1[6].MotionSubspace->size[0] = 6;
  obj->_pobj1[6].MotionSubspace->size[1] = 1;
  Invers_emxEnsureCapacity_real_T(obj->_pobj1[6].MotionSubspace,
    InverseKinematics_B.i7);
  for (InverseKinematics_B.b_kstr_a = 0; InverseKinematics_B.b_kstr_a < 6;
       InverseKinematics_B.b_kstr_a++) {
    obj->_pobj1[6].MotionSubspace->data[InverseKinematics_B.b_kstr_a] =
      InverseKinematics_B.msubspace_data_l[InverseKinematics_B.b_kstr_a];
  }

  InverseKinematics_B.i7 = obj->_pobj1[6].PositionLimitsInternal->size[0] *
    obj->_pobj1[6].PositionLimitsInternal->size[1];
  obj->_pobj1[6].PositionLimitsInternal->size[0] = 1;
  obj->_pobj1[6].PositionLimitsInternal->size[1] = 2;
  Invers_emxEnsureCapacity_real_T(obj->_pobj1[6].PositionLimitsInternal,
    InverseKinematics_B.i7);
  for (InverseKinematics_B.b_kstr_a = 0; InverseKinematics_B.b_kstr_a < 2;
       InverseKinematics_B.b_kstr_a++) {
    obj->_pobj1[6].PositionLimitsInternal->data[InverseKinematics_B.b_kstr_a] =
      InverseKinematics_B.poslim_data_c[InverseKinematics_B.b_kstr_a];
  }

  InverseKinematics_B.i7 = obj->_pobj1[6].HomePositionInternal->size[0];
  obj->_pobj1[6].HomePositionInternal->size[0] = 1;
  Invers_emxEnsureCapacity_real_T(obj->_pobj1[6].HomePositionInternal,
    InverseKinematics_B.i7);
  obj->_pobj1[6].HomePositionInternal->data[0] = 0.0;
  iobj_2->JointInternal = &obj->_pobj1[6];
  for (InverseKinematics_B.b_kstr_a = 0; InverseKinematics_B.b_kstr_a < 16;
       InverseKinematics_B.b_kstr_a++) {
    iobj_2->JointInternal->JointToParentTransform[InverseKinematics_B.b_kstr_a] =
      tmp_6[InverseKinematics_B.b_kstr_a];
  }

  for (InverseKinematics_B.b_kstr_a = 0; InverseKinematics_B.b_kstr_a < 16;
       InverseKinematics_B.b_kstr_a++) {
    iobj_2->JointInternal->ChildToJointTransform[InverseKinematics_B.b_kstr_a] =
      tmp_7[InverseKinematics_B.b_kstr_a];
  }

  InverseKinematics_B.i7 = iobj_2->JointInternal->MotionSubspace->size[0] *
    iobj_2->JointInternal->MotionSubspace->size[1];
  iobj_2->JointInternal->MotionSubspace->size[0] = 6;
  iobj_2->JointInternal->MotionSubspace->size[1] = 1;
  Invers_emxEnsureCapacity_real_T(iobj_2->JointInternal->MotionSubspace,
    InverseKinematics_B.i7);
  for (InverseKinematics_B.b_kstr_a = 0; InverseKinematics_B.b_kstr_a < 6;
       InverseKinematics_B.b_kstr_a++) {
    iobj_2->JointInternal->MotionSubspace->data[InverseKinematics_B.b_kstr_a] =
      tmp_8[InverseKinematics_B.b_kstr_a];
  }

  iobj_2->JointInternal->InTree = true;
  InverseKinematics_B.i7 = iobj_2->JointInternal->PositionLimitsInternal->size[0]
    * iobj_2->JointInternal->PositionLimitsInternal->size[1];
  iobj_2->JointInternal->PositionLimitsInternal->size[0] = 1;
  iobj_2->JointInternal->PositionLimitsInternal->size[1] = 2;
  Invers_emxEnsureCapacity_real_T(iobj_2->JointInternal->PositionLimitsInternal,
    InverseKinematics_B.i7);
  iobj_2->JointInternal->PositionLimitsInternal->data[0] = -0.52359877559829882;
  iobj_2->JointInternal->PositionLimitsInternal->data[iobj_2->
    JointInternal->PositionLimitsInternal->size[0]] = 2.0071286397934789;
  iobj_2->JointInternal->JointAxisInternal[0] = 0.0;
  iobj_2->JointInternal->JointAxisInternal[1] = 0.0;
  iobj_2->JointInternal->JointAxisInternal[2] = 1.0;
  InverseKinematics_B.i7 = iobj_2->JointInternal->HomePositionInternal->size[0];
  iobj_2->JointInternal->HomePositionInternal->size[0] = 1;
  Invers_emxEnsureCapacity_real_T(iobj_2->JointInternal->HomePositionInternal,
    InverseKinematics_B.i7);
  iobj_2->JointInternal->HomePositionInternal->data[0] = 0.0;
  iobj_2->CollisionsInternal = Inver_CollisionSet_CollisionSet(iobj_0, 0.0);
  obj->Bodies[0] = iobj_2;
  obj->Bodies[0]->Index = 1.0;
  iobj_2 = &obj->_pobj2[7];
  iobj_0 = &obj->_pobj0[7];
  InverseKinematics_B.i7 = iobj_2->NameInternal->size[0] * iobj_2->
    NameInternal->size[1];
  iobj_2->NameInternal->size[0] = 1;
  iobj_2->NameInternal->size[1] = 5;
  Invers_emxEnsureCapacity_char_T(iobj_2->NameInternal, InverseKinematics_B.i7);
  for (InverseKinematics_B.b_kstr_a = 0; InverseKinematics_B.b_kstr_a < 5;
       InverseKinematics_B.b_kstr_a++) {
    iobj_2->NameInternal->data[InverseKinematics_B.b_kstr_a] =
      tmp_9[InverseKinematics_B.b_kstr_a];
  }

  iobj_2->ParentIndex = 1.0;
  iobj_2->MassInternal = 0.36344406661;
  iobj_2->CenterOfMassInternal[0] = -2.5611240325462029E-10;
  iobj_2->CenterOfMassInternal[1] = 0.083716363621032963;
  iobj_2->CenterOfMassInternal[2] = -0.016750530368563846;
  for (InverseKinematics_B.b_kstr_a = 0; InverseKinematics_B.b_kstr_a < 9;
       InverseKinematics_B.b_kstr_a++) {
    iobj_2->InertiaInternal[InverseKinematics_B.b_kstr_a] =
      tmp_a[InverseKinematics_B.b_kstr_a];
  }

  for (InverseKinematics_B.b_kstr_a = 0; InverseKinematics_B.b_kstr_a < 36;
       InverseKinematics_B.b_kstr_a++) {
    iobj_2->SpatialInertia[InverseKinematics_B.b_kstr_a] =
      tmp_b[InverseKinematics_B.b_kstr_a];
  }

  obj->_pobj1[7].InTree = false;
  for (InverseKinematics_B.b_kstr_a = 0; InverseKinematics_B.b_kstr_a < 16;
       InverseKinematics_B.b_kstr_a++) {
    obj->_pobj1[7].JointToParentTransform[InverseKinematics_B.b_kstr_a] =
      tmp_2[InverseKinematics_B.b_kstr_a];
  }

  for (InverseKinematics_B.b_kstr_a = 0; InverseKinematics_B.b_kstr_a < 16;
       InverseKinematics_B.b_kstr_a++) {
    obj->_pobj1[7].ChildToJointTransform[InverseKinematics_B.b_kstr_a] =
      tmp_2[InverseKinematics_B.b_kstr_a];
  }

  InverseKinematics_B.i7 = obj->_pobj1[7].NameInternal->size[0] * obj->_pobj1[7]
    .NameInternal->size[1];
  obj->_pobj1[7].NameInternal->size[0] = 1;
  obj->_pobj1[7].NameInternal->size[1] = 6;
  Invers_emxEnsureCapacity_char_T(obj->_pobj1[7].NameInternal,
    InverseKinematics_B.i7);
  for (InverseKinematics_B.b_kstr_a = 0; InverseKinematics_B.b_kstr_a < 6;
       InverseKinematics_B.b_kstr_a++) {
    obj->_pobj1[7].NameInternal->data[InverseKinematics_B.b_kstr_a] =
      tmp_c[InverseKinematics_B.b_kstr_a];
  }

  InverseKinematics_B.i7 = obj->_pobj1[7].Type->size[0] * obj->_pobj1[7]
    .Type->size[1];
  obj->_pobj1[7].Type->size[0] = 1;
  obj->_pobj1[7].Type->size[1] = 8;
  Invers_emxEnsureCapacity_char_T(obj->_pobj1[7].Type, InverseKinematics_B.i7);
  for (InverseKinematics_B.b_kstr_a = 0; InverseKinematics_B.b_kstr_a < 8;
       InverseKinematics_B.b_kstr_a++) {
    obj->_pobj1[7].Type->data[InverseKinematics_B.b_kstr_a] =
      tmp_4[InverseKinematics_B.b_kstr_a];
  }

  InverseKinematics_B.i7 = jname->size[0] * jname->size[1];
  jname->size[0] = 1;
  jname->size[1] = obj->_pobj1[7].Type->size[1];
  Invers_emxEnsureCapacity_char_T(jname, InverseKinematics_B.i7);
  InverseKinematics_B.loop_ub_d = obj->_pobj1[7].Type->size[1] - 1;
  for (InverseKinematics_B.b_kstr_a = 0; InverseKinematics_B.b_kstr_a <=
       InverseKinematics_B.loop_ub_d; InverseKinematics_B.b_kstr_a++) {
    InverseKinematics_B.i7 = InverseKinematics_B.b_kstr_a;
    jname->data[InverseKinematics_B.i7] = obj->_pobj1[7].Type->
      data[InverseKinematics_B.i7];
  }

  InverseKinematics_B.b_bool_c = false;
  if (jname->size[1] != 8) {
  } else {
    InverseKinematics_B.b_kstr_a = 1;
    do {
      exitg1 = 0;
      if (InverseKinematics_B.b_kstr_a - 1 < 8) {
        if (jname->data[InverseKinematics_B.b_kstr_a - 1] !=
            InverseKinematics_B.b_ju[InverseKinematics_B.b_kstr_a - 1]) {
          exitg1 = 1;
        } else {
          InverseKinematics_B.b_kstr_a++;
        }
      } else {
        InverseKinematics_B.b_bool_c = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (InverseKinematics_B.b_bool_c) {
    InverseKinematics_B.b_kstr_a = 0;
  } else {
    for (InverseKinematics_B.b_kstr_a = 0; InverseKinematics_B.b_kstr_a < 9;
         InverseKinematics_B.b_kstr_a++) {
      InverseKinematics_B.b_b[InverseKinematics_B.b_kstr_a] =
        tmp_5[InverseKinematics_B.b_kstr_a];
    }

    InverseKinematics_B.b_bool_c = false;
    if (jname->size[1] != 9) {
    } else {
      InverseKinematics_B.b_kstr_a = 1;
      do {
        exitg1 = 0;
        if (InverseKinematics_B.b_kstr_a - 1 < 9) {
          if (jname->data[InverseKinematics_B.b_kstr_a - 1] !=
              InverseKinematics_B.b_b[InverseKinematics_B.b_kstr_a - 1]) {
            exitg1 = 1;
          } else {
            InverseKinematics_B.b_kstr_a++;
          }
        } else {
          InverseKinematics_B.b_bool_c = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (InverseKinematics_B.b_bool_c) {
      InverseKinematics_B.b_kstr_a = 1;
    } else {
      InverseKinematics_B.b_kstr_a = -1;
    }
  }

  switch (InverseKinematics_B.b_kstr_a) {
   case 0:
    InverseKinematics_B.iv3[0] = 0;
    InverseKinematics_B.iv3[1] = 0;
    InverseKinematics_B.iv3[2] = 1;
    InverseKinematics_B.iv3[3] = 0;
    InverseKinematics_B.iv3[4] = 0;
    InverseKinematics_B.iv3[5] = 0;
    for (InverseKinematics_B.b_kstr_a = 0; InverseKinematics_B.b_kstr_a < 6;
         InverseKinematics_B.b_kstr_a++) {
      InverseKinematics_B.msubspace_data_l[InverseKinematics_B.b_kstr_a] =
        InverseKinematics_B.iv3[InverseKinematics_B.b_kstr_a];
    }

    InverseKinematics_B.poslim_data_c[0] = -3.1415926535897931;
    InverseKinematics_B.poslim_data_c[1] = 3.1415926535897931;
    obj->_pobj1[7].VelocityNumber = 1.0;
    obj->_pobj1[7].PositionNumber = 1.0;
    obj->_pobj1[7].JointAxisInternal[0] = 0.0;
    obj->_pobj1[7].JointAxisInternal[1] = 0.0;
    obj->_pobj1[7].JointAxisInternal[2] = 1.0;
    break;

   case 1:
    InverseKinematics_B.iv3[0] = 0;
    InverseKinematics_B.iv3[1] = 0;
    InverseKinematics_B.iv3[2] = 0;
    InverseKinematics_B.iv3[3] = 0;
    InverseKinematics_B.iv3[4] = 0;
    InverseKinematics_B.iv3[5] = 1;
    for (InverseKinematics_B.b_kstr_a = 0; InverseKinematics_B.b_kstr_a < 6;
         InverseKinematics_B.b_kstr_a++) {
      InverseKinematics_B.msubspace_data_l[InverseKinematics_B.b_kstr_a] =
        InverseKinematics_B.iv3[InverseKinematics_B.b_kstr_a];
    }

    InverseKinematics_B.poslim_data_c[0] = -0.5;
    InverseKinematics_B.poslim_data_c[1] = 0.5;
    obj->_pobj1[7].VelocityNumber = 1.0;
    obj->_pobj1[7].PositionNumber = 1.0;
    obj->_pobj1[7].JointAxisInternal[0] = 0.0;
    obj->_pobj1[7].JointAxisInternal[1] = 0.0;
    obj->_pobj1[7].JointAxisInternal[2] = 1.0;
    break;

   default:
    for (InverseKinematics_B.b_kstr_a = 0; InverseKinematics_B.b_kstr_a < 6;
         InverseKinematics_B.b_kstr_a++) {
      InverseKinematics_B.msubspace_data_l[InverseKinematics_B.b_kstr_a] = 0;
    }

    InverseKinematics_B.poslim_data_c[0] = 0.0;
    InverseKinematics_B.poslim_data_c[1] = 0.0;
    obj->_pobj1[7].VelocityNumber = 0.0;
    obj->_pobj1[7].PositionNumber = 0.0;
    obj->_pobj1[7].JointAxisInternal[0] = 0.0;
    obj->_pobj1[7].JointAxisInternal[1] = 0.0;
    obj->_pobj1[7].JointAxisInternal[2] = 0.0;
    break;
  }

  InverseKinematics_B.i7 = obj->_pobj1[7].MotionSubspace->size[0] * obj->_pobj1
    [7].MotionSubspace->size[1];
  obj->_pobj1[7].MotionSubspace->size[0] = 6;
  obj->_pobj1[7].MotionSubspace->size[1] = 1;
  Invers_emxEnsureCapacity_real_T(obj->_pobj1[7].MotionSubspace,
    InverseKinematics_B.i7);
  for (InverseKinematics_B.b_kstr_a = 0; InverseKinematics_B.b_kstr_a < 6;
       InverseKinematics_B.b_kstr_a++) {
    obj->_pobj1[7].MotionSubspace->data[InverseKinematics_B.b_kstr_a] =
      InverseKinematics_B.msubspace_data_l[InverseKinematics_B.b_kstr_a];
  }

  InverseKinematics_B.i7 = obj->_pobj1[7].PositionLimitsInternal->size[0] *
    obj->_pobj1[7].PositionLimitsInternal->size[1];
  obj->_pobj1[7].PositionLimitsInternal->size[0] = 1;
  obj->_pobj1[7].PositionLimitsInternal->size[1] = 2;
  Invers_emxEnsureCapacity_real_T(obj->_pobj1[7].PositionLimitsInternal,
    InverseKinematics_B.i7);
  for (InverseKinematics_B.b_kstr_a = 0; InverseKinematics_B.b_kstr_a < 2;
       InverseKinematics_B.b_kstr_a++) {
    obj->_pobj1[7].PositionLimitsInternal->data[InverseKinematics_B.b_kstr_a] =
      InverseKinematics_B.poslim_data_c[InverseKinematics_B.b_kstr_a];
  }

  InverseKinematics_B.i7 = obj->_pobj1[7].HomePositionInternal->size[0];
  obj->_pobj1[7].HomePositionInternal->size[0] = 1;
  Invers_emxEnsureCapacity_real_T(obj->_pobj1[7].HomePositionInternal,
    InverseKinematics_B.i7);
  obj->_pobj1[7].HomePositionInternal->data[0] = 0.0;
  iobj_2->JointInternal = &obj->_pobj1[7];
  for (InverseKinematics_B.b_kstr_a = 0; InverseKinematics_B.b_kstr_a < 16;
       InverseKinematics_B.b_kstr_a++) {
    iobj_2->JointInternal->JointToParentTransform[InverseKinematics_B.b_kstr_a] =
      tmp_d[InverseKinematics_B.b_kstr_a];
  }

  for (InverseKinematics_B.b_kstr_a = 0; InverseKinematics_B.b_kstr_a < 16;
       InverseKinematics_B.b_kstr_a++) {
    iobj_2->JointInternal->ChildToJointTransform[InverseKinematics_B.b_kstr_a] =
      tmp_7[InverseKinematics_B.b_kstr_a];
  }

  InverseKinematics_B.i7 = iobj_2->JointInternal->MotionSubspace->size[0] *
    iobj_2->JointInternal->MotionSubspace->size[1];
  iobj_2->JointInternal->MotionSubspace->size[0] = 6;
  iobj_2->JointInternal->MotionSubspace->size[1] = 1;
  Invers_emxEnsureCapacity_real_T(iobj_2->JointInternal->MotionSubspace,
    InverseKinematics_B.i7);
  for (InverseKinematics_B.b_kstr_a = 0; InverseKinematics_B.b_kstr_a < 6;
       InverseKinematics_B.b_kstr_a++) {
    iobj_2->JointInternal->MotionSubspace->data[InverseKinematics_B.b_kstr_a] =
      tmp_8[InverseKinematics_B.b_kstr_a];
  }

  iobj_2->JointInternal->InTree = true;
  InverseKinematics_B.i7 = iobj_2->JointInternal->PositionLimitsInternal->size[0]
    * iobj_2->JointInternal->PositionLimitsInternal->size[1];
  iobj_2->JointInternal->PositionLimitsInternal->size[0] = 1;
  iobj_2->JointInternal->PositionLimitsInternal->size[1] = 2;
  Invers_emxEnsureCapacity_real_T(iobj_2->JointInternal->PositionLimitsInternal,
    InverseKinematics_B.i7);
  iobj_2->JointInternal->PositionLimitsInternal->data[0] = -1.5707963267948966;
  iobj_2->JointInternal->PositionLimitsInternal->data[iobj_2->
    JointInternal->PositionLimitsInternal->size[0]] = 0.78539816339744828;
  iobj_2->JointInternal->JointAxisInternal[0] = 0.0;
  iobj_2->JointInternal->JointAxisInternal[1] = 0.0;
  iobj_2->JointInternal->JointAxisInternal[2] = 1.0;
  InverseKinematics_B.i7 = iobj_2->JointInternal->HomePositionInternal->size[0];
  iobj_2->JointInternal->HomePositionInternal->size[0] = 1;
  Invers_emxEnsureCapacity_real_T(iobj_2->JointInternal->HomePositionInternal,
    InverseKinematics_B.i7);
  iobj_2->JointInternal->HomePositionInternal->data[0] = 0.0;
  iobj_2->CollisionsInternal = Inver_CollisionSet_CollisionSet(iobj_0, 0.0);
  obj->Bodies[1] = iobj_2;
  obj->Bodies[1]->Index = 2.0;
  obj->Bodies[2] = Inve_RigidBody_RigidBody_nlyk3r(&obj->_pobj2[8], &obj->
    _pobj0[8], &obj->_pobj1[8]);
  obj->Bodies[2]->Index = 3.0;
  obj->Bodies[3] = Inv_RigidBody_RigidBody_nlyk3r3(&obj->_pobj2[9], &obj->
    _pobj0[9], &obj->_pobj1[9]);
  obj->Bodies[3]->Index = 4.0;
  obj->Bodies[4] = In_RigidBody_RigidBody_nlyk3r3m(&obj->_pobj2[10],
    &obj->_pobj0[10], &obj->_pobj1[10]);
  obj->Bodies[4]->Index = 5.0;
  obj->Bodies[5] = I_RigidBody_RigidBody_nlyk3r3my(&obj->_pobj2[11],
    &obj->_pobj0[11], &obj->_pobj1[11]);
  obj->Bodies[5]->Index = 6.0;
  obj->Gravity[0] = 0.0;
  obj->Gravity[1] = 0.0;
  obj->Gravity[2] = -9.80665;
  iobj_2 = &obj->Base;
  iobj_0 = &obj->_pobj0[12];
  InverseKinematics_B.i7 = iobj_2->NameInternal->size[0] * iobj_2->
    NameInternal->size[1];
  iobj_2->NameInternal->size[0] = 1;
  iobj_2->NameInternal->size[1] = 4;
  Invers_emxEnsureCapacity_char_T(iobj_2->NameInternal, InverseKinematics_B.i7);
  iobj_2->NameInternal->data[0] = 'B';
  iobj_2->NameInternal->data[1] = 'a';
  iobj_2->NameInternal->data[2] = 's';
  iobj_2->NameInternal->data[3] = 'e';
  iobj_2->ParentIndex = -1.0;
  iobj_2->MassInternal = 0.0;
  iobj_2->CenterOfMassInternal[0] = 0.0;
  iobj_2->CenterOfMassInternal[1] = 0.0;
  iobj_2->CenterOfMassInternal[2] = 0.0;
  for (InverseKinematics_B.b_kstr_a = 0; InverseKinematics_B.b_kstr_a < 9;
       InverseKinematics_B.b_kstr_a++) {
    iobj_2->InertiaInternal[InverseKinematics_B.b_kstr_a] = 0.0;
  }

  for (InverseKinematics_B.b_kstr_a = 0; InverseKinematics_B.b_kstr_a < 36;
       InverseKinematics_B.b_kstr_a++) {
    iobj_2->SpatialInertia[InverseKinematics_B.b_kstr_a] = 0.0;
  }

  InverseKinematics_B.i7 = jname->size[0] * jname->size[1];
  jname->size[0] = 1;
  jname->size[1] = iobj_2->NameInternal->size[1] + 4;
  Invers_emxEnsureCapacity_char_T(jname, InverseKinematics_B.i7);
  InverseKinematics_B.loop_ub_d = iobj_2->NameInternal->size[1];
  for (InverseKinematics_B.b_kstr_a = 0; InverseKinematics_B.b_kstr_a <
       InverseKinematics_B.loop_ub_d; InverseKinematics_B.b_kstr_a++) {
    InverseKinematics_B.i7 = InverseKinematics_B.b_kstr_a;
    jname->data[InverseKinematics_B.i7] = iobj_2->NameInternal->
      data[InverseKinematics_B.i7];
  }

  jname->data[InverseKinematics_B.loop_ub_d] = '_';
  jname->data[InverseKinematics_B.loop_ub_d + 1] = 'j';
  jname->data[InverseKinematics_B.loop_ub_d + 2] = 'n';
  jname->data[InverseKinematics_B.loop_ub_d + 3] = 't';
  iobj_2->JointInternal = I_rigidBodyJoint_rigidBodyJoint(&obj->_pobj1[12],
    jname);
  iobj_2->CollisionsInternal = Inver_CollisionSet_CollisionSet(iobj_0, 0.0);
  obj->Base.Index = 0.0;
  InverseKinematic_emxFree_char_T(&jname);
  return b_obj;
}

void InverseKinematics::Inverse_genrand_uint32_vector_n(uint32_T mt[625],
  uint32_T u[2])
{
  for (InverseKinematics_B.b_j_o = 0; InverseKinematics_B.b_j_o < 2;
       InverseKinematics_B.b_j_o++) {
    InverseKinematics_B.mti = mt[624] + 1U;
    if (mt[624] + 1U >= 625U) {
      for (InverseKinematics_B.b_kk = 0; InverseKinematics_B.b_kk < 227;
           InverseKinematics_B.b_kk++) {
        InverseKinematics_B.y_k = (mt[InverseKinematics_B.b_kk + 1] &
          2147483647U) | (mt[InverseKinematics_B.b_kk] & 2147483648U);
        if ((InverseKinematics_B.y_k & 1U) == 0U) {
          InverseKinematics_B.y_k >>= 1U;
        } else {
          InverseKinematics_B.y_k = InverseKinematics_B.y_k >> 1U ^ 2567483615U;
        }

        mt[InverseKinematics_B.b_kk] = mt[InverseKinematics_B.b_kk + 397] ^
          InverseKinematics_B.y_k;
      }

      for (InverseKinematics_B.b_kk = 0; InverseKinematics_B.b_kk < 396;
           InverseKinematics_B.b_kk++) {
        InverseKinematics_B.y_k = (mt[InverseKinematics_B.b_kk + 227] &
          2147483648U) | (mt[InverseKinematics_B.b_kk + 228] & 2147483647U);
        if ((InverseKinematics_B.y_k & 1U) == 0U) {
          InverseKinematics_B.y_k >>= 1U;
        } else {
          InverseKinematics_B.y_k = InverseKinematics_B.y_k >> 1U ^ 2567483615U;
        }

        mt[InverseKinematics_B.b_kk + 227] = mt[InverseKinematics_B.b_kk] ^
          InverseKinematics_B.y_k;
      }

      InverseKinematics_B.y_k = (mt[623] & 2147483648U) | (mt[0] & 2147483647U);
      if ((InverseKinematics_B.y_k & 1U) == 0U) {
        InverseKinematics_B.y_k >>= 1U;
      } else {
        InverseKinematics_B.y_k = InverseKinematics_B.y_k >> 1U ^ 2567483615U;
      }

      mt[623] = mt[396] ^ InverseKinematics_B.y_k;
      InverseKinematics_B.mti = 1U;
    }

    InverseKinematics_B.y_k = mt[static_cast<int32_T>(InverseKinematics_B.mti) -
      1];
    mt[624] = InverseKinematics_B.mti;
    InverseKinematics_B.y_k ^= InverseKinematics_B.y_k >> 11U;
    InverseKinematics_B.y_k ^= InverseKinematics_B.y_k << 7U & 2636928640U;
    InverseKinematics_B.y_k ^= InverseKinematics_B.y_k << 15U & 4022730752U;
    u[InverseKinematics_B.b_j_o] = InverseKinematics_B.y_k >> 18U ^
      InverseKinematics_B.y_k;
  }
}

boolean_T InverseKinematics::InverseKinematic_is_valid_state(const uint32_T mt
  [625])
{
  boolean_T isvalid;
  if ((mt[624] >= 1U) && (mt[624] < 625U)) {
    isvalid = true;
  } else {
    isvalid = false;
  }

  if (isvalid) {
    int32_T k;
    boolean_T exitg1;
    isvalid = false;
    k = 0;
    exitg1 = false;
    while ((!exitg1) && (k + 1 < 625)) {
      if (mt[k] == 0U) {
        k++;
      } else {
        isvalid = true;
        exitg1 = true;
      }
    }
  }

  return isvalid;
}

void InverseKinematics::InverseKinematics_rand(real_T r[5])
{
  uint32_T b_u[2];
  for (int32_T b_k = 0; b_k < 5; b_k++) {
    real_T b_r;

    /* ========================= COPYRIGHT NOTICE ============================ */
    /*  This is a uniform (0,1) pseudorandom number generator based on:        */
    /*                                                                         */
    /*  A C-program for MT19937, with initialization improved 2002/1/26.       */
    /*  Coded by Takuji Nishimura and Makoto Matsumoto.                        */
    /*                                                                         */
    /*  Copyright (C) 1997 - 2002, Makoto Matsumoto and Takuji Nishimura,      */
    /*  All rights reserved.                                                   */
    /*                                                                         */
    /*  Redistribution and use in source and binary forms, with or without     */
    /*  modification, are permitted provided that the following conditions     */
    /*  are met:                                                               */
    /*                                                                         */
    /*    1. Redistributions of source code must retain the above copyright    */
    /*       notice, this list of conditions and the following disclaimer.     */
    /*                                                                         */
    /*    2. Redistributions in binary form must reproduce the above copyright */
    /*       notice, this list of conditions and the following disclaimer      */
    /*       in the documentation and/or other materials provided with the     */
    /*       distribution.                                                     */
    /*                                                                         */
    /*    3. The names of its contributors may not be used to endorse or       */
    /*       promote products derived from this software without specific      */
    /*       prior written permission.                                         */
    /*                                                                         */
    /*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS    */
    /*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT      */
    /*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR  */
    /*  A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT  */
    /*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,  */
    /*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT       */
    /*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,  */
    /*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY  */
    /*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT    */
    /*  (INCLUDING  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE */
    /*  OF THIS  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.  */
    /*                                                                         */
    /* =============================   END   ================================= */
    int32_T exitg1;
    do {
      exitg1 = 0;
      Inverse_genrand_uint32_vector_n(InverseKinematics_DW.state_c, b_u);
      b_r = (static_cast<real_T>(b_u[0] >> 5U) * 6.7108864E+7 +
             static_cast<real_T>(b_u[1] >> 6U)) * 1.1102230246251565E-16;
      if (b_r == 0.0) {
        if (!InverseKinematic_is_valid_state(InverseKinematics_DW.state_c)) {
          InverseKinematics_DW.state_c[0] = 5489U;
          InverseKinematics_DW.state_c[624] = 624U;
        }
      } else {
        exitg1 = 1;
      }
    } while (exitg1 == 0);

    r[b_k] = b_r;
  }
}

boolean_T InverseKinematics::InverseKinematics_strcmp(const
  emxArray_char_T_InverseKinema_T *a, const emxArray_char_T_InverseKinema_T *b)
{
  boolean_T b_bool;
  b_bool = false;
  InverseKinematics_B.d_i = (a->size[1] == 0);
  if (InverseKinematics_B.d_i && (b->size[1] == 0)) {
    b_bool = true;
  } else if (a->size[1] != b->size[1]) {
  } else {
    InverseKinematics_B.b_kstr_o = 1;
    int32_T exitg1;
    do {
      exitg1 = 0;
      if (InverseKinematics_B.b_kstr_o - 1 <= b->size[1] - 1) {
        InverseKinematics_B.i9 = InverseKinematics_B.b_kstr_o - 1;
        if (a->data[InverseKinematics_B.i9] != b->data[InverseKinematics_B.i9])
        {
          exitg1 = 1;
        } else {
          InverseKinematics_B.b_kstr_o++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  return b_bool;
}

real_T InverseKinematics::RigidBodyTree_findBodyIndexByNa
  (v_robotics_manip_internal_Rig_T *obj, const emxArray_char_T_InverseKinema_T
   *bodyname)
{
  emxArray_char_T_InverseKinema_T *bname;
  t_robotics_manip_internal_Rig_T *obj_0;
  real_T bid;
  InverseKinematic_emxInit_char_T(&bname, 2);
  bid = -1.0;
  InverseKinematics_B.i5 = bname->size[0] * bname->size[1];
  bname->size[0] = 1;
  bname->size[1] = obj->Base.NameInternal->size[1];
  Invers_emxEnsureCapacity_char_T(bname, InverseKinematics_B.i5);
  InverseKinematics_B.loop_ub_e = obj->Base.NameInternal->size[1] - 1;
  for (InverseKinematics_B.i5 = 0; InverseKinematics_B.i5 <=
       InverseKinematics_B.loop_ub_e; InverseKinematics_B.i5++) {
    InverseKinematics_B.i6 = InverseKinematics_B.i5;
    bname->data[InverseKinematics_B.i6] = obj->Base.NameInternal->
      data[InverseKinematics_B.i6];
  }

  if (InverseKinematics_strcmp(bname, bodyname)) {
    bid = 0.0;
  } else {
    boolean_T exitg1;
    InverseKinematics_B.b_c = obj->NumBodies;
    InverseKinematics_B.b_i_h = 0;
    exitg1 = false;
    while ((!exitg1) && (InverseKinematics_B.b_i_h <= static_cast<int32_T>
                         (InverseKinematics_B.b_c) - 1)) {
      obj_0 = obj->Bodies[InverseKinematics_B.b_i_h];
      InverseKinematics_B.i5 = bname->size[0] * bname->size[1];
      bname->size[0] = 1;
      bname->size[1] = obj_0->NameInternal->size[1];
      Invers_emxEnsureCapacity_char_T(bname, InverseKinematics_B.i5);
      InverseKinematics_B.loop_ub_e = obj_0->NameInternal->size[1] - 1;
      for (InverseKinematics_B.i5 = 0; InverseKinematics_B.i5 <=
           InverseKinematics_B.loop_ub_e; InverseKinematics_B.i5++) {
        InverseKinematics_B.i6 = InverseKinematics_B.i5;
        bname->data[InverseKinematics_B.i6] = obj_0->NameInternal->
          data[InverseKinematics_B.i6];
      }

      if (InverseKinematics_strcmp(bname, bodyname)) {
        bid = static_cast<real_T>(InverseKinematics_B.b_i_h) + 1.0;
        exitg1 = true;
      } else {
        InverseKinematics_B.b_i_h++;
      }
    }
  }

  InverseKinematic_emxFree_char_T(&bname);
  return bid;
}

void InverseKinematics::InverseKinematic_emxFree_real_T
  (emxArray_real_T_InverseKinema_T **pEmxArray)
{
  if (*pEmxArray != static_cast<emxArray_real_T_InverseKinema_T *>(NULL)) {
    if (((*pEmxArray)->data != static_cast<real_T *>(NULL)) && (*pEmxArray)
        ->canFreeData) {
      free((*pEmxArray)->data);
    }

    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = static_cast<emxArray_real_T_InverseKinema_T *>(NULL);
  }
}

t_robotics_manip_internal_Rig_T *InverseKinematics::
  InverseKinematic_RigidBody_copy(t_robotics_manip_internal_Rig_T *obj,
  l_robotics_manip_internal_Col_T *iobj_0, c_rigidBodyJoint_InverseKinem_T
  *iobj_1, t_robotics_manip_internal_Rig_T *iobj_2)
{
  c_rigidBodyJoint_InverseKinem_T *obj_0;
  emxArray_char_T_InverseKinema_T *jname;
  emxArray_char_T_InverseKinema_T *jtype;
  emxArray_real_T_InverseKinema_T *obj_3;
  k_robotics_manip_internal_Col_T *obj_2;
  l_robotics_manip_internal_Col_T *newObj;
  l_robotics_manip_internal_Col_T *obj_1;
  t_robotics_manip_internal_Rig_T *newbody;
  static const int8_T tmp[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1
  };

  static const char_T tmp_0[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_1[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const char_T tmp_2[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_3[128] = { '\x00', '\x01', '\x02', '\x03', '\x04',
    '\x05', '\x06', '\x07', '\x08', '\x09', '\x0a', '\x0b', '\x0c', '\x0d',
    '\x0e', '\x0f', '\x10', '\x11', '\x12', '\x13', '\x14', '\x15', '\x16',
    '\x17', '\x18', '\x19', '\x1a', '\x1b', '\x1c', '\x1d', '\x1e', '\x1f', ' ',
    '!', '\"', '#', '$', '%', '&', '\'', '(', ')', '*', '+', ',', '-', '.', '/',
    '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', ':', ';', '<', '=', '>',
    '?', '@', 'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l', 'm',
    'n', 'o', 'p', 'q', 'r', 's', 't', 'u', 'v', 'w', 'x', 'y', 'z', '[', '\\',
    ']', '^', '_', '`', 'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k',
    'l', 'm', 'n', 'o', 'p', 'q', 'r', 's', 't', 'u', 'v', 'w', 'x', 'y', 'z',
    '{', '|', '}', '~', '\x7f' };

  int32_T exitg1;
  boolean_T guard1 = false;
  boolean_T guard11 = false;
  boolean_T guard2 = false;
  boolean_T guard3 = false;
  InverseKinematic_emxInit_char_T(&jtype, 2);
  InverseKinematics_B.nmatched = jtype->size[0] * jtype->size[1];
  jtype->size[0] = 1;
  jtype->size[1] = obj->NameInternal->size[1];
  Invers_emxEnsureCapacity_char_T(jtype, InverseKinematics_B.nmatched);
  InverseKinematics_B.loop_ub_mc = obj->NameInternal->size[1] - 1;
  for (InverseKinematics_B.minnanb = 0; InverseKinematics_B.minnanb <=
       InverseKinematics_B.loop_ub_mc; InverseKinematics_B.minnanb++) {
    InverseKinematics_B.nmatched = InverseKinematics_B.minnanb;
    jtype->data[InverseKinematics_B.nmatched] = obj->NameInternal->
      data[InverseKinematics_B.nmatched];
  }

  newbody = iobj_2;
  InverseKinematics_B.nmatched = iobj_2->NameInternal->size[0] *
    iobj_2->NameInternal->size[1];
  iobj_2->NameInternal->size[0] = 1;
  iobj_2->NameInternal->size[1] = jtype->size[1];
  Invers_emxEnsureCapacity_char_T(iobj_2->NameInternal,
    InverseKinematics_B.nmatched);
  InverseKinematics_B.loop_ub_mc = jtype->size[1] - 1;
  for (InverseKinematics_B.minnanb = 0; InverseKinematics_B.minnanb <=
       InverseKinematics_B.loop_ub_mc; InverseKinematics_B.minnanb++) {
    InverseKinematics_B.nmatched = InverseKinematics_B.minnanb;
    iobj_2->NameInternal->data[InverseKinematics_B.nmatched] = jtype->
      data[InverseKinematics_B.nmatched];
  }

  InverseKinematic_emxInit_char_T(&jname, 2);
  InverseKinematics_B.nmatched = jname->size[0] * jname->size[1];
  jname->size[0] = 1;
  jname->size[1] = jtype->size[1] + 4;
  Invers_emxEnsureCapacity_char_T(jname, InverseKinematics_B.nmatched);
  InverseKinematics_B.loop_ub_mc = jtype->size[1];
  if (InverseKinematics_B.loop_ub_mc - 1 >= 0) {
    memcpy(&jname->data[0], &jtype->data[0], InverseKinematics_B.loop_ub_mc *
           sizeof(char_T));
  }

  jname->data[jtype->size[1]] = '_';
  jname->data[jtype->size[1] + 1] = 'j';
  jname->data[jtype->size[1] + 2] = 'n';
  jname->data[jtype->size[1] + 3] = 't';
  iobj_2->JointInternal = I_rigidBodyJoint_rigidBodyJoint(&iobj_1[0], jname);
  iobj_2->Index = -1.0;
  iobj_2->ParentIndex = -1.0;
  iobj_2->MassInternal = 1.0;
  iobj_2->CenterOfMassInternal[0] = 0.0;
  iobj_2->CenterOfMassInternal[1] = 0.0;
  iobj_2->CenterOfMassInternal[2] = 0.0;
  for (InverseKinematics_B.minnanb = 0; InverseKinematics_B.minnanb < 9;
       InverseKinematics_B.minnanb++) {
    InverseKinematics_B.b_I_bs[InverseKinematics_B.minnanb] = 0;
  }

  InverseKinematics_B.b_I_bs[0] = 1;
  InverseKinematics_B.b_I_bs[4] = 1;
  InverseKinematics_B.b_I_bs[8] = 1;
  for (InverseKinematics_B.minnanb = 0; InverseKinematics_B.minnanb < 9;
       InverseKinematics_B.minnanb++) {
    iobj_2->InertiaInternal[InverseKinematics_B.minnanb] =
      InverseKinematics_B.b_I_bs[InverseKinematics_B.minnanb];
  }

  for (InverseKinematics_B.minnanb = 0; InverseKinematics_B.minnanb < 36;
       InverseKinematics_B.minnanb++) {
    InverseKinematics_B.msubspace_data_n[InverseKinematics_B.minnanb] = 0;
  }

  for (InverseKinematics_B.minnanb = 0; InverseKinematics_B.minnanb < 6;
       InverseKinematics_B.minnanb++) {
    InverseKinematics_B.msubspace_data_n[InverseKinematics_B.minnanb + 6 *
      InverseKinematics_B.minnanb] = 1;
  }

  for (InverseKinematics_B.minnanb = 0; InverseKinematics_B.minnanb < 36;
       InverseKinematics_B.minnanb++) {
    iobj_2->SpatialInertia[InverseKinematics_B.minnanb] =
      InverseKinematics_B.msubspace_data_n[InverseKinematics_B.minnanb];
  }

  iobj_2->CollisionsInternal = Inver_CollisionSet_CollisionSet(&iobj_0[0], 0.0);
  obj_0 = obj->JointInternal;
  InverseKinematics_B.nmatched = jtype->size[0] * jtype->size[1];
  jtype->size[0] = 1;
  jtype->size[1] = obj_0->Type->size[1];
  Invers_emxEnsureCapacity_char_T(jtype, InverseKinematics_B.nmatched);
  InverseKinematics_B.loop_ub_mc = obj_0->Type->size[1] - 1;
  for (InverseKinematics_B.minnanb = 0; InverseKinematics_B.minnanb <=
       InverseKinematics_B.loop_ub_mc; InverseKinematics_B.minnanb++) {
    InverseKinematics_B.nmatched = InverseKinematics_B.minnanb;
    jtype->data[InverseKinematics_B.nmatched] = obj_0->Type->
      data[InverseKinematics_B.nmatched];
  }

  InverseKinematics_B.nmatched = jname->size[0] * jname->size[1];
  jname->size[0] = 1;
  jname->size[1] = obj_0->NameInternal->size[1];
  Invers_emxEnsureCapacity_char_T(jname, InverseKinematics_B.nmatched);
  InverseKinematics_B.loop_ub_mc = obj_0->NameInternal->size[1] - 1;
  for (InverseKinematics_B.minnanb = 0; InverseKinematics_B.minnanb <=
       InverseKinematics_B.loop_ub_mc; InverseKinematics_B.minnanb++) {
    InverseKinematics_B.nmatched = InverseKinematics_B.minnanb;
    jname->data[InverseKinematics_B.nmatched] = obj_0->NameInternal->
      data[InverseKinematics_B.nmatched];
  }

  iobj_1[1].InTree = false;
  for (InverseKinematics_B.minnanb = 0; InverseKinematics_B.minnanb < 16;
       InverseKinematics_B.minnanb++) {
    iobj_1[1].JointToParentTransform[InverseKinematics_B.minnanb] =
      tmp[InverseKinematics_B.minnanb];
  }

  for (InverseKinematics_B.minnanb = 0; InverseKinematics_B.minnanb < 16;
       InverseKinematics_B.minnanb++) {
    iobj_1[1].ChildToJointTransform[InverseKinematics_B.minnanb] =
      tmp[InverseKinematics_B.minnanb];
  }

  InverseKinematics_B.nmatched = iobj_1[1].NameInternal->size[0] * iobj_1[1].
    NameInternal->size[1];
  iobj_1[1].NameInternal->size[0] = 1;
  iobj_1[1].NameInternal->size[1] = jname->size[1];
  Invers_emxEnsureCapacity_char_T(iobj_1[1].NameInternal,
    InverseKinematics_B.nmatched);
  InverseKinematics_B.loop_ub_mc = jname->size[1] - 1;
  for (InverseKinematics_B.minnanb = 0; InverseKinematics_B.minnanb <=
       InverseKinematics_B.loop_ub_mc; InverseKinematics_B.minnanb++) {
    InverseKinematics_B.nmatched = InverseKinematics_B.minnanb;
    iobj_1[1].NameInternal->data[InverseKinematics_B.nmatched] = jname->
      data[InverseKinematics_B.nmatched];
  }

  InverseKinematic_emxFree_char_T(&jname);
  InverseKinematics_B.partial_match_size_idx_1 = 8;
  InverseKinematics_B.nmatched = 0;
  InverseKinematics_B.matched = false;
  for (InverseKinematics_B.minnanb = 0; InverseKinematics_B.minnanb < 8;
       InverseKinematics_B.minnanb++) {
    InverseKinematics_B.partial_match_data[InverseKinematics_B.minnanb] = ' ';
    InverseKinematics_B.vstr[InverseKinematics_B.minnanb] =
      tmp_0[InverseKinematics_B.minnanb];
  }

  guard1 = false;
  guard2 = false;
  guard3 = false;
  if (jtype->size[1] <= 8) {
    InverseKinematics_B.loop_ub_mc = jtype->size[1];
    for (InverseKinematics_B.minnanb = 0; InverseKinematics_B.minnanb < 8;
         InverseKinematics_B.minnanb++) {
      InverseKinematics_B.b_f[InverseKinematics_B.minnanb] =
        tmp_0[InverseKinematics_B.minnanb];
    }

    InverseKinematics_B.b_bool_a = false;
    InverseKinematics_B.minnanb = jtype->size[1];
    guard11 = false;
    if (InverseKinematics_B.loop_ub_mc <= InverseKinematics_B.minnanb) {
      if (InverseKinematics_B.minnanb <= InverseKinematics_B.loop_ub_mc) {
        InverseKinematics_B.loop_ub_mc = InverseKinematics_B.minnanb;
      }

      InverseKinematics_B.minnanb = InverseKinematics_B.loop_ub_mc - 1;
      guard11 = true;
    } else if (jtype->size[1] == 8) {
      InverseKinematics_B.minnanb = 7;
      guard11 = true;
    }

    if (guard11) {
      InverseKinematics_B.loop_ub_mc = 1;
      do {
        exitg1 = 0;
        if (InverseKinematics_B.loop_ub_mc - 1 <= InverseKinematics_B.minnanb) {
          if (tmp_3[static_cast<uint8_T>(jtype->
               data[InverseKinematics_B.loop_ub_mc - 1]) & 127] != tmp_3[
              static_cast<int32_T>
              (InverseKinematics_B.b_f[InverseKinematics_B.loop_ub_mc - 1])]) {
            exitg1 = 1;
          } else {
            InverseKinematics_B.loop_ub_mc++;
          }
        } else {
          InverseKinematics_B.b_bool_a = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (InverseKinematics_B.b_bool_a) {
      if (jtype->size[1] == 8) {
        InverseKinematics_B.nmatched = 1;
        InverseKinematics_B.partial_match_size_idx_1 = 8;
        for (InverseKinematics_B.minnanb = 0; InverseKinematics_B.minnanb < 8;
             InverseKinematics_B.minnanb++) {
          InverseKinematics_B.b_l[InverseKinematics_B.minnanb] =
            InverseKinematics_B.vstr[InverseKinematics_B.minnanb];
        }
      } else {
        InverseKinematics_B.partial_match_size_idx_1 = 8;
        for (InverseKinematics_B.minnanb = 0; InverseKinematics_B.minnanb < 8;
             InverseKinematics_B.minnanb++) {
          InverseKinematics_B.partial_match_data[InverseKinematics_B.minnanb] =
            InverseKinematics_B.vstr[InverseKinematics_B.minnanb];
        }

        InverseKinematics_B.matched = true;
        InverseKinematics_B.nmatched = 1;
        guard3 = true;
      }
    } else {
      guard3 = true;
    }
  } else {
    guard3 = true;
  }

  if (guard3) {
    for (InverseKinematics_B.minnanb = 0; InverseKinematics_B.minnanb < 9;
         InverseKinematics_B.minnanb++) {
      InverseKinematics_B.b_vstr[InverseKinematics_B.minnanb] =
        tmp_1[InverseKinematics_B.minnanb];
    }

    if (jtype->size[1] <= 9) {
      InverseKinematics_B.loop_ub_mc = jtype->size[1];
      for (InverseKinematics_B.minnanb = 0; InverseKinematics_B.minnanb < 9;
           InverseKinematics_B.minnanb++) {
        InverseKinematics_B.b_l[InverseKinematics_B.minnanb] =
          tmp_1[InverseKinematics_B.minnanb];
      }

      InverseKinematics_B.b_bool_a = false;
      InverseKinematics_B.minnanb = jtype->size[1];
      guard11 = false;
      if (InverseKinematics_B.loop_ub_mc <= InverseKinematics_B.minnanb) {
        if (InverseKinematics_B.minnanb <= InverseKinematics_B.loop_ub_mc) {
          InverseKinematics_B.loop_ub_mc = InverseKinematics_B.minnanb;
        }

        InverseKinematics_B.minnanb = InverseKinematics_B.loop_ub_mc - 1;
        guard11 = true;
      } else if (jtype->size[1] == 9) {
        InverseKinematics_B.minnanb = 8;
        guard11 = true;
      }

      if (guard11) {
        InverseKinematics_B.loop_ub_mc = 1;
        do {
          exitg1 = 0;
          if (InverseKinematics_B.loop_ub_mc - 1 <= InverseKinematics_B.minnanb)
          {
            if (tmp_3[static_cast<uint8_T>(jtype->
                 data[InverseKinematics_B.loop_ub_mc - 1]) & 127] != tmp_3[
                static_cast<int32_T>
                (InverseKinematics_B.b_l[InverseKinematics_B.loop_ub_mc - 1])])
            {
              exitg1 = 1;
            } else {
              InverseKinematics_B.loop_ub_mc++;
            }
          } else {
            InverseKinematics_B.b_bool_a = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }

      if (InverseKinematics_B.b_bool_a) {
        if (jtype->size[1] == 9) {
          InverseKinematics_B.nmatched = 1;
          InverseKinematics_B.partial_match_size_idx_1 = 9;
          for (InverseKinematics_B.minnanb = 0; InverseKinematics_B.minnanb < 9;
               InverseKinematics_B.minnanb++) {
            InverseKinematics_B.b_l[InverseKinematics_B.minnanb] =
              InverseKinematics_B.b_vstr[InverseKinematics_B.minnanb];
          }
        } else {
          if (!InverseKinematics_B.matched) {
            InverseKinematics_B.partial_match_size_idx_1 = 9;
            for (InverseKinematics_B.minnanb = 0; InverseKinematics_B.minnanb <
                 9; InverseKinematics_B.minnanb++) {
              InverseKinematics_B.partial_match_data[InverseKinematics_B.minnanb]
                = InverseKinematics_B.b_vstr[InverseKinematics_B.minnanb];
            }
          }

          InverseKinematics_B.matched = true;
          InverseKinematics_B.nmatched++;
          guard2 = true;
        }
      } else {
        guard2 = true;
      }
    } else {
      guard2 = true;
    }
  }

  if (guard2) {
    for (InverseKinematics_B.minnanb = 0; InverseKinematics_B.minnanb < 5;
         InverseKinematics_B.minnanb++) {
      InverseKinematics_B.c_vstr[InverseKinematics_B.minnanb] =
        tmp_2[InverseKinematics_B.minnanb];
    }

    if (jtype->size[1] <= 5) {
      InverseKinematics_B.loop_ub_mc = jtype->size[1];
      for (InverseKinematics_B.minnanb = 0; InverseKinematics_B.minnanb < 5;
           InverseKinematics_B.minnanb++) {
        InverseKinematics_B.b_ev[InverseKinematics_B.minnanb] =
          tmp_2[InverseKinematics_B.minnanb];
      }

      InverseKinematics_B.b_bool_a = false;
      InverseKinematics_B.minnanb = jtype->size[1];
      guard11 = false;
      if (InverseKinematics_B.loop_ub_mc <= InverseKinematics_B.minnanb) {
        if (InverseKinematics_B.minnanb <= InverseKinematics_B.loop_ub_mc) {
          InverseKinematics_B.loop_ub_mc = InverseKinematics_B.minnanb;
        }

        InverseKinematics_B.minnanb = InverseKinematics_B.loop_ub_mc - 1;
        guard11 = true;
      } else if (jtype->size[1] == 5) {
        InverseKinematics_B.minnanb = 4;
        guard11 = true;
      }

      if (guard11) {
        InverseKinematics_B.loop_ub_mc = 1;
        do {
          exitg1 = 0;
          if (InverseKinematics_B.loop_ub_mc - 1 <= InverseKinematics_B.minnanb)
          {
            if (tmp_3[static_cast<uint8_T>(jtype->
                 data[InverseKinematics_B.loop_ub_mc - 1]) & 127] != tmp_3[
                static_cast<int32_T>
                (InverseKinematics_B.b_ev[InverseKinematics_B.loop_ub_mc - 1])])
            {
              exitg1 = 1;
            } else {
              InverseKinematics_B.loop_ub_mc++;
            }
          } else {
            InverseKinematics_B.b_bool_a = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }

      if (InverseKinematics_B.b_bool_a) {
        if (jtype->size[1] == 5) {
          InverseKinematics_B.nmatched = 1;
          InverseKinematics_B.partial_match_size_idx_1 = 5;
          for (InverseKinematics_B.minnanb = 0; InverseKinematics_B.minnanb < 5;
               InverseKinematics_B.minnanb++) {
            InverseKinematics_B.b_l[InverseKinematics_B.minnanb] =
              InverseKinematics_B.c_vstr[InverseKinematics_B.minnanb];
          }
        } else {
          if (!InverseKinematics_B.matched) {
            InverseKinematics_B.partial_match_size_idx_1 = 5;
            for (InverseKinematics_B.minnanb = 0; InverseKinematics_B.minnanb <
                 5; InverseKinematics_B.minnanb++) {
              InverseKinematics_B.partial_match_data[InverseKinematics_B.minnanb]
                = InverseKinematics_B.c_vstr[InverseKinematics_B.minnanb];
            }
          }

          InverseKinematics_B.nmatched++;
          guard1 = true;
        }
      } else {
        guard1 = true;
      }
    } else {
      guard1 = true;
    }
  }

  if (guard1) {
    if (InverseKinematics_B.nmatched == 0) {
      InverseKinematics_B.partial_match_size_idx_1 = 8;
      for (InverseKinematics_B.minnanb = 0; InverseKinematics_B.minnanb < 8;
           InverseKinematics_B.minnanb++) {
        InverseKinematics_B.b_l[InverseKinematics_B.minnanb] = ' ';
      }
    } else {
      InverseKinematics_B.loop_ub_mc =
        InverseKinematics_B.partial_match_size_idx_1 - 1;
      memcpy(&InverseKinematics_B.b_l[0],
             &InverseKinematics_B.partial_match_data[0],
             (InverseKinematics_B.loop_ub_mc + 1) * sizeof(char_T));
    }
  }

  if ((InverseKinematics_B.nmatched == 0) || (jtype->size[1] == 0)) {
    InverseKinematics_B.partial_match_size_idx_1 = 8;
    for (InverseKinematics_B.minnanb = 0; InverseKinematics_B.minnanb < 8;
         InverseKinematics_B.minnanb++) {
      InverseKinematics_B.partial_match_data[InverseKinematics_B.minnanb] = ' ';
    }
  } else {
    InverseKinematics_B.loop_ub_mc =
      InverseKinematics_B.partial_match_size_idx_1 - 1;
    memcpy(&InverseKinematics_B.partial_match_data[0], &InverseKinematics_B.b_l
           [0], (InverseKinematics_B.loop_ub_mc + 1) * sizeof(char_T));
  }

  InverseKinematics_B.nmatched = iobj_1[1].Type->size[0] * iobj_1[1].Type->size
    [1];
  iobj_1[1].Type->size[0] = 1;
  iobj_1[1].Type->size[1] = InverseKinematics_B.partial_match_size_idx_1;
  Invers_emxEnsureCapacity_char_T(iobj_1[1].Type, InverseKinematics_B.nmatched);
  InverseKinematics_B.loop_ub_mc = InverseKinematics_B.partial_match_size_idx_1
    - 1;
  for (InverseKinematics_B.minnanb = 0; InverseKinematics_B.minnanb <=
       InverseKinematics_B.loop_ub_mc; InverseKinematics_B.minnanb++) {
    InverseKinematics_B.nmatched = InverseKinematics_B.minnanb;
    iobj_1[1].Type->data[InverseKinematics_B.nmatched] =
      InverseKinematics_B.partial_match_data[InverseKinematics_B.nmatched];
  }

  InverseKinematics_B.nmatched = jtype->size[0] * jtype->size[1];
  jtype->size[0] = 1;
  jtype->size[1] = iobj_1[1].Type->size[1];
  Invers_emxEnsureCapacity_char_T(jtype, InverseKinematics_B.nmatched);
  InverseKinematics_B.loop_ub_mc = iobj_1[1].Type->size[1] - 1;
  for (InverseKinematics_B.minnanb = 0; InverseKinematics_B.minnanb <=
       InverseKinematics_B.loop_ub_mc; InverseKinematics_B.minnanb++) {
    InverseKinematics_B.nmatched = InverseKinematics_B.minnanb;
    jtype->data[InverseKinematics_B.nmatched] = iobj_1[1].Type->
      data[InverseKinematics_B.nmatched];
  }

  for (InverseKinematics_B.minnanb = 0; InverseKinematics_B.minnanb < 8;
       InverseKinematics_B.minnanb++) {
    InverseKinematics_B.b_f[InverseKinematics_B.minnanb] =
      tmp_0[InverseKinematics_B.minnanb];
  }

  InverseKinematics_B.b_bool_a = false;
  if (jtype->size[1] != 8) {
  } else {
    InverseKinematics_B.loop_ub_mc = 1;
    do {
      exitg1 = 0;
      if (InverseKinematics_B.loop_ub_mc - 1 < 8) {
        if (jtype->data[InverseKinematics_B.loop_ub_mc - 1] !=
            InverseKinematics_B.b_f[InverseKinematics_B.loop_ub_mc - 1]) {
          exitg1 = 1;
        } else {
          InverseKinematics_B.loop_ub_mc++;
        }
      } else {
        InverseKinematics_B.b_bool_a = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (InverseKinematics_B.b_bool_a) {
    InverseKinematics_B.minnanb = 0;
  } else {
    for (InverseKinematics_B.minnanb = 0; InverseKinematics_B.minnanb < 9;
         InverseKinematics_B.minnanb++) {
      InverseKinematics_B.b_l[InverseKinematics_B.minnanb] =
        tmp_1[InverseKinematics_B.minnanb];
    }

    InverseKinematics_B.b_bool_a = false;
    if (jtype->size[1] != 9) {
    } else {
      InverseKinematics_B.loop_ub_mc = 1;
      do {
        exitg1 = 0;
        if (InverseKinematics_B.loop_ub_mc - 1 < 9) {
          if (jtype->data[InverseKinematics_B.loop_ub_mc - 1] !=
              InverseKinematics_B.b_l[InverseKinematics_B.loop_ub_mc - 1]) {
            exitg1 = 1;
          } else {
            InverseKinematics_B.loop_ub_mc++;
          }
        } else {
          InverseKinematics_B.b_bool_a = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (InverseKinematics_B.b_bool_a) {
      InverseKinematics_B.minnanb = 1;
    } else {
      InverseKinematics_B.minnanb = -1;
    }
  }

  switch (InverseKinematics_B.minnanb) {
   case 0:
    InverseKinematics_B.iv1[0] = 0;
    InverseKinematics_B.iv1[1] = 0;
    InverseKinematics_B.iv1[2] = 1;
    InverseKinematics_B.iv1[3] = 0;
    InverseKinematics_B.iv1[4] = 0;
    InverseKinematics_B.iv1[5] = 0;
    for (InverseKinematics_B.minnanb = 0; InverseKinematics_B.minnanb < 6;
         InverseKinematics_B.minnanb++) {
      InverseKinematics_B.msubspace_data_n[InverseKinematics_B.minnanb] =
        InverseKinematics_B.iv1[InverseKinematics_B.minnanb];
    }

    InverseKinematics_B.poslim_data_b[0] = -3.1415926535897931;
    InverseKinematics_B.poslim_data_b[1] = 3.1415926535897931;
    iobj_1[1].VelocityNumber = 1.0;
    iobj_1[1].PositionNumber = 1.0;
    iobj_1[1].JointAxisInternal[0] = 0.0;
    iobj_1[1].JointAxisInternal[1] = 0.0;
    iobj_1[1].JointAxisInternal[2] = 1.0;
    break;

   case 1:
    InverseKinematics_B.iv1[0] = 0;
    InverseKinematics_B.iv1[1] = 0;
    InverseKinematics_B.iv1[2] = 0;
    InverseKinematics_B.iv1[3] = 0;
    InverseKinematics_B.iv1[4] = 0;
    InverseKinematics_B.iv1[5] = 1;
    for (InverseKinematics_B.minnanb = 0; InverseKinematics_B.minnanb < 6;
         InverseKinematics_B.minnanb++) {
      InverseKinematics_B.msubspace_data_n[InverseKinematics_B.minnanb] =
        InverseKinematics_B.iv1[InverseKinematics_B.minnanb];
    }

    InverseKinematics_B.poslim_data_b[0] = -0.5;
    InverseKinematics_B.poslim_data_b[1] = 0.5;
    iobj_1[1].VelocityNumber = 1.0;
    iobj_1[1].PositionNumber = 1.0;
    iobj_1[1].JointAxisInternal[0] = 0.0;
    iobj_1[1].JointAxisInternal[1] = 0.0;
    iobj_1[1].JointAxisInternal[2] = 1.0;
    break;

   default:
    for (InverseKinematics_B.minnanb = 0; InverseKinematics_B.minnanb < 6;
         InverseKinematics_B.minnanb++) {
      InverseKinematics_B.msubspace_data_n[InverseKinematics_B.minnanb] = 0;
    }

    InverseKinematics_B.poslim_data_b[0] = 0.0;
    InverseKinematics_B.poslim_data_b[1] = 0.0;
    iobj_1[1].VelocityNumber = 0.0;
    iobj_1[1].PositionNumber = 0.0;
    iobj_1[1].JointAxisInternal[0] = 0.0;
    iobj_1[1].JointAxisInternal[1] = 0.0;
    iobj_1[1].JointAxisInternal[2] = 0.0;
    break;
  }

  InverseKinematics_B.nmatched = iobj_1[1].MotionSubspace->size[0] * iobj_1[1].
    MotionSubspace->size[1];
  iobj_1[1].MotionSubspace->size[0] = 6;
  iobj_1[1].MotionSubspace->size[1] = 1;
  Invers_emxEnsureCapacity_real_T(iobj_1[1].MotionSubspace,
    InverseKinematics_B.nmatched);
  for (InverseKinematics_B.minnanb = 0; InverseKinematics_B.minnanb < 6;
       InverseKinematics_B.minnanb++) {
    iobj_1[1].MotionSubspace->data[InverseKinematics_B.minnanb] =
      InverseKinematics_B.msubspace_data_n[InverseKinematics_B.minnanb];
  }

  InverseKinematics_B.nmatched = iobj_1[1].PositionLimitsInternal->size[0] *
    iobj_1[1].PositionLimitsInternal->size[1];
  iobj_1[1].PositionLimitsInternal->size[0] = 1;
  iobj_1[1].PositionLimitsInternal->size[1] = 2;
  Invers_emxEnsureCapacity_real_T(iobj_1[1].PositionLimitsInternal,
    InverseKinematics_B.nmatched);
  for (InverseKinematics_B.minnanb = 0; InverseKinematics_B.minnanb < 2;
       InverseKinematics_B.minnanb++) {
    iobj_1[1].PositionLimitsInternal->data[InverseKinematics_B.minnanb] =
      InverseKinematics_B.poslim_data_b[InverseKinematics_B.minnanb];
  }

  InverseKinematics_B.nmatched = iobj_1[1].HomePositionInternal->size[0];
  iobj_1[1].HomePositionInternal->size[0] = 1;
  Invers_emxEnsureCapacity_real_T(iobj_1[1].HomePositionInternal,
    InverseKinematics_B.nmatched);
  iobj_1[1].HomePositionInternal->data[0] = 0.0;
  InverseKinematics_B.nmatched = jtype->size[0] * jtype->size[1];
  jtype->size[0] = 1;
  jtype->size[1] = obj_0->NameInternal->size[1];
  Invers_emxEnsureCapacity_char_T(jtype, InverseKinematics_B.nmatched);
  InverseKinematics_B.loop_ub_mc = obj_0->NameInternal->size[1] - 1;
  for (InverseKinematics_B.minnanb = 0; InverseKinematics_B.minnanb <=
       InverseKinematics_B.loop_ub_mc; InverseKinematics_B.minnanb++) {
    InverseKinematics_B.nmatched = InverseKinematics_B.minnanb;
    jtype->data[InverseKinematics_B.nmatched] = obj_0->NameInternal->
      data[InverseKinematics_B.nmatched];
  }

  if (jtype->size[1] != 0) {
    InverseKinematics_B.nmatched = jtype->size[0] * jtype->size[1];
    jtype->size[0] = 1;
    jtype->size[1] = obj_0->NameInternal->size[1];
    Invers_emxEnsureCapacity_char_T(jtype, InverseKinematics_B.nmatched);
    InverseKinematics_B.loop_ub_mc = obj_0->NameInternal->size[1] - 1;
    for (InverseKinematics_B.minnanb = 0; InverseKinematics_B.minnanb <=
         InverseKinematics_B.loop_ub_mc; InverseKinematics_B.minnanb++) {
      InverseKinematics_B.nmatched = InverseKinematics_B.minnanb;
      jtype->data[InverseKinematics_B.nmatched] = obj_0->NameInternal->
        data[InverseKinematics_B.nmatched];
    }

    if (!iobj_1[1].InTree) {
      InverseKinematics_B.nmatched = iobj_1[1].NameInternal->size[0] * iobj_1[1]
        .NameInternal->size[1];
      iobj_1[1].NameInternal->size[0] = 1;
      iobj_1[1].NameInternal->size[1] = jtype->size[1];
      Invers_emxEnsureCapacity_char_T(iobj_1[1].NameInternal,
        InverseKinematics_B.nmatched);
      InverseKinematics_B.loop_ub_mc = jtype->size[1] - 1;
      for (InverseKinematics_B.minnanb = 0; InverseKinematics_B.minnanb <=
           InverseKinematics_B.loop_ub_mc; InverseKinematics_B.minnanb++) {
        InverseKinematics_B.nmatched = InverseKinematics_B.minnanb;
        iobj_1[1].NameInternal->data[InverseKinematics_B.nmatched] = jtype->
          data[InverseKinematics_B.nmatched];
      }
    }
  }

  InverseKinematic_emxFree_char_T(&jtype);
  InverseKinematic_emxInit_real_T(&obj_3, 1);
  InverseKinematics_B.loop_ub_mc = obj_0->PositionLimitsInternal->size[0] << 1;
  InverseKinematics_B.nmatched = iobj_1[1].PositionLimitsInternal->size[0] *
    iobj_1[1].PositionLimitsInternal->size[1];
  iobj_1[1].PositionLimitsInternal->size[0] = obj_0->
    PositionLimitsInternal->size[0];
  iobj_1[1].PositionLimitsInternal->size[1] = 2;
  Invers_emxEnsureCapacity_real_T(iobj_1[1].PositionLimitsInternal,
    InverseKinematics_B.nmatched);
  InverseKinematics_B.nmatched = obj_3->size[0];
  obj_3->size[0] = InverseKinematics_B.loop_ub_mc;
  Invers_emxEnsureCapacity_real_T(obj_3, InverseKinematics_B.nmatched);
  for (InverseKinematics_B.minnanb = 0; InverseKinematics_B.minnanb <
       InverseKinematics_B.loop_ub_mc; InverseKinematics_B.minnanb++) {
    obj_3->data[InverseKinematics_B.minnanb] = obj_0->
      PositionLimitsInternal->data[InverseKinematics_B.minnanb];
  }

  InverseKinematics_B.loop_ub_mc = obj_3->size[0];
  for (InverseKinematics_B.minnanb = 0; InverseKinematics_B.minnanb <
       InverseKinematics_B.loop_ub_mc; InverseKinematics_B.minnanb++) {
    iobj_1[1].PositionLimitsInternal->data[InverseKinematics_B.minnanb] =
      obj_3->data[InverseKinematics_B.minnanb];
  }

  InverseKinematics_B.nmatched = obj_3->size[0];
  obj_3->size[0] = obj_0->HomePositionInternal->size[0];
  Invers_emxEnsureCapacity_real_T(obj_3, InverseKinematics_B.nmatched);
  InverseKinematics_B.loop_ub_mc = obj_0->HomePositionInternal->size[0];
  for (InverseKinematics_B.minnanb = 0; InverseKinematics_B.minnanb <
       InverseKinematics_B.loop_ub_mc; InverseKinematics_B.minnanb++) {
    obj_3->data[InverseKinematics_B.minnanb] = obj_0->HomePositionInternal->
      data[InverseKinematics_B.minnanb];
  }

  InverseKinematics_B.nmatched = iobj_1[1].HomePositionInternal->size[0];
  iobj_1[1].HomePositionInternal->size[0] = obj_3->size[0];
  Invers_emxEnsureCapacity_real_T(iobj_1[1].HomePositionInternal,
    InverseKinematics_B.nmatched);
  InverseKinematics_B.loop_ub_mc = obj_3->size[0];
  for (InverseKinematics_B.minnanb = 0; InverseKinematics_B.minnanb <
       InverseKinematics_B.loop_ub_mc; InverseKinematics_B.minnanb++) {
    iobj_1[1].HomePositionInternal->data[InverseKinematics_B.minnanb] =
      obj_3->data[InverseKinematics_B.minnanb];
  }

  InverseKinematics_B.obj_idx_0 = obj_0->JointAxisInternal[0];
  InverseKinematics_B.obj_idx_1 = obj_0->JointAxisInternal[1];
  InverseKinematics_B.obj_idx_2 = obj_0->JointAxisInternal[2];
  iobj_1[1].JointAxisInternal[0] = InverseKinematics_B.obj_idx_0;
  iobj_1[1].JointAxisInternal[1] = InverseKinematics_B.obj_idx_1;
  iobj_1[1].JointAxisInternal[2] = InverseKinematics_B.obj_idx_2;
  InverseKinematics_B.loop_ub_mc = 6 * obj_0->MotionSubspace->size[1];
  InverseKinematics_B.nmatched = iobj_1[1].MotionSubspace->size[0] * iobj_1[1].
    MotionSubspace->size[1];
  iobj_1[1].MotionSubspace->size[0] = 6;
  iobj_1[1].MotionSubspace->size[1] = obj_0->MotionSubspace->size[1];
  Invers_emxEnsureCapacity_real_T(iobj_1[1].MotionSubspace,
    InverseKinematics_B.nmatched);
  InverseKinematics_B.nmatched = obj_3->size[0];
  obj_3->size[0] = InverseKinematics_B.loop_ub_mc;
  Invers_emxEnsureCapacity_real_T(obj_3, InverseKinematics_B.nmatched);
  for (InverseKinematics_B.minnanb = 0; InverseKinematics_B.minnanb <
       InverseKinematics_B.loop_ub_mc; InverseKinematics_B.minnanb++) {
    obj_3->data[InverseKinematics_B.minnanb] = obj_0->MotionSubspace->
      data[InverseKinematics_B.minnanb];
  }

  InverseKinematics_B.loop_ub_mc = obj_3->size[0];
  for (InverseKinematics_B.minnanb = 0; InverseKinematics_B.minnanb <
       InverseKinematics_B.loop_ub_mc; InverseKinematics_B.minnanb++) {
    iobj_1[1].MotionSubspace->data[InverseKinematics_B.minnanb] = obj_3->
      data[InverseKinematics_B.minnanb];
  }

  InverseKinematic_emxFree_real_T(&obj_3);
  for (InverseKinematics_B.minnanb = 0; InverseKinematics_B.minnanb < 16;
       InverseKinematics_B.minnanb++) {
    InverseKinematics_B.obj_c[InverseKinematics_B.minnanb] =
      obj_0->JointToParentTransform[InverseKinematics_B.minnanb];
  }

  for (InverseKinematics_B.minnanb = 0; InverseKinematics_B.minnanb < 16;
       InverseKinematics_B.minnanb++) {
    iobj_1[1].JointToParentTransform[InverseKinematics_B.minnanb] =
      InverseKinematics_B.obj_c[InverseKinematics_B.minnanb];
  }

  for (InverseKinematics_B.minnanb = 0; InverseKinematics_B.minnanb < 16;
       InverseKinematics_B.minnanb++) {
    InverseKinematics_B.obj_c[InverseKinematics_B.minnanb] =
      obj_0->ChildToJointTransform[InverseKinematics_B.minnanb];
  }

  for (InverseKinematics_B.minnanb = 0; InverseKinematics_B.minnanb < 16;
       InverseKinematics_B.minnanb++) {
    iobj_1[1].ChildToJointTransform[InverseKinematics_B.minnanb] =
      InverseKinematics_B.obj_c[InverseKinematics_B.minnanb];
  }

  iobj_2->JointInternal = &iobj_1[1];
  iobj_2->MassInternal = obj->MassInternal;
  InverseKinematics_B.obj_idx_0 = obj->CenterOfMassInternal[0];
  InverseKinematics_B.obj_idx_1 = obj->CenterOfMassInternal[1];
  InverseKinematics_B.obj_idx_2 = obj->CenterOfMassInternal[2];
  iobj_2->CenterOfMassInternal[0] = InverseKinematics_B.obj_idx_0;
  iobj_2->CenterOfMassInternal[1] = InverseKinematics_B.obj_idx_1;
  iobj_2->CenterOfMassInternal[2] = InverseKinematics_B.obj_idx_2;
  for (InverseKinematics_B.minnanb = 0; InverseKinematics_B.minnanb < 9;
       InverseKinematics_B.minnanb++) {
    InverseKinematics_B.obj_g[InverseKinematics_B.minnanb] =
      obj->InertiaInternal[InverseKinematics_B.minnanb];
  }

  for (InverseKinematics_B.minnanb = 0; InverseKinematics_B.minnanb < 9;
       InverseKinematics_B.minnanb++) {
    iobj_2->InertiaInternal[InverseKinematics_B.minnanb] =
      InverseKinematics_B.obj_g[InverseKinematics_B.minnanb];
  }

  for (InverseKinematics_B.minnanb = 0; InverseKinematics_B.minnanb < 36;
       InverseKinematics_B.minnanb++) {
    InverseKinematics_B.obj[InverseKinematics_B.minnanb] = obj->
      SpatialInertia[InverseKinematics_B.minnanb];
  }

  for (InverseKinematics_B.minnanb = 0; InverseKinematics_B.minnanb < 36;
       InverseKinematics_B.minnanb++) {
    iobj_2->SpatialInertia[InverseKinematics_B.minnanb] =
      InverseKinematics_B.obj[InverseKinematics_B.minnanb];
  }

  obj_1 = obj->CollisionsInternal;
  newObj = Inver_CollisionSet_CollisionSet(&iobj_0[1], obj_1->MaxElements);
  newObj->Size = obj_1->Size;
  InverseKinematics_B.obj_idx_0 = obj_1->Size;
  InverseKinematics_B.nmatched = static_cast<int32_T>
    (InverseKinematics_B.obj_idx_0) - 1;
  for (InverseKinematics_B.minnanb = 0; InverseKinematics_B.minnanb <=
       InverseKinematics_B.nmatched; InverseKinematics_B.minnanb++) {
    InverseKinematics_B.loop_ub_mc = InverseKinematics_B.minnanb;
    obj_2 = obj_1->CollisionGeometries->data[InverseKinematics_B.loop_ub_mc];
    newObj->CollisionGeometries->data[InverseKinematics_B.loop_ub_mc] = obj_2;
  }

  iobj_2->CollisionsInternal = newObj;
  return newbody;
}

void InverseKinematics::InverseKi_RigidBodyTree_addBody
  (v_robotics_manip_internal_Rig_T *obj, t_robotics_manip_internal_Rig_T *bodyin,
   const emxArray_char_T_InverseKinema_T *parentName,
   c_rigidBodyJoint_InverseKinem_T *iobj_0, t_robotics_manip_internal_Rig_T
   *iobj_1, l_robotics_manip_internal_Col_T *iobj_2)
{
  c_rigidBodyJoint_InverseKinem_T *jnt;
  emxArray_char_T_InverseKinema_T *bname;
  t_robotics_manip_internal_Rig_T *body;
  static const char_T tmp[5] = { 'f', 'i', 'x', 'e', 'd' };

  InverseKinematic_emxInit_char_T(&bname, 2);
  InverseKinematics_B.i1 = bname->size[0] * bname->size[1];
  bname->size[0] = 1;
  bname->size[1] = bodyin->NameInternal->size[1];
  Invers_emxEnsureCapacity_char_T(bname, InverseKinematics_B.i1);
  InverseKinematics_B.loop_ub_me = bodyin->NameInternal->size[1] - 1;
  for (InverseKinematics_B.b_kstr_h = 0; InverseKinematics_B.b_kstr_h <=
       InverseKinematics_B.loop_ub_me; InverseKinematics_B.b_kstr_h++) {
    InverseKinematics_B.i1 = InverseKinematics_B.b_kstr_h;
    bname->data[InverseKinematics_B.i1] = bodyin->NameInternal->
      data[InverseKinematics_B.i1];
  }

  RigidBodyTree_findBodyIndexByNa(obj, bname);
  InverseKinematics_B.pid = RigidBodyTree_findBodyIndexByNa(obj, parentName);
  InverseKinematics_B.b_index = obj->NumBodies + 1.0;
  body = InverseKinematic_RigidBody_copy(bodyin, &iobj_2[0], &iobj_0[0], iobj_1);
  obj->Bodies[static_cast<int32_T>(InverseKinematics_B.b_index) - 1] = body;
  body->Index = InverseKinematics_B.b_index;
  body->ParentIndex = InverseKinematics_B.pid;
  body->JointInternal->InTree = true;
  obj->NumBodies++;
  jnt = body->JointInternal;
  InverseKinematics_B.i1 = bname->size[0] * bname->size[1];
  bname->size[0] = 1;
  bname->size[1] = jnt->Type->size[1];
  Invers_emxEnsureCapacity_char_T(bname, InverseKinematics_B.i1);
  InverseKinematics_B.loop_ub_me = jnt->Type->size[1] - 1;
  for (InverseKinematics_B.b_kstr_h = 0; InverseKinematics_B.b_kstr_h <=
       InverseKinematics_B.loop_ub_me; InverseKinematics_B.b_kstr_h++) {
    InverseKinematics_B.i1 = InverseKinematics_B.b_kstr_h;
    bname->data[InverseKinematics_B.i1] = jnt->Type->data[InverseKinematics_B.i1];
  }

  for (InverseKinematics_B.b_kstr_h = 0; InverseKinematics_B.b_kstr_h < 5;
       InverseKinematics_B.b_kstr_h++) {
    InverseKinematics_B.b_af[InverseKinematics_B.b_kstr_h] =
      tmp[InverseKinematics_B.b_kstr_h];
  }

  InverseKinematics_B.b_bool_m = false;
  if (bname->size[1] != 5) {
  } else {
    InverseKinematics_B.b_kstr_h = 1;
    int32_T exitg1;
    do {
      exitg1 = 0;
      if (InverseKinematics_B.b_kstr_h - 1 < 5) {
        if (bname->data[InverseKinematics_B.b_kstr_h - 1] !=
            InverseKinematics_B.b_af[InverseKinematics_B.b_kstr_h - 1]) {
          exitg1 = 1;
        } else {
          InverseKinematics_B.b_kstr_h++;
        }
      } else {
        InverseKinematics_B.b_bool_m = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  InverseKinematic_emxFree_char_T(&bname);
  if (!InverseKinematics_B.b_bool_m) {
    obj->NumNonFixedBodies++;
    jnt = body->JointInternal;
    InverseKinematics_B.b_kstr_h = static_cast<int32_T>(body->Index) - 1;
    obj->PositionDoFMap[InverseKinematics_B.b_kstr_h] = obj->PositionNumber +
      1.0;
    obj->PositionDoFMap[InverseKinematics_B.b_kstr_h + 6] = obj->PositionNumber
      + jnt->PositionNumber;
    jnt = body->JointInternal;
    InverseKinematics_B.b_kstr_h = static_cast<int32_T>(body->Index) - 1;
    obj->VelocityDoFMap[InverseKinematics_B.b_kstr_h] = obj->VelocityNumber +
      1.0;
    obj->VelocityDoFMap[InverseKinematics_B.b_kstr_h + 6] = obj->VelocityNumber
      + jnt->VelocityNumber;
  } else {
    InverseKinematics_B.b_kstr_h = static_cast<int32_T>(body->Index);
    obj->PositionDoFMap[InverseKinematics_B.b_kstr_h - 1] = 0.0;
    obj->PositionDoFMap[InverseKinematics_B.b_kstr_h + 5] = -1.0;
    InverseKinematics_B.b_kstr_h = static_cast<int32_T>(body->Index);
    obj->VelocityDoFMap[InverseKinematics_B.b_kstr_h - 1] = 0.0;
    obj->VelocityDoFMap[InverseKinematics_B.b_kstr_h + 5] = -1.0;
  }

  jnt = body->JointInternal;
  obj->PositionNumber += jnt->PositionNumber;
  jnt = body->JointInternal;
  obj->VelocityNumber += jnt->VelocityNumber;
}

void InverseKinematics::inverseKinematics_set_RigidBody
  (b_inverseKinematics_InverseKi_T *obj, u_robotics_manip_internal_Rig_T
   *rigidbodytree, c_rigidBodyJoint_InverseKinem_T *iobj_0,
   t_robotics_manip_internal_Rig_T *iobj_1, l_robotics_manip_internal_Col_T
   *iobj_2, v_robotics_manip_internal_Rig_T *iobj_3)
{
  c_rigidBodyJoint_InverseKinem_T *iobj_1_0;
  emxArray_char_T_InverseKinema_T *bname;
  emxArray_char_T_InverseKinema_T *switch_expression;
  k_robotics_manip_internal_Col_T *obj_0;
  l_robotics_manip_internal_Col_T *iobj_0_0;
  l_robotics_manip_internal_Col_T *newObj;
  t_robotics_manip_internal_Rig_T *body;
  t_robotics_manip_internal_Rig_T *parent;
  static const int8_T tmp[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1
  };

  static const char_T tmp_0[8] = { 'b', 'a', 's', 'e', '_', 'j', 'n', 't' };

  static const char_T tmp_1[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_2[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_3[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  int32_T exitg1;
  body = &iobj_3->Base;
  iobj_0_0 = &iobj_3->_pobj1[0];
  InverseKinematics_B.obj_tmp = body->NameInternal->size[0] * body->
    NameInternal->size[1];
  body->NameInternal->size[0] = 1;
  body->NameInternal->size[1] = 4;
  Invers_emxEnsureCapacity_char_T(body->NameInternal,
    InverseKinematics_B.obj_tmp);
  body->NameInternal->data[0] = 'b';
  body->NameInternal->data[1] = 'a';
  body->NameInternal->data[2] = 's';
  body->NameInternal->data[3] = 'e';
  iobj_3->_pobj2[0].InTree = false;
  for (InverseKinematics_B.b_kstr = 0; InverseKinematics_B.b_kstr < 16;
       InverseKinematics_B.b_kstr++) {
    iobj_3->_pobj2[0].JointToParentTransform[InverseKinematics_B.b_kstr] =
      tmp[InverseKinematics_B.b_kstr];
  }

  for (InverseKinematics_B.b_kstr = 0; InverseKinematics_B.b_kstr < 16;
       InverseKinematics_B.b_kstr++) {
    iobj_3->_pobj2[0].ChildToJointTransform[InverseKinematics_B.b_kstr] =
      tmp[InverseKinematics_B.b_kstr];
  }

  InverseKinematics_B.obj_tmp = iobj_3->_pobj2[0].NameInternal->size[0] *
    iobj_3->_pobj2[0].NameInternal->size[1];
  iobj_3->_pobj2[0].NameInternal->size[0] = 1;
  iobj_3->_pobj2[0].NameInternal->size[1] = 8;
  Invers_emxEnsureCapacity_char_T(iobj_3->_pobj2[0].NameInternal,
    InverseKinematics_B.obj_tmp);
  for (InverseKinematics_B.b_kstr = 0; InverseKinematics_B.b_kstr < 8;
       InverseKinematics_B.b_kstr++) {
    iobj_3->_pobj2[0].NameInternal->data[InverseKinematics_B.b_kstr] =
      tmp_0[InverseKinematics_B.b_kstr];
  }

  InverseKinematics_B.obj_tmp = iobj_3->_pobj2[0].Type->size[0] * iobj_3->
    _pobj2[0].Type->size[1];
  iobj_3->_pobj2[0].Type->size[0] = 1;
  iobj_3->_pobj2[0].Type->size[1] = 5;
  Invers_emxEnsureCapacity_char_T(iobj_3->_pobj2[0].Type,
    InverseKinematics_B.obj_tmp);
  for (InverseKinematics_B.b_kstr = 0; InverseKinematics_B.b_kstr < 5;
       InverseKinematics_B.b_kstr++) {
    iobj_3->_pobj2[0].Type->data[InverseKinematics_B.b_kstr] =
      tmp_1[InverseKinematics_B.b_kstr];
  }

  InverseKinematic_emxInit_char_T(&switch_expression, 2);
  InverseKinematics_B.obj_tmp = switch_expression->size[0] *
    switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = iobj_3->_pobj2[0].Type->size[1];
  Invers_emxEnsureCapacity_char_T(switch_expression, InverseKinematics_B.obj_tmp);
  InverseKinematics_B.loop_ub_l = iobj_3->_pobj2[0].Type->size[1] - 1;
  for (InverseKinematics_B.b_kstr = 0; InverseKinematics_B.b_kstr <=
       InverseKinematics_B.loop_ub_l; InverseKinematics_B.b_kstr++) {
    InverseKinematics_B.obj_tmp = InverseKinematics_B.b_kstr;
    switch_expression->data[InverseKinematics_B.obj_tmp] = iobj_3->_pobj2[0].
      Type->data[InverseKinematics_B.obj_tmp];
  }

  for (InverseKinematics_B.b_kstr = 0; InverseKinematics_B.b_kstr < 8;
       InverseKinematics_B.b_kstr++) {
    InverseKinematics_B.b_j[InverseKinematics_B.b_kstr] =
      tmp_2[InverseKinematics_B.b_kstr];
  }

  InverseKinematics_B.b_bool_h = false;
  if (switch_expression->size[1] != 8) {
  } else {
    InverseKinematics_B.b_kstr = 1;
    do {
      exitg1 = 0;
      if (InverseKinematics_B.b_kstr - 1 < 8) {
        if (switch_expression->data[InverseKinematics_B.b_kstr - 1] !=
            InverseKinematics_B.b_j[InverseKinematics_B.b_kstr - 1]) {
          exitg1 = 1;
        } else {
          InverseKinematics_B.b_kstr++;
        }
      } else {
        InverseKinematics_B.b_bool_h = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (InverseKinematics_B.b_bool_h) {
    InverseKinematics_B.b_kstr = 0;
  } else {
    for (InverseKinematics_B.b_kstr = 0; InverseKinematics_B.b_kstr < 9;
         InverseKinematics_B.b_kstr++) {
      InverseKinematics_B.b_n[InverseKinematics_B.b_kstr] =
        tmp_3[InverseKinematics_B.b_kstr];
    }

    InverseKinematics_B.b_bool_h = false;
    if (switch_expression->size[1] != 9) {
    } else {
      InverseKinematics_B.b_kstr = 1;
      do {
        exitg1 = 0;
        if (InverseKinematics_B.b_kstr - 1 < 9) {
          if (switch_expression->data[InverseKinematics_B.b_kstr - 1] !=
              InverseKinematics_B.b_n[InverseKinematics_B.b_kstr - 1]) {
            exitg1 = 1;
          } else {
            InverseKinematics_B.b_kstr++;
          }
        } else {
          InverseKinematics_B.b_bool_h = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (InverseKinematics_B.b_bool_h) {
      InverseKinematics_B.b_kstr = 1;
    } else {
      InverseKinematics_B.b_kstr = -1;
    }
  }

  switch (InverseKinematics_B.b_kstr) {
   case 0:
    InverseKinematics_B.iv[0] = 0;
    InverseKinematics_B.iv[1] = 0;
    InverseKinematics_B.iv[2] = 1;
    InverseKinematics_B.iv[3] = 0;
    InverseKinematics_B.iv[4] = 0;
    InverseKinematics_B.iv[5] = 0;
    for (InverseKinematics_B.b_kstr = 0; InverseKinematics_B.b_kstr < 6;
         InverseKinematics_B.b_kstr++) {
      InverseKinematics_B.msubspace_data[InverseKinematics_B.b_kstr] =
        InverseKinematics_B.iv[InverseKinematics_B.b_kstr];
    }

    InverseKinematics_B.poslim_data[0] = -3.1415926535897931;
    InverseKinematics_B.poslim_data[1] = 3.1415926535897931;
    iobj_3->_pobj2[0].VelocityNumber = 1.0;
    iobj_3->_pobj2[0].PositionNumber = 1.0;
    iobj_3->_pobj2[0].JointAxisInternal[0] = 0.0;
    iobj_3->_pobj2[0].JointAxisInternal[1] = 0.0;
    iobj_3->_pobj2[0].JointAxisInternal[2] = 1.0;
    break;

   case 1:
    InverseKinematics_B.iv[0] = 0;
    InverseKinematics_B.iv[1] = 0;
    InverseKinematics_B.iv[2] = 0;
    InverseKinematics_B.iv[3] = 0;
    InverseKinematics_B.iv[4] = 0;
    InverseKinematics_B.iv[5] = 1;
    for (InverseKinematics_B.b_kstr = 0; InverseKinematics_B.b_kstr < 6;
         InverseKinematics_B.b_kstr++) {
      InverseKinematics_B.msubspace_data[InverseKinematics_B.b_kstr] =
        InverseKinematics_B.iv[InverseKinematics_B.b_kstr];
    }

    InverseKinematics_B.poslim_data[0] = -0.5;
    InverseKinematics_B.poslim_data[1] = 0.5;
    iobj_3->_pobj2[0].VelocityNumber = 1.0;
    iobj_3->_pobj2[0].PositionNumber = 1.0;
    iobj_3->_pobj2[0].JointAxisInternal[0] = 0.0;
    iobj_3->_pobj2[0].JointAxisInternal[1] = 0.0;
    iobj_3->_pobj2[0].JointAxisInternal[2] = 1.0;
    break;

   default:
    for (InverseKinematics_B.b_kstr = 0; InverseKinematics_B.b_kstr < 6;
         InverseKinematics_B.b_kstr++) {
      InverseKinematics_B.msubspace_data[InverseKinematics_B.b_kstr] = 0;
    }

    InverseKinematics_B.poslim_data[0] = 0.0;
    InverseKinematics_B.poslim_data[1] = 0.0;
    iobj_3->_pobj2[0].VelocityNumber = 0.0;
    iobj_3->_pobj2[0].PositionNumber = 0.0;
    iobj_3->_pobj2[0].JointAxisInternal[0] = 0.0;
    iobj_3->_pobj2[0].JointAxisInternal[1] = 0.0;
    iobj_3->_pobj2[0].JointAxisInternal[2] = 0.0;
    break;
  }

  InverseKinematics_B.obj_tmp = iobj_3->_pobj2[0].MotionSubspace->size[0] *
    iobj_3->_pobj2[0].MotionSubspace->size[1];
  iobj_3->_pobj2[0].MotionSubspace->size[0] = 6;
  iobj_3->_pobj2[0].MotionSubspace->size[1] = 1;
  Invers_emxEnsureCapacity_real_T(iobj_3->_pobj2[0].MotionSubspace,
    InverseKinematics_B.obj_tmp);
  for (InverseKinematics_B.b_kstr = 0; InverseKinematics_B.b_kstr < 6;
       InverseKinematics_B.b_kstr++) {
    iobj_3->_pobj2[0].MotionSubspace->data[InverseKinematics_B.b_kstr] =
      InverseKinematics_B.msubspace_data[InverseKinematics_B.b_kstr];
  }

  InverseKinematics_B.obj_tmp = iobj_3->_pobj2[0].PositionLimitsInternal->size[0]
    * iobj_3->_pobj2[0].PositionLimitsInternal->size[1];
  iobj_3->_pobj2[0].PositionLimitsInternal->size[0] = 1;
  iobj_3->_pobj2[0].PositionLimitsInternal->size[1] = 2;
  Invers_emxEnsureCapacity_real_T(iobj_3->_pobj2[0].PositionLimitsInternal,
    InverseKinematics_B.obj_tmp);
  for (InverseKinematics_B.b_kstr = 0; InverseKinematics_B.b_kstr < 2;
       InverseKinematics_B.b_kstr++) {
    iobj_3->_pobj2[0].PositionLimitsInternal->data[InverseKinematics_B.b_kstr] =
      InverseKinematics_B.poslim_data[InverseKinematics_B.b_kstr];
  }

  InverseKinematics_B.obj_tmp = iobj_3->_pobj2[0].HomePositionInternal->size[0];
  iobj_3->_pobj2[0].HomePositionInternal->size[0] = 1;
  Invers_emxEnsureCapacity_real_T(iobj_3->_pobj2[0].HomePositionInternal,
    InverseKinematics_B.obj_tmp);
  iobj_3->_pobj2[0].HomePositionInternal->data[0] = 0.0;
  body->JointInternal = &iobj_3->_pobj2[0];
  body->Index = -1.0;
  body->ParentIndex = -1.0;
  body->MassInternal = 1.0;
  body->CenterOfMassInternal[0] = 0.0;
  body->CenterOfMassInternal[1] = 0.0;
  body->CenterOfMassInternal[2] = 0.0;
  for (InverseKinematics_B.b_kstr = 0; InverseKinematics_B.b_kstr < 9;
       InverseKinematics_B.b_kstr++) {
    InverseKinematics_B.b_I_b[InverseKinematics_B.b_kstr] = 0;
  }

  InverseKinematics_B.b_I_b[0] = 1;
  InverseKinematics_B.b_I_b[4] = 1;
  InverseKinematics_B.b_I_b[8] = 1;
  for (InverseKinematics_B.b_kstr = 0; InverseKinematics_B.b_kstr < 9;
       InverseKinematics_B.b_kstr++) {
    body->InertiaInternal[InverseKinematics_B.b_kstr] =
      InverseKinematics_B.b_I_b[InverseKinematics_B.b_kstr];
  }

  for (InverseKinematics_B.b_kstr = 0; InverseKinematics_B.b_kstr < 36;
       InverseKinematics_B.b_kstr++) {
    InverseKinematics_B.msubspace_data[InverseKinematics_B.b_kstr] = 0;
  }

  for (InverseKinematics_B.b_kstr = 0; InverseKinematics_B.b_kstr < 6;
       InverseKinematics_B.b_kstr++) {
    InverseKinematics_B.msubspace_data[InverseKinematics_B.b_kstr + 6 *
      InverseKinematics_B.b_kstr] = 1;
  }

  for (InverseKinematics_B.b_kstr = 0; InverseKinematics_B.b_kstr < 36;
       InverseKinematics_B.b_kstr++) {
    body->SpatialInertia[InverseKinematics_B.b_kstr] =
      InverseKinematics_B.msubspace_data[InverseKinematics_B.b_kstr];
  }

  body->CollisionsInternal = Inver_CollisionSet_CollisionSet(iobj_0_0, 0.0);
  iobj_3->Base.Index = 0.0;
  InverseKinematics_rand(InverseKinematics_B.unusedExpr);
  iobj_0_0 = &iobj_3->_pobj1[1];
  iobj_1_0 = &iobj_3->_pobj2[1];
  body = &iobj_3->_pobj0[0];
  iobj_3->Bodies[0] = InverseKine_RigidBody_RigidBody(&(&(&iobj_3->_pobj0[0])[0])
    [0], &(&iobj_0_0[0])[0], &(&iobj_1_0[0])[0]);
  iobj_3->Bodies[1] = InverseKi_RigidBody_RigidBody_n(&(&body[0])[1],
    &(&iobj_0_0[0])[1], &(&iobj_1_0[0])[1]);
  iobj_3->Bodies[2] = InverseK_RigidBody_RigidBody_nl(&(&body[0])[2],
    &(&iobj_0_0[0])[2], &(&iobj_1_0[0])[2]);
  iobj_3->Bodies[3] = Inverse_RigidBody_RigidBody_nly(&(&body[0])[3],
    &(&iobj_0_0[0])[3], &(&iobj_1_0[0])[3]);
  iobj_3->Bodies[4] = Invers_RigidBody_RigidBody_nlyk(&(&body[0])[4],
    &(&iobj_0_0[0])[4], &(&iobj_1_0[0])[4]);
  iobj_3->Bodies[5] = Inver_RigidBody_RigidBody_nlyk3(&(&body[0])[5],
    &(&iobj_0_0[0])[5], &(&iobj_1_0[0])[5]);
  iobj_3->NumBodies = 0.0;
  iobj_3->NumNonFixedBodies = 0.0;
  iobj_3->PositionNumber = 0.0;
  iobj_3->VelocityNumber = 0.0;
  InverseKinematics_rand(InverseKinematics_B.unusedExpr);
  for (InverseKinematics_B.b_kstr = 0; InverseKinematics_B.b_kstr < 6;
       InverseKinematics_B.b_kstr++) {
    iobj_3->PositionDoFMap[InverseKinematics_B.b_kstr] = 0.0;
  }

  for (InverseKinematics_B.b_kstr = 0; InverseKinematics_B.b_kstr < 6;
       InverseKinematics_B.b_kstr++) {
    iobj_3->PositionDoFMap[InverseKinematics_B.b_kstr + 6] = -1.0;
  }

  for (InverseKinematics_B.b_kstr = 0; InverseKinematics_B.b_kstr < 6;
       InverseKinematics_B.b_kstr++) {
    iobj_3->VelocityDoFMap[InverseKinematics_B.b_kstr] = 0.0;
  }

  for (InverseKinematics_B.b_kstr = 0; InverseKinematics_B.b_kstr < 6;
       InverseKinematics_B.b_kstr++) {
    iobj_3->VelocityDoFMap[InverseKinematics_B.b_kstr + 6] = -1.0;
  }

  InverseKinematics_B.obj_tmp = switch_expression->size[0] *
    switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = rigidbodytree->Base.NameInternal->size[1];
  Invers_emxEnsureCapacity_char_T(switch_expression, InverseKinematics_B.obj_tmp);
  InverseKinematics_B.loop_ub_l = rigidbodytree->Base.NameInternal->size[1] - 1;
  for (InverseKinematics_B.b_kstr = 0; InverseKinematics_B.b_kstr <=
       InverseKinematics_B.loop_ub_l; InverseKinematics_B.b_kstr++) {
    InverseKinematics_B.obj_tmp = InverseKinematics_B.b_kstr;
    switch_expression->data[InverseKinematics_B.obj_tmp] =
      rigidbodytree->Base.NameInternal->data[InverseKinematics_B.obj_tmp];
  }

  InverseKinematic_emxInit_char_T(&bname, 2);
  InverseKinematics_B.bid_c = -1.0;
  InverseKinematics_B.obj_tmp = bname->size[0] * bname->size[1];
  bname->size[0] = 1;
  bname->size[1] = iobj_3->Base.NameInternal->size[1];
  Invers_emxEnsureCapacity_char_T(bname, InverseKinematics_B.obj_tmp);
  InverseKinematics_B.loop_ub_l = iobj_3->Base.NameInternal->size[1] - 1;
  for (InverseKinematics_B.b_kstr = 0; InverseKinematics_B.b_kstr <=
       InverseKinematics_B.loop_ub_l; InverseKinematics_B.b_kstr++) {
    InverseKinematics_B.obj_tmp = InverseKinematics_B.b_kstr;
    bname->data[InverseKinematics_B.obj_tmp] = iobj_3->Base.NameInternal->
      data[InverseKinematics_B.obj_tmp];
  }

  if (InverseKinematics_strcmp(bname, switch_expression)) {
    InverseKinematics_B.bid_c = 0.0;
  } else {
    boolean_T exitg2;
    InverseKinematics_B.b_m = iobj_3->NumBodies;
    InverseKinematics_B.iobj_3 = 0;
    exitg2 = false;
    while ((!exitg2) && (InverseKinematics_B.iobj_3 <= static_cast<int32_T>
                         (InverseKinematics_B.b_m) - 1)) {
      body = iobj_3->Bodies[InverseKinematics_B.iobj_3];
      InverseKinematics_B.obj_tmp = bname->size[0] * bname->size[1];
      bname->size[0] = 1;
      bname->size[1] = body->NameInternal->size[1];
      Invers_emxEnsureCapacity_char_T(bname, InverseKinematics_B.obj_tmp);
      InverseKinematics_B.loop_ub_l = body->NameInternal->size[1] - 1;
      for (InverseKinematics_B.b_kstr = 0; InverseKinematics_B.b_kstr <=
           InverseKinematics_B.loop_ub_l; InverseKinematics_B.b_kstr++) {
        InverseKinematics_B.obj_tmp = InverseKinematics_B.b_kstr;
        bname->data[InverseKinematics_B.obj_tmp] = body->NameInternal->
          data[InverseKinematics_B.obj_tmp];
      }

      if (InverseKinematics_strcmp(bname, switch_expression)) {
        InverseKinematics_B.bid_c = static_cast<real_T>
          (InverseKinematics_B.iobj_3) + 1.0;
        exitg2 = true;
      } else {
        InverseKinematics_B.iobj_3++;
      }
    }
  }

  if ((!(InverseKinematics_B.bid_c == 0.0)) && (InverseKinematics_B.bid_c < 0.0))
  {
    InverseKinematics_B.obj_tmp = iobj_3->Base.NameInternal->size[0] *
      iobj_3->Base.NameInternal->size[1];
    iobj_3->Base.NameInternal->size[0] = 1;
    iobj_3->Base.NameInternal->size[1] = switch_expression->size[1];
    Invers_emxEnsureCapacity_char_T(iobj_3->Base.NameInternal,
      InverseKinematics_B.obj_tmp);
    InverseKinematics_B.loop_ub_l = switch_expression->size[1] - 1;
    for (InverseKinematics_B.b_kstr = 0; InverseKinematics_B.b_kstr <=
         InverseKinematics_B.loop_ub_l; InverseKinematics_B.b_kstr++) {
      InverseKinematics_B.obj_tmp = InverseKinematics_B.b_kstr;
      iobj_3->Base.NameInternal->data[InverseKinematics_B.obj_tmp] =
        switch_expression->data[InverseKinematics_B.obj_tmp];
    }
  }

  InverseKinematic_emxFree_char_T(&switch_expression);
  iobj_0_0 = rigidbodytree->Base.CollisionsInternal;
  newObj = Inver_CollisionSet_CollisionSet(&(&iobj_2[0])[0],
    iobj_0_0->MaxElements);
  newObj->Size = iobj_0_0->Size;
  InverseKinematics_B.b_m = iobj_0_0->Size;
  InverseKinematics_B.b_kstr = static_cast<int32_T>(InverseKinematics_B.b_m) - 1;
  for (InverseKinematics_B.iobj_3 = 0; InverseKinematics_B.iobj_3 <=
       InverseKinematics_B.b_kstr; InverseKinematics_B.iobj_3++) {
    InverseKinematics_B.obj_tmp = InverseKinematics_B.iobj_3;
    obj_0 = iobj_0_0->CollisionGeometries->data[InverseKinematics_B.obj_tmp];
    newObj->CollisionGeometries->data[InverseKinematics_B.obj_tmp] = obj_0;
  }

  iobj_3->Base.CollisionsInternal = newObj;
  if (rigidbodytree->NumBodies >= 1.0) {
    body = rigidbodytree->Bodies[0];
    InverseKinematics_B.bid_c = body->ParentIndex;
    if (InverseKinematics_B.bid_c > 0.0) {
      parent = rigidbodytree->Bodies[static_cast<int32_T>
        (InverseKinematics_B.bid_c) - 1];
    } else {
      parent = &rigidbodytree->Base;
    }

    InverseKinematics_B.obj_tmp = bname->size[0] * bname->size[1];
    bname->size[0] = 1;
    bname->size[1] = parent->NameInternal->size[1];
    Invers_emxEnsureCapacity_char_T(bname, InverseKinematics_B.obj_tmp);
    InverseKinematics_B.loop_ub_l = parent->NameInternal->size[1] - 1;
    for (InverseKinematics_B.b_kstr = 0; InverseKinematics_B.b_kstr <=
         InverseKinematics_B.loop_ub_l; InverseKinematics_B.b_kstr++) {
      InverseKinematics_B.obj_tmp = InverseKinematics_B.b_kstr;
      bname->data[InverseKinematics_B.obj_tmp] = parent->NameInternal->
        data[InverseKinematics_B.obj_tmp];
    }

    InverseKi_RigidBodyTree_addBody(iobj_3, body, bname, &(&iobj_0[0])[0],
      &(&iobj_1[0])[0], &(&iobj_2[0])[1]);
  }

  if (rigidbodytree->NumBodies >= 2.0) {
    body = rigidbodytree->Bodies[1];
    InverseKinematics_B.bid_c = body->ParentIndex;
    if (InverseKinematics_B.bid_c > 0.0) {
      parent = rigidbodytree->Bodies[static_cast<int32_T>
        (InverseKinematics_B.bid_c) - 1];
    } else {
      parent = &rigidbodytree->Base;
    }

    InverseKinematics_B.obj_tmp = bname->size[0] * bname->size[1];
    bname->size[0] = 1;
    bname->size[1] = parent->NameInternal->size[1];
    Invers_emxEnsureCapacity_char_T(bname, InverseKinematics_B.obj_tmp);
    InverseKinematics_B.loop_ub_l = parent->NameInternal->size[1] - 1;
    for (InverseKinematics_B.b_kstr = 0; InverseKinematics_B.b_kstr <=
         InverseKinematics_B.loop_ub_l; InverseKinematics_B.b_kstr++) {
      InverseKinematics_B.obj_tmp = InverseKinematics_B.b_kstr;
      bname->data[InverseKinematics_B.obj_tmp] = parent->NameInternal->
        data[InverseKinematics_B.obj_tmp];
    }

    InverseKi_RigidBodyTree_addBody(iobj_3, body, bname, &(&iobj_0[0])[2],
      &(&iobj_1[0])[1], &(&iobj_2[0])[3]);
  }

  if (rigidbodytree->NumBodies >= 3.0) {
    body = rigidbodytree->Bodies[2];
    InverseKinematics_B.bid_c = body->ParentIndex;
    if (InverseKinematics_B.bid_c > 0.0) {
      parent = rigidbodytree->Bodies[static_cast<int32_T>
        (InverseKinematics_B.bid_c) - 1];
    } else {
      parent = &rigidbodytree->Base;
    }

    InverseKinematics_B.obj_tmp = bname->size[0] * bname->size[1];
    bname->size[0] = 1;
    bname->size[1] = parent->NameInternal->size[1];
    Invers_emxEnsureCapacity_char_T(bname, InverseKinematics_B.obj_tmp);
    InverseKinematics_B.loop_ub_l = parent->NameInternal->size[1] - 1;
    for (InverseKinematics_B.b_kstr = 0; InverseKinematics_B.b_kstr <=
         InverseKinematics_B.loop_ub_l; InverseKinematics_B.b_kstr++) {
      InverseKinematics_B.obj_tmp = InverseKinematics_B.b_kstr;
      bname->data[InverseKinematics_B.obj_tmp] = parent->NameInternal->
        data[InverseKinematics_B.obj_tmp];
    }

    InverseKi_RigidBodyTree_addBody(iobj_3, body, bname, &(&iobj_0[0])[4],
      &(&iobj_1[0])[2], &(&iobj_2[0])[5]);
  }

  if (rigidbodytree->NumBodies >= 4.0) {
    body = rigidbodytree->Bodies[3];
    InverseKinematics_B.bid_c = body->ParentIndex;
    if (InverseKinematics_B.bid_c > 0.0) {
      parent = rigidbodytree->Bodies[static_cast<int32_T>
        (InverseKinematics_B.bid_c) - 1];
    } else {
      parent = &rigidbodytree->Base;
    }

    InverseKinematics_B.obj_tmp = bname->size[0] * bname->size[1];
    bname->size[0] = 1;
    bname->size[1] = parent->NameInternal->size[1];
    Invers_emxEnsureCapacity_char_T(bname, InverseKinematics_B.obj_tmp);
    InverseKinematics_B.loop_ub_l = parent->NameInternal->size[1] - 1;
    for (InverseKinematics_B.b_kstr = 0; InverseKinematics_B.b_kstr <=
         InverseKinematics_B.loop_ub_l; InverseKinematics_B.b_kstr++) {
      InverseKinematics_B.obj_tmp = InverseKinematics_B.b_kstr;
      bname->data[InverseKinematics_B.obj_tmp] = parent->NameInternal->
        data[InverseKinematics_B.obj_tmp];
    }

    InverseKi_RigidBodyTree_addBody(iobj_3, body, bname, &(&iobj_0[0])[6],
      &(&iobj_1[0])[3], &(&iobj_2[0])[7]);
  }

  if (rigidbodytree->NumBodies >= 5.0) {
    body = rigidbodytree->Bodies[4];
    InverseKinematics_B.bid_c = body->ParentIndex;
    if (InverseKinematics_B.bid_c > 0.0) {
      parent = rigidbodytree->Bodies[static_cast<int32_T>
        (InverseKinematics_B.bid_c) - 1];
    } else {
      parent = &rigidbodytree->Base;
    }

    InverseKinematics_B.obj_tmp = bname->size[0] * bname->size[1];
    bname->size[0] = 1;
    bname->size[1] = parent->NameInternal->size[1];
    Invers_emxEnsureCapacity_char_T(bname, InverseKinematics_B.obj_tmp);
    InverseKinematics_B.loop_ub_l = parent->NameInternal->size[1] - 1;
    for (InverseKinematics_B.b_kstr = 0; InverseKinematics_B.b_kstr <=
         InverseKinematics_B.loop_ub_l; InverseKinematics_B.b_kstr++) {
      InverseKinematics_B.obj_tmp = InverseKinematics_B.b_kstr;
      bname->data[InverseKinematics_B.obj_tmp] = parent->NameInternal->
        data[InverseKinematics_B.obj_tmp];
    }

    InverseKi_RigidBodyTree_addBody(iobj_3, body, bname, &(&iobj_0[0])[8],
      &(&iobj_1[0])[4], &(&iobj_2[0])[9]);
  }

  if (rigidbodytree->NumBodies >= 6.0) {
    body = rigidbodytree->Bodies[5];
    InverseKinematics_B.bid_c = body->ParentIndex;
    if (InverseKinematics_B.bid_c > 0.0) {
      parent = rigidbodytree->Bodies[static_cast<int32_T>
        (InverseKinematics_B.bid_c) - 1];
    } else {
      parent = &rigidbodytree->Base;
    }

    InverseKinematics_B.obj_tmp = bname->size[0] * bname->size[1];
    bname->size[0] = 1;
    bname->size[1] = parent->NameInternal->size[1];
    Invers_emxEnsureCapacity_char_T(bname, InverseKinematics_B.obj_tmp);
    InverseKinematics_B.loop_ub_l = parent->NameInternal->size[1] - 1;
    for (InverseKinematics_B.b_kstr = 0; InverseKinematics_B.b_kstr <=
         InverseKinematics_B.loop_ub_l; InverseKinematics_B.b_kstr++) {
      InverseKinematics_B.obj_tmp = InverseKinematics_B.b_kstr;
      bname->data[InverseKinematics_B.obj_tmp] = parent->NameInternal->
        data[InverseKinematics_B.obj_tmp];
    }

    InverseKi_RigidBodyTree_addBody(iobj_3, body, bname, &(&iobj_0[0])[10],
      &(&iobj_1[0])[5], &(&iobj_2[0])[11]);
  }

  InverseKinematic_emxFree_char_T(&bname);
  obj->RigidBodyTreeInternal = iobj_3;
}

void InverseKinematics::InverseKinemat_SystemCore_setup
  (robotics_slmanip_internal_blo_T *obj)
{
  b_inverseKinematics_InverseKi_T *obj_0;
  c_rigidBodyJoint_InverseKinem_T *iobj_0;
  h_robotics_core_internal_Erro_T *iobj_4;
  l_robotics_manip_internal_Col_T *iobj_2;
  t_robotics_manip_internal_Rig_T *iobj_1;
  v_robotics_manip_internal_Rig_T *iobj_3;
  static const sdAmwXbnJnEmimT0NaJRtAD_Inver_T tmp = { 0.0,/* tv_sec */
    0.0                                /* tv_nsec */
  };

  static const char_T tmp_0[18] = { 'L', 'e', 'v', 'e', 'n', 'b', 'e', 'r', 'g',
    'M', 'a', 'r', 'q', 'u', 'a', 'r', 'd', 't' };

  obj->isInitialized = 1;
  Inv_RigidBodyTree_RigidBodyTree(&obj->TreeInternal);
  obj_0 = &obj->IKInternal;
  obj->IKInternal.isInitialized = 0;
  iobj_0 = &obj->IKInternal._pobj1[0];
  iobj_1 = &obj->IKInternal._pobj2[0];
  iobj_2 = &obj->IKInternal._pobj3[0];
  iobj_3 = &obj->IKInternal._pobj4;
  iobj_4 = &obj->IKInternal._pobj5;
  inverseKinematics_set_RigidBody(&obj->IKInternal, &obj->TreeInternal,
    &(&(&iobj_0[0])[0])[0], &(&(&iobj_1[0])[0])[0], &(&(&iobj_2[0])[0])[0],
    iobj_3);
  iobj_4->MaxNumIteration = 1500.0;
  iobj_4->MaxTime = 10.0;
  iobj_4->SolutionTolerance = 1.0E-6;
  iobj_4->ConstraintsOn = true;
  iobj_4->RandomRestart = true;
  iobj_4->StepTolerance = 1.0E-12;
  iobj_4->GradientTolerance = 5.0E-9;
  iobj_4->ErrorChangeTolerance = 1.0E-12;
  iobj_4->DampingBias = 0.0025;
  iobj_4->UseErrorDamping = true;
  for (InverseKinematics_B.ret = 0; InverseKinematics_B.ret < 18;
       InverseKinematics_B.ret++) {
    iobj_4->Name[InverseKinematics_B.ret] = tmp_0[InverseKinematics_B.ret];
  }

  iobj_4->TimeObj.StartTime = tmp;
  iobj_4->TimeObjInternal.StartTime = tmp;
  obj_0->Solver = iobj_4;
  iobj_4 = obj_0->Solver;
  InverseKinematics_B.params_ErrorChangeTolerance = iobj_4->ErrorChangeTolerance;
  InverseKinematics_B.params_DampingBias = iobj_4->DampingBias;
  InverseKinematics_B.params_UseErrorDamping = iobj_4->UseErrorDamping;
  for (InverseKinematics_B.ret = 0; InverseKinematics_B.ret < 18;
       InverseKinematics_B.ret++) {
    InverseKinematics_B.switch_expression[InverseKinematics_B.ret] =
      obj_0->Solver->Name[InverseKinematics_B.ret];
  }

  for (InverseKinematics_B.ret = 0; InverseKinematics_B.ret < 18;
       InverseKinematics_B.ret++) {
    InverseKinematics_B.b_d[InverseKinematics_B.ret] =
      tmp_0[InverseKinematics_B.ret];
  }

  InverseKinematics_B.ret = memcmp(&InverseKinematics_B.switch_expression[0],
    &InverseKinematics_B.b_d[0], 18);
  if (InverseKinematics_B.ret == 0) {
    InverseKinematics_B.params_ErrorChangeTolerance = 1.0E-12;
    InverseKinematics_B.params_DampingBias = 0.0025;
    InverseKinematics_B.params_UseErrorDamping = true;
  }

  iobj_4 = obj_0->Solver;
  iobj_4->MaxNumIteration = 1500.0;
  iobj_4->MaxTime = 10.0;
  iobj_4->GradientTolerance = 1.0E-7;
  iobj_4->SolutionTolerance = 1.0E-6;
  iobj_4->ConstraintsOn = true;
  iobj_4->RandomRestart = false;
  iobj_4->StepTolerance = 1.0E-14;
  iobj_4->ErrorChangeTolerance = InverseKinematics_B.params_ErrorChangeTolerance;
  iobj_4->DampingBias = InverseKinematics_B.params_DampingBias;
  iobj_4->UseErrorDamping = InverseKinematics_B.params_UseErrorDamping;
  obj_0->matlabCodegenIsDeleted = false;
}

void InverseKinematics::InverseKinem_SystemCore_setup_n
  (ros_slros2_internal_block_Pub_T *obj)
{
  rmw_qos_durability_policy_t durability;
  rmw_qos_history_policy_t history;
  rmw_qos_profile_t qos_profile;
  rmw_qos_reliability_policy_t reliability;
  char_T b_zeroDelimTopic[12];
  static const char_T tmp[11] = { '/', 'M', 'o', 't', 'o', 'r', 's', '/', 'R',
    'e', 'f' };

  obj->isSetupComplete = false;
  obj->isInitialized = 1;
  qos_profile = rmw_qos_profile_default;
  history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
  reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
  SET_QOS_VALUES(qos_profile, history, (size_t)1.0, durability, reliability);
  for (int32_T i = 0; i < 11; i++) {
    b_zeroDelimTopic[i] = tmp[i];
  }

  b_zeroDelimTopic[11] = '\x00';
  ros2::matlab::create_Pub_InverseKinematics_466(&b_zeroDelimTopic[0],
    qos_profile);
  obj->isSetupComplete = true;
}

void InverseKinematics::RigidBodyTree_get_JointPosition
  (v_robotics_manip_internal_Rig_T *obj, emxArray_real_T_InverseKinema_T *limits)
{
  c_rigidBodyJoint_InverseKinem_T *obj_0;
  emxArray_char_T_InverseKinema_T *a;
  t_robotics_manip_internal_Rig_T *body;
  real_T k;
  real_T pnum;
  int32_T b_kstr;
  int32_T loop_ub;
  char_T b[5];
  static const char_T tmp[5] = { 'f', 'i', 'x', 'e', 'd' };

  int32_T c_tmp;
  int32_T i;
  i = limits->size[0] * limits->size[1];
  limits->size[0] = static_cast<int32_T>(obj->PositionNumber);
  limits->size[1] = 2;
  Invers_emxEnsureCapacity_real_T(limits, i);
  loop_ub = (static_cast<int32_T>(obj->PositionNumber) << 1) - 1;
  if (loop_ub >= 0) {
    memset(&limits->data[0], 0, (loop_ub + 1) * sizeof(real_T));
  }

  k = 1.0;
  pnum = obj->NumBodies;
  c_tmp = static_cast<int32_T>(pnum) - 1;
  if (static_cast<int32_T>(pnum) - 1 >= 0) {
    for (b_kstr = 0; b_kstr < 5; b_kstr++) {
      b[b_kstr] = tmp[b_kstr];
    }
  }

  InverseKinematic_emxInit_char_T(&a, 2);
  for (int32_T limits_0 = 0; limits_0 <= c_tmp; limits_0++) {
    boolean_T b_bool;
    body = obj->Bodies[limits_0];
    i = a->size[0] * a->size[1];
    a->size[0] = 1;
    a->size[1] = body->JointInternal->Type->size[1];
    Invers_emxEnsureCapacity_char_T(a, i);
    loop_ub = body->JointInternal->Type->size[1] - 1;
    for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
      a->data[b_kstr] = body->JointInternal->Type->data[b_kstr];
    }

    b_bool = false;
    if (a->size[1] != 5) {
    } else {
      b_kstr = 1;
      int32_T exitg1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 5) {
          if (a->data[b_kstr - 1] != b[b_kstr - 1]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (!b_bool) {
      int32_T f;
      pnum = body->JointInternal->PositionNumber;
      pnum += k;
      if (k > pnum - 1.0) {
        f = 0;
      } else {
        f = static_cast<int32_T>(k) - 1;
      }

      obj_0 = body->JointInternal;
      loop_ub = obj_0->PositionLimitsInternal->size[0];
      for (b_kstr = 0; b_kstr < 2; b_kstr++) {
        for (i = 0; i < loop_ub; i++) {
          limits->data[(f + i) + limits->size[0] * b_kstr] =
            obj_0->PositionLimitsInternal->data[obj_0->
            PositionLimitsInternal->size[0] * b_kstr + i];
        }
      }

      k = pnum;
    }
  }

  InverseKinematic_emxFree_char_T(&a);
}

void InverseKinematics::InverseKinematic_emxInit_int8_T
  (emxArray_int8_T_InverseKinema_T **pEmxArray, int32_T numDimensions)
{
  emxArray_int8_T_InverseKinema_T *emxArray;
  *pEmxArray = static_cast<emxArray_int8_T_InverseKinema_T *>(malloc(sizeof
    (emxArray_int8_T_InverseKinema_T)));
  emxArray = *pEmxArray;
  emxArray->data = static_cast<int8_T *>(NULL);
  emxArray->numDimensions = numDimensions;
  emxArray->size = static_cast<int32_T *>(malloc(sizeof(int32_T) * numDimensions));
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (int32_T i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

void InverseKinematics::Invers_emxEnsureCapacity_int8_T
  (emxArray_int8_T_InverseKinema_T *emxArray, int32_T oldNumel)
{
  int32_T i;
  int32_T newNumel;
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  newNumel = 1;
  for (i = 0; i < emxArray->numDimensions; i++) {
    newNumel *= emxArray->size[i];
  }

  if (newNumel > emxArray->allocatedSize) {
    i = emxArray->allocatedSize;
    if (i < 16) {
      i = 16;
    }

    while (i < newNumel) {
      if (i > 1073741823) {
        i = MAX_int32_T;
      } else {
        i <<= 1;
      }
    }

    newData = calloc(static_cast<uint32_T>(i), sizeof(int8_T));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(int8_T) * oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }

    emxArray->data = static_cast<int8_T *>(newData);
    emxArray->allocatedSize = i;
    emxArray->canFreeData = true;
  }
}

void InverseKinematics::InverseKinematic_emxFree_int8_T
  (emxArray_int8_T_InverseKinema_T **pEmxArray)
{
  if (*pEmxArray != static_cast<emxArray_int8_T_InverseKinema_T *>(NULL)) {
    if (((*pEmxArray)->data != static_cast<int8_T *>(NULL)) && (*pEmxArray)
        ->canFreeData) {
      free((*pEmxArray)->data);
    }

    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = static_cast<emxArray_int8_T_InverseKinema_T *>(NULL);
  }
}

void InverseKinematics::InverseKin_binary_expand_op_nly(boolean_T in1[4], const
  real_T in2[4], const emxArray_real_T_InverseKinema_T *in3)
{
  int32_T stride_0_0;
  stride_0_0 = (in3->size[0] != 1);
  in1[0] = (in2[0] <= in3->data[in3->size[0]] + 4.4408920985006262E-16);
  in1[1] = (in2[1] <= in3->data[stride_0_0 + in3->size[0]] +
            4.4408920985006262E-16);
  in1[2] = (in2[2] <= in3->data[(stride_0_0 << 1) + in3->size[0]] +
            4.4408920985006262E-16);
  in1[3] = (in2[3] <= in3->data[3 * stride_0_0 + in3->size[0]] +
            4.4408920985006262E-16);
}

void InverseKinematics::InverseKine_binary_expand_op_nl(boolean_T in1[4], const
  real_T in2[4], const emxArray_real_T_InverseKinema_T *in3)
{
  int32_T stride_0_0;
  stride_0_0 = (in3->size[0] != 1);
  in1[0] = (in2[0] >= in3->data[0] - 4.4408920985006262E-16);
  in1[1] = (in2[1] >= in3->data[stride_0_0] - 4.4408920985006262E-16);
  in1[2] = (in2[2] >= in3->data[stride_0_0 << 1] - 4.4408920985006262E-16);
  in1[3] = (in2[3] >= in3->data[3 * stride_0_0] - 4.4408920985006262E-16);
}

void InverseKinematics::InverseKinematics_eml_find(const boolean_T x[4], int32_T
  i_data[], int32_T *i_size)
{
  int32_T b_ii;
  int32_T idx;
  boolean_T exitg1;
  idx = 0;
  b_ii = 1;
  exitg1 = false;
  while ((!exitg1) && (b_ii - 1 < 4)) {
    if (x[b_ii - 1]) {
      idx++;
      i_data[idx - 1] = b_ii;
      if (idx >= 4) {
        exitg1 = true;
      } else {
        b_ii++;
      }
    } else {
      b_ii++;
    }
  }

  if (idx < 1) {
    idx = 0;
  }

  *i_size = idx;
}

void InverseKinematics::InverseKinematics_tic(real_T *tstart_tv_sec, real_T
  *tstart_tv_nsec)
{
  coderTimespec b_timespec;
  if (!InverseKinematics_DW.method_not_empty) {
    InverseKinematics_DW.method_not_empty = true;
    coderInitTimeFunctions(&InverseKinematics_DW.freq);
  }

  coderTimeClockGettimeMonotonic(&b_timespec, InverseKinematics_DW.freq);
  *tstart_tv_sec = b_timespec.tv_sec;
  *tstart_tv_nsec = b_timespec.tv_nsec;
}

void InverseKinematics::I_RigidBodyTree_ancestorIndices
  (v_robotics_manip_internal_Rig_T *obj, t_robotics_manip_internal_Rig_T *body,
   emxArray_real_T_InverseKinema_T *indices)
{
  InverseKinematics_B.loop_ub_cn = indices->size[0] * indices->size[1];
  indices->size[0] = 1;
  indices->size[1] = static_cast<int32_T>(obj->NumBodies + 1.0);
  Invers_emxEnsureCapacity_real_T(indices, InverseKinematics_B.loop_ub_cn);
  InverseKinematics_B.loop_ub_cn = static_cast<int32_T>(obj->NumBodies + 1.0) -
    1;
  if (InverseKinematics_B.loop_ub_cn >= 0) {
    memset(&indices->data[0], 0, (InverseKinematics_B.loop_ub_cn + 1) * sizeof
           (real_T));
  }

  InverseKinematics_B.i = 2.0;
  indices->data[0] = body->Index;
  while (body->ParentIndex > 0.0) {
    body = obj->Bodies[static_cast<int32_T>(body->ParentIndex) - 1];
    indices->data[static_cast<int32_T>(InverseKinematics_B.i) - 1] = body->Index;
    InverseKinematics_B.i++;
  }

  if (body->Index > 0.0) {
    indices->data[static_cast<int32_T>(InverseKinematics_B.i) - 1] =
      body->ParentIndex;
    InverseKinematics_B.i++;
  }

  InverseKinematics_B.loop_ub_tmp = static_cast<int32_T>(InverseKinematics_B.i -
    1.0);
  for (InverseKinematics_B.loop_ub_cn = 0; InverseKinematics_B.loop_ub_cn <
       InverseKinematics_B.loop_ub_tmp; InverseKinematics_B.loop_ub_cn++) {
  }

  InverseKinematics_B.loop_ub_cn = indices->size[0] * indices->size[1];
  indices->size[0] = 1;
  indices->size[1] = static_cast<int32_T>(InverseKinematics_B.i - 1.0);
  Invers_emxEnsureCapacity_real_T(indices, InverseKinematics_B.loop_ub_cn);
}

void InverseKinematics::RigidBodyTree_kinematicPathInte
  (v_robotics_manip_internal_Rig_T *obj, t_robotics_manip_internal_Rig_T *body1,
   t_robotics_manip_internal_Rig_T *body2, emxArray_real_T_InverseKinema_T
   *indices)
{
  emxArray_real_T_InverseKinema_T *ancestorIndices1;
  emxArray_real_T_InverseKinema_T *ancestorIndices2;
  boolean_T exitg1;
  InverseKinematic_emxInit_real_T(&ancestorIndices1, 2);
  InverseKinematic_emxInit_real_T(&ancestorIndices2, 2);
  I_RigidBodyTree_ancestorIndices(obj, body1, ancestorIndices1);
  I_RigidBodyTree_ancestorIndices(obj, body2, ancestorIndices2);
  if (static_cast<real_T>(ancestorIndices1->size[1]) <= ancestorIndices2->size[1])
  {
    InverseKinematics_B.minPathLength = ancestorIndices1->size[1];
  } else {
    InverseKinematics_B.minPathLength = ancestorIndices2->size[1];
  }

  InverseKinematics_B.b_i_o = 0;
  exitg1 = false;
  while ((!exitg1) && (InverseKinematics_B.b_i_o <=
                       InverseKinematics_B.minPathLength - 2)) {
    if (ancestorIndices1->data[(ancestorIndices1->size[1] -
         InverseKinematics_B.b_i_o) - 2] != ancestorIndices2->data
        [(ancestorIndices2->size[1] - InverseKinematics_B.b_i_o) - 2]) {
      InverseKinematics_B.minPathLength = InverseKinematics_B.b_i_o + 1;
      exitg1 = true;
    } else {
      InverseKinematics_B.b_i_o++;
    }
  }

  InverseKinematics_B.b_i_o = ancestorIndices1->size[1] -
    InverseKinematics_B.minPathLength;
  if (InverseKinematics_B.b_i_o < 1) {
    InverseKinematics_B.e_l = -1;
  } else {
    InverseKinematics_B.e_l = InverseKinematics_B.b_i_o - 1;
  }

  InverseKinematics_B.b_i_o = ancestorIndices2->size[1] -
    InverseKinematics_B.minPathLength;
  if (InverseKinematics_B.b_i_o < 1) {
    InverseKinematics_B.j = 0;
    InverseKinematics_B.h = 1;
    InverseKinematics_B.b_i_o = -1;
  } else {
    InverseKinematics_B.j = InverseKinematics_B.b_i_o - 1;
    InverseKinematics_B.h = -1;
    InverseKinematics_B.b_i_o = 0;
  }

  InverseKinematics_B.i_m = indices->size[0] * indices->size[1];
  indices->size[0] = 1;
  InverseKinematics_B.loop_ub_m = div_s32(InverseKinematics_B.b_i_o -
    InverseKinematics_B.j, InverseKinematics_B.h);
  indices->size[1] = (InverseKinematics_B.loop_ub_m + InverseKinematics_B.e_l) +
    3;
  Invers_emxEnsureCapacity_real_T(indices, InverseKinematics_B.i_m);
  if (InverseKinematics_B.e_l >= 0) {
    memcpy(&indices->data[0], &ancestorIndices1->data[0],
           (InverseKinematics_B.e_l + 1) * sizeof(real_T));
  }

  indices->data[InverseKinematics_B.e_l + 1] = ancestorIndices1->
    data[ancestorIndices1->size[1] - InverseKinematics_B.minPathLength];
  InverseKinematic_emxFree_real_T(&ancestorIndices1);
  for (InverseKinematics_B.b_i_o = 0; InverseKinematics_B.b_i_o <=
       InverseKinematics_B.loop_ub_m; InverseKinematics_B.b_i_o++) {
    indices->data[(InverseKinematics_B.b_i_o + InverseKinematics_B.e_l) + 2] =
      ancestorIndices2->data[InverseKinematics_B.h * InverseKinematics_B.b_i_o +
      InverseKinematics_B.j];
  }

  InverseKinematic_emxFree_real_T(&ancestorIndices2);
}

void InverseKinematics::In_rigidBodyJoint_get_JointAxis(const
  c_rigidBodyJoint_InverseKinem_T *obj, real_T ax[3])
{
  int32_T b_kstr;
  char_T b_0[9];
  char_T b[8];
  boolean_T b_bool;
  static const char_T tmp[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_0[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  int32_T exitg1;
  boolean_T guard1 = false;
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp[b_kstr];
  }

  b_bool = false;
  if (obj->Type->size[1] != 8) {
  } else {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        if (obj->Type->data[b_kstr - 1] != b[b_kstr - 1]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  guard1 = false;
  if (b_bool) {
    guard1 = true;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      b_0[b_kstr] = tmp_0[b_kstr];
    }

    b_bool = false;
    if (obj->Type->size[1] != 9) {
    } else {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          if (obj->Type->data[b_kstr - 1] != b_0[b_kstr - 1]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (b_bool) {
      guard1 = true;
    } else {
      ax[0] = (rtNaN);
      ax[1] = (rtNaN);
      ax[2] = (rtNaN);
    }
  }

  if (guard1) {
    ax[0] = obj->JointAxisInternal[0];
    ax[1] = obj->JointAxisInternal[1];
    ax[2] = obj->JointAxisInternal[2];
  }
}

void InverseKinematics::InverseKinematics_cat(real_T varargin_1, real_T
  varargin_2, real_T varargin_3, real_T varargin_4, real_T varargin_5, real_T
  varargin_6, real_T varargin_7, real_T varargin_8, real_T varargin_9, real_T y
  [9])
{
  y[0] = varargin_1;
  y[1] = varargin_2;
  y[2] = varargin_3;
  y[3] = varargin_4;
  y[4] = varargin_5;
  y[5] = varargin_6;
  y[6] = varargin_7;
  y[7] = varargin_8;
  y[8] = varargin_9;
}

void InverseKinematics::InverseKinematics_mtimes(const real_T A[36], const
  emxArray_real_T_InverseKinema_T *B, emxArray_real_T_InverseKinema_T *C)
{
  InverseKinematics_B.n = B->size[1] - 1;
  InverseKinematics_B.b_j_p = C->size[0] * C->size[1];
  C->size[0] = 6;
  C->size[1] = B->size[1];
  Invers_emxEnsureCapacity_real_T(C, InverseKinematics_B.b_j_p);
  for (InverseKinematics_B.b_j_p = 0; InverseKinematics_B.b_j_p <=
       InverseKinematics_B.n; InverseKinematics_B.b_j_p++) {
    InverseKinematics_B.coffset_tmp = InverseKinematics_B.b_j_p * 6 - 1;
    for (InverseKinematics_B.b_i_p = 0; InverseKinematics_B.b_i_p < 6;
         InverseKinematics_B.b_i_p++) {
      InverseKinematics_B.s_p = 0.0;
      for (InverseKinematics_B.b_k_f = 0; InverseKinematics_B.b_k_f < 6;
           InverseKinematics_B.b_k_f++) {
        InverseKinematics_B.s_p += A[InverseKinematics_B.b_k_f * 6 +
          InverseKinematics_B.b_i_p] * B->data[(InverseKinematics_B.coffset_tmp
          + InverseKinematics_B.b_k_f) + 1];
      }

      C->data[(InverseKinematics_B.coffset_tmp + InverseKinematics_B.b_i_p) + 1]
        = InverseKinematics_B.s_p;
    }
  }
}

void InverseKinematics::RigidBodyTree_efficientFKAndJac
  (v_robotics_manip_internal_Rig_T *obj, const real_T qv[4], const
   emxArray_char_T_InverseKinema_T *body1Name, real_T T_data[], int32_T T_size[2],
   emxArray_real_T_InverseKinema_T *Jac)
{
  c_rigidBodyJoint_InverseKinem_T *joint;
  emxArray_char_T_InverseKinema_T *body2Name;
  emxArray_real_T_InverseKinema_T *b;
  emxArray_real_T_InverseKinema_T *kinematicPathIndices;
  emxArray_real_T_InverseKinema_T *tmp;
  t_robotics_manip_internal_Rig_T *body1;
  t_robotics_manip_internal_Rig_T *body2;
  static const char_T tmp_1[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_2[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  emxArray_real_T_InverseKinema_T *Jac_0;
  InverseKinematic_emxInit_char_T(&body2Name, 2);
  InverseKinematics_B.result_data_tmp = body2Name->size[0] * body2Name->size[1];
  body2Name->size[0] = 1;
  body2Name->size[1] = obj->Base.NameInternal->size[1];
  Invers_emxEnsureCapacity_char_T(body2Name, InverseKinematics_B.result_data_tmp);
  InverseKinematics_B.loop_ub_c = obj->Base.NameInternal->size[1] - 1;
  for (InverseKinematics_B.i_g = 0; InverseKinematics_B.i_g <=
       InverseKinematics_B.loop_ub_c; InverseKinematics_B.i_g++) {
    InverseKinematics_B.result_data_tmp = InverseKinematics_B.i_g;
    body2Name->data[InverseKinematics_B.result_data_tmp] =
      obj->Base.NameInternal->data[InverseKinematics_B.result_data_tmp];
  }

  InverseKinematics_B.bid1 = RigidBodyTree_findBodyIndexByNa(obj, body1Name);
  InverseKinematics_B.bid2 = RigidBodyTree_findBodyIndexByNa(obj, body2Name);
  if (InverseKinematics_B.bid1 == 0.0) {
    body1 = &obj->Base;
  } else {
    body1 = obj->Bodies[static_cast<int32_T>(InverseKinematics_B.bid1) - 1];
  }

  if (InverseKinematics_B.bid2 == 0.0) {
    body2 = &obj->Base;
  } else {
    body2 = obj->Bodies[static_cast<int32_T>(InverseKinematics_B.bid2) - 1];
  }

  InverseKinematic_emxInit_real_T(&kinematicPathIndices, 2);
  RigidBodyTree_kinematicPathInte(obj, body1, body2, kinematicPathIndices);
  memset(&InverseKinematics_B.T1[0], 0, sizeof(real_T) << 4U);
  InverseKinematics_B.T1[0] = 1.0;
  InverseKinematics_B.T1[5] = 1.0;
  InverseKinematics_B.T1[10] = 1.0;
  InverseKinematics_B.T1[15] = 1.0;
  InverseKinematics_B.result_data_tmp = Jac->size[0] * Jac->size[1];
  Jac->size[0] = 6;
  Jac->size[1] = static_cast<int32_T>(obj->PositionNumber);
  Invers_emxEnsureCapacity_real_T(Jac, InverseKinematics_B.result_data_tmp);
  InverseKinematics_B.loop_ub_c = 6 * static_cast<int32_T>(obj->PositionNumber)
    - 1;
  if (InverseKinematics_B.loop_ub_c >= 0) {
    memset(&Jac->data[0], 0, (InverseKinematics_B.loop_ub_c + 1) * sizeof(real_T));
  }

  InverseKinematics_B.c_f = kinematicPathIndices->size[1] - 2;
  if (kinematicPathIndices->size[1] - 2 >= 0) {
    for (InverseKinematics_B.i_g = 0; InverseKinematics_B.i_g < 5;
         InverseKinematics_B.i_g++) {
      InverseKinematics_B.b_p[InverseKinematics_B.i_g] =
        tmp_1[InverseKinematics_B.i_g];
    }
  }

  InverseKinematic_emxInit_real_T(&b, 2);
  InverseKinematic_emxInit_real_T(&tmp, 2);
  for (InverseKinematics_B.Jac = 0; InverseKinematics_B.Jac <=
       InverseKinematics_B.c_f; InverseKinematics_B.Jac++) {
    __m128d tmp_0;
    int32_T exitg1;
    InverseKinematics_B.result_data_tmp = InverseKinematics_B.Jac;
    if (kinematicPathIndices->data[InverseKinematics_B.result_data_tmp] != 0.0)
    {
      body1 = obj->Bodies[static_cast<int32_T>(kinematicPathIndices->
        data[InverseKinematics_B.result_data_tmp]) - 1];
    } else {
      body1 = &obj->Base;
    }

    InverseKinematics_B.bid1 = kinematicPathIndices->
      data[InverseKinematics_B.Jac + 1];
    if (InverseKinematics_B.bid1 != 0.0) {
      body2 = obj->Bodies[static_cast<int32_T>(InverseKinematics_B.bid1) - 1];
    } else {
      body2 = &obj->Base;
    }

    InverseKinematics_B.nextBodyIsParent = (body2->Index == body1->ParentIndex);
    if (InverseKinematics_B.nextBodyIsParent) {
      body2 = body1;
      InverseKinematics_B.jointSign = 1;
    } else {
      InverseKinematics_B.jointSign = -1;
    }

    joint = body2->JointInternal;
    InverseKinematics_B.result_data_tmp = body2Name->size[0] * body2Name->size[1];
    body2Name->size[0] = 1;
    body2Name->size[1] = joint->Type->size[1];
    Invers_emxEnsureCapacity_char_T(body2Name,
      InverseKinematics_B.result_data_tmp);
    InverseKinematics_B.loop_ub_c = joint->Type->size[1] - 1;
    for (InverseKinematics_B.i_g = 0; InverseKinematics_B.i_g <=
         InverseKinematics_B.loop_ub_c; InverseKinematics_B.i_g++) {
      InverseKinematics_B.result_data_tmp = InverseKinematics_B.i_g;
      body2Name->data[InverseKinematics_B.result_data_tmp] = joint->Type->
        data[InverseKinematics_B.result_data_tmp];
    }

    InverseKinematics_B.b_bool = false;
    if (body2Name->size[1] != 5) {
    } else {
      InverseKinematics_B.i_g = 1;
      do {
        exitg1 = 0;
        if (InverseKinematics_B.i_g - 1 < 5) {
          if (body2Name->data[InverseKinematics_B.i_g - 1] !=
              InverseKinematics_B.b_p[InverseKinematics_B.i_g - 1]) {
            exitg1 = 1;
          } else {
            InverseKinematics_B.i_g++;
          }
        } else {
          InverseKinematics_B.b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (InverseKinematics_B.b_bool) {
      for (InverseKinematics_B.i_g = 0; InverseKinematics_B.i_g < 16;
           InverseKinematics_B.i_g++) {
        InverseKinematics_B.Tj_c[InverseKinematics_B.i_g] =
          joint->JointToParentTransform[InverseKinematics_B.i_g];
      }

      InverseKinematics_B.result_data_tmp = body2Name->size[0] * body2Name->
        size[1];
      body2Name->size[0] = 1;
      body2Name->size[1] = joint->Type->size[1];
      Invers_emxEnsureCapacity_char_T(body2Name,
        InverseKinematics_B.result_data_tmp);
      InverseKinematics_B.loop_ub_c = joint->Type->size[1] - 1;
      for (InverseKinematics_B.i_g = 0; InverseKinematics_B.i_g <=
           InverseKinematics_B.loop_ub_c; InverseKinematics_B.i_g++) {
        InverseKinematics_B.result_data_tmp = InverseKinematics_B.i_g;
        body2Name->data[InverseKinematics_B.result_data_tmp] = joint->Type->
          data[InverseKinematics_B.result_data_tmp];
      }

      InverseKinematics_B.b_bool = false;
      if (body2Name->size[1] != 5) {
      } else {
        InverseKinematics_B.i_g = 1;
        do {
          exitg1 = 0;
          if (InverseKinematics_B.i_g - 1 < 5) {
            if (body2Name->data[InverseKinematics_B.i_g - 1] !=
                InverseKinematics_B.b_p[InverseKinematics_B.i_g - 1]) {
              exitg1 = 1;
            } else {
              InverseKinematics_B.i_g++;
            }
          } else {
            InverseKinematics_B.b_bool = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }

      if (InverseKinematics_B.b_bool) {
        InverseKinematics_B.i_g = 0;
      } else {
        for (InverseKinematics_B.i_g = 0; InverseKinematics_B.i_g < 8;
             InverseKinematics_B.i_g++) {
          InverseKinematics_B.b_bj[InverseKinematics_B.i_g] =
            tmp_2[InverseKinematics_B.i_g];
        }

        InverseKinematics_B.b_bool = false;
        if (body2Name->size[1] != 8) {
        } else {
          InverseKinematics_B.i_g = 1;
          do {
            exitg1 = 0;
            if (InverseKinematics_B.i_g - 1 < 8) {
              if (body2Name->data[InverseKinematics_B.i_g - 1] !=
                  InverseKinematics_B.b_bj[InverseKinematics_B.i_g - 1]) {
                exitg1 = 1;
              } else {
                InverseKinematics_B.i_g++;
              }
            } else {
              InverseKinematics_B.b_bool = true;
              exitg1 = 1;
            }
          } while (exitg1 == 0);
        }

        if (InverseKinematics_B.b_bool) {
          InverseKinematics_B.i_g = 1;
        } else {
          InverseKinematics_B.i_g = -1;
        }
      }

      switch (InverseKinematics_B.i_g) {
       case 0:
        memset(&InverseKinematics_B.T1j[0], 0, sizeof(real_T) << 4U);
        InverseKinematics_B.T1j[0] = 1.0;
        InverseKinematics_B.T1j[5] = 1.0;
        InverseKinematics_B.T1j[10] = 1.0;
        InverseKinematics_B.T1j[15] = 1.0;
        break;

       case 1:
        In_rigidBodyJoint_get_JointAxis(joint, InverseKinematics_B.v);
        InverseKinematics_B.bid2 = InverseKinematics_B.v[0];
        InverseKinematics_B.bid1_tmp = InverseKinematics_B.v[1];
        InverseKinematics_B.tempR_tmp = InverseKinematics_B.v[2];
        InverseKinematics_B.bid1 = 1.0 / sqrt((InverseKinematics_B.bid2 *
          InverseKinematics_B.bid2 + InverseKinematics_B.bid1_tmp *
          InverseKinematics_B.bid1_tmp) + InverseKinematics_B.tempR_tmp *
          InverseKinematics_B.tempR_tmp);
        InverseKinematics_B.v[0] = InverseKinematics_B.bid2 *
          InverseKinematics_B.bid1;
        InverseKinematics_B.v[1] = InverseKinematics_B.bid1_tmp *
          InverseKinematics_B.bid1;
        InverseKinematics_B.v[2] = InverseKinematics_B.tempR_tmp *
          InverseKinematics_B.bid1;
        InverseKinematics_B.bid2 = InverseKinematics_B.v[0] *
          InverseKinematics_B.v[1] * 0.0;
        InverseKinematics_B.bid1_tmp = InverseKinematics_B.v[0] *
          InverseKinematics_B.v[2] * 0.0;
        InverseKinematics_B.tempR_tmp = InverseKinematics_B.v[1] *
          InverseKinematics_B.v[2] * 0.0;
        InverseKinematics_cat(InverseKinematics_B.v[0] * InverseKinematics_B.v[0]
                              * 0.0 + 1.0, InverseKinematics_B.bid2 -
                              InverseKinematics_B.v[2] * 0.0,
                              InverseKinematics_B.bid1_tmp +
                              InverseKinematics_B.v[1] * 0.0,
                              InverseKinematics_B.bid2 + InverseKinematics_B.v[2]
                              * 0.0, InverseKinematics_B.v[1] *
                              InverseKinematics_B.v[1] * 0.0 + 1.0,
                              InverseKinematics_B.tempR_tmp -
                              InverseKinematics_B.v[0] * 0.0,
                              InverseKinematics_B.bid1_tmp -
                              InverseKinematics_B.v[1] * 0.0,
                              InverseKinematics_B.tempR_tmp +
                              InverseKinematics_B.v[0] * 0.0,
                              InverseKinematics_B.v[2] * InverseKinematics_B.v[2]
                              * 0.0 + 1.0, InverseKinematics_B.tempR);
        for (InverseKinematics_B.i_g = 0; InverseKinematics_B.i_g < 3;
             InverseKinematics_B.i_g++) {
          InverseKinematics_B.R[InverseKinematics_B.i_g] =
            InverseKinematics_B.tempR[InverseKinematics_B.i_g * 3];
          InverseKinematics_B.R[InverseKinematics_B.i_g + 3] =
            InverseKinematics_B.tempR[InverseKinematics_B.i_g * 3 + 1];
          InverseKinematics_B.R[InverseKinematics_B.i_g + 6] =
            InverseKinematics_B.tempR[InverseKinematics_B.i_g * 3 + 2];
        }

        memset(&InverseKinematics_B.T1j[0], 0, sizeof(real_T) << 4U);
        for (InverseKinematics_B.i_g = 0; InverseKinematics_B.i_g < 3;
             InverseKinematics_B.i_g++) {
          InverseKinematics_B.loop_ub_c = InverseKinematics_B.i_g << 2;
          InverseKinematics_B.T1j[InverseKinematics_B.loop_ub_c] =
            InverseKinematics_B.R[3 * InverseKinematics_B.i_g];
          InverseKinematics_B.T1j[InverseKinematics_B.loop_ub_c + 1] =
            InverseKinematics_B.R[3 * InverseKinematics_B.i_g + 1];
          InverseKinematics_B.T1j[InverseKinematics_B.loop_ub_c + 2] =
            InverseKinematics_B.R[3 * InverseKinematics_B.i_g + 2];
        }

        InverseKinematics_B.T1j[15] = 1.0;
        break;

       default:
        In_rigidBodyJoint_get_JointAxis(joint, InverseKinematics_B.v);
        memset(&InverseKinematics_B.tempR[0], 0, 9U * sizeof(real_T));
        InverseKinematics_B.tempR[0] = 1.0;
        InverseKinematics_B.tempR[4] = 1.0;
        InverseKinematics_B.tempR[8] = 1.0;
        for (InverseKinematics_B.i_g = 0; InverseKinematics_B.i_g < 3;
             InverseKinematics_B.i_g++) {
          InverseKinematics_B.loop_ub_c = InverseKinematics_B.i_g << 2;
          InverseKinematics_B.T1j[InverseKinematics_B.loop_ub_c] =
            InverseKinematics_B.tempR[3 * InverseKinematics_B.i_g];
          InverseKinematics_B.T1j[InverseKinematics_B.loop_ub_c + 1] =
            InverseKinematics_B.tempR[3 * InverseKinematics_B.i_g + 1];
          InverseKinematics_B.T1j[InverseKinematics_B.loop_ub_c + 2] =
            InverseKinematics_B.tempR[3 * InverseKinematics_B.i_g + 2];
          InverseKinematics_B.T1j[InverseKinematics_B.i_g + 12] =
            InverseKinematics_B.v[InverseKinematics_B.i_g] * 0.0;
        }

        InverseKinematics_B.T1j[3] = 0.0;
        InverseKinematics_B.T1j[7] = 0.0;
        InverseKinematics_B.T1j[11] = 0.0;
        InverseKinematics_B.T1j[15] = 1.0;
        break;
      }

      for (InverseKinematics_B.i_g = 0; InverseKinematics_B.i_g < 16;
           InverseKinematics_B.i_g++) {
        InverseKinematics_B.b[InverseKinematics_B.i_g] =
          joint->ChildToJointTransform[InverseKinematics_B.i_g];
      }

      for (InverseKinematics_B.i_g = 0; InverseKinematics_B.i_g < 4;
           InverseKinematics_B.i_g++) {
        for (InverseKinematics_B.g = 0; InverseKinematics_B.g < 4;
             InverseKinematics_B.g++) {
          InverseKinematics_B.f = InverseKinematics_B.g << 2;
          InverseKinematics_B.loop_ub_c = InverseKinematics_B.i_g +
            InverseKinematics_B.f;
          InverseKinematics_B.Tj_k[InverseKinematics_B.loop_ub_c] = 0.0;
          InverseKinematics_B.Tj_k[InverseKinematics_B.loop_ub_c] +=
            InverseKinematics_B.T1j[InverseKinematics_B.f] *
            InverseKinematics_B.Tj_c[InverseKinematics_B.i_g];
          InverseKinematics_B.Tj_k[InverseKinematics_B.loop_ub_c] +=
            InverseKinematics_B.T1j[InverseKinematics_B.f + 1] *
            InverseKinematics_B.Tj_c[InverseKinematics_B.i_g + 4];
          InverseKinematics_B.Tj_k[InverseKinematics_B.loop_ub_c] +=
            InverseKinematics_B.T1j[InverseKinematics_B.f + 2] *
            InverseKinematics_B.Tj_c[InverseKinematics_B.i_g + 8];
          InverseKinematics_B.Tj_k[InverseKinematics_B.loop_ub_c] +=
            InverseKinematics_B.T1j[InverseKinematics_B.f + 3] *
            InverseKinematics_B.Tj_c[InverseKinematics_B.i_g + 12];
        }

        for (InverseKinematics_B.g = 0; InverseKinematics_B.g < 4;
             InverseKinematics_B.g++) {
          InverseKinematics_B.loop_ub_c = InverseKinematics_B.g << 2;
          InverseKinematics_B.f = InverseKinematics_B.i_g +
            InverseKinematics_B.loop_ub_c;
          InverseKinematics_B.Tc2p[InverseKinematics_B.f] = 0.0;
          InverseKinematics_B.Tc2p[InverseKinematics_B.f] +=
            InverseKinematics_B.b[InverseKinematics_B.loop_ub_c] *
            InverseKinematics_B.Tj_k[InverseKinematics_B.i_g];
          InverseKinematics_B.Tc2p[InverseKinematics_B.f] +=
            InverseKinematics_B.b[InverseKinematics_B.loop_ub_c + 1] *
            InverseKinematics_B.Tj_k[InverseKinematics_B.i_g + 4];
          InverseKinematics_B.Tc2p[InverseKinematics_B.f] +=
            InverseKinematics_B.b[InverseKinematics_B.loop_ub_c + 2] *
            InverseKinematics_B.Tj_k[InverseKinematics_B.i_g + 8];
          InverseKinematics_B.Tc2p[InverseKinematics_B.f] +=
            InverseKinematics_B.b[InverseKinematics_B.loop_ub_c + 3] *
            InverseKinematics_B.Tj_k[InverseKinematics_B.i_g + 12];
        }
      }
    } else {
      InverseKinematics_B.i_g = static_cast<int32_T>(body2->Index);
      InverseKinematics_B.bid1 = obj->PositionDoFMap[InverseKinematics_B.i_g - 1];
      InverseKinematics_B.bid2 = obj->PositionDoFMap[InverseKinematics_B.i_g + 5];
      if (InverseKinematics_B.bid1 > InverseKinematics_B.bid2) {
        InverseKinematics_B.g = 0;
        InverseKinematics_B.f = 0;
      } else {
        InverseKinematics_B.g = static_cast<int32_T>(InverseKinematics_B.bid1) -
          1;
        InverseKinematics_B.f = static_cast<int32_T>(InverseKinematics_B.bid2);
      }

      for (InverseKinematics_B.i_g = 0; InverseKinematics_B.i_g < 16;
           InverseKinematics_B.i_g++) {
        InverseKinematics_B.Tj_c[InverseKinematics_B.i_g] =
          joint->JointToParentTransform[InverseKinematics_B.i_g];
      }

      InverseKinematics_B.result_data_tmp = body2Name->size[0] * body2Name->
        size[1];
      body2Name->size[0] = 1;
      body2Name->size[1] = joint->Type->size[1];
      Invers_emxEnsureCapacity_char_T(body2Name,
        InverseKinematics_B.result_data_tmp);
      InverseKinematics_B.loop_ub_c = joint->Type->size[1] - 1;
      for (InverseKinematics_B.i_g = 0; InverseKinematics_B.i_g <=
           InverseKinematics_B.loop_ub_c; InverseKinematics_B.i_g++) {
        InverseKinematics_B.result_data_tmp = InverseKinematics_B.i_g;
        body2Name->data[InverseKinematics_B.result_data_tmp] = joint->Type->
          data[InverseKinematics_B.result_data_tmp];
      }

      InverseKinematics_B.b_bool = false;
      if (body2Name->size[1] != 5) {
      } else {
        InverseKinematics_B.i_g = 1;
        do {
          exitg1 = 0;
          if (InverseKinematics_B.i_g - 1 < 5) {
            if (body2Name->data[InverseKinematics_B.i_g - 1] !=
                InverseKinematics_B.b_p[InverseKinematics_B.i_g - 1]) {
              exitg1 = 1;
            } else {
              InverseKinematics_B.i_g++;
            }
          } else {
            InverseKinematics_B.b_bool = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }

      if (InverseKinematics_B.b_bool) {
        InverseKinematics_B.i_g = 0;
      } else {
        for (InverseKinematics_B.i_g = 0; InverseKinematics_B.i_g < 8;
             InverseKinematics_B.i_g++) {
          InverseKinematics_B.b_bj[InverseKinematics_B.i_g] =
            tmp_2[InverseKinematics_B.i_g];
        }

        InverseKinematics_B.b_bool = false;
        if (body2Name->size[1] != 8) {
        } else {
          InverseKinematics_B.i_g = 1;
          do {
            exitg1 = 0;
            if (InverseKinematics_B.i_g - 1 < 8) {
              if (body2Name->data[InverseKinematics_B.i_g - 1] !=
                  InverseKinematics_B.b_bj[InverseKinematics_B.i_g - 1]) {
                exitg1 = 1;
              } else {
                InverseKinematics_B.i_g++;
              }
            } else {
              InverseKinematics_B.b_bool = true;
              exitg1 = 1;
            }
          } while (exitg1 == 0);
        }

        if (InverseKinematics_B.b_bool) {
          InverseKinematics_B.i_g = 1;
        } else {
          InverseKinematics_B.i_g = -1;
        }
      }

      switch (InverseKinematics_B.i_g) {
       case 0:
        memset(&InverseKinematics_B.T1j[0], 0, sizeof(real_T) << 4U);
        InverseKinematics_B.T1j[0] = 1.0;
        InverseKinematics_B.T1j[5] = 1.0;
        InverseKinematics_B.T1j[10] = 1.0;
        InverseKinematics_B.T1j[15] = 1.0;
        break;

       case 1:
        In_rigidBodyJoint_get_JointAxis(joint, InverseKinematics_B.v);
        InverseKinematics_B.i_g = 0;
        InverseKinematics_B.result_data[InverseKinematics_B.i_g] =
          InverseKinematics_B.v[0];
        InverseKinematics_B.result_data_tmp = 1;
        InverseKinematics_B.result_data[InverseKinematics_B.result_data_tmp] =
          InverseKinematics_B.v[1];
        InverseKinematics_B.loop_ub_c = 2;
        InverseKinematics_B.result_data[InverseKinematics_B.loop_ub_c] =
          InverseKinematics_B.v[2];
        if ((InverseKinematics_B.f - InverseKinematics_B.g != 0) - 1 >= 0) {
          InverseKinematics_B.result_data[3] = qv[InverseKinematics_B.g];
        }

        InverseKinematics_B.bid2 =
          InverseKinematics_B.result_data[InverseKinematics_B.i_g];
        InverseKinematics_B.sth =
          InverseKinematics_B.result_data[InverseKinematics_B.result_data_tmp];
        InverseKinematics_B.bid1_tmp =
          InverseKinematics_B.result_data[InverseKinematics_B.loop_ub_c];
        InverseKinematics_B.bid1 = 1.0 / sqrt((InverseKinematics_B.bid2 *
          InverseKinematics_B.bid2 + InverseKinematics_B.sth *
          InverseKinematics_B.sth) + InverseKinematics_B.bid1_tmp *
          InverseKinematics_B.bid1_tmp);
        InverseKinematics_B.v[0] = InverseKinematics_B.bid2 *
          InverseKinematics_B.bid1;
        InverseKinematics_B.v[1] = InverseKinematics_B.sth *
          InverseKinematics_B.bid1;
        InverseKinematics_B.v[2] = InverseKinematics_B.bid1_tmp *
          InverseKinematics_B.bid1;
        InverseKinematics_B.bid2 = InverseKinematics_B.result_data[3];
        InverseKinematics_B.bid1 = cos(InverseKinematics_B.bid2);
        InverseKinematics_B.sth = sin(InverseKinematics_B.bid2);
        InverseKinematics_B.bid2 = InverseKinematics_B.v[0] *
          InverseKinematics_B.v[1] * (1.0 - InverseKinematics_B.bid1);
        InverseKinematics_B.bid1_tmp = InverseKinematics_B.v[2] *
          InverseKinematics_B.sth;
        InverseKinematics_B.tempR_tmp = InverseKinematics_B.v[0] *
          InverseKinematics_B.v[2] * (1.0 - InverseKinematics_B.bid1);
        InverseKinematics_B.tempR_tmp_n = InverseKinematics_B.v[1] *
          InverseKinematics_B.sth;
        InverseKinematics_B.tempR_tmp_m = InverseKinematics_B.v[1] *
          InverseKinematics_B.v[2] * (1.0 - InverseKinematics_B.bid1);
        InverseKinematics_B.sth *= InverseKinematics_B.v[0];
        InverseKinematics_cat(InverseKinematics_B.v[0] * InverseKinematics_B.v[0]
                              * (1.0 - InverseKinematics_B.bid1) +
                              InverseKinematics_B.bid1, InverseKinematics_B.bid2
                              - InverseKinematics_B.bid1_tmp,
                              InverseKinematics_B.tempR_tmp +
                              InverseKinematics_B.tempR_tmp_n,
                              InverseKinematics_B.bid2 +
                              InverseKinematics_B.bid1_tmp,
                              InverseKinematics_B.v[1] * InverseKinematics_B.v[1]
                              * (1.0 - InverseKinematics_B.bid1) +
                              InverseKinematics_B.bid1,
                              InverseKinematics_B.tempR_tmp_m -
                              InverseKinematics_B.sth,
                              InverseKinematics_B.tempR_tmp -
                              InverseKinematics_B.tempR_tmp_n,
                              InverseKinematics_B.tempR_tmp_m +
                              InverseKinematics_B.sth, InverseKinematics_B.v[2] *
                              InverseKinematics_B.v[2] * (1.0 -
          InverseKinematics_B.bid1) + InverseKinematics_B.bid1,
                              InverseKinematics_B.tempR);
        for (InverseKinematics_B.i_g = 0; InverseKinematics_B.i_g < 3;
             InverseKinematics_B.i_g++) {
          InverseKinematics_B.R[InverseKinematics_B.i_g] =
            InverseKinematics_B.tempR[InverseKinematics_B.i_g * 3];
          InverseKinematics_B.R[InverseKinematics_B.i_g + 3] =
            InverseKinematics_B.tempR[InverseKinematics_B.i_g * 3 + 1];
          InverseKinematics_B.R[InverseKinematics_B.i_g + 6] =
            InverseKinematics_B.tempR[InverseKinematics_B.i_g * 3 + 2];
        }

        memset(&InverseKinematics_B.T1j[0], 0, sizeof(real_T) << 4U);
        for (InverseKinematics_B.i_g = 0; InverseKinematics_B.i_g < 3;
             InverseKinematics_B.i_g++) {
          InverseKinematics_B.loop_ub_c = InverseKinematics_B.i_g << 2;
          InverseKinematics_B.T1j[InverseKinematics_B.loop_ub_c] =
            InverseKinematics_B.R[3 * InverseKinematics_B.i_g];
          InverseKinematics_B.T1j[InverseKinematics_B.loop_ub_c + 1] =
            InverseKinematics_B.R[3 * InverseKinematics_B.i_g + 1];
          InverseKinematics_B.T1j[InverseKinematics_B.loop_ub_c + 2] =
            InverseKinematics_B.R[3 * InverseKinematics_B.i_g + 2];
        }

        InverseKinematics_B.T1j[15] = 1.0;
        break;

       default:
        In_rigidBodyJoint_get_JointAxis(joint, InverseKinematics_B.v);
        memset(&InverseKinematics_B.tempR[0], 0, 9U * sizeof(real_T));
        InverseKinematics_B.tempR[0] = 1.0;
        InverseKinematics_B.tempR[4] = 1.0;
        InverseKinematics_B.tempR[8] = 1.0;
        InverseKinematics_B.bid1 = qv[InverseKinematics_B.g];
        for (InverseKinematics_B.i_g = 0; InverseKinematics_B.i_g < 3;
             InverseKinematics_B.i_g++) {
          InverseKinematics_B.loop_ub_c = InverseKinematics_B.i_g << 2;
          InverseKinematics_B.T1j[InverseKinematics_B.loop_ub_c] =
            InverseKinematics_B.tempR[3 * InverseKinematics_B.i_g];
          InverseKinematics_B.T1j[InverseKinematics_B.loop_ub_c + 1] =
            InverseKinematics_B.tempR[3 * InverseKinematics_B.i_g + 1];
          InverseKinematics_B.T1j[InverseKinematics_B.loop_ub_c + 2] =
            InverseKinematics_B.tempR[3 * InverseKinematics_B.i_g + 2];
          InverseKinematics_B.T1j[InverseKinematics_B.i_g + 12] =
            InverseKinematics_B.v[InverseKinematics_B.i_g] *
            InverseKinematics_B.bid1;
        }

        InverseKinematics_B.T1j[3] = 0.0;
        InverseKinematics_B.T1j[7] = 0.0;
        InverseKinematics_B.T1j[11] = 0.0;
        InverseKinematics_B.T1j[15] = 1.0;
        break;
      }

      for (InverseKinematics_B.i_g = 0; InverseKinematics_B.i_g < 16;
           InverseKinematics_B.i_g++) {
        InverseKinematics_B.b[InverseKinematics_B.i_g] =
          joint->ChildToJointTransform[InverseKinematics_B.i_g];
      }

      for (InverseKinematics_B.i_g = 0; InverseKinematics_B.i_g < 4;
           InverseKinematics_B.i_g++) {
        for (InverseKinematics_B.g = 0; InverseKinematics_B.g < 4;
             InverseKinematics_B.g++) {
          InverseKinematics_B.f = InverseKinematics_B.g << 2;
          InverseKinematics_B.loop_ub_c = InverseKinematics_B.i_g +
            InverseKinematics_B.f;
          InverseKinematics_B.Tj_k[InverseKinematics_B.loop_ub_c] = 0.0;
          InverseKinematics_B.Tj_k[InverseKinematics_B.loop_ub_c] +=
            InverseKinematics_B.T1j[InverseKinematics_B.f] *
            InverseKinematics_B.Tj_c[InverseKinematics_B.i_g];
          InverseKinematics_B.Tj_k[InverseKinematics_B.loop_ub_c] +=
            InverseKinematics_B.T1j[InverseKinematics_B.f + 1] *
            InverseKinematics_B.Tj_c[InverseKinematics_B.i_g + 4];
          InverseKinematics_B.Tj_k[InverseKinematics_B.loop_ub_c] +=
            InverseKinematics_B.T1j[InverseKinematics_B.f + 2] *
            InverseKinematics_B.Tj_c[InverseKinematics_B.i_g + 8];
          InverseKinematics_B.Tj_k[InverseKinematics_B.loop_ub_c] +=
            InverseKinematics_B.T1j[InverseKinematics_B.f + 3] *
            InverseKinematics_B.Tj_c[InverseKinematics_B.i_g + 12];
        }

        for (InverseKinematics_B.g = 0; InverseKinematics_B.g < 4;
             InverseKinematics_B.g++) {
          InverseKinematics_B.loop_ub_c = InverseKinematics_B.g << 2;
          InverseKinematics_B.f = InverseKinematics_B.i_g +
            InverseKinematics_B.loop_ub_c;
          InverseKinematics_B.Tc2p[InverseKinematics_B.f] = 0.0;
          InverseKinematics_B.Tc2p[InverseKinematics_B.f] +=
            InverseKinematics_B.b[InverseKinematics_B.loop_ub_c] *
            InverseKinematics_B.Tj_k[InverseKinematics_B.i_g];
          InverseKinematics_B.Tc2p[InverseKinematics_B.f] +=
            InverseKinematics_B.b[InverseKinematics_B.loop_ub_c + 1] *
            InverseKinematics_B.Tj_k[InverseKinematics_B.i_g + 4];
          InverseKinematics_B.Tc2p[InverseKinematics_B.f] +=
            InverseKinematics_B.b[InverseKinematics_B.loop_ub_c + 2] *
            InverseKinematics_B.Tj_k[InverseKinematics_B.i_g + 8];
          InverseKinematics_B.Tc2p[InverseKinematics_B.f] +=
            InverseKinematics_B.b[InverseKinematics_B.loop_ub_c + 3] *
            InverseKinematics_B.Tj_k[InverseKinematics_B.i_g + 12];
        }
      }

      InverseKinematics_B.i_g = static_cast<int32_T>(body2->Index);
      InverseKinematics_B.bid1 = obj->VelocityDoFMap[InverseKinematics_B.i_g - 1];
      InverseKinematics_B.bid2 = obj->VelocityDoFMap[InverseKinematics_B.i_g + 5];
      if (InverseKinematics_B.nextBodyIsParent) {
        for (InverseKinematics_B.i_g = 0; InverseKinematics_B.i_g < 16;
             InverseKinematics_B.i_g++) {
          InverseKinematics_B.Tj_c[InverseKinematics_B.i_g] =
            joint->ChildToJointTransform[InverseKinematics_B.i_g];
        }
      } else {
        for (InverseKinematics_B.i_g = 0; InverseKinematics_B.i_g < 16;
             InverseKinematics_B.i_g++) {
          InverseKinematics_B.T1j[InverseKinematics_B.i_g] =
            joint->JointToParentTransform[InverseKinematics_B.i_g];
        }

        for (InverseKinematics_B.i_g = 0; InverseKinematics_B.i_g < 3;
             InverseKinematics_B.i_g++) {
          InverseKinematics_B.R[3 * InverseKinematics_B.i_g] =
            InverseKinematics_B.T1j[InverseKinematics_B.i_g];
          InverseKinematics_B.R[3 * InverseKinematics_B.i_g + 1] =
            InverseKinematics_B.T1j[InverseKinematics_B.i_g + 4];
          InverseKinematics_B.R[3 * InverseKinematics_B.i_g + 2] =
            InverseKinematics_B.T1j[InverseKinematics_B.i_g + 8];
        }

        for (InverseKinematics_B.i_g = 0; InverseKinematics_B.i_g <= 6;
             InverseKinematics_B.i_g += 2) {
          tmp_0 = _mm_loadu_pd(&InverseKinematics_B.R[InverseKinematics_B.i_g]);
          _mm_storeu_pd(&InverseKinematics_B.tempR[InverseKinematics_B.i_g],
                        _mm_mul_pd(tmp_0, _mm_set1_pd(-1.0)));
        }

        for (InverseKinematics_B.i_g = 8; InverseKinematics_B.i_g < 9;
             InverseKinematics_B.i_g++) {
          InverseKinematics_B.tempR[InverseKinematics_B.i_g] =
            -InverseKinematics_B.R[InverseKinematics_B.i_g];
        }

        for (InverseKinematics_B.i_g = 0; InverseKinematics_B.i_g < 3;
             InverseKinematics_B.i_g++) {
          InverseKinematics_B.loop_ub_c = InverseKinematics_B.i_g << 2;
          InverseKinematics_B.Tj_c[InverseKinematics_B.loop_ub_c] =
            InverseKinematics_B.R[3 * InverseKinematics_B.i_g];
          InverseKinematics_B.Tj_c[InverseKinematics_B.loop_ub_c + 1] =
            InverseKinematics_B.R[3 * InverseKinematics_B.i_g + 1];
          InverseKinematics_B.Tj_c[InverseKinematics_B.loop_ub_c + 2] =
            InverseKinematics_B.R[3 * InverseKinematics_B.i_g + 2];
          InverseKinematics_B.Tj_c[InverseKinematics_B.i_g + 12] =
            (InverseKinematics_B.tempR[InverseKinematics_B.i_g + 3] *
             InverseKinematics_B.T1j[13] +
             InverseKinematics_B.tempR[InverseKinematics_B.i_g] *
             InverseKinematics_B.T1j[12]) +
            InverseKinematics_B.tempR[InverseKinematics_B.i_g + 6] *
            InverseKinematics_B.T1j[14];
        }

        InverseKinematics_B.Tj_c[3] = 0.0;
        InverseKinematics_B.Tj_c[7] = 0.0;
        InverseKinematics_B.Tj_c[11] = 0.0;
        InverseKinematics_B.Tj_c[15] = 1.0;
      }

      for (InverseKinematics_B.i_g = 0; InverseKinematics_B.i_g < 4;
           InverseKinematics_B.i_g++) {
        for (InverseKinematics_B.g = 0; InverseKinematics_B.g < 4;
             InverseKinematics_B.g++) {
          InverseKinematics_B.f = InverseKinematics_B.g << 2;
          InverseKinematics_B.loop_ub_c = InverseKinematics_B.i_g +
            InverseKinematics_B.f;
          InverseKinematics_B.T1j[InverseKinematics_B.loop_ub_c] = 0.0;
          InverseKinematics_B.T1j[InverseKinematics_B.loop_ub_c] +=
            InverseKinematics_B.T1[InverseKinematics_B.f] *
            InverseKinematics_B.Tj_c[InverseKinematics_B.i_g];
          InverseKinematics_B.T1j[InverseKinematics_B.loop_ub_c] +=
            InverseKinematics_B.T1[InverseKinematics_B.f + 1] *
            InverseKinematics_B.Tj_c[InverseKinematics_B.i_g + 4];
          InverseKinematics_B.T1j[InverseKinematics_B.loop_ub_c] +=
            InverseKinematics_B.T1[InverseKinematics_B.f + 2] *
            InverseKinematics_B.Tj_c[InverseKinematics_B.i_g + 8];
          InverseKinematics_B.T1j[InverseKinematics_B.loop_ub_c] +=
            InverseKinematics_B.T1[InverseKinematics_B.f + 3] *
            InverseKinematics_B.Tj_c[InverseKinematics_B.i_g + 12];
        }
      }

      for (InverseKinematics_B.i_g = 0; InverseKinematics_B.i_g < 3;
           InverseKinematics_B.i_g++) {
        InverseKinematics_B.R[3 * InverseKinematics_B.i_g] =
          InverseKinematics_B.T1j[InverseKinematics_B.i_g];
        InverseKinematics_B.R[3 * InverseKinematics_B.i_g + 1] =
          InverseKinematics_B.T1j[InverseKinematics_B.i_g + 4];
        InverseKinematics_B.R[3 * InverseKinematics_B.i_g + 2] =
          InverseKinematics_B.T1j[InverseKinematics_B.i_g + 8];
      }

      for (InverseKinematics_B.i_g = 0; InverseKinematics_B.i_g <= 6;
           InverseKinematics_B.i_g += 2) {
        tmp_0 = _mm_loadu_pd(&InverseKinematics_B.R[InverseKinematics_B.i_g]);
        _mm_storeu_pd(&InverseKinematics_B.tempR[InverseKinematics_B.i_g],
                      _mm_mul_pd(tmp_0, _mm_set1_pd(-1.0)));
      }

      for (InverseKinematics_B.i_g = 8; InverseKinematics_B.i_g < 9;
           InverseKinematics_B.i_g++) {
        InverseKinematics_B.tempR[InverseKinematics_B.i_g] =
          -InverseKinematics_B.R[InverseKinematics_B.i_g];
      }

      for (InverseKinematics_B.i_g = 0; InverseKinematics_B.i_g < 3;
           InverseKinematics_B.i_g++) {
        InverseKinematics_B.loop_ub_c = InverseKinematics_B.i_g << 2;
        InverseKinematics_B.Tj_c[InverseKinematics_B.loop_ub_c] =
          InverseKinematics_B.R[3 * InverseKinematics_B.i_g];
        InverseKinematics_B.Tj_c[InverseKinematics_B.loop_ub_c + 1] =
          InverseKinematics_B.R[3 * InverseKinematics_B.i_g + 1];
        InverseKinematics_B.Tj_c[InverseKinematics_B.loop_ub_c + 2] =
          InverseKinematics_B.R[3 * InverseKinematics_B.i_g + 2];
        InverseKinematics_B.Tj_c[InverseKinematics_B.i_g + 12] =
          (InverseKinematics_B.tempR[InverseKinematics_B.i_g + 3] *
           InverseKinematics_B.T1j[13] +
           InverseKinematics_B.tempR[InverseKinematics_B.i_g] *
           InverseKinematics_B.T1j[12]) +
          InverseKinematics_B.tempR[InverseKinematics_B.i_g + 6] *
          InverseKinematics_B.T1j[14];
      }

      InverseKinematics_B.Tj_c[3] = 0.0;
      InverseKinematics_B.Tj_c[7] = 0.0;
      InverseKinematics_B.Tj_c[11] = 0.0;
      InverseKinematics_B.Tj_c[15] = 1.0;
      InverseKinematics_B.result_data_tmp = b->size[0] * b->size[1];
      b->size[0] = 6;
      b->size[1] = joint->MotionSubspace->size[1];
      Invers_emxEnsureCapacity_real_T(b, InverseKinematics_B.result_data_tmp);
      InverseKinematics_B.loop_ub_c = 6 * joint->MotionSubspace->size[1];
      for (InverseKinematics_B.i_g = 0; InverseKinematics_B.i_g <
           InverseKinematics_B.loop_ub_c; InverseKinematics_B.i_g++) {
        b->data[InverseKinematics_B.i_g] = joint->MotionSubspace->
          data[InverseKinematics_B.i_g];
      }

      if (InverseKinematics_B.bid1 > InverseKinematics_B.bid2) {
        InverseKinematics_B.f = 0;
      } else {
        InverseKinematics_B.f = static_cast<int32_T>(InverseKinematics_B.bid1) -
          1;
      }

      InverseKinematics_B.R[0] = 0.0;
      InverseKinematics_B.R[3] = -InverseKinematics_B.Tj_c[14];
      InverseKinematics_B.R[6] = InverseKinematics_B.Tj_c[13];
      InverseKinematics_B.R[1] = InverseKinematics_B.Tj_c[14];
      InverseKinematics_B.R[4] = 0.0;
      InverseKinematics_B.R[7] = -InverseKinematics_B.Tj_c[12];
      InverseKinematics_B.R[2] = -InverseKinematics_B.Tj_c[13];
      InverseKinematics_B.R[5] = InverseKinematics_B.Tj_c[12];
      InverseKinematics_B.R[8] = 0.0;
      for (InverseKinematics_B.i_g = 0; InverseKinematics_B.i_g < 3;
           InverseKinematics_B.i_g++) {
        for (InverseKinematics_B.g = 0; InverseKinematics_B.g < 3;
             InverseKinematics_B.g++) {
          InverseKinematics_B.loop_ub_c = 3 * InverseKinematics_B.g +
            InverseKinematics_B.i_g;
          InverseKinematics_B.tempR[InverseKinematics_B.loop_ub_c] = 0.0;
          InverseKinematics_B.result_data_tmp = InverseKinematics_B.g << 2;
          InverseKinematics_B.tempR[InverseKinematics_B.loop_ub_c] +=
            InverseKinematics_B.Tj_c[InverseKinematics_B.result_data_tmp] *
            InverseKinematics_B.R[InverseKinematics_B.i_g];
          InverseKinematics_B.tempR[InverseKinematics_B.loop_ub_c] +=
            InverseKinematics_B.Tj_c[InverseKinematics_B.result_data_tmp + 1] *
            InverseKinematics_B.R[InverseKinematics_B.i_g + 3];
          InverseKinematics_B.tempR[InverseKinematics_B.loop_ub_c] +=
            InverseKinematics_B.Tj_c[InverseKinematics_B.result_data_tmp + 2] *
            InverseKinematics_B.R[InverseKinematics_B.i_g + 6];
          InverseKinematics_B.Tj[InverseKinematics_B.g + 6 *
            InverseKinematics_B.i_g] = InverseKinematics_B.Tj_c
            [(InverseKinematics_B.i_g << 2) + InverseKinematics_B.g];
          InverseKinematics_B.Tj[InverseKinematics_B.g + 6 *
            (InverseKinematics_B.i_g + 3)] = 0.0;
        }
      }

      for (InverseKinematics_B.i_g = 0; InverseKinematics_B.i_g < 3;
           InverseKinematics_B.i_g++) {
        InverseKinematics_B.Tj[6 * InverseKinematics_B.i_g + 3] =
          InverseKinematics_B.tempR[3 * InverseKinematics_B.i_g];
        InverseKinematics_B.loop_ub_c = InverseKinematics_B.i_g << 2;
        InverseKinematics_B.g = (InverseKinematics_B.i_g + 3) * 6;
        InverseKinematics_B.Tj[InverseKinematics_B.g + 3] =
          InverseKinematics_B.Tj_c[InverseKinematics_B.loop_ub_c];
        InverseKinematics_B.Tj[6 * InverseKinematics_B.i_g + 4] =
          InverseKinematics_B.tempR[3 * InverseKinematics_B.i_g + 1];
        InverseKinematics_B.Tj[InverseKinematics_B.g + 4] =
          InverseKinematics_B.Tj_c[InverseKinematics_B.loop_ub_c + 1];
        InverseKinematics_B.Tj[6 * InverseKinematics_B.i_g + 5] =
          InverseKinematics_B.tempR[3 * InverseKinematics_B.i_g + 2];
        InverseKinematics_B.Tj[InverseKinematics_B.g + 5] =
          InverseKinematics_B.Tj_c[InverseKinematics_B.loop_ub_c + 2];
      }

      InverseKinematics_mtimes(InverseKinematics_B.Tj, b, tmp);
      InverseKinematics_B.loop_ub_c = tmp->size[1];
      for (InverseKinematics_B.i_g = 0; InverseKinematics_B.i_g <
           InverseKinematics_B.loop_ub_c; InverseKinematics_B.i_g++) {
        for (InverseKinematics_B.g = 0; InverseKinematics_B.g <= 4;
             InverseKinematics_B.g += 2) {
          tmp_0 = _mm_loadu_pd(&tmp->data[6 * InverseKinematics_B.i_g +
                               InverseKinematics_B.g]);
          _mm_storeu_pd(&Jac->data[InverseKinematics_B.g + 6 *
                        (InverseKinematics_B.f + InverseKinematics_B.i_g)],
                        _mm_mul_pd(tmp_0, _mm_set1_pd(static_cast<real_T>
            (InverseKinematics_B.jointSign))));
        }
      }
    }

    if (InverseKinematics_B.nextBodyIsParent) {
      for (InverseKinematics_B.i_g = 0; InverseKinematics_B.i_g < 4;
           InverseKinematics_B.i_g++) {
        for (InverseKinematics_B.g = 0; InverseKinematics_B.g < 4;
             InverseKinematics_B.g++) {
          InverseKinematics_B.loop_ub_c = InverseKinematics_B.g << 2;
          InverseKinematics_B.f = InverseKinematics_B.i_g +
            InverseKinematics_B.loop_ub_c;
          InverseKinematics_B.Tj_c[InverseKinematics_B.f] = 0.0;
          InverseKinematics_B.Tj_c[InverseKinematics_B.f] +=
            InverseKinematics_B.T1[InverseKinematics_B.loop_ub_c] *
            InverseKinematics_B.Tc2p[InverseKinematics_B.i_g];
          InverseKinematics_B.Tj_c[InverseKinematics_B.f] +=
            InverseKinematics_B.T1[InverseKinematics_B.loop_ub_c + 1] *
            InverseKinematics_B.Tc2p[InverseKinematics_B.i_g + 4];
          InverseKinematics_B.Tj_c[InverseKinematics_B.f] +=
            InverseKinematics_B.T1[InverseKinematics_B.loop_ub_c + 2] *
            InverseKinematics_B.Tc2p[InverseKinematics_B.i_g + 8];
          InverseKinematics_B.Tj_c[InverseKinematics_B.f] +=
            InverseKinematics_B.T1[InverseKinematics_B.loop_ub_c + 3] *
            InverseKinematics_B.Tc2p[InverseKinematics_B.i_g + 12];
        }
      }

      memcpy(&InverseKinematics_B.T1[0], &InverseKinematics_B.Tj_c[0], sizeof
             (real_T) << 4U);
    } else {
      for (InverseKinematics_B.i_g = 0; InverseKinematics_B.i_g < 3;
           InverseKinematics_B.i_g++) {
        InverseKinematics_B.R[3 * InverseKinematics_B.i_g] =
          InverseKinematics_B.Tc2p[InverseKinematics_B.i_g];
        InverseKinematics_B.R[3 * InverseKinematics_B.i_g + 1] =
          InverseKinematics_B.Tc2p[InverseKinematics_B.i_g + 4];
        InverseKinematics_B.R[3 * InverseKinematics_B.i_g + 2] =
          InverseKinematics_B.Tc2p[InverseKinematics_B.i_g + 8];
      }

      for (InverseKinematics_B.i_g = 0; InverseKinematics_B.i_g <= 6;
           InverseKinematics_B.i_g += 2) {
        tmp_0 = _mm_loadu_pd(&InverseKinematics_B.R[InverseKinematics_B.i_g]);
        _mm_storeu_pd(&InverseKinematics_B.tempR[InverseKinematics_B.i_g],
                      _mm_mul_pd(tmp_0, _mm_set1_pd(-1.0)));
      }

      for (InverseKinematics_B.i_g = 8; InverseKinematics_B.i_g < 9;
           InverseKinematics_B.i_g++) {
        InverseKinematics_B.tempR[InverseKinematics_B.i_g] =
          -InverseKinematics_B.R[InverseKinematics_B.i_g];
      }

      for (InverseKinematics_B.i_g = 0; InverseKinematics_B.i_g < 3;
           InverseKinematics_B.i_g++) {
        InverseKinematics_B.jointSign = InverseKinematics_B.i_g << 2;
        InverseKinematics_B.Tj_c[InverseKinematics_B.jointSign] =
          InverseKinematics_B.R[3 * InverseKinematics_B.i_g];
        InverseKinematics_B.Tj_c[InverseKinematics_B.jointSign + 1] =
          InverseKinematics_B.R[3 * InverseKinematics_B.i_g + 1];
        InverseKinematics_B.Tj_c[InverseKinematics_B.jointSign + 2] =
          InverseKinematics_B.R[3 * InverseKinematics_B.i_g + 2];
        InverseKinematics_B.Tj_c[InverseKinematics_B.i_g + 12] =
          (InverseKinematics_B.tempR[InverseKinematics_B.i_g + 3] *
           InverseKinematics_B.Tc2p[13] +
           InverseKinematics_B.tempR[InverseKinematics_B.i_g] *
           InverseKinematics_B.Tc2p[12]) +
          InverseKinematics_B.tempR[InverseKinematics_B.i_g + 6] *
          InverseKinematics_B.Tc2p[14];
      }

      InverseKinematics_B.Tj_c[3] = 0.0;
      InverseKinematics_B.Tj_c[7] = 0.0;
      InverseKinematics_B.Tj_c[11] = 0.0;
      InverseKinematics_B.Tj_c[15] = 1.0;
      for (InverseKinematics_B.i_g = 0; InverseKinematics_B.i_g < 4;
           InverseKinematics_B.i_g++) {
        for (InverseKinematics_B.g = 0; InverseKinematics_B.g < 4;
             InverseKinematics_B.g++) {
          InverseKinematics_B.loop_ub_c = InverseKinematics_B.g << 2;
          InverseKinematics_B.jointSign = InverseKinematics_B.i_g +
            InverseKinematics_B.loop_ub_c;
          InverseKinematics_B.Tc2p[InverseKinematics_B.jointSign] = 0.0;
          InverseKinematics_B.Tc2p[InverseKinematics_B.jointSign] +=
            InverseKinematics_B.T1[InverseKinematics_B.loop_ub_c] *
            InverseKinematics_B.Tj_c[InverseKinematics_B.i_g];
          InverseKinematics_B.Tc2p[InverseKinematics_B.jointSign] +=
            InverseKinematics_B.T1[InverseKinematics_B.loop_ub_c + 1] *
            InverseKinematics_B.Tj_c[InverseKinematics_B.i_g + 4];
          InverseKinematics_B.Tc2p[InverseKinematics_B.jointSign] +=
            InverseKinematics_B.T1[InverseKinematics_B.loop_ub_c + 2] *
            InverseKinematics_B.Tj_c[InverseKinematics_B.i_g + 8];
          InverseKinematics_B.Tc2p[InverseKinematics_B.jointSign] +=
            InverseKinematics_B.T1[InverseKinematics_B.loop_ub_c + 3] *
            InverseKinematics_B.Tj_c[InverseKinematics_B.i_g + 12];
        }
      }

      memcpy(&InverseKinematics_B.T1[0], &InverseKinematics_B.Tc2p[0], sizeof
             (real_T) << 4U);
    }
  }

  InverseKinematic_emxFree_real_T(&tmp);
  InverseKinematic_emxFree_real_T(&b);
  InverseKinematic_emxFree_char_T(&body2Name);
  InverseKinematic_emxFree_real_T(&kinematicPathIndices);
  for (InverseKinematics_B.i_g = 0; InverseKinematics_B.i_g < 3;
       InverseKinematics_B.i_g++) {
    InverseKinematics_B.Jac = InverseKinematics_B.i_g << 2;
    InverseKinematics_B.bid1 = InverseKinematics_B.T1[InverseKinematics_B.Jac];
    InverseKinematics_B.Tj[6 * InverseKinematics_B.i_g] =
      InverseKinematics_B.bid1;
    InverseKinematics_B.c_f = (InverseKinematics_B.i_g + 3) * 6;
    InverseKinematics_B.Tj[InverseKinematics_B.c_f] = 0.0;
    InverseKinematics_B.Tj[6 * InverseKinematics_B.i_g + 3] = 0.0;
    InverseKinematics_B.Tj[InverseKinematics_B.c_f + 3] =
      InverseKinematics_B.bid1;
    InverseKinematics_B.bid1 = InverseKinematics_B.T1[InverseKinematics_B.Jac +
      1];
    InverseKinematics_B.Tj[6 * InverseKinematics_B.i_g + 1] =
      InverseKinematics_B.bid1;
    InverseKinematics_B.Tj[InverseKinematics_B.c_f + 1] = 0.0;
    InverseKinematics_B.Tj[6 * InverseKinematics_B.i_g + 4] = 0.0;
    InverseKinematics_B.Tj[InverseKinematics_B.c_f + 4] =
      InverseKinematics_B.bid1;
    InverseKinematics_B.bid1 = InverseKinematics_B.T1[InverseKinematics_B.Jac +
      2];
    InverseKinematics_B.Tj[6 * InverseKinematics_B.i_g + 2] =
      InverseKinematics_B.bid1;
    InverseKinematics_B.Tj[InverseKinematics_B.c_f + 2] = 0.0;
    InverseKinematics_B.Tj[6 * InverseKinematics_B.i_g + 5] = 0.0;
    InverseKinematics_B.Tj[InverseKinematics_B.c_f + 5] =
      InverseKinematics_B.bid1;
  }

  InverseKinematic_emxInit_real_T(&Jac_0, 2);
  InverseKinematics_B.result_data_tmp = Jac_0->size[0] * Jac_0->size[1];
  Jac_0->size[0] = 6;
  Jac_0->size[1] = Jac->size[1];
  Invers_emxEnsureCapacity_real_T(Jac_0, InverseKinematics_B.result_data_tmp);
  InverseKinematics_B.loop_ub_c = Jac->size[0] * Jac->size[1] - 1;
  if (InverseKinematics_B.loop_ub_c >= 0) {
    memcpy(&Jac_0->data[0], &Jac->data[0], (InverseKinematics_B.loop_ub_c + 1) *
           sizeof(real_T));
  }

  InverseKinematics_mtimes(InverseKinematics_B.Tj, Jac_0, Jac);
  T_size[0] = 4;
  T_size[1] = 4;
  InverseKinematic_emxFree_real_T(&Jac_0);
  memcpy(&T_data[0], &InverseKinematics_B.T1[0], sizeof(real_T) << 4U);
}

real_T rt_hypotd_snf(real_T u0, real_T u1)
{
  real_T a;
  real_T y;
  a = fabs(u0);
  y = fabs(u1);
  if (a < y) {
    a /= y;
    y *= sqrt(a * a + 1.0);
  } else if (a > y) {
    y /= a;
    y = sqrt(y * y + 1.0) * a;
  } else if (!rtIsNaN(y)) {
    y = a * 1.4142135623730951;
  }

  return y;
}

creal_T InverseKinematics::InverseKinematics_sqrt(const creal_T x)
{
  creal_T b_x;
  real_T absxi;
  real_T absxr;
  if (x.im == 0.0) {
    if (x.re < 0.0) {
      absxr = 0.0;
      absxi = sqrt(-x.re);
    } else {
      absxr = sqrt(x.re);
      absxi = 0.0;
    }
  } else if (x.re == 0.0) {
    if (x.im < 0.0) {
      absxr = sqrt(-x.im / 2.0);
      absxi = -absxr;
    } else {
      absxr = sqrt(x.im / 2.0);
      absxi = absxr;
    }
  } else if (rtIsNaN(x.re)) {
    absxr = x.re;
    absxi = x.re;
  } else if (rtIsNaN(x.im)) {
    absxr = x.im;
    absxi = x.im;
  } else if (rtIsInf(x.im)) {
    absxr = fabs(x.im);
    absxi = x.im;
  } else if (rtIsInf(x.re)) {
    if (x.re < 0.0) {
      absxr = 0.0;
      absxi = x.im * -x.re;
    } else {
      absxr = x.re;
      absxi = 0.0;
    }
  } else {
    absxr = fabs(x.re);
    absxi = fabs(x.im);
    if ((absxr > 4.4942328371557893E+307) || (absxi > 4.4942328371557893E+307))
    {
      absxr *= 0.5;
      absxi = rt_hypotd_snf(absxr, absxi * 0.5);
      if (absxi > absxr) {
        absxr = sqrt(absxr / absxi + 1.0) * sqrt(absxi);
      } else {
        absxr = sqrt(absxi) * 1.4142135623730951;
      }
    } else {
      absxr = sqrt((rt_hypotd_snf(absxr, absxi) + absxr) * 0.5);
    }

    if (x.re > 0.0) {
      absxi = x.im / absxr * 0.5;
    } else {
      if (x.im < 0.0) {
        absxi = -absxr;
      } else {
        absxi = absxr;
      }

      absxr = x.im / absxi * 0.5;
    }
  }

  b_x.re = absxr;
  b_x.im = absxi;
  return b_x;
}

real_T rt_atan2d_snf(real_T u0, real_T u1)
{
  real_T y;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = (rtNaN);
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    int32_T u0_0;
    int32_T u1_0;
    if (u0 > 0.0) {
      u0_0 = 1;
    } else {
      u0_0 = -1;
    }

    if (u1 > 0.0) {
      u1_0 = 1;
    } else {
      u1_0 = -1;
    }

    y = atan2(static_cast<real_T>(u0_0), static_cast<real_T>(u1_0));
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(RT_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = atan2(u0, u1);
  }

  return y;
}

real_T InverseKinematics::InverseKinematics_xnrm2(int32_T n, const real_T x[9],
  int32_T ix0)
{
  real_T scale;
  real_T y;
  int32_T kend;
  y = 0.0;
  scale = 3.3121686421112381E-170;
  kend = ix0 + n;
  for (int32_T k = ix0; k < kend; k++) {
    real_T absxk;
    absxk = fabs(x[k - 1]);
    if (absxk > scale) {
      real_T t;
      t = scale / absxk;
      y = y * t * t + 1.0;
      scale = absxk;
    } else {
      real_T t;
      t = absxk / scale;
      y += t * t;
    }
  }

  return scale * sqrt(y);
}

real_T InverseKinematics::InverseKinematics_xdotc(int32_T n, const real_T x[9],
  int32_T ix0, const real_T y[9], int32_T iy0)
{
  real_T d;
  d = 0.0;
  for (int32_T k = 0; k < n; k++) {
    d += x[(ix0 + k) - 1] * y[(iy0 + k) - 1];
  }

  return d;
}

void InverseKinematics::InverseKinematics_xaxpy(int32_T n, real_T a, int32_T ix0,
  const real_T y[9], int32_T iy0, real_T b_y[9])
{
  memcpy(&b_y[0], &y[0], 9U * sizeof(real_T));
  if (!(a == 0.0)) {
    for (int32_T k = 0; k < n; k++) {
      int32_T b_y_tmp;
      b_y_tmp = (iy0 + k) - 1;
      b_y[b_y_tmp] += b_y[(ix0 + k) - 1] * a;
    }
  }
}

real_T InverseKinematics::InverseKinematics_xnrm2_n(const real_T x[3], int32_T
  ix0)
{
  real_T scale;
  real_T y;
  y = 0.0;
  scale = 3.3121686421112381E-170;
  for (int32_T k = ix0; k <= ix0 + 1; k++) {
    real_T absxk;
    absxk = fabs(x[k - 1]);
    if (absxk > scale) {
      real_T t;
      t = scale / absxk;
      y = y * t * t + 1.0;
      scale = absxk;
    } else {
      real_T t;
      t = absxk / scale;
      y += t * t;
    }
  }

  return scale * sqrt(y);
}

void InverseKinematics::InverseKinematics_xaxpy_nly(int32_T n, real_T a, const
  real_T x[9], int32_T ix0, real_T y[3], int32_T iy0)
{
  if (!(a == 0.0)) {
    int32_T scalarLB;
    int32_T tmp_0;
    int32_T vectorUB;
    scalarLB = (n / 2) << 1;
    vectorUB = scalarLB - 2;
    for (int32_T k = 0; k <= vectorUB; k += 2) {
      __m128d tmp;
      tmp_0 = (iy0 + k) - 1;
      tmp = _mm_loadu_pd(&y[tmp_0]);
      _mm_storeu_pd(&y[tmp_0], _mm_add_pd(_mm_mul_pd(_mm_loadu_pd(&x[(ix0 + k) -
        1]), _mm_set1_pd(a)), tmp));
    }

    for (int32_T k = scalarLB; k < n; k++) {
      tmp_0 = (iy0 + k) - 1;
      y[tmp_0] += x[(ix0 + k) - 1] * a;
    }
  }
}

void InverseKinematics::InverseKinematics_xaxpy_nl(int32_T n, real_T a, const
  real_T x[3], int32_T ix0, const real_T y[9], int32_T iy0, real_T b_y[9])
{
  memcpy(&b_y[0], &y[0], 9U * sizeof(real_T));
  if (!(a == 0.0)) {
    int32_T b_y_tmp;
    int32_T scalarLB;
    int32_T vectorUB;
    scalarLB = (n / 2) << 1;
    vectorUB = scalarLB - 2;
    for (int32_T k = 0; k <= vectorUB; k += 2) {
      __m128d tmp;
      b_y_tmp = (iy0 + k) - 1;
      tmp = _mm_loadu_pd(&b_y[b_y_tmp]);
      _mm_storeu_pd(&b_y[b_y_tmp], _mm_add_pd(_mm_mul_pd(_mm_loadu_pd(&x[(ix0 +
        k) - 1]), _mm_set1_pd(a)), tmp));
    }

    for (int32_T k = scalarLB; k < n; k++) {
      b_y_tmp = (iy0 + k) - 1;
      b_y[b_y_tmp] += x[(ix0 + k) - 1] * a;
    }
  }
}

void InverseKinematics::InverseKinematics_xswap(const real_T x[9], int32_T ix0,
  int32_T iy0, real_T b_x[9])
{
  real_T temp;
  memcpy(&b_x[0], &x[0], 9U * sizeof(real_T));
  temp = b_x[ix0 - 1];
  b_x[ix0 - 1] = b_x[iy0 - 1];
  b_x[iy0 - 1] = temp;
  temp = b_x[ix0];
  b_x[ix0] = b_x[iy0];
  b_x[iy0] = temp;
  temp = b_x[ix0 + 1];
  b_x[ix0 + 1] = b_x[iy0 + 1];
  b_x[iy0 + 1] = temp;
}

void InverseKinematics::InverseKinematics_xrotg(real_T a, real_T b, real_T *b_a,
  real_T *b_b, real_T *c, real_T *s)
{
  InverseKinematics_B.roe = b;
  InverseKinematics_B.absa = fabs(a);
  InverseKinematics_B.absb = fabs(b);
  if (InverseKinematics_B.absa > InverseKinematics_B.absb) {
    InverseKinematics_B.roe = a;
  }

  InverseKinematics_B.scale_h = InverseKinematics_B.absa +
    InverseKinematics_B.absb;
  if (InverseKinematics_B.scale_h == 0.0) {
    *s = 0.0;
    *c = 1.0;
    *b_a = 0.0;
    *b_b = 0.0;
  } else {
    InverseKinematics_B.ads = InverseKinematics_B.absa /
      InverseKinematics_B.scale_h;
    InverseKinematics_B.bds = InverseKinematics_B.absb /
      InverseKinematics_B.scale_h;
    *b_a = sqrt(InverseKinematics_B.ads * InverseKinematics_B.ads +
                InverseKinematics_B.bds * InverseKinematics_B.bds) *
      InverseKinematics_B.scale_h;
    if (InverseKinematics_B.roe < 0.0) {
      *b_a = -*b_a;
    }

    *c = a / *b_a;
    *s = b / *b_a;
    if (InverseKinematics_B.absa > InverseKinematics_B.absb) {
      *b_b = *s;
    } else if (*c != 0.0) {
      *b_b = 1.0 / *c;
    } else {
      *b_b = 1.0;
    }
  }
}

void InverseKinematics::InverseKinematics_xrot(const real_T x[9], int32_T ix0,
  int32_T iy0, real_T c, real_T s, real_T b_x[9])
{
  real_T temp;
  real_T temp_tmp;
  memcpy(&b_x[0], &x[0], 9U * sizeof(real_T));
  temp = b_x[iy0 - 1];
  temp_tmp = b_x[ix0 - 1];
  b_x[iy0 - 1] = temp * c - temp_tmp * s;
  b_x[ix0 - 1] = temp_tmp * c + temp * s;
  temp = b_x[ix0] * c + b_x[iy0] * s;
  b_x[iy0] = b_x[iy0] * c - b_x[ix0] * s;
  b_x[ix0] = temp;
  temp = b_x[iy0 + 1];
  temp_tmp = b_x[ix0 + 1];
  b_x[iy0 + 1] = temp * c - temp_tmp * s;
  b_x[ix0 + 1] = temp_tmp * c + temp * s;
}

void InverseKinematics::InverseKinematics_svd(const real_T A[9], real_T U[9],
  real_T s[3], real_T V[9])
{
  __m128d tmp;
  InverseKinematics_B.s[0] = 0.0;
  InverseKinematics_B.e_d[0] = 0.0;
  InverseKinematics_B.work[0] = 0.0;
  InverseKinematics_B.s[1] = 0.0;
  InverseKinematics_B.e_d[1] = 0.0;
  InverseKinematics_B.work[1] = 0.0;
  InverseKinematics_B.s[2] = 0.0;
  InverseKinematics_B.e_d[2] = 0.0;
  InverseKinematics_B.work[2] = 0.0;
  for (InverseKinematics_B.m_j = 0; InverseKinematics_B.m_j < 9;
       InverseKinematics_B.m_j++) {
    InverseKinematics_B.A[InverseKinematics_B.m_j] = A[InverseKinematics_B.m_j];
    U[InverseKinematics_B.m_j] = 0.0;
    V[InverseKinematics_B.m_j] = 0.0;
  }

  for (InverseKinematics_B.m_j = 0; InverseKinematics_B.m_j < 2;
       InverseKinematics_B.m_j++) {
    InverseKinematics_B.q_a = InverseKinematics_B.m_j + 1;
    InverseKinematics_B.qp1 = InverseKinematics_B.m_j + 2;
    InverseKinematics_B.qq_tmp = InverseKinematics_B.m_j * 3 +
      InverseKinematics_B.m_j;
    InverseKinematics_B.qq = InverseKinematics_B.qq_tmp + 1;
    InverseKinematics_B.apply_transform = false;
    InverseKinematics_B.nrm = InverseKinematics_xnrm2(3 -
      InverseKinematics_B.m_j, InverseKinematics_B.A, InverseKinematics_B.qq_tmp
      + 1);
    if (InverseKinematics_B.nrm > 0.0) {
      InverseKinematics_B.apply_transform = true;
      if (InverseKinematics_B.A[InverseKinematics_B.qq_tmp] < 0.0) {
        InverseKinematics_B.s[InverseKinematics_B.m_j] =
          -InverseKinematics_B.nrm;
      } else {
        InverseKinematics_B.s[InverseKinematics_B.m_j] = InverseKinematics_B.nrm;
      }

      if (fabs(InverseKinematics_B.s[InverseKinematics_B.m_j]) >=
          1.0020841800044864E-292) {
        InverseKinematics_B.nrm = 1.0 /
          InverseKinematics_B.s[InverseKinematics_B.m_j];
        InverseKinematics_B.b_ek = InverseKinematics_B.qq_tmp -
          InverseKinematics_B.m_j;
        InverseKinematics_B.qjj = (((((InverseKinematics_B.b_ek -
          InverseKinematics_B.qq_tmp) + 3) / 2) << 1) +
          InverseKinematics_B.qq_tmp) + 1;
        InverseKinematics_B.vectorUB_o = InverseKinematics_B.qjj - 2;
        for (InverseKinematics_B.k = InverseKinematics_B.qq;
             InverseKinematics_B.k <= InverseKinematics_B.vectorUB_o;
             InverseKinematics_B.k += 2) {
          tmp = _mm_loadu_pd(&InverseKinematics_B.A[InverseKinematics_B.k - 1]);
          _mm_storeu_pd(&InverseKinematics_B.A[InverseKinematics_B.k - 1],
                        _mm_mul_pd(tmp, _mm_set1_pd(InverseKinematics_B.nrm)));
        }

        for (InverseKinematics_B.k = InverseKinematics_B.qjj;
             InverseKinematics_B.k <= InverseKinematics_B.b_ek + 3;
             InverseKinematics_B.k++) {
          InverseKinematics_B.A[InverseKinematics_B.k - 1] *=
            InverseKinematics_B.nrm;
        }
      } else {
        InverseKinematics_B.b_ek = InverseKinematics_B.qq_tmp -
          InverseKinematics_B.m_j;
        InverseKinematics_B.qjj = (((((InverseKinematics_B.b_ek -
          InverseKinematics_B.qq_tmp) + 3) / 2) << 1) +
          InverseKinematics_B.qq_tmp) + 1;
        InverseKinematics_B.vectorUB_o = InverseKinematics_B.qjj - 2;
        for (InverseKinematics_B.k = InverseKinematics_B.qq;
             InverseKinematics_B.k <= InverseKinematics_B.vectorUB_o;
             InverseKinematics_B.k += 2) {
          tmp = _mm_loadu_pd(&InverseKinematics_B.A[InverseKinematics_B.k - 1]);
          _mm_storeu_pd(&InverseKinematics_B.A[InverseKinematics_B.k - 1],
                        _mm_div_pd(tmp, _mm_set1_pd
            (InverseKinematics_B.s[InverseKinematics_B.m_j])));
        }

        for (InverseKinematics_B.k = InverseKinematics_B.qjj;
             InverseKinematics_B.k <= InverseKinematics_B.b_ek + 3;
             InverseKinematics_B.k++) {
          InverseKinematics_B.A[InverseKinematics_B.k - 1] /=
            InverseKinematics_B.s[InverseKinematics_B.m_j];
        }
      }

      InverseKinematics_B.A[InverseKinematics_B.qq_tmp]++;
      InverseKinematics_B.s[InverseKinematics_B.m_j] =
        -InverseKinematics_B.s[InverseKinematics_B.m_j];
    } else {
      InverseKinematics_B.s[InverseKinematics_B.m_j] = 0.0;
    }

    for (InverseKinematics_B.qq = InverseKinematics_B.qp1;
         InverseKinematics_B.qq < 4; InverseKinematics_B.qq++) {
      InverseKinematics_B.qjj = ((InverseKinematics_B.qq - 1) * 3 +
        InverseKinematics_B.m_j) + 1;
      if (InverseKinematics_B.apply_transform) {
        memcpy(&InverseKinematics_B.A_g[0], &InverseKinematics_B.A[0], 9U *
               sizeof(real_T));
        InverseKinematics_xaxpy(3 - InverseKinematics_B.m_j,
          -(InverseKinematics_xdotc(3 - InverseKinematics_B.m_j,
          InverseKinematics_B.A, InverseKinematics_B.qq_tmp + 1,
          InverseKinematics_B.A, InverseKinematics_B.qjj) /
            InverseKinematics_B.A[InverseKinematics_B.qq_tmp]),
          InverseKinematics_B.qq_tmp + 1, InverseKinematics_B.A_g,
          InverseKinematics_B.qjj, InverseKinematics_B.A);
      }

      InverseKinematics_B.e_d[InverseKinematics_B.qq - 1] =
        InverseKinematics_B.A[InverseKinematics_B.qjj - 1];
    }

    memcpy(&U[(InverseKinematics_B.m_j * 3 + InverseKinematics_B.q_a) + -1],
           &InverseKinematics_B.A[(InverseKinematics_B.m_j * 3 +
            InverseKinematics_B.q_a) + -1], (-InverseKinematics_B.q_a + 4) *
           sizeof(real_T));
    if (InverseKinematics_B.m_j + 1 <= 1) {
      InverseKinematics_B.nrm = InverseKinematics_xnrm2_n
        (InverseKinematics_B.e_d, 2);
      if (InverseKinematics_B.nrm == 0.0) {
        InverseKinematics_B.e_d[0] = 0.0;
      } else {
        if (InverseKinematics_B.e_d[1] < 0.0) {
          InverseKinematics_B.rt = -InverseKinematics_B.nrm;
          InverseKinematics_B.e_d[0] = -InverseKinematics_B.nrm;
        } else {
          InverseKinematics_B.rt = InverseKinematics_B.nrm;
          InverseKinematics_B.e_d[0] = InverseKinematics_B.nrm;
        }

        if (fabs(InverseKinematics_B.rt) >= 1.0020841800044864E-292) {
          InverseKinematics_B.nrm = 1.0 / InverseKinematics_B.rt;
          InverseKinematics_B.qjj = ((((2 - InverseKinematics_B.m_j) / 2) << 1)
            + InverseKinematics_B.m_j) + 2;
          InverseKinematics_B.vectorUB_o = InverseKinematics_B.qjj - 2;
          for (InverseKinematics_B.k = InverseKinematics_B.qp1;
               InverseKinematics_B.k <= InverseKinematics_B.vectorUB_o;
               InverseKinematics_B.k += 2) {
            tmp = _mm_loadu_pd(&InverseKinematics_B.e_d[InverseKinematics_B.k -
                               1]);
            _mm_storeu_pd(&InverseKinematics_B.e_d[InverseKinematics_B.k - 1],
                          _mm_mul_pd(tmp, _mm_set1_pd(InverseKinematics_B.nrm)));
          }

          for (InverseKinematics_B.k = InverseKinematics_B.qjj;
               InverseKinematics_B.k < 4; InverseKinematics_B.k++) {
            InverseKinematics_B.e_d[InverseKinematics_B.k - 1] *=
              InverseKinematics_B.nrm;
          }
        } else {
          InverseKinematics_B.qjj = ((((2 - InverseKinematics_B.m_j) / 2) << 1)
            + InverseKinematics_B.m_j) + 2;
          InverseKinematics_B.vectorUB_o = InverseKinematics_B.qjj - 2;
          for (InverseKinematics_B.k = InverseKinematics_B.qp1;
               InverseKinematics_B.k <= InverseKinematics_B.vectorUB_o;
               InverseKinematics_B.k += 2) {
            tmp = _mm_loadu_pd(&InverseKinematics_B.e_d[InverseKinematics_B.k -
                               1]);
            _mm_storeu_pd(&InverseKinematics_B.e_d[InverseKinematics_B.k - 1],
                          _mm_div_pd(tmp, _mm_set1_pd(InverseKinematics_B.rt)));
          }

          for (InverseKinematics_B.k = InverseKinematics_B.qjj;
               InverseKinematics_B.k < 4; InverseKinematics_B.k++) {
            InverseKinematics_B.e_d[InverseKinematics_B.k - 1] /=
              InverseKinematics_B.rt;
          }
        }

        InverseKinematics_B.e_d[1]++;
        InverseKinematics_B.e_d[0] = -InverseKinematics_B.e_d[0];
        for (InverseKinematics_B.q_a = InverseKinematics_B.qp1;
             InverseKinematics_B.q_a < 4; InverseKinematics_B.q_a++) {
          InverseKinematics_B.work[InverseKinematics_B.q_a - 1] = 0.0;
        }

        for (InverseKinematics_B.q_a = InverseKinematics_B.qp1;
             InverseKinematics_B.q_a < 4; InverseKinematics_B.q_a++) {
          InverseKinematics_xaxpy_nly(2,
            InverseKinematics_B.e_d[InverseKinematics_B.q_a - 1],
            InverseKinematics_B.A, 3 * (InverseKinematics_B.q_a - 1) + 2,
            InverseKinematics_B.work, 2);
        }

        for (InverseKinematics_B.q_a = InverseKinematics_B.qp1;
             InverseKinematics_B.q_a < 4; InverseKinematics_B.q_a++) {
          memcpy(&InverseKinematics_B.A_g[0], &InverseKinematics_B.A[0], 9U *
                 sizeof(real_T));
          InverseKinematics_xaxpy_nl(2,
            -InverseKinematics_B.e_d[InverseKinematics_B.q_a - 1] /
            InverseKinematics_B.e_d[1], InverseKinematics_B.work, 2,
            InverseKinematics_B.A_g, (InverseKinematics_B.q_a - 1) * 3 + 2,
            InverseKinematics_B.A);
        }
      }

      for (InverseKinematics_B.q_a = InverseKinematics_B.qp1;
           InverseKinematics_B.q_a < 4; InverseKinematics_B.q_a++) {
        V[InverseKinematics_B.q_a - 1] =
          InverseKinematics_B.e_d[InverseKinematics_B.q_a - 1];
      }
    }
  }

  InverseKinematics_B.m_j = 2;
  InverseKinematics_B.s[2] = InverseKinematics_B.A[8];
  InverseKinematics_B.e_d[1] = InverseKinematics_B.A[7];
  InverseKinematics_B.e_d[2] = 0.0;
  U[6] = 0.0;
  U[7] = 0.0;
  U[8] = 1.0;
  for (InverseKinematics_B.q_a = 1; InverseKinematics_B.q_a >= 0;
       InverseKinematics_B.q_a--) {
    InverseKinematics_B.qq = 3 * InverseKinematics_B.q_a +
      InverseKinematics_B.q_a;
    if (InverseKinematics_B.s[InverseKinematics_B.q_a] != 0.0) {
      for (InverseKinematics_B.qp1 = InverseKinematics_B.q_a + 2;
           InverseKinematics_B.qp1 < 4; InverseKinematics_B.qp1++) {
        InverseKinematics_B.qjj = ((InverseKinematics_B.qp1 - 1) * 3 +
          InverseKinematics_B.q_a) + 1;
        memcpy(&InverseKinematics_B.A[0], &U[0], 9U * sizeof(real_T));
        InverseKinematics_xaxpy(3 - InverseKinematics_B.q_a,
          -(InverseKinematics_xdotc(3 - InverseKinematics_B.q_a, U,
          InverseKinematics_B.qq + 1, U, InverseKinematics_B.qjj) /
            U[InverseKinematics_B.qq]), InverseKinematics_B.qq + 1,
          InverseKinematics_B.A, InverseKinematics_B.qjj, U);
      }

      for (InverseKinematics_B.qp1 = InverseKinematics_B.q_a + 1;
           InverseKinematics_B.qp1 < 4; InverseKinematics_B.qp1++) {
        InverseKinematics_B.qjj = (3 * InverseKinematics_B.q_a +
          InverseKinematics_B.qp1) - 1;
        U[InverseKinematics_B.qjj] = -U[InverseKinematics_B.qjj];
      }

      U[InverseKinematics_B.qq]++;
      if (InverseKinematics_B.q_a - 1 >= 0) {
        U[3 * InverseKinematics_B.q_a] = 0.0;
      }
    } else {
      U[3 * InverseKinematics_B.q_a] = 0.0;
      U[3 * InverseKinematics_B.q_a + 1] = 0.0;
      U[3 * InverseKinematics_B.q_a + 2] = 0.0;
      U[InverseKinematics_B.qq] = 1.0;
    }
  }

  for (InverseKinematics_B.q_a = 2; InverseKinematics_B.q_a >= 0;
       InverseKinematics_B.q_a--) {
    if ((InverseKinematics_B.q_a + 1 <= 1) && (InverseKinematics_B.e_d[0] != 0.0))
    {
      memcpy(&InverseKinematics_B.A[0], &V[0], 9U * sizeof(real_T));
      InverseKinematics_xaxpy(2, -(InverseKinematics_xdotc(2, V, 2, V, 5) / V[1]),
        2, InverseKinematics_B.A, 5, V);
      memcpy(&InverseKinematics_B.A[0], &V[0], 9U * sizeof(real_T));
      InverseKinematics_xaxpy(2, -(InverseKinematics_xdotc(2, V, 2, V, 8) / V[1]),
        2, InverseKinematics_B.A, 8, V);
    }

    V[3 * InverseKinematics_B.q_a] = 0.0;
    V[3 * InverseKinematics_B.q_a + 1] = 0.0;
    V[3 * InverseKinematics_B.q_a + 2] = 0.0;
    V[InverseKinematics_B.q_a + 3 * InverseKinematics_B.q_a] = 1.0;
  }

  for (InverseKinematics_B.q_a = 0; InverseKinematics_B.q_a < 3;
       InverseKinematics_B.q_a++) {
    InverseKinematics_B.ztest = InverseKinematics_B.e_d[InverseKinematics_B.q_a];
    if (InverseKinematics_B.s[InverseKinematics_B.q_a] != 0.0) {
      InverseKinematics_B.rt = fabs
        (InverseKinematics_B.s[InverseKinematics_B.q_a]);
      InverseKinematics_B.nrm = InverseKinematics_B.s[InverseKinematics_B.q_a] /
        InverseKinematics_B.rt;
      InverseKinematics_B.s[InverseKinematics_B.q_a] = InverseKinematics_B.rt;
      if (InverseKinematics_B.q_a + 1 < 3) {
        InverseKinematics_B.ztest /= InverseKinematics_B.nrm;
      }

      InverseKinematics_B.qp1 = 3 * InverseKinematics_B.q_a;
      InverseKinematics_B.qjj = InverseKinematics_B.qp1 + 3;
      InverseKinematics_B.vectorUB_o = InverseKinematics_B.qp1 + 1;
      for (InverseKinematics_B.k = InverseKinematics_B.qp1 + 1;
           InverseKinematics_B.k <= InverseKinematics_B.vectorUB_o;
           InverseKinematics_B.k += 2) {
        tmp = _mm_loadu_pd(&U[InverseKinematics_B.k - 1]);
        _mm_storeu_pd(&U[InverseKinematics_B.k - 1], _mm_mul_pd(tmp, _mm_set1_pd
          (InverseKinematics_B.nrm)));
      }

      for (InverseKinematics_B.k = InverseKinematics_B.qjj;
           InverseKinematics_B.k <= InverseKinematics_B.qp1 + 3;
           InverseKinematics_B.k++) {
        U[InverseKinematics_B.k - 1] *= InverseKinematics_B.nrm;
      }
    }

    if ((InverseKinematics_B.q_a + 1 < 3) && (InverseKinematics_B.ztest != 0.0))
    {
      InverseKinematics_B.rt = fabs(InverseKinematics_B.ztest);
      InverseKinematics_B.nrm = InverseKinematics_B.rt /
        InverseKinematics_B.ztest;
      InverseKinematics_B.ztest = InverseKinematics_B.rt;
      InverseKinematics_B.s[InverseKinematics_B.q_a + 1] *=
        InverseKinematics_B.nrm;
      InverseKinematics_B.qp1 = (InverseKinematics_B.q_a + 1) * 3;
      InverseKinematics_B.qjj = InverseKinematics_B.qp1 + 3;
      InverseKinematics_B.vectorUB_o = InverseKinematics_B.qp1 + 1;
      for (InverseKinematics_B.k = InverseKinematics_B.qp1 + 1;
           InverseKinematics_B.k <= InverseKinematics_B.vectorUB_o;
           InverseKinematics_B.k += 2) {
        tmp = _mm_loadu_pd(&V[InverseKinematics_B.k - 1]);
        _mm_storeu_pd(&V[InverseKinematics_B.k - 1], _mm_mul_pd(tmp, _mm_set1_pd
          (InverseKinematics_B.nrm)));
      }

      for (InverseKinematics_B.k = InverseKinematics_B.qjj;
           InverseKinematics_B.k <= InverseKinematics_B.qp1 + 3;
           InverseKinematics_B.k++) {
        V[InverseKinematics_B.k - 1] *= InverseKinematics_B.nrm;
      }
    }

    InverseKinematics_B.e_d[InverseKinematics_B.q_a] = InverseKinematics_B.ztest;
  }

  InverseKinematics_B.qp1 = 0;
  InverseKinematics_B.nrm = 0.0;
  InverseKinematics_B.ztest = fabs(InverseKinematics_B.s[0]);
  InverseKinematics_B.rt = fabs(InverseKinematics_B.e_d[0]);
  if ((InverseKinematics_B.ztest >= InverseKinematics_B.rt) || rtIsNaN
      (InverseKinematics_B.rt)) {
    InverseKinematics_B.rt = InverseKinematics_B.ztest;
  }

  if ((!(InverseKinematics_B.rt <= 0.0)) && (!rtIsNaN(InverseKinematics_B.rt)))
  {
    InverseKinematics_B.nrm = InverseKinematics_B.rt;
  }

  InverseKinematics_B.ztest = fabs(InverseKinematics_B.s[1]);
  InverseKinematics_B.rt = fabs(InverseKinematics_B.e_d[1]);
  if ((InverseKinematics_B.ztest >= InverseKinematics_B.rt) || rtIsNaN
      (InverseKinematics_B.rt)) {
    InverseKinematics_B.rt = InverseKinematics_B.ztest;
  }

  if ((!(InverseKinematics_B.nrm >= InverseKinematics_B.rt)) && (!rtIsNaN
       (InverseKinematics_B.rt))) {
    InverseKinematics_B.nrm = InverseKinematics_B.rt;
  }

  InverseKinematics_B.ztest = fabs(InverseKinematics_B.s[2]);
  InverseKinematics_B.rt = fabs(InverseKinematics_B.e_d[2]);
  if ((InverseKinematics_B.ztest >= InverseKinematics_B.rt) || rtIsNaN
      (InverseKinematics_B.rt)) {
    InverseKinematics_B.rt = InverseKinematics_B.ztest;
  }

  if ((!(InverseKinematics_B.nrm >= InverseKinematics_B.rt)) && (!rtIsNaN
       (InverseKinematics_B.rt))) {
    InverseKinematics_B.nrm = InverseKinematics_B.rt;
  }

  while ((InverseKinematics_B.m_j + 1 > 0) && (!(InverseKinematics_B.qp1 >= 75)))
  {
    boolean_T exitg1;
    InverseKinematics_B.q_a = InverseKinematics_B.m_j;
    InverseKinematics_B.qq = InverseKinematics_B.m_j;
    exitg1 = false;
    while ((!exitg1) && (InverseKinematics_B.qq > -1)) {
      InverseKinematics_B.q_a = InverseKinematics_B.qq;
      if (InverseKinematics_B.qq == 0) {
        exitg1 = true;
      } else {
        InverseKinematics_B.rt = fabs
          (InverseKinematics_B.e_d[InverseKinematics_B.qq - 1]);
        if ((InverseKinematics_B.rt <= (fabs
              (InverseKinematics_B.s[InverseKinematics_B.qq - 1]) + fabs
              (InverseKinematics_B.s[InverseKinematics_B.qq])) *
             2.2204460492503131E-16) || (InverseKinematics_B.rt <=
             1.0020841800044864E-292) || ((InverseKinematics_B.qp1 > 20) &&
             (InverseKinematics_B.rt <= 2.2204460492503131E-16 *
              InverseKinematics_B.nrm))) {
          InverseKinematics_B.e_d[InverseKinematics_B.qq - 1] = 0.0;
          exitg1 = true;
        } else {
          InverseKinematics_B.qq--;
        }
      }
    }

    if (InverseKinematics_B.q_a == InverseKinematics_B.m_j) {
      InverseKinematics_B.qjj = 4;
    } else {
      InverseKinematics_B.qq = InverseKinematics_B.m_j + 1;
      InverseKinematics_B.qjj = InverseKinematics_B.m_j + 1;
      exitg1 = false;
      while ((!exitg1) && (InverseKinematics_B.qjj >= InverseKinematics_B.q_a))
      {
        InverseKinematics_B.qq = InverseKinematics_B.qjj;
        if (InverseKinematics_B.qjj == InverseKinematics_B.q_a) {
          exitg1 = true;
        } else {
          InverseKinematics_B.rt = 0.0;
          if (InverseKinematics_B.qjj < InverseKinematics_B.m_j + 1) {
            InverseKinematics_B.rt = fabs
              (InverseKinematics_B.e_d[InverseKinematics_B.qjj - 1]);
          }

          if (InverseKinematics_B.qjj > InverseKinematics_B.q_a + 1) {
            InverseKinematics_B.rt += fabs
              (InverseKinematics_B.e_d[InverseKinematics_B.qjj - 2]);
          }

          InverseKinematics_B.ztest = fabs
            (InverseKinematics_B.s[InverseKinematics_B.qjj - 1]);
          if ((InverseKinematics_B.ztest <= 2.2204460492503131E-16 *
               InverseKinematics_B.rt) || (InverseKinematics_B.ztest <=
               1.0020841800044864E-292)) {
            InverseKinematics_B.s[InverseKinematics_B.qjj - 1] = 0.0;
            exitg1 = true;
          } else {
            InverseKinematics_B.qjj--;
          }
        }
      }

      if (InverseKinematics_B.qq == InverseKinematics_B.q_a) {
        InverseKinematics_B.qjj = 3;
      } else if (InverseKinematics_B.m_j + 1 == InverseKinematics_B.qq) {
        InverseKinematics_B.qjj = 1;
      } else {
        InverseKinematics_B.qjj = 2;
        InverseKinematics_B.q_a = InverseKinematics_B.qq;
      }
    }

    switch (InverseKinematics_B.qjj) {
     case 1:
      InverseKinematics_B.rt = InverseKinematics_B.e_d[InverseKinematics_B.m_j -
        1];
      InverseKinematics_B.e_d[InverseKinematics_B.m_j - 1] = 0.0;
      for (InverseKinematics_B.qq = InverseKinematics_B.m_j;
           InverseKinematics_B.qq >= InverseKinematics_B.q_a + 1;
           InverseKinematics_B.qq--) {
        InverseKinematics_B.ztest = InverseKinematics_B.e_d[0];
        InverseKinematics_xrotg(InverseKinematics_B.s[InverseKinematics_B.qq - 1],
          InverseKinematics_B.rt, &InverseKinematics_B.s[InverseKinematics_B.qq
          - 1], &InverseKinematics_B.rt, &InverseKinematics_B.sqds,
          &InverseKinematics_B.b_ja);
        if (InverseKinematics_B.qq > InverseKinematics_B.q_a + 1) {
          InverseKinematics_B.rt = -InverseKinematics_B.b_ja *
            InverseKinematics_B.e_d[0];
          InverseKinematics_B.ztest = InverseKinematics_B.e_d[0] *
            InverseKinematics_B.sqds;
        }

        memcpy(&InverseKinematics_B.A[0], &V[0], 9U * sizeof(real_T));
        InverseKinematics_xrot(InverseKinematics_B.A, (InverseKinematics_B.qq -
          1) * 3 + 1, 3 * InverseKinematics_B.m_j + 1, InverseKinematics_B.sqds,
          InverseKinematics_B.b_ja, V);
        InverseKinematics_B.e_d[0] = InverseKinematics_B.ztest;
      }
      break;

     case 2:
      InverseKinematics_B.rt = InverseKinematics_B.e_d[InverseKinematics_B.q_a -
        1];
      InverseKinematics_B.e_d[InverseKinematics_B.q_a - 1] = 0.0;
      for (InverseKinematics_B.qq = InverseKinematics_B.q_a + 1;
           InverseKinematics_B.qq <= InverseKinematics_B.m_j + 1;
           InverseKinematics_B.qq++) {
        InverseKinematics_xrotg(InverseKinematics_B.s[InverseKinematics_B.qq - 1],
          InverseKinematics_B.rt, &InverseKinematics_B.s[InverseKinematics_B.qq
          - 1], &InverseKinematics_B.ztest, &InverseKinematics_B.sqds,
          &InverseKinematics_B.b_ja);
        InverseKinematics_B.ztest =
          InverseKinematics_B.e_d[InverseKinematics_B.qq - 1];
        InverseKinematics_B.rt = InverseKinematics_B.ztest *
          -InverseKinematics_B.b_ja;
        InverseKinematics_B.e_d[InverseKinematics_B.qq - 1] =
          InverseKinematics_B.ztest * InverseKinematics_B.sqds;
        memcpy(&InverseKinematics_B.A[0], &U[0], 9U * sizeof(real_T));
        InverseKinematics_xrot(InverseKinematics_B.A, (InverseKinematics_B.qq -
          1) * 3 + 1, (InverseKinematics_B.q_a - 1) * 3 + 1,
          InverseKinematics_B.sqds, InverseKinematics_B.b_ja, U);
      }
      break;

     case 3:
      InverseKinematics_B.ztest = fabs
        (InverseKinematics_B.s[InverseKinematics_B.m_j]);
      InverseKinematics_B.sqds = InverseKinematics_B.s[InverseKinematics_B.m_j -
        1];
      InverseKinematics_B.rt = fabs(InverseKinematics_B.sqds);
      if ((InverseKinematics_B.ztest >= InverseKinematics_B.rt) || rtIsNaN
          (InverseKinematics_B.rt)) {
        InverseKinematics_B.rt = InverseKinematics_B.ztest;
      }

      InverseKinematics_B.b_ja = InverseKinematics_B.e_d[InverseKinematics_B.m_j
        - 1];
      InverseKinematics_B.ztest = fabs(InverseKinematics_B.b_ja);
      if ((InverseKinematics_B.rt >= InverseKinematics_B.ztest) || rtIsNaN
          (InverseKinematics_B.ztest)) {
        InverseKinematics_B.ztest = InverseKinematics_B.rt;
      }

      InverseKinematics_B.rt = fabs
        (InverseKinematics_B.s[InverseKinematics_B.q_a]);
      if ((InverseKinematics_B.ztest >= InverseKinematics_B.rt) || rtIsNaN
          (InverseKinematics_B.rt)) {
        InverseKinematics_B.rt = InverseKinematics_B.ztest;
      }

      InverseKinematics_B.ztest = fabs
        (InverseKinematics_B.e_d[InverseKinematics_B.q_a]);
      if ((InverseKinematics_B.rt >= InverseKinematics_B.ztest) || rtIsNaN
          (InverseKinematics_B.ztest)) {
        InverseKinematics_B.ztest = InverseKinematics_B.rt;
      }

      InverseKinematics_B.rt = InverseKinematics_B.s[InverseKinematics_B.m_j] /
        InverseKinematics_B.ztest;
      InverseKinematics_B.smm1 = InverseKinematics_B.sqds /
        InverseKinematics_B.ztest;
      InverseKinematics_B.emm1 = InverseKinematics_B.b_ja /
        InverseKinematics_B.ztest;
      InverseKinematics_B.sqds = InverseKinematics_B.s[InverseKinematics_B.q_a] /
        InverseKinematics_B.ztest;
      InverseKinematics_B.b_ja = ((InverseKinematics_B.smm1 +
        InverseKinematics_B.rt) * (InverseKinematics_B.smm1 -
        InverseKinematics_B.rt) + InverseKinematics_B.emm1 *
        InverseKinematics_B.emm1) / 2.0;
      InverseKinematics_B.smm1 = InverseKinematics_B.rt *
        InverseKinematics_B.emm1;
      InverseKinematics_B.smm1 *= InverseKinematics_B.smm1;
      if ((InverseKinematics_B.b_ja != 0.0) || (InverseKinematics_B.smm1 != 0.0))
      {
        InverseKinematics_B.emm1 = sqrt(InverseKinematics_B.b_ja *
          InverseKinematics_B.b_ja + InverseKinematics_B.smm1);
        if (InverseKinematics_B.b_ja < 0.0) {
          InverseKinematics_B.emm1 = -InverseKinematics_B.emm1;
        }

        InverseKinematics_B.emm1 = InverseKinematics_B.smm1 /
          (InverseKinematics_B.b_ja + InverseKinematics_B.emm1);
      } else {
        InverseKinematics_B.emm1 = 0.0;
      }

      InverseKinematics_B.rt = (InverseKinematics_B.sqds +
        InverseKinematics_B.rt) * (InverseKinematics_B.sqds -
        InverseKinematics_B.rt) + InverseKinematics_B.emm1;
      InverseKinematics_B.sqds *=
        InverseKinematics_B.e_d[InverseKinematics_B.q_a] /
        InverseKinematics_B.ztest;
      for (InverseKinematics_B.vectorUB_o = InverseKinematics_B.q_a + 1;
           InverseKinematics_B.vectorUB_o <= InverseKinematics_B.m_j;
           InverseKinematics_B.vectorUB_o++) {
        InverseKinematics_xrotg(InverseKinematics_B.rt, InverseKinematics_B.sqds,
          &InverseKinematics_B.ztest, &InverseKinematics_B.emm1,
          &InverseKinematics_B.b_ja, &InverseKinematics_B.smm1);
        if (InverseKinematics_B.vectorUB_o > InverseKinematics_B.q_a + 1) {
          InverseKinematics_B.e_d[0] = InverseKinematics_B.ztest;
        }

        InverseKinematics_B.ztest =
          InverseKinematics_B.e_d[InverseKinematics_B.vectorUB_o - 1];
        InverseKinematics_B.rt =
          InverseKinematics_B.s[InverseKinematics_B.vectorUB_o - 1];
        InverseKinematics_B.e_d[InverseKinematics_B.vectorUB_o - 1] =
          InverseKinematics_B.ztest * InverseKinematics_B.b_ja -
          InverseKinematics_B.rt * InverseKinematics_B.smm1;
        InverseKinematics_B.sqds = InverseKinematics_B.smm1 *
          InverseKinematics_B.s[InverseKinematics_B.vectorUB_o];
        InverseKinematics_B.s[InverseKinematics_B.vectorUB_o] *=
          InverseKinematics_B.b_ja;
        InverseKinematics_B.qq = (InverseKinematics_B.vectorUB_o - 1) * 3 + 1;
        InverseKinematics_B.qjj = 3 * InverseKinematics_B.vectorUB_o + 1;
        memcpy(&InverseKinematics_B.A[0], &V[0], 9U * sizeof(real_T));
        InverseKinematics_xrot(InverseKinematics_B.A, InverseKinematics_B.qq,
          InverseKinematics_B.qjj, InverseKinematics_B.b_ja,
          InverseKinematics_B.smm1, V);
        InverseKinematics_xrotg(InverseKinematics_B.rt *
          InverseKinematics_B.b_ja + InverseKinematics_B.ztest *
          InverseKinematics_B.smm1, InverseKinematics_B.sqds,
          &InverseKinematics_B.s[InverseKinematics_B.vectorUB_o - 1],
          &InverseKinematics_B.a__3, &InverseKinematics_B.emm1,
          &InverseKinematics_B.d_sn);
        InverseKinematics_B.rt =
          InverseKinematics_B.e_d[InverseKinematics_B.vectorUB_o - 1] *
          InverseKinematics_B.emm1 + InverseKinematics_B.d_sn *
          InverseKinematics_B.s[InverseKinematics_B.vectorUB_o];
        InverseKinematics_B.s[InverseKinematics_B.vectorUB_o] =
          InverseKinematics_B.e_d[InverseKinematics_B.vectorUB_o - 1] *
          -InverseKinematics_B.d_sn + InverseKinematics_B.emm1 *
          InverseKinematics_B.s[InverseKinematics_B.vectorUB_o];
        InverseKinematics_B.sqds = InverseKinematics_B.d_sn *
          InverseKinematics_B.e_d[InverseKinematics_B.vectorUB_o];
        InverseKinematics_B.e_d[InverseKinematics_B.vectorUB_o] *=
          InverseKinematics_B.emm1;
        memcpy(&InverseKinematics_B.A[0], &U[0], 9U * sizeof(real_T));
        InverseKinematics_xrot(InverseKinematics_B.A, InverseKinematics_B.qq,
          InverseKinematics_B.qjj, InverseKinematics_B.emm1,
          InverseKinematics_B.d_sn, U);
      }

      InverseKinematics_B.e_d[InverseKinematics_B.m_j - 1] =
        InverseKinematics_B.rt;
      InverseKinematics_B.qp1++;
      break;

     default:
      if (InverseKinematics_B.s[InverseKinematics_B.q_a] < 0.0) {
        InverseKinematics_B.s[InverseKinematics_B.q_a] =
          -InverseKinematics_B.s[InverseKinematics_B.q_a];
        InverseKinematics_B.qp1 = 3 * InverseKinematics_B.q_a;
        InverseKinematics_B.qjj = InverseKinematics_B.qp1 + 3;
        InverseKinematics_B.vectorUB_o = InverseKinematics_B.qp1 + 1;
        for (InverseKinematics_B.k = InverseKinematics_B.qp1 + 1;
             InverseKinematics_B.k <= InverseKinematics_B.vectorUB_o;
             InverseKinematics_B.k += 2) {
          tmp = _mm_loadu_pd(&V[InverseKinematics_B.k - 1]);
          _mm_storeu_pd(&V[InverseKinematics_B.k - 1], _mm_mul_pd(tmp,
            _mm_set1_pd(-1.0)));
        }

        for (InverseKinematics_B.k = InverseKinematics_B.qjj;
             InverseKinematics_B.k <= InverseKinematics_B.qp1 + 3;
             InverseKinematics_B.k++) {
          V[InverseKinematics_B.k - 1] = -V[InverseKinematics_B.k - 1];
        }
      }

      InverseKinematics_B.qp1 = InverseKinematics_B.q_a + 1;
      while ((InverseKinematics_B.q_a + 1 < 3) &&
             (InverseKinematics_B.s[InverseKinematics_B.q_a] <
              InverseKinematics_B.s[InverseKinematics_B.qp1])) {
        InverseKinematics_B.rt = InverseKinematics_B.s[InverseKinematics_B.q_a];
        InverseKinematics_B.s[InverseKinematics_B.q_a] =
          InverseKinematics_B.s[InverseKinematics_B.qp1];
        InverseKinematics_B.s[InverseKinematics_B.qp1] = InverseKinematics_B.rt;
        InverseKinematics_B.qq = 3 * InverseKinematics_B.q_a + 1;
        InverseKinematics_B.qjj = (InverseKinematics_B.q_a + 1) * 3 + 1;
        memcpy(&InverseKinematics_B.A[0], &V[0], 9U * sizeof(real_T));
        InverseKinematics_xswap(InverseKinematics_B.A, InverseKinematics_B.qq,
          InverseKinematics_B.qjj, V);
        memcpy(&InverseKinematics_B.A[0], &U[0], 9U * sizeof(real_T));
        InverseKinematics_xswap(InverseKinematics_B.A, InverseKinematics_B.qq,
          InverseKinematics_B.qjj, U);
        InverseKinematics_B.q_a = InverseKinematics_B.qp1;
        InverseKinematics_B.qp1++;
      }

      InverseKinematics_B.qp1 = 0;
      InverseKinematics_B.m_j--;
      break;
    }
  }

  s[0] = InverseKinematics_B.s[0];
  s[1] = InverseKinematics_B.s[1];
  s[2] = InverseKinematics_B.s[2];
}

void InverseKinematics::InverseKine_IKHelpers_poseError(const real_T Td[16],
  const real_T T_data[], const int32_T T_size[2], real_T errorvec[6])
{
  __m128d tmp_0;
  boolean_T exitg1;
  InverseKinematics_B.y_tmp_i[0] = 1;
  InverseKinematics_B.y_tmp_i[1] = 2;
  InverseKinematics_B.y_tmp_i[2] = 3;
  for (InverseKinematics_B.b_k_px = 0; InverseKinematics_B.b_k_px < 3;
       InverseKinematics_B.b_k_px++) {
    InverseKinematics_B.y_tmp_o =
      InverseKinematics_B.y_tmp_i[InverseKinematics_B.b_k_px];
    InverseKinematics_B.T[3 * InverseKinematics_B.b_k_px] =
      T_data[InverseKinematics_B.y_tmp_o - 1];
    InverseKinematics_B.T_tmp = 3 * InverseKinematics_B.b_k_px + 1;
    InverseKinematics_B.T[InverseKinematics_B.T_tmp] = T_data
      [(InverseKinematics_B.y_tmp_o + T_size[0]) - 1];
    InverseKinematics_B.T_tmp_p = 3 * InverseKinematics_B.b_k_px + 2;
    InverseKinematics_B.T[InverseKinematics_B.T_tmp_p] = T_data[((T_size[0] << 1)
      + InverseKinematics_B.y_tmp_o) - 1];
    for (InverseKinematics_B.i3 = 0; InverseKinematics_B.i3 <= 0;
         InverseKinematics_B.i3 += 2) {
      InverseKinematics_B.y_tmp = 3 * InverseKinematics_B.b_k_px +
        InverseKinematics_B.i3;
      _mm_storeu_pd(&InverseKinematics_B.y[InverseKinematics_B.y_tmp],
                    _mm_set1_pd(0.0));
      tmp_0 = _mm_loadu_pd(&InverseKinematics_B.y[InverseKinematics_B.y_tmp]);
      _mm_storeu_pd(&InverseKinematics_B.y[InverseKinematics_B.y_tmp],
                    _mm_add_pd(tmp_0, _mm_mul_pd(_mm_set1_pd
        (InverseKinematics_B.T[3 * InverseKinematics_B.b_k_px]), _mm_loadu_pd
        (&Td[InverseKinematics_B.i3]))));
      tmp_0 = _mm_loadu_pd(&InverseKinematics_B.y[InverseKinematics_B.y_tmp]);
      _mm_storeu_pd(&InverseKinematics_B.y[InverseKinematics_B.y_tmp],
                    _mm_add_pd(_mm_mul_pd(_mm_set1_pd
        (InverseKinematics_B.T[InverseKinematics_B.T_tmp]), _mm_loadu_pd
        (&Td[InverseKinematics_B.i3 + 4])), tmp_0));
      tmp_0 = _mm_loadu_pd(&InverseKinematics_B.y[InverseKinematics_B.y_tmp]);
      _mm_storeu_pd(&InverseKinematics_B.y[InverseKinematics_B.y_tmp],
                    _mm_add_pd(_mm_mul_pd(_mm_set1_pd
        (InverseKinematics_B.T[InverseKinematics_B.T_tmp_p]), _mm_loadu_pd
        (&Td[InverseKinematics_B.i3 + 8])), tmp_0));
    }

    for (InverseKinematics_B.i3 = 2; InverseKinematics_B.i3 < 3;
         InverseKinematics_B.i3++) {
      InverseKinematics_B.y_tmp = 3 * InverseKinematics_B.b_k_px +
        InverseKinematics_B.i3;
      InverseKinematics_B.y[InverseKinematics_B.y_tmp] = 0.0;
      InverseKinematics_B.y[InverseKinematics_B.y_tmp] += InverseKinematics_B.T
        [3 * InverseKinematics_B.b_k_px] * Td[InverseKinematics_B.i3];
      InverseKinematics_B.y[InverseKinematics_B.y_tmp] +=
        Td[InverseKinematics_B.i3 + 4] *
        InverseKinematics_B.T[InverseKinematics_B.T_tmp];
      InverseKinematics_B.y[InverseKinematics_B.y_tmp] +=
        Td[InverseKinematics_B.i3 + 8] *
        InverseKinematics_B.T[InverseKinematics_B.T_tmp_p];
    }
  }

  InverseKinematics_B.u.re = (((InverseKinematics_B.y[0] +
    InverseKinematics_B.y[4]) + InverseKinematics_B.y[8]) - 1.0) * 0.5;
  if (!(fabs(InverseKinematics_B.u.re) > 1.0)) {
    InverseKinematics_B.v_lx.re = acos(InverseKinematics_B.u.re);
  } else {
    InverseKinematics_B.u_o.re = InverseKinematics_B.u.re + 1.0;
    InverseKinematics_B.u_o.im = 0.0;
    InverseKinematics_B.dc.re = 1.0 - InverseKinematics_B.u.re;
    InverseKinematics_B.dc.im = 0.0;
    InverseKinematics_B.v_lx.re = 2.0 * rt_atan2d_snf((InverseKinematics_sqrt
      (InverseKinematics_B.dc)).re, (InverseKinematics_sqrt
      (InverseKinematics_B.u_o)).re);
  }

  InverseKinematics_B.a_m = 2.0 * sin(InverseKinematics_B.v_lx.re);
  InverseKinematics_B.v_g[0] = (InverseKinematics_B.y[5] -
    InverseKinematics_B.y[7]) / InverseKinematics_B.a_m;
  InverseKinematics_B.v_g[1] = (InverseKinematics_B.y[6] -
    InverseKinematics_B.y[2]) / InverseKinematics_B.a_m;
  InverseKinematics_B.v_g[2] = (InverseKinematics_B.y[1] -
    InverseKinematics_B.y[3]) / InverseKinematics_B.a_m;
  if (rtIsNaN(InverseKinematics_B.v_lx.re) || rtIsInf
      (InverseKinematics_B.v_lx.re)) {
    InverseKinematics_B.a_m = (rtNaN);
  } else if (InverseKinematics_B.v_lx.re == 0.0) {
    InverseKinematics_B.a_m = 0.0;
  } else {
    InverseKinematics_B.a_m = fmod(InverseKinematics_B.v_lx.re,
      3.1415926535897931);
    InverseKinematics_B.rEQ0 = (InverseKinematics_B.a_m == 0.0);
    if (!InverseKinematics_B.rEQ0) {
      InverseKinematics_B.q = fabs(InverseKinematics_B.v_lx.re /
        3.1415926535897931);
      InverseKinematics_B.rEQ0 = !(fabs(InverseKinematics_B.q - floor
        (InverseKinematics_B.q + 0.5)) > 2.2204460492503131E-16 *
        InverseKinematics_B.q);
    }

    if (InverseKinematics_B.rEQ0) {
      InverseKinematics_B.a_m = 0.0;
    } else if (InverseKinematics_B.v_lx.re < 0.0) {
      InverseKinematics_B.a_m += 3.1415926535897931;
    }
  }

  InverseKinematics_B.rEQ0 = (InverseKinematics_B.a_m == 0.0);
  InverseKinematics_B.e_p = true;
  InverseKinematics_B.b_k_px = 0;
  exitg1 = false;
  while ((!exitg1) && (InverseKinematics_B.b_k_px < 3)) {
    if (!(InverseKinematics_B.v_g[InverseKinematics_B.b_k_px] == 0.0)) {
      InverseKinematics_B.e_p = false;
      exitg1 = true;
    } else {
      InverseKinematics_B.b_k_px++;
    }
  }

  if (InverseKinematics_B.rEQ0 || InverseKinematics_B.e_p) {
    InverseKinematics_B.T_tmp = (InverseKinematics_B.rEQ0 ||
      InverseKinematics_B.e_p) * 3 - 1;
    if (InverseKinematics_B.T_tmp >= 0) {
      memset(&InverseKinematics_B.vspecial_data[0], 0,
             (InverseKinematics_B.T_tmp + 1) * sizeof(real_T));
    }

    InverseKinematics_B.T_tmp = (InverseKinematics_B.rEQ0 ||
      InverseKinematics_B.e_p) - 1;
    for (InverseKinematics_B.T_tmp_p = 0; InverseKinematics_B.T_tmp_p <=
         InverseKinematics_B.T_tmp; InverseKinematics_B.T_tmp_p++) {
      memset(&InverseKinematics_B.T[0], 0, 9U * sizeof(real_T));
      InverseKinematics_B.T[0] = 1.0;
      InverseKinematics_B.T[4] = 1.0;
      InverseKinematics_B.T[8] = 1.0;
      for (InverseKinematics_B.b_k_px = 0; InverseKinematics_B.b_k_px <= 6;
           InverseKinematics_B.b_k_px += 2) {
        __m128d tmp;
        tmp_0 = _mm_loadu_pd(&InverseKinematics_B.T[InverseKinematics_B.b_k_px]);
        tmp = _mm_loadu_pd(&InverseKinematics_B.y[InverseKinematics_B.b_k_px]);
        _mm_storeu_pd(&InverseKinematics_B.T[InverseKinematics_B.b_k_px],
                      _mm_sub_pd(tmp_0, tmp));
      }

      for (InverseKinematics_B.b_k_px = 8; InverseKinematics_B.b_k_px < 9;
           InverseKinematics_B.b_k_px++) {
        InverseKinematics_B.T[InverseKinematics_B.b_k_px] -=
          InverseKinematics_B.y[InverseKinematics_B.b_k_px];
      }

      InverseKinematics_B.x_b = true;
      for (InverseKinematics_B.b_k_px = 0; InverseKinematics_B.b_k_px < 9;
           InverseKinematics_B.b_k_px++) {
        if (InverseKinematics_B.x_b) {
          InverseKinematics_B.a_m =
            InverseKinematics_B.T[InverseKinematics_B.b_k_px];
          if ((!rtIsInf(InverseKinematics_B.a_m)) && (!rtIsNaN
               (InverseKinematics_B.a_m))) {
          } else {
            InverseKinematics_B.x_b = false;
          }
        } else {
          InverseKinematics_B.x_b = false;
        }
      }

      if (InverseKinematics_B.x_b) {
        InverseKinematics_svd(InverseKinematics_B.T, InverseKinematics_B.b_U,
                              InverseKinematics_B.v_l, InverseKinematics_B.V);
      } else {
        for (InverseKinematics_B.b_k_px = 0; InverseKinematics_B.b_k_px < 9;
             InverseKinematics_B.b_k_px++) {
          InverseKinematics_B.V[InverseKinematics_B.b_k_px] = (rtNaN);
        }
      }

      InverseKinematics_B.vspecial_data[0] = InverseKinematics_B.V[6];
      InverseKinematics_B.vspecial_data[1] = InverseKinematics_B.V[7];
      InverseKinematics_B.vspecial_data[2] = InverseKinematics_B.V[8];
    }

    InverseKinematics_B.b_k_px = 0;
    if (InverseKinematics_B.rEQ0 || InverseKinematics_B.e_p) {
      for (InverseKinematics_B.T_tmp = 0; InverseKinematics_B.T_tmp < 1;
           InverseKinematics_B.T_tmp++) {
        InverseKinematics_B.b_k_px++;
      }
    }

    if (InverseKinematics_B.b_k_px - 1 >= 0) {
      InverseKinematics_B.v_g[0] = InverseKinematics_B.vspecial_data[0];
      InverseKinematics_B.v_g[1] = InverseKinematics_B.vspecial_data[1];
      InverseKinematics_B.v_g[2] = InverseKinematics_B.vspecial_data[2];
    }
  }

  InverseKinematics_B.v_l[0] = InverseKinematics_B.v_g[0];
  InverseKinematics_B.v_l[1] = InverseKinematics_B.v_g[1];
  InverseKinematics_B.v_l[2] = InverseKinematics_B.v_g[2];
  InverseKinematics_B.a_m = 1.0 / sqrt((InverseKinematics_B.v_g[0] *
    InverseKinematics_B.v_g[0] + InverseKinematics_B.v_g[1] *
    InverseKinematics_B.v_g[1]) + InverseKinematics_B.v_g[2] *
    InverseKinematics_B.v_g[2]);
  errorvec[0] = InverseKinematics_B.v_l[0] * InverseKinematics_B.a_m *
    InverseKinematics_B.v_lx.re;
  errorvec[3] = Td[12] - T_data[T_size[0] * 3];
  errorvec[1] = InverseKinematics_B.v_l[1] * InverseKinematics_B.a_m *
    InverseKinematics_B.v_lx.re;
  errorvec[4] = Td[13] - T_data[T_size[0] * 3 + 1];
  errorvec[2] = InverseKinematics_B.v_l[2] * InverseKinematics_B.a_m *
    InverseKinematics_B.v_lx.re;
  errorvec[5] = Td[14] - T_data[T_size[0] * 3 + 2];
}

void InverseKinematics::InverseKinematics_mtimes_n(const real_T A[6], const
  emxArray_real_T_InverseKinema_T *B, emxArray_real_T_InverseKinema_T *C)
{
  int32_T b_j;
  int32_T n;
  n = B->size[1] - 1;
  b_j = C->size[0] * C->size[1];
  C->size[0] = 1;
  C->size[1] = B->size[1];
  Invers_emxEnsureCapacity_real_T(C, b_j);
  for (b_j = 0; b_j <= n; b_j++) {
    real_T s;
    int32_T boffset;
    boffset = b_j * 6 - 1;
    s = 0.0;
    for (int32_T b_k = 0; b_k < 6; b_k++) {
      s += B->data[(boffset + b_k) + 1] * A[b_k];
    }

    C->data[b_j] = s;
  }
}

void InverseKinematics::InverseKinema_emxInit_boolean_T
  (emxArray_boolean_T_InverseKin_T **pEmxArray, int32_T numDimensions)
{
  emxArray_boolean_T_InverseKin_T *emxArray;
  *pEmxArray = static_cast<emxArray_boolean_T_InverseKin_T *>(malloc(sizeof
    (emxArray_boolean_T_InverseKin_T)));
  emxArray = *pEmxArray;
  emxArray->data = static_cast<boolean_T *>(NULL);
  emxArray->numDimensions = numDimensions;
  emxArray->size = static_cast<int32_T *>(malloc(sizeof(int32_T) * numDimensions));
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (int32_T i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

real_T InverseKinematics::InverseKinematics_norm_n(const real_T x[6])
{
  real_T scale;
  real_T y;
  y = 0.0;
  scale = 3.3121686421112381E-170;
  for (int32_T b_k = 0; b_k < 6; b_k++) {
    real_T absxk;
    absxk = fabs(x[b_k]);
    if (absxk > scale) {
      real_T t;
      t = scale / absxk;
      y = y * t * t + 1.0;
      scale = absxk;
    } else {
      real_T t;
      t = absxk / scale;
      y += t * t;
    }
  }

  return scale * sqrt(y);
}

void InverseKinematics::InverseKinematics_minus_n
  (emxArray_real_T_InverseKinema_T *in1, const emxArray_real_T_InverseKinema_T
   *in2)
{
  emxArray_real_T_InverseKinema_T *in2_0;
  int32_T i;
  int32_T loop_ub;
  int32_T stride_0_0;
  int32_T stride_1_0;
  InverseKinematic_emxInit_real_T(&in2_0, 1);
  i = in2_0->size[0];
  in2_0->size[0] = in1->size[0] == 1 ? in2->size[0] : in1->size[0];
  Invers_emxEnsureCapacity_real_T(in2_0, i);
  stride_0_0 = (in2->size[0] != 1);
  stride_1_0 = (in1->size[0] != 1);
  loop_ub = in1->size[0] == 1 ? in2->size[0] : in1->size[0];
  for (i = 0; i < loop_ub; i++) {
    in2_0->data[i] = in2->data[i * stride_0_0] - in1->data[i * stride_1_0];
  }

  i = in1->size[0];
  in1->size[0] = in2_0->size[0];
  Invers_emxEnsureCapacity_real_T(in1, i);
  loop_ub = in2_0->size[0];
  if (loop_ub - 1 >= 0) {
    memcpy(&in1->data[0], &in2_0->data[0], loop_ub * sizeof(real_T));
  }

  InverseKinematic_emxFree_real_T(&in2_0);
}

void InverseKinematics::Inv_emxEnsureCapacity_boolean_T
  (emxArray_boolean_T_InverseKin_T *emxArray, int32_T oldNumel)
{
  int32_T i;
  int32_T newNumel;
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  newNumel = 1;
  for (i = 0; i < emxArray->numDimensions; i++) {
    newNumel *= emxArray->size[i];
  }

  if (newNumel > emxArray->allocatedSize) {
    i = emxArray->allocatedSize;
    if (i < 16) {
      i = 16;
    }

    while (i < newNumel) {
      if (i > 1073741823) {
        i = MAX_int32_T;
      } else {
        i <<= 1;
      }
    }

    newData = calloc(static_cast<uint32_T>(i), sizeof(boolean_T));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(boolean_T) * oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }

    emxArray->data = static_cast<boolean_T *>(newData);
    emxArray->allocatedSize = i;
    emxArray->canFreeData = true;
  }
}

real_T InverseKinematics::InverseKinematics_toc(real_T tstart_tv_sec, real_T
  tstart_tv_nsec)
{
  coderTimespec b_timespec;
  if (!InverseKinematics_DW.method_not_empty) {
    InverseKinematics_DW.method_not_empty = true;
    coderInitTimeFunctions(&InverseKinematics_DW.freq);
  }

  coderTimeClockGettimeMonotonic(&b_timespec, InverseKinematics_DW.freq);
  return (b_timespec.tv_nsec - tstart_tv_nsec) / 1.0E+9 + (b_timespec.tv_sec -
    tstart_tv_sec);
}

void InverseKinematics::InverseKinematics_mldivide(const real_T A[16], const
  emxArray_real_T_InverseKinema_T *B, real_T Y_data[], int32_T *Y_size)
{
  memcpy(&InverseKinematics_B.c_x[0], &A[0], sizeof(real_T) << 4U);
  InverseKinematics_B.b_ipiv[0] = 1;
  InverseKinematics_B.b_ipiv[1] = 2;
  InverseKinematics_B.b_ipiv[2] = 3;
  for (InverseKinematics_B.kAcol = 0; InverseKinematics_B.kAcol < 3;
       InverseKinematics_B.kAcol++) {
    InverseKinematics_B.c_b = InverseKinematics_B.kAcol * 5 + 2;
    InverseKinematics_B.jj = InverseKinematics_B.kAcol * 5;
    InverseKinematics_B.c_a = 4 - InverseKinematics_B.kAcol;
    InverseKinematics_B.a_g = 1;
    InverseKinematics_B.smax = fabs
      (InverseKinematics_B.c_x[InverseKinematics_B.jj]);
    for (InverseKinematics_B.jA = 2; InverseKinematics_B.jA <=
         InverseKinematics_B.c_a; InverseKinematics_B.jA++) {
      InverseKinematics_B.s_c = fabs(InverseKinematics_B.c_x
        [(InverseKinematics_B.c_b + InverseKinematics_B.jA) - 3]);
      if (InverseKinematics_B.s_c > InverseKinematics_B.smax) {
        InverseKinematics_B.a_g = InverseKinematics_B.jA;
        InverseKinematics_B.smax = InverseKinematics_B.s_c;
      }
    }

    if (InverseKinematics_B.c_x[(InverseKinematics_B.c_b +
         InverseKinematics_B.a_g) - 3] != 0.0) {
      if (InverseKinematics_B.a_g - 1 != 0) {
        InverseKinematics_B.a_g += InverseKinematics_B.kAcol;
        InverseKinematics_B.b_ipiv[InverseKinematics_B.kAcol] =
          static_cast<int8_T>(InverseKinematics_B.a_g);
        InverseKinematics_B.smax =
          InverseKinematics_B.c_x[InverseKinematics_B.kAcol];
        InverseKinematics_B.c_x[InverseKinematics_B.kAcol] =
          InverseKinematics_B.c_x[InverseKinematics_B.a_g - 1];
        InverseKinematics_B.c_x[InverseKinematics_B.a_g - 1] =
          InverseKinematics_B.smax;
        InverseKinematics_B.smax =
          InverseKinematics_B.c_x[InverseKinematics_B.kAcol + 4];
        InverseKinematics_B.c_x[InverseKinematics_B.kAcol + 4] =
          InverseKinematics_B.c_x[InverseKinematics_B.a_g + 3];
        InverseKinematics_B.c_x[InverseKinematics_B.a_g + 3] =
          InverseKinematics_B.smax;
        InverseKinematics_B.smax =
          InverseKinematics_B.c_x[InverseKinematics_B.kAcol + 8];
        InverseKinematics_B.c_x[InverseKinematics_B.kAcol + 8] =
          InverseKinematics_B.c_x[InverseKinematics_B.a_g + 7];
        InverseKinematics_B.c_x[InverseKinematics_B.a_g + 7] =
          InverseKinematics_B.smax;
        InverseKinematics_B.smax =
          InverseKinematics_B.c_x[InverseKinematics_B.kAcol + 12];
        InverseKinematics_B.c_x[InverseKinematics_B.kAcol + 12] =
          InverseKinematics_B.c_x[InverseKinematics_B.a_g + 11];
        InverseKinematics_B.c_x[InverseKinematics_B.a_g + 11] =
          InverseKinematics_B.smax;
      }

      InverseKinematics_B.a_g = InverseKinematics_B.c_b -
        InverseKinematics_B.kAcol;
      for (InverseKinematics_B.c_a = InverseKinematics_B.c_b;
           InverseKinematics_B.c_a <= InverseKinematics_B.a_g + 2;
           InverseKinematics_B.c_a++) {
        InverseKinematics_B.c_x[InverseKinematics_B.c_a - 1] /=
          InverseKinematics_B.c_x[InverseKinematics_B.jj];
      }
    }

    InverseKinematics_B.c_a = 3 - InverseKinematics_B.kAcol;
    InverseKinematics_B.jA = InverseKinematics_B.jj;
    for (InverseKinematics_B.j_e = 0; InverseKinematics_B.j_e <
         InverseKinematics_B.c_a; InverseKinematics_B.j_e++) {
      InverseKinematics_B.smax = InverseKinematics_B.c_x
        [((InverseKinematics_B.j_e << 2) + InverseKinematics_B.jj) + 4];
      if (InverseKinematics_B.smax != 0.0) {
        InverseKinematics_B.a_g = InverseKinematics_B.jA + 6;
        InverseKinematics_B.c_fi = InverseKinematics_B.jA -
          InverseKinematics_B.kAcol;
        for (InverseKinematics_B.ijA = InverseKinematics_B.a_g;
             InverseKinematics_B.ijA <= InverseKinematics_B.c_fi + 8;
             InverseKinematics_B.ijA++) {
          InverseKinematics_B.c_x[InverseKinematics_B.ijA - 1] +=
            InverseKinematics_B.c_x[((InverseKinematics_B.c_b +
            InverseKinematics_B.ijA) - InverseKinematics_B.jA) - 7] *
            -InverseKinematics_B.smax;
        }
      }

      InverseKinematics_B.jA += 4;
    }
  }

  *Y_size = B->size[0];
  InverseKinematics_B.c_b = B->size[0];
  if (InverseKinematics_B.c_b - 1 >= 0) {
    memcpy(&Y_data[0], &B->data[0], InverseKinematics_B.c_b * sizeof(real_T));
  }

  if (InverseKinematics_B.b_ipiv[0] != 1) {
    InverseKinematics_B.smax = Y_data[0];
    Y_data[0] = Y_data[InverseKinematics_B.b_ipiv[0] - 1];
    Y_data[InverseKinematics_B.b_ipiv[0] - 1] = InverseKinematics_B.smax;
  }

  if (InverseKinematics_B.b_ipiv[1] != 2) {
    InverseKinematics_B.smax = Y_data[1];
    Y_data[1] = Y_data[InverseKinematics_B.b_ipiv[1] - 1];
    Y_data[InverseKinematics_B.b_ipiv[1] - 1] = InverseKinematics_B.smax;
  }

  if (InverseKinematics_B.b_ipiv[2] != 3) {
    InverseKinematics_B.smax = Y_data[2];
    Y_data[2] = Y_data[InverseKinematics_B.b_ipiv[2] - 1];
    Y_data[InverseKinematics_B.b_ipiv[2] - 1] = InverseKinematics_B.smax;
  }

  for (InverseKinematics_B.c_b = 0; InverseKinematics_B.c_b < 4;
       InverseKinematics_B.c_b++) {
    InverseKinematics_B.kAcol = (InverseKinematics_B.c_b << 2) - 1;
    if (Y_data[InverseKinematics_B.c_b] != 0.0) {
      for (InverseKinematics_B.c_a = InverseKinematics_B.c_b + 2;
           InverseKinematics_B.c_a < 5; InverseKinematics_B.c_a++) {
        Y_data[InverseKinematics_B.c_a - 1] -=
          InverseKinematics_B.c_x[InverseKinematics_B.c_a +
          InverseKinematics_B.kAcol] * Y_data[InverseKinematics_B.c_b];
      }
    }
  }

  for (InverseKinematics_B.jA = 3; InverseKinematics_B.jA >= 0;
       InverseKinematics_B.jA--) {
    InverseKinematics_B.kAcol = InverseKinematics_B.jA << 2;
    if (Y_data[InverseKinematics_B.jA] != 0.0) {
      Y_data[InverseKinematics_B.jA] /=
        InverseKinematics_B.c_x[InverseKinematics_B.jA +
        InverseKinematics_B.kAcol];
      InverseKinematics_B.a_g = InverseKinematics_B.jA - 1;
      for (InverseKinematics_B.c_b = 0; InverseKinematics_B.c_b <=
           InverseKinematics_B.a_g; InverseKinematics_B.c_b++) {
        Y_data[InverseKinematics_B.c_b] -=
          InverseKinematics_B.c_x[InverseKinematics_B.c_b +
          InverseKinematics_B.kAcol] * Y_data[InverseKinematics_B.jA];
      }
    }
  }
}

void InverseKinematics::InverseKinem_binary_expand_op_n(real_T in1_data[],
  int32_T *in1_size, const emxArray_real_T_InverseKinema_T *in2, real_T in3,
  const real_T in4[16], const emxArray_real_T_InverseKinema_T *in5)
{
  InverseKinematics_B.stride_0_0 = (in2->size[0] != 1);
  InverseKinematics_B.stride_0_1 = (in2->size[1] != 1);
  InverseKinematics_B.aux_0_1 = 0;
  for (InverseKinematics_B.i4 = 0; InverseKinematics_B.i4 < 4;
       InverseKinematics_B.i4++) {
    InverseKinematics_B.in2_tmp = InverseKinematics_B.i4 << 2;
    InverseKinematics_B.in2[InverseKinematics_B.in2_tmp] =
      -(in4[InverseKinematics_B.in2_tmp] * in3 + in2->data[in2->size[0] *
        InverseKinematics_B.aux_0_1]);
    InverseKinematics_B.in2[InverseKinematics_B.in2_tmp + 1] =
      -(in4[InverseKinematics_B.in2_tmp + 1] * in3 + in2->data[in2->size[0] *
        InverseKinematics_B.aux_0_1 + InverseKinematics_B.stride_0_0]);
    InverseKinematics_B.in2[InverseKinematics_B.in2_tmp + 2] =
      -(in4[InverseKinematics_B.in2_tmp + 2] * in3 + in2->data
        [(InverseKinematics_B.stride_0_0 << 1) + in2->size[0] *
        InverseKinematics_B.aux_0_1]);
    InverseKinematics_B.in2[InverseKinematics_B.in2_tmp + 3] =
      -(in4[InverseKinematics_B.in2_tmp + 3] * in3 + in2->data[3 *
        InverseKinematics_B.stride_0_0 + in2->size[0] *
        InverseKinematics_B.aux_0_1]);
    InverseKinematics_B.aux_0_1 += InverseKinematics_B.stride_0_1;
  }

  InverseKinematics_mldivide(InverseKinematics_B.in2, in5, in1_data, in1_size);
}

void InverseKinematics::InverseKinematics_expand_max(const
  emxArray_real_T_InverseKinema_T *a, const real_T b[4], real_T c[4])
{
  real_T u0;
  int32_T acoef;
  acoef = (a->size[0] != 1);
  if ((a->data[0] >= b[0]) || rtIsNaN(b[0])) {
    c[0] = a->data[0];
  } else {
    c[0] = b[0];
  }

  u0 = a->data[acoef];
  if ((u0 >= b[1]) || rtIsNaN(b[1])) {
    c[1] = u0;
  } else {
    c[1] = b[1];
  }

  u0 = a->data[acoef << 1];
  if ((u0 >= b[2]) || rtIsNaN(b[2])) {
    c[2] = u0;
  } else {
    c[2] = b[2];
  }

  u0 = a->data[acoef * 3];
  if ((u0 >= b[3]) || rtIsNaN(b[3])) {
    c[3] = u0;
  } else {
    c[3] = b[3];
  }
}

void InverseKinematics::InverseKinematics_expand_min(const
  emxArray_real_T_InverseKinema_T *a, const real_T b[4], real_T c[4])
{
  real_T u0;
  int32_T acoef;
  acoef = (a->size[0] != 1);
  if ((a->data[0] <= b[0]) || rtIsNaN(b[0])) {
    c[0] = a->data[0];
  } else {
    c[0] = b[0];
  }

  u0 = a->data[acoef];
  if ((u0 <= b[1]) || rtIsNaN(b[1])) {
    c[1] = u0;
  } else {
    c[1] = b[1];
  }

  u0 = a->data[acoef << 1];
  if ((u0 <= b[2]) || rtIsNaN(b[2])) {
    c[2] = u0;
  } else {
    c[2] = b[2];
  }

  u0 = a->data[acoef * 3];
  if ((u0 <= b[3]) || rtIsNaN(b[3])) {
    c[3] = u0;
  } else {
    c[3] = b[3];
  }
}

void InverseKinematics::InverseKinema_emxFree_boolean_T
  (emxArray_boolean_T_InverseKin_T **pEmxArray)
{
  if (*pEmxArray != static_cast<emxArray_boolean_T_InverseKin_T *>(NULL)) {
    if (((*pEmxArray)->data != static_cast<boolean_T *>(NULL)) && (*pEmxArray)
        ->canFreeData) {
      free((*pEmxArray)->data);
    }

    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = static_cast<emxArray_boolean_T_InverseKin_T *>(NULL);
  }
}

void InverseKinematics::ErrorDampedLevenbergMarquardt_s
  (h_robotics_core_internal_Erro_T *obj, real_T xSol[4],
   c_robotics_core_internal_NLPS_T *exitFlag, real_T *en, real_T *iter)
{
  emxArray_boolean_T_InverseKin_T *x;
  emxArray_char_T_InverseKinema_T *bodyName;
  emxArray_real_T_InverseKinema_T *H0;
  emxArray_real_T_InverseKinema_T *J;
  emxArray_real_T_InverseKinema_T *b;
  emxArray_real_T_InverseKinema_T *ev;
  emxArray_real_T_InverseKinema_T *evprev;
  emxArray_real_T_InverseKinema_T *grad;
  emxArray_real_T_InverseKinema_T *tmp;
  emxArray_real_T_InverseKinema_T *y;
  f_robotics_manip_internal_IKE_T *args;
  v_robotics_manip_internal_Rig_T *treeInternal;
  static const real_T tmp_0[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  __m128d tmp_2;
  emxArray_real_T_InverseKinema_T *J_0;
  InverseKinematic_emxInit_char_T(&bodyName, 2);
  xSol[0] = obj->SeedInternal[0];
  xSol[1] = obj->SeedInternal[1];
  xSol[2] = obj->SeedInternal[2];
  xSol[3] = obj->SeedInternal[3];
  InverseKinematics_tic(&obj->TimeObjInternal.StartTime.tv_sec,
                        &obj->TimeObjInternal.StartTime.tv_nsec);
  InverseKinematics_B.xprev_idx_0 = xSol[0];
  InverseKinematics_B.xprev_idx_1 = xSol[1];
  InverseKinematics_B.xprev_idx_2 = xSol[2];
  InverseKinematics_B.xprev_idx_3 = xSol[3];
  args = obj->ExtraArgs;
  treeInternal = args->Robot;
  InverseKinematics_B.J = bodyName->size[0] * bodyName->size[1];
  bodyName->size[0] = 1;
  bodyName->size[1] = args->BodyName->size[1];
  Invers_emxEnsureCapacity_char_T(bodyName, InverseKinematics_B.J);
  InverseKinematics_B.kend = args->BodyName->size[1] - 1;
  for (InverseKinematics_B.b_k_i = 0; InverseKinematics_B.b_k_i <=
       InverseKinematics_B.kend; InverseKinematics_B.b_k_i++) {
    InverseKinematics_B.J = InverseKinematics_B.b_k_i;
    bodyName->data[InverseKinematics_B.J] = args->BodyName->
      data[InverseKinematics_B.J];
  }

  for (InverseKinematics_B.b_k_i = 0; InverseKinematics_B.b_k_i < 16;
       InverseKinematics_B.b_k_i++) {
    InverseKinematics_B.Td[InverseKinematics_B.b_k_i] = args->
      Tform[InverseKinematics_B.b_k_i];
  }

  for (InverseKinematics_B.b_k_i = 0; InverseKinematics_B.b_k_i < 36;
       InverseKinematics_B.b_k_i++) {
    InverseKinematics_B.weightMatrix_m[InverseKinematics_B.b_k_i] =
      args->WeightMatrix[InverseKinematics_B.b_k_i];
  }

  InverseKinematic_emxInit_real_T(&J, 2);
  RigidBodyTree_efficientFKAndJac(treeInternal, xSol, bodyName,
    InverseKinematics_B.T_data, InverseKinematics_B.T_size, J);
  InverseKine_IKHelpers_poseError(InverseKinematics_B.Td,
    InverseKinematics_B.T_data, InverseKinematics_B.T_size,
    InverseKinematics_B.e);
  InverseKinematics_B.J = args->ErrTemp->size[0];
  args->ErrTemp->size[0] = 6;
  Invers_emxEnsureCapacity_real_T(args->ErrTemp, InverseKinematics_B.J);
  for (InverseKinematics_B.b_k_i = 0; InverseKinematics_B.b_k_i < 6;
       InverseKinematics_B.b_k_i++) {
    args->ErrTemp->data[InverseKinematics_B.b_k_i] =
      InverseKinematics_B.e[InverseKinematics_B.b_k_i];
  }

  InverseKinematics_B.absxk = 0.0;
  for (InverseKinematics_B.b_k_i = 0; InverseKinematics_B.b_k_i < 6;
       InverseKinematics_B.b_k_i++) {
    InverseKinematics_B.e_m[InverseKinematics_B.b_k_i] = 0.0;
    for (InverseKinematics_B.kend = 0; InverseKinematics_B.kend < 6;
         InverseKinematics_B.kend++) {
      InverseKinematics_B.e_m[InverseKinematics_B.b_k_i] +=
        InverseKinematics_B.weightMatrix_m[6 * InverseKinematics_B.b_k_i +
        InverseKinematics_B.kend] * (0.5 *
        InverseKinematics_B.e[InverseKinematics_B.kend]);
    }

    InverseKinematics_B.absxk +=
      InverseKinematics_B.e_m[InverseKinematics_B.b_k_i] *
      InverseKinematics_B.e[InverseKinematics_B.b_k_i];
  }

  args->CostTemp = InverseKinematics_B.absxk;
  for (InverseKinematics_B.b_k_i = 0; InverseKinematics_B.b_k_i < 6;
       InverseKinematics_B.b_k_i++) {
    InverseKinematics_B.e_m[InverseKinematics_B.b_k_i] = 0.0;
    for (InverseKinematics_B.kend = 0; InverseKinematics_B.kend < 6;
         InverseKinematics_B.kend++) {
      InverseKinematics_B.e_m[InverseKinematics_B.b_k_i] +=
        InverseKinematics_B.weightMatrix_m[6 * InverseKinematics_B.b_k_i +
        InverseKinematics_B.kend] *
        InverseKinematics_B.e[InverseKinematics_B.kend];
    }
  }

  InverseKinematic_emxInit_real_T(&J_0, 2);
  InverseKinematics_B.J = J_0->size[0] * J_0->size[1];
  J_0->size[0] = 6;
  J_0->size[1] = J->size[1];
  Invers_emxEnsureCapacity_real_T(J_0, InverseKinematics_B.J);
  InverseKinematics_B.kend = 6 * J->size[1];
  InverseKinematics_B.m = (InverseKinematics_B.kend / 2) << 1;
  InverseKinematics_B.coffset = InverseKinematics_B.m - 2;
  for (InverseKinematics_B.b_k_i = 0; InverseKinematics_B.b_k_i <=
       InverseKinematics_B.coffset; InverseKinematics_B.b_k_i += 2) {
    tmp_2 = _mm_loadu_pd(&J->data[InverseKinematics_B.b_k_i]);
    _mm_storeu_pd(&J_0->data[InverseKinematics_B.b_k_i], _mm_mul_pd(tmp_2,
      _mm_set1_pd(-1.0)));
  }

  for (InverseKinematics_B.b_k_i = InverseKinematics_B.m;
       InverseKinematics_B.b_k_i < InverseKinematics_B.kend;
       InverseKinematics_B.b_k_i++) {
    J_0->data[InverseKinematics_B.b_k_i] = -J->data[InverseKinematics_B.b_k_i];
  }

  InverseKinematic_emxInit_real_T(&tmp, 2);
  InverseKinematics_mtimes_n(InverseKinematics_B.e_m, J_0, tmp);
  InverseKinematics_B.J = args->GradTemp->size[0];
  args->GradTemp->size[0] = tmp->size[1];
  Invers_emxEnsureCapacity_real_T(args->GradTemp, InverseKinematics_B.J);
  InverseKinematics_B.kend = tmp->size[1];
  for (InverseKinematics_B.b_k_i = 0; InverseKinematics_B.b_k_i <
       InverseKinematics_B.kend; InverseKinematics_B.b_k_i++) {
    args->GradTemp->data[InverseKinematics_B.b_k_i] = tmp->
      data[InverseKinematics_B.b_k_i];
  }

  InverseKinematic_emxInit_real_T(&evprev, 1);
  obj->ExtraArgs = args;
  args = obj->ExtraArgs;
  InverseKinematics_B.J = evprev->size[0];
  evprev->size[0] = args->ErrTemp->size[0];
  Invers_emxEnsureCapacity_real_T(evprev, InverseKinematics_B.J);
  InverseKinematics_B.kend = args->ErrTemp->size[0];
  for (InverseKinematics_B.b_k_i = 0; InverseKinematics_B.b_k_i <
       InverseKinematics_B.kend; InverseKinematics_B.b_k_i++) {
    evprev->data[InverseKinematics_B.b_k_i] = args->ErrTemp->
      data[InverseKinematics_B.b_k_i];
  }

  InverseKinematics_B.d = obj->MaxNumIterationInternal;
  InverseKinematics_B.b_i_f = 0;
  InverseKinematic_emxInit_real_T(&grad, 1);
  InverseKinematic_emxInit_real_T(&H0, 2);
  InverseKinematic_emxInit_real_T(&ev, 1);
  InverseKinematic_emxInit_real_T(&y, 2);
  InverseKinematic_emxInit_real_T(&b, 1);
  InverseKinema_emxInit_boolean_T(&x, 1);
  int32_T exitg1;
  do {
    exitg1 = 0;
    if (InverseKinematics_B.b_i_f <= static_cast<int32_T>(InverseKinematics_B.d)
        - 1) {
      args = obj->ExtraArgs;
      treeInternal = args->Robot;
      InverseKinematics_B.J = bodyName->size[0] * bodyName->size[1];
      bodyName->size[0] = 1;
      bodyName->size[1] = args->BodyName->size[1];
      Invers_emxEnsureCapacity_char_T(bodyName, InverseKinematics_B.J);
      InverseKinematics_B.kend = args->BodyName->size[1] - 1;
      for (InverseKinematics_B.b_k_i = 0; InverseKinematics_B.b_k_i <=
           InverseKinematics_B.kend; InverseKinematics_B.b_k_i++) {
        InverseKinematics_B.J = InverseKinematics_B.b_k_i;
        bodyName->data[InverseKinematics_B.J] = args->BodyName->
          data[InverseKinematics_B.J];
      }

      for (InverseKinematics_B.b_k_i = 0; InverseKinematics_B.b_k_i < 16;
           InverseKinematics_B.b_k_i++) {
        InverseKinematics_B.Td[InverseKinematics_B.b_k_i] = args->
          Tform[InverseKinematics_B.b_k_i];
      }

      for (InverseKinematics_B.b_k_i = 0; InverseKinematics_B.b_k_i < 36;
           InverseKinematics_B.b_k_i++) {
        InverseKinematics_B.weightMatrix_m[InverseKinematics_B.b_k_i] =
          args->WeightMatrix[InverseKinematics_B.b_k_i];
      }

      RigidBodyTree_efficientFKAndJac(treeInternal, xSol, bodyName,
        InverseKinematics_B.T_data, InverseKinematics_B.T_size, J);
      InverseKinematics_B.J = J_0->size[0] * J_0->size[1];
      J_0->size[0] = 6;
      J_0->size[1] = J->size[1];
      Invers_emxEnsureCapacity_real_T(J_0, InverseKinematics_B.J);
      InverseKinematics_B.kend = 6 * J->size[1];
      InverseKinematics_B.m = (InverseKinematics_B.kend / 2) << 1;
      InverseKinematics_B.coffset = InverseKinematics_B.m - 2;
      for (InverseKinematics_B.b_k_i = 0; InverseKinematics_B.b_k_i <=
           InverseKinematics_B.coffset; InverseKinematics_B.b_k_i += 2) {
        tmp_2 = _mm_loadu_pd(&J->data[InverseKinematics_B.b_k_i]);
        _mm_storeu_pd(&J_0->data[InverseKinematics_B.b_k_i], _mm_mul_pd(tmp_2,
          _mm_set1_pd(-1.0)));
      }

      for (InverseKinematics_B.b_k_i = InverseKinematics_B.m;
           InverseKinematics_B.b_k_i < InverseKinematics_B.kend;
           InverseKinematics_B.b_k_i++) {
        J_0->data[InverseKinematics_B.b_k_i] = -J->
          data[InverseKinematics_B.b_k_i];
      }

      InverseKine_IKHelpers_poseError(InverseKinematics_B.Td,
        InverseKinematics_B.T_data, InverseKinematics_B.T_size,
        InverseKinematics_B.e);
      InverseKinematics_B.J = args->ErrTemp->size[0];
      args->ErrTemp->size[0] = 6;
      Invers_emxEnsureCapacity_real_T(args->ErrTemp, InverseKinematics_B.J);
      for (InverseKinematics_B.b_k_i = 0; InverseKinematics_B.b_k_i < 6;
           InverseKinematics_B.b_k_i++) {
        args->ErrTemp->data[InverseKinematics_B.b_k_i] =
          InverseKinematics_B.e[InverseKinematics_B.b_k_i];
      }

      InverseKinematics_B.absxk = 0.0;
      for (InverseKinematics_B.b_k_i = 0; InverseKinematics_B.b_k_i < 6;
           InverseKinematics_B.b_k_i++) {
        InverseKinematics_B.e_m[InverseKinematics_B.b_k_i] = 0.0;
        for (InverseKinematics_B.kend = 0; InverseKinematics_B.kend < 6;
             InverseKinematics_B.kend++) {
          InverseKinematics_B.e_m[InverseKinematics_B.b_k_i] +=
            InverseKinematics_B.weightMatrix_m[6 * InverseKinematics_B.b_k_i +
            InverseKinematics_B.kend] * (0.5 *
            InverseKinematics_B.e[InverseKinematics_B.kend]);
        }

        InverseKinematics_B.absxk +=
          InverseKinematics_B.e_m[InverseKinematics_B.b_k_i] *
          InverseKinematics_B.e[InverseKinematics_B.b_k_i];
      }

      args->CostTemp = InverseKinematics_B.absxk;
      for (InverseKinematics_B.b_k_i = 0; InverseKinematics_B.b_k_i < 6;
           InverseKinematics_B.b_k_i++) {
        InverseKinematics_B.e_m[InverseKinematics_B.b_k_i] = 0.0;
        for (InverseKinematics_B.kend = 0; InverseKinematics_B.kend < 6;
             InverseKinematics_B.kend++) {
          InverseKinematics_B.e_m[InverseKinematics_B.b_k_i] +=
            InverseKinematics_B.weightMatrix_m[6 * InverseKinematics_B.b_k_i +
            InverseKinematics_B.kend] *
            InverseKinematics_B.e[InverseKinematics_B.kend];
        }
      }

      InverseKinematics_mtimes_n(InverseKinematics_B.e_m, J_0, tmp);
      InverseKinematics_B.J = args->GradTemp->size[0];
      args->GradTemp->size[0] = tmp->size[1];
      Invers_emxEnsureCapacity_real_T(args->GradTemp, InverseKinematics_B.J);
      InverseKinematics_B.kend = tmp->size[1];
      for (InverseKinematics_B.b_k_i = 0; InverseKinematics_B.b_k_i <
           InverseKinematics_B.kend; InverseKinematics_B.b_k_i++) {
        args->GradTemp->data[InverseKinematics_B.b_k_i] = tmp->
          data[InverseKinematics_B.b_k_i];
      }

      InverseKinematics_B.cost = args->CostTemp;
      obj->ExtraArgs = args;
      args = obj->ExtraArgs;
      InverseKinematics_B.J = grad->size[0];
      grad->size[0] = args->GradTemp->size[0];
      Invers_emxEnsureCapacity_real_T(grad, InverseKinematics_B.J);
      InverseKinematics_B.kend = args->GradTemp->size[0];
      for (InverseKinematics_B.b_k_i = 0; InverseKinematics_B.b_k_i <
           InverseKinematics_B.kend; InverseKinematics_B.b_k_i++) {
        grad->data[InverseKinematics_B.b_k_i] = args->GradTemp->
          data[InverseKinematics_B.b_k_i];
      }

      args = obj->ExtraArgs;
      for (InverseKinematics_B.b_k_i = 0; InverseKinematics_B.b_k_i < 36;
           InverseKinematics_B.b_k_i++) {
        InverseKinematics_B.a[InverseKinematics_B.b_k_i] = args->
          WeightMatrix[InverseKinematics_B.b_k_i];
      }

      InverseKinematics_B.J = b->size[0];
      b->size[0] = args->ErrTemp->size[0];
      Invers_emxEnsureCapacity_real_T(b, InverseKinematics_B.J);
      InverseKinematics_B.kend = args->ErrTemp->size[0];
      for (InverseKinematics_B.b_k_i = 0; InverseKinematics_B.b_k_i <
           InverseKinematics_B.kend; InverseKinematics_B.b_k_i++) {
        b->data[InverseKinematics_B.b_k_i] = args->ErrTemp->
          data[InverseKinematics_B.b_k_i];
      }

      InverseKinematics_B.J = ev->size[0];
      ev->size[0] = args->ErrTemp->size[0];
      Invers_emxEnsureCapacity_real_T(ev, InverseKinematics_B.J);
      InverseKinematics_B.kend = args->ErrTemp->size[0];
      for (InverseKinematics_B.b_k_i = 0; InverseKinematics_B.b_k_i <
           InverseKinematics_B.kend; InverseKinematics_B.b_k_i++) {
        ev->data[InverseKinematics_B.b_k_i] = args->ErrTemp->
          data[InverseKinematics_B.b_k_i];
      }

      for (InverseKinematics_B.b_k_i = 0; InverseKinematics_B.b_k_i < 6;
           InverseKinematics_B.b_k_i++) {
        InverseKinematics_B.e[InverseKinematics_B.b_k_i] = 0.0;
        for (InverseKinematics_B.kend = 0; InverseKinematics_B.kend < 6;
             InverseKinematics_B.kend++) {
          InverseKinematics_B.e[InverseKinematics_B.b_k_i] +=
            InverseKinematics_B.a[6 * InverseKinematics_B.kend +
            InverseKinematics_B.b_k_i] * b->data[InverseKinematics_B.kend];
        }
      }

      *en = InverseKinematics_norm_n(InverseKinematics_B.e);
      *iter = static_cast<real_T>(InverseKinematics_B.b_i_f) + 1.0;
      if (grad->size[0] == 0) {
        InverseKinematics_B.cc = 0.0;
      } else {
        InverseKinematics_B.cc = 0.0;
        if (grad->size[0] == 1) {
          InverseKinematics_B.cc = fabs(grad->data[0]);
        } else {
          InverseKinematics_B.scale = 3.3121686421112381E-170;
          InverseKinematics_B.kend = grad->size[0] - 1;
          for (InverseKinematics_B.b_k_i = 0; InverseKinematics_B.b_k_i <=
               InverseKinematics_B.kend; InverseKinematics_B.b_k_i++) {
            InverseKinematics_B.absxk = fabs(grad->
              data[InverseKinematics_B.b_k_i]);
            if (InverseKinematics_B.absxk > InverseKinematics_B.scale) {
              InverseKinematics_B.t = InverseKinematics_B.scale /
                InverseKinematics_B.absxk;
              InverseKinematics_B.cc = InverseKinematics_B.cc *
                InverseKinematics_B.t * InverseKinematics_B.t + 1.0;
              InverseKinematics_B.scale = InverseKinematics_B.absxk;
            } else {
              InverseKinematics_B.t = InverseKinematics_B.absxk /
                InverseKinematics_B.scale;
              InverseKinematics_B.cc += InverseKinematics_B.t *
                InverseKinematics_B.t;
            }
          }

          InverseKinematics_B.cc = InverseKinematics_B.scale * sqrt
            (InverseKinematics_B.cc);
        }
      }

      InverseKinematics_B.flag = (InverseKinematics_B.cc <
        obj->GradientTolerance);
      if (InverseKinematics_B.flag) {
        *exitFlag = LocalMinimumFound;
        exitg1 = 1;
      } else {
        boolean_T exitg2;
        boolean_T guard1 = false;
        boolean_T guard2 = false;
        guard1 = false;
        guard2 = false;
        if (static_cast<real_T>(InverseKinematics_B.b_i_f) + 1.0 > 1.0) {
          InverseKinematics_B.x_a[0] = (fabs(xSol[0] -
            InverseKinematics_B.xprev_idx_0) < obj->StepTolerance);
          InverseKinematics_B.x_a[1] = (fabs(xSol[1] -
            InverseKinematics_B.xprev_idx_1) < obj->StepTolerance);
          InverseKinematics_B.x_a[2] = (fabs(xSol[2] -
            InverseKinematics_B.xprev_idx_2) < obj->StepTolerance);
          InverseKinematics_B.x_a[3] = (fabs(xSol[3] -
            InverseKinematics_B.xprev_idx_3) < obj->StepTolerance);
          InverseKinematics_B.flag = true;
          InverseKinematics_B.b_k_i = 0;
          exitg2 = false;
          while ((!exitg2) && (InverseKinematics_B.b_k_i < 4)) {
            if (!InverseKinematics_B.x_a[InverseKinematics_B.b_k_i]) {
              InverseKinematics_B.flag = false;
              exitg2 = true;
            } else {
              InverseKinematics_B.b_k_i++;
            }
          }

          if (InverseKinematics_B.flag) {
            *exitFlag = StepSizeBelowMinimum;
            exitg1 = 1;
          } else {
            guard2 = true;
          }
        } else {
          guard2 = true;
        }

        if (guard2) {
          if (static_cast<real_T>(InverseKinematics_B.b_i_f) + 1.0 > 1.0) {
            if (ev->size[0] == evprev->size[0]) {
              InverseKinematics_B.J = evprev->size[0];
              evprev->size[0] = ev->size[0];
              Invers_emxEnsureCapacity_real_T(evprev, InverseKinematics_B.J);
              InverseKinematics_B.kend = ev->size[0];
              InverseKinematics_B.m = (ev->size[0] / 2) << 1;
              InverseKinematics_B.coffset = InverseKinematics_B.m - 2;
              for (InverseKinematics_B.b_k_i = 0; InverseKinematics_B.b_k_i <=
                   InverseKinematics_B.coffset; InverseKinematics_B.b_k_i += 2)
              {
                __m128d tmp_1;
                tmp_2 = _mm_loadu_pd(&ev->data[InverseKinematics_B.b_k_i]);
                tmp_1 = _mm_loadu_pd(&evprev->data[InverseKinematics_B.b_k_i]);
                _mm_storeu_pd(&evprev->data[InverseKinematics_B.b_k_i],
                              _mm_sub_pd(tmp_2, tmp_1));
              }

              for (InverseKinematics_B.b_k_i = InverseKinematics_B.m;
                   InverseKinematics_B.b_k_i < InverseKinematics_B.kend;
                   InverseKinematics_B.b_k_i++) {
                evprev->data[InverseKinematics_B.b_k_i] = ev->
                  data[InverseKinematics_B.b_k_i] - evprev->
                  data[InverseKinematics_B.b_k_i];
              }
            } else {
              InverseKinematics_minus_n(evprev, ev);
            }

            InverseKinematics_B.kend = evprev->size[0] - 1;
            InverseKinematics_B.J = b->size[0];
            b->size[0] = evprev->size[0];
            Invers_emxEnsureCapacity_real_T(b, InverseKinematics_B.J);
            for (InverseKinematics_B.b_k_i = 0; InverseKinematics_B.b_k_i <=
                 InverseKinematics_B.kend; InverseKinematics_B.b_k_i++) {
              b->data[InverseKinematics_B.b_k_i] = fabs(evprev->
                data[InverseKinematics_B.b_k_i]);
            }

            InverseKinematics_B.J = x->size[0];
            x->size[0] = b->size[0];
            Inv_emxEnsureCapacity_boolean_T(x, InverseKinematics_B.J);
            InverseKinematics_B.kend = b->size[0];
            for (InverseKinematics_B.b_k_i = 0; InverseKinematics_B.b_k_i <
                 InverseKinematics_B.kend; InverseKinematics_B.b_k_i++) {
              x->data[InverseKinematics_B.b_k_i] = (b->
                data[InverseKinematics_B.b_k_i] < obj->ErrorChangeTolerance);
            }

            InverseKinematics_B.flag = true;
            InverseKinematics_B.b_k_i = 0;
            exitg2 = false;
            while ((!exitg2) && (InverseKinematics_B.b_k_i + 1 <= x->size[0])) {
              if (!x->data[InverseKinematics_B.b_k_i]) {
                InverseKinematics_B.flag = false;
                exitg2 = true;
              } else {
                InverseKinematics_B.b_k_i++;
              }
            }

            if (InverseKinematics_B.flag) {
              *exitFlag = ChangeInErrorBelowMinimum;
              exitg1 = 1;
            } else {
              guard1 = true;
            }
          } else {
            guard1 = true;
          }
        }

        if (guard1) {
          InverseKinematics_B.xprev_idx_0 = InverseKinematics_toc
            (obj->TimeObjInternal.StartTime.tv_sec,
             obj->TimeObjInternal.StartTime.tv_nsec);
          InverseKinematics_B.flag = (InverseKinematics_B.xprev_idx_0 >
            obj->MaxTimeInternal);
          if (InverseKinematics_B.flag) {
            *exitFlag = TimeLimitExceeded;
            exitg1 = 1;
          } else {
            InverseKinematics_B.J = evprev->size[0];
            evprev->size[0] = ev->size[0];
            Invers_emxEnsureCapacity_real_T(evprev, InverseKinematics_B.J);
            InverseKinematics_B.kend = ev->size[0];
            if (InverseKinematics_B.kend - 1 >= 0) {
              memcpy(&evprev->data[0], &ev->data[0], InverseKinematics_B.kend *
                     sizeof(real_T));
            }

            InverseKinematics_B.xprev_idx_0 = xSol[0];
            InverseKinematics_B.xprev_idx_1 = xSol[1];
            InverseKinematics_B.xprev_idx_2 = xSol[2];
            InverseKinematics_B.xprev_idx_3 = xSol[3];
            InverseKinematics_B.flag = obj->UseErrorDamping;
            InverseKinematics_B.cc = static_cast<real_T>
              (InverseKinematics_B.flag) * InverseKinematics_B.cost;
            InverseKinematics_B.absxk = InverseKinematics_B.cc +
              obj->DampingBias;
            InverseKinematics_B.m = J_0->size[1];
            InverseKinematics_B.J = y->size[0] * y->size[1];
            y->size[0] = J_0->size[1];
            y->size[1] = 6;
            Invers_emxEnsureCapacity_real_T(y, InverseKinematics_B.J);
            for (InverseKinematics_B.kend = 0; InverseKinematics_B.kend < 6;
                 InverseKinematics_B.kend++) {
              InverseKinematics_B.coffset = InverseKinematics_B.kend *
                InverseKinematics_B.m - 1;
              InverseKinematics_B.boffset = InverseKinematics_B.kend * 6 - 1;
              for (InverseKinematics_B.J = 0; InverseKinematics_B.J <
                   InverseKinematics_B.m; InverseKinematics_B.J++) {
                InverseKinematics_B.aoffset = InverseKinematics_B.J * 6 - 1;
                InverseKinematics_B.scale = 0.0;
                for (InverseKinematics_B.b_k_i = 0; InverseKinematics_B.b_k_i <
                     6; InverseKinematics_B.b_k_i++) {
                  InverseKinematics_B.scale += J_0->data
                    [(InverseKinematics_B.b_k_i + InverseKinematics_B.aoffset) +
                    1] * InverseKinematics_B.weightMatrix_m
                    [(InverseKinematics_B.b_k_i + InverseKinematics_B.boffset) +
                    1];
                }

                y->data[(InverseKinematics_B.coffset + InverseKinematics_B.J) +
                  1] = InverseKinematics_B.scale;
              }
            }

            InverseKinematics_B.m = y->size[0];
            InverseKinematics_B.aoffset = J_0->size[1] - 1;
            InverseKinematics_B.J = H0->size[0] * H0->size[1];
            H0->size[0] = y->size[0];
            H0->size[1] = J_0->size[1];
            Invers_emxEnsureCapacity_real_T(H0, InverseKinematics_B.J);
            for (InverseKinematics_B.kend = 0; InverseKinematics_B.kend <=
                 InverseKinematics_B.aoffset; InverseKinematics_B.kend++) {
              InverseKinematics_B.coffset = InverseKinematics_B.kend *
                InverseKinematics_B.m - 1;
              InverseKinematics_B.boffset = InverseKinematics_B.kend * 6 - 1;
              for (InverseKinematics_B.J = 0; InverseKinematics_B.J <
                   InverseKinematics_B.m; InverseKinematics_B.J++) {
                InverseKinematics_B.scale = 0.0;
                for (InverseKinematics_B.b_k_i = 0; InverseKinematics_B.b_k_i <
                     6; InverseKinematics_B.b_k_i++) {
                  InverseKinematics_B.scale += y->data[InverseKinematics_B.b_k_i
                    * y->size[0] + InverseKinematics_B.J] * J_0->data
                    [(InverseKinematics_B.boffset + InverseKinematics_B.b_k_i) +
                    1];
                }

                H0->data[(InverseKinematics_B.coffset + InverseKinematics_B.J) +
                  1] = InverseKinematics_B.scale;
              }
            }

            if ((H0->size[0] == 4) && (H0->size[1] == 4)) {
              for (InverseKinematics_B.b_k_i = 0; InverseKinematics_B.b_k_i < 4;
                   InverseKinematics_B.b_k_i++) {
                for (InverseKinematics_B.kend = 0; InverseKinematics_B.kend <= 2;
                     InverseKinematics_B.kend += 2) {
                  InverseKinematics_B.J = (InverseKinematics_B.b_k_i << 2) +
                    InverseKinematics_B.kend;
                  tmp_2 = _mm_loadu_pd(&H0->data[H0->size[0] *
                                       InverseKinematics_B.b_k_i +
                                       InverseKinematics_B.kend]);
                  _mm_storeu_pd(&InverseKinematics_B.Td[InverseKinematics_B.J],
                                _mm_mul_pd(_mm_add_pd(_mm_mul_pd(_mm_loadu_pd
                    (&tmp_0[InverseKinematics_B.J]), _mm_set1_pd
                    (InverseKinematics_B.absxk)), tmp_2), _mm_set1_pd(-1.0)));
                }
              }

              InverseKinematics_mldivide(InverseKinematics_B.Td, grad,
                InverseKinematics_B.step_data, &InverseKinematics_B.step_size);
            } else {
              InverseKinem_binary_expand_op_n(InverseKinematics_B.step_data,
                &InverseKinematics_B.step_size, H0, InverseKinematics_B.absxk,
                tmp_0, grad);
            }

            args = obj->ExtraArgs;
            treeInternal = args->Robot;
            InverseKinematics_B.J = bodyName->size[0] * bodyName->size[1];
            bodyName->size[0] = 1;
            bodyName->size[1] = args->BodyName->size[1];
            Invers_emxEnsureCapacity_char_T(bodyName, InverseKinematics_B.J);
            InverseKinematics_B.kend = args->BodyName->size[1] - 1;
            for (InverseKinematics_B.b_k_i = 0; InverseKinematics_B.b_k_i <=
                 InverseKinematics_B.kend; InverseKinematics_B.b_k_i++) {
              InverseKinematics_B.J = InverseKinematics_B.b_k_i;
              bodyName->data[InverseKinematics_B.J] = args->BodyName->
                data[InverseKinematics_B.J];
            }

            for (InverseKinematics_B.b_k_i = 0; InverseKinematics_B.b_k_i < 16;
                 InverseKinematics_B.b_k_i++) {
              InverseKinematics_B.Td[InverseKinematics_B.b_k_i] = args->
                Tform[InverseKinematics_B.b_k_i];
            }

            for (InverseKinematics_B.b_k_i = 0; InverseKinematics_B.b_k_i < 36;
                 InverseKinematics_B.b_k_i++) {
              InverseKinematics_B.weightMatrix_m[InverseKinematics_B.b_k_i] =
                args->WeightMatrix[InverseKinematics_B.b_k_i];
            }

            InverseKinematics_B.y_d[0] = xSol[0] +
              InverseKinematics_B.step_data[0];
            InverseKinematics_B.y_d[1] = xSol[1] +
              InverseKinematics_B.step_data[1];
            InverseKinematics_B.y_d[2] = xSol[2] +
              InverseKinematics_B.step_data[2];
            InverseKinematics_B.y_d[3] = xSol[3] +
              InverseKinematics_B.step_data[3];
            RigidBodyTree_efficientFKAndJac(treeInternal,
              InverseKinematics_B.y_d, bodyName, InverseKinematics_B.T_data,
              InverseKinematics_B.T_size, J);
            InverseKine_IKHelpers_poseError(InverseKinematics_B.Td,
              InverseKinematics_B.T_data, InverseKinematics_B.T_size,
              InverseKinematics_B.e);
            InverseKinematics_B.J = args->ErrTemp->size[0];
            args->ErrTemp->size[0] = 6;
            Invers_emxEnsureCapacity_real_T(args->ErrTemp, InverseKinematics_B.J);
            for (InverseKinematics_B.b_k_i = 0; InverseKinematics_B.b_k_i < 6;
                 InverseKinematics_B.b_k_i++) {
              args->ErrTemp->data[InverseKinematics_B.b_k_i] =
                InverseKinematics_B.e[InverseKinematics_B.b_k_i];
            }

            InverseKinematics_B.absxk = 0.0;
            for (InverseKinematics_B.b_k_i = 0; InverseKinematics_B.b_k_i < 6;
                 InverseKinematics_B.b_k_i++) {
              InverseKinematics_B.e_m[InverseKinematics_B.b_k_i] = 0.0;
              for (InverseKinematics_B.kend = 0; InverseKinematics_B.kend < 6;
                   InverseKinematics_B.kend++) {
                InverseKinematics_B.e_m[InverseKinematics_B.b_k_i] +=
                  InverseKinematics_B.weightMatrix_m[6 *
                  InverseKinematics_B.b_k_i + InverseKinematics_B.kend] * (0.5 *
                  InverseKinematics_B.e[InverseKinematics_B.kend]);
              }

              InverseKinematics_B.absxk +=
                InverseKinematics_B.e_m[InverseKinematics_B.b_k_i] *
                InverseKinematics_B.e[InverseKinematics_B.b_k_i];
            }

            args->CostTemp = InverseKinematics_B.absxk;
            for (InverseKinematics_B.b_k_i = 0; InverseKinematics_B.b_k_i < 6;
                 InverseKinematics_B.b_k_i++) {
              InverseKinematics_B.e_m[InverseKinematics_B.b_k_i] = 0.0;
              for (InverseKinematics_B.kend = 0; InverseKinematics_B.kend < 6;
                   InverseKinematics_B.kend++) {
                InverseKinematics_B.e_m[InverseKinematics_B.b_k_i] +=
                  InverseKinematics_B.weightMatrix_m[6 *
                  InverseKinematics_B.b_k_i + InverseKinematics_B.kend] *
                  InverseKinematics_B.e[InverseKinematics_B.kend];
              }
            }

            InverseKinematics_B.J = J_0->size[0] * J_0->size[1];
            J_0->size[0] = 6;
            J_0->size[1] = J->size[1];
            Invers_emxEnsureCapacity_real_T(J_0, InverseKinematics_B.J);
            InverseKinematics_B.kend = 6 * J->size[1];
            InverseKinematics_B.m = (InverseKinematics_B.kend / 2) << 1;
            InverseKinematics_B.coffset = InverseKinematics_B.m - 2;
            for (InverseKinematics_B.b_k_i = 0; InverseKinematics_B.b_k_i <=
                 InverseKinematics_B.coffset; InverseKinematics_B.b_k_i += 2) {
              tmp_2 = _mm_loadu_pd(&J->data[InverseKinematics_B.b_k_i]);
              _mm_storeu_pd(&J_0->data[InverseKinematics_B.b_k_i], _mm_mul_pd
                            (tmp_2, _mm_set1_pd(-1.0)));
            }

            for (InverseKinematics_B.b_k_i = InverseKinematics_B.m;
                 InverseKinematics_B.b_k_i < InverseKinematics_B.kend;
                 InverseKinematics_B.b_k_i++) {
              J_0->data[InverseKinematics_B.b_k_i] = -J->
                data[InverseKinematics_B.b_k_i];
            }

            InverseKinematics_mtimes_n(InverseKinematics_B.e_m, J_0, tmp);
            InverseKinematics_B.J = args->GradTemp->size[0];
            args->GradTemp->size[0] = tmp->size[1];
            Invers_emxEnsureCapacity_real_T(args->GradTemp,
              InverseKinematics_B.J);
            InverseKinematics_B.kend = tmp->size[1];
            for (InverseKinematics_B.b_k_i = 0; InverseKinematics_B.b_k_i <
                 InverseKinematics_B.kend; InverseKinematics_B.b_k_i++) {
              args->GradTemp->data[InverseKinematics_B.b_k_i] = tmp->
                data[InverseKinematics_B.b_k_i];
            }

            InverseKinematics_B.absxk = args->CostTemp;
            InverseKinematics_B.scale = 1.0;
            while (InverseKinematics_B.absxk > InverseKinematics_B.cost) {
              InverseKinematics_B.scale *= 2.5;
              InverseKinematics_B.absxk = InverseKinematics_B.scale *
                obj->DampingBias + InverseKinematics_B.cc;
              if ((H0->size[0] == 4) && (H0->size[1] == 4)) {
                for (InverseKinematics_B.b_k_i = 0; InverseKinematics_B.b_k_i <
                     4; InverseKinematics_B.b_k_i++) {
                  for (InverseKinematics_B.kend = 0; InverseKinematics_B.kend <=
                       2; InverseKinematics_B.kend += 2) {
                    InverseKinematics_B.J = (InverseKinematics_B.b_k_i << 2) +
                      InverseKinematics_B.kend;
                    tmp_2 = _mm_loadu_pd(&H0->data[H0->size[0] *
                                         InverseKinematics_B.b_k_i +
                                         InverseKinematics_B.kend]);
                    _mm_storeu_pd(&InverseKinematics_B.Td[InverseKinematics_B.J],
                                  _mm_mul_pd(_mm_add_pd(_mm_mul_pd(_mm_loadu_pd(
                      &tmp_0[InverseKinematics_B.J]), _mm_set1_pd
                      (InverseKinematics_B.absxk)), tmp_2), _mm_set1_pd(-1.0)));
                  }
                }

                InverseKinematics_mldivide(InverseKinematics_B.Td, grad,
                  InverseKinematics_B.step_data, &InverseKinematics_B.step_size);
              } else {
                InverseKinem_binary_expand_op_n(InverseKinematics_B.step_data,
                  &InverseKinematics_B.step_size, H0, InverseKinematics_B.absxk,
                  tmp_0, grad);
              }

              args = obj->ExtraArgs;
              treeInternal = args->Robot;
              InverseKinematics_B.J = bodyName->size[0] * bodyName->size[1];
              bodyName->size[0] = 1;
              bodyName->size[1] = args->BodyName->size[1];
              Invers_emxEnsureCapacity_char_T(bodyName, InverseKinematics_B.J);
              InverseKinematics_B.kend = args->BodyName->size[1] - 1;
              for (InverseKinematics_B.b_k_i = 0; InverseKinematics_B.b_k_i <=
                   InverseKinematics_B.kend; InverseKinematics_B.b_k_i++) {
                InverseKinematics_B.J = InverseKinematics_B.b_k_i;
                bodyName->data[InverseKinematics_B.J] = args->BodyName->
                  data[InverseKinematics_B.J];
              }

              for (InverseKinematics_B.b_k_i = 0; InverseKinematics_B.b_k_i < 16;
                   InverseKinematics_B.b_k_i++) {
                InverseKinematics_B.Td[InverseKinematics_B.b_k_i] = args->
                  Tform[InverseKinematics_B.b_k_i];
              }

              for (InverseKinematics_B.b_k_i = 0; InverseKinematics_B.b_k_i < 36;
                   InverseKinematics_B.b_k_i++) {
                InverseKinematics_B.weightMatrix_m[InverseKinematics_B.b_k_i] =
                  args->WeightMatrix[InverseKinematics_B.b_k_i];
              }

              InverseKinematics_B.y_d[0] = xSol[0] +
                InverseKinematics_B.step_data[0];
              InverseKinematics_B.y_d[1] = xSol[1] +
                InverseKinematics_B.step_data[1];
              InverseKinematics_B.y_d[2] = xSol[2] +
                InverseKinematics_B.step_data[2];
              InverseKinematics_B.y_d[3] = xSol[3] +
                InverseKinematics_B.step_data[3];
              RigidBodyTree_efficientFKAndJac(treeInternal,
                InverseKinematics_B.y_d, bodyName, InverseKinematics_B.T_data,
                InverseKinematics_B.T_size, J);
              InverseKine_IKHelpers_poseError(InverseKinematics_B.Td,
                InverseKinematics_B.T_data, InverseKinematics_B.T_size,
                InverseKinematics_B.e);
              InverseKinematics_B.J = args->ErrTemp->size[0];
              args->ErrTemp->size[0] = 6;
              Invers_emxEnsureCapacity_real_T(args->ErrTemp,
                InverseKinematics_B.J);
              for (InverseKinematics_B.b_k_i = 0; InverseKinematics_B.b_k_i < 6;
                   InverseKinematics_B.b_k_i++) {
                args->ErrTemp->data[InverseKinematics_B.b_k_i] =
                  InverseKinematics_B.e[InverseKinematics_B.b_k_i];
              }

              InverseKinematics_B.absxk = 0.0;
              for (InverseKinematics_B.b_k_i = 0; InverseKinematics_B.b_k_i < 6;
                   InverseKinematics_B.b_k_i++) {
                InverseKinematics_B.e_m[InverseKinematics_B.b_k_i] = 0.0;
                for (InverseKinematics_B.kend = 0; InverseKinematics_B.kend < 6;
                     InverseKinematics_B.kend++) {
                  InverseKinematics_B.e_m[InverseKinematics_B.b_k_i] +=
                    InverseKinematics_B.weightMatrix_m[6 *
                    InverseKinematics_B.b_k_i + InverseKinematics_B.kend] * (0.5
                    * InverseKinematics_B.e[InverseKinematics_B.kend]);
                }

                InverseKinematics_B.absxk +=
                  InverseKinematics_B.e_m[InverseKinematics_B.b_k_i] *
                  InverseKinematics_B.e[InverseKinematics_B.b_k_i];
              }

              args->CostTemp = InverseKinematics_B.absxk;
              for (InverseKinematics_B.b_k_i = 0; InverseKinematics_B.b_k_i < 6;
                   InverseKinematics_B.b_k_i++) {
                InverseKinematics_B.e_m[InverseKinematics_B.b_k_i] = 0.0;
                for (InverseKinematics_B.kend = 0; InverseKinematics_B.kend < 6;
                     InverseKinematics_B.kend++) {
                  InverseKinematics_B.e_m[InverseKinematics_B.b_k_i] +=
                    InverseKinematics_B.weightMatrix_m[6 *
                    InverseKinematics_B.b_k_i + InverseKinematics_B.kend] *
                    InverseKinematics_B.e[InverseKinematics_B.kend];
                }
              }

              InverseKinematics_B.J = J_0->size[0] * J_0->size[1];
              J_0->size[0] = 6;
              J_0->size[1] = J->size[1];
              Invers_emxEnsureCapacity_real_T(J_0, InverseKinematics_B.J);
              InverseKinematics_B.kend = 6 * J->size[1];
              InverseKinematics_B.m = (InverseKinematics_B.kend / 2) << 1;
              InverseKinematics_B.coffset = InverseKinematics_B.m - 2;
              for (InverseKinematics_B.b_k_i = 0; InverseKinematics_B.b_k_i <=
                   InverseKinematics_B.coffset; InverseKinematics_B.b_k_i += 2)
              {
                tmp_2 = _mm_loadu_pd(&J->data[InverseKinematics_B.b_k_i]);
                _mm_storeu_pd(&J_0->data[InverseKinematics_B.b_k_i], _mm_mul_pd
                              (tmp_2, _mm_set1_pd(-1.0)));
              }

              for (InverseKinematics_B.b_k_i = InverseKinematics_B.m;
                   InverseKinematics_B.b_k_i < InverseKinematics_B.kend;
                   InverseKinematics_B.b_k_i++) {
                J_0->data[InverseKinematics_B.b_k_i] = -J->
                  data[InverseKinematics_B.b_k_i];
              }

              InverseKinematics_mtimes_n(InverseKinematics_B.e_m, J_0, tmp);
              InverseKinematics_B.J = args->GradTemp->size[0];
              args->GradTemp->size[0] = tmp->size[1];
              Invers_emxEnsureCapacity_real_T(args->GradTemp,
                InverseKinematics_B.J);
              InverseKinematics_B.kend = tmp->size[1];
              for (InverseKinematics_B.b_k_i = 0; InverseKinematics_B.b_k_i <
                   InverseKinematics_B.kend; InverseKinematics_B.b_k_i++) {
                args->GradTemp->data[InverseKinematics_B.b_k_i] = tmp->
                  data[InverseKinematics_B.b_k_i];
              }

              InverseKinematics_B.absxk = args->CostTemp;
            }

            InverseKinematics_B.cost = xSol[0] + InverseKinematics_B.step_data[0];
            xSol[0] += InverseKinematics_B.step_data[0];
            InverseKinematics_B.absxk = xSol[1] + InverseKinematics_B.step_data
              [1];
            xSol[1] += InverseKinematics_B.step_data[1];
            InverseKinematics_B.cc = xSol[2] + InverseKinematics_B.step_data[2];
            xSol[2] += InverseKinematics_B.step_data[2];
            InverseKinematics_B.scale = xSol[3] + InverseKinematics_B.step_data
              [3];
            xSol[3] += InverseKinematics_B.step_data[3];
            if (obj->ConstraintsOn) {
              args = obj->ExtraArgs;
              InverseKinematics_B.kend = args->Limits->size[0];
              InverseKinematics_B.J = grad->size[0];
              grad->size[0] = InverseKinematics_B.kend;
              Invers_emxEnsureCapacity_real_T(grad, InverseKinematics_B.J);
              for (InverseKinematics_B.b_k_i = 0; InverseKinematics_B.b_k_i <
                   InverseKinematics_B.kend; InverseKinematics_B.b_k_i++) {
                grad->data[InverseKinematics_B.b_k_i] = args->Limits->
                  data[InverseKinematics_B.b_k_i];
              }

              if (grad->size[0] == 4) {
                if ((grad->data[0] >= InverseKinematics_B.cost) || rtIsNaN
                    (InverseKinematics_B.cost)) {
                  InverseKinematics_B.y_d[0] = grad->data[0];
                } else {
                  InverseKinematics_B.y_d[0] = InverseKinematics_B.cost;
                }

                if ((grad->data[1] >= InverseKinematics_B.absxk) || rtIsNaN
                    (InverseKinematics_B.absxk)) {
                  InverseKinematics_B.y_d[1] = grad->data[1];
                } else {
                  InverseKinematics_B.y_d[1] = InverseKinematics_B.absxk;
                }

                if ((grad->data[2] >= InverseKinematics_B.cc) || rtIsNaN
                    (InverseKinematics_B.cc)) {
                  InverseKinematics_B.y_d[2] = grad->data[2];
                } else {
                  InverseKinematics_B.y_d[2] = InverseKinematics_B.cc;
                }

                if ((grad->data[3] >= InverseKinematics_B.scale) || rtIsNaN
                    (InverseKinematics_B.scale)) {
                  InverseKinematics_B.y_d[3] = grad->data[3];
                } else {
                  InverseKinematics_B.y_d[3] = InverseKinematics_B.scale;
                }
              } else {
                InverseKinematics_expand_max(grad, xSol, InverseKinematics_B.y_d);
              }

              InverseKinematics_B.kend = args->Limits->size[0];
              InverseKinematics_B.J = grad->size[0];
              grad->size[0] = InverseKinematics_B.kend;
              Invers_emxEnsureCapacity_real_T(grad, InverseKinematics_B.J);
              for (InverseKinematics_B.b_k_i = 0; InverseKinematics_B.b_k_i <
                   InverseKinematics_B.kend; InverseKinematics_B.b_k_i++) {
                grad->data[InverseKinematics_B.b_k_i] = args->Limits->
                  data[InverseKinematics_B.b_k_i + args->Limits->size[0]];
              }

              if (grad->size[0] == 4) {
                if ((grad->data[0] <= InverseKinematics_B.y_d[0]) || rtIsNaN
                    (InverseKinematics_B.y_d[0])) {
                  xSol[0] = grad->data[0];
                } else {
                  xSol[0] = InverseKinematics_B.y_d[0];
                }

                if ((grad->data[1] <= InverseKinematics_B.y_d[1]) || rtIsNaN
                    (InverseKinematics_B.y_d[1])) {
                  xSol[1] = grad->data[1];
                } else {
                  xSol[1] = InverseKinematics_B.y_d[1];
                }

                if ((grad->data[2] <= InverseKinematics_B.y_d[2]) || rtIsNaN
                    (InverseKinematics_B.y_d[2])) {
                  xSol[2] = grad->data[2];
                } else {
                  xSol[2] = InverseKinematics_B.y_d[2];
                }

                if ((grad->data[3] <= InverseKinematics_B.y_d[3]) || rtIsNaN
                    (InverseKinematics_B.y_d[3])) {
                  xSol[3] = grad->data[3];
                } else {
                  xSol[3] = InverseKinematics_B.y_d[3];
                }
              } else {
                InverseKinematics_expand_min(grad, InverseKinematics_B.y_d, xSol);
              }
            }

            InverseKinematics_B.b_i_f++;
          }
        }
      }
    } else {
      args = obj->ExtraArgs;
      for (InverseKinematics_B.b_k_i = 0; InverseKinematics_B.b_k_i < 36;
           InverseKinematics_B.b_k_i++) {
        InverseKinematics_B.a[InverseKinematics_B.b_k_i] = args->
          WeightMatrix[InverseKinematics_B.b_k_i];
      }

      InverseKinematics_B.J = b->size[0];
      b->size[0] = args->ErrTemp->size[0];
      Invers_emxEnsureCapacity_real_T(b, InverseKinematics_B.J);
      InverseKinematics_B.kend = args->ErrTemp->size[0];
      for (InverseKinematics_B.b_k_i = 0; InverseKinematics_B.b_k_i <
           InverseKinematics_B.kend; InverseKinematics_B.b_k_i++) {
        b->data[InverseKinematics_B.b_k_i] = args->ErrTemp->
          data[InverseKinematics_B.b_k_i];
      }

      for (InverseKinematics_B.b_k_i = 0; InverseKinematics_B.b_k_i < 6;
           InverseKinematics_B.b_k_i++) {
        InverseKinematics_B.e[InverseKinematics_B.b_k_i] = 0.0;
      }

      for (InverseKinematics_B.kend = 0; InverseKinematics_B.kend < 6;
           InverseKinematics_B.kend++) {
        for (InverseKinematics_B.b_k_i = 0; InverseKinematics_B.b_k_i <= 4;
             InverseKinematics_B.b_k_i += 2) {
          __m128d tmp_1;
          tmp_2 = _mm_loadu_pd(&InverseKinematics_B.a[6 *
                               InverseKinematics_B.kend +
                               InverseKinematics_B.b_k_i]);
          tmp_1 = _mm_loadu_pd(&InverseKinematics_B.e[InverseKinematics_B.b_k_i]);
          _mm_storeu_pd(&InverseKinematics_B.e[InverseKinematics_B.b_k_i],
                        _mm_add_pd(_mm_mul_pd(tmp_2, _mm_set1_pd(b->
            data[InverseKinematics_B.kend])), tmp_1));
        }
      }

      *en = InverseKinematics_norm_n(InverseKinematics_B.e);
      *iter = obj->MaxNumIterationInternal;
      *exitFlag = IterationLimitExceeded;
      exitg1 = 1;
    }
  } while (exitg1 == 0);

  InverseKinematic_emxFree_real_T(&J_0);
  InverseKinematic_emxFree_real_T(&tmp);
  InverseKinema_emxFree_boolean_T(&x);
  InverseKinematic_emxFree_real_T(&b);
  InverseKinematic_emxFree_real_T(&J);
  InverseKinematic_emxFree_char_T(&bodyName);
  InverseKinematic_emxFree_real_T(&y);
  InverseKinematic_emxFree_real_T(&ev);
  InverseKinematic_emxFree_real_T(&H0);
  InverseKinematic_emxFree_real_T(&grad);
  InverseKinematic_emxFree_real_T(&evprev);
}

boolean_T InverseKinematics::InverseKinematics_any(const
  emxArray_boolean_T_InverseKin_T *x)
{
  int32_T ix;
  boolean_T exitg1;
  boolean_T y;
  y = false;
  ix = 0;
  exitg1 = false;
  while ((!exitg1) && (ix + 1 <= x->size[0])) {
    if (x->data[ix]) {
      y = true;
      exitg1 = true;
    } else {
      ix++;
    }
  }

  return y;
}

void InverseKinematics::InverseKinematics_randn(const real_T varargin_1[2],
  emxArray_real_T_InverseKinema_T *r)
{
  static const real_T tmp[257] = { 1.0, 0.977101701267673, 0.959879091800108,
    0.9451989534423, 0.932060075959231, 0.919991505039348, 0.908726440052131,
    0.898095921898344, 0.887984660755834, 0.878309655808918, 0.869008688036857,
    0.860033621196332, 0.851346258458678, 0.842915653112205, 0.834716292986884,
    0.826726833946222, 0.818929191603703, 0.811307874312656, 0.803849483170964,
    0.796542330422959, 0.789376143566025, 0.782341832654803, 0.775431304981187,
    0.768637315798486, 0.761953346836795, 0.755373506507096, 0.748892447219157,
    0.742505296340151, 0.736207598126863, 0.729995264561476, 0.72386453346863,
    0.717811932630722, 0.711834248878248, 0.705928501332754, 0.700091918136512,
    0.694321916126117, 0.688616083004672, 0.682972161644995, 0.677388036218774,
    0.671861719897082, 0.66639134390875, 0.660975147776663, 0.655611470579697,
    0.650298743110817, 0.645035480820822, 0.639820277453057, 0.634651799287624,
    0.629528779924837, 0.624450015547027, 0.619414360605834, 0.614420723888914,
    0.609468064925773, 0.604555390697468, 0.599681752619125, 0.594846243767987,
    0.590047996332826, 0.585286179263371, 0.580559996100791, 0.575868682972354,
    0.571211506735253, 0.566587763256165, 0.561996775814525, 0.557437893618766,
    0.552910490425833, 0.548413963255266, 0.543947731190026, 0.539511234256952,
    0.535103932380458, 0.530725304403662, 0.526374847171684, 0.522052074672322,
    0.517756517229756, 0.513487720747327, 0.509245245995748, 0.505028667943468,
    0.500837575126149, 0.49667156905249, 0.492530263643869, 0.488413284705458,
    0.484320269426683, 0.480250865909047, 0.476204732719506, 0.47218153846773,
    0.468180961405694, 0.464202689048174, 0.460246417812843, 0.456311852678716,
    0.452398706861849, 0.448506701507203, 0.444635565395739, 0.440785034665804,
    0.436954852547985, 0.433144769112652, 0.429354541029442, 0.425583931338022,
    0.421832709229496, 0.418100649837848, 0.414387534040891, 0.410693148270188,
    0.407017284329473, 0.403359739221114, 0.399720314980197, 0.396098818515832,
    0.392495061459315, 0.388908860018789, 0.385340034840077, 0.381788410873393,
    0.378253817245619, 0.374736087137891, 0.371235057668239, 0.367750569779032,
    0.364282468129004, 0.360830600989648, 0.357394820145781, 0.353974980800077,
    0.350570941481406, 0.347182563956794, 0.343809713146851, 0.340452257044522,
    0.337110066637006, 0.333783015830718, 0.330470981379163, 0.327173842813601,
    0.323891482376391, 0.320623784956905, 0.317370638029914, 0.314131931596337,
    0.310907558126286, 0.307697412504292, 0.30450139197665, 0.301319396100803,
    0.298151326696685, 0.294997087799962, 0.291856585617095, 0.288729728482183,
    0.285616426815502, 0.282516593083708, 0.279430141761638, 0.276356989295668,
    0.273297054068577, 0.270250256365875, 0.267216518343561, 0.264195763997261,
    0.261187919132721, 0.258192911337619, 0.255210669954662, 0.252241126055942,
    0.249284212418529, 0.246339863501264, 0.24340801542275, 0.240488605940501,
    0.237581574431238, 0.23468686187233, 0.231804410824339, 0.228934165414681,
    0.226076071322381, 0.223230075763918, 0.220396127480152, 0.217574176724331,
    0.214764175251174, 0.211966076307031, 0.209179834621125, 0.206405406397881,
    0.203642749310335, 0.200891822494657, 0.198152586545776, 0.195425003514135,
    0.192709036903589, 0.190004651670465, 0.187311814223801, 0.1846304924268,
    0.181960655599523, 0.179302274522848, 0.176655321443735, 0.174019770081839,
    0.171395595637506, 0.168782774801212, 0.166181285764482, 0.163591108232366,
    0.161012223437511, 0.158444614155925, 0.15588826472448, 0.153343161060263,
    0.150809290681846, 0.148286642732575, 0.145775208005994, 0.143274978973514,
    0.140785949814445, 0.138308116448551, 0.135841476571254, 0.133386029691669,
    0.130941777173644, 0.12850872228, 0.126086870220186, 0.123676228201597,
    0.12127680548479, 0.11888861344291, 0.116511665625611, 0.114145977827839,
    0.111791568163838, 0.109448457146812, 0.107116667774684, 0.104796225622487,
    0.102487158941935, 0.10018949876881, 0.0979032790388625, 0.095628536713009,
    0.093365311912691, 0.0911136480663738, 0.0888735920682759,
    0.0866451944505581, 0.0844285095703535, 0.082223595813203,
    0.0800305158146631, 0.0778493367020961, 0.0756801303589272,
    0.0735229737139814, 0.0713779490588905, 0.0692451443970068,
    0.0671246538277886, 0.065016577971243, 0.0629210244377582, 0.06083810834954,
    0.0587679529209339, 0.0567106901062031, 0.0546664613248891,
    0.0526354182767924, 0.0506177238609479, 0.0486135532158687,
    0.0466230949019305, 0.0446465522512946, 0.0426841449164746,
    0.0407361106559411, 0.0388027074045262, 0.0368842156885674,
    0.0349809414617162, 0.0330932194585786, 0.0312214171919203,
    0.0293659397581334, 0.0275272356696031, 0.0257058040085489,
    0.0239022033057959, 0.0221170627073089, 0.0203510962300445,
    0.0186051212757247, 0.0168800831525432, 0.0151770883079353,
    0.0134974506017399, 0.0118427578579079, 0.0102149714397015,
    0.00861658276939875, 0.00705087547137324, 0.00552240329925101,
    0.00403797259336304, 0.00260907274610216, 0.0012602859304986,
    0.000477467764609386 };

  const real_T *fitab;
  InverseKinematics_B.b_k_p = r->size[0];
  r->size[0] = static_cast<int32_T>(varargin_1[0]);
  Invers_emxEnsureCapacity_real_T(r, InverseKinematics_B.b_k_p);
  InverseKinematics_B.d_tmp = static_cast<int32_T>(varargin_1[0]) - 1;
  if (static_cast<int32_T>(varargin_1[0]) - 1 >= 0) {
    InverseKinematics_B.xi[0] = 0.0;
    InverseKinematics_B.xi[1] = 0.215241895984875;
    InverseKinematics_B.xi[2] = 0.286174591792068;
    InverseKinematics_B.xi[3] = 0.335737519214422;
    InverseKinematics_B.xi[4] = 0.375121332878378;
    InverseKinematics_B.xi[5] = 0.408389134611989;
    InverseKinematics_B.xi[6] = 0.43751840220787;
    InverseKinematics_B.xi[7] = 0.46363433679088;
    InverseKinematics_B.xi[8] = 0.487443966139235;
    InverseKinematics_B.xi[9] = 0.50942332960209;
    InverseKinematics_B.xi[10] = 0.529909720661557;
    InverseKinematics_B.xi[11] = 0.549151702327164;
    InverseKinematics_B.xi[12] = 0.567338257053817;
    InverseKinematics_B.xi[13] = 0.584616766106378;
    InverseKinematics_B.xi[14] = 0.601104617755991;
    InverseKinematics_B.xi[15] = 0.61689699000775;
    InverseKinematics_B.xi[16] = 0.63207223638606;
    InverseKinematics_B.xi[17] = 0.646695714894993;
    InverseKinematics_B.xi[18] = 0.660822574244419;
    InverseKinematics_B.xi[19] = 0.674499822837293;
    InverseKinematics_B.xi[20] = 0.687767892795788;
    InverseKinematics_B.xi[21] = 0.700661841106814;
    InverseKinematics_B.xi[22] = 0.713212285190975;
    InverseKinematics_B.xi[23] = 0.725446140909999;
    InverseKinematics_B.xi[24] = 0.737387211434295;
    InverseKinematics_B.xi[25] = 0.749056662017815;
    InverseKinematics_B.xi[26] = 0.760473406430107;
    InverseKinematics_B.xi[27] = 0.771654424224568;
    InverseKinematics_B.xi[28] = 0.782615023307232;
    InverseKinematics_B.xi[29] = 0.793369058840623;
    InverseKinematics_B.xi[30] = 0.80392911698997;
    InverseKinematics_B.xi[31] = 0.814306670135215;
    InverseKinematics_B.xi[32] = 0.824512208752291;
    InverseKinematics_B.xi[33] = 0.834555354086381;
    InverseKinematics_B.xi[34] = 0.844444954909153;
    InverseKinematics_B.xi[35] = 0.854189171008163;
    InverseKinematics_B.xi[36] = 0.863795545553308;
    InverseKinematics_B.xi[37] = 0.87327106808886;
    InverseKinematics_B.xi[38] = 0.882622229585165;
    InverseKinematics_B.xi[39] = 0.891855070732941;
    InverseKinematics_B.xi[40] = 0.900975224461221;
    InverseKinematics_B.xi[41] = 0.909987953496718;
    InverseKinematics_B.xi[42] = 0.91889818364959;
    InverseKinematics_B.xi[43] = 0.927710533401999;
    InverseKinematics_B.xi[44] = 0.936429340286575;
    InverseKinematics_B.xi[45] = 0.945058684468165;
    InverseKinematics_B.xi[46] = 0.953602409881086;
    InverseKinematics_B.xi[47] = 0.96206414322304;
    InverseKinematics_B.xi[48] = 0.970447311064224;
    InverseKinematics_B.xi[49] = 0.978755155294224;
    InverseKinematics_B.xi[50] = 0.986990747099062;
    InverseKinematics_B.xi[51] = 0.99515699963509;
    InverseKinematics_B.xi[52] = 1.00325667954467;
    InverseKinematics_B.xi[53] = 1.01129241744;
    InverseKinematics_B.xi[54] = 1.01926671746548;
    InverseKinematics_B.xi[55] = 1.02718196603564;
    InverseKinematics_B.xi[56] = 1.03504043983344;
    InverseKinematics_B.xi[57] = 1.04284431314415;
    InverseKinematics_B.xi[58] = 1.05059566459093;
    InverseKinematics_B.xi[59] = 1.05829648333067;
    InverseKinematics_B.xi[60] = 1.06594867476212;
    InverseKinematics_B.xi[61] = 1.07355406579244;
    InverseKinematics_B.xi[62] = 1.0811144097034;
    InverseKinematics_B.xi[63] = 1.08863139065398;
    InverseKinematics_B.xi[64] = 1.09610662785202;
    InverseKinematics_B.xi[65] = 1.10354167942464;
    InverseKinematics_B.xi[66] = 1.11093804601357;
    InverseKinematics_B.xi[67] = 1.11829717411934;
    InverseKinematics_B.xi[68] = 1.12562045921553;
    InverseKinematics_B.xi[69] = 1.13290924865253;
    InverseKinematics_B.xi[70] = 1.14016484436815;
    InverseKinematics_B.xi[71] = 1.14738850542085;
    InverseKinematics_B.xi[72] = 1.15458145035993;
    InverseKinematics_B.xi[73] = 1.16174485944561;
    InverseKinematics_B.xi[74] = 1.16887987673083;
    InverseKinematics_B.xi[75] = 1.17598761201545;
    InverseKinematics_B.xi[76] = 1.18306914268269;
    InverseKinematics_B.xi[77] = 1.19012551542669;
    InverseKinematics_B.xi[78] = 1.19715774787944;
    InverseKinematics_B.xi[79] = 1.20416683014438;
    InverseKinematics_B.xi[80] = 1.2111537262437;
    InverseKinematics_B.xi[81] = 1.21811937548548;
    InverseKinematics_B.xi[82] = 1.22506469375653;
    InverseKinematics_B.xi[83] = 1.23199057474614;
    InverseKinematics_B.xi[84] = 1.23889789110569;
    InverseKinematics_B.xi[85] = 1.24578749554863;
    InverseKinematics_B.xi[86] = 1.2526602218949;
    InverseKinematics_B.xi[87] = 1.25951688606371;
    InverseKinematics_B.xi[88] = 1.26635828701823;
    InverseKinematics_B.xi[89] = 1.27318520766536;
    InverseKinematics_B.xi[90] = 1.27999841571382;
    InverseKinematics_B.xi[91] = 1.28679866449324;
    InverseKinematics_B.xi[92] = 1.29358669373695;
    InverseKinematics_B.xi[93] = 1.30036323033084;
    InverseKinematics_B.xi[94] = 1.30712898903073;
    InverseKinematics_B.xi[95] = 1.31388467315022;
    InverseKinematics_B.xi[96] = 1.32063097522106;
    InverseKinematics_B.xi[97] = 1.32736857762793;
    InverseKinematics_B.xi[98] = 1.33409815321936;
    InverseKinematics_B.xi[99] = 1.3408203658964;
    InverseKinematics_B.xi[100] = 1.34753587118059;
    InverseKinematics_B.xi[101] = 1.35424531676263;
    InverseKinematics_B.xi[102] = 1.36094934303328;
    InverseKinematics_B.xi[103] = 1.36764858359748;
    InverseKinematics_B.xi[104] = 1.37434366577317;
    InverseKinematics_B.xi[105] = 1.38103521107586;
    InverseKinematics_B.xi[106] = 1.38772383568998;
    InverseKinematics_B.xi[107] = 1.39441015092814;
    InverseKinematics_B.xi[108] = 1.40109476367925;
    InverseKinematics_B.xi[109] = 1.4077782768464;
    InverseKinematics_B.xi[110] = 1.41446128977547;
    InverseKinematics_B.xi[111] = 1.42114439867531;
    InverseKinematics_B.xi[112] = 1.42782819703026;
    InverseKinematics_B.xi[113] = 1.43451327600589;
    InverseKinematics_B.xi[114] = 1.44120022484872;
    InverseKinematics_B.xi[115] = 1.44788963128058;
    InverseKinematics_B.xi[116] = 1.45458208188841;
    InverseKinematics_B.xi[117] = 1.46127816251028;
    InverseKinematics_B.xi[118] = 1.46797845861808;
    InverseKinematics_B.xi[119] = 1.47468355569786;
    InverseKinematics_B.xi[120] = 1.48139403962819;
    InverseKinematics_B.xi[121] = 1.48811049705745;
    InverseKinematics_B.xi[122] = 1.49483351578049;
    InverseKinematics_B.xi[123] = 1.50156368511546;
    InverseKinematics_B.xi[124] = 1.50830159628131;
    InverseKinematics_B.xi[125] = 1.51504784277671;
    InverseKinematics_B.xi[126] = 1.521803020761;
    InverseKinematics_B.xi[127] = 1.52856772943771;
    InverseKinematics_B.xi[128] = 1.53534257144151;
    InverseKinematics_B.xi[129] = 1.542128153229;
    InverseKinematics_B.xi[130] = 1.54892508547417;
    InverseKinematics_B.xi[131] = 1.55573398346918;
    InverseKinematics_B.xi[132] = 1.56255546753104;
    InverseKinematics_B.xi[133] = 1.56939016341512;
    InverseKinematics_B.xi[134] = 1.57623870273591;
    InverseKinematics_B.xi[135] = 1.58310172339603;
    InverseKinematics_B.xi[136] = 1.58997987002419;
    InverseKinematics_B.xi[137] = 1.59687379442279;
    InverseKinematics_B.xi[138] = 1.60378415602609;
    InverseKinematics_B.xi[139] = 1.61071162236983;
    InverseKinematics_B.xi[140] = 1.61765686957301;
    InverseKinematics_B.xi[141] = 1.62462058283303;
    InverseKinematics_B.xi[142] = 1.63160345693487;
    InverseKinematics_B.xi[143] = 1.63860619677555;
    InverseKinematics_B.xi[144] = 1.64562951790478;
    InverseKinematics_B.xi[145] = 1.65267414708306;
    InverseKinematics_B.xi[146] = 1.65974082285818;
    InverseKinematics_B.xi[147] = 1.66683029616166;
    InverseKinematics_B.xi[148] = 1.67394333092612;
    InverseKinematics_B.xi[149] = 1.68108070472517;
    InverseKinematics_B.xi[150] = 1.68824320943719;
    InverseKinematics_B.xi[151] = 1.69543165193456;
    InverseKinematics_B.xi[152] = 1.70264685479992;
    InverseKinematics_B.xi[153] = 1.7098896570713;
    InverseKinematics_B.xi[154] = 1.71716091501782;
    InverseKinematics_B.xi[155] = 1.72446150294804;
    InverseKinematics_B.xi[156] = 1.73179231405296;
    InverseKinematics_B.xi[157] = 1.73915426128591;
    InverseKinematics_B.xi[158] = 1.74654827828172;
    InverseKinematics_B.xi[159] = 1.75397532031767;
    InverseKinematics_B.xi[160] = 1.76143636531891;
    InverseKinematics_B.xi[161] = 1.76893241491127;
    InverseKinematics_B.xi[162] = 1.77646449552452;
    InverseKinematics_B.xi[163] = 1.78403365954944;
    InverseKinematics_B.xi[164] = 1.79164098655216;
    InverseKinematics_B.xi[165] = 1.79928758454972;
    InverseKinematics_B.xi[166] = 1.80697459135082;
    InverseKinematics_B.xi[167] = 1.81470317596628;
    InverseKinematics_B.xi[168] = 1.82247454009388;
    InverseKinematics_B.xi[169] = 1.83028991968276;
    InverseKinematics_B.xi[170] = 1.83815058658281;
    InverseKinematics_B.xi[171] = 1.84605785028518;
    InverseKinematics_B.xi[172] = 1.8540130597602;
    InverseKinematics_B.xi[173] = 1.86201760539967;
    InverseKinematics_B.xi[174] = 1.87007292107127;
    InverseKinematics_B.xi[175] = 1.878180486293;
    InverseKinematics_B.xi[176] = 1.88634182853678;
    InverseKinematics_B.xi[177] = 1.8945585256707;
    InverseKinematics_B.xi[178] = 1.90283220855043;
    InverseKinematics_B.xi[179] = 1.91116456377125;
    InverseKinematics_B.xi[180] = 1.91955733659319;
    InverseKinematics_B.xi[181] = 1.92801233405266;
    InverseKinematics_B.xi[182] = 1.93653142827569;
    InverseKinematics_B.xi[183] = 1.94511656000868;
    InverseKinematics_B.xi[184] = 1.95376974238465;
    InverseKinematics_B.xi[185] = 1.96249306494436;
    InverseKinematics_B.xi[186] = 1.97128869793366;
    InverseKinematics_B.xi[187] = 1.98015889690048;
    InverseKinematics_B.xi[188] = 1.98910600761744;
    InverseKinematics_B.xi[189] = 1.99813247135842;
    InverseKinematics_B.xi[190] = 2.00724083056053;
    InverseKinematics_B.xi[191] = 2.0164337349062;
    InverseKinematics_B.xi[192] = 2.02571394786385;
    InverseKinematics_B.xi[193] = 2.03508435372962;
    InverseKinematics_B.xi[194] = 2.04454796521753;
    InverseKinematics_B.xi[195] = 2.05410793165065;
    InverseKinematics_B.xi[196] = 2.06376754781173;
    InverseKinematics_B.xi[197] = 2.07353026351874;
    InverseKinematics_B.xi[198] = 2.0833996939983;
    InverseKinematics_B.xi[199] = 2.09337963113879;
    InverseKinematics_B.xi[200] = 2.10347405571488;
    InverseKinematics_B.xi[201] = 2.11368715068665;
    InverseKinematics_B.xi[202] = 2.12402331568952;
    InverseKinematics_B.xi[203] = 2.13448718284602;
    InverseKinematics_B.xi[204] = 2.14508363404789;
    InverseKinematics_B.xi[205] = 2.15581781987674;
    InverseKinematics_B.xi[206] = 2.16669518035431;
    InverseKinematics_B.xi[207] = 2.17772146774029;
    InverseKinematics_B.xi[208] = 2.18890277162636;
    InverseKinematics_B.xi[209] = 2.20024554661128;
    InverseKinematics_B.xi[210] = 2.21175664288416;
    InverseKinematics_B.xi[211] = 2.22344334009251;
    InverseKinematics_B.xi[212] = 2.23531338492992;
    InverseKinematics_B.xi[213] = 2.24737503294739;
    InverseKinematics_B.xi[214] = 2.25963709517379;
    InverseKinematics_B.xi[215] = 2.27210899022838;
    InverseKinematics_B.xi[216] = 2.28480080272449;
    InverseKinematics_B.xi[217] = 2.29772334890286;
    InverseKinematics_B.xi[218] = 2.31088825060137;
    InverseKinematics_B.xi[219] = 2.32430801887113;
    InverseKinematics_B.xi[220] = 2.33799614879653;
    InverseKinematics_B.xi[221] = 2.35196722737914;
    InverseKinematics_B.xi[222] = 2.36623705671729;
    InverseKinematics_B.xi[223] = 2.38082279517208;
    InverseKinematics_B.xi[224] = 2.39574311978193;
    InverseKinematics_B.xi[225] = 2.41101841390112;
    InverseKinematics_B.xi[226] = 2.42667098493715;
    InverseKinematics_B.xi[227] = 2.44272531820036;
    InverseKinematics_B.xi[228] = 2.4592083743347;
    InverseKinematics_B.xi[229] = 2.47614993967052;
    InverseKinematics_B.xi[230] = 2.49358304127105;
    InverseKinematics_B.xi[231] = 2.51154444162669;
    InverseKinematics_B.xi[232] = 2.53007523215985;
    InverseKinematics_B.xi[233] = 2.54922155032478;
    InverseKinematics_B.xi[234] = 2.56903545268184;
    InverseKinematics_B.xi[235] = 2.58957598670829;
    InverseKinematics_B.xi[236] = 2.61091051848882;
    InverseKinematics_B.xi[237] = 2.63311639363158;
    InverseKinematics_B.xi[238] = 2.65628303757674;
    InverseKinematics_B.xi[239] = 2.68051464328574;
    InverseKinematics_B.xi[240] = 2.70593365612306;
    InverseKinematics_B.xi[241] = 2.73268535904401;
    InverseKinematics_B.xi[242] = 2.76094400527999;
    InverseKinematics_B.xi[243] = 2.79092117400193;
    InverseKinematics_B.xi[244] = 2.82287739682644;
    InverseKinematics_B.xi[245] = 2.85713873087322;
    InverseKinematics_B.xi[246] = 2.89412105361341;
    InverseKinematics_B.xi[247] = 2.93436686720889;
    InverseKinematics_B.xi[248] = 2.97860327988184;
    InverseKinematics_B.xi[249] = 3.02783779176959;
    InverseKinematics_B.xi[250] = 3.08352613200214;
    InverseKinematics_B.xi[251] = 3.147889289518;
    InverseKinematics_B.xi[252] = 3.2245750520478;
    InverseKinematics_B.xi[253] = 3.32024473383983;
    InverseKinematics_B.xi[254] = 3.44927829856143;
    InverseKinematics_B.xi[255] = 3.65415288536101;
    InverseKinematics_B.xi[256] = 3.91075795952492;
    fitab = &tmp[0];
  }

  for (InverseKinematics_B.b_k_p = 0; InverseKinematics_B.b_k_p <=
       InverseKinematics_B.d_tmp; InverseKinematics_B.b_k_p++) {
    int32_T exitg1;
    do {
      exitg1 = 0;
      Inverse_genrand_uint32_vector_n(InverseKinematics_DW.state_c,
        InverseKinematics_B.u32);
      InverseKinematics_B.i_e = static_cast<int32_T>((InverseKinematics_B.u32[1]
        >> 24U) + 1U);
      InverseKinematics_B.b_r = ((static_cast<real_T>(InverseKinematics_B.u32[0]
        >> 3U) * 1.6777216E+7 + static_cast<real_T>(static_cast<int32_T>
        (InverseKinematics_B.u32[1]) & 16777215)) * 2.2204460492503131E-16 - 1.0)
        * InverseKinematics_B.xi[InverseKinematics_B.i_e];
      if (fabs(InverseKinematics_B.b_r) <=
          InverseKinematics_B.xi[InverseKinematics_B.i_e - 1]) {
        exitg1 = 1;
      } else if (InverseKinematics_B.i_e < 256) {
        /* ========================= COPYRIGHT NOTICE ============================ */
        /*  This is a uniform (0,1) pseudorandom number generator based on:        */
        /*                                                                         */
        /*  A C-program for MT19937, with initialization improved 2002/1/26.       */
        /*  Coded by Takuji Nishimura and Makoto Matsumoto.                        */
        /*                                                                         */
        /*  Copyright (C) 1997 - 2002, Makoto Matsumoto and Takuji Nishimura,      */
        /*  All rights reserved.                                                   */
        /*                                                                         */
        /*  Redistribution and use in source and binary forms, with or without     */
        /*  modification, are permitted provided that the following conditions     */
        /*  are met:                                                               */
        /*                                                                         */
        /*    1. Redistributions of source code must retain the above copyright    */
        /*       notice, this list of conditions and the following disclaimer.     */
        /*                                                                         */
        /*    2. Redistributions in binary form must reproduce the above copyright */
        /*       notice, this list of conditions and the following disclaimer      */
        /*       in the documentation and/or other materials provided with the     */
        /*       distribution.                                                     */
        /*                                                                         */
        /*    3. The names of its contributors may not be used to endorse or       */
        /*       promote products derived from this software without specific      */
        /*       prior written permission.                                         */
        /*                                                                         */
        /*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS    */
        /*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT      */
        /*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR  */
        /*  A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT  */
        /*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,  */
        /*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT       */
        /*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,  */
        /*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY  */
        /*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT    */
        /*  (INCLUDING  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE */
        /*  OF THIS  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.  */
        /*                                                                         */
        /* =============================   END   ================================= */
        int32_T exitg2;
        do {
          exitg2 = 0;
          Inverse_genrand_uint32_vector_n(InverseKinematics_DW.state_c,
            InverseKinematics_B.u32);
          InverseKinematics_B.x = (static_cast<real_T>(InverseKinematics_B.u32[0]
            >> 5U) * 6.7108864E+7 + static_cast<real_T>(InverseKinematics_B.u32
            [1] >> 6U)) * 1.1102230246251565E-16;
          if (InverseKinematics_B.x == 0.0) {
            if (!InverseKinematic_is_valid_state(InverseKinematics_DW.state_c))
            {
              InverseKinematics_DW.state_c[0] = 5489U;
              InverseKinematics_DW.state_c[624] = 624U;
            }
          } else {
            exitg2 = 1;
          }
        } while (exitg2 == 0);

        if ((fitab[InverseKinematics_B.i_e - 1] - fitab[InverseKinematics_B.i_e])
            * InverseKinematics_B.x + fitab[InverseKinematics_B.i_e] < exp(-0.5 *
             InverseKinematics_B.b_r * InverseKinematics_B.b_r)) {
          exitg1 = 1;
        }
      } else {
        do {
          int32_T exitg2;

          /* ========================= COPYRIGHT NOTICE ============================ */
          /*  This is a uniform (0,1) pseudorandom number generator based on:        */
          /*                                                                         */
          /*  A C-program for MT19937, with initialization improved 2002/1/26.       */
          /*  Coded by Takuji Nishimura and Makoto Matsumoto.                        */
          /*                                                                         */
          /*  Copyright (C) 1997 - 2002, Makoto Matsumoto and Takuji Nishimura,      */
          /*  All rights reserved.                                                   */
          /*                                                                         */
          /*  Redistribution and use in source and binary forms, with or without     */
          /*  modification, are permitted provided that the following conditions     */
          /*  are met:                                                               */
          /*                                                                         */
          /*    1. Redistributions of source code must retain the above copyright    */
          /*       notice, this list of conditions and the following disclaimer.     */
          /*                                                                         */
          /*    2. Redistributions in binary form must reproduce the above copyright */
          /*       notice, this list of conditions and the following disclaimer      */
          /*       in the documentation and/or other materials provided with the     */
          /*       distribution.                                                     */
          /*                                                                         */
          /*    3. The names of its contributors may not be used to endorse or       */
          /*       promote products derived from this software without specific      */
          /*       prior written permission.                                         */
          /*                                                                         */
          /*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS    */
          /*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT      */
          /*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR  */
          /*  A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT  */
          /*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,  */
          /*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT       */
          /*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,  */
          /*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY  */
          /*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT    */
          /*  (INCLUDING  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE */
          /*  OF THIS  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.  */
          /*                                                                         */
          /* =============================   END   ================================= */
          do {
            exitg2 = 0;
            Inverse_genrand_uint32_vector_n(InverseKinematics_DW.state_c,
              InverseKinematics_B.u32);
            InverseKinematics_B.x = (static_cast<real_T>
              (InverseKinematics_B.u32[0] >> 5U) * 6.7108864E+7 +
              static_cast<real_T>(InverseKinematics_B.u32[1] >> 6U)) *
              1.1102230246251565E-16;
            if (InverseKinematics_B.x == 0.0) {
              if (!InverseKinematic_is_valid_state(InverseKinematics_DW.state_c))
              {
                InverseKinematics_DW.state_c[0] = 5489U;
                InverseKinematics_DW.state_c[624] = 624U;
              }
            } else {
              exitg2 = 1;
            }
          } while (exitg2 == 0);

          InverseKinematics_B.x = log(InverseKinematics_B.x) * 0.273661237329758;

          /* ========================= COPYRIGHT NOTICE ============================ */
          /*  This is a uniform (0,1) pseudorandom number generator based on:        */
          /*                                                                         */
          /*  A C-program for MT19937, with initialization improved 2002/1/26.       */
          /*  Coded by Takuji Nishimura and Makoto Matsumoto.                        */
          /*                                                                         */
          /*  Copyright (C) 1997 - 2002, Makoto Matsumoto and Takuji Nishimura,      */
          /*  All rights reserved.                                                   */
          /*                                                                         */
          /*  Redistribution and use in source and binary forms, with or without     */
          /*  modification, are permitted provided that the following conditions     */
          /*  are met:                                                               */
          /*                                                                         */
          /*    1. Redistributions of source code must retain the above copyright    */
          /*       notice, this list of conditions and the following disclaimer.     */
          /*                                                                         */
          /*    2. Redistributions in binary form must reproduce the above copyright */
          /*       notice, this list of conditions and the following disclaimer      */
          /*       in the documentation and/or other materials provided with the     */
          /*       distribution.                                                     */
          /*                                                                         */
          /*    3. The names of its contributors may not be used to endorse or       */
          /*       promote products derived from this software without specific      */
          /*       prior written permission.                                         */
          /*                                                                         */
          /*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS    */
          /*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT      */
          /*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR  */
          /*  A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT  */
          /*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,  */
          /*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT       */
          /*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,  */
          /*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY  */
          /*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT    */
          /*  (INCLUDING  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE */
          /*  OF THIS  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.  */
          /*                                                                         */
          /* =============================   END   ================================= */
          do {
            exitg2 = 0;
            Inverse_genrand_uint32_vector_n(InverseKinematics_DW.state_c,
              InverseKinematics_B.u32);
            InverseKinematics_B.d_u = (static_cast<real_T>
              (InverseKinematics_B.u32[0] >> 5U) * 6.7108864E+7 +
              static_cast<real_T>(InverseKinematics_B.u32[1] >> 6U)) *
              1.1102230246251565E-16;
            if (InverseKinematics_B.d_u == 0.0) {
              if (!InverseKinematic_is_valid_state(InverseKinematics_DW.state_c))
              {
                InverseKinematics_DW.state_c[0] = 5489U;
                InverseKinematics_DW.state_c[624] = 624U;
              }
            } else {
              exitg2 = 1;
            }
          } while (exitg2 == 0);
        } while (!(-2.0 * log(InverseKinematics_B.d_u) > InverseKinematics_B.x *
                   InverseKinematics_B.x));

        if (InverseKinematics_B.b_r < 0.0) {
          InverseKinematics_B.b_r = InverseKinematics_B.x - 3.65415288536101;
        } else {
          InverseKinematics_B.b_r = 3.65415288536101 - InverseKinematics_B.x;
        }

        exitg1 = 1;
      }
    } while (exitg1 == 0);

    r->data[InverseKinematics_B.b_k_p] = InverseKinematics_B.b_r;
  }
}

void InverseKinematics::InverseKinematics_minus(emxArray_real_T_InverseKinema_T *
  in1, const emxArray_real_T_InverseKinema_T *in2)
{
  emxArray_real_T_InverseKinema_T *in1_0;
  int32_T i;
  int32_T loop_ub;
  int32_T stride_0_0;
  int32_T stride_1_0;
  InverseKinematic_emxInit_real_T(&in1_0, 1);
  i = in1_0->size[0];
  in1_0->size[0] = in2->size[0] == 1 ? in1->size[0] : in2->size[0];
  Invers_emxEnsureCapacity_real_T(in1_0, i);
  stride_0_0 = (in1->size[0] != 1);
  stride_1_0 = (in2->size[0] != 1);
  loop_ub = in2->size[0] == 1 ? in1->size[0] : in2->size[0];
  for (i = 0; i < loop_ub; i++) {
    in1_0->data[i] = in1->data[i * stride_0_0] - in2->data[i * stride_1_0];
  }

  i = in1->size[0];
  in1->size[0] = in1_0->size[0];
  Invers_emxEnsureCapacity_real_T(in1, i);
  loop_ub = in1_0->size[0];
  if (loop_ub - 1 >= 0) {
    memcpy(&in1->data[0], &in1_0->data[0], loop_ub * sizeof(real_T));
  }

  InverseKinematic_emxFree_real_T(&in1_0);
}

void InverseKinematics::InverseKinematics_plus(emxArray_real_T_InverseKinema_T
  *in1, const emxArray_real_T_InverseKinema_T *in2)
{
  emxArray_real_T_InverseKinema_T *in2_0;
  int32_T i;
  int32_T loop_ub;
  int32_T stride_0_0;
  int32_T stride_1_0;
  InverseKinematic_emxInit_real_T(&in2_0, 1);
  i = in2_0->size[0];
  in2_0->size[0] = in1->size[0] == 1 ? in2->size[0] : in1->size[0];
  Invers_emxEnsureCapacity_real_T(in2_0, i);
  stride_0_0 = (in2->size[0] != 1);
  stride_1_0 = (in1->size[0] != 1);
  loop_ub = in1->size[0] == 1 ? in2->size[0] : in1->size[0];
  for (i = 0; i < loop_ub; i++) {
    in2_0->data[i] = in2->data[i * stride_0_0] + in1->data[i * stride_1_0];
  }

  i = in1->size[0];
  in1->size[0] = in2_0->size[0];
  Invers_emxEnsureCapacity_real_T(in1, i);
  loop_ub = in2_0->size[0];
  if (loop_ub - 1 >= 0) {
    memcpy(&in1->data[0], &in2_0->data[0], loop_ub * sizeof(real_T));
  }

  InverseKinematic_emxFree_real_T(&in2_0);
}

void InverseKinematics::InverseKinematics_rand_n(real_T varargin_1,
  emxArray_real_T_InverseKinema_T *r)
{
  int32_T b_k;
  int32_T d;
  uint32_T b_u[2];
  b_k = r->size[0];
  r->size[0] = static_cast<int32_T>(varargin_1);
  Invers_emxEnsureCapacity_real_T(r, b_k);
  d = static_cast<int32_T>(varargin_1) - 1;
  for (b_k = 0; b_k <= d; b_k++) {
    real_T b_r;

    /* ========================= COPYRIGHT NOTICE ============================ */
    /*  This is a uniform (0,1) pseudorandom number generator based on:        */
    /*                                                                         */
    /*  A C-program for MT19937, with initialization improved 2002/1/26.       */
    /*  Coded by Takuji Nishimura and Makoto Matsumoto.                        */
    /*                                                                         */
    /*  Copyright (C) 1997 - 2002, Makoto Matsumoto and Takuji Nishimura,      */
    /*  All rights reserved.                                                   */
    /*                                                                         */
    /*  Redistribution and use in source and binary forms, with or without     */
    /*  modification, are permitted provided that the following conditions     */
    /*  are met:                                                               */
    /*                                                                         */
    /*    1. Redistributions of source code must retain the above copyright    */
    /*       notice, this list of conditions and the following disclaimer.     */
    /*                                                                         */
    /*    2. Redistributions in binary form must reproduce the above copyright */
    /*       notice, this list of conditions and the following disclaimer      */
    /*       in the documentation and/or other materials provided with the     */
    /*       distribution.                                                     */
    /*                                                                         */
    /*    3. The names of its contributors may not be used to endorse or       */
    /*       promote products derived from this software without specific      */
    /*       prior written permission.                                         */
    /*                                                                         */
    /*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS    */
    /*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT      */
    /*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR  */
    /*  A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT  */
    /*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,  */
    /*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT       */
    /*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,  */
    /*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY  */
    /*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT    */
    /*  (INCLUDING  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE */
    /*  OF THIS  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.  */
    /*                                                                         */
    /* =============================   END   ================================= */
    int32_T exitg1;
    do {
      exitg1 = 0;
      Inverse_genrand_uint32_vector_n(InverseKinematics_DW.state_c, b_u);
      b_r = (static_cast<real_T>(b_u[0] >> 5U) * 6.7108864E+7 + static_cast<
             real_T>(b_u[1] >> 6U)) * 1.1102230246251565E-16;
      if (b_r == 0.0) {
        if (!InverseKinematic_is_valid_state(InverseKinematics_DW.state_c)) {
          InverseKinematics_DW.state_c[0] = 5489U;
          InverseKinematics_DW.state_c[624] = 624U;
        }
      } else {
        exitg1 = 1;
      }
    } while (exitg1 == 0);

    r->data[b_k] = b_r;
  }
}

void InverseKinematics::InverseKinemat_binary_expand_op
  (emxArray_real_T_InverseKinema_T *in1, const emxArray_real_T_InverseKinema_T
   *in2, const emxArray_real_T_InverseKinema_T *in3)
{
  emxArray_real_T_InverseKinema_T *in2_0;
  int32_T i;
  int32_T loop_ub;
  int32_T stride_0_0;
  int32_T stride_1_0;
  int32_T stride_2_0;
  int32_T stride_3_0;
  InverseKinematic_emxInit_real_T(&in2_0, 1);
  i = in2_0->size[0];
  in2_0->size[0] = ((in2->size[0] == 1 ? in1->size[0] : in2->size[0]) == 1 ?
                    in3->size[0] : in2->size[0] == 1 ? in1->size[0] : in2->size
                    [0]) == 1 ? in2->size[0] : (in2->size[0] == 1 ? in1->size[0]
    : in2->size[0]) == 1 ? in3->size[0] : in2->size[0] == 1 ? in1->size[0] :
    in2->size[0];
  Invers_emxEnsureCapacity_real_T(in2_0, i);
  stride_0_0 = (in2->size[0] != 1);
  stride_1_0 = (in3->size[0] != 1);
  stride_2_0 = (in1->size[0] != 1);
  stride_3_0 = (in2->size[0] != 1);
  loop_ub = ((in2->size[0] == 1 ? in1->size[0] : in2->size[0]) == 1 ? in3->size
             [0] : in2->size[0] == 1 ? in1->size[0] : in2->size[0]) == 1 ?
    in2->size[0] : (in2->size[0] == 1 ? in1->size[0] : in2->size[0]) == 1 ?
    in3->size[0] : in2->size[0] == 1 ? in1->size[0] : in2->size[0];
  for (i = 0; i < loop_ub; i++) {
    in2_0->data[i] = (in1->data[i * stride_2_0] - in2->data[i * stride_3_0]) *
      in3->data[i * stride_1_0] + in2->data[i * stride_0_0];
  }

  i = in1->size[0];
  in1->size[0] = in2_0->size[0];
  Invers_emxEnsureCapacity_real_T(in1, i);
  loop_ub = in2_0->size[0];
  if (loop_ub - 1 >= 0) {
    memcpy(&in1->data[0], &in2_0->data[0], loop_ub * sizeof(real_T));
  }

  InverseKinematic_emxFree_real_T(&in2_0);
}

void InverseKinematics::Invers_NLPSolverInterface_solve
  (h_robotics_core_internal_Erro_T *obj, const real_T seed[4], real_T xSol[4],
   real_T *solutionInfo_Iterations, real_T *solutionInfo_RRAttempts, real_T
   *solutionInfo_Error, real_T *solutionInfo_ExitFlag, char_T
   solutionInfo_Status_data[], int32_T solutionInfo_Status_size[2])
{
  c_rigidBodyJoint_InverseKinem_T *obj_1;
  emxArray_boolean_T_InverseKin_T *b;
  emxArray_boolean_T_InverseKin_T *tmp;
  emxArray_boolean_T_InverseKin_T *tmp_0;
  emxArray_real_T_InverseKinema_T *lb;
  emxArray_real_T_InverseKinema_T *newseed;
  emxArray_real_T_InverseKinema_T *rn;
  emxArray_real_T_InverseKinema_T *ub;
  f_robotics_manip_internal_IKE_T *args;
  v_robotics_manip_internal_Rig_T *obj_0;
  static const char_T tmp_4[14] = { 'b', 'e', 's', 't', ' ', 'a', 'v', 'a', 'i',
    'l', 'a', 'b', 'l', 'e' };

  static const char_T tmp_5[7] = { 's', 'u', 'c', 'c', 'e', 's', 's' };

  boolean_T exitg1;
  obj->MaxNumIterationInternal = obj->MaxNumIteration;
  obj->MaxTimeInternal = obj->MaxTime;
  obj->SeedInternal[0] = seed[0];
  obj->SeedInternal[1] = seed[1];
  obj->SeedInternal[2] = seed[2];
  obj->SeedInternal[3] = seed[3];
  InverseKinematics_B.tol = obj->SolutionTolerance;
  InverseKinematics_tic(&obj->TimeObj.StartTime.tv_sec,
                        &obj->TimeObj.StartTime.tv_nsec);
  ErrorDampedLevenbergMarquardt_s(obj, xSol, &InverseKinematics_B.exitFlag,
    &InverseKinematics_B.err, &InverseKinematics_B.iter);
  *solutionInfo_RRAttempts = 0.0;
  *solutionInfo_Iterations = InverseKinematics_B.iter;
  *solutionInfo_Error = InverseKinematics_B.err;
  InverseKinematics_B.exitFlagPrev = InverseKinematics_B.exitFlag;
  InverseKinematic_emxInit_real_T(&newseed, 1);
  InverseKinematic_emxInit_real_T(&ub, 1);
  InverseKinematic_emxInit_real_T(&lb, 1);
  InverseKinematic_emxInit_real_T(&rn, 1);
  InverseKinema_emxInit_boolean_T(&b, 1);
  InverseKinema_emxInit_boolean_T(&tmp, 1);
  InverseKinema_emxInit_boolean_T(&tmp_0, 1);
  exitg1 = false;
  while ((!exitg1) && (obj->RandomRestart && (InverseKinematics_B.err >
           InverseKinematics_B.tol))) {
    obj->MaxNumIterationInternal -= InverseKinematics_B.iter;
    InverseKinematics_B.err = InverseKinematics_toc
      (obj->TimeObj.StartTime.tv_sec, obj->TimeObj.StartTime.tv_nsec);
    obj->MaxTimeInternal = obj->MaxTime - InverseKinematics_B.err;
    if (obj->MaxNumIterationInternal <= 0.0) {
      InverseKinematics_B.exitFlag = IterationLimitExceeded;
    }

    if ((InverseKinematics_B.exitFlag == IterationLimitExceeded) ||
        (InverseKinematics_B.exitFlag == TimeLimitExceeded)) {
      InverseKinematics_B.exitFlagPrev = InverseKinematics_B.exitFlag;
      exitg1 = true;
    } else {
      args = obj->ExtraArgs;
      obj_0 = args->Robot;
      InverseKinematics_B.ix = newseed->size[0];
      newseed->size[0] = static_cast<int32_T>(obj_0->PositionNumber);
      Invers_emxEnsureCapacity_real_T(newseed, InverseKinematics_B.ix);
      InverseKinematics_B.nx = static_cast<int32_T>(obj_0->PositionNumber);
      if (InverseKinematics_B.nx - 1 >= 0) {
        memset(&newseed->data[0], 0, InverseKinematics_B.nx * sizeof(real_T));
      }

      InverseKinematics_B.err = obj_0->NumBodies;
      InverseKinematics_B.c_i = static_cast<int32_T>(InverseKinematics_B.err) -
        1;
      for (InverseKinematics_B.b_i = 0; InverseKinematics_B.b_i <=
           InverseKinematics_B.c_i; InverseKinematics_B.b_i++) {
        InverseKinematics_B.err = obj_0->PositionDoFMap[InverseKinematics_B.b_i];
        InverseKinematics_B.iter = obj_0->PositionDoFMap[InverseKinematics_B.b_i
          + 6];
        if (InverseKinematics_B.err <= InverseKinematics_B.iter) {
          obj_1 = obj_0->Bodies[InverseKinematics_B.b_i]->JointInternal;
          if (static_cast<int32_T>(obj_1->PositionNumber) == 0) {
            InverseKinematics_B.ix = ub->size[0];
            ub->size[0] = 1;
            Invers_emxEnsureCapacity_real_T(ub, InverseKinematics_B.ix);
            ub->data[0] = (rtNaN);
          } else {
            __m128d tmp_2;
            __m128d tmp_3;
            boolean_T exitg2;
            boolean_T guard1 = false;
            boolean_T guard2 = false;
            boolean_T guard3 = false;
            InverseKinematics_B.nx = obj_1->PositionLimitsInternal->size[0];
            InverseKinematics_B.ix = ub->size[0];
            ub->size[0] = InverseKinematics_B.nx;
            Invers_emxEnsureCapacity_real_T(ub, InverseKinematics_B.ix);
            for (InverseKinematics_B.ix = 0; InverseKinematics_B.ix <
                 InverseKinematics_B.nx; InverseKinematics_B.ix++) {
              ub->data[InverseKinematics_B.ix] = obj_1->
                PositionLimitsInternal->data[InverseKinematics_B.ix +
                obj_1->PositionLimitsInternal->size[0]];
            }

            InverseKinematics_B.nx = obj_1->PositionLimitsInternal->size[0];
            InverseKinematics_B.ix = lb->size[0];
            lb->size[0] = InverseKinematics_B.nx;
            Invers_emxEnsureCapacity_real_T(lb, InverseKinematics_B.ix);
            for (InverseKinematics_B.ix = 0; InverseKinematics_B.ix <
                 InverseKinematics_B.nx; InverseKinematics_B.ix++) {
              lb->data[InverseKinematics_B.ix] = obj_1->
                PositionLimitsInternal->data[InverseKinematics_B.ix];
            }

            InverseKinematics_B.ix = b->size[0];
            b->size[0] = lb->size[0];
            Inv_emxEnsureCapacity_boolean_T(b, InverseKinematics_B.ix);
            InverseKinematics_B.nx = lb->size[0];
            for (InverseKinematics_B.ix = 0; InverseKinematics_B.ix <
                 InverseKinematics_B.nx; InverseKinematics_B.ix++) {
              b->data[InverseKinematics_B.ix] = rtIsInf(lb->
                data[InverseKinematics_B.ix]);
            }

            InverseKinematics_B.ix = tmp->size[0];
            tmp->size[0] = lb->size[0];
            Inv_emxEnsureCapacity_boolean_T(tmp, InverseKinematics_B.ix);
            InverseKinematics_B.nx = lb->size[0];
            for (InverseKinematics_B.ix = 0; InverseKinematics_B.ix <
                 InverseKinematics_B.nx; InverseKinematics_B.ix++) {
              tmp->data[InverseKinematics_B.ix] = rtIsNaN(lb->
                data[InverseKinematics_B.ix]);
            }

            InverseKinematics_B.nx = b->size[0];
            for (InverseKinematics_B.ix = 0; InverseKinematics_B.ix <
                 InverseKinematics_B.nx; InverseKinematics_B.ix++) {
              b->data[InverseKinematics_B.ix] = ((!b->
                data[InverseKinematics_B.ix]) && (!tmp->
                data[InverseKinematics_B.ix]));
            }

            InverseKinematics_B.y_f = true;
            InverseKinematics_B.ix = 0;
            exitg2 = false;
            while ((!exitg2) && (InverseKinematics_B.ix + 1 <= b->size[0])) {
              if (!b->data[InverseKinematics_B.ix]) {
                InverseKinematics_B.y_f = false;
                exitg2 = true;
              } else {
                InverseKinematics_B.ix++;
              }
            }

            guard1 = false;
            guard2 = false;
            guard3 = false;
            if (InverseKinematics_B.y_f) {
              InverseKinematics_B.ix = b->size[0];
              b->size[0] = ub->size[0];
              Inv_emxEnsureCapacity_boolean_T(b, InverseKinematics_B.ix);
              InverseKinematics_B.nx = ub->size[0];
              for (InverseKinematics_B.ix = 0; InverseKinematics_B.ix <
                   InverseKinematics_B.nx; InverseKinematics_B.ix++) {
                b->data[InverseKinematics_B.ix] = rtIsInf(ub->
                  data[InverseKinematics_B.ix]);
              }

              InverseKinematics_B.ix = tmp->size[0];
              tmp->size[0] = ub->size[0];
              Inv_emxEnsureCapacity_boolean_T(tmp, InverseKinematics_B.ix);
              InverseKinematics_B.nx = ub->size[0];
              for (InverseKinematics_B.ix = 0; InverseKinematics_B.ix <
                   InverseKinematics_B.nx; InverseKinematics_B.ix++) {
                tmp->data[InverseKinematics_B.ix] = rtIsNaN(ub->
                  data[InverseKinematics_B.ix]);
              }

              InverseKinematics_B.nx = b->size[0];
              for (InverseKinematics_B.ix = 0; InverseKinematics_B.ix <
                   InverseKinematics_B.nx; InverseKinematics_B.ix++) {
                b->data[InverseKinematics_B.ix] = ((!b->
                  data[InverseKinematics_B.ix]) && (!tmp->
                  data[InverseKinematics_B.ix]));
              }

              InverseKinematics_B.y_f = true;
              InverseKinematics_B.ix = 0;
              exitg2 = false;
              while ((!exitg2) && (InverseKinematics_B.ix + 1 <= b->size[0])) {
                if (!b->data[InverseKinematics_B.ix]) {
                  InverseKinematics_B.y_f = false;
                  exitg2 = true;
                } else {
                  InverseKinematics_B.ix++;
                }
              }

              if (InverseKinematics_B.y_f) {
                InverseKinematics_rand_n(obj_1->PositionNumber, rn);
                if ((ub->size[0] == lb->size[0]) && ((ub->size[0] == 1 ?
                      lb->size[0] : ub->size[0]) == rn->size[0]) && ((rn->size[0]
                      == 1 ? ub->size[0] == 1 ? lb->size[0] : ub->size[0] :
                      rn->size[0]) == lb->size[0])) {
                  InverseKinematics_B.ix = ub->size[0];
                  ub->size[0] = lb->size[0];
                  Invers_emxEnsureCapacity_real_T(ub, InverseKinematics_B.ix);
                  InverseKinematics_B.nx = lb->size[0];
                  InverseKinematics_B.scalarLB = (lb->size[0] / 2) << 1;
                  InverseKinematics_B.vectorUB = InverseKinematics_B.scalarLB -
                    2;
                  for (InverseKinematics_B.ix = 0; InverseKinematics_B.ix <=
                       InverseKinematics_B.vectorUB; InverseKinematics_B.ix += 2)
                  {
                    __m128d tmp_1;
                    tmp_2 = _mm_loadu_pd(&ub->data[InverseKinematics_B.ix]);
                    tmp_3 = _mm_loadu_pd(&lb->data[InverseKinematics_B.ix]);
                    tmp_1 = _mm_loadu_pd(&rn->data[InverseKinematics_B.ix]);
                    _mm_storeu_pd(&ub->data[InverseKinematics_B.ix], _mm_add_pd
                                  (_mm_mul_pd(_mm_sub_pd(tmp_2, tmp_3), tmp_1),
                                   tmp_3));
                  }

                  for (InverseKinematics_B.ix = InverseKinematics_B.scalarLB;
                       InverseKinematics_B.ix < InverseKinematics_B.nx;
                       InverseKinematics_B.ix++) {
                    InverseKinematics_B.lb = lb->data[InverseKinematics_B.ix];
                    ub->data[InverseKinematics_B.ix] = (ub->
                      data[InverseKinematics_B.ix] - InverseKinematics_B.lb) *
                      rn->data[InverseKinematics_B.ix] + InverseKinematics_B.lb;
                  }
                } else {
                  InverseKinemat_binary_expand_op(ub, lb, rn);
                }
              } else {
                guard3 = true;
              }
            } else {
              guard3 = true;
            }

            if (guard3) {
              InverseKinematics_B.ix = b->size[0];
              b->size[0] = lb->size[0];
              Inv_emxEnsureCapacity_boolean_T(b, InverseKinematics_B.ix);
              InverseKinematics_B.nx = lb->size[0];
              for (InverseKinematics_B.ix = 0; InverseKinematics_B.ix <
                   InverseKinematics_B.nx; InverseKinematics_B.ix++) {
                b->data[InverseKinematics_B.ix] = rtIsInf(lb->
                  data[InverseKinematics_B.ix]);
              }

              InverseKinematics_B.ix = tmp->size[0];
              tmp->size[0] = lb->size[0];
              Inv_emxEnsureCapacity_boolean_T(tmp, InverseKinematics_B.ix);
              InverseKinematics_B.nx = lb->size[0];
              for (InverseKinematics_B.ix = 0; InverseKinematics_B.ix <
                   InverseKinematics_B.nx; InverseKinematics_B.ix++) {
                tmp->data[InverseKinematics_B.ix] = rtIsNaN(lb->
                  data[InverseKinematics_B.ix]);
              }

              InverseKinematics_B.nx = b->size[0];
              for (InverseKinematics_B.ix = 0; InverseKinematics_B.ix <
                   InverseKinematics_B.nx; InverseKinematics_B.ix++) {
                b->data[InverseKinematics_B.ix] = ((!b->
                  data[InverseKinematics_B.ix]) && (!tmp->
                  data[InverseKinematics_B.ix]));
              }

              InverseKinematics_B.y_f = true;
              InverseKinematics_B.ix = 0;
              exitg2 = false;
              while ((!exitg2) && (InverseKinematics_B.ix + 1 <= b->size[0])) {
                if (!b->data[InverseKinematics_B.ix]) {
                  InverseKinematics_B.y_f = false;
                  exitg2 = true;
                } else {
                  InverseKinematics_B.ix++;
                }
              }

              if (InverseKinematics_B.y_f) {
                InverseKinematics_B.ix = tmp->size[0];
                tmp->size[0] = ub->size[0];
                Inv_emxEnsureCapacity_boolean_T(tmp, InverseKinematics_B.ix);
                InverseKinematics_B.nx = ub->size[0];
                for (InverseKinematics_B.ix = 0; InverseKinematics_B.ix <
                     InverseKinematics_B.nx; InverseKinematics_B.ix++) {
                  tmp->data[InverseKinematics_B.ix] = rtIsInf(ub->
                    data[InverseKinematics_B.ix]);
                }

                InverseKinematics_B.ix = b->size[0];
                b->size[0] = ub->size[0];
                Inv_emxEnsureCapacity_boolean_T(b, InverseKinematics_B.ix);
                InverseKinematics_B.nx = ub->size[0];
                for (InverseKinematics_B.ix = 0; InverseKinematics_B.ix <
                     InverseKinematics_B.nx; InverseKinematics_B.ix++) {
                  b->data[InverseKinematics_B.ix] = rtIsNaN(ub->
                    data[InverseKinematics_B.ix]);
                }

                InverseKinematics_B.ix = tmp_0->size[0];
                tmp_0->size[0] = tmp->size[0];
                Inv_emxEnsureCapacity_boolean_T(tmp_0, InverseKinematics_B.ix);
                InverseKinematics_B.nx = tmp->size[0];
                for (InverseKinematics_B.ix = 0; InverseKinematics_B.ix <
                     InverseKinematics_B.nx; InverseKinematics_B.ix++) {
                  tmp_0->data[InverseKinematics_B.ix] = (tmp->
                    data[InverseKinematics_B.ix] || b->
                    data[InverseKinematics_B.ix]);
                }

                if (InverseKinematics_any(tmp_0)) {
                  InverseKinematics_B.ub[0] = lb->size[0];
                  InverseKinematics_B.ub[1] = 1.0;
                  InverseKinematics_randn(InverseKinematics_B.ub, rn);
                  InverseKinematics_B.nx = rn->size[0] - 1;
                  InverseKinematics_B.ix = ub->size[0];
                  ub->size[0] = rn->size[0];
                  Invers_emxEnsureCapacity_real_T(ub, InverseKinematics_B.ix);
                  for (InverseKinematics_B.ix = 0; InverseKinematics_B.ix <=
                       InverseKinematics_B.nx; InverseKinematics_B.ix++) {
                    ub->data[InverseKinematics_B.ix] = fabs(rn->
                      data[InverseKinematics_B.ix]);
                  }

                  if (lb->size[0] == ub->size[0]) {
                    InverseKinematics_B.ix = ub->size[0];
                    ub->size[0] = lb->size[0];
                    Invers_emxEnsureCapacity_real_T(ub, InverseKinematics_B.ix);
                    InverseKinematics_B.nx = lb->size[0];
                    InverseKinematics_B.scalarLB = (lb->size[0] / 2) << 1;
                    InverseKinematics_B.vectorUB = InverseKinematics_B.scalarLB
                      - 2;
                    for (InverseKinematics_B.ix = 0; InverseKinematics_B.ix <=
                         InverseKinematics_B.vectorUB; InverseKinematics_B.ix +=
                         2) {
                      tmp_2 = _mm_loadu_pd(&lb->data[InverseKinematics_B.ix]);
                      tmp_3 = _mm_loadu_pd(&ub->data[InverseKinematics_B.ix]);
                      _mm_storeu_pd(&ub->data[InverseKinematics_B.ix],
                                    _mm_add_pd(tmp_2, tmp_3));
                    }

                    for (InverseKinematics_B.ix = InverseKinematics_B.scalarLB;
                         InverseKinematics_B.ix < InverseKinematics_B.nx;
                         InverseKinematics_B.ix++) {
                      ub->data[InverseKinematics_B.ix] += lb->
                        data[InverseKinematics_B.ix];
                    }
                  } else {
                    InverseKinematics_plus(ub, lb);
                  }
                } else {
                  guard2 = true;
                }
              } else {
                guard2 = true;
              }
            }

            if (guard2) {
              InverseKinematics_B.ix = tmp->size[0];
              tmp->size[0] = lb->size[0];
              Inv_emxEnsureCapacity_boolean_T(tmp, InverseKinematics_B.ix);
              InverseKinematics_B.nx = lb->size[0];
              for (InverseKinematics_B.ix = 0; InverseKinematics_B.ix <
                   InverseKinematics_B.nx; InverseKinematics_B.ix++) {
                tmp->data[InverseKinematics_B.ix] = rtIsInf(lb->
                  data[InverseKinematics_B.ix]);
              }

              InverseKinematics_B.ix = b->size[0];
              b->size[0] = lb->size[0];
              Inv_emxEnsureCapacity_boolean_T(b, InverseKinematics_B.ix);
              InverseKinematics_B.nx = lb->size[0];
              for (InverseKinematics_B.ix = 0; InverseKinematics_B.ix <
                   InverseKinematics_B.nx; InverseKinematics_B.ix++) {
                b->data[InverseKinematics_B.ix] = rtIsNaN(lb->
                  data[InverseKinematics_B.ix]);
              }

              InverseKinematics_B.ix = tmp_0->size[0];
              tmp_0->size[0] = tmp->size[0];
              Inv_emxEnsureCapacity_boolean_T(tmp_0, InverseKinematics_B.ix);
              InverseKinematics_B.nx = tmp->size[0];
              for (InverseKinematics_B.ix = 0; InverseKinematics_B.ix <
                   InverseKinematics_B.nx; InverseKinematics_B.ix++) {
                tmp_0->data[InverseKinematics_B.ix] = (tmp->
                  data[InverseKinematics_B.ix] || b->data[InverseKinematics_B.ix]);
              }

              if (InverseKinematics_any(tmp_0)) {
                InverseKinematics_B.ix = b->size[0];
                b->size[0] = ub->size[0];
                Inv_emxEnsureCapacity_boolean_T(b, InverseKinematics_B.ix);
                InverseKinematics_B.nx = ub->size[0];
                for (InverseKinematics_B.ix = 0; InverseKinematics_B.ix <
                     InverseKinematics_B.nx; InverseKinematics_B.ix++) {
                  b->data[InverseKinematics_B.ix] = rtIsInf(ub->
                    data[InverseKinematics_B.ix]);
                }

                InverseKinematics_B.ix = tmp->size[0];
                tmp->size[0] = ub->size[0];
                Inv_emxEnsureCapacity_boolean_T(tmp, InverseKinematics_B.ix);
                InverseKinematics_B.nx = ub->size[0];
                for (InverseKinematics_B.ix = 0; InverseKinematics_B.ix <
                     InverseKinematics_B.nx; InverseKinematics_B.ix++) {
                  tmp->data[InverseKinematics_B.ix] = rtIsNaN(ub->
                    data[InverseKinematics_B.ix]);
                }

                InverseKinematics_B.nx = b->size[0];
                for (InverseKinematics_B.ix = 0; InverseKinematics_B.ix <
                     InverseKinematics_B.nx; InverseKinematics_B.ix++) {
                  b->data[InverseKinematics_B.ix] = ((!b->
                    data[InverseKinematics_B.ix]) && (!tmp->
                    data[InverseKinematics_B.ix]));
                }

                InverseKinematics_B.y_f = true;
                InverseKinematics_B.ix = 0;
                exitg2 = false;
                while ((!exitg2) && (InverseKinematics_B.ix + 1 <= b->size[0]))
                {
                  if (!b->data[InverseKinematics_B.ix]) {
                    InverseKinematics_B.y_f = false;
                    exitg2 = true;
                  } else {
                    InverseKinematics_B.ix++;
                  }
                }

                if (InverseKinematics_B.y_f) {
                  InverseKinematics_B.ub[0] = ub->size[0];
                  InverseKinematics_B.ub[1] = 1.0;
                  InverseKinematics_randn(InverseKinematics_B.ub, rn);
                  InverseKinematics_B.nx = rn->size[0] - 1;
                  InverseKinematics_B.ix = lb->size[0];
                  lb->size[0] = rn->size[0];
                  Invers_emxEnsureCapacity_real_T(lb, InverseKinematics_B.ix);
                  for (InverseKinematics_B.ix = 0; InverseKinematics_B.ix <=
                       InverseKinematics_B.nx; InverseKinematics_B.ix++) {
                    lb->data[InverseKinematics_B.ix] = fabs(rn->
                      data[InverseKinematics_B.ix]);
                  }

                  if (ub->size[0] == lb->size[0]) {
                    InverseKinematics_B.nx = ub->size[0];
                    InverseKinematics_B.scalarLB = (ub->size[0] / 2) << 1;
                    InverseKinematics_B.vectorUB = InverseKinematics_B.scalarLB
                      - 2;
                    for (InverseKinematics_B.ix = 0; InverseKinematics_B.ix <=
                         InverseKinematics_B.vectorUB; InverseKinematics_B.ix +=
                         2) {
                      tmp_2 = _mm_loadu_pd(&ub->data[InverseKinematics_B.ix]);
                      tmp_3 = _mm_loadu_pd(&lb->data[InverseKinematics_B.ix]);
                      _mm_storeu_pd(&ub->data[InverseKinematics_B.ix],
                                    _mm_sub_pd(tmp_2, tmp_3));
                    }

                    for (InverseKinematics_B.ix = InverseKinematics_B.scalarLB;
                         InverseKinematics_B.ix < InverseKinematics_B.nx;
                         InverseKinematics_B.ix++) {
                      ub->data[InverseKinematics_B.ix] -= lb->
                        data[InverseKinematics_B.ix];
                    }
                  } else {
                    InverseKinematics_minus(ub, lb);
                  }
                } else {
                  guard1 = true;
                }
              } else {
                guard1 = true;
              }
            }

            if (guard1) {
              InverseKinematics_B.ub[0] = ub->size[0];
              InverseKinematics_B.ub[1] = 1.0;
              InverseKinematics_randn(InverseKinematics_B.ub, ub);
            }
          }

          if (InverseKinematics_B.err > InverseKinematics_B.iter) {
            InverseKinematics_B.nx = 0;
            InverseKinematics_B.ix = 0;
          } else {
            InverseKinematics_B.nx = static_cast<int32_T>
              (InverseKinematics_B.err) - 1;
            InverseKinematics_B.ix = static_cast<int32_T>
              (InverseKinematics_B.iter);
          }

          InverseKinematics_B.scalarLB = InverseKinematics_B.ix -
            InverseKinematics_B.nx;
          for (InverseKinematics_B.ix = 0; InverseKinematics_B.ix <
               InverseKinematics_B.scalarLB; InverseKinematics_B.ix++) {
            newseed->data[InverseKinematics_B.nx + InverseKinematics_B.ix] =
              ub->data[InverseKinematics_B.ix];
          }
        }
      }

      obj->SeedInternal[0] = newseed->data[0];
      obj->SeedInternal[1] = newseed->data[1];
      obj->SeedInternal[2] = newseed->data[2];
      obj->SeedInternal[3] = newseed->data[3];
      ErrorDampedLevenbergMarquardt_s(obj, InverseKinematics_B.c_xSol,
        &InverseKinematics_B.exitFlag, &InverseKinematics_B.err,
        &InverseKinematics_B.iter);
      if (InverseKinematics_B.err < *solutionInfo_Error) {
        xSol[0] = InverseKinematics_B.c_xSol[0];
        xSol[1] = InverseKinematics_B.c_xSol[1];
        xSol[2] = InverseKinematics_B.c_xSol[2];
        xSol[3] = InverseKinematics_B.c_xSol[3];
        *solutionInfo_Error = InverseKinematics_B.err;
        InverseKinematics_B.exitFlagPrev = InverseKinematics_B.exitFlag;
      }

      (*solutionInfo_RRAttempts)++;
      *solutionInfo_Iterations += InverseKinematics_B.iter;
    }
  }

  InverseKinema_emxFree_boolean_T(&tmp_0);
  InverseKinema_emxFree_boolean_T(&tmp);
  InverseKinema_emxFree_boolean_T(&b);
  InverseKinematic_emxFree_real_T(&rn);
  InverseKinematic_emxFree_real_T(&lb);
  InverseKinematic_emxFree_real_T(&ub);
  InverseKinematic_emxFree_real_T(&newseed);
  *solutionInfo_ExitFlag = InverseKinematics_B.exitFlagPrev;
  if (*solutionInfo_Error < InverseKinematics_B.tol) {
    solutionInfo_Status_size[0] = 1;
    solutionInfo_Status_size[1] = 7;
    for (InverseKinematics_B.ix = 0; InverseKinematics_B.ix < 7;
         InverseKinematics_B.ix++) {
      solutionInfo_Status_data[InverseKinematics_B.ix] =
        tmp_5[InverseKinematics_B.ix];
    }
  } else {
    solutionInfo_Status_size[0] = 1;
    solutionInfo_Status_size[1] = 14;
    for (InverseKinematics_B.ix = 0; InverseKinematics_B.ix < 14;
         InverseKinematics_B.ix++) {
      solutionInfo_Status_data[InverseKinematics_B.ix] =
        tmp_4[InverseKinematics_B.ix];
    }
  }
}

void InverseKinematics::InverseKinemati_emxInit_int32_T
  (emxArray_int32_T_InverseKinem_T **pEmxArray, int32_T numDimensions)
{
  emxArray_int32_T_InverseKinem_T *emxArray;
  *pEmxArray = static_cast<emxArray_int32_T_InverseKinem_T *>(malloc(sizeof
    (emxArray_int32_T_InverseKinem_T)));
  emxArray = *pEmxArray;
  emxArray->data = static_cast<int32_T *>(NULL);
  emxArray->numDimensions = numDimensions;
  emxArray->size = static_cast<int32_T *>(malloc(sizeof(int32_T) * numDimensions));
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (int32_T i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

void InverseKinematics::Inver_emxEnsureCapacity_int32_T
  (emxArray_int32_T_InverseKinem_T *emxArray, int32_T oldNumel)
{
  int32_T i;
  int32_T newNumel;
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  newNumel = 1;
  for (i = 0; i < emxArray->numDimensions; i++) {
    newNumel *= emxArray->size[i];
  }

  if (newNumel > emxArray->allocatedSize) {
    i = emxArray->allocatedSize;
    if (i < 16) {
      i = 16;
    }

    while (i < newNumel) {
      if (i > 1073741823) {
        i = MAX_int32_T;
      } else {
        i <<= 1;
      }
    }

    newData = calloc(static_cast<uint32_T>(i), sizeof(int32_T));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(int32_T) * oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }

    emxArray->data = static_cast<int32_T *>(newData);
    emxArray->allocatedSize = i;
    emxArray->canFreeData = true;
  }
}

void InverseKinematics::InverseKinemati_emxFree_int32_T
  (emxArray_int32_T_InverseKinem_T **pEmxArray)
{
  if (*pEmxArray != static_cast<emxArray_int32_T_InverseKinem_T *>(NULL)) {
    if (((*pEmxArray)->data != static_cast<int32_T *>(NULL)) && (*pEmxArray)
        ->canFreeData) {
      free((*pEmxArray)->data);
    }

    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = static_cast<emxArray_int32_T_InverseKinem_T *>(NULL);
  }
}

void InverseKinematics::InverseKinemat_emxInit_uint32_T
  (emxArray_uint32_T_InverseKine_T **pEmxArray, int32_T numDimensions)
{
  emxArray_uint32_T_InverseKine_T *emxArray;
  *pEmxArray = static_cast<emxArray_uint32_T_InverseKine_T *>(malloc(sizeof
    (emxArray_uint32_T_InverseKine_T)));
  emxArray = *pEmxArray;
  emxArray->data = static_cast<uint32_T *>(NULL);
  emxArray->numDimensions = numDimensions;
  emxArray->size = static_cast<int32_T *>(malloc(sizeof(int32_T) * numDimensions));
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (int32_T i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

void InverseKinematics::Inve_emxEnsureCapacity_uint32_T
  (emxArray_uint32_T_InverseKine_T *emxArray, int32_T oldNumel)
{
  int32_T i;
  int32_T newNumel;
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  newNumel = 1;
  for (i = 0; i < emxArray->numDimensions; i++) {
    newNumel *= emxArray->size[i];
  }

  if (newNumel > emxArray->allocatedSize) {
    i = emxArray->allocatedSize;
    if (i < 16) {
      i = 16;
    }

    while (i < newNumel) {
      if (i > 1073741823) {
        i = MAX_int32_T;
      } else {
        i <<= 1;
      }
    }

    newData = calloc(static_cast<uint32_T>(i), sizeof(uint32_T));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(uint32_T) * oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }

    emxArray->data = static_cast<uint32_T *>(newData);
    emxArray->allocatedSize = i;
    emxArray->canFreeData = true;
  }
}

void InverseKinematics::InverseKinemat_emxFree_uint32_T
  (emxArray_uint32_T_InverseKine_T **pEmxArray)
{
  if (*pEmxArray != static_cast<emxArray_uint32_T_InverseKine_T *>(NULL)) {
    if (((*pEmxArray)->data != static_cast<uint32_T *>(NULL)) && (*pEmxArray)
        ->canFreeData) {
      free((*pEmxArray)->data);
    }

    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = static_cast<emxArray_uint32_T_InverseKine_T *>(NULL);
  }
}

void InverseKinematics::Inver_inverseKinematics_solve_n
  (b_inverseKinematics_InverseKi_T *obj, real_T initialGuess[4], real_T
   *solutionInfo_Iterations, real_T *solutionInfo_NumRandomRestarts, real_T
   *solutionInfo_PoseErrorNorm, real_T *solutionInfo_ExitFlag, char_T
   solutionInfo_Status_data[], int32_T solutionInfo_Status_size[2])
{
  emxArray_char_T_InverseKinema_T *endEffectorName;
  emxArray_int32_T_InverseKinem_T *h;
  emxArray_real_T_InverseKinema_T *bodyIndices;
  emxArray_real_T_InverseKinema_T *e;
  emxArray_real_T_InverseKinema_T *limits;
  emxArray_real_T_InverseKinema_T *positionIndices;
  emxArray_uint32_T_InverseKine_T *y;
  t_robotics_manip_internal_Rig_T *body;
  v_robotics_manip_internal_Rig_T *obj_0;
  boolean_T exitg1;
  boolean_T guard1 = false;
  InverseKinematic_emxInit_real_T(&limits, 2);
  obj_0 = obj->RigidBodyTreeInternal;
  RigidBodyTree_get_JointPosition(obj_0, limits);
  if (limits->size[0] == 4) {
    InverseKinematics_B.ubOK[0] = (initialGuess[0] <= limits->data[limits->size
      [0]] + 4.4408920985006262E-16);
    InverseKinematics_B.ubOK[1] = (initialGuess[1] <= limits->data[1 +
      limits->size[0]] + 4.4408920985006262E-16);
    InverseKinematics_B.ubOK[2] = (initialGuess[2] <= limits->data[2 +
      limits->size[0]] + 4.4408920985006262E-16);
    InverseKinematics_B.ubOK[3] = (initialGuess[3] <= limits->data[3 +
      limits->size[0]] + 4.4408920985006262E-16);
  } else {
    InverseKin_binary_expand_op_nly(InverseKinematics_B.ubOK, initialGuess,
      limits);
  }

  if (limits->size[0] == 4) {
    InverseKinematics_B.lbOK[0] = (initialGuess[0] >= limits->data[0] -
      4.4408920985006262E-16);
    InverseKinematics_B.lbOK[1] = (initialGuess[1] >= limits->data[1] -
      4.4408920985006262E-16);
    InverseKinematics_B.lbOK[2] = (initialGuess[2] >= limits->data[2] -
      4.4408920985006262E-16);
    InverseKinematics_B.lbOK[3] = (initialGuess[3] >= limits->data[3] -
      4.4408920985006262E-16);
  } else {
    InverseKine_binary_expand_op_nl(InverseKinematics_B.lbOK, initialGuess,
      limits);
  }

  InverseKinematics_B.y_c = true;
  InverseKinematics_B.b_k = 0;
  exitg1 = false;
  while ((!exitg1) && (InverseKinematics_B.b_k < 4)) {
    if (!InverseKinematics_B.ubOK[InverseKinematics_B.b_k]) {
      InverseKinematics_B.y_c = false;
      exitg1 = true;
    } else {
      InverseKinematics_B.b_k++;
    }
  }

  guard1 = false;
  if (InverseKinematics_B.y_c) {
    InverseKinematics_B.y_c = true;
    InverseKinematics_B.b_k = 0;
    exitg1 = false;
    while ((!exitg1) && (InverseKinematics_B.b_k < 4)) {
      if (!InverseKinematics_B.lbOK[InverseKinematics_B.b_k]) {
        InverseKinematics_B.y_c = false;
        exitg1 = true;
      } else {
        InverseKinematics_B.b_k++;
      }
    }

    if (InverseKinematics_B.y_c) {
    } else {
      guard1 = true;
    }
  } else {
    guard1 = true;
  }

  if (guard1) {
    InverseKinematics_B.ubOK_a[0] = !InverseKinematics_B.ubOK[0];
    InverseKinematics_B.ubOK_a[1] = !InverseKinematics_B.ubOK[1];
    InverseKinematics_B.ubOK_a[2] = !InverseKinematics_B.ubOK[2];
    InverseKinematics_B.ubOK_a[3] = !InverseKinematics_B.ubOK[3];
    InverseKinematics_eml_find(InverseKinematics_B.ubOK_a,
      InverseKinematics_B.tmp_data, &InverseKinematics_B.tmp_size);
    InverseKinematics_B.indicesUpperBoundViolation_size =
      InverseKinematics_B.tmp_size;
    InverseKinematics_B.loop_ub_o = InverseKinematics_B.tmp_size;
    if (InverseKinematics_B.loop_ub_o - 1 >= 0) {
      memcpy(&InverseKinematics_B.indicesUpperBoundViolation_data[0],
             &InverseKinematics_B.tmp_data[0], InverseKinematics_B.loop_ub_o *
             sizeof(int32_T));
    }

    for (InverseKinematics_B.b_k = 0; InverseKinematics_B.b_k <
         InverseKinematics_B.indicesUpperBoundViolation_size;
         InverseKinematics_B.b_k++) {
      InverseKinematics_B.indicesUpperBoundViolation =
        InverseKinematics_B.indicesUpperBoundViolation_data[InverseKinematics_B.b_k];
      initialGuess[InverseKinematics_B.indicesUpperBoundViolation - 1] =
        limits->data[(InverseKinematics_B.indicesUpperBoundViolation +
                      limits->size[0]) - 1];
    }

    InverseKinematics_B.ubOK[0] = !InverseKinematics_B.lbOK[0];
    InverseKinematics_B.ubOK[1] = !InverseKinematics_B.lbOK[1];
    InverseKinematics_B.ubOK[2] = !InverseKinematics_B.lbOK[2];
    InverseKinematics_B.ubOK[3] = !InverseKinematics_B.lbOK[3];
    InverseKinematics_eml_find(InverseKinematics_B.ubOK,
      InverseKinematics_B.tmp_data, &InverseKinematics_B.tmp_size);
    InverseKinematics_B.indicesUpperBoundViolation_size =
      InverseKinematics_B.tmp_size;
    InverseKinematics_B.loop_ub_o = InverseKinematics_B.tmp_size;
    if (InverseKinematics_B.loop_ub_o - 1 >= 0) {
      memcpy(&InverseKinematics_B.indicesUpperBoundViolation_data[0],
             &InverseKinematics_B.tmp_data[0], InverseKinematics_B.loop_ub_o *
             sizeof(int32_T));
    }

    for (InverseKinematics_B.b_k = 0; InverseKinematics_B.b_k <
         InverseKinematics_B.indicesUpperBoundViolation_size;
         InverseKinematics_B.b_k++) {
      InverseKinematics_B.indicesUpperBoundViolation =
        InverseKinematics_B.indicesUpperBoundViolation_data[InverseKinematics_B.b_k];
      initialGuess[InverseKinematics_B.indicesUpperBoundViolation - 1] =
        limits->data[InverseKinematics_B.indicesUpperBoundViolation - 1];
    }
  }

  InverseKinematic_emxInit_char_T(&endEffectorName, 2);
  Invers_NLPSolverInterface_solve(obj->Solver, initialGuess,
    InverseKinematics_B.qvSolRaw, solutionInfo_Iterations,
    solutionInfo_NumRandomRestarts, solutionInfo_PoseErrorNorm,
    solutionInfo_ExitFlag, solutionInfo_Status_data, solutionInfo_Status_size);
  obj_0 = obj->RigidBodyTreeInternal;
  InverseKinematics_B.nm1d2 = endEffectorName->size[0] * endEffectorName->size[1];
  endEffectorName->size[0] = 1;
  endEffectorName->size[1] = obj->Solver->ExtraArgs->BodyName->size[1];
  Invers_emxEnsureCapacity_char_T(endEffectorName, InverseKinematics_B.nm1d2);
  InverseKinematics_B.loop_ub_o = obj->Solver->ExtraArgs->BodyName->size[1] - 1;
  for (InverseKinematics_B.b_k = 0; InverseKinematics_B.b_k <=
       InverseKinematics_B.loop_ub_o; InverseKinematics_B.b_k++) {
    InverseKinematics_B.nm1d2 = InverseKinematics_B.b_k;
    endEffectorName->data[InverseKinematics_B.nm1d2] = obj->Solver->
      ExtraArgs->BodyName->data[InverseKinematics_B.nm1d2];
  }

  InverseKinematic_emxInit_real_T(&bodyIndices, 1);
  InverseKinematics_B.nm1d2 = bodyIndices->size[0];
  bodyIndices->size[0] = static_cast<int32_T>(obj_0->NumBodies);
  Invers_emxEnsureCapacity_real_T(bodyIndices, InverseKinematics_B.nm1d2);
  InverseKinematics_B.loop_ub_o = static_cast<int32_T>(obj_0->NumBodies);
  if (InverseKinematics_B.loop_ub_o - 1 >= 0) {
    memset(&bodyIndices->data[0], 0, InverseKinematics_B.loop_ub_o * sizeof
           (real_T));
  }

  InverseKinematics_B.bid = RigidBodyTree_findBodyIndexByNa(obj_0,
    endEffectorName);
  InverseKinematic_emxFree_char_T(&endEffectorName);
  if (InverseKinematics_B.bid == 0.0) {
    InverseKinematics_B.nm1d2 = bodyIndices->size[0];
    bodyIndices->size[0] = 1;
    Invers_emxEnsureCapacity_real_T(bodyIndices, InverseKinematics_B.nm1d2);
    bodyIndices->data[0] = 0.0;
  } else {
    body = obj_0->Bodies[static_cast<int32_T>(InverseKinematics_B.bid) - 1];
    InverseKinematics_B.bid = 1.0;
    while (body->ParentIndex != 0.0) {
      bodyIndices->data[static_cast<int32_T>(InverseKinematics_B.bid) - 1] =
        body->Index;
      body = obj_0->Bodies[static_cast<int32_T>(body->ParentIndex) - 1];
      InverseKinematics_B.bid++;
    }

    if (InverseKinematics_B.bid - 1.0 < 1.0) {
      InverseKinematics_B.indicesUpperBoundViolation_size = -1;
    } else {
      InverseKinematics_B.indicesUpperBoundViolation_size = static_cast<int32_T>
        (InverseKinematics_B.bid - 1.0) - 1;
    }

    InverseKinematics_B.nm1d2 = bodyIndices->size[0];
    bodyIndices->size[0] = InverseKinematics_B.indicesUpperBoundViolation_size +
      3;
    Invers_emxEnsureCapacity_real_T(bodyIndices, InverseKinematics_B.nm1d2);
    bodyIndices->data[InverseKinematics_B.indicesUpperBoundViolation_size + 1] =
      body->Index;
    bodyIndices->data[InverseKinematics_B.indicesUpperBoundViolation_size + 2] =
      0.0;
  }

  obj_0 = obj->RigidBodyTreeInternal;
  InverseKinematics_B.b_k = bodyIndices->size[0] - 1;
  InverseKinematics_B.indicesUpperBoundViolation_size = 0;
  for (InverseKinematics_B.indicesUpperBoundViolation = 0;
       InverseKinematics_B.indicesUpperBoundViolation <= InverseKinematics_B.b_k;
       InverseKinematics_B.indicesUpperBoundViolation++) {
    if (bodyIndices->data[InverseKinematics_B.indicesUpperBoundViolation] != 0.0)
    {
      InverseKinematics_B.indicesUpperBoundViolation_size++;
    }
  }

  InverseKinemati_emxInit_int32_T(&h, 1);
  InverseKinematics_B.nm1d2 = h->size[0];
  h->size[0] = InverseKinematics_B.indicesUpperBoundViolation_size;
  Inver_emxEnsureCapacity_int32_T(h, InverseKinematics_B.nm1d2);
  InverseKinematics_B.indicesUpperBoundViolation_size = 0;
  for (InverseKinematics_B.indicesUpperBoundViolation = 0;
       InverseKinematics_B.indicesUpperBoundViolation <= InverseKinematics_B.b_k;
       InverseKinematics_B.indicesUpperBoundViolation++) {
    if (bodyIndices->data[InverseKinematics_B.indicesUpperBoundViolation] != 0.0)
    {
      h->data[InverseKinematics_B.indicesUpperBoundViolation_size] =
        InverseKinematics_B.indicesUpperBoundViolation + 1;
      InverseKinematics_B.indicesUpperBoundViolation_size++;
    }
  }

  InverseKinematics_B.nm1d2 = limits->size[0] * limits->size[1];
  limits->size[0] = h->size[0];
  limits->size[1] = 2;
  Invers_emxEnsureCapacity_real_T(limits, InverseKinematics_B.nm1d2);
  InverseKinematics_B.loop_ub_o = h->size[0];
  for (InverseKinematics_B.b_k = 0; InverseKinematics_B.b_k < 2;
       InverseKinematics_B.b_k++) {
    for (InverseKinematics_B.indicesUpperBoundViolation = 0;
         InverseKinematics_B.indicesUpperBoundViolation <
         InverseKinematics_B.loop_ub_o;
         InverseKinematics_B.indicesUpperBoundViolation++) {
      limits->data[InverseKinematics_B.indicesUpperBoundViolation + limits->
        size[0] * InverseKinematics_B.b_k] = obj_0->PositionDoFMap[(static_cast<
        int32_T>(bodyIndices->data[h->
                 data[InverseKinematics_B.indicesUpperBoundViolation] - 1]) + 6 *
        InverseKinematics_B.b_k) - 1];
    }
  }

  InverseKinemati_emxFree_int32_T(&h);
  InverseKinematic_emxFree_real_T(&bodyIndices);
  InverseKinematic_emxInit_real_T(&positionIndices, 2);
  InverseKinematics_B.nm1d2 = positionIndices->size[0] * positionIndices->size[1];
  positionIndices->size[0] = 1;
  positionIndices->size[1] = static_cast<int32_T>(obj_0->PositionNumber);
  Invers_emxEnsureCapacity_real_T(positionIndices, InverseKinematics_B.nm1d2);
  InverseKinematics_B.loop_ub_o = static_cast<int32_T>(obj_0->PositionNumber) -
    1;
  if (InverseKinematics_B.loop_ub_o >= 0) {
    memset(&positionIndices->data[0], 0, (InverseKinematics_B.loop_ub_o + 1) *
           sizeof(real_T));
  }

  InverseKinematics_B.bid = 0.0;
  InverseKinematics_B.indicesUpperBoundViolation_size = limits->size[0] - 1;
  InverseKinematic_emxInit_real_T(&e, 2);
  InverseKinemat_emxInit_uint32_T(&y, 2);
  for (InverseKinematics_B.indicesUpperBoundViolation = 0;
       InverseKinematics_B.indicesUpperBoundViolation <=
       InverseKinematics_B.indicesUpperBoundViolation_size;
       InverseKinematics_B.indicesUpperBoundViolation++) {
    InverseKinematics_B.numPositions_tmp = limits->
      data[InverseKinematics_B.indicesUpperBoundViolation + limits->size[0]] -
      limits->data[InverseKinematics_B.indicesUpperBoundViolation];
    if (InverseKinematics_B.numPositions_tmp + 1.0 > 0.0) {
      if (InverseKinematics_B.numPositions_tmp + 1.0 < 1.0) {
        y->size[0] = 1;
        y->size[1] = 0;
      } else {
        InverseKinematics_B.nm1d2 = y->size[0] * y->size[1];
        y->size[0] = 1;
        y->size[1] = static_cast<int32_T>((InverseKinematics_B.numPositions_tmp
          + 1.0) - 1.0) + 1;
        Inve_emxEnsureCapacity_uint32_T(y, InverseKinematics_B.nm1d2);
        InverseKinematics_B.loop_ub_o = static_cast<int32_T>
          ((InverseKinematics_B.numPositions_tmp + 1.0) - 1.0);
        for (InverseKinematics_B.b_k = 0; InverseKinematics_B.b_k <=
             InverseKinematics_B.loop_ub_o; InverseKinematics_B.b_k++) {
          y->data[InverseKinematics_B.b_k] = InverseKinematics_B.b_k + 1U;
        }
      }

      if (rtIsNaN(limits->data[InverseKinematics_B.indicesUpperBoundViolation]) ||
          rtIsNaN(limits->data[InverseKinematics_B.indicesUpperBoundViolation +
                  limits->size[0]])) {
        InverseKinematics_B.nm1d2 = e->size[0] * e->size[1];
        e->size[0] = 1;
        e->size[1] = 1;
        Invers_emxEnsureCapacity_real_T(e, InverseKinematics_B.nm1d2);
        e->data[0] = (rtNaN);
      } else if (limits->data[InverseKinematics_B.indicesUpperBoundViolation +
                 limits->size[0]] < limits->
                 data[InverseKinematics_B.indicesUpperBoundViolation]) {
        e->size[0] = 1;
        e->size[1] = 0;
      } else if ((rtIsInf(limits->
                          data[InverseKinematics_B.indicesUpperBoundViolation]) ||
                  rtIsInf(limits->
                          data[InverseKinematics_B.indicesUpperBoundViolation +
                          limits->size[0]])) && (limits->
                  data[InverseKinematics_B.indicesUpperBoundViolation +
                  limits->size[0]] == limits->
                  data[InverseKinematics_B.indicesUpperBoundViolation])) {
        InverseKinematics_B.nm1d2 = e->size[0] * e->size[1];
        e->size[0] = 1;
        e->size[1] = 1;
        Invers_emxEnsureCapacity_real_T(e, InverseKinematics_B.nm1d2);
        e->data[0] = (rtNaN);
      } else if (floor(limits->
                       data[InverseKinematics_B.indicesUpperBoundViolation]) ==
                 limits->data[InverseKinematics_B.indicesUpperBoundViolation]) {
        InverseKinematics_B.nm1d2 = e->size[0] * e->size[1];
        e->size[0] = 1;
        e->size[1] = static_cast<int32_T>(InverseKinematics_B.numPositions_tmp)
          + 1;
        Invers_emxEnsureCapacity_real_T(e, InverseKinematics_B.nm1d2);
        InverseKinematics_B.loop_ub_o = static_cast<int32_T>(limits->
          data[InverseKinematics_B.indicesUpperBoundViolation + limits->size[0]]
          - limits->data[InverseKinematics_B.indicesUpperBoundViolation]);
        for (InverseKinematics_B.b_k = 0; InverseKinematics_B.b_k <=
             InverseKinematics_B.loop_ub_o; InverseKinematics_B.b_k++) {
          e->data[InverseKinematics_B.b_k] = limits->
            data[InverseKinematics_B.indicesUpperBoundViolation] +
            static_cast<real_T>(InverseKinematics_B.b_k);
        }
      } else {
        InverseKinematics_B.ndbl = floor(InverseKinematics_B.numPositions_tmp +
          0.5);
        InverseKinematics_B.apnd = limits->
          data[InverseKinematics_B.indicesUpperBoundViolation] +
          InverseKinematics_B.ndbl;
        InverseKinematics_B.cdiff = InverseKinematics_B.apnd - limits->
          data[InverseKinematics_B.indicesUpperBoundViolation + limits->size[0]];
        InverseKinematics_B.u0 = fabs(limits->
          data[InverseKinematics_B.indicesUpperBoundViolation]);
        InverseKinematics_B.u1 = fabs(limits->
          data[InverseKinematics_B.indicesUpperBoundViolation + limits->size[0]]);
        if ((InverseKinematics_B.u0 >= InverseKinematics_B.u1) || rtIsNaN
            (InverseKinematics_B.u1)) {
          InverseKinematics_B.u1 = InverseKinematics_B.u0;
        }

        if (fabs(InverseKinematics_B.cdiff) < 4.4408920985006262E-16 *
            InverseKinematics_B.u1) {
          InverseKinematics_B.ndbl++;
          InverseKinematics_B.apnd = limits->
            data[InverseKinematics_B.indicesUpperBoundViolation + limits->size[0]];
        } else if (InverseKinematics_B.cdiff > 0.0) {
          InverseKinematics_B.apnd = (InverseKinematics_B.ndbl - 1.0) +
            limits->data[InverseKinematics_B.indicesUpperBoundViolation];
        } else {
          InverseKinematics_B.ndbl++;
        }

        if (InverseKinematics_B.ndbl >= 0.0) {
          InverseKinematics_B.loop_ub_o = static_cast<int32_T>
            (InverseKinematics_B.ndbl);
        } else {
          InverseKinematics_B.loop_ub_o = 0;
        }

        InverseKinematics_B.nm1d2 = e->size[0] * e->size[1];
        e->size[0] = 1;
        e->size[1] = InverseKinematics_B.loop_ub_o;
        Invers_emxEnsureCapacity_real_T(e, InverseKinematics_B.nm1d2);
        if (InverseKinematics_B.loop_ub_o > 0) {
          e->data[0] = limits->
            data[InverseKinematics_B.indicesUpperBoundViolation];
          if (InverseKinematics_B.loop_ub_o > 1) {
            e->data[InverseKinematics_B.loop_ub_o - 1] =
              InverseKinematics_B.apnd;
            InverseKinematics_B.nm1d2 = (InverseKinematics_B.loop_ub_o - 1) / 2;
            InverseKinematics_B.c_l = InverseKinematics_B.nm1d2 - 2;
            for (InverseKinematics_B.b_k = 0; InverseKinematics_B.b_k <=
                 InverseKinematics_B.c_l; InverseKinematics_B.b_k++) {
              e->data[InverseKinematics_B.b_k + 1] = static_cast<real_T>
                (InverseKinematics_B.b_k + 1) + limits->
                data[InverseKinematics_B.indicesUpperBoundViolation];
              e->data[(InverseKinematics_B.loop_ub_o - InverseKinematics_B.b_k)
                - 2] = InverseKinematics_B.apnd - static_cast<real_T>
                (InverseKinematics_B.b_k + 1);
            }

            if (InverseKinematics_B.nm1d2 << 1 == InverseKinematics_B.loop_ub_o
                - 1) {
              e->data[InverseKinematics_B.nm1d2] = (limits->
                data[InverseKinematics_B.indicesUpperBoundViolation] +
                InverseKinematics_B.apnd) / 2.0;
            } else {
              e->data[InverseKinematics_B.nm1d2] = limits->
                data[InverseKinematics_B.indicesUpperBoundViolation] +
                static_cast<real_T>(InverseKinematics_B.nm1d2);
              e->data[InverseKinematics_B.nm1d2 + 1] = InverseKinematics_B.apnd
                - static_cast<real_T>(InverseKinematics_B.nm1d2);
            }
          }
        }
      }

      InverseKinematics_B.loop_ub_o = e->size[1] - 1;
      for (InverseKinematics_B.b_k = 0; InverseKinematics_B.b_k <=
           InverseKinematics_B.loop_ub_o; InverseKinematics_B.b_k++) {
        InverseKinematics_B.nm1d2 = InverseKinematics_B.b_k;
        positionIndices->data[static_cast<int32_T>(InverseKinematics_B.bid +
          static_cast<real_T>(y->data[InverseKinematics_B.nm1d2])) - 1] =
          e->data[InverseKinematics_B.nm1d2];
      }

      InverseKinematics_B.bid += InverseKinematics_B.numPositions_tmp + 1.0;
    }
  }

  InverseKinemat_emxFree_uint32_T(&y);
  InverseKinematic_emxFree_real_T(&e);
  InverseKinematic_emxFree_real_T(&limits);
  if (InverseKinematics_B.bid < 1.0) {
    InverseKinematics_B.indicesUpperBoundViolation = -1;
  } else {
    InverseKinematics_B.indicesUpperBoundViolation = static_cast<int32_T>
      (InverseKinematics_B.bid) - 1;
  }

  for (InverseKinematics_B.b_k = 0; InverseKinematics_B.b_k <=
       InverseKinematics_B.indicesUpperBoundViolation; InverseKinematics_B.b_k++)
  {
    InverseKinematics_B.bid = positionIndices->data[InverseKinematics_B.b_k];
    initialGuess[static_cast<int32_T>(InverseKinematics_B.bid) - 1] =
      InverseKinematics_B.qvSolRaw[static_cast<int32_T>(InverseKinematics_B.bid)
      - 1];
  }

  InverseKinematic_emxFree_real_T(&positionIndices);
}

void InverseKinematics::Inve_inverseKinematics_stepImpl
  (b_inverseKinematics_InverseKi_T *obj, const real_T tform[16], const real_T
   weights[6], const real_T initialGuess[4], real_T QSol[4])
{
  f_robotics_manip_internal_IKE_T *args;
  static const char_T tmp[5] = { 'B', 'o', 'd', 'y', '5' };

  memset(&InverseKinematics_B.weightMatrix[0], 0, 36U * sizeof(real_T));
  for (InverseKinematics_B.b_j_i = 0; InverseKinematics_B.b_j_i < 6;
       InverseKinematics_B.b_j_i++) {
    InverseKinematics_B.weightMatrix[InverseKinematics_B.b_j_i + 6 *
      InverseKinematics_B.b_j_i] = weights[InverseKinematics_B.b_j_i];
  }

  args = obj->Solver->ExtraArgs;
  for (InverseKinematics_B.b_j_i = 0; InverseKinematics_B.b_j_i < 36;
       InverseKinematics_B.b_j_i++) {
    args->WeightMatrix[InverseKinematics_B.b_j_i] =
      InverseKinematics_B.weightMatrix[InverseKinematics_B.b_j_i];
  }

  InverseKinematics_B.b_j_i = args->BodyName->size[0] * args->BodyName->size[1];
  args->BodyName->size[0] = 1;
  args->BodyName->size[1] = 5;
  Invers_emxEnsureCapacity_char_T(args->BodyName, InverseKinematics_B.b_j_i);
  for (InverseKinematics_B.b_j_i = 0; InverseKinematics_B.b_j_i < 5;
       InverseKinematics_B.b_j_i++) {
    args->BodyName->data[InverseKinematics_B.b_j_i] =
      tmp[InverseKinematics_B.b_j_i];
  }

  for (InverseKinematics_B.b_j_i = 0; InverseKinematics_B.b_j_i < 16;
       InverseKinematics_B.b_j_i++) {
    args->Tform[InverseKinematics_B.b_j_i] = tform[InverseKinematics_B.b_j_i];
  }

  QSol[0] = initialGuess[0];
  QSol[1] = initialGuess[1];
  QSol[2] = initialGuess[2];
  QSol[3] = initialGuess[3];
  Inver_inverseKinematics_solve_n(obj, QSol, &InverseKinematics_B.expl_temp,
    &InverseKinematics_B.expl_temp_n, &InverseKinematics_B.expl_temp_i,
    &InverseKinematics_B.expl_temp_o, InverseKinematics_B.expl_temp_data,
    InverseKinematics_B.expl_temp_size);
}

void InverseKinematics::emxFreeStruct_t_robotics_manip_
  (t_robotics_manip_internal_Rig_T *pStruct)
{
  InverseKinematic_emxFree_char_T(&pStruct->NameInternal);
}

void InverseKinematics::emxFreeStruct_l_robotics_manip_
  (l_robotics_manip_internal_Col_T *pStruct)
{
  InverseK_emxFree_unnamed_struct(&pStruct->CollisionGeometries);
}

void InverseKinematics::emxFreeMatrix_l_robotics_manip_
  (l_robotics_manip_internal_Col_T pMatrix[13])
{
  for (int32_T i = 0; i < 13; i++) {
    emxFreeStruct_l_robotics_manip_(&pMatrix[i]);
  }
}

void InverseKinematics::emxFreeStruct_c_rigidBodyJoint
  (c_rigidBodyJoint_InverseKinem_T *pStruct)
{
  InverseKinematic_emxFree_char_T(&pStruct->Type);
  InverseKinematic_emxFree_real_T(&pStruct->MotionSubspace);
  InverseKinematic_emxFree_char_T(&pStruct->NameInternal);
  InverseKinematic_emxFree_real_T(&pStruct->PositionLimitsInternal);
  InverseKinematic_emxFree_real_T(&pStruct->HomePositionInternal);
}

void InverseKinematics::emxFreeMatrix_c_rigidBodyJoint
  (c_rigidBodyJoint_InverseKinem_T pMatrix[13])
{
  for (int32_T i = 0; i < 13; i++) {
    emxFreeStruct_c_rigidBodyJoint(&pMatrix[i]);
  }
}

void InverseKinematics::emxFreeMatrix_t_robotics_manip_
  (t_robotics_manip_internal_Rig_T pMatrix[12])
{
  for (int32_T i = 0; i < 12; i++) {
    emxFreeStruct_t_robotics_manip_(&pMatrix[i]);
  }
}

void InverseKinematics::emxFreeStruct_u_robotics_manip_
  (u_robotics_manip_internal_Rig_T *pStruct)
{
  emxFreeStruct_t_robotics_manip_(&pStruct->Base);
  emxFreeMatrix_l_robotics_manip_(pStruct->_pobj0);
  emxFreeMatrix_c_rigidBodyJoint(pStruct->_pobj1);
  emxFreeMatrix_t_robotics_manip_(pStruct->_pobj2);
}

void InverseKinematics::emxFreeStruct_f_robotics_manip_
  (f_robotics_manip_internal_IKE_T *pStruct)
{
  InverseKinematic_emxFree_real_T(&pStruct->Limits);
  InverseKinematic_emxFree_char_T(&pStruct->BodyName);
  InverseKinematic_emxFree_real_T(&pStruct->ErrTemp);
  InverseKinematic_emxFree_real_T(&pStruct->GradTemp);
}

void InverseKinematics::emxFreeMatrix_c_rigidBodyJoint1
  (c_rigidBodyJoint_InverseKinem_T pMatrix[12])
{
  for (int32_T i = 0; i < 12; i++) {
    emxFreeStruct_c_rigidBodyJoint(&pMatrix[i]);
  }
}

void InverseKinematics::emxFreeMatrix_t_robotics_mani_n
  (t_robotics_manip_internal_Rig_T pMatrix[6])
{
  for (int32_T i = 0; i < 6; i++) {
    emxFreeStruct_t_robotics_manip_(&pMatrix[i]);
  }
}

void InverseKinematics::emxFreeMatrix_l_robotics_mani_n
  (l_robotics_manip_internal_Col_T pMatrix[7])
{
  for (int32_T i = 0; i < 7; i++) {
    emxFreeStruct_l_robotics_manip_(&pMatrix[i]);
  }
}

void InverseKinematics::emxFreeMatrix_c_rigidBodyJoint2
  (c_rigidBodyJoint_InverseKinem_T pMatrix[7])
{
  for (int32_T i = 0; i < 7; i++) {
    emxFreeStruct_c_rigidBodyJoint(&pMatrix[i]);
  }
}

void InverseKinematics::emxFreeStruct_v_robotics_manip_
  (v_robotics_manip_internal_Rig_T *pStruct)
{
  emxFreeStruct_t_robotics_manip_(&pStruct->Base);
  emxFreeMatrix_t_robotics_mani_n(pStruct->_pobj0);
  emxFreeMatrix_l_robotics_mani_n(pStruct->_pobj1);
  emxFreeMatrix_c_rigidBodyJoint2(pStruct->_pobj2);
}

void InverseKinematics::emxFreeStruct_b_inverseKinemati
  (b_inverseKinematics_InverseKi_T *pStruct)
{
  InverseKinematic_emxFree_real_T(&pStruct->Limits);
  emxFreeStruct_f_robotics_manip_(&pStruct->_pobj0);
  emxFreeMatrix_c_rigidBodyJoint1(pStruct->_pobj1);
  emxFreeMatrix_t_robotics_mani_n(pStruct->_pobj2);
  emxFreeMatrix_l_robotics_manip_(pStruct->_pobj3);
  emxFreeStruct_v_robotics_manip_(&pStruct->_pobj4);
}

void InverseKinematics::emxFreeStruct_robotics_slmanip_
  (robotics_slmanip_internal_blo_T *pStruct)
{
  emxFreeStruct_u_robotics_manip_(&pStruct->TreeInternal);
  emxFreeStruct_b_inverseKinemati(&pStruct->IKInternal);
}

/* Model step function */
void InverseKinematics::step()
{
  b_inverseKinematics_InverseKi_T *obj;
  emxArray_int8_T_InverseKinema_T *b_gradTmp;
  emxArray_real_T_InverseKinema_T *tmp;

  /* MATLABSystem: '<S3>/SourceBlock' */
  InverseKinematics_B.b_varargout_1_m = ros2::matlab::
    getLatestMessage_Sub_InverseKinematics_562
    (&InverseKinematics_B.b_varargout_2);

  /* Outputs for Enabled SubSystem: '<S3>/Enabled Subsystem' incorporates:
   *  EnablePort: '<S7>/Enable'
   */
  if (InverseKinematics_B.b_varargout_1_m) {
    /* SignalConversion generated from: '<S7>/In1' */
    InverseKinematics_B.In1 = InverseKinematics_B.b_varargout_2;
  }

  /* End of MATLABSystem: '<S3>/SourceBlock' */
  /* End of Outputs for SubSystem: '<S3>/Enabled Subsystem' */

  /* MATLABSystem: '<S1>/Coordinate Transformation Conversion1' incorporates:
   *  SignalConversion generated from: '<S1>/Coordinate Transformation Conversion1'
   */
  for (InverseKinematics_B.b_jcol = 0; InverseKinematics_B.b_jcol < 16;
       InverseKinematics_B.b_jcol++) {
    InverseKinematics_B.b_I[InverseKinematics_B.b_jcol] = 0;
  }

  InverseKinematics_B.b_I[0] = 1;
  InverseKinematics_B.b_I[5] = 1;
  InverseKinematics_B.b_I[10] = 1;
  InverseKinematics_B.b_I[15] = 1;
  for (InverseKinematics_B.b_jcol = 0; InverseKinematics_B.b_jcol < 4;
       InverseKinematics_B.b_jcol++) {
    InverseKinematics_B.iacol_tmp = (InverseKinematics_B.b_jcol << 2) - 1;
    InverseKinematics_B.out[InverseKinematics_B.iacol_tmp + 1] =
      InverseKinematics_B.b_I[InverseKinematics_B.iacol_tmp + 1];
    InverseKinematics_B.out[InverseKinematics_B.iacol_tmp + 2] =
      InverseKinematics_B.b_I[InverseKinematics_B.iacol_tmp + 2];
    InverseKinematics_B.out[InverseKinematics_B.iacol_tmp + 3] =
      InverseKinematics_B.b_I[InverseKinematics_B.iacol_tmp + 3];
    InverseKinematics_B.out[InverseKinematics_B.iacol_tmp + 4] =
      InverseKinematics_B.b_I[InverseKinematics_B.iacol_tmp + 4];
  }

  InverseKinematics_B.out[12] = InverseKinematics_B.In1.x;
  InverseKinematics_B.out[13] = InverseKinematics_B.In1.y;
  InverseKinematics_B.out[14] = InverseKinematics_B.In1.z;

  /* MATLABSystem: '<S4>/MATLAB System' incorporates:
   *  Constant: '<S1>/Constant1'
   *  Delay: '<S1>/Delay'
   *  MATLABSystem: '<S1>/Coordinate Transformation Conversion1'
   */
  obj = &InverseKinematics_DW.obj.IKInternal;
  if (InverseKinematics_DW.obj.IKInternal.isInitialized != 1) {
    InverseKinematics_DW.obj.IKInternal.isSetupComplete = false;
    InverseKinematics_DW.obj.IKInternal.isInitialized = 1;
    RigidBodyTree_get_JointPosition
      (InverseKinematics_DW.obj.IKInternal.RigidBodyTreeInternal,
       InverseKinematics_DW.obj.IKInternal.Limits);
    InverseKinematics_DW.obj.IKInternal.Solver->ExtraArgs = &obj->_pobj0;
    for (InverseKinematics_B.b_jcol = 0; InverseKinematics_B.b_jcol < 36;
         InverseKinematics_B.b_jcol++) {
      InverseKinematics_DW.obj.IKInternal.Solver->ExtraArgs->
        WeightMatrix[InverseKinematics_B.b_jcol] = 0.0;
    }

    InverseKinematic_emxInit_real_T(&tmp, 1);
    InverseKinematics_DW.obj.IKInternal.Solver->ExtraArgs->Robot =
      InverseKinematics_DW.obj.IKInternal.RigidBodyTreeInternal;
    InverseKinematics_B.loop_ub =
      InverseKinematics_DW.obj.IKInternal.Limits->size[0] << 1;
    InverseKinematics_B.b_jcol =
      InverseKinematics_DW.obj.IKInternal.Solver->ExtraArgs->Limits->size[0] *
      InverseKinematics_DW.obj.IKInternal.Solver->ExtraArgs->Limits->size[1];
    InverseKinematics_DW.obj.IKInternal.Solver->ExtraArgs->Limits->size[0] =
      InverseKinematics_DW.obj.IKInternal.Limits->size[0];
    InverseKinematics_DW.obj.IKInternal.Solver->ExtraArgs->Limits->size[1] = 2;
    Invers_emxEnsureCapacity_real_T
      (InverseKinematics_DW.obj.IKInternal.Solver->ExtraArgs->Limits,
       InverseKinematics_B.b_jcol);
    InverseKinematics_B.b_jcol = tmp->size[0];
    tmp->size[0] = InverseKinematics_B.loop_ub;
    Invers_emxEnsureCapacity_real_T(tmp, InverseKinematics_B.b_jcol);
    for (InverseKinematics_B.iacol_tmp = 0; InverseKinematics_B.iacol_tmp <
         InverseKinematics_B.loop_ub; InverseKinematics_B.iacol_tmp++) {
      tmp->data[InverseKinematics_B.iacol_tmp] =
        InverseKinematics_DW.obj.IKInternal.Limits->
        data[InverseKinematics_B.iacol_tmp];
    }

    InverseKinematics_B.loop_ub = tmp->size[0];
    for (InverseKinematics_B.iacol_tmp = 0; InverseKinematics_B.iacol_tmp <
         InverseKinematics_B.loop_ub; InverseKinematics_B.iacol_tmp++) {
      InverseKinematics_DW.obj.IKInternal.Solver->ExtraArgs->Limits->
        data[InverseKinematics_B.iacol_tmp] = tmp->
        data[InverseKinematics_B.iacol_tmp];
    }

    InverseKinematic_emxFree_real_T(&tmp);
    for (InverseKinematics_B.b_jcol = 0; InverseKinematics_B.b_jcol < 16;
         InverseKinematics_B.b_jcol++) {
      InverseKinematics_B.b_I[InverseKinematics_B.b_jcol] = 0;
    }

    InverseKinematics_B.b_I[0] = 1;
    InverseKinematics_B.b_I[5] = 1;
    InverseKinematics_B.b_I[10] = 1;
    InverseKinematics_B.b_I[15] = 1;
    for (InverseKinematics_B.b_jcol = 0; InverseKinematics_B.b_jcol < 16;
         InverseKinematics_B.b_jcol++) {
      InverseKinematics_DW.obj.IKInternal.Solver->ExtraArgs->
        Tform[InverseKinematics_B.b_jcol] =
        InverseKinematics_B.b_I[InverseKinematics_B.b_jcol];
    }

    InverseKinematics_DW.obj.IKInternal.Solver->ExtraArgs->BodyName->size[0] = 1;
    InverseKinematics_DW.obj.IKInternal.Solver->ExtraArgs->BodyName->size[1] = 0;
    InverseKinematics_B.b_jcol =
      InverseKinematics_DW.obj.IKInternal.Solver->ExtraArgs->ErrTemp->size[0];
    InverseKinematics_DW.obj.IKInternal.Solver->ExtraArgs->ErrTemp->size[0] = 6;
    Invers_emxEnsureCapacity_real_T
      (InverseKinematics_DW.obj.IKInternal.Solver->ExtraArgs->ErrTemp,
       InverseKinematics_B.b_jcol);
    for (InverseKinematics_B.b_jcol = 0; InverseKinematics_B.b_jcol < 6;
         InverseKinematics_B.b_jcol++) {
      InverseKinematics_DW.obj.IKInternal.Solver->ExtraArgs->ErrTemp->
        data[InverseKinematics_B.b_jcol] = 0.0;
    }

    InverseKinematic_emxInit_int8_T(&b_gradTmp, 1);
    InverseKinematics_DW.obj.IKInternal.Solver->ExtraArgs->CostTemp = 0.0;
    InverseKinematics_B.b_jcol = b_gradTmp->size[0];
    b_gradTmp->size[0] = static_cast<int32_T>
      (InverseKinematics_DW.obj.IKInternal.RigidBodyTreeInternal->PositionNumber);
    Invers_emxEnsureCapacity_int8_T(b_gradTmp, InverseKinematics_B.b_jcol);
    InverseKinematics_B.loop_ub = static_cast<int32_T>
      (InverseKinematics_DW.obj.IKInternal.RigidBodyTreeInternal->PositionNumber);
    if (InverseKinematics_B.loop_ub - 1 >= 0) {
      memset(&b_gradTmp->data[0], 0, InverseKinematics_B.loop_ub * sizeof(int8_T));
    }

    InverseKinematics_B.b_jcol =
      InverseKinematics_DW.obj.IKInternal.Solver->ExtraArgs->GradTemp->size[0];
    InverseKinematics_DW.obj.IKInternal.Solver->ExtraArgs->GradTemp->size[0] =
      b_gradTmp->size[0];
    Invers_emxEnsureCapacity_real_T
      (InverseKinematics_DW.obj.IKInternal.Solver->ExtraArgs->GradTemp,
       InverseKinematics_B.b_jcol);
    InverseKinematics_B.loop_ub = b_gradTmp->size[0];
    InverseKinematic_emxFree_int8_T(&b_gradTmp);
    for (InverseKinematics_B.b_jcol = 0; InverseKinematics_B.b_jcol <
         InverseKinematics_B.loop_ub; InverseKinematics_B.b_jcol++) {
      InverseKinematics_DW.obj.IKInternal.Solver->ExtraArgs->GradTemp->
        data[InverseKinematics_B.b_jcol] = 0.0;
    }

    InverseKinematics_DW.obj.IKInternal.isSetupComplete = true;
  }

  Inve_inverseKinematics_stepImpl(&InverseKinematics_DW.obj.IKInternal,
    InverseKinematics_B.out, InverseKinematics_P.Constant1_Value,
    InverseKinematics_DW.Delay_DSTATE, InverseKinematics_B.b_varargout_1);

  /* BusAssignment: '<S2>/Bus Assignment' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   */
  InverseKinematics_B.BusAssignment.x = InverseKinematics_B.b_varargout_1[0];
  InverseKinematics_B.BusAssignment.y = InverseKinematics_B.b_varargout_1[1];
  InverseKinematics_B.BusAssignment.z = InverseKinematics_B.b_varargout_1[2];
  InverseKinematics_B.BusAssignment.w = InverseKinematics_B.b_varargout_1[3];

  /* MATLABSystem: '<S6>/SinkBlock' */
  ros2::matlab::publish_Pub_InverseKinematics_466
    (&InverseKinematics_B.BusAssignment);

  /* Update for Delay: '<S1>/Delay' incorporates:
   *  MATLABSystem: '<S4>/MATLAB System'
   */
  InverseKinematics_DW.Delay_DSTATE[0] = InverseKinematics_B.b_varargout_1[0];
  InverseKinematics_DW.Delay_DSTATE[1] = InverseKinematics_B.b_varargout_1[1];
  InverseKinematics_DW.Delay_DSTATE[2] = InverseKinematics_B.b_varargout_1[2];
  InverseKinematics_DW.Delay_DSTATE[3] = InverseKinematics_B.b_varargout_1[3];
}

/* Model initialize function */
void InverseKinematics::initialize()
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  {
    static const uint32_T tmp[625] = { 5489U, 1301868182U, 2938499221U,
      2950281878U, 1875628136U, 751856242U, 944701696U, 2243192071U, 694061057U,
      219885934U, 2066767472U, 3182869408U, 485472502U, 2336857883U, 1071588843U,
      3418470598U, 951210697U, 3693558366U, 2923482051U, 1793174584U,
      2982310801U, 1586906132U, 1951078751U, 1808158765U, 1733897588U,
      431328322U, 4202539044U, 530658942U, 1714810322U, 3025256284U, 3342585396U,
      1937033938U, 2640572511U, 1654299090U, 3692403553U, 4233871309U,
      3497650794U, 862629010U, 2943236032U, 2426458545U, 1603307207U,
      1133453895U, 3099196360U, 2208657629U, 2747653927U, 931059398U, 761573964U,
      3157853227U, 785880413U, 730313442U, 124945756U, 2937117055U, 3295982469U,
      1724353043U, 3021675344U, 3884886417U, 4010150098U, 4056961966U,
      699635835U, 2681338818U, 1339167484U, 720757518U, 2800161476U, 2376097373U,
      1532957371U, 3902664099U, 1238982754U, 3725394514U, 3449176889U,
      3570962471U, 4287636090U, 4087307012U, 3603343627U, 202242161U,
      2995682783U, 1620962684U, 3704723357U, 371613603U, 2814834333U,
      2111005706U, 624778151U, 2094172212U, 4284947003U, 1211977835U, 991917094U,
      1570449747U, 2962370480U, 1259410321U, 170182696U, 146300961U, 2836829791U,
      619452428U, 2723670296U, 1881399711U, 1161269684U, 1675188680U,
      4132175277U, 780088327U, 3409462821U, 1036518241U, 1834958505U,
      3048448173U, 161811569U, 618488316U, 44795092U, 3918322701U, 1924681712U,
      3239478144U, 383254043U, 4042306580U, 2146983041U, 3992780527U,
      3518029708U, 3545545436U, 3901231469U, 1896136409U, 2028528556U,
      2339662006U, 501326714U, 2060962201U, 2502746480U, 561575027U, 581893337U,
      3393774360U, 1778912547U, 3626131687U, 2175155826U, 319853231U, 986875531U,
      819755096U, 2915734330U, 2688355739U, 3482074849U, 2736559U, 2296975761U,
      1029741190U, 2876812646U, 690154749U, 579200347U, 4027461746U, 1285330465U,
      2701024045U, 4117700889U, 759495121U, 3332270341U, 2313004527U,
      2277067795U, 4131855432U, 2722057515U, 1264804546U, 3848622725U,
      2211267957U, 4100593547U, 959123777U, 2130745407U, 3194437393U, 486673947U,
      1377371204U, 17472727U, 352317554U, 3955548058U, 159652094U, 1232063192U,
      3835177280U, 49423123U, 3083993636U, 733092U, 2120519771U, 2573409834U,
      1112952433U, 3239502554U, 761045320U, 1087580692U, 2540165110U, 641058802U,
      1792435497U, 2261799288U, 1579184083U, 627146892U, 2165744623U,
      2200142389U, 2167590760U, 2381418376U, 1793358889U, 3081659520U,
      1663384067U, 2009658756U, 2689600308U, 739136266U, 2304581039U,
      3529067263U, 591360555U, 525209271U, 3131882996U, 294230224U, 2076220115U,
      3113580446U, 1245621585U, 1386885462U, 3203270426U, 123512128U, 12350217U,
      354956375U, 4282398238U, 3356876605U, 3888857667U, 157639694U, 2616064085U,
      1563068963U, 2762125883U, 4045394511U, 4180452559U, 3294769488U,
      1684529556U, 1002945951U, 3181438866U, 22506664U, 691783457U, 2685221343U,
      171579916U, 3878728600U, 2475806724U, 2030324028U, 3331164912U,
      1708711359U, 1970023127U, 2859691344U, 2588476477U, 2748146879U,
      136111222U, 2967685492U, 909517429U, 2835297809U, 3206906216U, 3186870716U,
      341264097U, 2542035121U, 3353277068U, 548223577U, 3170936588U, 1678403446U,
      297435620U, 2337555430U, 466603495U, 1132321815U, 1208589219U, 696392160U,
      894244439U, 2562678859U, 470224582U, 3306867480U, 201364898U, 2075966438U,
      1767227936U, 2929737987U, 3674877796U, 2654196643U, 3692734598U,
      3528895099U, 2796780123U, 3048728353U, 842329300U, 191554730U, 2922459673U,
      3489020079U, 3979110629U, 1022523848U, 2202932467U, 3583655201U,
      3565113719U, 587085778U, 4176046313U, 3013713762U, 950944241U, 396426791U,
      3784844662U, 3477431613U, 3594592395U, 2782043838U, 3392093507U,
      3106564952U, 2829419931U, 1358665591U, 2206918825U, 3170783123U, 31522386U,
      2988194168U, 1782249537U, 1105080928U, 843500134U, 1225290080U,
      1521001832U, 3605886097U, 2802786495U, 2728923319U, 3996284304U,
      903417639U, 1171249804U, 1020374987U, 2824535874U, 423621996U, 1988534473U,
      2493544470U, 1008604435U, 1756003503U, 1488867287U, 1386808992U,
      732088248U, 1780630732U, 2482101014U, 976561178U, 1543448953U, 2602866064U,
      2021139923U, 1952599828U, 2360242564U, 2117959962U, 2753061860U,
      2388623612U, 4138193781U, 2962920654U, 2284970429U, 766920861U,
      3457264692U, 2879611383U, 815055854U, 2332929068U, 1254853997U,
      3740375268U, 3799380844U, 4091048725U, 2006331129U, 1982546212U,
      686850534U, 1907447564U, 2682801776U, 2780821066U, 998290361U, 1342433871U,
      4195430425U, 607905174U, 3902331779U, 2454067926U, 1708133115U,
      1170874362U, 2008609376U, 3260320415U, 2211196135U, 433538229U,
      2728786374U, 2189520818U, 262554063U, 1182318347U, 3710237267U,
      1221022450U, 715966018U, 2417068910U, 2591870721U, 2870691989U,
      3418190842U, 4238214053U, 1540704231U, 1575580968U, 2095917976U,
      4078310857U, 2313532447U, 2110690783U, 4056346629U, 4061784526U,
      1123218514U, 551538993U, 597148360U, 4120175196U, 3581618160U, 3181170517U,
      422862282U, 3227524138U, 1713114790U, 662317149U, 1230418732U, 928171837U,
      1324564878U, 1928816105U, 1786535431U, 2878099422U, 3290185549U,
      539474248U, 1657512683U, 552370646U, 1671741683U, 3655312128U, 1552739510U,
      2605208763U, 1441755014U, 181878989U, 3124053868U, 1447103986U,
      3183906156U, 1728556020U, 3502241336U, 3055466967U, 1013272474U,
      818402132U, 1715099063U, 2900113506U, 397254517U, 4194863039U, 1009068739U,
      232864647U, 2540223708U, 2608288560U, 2415367765U, 478404847U, 3455100648U,
      3182600021U, 2115988978U, 434269567U, 4117179324U, 3461774077U, 887256537U,
      3545801025U, 286388911U, 3451742129U, 1981164769U, 786667016U, 3310123729U,
      3097811076U, 2224235657U, 2959658883U, 3370969234U, 2514770915U,
      3345656436U, 2677010851U, 2206236470U, 271648054U, 2342188545U,
      4292848611U, 3646533909U, 3754009956U, 3803931226U, 4160647125U,
      1477814055U, 4043852216U, 1876372354U, 3133294443U, 3871104810U,
      3177020907U, 2074304428U, 3479393793U, 759562891U, 164128153U, 1839069216U,
      2114162633U, 3989947309U, 3611054956U, 1333547922U, 835429831U, 494987340U,
      171987910U, 1252001001U, 370809172U, 3508925425U, 2535703112U, 1276855041U,
      1922855120U, 835673414U, 3030664304U, 613287117U, 171219893U, 3423096126U,
      3376881639U, 2287770315U, 1658692645U, 1262815245U, 3957234326U,
      1168096164U, 2968737525U, 2655813712U, 2132313144U, 3976047964U,
      326516571U, 353088456U, 3679188938U, 3205649712U, 2654036126U, 1249024881U,
      880166166U, 691800469U, 2229503665U, 1673458056U, 4032208375U, 1851778863U,
      2563757330U, 376742205U, 1794655231U, 340247333U, 1505873033U, 396524441U,
      879666767U, 3335579166U, 3260764261U, 3335999539U, 506221798U, 4214658741U,
      975887814U, 2080536343U, 3360539560U, 571586418U, 138896374U, 4234352651U,
      2737620262U, 3928362291U, 1516365296U, 38056726U, 3599462320U, 3585007266U,
      3850961033U, 471667319U, 1536883193U, 2310166751U, 1861637689U,
      2530999841U, 4139843801U, 2710569485U, 827578615U, 2012334720U,
      2907369459U, 3029312804U, 2820112398U, 1965028045U, 35518606U, 2478379033U,
      643747771U, 1924139484U, 4123405127U, 3811735531U, 3429660832U,
      3285177704U, 1948416081U, 1311525291U, 1183517742U, 1739192232U,
      3979815115U, 2567840007U, 4116821529U, 213304419U, 4125718577U,
      1473064925U, 2442436592U, 1893310111U, 4195361916U, 3747569474U,
      828465101U, 2991227658U, 750582866U, 1205170309U, 1409813056U, 678418130U,
      1171531016U, 3821236156U, 354504587U, 4202874632U, 3882511497U,
      1893248677U, 1903078632U, 26340130U, 2069166240U, 3657122492U, 3725758099U,
      831344905U, 811453383U, 3447711422U, 2434543565U, 4166886888U, 3358210805U,
      4142984013U, 2988152326U, 3527824853U, 982082992U, 2809155763U, 190157081U,
      3340214818U, 2365432395U, 2548636180U, 2894533366U, 3474657421U,
      2372634704U, 2845748389U, 43024175U, 2774226648U, 1987702864U, 3186502468U,
      453610222U, 4204736567U, 1392892630U, 2471323686U, 2470534280U,
      3541393095U, 4269885866U, 3909911300U, 759132955U, 1482612480U, 667715263U,
      1795580598U, 2337923983U, 3390586366U, 581426223U, 1515718634U, 476374295U,
      705213300U, 363062054U, 2084697697U, 2407503428U, 2292957699U, 2426213835U,
      2199989172U, 1987356470U, 4026755612U, 2147252133U, 270400031U,
      1367820199U, 2369854699U, 2844269403U, 79981964U, 624U };

    /* Start for MATLABSystem: '<S3>/SourceBlock' */
    InverseKinematics_DW.obj_n.matlabCodegenIsDeleted = true;
    InverseKinematics_DW.obj_n.isInitialized = 0;
    InverseKinematics_DW.obj_n.matlabCodegenIsDeleted = false;
    InverseKinematics_DW.objisempty = true;
    InverseKine_SystemCore_setup_nl(&InverseKinematics_DW.obj_n);

    /* Start for MATLABSystem: '<S1>/Coordinate Transformation Conversion1' */
    InverseKinematics_DW.obj_j.isInitialized = 0;
    InverseKinematics_DW.objisempty_an = true;
    InverseKinematics_DW.obj_j.isInitialized = 1;
    emxInitStruct_robotics_slmanip_(&InverseKinematics_DW.obj);

    /* Start for MATLABSystem: '<S4>/MATLAB System' */
    for (InverseKinematics_B.i_h = 0; InverseKinematics_B.i_h < 13;
         InverseKinematics_B.i_h++) {
      InverseKinematics_DW.obj.TreeInternal._pobj0[InverseKinematics_B.i_h].
        _pobj0.matlabCodegenIsDeleted = true;
    }

    for (InverseKinematics_B.i_h = 0; InverseKinematics_B.i_h < 7;
         InverseKinematics_B.i_h++) {
      InverseKinematics_DW.obj.IKInternal._pobj4._pobj1[InverseKinematics_B.i_h]
        ._pobj0.matlabCodegenIsDeleted = true;
    }

    for (InverseKinematics_B.i_h = 0; InverseKinematics_B.i_h < 13;
         InverseKinematics_B.i_h++) {
      InverseKinematics_DW.obj.IKInternal._pobj3[InverseKinematics_B.i_h].
        _pobj0.matlabCodegenIsDeleted = true;
    }

    InverseKinematics_DW.obj.IKInternal.matlabCodegenIsDeleted = true;
    InverseKinematics_DW.obj.matlabCodegenIsDeleted = true;
    InverseKinematics_DW.method_a = 7U;
    InverseKinematics_DW.freq_not_empty = true;
    InverseKinematics_DW.state = 1144108930U;
    InverseKinematics_DW.state_not_empty = true;
    InverseKinematics_DW.state_p[0] = 362436069U;
    InverseKinematics_DW.state_p[1] = 521288629U;
    InverseKinematics_DW.state_not_empty_l = true;
    memcpy(&InverseKinematics_DW.state_c[0], &tmp[0], 625U * sizeof(uint32_T));
    InverseKinematics_DW.method_not_empty_f = true;
    InverseKinematics_DW.state_not_empty_c = true;
    InverseKinematics_DW.state_pz[0] = 362436069U;
    InverseKinematics_DW.state_pz[1] = 521288629U;
    InverseKinematics_DW.state_not_empty_b = true;
    InverseKinematics_DW.obj.isInitialized = 0;
    InverseKinematics_DW.obj.matlabCodegenIsDeleted = false;
    InverseKinematics_DW.objisempty_a = true;
    InverseKinemat_SystemCore_setup(&InverseKinematics_DW.obj);

    /* End of Start for MATLABSystem: '<S4>/MATLAB System' */

    /* Start for MATLABSystem: '<S6>/SinkBlock' */
    InverseKinematics_DW.obj_k.matlabCodegenIsDeleted = true;
    InverseKinematics_DW.obj_k.isInitialized = 0;
    InverseKinematics_DW.obj_k.matlabCodegenIsDeleted = false;
    InverseKinematics_DW.objisempty_b = true;
    InverseKinem_SystemCore_setup_n(&InverseKinematics_DW.obj_k);
  }

  /* InitializeConditions for Delay: '<S1>/Delay' */
  InverseKinematics_DW.Delay_DSTATE[0] =
    InverseKinematics_P.Delay_InitialCondition[0];
  InverseKinematics_DW.Delay_DSTATE[1] =
    InverseKinematics_P.Delay_InitialCondition[1];
  InverseKinematics_DW.Delay_DSTATE[2] =
    InverseKinematics_P.Delay_InitialCondition[2];
  InverseKinematics_DW.Delay_DSTATE[3] =
    InverseKinematics_P.Delay_InitialCondition[3];

  /* SystemInitialize for Enabled SubSystem: '<S3>/Enabled Subsystem' */
  /* SystemInitialize for SignalConversion generated from: '<S7>/In1' incorporates:
   *  Outport: '<S7>/Out1'
   */
  InverseKinematics_B.In1 = InverseKinematics_P.Out1_Y0;

  /* End of SystemInitialize for SubSystem: '<S3>/Enabled Subsystem' */
}

/* Model terminate function */
void InverseKinematics::terminate()
{
  void* geometryInternal;
  b_inverseKinematics_InverseKi_T *obj;
  k_robotics_manip_internal_Col_T *obj_0;

  /* Terminate for MATLABSystem: '<S3>/SourceBlock' */
  if (!InverseKinematics_DW.obj_n.matlabCodegenIsDeleted) {
    InverseKinematics_DW.obj_n.matlabCodegenIsDeleted = true;
  }

  /* End of Terminate for MATLABSystem: '<S3>/SourceBlock' */

  /* Terminate for MATLABSystem: '<S4>/MATLAB System' */
  if (!InverseKinematics_DW.obj.matlabCodegenIsDeleted) {
    InverseKinematics_DW.obj.matlabCodegenIsDeleted = true;
  }

  obj = &InverseKinematics_DW.obj.IKInternal;
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
    if (obj->isInitialized == 1) {
      obj->isInitialized = 2;
    }
  }

  for (int32_T b = 0; b < 13; b++) {
    obj_0 = &InverseKinematics_DW.obj.IKInternal._pobj3[b]._pobj0;
    if (!obj_0->matlabCodegenIsDeleted) {
      obj_0->matlabCodegenIsDeleted = true;
      geometryInternal = obj_0->CollisionPrimitive;
      collisioncodegen_destructGeometry(&geometryInternal);
    }
  }

  for (int32_T b = 0; b < 7; b++) {
    obj_0 = &InverseKinematics_DW.obj.IKInternal._pobj4._pobj1[b]._pobj0;
    if (!obj_0->matlabCodegenIsDeleted) {
      obj_0->matlabCodegenIsDeleted = true;
      geometryInternal = obj_0->CollisionPrimitive;
      collisioncodegen_destructGeometry(&geometryInternal);
    }
  }

  for (int32_T b = 0; b < 13; b++) {
    obj_0 = &InverseKinematics_DW.obj.TreeInternal._pobj0[b]._pobj0;
    if (!obj_0->matlabCodegenIsDeleted) {
      obj_0->matlabCodegenIsDeleted = true;
      geometryInternal = obj_0->CollisionPrimitive;
      collisioncodegen_destructGeometry(&geometryInternal);
    }
  }

  /* End of Terminate for MATLABSystem: '<S4>/MATLAB System' */
  emxFreeStruct_robotics_slmanip_(&InverseKinematics_DW.obj);

  /* Terminate for MATLABSystem: '<S6>/SinkBlock' */
  if (!InverseKinematics_DW.obj_k.matlabCodegenIsDeleted) {
    InverseKinematics_DW.obj_k.matlabCodegenIsDeleted = true;
  }

  /* End of Terminate for MATLABSystem: '<S6>/SinkBlock' */
}

/* Constructor */
InverseKinematics::InverseKinematics() :
  InverseKinematics_B(),
  InverseKinematics_DW(),
  InverseKinematics_M()
{
  /* Currently there is no constructor body generated.*/
}

/* Destructor */
InverseKinematics::~InverseKinematics()
{
  /* Currently there is no destructor body generated.*/
}

/* Real-Time Model get method */
RT_MODEL_InverseKinematics_T * InverseKinematics::getRTM()
{
  return (&InverseKinematics_M);
}
