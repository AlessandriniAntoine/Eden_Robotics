/*
 * axes.cpp
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

#include "axes.h"
#include "axes_private.h"

/* Block signals (default storage) */
B_axes_T axes_B;

/* Block states (default storage) */
DW_axes_T axes_DW;

/* Real-time model */
RT_MODEL_axes_T axes_M_ = RT_MODEL_axes_T();
RT_MODEL_axes_T *const axes_M = &axes_M_;

/* Forward declaration for local functions */
static void axes_emxInit_char_T(emxArray_char_T_axes_T **pEmxArray, int32_T
  numDimensions);
static void emxInitStruct_c_rigidBodyJoint(c_rigidBodyJoint_axes_T *pStruct);
static void axes_emxInit_unnamed_struct(emxArray_unnamed_struct_axes_T
  **pEmxArray, int32_T numDimensions);
static void emxInitStruct_j_robotics_manip_(j_robotics_manip_internal_Col_T
  *pStruct);
static void emxInitStruct_k_robotics_manip_(k_robotics_manip_internal_Rig_T
  *pStruct);
static void emxInitMatrix_k_robotics_manip_(k_robotics_manip_internal_Rig_T
  pMatrix[6]);
static void emxInitStruct_l_robotics_manip_(l_robotics_manip_internal_Rig_T
  *pStruct);
static void emxInitStruct_robotics_slmanip_(robotics_slmanip_internal_blo_T
  *pStruct);
static void axes_emxEnsureCapacity_char_T(emxArray_char_T_axes_T *emxArray,
  int32_T oldNumel);
static void emxEnsureCapacity_unnamed_struc(emxArray_unnamed_struct_axes_T
  *emxArray, int32_T oldNumel);
static void axes_emxFree_unnamed_struct(emxArray_unnamed_struct_axes_T
  **pEmxArray);
static j_robotics_manip_internal_Col_T *axes_CollisionSet_CollisionSet
  (j_robotics_manip_internal_Col_T *obj);
static void axes_emxFree_char_T(emxArray_char_T_axes_T **pEmxArray);
static k_robotics_manip_internal_Rig_T *axes_RigidBody_RigidBody
  (k_robotics_manip_internal_Rig_T *obj);
static k_robotics_manip_internal_Rig_T *axes_RigidBody_RigidBody_c
  (k_robotics_manip_internal_Rig_T *obj);
static k_robotics_manip_internal_Rig_T *axes_RigidBody_RigidBody_cu
  (k_robotics_manip_internal_Rig_T *obj);
static void axes_SystemCore_setup(robotics_slmanip_internal_blo_T *obj);
static void axes_emxInit_d_cell_wrap(emxArray_d_cell_wrap_axes_T **pEmxArray,
  int32_T numDimensions);
static void a_emxEnsureCapacity_d_cell_wrap(emxArray_d_cell_wrap_axes_T
  *emxArray, int32_T oldNumel);
static void ax_rigidBodyJoint_get_JointAxis(const c_rigidBodyJoint_axes_T *obj,
  real_T ax[3]);
static void axes_emxFree_d_cell_wrap(emxArray_d_cell_wrap_axes_T **pEmxArray);
static void emxFreeStruct_c_rigidBodyJoint(c_rigidBodyJoint_axes_T *pStruct);
static void emxFreeStruct_j_robotics_manip_(j_robotics_manip_internal_Col_T
  *pStruct);
static void emxFreeStruct_k_robotics_manip_(k_robotics_manip_internal_Rig_T
  *pStruct);
static void emxFreeMatrix_k_robotics_manip_(k_robotics_manip_internal_Rig_T
  pMatrix[6]);
static void emxFreeStruct_l_robotics_manip_(l_robotics_manip_internal_Rig_T
  *pStruct);
static void emxFreeStruct_robotics_slmanip_(robotics_slmanip_internal_blo_T
  *pStruct);

/*
 * Output and update for action system:
 *    '<S4>/poser'
 *    '<S4>/zero'
 */
void axes_poser(real_T rty_Out1[3], P_poser_axes_T *localP)
{
  /* SignalConversion generated from: '<S7>/Out1' incorporates:
   *  Constant: '<S7>/Constant'
   */
  rty_Out1[0] = localP->Constant_Value[0];
  rty_Out1[1] = localP->Constant_Value[1];
  rty_Out1[2] = localP->Constant_Value[2];
}

static void axes_emxInit_char_T(emxArray_char_T_axes_T **pEmxArray, int32_T
  numDimensions)
{
  emxArray_char_T_axes_T *emxArray;
  *pEmxArray = (emxArray_char_T_axes_T *)malloc(sizeof(emxArray_char_T_axes_T));
  emxArray = *pEmxArray;
  emxArray->data = (char_T *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int32_T *)malloc(sizeof(int32_T) * numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (int32_T i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

static void emxInitStruct_c_rigidBodyJoint(c_rigidBodyJoint_axes_T *pStruct)
{
  axes_emxInit_char_T(&pStruct->Type, 2);
}

static void axes_emxInit_unnamed_struct(emxArray_unnamed_struct_axes_T
  **pEmxArray, int32_T numDimensions)
{
  emxArray_unnamed_struct_axes_T *emxArray;
  *pEmxArray = (emxArray_unnamed_struct_axes_T *)malloc(sizeof
    (emxArray_unnamed_struct_axes_T));
  emxArray = *pEmxArray;
  emxArray->data = (i_robotics_manip_internal_Col_T **)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int32_T *)malloc(sizeof(int32_T) * numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (int32_T i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

static void emxInitStruct_j_robotics_manip_(j_robotics_manip_internal_Col_T
  *pStruct)
{
  axes_emxInit_unnamed_struct(&pStruct->CollisionGeometries, 2);
}

static void emxInitStruct_k_robotics_manip_(k_robotics_manip_internal_Rig_T
  *pStruct)
{
  axes_emxInit_char_T(&pStruct->NameInternal, 2);
  emxInitStruct_c_rigidBodyJoint(&pStruct->JointInternal);
  emxInitStruct_j_robotics_manip_(&pStruct->CollisionsInternal);
}

static void emxInitMatrix_k_robotics_manip_(k_robotics_manip_internal_Rig_T
  pMatrix[6])
{
  for (int32_T i = 0; i < 6; i++) {
    emxInitStruct_k_robotics_manip_(&pMatrix[i]);
  }
}

static void emxInitStruct_l_robotics_manip_(l_robotics_manip_internal_Rig_T
  *pStruct)
{
  emxInitStruct_k_robotics_manip_(&pStruct->Base);
  emxInitMatrix_k_robotics_manip_(pStruct->_pobj0);
}

static void emxInitStruct_robotics_slmanip_(robotics_slmanip_internal_blo_T
  *pStruct)
{
  emxInitStruct_l_robotics_manip_(&pStruct->TreeInternal);
}

static void axes_emxEnsureCapacity_char_T(emxArray_char_T_axes_T *emxArray,
  int32_T oldNumel)
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

    newData = calloc(static_cast<uint32_T>(i), sizeof(char_T));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(char_T) * oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }

    emxArray->data = (char_T *)newData;
    emxArray->allocatedSize = i;
    emxArray->canFreeData = true;
  }
}

static void emxEnsureCapacity_unnamed_struc(emxArray_unnamed_struct_axes_T
  *emxArray, int32_T oldNumel)
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

    newData = calloc(static_cast<uint32_T>(i), sizeof
                     (i_robotics_manip_internal_Col_T *));
    if (emxArray->data != NULL) {
      memcpy(newData, (void *)emxArray->data, sizeof
             (i_robotics_manip_internal_Col_T *) * oldNumel);
      if (emxArray->canFreeData) {
        free((void *)emxArray->data);
      }
    }

    emxArray->data = (i_robotics_manip_internal_Col_T **)newData;
    emxArray->allocatedSize = i;
    emxArray->canFreeData = true;
  }
}

static void axes_emxFree_unnamed_struct(emxArray_unnamed_struct_axes_T
  **pEmxArray)
{
  if (*pEmxArray != (emxArray_unnamed_struct_axes_T *)NULL) {
    if (((*pEmxArray)->data != (i_robotics_manip_internal_Col_T **)NULL) &&
        (*pEmxArray)->canFreeData) {
      free((void *)(*pEmxArray)->data);
    }

    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = (emxArray_unnamed_struct_axes_T *)NULL;
  }
}

static j_robotics_manip_internal_Col_T *axes_CollisionSet_CollisionSet
  (j_robotics_manip_internal_Col_T *obj)
{
  emxArray_unnamed_struct_axes_T *e;
  i_robotics_manip_internal_Col_T *obj_0;
  j_robotics_manip_internal_Col_T *b_obj;
  int32_T b_i;
  axes_emxInit_unnamed_struct(&e, 2);
  b_obj = obj;
  obj->MaxElements = 0.0;
  b_i = e->size[0] * e->size[1];
  e->size[1] = static_cast<int32_T>(obj->MaxElements);
  emxEnsureCapacity_unnamed_struc(e, b_i);
  b_i = obj->CollisionGeometries->size[0] * obj->CollisionGeometries->size[1];
  obj->CollisionGeometries->size[0] = 1;
  obj->CollisionGeometries->size[1] = e->size[1];
  emxEnsureCapacity_unnamed_struc(obj->CollisionGeometries, b_i);
  axes_B.defaultCollisionObj_GeometryInt = 0;
  obj_0 = &obj->_pobj0;
  obj->_pobj0.CollisionPrimitive = axes_B.defaultCollisionObj_GeometryInt;
  obj->_pobj0.matlabCodegenIsDeleted = false;
  axes_B.c = obj->MaxElements;
  axes_B.d_b = static_cast<int32_T>(axes_B.c) - 1;
  axes_emxFree_unnamed_struct(&e);
  for (b_i = 0; b_i <= axes_B.d_b; b_i++) {
    obj->CollisionGeometries->data[b_i] = obj_0;
  }

  return b_obj;
}

static void axes_emxFree_char_T(emxArray_char_T_axes_T **pEmxArray)
{
  if (*pEmxArray != (emxArray_char_T_axes_T *)NULL) {
    if (((*pEmxArray)->data != (char_T *)NULL) && (*pEmxArray)->canFreeData) {
      free((*pEmxArray)->data);
    }

    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = (emxArray_char_T_axes_T *)NULL;
  }
}

static k_robotics_manip_internal_Rig_T *axes_RigidBody_RigidBody
  (k_robotics_manip_internal_Rig_T *obj)
{
  emxArray_char_T_axes_T *switch_expression;
  k_robotics_manip_internal_Rig_T *b_obj;
  boolean_T b_bool;
  static const char_T tmp[5] = { 'B', 'o', 'd', 'y', '4' };

  static const char_T tmp_0[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_1[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const real_T tmp_2[16] = { 1.0, 0.0, -0.0, 0.0, 0.0, -1.0,
    1.2246467991473532E-16, 0.0, 0.0, -1.2246467991473532E-16, -1.0, 0.0,
    1.7725858487190743E-17, -0.15500000000000003, -0.014125000000000037, 1.0 };

  static const real_T tmp_3[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  int32_T exitg1;
  b_obj = obj;
  axes_B.i3 = obj->NameInternal->size[0] * obj->NameInternal->size[1];
  obj->NameInternal->size[0] = 1;
  obj->NameInternal->size[1] = 5;
  axes_emxEnsureCapacity_char_T(obj->NameInternal, axes_B.i3);
  for (axes_B.b_kstr_bn = 0; axes_B.b_kstr_bn < 5; axes_B.b_kstr_bn++) {
    obj->NameInternal->data[axes_B.b_kstr_bn] = tmp[axes_B.b_kstr_bn];
  }

  obj->ParentIndex = 3.0;
  axes_B.i3 = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  axes_emxEnsureCapacity_char_T(obj->JointInternal.Type, axes_B.i3);
  for (axes_B.b_kstr_bn = 0; axes_B.b_kstr_bn < 8; axes_B.b_kstr_bn++) {
    obj->JointInternal.Type->data[axes_B.b_kstr_bn] = tmp_0[axes_B.b_kstr_bn];
  }

  axes_emxInit_char_T(&switch_expression, 2);
  axes_B.i3 = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  axes_emxEnsureCapacity_char_T(switch_expression, axes_B.i3);
  axes_B.loop_ub_d = obj->JointInternal.Type->size[1] - 1;
  for (axes_B.b_kstr_bn = 0; axes_B.b_kstr_bn <= axes_B.loop_ub_d;
       axes_B.b_kstr_bn++) {
    axes_B.i3 = axes_B.b_kstr_bn;
    switch_expression->data[axes_B.i3] = obj->JointInternal.Type->data[axes_B.i3];
  }

  for (axes_B.b_kstr_bn = 0; axes_B.b_kstr_bn < 8; axes_B.b_kstr_bn++) {
    axes_B.b_j[axes_B.b_kstr_bn] = tmp_0[axes_B.b_kstr_bn];
  }

  b_bool = false;
  if (switch_expression->size[1] != 8) {
  } else {
    axes_B.b_kstr_bn = 1;
    do {
      exitg1 = 0;
      if (axes_B.b_kstr_bn - 1 < 8) {
        if (switch_expression->data[axes_B.b_kstr_bn - 1] !=
            axes_B.b_j[axes_B.b_kstr_bn - 1]) {
          exitg1 = 1;
        } else {
          axes_B.b_kstr_bn++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (b_bool) {
    axes_B.b_kstr_bn = 0;
  } else {
    for (axes_B.b_kstr_bn = 0; axes_B.b_kstr_bn < 9; axes_B.b_kstr_bn++) {
      axes_B.b_f[axes_B.b_kstr_bn] = tmp_1[axes_B.b_kstr_bn];
    }

    b_bool = false;
    if (switch_expression->size[1] != 9) {
    } else {
      axes_B.b_kstr_bn = 1;
      do {
        exitg1 = 0;
        if (axes_B.b_kstr_bn - 1 < 9) {
          if (switch_expression->data[axes_B.b_kstr_bn - 1] !=
              axes_B.b_f[axes_B.b_kstr_bn - 1]) {
            exitg1 = 1;
          } else {
            axes_B.b_kstr_bn++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (b_bool) {
      axes_B.b_kstr_bn = 1;
    } else {
      axes_B.b_kstr_bn = -1;
    }
  }

  axes_emxFree_char_T(&switch_expression);
  switch (axes_B.b_kstr_bn) {
   case 0:
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    obj->JointInternal.PositionNumber = 0.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  for (axes_B.b_kstr_bn = 0; axes_B.b_kstr_bn < 16; axes_B.b_kstr_bn++) {
    obj->JointInternal.JointToParentTransform[axes_B.b_kstr_bn] =
      tmp_2[axes_B.b_kstr_bn];
  }

  for (axes_B.b_kstr_bn = 0; axes_B.b_kstr_bn < 16; axes_B.b_kstr_bn++) {
    obj->JointInternal.ChildToJointTransform[axes_B.b_kstr_bn] =
      tmp_3[axes_B.b_kstr_bn];
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 1.0;
  axes_CollisionSet_CollisionSet(&obj->CollisionsInternal);
  return b_obj;
}

static k_robotics_manip_internal_Rig_T *axes_RigidBody_RigidBody_c
  (k_robotics_manip_internal_Rig_T *obj)
{
  emxArray_char_T_axes_T *switch_expression;
  k_robotics_manip_internal_Rig_T *b_obj;
  boolean_T b_bool;
  static const char_T tmp[5] = { 'B', 'o', 'd', 'y', '5' };

  static const char_T tmp_0[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_1[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_2[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const real_T tmp_3[16] = { 6.123233995736766E-17, 1.0, -0.0, 0.0, 1.0,
    -6.123233995736766E-17, 1.2246467991473532E-16, 0.0, 1.2246467991473532E-16,
    -7.498798913309288E-33, -1.0, 0.0, -0.0011000000000000749,
    0.23624000000000006, 0.010499999999999877, 1.0 };

  static const real_T tmp_4[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  int32_T exitg1;
  b_obj = obj;
  axes_B.i2 = obj->NameInternal->size[0] * obj->NameInternal->size[1];
  obj->NameInternal->size[0] = 1;
  obj->NameInternal->size[1] = 5;
  axes_emxEnsureCapacity_char_T(obj->NameInternal, axes_B.i2);
  for (axes_B.b_kstr_l = 0; axes_B.b_kstr_l < 5; axes_B.b_kstr_l++) {
    obj->NameInternal->data[axes_B.b_kstr_l] = tmp[axes_B.b_kstr_l];
  }

  obj->ParentIndex = 4.0;
  axes_B.i2 = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 5;
  axes_emxEnsureCapacity_char_T(obj->JointInternal.Type, axes_B.i2);
  for (axes_B.b_kstr_l = 0; axes_B.b_kstr_l < 5; axes_B.b_kstr_l++) {
    obj->JointInternal.Type->data[axes_B.b_kstr_l] = tmp_0[axes_B.b_kstr_l];
  }

  axes_emxInit_char_T(&switch_expression, 2);
  axes_B.i2 = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  axes_emxEnsureCapacity_char_T(switch_expression, axes_B.i2);
  axes_B.loop_ub_h = obj->JointInternal.Type->size[1] - 1;
  for (axes_B.b_kstr_l = 0; axes_B.b_kstr_l <= axes_B.loop_ub_h; axes_B.b_kstr_l
       ++) {
    axes_B.i2 = axes_B.b_kstr_l;
    switch_expression->data[axes_B.i2] = obj->JointInternal.Type->data[axes_B.i2];
  }

  for (axes_B.b_kstr_l = 0; axes_B.b_kstr_l < 8; axes_B.b_kstr_l++) {
    axes_B.b_l[axes_B.b_kstr_l] = tmp_1[axes_B.b_kstr_l];
  }

  b_bool = false;
  if (switch_expression->size[1] != 8) {
  } else {
    axes_B.b_kstr_l = 1;
    do {
      exitg1 = 0;
      if (axes_B.b_kstr_l - 1 < 8) {
        if (switch_expression->data[axes_B.b_kstr_l - 1] !=
            axes_B.b_l[axes_B.b_kstr_l - 1]) {
          exitg1 = 1;
        } else {
          axes_B.b_kstr_l++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (b_bool) {
    axes_B.b_kstr_l = 0;
  } else {
    for (axes_B.b_kstr_l = 0; axes_B.b_kstr_l < 9; axes_B.b_kstr_l++) {
      axes_B.b_cv[axes_B.b_kstr_l] = tmp_2[axes_B.b_kstr_l];
    }

    b_bool = false;
    if (switch_expression->size[1] != 9) {
    } else {
      axes_B.b_kstr_l = 1;
      do {
        exitg1 = 0;
        if (axes_B.b_kstr_l - 1 < 9) {
          if (switch_expression->data[axes_B.b_kstr_l - 1] !=
              axes_B.b_cv[axes_B.b_kstr_l - 1]) {
            exitg1 = 1;
          } else {
            axes_B.b_kstr_l++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (b_bool) {
      axes_B.b_kstr_l = 1;
    } else {
      axes_B.b_kstr_l = -1;
    }
  }

  axes_emxFree_char_T(&switch_expression);
  switch (axes_B.b_kstr_l) {
   case 0:
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    obj->JointInternal.PositionNumber = 0.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  for (axes_B.b_kstr_l = 0; axes_B.b_kstr_l < 16; axes_B.b_kstr_l++) {
    obj->JointInternal.JointToParentTransform[axes_B.b_kstr_l] =
      tmp_3[axes_B.b_kstr_l];
  }

  for (axes_B.b_kstr_l = 0; axes_B.b_kstr_l < 16; axes_B.b_kstr_l++) {
    obj->JointInternal.ChildToJointTransform[axes_B.b_kstr_l] =
      tmp_4[axes_B.b_kstr_l];
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 0.0;
  axes_CollisionSet_CollisionSet(&obj->CollisionsInternal);
  return b_obj;
}

static k_robotics_manip_internal_Rig_T *axes_RigidBody_RigidBody_cu
  (k_robotics_manip_internal_Rig_T *obj)
{
  emxArray_char_T_axes_T *switch_expression;
  k_robotics_manip_internal_Rig_T *b_obj;
  boolean_T b_bool;
  static const char_T tmp[5] = { 'B', 'o', 'd', 'y', '6' };

  static const char_T tmp_0[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_1[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_2[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const real_T tmp_3[16] = { 2.1073424178067666E-8, 0.0,
    0.99999999999999978, 0.0, 0.99999999999999978, 2.83276944882399E-16,
    -2.1073424178067666E-8, 0.0, -2.8327694488239893E-16, 1.0,
    5.9696152193738883E-24, 0.0, -0.055500000000000022, 0.08049999999999996,
    0.010499999999999971, 1.0 };

  static const real_T tmp_4[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  int32_T exitg1;
  b_obj = obj;
  axes_B.i1 = obj->NameInternal->size[0] * obj->NameInternal->size[1];
  obj->NameInternal->size[0] = 1;
  obj->NameInternal->size[1] = 5;
  axes_emxEnsureCapacity_char_T(obj->NameInternal, axes_B.i1);
  for (axes_B.b_kstr_b = 0; axes_B.b_kstr_b < 5; axes_B.b_kstr_b++) {
    obj->NameInternal->data[axes_B.b_kstr_b] = tmp[axes_B.b_kstr_b];
  }

  obj->ParentIndex = 4.0;
  axes_B.i1 = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 5;
  axes_emxEnsureCapacity_char_T(obj->JointInternal.Type, axes_B.i1);
  for (axes_B.b_kstr_b = 0; axes_B.b_kstr_b < 5; axes_B.b_kstr_b++) {
    obj->JointInternal.Type->data[axes_B.b_kstr_b] = tmp_0[axes_B.b_kstr_b];
  }

  axes_emxInit_char_T(&switch_expression, 2);
  axes_B.i1 = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  axes_emxEnsureCapacity_char_T(switch_expression, axes_B.i1);
  axes_B.loop_ub_n = obj->JointInternal.Type->size[1] - 1;
  for (axes_B.b_kstr_b = 0; axes_B.b_kstr_b <= axes_B.loop_ub_n; axes_B.b_kstr_b
       ++) {
    axes_B.i1 = axes_B.b_kstr_b;
    switch_expression->data[axes_B.i1] = obj->JointInternal.Type->data[axes_B.i1];
  }

  for (axes_B.b_kstr_b = 0; axes_B.b_kstr_b < 8; axes_B.b_kstr_b++) {
    axes_B.b_n[axes_B.b_kstr_b] = tmp_1[axes_B.b_kstr_b];
  }

  b_bool = false;
  if (switch_expression->size[1] != 8) {
  } else {
    axes_B.b_kstr_b = 1;
    do {
      exitg1 = 0;
      if (axes_B.b_kstr_b - 1 < 8) {
        if (switch_expression->data[axes_B.b_kstr_b - 1] !=
            axes_B.b_n[axes_B.b_kstr_b - 1]) {
          exitg1 = 1;
        } else {
          axes_B.b_kstr_b++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (b_bool) {
    axes_B.b_kstr_b = 0;
  } else {
    for (axes_B.b_kstr_b = 0; axes_B.b_kstr_b < 9; axes_B.b_kstr_b++) {
      axes_B.b_p[axes_B.b_kstr_b] = tmp_2[axes_B.b_kstr_b];
    }

    b_bool = false;
    if (switch_expression->size[1] != 9) {
    } else {
      axes_B.b_kstr_b = 1;
      do {
        exitg1 = 0;
        if (axes_B.b_kstr_b - 1 < 9) {
          if (switch_expression->data[axes_B.b_kstr_b - 1] !=
              axes_B.b_p[axes_B.b_kstr_b - 1]) {
            exitg1 = 1;
          } else {
            axes_B.b_kstr_b++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (b_bool) {
      axes_B.b_kstr_b = 1;
    } else {
      axes_B.b_kstr_b = -1;
    }
  }

  axes_emxFree_char_T(&switch_expression);
  switch (axes_B.b_kstr_b) {
   case 0:
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    obj->JointInternal.PositionNumber = 0.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  for (axes_B.b_kstr_b = 0; axes_B.b_kstr_b < 16; axes_B.b_kstr_b++) {
    obj->JointInternal.JointToParentTransform[axes_B.b_kstr_b] =
      tmp_3[axes_B.b_kstr_b];
  }

  for (axes_B.b_kstr_b = 0; axes_B.b_kstr_b < 16; axes_B.b_kstr_b++) {
    obj->JointInternal.ChildToJointTransform[axes_B.b_kstr_b] =
      tmp_4[axes_B.b_kstr_b];
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 0.0;
  axes_CollisionSet_CollisionSet(&obj->CollisionsInternal);
  return b_obj;
}

static void axes_SystemCore_setup(robotics_slmanip_internal_blo_T *obj)
{
  emxArray_char_T_axes_T *switch_expression;
  l_robotics_manip_internal_Rig_T *obj_0;
  boolean_T b_bool;
  static const char_T tmp[5] = { 'B', 'o', 'd', 'y', '1' };

  static const char_T tmp_0[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_1[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const real_T tmp_2[16] = { 1.0, 0.0, -0.0, 0.0, 0.0, -1.0,
    1.2246467991473532E-16, 0.0, 0.0, -1.2246467991473532E-16, -1.0, 0.0,
    0.060164888, 0.059081195000000031, 0.069308911, 1.0 };

  static const real_T tmp_3[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  static const char_T tmp_4[5] = { 'B', 'o', 'd', 'y', '2' };

  static const real_T tmp_5[16] = { -1.0, 1.2246467991473532E-16, -0.0, 0.0,
    -7.498798913309288E-33, -6.123233995736766E-17, -1.0, 0.0,
    -1.2246467991473532E-16, -1.0, 6.123233995736766E-17, 0.0,
    -1.3877787807814457E-17, -0.018000000000000044, -0.11600000000000002, 1.0 };

  static const char_T tmp_6[5] = { 'B', 'o', 'd', 'y', '3' };

  static const real_T tmp_7[16] = { 6.123233995736766E-17, -1.0, -0.0, 0.0, -1.0,
    -6.123233995736766E-17, 1.2246467991473532E-16, 0.0, -1.2246467991473532E-16,
    -7.498798913309288E-33, -1.0, 0.0, -3.5971225173156E-17, 0.16,
    -0.042000000000000058, 1.0 };

  static const char_T tmp_8[5] = { 'f', 'i', 'x', 'e', 'd' };

  int32_T exitg1;
  obj->isInitialized = 1;
  obj_0 = &obj->TreeInternal;
  axes_B.i = obj->TreeInternal._pobj0[0].NameInternal->size[0] *
    obj->TreeInternal._pobj0[0].NameInternal->size[1];
  obj->TreeInternal._pobj0[0].NameInternal->size[0] = 1;
  obj->TreeInternal._pobj0[0].NameInternal->size[1] = 5;
  axes_emxEnsureCapacity_char_T(obj->TreeInternal._pobj0[0].NameInternal,
    axes_B.i);
  for (axes_B.b_kstr = 0; axes_B.b_kstr < 5; axes_B.b_kstr++) {
    obj->TreeInternal._pobj0[0].NameInternal->data[axes_B.b_kstr] =
      tmp[axes_B.b_kstr];
  }

  obj->TreeInternal._pobj0[0].ParentIndex = 0.0;
  axes_B.i = obj->TreeInternal._pobj0[0].JointInternal.Type->size[0] *
    obj->TreeInternal._pobj0[0].JointInternal.Type->size[1];
  obj->TreeInternal._pobj0[0].JointInternal.Type->size[0] = 1;
  obj->TreeInternal._pobj0[0].JointInternal.Type->size[1] = 8;
  axes_emxEnsureCapacity_char_T(obj->TreeInternal._pobj0[0].JointInternal.Type,
    axes_B.i);
  for (axes_B.b_kstr = 0; axes_B.b_kstr < 8; axes_B.b_kstr++) {
    obj->TreeInternal._pobj0[0].JointInternal.Type->data[axes_B.b_kstr] =
      tmp_0[axes_B.b_kstr];
  }

  axes_emxInit_char_T(&switch_expression, 2);
  axes_B.i = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->TreeInternal._pobj0[0]
    .JointInternal.Type->size[1];
  axes_emxEnsureCapacity_char_T(switch_expression, axes_B.i);
  axes_B.loop_ub = obj->TreeInternal._pobj0[0].JointInternal.Type->size[1] - 1;
  for (axes_B.b_kstr = 0; axes_B.b_kstr <= axes_B.loop_ub; axes_B.b_kstr++) {
    axes_B.i = axes_B.b_kstr;
    switch_expression->data[axes_B.i] = obj->TreeInternal._pobj0[0].
      JointInternal.Type->data[axes_B.i];
  }

  for (axes_B.b_kstr = 0; axes_B.b_kstr < 8; axes_B.b_kstr++) {
    axes_B.b_m[axes_B.b_kstr] = tmp_0[axes_B.b_kstr];
  }

  b_bool = false;
  if (switch_expression->size[1] != 8) {
  } else {
    axes_B.b_kstr = 1;
    do {
      exitg1 = 0;
      if (axes_B.b_kstr - 1 < 8) {
        if (switch_expression->data[axes_B.b_kstr - 1] !=
            axes_B.b_m[axes_B.b_kstr - 1]) {
          exitg1 = 1;
        } else {
          axes_B.b_kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (b_bool) {
    axes_B.b_kstr = 0;
  } else {
    for (axes_B.b_kstr = 0; axes_B.b_kstr < 9; axes_B.b_kstr++) {
      axes_B.b_b[axes_B.b_kstr] = tmp_1[axes_B.b_kstr];
    }

    b_bool = false;
    if (switch_expression->size[1] != 9) {
    } else {
      axes_B.b_kstr = 1;
      do {
        exitg1 = 0;
        if (axes_B.b_kstr - 1 < 9) {
          if (switch_expression->data[axes_B.b_kstr - 1] !=
              axes_B.b_b[axes_B.b_kstr - 1]) {
            exitg1 = 1;
          } else {
            axes_B.b_kstr++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (b_bool) {
      axes_B.b_kstr = 1;
    } else {
      axes_B.b_kstr = -1;
    }
  }

  switch (axes_B.b_kstr) {
   case 0:
    obj->TreeInternal._pobj0[0].JointInternal.PositionNumber = 1.0;
    obj->TreeInternal._pobj0[0].JointInternal.JointAxisInternal[0] = 0.0;
    obj->TreeInternal._pobj0[0].JointInternal.JointAxisInternal[1] = 0.0;
    obj->TreeInternal._pobj0[0].JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    obj->TreeInternal._pobj0[0].JointInternal.PositionNumber = 1.0;
    obj->TreeInternal._pobj0[0].JointInternal.JointAxisInternal[0] = 0.0;
    obj->TreeInternal._pobj0[0].JointInternal.JointAxisInternal[1] = 0.0;
    obj->TreeInternal._pobj0[0].JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    obj->TreeInternal._pobj0[0].JointInternal.PositionNumber = 0.0;
    obj->TreeInternal._pobj0[0].JointInternal.JointAxisInternal[0] = 0.0;
    obj->TreeInternal._pobj0[0].JointInternal.JointAxisInternal[1] = 0.0;
    obj->TreeInternal._pobj0[0].JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  for (axes_B.b_kstr = 0; axes_B.b_kstr < 16; axes_B.b_kstr++) {
    obj_0->_pobj0[0].JointInternal.JointToParentTransform[axes_B.b_kstr] =
      tmp_2[axes_B.b_kstr];
  }

  for (axes_B.b_kstr = 0; axes_B.b_kstr < 16; axes_B.b_kstr++) {
    obj_0->_pobj0[0].JointInternal.ChildToJointTransform[axes_B.b_kstr] =
      tmp_3[axes_B.b_kstr];
  }

  obj_0->_pobj0[0].JointInternal.JointAxisInternal[0] = 0.0;
  obj_0->_pobj0[0].JointInternal.JointAxisInternal[1] = 0.0;
  obj_0->_pobj0[0].JointInternal.JointAxisInternal[2] = 1.0;
  axes_CollisionSet_CollisionSet(&obj_0->_pobj0[0].CollisionsInternal);
  obj->TreeInternal.Bodies[0] = &obj_0->_pobj0[0];
  axes_B.i = obj_0->_pobj0[1].NameInternal->size[0] * obj_0->_pobj0[1].
    NameInternal->size[1];
  obj_0->_pobj0[1].NameInternal->size[0] = 1;
  obj_0->_pobj0[1].NameInternal->size[1] = 5;
  axes_emxEnsureCapacity_char_T(obj_0->_pobj0[1].NameInternal, axes_B.i);
  for (axes_B.b_kstr = 0; axes_B.b_kstr < 5; axes_B.b_kstr++) {
    obj_0->_pobj0[1].NameInternal->data[axes_B.b_kstr] = tmp_4[axes_B.b_kstr];
  }

  obj_0->_pobj0[1].ParentIndex = 1.0;
  axes_B.i = obj->TreeInternal._pobj0[1].JointInternal.Type->size[0] *
    obj->TreeInternal._pobj0[1].JointInternal.Type->size[1];
  obj->TreeInternal._pobj0[1].JointInternal.Type->size[0] = 1;
  obj->TreeInternal._pobj0[1].JointInternal.Type->size[1] = 8;
  axes_emxEnsureCapacity_char_T(obj->TreeInternal._pobj0[1].JointInternal.Type,
    axes_B.i);
  for (axes_B.b_kstr = 0; axes_B.b_kstr < 8; axes_B.b_kstr++) {
    obj->TreeInternal._pobj0[1].JointInternal.Type->data[axes_B.b_kstr] =
      tmp_0[axes_B.b_kstr];
  }

  axes_B.i = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->TreeInternal._pobj0[1]
    .JointInternal.Type->size[1];
  axes_emxEnsureCapacity_char_T(switch_expression, axes_B.i);
  axes_B.loop_ub = obj->TreeInternal._pobj0[1].JointInternal.Type->size[1] - 1;
  for (axes_B.b_kstr = 0; axes_B.b_kstr <= axes_B.loop_ub; axes_B.b_kstr++) {
    axes_B.i = axes_B.b_kstr;
    switch_expression->data[axes_B.i] = obj->TreeInternal._pobj0[1].
      JointInternal.Type->data[axes_B.i];
  }

  for (axes_B.b_kstr = 0; axes_B.b_kstr < 8; axes_B.b_kstr++) {
    axes_B.b_m[axes_B.b_kstr] = tmp_0[axes_B.b_kstr];
  }

  b_bool = false;
  if (switch_expression->size[1] != 8) {
  } else {
    axes_B.b_kstr = 1;
    do {
      exitg1 = 0;
      if (axes_B.b_kstr - 1 < 8) {
        if (switch_expression->data[axes_B.b_kstr - 1] !=
            axes_B.b_m[axes_B.b_kstr - 1]) {
          exitg1 = 1;
        } else {
          axes_B.b_kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (b_bool) {
    axes_B.b_kstr = 0;
  } else {
    for (axes_B.b_kstr = 0; axes_B.b_kstr < 9; axes_B.b_kstr++) {
      axes_B.b_b[axes_B.b_kstr] = tmp_1[axes_B.b_kstr];
    }

    b_bool = false;
    if (switch_expression->size[1] != 9) {
    } else {
      axes_B.b_kstr = 1;
      do {
        exitg1 = 0;
        if (axes_B.b_kstr - 1 < 9) {
          if (switch_expression->data[axes_B.b_kstr - 1] !=
              axes_B.b_b[axes_B.b_kstr - 1]) {
            exitg1 = 1;
          } else {
            axes_B.b_kstr++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (b_bool) {
      axes_B.b_kstr = 1;
    } else {
      axes_B.b_kstr = -1;
    }
  }

  switch (axes_B.b_kstr) {
   case 0:
    obj->TreeInternal._pobj0[1].JointInternal.PositionNumber = 1.0;
    obj->TreeInternal._pobj0[1].JointInternal.JointAxisInternal[0] = 0.0;
    obj->TreeInternal._pobj0[1].JointInternal.JointAxisInternal[1] = 0.0;
    obj->TreeInternal._pobj0[1].JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    obj->TreeInternal._pobj0[1].JointInternal.PositionNumber = 1.0;
    obj->TreeInternal._pobj0[1].JointInternal.JointAxisInternal[0] = 0.0;
    obj->TreeInternal._pobj0[1].JointInternal.JointAxisInternal[1] = 0.0;
    obj->TreeInternal._pobj0[1].JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    obj->TreeInternal._pobj0[1].JointInternal.PositionNumber = 0.0;
    obj->TreeInternal._pobj0[1].JointInternal.JointAxisInternal[0] = 0.0;
    obj->TreeInternal._pobj0[1].JointInternal.JointAxisInternal[1] = 0.0;
    obj->TreeInternal._pobj0[1].JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  for (axes_B.b_kstr = 0; axes_B.b_kstr < 16; axes_B.b_kstr++) {
    obj_0->_pobj0[1].JointInternal.JointToParentTransform[axes_B.b_kstr] =
      tmp_5[axes_B.b_kstr];
  }

  for (axes_B.b_kstr = 0; axes_B.b_kstr < 16; axes_B.b_kstr++) {
    obj_0->_pobj0[1].JointInternal.ChildToJointTransform[axes_B.b_kstr] =
      tmp_3[axes_B.b_kstr];
  }

  obj_0->_pobj0[1].JointInternal.JointAxisInternal[0] = 0.0;
  obj_0->_pobj0[1].JointInternal.JointAxisInternal[1] = 0.0;
  obj_0->_pobj0[1].JointInternal.JointAxisInternal[2] = 1.0;
  axes_CollisionSet_CollisionSet(&obj_0->_pobj0[1].CollisionsInternal);
  obj->TreeInternal.Bodies[1] = &obj_0->_pobj0[1];
  axes_B.i = obj_0->_pobj0[2].NameInternal->size[0] * obj_0->_pobj0[2].
    NameInternal->size[1];
  obj_0->_pobj0[2].NameInternal->size[0] = 1;
  obj_0->_pobj0[2].NameInternal->size[1] = 5;
  axes_emxEnsureCapacity_char_T(obj_0->_pobj0[2].NameInternal, axes_B.i);
  for (axes_B.b_kstr = 0; axes_B.b_kstr < 5; axes_B.b_kstr++) {
    obj_0->_pobj0[2].NameInternal->data[axes_B.b_kstr] = tmp_6[axes_B.b_kstr];
  }

  obj_0->_pobj0[2].ParentIndex = 2.0;
  axes_B.i = obj->TreeInternal._pobj0[2].JointInternal.Type->size[0] *
    obj->TreeInternal._pobj0[2].JointInternal.Type->size[1];
  obj->TreeInternal._pobj0[2].JointInternal.Type->size[0] = 1;
  obj->TreeInternal._pobj0[2].JointInternal.Type->size[1] = 8;
  axes_emxEnsureCapacity_char_T(obj->TreeInternal._pobj0[2].JointInternal.Type,
    axes_B.i);
  for (axes_B.b_kstr = 0; axes_B.b_kstr < 8; axes_B.b_kstr++) {
    obj->TreeInternal._pobj0[2].JointInternal.Type->data[axes_B.b_kstr] =
      tmp_0[axes_B.b_kstr];
  }

  axes_B.i = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->TreeInternal._pobj0[2]
    .JointInternal.Type->size[1];
  axes_emxEnsureCapacity_char_T(switch_expression, axes_B.i);
  axes_B.loop_ub = obj->TreeInternal._pobj0[2].JointInternal.Type->size[1] - 1;
  for (axes_B.b_kstr = 0; axes_B.b_kstr <= axes_B.loop_ub; axes_B.b_kstr++) {
    axes_B.i = axes_B.b_kstr;
    switch_expression->data[axes_B.i] = obj->TreeInternal._pobj0[2].
      JointInternal.Type->data[axes_B.i];
  }

  for (axes_B.b_kstr = 0; axes_B.b_kstr < 8; axes_B.b_kstr++) {
    axes_B.b_m[axes_B.b_kstr] = tmp_0[axes_B.b_kstr];
  }

  b_bool = false;
  if (switch_expression->size[1] != 8) {
  } else {
    axes_B.b_kstr = 1;
    do {
      exitg1 = 0;
      if (axes_B.b_kstr - 1 < 8) {
        if (switch_expression->data[axes_B.b_kstr - 1] !=
            axes_B.b_m[axes_B.b_kstr - 1]) {
          exitg1 = 1;
        } else {
          axes_B.b_kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (b_bool) {
    axes_B.b_kstr = 0;
  } else {
    for (axes_B.b_kstr = 0; axes_B.b_kstr < 9; axes_B.b_kstr++) {
      axes_B.b_b[axes_B.b_kstr] = tmp_1[axes_B.b_kstr];
    }

    b_bool = false;
    if (switch_expression->size[1] != 9) {
    } else {
      axes_B.b_kstr = 1;
      do {
        exitg1 = 0;
        if (axes_B.b_kstr - 1 < 9) {
          if (switch_expression->data[axes_B.b_kstr - 1] !=
              axes_B.b_b[axes_B.b_kstr - 1]) {
            exitg1 = 1;
          } else {
            axes_B.b_kstr++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (b_bool) {
      axes_B.b_kstr = 1;
    } else {
      axes_B.b_kstr = -1;
    }
  }

  switch (axes_B.b_kstr) {
   case 0:
    obj->TreeInternal._pobj0[2].JointInternal.PositionNumber = 1.0;
    obj->TreeInternal._pobj0[2].JointInternal.JointAxisInternal[0] = 0.0;
    obj->TreeInternal._pobj0[2].JointInternal.JointAxisInternal[1] = 0.0;
    obj->TreeInternal._pobj0[2].JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    obj->TreeInternal._pobj0[2].JointInternal.PositionNumber = 1.0;
    obj->TreeInternal._pobj0[2].JointInternal.JointAxisInternal[0] = 0.0;
    obj->TreeInternal._pobj0[2].JointInternal.JointAxisInternal[1] = 0.0;
    obj->TreeInternal._pobj0[2].JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    obj->TreeInternal._pobj0[2].JointInternal.PositionNumber = 0.0;
    obj->TreeInternal._pobj0[2].JointInternal.JointAxisInternal[0] = 0.0;
    obj->TreeInternal._pobj0[2].JointInternal.JointAxisInternal[1] = 0.0;
    obj->TreeInternal._pobj0[2].JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  for (axes_B.b_kstr = 0; axes_B.b_kstr < 16; axes_B.b_kstr++) {
    obj_0->_pobj0[2].JointInternal.JointToParentTransform[axes_B.b_kstr] =
      tmp_7[axes_B.b_kstr];
  }

  for (axes_B.b_kstr = 0; axes_B.b_kstr < 16; axes_B.b_kstr++) {
    obj_0->_pobj0[2].JointInternal.ChildToJointTransform[axes_B.b_kstr] =
      tmp_3[axes_B.b_kstr];
  }

  obj_0->_pobj0[2].JointInternal.JointAxisInternal[0] = 0.0;
  obj_0->_pobj0[2].JointInternal.JointAxisInternal[1] = 0.0;
  obj_0->_pobj0[2].JointInternal.JointAxisInternal[2] = 1.0;
  axes_CollisionSet_CollisionSet(&obj_0->_pobj0[2].CollisionsInternal);
  obj->TreeInternal.Bodies[2] = &obj_0->_pobj0[2];
  obj->TreeInternal.Bodies[3] = axes_RigidBody_RigidBody
    (&obj->TreeInternal._pobj0[3]);
  obj->TreeInternal.Bodies[4] = axes_RigidBody_RigidBody_c
    (&obj->TreeInternal._pobj0[4]);
  obj->TreeInternal.Bodies[5] = axes_RigidBody_RigidBody_cu
    (&obj->TreeInternal._pobj0[5]);
  obj->TreeInternal.NumBodies = 6.0;
  obj->TreeInternal.PositionNumber = 4.0;
  axes_B.i = obj_0->Base.NameInternal->size[0] * obj_0->Base.NameInternal->size
    [1];
  obj_0->Base.NameInternal->size[0] = 1;
  obj_0->Base.NameInternal->size[1] = 4;
  axes_emxEnsureCapacity_char_T(obj_0->Base.NameInternal, axes_B.i);
  obj_0->Base.NameInternal->data[0] = 'B';
  obj_0->Base.NameInternal->data[1] = 'a';
  obj_0->Base.NameInternal->data[2] = 's';
  obj_0->Base.NameInternal->data[3] = 'e';
  obj_0->Base.ParentIndex = -1.0;
  axes_B.i = obj->TreeInternal.Base.JointInternal.Type->size[0] *
    obj->TreeInternal.Base.JointInternal.Type->size[1];
  obj->TreeInternal.Base.JointInternal.Type->size[0] = 1;
  obj->TreeInternal.Base.JointInternal.Type->size[1] = 5;
  axes_emxEnsureCapacity_char_T(obj->TreeInternal.Base.JointInternal.Type,
    axes_B.i);
  for (axes_B.b_kstr = 0; axes_B.b_kstr < 5; axes_B.b_kstr++) {
    obj->TreeInternal.Base.JointInternal.Type->data[axes_B.b_kstr] =
      tmp_8[axes_B.b_kstr];
  }

  axes_B.i = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->TreeInternal.Base.JointInternal.Type->size[1];
  axes_emxEnsureCapacity_char_T(switch_expression, axes_B.i);
  axes_B.loop_ub = obj->TreeInternal.Base.JointInternal.Type->size[1] - 1;
  for (axes_B.b_kstr = 0; axes_B.b_kstr <= axes_B.loop_ub; axes_B.b_kstr++) {
    axes_B.i = axes_B.b_kstr;
    switch_expression->data[axes_B.i] =
      obj->TreeInternal.Base.JointInternal.Type->data[axes_B.i];
  }

  for (axes_B.b_kstr = 0; axes_B.b_kstr < 8; axes_B.b_kstr++) {
    axes_B.b_m[axes_B.b_kstr] = tmp_0[axes_B.b_kstr];
  }

  b_bool = false;
  if (switch_expression->size[1] != 8) {
  } else {
    axes_B.b_kstr = 1;
    do {
      exitg1 = 0;
      if (axes_B.b_kstr - 1 < 8) {
        if (switch_expression->data[axes_B.b_kstr - 1] !=
            axes_B.b_m[axes_B.b_kstr - 1]) {
          exitg1 = 1;
        } else {
          axes_B.b_kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (b_bool) {
    axes_B.b_kstr = 0;
  } else {
    for (axes_B.b_kstr = 0; axes_B.b_kstr < 9; axes_B.b_kstr++) {
      axes_B.b_b[axes_B.b_kstr] = tmp_1[axes_B.b_kstr];
    }

    b_bool = false;
    if (switch_expression->size[1] != 9) {
    } else {
      axes_B.b_kstr = 1;
      do {
        exitg1 = 0;
        if (axes_B.b_kstr - 1 < 9) {
          if (switch_expression->data[axes_B.b_kstr - 1] !=
              axes_B.b_b[axes_B.b_kstr - 1]) {
            exitg1 = 1;
          } else {
            axes_B.b_kstr++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (b_bool) {
      axes_B.b_kstr = 1;
    } else {
      axes_B.b_kstr = -1;
    }
  }

  axes_emxFree_char_T(&switch_expression);
  switch (axes_B.b_kstr) {
   case 0:
    obj->TreeInternal.Base.JointInternal.PositionNumber = 1.0;
    obj->TreeInternal.Base.JointInternal.JointAxisInternal[0] = 0.0;
    obj->TreeInternal.Base.JointInternal.JointAxisInternal[1] = 0.0;
    obj->TreeInternal.Base.JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    obj->TreeInternal.Base.JointInternal.PositionNumber = 1.0;
    obj->TreeInternal.Base.JointInternal.JointAxisInternal[0] = 0.0;
    obj->TreeInternal.Base.JointInternal.JointAxisInternal[1] = 0.0;
    obj->TreeInternal.Base.JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    obj->TreeInternal.Base.JointInternal.PositionNumber = 0.0;
    obj->TreeInternal.Base.JointInternal.JointAxisInternal[0] = 0.0;
    obj->TreeInternal.Base.JointInternal.JointAxisInternal[1] = 0.0;
    obj->TreeInternal.Base.JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  axes_CollisionSet_CollisionSet(&obj_0->Base.CollisionsInternal);
}

static void axes_emxInit_d_cell_wrap(emxArray_d_cell_wrap_axes_T **pEmxArray,
  int32_T numDimensions)
{
  emxArray_d_cell_wrap_axes_T *emxArray;
  *pEmxArray = (emxArray_d_cell_wrap_axes_T *)malloc(sizeof
    (emxArray_d_cell_wrap_axes_T));
  emxArray = *pEmxArray;
  emxArray->data = (d_cell_wrap_axes_T *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int32_T *)malloc(sizeof(int32_T) * numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (int32_T i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

static void a_emxEnsureCapacity_d_cell_wrap(emxArray_d_cell_wrap_axes_T
  *emxArray, int32_T oldNumel)
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

    newData = calloc(static_cast<uint32_T>(i), sizeof(d_cell_wrap_axes_T));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(d_cell_wrap_axes_T) * oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }

    emxArray->data = (d_cell_wrap_axes_T *)newData;
    emxArray->allocatedSize = i;
    emxArray->canFreeData = true;
  }
}

static void ax_rigidBodyJoint_get_JointAxis(const c_rigidBodyJoint_axes_T *obj,
  real_T ax[3])
{
  int32_T b_kstr;
  boolean_T b_bool;
  static const char_T tmp[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_0[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  int32_T exitg1;
  boolean_T guard1 = false;
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    axes_B.b_g1[b_kstr] = tmp[b_kstr];
  }

  b_bool = false;
  if (obj->Type->size[1] != 8) {
  } else {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        if (obj->Type->data[b_kstr - 1] != axes_B.b_g1[b_kstr - 1]) {
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
      axes_B.b_c[b_kstr] = tmp_0[b_kstr];
    }

    b_bool = false;
    if (obj->Type->size[1] != 9) {
    } else {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          if (obj->Type->data[b_kstr - 1] != axes_B.b_c[b_kstr - 1]) {
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

static void axes_emxFree_d_cell_wrap(emxArray_d_cell_wrap_axes_T **pEmxArray)
{
  if (*pEmxArray != (emxArray_d_cell_wrap_axes_T *)NULL) {
    if (((*pEmxArray)->data != (d_cell_wrap_axes_T *)NULL) && (*pEmxArray)
        ->canFreeData) {
      free((*pEmxArray)->data);
    }

    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = (emxArray_d_cell_wrap_axes_T *)NULL;
  }
}

static void emxFreeStruct_c_rigidBodyJoint(c_rigidBodyJoint_axes_T *pStruct)
{
  axes_emxFree_char_T(&pStruct->Type);
}

static void emxFreeStruct_j_robotics_manip_(j_robotics_manip_internal_Col_T
  *pStruct)
{
  axes_emxFree_unnamed_struct(&pStruct->CollisionGeometries);
}

static void emxFreeStruct_k_robotics_manip_(k_robotics_manip_internal_Rig_T
  *pStruct)
{
  axes_emxFree_char_T(&pStruct->NameInternal);
  emxFreeStruct_c_rigidBodyJoint(&pStruct->JointInternal);
  emxFreeStruct_j_robotics_manip_(&pStruct->CollisionsInternal);
}

static void emxFreeMatrix_k_robotics_manip_(k_robotics_manip_internal_Rig_T
  pMatrix[6])
{
  for (int32_T i = 0; i < 6; i++) {
    emxFreeStruct_k_robotics_manip_(&pMatrix[i]);
  }
}

static void emxFreeStruct_l_robotics_manip_(l_robotics_manip_internal_Rig_T
  *pStruct)
{
  emxFreeStruct_k_robotics_manip_(&pStruct->Base);
  emxFreeMatrix_k_robotics_manip_(pStruct->_pobj0);
}

static void emxFreeStruct_robotics_slmanip_(robotics_slmanip_internal_blo_T
  *pStruct)
{
  emxFreeStruct_l_robotics_manip_(&pStruct->TreeInternal);
}

/* Model step function */
void axes_step(void)
{
  SL_Bus_axes_std_msgs_Int16 b_varargout_2;
  emxArray_char_T_axes_T *switch_expression;
  emxArray_d_cell_wrap_axes_T *Ttree;
  k_robotics_manip_internal_Rig_T *body;
  l_robotics_manip_internal_Rig_T *obj;
  int8_T rtPrevAction;
  boolean_T b_varargout_1;
  static const int8_T tmp[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1
  };

  static const char_T tmp_0[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_1[5] = { 'B', 'o', 'd', 'y', '6' };

  static const char_T tmp_2[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  /* Outputs for Atomic SubSystem: '<Root>/Subscribe1' */
  /* MATLABSystem: '<S3>/SourceBlock' incorporates:
   *  Inport: '<S18>/In1'
   */
  b_varargout_1 = Sub_axes_64.getLatestMessage(&b_varargout_2);

  /* Outputs for Enabled SubSystem: '<S3>/Enabled Subsystem' incorporates:
   *  EnablePort: '<S18>/Enable'
   */
  if (b_varargout_1) {
    axes_B.In1_h = b_varargout_2;
  }

  /* End of MATLABSystem: '<S3>/SourceBlock' */
  /* End of Outputs for SubSystem: '<S3>/Enabled Subsystem' */
  /* End of Outputs for SubSystem: '<Root>/Subscribe1' */

  /* If: '<Root>/If' */
  rtPrevAction = axes_DW.If_ActiveSubsystem;
  axes_DW.If_ActiveSubsystem = static_cast<int8_T>((axes_B.In1_h.Data > 5) ||
    (axes_B.In1_h.Data <= 0));
  if ((rtPrevAction != axes_DW.If_ActiveSubsystem) && (rtPrevAction == 0)) {
    /* If: '<S1>/If' */
    axes_DW.If_ActiveSubsystem_d = -1;
  }

  if (axes_DW.If_ActiveSubsystem == 0) {
    /* Outputs for IfAction SubSystem: '<Root>/1 2' incorporates:
     *  ActionPort: '<S1>/Action'
     */
    /* If: '<S1>/If' incorporates:
     *  If: '<S4>/If'
     *  Inport: '<S12>/In1'
     *  MATLABSystem: '<S10>/SourceBlock'
     *  MATLABSystem: '<S13>/SourceBlock'
     *  MATLABSystem: '<S9>/MATLAB System'
     */
    axes_DW.If_ActiveSubsystem_d = static_cast<int8_T>(axes_B.In1_h.Data != 1);
    if (axes_DW.If_ActiveSubsystem_d == 0) {
      int32_T T1_tmp;
      int32_T b_tmp;
      int32_T exitg1;
      int32_T loop_ub;
      boolean_T exitg2;

      /* Outputs for IfAction SubSystem: '<S1>/Deplacement' incorporates:
       *  ActionPort: '<S5>/Action'
       */
      /* Outputs for Atomic SubSystem: '<S11>/Subscribe' */
      /* MATLABSystem: '<S13>/SourceBlock' */
      b_varargout_1 = Sub_axes_47.getLatestMessage(&axes_B.b_varargout_2_c);

      /* Outputs for Enabled SubSystem: '<S11>/Subsystem' incorporates:
       *  EnablePort: '<S14>/Enable'
       */
      /* Outputs for Enabled SubSystem: '<S13>/Enabled Subsystem' incorporates:
       *  EnablePort: '<S15>/Enable'
       */
      if (b_varargout_1) {
        /* SignalConversion generated from: '<S14>/Bus Selector' */
        axes_B.X = axes_B.b_varargout_2_c.X;

        /* SignalConversion generated from: '<S14>/Bus Selector' */
        axes_B.Y = axes_B.b_varargout_2_c.Y;

        /* SignalConversion generated from: '<S14>/Bus Selector' */
        axes_B.Z = axes_B.b_varargout_2_c.Z;

        /* SignalConversion generated from: '<S14>/Bus Selector' */
        axes_B.W = axes_B.b_varargout_2_c.W;
      }

      /* End of Outputs for SubSystem: '<S13>/Enabled Subsystem' */
      /* End of Outputs for SubSystem: '<S11>/Subsystem' */
      /* End of Outputs for SubSystem: '<S11>/Subscribe' */

      /* SignalConversion generated from: '<S9>/MATLAB System' incorporates:
       *  MATLABSystem: '<S13>/SourceBlock'
       */
      axes_B.TmpSignalConversionAtMATLAB[0] = axes_B.X;
      axes_B.TmpSignalConversionAtMATLAB[1] = axes_B.Y;
      axes_B.TmpSignalConversionAtMATLAB[2] = axes_B.Z;
      axes_B.TmpSignalConversionAtMATLAB[3] = axes_B.W;

      /* MATLABSystem: '<S9>/MATLAB System' */
      obj = &axes_DW.obj.TreeInternal;
      axes_B.n = axes_DW.obj.TreeInternal.NumBodies;
      for (axes_B.ntilecols = 0; axes_B.ntilecols < 16; axes_B.ntilecols++) {
        axes_B.expl_temp.f1[axes_B.ntilecols] = tmp[axes_B.ntilecols];
      }

      /* End of Outputs for SubSystem: '<S1>/Deplacement' */
      axes_emxInit_d_cell_wrap(&Ttree, 2);

      /* Outputs for IfAction SubSystem: '<S1>/Deplacement' incorporates:
       *  ActionPort: '<S5>/Action'
       */
      T1_tmp = Ttree->size[0] * Ttree->size[1];

      /* MATLABSystem: '<S9>/MATLAB System' */
      Ttree->size[0] = 1;
      Ttree->size[1] = static_cast<int32_T>(axes_B.n);
      a_emxEnsureCapacity_d_cell_wrap(Ttree, T1_tmp);

      /* MATLABSystem: '<S9>/MATLAB System' */
      if (static_cast<int32_T>(axes_B.n) != 0) {
        axes_B.ntilecols = static_cast<int32_T>(axes_B.n) - 1;
        for (axes_B.b_jtilecol = 0; axes_B.b_jtilecol <= axes_B.ntilecols;
             axes_B.b_jtilecol++) {
          Ttree->data[axes_B.b_jtilecol] = axes_B.expl_temp;
        }
      }

      axes_B.k = 1.0;
      b_tmp = static_cast<int32_T>(axes_B.n) - 1;
      if (0 <= static_cast<int32_T>(axes_B.n) - 1) {
        for (axes_B.ntilecols = 0; axes_B.ntilecols < 5; axes_B.ntilecols++) {
          axes_B.b_d[axes_B.ntilecols] = tmp_0[axes_B.ntilecols];
        }
      }

      /* End of Outputs for SubSystem: '<S1>/Deplacement' */
      axes_emxInit_char_T(&switch_expression, 2);

      /* Outputs for IfAction SubSystem: '<S1>/Deplacement' incorporates:
       *  ActionPort: '<S5>/Action'
       */
      /* MATLABSystem: '<S9>/MATLAB System' */
      for (axes_B.b_jtilecol = 0; axes_B.b_jtilecol <= b_tmp; axes_B.b_jtilecol
           ++) {
        body = obj->Bodies[axes_B.b_jtilecol];
        axes_B.n = body->JointInternal.PositionNumber;
        axes_B.n += axes_B.k;
        if (axes_B.k > axes_B.n - 1.0) {
          axes_B.e = 0;
          axes_B.d = 0;
        } else {
          axes_B.e = static_cast<int32_T>(axes_B.k) - 1;
          axes_B.d = static_cast<int32_T>(axes_B.n - 1.0);
        }

        for (axes_B.ntilecols = 0; axes_B.ntilecols < 16; axes_B.ntilecols++) {
          axes_B.T1[axes_B.ntilecols] =
            body->JointInternal.JointToParentTransform[axes_B.ntilecols];
        }

        T1_tmp = switch_expression->size[0] * switch_expression->size[1];
        switch_expression->size[0] = 1;
        switch_expression->size[1] = body->JointInternal.Type->size[1];
        axes_emxEnsureCapacity_char_T(switch_expression, T1_tmp);
        loop_ub = body->JointInternal.Type->size[1] - 1;
        for (axes_B.ntilecols = 0; axes_B.ntilecols <= loop_ub; axes_B.ntilecols
             ++) {
          T1_tmp = axes_B.ntilecols;
          switch_expression->data[T1_tmp] = body->JointInternal.Type->
            data[T1_tmp];
        }

        b_varargout_1 = false;
        if (switch_expression->size[1] != 5) {
        } else {
          axes_B.ntilecols = 1;
          do {
            exitg1 = 0;
            if (axes_B.ntilecols - 1 < 5) {
              if (switch_expression->data[axes_B.ntilecols - 1] !=
                  axes_B.b_d[axes_B.ntilecols - 1]) {
                exitg1 = 1;
              } else {
                axes_B.ntilecols++;
              }
            } else {
              b_varargout_1 = true;
              exitg1 = 1;
            }
          } while (exitg1 == 0);
        }

        if (b_varargout_1) {
          axes_B.ntilecols = 0;
        } else {
          for (axes_B.ntilecols = 0; axes_B.ntilecols < 8; axes_B.ntilecols++) {
            axes_B.b_g[axes_B.ntilecols] = tmp_2[axes_B.ntilecols];
          }

          b_varargout_1 = false;
          if (switch_expression->size[1] != 8) {
          } else {
            axes_B.ntilecols = 1;
            do {
              exitg1 = 0;
              if (axes_B.ntilecols - 1 < 8) {
                if (switch_expression->data[axes_B.ntilecols - 1] !=
                    axes_B.b_g[axes_B.ntilecols - 1]) {
                  exitg1 = 1;
                } else {
                  axes_B.ntilecols++;
                }
              } else {
                b_varargout_1 = true;
                exitg1 = 1;
              }
            } while (exitg1 == 0);
          }

          if (b_varargout_1) {
            axes_B.ntilecols = 1;
          } else {
            axes_B.ntilecols = -1;
          }
        }

        switch (axes_B.ntilecols) {
         case 0:
          memset(&axes_B.T2[0], 0, sizeof(real_T) << 4U);
          axes_B.T2[0] = 1.0;
          axes_B.T2[5] = 1.0;
          axes_B.T2[10] = 1.0;
          axes_B.T2[15] = 1.0;
          break;

         case 1:
          ax_rigidBodyJoint_get_JointAxis(&body->JointInternal, axes_B.v);
          loop_ub = axes_B.d - axes_B.e;
          for (axes_B.ntilecols = 0; axes_B.ntilecols < loop_ub;
               axes_B.ntilecols++) {
            axes_B.e_data[axes_B.ntilecols] = axes_B.e + axes_B.ntilecols;
          }

          axes_B.ntilecols = 0;
          axes_B.result_data[axes_B.ntilecols] = axes_B.v[0];
          axes_B.e = 1;
          axes_B.result_data[axes_B.e] = axes_B.v[1];
          axes_B.d = 2;
          axes_B.result_data[axes_B.d] = axes_B.v[2];
          if (0 <= (loop_ub != 0) - 1) {
            axes_B.result_data[3] =
              axes_B.TmpSignalConversionAtMATLAB[axes_B.e_data[0]];
          }

          axes_B.theta = axes_B.result_data[axes_B.ntilecols];
          axes_B.k_tmp = axes_B.result_data[axes_B.e];
          axes_B.k_tmp_g = axes_B.result_data[axes_B.d];
          axes_B.k = 1.0 / sqrt((axes_B.theta * axes_B.theta + axes_B.k_tmp *
            axes_B.k_tmp) + axes_B.k_tmp_g * axes_B.k_tmp_g);
          axes_B.v[0] = axes_B.theta * axes_B.k;
          axes_B.v[1] = axes_B.k_tmp * axes_B.k;
          axes_B.v[2] = axes_B.k_tmp_g * axes_B.k;
          axes_B.theta = axes_B.result_data[3];
          axes_B.k = cos(axes_B.theta);
          axes_B.theta = sin(axes_B.theta);
          axes_B.tempR[0] = axes_B.v[0] * axes_B.v[0] * (1.0 - axes_B.k) +
            axes_B.k;
          axes_B.k_tmp = axes_B.v[0] * axes_B.v[1] * (1.0 - axes_B.k);
          axes_B.k_tmp_g = axes_B.v[2] * axes_B.theta;
          axes_B.tempR[1] = axes_B.k_tmp - axes_B.k_tmp_g;
          axes_B.tempR_tmp = axes_B.v[0] * axes_B.v[2] * (1.0 - axes_B.k);
          axes_B.tempR_tmp_d = axes_B.v[1] * axes_B.theta;
          axes_B.tempR[2] = axes_B.tempR_tmp + axes_B.tempR_tmp_d;
          axes_B.tempR[3] = axes_B.k_tmp + axes_B.k_tmp_g;
          axes_B.tempR[4] = axes_B.v[1] * axes_B.v[1] * (1.0 - axes_B.k) +
            axes_B.k;
          axes_B.k_tmp = axes_B.v[1] * axes_B.v[2] * (1.0 - axes_B.k);
          axes_B.k_tmp_g = axes_B.v[0] * axes_B.theta;
          axes_B.tempR[5] = axes_B.k_tmp - axes_B.k_tmp_g;
          axes_B.tempR[6] = axes_B.tempR_tmp - axes_B.tempR_tmp_d;
          axes_B.tempR[7] = axes_B.k_tmp + axes_B.k_tmp_g;
          axes_B.tempR[8] = axes_B.v[2] * axes_B.v[2] * (1.0 - axes_B.k) +
            axes_B.k;
          for (axes_B.ntilecols = 0; axes_B.ntilecols < 3; axes_B.ntilecols++) {
            axes_B.R[axes_B.ntilecols] = axes_B.tempR[axes_B.ntilecols * 3];
            axes_B.R[axes_B.ntilecols + 3] = axes_B.tempR[axes_B.ntilecols * 3 +
              1];
            axes_B.R[axes_B.ntilecols + 6] = axes_B.tempR[axes_B.ntilecols * 3 +
              2];
          }

          memset(&axes_B.T2[0], 0, sizeof(real_T) << 4U);
          for (axes_B.ntilecols = 0; axes_B.ntilecols < 3; axes_B.ntilecols++) {
            axes_B.e = axes_B.ntilecols << 2;
            axes_B.T2[axes_B.e] = axes_B.R[3 * axes_B.ntilecols];
            axes_B.T2[axes_B.e + 1] = axes_B.R[3 * axes_B.ntilecols + 1];
            axes_B.T2[axes_B.e + 2] = axes_B.R[3 * axes_B.ntilecols + 2];
          }

          axes_B.T2[15] = 1.0;
          break;

         default:
          ax_rigidBodyJoint_get_JointAxis(&body->JointInternal, axes_B.v);
          memset(&axes_B.tempR[0], 0, 9U * sizeof(real_T));
          axes_B.tempR[0] = 1.0;
          axes_B.tempR[4] = 1.0;
          axes_B.tempR[8] = 1.0;
          axes_B.k = axes_B.TmpSignalConversionAtMATLAB[axes_B.e];
          for (axes_B.ntilecols = 0; axes_B.ntilecols < 3; axes_B.ntilecols++) {
            axes_B.e = axes_B.ntilecols << 2;
            axes_B.T2[axes_B.e] = axes_B.tempR[3 * axes_B.ntilecols];
            axes_B.T2[axes_B.e + 1] = axes_B.tempR[3 * axes_B.ntilecols + 1];
            axes_B.T2[axes_B.e + 2] = axes_B.tempR[3 * axes_B.ntilecols + 2];
            axes_B.T2[axes_B.ntilecols + 12] = axes_B.v[axes_B.ntilecols] *
              axes_B.k;
          }

          axes_B.T2[3] = 0.0;
          axes_B.T2[7] = 0.0;
          axes_B.T2[11] = 0.0;
          axes_B.T2[15] = 1.0;
          break;
        }

        for (axes_B.ntilecols = 0; axes_B.ntilecols < 16; axes_B.ntilecols++) {
          axes_B.b[axes_B.ntilecols] = body->
            JointInternal.ChildToJointTransform[axes_B.ntilecols];
        }

        for (axes_B.ntilecols = 0; axes_B.ntilecols < 4; axes_B.ntilecols++) {
          for (axes_B.e = 0; axes_B.e < 4; axes_B.e++) {
            axes_B.d = axes_B.e << 2;
            loop_ub = axes_B.ntilecols + axes_B.d;
            axes_B.T1_m[loop_ub] = 0.0;
            axes_B.T1_m[loop_ub] += axes_B.T2[axes_B.d] *
              axes_B.T1[axes_B.ntilecols];
            axes_B.T1_m[loop_ub] += axes_B.T2[axes_B.d + 1] *
              axes_B.T1[axes_B.ntilecols + 4];
            axes_B.T1_m[loop_ub] += axes_B.T2[axes_B.d + 2] *
              axes_B.T1[axes_B.ntilecols + 8];
            axes_B.T1_m[loop_ub] += axes_B.T2[axes_B.d + 3] *
              axes_B.T1[axes_B.ntilecols + 12];
          }

          for (axes_B.e = 0; axes_B.e < 4; axes_B.e++) {
            axes_B.d = axes_B.e << 2;
            loop_ub = axes_B.ntilecols + axes_B.d;
            Ttree->data[axes_B.b_jtilecol].f1[loop_ub] = 0.0;
            T1_tmp = axes_B.b_jtilecol;
            Ttree->data[axes_B.b_jtilecol].f1[loop_ub] = axes_B.b[axes_B.d] *
              axes_B.T1_m[axes_B.ntilecols] + Ttree->data[T1_tmp].f1[loop_ub];
            Ttree->data[axes_B.b_jtilecol].f1[loop_ub] = axes_B.b[axes_B.d + 1] *
              axes_B.T1_m[axes_B.ntilecols + 4] + Ttree->data[T1_tmp].f1[loop_ub];
            Ttree->data[axes_B.b_jtilecol].f1[loop_ub] = axes_B.b[axes_B.d + 2] *
              axes_B.T1_m[axes_B.ntilecols + 8] + Ttree->data[T1_tmp].f1[loop_ub];
            Ttree->data[axes_B.b_jtilecol].f1[loop_ub] = axes_B.b[axes_B.d + 3] *
              axes_B.T1_m[axes_B.ntilecols + 12] + Ttree->data[T1_tmp]
              .f1[loop_ub];
          }
        }

        axes_B.k = axes_B.n;
        if (body->ParentIndex > 0.0) {
          for (axes_B.ntilecols = 0; axes_B.ntilecols < 16; axes_B.ntilecols++)
          {
            axes_B.T1[axes_B.ntilecols] = Ttree->data[static_cast<int32_T>
              (body->ParentIndex) - 1].f1[axes_B.ntilecols];
          }

          for (axes_B.ntilecols = 0; axes_B.ntilecols < 4; axes_B.ntilecols++) {
            for (axes_B.e = 0; axes_B.e < 4; axes_B.e++) {
              axes_B.d = axes_B.e << 2;
              loop_ub = axes_B.ntilecols + axes_B.d;
              axes_B.T1_m[loop_ub] = 0.0;
              T1_tmp = axes_B.b_jtilecol;
              axes_B.T1_m[loop_ub] += Ttree->data[T1_tmp].f1[axes_B.d] *
                axes_B.T1[axes_B.ntilecols];
              axes_B.T1_m[loop_ub] += Ttree->data[T1_tmp].f1[axes_B.d + 1] *
                axes_B.T1[axes_B.ntilecols + 4];
              axes_B.T1_m[loop_ub] += Ttree->data[T1_tmp].f1[axes_B.d + 2] *
                axes_B.T1[axes_B.ntilecols + 8];
              axes_B.T1_m[loop_ub] += Ttree->data[T1_tmp].f1[axes_B.d + 3] *
                axes_B.T1[axes_B.ntilecols + 12];
            }
          }

          for (axes_B.ntilecols = 0; axes_B.ntilecols < 16; axes_B.ntilecols++)
          {
            Ttree->data[axes_B.b_jtilecol].f1[axes_B.ntilecols] =
              axes_B.T1_m[axes_B.ntilecols];
          }
        }
      }

      axes_B.n = -1.0;
      T1_tmp = switch_expression->size[0] * switch_expression->size[1];

      /* MATLABSystem: '<S9>/MATLAB System' */
      switch_expression->size[0] = 1;
      switch_expression->size[1] = obj->Base.NameInternal->size[1];
      axes_emxEnsureCapacity_char_T(switch_expression, T1_tmp);

      /* MATLABSystem: '<S9>/MATLAB System' */
      loop_ub = obj->Base.NameInternal->size[1] - 1;
      for (axes_B.ntilecols = 0; axes_B.ntilecols <= loop_ub; axes_B.ntilecols++)
      {
        T1_tmp = axes_B.ntilecols;
        switch_expression->data[T1_tmp] = obj->Base.NameInternal->data[T1_tmp];
      }

      for (axes_B.ntilecols = 0; axes_B.ntilecols < 5; axes_B.ntilecols++) {
        axes_B.b_d[axes_B.ntilecols] = tmp_1[axes_B.ntilecols];
      }

      b_varargout_1 = false;
      if (switch_expression->size[1] != 5) {
      } else {
        axes_B.ntilecols = 1;
        do {
          exitg1 = 0;
          if (axes_B.ntilecols - 1 < 5) {
            if (switch_expression->data[axes_B.ntilecols - 1] !=
                axes_B.b_d[axes_B.ntilecols - 1]) {
              exitg1 = 1;
            } else {
              axes_B.ntilecols++;
            }
          } else {
            b_varargout_1 = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }

      if (b_varargout_1) {
        axes_B.n = 0.0;
      } else {
        axes_B.k = axes_DW.obj.TreeInternal.NumBodies;
        axes_B.b_jtilecol = 0;
        exitg2 = false;
        while ((!exitg2) && (axes_B.b_jtilecol <= static_cast<int32_T>(axes_B.k)
                             - 1)) {
          body = obj->Bodies[axes_B.b_jtilecol];
          T1_tmp = switch_expression->size[0] * switch_expression->size[1];
          switch_expression->size[0] = 1;
          switch_expression->size[1] = body->NameInternal->size[1];
          axes_emxEnsureCapacity_char_T(switch_expression, T1_tmp);
          loop_ub = body->NameInternal->size[1] - 1;
          for (axes_B.ntilecols = 0; axes_B.ntilecols <= loop_ub;
               axes_B.ntilecols++) {
            T1_tmp = axes_B.ntilecols;
            switch_expression->data[T1_tmp] = body->NameInternal->data[T1_tmp];
          }

          for (axes_B.ntilecols = 0; axes_B.ntilecols < 5; axes_B.ntilecols++) {
            axes_B.b_d[axes_B.ntilecols] = tmp_1[axes_B.ntilecols];
          }

          b_varargout_1 = false;
          if (switch_expression->size[1] != 5) {
          } else {
            axes_B.ntilecols = 1;
            do {
              exitg1 = 0;
              if (axes_B.ntilecols - 1 < 5) {
                if (switch_expression->data[axes_B.ntilecols - 1] !=
                    axes_B.b_d[axes_B.ntilecols - 1]) {
                  exitg1 = 1;
                } else {
                  axes_B.ntilecols++;
                }
              } else {
                b_varargout_1 = true;
                exitg1 = 1;
              }
            } while (exitg1 == 0);
          }

          if (b_varargout_1) {
            axes_B.n = static_cast<real_T>(axes_B.b_jtilecol) + 1.0;
            exitg2 = true;
          } else {
            axes_B.b_jtilecol++;
          }
        }
      }

      if (axes_B.n == 0.0) {
        memset(&axes_B.T1[0], 0, sizeof(real_T) << 4U);
        axes_B.T1[0] = 1.0;
        axes_B.T1[5] = 1.0;
        axes_B.T1[10] = 1.0;
        axes_B.T1[15] = 1.0;
      } else {
        for (axes_B.ntilecols = 0; axes_B.ntilecols < 16; axes_B.ntilecols++) {
          axes_B.T1[axes_B.ntilecols] = Ttree->data[static_cast<int32_T>
            (axes_B.n) - 1].f1[axes_B.ntilecols];
        }
      }

      axes_B.n = -1.0;
      T1_tmp = switch_expression->size[0] * switch_expression->size[1];

      /* MATLABSystem: '<S9>/MATLAB System' */
      switch_expression->size[0] = 1;
      switch_expression->size[1] = obj->Base.NameInternal->size[1];
      axes_emxEnsureCapacity_char_T(switch_expression, T1_tmp);

      /* MATLABSystem: '<S9>/MATLAB System' */
      loop_ub = obj->Base.NameInternal->size[1] - 1;
      for (axes_B.ntilecols = 0; axes_B.ntilecols <= loop_ub; axes_B.ntilecols++)
      {
        T1_tmp = axes_B.ntilecols;
        switch_expression->data[T1_tmp] = obj->Base.NameInternal->data[T1_tmp];
      }

      axes_B.b_lx[0] = 'B';
      axes_B.b_lx[1] = 'a';
      axes_B.b_lx[2] = 's';
      axes_B.b_lx[3] = 'e';
      b_varargout_1 = false;
      if (switch_expression->size[1] != 4) {
      } else {
        axes_B.ntilecols = 1;
        do {
          exitg1 = 0;
          if (axes_B.ntilecols - 1 < 4) {
            if (switch_expression->data[axes_B.ntilecols - 1] !=
                axes_B.b_lx[axes_B.ntilecols - 1]) {
              exitg1 = 1;
            } else {
              axes_B.ntilecols++;
            }
          } else {
            b_varargout_1 = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }

      if (b_varargout_1) {
        axes_B.n = 0.0;
      } else {
        axes_B.k = axes_DW.obj.TreeInternal.NumBodies;
        axes_B.b_jtilecol = 0;
        exitg2 = false;
        while ((!exitg2) && (axes_B.b_jtilecol <= static_cast<int32_T>(axes_B.k)
                             - 1)) {
          body = obj->Bodies[axes_B.b_jtilecol];
          T1_tmp = switch_expression->size[0] * switch_expression->size[1];
          switch_expression->size[0] = 1;
          switch_expression->size[1] = body->NameInternal->size[1];
          axes_emxEnsureCapacity_char_T(switch_expression, T1_tmp);
          loop_ub = body->NameInternal->size[1] - 1;
          for (axes_B.ntilecols = 0; axes_B.ntilecols <= loop_ub;
               axes_B.ntilecols++) {
            T1_tmp = axes_B.ntilecols;
            switch_expression->data[T1_tmp] = body->NameInternal->data[T1_tmp];
          }

          axes_B.b_lx[0] = 'B';
          axes_B.b_lx[1] = 'a';
          axes_B.b_lx[2] = 's';
          axes_B.b_lx[3] = 'e';
          b_varargout_1 = false;
          if (switch_expression->size[1] != 4) {
          } else {
            axes_B.ntilecols = 1;
            do {
              exitg1 = 0;
              if (axes_B.ntilecols - 1 < 4) {
                if (switch_expression->data[axes_B.ntilecols - 1] !=
                    axes_B.b_lx[axes_B.ntilecols - 1]) {
                  exitg1 = 1;
                } else {
                  axes_B.ntilecols++;
                }
              } else {
                b_varargout_1 = true;
                exitg1 = 1;
              }
            } while (exitg1 == 0);
          }

          if (b_varargout_1) {
            axes_B.n = static_cast<real_T>(axes_B.b_jtilecol) + 1.0;
            exitg2 = true;
          } else {
            axes_B.b_jtilecol++;
          }
        }
      }

      /* End of Outputs for SubSystem: '<S1>/Deplacement' */
      axes_emxFree_char_T(&switch_expression);

      /* Outputs for IfAction SubSystem: '<S1>/Deplacement' incorporates:
       *  ActionPort: '<S5>/Action'
       */
      /* MATLABSystem: '<S9>/MATLAB System' */
      if (axes_B.n == 0.0) {
        memset(&axes_B.T2[0], 0, sizeof(real_T) << 4U);
        axes_B.T2[0] = 1.0;
        axes_B.T2[5] = 1.0;
        axes_B.T2[10] = 1.0;
        axes_B.T2[15] = 1.0;
      } else {
        for (axes_B.ntilecols = 0; axes_B.ntilecols < 16; axes_B.ntilecols++) {
          axes_B.T2[axes_B.ntilecols] = Ttree->data[static_cast<int32_T>
            (axes_B.n) - 1].f1[axes_B.ntilecols];
        }
      }

      /* End of Outputs for SubSystem: '<S1>/Deplacement' */
      axes_emxFree_d_cell_wrap(&Ttree);

      /* Outputs for IfAction SubSystem: '<S1>/Deplacement' incorporates:
       *  ActionPort: '<S5>/Action'
       */
      /* MATLABSystem: '<S9>/MATLAB System' */
      for (axes_B.ntilecols = 0; axes_B.ntilecols < 3; axes_B.ntilecols++) {
        axes_B.R[3 * axes_B.ntilecols] = axes_B.T2[axes_B.ntilecols];
        axes_B.R[3 * axes_B.ntilecols + 1] = axes_B.T2[axes_B.ntilecols + 4];
        axes_B.R[3 * axes_B.ntilecols + 2] = axes_B.T2[axes_B.ntilecols + 8];
      }

      /* Outputs for Atomic SubSystem: '<S5>/Subscribe' */
      /* MATLABSystem: '<S10>/SourceBlock' */
      b_varargout_1 = Sub_axes_7.getLatestMessage(&axes_B.b_varargout_2);

      /* Outputs for Enabled SubSystem: '<S10>/Enabled Subsystem' incorporates:
       *  EnablePort: '<S12>/Enable'
       */
      if (b_varargout_1) {
        axes_B.In1 = axes_B.b_varargout_2;
      }

      /* End of Outputs for SubSystem: '<S10>/Enabled Subsystem' */
      /* End of Outputs for SubSystem: '<S5>/Subscribe' */

      /* MATLABSystem: '<S9>/MATLAB System' incorporates:
       *  Inport: '<S12>/In1'
       *  MATLABSystem: '<S10>/SourceBlock'
       */
      for (axes_B.ntilecols = 0; axes_B.ntilecols < 9; axes_B.ntilecols++) {
        axes_B.tempR[axes_B.ntilecols] = -axes_B.R[axes_B.ntilecols];
      }

      for (axes_B.ntilecols = 0; axes_B.ntilecols < 3; axes_B.ntilecols++) {
        axes_B.b_jtilecol = axes_B.ntilecols << 2;
        axes_B.b[axes_B.b_jtilecol] = axes_B.R[3 * axes_B.ntilecols];
        axes_B.b[axes_B.b_jtilecol + 1] = axes_B.R[3 * axes_B.ntilecols + 1];
        axes_B.b[axes_B.b_jtilecol + 2] = axes_B.R[3 * axes_B.ntilecols + 2];
        axes_B.b[axes_B.ntilecols + 12] = (axes_B.tempR[axes_B.ntilecols + 3] *
          axes_B.T2[13] + axes_B.tempR[axes_B.ntilecols] * axes_B.T2[12]) +
          axes_B.tempR[axes_B.ntilecols + 6] * axes_B.T2[14];
      }

      axes_B.b[3] = 0.0;
      axes_B.b[7] = 0.0;
      axes_B.b[11] = 0.0;
      axes_B.b[15] = 1.0;

      /* SignalConversion generated from: '<S5>/Product' incorporates:
       *  Constant: '<S5>/Constant'
       *  Constant: '<S5>/Constant1'
       *  Constant: '<S5>/Constant2'
       *  Constant: '<S5>/Constant3'
       *  Gain: '<S5>/Gain'
       *  Gain: '<S5>/Gain1'
       *  Gain: '<S5>/Gain2'
       *  Sum: '<S5>/Add'
       *  Sum: '<S5>/Add1'
       *  Sum: '<S5>/Add2'
       */
      axes_B.TmpSignalConversionAtMATLAB[0] = axes_P.Gain1_Gain *
        axes_B.In1.Axes[3] + axes_P.Constant2_Value;
      axes_B.TmpSignalConversionAtMATLAB[1] = axes_P.Gain2_Gain *
        axes_B.In1.Axes[4] + axes_P.Constant1_Value;
      axes_B.TmpSignalConversionAtMATLAB[2] = axes_P.Gain_Gain *
        axes_B.In1.Axes[1] + axes_P.Constant_Value_j;
      axes_B.TmpSignalConversionAtMATLAB[3] = axes_P.Constant3_Value;
      for (axes_B.ntilecols = 0; axes_B.ntilecols < 4; axes_B.ntilecols++) {
        /* Product: '<S5>/Product' */
        axes_B.result_data[axes_B.ntilecols] = 0.0;
        for (axes_B.e = 0; axes_B.e < 4; axes_B.e++) {
          /* MATLABSystem: '<S9>/MATLAB System' incorporates:
           *  Product: '<S5>/Product'
           */
          b_tmp = axes_B.e << 2;
          axes_B.b_jtilecol = axes_B.ntilecols + b_tmp;
          axes_B.T2[axes_B.b_jtilecol] = 0.0;
          axes_B.T2[axes_B.b_jtilecol] += axes_B.T1[b_tmp] *
            axes_B.b[axes_B.ntilecols];
          axes_B.T2[axes_B.b_jtilecol] += axes_B.T1[b_tmp + 1] *
            axes_B.b[axes_B.ntilecols + 4];
          axes_B.T2[axes_B.b_jtilecol] += axes_B.T1[b_tmp + 2] *
            axes_B.b[axes_B.ntilecols + 8];
          axes_B.T2[axes_B.b_jtilecol] += axes_B.T1[b_tmp + 3] *
            axes_B.b[axes_B.ntilecols + 12];
          axes_B.result_data[axes_B.ntilecols] += axes_B.T2[axes_B.b_jtilecol] *
            axes_B.TmpSignalConversionAtMATLAB[axes_B.e];
        }
      }

      /* Gain: '<S5>/Gain4' incorporates:
       *  Gain: '<S5>/Gain3'
       *  MATLABSystem: '<S9>/MATLAB System'
       *  Product: '<S5>/Product'
       */
      axes_B.v[0] = axes_P.Gain3_Gain * axes_B.result_data[0] *
        axes_P.Gain4_Gain;
      axes_B.v[1] = axes_P.Gain3_Gain * axes_B.result_data[1] *
        axes_P.Gain4_Gain;
      axes_B.v[2] = axes_P.Gain3_Gain * axes_B.result_data[2] *
        axes_P.Gain4_Gain;

      /* End of Outputs for SubSystem: '<S1>/Deplacement' */

      /* Outputs for IfAction SubSystem: '<S1>/3 4 5' incorporates:
       *  ActionPort: '<S4>/Action'
       */
    } else if (axes_B.In1_h.Data == 3) {
      /* Outputs for IfAction SubSystem: '<S4>/poser' incorporates:
       *  ActionPort: '<S7>/Action'
       */
      /* If: '<S4>/If' */
      axes_poser(axes_B.v, &axes_P.poser);

      /* End of Outputs for SubSystem: '<S4>/poser' */
    } else {
      /* Outputs for IfAction SubSystem: '<S4>/zero' incorporates:
       *  ActionPort: '<S8>/Action'
       */
      /* If: '<S4>/If' */
      axes_poser(axes_B.v, &axes_P.zero);

      /* End of Outputs for SubSystem: '<S4>/zero' */

      /* End of Outputs for SubSystem: '<S1>/3 4 5' */
    }

    /* BusAssignment: '<S6>/Bus Assignment' */
    axes_B.BusAssignment.X = axes_B.v[0];
    axes_B.BusAssignment.Y = axes_B.v[1];
    axes_B.BusAssignment.Z = axes_B.v[2];

    /* Outputs for Atomic SubSystem: '<S6>/Publish' */
    /* MATLABSystem: '<S17>/SinkBlock' */
    Pub_axes_120.publish(&axes_B.BusAssignment);

    /* End of Outputs for SubSystem: '<S6>/Publish' */
    /* End of Outputs for SubSystem: '<Root>/1 2' */
  }

  /* End of If: '<Root>/If' */
}

/* Model initialize function */
void axes_initialize(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* block I/O */
  (void) memset((static_cast<void *>(&axes_B)), 0,
                sizeof(B_axes_T));

  /* states (dwork) */
  (void) memset(static_cast<void *>(&axes_DW), 0,
                sizeof(DW_axes_T));

  {
    static const char_T tmp[5] = { '/', 'e', 't', 'a', 't' };

    static const char_T tmp_0[13] = { '/', 'a', 'n', 'g', 'l', 'e', 's', '_',
      'j', 'o', 'i', 'n', 't' };

    static const char_T tmp_1[15] = { '/', 'p', 'o', 's', 'i', 't', 'i', 'o',
      'n', '_', 'r', 'o', 'b', 'o', 't' };

    /* Start for Atomic SubSystem: '<Root>/Subscribe1' */
    /* Start for MATLABSystem: '<S3>/SourceBlock' */
    axes_DW.obj_g.matlabCodegenIsDeleted = true;
    axes_DW.obj_g.isInitialized = 0;
    axes_DW.obj_g.matlabCodegenIsDeleted = false;
    axes_DW.objisempty = true;
    axes_DW.obj_g.isSetupComplete = false;
    axes_DW.obj_g.isInitialized = 1;
    for (axes_B.b_o = 0; axes_B.b_o < 5; axes_B.b_o++) {
      axes_B.b_zeroDelimTopic_l[axes_B.b_o] = tmp[axes_B.b_o];
    }

    axes_B.b_zeroDelimTopic_l[5] = '\x00';
    Sub_axes_64.createSubscriber(&axes_B.b_zeroDelimTopic_l[0], 1);
    axes_DW.obj_g.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<S3>/SourceBlock' */
    /* End of Start for SubSystem: '<Root>/Subscribe1' */

    /* Start for If: '<Root>/If' */
    axes_DW.If_ActiveSubsystem = -1;

    /* Start for IfAction SubSystem: '<Root>/1 2' */
    /* Start for If: '<S1>/If' */
    axes_DW.If_ActiveSubsystem_d = -1;

    /* Start for IfAction SubSystem: '<S1>/Deplacement' */
    /* Start for Atomic SubSystem: '<S11>/Subscribe' */
    /* Start for MATLABSystem: '<S13>/SourceBlock' */
    axes_DW.obj_k.matlabCodegenIsDeleted = true;
    axes_DW.obj_k.isInitialized = 0;
    axes_DW.obj_k.matlabCodegenIsDeleted = false;
    axes_DW.objisempty_b = true;
    axes_DW.obj_k.isSetupComplete = false;
    axes_DW.obj_k.isInitialized = 1;
    for (axes_B.b_o = 0; axes_B.b_o < 13; axes_B.b_o++) {
      axes_B.b_zeroDelimTopic_k[axes_B.b_o] = tmp_0[axes_B.b_o];
    }

    axes_B.b_zeroDelimTopic_k[13] = '\x00';
    Sub_axes_47.createSubscriber(&axes_B.b_zeroDelimTopic_k[0], 1);
    axes_DW.obj_k.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<S13>/SourceBlock' */
    /* End of Start for SubSystem: '<S11>/Subscribe' */
    /* End of Start for SubSystem: '<S1>/Deplacement' */
    /* End of Start for SubSystem: '<Root>/1 2' */
    emxInitStruct_robotics_slmanip_(&axes_DW.obj);

    /* Start for IfAction SubSystem: '<Root>/1 2' */
    /* Start for IfAction SubSystem: '<S1>/Deplacement' */
    /* Start for MATLABSystem: '<S9>/MATLAB System' */
    for (axes_B.b_o = 0; axes_B.b_o < 6; axes_B.b_o++) {
      axes_DW.obj.TreeInternal._pobj0[axes_B.b_o].
        CollisionsInternal._pobj0.matlabCodegenIsDeleted = true;
    }

    axes_DW.obj.TreeInternal.Base.CollisionsInternal._pobj0.matlabCodegenIsDeleted
      = true;
    axes_DW.obj.isInitialized = 0;
    axes_DW.objisempty_f = true;
    axes_SystemCore_setup(&axes_DW.obj);

    /* End of Start for MATLABSystem: '<S9>/MATLAB System' */

    /* Start for Atomic SubSystem: '<S5>/Subscribe' */
    /* Start for MATLABSystem: '<S10>/SourceBlock' */
    axes_DW.obj_n.matlabCodegenIsDeleted = true;
    axes_DW.obj_n.isInitialized = 0;
    axes_DW.obj_n.matlabCodegenIsDeleted = false;
    axes_DW.objisempty_l = true;
    axes_DW.obj_n.isSetupComplete = false;
    axes_DW.obj_n.isInitialized = 1;
    axes_B.b_zeroDelimTopic_d[0] = '/';
    axes_B.b_zeroDelimTopic_d[1] = 'j';
    axes_B.b_zeroDelimTopic_d[2] = 'o';
    axes_B.b_zeroDelimTopic_d[3] = 'y';
    axes_B.b_zeroDelimTopic_d[4] = '\x00';
    Sub_axes_7.createSubscriber(&axes_B.b_zeroDelimTopic_d[0], 1);
    axes_DW.obj_n.isSetupComplete = true;

    /* End of Start for SubSystem: '<S5>/Subscribe' */
    /* End of Start for SubSystem: '<S1>/Deplacement' */

    /* Start for Atomic SubSystem: '<S6>/Publish' */
    /* Start for MATLABSystem: '<S17>/SinkBlock' */
    axes_DW.obj_p.matlabCodegenIsDeleted = true;
    axes_DW.obj_p.isInitialized = 0;
    axes_DW.obj_p.matlabCodegenIsDeleted = false;
    axes_DW.objisempty_i = true;
    axes_DW.obj_p.isSetupComplete = false;
    axes_DW.obj_p.isInitialized = 1;
    for (axes_B.b_o = 0; axes_B.b_o < 15; axes_B.b_o++) {
      axes_B.b_zeroDelimTopic[axes_B.b_o] = tmp_1[axes_B.b_o];
    }

    axes_B.b_zeroDelimTopic[15] = '\x00';
    Pub_axes_120.createPublisher(&axes_B.b_zeroDelimTopic[0], 1);
    axes_DW.obj_p.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<S17>/SinkBlock' */
    /* End of Start for SubSystem: '<S6>/Publish' */
    /* End of Start for SubSystem: '<Root>/1 2' */
  }

  /* SystemInitialize for Atomic SubSystem: '<Root>/Subscribe1' */
  /* SystemInitialize for Enabled SubSystem: '<S3>/Enabled Subsystem' */
  /* SystemInitialize for Outport: '<S18>/Out1' incorporates:
   *  Inport: '<S18>/In1'
   */
  axes_B.In1_h = axes_P.Out1_Y0_g;

  /* End of SystemInitialize for SubSystem: '<S3>/Enabled Subsystem' */
  /* End of SystemInitialize for SubSystem: '<Root>/Subscribe1' */

  /* SystemInitialize for IfAction SubSystem: '<Root>/1 2' */
  /* SystemInitialize for IfAction SubSystem: '<S1>/Deplacement' */
  /* SystemInitialize for Enabled SubSystem: '<S11>/Subsystem' */
  /* SystemInitialize for SignalConversion generated from: '<S14>/Bus Selector' incorporates:
   *  Outport: '<S14>/Out1'
   */
  axes_B.X = axes_P.Out1_Y0_l;

  /* SystemInitialize for SignalConversion generated from: '<S14>/Bus Selector' incorporates:
   *  Outport: '<S14>/Out1'
   */
  axes_B.Y = axes_P.Out1_Y0_l;

  /* SystemInitialize for SignalConversion generated from: '<S14>/Bus Selector' incorporates:
   *  Outport: '<S14>/Out1'
   */
  axes_B.Z = axes_P.Out1_Y0_l;

  /* SystemInitialize for SignalConversion generated from: '<S14>/Bus Selector' incorporates:
   *  Outport: '<S14>/Out1'
   */
  axes_B.W = axes_P.Out1_Y0_l;

  /* End of SystemInitialize for SubSystem: '<S11>/Subsystem' */

  /* SystemInitialize for Atomic SubSystem: '<S5>/Subscribe' */
  /* SystemInitialize for Enabled SubSystem: '<S10>/Enabled Subsystem' */
  /* SystemInitialize for Outport: '<S12>/Out1' incorporates:
   *  Inport: '<S12>/In1'
   */
  axes_B.In1 = axes_P.Out1_Y0;

  /* End of SystemInitialize for SubSystem: '<S10>/Enabled Subsystem' */
  /* End of SystemInitialize for SubSystem: '<S5>/Subscribe' */
  /* End of SystemInitialize for SubSystem: '<S1>/Deplacement' */
  /* End of SystemInitialize for SubSystem: '<Root>/1 2' */
}

/* Model terminate function */
void axes_terminate(void)
{
  void* geometryInternal;
  i_robotics_manip_internal_Col_T *obj;

  /* Terminate for Atomic SubSystem: '<Root>/Subscribe1' */
  /* Terminate for MATLABSystem: '<S3>/SourceBlock' */
  if (!axes_DW.obj_g.matlabCodegenIsDeleted) {
    axes_DW.obj_g.matlabCodegenIsDeleted = true;
  }

  /* End of Terminate for MATLABSystem: '<S3>/SourceBlock' */
  /* End of Terminate for SubSystem: '<Root>/Subscribe1' */

  /* Terminate for IfAction SubSystem: '<Root>/1 2' */
  /* Terminate for IfAction SubSystem: '<S1>/Deplacement' */
  /* Terminate for Atomic SubSystem: '<S11>/Subscribe' */
  /* Terminate for MATLABSystem: '<S13>/SourceBlock' */
  if (!axes_DW.obj_k.matlabCodegenIsDeleted) {
    axes_DW.obj_k.matlabCodegenIsDeleted = true;
  }

  /* End of Terminate for MATLABSystem: '<S13>/SourceBlock' */
  /* End of Terminate for SubSystem: '<S11>/Subscribe' */

  /* Terminate for MATLABSystem: '<S9>/MATLAB System' */
  obj = &axes_DW.obj.TreeInternal.Base.CollisionsInternal._pobj0;
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
    geometryInternal = obj->CollisionPrimitive;
    collisioncodegen_destructGeometry(&geometryInternal);
  }

  for (int32_T b = 0; b < 6; b++) {
    obj = &axes_DW.obj.TreeInternal._pobj0[b].CollisionsInternal._pobj0;
    if (!obj->matlabCodegenIsDeleted) {
      obj->matlabCodegenIsDeleted = true;
      geometryInternal = obj->CollisionPrimitive;
      collisioncodegen_destructGeometry(&geometryInternal);
    }
  }

  /* End of Terminate for MATLABSystem: '<S9>/MATLAB System' */
  /* End of Terminate for SubSystem: '<S1>/Deplacement' */
  /* End of Terminate for SubSystem: '<Root>/1 2' */
  emxFreeStruct_robotics_slmanip_(&axes_DW.obj);

  /* Terminate for IfAction SubSystem: '<Root>/1 2' */
  /* Terminate for IfAction SubSystem: '<S1>/Deplacement' */
  /* Terminate for Atomic SubSystem: '<S5>/Subscribe' */
  /* Terminate for MATLABSystem: '<S10>/SourceBlock' */
  if (!axes_DW.obj_n.matlabCodegenIsDeleted) {
    axes_DW.obj_n.matlabCodegenIsDeleted = true;
  }

  /* End of Terminate for MATLABSystem: '<S10>/SourceBlock' */
  /* End of Terminate for SubSystem: '<S5>/Subscribe' */
  /* End of Terminate for SubSystem: '<S1>/Deplacement' */

  /* Terminate for Atomic SubSystem: '<S6>/Publish' */
  /* Terminate for MATLABSystem: '<S17>/SinkBlock' */
  if (!axes_DW.obj_p.matlabCodegenIsDeleted) {
    axes_DW.obj_p.matlabCodegenIsDeleted = true;
  }

  /* End of Terminate for MATLABSystem: '<S17>/SinkBlock' */
  /* End of Terminate for SubSystem: '<S6>/Publish' */
  /* End of Terminate for SubSystem: '<Root>/1 2' */
}
