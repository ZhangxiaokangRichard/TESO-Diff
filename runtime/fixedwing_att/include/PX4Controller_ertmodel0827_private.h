//
// File: PX4Controller_ertmodel0827_private.h
//
// Code generated for Simulink model 'PX4Controller_ertmodel0827'.
//
// Model version                  : 1.229
// Simulink Coder version         : 9.5 (R2021a) 14-Nov-2020
// C/C++ source code generated on : Tue Nov  3 10:49:30 2026
//
// Target selection: ert.tlc
// Embedded hardware selection: Intel->x86-64 (Windows64)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#ifndef RTW_HEADER_PX4Controller_ertmodel0827_private_h_
#define RTW_HEADER_PX4Controller_ertmodel0827_private_h_
#include "rtwtypes.h"

// Private macros used by the generated code to access rtModel
#ifndef rtmIsMajorTimeStep
#define rtmIsMajorTimeStep(rtm)        (((rtm)->Timing.simTimeStep) == MAJOR_TIME_STEP)
#endif

#ifndef rtmIsMinorTimeStep
#define rtmIsMinorTimeStep(rtm)        (((rtm)->Timing.simTimeStep) == MINOR_TIME_STEP)
#endif

#ifndef rtmSetTPtr
#define rtmSetTPtr(rtm, val)           ((rtm)->Timing.t = (val))
#endif

extern void rt_invd3x3_snf(const real_T u[9], real_T y[9]);
extern real_T rt_powd_snf(real_T u0, real_T u1);
extern void rt_mrdivide_U1d1x3_U2d_9vOrDY9Z(const real_T u0[3], const real_T u1
  [9], real_T y[3]);
extern real_T look1_binlxpw(real_T u0, const real_T bp0[], const real_T table[],
  uint32_T maxIndex);

// private model entry point functions
extern void PX4Controller_ertmodel0827_derivatives();

#endif                      // RTW_HEADER_PX4Controller_ertmodel0827_private_h_

//
// File trailer for generated code.
//
// [EOF]
//
