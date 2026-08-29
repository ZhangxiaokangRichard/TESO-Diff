//
// File: AVANTI2_ertmodel0827_data.cpp
//
// Code generated for Simulink model 'AVANTI2_ertmodel0827'.
//
// Model version                  : 1.134
// Simulink Coder version         : 9.5 (R2021a) 14-Nov-2020
// C/C++ source code generated on : Thu Aug 27 16:32:39 2026
//
// Target selection: ert.tlc
// Embedded hardware selection: Intel->x86-64 (Windows64)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#include "AVANTI2_ertmodel0827.h"
#include "AVANTI2_ertmodel0827_private.h"

// Invariant block signals (default storage)
const AVANTI2_ertmodel0827ModelClass::ConstB_AVANTI2_ertmodel0827_T
  AVANTI2_ertmodel0827_ConstB = {
  {
    1.6931,
    0.0,
    0.0826,
    0.0,
    0.0,
    0.0,
    0.0,
    1.0708,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0826,
    0.0,
    2.7281,
    0.0,
    0.0,
    0.0
  }
  ,                                    // '<S9>/Vector Concatenate'

  {
    1.6931,
    0.0,
    0.0826,
    0.0,
    1.0708,
    0.0,
    0.0826,
    0.0,
    2.7281
  }
  ,                                    // '<S8>/Selector'

  {
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0
  }
  ,                                    // '<S8>/Selector1'

  {
    1.6931,
    0.0,
    0.0826,
    0.0,
    1.0708,
    0.0,
    0.0826,
    0.0,
    2.7281
  }
  ,                                    // '<S8>/Selector2'
  0.0
  ,                                    // '<S32>/Gain1'

  {
    0.0,
    0.0,
    66.708
  }
  ,                                    // '<S26>/G=mg'
  0.031608171698050413
  ,                                    // '<S29>/Divide'
  0.17778687155707087
  ,                                    // '<S29>/Sqrt'
  0.35557374311414175
  ,                                    // '<S29>/Gain'
  6.2831853071795862
  ,                                    // '<S29>/Gain8'
  6.2831853071795862
  ,                                    // '<S29>/Gain9'
  0.015985224289494965
  ,                                    // '<S29>/Power1'
  0.0056839260351348219
  // '<S29>/Power2'
};

// Constant parameters (default storage)
const AVANTI2_ertmodel0827ModelClass::ConstP_AVANTI2_ertmodel0827_T
  AVANTI2_ertmodel0827_ConstP = {
  // Expression: e_lim*[-1:1]
  //  Referenced by: '<S1>/1-D Lookup Table'

  { -12.0, 0.0, 12.0 },

  // Pooled Parameter (Expression: [-1:1])
  //  Referenced by:
  //    '<S1>/1-D Lookup Table'
  //    '<S1>/1-D Lookup Table1'
  //    '<S1>/1-D Lookup Table2'

  { -1.0, 0.0, 1.0 },

  // Expression: a_lim*[-1:1]
  //  Referenced by: '<S1>/1-D Lookup Table1'

  { -10.0, 0.0, 10.0 },

  // Expression: r_lim*[-1:1]
  //  Referenced by: '<S1>/1-D Lookup Table2'

  { -22.0, 0.0, 22.0 }
};

//
// File trailer for generated code.
//
// [EOF]
//
