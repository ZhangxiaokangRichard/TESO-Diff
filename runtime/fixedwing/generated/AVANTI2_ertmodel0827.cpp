//
// File: AVANTI2_ertmodel0827.cpp
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

real_T look1_binlxpw(real_T u0, const real_T bp0[], const real_T table[],
                     uint32_T maxIndex)
{
  real_T frac;
  real_T yL_0d0;
  uint32_T bpIdx;
  uint32_T iLeft;
  uint32_T iRght;

  // Column-major Lookup 1-D
  // Search method: 'binary'
  // Use previous index: 'off'
  // Interpolation method: 'Linear point-slope'
  // Extrapolation method: 'Linear'
  // Use last breakpoint for index at or above upper limit: 'off'
  // Remove protection against out-of-range input in generated code: 'off'

  // Prelookup - Index and Fraction
  // Index Search method: 'binary'
  // Extrapolation method: 'Linear'
  // Use previous index: 'off'
  // Use last breakpoint for index at or above upper limit: 'off'
  // Remove protection against out-of-range input in generated code: 'off'

  if (u0 <= bp0[0U]) {
    iLeft = 0U;
    frac = (u0 - bp0[0U]) / (bp0[1U] - bp0[0U]);
  } else if (u0 < bp0[maxIndex]) {
    // Binary Search
    bpIdx = maxIndex >> 1U;
    iLeft = 0U;
    iRght = maxIndex;
    while (iRght - iLeft > 1U) {
      if (u0 < bp0[bpIdx]) {
        iRght = bpIdx;
      } else {
        iLeft = bpIdx;
      }

      bpIdx = (iRght + iLeft) >> 1U;
    }

    frac = (u0 - bp0[iLeft]) / (bp0[iLeft + 1U] - bp0[iLeft]);
  } else {
    iLeft = maxIndex - 1U;
    frac = (u0 - bp0[maxIndex - 1U]) / (bp0[maxIndex] - bp0[maxIndex - 1U]);
  }

  // Column-major Interpolation 1-D
  // Interpolation method: 'Linear point-slope'
  // Use last breakpoint for index at or above upper limit: 'off'
  // Overflow mode: 'portable wrapping'

  yL_0d0 = table[iLeft];
  return (table[iLeft + 1U] - yL_0d0) * frac + yL_0d0;
}

// State reduction function
void local_stateReduction(real_T* x, int_T* p, int_T n, real_T* r)
{
  int_T i, j;
  for (i = 0, j = 0; i < n; ++i, ++j) {
    int_T k = p[i];
    real_T lb = r[j++];
    real_T xk = x[k]-lb;
    real_T rk = r[j]-lb;
    int_T q = (int_T) std::floor(xk/rk);
    if (q) {
      x[k] = xk-q*rk+lb;
    }
  }
}

//
// This function updates continuous states using the ODE4 fixed-step
// solver algorithm
//
void AVANTI2_ertmodel0827ModelClass::rt_ertODEUpdateContinuousStates
  (RTWSolverInfo *si )
{
  time_T t = rtsiGetT(si);
  time_T tnew = rtsiGetSolverStopTime(si);
  time_T h = rtsiGetStepSize(si);
  real_T *x = rtsiGetContStates(si);
  ODE4_IntgData *id = static_cast<ODE4_IntgData *>(rtsiGetSolverData(si));
  real_T *y = id->y;
  real_T *f0 = id->f[0];
  real_T *f1 = id->f[1];
  real_T *f2 = id->f[2];
  real_T *f3 = id->f[3];
  real_T temp;
  int_T i;
  int_T nXc = 12;
  rtsiSetSimTimeStep(si,MINOR_TIME_STEP);

  // Save the state values at time t in y, we'll use x as ynew.
  (void) std::memcpy(y, x,
                     static_cast<uint_T>(nXc)*sizeof(real_T));

  // Assumes that rtsiSetT and ModelOutputs are up-to-date
  // f0 = f(t,y)
  rtsiSetdX(si, f0);
  AVANTI2_ertmodel0827_derivatives();

  // f1 = f(t + (h/2), y + (h/2)*f0)
  temp = 0.5 * h;
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (temp*f0[i]);
  }

  rtsiSetT(si, t + temp);
  rtsiSetdX(si, f1);
  this->step();
  AVANTI2_ertmodel0827_derivatives();

  // f2 = f(t + (h/2), y + (h/2)*f1)
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (temp*f1[i]);
  }

  rtsiSetdX(si, f2);
  this->step();
  AVANTI2_ertmodel0827_derivatives();

  // f3 = f(t + h, y + h*f2)
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (h*f2[i]);
  }

  rtsiSetT(si, tnew);
  rtsiSetdX(si, f3);
  this->step();
  AVANTI2_ertmodel0827_derivatives();

  // tnew = t + h
  // ynew = y + (h/6)*(f0 + 2*f1 + 2*f2 + 2*f3)
  temp = h / 6.0;
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + temp*(f0[i] + 2.0*f1[i] + 2.0*f2[i] + f3[i]);
  }

  local_stateReduction(rtsiGetContStates(si), rtsiGetPeriodicContStateIndices(si),
                       3,
                       rtsiGetPeriodicContStateRanges(si));
  rtsiSetSimTimeStep(si,MAJOR_TIME_STEP);
}

real_T rt_powd_snf(real_T u0, real_T u1)
{
  real_T tmp;
  real_T tmp_0;
  real_T y;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = (rtNaN);
  } else {
    tmp = std::abs(u0);
    tmp_0 = std::abs(u1);
    if (rtIsInf(u1)) {
      if (tmp == 1.0) {
        y = 1.0;
      } else if (tmp > 1.0) {
        if (u1 > 0.0) {
          y = (rtInf);
        } else {
          y = 0.0;
        }
      } else if (u1 > 0.0) {
        y = 0.0;
      } else {
        y = (rtInf);
      }
    } else if (tmp_0 == 0.0) {
      y = 1.0;
    } else if (tmp_0 == 1.0) {
      if (u1 > 0.0) {
        y = u0;
      } else {
        y = 1.0 / u0;
      }
    } else if (u1 == 2.0) {
      y = u0 * u0;
    } else if ((u1 == 0.5) && (u0 >= 0.0)) {
      y = std::sqrt(u0);
    } else if ((u0 < 0.0) && (u1 > std::floor(u1))) {
      y = (rtNaN);
    } else {
      y = std::pow(u0, u1);
    }
  }

  return y;
}

void rt_mrdivide_U1d1x3_U2d_9vOrDY9Z(const real_T u0[3], const real_T u1[9],
  real_T y[3])
{
  real_T A[9];
  real_T a21;
  real_T maxval;
  int32_T r1;
  int32_T r2;
  int32_T r3;
  int32_T rtemp;
  std::memcpy(&A[0], &u1[0], 9U * sizeof(real_T));
  r1 = 0;
  r2 = 1;
  r3 = 2;
  maxval = std::abs(u1[0]);
  a21 = std::abs(u1[1]);
  if (a21 > maxval) {
    maxval = a21;
    r1 = 1;
    r2 = 0;
  }

  if (std::abs(u1[2]) > maxval) {
    r1 = 2;
    r2 = 1;
    r3 = 0;
  }

  A[r2] = u1[r2] / u1[r1];
  A[r3] /= A[r1];
  A[r2 + 3] -= A[r1 + 3] * A[r2];
  A[r3 + 3] -= A[r1 + 3] * A[r3];
  A[r2 + 6] -= A[r1 + 6] * A[r2];
  A[r3 + 6] -= A[r1 + 6] * A[r3];
  if (std::abs(A[r3 + 3]) > std::abs(A[r2 + 3])) {
    rtemp = r2 + 1;
    r2 = r3;
    r3 = rtemp - 1;
  }

  A[r3 + 3] /= A[r2 + 3];
  A[r3 + 6] -= A[r3 + 3] * A[r2 + 6];
  y[r1] = u0[0] / A[r1];
  y[r2] = u0[1] - A[r1 + 3] * y[r1];
  y[r3] = u0[2] - A[r1 + 6] * y[r1];
  y[r2] /= A[r2 + 3];
  y[r3] -= A[r2 + 6] * y[r2];
  y[r3] /= A[r3 + 6];
  y[r2] -= A[r3 + 3] * y[r3];
  y[r1] -= y[r3] * A[r3];
  y[r1] -= y[r2] * A[r2];
}

// Model step function
void AVANTI2_ertmodel0827ModelClass::step()
{
  real_T rtb_Transpose1[9];
  real_T rtb_Transpose1_0[9];
  real_T rtb_VectorConcatenate[9];
  real_T tmp[9];
  real_T tmp_0[9];
  real_T tmp_1[9];
  real_T tmp_2[9];
  real_T tmp_3[9];
  real_T rtb_Sum1_f[3];
  real_T rtb_UnitConversion[3];
  real_T rtb_V_windtoBody[3];
  real_T rtb_VectorConcatenate_0[3];
  real_T rtb_sincos_o2_i[3];
  real_T Switch;
  real_T rtb_Divide1;
  real_T rtb_Gain1_h;
  real_T rtb_Sqrt;
  real_T rtb_p;
  real_T rtb_q;
  real_T rtb_q_dot;
  real_T rtb_r;
  real_T rtb_r_dot;
  real_T rtb_sincos_o2_e_idx_1;
  real_T rtb_sincos_o2_idx_0;
  real_T rtb_sincos_o2_idx_1;
  real_T rtb_sincos_o2_idx_2;
  real_T rtb_v;
  real_T rtb_w;
  real_T u0;
  real_T u0_0;
  real_T u0_1;
  int32_T iU;
  int32_T rtb_Switch1;
  if (rtmIsMajorTimeStep((&AVANTI2_ertmodel0827_M))) {
    // set solver stop time
    rtsiSetSolverStopTime(&(&AVANTI2_ertmodel0827_M)->solverInfo,
                          (((&AVANTI2_ertmodel0827_M)->Timing.clockTick0+1)*
      (&AVANTI2_ertmodel0827_M)->Timing.stepSize0));
  }                                    // end MajorTimeStep

  // Update absolute time of base rate at minor time step
  if (rtmIsMinorTimeStep((&AVANTI2_ertmodel0827_M))) {
    (&AVANTI2_ertmodel0827_M)->Timing.t[0] = rtsiGetT(&(&AVANTI2_ertmodel0827_M
      )->solverInfo);
  }

  // Trigonometry: '<S15>/sincos' incorporates:
  //   Integrator: '<S7>/phi theta psi'
  //   SignalConversion generated from: '<S15>/sincos'

  rtb_sincos_o2_idx_0 = std::cos(AVANTI2_ertmodel0827_X.phithetapsi_CSTATE[2]);
  Switch = std::sin(AVANTI2_ertmodel0827_X.phithetapsi_CSTATE[2]);
  rtb_sincos_o2_idx_1 = std::cos(AVANTI2_ertmodel0827_X.phithetapsi_CSTATE[1]);
  rtb_Sqrt = std::sin(AVANTI2_ertmodel0827_X.phithetapsi_CSTATE[1]);
  rtb_sincos_o2_idx_2 = std::cos(AVANTI2_ertmodel0827_X.phithetapsi_CSTATE[0]);
  rtb_Gain1_h = std::sin(AVANTI2_ertmodel0827_X.phithetapsi_CSTATE[0]);

  // Fcn: '<S15>/Fcn11'
  rtb_VectorConcatenate[0] = rtb_sincos_o2_idx_0 * rtb_sincos_o2_idx_1;

  // Fcn: '<S15>/Fcn21' incorporates:
  //   Fcn: '<S15>/Fcn22'

  rtb_sincos_o2_e_idx_1 = rtb_Sqrt * rtb_Gain1_h;
  rtb_VectorConcatenate[1] = rtb_sincos_o2_e_idx_1 * rtb_sincos_o2_idx_0 -
    Switch * rtb_sincos_o2_idx_2;

  // Fcn: '<S15>/Fcn31' incorporates:
  //   Fcn: '<S15>/Fcn32'

  rtb_Divide1 = rtb_Sqrt * rtb_sincos_o2_idx_2;
  rtb_VectorConcatenate[2] = rtb_Divide1 * rtb_sincos_o2_idx_0 + Switch *
    rtb_Gain1_h;

  // Fcn: '<S15>/Fcn12'
  rtb_VectorConcatenate[3] = Switch * rtb_sincos_o2_idx_1;

  // Fcn: '<S15>/Fcn22'
  rtb_VectorConcatenate[4] = rtb_sincos_o2_e_idx_1 * Switch +
    rtb_sincos_o2_idx_0 * rtb_sincos_o2_idx_2;

  // Fcn: '<S15>/Fcn32'
  rtb_VectorConcatenate[5] = rtb_Divide1 * Switch - rtb_sincos_o2_idx_0 *
    rtb_Gain1_h;

  // Fcn: '<S15>/Fcn13'
  rtb_VectorConcatenate[6] = -rtb_Sqrt;

  // Fcn: '<S15>/Fcn23'
  rtb_VectorConcatenate[7] = rtb_sincos_o2_idx_1 * rtb_Gain1_h;

  // Fcn: '<S15>/Fcn33'
  rtb_VectorConcatenate[8] = rtb_sincos_o2_idx_1 * rtb_sincos_o2_idx_2;
  if (rtmIsMajorTimeStep((&AVANTI2_ertmodel0827_M))) {
    // ZeroOrderHold: '<Root>/CodegenInputHold_w_x' incorporates:
    //   Inport: '<Root>/w_x'

    AVANTI2_ertmodel0827_B.wind[0] = AVANTI2_ertmodel0827_U.w_x;

    // ZeroOrderHold: '<Root>/CodegenInputHold_w_y' incorporates:
    //   Inport: '<Root>/w_y'

    AVANTI2_ertmodel0827_B.wind[1] = AVANTI2_ertmodel0827_U.w_y;

    // ZeroOrderHold: '<Root>/CodegenInputHold_w_z' incorporates:
    //   Inport: '<Root>/w_z'

    AVANTI2_ertmodel0827_B.wind[2] = AVANTI2_ertmodel0827_U.w_z;
  }

  // Sum: '<S25>/Sum' incorporates:
  //   Concatenate: '<S17>/Vector Concatenate'
  //   Integrator: '<S2>/ub,vb,wb'
  //   Product: '<S25>/V_wind to Body'

  for (iU = 0; iU < 3; iU++) {
    rtb_V_windtoBody[iU] = AVANTI2_ertmodel0827_X.ubvbwb_CSTATE[iU] -
      ((rtb_VectorConcatenate[iU + 3] * AVANTI2_ertmodel0827_B.wind[1] +
        rtb_VectorConcatenate[iU] * AVANTI2_ertmodel0827_B.wind[0]) +
       rtb_VectorConcatenate[iU + 6] * AVANTI2_ertmodel0827_B.wind[2]);
  }

  // End of Sum: '<S25>/Sum'

  // Clock: '<S25>/Clock1' incorporates:
  //   Clock: '<S25>/Clock2'
  //   Clock: '<S25>/Clock3'
  //   Clock: '<S25>/Clock4'
  //   Clock: '<S25>/Clock5'
  //   Clock: '<S25>/Clock6'
  //   Clock: '<S25>/Clock7'
  //   Clock: '<S25>/Clock8'
  //   Clock: '<S25>/Clock9'

  rtb_sincos_o2_idx_1 = (&AVANTI2_ertmodel0827_M)->Timing.t[0];

  // Switch: '<S25>/Switch3' incorporates:
  //   Clock: '<S25>/Clock1'
  //   Product: '<S25>/Divide'
  //   Sum: '<S25>/Sum1'
  //   Trigonometry: '<S25>/Atan'

  if (rtb_sincos_o2_idx_1 > 0.0) {
    rtb_sincos_o2_idx_0 = std::atan(rtb_V_windtoBody[2] / rtb_V_windtoBody[0]) +
      AVANTI2_ertmodel0827_ConstB.Gain1;
  } else {
    rtb_sincos_o2_idx_0 = AVANTI2_ertmodel0827_ConstB.Gain1;
  }

  // End of Switch: '<S25>/Switch3'

  // Sqrt: '<S25>/Sqrt' incorporates:
  //   Math: '<S25>/Square1'
  //   Math: '<S25>/Square2'
  //   Math: '<S25>/Square3'
  //   Sum: '<S25>/Sum7'

  rtb_Sqrt = std::sqrt((rtb_V_windtoBody[0] * rtb_V_windtoBody[0] +
                        rtb_V_windtoBody[1] * rtb_V_windtoBody[1]) +
                       rtb_V_windtoBody[2] * rtb_V_windtoBody[2]);

  // Switch: '<S25>/Switch1'
  rtb_Switch1 = (rtb_Sqrt > 0.0);

  // Switch: '<S25>/Switch4' incorporates:
  //   Switch: '<S25>/Switch'
  //   Switch: '<S25>/Switch17'
  //   Switch: '<S25>/Switch2'

  if (rtb_Switch1 > 0) {
    // Product: '<S25>/Divide1'
    rtb_sincos_o2_e_idx_1 = rtb_V_windtoBody[1] / rtb_Sqrt;

    // Trigonometry: '<S25>/Atan1'
    if (rtb_sincos_o2_e_idx_1 > 1.0) {
      rtb_sincos_o2_e_idx_1 = 1.0;
    } else if (rtb_sincos_o2_e_idx_1 < -1.0) {
      rtb_sincos_o2_e_idx_1 = -1.0;
    }

    // Switch: '<S25>/Switch' incorporates:
    //   Trigonometry: '<S25>/Atan1'

    Switch = std::asin(rtb_sincos_o2_e_idx_1);
  } else if (rtb_sincos_o2_idx_1 > 0.0) {
    // Product: '<S25>/Divide1'
    rtb_sincos_o2_e_idx_1 = rtb_V_windtoBody[1] / rtb_Sqrt;

    // Trigonometry: '<S25>/Atan1'
    if (rtb_sincos_o2_e_idx_1 > 1.0) {
      rtb_sincos_o2_e_idx_1 = 1.0;
    } else if (rtb_sincos_o2_e_idx_1 < -1.0) {
      rtb_sincos_o2_e_idx_1 = -1.0;
    }

    // Switch: '<S25>/Switch2' incorporates:
    //   Switch: '<S25>/Switch'
    //   Trigonometry: '<S25>/Atan1'

    Switch = std::asin(rtb_sincos_o2_e_idx_1);
  } else {
    // Switch: '<S25>/Switch4' incorporates:
    //   Constant: '<S25>/Constant15'
    //   Switch: '<S25>/Switch17'

    rtb_sincos_o2_idx_0 = 0.0;

    // Switch: '<S25>/Switch' incorporates:
    //   Constant: '<S25>/Constant6'
    //   Switch: '<S25>/Switch2'

    Switch = 0.0;
  }

  // End of Switch: '<S25>/Switch4'

  // Trigonometry: '<S33>/sincos' incorporates:
  //   SignalConversion generated from: '<S25>/Matrix Concatenate'
  //
  rtb_sincos_o2_idx_2 = std::cos(rtb_sincos_o2_idx_0);
  rtb_Gain1_h = std::sin(rtb_sincos_o2_idx_0);
  rtb_sincos_o2_e_idx_1 = std::cos(Switch);
  rtb_Divide1 = std::sin(Switch);

  // Product: '<S34>/u(3)*u(4)'
  rtb_Transpose1[0] = rtb_sincos_o2_idx_2 * rtb_sincos_o2_e_idx_1;

  // UnaryMinus: '<S37>/Unary Minus' incorporates:
  //   Product: '<S37>/u(2)*u(3)'

  rtb_Transpose1[1] = -(rtb_sincos_o2_idx_2 * rtb_Divide1);

  // UnaryMinus: '<S40>/Unary Minus'
  rtb_Transpose1[2] = -rtb_Gain1_h;

  // SignalConversion generated from: '<S43>/Vector Concatenate'
  rtb_Transpose1[3] = rtb_Divide1;

  // SignalConversion generated from: '<S43>/Vector Concatenate'
  rtb_Transpose1[4] = rtb_sincos_o2_e_idx_1;

  // SignalConversion generated from: '<S43>/Vector Concatenate' incorporates:
  //   Constant: '<S41>/Constant'

  rtb_Transpose1[5] = 0.0;

  // Product: '<S36>/u(1)*u(4)'
  rtb_Transpose1[6] = rtb_Gain1_h * rtb_sincos_o2_e_idx_1;

  // UnaryMinus: '<S39>/Unary Minus' incorporates:
  //   Product: '<S39>/u(1)*u(2)'

  rtb_Transpose1[7] = -(rtb_Gain1_h * rtb_Divide1);

  // SignalConversion generated from: '<S43>/Vector Concatenate'
  rtb_Transpose1[8] = rtb_sincos_o2_idx_2;

  // Math: '<S25>/Transpose1'
  for (iU = 0; iU < 3; iU++) {
    rtb_Transpose1_0[3 * iU] = rtb_Transpose1[iU];
    rtb_Transpose1_0[3 * iU + 1] = rtb_Transpose1[iU + 3];
    rtb_Transpose1_0[3 * iU + 2] = rtb_Transpose1[iU + 6];
  }

  std::memcpy(&rtb_Transpose1[0], &rtb_Transpose1_0[0], 9U * sizeof(real_T));

  // End of Math: '<S25>/Transpose1'

  // Switch: '<S6>/Switch' incorporates:
  //   Gain: '<S6>/Gain'
  //   Integrator: '<S2>/xe,ye,ze'

  if (-AVANTI2_ertmodel0827_X.xeyeze_CSTATE[2] > 0.0) {
    rtb_Gain1_h = -AVANTI2_ertmodel0827_X.xeyeze_CSTATE[2];
  } else {
    rtb_Gain1_h = 0.0;
  }

  // End of Switch: '<S6>/Switch'

  // Saturate: '<S28>/Limit  altitude  to troposhere'
  if (rtb_Gain1_h > 11000.0) {
    rtb_sincos_o2_idx_2 = 11000.0;
  } else if (rtb_Gain1_h < 0.0) {
    rtb_sincos_o2_idx_2 = 0.0;
  } else {
    rtb_sincos_o2_idx_2 = rtb_Gain1_h;
  }

  // End of Saturate: '<S28>/Limit  altitude  to troposhere'

  // Sum: '<S28>/Sum1' incorporates:
  //   Constant: '<S28>/Sea Level  Temperature'
  //   Gain: '<S28>/Lapse Rate'

  rtb_Divide1 = 288.15 - 0.0065 * rtb_sincos_o2_idx_2;

  // Gain: '<S28>/1//T0'
  rtb_sincos_o2_idx_2 = 0.00347041471455839 * rtb_Divide1;

  // Saturate: '<S28>/Limit  altitude  to Stratosphere' incorporates:
  //   Constant: '<S28>/Altitude of Troposphere'
  //   Sum: '<S28>/Sum'

  if (11000.0 - rtb_Gain1_h > 0.0) {
    rtb_sincos_o2_e_idx_1 = 0.0;
  } else if (11000.0 - rtb_Gain1_h < -9000.0) {
    rtb_sincos_o2_e_idx_1 = -9000.0;
  } else {
    rtb_sincos_o2_e_idx_1 = 11000.0 - rtb_Gain1_h;
  }

  // End of Saturate: '<S28>/Limit  altitude  to Stratosphere'

  // Product: '<S28>/Product3' incorporates:
  //   Constant: '<S28>/Constant'
  //   Gain: '<S28>/g//R'
  //   Gain: '<S28>/rho0'
  //   Math: '<S28>/(T//T0)^(g//LR) '
  //   Math: '<S28>/Stratosphere Model'
  //   Product: '<S28>/Product'
  //   Product: '<S28>/Product1'
  //
  //  About '<S28>/Stratosphere Model':
  //   Operator: exp

  rtb_Gain1_h = rt_powd_snf(rtb_sincos_o2_idx_2, 5.2558756014667134) /
    rtb_sincos_o2_idx_2 * 1.225 * std::exp(1.0 / rtb_Divide1 *
    (0.034163191409533639 * rtb_sincos_o2_e_idx_1));

  // Product: '<S25>/Product1' incorporates:
  //   Gain: '<S25>/Gain'
  //   Integrator: '<S2>/ub,vb,wb'
  //   Math: '<S25>/Square4'
  //   Math: '<S25>/Square5'
  //   Math: '<S25>/Square6'
  //   Sum: '<S25>/Sum18'

  rtb_sincos_o2_idx_2 = ((AVANTI2_ertmodel0827_X.ubvbwb_CSTATE[0] *
    AVANTI2_ertmodel0827_X.ubvbwb_CSTATE[0] +
    AVANTI2_ertmodel0827_X.ubvbwb_CSTATE[1] *
    AVANTI2_ertmodel0827_X.ubvbwb_CSTATE[1]) +
    AVANTI2_ertmodel0827_X.ubvbwb_CSTATE[2] *
    AVANTI2_ertmodel0827_X.ubvbwb_CSTATE[2]) * 0.5 * rtb_Gain1_h;

  // Sum: '<S25>/Sum4' incorporates:
  //   Constant: '<S25>/Constant4'
  //   Gain: '<S25>/Gain43'
  //   Sum: '<S25>/Sum2'

  rtb_Divide1 = 6.4854 * rtb_sincos_o2_idx_0 + 0.6483;
  if (rtmIsMajorTimeStep((&AVANTI2_ertmodel0827_M))) {
    // Lookup_n-D: '<S1>/1-D Lookup Table' incorporates:
    //   Inport: '<Root>/delta_e'

    rtb_sincos_o2_e_idx_1 = look1_binlxpw(AVANTI2_ertmodel0827_U.delta_e,
      AVANTI2_ertmodel0827_ConstP.pooled11,
      AVANTI2_ertmodel0827_ConstP.uDLookupTable_tableData, 2U);

    // Lookup_n-D: '<S1>/1-D Lookup Table1' incorporates:
    //   Inport: '<Root>/delta_a'

    u0 = look1_binlxpw(AVANTI2_ertmodel0827_U.delta_a,
                       AVANTI2_ertmodel0827_ConstP.pooled11,
                       AVANTI2_ertmodel0827_ConstP.uDLookupTable1_tableData, 2U);

    // Lookup_n-D: '<S1>/1-D Lookup Table2' incorporates:
    //   Inport: '<Root>/delta_r'

    u0_0 = look1_binlxpw(AVANTI2_ertmodel0827_U.delta_r,
                         AVANTI2_ertmodel0827_ConstP.pooled11,
                         AVANTI2_ertmodel0827_ConstP.uDLookupTable2_tableData,
                         2U);

    // Saturate: '<S46>/Command Saturation'
    // Unit Conversion - from: deg to: rad
    // Expression: output = (0.0174533*input) + (0)
    if (rtb_sincos_o2_e_idx_1 > 12.0) {
      u0_1 = 12.0;
    } else if (rtb_sincos_o2_e_idx_1 < -12.0) {
      u0_1 = -12.0;
    } else {
      u0_1 = rtb_sincos_o2_e_idx_1;
    }

    // UnitConversion: '<S44>/Unit Conversion'
    rtb_UnitConversion[0] = 0.017453292519943295 * u0_1;

    // Saturate: '<S45>/Command Saturation'
    if (u0 > 10.0) {
      u0 = 10.0;
    } else if (u0 < -10.0) {
      u0 = -10.0;
    }

    // End of Saturate: '<S45>/Command Saturation'

    // UnitConversion: '<S44>/Unit Conversion'
    rtb_UnitConversion[1] = 0.017453292519943295 * u0;

    // Saturate: '<S47>/Command Saturation'
    if (u0_0 > 25.0) {
      u0_0 = 25.0;
    } else if (u0_0 < -25.0) {
      u0_0 = -25.0;
    }

    // End of Saturate: '<S47>/Command Saturation'

    // UnitConversion: '<S44>/Unit Conversion'
    rtb_UnitConversion[2] = 0.017453292519943295 * u0_0;

    // Saturate: '<S46>/Command Saturation'
    if (rtb_sincos_o2_e_idx_1 > 12.0) {
      rtb_sincos_o2_e_idx_1 = 12.0;
    } else if (rtb_sincos_o2_e_idx_1 < -12.0) {
      rtb_sincos_o2_e_idx_1 = -12.0;
    }

    // Gain: '<S25>/Gain46' incorporates:
    //   UnitConversion: '<S44>/Unit Conversion'

    AVANTI2_ertmodel0827_B.Gain46 = 0.017453292519943295 * rtb_sincos_o2_e_idx_1
      * 0.2925;

    // Gain: '<S25>/Gain36'
    AVANTI2_ertmodel0827_B.Gain36 = -0.1358 * rtb_UnitConversion[1];

    // Gain: '<S25>/Gain27'
    AVANTI2_ertmodel0827_B.Gain27 = -0.1472 * rtb_UnitConversion[2];
  }

  // Switch: '<S25>/Switch12' incorporates:
  //   Switch: '<S25>/Switch13'

  if (rtb_Switch1 > 0) {
    // Gain: '<S25>/Gain16' incorporates:
    //   Constant: '<S25>/Constant2'
    //   Gain: '<S25>/Gain42'
    //   MATLAB Function: '<S25>/0.018u^2+0.003583u+0.00128'
    //   Product: '<S25>/Product8'
    //   Sum: '<S25>/Sum4'
    //   Sum: '<S25>/Sum8'

    rtb_Sum1_f[0] = -(((((rtb_Divide1 * rtb_Divide1 * 0.018 + 0.003583 *
                          rtb_Divide1) + 0.00128) + 0.017) +
                       AVANTI2_ertmodel0827_B.Gain46) * 0.957 *
                      rtb_sincos_o2_idx_2);
  } else if (rtb_sincos_o2_idx_1 > 0.0) {
    // Switch: '<S25>/Switch13' incorporates:
    //   Constant: '<S25>/Constant2'
    //   Gain: '<S25>/Gain16'
    //   Gain: '<S25>/Gain42'
    //   MATLAB Function: '<S25>/0.018u^2+0.003583u+0.00128'
    //   Product: '<S25>/Product8'
    //   Sum: '<S25>/Sum4'
    //   Sum: '<S25>/Sum8'

    rtb_Sum1_f[0] = -(((((rtb_Divide1 * rtb_Divide1 * 0.018 + 0.003583 *
                          rtb_Divide1) + 0.00128) + 0.017) +
                       AVANTI2_ertmodel0827_B.Gain46) * 0.957 *
                      rtb_sincos_o2_idx_2);
  } else {
    // Gain: '<S25>/Gain16' incorporates:
    //   Constant: '<S25>/Constant13'
    //   Switch: '<S25>/Switch13'

    rtb_Sum1_f[0] = -0.0;
  }

  // End of Switch: '<S25>/Switch12'
  if (rtmIsMajorTimeStep((&AVANTI2_ertmodel0827_M))) {
    // Gain: '<S25>/Gain10'
    AVANTI2_ertmodel0827_B.Gain10 = 0.4561 * rtb_UnitConversion[0];
  }

  // Switch: '<S25>/Switch10' incorporates:
  //   Constant: '<S25>/Constant18'
  //   Constant: '<S25>/Constant21'
  //   Constant: '<S25>/Constant22'
  //   Constant: '<S25>/Constant23'
  //   Gain: '<S25>/Gain1'
  //   Gain: '<S25>/Gain17'
  //   Gain: '<S25>/Gain24'
  //   Gain: '<S25>/Gain28'
  //   Gain: '<S25>/Gain29'
  //   Gain: '<S25>/Gain30'
  //   Gain: '<S25>/Gain35'
  //   Gain: '<S25>/Gain8'
  //   Gain: '<S25>/Gain9'
  //   Integrator: '<S2>/p,q,r '
  //   Product: '<S25>/Divide10'
  //   Product: '<S25>/Divide4'
  //   Product: '<S25>/Divide9'
  //   Product: '<S25>/Product3'
  //   Product: '<S25>/Product6'
  //   Sum: '<S25>/Sum14'
  //   Sum: '<S25>/Sum15'
  //   Sum: '<S25>/Sum16'
  //   Sum: '<S25>/Sum2'
  //   Sum: '<S25>/Sum6'
  //   Switch: '<S25>/Switch5'
  //   Switch: '<S25>/Switch7'

  if (rtb_Switch1 > 0) {
    rtb_Sum1_f[1] = ((((2.16 * AVANTI2_ertmodel0827_X.pqr_CSTATE[0] / (rtb_Sqrt
      + 1.0E-6) * -0.2929 + -0.2527 * Switch) + 2.16 *
                       AVANTI2_ertmodel0827_X.pqr_CSTATE[2] / (rtb_Sqrt + 1.0E-6)
                       * 0.1497) + AVANTI2_ertmodel0827_B.Gain36) +
                     AVANTI2_ertmodel0827_B.Gain27) * 0.957 *
      rtb_sincos_o2_idx_2;
    rtb_Divide1 = ((0.11076388888888888 * AVANTI2_ertmodel0827_X.pqr_CSTATE[1] /
                    (rtb_Sqrt + 1.0E-6) * 11.5476 + rtb_Divide1) +
                   AVANTI2_ertmodel0827_B.Gain10) * 0.957 * rtb_sincos_o2_idx_2;
  } else {
    if (rtb_sincos_o2_idx_1 > 0.0) {
      // Switch: '<S25>/Switch7' incorporates:
      //   Constant: '<S25>/Constant21'
      //   Constant: '<S25>/Constant22'
      //   Gain: '<S25>/Gain17'
      //   Gain: '<S25>/Gain24'
      //   Gain: '<S25>/Gain28'
      //   Gain: '<S25>/Gain29'
      //   Gain: '<S25>/Gain30'
      //   Gain: '<S25>/Gain35'
      //   Integrator: '<S2>/p,q,r '
      //   Product: '<S25>/Divide10'
      //   Product: '<S25>/Divide9'
      //   Product: '<S25>/Product6'
      //   Sum: '<S25>/Sum14'
      //   Sum: '<S25>/Sum15'
      //   Sum: '<S25>/Sum6'

      rtb_Sum1_f[1] = ((((2.16 * AVANTI2_ertmodel0827_X.pqr_CSTATE[0] /
                          (rtb_Sqrt + 1.0E-6) * -0.2929 + -0.2527 * Switch) +
                         2.16 * AVANTI2_ertmodel0827_X.pqr_CSTATE[2] / (rtb_Sqrt
        + 1.0E-6) * 0.1497) + AVANTI2_ertmodel0827_B.Gain36) +
                       AVANTI2_ertmodel0827_B.Gain27) * 0.957 *
        rtb_sincos_o2_idx_2;
    } else {
      rtb_Sum1_f[1] = 0.0;
    }

    // Switch: '<S25>/Switch14' incorporates:
    //   Constant: '<S25>/Constant18'
    //   Constant: '<S25>/Constant23'
    //   Constant: '<S25>/Constant3'
    //   Constant: '<S25>/Constant9'
    //   Gain: '<S25>/Gain1'
    //   Gain: '<S25>/Gain7'
    //   Gain: '<S25>/Gain8'
    //   Gain: '<S25>/Gain9'
    //   Integrator: '<S2>/p,q,r '
    //   Product: '<S25>/Divide4'
    //   Product: '<S25>/Product3'
    //   Sum: '<S25>/Sum16'
    //   Sum: '<S25>/Sum2'
    //   Switch: '<S25>/Switch7'

    if (rtb_sincos_o2_idx_1 > 0.0) {
      rtb_Divide1 = ((0.11076388888888888 * AVANTI2_ertmodel0827_X.pqr_CSTATE[1]
                      / (rtb_Sqrt + 1.0E-6) * 11.5476 + (6.4854 *
        rtb_sincos_o2_idx_0 + 0.6483)) + AVANTI2_ertmodel0827_B.Gain10) * 0.957 *
        rtb_sincos_o2_idx_2;
    } else {
      rtb_Divide1 = 0.0;
    }

    // End of Switch: '<S25>/Switch14'
  }

  // End of Switch: '<S25>/Switch10'

  // Gain: '<S25>/Gain23'
  rtb_Sum1_f[2] = -rtb_Divide1;
  if (rtmIsMajorTimeStep((&AVANTI2_ertmodel0827_M))) {
    // Saturate: '<Root>/Saturation' incorporates:
    //   Inport: '<Root>/delta_t'

    if (AVANTI2_ertmodel0827_U.delta_t > 1.0) {
      rtb_sincos_o2_e_idx_1 = 1.0;
    } else if (AVANTI2_ertmodel0827_U.delta_t < 0.0) {
      rtb_sincos_o2_e_idx_1 = 0.0;
    } else {
      rtb_sincos_o2_e_idx_1 = AVANTI2_ertmodel0827_U.delta_t;
    }

    // End of Saturate: '<Root>/Saturation'

    // Gain: '<S29>/Gain4' incorporates:
    //   Constant: '<S29>/Constant'
    //   Math: '<S29>/Power'

    rtb_r_dot = 900.0 * rt_powd_snf(rtb_sincos_o2_e_idx_1, 0.98);

    // Product: '<S29>/Product2'
    rtb_q_dot = AVANTI2_ertmodel0827_ConstB.Sqrt * rtb_r_dot;

    // Switch: '<S29>/Switch1'
    if (rtb_q_dot > 0.0) {
      // Switch: '<S29>/Switch1'
      AVANTI2_ertmodel0827_B.Switch1 = rtb_q_dot;
    } else {
      // Switch: '<S29>/Switch1' incorporates:
      //   Constant: '<S29>/Constant4'

      AVANTI2_ertmodel0827_B.Switch1 = 0.001;
    }

    // End of Switch: '<S29>/Switch1'

    // Product: '<S29>/Divide4'
    rtb_q_dot = rtb_r_dot / AVANTI2_ertmodel0827_ConstB.Gain8;

    // Math: '<S29>/Square'
    AVANTI2_ertmodel0827_B.Square = rtb_q_dot * rtb_q_dot;
  }

  // Product: '<S29>/Divide1' incorporates:
  //   Constant: '<S29>/Pi'
  //   Product: '<S29>/Product1'

  rtb_Divide1 = rtb_Sqrt * 3.1415926535897931 / AVANTI2_ertmodel0827_B.Switch1;

  // Product: '<S29>/Product3' incorporates:
  //   Constant: '<S29>/Jmax'
  //   Constant: '<S29>/Jmax2'
  //   Gain: '<S29>/Gain1'
  //   Product: '<S29>/Divide2'
  //   Sum: '<S29>/Sum'

  rtb_sincos_o2_e_idx_1 = (1.0 - rtb_Divide1 / 0.8) * 0.0999 * rtb_Gain1_h *
    AVANTI2_ertmodel0827_ConstB.Power1 * AVANTI2_ertmodel0827_B.Square;

  // Switch: '<S29>/Switch'
  if (rtb_sincos_o2_e_idx_1 > 0.0) {
    rtb_V_windtoBody[0] = rtb_sincos_o2_e_idx_1;
  } else {
    rtb_V_windtoBody[0] = 0.0;
  }

  // End of Switch: '<S29>/Switch'

  // SignalConversion generated from: '<S29>/Matrix Concatenate' incorporates:
  //   Constant: '<S29>/Constant1'

  rtb_V_windtoBody[1] = 0.0;

  // SignalConversion generated from: '<S29>/Matrix Concatenate' incorporates:
  //   Constant: '<S29>/Constant1'

  rtb_V_windtoBody[2] = 0.0;
  for (iU = 0; iU < 3; iU++) {
    // Sum: '<S27>/Sum' incorporates:
    //   Concatenate: '<S17>/Vector Concatenate'
    //   Concatenate: '<S29>/Matrix Concatenate'
    //   Gain: '<S26>/G=mg'
    //   Math: '<S25>/Transpose1'
    //   Math: '<S29>/Transpose'
    //   Product: '<S25>/Matrix Multiply'
    //   Product: '<S26>/Matrix Multiply'
    //   Trigonometry: '<S16>/sincos'

    rtb_VectorConcatenate_0[iU] = ((rtb_Transpose1[iU + 3] * rtb_Sum1_f[1] +
      rtb_Transpose1[iU] * rtb_Sum1_f[0]) + rtb_Transpose1[iU + 6] * rtb_Sum1_f
      [2]) + ((rtb_VectorConcatenate[iU + 6] * AVANTI2_ertmodel0827_ConstB.Gmg[2]
               + (rtb_VectorConcatenate[iU + 3] *
                  AVANTI2_ertmodel0827_ConstB.Gmg[1] + rtb_VectorConcatenate[iU]
                  * AVANTI2_ertmodel0827_ConstB.Gmg[0])) + rtb_V_windtoBody[iU]);
  }

  // Sum: '<S27>/Sum'
  rtb_Sum1_f[0] = rtb_VectorConcatenate_0[0];
  rtb_Sum1_f[1] = rtb_VectorConcatenate_0[1];
  rtb_Sum1_f[2] = rtb_VectorConcatenate_0[2];

  // Switch: '<S27>/Switch1' incorporates:
  //   Gain: '<S6>/Gain1'
  //   Integrator: '<S2>/xe,ye,ze'

  if (!(-AVANTI2_ertmodel0827_X.xeyeze_CSTATE[2] > 0.0)) {
    // Switch: '<S27>/Switch' incorporates:
    //   Gain: '<S27>/Gain1'
    //   Sum: '<S27>/Sum'

    if (rtb_VectorConcatenate_0[2] > 0.0) {
      rtb_Sum1_f[0] = rtb_VectorConcatenate_0[0];
      rtb_Sum1_f[1] = rtb_VectorConcatenate_0[1];
      rtb_Sum1_f[2] = 0.0 * rtb_VectorConcatenate_0[2];
    }

    // End of Switch: '<S27>/Switch'
  }

  // End of Switch: '<S27>/Switch1'

  // Sum: '<S2>/Sum' incorporates:
  //   Constant: '<S9>/Constant'
  //   Integrator: '<S2>/p,q,r '
  //   Integrator: '<S2>/ub,vb,wb'
  //   Product: '<S23>/i x j'
  //   Product: '<S23>/j x k'
  //   Product: '<S23>/k x i'
  //   Product: '<S24>/i x k'
  //   Product: '<S24>/j x i'
  //   Product: '<S24>/k x j'
  //   Product: '<S2>/Product'
  //   Sum: '<S10>/Sum'

  AVANTI2_ertmodel0827_B.Sum[0] = (AVANTI2_ertmodel0827_X.ubvbwb_CSTATE[1] *
    AVANTI2_ertmodel0827_X.pqr_CSTATE[2] - AVANTI2_ertmodel0827_X.pqr_CSTATE[1] *
    AVANTI2_ertmodel0827_X.ubvbwb_CSTATE[2]) + rtb_Sum1_f[0] / 6.8;
  AVANTI2_ertmodel0827_B.Sum[1] = (AVANTI2_ertmodel0827_X.pqr_CSTATE[0] *
    AVANTI2_ertmodel0827_X.ubvbwb_CSTATE[2] -
    AVANTI2_ertmodel0827_X.ubvbwb_CSTATE[0] * AVANTI2_ertmodel0827_X.pqr_CSTATE
    [2]) + rtb_Sum1_f[1] / 6.8;
  AVANTI2_ertmodel0827_B.Sum[2] = (AVANTI2_ertmodel0827_X.ubvbwb_CSTATE[0] *
    AVANTI2_ertmodel0827_X.pqr_CSTATE[1] - AVANTI2_ertmodel0827_X.pqr_CSTATE[0] *
    AVANTI2_ertmodel0827_X.ubvbwb_CSTATE[1]) + rtb_Sum1_f[2] / 6.8;
  if (rtmIsMajorTimeStep((&AVANTI2_ertmodel0827_M))) {
    // Product: '<S29>/Divide5'
    rtb_r_dot /= AVANTI2_ertmodel0827_ConstB.Gain9;

    // Math: '<S29>/Square1'
    AVANTI2_ertmodel0827_B.Square1 = rtb_r_dot * rtb_r_dot;

    // Gain: '<S25>/Gain6'
    AVANTI2_ertmodel0827_B.Gain6 = -0.3131 * rtb_UnitConversion[1];

    // Gain: '<S25>/Gain39'
    AVANTI2_ertmodel0827_B.Gain39 = -0.00518 * rtb_UnitConversion[2];

    // Gain: '<S25>/Gain15'
    AVANTI2_ertmodel0827_B.Gain15 = -2.1075 * rtb_UnitConversion[0];
  }

  // Switch: '<S25>/Switch16' incorporates:
  //   Constant: '<S25>/Constant10'
  //   Constant: '<S25>/Constant19'
  //   Constant: '<S25>/Constant20'
  //   Gain: '<S25>/Gain37'
  //   Gain: '<S25>/Gain4'
  //   Gain: '<S25>/Gain47'
  //   Gain: '<S25>/Gain48'
  //   Gain: '<S25>/Gain49'
  //   Gain: '<S25>/Gain5'
  //   Integrator: '<S2>/p,q,r '
  //   Product: '<S25>/Divide7'
  //   Product: '<S25>/Divide8'
  //   Product: '<S25>/Product2'
  //   Sum: '<S25>/Sum11'
  //   Sum: '<S25>/Sum12'
  //   Sum: '<S25>/Sum13'
  //   Switch: '<S25>/Switch8'

  if (rtb_Switch1 > 0) {
    rtb_sincos_o2_i[0] = ((((2.16 * AVANTI2_ertmodel0827_X.pqr_CSTATE[0] /
      (rtb_Sqrt + 1.0E-6) * -0.7261 + -0.1797 * Switch) + 2.16 *
      AVANTI2_ertmodel0827_X.pqr_CSTATE[2] / (rtb_Sqrt + 1.0E-6) * 0.186) +
      AVANTI2_ertmodel0827_B.Gain6) + AVANTI2_ertmodel0827_B.Gain39) * 0.957 *
      rtb_sincos_o2_idx_2;
  } else if (rtb_sincos_o2_idx_1 > 0.0) {
    // Switch: '<S25>/Switch8' incorporates:
    //   Constant: '<S25>/Constant19'
    //   Constant: '<S25>/Constant20'
    //   Gain: '<S25>/Gain37'
    //   Gain: '<S25>/Gain4'
    //   Gain: '<S25>/Gain47'
    //   Gain: '<S25>/Gain48'
    //   Gain: '<S25>/Gain49'
    //   Gain: '<S25>/Gain5'
    //   Integrator: '<S2>/p,q,r '
    //   Product: '<S25>/Divide7'
    //   Product: '<S25>/Divide8'
    //   Product: '<S25>/Product2'
    //   Sum: '<S25>/Sum11'
    //   Sum: '<S25>/Sum12'
    //   Sum: '<S25>/Sum13'

    rtb_sincos_o2_i[0] = ((((2.16 * AVANTI2_ertmodel0827_X.pqr_CSTATE[0] /
      (rtb_Sqrt + 1.0E-6) * -0.7261 + -0.1797 * Switch) + 2.16 *
      AVANTI2_ertmodel0827_X.pqr_CSTATE[2] / (rtb_Sqrt + 1.0E-6) * 0.186) +
      AVANTI2_ertmodel0827_B.Gain6) + AVANTI2_ertmodel0827_B.Gain39) * 0.957 *
      rtb_sincos_o2_idx_2;
  } else {
    rtb_sincos_o2_i[0] = 0.0;
  }

  // End of Switch: '<S25>/Switch16'
  if (rtmIsMajorTimeStep((&AVANTI2_ertmodel0827_M))) {
    // Gain: '<S25>/Gain19'
    AVANTI2_ertmodel0827_B.Gain19 = -0.0124 * rtb_UnitConversion[1];

    // Gain: '<S25>/Gain20'
    AVANTI2_ertmodel0827_B.Gain20 = 0.04098 * rtb_UnitConversion[2];
  }

  // Switch: '<S25>/Switch11' incorporates:
  //   Constant: '<S25>/Constant11'
  //   Constant: '<S25>/Constant16'
  //   Constant: '<S25>/Constant24'
  //   Constant: '<S25>/Constant5'
  //   Constant: '<S25>/Constant8'
  //   Gain: '<S25>/Gain11'
  //   Gain: '<S25>/Gain12'
  //   Gain: '<S25>/Gain13'
  //   Gain: '<S25>/Gain14'
  //   Gain: '<S25>/Gain18'
  //   Gain: '<S25>/Gain2'
  //   Gain: '<S25>/Gain21'
  //   Gain: '<S25>/Gain22'
  //   Gain: '<S25>/Gain3'
  //   Gain: '<S25>/Gain38'
  //   Integrator: '<S2>/p,q,r '
  //   Product: '<S25>/Divide2'
  //   Product: '<S25>/Divide3'
  //   Product: '<S25>/Divide6'
  //   Product: '<S25>/Product4'
  //   Product: '<S25>/Product7'
  //   Sum: '<S25>/Sum10'
  //   Sum: '<S25>/Sum17'
  //   Sum: '<S25>/Sum3'
  //   Sum: '<S25>/Sum5'
  //   Sum: '<S25>/Sum9'
  //   Switch: '<S25>/Switch15'
  //   Switch: '<S25>/Switch6'

  if (rtb_Switch1 > 0) {
    rtb_sincos_o2_i[1] = ((0.11076388888888888 *
      AVANTI2_ertmodel0827_X.pqr_CSTATE[1] / (rtb_Sqrt + 1.0E-6) * -23.5033 +
      (-1.6427 * rtb_sincos_o2_idx_0 + -0.07792)) +
                          AVANTI2_ertmodel0827_B.Gain15) * 0.957 *
      rtb_sincos_o2_idx_2;
    rtb_sincos_o2_i[2] = ((((2.16 * AVANTI2_ertmodel0827_X.pqr_CSTATE[0] /
      (rtb_Sqrt + 1.0E-6) * -0.06726 + 0.0267 * Switch) + 2.16 *
      AVANTI2_ertmodel0827_X.pqr_CSTATE[2] / (rtb_Sqrt + 1.0E-6) * 0.03438) +
      AVANTI2_ertmodel0827_B.Gain19) + AVANTI2_ertmodel0827_B.Gain20) * 0.957 *
      rtb_sincos_o2_idx_2;
  } else {
    if (rtb_sincos_o2_idx_1 > 0.0) {
      // Switch: '<S25>/Switch6' incorporates:
      //   Constant: '<S25>/Constant24'
      //   Constant: '<S25>/Constant8'
      //   Gain: '<S25>/Gain11'
      //   Gain: '<S25>/Gain12'
      //   Gain: '<S25>/Gain13'
      //   Gain: '<S25>/Gain14'
      //   Integrator: '<S2>/p,q,r '
      //   Product: '<S25>/Divide3'
      //   Product: '<S25>/Product4'
      //   Sum: '<S25>/Sum17'
      //   Sum: '<S25>/Sum3'

      rtb_sincos_o2_i[1] = ((0.11076388888888888 *
        AVANTI2_ertmodel0827_X.pqr_CSTATE[1] / (rtb_Sqrt + 1.0E-6) * -23.5033 +
        (-1.6427 * rtb_sincos_o2_idx_0 + -0.07792)) +
                            AVANTI2_ertmodel0827_B.Gain15) * 0.957 *
        rtb_sincos_o2_idx_2;
    } else {
      rtb_sincos_o2_i[1] = 0.0;
    }

    // Switch: '<S25>/Switch9' incorporates:
    //   Constant: '<S25>/Constant11'
    //   Constant: '<S25>/Constant12'
    //   Constant: '<S25>/Constant16'
    //   Constant: '<S25>/Constant5'
    //   Gain: '<S25>/Gain18'
    //   Gain: '<S25>/Gain2'
    //   Gain: '<S25>/Gain21'
    //   Gain: '<S25>/Gain22'
    //   Gain: '<S25>/Gain3'
    //   Gain: '<S25>/Gain38'
    //   Integrator: '<S2>/p,q,r '
    //   Product: '<S25>/Divide2'
    //   Product: '<S25>/Divide6'
    //   Product: '<S25>/Product7'
    //   Sum: '<S25>/Sum10'
    //   Sum: '<S25>/Sum5'
    //   Sum: '<S25>/Sum9'
    //   Switch: '<S25>/Switch6'

    if (rtb_sincos_o2_idx_1 > 0.0) {
      rtb_sincos_o2_i[2] = ((((2.16 * AVANTI2_ertmodel0827_X.pqr_CSTATE[0] /
        (rtb_Sqrt + 1.0E-6) * -0.06726 + 0.0267 * Switch) + 2.16 *
        AVANTI2_ertmodel0827_X.pqr_CSTATE[2] / (rtb_Sqrt + 1.0E-6) * 0.03438) +
        AVANTI2_ertmodel0827_B.Gain19) + AVANTI2_ertmodel0827_B.Gain20) * 0.957 *
        rtb_sincos_o2_idx_2;
    } else {
      rtb_sincos_o2_i[2] = 0.0;
    }

    // End of Switch: '<S25>/Switch9'
  }

  // End of Switch: '<S25>/Switch11'

  // Sum: '<S27>/Sum1' incorporates:
  //   Constant: '<S29>/Jmax1'
  //   Constant: '<S29>/Jmax3'
  //   Gain: '<S29>/Gain2'
  //   Gain: '<S29>/Gain3'
  //   Product: '<S29>/Divide3'
  //   Product: '<S29>/Product4'
  //   Sum: '<S29>/Sum1'
  //   Trigonometry: '<S16>/sincos'

  rtb_sincos_o2_idx_1 = (1.0 - rtb_Divide1 / 0.75) * 0.0 * rtb_Gain1_h *
    AVANTI2_ertmodel0827_ConstB.Power2 * AVANTI2_ertmodel0827_B.Square1 * -0.0 +
    rtb_sincos_o2_i[0];
  rtb_Sum1_f[0] = rtb_sincos_o2_idx_1;
  rtb_Sum1_f[1] = rtb_sincos_o2_i[1];
  rtb_Sum1_f[2] = rtb_sincos_o2_i[2];

  // Switch: '<S27>/Switch3' incorporates:
  //   Gain: '<S6>/Gain1'
  //   Integrator: '<S2>/xe,ye,ze'

  if (!(-AVANTI2_ertmodel0827_X.xeyeze_CSTATE[2] > 0.0)) {
    // Switch: '<S27>/Switch2' incorporates:
    //   Constant: '<S29>/Jmax1'
    //   Constant: '<S29>/Jmax3'
    //   Gain: '<S27>/Gain2'
    //   Gain: '<S27>/Gain3'
    //   Gain: '<S29>/Gain2'
    //   Gain: '<S29>/Gain3'
    //   Product: '<S29>/Divide3'
    //   Product: '<S29>/Product4'
    //   Sum: '<S27>/Sum1'
    //   Sum: '<S29>/Sum1'
    //   Trigonometry: '<S16>/sincos'

    if (rtb_sincos_o2_i[1] > 0.0) {
      rtb_Sum1_f[0] = ((1.0 - rtb_Divide1 / 0.75) * 0.0 * rtb_Gain1_h *
                       AVANTI2_ertmodel0827_ConstB.Power2 *
                       AVANTI2_ertmodel0827_B.Square1 * -0.0 + rtb_sincos_o2_i[0])
        * 0.0;
      rtb_Sum1_f[1] = rtb_sincos_o2_i[1];
      rtb_Sum1_f[2] = 0.0 * rtb_sincos_o2_i[2];
    } else {
      rtb_Sum1_f[0] = rtb_sincos_o2_idx_1 * 0.0;
      rtb_Sum1_f[1] = 0.0 * rtb_sincos_o2_i[1];
      rtb_Sum1_f[2] = 0.0 * rtb_sincos_o2_i[2];
    }

    // End of Switch: '<S27>/Switch2'
  }

  // End of Switch: '<S27>/Switch3'
  for (iU = 0; iU < 3; iU++) {
    // Product: '<S20>/Product' incorporates:
    //   Integrator: '<S2>/p,q,r '
    //   Selector: '<S8>/Selector1'
    //   Trigonometry: '<S16>/sincos'

    rtb_sincos_o2_i[iU] = AVANTI2_ertmodel0827_ConstB.Selector1[iU + 6] *
      AVANTI2_ertmodel0827_X.pqr_CSTATE[2] +
      (AVANTI2_ertmodel0827_ConstB.Selector1[iU + 3] *
       AVANTI2_ertmodel0827_X.pqr_CSTATE[1] +
       AVANTI2_ertmodel0827_ConstB.Selector1[iU] *
       AVANTI2_ertmodel0827_X.pqr_CSTATE[0]);

    // Product: '<S19>/Product' incorporates:
    //   Integrator: '<S2>/p,q,r '
    //   Selector: '<S8>/Selector'
    //   Sum: '<S18>/Sum'

    rtb_UnitConversion[iU] = AVANTI2_ertmodel0827_ConstB.Selector[iU + 6] *
      AVANTI2_ertmodel0827_X.pqr_CSTATE[2] +
      (AVANTI2_ertmodel0827_ConstB.Selector[iU + 3] *
       AVANTI2_ertmodel0827_X.pqr_CSTATE[1] +
       AVANTI2_ertmodel0827_ConstB.Selector[iU] *
       AVANTI2_ertmodel0827_X.pqr_CSTATE[0]);
  }

  // Sum: '<S8>/Sum2' incorporates:
  //   Integrator: '<S2>/p,q,r '
  //   Product: '<S21>/i x j'
  //   Product: '<S21>/j x k'
  //   Product: '<S21>/k x i'
  //   Product: '<S22>/i x k'
  //   Product: '<S22>/j x i'
  //   Product: '<S22>/k x j'
  //   Sum: '<S18>/Sum'

  rtb_Sum1_f[0] = (rtb_Sum1_f[0] - rtb_sincos_o2_i[0]) -
    (AVANTI2_ertmodel0827_X.pqr_CSTATE[1] * rtb_UnitConversion[2] -
     rtb_UnitConversion[1] * AVANTI2_ertmodel0827_X.pqr_CSTATE[2]);
  rtb_Sum1_f[1] = (rtb_Sum1_f[1] - rtb_sincos_o2_i[1]) - (rtb_UnitConversion[0] *
    AVANTI2_ertmodel0827_X.pqr_CSTATE[2] - AVANTI2_ertmodel0827_X.pqr_CSTATE[0] *
    rtb_UnitConversion[2]);
  rtb_Sum1_f[2] = (rtb_Sum1_f[2] - rtb_sincos_o2_i[2]) -
    (AVANTI2_ertmodel0827_X.pqr_CSTATE[0] * rtb_UnitConversion[1] -
     rtb_UnitConversion[0] * AVANTI2_ertmodel0827_X.pqr_CSTATE[1]);

  // Product: '<S8>/Product2' incorporates:
  //   Selector: '<S8>/Selector2'
  //   Trigonometry: '<S16>/sincos'

  rt_mrdivide_U1d1x3_U2d_9vOrDY9Z(rtb_Sum1_f,
    AVANTI2_ertmodel0827_ConstB.Selector2, AVANTI2_ertmodel0827_B.Product2);
  for (iU = 0; iU < 3; iU++) {
    // Product: '<S14>/Product' incorporates:
    //   Concatenate: '<S17>/Vector Concatenate'
    //   Integrator: '<S2>/ub,vb,wb'
    //   Math: '<S2>/Transpose'

    AVANTI2_ertmodel0827_B.Product[iU] = 0.0;
    AVANTI2_ertmodel0827_B.Product[iU] += rtb_VectorConcatenate[3 * iU] *
      AVANTI2_ertmodel0827_X.ubvbwb_CSTATE[0];
    AVANTI2_ertmodel0827_B.Product[iU] += rtb_VectorConcatenate[3 * iU + 1] *
      AVANTI2_ertmodel0827_X.ubvbwb_CSTATE[1];
    AVANTI2_ertmodel0827_B.Product[iU] += rtb_VectorConcatenate[3 * iU + 2] *
      AVANTI2_ertmodel0827_X.ubvbwb_CSTATE[2];

    // Trigonometry: '<S16>/sincos' incorporates:
    //   Integrator: '<S7>/phi theta psi'

    rtb_sincos_o2_idx_1 = std::sin(AVANTI2_ertmodel0827_X.phithetapsi_CSTATE[iU]);
    rtb_Sqrt = std::cos(AVANTI2_ertmodel0827_X.phithetapsi_CSTATE[iU]);

    // Fcn: '<S16>/phidot' incorporates:
    //   Integrator: '<S2>/p,q,r '

    rtb_Transpose1[iU] = AVANTI2_ertmodel0827_X.pqr_CSTATE[iU];
    rtb_Transpose1[iU + 3] = rtb_sincos_o2_idx_1;
    rtb_Transpose1[iU + 6] = rtb_Sqrt;
    rtb_Transpose1_0[iU] = AVANTI2_ertmodel0827_X.pqr_CSTATE[iU];
    rtb_Transpose1_0[iU + 3] = rtb_sincos_o2_idx_1;
    rtb_Transpose1_0[iU + 6] = rtb_Sqrt;
    tmp[iU] = AVANTI2_ertmodel0827_X.pqr_CSTATE[iU];
    tmp[iU + 3] = rtb_sincos_o2_idx_1;
    tmp[iU + 6] = rtb_Sqrt;
    tmp_0[iU] = AVANTI2_ertmodel0827_X.pqr_CSTATE[iU];
    tmp_0[iU + 3] = rtb_sincos_o2_idx_1;
    tmp_0[iU + 6] = rtb_Sqrt;
    tmp_1[iU] = AVANTI2_ertmodel0827_X.pqr_CSTATE[iU];
    tmp_1[iU + 3] = rtb_sincos_o2_idx_1;
    tmp_1[iU + 6] = rtb_Sqrt;
    tmp_2[iU] = AVANTI2_ertmodel0827_X.pqr_CSTATE[iU];
    tmp_2[iU + 3] = rtb_sincos_o2_idx_1;
    tmp_2[iU + 6] = rtb_Sqrt;
    tmp_3[iU] = AVANTI2_ertmodel0827_X.pqr_CSTATE[iU];
    tmp_3[iU + 3] = rtb_sincos_o2_idx_1;
    tmp_3[iU + 6] = rtb_Sqrt;

    // Trigonometry: '<S16>/sincos'
    rtb_Sum1_f[iU] = rtb_sincos_o2_idx_1;
    rtb_sincos_o2_i[iU] = rtb_Sqrt;
  }

  // SignalConversion generated from: '<S7>/phi theta psi' incorporates:
  //   Fcn: '<S16>/phidot'
  //   Fcn: '<S16>/psidot'
  //   Fcn: '<S16>/thetadot'
  //   Integrator: '<S2>/p,q,r '

  AVANTI2_ertmodel0827_B.TmpSignalConversionAtphithetaps[0] = (rtb_Transpose1_0
    [1] * tmp[3] + tmp_0[2] * tmp_1[6]) * (tmp_2[4] / tmp_3[7]) +
    rtb_Transpose1[0];
  AVANTI2_ertmodel0827_B.TmpSignalConversionAtphithetaps[1] = rtb_sincos_o2_i[0]
    * AVANTI2_ertmodel0827_X.pqr_CSTATE[1] - rtb_Sum1_f[0] *
    AVANTI2_ertmodel0827_X.pqr_CSTATE[2];
  AVANTI2_ertmodel0827_B.TmpSignalConversionAtphithetaps[2] = (rtb_Sum1_f[0] *
    AVANTI2_ertmodel0827_X.pqr_CSTATE[1] + rtb_sincos_o2_i[0] *
    AVANTI2_ertmodel0827_X.pqr_CSTATE[2]) / rtb_sincos_o2_i[1];
  if (rtmIsMajorTimeStep((&AVANTI2_ertmodel0827_M))) {
    // ZeroOrderHold generated from: '<Root>/CodegenOutputHold_NED'
    rtb_q_dot = AVANTI2_ertmodel0827_B.Product[0];

    // ZeroOrderHold generated from: '<Root>/CodegenOutputHold_NED' incorporates:
    //   Integrator: '<S2>/xe,ye,ze'

    rtb_r_dot = AVANTI2_ertmodel0827_X.xeyeze_CSTATE[0];

    // ZeroOrderHold generated from: '<Root>/CodegenOutputHold_NED' incorporates:
    //   Integrator: '<S2>/xe,ye,ze'

    rtb_r = AVANTI2_ertmodel0827_X.xeyeze_CSTATE[1];

    // ZeroOrderHold generated from: '<Root>/CodegenOutputHold_NED' incorporates:
    //   Gain: '<S1>/Gain'
    //   Integrator: '<S2>/xe,ye,ze'

    rtb_q = -AVANTI2_ertmodel0827_X.xeyeze_CSTATE[2];

    // ZeroOrderHold generated from: '<Root>/CodegenOutputHold_NED'
    rtb_p = AVANTI2_ertmodel0827_B.Product[1];

    // ZeroOrderHold generated from: '<Root>/CodegenOutputHold_NED' incorporates:
    //   Gain: '<S1>/Gain1'

    rtb_w = -AVANTI2_ertmodel0827_B.Product[2];

    // ZeroOrderHold generated from: '<Root>/CodegenOutputHold_NED' incorporates:
    //   Integrator: '<S7>/phi theta psi'
    //   UnitConversion: '<S3>/Unit Conversion'

    rtb_v = 57.295779513082323 * AVANTI2_ertmodel0827_X.phithetapsi_CSTATE[0];
  }

  // Unit Conversion - from: rad to: deg
  // Expression: output = (57.2958*input) + (0)
  // Unit Conversion - from: rad to: deg
  // Expression: output = (57.2958*input) + (0)
  if (rtmIsMajorTimeStep((&AVANTI2_ertmodel0827_M))) {
    // Outport: '<Root>/NED' incorporates:
    //   Integrator: '<S7>/phi theta psi'
    //   UnitConversion: '<S4>/Unit Conversion'
    //   UnitConversion: '<S5>/Unit Conversion'

    AVANTI2_ertmodel0827_Y.NED[0] = rtb_r_dot;
    AVANTI2_ertmodel0827_Y.NED[1] = rtb_r;
    AVANTI2_ertmodel0827_Y.NED[2] = rtb_q;
    AVANTI2_ertmodel0827_Y.NED[3] = rtb_q_dot;
    AVANTI2_ertmodel0827_Y.NED[4] = rtb_p;
    AVANTI2_ertmodel0827_Y.NED[5] = rtb_w;
    AVANTI2_ertmodel0827_Y.NED[6] = rtb_v;
    AVANTI2_ertmodel0827_Y.NED[7] = 57.295779513082323 *
      AVANTI2_ertmodel0827_X.phithetapsi_CSTATE[1];
    AVANTI2_ertmodel0827_Y.NED[8] = 57.295779513082323 *
      AVANTI2_ertmodel0827_X.phithetapsi_CSTATE[2];

    // Outport: '<Root>/BODY' incorporates:
    //   Integrator: '<S2>/p,q,r '
    //   Integrator: '<S2>/ub,vb,wb'
    //   ZeroOrderHold generated from: '<Root>/CodegenOutputHold_BODY'

    AVANTI2_ertmodel0827_Y.BODY[0] = AVANTI2_ertmodel0827_X.ubvbwb_CSTATE[0];
    AVANTI2_ertmodel0827_Y.BODY[1] = AVANTI2_ertmodel0827_X.ubvbwb_CSTATE[1];
    AVANTI2_ertmodel0827_Y.BODY[2] = AVANTI2_ertmodel0827_X.ubvbwb_CSTATE[2];
    AVANTI2_ertmodel0827_Y.BODY[3] = AVANTI2_ertmodel0827_X.pqr_CSTATE[0];
    AVANTI2_ertmodel0827_Y.BODY[4] = AVANTI2_ertmodel0827_X.pqr_CSTATE[1];
    AVANTI2_ertmodel0827_Y.BODY[5] = AVANTI2_ertmodel0827_X.pqr_CSTATE[2];
    AVANTI2_ertmodel0827_Y.BODY[6] = AVANTI2_ertmodel0827_B.Product2[0];
    AVANTI2_ertmodel0827_Y.BODY[7] = AVANTI2_ertmodel0827_B.Product2[1];
    AVANTI2_ertmodel0827_Y.BODY[8] = AVANTI2_ertmodel0827_B.Product2[2];
    AVANTI2_ertmodel0827_Y.BODY[9] = AVANTI2_ertmodel0827_B.Sum[0];
    AVANTI2_ertmodel0827_Y.BODY[10] = AVANTI2_ertmodel0827_B.Sum[1];
    AVANTI2_ertmodel0827_Y.BODY[11] = AVANTI2_ertmodel0827_B.Sum[2];

    // Outport: '<Root>/beta' incorporates:
    //   ZeroOrderHold: '<Root>/CodegenOutputHold_beta'

    AVANTI2_ertmodel0827_Y.beta = Switch;

    // Outport: '<Root>/alpha' incorporates:
    //   ZeroOrderHold: '<Root>/CodegenOutputHold_alpha'

    AVANTI2_ertmodel0827_Y.alpha = rtb_sincos_o2_idx_0;
  }

  // Unit Conversion - from: rad to: deg
  // Expression: output = (57.2958*input) + (0)
  if (rtmIsMajorTimeStep((&AVANTI2_ertmodel0827_M))) {
    rt_ertODEUpdateContinuousStates(&(&AVANTI2_ertmodel0827_M)->solverInfo);

    // Update absolute time for base rate
    // The "clockTick0" counts the number of times the code of this task has
    //  been executed. The absolute time is the multiplication of "clockTick0"
    //  and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
    //  overflow during the application lifespan selected.

    ++(&AVANTI2_ertmodel0827_M)->Timing.clockTick0;
    (&AVANTI2_ertmodel0827_M)->Timing.t[0] = rtsiGetSolverStopTime
      (&(&AVANTI2_ertmodel0827_M)->solverInfo);

    {
      // Update absolute timer for sample time: [0.02s, 0.0s]
      // The "clockTick1" counts the number of times the code of this task has
      //  been executed. The resolution of this integer timer is 0.02, which is the step size
      //  of the task. Size of "clockTick1" ensures timer will not overflow during the
      //  application lifespan selected.

      (&AVANTI2_ertmodel0827_M)->Timing.clockTick1++;
    }
  }                                    // end MajorTimeStep
}

// Derivatives for root system: '<Root>'
void AVANTI2_ertmodel0827ModelClass::AVANTI2_ertmodel0827_derivatives()
{
  AVANTI2_ertmodel0827ModelClass::XDot_AVANTI2_ertmodel0827_T *_rtXdot;
  _rtXdot = ((XDot_AVANTI2_ertmodel0827_T *) (&AVANTI2_ertmodel0827_M)->derivs);

  // Derivatives for Integrator: '<S7>/phi theta psi'
  _rtXdot->phithetapsi_CSTATE[0] =
    AVANTI2_ertmodel0827_B.TmpSignalConversionAtphithetaps[0];

  // Derivatives for Integrator: '<S2>/ub,vb,wb'
  _rtXdot->ubvbwb_CSTATE[0] = AVANTI2_ertmodel0827_B.Sum[0];

  // Derivatives for Integrator: '<S2>/xe,ye,ze'
  _rtXdot->xeyeze_CSTATE[0] = AVANTI2_ertmodel0827_B.Product[0];

  // Derivatives for Integrator: '<S2>/p,q,r '
  _rtXdot->pqr_CSTATE[0] = AVANTI2_ertmodel0827_B.Product2[0];

  // Derivatives for Integrator: '<S7>/phi theta psi'
  _rtXdot->phithetapsi_CSTATE[1] =
    AVANTI2_ertmodel0827_B.TmpSignalConversionAtphithetaps[1];

  // Derivatives for Integrator: '<S2>/ub,vb,wb'
  _rtXdot->ubvbwb_CSTATE[1] = AVANTI2_ertmodel0827_B.Sum[1];

  // Derivatives for Integrator: '<S2>/xe,ye,ze'
  _rtXdot->xeyeze_CSTATE[1] = AVANTI2_ertmodel0827_B.Product[1];

  // Derivatives for Integrator: '<S2>/p,q,r '
  _rtXdot->pqr_CSTATE[1] = AVANTI2_ertmodel0827_B.Product2[1];

  // Derivatives for Integrator: '<S7>/phi theta psi'
  _rtXdot->phithetapsi_CSTATE[2] =
    AVANTI2_ertmodel0827_B.TmpSignalConversionAtphithetaps[2];

  // Derivatives for Integrator: '<S2>/ub,vb,wb'
  _rtXdot->ubvbwb_CSTATE[2] = AVANTI2_ertmodel0827_B.Sum[2];

  // Derivatives for Integrator: '<S2>/xe,ye,ze'
  _rtXdot->xeyeze_CSTATE[2] = AVANTI2_ertmodel0827_B.Product[2];

  // Derivatives for Integrator: '<S2>/p,q,r '
  _rtXdot->pqr_CSTATE[2] = AVANTI2_ertmodel0827_B.Product2[2];
}

// Model initialize function
void AVANTI2_ertmodel0827ModelClass::initialize()
{
  // Registration code

  // initialize non-finites
  rt_InitInfAndNaN(sizeof(real_T));

  {
    // Setup solver object
    rtsiSetSimTimeStepPtr(&(&AVANTI2_ertmodel0827_M)->solverInfo,
                          &(&AVANTI2_ertmodel0827_M)->Timing.simTimeStep);
    rtsiSetTPtr(&(&AVANTI2_ertmodel0827_M)->solverInfo, &rtmGetTPtr
                ((&AVANTI2_ertmodel0827_M)));
    rtsiSetStepSizePtr(&(&AVANTI2_ertmodel0827_M)->solverInfo,
                       &(&AVANTI2_ertmodel0827_M)->Timing.stepSize0);
    rtsiSetdXPtr(&(&AVANTI2_ertmodel0827_M)->solverInfo,
                 &(&AVANTI2_ertmodel0827_M)->derivs);
    rtsiSetContStatesPtr(&(&AVANTI2_ertmodel0827_M)->solverInfo, (real_T **)
                         &(&AVANTI2_ertmodel0827_M)->contStates);
    rtsiSetNumContStatesPtr(&(&AVANTI2_ertmodel0827_M)->solverInfo,
      &(&AVANTI2_ertmodel0827_M)->Sizes.numContStates);
    rtsiSetNumPeriodicContStatesPtr(&(&AVANTI2_ertmodel0827_M)->solverInfo,
      &(&AVANTI2_ertmodel0827_M)->Sizes.numPeriodicContStates);
    rtsiSetPeriodicContStateIndicesPtr(&(&AVANTI2_ertmodel0827_M)->solverInfo,
      &(&AVANTI2_ertmodel0827_M)->periodicContStateIndices);
    rtsiSetPeriodicContStateRangesPtr(&(&AVANTI2_ertmodel0827_M)->solverInfo, &(
      &AVANTI2_ertmodel0827_M)->periodicContStateRanges);
    rtsiSetErrorStatusPtr(&(&AVANTI2_ertmodel0827_M)->solverInfo,
                          (&rtmGetErrorStatus((&AVANTI2_ertmodel0827_M))));
    rtsiSetRTModelPtr(&(&AVANTI2_ertmodel0827_M)->solverInfo,
                      (&AVANTI2_ertmodel0827_M));
  }

  rtsiSetSimTimeStep(&(&AVANTI2_ertmodel0827_M)->solverInfo, MAJOR_TIME_STEP);
  (&AVANTI2_ertmodel0827_M)->intgData.y = (&AVANTI2_ertmodel0827_M)->odeY;
  (&AVANTI2_ertmodel0827_M)->intgData.f[0] = (&AVANTI2_ertmodel0827_M)->odeF[0];
  (&AVANTI2_ertmodel0827_M)->intgData.f[1] = (&AVANTI2_ertmodel0827_M)->odeF[1];
  (&AVANTI2_ertmodel0827_M)->intgData.f[2] = (&AVANTI2_ertmodel0827_M)->odeF[2];
  (&AVANTI2_ertmodel0827_M)->intgData.f[3] = (&AVANTI2_ertmodel0827_M)->odeF[3];
  (&AVANTI2_ertmodel0827_M)->contStates = ((X_AVANTI2_ertmodel0827_T *)
    &AVANTI2_ertmodel0827_X);
  (&AVANTI2_ertmodel0827_M)->periodicContStateIndices = ((int_T*)
    AVANTI2_ertmodel08_PeriodicIndX);
  (&AVANTI2_ertmodel0827_M)->periodicContStateRanges = ((real_T*)
    AVANTI2_ertmodel08_PeriodicRngX);
  rtsiSetSolverData(&(&AVANTI2_ertmodel0827_M)->solverInfo, static_cast<void *>(
    &(&AVANTI2_ertmodel0827_M)->intgData));
  rtsiSetSolverName(&(&AVANTI2_ertmodel0827_M)->solverInfo,"ode4");
  rtmSetTPtr((&AVANTI2_ertmodel0827_M), &(&AVANTI2_ertmodel0827_M)
             ->Timing.tArray[0]);
  (&AVANTI2_ertmodel0827_M)->Timing.stepSize0 = 0.02;

  // InitializeConditions for Integrator: '<S7>/phi theta psi'
  AVANTI2_ertmodel0827_X.phithetapsi_CSTATE[0] = 0.0;

  // InitializeConditions for Integrator: '<S2>/ub,vb,wb'
  AVANTI2_ertmodel0827_X.ubvbwb_CSTATE[0] = 14.1;

  // InitializeConditions for Integrator: '<S2>/xe,ye,ze'
  AVANTI2_ertmodel0827_X.xeyeze_CSTATE[0] = 0.0;

  // InitializeConditions for Integrator: '<S2>/p,q,r '
  AVANTI2_ertmodel0827_X.pqr_CSTATE[0] = 0.0;

  // InitializeConditions for Integrator: '<S7>/phi theta psi'
  AVANTI2_ertmodel0827_X.phithetapsi_CSTATE[1] = 0.00178;

  // InitializeConditions for Integrator: '<S2>/ub,vb,wb'
  AVANTI2_ertmodel0827_X.ubvbwb_CSTATE[1] = 0.0;

  // InitializeConditions for Integrator: '<S2>/xe,ye,ze'
  AVANTI2_ertmodel0827_X.xeyeze_CSTATE[1] = 0.0;

  // InitializeConditions for Integrator: '<S2>/p,q,r '
  AVANTI2_ertmodel0827_X.pqr_CSTATE[1] = 0.0;

  // InitializeConditions for Integrator: '<S7>/phi theta psi'
  AVANTI2_ertmodel0827_X.phithetapsi_CSTATE[2] = 0.0;

  // InitializeConditions for Integrator: '<S2>/ub,vb,wb'
  AVANTI2_ertmodel0827_X.ubvbwb_CSTATE[2] = 0.0251;

  // InitializeConditions for Integrator: '<S2>/xe,ye,ze'
  AVANTI2_ertmodel0827_X.xeyeze_CSTATE[2] = -1000.0;

  // InitializeConditions for Integrator: '<S2>/p,q,r '
  AVANTI2_ertmodel0827_X.pqr_CSTATE[2] = 0.0;

  // InitializeConditions for root-level periodic continuous states
  {
    int_T rootPeriodicContStateIndices[3] = { 0, 1, 2 };

    real_T rootPeriodicContStateRanges[6] = { -3.1415926535897931,
      3.1415926535897931, -3.1415926535897931, 3.1415926535897931,
      -3.1415926535897931, 3.1415926535897931 };

    (void) std::memcpy((void*)AVANTI2_ertmodel08_PeriodicIndX,
                       rootPeriodicContStateIndices,
                       3*sizeof(int_T));
    (void) std::memcpy((void*)AVANTI2_ertmodel08_PeriodicRngX,
                       rootPeriodicContStateRanges,
                       6*sizeof(real_T));
  }
}

// Model terminate function
void AVANTI2_ertmodel0827ModelClass::terminate()
{
  // (no terminate code required)
}

// Constructor
AVANTI2_ertmodel0827ModelClass::AVANTI2_ertmodel0827ModelClass() :
  AVANTI2_ertmodel0827_U(),
  AVANTI2_ertmodel0827_Y(),
  AVANTI2_ertmodel0827_B(),
  AVANTI2_ertmodel0827_DW(),
  AVANTI2_ertmodel0827_X(),
  AVANTI2_ertmodel08_PeriodicIndX(),
  AVANTI2_ertmodel08_PeriodicRngX(),
  AVANTI2_ertmodel0827_M()
{
  // Currently there is no constructor body generated.
}

// Destructor
AVANTI2_ertmodel0827ModelClass::~AVANTI2_ertmodel0827ModelClass()
{
  // Currently there is no destructor body generated.
}

// Real-Time Model get method
AVANTI2_ertmodel0827ModelClass::RT_MODEL_AVANTI2_ertmodel0827_T
  * AVANTI2_ertmodel0827ModelClass::getRTM()
{
  return (&AVANTI2_ertmodel0827_M);
}

//
// File trailer for generated code.
//
// [EOF]
//
