//
// File: PX4Controller_ertmodel0827.cpp
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
#include "PX4Controller_ertmodel0827.h"
#include "PX4Controller_ertmodel0827_private.h"

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
void PX4Controller_ertmodel0827ModelClass::rt_ertODEUpdateContinuousStates
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
  PX4Controller_ertmodel0827_derivatives();

  // f1 = f(t + (h/2), y + (h/2)*f0)
  temp = 0.5 * h;
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (temp*f0[i]);
  }

  rtsiSetT(si, t + temp);
  rtsiSetdX(si, f1);
  this->step();
  PX4Controller_ertmodel0827_derivatives();

  // f2 = f(t + (h/2), y + (h/2)*f1)
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (temp*f1[i]);
  }

  rtsiSetdX(si, f2);
  this->step();
  PX4Controller_ertmodel0827_derivatives();

  // f3 = f(t + h, y + h*f2)
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (h*f2[i]);
  }

  rtsiSetT(si, tnew);
  rtsiSetdX(si, f3);
  this->step();
  PX4Controller_ertmodel0827_derivatives();

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

void rt_invd3x3_snf(const real_T u[9], real_T y[9])
{
  real_T x[9];
  real_T absx11;
  real_T absx21;
  real_T absx31;
  int32_T itmp;
  int32_T p1;
  int32_T p2;
  int32_T p3;
  std::memcpy(&x[0], &u[0], 9U * sizeof(real_T));
  p1 = 1;
  p2 = 3;
  p3 = 6;
  absx11 = std::abs(u[0]);
  absx21 = std::abs(u[1]);
  absx31 = std::abs(u[2]);
  if ((absx21 > absx11) && (absx21 > absx31)) {
    p1 = 4;
    p2 = 0;
    x[0] = u[1];
    x[1] = u[0];
    x[3] = u[4];
    x[4] = u[3];
    x[6] = u[7];
    x[7] = u[6];
  } else if (absx31 > absx11) {
    p1 = 7;
    p3 = 0;
    x[2] = x[0];
    x[0] = u[2];
    x[5] = x[3];
    x[3] = u[5];
    x[8] = x[6];
    x[6] = u[8];
  }

  absx11 = x[1] / x[0];
  x[1] = absx11;
  absx21 = x[2] / x[0];
  x[2] = absx21;
  x[4] -= absx11 * x[3];
  x[5] -= absx21 * x[3];
  x[7] -= absx11 * x[6];
  x[8] -= absx21 * x[6];
  if (std::abs(x[5]) > std::abs(x[4])) {
    itmp = p2;
    p2 = p3;
    p3 = itmp;
    x[1] = absx21;
    x[2] = absx11;
    absx11 = x[4];
    x[4] = x[5];
    x[5] = absx11;
    absx11 = x[7];
    x[7] = x[8];
    x[8] = absx11;
  }

  absx11 = x[5] / x[4];
  x[8] -= absx11 * x[7];
  absx21 = (x[1] * absx11 - x[2]) / x[8];
  absx31 = -(x[7] * absx21 + x[1]) / x[4];
  y[p1 - 1] = ((1.0 - x[3] * absx31) - x[6] * absx21) / x[0];
  y[p1] = absx31;
  y[p1 + 1] = absx21;
  absx21 = -absx11 / x[8];
  absx31 = (1.0 - x[7] * absx21) / x[4];
  y[p2] = -(x[3] * absx31 + x[6] * absx21) / x[0];
  y[p2 + 1] = absx31;
  y[p2 + 2] = absx21;
  absx21 = 1.0 / x[8];
  absx31 = -x[7] * absx21 / x[4];
  y[p3] = -(x[3] * absx31 + x[6] * absx21) / x[0];
  y[p3 + 1] = absx31;
  y[p3 + 2] = absx21;
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
void PX4Controller_ertmodel0827ModelClass::step()
{
  // local block i/o variables
  real_T rtb_FilterCoefficient;
  real_T rtb_FilterCoefficient_h;
  real_T rtb_FilterCoefficient_p;
  real_T rtb_SumI4;
  real_T rtb_SumI4_m;
  real_T rtb_SumI4_b;
  real_T rtb_VectorConcatenate[18];
  real_T rtb_VectorConcatenate_a[9];
  real_T rtb_VectorConcatenate_o[9];
  real_T NormalizedSurfaceLimit[3];
  real_T rtb_Sum1_f[3];
  real_T rtb_Sum1_h[3];
  real_T rtb_Sum_c0[3];
  real_T rtb_V_windtoBody[3];
  real_T rtb_sincos_o1_p[3];
  real_T Selector_tmp;
  real_T Switch;
  real_T TmpSignalConversionAtphithetaps;
  real_T rtb_Divide1;
  real_T rtb_Gain1_h;
  real_T rtb_Integrator_e;
  real_T rtb_Product1_d;
  real_T rtb_Product3_n;
  real_T rtb_Product5;
  real_T rtb_Product_hi;
  real_T rtb_Product_i;
  real_T rtb_Product_n;
  real_T rtb_Sqrt;
  real_T rtb_Sum_db;
  real_T rtb_Switch1;
  real_T rtb_Switch3;
  real_T rtb_UnitConversion_idx_0;
  real_T rtb_UnitConversion_idx_1;
  real_T rtb_UnitConversion_idx_2;
  real_T rtb_V_windtoBody_tmp;
  real_T rtb_V_windtoBody_tmp_0;
  real_T rtb_p;
  real_T rtb_phi;
  real_T rtb_q;
  real_T rtb_r;
  real_T rtb_sincos_o1_m_idx_0;
  real_T rtb_sincos_o1_m_idx_1;
  real_T rtb_sincos_o1_m_idx_2;
  real_T rtb_sincos_o2_e_idx_1;
  real_T rtb_sincos_o2_idx_0;
  real_T rtb_sincos_o2_idx_2_tmp;
  real_T rtb_theta;
  real_T rtb_u_dot;
  real_T rtb_v_dot;
  real_T rtb_w_dot;
  int32_T i;
  int32_T rtb_VectorConcatenate_tmp;
  int32_T rtb_VectorConcatenate_tmp_0;
  int32_T rtb_VectorConcatenate_tmp_1;
  if (rtmIsMajorTimeStep((&PX4Controller_ertmodel0827_M))) {
    // set solver stop time
    rtsiSetSolverStopTime(&(&PX4Controller_ertmodel0827_M)->solverInfo,
                          (((&PX4Controller_ertmodel0827_M)->Timing.clockTick0+1)*
      (&PX4Controller_ertmodel0827_M)->Timing.stepSize0));
  }                                    // end MajorTimeStep

  // Update absolute time of base rate at minor time step
  if (rtmIsMinorTimeStep((&PX4Controller_ertmodel0827_M))) {
    (&PX4Controller_ertmodel0827_M)->Timing.t[0] = rtsiGetT
      (&(&PX4Controller_ertmodel0827_M)->solverInfo);
  }

  if (rtmIsMajorTimeStep((&PX4Controller_ertmodel0827_M))) {
    for (i = 0; i < 3; i++) {
      // Concatenate: '<S183>/Vector Concatenate' incorporates:
      //   Constant: '<S183>/Constant1'
      //   Constant: '<S183>/Constant2'
      //   Selector: '<S182>/Selector1'

      rtb_VectorConcatenate[6 * i] =
        PX4Controller_ertmodel0827_P.uDOFEulerAngles_inertia[3 * i];
      rtb_VectorConcatenate_tmp = 6 * i + 3;
      rtb_VectorConcatenate[rtb_VectorConcatenate_tmp] =
        PX4Controller_ertmodel0827_P.Constant2_Value[3 * i];

      // Selector: '<S182>/Selector1' incorporates:
      //   Concatenate: '<S183>/Vector Concatenate'

      PX4Controller_ertmodel0827_B.Selector1[3 * i] =
        rtb_VectorConcatenate[rtb_VectorConcatenate_tmp];

      // Selector: '<S182>/Selector' incorporates:
      //   Concatenate: '<S183>/Vector Concatenate'
      //   Selector: '<S182>/Selector2'

      Selector_tmp = rtb_VectorConcatenate[6 * i];

      // Selector: '<S182>/Selector'
      PX4Controller_ertmodel0827_B.Selector[3 * i] = Selector_tmp;

      // Selector: '<S182>/Selector2'
      PX4Controller_ertmodel0827_B.Selector2[3 * i] = Selector_tmp;

      // Concatenate: '<S183>/Vector Concatenate' incorporates:
      //   Constant: '<S183>/Constant1'
      //   Constant: '<S183>/Constant2'
      //   Selector: '<S182>/Selector'
      //   Selector: '<S182>/Selector1'
      //   Selector: '<S182>/Selector2'

      rtb_VectorConcatenate_tmp = 3 * i + 1;
      rtb_VectorConcatenate_tmp_1 = 6 * i + 1;
      rtb_VectorConcatenate[rtb_VectorConcatenate_tmp_1] =
        PX4Controller_ertmodel0827_P.uDOFEulerAngles_inertia[rtb_VectorConcatenate_tmp];
      rtb_VectorConcatenate_tmp_0 = 6 * i + 4;
      rtb_VectorConcatenate[rtb_VectorConcatenate_tmp_0] =
        PX4Controller_ertmodel0827_P.Constant2_Value[rtb_VectorConcatenate_tmp];

      // Selector: '<S182>/Selector1' incorporates:
      //   Concatenate: '<S183>/Vector Concatenate'

      PX4Controller_ertmodel0827_B.Selector1[rtb_VectorConcatenate_tmp] =
        rtb_VectorConcatenate[rtb_VectorConcatenate_tmp_0];

      // Selector: '<S182>/Selector' incorporates:
      //   Concatenate: '<S183>/Vector Concatenate'
      //   Selector: '<S182>/Selector2'

      Selector_tmp = rtb_VectorConcatenate[rtb_VectorConcatenate_tmp_1];

      // Selector: '<S182>/Selector'
      PX4Controller_ertmodel0827_B.Selector[rtb_VectorConcatenate_tmp] =
        Selector_tmp;

      // Selector: '<S182>/Selector2'
      PX4Controller_ertmodel0827_B.Selector2[rtb_VectorConcatenate_tmp] =
        Selector_tmp;

      // Concatenate: '<S183>/Vector Concatenate' incorporates:
      //   Constant: '<S183>/Constant1'
      //   Constant: '<S183>/Constant2'
      //   Selector: '<S182>/Selector'
      //   Selector: '<S182>/Selector1'
      //   Selector: '<S182>/Selector2'

      rtb_VectorConcatenate_tmp = 3 * i + 2;
      rtb_VectorConcatenate_tmp_1 = 6 * i + 2;
      rtb_VectorConcatenate[rtb_VectorConcatenate_tmp_1] =
        PX4Controller_ertmodel0827_P.uDOFEulerAngles_inertia[rtb_VectorConcatenate_tmp];
      rtb_VectorConcatenate_tmp_0 = 6 * i + 5;
      rtb_VectorConcatenate[rtb_VectorConcatenate_tmp_0] =
        PX4Controller_ertmodel0827_P.Constant2_Value[rtb_VectorConcatenate_tmp];

      // Selector: '<S182>/Selector1' incorporates:
      //   Concatenate: '<S183>/Vector Concatenate'

      PX4Controller_ertmodel0827_B.Selector1[rtb_VectorConcatenate_tmp] =
        rtb_VectorConcatenate[rtb_VectorConcatenate_tmp_0];

      // Selector: '<S182>/Selector' incorporates:
      //   Concatenate: '<S183>/Vector Concatenate'
      //   Selector: '<S182>/Selector2'

      Selector_tmp = rtb_VectorConcatenate[rtb_VectorConcatenate_tmp_1];

      // Selector: '<S182>/Selector'
      PX4Controller_ertmodel0827_B.Selector[rtb_VectorConcatenate_tmp] =
        Selector_tmp;

      // Selector: '<S182>/Selector2'
      PX4Controller_ertmodel0827_B.Selector2[rtb_VectorConcatenate_tmp] =
        Selector_tmp;
    }

    // Product: '<S171>/Product' incorporates:
    //   Constant: '<S171>/Constant'
    //   Constant: '<S171>/Constant1'
    //   Gain: '<S171>/Gain1'
    //   Gain: '<S171>/Gain2'
    //   Math: '<S171>/Math Function1'

    rtb_phi = PX4Controller_ertmodel0827_P.V_I0 *
      PX4Controller_ertmodel0827_P.V_I0 *
      PX4Controller_ertmodel0827_P.Gain2_Gain_b * PX4Controller_ertmodel0827_P.S
      * PX4Controller_ertmodel0827_P.Constant1_Value;

    // Product: '<S4>/Divide' incorporates:
    //   Constant: '<S4>/Constant'
    //   Gain: '<S4>/Gain'

    rtb_VectorConcatenate_a[0] = PX4Controller_ertmodel0827_P.dCldsigmaA *
      0.17453292519943295 * PX4Controller_ertmodel0827_P.b * rtb_phi /
      PX4Controller_ertmodel0827_P.Ix;

    // SignalConversion generated from: '<S173>/Vector Concatenate' incorporates:
    //   Constant: '<S4>/Constant1'

    rtb_VectorConcatenate_a[1] = PX4Controller_ertmodel0827_P.Constant1_Value_n;

    // Product: '<S4>/Divide3' incorporates:
    //   Constant: '<S4>/Constant3'
    //   Gain: '<S4>/Gain3'

    rtb_VectorConcatenate_a[2] = PX4Controller_ertmodel0827_P.dCndsigmaA *
      0.17453292519943295 * PX4Controller_ertmodel0827_P.b * rtb_phi /
      PX4Controller_ertmodel0827_P.Iz;

    // SignalConversion generated from: '<S173>/Vector Concatenate' incorporates:
    //   Constant: '<S4>/Constant1'

    rtb_VectorConcatenate_a[3] = PX4Controller_ertmodel0827_P.Constant1_Value_n;

    // Product: '<S4>/Divide2' incorporates:
    //   Constant: '<S4>/Constant2'
    //   Gain: '<S4>/Gain2'

    rtb_VectorConcatenate_a[4] = PX4Controller_ertmodel0827_P.dCmdsigmaE *
      0.20943951023931956 * PX4Controller_ertmodel0827_P.c * rtb_phi /
      PX4Controller_ertmodel0827_P.Iy;

    // SignalConversion generated from: '<S173>/Vector Concatenate' incorporates:
    //   Constant: '<S4>/Constant1'

    rtb_VectorConcatenate_a[5] = PX4Controller_ertmodel0827_P.Constant1_Value_n;

    // Product: '<S4>/Divide1' incorporates:
    //   Constant: '<S4>/Constant'
    //   Gain: '<S4>/Gain1'

    rtb_VectorConcatenate_a[6] = PX4Controller_ertmodel0827_P.dCldsigmaR *
      0.38397243543875248 * PX4Controller_ertmodel0827_P.b * rtb_phi /
      PX4Controller_ertmodel0827_P.Ix;

    // SignalConversion generated from: '<S173>/Vector Concatenate' incorporates:
    //   Constant: '<S4>/Constant1'

    rtb_VectorConcatenate_a[7] = PX4Controller_ertmodel0827_P.Constant1_Value_n;

    // Product: '<S4>/Divide4' incorporates:
    //   Constant: '<S4>/Constant3'
    //   Gain: '<S4>/Gain4'

    rtb_VectorConcatenate_a[8] = PX4Controller_ertmodel0827_P.dCndsigmaR *
      0.38397243543875248 * PX4Controller_ertmodel0827_P.b * rtb_phi /
      PX4Controller_ertmodel0827_P.Iz;

    // Product: '<S172>/Product' incorporates:
    //   Concatenate: '<S173>/Vector Concatenate'

    rt_invd3x3_snf(rtb_VectorConcatenate_a, PX4Controller_ertmodel0827_B.Product);

    // Product: '<Root>/MixerInverseForAW' incorporates:
    //   Product: '<S172>/Product'

    rt_invd3x3_snf(PX4Controller_ertmodel0827_B.Product,
                   PX4Controller_ertmodel0827_B.MixerInverseForAW);

    // Gain: '<S206>/Gain1' incorporates:
    //   Constant: '<S199>/Fixed_Angle_of_Attack'

    PX4Controller_ertmodel0827_B.Gain1 =
      PX4Controller_ertmodel0827_P.Gain1_Gain_p *
      PX4Controller_ertmodel0827_P.alpha_0;

    // ZeroOrderHold: '<S5>/CodegenInputHold_w_x' incorporates:
    //   Inport: '<Root>/w_speed'

    PX4Controller_ertmodel0827_B.wind[0] = PX4Controller_ertmodel0827_U.w_speed
      [0];

    // ZeroOrderHold: '<S5>/CodegenInputHold_w_y' incorporates:
    //   Inport: '<Root>/w_speed'

    PX4Controller_ertmodel0827_B.wind[1] = PX4Controller_ertmodel0827_U.w_speed
      [1];

    // ZeroOrderHold: '<S5>/CodegenInputHold_w_z' incorporates:
    //   Inport: '<Root>/w_speed'

    PX4Controller_ertmodel0827_B.wind[2] = PX4Controller_ertmodel0827_U.w_speed
      [2];
  }

  // Trigonometry: '<S189>/sincos' incorporates:
  //   Integrator: '<S181>/phi theta psi'
  //   SignalConversion generated from: '<S189>/sincos'
  //   Trigonometry: '<S190>/sincos'

  rtb_sincos_o2_idx_0 = std::cos
    (PX4Controller_ertmodel0827_X.phithetapsi_CSTATE[2]);
  rtb_Divide1 = std::sin(PX4Controller_ertmodel0827_X.phithetapsi_CSTATE[2]);
  Selector_tmp = std::cos(PX4Controller_ertmodel0827_X.phithetapsi_CSTATE[1]);
  rtb_V_windtoBody_tmp_0 = std::sin
    (PX4Controller_ertmodel0827_X.phithetapsi_CSTATE[1]);
  rtb_sincos_o2_idx_2_tmp = std::cos
    (PX4Controller_ertmodel0827_X.phithetapsi_CSTATE[0]);
  rtb_V_windtoBody_tmp = std::sin
    (PX4Controller_ertmodel0827_X.phithetapsi_CSTATE[0]);

  // Fcn: '<S189>/Fcn11' incorporates:
  //   Trigonometry: '<S189>/sincos'

  rtb_VectorConcatenate_o[0] = rtb_sincos_o2_idx_0 * Selector_tmp;

  // Fcn: '<S189>/Fcn21' incorporates:
  //   Fcn: '<S189>/Fcn22'
  //   Trigonometry: '<S189>/sincos'

  rtb_Switch3 = rtb_V_windtoBody_tmp_0 * rtb_V_windtoBody_tmp;
  rtb_VectorConcatenate_o[1] = rtb_Switch3 * rtb_sincos_o2_idx_0 - rtb_Divide1 *
    rtb_sincos_o2_idx_2_tmp;

  // Fcn: '<S189>/Fcn31' incorporates:
  //   Fcn: '<S189>/Fcn32'
  //   Trigonometry: '<S189>/sincos'

  Switch = rtb_V_windtoBody_tmp_0 * rtb_sincos_o2_idx_2_tmp;
  rtb_VectorConcatenate_o[2] = Switch * rtb_sincos_o2_idx_0 + rtb_Divide1 *
    rtb_V_windtoBody_tmp;

  // Fcn: '<S189>/Fcn12' incorporates:
  //   Trigonometry: '<S189>/sincos'

  rtb_VectorConcatenate_o[3] = rtb_Divide1 * Selector_tmp;

  // Fcn: '<S189>/Fcn22' incorporates:
  //   Trigonometry: '<S189>/sincos'

  rtb_VectorConcatenate_o[4] = rtb_Switch3 * rtb_Divide1 + rtb_sincos_o2_idx_0 *
    rtb_sincos_o2_idx_2_tmp;

  // Fcn: '<S189>/Fcn32' incorporates:
  //   Trigonometry: '<S189>/sincos'

  rtb_VectorConcatenate_o[5] = Switch * rtb_Divide1 - rtb_sincos_o2_idx_0 *
    rtb_V_windtoBody_tmp;

  // Fcn: '<S189>/Fcn13' incorporates:
  //   Trigonometry: '<S189>/sincos'

  rtb_VectorConcatenate_o[6] = -rtb_V_windtoBody_tmp_0;

  // Fcn: '<S189>/Fcn23' incorporates:
  //   Trigonometry: '<S189>/sincos'

  rtb_VectorConcatenate_o[7] = Selector_tmp * rtb_V_windtoBody_tmp;

  // Fcn: '<S189>/Fcn33' incorporates:
  //   Trigonometry: '<S189>/sincos'

  rtb_VectorConcatenate_o[8] = Selector_tmp * rtb_sincos_o2_idx_2_tmp;

  // Sum: '<S199>/Sum' incorporates:
  //   Concatenate: '<S191>/Vector Concatenate'
  //   Integrator: '<S176>/ub,vb,wb'
  //   Product: '<S199>/V_wind to Body'

  for (i = 0; i < 3; i++) {
    rtb_V_windtoBody[i] = PX4Controller_ertmodel0827_X.ubvbwb_CSTATE[i] -
      ((rtb_VectorConcatenate_o[i + 3] * PX4Controller_ertmodel0827_B.wind[1] +
        rtb_VectorConcatenate_o[i] * PX4Controller_ertmodel0827_B.wind[0]) +
       rtb_VectorConcatenate_o[i + 6] * PX4Controller_ertmodel0827_B.wind[2]);
  }

  // End of Sum: '<S199>/Sum'

  // Clock: '<S199>/Clock1' incorporates:
  //   Clock: '<S199>/Clock2'
  //   Clock: '<S199>/Clock3'
  //   Clock: '<S199>/Clock4'
  //   Clock: '<S199>/Clock5'
  //   Clock: '<S199>/Clock6'
  //   Clock: '<S199>/Clock7'
  //   Clock: '<S199>/Clock8'
  //   Clock: '<S199>/Clock9'

  TmpSignalConversionAtphithetaps = (&PX4Controller_ertmodel0827_M)->Timing.t[0];

  // Switch: '<S199>/Switch3' incorporates:
  //   Clock: '<S199>/Clock1'
  //   Product: '<S199>/Divide'
  //   Sum: '<S199>/Sum1'
  //   Trigonometry: '<S199>/Atan'

  if (TmpSignalConversionAtphithetaps >
      PX4Controller_ertmodel0827_P.Switch3_Threshold) {
    rtb_Switch3 = std::atan(rtb_V_windtoBody[2] / rtb_V_windtoBody[0]) +
      PX4Controller_ertmodel0827_B.Gain1;
  } else {
    rtb_Switch3 = PX4Controller_ertmodel0827_B.Gain1;
  }

  // End of Switch: '<S199>/Switch3'

  // Sqrt: '<S199>/Sqrt' incorporates:
  //   Math: '<S199>/Square1'
  //   Math: '<S199>/Square2'
  //   Math: '<S199>/Square3'
  //   Sum: '<S199>/Sum7'

  rtb_Sqrt = std::sqrt((rtb_V_windtoBody[0] * rtb_V_windtoBody[0] +
                        rtb_V_windtoBody[1] * rtb_V_windtoBody[1]) +
                       rtb_V_windtoBody[2] * rtb_V_windtoBody[2]);

  // Switch: '<S199>/Switch1' incorporates:
  //   Constant: '<S199>/Constant'
  //   Constant: '<S199>/Constant1'

  if (rtb_Sqrt > PX4Controller_ertmodel0827_P.Switch1_Threshold) {
    rtb_Switch1 = PX4Controller_ertmodel0827_P.Constant_Value;
  } else {
    rtb_Switch1 = PX4Controller_ertmodel0827_P.Constant1_Value_i;
  }

  // End of Switch: '<S199>/Switch1'

  // Switch: '<S199>/Switch4'
  if (!(rtb_Switch1 > PX4Controller_ertmodel0827_P.Switch4_Threshold)) {
    // Switch: '<S199>/Switch17'
    if (!(TmpSignalConversionAtphithetaps >
          PX4Controller_ertmodel0827_P.Switch17_Threshold)) {
      // Switch: '<S199>/Switch4' incorporates:
      //   Constant: '<S199>/Constant15'

      rtb_Switch3 = PX4Controller_ertmodel0827_P.Constant15_Value;
    }

    // End of Switch: '<S199>/Switch17'
  }

  // End of Switch: '<S199>/Switch4'

  // Switch: '<S199>/Switch' incorporates:
  //   Switch: '<S199>/Switch2'

  if (rtb_Switch1 > PX4Controller_ertmodel0827_P.Switch_Threshold_l) {
    // Product: '<S199>/Divide1'
    rtb_Divide1 = rtb_V_windtoBody[1] / rtb_Sqrt;

    // Trigonometry: '<S199>/Atan1'
    if (rtb_Divide1 > 1.0) {
      rtb_Divide1 = 1.0;
    } else if (rtb_Divide1 < -1.0) {
      rtb_Divide1 = -1.0;
    }

    // Switch: '<S199>/Switch' incorporates:
    //   Trigonometry: '<S199>/Atan1'

    Switch = std::asin(rtb_Divide1);
  } else if (TmpSignalConversionAtphithetaps >
             PX4Controller_ertmodel0827_P.Switch2_Threshold) {
    // Product: '<S199>/Divide1'
    rtb_Divide1 = rtb_V_windtoBody[1] / rtb_Sqrt;

    // Trigonometry: '<S199>/Atan1'
    if (rtb_Divide1 > 1.0) {
      rtb_Divide1 = 1.0;
    } else if (rtb_Divide1 < -1.0) {
      rtb_Divide1 = -1.0;
    }

    // Switch: '<S199>/Switch2' incorporates:
    //   Switch: '<S199>/Switch'
    //   Trigonometry: '<S199>/Atan1'

    Switch = std::asin(rtb_Divide1);
  } else {
    // Switch: '<S199>/Switch' incorporates:
    //   Constant: '<S199>/Constant6'

    Switch = PX4Controller_ertmodel0827_P.Constant6_Value;
  }

  // End of Switch: '<S199>/Switch'

  // Trigonometry: '<S207>/sincos' incorporates:
  //   SignalConversion generated from: '<S199>/Matrix Concatenate'
  //
  rtb_sincos_o2_idx_0 = std::cos(rtb_Switch3);
  rtb_Product1_d = std::sin(rtb_Switch3);
  rtb_sincos_o2_e_idx_1 = std::cos(Switch);
  rtb_Product3_n = std::sin(Switch);

  // Product: '<S208>/u(3)*u(4)'
  PX4Controller_ertmodel0827_B.VectorConcatenate[0] = rtb_sincos_o2_idx_0 *
    rtb_sincos_o2_e_idx_1;

  // UnaryMinus: '<S211>/Unary Minus' incorporates:
  //   Product: '<S211>/u(2)*u(3)'

  PX4Controller_ertmodel0827_B.VectorConcatenate[1] = -(rtb_sincos_o2_idx_0 *
    rtb_Product3_n);

  // UnaryMinus: '<S214>/Unary Minus'
  PX4Controller_ertmodel0827_B.VectorConcatenate[2] = -rtb_Product1_d;

  // SignalConversion generated from: '<S217>/Vector Concatenate'
  PX4Controller_ertmodel0827_B.VectorConcatenate[3] = rtb_Product3_n;

  // SignalConversion generated from: '<S217>/Vector Concatenate'
  PX4Controller_ertmodel0827_B.VectorConcatenate[4] = rtb_sincos_o2_e_idx_1;
  if (rtmIsMajorTimeStep((&PX4Controller_ertmodel0827_M))) {
    // Constant: '<S215>/Constant'
    PX4Controller_ertmodel0827_B.VectorConcatenate[5] =
      PX4Controller_ertmodel0827_P.Constant_Value_j;
  }

  // Product: '<S210>/u(1)*u(4)'
  PX4Controller_ertmodel0827_B.VectorConcatenate[6] = rtb_Product1_d *
    rtb_sincos_o2_e_idx_1;

  // UnaryMinus: '<S213>/Unary Minus' incorporates:
  //   Product: '<S213>/u(1)*u(2)'

  PX4Controller_ertmodel0827_B.VectorConcatenate[7] = -(rtb_Product1_d *
    rtb_Product3_n);

  // SignalConversion generated from: '<S217>/Vector Concatenate'
  PX4Controller_ertmodel0827_B.VectorConcatenate[8] = rtb_sincos_o2_idx_0;

  // Gain: '<S180>/Gain' incorporates:
  //   Integrator: '<S176>/xe,ye,ze'

  rtb_Product_hi = PX4Controller_ertmodel0827_P.Gain_Gain *
    PX4Controller_ertmodel0827_X.xeyeze_CSTATE[2];

  // Switch: '<S180>/Switch' incorporates:
  //   Constant: '<S180>/Constant1'

  if (!(rtb_Product_hi > PX4Controller_ertmodel0827_P.Switch_Threshold_k)) {
    rtb_Product_hi = PX4Controller_ertmodel0827_P.Constant1_Value_h;
  }

  // End of Switch: '<S180>/Switch'

  // Saturate: '<S202>/Limit  altitude  to troposhere'
  if (rtb_Product_hi >
      PX4Controller_ertmodel0827_P.Limitaltitudetotroposhere_Upper) {
    rtb_sincos_o2_idx_0 =
      PX4Controller_ertmodel0827_P.Limitaltitudetotroposhere_Upper;
  } else if (rtb_Product_hi <
             PX4Controller_ertmodel0827_P.Limitaltitudetotroposhere_Lower) {
    rtb_sincos_o2_idx_0 =
      PX4Controller_ertmodel0827_P.Limitaltitudetotroposhere_Lower;
  } else {
    rtb_sincos_o2_idx_0 = rtb_Product_hi;
  }

  // End of Saturate: '<S202>/Limit  altitude  to troposhere'

  // Sum: '<S202>/Sum1' incorporates:
  //   Constant: '<S202>/Sea Level  Temperature'
  //   Gain: '<S202>/Lapse Rate'

  rtb_Product_i = PX4Controller_ertmodel0827_P.SeaLevelTemperature_Value -
    PX4Controller_ertmodel0827_P.LapseRate_Gain * rtb_sincos_o2_idx_0;

  // Gain: '<S202>/1//T0'
  rtb_Product1_d = PX4Controller_ertmodel0827_P.uT0_Gain * rtb_Product_i;

  // Sum: '<S202>/Sum' incorporates:
  //   Constant: '<S202>/Altitude of Troposphere'

  rtb_Divide1 = PX4Controller_ertmodel0827_P.AltitudeofTroposphere_Value -
    rtb_Product_hi;

  // Math: '<S202>/(T//T0)^(g//LR) ' incorporates:
  //   Constant: '<S202>/Constant'

  if ((rtb_Product1_d < 0.0) && (PX4Controller_ertmodel0827_P.Constant_Value_p >
       std::floor(PX4Controller_ertmodel0827_P.Constant_Value_p))) {
    rtb_sincos_o2_idx_0 = -rt_powd_snf(-rtb_Product1_d,
      PX4Controller_ertmodel0827_P.Constant_Value_p);
  } else {
    rtb_sincos_o2_idx_0 = rt_powd_snf(rtb_Product1_d,
      PX4Controller_ertmodel0827_P.Constant_Value_p);
  }

  // End of Math: '<S202>/(T//T0)^(g//LR) '

  // Saturate: '<S202>/Limit  altitude  to Stratosphere'
  if (rtb_Divide1 > PX4Controller_ertmodel0827_P.LimitaltitudetoStratosphere_Upp)
  {
    rtb_Divide1 = PX4Controller_ertmodel0827_P.LimitaltitudetoStratosphere_Upp;
  } else if (rtb_Divide1 <
             PX4Controller_ertmodel0827_P.LimitaltitudetoStratosphere_Low) {
    rtb_Divide1 = PX4Controller_ertmodel0827_P.LimitaltitudetoStratosphere_Low;
  }

  // End of Saturate: '<S202>/Limit  altitude  to Stratosphere'

  // Product: '<S202>/Product3' incorporates:
  //   Gain: '<S202>/g//R'
  //   Gain: '<S202>/rho0'
  //   Math: '<S202>/Stratosphere Model'
  //   Product: '<S202>/Product'
  //   Product: '<S202>/Product1'
  //
  //  About '<S202>/Stratosphere Model':
  //   Operator: exp

  rtb_Product3_n = rtb_sincos_o2_idx_0 / rtb_Product1_d *
    PX4Controller_ertmodel0827_P.rho0_Gain * std::exp(1.0 / rtb_Product_i *
    (PX4Controller_ertmodel0827_P.gR_Gain * rtb_Divide1));

  // Product: '<S199>/Product1' incorporates:
  //   Gain: '<S199>/Gain'
  //   Integrator: '<S176>/ub,vb,wb'
  //   Math: '<S199>/Square4'
  //   Math: '<S199>/Square5'
  //   Math: '<S199>/Square6'
  //   Sum: '<S199>/Sum18'

  rtb_Product1_d = ((PX4Controller_ertmodel0827_X.ubvbwb_CSTATE[0] *
                     PX4Controller_ertmodel0827_X.ubvbwb_CSTATE[0] +
                     PX4Controller_ertmodel0827_X.ubvbwb_CSTATE[1] *
                     PX4Controller_ertmodel0827_X.ubvbwb_CSTATE[1]) +
                    PX4Controller_ertmodel0827_X.ubvbwb_CSTATE[2] *
                    PX4Controller_ertmodel0827_X.ubvbwb_CSTATE[2]) *
    PX4Controller_ertmodel0827_P.Gain_Gain_e * rtb_Product3_n;

  // Sum: '<S199>/Sum4' incorporates:
  //   Constant: '<S199>/Constant4'
  //   Gain: '<S199>/Gain43'
  //   Sum: '<S199>/Sum2'

  rtb_Gain1_h = PX4Controller_ertmodel0827_P.dCLdalpha * rtb_Switch3 +
    PX4Controller_ertmodel0827_P.CL0;

  // UnitConversion: '<S177>/Unit Conversion' incorporates:
  //   Integrator: '<S181>/phi theta psi'

  // Unit Conversion - from: rad to: deg
  // Expression: output = (57.2958*input) + (0)
  rtb_sincos_o2_idx_0 = 57.295779513082323 *
    PX4Controller_ertmodel0827_X.phithetapsi_CSTATE[0];
  if (rtmIsMajorTimeStep((&PX4Controller_ertmodel0827_M))) {
    // Constant: '<S12>/Constant'
    PX4Controller_ertmodel0827_B.VectorConcatenate_m[0] =
      PX4Controller_ertmodel0827_P.Constant_Value_i;

    // SignalConversion generated from: '<S170>/Vector Concatenate' incorporates:
    //   Constant: '<S12>/Constant1'

    PX4Controller_ertmodel0827_B.VectorConcatenate_m[1] =
      PX4Controller_ertmodel0827_P.Constant1_Value_o;

    // SignalConversion generated from: '<S170>/Vector Concatenate' incorporates:
    //   Constant: '<S12>/Constant1'

    PX4Controller_ertmodel0827_B.VectorConcatenate_m[2] =
      PX4Controller_ertmodel0827_P.Constant1_Value_o;

    // SignalConversion generated from: '<S170>/Vector Concatenate' incorporates:
    //   Constant: '<S12>/Constant1'

    PX4Controller_ertmodel0827_B.VectorConcatenate_m[3] =
      PX4Controller_ertmodel0827_P.Constant1_Value_o;

    // ZeroOrderHold generated from: '<S5>/CodegenOutputHold_NED'
    rtb_phi = rtb_sincos_o2_idx_0;

    // UnitConversion: '<S1>/Unit Conversion' incorporates:
    //   ZeroOrderHold generated from: '<S5>/CodegenOutputHold_NED'

    // Unit Conversion - from: deg to: rad
    // Expression: output = (0.0174533*input) + (0)
    rtb_Integrator_e = 0.017453292519943295 * rtb_sincos_o2_idx_0;

    // Trigonometry: '<S12>/Trigonometric Function2'
    rtb_Sum_db = std::cos(rtb_Integrator_e);

    // SignalConversion generated from: '<S170>/Vector Concatenate'
    PX4Controller_ertmodel0827_B.VectorConcatenate_m[4] = rtb_Sum_db;

    // Trigonometry: '<S12>/Trigonometric Function'
    rtb_r = std::sin(rtb_Integrator_e);

    // Gain: '<S12>/Gain'
    PX4Controller_ertmodel0827_B.VectorConcatenate_m[5] =
      PX4Controller_ertmodel0827_P.Gain_Gain_b * rtb_r;
  }

  // UnitConversion: '<S178>/Unit Conversion' incorporates:
  //   Integrator: '<S181>/phi theta psi'

  // Unit Conversion - from: rad to: deg
  // Expression: output = (57.2958*input) + (0)
  rtb_sincos_o2_idx_0 = 57.295779513082323 *
    PX4Controller_ertmodel0827_X.phithetapsi_CSTATE[1];
  if (rtmIsMajorTimeStep((&PX4Controller_ertmodel0827_M))) {
    // ZeroOrderHold generated from: '<S5>/CodegenOutputHold_NED'
    rtb_theta = rtb_sincos_o2_idx_0;

    // UnitConversion: '<S2>/Unit Conversion' incorporates:
    //   ZeroOrderHold generated from: '<S5>/CodegenOutputHold_NED'

    // Unit Conversion - from: deg to: rad
    // Expression: output = (0.0174533*input) + (0)
    rtb_q = 0.017453292519943295 * rtb_sincos_o2_idx_0;

    // Gain: '<S12>/Gain1' incorporates:
    //   Trigonometry: '<S12>/Trigonometric Function3'

    PX4Controller_ertmodel0827_B.VectorConcatenate_m[6] =
      PX4Controller_ertmodel0827_P.Gain1_Gain_f * std::sin(rtb_q);

    // Trigonometry: '<S12>/Trigonometric Function1'
    rtb_p = std::cos(rtb_q);

    // Product: '<S12>/Product'
    PX4Controller_ertmodel0827_B.VectorConcatenate_m[7] = rtb_p * rtb_r;

    // Product: '<S12>/Product1'
    PX4Controller_ertmodel0827_B.VectorConcatenate_m[8] = rtb_Sum_db * rtb_p;

    // Gain: '<S3>/P_phi_sp' incorporates:
    //   Inport: '<Root>/phi_sp'
    //   Sum: '<S3>/Sum'

    PX4Controller_ertmodel0827_B.VectorConcatenate_e[0] =
      (PX4Controller_ertmodel0827_U.phi_sp - rtb_Integrator_e) *
      PX4Controller_ertmodel0827_P.P_phi_sp_Gain;

    // Gain: '<S3>/P_theta_sp' incorporates:
    //   Inport: '<Root>/theta_sp'
    //   Sum: '<S3>/Sum1'

    PX4Controller_ertmodel0827_B.VectorConcatenate_e[1] =
      (PX4Controller_ertmodel0827_U.theta_sp - rtb_q) *
      PX4Controller_ertmodel0827_P.P_theta_sp_Gain;

    // Trigonometry: '<S13>/Trigonometric Function1' incorporates:
    //   Inport: '<Root>/theta_sp'

    PX4Controller_ertmodel0827_B.TrigonometricFunction1 = std::cos
      (PX4Controller_ertmodel0827_U.theta_sp);

    // Trigonometry: '<S13>/Trigonometric Function' incorporates:
    //   Inport: '<Root>/phi_sp'

    PX4Controller_ertmodel0827_B.TrigonometricFunction = std::tan
      (PX4Controller_ertmodel0827_U.phi_sp);
  }

  // Product: '<S13>/Product1' incorporates:
  //   Constant: '<S13>/Constant'
  //   Constant: '<S13>/Constant1'
  //   Gain: '<S13>/g'
  //   Product: '<S13>/Divide'
  //   Product: '<S13>/Product'
  //   Sum: '<S13>/Sum'

  PX4Controller_ertmodel0827_B.VectorConcatenate_e[2] =
    PX4Controller_ertmodel0827_P.Constant_Value_p1 / (rtb_Sqrt +
    PX4Controller_ertmodel0827_P.Constant1_Value_f) *
    PX4Controller_ertmodel0827_P.g_Gain *
    PX4Controller_ertmodel0827_B.TrigonometricFunction *
    PX4Controller_ertmodel0827_B.TrigonometricFunction1;

  // Product: '<S3>/Matrix Multiply' incorporates:
  //   Concatenate: '<S170>/Vector Concatenate'

  for (i = 0; i < 3; i++) {
    rtb_V_windtoBody[i] = PX4Controller_ertmodel0827_B.VectorConcatenate_m[i + 6]
      * PX4Controller_ertmodel0827_B.VectorConcatenate_e[2] +
      (PX4Controller_ertmodel0827_B.VectorConcatenate_m[i + 3] *
       PX4Controller_ertmodel0827_B.VectorConcatenate_e[1] +
       PX4Controller_ertmodel0827_B.VectorConcatenate_m[i] *
       PX4Controller_ertmodel0827_B.VectorConcatenate_e[0]);
  }

  // End of Product: '<S3>/Matrix Multiply'
  if (rtmIsMajorTimeStep((&PX4Controller_ertmodel0827_M))) {
    // Math: '<S21>/Math Function' incorporates:
    //   Constant: '<S21>/V_I0'

    PX4Controller_ertmodel0827_B.MathFunction =
      PX4Controller_ertmodel0827_P.V_I0 * PX4Controller_ertmodel0827_P.V_I0;

    // ZeroOrderHold generated from: '<S5>/CodegenOutputHold_BODY' incorporates:
    //   Integrator: '<S176>/p,q,r '

    rtb_p = PX4Controller_ertmodel0827_X.pqr_CSTATE[0];

    // SignalConversion generated from: '<Root>/Vector Concatenate1' incorporates:
    //   Integrator: '<S176>/p,q,r '

    PX4Controller_ertmodel0827_B.VectorConcatenate1[0] =
      PX4Controller_ertmodel0827_X.pqr_CSTATE[0];

    // ZeroOrderHold generated from: '<S5>/CodegenOutputHold_BODY' incorporates:
    //   Integrator: '<S176>/p,q,r '

    rtb_q = PX4Controller_ertmodel0827_X.pqr_CSTATE[1];

    // SignalConversion generated from: '<Root>/Vector Concatenate1' incorporates:
    //   Integrator: '<S176>/p,q,r '

    PX4Controller_ertmodel0827_B.VectorConcatenate1[1] =
      PX4Controller_ertmodel0827_X.pqr_CSTATE[1];

    // ZeroOrderHold generated from: '<S5>/CodegenOutputHold_BODY' incorporates:
    //   Integrator: '<S176>/p,q,r '

    rtb_r = PX4Controller_ertmodel0827_X.pqr_CSTATE[2];

    // SignalConversion generated from: '<Root>/Vector Concatenate1' incorporates:
    //   Integrator: '<S176>/p,q,r '

    PX4Controller_ertmodel0827_B.VectorConcatenate1[2] =
      PX4Controller_ertmodel0827_X.pqr_CSTATE[2];
  }

  // Sqrt: '<S9>/Sqrt' incorporates:
  //   Sqrt: '<S10>/Sqrt'
  //   Sqrt: '<S11>/Sqrt'

  rtb_Product_n = std::sqrt(rtb_Product3_n);

  // Product: '<S9>/Product' incorporates:
  //   Gain: '<S9>/Gain'
  //   Sqrt: '<S9>/Sqrt'

  rtb_Product_i = PX4Controller_ertmodel0827_P.Gain_Gain_j * rtb_Product_n *
    rtb_Sqrt;

  // Sum: '<S3>/Sum2'
  rtb_sincos_o2_idx_0 = rtb_V_windtoBody[0] -
    PX4Controller_ertmodel0827_B.VectorConcatenate1[0];
  rtb_Integrator_e = rtb_V_windtoBody[1] -
    PX4Controller_ertmodel0827_B.VectorConcatenate1[1];
  rtb_sincos_o2_e_idx_1 = rtb_V_windtoBody[2] -
    PX4Controller_ertmodel0827_B.VectorConcatenate1[2];
  if (rtmIsMajorTimeStep((&PX4Controller_ertmodel0827_M))) {
    // Gain: '<S56>/Filter Coefficient' incorporates:
    //   DiscreteIntegrator: '<S48>/Filter'
    //   Gain: '<S47>/Derivative Gain'
    //   Sum: '<S48>/SumD'

    rtb_FilterCoefficient = (PX4Controller_ertmodel0827_P.DiscretePDController_D
      * rtb_sincos_o2_idx_0 - PX4Controller_ertmodel0827_DW.Filter_DSTATE) *
      PX4Controller_ertmodel0827_P.DiscretePDController_N;

    // Sum: '<S62>/Sum' incorporates:
    //   DiscreteIntegrator: '<S53>/Integrator'
    //   Gain: '<S58>/Proportional Gain'

    rtb_Sum_db = (PX4Controller_ertmodel0827_P.DiscretePDController_P *
                  rtb_sincos_o2_idx_0 +
                  PX4Controller_ertmodel0827_DW.Integrator_DSTATE) +
      rtb_FilterCoefficient;

    // Saturate: '<S60>/Saturation'
    if (rtb_Sum_db >
        PX4Controller_ertmodel0827_P.DiscretePDController_UpperSatur) {
      // Saturate: '<S60>/Saturation'
      PX4Controller_ertmodel0827_B.Saturation =
        PX4Controller_ertmodel0827_P.DiscretePDController_UpperSatur;
    } else if (rtb_Sum_db <
               PX4Controller_ertmodel0827_P.DiscretePDController_LowerSatur) {
      // Saturate: '<S60>/Saturation'
      PX4Controller_ertmodel0827_B.Saturation =
        PX4Controller_ertmodel0827_P.DiscretePDController_LowerSatur;
    } else {
      // Saturate: '<S60>/Saturation'
      PX4Controller_ertmodel0827_B.Saturation = rtb_Sum_db;
    }

    // End of Saturate: '<S60>/Saturation'

    // Math: '<S121>/Math Function' incorporates:
    //   Constant: '<S121>/V_I0'

    PX4Controller_ertmodel0827_B.MathFunction_h =
      PX4Controller_ertmodel0827_P.V_I0 * PX4Controller_ertmodel0827_P.V_I0;

    // Gain: '<S147>/Derivative Gain'
    rtb_v_dot = PX4Controller_ertmodel0827_P.DiscretePDController_D_o *
      rtb_Integrator_e;

    // Gain: '<S156>/Filter Coefficient' incorporates:
    //   DiscreteIntegrator: '<S148>/Filter'
    //   Sum: '<S148>/SumD'

    rtb_FilterCoefficient_h = (rtb_v_dot -
      PX4Controller_ertmodel0827_DW.Filter_DSTATE_j) *
      PX4Controller_ertmodel0827_P.DiscretePDController_N_n;

    // Sum: '<S162>/Sum' incorporates:
    //   DiscreteIntegrator: '<S153>/Integrator'
    //   Gain: '<S158>/Proportional Gain'

    rtb_w_dot = (PX4Controller_ertmodel0827_P.DiscretePDController_P_k *
                 rtb_Integrator_e +
                 PX4Controller_ertmodel0827_DW.Integrator_DSTATE_b) +
      rtb_FilterCoefficient_h;

    // Saturate: '<S160>/Saturation'
    if (rtb_w_dot > PX4Controller_ertmodel0827_P.DiscretePDController_UpperSat_e)
    {
      // Saturate: '<S160>/Saturation'
      PX4Controller_ertmodel0827_B.Saturation_e =
        PX4Controller_ertmodel0827_P.DiscretePDController_UpperSat_e;
    } else if (rtb_w_dot <
               PX4Controller_ertmodel0827_P.DiscretePDController_LowerSat_h) {
      // Saturate: '<S160>/Saturation'
      PX4Controller_ertmodel0827_B.Saturation_e =
        PX4Controller_ertmodel0827_P.DiscretePDController_LowerSat_h;
    } else {
      // Saturate: '<S160>/Saturation'
      PX4Controller_ertmodel0827_B.Saturation_e = rtb_w_dot;
    }

    // End of Saturate: '<S160>/Saturation'

    // Math: '<S71>/Math Function' incorporates:
    //   Constant: '<S71>/V_I0'

    PX4Controller_ertmodel0827_B.MathFunction_f =
      PX4Controller_ertmodel0827_P.V_I0 * PX4Controller_ertmodel0827_P.V_I0;
  }

  // Product: '<S11>/Product' incorporates:
  //   Gain: '<S11>/Gain'

  rtb_Product_hi = PX4Controller_ertmodel0827_P.Gain_Gain_bc * rtb_Product_n *
    rtb_Sqrt;

  // Product: '<S10>/Product' incorporates:
  //   Gain: '<S10>/Gain'

  rtb_Product_n = PX4Controller_ertmodel0827_P.Gain_Gain_l * rtb_Product_n *
    rtb_Sqrt;
  if (rtmIsMajorTimeStep((&PX4Controller_ertmodel0827_M))) {
    // Gain: '<S97>/Derivative Gain'
    rtb_u_dot = PX4Controller_ertmodel0827_P.DiscretePDController_D_h *
      rtb_sincos_o2_e_idx_1;

    // Gain: '<S106>/Filter Coefficient' incorporates:
    //   DiscreteIntegrator: '<S98>/Filter'
    //   Sum: '<S98>/SumD'

    rtb_FilterCoefficient_p = (rtb_u_dot -
      PX4Controller_ertmodel0827_DW.Filter_DSTATE_n) *
      PX4Controller_ertmodel0827_P.DiscretePDController_N_j;

    // Sum: '<S112>/Sum' incorporates:
    //   DiscreteIntegrator: '<S103>/Integrator'
    //   Gain: '<S108>/Proportional Gain'

    rtb_v_dot = (PX4Controller_ertmodel0827_P.DiscretePDController_P_n *
                 rtb_sincos_o2_e_idx_1 +
                 PX4Controller_ertmodel0827_DW.Integrator_DSTATE_e) +
      rtb_FilterCoefficient_p;

    // Saturate: '<S110>/Saturation'
    if (rtb_v_dot > PX4Controller_ertmodel0827_P.DiscretePDController_UpperSat_i)
    {
      // Saturate: '<S110>/Saturation'
      PX4Controller_ertmodel0827_B.Saturation_n =
        PX4Controller_ertmodel0827_P.DiscretePDController_UpperSat_i;
    } else if (rtb_v_dot <
               PX4Controller_ertmodel0827_P.DiscretePDController_LowerSat_f) {
      // Saturate: '<S110>/Saturation'
      PX4Controller_ertmodel0827_B.Saturation_n =
        PX4Controller_ertmodel0827_P.DiscretePDController_LowerSat_f;
    } else {
      // Saturate: '<S110>/Saturation'
      PX4Controller_ertmodel0827_B.Saturation_n = rtb_v_dot;
    }

    // End of Saturate: '<S110>/Saturation'
  }

  // Math: '<S21>/Math Function1' incorporates:
  //   Math: '<S9>/aw_V2'

  rtb_Product_i *= rtb_Product_i;

  // Sum: '<S3>/Sum4' incorporates:
  //   Constant: '<S14>/Vt0'
  //   Constant: '<S14>/e'
  //   Constant: '<S21>/e'
  //   Gain: '<S15>/FFGain'
  //   Math: '<S21>/Math Function1'
  //   Product: '<S14>/Divide'
  //   Product: '<S14>/Product'
  //   Product: '<S21>/Divide'
  //   Product: '<S21>/Product'
  //   Sum: '<S14>/Sum'
  //   Sum: '<S21>/Sum'

  rtb_sincos_o1_m_idx_0 = PX4Controller_ertmodel0827_B.MathFunction /
    (rtb_Product_i + PX4Controller_ertmodel0827_P.e_Value_o) *
    PX4Controller_ertmodel0827_B.Saturation + PX4Controller_ertmodel0827_P.V_t0 /
    (rtb_Sqrt + PX4Controller_ertmodel0827_P.e_Value) *
    (PX4Controller_ertmodel0827_P.FFGain_Gain * rtb_V_windtoBody[0]);

  // Math: '<S121>/Math Function1' incorporates:
  //   Math: '<S11>/aw_V2'

  rtb_Product_hi *= rtb_Product_hi;

  // Sum: '<S3>/Sum4' incorporates:
  //   Constant: '<S121>/e'
  //   Constant: '<S18>/Vt0'
  //   Constant: '<S18>/e'
  //   Gain: '<S19>/FFGain'
  //   Math: '<S121>/Math Function1'
  //   Product: '<S121>/Divide'
  //   Product: '<S121>/Product'
  //   Product: '<S18>/Divide'
  //   Product: '<S18>/Product'
  //   Sum: '<S121>/Sum'
  //   Sum: '<S18>/Sum'

  rtb_sincos_o1_m_idx_1 = PX4Controller_ertmodel0827_B.MathFunction_h /
    (rtb_Product_hi + PX4Controller_ertmodel0827_P.e_Value_g) *
    PX4Controller_ertmodel0827_B.Saturation_e +
    PX4Controller_ertmodel0827_P.V_t0 / (rtb_Sqrt +
    PX4Controller_ertmodel0827_P.e_Value_b) *
    (PX4Controller_ertmodel0827_P.FFGain_Gain_d * rtb_V_windtoBody[1]);

  // Math: '<S71>/Math Function1' incorporates:
  //   Math: '<S10>/aw_V2'

  rtb_Product_n *= rtb_Product_n;

  // Sum: '<S3>/Sum4' incorporates:
  //   Constant: '<S16>/Vt0'
  //   Constant: '<S16>/e'
  //   Constant: '<S71>/e'
  //   Gain: '<S17>/FFGain'
  //   Math: '<S71>/Math Function1'
  //   Product: '<S16>/Divide'
  //   Product: '<S16>/Product'
  //   Product: '<S71>/Divide'
  //   Product: '<S71>/Product'
  //   Sum: '<S16>/Sum'
  //   Sum: '<S71>/Sum'

  rtb_sincos_o1_m_idx_2 = PX4Controller_ertmodel0827_B.MathFunction_f /
    (rtb_Product_n + PX4Controller_ertmodel0827_P.e_Value_bf) *
    PX4Controller_ertmodel0827_B.Saturation_n +
    PX4Controller_ertmodel0827_P.V_t0 / (rtb_Sqrt +
    PX4Controller_ertmodel0827_P.e_Value_c) *
    (PX4Controller_ertmodel0827_P.FFGain_Gain_c * rtb_V_windtoBody[2]);
  for (i = 0; i < 3; i++) {
    // Product: '<Root>/Matrix Multiply' incorporates:
    //   Product: '<S172>/Product'

    rtb_Divide1 = PX4Controller_ertmodel0827_B.Product[i + 6] *
      rtb_sincos_o1_m_idx_2 + (PX4Controller_ertmodel0827_B.Product[i + 3] *
      rtb_sincos_o1_m_idx_1 + PX4Controller_ertmodel0827_B.Product[i] *
      rtb_sincos_o1_m_idx_0);

    // Saturate: '<Root>/NormalizedSurfaceLimit'
    if (rtb_Divide1 >
        PX4Controller_ertmodel0827_P.NormalizedSurfaceLimit_UpperSat) {
      // Saturate: '<Root>/NormalizedSurfaceLimit'
      NormalizedSurfaceLimit[i] =
        PX4Controller_ertmodel0827_P.NormalizedSurfaceLimit_UpperSat;
    } else if (rtb_Divide1 <
               PX4Controller_ertmodel0827_P.NormalizedSurfaceLimit_LowerSat) {
      // Saturate: '<Root>/NormalizedSurfaceLimit'
      NormalizedSurfaceLimit[i] =
        PX4Controller_ertmodel0827_P.NormalizedSurfaceLimit_LowerSat;
    } else {
      // Saturate: '<Root>/NormalizedSurfaceLimit'
      NormalizedSurfaceLimit[i] = rtb_Divide1;
    }

    // End of Saturate: '<Root>/NormalizedSurfaceLimit'

    // Product: '<Root>/Matrix Multiply'
    rtb_V_windtoBody[i] = rtb_Divide1;
  }

  if (rtmIsMajorTimeStep((&PX4Controller_ertmodel0827_M))) {
    // ZeroOrderHold: '<S5>/CodegenInputHold_delta_r'
    rtb_u_dot = NormalizedSurfaceLimit[2];

    // Lookup_n-D: '<S175>/1-D Lookup Table' incorporates:
    //   ZeroOrderHold: '<S5>/CodegenInputHold_delta_e'

    rtb_Divide1 = look1_binlxpw(NormalizedSurfaceLimit[1],
      PX4Controller_ertmodel0827_P.uDLookupTable_bp01Data,
      PX4Controller_ertmodel0827_P.uDLookupTable_tableData, 2U);

    // Lookup_n-D: '<S175>/1-D Lookup Table1' incorporates:
    //   ZeroOrderHold: '<S5>/CodegenInputHold_delta_a'

    rtb_UnitConversion_idx_1 = look1_binlxpw(NormalizedSurfaceLimit[0],
      PX4Controller_ertmodel0827_P.uDLookupTable1_bp01Data,
      PX4Controller_ertmodel0827_P.uDLookupTable1_tableData, 2U);

    // Lookup_n-D: '<S175>/1-D Lookup Table2' incorporates:
    //   ZeroOrderHold: '<S5>/CodegenInputHold_delta_r'

    rtb_UnitConversion_idx_2 = look1_binlxpw(NormalizedSurfaceLimit[2],
      PX4Controller_ertmodel0827_P.uDLookupTable2_bp01Data,
      PX4Controller_ertmodel0827_P.uDLookupTable2_tableData, 2U);

    // Saturate: '<S220>/Command Saturation'
    // Unit Conversion - from: deg to: rad
    // Expression: output = (0.0174533*input) + (0)
    if (rtb_Divide1 > PX4Controller_ertmodel0827_P.e_lim) {
      rtb_Divide1 = PX4Controller_ertmodel0827_P.e_lim;
    } else if (rtb_Divide1 < -PX4Controller_ertmodel0827_P.e_lim) {
      rtb_Divide1 = -PX4Controller_ertmodel0827_P.e_lim;
    }

    // End of Saturate: '<S220>/Command Saturation'

    // UnitConversion: '<S218>/Unit Conversion'
    rtb_UnitConversion_idx_0 = 0.017453292519943295 * rtb_Divide1;

    // Saturate: '<S219>/Command Saturation'
    if (rtb_UnitConversion_idx_1 > PX4Controller_ertmodel0827_P.a_lim) {
      rtb_UnitConversion_idx_1 = PX4Controller_ertmodel0827_P.a_lim;
    } else if (rtb_UnitConversion_idx_1 < -PX4Controller_ertmodel0827_P.a_lim) {
      rtb_UnitConversion_idx_1 = -PX4Controller_ertmodel0827_P.a_lim;
    }

    // End of Saturate: '<S219>/Command Saturation'

    // UnitConversion: '<S218>/Unit Conversion'
    rtb_UnitConversion_idx_1 *= 0.017453292519943295;

    // Saturate: '<S221>/Command Saturation'
    if (rtb_UnitConversion_idx_2 >
        PX4Controller_ertmodel0827_P.CommandSaturation_UpperSat) {
      rtb_UnitConversion_idx_2 =
        PX4Controller_ertmodel0827_P.CommandSaturation_UpperSat;
    } else if (rtb_UnitConversion_idx_2 <
               PX4Controller_ertmodel0827_P.CommandSaturation_LowerSat) {
      rtb_UnitConversion_idx_2 =
        PX4Controller_ertmodel0827_P.CommandSaturation_LowerSat;
    }

    // End of Saturate: '<S221>/Command Saturation'

    // UnitConversion: '<S218>/Unit Conversion'
    rtb_UnitConversion_idx_2 *= 0.017453292519943295;

    // Gain: '<S199>/Gain46'
    PX4Controller_ertmodel0827_B.Gain46 =
      PX4Controller_ertmodel0827_P.dCDdsigmaE * rtb_UnitConversion_idx_0;
  }

  // Switch: '<S199>/Switch12' incorporates:
  //   Constant: '<S199>/Constant13'
  //   Constant: '<S199>/Constant2'
  //   Gain: '<S199>/Gain42'
  //   MATLAB Function: '<S199>/0.018u^2+0.003583u+0.00128'
  //   Product: '<S199>/Product8'
  //   Sum: '<S199>/Sum4'
  //   Sum: '<S199>/Sum8'
  //   Switch: '<S199>/Switch13'

  if (rtb_Switch1 > PX4Controller_ertmodel0827_P.Switch12_Threshold) {
    rtb_Divide1 = ((((rtb_Gain1_h * rtb_Gain1_h * 0.018 + 0.003583 * rtb_Gain1_h)
                     + 0.00128) + PX4Controller_ertmodel0827_P.CD0) +
                   PX4Controller_ertmodel0827_B.Gain46) *
      PX4Controller_ertmodel0827_P.S * rtb_Product1_d;
  } else if (TmpSignalConversionAtphithetaps >
             PX4Controller_ertmodel0827_P.Switch13_Threshold) {
    // Switch: '<S199>/Switch13' incorporates:
    //   Constant: '<S199>/Constant2'
    //   Gain: '<S199>/Gain42'
    //   MATLAB Function: '<S199>/0.018u^2+0.003583u+0.00128'
    //   Product: '<S199>/Product8'
    //   Sum: '<S199>/Sum4'
    //   Sum: '<S199>/Sum8'

    rtb_Divide1 = ((((rtb_Gain1_h * rtb_Gain1_h * 0.018 + 0.003583 * rtb_Gain1_h)
                     + 0.00128) + PX4Controller_ertmodel0827_P.CD0) +
                   PX4Controller_ertmodel0827_B.Gain46) *
      PX4Controller_ertmodel0827_P.S * rtb_Product1_d;
  } else {
    rtb_Divide1 = PX4Controller_ertmodel0827_P.Constant13_Value;
  }

  // End of Switch: '<S199>/Switch12'

  // Gain: '<S199>/Gain16'
  rtb_sincos_o1_p[0] = PX4Controller_ertmodel0827_P.Gain16_Gain * rtb_Divide1;
  if (rtmIsMajorTimeStep((&PX4Controller_ertmodel0827_M))) {
    // Gain: '<S199>/Gain36'
    PX4Controller_ertmodel0827_B.Gain36 =
      PX4Controller_ertmodel0827_P.dCydsigmaA * rtb_UnitConversion_idx_1;

    // Gain: '<S199>/Gain27'
    PX4Controller_ertmodel0827_B.Gain27 =
      PX4Controller_ertmodel0827_P.dCydsigmaR * rtb_UnitConversion_idx_2;

    // Gain: '<S199>/Gain10'
    PX4Controller_ertmodel0827_B.Gain10 =
      PX4Controller_ertmodel0827_P.dCLdsigmaE * rtb_UnitConversion_idx_0;
  }

  // Switch: '<S199>/Switch10' incorporates:
  //   Constant: '<S199>/Constant17'
  //   Constant: '<S199>/Constant18'
  //   Constant: '<S199>/Constant21'
  //   Constant: '<S199>/Constant22'
  //   Gain: '<S199>/Gain17'
  //   Gain: '<S199>/Gain24'
  //   Gain: '<S199>/Gain28'
  //   Gain: '<S199>/Gain29'
  //   Gain: '<S199>/Gain30'
  //   Gain: '<S199>/Gain35'
  //   Integrator: '<S176>/p,q,r '
  //   Product: '<S199>/Divide10'
  //   Product: '<S199>/Divide9'
  //   Product: '<S199>/Product6'
  //   Sum: '<S199>/Sum14'
  //   Sum: '<S199>/Sum15'
  //   Sum: '<S199>/Sum6'
  //   Switch: '<S199>/Switch7'

  if (rtb_Switch1 > PX4Controller_ertmodel0827_P.Switch10_Threshold) {
    rtb_sincos_o1_p[1] = ((((PX4Controller_ertmodel0827_P.b / 2.0 *
      PX4Controller_ertmodel0827_X.pqr_CSTATE[0] / (rtb_Sqrt +
      PX4Controller_ertmodel0827_P.Constant22_Value) *
      PX4Controller_ertmodel0827_P.dCydp +
      (PX4Controller_ertmodel0827_P.dCydbeta * Switch +
       PX4Controller_ertmodel0827_P.Cy0)) + PX4Controller_ertmodel0827_P.b / 2.0
      * PX4Controller_ertmodel0827_X.pqr_CSTATE[2] / (rtb_Sqrt +
      PX4Controller_ertmodel0827_P.Constant21_Value) *
      PX4Controller_ertmodel0827_P.dCydr) + PX4Controller_ertmodel0827_B.Gain36)
                          + PX4Controller_ertmodel0827_B.Gain27) *
      PX4Controller_ertmodel0827_P.S * rtb_Product1_d;
  } else if (TmpSignalConversionAtphithetaps >
             PX4Controller_ertmodel0827_P.Switch7_Threshold) {
    // Switch: '<S199>/Switch7' incorporates:
    //   Constant: '<S199>/Constant17'
    //   Constant: '<S199>/Constant21'
    //   Constant: '<S199>/Constant22'
    //   Gain: '<S199>/Gain17'
    //   Gain: '<S199>/Gain24'
    //   Gain: '<S199>/Gain28'
    //   Gain: '<S199>/Gain29'
    //   Gain: '<S199>/Gain30'
    //   Gain: '<S199>/Gain35'
    //   Integrator: '<S176>/p,q,r '
    //   Product: '<S199>/Divide10'
    //   Product: '<S199>/Divide9'
    //   Product: '<S199>/Product6'
    //   Sum: '<S199>/Sum14'
    //   Sum: '<S199>/Sum15'
    //   Sum: '<S199>/Sum6'

    rtb_sincos_o1_p[1] = ((((PX4Controller_ertmodel0827_P.b / 2.0 *
      PX4Controller_ertmodel0827_X.pqr_CSTATE[0] / (rtb_Sqrt +
      PX4Controller_ertmodel0827_P.Constant22_Value) *
      PX4Controller_ertmodel0827_P.dCydp +
      (PX4Controller_ertmodel0827_P.dCydbeta * Switch +
       PX4Controller_ertmodel0827_P.Cy0)) + PX4Controller_ertmodel0827_P.b / 2.0
      * PX4Controller_ertmodel0827_X.pqr_CSTATE[2] / (rtb_Sqrt +
      PX4Controller_ertmodel0827_P.Constant21_Value) *
      PX4Controller_ertmodel0827_P.dCydr) + PX4Controller_ertmodel0827_B.Gain36)
                          + PX4Controller_ertmodel0827_B.Gain27) *
      PX4Controller_ertmodel0827_P.S * rtb_Product1_d;
  } else {
    rtb_sincos_o1_p[1] = PX4Controller_ertmodel0827_P.Constant18_Value;
  }

  // End of Switch: '<S199>/Switch10'

  // Switch: '<S199>/Switch5' incorporates:
  //   Constant: '<S199>/Constant23'
  //   Constant: '<S199>/Constant9'
  //   Gain: '<S199>/Gain1'
  //   Gain: '<S199>/Gain8'
  //   Gain: '<S199>/Gain9'
  //   Integrator: '<S176>/p,q,r '
  //   Product: '<S199>/Divide4'
  //   Product: '<S199>/Product3'
  //   Sum: '<S199>/Sum16'
  //   Sum: '<S199>/Sum2'
  //   Switch: '<S199>/Switch14'

  if (rtb_Switch1 > PX4Controller_ertmodel0827_P.Switch5_Threshold) {
    rtb_Divide1 = ((PX4Controller_ertmodel0827_P.c / 2.0 *
                    PX4Controller_ertmodel0827_X.pqr_CSTATE[1] / (rtb_Sqrt +
      PX4Controller_ertmodel0827_P.Constant23_Value) *
                    PX4Controller_ertmodel0827_P.dCLdq + rtb_Gain1_h) +
                   PX4Controller_ertmodel0827_B.Gain10) *
      PX4Controller_ertmodel0827_P.S * rtb_Product1_d;
  } else if (TmpSignalConversionAtphithetaps >
             PX4Controller_ertmodel0827_P.Switch14_Threshold) {
    // Switch: '<S199>/Switch14' incorporates:
    //   Constant: '<S199>/Constant23'
    //   Constant: '<S199>/Constant3'
    //   Gain: '<S199>/Gain1'
    //   Gain: '<S199>/Gain7'
    //   Gain: '<S199>/Gain8'
    //   Gain: '<S199>/Gain9'
    //   Integrator: '<S176>/p,q,r '
    //   Product: '<S199>/Divide4'
    //   Product: '<S199>/Product3'
    //   Sum: '<S199>/Sum16'
    //   Sum: '<S199>/Sum2'

    rtb_Divide1 = ((PX4Controller_ertmodel0827_P.c / 2.0 *
                    PX4Controller_ertmodel0827_X.pqr_CSTATE[1] / (rtb_Sqrt +
      PX4Controller_ertmodel0827_P.Constant23_Value) *
                    PX4Controller_ertmodel0827_P.dCLdq +
                    (PX4Controller_ertmodel0827_P.dCLdalpha * rtb_Switch3 +
                     PX4Controller_ertmodel0827_P.CL0)) +
                   PX4Controller_ertmodel0827_B.Gain10) *
      PX4Controller_ertmodel0827_P.S * rtb_Product1_d;
  } else {
    rtb_Divide1 = PX4Controller_ertmodel0827_P.Constant9_Value;
  }

  // End of Switch: '<S199>/Switch5'

  // Gain: '<S199>/Gain23'
  rtb_sincos_o1_p[2] = PX4Controller_ertmodel0827_P.Gain23_Gain * rtb_Divide1;
  if (rtmIsMajorTimeStep((&PX4Controller_ertmodel0827_M))) {
    // Gain: '<S200>/G=mg' incorporates:
    //   Constant: '<S180>/Constant'

    PX4Controller_ertmodel0827_B.Gmg[0] = PX4Controller_ertmodel0827_P.Gmg_Gain
      [0] * PX4Controller_ertmodel0827_P.m;
    PX4Controller_ertmodel0827_B.Gmg[1] = PX4Controller_ertmodel0827_P.Gmg_Gain
      [1] * PX4Controller_ertmodel0827_P.m;
    PX4Controller_ertmodel0827_B.Gmg[2] = PX4Controller_ertmodel0827_P.Gmg_Gain
      [2] * PX4Controller_ertmodel0827_P.m;

    // Sqrt: '<S203>/Sqrt' incorporates:
    //   Constant: '<S203>/Constant3'
    //   Constant: '<S203>/Pi'
    //   Product: '<S203>/Divide'

    rtb_u_dot = std::sqrt(PX4Controller_ertmodel0827_P.Sprop /
                          PX4Controller_ertmodel0827_P.Pi_Value);

    // Saturate: '<S5>/Saturation' incorporates:
    //   Inport: '<Root>/T_cmd'

    if (PX4Controller_ertmodel0827_U.T_cmd >
        PX4Controller_ertmodel0827_P.Saturation_UpperSat) {
      rtb_Product5 = PX4Controller_ertmodel0827_P.Saturation_UpperSat;
    } else if (PX4Controller_ertmodel0827_U.T_cmd <
               PX4Controller_ertmodel0827_P.Saturation_LowerSat) {
      rtb_Product5 = PX4Controller_ertmodel0827_P.Saturation_LowerSat;
    } else {
      rtb_Product5 = PX4Controller_ertmodel0827_U.T_cmd;
    }

    // End of Saturate: '<S5>/Saturation'

    // Saturate: '<S203>/Saturation'
    if (rtb_Product5 > PX4Controller_ertmodel0827_P.Saturation_UpperSat_d) {
      rtb_Product5 = PX4Controller_ertmodel0827_P.Saturation_UpperSat_d;
    } else if (rtb_Product5 < PX4Controller_ertmodel0827_P.Saturation_LowerSat_i)
    {
      rtb_Product5 = PX4Controller_ertmodel0827_P.Saturation_LowerSat_i;
    }

    // End of Saturate: '<S203>/Saturation'

    // Gain: '<S203>/Gain4' incorporates:
    //   Constant: '<S203>/Constant'
    //   Math: '<S203>/Power'

    rtb_Product5 = PX4Controller_ertmodel0827_P.omega_max * rt_powd_snf
      (rtb_Product5, PX4Controller_ertmodel0827_P.Constant_Value_e);

    // Product: '<S203>/Product2'
    rtb_Divide1 = rtb_u_dot * rtb_Product5;

    // Switch: '<S203>/Switch1'
    if (rtb_Divide1 > PX4Controller_ertmodel0827_P.Switch1_Threshold_b) {
      // Switch: '<S203>/Switch1'
      PX4Controller_ertmodel0827_B.Switch1 = rtb_Divide1;
    } else {
      // Switch: '<S203>/Switch1' incorporates:
      //   Constant: '<S203>/Constant4'

      PX4Controller_ertmodel0827_B.Switch1 =
        PX4Controller_ertmodel0827_P.Constant4_Value;
    }

    // End of Switch: '<S203>/Switch1'

    // Gain: '<S203>/Gain'
    rtb_u_dot *= PX4Controller_ertmodel0827_P.Gain_Gain_c;

    // Math: '<S203>/Power1' incorporates:
    //   Constant: '<S203>/constant1'

    PX4Controller_ertmodel0827_B.Power1 = rt_powd_snf(rtb_u_dot,
      PX4Controller_ertmodel0827_P.constant1_Value);

    // Product: '<S203>/Divide4' incorporates:
    //   Constant: '<S203>/Pi1'
    //   Gain: '<S203>/Gain8'

    rtb_Divide1 = rtb_Product5 / (PX4Controller_ertmodel0827_P.Gain8_Gain *
      PX4Controller_ertmodel0827_P.Pi1_Value);

    // Math: '<S203>/Square'
    PX4Controller_ertmodel0827_B.Square = rtb_Divide1 * rtb_Divide1;
  }

  // Product: '<S203>/Divide1' incorporates:
  //   Constant: '<S203>/Pi'
  //   Product: '<S203>/Product1'

  rtb_Divide1 = rtb_Sqrt * PX4Controller_ertmodel0827_P.Pi_Value /
    PX4Controller_ertmodel0827_B.Switch1;

  // Product: '<S203>/Product3' incorporates:
  //   Constant: '<S203>/Jmax'
  //   Constant: '<S203>/Jmax2'
  //   Gain: '<S203>/Gain1'
  //   Product: '<S203>/Divide2'
  //   Sum: '<S203>/Sum'

  rtb_Gain1_h = (PX4Controller_ertmodel0827_P.Jmax2_Value - rtb_Divide1 /
                 PX4Controller_ertmodel0827_P.Jmax_Value) *
    PX4Controller_ertmodel0827_P.k_motor * rtb_Product3_n *
    PX4Controller_ertmodel0827_B.Power1 * PX4Controller_ertmodel0827_B.Square;

  // Switch: '<S203>/Switch' incorporates:
  //   Constant: '<S203>/Constant1'

  if (rtb_Gain1_h > PX4Controller_ertmodel0827_P.Switch_Threshold_h) {
    rtb_Sum_c0[0] = rtb_Gain1_h;
  } else {
    rtb_Sum_c0[0] = PX4Controller_ertmodel0827_P.Constant1_Value_a;
  }

  // End of Switch: '<S203>/Switch'

  // SignalConversion generated from: '<S203>/Matrix Concatenate' incorporates:
  //   Constant: '<S203>/Constant1'

  rtb_Sum_c0[1] = PX4Controller_ertmodel0827_P.Constant1_Value_a;

  // SignalConversion generated from: '<S203>/Matrix Concatenate' incorporates:
  //   Constant: '<S203>/Constant1'

  rtb_Sum_c0[2] = PX4Controller_ertmodel0827_P.Constant1_Value_a;
  for (i = 0; i < 3; i++) {
    // Sum: '<S201>/Sum' incorporates:
    //   Concatenate: '<S191>/Vector Concatenate'
    //   Concatenate: '<S217>/Vector Concatenate'
    //   Math: '<S199>/Transpose1'
    //   Product: '<S199>/Matrix Multiply'
    //   Product: '<S200>/Matrix Multiply'
    //   Sum: '<S192>/Sum'
    //   Trigonometry: '<S190>/sincos'

    rtb_Sum1_f[i] = ((PX4Controller_ertmodel0827_B.VectorConcatenate[3 * i + 1] *
                      rtb_sincos_o1_p[1] +
                      PX4Controller_ertmodel0827_B.VectorConcatenate[3 * i] *
                      rtb_sincos_o1_p[0]) +
                     PX4Controller_ertmodel0827_B.VectorConcatenate[3 * i + 2] *
                     rtb_sincos_o1_p[2]) + ((rtb_VectorConcatenate_o[i + 6] *
      PX4Controller_ertmodel0827_B.Gmg[2] + (rtb_VectorConcatenate_o[i + 3] *
      PX4Controller_ertmodel0827_B.Gmg[1] + rtb_VectorConcatenate_o[i] *
      PX4Controller_ertmodel0827_B.Gmg[0])) + rtb_Sum_c0[i]);
  }

  // Gain: '<S180>/Gain1' incorporates:
  //   Integrator: '<S176>/xe,ye,ze'

  rtb_Gain1_h = PX4Controller_ertmodel0827_P.Gain1_Gain_c *
    PX4Controller_ertmodel0827_X.xeyeze_CSTATE[2];

  // Switch: '<S201>/Switch1'
  if (!(rtb_Gain1_h > PX4Controller_ertmodel0827_P.Switch1_Threshold_h)) {
    // Switch: '<S201>/Switch' incorporates:
    //   Gain: '<S201>/Gain1'
    //   Sum: '<S201>/Sum'

    if (rtb_Sum1_f[2] > PX4Controller_ertmodel0827_P.Switch_Threshold) {
      rtb_Sum1_f[0] *= PX4Controller_ertmodel0827_P.Gain1_Gain[0];
      rtb_Sum1_f[1] *= PX4Controller_ertmodel0827_P.Gain1_Gain[1];
      rtb_Sum1_f[2] *= PX4Controller_ertmodel0827_P.Gain1_Gain[2];
    }

    // End of Switch: '<S201>/Switch'
  }

  // End of Switch: '<S201>/Switch1'

  // Sum: '<S176>/Sum' incorporates:
  //   Constant: '<S183>/Constant'
  //   Integrator: '<S176>/p,q,r '
  //   Integrator: '<S176>/ub,vb,wb'
  //   Product: '<S176>/Product'
  //   Product: '<S197>/i x j'
  //   Product: '<S197>/j x k'
  //   Product: '<S197>/k x i'
  //   Product: '<S198>/i x k'
  //   Product: '<S198>/j x i'
  //   Product: '<S198>/k x j'
  //   Sum: '<S184>/Sum'

  PX4Controller_ertmodel0827_B.Sum[0] =
    (PX4Controller_ertmodel0827_X.ubvbwb_CSTATE[1] *
     PX4Controller_ertmodel0827_X.pqr_CSTATE[2] -
     PX4Controller_ertmodel0827_X.pqr_CSTATE[1] *
     PX4Controller_ertmodel0827_X.ubvbwb_CSTATE[2]) + rtb_Sum1_f[0] /
    PX4Controller_ertmodel0827_P.m;
  PX4Controller_ertmodel0827_B.Sum[1] =
    (PX4Controller_ertmodel0827_X.pqr_CSTATE[0] *
     PX4Controller_ertmodel0827_X.ubvbwb_CSTATE[2] -
     PX4Controller_ertmodel0827_X.ubvbwb_CSTATE[0] *
     PX4Controller_ertmodel0827_X.pqr_CSTATE[2]) + rtb_Sum1_f[1] /
    PX4Controller_ertmodel0827_P.m;
  PX4Controller_ertmodel0827_B.Sum[2] =
    (PX4Controller_ertmodel0827_X.ubvbwb_CSTATE[0] *
     PX4Controller_ertmodel0827_X.pqr_CSTATE[1] -
     PX4Controller_ertmodel0827_X.pqr_CSTATE[0] *
     PX4Controller_ertmodel0827_X.ubvbwb_CSTATE[1]) + rtb_Sum1_f[2] /
    PX4Controller_ertmodel0827_P.m;
  if (rtmIsMajorTimeStep((&PX4Controller_ertmodel0827_M))) {
    // Math: '<S203>/Power2' incorporates:
    //   Constant: '<S203>/constant2'

    PX4Controller_ertmodel0827_B.Power2 = rt_powd_snf(rtb_u_dot,
      PX4Controller_ertmodel0827_P.constant2_Value);

    // Product: '<S203>/Divide5' incorporates:
    //   Constant: '<S203>/Pi2'
    //   Gain: '<S203>/Gain9'

    rtb_Product5 /= PX4Controller_ertmodel0827_P.Gain9_Gain *
      PX4Controller_ertmodel0827_P.Pi2_Value;

    // Math: '<S203>/Square1'
    PX4Controller_ertmodel0827_B.Square1 = rtb_Product5 * rtb_Product5;

    // Gain: '<S199>/Gain6'
    PX4Controller_ertmodel0827_B.Gain6 = PX4Controller_ertmodel0827_P.dCldsigmaA
      * rtb_UnitConversion_idx_1;

    // Gain: '<S199>/Gain39'
    PX4Controller_ertmodel0827_B.Gain39 =
      PX4Controller_ertmodel0827_P.dCldsigmaR * rtb_UnitConversion_idx_2;

    // Gain: '<S199>/Gain15'
    PX4Controller_ertmodel0827_B.Gain15 =
      PX4Controller_ertmodel0827_P.dCmdsigmaE * rtb_UnitConversion_idx_0;
  }

  // Switch: '<S199>/Switch16' incorporates:
  //   Constant: '<S199>/Constant10'
  //   Constant: '<S199>/Constant19'
  //   Constant: '<S199>/Constant20'
  //   Constant: '<S199>/Constant7'
  //   Gain: '<S199>/Gain37'
  //   Gain: '<S199>/Gain4'
  //   Gain: '<S199>/Gain47'
  //   Gain: '<S199>/Gain48'
  //   Gain: '<S199>/Gain49'
  //   Gain: '<S199>/Gain5'
  //   Integrator: '<S176>/p,q,r '
  //   Product: '<S199>/Divide7'
  //   Product: '<S199>/Divide8'
  //   Product: '<S199>/Product2'
  //   Sum: '<S199>/Sum11'
  //   Sum: '<S199>/Sum12'
  //   Sum: '<S199>/Sum13'
  //   Switch: '<S199>/Switch8'

  if (rtb_Switch1 > PX4Controller_ertmodel0827_P.Switch16_Threshold) {
    rtb_sincos_o1_p[0] = ((((PX4Controller_ertmodel0827_P.b / 2.0 *
      PX4Controller_ertmodel0827_X.pqr_CSTATE[0] / (rtb_Sqrt +
      PX4Controller_ertmodel0827_P.Constant20_Value) *
      PX4Controller_ertmodel0827_P.dCldp +
      (PX4Controller_ertmodel0827_P.dCldbeta * Switch +
       PX4Controller_ertmodel0827_P.Cl0)) + PX4Controller_ertmodel0827_P.b / 2.0
      * PX4Controller_ertmodel0827_X.pqr_CSTATE[2] / (rtb_Sqrt +
      PX4Controller_ertmodel0827_P.Constant19_Value) *
      PX4Controller_ertmodel0827_P.dCldr) + PX4Controller_ertmodel0827_B.Gain6)
                          + PX4Controller_ertmodel0827_B.Gain39) *
      (PX4Controller_ertmodel0827_P.S * PX4Controller_ertmodel0827_P.b) *
      rtb_Product1_d;
  } else if (TmpSignalConversionAtphithetaps >
             PX4Controller_ertmodel0827_P.Switch8_Threshold) {
    // Switch: '<S199>/Switch8' incorporates:
    //   Constant: '<S199>/Constant19'
    //   Constant: '<S199>/Constant20'
    //   Constant: '<S199>/Constant7'
    //   Gain: '<S199>/Gain37'
    //   Gain: '<S199>/Gain4'
    //   Gain: '<S199>/Gain47'
    //   Gain: '<S199>/Gain48'
    //   Gain: '<S199>/Gain49'
    //   Gain: '<S199>/Gain5'
    //   Integrator: '<S176>/p,q,r '
    //   Product: '<S199>/Divide7'
    //   Product: '<S199>/Divide8'
    //   Product: '<S199>/Product2'
    //   Sum: '<S199>/Sum11'
    //   Sum: '<S199>/Sum12'
    //   Sum: '<S199>/Sum13'

    rtb_sincos_o1_p[0] = ((((PX4Controller_ertmodel0827_P.b / 2.0 *
      PX4Controller_ertmodel0827_X.pqr_CSTATE[0] / (rtb_Sqrt +
      PX4Controller_ertmodel0827_P.Constant20_Value) *
      PX4Controller_ertmodel0827_P.dCldp +
      (PX4Controller_ertmodel0827_P.dCldbeta * Switch +
       PX4Controller_ertmodel0827_P.Cl0)) + PX4Controller_ertmodel0827_P.b / 2.0
      * PX4Controller_ertmodel0827_X.pqr_CSTATE[2] / (rtb_Sqrt +
      PX4Controller_ertmodel0827_P.Constant19_Value) *
      PX4Controller_ertmodel0827_P.dCldr) + PX4Controller_ertmodel0827_B.Gain6)
                          + PX4Controller_ertmodel0827_B.Gain39) *
      (PX4Controller_ertmodel0827_P.S * PX4Controller_ertmodel0827_P.b) *
      rtb_Product1_d;
  } else {
    rtb_sincos_o1_p[0] = PX4Controller_ertmodel0827_P.Constant10_Value;
  }

  // End of Switch: '<S199>/Switch16'

  // Switch: '<S199>/Switch11' incorporates:
  //   Constant: '<S199>/Constant11'
  //   Constant: '<S199>/Constant24'
  //   Constant: '<S199>/Constant8'
  //   Gain: '<S199>/Gain11'
  //   Gain: '<S199>/Gain12'
  //   Gain: '<S199>/Gain13'
  //   Gain: '<S199>/Gain14'
  //   Integrator: '<S176>/p,q,r '
  //   Product: '<S199>/Divide3'
  //   Product: '<S199>/Product4'
  //   Sum: '<S199>/Sum17'
  //   Sum: '<S199>/Sum3'
  //   Switch: '<S199>/Switch6'

  if (rtb_Switch1 > PX4Controller_ertmodel0827_P.Switch11_Threshold) {
    rtb_sincos_o1_p[1] = ((PX4Controller_ertmodel0827_P.c / 2.0 *
      PX4Controller_ertmodel0827_X.pqr_CSTATE[1] / (rtb_Sqrt +
      PX4Controller_ertmodel0827_P.Constant24_Value) *
      PX4Controller_ertmodel0827_P.dCmdq +
      (PX4Controller_ertmodel0827_P.dCmdalpha * rtb_Switch3 +
       PX4Controller_ertmodel0827_P.Cm0)) + PX4Controller_ertmodel0827_B.Gain15)
      * (PX4Controller_ertmodel0827_P.S * PX4Controller_ertmodel0827_P.c) *
      rtb_Product1_d;
  } else if (TmpSignalConversionAtphithetaps >
             PX4Controller_ertmodel0827_P.Switch6_Threshold) {
    // Switch: '<S199>/Switch6' incorporates:
    //   Constant: '<S199>/Constant24'
    //   Constant: '<S199>/Constant8'
    //   Gain: '<S199>/Gain11'
    //   Gain: '<S199>/Gain12'
    //   Gain: '<S199>/Gain13'
    //   Gain: '<S199>/Gain14'
    //   Integrator: '<S176>/p,q,r '
    //   Product: '<S199>/Divide3'
    //   Product: '<S199>/Product4'
    //   Sum: '<S199>/Sum17'
    //   Sum: '<S199>/Sum3'

    rtb_sincos_o1_p[1] = ((PX4Controller_ertmodel0827_P.c / 2.0 *
      PX4Controller_ertmodel0827_X.pqr_CSTATE[1] / (rtb_Sqrt +
      PX4Controller_ertmodel0827_P.Constant24_Value) *
      PX4Controller_ertmodel0827_P.dCmdq +
      (PX4Controller_ertmodel0827_P.dCmdalpha * rtb_Switch3 +
       PX4Controller_ertmodel0827_P.Cm0)) + PX4Controller_ertmodel0827_B.Gain15)
      * (PX4Controller_ertmodel0827_P.S * PX4Controller_ertmodel0827_P.c) *
      rtb_Product1_d;
  } else {
    rtb_sincos_o1_p[1] = PX4Controller_ertmodel0827_P.Constant11_Value;
  }

  // End of Switch: '<S199>/Switch11'
  if (rtmIsMajorTimeStep((&PX4Controller_ertmodel0827_M))) {
    // Gain: '<S199>/Gain19'
    PX4Controller_ertmodel0827_B.Gain19 =
      PX4Controller_ertmodel0827_P.dCndsigmaA * rtb_UnitConversion_idx_1;

    // Gain: '<S199>/Gain20'
    PX4Controller_ertmodel0827_B.Gain20 =
      PX4Controller_ertmodel0827_P.dCndsigmaR * rtb_UnitConversion_idx_2;
  }

  // Switch: '<S199>/Switch15' incorporates:
  //   Constant: '<S199>/Constant12'
  //   Constant: '<S199>/Constant14'
  //   Constant: '<S199>/Constant16'
  //   Constant: '<S199>/Constant5'
  //   Gain: '<S199>/Gain18'
  //   Gain: '<S199>/Gain2'
  //   Gain: '<S199>/Gain21'
  //   Gain: '<S199>/Gain22'
  //   Gain: '<S199>/Gain3'
  //   Gain: '<S199>/Gain38'
  //   Integrator: '<S176>/p,q,r '
  //   Product: '<S199>/Divide2'
  //   Product: '<S199>/Divide6'
  //   Product: '<S199>/Product7'
  //   Sum: '<S199>/Sum10'
  //   Sum: '<S199>/Sum5'
  //   Sum: '<S199>/Sum9'
  //   Switch: '<S199>/Switch9'

  if (rtb_Switch1 > PX4Controller_ertmodel0827_P.Switch15_Threshold) {
    rtb_sincos_o1_p[2] = ((((PX4Controller_ertmodel0827_P.b / 2.0 *
      PX4Controller_ertmodel0827_X.pqr_CSTATE[0] / (rtb_Sqrt +
      PX4Controller_ertmodel0827_P.Constant16_Value) *
      PX4Controller_ertmodel0827_P.dCndp +
      (PX4Controller_ertmodel0827_P.dCndbeta * Switch +
       PX4Controller_ertmodel0827_P.Cn0)) + PX4Controller_ertmodel0827_P.b / 2.0
      * PX4Controller_ertmodel0827_X.pqr_CSTATE[2] / (rtb_Sqrt +
      PX4Controller_ertmodel0827_P.Constant5_Value) *
      PX4Controller_ertmodel0827_P.dCndr) + PX4Controller_ertmodel0827_B.Gain19)
                          + PX4Controller_ertmodel0827_B.Gain20) *
      (PX4Controller_ertmodel0827_P.S * PX4Controller_ertmodel0827_P.b) *
      rtb_Product1_d;
  } else if (TmpSignalConversionAtphithetaps >
             PX4Controller_ertmodel0827_P.Switch9_Threshold) {
    // Switch: '<S199>/Switch9' incorporates:
    //   Constant: '<S199>/Constant14'
    //   Constant: '<S199>/Constant16'
    //   Constant: '<S199>/Constant5'
    //   Gain: '<S199>/Gain18'
    //   Gain: '<S199>/Gain2'
    //   Gain: '<S199>/Gain21'
    //   Gain: '<S199>/Gain22'
    //   Gain: '<S199>/Gain3'
    //   Gain: '<S199>/Gain38'
    //   Integrator: '<S176>/p,q,r '
    //   Product: '<S199>/Divide2'
    //   Product: '<S199>/Divide6'
    //   Product: '<S199>/Product7'
    //   Sum: '<S199>/Sum10'
    //   Sum: '<S199>/Sum5'
    //   Sum: '<S199>/Sum9'

    rtb_sincos_o1_p[2] = ((((PX4Controller_ertmodel0827_P.b / 2.0 *
      PX4Controller_ertmodel0827_X.pqr_CSTATE[0] / (rtb_Sqrt +
      PX4Controller_ertmodel0827_P.Constant16_Value) *
      PX4Controller_ertmodel0827_P.dCndp +
      (PX4Controller_ertmodel0827_P.dCndbeta * Switch +
       PX4Controller_ertmodel0827_P.Cn0)) + PX4Controller_ertmodel0827_P.b / 2.0
      * PX4Controller_ertmodel0827_X.pqr_CSTATE[2] / (rtb_Sqrt +
      PX4Controller_ertmodel0827_P.Constant5_Value) *
      PX4Controller_ertmodel0827_P.dCndr) + PX4Controller_ertmodel0827_B.Gain19)
                          + PX4Controller_ertmodel0827_B.Gain20) *
      (PX4Controller_ertmodel0827_P.S * PX4Controller_ertmodel0827_P.b) *
      rtb_Product1_d;
  } else {
    rtb_sincos_o1_p[2] = PX4Controller_ertmodel0827_P.Constant12_Value;
  }

  // End of Switch: '<S199>/Switch15'

  // Sum: '<S201>/Sum1' incorporates:
  //   Constant: '<S203>/Constant2'
  //   Constant: '<S203>/Jmax1'
  //   Constant: '<S203>/Jmax3'
  //   Gain: '<S203>/Gain2'
  //   Gain: '<S203>/Gain3'
  //   Product: '<Root>/AchievableDeltaAlpha'
  //   Product: '<S203>/Divide3'
  //   Product: '<S203>/Product4'
  //   Sum: '<S203>/Sum1'
  //   Trigonometry: '<S190>/sincos'

  rtb_UnitConversion_idx_0 = (PX4Controller_ertmodel0827_P.Jmax3_Value -
    rtb_Divide1 / PX4Controller_ertmodel0827_P.Jmax1_Value) *
    PX4Controller_ertmodel0827_P.k_omega * rtb_Product3_n *
    PX4Controller_ertmodel0827_B.Power2 * PX4Controller_ertmodel0827_B.Square1 *
    -PX4Controller_ertmodel0827_P.kTp + rtb_sincos_o1_p[0];
  rtb_Sum1_f[0] = rtb_UnitConversion_idx_0;
  rtb_UnitConversion_idx_1 = PX4Controller_ertmodel0827_P.Constant2_Value_c +
    rtb_sincos_o1_p[1];
  rtb_Sum1_f[1] = rtb_UnitConversion_idx_1;
  rtb_UnitConversion_idx_2 = PX4Controller_ertmodel0827_P.Constant2_Value_c +
    rtb_sincos_o1_p[2];
  rtb_Sum1_f[2] = rtb_UnitConversion_idx_2;

  // Switch: '<S201>/Switch3'
  if (!(rtb_Gain1_h > PX4Controller_ertmodel0827_P.Switch3_Threshold_l)) {
    // Switch: '<S201>/Switch2' incorporates:
    //   Gain: '<S201>/Gain2'
    //   Gain: '<S201>/Gain3'
    //   Sum: '<S201>/Sum1'

    if (rtb_UnitConversion_idx_1 >
        PX4Controller_ertmodel0827_P.Switch2_Threshold_n) {
      rtb_Sum1_f[0] = rtb_UnitConversion_idx_0 *
        PX4Controller_ertmodel0827_P.Gain2_Gain[0];
      rtb_Sum1_f[1] = PX4Controller_ertmodel0827_P.Gain2_Gain[1] *
        rtb_UnitConversion_idx_1;
      rtb_Sum1_f[2] = rtb_UnitConversion_idx_2 *
        PX4Controller_ertmodel0827_P.Gain2_Gain[2];
    } else {
      rtb_Sum1_f[0] = rtb_UnitConversion_idx_0 *
        PX4Controller_ertmodel0827_P.Gain3_Gain[0];
      rtb_Sum1_f[1] = PX4Controller_ertmodel0827_P.Gain3_Gain[1] *
        rtb_UnitConversion_idx_1;
      rtb_Sum1_f[2] = rtb_UnitConversion_idx_2 *
        PX4Controller_ertmodel0827_P.Gain3_Gain[2];
    }

    // End of Switch: '<S201>/Switch2'
  }

  // End of Switch: '<S201>/Switch3'
  for (i = 0; i < 3; i++) {
    // Product: '<S194>/Product' incorporates:
    //   Integrator: '<S176>/p,q,r '
    //   Selector: '<S182>/Selector1'
    //   Trigonometry: '<S190>/sincos'

    rtb_sincos_o1_p[i] = PX4Controller_ertmodel0827_B.Selector1[i + 6] *
      PX4Controller_ertmodel0827_X.pqr_CSTATE[2] +
      (PX4Controller_ertmodel0827_B.Selector1[i + 3] *
       PX4Controller_ertmodel0827_X.pqr_CSTATE[1] +
       PX4Controller_ertmodel0827_B.Selector1[i] *
       PX4Controller_ertmodel0827_X.pqr_CSTATE[0]);

    // Product: '<S193>/Product' incorporates:
    //   Integrator: '<S176>/p,q,r '
    //   Selector: '<S182>/Selector'
    //   Sum: '<S192>/Sum'

    rtb_Sum_c0[i] = PX4Controller_ertmodel0827_B.Selector[i + 6] *
      PX4Controller_ertmodel0827_X.pqr_CSTATE[2] +
      (PX4Controller_ertmodel0827_B.Selector[i + 3] *
       PX4Controller_ertmodel0827_X.pqr_CSTATE[1] +
       PX4Controller_ertmodel0827_B.Selector[i] *
       PX4Controller_ertmodel0827_X.pqr_CSTATE[0]);
  }

  // Sum: '<S182>/Sum2' incorporates:
  //   Integrator: '<S176>/p,q,r '
  //   Product: '<S195>/j x k'
  //   Product: '<S196>/k x j'
  //   Sum: '<S192>/Sum'

  rtb_Sum1_h[0] = (rtb_Sum1_f[0] - rtb_sincos_o1_p[0]) -
    (PX4Controller_ertmodel0827_X.pqr_CSTATE[1] * rtb_Sum_c0[2] - rtb_Sum_c0[1] *
     PX4Controller_ertmodel0827_X.pqr_CSTATE[2]);

  // Sum: '<Root>/SurfaceClipResidual'
  rtb_V_windtoBody[0] = NormalizedSurfaceLimit[0] - rtb_V_windtoBody[0];

  // Sum: '<S182>/Sum2' incorporates:
  //   Integrator: '<S176>/p,q,r '
  //   Product: '<S195>/k x i'
  //   Product: '<S196>/i x k'
  //   Sum: '<S192>/Sum'

  rtb_Sum1_h[1] = (rtb_Sum1_f[1] - rtb_sincos_o1_p[1]) - (rtb_Sum_c0[0] *
    PX4Controller_ertmodel0827_X.pqr_CSTATE[2] -
    PX4Controller_ertmodel0827_X.pqr_CSTATE[0] * rtb_Sum_c0[2]);

  // Sum: '<Root>/SurfaceClipResidual'
  rtb_V_windtoBody[1] = NormalizedSurfaceLimit[1] - rtb_V_windtoBody[1];

  // Sum: '<S182>/Sum2' incorporates:
  //   Integrator: '<S176>/p,q,r '
  //   Product: '<S195>/i x j'
  //   Product: '<S196>/j x i'
  //   Sum: '<S192>/Sum'

  rtb_Sum1_h[2] = (rtb_Sum1_f[2] - rtb_sincos_o1_p[2]) -
    (PX4Controller_ertmodel0827_X.pqr_CSTATE[0] * rtb_Sum_c0[1] - rtb_Sum_c0[0] *
     PX4Controller_ertmodel0827_X.pqr_CSTATE[1]);

  // Sum: '<Root>/SurfaceClipResidual'
  rtb_V_windtoBody[2] = NormalizedSurfaceLimit[2] - rtb_V_windtoBody[2];

  // Product: '<S182>/Product2' incorporates:
  //   Selector: '<S182>/Selector2'

  rt_mrdivide_U1d1x3_U2d_9vOrDY9Z(rtb_Sum1_h,
    PX4Controller_ertmodel0827_B.Selector2,
    PX4Controller_ertmodel0827_B.Product2);
  for (i = 0; i < 3; i++) {
    // Product: '<S188>/Product' incorporates:
    //   Concatenate: '<S191>/Vector Concatenate'
    //   Integrator: '<S176>/ub,vb,wb'
    //   Math: '<S176>/Transpose'

    PX4Controller_ertmodel0827_B.Product_o[i] = 0.0;
    PX4Controller_ertmodel0827_B.Product_o[i] += rtb_VectorConcatenate_o[3 * i] *
      PX4Controller_ertmodel0827_X.ubvbwb_CSTATE[0];
    PX4Controller_ertmodel0827_B.Product_o[i] += rtb_VectorConcatenate_o[3 * i +
      1] * PX4Controller_ertmodel0827_X.ubvbwb_CSTATE[1];
    PX4Controller_ertmodel0827_B.Product_o[i] += rtb_VectorConcatenate_o[3 * i +
      2] * PX4Controller_ertmodel0827_X.ubvbwb_CSTATE[2];

    // Product: '<Root>/AchievableDeltaAlpha' incorporates:
    //   Product: '<Root>/MixerInverseForAW'

    rtb_Sum1_f[i] = PX4Controller_ertmodel0827_B.MixerInverseForAW[i + 6] *
      rtb_V_windtoBody[2] + (PX4Controller_ertmodel0827_B.MixerInverseForAW[i +
      3] * rtb_V_windtoBody[1] +
      PX4Controller_ertmodel0827_B.MixerInverseForAW[i] * rtb_V_windtoBody[0]);
  }

  // Fcn: '<S190>/phidot' incorporates:
  //   Fcn: '<S190>/psidot'
  //   Integrator: '<S176>/p,q,r '

  TmpSignalConversionAtphithetaps = PX4Controller_ertmodel0827_X.pqr_CSTATE[1] *
    rtb_V_windtoBody_tmp + PX4Controller_ertmodel0827_X.pqr_CSTATE[2] *
    rtb_sincos_o2_idx_2_tmp;

  // SignalConversion generated from: '<S181>/phi theta psi' incorporates:
  //   Fcn: '<S190>/phidot'
  //   Fcn: '<S190>/psidot'
  //   Fcn: '<S190>/thetadot'
  //   Integrator: '<S176>/p,q,r '

  PX4Controller_ertmodel0827_B.TmpSignalConversionAtphithetaps[0] =
    TmpSignalConversionAtphithetaps * (rtb_V_windtoBody_tmp_0 / Selector_tmp) +
    PX4Controller_ertmodel0827_X.pqr_CSTATE[0];
  PX4Controller_ertmodel0827_B.TmpSignalConversionAtphithetaps[1] =
    rtb_sincos_o2_idx_2_tmp * PX4Controller_ertmodel0827_X.pqr_CSTATE[1] -
    PX4Controller_ertmodel0827_X.pqr_CSTATE[2] * rtb_V_windtoBody_tmp;
  PX4Controller_ertmodel0827_B.TmpSignalConversionAtphithetaps[2] =
    TmpSignalConversionAtphithetaps / Selector_tmp;

  // Unit Conversion - from: rad to: deg
  // Expression: output = (57.2958*input) + (0)
  if (rtmIsMajorTimeStep((&PX4Controller_ertmodel0827_M))) {
    // Outport: '<Root>/POS_ned' incorporates:
    //   Gain: '<S175>/Gain'
    //   Gain: '<S175>/Gain1'
    //   Integrator: '<S176>/xe,ye,ze'
    //   Integrator: '<S181>/phi theta psi'
    //   UnitConversion: '<S179>/Unit Conversion'
    //   ZeroOrderHold generated from: '<S5>/CodegenOutputHold_NED'

    PX4Controller_ertmodel0827_Y.POS_ned[0] =
      PX4Controller_ertmodel0827_X.xeyeze_CSTATE[0];
    PX4Controller_ertmodel0827_Y.POS_ned[1] =
      PX4Controller_ertmodel0827_X.xeyeze_CSTATE[1];
    PX4Controller_ertmodel0827_Y.POS_ned[2] =
      PX4Controller_ertmodel0827_P.Gain_Gain_ca *
      PX4Controller_ertmodel0827_X.xeyeze_CSTATE[2];
    PX4Controller_ertmodel0827_Y.POS_ned[3] =
      PX4Controller_ertmodel0827_B.Product_o[0];
    PX4Controller_ertmodel0827_Y.POS_ned[4] =
      PX4Controller_ertmodel0827_B.Product_o[1];
    PX4Controller_ertmodel0827_Y.POS_ned[5] =
      PX4Controller_ertmodel0827_P.Gain1_Gain_fx *
      PX4Controller_ertmodel0827_B.Product_o[2];
    PX4Controller_ertmodel0827_Y.POS_ned[6] = rtb_phi;
    PX4Controller_ertmodel0827_Y.POS_ned[7] = rtb_theta;
    PX4Controller_ertmodel0827_Y.POS_ned[8] = 57.295779513082323 *
      PX4Controller_ertmodel0827_X.phithetapsi_CSTATE[2];

    // Sum: '<S46>/SumI4' incorporates:
    //   Constant: '<S9>/aw_eps'
    //   Gain: '<S46>/Kb'
    //   Gain: '<S50>/Integral Gain'
    //   Gain: '<S64>/Kt'
    //   Gain: '<S9>/aw_inv_Vref2'
    //   Product: '<S9>/aw_unscale_PI'
    //   Sum: '<S46>/SumI2'
    //   Sum: '<S64>/SumI3'
    //   Sum: '<S65>/SumI1'
    //   Sum: '<S9>/aw_V2_plus_eps'
    //   Sum: '<S9>/aw_pid_tracking_sum'

    rtb_SumI4 = ((((rtb_Product_i + PX4Controller_ertmodel0827_P.aw_eps_Value) *
                   PX4Controller_ertmodel0827_P.aw_inv_Vref2_Gain * rtb_Sum1_f[0]
                   + PX4Controller_ertmodel0827_B.Saturation) -
                  PX4Controller_ertmodel0827_B.Saturation) *
                 PX4Controller_ertmodel0827_P.DiscretePDController_Kt +
                 PX4Controller_ertmodel0827_P.DiscretePDController_I *
                 rtb_sincos_o2_idx_0) + (PX4Controller_ertmodel0827_B.Saturation
      - rtb_Sum_db) * PX4Controller_ertmodel0827_P.DiscretePDController_Kb;

    // Sum: '<S96>/SumI4' incorporates:
    //   Constant: '<S10>/aw_eps'
    //   Gain: '<S100>/Integral Gain'
    //   Gain: '<S10>/aw_inv_Vref2'
    //   Gain: '<S114>/Kt'
    //   Gain: '<S96>/Kb'
    //   Product: '<S10>/aw_unscale_PI'
    //   Sum: '<S10>/aw_V2_plus_eps'
    //   Sum: '<S10>/aw_pid_tracking_sum'
    //   Sum: '<S114>/SumI3'
    //   Sum: '<S115>/SumI1'
    //   Sum: '<S96>/SumI2'

    rtb_SumI4_m = ((((rtb_Product_n +
                      PX4Controller_ertmodel0827_P.aw_eps_Value_l) *
                     PX4Controller_ertmodel0827_P.aw_inv_Vref2_Gain_f *
                     rtb_Sum1_f[2] + PX4Controller_ertmodel0827_B.Saturation_n)
                    - PX4Controller_ertmodel0827_B.Saturation_n) *
                   PX4Controller_ertmodel0827_P.DiscretePDController_Kt_k +
                   PX4Controller_ertmodel0827_P.DiscretePDController_I_a *
                   rtb_sincos_o2_e_idx_1) +
      (PX4Controller_ertmodel0827_B.Saturation_n - rtb_v_dot) *
      PX4Controller_ertmodel0827_P.DiscretePDController_Kb_p;

    // Sum: '<S146>/SumI4' incorporates:
    //   Constant: '<S11>/aw_eps'
    //   Gain: '<S11>/aw_inv_Vref2'
    //   Gain: '<S146>/Kb'
    //   Gain: '<S150>/Integral Gain'
    //   Gain: '<S164>/Kt'
    //   Product: '<S11>/aw_unscale_PI'
    //   Sum: '<S11>/aw_V2_plus_eps'
    //   Sum: '<S11>/aw_pid_tracking_sum'
    //   Sum: '<S146>/SumI2'
    //   Sum: '<S164>/SumI3'
    //   Sum: '<S165>/SumI1'

    rtb_SumI4_b = ((((rtb_Product_hi +
                      PX4Controller_ertmodel0827_P.aw_eps_Value_f) *
                     PX4Controller_ertmodel0827_P.aw_inv_Vref2_Gain_d *
                     rtb_Sum1_f[1] + PX4Controller_ertmodel0827_B.Saturation_e)
                    - PX4Controller_ertmodel0827_B.Saturation_e) *
                   PX4Controller_ertmodel0827_P.DiscretePDController_Kt_g +
                   PX4Controller_ertmodel0827_P.DiscretePDController_I_c *
                   rtb_Integrator_e) +
      (PX4Controller_ertmodel0827_B.Saturation_e - rtb_w_dot) *
      PX4Controller_ertmodel0827_P.DiscretePDController_Kb_l;

    // Outport: '<Root>/STATE_body' incorporates:
    //   Integrator: '<S176>/ub,vb,wb'
    //   ZeroOrderHold generated from: '<S5>/CodegenOutputHold_BODY'

    PX4Controller_ertmodel0827_Y.STATE_body[0] =
      PX4Controller_ertmodel0827_X.ubvbwb_CSTATE[0];
    PX4Controller_ertmodel0827_Y.STATE_body[1] =
      PX4Controller_ertmodel0827_X.ubvbwb_CSTATE[1];
    PX4Controller_ertmodel0827_Y.STATE_body[2] =
      PX4Controller_ertmodel0827_X.ubvbwb_CSTATE[2];
    PX4Controller_ertmodel0827_Y.STATE_body[3] = rtb_p;
    PX4Controller_ertmodel0827_Y.STATE_body[4] = rtb_q;
    PX4Controller_ertmodel0827_Y.STATE_body[5] = rtb_r;
    PX4Controller_ertmodel0827_Y.STATE_body[6] =
      PX4Controller_ertmodel0827_B.Product2[0];
    PX4Controller_ertmodel0827_Y.STATE_body[7] =
      PX4Controller_ertmodel0827_B.Product2[1];
    PX4Controller_ertmodel0827_Y.STATE_body[8] =
      PX4Controller_ertmodel0827_B.Product2[2];
    PX4Controller_ertmodel0827_Y.STATE_body[9] =
      PX4Controller_ertmodel0827_B.Sum[0];
    PX4Controller_ertmodel0827_Y.STATE_body[10] =
      PX4Controller_ertmodel0827_B.Sum[1];
    PX4Controller_ertmodel0827_Y.STATE_body[11] =
      PX4Controller_ertmodel0827_B.Sum[2];

    // Outport: '<Root>/beta' incorporates:
    //   ZeroOrderHold: '<S5>/CodegenOutputHold_beta'

    PX4Controller_ertmodel0827_Y.beta = Switch;

    // Outport: '<Root>/alpha' incorporates:
    //   ZeroOrderHold: '<S5>/CodegenOutputHold_alpha'

    PX4Controller_ertmodel0827_Y.alpha = rtb_Switch3;

    // Assertion: '<S172>/Assertion' incorporates:
    //   Product: '<S174>/Product'
    //   Product: '<S174>/Product1'
    //   Product: '<S174>/Product2'
    //   Product: '<S174>/Product3'
    //   Product: '<S174>/Product4'
    //   Product: '<S174>/Product5'
    //   Sum: '<S174>/Sum'

    utAssert(((((rtb_VectorConcatenate_a[0] * rtb_VectorConcatenate_a[4] *
                 rtb_VectorConcatenate_a[8] - rtb_VectorConcatenate_a[0] *
                 rtb_VectorConcatenate_a[5] * rtb_VectorConcatenate_a[7]) -
                rtb_VectorConcatenate_a[1] * rtb_VectorConcatenate_a[3] *
                rtb_VectorConcatenate_a[8]) + rtb_VectorConcatenate_a[2] *
               rtb_VectorConcatenate_a[3] * rtb_VectorConcatenate_a[7]) +
              rtb_VectorConcatenate_a[1] * rtb_VectorConcatenate_a[5] *
              rtb_VectorConcatenate_a[6]) - rtb_VectorConcatenate_a[2] *
             rtb_VectorConcatenate_a[4] * rtb_VectorConcatenate_a[6] != 0.0);
  }

  if (rtmIsMajorTimeStep((&PX4Controller_ertmodel0827_M))) {
    if (rtmIsMajorTimeStep((&PX4Controller_ertmodel0827_M))) {
      // Update for DiscreteIntegrator: '<S53>/Integrator'
      PX4Controller_ertmodel0827_DW.Integrator_DSTATE +=
        PX4Controller_ertmodel0827_P.Integrator_gainval * rtb_SumI4;

      // Update for DiscreteIntegrator: '<S48>/Filter'
      PX4Controller_ertmodel0827_DW.Filter_DSTATE +=
        PX4Controller_ertmodel0827_P.Filter_gainval * rtb_FilterCoefficient;

      // Update for DiscreteIntegrator: '<S153>/Integrator'
      PX4Controller_ertmodel0827_DW.Integrator_DSTATE_b +=
        PX4Controller_ertmodel0827_P.Integrator_gainval_j * rtb_SumI4_b;

      // Update for DiscreteIntegrator: '<S148>/Filter'
      PX4Controller_ertmodel0827_DW.Filter_DSTATE_j +=
        PX4Controller_ertmodel0827_P.Filter_gainval_f * rtb_FilterCoefficient_h;

      // Update for DiscreteIntegrator: '<S103>/Integrator'
      PX4Controller_ertmodel0827_DW.Integrator_DSTATE_e +=
        PX4Controller_ertmodel0827_P.Integrator_gainval_b * rtb_SumI4_m;

      // Update for DiscreteIntegrator: '<S98>/Filter'
      PX4Controller_ertmodel0827_DW.Filter_DSTATE_n +=
        PX4Controller_ertmodel0827_P.Filter_gainval_c * rtb_FilterCoefficient_p;
    }
  }                                    // end MajorTimeStep

  if (rtmIsMajorTimeStep((&PX4Controller_ertmodel0827_M))) {
    rt_ertODEUpdateContinuousStates(&(&PX4Controller_ertmodel0827_M)->solverInfo);

    // Update absolute time for base rate
    // The "clockTick0" counts the number of times the code of this task has
    //  been executed. The absolute time is the multiplication of "clockTick0"
    //  and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
    //  overflow during the application lifespan selected.

    ++(&PX4Controller_ertmodel0827_M)->Timing.clockTick0;
    (&PX4Controller_ertmodel0827_M)->Timing.t[0] = rtsiGetSolverStopTime
      (&(&PX4Controller_ertmodel0827_M)->solverInfo);

    {
      // Update absolute timer for sample time: [0.02s, 0.0s]
      // The "clockTick1" counts the number of times the code of this task has
      //  been executed. The resolution of this integer timer is 0.02, which is the step size
      //  of the task. Size of "clockTick1" ensures timer will not overflow during the
      //  application lifespan selected.

      (&PX4Controller_ertmodel0827_M)->Timing.clockTick1++;
    }
  }                                    // end MajorTimeStep
}

// Derivatives for root system: '<Root>'
void PX4Controller_ertmodel0827ModelClass::
  PX4Controller_ertmodel0827_derivatives()
{
  PX4Controller_ertmodel0827ModelClass::XDot_PX4Controller_ertmodel08_T *_rtXdot;
  _rtXdot = ((XDot_PX4Controller_ertmodel08_T *) (&PX4Controller_ertmodel0827_M
             )->derivs);

  // Derivatives for Integrator: '<S181>/phi theta psi'
  _rtXdot->phithetapsi_CSTATE[0] =
    PX4Controller_ertmodel0827_B.TmpSignalConversionAtphithetaps[0];

  // Derivatives for Integrator: '<S176>/ub,vb,wb'
  _rtXdot->ubvbwb_CSTATE[0] = PX4Controller_ertmodel0827_B.Sum[0];

  // Derivatives for Integrator: '<S176>/xe,ye,ze'
  _rtXdot->xeyeze_CSTATE[0] = PX4Controller_ertmodel0827_B.Product_o[0];

  // Derivatives for Integrator: '<S176>/p,q,r '
  _rtXdot->pqr_CSTATE[0] = PX4Controller_ertmodel0827_B.Product2[0];

  // Derivatives for Integrator: '<S181>/phi theta psi'
  _rtXdot->phithetapsi_CSTATE[1] =
    PX4Controller_ertmodel0827_B.TmpSignalConversionAtphithetaps[1];

  // Derivatives for Integrator: '<S176>/ub,vb,wb'
  _rtXdot->ubvbwb_CSTATE[1] = PX4Controller_ertmodel0827_B.Sum[1];

  // Derivatives for Integrator: '<S176>/xe,ye,ze'
  _rtXdot->xeyeze_CSTATE[1] = PX4Controller_ertmodel0827_B.Product_o[1];

  // Derivatives for Integrator: '<S176>/p,q,r '
  _rtXdot->pqr_CSTATE[1] = PX4Controller_ertmodel0827_B.Product2[1];

  // Derivatives for Integrator: '<S181>/phi theta psi'
  _rtXdot->phithetapsi_CSTATE[2] =
    PX4Controller_ertmodel0827_B.TmpSignalConversionAtphithetaps[2];

  // Derivatives for Integrator: '<S176>/ub,vb,wb'
  _rtXdot->ubvbwb_CSTATE[2] = PX4Controller_ertmodel0827_B.Sum[2];

  // Derivatives for Integrator: '<S176>/xe,ye,ze'
  _rtXdot->xeyeze_CSTATE[2] = PX4Controller_ertmodel0827_B.Product_o[2];

  // Derivatives for Integrator: '<S176>/p,q,r '
  _rtXdot->pqr_CSTATE[2] = PX4Controller_ertmodel0827_B.Product2[2];
}

// Model initialize function
void PX4Controller_ertmodel0827ModelClass::initialize()
{
  // Registration code

  // initialize non-finites
  rt_InitInfAndNaN(sizeof(real_T));

  {
    // Setup solver object
    rtsiSetSimTimeStepPtr(&(&PX4Controller_ertmodel0827_M)->solverInfo,
                          &(&PX4Controller_ertmodel0827_M)->Timing.simTimeStep);
    rtsiSetTPtr(&(&PX4Controller_ertmodel0827_M)->solverInfo, &rtmGetTPtr
                ((&PX4Controller_ertmodel0827_M)));
    rtsiSetStepSizePtr(&(&PX4Controller_ertmodel0827_M)->solverInfo,
                       &(&PX4Controller_ertmodel0827_M)->Timing.stepSize0);
    rtsiSetdXPtr(&(&PX4Controller_ertmodel0827_M)->solverInfo,
                 &(&PX4Controller_ertmodel0827_M)->derivs);
    rtsiSetContStatesPtr(&(&PX4Controller_ertmodel0827_M)->solverInfo, (real_T **)
                         &(&PX4Controller_ertmodel0827_M)->contStates);
    rtsiSetNumContStatesPtr(&(&PX4Controller_ertmodel0827_M)->solverInfo,
      &(&PX4Controller_ertmodel0827_M)->Sizes.numContStates);
    rtsiSetNumPeriodicContStatesPtr(&(&PX4Controller_ertmodel0827_M)->solverInfo,
      &(&PX4Controller_ertmodel0827_M)->Sizes.numPeriodicContStates);
    rtsiSetPeriodicContStateIndicesPtr(&(&PX4Controller_ertmodel0827_M)
      ->solverInfo, &(&PX4Controller_ertmodel0827_M)->periodicContStateIndices);
    rtsiSetPeriodicContStateRangesPtr(&(&PX4Controller_ertmodel0827_M)
      ->solverInfo, &(&PX4Controller_ertmodel0827_M)->periodicContStateRanges);
    rtsiSetErrorStatusPtr(&(&PX4Controller_ertmodel0827_M)->solverInfo,
                          (&rtmGetErrorStatus((&PX4Controller_ertmodel0827_M))));
    rtsiSetRTModelPtr(&(&PX4Controller_ertmodel0827_M)->solverInfo,
                      (&PX4Controller_ertmodel0827_M));
  }

  rtsiSetSimTimeStep(&(&PX4Controller_ertmodel0827_M)->solverInfo,
                     MAJOR_TIME_STEP);
  (&PX4Controller_ertmodel0827_M)->intgData.y = (&PX4Controller_ertmodel0827_M
    )->odeY;
  (&PX4Controller_ertmodel0827_M)->intgData.f[0] =
    (&PX4Controller_ertmodel0827_M)->odeF[0];
  (&PX4Controller_ertmodel0827_M)->intgData.f[1] =
    (&PX4Controller_ertmodel0827_M)->odeF[1];
  (&PX4Controller_ertmodel0827_M)->intgData.f[2] =
    (&PX4Controller_ertmodel0827_M)->odeF[2];
  (&PX4Controller_ertmodel0827_M)->intgData.f[3] =
    (&PX4Controller_ertmodel0827_M)->odeF[3];
  (&PX4Controller_ertmodel0827_M)->contStates = ((X_PX4Controller_ertmodel0827_T
    *) &PX4Controller_ertmodel0827_X);
  (&PX4Controller_ertmodel0827_M)->periodicContStateIndices = ((int_T*)
    PX4Controller_ertm_PeriodicIndX);
  (&PX4Controller_ertmodel0827_M)->periodicContStateRanges = ((real_T*)
    PX4Controller_ertm_PeriodicRngX);
  rtsiSetSolverData(&(&PX4Controller_ertmodel0827_M)->solverInfo, static_cast<
                    void *>(&(&PX4Controller_ertmodel0827_M)->intgData));
  rtsiSetSolverName(&(&PX4Controller_ertmodel0827_M)->solverInfo,"ode4");
  rtmSetTPtr((&PX4Controller_ertmodel0827_M), &(&PX4Controller_ertmodel0827_M)
             ->Timing.tArray[0]);
  (&PX4Controller_ertmodel0827_M)->Timing.stepSize0 = 0.02;

  // InitializeConditions for Integrator: '<S181>/phi theta psi'
  PX4Controller_ertmodel0827_X.phithetapsi_CSTATE[0] =
    PX4Controller_ertmodel0827_P.init_phi_theta_psi[0];

  // InitializeConditions for Integrator: '<S176>/ub,vb,wb'
  PX4Controller_ertmodel0827_X.ubvbwb_CSTATE[0] =
    PX4Controller_ertmodel0827_P.init_u_v_w[0];

  // InitializeConditions for Integrator: '<S176>/xe,ye,ze'
  PX4Controller_ertmodel0827_X.xeyeze_CSTATE[0] =
    PX4Controller_ertmodel0827_P.init_Xe_Ye_Ze[0];

  // InitializeConditions for Integrator: '<S176>/p,q,r '
  PX4Controller_ertmodel0827_X.pqr_CSTATE[0] =
    PX4Controller_ertmodel0827_P.init_p_q_r[0];

  // InitializeConditions for Integrator: '<S181>/phi theta psi'
  PX4Controller_ertmodel0827_X.phithetapsi_CSTATE[1] =
    PX4Controller_ertmodel0827_P.init_phi_theta_psi[1];

  // InitializeConditions for Integrator: '<S176>/ub,vb,wb'
  PX4Controller_ertmodel0827_X.ubvbwb_CSTATE[1] =
    PX4Controller_ertmodel0827_P.init_u_v_w[1];

  // InitializeConditions for Integrator: '<S176>/xe,ye,ze'
  PX4Controller_ertmodel0827_X.xeyeze_CSTATE[1] =
    PX4Controller_ertmodel0827_P.init_Xe_Ye_Ze[1];

  // InitializeConditions for Integrator: '<S176>/p,q,r '
  PX4Controller_ertmodel0827_X.pqr_CSTATE[1] =
    PX4Controller_ertmodel0827_P.init_p_q_r[1];

  // InitializeConditions for Integrator: '<S181>/phi theta psi'
  PX4Controller_ertmodel0827_X.phithetapsi_CSTATE[2] =
    PX4Controller_ertmodel0827_P.init_phi_theta_psi[2];

  // InitializeConditions for Integrator: '<S176>/ub,vb,wb'
  PX4Controller_ertmodel0827_X.ubvbwb_CSTATE[2] =
    PX4Controller_ertmodel0827_P.init_u_v_w[2];

  // InitializeConditions for Integrator: '<S176>/xe,ye,ze'
  PX4Controller_ertmodel0827_X.xeyeze_CSTATE[2] =
    PX4Controller_ertmodel0827_P.init_Xe_Ye_Ze[2];

  // InitializeConditions for Integrator: '<S176>/p,q,r '
  PX4Controller_ertmodel0827_X.pqr_CSTATE[2] =
    PX4Controller_ertmodel0827_P.init_p_q_r[2];

  // InitializeConditions for DiscreteIntegrator: '<S53>/Integrator'
  PX4Controller_ertmodel0827_DW.Integrator_DSTATE =
    PX4Controller_ertmodel0827_P.DiscretePDController_Initial_hm;

  // InitializeConditions for DiscreteIntegrator: '<S48>/Filter'
  PX4Controller_ertmodel0827_DW.Filter_DSTATE =
    PX4Controller_ertmodel0827_P.DiscretePDController_InitialCon;

  // InitializeConditions for DiscreteIntegrator: '<S153>/Integrator'
  PX4Controller_ertmodel0827_DW.Integrator_DSTATE_b =
    PX4Controller_ertmodel0827_P.DiscretePDController_InitialC_b;

  // InitializeConditions for DiscreteIntegrator: '<S148>/Filter'
  PX4Controller_ertmodel0827_DW.Filter_DSTATE_j =
    PX4Controller_ertmodel0827_P.DiscretePDController_InitialC_h;

  // InitializeConditions for DiscreteIntegrator: '<S103>/Integrator'
  PX4Controller_ertmodel0827_DW.Integrator_DSTATE_e =
    PX4Controller_ertmodel0827_P.DiscretePDController_Initial_bz;

  // InitializeConditions for DiscreteIntegrator: '<S98>/Filter'
  PX4Controller_ertmodel0827_DW.Filter_DSTATE_n =
    PX4Controller_ertmodel0827_P.DiscretePDController_InitialC_a;

  // InitializeConditions for root-level periodic continuous states
  {
    int_T rootPeriodicContStateIndices[3] = { 0, 1, 2 };

    real_T rootPeriodicContStateRanges[6] = { -3.1415926535897931,
      3.1415926535897931, -3.1415926535897931, 3.1415926535897931,
      -3.1415926535897931, 3.1415926535897931 };

    (void) std::memcpy((void*)PX4Controller_ertm_PeriodicIndX,
                       rootPeriodicContStateIndices,
                       3*sizeof(int_T));
    (void) std::memcpy((void*)PX4Controller_ertm_PeriodicRngX,
                       rootPeriodicContStateRanges,
                       6*sizeof(real_T));
  }
}

// Model terminate function
void PX4Controller_ertmodel0827ModelClass::terminate()
{
  // (no terminate code required)
}

// Constructor
PX4Controller_ertmodel0827ModelClass::PX4Controller_ertmodel0827ModelClass() :
  PX4Controller_ertmodel0827_U(),
  PX4Controller_ertmodel0827_Y(),
  PX4Controller_ertmodel0827_B(),
  PX4Controller_ertmodel0827_DW(),
  PX4Controller_ertmodel0827_X(),
  PX4Controller_ertm_PeriodicIndX(),
  PX4Controller_ertm_PeriodicRngX(),
  PX4Controller_ertmodel0827_M()
{
  // Currently there is no constructor body generated.
}

// Destructor
PX4Controller_ertmodel0827ModelClass::~PX4Controller_ertmodel0827ModelClass()
{
  // Currently there is no destructor body generated.
}

// Real-Time Model get method
PX4Controller_ertmodel0827ModelClass::RT_MODEL_PX4Controller_ertmod_T
  * PX4Controller_ertmodel0827ModelClass::getRTM()
{
  return (&PX4Controller_ertmodel0827_M);
}

//
// File trailer for generated code.
//
// [EOF]
//
