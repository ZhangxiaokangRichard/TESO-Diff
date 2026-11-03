//
// File: PX4Controller_ertmodel0827_data.cpp
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

// Block parameters (default storage)
PX4Controller_ertmodel0827ModelClass::P_PX4Controller_ertmodel0827_T
  PX4Controller_ertmodel0827ModelClass::PX4Controller_ertmodel0827_P = {
  // Variable: CD0
  //  Referenced by: '<S199>/Constant2'

  0.017,

  // Variable: CL0
  //  Referenced by:
  //    '<S199>/Constant3'
  //    '<S199>/Constant4'

  0.6483,

  // Variable: Cl0
  //  Referenced by: '<S199>/Constant7'

  0.0,

  // Variable: Cm0
  //  Referenced by: '<S199>/Constant8'

  -0.07792,

  // Variable: Cn0
  //  Referenced by: '<S199>/Constant14'

  0.0,

  // Variable: Cy0
  //  Referenced by: '<S199>/Constant17'

  0.0,

  // Variable: Ix
  //  Referenced by: '<S4>/Constant'

  1.6931,

  // Variable: Iy
  //  Referenced by: '<S4>/Constant2'

  1.0708,

  // Variable: Iz
  //  Referenced by: '<S4>/Constant3'

  2.7281,

  // Variable: S
  //  Referenced by:
  //    '<S171>/Gain1'
  //    '<S199>/Gain1'
  //    '<S199>/Gain11'
  //    '<S199>/Gain21'
  //    '<S199>/Gain29'
  //    '<S199>/Gain42'
  //    '<S199>/Gain47'

  0.957,

  // Variable: Sprop
  //  Referenced by: '<S203>/Constant3'

  0.0993,

  // Variable: V_I0
  //  Referenced by:
  //    '<S171>/Constant'
  //    '<S21>/V_I0'
  //    '<S71>/V_I0'
  //    '<S121>/V_I0'

  16.0,

  // Variable: V_t0
  //  Referenced by:
  //    '<S14>/Vt0'
  //    '<S16>/Vt0'
  //    '<S18>/Vt0'

  16.0,

  // Variable: a_lim
  //  Referenced by: '<S219>/Command Saturation'

  10.0,

  // Variable: alpha_0
  //  Referenced by: '<S199>/Fixed_Angle_of_Attack'

  0.0,

  // Variable: b
  //  Referenced by:
  //    '<S4>/Gain'
  //    '<S4>/Gain1'
  //    '<S4>/Gain3'
  //    '<S4>/Gain4'
  //    '<S199>/Gain17'
  //    '<S199>/Gain2'
  //    '<S199>/Gain21'
  //    '<S199>/Gain24'
  //    '<S199>/Gain3'
  //    '<S199>/Gain37'
  //    '<S199>/Gain4'
  //    '<S199>/Gain47'

  4.32,

  // Variable: c
  //  Referenced by:
  //    '<S4>/Gain2'
  //    '<S199>/Gain11'
  //    '<S199>/Gain14'
  //    '<S199>/Gain9'

  0.22152777777777777,

  // Variable: dCDdsigmaE
  //  Referenced by: '<S199>/Gain46'

  0.2925,

  // Variable: dCLdalpha
  //  Referenced by:
  //    '<S199>/Gain43'
  //    '<S199>/Gain7'

  6.4854,

  // Variable: dCLdq
  //  Referenced by: '<S199>/Gain8'

  11.5476,

  // Variable: dCLdsigmaE
  //  Referenced by: '<S199>/Gain10'

  0.4561,

  // Variable: dCldbeta
  //  Referenced by: '<S199>/Gain5'

  -0.1797,

  // Variable: dCldp
  //  Referenced by: '<S199>/Gain49'

  -0.7261,

  // Variable: dCldr
  //  Referenced by: '<S199>/Gain48'

  0.186,

  // Variable: dCldsigmaA
  //  Referenced by:
  //    '<S4>/Gain'
  //    '<S199>/Gain6'

  -0.3131,

  // Variable: dCldsigmaR
  //  Referenced by:
  //    '<S4>/Gain1'
  //    '<S199>/Gain39'

  -0.00518,

  // Variable: dCmdalpha
  //  Referenced by: '<S199>/Gain12'

  -1.6427,

  // Variable: dCmdq
  //  Referenced by: '<S199>/Gain13'

  -23.5033,

  // Variable: dCmdsigmaE
  //  Referenced by:
  //    '<S4>/Gain2'
  //    '<S199>/Gain15'

  -2.1075,

  // Variable: dCndbeta
  //  Referenced by: '<S199>/Gain18'

  0.0267,

  // Variable: dCndp
  //  Referenced by: '<S199>/Gain38'

  -0.06726,

  // Variable: dCndr
  //  Referenced by: '<S199>/Gain22'

  0.03438,

  // Variable: dCndsigmaA
  //  Referenced by:
  //    '<S4>/Gain3'
  //    '<S199>/Gain19'

  -0.0124,

  // Variable: dCndsigmaR
  //  Referenced by:
  //    '<S4>/Gain4'
  //    '<S199>/Gain20'

  0.04098,

  // Variable: dCydbeta
  //  Referenced by: '<S199>/Gain28'

  -0.2527,

  // Variable: dCydp
  //  Referenced by: '<S199>/Gain35'

  -0.2929,

  // Variable: dCydr
  //  Referenced by: '<S199>/Gain30'

  0.1497,

  // Variable: dCydsigmaA
  //  Referenced by: '<S199>/Gain36'

  -0.1358,

  // Variable: dCydsigmaR
  //  Referenced by: '<S199>/Gain27'

  -0.1472,

  // Variable: e_lim
  //  Referenced by: '<S220>/Command Saturation'

  12.0,

  // Variable: init_Xe_Ye_Ze
  //  Referenced by: '<S176>/xe,ye,ze'

  { 0.0, 0.0, -100.0 },

  // Variable: init_p_q_r
  //  Referenced by: '<S176>/p,q,r '

  { 0.0, 0.0, 0.0 },

  // Variable: init_phi_theta_psi
  //  Referenced by: '<S181>/phi theta psi'

  { 0.0, 0.0068558, 0.0 },

  // Variable: init_u_v_w
  //  Referenced by: '<S176>/ub,vb,wb'

  { 13.4892, 0.0, 0.09248 },

  // Variable: kTp
  //  Referenced by: '<S203>/Gain3'

  0.0,

  // Variable: k_motor
  //  Referenced by: '<S203>/Gain1'

  0.2,

  // Variable: k_omega
  //  Referenced by: '<S203>/Gain2'

  0.0,

  // Variable: m
  //  Referenced by:
  //    '<S180>/Constant'
  //    '<S183>/Constant'

  6.8,

  // Variable: omega_max
  //  Referenced by: '<S203>/Gain4'

  900.0,

  // Mask Parameter: DiscretePDController_D
  //  Referenced by: '<S47>/Derivative Gain'

  0.0,

  // Mask Parameter: DiscretePDController_D_o
  //  Referenced by: '<S147>/Derivative Gain'

  0.0,

  // Mask Parameter: DiscretePDController_D_h
  //  Referenced by: '<S97>/Derivative Gain'

  0.0,

  // Mask Parameter: DiscretePDController_I
  //  Referenced by: '<S50>/Integral Gain'

  17.3911759678,

  // Mask Parameter: DiscretePDController_I_a
  //  Referenced by: '<S100>/Integral Gain'

  18.30415489,

  // Mask Parameter: DiscretePDController_I_c
  //  Referenced by: '<S150>/Integral Gain'

  18.289049312,

  // Mask Parameter: DiscretePDController_InitialCon
  //  Referenced by: '<S48>/Filter'

  0.0,

  // Mask Parameter: DiscretePDController_InitialC_h
  //  Referenced by: '<S148>/Filter'

  0.0,

  // Mask Parameter: DiscretePDController_InitialC_a
  //  Referenced by: '<S98>/Filter'

  0.0,

  // Mask Parameter: DiscretePDController_Initial_hm
  //  Referenced by: '<S53>/Integrator'

  0.0,

  // Mask Parameter: DiscretePDController_InitialC_b
  //  Referenced by: '<S153>/Integrator'

  0.0,

  // Mask Parameter: DiscretePDController_Initial_bz
  //  Referenced by: '<S103>/Integrator'

  0.0,

  // Mask Parameter: DiscretePDController_Kb
  //  Referenced by: '<S46>/Kb'

  5.0,

  // Mask Parameter: DiscretePDController_Kb_p
  //  Referenced by: '<S96>/Kb'

  5.0,

  // Mask Parameter: DiscretePDController_Kb_l
  //  Referenced by: '<S146>/Kb'

  5.0,

  // Mask Parameter: DiscretePDController_Kt
  //  Referenced by: '<S64>/Kt'

  5.0,

  // Mask Parameter: DiscretePDController_Kt_k
  //  Referenced by: '<S114>/Kt'

  5.0,

  // Mask Parameter: DiscretePDController_Kt_g
  //  Referenced by: '<S164>/Kt'

  5.0,

  // Mask Parameter: DiscretePDController_LowerSatur
  //  Referenced by: '<S60>/Saturation'

  -10.0,

  // Mask Parameter: DiscretePDController_LowerSat_h
  //  Referenced by: '<S160>/Saturation'

  -10.0,

  // Mask Parameter: DiscretePDController_LowerSat_f
  //  Referenced by: '<S110>/Saturation'

  -5.0,

  // Mask Parameter: DiscretePDController_N
  //  Referenced by: '<S56>/Filter Coefficient'

  100.0,

  // Mask Parameter: DiscretePDController_N_n
  //  Referenced by: '<S156>/Filter Coefficient'

  100.0,

  // Mask Parameter: DiscretePDController_N_j
  //  Referenced by: '<S106>/Filter Coefficient'

  100.0,

  // Mask Parameter: DiscretePDController_P
  //  Referenced by: '<S58>/Proportional Gain'

  2.55548530054,

  // Mask Parameter: DiscretePDController_P_k
  //  Referenced by: '<S158>/Proportional Gain'

  37.1730053299,

  // Mask Parameter: DiscretePDController_P_n
  //  Referenced by: '<S108>/Proportional Gain'

  56.6530807318,

  // Mask Parameter: DiscretePDController_UpperSatur
  //  Referenced by: '<S60>/Saturation'

  10.0,

  // Mask Parameter: DiscretePDController_UpperSat_e
  //  Referenced by: '<S160>/Saturation'

  10.0,

  // Mask Parameter: DiscretePDController_UpperSat_i
  //  Referenced by: '<S110>/Saturation'

  5.0,

  // Mask Parameter: uDOFEulerAngles_inertia
  //  Referenced by: '<S183>/Constant1'

  { 1.6931, 0.0, 0.0826, 0.0, 1.0708, 0.0, 0.0826, 0.0, 2.7281 },

  // Expression: 0
  //  Referenced by: '<S199>/Switch7'

  0.0,

  // Expression: 0
  //  Referenced by: '<S199>/Switch6'

  0.0,

  // Expression: 0
  //  Referenced by: '<S199>/Switch13'

  0.0,

  // Expression: 0
  //  Referenced by: '<S199>/Switch9'

  0.0,

  // Expression: 0
  //  Referenced by: '<S199>/Switch8'

  0.0,

  // Expression: 0
  //  Referenced by: '<S199>/Switch17'

  0.0,

  // Expression: 0
  //  Referenced by: '<S199>/Switch14'

  0.0,

  // Expression: 0
  //  Referenced by: '<S199>/Switch2'

  0.0,

  // Expression: [1; 1; 0]
  //  Referenced by: '<S201>/Gain1'

  { 1.0, 1.0, 0.0 },

  // Expression: 0
  //  Referenced by: '<S201>/Switch'

  0.0,

  // Expression: [0; 1; 0]
  //  Referenced by: '<S201>/Gain2'

  { 0.0, 1.0, 0.0 },

  // Expression: [0; 0; 0]
  //  Referenced by: '<S201>/Gain3'

  { 0.0, 0.0, 0.0 },

  // Expression: 0
  //  Referenced by: '<S201>/Switch2'

  0.0,

  // Expression: 0.001
  //  Referenced by: '<S203>/Constant4'

  0.001,

  // Expression: zeros(3)
  //  Referenced by: '<S183>/Constant2'

  { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 },

  // Expression: pi
  //  Referenced by: '<S181>/phi theta psi'

  3.1415926535897931,

  // Expression: -pi
  //  Referenced by: '<S181>/phi theta psi'

  -3.1415926535897931,

  // Expression: 0.5
  //  Referenced by: '<S171>/Gain2'

  0.5,

  // Expression: 1.225
  //  Referenced by: '<S171>/Constant1'

  1.225,

  // Expression: 0
  //  Referenced by: '<S4>/Constant1'

  0.0,

  // Expression: pi/180
  //  Referenced by: '<S206>/Gain1'

  0.017453292519943295,

  // Expression: 0
  //  Referenced by: '<S199>/Switch3'

  0.0,

  // Expression: 1
  //  Referenced by: '<S199>/Constant'

  1.0,

  // Expression: 0
  //  Referenced by: '<S199>/Constant1'

  0.0,

  // Expression: 0
  //  Referenced by: '<S199>/Switch1'

  0.0,

  // Expression: 0
  //  Referenced by: '<S199>/Constant15'

  0.0,

  // Expression: 0
  //  Referenced by: '<S199>/Switch4'

  0.0,

  // Expression: 0
  //  Referenced by: '<S199>/Constant6'

  0.0,

  // Expression: 0
  //  Referenced by: '<S199>/Switch'

  0.0,

  // Expression: 0
  //  Referenced by: '<S215>/Constant'

  0.0,

  // Expression: T0
  //  Referenced by: '<S202>/Sea Level  Temperature'

  288.15,

  // Expression: -1
  //  Referenced by: '<S180>/Gain'

  -1.0,

  // Expression: 0
  //  Referenced by: '<S180>/Constant1'

  0.0,

  // Expression: 0
  //  Referenced by: '<S180>/Switch'

  0.0,

  // Expression: h_trop
  //  Referenced by: '<S202>/Limit  altitude  to troposhere'

  11000.0,

  // Expression: h0
  //  Referenced by: '<S202>/Limit  altitude  to troposhere'

  0.0,

  // Expression: L
  //  Referenced by: '<S202>/Lapse Rate'

  0.0065,

  // Expression: 1/T0
  //  Referenced by: '<S202>/1//T0'

  0.00347041471455839,

  // Expression: g/(L*R)
  //  Referenced by: '<S202>/Constant'

  5.2558756014667134,

  // Expression: rho0
  //  Referenced by: '<S202>/rho0'

  1.225,

  // Expression: h_trop
  //  Referenced by: '<S202>/Altitude of Troposphere'

  11000.0,

  // Expression: 0
  //  Referenced by: '<S202>/Limit  altitude  to Stratosphere'

  0.0,

  // Expression: h_trop-h_strat
  //  Referenced by: '<S202>/Limit  altitude  to Stratosphere'

  -9000.0,

  // Expression: g/R
  //  Referenced by: '<S202>/g//R'

  0.034163191409533639,

  // Expression: 1/2
  //  Referenced by: '<S199>/Gain'

  0.5,

  // Expression: 0.000001
  //  Referenced by: '<S14>/e'

  1.0E-6,

  // Expression: 1
  //  Referenced by: '<S12>/Constant'

  1.0,

  // Expression: 0
  //  Referenced by: '<S12>/Constant1'

  0.0,

  // Expression: -1
  //  Referenced by: '<S12>/Gain'

  -1.0,

  // Expression: -1
  //  Referenced by: '<S12>/Gain1'

  -1.0,

  // Expression: P_phi_sp
  //  Referenced by: '<S3>/P_phi_sp'

  6.5,

  // Expression: P_theta_sp
  //  Referenced by: '<S3>/P_theta_sp'

  6.5,

  // Expression: 1
  //  Referenced by: '<S13>/Constant'

  1.0,

  // Expression: 0.0000001
  //  Referenced by: '<S13>/Constant1'

  1.0E-7,

  // Expression: 9.8
  //  Referenced by: '<S13>/g'

  9.8,

  // Expression: FW_RR
  //  Referenced by: '<S15>/FFGain'

  33.0766501559,

  // Expression: 0.000001
  //  Referenced by: '<S18>/e'

  1.0E-6,

  // Expression: FW_PR
  //  Referenced by: '<S19>/FFGain'

  73.5591994785,

  // Expression: 0.000001
  //  Referenced by: '<S16>/e'

  1.0E-6,

  // Expression: FW_YR
  //  Referenced by: '<S17>/FFGain'

  5.88724008066,

  // Expression: 1/sqrt(1.225)
  //  Referenced by: '<S9>/Gain'

  0.90350790290525129,

  // Expression: 0.000001
  //  Referenced by: '<S21>/e'

  1.0E-6,

  // Computed Parameter: Integrator_gainval
  //  Referenced by: '<S53>/Integrator'

  0.02,

  // Computed Parameter: Filter_gainval
  //  Referenced by: '<S48>/Filter'

  0.02,

  // Expression: 1/sqrt(1.225)
  //  Referenced by: '<S11>/Gain'

  0.90350790290525129,

  // Expression: 0.000001
  //  Referenced by: '<S121>/e'

  1.0E-6,

  // Computed Parameter: Integrator_gainval_j
  //  Referenced by: '<S153>/Integrator'

  0.02,

  // Computed Parameter: Filter_gainval_f
  //  Referenced by: '<S148>/Filter'

  0.02,

  // Expression: 1/sqrt(1.225)
  //  Referenced by: '<S10>/Gain'

  0.90350790290525129,

  // Expression: 0.000001
  //  Referenced by: '<S71>/e'

  1.0E-6,

  // Computed Parameter: Integrator_gainval_b
  //  Referenced by: '<S103>/Integrator'

  0.02,

  // Computed Parameter: Filter_gainval_c
  //  Referenced by: '<S98>/Filter'

  0.02,

  // Expression: 1
  //  Referenced by: '<Root>/NormalizedSurfaceLimit'

  1.0,

  // Expression: -1
  //  Referenced by: '<Root>/NormalizedSurfaceLimit'

  -1.0,

  // Expression: e_lim*[-1:1]
  //  Referenced by: '<S175>/1-D Lookup Table'

  { -12.0, 0.0, 12.0 },

  // Expression: [-1:1]
  //  Referenced by: '<S175>/1-D Lookup Table'

  { -1.0, 0.0, 1.0 },

  // Expression: a_lim*[-1:1]
  //  Referenced by: '<S175>/1-D Lookup Table1'

  { -10.0, 0.0, 10.0 },

  // Expression: [-1:1]
  //  Referenced by: '<S175>/1-D Lookup Table1'

  { -1.0, 0.0, 1.0 },

  // Expression: r_lim*[-1:1]
  //  Referenced by: '<S175>/1-D Lookup Table2'

  { -22.0, 0.0, 22.0 },

  // Expression: [-1:1]
  //  Referenced by: '<S175>/1-D Lookup Table2'

  { -1.0, 0.0, 1.0 },

  // Expression: 25
  //  Referenced by: '<S221>/Command Saturation'

  25.0,

  // Expression: -25
  //  Referenced by: '<S221>/Command Saturation'

  -25.0,

  // Expression: 0
  //  Referenced by: '<S199>/Constant13'

  0.0,

  // Expression: 0
  //  Referenced by: '<S199>/Switch12'

  0.0,

  // Expression: -1
  //  Referenced by: '<S199>/Gain16'

  -1.0,

  // Expression: 0.000001
  //  Referenced by: '<S199>/Constant22'

  1.0E-6,

  // Expression: 0.000001
  //  Referenced by: '<S199>/Constant21'

  1.0E-6,

  // Expression: 0
  //  Referenced by: '<S199>/Constant18'

  0.0,

  // Expression: 0
  //  Referenced by: '<S199>/Switch10'

  0.0,

  // Expression: 0.000001
  //  Referenced by: '<S199>/Constant23'

  1.0E-6,

  // Expression: 0
  //  Referenced by: '<S199>/Constant9'

  0.0,

  // Expression: 0
  //  Referenced by: '<S199>/Switch5'

  0.0,

  // Expression: -1
  //  Referenced by: '<S199>/Gain23'

  -1.0,

  // Expression: [0; 0; 9.81]
  //  Referenced by: '<S200>/G=mg'

  { 0.0, 0.0, 9.81 },

  // Expression: 1
  //  Referenced by: '<S203>/Jmax2'

  1.0,

  // Expression: pi
  //  Referenced by: '<S203>/Pi'

  3.1415926535897931,

  // Expression: 1
  //  Referenced by: '<S5>/Saturation'

  1.0,

  // Expression: 0
  //  Referenced by: '<S5>/Saturation'

  0.0,

  // Expression: 1
  //  Referenced by: '<S203>/Saturation'

  1.0,

  // Expression: 0
  //  Referenced by: '<S203>/Saturation'

  0.0,

  // Expression: 0.98
  //  Referenced by: '<S203>/Constant'

  0.98,

  // Expression: 0
  //  Referenced by: '<S203>/Switch1'

  0.0,

  // Expression: 0.95
  //  Referenced by: '<S203>/Jmax'

  0.95,

  // Expression: 2
  //  Referenced by: '<S203>/Gain'

  2.0,

  // Expression: 4
  //  Referenced by: '<S203>/constant1'

  4.0,

  // Expression: pi
  //  Referenced by: '<S203>/Pi1'

  3.1415926535897931,

  // Expression: 2
  //  Referenced by: '<S203>/Gain8'

  2.0,

  // Expression: 0
  //  Referenced by: '<S203>/Constant1'

  0.0,

  // Expression: 0
  //  Referenced by: '<S203>/Switch'

  0.0,

  // Expression: -1
  //  Referenced by: '<S180>/Gain1'

  -1.0,

  // Expression: 0
  //  Referenced by: '<S201>/Switch1'

  0.0,

  // Expression: 0.75
  //  Referenced by: '<S203>/Jmax1'

  0.75,

  // Expression: 1
  //  Referenced by: '<S203>/Jmax3'

  1.0,

  // Expression: 5
  //  Referenced by: '<S203>/constant2'

  5.0,

  // Expression: pi
  //  Referenced by: '<S203>/Pi2'

  3.1415926535897931,

  // Expression: 2
  //  Referenced by: '<S203>/Gain9'

  2.0,

  // Expression: 0
  //  Referenced by: '<S203>/Constant2'

  0.0,

  // Expression: 0.000001
  //  Referenced by: '<S199>/Constant20'

  1.0E-6,

  // Expression: 0.000001
  //  Referenced by: '<S199>/Constant19'

  1.0E-6,

  // Expression: 0
  //  Referenced by: '<S199>/Constant10'

  0.0,

  // Expression: 0
  //  Referenced by: '<S199>/Switch16'

  0.0,

  // Expression: 0.000001
  //  Referenced by: '<S199>/Constant24'

  1.0E-6,

  // Expression: 0
  //  Referenced by: '<S199>/Constant11'

  0.0,

  // Expression: 0
  //  Referenced by: '<S199>/Switch11'

  0.0,

  // Expression: 0.000001
  //  Referenced by: '<S199>/Constant16'

  1.0E-6,

  // Expression: 0.000001
  //  Referenced by: '<S199>/Constant5'

  1.0E-6,

  // Expression: 0
  //  Referenced by: '<S199>/Constant12'

  0.0,

  // Expression: 0
  //  Referenced by: '<S199>/Switch15'

  0.0,

  // Expression: 0
  //  Referenced by: '<S201>/Switch3'

  0.0,

  // Expression: -1
  //  Referenced by: '<S175>/Gain'

  -1.0,

  // Expression: -1
  //  Referenced by: '<S175>/Gain1'

  -1.0,

  // Expression: 1e-6
  //  Referenced by: '<S9>/aw_eps'

  1.0E-6,

  // Expression: 1/(V_I0^2)
  //  Referenced by: '<S9>/aw_inv_Vref2'

  0.00390625,

  // Expression: 1e-6
  //  Referenced by: '<S10>/aw_eps'

  1.0E-6,

  // Expression: 1/(V_I0^2)
  //  Referenced by: '<S10>/aw_inv_Vref2'

  0.00390625,

  // Expression: 1e-6
  //  Referenced by: '<S11>/aw_eps'

  1.0E-6,

  // Expression: 1/(V_I0^2)
  //  Referenced by: '<S11>/aw_inv_Vref2'

  0.00390625
};

//
// File trailer for generated code.
//
// [EOF]
//
