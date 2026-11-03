//
// File: PX4Controller_ertmodel0827.h
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
#ifndef RTW_HEADER_PX4Controller_ertmodel0827_h_
#define RTW_HEADER_PX4Controller_ertmodel0827_h_
#include <cstring>
#include <cmath>
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "PX4Controller_ertmodel0827_types.h"
#include "rt_assert.h"
#include "rt_nonfinite.h"

// Macros for accessing real-time model data structure
#ifndef rtmGetContStateDisabled
#define rtmGetContStateDisabled(rtm)   ((rtm)->contStateDisabled)
#endif

#ifndef rtmSetContStateDisabled
#define rtmSetContStateDisabled(rtm, val) ((rtm)->contStateDisabled = (val))
#endif

#ifndef rtmGetContStates
#define rtmGetContStates(rtm)          ((rtm)->contStates)
#endif

#ifndef rtmSetContStates
#define rtmSetContStates(rtm, val)     ((rtm)->contStates = (val))
#endif

#ifndef rtmGetContTimeOutputInconsistentWithStateAtMajorStepFlag
#define rtmGetContTimeOutputInconsistentWithStateAtMajorStepFlag(rtm) ((rtm)->CTOutputIncnstWithState)
#endif

#ifndef rtmSetContTimeOutputInconsistentWithStateAtMajorStepFlag
#define rtmSetContTimeOutputInconsistentWithStateAtMajorStepFlag(rtm, val) ((rtm)->CTOutputIncnstWithState = (val))
#endif

#ifndef rtmGetDerivCacheNeedsReset
#define rtmGetDerivCacheNeedsReset(rtm) ((rtm)->derivCacheNeedsReset)
#endif

#ifndef rtmSetDerivCacheNeedsReset
#define rtmSetDerivCacheNeedsReset(rtm, val) ((rtm)->derivCacheNeedsReset = (val))
#endif

#ifndef rtmGetIntgData
#define rtmGetIntgData(rtm)            ((rtm)->intgData)
#endif

#ifndef rtmSetIntgData
#define rtmSetIntgData(rtm, val)       ((rtm)->intgData = (val))
#endif

#ifndef rtmGetOdeF
#define rtmGetOdeF(rtm)                ((rtm)->odeF)
#endif

#ifndef rtmSetOdeF
#define rtmSetOdeF(rtm, val)           ((rtm)->odeF = (val))
#endif

#ifndef rtmGetOdeY
#define rtmGetOdeY(rtm)                ((rtm)->odeY)
#endif

#ifndef rtmSetOdeY
#define rtmSetOdeY(rtm, val)           ((rtm)->odeY = (val))
#endif

#ifndef rtmGetPeriodicContStateIndices
#define rtmGetPeriodicContStateIndices(rtm) ((rtm)->periodicContStateIndices)
#endif

#ifndef rtmSetPeriodicContStateIndices
#define rtmSetPeriodicContStateIndices(rtm, val) ((rtm)->periodicContStateIndices = (val))
#endif

#ifndef rtmGetPeriodicContStateRanges
#define rtmGetPeriodicContStateRanges(rtm) ((rtm)->periodicContStateRanges)
#endif

#ifndef rtmSetPeriodicContStateRanges
#define rtmSetPeriodicContStateRanges(rtm, val) ((rtm)->periodicContStateRanges = (val))
#endif

#ifndef rtmGetZCCacheNeedsReset
#define rtmGetZCCacheNeedsReset(rtm)   ((rtm)->zCCacheNeedsReset)
#endif

#ifndef rtmSetZCCacheNeedsReset
#define rtmSetZCCacheNeedsReset(rtm, val) ((rtm)->zCCacheNeedsReset = (val))
#endif

#ifndef rtmGetdX
#define rtmGetdX(rtm)                  ((rtm)->derivs)
#endif

#ifndef rtmSetdX
#define rtmSetdX(rtm, val)             ((rtm)->derivs = (val))
#endif

#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

#ifndef rtmGetStopRequested
#define rtmGetStopRequested(rtm)       ((rtm)->Timing.stopRequestedFlag)
#endif

#ifndef rtmSetStopRequested
#define rtmSetStopRequested(rtm, val)  ((rtm)->Timing.stopRequestedFlag = (val))
#endif

#ifndef rtmGetStopRequestedPtr
#define rtmGetStopRequestedPtr(rtm)    (&((rtm)->Timing.stopRequestedFlag))
#endif

#ifndef rtmGetT
#define rtmGetT(rtm)                   (rtmGetTPtr((rtm))[0])
#endif

#ifndef rtmGetTPtr
#define rtmGetTPtr(rtm)                ((rtm)->Timing.t)
#endif

#ifndef ODE4_INTG
#define ODE4_INTG

// ODE4 Integration Data
struct ODE4_IntgData {
  real_T *y;                           // output
  real_T *f[4];                        // derivatives
};

#endif

// Class declaration for model PX4Controller_ertmodel0827
class PX4Controller_ertmodel0827ModelClass {
  // public data and function members
 public:
  // Block signals (default storage)
  struct B_PX4Controller_ertmodel0827_T {
    real_T Selector1[9];               // '<S182>/Selector1'
    real_T Selector[9];                // '<S182>/Selector'
    real_T Selector2[9];               // '<S182>/Selector2'
    real_T Product[9];                 // '<S172>/Product'
    real_T MixerInverseForAW[9];       // '<Root>/MixerInverseForAW'
    real_T Gain1;                      // '<S206>/Gain1'
    real_T wind[3];                    // '<S5>/wind'
    real_T VectorConcatenate[9];       // '<S217>/Vector Concatenate'
    real_T VectorConcatenate_m[9];     // '<S170>/Vector Concatenate'
    real_T TrigonometricFunction1;     // '<S13>/Trigonometric Function1'
    real_T TrigonometricFunction;      // '<S13>/Trigonometric Function'
    real_T VectorConcatenate_e[3];     // '<S3>/Vector Concatenate'
    real_T MathFunction;               // '<S21>/Math Function'
    real_T VectorConcatenate1[3];      // '<Root>/Vector Concatenate1'
    real_T Saturation;                 // '<S60>/Saturation'
    real_T MathFunction_h;             // '<S121>/Math Function'
    real_T Saturation_e;               // '<S160>/Saturation'
    real_T MathFunction_f;             // '<S71>/Math Function'
    real_T Saturation_n;               // '<S110>/Saturation'
    real_T Gain46;                     // '<S199>/Gain46'
    real_T Gain36;                     // '<S199>/Gain36'
    real_T Gain27;                     // '<S199>/Gain27'
    real_T Gain10;                     // '<S199>/Gain10'
    real_T Gmg[3];                     // '<S200>/G=mg'
    real_T Switch1;                    // '<S203>/Switch1'
    real_T Power1;                     // '<S203>/Power1'
    real_T Square;                     // '<S203>/Square'
    real_T Sum[3];                     // '<S176>/Sum'
    real_T Power2;                     // '<S203>/Power2'
    real_T Square1;                    // '<S203>/Square1'
    real_T Gain6;                      // '<S199>/Gain6'
    real_T Gain39;                     // '<S199>/Gain39'
    real_T Gain15;                     // '<S199>/Gain15'
    real_T Gain19;                     // '<S199>/Gain19'
    real_T Gain20;                     // '<S199>/Gain20'
    real_T Product2[3];                // '<S182>/Product2'
    real_T Product_o[3];               // '<S188>/Product'
    real_T TmpSignalConversionAtphithetaps[3];// '<S181>/phidot thetadot psidot' 
  };

  // Block states (default storage) for system '<Root>'
  struct DW_PX4Controller_ertmodel0827_T {
    real_T Integrator_DSTATE;          // '<S53>/Integrator'
    real_T Filter_DSTATE;              // '<S48>/Filter'
    real_T Integrator_DSTATE_b;        // '<S153>/Integrator'
    real_T Filter_DSTATE_j;            // '<S148>/Filter'
    real_T Integrator_DSTATE_e;        // '<S103>/Integrator'
    real_T Filter_DSTATE_n;            // '<S98>/Filter'
    real_T Product_DWORK4[9];          // '<S172>/Product'
    real_T MixerInverseForAW_DWORK4[9];// '<Root>/MixerInverseForAW'
    real_T Product2_DWORK4[9];         // '<S182>/Product2'
  };

  // Continuous states (default storage)
  struct X_PX4Controller_ertmodel0827_T {
    real_T phithetapsi_CSTATE[3];      // '<S181>/phi theta psi'
    real_T ubvbwb_CSTATE[3];           // '<S176>/ub,vb,wb'
    real_T xeyeze_CSTATE[3];           // '<S176>/xe,ye,ze'
    real_T pqr_CSTATE[3];              // '<S176>/p,q,r '
  };

  // Periodic continuous state vector (global)
  typedef int_T PeriodicIndX_PX4Controller_er_T[3];
  typedef real_T PeriodicRngX_PX4Controller_er_T[6];

  // State derivatives (default storage)
  struct XDot_PX4Controller_ertmodel08_T {
    real_T phithetapsi_CSTATE[3];      // '<S181>/phi theta psi'
    real_T ubvbwb_CSTATE[3];           // '<S176>/ub,vb,wb'
    real_T xeyeze_CSTATE[3];           // '<S176>/xe,ye,ze'
    real_T pqr_CSTATE[3];              // '<S176>/p,q,r '
  };

  // State disabled
  struct XDis_PX4Controller_ertmodel08_T {
    boolean_T phithetapsi_CSTATE[3];   // '<S181>/phi theta psi'
    boolean_T ubvbwb_CSTATE[3];        // '<S176>/ub,vb,wb'
    boolean_T xeyeze_CSTATE[3];        // '<S176>/xe,ye,ze'
    boolean_T pqr_CSTATE[3];           // '<S176>/p,q,r '
  };

  // External inputs (root inport signals with default storage)
  struct ExtU_PX4Controller_ertmodel08_T {
    real_T phi_sp;                     // '<Root>/phi_sp'
    real_T theta_sp;                   // '<Root>/theta_sp'
    real_T T_cmd;                      // '<Root>/T_cmd'
    real_T w_speed[3];                 // '<Root>/w_speed'
  };

  // External outputs (root outports fed by signals with default storage)
  struct ExtY_PX4Controller_ertmodel08_T {
    real_T alpha;                      // '<Root>/alpha'
    real_T beta;                       // '<Root>/beta'
    real_T POS_ned[9];                 // '<Root>/POS_ned'
    real_T STATE_body[12];             // '<Root>/STATE_body'
  };

  // Parameters (default storage)
  struct P_PX4Controller_ertmodel0827_T {
    real_T CD0;                        // Variable: CD0
                                          //  Referenced by: '<S199>/Constant2'

    real_T CL0;                        // Variable: CL0
                                          //  Referenced by:
                                          //    '<S199>/Constant3'
                                          //    '<S199>/Constant4'

    real_T Cl0;                        // Variable: Cl0
                                          //  Referenced by: '<S199>/Constant7'

    real_T Cm0;                        // Variable: Cm0
                                          //  Referenced by: '<S199>/Constant8'

    real_T Cn0;                        // Variable: Cn0
                                          //  Referenced by: '<S199>/Constant14'

    real_T Cy0;                        // Variable: Cy0
                                          //  Referenced by: '<S199>/Constant17'

    real_T Ix;                         // Variable: Ix
                                          //  Referenced by: '<S4>/Constant'

    real_T Iy;                         // Variable: Iy
                                          //  Referenced by: '<S4>/Constant2'

    real_T Iz;                         // Variable: Iz
                                          //  Referenced by: '<S4>/Constant3'

    real_T S;                          // Variable: S
                                          //  Referenced by:
                                          //    '<S171>/Gain1'
                                          //    '<S199>/Gain1'
                                          //    '<S199>/Gain11'
                                          //    '<S199>/Gain21'
                                          //    '<S199>/Gain29'
                                          //    '<S199>/Gain42'
                                          //    '<S199>/Gain47'

    real_T Sprop;                      // Variable: Sprop
                                          //  Referenced by: '<S203>/Constant3'

    real_T V_I0;                       // Variable: V_I0
                                          //  Referenced by:
                                          //    '<S171>/Constant'
                                          //    '<S21>/V_I0'
                                          //    '<S71>/V_I0'
                                          //    '<S121>/V_I0'

    real_T V_t0;                       // Variable: V_t0
                                          //  Referenced by:
                                          //    '<S14>/Vt0'
                                          //    '<S16>/Vt0'
                                          //    '<S18>/Vt0'

    real_T a_lim;                      // Variable: a_lim
                                          //  Referenced by: '<S219>/Command Saturation'

    real_T alpha_0;                    // Variable: alpha_0
                                          //  Referenced by: '<S199>/Fixed_Angle_of_Attack'

    real_T b;                          // Variable: b
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

    real_T c;                          // Variable: c
                                          //  Referenced by:
                                          //    '<S4>/Gain2'
                                          //    '<S199>/Gain11'
                                          //    '<S199>/Gain14'
                                          //    '<S199>/Gain9'

    real_T dCDdsigmaE;                 // Variable: dCDdsigmaE
                                          //  Referenced by: '<S199>/Gain46'

    real_T dCLdalpha;                  // Variable: dCLdalpha
                                          //  Referenced by:
                                          //    '<S199>/Gain43'
                                          //    '<S199>/Gain7'

    real_T dCLdq;                      // Variable: dCLdq
                                          //  Referenced by: '<S199>/Gain8'

    real_T dCLdsigmaE;                 // Variable: dCLdsigmaE
                                          //  Referenced by: '<S199>/Gain10'

    real_T dCldbeta;                   // Variable: dCldbeta
                                          //  Referenced by: '<S199>/Gain5'

    real_T dCldp;                      // Variable: dCldp
                                          //  Referenced by: '<S199>/Gain49'

    real_T dCldr;                      // Variable: dCldr
                                          //  Referenced by: '<S199>/Gain48'

    real_T dCldsigmaA;                 // Variable: dCldsigmaA
                                          //  Referenced by:
                                          //    '<S4>/Gain'
                                          //    '<S199>/Gain6'

    real_T dCldsigmaR;                 // Variable: dCldsigmaR
                                          //  Referenced by:
                                          //    '<S4>/Gain1'
                                          //    '<S199>/Gain39'

    real_T dCmdalpha;                  // Variable: dCmdalpha
                                          //  Referenced by: '<S199>/Gain12'

    real_T dCmdq;                      // Variable: dCmdq
                                          //  Referenced by: '<S199>/Gain13'

    real_T dCmdsigmaE;                 // Variable: dCmdsigmaE
                                          //  Referenced by:
                                          //    '<S4>/Gain2'
                                          //    '<S199>/Gain15'

    real_T dCndbeta;                   // Variable: dCndbeta
                                          //  Referenced by: '<S199>/Gain18'

    real_T dCndp;                      // Variable: dCndp
                                          //  Referenced by: '<S199>/Gain38'

    real_T dCndr;                      // Variable: dCndr
                                          //  Referenced by: '<S199>/Gain22'

    real_T dCndsigmaA;                 // Variable: dCndsigmaA
                                          //  Referenced by:
                                          //    '<S4>/Gain3'
                                          //    '<S199>/Gain19'

    real_T dCndsigmaR;                 // Variable: dCndsigmaR
                                          //  Referenced by:
                                          //    '<S4>/Gain4'
                                          //    '<S199>/Gain20'

    real_T dCydbeta;                   // Variable: dCydbeta
                                          //  Referenced by: '<S199>/Gain28'

    real_T dCydp;                      // Variable: dCydp
                                          //  Referenced by: '<S199>/Gain35'

    real_T dCydr;                      // Variable: dCydr
                                          //  Referenced by: '<S199>/Gain30'

    real_T dCydsigmaA;                 // Variable: dCydsigmaA
                                          //  Referenced by: '<S199>/Gain36'

    real_T dCydsigmaR;                 // Variable: dCydsigmaR
                                          //  Referenced by: '<S199>/Gain27'

    real_T e_lim;                      // Variable: e_lim
                                          //  Referenced by: '<S220>/Command Saturation'

    real_T init_Xe_Ye_Ze[3];           // Variable: init_Xe_Ye_Ze
                                          //  Referenced by: '<S176>/xe,ye,ze'

    real_T init_p_q_r[3];              // Variable: init_p_q_r
                                          //  Referenced by: '<S176>/p,q,r '

    real_T init_phi_theta_psi[3];      // Variable: init_phi_theta_psi
                                          //  Referenced by: '<S181>/phi theta psi'

    real_T init_u_v_w[3];              // Variable: init_u_v_w
                                          //  Referenced by: '<S176>/ub,vb,wb'

    real_T kTp;                        // Variable: kTp
                                          //  Referenced by: '<S203>/Gain3'

    real_T k_motor;                    // Variable: k_motor
                                          //  Referenced by: '<S203>/Gain1'

    real_T k_omega;                    // Variable: k_omega
                                          //  Referenced by: '<S203>/Gain2'

    real_T m;                          // Variable: m
                                          //  Referenced by:
                                          //    '<S180>/Constant'
                                          //    '<S183>/Constant'

    real_T omega_max;                  // Variable: omega_max
                                          //  Referenced by: '<S203>/Gain4'

    real_T DiscretePDController_D;     // Mask Parameter: DiscretePDController_D
                                          //  Referenced by: '<S47>/Derivative Gain'

    real_T DiscretePDController_D_o; // Mask Parameter: DiscretePDController_D_o
                                        //  Referenced by: '<S147>/Derivative Gain'

    real_T DiscretePDController_D_h; // Mask Parameter: DiscretePDController_D_h
                                        //  Referenced by: '<S97>/Derivative Gain'

    real_T DiscretePDController_I;     // Mask Parameter: DiscretePDController_I
                                          //  Referenced by: '<S50>/Integral Gain'

    real_T DiscretePDController_I_a; // Mask Parameter: DiscretePDController_I_a
                                        //  Referenced by: '<S100>/Integral Gain'

    real_T DiscretePDController_I_c; // Mask Parameter: DiscretePDController_I_c
                                        //  Referenced by: '<S150>/Integral Gain'

    real_T DiscretePDController_InitialCon;
                              // Mask Parameter: DiscretePDController_InitialCon
                                 //  Referenced by: '<S48>/Filter'

    real_T DiscretePDController_InitialC_h;
                              // Mask Parameter: DiscretePDController_InitialC_h
                                 //  Referenced by: '<S148>/Filter'

    real_T DiscretePDController_InitialC_a;
                              // Mask Parameter: DiscretePDController_InitialC_a
                                 //  Referenced by: '<S98>/Filter'

    real_T DiscretePDController_Initial_hm;
                              // Mask Parameter: DiscretePDController_Initial_hm
                                 //  Referenced by: '<S53>/Integrator'

    real_T DiscretePDController_InitialC_b;
                              // Mask Parameter: DiscretePDController_InitialC_b
                                 //  Referenced by: '<S153>/Integrator'

    real_T DiscretePDController_Initial_bz;
                              // Mask Parameter: DiscretePDController_Initial_bz
                                 //  Referenced by: '<S103>/Integrator'

    real_T DiscretePDController_Kb;   // Mask Parameter: DiscretePDController_Kb
                                         //  Referenced by: '<S46>/Kb'

    real_T DiscretePDController_Kb_p;
                                    // Mask Parameter: DiscretePDController_Kb_p
                                       //  Referenced by: '<S96>/Kb'

    real_T DiscretePDController_Kb_l;
                                    // Mask Parameter: DiscretePDController_Kb_l
                                       //  Referenced by: '<S146>/Kb'

    real_T DiscretePDController_Kt;   // Mask Parameter: DiscretePDController_Kt
                                         //  Referenced by: '<S64>/Kt'

    real_T DiscretePDController_Kt_k;
                                    // Mask Parameter: DiscretePDController_Kt_k
                                       //  Referenced by: '<S114>/Kt'

    real_T DiscretePDController_Kt_g;
                                    // Mask Parameter: DiscretePDController_Kt_g
                                       //  Referenced by: '<S164>/Kt'

    real_T DiscretePDController_LowerSatur;
                              // Mask Parameter: DiscretePDController_LowerSatur
                                 //  Referenced by: '<S60>/Saturation'

    real_T DiscretePDController_LowerSat_h;
                              // Mask Parameter: DiscretePDController_LowerSat_h
                                 //  Referenced by: '<S160>/Saturation'

    real_T DiscretePDController_LowerSat_f;
                              // Mask Parameter: DiscretePDController_LowerSat_f
                                 //  Referenced by: '<S110>/Saturation'

    real_T DiscretePDController_N;     // Mask Parameter: DiscretePDController_N
                                          //  Referenced by: '<S56>/Filter Coefficient'

    real_T DiscretePDController_N_n; // Mask Parameter: DiscretePDController_N_n
                                        //  Referenced by: '<S156>/Filter Coefficient'

    real_T DiscretePDController_N_j; // Mask Parameter: DiscretePDController_N_j
                                        //  Referenced by: '<S106>/Filter Coefficient'

    real_T DiscretePDController_P;     // Mask Parameter: DiscretePDController_P
                                          //  Referenced by: '<S58>/Proportional Gain'

    real_T DiscretePDController_P_k; // Mask Parameter: DiscretePDController_P_k
                                        //  Referenced by: '<S158>/Proportional Gain'

    real_T DiscretePDController_P_n; // Mask Parameter: DiscretePDController_P_n
                                        //  Referenced by: '<S108>/Proportional Gain'

    real_T DiscretePDController_UpperSatur;
                              // Mask Parameter: DiscretePDController_UpperSatur
                                 //  Referenced by: '<S60>/Saturation'

    real_T DiscretePDController_UpperSat_e;
                              // Mask Parameter: DiscretePDController_UpperSat_e
                                 //  Referenced by: '<S160>/Saturation'

    real_T DiscretePDController_UpperSat_i;
                              // Mask Parameter: DiscretePDController_UpperSat_i
                                 //  Referenced by: '<S110>/Saturation'

    real_T uDOFEulerAngles_inertia[9];// Mask Parameter: uDOFEulerAngles_inertia
                                         //  Referenced by: '<S183>/Constant1'

    real_T Switch7_Threshold;          // Expression: 0
                                          //  Referenced by: '<S199>/Switch7'

    real_T Switch6_Threshold;          // Expression: 0
                                          //  Referenced by: '<S199>/Switch6'

    real_T Switch13_Threshold;         // Expression: 0
                                          //  Referenced by: '<S199>/Switch13'

    real_T Switch9_Threshold;          // Expression: 0
                                          //  Referenced by: '<S199>/Switch9'

    real_T Switch8_Threshold;          // Expression: 0
                                          //  Referenced by: '<S199>/Switch8'

    real_T Switch17_Threshold;         // Expression: 0
                                          //  Referenced by: '<S199>/Switch17'

    real_T Switch14_Threshold;         // Expression: 0
                                          //  Referenced by: '<S199>/Switch14'

    real_T Switch2_Threshold;          // Expression: 0
                                          //  Referenced by: '<S199>/Switch2'

    real_T Gain1_Gain[3];              // Expression: [1; 1; 0]
                                          //  Referenced by: '<S201>/Gain1'

    real_T Switch_Threshold;           // Expression: 0
                                          //  Referenced by: '<S201>/Switch'

    real_T Gain2_Gain[3];              // Expression: [0; 1; 0]
                                          //  Referenced by: '<S201>/Gain2'

    real_T Gain3_Gain[3];              // Expression: [0; 0; 0]
                                          //  Referenced by: '<S201>/Gain3'

    real_T Switch2_Threshold_n;        // Expression: 0
                                          //  Referenced by: '<S201>/Switch2'

    real_T Constant4_Value;            // Expression: 0.001
                                          //  Referenced by: '<S203>/Constant4'

    real_T Constant2_Value[9];         // Expression: zeros(3)
                                          //  Referenced by: '<S183>/Constant2'

    real_T phithetapsi_WrappedStateUpperVa;// Expression: pi
                                              //  Referenced by: '<S181>/phi theta psi'

    real_T phithetapsi_WrappedStateLowerVa;// Expression: -pi
                                              //  Referenced by: '<S181>/phi theta psi'

    real_T Gain2_Gain_b;               // Expression: 0.5
                                          //  Referenced by: '<S171>/Gain2'

    real_T Constant1_Value;            // Expression: 1.225
                                          //  Referenced by: '<S171>/Constant1'

    real_T Constant1_Value_n;          // Expression: 0
                                          //  Referenced by: '<S4>/Constant1'

    real_T Gain1_Gain_p;               // Expression: pi/180
                                          //  Referenced by: '<S206>/Gain1'

    real_T Switch3_Threshold;          // Expression: 0
                                          //  Referenced by: '<S199>/Switch3'

    real_T Constant_Value;             // Expression: 1
                                          //  Referenced by: '<S199>/Constant'

    real_T Constant1_Value_i;          // Expression: 0
                                          //  Referenced by: '<S199>/Constant1'

    real_T Switch1_Threshold;          // Expression: 0
                                          //  Referenced by: '<S199>/Switch1'

    real_T Constant15_Value;           // Expression: 0
                                          //  Referenced by: '<S199>/Constant15'

    real_T Switch4_Threshold;          // Expression: 0
                                          //  Referenced by: '<S199>/Switch4'

    real_T Constant6_Value;            // Expression: 0
                                          //  Referenced by: '<S199>/Constant6'

    real_T Switch_Threshold_l;         // Expression: 0
                                          //  Referenced by: '<S199>/Switch'

    real_T Constant_Value_j;           // Expression: 0
                                          //  Referenced by: '<S215>/Constant'

    real_T SeaLevelTemperature_Value;  // Expression: T0
                                          //  Referenced by: '<S202>/Sea Level  Temperature'

    real_T Gain_Gain;                  // Expression: -1
                                          //  Referenced by: '<S180>/Gain'

    real_T Constant1_Value_h;          // Expression: 0
                                          //  Referenced by: '<S180>/Constant1'

    real_T Switch_Threshold_k;         // Expression: 0
                                          //  Referenced by: '<S180>/Switch'

    real_T Limitaltitudetotroposhere_Upper;// Expression: h_trop
                                              //  Referenced by: '<S202>/Limit  altitude  to troposhere'

    real_T Limitaltitudetotroposhere_Lower;// Expression: h0
                                              //  Referenced by: '<S202>/Limit  altitude  to troposhere'

    real_T LapseRate_Gain;             // Expression: L
                                          //  Referenced by: '<S202>/Lapse Rate'

    real_T uT0_Gain;                   // Expression: 1/T0
                                          //  Referenced by: '<S202>/1//T0'

    real_T Constant_Value_p;           // Expression: g/(L*R)
                                          //  Referenced by: '<S202>/Constant'

    real_T rho0_Gain;                  // Expression: rho0
                                          //  Referenced by: '<S202>/rho0'

    real_T AltitudeofTroposphere_Value;// Expression: h_trop
                                          //  Referenced by: '<S202>/Altitude of Troposphere'

    real_T LimitaltitudetoStratosphere_Upp;// Expression: 0
                                              //  Referenced by: '<S202>/Limit  altitude  to Stratosphere'

    real_T LimitaltitudetoStratosphere_Low;// Expression: h_trop-h_strat
                                              //  Referenced by: '<S202>/Limit  altitude  to Stratosphere'

    real_T gR_Gain;                    // Expression: g/R
                                          //  Referenced by: '<S202>/g//R'

    real_T Gain_Gain_e;                // Expression: 1/2
                                          //  Referenced by: '<S199>/Gain'

    real_T e_Value;                    // Expression: 0.000001
                                          //  Referenced by: '<S14>/e'

    real_T Constant_Value_i;           // Expression: 1
                                          //  Referenced by: '<S12>/Constant'

    real_T Constant1_Value_o;          // Expression: 0
                                          //  Referenced by: '<S12>/Constant1'

    real_T Gain_Gain_b;                // Expression: -1
                                          //  Referenced by: '<S12>/Gain'

    real_T Gain1_Gain_f;               // Expression: -1
                                          //  Referenced by: '<S12>/Gain1'

    real_T P_phi_sp_Gain;              // Expression: P_phi_sp
                                          //  Referenced by: '<S3>/P_phi_sp'

    real_T P_theta_sp_Gain;            // Expression: P_theta_sp
                                          //  Referenced by: '<S3>/P_theta_sp'

    real_T Constant_Value_p1;          // Expression: 1
                                          //  Referenced by: '<S13>/Constant'

    real_T Constant1_Value_f;          // Expression: 0.0000001
                                          //  Referenced by: '<S13>/Constant1'

    real_T g_Gain;                     // Expression: 9.8
                                          //  Referenced by: '<S13>/g'

    real_T FFGain_Gain;                // Expression: FW_RR
                                          //  Referenced by: '<S15>/FFGain'

    real_T e_Value_b;                  // Expression: 0.000001
                                          //  Referenced by: '<S18>/e'

    real_T FFGain_Gain_d;              // Expression: FW_PR
                                          //  Referenced by: '<S19>/FFGain'

    real_T e_Value_c;                  // Expression: 0.000001
                                          //  Referenced by: '<S16>/e'

    real_T FFGain_Gain_c;              // Expression: FW_YR
                                          //  Referenced by: '<S17>/FFGain'

    real_T Gain_Gain_j;                // Expression: 1/sqrt(1.225)
                                          //  Referenced by: '<S9>/Gain'

    real_T e_Value_o;                  // Expression: 0.000001
                                          //  Referenced by: '<S21>/e'

    real_T Integrator_gainval;         // Computed Parameter: Integrator_gainval
                                          //  Referenced by: '<S53>/Integrator'

    real_T Filter_gainval;             // Computed Parameter: Filter_gainval
                                          //  Referenced by: '<S48>/Filter'

    real_T Gain_Gain_bc;               // Expression: 1/sqrt(1.225)
                                          //  Referenced by: '<S11>/Gain'

    real_T e_Value_g;                  // Expression: 0.000001
                                          //  Referenced by: '<S121>/e'

    real_T Integrator_gainval_j;     // Computed Parameter: Integrator_gainval_j
                                        //  Referenced by: '<S153>/Integrator'

    real_T Filter_gainval_f;           // Computed Parameter: Filter_gainval_f
                                          //  Referenced by: '<S148>/Filter'

    real_T Gain_Gain_l;                // Expression: 1/sqrt(1.225)
                                          //  Referenced by: '<S10>/Gain'

    real_T e_Value_bf;                 // Expression: 0.000001
                                          //  Referenced by: '<S71>/e'

    real_T Integrator_gainval_b;     // Computed Parameter: Integrator_gainval_b
                                        //  Referenced by: '<S103>/Integrator'

    real_T Filter_gainval_c;           // Computed Parameter: Filter_gainval_c
                                          //  Referenced by: '<S98>/Filter'

    real_T NormalizedSurfaceLimit_UpperSat;// Expression: 1
                                              //  Referenced by: '<Root>/NormalizedSurfaceLimit'

    real_T NormalizedSurfaceLimit_LowerSat;// Expression: -1
                                              //  Referenced by: '<Root>/NormalizedSurfaceLimit'

    real_T uDLookupTable_tableData[3]; // Expression: e_lim*[-1:1]
                                          //  Referenced by: '<S175>/1-D Lookup Table'

    real_T uDLookupTable_bp01Data[3];  // Expression: [-1:1]
                                          //  Referenced by: '<S175>/1-D Lookup Table'

    real_T uDLookupTable1_tableData[3];// Expression: a_lim*[-1:1]
                                          //  Referenced by: '<S175>/1-D Lookup Table1'

    real_T uDLookupTable1_bp01Data[3]; // Expression: [-1:1]
                                          //  Referenced by: '<S175>/1-D Lookup Table1'

    real_T uDLookupTable2_tableData[3];// Expression: r_lim*[-1:1]
                                          //  Referenced by: '<S175>/1-D Lookup Table2'

    real_T uDLookupTable2_bp01Data[3]; // Expression: [-1:1]
                                          //  Referenced by: '<S175>/1-D Lookup Table2'

    real_T CommandSaturation_UpperSat; // Expression: 25
                                          //  Referenced by: '<S221>/Command Saturation'

    real_T CommandSaturation_LowerSat; // Expression: -25
                                          //  Referenced by: '<S221>/Command Saturation'

    real_T Constant13_Value;           // Expression: 0
                                          //  Referenced by: '<S199>/Constant13'

    real_T Switch12_Threshold;         // Expression: 0
                                          //  Referenced by: '<S199>/Switch12'

    real_T Gain16_Gain;                // Expression: -1
                                          //  Referenced by: '<S199>/Gain16'

    real_T Constant22_Value;           // Expression: 0.000001
                                          //  Referenced by: '<S199>/Constant22'

    real_T Constant21_Value;           // Expression: 0.000001
                                          //  Referenced by: '<S199>/Constant21'

    real_T Constant18_Value;           // Expression: 0
                                          //  Referenced by: '<S199>/Constant18'

    real_T Switch10_Threshold;         // Expression: 0
                                          //  Referenced by: '<S199>/Switch10'

    real_T Constant23_Value;           // Expression: 0.000001
                                          //  Referenced by: '<S199>/Constant23'

    real_T Constant9_Value;            // Expression: 0
                                          //  Referenced by: '<S199>/Constant9'

    real_T Switch5_Threshold;          // Expression: 0
                                          //  Referenced by: '<S199>/Switch5'

    real_T Gain23_Gain;                // Expression: -1
                                          //  Referenced by: '<S199>/Gain23'

    real_T Gmg_Gain[3];                // Expression: [0; 0; 9.81]
                                          //  Referenced by: '<S200>/G=mg'

    real_T Jmax2_Value;                // Expression: 1
                                          //  Referenced by: '<S203>/Jmax2'

    real_T Pi_Value;                   // Expression: pi
                                          //  Referenced by: '<S203>/Pi'

    real_T Saturation_UpperSat;        // Expression: 1
                                          //  Referenced by: '<S5>/Saturation'

    real_T Saturation_LowerSat;        // Expression: 0
                                          //  Referenced by: '<S5>/Saturation'

    real_T Saturation_UpperSat_d;      // Expression: 1
                                          //  Referenced by: '<S203>/Saturation'

    real_T Saturation_LowerSat_i;      // Expression: 0
                                          //  Referenced by: '<S203>/Saturation'

    real_T Constant_Value_e;           // Expression: 0.98
                                          //  Referenced by: '<S203>/Constant'

    real_T Switch1_Threshold_b;        // Expression: 0
                                          //  Referenced by: '<S203>/Switch1'

    real_T Jmax_Value;                 // Expression: 0.95
                                          //  Referenced by: '<S203>/Jmax'

    real_T Gain_Gain_c;                // Expression: 2
                                          //  Referenced by: '<S203>/Gain'

    real_T constant1_Value;            // Expression: 4
                                          //  Referenced by: '<S203>/constant1'

    real_T Pi1_Value;                  // Expression: pi
                                          //  Referenced by: '<S203>/Pi1'

    real_T Gain8_Gain;                 // Expression: 2
                                          //  Referenced by: '<S203>/Gain8'

    real_T Constant1_Value_a;          // Expression: 0
                                          //  Referenced by: '<S203>/Constant1'

    real_T Switch_Threshold_h;         // Expression: 0
                                          //  Referenced by: '<S203>/Switch'

    real_T Gain1_Gain_c;               // Expression: -1
                                          //  Referenced by: '<S180>/Gain1'

    real_T Switch1_Threshold_h;        // Expression: 0
                                          //  Referenced by: '<S201>/Switch1'

    real_T Jmax1_Value;                // Expression: 0.75
                                          //  Referenced by: '<S203>/Jmax1'

    real_T Jmax3_Value;                // Expression: 1
                                          //  Referenced by: '<S203>/Jmax3'

    real_T constant2_Value;            // Expression: 5
                                          //  Referenced by: '<S203>/constant2'

    real_T Pi2_Value;                  // Expression: pi
                                          //  Referenced by: '<S203>/Pi2'

    real_T Gain9_Gain;                 // Expression: 2
                                          //  Referenced by: '<S203>/Gain9'

    real_T Constant2_Value_c;          // Expression: 0
                                          //  Referenced by: '<S203>/Constant2'

    real_T Constant20_Value;           // Expression: 0.000001
                                          //  Referenced by: '<S199>/Constant20'

    real_T Constant19_Value;           // Expression: 0.000001
                                          //  Referenced by: '<S199>/Constant19'

    real_T Constant10_Value;           // Expression: 0
                                          //  Referenced by: '<S199>/Constant10'

    real_T Switch16_Threshold;         // Expression: 0
                                          //  Referenced by: '<S199>/Switch16'

    real_T Constant24_Value;           // Expression: 0.000001
                                          //  Referenced by: '<S199>/Constant24'

    real_T Constant11_Value;           // Expression: 0
                                          //  Referenced by: '<S199>/Constant11'

    real_T Switch11_Threshold;         // Expression: 0
                                          //  Referenced by: '<S199>/Switch11'

    real_T Constant16_Value;           // Expression: 0.000001
                                          //  Referenced by: '<S199>/Constant16'

    real_T Constant5_Value;            // Expression: 0.000001
                                          //  Referenced by: '<S199>/Constant5'

    real_T Constant12_Value;           // Expression: 0
                                          //  Referenced by: '<S199>/Constant12'

    real_T Switch15_Threshold;         // Expression: 0
                                          //  Referenced by: '<S199>/Switch15'

    real_T Switch3_Threshold_l;        // Expression: 0
                                          //  Referenced by: '<S201>/Switch3'

    real_T Gain_Gain_ca;               // Expression: -1
                                          //  Referenced by: '<S175>/Gain'

    real_T Gain1_Gain_fx;              // Expression: -1
                                          //  Referenced by: '<S175>/Gain1'

    real_T aw_eps_Value;               // Expression: 1e-6
                                          //  Referenced by: '<S9>/aw_eps'

    real_T aw_inv_Vref2_Gain;          // Expression: 1/(V_I0^2)
                                          //  Referenced by: '<S9>/aw_inv_Vref2'

    real_T aw_eps_Value_l;             // Expression: 1e-6
                                          //  Referenced by: '<S10>/aw_eps'

    real_T aw_inv_Vref2_Gain_f;        // Expression: 1/(V_I0^2)
                                          //  Referenced by: '<S10>/aw_inv_Vref2'

    real_T aw_eps_Value_f;             // Expression: 1e-6
                                          //  Referenced by: '<S11>/aw_eps'

    real_T aw_inv_Vref2_Gain_d;        // Expression: 1/(V_I0^2)
                                          //  Referenced by: '<S11>/aw_inv_Vref2'

  };

  // Real-time Model Data Structure
  struct RT_MODEL_PX4Controller_ertmod_T {
    const char_T *errorStatus;
    RTWSolverInfo solverInfo;
    X_PX4Controller_ertmodel0827_T *contStates;
    int_T *periodicContStateIndices;
    real_T *periodicContStateRanges;
    real_T *derivs;
    boolean_T *contStateDisabled;
    boolean_T zCCacheNeedsReset;
    boolean_T derivCacheNeedsReset;
    boolean_T CTOutputIncnstWithState;
    real_T odeY[12];
    real_T odeF[4][12];
    ODE4_IntgData intgData;

    //
    //  Sizes:
    //  The following substructure contains sizes information
    //  for many of the model attributes such as inputs, outputs,
    //  dwork, sample times, etc.

    struct {
      int_T numContStates;
      int_T numPeriodicContStates;
      int_T numSampTimes;
    } Sizes;

    //
    //  Timing:
    //  The following substructure contains information regarding
    //  the timing information for the model.

    struct {
      uint32_T clockTick0;
      time_T stepSize0;
      uint32_T clockTick1;
      SimTimeStep simTimeStep;
      boolean_T stopRequestedFlag;
      time_T *t;
      time_T tArray[2];
    } Timing;
  };

  // Tunable parameters
  static P_PX4Controller_ertmodel0827_T PX4Controller_ertmodel0827_P;

  // External inputs
  ExtU_PX4Controller_ertmodel08_T PX4Controller_ertmodel0827_U;

  // External outputs
  ExtY_PX4Controller_ertmodel08_T PX4Controller_ertmodel0827_Y;

  // model initialize function
  void initialize();

  // model step function
  void step();

  // model terminate function
  void terminate();

  // Constructor
  PX4Controller_ertmodel0827ModelClass();

  // Destructor
  ~PX4Controller_ertmodel0827ModelClass();

  // Root-level structure-based inputs set method

  // Root inports set method
  void setExternalInputs(const ExtU_PX4Controller_ertmodel08_T
    * pExtU_PX4Controller_ertmodel08_T)
  {
    PX4Controller_ertmodel0827_U = *pExtU_PX4Controller_ertmodel08_T;
  }

  // Root-level structure-based outputs get method

  // Root outports get method
  const PX4Controller_ertmodel0827ModelClass::ExtY_PX4Controller_ertmodel08_T
    & getExternalOutputs() const
  {
    return PX4Controller_ertmodel0827_Y;
  }

  // Real-Time Model get method
  PX4Controller_ertmodel0827ModelClass::RT_MODEL_PX4Controller_ertmod_T * getRTM
    ();

  // private data and function members
 private:
  // Block signals
  B_PX4Controller_ertmodel0827_T PX4Controller_ertmodel0827_B;

  // Block states
  DW_PX4Controller_ertmodel0827_T PX4Controller_ertmodel0827_DW;
  X_PX4Controller_ertmodel0827_T PX4Controller_ertmodel0827_X;// Block continuous states 
  PeriodicIndX_PX4Controller_er_T PX4Controller_ertm_PeriodicIndX;// Block periodic continuous states 
  PeriodicRngX_PX4Controller_er_T PX4Controller_ertm_PeriodicRngX;

  // Real-Time Model
  RT_MODEL_PX4Controller_ertmod_T PX4Controller_ertmodel0827_M;

  // Continuous states update member function
  void rt_ertODEUpdateContinuousStates(RTWSolverInfo *si );

  // Derivatives member function
  void PX4Controller_ertmodel0827_derivatives();
};

//-
//  These blocks were eliminated from the model due to optimizations:
//
//  Block '<Root>/Add' : Unused code path elimination
//  Block '<S187>/Unit Conversion' : Unused code path elimination
//  Block '<S202>/P0' : Unused code path elimination
//  Block '<S202>/Product2' : Unused code path elimination
//  Block '<S202>/a' : Unused code path elimination
//  Block '<S202>/gamma*R' : Unused code path elimination
//  Block '<Root>/Sqrt' : Unused code path elimination
//  Block '<Root>/Square' : Unused code path elimination
//  Block '<Root>/Square1' : Unused code path elimination
//  Block '<Root>/Square2' : Unused code path elimination
//  Block '<Root>/CodegenInputHold_T_cmd' : Eliminated since input and output rates are identical
//  Block '<Root>/CodegenInputHold_phi_sp' : Eliminated since input and output rates are identical
//  Block '<Root>/CodegenInputHold_theta_sp' : Eliminated since input and output rates are identical
//  Block '<Root>/CodegenInputHold_w_speed' : Eliminated since input and output rates are identical
//  Block '<Root>/CodegenOutputHold_alpha' : Eliminated since input and output rates are identical
//  Block '<Root>/CodegenOutputHold_beta' : Eliminated since input and output rates are identical
//  Block '<S170>/Reshape (9) to [3x3] column-major' : Reshape block reduction
//  Block '<S174>/Reshape' : Reshape block reduction
//  Block '<S173>/Reshape (9) to [3x3] column-major' : Reshape block reduction
//  Block '<S191>/Reshape (9) to [3x3] column-major' : Reshape block reduction
//  Block '<S193>/Reshape1' : Reshape block reduction
//  Block '<S193>/Reshape2' : Reshape block reduction
//  Block '<S194>/Reshape1' : Reshape block reduction
//  Block '<S194>/Reshape2' : Reshape block reduction
//  Block '<S182>/Reshape' : Reshape block reduction
//  Block '<S182>/Reshape1' : Reshape block reduction
//  Block '<S185>/Unit Conversion' : Eliminated nontunable gain of 1
//  Block '<S186>/Unit Conversion' : Eliminated nontunable gain of 1
//  Block '<S188>/Reshape1' : Reshape block reduction
//  Block '<S188>/Reshape2' : Reshape block reduction
//  Block '<S217>/Reshape (9) to [3x3] column-major' : Reshape block reduction
//  Block '<S5>/CodegenInputHold_delta_t' : Eliminated since input and output rates are identical


//-
//  The generated code includes comments that allow you to trace directly
//  back to the appropriate location in the model.  The basic format
//  is <system>/block_name, where system is the system number (uniquely
//  assigned by Simulink) and block_name is the name of the block.
//
//  Use the MATLAB hilite_system command to trace the generated code back
//  to the model.  For example,
//
//  hilite_system('<S3>')    - opens system 3
//  hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
//
//  Here is the system hierarchy for this model
//
//  '<Root>' : 'PX4Controller_ertmodel0827'
//  '<S1>'   : 'PX4Controller_ertmodel0827/Angle Conversion'
//  '<S2>'   : 'PX4Controller_ertmodel0827/Angle Conversion1'
//  '<S3>'   : 'PX4Controller_ertmodel0827/InnerLoopControl'
//  '<S4>'   : 'PX4Controller_ertmodel0827/MixControlMatrix'
//  '<S5>'   : 'PX4Controller_ertmodel0827/Plant'
//  '<S6>'   : 'PX4Controller_ertmodel0827/InnerLoopControl/FW_phi'
//  '<S7>'   : 'PX4Controller_ertmodel0827/InnerLoopControl/FW_psi'
//  '<S8>'   : 'PX4Controller_ertmodel0827/InnerLoopControl/FW_theta'
//  '<S9>'   : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem'
//  '<S10>'  : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem1'
//  '<S11>'  : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem4'
//  '<S12>'  : 'PX4Controller_ertmodel0827/InnerLoopControl/theta_phi_psi_to_OMEGA'
//  '<S13>'  : 'PX4Controller_ertmodel0827/InnerLoopControl/turn_coordination'
//  '<S14>'  : 'PX4Controller_ertmodel0827/InnerLoopControl/FW_phi/FFScaler'
//  '<S15>'  : 'PX4Controller_ertmodel0827/InnerLoopControl/FW_phi/FeedForward'
//  '<S16>'  : 'PX4Controller_ertmodel0827/InnerLoopControl/FW_psi/FFScaler'
//  '<S17>'  : 'PX4Controller_ertmodel0827/InnerLoopControl/FW_psi/FeedForward'
//  '<S18>'  : 'PX4Controller_ertmodel0827/InnerLoopControl/FW_theta/FFScaler'
//  '<S19>'  : 'PX4Controller_ertmodel0827/InnerLoopControl/FW_theta/FeedForward'
//  '<S20>'  : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem/Discrete PD Controller'
//  '<S21>'  : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem/PIScaler'
//  '<S22>'  : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem/Discrete PD Controller/Anti-windup'
//  '<S23>'  : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem/Discrete PD Controller/D Gain'
//  '<S24>'  : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem/Discrete PD Controller/Filter'
//  '<S25>'  : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem/Discrete PD Controller/Filter ICs'
//  '<S26>'  : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem/Discrete PD Controller/I Gain'
//  '<S27>'  : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem/Discrete PD Controller/Ideal P Gain'
//  '<S28>'  : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem/Discrete PD Controller/Ideal P Gain Fdbk'
//  '<S29>'  : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem/Discrete PD Controller/Integrator'
//  '<S30>'  : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem/Discrete PD Controller/Integrator ICs'
//  '<S31>'  : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem/Discrete PD Controller/N Copy'
//  '<S32>'  : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem/Discrete PD Controller/N Gain'
//  '<S33>'  : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem/Discrete PD Controller/P Copy'
//  '<S34>'  : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem/Discrete PD Controller/Parallel P Gain'
//  '<S35>'  : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem/Discrete PD Controller/Reset Signal'
//  '<S36>'  : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem/Discrete PD Controller/Saturation'
//  '<S37>'  : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem/Discrete PD Controller/Saturation Fdbk'
//  '<S38>'  : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem/Discrete PD Controller/Sum'
//  '<S39>'  : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem/Discrete PD Controller/Sum Fdbk'
//  '<S40>'  : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem/Discrete PD Controller/Tracking Mode'
//  '<S41>'  : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem/Discrete PD Controller/Tracking Mode Sum'
//  '<S42>'  : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem/Discrete PD Controller/Tsamp - Integral'
//  '<S43>'  : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem/Discrete PD Controller/Tsamp - Ngain'
//  '<S44>'  : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem/Discrete PD Controller/postSat Signal'
//  '<S45>'  : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem/Discrete PD Controller/preSat Signal'
//  '<S46>'  : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem/Discrete PD Controller/Anti-windup/Back Calculation'
//  '<S47>'  : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem/Discrete PD Controller/D Gain/Internal Parameters'
//  '<S48>'  : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem/Discrete PD Controller/Filter/Disc. Forward Euler Filter'
//  '<S49>'  : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem/Discrete PD Controller/Filter ICs/Internal IC - Filter'
//  '<S50>'  : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem/Discrete PD Controller/I Gain/Internal Parameters'
//  '<S51>'  : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem/Discrete PD Controller/Ideal P Gain/Passthrough'
//  '<S52>'  : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem/Discrete PD Controller/Ideal P Gain Fdbk/Disabled'
//  '<S53>'  : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem/Discrete PD Controller/Integrator/Discrete'
//  '<S54>'  : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem/Discrete PD Controller/Integrator ICs/Internal IC'
//  '<S55>'  : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem/Discrete PD Controller/N Copy/Disabled'
//  '<S56>'  : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem/Discrete PD Controller/N Gain/Internal Parameters'
//  '<S57>'  : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem/Discrete PD Controller/P Copy/Disabled'
//  '<S58>'  : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem/Discrete PD Controller/Parallel P Gain/Internal Parameters'
//  '<S59>'  : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem/Discrete PD Controller/Reset Signal/Disabled'
//  '<S60>'  : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem/Discrete PD Controller/Saturation/Enabled'
//  '<S61>'  : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem/Discrete PD Controller/Saturation Fdbk/Disabled'
//  '<S62>'  : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem/Discrete PD Controller/Sum/Sum_PID'
//  '<S63>'  : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem/Discrete PD Controller/Sum Fdbk/Disabled'
//  '<S64>'  : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem/Discrete PD Controller/Tracking Mode/Enabled'
//  '<S65>'  : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem/Discrete PD Controller/Tracking Mode Sum/Tracking Mode'
//  '<S66>'  : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem/Discrete PD Controller/Tsamp - Integral/Passthrough'
//  '<S67>'  : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem/Discrete PD Controller/Tsamp - Ngain/Passthrough'
//  '<S68>'  : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem/Discrete PD Controller/postSat Signal/Forward_Path'
//  '<S69>'  : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem/Discrete PD Controller/preSat Signal/Forward_Path'
//  '<S70>'  : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem1/Discrete PD Controller'
//  '<S71>'  : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem1/PIScaler'
//  '<S72>'  : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem1/Discrete PD Controller/Anti-windup'
//  '<S73>'  : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem1/Discrete PD Controller/D Gain'
//  '<S74>'  : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem1/Discrete PD Controller/Filter'
//  '<S75>'  : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem1/Discrete PD Controller/Filter ICs'
//  '<S76>'  : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem1/Discrete PD Controller/I Gain'
//  '<S77>'  : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem1/Discrete PD Controller/Ideal P Gain'
//  '<S78>'  : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem1/Discrete PD Controller/Ideal P Gain Fdbk'
//  '<S79>'  : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem1/Discrete PD Controller/Integrator'
//  '<S80>'  : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem1/Discrete PD Controller/Integrator ICs'
//  '<S81>'  : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem1/Discrete PD Controller/N Copy'
//  '<S82>'  : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem1/Discrete PD Controller/N Gain'
//  '<S83>'  : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem1/Discrete PD Controller/P Copy'
//  '<S84>'  : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem1/Discrete PD Controller/Parallel P Gain'
//  '<S85>'  : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem1/Discrete PD Controller/Reset Signal'
//  '<S86>'  : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem1/Discrete PD Controller/Saturation'
//  '<S87>'  : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem1/Discrete PD Controller/Saturation Fdbk'
//  '<S88>'  : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem1/Discrete PD Controller/Sum'
//  '<S89>'  : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem1/Discrete PD Controller/Sum Fdbk'
//  '<S90>'  : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem1/Discrete PD Controller/Tracking Mode'
//  '<S91>'  : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem1/Discrete PD Controller/Tracking Mode Sum'
//  '<S92>'  : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem1/Discrete PD Controller/Tsamp - Integral'
//  '<S93>'  : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem1/Discrete PD Controller/Tsamp - Ngain'
//  '<S94>'  : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem1/Discrete PD Controller/postSat Signal'
//  '<S95>'  : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem1/Discrete PD Controller/preSat Signal'
//  '<S96>'  : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem1/Discrete PD Controller/Anti-windup/Back Calculation'
//  '<S97>'  : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem1/Discrete PD Controller/D Gain/Internal Parameters'
//  '<S98>'  : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem1/Discrete PD Controller/Filter/Disc. Forward Euler Filter'
//  '<S99>'  : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem1/Discrete PD Controller/Filter ICs/Internal IC - Filter'
//  '<S100>' : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem1/Discrete PD Controller/I Gain/Internal Parameters'
//  '<S101>' : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem1/Discrete PD Controller/Ideal P Gain/Passthrough'
//  '<S102>' : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem1/Discrete PD Controller/Ideal P Gain Fdbk/Disabled'
//  '<S103>' : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem1/Discrete PD Controller/Integrator/Discrete'
//  '<S104>' : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem1/Discrete PD Controller/Integrator ICs/Internal IC'
//  '<S105>' : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem1/Discrete PD Controller/N Copy/Disabled'
//  '<S106>' : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem1/Discrete PD Controller/N Gain/Internal Parameters'
//  '<S107>' : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem1/Discrete PD Controller/P Copy/Disabled'
//  '<S108>' : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem1/Discrete PD Controller/Parallel P Gain/Internal Parameters'
//  '<S109>' : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem1/Discrete PD Controller/Reset Signal/Disabled'
//  '<S110>' : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem1/Discrete PD Controller/Saturation/Enabled'
//  '<S111>' : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem1/Discrete PD Controller/Saturation Fdbk/Disabled'
//  '<S112>' : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem1/Discrete PD Controller/Sum/Sum_PID'
//  '<S113>' : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem1/Discrete PD Controller/Sum Fdbk/Disabled'
//  '<S114>' : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem1/Discrete PD Controller/Tracking Mode/Enabled'
//  '<S115>' : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem1/Discrete PD Controller/Tracking Mode Sum/Tracking Mode'
//  '<S116>' : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem1/Discrete PD Controller/Tsamp - Integral/Passthrough'
//  '<S117>' : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem1/Discrete PD Controller/Tsamp - Ngain/Passthrough'
//  '<S118>' : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem1/Discrete PD Controller/postSat Signal/Forward_Path'
//  '<S119>' : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem1/Discrete PD Controller/preSat Signal/Forward_Path'
//  '<S120>' : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem4/Discrete PD Controller'
//  '<S121>' : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem4/PIScaler'
//  '<S122>' : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem4/Discrete PD Controller/Anti-windup'
//  '<S123>' : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem4/Discrete PD Controller/D Gain'
//  '<S124>' : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem4/Discrete PD Controller/Filter'
//  '<S125>' : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem4/Discrete PD Controller/Filter ICs'
//  '<S126>' : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem4/Discrete PD Controller/I Gain'
//  '<S127>' : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem4/Discrete PD Controller/Ideal P Gain'
//  '<S128>' : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem4/Discrete PD Controller/Ideal P Gain Fdbk'
//  '<S129>' : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem4/Discrete PD Controller/Integrator'
//  '<S130>' : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem4/Discrete PD Controller/Integrator ICs'
//  '<S131>' : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem4/Discrete PD Controller/N Copy'
//  '<S132>' : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem4/Discrete PD Controller/N Gain'
//  '<S133>' : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem4/Discrete PD Controller/P Copy'
//  '<S134>' : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem4/Discrete PD Controller/Parallel P Gain'
//  '<S135>' : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem4/Discrete PD Controller/Reset Signal'
//  '<S136>' : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem4/Discrete PD Controller/Saturation'
//  '<S137>' : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem4/Discrete PD Controller/Saturation Fdbk'
//  '<S138>' : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem4/Discrete PD Controller/Sum'
//  '<S139>' : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem4/Discrete PD Controller/Sum Fdbk'
//  '<S140>' : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem4/Discrete PD Controller/Tracking Mode'
//  '<S141>' : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem4/Discrete PD Controller/Tracking Mode Sum'
//  '<S142>' : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem4/Discrete PD Controller/Tsamp - Integral'
//  '<S143>' : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem4/Discrete PD Controller/Tsamp - Ngain'
//  '<S144>' : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem4/Discrete PD Controller/postSat Signal'
//  '<S145>' : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem4/Discrete PD Controller/preSat Signal'
//  '<S146>' : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem4/Discrete PD Controller/Anti-windup/Back Calculation'
//  '<S147>' : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem4/Discrete PD Controller/D Gain/Internal Parameters'
//  '<S148>' : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem4/Discrete PD Controller/Filter/Disc. Forward Euler Filter'
//  '<S149>' : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem4/Discrete PD Controller/Filter ICs/Internal IC - Filter'
//  '<S150>' : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem4/Discrete PD Controller/I Gain/Internal Parameters'
//  '<S151>' : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem4/Discrete PD Controller/Ideal P Gain/Passthrough'
//  '<S152>' : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem4/Discrete PD Controller/Ideal P Gain Fdbk/Disabled'
//  '<S153>' : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem4/Discrete PD Controller/Integrator/Discrete'
//  '<S154>' : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem4/Discrete PD Controller/Integrator ICs/Internal IC'
//  '<S155>' : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem4/Discrete PD Controller/N Copy/Disabled'
//  '<S156>' : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem4/Discrete PD Controller/N Gain/Internal Parameters'
//  '<S157>' : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem4/Discrete PD Controller/P Copy/Disabled'
//  '<S158>' : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem4/Discrete PD Controller/Parallel P Gain/Internal Parameters'
//  '<S159>' : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem4/Discrete PD Controller/Reset Signal/Disabled'
//  '<S160>' : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem4/Discrete PD Controller/Saturation/Enabled'
//  '<S161>' : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem4/Discrete PD Controller/Saturation Fdbk/Disabled'
//  '<S162>' : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem4/Discrete PD Controller/Sum/Sum_PID'
//  '<S163>' : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem4/Discrete PD Controller/Sum Fdbk/Disabled'
//  '<S164>' : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem4/Discrete PD Controller/Tracking Mode/Enabled'
//  '<S165>' : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem4/Discrete PD Controller/Tracking Mode Sum/Tracking Mode'
//  '<S166>' : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem4/Discrete PD Controller/Tsamp - Integral/Passthrough'
//  '<S167>' : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem4/Discrete PD Controller/Tsamp - Ngain/Passthrough'
//  '<S168>' : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem4/Discrete PD Controller/postSat Signal/Forward_Path'
//  '<S169>' : 'PX4Controller_ertmodel0827/InnerLoopControl/Subsystem4/Discrete PD Controller/preSat Signal/Forward_Path'
//  '<S170>' : 'PX4Controller_ertmodel0827/InnerLoopControl/theta_phi_psi_to_OMEGA/Create 3x3 Matrix'
//  '<S171>' : 'PX4Controller_ertmodel0827/MixControlMatrix/1//2*rou*V_ref^2*S'
//  '<S172>' : 'PX4Controller_ertmodel0827/MixControlMatrix/Invert  3x3 Matrix'
//  '<S173>' : 'PX4Controller_ertmodel0827/MixControlMatrix/Mix Control Matrix'
//  '<S174>' : 'PX4Controller_ertmodel0827/MixControlMatrix/Invert  3x3 Matrix/Determinant of 3x3 Matrix'
//  '<S175>' : 'PX4Controller_ertmodel0827/Plant/AVANTI__UAV'
//  '<S176>' : 'PX4Controller_ertmodel0827/Plant/AVANTI__UAV/6DOF (Euler Angles)'
//  '<S177>' : 'PX4Controller_ertmodel0827/Plant/AVANTI__UAV/Angle Conversion'
//  '<S178>' : 'PX4Controller_ertmodel0827/Plant/AVANTI__UAV/Angle Conversion1'
//  '<S179>' : 'PX4Controller_ertmodel0827/Plant/AVANTI__UAV/Angle Conversion2'
//  '<S180>' : 'PX4Controller_ertmodel0827/Plant/AVANTI__UAV/Forces and Movement'
//  '<S181>' : 'PX4Controller_ertmodel0827/Plant/AVANTI__UAV/6DOF (Euler Angles)/Calculate DCM & Euler Angles'
//  '<S182>' : 'PX4Controller_ertmodel0827/Plant/AVANTI__UAV/6DOF (Euler Angles)/Calculate omega_dot'
//  '<S183>' : 'PX4Controller_ertmodel0827/Plant/AVANTI__UAV/6DOF (Euler Angles)/Determine Force,  Mass & Inertia'
//  '<S184>' : 'PX4Controller_ertmodel0827/Plant/AVANTI__UAV/6DOF (Euler Angles)/Vbxw'
//  '<S185>' : 'PX4Controller_ertmodel0827/Plant/AVANTI__UAV/6DOF (Euler Angles)/Velocity Conversion'
//  '<S186>' : 'PX4Controller_ertmodel0827/Plant/AVANTI__UAV/6DOF (Euler Angles)/Velocity Conversion1'
//  '<S187>' : 'PX4Controller_ertmodel0827/Plant/AVANTI__UAV/6DOF (Euler Angles)/Velocity Conversion2'
//  '<S188>' : 'PX4Controller_ertmodel0827/Plant/AVANTI__UAV/6DOF (Euler Angles)/transform to Inertial axes '
//  '<S189>' : 'PX4Controller_ertmodel0827/Plant/AVANTI__UAV/6DOF (Euler Angles)/Calculate DCM & Euler Angles/Rotation Angles to Direction Cosine Matrix'
//  '<S190>' : 'PX4Controller_ertmodel0827/Plant/AVANTI__UAV/6DOF (Euler Angles)/Calculate DCM & Euler Angles/phidot thetadot psidot'
//  '<S191>' : 'PX4Controller_ertmodel0827/Plant/AVANTI__UAV/6DOF (Euler Angles)/Calculate DCM & Euler Angles/Rotation Angles to Direction Cosine Matrix/Create 3x3 Matrix'
//  '<S192>' : 'PX4Controller_ertmodel0827/Plant/AVANTI__UAV/6DOF (Euler Angles)/Calculate omega_dot/3x3 Cross Product'
//  '<S193>' : 'PX4Controller_ertmodel0827/Plant/AVANTI__UAV/6DOF (Euler Angles)/Calculate omega_dot/I x w'
//  '<S194>' : 'PX4Controller_ertmodel0827/Plant/AVANTI__UAV/6DOF (Euler Angles)/Calculate omega_dot/I x w1'
//  '<S195>' : 'PX4Controller_ertmodel0827/Plant/AVANTI__UAV/6DOF (Euler Angles)/Calculate omega_dot/3x3 Cross Product/Subsystem'
//  '<S196>' : 'PX4Controller_ertmodel0827/Plant/AVANTI__UAV/6DOF (Euler Angles)/Calculate omega_dot/3x3 Cross Product/Subsystem1'
//  '<S197>' : 'PX4Controller_ertmodel0827/Plant/AVANTI__UAV/6DOF (Euler Angles)/Vbxw/Subsystem'
//  '<S198>' : 'PX4Controller_ertmodel0827/Plant/AVANTI__UAV/6DOF (Euler Angles)/Vbxw/Subsystem1'
//  '<S199>' : 'PX4Controller_ertmodel0827/Plant/AVANTI__UAV/Forces and Movement/Aero_Dynamic_Model'
//  '<S200>' : 'PX4Controller_ertmodel0827/Plant/AVANTI__UAV/Forces and Movement/Gravity_Forces_and_Movement'
//  '<S201>' : 'PX4Controller_ertmodel0827/Plant/AVANTI__UAV/Forces and Movement/GroundEffect'
//  '<S202>' : 'PX4Controller_ertmodel0827/Plant/AVANTI__UAV/Forces and Movement/ISA Atmosphere Model'
//  '<S203>' : 'PX4Controller_ertmodel0827/Plant/AVANTI__UAV/Forces and Movement/Motor_Model'
//  '<S204>' : 'PX4Controller_ertmodel0827/Plant/AVANTI__UAV/Forces and Movement/control_surfaces'
//  '<S205>' : 'PX4Controller_ertmodel0827/Plant/AVANTI__UAV/Forces and Movement/Aero_Dynamic_Model/0.018u^2+0.003583u+0.00128'
//  '<S206>' : 'PX4Controller_ertmodel0827/Plant/AVANTI__UAV/Forces and Movement/Aero_Dynamic_Model/Degrees to Radians'
//  '<S207>' : 'PX4Controller_ertmodel0827/Plant/AVANTI__UAV/Forces and Movement/Aero_Dynamic_Model/Direction Cosine Matrix Body to Wind'
//  '<S208>' : 'PX4Controller_ertmodel0827/Plant/AVANTI__UAV/Forces and Movement/Aero_Dynamic_Model/Direction Cosine Matrix Body to Wind/A11'
//  '<S209>' : 'PX4Controller_ertmodel0827/Plant/AVANTI__UAV/Forces and Movement/Aero_Dynamic_Model/Direction Cosine Matrix Body to Wind/A12'
//  '<S210>' : 'PX4Controller_ertmodel0827/Plant/AVANTI__UAV/Forces and Movement/Aero_Dynamic_Model/Direction Cosine Matrix Body to Wind/A13'
//  '<S211>' : 'PX4Controller_ertmodel0827/Plant/AVANTI__UAV/Forces and Movement/Aero_Dynamic_Model/Direction Cosine Matrix Body to Wind/A21'
//  '<S212>' : 'PX4Controller_ertmodel0827/Plant/AVANTI__UAV/Forces and Movement/Aero_Dynamic_Model/Direction Cosine Matrix Body to Wind/A22'
//  '<S213>' : 'PX4Controller_ertmodel0827/Plant/AVANTI__UAV/Forces and Movement/Aero_Dynamic_Model/Direction Cosine Matrix Body to Wind/A23'
//  '<S214>' : 'PX4Controller_ertmodel0827/Plant/AVANTI__UAV/Forces and Movement/Aero_Dynamic_Model/Direction Cosine Matrix Body to Wind/A31'
//  '<S215>' : 'PX4Controller_ertmodel0827/Plant/AVANTI__UAV/Forces and Movement/Aero_Dynamic_Model/Direction Cosine Matrix Body to Wind/A32'
//  '<S216>' : 'PX4Controller_ertmodel0827/Plant/AVANTI__UAV/Forces and Movement/Aero_Dynamic_Model/Direction Cosine Matrix Body to Wind/A33'
//  '<S217>' : 'PX4Controller_ertmodel0827/Plant/AVANTI__UAV/Forces and Movement/Aero_Dynamic_Model/Direction Cosine Matrix Body to Wind/Create Transformation Matrix'
//  '<S218>' : 'PX4Controller_ertmodel0827/Plant/AVANTI__UAV/Forces and Movement/control_surfaces/Angle Conversion'
//  '<S219>' : 'PX4Controller_ertmodel0827/Plant/AVANTI__UAV/Forces and Movement/control_surfaces/alieron'
//  '<S220>' : 'PX4Controller_ertmodel0827/Plant/AVANTI__UAV/Forces and Movement/control_surfaces/elevator'
//  '<S221>' : 'PX4Controller_ertmodel0827/Plant/AVANTI__UAV/Forces and Movement/control_surfaces/rudder'

#endif                              // RTW_HEADER_PX4Controller_ertmodel0827_h_

//
// File trailer for generated code.
//
// [EOF]
//
