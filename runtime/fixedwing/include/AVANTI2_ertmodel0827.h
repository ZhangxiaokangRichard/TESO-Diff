//
// File: AVANTI2_ertmodel0827.h
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
#ifndef RTW_HEADER_AVANTI2_ertmodel0827_h_
#define RTW_HEADER_AVANTI2_ertmodel0827_h_
#include <cstring>
#include <cmath>
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "AVANTI2_ertmodel0827_types.h"
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

// Class declaration for model AVANTI2_ertmodel0827
class AVANTI2_ertmodel0827ModelClass {
  // public data and function members
 public:
  // Block signals (default storage)
  struct B_AVANTI2_ertmodel0827_T {
    real_T wind[3];                    // '<Root>/wind'
    real_T Gain46;                     // '<S25>/Gain46'
    real_T Gain36;                     // '<S25>/Gain36'
    real_T Gain27;                     // '<S25>/Gain27'
    real_T Gain10;                     // '<S25>/Gain10'
    real_T Switch1;                    // '<S29>/Switch1'
    real_T Square;                     // '<S29>/Square'
    real_T Sum[3];                     // '<S2>/Sum'
    real_T Square1;                    // '<S29>/Square1'
    real_T Gain6;                      // '<S25>/Gain6'
    real_T Gain39;                     // '<S25>/Gain39'
    real_T Gain15;                     // '<S25>/Gain15'
    real_T Gain19;                     // '<S25>/Gain19'
    real_T Gain20;                     // '<S25>/Gain20'
    real_T Product2[3];                // '<S8>/Product2'
    real_T Product[3];                 // '<S14>/Product'
    real_T TmpSignalConversionAtphithetaps[3];// '<S7>/phidot thetadot psidot'
  };

  // Block states (default storage) for system '<Root>'
  struct DW_AVANTI2_ertmodel0827_T {
    real_T Product2_DWORK4[9];         // '<S8>/Product2'
  };

  // Continuous states (default storage)
  struct X_AVANTI2_ertmodel0827_T {
    real_T phithetapsi_CSTATE[3];      // '<S7>/phi theta psi'
    real_T ubvbwb_CSTATE[3];           // '<S2>/ub,vb,wb'
    real_T xeyeze_CSTATE[3];           // '<S2>/xe,ye,ze'
    real_T pqr_CSTATE[3];              // '<S2>/p,q,r '
  };

  // Periodic continuous state vector (global)
  typedef int_T PeriodicIndX_AVANTI2_ertmodel_T[3];
  typedef real_T PeriodicRngX_AVANTI2_ertmodel_T[6];

  // State derivatives (default storage)
  struct XDot_AVANTI2_ertmodel0827_T {
    real_T phithetapsi_CSTATE[3];      // '<S7>/phi theta psi'
    real_T ubvbwb_CSTATE[3];           // '<S2>/ub,vb,wb'
    real_T xeyeze_CSTATE[3];           // '<S2>/xe,ye,ze'
    real_T pqr_CSTATE[3];              // '<S2>/p,q,r '
  };

  // State disabled
  struct XDis_AVANTI2_ertmodel0827_T {
    boolean_T phithetapsi_CSTATE[3];   // '<S7>/phi theta psi'
    boolean_T ubvbwb_CSTATE[3];        // '<S2>/ub,vb,wb'
    boolean_T xeyeze_CSTATE[3];        // '<S2>/xe,ye,ze'
    boolean_T pqr_CSTATE[3];           // '<S2>/p,q,r '
  };

  // Invariant block signals (default storage)
  struct ConstB_AVANTI2_ertmodel0827_T {
    real_T VectorConcatenate[18];      // '<S9>/Vector Concatenate'
    real_T Selector[9];                // '<S8>/Selector'
    real_T Selector1[9];               // '<S8>/Selector1'
    real_T Selector2[9];               // '<S8>/Selector2'
    real_T Gain1;                      // '<S32>/Gain1'
    real_T Gmg[3];                     // '<S26>/G=mg'
    real_T Divide;                     // '<S29>/Divide'
    real_T Sqrt;                       // '<S29>/Sqrt'
    real_T Gain;                       // '<S29>/Gain'
    real_T Gain8;                      // '<S29>/Gain8'
    real_T Gain9;                      // '<S29>/Gain9'
    real_T Power1;                     // '<S29>/Power1'
    real_T Power2;                     // '<S29>/Power2'
  };

  // Constant parameters (default storage)
  struct ConstP_AVANTI2_ertmodel0827_T {
    // Expression: e_lim*[-1:1]
    //  Referenced by: '<S1>/1-D Lookup Table'

    real_T uDLookupTable_tableData[3];

    // Pooled Parameter (Expression: [-1:1])
    //  Referenced by:
    //    '<S1>/1-D Lookup Table'
    //    '<S1>/1-D Lookup Table1'
    //    '<S1>/1-D Lookup Table2'

    real_T pooled11[3];

    // Expression: a_lim*[-1:1]
    //  Referenced by: '<S1>/1-D Lookup Table1'

    real_T uDLookupTable1_tableData[3];

    // Expression: r_lim*[-1:1]
    //  Referenced by: '<S1>/1-D Lookup Table2'

    real_T uDLookupTable2_tableData[3];
  };

  // External inputs (root inport signals with default storage)
  struct ExtU_AVANTI2_ertmodel0827_T {
    real_T delta_e;                    // '<Root>/delta_e'
    real_T delta_a;                    // '<Root>/delta_a'
    real_T delta_r;                    // '<Root>/delta_r'
    real_T delta_t;                    // '<Root>/delta_t'
    real_T w_x;                        // '<Root>/w_x'
    real_T w_y;                        // '<Root>/w_y'
    real_T w_z;                        // '<Root>/w_z'
  };

  // External outputs (root outports fed by signals with default storage)
  struct ExtY_AVANTI2_ertmodel0827_T {
    real_T alpha;                      // '<Root>/alpha'
    real_T beta;                       // '<Root>/beta'
    real_T NED[9];                     // '<Root>/NED'
    real_T BODY[12];                   // '<Root>/BODY'
  };

  // Real-time Model Data Structure
  struct RT_MODEL_AVANTI2_ertmodel0827_T {
    const char_T *errorStatus;
    RTWSolverInfo solverInfo;
    X_AVANTI2_ertmodel0827_T *contStates;
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

  // External inputs
  ExtU_AVANTI2_ertmodel0827_T AVANTI2_ertmodel0827_U;

  // External outputs
  ExtY_AVANTI2_ertmodel0827_T AVANTI2_ertmodel0827_Y;

  // model initialize function
  void initialize();

  // model step function
  void step();

  // model terminate function
  void terminate();

  // Constructor
  AVANTI2_ertmodel0827ModelClass();

  // Destructor
  ~AVANTI2_ertmodel0827ModelClass();

  // Root-level structure-based inputs set method

  // Root inports set method
  void setExternalInputs(const ExtU_AVANTI2_ertmodel0827_T
    * pExtU_AVANTI2_ertmodel0827_T)
  {
    AVANTI2_ertmodel0827_U = *pExtU_AVANTI2_ertmodel0827_T;
  }

  // Root-level structure-based outputs get method

  // Root outports get method
  const AVANTI2_ertmodel0827ModelClass::ExtY_AVANTI2_ertmodel0827_T
    & getExternalOutputs() const
  {
    return AVANTI2_ertmodel0827_Y;
  }

  // Real-Time Model get method
  AVANTI2_ertmodel0827ModelClass::RT_MODEL_AVANTI2_ertmodel0827_T * getRTM();

  // private data and function members
 private:
  // Block signals
  B_AVANTI2_ertmodel0827_T AVANTI2_ertmodel0827_B;

  // Block states
  DW_AVANTI2_ertmodel0827_T AVANTI2_ertmodel0827_DW;
  X_AVANTI2_ertmodel0827_T AVANTI2_ertmodel0827_X;// Block continuous states
  PeriodicIndX_AVANTI2_ertmodel_T AVANTI2_ertmodel08_PeriodicIndX;// Block periodic continuous states 
  PeriodicRngX_AVANTI2_ertmodel_T AVANTI2_ertmodel08_PeriodicRngX;

  // Real-Time Model
  RT_MODEL_AVANTI2_ertmodel0827_T AVANTI2_ertmodel0827_M;

  // Continuous states update member function
  void rt_ertODEUpdateContinuousStates(RTWSolverInfo *si );

  // Derivatives member function
  void AVANTI2_ertmodel0827_derivatives();
};

extern const AVANTI2_ertmodel0827ModelClass::ConstB_AVANTI2_ertmodel0827_T
  AVANTI2_ertmodel0827_ConstB;         // constant block i/o

// Constant parameters (default storage)
extern const AVANTI2_ertmodel0827ModelClass::ConstP_AVANTI2_ertmodel0827_T
  AVANTI2_ertmodel0827_ConstP;

//-
//  These blocks were eliminated from the model due to optimizations:
//
//  Block '<S13>/Unit Conversion' : Unused code path elimination
//  Block '<S28>/P0' : Unused code path elimination
//  Block '<S28>/Product2' : Unused code path elimination
//  Block '<S28>/a' : Unused code path elimination
//  Block '<S28>/gamma*R' : Unused code path elimination
//  Block '<S17>/Reshape (9) to [3x3] column-major' : Reshape block reduction
//  Block '<S19>/Reshape1' : Reshape block reduction
//  Block '<S19>/Reshape2' : Reshape block reduction
//  Block '<S20>/Reshape1' : Reshape block reduction
//  Block '<S20>/Reshape2' : Reshape block reduction
//  Block '<S8>/Reshape' : Reshape block reduction
//  Block '<S8>/Reshape1' : Reshape block reduction
//  Block '<S11>/Unit Conversion' : Eliminated nontunable gain of 1
//  Block '<S12>/Unit Conversion' : Eliminated nontunable gain of 1
//  Block '<S14>/Reshape1' : Reshape block reduction
//  Block '<S14>/Reshape2' : Reshape block reduction
//  Block '<S43>/Reshape (9) to [3x3] column-major' : Reshape block reduction
//  Block '<Root>/CodegenInputHold_delta_a' : Eliminated since input and output rates are identical
//  Block '<Root>/CodegenInputHold_delta_e' : Eliminated since input and output rates are identical
//  Block '<Root>/CodegenInputHold_delta_r' : Eliminated since input and output rates are identical
//  Block '<Root>/CodegenInputHold_delta_t' : Eliminated since input and output rates are identical


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
//  '<Root>' : 'AVANTI2_ertmodel0827'
//  '<S1>'   : 'AVANTI2_ertmodel0827/AVANTI__UAV'
//  '<S2>'   : 'AVANTI2_ertmodel0827/AVANTI__UAV/6DOF (Euler Angles)'
//  '<S3>'   : 'AVANTI2_ertmodel0827/AVANTI__UAV/Angle Conversion'
//  '<S4>'   : 'AVANTI2_ertmodel0827/AVANTI__UAV/Angle Conversion1'
//  '<S5>'   : 'AVANTI2_ertmodel0827/AVANTI__UAV/Angle Conversion2'
//  '<S6>'   : 'AVANTI2_ertmodel0827/AVANTI__UAV/Forces and Movement'
//  '<S7>'   : 'AVANTI2_ertmodel0827/AVANTI__UAV/6DOF (Euler Angles)/Calculate DCM & Euler Angles'
//  '<S8>'   : 'AVANTI2_ertmodel0827/AVANTI__UAV/6DOF (Euler Angles)/Calculate omega_dot'
//  '<S9>'   : 'AVANTI2_ertmodel0827/AVANTI__UAV/6DOF (Euler Angles)/Determine Force,  Mass & Inertia'
//  '<S10>'  : 'AVANTI2_ertmodel0827/AVANTI__UAV/6DOF (Euler Angles)/Vbxw'
//  '<S11>'  : 'AVANTI2_ertmodel0827/AVANTI__UAV/6DOF (Euler Angles)/Velocity Conversion'
//  '<S12>'  : 'AVANTI2_ertmodel0827/AVANTI__UAV/6DOF (Euler Angles)/Velocity Conversion1'
//  '<S13>'  : 'AVANTI2_ertmodel0827/AVANTI__UAV/6DOF (Euler Angles)/Velocity Conversion2'
//  '<S14>'  : 'AVANTI2_ertmodel0827/AVANTI__UAV/6DOF (Euler Angles)/transform to Inertial axes '
//  '<S15>'  : 'AVANTI2_ertmodel0827/AVANTI__UAV/6DOF (Euler Angles)/Calculate DCM & Euler Angles/Rotation Angles to Direction Cosine Matrix'
//  '<S16>'  : 'AVANTI2_ertmodel0827/AVANTI__UAV/6DOF (Euler Angles)/Calculate DCM & Euler Angles/phidot thetadot psidot'
//  '<S17>'  : 'AVANTI2_ertmodel0827/AVANTI__UAV/6DOF (Euler Angles)/Calculate DCM & Euler Angles/Rotation Angles to Direction Cosine Matrix/Create 3x3 Matrix'
//  '<S18>'  : 'AVANTI2_ertmodel0827/AVANTI__UAV/6DOF (Euler Angles)/Calculate omega_dot/3x3 Cross Product'
//  '<S19>'  : 'AVANTI2_ertmodel0827/AVANTI__UAV/6DOF (Euler Angles)/Calculate omega_dot/I x w'
//  '<S20>'  : 'AVANTI2_ertmodel0827/AVANTI__UAV/6DOF (Euler Angles)/Calculate omega_dot/I x w1'
//  '<S21>'  : 'AVANTI2_ertmodel0827/AVANTI__UAV/6DOF (Euler Angles)/Calculate omega_dot/3x3 Cross Product/Subsystem'
//  '<S22>'  : 'AVANTI2_ertmodel0827/AVANTI__UAV/6DOF (Euler Angles)/Calculate omega_dot/3x3 Cross Product/Subsystem1'
//  '<S23>'  : 'AVANTI2_ertmodel0827/AVANTI__UAV/6DOF (Euler Angles)/Vbxw/Subsystem'
//  '<S24>'  : 'AVANTI2_ertmodel0827/AVANTI__UAV/6DOF (Euler Angles)/Vbxw/Subsystem1'
//  '<S25>'  : 'AVANTI2_ertmodel0827/AVANTI__UAV/Forces and Movement/Aero_Dynamic_Model'
//  '<S26>'  : 'AVANTI2_ertmodel0827/AVANTI__UAV/Forces and Movement/Gravity_Forces_and_Movement'
//  '<S27>'  : 'AVANTI2_ertmodel0827/AVANTI__UAV/Forces and Movement/GroundEffect'
//  '<S28>'  : 'AVANTI2_ertmodel0827/AVANTI__UAV/Forces and Movement/ISA Atmosphere Model'
//  '<S29>'  : 'AVANTI2_ertmodel0827/AVANTI__UAV/Forces and Movement/Motor_Model'
//  '<S30>'  : 'AVANTI2_ertmodel0827/AVANTI__UAV/Forces and Movement/control_surfaces'
//  '<S31>'  : 'AVANTI2_ertmodel0827/AVANTI__UAV/Forces and Movement/Aero_Dynamic_Model/0.018u^2+0.003583u+0.00128'
//  '<S32>'  : 'AVANTI2_ertmodel0827/AVANTI__UAV/Forces and Movement/Aero_Dynamic_Model/Degrees to Radians'
//  '<S33>'  : 'AVANTI2_ertmodel0827/AVANTI__UAV/Forces and Movement/Aero_Dynamic_Model/Direction Cosine Matrix Body to Wind'
//  '<S34>'  : 'AVANTI2_ertmodel0827/AVANTI__UAV/Forces and Movement/Aero_Dynamic_Model/Direction Cosine Matrix Body to Wind/A11'
//  '<S35>'  : 'AVANTI2_ertmodel0827/AVANTI__UAV/Forces and Movement/Aero_Dynamic_Model/Direction Cosine Matrix Body to Wind/A12'
//  '<S36>'  : 'AVANTI2_ertmodel0827/AVANTI__UAV/Forces and Movement/Aero_Dynamic_Model/Direction Cosine Matrix Body to Wind/A13'
//  '<S37>'  : 'AVANTI2_ertmodel0827/AVANTI__UAV/Forces and Movement/Aero_Dynamic_Model/Direction Cosine Matrix Body to Wind/A21'
//  '<S38>'  : 'AVANTI2_ertmodel0827/AVANTI__UAV/Forces and Movement/Aero_Dynamic_Model/Direction Cosine Matrix Body to Wind/A22'
//  '<S39>'  : 'AVANTI2_ertmodel0827/AVANTI__UAV/Forces and Movement/Aero_Dynamic_Model/Direction Cosine Matrix Body to Wind/A23'
//  '<S40>'  : 'AVANTI2_ertmodel0827/AVANTI__UAV/Forces and Movement/Aero_Dynamic_Model/Direction Cosine Matrix Body to Wind/A31'
//  '<S41>'  : 'AVANTI2_ertmodel0827/AVANTI__UAV/Forces and Movement/Aero_Dynamic_Model/Direction Cosine Matrix Body to Wind/A32'
//  '<S42>'  : 'AVANTI2_ertmodel0827/AVANTI__UAV/Forces and Movement/Aero_Dynamic_Model/Direction Cosine Matrix Body to Wind/A33'
//  '<S43>'  : 'AVANTI2_ertmodel0827/AVANTI__UAV/Forces and Movement/Aero_Dynamic_Model/Direction Cosine Matrix Body to Wind/Create Transformation Matrix'
//  '<S44>'  : 'AVANTI2_ertmodel0827/AVANTI__UAV/Forces and Movement/control_surfaces/Angle Conversion'
//  '<S45>'  : 'AVANTI2_ertmodel0827/AVANTI__UAV/Forces and Movement/control_surfaces/alieron'
//  '<S46>'  : 'AVANTI2_ertmodel0827/AVANTI__UAV/Forces and Movement/control_surfaces/elevator'
//  '<S47>'  : 'AVANTI2_ertmodel0827/AVANTI__UAV/Forces and Movement/control_surfaces/rudder'

#endif                                 // RTW_HEADER_AVANTI2_ertmodel0827_h_

//
// File trailer for generated code.
//
// [EOF]
//
