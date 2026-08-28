//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: kinematics.h
//
// Code generated for Simulink model 'kinematics'.
//
// Model version                  : 2.41
// Simulink Coder version         : 9.7 (R2022a) 13-Nov-2021
// C/C++ source code generated on : Wed Sep  7 16:24:21 2022
//
// Target selection: ert.tlc
// Embedded hardware selection: ARM Compatible->ARM 64-bit (LP64)
// Code generation objectives:
//    1. Execution efficiency
//    2. RAM efficiency
// Validation result: Not run
//
#ifndef RTW_HEADER_kinematics_h_
#define RTW_HEADER_kinematics_h_
#include "rtwtypes.h"
#include <utility>
#include <array>
#include "rtw_modelmap.h"
#include <stddef.h>
#include "zero_crossing_types.h"

// Model Code Variants

// Macros for accessing real-time model data structure
#ifndef rtmGetDataMapInfo
#define rtmGetDataMapInfo(rtm)         ((rtm)->DataMapInfo)
#endif

#ifndef rtmSetDataMapInfo
#define rtmSetDataMapInfo(rtm, val)    ((rtm)->DataMapInfo = (val))
#endif

#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

#ifndef SS_UINT64
#define SS_UINT64                      17
#endif

#ifndef SS_INT64
#define SS_INT64                       18
#endif

// Function to get C API Model Mapping Static Info
extern const rtwCAPI_ModelMappingStaticInfo*
  kinematics_GetCAPIStaticMap(void);
extern "C" {
  static real_T rtGetInf(void);
  static real32_T rtGetInfF(void);
  static real_T rtGetMinusInf(void);
  static real32_T rtGetMinusInfF(void);
}                                      // extern "C"
  extern "C"
{
  static real_T rtGetNaN(void);
  static real32_T rtGetNaNF(void);
}                                      // extern "C"

extern "C" {
  extern real_T rtInf;
  extern real_T rtMinusInf;
  extern real_T rtNaN;
  extern real32_T rtInfF;
  extern real32_T rtMinusInfF;
  extern real32_T rtNaNF;
  static void rt_InitInfAndNaN(size_t realSize);
  static boolean_T rtIsInf(real_T value);
  static boolean_T rtIsInfF(real32_T value);
  static boolean_T rtIsNaN(real_T value);
  static boolean_T rtIsNaNF(real32_T value);
  struct BigEndianIEEEDouble {
    struct {
      uint32_T wordH;
      uint32_T wordL;
    } words;
  };

  struct LittleEndianIEEEDouble {
    struct {
      uint32_T wordL;
      uint32_T wordH;
    } words;
  };

  struct IEEESingle {
    union {
      real32_T wordLreal;
      uint32_T wordLuint;
    } wordL;
  };
}                                      // extern "C"
  // Class declaration for model kinematics
  class kinematics final
{
  // public data and function members
 public:
  // Block signals and states (default storage) for system '<Root>'
  struct D_Work {
    std::array<real_T, 3> DiscreteTimeIntegrator2_DSTATE;// '<S1>/Discrete-Time Integrator2' 
    std::array<real_T, 3> UnitDelay_DSTATE;// '<S1>/Unit Delay'
    std::array<real_T, 2> UnitDelay4_DSTATE;// '<S1>/Unit Delay4'
    std::array<real_T, 23> DiscreteFIRFilter1_states;// '<S1>/Discrete FIR Filter1' 
    real_T UnitDelay3_DSTATE;          // '<S1>/Unit Delay3'
    int32_T DiscreteFIRFilter1_circBuf;// '<S1>/Discrete FIR Filter1'
    boolean_T UnitDelay5_DSTATE;       // '<S1>/Unit Delay5'
    boolean_T Memory_PreviousInput;    // '<S13>/Memory'
  };

  // Zero-crossing (trigger) state
  struct PrevZCSigStates {
    ZCSigState DiscreteFIRFilter1_Reset_ZCE;// '<S1>/Discrete FIR Filter1'
  };

  // Constant parameters (default storage)
  struct ConstParam {
    // Expression: h
    //  Referenced by: '<S1>/h'

    std::array<real_T, 8> h_Value;

    // Expression: [0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1]
    //  Referenced by: '<S1>/Discrete FIR Filter1'

    std::array<real_T, 24> DiscreteFIRFilter1_Coefficients;

    // Computed Parameter: Logic_table
    //  Referenced by: '<S13>/Logic'

    std::array<boolean_T, 16> Logic_table;
  };

  // External inputs (root inport signals with default storage)
  struct ExternalInputs {
    real_T VX_out;                     // '<Root>/X_dot'
    real_T VY_out;                     // '<Root>/Y_dot'
    real_T U_p;                        // '<Root>/U'
    std::array<real_T, 4> beta_hat;    // '<Root>/beta_hat'
    real_T TS;                         // '<Root>/TS'
    std::array<real_T, 4> beta_dot_hat;// '<Root>/beta_dot_hat'
  };

  // External outputs (root outports fed by signals with default storage)
  struct ExternalOutputs {
    std::array<real_T, 4> Beta_dot;    // '<Root>/Beta_dot'
    std::array<real_T, 2> input_ICR;   // '<Root>/input_ICR'
    std::array<real_T, 2> controller_ICR;// '<Root>/controller_ICR'
    std::array<real_T, 2> feasable_ICR;// '<Root>/feasable_ICR'
    std::array<real_T, 4> omega;       // '<Root>/omega'
    std::array<real_T, 2> current_ICR; // '<Root>/current_ICR'
    std::array<real_T, 4> beta_next;   // '<Root>/beta_next'
    boolean_T indirect_mode;           // '<Root>/indirect_mode'
    std::array<real_T, 2> border_ICR;  // '<Root>/border_ICR'
    std::array<real_T, 3> eta_dot_constrained;// '<Root>/eta_dot_constrained'
    std::array<real_T, 3> eta_dot_ref_init;// '<Root>/eta_dot_ref_init'
  };

  // Real-time Model Data Structure
  struct RT_MODEL {
    const char_T * volatile errorStatus;

    //
    //  DataMapInfo:
    //  The following substructure contains information regarding
    //  structures generated in the model's C API.

    struct {
      rtwCAPI_ModelMappingInfo mmi;
    } DataMapInfo;
  };

  // Copy Constructor
  kinematics(kinematics const&) = delete;

  // Assignment Operator
  kinematics& operator= (kinematics const&) & = delete;

  // Move Constructor
  kinematics(kinematics &&) = delete;

  // Move Assignment Operator
  kinematics& operator= (kinematics &&) = delete;

  // Real-Time Model get method
  kinematics::RT_MODEL * getRTM();

  // Real-Time Model set method
  void setRTM(const RT_MODEL *pM);

  // Block states
  D_Work DWork;

  // Triggered events
  PrevZCSigStates PrevZCSigState;

  // Root inports set method
  void setExternalInputs(const ExternalInputs *pExternalInputs);

  // Root outports get method
  const ExternalOutputs &getExternalOutputs() const;

  // Block states get method
  const D_Work &getDWork() const;

  // Block states set method
  void setDWork(const D_Work *pD_Work);

  // Event data get method
  const PrevZCSigStates &getZCEventData() const;

  // Event data set method
  void setZCEventData(const PrevZCSigStates *pPrevZCSigStates);

  // model initialize function
  void initialize();

  // model step function
  void step();

  // model terminate function
  static void terminate();

  // Constructor
  kinematics();

  // Destructor
  ~kinematics();

  // private data and function members
 private:
  // External inputs
  ExternalInputs U;

  // External outputs
  ExternalOutputs Y;

  // private member function(s) for subsystem '<Root>'
  real_T minimum(const real_T x[4]);
  real_T mod(real_T x);
  boolean_T isAngBetween(real_T theta, real_T lb, real_T ub);
  real_T eML_blk_kernel_anonFcn1(const real_T beta_min_ref[4], const real_T
    beta_max_ref[4], const real_T h[8], const real_T x[2]);
  real_T norm(const real_T x[2]);
  void mldivide(const real_T A[4], const real_T B_0[2], real_T Y_0[2]);
  void schnittpunkte(const real_T n1[2], const real_T n2[2], const real_T m11[2],
                     const real_T m12[2], const real_T m21[2], const real_T m22
                     [2], real_T R_max, real_T intersections[12]);
  real_T minimum_e(const real_T x[36]);
  void angdiff(const real_T x[4], const real_T y[4], real_T delta[4]);

  // Real-Time Model
  RT_MODEL M;
}

;

// Constant parameters (default storage)
extern const kinematics::ConstParam ConstP;

//-
//  These blocks were eliminated from the model due to optimizations:
//
//  Block '<S1>/Gain4' : Eliminated nontunable gain of 1


//-
//  The generated code includes comments that allow you to trace directly
//  back to the appropriate location in the model.  The basic format
//  is <system>/block_name, where system is the system number (uniquely
//  assigned by Simulink) and block_name is the name of the block.
//
//  Note that this particular code originates from a subsystem build,
//  and has its own system numbers different from the parent model.
//  Refer to the system hierarchy for this subsystem below, and use the
//  MATLAB hilite_system command to trace the generated code back
//  to the parent model.  For example,
//
//  hilite_system('MEROPE_Steering2/kinematics')    - opens subsystem MEROPE_Steering2/kinematics
//  hilite_system('MEROPE_Steering2/kinematics/Kp') - opens and selects block Kp
//
//  Here is the system hierarchy for this model
//
//  '<Root>' : 'MEROPE_Steering2'
//  '<S1>'   : 'MEROPE_Steering2/kinematics'
//  '<S2>'   : 'MEROPE_Steering2/kinematics/Controller'
//  '<S3>'   : 'MEROPE_Steering2/kinematics/Current ICR'
//  '<S4>'   : 'MEROPE_Steering2/kinematics/Direct or Complementary Route Decision'
//  '<S5>'   : 'MEROPE_Steering2/kinematics/Eta_dot2WheelVelocity'
//  '<S6>'   : 'MEROPE_Steering2/kinematics/Feasable ICR Optimization'
//  '<S7>'   : 'MEROPE_Steering2/kinematics/ICR Position Controller'
//  '<S8>'   : 'MEROPE_Steering2/kinematics/ICR2SteerAngles'
//  '<S9>'   : 'MEROPE_Steering2/kinematics/Kinematic Constraint Matrix'
//  '<S10>'  : 'MEROPE_Steering2/kinematics/Optimal Border Point Calculation'
//  '<S11>'  : 'MEROPE_Steering2/kinematics/Retain Translation'
//  '<S12>'  : 'MEROPE_Steering2/kinematics/Route Planning'
//  '<S13>'  : 'MEROPE_Steering2/kinematics/S-R Flip-Flop'
//  '<S14>'  : 'MEROPE_Steering2/kinematics/SteerAngles2SteerSpeed'

#endif                                 // RTW_HEADER_kinematics_h_

//
// File trailer for generated code.
//
// [EOF]
//
