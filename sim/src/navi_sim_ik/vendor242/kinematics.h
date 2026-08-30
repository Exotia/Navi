/*
 * kinematics.h
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "kinematics".
 *
 * Model version              : 2.42
 * Simulink Coder version : 9.9 (R2023a) 19-Nov-2022
 * C++ source code generated on : Wed Sep  6 17:54:27 2023
 *
 * Target selection: grt.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: ARM Compatible->ARM 64-bit (LP64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef RTW_HEADER_kinematics_h_
#define RTW_HEADER_kinematics_h_
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "kinematics_types.h"

extern "C"
{

#include "rt_nonfinite.h"

}

extern "C"
{

#include "rtGetInf.h"

}

extern "C"
{

#include "rtGetNaN.h"

}

#include "zero_crossing_types.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

/* Block signals (default storage) */
struct B_kinematics_T {
  real_T DataTypeConversion1;          /* '<S1>/Data Type Conversion1' */
};

/* Block states (default storage) for system '<Root>' */
struct DW_kinematics_T {
  real_T DiscreteTimeIntegrator2_DSTATE[3];/* '<S1>/Discrete-Time Integrator2' */
  real_T UnitDelay_DSTATE[3];          /* '<S1>/Unit Delay' */
  real_T UnitDelay3_DSTATE;            /* '<S1>/Unit Delay3' */
  real_T UnitDelay4_DSTATE[2];         /* '<S1>/Unit Delay4' */
  real_T DiscreteFIRFilter1_states[23];/* '<S1>/Discrete FIR Filter1' */
  int32_T DiscreteFIRFilter1_circBuf;  /* '<S1>/Discrete FIR Filter1' */
  boolean_T UnitDelay5_DSTATE;         /* '<S1>/Unit Delay5' */
  boolean_T Memory_PreviousInput;      /* '<S13>/Memory' */
};

/* Zero-crossing (trigger) state */
struct PrevZCX_kinematics_T {
  ZCSigState DiscreteFIRFilter1_Reset_ZCE;/* '<S1>/Discrete FIR Filter1' */
};

/* Constant parameters (default storage) */
struct ConstP_kinematics_T {
  /* Expression: [0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1]
   * Referenced by: '<S1>/Discrete FIR Filter1'
   */
  real_T DiscreteFIRFilter1_Coefficients[24];

  /* Computed Parameter: Logic_table
   * Referenced by: '<S13>/Logic'
   */
  boolean_T Logic_table[16];
};

/* External inputs (root inport signals with default storage) */
struct ExtU_kinematics_T {
  real_T VX_out;                       /* '<Root>/X_dot' */
  real_T VY_out;                       /* '<Root>/Y_dot' */
  real_T U;                            /* '<Root>/U' */
  real_T beta_hat[4];                  /* '<Root>/beta_hat' */
  real_T TS;                           /* '<Root>/TS' */
  real_T hParams[8];                   /* '<Root>/hParams' */
  real_T beta_ddot_max;                /* '<Root>/beta_ddot_max' */
  real_T beta_dot_max;                 /* '<Root>/beta_dot_max' */
  real_T acceleration_factor;          /* '<Root>/acceleration_factor' */
  real_T beta_dot_hat[4];              /* '<Root>/beta_dot_hat' */
};

/* External outputs (root outports fed by signals with default storage) */
struct ExtY_kinematics_T {
  real_T Beta_dot[4];                  /* '<Root>/Beta_dot' */
  real_T input_ICR[2];                 /* '<Root>/input_ICR' */
  real_T controller_ICR[2];            /* '<Root>/controller_ICR' */
  real_T feasable_ICR[2];              /* '<Root>/feasable_ICR' */
  real_T omega[4];                     /* '<Root>/omega' */
  real_T current_ICR[2];               /* '<Root>/current_ICR' */
  real_T beta_next[4];                 /* '<Root>/beta_next' */
  boolean_T indirect_mode;             /* '<Root>/indirect_mode' */
  real_T border_ICR[2];                /* '<Root>/border_ICR' */
  real_T eta_dot_constrained[3];       /* '<Root>/eta_dot_constrained' */
  real_T eta_dot_ref_init[3];          /* '<Root>/eta_dot_ref_init' */
};

/* Real-time Model Data Structure */
struct tag_RTM_kinematics_T {
  const char_T *errorStatus;
};

/* Constant parameters (default storage) */
extern const ConstP_kinematics_T kinematics_ConstP;

/* Class declaration for model kinematics */
class kinematics final
{
  /* public data and function members */
 public:
  /* Copy Constructor */
  kinematics(kinematics const&) = delete;

  /* Assignment Operator */
  kinematics& operator= (kinematics const&) & = delete;

  /* Move Constructor */
  kinematics(kinematics &&) = delete;

  /* Move Assignment Operator */
  kinematics& operator= (kinematics &&) = delete;

  /* Real-Time Model get method */
  RT_MODEL_kinematics_T * getRTM();

  /* Root inports set method */
  void setExternalInputs(const ExtU_kinematics_T *pExtU_kinematics_T)
  {
    kinematics_U = *pExtU_kinematics_T;
  }

  /* Root outports get method */
  const ExtY_kinematics_T &getExternalOutputs() const
  {
    return kinematics_Y;
  }

  void ModelPrevZCStateInit();

  /* Initial conditions function */
  void initialize();

  /* model step function */
  void step();

  /* model terminate function */
  static void terminate();

  /* Constructor */
  kinematics();

  /* Destructor */
  ~kinematics();

  /* private data and function members */
 private:
  /* External inputs */
  ExtU_kinematics_T kinematics_U;

  /* External outputs */
  ExtY_kinematics_T kinematics_Y;

  /* Block signals */
  B_kinematics_T kinematics_B;

  /* Block states */
  DW_kinematics_T kinematics_DW;

  /* Triggered events */
  PrevZCX_kinematics_T kinematics_PrevZCX;

  /* private member function(s) for subsystem '<Root>'*/
  real_T kinematics_minimum(const real_T x[4]);
  real_T kinematics_mod(real_T x);
  boolean_T kinematics_isAngBetween(real_T theta, real_T lb, real_T ub);
  real_T kinemat_eML_blk_kernel_anonFcn1(const real_T beta_min_ref[4], const
    real_T beta_max_ref[4], const real_T h[8], const real_T x[2]);
  real_T kinematics_norm(const real_T x[2]);
  void kinematics_mldivide(const real_T A[4], const real_T B[2], real_T Y[2]);
  void kinematics_schnittpunkte(const real_T n1[2], const real_T n2[2], const
    real_T m11[2], const real_T m12[2], const real_T m21[2], const real_T m22[2],
    real_T R_max, real_T intersections[12]);
  real_T kinematics_minimum_k(const real_T x[36]);

  /* Real-Time Model */
  RT_MODEL_kinematics_T kinematics_M;
};

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S1>/Multiply' : Eliminated nontunable gain of 1
 */

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Note that this particular code originates from a subsystem build,
 * and has its own system numbers different from the parent model.
 * Refer to the system hierarchy for this subsystem below, and use the
 * MATLAB hilite_system command to trace the generated code back
 * to the parent model.  For example,
 *
 * hilite_system('MEROPE_Steering2/kinematics')    - opens subsystem MEROPE_Steering2/kinematics
 * hilite_system('MEROPE_Steering2/kinematics/Kp') - opens and selects block Kp
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'MEROPE_Steering2'
 * '<S1>'   : 'MEROPE_Steering2/kinematics'
 * '<S2>'   : 'MEROPE_Steering2/kinematics/Controller'
 * '<S3>'   : 'MEROPE_Steering2/kinematics/Current ICR'
 * '<S4>'   : 'MEROPE_Steering2/kinematics/Direct or Complementary Route Decision'
 * '<S5>'   : 'MEROPE_Steering2/kinematics/Eta_dot2WheelVelocity'
 * '<S6>'   : 'MEROPE_Steering2/kinematics/Feasable ICR Optimization'
 * '<S7>'   : 'MEROPE_Steering2/kinematics/ICR Position Controller'
 * '<S8>'   : 'MEROPE_Steering2/kinematics/ICR2SteerAngles'
 * '<S9>'   : 'MEROPE_Steering2/kinematics/Kinematic Constraint Matrix'
 * '<S10>'  : 'MEROPE_Steering2/kinematics/Optimal Border Point Calculation'
 * '<S11>'  : 'MEROPE_Steering2/kinematics/Retain Translation'
 * '<S12>'  : 'MEROPE_Steering2/kinematics/Route Planning'
 * '<S13>'  : 'MEROPE_Steering2/kinematics/S-R Flip-Flop'
 * '<S14>'  : 'MEROPE_Steering2/kinematics/SteerAngles2SteerSpeed'
 */
#endif                                 /* RTW_HEADER_kinematics_h_ */
