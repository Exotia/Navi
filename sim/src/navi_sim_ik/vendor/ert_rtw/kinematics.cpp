//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: kinematics.cpp
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
#include "kinematics.h"
#include "rtwtypes.h"
#include <cmath>
#include <cstring>
#include <array>
#include "kinematics_capi.h"
#include <stddef.h>
#include "zero_crossing_types.h"
#define NumBitsPerChar                 8U

extern real_T rt_atan2d_snf(real_T u0, real_T u1);

//===========*
//  Constants *
// ===========
#define RT_PI                          3.14159265358979323846
#define RT_PIF                         3.1415927F
#define RT_LN_10                       2.30258509299404568402
#define RT_LN_10F                      2.3025851F
#define RT_LOG10E                      0.43429448190325182765
#define RT_LOG10EF                     0.43429449F
#define RT_E                           2.7182818284590452354
#define RT_EF                          2.7182817F

//
//  UNUSED_PARAMETER(x)
//    Used to specify that a function parameter (argument) is required but not
//    accessed by the function body.

#ifndef UNUSED_PARAMETER
#if defined(__LCC__)
#define UNUSED_PARAMETER(x)                                      // do nothing
#else

//
//  This is the semi-ANSI standard way of indicating that an
//  unused function parameter is required.

#define UNUSED_PARAMETER(x)            (void) (x)
#endif
#endif

extern "C" {
  real_T rtInf;
  real_T rtMinusInf;
  real_T rtNaN;
  real32_T rtInfF;
  real32_T rtMinusInfF;
  real32_T rtNaNF;
}
  extern "C"
{
  //
  // Initialize rtInf needed by the generated code.
  // Inf is initialized as non-signaling. Assumes IEEE.
  //
  static real_T rtGetInf(void)
  {
    size_t bitsPerReal{ sizeof(real_T) * (NumBitsPerChar) };

    real_T inf{ 0.0 };

    if (bitsPerReal == 32U) {
      inf = rtGetInfF();
    } else {
      union {
        LittleEndianIEEEDouble bitVal;
        real_T fltVal;
      } tmpVal;

      tmpVal.bitVal.words.wordH = 0x7FF00000U;
      tmpVal.bitVal.words.wordL = 0x00000000U;
      inf = tmpVal.fltVal;
    }

    return inf;
  }

  //
  // Initialize rtInfF needed by the generated code.
  // Inf is initialized as non-signaling. Assumes IEEE.
  //
  static real32_T rtGetInfF(void)
  {
    IEEESingle infF;
    infF.wordL.wordLuint = 0x7F800000U;
    return infF.wordL.wordLreal;
  }

  //
  // Initialize rtMinusInf needed by the generated code.
  // Inf is initialized as non-signaling. Assumes IEEE.
  //
  static real_T rtGetMinusInf(void)
  {
    size_t bitsPerReal{ sizeof(real_T) * (NumBitsPerChar) };

    real_T minf{ 0.0 };

    if (bitsPerReal == 32U) {
      minf = rtGetMinusInfF();
    } else {
      union {
        LittleEndianIEEEDouble bitVal;
        real_T fltVal;
      } tmpVal;

      tmpVal.bitVal.words.wordH = 0xFFF00000U;
      tmpVal.bitVal.words.wordL = 0x00000000U;
      minf = tmpVal.fltVal;
    }

    return minf;
  }

  //
  // Initialize rtMinusInfF needed by the generated code.
  // Inf is initialized as non-signaling. Assumes IEEE.
  //
  static real32_T rtGetMinusInfF(void)
  {
    IEEESingle minfF;
    minfF.wordL.wordLuint = 0xFF800000U;
    return minfF.wordL.wordLreal;
  }
}

extern "C" {
  //
  // Initialize rtNaN needed by the generated code.
  // NaN is initialized as non-signaling. Assumes IEEE.
  //
  static real_T rtGetNaN(void)
  {
    size_t bitsPerReal{ sizeof(real_T) * (NumBitsPerChar) };

    real_T nan{ 0.0 };

    if (bitsPerReal == 32U) {
      nan = rtGetNaNF();
    } else {
      union {
        LittleEndianIEEEDouble bitVal;
        real_T fltVal;
      } tmpVal;

      tmpVal.bitVal.words.wordH = 0xFFF80000U;
      tmpVal.bitVal.words.wordL = 0x00000000U;
      nan = tmpVal.fltVal;
    }

    return nan;
  }

  //
  // Initialize rtNaNF needed by the generated code.
  // NaN is initialized as non-signaling. Assumes IEEE.
  //
  static real32_T rtGetNaNF(void)
  {
    IEEESingle nanF{ { 0.0F } };

    nanF.wordL.wordLuint = 0xFFC00000U;
    return nanF.wordL.wordLreal;
  }
}
  extern "C"
{
  //
  // Initialize the rtInf, rtMinusInf, and rtNaN needed by the
  // generated code. NaN is initialized as non-signaling. Assumes IEEE.
  //
  static void rt_InitInfAndNaN(size_t realSize)
  {
    (void) (realSize);
    rtNaN = rtGetNaN();
    rtNaNF = rtGetNaNF();
    rtInf = rtGetInf();
    rtInfF = rtGetInfF();
    rtMinusInf = rtGetMinusInf();
    rtMinusInfF = rtGetMinusInfF();
  }

  // Test if value is infinite
  static boolean_T rtIsInf(real_T value)
  {
    return (boolean_T)((value==rtInf || value==rtMinusInf) ? 1U : 0U);
  }

  // Test if single-precision value is infinite
  static boolean_T rtIsInfF(real32_T value)
  {
    return (boolean_T)(((value)==rtInfF || (value)==rtMinusInfF) ? 1U : 0U);
  }

  // Test if value is not a number
  static boolean_T rtIsNaN(real_T value)
  {
    boolean_T result{ (boolean_T) 0 };

    size_t bitsPerReal{ sizeof(real_T) * (NumBitsPerChar) };

    if (bitsPerReal == 32U) {
      result = rtIsNaNF((real32_T)value);
    } else {
      union {
        LittleEndianIEEEDouble bitVal;
        real_T fltVal;
      } tmpVal;

      tmpVal.fltVal = value;
      result = (boolean_T)((tmpVal.bitVal.words.wordH & 0x7FF00000) ==
                           0x7FF00000 &&
                           ( (tmpVal.bitVal.words.wordH & 0x000FFFFF) != 0 ||
                            (tmpVal.bitVal.words.wordL != 0) ));
    }

    return result;
  }

  // Test if single-precision value is not a number
  static boolean_T rtIsNaNF(real32_T value)
  {
    IEEESingle tmp;
    tmp.wordL.wordLreal = value;
    return (boolean_T)( (tmp.wordL.wordLuint & 0x7F800000) == 0x7F800000 &&
                       (tmp.wordL.wordLuint & 0x007FFFFF) != 0 );
  }
}

// Function for MATLAB Function: '<S1>/Controller'
real_T kinematics::minimum(const real_T x[4])
{
  real_T ex;
  int32_T idx;
  if (!std::isnan(x[0])) {
    idx = 1;
  } else {
    int32_T k;
    boolean_T exitg1;
    idx = 0;
    k = 2;
    exitg1 = false;
    while ((!exitg1) && (k < 5)) {
      if (!std::isnan(x[k - 1])) {
        idx = k;
        exitg1 = true;
      } else {
        k++;
      }
    }
  }

  if (idx == 0) {
    ex = x[0];
  } else {
    ex = x[idx - 1];
    while (idx + 1 <= 4) {
      if (ex > x[idx]) {
        ex = x[idx];
      }

      idx++;
    }
  }

  return ex;
}

// Function for MATLAB Function: '<S1>/Feasable ICR Optimization'
real_T kinematics::mod(real_T x)
{
  real_T r;
  if (std::isnan(x)) {
    r = (rtNaN);
  } else if (std::isinf(x)) {
    r = (rtNaN);
  } else if (x == 0.0) {
    r = 0.0;
  } else {
    boolean_T rEQ0;
    r = std::fmod(x, 6.2831853071795862);
    rEQ0 = (r == 0.0);
    if (!rEQ0) {
      real_T q;
      q = std::abs(x / 6.2831853071795862);
      rEQ0 = !(std::abs(q - std::floor(q + 0.5)) > 2.2204460492503131E-16 * q);
    }

    if (rEQ0) {
      r = 0.0;
    } else if (x < 0.0) {
      r += 6.2831853071795862;
    }
  }

  return r;
}

real_T rt_atan2d_snf(real_T u0, real_T u1)
{
  real_T y;
  if (std::isnan(u0) || std::isnan(u1)) {
    y = (rtNaN);
  } else if (std::isinf(u0) && std::isinf(u1)) {
    int32_T u0_0;
    int32_T u1_0;
    if (u0 > 0.0) {
      u0_0 = 1;
    } else {
      u0_0 = -1;
    }

    if (u1 > 0.0) {
      u1_0 = 1;
    } else {
      u1_0 = -1;
    }

    y = std::atan2(static_cast<real_T>(u0_0), static_cast<real_T>(u1_0));
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(RT_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = std::atan2(u0, u1);
  }

  return y;
}

// Function for MATLAB Function: '<S1>/Feasable ICR Optimization'
boolean_T kinematics::isAngBetween(real_T theta, real_T lb, real_T ub)
{
  lb -= 0.0001;
  ub += 0.0001;
  return ((lb <= theta) && (theta <= ub)) || ((((theta >= 0.0) && (theta <= ub))
    || (lb <= theta)) && (lb > ub));
}

// Function for MATLAB Function: '<S1>/Feasable ICR Optimization'
real_T kinematics::eML_blk_kernel_anonFcn1(const real_T beta_min_ref[4], const
  real_T beta_max_ref[4], const real_T h[8], const real_T x[2])
{
  std::array<boolean_T, 2> b_x;
  std::array<real_T, 4> c;
  std::array<boolean_T, 4> c_x;
  real_T varargout_1;
  for (int32_T nz{0}; nz < 4; nz++) {
    real_T beta_i;
    real_T beta_i1;
    int32_T b_x_tmp;
    boolean_T exitg1;
    boolean_T y;
    b_x_tmp = nz << 1;
    beta_i = x[0] - h[b_x_tmp];
    b_x[0] = (beta_i == 0.0);
    beta_i1 = x[1] - h[b_x_tmp + 1];
    b_x[1] = (beta_i1 == 0.0);
    y = true;
    b_x_tmp = 0;
    exitg1 = false;
    while ((!exitg1) && (b_x_tmp < 2)) {
      if (!b_x[b_x_tmp]) {
        y = false;
        exitg1 = true;
      } else {
        b_x_tmp++;
      }
    }

    if (y) {
      c[nz] = 1.0;
    } else {
      real_T beta_i2;
      beta_i = rt_atan2d_snf(beta_i1, beta_i) - 1.5707963267948966;
      beta_i1 = mod(beta_i);
      if ((beta_i1 == 0.0) && (beta_i > 0.0)) {
        beta_i1 = 6.2831853071795862;
      }

      beta_i2 = mod(beta_i + 3.1415926535897931);
      if ((beta_i2 == 0.0) && (beta_i + 3.1415926535897931 > 0.0)) {
        beta_i2 = 6.2831853071795862;
      }

      if (isAngBetween(beta_i1, beta_min_ref[nz], beta_max_ref[nz])) {
        y = true;
      } else {
        y = isAngBetween(beta_i2, beta_min_ref[nz], beta_max_ref[nz]);
      }

      c[nz] = y;
    }

    c_x[nz] = (c[nz] == 0.0);
  }

  varargout_1 = ((c_x[0] + c_x[1]) + c_x[2]) + c_x[3];
  return varargout_1;
}

// Function for MATLAB Function: '<S1>/Feasable ICR Optimization'
real_T kinematics::norm(const real_T x[2])
{
  real_T absxk;
  real_T scale;
  real_T t;
  real_T y;
  scale = 3.3121686421112381E-170;
  absxk = std::abs(x[0]);
  if (absxk > 3.3121686421112381E-170) {
    y = 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    y = t * t;
  }

  absxk = std::abs(x[1]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  return scale * std::sqrt(y);
}

// Function for MATLAB Function: '<S1>/Feasable ICR Optimization'
void kinematics::mldivide(const real_T A[4], const real_T B_0[2], real_T Y_0[2])
{
  real_T Y_tmp;
  real_T a21;
  int32_T r1;
  int32_T r2;
  if (std::abs(A[1]) > std::abs(A[0])) {
    r1 = 1;
    r2 = 0;
  } else {
    r1 = 0;
    r2 = 1;
  }

  a21 = A[r2] / A[r1];
  Y_tmp = A[r1 + 2];
  Y_0[1] = (B_0[r2] - B_0[r1] * a21) / (A[r2 + 2] - Y_tmp * a21);
  Y_0[0] = (B_0[r1] - Y_tmp * Y_0[1]) / A[r1];
}

// Function for MATLAB Function: '<S1>/Feasable ICR Optimization'
void kinematics::schnittpunkte(const real_T n1[2], const real_T n2[2], const
  real_T m11[2], const real_T m12[2], const real_T m21[2], const real_T m22[2],
  real_T R_max, real_T intersections[12])
{
  std::array<real_T, 12> lambda;
  std::array<real_T, 2> lambda_tmp;
  std::array<real_T, 4> m11_0;
  std::memset(&lambda[0], 0, 12U * sizeof(real_T));
  lambda_tmp[0] = n1[0] - n2[0];
  m11_0[0] = -m11[0];
  m11_0[2] = m21[0];
  lambda_tmp[1] = n1[1] - n2[1];
  m11_0[1] = -m11[1];
  m11_0[3] = m21[1];
  mldivide(&m11_0[0], &lambda_tmp[0], &lambda[0]);
  m11_0[0] = -m12[0];
  m11_0[2] = m21[0];
  m11_0[1] = -m12[1];
  m11_0[3] = m21[1];
  mldivide(&m11_0[0], &lambda_tmp[0], &lambda[2]);
  m11_0[0] = -m11[0];
  m11_0[2] = m22[0];
  m11_0[1] = -m11[1];
  m11_0[3] = m22[1];
  mldivide(&m11_0[0], &lambda_tmp[0], &lambda[4]);
  m11_0[0] = -m12[0];
  m11_0[2] = m22[0];
  m11_0[1] = -m12[1];
  m11_0[3] = m22[1];
  mldivide(&m11_0[0], &lambda_tmp[0], &lambda[6]);
  for (int32_T i{0}; i < 6; i++) {
    real_T lambda_0;
    lambda_0 = lambda[i];
    if (lambda_0 == (rtInf)) {
      lambda_0 = R_max;
    }

    if (lambda_0 == (rtMinusInf)) {
      lambda_0 = -R_max;
    }

    lambda[i] = lambda_0;
  }

  intersections[0] = lambda[0] * m11[0] + n1[0];
  intersections[2] = m12[0] * lambda[2] + n1[0];
  intersections[4] = m11[0] * lambda[4] + n1[0];
  intersections[6] = m12[0] * lambda[6] + n1[0];
  intersections[8] = n1[0];
  intersections[10] = n2[0];
  intersections[1] = lambda[0] * m11[1] + n1[1];
  intersections[3] = m12[1] * lambda[2] + n1[1];
  intersections[5] = m11[1] * lambda[4] + n1[1];
  intersections[7] = m12[1] * lambda[6] + n1[1];
  intersections[9] = n1[1];
  intersections[11] = n2[1];
}

// Function for MATLAB Function: '<S1>/Feasable ICR Optimization'
real_T kinematics::minimum_e(const real_T x[36])
{
  real_T ex;
  int32_T idx;
  if (!std::isnan(x[0])) {
    idx = 1;
  } else {
    int32_T k;
    boolean_T exitg1;
    idx = 0;
    k = 2;
    exitg1 = false;
    while ((!exitg1) && (k < 37)) {
      if (!std::isnan(x[k - 1])) {
        idx = k;
        exitg1 = true;
      } else {
        k++;
      }
    }
  }

  if (idx == 0) {
    ex = x[0];
  } else {
    ex = x[idx - 1];
    while (idx + 1 <= 36) {
      if (ex > x[idx]) {
        ex = x[idx];
      }

      idx++;
    }
  }

  return ex;
}

// Function for MATLAB Function: '<S1>/SteerAngles2SteerSpeed'
void kinematics::angdiff(const real_T x[4], const real_T y[4], real_T delta[4])
{
  real_T delta_0;
  real_T lambda;
  delta_0 = y[0] - x[0];
  if ((delta_0 < -3.1415926535897931) || (delta_0 > 3.1415926535897931)) {
    lambda = mod(delta_0 + 3.1415926535897931);
    if ((lambda == 0.0) && (delta_0 + 3.1415926535897931 > 0.0)) {
      lambda = 6.2831853071795862;
    }

    delta_0 = lambda - 3.1415926535897931;
  }

  delta[0] = delta_0;
  delta_0 = y[1] - x[1];
  if ((delta_0 < -3.1415926535897931) || (delta_0 > 3.1415926535897931)) {
    lambda = mod(delta_0 + 3.1415926535897931);
    if ((lambda == 0.0) && (delta_0 + 3.1415926535897931 > 0.0)) {
      lambda = 6.2831853071795862;
    }

    delta_0 = lambda - 3.1415926535897931;
  }

  delta[1] = delta_0;
  delta_0 = y[2] - x[2];
  if ((delta_0 < -3.1415926535897931) || (delta_0 > 3.1415926535897931)) {
    lambda = mod(delta_0 + 3.1415926535897931);
    if ((lambda == 0.0) && (delta_0 + 3.1415926535897931 > 0.0)) {
      lambda = 6.2831853071795862;
    }

    delta_0 = lambda - 3.1415926535897931;
  }

  delta[2] = delta_0;
  delta_0 = y[3] - x[3];
  if ((delta_0 < -3.1415926535897931) || (delta_0 > 3.1415926535897931)) {
    lambda = mod(delta_0 + 3.1415926535897931);
    if ((lambda == 0.0) && (delta_0 + 3.1415926535897931 > 0.0)) {
      lambda = 6.2831853071795862;
    }

    delta_0 = lambda - 3.1415926535897931;
  }

  delta[3] = delta_0;
}

// Model step function
void kinematics::step()
{
  static const std::array<int8_T, 16> b_b{ { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0,
      0, 0, 0, 1 } };

  std::array<real_T, 12> G;
  std::array<real_T, 12> X_0;
  std::array<real_T, 4> X_tilde;
  std::array<real_T, 4> Y_tilde;
  std::array<int8_T, 9> b_I;
  std::array<int8_T, 16> b_I_0;
  std::array<real_T, 3> b_I_1;
  std::array<boolean_T, 2> b_x;
  std::array<real_T, 16> closestPoint;
  std::array<real_T, 2> closestPoint_0;
  std::array<real_T, 4> cost;
  std::array<real_T, 4> delta_beta;
  std::array<real_T, 72> intersections;
  std::array<real_T, 36> intersectionsCost;
  std::array<int8_T, 4> ipiv;
  std::array<real_T, 2> m11;
  std::array<real_T, 2> m12;
  std::array<real_T, 2> m21;
  std::array<real_T, 2> m22;
  std::array<real_T, 2> m31;
  std::array<real_T, 2> m32;
  std::array<real_T, 2> m41;
  std::array<real_T, 2> m42;
  std::array<real_T, 2> rtb_ICR_ref_k;
  std::array<real_T, 3> rtb_eta_dot_ref;
  std::array<real_T, 4> rtb_omega;
  std::array<real_T, 8> targetPoints;
  real_T Y_dot_max;
  real_T Y_tilde_tmp;
  real_T beta_max_ref;
  real_T m21_tmp;
  real_T m31_tmp;
  real_T mu;
  real_T rtb_DiscreteFIRFilter1;
  real_T rtb_ICR_curr_idx_0;
  real_T rtb_ICR_curr_idx_1;
  real_T rtb_Sum3_idx_0;
  real_T rtb_Sum3_idx_1;
  real_T rtb_Sum3_idx_2;
  real_T rtb_eta_dot_old_idx_0;
  real_T rtb_eta_dot_old_idx_1;
  real_T rtb_eta_dot_old_idx_2;
  real_T s;
  real_T smax;
  real_T t;
  real_T tmp;
  int32_T b_ix;
  int32_T c_ix;
  int32_T ix;
  int32_T jA;
  int32_T jj;
  int32_T jp;
  boolean_T exitg1;
  boolean_T guard1{ false };

  boolean_T rEQ0;
  boolean_T rtb_Logic_idx_0;
  boolean_T rtb_UnitDelay5;

  // Outputs for Atomic SubSystem: '<Root>/kinematics'
  // Outport: '<Root>/eta_dot_ref_init' incorporates:
  //   DiscreteIntegrator: '<S1>/Discrete-Time Integrator2'

  Y.eta_dot_ref_init[0] = DWork.DiscreteTimeIntegrator2_DSTATE[0];
  Y.eta_dot_ref_init[1] = DWork.DiscreteTimeIntegrator2_DSTATE[1];
  Y.eta_dot_ref_init[2] = DWork.DiscreteTimeIntegrator2_DSTATE[2];

  // MATLAB Function: '<S1>/Kinematic Constraint Matrix' incorporates:
  //   Constant: '<S1>/h'
  //   DiscreteIntegrator: '<S1>/Discrete-Time Integrator2'
  //   Inport: '<Root>/beta_hat'

  smax = std::sin(U.beta_hat[0]);
  s = std::cos(U.beta_hat[0]);
  Y_dot_max = std::sin(U.beta_hat[1]);
  mu = std::cos(U.beta_hat[1]);
  t = std::sin(U.beta_hat[2]);
  rtb_ICR_curr_idx_0 = std::cos(U.beta_hat[2]);
  rtb_ICR_curr_idx_1 = std::sin(U.beta_hat[3]);
  rtb_eta_dot_old_idx_0 = std::cos(U.beta_hat[3]);
  G[0] = -smax;
  G[4] = s;
  G[8] = 0.404 * s + -0.285 * smax;
  G[1] = -Y_dot_max;
  G[5] = mu;
  G[9] = 0.404 * mu + 0.285 * Y_dot_max;
  G[2] = -t;
  G[6] = rtb_ICR_curr_idx_0;
  G[10] = -0.404 * rtb_ICR_curr_idx_0 + 0.285 * t;
  G[3] = -rtb_ICR_curr_idx_1;
  G[7] = rtb_eta_dot_old_idx_0;
  G[11] = -0.404 * rtb_eta_dot_old_idx_0 + -0.285 * rtb_ICR_curr_idx_1;
  for (jj = 0; jj < 4; jj++) {
    X_0[3 * jj] = G[jj];
    X_0[3 * jj + 1] = G[jj + 4];
    X_0[3 * jj + 2] = G[jj + 8];
  }

  for (jj = 0; jj < 4; jj++) {
    for (jp = 0; jp < 4; jp++) {
      jA = (jp << 2) + jj;
      closestPoint[jA] = ((X_0[3 * jp + 1] * G[jj + 4] + X_0[3 * jp] * G[jj]) +
                          X_0[3 * jp + 2] * G[jj + 8]) + static_cast<real_T>
        (b_b[jA]) * 0.010000000000000002;
    }

    ipiv[jj] = static_cast<int8_T>(jj + 1);
  }

  for (jp = 0; jp < 3; jp++) {
    jj = jp * 5;
    jA = 0;
    ix = jj;
    smax = std::abs(closestPoint[jj]);
    for (b_ix = 2; b_ix <= 4 - jp; b_ix++) {
      ix++;
      s = std::abs(closestPoint[ix]);
      if (s > smax) {
        jA = b_ix - 1;
        smax = s;
      }
    }

    if (closestPoint[jj + jA] != 0.0) {
      if (jA != 0) {
        jA += jp;
        ipiv[jp] = static_cast<int8_T>(jA + 1);
        smax = closestPoint[jp];
        closestPoint[jp] = closestPoint[jA];
        closestPoint[jA] = smax;
        smax = closestPoint[jp + 4];
        closestPoint[jp + 4] = closestPoint[jA + 4];
        closestPoint[jA + 4] = smax;
        smax = closestPoint[jp + 8];
        closestPoint[jp + 8] = closestPoint[jA + 8];
        closestPoint[jA + 8] = smax;
        smax = closestPoint[jp + 12];
        closestPoint[jp + 12] = closestPoint[jA + 12];
        closestPoint[jA + 12] = smax;
      }

      ix = (jj - jp) + 4;
      for (jA = jj + 1; jA < ix; jA++) {
        closestPoint[jA] /= closestPoint[jj];
      }
    }

    jA = jj;
    ix = jj + 4;
    for (b_ix = 0; b_ix <= 2 - jp; b_ix++) {
      if (closestPoint[ix] != 0.0) {
        int32_T c;
        int32_T ijA;
        smax = -closestPoint[ix];
        c_ix = jj + 1;
        ijA = jA + 5;
        c = (jA - jp) + 8;
        while (ijA + 1 <= c) {
          closestPoint[ijA] += closestPoint[c_ix] * smax;
          c_ix++;
          ijA++;
        }
      }

      ix += 4;
      jA += 4;
    }
  }

  for (jj = 0; jj < 4; jj++) {
    jA = 3 * jj;
    ix = jj << 2;
    for (jp = 0; jp < jj; jp++) {
      b_ix = 3 * jp;
      smax = closestPoint[jp + ix];
      if (smax != 0.0) {
        X_0[jA] -= smax * X_0[b_ix];
        X_0[jA + 1] -= X_0[b_ix + 1] * smax;
        X_0[jA + 2] -= closestPoint[jp + ix] * X_0[b_ix + 2];
      }
    }

    smax = 1.0 / closestPoint[jj + ix];
    X_0[jA] *= smax;
    X_0[jA + 1] *= smax;
    X_0[jA + 2] *= smax;
  }

  for (jj = 3; jj >= 0; jj--) {
    ix = 3 * jj;
    b_ix = (jj << 2) - 1;
    for (jp = jj + 2; jp < 5; jp++) {
      c_ix = (jp - 1) * 3;
      if (closestPoint[jp + b_ix] != 0.0) {
        X_0[ix] -= closestPoint[jp + b_ix] * X_0[c_ix];
        rtb_ICR_curr_idx_0 = closestPoint[jp + b_ix];
        X_0[ix + 1] -= X_0[c_ix + 1] * rtb_ICR_curr_idx_0;
        X_0[ix + 2] -= X_0[c_ix + 2] * rtb_ICR_curr_idx_0;
      }
    }
  }

  for (jj = 2; jj >= 0; jj--) {
    int8_T ipiv_0;
    ipiv_0 = ipiv[jj];
    if (jj + 1 != ipiv_0) {
      smax = X_0[3 * jj];
      jp = (ipiv_0 - 1) * 3;
      X_0[3 * jj] = X_0[jp];
      X_0[jp] = smax;
      jA = 3 * jj + 1;
      smax = X_0[jA];
      X_0[jA] = X_0[jp + 1];
      X_0[jp + 1] = smax;
      jA = 3 * jj + 2;
      smax = X_0[jA];
      X_0[jA] = X_0[jp + 2];
      X_0[jp + 2] = smax;
    }
  }

  for (jj = 0; jj < 9; jj++) {
    b_I[jj] = 0;
  }

  b_I[0] = 1;
  b_I[4] = 1;
  b_I[8] = 1;
  for (jj = 0; jj < 3; jj++) {
    b_I_1[jj] = 0.0;
    for (jp = 0; jp < 3; jp++) {
      jA = jp << 2;
      b_I_1[jj] += (static_cast<real_T>(b_I[3 * jp + jj]) - (((G[jA + 1] *
        X_0[jj + 3] + G[jA] * X_0[jj]) + G[jA + 2] * X_0[jj + 6]) + G[jA + 3] *
        X_0[jj + 9])) * DWork.DiscreteTimeIntegrator2_DSTATE[jp];
    }

    rtb_eta_dot_ref[jj] = b_I_1[jj];
  }

  // End of MATLAB Function: '<S1>/Kinematic Constraint Matrix'
  for (jA = 0; jA < 4; jA++) {
    // MATLAB Function: '<S1>/Eta_dot2WheelVelocity' incorporates:
    //   Constant: '<S1>/h'
    //   Inport: '<Root>/beta_hat'

    smax = std::cos(U.beta_hat[jA]);
    Y_dot_max = std::sin(U.beta_hat[jA]);
    jj = jA << 1;
    rtb_omega[jA] = (-ConstP.h_Value[jj + 1] * smax + ConstP.h_Value[jj] *
                     Y_dot_max) * 3.7037037037037033 * rtb_eta_dot_ref[2] +
      (3.7037037037037033 * smax * rtb_eta_dot_ref[0] + 3.7037037037037033 *
       Y_dot_max * rtb_eta_dot_ref[1]);

    // MATLAB Function: '<S1>/Current ICR' incorporates:
    //   Inport: '<Root>/beta_hat'

    t = U.beta_hat[jA];
    if ((t < -3.1415926535897931) || (t > 3.1415926535897931)) {
      if (std::isinf(t + 3.1415926535897931)) {
        s = (rtNaN);
      } else if (t + 3.1415926535897931 == 0.0) {
        s = 0.0;
      } else {
        s = std::fmod(t + 3.1415926535897931, 6.2831853071795862);
        rEQ0 = (s == 0.0);
        if (!rEQ0) {
          smax = std::abs((t + 3.1415926535897931) / 6.2831853071795862);
          rEQ0 = !(std::abs(smax - std::floor(smax + 0.5)) >
                   2.2204460492503131E-16 * smax);
        }

        if (rEQ0) {
          s = 0.0;
        } else if (t + 3.1415926535897931 < 0.0) {
          s += 6.2831853071795862;
        }
      }

      if ((s == 0.0) && (t + 3.1415926535897931 > 0.0)) {
        s = 6.2831853071795862;
      }

      t = s - 3.1415926535897931;
    }

    cost[jA] = t;
  }

  // MATLAB Function: '<S1>/Current ICR' incorporates:
  //   Constant: '<S1>/Rmax'
  //   Constant: '<S1>/beta_thr'
  //   Constant: '<S1>/h'

  std::memset(&closestPoint[0], 0, sizeof(real_T) << 4U);
  for (jA = 0; jA < 4; jA++) {
    if (jA + 1 != 1) {
      smax = cost[jA] - cost[0];
      if (smax <= 1.5707963267948966) {
        closestPoint[jA] = std::abs(smax);
      } else {
        closestPoint[jA] = std::abs(smax) - 3.1415926535897931;
      }
    }

    if (jA + 1 != 2) {
      smax = cost[jA] - cost[1];
      if (smax <= 1.5707963267948966) {
        closestPoint[jA + 4] = std::abs(cost[jA] - cost[1]);
      } else {
        closestPoint[jA + 4] = std::abs(smax) - 3.1415926535897931;
      }
    }

    if (jA + 1 != 3) {
      smax = cost[jA] - cost[2];
      if (smax <= 1.5707963267948966) {
        closestPoint[jA + 8] = std::abs(cost[jA] - cost[2]);
      } else {
        closestPoint[jA + 8] = std::abs(smax) - 3.1415926535897931;
      }
    }

    if (jA + 1 != 4) {
      smax = cost[jA] - cost[3];
      if (smax <= 1.5707963267948966) {
        closestPoint[jA + 12] = std::abs(cost[jA] - cost[3]);
      } else {
        closestPoint[jA + 12] = std::abs(smax) - 3.1415926535897931;
      }
    }
  }

  for (jj = 0; jj < 16; jj++) {
    b_I_0[jj] = 0;
  }

  b_I_0[0] = 1;
  b_I_0[5] = 1;
  b_I_0[10] = 1;
  b_I_0[15] = 1;
  for (jj = 0; jj < 16; jj++) {
    closestPoint[jj] -= static_cast<real_T>(b_I_0[jj]);
  }

  if (!std::isnan(closestPoint[0])) {
    jj = 1;
  } else {
    jj = 0;
    jp = 2;
    exitg1 = false;
    while ((!exitg1) && (jp < 17)) {
      if (!std::isnan(closestPoint[jp - 1])) {
        jj = jp;
        exitg1 = true;
      } else {
        jp++;
      }
    }
  }

  if (jj == 0) {
    jA = 0;
  } else {
    s = closestPoint[jj - 1];
    jA = jj - 1;
    while (jj + 1 <= 16) {
      if (s < closestPoint[jj]) {
        s = closestPoint[jj];
        jA = jj;
      }

      jj++;
    }
  }

  jp = jA / 4;
  jj = jA - (jp << 2);
  X_tilde[0] = std::abs(cost[0]);
  X_tilde[1] = std::abs(cost[1]);
  X_tilde[2] = std::abs(cost[2]);
  X_tilde[3] = std::abs(cost[3]);
  if (!std::isnan(X_tilde[0])) {
    ix = 1;
  } else {
    ix = 0;
    jA = 2;
    exitg1 = false;
    while ((!exitg1) && (jA < 5)) {
      if (!std::isnan(X_tilde[jA - 1])) {
        ix = jA;
        exitg1 = true;
      } else {
        jA++;
      }
    }
  }

  if (ix == 0) {
    smax = X_tilde[0];
  } else {
    smax = X_tilde[ix - 1];
    while (ix + 1 <= 4) {
      if (smax < X_tilde[ix]) {
        smax = X_tilde[ix];
      }

      ix++;
    }
  }

  if (smax < 0.0031415) {
    s = 50.0 * std::cos(cost[jj]);
    smax = 50.0 * std::sin(cost[jj]);
  } else {
    smax = std::tan(cost[jj]);
    s = std::tan(cost[jp]);
    jj <<= 1;
    jp <<= 1;
    rtb_ICR_curr_idx_0 = ConstP.h_Value[jj + 1];
    rtb_ICR_curr_idx_1 = ConstP.h_Value[jj];
    s = ((rtb_ICR_curr_idx_0 * smax + (rtb_ICR_curr_idx_1 - ConstP.h_Value[jp]))
         - ConstP.h_Value[jp + 1] * s) / (smax - s);
    smax = rtb_ICR_curr_idx_1 - (s - rtb_ICR_curr_idx_0) * smax;
  }

  rtb_ICR_curr_idx_0 = smax;
  rtb_ICR_curr_idx_1 = s;

  // UnitDelay: '<S1>/Unit Delay'
  rtb_eta_dot_old_idx_0 = DWork.UnitDelay_DSTATE[0];
  rtb_eta_dot_old_idx_1 = DWork.UnitDelay_DSTATE[1];
  rtb_eta_dot_old_idx_2 = DWork.UnitDelay_DSTATE[2];

  // SignalConversion generated from: '<S11>/ SFunction ' incorporates:
  //   Inport: '<Root>/U'
  //   Inport: '<Root>/X_dot'
  //   Inport: '<Root>/Y_dot'
  //   MATLAB Function: '<S1>/Retain Translation'

  rtb_Sum3_idx_0 = U.VX_out;
  rtb_Sum3_idx_1 = U.VY_out;
  rtb_Sum3_idx_2 = U.U_p;

  // MATLAB Function: '<S1>/Retain Translation' incorporates:
  //   Inport: '<Root>/U'
  //   Inport: '<Root>/X_dot'
  //   Inport: '<Root>/Y_dot'
  //   UnitDelay: '<S1>/Unit Delay'

  if (U.VX_out == 0.0) {
    if (U.VY_out == 0.0) {
      if (U.U_p == 0.0) {
        rtb_Sum3_idx_0 = DWork.UnitDelay_DSTATE[0];
        rtb_Sum3_idx_1 = DWork.UnitDelay_DSTATE[1];
        rtb_Sum3_idx_2 = DWork.UnitDelay_DSTATE[2];
      } else {
        rtb_eta_dot_old_idx_0 = U.VX_out;
        rtb_eta_dot_old_idx_1 = U.VY_out;
        rtb_eta_dot_old_idx_2 = U.U_p;
      }
    } else {
      rtb_eta_dot_old_idx_0 = U.VX_out;
      rtb_eta_dot_old_idx_1 = U.VY_out;
      rtb_eta_dot_old_idx_2 = U.U_p;
    }
  } else {
    rtb_eta_dot_old_idx_0 = U.VX_out;
    rtb_eta_dot_old_idx_1 = U.VY_out;
    rtb_eta_dot_old_idx_2 = U.U_p;
  }

  // MATLAB Function: '<S1>/ICR Position Controller' incorporates:
  //   Constant: '<S1>/Rmax'
  //   Constant: '<S1>/delta'

  if (rtb_Sum3_idx_2 >= 0.0) {
    jj = 1;
  } else {
    jj = -1;
  }

  rtb_Sum3_idx_1 = std::tanh(-rtb_Sum3_idx_1 / (static_cast<real_T>(jj) * 0.005
    + rtb_Sum3_idx_2) / 50.0) * 50.0;
  if (rtb_Sum3_idx_2 >= 0.0) {
    jj = 1;
  } else {
    jj = -1;
  }

  rtb_Sum3_idx_0 = std::tanh(rtb_Sum3_idx_0 / (static_cast<real_T>(jj) * 0.005 +
    rtb_Sum3_idx_2) / 50.0) * 50.0;

  // End of MATLAB Function: '<S1>/ICR Position Controller'

  // MATLAB Function: '<S1>/Controller' incorporates:
  //   Constant: '<S1>/beta_dot_max'
  //   Constant: '<S1>/h'
  //   Inport: '<Root>/TS'
  //   MATLAB Function: '<S1>/Current ICR'
  //   MATLAB Function: '<S1>/Optimal Border Point Calculation'

  rtb_ICR_ref_k[0] = rtb_Sum3_idx_1;
  rtb_ICR_ref_k[1] = rtb_Sum3_idx_0;
  rtb_Sum3_idx_2 = std::abs(smax - 0.404);
  Y_tilde_tmp = std::abs(s - -0.285);
  mu = rtb_Sum3_idx_2 * rtb_Sum3_idx_2;
  cost[0] = (mu / Y_tilde_tmp + Y_tilde_tmp) * 2.0;
  t = Y_tilde_tmp * Y_tilde_tmp;
  X_tilde[0] = (t / rtb_Sum3_idx_2 + rtb_Sum3_idx_2) * 2.0;
  Y_dot_max = std::abs(s - 0.285);
  cost[1] = (mu / Y_dot_max + Y_dot_max) * 2.0;
  rtb_DiscreteFIRFilter1 = Y_dot_max * Y_dot_max;
  X_tilde[1] = (rtb_DiscreteFIRFilter1 / rtb_Sum3_idx_2 + rtb_Sum3_idx_2) * 2.0;
  rtb_Sum3_idx_2 = std::abs(smax - -0.404);
  mu = rtb_Sum3_idx_2 * rtb_Sum3_idx_2;
  cost[2] = (mu / Y_dot_max + Y_dot_max) * 2.0;
  X_tilde[2] = (rtb_DiscreteFIRFilter1 / rtb_Sum3_idx_2 + rtb_Sum3_idx_2) * 2.0;
  cost[3] = (mu / Y_tilde_tmp + Y_tilde_tmp) * 2.0;
  X_tilde[3] = (t / rtb_Sum3_idx_2 + rtb_Sum3_idx_2) * 2.0;
  smax = minimum(&cost[0]) * U.TS;
  Y_dot_max = minimum(&X_tilde[0]) * U.TS;
  rtb_Sum3_idx_2 = rtb_Sum3_idx_1 - rtb_ICR_curr_idx_0;
  Y_tilde_tmp = rtb_Sum3_idx_0 - s;
  b_x[0] = (rtb_Sum3_idx_1 < rtb_ICR_curr_idx_0 + smax);
  b_x[1] = (rtb_Sum3_idx_0 < s + Y_dot_max);
  rEQ0 = true;
  jA = 0;
  exitg1 = false;
  while ((!exitg1) && (jA < 2)) {
    if (!b_x[jA]) {
      rEQ0 = false;
      exitg1 = true;
    } else {
      jA++;
    }
  }

  guard1 = false;
  if (rEQ0) {
    b_x[0] = (rtb_Sum3_idx_1 > rtb_ICR_curr_idx_0 - smax);
    b_x[1] = (rtb_Sum3_idx_0 > s - Y_dot_max);
    jp = 0;
    exitg1 = false;
    while ((!exitg1) && (jp < 2)) {
      if (!b_x[jp]) {
        rEQ0 = false;
        exitg1 = true;
      } else {
        jp++;
      }
    }

    if (rEQ0) {
      rtb_ICR_ref_k[0] = rtb_Sum3_idx_1;
      rtb_ICR_ref_k[1] = rtb_Sum3_idx_0;
    } else {
      guard1 = true;
    }
  } else {
    guard1 = true;
  }

  if (guard1) {
    s = Y_dot_max / Y_tilde_tmp;
    if ((s >= 0.0) && (s <= 1.0)) {
      mu = (smax - rtb_Sum3_idx_2 / Y_tilde_tmp * Y_dot_max) / (2.0 * smax);
      if ((mu >= 0.0) && (mu <= 1.0)) {
        rtb_ICR_ref_k[0] = s * rtb_Sum3_idx_2 + rtb_ICR_curr_idx_0;
        rtb_ICR_ref_k[1] = s * Y_tilde_tmp + rtb_ICR_curr_idx_1;
      }
    }

    s = smax / rtb_Sum3_idx_2;
    if ((s >= 0.0) && (s <= 1.0)) {
      mu = (Y_dot_max - Y_tilde_tmp / rtb_Sum3_idx_2 * smax) / (2.0 * Y_dot_max);
      if ((mu >= 0.0) && (mu <= 1.0)) {
        rtb_ICR_ref_k[0] = s * rtb_Sum3_idx_2 + rtb_ICR_curr_idx_0;
        rtb_ICR_ref_k[1] = s * Y_tilde_tmp + rtb_ICR_curr_idx_1;
      }
    }

    s = -Y_dot_max / Y_tilde_tmp;
    if ((s >= 0.0) && (s <= 1.0)) {
      mu = (smax - rtb_Sum3_idx_2 / Y_tilde_tmp * Y_dot_max) / (2.0 * smax);
      if ((mu >= 0.0) && (mu <= 1.0)) {
        rtb_ICR_ref_k[0] = s * rtb_Sum3_idx_2 + rtb_ICR_curr_idx_0;
        rtb_ICR_ref_k[1] = s * Y_tilde_tmp + rtb_ICR_curr_idx_1;
      }
    }

    s = -smax / rtb_Sum3_idx_2;
    if ((s >= 0.0) && (s <= 1.0)) {
      mu = (Y_dot_max - Y_tilde_tmp / rtb_Sum3_idx_2 * smax) / (2.0 * Y_dot_max);
      if ((mu >= 0.0) && (mu <= 1.0)) {
        rtb_ICR_ref_k[0] = s * rtb_Sum3_idx_2 + rtb_ICR_curr_idx_0;
        rtb_ICR_ref_k[1] = s * Y_tilde_tmp + rtb_ICR_curr_idx_1;
      }
    }
  }

  // End of MATLAB Function: '<S1>/Controller'

  // MATLAB Function: '<S1>/Direct or Complementary Route Decision'
  rEQ0 = false;
  if ((!(rtb_Sum3_idx_1 * rtb_Sum3_idx_1 / 0.36 + rtb_Sum3_idx_0 *
         rtb_Sum3_idx_0 / 0.09 <= 1.0)) && (!(rtb_ICR_curr_idx_0 *
        rtb_ICR_curr_idx_0 / 0.36 + rtb_ICR_curr_idx_1 * rtb_ICR_curr_idx_1 /
        0.09 <= 1.0))) {
    smax = rtb_ICR_curr_idx_0 - rtb_Sum3_idx_1;
    s = (0.0 - rtb_Sum3_idx_1) * smax;
    Y_dot_max = smax * smax;
    mu = smax;
    smax = rtb_ICR_curr_idx_1 - rtb_Sum3_idx_0;
    s = ((0.0 - rtb_Sum3_idx_0) * smax + s) / (smax * smax + Y_dot_max);
    if (s < 0.0) {
      Y_dot_max = rtb_Sum3_idx_1;
      smax = rtb_Sum3_idx_0;
    } else if (s > 1.0) {
      Y_dot_max = rtb_Sum3_idx_1 + mu;
      smax += rtb_Sum3_idx_0;
    } else {
      Y_dot_max = s * mu + rtb_Sum3_idx_1;
      smax = s * smax + rtb_Sum3_idx_0;
    }

    rEQ0 = (Y_dot_max * Y_dot_max / 0.36 + smax * smax / 0.09 <= 1.0);
  }

  // End of MATLAB Function: '<S1>/Direct or Complementary Route Decision'

  // CombinatorialLogic: '<S13>/Logic' incorporates:
  //   DataTypeConversion: '<S1>/Data Type Conversion2'
  //   Memory: '<S13>/Memory'
  //   UnitDelay: '<S1>/Unit Delay3'

  rtb_Logic_idx_0 = ConstP.Logic_table[(((static_cast<uint32_T>(rEQ0) << 1) +
    (DWork.UnitDelay3_DSTATE != 0.0)) << 1) + DWork.Memory_PreviousInput];

  // MATLAB Function: '<S1>/Optimal Border Point Calculation' incorporates:
  //   Constant: '<S1>/Rmax'

  s = Y_tilde_tmp / rtb_Sum3_idx_2;
  targetPoints[0] = s * 0.0 + 50.0;
  targetPoints[1] = s * 50.0;
  s = -Y_tilde_tmp / rtb_Sum3_idx_2;
  targetPoints[2] = s * 0.0 + -50.0;
  targetPoints[3] = s * 50.0;
  s = rtb_Sum3_idx_2 / Y_tilde_tmp;
  targetPoints[4] = s * 50.0;
  targetPoints[5] = s * 0.0 + 50.0;
  s = -rtb_Sum3_idx_2 / Y_tilde_tmp;
  targetPoints[6] = s * 50.0;
  targetPoints[7] = s * 0.0 + -50.0;
  smax = (rtInf);
  jj = 0;
  for (jA = 0; jA < 4; jA++) {
    Y_dot_max = 3.3121686421112381E-170;
    jp = jA << 1;
    mu = std::abs(targetPoints[jp] - rtb_ICR_curr_idx_0);
    if (mu > 3.3121686421112381E-170) {
      s = 1.0;
      Y_dot_max = mu;
    } else {
      t = mu / 3.3121686421112381E-170;
      s = t * t;
    }

    mu = std::abs(targetPoints[jp + 1] - rtb_ICR_curr_idx_1);
    if (mu > Y_dot_max) {
      t = Y_dot_max / mu;
      s = s * t * t + 1.0;
      Y_dot_max = mu;
    } else {
      t = mu / Y_dot_max;
      s += t * t;
    }

    t = Y_dot_max * std::sqrt(s);
    if (t < smax) {
      smax = t;
      jj = jA;
    }
  }

  jj <<= 1;
  rtb_Sum3_idx_2 = targetPoints[jj];
  Y_tilde_tmp = targetPoints[jj + 1];

  // Switch: '<S1>/Switch1' incorporates:
  //   UnitDelay: '<S1>/Unit Delay4'
  //   UnitDelay: '<S1>/Unit Delay5'

  if (DWork.UnitDelay5_DSTATE) {
    // MATLAB Function: '<S1>/Optimal Border Point Calculation' incorporates:
    //   UnitDelay: '<S1>/Unit Delay4'

    rtb_Sum3_idx_2 = DWork.UnitDelay4_DSTATE[0];
    Y_tilde_tmp = DWork.UnitDelay4_DSTATE[1];
  }

  // End of Switch: '<S1>/Switch1'

  // MATLAB Function: '<S1>/Route Planning' incorporates:
  //   Switch: '<S1>/Switch1'

  rEQ0 = false;
  if (rtb_Logic_idx_0) {
    Y_dot_max = 3.3121686421112381E-170;
    mu = std::abs(rtb_ICR_curr_idx_0 - rtb_Sum3_idx_2);
    if (mu > 3.3121686421112381E-170) {
      s = 1.0;
      Y_dot_max = mu;
    } else {
      t = mu / 3.3121686421112381E-170;
      s = t * t;
    }

    mu = std::abs(rtb_ICR_curr_idx_1 - Y_tilde_tmp);
    if (mu > Y_dot_max) {
      t = Y_dot_max / mu;
      s = s * t * t + 1.0;
      Y_dot_max = mu;
    } else {
      t = mu / Y_dot_max;
      s += t * t;
    }

    s = Y_dot_max * std::sqrt(s);
    if (s < 5.0) {
      rtb_ICR_ref_k[0] = -rtb_Sum3_idx_2;
      rtb_ICR_ref_k[1] = -Y_tilde_tmp;
      rEQ0 = true;
    } else {
      rtb_ICR_ref_k[0] = rtb_Sum3_idx_2;
      rtb_ICR_ref_k[1] = Y_tilde_tmp;
    }
  }

  // End of MATLAB Function: '<S1>/Route Planning'

  // MATLAB Function: '<S1>/Feasable ICR Optimization' incorporates:
  //   Constant: '<S1>/beta_ddot_max'
  //   Constant: '<S1>/beta_dot_max'
  //   Constant: '<S1>/h'
  //   Inport: '<Root>/TS'
  //   Inport: '<Root>/beta_dot_hat'
  //   Inport: '<Root>/beta_hat'

  t = U.beta_dot_hat[0] - 200.0 * U.TS;
  if (t < -2.0) {
    t = -2.0;
  }

  smax = 200.0 * U.TS + U.beta_dot_hat[0];
  if (smax > 2.0) {
    smax = 2.0;
  }

  s = t * U.TS + U.beta_hat[0];
  rtb_UnitDelay5 = (s > 0.0);
  s = mod(s);
  if ((s == 0.0) && rtb_UnitDelay5) {
    s = 6.2831853071795862;
  }

  Y_tilde[0] = s;
  s = smax * U.TS + U.beta_hat[0];
  rtb_UnitDelay5 = (s > 0.0);
  s = mod(s);
  if ((s == 0.0) && rtb_UnitDelay5) {
    s = 6.2831853071795862;
  }

  X_tilde[0] = s;
  t = U.beta_dot_hat[1] - 200.0 * U.TS;
  if (t < -2.0) {
    t = -2.0;
  }

  smax = 200.0 * U.TS + U.beta_dot_hat[1];
  if (smax > 2.0) {
    smax = 2.0;
  }

  s = t * U.TS + U.beta_hat[1];
  rtb_UnitDelay5 = (s > 0.0);
  s = mod(s);
  if ((s == 0.0) && rtb_UnitDelay5) {
    s = 6.2831853071795862;
  }

  Y_tilde[1] = s;
  s = smax * U.TS + U.beta_hat[1];
  rtb_UnitDelay5 = (s > 0.0);
  s = mod(s);
  if ((s == 0.0) && rtb_UnitDelay5) {
    s = 6.2831853071795862;
  }

  X_tilde[1] = s;
  t = U.beta_dot_hat[2] - 200.0 * U.TS;
  if (t < -2.0) {
    t = -2.0;
  }

  smax = 200.0 * U.TS + U.beta_dot_hat[2];
  if (smax > 2.0) {
    smax = 2.0;
  }

  s = t * U.TS + U.beta_hat[2];
  rtb_UnitDelay5 = (s > 0.0);
  s = mod(s);
  if ((s == 0.0) && rtb_UnitDelay5) {
    s = 6.2831853071795862;
  }

  Y_tilde[2] = s;
  s = smax * U.TS + U.beta_hat[2];
  rtb_UnitDelay5 = (s > 0.0);
  s = mod(s);
  if ((s == 0.0) && rtb_UnitDelay5) {
    s = 6.2831853071795862;
  }

  X_tilde[2] = s;
  t = U.beta_dot_hat[3] - 200.0 * U.TS;
  if (t < -2.0) {
    t = -2.0;
  }

  smax = 200.0 * U.TS + U.beta_dot_hat[3];
  if (smax > 2.0) {
    smax = 2.0;
  }

  s = t * U.TS + U.beta_hat[3];
  rtb_UnitDelay5 = (s > 0.0);
  s = mod(s);
  if ((s == 0.0) && rtb_UnitDelay5) {
    s = 6.2831853071795862;
  }

  Y_tilde[3] = s;
  s = smax * U.TS + U.beta_hat[3];
  rtb_UnitDelay5 = (s > 0.0);
  s = mod(s);
  if ((s == 0.0) && rtb_UnitDelay5) {
    s = 6.2831853071795862;
  }

  X_tilde[3] = s;
  Y_dot_max = rtb_ICR_ref_k[0];
  mu = rtb_ICR_ref_k[1];
  if (eML_blk_kernel_anonFcn1(&Y_tilde[0], &X_tilde[0], &ConstP.h_Value[0],
       &rtb_ICR_ref_k[0]) > 0.0) {
    real_T m41_tmp;
    smax = std::sin(Y_tilde[0] + 1.5707963267948966);
    rtb_DiscreteFIRFilter1 = std::cos(Y_tilde[0] + 1.5707963267948966);
    m11[0] = rtb_DiscreteFIRFilter1;
    m11[1] = smax;
    m12[0] = std::cos(X_tilde[0] + 1.5707963267948966);
    m12[1] = std::sin(X_tilde[0] + 1.5707963267948966);
    t = std::sin(Y_tilde[1] + 1.5707963267948966);
    m21_tmp = std::cos(Y_tilde[1] + 1.5707963267948966);
    m21[0] = m21_tmp;
    m21[1] = t;
    m22[0] = std::cos(X_tilde[1] + 1.5707963267948966);
    m22[1] = std::sin(X_tilde[1] + 1.5707963267948966);
    beta_max_ref = std::sin(Y_tilde[2] + 1.5707963267948966);
    m31_tmp = std::cos(Y_tilde[2] + 1.5707963267948966);
    m31[0] = m31_tmp;
    m31[1] = beta_max_ref;
    m32[0] = std::cos(X_tilde[2] + 1.5707963267948966);
    m32[1] = std::sin(X_tilde[2] + 1.5707963267948966);
    tmp = std::sin(Y_tilde[3] + 1.5707963267948966);
    m41_tmp = std::cos(Y_tilde[3] + 1.5707963267948966);
    m41[0] = m41_tmp;
    m41[1] = tmp;
    m42[0] = std::cos(s + 1.5707963267948966);
    m42[1] = std::sin(s + 1.5707963267948966);
    s = ((rtb_ICR_ref_k[0] - 0.404) * rtb_DiscreteFIRFilter1 + (rtb_ICR_ref_k[1]
          - -0.285) * smax) / (rtb_DiscreteFIRFilter1 * rtb_DiscreteFIRFilter1 +
      smax * smax);
    closestPoint[0] = rtb_DiscreteFIRFilter1 * s + 0.404;
    closestPoint[1] = s * smax + -0.285;
    s = ((rtb_ICR_ref_k[0] - 0.404) * m12[0] + (rtb_ICR_ref_k[1] - -0.285) *
         m12[1]) / (m12[0] * m12[0] + m12[1] * m12[1]);
    closestPoint[2] = s * m12[0] + 0.404;
    closestPoint[3] = s * m12[1] + -0.285;
    s = ((rtb_ICR_ref_k[0] - 0.404) * m21_tmp + (rtb_ICR_ref_k[1] - 0.285) * t) /
      (m21_tmp * m21_tmp + t * t);
    closestPoint[4] = m21_tmp * s + 0.404;
    closestPoint[5] = s * t + 0.285;
    s = ((rtb_ICR_ref_k[0] - 0.404) * m22[0] + (rtb_ICR_ref_k[1] - 0.285) * m22
         [1]) / (m22[0] * m22[0] + m22[1] * m22[1]);
    closestPoint[6] = s * m22[0] + 0.404;
    closestPoint[7] = s * m22[1] + 0.285;
    s = ((rtb_ICR_ref_k[0] - -0.404) * m31_tmp + (rtb_ICR_ref_k[1] - 0.285) *
         beta_max_ref) / (m31_tmp * m31_tmp + beta_max_ref * beta_max_ref);
    closestPoint[8] = m31_tmp * s + -0.404;
    closestPoint[9] = s * beta_max_ref + 0.285;
    s = ((rtb_ICR_ref_k[0] - -0.404) * m32[0] + (rtb_ICR_ref_k[1] - 0.285) *
         m32[1]) / (m32[0] * m32[0] + m32[1] * m32[1]);
    closestPoint[10] = s * m32[0] + -0.404;
    closestPoint[11] = s * m32[1] + 0.285;
    s = ((rtb_ICR_ref_k[0] - -0.404) * m41_tmp + (rtb_ICR_ref_k[1] - -0.285) *
         tmp) / (m41_tmp * m41_tmp + tmp * tmp);
    closestPoint[12] = m41_tmp * s + -0.404;
    closestPoint[13] = s * tmp + -0.285;
    s = ((rtb_ICR_ref_k[0] - -0.404) * m42[0] + (rtb_ICR_ref_k[1] - -0.285) *
         m42[1]) / (m42[0] * m42[0] + m42[1] * m42[1]);
    closestPoint[14] = s * m42[0] + -0.404;
    closestPoint[15] = s * m42[1] + -0.285;
    for (c_ix = 0; c_ix < 8; c_ix++) {
      jA = c_ix << 1;
      closestPoint_0[0] = closestPoint[jA] - rtb_ICR_ref_k[0];
      closestPoint_0[1] = closestPoint[jA + 1] - rtb_ICR_ref_k[1];
      s = norm(&closestPoint_0[0]);
      targetPoints[c_ix] = eML_blk_kernel_anonFcn1(&Y_tilde[0], &X_tilde[0],
        &ConstP.h_Value[0], &closestPoint[jA]) * 250000.0 + s * s;
    }

    std::memset(&intersections[0], 0, 72U * sizeof(real_T));
    schnittpunkte(&ConstP.h_Value[0], &ConstP.h_Value[2], &m11[0], &m12[0],
                  &m21[0], &m22[0], 500.0, &intersections[0]);
    schnittpunkte(&ConstP.h_Value[0], &ConstP.h_Value[4], &m11[0], &m12[0],
                  &m31[0], &m32[0], 500.0, &intersections[12]);
    schnittpunkte(&ConstP.h_Value[0], &ConstP.h_Value[6], &m11[0], &m12[0],
                  &m41[0], &m42[0], 500.0, &intersections[24]);
    schnittpunkte(&ConstP.h_Value[2], &ConstP.h_Value[4], &m21[0], &m22[0],
                  &m31[0], &m32[0], 500.0, &intersections[36]);
    schnittpunkte(&ConstP.h_Value[2], &ConstP.h_Value[6], &m21[0], &m22[0],
                  &m41[0], &m42[0], 500.0, &intersections[48]);
    schnittpunkte(&ConstP.h_Value[4], &ConstP.h_Value[6], &m31[0], &m32[0],
                  &m41[0], &m42[0], 500.0, &intersections[60]);
    for (jp = 0; jp < 36; jp++) {
      m11[0] = intersections[jp << 1] - rtb_ICR_ref_k[0];
      m11[1] = intersections[(jp << 1) + 1] - rtb_ICR_ref_k[1];
      s = norm(&m11[0]);
      intersectionsCost[jp] = eML_blk_kernel_anonFcn1(&Y_tilde[0], &X_tilde[0],
        &ConstP.h_Value[0], &intersections[jp << 1]) * 250000.0 + s * s;
    }

    if (!std::isnan(targetPoints[0])) {
      jj = 1;
    } else {
      jj = 0;
      jp = 2;
      exitg1 = false;
      while ((!exitg1) && (jp < 9)) {
        if (!std::isnan(targetPoints[jp - 1])) {
          jj = jp;
          exitg1 = true;
        } else {
          jp++;
        }
      }
    }

    if (jj == 0) {
      s = targetPoints[0];
    } else {
      s = targetPoints[jj - 1];
      while (jj + 1 <= 8) {
        if (s > targetPoints[jj]) {
          s = targetPoints[jj];
        }

        jj++;
      }
    }

    smax = minimum_e(&intersectionsCost[0]);
    if (s < smax) {
      smax = (rtInf);
      jj = -1;
      for (jA = 0; jA < 8; jA++) {
        s = targetPoints[jA];
        if (s < smax) {
          jj = jA;
          smax = s;
        }
      }

      jj <<= 1;
      Y_dot_max = closestPoint[jj];
      mu = closestPoint[jj + 1];
    } else if (smax < (rtInf)) {
      smax = (rtInf);
      jj = -1;
      for (jA = 0; jA < 36; jA++) {
        s = intersectionsCost[jA];
        if (s < smax) {
          jj = jA;
          smax = s;
        }
      }

      jj <<= 1;
      Y_dot_max = intersections[jj];
      mu = intersections[jj + 1];
    }
  }

  // End of MATLAB Function: '<S1>/Feasable ICR Optimization'

  // MATLAB Function: '<S1>/ICR2SteerAngles' incorporates:
  //   Constant: '<S1>/h'

  s = rt_atan2d_snf(mu - -0.285, Y_dot_max - 0.404) - 1.5707963267948966;

  // MATLAB Function: '<S1>/SteerAngles2SteerSpeed' incorporates:
  //   Inport: '<Root>/beta_hat'

  smax = mod(s);
  if ((smax == 0.0) && (s > 0.0)) {
    smax = 6.2831853071795862;
  }

  beta_max_ref = mod(smax + 3.1415926535897931);
  if ((beta_max_ref == 0.0) && (smax + 3.1415926535897931 > 0.0)) {
    beta_max_ref = 6.2831853071795862;
  }

  t = mod(U.beta_hat[0]);
  if ((t == 0.0) && (U.beta_hat[0] > 0.0)) {
    t = 6.2831853071795862;
  }

  // End of Outputs for SubSystem: '<Root>/kinematics'

  // Outport: '<Root>/beta_next'
  Y.beta_next[0] = s;

  // Outputs for Atomic SubSystem: '<Root>/kinematics'
  // MATLAB Function: '<S1>/SteerAngles2SteerSpeed'
  Y_tilde[0] = smax;
  X_tilde[0] = beta_max_ref;
  cost[0] = t;

  // MATLAB Function: '<S1>/ICR2SteerAngles' incorporates:
  //   Constant: '<S1>/h'

  s = rt_atan2d_snf(mu - 0.285, Y_dot_max - 0.404) - 1.5707963267948966;

  // MATLAB Function: '<S1>/SteerAngles2SteerSpeed' incorporates:
  //   Inport: '<Root>/beta_hat'

  smax = mod(s);
  if ((smax == 0.0) && (s > 0.0)) {
    smax = 6.2831853071795862;
  }

  beta_max_ref = mod(smax + 3.1415926535897931);
  if ((beta_max_ref == 0.0) && (smax + 3.1415926535897931 > 0.0)) {
    beta_max_ref = 6.2831853071795862;
  }

  t = mod(U.beta_hat[1]);
  if ((t == 0.0) && (U.beta_hat[1] > 0.0)) {
    t = 6.2831853071795862;
  }

  // End of Outputs for SubSystem: '<Root>/kinematics'

  // Outport: '<Root>/beta_next'
  Y.beta_next[1] = s;

  // Outputs for Atomic SubSystem: '<Root>/kinematics'
  // MATLAB Function: '<S1>/SteerAngles2SteerSpeed'
  Y_tilde[1] = smax;
  X_tilde[1] = beta_max_ref;
  cost[1] = t;

  // MATLAB Function: '<S1>/ICR2SteerAngles' incorporates:
  //   Constant: '<S1>/h'

  s = rt_atan2d_snf(mu - 0.285, Y_dot_max - -0.404) - 1.5707963267948966;

  // MATLAB Function: '<S1>/SteerAngles2SteerSpeed' incorporates:
  //   Inport: '<Root>/beta_hat'

  smax = mod(s);
  if ((smax == 0.0) && (s > 0.0)) {
    smax = 6.2831853071795862;
  }

  beta_max_ref = mod(smax + 3.1415926535897931);
  if ((beta_max_ref == 0.0) && (smax + 3.1415926535897931 > 0.0)) {
    beta_max_ref = 6.2831853071795862;
  }

  t = mod(U.beta_hat[2]);
  if ((t == 0.0) && (U.beta_hat[2] > 0.0)) {
    t = 6.2831853071795862;
  }

  // End of Outputs for SubSystem: '<Root>/kinematics'

  // Outport: '<Root>/beta_next'
  Y.beta_next[2] = s;

  // Outputs for Atomic SubSystem: '<Root>/kinematics'
  // MATLAB Function: '<S1>/SteerAngles2SteerSpeed'
  Y_tilde[2] = smax;
  X_tilde[2] = beta_max_ref;
  cost[2] = t;

  // MATLAB Function: '<S1>/ICR2SteerAngles' incorporates:
  //   Constant: '<S1>/h'

  s = rt_atan2d_snf(mu - -0.285, Y_dot_max - -0.404) - 1.5707963267948966;

  // MATLAB Function: '<S1>/SteerAngles2SteerSpeed' incorporates:
  //   Inport: '<Root>/beta_hat'

  smax = mod(s);
  if ((smax == 0.0) && (s > 0.0)) {
    smax = 6.2831853071795862;
  }

  beta_max_ref = mod(smax + 3.1415926535897931);
  if ((beta_max_ref == 0.0) && (smax + 3.1415926535897931 > 0.0)) {
    beta_max_ref = 6.2831853071795862;
  }

  t = mod(U.beta_hat[3]);
  if ((t == 0.0) && (U.beta_hat[3] > 0.0)) {
    t = 6.2831853071795862;
  }

  Y_tilde[3] = smax;
  X_tilde[3] = beta_max_ref;
  cost[3] = t;
  angdiff(&cost[0], &Y_tilde[0], &delta_beta[0]);
  angdiff(&cost[0], &X_tilde[0], &Y_tilde[0]);

  // Outport: '<Root>/Beta_dot' incorporates:
  //   Inport: '<Root>/TS'
  //   MATLAB Function: '<S1>/SteerAngles2SteerSpeed'

  Y.Beta_dot[0] = delta_beta[0] / U.TS;

  // MATLAB Function: '<S1>/SteerAngles2SteerSpeed' incorporates:
  //   Inport: '<Root>/TS'
  //   Outport: '<Root>/Beta_dot'

  if (std::abs(Y_tilde[0]) < std::abs(delta_beta[0])) {
    Y.Beta_dot[0] = Y_tilde[0] / U.TS;
  }

  // Outport: '<Root>/Beta_dot' incorporates:
  //   Inport: '<Root>/TS'
  //   MATLAB Function: '<S1>/SteerAngles2SteerSpeed'

  Y.Beta_dot[1] = delta_beta[1] / U.TS;

  // MATLAB Function: '<S1>/SteerAngles2SteerSpeed' incorporates:
  //   Inport: '<Root>/TS'
  //   Outport: '<Root>/Beta_dot'

  if (std::abs(Y_tilde[1]) < std::abs(delta_beta[1])) {
    Y.Beta_dot[1] = Y_tilde[1] / U.TS;
  }

  // Outport: '<Root>/Beta_dot' incorporates:
  //   Inport: '<Root>/TS'
  //   MATLAB Function: '<S1>/SteerAngles2SteerSpeed'

  Y.Beta_dot[2] = delta_beta[2] / U.TS;

  // MATLAB Function: '<S1>/SteerAngles2SteerSpeed' incorporates:
  //   Inport: '<Root>/TS'
  //   Outport: '<Root>/Beta_dot'

  if (std::abs(Y_tilde[2]) < std::abs(delta_beta[2])) {
    Y.Beta_dot[2] = Y_tilde[2] / U.TS;
  }

  // Outport: '<Root>/Beta_dot' incorporates:
  //   Inport: '<Root>/TS'
  //   MATLAB Function: '<S1>/SteerAngles2SteerSpeed'

  Y.Beta_dot[3] = delta_beta[3] / U.TS;

  // MATLAB Function: '<S1>/SteerAngles2SteerSpeed' incorporates:
  //   Inport: '<Root>/TS'
  //   Outport: '<Root>/Beta_dot'

  if (std::abs(Y_tilde[3]) < std::abs(delta_beta[3])) {
    Y.Beta_dot[3] = Y_tilde[3] / U.TS;
  }

  // DiscreteFir: '<S1>/Discrete FIR Filter1' incorporates:
  //   DataTypeConversion: '<S1>/Data Type Conversion2'
  //   UnitDelay: '<S1>/Unit Delay3'

  if ((((DWork.UnitDelay3_DSTATE != 0.0) !=
        (PrevZCSigState.DiscreteFIRFilter1_Reset_ZCE == POS_ZCSIG)) &&
       (PrevZCSigState.DiscreteFIRFilter1_Reset_ZCE != UNINITIALIZED_ZCSIG)) ||
      (DWork.UnitDelay3_DSTATE != 0.0)) {
    DWork.DiscreteFIRFilter1_circBuf = 0;
    std::memset(&DWork.DiscreteFIRFilter1_states[0], 0, 23U * sizeof(real_T));
  }

  PrevZCSigState.DiscreteFIRFilter1_Reset_ZCE = (DWork.UnitDelay3_DSTATE != 0.0);
  rtb_DiscreteFIRFilter1 = 0.0;
  ix = 1;
  for (jj = DWork.DiscreteFIRFilter1_circBuf; jj < 23; jj++) {
    rtb_DiscreteFIRFilter1 += DWork.DiscreteFIRFilter1_states[jj] *
      ConstP.DiscreteFIRFilter1_Coefficients[ix];
    ix++;
  }

  for (jj = 0; jj < DWork.DiscreteFIRFilter1_circBuf; jj++) {
    rtb_DiscreteFIRFilter1 += DWork.DiscreteFIRFilter1_states[jj] *
      ConstP.DiscreteFIRFilter1_Coefficients[ix];
    ix++;
  }

  // End of DiscreteFir: '<S1>/Discrete FIR Filter1'

  // Update for DiscreteIntegrator: '<S1>/Discrete-Time Integrator2'
  smax = DWork.DiscreteTimeIntegrator2_DSTATE[0];

  // Sum: '<S1>/Sum3' incorporates:
  //   DiscreteIntegrator: '<S1>/Discrete-Time Integrator2'

  t = DWork.DiscreteTimeIntegrator2_DSTATE[0];

  // Update for DiscreteIntegrator: '<S1>/Discrete-Time Integrator2'
  beta_max_ref = DWork.DiscreteTimeIntegrator2_DSTATE[1];

  // Sum: '<S1>/Sum3' incorporates:
  //   DiscreteIntegrator: '<S1>/Discrete-Time Integrator2'

  tmp = DWork.DiscreteTimeIntegrator2_DSTATE[1];

  // Update for DiscreteIntegrator: '<S1>/Discrete-Time Integrator2'
  m21_tmp = DWork.DiscreteTimeIntegrator2_DSTATE[2];

  // Sum: '<S1>/Sum3' incorporates:
  //   DiscreteIntegrator: '<S1>/Discrete-Time Integrator2'

  m31_tmp = DWork.DiscreteTimeIntegrator2_DSTATE[2];

  // Update for DiscreteIntegrator: '<S1>/Discrete-Time Integrator2' incorporates:
  //   Gain: '<S1>/Gain3'
  //   Inport: '<Root>/U'
  //   Inport: '<Root>/X_dot'
  //   Inport: '<Root>/Y_dot'
  //   Sum: '<S1>/Sum3'

  DWork.DiscreteTimeIntegrator2_DSTATE[0] = (U.VX_out - t) * 2.0 * 0.06 + smax;
  DWork.DiscreteTimeIntegrator2_DSTATE[1] = (U.VY_out - tmp) * 2.0 * 0.06 +
    beta_max_ref;
  DWork.DiscreteTimeIntegrator2_DSTATE[2] = (U.U_p - m31_tmp) * 2.0 * 0.06 +
    m21_tmp;

  // Update for UnitDelay: '<S1>/Unit Delay'
  DWork.UnitDelay_DSTATE[0] = rtb_eta_dot_old_idx_0;
  DWork.UnitDelay_DSTATE[1] = rtb_eta_dot_old_idx_1;
  DWork.UnitDelay_DSTATE[2] = rtb_eta_dot_old_idx_2;

  // Update for UnitDelay: '<S1>/Unit Delay3' incorporates:
  //   Sum: '<S1>/Sum1'

  DWork.UnitDelay3_DSTATE = static_cast<real_T>(rEQ0) + rtb_DiscreteFIRFilter1;

  // Update for Memory: '<S13>/Memory'
  DWork.Memory_PreviousInput = rtb_Logic_idx_0;

  // Update for UnitDelay: '<S1>/Unit Delay5'
  DWork.UnitDelay5_DSTATE = rtb_Logic_idx_0;

  // Update for UnitDelay: '<S1>/Unit Delay4' incorporates:
  //   Switch: '<S1>/Switch1'

  DWork.UnitDelay4_DSTATE[0] = rtb_Sum3_idx_2;
  DWork.UnitDelay4_DSTATE[1] = Y_tilde_tmp;

  // Update for DiscreteFir: '<S1>/Discrete FIR Filter1' incorporates:
  //   DataTypeConversion: '<S1>/Data Type Conversion1'

  // Update circular buffer index
  DWork.DiscreteFIRFilter1_circBuf--;
  if (DWork.DiscreteFIRFilter1_circBuf < 0) {
    DWork.DiscreteFIRFilter1_circBuf = 22;
  }

  // Update circular buffer
  DWork.DiscreteFIRFilter1_states[DWork.DiscreteFIRFilter1_circBuf] =
    rtb_Logic_idx_0;

  // End of Update for DiscreteFir: '<S1>/Discrete FIR Filter1'
  // End of Outputs for SubSystem: '<Root>/kinematics'

  // Outport: '<Root>/omega'
  Y.omega[0] = rtb_omega[0];
  Y.omega[1] = rtb_omega[1];
  Y.omega[2] = rtb_omega[2];
  Y.omega[3] = rtb_omega[3];

  // Outport: '<Root>/beta_next'
  Y.beta_next[3] = s;

  // Outport: '<Root>/eta_dot_constrained'
  Y.eta_dot_constrained[0] = rtb_eta_dot_ref[0];
  Y.eta_dot_constrained[1] = rtb_eta_dot_ref[1];
  Y.eta_dot_constrained[2] = rtb_eta_dot_ref[2];

  // Outport: '<Root>/input_ICR'
  Y.input_ICR[0] = rtb_Sum3_idx_1;

  // Outport: '<Root>/controller_ICR'
  Y.controller_ICR[0] = rtb_ICR_ref_k[0];

  // Outport: '<Root>/feasable_ICR'
  Y.feasable_ICR[0] = Y_dot_max;

  // Outport: '<Root>/current_ICR'
  Y.current_ICR[0] = rtb_ICR_curr_idx_0;

  // Outport: '<Root>/border_ICR' incorporates:
  //   Switch: '<S1>/Switch1'

  Y.border_ICR[0] = rtb_Sum3_idx_2;

  // Outport: '<Root>/input_ICR'
  Y.input_ICR[1] = rtb_Sum3_idx_0;

  // Outport: '<Root>/controller_ICR'
  Y.controller_ICR[1] = rtb_ICR_ref_k[1];

  // Outport: '<Root>/feasable_ICR'
  Y.feasable_ICR[1] = mu;

  // Outport: '<Root>/current_ICR'
  Y.current_ICR[1] = rtb_ICR_curr_idx_1;

  // Outport: '<Root>/border_ICR'
  Y.border_ICR[1] = Y_tilde_tmp;

  // Outport: '<Root>/indirect_mode'
  Y.indirect_mode = rtb_Logic_idx_0;
}

// Model initialize function
void kinematics::initialize()
{
  // Registration code

  // initialize non-finites
  rt_InitInfAndNaN(sizeof(real_T));

  // Initialize DataMapInfo substructure containing ModelMap for C API
  kinematics_InitializeDataMapInfo((&M));
  PrevZCSigState.DiscreteFIRFilter1_Reset_ZCE = UNINITIALIZED_ZCSIG;
}

// Model terminate function
void kinematics::terminate()
{
  // (no terminate code required)
}

// Root inports set method
void kinematics::setExternalInputs(const kinematics::ExternalInputs
  *pExternalInputs)
{
  U = *pExternalInputs;
}

// Root outports get method
const kinematics::ExternalOutputs &kinematics::getExternalOutputs() const
{
  return Y;
}

// Block states get method
const kinematics::D_Work &kinematics::getDWork() const
{
  return DWork;
}

// Block states set method
void kinematics::setDWork(const kinematics::D_Work *pD_Work)
{
  DWork = std::move<const kinematics::D_Work &>(*pD_Work);
}

// Event data get method
const kinematics::PrevZCSigStates &kinematics::getZCEventData() const
{
  return PrevZCSigState;
}

// Event data set method
void kinematics::setZCEventData(const kinematics::PrevZCSigStates
  *pPrevZCSigStates)
{
  PrevZCSigState = *pPrevZCSigStates;
}

// Constructor
kinematics::kinematics() :
  DWork(),
  PrevZCSigState(),
  U(),
  Y(),
  M()
{
  // Currently there is no constructor body generated.
}

// Destructor
kinematics::~kinematics()
{
  // Currently there is no destructor body generated.
}

// Real-Time Model get method
kinematics::RT_MODEL * kinematics::getRTM()
{
  return (&M);
}

// Real-Time Model set method
void kinematics::setRTM(const RT_MODEL *pM)
{
  M = *pM;
}

//
// File trailer for generated code.
//
// [EOF]
//
