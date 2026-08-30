/*
 * kinematics.cpp
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

#include "kinematics.h"
#include "rtwtypes.h"
#include <cmath>
#include <cstring>
#include "kinematics_private.h"
#include "rt_defines.h"
#include "zero_crossing_types.h"

#define WHEEL_RADIUS_REZIPROC 8

extern "C"
{

#include "rt_nonfinite.h"

}

/* Function for MATLAB Function: '<S1>/Controller' */
real_T kinematics::kinematics_minimum(const real_T x[4])
{
  real_T ex;
  int32_T idx;
  int32_T k;
  if (!std::isnan(x[0])) {
    idx = 1;
  } else {
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
    for (k = idx + 1; k < 5; k++) {
      real_T x_0;
      x_0 = x[k - 1];
      if (ex > x_0) {
        ex = x_0;
      }
    }
  }

  return ex;
}

/* Function for MATLAB Function: '<S1>/Feasable ICR Optimization' */
real_T kinematics::kinematics_mod(real_T x)
{
  real_T r;
  if (std::isnan(x) || std::isinf(x)) {
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
    int32_T tmp;
    int32_T tmp_0;
    if (u0 > 0.0) {
      tmp = 1;
    } else {
      tmp = -1;
    }

    if (u1 > 0.0) {
      tmp_0 = 1;
    } else {
      tmp_0 = -1;
    }

    y = std::atan2(static_cast<real_T>(tmp), static_cast<real_T>(tmp_0));
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

/* Function for MATLAB Function: '<S1>/Feasable ICR Optimization' */
boolean_T kinematics::kinematics_isAngBetween(real_T theta, real_T lb, real_T ub)
{
  lb -= 0.0001;
  ub += 0.0001;
  return ((lb <= theta) && (theta <= ub)) || ((((theta >= 0.0) && (theta <= ub))
    || (lb <= theta)) && (lb > ub));
}

/* Function for MATLAB Function: '<S1>/Feasable ICR Optimization' */
real_T kinematics::kinemat_eML_blk_kernel_anonFcn1(const real_T beta_min_ref[4],
  const real_T beta_max_ref[4], const real_T h[8], const real_T x[2])
{
  real_T beta_i1;
  real_T beta_i2;
  real_T beta_i_tmp;
  real_T beta_max_ref_0;
  int32_T b_x_tmp;
  int32_T i;
  boolean_T c_x[4];
  boolean_T b_x[2];
  boolean_T exitg1;
  boolean_T y;
  for (i = 0; i < 4; i++) {
    b_x_tmp = i << 1;
    beta_i1 = x[0] - h[b_x_tmp];
    b_x[0] = (beta_i1 == 0.0);
    beta_i2 = x[1] - h[b_x_tmp + 1];
    b_x[1] = (beta_i2 == 0.0);
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
      b_x_tmp = 1;
    } else {
      beta_i_tmp = rt_atan2d_snf(beta_i2, beta_i1) - 1.5707963267948966;
      beta_i1 = kinematics_mod(beta_i_tmp);
      if ((beta_i1 == 0.0) && (beta_i_tmp > 0.0)) {
        beta_i1 = 6.2831853071795862;
      }

      beta_i2 = kinematics_mod(beta_i_tmp + 3.1415926535897931);
      if ((beta_i2 == 0.0) && (beta_i_tmp + 3.1415926535897931 > 0.0)) {
        beta_i2 = 6.2831853071795862;
      }

      beta_i_tmp = beta_min_ref[i];
      beta_max_ref_0 = beta_max_ref[i];
      if (kinematics_isAngBetween(beta_i1, beta_i_tmp, beta_max_ref_0) ||
          kinematics_isAngBetween(beta_i2, beta_i_tmp, beta_max_ref_0)) {
        y = true;
      } else {
        y = false;
      }

      b_x_tmp = y;
    }

    c_x[i] = (b_x_tmp == 0);
  }

  return ((c_x[0] + c_x[1]) + c_x[2]) + c_x[3];
}

/* Function for MATLAB Function: '<S1>/Feasable ICR Optimization' */
real_T kinematics::kinematics_norm(const real_T x[2])
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

/* Function for MATLAB Function: '<S1>/Feasable ICR Optimization' */
void kinematics::kinematics_mldivide(const real_T A[4], const real_T B[2],
  real_T Y[2])
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
  Y[1] = (B[r2] - B[r1] * a21) / (A[r2 + 2] - Y_tmp * a21);
  Y[0] = (B[r1] - Y_tmp * Y[1]) / A[r1];
}

/* Function for MATLAB Function: '<S1>/Feasable ICR Optimization' */
void kinematics::kinematics_schnittpunkte(const real_T n1[2], const real_T n2[2],
  const real_T m11[2], const real_T m12[2], const real_T m21[2], const real_T
  m22[2], real_T R_max, real_T intersections[12])
{
  real_T lambda[12];
  real_T m11_0[4];
  real_T lambda_tmp[2];
  real_T lambda_0;
  int32_T i;
  std::memset(&lambda[0], 0, 12U * sizeof(real_T));
  lambda_tmp[0] = n1[0] - n2[0];
  m11_0[0] = -m11[0];
  m11_0[2] = m21[0];
  lambda_tmp[1] = n1[1] - n2[1];
  m11_0[1] = -m11[1];
  m11_0[3] = m21[1];
  kinematics_mldivide(m11_0, lambda_tmp, &lambda[0]);
  m11_0[0] = -m12[0];
  m11_0[2] = m21[0];
  m11_0[1] = -m12[1];
  m11_0[3] = m21[1];
  kinematics_mldivide(m11_0, lambda_tmp, &lambda[2]);
  m11_0[0] = -m11[0];
  m11_0[2] = m22[0];
  m11_0[1] = -m11[1];
  m11_0[3] = m22[1];
  kinematics_mldivide(m11_0, lambda_tmp, &lambda[4]);
  m11_0[0] = -m12[0];
  m11_0[2] = m22[0];
  m11_0[1] = -m12[1];
  m11_0[3] = m22[1];
  kinematics_mldivide(m11_0, lambda_tmp, &lambda[6]);
  for (i = 0; i < 6; i++) {
    lambda_0 = lambda[i];
    if (lambda_0 == (rtInf)) {
      lambda_0 = R_max;
      lambda[i] = R_max;
    }

    if (lambda_0 == (rtMinusInf)) {
      lambda[i] = -R_max;
    }
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

/* Function for MATLAB Function: '<S1>/Feasable ICR Optimization' */
real_T kinematics::kinematics_minimum_k(const real_T x[36])
{
  real_T ex;
  int32_T idx;
  int32_T k;
  if (!std::isnan(x[0])) {
    idx = 1;
  } else {
    boolean_T exitg1;
    idx = 0;
    k = 2;
    exitg1 = false;
    while ((!exitg1) && (k <= 36)) {
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
    for (k = idx + 1; k < 37; k++) {
      real_T x_0;
      x_0 = x[k - 1];
      if (ex > x_0) {
        ex = x_0;
      }
    }
  }

  return ex;
}

/* Model step function */
void kinematics::step()
{
  static const int8_T b_b[16]{ 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 };

  real_T intersections[72];
  real_T intersectionsCost[36];
  real_T closestPoint[16];
  real_T G[12];
  real_T X[12];
  real_T targetPoints[8];
  real_T X_tilde[4];
  real_T cost[4];
  real_T closestPoint_0[2];
  real_T m11[2];
  real_T m12[2];
  real_T m21[2];
  real_T m22[2];
  real_T m31[2];
  real_T m32[2];
  real_T m41[2];
  real_T m42[2];
  real_T G_tmp;
  real_T ICR_Y_curr;
  real_T absxk;
  real_T c_lambda;
  real_T d_lambda;
  real_T m11_tmp;
  real_T m21_tmp;
  real_T q;
  real_T rtb_eta_dot_idx_2;
  real_T s;
  real_T smax;
  int32_T d;
  int32_T ijA;
  int32_T jA;
  int32_T jAcol;
  int32_T jj;
  int32_T kBcol;
  int32_T vk;
  int8_T b_I_0[16];
  int8_T b_I[9];
  int8_T ipiv[4];
  int8_T ipiv_0;
  boolean_T b_x[2];
  boolean_T exitg1;
  boolean_T guard1;
  boolean_T positiveInput;
  boolean_T rEQ0;

  /* Outputs for Atomic SubSystem: '<Root>/kinematics' */
  /* Outport: '<Root>/eta_dot_ref_init' incorporates:
   *  DiscreteIntegrator: '<S1>/Discrete-Time Integrator2'
   */
  kinematics_Y.eta_dot_ref_init[0] =
    kinematics_DW.DiscreteTimeIntegrator2_DSTATE[0];
  kinematics_Y.eta_dot_ref_init[1] =
    kinematics_DW.DiscreteTimeIntegrator2_DSTATE[1];
  kinematics_Y.eta_dot_ref_init[2] =
    kinematics_DW.DiscreteTimeIntegrator2_DSTATE[2];

  /* MATLAB Function: '<S1>/Kinematic Constraint Matrix' incorporates:
   *  DiscreteIntegrator: '<S1>/Discrete-Time Integrator2'
   *  Inport: '<Root>/beta_hat'
   *  Inport: '<Root>/hParams'
   */
  smax = std::sin(kinematics_U.beta_hat[0]);
  s = std::cos(kinematics_U.beta_hat[0]);
  rtb_eta_dot_idx_2 = std::sin(kinematics_U.beta_hat[1]);
  ICR_Y_curr = std::cos(kinematics_U.beta_hat[1]);
  absxk = std::sin(kinematics_U.beta_hat[2]);
  q = std::cos(kinematics_U.beta_hat[2]);
  G_tmp = std::sin(kinematics_U.beta_hat[3]);
  d_lambda = std::cos(kinematics_U.beta_hat[3]);
  G[0] = -smax;
  G[4] = s;
  G[8] = kinematics_U.hParams[0] * s + kinematics_U.hParams[1] * smax;
  G[1] = -rtb_eta_dot_idx_2;
  G[5] = ICR_Y_curr;
  G[9] = kinematics_U.hParams[2] * ICR_Y_curr + kinematics_U.hParams[3] *
    rtb_eta_dot_idx_2;
  G[2] = -absxk;
  G[6] = q;
  G[10] = kinematics_U.hParams[4] * q + kinematics_U.hParams[5] * absxk;
  G[3] = -G_tmp;
  G[7] = d_lambda;
  G[11] = kinematics_U.hParams[6] * d_lambda + kinematics_U.hParams[7] * G_tmp;
  for (vk = 0; vk < 4; vk++) {
    X[3 * vk] = G[vk];
    jj = 3 * vk + 1;
    X[jj] = G[vk + 4];
    jAcol = 3 * vk + 2;
    X[jAcol] = G[vk + 8];
    smax = X[jj];
    s = X[3 * vk];
    rtb_eta_dot_idx_2 = X[jAcol];
    for (jj = 0; jj < 4; jj++) {
      jAcol = (vk << 2) + jj;
      closestPoint[jAcol] = ((G[jj + 4] * smax + s * G[jj]) + G[jj + 8] *
        rtb_eta_dot_idx_2) + static_cast<real_T>(b_b[jAcol]) *
        0.010000000000000002;
    }

    ipiv[vk] = static_cast<int8_T>(vk + 1);
  }

  for (vk = 0; vk < 3; vk++) {
    jj = vk * 5;
    jA = 4 - vk;
    kBcol = 0;
    smax = std::abs(closestPoint[jj]);
    for (jAcol = 2; jAcol <= jA; jAcol++) {
      s = std::abs(closestPoint[(jj + jAcol) - 1]);
      if (s > smax) {
        kBcol = jAcol - 1;
        smax = s;
      }
    }

    if (closestPoint[jj + kBcol] != 0.0) {
      if (kBcol != 0) {
        jAcol = vk + kBcol;
        ipiv[vk] = static_cast<int8_T>(jAcol + 1);
        smax = closestPoint[vk];
        closestPoint[vk] = closestPoint[jAcol];
        closestPoint[jAcol] = smax;
        smax = closestPoint[vk + 4];
        closestPoint[vk + 4] = closestPoint[jAcol + 4];
        closestPoint[jAcol + 4] = smax;
        smax = closestPoint[vk + 8];
        closestPoint[vk + 8] = closestPoint[jAcol + 8];
        closestPoint[jAcol + 8] = smax;
        smax = closestPoint[vk + 12];
        closestPoint[vk + 12] = closestPoint[jAcol + 12];
        closestPoint[jAcol + 12] = smax;
      }

      jA = (jj - vk) + 4;
      for (jAcol = jj + 2; jAcol <= jA; jAcol++) {
        closestPoint[jAcol - 1] /= closestPoint[jj];
      }
    }

    jAcol = 2 - vk;
    jA = jj + 6;
    for (kBcol = 0; kBcol <= jAcol; kBcol++) {
      d_lambda = closestPoint[((kBcol << 2) + jj) + 4];
      if (d_lambda != 0.0) {
        d = (jA - vk) + 2;
        for (ijA = jA; ijA <= d; ijA++) {
          closestPoint[ijA - 1] += closestPoint[((jj + ijA) - jA) + 1] *
            -d_lambda;
        }
      }

      jA += 4;
    }
  }

  for (vk = 0; vk < 4; vk++) {
    jj = 3 * vk;
    jAcol = vk << 2;
    for (jA = 0; jA < vk; jA++) {
      kBcol = 3 * jA;
      d_lambda = closestPoint[jA + jAcol];
      if (d_lambda != 0.0) {
        X[jj] -= d_lambda * X[kBcol];
        X[jj + 1] -= X[kBcol + 1] * d_lambda;
        X[jj + 2] -= X[kBcol + 2] * d_lambda;
      }
    }

    smax = 1.0 / closestPoint[vk + jAcol];
    X[jj] *= smax;
    X[jj + 1] *= smax;
    X[jj + 2] *= smax;
  }

  for (jj = 3; jj >= 0; jj--) {
    jAcol = 3 * jj;
    jA = (jj << 2) - 1;
    for (vk = jj + 2; vk < 5; vk++) {
      kBcol = (vk - 1) * 3;
      d_lambda = closestPoint[vk + jA];
      if (d_lambda != 0.0) {
        X[jAcol] -= d_lambda * X[kBcol];
        X[jAcol + 1] -= X[kBcol + 1] * d_lambda;
        X[jAcol + 2] -= X[kBcol + 2] * d_lambda;
      }
    }
  }

  for (vk = 2; vk >= 0; vk--) {
    ipiv_0 = ipiv[vk];
    if (vk + 1 != ipiv_0) {
      smax = X[3 * vk];
      jj = (ipiv_0 - 1) * 3;
      X[3 * vk] = X[jj];
      X[jj] = smax;
      jAcol = 3 * vk + 1;
      smax = X[jAcol];
      X[jAcol] = X[jj + 1];
      X[jj + 1] = smax;
      jAcol = 3 * vk + 2;
      smax = X[jAcol];
      X[jAcol] = X[jj + 2];
      X[jj + 2] = smax;
    }
  }

  for (vk = 0; vk < 9; vk++) {
    b_I[vk] = 0;
  }

  b_I[0] = 1;
  b_I[4] = 1;
  b_I[8] = 1;
  for (vk = 0; vk < 3; vk++) {
    ICR_Y_curr = 0.0;
    smax = X[vk + 3];
    s = X[vk];
    rtb_eta_dot_idx_2 = X[vk + 6];
    absxk = X[vk + 9];
    for (jj = 0; jj < 3; jj++) {
      jAcol = jj << 2;
      ICR_Y_curr += (static_cast<real_T>(b_I[3 * jj + vk]) - (((G[jAcol + 1] *
        smax + G[jAcol] * s) + G[jAcol + 2] * rtb_eta_dot_idx_2) + G[jAcol + 3] *
        absxk)) * kinematics_DW.DiscreteTimeIntegrator2_DSTATE[jj];
    }

    kinematics_Y.eta_dot_constrained[vk] = ICR_Y_curr;
  }

  /* End of MATLAB Function: '<S1>/Kinematic Constraint Matrix' */

  /* MATLAB Function: '<S1>/Eta_dot2WheelVelocity' incorporates:
   *  Inport: '<Root>/beta_hat'
   *  Inport: '<Root>/hParams'
   *  MATLAB Function: '<S1>/Current ICR'
   */
  s = kinematics_Y.eta_dot_constrained[2];
  ICR_Y_curr = kinematics_Y.eta_dot_constrained[0];
  absxk = kinematics_Y.eta_dot_constrained[1];
  for (vk = 0; vk < 4; vk++) {
    smax = std::cos(kinematics_U.beta_hat[vk]);
    rtb_eta_dot_idx_2 = std::sin(kinematics_U.beta_hat[vk]);
    jj = vk << 1;
    kinematics_Y.omega[vk] = (-kinematics_U.hParams[jj + 1] * smax +
      kinematics_U.hParams[jj] * rtb_eta_dot_idx_2) * WHEEL_RADIUS_REZIPROC * s +
      (WHEEL_RADIUS_REZIPROC * smax * ICR_Y_curr + WHEEL_RADIUS_REZIPROC *
       rtb_eta_dot_idx_2 * absxk);

    /* MATLAB Function: '<S1>/Current ICR' incorporates:
     *  Inport: '<Root>/beta_hat'
     *  Inport: '<Root>/hParams'
     */
    smax = kinematics_U.beta_hat[vk];
    cost[vk] = smax;

    /* MATLAB Function: '<S1>/Current ICR' */
    if ((smax < -3.1415926535897931) || (smax > 3.1415926535897931)) {
      if (std::isinf(smax + 3.1415926535897931)) {
        rtb_eta_dot_idx_2 = (rtNaN);
      } else if (smax + 3.1415926535897931 == 0.0) {
        rtb_eta_dot_idx_2 = 0.0;
      } else {
        rtb_eta_dot_idx_2 = std::fmod(smax + 3.1415926535897931,
          6.2831853071795862);
        rEQ0 = (rtb_eta_dot_idx_2 == 0.0);
        if (!rEQ0) {
          q = std::abs((smax + 3.1415926535897931) / 6.2831853071795862);
          rEQ0 = !(std::abs(q - std::floor(q + 0.5)) > 2.2204460492503131E-16 *
                   q);
        }

        if (rEQ0) {
          rtb_eta_dot_idx_2 = 0.0;
        } else if (smax + 3.1415926535897931 < 0.0) {
          rtb_eta_dot_idx_2 += 6.2831853071795862;
        }
      }

      if ((rtb_eta_dot_idx_2 == 0.0) && (smax + 3.1415926535897931 > 0.0)) {
        rtb_eta_dot_idx_2 = 6.2831853071795862;
      }

      cost[vk] = rtb_eta_dot_idx_2 - 3.1415926535897931;
    }
  }

  /* End of MATLAB Function: '<S1>/Eta_dot2WheelVelocity' */

  /* MATLAB Function: '<S1>/Current ICR' incorporates:
   *  Constant: '<S1>/Rmax'
   *  Constant: '<S1>/beta_thr'
   *  Inport: '<Root>/hParams'
   */
  std::memset(&closestPoint[0], 0, sizeof(real_T) << 4U);
  for (vk = 0; vk < 4; vk++) {
    if (vk + 1 != 1) {
      d_lambda = cost[vk] - cost[0];
      if (d_lambda <= 1.5707963267948966) {
        closestPoint[vk] = std::abs(d_lambda);
      } else {
        closestPoint[vk] = std::abs(d_lambda) - 3.1415926535897931;
      }
    }

    if (vk + 1 != 2) {
      d_lambda = cost[vk] - cost[1];
      if (d_lambda <= 1.5707963267948966) {
        closestPoint[vk + 4] = std::abs(d_lambda);
      } else {
        closestPoint[vk + 4] = std::abs(d_lambda) - 3.1415926535897931;
      }
    }

    if (vk + 1 != 3) {
      d_lambda = cost[vk] - cost[2];
      if (d_lambda <= 1.5707963267948966) {
        closestPoint[vk + 8] = std::abs(d_lambda);
      } else {
        closestPoint[vk + 8] = std::abs(d_lambda) - 3.1415926535897931;
      }
    }

    if (vk + 1 != 4) {
      d_lambda = cost[vk] - cost[3];
      if (d_lambda <= 1.5707963267948966) {
        closestPoint[vk + 12] = std::abs(d_lambda);
      } else {
        closestPoint[vk + 12] = std::abs(d_lambda) - 3.1415926535897931;
      }
    }
  }

  for (vk = 0; vk < 16; vk++) {
    b_I_0[vk] = 0;
  }

  b_I_0[0] = 1;
  b_I_0[5] = 1;
  b_I_0[10] = 1;
  b_I_0[15] = 1;
  for (vk = 0; vk < 16; vk++) {
    closestPoint[vk] -= static_cast<real_T>(b_I_0[vk]);
  }

  if (!std::isnan(closestPoint[0])) {
    jj = 0;
  } else {
    jj = -1;
    vk = 2;
    exitg1 = false;
    while ((!exitg1) && (vk < 17)) {
      if (!std::isnan(closestPoint[vk - 1])) {
        jj = vk - 1;
        exitg1 = true;
      } else {
        vk++;
      }
    }
  }

  if (jj + 1 == 0) {
    jA = 0;
  } else {
    smax = closestPoint[jj];
    jA = jj;
    for (jAcol = jj + 2; jAcol < 17; jAcol++) {
      s = closestPoint[jAcol - 1];
      if (smax < s) {
        smax = s;
        jA = jAcol - 1;
      }
    }
  }

  vk = jA / 4;
  jj = jA - (vk << 2);
  X_tilde[0] = std::abs(cost[0]);
  X_tilde[1] = std::abs(cost[1]);
  X_tilde[2] = std::abs(cost[2]);
  X_tilde[3] = std::abs(cost[3]);
  if (!std::isnan(X_tilde[0])) {
    jAcol = 1;
  } else {
    jAcol = 0;
    jA = 2;
    exitg1 = false;
    while ((!exitg1) && (jA < 5)) {
      if (!std::isnan(X_tilde[jA - 1])) {
        jAcol = jA;
        exitg1 = true;
      } else {
        jA++;
      }
    }
  }

  if (jAcol == 0) {
    smax = X_tilde[0];
  } else {
    smax = X_tilde[jAcol - 1];
    for (jA = jAcol + 1; jA < 5; jA++) {
      s = X_tilde[jA - 1];
      if (smax < s) {
        smax = s;
      }
    }
  }

  if (smax < 0.0031415) {
    ICR_Y_curr = 50.0 * std::cos(cost[jj]);
    kinematics_Y.current_ICR[0] = 50.0 * std::sin(cost[jj]);
  } else {
    smax = std::tan(cost[jj]);
    ICR_Y_curr = std::tan(cost[vk]);
    jj <<= 1;
    vk <<= 1;
    s = kinematics_U.hParams[jj];
    rtb_eta_dot_idx_2 = kinematics_U.hParams[jj + 1];
    ICR_Y_curr = ((rtb_eta_dot_idx_2 * smax + (s - kinematics_U.hParams[vk])) -
                  kinematics_U.hParams[vk + 1] * ICR_Y_curr) / (smax -
      ICR_Y_curr);
    kinematics_Y.current_ICR[0] = s - (ICR_Y_curr - rtb_eta_dot_idx_2) * smax;
  }

  kinematics_Y.current_ICR[1] = ICR_Y_curr;

  /* SignalConversion generated from: '<S11>/ SFunction ' incorporates:
   *  Inport: '<Root>/U'
   *  Inport: '<Root>/X_dot'
   *  Inport: '<Root>/Y_dot'
   *  MATLAB Function: '<S1>/Retain Translation'
   */
  smax = kinematics_U.VX_out;
  s = kinematics_U.VY_out;
  rtb_eta_dot_idx_2 = kinematics_U.U;

  /* MATLAB Function: '<S1>/Retain Translation' incorporates:
   *  Inport: '<Root>/U'
   *  Inport: '<Root>/X_dot'
   *  Inport: '<Root>/Y_dot'
   *  UnitDelay: '<S1>/Unit Delay'
   */
  if ((kinematics_U.VX_out == 0.0) && (kinematics_U.VY_out == 0.0)) {
    if (kinematics_U.U == 0.0) {
      smax = kinematics_DW.UnitDelay_DSTATE[0];
      s = kinematics_DW.UnitDelay_DSTATE[1];
      rtb_eta_dot_idx_2 = kinematics_DW.UnitDelay_DSTATE[2];
    } else {
      kinematics_DW.UnitDelay_DSTATE[0] = kinematics_U.VX_out;
      kinematics_DW.UnitDelay_DSTATE[1] = kinematics_U.VY_out;
      kinematics_DW.UnitDelay_DSTATE[2] = kinematics_U.U;
    }
  } else {
    kinematics_DW.UnitDelay_DSTATE[0] = kinematics_U.VX_out;
    kinematics_DW.UnitDelay_DSTATE[1] = kinematics_U.VY_out;
    kinematics_DW.UnitDelay_DSTATE[2] = kinematics_U.U;
  }

  /* MATLAB Function: '<S1>/ICR Position Controller' incorporates:
   *  Constant: '<S1>/Rmax'
   *  Constant: '<S1>/delta'
   */
  if (rtb_eta_dot_idx_2 >= 0.0) {
    vk = 1;
  } else {
    vk = -1;
  }

  kinematics_Y.input_ICR[0] = std::tanh(-s / (static_cast<real_T>(vk) * 0.005 +
    rtb_eta_dot_idx_2) / 50.0) * 50.0;
  if (rtb_eta_dot_idx_2 >= 0.0) {
    vk = 1;
  } else {
    vk = -1;
  }

  kinematics_Y.input_ICR[1] = std::tanh(smax / (static_cast<real_T>(vk) * 0.005
    + rtb_eta_dot_idx_2) / 50.0) * 50.0;

  /* End of MATLAB Function: '<S1>/ICR Position Controller' */

  /* MATLAB Function: '<S1>/Controller' incorporates:
   *  Inport: '<Root>/TS'
   *  Inport: '<Root>/beta_dot_max'
   *  Inport: '<Root>/hParams'
   *  MATLAB Function: '<S1>/Optimal Border Point Calculation'
   */
  kinematics_Y.controller_ICR[0] = kinematics_Y.input_ICR[0];
  kinematics_Y.controller_ICR[1] = kinematics_Y.input_ICR[1];
  s = std::abs(kinematics_Y.current_ICR[0] - kinematics_U.hParams[0]);
  ICR_Y_curr = std::abs(kinematics_Y.current_ICR[1] - kinematics_U.hParams[1]);
  cost[0] = (s * s / ICR_Y_curr + ICR_Y_curr) * kinematics_U.beta_dot_max;
  X_tilde[0] = (ICR_Y_curr * ICR_Y_curr / s + s) * kinematics_U.beta_dot_max;
  s = std::abs(kinematics_Y.current_ICR[0] - kinematics_U.hParams[2]);
  ICR_Y_curr = std::abs(kinematics_Y.current_ICR[1] - kinematics_U.hParams[3]);
  cost[1] = (s * s / ICR_Y_curr + ICR_Y_curr) * kinematics_U.beta_dot_max;
  X_tilde[1] = (ICR_Y_curr * ICR_Y_curr / s + s) * kinematics_U.beta_dot_max;
  s = std::abs(kinematics_Y.current_ICR[0] - kinematics_U.hParams[4]);
  ICR_Y_curr = std::abs(kinematics_Y.current_ICR[1] - kinematics_U.hParams[5]);
  cost[2] = (s * s / ICR_Y_curr + ICR_Y_curr) * kinematics_U.beta_dot_max;
  X_tilde[2] = (ICR_Y_curr * ICR_Y_curr / s + s) * kinematics_U.beta_dot_max;
  s = std::abs(kinematics_Y.current_ICR[0] - kinematics_U.hParams[6]);
  ICR_Y_curr = std::abs(kinematics_Y.current_ICR[1] - kinematics_U.hParams[7]);
  cost[3] = (s * s / ICR_Y_curr + ICR_Y_curr) * kinematics_U.beta_dot_max;
  X_tilde[3] = (ICR_Y_curr * ICR_Y_curr / s + s) * kinematics_U.beta_dot_max;
  smax = kinematics_minimum(cost) * kinematics_U.TS;
  s = kinematics_minimum(X_tilde) * kinematics_U.TS;
  m11[0] = kinematics_Y.input_ICR[0] - kinematics_Y.current_ICR[0];
  m11[1] = kinematics_Y.input_ICR[1] - kinematics_Y.current_ICR[1];
  b_x[0] = (kinematics_Y.input_ICR[0] < kinematics_Y.current_ICR[0] + smax);
  b_x[1] = (kinematics_Y.input_ICR[1] < kinematics_Y.current_ICR[1] + s);
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
    b_x[0] = (kinematics_Y.input_ICR[0] > kinematics_Y.current_ICR[0] - smax);
    b_x[1] = (kinematics_Y.input_ICR[1] > kinematics_Y.current_ICR[1] - s);
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

    if (rEQ0) {
    } else {
      guard1 = true;
    }
  } else {
    guard1 = true;
  }

  if (guard1) {
    rtb_eta_dot_idx_2 = s / m11[1];
    if ((rtb_eta_dot_idx_2 >= 0.0) && (rtb_eta_dot_idx_2 <= 1.0)) {
      ICR_Y_curr = (smax - m11[0] / m11[1] * s) / (2.0 * smax);
      if ((ICR_Y_curr >= 0.0) && (ICR_Y_curr <= 1.0)) {
        kinematics_Y.controller_ICR[0] = rtb_eta_dot_idx_2 * m11[0] +
          kinematics_Y.current_ICR[0];
        kinematics_Y.controller_ICR[1] = rtb_eta_dot_idx_2 * m11[1] +
          kinematics_Y.current_ICR[1];
      }
    }

    rtb_eta_dot_idx_2 = smax / m11[0];
    if ((rtb_eta_dot_idx_2 >= 0.0) && (rtb_eta_dot_idx_2 <= 1.0)) {
      ICR_Y_curr = (s - m11[1] / m11[0] * smax) / (2.0 * s);
      if ((ICR_Y_curr >= 0.0) && (ICR_Y_curr <= 1.0)) {
        kinematics_Y.controller_ICR[0] = rtb_eta_dot_idx_2 * m11[0] +
          kinematics_Y.current_ICR[0];
        kinematics_Y.controller_ICR[1] = rtb_eta_dot_idx_2 * m11[1] +
          kinematics_Y.current_ICR[1];
      }
    }

    rtb_eta_dot_idx_2 = -s / m11[1];
    if ((rtb_eta_dot_idx_2 >= 0.0) && (rtb_eta_dot_idx_2 <= 1.0)) {
      ICR_Y_curr = (smax - m11[0] / m11[1] * s) / (2.0 * smax);
      if ((ICR_Y_curr >= 0.0) && (ICR_Y_curr <= 1.0)) {
        kinematics_Y.controller_ICR[0] = rtb_eta_dot_idx_2 * m11[0] +
          kinematics_Y.current_ICR[0];
        kinematics_Y.controller_ICR[1] = rtb_eta_dot_idx_2 * m11[1] +
          kinematics_Y.current_ICR[1];
      }
    }

    rtb_eta_dot_idx_2 = -smax / m11[0];
    if ((rtb_eta_dot_idx_2 >= 0.0) && (rtb_eta_dot_idx_2 <= 1.0)) {
      ICR_Y_curr = (s - m11[1] / m11[0] * smax) / (2.0 * s);
      if ((ICR_Y_curr >= 0.0) && (ICR_Y_curr <= 1.0)) {
        kinematics_Y.controller_ICR[0] = rtb_eta_dot_idx_2 * m11[0] +
          kinematics_Y.current_ICR[0];
        kinematics_Y.controller_ICR[1] = rtb_eta_dot_idx_2 * m11[1] +
          kinematics_Y.current_ICR[1];
      }
    }
  }

  /* End of MATLAB Function: '<S1>/Controller' */

  /* MATLAB Function: '<S1>/Direct or Complementary Route Decision' */
  rEQ0 = false;
  if ((!(kinematics_Y.input_ICR[0] * kinematics_Y.input_ICR[0] / 0.36 +
         kinematics_Y.input_ICR[1] * kinematics_Y.input_ICR[1] / 0.09 <= 1.0)) &&
      (!(kinematics_Y.current_ICR[0] * kinematics_Y.current_ICR[0] / 0.36 +
         kinematics_Y.current_ICR[1] * kinematics_Y.current_ICR[1] / 0.09 <= 1.0)))
  {
    smax = kinematics_Y.current_ICR[0] - kinematics_Y.input_ICR[0];
    m12[0] = smax;
    c_lambda = smax * -kinematics_Y.input_ICR[0];
    s = smax * smax;
    smax = kinematics_Y.current_ICR[1] - kinematics_Y.input_ICR[1];
    rtb_eta_dot_idx_2 = (smax * -kinematics_Y.input_ICR[1] + c_lambda) / (smax *
      smax + s);
    if (rtb_eta_dot_idx_2 < 0.0) {
      m12[0] = kinematics_Y.input_ICR[0];
      m12[1] = kinematics_Y.input_ICR[1];
    } else if (rtb_eta_dot_idx_2 > 1.0) {
      m12[0] += kinematics_Y.input_ICR[0];
      m12[1] = kinematics_Y.input_ICR[1] + smax;
    } else {
      m12[0] = rtb_eta_dot_idx_2 * m12[0] + kinematics_Y.input_ICR[0];
      m12[1] = rtb_eta_dot_idx_2 * smax + kinematics_Y.input_ICR[1];
    }

    rEQ0 = (m12[0] * m12[0] / 0.36 + m12[1] * m12[1] / 0.09 <= 1.0);
  }

  /* End of MATLAB Function: '<S1>/Direct or Complementary Route Decision' */

  /* CombinatorialLogic: '<S13>/Logic' incorporates:
   *  DataTypeConversion: '<S1>/Data Type Conversion2'
   *  Memory: '<S13>/Memory'
   *  UnitDelay: '<S1>/Unit Delay3'
   */
  kinematics_DW.Memory_PreviousInput = kinematics_ConstP.Logic_table[(((
    static_cast<uint32_T>(rEQ0) << 1) + (kinematics_DW.UnitDelay3_DSTATE != 0.0))
    << 1) + kinematics_DW.Memory_PreviousInput];

  /* MATLAB Function: '<S1>/Optimal Border Point Calculation' incorporates:
   *  Constant: '<S1>/Rmax'
   */
  rtb_eta_dot_idx_2 = m11[1] / m11[0];
  targetPoints[0] = rtb_eta_dot_idx_2 * 0.0 + 50.0;
  targetPoints[1] = rtb_eta_dot_idx_2 * 50.0;
  rtb_eta_dot_idx_2 = -m11[1] / m11[0];
  targetPoints[2] = rtb_eta_dot_idx_2 * 0.0 - 50.0;
  targetPoints[3] = rtb_eta_dot_idx_2 * 50.0;
  rtb_eta_dot_idx_2 = m11[0] / m11[1];
  targetPoints[4] = rtb_eta_dot_idx_2 * 50.0;
  targetPoints[5] = rtb_eta_dot_idx_2 * 0.0 + 50.0;
  rtb_eta_dot_idx_2 = -m11[0] / m11[1];
  targetPoints[6] = rtb_eta_dot_idx_2 * 50.0;
  targetPoints[7] = rtb_eta_dot_idx_2 * 0.0 - 50.0;
  s = (rtInf);
  jj = 0;
  rtb_eta_dot_idx_2 = kinematics_Y.current_ICR[0];
  ICR_Y_curr = kinematics_Y.current_ICR[1];
  for (vk = 0; vk < 4; vk++) {
    smax = 3.3121686421112381E-170;
    jAcol = vk << 1;
    absxk = std::abs(targetPoints[jAcol] - rtb_eta_dot_idx_2);
    if (absxk > 3.3121686421112381E-170) {
      G_tmp = 1.0;
      smax = absxk;
    } else {
      q = absxk / 3.3121686421112381E-170;
      G_tmp = q * q;
    }

    absxk = std::abs(targetPoints[jAcol + 1] - ICR_Y_curr);
    if (absxk > smax) {
      q = smax / absxk;
      G_tmp = G_tmp * q * q + 1.0;
      smax = absxk;
    } else {
      q = absxk / smax;
      G_tmp += q * q;
    }

    smax *= std::sqrt(G_tmp);
    if (smax < s) {
      s = smax;
      jj = vk;
    }
  }

  /* Switch: '<S1>/Switch1' incorporates:
   *  MATLAB Function: '<S1>/Optimal Border Point Calculation'
   *  UnitDelay: '<S1>/Unit Delay5'
   */
  if (!kinematics_DW.UnitDelay5_DSTATE) {
    /* MATLAB Function: '<S1>/Optimal Border Point Calculation' */
    vk = jj << 1;
    kinematics_DW.UnitDelay4_DSTATE[0] = targetPoints[vk];
    kinematics_DW.UnitDelay4_DSTATE[1] = targetPoints[vk + 1];
  }

  /* End of Switch: '<S1>/Switch1' */

  /* MATLAB Function: '<S1>/Route Planning' incorporates:
   *  Switch: '<S1>/Switch1'
   */
  rEQ0 = false;
  if (kinematics_DW.Memory_PreviousInput) {
    smax = 3.3121686421112381E-170;
    absxk = std::abs(kinematics_Y.current_ICR[0] -
                     kinematics_DW.UnitDelay4_DSTATE[0]);
    if (absxk > 3.3121686421112381E-170) {
      G_tmp = 1.0;
      smax = absxk;
    } else {
      q = absxk / 3.3121686421112381E-170;
      G_tmp = q * q;
    }

    absxk = std::abs(kinematics_Y.current_ICR[1] -
                     kinematics_DW.UnitDelay4_DSTATE[1]);
    if (absxk > smax) {
      q = smax / absxk;
      G_tmp = G_tmp * q * q + 1.0;
      smax = absxk;
    } else {
      q = absxk / smax;
      G_tmp += q * q;
    }

    G_tmp = smax * std::sqrt(G_tmp);
    if (G_tmp < 5.0) {
      kinematics_Y.controller_ICR[0] = -kinematics_DW.UnitDelay4_DSTATE[0];
      kinematics_Y.controller_ICR[1] = -kinematics_DW.UnitDelay4_DSTATE[1];
      rEQ0 = true;
    } else {
      kinematics_Y.controller_ICR[0] = kinematics_DW.UnitDelay4_DSTATE[0];
      kinematics_Y.controller_ICR[1] = kinematics_DW.UnitDelay4_DSTATE[1];
    }
  }

  /* End of MATLAB Function: '<S1>/Route Planning' */

  /* MATLAB Function: '<S1>/Feasable ICR Optimization' incorporates:
   *  Inport: '<Root>/TS'
   *  Inport: '<Root>/beta_ddot_max'
   *  Inport: '<Root>/beta_dot_hat'
   *  Inport: '<Root>/beta_dot_max'
   *  Inport: '<Root>/beta_hat'
   *  Inport: '<Root>/hParams'
   */
  ICR_Y_curr = kinematics_U.beta_ddot_max * kinematics_U.TS;
  smax = kinematics_U.beta_dot_hat[0] - ICR_Y_curr;
  if (smax < -kinematics_U.beta_dot_max) {
    smax = -kinematics_U.beta_dot_max;
  }

  s = ICR_Y_curr + kinematics_U.beta_dot_hat[0];
  if (s > kinematics_U.beta_dot_max) {
    s = kinematics_U.beta_dot_max;
  }

  rtb_eta_dot_idx_2 = smax * kinematics_U.TS + kinematics_U.beta_hat[0];
  positiveInput = (rtb_eta_dot_idx_2 > 0.0);
  rtb_eta_dot_idx_2 = kinematics_mod(rtb_eta_dot_idx_2);
  if ((rtb_eta_dot_idx_2 == 0.0) && positiveInput) {
    rtb_eta_dot_idx_2 = 6.2831853071795862;
  }

  cost[0] = rtb_eta_dot_idx_2;
  absxk = s * kinematics_U.TS + kinematics_U.beta_hat[0];
  positiveInput = (absxk > 0.0);
  absxk = kinematics_mod(absxk);
  if ((absxk == 0.0) && positiveInput) {
    absxk = 6.2831853071795862;
  }

  X_tilde[0] = absxk;
  smax = kinematics_U.beta_dot_hat[1] - ICR_Y_curr;
  if (smax < -kinematics_U.beta_dot_max) {
    smax = -kinematics_U.beta_dot_max;
  }

  s = ICR_Y_curr + kinematics_U.beta_dot_hat[1];
  if (s > kinematics_U.beta_dot_max) {
    s = kinematics_U.beta_dot_max;
  }

  rtb_eta_dot_idx_2 = smax * kinematics_U.TS + kinematics_U.beta_hat[1];
  positiveInput = (rtb_eta_dot_idx_2 > 0.0);
  rtb_eta_dot_idx_2 = kinematics_mod(rtb_eta_dot_idx_2);
  if ((rtb_eta_dot_idx_2 == 0.0) && positiveInput) {
    rtb_eta_dot_idx_2 = 6.2831853071795862;
  }

  cost[1] = rtb_eta_dot_idx_2;
  absxk = s * kinematics_U.TS + kinematics_U.beta_hat[1];
  positiveInput = (absxk > 0.0);
  absxk = kinematics_mod(absxk);
  if ((absxk == 0.0) && positiveInput) {
    absxk = 6.2831853071795862;
  }

  X_tilde[1] = absxk;
  smax = kinematics_U.beta_dot_hat[2] - ICR_Y_curr;
  if (smax < -kinematics_U.beta_dot_max) {
    smax = -kinematics_U.beta_dot_max;
  }

  s = ICR_Y_curr + kinematics_U.beta_dot_hat[2];
  if (s > kinematics_U.beta_dot_max) {
    s = kinematics_U.beta_dot_max;
  }

  rtb_eta_dot_idx_2 = smax * kinematics_U.TS + kinematics_U.beta_hat[2];
  positiveInput = (rtb_eta_dot_idx_2 > 0.0);
  rtb_eta_dot_idx_2 = kinematics_mod(rtb_eta_dot_idx_2);
  if ((rtb_eta_dot_idx_2 == 0.0) && positiveInput) {
    rtb_eta_dot_idx_2 = 6.2831853071795862;
  }

  cost[2] = rtb_eta_dot_idx_2;
  absxk = s * kinematics_U.TS + kinematics_U.beta_hat[2];
  positiveInput = (absxk > 0.0);
  absxk = kinematics_mod(absxk);
  if ((absxk == 0.0) && positiveInput) {
    absxk = 6.2831853071795862;
  }

  X_tilde[2] = absxk;
  smax = kinematics_U.beta_dot_hat[3] - ICR_Y_curr;
  if (smax < -kinematics_U.beta_dot_max) {
    smax = -kinematics_U.beta_dot_max;
  }

  s = ICR_Y_curr + kinematics_U.beta_dot_hat[3];
  if (s > kinematics_U.beta_dot_max) {
    s = kinematics_U.beta_dot_max;
  }

  rtb_eta_dot_idx_2 = smax * kinematics_U.TS + kinematics_U.beta_hat[3];
  positiveInput = (rtb_eta_dot_idx_2 > 0.0);
  rtb_eta_dot_idx_2 = kinematics_mod(rtb_eta_dot_idx_2);
  if ((rtb_eta_dot_idx_2 == 0.0) && positiveInput) {
    rtb_eta_dot_idx_2 = 6.2831853071795862;
  }

  cost[3] = rtb_eta_dot_idx_2;
  absxk = s * kinematics_U.TS + kinematics_U.beta_hat[3];
  positiveInput = (absxk > 0.0);
  absxk = kinematics_mod(absxk);
  if ((absxk == 0.0) && positiveInput) {
    absxk = 6.2831853071795862;
  }

  X_tilde[3] = absxk;
  kinematics_Y.feasable_ICR[0] = kinematics_Y.controller_ICR[0];
  kinematics_Y.feasable_ICR[1] = kinematics_Y.controller_ICR[1];
  if (kinemat_eML_blk_kernel_anonFcn1(cost, X_tilde, kinematics_U.hParams,
       kinematics_Y.controller_ICR) > 0.0) {
    d_lambda = std::sin(cost[0] + 1.5707963267948966);
    m11_tmp = std::cos(cost[0] + 1.5707963267948966);
    m11[0] = m11_tmp;
    m11[1] = d_lambda;
    m12[0] = std::cos(X_tilde[0] + 1.5707963267948966);
    m12[1] = std::sin(X_tilde[0] + 1.5707963267948966);
    ICR_Y_curr = std::sin(cost[1] + 1.5707963267948966);
    m21_tmp = std::cos(cost[1] + 1.5707963267948966);
    m21[0] = m21_tmp;
    m21[1] = ICR_Y_curr;
    m22[0] = std::cos(X_tilde[1] + 1.5707963267948966);
    m22[1] = std::sin(X_tilde[1] + 1.5707963267948966);
    s = std::sin(cost[2] + 1.5707963267948966);
    G_tmp = std::cos(cost[2] + 1.5707963267948966);
    m31[0] = G_tmp;
    m31[1] = s;
    m32[0] = std::cos(X_tilde[2] + 1.5707963267948966);
    m32[1] = std::sin(X_tilde[2] + 1.5707963267948966);
    smax = std::sin(rtb_eta_dot_idx_2 + 1.5707963267948966);
    rtb_eta_dot_idx_2 = std::cos(rtb_eta_dot_idx_2 + 1.5707963267948966);
    m41[0] = rtb_eta_dot_idx_2;
    m41[1] = smax;
    m42[0] = std::cos(absxk + 1.5707963267948966);
    m42[1] = std::sin(absxk + 1.5707963267948966);
    absxk = kinematics_Y.controller_ICR[0] - kinematics_U.hParams[0];
    q = absxk;
    c_lambda = m11_tmp * absxk;
    absxk = kinematics_Y.controller_ICR[1] - kinematics_U.hParams[1];
    c_lambda = (d_lambda * absxk + c_lambda) / (m11_tmp * m11_tmp + d_lambda *
      d_lambda);
    closestPoint[0] = m11_tmp * c_lambda + kinematics_U.hParams[0];
    closestPoint[1] = c_lambda * d_lambda + kinematics_U.hParams[1];
    d_lambda = (m12[0] * q + m12[1] * absxk) / (m12[0] * m12[0] + m12[1] * m12[1]);
    closestPoint[2] = d_lambda * m12[0] + kinematics_U.hParams[0];
    absxk = kinematics_Y.controller_ICR[0] - kinematics_U.hParams[2];
    q = absxk;
    m11_tmp = m21_tmp * absxk;
    closestPoint[3] = d_lambda * m12[1] + kinematics_U.hParams[1];
    absxk = kinematics_Y.controller_ICR[1] - kinematics_U.hParams[3];
    d_lambda = (ICR_Y_curr * absxk + m11_tmp) / (m21_tmp * m21_tmp + ICR_Y_curr *
      ICR_Y_curr);
    closestPoint[4] = m21_tmp * d_lambda + kinematics_U.hParams[2];
    closestPoint[5] = d_lambda * ICR_Y_curr + kinematics_U.hParams[3];
    ICR_Y_curr = (m22[0] * q + m22[1] * absxk) / (m22[0] * m22[0] + m22[1] *
      m22[1]);
    closestPoint[6] = ICR_Y_curr * m22[0] + kinematics_U.hParams[2];
    absxk = kinematics_Y.controller_ICR[0] - kinematics_U.hParams[4];
    q = absxk;
    d_lambda = G_tmp * absxk;
    closestPoint[7] = ICR_Y_curr * m22[1] + kinematics_U.hParams[3];
    absxk = kinematics_Y.controller_ICR[1] - kinematics_U.hParams[5];
    ICR_Y_curr = (s * absxk + d_lambda) / (G_tmp * G_tmp + s * s);
    closestPoint[8] = G_tmp * ICR_Y_curr + kinematics_U.hParams[4];
    closestPoint[9] = ICR_Y_curr * s + kinematics_U.hParams[5];
    s = (m32[0] * q + m32[1] * absxk) / (m32[0] * m32[0] + m32[1] * m32[1]);
    closestPoint[10] = s * m32[0] + kinematics_U.hParams[4];
    absxk = kinematics_Y.controller_ICR[0] - kinematics_U.hParams[6];
    q = absxk;
    ICR_Y_curr = rtb_eta_dot_idx_2 * absxk;
    closestPoint[11] = s * m32[1] + kinematics_U.hParams[5];
    absxk = kinematics_Y.controller_ICR[1] - kinematics_U.hParams[7];
    s = (smax * absxk + ICR_Y_curr) / (rtb_eta_dot_idx_2 * rtb_eta_dot_idx_2 +
      smax * smax);
    closestPoint[12] = rtb_eta_dot_idx_2 * s + kinematics_U.hParams[6];
    closestPoint[13] = s * smax + kinematics_U.hParams[7];
    smax = (m42[0] * q + m42[1] * absxk) / (m42[0] * m42[0] + m42[1] * m42[1]);
    closestPoint[14] = smax * m42[0] + kinematics_U.hParams[6];
    closestPoint[15] = smax * m42[1] + kinematics_U.hParams[7];
    for (vk = 0; vk < 8; vk++) {
      jAcol = vk << 1;
      closestPoint_0[0] = closestPoint[jAcol] - kinematics_Y.controller_ICR[0];
      closestPoint_0[1] = closestPoint[jAcol + 1] - kinematics_Y.controller_ICR
        [1];
      smax = kinematics_norm(closestPoint_0);
      targetPoints[vk] = kinemat_eML_blk_kernel_anonFcn1(cost, X_tilde,
        kinematics_U.hParams, &closestPoint[jAcol]) * 250000.0 + smax * smax;
    }

    std::memset(&intersections[0], 0, 72U * sizeof(real_T));
    kinematics_schnittpunkte(&kinematics_U.hParams[0], &kinematics_U.hParams[2],
      m11, m12, m21, m22, 500.0, &intersections[0]);
    kinematics_schnittpunkte(&kinematics_U.hParams[0], &kinematics_U.hParams[4],
      m11, m12, m31, m32, 500.0, &intersections[12]);
    kinematics_schnittpunkte(&kinematics_U.hParams[0], &kinematics_U.hParams[6],
      m11, m12, m41, m42, 500.0, &intersections[24]);
    kinematics_schnittpunkte(&kinematics_U.hParams[2], &kinematics_U.hParams[4],
      m21, m22, m31, m32, 500.0, &intersections[36]);
    kinematics_schnittpunkte(&kinematics_U.hParams[2], &kinematics_U.hParams[6],
      m21, m22, m41, m42, 500.0, &intersections[48]);
    kinematics_schnittpunkte(&kinematics_U.hParams[4], &kinematics_U.hParams[6],
      m31, m32, m41, m42, 500.0, &intersections[60]);
    for (vk = 0; vk < 36; vk++) {
      jj = vk << 1;
      m11[0] = intersections[jj] - kinematics_Y.controller_ICR[0];
      m11[1] = intersections[jj + 1] - kinematics_Y.controller_ICR[1];
      smax = kinematics_norm(m11);
      intersectionsCost[vk] = kinemat_eML_blk_kernel_anonFcn1(cost, X_tilde,
        kinematics_U.hParams, &intersections[jj]) * 250000.0 + smax * smax;
    }

    if (!std::isnan(targetPoints[0])) {
      jj = 1;
    } else {
      jj = 0;
      vk = 2;
      exitg1 = false;
      while ((!exitg1) && (vk <= 8)) {
        if (!std::isnan(targetPoints[vk - 1])) {
          jj = vk;
          exitg1 = true;
        } else {
          vk++;
        }
      }
    }

    if (jj == 0) {
      smax = targetPoints[0];
    } else {
      smax = targetPoints[jj - 1];
      for (vk = jj + 1; vk < 9; vk++) {
        d_lambda = targetPoints[vk - 1];
        if (smax > d_lambda) {
          smax = d_lambda;
        }
      }
    }

    d_lambda = kinematics_minimum_k(intersectionsCost);
    if (smax < d_lambda) {
      smax = (rtInf);
      vk = -1;
      for (jj = 0; jj < 8; jj++) {
        s = targetPoints[jj];
        if (s < smax) {
          vk = jj;
          smax = s;
        }
      }

      vk <<= 1;
      kinematics_Y.feasable_ICR[0] = closestPoint[vk];
      kinematics_Y.feasable_ICR[1] = closestPoint[vk + 1];
    } else if (d_lambda < (rtInf)) {
      smax = (rtInf);
      vk = -1;
      for (jAcol = 0; jAcol < 36; jAcol++) {
        s = intersectionsCost[jAcol];
        if (s < smax) {
          vk = jAcol;
          smax = s;
        }
      }

      vk <<= 1;
      kinematics_Y.feasable_ICR[0] = intersections[vk];
      kinematics_Y.feasable_ICR[1] = intersections[vk + 1];
    }
  }

  /* End of MATLAB Function: '<S1>/Feasable ICR Optimization' */
  for (vk = 0; vk < 4; vk++) {
    /* MATLAB Function: '<S1>/ICR2SteerAngles' incorporates:
     *  Inport: '<Root>/hParams'
     */
    jj = vk << 1;
    kinematics_Y.beta_next[vk] = rt_atan2d_snf(kinematics_Y.feasable_ICR[1] -
      kinematics_U.hParams[jj + 1], kinematics_Y.feasable_ICR[0] -
      kinematics_U.hParams[jj]) - 1.5707963267948966;

    /* MATLAB Function: '<S1>/SteerAngles2SteerSpeed' incorporates:
     *  Inport: '<Root>/beta_hat'
     */
    s = kinematics_mod(kinematics_Y.beta_next[vk]);
    if ((s == 0.0) && (kinematics_Y.beta_next[vk] > 0.0)) {
      s = 6.2831853071795862;
    }

    ICR_Y_curr = kinematics_mod(s + 3.1415926535897931);
    if ((ICR_Y_curr == 0.0) && (s + 3.1415926535897931 > 0.0)) {
      ICR_Y_curr = 6.2831853071795862;
    }

    smax = kinematics_mod(kinematics_U.beta_hat[vk]);
    if ((smax == 0.0) && (kinematics_U.beta_hat[vk] > 0.0)) {
      smax = 6.2831853071795862;
    }

    s -= smax;
    if ((s < -3.1415926535897931) || (s > 3.1415926535897931)) {
      rtb_eta_dot_idx_2 = kinematics_mod(s + 3.1415926535897931);
      if ((rtb_eta_dot_idx_2 == 0.0) && (s + 3.1415926535897931 > 0.0)) {
        rtb_eta_dot_idx_2 = 6.2831853071795862;
      }

      s = rtb_eta_dot_idx_2 - 3.1415926535897931;
    }

    ICR_Y_curr -= smax;
    if ((ICR_Y_curr < -3.1415926535897931) || (ICR_Y_curr > 3.1415926535897931))
    {
      absxk = kinematics_mod(ICR_Y_curr + 3.1415926535897931);
      if ((absxk == 0.0) && (ICR_Y_curr + 3.1415926535897931 > 0.0)) {
        absxk = 6.2831853071795862;
      }

      ICR_Y_curr = absxk - 3.1415926535897931;
    }

    /* Outport: '<Root>/Beta_dot' incorporates:
     *  Inport: '<Root>/TS'
     *  MATLAB Function: '<S1>/SteerAngles2SteerSpeed'
     */
    kinematics_Y.Beta_dot[vk] = s / kinematics_U.TS;

    /* MATLAB Function: '<S1>/SteerAngles2SteerSpeed' incorporates:
     *  Inport: '<Root>/TS'
     *  Outport: '<Root>/Beta_dot'
     */
    if (std::abs(ICR_Y_curr) < std::abs(s)) {
      kinematics_Y.Beta_dot[vk] = ICR_Y_curr / kinematics_U.TS;
    }
  }

  /* DataTypeConversion: '<S1>/Data Type Conversion1' */
  kinematics_B.DataTypeConversion1 = kinematics_DW.Memory_PreviousInput;

  /* DiscreteFir: '<S1>/Discrete FIR Filter1' incorporates:
   *  DataTypeConversion: '<S1>/Data Type Conversion2'
   *  UnitDelay: '<S1>/Unit Delay3'
   */
  if ((((kinematics_DW.UnitDelay3_DSTATE == 0.0) ==
        (kinematics_PrevZCX.DiscreteFIRFilter1_Reset_ZCE == POS_ZCSIG)) &&
       (kinematics_PrevZCX.DiscreteFIRFilter1_Reset_ZCE != UNINITIALIZED_ZCSIG))
      || (kinematics_DW.UnitDelay3_DSTATE != 0.0)) {
    kinematics_DW.DiscreteFIRFilter1_circBuf = 0;
    std::memset(&kinematics_DW.DiscreteFIRFilter1_states[0], 0, 23U * sizeof
                (real_T));
  }

  kinematics_PrevZCX.DiscreteFIRFilter1_Reset_ZCE =
    (kinematics_DW.UnitDelay3_DSTATE != 0.0);
  rtb_eta_dot_idx_2 = 0.0;
  jj = 1;
  for (vk = kinematics_DW.DiscreteFIRFilter1_circBuf; vk < 23; vk++) {
    rtb_eta_dot_idx_2 += kinematics_DW.DiscreteFIRFilter1_states[vk] *
      kinematics_ConstP.DiscreteFIRFilter1_Coefficients[jj];
    jj++;
  }

  for (vk = 0; vk < kinematics_DW.DiscreteFIRFilter1_circBuf; vk++) {
    rtb_eta_dot_idx_2 += kinematics_DW.DiscreteFIRFilter1_states[vk] *
      kinematics_ConstP.DiscreteFIRFilter1_Coefficients[jj];
    jj++;
  }

  /* Update for DiscreteIntegrator: '<S1>/Discrete-Time Integrator2' */
  d_lambda = kinematics_DW.DiscreteTimeIntegrator2_DSTATE[0];

  /* Sum: '<S1>/Sum3' incorporates:
   *  DiscreteIntegrator: '<S1>/Discrete-Time Integrator2'
   */
  ICR_Y_curr = kinematics_DW.DiscreteTimeIntegrator2_DSTATE[0];

  /* Update for DiscreteIntegrator: '<S1>/Discrete-Time Integrator2' */
  s = kinematics_DW.DiscreteTimeIntegrator2_DSTATE[1];

  /* Sum: '<S1>/Sum3' incorporates:
   *  DiscreteIntegrator: '<S1>/Discrete-Time Integrator2'
   */
  smax = kinematics_DW.DiscreteTimeIntegrator2_DSTATE[1];

  /* Update for DiscreteIntegrator: '<S1>/Discrete-Time Integrator2' */
  absxk = kinematics_DW.DiscreteTimeIntegrator2_DSTATE[2];

  /* Sum: '<S1>/Sum3' incorporates:
   *  DiscreteIntegrator: '<S1>/Discrete-Time Integrator2'
   */
  q = kinematics_DW.DiscreteTimeIntegrator2_DSTATE[2];

  /* Update for DiscreteIntegrator: '<S1>/Discrete-Time Integrator2' incorporates:
   *  Inport: '<Root>/U'
   *  Inport: '<Root>/X_dot'
   *  Inport: '<Root>/Y_dot'
   *  Inport: '<Root>/acceleration_factor'
   *  Product: '<S1>/Matrix Multiply'
   *  Sum: '<S1>/Sum3'
   */
  kinematics_DW.DiscreteTimeIntegrator2_DSTATE[0] = (kinematics_U.VX_out -
    ICR_Y_curr) * kinematics_U.acceleration_factor * 0.06 + d_lambda;
  kinematics_DW.DiscreteTimeIntegrator2_DSTATE[1] = (kinematics_U.VY_out - smax)
    * kinematics_U.acceleration_factor * 0.06 + s;
  kinematics_DW.DiscreteTimeIntegrator2_DSTATE[2] = (kinematics_U.U - q) *
    kinematics_U.acceleration_factor * 0.06 + absxk;

  /* Update for UnitDelay: '<S1>/Unit Delay3' incorporates:
   *  DiscreteFir: '<S1>/Discrete FIR Filter1'
   *  Sum: '<S1>/Sum1'
   */
  kinematics_DW.UnitDelay3_DSTATE = static_cast<real_T>(rEQ0) +
    rtb_eta_dot_idx_2;

  /* Update for UnitDelay: '<S1>/Unit Delay5' */
  kinematics_DW.UnitDelay5_DSTATE = kinematics_DW.Memory_PreviousInput;

  /* Update for DiscreteFir: '<S1>/Discrete FIR Filter1' */
  /* Update circular buffer index */
  kinematics_DW.DiscreteFIRFilter1_circBuf--;
  if (kinematics_DW.DiscreteFIRFilter1_circBuf < 0) {
    kinematics_DW.DiscreteFIRFilter1_circBuf = 22;
  }

  /* Update circular buffer */
  kinematics_DW.DiscreteFIRFilter1_states[kinematics_DW.DiscreteFIRFilter1_circBuf]
    = kinematics_B.DataTypeConversion1;

  /* End of Update for DiscreteFir: '<S1>/Discrete FIR Filter1' */
  /* End of Outputs for SubSystem: '<Root>/kinematics' */

  /* Outport: '<Root>/border_ICR' incorporates:
   *  Switch: '<S1>/Switch1'
   */
  kinematics_Y.border_ICR[0] = kinematics_DW.UnitDelay4_DSTATE[0];
  kinematics_Y.border_ICR[1] = kinematics_DW.UnitDelay4_DSTATE[1];

  /* Outport: '<Root>/indirect_mode' */
  kinematics_Y.indirect_mode = kinematics_DW.Memory_PreviousInput;
}

/* Model initialize function */
void kinematics::initialize()
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));
  kinematics_PrevZCX.DiscreteFIRFilter1_Reset_ZCE = UNINITIALIZED_ZCSIG;
}

/* Model terminate function */
void kinematics::terminate()
{
  /* (no terminate code required) */
}

/* Constructor */
kinematics::kinematics() :
  kinematics_U(),
  kinematics_Y(),
  kinematics_B(),
  kinematics_DW(),
  kinematics_PrevZCX(),
  kinematics_M()
{
  /* Currently there is no constructor body generated.*/
}

/* Destructor */
/* Currently there is no destructor body generated.*/
kinematics::~kinematics() = default;

/* Real-Time Model get method */
RT_MODEL_kinematics_T * kinematics::getRTM()
{
  return (&kinematics_M);
}
