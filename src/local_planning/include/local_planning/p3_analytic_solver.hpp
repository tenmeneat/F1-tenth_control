// Copyright 2026 2026_IFAC contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

// Production-owned copy of the validated closed-form primitives for the frozen P3 harmonic
// equations. This header deliberately contains no planner policy and no lateral search.

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace p3_analytic_solver
{

inline constexpr const char * kValidatedSourceSha256 =
  "a5accda81bc18dade6801dcb66973070b65d6f518";

constexpr double kSourceResidualTolerance = 1.0e-12;
constexpr double kCoefficientRelativeTolerance =
  128.0 * std::numeric_limits<double>::epsilon();
constexpr double kDiscriminantRelativeTolerance =
  256.0 * std::numeric_limits<double>::epsilon();
constexpr double kDedupRelativeTolerance =
  128.0 * std::numeric_limits<double>::epsilon();

enum class ActiveBranch
{
  SameSignPositive,
  SameSignNegative,
};

inline std::string branchName(ActiveBranch branch)
{
  return branch == ActiveBranch::SameSignPositive ?
         "SAME_SIGN_POSITIVE" : "SAME_SIGN_NEGATIVE";
}

inline std::string sourceBranch(double left, double right)
{
  if (left == 0.0 && right == 0.0) {
    return "ZERO_BOTH";
  }
  if (left == 0.0) {
    return "ZERO_LEFT";
  }
  if (right == 0.0) {
    return "ZERO_RIGHT";
  }
  if (left > 0.0 && right > 0.0) {
    return "SAME_SIGN_POSITIVE";
  }
  if (left < 0.0 && right < 0.0) {
    return "SAME_SIGN_NEGATIVE";
  }
  return "SIGN_CHANGE";
}

inline double sourceHarmonicDerivative(
  double h_left, double h_right, double left, double right)
{
  return left * right > 0.0 ?
         (h_left + h_right) / (h_left / left + h_right / right) : 0.0;
}

struct FractionalMap
{
  // m(x) = (a*x+b)/(c*x+e), where x is d_mid.
  double a{0.0};
  double b{0.0};
  double c{0.0};
  double e{0.0};
  double h_left{0.0};
  double h_right{0.0};
  double d_fixed_outer{0.0};
  double d_target{0.0};
  bool variable_on_right{true};
  ActiveBranch assumed_branch{ActiveBranch::SameSignPositive};
};

inline FractionalMap makeEntryMap(
  double h_entry, double h_inner, double d_ego, double d_target,
  ActiveBranch branch)
{
  const double fixed_slope = (d_target - d_ego) / h_entry;
  const double k = (h_entry + h_inner) * fixed_slope;
  return {
    k,
    -k * d_target,
    h_entry,
    h_inner * h_inner * fixed_slope - h_entry * d_target,
    h_entry,
    h_inner,
    d_ego,
    d_target,
    true,
    branch,
  };
}

inline FractionalMap makeExitMap(
  double h_inner, double h_exit, double d_target, double d_final,
  ActiveBranch branch)
{
  const double fixed_slope = (d_final - d_target) / h_exit;
  const double k = (h_inner + h_exit) * fixed_slope;
  return {
    -k,
    k * d_target,
    -h_exit,
    h_inner * h_inner * fixed_slope + h_exit * d_target,
    h_inner,
    h_exit,
    d_final,
    d_target,
    false,
    branch,
  };
}

inline std::array<double, 2> secants(const FractionalMap & map, double d_mid)
{
  if (map.variable_on_right) {
    return {
      (map.d_target - map.d_fixed_outer) / map.h_left,
      (d_mid - map.d_target) / map.h_right,
    };
  }
  return {
    (map.d_target - d_mid) / map.h_left,
    (map.d_fixed_outer - map.d_target) / map.h_right,
  };
}

inline double evaluateFractional(const FractionalMap & map, double d_mid)
{
  return (map.a * d_mid + map.b) / (map.c * d_mid + map.e);
}

inline double evaluateExactSource(const FractionalMap & map, double d_mid)
{
  const auto q = secants(map, d_mid);
  return sourceHarmonicDerivative(map.h_left, map.h_right, q[0], q[1]);
}

inline double derivativeWrtMid(const FractionalMap & map, double d_mid)
{
  const double denominator = map.c * d_mid + map.e;
  return (map.a * map.e - map.b * map.c) / (denominator * denominator);
}

inline double harmonicDenominator(const FractionalMap & map, double d_mid)
{
  const auto q = secants(map, d_mid);
  return map.h_left * q[1] + map.h_right * q[0];
}

inline double normalizedHarmonicDenominator(const FractionalMap & map, double d_mid)
{
  const auto q = secants(map, d_mid);
  const double first = map.h_left * q[1];
  const double second = map.h_right * q[0];
  const double scale = std::abs(first) + std::abs(second);
  return scale == 0.0 ? 0.0 : std::abs(first + second) / scale;
}

inline std::string conditionFlag(const FractionalMap & map, double d_mid)
{
  const double denominator = harmonicDenominator(map, d_mid);
  if (denominator == 0.0) {
    return "SINGULAR";
  }
  const double normalized = normalizedHarmonicDenominator(map, d_mid);
  if (normalized <= kCoefficientRelativeTolerance) {
    return "NEAR_SINGULAR";
  }
  if (normalized <= std::sqrt(std::numeric_limits<double>::epsilon())) {
    return "ELEVATED_CONDITIONING";
  }
  return "WELL_CONDITIONED";
}

struct InverseResult
{
  std::vector<double> raw_roots;
  std::vector<double> branch_roots;
  std::vector<double> bounded_roots;
  std::string algebraic_status;
  double inverse_denominator{0.0};
  double inverse_numerator{0.0};
  double harmonic_denominator{std::numeric_limits<double>::quiet_NaN()};
  double dm_dd_mid{std::numeric_limits<double>::quiet_NaN()};
  double inverse_jacobian{std::numeric_limits<double>::infinity()};
  double forward_residual{std::numeric_limits<double>::infinity()};
  std::string condition_flag{"SINGULAR"};
};

inline bool sameRoot(double first, double second)
{
  return std::abs(first - second) <= kDedupRelativeTolerance *
         std::max({1.0, std::abs(first), std::abs(second)});
}

inline void sortAndDeduplicate(std::vector<double> & roots)
{
  std::sort(roots.begin(), roots.end());
  if (roots.size() == 2U && sameRoot(roots[0], roots[1])) {
    roots.resize(1U);
  }
}

inline InverseResult invertSideDerivative(
  const FractionalMap & map, double desired_derivative,
  double lower_bound, double upper_bound)
{
  InverseResult result;
  result.inverse_denominator = desired_derivative * map.c - map.a;
  result.inverse_numerator = map.b - desired_derivative * map.e;
  const double coefficient_scale = std::max({
      1.0, std::abs(desired_derivative * map.c), std::abs(map.a),
      std::abs(map.b), std::abs(desired_derivative * map.e)});
  const double zero_tolerance = kCoefficientRelativeTolerance * coefficient_scale;
  if (std::abs(result.inverse_denominator) <= zero_tolerance) {
    result.algebraic_status = std::abs(result.inverse_numerator) <= zero_tolerance ?
      "DENOMINATOR_SINGULAR_INDETERMINATE" : "DENOMINATOR_SINGULAR_NO_SOLUTION";
    return result;
  }
  const double root = result.inverse_numerator / result.inverse_denominator;
  if (!std::isfinite(root)) {
    result.algebraic_status = "NONFINITE_ROOT";
    return result;
  }
  result.raw_roots.push_back(root);
  const auto q = secants(map, root);
  if (sourceBranch(q[0], q[1]) != branchName(map.assumed_branch)) {
    result.algebraic_status = "BRANCH_VIOLATION";
    return result;
  }
  result.branch_roots.push_back(root);
  result.harmonic_denominator = harmonicDenominator(map, root);
  result.dm_dd_mid = derivativeWrtMid(map, root);
  result.inverse_jacobian = result.dm_dd_mid == 0.0 ?
    std::numeric_limits<double>::infinity() : std::abs(1.0 / result.dm_dd_mid);
  result.forward_residual = evaluateExactSource(map, root) - desired_derivative;
  result.condition_flag = conditionFlag(map, root);
  if (root < lower_bound || root > upper_bound) {
    result.algebraic_status = "LATERAL_BOUND_VIOLATION";
    return result;
  }
  const double residual_limit = kSourceResidualTolerance *
    std::max(1.0, std::abs(desired_derivative));
  if (std::abs(result.forward_residual) > residual_limit) {
    result.algebraic_status = "FORWARD_RESIDUAL_VIOLATION";
    return result;
  }
  result.bounded_roots.push_back(root);
  result.algebraic_status = "VALID_FINITE_ROOT";
  return result;
}

struct Polynomial
{
  double quadratic{0.0};
  double linear{0.0};
  double constant{0.0};
};

struct PolynomialRoots
{
  std::vector<double> raw_roots;
  std::string equation_class;
  double discriminant{std::numeric_limits<double>::quiet_NaN()};
  double discriminant_tolerance{0.0};
  bool roundoff_discriminant_clamped{false};
};

inline Polynomial samplePolynomial(
  double affine_slope, double affine_intercept, double derivative_weight,
  const FractionalMap & map, double desired_sample)
{
  const double shifted = affine_intercept - desired_sample;
  return {
    affine_slope * map.c,
    affine_slope * map.e + shifted * map.c + derivative_weight * map.a,
    shifted * map.e + derivative_weight * map.b,
  };
}

inline PolynomialRoots solvePolynomialStable(const Polynomial & input)
{
  PolynomialRoots result;
  const double scale = std::max({
      std::abs(input.quadratic), std::abs(input.linear),
      std::abs(input.constant), std::numeric_limits<double>::min()});
  const double a = input.quadratic / scale;
  const double b = input.linear / scale;
  const double c = input.constant / scale;
  if (input.quadratic == 0.0) {
    if (input.linear == 0.0) {
      result.equation_class = input.constant == 0.0 ?
        "CONSTANT_ALL_ROOTS" : "CONSTANT_NO_ROOT";
      return result;
    }
    result.raw_roots.push_back(-c / b);
    result.equation_class = "LINEAR_ONE_ROOT";
    return result;
  }
  const double square = b * b;
  const double product = 4.0 * a * c;
  double discriminant = square - product;
  result.discriminant_tolerance = kDiscriminantRelativeTolerance *
    std::max(std::abs(square) + std::abs(product), std::numeric_limits<double>::min());
  result.discriminant = discriminant;
  if (discriminant < 0.0 && discriminant >= -result.discriminant_tolerance) {
    discriminant = 0.0;
    result.roundoff_discriminant_clamped = true;
  }
  if (discriminant < 0.0) {
    result.equation_class = "QUADRATIC_NO_REAL_ROOT";
    return result;
  }
  const double root_discriminant = std::sqrt(discriminant);
  if (root_discriminant == 0.0) {
    result.raw_roots.push_back(-0.5 * b / a);
    result.equation_class = "QUADRATIC_DOUBLE_ROOT";
    return result;
  }
  const double q = -0.5 * (b + std::copysign(root_discriminant, b));
  result.raw_roots.push_back(q / a);
  result.raw_roots.push_back(c / q);
  sortAndDeduplicate(result.raw_roots);
  result.equation_class = result.raw_roots.size() == 1U ?
    "QUADRATIC_NUMERICALLY_DUPLICATE_ROOT" : "QUADRATIC_TWO_ROOTS";
  return result;
}

inline double polynomialResidual(const Polynomial & polynomial, double root)
{
  return (polynomial.quadratic * root + polynomial.linear) * root +
         polynomial.constant;
}

}  // namespace p3_analytic_solver
