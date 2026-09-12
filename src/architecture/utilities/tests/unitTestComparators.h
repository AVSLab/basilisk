/*
 ISC License

 Copyright (c) 2023, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

 Permission to use, copy, modify, and/or distribute this software for any
 purpose with or without fee is hereby granted, provided that the above
 copyright notice and this permission notice appear in all copies.

 THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

 */

#ifndef UNITTESTCOMPARATORS_H
#define UNITTESTCOMPARATORS_H

#include <math.h>

/**
 * @brief Compare finite scalars with an inclusive absolute tolerance.
 * @param a First value, in the same units as b and accuracy.
 * @param b Second value.
 * @param accuracy Finite, nonnegative absolute tolerance.
 * @return Nonzero if the values satisfy the tolerance, otherwise zero.
 */
static inline int
isEqual(double a, double b, double accuracy)
{
    return isfinite(a) && isfinite(b) && isfinite(accuracy) && accuracy >= 0.0 && fabs(a - b) <= accuracy;
}

/**
 * @brief Compare finite scalars relative to the magnitude of the first value.
 * @param a Reference value, in the same units as b.
 * @param b Second value.
 * @param accuracy Finite, nonnegative relative tolerance (dimensionless).
 * @return Nonzero if abs(a - b) / abs(a) is at most accuracy, otherwise zero.
 * @note A zero reference or zero tolerance requires exact equality. NaN and
 * infinity are rejected, including equal infinities.
 */
static inline int
isEqualRel(double a, double b, double accuracy)
{
    if (!isfinite(a) || !isfinite(b) || !isfinite(accuracy) || accuracy < 0.0) {
        return 0;
    }
    if (a == 0.0 || accuracy == 0.0) {
        return a == b;
    }
    const double difference = fabs(a - b);
    // Divide before subtracting only when subtraction overflows.
    const double relativeError = isfinite(difference) ? difference / fabs(a) : fabs(a / fabs(a) - b / fabs(a));
    return relativeError <= accuracy;
}

#ifdef __cplusplus

#include <algorithm>
#include <cmath>
#include <gtest/gtest.h>

namespace unitTestComparators {

/**
 * @brief Format a scalar relative-comparison failure for GoogleTest.
 * @param firstExpression Source expression for first.
 * @param secondExpression Source expression for second.
 * @param toleranceExpression Source expression for tolerance.
 * @param first Reference value, in the same units as second.
 * @param second Value to compare.
 * @param tolerance Finite, nonnegative relative tolerance (dimensionless).
 * @return An assertion result with the operands and tolerance on failure.
 */
static inline ::testing::AssertionResult
scalarNearRel(const char* firstExpression,
              const char* secondExpression,
              const char* toleranceExpression,
              double first,
              double second,
              double tolerance)
{
    if (isEqualRel(first, second, tolerance)) {
        return ::testing::AssertionSuccess();
    }
    return ::testing::AssertionFailure()
           << "Relative comparison failed: " << firstExpression << " = " << first << ", " << secondExpression << " = "
           << second << ", " << toleranceExpression << " = " << tolerance
           << ". Required: abs(first - second) / abs(first) <= tolerance; a zero reference or zero tolerance "
              "requires exact equality. Values must be finite and tolerance must be finite and nonnegative.";
}

/**
 * @brief Compare three finite vector components and format the first failure.
 * @param firstExpression Source expression for first.
 * @param secondExpression Source expression for second.
 * @param toleranceExpression Source expression for tolerance.
 * @param first Reference vector with three readable components.
 * @param second Vector with three readable components, in the same units as first.
 * @param tolerance Finite, nonnegative tolerance; dimensionless for relative comparisons.
 * @tparam relative Whether to divide each component error by the Euclidean norm of first.
 * @return An assertion result including both vectors and the failing component on failure.
 * @note A zero reference vector or zero tolerance requires exact equality. NaN and
 * infinity are rejected, including equal infinities. Absolute comparisons use the
 * vector's units and check each component independently.
 */
template<bool relative>
inline ::testing::AssertionResult
vector3Near(const char* firstExpression,
            const char* secondExpression,
            const char* toleranceExpression,
            const double* first,
            const double* second,
            double tolerance)
{
    if (!std::isfinite(tolerance) || tolerance < 0.0) {
        return ::testing::AssertionFailure()
               << toleranceExpression << " = " << tolerance << "; tolerance must be finite and nonnegative.";
    }
    for (int i = 0; i < 3; ++i) {
        if (!std::isfinite(first[i]) || !std::isfinite(second[i])) {
            return ::testing::AssertionFailure()
                   << "Non-finite vector component " << i << ": " << firstExpression << "[" << i << "] = " << first[i]
                   << ", " << secondExpression << "[" << i << "] = " << second[i];
        }
    }

    double scale = 1.0;      // [-] Absolute comparisons do not normalize the error.
    double scaledNorm = 1.0; // [-]
    if (relative) {
        scale = std::hypot(std::hypot(first[0], first[1]), first[2]);
        if (!std::isfinite(scale) || std::fpclassify(scale) == FP_SUBNORMAL) {
            // Keep the norm factored to avoid overflow and rounding in the subnormal range.
            scale = std::max({ std::fabs(first[0]), std::fabs(first[1]), std::fabs(first[2]) });
            scaledNorm = std::hypot(std::hypot(first[0] / scale, first[1] / scale), first[2] / scale);
        }
    }
    for (int i = 0; i < 3; ++i) {
        const double difference = std::fabs(first[i] - second[i]);
        double error = scale == 0.0 ? difference
                                    : (std::isfinite(difference) ? difference / scale
                                                                 : std::fabs(first[i] / scale - second[i] / scale)) /
                                        scaledNorm;
        if (std::isinf(error) && std::isfinite(difference) && scaledNorm > 1.0) {
            // Dividing by a tiny scale can overflow before the scaled norm reduces the result.
            error = (difference / scaledNorm) / scale;
        }
        const bool equal = (scale == 0.0 || tolerance == 0.0) ? first[i] == second[i] : error <= tolerance;
        if (!equal) {
            return ::testing::AssertionFailure()
                   << (relative ? "Relative" : "Absolute") << " vector comparison failed at component " << i << ": "
                   << firstExpression << " = [" << first[0] << ", " << first[1] << ", " << first[2] << "], "
                   << secondExpression << " = [" << second[0] << ", " << second[1] << ", " << second[2] << "], "
                   << toleranceExpression << " = " << tolerance << ", component error = " << error
                   << (relative ? ". Errors are normalized by norm(first); a zero reference requires exact equality."
                                : ". Errors are absolute component differences.");
        }
    }
    return ::testing::AssertionSuccess();
}

} // namespace unitTestComparators

/**
 * @brief Nonfatal scalar assertion using abs(first - second) / abs(first) <= tolerance.
 * @param first Finite reference scalar.
 * @param second Finite scalar in the same units as first.
 * @param tolerance Finite, nonnegative dimensionless relative tolerance.
 * @note A zero reference or zero tolerance requires exact equality. Each argument
 * is evaluated once. A diagnostic can be appended with the stream operator.
 */
#define EXPECT_NEAR_REL(first, second, tolerance)                                                                      \
    EXPECT_PRED_FORMAT3(::unitTestComparators::scalarNearRel, first, second, tolerance)

/**
 * @brief Nonfatal assertion that each of three component errors is at most tolerance.
 * @param first Pointer or array containing three finite reference components.
 * @param second Pointer or array containing three finite components in the same units as first.
 * @param tolerance Finite, nonnegative absolute tolerance, in the vector's units.
 * @note Each argument is evaluated once. A diagnostic can be appended with the stream operator.
 */
#define EXPECT_VECTOR3_NEAR(first, second, tolerance)                                                                  \
    EXPECT_PRED_FORMAT3(::unitTestComparators::vector3Near<false>, first, second, tolerance)

/**
 * @brief Nonfatal assertion that each component error / norm(first) is at most tolerance.
 * @param first Pointer or array containing three finite reference components.
 * @param second Pointer or array containing three finite components in the same units as first.
 * @param tolerance Finite, nonnegative dimensionless relative tolerance.
 * @note A zero reference vector or zero tolerance requires exact equality. Each argument
 * is evaluated once. A diagnostic can be appended with the stream operator.
 */
#define EXPECT_VECTOR3_NEAR_REL(first, second, tolerance)                                                              \
    EXPECT_PRED_FORMAT3(::unitTestComparators::vector3Near<true>, first, second, tolerance)

#endif // __cplusplus

#endif // UNITTESTCOMPARATORS_H
