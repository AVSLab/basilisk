/*
 ISC License

 Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#ifndef stochasticTableauValidation_h
#define stochasticTableauValidation_h

#include <array>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <tuple>
#include <type_traits>

/** @brief Constructor-time checks shared by explicit stochastic RK tableau families.
 * These validate finite coefficients and explicit stage dependencies, not the
 * mathematical order conditions or applicability of a method to a particular SDE.
 */
namespace stochastic_tableau {
/** @brief Reject a nonfinite entry before coefficients can enter stage arithmetic. */
template<size_t numberStages>
void
validateFinite(const std::array<double, numberStages>& coefficients)
{
    for (double coefficient : coefficients) {
        if (!std::isfinite(coefficient)) {
            throw std::invalid_argument("Stochastic Runge-Kutta coefficients must all be finite.");
        }
    }
}

/** @brief Require finite coefficients and zero diagonal/upper triangle for explicit stages. */
template<size_t numberStages>
void
validateExplicitMatrix(const std::array<std::array<double, numberStages>, numberStages>& matrix)
{
    using Matrix = typename std::decay<decltype(matrix)>::type;
    static_assert(std::tuple_size<Matrix>::value == numberStages,
                  "Stochastic Runge-Kutta coefficient matrices must be square.");
    static_assert(std::tuple_size<typename Matrix::value_type>::value == numberStages,
                  "Stochastic Runge-Kutta coefficient matrices must be square.");

    for (size_t rowIndex = 0; rowIndex < numberStages; ++rowIndex) {
        validateFinite(matrix[rowIndex]);
        for (size_t columnIndex = rowIndex; columnIndex < numberStages; ++columnIndex) {
            if (matrix[rowIndex][columnIndex] != 0.0) {
                throw std::invalid_argument("Explicit stochastic Runge-Kutta coefficient matrices "
                                            "must be strictly lower triangular.");
            }
        }
    }
}
}

#endif /* stochasticTableauValidation_h */
