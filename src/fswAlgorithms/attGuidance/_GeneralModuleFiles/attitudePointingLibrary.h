/*
 ISC License

 Copyright (c) 2023, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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


#include "architecture/utilities/macroDefinitions.h"
#include <stdint.h>

//! @brief The SolutionSpace class contains the solutions of the nonlinear inequality
//! |Ax^2 + Bx + C| / (1+x^2) >= 0
class SolutionSpace {
public:
    SolutionSpace(double A, double B, double C, double tol);
    ~SolutionSpace();

    bool   isEmpty();                //!< returns whether the solution set is empty or not
    int    numberOfZeros();          //!< returns the number of zeros of the associated equation
    double returnAbsMin(int idx);    //!< return the minimum value of f(x) = |Ax^2 + Bx + C| / (1+x^2)
    bool   contains(double psi);     //!< determines whether psi is contained in the solution space
    double passThrough(double psi);  //!< "passes" psi through the solution space

private:
    bool   emptySet{};          //!< determines whether the solution space is empty
    double inf{};               //!< inferior end of the solution space
    double sup{};               //!< superior end of the solution space
    double zero[2]{};           //!< zero(s) of the associated equation
    int    zeros{};             //!< number of distinct solutions of the associated equation
    double psiMin{};            //!< psi = 2*atan(x) for which y = |Ax^2 + Bx + C| / (1+x^2) is minimum
    double psiMax{};            //!< psi = 2*atan(x) for which y = |Ax^2 + Bx + C| / (1+x^2) is maximum
    double yMin{};              //!< minimum value of y = |Ax^2 + Bx + C| / (1+x^2)
    double yMax{};              //!< maximum vaule of y = |Ax^2 + Bx + C| / (1+x^2)
};

void boresightAlignment(double hRefHat[3], double hReqHat[3], double tol, double DB[3][3]);

void finiteDifferencesRatesAndAcc(double sigma_RN[3],
                                  double sigma_RN_1[3],
                                  double sigma_RN_2[3],
                                  uint64_t TNanos,
                                  uint64_t T1Nanos,
                                  uint64_t T2Nanos,
                                  int callCount,
                                  double omega_RN_R[3],
                                  double omegaDot_RN_R[3]);
