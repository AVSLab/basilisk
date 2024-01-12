/*
 ISC License

 Copyright (c) 2021, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#include "constrainedAxisPointingLibrary.h"
#include "architecture/utilities/astroConstants.h"
#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/macroDefinitions.h"
#include "architecture/utilities/rigidBodyKinematics.h"
#ifdef _WIN32
#define _USE_MATH_DEFINES
#endif

/*! This constructor initializes an SolutionSpace class for the solution
    of the second order inequality (At^2+Bt+C)/(1+t^2) >= 0 */
SolutionSpace::SolutionSpace(double A, double B, double C, double tol)
{
    if (fabs(A) < tol) {
        if (fabs(B) < tol) {
            solveZerothOrder(C);
        }
        else {
            solveFirstOrder(B, C);
        }
    }
    else {
        solveSecondOrder(A, B, C);
    }
    // compute max and min of the function
    double psi1;    // PRA #1
    double psi2;    // PRA #2
    double y1;      // fcn value #1
    double y2;      // fcn value #2
    if (fabs(B) > tol) {
        double q = (A - C) / B;
        double t1 = q - sqrt(q*q + 1);
        double t2 = q + sqrt(q*q + 1);
        y1 = (A*t1*t1 + B*t1 + C) / (1+t1*t1);
        y2 = (A*t2*t2 + B*t2 + C) / (1+t2*t2);
        psi1 = 2 * atan(t1);
        psi2 = 2 * atan(t2);
    }
    else {
        psi1 = 0;
        psi2 = MPI;
        y1 = C;
        y2 = A;
    }
    if (y1 < y2) {
        this->psiMin = psi1;
        this->psiMax = psi2;
        this->yMin = y1;
        this->yMax = y2;
    }
    else {
        this->psiMin = psi2;
        this->psiMax = psi1;
        this->yMin = y2;
        this->yMax = y1;
    }
}

/*! Generic destructor */
SolutionSpace::~SolutionSpace() = default;

/*! Solves C / (1+x^2) >= 0 */
void SolutionSpace::solveZerothOrder(double C)
{
    this->zeros = false;
    if (C < 0) {
        this->emptySet = true;
    }
    else {
        this->emptySet = false;
        this->inf = -M_PI;
        this->sup =  M_PI;
    }
}

/*! Solves (Bx + C) / (1+x^2) >= 0 */
void SolutionSpace::solveFirstOrder(double B, double C)
{
    this->zeros = true;
    this->emptySet = false;
    if (B > 0) {
        this->inf = 2 * atan(-C / B);
        this->sup = M_PI;
    }
    else {
        this->inf = -M_PI;
        this->sup = 2 * atan(-C / B);
    }
    this->zero[0] = this->inf;
    this->zero[1] = this->sup;
}

/*! Solves (Ax^2 + Bx + C) / (1+x^2) >= 0 */
void SolutionSpace::solveSecondOrder(double A, double B, double C)
{
    double Delta = B*B - 4*A*C;
    if (Delta >= 0) {
        this->emptySet = false;
        this->zeros = true;
        double t1 = (-B - sqrt(Delta)) / (2*A);
        double t2 = (-B + sqrt(Delta)) / (2*A);
        if (A > 0) {
            this->inf = 2 * atan(t2);
            this->sup = 2 * (atan(t1) + M_PI);
        }
        else {
            this->inf = 2 * atan(t2);
            this->sup = 2 * atan(t1);
        }
        this->zero[0] = this->inf;
        this->zero[1] = this->sup;
    }
    else {
        this->zeros = false;
        if (A > 0) {
            this->emptySet = false;
            this->inf = -M_PI;
            this->sup =  M_PI;
        }
        else {
            this->emptySet = true;
        }
    }
}

/*! Define this method that returns the number of zeros method */
bool SolutionSpace::isEmpty() const {return this->emptySet;}

/*! Define this method that returns the zeros bool class variable */
bool SolutionSpace::hasZeros() const {return this->zeros;}

/*! Define this method that returns the absolute minimum of the function method */
double SolutionSpace::returnAbsMin(int idx) const
{
    if (this->zeros) {
        if (idx < 2) {
            return this->zero[0];
        } else {
            return this->zero[1];
        }
    }
    else {
        if (fabs(this->yMin) < fabs(this->yMax)) {
            return this->psiMin;
        } else {
            return this->psiMax;
        }
    }
}

/*! Define this method that returns whether the input is contained in the solution space */
bool SolutionSpace::contains(double psi) const
{
    if (this->emptySet) {
        return false;
    }
    if (psi < this->inf && psi+2*MPI > this->sup) {
        return false;
    }
    if (psi > this->sup) {
        return false;
    }
    return true;
}

/*! Define this method the closest value in the solution space to the input */
double SolutionSpace::passThrough(double psi) const
{
    if (!this->emptySet) {
        if (psi > this->sup) {
            return this->sup;
        }
        else if (psi < this->inf && psi + 2 * MPI > this->sup) {
            double dt1 = this->inf - psi;
            double dt2 = psi + 2 * MPI - this->sup;
            if (dt1 < dt2) {
                return this->inf;
            } else {
                return this->sup;
            }
        }
        else {
            return psi;
        }
    }
    else {
        return this->returnAbsMin(1);
    }
}

/*! This function computes the DCM DB that aligns direction hRefHat to a requested direction hReqHat */
void boresightAlignment(double hRefHat[3], double hReqHat[3], double tol, double DB[3][3])
{
    /*! compute principal rotation angle (phi) and vector (e_phi) */
    double phi = acos( fmin( fmax( v3Dot(hRefHat, hReqHat), -1 ), 1 ) );
    double e_phi[3];
    v3Cross(hRefHat, hReqHat, e_phi);
    // If phi = PI, e_phi can be any vector perpendicular to hRefHat_B
    if (fabs(phi-MPI) < tol) {
        phi = MPI;
        v3Perpendicular(hRefHat, e_phi);
    }
    else if (fabs(phi) < tol) {
        phi = 0;
    }
    // normalize e_phi
    v3Normalize(e_phi, e_phi);

    /*! define first rotation R1B */
    double PRV_phi[3];
    v3Scale(phi, e_phi, PRV_phi);
    PRV2C(PRV_phi, DB);
}

void computeReferenceFrame(double hRefHat_B[3], double hReqHat_N[3], double rHat_SB_B[3],
                           double a1Hat_B[3], double a2Hat_B[3], double beta, double BN[3][3],
                           AlignmentPriority alignmentPriority, double epsilon, double RN[3][3])
{
    /*! map requested heading into current B frame */
    double hReqHat_B[3];
    m33MultV3(BN, hReqHat_N, hReqHat_B);

    /*! compute intermediate rotation DB to align the boresight */
    double DB[3][3];
    boresightAlignment(hRefHat_B, hReqHat_B, epsilon, DB);

    /*! map Sun direction vector to intermediate frame */
    double rHat_SB_D[3];
    m33MultV3(DB, rHat_SB_B, rHat_SB_D);

    /*! define the coefficients of the quadratic equation A, B, and C for the solar array drive axis */
    double e_psi[3];
    v3Copy(hRefHat_B, e_psi);
    double b3[3];
    v3Cross(rHat_SB_D, e_psi, b3);
    double A = 2 * v3Dot(rHat_SB_D, e_psi) * v3Dot(e_psi, a1Hat_B) - v3Dot(a1Hat_B, rHat_SB_D);
    double B = 2 * v3Dot(a1Hat_B, b3);
    double C = v3Dot(a1Hat_B, rHat_SB_D);

    /*! define the coefficients of the quadratic equation D, E, and F for the Sun-constrained axis */
    double D = 2 * v3Dot(rHat_SB_D, e_psi) * v3Dot(e_psi, a2Hat_B) - v3Dot(a2Hat_B, rHat_SB_D);
    double E = 2 * v3Dot(a2Hat_B, b3);
    double F = v3Dot(a2Hat_B, rHat_SB_D);

    /*! compute the solution(s) to the optimized solar array alignment problem */
    SolutionSpace solarArraySolutions(A, B, C, epsilon);

    double PRV_psi[3];
    switch (alignmentPriority) {

        case AlignmentPriority::SolarArrayAlign :
            if (solarArraySolutions.hasZeros()) {
                double psi1 = solarArraySolutions.returnAbsMin(1);
                double psi2 = solarArraySolutions.returnAbsMin(2);
                double PRV_psi1[3];
                v3Scale(psi1, e_psi, PRV_psi1);
                double PRV_psi2[3];
                v3Scale(psi2, e_psi, PRV_psi2);
                double P1D[3][3];
                PRV2C(PRV_psi1, P1D);
                double P2D[3][3];
                PRV2C(PRV_psi2, P2D);
                double rHat_SB_P1[3];
                m33MultV3(P1D, rHat_SB_D, rHat_SB_P1);
                double rHat_SB_P2[3];
                m33MultV3(P2D, rHat_SB_D, rHat_SB_P2);
                if (v3Dot(a2Hat_B, rHat_SB_P2) - v3Dot(a2Hat_B, rHat_SB_P1) > epsilon) {
                    v3Scale(psi2, e_psi, PRV_psi);
                }
                else {
                    v3Scale(psi1, e_psi, PRV_psi);
                }
            }
            else {
                double psi = solarArraySolutions.returnAbsMin(1);
                v3Scale(psi, e_psi, PRV_psi);
            }
            break;

        case AlignmentPriority::SunConstrAxisAlign :
            double k = cos(beta);
            SolutionSpace sunConstAxisSolutions(D - k, E, F - k, epsilon);
            if (bool empty = sunConstAxisSolutions.isEmpty(); empty) {
                double psi = sunConstAxisSolutions.returnAbsMin(1);
                v3Scale(psi, e_psi, PRV_psi);
            }
            else {
                if (solarArraySolutions.hasZeros()) {
                    double psi1 = solarArraySolutions.returnAbsMin(1);
                    double psi2 = solarArraySolutions.returnAbsMin(2);
                    double deltaPsi1 = psi1 - sunConstAxisSolutions.passThrough(psi1);
                    double deltaPsi2 = psi2 - sunConstAxisSolutions.passThrough(psi2);
                    if (fabs(deltaPsi2) - fabs(deltaPsi1) > epsilon) {
                        v3Scale(psi1, e_psi, PRV_psi);
                    }
                    else {
                        v3Scale(psi2, e_psi, PRV_psi);
                    }
                }
                else {
                    double psi = solarArraySolutions.returnAbsMin(1);
                    psi = sunConstAxisSolutions.passThrough(psi);
                    v3Scale(psi, e_psi, PRV_psi);
                }
            }
            break;
    }

    /*! map PRV to RD direction cosine matrix */
    double RD[3][3];
    PRV2C(PRV_psi, RD);
    double RB[3][3];
    m33MultM33(RD, DB, RB);
    m33MultM33(RB, BN, RN);
}



/*! This function implements second-order finite differences to compute reference angular
    rates and accelerations */
void finiteDifferencesRatesAndAcc(double sigma_RN[3], double sigma_RN_1[3], double sigma_RN_2[3],
                                  uint64_t *TNanos, uint64_t *T1Nanos, uint64_t *T2Nanos, int *callCount,
                                  double omega_RN_R[3], double omegaDot_RN_R[3])
{
    // switch sigma_RN_1 and sigma_RN_2 if needed
    double delSigma[3];
    v3Subtract(sigma_RN, sigma_RN_1, delSigma);
    if (v3Norm(delSigma) > 1) {
        MRPshadow(sigma_RN_1, sigma_RN_1);
    }
    v3Subtract(sigma_RN_1, sigma_RN_2, delSigma);
    if (v3Norm(delSigma) > 1) {
        MRPshadow(sigma_RN_2, sigma_RN_2);
    }
    // if first update call, derivatives are set to zero
    double T1Seconds;
    double T2Seconds;
    double sigmaDot_RN[3];
    double sigmaDDot_RN[3];
    if (*callCount == 0) {
        v3SetZero(sigmaDot_RN);
        v3SetZero(sigmaDDot_RN);
        // store information for next time step
    }
    // if second update call, derivatives are computed with first order finite differences
    else if (*callCount == 1) {
        T1Seconds = double (*T1Nanos - *TNanos) * NANO2SEC;
        for (int j = 0; j < 3; j++) {
            sigmaDot_RN[j] = (sigma_RN_1[j] - sigma_RN[j]) / T1Seconds;
        }
        v3SetZero(sigmaDDot_RN);
        // store information for next time step
        *T2Nanos = *T1Nanos;
        *T1Nanos = *TNanos;
        v3Copy(sigma_RN_1, sigma_RN_2);
        v3Copy(sigma_RN, sigma_RN_1);
    }
    // if third update call or higher, derivatives are computed with second order finite differences
    else {
        T1Seconds = double (*T1Nanos - *TNanos) * NANO2SEC;
        T2Seconds = double (*T2Nanos - *TNanos) * NANO2SEC;
        for (int j = 0; j < 3; j++) {
            sigmaDot_RN[j] = ((sigma_RN_1[j]*T2Seconds*T2Seconds - sigma_RN_2[j]*T1Seconds*T1Seconds) / (T2Seconds - T1Seconds) - sigma_RN[j] * (T2Seconds + T1Seconds)) / T1Seconds / T2Seconds;
            sigmaDDot_RN[j] = 2 * ((sigma_RN_1[j]*T2Seconds - sigma_RN_2[j]*T1Seconds) / (T1Seconds - T2Seconds) + sigma_RN[j]) / T1Seconds / T2Seconds;
        }
        // store information for next time step
        *T2Nanos = *T1Nanos;
        *T1Nanos = *TNanos;
        v3Copy(sigma_RN_1, sigma_RN_2);
        v3Copy(sigma_RN, sigma_RN_1);
    }
    *callCount += 1;
    dMRP2Omega(sigma_RN, sigmaDot_RN, omega_RN_R);
    ddMRP2dOmega(sigma_RN, sigmaDot_RN, sigmaDDot_RN, omegaDot_RN_R);
}
