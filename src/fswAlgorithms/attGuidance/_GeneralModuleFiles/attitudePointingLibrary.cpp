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

#include "attitudePointingLibrary.h"
#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/rigidBodyKinematics.h"
#ifdef _WIN32
#define _USE_MATH_DEFINES
#endif
#include <math.h>

/*! This constructor initializes an SolutionSpace class for the solution
    of the second order inequality (At^2+Bt+C)/(1+t^2) >= 0 */
SolutionSpace::SolutionSpace(double A, double B, double C, double tol)
{
    double Delta = B*B - 4*A*C;
    // A is nonzero
    if (fabs(A) > tol) {
        if ((Delta > 0) && (fabs(Delta) > tol)) {
            this->emptySet = false;
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
            this->zeros = 2;
        }
        else if ((Delta < 0) && (fabs(Delta) > tol)) {
            if (A > 0) {
                this->emptySet = false;
                this->inf = -M_PI;
                this->sup =  M_PI;
            }
            else {
                this->emptySet = true;
            }
            this->zeros = 0;
        }
        else {
            this->emptySet = false;
            if (A > 0) {
                this->inf = -M_PI;
                this->sup =  M_PI;
            }
            else {
                this->inf = 2 * atan(-B / (2*A));
                this->sup = 2 * atan(-B / (2*A));
            }
            this->zero[0] = 2 * atan(-B / (2*A));
            this->zero[1] = 2 * atan(-B / (2*A));
        }
    }
    // A is zero
    else {
        // B is nonzero
        if (fabs(B) > tol) {
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
            this->zeros = 2;
        }
        // B is zero
        else {
            if (C > 0) {
                this->emptySet = false;
                this->inf = -M_PI;
                this->sup =  M_PI;
            }
            else {
                this->emptySet = true;
            }
            this->zeros = 0;
        }
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
        psi2 = M_PI;
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

/*! Define this method that returns the number of zeros method */
bool SolutionSpace::isEmpty() {return this->emptySet;}

/*! Define this method that returns the number of zeros method */
int SolutionSpace::numberOfZeros() {return this->zeros;}

/*! Define this method that returns the absolute minimum of the function method */
double SolutionSpace::returnAbsMin(int idx)
{
    if ((idx < 1) || (idx > 2)) {
        return 0;
    }

    switch (this->zeros) {

        case 2 :
            return this->zero[idx-1];

        case 1 :
            return this->zero[0];

        case 0 :
            if (fabs(this->yMin) < fabs(this->yMax)) {
                return this->psiMin;
            }
            else {
                return this->psiMax;
            }

        default :
            return 0;
    }
}

/*! Define this method that returns whether the input is contained in the solution space */
bool SolutionSpace::contains(double psi)
{
    if (this->emptySet) {
        return false;
    }
    if (psi < this->inf && psi+2*M_PI > this->sup) {
        return false;
    }
    if (psi > this->sup) {
        return false;
    }
    return true;
}

/*! Define this method the closest value in the solution space to the input */
double SolutionSpace::passThrough(double psi)
{
    if (!this->emptySet) {
        if (psi > this->sup) {
            return this->sup;
        }
        else if (psi < this->inf && psi + 2 * M_PI > this->sup) {
            double dt1 = this->inf - psi;
            double dt2 = psi + 2 * M_PI - this->sup;
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
    if (fabs(phi-M_PI) < tol) {
        phi = M_PI;
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

/*! This function implements second-order finite differences to compute reference angular
    rates and accelerations */
void finiteDifferencesRatesAndAcc(double sigma_RN[3], double sigma_RN_1[3], double sigma_RN_2[3],
                                  uint64_t callTime, uint64_t T1Nanos, uint64_t T2Nanos, int callCount,
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
    if (callCount == 0) {
        v3SetZero(sigmaDot_RN);
        v3SetZero(sigmaDDot_RN);
        // store information for next time step
    }
    // if second update call, derivatives are computed with first order finite differences
    else if (callCount == 1) {
        T1Seconds = (T1Nanos - callTime) * NANO2SEC;
        for (int j = 0; j < 3; j++) {
            sigmaDot_RN[j] = (sigma_RN_1[j] - sigma_RN[j]) / T1Seconds;
        }
        v3SetZero(sigmaDDot_RN);
        // store information for next time step
        T2Nanos = T1Nanos;
        T1Nanos = callTime;
        v3Copy(sigma_RN_1, sigma_RN_2);
        v3Copy(sigma_RN, sigma_RN_1);
    }
    // if third update call or higher, derivatives are computed with second order finite differences
    else {
        T1Seconds = (T1Nanos - callTime) * NANO2SEC;
        T2Seconds = (T2Nanos - callTime) * NANO2SEC;
        for (int j = 0; j < 3; j++) {
            sigmaDot_RN[j] = ((sigma_RN_1[j]*T2Seconds*T2Seconds - sigma_RN_2[j]*T1Seconds*T1Seconds) / (T2Seconds - T1Seconds) - sigma_RN[j] * (T2Seconds + T1Seconds)) / T1Seconds / T2Seconds;
            sigmaDDot_RN[j] = 2 * ((sigma_RN_1[j]*T2Seconds - sigma_RN_2[j]*T1Seconds) / (T1Seconds - T2Seconds) + sigma_RN[j]) / T1Seconds / T2Seconds;
        }
        // store information for next time step
        T2Nanos = T1Nanos;
        T1Nanos = callTime;
        v3Copy(sigma_RN_1, sigma_RN_2);
        v3Copy(sigma_RN, sigma_RN_1);
    }
    callCount += 1;
    dMRP2Omega(sigma_RN, sigmaDot_RN, omega_RN_R);
    ddMRP2dOmega(sigma_RN, sigmaDot_RN, sigmaDDot_RN, omegaDot_RN_R);
}
