/*
 ISC License

 Copyright (c) 2023, Autonomous Vehicle Systems Lab, University of Colorado Boulder

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

#include "fswAlgorithms/orbitControl/lambertSolver/lambertSolver.h"
#include "architecture/utilities/linearAlgebra.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace {
constexpr double rootTimeTolerance = 1e-12; // [-] Maximum relative flight-time residual.
}

/*! This is the constructor for the module class.  It sets default variable
    values and initializes the various parts of the model */
LambertSolver::LambertSolver() = default;

/*! Module Destructor */
LambertSolver::~LambertSolver() = default;

/*! This method is used to reset the module and checks that required input messages are connected.
    @param currentSimNanos current simulation time in nano-seconds

*/
void LambertSolver::Reset(uint64_t currentSimNanos [[maybe_unused]])
{
    // check that required input messages are connected
    if (!this->lambertProblemInMsg.isLinked()) {
        bskLogger.bskError("lambertSolver.lambertProblemInMsg was not linked.");
    }
}

/*! This is the main method that gets called every time the module is updated.
    It computes the solution of Lambert's problem.
    @param currentSimNanos current simulation time in nano-seconds

*/
void LambertSolver::UpdateState(uint64_t currentSimNanos)
{
    // read messages
    this->readMessages();
    // compute geometry of given Lambert problem
    this->problemGeometry();
    // find free variable x that satisfies the given time of flight
    this->findx();

    // Only reconstruct velocities for converged roots. A finite root alone does not
    // guarantee finite velocities when the geometry is numerically unresolvable.
    if (this->solution.converged) {
        this->vvecs = this->computeVelocities(this->solution.x);
        this->solution.converged = this->vvecs[0].allFinite() && this->vvecs[1].allFinite();
        if (!this->solution.converged) {
            bskLogger.bskLog(BSK_WARNING, "lambertSolver: first solution has nonfinite velocities.");
        }
    }
    if (this->solutionSol2.converged) {
        this->vvecsSol2 = this->computeVelocities(this->solutionSol2.x);
        this->solutionSol2.converged = this->vvecsSol2[0].allFinite() && this->vvecsSol2[1].allFinite();
        if (!this->solutionSol2.converged) {
            bskLogger.bskLog(BSK_WARNING, "lambertSolver: second solution has nonfinite velocities.");
        }
    }

    // write messages
    this->writeMessages(currentSimNanos);
}

/*! This method reads the input messages each call of updateState. It also checks if the message contents are valid for
    this module.

*/
void LambertSolver::readMessages()
{
    LambertProblemMsgPayload lambertProblemInMsgBuffer = this->lambertProblemInMsg();

    // check if input parameters are valid
    if (lambertProblemInMsgBuffer.mu <= 0.0) {
        bskLogger.bskError("lambertSolver: mu must be positive.");
    } else {
        this->mu = lambertProblemInMsgBuffer.mu;
    }
    if (lambertProblemInMsgBuffer.transferTime <= 0.0) {
        bskLogger.bskError("lambertSolver: transferTime must be positive.");
    } else {
        this->transferTime = lambertProblemInMsgBuffer.transferTime;
    }
    if (lambertProblemInMsgBuffer.numRevolutions < 0) {
        bskLogger.bskError("lambertSolver: numberOfRevolutions must be zero or positive.");
    } else {
        this->numberOfRevolutions = lambertProblemInMsgBuffer.numRevolutions;
    }
    this->solverMethod = lambertProblemInMsgBuffer.solverMethod;
    if (lambertProblemInMsgBuffer.transferDirection != PROGRADE &&
        lambertProblemInMsgBuffer.transferDirection != RETROGRADE) {
        bskLogger.bskError("lambertSolver: transferDirection must be PROGRADE or RETROGRADE.");
    }
    this->transferDirection = static_cast<LambertTransferDirection>(lambertProblemInMsgBuffer.transferDirection);
    this->r1_N = cArray2EigenVector3d(lambertProblemInMsgBuffer.r1_N);
    this->r2_N = cArray2EigenVector3d(lambertProblemInMsgBuffer.r2_N);
}

/*! This method writes the output messages each call of updateState
    @param currentSimNanos current simulation time in nano-seconds

*/
void LambertSolver::writeMessages(uint64_t currentSimNanos)
{
    // Make local copies of messages
    LambertSolutionMsgPayload lambertSolutionOutMsgBuffer;
    LambertPerformanceMsgPayload lambertPerformanceOutMsgBuffer;
    // Always zero the output message buffers before assigning values
    lambertSolutionOutMsgBuffer = this->lambertSolutionOutMsg.zeroMsgPayload;
    lambertPerformanceOutMsgBuffer = this->lambertPerformanceOutMsg.zeroMsgPayload;

    // A failed branch retains its zero payload and validity flag independently of
    // the other branch. Never expose a stale root or an unconverged velocity.
    if (this->solution.converged) {
        eigenVector3d2CArray(this->vvecs[0], lambertSolutionOutMsgBuffer.v1_N);
        eigenVector3d2CArray(this->vvecs[1], lambertSolutionOutMsgBuffer.v2_N);
        lambertSolutionOutMsgBuffer.valid = 1;
        lambertPerformanceOutMsgBuffer.x = this->solution.x;
        lambertPerformanceOutMsgBuffer.numIter = this->solution.iterations;
        lambertPerformanceOutMsgBuffer.errX = this->solution.error;
    }
    if (this->solutionSol2.converged) {
        eigenVector3d2CArray(this->vvecsSol2[0], lambertSolutionOutMsgBuffer.v1Sol2_N);
        eigenVector3d2CArray(this->vvecsSol2[1], lambertSolutionOutMsgBuffer.v2Sol2_N);
        lambertSolutionOutMsgBuffer.validSol2 = 1;
        lambertPerformanceOutMsgBuffer.xSol2 = this->solutionSol2.x;
        lambertPerformanceOutMsgBuffer.numIterSol2 = this->solutionSol2.iterations;
        lambertPerformanceOutMsgBuffer.errXSol2 = this->solutionSol2.error;
    }

    // Write to the output messages
    this->lambertSolutionOutMsg.write(&lambertSolutionOutMsgBuffer, this->moduleID, currentSimNanos);
    this->lambertPerformanceOutMsg.write(&lambertPerformanceOutMsgBuffer, this->moduleID, currentSimNanos);
}

/*! This method computes the problem geometry for the given parameters of Lambert's problem.
    The orbit frame is also determined.

*/
void LambertSolver::problemGeometry()
{
    double c = (this->r2_N - this->r1_N).norm(); // chord length
    double r1 = this->r1_N.norm();
    double r2 = this->r2_N.norm();
    double s = 1.0 / 2.0 * (r1 + r2 + c); // semiperimeter

    // lambda parameter
    this->lambda = sqrt(1.0 - c / s);
    // non-dimensional time-of-flight
    this->TOF = sqrt(2.0 * this->mu / pow(s, 3)) * this->transferTime;

    // compute orbit frame unit vectors

    // radial direction at for initial and final position vector
    Eigen::Vector3d i_r1_N = this->r1_N / r1;
    Eigen::Vector3d i_r2_N = this->r2_N / r2;
    // check alignment of the two position vectors
    Eigen::Vector3d radialCross = i_r1_N.cross(i_r2_N);
    double sinTheta = radialCross.norm();
    // Alignment rejects only the current input, not subsequent module updates.
    this->noSolution = abs(sinTheta) < sin(this->alignmentThreshold * M_PI / 180.);
    if (this->noSolution) {
        bskLogger.bskLog(BSK_WARNING,
                         "lambertSolver: position vectors r1 and r2 are too aligned, that is, the angle between "
                         "them is smaller than the angle alignmentThreshold (default: 1.0 degrees). "
                         "They might not define a plane, so no solution is returned.");
        return;
    }
    // orbit normal direction
    Eigen::Vector3d i_h_N = radialCross / sinTheta;

    // Orient the orbit normal to match the requested direction. Reversing it selects
    // the complementary transfer angle, which also reverses the sign of lambda.
    // Treat cancellation-level roundoff in the z-component as a polar tie: PROGRADE
    // retains r1 cross r2 and RETROGRADE reverses it. Scale the tolerance by the terms
    // being subtracted so resolvable near-polar signs are preserved.
    const double crossZScale = std::abs(i_r1_N(0) * i_r2_N(1)) + std::abs(i_r1_N(1) * i_r2_N(0));
    const double polarTolerance = 8.0 * std::numeric_limits<double>::epsilon() * crossZScale;
    if ((radialCross(2) < -polarTolerance) != (this->transferDirection == RETROGRADE)) {
        this->lambda = -this->lambda;
        i_h_N = -i_h_N;
    }
    // second unit vectors
    Eigen::Vector3d i_t1_N = i_h_N.cross(i_r1_N);
    Eigen::Vector3d i_t2_N = i_h_N.cross(i_r2_N);

    this->Oframe1 = { i_r1_N, i_t1_N, i_h_N };
    this->Oframe2 = { i_r2_N, i_t2_N, i_h_N };
}

/*! @brief Find the free variable on each applicable branch of the time-of-flight curve.
 */
void LambertSolver::findx()
{
    this->solution = {};
    this->solutionSol2 = {};
    if (this->noSolution) {
        return;
    }
    if (!std::isfinite(this->TOF) || this->TOF <= 0.0 || !std::isfinite(this->lambda) ||
        std::abs(this->lambda) >= 1.0) {
        bskLogger.bskLog(BSK_WARNING, "lambertSolver: root search requires finite time and geometry.");
        return;
    }

    std::array<double, 2> x0;
    if (this->solverMethod == GOODING) {
        x0 = this->goodingInitialGuess(this->lambda, this->TOF);
    } else if (this->solverMethod == IZZO) {
        x0 = this->izzoInitialGuess(this->lambda, this->TOF);
    } else {
        return;
    }
    if (this->numberOfRevolutions > 0) {
        if (!this->multiRevSolution) {
            return;
        }
        if (x0[0] == x0[1]) {
            // The minimum-time search already checks convergence at the coalesced root.
            this->solution = { x0[0], 0, 0.0, true };
            this->solutionSol2 = this->solution;
            return;
        }
        this->solution = this->solveRoot(this->TOF, x0[0], this->numberOfRevolutions, -1.0, this->xMin, false);
        this->solutionSol2 = this->solveRoot(this->TOF, x0[1], this->numberOfRevolutions, this->xMin, 1.0, true);
    } else {
        // Preserve the analytic parabolic solution when it meets the same residual
        // tolerance as an iterated root, including roundoff in the input geometry.
        const double parabolicTime = this->x2tof(1.0, 0, this->lambda);
        if (std::abs(this->TOF - parabolicTime) <= rootTimeTolerance * this->TOF) {
            this->solution = { 1.0, 0, 0.0, true };
            return;
        }
        // T(x) decreases from infinity at x = -1 to zero as x tends to infinity.
        // Expand the finite upper bound for hyperbolic transfers; never evaluate -1.
        double xUpper = std::isfinite(x0[0]) ? std::max(1.0, x0[0]) : 1.0;
        const int expansionLimit = 64;
        bool bracketed = false;
        for (int j = 0; j < expansionLimit; ++j) {
            const double upperTime = this->x2tof(xUpper, 0, this->lambda);
            if (!std::isfinite(upperTime) || upperTime <= 0.0) {
                break;
            }
            if (upperTime <= this->TOF) {
                bracketed = true;
                break;
            }
            xUpper *= 2.0;
        }
        if (!bracketed) {
            bskLogger.bskLog(BSK_WARNING, "lambertSolver: could not bracket the zero-revolution root.");
            return;
        }
        this->solution = this->solveRoot(this->TOF, x0[0], 0, -1.0, xUpper, false);
    }
}

/*! This method computes the velocities at the initial and final position for a given free variable x
    @param x free variable of Lambert's problem that satisfies the given time of flight
    @return std::array<Eigen::Vector3d, 2>
*/
std::array<Eigen::Vector3d, 2> LambertSolver::computeVelocities(double x)
{
    double c = (this->r2_N - this->r1_N).norm(); // chord length
    double r1 = this->r1_N.norm();
    double r2 = this->r2_N.norm();
    double s = 1.0 / 2.0 * (r1 + r2 + c); // semiperimeter

    double y = sqrt(1.0 - pow(this->lambda, 2) * (1.0 - pow(x, 2)));
    double gamma = sqrt(this->mu * s / 2);
    double rho = (r1 - r2) / c;
    double sigma = sqrt(1.0 - pow(rho, 2));

    // compute velocity components for v1_N and v2_N
    double Vr1 = gamma * ((this->lambda * y - x) - rho * (this->lambda * y + x)) / r1;
    double Vr2 = -gamma * ((this->lambda * y - x) + rho * (this->lambda * y + x)) / r2;
    double Vt1 = gamma * sigma * (y + this->lambda * x) / r1;
    double Vt2 = gamma * sigma * (y + this->lambda * x) / r2;

    // get orbit frame unit vectors
    Eigen::Vector3d i_r1_N = this->Oframe1.at(0);
    Eigen::Vector3d i_t1_N = this->Oframe1.at(1);
    Eigen::Vector3d i_r2_N = this->Oframe2.at(0);
    Eigen::Vector3d i_t2_N = this->Oframe2.at(1);

    // initial and final velocity of Lambert solution orbit
    Eigen::Vector3d v1_N = Vr1 * i_r1_N + Vt1 * i_t1_N;
    Eigen::Vector3d v2_N = Vr2 * i_r2_N + Vt2 * i_t2_N;

    std::array<Eigen::Vector3d, 2> velocities = { v1_N, v2_N };
    return velocities;
}

/*! This method computes the initial guess for the free variable x using the Gooding procedure
    @param lam lambda parameter that defines the problem geometry
    @param T non-dimensional time-of-flight
    @return std::array<double, 2>
*/
std::array<double, 2> LambertSolver::goodingInitialGuess(double lam, double T)
{
    // T for x=0 and zero revolution
    double T00 = acos(lam) + lam * sqrt(1.0 - pow(lam, 2));
    // T for x=0 and M revolutions
    double T0M = T00 + this->numberOfRevolutions * M_PI;

    double x0Sol1 = 0.0; // initial guess for solution 1
    double x0Sol2 = 0.0; // initial guess for solution 2 (multi-revolutions only)

    double c0 = 1.7;
    double c1 = 0.5;
    double c2 = 0.03;
    double c3 = 0.15;
    double c41 = 1.0;
    double c42 = 0.24;

    double theta = 2.0 * atan2(1.0 - pow(lam, 2), 2.0 * lam);
    // atan2 already places theta above pi for negative lambda. Do not reflect it.

    // initial guess procedure for Gooding algorithm. Involves several patches of solutions
    if (this->numberOfRevolutions == 0) {
        // zero revolution case
        double x01 = -(T - T00) / (T - T00 + 4.0);
        // solution patches
        double W = x01 + 1.7 * sqrt(2.0 - theta / M_PI);
        double x03;
        if (W >= 0.0) {
            x03 = x01;
        } else {
            double x02 = -sqrt((T - T00) / (T + 1.0 / 2.0 * T00));
            double w = pow(-W, 1.0 / 16.0);
            x03 = x01 + w * (x02 - x01);
        }
        double scale = 1.0 + c1 * x03 * (1.0 + x01) - c2 * pow(x03, 2) * sqrt(1.0 + x01);

        x0Sol1 = scale * x03;
    } else {
        // Use the converged minimum and its location for both starters.
        const auto minimum = this->getTmin(T0M, this->numberOfRevolutions);
        if (!minimum) {
            this->multiRevSolution = false;
            return { 0.0, 0.0 };
        }
        const double x_M = (*minimum)[0];
        this->xMin = x_M;
        const double Tmin = (*minimum)[1];
        const double minTimeTolerance = 8.0 * std::numeric_limits<double>::epsilon() * Tmin;
        if (T < Tmin - minTimeTolerance) {
            this->multiRevSolution = false;
            bskLogger.bskLog(BSK_WARNING,
                             "lambertSolver: no multi-revolution solution exists for the given time of flight.");
            return { 0.0, 0.0 };
        }
        this->multiRevSolution = true;
        if (abs(T - Tmin) <= minTimeTolerance) {
            return { x_M, x_M };
        }

        // get derivatives of T at x_M
        std::array<double, 3> DTs = dTdx(x_M, Tmin, this->lambda);
        double D2T = DTs[1];

        double TdiffM = T - Tmin;
        double Tdiff = T - T0M;

        // Basilisk's non-dimensional time is half Gooding's XLAMB time, so the
        // bilinear starter and its weighting use 2 rather than XLAMB's 4.
        if (T <= T0M) {
            double term1 = TdiffM / (0.5 * D2T - TdiffM * (0.5 * D2T / (T0M - Tmin) - 1 / pow(x_M, 2)));
            x0Sol1 = x_M - sqrt(term1);
        } else {
            double x01_1 = -Tdiff / (Tdiff + 2.0);
            double W = x01_1 + c0 * sqrt(2.0 * (1.0 - theta / (2 * M_PI)));
            if (W < 0.0) {
                x01_1 -= pow(-W, 1.0 / 16.0) * (x01_1 + sqrt(Tdiff / (Tdiff + 1.5 * T0M)));
            }
            // Apply the correction after either bilinear starter patch.
            double W2 = 2.0 / (2.0 + Tdiff);
            x0Sol1 =
                x01_1 * (1.0 + (1.0 + this->numberOfRevolutions + c42 * (theta / (2 * M_PI) - 0.5)) /
                                   (1.0 + c3 * this->numberOfRevolutions) * x01_1 * (c1 * W2 - c2 * x01_1 * sqrt(W2)));
        }

        // The second starter is a positive displacement from the minimum.
        double term2 = TdiffM / (0.5 * D2T + TdiffM / pow(1 - x_M, 2));
        double x01_2 = sqrt(term2);
        double W = x_M + x01_2;
        W = W * 2.0 / (2.0 + TdiffM) + pow(1.0 - W, 2);
        x0Sol2 = x01_2 * (1.0 - (1.0 + this->numberOfRevolutions + c41 * (theta / (2 * M_PI) - 0.5)) /
                                    (1.0 + c3 * this->numberOfRevolutions) * x01_2 * (c1 * W + c2 * x01_2 * sqrt(W))) +
                 x_M;
    }

    std::array<double, 2> x0 = { x0Sol1, x0Sol2 };
    return x0;
}

/*! This method computes the initial guess for the free variable x using the Izzo procedure
    @param lam lambda parameter that defines the problem geometry
    @param T non-dimensional time-of-flight
    @return std::array<double, 2>
*/
std::array<double, 2> LambertSolver::izzoInitialGuess(double lam, double T)
{
    // T for x=0 and zero revolution
    double T00 = acos(lam) + lam * sqrt(1.0 - pow(lam, 2));
    // T forx=1 (parabolic case)
    double T1 = 2.0 / 3.0 * (1.0 - pow(lam, 3));
    // T for x=0 and M revolutions
    double T0M = T00 + this->numberOfRevolutions * M_PI;

    double x0Sol1 = 0.0; // initial guess for solution 1
    double x0Sol2 = 0.0; // initial guess for solution 2 (multi-revolutions only)

    // initial guess procedure for Izzo algorithm
    if (this->numberOfRevolutions == 0) {
        // zero revolution case
        if (T >= T00) {
            x0Sol1 = pow(T00 / T, 2.0 / 3.0) - 1.0;
        } else if (T <= T1) {
            x0Sol1 = 5.0 / 2.0 * T1 * (T1 - T) / (T * (1.0 - pow(lam, 5))) + 1.0;
        } else {
            // Interpolate log(1+x) from 0 at T00 to log(2) at T1.
            x0Sol1 = pow(T00 / T, log(2.0) / log(T00 / T1)) - 1.0;
        }
    } else {
        // multi-revolution case
        const auto minimum = this->getTmin(T0M, this->numberOfRevolutions);
        if (!minimum) {
            this->multiRevSolution = false;
            return { 0.0, 0.0 };
        }
        this->xMin = (*minimum)[0];
        const double Tmin = (*minimum)[1];
        const double minTimeTolerance = 8.0 * std::numeric_limits<double>::epsilon() * Tmin;
        if (T < Tmin - minTimeTolerance) {
            // if T < Tmin, no multi-revolution solution exists for the given time of flight T
            this->multiRevSolution = false;
            bskLogger.bskLog(BSK_WARNING,
                             "lambertSolver: no multi-revolution solution exists for the given time of flight.");
            std::array<double, 2> x0 = { 0.0, 0.0 };
            return x0;
        }
        // if T > Tmin, two multi-revolution solutions exist for given time of flight T (or one solution if T = Tmin)
        this->multiRevSolution = true;

        if (abs(T - Tmin) <= minTimeTolerance) {
            return { (*minimum)[0], (*minimum)[0] };
        }

        double term1 = pow((this->numberOfRevolutions * M_PI + M_PI) / (8.0 * T), (2.0 / 3.0));
        double term2 = pow((8.0 * T) / (this->numberOfRevolutions * M_PI), (2.0 / 3.0));
        x0Sol1 = (term1 - 1.0) / (term1 + 1.0);
        x0Sol2 = (term2 - 1.0) / (term2 + 1.0);
    }

    std::array<double, 2> x0 = { x0Sol1, x0Sol2 };
    return x0;
}

/*! This method computes the non-dimensional time of flight (TOF) for a given x
    @param x free variable of Lambert's problem that satisfies the given time of flight
    @param N number of revolutions
    @param lam lambda parameter that defines the problem geometry
    @return double
*/
double LambertSolver::x2tof(double x, int N, double lam)
{
    // if x is close to 1.0 (parabolic case), time-of-flight is computed according to Battin
    double battin = 0.01;
    double dist = abs(x - 1.0);
    double u = 1.0 - pow(x, 2);
    double y = sqrt(1.0 - pow(lam, 2) * u);

    double tof;
    if (x == 1.0 && N == 0) {
        // exact time of flight for parabolic case

        tof = 2.0 / 3.0 * (1.0 - pow(lam, 3));
    } else if (dist < battin) {
        // Use the smooth Battin formulation even arbitrarily close to x = 1.
        // A constant parabolic interval would conceal the true flight-time residual.

        double eta = y - lam * x;
        double S1 = 0.5 * (1.0 - lam - x * eta);
        double Q = 4.0 / 3.0 * hypergeometricF(S1);
        tof = (pow(eta, 3) * Q + 4.0 * lam * eta) / 2.0 + N * M_PI / (pow(abs(u), 1.5));
    } else {
        // Use Lancaster formulation

        double f = (y - lam * x) * sqrt(abs(u));
        double g = x * y + lam * u;
        double d;
        if (u > 0.0) {
            // elliptic case
            double psi = atan2(f, g);
            d = psi + N * M_PI;
        } else {
            // hyperbolic case
            d = atanh(f / g);
        }

        // non-dimensional time-of-flight
        tof = (d / sqrt(abs(u)) - x + lam * y) / u;
    }

    return tof;
}

/*! This method computes the derivatives of the time of flight curve T(x)
    @param x free variable of Lambert's problem that satisfies the given time of flight
    @param T requested non-dimensional time-of-flight
    @param lam lambda parameter that defines the problem geometry
    @return std::array<double, 3>
*/
std::array<double, 3> LambertSolver::dTdx(double x, double T, double lam)
{
    double DT;  // dT/dx
    double D2T; // d2T/dx2
    double D3T; // d3T/dx3
    if (abs(x - 1.0) < 1e-8) {
        // exact derivative for parabolic case
        DT = 2.0 / 5.0 * (pow(lam, 5) - 1.0);
        D2T = 0.0;
        D3T = 0.0;
    } else {
        double u = 1.0 - pow(x, 2);
        double y = sqrt(1.0 - pow(lam, 2) * u);
        DT = 1.0 / u * (3.0 * T * x - 2.0 + 2.0 * pow(lam, 3) * x / y);
        D2T = 1.0 / u * (3.0 * T + 5.0 * x * DT + 2.0 * (1. - pow(lam, 2)) * pow(lam, 3) / pow(y, 3));
        D3T = 1.0 / u * (7.0 * x * D2T + 8.0 * DT - 6.0 * (1. - pow(lam, 2)) * pow(lam, 5) * x / pow(y, 5));
    }

    std::array<double, 3> DTs = { DT, D2T, D3T };

    return DTs;
}

/*! @brief Solve one monotonic flight-time branch with safeguarded Halley or Householder steps.
    @param T Requested positive non-dimensional flight time.
    @param x0 Initial root estimate; an out-of-bracket estimate is replaced by the midpoint.
    @param N Number of complete revolutions.
    @param xLower Lower bound of the root's branch, including the open elliptic endpoint -1.
    @param xUpper Upper bound, including the open endpoint 1 for a multi-revolution branch.
    @param increasing True for the right multi-revolution branch; false for decreasing branches.
    @return Last root estimate, iteration count, step or correction magnitude, and explicit convergence status.
    @note Bisection replaces nonfinite, out-of-bracket, or insufficiently shrinking high-order steps.
          The shrink check compares consecutive high-order steps; bisection clears that history.
          Success requires both a small step or correction and a small relative flight-time residual,
          or an exact finite flight-time match. Numerical failure never constitutes convergence.
*/
LambertSolver::RootResult LambertSolver::solveRoot(double T,
                                                   double x0,
                                                   int N,
                                                   double xLower,
                                                   double xUpper,
                                                   bool increasing)
{
    const double xTolerance = 1e-8; // [-] Maximum final step or correction in the root variable.
    const int iterationLimit = 80;
    RootResult result{};
    result.error = std::numeric_limits<double>::infinity();
    std::optional<double> previousHighOrderStep; // [-] Empty initially and after bisection.
    // A finite zero-revolution upper endpoint is admissible and can be the starter
    // itself. Multi-revolution endpoints remain open to exclude the singularity.
    if (!std::isfinite(x0) || x0 <= xLower || x0 > xUpper || (N > 0 && x0 == xUpper)) {
        x0 = xLower + 0.5 * (xUpper - xLower);
    }

    for (int j = 0; j <= iterationLimit; ++j) {
        result.x = x0;
        result.iterations = j;
        const double tof = this->x2tof(x0, N, this->lambda);
        if (!std::isfinite(tof) || tof <= 0.0 || x0 <= -1.0 || (N > 0 && x0 >= 1.0)) {
            break;
        }
        const double residual = tof - T;
        // An exact match has a zero correction even if the preceding step was large.
        if (residual == 0.0) {
            result.error = 0.0;
        }
        if (result.error <= xTolerance && std::abs(residual) <= rootTimeTolerance * T) {
            result.converged = true;
            return result;
        }
        if (j == iterationLimit) {
            break;
        }

        if ((residual < 0.0) == increasing) {
            xLower = x0;
        } else {
            xUpper = x0;
        }
        double xNew = xLower + 0.5 * (xUpper - xLower);
        bool usedHighOrderStep = false;
        const auto derivatives = this->dTdx(x0, tof, this->lambda);
        const double first = derivatives[0];
        const double second = derivatives[1];
        const double third = derivatives[2];
        const double numerator =
            this->solverMethod == GOODING ? residual * first : residual * (first * first - residual * second / 2.0);
        const double denominator = this->solverMethod == GOODING ? first * first - residual * second / 2.0
                                                                 : first * (first * first - residual * second) +
                                                                       third * residual * residual / 6.0;
        if (std::isfinite(numerator) && std::isfinite(denominator) && denominator != 0.0) {
            const double correction = numerator / denominator;
            // The residual can pass immediately after a large step. Accept a small
            // correction before the strict bracket check rejects candidate == x0
            // (the updated endpoint) and sends an accurate root back to the midpoint.
            if (std::isfinite(correction) && std::abs(correction) <= xTolerance &&
                std::abs(residual) <= rootTimeTolerance * T) {
                result.error = std::abs(correction);
                result.converged = true;
                return result;
            }
            const double candidate = x0 - correction;
            // Compare consecutive high-order steps. After bisection, allow an
            // admissible high-order step to resume without comparing it to the
            // bisection distance, which can otherwise force repeated fallback.
            if (std::isfinite(candidate) && candidate > xLower && candidate < xUpper &&
                (!previousHighOrderStep || std::abs(candidate - x0) <= 0.5 * *previousHighOrderStep)) {
                xNew = candidate;
                usedHighOrderStep = true;
            }
        }
        // A collapsed bracket cannot improve the residual; do not call stagnation convergence.
        if (xNew == x0) {
            break;
        }
        result.error = std::abs(xNew - x0);
        if (usedHighOrderStep) {
            previousHighOrderStep = result.error;
        } else {
            previousHighOrderStep.reset();
        }
        x0 = xNew;
    }
    bskLogger.bskLog(BSK_WARNING, "lambertSolver: root search failed to converge within its numerical limits.");
    return result;
}

/*! @brief Locate the multi-revolution minimum time using safeguarded Halley iterations.
    @param T0M Non-dimensional time of flight at x = 0 for N revolutions.
    @param N Number of revolutions, which must be positive.
    @return The converged location x_M and minimum time T(x_M), or std::nullopt if the search fails.
    @note Halley steps remain inside a derivative-sign bracket. Bisection is used when a step leaves
          the bracket or does not sufficiently decrease in size. Convergence requires a finite time
          and a derivative residual consistent with the tolerance on x_M.
*/
std::optional<std::array<double, 2>> LambertSolver::getTmin(double T0M, int N)
{
    const double tol = 1e-13; // [-] convergence tolerance for the minimum's location
    const int iterMax = 64;   // Allow enough iterations for bisection to reach tol.
    if (N <= 0 || !std::isfinite(T0M) || !std::isfinite(this->lambda) || abs(this->lambda) >= 1.0) {
        bskLogger.bskLog(BSK_WARNING, "lambertSolver: minimum-time search requires finite elliptic geometry.");
        return std::nullopt;
    }

    // For N > 0, dT/dx = -2 at x = 0 and tends to positive infinity as x approaches 1.
    double xLower = 0.0;
    double xUpper = 1.0;
    double previousStep = xUpper - xLower;
    double x0 = xLower;
    double tof = T0M;
    for (int j = 0; j < iterMax; ++j) {
        const std::array<double, 3> DTs = dTdx(x0, tof, this->lambda);
        const double DT = DTs[0];
        const double D2T = DTs[1];
        const double D3T = DTs[2];
        if (!std::isfinite(tof) || tof <= 0.0 || !std::isfinite(DT) || !std::isfinite(D2T) || !std::isfinite(D3T)) {
            break;
        }
        // A small step alone is insufficient: the derivative must vanish at a local minimum.
        if (D2T > 0.0 && abs(DT) <= tol * D2T) {
            return std::array<double, 2>{ x0, tof };
        }
        if (DT < 0.0) {
            xLower = x0;
        } else {
            xUpper = x0;
        }

        double xnew = 0.5 * (xLower + xUpper);
        const double denominator = D2T * D2T - DT * D3T / 2.0;
        if (std::isfinite(denominator) && denominator != 0.0) {
            const double step = DT * D2T / denominator;
            const double candidate = x0 - step;
            if (std::isfinite(candidate) && candidate > xLower && candidate < xUpper &&
                abs(step) <= 0.5 * previousStep) {
                xnew = candidate;
            }
        }
        previousStep = abs(xnew - x0);
        x0 = xnew;
        tof = x2tof(x0, N, this->lambda);
    }
    bskLogger.bskLog(BSK_WARNING, "lambertSolver: minimum-time search failed to converge to a finite minimum.");
    return std::nullopt;
}

/*! This method computes the hypergeometric function 2F1(a,b,c,z)
    @param z argument of hypergeometric function
    @return double
*/
double LambertSolver::hypergeometricF(double z)
{
    double tol = 1e-11;
    double Sj = 1.0;
    double Cj = 1.0;
    double a = 3.0;
    double b = 1.0;
    double c = 2.5;

    double Cj1;
    double Sj1;
    double err;
    for (int j = 0; j <= 12; ++j) {
        Cj1 = Cj * (a + j) * (b + j) / (c + j) * z / (j + 1);
        Sj1 = Sj + Cj1;
        err = abs(Cj1);
        Sj = Sj1;
        Cj = Cj1;
        if (err < tol) {
            break;
        }
    }
    return Sj;
}

void LambertSolver::setAlignmentThreshold(const double value)
{
    this->alignmentThreshold = value;
}
