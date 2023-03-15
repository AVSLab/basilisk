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
#include <cmath>

/*! This is the constructor for the module class.  It sets default variable
    values and initializes the various parts of the model */
LambertSolver::LambertSolver() = default;

/*! Module Destructor */
LambertSolver::~LambertSolver() = default;

/*! This method is used to reset the module and checks that required input messages are connected.
    @param CurrentSimNanos current simulation time in nano-seconds
    @return void
*/
void LambertSolver::Reset(uint64_t CurrentSimNanos)
{
    // check that required input messages are connected
    if (!this->lambertProblemInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "lambertSolver.lambertProblemInMsg was not linked.");
    }
}

/*! This is the main method that gets called every time the module is updated. It computes the solution of Lambert's problem.
    @param CurrentSimNanos current simulation time in nano-seconds
    @return void
*/
void LambertSolver::UpdateState(uint64_t CurrentSimNanos)
{
}

/*! This method reads the input messages each call of updateState. It also checks if the message contents are valid for this module.
    @return void
*/
void LambertSolver::readMessages(){
    LambertProblemMsgPayload lambertProblemInMsgBuffer = this->lambertProblemInMsg();

    // check if input parameters are valid
    if (lambertProblemInMsgBuffer.mu <= 0.0){
        bskLogger.bskLog(BSK_ERROR, "lambertSolver: mu must be positive.");
    } else {
        this->mu = lambertProblemInMsgBuffer.mu;
    }
    if (lambertProblemInMsgBuffer.transferTime <= 0.0){
        bskLogger.bskLog(BSK_ERROR, "lambertSolver: transferTime must be positive.");
    } else {
        this->transferTime = lambertProblemInMsgBuffer.transferTime;
    }
    if (lambertProblemInMsgBuffer.numRevolutions < 0){
        bskLogger.bskLog(BSK_ERROR, "lambertSolver: numberOfRevolutions must be zero or positive.");
    } else {
        this->numberOfRevolutions = lambertProblemInMsgBuffer.numRevolutions;
    }
    if (!(strcmp(lambertProblemInMsgBuffer.solverName, "Gooding") == 0 || strcmp(lambertProblemInMsgBuffer.solverName, "Izzo") == 0)){
        bskLogger.bskLog(BSK_ERROR, "lambertSolver: solverName must be either 'Gooding' or 'Izzo'.");
    } else {
        this->solverName.assign(lambertProblemInMsgBuffer.solverName);
    }

    this->r1vec = cArray2EigenVector3d(lambertProblemInMsgBuffer.r1vec);
    this->r2vec = cArray2EigenVector3d(lambertProblemInMsgBuffer.r2vec);
}

/*! This method computes the problem geometry for the given parameters of Lambert's problem. The orbit frame is also determined.
    @return void
*/
void LambertSolver::problemGeometry()
{
    double c = (this->r2vec - this->r1vec).norm(); // chord length
    double r1 = this->r1vec.norm();
    double r2 = this->r2vec.norm();
    double s = 1.0/2.0*(r1+r2+c); // semiperimeter

    // lambda parameter
    this->lambda = sqrt(1.0-c/s);
    // non-dimensional time-of-flight
    this->TOF = sqrt(2.0*this->mu/pow(s,3))*this->transferTime;

    // compute orbit frame unit vectors

    // radial direction at for initial and final position vector
    Eigen::Vector3d i_r1 = this->r1vec/r1;
    Eigen::Vector3d i_r2 = this->r2vec/r2;
    // check alignment of the two position vectors
    double sinTheta = i_r1.cross(i_r2).norm();
    if (abs(sinTheta) < sin(this->alignmentThreshold * M_PI/180.)){
        bskLogger.bskLog(BSK_WARNING, "lambertSolver: position vectors r1 and r2 are too aligned, that is, the angle between them is smaller than the angle alignmentThreshold (default: 1.0 degrees). They might not define a plane, so no solution is returned.");
        this->noSolution = true;
    }
    // orbit normal direction
    Eigen::Vector3d i_h = i_r1.cross(i_r2)/(i_r1.cross(i_r2).norm());

    double sign = 1.0;
    if (i_h(2) < 0.0){
        // Transfer angle is greater than 180 degrees
        this->lambda = -this->lambda;
        sign = -sign;
    }
    // second unit vectors
    Eigen::Vector3d i_t1 = i_h.cross(i_r1)*sign;
    Eigen::Vector3d i_t2 = i_h.cross(i_r2)*sign;

    this->Oframe1 = {i_r1, i_t1, i_h};
    this->Oframe2 = {i_r2, i_t2, i_h};
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
    double y = sqrt(1.0- pow(lam, 2) * u);

    double tof;
    if (dist < 1e-8){
        // exact time of flight for parabolic case

        tof = 2.0/3.0*(1.0 - pow(lam,3));
    }
    else if (dist < battin){
        // Use Battin formulation

        double eta = y - lam * x;
        double S1 = 0.5 * (1.0 - lam - x * eta);
        double Q = 4.0 / 3.0 * hypergeometricF(S1);
        tof = (pow(eta,3)*Q + 4.0 * lam * eta) / 2.0 + N * M_PI / (pow(abs(u), 1.5));
    }
    else{
        // Use Lancaster formulation

        double f = (y - lam * x) * sqrt(abs(u));
        double g = x*y + lam * u;
        double d;
        if (u > 0.0){
            // elliptic case
            double psi = atan2(f,g);
            d = psi + N*M_PI;
        }
        else{
            // hyperbolic case
            d = atanh(f/g);
        }

        // non-dimensional time-of-flight
        tof = (d/sqrt(abs(u)) - x + lam * y) / u;
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
    double DT; // dT/dx
    double D2T; // d2T/dx2
    double D3T; // d3T/dx3
    if (abs(x - 1.0) < 1e-8){
        // exact derivative for parabolic case
        DT = 2.0/5.0*(pow(lam,5) - 1.0);
        D2T = 0.0;
        D3T = 0.0;
    }
    else{
        double u = 1.0 - pow(x, 2);
        double y = sqrt(1.0- pow(lam, 2) * u);
        DT = 1.0/u*(3.0*T*x - 2.0 + 2.0 * pow(lam, 3) * x / y);
        D2T = 1.0/u*(3.0*T + 5.0*x*DT + 2.0 * (1. - pow(lam, 2))
                * pow(lam, 3) / pow(y, 3));
        D3T = 1.0/u*(7.0*x*D2T + 8.0*DT - 6.0 * (1. - pow(lam, 2))
                * pow(lam, 5) * x / pow(y, 5));
    }

    std::array<double, 3> DTs = {DT, D2T, D3T};

    return DTs;
}

/*! This method includes a 3rd order householder root-finder to find the free variable x that satisfies the time-of-flight constraint
    @param T requested non-dimensional time-of-flight
    @param x0 initial guess for x free variable of Lambert's problem that satisfies the given time of flight
    @param N number of revolutions
    @return std::array<double, 3>
*/
std::array<double, 3> LambertSolver::householder(double T, double x0, int N)
{
    double tol = 1e-8;
    int iterMax = 8;
    double xnew = 0.0; // initialize
    int numIter = 0; // number of iterations
    double err;

    for (int j = 0; j < iterMax; ++j){
        // compute non-dimensional time-of-flight T for given x
        double tof = x2tof(x0, N, this->lambda);
        // get derivatives of T
        std::array<double, 3> DTs = dTdx(x0, tof, this->lambda);
        double DT = DTs[0];
        double D2T = DTs[1];
        double D3T = DTs[2];

        double delta = tof - T;
        // compute new x using 3rd order householder algorithm
        xnew = x0 - delta*(pow(DT,2) - delta*D2T/2.0)/(DT*(pow(DT,2) - delta*D2T)
                + D3T*pow(delta,2)/6.0);

        err = abs(x0 - xnew);
        x0 = xnew;
        numIter += numIter;
        if (err < tol){
            break;
        }
    }
    std::array<double, 3> sol = {xnew, static_cast<double>(numIter), err};

    return sol;
}

/*! This method includes a halley root-finder (2nd order householder) to find the free variable x that satisfies the time-of-flight constraint
    @param T requested non-dimensional time-of-flight
    @param x0 initial guess for x free variable of Lambert's problem that satisfies the given time of flight
    @param N number of revolutions
    @return std::array<double, 3>
*/
std::array<double, 3> LambertSolver::halley(double T, double x0, int N)
{
    double tol = 1e-8;
    int iterMax = 8;
    double xnew = 0.0; // initialize
    int numIter = 0; // number of iterations
    double err;

    for (int j = 0; j < iterMax; ++j){
        // compute non-dimensional time-of-flight T for given x
        double tof = x2tof(x0, N, this->lambda);
        // get derivatives of T
        std::array<double, 3> DTs = dTdx(x0, tof, this->lambda);
        double DT = DTs[0];
        double D2T = DTs[1];

        double delta = tof - T;
        // compute new x using halley algorithm
        xnew = x0 - delta*DT/(pow(DT,2) - delta*D2T/2.0);
        err = abs(x0 - xnew);
        x0 = xnew;
        numIter += numIter;
        if (err < tol){
            break;
        }
    }

    std::array<double, 3> sol = {xnew, static_cast<double>(numIter), err};

    return sol;
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
    for (int j = 0; j <= 12; ++j){
        Cj1 = Cj * (a + j) * (b + j) / (c + j) * z / (j + 1);
        Sj1 = Sj + Cj1;
        err = abs(Cj1);
        Sj = Sj1;
        Cj = Cj1;
        if (err < tol){
            break;
        }
    }
    return Sj;
}
