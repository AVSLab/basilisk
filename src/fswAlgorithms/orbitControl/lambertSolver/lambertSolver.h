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


#ifndef LAMBERTSOLVER_H
#define LAMBERTSOLVER_H

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/msgPayloadDefC/LambertProblemMsgPayload.h"
#include "architecture/msgPayloadDefC/LambertSolutionMsgPayload.h"
#include "architecture/msgPayloadDefC/LambertPerformanceMsgPayload.h"
#include "architecture/utilities/bskLogging.h"
#include "architecture/messaging/messaging.h"
#include "architecture/utilities/avsEigenSupport.h"
#include "architecture/utilities/astroConstants.h"
#include <vector>
#include <array>

/*! @brief This module solves Lambert's problem using either the Gooding or the Izzo algorithm.
 */
class LambertSolver: public SysModel {
public:
    LambertSolver();
    ~LambertSolver();

    void Reset(uint64_t currentSimNanos) override;
    void UpdateState(uint64_t currentSimNanos) override;

    ReadFunctor<LambertProblemMsgPayload> lambertProblemInMsg;          //!< lambert problem input message
    Message<LambertSolutionMsgPayload> lambertSolutionOutMsg;           //!< lambert solution output message
    Message<LambertPerformanceMsgPayload> lambertPerformanceOutMsg;     //!< lambert performance output message

    BSKLogger bskLogger;                                                //!< BSK Logging

    /** setter for `alignmentThreshold` */
    void setAlignmentThreshold(const double value);
    /** getter for `alignmentThreshold` */
    double getlignmentThreshold() const {return this->alignmentThreshold;}

private:
    void readMessages();
    void writeMessages(uint64_t currentSimNanos);
    void problemGeometry();
    std::array<double, 2> goodingInitialGuess(double lambda, double T);
    std::array<double, 2> izzoInitialGuess(double lambda, double T);
    void findx();
    std::array<Eigen::Vector3d, 2> computeVelocities(double x);
    double x2tof(double x, int N, double lam);
    std::array<double, 3> dTdx(double x, double T, double lam);
    std::array<double, 3> householder(double T, double x0, int N);
    std::array<double, 3> halley(double T, double x0, int N);
    double getTmin(double T0M, int N);
    double hypergeometricF(double z);

    double alignmentThreshold = 1.0; //!< [deg] minimum angle between position vectors so they are not too aligned.
    SolverMethod solverMethod; //!< lambert solver algorithm (GOODING or IZZO)
    Eigen::Vector3d r1_N; //!< position vector at t0
    Eigen::Vector3d r2_N; //!< position vector at t1
    double transferTime{}; //!< time of flight between r1_N and r2_N (t1-t0)
    double mu{}; //!< gravitational parameter
    int numberOfRevolutions{}; //!< number of revolutions
    double TOF{}; //!< non-dimensional time-of-flight constraint
    double lambda{}; //!< parameter of Lambert"s problem that defines problem geometry
    bool multiRevSolution{}; //!< boolean flag if multi-revolution solutions exist or not
    bool noSolution{}; //!< boolean flag if no solution should be returned (in case of 180 deg transfer angle)
    std::array<Eigen::Vector3d, 3> Oframe1; //!< array containing the orbit frame unit vectors at t0
    std::array<Eigen::Vector3d, 3> Oframe2; //!< array containing the orbit frame unit vectors at t1
    std::array<Eigen::Vector3d, 2> vvecs; //!< array containing the velocity vector solutions at t0 and t1
    std::array<Eigen::Vector3d, 2> vvecsSol2; //!< array containing the velocity vector solutions at t0 and t1 (sol 2)
    double X{}; //!< solution for free variable of Lambert's problem
    double XSol2{}; //!< second solution for free variable of Lambert's problem
    int numIter{}; //!< number of root finder iterations to find X
    int numIterSol2{}; //!< number of root finder iterations to find X_sol2
    double errX{}; //!< difference in X between last and second-to-last iteration
    double errXSol2{}; //!< difference in X between last and second-to-last iteration (for X_sol2)
};

#endif
