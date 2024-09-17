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


#ifndef LAMBERTPLANNER_H
#define LAMBERTPLANNER_H

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/msgPayloadDefC/NavTransMsgPayload.h"
#include "architecture/msgPayloadDefC/LambertProblemMsgPayload.h"
#include "architecture/utilities/bskLogging.h"
#include "architecture/messaging/messaging.h"
#include "architecture/utilities/avsEigenSupport.h"
#include "architecture/utilities/astroConstants.h"
#include <vector>

/*! @brief This module creates the LambertProblemMsg to be used for the LambertSolver module
 */
class LambertPlanner: public SysModel {
public:
    LambertPlanner();
    ~LambertPlanner();

    void Reset(uint64_t currentSimNanos) override;
    void UpdateState(uint64_t currentSimNanos) override;

    void useSolverIzzoMethod();
    void useSolverGoodingMethod();

    ReadFunctor<NavTransMsgPayload> navTransInMsg;                      //!< translational navigation input message
    Message<LambertProblemMsgPayload> lambertProblemOutMsg;             //!< lambert problem output message

    BSKLogger bskLogger;                                                //!< BSK Logging

    /** setter for `r_TN_N` */
    void setR_TN_N(const Eigen::Vector3d value);
    /** getter for `r_TN_N` */
    Eigen::Vector3d getR_TN_N() const {return this->r_TN_N;}
    /** setter for `finalTime` */
    void setFinalTime(const double value);
    /** getter for `finalTime` */
    double getFinalTime() const {return this->finalTime;}
    /** setter for `maneuverTime` */
    void setManeuverTime(const double value);
    /** getter for `maneuverTime` */
    double getManeuverTime() const {return this->maneuverTime;}
    /** setter for `mu` */
    void setMu(const double value);
    /** getter for `mu` */
    double getMu() const {return this->mu;}
    /** setter for `numRevolutions` */
    void setNumRevolutions(const int value);
    /** getter for `numRevolutions` */
    int getNumRevolutions() const {return this->numRevolutions;}

private:
    void readMessages();
    void writeMessages(uint64_t currentSimNanos);
    std::pair<std::vector<double>, std::vector<Eigen::VectorXd>> propagate(
            const std::function<Eigen::VectorXd(double, Eigen::VectorXd)>& EOM,
            std::array<double, 2> interval,
            const Eigen::VectorXd& X0,
            double dt);
    Eigen::VectorXd RK4(const std::function<Eigen::VectorXd(double, Eigen::VectorXd)>& ODEfunction,
                        const Eigen::VectorXd& X0,
                        double t0,
                        double dt);

    Eigen::Vector3d r_TN_N; //!< [m] targeted position vector with respect to celestial body at finalTime, in N frame
    double finalTime{}; //!< [s] time at which target position should be reached
    double maneuverTime{}; //!< [s] time at which maneuver should be executed
    double mu{}; //!< [m^3 s^-2] gravitational parameter
    int numRevolutions = 0; //!< [-] number of revolutions to be completed (completed orbits)
    SolverMethod solverMethod; //!< lambert solver algorithm (GOODING or IZZO)
    double time{}; //!< [s] Current vehicle time-tag associated with measurements
    Eigen::Vector3d r_N; //!< [m] Current inertial spacecraft position vector in inertial frame N components
    Eigen::Vector3d v_N; //!< [m/s] Current inertial velocity of the spacecraft in inertial frame N components
    Eigen::Vector3d rm_N; //!< [m] Expected inertial spacecraft position vector at maneuver time tm in inertial frame N
};

#endif
