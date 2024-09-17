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


#ifndef LAMBERTVALIDATOR_H
#define LAMBERTVALIDATOR_H

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/msgPayloadDefC/NavTransMsgPayload.h"
#include "architecture/msgPayloadDefC/LambertProblemMsgPayload.h"
#include "architecture/msgPayloadDefC/LambertSolutionMsgPayload.h"
#include "architecture/msgPayloadDefC/LambertPerformanceMsgPayload.h"
#include "architecture/msgPayloadDefC/DvBurnCmdMsgPayload.h"
#include "architecture/msgPayloadDefC/LambertValidatorMsgPayload.h"
#include "architecture/utilities/bskLogging.h"
#include "architecture/messaging/messaging.h"
#include "architecture/utilities/avsEigenSupport.h"
#include "architecture/utilities/astroConstants.h"

#define NUM_INITIALSTATES 27 // number of initial states

/*! @brief This module validates if the Lambert velocity solution violates any constraints and, if not,
    creates the Delta-V burn command message
 */
class LambertValidator: public SysModel {
public:
    LambertValidator();
    ~LambertValidator();

    void Reset(uint64_t currentSimNanos) override;
    void UpdateState(uint64_t currentSimNanos) override;

    ReadFunctor<NavTransMsgPayload> navTransInMsg;                          //!< translational navigation input message
    ReadFunctor<LambertProblemMsgPayload> lambertProblemInMsg;              //!< lambert problem input message
    ReadFunctor<LambertSolutionMsgPayload> lambertSolutionInMsg;            //!< lambert solution input message
    ReadFunctor<LambertPerformanceMsgPayload> lambertPerformanceInMsg;      //!< lambert performance input message
    Message<DvBurnCmdMsgPayload> dvBurnCmdOutMsg;                           //!< Delta-V burn command message
    Message<LambertValidatorMsgPayload> lambertValidatorOutMsg;             //!< Lambert Validator results message

    BSKLogger bskLogger;                                                    //!< BSK Logging

    /** setter for `lambertSolutionSpecifier` */
    void setLambertSolutionSpecifier(const double value);
    /** getter for `lambertSolutionSpecifier` */
    double getLambertSolutionSpecifier() const {return this->lambertSolutionSpecifier;}
    /** setter for `finalTime` */
    void setFinalTime(const double value);
    /** getter for `finalTime` */
    double getFinalTime() const {return this->finalTime;}
    /** setter for `maneuverTime` */
    void setManeuverTime(const double value);
    /** getter for `maneuverTime` */
    double getManeuverTime() const {return this->maneuverTime;}
    /** setter for `maxDistanceTarget` */
    void setMaxDistanceTarget(const double value);
    /** getter for `maxDistanceTarget` */
    double getMaxDistanceTarget() const {return this->maxDistanceTarget;}
    /** setter for `minOrbitRadius` */
    void setMinOrbitRadius(const double value);
    /** getter for `minOrbitRadius` */
    double getMinOrbitRadius() const {return this->minOrbitRadius;}
    /** setter for `uncertaintyStates` */
    void setUncertaintyStates(const Eigen::MatrixXd& value);
    /** getter for `uncertaintyStates` */
    Eigen::MatrixXd getUncertaintyStates() const {return this->uncertaintyStates;}
    /** setter for `uncertaintyDV` */
    void setUncertaintyDV(const double value);
    /** getter for `uncertaintyDV` */
    double getUncertaintyDV() const {return this->uncertaintyDV;}
    /** setter for `dvConvergenceTolerance` */
    void setDvConvergenceTolerance(const double value);
    /** getter for `dvConvergenceTolerance` */
    double getDvConvergenceTolerance() const {return this->dvConvergenceTolerance;}
    /** setter for `ignoreConstraintViolations` */
    void setIgnoreConstraintViolations(const bool value);
    /** getter for `ignoreConstraintViolations` */
    bool getIgnoreConstraintViolations() const {return this->ignoreConstraintViolations;}

private:
    void readMessages();
    void writeMessages(uint64_t currentSimNanos);
    std::array<Eigen::VectorXd, NUM_INITIALSTATES> getInitialStates();
    void countViolations(std::array<Eigen::VectorXd, NUM_INITIALSTATES> initialStates);
    void checkConstraintViolations(std::vector<double> t, std::vector<Eigen::VectorXd> X);
    bool checkPerformance() const;
    std::pair<std::vector<double>, std::vector<Eigen::VectorXd>> propagate(
            const std::function<Eigen::VectorXd(double, Eigen::VectorXd)>& EOM,
            std::array<double, 2> interval,
            const Eigen::VectorXd& X0,
            double dt);
    Eigen::VectorXd RK4(const std::function<Eigen::VectorXd(double, Eigen::VectorXd)>& ODEfunction,
                        const Eigen::VectorXd& X0,
                        double t0,
                        double dt);

    double lambertSolutionSpecifier = 1; //!< [-] which Lambert solution (1 or 2), if applicable, should be used
    double finalTime{}; //!< [s] time at which target position should be reached
    double maneuverTime{}; //!< [s] time at which maneuver should be executed
    double maxDistanceTarget{}; //!< [m] maximum acceptable distance from target location at final time
    double minOrbitRadius{}; //!< [m] minimum acceptable orbit radius
    Eigen::MatrixXd uncertaintyStates; //!< 6x6 matrix square root of the covariance matrix of errors, in Hill frame
    double uncertaintyDV = 0.1; //!< [m/s] uncertainty of the Delta-V magnitude
    double dvConvergenceTolerance = 1e-3; //!< [m/s] tolerance on difference between DeltaV solutions between time steps
    bool ignoreConstraintViolations = false; //!< override flag to write DV message despite constraint violations
    double time{}; //!< [s] Current vehicle time-tag associated with measurements
    Eigen::Vector3d r_N; //!< [m] Current inertial spacecraft position vector in inertial frame N components
    Eigen::Vector3d v_N; //!< [m/s] Current inertial velocity of the spacecraft in inertial frame N components
    double mu{}; //!< [m^3 s^-2] gravitational parameter
    Eigen::Vector3d r_TN_N; //!< [m] Targeted position vector with respect to celestial body at finalTime, in N frame
    Eigen::Vector3d vLambert_N; //!< [m/s] Lambert velocity solution at maneuver time in inertial frame N components
    int validLambert{}; //!< [-] valid Lambert solution if 1, not if 0
    double xLambert{}; //!< [-] solution for free variable (iteration variable) of Lambert problem
    int numIterLambert{}; //!< [-] number of root-finder iterations to find x
    double errXLambert{}; //!< [-] difference in x between last and second-to-last iteration
    double prevLambertSolutionX = 0; //!< [-] solution of Lambert problem from previous time step
    Eigen::Vector3d prevDv_N{}; //!< solution for Delta-V from previous time step
    Eigen::Vector3d rm_N; //!< [m] Expected inertial spacecraft position vector at maneuver time tm in inertial frame N
    Eigen::Vector3d vm_N; //!< [m/s] Expected inertial spacecraft velocity vector at maneuver time tm in inertial frame
    Eigen::Vector3d dv_N; //!< [m/s] requested Delta-V in N frame components
    Eigen::Matrix3d dcm_HN; //!< [-] direction cosine matrix (DCM) from inertial frame N to Hill frame H
    int violationsDistanceTarget = 0; //!< [-] number of violations of the maxDistanceTarget constraint
    int violationsOrbitRadius = 0; //!< [-] number of violations of the minOrbitRadius constraint
    double timestep = 10.; //!< [s] time step used for RK4 propagation
    std::function<Eigen::VectorXd(double, Eigen::VectorXd)> EOM_2BP; //!< equations of motion to be used for RK4
    int maxNumIterLambert = 6; //!< [-] maximum number of iterations for Lambert solver root finder to find x
    double xToleranceLambert = 1e-8; //!< [-] tolerance for Lambert solver root finder to find x
    double xConvergenceTolerance = 1e-2; //!< [-] tolerance on difference between x solutions between time steps
};

#endif
