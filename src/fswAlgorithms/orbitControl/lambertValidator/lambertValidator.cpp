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

#include "fswAlgorithms/orbitControl/lambertValidator/lambertValidator.h"
#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/macroDefinitions.h"
#include "architecture/utilities/orbitalMotion.h"
#include <cmath>
#include <vector>
#include <array>
#include <unsupported/Eigen/MatrixFunctions>

/*! This is the constructor for the module class.  It sets default variable
    values and initializes the various parts of the model */
LambertValidator::LambertValidator()
{
    this->prevDv_N.fill(0.0);
}

/*! Module Destructor */
LambertValidator::~LambertValidator() = default;

/*! This method is used to reset the module and checks that required input messages are connected.
    @param currentSimNanos current simulation time in nano-seconds

*/
void LambertValidator::Reset(uint64_t currentSimNanos)
{
    // check that required input messages are connected
    if (!this->navTransInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "lambertValidator.navTransInMsg was not linked.");
    }
    if (!this->lambertProblemInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "lambertValidator.lambertProblemInMsg was not linked.");
    }
    if (!this->lambertSolutionInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "lambertValidator.lambertSolutionInMsg was not linked.");
    }
    if (!this->lambertPerformanceInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "lambertValidator.lambertPerformanceInMsg was not linked.");
    }

    // check that the provided input module parameters are valid
    if (this->finalTime - this->maneuverTime < 0.0){
        bskLogger.bskLog(BSK_ERROR,
                         "lambertValidator: Maneuver start time maneuverTime must be before final time finalTime.");
    }
}

/*! This is the main method that gets called every time the module is updated.
    @param currentSimNanos current simulation time in nano-seconds

*/
void LambertValidator::UpdateState(uint64_t currentSimNanos)
{
    // read messages
    this->readMessages();

    // initial state vector
    Eigen::VectorXd X0(this->r_N.rows()+this->v_N.rows(), this->r_N.cols());
    X0 << this->r_N,
            this->v_N;

    // equations of motion (assuming two body point mass gravity)
    this->EOM_2BP = [this](double t, Eigen::VectorXd state)
    {
        Eigen::VectorXd stateDerivative(state.size());

        stateDerivative.segment(0,3) = state.segment(3, 3);
        stateDerivative.segment(3, 3) = -this->mu/(pow(state.head(3).norm(),3)) * state.head(3);

        return stateDerivative;
    };

    // propagate to obtain expected position at maneuver time
    std::pair<std::vector<double>, std::vector<Eigen::VectorXd>> states = this->propagate(
            this->EOM_2BP,
            {this->time, this->maneuverTime},
            X0,
            10);
    std::vector<Eigen::VectorXd> X = states.second;
    Eigen::VectorXd Xm = X.back();
    this->rm_N = Xm.head(3);
    this->vm_N = Xm.tail(3);

    // compute required Delta-V vector
    this->dv_N = this->vLambert_N - this->vm_N;

    // only propagate the perturbed initial states if Lambert solution is valid in order to safe computation effort
    // also skip propagation of perturbed initial states if the constraint violations should be ignored anyway
    if (this->validLambert == 1 && !this->ignoreConstraintViolations) {
        std::array<Eigen::VectorXd, NUM_INITIALSTATES> initialStates = this->getInitialStates();
        // check if any of the perturbed initial states violates the given constraints
        this->countViolations(initialStates);
    }

    // write messages
    this->writeMessages(currentSimNanos);

    /* update information about previous time step.
       Delta-V vector corresponds to computed Delta-V vector, not the commanded Delta-V written to the message
       (which is zeroed if constraints are violated) */
    this->prevLambertSolutionX = this->xLambert;
    this->prevDv_N = this->dv_N;
}

/*! This method reads the input messages each call of updateState.
    It also checks if the message contents are valid for this module.

*/
void LambertValidator::readMessages()
{
    NavTransMsgPayload navTransInMsgBuffer = this->navTransInMsg();
    LambertProblemMsgPayload lambertProblemInMsgBuffer = this->lambertProblemInMsg();
    LambertSolutionMsgPayload lambertSolutionInMsgBuffer = this->lambertSolutionInMsg();
    LambertPerformanceMsgPayload lambertPerformanceInMsgBuffer = this->lambertPerformanceInMsg();

    // check if input parameters are valid
    if (lambertProblemInMsgBuffer.mu <= 0.0){
        bskLogger.bskLog(BSK_ERROR, "lambertSolver: mu must be positive.");
    } else {
        this->mu = lambertProblemInMsgBuffer.mu;
    }
    if (this->maneuverTime - navTransInMsgBuffer.timeTag < 0.0){
        bskLogger.bskLog(BSK_ERROR,
                         "lambertValidator: current time must be before maneuver time maneuverTime.");
    } else {
        this->time = navTransInMsgBuffer.timeTag;
    }

    // current spacecraft state
    this->r_N = cArray2EigenVector3d(navTransInMsgBuffer.r_BN_N);
    this->v_N = cArray2EigenVector3d(navTransInMsgBuffer.v_BN_N);

    // targeted position
    this->r_TN_N = cArray2EigenVector3d(lambertProblemInMsgBuffer.r2_N);

    // lambert solution and performance message content
    if (this->lambertSolutionSpecifier == 1){
        this->vLambert_N = cArray2EigenVector3d(lambertSolutionInMsgBuffer.v1_N);
        this->validLambert = lambertSolutionInMsgBuffer.valid;
        this->xLambert = lambertPerformanceInMsgBuffer.x;
        this->numIterLambert = lambertPerformanceInMsgBuffer.numIter;
        this->errXLambert = lambertPerformanceInMsgBuffer.errX;
    }
    else if (this->lambertSolutionSpecifier == 2){
        this->vLambert_N = cArray2EigenVector3d(lambertSolutionInMsgBuffer.v1Sol2_N);
        this->validLambert = lambertSolutionInMsgBuffer.validSol2;
        this->xLambert = lambertPerformanceInMsgBuffer.xSol2;
        this->numIterLambert = lambertPerformanceInMsgBuffer.numIterSol2;
        this->errXLambert = lambertPerformanceInMsgBuffer.errXSol2;
    } else {
        bskLogger.bskLog(BSK_ERROR,
                         "lambertValidator: the parameter lambertSolutionSpecifier that specifies which "
                         "Lambert solution should be used must be either 1 or 2.");
    }
}

/*! This method writes the output messages each call of updateState
    @param currentSimNanos current simulation time in nano-seconds

*/
void LambertValidator::writeMessages(uint64_t currentSimNanos)
{
    DvBurnCmdMsgPayload dvBurnCmdOutMsgBuffer;
    dvBurnCmdOutMsgBuffer = this->dvBurnCmdOutMsg.zeroMsgPayload;
    LambertValidatorMsgPayload lambertValidatorMsgBuffer;
    lambertValidatorMsgBuffer = this->lambertValidatorOutMsg.zeroMsgPayload;

    // DV Rotation vector and rotation magnitude not used by this module. Set to arbitrary unit vector and zero.
    Eigen::Vector3d dvRotVecUnit;
    dvRotVecUnit << 1., 0., 0.;
    double dvRotVecMag = 0.;

    auto burnStartTime = static_cast<uint64_t>(this->maneuverTime * SEC2NANO);

    // check if any constraints are violated and if the Lambert solution converged
    bool goodSolution = this->checkPerformance();

    // Write Delta-V message content only if all checks on performance and violations were passed
    if (goodSolution) {
        eigenVector3d2CArray(this->dv_N, dvBurnCmdOutMsgBuffer.dvInrtlCmd);
        eigenVector3d2CArray(dvRotVecUnit, dvBurnCmdOutMsgBuffer.dvRotVecUnit);
        dvBurnCmdOutMsgBuffer.dvRotVecMag = dvRotVecMag;
        dvBurnCmdOutMsgBuffer.burnStartTime = burnStartTime;
    }

    // difference of Lambert solution w.r.t. previous time step
    double solutionDifference = xLambert - prevLambertSolutionX;
    double dvDifference = (dv_N - prevDv_N).norm();

    // Lambert Validator Message content: specify why no Delta-V command was returned
    if (this->validLambert != 1) {
        lambertValidatorMsgBuffer.failedValidLambert = 1;
    }
    if (this->numIterLambert >= this->maxNumIterLambert) {
        lambertValidatorMsgBuffer.failedNumIterationsLambert = 1;
    }
    if (abs(this->errXLambert) >= this->xToleranceLambert) {
        lambertValidatorMsgBuffer.failedXToleranceLambert = 1;
    }
    if (abs(solutionDifference) >= this->xConvergenceTolerance) {
        lambertValidatorMsgBuffer.failedXSolutionConvergence = 1;
    }
    if (abs(dvDifference) >= this->dvConvergenceTolerance) {
        lambertValidatorMsgBuffer.failedDvSolutionConvergence = 1;
    }
    if (this->violationsDistanceTarget != 0) {
        lambertValidatorMsgBuffer.failedDistanceTargetConstraint = 1;
    }
    if (this->violationsOrbitRadius != 0) {
        lambertValidatorMsgBuffer.failedOrbitRadiusConstraint = 1;
    }
    // Lambert Validator Message content: extra information
    lambertValidatorMsgBuffer.xSolutionDifference = solutionDifference;
    lambertValidatorMsgBuffer.dvSolutionDifference = dvDifference;
    lambertValidatorMsgBuffer.violationsDistanceTarget = this->violationsDistanceTarget;
    lambertValidatorMsgBuffer.violationsOrbitRadius = this->violationsOrbitRadius;
    // Delta-V vector that would be returned if all checks passed
    eigenVector3d2CArray(this->dv_N, lambertValidatorMsgBuffer.dv_N);

    // Write to the output messages
    this->dvBurnCmdOutMsg.write(&dvBurnCmdOutMsgBuffer, this->moduleID, currentSimNanos);
    this->lambertValidatorOutMsg.write(&lambertValidatorMsgBuffer, this->moduleID, currentSimNanos);
}

/*! This method creates the initial state vectors that will be propagated by the module
    @return std::array<Eigen::VectorXd, NUM_INITIALSTATES>
*/
std::array<Eigen::VectorXd, NUM_INITIALSTATES> LambertValidator::getInitialStates()
{
    // size of state vector
    int N = 6;

    // get direction cosine matrix (DCM) from inertial frame N to Hill frame H
    double rc_N[3];
    double vc_N[3];
    double HN[3][3];
    eigenVector3d2CArray(this->rm_N, rc_N);
    eigenVector3d2CArray(this->vm_N, vc_N);
    hillFrame(rc_N, vc_N, HN);
    this->dcm_HN = c2DArray2EigenMatrix3d(HN);

    // vectors expressed in Hill frame
    Eigen::Vector3d rm_H = this->dcm_HN * this->rm_N;
    Eigen::Vector3d vm_H = this->dcm_HN * this->vm_N;
    Eigen::Vector3d dv_H = this->dcm_HN * this->dv_N;
    Eigen::Vector3d dvHat_H = dv_H.normalized();

    // DCM block matrix that can be used on entire 6x1 state vector
    Eigen::MatrixXd dcm_HN_state(N, N);
    dcm_HN_state << this->dcm_HN, Eigen::Matrix3d::Zero(),
                    Eigen::Matrix3d::Zero(), this->dcm_HN;

    // nominal state vector (not perturbed) in Hill frame
    Eigen::VectorXd X0nom_H(N, 1);
    X0nom_H << rm_H,
            vm_H;

    // square root of uncertainty covariance matrix
    Eigen::MatrixXd Psqrt(N,N);
    Psqrt = this->uncertaintyStates;

    // Create initial state vectors.
    // Perturb all states in + and - direction, and for each case use min and max expected DV magnitude
    std::array<Eigen::VectorXd, NUM_INITIALSTATES> initialStates;
    for (int c1=0; c1 < N; c1++) {
        for (int c2=0; c2 < 2; c2++) {
            Eigen::VectorXd X0_H(N, 1);
            Eigen::VectorXd X0minDV_H(N, 1);
            Eigen::VectorXd X0maxDV_H(N, 1);

            double multiplier;
            if (c2 == 0) {
                multiplier = 1.;
            } else {
                multiplier = -1.;
            }

            // perturb initial state
            X0_H = X0nom_H + multiplier*Psqrt.col(c1);
            X0minDV_H = X0_H;
            X0maxDV_H = X0_H;

            // add DV magnitude perturbation
            X0minDV_H.segment(3, 3) += dv_H - this->uncertaintyDV * dvHat_H;
            X0maxDV_H.segment(3, 3) += dv_H + this->uncertaintyDV * dvHat_H;

            initialStates.at(c2*N + c1) = dcm_HN_state.transpose() * X0minDV_H;
            initialStates.at(2*N + c2*N + c1) = dcm_HN_state.transpose() * X0maxDV_H;
        }
    }

    // 3 more initial states for unperturbed state, only DV perturbation
    Eigen::VectorXd X0_H(N, 1);
    Eigen::VectorXd X0minDV_H(N, 1);
    Eigen::VectorXd X0maxDV_H(N, 1);

    X0_H = X0nom_H;
    X0minDV_H = X0_H;
    X0maxDV_H = X0_H;

    X0_H.segment(3, 3) += dv_H;
    X0minDV_H.segment(3, 3) += dv_H - this->uncertaintyDV * dvHat_H;
    X0maxDV_H.segment(3, 3) += dv_H + this->uncertaintyDV * dvHat_H;

    initialStates.at(4*N) = dcm_HN_state.transpose() * X0_H;
    initialStates.at(4*N + 1) = dcm_HN_state.transpose() * X0minDV_H;
    initialStates.at(4*N + 2) = dcm_HN_state.transpose() * X0maxDV_H;

    return initialStates;
}

/*! This method propagates the various initial states and counts the number of constraints that are violated
    @param initialStates array of initial state vectors to be propagated

*/
void LambertValidator::countViolations(std::array<Eigen::VectorXd, NUM_INITIALSTATES> initialStates)
{
    this->violationsDistanceTarget = 0;
    this->violationsOrbitRadius = 0;

    // propagate each initial condition from maneuver time to final time and check if constraints are violated
    for (int c=0; c < NUM_INITIALSTATES; c++) {
        Eigen::VectorXd X0 = initialStates.at(c);

        std::pair<std::vector<double>, std::vector<Eigen::VectorXd>> states = this->propagate(
                this->EOM_2BP,
                {this->maneuverTime, this->finalTime},
                X0,
                this->timestep);
        std::vector<double> t = states.first;
        std::vector<Eigen::VectorXd> X = states.second;

        this->checkConstraintViolations(t, X);
    }
}

/*! This method checks if the provided trajectory violates any constraints
    @param t time steps
    @param X state for each time step

*/
void LambertValidator::checkConstraintViolations(std::vector<double> t, std::vector<Eigen::VectorXd> X)
{
    // check maximum distance from target at final time constraint
    Eigen::Vector3d rf_BN_N = X.back().head(3);
    Eigen::Vector3d rf_BT_N = rf_BN_N - this->r_TN_N;
    if (rf_BT_N.norm() > this->maxDistanceTarget) {
        this->violationsDistanceTarget++;
    }
    // check minimum orbit radius constraint
    for (int c=0; c < t.size(); c++) {
        Eigen::Vector3d r_BN_N = X.at(c).head(3);
        if (r_BN_N.norm() < this->minOrbitRadius) {
            this->violationsOrbitRadius++;
            break;
        }
    }
}

/*! This method returns true only if the Lambert solution is valid, converged, and no constraints are violated
    @return bool
*/
bool LambertValidator::checkPerformance() const
{
    // difference of Lambert solution w.r.t. previous time step
    double solutionDifference = xLambert - prevLambertSolutionX;
    double dvDifference = (dv_N - prevDv_N).norm();

    bool goodSolution = false;

    /* Delta-V should only be commanded if Lambert solution is valid, converged, and resulting trajectory
       doesn't violate any constraints.
       Also return good solution if constraint violations (and convergence) should be ignored,
       as long as the Lambert solution is valid. */
    if ((this->validLambert == 1 &&
        this->numIterLambert < this->maxNumIterLambert && // Lambert module should usually take only 2-3 iterations,
        // so maxNumIterLambert is intentionally lower than in lambertSolver module
        abs(this->errXLambert) < this->xToleranceLambert &&
        abs(solutionDifference) < this->xConvergenceTolerance &&
        abs(dvDifference) < this->dvConvergenceTolerance &&
        this->violationsDistanceTarget == 0 &&
        this->violationsOrbitRadius == 0)
        ||
        (this->validLambert == 1 && this->ignoreConstraintViolations))
    {
        goodSolution = true;
    }

    return goodSolution;
}

/*! This method integrates the provided equations of motion using Runge-Kutta 4 (RK4) and returns the time steps
    and state vectors at each time step.
    @param EOM equations of motion function to be propagated
    @param interval integration interval
    @param X0 initial state
    @param dt time step
    @return std::pair<std::vector<double>, std::vector<Eigen::VectorXd>>
*/
std::pair<std::vector<double>, std::vector<Eigen::VectorXd>> LambertValidator::propagate(
        const std::function<Eigen::VectorXd(double, Eigen::VectorXd)>& EOM,
        std::array<double, 2> interval,
        const Eigen::VectorXd& X0,
        double dt)
{
    double t0 = interval[0];
    double tf = interval[1];

    std::vector<double> t = {t0};
    std::vector<Eigen::VectorXd> X = {X0};

    // propagate forward to tf
    double N = ceil(abs(tf-t0)/dt);
    for (int c=0; c < N; c++) {
        double step = std::min(dt,abs(tf-t.at(c))); // for last time step, step size might be smaller than dt
        // special case for backwards propagation
        if (tf < t0) {
            step = -step;
        }

        Eigen::VectorXd Xnew = this->RK4(EOM, X.at(c), t.at(c), step);
        double tnew = t.at(c) + step;

        t.push_back(tnew);
        X.push_back(Xnew);
    }
    std::pair<std::vector<double>, std::vector<Eigen::VectorXd>> statesOut = {t,X};

    return statesOut;
}

/*! This method provides the 4th order Runge-Kutta (RK4)
    @param ODEfunction function handle that includes the equations of motion
    @param X0 initial state
    @param t0 initial time
    @param dt time step
    @return Eigen::VectorXd
*/
Eigen::VectorXd LambertValidator::RK4(const std::function<Eigen::VectorXd(double, Eigen::VectorXd)>& ODEfunction,
                                      const Eigen::VectorXd& X0,
                                      double t0,
                                      double dt)
{
    double h = dt;

    Eigen::VectorXd k1 = ODEfunction(t0, X0);
    Eigen::VectorXd k2 = ODEfunction(t0 + h/2., X0 + h*k1/2.);
    Eigen::VectorXd k3 = ODEfunction(t0 + h/2., X0 + h*k2/2.);
    Eigen::VectorXd k4 = ODEfunction(t0 + h, X0 + h*k3);

    Eigen::VectorXd X = X0 + 1./6.*h*(k1 + 2.*k2 + 2.*k3 + k4);

    return X;
}

void LambertValidator::setLambertSolutionSpecifier(const double value){
    this->lambertSolutionSpecifier = value;
}

void LambertValidator::setFinalTime(const double value){
    this->finalTime = value;
}

void LambertValidator::setManeuverTime(const double value){
    this->maneuverTime = value;
}

void LambertValidator::setMaxDistanceTarget(const double value){
    this->maxDistanceTarget = value;
}

void LambertValidator::setMinOrbitRadius(const double value){
    this->minOrbitRadius = value;
}

void LambertValidator::setUncertaintyStates(const Eigen::MatrixXd& value){
    this->uncertaintyStates = value;
}

void LambertValidator::setUncertaintyDV(const double value){
    this->uncertaintyDV = value;
}

void LambertValidator::setDvConvergenceTolerance(const double value){
    this->dvConvergenceTolerance = value;
}

void LambertValidator::setIgnoreConstraintViolations(const bool value){
    this->ignoreConstraintViolations = value;
}
