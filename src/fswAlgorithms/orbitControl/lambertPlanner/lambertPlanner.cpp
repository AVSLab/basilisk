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

#include "fswAlgorithms/orbitControl/lambertPlanner/lambertPlanner.h"
#include "architecture/utilities/linearAlgebra.h"
#include <cmath>
#include <array>

/*! This is the constructor for the module class.  It sets default variable
    values and initializes the various parts of the model */
LambertPlanner::LambertPlanner()
{
    this->useSolverIzzoMethod();
}

/*! Module Destructor */
LambertPlanner::~LambertPlanner() = default;

/*! This method is used to reset the module and checks that required input messages are connected.
    @param currentSimNanos current simulation time in nano-seconds

*/
void LambertPlanner::Reset(uint64_t currentSimNanos)
{
    // check that required input messages are connected
    if (!this->navTransInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "lambertPlanner.navTransInMsg was not linked.");
    }

    // check that the provided input module parameters are valid
    if (this->finalTime - this->maneuverTime < 0.0){
        bskLogger.bskLog(BSK_ERROR,
                         "lambertPlanner: Maneuver start time maneuverTime must be before final time finalTime.");
    }
}

/*! This is the main method that gets called every time the module is updated.
    @param currentSimNanos current simulation time in nano-seconds

*/
void LambertPlanner::UpdateState(uint64_t currentSimNanos)
{
    // read messages
    this->readMessages();

    // initial state vector
    Eigen::VectorXd X0(this->r_N.rows()+this->v_N.rows(), this->r_N.cols());
    X0 << this->r_N,
            this->v_N;

    // equations of motion (assuming two body point mass gravity)
    std::function<Eigen::VectorXd(double, Eigen::VectorXd)> EOM = [this](double t, Eigen::VectorXd state)
    {
        Eigen::VectorXd stateDerivative(state.size());

        stateDerivative.segment(0,3) = state.segment(3, 3);
        stateDerivative.segment(3, 3) = -this->mu/(pow(state.head(3).norm(),3)) * state.head(3);

        return stateDerivative;
    };

    // propagate to obtain expected position at maneuver time tm
    std::pair<std::vector<double>, std::vector<Eigen::VectorXd>> states = this->propagate(
            EOM,
            {this->time, this->maneuverTime},
            X0,
            10);
    std::vector<Eigen::VectorXd> X = states.second;
    Eigen::VectorXd Xm = X.back();
    this->rm_N = Xm.head(3);

    // write messages
    this->writeMessages(currentSimNanos);
}

/*! This method sets the lambert solver algorithm that should be used to the method by Izzo.

*/
void LambertPlanner::useSolverIzzoMethod()
{
    this->solverMethod = IZZO;
}

/*! This method sets the lambert solver algorithm that should be used to the method by Gooding.

*/
void LambertPlanner::useSolverGoodingMethod()
{
    this->solverMethod = GOODING;
}

/*! This method reads the input messages each call of updateState.
    It also checks if the message contents are valid for this module.

*/
void LambertPlanner::readMessages()
{
    NavTransMsgPayload navTransInMsgBuffer = this->navTransInMsg();

    if (this->maneuverTime - navTransInMsgBuffer.timeTag < 0.0){
        bskLogger.bskLog(BSK_ERROR,
                         "lambertPlanner: current time must be before maneuver time maneuverTime.");
    } else {
        this->time = navTransInMsgBuffer.timeTag;
    }
    this->r_N = cArray2EigenVector3d(navTransInMsgBuffer.r_BN_N);
    this->v_N = cArray2EigenVector3d(navTransInMsgBuffer.v_BN_N);
}

/*! This method writes the output messages each call of updateState
    @param currentSimNanos current simulation time in nano-seconds

*/
void LambertPlanner::writeMessages(uint64_t currentSimNanos)
{
    LambertProblemMsgPayload lambertProblemOutMsgBuffer;
    lambertProblemOutMsgBuffer = this->lambertProblemOutMsg.zeroMsgPayload;

    // Write message content
    lambertProblemOutMsgBuffer.solverMethod = this->solverMethod;
    eigenVector3d2CArray(this->rm_N, lambertProblemOutMsgBuffer.r1_N);
    eigenVector3d2CArray(this->r_TN_N, lambertProblemOutMsgBuffer.r2_N);
    lambertProblemOutMsgBuffer.transferTime = this->finalTime - this->maneuverTime;
    lambertProblemOutMsgBuffer.mu = this->mu;
    lambertProblemOutMsgBuffer.numRevolutions = this->numRevolutions;

    // Write to the output messages
    this->lambertProblemOutMsg.write(&lambertProblemOutMsgBuffer, this->moduleID, currentSimNanos);
}

/*! This method integrates the provided equations of motion using Runge-Kutta 4 (RK4) and returns the time steps and
    state vectors at each time step.
    @param EOM equations of motion function to be propagated
    @param interval integration interval
    @param X0 initial state
    @param dt time step
    @return std::pair<std::vector<double>, std::vector<Eigen::VectorXd>>
*/
std::pair<std::vector<double>, std::vector<Eigen::VectorXd>> LambertPlanner::propagate(
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
Eigen::VectorXd LambertPlanner::RK4(const std::function<Eigen::VectorXd(double, Eigen::VectorXd)>& ODEfunction,
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

void LambertPlanner::setR_TN_N(const Eigen::Vector3d value)
{
    this->r_TN_N = value;
}

void LambertPlanner::setFinalTime(const double value){
    this->finalTime = value;
}

void LambertPlanner::setManeuverTime(const double value){
    this->maneuverTime = value;
}

void LambertPlanner::setMu(const double value){
    if (value < 0.0){
        bskLogger.bskLog(BSK_ERROR, "lambertPlanner: mu must be positive.");
    }
    this->mu = value;
}

void LambertPlanner::setNumRevolutions(const int value){
    this->numRevolutions = value;
}
