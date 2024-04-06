/*
 ISC License
 
 Copyright (c) 2024, University of Colorado at Boulder
 
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

#include "flybyODuKF.h"

FlybyODuKF::FlybyODuKF() = default;

FlybyODuKF::~FlybyODuKF() = default;

/*! Reset the flyby OD filter to an initial state and
 initializes the internal estimation matrices.
 @return void
 @param CurrentSimNanos The clock time at which the function was called (nanoseconds)
 */
void FlybyODuKF::customReset() {
    /*! - Check if the required message has not been connected */
    assert(this->opNavHeadingMsg.isLinked());

    /*! - Initialize filter parameters and change units to km and s */
    this->muCentral *= pow(this->unitConversion, 3); // mu is input in meters
}

/*! Read the message containing the measurement data.
 * It updates class variables relating to measurement data including validity and time tags.
 @return void
 */
void FlybyODuKF::writeOutputMessages(uint64_t CurrentSimNanos) {
    NavTransMsgPayload navTransOutMsgBuffer = this->navTransOutMsg.zeroMsgPayload;
    FilterMsgPayload opNavFilterMsgBuffer = this->opNavFilterMsg.zeroMsgPayload;
    FilterResidualsMsgPayload residualsBuffer = this->opNavResidualMsg.zeroMsgPayload;

    /*! - Write the flyby OD estimate into the copy of the navigation message structure*/
    eigenMatrixXd2CArray(1/this->unitConversion*this->state.head(3), navTransOutMsgBuffer.r_BN_N);
    eigenMatrixXd2CArray(1/this->unitConversion*this->state.tail(3), navTransOutMsgBuffer.v_BN_N);

    /*! - Populate the filter states output buffer and write the output message*/
    opNavFilterMsgBuffer.timeTag = this->previousFilterTimeTag;
    eigenMatrixXd2CArray(1/this->unitConversion*this->state, opNavFilterMsgBuffer.state);
    eigenMatrixXd2CArray(1/this->unitConversion*this->xBar, opNavFilterMsgBuffer.stateError);
    eigenMatrixXd2CArray(1/this->unitConversion/this->unitConversion*this->covar, opNavFilterMsgBuffer.covar);
    opNavFilterMsgBuffer.numberOfStates = this->state.size();

    auto optionalMeasurement = this->measurements[0];
    if (optionalMeasurement.has_value()) {
        auto measurement = Measurement();
        measurement = optionalMeasurement.value();
        residualsBuffer.valid = true;
        residualsBuffer.numberOfObservations = 1;
        residualsBuffer.sizeOfObservations = measurement.observation.size();
        eigenMatrixXd2CArray(measurement.observation, &residualsBuffer.observation[0]);
        eigenMatrixXd2CArray(measurement.postFitResiduals, &residualsBuffer.postFits[0]);
        eigenMatrixXd2CArray(measurement.preFitResiduals, &residualsBuffer.preFits[0]);
        this->measurements[0].reset();
    }
    this->opNavResidualMsg.write(&residualsBuffer, this->moduleID, CurrentSimNanos);
    this->navTransOutMsg.write(&navTransOutMsgBuffer, this->moduleID, CurrentSimNanos);
    this->opNavFilterMsg.write(&opNavFilterMsgBuffer, this->moduleID, CurrentSimNanos);
}

/*! Read the message containing the measurement data.
 * It updates class variables relating to measurement data including validity and time tags.
 @return void
 */
void FlybyODuKF::readFilterMeasurements() {
    this->opNavHeadingBuffer = this->opNavHeadingMsg();
    auto headingMeasurement = Measurement();

    headingMeasurement.timeTag = this->opNavHeadingBuffer.timeTag;
    headingMeasurement.validity = this->opNavHeadingBuffer.valid;

    if (headingMeasurement.validity && headingMeasurement.timeTag >= this->previousFilterTimeTag){
        /*! - Read measurement and cholesky decomposition its noise*/
        headingMeasurement.observation = cArray2EigenVector3d(this->opNavHeadingBuffer.rhat_BN_N);
        headingMeasurement.observation.normalize();
        headingMeasurement.size = 3;
        headingMeasurement.noise = this->measNoiseScaling * cArray2EigenMatrixXd(this->opNavHeadingBuffer.covar_N,
                                                                               (int) headingMeasurement.size,
                                                                               (int) headingMeasurement.size);
        headingMeasurement.model = normalizedFirstThreeStates;
        this->measurements[0] = headingMeasurement;
    }
}

/*! Integrate the equations of motion of two body point mass gravity using Runge-Kutta 4 (RK4)
    @param interval integration interval
    @param X0 initial state
    @param dt time step
    @return Eigen::VectorXd
*/
Eigen::VectorXd FlybyODuKF::propagate(std::array<double, 2> interval, const Eigen::VectorXd& X0, double dt) const
{
    double t_0 = interval[0];
    double t_f = interval[1];
    double t = t_0;
    Eigen::VectorXd X = X0;

    std::function<Eigen::VectorXd(double, Eigen::VectorXd)> f = [this](double t, Eigen::VectorXd state)
    {
        Eigen::VectorXd stateDerivative(state.size());
        /*! Implement point mass gravity for the propagation */
        stateDerivative.segment(0,3) = state.segment(3, 3);
        stateDerivative.segment(3,3) = - this->muCentral/(pow(state.head(3).norm(),3)) * state.head(3);

        return stateDerivative;
    };

    /*! Propagate to t_final with an RK4 integrator */
    double N = ceil((t_f-t_0)/dt);
    for (int c=0; c < N; c++) {
        double step = std::min(dt,t_f-t);
        X = this->rk4(f, X, t, step);
        t = t + step;
    }

    return X;
}

/*! Set the filter measurement noise scale factor if desirable
    @param double measurementNoiseScale
    @return void
    */
void FlybyODuKF::setMeasurementNoiseScale(const double measurementNoiseScale) {
    this->measNoiseScaling = measurementNoiseScale;
}

/*! Get the filter measurement noise scale factor
    @return double measNoiseScaling
    */
double FlybyODuKF::getMeasurementNoiseScale() const {
    return this->measNoiseScaling;
}

/*! Set the gravitational parameter used for orbit propagation
    @param double muInput
    @return void
    */
void FlybyODuKF::setCentralBodyGravitationParameter(const double muInput) {
    this->muCentral = muInput;
}

/*! Get gravitational parameter used for orbit propagation
    @return double muCentral
    */
double FlybyODuKF::getCentralBodyGravitationParameter() const {
    return this->muCentral;
}
