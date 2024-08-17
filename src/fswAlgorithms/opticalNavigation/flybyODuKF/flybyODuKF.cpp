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
    double centralBody = this->muCentral;
    std::function<StateVector(double, const StateVector)> twoBodyDynamics = [centralBody](double t, const StateVector &state){
        StateVector XDot;
        /*! Implement propagation with rate derivatives set to zero */
        /*! Implement point mass gravity for the propagation */
        PositionState flybyPosition;
        VelocityState flybyVelocity;
        flybyPosition.setValues(state.getVelocityStates());
        flybyVelocity.setValues(-centralBody/(pow(state.getPositionStates().norm(),3)) * state.getPositionStates());

        XDot.setPosition(flybyPosition);
        XDot.setVelocity(flybyVelocity);

        return XDot;
    };

    /*! - Set the filter dynamics */
    this->dynamics.setDynamics(twoBodyDynamics);
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
    eigenMatrixXd2CArray(this->state.scale(1/this->unitConversion).getPositionStates(), navTransOutMsgBuffer.r_BN_N);
    eigenMatrixXd2CArray(this->state.scale(1/this->unitConversion).getVelocityStates(), navTransOutMsgBuffer.v_BN_N);

    /*! - Populate the filter states output buffer and write the output message*/
    opNavFilterMsgBuffer.timeTag = this->previousFilterTimeTag;
    eigenMatrixXd2CArray(this->state.scale(1/this->unitConversion).returnValues(), opNavFilterMsgBuffer.state);
    eigenMatrixXd2CArray(this->xBar.scale(1/this->unitConversion).returnValues(), opNavFilterMsgBuffer.stateError);
    eigenMatrixXd2CArray(1/this->unitConversion/this->unitConversion*this->covar, opNavFilterMsgBuffer.covar);
    opNavFilterMsgBuffer.numberOfStates = this->state.size();

    auto optionalMeasurement = this->measurements[0];
    if (optionalMeasurement.has_value()) {
        auto measurement = MeasurementModel();
        measurement = optionalMeasurement.value();
        residualsBuffer.valid = true;
        residualsBuffer.numberOfObservations = 1;
        residualsBuffer.sizeOfObservations = measurement.getObservation().size();
        eigenMatrixXd2CArray(measurement.getObservation(), &residualsBuffer.observation[0]);
        eigenMatrixXd2CArray(measurement.getPostFitResiduals(), &residualsBuffer.postFits[0]);
        eigenMatrixXd2CArray(measurement.getPreFitResiduals(), &residualsBuffer.preFits[0]);
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
    auto headingMeasurement = MeasurementModel();

    headingMeasurement.setTimeTag(this->opNavHeadingBuffer.timeTag);
    headingMeasurement.setValidity(this->opNavHeadingBuffer.valid);

    if (headingMeasurement.getValidity() && headingMeasurement.getTimeTag() >= this->previousFilterTimeTag){
        /*! - Read measurement and cholesky decomposition its noise*/
        headingMeasurement.setObservation(cArray2EigenVector3d(this->opNavHeadingBuffer.rhat_BN_N));
        headingMeasurement.getObservation().normalize();
        headingMeasurement.setMeasurementNoise(this->measNoiseScaling * cArray2EigenMatrixXd(this->opNavHeadingBuffer.covar_N,
                                                                               (int) headingMeasurement.size(),
                                                                               (int) headingMeasurement.size()));
        headingMeasurement.setMeasurementModel(MeasurementModel::normalizedPositionStates);
        this->measurements[0] = headingMeasurement;
    }
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
