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

#include "sunlineSRuKF.h"

SunlineSRuKF::SunlineSRuKF() = default;

SunlineSRuKF::~SunlineSRuKF() = default;

/*! Initialize C-wrapped output messages */
void SunlineSRuKF::SelfInit(){
    NavAttMsg_C_init(&this->navAttOutMsgC);
    this->setFilterDynamics(SunlineSRuKF::stateDerivative);
}

/*! Reset the sunline filter to an initial state and
 initializes the internal estimation matrices.
 @return void
 @param CurrentSimNanos The clock time at which the function was called (nanoseconds)
 */
void SunlineSRuKF::customReset() {
    /*! - Check if the required messages have been connected */
    assert(this->cssDataInMsg.isLinked());
    assert(this->cssConfigInMsg.isLinked());
    assert(this->navAttInMsg.isLinked());

    /*! read in CSS configuration message */
    this->cssConfigInputBuffer = this->cssConfigInMsg();
}

/*! Normalize the updated sunline estimate
 @return void
 @param CurrentSimNanos The clock time at which the function was called (nanoseconds)
 */
void SunlineSRuKF::customFinalizeUpdate() {
    PositionState heading;
    heading.setValues(this->state.getPositionStates().normalized());
    this->state.setPosition(heading);
}

/*! Read the message containing the measurement data.
 It updates class variables relating to measurement data including validity and time tags.
 @return void
 */
void SunlineSRuKF::writeOutputMessages(uint64_t CurrentSimNanos) {
    NavAttMsgPayload navAttOutMsgBuffer = this->navAttOutMsg.zeroMsgPayload;
    FilterMsgPayload filterMsgBuffer = this->filterOutMsg.zeroMsgPayload;
    FilterResidualsMsgPayload filterGyroResMsgBuffer = this->filterGyroResOutMsg.zeroMsgPayload;
    FilterResidualsMsgPayload filterCssResMsgBuffer = this->filterCssResOutMsg.zeroMsgPayload;

    /*! - Write the sunline estimate into the copy of the navigation message structure*/
    eigenMatrixXd2CArray(this->state.getPositionStates(), navAttOutMsgBuffer.vehSunPntBdy);

    /*! - Populate the filter states output buffer and write the output message*/
    filterMsgBuffer.timeTag = this->previousFilterTimeTag;
    eigenMatrixXd2CArray(this->state.returnValues(), filterMsgBuffer.state);
    eigenMatrixXd2CArray(this->xBar.returnValues(), filterMsgBuffer.stateError);
    eigenMatrixXd2CArray(this->covar, filterMsgBuffer.covar);
    filterMsgBuffer.numberOfStates = this->state.size();

    int i = 0;
    for(auto optionalMeasurement : this->measurements){
        if (optionalMeasurement.has_value() && optionalMeasurement->getMeasurementName() == "gyro") {
            auto measurement = MeasurementModel();
            measurement = optionalMeasurement.value();
            filterGyroResMsgBuffer.valid = true;
            filterGyroResMsgBuffer.numberOfObservations = 1;
            filterGyroResMsgBuffer.sizeOfObservations = measurement.size();
            eigenMatrixXd2CArray(measurement.getObservation(), &filterGyroResMsgBuffer.observation[0]);
            eigenMatrixXd2CArray(measurement.getPostFitResiduals(), &filterGyroResMsgBuffer.postFits[0]);
            eigenMatrixXd2CArray(measurement.getPreFitResiduals(), &filterGyroResMsgBuffer.preFits[0]);
        }
        else if (optionalMeasurement.has_value() && optionalMeasurement->getMeasurementName() == "css") {
            auto measurement = MeasurementModel();
            measurement = optionalMeasurement.value();
            filterCssResMsgBuffer.valid = true;
            filterCssResMsgBuffer.numberOfObservations = 1;
            filterCssResMsgBuffer.sizeOfObservations = measurement.size();
            eigenMatrixXd2CArray(measurement.getObservation(), &filterCssResMsgBuffer.observation[0]);
            eigenMatrixXd2CArray(measurement.getPostFitResiduals(), &filterCssResMsgBuffer.postFits[0]);
            eigenMatrixXd2CArray(measurement.getPreFitResiduals(), &filterCssResMsgBuffer.preFits[0]);
        }
        this->measurements[i].reset();
        i += 1;
    }


    this->navAttOutMsg.write(&navAttOutMsgBuffer, this->moduleID, CurrentSimNanos);
    NavAttMsg_C_write(&navAttOutMsgBuffer, &this->navAttOutMsgC, this->moduleID, CurrentSimNanos);
    this->filterOutMsg.write(&filterMsgBuffer, this->moduleID, CurrentSimNanos);
    this->filterCssResOutMsg.write(&filterCssResMsgBuffer, this->moduleID, CurrentSimNanos);
    this->filterGyroResOutMsg.write(&filterGyroResMsgBuffer, this->moduleID, CurrentSimNanos);
}

/*! Read the rate gyro input message
 @return void
 */
void SunlineSRuKF::readGyroMeasurements() {
    /*! Read rate gyro measurements */
    NavAttMsgPayload navAttInputBuffer = this->navAttInMsg();

    if (navAttInputBuffer.timeTag >= this->previousFilterTimeTag){
        auto gyroMeasurements = MeasurementModel();
        gyroMeasurements.setValidity(true);
        gyroMeasurements.setMeasurementName("gyro");
        gyroMeasurements.setTimeTag(navAttInputBuffer.timeTag);
        gyroMeasurements.setObservation(cArray2EigenVector3d(navAttInputBuffer.omega_BN_B));
        gyroMeasurements.setMeasurementModel(MeasurementModel::velocityStates);
        Eigen::MatrixXd I = Eigen::Matrix3d::Identity();
        gyroMeasurements.setMeasurementNoise(this->measNoiseScaling * pow(this->gyroMeasNoiseStd, 2) * I);

        /*! - Read measurement and cholesky decomposition its noise*/
        this->measurements[this->filterMeasurement] = gyroMeasurements;
        this->filterMeasurement += 1;
    }
}

/*! Read the coarse sun sensor input message
 @return void
 */
void SunlineSRuKF::readCssMeasurements() {
    /*! Read css data msg */
    CSSArraySensorMsgPayload cssInputBuffer = this->cssDataInMsg();
    auto cssMeasurements = MeasurementModel();
    cssMeasurements.setValidity(false);

    /*! - Zero the observed active CSS count */
    this->numActiveCss = 0;

    /*! - Define the linear model matrix H */
    Eigen::MatrixXd hMatrix;
    Eigen::VectorXd cssObservation;

    /*! - Loop over the maximum number of sensors to check for good measurements */
    /*! -# Isolate if measurement is good */
    /*! -# Set body vector for this measurement */
    /*! -# Get measurement value into observation vector */
    /*! -# Set inverse noise matrix */
    /*! -# increase the number of valid observations */
    /*! -# Otherwise just continue */
    for(uint32_t i=0; i<this->cssConfigInputBuffer.nCSS; ++i)
    {
        if (cssInputBuffer.CosValue[i] > this->sensorUseThresh)
        {
            cssMeasurements.setValidity(true);
            cssObservation.conservativeResize(this->numActiveCss+1);
            cssObservation(this->numActiveCss) = cssInputBuffer.CosValue[i];
            hMatrix.conservativeResize(this->numActiveCss+1, 3);
            for (int j=0; j<3; ++j) {
                hMatrix(this->numActiveCss,j) = this->cssConfigInputBuffer.cssVals[i].CBias *
                        this->cssConfigInputBuffer.cssVals[i].nHat_B[j];
            }
            cssMeasurements.setTimeTag(cssInputBuffer.timeTag);
            this->numActiveCss += 1;
        }
    }

    std::function<const Eigen::VectorXd(const StateVector)> linearModel = [hMatrix](const StateVector &state) {
        Eigen::VectorXd observed = hMatrix * state.getPositionStates();
        return observed;
    };

    if (cssMeasurements.getValidity() && cssMeasurements.getTimeTag() >= this->previousFilterTimeTag){
        /*! - Read measurement and cholesky decomposition its noise*/
        Eigen::MatrixXd I(this->numActiveCss, this->numActiveCss);
        I.setIdentity();
        cssMeasurements.setMeasurementNoise(this->measNoiseScaling * pow(this->cssMeasNoiseStd, 2) * I);
        cssMeasurements.setObservation(cssObservation);
        cssMeasurements.setMeasurementModel(linearModel);
        cssMeasurements.setMeasurementName("css");
        this->measurements[this->filterMeasurement] = cssMeasurements;
        this->filterMeasurement += 1;
    }
}

/*! Read the message containing the measurement data.
 * It updates class variables relating to measurement data including validity and time tags.
 @return void
 */
void SunlineSRuKF::readFilterMeasurements() {
    /*! zero filter measurement index */
    this->filterMeasurement = 0;

    this->readGyroMeasurements();
    this->readCssMeasurements();
}

/*! Define the equations of motion for the filter dynamics
    @param double time
    @return StateVector inputState
    @return StateVector outputState
    */
StateVector SunlineSRuKF::stateDerivative(const double t, const StateVector &state){
    StateVector XDot;
    /*! Implement propagation with rate derivatives set to zero */
    Eigen::Vector3d sHat  = state.getPositionStates();
    Eigen::Vector3d omega = state.getVelocityStates();

    PositionState xDotPosition;
    VelocityState xDotVelocity;

    xDotPosition.setValues(sHat.cross(omega));
    xDotVelocity.setValues(Eigen::VectorXd::Zero(3));

    XDot.setPosition(xDotPosition);
    XDot.setVelocity(xDotVelocity);

    return XDot;
};

/*! Set the CSS measurement noise
    @param double cssMeasurementNoise
    @return void
    */
void SunlineSRuKF::setCssMeasurementNoiseStd(const double cssMeasurementNoiseStd) {
    this->cssMeasNoiseStd = cssMeasurementNoiseStd;
}

/*! Set the gyro measurement noise
    @param double gyroMeasurementNoise
    @return void
    */
void SunlineSRuKF::setGyroMeasurementNoiseStd(const double gyroMeasurementNoiseStd) {
    this->gyroMeasNoiseStd = gyroMeasurementNoiseStd;
}

/*! Get the CSS measurement noise
    @param double cssMeasurementNoise
    @return void
    */
double SunlineSRuKF::getCssMeasurementNoiseStd() const {
    return this->cssMeasNoiseStd;
}

/*! Get the gyro measurement noise
    @param double gyroMeasurementNoise
    @return void
    */
double SunlineSRuKF::getGyroMeasurementNoiseStd() const {
    return this->gyroMeasNoiseStd;
}

/*! Set the threshold value to accept a css measurement
    @param double threshold
    @return void
    */
void SunlineSRuKF::setSensorThreshold(double threshold){
    this->sensorUseThresh = threshold;
}

/*! Get the threshold value to accept a css measurement
    @return double threshold
    */
double SunlineSRuKF::getSensorThreshold() const{
    return this->sensorUseThresh;
}