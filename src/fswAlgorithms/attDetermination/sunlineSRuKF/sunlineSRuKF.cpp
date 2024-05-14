/*
 ISC License

 Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

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
    this->state.head(3).normalize();
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
    eigenMatrixXd2CArray(this->state.head(3), navAttOutMsgBuffer.vehSunPntBdy);

    /*! - Populate the filter states output buffer and write the output message*/
    filterMsgBuffer.timeTag = this->previousFilterTimeTag;
    eigenMatrixXd2CArray(1/this->unitConversion*this->state, filterMsgBuffer.state);
    eigenMatrixXd2CArray(1/this->unitConversion*this->xBar, filterMsgBuffer.stateError);
    eigenMatrixXd2CArray(1/this->unitConversion/this->unitConversion*this->covar, filterMsgBuffer.covar);
    filterMsgBuffer.numberOfStates = this->state.size();

    auto optionalMeasurement = this->measurements[0];
    if (optionalMeasurement.has_value() && optionalMeasurement->name == "gyro") {
        auto measurement = Measurement();
        measurement = optionalMeasurement.value();
        filterGyroResMsgBuffer.valid = true;
        filterGyroResMsgBuffer.numberOfObservations = 1;
        filterGyroResMsgBuffer.sizeOfObservations = measurement.observation.size();
        eigenMatrixXd2CArray(measurement.observation, &filterGyroResMsgBuffer.observation[0]);
        eigenMatrixXd2CArray(measurement.postFitResiduals, &filterGyroResMsgBuffer.postFits[0]);
        eigenMatrixXd2CArray(measurement.preFitResiduals, &filterGyroResMsgBuffer.preFits[0]);
        this->measurements[0].reset();
    }
    optionalMeasurement = this->measurements[1];
    if (optionalMeasurement.has_value() && optionalMeasurement->name == "css") {
        auto measurement = Measurement();
        measurement = optionalMeasurement.value();
        filterCssResMsgBuffer.valid = true;
        filterCssResMsgBuffer.numberOfObservations = 1;
        filterCssResMsgBuffer.sizeOfObservations = measurement.observation.size();
        eigenMatrixXd2CArray(measurement.observation, &filterCssResMsgBuffer.observation[0]);
        eigenMatrixXd2CArray(measurement.postFitResiduals, &filterCssResMsgBuffer.postFits[0]);
        eigenMatrixXd2CArray(measurement.preFitResiduals, &filterCssResMsgBuffer.preFits[0]);
        this->measurements[1].reset();
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

    auto gyroMeasurements = Measurement();
    gyroMeasurements.validity = true;
    gyroMeasurements.name = "gyro";
    gyroMeasurements.size = 3;
    gyroMeasurements.timeTag = navAttInputBuffer.timeTag;
    gyroMeasurements.observation = cArray2EigenVector3d(navAttInputBuffer.omega_BN_B);
    gyroMeasurements.model = lastThreeStates;
    gyroMeasurements.noise.resize(3, 3);
    Eigen::MatrixXd I = Eigen::Matrix3d::Identity();
    gyroMeasurements.noise = pow(this->measNoiseScaling * this->gyroMeasNoiseStd, 2) * I;

    if (gyroMeasurements.validity && gyroMeasurements.timeTag >= this->previousFilterTimeTag){
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

    auto cssMeasurements = Measurement();

    /*! - Zero the observed active CSS count */
    this->numActiveCss = 0;

    /*! - Define the linear model matrix H */
    Eigen::MatrixXd H;

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
            cssMeasurements.validity = true;
            cssMeasurements.timeTag = cssInputBuffer.timeTag;
            cssMeasurements.observation.conservativeResize(this->numActiveCss+1);
            cssMeasurements.observation(this->numActiveCss) = cssInputBuffer.CosValue[i];
            H.conservativeResize(this->numActiveCss+1, 3);
            for (int j=0; j<3; ++j) {
                H(this->numActiveCss,j) = this->cssConfigInputBuffer.cssVals[i].CBias * this->cssConfigInputBuffer.cssVals[i].nHat_B[j];
            }
            cssMeasurements.noise.resize(this->numActiveCss+1, this->numActiveCss+1);
            Eigen::MatrixXd I(this->numActiveCss+1, this->numActiveCss+1);
            I.setIdentity();
            cssMeasurements.noise = pow(this->measNoiseScaling * this->cssMeasNoiseStd, 2) * I;
            this->numActiveCss += 1;
        }
    }
    cssMeasurements.size = this->numActiveCss;

    std::function<const Eigen::VectorXd(const Eigen::VectorXd)> linearModel = [H](const Eigen::VectorXd &state) {
        return H * state.head(3);
    };

    if (cssMeasurements.validity && cssMeasurements.timeTag >= this->previousFilterTimeTag){
        /*! - Read measurement and cholesky decomposition its noise*/
        cssMeasurements.model = linearModel;
        cssMeasurements.name = "css";
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

/*! Integrate the equations of motion of two body point mass gravity using Runge-Kutta 4 (RK4)
    @param interval integration interval
    @param X0 initial state
    @param dt time step
    @return Eigen::VectorXd
*/
Eigen::VectorXd SunlineSRuKF::propagate(std::array<double, 2> interval, const Eigen::VectorXd& X0, double dt){
    double t_0 = interval[0];
    double t_f = interval[1];
    double t = t_0;
    Eigen::VectorXd X = X0;

    std::function<Eigen::VectorXd(double, Eigen::VectorXd)> stateDerivative = [](double t, Eigen::VectorXd state)
    {
        Eigen::VectorXd XDot(state.size());
        /*! Implement propagation with rate derivatives set to zero */
        Eigen::Vector3d sHat  = state.segment(0, 3);
        Eigen::Vector3d omega = state.segment(3, 3);
        XDot.segment(0,3) = sHat.cross(omega);
        XDot.segment(3,3).setZero();

        return XDot;
    };

    /*! Propagate to t_final with an RK4 integrator */
    double N = ceil((t_f-t_0)/dt);
    for (int c=0; c < N; c++) {
        double step = std::min(dt,t_f-t);
        X = this->rk4(stateDerivative, X, t, step);
        t = t + step;
    }

    return X;
}

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

/*! Set the filter measurement noise scale factor if desirable
    @param double measurementNoiseScale
    @return void
    */
void SunlineSRuKF::setMeasurementNoiseScale(const double measurementNoiseScale) {
    this->measNoiseScaling = measurementNoiseScale;
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

/*! Get the filter measurement noise scale factor
    @return double measNoiseScaling
    */
double SunlineSRuKF::getMeasurementNoiseScale() const {
    return this->measNoiseScaling;
}
