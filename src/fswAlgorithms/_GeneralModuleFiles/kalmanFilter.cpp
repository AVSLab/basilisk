
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

#include "kalmanFilter.h"

KalmanFilter::KalmanFilter() = default;

KalmanFilter::~KalmanFilter() = default;

void KalmanFilter::Reset(uint64_t currentSimNanos) {
    assert(this->stateInitial.size() == this->covarInitial.rows() &&
    this->stateInitial.size() == this->covarInitial.cols());

    this->state = this->stateInitial.scale(this->unitConversion);
    this->stateLogged = this->state;
    this->covar = this->unitConversion*this->unitConversion * this->covarInitial;
    this->covar.resize(this->state.size(), this->state.size());

    this->previousFilterTimeTag = (double) currentSimNanos*NANO2SEC;
}

/*! Take the relative position measurements and outputs an estimate of the
 spacecraft states in the inertial frame.
 @return void
 @param currentSimNanos The clock time at which the function was called (nanoseconds)
 */
void KalmanFilter::UpdateState(uint64_t currentSimNanos)
{
    this->customInitializeUpdate();
    /*! Read all available measurements, add their information to the Measurement container class, then sort the
     * vector in chronological order */
    this->readFilterMeasurements();
    this->orderMeasurementsChronologically();
    /*! Loop through all of the measurements assuming they are in chronological order by first testing if a value
     * has been populated in the measurements array*/
     for (int index =0 ; index < MAX_MEASUREMENT_NUMBER; ++index) {
         auto measurement = MeasurementModel();
         if (!this->measurements[index].has_value()){
             continue;}
         else{
             measurement = this->measurements[index].value();}
         /*! - If the time tag from a valid measurement is new compared to previous step,
         propagate and update the filter*/
         if (measurement.getTimeTag() >= this->previousFilterTimeTag && measurement.getValidity()) {
             /*! - time update to the measurement time and compute pre-fit residuals*/
             this->timeUpdate(measurement.getTimeTag());
             measurement.setPreFitResiduals(this->computeResiduals(measurement));
             /*! - measurement update and compute post-fit residuals  */
             this->measurementUpdate(measurement);
             measurement.setPostFitResiduals(measurement.getObservation() - measurement.model(this->state));
             this->measurements[index] = measurement;
         }
     }
    /*! - If current clock time is further ahead than the last measurement time, then
    propagate to this current time-step*/
    if ((double) currentSimNanos * NANO2SEC > this->previousFilterTimeTag) {
        this->timeUpdate((double) currentSimNanos * NANO2SEC);
    }
    this->customFinalizeUpdate();
    this->writeOutputMessages(currentSimNanos);
}

/*!- Order the measurements chronologically (standard sort)
 @return void
 */
void KalmanFilter::orderMeasurementsChronologically(){
    std::sort(this->measurements.begin(), this->measurements.end(),
              [](std::optional<MeasurementModel> meas1, std::optional<MeasurementModel> meas2){
                    if (!meas1.has_value()){return false;}
                    else if (!meas2.has_value()){return true;}
                    else {return meas1.value().getTimeTag() < meas2.value().getTimeTag();}
                                  });
}

/*! Set the filter initial state position vector
    @param Eigen::VectorXd initial position vector
    @return void
    */
void KalmanFilter::setInitialPosition(const Eigen::VectorXd &initialPositionInput){
    PositionState positionState;
    positionState.setValues(initialPositionInput);
    this->stateInitial.setPosition(positionState);
}

/*! Get the filter initial state position vector
    @return std::optional<Eigen::VectorXd> initial position vector
    */
std::optional<Eigen::VectorXd> KalmanFilter::getInitialPosition() const {
    std::optional<Eigen::VectorXd> position;
    if (this->stateInitial.hasPosition()){
        position = this->stateInitial.getPositionStates();
    }
    return position;
}

/*! Set the filter initial state velocity vector
    @param Eigen::VectorXd  initial velocity vector
    @return void
    */
void KalmanFilter::setInitialVelocity(const Eigen::VectorXd &initialVelocityInput){
    VelocityState velocityState;
    velocityState.setValues(initialVelocityInput);
    this->stateInitial.setVelocity(velocityState);
}

/*! Get the filter initial state velocity vector
    @return  std::optional<Eigen::VectorXd> initial velocity vector
    */
std::optional<Eigen::VectorXd> KalmanFilter::getInitialVelocity() const {
    std::optional<Eigen::VectorXd> velocity;
    if (this->stateInitial.hasVelocity()){
        velocity = this->stateInitial.getVelocityStates();
    }
    return velocity;
}

/*! Set the filter initial state acceleration vector
    @param  Eigen::VectorXd initial acceleration vector
    @return void
    */
void KalmanFilter::setInitialAcceleration(const Eigen::VectorXd &initialAccelerationInput){
    AccelerationState accelerationState;
    accelerationState.setValues(initialAccelerationInput);
    this->stateInitial.setAcceleration(accelerationState);
}

/*! Get the filter initial state acceleration vector
    @return  std::optional<Eigen::VectorXd> initial acceleration vector
    */
std::optional<Eigen::VectorXd> KalmanFilter::getInitialAcceleration() const {
    std::optional<Eigen::VectorXd> acceleration;
    if (this->stateInitial.hasAcceleration()){
        acceleration = this->stateInitial.getAccelerationStates();
    }
    return acceleration;
}

/*! Set the filter initial state bias vector
    @param Eigen::VectorXd  initial bias vector
    @return void
    */
void KalmanFilter::setInitialBias(const Eigen::VectorXd &initialBiasInput){
    BiasState biasState;
    biasState.setValues(initialBiasInput);
    this->stateInitial.setBias(biasState);
}

/*! Get the filter initial state bias vector
    @return std::optional<Eigen::VectorXd> initial bias vector
    */
std::optional<Eigen::VectorXd> KalmanFilter::getInitialBias() const {
    std::optional<Eigen::VectorXd> bias;
    if (this->stateInitial.hasBias()){
        bias = this->stateInitial.getBiasStates();
    }
    return bias;
}

/*! Set the filter initial state consider parameter vector
    @param Eigen::VectorXd initial consider parameter vector
    @return void
    */
void KalmanFilter::setInitialConsiderParameters(const Eigen::VectorXd &initialConsiderInput){
    ConsiderState considerState;
    considerState.setValues(initialConsiderInput);
    this->stateInitial.setConsider(considerState);
}

/*! Get the filter initial state consider parameter vector
    @return std::optional<Eigen::VectorXd>  initial consider parameter vector
    */
std::optional<Eigen::VectorXd> KalmanFilter::getInitialConsiderParameters() const {
    std::optional<Eigen::VectorXd> consider;
    if (this->stateInitial.hasConsider()){
        consider = this->stateInitial.getConsiderStates();
    }
    return consider;
}

/*! Set the filter equations of motion of the state (X_dot = f(t, X))
    @param Eigen::VectorXd initialStateInput
    @return void
    */
void KalmanFilter::setFilterDynamics(const std::function<const StateVector(const double, const StateVector&)>&
        dynamicsPropagator){
    this->dynamics.setDynamics(dynamicsPropagator);
}

/*! Set the filter initial state covariance
    @param Eigen::MatrixXd initialCovarianceInput
    @return void
    */
void KalmanFilter::setInitialCovariance(const Eigen::MatrixXd &initialCovarianceInput){
    this->covarInitial.resize(initialCovarianceInput.rows(), initialCovarianceInput.cols());
    this->covarInitial << initialCovarianceInput;
}

/*! Get the filter initial state covariance
    @return Eigen::MatrixXd covarInitial
    */
Eigen::MatrixXd KalmanFilter::getInitialCovariance() const {
    return this->covarInitial;
}

/*! Set the filter process noise
    @param Eigen::MatrixXd processNoiseInput
    @return void
    */
void KalmanFilter::setProcessNoise(const Eigen::MatrixXd &processNoiseInput){
    this->processNoise.resize(processNoiseInput.rows(), processNoiseInput.cols());
    this->processNoise << processNoiseInput;
}

/*! Get the filter process noise
    @return Eigen::MatrixXd processNoise
    */
Eigen::MatrixXd KalmanFilter::getProcessNoise() const {
    return this->processNoise;
}

/*! Set the filter measurement noise scale factor if desirable
    @param double measurementNoiseScale
    @return void
    */
void KalmanFilter::setMeasurementNoiseScale(const double measurementNoiseScale) {
    this->measNoiseScaling = measurementNoiseScale;
}

/*! Get the filter measurement noise scale factor
    @return double measurementNoiseScale
    */
double KalmanFilter::getMeasurementNoiseScale() const {
    return this->measNoiseScaling;
}

/*! Set a unit conversion factor, for instance if desirable to solve for a state in km in the filter, but Basilisk's
 * outside facing interface is in SI
    @param double conversion
    */
void KalmanFilter::setUnitConversionFromSItoState(const double conversion){
    this->unitConversion = conversion;
}

/*! Get a unit conversion factor, for instance if desirable to solve for a state in km in the filter, but Basilisk's
 * outside facing interface is in SI
    @return double unitConversion
    */
double KalmanFilter::getUnitConversionFromSItoState() const{
    return this->unitConversion;
}
