
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

#include "ekfInterface.h"

EkfInterface::EkfInterface(FilterType type){
    this->ckfOnlyMode = type;
}

EkfInterface::~EkfInterface() = default;

/*! Reset all of the filter states, including the custom reset
 @return void
 @param currentSimNanos The clock time at which the function was called (nanoseconds)
 */
void EkfInterface::Reset(uint64_t currentSimNanos)
{
    this->customReset();

    assert(this->stateInitial.size() == this->covarInitial.rows() &&
    this->stateInitial.size() == this->covarInitial.cols());
    assert(this->stateInitial.size() - this->constantRateStates.size() == this->processNoise.rows() &&
    this->stateInitial.size() - this->constantRateStates.size() == this->processNoise.cols());

    this->state = this->unitConversion * this->stateInitial;
    this->stateError.resize(this->state.size());
    this->stateError.setZero();
    this->stateTransitionMatrix = Eigen::MatrixXd::Identity(this->state.size(), this->state.size());
    this->covar = this->unitConversion*this->unitConversion * this->covarInitial;
    this->covar.resize(this->state.size(), this->state.size());
    this->previousFilterTimeTag = (double) currentSimNanos*NANO2SEC;
    if (this->constantRateStates.size() > 0){
        this->constantRateStates = this->unitConversion * this->constantRateStates;
        this->processNoise.resize(this->constantRateStates.size(), this->constantRateStates.size());
    }
    else{
        this->processNoise.resize(this->state.size()/2, this->state.size()/2);
    }

}

/*! Filter update
 @return void
 @param currentSimNanos The clock time at which the function was called (nanoseconds)
 */
void EkfInterface::UpdateState(uint64_t currentSimNanos)
{
    this->customInitializeUpdate();
    /*! Read all available measurements, add their information to the Measurement container class, then sort the
     * vector in chronological order */
    this->readFilterMeasurements();
    this->orderMeasurementsChronologically();
    /*! Loop through all of the measurements assuming they are in chronological order by first testing if a value
     * has been populated in the measurements array*/
     for (int index =0 ; index < MAX_MEASUREMENT_NUMBER; ++index) {
         auto measurement = Measurement();
         if (!this->measurements[index].has_value()){
             continue;}
         else{
             measurement = this->measurements[index].value();}
         /*! - If the time tag from a valid measurement is new compared to previous step,
         propagate and update the filter*/
         if (measurement.timeTag >= this->previousFilterTimeTag && measurement.validity) {
             /*! - time update to the measurement time and compute pre-fit residuals*/
             this->timeUpdate(measurement.timeTag);
             measurement.preFitResiduals = EkfInterface::computeResiduals(measurement);
             /*! - measurement update and compute post-fit residuals  */
             this->measurementUpdate(measurement);
             measurement.postFitResiduals = EkfInterface::computeResiduals(measurement);
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

/*! Perform the time update for kalman filter.
 @return void
 @param updateTime The time that we need to fix the filter to (seconds)
 */
void EkfInterface::timeUpdate(const double updateTime)
{
    double dt = updateTime - this->previousFilterTimeTag;
    this->stateTransitionMatrix = Eigen::MatrixXd::Identity(this->state.size(), this->state.size());
    std::array<double, 2> time = {0, dt};

    /*! - Prepare full state and STM vector which is the state and flattened STM concatenated */
    Eigen::MatrixXd reshapedStm = this->stateTransitionMatrix.reshaped<Eigen::RowMajor>().transpose();
    Eigen::VectorXd flatStm(Eigen::Map<Eigen::VectorXd>(reshapedStm.data(), reshapedStm.cols()*reshapedStm.rows()));
    Eigen::VectorXd stateStm(this->state.size() + flatStm.size());
    stateStm << this->state, flatStm;

    /*! - Propagate full state and STM vector using dynamics specified in child class */
    Eigen::VectorXd propagatedStateStm;
    propagatedStateStm = propagate(time, stateStm, dt);

    Eigen::VectorXd propagatedStm;
    propagatedStm = propagatedStateStm.tail(this->state.size()*this->state.size());
    Eigen::MatrixXd stmTranspose(Eigen::Map<Eigen::MatrixXd>(propagatedStm.data(), this->state.size(), this->state.size()));
    this->stateTransitionMatrix = stmTranspose.transpose();

    /*! - Unpack propagated states and update state, and state error */
    this->state = propagatedStateStm.head(this->state.size());
    this->stateError = this->stateTransitionMatrix * this->stateError;

    /*! - Update the covariance Pbar = Phi*P*Phi^T + Gamma*Q*Gamma^T
     * The process noise mapping will depend on the number of "rate" states */
    Eigen::MatrixXd processNoiseMapping;
    if (this->constantRateStates.size() > 0) {
        processNoiseMapping = Eigen::MatrixXd::Identity(this->state.size(), this->state.size());
        processNoiseMapping *= pow(dt, 2) / 2;
    }
    else{
        processNoiseMapping.setZero(this->state.size(), this->state.size()/2);
        processNoiseMapping.block(0, 0, this->state.size()/2, this->state.size()/2) =
            pow(dt, 2) / 2 * Eigen::MatrixXd::Identity(this->state.size()/2, this->state.size()/2);
        processNoiseMapping.block(this->state.size()/2, 0, this->state.size()/2, this->state.size()/2) =
            dt * Eigen::MatrixXd::Identity(this->state.size()/2, this->state.size()/2);
    }
    this->covar = this->stateTransitionMatrix*this->covar*this->stateTransitionMatrix.transpose() +
            processNoiseMapping*this->processNoise*processNoiseMapping.transpose();

    this->previousFilterTimeTag = updateTime;
}


/*! Perform the measurement update for the kalman filter.
 @param Measurement
 @return void
 */
void EkfInterface::measurementUpdate(const Measurement &measurement)
{
    /*! - Compute the valid observations delta */
    Eigen::VectorXd measurementDelta = measurement.observation - measurement.model(this->state);
    /*! - Compute the measurement matrix at this state */
    Eigen::MatrixXd measurementMatrix = computeMeasurementMatrix(this->state);

    /*! - Compute the Kalman Gain */
    Eigen::MatrixXd kalmanGain = EkfInterface::computeKalmanGain(this->covar, measurementMatrix, measurement.noise);

    /*! - Update the covariance */
    EkfInterface::updateCovariance(measurementMatrix, measurement.noise, kalmanGain);
    if (this->covar.maxCoeff() > this->minCovarNorm || this->ckfOnlyMode == FilterType::Classical){
        /*! - Compute the update with a CKF if the covariance is high at the time of the update to avoid divergence*/
        EkfInterface::ckfUpdate(kalmanGain, measurementDelta, measurementMatrix);
    }
    else{
        /*! - Compute the update with a EKF, the reference state is changed by the filter update */
        EkfInterface::ekfUpdate(kalmanGain, measurementDelta);
    }
}


/*! Compute the Kalman Gain
@param Eigen::MatrixXd covar
@param Eigen::MatrixXd measurementMatrix
@param Eigen::MatrixXd measurementNoise
@return Eigen::MatrixXd
 */
Eigen::MatrixXd EkfInterface::computeKalmanGain(const Eigen::MatrixXd &covariance,
                                                const Eigen::MatrixXd &measurementMatrix,
                                                const Eigen::MatrixXd &measurementNoise) const {
    Eigen::MatrixXd kalmanGain(covariance.cols(), measurementNoise.cols());
    kalmanGain = covariance*measurementMatrix.transpose();
    kalmanGain *= (measurementMatrix*covariance*measurementMatrix.transpose() + measurementNoise).inverse();
    return kalmanGain;
}


/*! Update the covariance using the Joseph form of the update
@param Eigen::MatrixXd measMat
@param Eigen::MatrixXd noise
@param Eigen::MatrixXd kalmanGain
@return void
 */
void EkfInterface::updateCovariance(const Eigen::MatrixXd &measMat, const Eigen::MatrixXd &noise, const Eigen::MatrixXd &kalmanGain){
    Eigen::MatrixXd josephTransform(this->state.size(), this->state.size());
    josephTransform = Eigen::MatrixXd::Identity(this->state.size(), this->state.size()) - kalmanGain*measMat;
    this->covar = josephTransform*this->covar*josephTransform.transpose();
    this->covar += kalmanGain*noise*kalmanGain.transpose();
}


/*! Classical Kalman Filter Update (the reference state is unchanged)
@param Eigen::MatrixXd kalmanGain
@param Eigen::VectorXd measurementDelta
@param Eigen::MatrixXd measurementMatrix
@return void
 */
void EkfInterface::ckfUpdate(const Eigen::MatrixXd &kalmanGain,
                             const Eigen::VectorXd &measurementDelta,
                             const Eigen::MatrixXd &measurementMatrix){

    this->stateError = this->stateError + kalmanGain*(measurementDelta - measurementMatrix*this->stateError);
    this->updatedWithCkf = true;
}

/*! Extended Kalman Filter Update (the reference state is updated given the state error)
@param Eigen::MatrixXd kalmanGain
@param Eigen::VectorXd measurementDelta
@return void
 */
void EkfInterface::ekfUpdate(const Eigen::MatrixXd &kalmanGain, const Eigen::VectorXd &measurementDelta){

    this->stateError = kalmanGain*measurementDelta;
    this->state += this->stateError;
    this->updatedWithCkf = false;
}

/*! Compute the measurement residuals at a given time.
@param Measurement
@return Eigen::VectorXd
 */
Eigen::VectorXd EkfInterface::computeResiduals(const Measurement &measurement)
{
    Eigen::VectorXd measurementDelta(measurement.observation - measurement.model(this->state));
    Eigen::MatrixXd measurementMatrix = computeMeasurementMatrix(this->state);

    return measurementDelta - measurementMatrix*this->stateError;
}

/*!- Order the measurements chronologically (standard sort)
 @return void
 */
void EkfInterface::orderMeasurementsChronologically(){
    std::sort(this->measurements.begin(), this->measurements.end(),
              [](std::optional<Measurement> meas1, std::optional<Measurement> meas2){
                    if (!meas1.has_value()){return false;}
                    else if (!meas2.has_value()){return true;}
                    else {return meas1.value().timeTag < meas2.value().timeTag;}
                                  });
}

/*! Runge-Kutta 4 (RK4) function for state propagation
    @param ODEfunction function handle that includes the equations of motion
    @param X0 initial state
    @param t0 initial time
    @param dt time step
    @return Eigen::VectorXd
*/
Eigen::VectorXd EkfInterface::rk4(const std::function<Eigen::VectorXd(double, Eigen::VectorXd)>& ODEfunction,
                                const Eigen::VectorXd& X0,
                                double t0,
                                double dt) const
{
    double h = dt;

    Eigen::VectorXd k1 = ODEfunction(t0, X0);
    Eigen::VectorXd k2 = ODEfunction(t0 + h/2., X0 + h*k1/2.);
    Eigen::VectorXd k3 = ODEfunction(t0 + h/2., X0 + h*k2/2.);
    Eigen::VectorXd k4 = ODEfunction(t0 + h, X0 + h*k3);

    Eigen::VectorXd X = X0 + 1./6.*h*(k1 + 2.*k2 + 2.*k3 + k4);

    return X;
}

/*! Set a minimum value (infinite norm, meaning maximal term) of the covariance before switching to Extended KF updates.
 * This prevents divergence if the initial covariance is high and the state changes too abruptly
    @param double infiniteNorm
    @return void
    */
void EkfInterface::setMinimumCovarianceNormForEkf(const double infiniteNorm){
    assert(this->ckfOnlyMode == FilterType::Extended && "EKF minimum norm set in a Classical implementation: this is "
                                                        "only used in an extended kalman filter to temporarily use "
                                                        "linear updates when the covariance is high");
    this->minCovarNorm = infiniteNorm;
}

/*! Get the minimum value of the covariance before switching to Extended KF updates.
    @return double infiniteNorm
    */
double EkfInterface::getMinimumCovarianceNormForEkf() const {
    assert(this->ckfOnlyMode == FilterType::Extended && "EKF minimum norm requested in a Classical implementation: "
                                                        "this is only used in an extended kalman filter to temporarily "
                                                        "use linear updates when the covariance is high");
    return this->minCovarNorm;
}

/*! Set the filter initial state vector
    @param Eigen::VectorXd initialStateInput
    @return void
    */
void EkfInterface::setInitialState(const Eigen::VectorXd &initialStateInput){
    this->stateInitial.resize(initialStateInput.size());
    this->stateInitial << initialStateInput;
}

/*! Get the filter initial state vector
    @return Eigen::VectorXd stateInitial
    */
Eigen::VectorXd EkfInterface::getInitialState() const {
    return this->stateInitial;
}

/*! Set the filter initial state covariance
    @param Eigen::MatrixXd initialCovarianceInput
    @return void
    */
void EkfInterface::setInitialCovariance(const Eigen::MatrixXd &initialCovarianceInput){
    this->covarInitial.resize(initialCovarianceInput.rows(), initialCovarianceInput.cols());
    this->covarInitial << initialCovarianceInput;
}

/*! Get the filter initial state covariance
    @return Eigen::MatrixXd covarInitial
    */
Eigen::MatrixXd EkfInterface::getInitialCovariance() const {
    return this->covarInitial;
}

/*! Set the filter process noise
    @param Eigen::MatrixXd processNoiseInput
    @return void
    */
void EkfInterface::setProcessNoise(const Eigen::MatrixXd &processNoiseInput){
    this->processNoise.resize(processNoiseInput.rows(), processNoiseInput.cols());
    this->processNoise << processNoiseInput;
}

/*! Get the filter process noise
    @return Eigen::MatrixXd processNoise
    */
Eigen::MatrixXd EkfInterface::getProcessNoise() const {
    return this->processNoise;
}

/*! Set a unit conversion factor, for instance if desirable to solve for a state in km in the filter, but Basilisk's
 * outside facing interface is in SI
    @param double conversion
    */
void EkfInterface::setUnitConversionFromSItoState(const double conversion){
    this->unitConversion = conversion;
}

/*! Get a unit conversion factor, for instance if desirable to solve for a state in km in the filter, but Basilisk's
 * outside facing interface is in SI
    @return double unitConversion
    */
double EkfInterface::getUnitConversionFromSItoState() const{
    return this->unitConversion;
}

/*! Set a constant rate with which the rates will evolve: states(k+1) = states(k) + rateStates*dt
    @param Eigen::VectorXd rateStates
    @return void
    */
void EkfInterface::setConstantRateStates(const Eigen::VectorXd &rateStates){
    this->constantRateStates = rateStates;
}

/*! Get the constant rate
    @return Eigen::VectorXd rateStates
    */
Eigen::VectorXd EkfInterface::getConstantRateStates() const{
    return this->constantRateStates;
}
