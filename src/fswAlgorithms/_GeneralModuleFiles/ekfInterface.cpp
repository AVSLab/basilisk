
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
    this->filterType = type;
}

EkfInterface::~EkfInterface() = default;

/*! Reset all of the filter states, including the custom reset
 @return void
 @param currentSimNanos The clock time at which the function was called (nanoseconds)
 */
void EkfInterface::Reset(uint64_t currentSimNanos)
{
    KalmanFilter::Reset(currentSimNanos);
    this->stateError = Eigen::VectorXd ::Zero(this->state.size());
    this->stateTransitionMatrix = Eigen::MatrixXd::Identity(this->state.size(), this->state.size());
    if (this->stateInitial.hasVelocity()) {
        this->processNoise.resize(this->state.getVelocityStates().size(), this->state.getVelocityStates().size());
    }
    else {
        this->processNoise.resize(this->state.getPositionStates().size(), this->state.getPositionStates().size());
    }

    this->customReset();
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

    /*! - Propagate full state and STM vector using dynamics specified in child class */
    StateVector stateStm = this->state;
    stateStm.attachStm(this->stateTransitionMatrix);
    StateVector propagatedStateStm = this->dynamics.propagate(time, stateStm, dt);

    this->stateTransitionMatrix = propagatedStateStm.detachStm();

    /*! - Unpack propagated states and update state, and state error */
    this->state = propagatedStateStm;
    this->stateError = this->stateTransitionMatrix * this->stateError;
    this->stateLogged = this->state.addVector(this->stateError);

    /*! - Update the covariance Pbar = Phi*P*Phi^T + Gamma*Q*Gamma^T
     * The process noise mapping will depend on the number of "rate" states */
    Eigen::MatrixXd processNoiseMapping;
    if (!this->state.hasVelocity()) {
        processNoiseMapping = Eigen::MatrixXd::Identity(this->state.getPositionStates().size(),
                                                        this->state.getPositionStates().size());
        processNoiseMapping *= pow(dt, 2) / 2;
    }
    else{
        processNoiseMapping.setZero(this->state.getPositionStates().size() + this->state.getVelocityStates().size(),
                                    this->state.getVelocityStates().size());
        processNoiseMapping.block(0, 0, this->state.getPositionStates().size(),
                                  this->state.getPositionStates().size()) =
            pow(dt, 2) / 2 * Eigen::MatrixXd::Identity(this->state.getPositionStates().size(),
                                  this->state.getPositionStates().size());
        processNoiseMapping.block(this->state.getPositionStates().size(), 0, this->state.getVelocityStates().size(),
                                                        this->state.getVelocityStates().size()) =
            dt * Eigen::MatrixXd::Identity(this->state.getVelocityStates().size(),
                                                        this->state.getVelocityStates().size());
    }
    this->covar = this->stateTransitionMatrix*this->covar*this->stateTransitionMatrix.transpose() +
            processNoiseMapping*this->processNoise*processNoiseMapping.transpose();

    this->previousFilterTimeTag = updateTime;
}


/*! Perform the measurement update for the kalman filter.
 @param Measurement
 @return void
 */
void EkfInterface::measurementUpdate(const MeasurementModel &measurement)
{
    /*! - Compute the valid observations delta */
    Eigen::VectorXd measurementDelta = measurement.getObservation() - measurement.model(this->state);
    /*! - Compute the measurement matrix at this state */
    Eigen::MatrixXd measurementMatrix = measurement.computeMeasurementMatrix(this->state);

    /*! - Compute the Kalman Gain */
    Eigen::MatrixXd kalmanGain = EkfInterface::computeKalmanGain(this->covar, measurementMatrix, measurement.getMeasurementNoise());

    /*! - Update the covariance */
    EkfInterface::updateCovariance(measurementMatrix, measurement.getMeasurementNoise(), kalmanGain);
    if (this->covar.maxCoeff() > this->minCovarNorm || this->filterType == FilterType::Classical){
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
    this->stateLogged = this->state.addVector(this->stateError);
}

/*! Extended Kalman Filter Update (the reference state is updated given the state error)
@param Eigen::MatrixXd kalmanGain
@param Eigen::VectorXd measurementDelta
@return void
 */
void EkfInterface::ekfUpdate(const Eigen::MatrixXd &kalmanGain, const Eigen::VectorXd &measurementDelta){

    this->stateError = kalmanGain*measurementDelta;

    this->state = this->state.addVector(this->stateError);
    this->stateLogged = this->state;
}

/*! Compute the measurement residuals at a given time.
@param Measurement
@return Eigen::VectorXd
 */
Eigen::VectorXd EkfInterface::computeResiduals(const MeasurementModel &measurement)
{
    Eigen::VectorXd measurementDelta(measurement.getObservation() - measurement.model(this->state));
    Eigen::MatrixXd measurementMatrix = measurement.computeMeasurementMatrix(this->state);

    return measurementDelta - measurementMatrix*this->stateError;
}

/*! Get the filter dynamics matrix (A = df/dX evaluated at the reference)
    @return Eigen::VectorXd stateInitial
    */
void EkfInterface::setFilterDynamicsMatrix(const std::function<const Eigen::MatrixXd(const double, const StateVector&)>&
            dynamicsMatrixCalculator){
    this->dynamics.setDynamicsMatrix(dynamicsMatrixCalculator);
}

/*! Set a minimum value (infinite norm, meaning maximal term) of the covariance before switching to Extended KF updates.
 * This prevents divergence if the initial covariance is high and the state changes too abruptly
    @param double infiniteNorm
    @return void
    */
void EkfInterface::setMinimumCovarianceNormForEkf(const double infiniteNorm){
    assert(this->filterType == FilterType::Extended && "EKF minimum norm set in a Classical implementation: this is "
                                                        "only used in an extended kalman filter to temporarily use "
                                                        "linear updates when the covariance is high");
    this->minCovarNorm = infiniteNorm;
}

/*! Get the minimum value of the covariance before switching to Extended KF updates.
    @return double infiniteNorm
    */
double EkfInterface::getMinimumCovarianceNormForEkf() const {
    assert(this->filterType == FilterType::Extended && "EKF minimum norm requested in a Classical implementation: "
                                                        "this is only used in an extended kalman filter to temporarily "
                                                        "use linear updates when the covariance is high");
    return this->minCovarNorm;
}
