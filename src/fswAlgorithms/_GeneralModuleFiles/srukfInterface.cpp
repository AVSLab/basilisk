
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

#include "srukfInterface.h"

SRukfInterface::SRukfInterface() = default;

SRukfInterface::~SRukfInterface() = default;

/*! Reset all of the filter states, including the custom reset
 @return void
 @param currentSimNanos The clock time at which the function was called (nanoseconds)
 */
void SRukfInterface::Reset(uint64_t currentSimNanos)
{
    this->customReset();

    assert(this->stateInitial.size() == this->covarInitial.rows() &&
    this->stateInitial.size() == this->covarInitial.cols());
    assert(this->stateInitial.size() == this->processNoise.rows() &&
    this->stateInitial.size() == this->processNoise.cols());

    this->state = this->unitConversion * this->stateInitial;
    this->sBar = this->unitConversion*this->unitConversion * this->covarInitial;
    this->covar = this->unitConversion*this->unitConversion * this->covarInitial;
    this->covar.resize(this->state.size(), this->state.size());
    this->sBar.resize(this->state.size(), this->state.size());
    this->processNoise.resize(this->state.size(), this->state.size());

    this->previousFilterTimeTag = (double) currentSimNanos*NANO2SEC;
    this->numberSigmaPoints = this->state.size()*2+1;

    /*! - Ensure that all internal filter matrices are zeroed*/
    this->wM.setZero(this->numberSigmaPoints);
    this->wC.setZero(this->numberSigmaPoints);
    this->sBar.setZero(this->state.size(), this->state.size());
    this->sigmaPoints.setZero(this->state.size(), this->numberSigmaPoints);
    this->cholProcessNoise.setZero(this->state.size(), this->state.size());

    /*! - Set lambda/gamma to standard value for unscented kalman filters */
    this->lambda = (double) this->state.size()*(this->alpha*this->alpha - 1);
    this->eta = sqrt((double) this->state.size() + this->lambda);

    /*! - Set the wM/wC vectors to standard values for unscented kalman filters*/
    this->wM(0) = this->lambda / ((double) this->state.size() + this->lambda);
    this->wC(0) = this->lambda / ((double) this->state.size() + this->lambda) +
            (1 - this->alpha*this->alpha + this->beta);
    for (size_t i = 1; i < this->numberSigmaPoints; ++i)
    {
        this->wM(i) = 1.0 / (2.0 * ((double) this->state.size() + this->lambda));
        this->wC(i) = this->wM(i);
    }

    /*! - Perform cholesky decompositions of covariance and noise */
    this->sBar = SRukfInterface::choleskyDecomposition(this->covar);
    this->cholProcessNoise = SRukfInterface::choleskyDecomposition(this->unitConversion*this->unitConversion*
            this->processNoise);
}

/*! Take the relative position measurements and outputs an estimate of the
 spacecraft states in the inertial frame.
 @return void
 @param currentSimNanos The clock time at which the function was called (nanoseconds)
 */
void SRukfInterface::UpdateState(uint64_t currentSimNanos)
{
    this->customInitializeUpdate();
    /*! Read all available measurements, add their information to the Measurement container class, then sort the
     * vector in chronological order */
    this->readFilterMeasurements();
    this->orderMeasurementsChronologically();
    /*! Loop through all of the measurements assuming they are in chronological order by first testing if a value
     * has been populated in the measurements array*/
     for (int index =0 ; index < MAX_MEASUREMENT_DEFAULT; ++index) {
         auto measurement = Measurement();
         if (!this->measurements[index].has_value()){
             continue;}
         else{
             measurement = this->measurements[index].value();}
         /*! - If the time tag from a valid measurement is new compared to previous step,
         propagate and update the filter*/
         if (measurement.timeTag * NANO2SEC >= this->previousFilterTimeTag && measurement.validity) {
              /*! - prepare measurement data */
             measurement.choleskyNoise = SRukfInterface::choleskyDecomposition(measurement.noise);

             /*! - time update to the measurement time and compute pre-fit residuals*/
             this->timeUpdate(measurement.timeTag * NANO2SEC);
             measurement.preFitResiduals = this->computeResiduals(measurement);
             /*! - measurement update and compute post-fit residuals  */
             this->measurementUpdate(measurement);
             measurement.postFitResiduals = measurement.observation - measurement.model(this->state);
             this->measurements[index] = measurement;
         }
     }
    /*! - If current clock time is further ahead than the last measurement time, then
    propagate to this current time-step*/
    if ((double) currentSimNanos * NANO2SEC >= this->previousFilterTimeTag) {
        this->timeUpdate((double) currentSimNanos * NANO2SEC);
    }
    this->customFinalizeUpdate();
    this->writeOutputMessages(currentSimNanos);
}

/*! Perform the time update for kalman filter.
 It propagates the sigma points forward in time and then gets the current
 covariance and state estimates.
 @return void
 @param updateTime The time that we need to fix the filter to (seconds)
 */
void SRukfInterface::timeUpdate(double updateTime)
{
    Eigen::VectorXd propagatedSigmaPoint;
    Eigen::MatrixXd A(this->state.size(), 3*this->state.size());

    double dt = updateTime - this->previousFilterTimeTag;
    std::array<double, 2> time = {0, dt};

    /*! - Copy over the current state estimate into the 0th Sigma point and propagate by dt*/
    this->sigmaPoints.col(0) = propagate(time, this->state, dt);
    /*! - Scale that Sigma point by the appopriate scaling factor (Wm[0])*/
    this->xBar = this->wM(0)*this->sigmaPoints.col(0);

    /*! - For each Sigma point, apply sBar-based error, propagate forward, and scale by Wm just like 0th.
     Note that we perform +/- sigma points simultaneously in loop to save loop values.*/
    for (size_t i = 1; i<this->state.size() + 1; ++i)
    {
        /*! - Adding covariance columns from sigma points*/
        this->sigmaPoints.col(i) = propagate(time, this->state + this->eta * this->sBar.col(i-1), dt);
        /*! - Subtracting covariance columns from sigma points*/
        this->sigmaPoints.col(i + this->state.size()) =
                propagate(time, this->state - this->eta * this->sBar.col(i-1), dt);
    }

    /*! - Compute xbar according to Eq (19)*/
    for (size_t i = 1; i<this->numberSigmaPoints; ++i)
    {
        this->xBar += this->wM(i)*this->sigmaPoints.col(i);
    }

    /*! - Assemble the A matrix for QR decomposition as seen in equation 20 in the reference document*/
    for (size_t i = 1; i < this->numberSigmaPoints; ++i)
    {
        A.col(i-1) = sqrt(this->wC(i))*(this->sigmaPoints.col(i) - this->xBar);
    }

    A.block(0, this->numberSigmaPoints-1, this->state.size(), this->state.size()) =
            this->cholProcessNoise;

    /*! - QR decomposition (only R is of interest) of the A matrix provides the new sBar matrix*/
    this->sBar = SRukfInterface::qrDecompositionJustR(A);

    /*! - Shift the sBar matrix over by the xBar vector using the appropriate weight
     like in equation 21 in design document.*/
    Eigen::VectorXd xError = this->sigmaPoints.col(0) - this->xBar;

    /*! - Cholesky update block for vectors.*/
    this->sBar = SRukfInterface::choleskyUpDownDate(this->sBar, xError, this->wC(0));

    this->covar = this->sBar*this->sBar.transpose();
    this->state = this->sigmaPoints.col(0);
    this->previousFilterTimeTag = updateTime;
}


/*! Perform the measurement update for the kalman filter.
 It applies the observations in the obs vectors to the current state estimate and
 updates the state/covariance with that information.
 @param Measurement
 @return void
 */
void SRukfInterface::measurementUpdate(const Measurement &measurement)
{
    /*! - Compute the valid observations and the measurement model for all observations*/
    Eigen::MatrixXd yMeas(measurement.size, this->numberSigmaPoints);
    for(size_t j=0; j < this->numberSigmaPoints; ++j)
    {
        /*! Sigma points positions need to be normalized for the measurement model.*/
        yMeas.col(j) = measurement.model(this->sigmaPoints.col(j));
    }

    /*! - Compute the value for the yBar parameter (note that this is equation 23 in the
     time update section of the reference document*/
    Eigen::VectorXd yBar;
    yBar.setZero(measurement.size);
    for(size_t i=0; i<this->numberSigmaPoints; ++i)
    {
        yBar += this->wM(i) * yMeas.col(i);
    }

    /*! - Populate the matrix that we perform the QR decomposition on in the measurement
     update section of the code.  This is based on the difference between the yBar
     parameter and the calculated measurement models.  Equation 24. */
    Eigen::MatrixXd A(measurement.size, 2*this->state.size() + measurement.size);
    for(size_t i=1; i<this->numberSigmaPoints; ++i)
    {
        A.col(i-1) = sqrt(this->wC(1))*(yMeas.col(i) - yBar);
    }
    A.block(0, this->numberSigmaPoints-1, measurement.size, measurement.size) =
            measurement.choleskyNoise;

    /*! - Perform QR decomposition (only R again) of the above matrix to obtain the
     current Sy matrix*/
    Eigen::MatrixXd sy;
    sy.setZero(measurement.size, measurement.size);
    sy = SRukfInterface::qrDecompositionJustR(A);

    /*! - Cholesky update block for vectors.*/
    Eigen::VectorXd yError = yMeas.col(0) - yBar;
    sy = SRukfInterface::choleskyUpDownDate(sy, yError, this->wC(0));

    /*! - Construct the Pxy matrix (equation 26) which multiplies the Sigma-point cloud
     by the measurement model cloud (weighted) to get the total Pxy matrix*/
    Eigen::VectorXd xError;
    xError.setZero(this->state.size());
    Eigen::MatrixXd kMat;
    kMat.setZero(this->state.size(), measurement.size);
    Eigen::MatrixXd STkMatT;
    STkMatT.setZero(measurement.size, this->state.size());
    Eigen::MatrixXd pXY;
    pXY.setZero(this->state.size(), measurement.size);

    for(size_t i=0; i<this->numberSigmaPoints; ++i)
    {
        xError = this->sigmaPoints.col(i) - this->xBar;
        yError = yMeas.col(i) - yBar;
        kMat =  this->wC(i) * xError * yError.transpose();
        pXY += kMat;
    }

    /*! - Then we need to invert the SyT*Sy matrix to get the Kalman gain factor.  Since
     The Sy matrix is lower triangular, we can do a back-sub inversion instead of
     a full matrix inversion. Equation 27 in the reference document.*/
    STkMatT = SRukfInterface::forwardSubstitution(sy, pXY.transpose());
    kMat = SRukfInterface::backSubstitution(sy.transpose(), STkMatT).transpose();

    /*! - Difference the yBar and the observations to get the observed error and
     multiply by the Kalman Gain to get the state update.  Add the state update
     to the state to get the updated state value (equation 27).*/
    this->state = this->xBar + kMat*(measurement.observation - yBar);

    /*! - Compute the updated matrix U from equation 28 */
    Eigen::MatrixXd Umat;
    Umat.setZero(this->state.size(), measurement.size);
    Umat = kMat * sy;

    /*! - For each column in the update matrix, perform a cholesky down-date on it to
     get the total shifted S matrix (called sBar in internal parameters*/
    for(int i=0; i < Umat.cols(); ++i)
    {
        this->sBar = SRukfInterface::choleskyUpDownDate(this->sBar, Umat.col(i), -1);
    }

    /*! - Compute equivalent covariance based on updated sBar matrix*/
    this->covar = this->sBar*this->sBar.transpose();
}

/*! Compute the measurement residuals if the measurement data was fresh.
 * The post fits are y - ybar if a measurement was read, if observations are not present
 * a flag is raised to not compute post fit residuals
@param Measurement
@return Eigen::VectorXd
 */
Eigen::VectorXd SRukfInterface::computeResiduals(const Measurement &measurement)
{
    /*! - Compute Post Fit Residuals, first get Y (eq 22) using the states post fit*/
    Eigen::MatrixXd yMeas(measurement.size, this->numberSigmaPoints);
    for(size_t j=0; j < this->numberSigmaPoints; ++j)
    {
        /*! Sigma points positions need to be normalized for the measurement model.*/
        yMeas.col(j) = measurement.model(this->sigmaPoints.col(j));
    }
    /*! - Compute the value for the yBar parameter (equation 23)*/
    Eigen::VectorXd yBar;
    yBar.setZero(measurement.observation.size());
    for(size_t i=0; i<this->numberSigmaPoints; ++i){
        yBar += this->wM(i)*yMeas.col(i);
    }
    return measurement.observation - yBar;
}

/*!- Order the measurements chronologically (standard sort)
 @return void
 */
void SRukfInterface::orderMeasurementsChronologically(){
    std::sort(this->measurements.begin(), this->measurements.end(),
              [](std::optional<Measurement> meas1, std::optional<Measurement> meas2){
                    if (!meas1.has_value()){return false;}
                    else if (!meas2.has_value()){return true;}
                    else {return meas1.value().timeTag < meas2.value().timeTag;}
                                  });
}

/*! Perform a QR decomposition using HouseholderQR from Eigen, but only returns the transpose of the
 * upper triangular R matrix truncated such that it is the same size as the input matrix.
 @return Eigen::MatrixXd
 @param Eigen::MatrixXd input : The input matrix. If not square, provide it with more cols then rows
 */
Eigen::MatrixXd SRukfInterface::qrDecompositionJustR(const Eigen::MatrixXd input) const
{
    Eigen::HouseholderQR<Eigen::MatrixXd> qrDecomposition(input.transpose());
    Eigen::MatrixXd R_tilde;
    R_tilde.setZero(input.rows(), input.rows());
    Eigen::MatrixXd R;
    R.setZero(input.cols(), input.rows());

    /*! Use Eigen Householder method to perform a QR decomposition and retrieve just the R matrix.
     * Math is described in the first bullet of section 3 page 3 of the reference document */
    Eigen::MatrixXd Q;
    Q = qrDecomposition.householderQ();
    R = Q.transpose()*input.transpose();
    R_tilde = R.block(0,0,input.rows(), input.rows());

    /*! Zero all terms that should be zero to avoid errors accumulating */
    for (int i =0; i < R_tilde.rows(); ++i){
        for (int j = 0 ; j < i; ++j){
            R_tilde(i, j) = 0;
        }
    }

    return R_tilde.transpose();
}

/*! Perform the cholesky up or down date. The sign of the value term determines weather to update or downdate
 * the input matrix using the input vector.
 * Return the QR decomposed matrix of the up/down dated input matrix.
 @return Eigen::MatrixXd
 @param Eigen::MatrixXd input : The input matrix which is recomposed into a P matrix P = S.S^T
 @param Eigen::VectorXd inputVector : Take it's outer product V.V^T to add into the recomposed P matrix
 @param Eigen::VectorXd coefficient : Factor that is square rooted and scales the outer product P +/- sqrt(v)V.V^T
 */
Eigen::MatrixXd SRukfInterface::choleskyUpDownDate(const Eigen::MatrixXd input,
                                               const Eigen::VectorXd inputVector,
                                               const double coefficient) const
{
    Eigen::MatrixXd P;
    P.setZero(inputVector.size(), inputVector.size());

    /*! Perform the Cholesky factor updating.
     * Math is described in the second bullet of section 3 page 3 of the reference document */
    P = input * input.transpose();
    int sign = (coefficient > 0) ? 1 : -1;
    P += sign*abs(coefficient)*inputVector*inputVector.transpose();

    Eigen::MatrixXd A;
    A = SRukfInterface::choleskyDecomposition(P);
    return qrDecompositionJustR(A);
}

/*! Perform the cholesky decomposition of an input matrix using Eigen's LLT
 @return Eigen::MatrixXd
 @param Eigen::MatrixXd input : The input matrix
 */
Eigen::MatrixXd SRukfInterface::choleskyDecomposition(const Eigen::MatrixXd input) const
{
    Eigen::LLT<Eigen::MatrixXd> choleskyDecomp(input);
    return choleskyDecomp.matrixL();
}

/*! Perform a generic back substitution to solve x in Ux = b
 @return Eigen::MatrixXd
 @param Eigen::MatrixXd U, an upper triangular matrix
 @param Eigen::MatrixXd b, the right hand side of the Ux = b
 */
Eigen::MatrixXd SRukfInterface::backSubstitution(const Eigen::MatrixXd U, const Eigen::MatrixXd b) const
{
    assert(U.rows() == b.rows());

    Eigen::MatrixXd x;
    Eigen::VectorXd xCol;

    x.setZero(b.rows(), b.cols());
    for (int col =0; col < b.cols(); ++col){
        xCol.setZero(b.rows());
        for (long i = U.rows()-1; i >= 0; --i){
            xCol(i) = b(i, col);
            for (long j = i+1 ; j < U.rows(); ++j){
                xCol(i) = xCol(i) - U(i,j)*xCol(j);
            }
            xCol(i) = xCol(i)/U(i,i);
        }
        x.col(col) = xCol;
    }

    return x;
}

/*! Perform a generic forward substitution to solve x in Lx = b
 @return Eigen::MatrixXd
 @param Eigen::MatrixXd L, an lower triangular matrix
 @param Eigen::MatrixXd b, the right hand side of the Ux = b
 */
Eigen::MatrixXd SRukfInterface::forwardSubstitution(const Eigen::MatrixXd L, const Eigen::MatrixXd b) const
{
    assert(L.rows() == b.rows());

    Eigen::MatrixXd x;
    Eigen::VectorXd xCol;

    x.setZero(b.rows(), b.cols());
    for (int col =0; col < b.cols(); ++col){
        xCol.setZero(b.rows());
        for (int i =0; i < L.rows(); ++i){
            xCol(i) = b(i, col);
            for (int j = 0 ; j < i; ++j){
                xCol(i) = xCol(i) - L(i,j)*xCol(j);
            }
            xCol(i) = xCol(i)/L(i,i);
        }
        x.col(col) = xCol;
    }
    return x;
}


/*! Runge-Kutta 4 (RK4) function for state propagation
    @param ODEfunction function handle that includes the equations of motion
    @param X0 initial state
    @param t0 initial time
    @param dt time step
    @return Eigen::VectorXd
*/
Eigen::VectorXd SRukfInterface::rk4(const std::function<Eigen::VectorXd(double, Eigen::VectorXd)>& ODEfunction,
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

/*! Set the filter alpha parameter
    @param double alphaInput
    @return void
    */
void SRukfInterface::setAlpha(const double alphaInput){
    this->alpha = alphaInput;
}

/*! Get the filter alpha parameter
    @return double alpha
    */
double SRukfInterface::getAlpha() const {
    return this->alpha;
}

/*! Set the filter beta parameter
    @param double betaInput
    @return void
    */
void SRukfInterface::setBeta(const double betaInput){
    this->beta = betaInput;
}

/*! Get the filter beta parameter
    @return double beta
    */
double SRukfInterface::getBeta() const {
    return this->beta;
}

/*! Set the filter initial state vector
    @param Eigen::VectorXd initialStateInput
    @return void
    */
void SRukfInterface::setInitialState(const Eigen::VectorXd initialStateInput){
    this->stateInitial.resize(initialStateInput.size());
    this->stateInitial << initialStateInput;
}

/*! Get the filter initial state vector
    @return Eigen::VectorXd stateInitial
    */
Eigen::VectorXd SRukfInterface::getInitialState() const {
    return this->stateInitial;
}

/*! Set the filter initial state covariance
    @param Eigen::MatrixXd initialCovarianceInput
    @return void
    */
void SRukfInterface::setInitialCovariance(const Eigen::MatrixXd initialCovarianceInput){
    this->covarInitial.resize(initialCovarianceInput.rows(), initialCovarianceInput.cols());
    this->covarInitial << initialCovarianceInput;
}

/*! Get the filter initial state covariance
    @return Eigen::MatrixXd covarInitial
    */
Eigen::MatrixXd SRukfInterface::getInitialCovariance() const {
    return this->covarInitial;
}

/*! Set the filter process noise
    @param Eigen::MatrixXd processNoiseInput
    @return void
    */
void SRukfInterface::setProcessNoise(const Eigen::MatrixXd processNoiseInput){
    this->processNoise.resize(processNoiseInput.rows(), processNoiseInput.cols());
    this->processNoise << processNoiseInput;
}

/*! Get the filter process noise
    @return Eigen::MatrixXd processNoise
    */
Eigen::MatrixXd SRukfInterface::getProcessNoise() const {
    return this->processNoise;
}

/*! Set a unit conversion factor, for instance if desirable to solve for a state in km in the filter, but Basilisk's
 * outside facing interface is in SI
    @param double conversion
    */
void SRukfInterface::setUnitConversionFromSItoState(const double conversion){
    this->unitConversion = conversion;
}

/*! Get a unit conversion factor, for instance if desirable to solve for a state in km in the filter, but Basilisk's
 * outside facing interface is in SI
    @return double unitConversion
    */
double SRukfInterface::getUnitConversionFromSItoState() const{
    return this->unitConversion;
}
