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
void FlybyODuKF::Reset(uint64_t CurrentSimNanos)
{
    /*! - Check if the required message has not been connected */
    if (!this->opNavHeadingMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR,  "Error: flybyODuKF opNavUnitVecInMsg wasn't connected.");
    }

    /*! - Initialize filter parameters and change units to km and s */
    this->muCentral *= 1E-9; // mu is input in meters
    this->state = 1E-3 * this->stateInitial;
    this->sBar = 1E-6 * this->covarInitial;
    this->covar = 1E-6 * this->covarInitial;
    this->covar.resize(this->state.size(), this->state.size());
    this->sBar.resize(this->state.size(), this->state.size());
    this->measurementNoise.resize(this->obs.size(), this->obs.size());
    this->processNoise.resize(this->state.size(), this->state.size());

    this->previousFilterTimeTag = (double) CurrentSimNanos*NANO2SEC;
    this->numberSigmaPoints = this->state.size()*2+1;
    this->dt = 0;

    /*! - Ensure that all internal filter matrices are zeroed*/
    this->obs.setZero(this->state.size());
    this->wM.setZero(this->numberSigmaPoints);
    this->wC.setZero(this->numberSigmaPoints);
    this->sBar.setZero(this->state.size(), this->state.size());
    this->sigmaPoints.setZero(this->state.size(), this->numberSigmaPoints);
    this->cholProcessNoise.setZero(this->state.size(), this->state.size());

    /*! - Set lambda/gamma to standard value for unscented kalman filters */
    this->lambdaParameter = (double) this->state.size()*(this->alphaParameter*this->alphaParameter - 1);
    this->etaParameter = sqrt((double) this->state.size() + this->lambdaParameter);

    /*! - Set the wM/wC vectors to standard values for unscented kalman filters*/
    this->wM(0) = this->lambdaParameter / ((double) this->state.size() + this->lambdaParameter);
    this->wC(0) = this->lambdaParameter / ((double) this->state.size() + this->lambdaParameter) +
            (1 - this->alphaParameter*this->alphaParameter + this->betaParameter);
    for (size_t i = 1; i < this->numberSigmaPoints; i++)
    {
        this->wM(i) = 1.0 / (2.0 * ((double) this->state.size() + this->lambdaParameter));
        this->wC(i) = this->wM(i);
    }

    /*! - Perform cholesky decompositions of covariance and noise */
    this->sBar = FlybyODuKF::choleskyDecomposition(this->covar);
    this->cholProcessNoise = FlybyODuKF::choleskyDecomposition(this->processNoise * 1E-6);
}

/*! Take the relative position measurements and outputs an estimate of the
 spacecraft states in the inertial frame.
 @return void
 @param CurrentSimNanos The clock time at which the function was called (nanoseconds)
 */
void FlybyODuKF::UpdateState(uint64_t CurrentSimNanos)
{
    this->readFilterMeasurements();
    this->computePostFits = false;
    /*! - If the time tag from the measured data is new compared to previous step,
    propagate and update the filter*/
    if(this->opNavHeadingBuffer.timeTag * NANO2SEC >= this->previousFilterTimeTag && this->opNavHeadingBuffer.valid)
    {
        this->timeUpdate(this->opNavHeadingBuffer.timeTag * NANO2SEC);
        this->measurementUpdate();
        this->computePostFitResiudals();
    }
    /*! - If current clock time is further ahead than the measured time, then
     propagate to this current time-step*/
    if((double) CurrentSimNanos*NANO2SEC >= this->previousFilterTimeTag)
    {
        this->timeUpdate((double) CurrentSimNanos * NANO2SEC);
    }

    this->writeOutputMessages(CurrentSimNanos);
}

/*! Perform the time update for the flyby OD kalman filter.
 It propagates the sigma points forward in time and then gets the current
 covariance and state estimates.
 @return void
 @param updateTime The time that we need to fix the filter to (seconds)
 */
void FlybyODuKF::timeUpdate(double updateTime)
{
    Eigen::VectorXd propagatedSigmaPoint;
    Eigen::MatrixXd A(this->state.size(), 3*this->state.size());

    this->dt = updateTime - this->previousFilterTimeTag;
    std::array<double, 2> time = {0, this->dt};

    /*! - Copy over the current state estimate into the 0th Sigma point and propagate by dt*/
    this->sigmaPoints.col(0) = propagate(time, this->state, this->dt);
    /*! - Scale that Sigma point by the appopriate scaling factor (Wm[0])*/
    this->xBar = this->wM(0)*this->sigmaPoints.col(0);

    /*! - For each Sigma point, apply sBar-based error, propagate forward, and scale by Wm just like 0th.
     Note that we perform +/- sigma points simultaneously in loop to save loop values.*/
    for (size_t i = 1; i<this->state.size() + 1; i++)
    {
        /*! - Adding covariance columns from sigma points*/
        this->sigmaPoints.col(i) = propagate(time, this->state + this->etaParameter * this->sBar.col(i-1), this->dt);
        /*! - Subtracting covariance columns from sigma points*/
        this->sigmaPoints.col( i + this->state.size()) =
                propagate(time, this->state - this->etaParameter * this->sBar.col(i-1), this->dt);
    }

    /*! - Compute xbar according to Eq (19)*/
    for (size_t i = 1; i<this->numberSigmaPoints; i++)
    {
        this->xBar += this->wM(i)*this->sigmaPoints.col(i);
    }

    /*! - Assemble the A matrix for QR decomposition as seen in equation 20 in the reference document*/
    for (size_t i = 1; i < this->numberSigmaPoints; i++)
    {
        A.col(i-1) = sqrt(this->wC(i))*(this->sigmaPoints.col(i) - this->xBar);
    }

    A.block(0, this->numberSigmaPoints-1, this->state.size(), this->state.size()) =
            this->cholProcessNoise;

    /*! - QR decomposition (only R is of interest) of the A matrix provides the new sBar matrix*/
    this->sBar = FlybyODuKF::qrDecompositionJustR(A);

    /*! - Shift the sBar matrix over by the xBar vector using the appropriate weight
     like in equation 21 in design document.*/
    Eigen::VectorXd xError = this->sigmaPoints.col(0) - this->xBar;

    /*! - Cholesky update block for vectors.*/
    this->sBar = FlybyODuKF::choleskyUpDownDate(this->sBar, xError, this->wC(0));

    this->covar = this->sBar*this->sBar.transpose();
    this->state = this->sigmaPoints.col(0);
    this->previousFilterTimeTag = updateTime;
}


/*! Read the message containing the measurement data.
 * It updates class variables relating to measurement data including validity and time tags.
 @return void
 */
void FlybyODuKF::writeOutputMessages(uint64_t CurrentSimNanos) {
    this->opNavFilterMsgBuffer = this->opNavFilterMsg.zeroMsgPayload;
    this->opNavResidualMsgBuffer = this->opNavResidualMsg.zeroMsgPayload;
    this->navTransOutMsgBuffer = this->navTransOutMsg.zeroMsgPayload;

    /*! - Write the flyby OD estimate into the copy of the navigation message structure*/
    eigenMatrixXd2CArray(1e3*this->state.head(3), this->navTransOutMsgBuffer.r_BN_N);
    eigenMatrixXd2CArray(1e3*this->state.tail(3), this->navTransOutMsgBuffer.v_BN_N);

    /*! - Populate the filter states output buffer and write the output message*/
    this->opNavFilterMsgBuffer.timeTag = this->previousFilterTimeTag;
    eigenMatrixXd2CArray( 1e3*this->state, this->opNavFilterMsgBuffer.state);
    eigenMatrixXd2CArray( 1e3*this->xBar, this->opNavFilterMsgBuffer.stateError);
    eigenMatrixXd2CArray(1e6*this->covar, this->opNavFilterMsgBuffer.covar);

    if (this->computePostFits){
        eigenMatrixXd2CArray(this->postFits, this->opNavResidualMsgBuffer.postFits);
    }

    this->navTransOutMsg.write(&this->navTransOutMsgBuffer, this->moduleID, CurrentSimNanos);
    this->opNavFilterMsg.write(&this->opNavFilterMsgBuffer, this->moduleID, CurrentSimNanos);
    this->opNavResidualMsg.write(&this->opNavResidualMsgBuffer, this->moduleID, CurrentSimNanos);
}

/*! Read the message containing the measurement data.
 * It updates class variables relating to measurement data including validity and time tags.
 @return void
 */
void FlybyODuKF::readFilterMeasurements() {
    this->opNavHeadingBuffer = this->opNavHeadingMsg();

    if (this->opNavHeadingBuffer.valid){
        /*! - Read measurement and cholesky decomposition its noise*/
        this->obs = cArray2EigenVector3d(this->opNavHeadingBuffer.rhat_BN_N);
        this->obs.normalize();
        this->measurementNoise = this->measNoiseScaling * cArray2EigenMatrixXd(this->opNavHeadingBuffer.covar_N,
                                                                               (int) this->obs.size(),
                                                                               (int) this->obs.size());
        this->cholMeasurementNoise = FlybyODuKF::choleskyDecomposition(this->measurementNoise);
    }
}

/*! Compute the measurement model.  Given that the data is coming from
 the center of brightness Converter, the transformation has already taken place from pixel data
 to spacecraft position.
 @return void
 */
void FlybyODuKF::measurementModel()
{
    this->yMeas.setZero(3, this->numberSigmaPoints);
    for(size_t j=0; j < this->numberSigmaPoints; j++)
    {
        /*! Sigma points positions need to be normalized for the measurement model.*/
        this->yMeas.col(j) = this->sigmaPoints.col(j).head(3).normalized();
    }
}

/*! Compute the post fit residuals if the measurement data was fresh.
 * The post fits are y - ybar if a measurement was read, if observations are not present
 * a flag is raised to not compute post fit residuals
@return void
 */
void FlybyODuKF::computePostFitResiudals()
{
    /*! - Compute Post Fit Residuals, first get Y (eq 22) using the states post fit*/
    this->measurementModel();
    /*! - Compute the value for the yBar parameter (equation 23)*/
    this->postFits.setZero(this->obs.size());
    Eigen::VectorXd yBar;
    yBar.setZero(this->obs.size());
    for(size_t i=0; i<this->numberSigmaPoints; i++){
        yBar += this->wM(i)*this->yMeas.col(i);
    }
    this->postFits = this->obs - yBar;
    this->computePostFits = true;
}

/*! Perform the measurement update for the kalman filter.
 It applies the observations in the obs vectors to the current state estimate and
 updates the state/covariance with that information.
 @return void
 */
void FlybyODuKF::measurementUpdate()
{
    /*! - Compute the valid observations and the measurement model for all observations*/
    this->measurementModel();
    
    /*! - Compute the value for the yBar parameter (note that this is equation 23 in the
     time update section of the reference document*/
    Eigen::VectorXd yBar;
    yBar.setZero(this->obs.size());
    for(size_t i=0; i<this->numberSigmaPoints; i++)
    {
        yBar += this->wM(i) * this->yMeas.col(i);
    }
    
    /*! - Populate the matrix that we perform the QR decomposition on in the measurement
     update section of the code.  This is based on the difference between the yBar
     parameter and the calculated measurement models.  Equation 24. */
    Eigen::MatrixXd A(this->obs.size(), 2*this->state.size() + this->obs.size());
    for(size_t i=1; i<this->numberSigmaPoints; i++)
    {
        A.col(i-1) = sqrt(this->wC(1))*(yMeas.col(i) - yBar);
    }
    A.block(0, this->numberSigmaPoints-1, this->obs.size(), this->obs.size()) =
            this->cholMeasurementNoise;

    /*! - Perform QR decomposition (only R again) of the above matrix to obtain the
     current Sy matrix*/
    Eigen::MatrixXd sy;
    sy.setZero(this->obs.size(), this->obs.size());
    sy = FlybyODuKF::qrDecompositionJustR(A);

    /*! - Cholesky update block for vectors.*/
    Eigen::VectorXd yError = this->yMeas.col(0) - yBar;
    sy = FlybyODuKF::choleskyUpDownDate(sy, yError, this->wC(0));

    /*! - Construct the Pxy matrix (equation 26) which multiplies the Sigma-point cloud
     by the measurement model cloud (weighted) to get the total Pxy matrix*/
    Eigen::VectorXd xError;
    xError.setZero(this->state.size());
    Eigen::MatrixXd kMat;
    kMat.setZero(this->state.size(), this->obs.size());
    Eigen::MatrixXd STkMatT;
    STkMatT.setZero(this->obs.size(), this->state.size());
    Eigen::MatrixXd pXY;
    pXY.setZero(this->state.size(), this->obs.size());

    for(size_t i=0; i<this->numberSigmaPoints; i++)
    {
        xError = this->sigmaPoints.col(i) - this->xBar;
        yError = this->yMeas.col(i) - yBar;
        kMat =  this->wC(i) * xError * yError.transpose();
        pXY += kMat;
    }

    /*! - Then we need to invert the SyT*Sy matrix to get the Kalman gain factor.  Since
     The Sy matrix is lower triangular, we can do a back-sub inversion instead of
     a full matrix inversion. Equation 27 in the reference document.*/
    STkMatT = FlybyODuKF::forwardSubstitution(sy, pXY.transpose());
    kMat = FlybyODuKF::backSubstitution(sy.transpose(), STkMatT).transpose();

    /*! - Difference the yBar and the observations to get the observed error and
     multiply by the Kalman Gain to get the state update.  Add the state update
     to the state to get the updated state value (equation 27).*/
    this->state = this->xBar + kMat*(this->obs - yBar);

    /*! - Compute the updated matrix U from equation 28 */
    Eigen::MatrixXd Umat;
    Umat.setZero(this->state.size(), this->obs.size());
    Umat = kMat * sy;

    /*! - For each column in the update matrix, perform a cholesky down-date on it to
     get the total shifted S matrix (called sBar in internal parameters*/
    for(int i=0; i < Umat.cols(); i++)
    {
        this->sBar = FlybyODuKF::choleskyUpDownDate(this->sBar, Umat.col(i), -1);
    }

    /*! - Compute equivalent covariance based on updated sBar matrix*/
    this->covar = this->sBar*this->sBar.transpose();
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

/*! Runge-Kutta 4 (RK4) function for the state propagation
    @param ODEfunction function handle that includes the equations of motion
    @param X0 initial state
    @param t0 initial time
    @param dt time step
    @return Eigen::VectorXd
*/
Eigen::VectorXd FlybyODuKF::rk4(const std::function<Eigen::VectorXd(double, Eigen::VectorXd)>& ODEfunction,
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

/*! Perform a QR decomposition using HouseholderQR from Eigen, but only returns the transpose of the
 * upper triangular R matrix truncated such that it is the same size as the input matrix.
 @return Eigen::MatrixXd
 @param Eigen::MatrixXd input : The input matrix. If not square, provide it with more cols then rows
 */
Eigen::MatrixXd FlybyODuKF::qrDecompositionJustR(const Eigen::MatrixXd input) const
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
    for (int i =0; i < R_tilde.rows(); i ++){
        for (int j = 0 ; j < i; j ++){
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
Eigen::MatrixXd FlybyODuKF::choleskyUpDownDate(const Eigen::MatrixXd input,
                                               const Eigen::VectorXd inputVector,
                                               const double coefficient) const
{
    Eigen::MatrixXd P;
    P.setZero(inputVector.size(), inputVector.size());

    /*! Perform the Cholesky factor updating.
     * Math is described in the second bullet of section 3 page 3 of the reference document */
    P = input * input.transpose();
    P += ((coefficient > 0) - (coefficient < 0))*abs(coefficient)*inputVector*inputVector.transpose();

    Eigen::MatrixXd A;
    A = FlybyODuKF::choleskyDecomposition(P);
    return qrDecompositionJustR(A);
}

/*! Perform the cholesky decomposition of an input matrix using Eigen's LLT
 @return Eigen::MatrixXd
 @param Eigen::MatrixXd input : The input matrix
 */
Eigen::MatrixXd FlybyODuKF::choleskyDecomposition(const Eigen::MatrixXd input) const
{
    Eigen::LLT<Eigen::MatrixXd> choleskyDecomp(input);
    return choleskyDecomp.matrixL();
}

/*! Perform a generic back substitution to solve x in Ux = b
 @return Eigen::MatrixXd
 @param Eigen::MatrixXd U, an upper triangular matrix
 @param Eigen::MatrixXd b, the right hand side of the Ux = b
 */
Eigen::MatrixXd FlybyODuKF::backSubstitution(const Eigen::MatrixXd U, const Eigen::MatrixXd b) const
{
    assert(U.rows() == b.rows());

    Eigen::MatrixXd x;
    Eigen::VectorXd xCol;

    x.setZero(b.rows(), b.cols());
    for (int col =0; col < b.cols(); col++){
        xCol.setZero(b.rows());
        for (long i = U.rows()-1; i >= 0; i--){
            xCol(i) = b(i, col);
            for (long j = i+1 ; j < U.rows(); j++){
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
Eigen::MatrixXd FlybyODuKF::forwardSubstitution(const Eigen::MatrixXd L, const Eigen::MatrixXd b) const
{
    assert(L.rows() == b.rows());

    Eigen::MatrixXd x;
    Eigen::VectorXd xCol;

    x.setZero(b.rows(), b.cols());
    for (int col =0; col < b.cols(); col++){
        xCol.setZero(b.rows());
        for (int i =0; i < L.rows(); i++){
            xCol(i) = b(i, col);
            for (int j = 0 ; j < i; j++){
                xCol(i) = xCol(i) - L(i,j)*xCol(j);
            }
            xCol(i) = xCol(i)/L(i,i);
        }
        x.col(col) = xCol;
    }

    return x;
}
