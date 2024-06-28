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

#include "inertialAttitudeUkf.h"

InertialAttitudeUkf::InertialAttitudeUkf(AttitudeFilterMethod method){
    this->measurementAcceptanceMethod = method;
}

InertialAttitudeUkf::~InertialAttitudeUkf() = default;


void InertialAttitudeUkf::customReset(){
    /*! No custom reset for this module */
}

/*! Before every update, check the MRP norm for a shadow set switch
 @return void
 */
void InertialAttitudeUkf::customInitializeUpdate(){
    this->state.head(3) << mrpSwitch(this->state.head(3), this->mrpSwitchThreshold);
}

/*! After every update, check the MRP norm for a shadow set switch
 @return void
 */
void InertialAttitudeUkf::customFinalizeUpdate(){
    this->state.head(3) << mrpSwitch(this->state.head(3), this->mrpSwitchThreshold);
}

/*! Read the message containing the measurement data.
 * It updates class variables relating to measurement data including validity and time tags.
 @return void
 */
void InertialAttitudeUkf::writeOutputMessages(uint64_t CurrentSimNanos) {
    NavAttMsgPayload navAttPayload = this->navAttitudeOutputMsg.zeroMsgPayload;
    FilterMsgPayload filterPayload = this->inertialFilterOutputMsg.zeroMsgPayload;
    FilterResidualsMsgPayload starTrackerPayload = this->starTrackerResidualMsg.zeroMsgPayload;
    FilterResidualsMsgPayload gyroPayload = this->gyroResidualMsg.zeroMsgPayload;

    /*! - Write the flyby OD estimate into the copy of the navigation message structure*/
    navAttPayload.timeTag = this->previousFilterTimeTag;
    eigenMatrixXd2CArray(this->state.head(3), navAttPayload.sigma_BN);
    eigenMatrixXd2CArray(this->state.tail(3), navAttPayload.omega_BN_B);

    /*! - Populate the filter states output buffer and write the output message*/
    filterPayload.timeTag = this->previousFilterTimeTag;
    eigenMatrixXd2CArray(this->state, filterPayload.state);
    eigenMatrixXd2CArray(this->xBar, filterPayload.stateError);
    eigenMatrixXd2CArray(this->covar, filterPayload.covar);

    for (size_t index = 0; index < MAX_MEASUREMENT_NUMBER; index ++){
        if (this->measurements[index].has_value()) {
            auto measurement = this->measurements[index].value();
            if (measurement.name == "starTracker"){
                starTrackerPayload.valid = true;
                starTrackerPayload.numberOfObservations = 1;
                starTrackerPayload.sizeOfObservations = measurement.observation.size();
                eigenMatrixXd2CArray(measurement.observation, &starTrackerPayload.observation[0]);
                eigenMatrixXd2CArray(measurement.postFitResiduals, &starTrackerPayload.postFits[0]);
                eigenMatrixXd2CArray(measurement.preFitResiduals, &starTrackerPayload.preFits[0]);
                }
            if (measurement.name == "gyro"){
                gyroPayload.valid = true;
                gyroPayload.numberOfObservations = 1;
                gyroPayload.sizeOfObservations = measurement.observation.size();
                eigenMatrixXd2CArray(measurement.observation, &gyroPayload.observation[0]);
                eigenMatrixXd2CArray(measurement.postFitResiduals, &gyroPayload.postFits[0]);
                eigenMatrixXd2CArray(measurement.preFitResiduals, &gyroPayload.preFits[0]);
                }
            this->measurements[index].reset();
        }
    }
    this->starTrackerResidualMsg.write(&starTrackerPayload, this->moduleID, CurrentSimNanos);
    this->gyroResidualMsg.write(&gyroPayload, this->moduleID, CurrentSimNanos);

    this->navAttitudeOutputMsg.write(&navAttPayload, this->moduleID, CurrentSimNanos);
    this->inertialFilterOutputMsg.write(&filterPayload, this->moduleID, CurrentSimNanos);
}

/*! Read current RW speends and populate the accelerations in order to propagate
* @return void
* */
void InertialAttitudeUkf::readRWSpeedData(){
    RWSpeedMsgPayload rwSpeedPayload = this->rwSpeedMsg();
    uint64_t wheelSpeedTime = this->rwSpeedMsg.timeWritten();
    if (this->firstFilterPass){
        this->wheelAccelerations << 0,0,0,0;
        this->previousWheelSpeeds = Eigen::Map<Eigen::Matrix<double, 1, 4>>(rwSpeedPayload.wheelSpeeds);
        this->previousWheelSpeedTime = wheelSpeedTime*NANO2SEC;
    }
    else{
        double dt = wheelSpeedTime*NANO2SEC - this->previousWheelSpeedTime;
        this->wheelAccelerations = (Eigen::Map<Eigen::Matrix<double, 1, 4>>(rwSpeedPayload.wheelSpeeds) - this->previousWheelSpeeds)/dt;
    }
}

/*! Loop through the all the input star trackers and populate their measurement container if they are foward
 * in time
* @return void
* */
void InertialAttitudeUkf::readStarTrackerData(){
    for (int index = 0; index < this->numberOfStarTackers; index ++){
        auto starTracker = this->starTrackerMessages[index].starTrackerMsg();
        if (starTracker.timeTag*NANO2SEC > this->previousFilterTimeTag){
            auto starTrackerMeasurement = Measurement();
            starTrackerMeasurement.name = "starTracker";
            starTrackerMeasurement.timeTag = starTracker.timeTag*NANO2SEC;
            starTrackerMeasurement.validity = true;
            starTrackerMeasurement.size = 3;

            starTrackerMeasurement.noise = this->starTrackerMessages[index].measurementNoise;
            starTrackerMeasurement.observation = mrpSwitch(Eigen::Map<Eigen::Vector3d>(starTracker.MRP_BdyInrtl),
                    this->mrpSwitchThreshold);
            starTrackerMeasurement.model = mrpFirstThreeStates;
            this->measurements[this->measurementIndex] = starTrackerMeasurement;
            this->measurementIndex += 1;
            this->validStarTracker = true;
            /*! - Only consider the filter started once a Star Tracker image is processed */
            if (this->firstFilterPass){this->firstFilterPass = false;}
        }
        else{this->validStarTracker=false;}
    }
}

/*! Loop through the entire gyro buffer to find the first index that is in the future compared to the
* previousFilterTimeTag. This does not assume the data comes in chronological order since the gyro data
* is a ring buffer and can wrap around
* @return void
* */
void InertialAttitudeUkf::readGyroData(){
    int smallestFutureIndex = 0;
    int numberOfValidGyroMeasurements = 0;
    double firstFutureTime = -1;
    double meanMeasurementTime = 0;
    AccDataMsgPayload gyrBuffer = this->accelDataMsg();
    for (int index = 0; index < MAX_ACC_BUF_PKT; index++) {
        double gyroMeasuredTime = gyrBuffer.accPkts[index].measTime*NANO2SEC;
        if (gyroMeasuredTime > this->previousFilterTimeTag) {
            if (gyroMeasuredTime < firstFutureTime || firstFutureTime<0){
                smallestFutureIndex = index;
                firstFutureTime = gyroMeasuredTime;
            }
            meanMeasurementTime += gyroMeasuredTime;
            numberOfValidGyroMeasurements += 1;
        }
    }
    auto lowPass = LowPassFilter();
    lowPass.setFilterCutoff(this->cutOffFrequency);
    lowPass.setFilterStep(this->hStep);
    if (numberOfValidGyroMeasurements > 0){
        meanMeasurementTime /= numberOfValidGyroMeasurements;
        /*! - Loop through buffer for all future measurements since the previous time to filter omega_BN_B*/
        for (int index = 0; index < MAX_ACC_BUF_PKT; index++) {
            int shiftedIndex = (index + smallestFutureIndex) % MAX_ACC_BUF_PKT;
            auto omega_BN_B = Eigen::Map<Eigen::Vector3d>(gyrBuffer.accPkts[shiftedIndex].gyro_B);
            /*! - Apply low-pass filter to gyro measurements to get smoothed body rate*/
            lowPass.processMeasurement(omega_BN_B);
        }

        auto gyroMeasurement = Measurement();
        gyroMeasurement.name = "gyro";
        gyroMeasurement.timeTag = meanMeasurementTime;
        gyroMeasurement.validity = true;
        gyroMeasurement.size = 3;

        gyroMeasurement.noise = this->gyroNoise/std::sqrt(numberOfValidGyroMeasurements);
        gyroMeasurement.observation = lowPass.getCurrentState();
        gyroMeasurement.model = lastThreeStates;
        this->measurements[this->measurementIndex] = gyroMeasurement;
        this->measurementIndex += 1;
    }
}

/*! Read the message containing the measurement data.
 * It updates class variables relating to measurement data including validity and time tags.
 @return void
 */
void InertialAttitudeUkf::readFilterMeasurements() {
    /*! - Read static RW and spacecraft config data message and store it in module fields */
    if (this->firstFilterPass) {
        this->rwArrayConfigPayload = this->rwArrayConfigMsg();
        auto vehicleConfigPayload = this->vehicleConfigMsg();

        this->spacecraftInertia = cArray2EigenMatrix3d(vehicleConfigPayload.ISCPntB_B);
        this->spacecraftInertiaInverse = this->spacecraftInertia.inverse();
    }

    this->measurementIndex = 0;
    /*! - Read in wheel speeds, their time, and compute the wheel accelerations for the propagation method*/
    readRWSpeedData();
    /*! - Read star tracker measurements*/
    readStarTrackerData();
    /*! Only add the gyro measurements to processing if the filter is in a mode that desires that */
    if (this->measurementAcceptanceMethod == AttitudeFilterMethod::AllMeasurements) {
        readGyroData();
    }
    if (measurementAcceptanceMethod == AttitudeFilterMethod::GyroWhenDazzled && !this->validStarTracker) {
        readGyroData();
    }
}

/*! Integrate the equations of motion of two body point mass gravity using Runge-Kutta 4 (RK4)
    @param interval integration interval
    @param X0 initial state
    @param dt time step
    @return Eigen::VectorXd
*/
Eigen::VectorXd InertialAttitudeUkf::propagate(std::array<double, 2> interval, const Eigen::VectorXd& X0, double dt){
    double t_0 = interval[0];
    double t_f = interval[1];
    double t = t_0;
    Eigen::VectorXd X = X0;

    std::function<Eigen::VectorXd(double, Eigen::VectorXd)> attitudeDynamics = [this](double t, Eigen::VectorXd state)
    {
        Eigen::Vector3d mrp(state.head(3));
        Eigen::Vector3d omega(state.tail(3));
        Eigen::MatrixXd Bmat = bmatMrp(mrp);

        Eigen::VectorXd stateDerivative(state.size());
        stateDerivative.head(3) << 0.25*Bmat*omega;

        Eigen::Vector3d wheelTorque = Eigen::Vector3d::Zero();
        for(int i=0; i<this->rwArrayConfigPayload.numRW; i++){
            Eigen::Vector3d GsMatrix = Eigen::Map<Eigen::Vector3d>(&this->rwArrayConfigPayload.GsMatrix_B[i*3]);
            wheelTorque -= this->wheelAccelerations[i]*this->rwArrayConfigPayload.JsList[i]*GsMatrix;
        }

        stateDerivative.tail(3) << -this->spacecraftInertiaInverse*(tildeMatrix(omega)*this->spacecraftInertia*omega + wheelTorque);

        return stateDerivative;
    };

    /*! Propagate to t_final with an RK4 integrator */
    double N = ceil((t_f-t_0)/dt);
    for (int c=0; c < N; c++) {
        double step = std::min(dt,t_f-t);
        X = this->rk4(attitudeDynamics, X, t, step);
        t = t + step;
    }

    return X;
}

/*! Set the gyro measurement noise matrix
    @param Eigen::Matrix3d gyroNoise
    @return void
    */
void InertialAttitudeUkf::setGyroNoise(const Eigen::Matrix3d &gyroNoiseInput) {
    this->gyroNoise = gyroNoiseInput;
}

/*! Get he gyro measurement noise matrix
    @return Eigen::Matrix3d gyroNoise
    */
Eigen::Matrix3d InertialAttitudeUkf::getGyroNoise() const {
    return this->gyroNoise;
}

/*! Add a star tracker to the filter solution using the StarTrackerMessage class
    @return StarTrackerMessage starTracker
    */
void InertialAttitudeUkf::addStarTrackerInput(const StarTrackerMessage &starTracker){
    this->starTrackerMessages[this->numberOfStarTackers] = starTracker;
    this->numberOfStarTackers += 1;
}

/*! Get the star tracker measurement noise matrix for a particular number (indexed at 0)
    @param int starTrackerNumber
    @return Eigen::Matrix3d starTrackerMeasurementNoise
    */
Eigen::Matrix3d InertialAttitudeUkf::getStarTrackerNoise(int starTrackerNumber) const {
    assert(starTrackerNumber < this->numberOfStarTackers);
    return this->starTrackerMessages[starTrackerNumber].measurementNoise;
}

/*! Set the low pass filter parameters for the
    @param double step
    @param double frequencyCutOff
    */
void InertialAttitudeUkf::setLowPassFilter(double step, double frequencyCutOff){
    this->hStep = step;
    this->cutOffFrequency = frequencyCutOff;
}
