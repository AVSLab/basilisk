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
    std::function<StateVector(double, const StateVector)> attitudeDynamics = [this](double t, const StateVector &state){
        Eigen::Vector3d mrp(state.getPositionStates());
        Eigen::Vector3d omega(state.getVelocityStates());
        Eigen::MatrixXd bMat = bmatMrp(mrp);

        StateVector stateDerivative;
        PositionState mrpDot;
        mrpDot.setValues(0.25*bMat*omega);
        stateDerivative.setPosition(mrpDot);

        Eigen::Vector3d wheelTorque = Eigen::Vector3d::Zero();
        for(int i=0; i<this->rwArrayConfigPayload.numRW; i++){
            Eigen::Vector3d gsMatrix = Eigen::Map<Eigen::Vector3d>(&this->rwArrayConfigPayload.GsMatrix_B[i*3]);
            wheelTorque -= this->wheelAccelerations[i]*this->rwArrayConfigPayload.JsList[i]*gsMatrix;
        }

        VelocityState omegaDot;
        omegaDot.setValues(-this->spacecraftInertiaInverse*(tildeMatrix(omega)*this->spacecraftInertia*omega + wheelTorque));
        stateDerivative.setVelocity(omegaDot);

        return stateDerivative;
    };
    this->dynamics.setDynamics(attitudeDynamics);
}

/*! Before every update, check the MRP norm for a shadow set switch
 @return void
 */
void InertialAttitudeUkf::customInitializeUpdate(){
    PositionState mrp;
    mrp.setValues(mrpSwitch(this->state.getPositionStates(), this->mrpSwitchThreshold));
    this->state.setPosition(mrp);
}

/*! After every update, check the MRP norm for a shadow set switch
 @return void
 */
void InertialAttitudeUkf::customFinalizeUpdate(){
    PositionState mrp;
    mrp.setValues( mrpSwitch(this->state.getPositionStates(), this->mrpSwitchThreshold));
    this->state.setPosition(mrp);
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
    eigenMatrixXd2CArray(this->state.getPositionStates(), navAttPayload.sigma_BN);
    eigenMatrixXd2CArray(this->state.getVelocityStates(), navAttPayload.omega_BN_B);

    /*! - Populate the filter states output buffer and write the output message*/
    filterPayload.timeTag = this->previousFilterTimeTag;
    eigenMatrixXd2CArray(this->state.returnValues(), filterPayload.state);
    eigenMatrixXd2CArray(this->xBar.returnValues(), filterPayload.stateError);
    eigenMatrixXd2CArray(this->covar, filterPayload.covar);

    for (size_t index = 0; index < MAX_MEASUREMENT_NUMBER; index ++){
        if (this->measurements[index].has_value()) {
            auto measurement = this->measurements[index].value();
            if (measurement.getMeasurementName() == "starTracker"){
                starTrackerPayload.valid = true;
                starTrackerPayload.numberOfObservations = 1;
                starTrackerPayload.sizeOfObservations = measurement.size();
                eigenMatrixXd2CArray(measurement.getObservation(), &starTrackerPayload.observation[0]);
                eigenMatrixXd2CArray(measurement.getPostFitResiduals(), &starTrackerPayload.postFits[0]);
                eigenMatrixXd2CArray(measurement.getPreFitResiduals(), &starTrackerPayload.preFits[0]);
                }
            if (measurement.getMeasurementName() == "gyro"){
                gyroPayload.valid = true;
                gyroPayload.numberOfObservations = 1;
                gyroPayload.sizeOfObservations = measurement.size();
                eigenMatrixXd2CArray(measurement.getObservation(), &gyroPayload.observation[0]);
                eigenMatrixXd2CArray(measurement.getPostFitResiduals(), &gyroPayload.postFits[0]);
                eigenMatrixXd2CArray(measurement.getPreFitResiduals(), &gyroPayload.preFits[0]);
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
            auto starTrackerMeasurement = MeasurementModel();
            starTrackerMeasurement.setMeasurementName("starTracker");
            starTrackerMeasurement.setTimeTag(starTracker.timeTag*NANO2SEC);
            starTrackerMeasurement.setValidity(true);

            starTrackerMeasurement.setMeasurementNoise(
                    this->measNoiseScaling * this->starTrackerMessages[index].measurementNoise);
            starTrackerMeasurement.setObservation(mrpSwitch(Eigen::Map<Eigen::Vector3d>(starTracker.MRP_BdyInrtl),
                    this->mrpSwitchThreshold));
            starTrackerMeasurement.setMeasurementModel(MeasurementModel::mrpStates);
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

        auto gyroMeasurement = MeasurementModel();
        gyroMeasurement.setMeasurementName("gyro");
        gyroMeasurement.setTimeTag(meanMeasurementTime);
        gyroMeasurement.setValidity(true);

        gyroMeasurement.setMeasurementNoise(
                this->measNoiseScaling * this->gyroNoise/std::sqrt(numberOfValidGyroMeasurements));
        gyroMeasurement.setObservation(lowPass.getCurrentState());
        gyroMeasurement.setMeasurementModel(MeasurementModel::velocityStates);
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
