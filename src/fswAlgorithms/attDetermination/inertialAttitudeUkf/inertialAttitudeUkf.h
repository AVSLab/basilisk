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

/*! @brief Top level structure for the inertial attitude unscented kalman filter.
 Used to estimate the spacecraft's inertial attitude as MRPs and attitude rate.
 */

#ifndef INERTIAL_ATTITUDE_UKF_H
#define INERTIAL_ATTITUDE_UKF_H

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/messaging/messaging.h"
#include "architecture/utilities/macroDefinitions.h"
#include "architecture/utilities/rigidBodyKinematics.hpp"
#include "architecture/utilities/signalProcessing.h"

#include "architecture/msgPayloadDefC/NavAttMsgPayload.h"
#include "architecture/msgPayloadDefCpp/FilterMsgPayload.h"
#include "architecture/msgPayloadDefCpp/FilterResidualsMsgPayload.h"

#include "architecture/msgPayloadDefC/STAttMsgPayload.h"
#include "architecture/msgPayloadDefC/AccDataMsgPayload.h"
#include "architecture/msgPayloadDefC/RWSpeedMsgPayload.h"
#include "architecture/msgPayloadDefC/VehicleConfigMsgPayload.h"
#include "architecture/msgPayloadDefC/RWArrayConfigMsgPayload.h"

#include "fswAlgorithms/_GeneralModuleFiles/srukfInterface.h"
#include "fswAlgorithms/_GeneralModuleFiles/measurementModels.h"

/*! @brief Star Tracker (ST) sensor container class. Contains the msg input name and Id and sensor noise value. */
class StarTrackerMessage{
public:
    ReadFunctor<STAttMsgPayload> starTrackerMsg;                       //!< star tracker input message
    Eigen::Matrix3d measurementNoise;                        //!< [-] Per axis noise on the ST
};

enum class AttitudeFilterMethod {StarOnly, GyroWhenDazzled, AllMeasurements};

/*! @brief Inertial Attitude filter which reads Star Tracker measurements and gyro measurements */
class InertialAttitudeUkf: public SRukfInterface {
public:
    InertialAttitudeUkf(AttitudeFilterMethod method);
    ~InertialAttitudeUkf() override;

private:
    void customReset() override;
    void readFilterMeasurements() override;
    void writeOutputMessages(uint64_t CurrentSimNanos) override;
    void customInitializeUpdate() override;
    void customFinalizeUpdate() override;

    /*! Specific read messages */
    void readRWSpeedData();
    void readStarTrackerData();
    void readGyroData();


public:
    ReadFunctor<RWArrayConfigMsgPayload> rwArrayConfigMsg;
    RWArrayConfigMsgPayload rwArrayConfigPayload;
    ReadFunctor<VehicleConfigMsgPayload> vehicleConfigMsg;
    ReadFunctor<RWSpeedMsgPayload> rwSpeedMsg;
    ReadFunctor<AccDataMsgPayload> accelDataMsg;

    Message<NavAttMsgPayload> navAttitudeOutputMsg;
    Message<FilterMsgPayload> inertialFilterOutputMsg;
    Message<FilterResidualsMsgPayload> starTrackerResidualMsg;
    Message<FilterResidualsMsgPayload> gyroResidualMsg;

    void setGyroNoise(const Eigen::Matrix3d &gyroNoise);
    Eigen::Matrix3d getGyroNoise() const;
    void addStarTrackerInput(const StarTrackerMessage &starTracker);
    Eigen::Matrix3d getStarTrackerNoise(int starTrackerNumber) const;
    void setLowPassFilter(double step, double frequencyCutOff);

private:
    AttitudeFilterMethod measurementAcceptanceMethod;
    bool firstFilterPass = true;
    bool validStarTracker = false;

    Eigen::Vector<double, 4> wheelAccelerations;
    Eigen::Matrix<double, 1, 4> previousWheelSpeeds;
    Eigen::Matrix3d spacecraftInertia;
    Eigen::Matrix3d spacecraftInertiaInverse;
    double previousWheelSpeedTime = 0;
    double hStep = 0.5;
    double cutOffFrequency = 15.0/(2*M_PI);

    Eigen::Vector3d filteredOmega_BN_B;
    Eigen::Matrix3d gyroNoise;
    std::array<StarTrackerMessage, MAX_ST_VEH_COUNT> starTrackerMessages;
    int numberOfStarTackers = 0;
    int measurementIndex = 0;
    double mrpSwitchThreshold = 1; //!< [-] Threshold for switching MRP to/from the shadow set
};

#endif
