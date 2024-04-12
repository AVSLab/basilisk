/*
 ISC License

 Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#ifndef SIMPLE_NAV_H
#define SIMPLE_NAV_H

#include <vector>
#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/utilities/gauss_markov.h"
#include "architecture/msgPayloadDefC/SCStatesMsgPayload.h"
#include "architecture/msgPayloadDefC/SpicePlanetStateMsgPayload.h"
#include "architecture/msgPayloadDefC/NavAttMsgPayload.h"
#include "architecture/msgPayloadDefC/NavTransMsgPayload.h"
#include "architecture/msgPayloadDefC/EphemerisMsgPayload.h"
#include "architecture/msgPayloadDefC/AccPktDataMsgPayload.h"
#include "architecture/msgPayloadDefC/AccDataMsgPayload.h"
#include "architecture/utilities/bskLogging.h"
#include <Eigen/Dense>
#include "architecture/messaging/messaging.h"

/*! @brief simple navigation module class */
class SimpleNav: public SysModel {
public:
    SimpleNav();
    ~SimpleNav();

    void Reset(uint64_t CurrentSimNanos);
    void UpdateState(uint64_t CurrentSimNanos);
    void computeTrueOutput(uint64_t Clock);
    void computeErrors(uint64_t CurrentSimNanos);
    void applyErrors();
    void readInputMessages();
    void writeOutputMessages(uint64_t Clock);

public:
    double gyroStandardDeviation=1E-5;    //!< Standard deviation for each rate component
    double accelStandardDeviation=1E-8;    //!< Standard deviation for each acceleration component
    double gyroBias=0 ;    //!<  Bias for each rate component
    double accelBias=0;    //!<  Bias for each acceleration component
    double gyroErrors[3*MAX_ACC_BUF_PKT]; //!<  Errors to apply to each gyro measurement
    double accelErrors[3*MAX_ACC_BUF_PKT]; //!<  Errors to apply to each accelerometer measurement
    int numberOfGyroBuffers=100;       //!< Number of gyro measurements per timestep
    int gyroFrequencyPerSecond=500;       //!< Number of gyro measurements per second
    Eigen::MatrixXd PMatrix;          //!< -- Cholesky-decomposition or matrix square root of the covariance matrix to apply errors with
    Eigen::VectorXd walkBounds;       //!< -- "3-sigma" errors to permit for states
    Eigen::VectorXd navErrors;        //!< -- Current navigation errors applied to truth
    Message<NavAttMsgPayload> attOutMsg;        //!< attitude navigation output msg
    Message<NavTransMsgPayload> transOutMsg;    //!< translation navigation output msg
    Message<EphemerisMsgPayload> scEphemOutMsg;    //!< translation navigation output msg
    Message<AccDataMsgPayload> accelDataOutMsg; //!< accelerometer and gyro data output msg
    bool crossTrans;                  //!< -- Have position error depend on velocity
    bool crossAtt;                    //!< -- Have attitude depend on attitude rate
    NavAttMsgPayload trueAttState;        //!< -- attitude nav state without errors
    NavAttMsgPayload estAttState;         //!< -- attitude nav state including errors
    NavTransMsgPayload trueTransState;    //!< -- translation nav state without errors
    NavTransMsgPayload estTransState;     //!< -- translation nav state including errors
    EphemerisMsgPayload spacecraftEphemerisState;    //!< -- full spacecraft ephemeris state with errors
    AccDataMsgPayload accelDataState;  //!< accelerometer and gyro data payload
    SCStatesMsgPayload inertialState; //!< -- input inertial state from Star Tracker
    SpicePlanetStateMsgPayload sunState;  //!< -- input Sun state
    BSKLogger bskLogger;              //!< -- BSK Logging

    ReadFunctor<SCStatesMsgPayload> scStateInMsg;      //!< spacecraft state input msg
    ReadFunctor<SpicePlanetStateMsgPayload> sunStateInMsg; //!< (optional) sun state input input msg

private:
    Eigen::MatrixXd AMatrix;           //!< -- The matrix used to propagate the state
    GaussMarkov errorModel;            //!< -- Gauss-markov error states
    uint64_t prevTime;                 //!< -- Previous simulation time observed
};


#endif
