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

#ifndef STAR_TRACKER_H
#define STAR_TRACKER_H

#include <vector>
#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/utilities/gauss_markov.h"

#include "architecture/msgPayloadDefC/SCStatesMsgPayload.h"
#include "architecture/msgPayloadDefC/STSensorMsgPayload.h"
#include "architecture/messaging/messaging.h"

#include <Eigen/Dense>
#include "architecture/utilities/avsEigenMRP.h"
#include "architecture/utilities/bskLogging.h"


/*! @brief star tracker class */
class StarTracker: public SysModel {
public:
    StarTracker();
    ~StarTracker();
    
    void UpdateState(uint64_t CurrentSimNanos);
    void Reset(uint64_t CurrentClock);          //!< Method for reseting the module
    void readInputMessages();
    void writeOutputMessages(uint64_t Clock);
    void computeSensorErrors();
    void applySensorErrors();
    void computeTrueOutput();
    void computeQuaternion(double *sigma, STSensorMsgPayload *sensorValue);
    
public:
    
    uint64_t sensorTimeTag;            //!< [ns] Current time tag for sensor out
    ReadFunctor<SCStatesMsgPayload> scStateInMsg;    //!< [-] sc input state message
    Message<STSensorMsgPayload> sensorOutMsg;   //!< [-] sensor output state message

    Eigen::Matrix3d PMatrix;      //!< [-] Cholesky-decomposition or matrix square root of the covariance matrix to apply errors with
    Eigen::Vector3d walkBounds;   //!< [-] "3-sigma" errors to permit for states
    Eigen::Vector3d navErrors;    //!< [-] Current navigation errors applied to truth

    double dcm_CB[3][3];                 //!< [-] Transformation matrix from body to case
    STSensorMsgPayload trueValues;  //!< [-] total measurement without perturbations
    STSensorMsgPayload sensedValues;//!< [-] total measurement including perturbations
    double mrpErrors[3];              //!< [-] Errors to be applied to the input MRP set indicating whether
    SCStatesMsgPayload scState;      //!< [-] Module variable where the input State Data message is stored
    BSKLogger bskLogger;                      //!< -- BSK Logging




private:
    Eigen::Matrix3d AMatrix;      //!< [-] AMatrix that we use for error propagation
    GaussMarkov errorModel;           //!< [-] Gauss-markov error states
};


#endif
