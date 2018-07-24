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
#include "_GeneralModuleFiles/sys_model.h"
#include "utilities/gauss_markov.h"
#include "simMessages/scPlusStatesSimMsg.h"
#include "simMessages/spiceTimeSimMsg.h"
#include "simFswInterfaceMessages/stSensorIntMsg.h"
#include <Eigen/Dense>
#include "../simulation/utilities/avsEigenMRP.h"


//!@brief Sensor model to simulate a Star Tracker.
/*!
 The module
 [PDF Description](Basilisk-star_tracker-20161101.pdf)
 contains further information on this module's function,
 how to run it, as well as testing.
 The corruption types are outlined in this
 [PDF document](BasiliskCorruptions.pdf).
 */
class StarTracker: public SysModel {
public:
    StarTracker();
    ~StarTracker();
    
    bool LinkMessages();
    void UpdateState(uint64_t CurrentSimNanos);
    void SelfInit();
    void CrossInit();
    void readInputMessages();
    void writeOutputMessages(uint64_t Clock);
    void computeSensorErrors();
    void applySensorErrors();
    void computeTrueOutput();
    void computeQuaternion(double *sigma, STSensorIntMsg *sensorValue);
    
public:
    
    uint64_t sensorTimeTag;            //!< [ns] Current time tag for sensor out
    std::string inputStateMessage;    //!< [-] String for the input state message
    std::string outputStateMessage;   //!< [-] String for the output state message
    bool messagesLinked;              //!< [-] Indicator for whether inputs bound
    Eigen::Matrix3d PMatrix;      //!< [-] Covariance matrix used to perturb state
    Eigen::Vector3d walkBounds;   //!< [-] "3-sigma" errors to permit for states
    Eigen::Vector3d navErrors;    //!< [-] Current navigation errors applied to truth
    uint64_t OutputBufferCount;       //!< [-] Count on the number of output message buffers
    double dcm_CB[3][3];                 //!< [-] Transformation matrix from body to case
    STSensorIntMsg trueValues;  //!< [-] total measurement without perturbations
    STSensorIntMsg sensedValues;//!< [-] total measurement including perturbations
    double mrpErrors[3];              //!< [-] Errors to be applied to the input MRP set indicating whether
    SCPlusStatesSimMsg scState;      //!< [-] Module variable where the input State Data message is stored

    
    
    
private:
    Eigen::Matrix3d AMatrix;      //!< [-] AMatrix that we use for error propagation
    int64_t inputStateID;             //!< [-] Connection to input state message
    int64_t outputStateID;            //!< [-] Connection to outgoing state message
    GaussMarkov errorModel;           //!< [-] Gauss-markov error states
};

#endif
