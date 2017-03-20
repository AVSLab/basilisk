/*
 ISC License

 Copyright (c) 2016-2017, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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
#include <cstring>
#include "_GeneralModuleFiles/sys_model.h"
#include "utilities/gauss_markov.h"
#include "simMessages/scPlusStatesSimMsg.h"
#include "simMessages/spiceTimeSimMsg.h"
#include "../SimFswInterfaceMessages/stSensorIntMsg.h"

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
    void computeSensorTimeTag(uint64_t CurrentSimNanos);
    void computeQuaternion(double *sigma, STSensorIntMsg *sensorValue);

public:

    double sensorTimeTag;             //!< [s] Current time tag for sensor out
    std::string inputStateMessage;    //!< [-] String for the input state message
    std::string inputTimeMessage;     //!< [-] String for time input msg
    std::string outputStateMessage;   //!< [-] String for the output state message
    bool messagesLinked;              //!< [-] Indicator for whether inputs bound
    std::vector<double> PMatrix;      //!< [-] Covariance matrix used to perturb state
    std::vector<double> walkBounds;   //!< [-] "3-sigma" errors to permit for states
    std::vector<double> navErrors;    //!< [-] Current navigation errors applied to truth
    uint64_t OutputBufferCount;       //!< [-] Count on the number of output message buffers
    double dcm_CS[9];                 //!< [-] Transformation matrix from structure to case
    STSensorIntMsg trueValues;  //!< [-] total measurement without perturbations
    STSensorIntMsg sensedValues;//!< [-] total measurement including perturbations
    double mrpErrors[3];              //!< [-] Errors to be applied to the input MRP set indicating whether
    uint64_t envTimeClock;            //!< [ns] Clock associated with the environment time message
    SpiceTimeSimMsg timeState;       //!< [-] Module variable where the input Spice Time message is stored
    SCPlusStatesSimMsg scState;      //!< [-] Module variable where the input State Data message is stored




private:
    std::vector<double> AMatrix;      //!< [-] AMatrix that we use for error propagation
    int64_t inputTimeID;              //!< [-] Connect to input time message
    int64_t inputStateID;             //!< [-] Connection to input state message
    int64_t outputStateID;            //!< [-] Connection to outgoing state message
    GaussMarkov errorModel;           //!< [-] Gauss-markov error states
};

#endif
