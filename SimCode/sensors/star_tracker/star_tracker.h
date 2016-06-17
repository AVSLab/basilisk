/*
Copyright (c) 2016, Autonomous Vehicle Systems Lab, Univeristy of Colorado at Boulder

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
#include "utilities/sys_model.h"
#include "utilities/gauss_markov.h"
#include "utilities/dyn_effector.h"
#include "../ADCSAlgorithms/sensorInterfaces/STSensorData/stHwInterface.h"
#include "environment/spice/spice_interface.h"


class StarTracker: public SysModel {
public:
    StarTracker();
    ~StarTracker();
    
    bool LinkMessages();
    void UpdateState(uint64_t CurrentSimNanos);
    void SelfInit();
    void CrossInit();
    void readInputs();
    void writeOutputs(uint64_t Clock);
    void computeErrors();
    void applyErrors();
    void computeOutputs(uint64_t CurrentSimNanos);
    
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
    double T_CaseStr[9];              //!< [-] Transformation matrix from case to body
    StarTrackerHWOutput localOutput;  //!< [-] Class-local storage for output message
    int32_t isOutputTruth;            //!< [-] Flag indicating whether the output information is the truth or is corrupted with sensor errors
    
    double mrpErrors[3];
    double sigmaOutput[3];
    SpiceTimeOutput timeState;
    OutputStateData trueState;

    
    
    
private:
    std::vector<double> AMatrix;      //!< [-] AMatrix that we use for error propagation
    int64_t inputTimeID;              //!< [-] Connect to input time message
    int64_t inputStateID;             //!< [-] Connection to input state message
    int64_t outputStateID;            //!< [-] Connection to outgoing state message
    GaussMarkov errorModel;           //!< [-] Gauss-markov error states
};

#endif
