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

#ifndef AGGREGATE_SUN_SENSORS_H
#define AGGREGATE_SUN_SENSORS_H

#include <vector>
#include "utilities/sys_model.h"
#include "sensors/sun_sensor/coarse_sun_sensor.h"
#include "utilities/dyn_effector.h"
#include <random>

class AggregateSunSensors: public SysModel {
public:
	AggregateSunSensors();
    ~AggregateSunSensors();
    
    void CrossInit();
    void SelfInit();
    void UpdateState(uint64_t CurrentSimNanos);
    void ReadInputs();
    void WriteOutputs(uint64_t Clock);
    
public:
    std::vector<std::string> inputCSSMsgs;      //!< [-] List of input CSS messages 
	std::string outputAggMsg;                  //!< [-] Message name for CSS output data 
    uint64_t outputBufferCount;      //!< [-] number of output msgs stored
	std::vector<CSSRawOutputData> cssStorage; //!< [-] Local storage for all CSS inputs
private:
    std::vector<int64_t> inputCSSIDs;//!< [-] Vector of IDs associated with input names
    int64_t outputDataID;            //!< [-] Connect to output CSS data
	CSSRawOutputData *outputBuffer;     //!< [-] dynamically allocated message buffer stored for minimal allocation
};

#endif
