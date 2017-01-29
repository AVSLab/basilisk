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

#ifndef RW_VOLTAGE_INTERFACE_H
#define RW_VOLTAGE_INTERFACE_H

#include <vector>
#include "_GeneralModuleFiles/sys_model.h"
#include "utilities/simMacros.h"
/*! \addtogroup SimModelGroup
 * @{
 */

typedef struct {
    double Voltage; //!< V, voltage input for RW
}RWVoltageStruct;

//!@brief Interface module to convert RW input voltage to a motor torque output
class RWVoltageInterface: public SysModel {
public:
    RWVoltageInterface();
    ~RWVoltageInterface();
   
    void SelfInit();
    void CrossInit();
    void computeRWMotorTorque();
    void UpdateState(uint64_t CurrentSimNanos);
    void readInputMessages();
    void writeOutputMessages(uint64_t Clock);
    
public:
    uint64_t outputBufferCount;         //!< --     Number of output state buffers in msg
    std::string rwVoltageInMsgName;     //!< --     Message that contains RW voltage input states
    std::string rwMotorTorqueOutMsgName;//!< --     Output Message for RW motor torques
    double voltage2TorqueGain;          //!< Nm/V   gain to convert voltage to motor torque
    double rwTorque[SIM_MAX_EFF_CNT];   //!< Nm     RW motor torque array
private:
    int64_t rwVoltageInMsgID;           //!< -- Message ID associated with RW voltage input state
    int64_t rwMotorTorqueOutMsgID;      //!< -- Message ID associated with RW motor torque output state
    RWVoltageStruct *IncomingVoltageBuffer;//!< -- One-time allocation for time savings
    uint64_t prevTime;                  //!< -- Previous simulation time observed
};

/*! @} */

#endif
