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

#ifndef MOTOR_THERMAL_H
#define MOTOR_THERMAL_H

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/msgPayloadDefC/TemperatureMsgPayload.h"
#include "architecture/msgPayloadDefC/RWConfigLogMsgPayload.h"
#include "architecture/utilities/bskLogging.h"
#include "architecture/messaging/messaging.h"

/*! @brief Motor temperature module.  It simulates the heating and cooling of a motor based on ambient temperature, as well as heat generated during spin-up or breaking. */
class MotorThermal: public SysModel {
public:
    MotorThermal();
    ~MotorThermal();

    void Reset(uint64_t CurrentSimNanos);
    void UpdateState(uint64_t CurrentSimNanos);
    void readInputMessages();
    void writeOutputMessages(uint64_t CurrentClock);
    void computeTemperature(uint64_t CurrentSimNanos);

public:
    Message<TemperatureMsgPayload> temperatureOutMsg;   //!< [Celsius] temperature output message
    ReadFunctor<RWConfigLogMsgPayload> rwStateInMsg;   //!< reaction wheel state input message
    double efficiency;                  //!< efficiency factor to convert power into mechanical power
    double currentTemperature;          //!< [Celsius] stored temperature
    double ambientTemperature;          //!< [Celsius] ambient temperature for heat dissipation
    double ambientThermalResistance;    //!< [W/Celsius] ambient thermal resistance to convert heat into temperature
    double motorHeatCapacity;           //!< [J/Celsius] motor heat caapcity to convert heat into temperature
    BSKLogger bskLogger;                //!< -- BSK Logging

private:
    TemperatureMsgPayload temperatureBuffer;    //!< temperature buffer for internal calculations
    RWConfigLogMsgPayload rwStateBuffer;        //!< reaction wheel state buffer

    uint64_t prevTime;                 //!< -- Previous simulation time observed

};


#endif
