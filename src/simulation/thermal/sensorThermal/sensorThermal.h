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

#ifndef BASILISK_SENSORTHERMAL_H
#define BASILISK_SENSORTHERMAL_H

#include <Eigen/Dense>
#include <vector>
#include "architecture/messaging/messaging.h"

#include "architecture/msgPayloadDefC/SCStatesMsgPayload.h"
#include "architecture/msgPayloadDefC/SpicePlanetStateMsgPayload.h"
#include "architecture/msgPayloadDefC/EclipseMsgPayload.h"
#include "architecture/msgPayloadDefC/TemperatureMsgPayload.h"
#include "architecture/msgPayloadDefC/DeviceStatusMsgPayload.h"

#include "architecture/utilities/bskLogging.h"



/*! @brief sensor thermal class */
class SensorThermal: public SysModel {

public:
    SensorThermal();
    ~SensorThermal();
    void Reset(uint64_t CurrentClock);
    void UpdateState(uint64_t CurrentSimNanos);

private:
    void evaluateThermalModel(uint64_t CurrentSimSeconds);
    void computeSunData();
    void writeMessages(uint64_t CurrentClock);
    void readMessages();

public:
    ReadFunctor<SpicePlanetStateMsgPayload> sunInMsg;   //!< [-] sun data input message
    ReadFunctor<DeviceStatusMsgPayload> sensorStatusInMsg; //!< optional sensor power status input message
    ReadFunctor<SCStatesMsgPayload> stateInMsg;     //!< [-] spacecraft state input message
    ReadFunctor<EclipseMsgPayload> sunEclipseInMsg;     //!< [-] sun eclipse state input message
    Message<TemperatureMsgPayload> temperatureOutMsg; //!< output temperature message

    Eigen::Vector3d nHat_B;                     //!< [-] Sensor normal unit vector relative to the spacecraft body frame.

    double sensorPowerDraw;                     //!< [W] Power consumed by the sensor (+).
    uint64_t sensorPowerStatus;                //!< [-] Sensor on/off status (0 off / 1 on)
    double sensorArea;                          //!< [m^2] Sensor area in meters squared
    double sensorAbsorptivity;                  //!< [-] Sensor absorptivity (between 0 and 1)
    double sensorEmissivity;                    //!< [-] Sensor emissivity (between 0 and 1)
    double sensorMass;                          //!< [kg] Sensor mass in kg
    double sensorSpecificHeat;                  //!< [J/kg/K] Sensor specific heat
    double T_0;                                 //!< [C] Initial temperature

    BSKLogger bskLogger;                          //!< -- BSK Logging

private:
    TemperatureMsgPayload temperatureMsgBuffer; //!< buffer of output message
    double projectedArea;                       //!< [m^2] Area of the sensor projected along the sun vector.
    SpicePlanetStateMsgPayload sunData;         //!< [-] sun message input buffer
    SCStatesMsgPayload stateCurrent;            //!< [-] Current spacecraft state
    double shadowFactor;                        //!< [-] solar eclipse shadow factor from 0 (fully obscured) to 1 (fully visible)
    double sensorTemp;                          //!< [C] Current temperature
    double Q_in;                                //!< [W] Current power in
    double Q_out;                               //!< [W] Current power out
    double S;                                   //!< [W/m^2] Solar constant
    double boltzmannConst;                      //!< [W/m^2/K^4] Boltzmann constant
    uint64_t CurrentSimSecondsOld;              //!< [s] Seconds at last iteration


};


#endif //BASILISK_SENSORTHERMAL_H
