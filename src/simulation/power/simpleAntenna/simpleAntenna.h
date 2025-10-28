/*
 ISC License

 Copyright (c) 2025, Autonomous Vehicle Systems Lab, University of Colorado Boulder

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


#ifndef SIMPLEANTENNA_H
#define SIMPLEANTENNA_H

#include <vector>
#include <stdint.h>
#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/utilities/astroConstants.h"
#include "architecture/utilities/avsEigenSupport.h"
#include "architecture/utilities/rigidBodyKinematics.h"

#include "AntennaDefinitions.h"
#include "architecture/messaging/messaging.h"
#include "architecture/msgPayloadDefC/SCStatesMsgPayload.h"
#include "architecture/msgPayloadDefC/GroundStateMsgPayload.h"
#include "architecture/msgPayloadDefC/antennaStateMsgPayload.h"
#include "architecture/msgPayloadDefC/PowerNodeUsageMsgPayload.h"
#include "architecture/msgPayloadDefC/SpicePlanetStateMsgPayload.h"
#include "architecture/msgPayloadDefC/antennaOutMsgPayload.h"

#include "simulation/power/_GeneralModuleFiles/powerNodeBase.h"

#include "architecture/utilities/bskLogging.h"

/*! @brief This module describes how the simple antenna model operates within the simulation environment.The purpose of this module is to provide a basic representation of antenna behavior for spacecraft communication systems.The antenna is based on a simple model (2D Gaussian beam pattern). The user is able to set gain, HPBW (elevation and azimuth), Tx and Rx power demand, radiation efficiency, bandwidth, noise temperature.If the antenna is in ground environment, atmospheric losses are computed based on the ITU-R P.676-12 model.In this case the user is able to set ambient pressure, temperature, humidity, oxygen and water vapor partial pressures (default values according to ISA).
 */
class SimpleAntenna: public PowerNodeBase {
public: //Functions
    SimpleAntenna();
    ~SimpleAntenna() = default;
    void Reset(uint64_t CurrentSimNanos);

    /** Getters **/
    /** getter for `antennaID` property */
    const char* getAntennaName() const {return this->antennaID;}
    /** getter for `environment` property (space, ground, unknown) */
    AntennaTypes::EnvironmentType getEnvironment() const {return this->environment;}
    /** getter for `antennaState` property (off, Rx, Tx, RxTx) */
    AntennaTypes::AntennaStateEnum getAntennaState() const {return this->antennaState;}
    /** getter for `G` property */
    double getAntennaGain_dB() const {return this->G;}
    /** getter for `frequency` property */
    double getAntennaFrequency() const {return this->frequency;}
    /** getter for `HPBW` property */
    std::vector<double> getAntennaHPBW() const {return this->HPBW;}
    /** getter for `P_Tx` property */
    double getAntennaP_Tx() const {return this->P_Tx;}
    /** getter for `P_Rx` property */
    double getAntennaP_Rx() const {return this->P_Rx;}
    /** getter for `T_E` property (equivalent noise temperature) */
    double getT_E() const {return this->T_E;}
    /** getter for `B` property (bandwidth) */
    double getAntennaBandwidth() const {return this->B;}
    /** getter for `eta_r` property (radiation efficiency) */
    double getAntennaRadEfficiency() const {return this->eta_r;}
    /** getter for `T_amb` property (ambient temperature) */
    double getEnvironmentTemperature() const {return this->T_amb;}
    /** getter for `p_amb` property (ambient pressure) */
    double getEnvironmentPressure() const {return this->p_amb;}
    /** getter for `rho_amb` property (ambient humidity) */
    double getEnvironmentHumidity() const {return this->rho_amb;}
    /** getter for `x_o2` property (oxygen molar fraction) */
    double getX_o2() const {return this->x_o2;}
    /** getter for `x_h2o` property (water vapor molar fraction) */
    double getX_h2o() const {return this->x_h2o;}
    /** getter for `r_AB_B` property (antenna position to SC hub)*/
    Eigen::Vector3d getAntennaPositionBodyFrame() const {return this->r_AB_B;}
    /** getter for `sigma_BA` property (antenna orientation to SC hub)*/
    Eigen::MRPd getAntennaOrientationBodyFrame() const {return this->sigma_BA;}
    /** getter for `P_eirp` property (effective isotropic radiated power)*/
    double getP_eirp() const {return this->P_eirp;}

    //Setters
    /** setter for `antennaState` property */
    void setAntennaName(const char*);
    void setAntennaState(AntennaTypes::AntennaStateEnum);
    /** setter for `G` property */
    void setAntennaGain_dB(double);
    /** setter for frequency property */
    void setAntennaFrequency(double);
    /** setter for `HPBW ratio` "k" property */
    void setAntennaHpbwRatio(double);
    /** setter for `P_Tx` property */
    void setAntennaP_Tx(double);
    /** setter for `P_Rx` property */
    void setAntennaP_Rx(double);
    /** setter for `B` property */
    void setAntennaBandwidth(double);
    /** setter for `T_E` property */
    void setAntennaT_E(double);
    /** setter for `eta_r` property */
    void setAntennaRadEfficiency(double);
    /** setter for `T_amb` property */
    void setAntennaEnvTemperature(double);
    /** setter for `p_amb` property */
    void setAntennaEnvPressure(double);
    /** setter for `rho_amb` property */
    void setAntennaEnvHumidity(double);
    /** setter for `x_o2` property */
    void setAntennaEnvX_o2(double);
    /** setter for `x_h2o` property */
    void setAntennaEnvX_h2o(double);
    /** setter for `p_na_n` property */
    void setAntennaPositionBodyFrame(Eigen::Vector3d);
    /** setter for `q_a_n` property */
    void setAntennaOrientationBodyFrame(Eigen::Vector3d);

private:
    void initializeAntenna();
    void UpdateState(uint64_t CurrentSimNanos);
    void readInputMessages();
    void writeOutputMessages(uint64_t CurrentSimNanos);
    double calculateTsky();
    double calculateTambient();
    void populateOutputMsg(AntennaOutMsgPayload *msgBuffer);
    void calculateAntennaPositionAndOrientation();
//    void calculateAntennaVelocity();
    void calculateAntennaNoise();
    void evaluatePowerModel(PowerNodeUsageMsgPayload *powerUsageMsg);

public:
    // Input Messages
    ReadFunctor<SCStatesMsgPayload>         scStateInMsg;          //!< [-] sc input state message
    ReadFunctor<GroundStateMsgPayload>      groundStateInMsg;      //!< ground state
    ReadFunctor<AntennaStateMsgPayload>     antennaSetStateInMsg;  //!< setting antenna to [off / Rx / Tx / RxTx]
    ReadFunctor<SpicePlanetStateMsgPayload> spicePlanetStateInMsg; //!< [-] sun data input message

    // Variables to store (input) message data
    SCStatesMsgPayload                      scState;               //!< [-] Module variable where the input State Data message is stored
    GroundStateMsgPayload                   groundState;           //!< ground state
    AntennaStateMsgPayload                  antennaStateMsg;       //!< antenna output message
    SpicePlanetStateMsgPayload              spicePlanetStateMsg;   //!< [-] Module variable where the input Sun Data message is stored

    // Output Messages
    Message<AntennaOutMsgPayload>           antennaOutMsg;         //!< output msg description

    // Variables to store (output) message data
    AntennaOutMsgPayload                    antennaPhysicalState;  //!< output msg description

    BSKLogger                               bskLogger;             //!< BSK Logging

private:
    char antennaID[20];                          //!< --      Antenna identifier
    AntennaTypes::EnvironmentType environment;   //!< [-]     Antenna environment of antenna 0: space, 2: ground
    AntennaTypes::AntennaStateEnum antennaState; //!< [-]     Antenna state: 0: off, 1: Rx, 2: Tx, 3: RxTx
    double frequency;                            //!< [Hz]    Antenna operating frequency (center frequency)
    double G;                                    //!< [dB]    Antenna gain 1 - inf
    double k;                                    //!< [-]     k = HPBW_az / HPBW_el; k -> ratio
    std::vector<double> HPBW;                    //!< [rad]   half-power beamwidth in azimuth and elevation
    double P_Tx;                                 //!< [W]     transmit power
    double P_Rx;                                 //!< [W]     receive power
    double B;                                    //!< [Hz]    antenna bandwidth
    double T_E;                                  //!< [K]     equivalent noise temperature (default 50K)
    double P_N;                                  //!< [W]     antenna noise power
    double eta_r;                                //!< [-]     antenna radiation efficiency (default 1.0)
    double P_eirp;                               //!< [dB]    equivalent isotropically radiated power
    double T_amb;                                //!< [K]     ambient temperature (default ISA)
    double p_amb;                                //!< [Pa]    ambient pressure (default ISA)
    double rho_amb;                              //!< [g/m^3] ambient humidity (default ISA)
    double x_o2;                                 //!< [Pa]    oxygen partial pressure (default 0.21*p_amb)
    double x_h2o;                                //!< [Pa]    water vapor partial pressure (default 0.0*p_amb)
    Eigen::Vector3d r_AP_N;                      //!< [m]     position of the antenna wrt to the celestial body frame {P} (used for ground antennas only)
    Eigen::Vector3d r_AB_B;                      //!< [m]     antenna position relative to body frame
    Eigen::MRPd     sigma_BA;                    //!< [-]     antenna orientation relative to body frame
    Eigen::Vector3d r_AN_N;                      //!< [-]     antenna position in N frame
    Eigen::MRPd     sigma_NA;                    //!< [-]     antenna orientation in N frame
    Eigen::Vector3d v_AN;                        //!< [-]     antenna velocity in N frame (stub, future use)
    Eigen::Matrix3d dcm_NA;                      //!< [-]     DCM from Antenna {A} to Inertial {N} frame
};

#endif
