/*
 ISC License

 Copyright (c) 2025, Department of Engineering Cybernetics, NTNU, Norway

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
#include <cstdint>
#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/utilities/astroConstants.h"
#include "architecture/utilities/avsEigenSupport.h"
#include "architecture/utilities/rigidBodyKinematics.h"
#include "architecture/utilities/haslamBackgroundRadiation.h"

#include "../_GeneralModuleFiles/AntennaDefinitions.h"
#include "architecture/messaging/messaging.h"
#include "architecture/msgPayloadDefC/SCStatesMsgPayload.h"
#include "architecture/msgPayloadDefC/GroundStateMsgPayload.h"
#include "architecture/msgPayloadDefC/AntennaStateMsgPayload.h"
#include "architecture/msgPayloadDefC/SpicePlanetStateMsgPayload.h"
#include "architecture/msgPayloadDefC/AntennaLogMsgPayload.h"
#include "architecture/msgPayloadDefC/EclipseMsgPayload.h"

#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/bskLogging.h"

/*! @brief This module describes how the simple antenna model operates within the simulation environment.The purpose of this module is to provide a basic
           representation of antenna behavior for spacecraft communication systems.The antenna is based on a simple model (2D Gaussian beam pattern).
           The user is able to set directivity, HPBW (elevation and azimuth), power demand for Tx and Rx, radiation efficiency, bandwidth and system noise temperature.
*/
class SimpleAntenna: public SysModel {
public: //Functions
    SimpleAntenna();
    ~SimpleAntenna() = default;
    void Reset(uint64_t CurrentSimNanos);
    void UpdateState(uint64_t CurrentSimNanos);
    void addPlanetToModel(Message<SpicePlanetStateMsgPayload> *tmpSpMsg);
    void configureBrightnessFile(const std::string& file);
    double calculateTsky();

    /** Getters **/
    /** getter for antennaName */
    std::string getAntennaName() const {return this->antennaName;}
    /** getter for `environment` property (space, ground, unknown) */
    AntennaTypes::EnvironmentType getAntennaEnvironment() const {return this->environment;}
    /** getter for `antennaState` property (off, Rx, Tx, RxTx) */
    AntennaTypes::AntennaStateEnum getAntennaState() const {return this->antennaState;}
    /** getter for `frequency` property */
    double getAntennaFrequency() const {return this->frequency;}
    /** getter for `B` property (bandwidth) */
    double getAntennaBandwidth() const {return this->B;}
    /** getter for `DdB` property */
    double getAntennaDdB() const {return this->DdB;}
    /** getter for HPBW ratio, k **/
    double getAntennaHpbwRatio() const {return this->k;}
    /** getter for `HPBW` property */
    std::vector<double> getAntennaHPBW() const {return this->HPBW;}
    /** getter for `P_Tx` property */
    double getAntennaP_Tx() const {return this->P_Tx;}
    /** getter for `P_Rx` property */
    double getAntennaP_Rx() const {return this->P_Rx;}
    /** getter for `T_E` property (equivalent noise temperature) */
    double getT_E() const {return this->T_E;}
    /** get antenna noise power P_N **/
    double getAntennaNoisePower() const {return this->P_N;}
    /** getter for `eta_rad` property (radiation efficiency) */
    double getAntennaRadEfficiency() const {return this->eta_r;}
    /** getter for `P_eirp_dB` property (effective isotropic radiated power)*/
    double getP_eirp_dB() const {return this->P_eirp_dB;}
    /** getter for `G_TN` property (antenna gain over system noise temperature)*/
    double getG_TN() const {return this->G_TN;}
    /** getter for `T_Ambient` property (ambient temperature) */
    double getAntennaEnvTemperature() const {return this->T_Ambient;}
    /** get antenna position in N frame **/
    Eigen::Vector3d getAntennaPositionInertialFrame() const {return this->r_AN_N;}
    /** get antenna orientation in N frame **/
    Eigen::MRPd getAntennaOrientationInertialFrame() const {return this->sigma_AN;}
    /** get antenna velocity in N frame **/
    Eigen::Vector3d getAntennaVelocityInertialFrame() const {return this->v_AN_N;}
    /** getter for `r_AB_B` property (antenna position to SC hub)*/
    Eigen::Vector3d getAntennaPositionBodyFrame() const {return this->r_AB_B;}
    /** getter for `sigma_AB` property (antenna orientation to SC hub)*/
    Eigen::MRPd getAntennaOrientationBodyFrame() const {return this->sigma_AB;}
    /** get antenna position wrt to celestial body decomposed in {N}-frame **/
    Eigen::Vector3d getAntennaPositionCelestialBodyInertialFrame() const {return this->r_AP_N;}
    /** getter for `useHaslamMap` property */
    bool getUseHaslamMap() const {return this->useHaslamMap;}

    //Setters
    /** setter for `antennaName` property */
    void setAntennaName(const std::string&);
    /** setter for `antennaState` property */
    void setAntennaState(AntennaTypes::AntennaStateEnum);
    /** setter for `DdB` property */
    void setAntennaDirectivity_dB(double);
    /** setter for frequency property */
    void setAntennaFrequency(double);
    /** setter for `HPBW ratio` "k" property */
    void setAntennaHpbwRatio(double);
    /** setter for `P_Tx` property */
    void setAntennaP_Tx(double);
    /** setter for `P_Rx` property */
    void setAntennaP_Rx(double);
    /** setter for `T_E` property */
    void setAntennaEquivalentNoiseTemp(double);
    /** setter for `B` property */
    void setAntennaBandwidth(double);
    /** setter for `eta_rad` property */
    void setAntennaRadEfficiency(double);
    /** setter for `T_Ambient` property */
    void setAntennaEnvironmentTemperature(double);
    /** setter for `r_AB_B` property (antenna position relative to body frame) */
    void setAntennaPositionBodyFrame(Eigen::Vector3d);
    /** setter for `sigma_AB` property */
    void setAntennaOrientationBodyFrame(Eigen::MRPd);
    /** setter for `useHaslamMap` property */
    void setUseHaslamMap(bool);

private:
    void initializeAntenna();
    void readInputMessages();
    void writeOutputMessages(uint64_t CurrentSimNanos);
    double calculatePlanetCoverage(double planetRadius, Eigen::Vector3d r_PN_N, double azimuth, double elevation);
    double getPlanetEquatorialRadius(std::string planetSpiceName);
    double calculateTambient();
    void populateOutputMsg(AntennaLogMsgPayload *msgBuffer);
    void calculateAntennaPositionAndOrientation();
//    void calculateAntennaVelocity();
    void calculateAntennaNoise();

public:
    // Input Messages
    ReadFunctor<SCStatesMsgPayload>         scStateInMsg;          //!< [-] sc input state message
    ReadFunctor<GroundStateMsgPayload>      groundStateInMsg;      //!< ground state
    ReadFunctor<AntennaStateMsgPayload>     antennaSetStateInMsg;  //!< setting antenna to [off / Rx / Tx / RxTx]
    ReadFunctor<SpicePlanetStateMsgPayload> sunInMsg; //!< [-] sun data input message
    std::vector<ReadFunctor<SpicePlanetStateMsgPayload>> planetInMsgs;  //!< A vector of planet incoming state message names ordered by the sequence in which planet are added to the module
    ReadFunctor<EclipseMsgPayload> sunEclipseInMsg;     //!< [-] Messun eclipse state input message

    // Variables to store (input) message data
    SCStatesMsgPayload                      scState;               //!< [-] Module variable where the input State Data message is stored
    GroundStateMsgPayload                   groundState;           //!< ground state
    AntennaStateMsgPayload                  antennaStateMsg;       //!< antenna output message
    SpicePlanetStateMsgPayload              spicePlanetStateMsg;   //!< [-] Module variable where the input Sun Data message is stored

    // Output Messages
    Message<AntennaLogMsgPayload>           antennaOutMsg;         //!< output msg description

    // Variables to store (output) message data
    AntennaLogMsgPayload                    antennaPhysicalStateMsgBuffer;  //!< output msg description

    BSKLogger                               bskLogger;             //!< BSK Logging
    double rEqCustom;  //!< [m] Custom radius

private:
    double                     minTreshold = 1e-6;          //!< [-]     Minimum threshold for power and directivity values

    std::string                antennaName;                 //!< [-]     Antenna name
    AntennaTypes::EnvironmentType           environment;    //!< [-]     Antenna environment of antenna 0: SPACE, 1: EARTH, -1: UNKNOWN
    AntennaTypes::AntennaStateEnum          antennaState;   //!< [-]     Antenna state: 0: off, 1: Rx, 2: Tx, 3: RxTx, -1: UNKNOWN
    std::vector<SpicePlanetStateMsgPayload> planetBuffer;   //!< [-]     Buffer of the spacecraft state input messages
    double                     frequency;                   //!< [Hz]    Antenna operating frequency (center frequency)
    double                     B;                           //!< [Hz]    Antenna bandwidth
    double                     DdB;                         //!< [dB]    Antenna gain (default 20 dBi --> gain of 100 in linear scale)
    double                     k;                           //!< [-]     Elliptical Gaussian beamwidth ratio: k = HPBW_az / HPBW_el
    double                     P_Tx;                        //!< [W]     Transmit power
    double                     P_Rx;                        //!< [W]     Receive power
    double                     T_E;                         //!< [K]     Equivalent noise temperature (default 50K) => Contribution of receiver and other components
    double                     eta_r;                       //!< [-]     Antenna radiation efficiency (default 0.5)
    std::vector<double>        HPBW;                        //!< [rad]   Half-power beamwidth in azimuth and elevation
    double                     P_N;                         //!< [W]     Antenna noise power
    double                     P_eirp_dB;                   //!< [dB]    Equivalent isotropically radiated power
    double                     G_TN;                        //!< [dB/K]  Antenna gain over system noise temperature
    double                     T_Ambient;                   //!< [K]     Ambient temperature (default ISA)
    double                     T_AmbientSet;                //!< [K]     User defined Ambient temperature
    Eigen::Vector3d            r_AN_N;                      //!< [m]     Antenna position in N frame
    Eigen::MRPd                sigma_AN;                    //!< [-]     Antenna orientation in N frame
    Eigen::Vector3d            v_AN_N;                      //!< [m/s]   Antenna velocity in N frame (stub, future use)
    Eigen::Vector3d            r_AB_B;                      //!< [m]     Antenna position relative to body frame
    Eigen::MRPd                sigma_AB;                    //!< [-]     Antenna orientation relative to body frame
    Eigen::Vector3d            r_AP_N;                      //!< [m]     Position of the antenna wrt to the celestial body frame {P} (used for ground antennas only)
    Eigen::Vector3d            r_BN_N;                      //!< [m]     Position of the SC in the inertial frame {N} AND Spacecraft B position relative to sun S
    Eigen::Vector3d            nHat_A;                      //!< [-]     Antenna boresight vector in antenna frame
    bool                       useHaslamMap;                //!< [bool]  Flag to use Haslam sky map data structure (slower but more accurate)
    SpicePlanetStateMsgPayload sunData;                     //!< [-]     Sun data structure
    double illuminationFactor;                              //!< [-] solar eclipse shadow factor from 0 (fully obscured) to 1 (fully visible)
};
#endif
