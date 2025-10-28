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

#include "simulation/power/simpleAntenna/simpleAntenna.h"
//#include "simulation/sensors/simpleAntenna/simpleAntenna.h"
#include <iostream>
#include <cstring>
#include <math.h>

/*! This is the constructor for the module class.  It sets default variable
    values and initializes the various parts of the model */
SimpleAntenna::SimpleAntenna()
{
    // initialize module variables
    // Antenna configuration and state
    std::strcpy(this->antennaID, "antenna");                          // --        Antenna identifier
    this->environment  = {};                                          // [-]       Antenna environment of antenna 0: space, 1: ground
    this->antennaState = AntennaTypes::AntennaStateEnum::ANTENNA_OFF; // [-]       Antenna state
    // Antenna Properties
    this->frequency    = {};                                          // [Hz]      Antenna operating frequency
    this->G            = 0;                                           // [dB]      Antenna gain (default 0 dB --> isotropic antenna)
    this->k            = {1};                                         // [-]       HPBW_az / HPBW_el ratio (default 1 --> symmetric beam)
    this->HPBW         = {-1, -1};                                    // [degrees] Half-power beamwidth in azimuth and elevation (default -1,-1: not set for isotropic antenna)
    this->P_Tx         = {};                                          // [W]       Transmit power
    this->P_Rx         = {};                                          // [W]       Receive power
    this->B            = {};                                          // [Hz]      Antenna frequency bandwidth
    this->T_E          = 50.0;                                        // [K]       Equivalent noise temperature (default 50K)
    this->P_N          = {};                                          // [K]       Noise power of the antenna
    this->eta_r        = 0.5;                                         // [-]       Antenna radiation efficiency (default 0.5)
    this->P_eirp       = {};                                          // [dB]      Equivalent isotropically radiated power (EIRP) (determined at initialization)
    // Antenna enviroment properties
    this->T_amb        = {-1};                                        // [K]       Ambient temperature
    this->p_amb        = {};                                          // [Pa]      Ambient pressure
    this->rho_amb      = {};                                          // [g/m^3]   Ambient humidity (default ISA)
    this->x_o2         = 0.21;                                        // [-]       Oxygen content in the air (default 0.21)
    this->x_h2o        = 0.0;                                         // [-]       Water vapor content (default 0.0*p_amb)
    this->r_AP_N       = Eigen::Vector3d::Zero();                     // [m]       Position of the antenna wrt to the celestial body frame {P} (used for ground antennas only)
    // Antenna position and orientation
    this->r_AB_B       = Eigen::Vector3d::Zero();                     // [m]       Antenna position relative to body frame    (default aligned with body)
    this->sigma_BA     = Eigen::MRPd::Identity();                     // [-]       Antenna orientation relative to body frame (default aligned with body)
    this->r_AN_N       = Eigen::Vector3d::Zero();                     // [-]       Antenna position in N frame
    this->sigma_NA     = Eigen::MRPd::Identity();                     // [-]       Antenna orientation in N frame
    this->v_AN         = Eigen::Vector3d::Zero();                     // [-]       Antenna velocity in N frame (stub, future use)
    this->dcm_NA       = Eigen::Matrix3d::Identity();                 // [-]       DCM from Antenna {A} to Inertial {N} frame
}

/*! This method is used to reset the module and checks that required input messages are connected.
*/
void SimpleAntenna::Reset(uint64_t CurrentSimNanos)
{
    // check that required input messages are connected
    if (!this->scStateInMsg.isLinked() && !this->groundStateInMsg.isLinked()) {
        // Antenna is NOT connected to ground or spacecraft (needs to be connected to one)
        bskLogger.bskLog(BSK_ERROR, "Neither SimpleAntenna.scStateInMsg nor SimpleAntenna.groundState was linked.");
    }
    if (this->scStateInMsg.isLinked() && this->groundStateInMsg.isLinked()) {
        // Antenna is connected to BOTH ground and spacecraft (needs to be connected to only one)
        bskLogger.bskLog(BSK_ERROR, "Both SimpleAntenna.scStateInMsg and SimpleAntenna.groundState were linked. Please link only one to define the environment.");
    }
    if (!this->antennaSetStateInMsg.isLinked()) {
        // Antenna state message is NOT linked -> Antenna remains OFF and cannot be switched on! (warning only)
        bskLogger.bskLog(BSK_WARNING, "SimpleAntenna.antennaSetStateInMsg was not linked (Antenna is by default OFF you wont be able to change this).");
    }
    if (!this->spicePlanetStateInMsg.isLinked()) {
        // Sun data message is NOT linked -> space background temperature is uniform (warning only)
        bskLogger.bskLog(BSK_WARNING, "SimpleAntenna.spicePlanetStateInMsg was not linked => Space background temperature is set to uniform temperature.");
    }

    // Sanity checks (make sure the following parameters are set: G, P_Tx)
    if (this->G < 0) {
        bskLogger.bskLog(BSK_ERROR, "SimpleAntenna antenna gain G [dB] is invalid. It must be greater than or equal to 0 dB. Setting to default value of 0 dB (isotropic antenna).");
    }
    if (this->P_Tx == 0) {
        bskLogger.bskLog(BSK_ERROR, "SimpleAntenna transmit power P_Tx [W] is not set.");
    }
    // Warn the user if the following parameters are not set: frequency, B, k, P_Rx
    if (this->frequency == 0) {
        bskLogger.bskLog(BSK_WARNING, "SimpleAntenna operating frequency [Hz] is not set. Setting to default value for S-band (2.35 GHz).");
        this->frequency = 2.35e9; // Set default frequency to 2.35 GHz
    }
    if (this->B == 0) {
        bskLogger.bskLog(BSK_WARNING, "SimpleAntenna bandwidth B [Hz] is not set. Setting to default value of 2 MHz.");
        this->B = 2.0e6;          // Set default bandwidth to 2 MHz
    }
    if (this->k >= 10 || this->k <= 0.1) {
        bskLogger.bskLog(BSK_WARNING, "SimpleAntenna antenna HPBW ratio (k) is invalid, must be between 0.1 and 10. Setting to default value of 1.0.");
        this->k = 1.0;
    }
    if (this->P_Rx == 0) {
        bskLogger.bskLog(BSK_WARNING, "SimpleAntenna receive power P_Rx [W] is not set. Setting to 0.1 * P_Tx.");
        this->P_Rx = 0.1 * this->P_Tx;
    }

    // initialize antenna environment
    this->initializeAntenna();
}

void SimpleAntenna::initializeAntenna()
{
    // Calculate HPBW_el & HPBW_az based on k and G
    if (this->G == 0) {
        // Isotropic antenna (HPBW not defined)
        this->HPBW = {-1, -1};
    } else if (this->G > 0) {
        double gain_lin = pow(10, this->G / 10);                      // linear scale gain
        double HPBW_el  = sqrt((log(2) * 16 * this->k) / (gain_lin)); // [rad]
        double HPBW_az  = this->k * HPBW_el;                          // [rad]
        this->HPBW      = {HPBW_az, HPBW_el};
    } else {
        // robustness (unreachable)
        bskLogger.bskLog(BSK_ERROR, "SimpleAntenna antenna gain G [dB] is invalid (negative)");
    }

    // Calculate antenna EIRP
    this->P_eirp = 10 * log10(this->P_Tx) + 10 * log10(this->eta_r) + this->G;

    // Determine environment
    // Check if groundMsg or Spacecraft is linked
    if (this ->scStateInMsg.isLinked() & !this->groundStateInMsg.isLinked()) {
        this->environment = AntennaTypes::EnvironmentType::ENVIRONMENT_SPACE; // Space
    } else if (this->groundStateInMsg.isLinked() & !this->scStateInMsg.isLinked()) {
        this->environment = AntennaTypes::EnvironmentType::ENVIRONMENT_EARTH; // Ground
    } else {
        // Error condition
        bskLogger.bskLog(BSK_ERROR, "SimpleAntenna: Unable to determine environment. Please link either GroundState or scState message, not both.");
        this->environment = AntennaTypes::EnvironmentType::_ENVIRONMENT_UNKNOWN; // Unknown
    }
}

/*! This is the main method that gets called every time the module is updated.  Provide an appropriate description.
*/
void SimpleAntenna::UpdateState(uint64_t CurrentSimNanos)
{
    this->readInputMessages();                       // read in the input messages
    this->calculateAntennaPositionAndOrientation();  // calculate antenna position and attitude in {N} frame
    this->calculateAntennaNoise();                   // calculate antenna noise temperature
    this->writeOutputMessages(CurrentSimNanos);      // write the output messages
}

void SimpleAntenna::readInputMessages()
{
    if (this->environment == AntennaTypes::EnvironmentType::ENVIRONMENT_SPACE) {
        // Space environment
        this->scState = this->scStateInMsg();
    } else if (this->environment == AntennaTypes::EnvironmentType::ENVIRONMENT_EARTH) {
        // Ground environment
        this->groundState = this->groundStateInMsg();
    }
    this->antennaStateMsg = this->antennaSetStateInMsg();
    this->antennaState    = static_cast<AntennaTypes::AntennaStateEnum>(this->antennaStateMsg.antennaState);
}

void SimpleAntenna::writeOutputMessages(uint64_t CurrentSimNanos)
{
    this->populateOutputMsg(&this->antennaPhysicalState);
    this->antennaOutMsg.write(&this->antennaPhysicalState, this->moduleID, CurrentSimNanos); // write antenna state message
}

/* Calculate sky temperature based on environmental parameters
*/
double SimpleAntenna::calculateTsky()
{
    double          T_sky, azimuth, elevation;
    Eigen::Vector3d r_CelN_A, r_CelA_A;
    Eigen::Vector3d n_A_A = Eigen::Vector3d::UnitZ(); // antenna boresight in antenna frame
    if (this->environment == AntennaTypes::EnvironmentType::ENVIRONMENT_SPACE) {
        // Space based antenna
        if (this->spicePlanetStateInMsg.isLinked()) {
            // Spice message is linked
            T_sky = CMB_TEMPERATURE + 10.0;  // Cosmic Microwave Background temperature + 10K for other background sources (earth / sun / moon / distance celestial bodies)
            // TODO future expansion
            // 1: Calculate how much (solid angle) of the antenna FOV is occupied by celestial body(s)
            // 2: Get the temperature of the celestial body(s) (from spice or constant?)
            // 3: Intgrate of the FOV to get T_sky
        } else {                                       // Spice message is NOT linked
            // No spice message, set to uniform CMB temperature
            T_sky = CMB_TEMPERATURE + 10.0;  // Cosmic Microwave Background temperature + 10K for other background sources (earth / sun / moon / distance celestial bodies)
        }
    } else if (this->environment == AntennaTypes::EnvironmentType::ENVIRONMENT_EARTH) {
        // Ground based antenna
        T_sky = CMB_TEMPERATURE + 10.0; // Simplified model (TODO, improve and make it dependent on pointing direction)
        // 1: Calculate temperature of T_sky based on the antenna pointing direction relative to horizon
    }
    return T_sky;
}

double SimpleAntenna::calculateTambient()
{
    double T_A;
    if (this->environment == AntennaTypes::EnvironmentType::ENVIRONMENT_SPACE) {
        // Space environment
        T_A = 3.0; // TODO improve with "eclipse" conditions based on sun/earth/moon position
    } else if (this->environment == AntennaTypes::EnvironmentType::ENVIRONMENT_EARTH) {
        // Ground environment
        // Simplified ISA model (TODO, improve with real atmospheric model)
        double altitude = sqrt(this->groundState.r_LP_N[0] * this->groundState.r_LP_N[0] +
                               this->groundState.r_LP_N[1] * this->groundState.r_LP_N[1] +
                               this->groundState.r_LP_N[2] * this->groundState.r_LP_N[2]) - REQ_EARTH; // [m] Altitude above sea level
        T_A             = 288.15 - 0.0065 * altitude; // [K] Ambient temperature based on altitude (ISA) (TODO IMPROVE)
    }
    return T_A;
}

void SimpleAntenna::populateOutputMsg(AntennaOutMsgPayload *output)
{
    std::strncpy(output->antennaID, this->antennaID, sizeof(output->antennaID));
    output->r_AN_N[0]      = this->r_AN_N[0];
    output->r_AN_N[1]      = this->r_AN_N[1];
    output->r_AN_N[2]      = this->r_AN_N[2];
    output->sigma_NA[0]    = this->sigma_NA.x();
    output->sigma_NA[1]    = this->sigma_NA.y();
    output->sigma_NA[2]    = this->sigma_NA.z();
    output->v_AN[0]        = this->v_AN[0];
    output->v_AN[1]        = this->v_AN[1];
    output->v_AN[2]        = this->v_AN[2];
    output->P_eirp         = this->P_eirp;
    output->P_N            = this->P_N;
    output->B              = this->B;
    output->frequency      = this->frequency;
    output->antennaState   = static_cast<uint32_t>(this->antennaState);
    output->environment    = static_cast<uint32_t>(this->environment);
    if (this->environment == AntennaTypes::EnvironmentType::ENVIRONMENT_SPACE) {
        // Space set amb Temperature only
        output->T_amb       = this->T_amb;
        output->p_amb       = 0.0;
        output->rho_amb     = 0.0;
        output->x_o2        = 0.0;
        output->x_h2o       = 0.0;
        output->r_AP_N[0]   = 0.0;
        output->r_AP_N[1]   = 0.0;
        output->r_AP_N[2]   = 0.0;
    } else if (this->environment == AntennaTypes::EnvironmentType::ENVIRONMENT_EARTH) {
        // Ground set all environment properties
        output->T_amb       = this->T_amb;
        output->p_amb       = this->p_amb;
        output->rho_amb     = this->rho_amb;
        output->x_o2        = this->x_o2;
        output->x_h2o       = this->x_h2o;
        output->r_AP_N[0]   = this->r_AP_N[0];
        output->r_AP_N[1]   = this->r_AP_N[1];
        output->r_AP_N[2]   = this->r_AP_N[2];
    } else {
        // Raise error (should not happen due to checks in Reset)
        bskLogger.bskLog(BSK_ERROR, "SimpleAntenna: Unable to populate output message due to unknown environment.");
    }

}

void SimpleAntenna::calculateAntennaPositionAndOrientation()
{
    if (this->environment == AntennaTypes::EnvironmentType::ENVIRONMENT_SPACE) {
        // Space environment
        // Calculate the antenna position in N frame
        Eigen::MRPd     sigma_BN  = cArray2EigenMRPd(this->scState.sigma_BN);           // Convert c-array to Eigen MRP
        Eigen::Matrix3d dcm_NB    = sigma_BN.toRotationMatrix().transpose();            // Calculate DCM_NB from MRP_BN
        Eigen::Vector3d r_AB_N    = dcm_NB * this->r_AB_B;                              // Antenna position on SC decomposed in N-frame:         r_AB_N = dcm_NB * r_AB_B
        Eigen::Vector3d r_BN_N    = cArray2EigenVector3d(this->scState.r_BN_N);         // Convert c-array to Eigen Vector3d
        this->r_AN_N              = r_BN_N + dcm_NB * this->r_AB_B;                     // Antenna position in N-frame, decomposed in N-frame:   r_AN_N = r_AB_N + r_BN_N

        // Calculate the antenna orientation in N frame
        this->dcm_NA              = dcm_NB * this->sigma_BA.toRotationMatrix();            // DCM from A-frame to N-frame:   dcm_AN = dcm_NB * dcm_BA
        this->sigma_NA            = eigenC2MRP(this->dcm_NA);                              // Convert DCM to MRP

        // Antenna boresight vector in N frame
//        this->n_A_N               = this->dcm_NA * Eigen::Vector3d::UnitZ();                    // boresight along antenna z-axis

        // Calculate the antenna velocity in the N frame (stub )
        this->v_AN = cArray2EigenVector3d(this->scState.v_BN_N);                                              //TODO Future implementation (transport theorem) (for now antenna velocity = spacecraft velocity)
    } else if (this->environment == AntennaTypes::EnvironmentType::ENVIRONMENT_EARTH) {
        // Ground environment
        // Antenna position and orientation are defined in the groundState message
        this->r_AN_N   = cArray2EigenVector3d(this->groundState.r_LN_N);                // TODO check if this is correct (Antenna position with respect to L frame?)
        this->v_AN     = Eigen::Vector3d::Zero();                                       // TODO groundState message does not have velocity yet should be velocity of ground station in N frame

        this->r_AP_N   = cArray2EigenVector3d(this->groundState.r_LP_N);
    }
}

void SimpleAntenna::calculateAntennaNoise()
{
    // Calculate antenna noise temperature based on T_antenna, T_amb
    // Simplified model (TODO, improve with atmospheric absorption model)
    double T_sky, T_A;
    T_sky = SimpleAntenna::calculateTsky();
    if (this->T_amb == -1) {
        // T_amb not set by user, calculate T_A
        T_A =SimpleAntenna::calculateTambient();
    } else {
        // T_amb set by user
        T_A = this->T_amb;
    }
    double T_ant = (1 - this->eta_r) * T_A + T_sky; // Antenna noise temperature
    double T_S   = this->T_E + T_ant;               // System noise temperature
    this->P_N    = K_BOLTZMANN * T_S * this->B;     // Noise power
}

void SimpleAntenna::evaluatePowerModel(PowerNodeUsageMsgPayload *powerUsage){
    if (this->antennaState == AntennaTypes::AntennaStateEnum::ANTENNA_OFF) {
        // Off
        powerUsage->netPower = 0.0;
    } else if (this->antennaState == AntennaTypes::AntennaStateEnum::ANTENNA_RX) {
        // Rx
        powerUsage->netPower = -this->P_Rx;
    } else if (this->antennaState == AntennaTypes::AntennaStateEnum::ANTENNA_TX) {
        // Tx
        powerUsage->netPower = -this->P_Tx;
    } else if (this->antennaState == AntennaTypes::AntennaStateEnum::ANTENNA_RXTX) {
        // RxTx
        powerUsage->netPower = -(this->P_Rx + this->P_Tx);
    } else {
        bskLogger.bskLog(BSK_ERROR, "SimpleAntenna: Invalid antenna state.");
        powerUsage->netPower = 0.0;
    }
}

/*******************/
/***** Setters *****/
/*******************/
void SimpleAntenna::setAntennaName(const char* name) {
    strncpy(this->antennaID, name, sizeof(this->antennaID) - 1);
    this->antennaID[sizeof(this->antennaID) - 1] = '\0';  // Ensure null-termination
}

void SimpleAntenna::setAntennaState(AntennaTypes::AntennaStateEnum var) {
    if (var < AntennaTypes::AntennaStateEnum::ANTENNA_OFF || var > AntennaTypes::AntennaStateEnum::ANTENNA_RXTX) {
        bskLogger.bskLog(BSK_WARNING, "SimpleAntenna: Attempting to set invalid antenna state. Value must be between 0 (Off), 1 (Rx), 2 (Tx), 3 (RxTx). Setting to OFF (0).");
        this->antennaState = AntennaTypes::AntennaStateEnum::ANTENNA_OFF;
        return;
    }
    this->antennaState = var;  // [-] antenna state: 0: off, 1: Rx, 2: Tx, 3: RxTx
}
void SimpleAntenna::setAntennaGain_dB(double var) {
    if (var < 0) {
        bskLogger.bskLog(BSK_WARNING, "SimpleAntenna: Attempting to set invalid antenna gain G[dB]. Value must be non-negative. Setting to default value of 0 dB (omnidirectional antenna).");
        this->G = 0;
        return;
    }
    this->G = var;             // [dB] antenna gain
}
void SimpleAntenna::setAntennaFrequency(double var) {
    if (var <= 0) {
        bskLogger.bskLog(BSK_WARNING, "SimpleAntenna: Attempting to set invalid antenna frequency [Hz]. Value must be positive. Setting to default value of 3e9 Hz (S-band).");
        this->frequency = 3e9;
        return;
    }
    this->frequency = var;     // [Hz] antenna operating frequency
}
void SimpleAntenna::setAntennaHpbwRatio(double var) { // TODO check if HPBW can be used to calculate G for non-isotropic antennas
    if (var <= 0) {
        bskLogger.bskLog(BSK_WARNING, "SimpleAntenna: Attempting to set invalid antenna HPBW ratio. Value must be positive. Setting to default value of 1.0.");
        this->k = 1.0;
        return;
    }
    this->k = var;
}
void SimpleAntenna::setAntennaP_Tx(double var) {
    if (var < 0) {
        bskLogger.bskLog(BSK_WARNING, "SimpleAntenna: Attempting to set invalid transmit power P_Tx [W]. Value must be non-negative. Setting to default value of 100 W.");
        this->P_Tx = 100;
        return;
    }
    this->P_Tx = var;          // [W] transmit power
}
void SimpleAntenna::setAntennaP_Rx(double var) {
    if (var < 0) {
        bskLogger.bskLog(BSK_WARNING, "SimpleAntenna: Attempting to set invalid receive power P_Rx [W]. Value must be non-negative. Setting to default value of 10 W.");
        this->P_Rx = 10;
        return;
    }
    this->P_Rx = var;          // [W] receive power
}
void SimpleAntenna::setAntennaBandwidth(double var) {
    if (var <= 0) {
        bskLogger.bskLog(BSK_WARNING, "SimpleAntenna: Attempting to set invalid bandwidth B [Hz]. Value must be positive. Setting to default value of 30 MHz. (typical for S-band)");
        this->B = 30e6;
        return;
    }
    this->B = var;             // [Hz] antenna frequency bandwidth
}
void SimpleAntenna::setAntennaT_E(double var) {
    this->T_E = var;           // [K] equivalent noise temperature
}
void SimpleAntenna::setAntennaRadEfficiency(double var) {
    if (var < 0 || var > 1) {
        bskLogger.bskLog(BSK_WARNING, "SimpleAntenna: Attempting to set invalid antenna radiation efficiency eta_r. Value must be between 0 and 1. Setting to default value of 1.0.");
        this->eta_r = 1.0;
        return;
    }
    this->eta_r = var;         // [-] antenna radiation efficiency
}
void SimpleAntenna::setAntennaEnvTemperature(double var) {
    if (var < 0) {
        bskLogger.bskLog(BSK_WARNING, "SimpleAntenna: Attempting to set invalid environment temperature T_amb [K]. Value must be non-negative. Setting to default value of 100 K.");
        this->T_amb = 100.0;
        return;
    }
    this->T_amb = var;
}
void SimpleAntenna::setAntennaEnvPressure(double var) {
    if (var < 0) {
        bskLogger.bskLog(BSK_WARNING, "SimpleAntenna: Attempting to set invalid environment pressure p_amb [Pa]. Value must be non-negative. Setting to default value of 0 Pa.");
        this->p_amb = 0;
        return;
    }
    this->p_amb = var;
}
void SimpleAntenna::setAntennaEnvHumidity(double var) {
    if (var < 0) {
        bskLogger.bskLog(BSK_WARNING, "SimpleAntenna: Attempting to set invalid environment humidity rho_amb [-]. Value must be non-negative. Setting to default value of 0 g/m^3.");
        this->rho_amb = 0;
        return;
    }
    this->rho_amb = var;
}
void SimpleAntenna::setAntennaEnvX_o2(double var) {
    // oxygen partial pressure/ambient pressure [-]
    this->x_o2 = var;
}
void SimpleAntenna::setAntennaEnvX_h2o(double var) {
    this->x_h2o = var;
}
void SimpleAntenna::setAntennaPositionBodyFrame(Eigen::Vector3d var) {
    this->r_AB_B = var;
}
void SimpleAntenna::setAntennaOrientationBodyFrame(Eigen::Vector3d var) {
    this->sigma_BA = var;
}
