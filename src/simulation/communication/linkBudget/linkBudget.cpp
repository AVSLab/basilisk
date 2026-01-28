/*
 ISC License

 Copyright (c) 2025, Department of Engineering Cybernetics, NTNU

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

 - POSSIBLE FUTURE EXPANSION: Implement Impedance mismatch loss
 - POSSIBLE FUTURE EXPANSION: Implement polarization change due to atmosphere/ionosphere
 - POSSIBLE FUTURE EXPANSION: Implement rain attenuation                  (ITU-R P.838)
 - POSSIBLE FUTURE EXPANSION: Implement fog/clouds attenuation            (ITU-R P.840)
 - POSSIBLE FUTURE EXPANSION: Implement ionospheric scintillation effects (ITU-R P.618)
 - POSSIBLE FUTURE EXPANSION: Implement other ionospheric effects         (ITU-R P.531)
*/

#include "simulation/communication/linkBudget/linkBudget.h"
#include "simulation/communication/_GeneralModuleFiles/AntennaDefinitions.h"
#include "architecture/utilities/ItuRefAtmosphere.h"
#include <iostream>
#include <cstring>
#include <cmath>

/*! This is the constructor for the module class. It sets default variable
    values and initializes the various parts of the model */
LinkBudget::LinkBudget()
{
    // initialize module variables
    this->antennaPlacement  = LinkBudgetTypes::AntennaPlacement::_UNKNOWN_BLOCKED; // [-] Antenna placement type
    this->gndAntPnt         = nullptr;  // [-]  Ground antenna index (0 if none, 1 if ant 1 is ground, 2 if ant 2 is ground)
    this->scAntPnt          = nullptr;  // [-]  Spacecraft antenna index (0 if none, 1 if ant 1 is space, 2 if ant 2 is space)
    this->distance          = {};       // [m]  Distance between antennas
    this->L_FSPL            = {};       // [dB] Free space path loss
    this->L_atm             = {0};      // [dB] Atmospheric attenuation loss
    this->L_freq            = {0};      // [dB] Frequency offset loss
    this->L_point           = {0};      // [dB] Pointing mismatch loss
    this->P_Rx1             = {};       // [W]  Received power at antenna 1 TODO Check there is no misup with P_Rx power demanded by Antenna module
    this->P_Rx2             = {};       // [W]  Received power at antenna 2
    this->CNR1              = {-1.0};   // [-]  Carrier-to-noise ratio for antenna 1
    this->CNR2              = {-1.0};   // [-]  Carrier-to-noise ratio for antenna 2
    this->B_overlap         = {};       // [Hz] Bandwidth overlap between the two antennas. If frequency loss calc. disabled, assume full bandwidth overlap
    this->centerfreq        = {};       // [Hz] Center frequency of the two antennas
    this->atmosAtt          = false;    // [-]  Disable atmospheric attenuation by default (default value for L_atm = 0 dB, can be set by setter)
    this->pointingLoss      = true;     // [-]  Enable pointing loss by default (default value for L_point calculated, if disabled L_point = 0 dB & settable set by setter)
    this->freqLoss          = true;     // [-]  Enable frequency offset loss by default (default value for L_freq calculated, if disabled L_freq = 0 dB & settable set by setter)
    this->oxygenLookup      = {};       // [-]  Oxygen absorption coefficients lookup table filled with values in initialization (ITU-R P.676-13)
    this->waterVaporLookup  = {};       // [-]  Water vapor absorption coefficients lookup table filled with values in initialization (ITU-R P.676-13)
    this->h_0               = {-1};     // [m]  Altitude of ground antenna (default -1: uninitialized)
    this->n_0               = {-1};     // [-]  Refractivity at ground antenna altitude (default -1: uninitialized)
    this->linkValid         = false;    // [-]  Flag indicating if the link is valid (sufficient bandwidth overlap, antennas in correct states, etc.)
}

/*! This method is used to reset the module and checks that required input messages are connect.
*/
void LinkBudget::Reset(uint64_t CurrentSimNanos)
{
    // check that required input messages are connected
    if (!this->antennaInPayload_1.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "LinkBudget.antennaInPayload_1 was not linked.");
    }
    if (!this->antennaInPayload_2.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "LinkBudget.antennaInPayload_2 was not linked.");
    }
    this->initialization();
}

void LinkBudget::initialization()
{
    // Read current message values before using them
    this->readMessages();

    // Determine antenna environments
    this->env1 = static_cast<AntennaTypes::EnvironmentType>(this->antennaIn_1.environment);
    this->env2 = static_cast<AntennaTypes::EnvironmentType>(this->antennaIn_2.environment);

    // Determine link-budget situation
    if (this->env1 == AntennaTypes::EnvironmentType::ENVIRONMENT_EARTH &&
        this->env2 == AntennaTypes::EnvironmentType::ENVIRONMENT_EARTH) {
        // -------- Both antennas are on EARTH --> not supported --------
        this->antennaPlacement = LinkBudgetTypes::AntennaPlacement::_UNKNOWN_BLOCKED;
        bskLogger.bskLog(BSK_ERROR, "This module does currently not support link budget calculations between two ground stations.");
        return;
    } else if (this->env1 == AntennaTypes::EnvironmentType::ENVIRONMENT_EARTH &&
               this->env2 == AntennaTypes::EnvironmentType::ENVIRONMENT_SPACE) {
        // -------- Antenna 1 is on EARTH, antenna 2 is in SPACE --------
        this->antennaPlacement = LinkBudgetTypes::AntennaPlacement::SPACE_EARTH;
        this->gndAntPnt        = &this->antennaIn_1;
        this->scAntPnt         = &this->antennaIn_2;
    } else if (this->env1 == AntennaTypes::EnvironmentType::ENVIRONMENT_SPACE &&
               this->env2 == AntennaTypes::EnvironmentType::ENVIRONMENT_EARTH) {
        // -------- Antenna 2 is on EARTH, antenna 1 is in SPACE --------
        this->antennaPlacement = LinkBudgetTypes::AntennaPlacement::SPACE_EARTH;
        this->gndAntPnt        = &this->antennaIn_2;
        this->scAntPnt         = &this->antennaIn_1;
    } else if (this->env1 == AntennaTypes::EnvironmentType::ENVIRONMENT_SPACE &&
               this->env2 == AntennaTypes::EnvironmentType::ENVIRONMENT_SPACE) {
        // -------- Both antennas are in SPACE --------
        this->antennaPlacement = LinkBudgetTypes::AntennaPlacement::SPACE_SPACE;
        this->atmosAtt         = false;    // disable atmospheric attenuation for Space-Space links
        this->gndAntPnt        = nullptr;
        this->scAntPnt         = nullptr;
    } else {
        // robustness case: unknown antenna environments
        this->antennaPlacement = LinkBudgetTypes::AntennaPlacement::_UNKNOWN_BLOCKED;
        bskLogger.bskLog(BSK_ERROR, "LinkBudget: Unable to determine antenna placement type.");
        return;
    }

    // If atmospheric attenuation is enabled, calculate the ground antenna parameters (fixed values for ground antenna)
    if (this->atmosAtt && this->gndAntPnt != nullptr) {
        // Calculate h_0 and alpha_0 for the ground antenna (For now assuming Ground Enviroment -> EARTH [This needs to be updated if more planets are added])
        this->h_0    = sqrt(this->gndAntPnt->r_AP_N[0]*this->gndAntPnt->r_AP_N[0] +
                            this->gndAntPnt->r_AP_N[1]*this->gndAntPnt->r_AP_N[1] +
                            this->gndAntPnt->r_AP_N[2]*this->gndAntPnt->r_AP_N[2]) - REQ_EARTH *1e3;
        // Atmospheric conditions at ground antenna altitude
        double T_0   = ItuAtmosphere::getTempISA(this->h_0);               // [K]     Temperature at ground antenna altitude
        double P_0   = ItuAtmosphere::getPresISA(this->h_0);               // [hPa]   Pressure at ground antenna altitude
        double rho_0 = ItuAtmosphere::getWaterVapDensityISA(this->h_0);    // [g/m3]  Water vapor density at ground antenna altitude
        double e_0   = (rho_0 * T_0) / (216.7);                            // [hPa]   Water vapor partial pressure at ground antenna altitude (ITU-R P.453-14 eq: 10)
        // Refractivity at ground antenna altitude
        n_0 = 1 + (77.6 * (P_0 / T_0) - 5.6 * (e_0 /T_0) + 3.75e5 * (e_0/ (T_0 * T_0))) * 1e-6; // [-] (ITU-R P.453-14 eq: 1 & eq: 6)

        // (done once at initialization, during simulation lookup)
        this->generateLookupTable(LinkBudgetTypes::GasType::OXYGEN,      &this->oxygenLookup);
        this->generateLookupTable(LinkBudgetTypes::GasType::WATER_VAPOR, &this->waterVaporLookup);

        this->precomputeAtmosphericAttenuationAtLayers(&this->attenuationLookup, this->gndAntPnt->frequency);
    }
}

/*! This is the main method that gets called every time the module is updated.  Provide an appropriate description.
*/
void LinkBudget::UpdateState(uint64_t CurrentSimNanos)
{
    // read in the input messages
    this->readMessages();

    // FUTURE IMPROVEMENT: calculate doppler shift here

    // Calculate link budget parameters
    this->calculateLinkBudget();
    this->calculateCNR();
    this->writeOutputMessages(CurrentSimNanos);
}

void LinkBudget::readMessages()
{
    // read in the input messages
    this->antennaIn_1         = this->antennaInPayload_1();
    this->antennaIn_2         = this->antennaInPayload_2();
}

void LinkBudget::calculateLinkBudget()
{
    if (this->freqLoss) {
        // Calculate frequency offset loss only if enabled
        this->calculateFrequencyOffsetLoss();
    } else {
        // If frequency offset loss calculation is disabled, calculate
        // bandwidth overlap and center frequency for FSPL calculation
        this->B_overlap = std::min(this->antennaIn_1.B, this->antennaIn_2.B);                 // [Hz] If frequency loss disabled, assume full bandwidth overlap
        this->centerfreq = (this->antennaIn_1.frequency + this->antennaIn_2.frequency) / 2.0; // [Hz] Average frequency
        if (this->B_overlap > 0.0) {
            this->linkValid = true;
        }
    }
    // Free Space Path Loss is always calculated
    this->calculateFSPL();
    if (this->atmosAtt) {
        // Calculate atmospheric loss only if enabled
        this->calculateAtmosphericLoss();
    }
    if (this->pointingLoss) {
        // Calculate pointing loss only if enabled
        this->calculatePointingLoss();
    }
}
void LinkBudget::calculateFSPL()
{
    double dx       = this->antennaIn_1.r_AN_N[0] - this->antennaIn_2.r_AN_N[0];
    double dy       = this->antennaIn_1.r_AN_N[1] - this->antennaIn_2.r_AN_N[1];
    double dz       = this->antennaIn_1.r_AN_N[2] - this->antennaIn_2.r_AN_N[2];
    this->distance  = std::sqrt(dx*dx + dy*dy + dz*dz); // [m]
    if (this->distance <= 0.0) {
        bskLogger.bskLog(BSK_ERROR, "LinkBudget: Distance between antennas is zero or negative.");
    }
    if (this->distance <= 10.0*1e3) {
        bskLogger.bskLog(BSK_WARNING, "LinkBudget: You are calculating a linkBudget for a distance below 10 km. the gaussian-beam assumption might be inaccurate at this range.");
    }
    double lambda    = SPEED_LIGHT / this->centerfreq;
    this->L_FSPL     = 20.0 * std::log10((4.0 * MPI * distance) / lambda); // 20*log10(4*pi*d / lambda)
}
void LinkBudget::calculateAtmosphericLoss()
{
    double h_i;                           // [m]             Altitude, ground station altitude (above spherical earth surface)
    double gamma_i;                       // [dB/km]         Specific attenuation for layer i
    double delta_h_layer_i;               // [m]             Layer thickness
    double n_i;                           // [-]             Refractivity at altitude h_i
    double attenuation      = 0.0;        // [dB]            Total atmospheric attenuation at altitude h_i (initialized to zero)

    // Calculate slant angle at the ground antenna location
    Eigen::Vector3d r_AgN_N = {this->gndAntPnt->r_AN_N[0],     this->gndAntPnt->r_AN_N[1],    this->gndAntPnt->r_AN_N[2]};   // Position of gnd antenna wrt to {N} frame in {N} frame
    Eigen::Vector3d r_AsN_N = {this->scAntPnt->r_AN_N[0],      this->scAntPnt->r_AN_N[1],     this->scAntPnt->r_AN_N[2]};    // Position of sc antenna wrt {A} frame in {N} frame
    Eigen::Vector3d n_Ag_n  = {this->gndAntPnt->nHat_LP_N [0], this->gndAntPnt->nHat_LP_N [1], this->gndAntPnt->nHat_LP_N [2]}; // n-vector at antenna location in {N} frame
    double cosSlant_i       = this->calcCosSlantGround(r_AgN_N, n_Ag_n, r_AsN_N);                                            // [cos(rad)] Cosinus of slant angle at the ground

    // Check initial elevation angle and warn once if below minimum
    double sinSlant_i = std::sqrt(std::max(0.0, 1.0 - cosSlant_i * cosSlant_i));
    if (sinSlant_i < MIN_SIN_ELEVATION) {
        bskLogger.bskLog(BSK_WARNING, "LinkBudget: Elevation angle below 5 deg, clamping for atmospheric calculation.");
    }

    // Loop to loop through AttenuationLookupTable until top of atmosphere is reached
    for (size_t i=0; i < this->attenuationLookup.layer_i.size(); i++) {
        h_i             = this->attenuationLookup.layer_i[i];         // [m]     Altitude layer i
        gamma_i         = this->attenuationLookup.gamma_i[i];         // [dB/km] Attenuation value at altitude layer i
        delta_h_layer_i = this->attenuationLookup.delta_h_layer_i[i]; // [m]     Thickness of altitude layer i (ITU-R P.676-13 eq 16a)
        n_i             = this->attenuationLookup.n_i[i];             // [-]     Refractivity at altitude layer i

        sinSlant_i = std::sqrt(std::max(0.0, 1.0 - cosSlant_i * cosSlant_i));
        if (sinSlant_i < MIN_SIN_ELEVATION) {
            sinSlant_i = MIN_SIN_ELEVATION;
        }

        // Total attenuation at layer i
        attenuation += gamma_i * delta_h_layer_i * 1e-3 / sinSlant_i; // [dB] (ITU-R P.676-13 eq 11)
        // calculate cosine slant angle for the next layer (ITU-R P.676-13 eq. 12)
        cosSlant_i = ((REQ_EARTH * 1e3 + h_0) * n_0)/((REQ_EARTH * 1e3 + h_i) * n_i) * cosSlant_i;  // [-] Cosinus of slant angle at altitude h_i
    }
    this->L_atm = attenuation; // [dB] Total atmospheric attenuation
}
/* Slant angle calculation at ground antenna location (helper fuction for atmospheric attenuation calculation)
*/
double LinkBudget::calcCosSlantGround(Eigen::Vector3d r_AgN_N, Eigen::Vector3d n_Ag_N, Eigen::Vector3d r_AsN_N)
{
    // (ITU-R P.676-13 eq 12)
    Eigen::Vector3d n_AsAg_N = (r_AsN_N - r_AgN_N).normalized(); // Vector from ground antenna to spacecraft in {N} frame
    double          v_n      = n_AsAg_N.dot(n_Ag_N.normalized());
    double          v_t      = std::sqrt(1.0 - v_n*v_n);
    double          alpha0   = std::atan2(v_n, v_t);
    return std::cos(alpha0);
}
/* getPointingError: Calculate pointing error between two antennas (how much off axis to the other antenna in azimuth and elevation)
*/
Eigen::Vector2d LinkBudget::getPointingError(Eigen::Vector3d r_A1N_N, Eigen::Vector3d r_A2N_N, Eigen::MRPd sigma_AN1)
{
    Eigen::Matrix3d dcm_NA1   = sigma_AN1.toRotationMatrix();               // DCM from {A1} to {N} frame
    // Calculate the unit vector from antenna 1 to antenna 2
    Eigen::Vector3d n_A2A1_A1 = dcm_NA1 * (r_A2N_N - r_A1N_N).normalized();
    // boresight vector of antenna 1 in {A1} frame
    Eigen::Vector3d n_A1_A1   = Eigen::Vector3d::UnitZ();

    double azimuthError   = std::atan2(n_A2A1_A1[0], n_A2A1_A1[2]);  // [rad] Azimuth pointing error // TODO protect against division by zero
    double elevationError = std::atan2(n_A2A1_A1[1], n_A2A1_A1[2]);  // [rad] Elevation pointing error // TODO protect against division by zero
    return {azimuthError, elevationError};
}
/* Calculate pointing loss based on pointing errors of both antennas
   -> This function assumes a 'gaussian' antenna pattern' in the main lobe / far-field
*/
void LinkBudget::calculatePointingLoss()
{
    // Check if one antenna is on the ground
    if (this->gndAntPnt == nullptr) {
        // No ground antenna

        Eigen::Vector2d errors1 = this->getPointingError(cArray2EigenVector3d(this->antennaIn_1.r_AN_N), \
                                                         cArray2EigenVector3d(this->antennaIn_2.r_AN_N), \
                                                         cArray2EigenMRPd(this->antennaIn_1.sigma_AN));
        Eigen::Vector2d errors2 = this->getPointingError(cArray2EigenVector3d(this->antennaIn_2.r_AN_N), \
                                                         cArray2EigenVector3d(this->antennaIn_1.r_AN_N), \
                                                         cArray2EigenMRPd(this->antennaIn_2.sigma_AN));
        double L_point1 = 10 * std::log10(std::exp(1.0)) * 4.0 * std::log(2.0) * \
                          ((errors1[0]*errors1[0])/(this->antennaIn_1.HPBW_az*this->antennaIn_1.HPBW_az) + \
                          (errors1[1]*errors1[1])/(this->antennaIn_1.HPBW_el*this->antennaIn_1.HPBW_el)); // [dB] Pointing loss
        double L_point2 = 10 * std::log10(std::exp(1.0)) * 4.0 * std::log(2.0) * \
                          ((errors2[0]*errors2[0])/(this->antennaIn_2.HPBW_az*this->antennaIn_2.HPBW_az) + \
                          (errors2[1]*errors2[1])/(this->antennaIn_2.HPBW_el*this->antennaIn_2.HPBW_el)); // [dB] Pointing loss
        this->L_point   = L_point1 + L_point2;
        return;
    } else if (this->gndAntPnt != nullptr && this->scAntPnt != nullptr) {
        // Ground antenna is assumed to be perfectly pointed towards the spacecraft -> No pointing loss from ground antenna
        Eigen::Vector2d errors = this->getPointingError(cArray2EigenVector3d(this->scAntPnt->r_AN_N), \
                                                        cArray2EigenVector3d(this->gndAntPnt->r_AN_N), \
                                                        cArray2EigenMRPd(this->scAntPnt->sigma_AN));
        this->L_point = 10 * std::log10(std::exp(1.0)) * 4.0 * std::log(2.0) * \
                    ((errors[0]*errors[0])/(this->scAntPnt->HPBW_az*this->scAntPnt->HPBW_az) + \
                    (errors[1]*errors[1])/(this->scAntPnt->HPBW_el*this->scAntPnt->HPBW_el)); // [dB] Pointing loss
        return;
    }
}
void LinkBudget::calculateFrequencyOffsetLoss()
{
    // lower freqency (take the highest lower frequency)
    double f_low     = std::max(this->antennaIn_1.frequency - this->antennaIn_1.B/2.0,
                                this->antennaIn_2.frequency - this->antennaIn_2.B/2.0);
    // upper frequency (take the lowest upper frequency)
    double f_high    =  std::min(this->antennaIn_1.frequency + this->antennaIn_1.B/2.0,
                                 this->antennaIn_2.frequency + this->antennaIn_2.B/2.0);
    this->B_overlap  = f_high - f_low;          // overlapping bandwidth
    this->centerfreq = (f_high + f_low) / 2.0; // Update center frequency of the overlapping bandwidth

    // Check if the bandwidth is smaller than the the bandwidth of the antenna with the smallest bandwidth
    double smaller_bandwidth = std::min(this->antennaIn_1.B, this->antennaIn_2.B);

    if (this->B_overlap <= smaller_bandwidth && this->B_overlap > 0.0) {
        // Partial overlapping bandwidth -> calculate frequency offset loss
        this->L_freq = 10.0 * std::log10(this->B_overlap / smaller_bandwidth); // [dB] Frequency offset loss
        this->linkValid = true;
        return;
    } else if (this->B_overlap <= 0.0) {
        // No overlapping bandwidth -> 100% loss
        bskLogger.bskLog(BSK_WARNING, "LinkBudget: No overlapping bandwidth between the two antennas.");
        this->linkValid = false;
        return;
    } else {
        // No frequency offset loss (this should be the default case for a well designed link)
        this->L_freq = 0.0;
        this->linkValid = true;
        return;
    }
}
void LinkBudget::generateLookupTable(LinkBudgetTypes::GasType gasType, LookupTable* lookupTable)
{
    std::string filename;
    std::string line;
    bool        inDataSection = false;

    // Determine which file to read based on gas type
    if (gasType == LinkBudgetTypes::GasType::OXYGEN) {
        filename = "supportData/AtmRadioFreqDampData/oxygen.json";
    } else if (gasType == LinkBudgetTypes::GasType::WATER_VAPOR) {
        filename = "supportData/AtmRadioFreqDampData/waterVapour.json";
    } else {
        bskLogger.bskLog(BSK_ERROR, "LinkBudget: Invalid gas type for lookup table generation.");
        return;
    }
    // Clear any existing data
    lookupTable->f_l.clear();
    lookupTable->l_1.clear();
    lookupTable->l_2.clear();
    lookupTable->l_3.clear();
    lookupTable->l_4.clear();
    lookupTable->l_5.clear();
    lookupTable->l_6.clear();

    // Read JSON file
    std::ifstream file(filename);
    if (!file.is_open()) {
        bskLogger.bskLog(BSK_ERROR, "LinkBudget: Unable to open lookup table file for oxygen or water vapor lookup table generation.");
        return;
    }

    while (std::getline(file, line)) {
        // Skip empty lines and whitespace
        if (line.find_first_not_of(" \t\r\n") == std::string::npos) {
            continue;
        }

        // Skip forward to "data" section
        if (line.find("\"data\"") != std::string::npos) {
            inDataSection = true;
            continue;
        }

        // Parse data entries
        if (inDataSection) {
            // Check for end of data array
            if (line.find("]") != std::string::npos) {
                break;
            }

            // Skip lines that don't contain data (should not happen)
            if (line.find("f0") == std::string::npos) {
                continue;
            }

            // Extract values from the JSON line
            double f0 = 0, c1 = 0, c2 = 0, c3 = 0, c4 = 0, c5 = 0, c6 = 0;
            size_t pos = line.find("\"f0\":");

            if (gasType == LinkBudgetTypes::GasType::OXYGEN) {
                // Parse oxygen data line
                size_t pos = line.find("\"f0\":");
                if (pos != std::string::npos || sscanf(line.c_str() + pos, "\"f0\": %lf", &f0) == 1) {
                    bskLogger.bskLog(BSK_ERROR, "LinkBudget: Error parsing f0 value in oxygen lookup table.");
                }
                pos = line.find("\"a1\":");
                if (pos != std::string::npos || sscanf(line.c_str() + pos, "\"a1\": %lf", &c1) == 1) {
                    bskLogger.bskLog(BSK_ERROR, "LinkBudget: Error parsing a1 value in oxygen lookup table.");
                }
                pos = line.find("\"a2\":");
                if (pos != std::string::npos || sscanf(line.c_str() + pos, "\"a2\": %lf", &c2) == 1) {
                    bskLogger.bskLog(BSK_ERROR, "LinkBudget: Error parsing a2 value in oxygen lookup table.");
                }
                pos = line.find("\"a3\":");
                if (pos != std::string::npos || sscanf(line.c_str() + pos, "\"a3\": %lf", &c3) == 1) {
                    bskLogger.bskLog(BSK_ERROR, "LinkBudget: Error parsing a3 value in oxygen lookup table.");
                }
                pos = line.find("\"a4\":");
                if (pos != std::string::npos || sscanf(line.c_str() + pos, "\"a4\": %lf", &c4) == 1) {
                    bskLogger.bskLog(BSK_ERROR, "LinkBudget: Error parsing a4 value in oxygen lookup table.");
                }
                pos = line.find("\"a5\":");
                if (pos != std::string::npos || sscanf(line.c_str() + pos, "\"a5\": %lf", &c5) == 1) {
                    bskLogger.bskLog(BSK_ERROR, "LinkBudget: Error parsing a5 value in oxygen lookup table.");
                }
                pos = line.find("\"a6\":");
                if (pos != std::string::npos || sscanf(line.c_str() + pos, "\"a6\": %lf", &c6) == 1) {
                    bskLogger.bskLog(BSK_ERROR, "LinkBudget: Error parsing a6 value in oxygen lookup table.");
                }
            } else {
                // Parse water vapor data line
                size_t pos = line.find("\"f0\":");
                if (pos != std::string::npos || sscanf(line.c_str() + pos, "\"f0\": %lf", &f0) == 1) {
                    bskLogger.bskLog(BSK_ERROR, "LinkBudget: Error parsing f0 value in water vapor lookup table.");
                }
                pos = line.find("\"b1\":");
                if (pos != std::string::npos || sscanf(line.c_str() + pos, "\"b1\": %lf", &c1) == 1) {
                    bskLogger.bskLog(BSK_ERROR, "LinkBudget: Error parsing b1 value in water vapor lookup table.");
                }
                pos = line.find("\"b2\":");
                if (pos != std::string::npos || sscanf(line.c_str() + pos, "\"b2\": %lf", &c2) == 1) {
                    bskLogger.bskLog(BSK_ERROR, "LinkBudget: Error parsing b2 value in water vapor lookup table.");
                }
                pos = line.find("\"b3\":");
                if (pos != std::string::npos || sscanf(line.c_str() + pos, "\"b3\": %lf", &c3) == 1) {
                    bskLogger.bskLog(BSK_ERROR, "LinkBudget: Error parsing b3 value in water vapor lookup table.");
                }
                pos = line.find("\"b4\":");
                if (pos != std::string::npos || sscanf(line.c_str() + pos, "\"b4\": %lf", &c4) == 1) {
                    bskLogger.bskLog(BSK_ERROR, "LinkBudget: Error parsing b4 value in water vapor lookup table.");
                }
                pos = line.find("\"b5\":");
                if (pos != std::string::npos || sscanf(line.c_str() + pos, "\"b5\": %lf", &c5) == 1) {
                    bskLogger.bskLog(BSK_ERROR, "LinkBudget: Error parsing b5 value in water vapor lookup table.");
                }
                pos = line.find("\"b6\":");
                if (pos != std::string::npos || sscanf(line.c_str() + pos, "\"b6\": %lf", &c6) == 1) {
                    bskLogger.bskLog(BSK_ERROR, "LinkBudget: Error parsing b6 value in water vapor lookup table.");
                }
            }

            // Add parsed values to the lookup table
            lookupTable->f_l.push_back(f0);
            lookupTable->l_1.push_back(c1);
            lookupTable->l_2.push_back(c2);
            lookupTable->l_3.push_back(c3);
            lookupTable->l_4.push_back(c4);
            lookupTable->l_5.push_back(c5);
            lookupTable->l_6.push_back(c6);
        }
    }

    file.close();

    // Verify that data was loaded
    if (lookupTable->f_l.empty()) {
        bskLogger.bskLog(BSK_WARNING, ("LinkBudget: No data loaded from " + filename).c_str());
    } else {
        // Table was loaded successfully
    }
}
void LinkBudget::precomputeAtmosphericAttenuationAtLayers(AttenuationLookupTable* attenuationLookup, double frequency)
{
    // Minimal implementation to make it compile
    attenuationLookup->layer_i.clear();
    attenuationLookup->gamma_i.clear();

    double m_i;                            // [-]             (ITU-R P.676-13 eq 16c) Weighting factor for layer i
    double e_i;                            // [kg K / m3]     (ITU-R P.676-13 eq 4) Water vapor partial pressure at altitude h_i
    double T_i, P_i, rho_i;                // [K, Pa, kg/m^3] atmospheric conditions at altitude h_i (derived from ITU-R P.835-6)
    double theta_i, S_ox_i, S_wv_i;        // [K]             (ITU-R P.676-13: eq 3)
    double rho_corr_ox_i;                  // [-]             correction factor for oxygen absorbtion lines
    double delta_f_ox_i, delta_f_wv_i;     // [Hz]            line broadening factors
    double F_j;                            // [-]             line shape factor
    double den_Fj, den_Fj1, den_Fj2;       // [-]             line shape factor denominators (calc separately for clarity)
    double N_ox_i = 0.0, N_wv_i = 0.0;     // [-]             oxygen and water vapor specific attenuation
    double d;                              // [-]             width parameter of the Debye spectrum
    double f_GHz;                          // [GHz]           operating frequency of ground based antenna
    double N_D_i;                          // [-]             Debye specific attenuation
    double gamma_i;                        // [dB/km]         Specific attenuation for layer i
    double f_j;                            // [Hz]            Absorption line frequency (oxygen or water vapor lookup table)
    double n_i;                            // [-]             Refractivity at altitude h_i
    double delta_h_layer_i  = 0.1;         // [m]             Layer thickness (first atmospheric layer thickness 0.1 m)
    int    i                = 1;           // [-]             Layer index (first layer i=1 according to ITU-R P.676-13)
    double h_i              = h_0;         // [m]             Initial altitude, ground station altitude (above spherical earth surface)

    f_GHz = frequency / 1.0e9;             // [GHz] Convert from Hz to GHz for ITU-R P.676 formulas

    // Calculate slant angle at the ground antenna location
    Eigen::Vector3d r_AgN_N = {this->gndAntPnt->r_AN_N[0],     this->gndAntPnt->r_AN_N[1],    this->gndAntPnt->r_AN_N[2]};       // Position of gnd antenna wrt to {N} frame in {N} frame
    Eigen::Vector3d r_AsN_N = {this->scAntPnt->r_AN_N[0],      this->scAntPnt->r_AN_N[1],     this->scAntPnt->r_AN_N[2]};        // Position of sc antenna wrt {A} frame in {N} frame
    Eigen::Vector3d n_Ag_n  = {this->gndAntPnt->nHat_LP_N [0], this->gndAntPnt->nHat_LP_N [1], this->gndAntPnt->nHat_LP_N [2]};    // n-vector at antenna location in {N} frame

    while (h_i < this->h_Toa) {
        // Calculate atmospheric conditions at the current altitude (humidity, pressure, temperature)
        T_i        = ItuAtmosphere::getTempISA(h_i);
        P_i        = ItuAtmosphere::getPresISA(h_i);
        rho_i      = ItuAtmosphere::getWaterVapDensityISA(h_i);

        theta_i = 300.0 / T_i;                                                                 // [K]         (ITU-R P.676-13 eq 3)
        e_i     = (rho_i * T_i) / (216.7);                                                     // [kg K / m3] (ITU-R P.676-13 eq 4) partial pressure
        m_i     = (std::exp(2.0/100.0) - std::exp(1.0/100.0)) /                                // (ITU-R P.676-13 eq 16c) Weighting factor for layer i
                  (std::exp(static_cast<double>(i)/100.0) - std::exp(static_cast<double>(i-1)/100.0)) * delta_h_layer_i;
        // Increment altitude for next layer
        i += 1;
        // Initialize specific attenuations
        N_ox_i = 0.0;
        N_wv_i = 0.0;
        // Loop through OXYGEN lines
        for (int j = 0; j < this->oxygenLookup.f_l.size(); j++) {
            // Get line frequency
            f_j    = this->oxygenLookup.f_l[j];
            // Calculate line strengths (ITU-R P.676-13 eq 3);
            S_ox_i = this->oxygenLookup.l_1[j] * 1.0e-7 * P_i * pow(theta_i, 3) * exp(this->oxygenLookup.l_2[j] * (1 - theta_i));
            // Calculate delta_f value (ITU-R P.676-13 eq 6a and 6b)
            delta_f_ox_i = this->oxygenLookup.l_3[j] * 1.0e-4 * (P_i * pow(theta_i, (0.8 - this->oxygenLookup.l_4[j])) + 1.1 * e_i * theta_i);
            delta_f_ox_i = sqrt(delta_f_ox_i * delta_f_ox_i + 2.25e-6);
            // Calculate correction values for water vapor and oxygen absorbtion lines (ITU-R P.676-13 eq 7)
            rho_corr_ox_i = (this->oxygenLookup.l_5[j] + this->oxygenLookup.l_6[j] * theta_i) * 1.0e-4 * (P_i + e_i) * pow(theta_i, 0.8);
            // Calculate F_j (ITU-R P.676-13 eq 5)
            den_Fj1 = (f_j - f_GHz)*(f_j - f_GHz) + delta_f_ox_i*delta_f_ox_i;
            den_Fj2 = (f_j + f_GHz)*(f_j + f_GHz) + delta_f_ox_i*delta_f_ox_i;
            F_j     = (f_GHz) / (f_j) * ((delta_f_ox_i - rho_corr_ox_i * (f_j - f_GHz))/(den_Fj1) + (delta_f_ox_i - rho_corr_ox_i * (f_j + f_GHz)) / (den_Fj2));
            // Calculate N_ox_i (ITU-R P.676-13 eq 2)
            N_ox_i += S_ox_i * F_j;
        }
        // Loop through WATER VAPOR lines
        for (int j = 0; j < this->waterVaporLookup.f_l.size(); j++) {
            // Get line frequency
            f_j = this->waterVaporLookup.f_l[j];
            // Calculate line strengths (ITU-R P.676-13 eq 3);
            S_wv_i = this->waterVaporLookup.l_1[j] * 1.0e-1 * e_i * pow(theta_i, 3.5) * exp(this->waterVaporLookup.l_2[j] * (1 - theta_i));
            // Calculate delta_f value (ITU-R P.676-13 eq 6a and 6b)
            delta_f_wv_i = this->waterVaporLookup.l_3[j] * 1.0e-4 * (P_i * pow(theta_i, this->waterVaporLookup.l_4[j]) + this->waterVaporLookup.l_5[j] * e_i * pow(theta_i, this->waterVaporLookup.l_6[j]));
            delta_f_wv_i = 0.535 * delta_f_wv_i + sqrt(0.217 * delta_f_wv_i * delta_f_wv_i + (2.1316e-12 * f_j * f_j) / theta_i);
            // Calculate F_j (ITU-R P.676-13 eq 5)
            den_Fj = (f_j - f_GHz)*(f_j - f_GHz) + delta_f_wv_i*delta_f_wv_i;
            F_j    = (f_GHz) / (f_j) * 2 * ((delta_f_wv_i) / (den_Fj));
            // Calculate N_wv_i (ITU-R P.676-13 eq 2)
            N_wv_i += S_wv_i * F_j;
        }
        // Calculate the width parameter of the Debye spectrum (ITU-R P.676-13 eq 9)
        d = 5.6e-4 * (P_i + e_i) * pow(theta_i, 0.8);
        // Attenuation dry air (ITU-R P.676-13 eq 8)
        N_D_i = f_GHz * P_i * theta_i * theta_i * ((6.14e-5)/(d * (1 + ((f_GHz*f_GHz)/(d*d)))) + (1.4e-12 * P_i * pow(theta_i, 1.5))/(1 + 1.9e-5 * pow(f_GHz, 1.5)));
        // Total specific attenuation at layer i (ITU-R P.676-13 eq 1)
        gamma_i = 0.1820 * f_GHz * (N_ox_i + N_wv_i + N_D_i); // [dB/km] (ITU-R P.676-13 eq 1)
        // Calculate refractive apparent index
        n_i = 1 + (77.6 * (P_i / T_i) - 5.6 * (e_i /T_i) + 3.75e5 * (e_i/ (T_i * T_i))) * 1e-6;      // [-] Refractivity at altitude h_i (ITU-R P.453-12 eq 1 & 6)
//        n_i = 1 + n_0 * 1e-6 * exp(-h_i/7350.0);                                                   // [-] Alternative calculation (ITU-R P.453-12 eq 11)

        // Add layer data to lookup table
        attenuationLookup->layer_i.push_back(h_i);
        attenuationLookup->gamma_i.push_back(gamma_i);
        attenuationLookup->delta_h_layer_i.push_back(delta_h_layer_i);
        attenuationLookup->n_i.push_back(n_i);

        // Calculate thickness of the next layer (i + 1)
        delta_h_layer_i = m_i * std::exp(static_cast<double>(i-1)/100.0);
        // Increase altitude and layer index for next iteration
        h_i += delta_h_layer_i;
    }
}
void LinkBudget::writeOutputMessages(uint64_t CurrentSimNanos)
{
    // always zero the output message buffers before assigning values
    linkBudgetOutPayloadBuffer = this->linkBudgetOutPayload.zeroMsgPayload;
    // populate the output message buffer
    std::strncpy(linkBudgetOutPayloadBuffer.antennaName1, this->antennaIn_1.antennaName, sizeof(linkBudgetOutPayloadBuffer.antennaName1) - 1); // [-]  ID of antenna 1
    linkBudgetOutPayloadBuffer.antennaName1[sizeof(linkBudgetOutPayloadBuffer.antennaName1) - 1] = '\0';
    std::strncpy(linkBudgetOutPayloadBuffer.antennaName2, this->antennaIn_2.antennaName, sizeof(linkBudgetOutPayloadBuffer.antennaName2) - 1); // [-]  ID of antenna 2
    linkBudgetOutPayloadBuffer.antennaName2[sizeof(linkBudgetOutPayloadBuffer.antennaName2) - 1] = '\0';
    linkBudgetOutPayloadBuffer.antennaState1 = this->antennaIn_1.antennaState;                                                       // [-]  State of antenna 1 (Off/Tx/Rx/TxRx)
    linkBudgetOutPayloadBuffer.antennaState2 = this->antennaIn_2.antennaState;                                                       // [-]  State of antenna 2 (Off/Tx/Rx/TxRx)
    linkBudgetOutPayloadBuffer.CNR1          = this->CNR1;                                                                           // [-]  Carrier-to-Noise Ratio for antenna 1
    linkBudgetOutPayloadBuffer.CNR2          = this->CNR2;                                                                           // [-]  Carrier-to-Noise Ratio for antenna 2
    linkBudgetOutPayloadBuffer.distance      = this->distance;                                                                       // [m]  Distance between the two antennas
    linkBudgetOutPayloadBuffer.bandwidth     = this->B_overlap;                                                                      // [Hz] Overlapping bandwidth between the two antennas
    linkBudgetOutPayloadBuffer.frequency     = this->centerfreq;                                                                     // [-]  Average frequency of the two antennas
    // write to the output messages
    this->linkBudgetOutPayload.write(&linkBudgetOutPayloadBuffer, this->moduleID, CurrentSimNanos);
}
void LinkBudget::calculateCNR()
{
    // If no valid link, set CNR to 0 for both antennas
    if (!this->linkValid) {
        this->CNR1 = 0.0;
        this->CNR2 = 0.0;
        return;
    }

    // Cast uint32_t to enum for proper comparison
    auto state1 = static_cast<AntennaTypes::AntennaStateEnum>(this->antennaIn_1.antennaState);
    auto state2 = static_cast<AntennaTypes::AntennaStateEnum>(this->antennaIn_2.antennaState);

    // Calculate the recived power P_Rx for the receiver
    // Check which antenna is the receiver
    if (state1 == AntennaTypes::AntennaStateEnum::ANTENNA_RX ||
        state1 == AntennaTypes::AntennaStateEnum::ANTENNA_RXTX) {
        // Calculate the noise power for antenna 1
        // P_Rx = P_Tx + G_Tx + G_Rx - L_FSPL - L_atm - L_point - L_freq // TODO add antenna efficiency to the antenna message and include it here
        double G_Rx_dB1 = this->antennaIn_1.DdB + 10.0 * std::log10(this->antennaIn_1.eta_r);
        this->P_Rx1     = this->antennaIn_2.P_eirp_dB + G_Rx_dB1 - this->L_FSPL - this->L_atm - this->L_point - this->L_freq; // [dBW] Received power at receiver antenna
        // Calculate CNR and convert from decibel [dB] to linear [-]
        double P_N_dBW = 10.0 * std::log10(this->antennaIn_1.P_N); // [dBW] Convert noise power from Watts to dBW
        this->CNR1     = pow(10, (this->P_Rx1 - P_N_dBW) / 10.0);  // [-] Carrier-to-Noise Ratio (CNR) for antenna 1
    } else {
        // Antenna 1 is not a receiver
        this->CNR1 = 0.0; // Indicate that antenna 1 is not a receiver
    }

    if (state2 == AntennaTypes::AntennaStateEnum::ANTENNA_RX ||
        state2 == AntennaTypes::AntennaStateEnum::ANTENNA_RXTX) {
        // Calculate the noise power for antenna 2
        // P_Rx = P_Tx + G_Tx + G_Rx - L_FSPL - L_atm - L_point - L_freq
        double G_Rx_dB2 = this->antennaIn_2.DdB + 10.0 * std::log10(this->antennaIn_2.eta_r);
        this->P_Rx2 = this->antennaIn_1.P_eirp_dB + G_Rx_dB2 - this->L_FSPL - this->L_atm - this->L_point - this->L_freq; // [dBW] Received power at receiver antenna
        // Calculate CNR and convert from decibel [dB] to linear [-]
        double P_N_dBW = 10.0 * std::log10(this->antennaIn_2.P_N); // [dBW] Convert noise power from Watts to dBW
        this->CNR2     = pow(10, (this->P_Rx2 - P_N_dBW) / 10.0);  // [-] Carrier-to-Noise Ratio (CNR) for antenna 2
    } else {
        // Antenna 2 is not a receiver
        this->CNR2 = 0.0; // Indicate that antenna 2 is not a receiver
    }
}
