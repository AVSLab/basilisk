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

#include <cstring>
#include <cmath>
#include "simpleAntenna.h"
#include "architecture/utilities/brightnessTemperatureSolarSystem.hpp"

/*! This is the constructor for the module class.  It sets default variable
    values and initializes the various parts of the model */
SimpleAntenna::SimpleAntenna()
{
    // initialize module variables
    // Antenna configuration and state
    this->antennaName  = "";                                                    // --        Antenna name
    this->environment  = AntennaTypes::EnvironmentType::_ENVIRONMENT_UNKNOWN;   // [-]       Antenna environment of antenna 0: SPACE, 1: GROUND
    this->antennaState = AntennaTypes::AntennaStateEnum::ANTENNA_OFF;           // [-]       Antenna state (OFF, RX, TX, RXTX, UNKNOWN) (default OFF)
    // Antenna Properties (set by user)
    this->frequency    = -1;                                                    // [Hz]      Antenna operating frequency (default 2.2 GHz -> S-band)
    this->B            = -1;                                                    // [Hz]      Antenna frequency bandwidth (default 5 MHz)
    this->DdB          = -1;                                                    // [dB]      Antenna directivity (default 20 dBi --> directivity of 100 in linear scale)
    this->k            = -1;                                                    // [-]       Beamwidth asymmetry ratio "k = HPBW_az / HPBW_el" (default 1 --> symmetric beam)
    this->P_Tx         = -1;                                                    // [W]       Transmit power
    this->P_Rx         = -1;                                                    // [W]       Receive power
    this->T_E          = -1;                                                    // [K]       Equivalent noise temperature
    this->eta_r        = -1;                                                    // [-]       Antenna radiation efficiency
    this->HPBW         = {-1,-1};                                               // [rad]     Half-power beamwidth in azimuth and elevation (this is initialized in initializeAntenna())
    // Antenna performance metrics (Calculated)
    this->P_N          = {};                                                    // [W]       Noise power of the antenna
    this->P_eirp_dB    = {};                                                    // [dB]      Equivalent isotropically radiated power (EIRP) (determined at initialization)
    this->G_TN         = {};                                                    // [dB/K]    Antenna gain over system noise temperature (G/T_N)
    // Antenna enviroment properties
    this->T_AmbientSet = -1;                                                    // [K]       User defined Ambient temperature
    this->T_Ambient    = -1;                                                    // [K]       Ambient temperature (calculated if not provided by user)
    // Antenna position and orientation
    this->r_AN_N       = Eigen::Vector3d::Zero();                               // [m]       Antenna position in N frame
    this->sigma_AN     = Eigen::MRPd::Identity();                               // [-]       Antenna orientation in N frame
    this->v_AN_N         = Eigen::Vector3d::Zero();                             // [m/s]     Antenna velocity in N frame (stub, future use)
    // SPACE
    this->r_AB_B       = Eigen::Vector3d::Zero();                               // [m]       Antenna position relative to body frame    (default aligned with body)
    this->sigma_AB     = Eigen::MRPd::Identity();                               // [-]       Antenna orientation relative to body frame (default aligned with body)
    // GROUND
    this->r_AP_N       = Eigen::Vector3d::Zero();                               // [m]       Position of the antenna wrt to the celestial body frame {P} (used for ground antennas only)
    // Antenna Noise
    this->useHaslamMap = false;                                                 // [bool]    Haslam sky map data structure (initialized on first use)
    this->sunData      = {};                                                    // [-]       Sun data structure
    this->rEqCustom    = -1.0;                                                  // [m]       Custom planet equatorial radius (if applicable)
}
/*! This method is used to reset the module and checks that required input messages are connected.
*/
void SimpleAntenna::Reset(uint64_t CurrentSimNanos)
{
    // ERRORS
    // check that required input messages are connected
    if (!this->scStateInMsg.isLinked() && !this->groundStateInMsg.isLinked()) {
        // Antenna is NOT connected to ground or spacecraft (needs to be connected to one)
        bskLogger.bskLog(BSK_ERROR, "Neither SimpleAntenna.scStateInMsg nor SimpleAntenna.groundState was linked.");
    }
    if (this->scStateInMsg.isLinked() && this->groundStateInMsg.isLinked()) {
        // Antenna is connected to BOTH ground and spacecraft (needs to be connected to only one)
        bskLogger.bskLog(BSK_ERROR, "Both SimpleAntenna.scStateInMsg and SimpleAntenna.groundState were linked. Please link only one to define the environment.");
    }
    if (this->antennaName.empty()) {
        // Antenna name is empty
        bskLogger.bskLog(BSK_ERROR, "SimpleAntenna antennaName has not been set!");
    }
    if (this->frequency == -1) {
        // Antenna operating frequency is invalid
        bskLogger.bskLog(BSK_ERROR, "SimpleAntenna operating frequency [Hz] has not been set!");
    }
    if (this->B == -1) {
        // Antenna bandwidth is invalid
        bskLogger.bskLog(BSK_ERROR, "SimpleAntenna bandwidth B [Hz] has not been set!");
    }
    if (this->DdB == -1) {
        // Antenna directivity is invalid
        bskLogger.bskLog(BSK_ERROR, "SimpleAntenna directivity D [dB] has not been set!");
    }
    if (this->k == -1) {
        // Antenna HPBW ratio is invalid
        bskLogger.bskLog(BSK_ERROR, "SimpleAntenna HPBW ratio k has not been set!");
    }
    if (this->P_Tx == -1) {
        // Antenna transmit power is invalid
        bskLogger.bskLog(BSK_ERROR, "SimpleAntenna transmit power P_Tx [W] has not been set!");
    }
    if (this->P_Rx == -1) {
        // Antenna receive power is invalid
        bskLogger.bskLog(BSK_WARNING, "SimpleAntenna receive power P_Rx [W] has not been set! --> Setting to P_Rx = 0.0 W.");
        this->P_Rx = 0.0; // [W] set default receive power
    }
    if (this->T_E == -1) {
        // Antenna equivalent noise temperature is invalid
        bskLogger.bskLog(BSK_ERROR, "SimpleAntenna equivalent noise temperature T_E [K] has not been set!");
    }
    if (this->eta_r == -1) {
        // Antenna radiation efficiency is invalid
        bskLogger.bskLog(BSK_ERROR, "SimpleAntenna radiation efficiency eta_r has not been set!");
    }
    if (!this->sunInMsg.isLinked() && this->planetInMsgs.size() != 0) {
        // Sun data message is NOT linked while planet messages are linked (sun data is required to calculate sky noise)
        bskLogger.bskLog(BSK_ERROR, "SimpleAntenna.sunInMsg was not linked while planetInMsgs is not empty. Sun data is required to calculate sky noise.");
    }


    // WARNINGS
    if (!this->antennaSetStateInMsg.isLinked()) {
        // Antenna state message is NOT linked -> Antenna remains OFF and cannot be switched on! (warning only)
        bskLogger.bskLog(BSK_INFORMATION, "SimpleAntenna.antennaSetStateInMsg was not linked (Antenna is by default OFF). --> Antenna state can only be changed through setter.");
    }
    if (!this->sunInMsg.isLinked()) {
        // Sun data message is NOT linked -> space background temperature is uniform (warning only)
        bskLogger.bskLog(BSK_INFORMATION, "SimpleAntenna.sunInMsg was not linked --> Space background temperature is set to uniform temperature.");
    }
    if (this->planetInMsgs.size() == 0) {
        bskLogger.bskLog(BSK_INFORMATION, "SimpleAntenna.planetInMsgs is empty --> Space background temperature is set based on just space and sun.");
    }
    if (!this->sunEclipseInMsg.isLinked()) {
        // Sun eclipse message is NOT linked -> ground antenna cannot determine eclipse state (warning only)
        bskLogger.bskLog(BSK_INFORMATION, "SimpleAntenna.sunEclipseInMsg was not linked --> Ground antenna cannot determine eclipse state.");
    }

    // initialize antenna environment
    this->initializeAntenna();
}

void SimpleAntenna::initializeAntenna()
{
    // Determine environment (SPACE or EARTH)  -->  Check if groundMsg or Spacecraft is linked
    if (this->scStateInMsg.isLinked()) {
        this->environment = AntennaTypes::EnvironmentType::ENVIRONMENT_SPACE; // SPACE
    } else if (this->groundStateInMsg.isLinked()) {
        this->environment = AntennaTypes::EnvironmentType::ENVIRONMENT_EARTH; // EARTH
    } else {
        // Error condition (unreachable due to checks in Reset())
        bskLogger.bskLog(BSK_ERROR, "SimpleAntenna: Unable to determine environment. Please link either GroundState or scState message, not both.");
    }

    // Calculate HPBW_el && HPBW_az based on k and G
    double dir_lin  = pow(10, this->DdB / 10);                                          // [-]   Linear scale directivity
    double HPBW_el  = sqrt((log(2) * 16) / (dir_lin * this->k));                        // [rad] HPBW in elevation (derived for a gaussian beam pattern) (with default values HPBW_el = 0.333 rad = 19.1 deg)
    double HPBW_az  = this->k * HPBW_el;                                                // [rad] HPBW in azimuth (with default values HPBW_el = 0.333 rad = 19.1 deg)
    this->HPBW      = {HPBW_az, HPBW_el};                                               // [rad] store HPBW in azimuth and elevation

    // Calculate antenna EIRP
    this->P_eirp_dB = 10 * log10(this->P_Tx) + 10 * log10(this->eta_r) + this->DdB;     // [dB] (with default values: P_eirp_dB = 20 dBW + (-3 dB) + 20 dB = 37 dBW)

    if (this->sunInMsg.isLinked() && this->environment == AntennaTypes::EnvironmentType::ENVIRONMENT_SPACE && this->useHaslamMap) {
    // Add a check to ensure the file path is set and not empty
        if (!HaslamMap::getInstance().isInitialized()) {
            try {
                HaslamMap::getInstance().initialize();
            } catch (...) {
                bskLogger.bskLog(BSK_ERROR, "HaslamMap failed to initialize. Check FITS file path.");
            }
        }
    }
}

/*! This is the main method that gets called every time the module is updated.  Provide an appropriate description.
*/
void SimpleAntenna::UpdateState(uint64_t CurrentSimNanos)
{
    this->readInputMessages();                       // Read in the input messages
    this->calculateAntennaPositionAndOrientation();  // Calculate antenna position and attitude in {N} frame
    this->calculateAntennaNoise();                   // Calculate antenna noise temperature
    this->writeOutputMessages(CurrentSimNanos);      // Write the output messages
}

void SimpleAntenna::readInputMessages()
{
    if (this->environment == AntennaTypes::EnvironmentType::ENVIRONMENT_SPACE) {
        // SPACE environment
        this->scState = this->scStateInMsg();
    } else if (this->environment == AntennaTypes::EnvironmentType::ENVIRONMENT_EARTH) {
        // EARTH environment
        this->groundState = this->groundStateInMsg();
    }
    if (this->antennaSetStateInMsg.isLinked()) {
        this->antennaStateMsg = this->antennaSetStateInMsg();
        this->antennaState    = static_cast<AntennaTypes::AntennaStateEnum>(this->antennaStateMsg.antennaState);
    }
    if(this->sunInMsg.isLinked())
    {
        this->sunData = this->sunInMsg();
    }
    if (!this->planetInMsgs.empty()) {
        for (long unsigned int c = 0; c<this->planetInMsgs.size(); c++){
            this->planetBuffer[c] = this->planetInMsgs[c]();
        }
    }
    if(this->sunEclipseInMsg.isLinked()) {
        EclipseMsgPayload sunVisibilityFactor;          // sun visiblity input message
        sunVisibilityFactor = this->sunEclipseInMsg();
        this->illuminationFactor = sunVisibilityFactor.illuminationFactor;
    }
}

void SimpleAntenna::writeOutputMessages(uint64_t CurrentSimNanos)
{
    // Zero the output message buffer
    this->antennaPhysicalStateMsgBuffer = this->antennaOutMsg.zeroMsgPayload;
    // Populate and write the output message
    this->populateOutputMsg(&this->antennaPhysicalStateMsgBuffer);
    // Write the output message
    this->antennaOutMsg.write(&this->antennaPhysicalStateMsgBuffer, this->moduleID, CurrentSimNanos); // write antenna state message
}

/* Calculate sky temperature based on environmental parameters
*/
double SimpleAntenna::calculateTsky()
{
    /* NOTE: This is a simplified model for sky brightness temperature calculation
             Neglected are: - Noise due to lightning and other static electrical discharges
                            - Man-made noise radio noise
                            - Solar activity variations
    */
    double T_sky = 0.0;
    double T_sun = 0.0;                                     //!< [K] Brightness temperature of sun initialized to zero
    double T_earth = 0.0;                                   //!< [K] Brightness temperature of earth initialized to zero
    double T_moon = 0.0;                                    //!< [K] Brightness temperature of moon initialized to zero
    double T_galaxy = 0.0;                                  //!< [K] Brightness temperature of galaxy initialized to zero
    Eigen::Matrix3d dcm_BN;                                 //!< [-] DCM from inertial {N} to body {B}
    Eigen::Vector3d r_SN_N;                                 //!< [m] Sun position relative to inertial
    Eigen::Vector3d r_SB_N;                                 //!< [m] Sun position relative to S/C in {N} frame
    Eigen::Vector3d sHat_N;                                 //!< [-] Unit sun heading vector in {N} frame
    Eigen::Vector3d sHat_A;                                 //!< [-] Unit sun heading vector in {A} frame
    Eigen::Vector3d r_PN_N;                                 //!< [m] Planet position relative to inertial in frame N
    Eigen::Vector3d r_PB_N;                                 //!< [m] Planet position relative to S/C in frame N
    Eigen::Vector3d pHat_N;                                 //!< [-] Unit planet heading vector in inertial frame N
    Eigen::Vector3d pHat_A;                                 //!< [-] Unit planet heading vector in antenna frame A
    Eigen::MRPd     sigma_BN;                               //!< [-] MRP attitude of body relative to inertial
    Eigen::Vector3d sHat_B;                                 //!< [-] Unit Sun heading vector relative to the spacecraft in B frame.
    Eigen::Matrix3d dcm_BA;                                 //!< [-] DCM from antenna A to body B
    double galaxyCoverage = 1.0;                                  //!< [-] Ratio of galaxy coverage in the antenna field of view
    std::vector<SpicePlanetStateMsgPayload>::iterator planetIt;
    if (this->environment == AntennaTypes::EnvironmentType::ENVIRONMENT_SPACE) {
        // SPACE based antenna
        if (this->sunInMsg.isLinked()) {
            // Check if the Sun is in the field of view
            r_SN_N = cArray2EigenVector3d(this->sunData.PositionVector);
            double sunCoverage = this->calculatePlanetCoverage(REQ_SUN * 1000, r_SN_N, this->HPBW[0]/2 , this->HPBW[1]/2);

            if (!this->planetInMsgs.empty()){
                for(planetIt = this->planetBuffer.begin(); planetIt != this->planetBuffer.end(); planetIt++)
                {
                    r_PN_N = cArray2EigenVector3d(planetIt->PositionVector);
                    std::string plName = planetIt->PlanetName;
                    double planetRadius   = this->getPlanetEquatorialRadius(plName);
                    double planetCoverage = this->calculatePlanetCoverage(planetRadius, r_PN_N, this->HPBW[0]/2 , this->HPBW[1]/2);

                    if (plName == "earth") {
                        T_earth = getBrightnessTemperatureFromData("Earth", this->frequency);   // [K] Get earth brightness temperature from data file
                        T_earth *= planetCoverage;                                             // [K] Scale by earth coverage factor
                        galaxyCoverage = galaxyCoverage - planetCoverage;                                 // [-] Scale galaxy coverage factor
                    }else if (plName == "moon") {
                        T_moon  = getBrightnessTemperatureFromData("Moon",  this->frequency);   // [K] Get moon brightness temperature from data file
                        T_moon *= planetCoverage;                                              // [K] Scale by moon coverage factor
                        galaxyCoverage = galaxyCoverage - planetCoverage;                                 // [-] Scale galaxy coverage factor
                    }
                }
                if(this->sunEclipseInMsg.isLinked()) {
                    if (this->illuminationFactor == 0) {
                    sunCoverage = 0.0;
                    }
                }

            }
            T_sun   = getBrightnessTemperatureFromData("Sun",   this->frequency);   // [K] Get sun brightness temperature from data file
            T_sun *= sunCoverage;                                                   // [K] Scale by sun coverage factor
            galaxyCoverage = galaxyCoverage - sunCoverage;                                     // [-] Scale galaxy coverage factor

            if (this->useHaslamMap) {
                // Get sky brightness temperature from Haslam map at 408 MHz and scale to frequency of interest
                Eigen::Vector3d n_A_N = this->sigma_AN.toRotationMatrix() * Eigen::Vector3d::UnitZ(); // Antenna boresight in inertial frame
                double samplingAngle  = 0.5 * std::sqrt(this->HPBW[0] * this->HPBW[1]);  // [rad] geometric mean half-angle
                double T_b_408MHz     = HaslamMap::getInstance().getBrightnessTemperature(n_A_N, samplingAngle);
                T_galaxy              = HaslamMap::getInstance().scaleToFrequency(T_b_408MHz, this->frequency) + CMB_TEMPERATURE; // [K] Scale brightness temperature to frequency of interest and add CMB
            } else {
                T_galaxy = getBrightnessTemperatureFromData("Galaxy", this->frequency) + CMB_TEMPERATURE; // [K] Get galaxy brightness temperature from data file
            }
        } else { // Spice message is NOT linked
            // Sky brightness temperature uniform (Ground antenna pointing direction is determined in "link-budget" module)
//            T_sky = T_galaxy;  // Cosmic Microwave Background temperature + 100K for other background sources (earth / sun / moon / distance celestial bodies)
        }
        T_sky = T_earth + T_moon + T_sun + T_galaxy*galaxyCoverage;

    } else if (this->environment == AntennaTypes::EnvironmentType::ENVIRONMENT_EARTH) {
        // Ground based antenna
        // POTENTIAL IMPROVEMENT: Introduce antenna pointing direction for ground based antennas, and vary T_sky based on boresight elevation angle
        // T_sky = T_CMB + T_atm (1 - exp(-tau_0 / sin(elevation))) tau_0 = zenith optical depth (depends on frequency and atmospheric conditions)
        T_sky = 200.0; // [K] Typical sky temperature for ground based antennas
    }
    return T_sky;
}

/**
 * @brief Compute fraction of antenna beam covered by object
 * @param planetRadius
 * @param r_PN_N
 * @param azimuth Beam half-angle in azimuth [rad]
 * @param elevation Beam half-angle in elevation [rad]
 * @return coverage fraction [0..1]
 */
double SimpleAntenna::calculatePlanetCoverage(double planetRadius, Eigen::Vector3d r_PN_N, double azimuth, double elevation)
{
    Eigen::Vector3d r_PB_N;         //!< [m] Planet position relative to spacecraft body B
    Eigen::Matrix3d dcm_BN;         //!< [] DCM from inertial N to body B
    Eigen::Matrix3d dcm_BA;         //!< [] DCM from inertial N to body B
    Eigen::Vector3d pHat_N;         //!< [] unit planet heading vector in inertial frame N
    Eigen::Vector3d pHat_A;         //!< [] unit planet heading vector in antenna frame A
    Eigen::MRPd sigma_BN;           //!< [] MRP attitude of body relative to inertial

    //! Read Message data to eigen
    sigma_BN  = cArray2EigenMRPd(this->scState.sigma_BN);

    //! Find planet heading unit vector
    r_PB_N          = r_PN_N - this->r_BN_N;
    double r_PBNorm = r_PB_N.norm();
    pHat_N   = r_PB_N.normalized();

    // Transform into antenna frame
    Eigen::Matrix3d dcm_AN = this->sigma_AN.toRotationMatrix().transpose();
    pHat_A                 = dcm_AN * pHat_N;

    // Planet azimuth/elevation in antenna frame
    double az = atan2(pHat_A.y(), pHat_A.z());
    double el = atan2(pHat_A.x(), pHat_A.z());

    // Angular radius of object
    double alphaP = safeAsin(planetRadius / r_PBNorm);

    // Normalize into a unit-circle beam model
    double y = az / azimuth;
    double x = el / elevation;
    double d = safeSqrt(x*x + y*y);        // normalized distance to beam center
    double r = alphaP / safeSqrt(azimuth * elevation);  // normalized planet radius
    const double R = 1.0;  // beam unit radius

    // No overlap case
    if (d >= R + r) {
        return 0.0;
    }

    // Object completely inside beam case
    if (d <= fabs(R - r)) {
        double A_p = r * r;
        double A_beam = R * R;
        return fmin(A_p / A_beam, 1.0);
    }

    // Partial overlap (circle-intersection area)
    double R2 = R * R;
    double r2 = r * r;

    double alpha = safeAcos((d*d + R2 - r2) / (2*d*R));
    double beta  = safeAcos((d*d + r2 - R2) / (2*d*r));
    double A_overlap =
        R2 * alpha +
        r2 * beta -
        0.5 * safeSqrt(
            (-d + R + r) *
            (d + R - r) *
            (d - R + r) *
            (d + R + r)
        );

    double A_beam = M_PI * R2;

    return std::clamp(A_overlap / A_beam, 0.0, 1.0);
}

/*! This method adds planet state data message names to a vector.
 @param tmpSpMsg The planet name

 */
void SimpleAntenna::addPlanetToModel(Message<SpicePlanetStateMsgPayload> *tmpSpMsg)
{
    this->planetInMsgs.push_back(tmpSpMsg->addSubscriber());

    SpicePlanetStateMsgPayload tmpMsg;
    this->planetBuffer.push_back(tmpMsg);
    return;
}

/*! This method return planet radii.
 @param  planetSpiceName The planet name according to the spice NAIF Integer ID codes.
 @return double  The equatorial radius in metres associated with the given planet name.
 */
double SimpleAntenna::getPlanetEquatorialRadius(std::string planetSpiceName)
{
    if (planetSpiceName == "mercury") {
        return REQ_MERCURY*1000.0; // [meters]
    } else if (planetSpiceName == "venus") {
        return REQ_VENUS*1000.0;
    } else if (planetSpiceName == "earth") {
        return REQ_EARTH*1000.0;
    } else if (planetSpiceName == "moon") {
        return REQ_MOON*1000.0;
    } else if (planetSpiceName == "mars barycenter") {
        return REQ_MARS*1000.0;
    } else if (planetSpiceName == "mars") {
        return REQ_MARS*1000.0;
    } else if (planetSpiceName == "jupiter barycenter") {
        return REQ_JUPITER*1000.0;
    } else if (planetSpiceName == "saturn") {
        return REQ_SATURN*1000.0;
    } else if (planetSpiceName == "uranus") {
        return REQ_URANUS*1000.0;
    } else if (planetSpiceName == "neptune") {
        return REQ_NEPTUNE*1000.0;
    } else if (planetSpiceName == "custom") {
        if (rEqCustom <= 0.0) {
            bskLogger.bskLog(BSK_ERROR, "SimpleAntenna: Invalid rEqCustom set.");
            return 1.0;
        } else {
            return rEqCustom;
        }
    } else {
        bskLogger.bskLog(BSK_ERROR, "SimpleAntenna: unrecognized planetSpiceName.");
        return 1.0;
    }
}

double SimpleAntenna::calculateTambient()
{
    double T_ambientLocal; // [K]
    double T_groundLocal;  // [K]
    double dTdh_local;     // [K/m]
    if (this->environment == AntennaTypes::EnvironmentType::ENVIRONMENT_SPACE) {
        // SPACE environment
        T_ambientLocal = 150.0; // [K] Assumed average ambient temperature
    } else if (this->environment == AntennaTypes::EnvironmentType::ENVIRONMENT_EARTH) {
        // EARTH environment
        T_groundLocal = 288.15; // [K]   Standard ground temperature at sea level (15 C ITU-R P.835-6)
        dTdh_local    = 0.0065; // [K/m] Standard temperature lapse rate in the Troposphere (Earth) (ITU-R P.835-6)
        double altitude = sqrt(this->groundState.r_LP_N[0] * this->groundState.r_LP_N[0] +
                               this->groundState.r_LP_N[1] * this->groundState.r_LP_N[1] +
                               this->groundState.r_LP_N[2] * this->groundState.r_LP_N[2]) - REQ_EARTH*1000.0; // [m] Altitude above spherical Earth surface
        T_ambientLocal       = T_groundLocal - dTdh_local * altitude;                                  // [K] ITU-R P.835-6 eq. 1 (assumption altitude of ground station always < 11 km)
    } else {
        // UNKNOWN environment (FUTURE EXPANSION)
        bskLogger.bskLog(BSK_ERROR, "SimpleAntenna: Unable to calculate ambient temperature due to unknown environment.");
        T_ambientLocal = 150.0; // [K] Default value
    }
    return T_ambientLocal;
}

void SimpleAntenna::populateOutputMsg(AntennaLogMsgPayload *output)
{
    std::snprintf(output->antennaName, sizeof(output->antennaName), "%s", this->antennaName.c_str());
    output->antennaName[sizeof(output->antennaName)-1] = '\0';
    output->environment    = static_cast<uint32_t>(this->environment);
    output->antennaState   = static_cast<uint32_t>(this->antennaState);
    output->frequency      = this->frequency;
    output->B              = this->B;
    output->HPBW_az        = this->HPBW[0];
    output->HPBW_el        = this->HPBW[1];
    output->P_Tx           = this->P_Tx;
    output->P_Rx           = this->P_Rx;
    output->DdB            = this->DdB;
    output->G_TN           = this->G_TN;
    // above is just mapping
    output->P_N            = this->P_N;
    output->P_eirp_dB      = this->P_eirp_dB;
    output->T_Ambient      = this->T_Ambient;
    output->r_AN_N[0]      = this->r_AN_N[0];
    output->r_AN_N[1]      = this->r_AN_N[1];
    output->r_AN_N[2]      = this->r_AN_N[2];
    output->sigma_AN[0]    = this->sigma_AN.x();
    output->sigma_AN[1]    = this->sigma_AN.y();
    output->sigma_AN[2]    = this->sigma_AN.z();
    output->v_AN_N[0]        = this->v_AN_N[0];
    output->v_AN_N[1]        = this->v_AN_N[1];
    output->v_AN_N[2]        = this->v_AN_N[2];
    if (this->environment == AntennaTypes::EnvironmentType::ENVIRONMENT_SPACE) {
        // SPACE
        output->r_AP_N[0]    = 0.0;
        output->r_AP_N[1]    = 0.0;
        output->r_AP_N[2]    = 0.0;
    } else if (this->environment == AntennaTypes::EnvironmentType::ENVIRONMENT_EARTH) {
        // EARTH
        output->r_AP_N[0]    = this->r_AP_N[0];
        output->r_AP_N[1]    = this->r_AP_N[1];
        output->r_AP_N[2]    = this->r_AP_N[2];
    } else {
        // Raise error (should not happen due to checks in Reset)
        bskLogger.bskLog(BSK_ERROR, "SimpleAntenna: Unable to populate output message due to unknown environment.");
    }
}

void SimpleAntenna::calculateAntennaPositionAndOrientation()
{
    Eigen::Matrix3d dcm_NA; // DCM from antenna frame {A} to inertial frame {N}
    if (this->environment == AntennaTypes::EnvironmentType::ENVIRONMENT_SPACE) {
        // SPACE environment
        // Calculate the antenna position in N frame
        Eigen::MRPd     sigma_BN  = cArray2EigenMRPd(this->scState.sigma_BN);                 // Convert c-array to Eigen MRP
        Eigen::Matrix3d dcm_NB    = sigma_BN.toRotationMatrix();                              // Calculate DCM_NB from MRP_BN
        this->r_BN_N              = cArray2EigenVector3d(this->scState.r_BN_N);               // Convert c-array to Eigen Vector3d
        this->r_AN_N              = this->r_BN_N + dcm_NB * this->r_AB_B;                     // Antenna position in N-frame, decomposed in N-frame:   r_AN_N = r_AB_N + r_BN_N

        // Calculate the antenna orientation in {N} frame
        dcm_NA                    = dcm_NB * this->sigma_AB.toRotationMatrix();               // DCM from {A} frame to {N} frame:   dcm_NA = dcm_NB * dcm_BA
        this->sigma_AN            = eigenMRPd2Vector3d(eigenC2MRP(dcm_NA.transpose()));       // Convert DCM to MRP

        // Calculate the antenna velocity in the {N} frame
        this->v_AN_N = cArray2EigenVector3d(this->scState.v_BN_N);                            // Antenna velocity in {N} frame (assumed same as spacecraft velocity)
    } else if (this->environment == AntennaTypes::EnvironmentType::ENVIRONMENT_EARTH) {
        // GROUND environment
        // Antenna position and orientation are defined in the groundState message
        this->r_AN_N   = cArray2EigenVector3d(this->groundState.r_LN_N);                      // Antenna position in {N} frame (same as ground location)
        this->v_AN_N     = Eigen::Vector3d::Zero();                                           // FUTURE EXPANSION: Antenna velocity in {N} frame (currently assumed zero)

        this->r_AP_N   = cArray2EigenVector3d(this->groundState.r_LP_N);                      // Antenna position in {P} frame
    }
}

void SimpleAntenna::calculateAntennaNoise()
{
    // Calculate antenna noise temperature based on T_sky and T_Ambient
    double T_sky;
    T_sky = SimpleAntenna::calculateTsky();
    // If the user has not set T_Ambient, calculate it based on environment
    if (this->T_AmbientSet == -1) {
        // T_Ambient not set by user -> calculate T_Ambient based on environment
        this->T_Ambient = SimpleAntenna::calculateTambient(); // Set after first iteration if not set by the user
    } else {
        this->T_Ambient = this->T_AmbientSet; // Use user defined T_Ambient
    }
    double T_ant = (1 - this->eta_r) * this->T_Ambient + this->eta_r * T_sky;  // [K]    Antenna noise temperature
    double T_S   = this->T_E + T_ant;                                    // [K]    System noise temperature
    this->P_N    = K_BOLTZMANN * T_S * this->B;                          // [W]    Noise power

    // Calculate antenna G/T_N
    this->G_TN = this->DdB +  10 * log10(this->eta_r) - 10 * log10(T_S); // [dB/K] Antenna gain over system noise temperature
}

/*! Configure the path to the Haslam 408 MHz all-sky brightness temperature FITS file.
This must be called before setUseHaslamMap(true).
@param file Full path to the Haslam FITS file
*/
void SimpleAntenna::configureBrightnessFile(const std::string& file) {
    HaslamMap::getInstance().configureBrightnessFile(file);
}

/*******************/
/***** Setters *****/
/*******************/
void SimpleAntenna::setAntennaName(const std::string& var)
{
    /*Make sure antennaName is < 20 characters */
    if (var.length() >= 20) {
        bskLogger.bskLog(BSK_ERROR, "SimpleAntenna: Antenna name length exceeds 20 characters.");
    }
    this->antennaName = var;
}

void SimpleAntenna::setAntennaState(AntennaTypes::AntennaStateEnum var) {
    if (var < AntennaTypes::AntennaStateEnum::ANTENNA_OFF || var > AntennaTypes::AntennaStateEnum::ANTENNA_RXTX) {
        bskLogger.bskLog(BSK_WARNING, "SimpleAntenna: Attempting to set invalid antenna state. Value must be between 0 (Off), 1 (Rx), 2 (Tx), 3 (RxTx). Setting to OFF (0).");
        this->antennaState = AntennaTypes::AntennaStateEnum::ANTENNA_OFF;
        return;
    }
    this->antennaState = var;    // [-] antenna state: 0: OFF, 1: Rx, 2: Tx, 3: RxTx
}
void SimpleAntenna::setAntennaFrequency(double var) {
    if (var <= 3e7 || var >= 6e10) {        // Center frequency shall be between 30 Hz and 60 GHz
        bskLogger.bskLog(BSK_ERROR, "SimpleAntenna operating frequency [Hz] is set outside the valid range of 30 MHz to 60 GHz.");
    }
    this->frequency = var;       // [Hz] antenna operating frequency
}
void SimpleAntenna::setAntennaBandwidth(double var) {
    if (var <= minTreshold) {                // Bandwidth cannot be zero or negative
        bskLogger.bskLog(BSK_ERROR, "SimpleAntenna bandwidth B [Hz] is too small, must be larger than minimum threshold (10^-6 Hz).");
    }
    this->B = var;               // [Hz] antenna frequency bandwidth
}
void SimpleAntenna::setAntennaDirectivity_dB(double var) {
    if (var <= 9) {
        bskLogger.bskLog(BSK_ERROR, "SimpleAntenna: Attempting to set invalid low antenna directivity D[dB]. Value must be > 9dB for the assumptions made in this model.");
    }
    this->DdB = var;             // [dB] antenna directivity
}
void SimpleAntenna::setAntennaHpbwRatio(double var) {
    if (var <= 0.5 || var >= 5.0) {
        bskLogger.bskLog(BSK_ERROR, "SimpleAntenna: Attempting to set invalid antenna HPBW ratio. Value must be between 0.5 and 5 (set to k=1.0 for a symmetric beam).");
    }
    this->k = var;
}
void SimpleAntenna::setAntennaP_Tx(double var) {
    if (var <= this->minTreshold) {
        bskLogger.bskLog(BSK_ERROR, "SimpleAntenna transmit power P_Tx [W] is set to a value below the minimum threshold (10^-6 W) => Antenna transmit power must be larger than minimal threshold.");
        return;
    }
    this->P_Tx = var;            // [W] transmit power
}
void SimpleAntenna::setAntennaP_Rx(double var) {
    if (var < 0) {
        bskLogger.bskLog(BSK_WARNING, "SimpleAntenna: Attempting to set to a negative receive power P_Rx [W]. Value must be non-negative");
        return;
    }
    this->P_Rx = var;            // [W] receive power
}
void SimpleAntenna::setAntennaEquivalentNoiseTemp(double var) {
    if (var < this->minTreshold) {
        bskLogger.bskLog(BSK_ERROR, "SimpleAntenna equivalent noise temperature T_E [K] is set to a negative value => Antenna equivalent noise temperature must be non-negative.");
        return;
    }
    this->T_E = var;             // [K] equivalent noise temperature
}
void SimpleAntenna::setAntennaRadEfficiency(double var) {
    if (var < 0 || var > 1) {
        bskLogger.bskLog(BSK_ERROR, "SimpleAntenna: Attempting to set invalid antenna radiation efficiency eta_r. Value must be between 0 and 1");
    }
    this->eta_r = var;           // [-] antenna radiation efficiency
}
void SimpleAntenna::setAntennaEnvironmentTemperature(double var) {
    if (var < this->minTreshold) {
        bskLogger.bskLog(BSK_ERROR, "SimpleAntenna: Attempting to set an unrealistic environment temperature T_Ambient [K]. Value must be > 1e-6 K.");
    }
    this->T_AmbientSet = var;
}
void SimpleAntenna::setAntennaPositionBodyFrame(Eigen::Vector3d var) {
    this->r_AB_B = var;
}
void SimpleAntenna::setAntennaOrientationBodyFrame(Eigen::MRPd var) {
    this->sigma_AB = var;
}

void SimpleAntenna::setUseHaslamMap(bool var) {
    this->useHaslamMap = var;
}
