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

*/

#ifndef LINKBUDGET_H
#define LINKBUDGET_H

#include <fstream>
#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/msgPayloadDefC/AntennaLogMsgPayload.h"
#include "architecture/msgPayloadDefC/SpicePlanetStateMsgPayload.h"
#include "architecture/msgPayloadDefC/LinkBudgetMsgPayload.h"
#include "architecture/utilities/bskLogging.h"
#include "architecture/messaging/messaging.h"
#include "architecture/utilities/astroConstants.h"
#include "../_GeneralModuleFiles/LinkBudgetTypes.h"
#include "../_GeneralModuleFiles/AntennaDefinitions.h"
#include "architecture/utilities/avsEigenSupport.h"

/*! @brief This module describes how the radio link-budget model operates within the simulation environment.
 *  The purpose of this module is to provide a basic representation of radio-link communication between two spacecrafts
 *  or between a spacecraft and a ground station.
 */
struct LookupTable {
    std::vector<double> f_l;          //!< [GHz] Frequency lookup values
    std::vector<double> l_1;          //!< [-] Lookup coefficient a1, b1
    std::vector<double> l_2;          //!< [-] Lookup coefficient a2, b2
    std::vector<double> l_3;          //!< [-] Lookup coefficient a3, b3
    std::vector<double> l_4;          //!< [-] Lookup coefficient a4, b4
    std::vector<double> l_5;          //!< [-] Lookup coefficient a5, b5
    std::vector<double> l_6;          //!< [-] Lookup coefficient a6, b6
};

/*! @brief Precomputed atmospheric attenuation profile
 *  Stores layer-by-layer attenuation data for slant path integration
 */
struct AttenuationLookupTable {
    std::vector<double> layer_i;         //!< [m] Altitude layers
    std::vector<double> gamma_i;         //!< [dB/km] Attenuation values at each altitude layer (vertical)
    std::vector<double> delta_h_layer_i; //!< [m] Thickness of each altitude layer
    std::vector<double> n_i;             //!< [-] Refractivity at each altitude layer
};

/*! @brief Radio link budget model for spacecraft-to-spacecraft or spacecraft-to-ground communication
 *
 *  This module computes the end-to-end link budget between two antennas, accounting for:
 *  - Free space path loss (FSPL)
 *  - Atmospheric attenuation (ITU-R P.676)
 *  - Antenna pointing losses
 *  - Frequency offset losses
 *  - Carrier-to-noise ratio (CNR)
 */
class LinkBudget:  public SysModel {
public:
    LinkBudget();
    ~LinkBudget() = default;

    void Reset(uint64_t CurrentSimNanos);
    void UpdateState(uint64_t CurrentSimNanos);

public:
    ReadFunctor<AntennaLogMsgPayload>       antennaInPayload_1;     //!< antenna output antenna 1
    ReadFunctor<AntennaLogMsgPayload>       antennaInPayload_2;     //!< antenna output antenna 2

    // local copies of message buffers
    AntennaLogMsgPayload       antennaIn_1;                         //!< local copy of message buffer
    AntennaLogMsgPayload       antennaIn_2;                         //!< local copy of message buffer

    // Output Messages
    Message<LinkBudgetMsgPayload> linkBudgetOutPayload;             //!< output msg description

    // Variables to store (output) message data
    LinkBudgetMsgPayload  linkBudgetOutPayloadBuffer;               //!< local copy of message buffer

    BSKLogger bskLogger;                                            //!< BSK Logging

    /** GETTERS **/
    /*! @brief Get 'free space path loss'
     *  @return FSPL [dB]
     */
    double getL_FSPL() const {return this->L_FSPL;}

    /*! @brief Get atmospheric attenuation loss
     *  @return L_atm [dB]
     */
    double getL_atm() const {return this->L_atm;}

    /*! @brief Get frequency offset loss
     *  @return L_freq [dB]
     */
    double getL_freq() const {return this->L_freq;}

    /*! @brief Get pointing loss
     *  @return L_point [dB]
     */
    double getL_point() const {return this->L_point;}

    /*! @brief Get carrier to noise ratio of antenna1
     *  @return CNR1 [-]
     */
    double getCNR1() const {return this->CNR1;}

    /*! @brief Get carrier to noise ratio of antenna2
     *  @return CNR2 [-]
     */
    double getCNR2() const {return this->CNR2;}

private:
    LinkBudgetTypes::AntennaPlacement antennaPlacement;  //!< [-]   Antenna placement type
    AntennaTypes::EnvironmentType     env1;              //!< [-]   Antenna environment of antenna 1: space, 2: ground
    AntennaTypes::EnvironmentType     env2;              //!< [-]   Antenna environment of antenna 2: space, 2: ground
    double                            distance;          //!< [m]   Distance between antennas
    double                            L_FSPL;            //!< [dB]  Free space path loss
    double                            L_atm;             //!< [dB]  Atmospheric loss
    double                            L_freq;            //!< [dB]  Frequency offset loss
    double                            L_point;           //!< [dB]  Pointing loss
    double                            P_Rx1;             //!< [W]   Antenna receive power
    double                            P_Rx2;             //!< [W]   Antenna receive power
    double                            CNR1;              //!< [-]   Carrier to noise ratio
    double                            CNR2;              //!< [-]   Carrier to noise ratio
    double                            B_overlap;         //!< [Hz]  Overlapping bandwidth
    double                            centerfreq;        //!< [Hz]  Center frequency of the two antennas

    /* Atmospheric coefficients */
    const double          h_Toa      = 100.0e3;          //!< [m]   Height of top of atmosphere for attenuation integration(TOA = 100 km "Karman line") (above spherical earth surface)
    AntennaLogMsgPayload  *gndAntPnt = nullptr;          //!< [-]   Pointer to the ground antenna msg payload
    AntennaLogMsgPayload  *scAntPnt  = nullptr;          //!< [-]   Pointer to the spacecraft antenna msg payload
    bool                   atmosAtt;                     //!< [-]   Enable/disable atmospheric attenuation
    bool                   pointingLoss;                 //!< [-]   Enable pointing loss by default
    bool                   freqLoss;                     //!< [-]   Enable frequency offset loss by default
    LookupTable            oxygenLookup;                 //!< [-]   Oxygen absorption coefficients       # TODO this should be a lookup table with frq a1 a2 a3 a4 a5 a6
    LookupTable            waterVaporLookup;             //!< [-]   Water vapor absorption coefficients  # TODO this should be a lookup table with frq b1 b2 b3 b4 b5 b6
    AttenuationLookupTable attenuationLookup;            //!< [-]   Atmospheric attenuation lookup table
    double                 h_0;                          //!< [m]   Altitude of ground antenna
    double                 n_0;                          //!< [-]   Refractivity at ground antenna altitude
    bool                   linkValid;                    //!< [-]   Flag indicating if the link is valid (sufficient bandwidth overlap, antennas in correct states, etc.)

    /* Constants, */
    /* Clamp to minimum 5° elevation angle for numerical stability */
    const double MIN_SIN_ELEVATION = 0.0872;  // sin(5°) ≈ 0.0872

private:
    void readMessages();
    void initialization();
    void calculateLinkBudget();
    void calculateFSPL();
    void calculateAtmosphericLoss();
    double calcCosSlantGround(Eigen::Vector3d r_AN_N, Eigen::Vector3d n_A_N, Eigen::Vector3d r_BN_N);
    void generateLookupTable(LinkBudgetTypes::GasType gasType, LookupTable* lookupTable);
    void precomputeAtmosphericAttenuationAtLayers(AttenuationLookupTable* tableAttenuation, double frequency);
    Eigen::Vector2d getPointingError(Eigen::Vector3d r_A1N_N, Eigen::Vector3d r_A2N_N, Eigen::MRPd sigma_NA1);
    void calculatePointingLoss();
    void calculateFrequencyOffsetLoss();
    void calculateCNR();
    void writeOutputMessages(uint64_t CurrentSimNanos);
};
#endif
