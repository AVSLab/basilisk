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

#ifndef RADIATION_PRESSURE_H
#define RADIATION_PRESSURE_H

#include <vector>
#include "_GeneralModuleFiles/sys_model.h"
#include "../_GeneralModuleFiles/dynamicEffector.h"
#include "../_GeneralModuleFiles/stateData.h"
#include "../_GeneralModuleFiles/dynParamManager.h"
#include "simMessages/spicePlanetStateSimMsg.h"
#include "simMessages/scPlusStatesSimMsg.h"
#include "simMessages/eclipseSimMsg.h"

/*! \addtogroup SimModelGroup
 * @{
 */


typedef enum {
    SRP_CANNONBALL_MODEL,
    SRP_FACETED_CPU_MODEL
} srpModel_t;


//! @brief Radiation pressure dynamics class used to compute
/*!
 The module
 [PDF Description](Basilisk-RadiationPressure-20170712.pdf)
 contains further information on this module's function,
 how to run it, as well as testing.
*/
//  SRP effects on body
class RadiationPressure: public SysModel, public DynamicEffector{
public:
    RadiationPressure();
    ~RadiationPressure();

    void SelfInit();
    void CrossInit();
    void UpdateState(uint64_t CurrentSimNanos);
    void linkInStates(DynParamManager& statesIn);
    void readInputMessages();
    void computeForceTorque(double integTime);
    void setUseCannonballModel();
    void setUseFacetedCPUModel();
    void addForceLookupBEntry(Eigen::Vector3d vec);
    void addTorqueLookupBEntry(Eigen::Vector3d vec);
    void addSHatLookupBEntry(Eigen::Vector3d vec);
    
private:
    void computeCannonballModel(Eigen::Vector3d rSunB_B);
    void computeLookupModel(Eigen::Vector3d rSunB_B);

public:
    double  area; //!< m^2 Body surface area
    double  coefficientReflection; //!< -- Factor grouping surface optical properties
    std::string sunEphmInMsgName; //!< -- Message name for the sun state
    std::string stateInMsgName; //!< -- Message name for the S/C state
    std::string sunEclipseInMsgName;            //!< [-] Message name for sun eclipse state message
    std::vector<Eigen::Vector3d> lookupForce_B;     //!< -- Force on S/C at 1 AU from sun
    std::vector<Eigen::Vector3d> lookupTorque_B;    //!< -- Torque on S/C
    std::vector<Eigen::Vector3d> lookupSHat_B;      //!< -- S/C to sun unit vector defined in the body frame.

private:
    srpModel_t  srpModel; //!< -- specifies which SRP model to use
    int64_t sunEphmInMsgId; //!< -- Message ID for incoming sun ephemeris data
    int64_t sunEclipseInMsgId;                  //!< [-] Connect to input sun eclipse message
    SpicePlanetStateSimMsg sunEphmInBuffer; //!< -- Buffer for incoming ephemeris message data
    int64_t stateInMsgId; //!< -- Message ID for incoming SC state data
    bool stateRead; //!< -- Indicates a succesful read of incoming SC state message data
    SCPlusStatesSimMsg stateInBuffer; //!< -- Buffer for incoming state message data
    EclipseSimMsg sunVisibilityFactor;              //!< [-] scaling parameter from 0 (fully obscured) to 1 (fully visible)
};

/*! @} */

#endif
