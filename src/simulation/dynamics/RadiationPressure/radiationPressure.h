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
#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "simulation/dynamics/_GeneralModuleFiles/dynamicEffector.h"
#include "simulation/dynamics/_GeneralModuleFiles/stateData.h"
#include "simulation/dynamics/_GeneralModuleFiles/dynParamManager.h"

#include "architecture/msgPayloadDefC/SCStatesMsgPayload.h"
#include "architecture/msgPayloadDefC/SpicePlanetStateMsgPayload.h"
#include "architecture/msgPayloadDefC/EclipseMsgPayload.h"
#include "architecture/messaging/messaging.h"

#include "architecture/utilities/bskLogging.h"




typedef enum {
    SRP_CANNONBALL_MODEL,
    SRP_FACETED_CPU_MODEL
} srpModel_t;



//  SRP effects on body
/*! @brief solar radiation pressure dynamic effector */
class RadiationPressure: public SysModel, public DynamicEffector{
public:
    RadiationPressure();
    ~RadiationPressure();

    void Reset(uint64_t CurrentSimNanos);
    void UpdateState(uint64_t CurrentSimNanos);
    void linkInStates(DynParamManager& statesIn);
    void readInputMessages();
    void computeForceTorque(double integTime, double timeStep);
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
    double  coefficientReflection;                  //!< -- Factor grouping surface optical properties
    ReadFunctor<SpicePlanetStateMsgPayload> sunEphmInMsg;   //!< -- sun state input message
    ReadFunctor<EclipseMsgPayload> sunEclipseInMsg;         //!< -- (optional) sun eclipse input message
    std::vector<Eigen::Vector3d> lookupForce_B;     //!< -- Force on S/C at 1 AU from sun
    std::vector<Eigen::Vector3d> lookupTorque_B;    //!< -- Torque on S/C
    std::vector<Eigen::Vector3d> lookupSHat_B;      //!< -- S/C to sun unit vector defined in the body frame.
    BSKLogger bskLogger;                      //!< -- BSK Logging

private:
    srpModel_t  srpModel; //!< -- specifies which SRP model to use
    SpicePlanetStateMsgPayload sunEphmInBuffer;    //!< -- Buffer for incoming ephemeris message data
    bool stateRead; //!< -- Indicates a succesful read of incoming SC state message data
    EclipseMsgPayload sunVisibilityFactor;          //!< [-] scaling parameter from 0 (fully obscured) to 1 (fully visible)
    StateData *hubR_N;                          //!< -- State data accesss to inertial position for the hub
    StateData *hubSigma;                                   //!< -- Hub/Inertial attitude represented by MRP

};


#endif
