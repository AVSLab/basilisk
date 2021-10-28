/*
 ISC License

 Copyright (c) 2020, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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


#ifndef GRAVITY_GRADIENT_DYNAMIC_EFFECTOR_H
#define GRAVITY_GRADIENT_DYNAMIC_EFFECTOR_H

#include <Eigen/Dense>
#include <vector>
#include "simulation/dynamics/_GeneralModuleFiles/dynamicEffector.h"
#include "simulation/dynamics/_GeneralModuleFiles/stateData.h"
#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/utilities/avsEigenMRP.h"
#include "architecture/utilities/avsEigenSupport.h"
#include "architecture/utilities/bskLogging.h"

#include "architecture/msgPayloadDefC/GravityGradientMsgPayload.h"
#include "architecture/messaging/messaging.h"



/*! @brief gravity gradient gradient module */
class GravityGradientEffector: public SysModel, public DynamicEffector {
public:
    GravityGradientEffector();
    ~GravityGradientEffector();
    void linkInStates(DynParamManager& states);
    void computeForceTorque(double integTime, double timeStep);
    void Reset(uint64_t CurrentSimNanos);
    void UpdateState(uint64_t CurrentSimNanos);
    void WriteOutputMessages(uint64_t CurrentClock);
    void addPlanetName(std::string planetName);


public:
    Message<GravityGradientMsgPayload> gravityGradientOutMsg; //!< output message containing the gravity gradient 
    StateData *hubSigma;                            //!< Hub/Inertial attitude represented by MRP
    StateData *r_BN_N;                              //!< Hub/Inertial position vector in inertial frame components
    Eigen::MatrixXd *ISCPntB_B;                     //!< [kg m^2] current spacecraft inertia about point B, B-frame components
    Eigen::MatrixXd *c_B;                           //!< [m] Vector from point B to CoM of s/c in B frame components
    Eigen::MatrixXd *m_SC;                          //!< [kg] mass of spacecraft
    std::vector<Eigen::MatrixXd *> r_PN_N;          //!< [m] vector of inertial planet positions
    std::vector<Eigen::MatrixXd *> muPlanet;        //!< [m^3/s^-2] gravitational constant of planet

    BSKLogger bskLogger;                            //!< BSK Logging

private:
    std::vector<std::string> planetPropertyNames;   //!< Names of planets we want to track

};


#endif /* GRAVITY_GRADIENT_DYNAMIC_EFFECTOR_H */
