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

#ifndef DYNAMIC_EFFECTOR_H
#define DYNAMIC_EFFECTOR_H

#include <Eigen/Dense>
#include "dynParamManager.h"

/*! @brief Abstract class that is used to implement an effector impacting a dynamic body that does not itself maintain a
    state or represent a changing component of the body (for example: gravity, thrusters, SRP, etc.) */
class DynamicEffector {
public:
    DynamicEffector();                      //!< -- Constructor
    virtual ~DynamicEffector();             //!< -- Destructor
    virtual void computeStateContribution(double integTime); 
    virtual void linkInStates(DynParamManager& states) = 0;  //!< -- Method to get access to other states/stateEffectors
    virtual void computeForceTorque(double integTime) = 0;  //!< -- Method to computeForce and torque on the body
    
public:
    Eigen::VectorXd stateDerivContribution; //!< -- DynamicEffectors contribution to a stateEffector
    Eigen::Vector3d forceExternal_N;        //!< [N] External force applied by this effector in inertial components
    Eigen::Vector3d forceExternal_B;        //!< [N] External force applied by this effector in body frame components
    Eigen::Vector3d torqueExternalPntB_B;   //!< [Nm] External torque applied by this effector
};

#endif /* DYNAMIC_EFFECTOR_H */
