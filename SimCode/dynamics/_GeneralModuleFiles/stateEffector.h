/*
 Copyright (c) 2016, Autonomous Vehicle Systems Lab, Univeristy of Colorado at Boulder
 
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


#ifndef STATE_EFFECTOR_H
#define STATE_EFFECTOR_H

#include "dynParamManager.h"
#include <Eigen/Dense>
/*! @brief Abstract class that is used to implement an effector impacting a dynamic body 
           that does not itself maintain a state or represent a changing component of
           the body (for example: gravity, thrusters, solar radiation pressure, etc.)
 */

typedef struct {
    Eigen::Matrix3d IEffPntB_B;           //! [kgm2] Inertia of effector relative to effector CoM in B
    Eigen::Vector3d rCB_B;                 //! [m] Center of component with respect to attachment in B
    Eigen::Vector3d rPrimeCB_B;
    Eigen::Matrix3d IEffPrimePntB_B;
    double mEff;                           //! [kg] Mass of the effector
}EffectorMassProps;

class StateEffector {
public:
    EffectorMassProps effProps;
    
public:
    StateEffector();
    virtual ~StateEffector();
    virtual void registerStates(DynParamManager& states) = 0;
    virtual void linkInStates(DynParamManager& states) = 0;
//    virtual void updateBackSubstitution(double integTime)=0;
    virtual void updateContributions(double integTime, Eigen::Matrix3d & matrixAcontr, Eigen::Matrix3d & matrixBcontr, Eigen::Matrix3d & matrixCcontr, Eigen::Matrix3d & matrixDcontr, Eigen::Vector3d & vecTranscontr, Eigen::Vector3d & vecRotcontr);
    virtual void computeDerivatives(double integTime)=0;
    virtual void updateEffectorMassProps(double integTime);
    virtual void updateEffectorMassPropRates(double integTime);
};

#endif /* STATE_EFFECTOR_H */
