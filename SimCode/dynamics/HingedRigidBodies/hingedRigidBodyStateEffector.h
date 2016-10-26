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


#ifndef HINGED_RIGID_BODY_STATE_EFFECTOR_H
#define HINGED_RIGID_BODY_STATE_EFFECTOR_H

#include "../_GeneralModuleFiles/stateEffector.h"
#include "../_GeneralModuleFiles/stateData.h"
#include "../_GeneralModuleFiles/sys_model.h"
#include <Eigen/Dense>
/*! @brief Abstract class that is used to implement an effector impacting a dynamic body 
           that does not itself maintain a state or represent a changing component of
           the body (for example: gravity, thrusters, solar radiation pressure, etc.)
 */

class HingedRigidBodyStateEffector : public StateEffector, public SysModel {
public:
    HingedRigidBodyStateEffector();
    ~HingedRigidBodyStateEffector();
    void registerStates(DynParamManager& statesIn);
    void linkInStates(DynParamManager& states);
    void updateContributions(double integTime, Eigen::Matrix3d & matrixAcontr, Eigen::Matrix3d & matrixBcontr, Eigen::Matrix3d & matrixCcontr, Eigen::Matrix3d & matrixDcontr, Eigen::Vector3d & vecTranscontr, Eigen::Vector3d & vecRotcontr);
    void computeDerivatives(double integTime);
    void updateEffectorMassProps(double integTime);
    void updateEffectorMassPropRates(double integTime);

public:

private:
    
};

#endif /* STATE_EFFECTOR_H */
