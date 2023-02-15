/*
 ISC License

 Copyright (c) 2023, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#ifndef PRESCRIBED_MOTION_STATE_EFFECTOR_H
#define PRESCRIBED_MOTION_STATE_EFFECTOR_H

#include "simulation/dynamics/_GeneralModuleFiles/stateEffector.h"
#include "architecture/_GeneralModuleFiles/sys_model.h"

/*! @brief prescribed motion state effector class */
class PrescribedMotionStateEffector: public StateEffector, public SysModel {
public:

private:

public:
    PrescribedMotionStateEffector();                        //!< Constructor
    ~PrescribedMotionStateEffector();                       //!< Destructor
    void Reset(uint64_t CurrentClock);                      //!< Method for reset
    void writeOutputStateMessages(uint64_t CurrentClock);   //!< Method for writing the output messages
	void UpdateState(uint64_t CurrentSimNanos);             //!< Method for updating the effector states
    void registerStates(DynParamManager& statesIn);         //!< Method for registering the effector's states
    void linkInStates(DynParamManager& states);             //!< Method for giving the effector access to hub states
    void updateContributions(double integTime, BackSubMatrices & backSubContr, Eigen::Vector3d sigma_BN, Eigen::Vector3d omega_BN_B, Eigen::Vector3d g_N);  //!< Method for computing the effector's back-substitution contributions
    void computeDerivatives(double integTime, Eigen::Vector3d rDDot_BN_N, Eigen::Vector3d omegaDot_BN_B, Eigen::Vector3d sigma_BN);                         //!< Method for effector to compute its state derivatives
    void updateEffectorMassProps(double integTime);         //!< Method for calculating the effector mass props and prop rates
    void updateEnergyMomContributions(double integTime, Eigen::Vector3d & rotAngMomPntCContr_B, double & rotEnergyContr, Eigen::Vector3d omega_BN_B);       //!< Method for computing the energy and momentum of the effector
    void computePrescribedMotionInertialStates();           //!< Method for computing the effector's states relative to the inertial frame
};

#endif /* PRESCRIBED_MOTION_STATE_EFFECTOR_H */
