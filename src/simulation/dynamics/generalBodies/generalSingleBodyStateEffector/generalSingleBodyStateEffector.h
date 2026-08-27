/*
 ISC License

 Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#ifndef GENERAL_SINGLE_BODY_STATE_EFFECTOR_H
#define GENERAL_SINGLE_BODY_STATE_EFFECTOR_H

#include "simulation/dynamics/_GeneralModuleFiles/stateEffector.h"
#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "simulation/dynamics/_GeneralModuleFiles/stateData.h"
#include "architecture/utilities/avsEigenSupport.h"
#include "architecture/utilities/avsEigenMRP.h"

/*! @brief General rigid body state effector class */
class GeneralSingleBodyStateEffector: public StateEffector, public SysModel {
public:
    GeneralSingleBodyStateEffector();
    ~GeneralSingleBodyStateEffector() = default;

    void Reset(uint64_t currentSimNanos) override;
    void registerStates(DynParamManager& statesIn) override;
    void linkInStates(DynParamManager& states) override;
    void UpdateState(uint64_t currentSimNanos) override;
    void computeGeneralBodyInertialStates();
    void writeOutputStateMessages(uint64_t currentSimNanos) override;
    void updateEffectorMassProps(double integTime) override;
    void updateContributions(double integTime,
                             BackSubMatrices & backSubContr,
                             Eigen::MRPd sigma_BN,
                             Eigen::Vector3d omega_BN_B,
                             Eigen::Vector3d g_N) override;
    void computeDerivatives(double integTime,
                            Eigen::Vector3d rDDot_BN_N,
                            Eigen::Vector3d omegaDot_BN_B,
                            Eigen::MRPd sigma_BN) override;
    void updateEnergyMomContributions(double integTime,
                                      Eigen::Vector3d & rotAngMomPntCContr_B,
                                      double & rotEnergyContr,
                                      Eigen::Vector3d omega_BN_B) override;

private:
    static uint64_t effectorID;
};

#endif /* GENERAL_SINGLE_BODY_STATE_EFFECTOR_H */
