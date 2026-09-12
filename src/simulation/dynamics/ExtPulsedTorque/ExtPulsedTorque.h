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

#ifndef EXT_PULSED_TORQUE_H
#define EXT_PULSED_TORQUE_H

#include <cstdint>
#include <Eigen/Dense>
#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "simulation/dynamics/_GeneralModuleFiles/dynParamManager.h"
#include "simulation/dynamics/_GeneralModuleFiles/dynamicEffector.h"
#include "architecture/utilities/bskLogging.h"


/*! @brief external pulsed torque module class */
class ExtPulsedTorque final: public SysModel, public DynamicEffector{
public:
    ExtPulsedTorque();
    ~ExtPulsedTorque() override;

    void Reset(uint64_t CurrentSimNanos) override;
    void UpdateState(uint64_t CurrentSimNanos) override;
    void linkInStates(DynParamManager& statesIn) override;
    void writeOutputMessages(uint64_t currentClock);
    void readInputMessages();
    void computeForceTorque(double integTime, double timeStep) override;

private:
    void validateConfiguration(); //!< Validate the torque, pulse counts, and interval duration

public:
    Eigen::Vector3d pulsedTorqueExternalPntB_B = Eigen::Vector3d::Zero(); //!< [N*m] pulsed torque about B in B components
    int countOnPulse = 0; //!< Number of pulse intervals for each positive and negative pulse; zero disables the torque
    int countOff = 0; //!< Number of pulse intervals with no torque between pulse pairs
    double pulseInterval = 1.0; //!< [s] Duration of one pulse-count interval, independent of the integrator step
    BSKLogger bskLogger;                      //!< BSK Logging

};


#endif
