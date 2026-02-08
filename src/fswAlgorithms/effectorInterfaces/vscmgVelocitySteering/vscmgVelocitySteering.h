/*
 ISC License

 Copyright (c) 2025, Autonomous Vehicle Systems Lab, University of Colorado Boulder

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


#ifndef VSCMG_VELOCITY_STEERING_H
#define VSCMG_VELOCITY_STEERING_H

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/msgPayloadDefC/VSCMGArrayConfigMsgPayload.h"
#include "architecture/msgPayloadDefC/CmdTorqueBodyMsgPayload.h"
#include "architecture/msgPayloadDefC/NavAttMsgPayload.h"
#include "architecture/msgPayloadDefC/AttGuidMsgPayload.h"
#include "architecture/msgPayloadDefC/VSCMGSpeedMsgPayload.h"
#include "cMsgCInterface/VSCMGRefStatesMsg_C.h"
#include "architecture/utilities/bskLogging.h"
#include "architecture/messaging/messaging.h"
#include <Eigen/Dense>
#include "architecture/utilities/avsEigenSupport.h"

/*! @brief Mapping desired control torque vector to gimbal rates and RW wheel accelerations
 */
class VscmgVelocitySteering: public SysModel {
public:
    VscmgVelocitySteering();
    ~VscmgVelocitySteering() = default;
    void SelfInit();
    void Reset(uint64_t CurrentSimNanos);
    void UpdateState(uint64_t CurrentSimNanos);

public:
    ReadFunctor<VSCMGArrayConfigMsgPayload> vscmgParamsInMsg;  //!< [-] VSCMG array configuration input message
    ReadFunctor<CmdTorqueBodyMsgPayload> vehControlInMsg;  //!< [-] vehicle control (Lr) input message
    ReadFunctor<NavAttMsgPayload> attNavInMsg;  //!< [-] attitude navigation input message
    ReadFunctor<AttGuidMsgPayload> attGuideInMsg;  //!< [-] attitude guidance input message
    ReadFunctor<VSCMGSpeedMsgPayload> speedsInMsg;  //!< [-] VSCMG speeds input message
    Message<VSCMGRefStatesMsgPayload> vscmgRefStatesOutMsg;  //!< [-] reference VSCMG states C++ output message
    VSCMGRefStatesMsg_C               vscmgRefStatesOutMsgC = {};  //!< [-] reference VSCMG states C output message
    BSKLogger bskLogger;              //!< BSK Logging

    /** setter for `mu` property */
    void setMu(double);
    /** getter for `mu` property */
    double getMu() const {return this->mu;}
    /** setter for `W0_s` property */
    void setW0_s(std::vector<double>);
    /** getter for `W0_s` property */
    std::vector<double> getW0_s() const {return this->W0_s;}
    /** setter for `W_g` property */
    void setW_g(std::vector<double>);
    /** getter for `W_g` property */
    std::vector<double> getW_g() const {return this->W_g;}

private:
    double mu;  //!< [-] control parameter for wheel weights
    double h_bar_squared;  //!< [kg^2-m^4/s^4] square of nominal RW angular momentums
    std::vector<double> W0_s;  //!< [-] vector of static wheel weights
    std::vector<double> W_g;  //!< [-] vector of gimbal wheel weights
    std::vector<double> h_bar;  //!< [kg-m^2/s] vector of nominal RW angular momentums
    VSCMGRefStatesMsgPayload outputRefStates;  //!< [-] output reference states for the VSCMGs
    VSCMGArrayConfigMsgPayload vscmgConfigParams;  //!< [-] struct to store message containing VSCMG config parameters in body B frame

};


#endif
