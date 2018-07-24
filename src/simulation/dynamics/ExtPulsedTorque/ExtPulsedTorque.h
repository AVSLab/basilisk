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

#include "_GeneralModuleFiles/sys_model.h"
#include "../_GeneralModuleFiles/dynamicEffector.h"



/*! \addtogroup SimModelGroup
 * @{
 */
    
//! @brief dynEffector Class used to provide a direct pulsed external torque on body
/*!  The module
 [PDF Description](Basilisk-extPulsedTorque-20170324.pdf)
 contains further information on this module's function,
 how to run it, as well as testing.
*/
class ExtPulsedTorque: public SysModel, public DynamicEffector{
public:
    ExtPulsedTorque();
    ~ExtPulsedTorque();

    void SelfInit();
    void CrossInit();
    void UpdateState(uint64_t CurrentSimNanos);
    void linkInStates(DynParamManager& statesIn);
    void writeOutputMessages(uint64_t currentClock);
    void readInputMessages();
    void computeForceTorque(double integTime);

private:
    int    c;                                   //!< numer of time steps for pulse

public:
    Eigen::Vector3d pulsedTorqueExternalPntB_B; //!< pulsed torque vector about point B, in B frame components 
    int countOnPulse;                           //!< number of integration time steps to simulate a pulse
    int countOff;                               //!< number of integration time steps to have no pulses

};

/*! @} */

#endif
