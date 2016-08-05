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

#ifndef ExternalForceTorque_H
#define ExternalForceTorque_H

#include <vector>
#include <array>
#include "_GeneralModuleFiles/sys_model.h"
#include "_GeneralModuleFiles/dyn_effector.h"
#include "environment/spice/spice_planet_state.h"

/*! \addtogroup SimModelGroup
 * @{
 */

//! @brief The brief note is a single sentence to describe the function of this sim module.
/*! This is followed by a more lengthy description if necessary. This class is used to ...*/
/* Delete: The module class must inhert from SysModel class. The module class may inheret the DynEffector class is being included in uncoupled dynamics calculations. */
class ExternalForceTorque: public SysModel, public DynEffector{
    public:
    ExternalForceTorque();
    ~ExternalForceTorque();
    
    void SelfInit();
    void CrossInit();
    void UpdateState(uint64_t CurrentSimNanos);
    
    void writeOutputMessages(uint64_t CurrentClock);
    void readInputs();
    
    void ComputeDynamics(MassPropsData *massPropsData, OutputStateData *bodyState, double currentTime);
    
    private:
    
    public:
    double extForce_N[3];               //!< [N]  external force in inertial  frame components
    double extForce_B[3];               //!< [N]  external force in body frame components
    double extTorquePntB_B[3];          //!< [Nm] external torque in body frame components

    private:
};

/*! @} */

#endif /* ExternalForceTorque_H */
