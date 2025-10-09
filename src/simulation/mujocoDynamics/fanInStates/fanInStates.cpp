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


#include "simulation/mujocoDynamics/fanInStates/fanInStates.h"
#include <iostream>
#include <cstring>
#include <mujoco/mujoco.h>

/*! Initialize C-wrapped output messages */
void
FanInStates::SelfInit(){
    JointArrayStateMsg_C_init(&this->jointStatesOutMsgC);
}

/*! This is the constructor for the module class.  It sets default variable
    values and initializes the various parts of the model */
FanInStates::FanInStates()
{
    if (!scene) {
        bskLogger.bskLog(BSK_ERROR, "FanInStates: scene pointer not set!");
    }
}

/*! This method is used to reset the module and checks that required input messages are connect.
*/
void FanInStates::Reset(uint64_t CurrentSimNanos)
{
    // check that required input messages are connected

}


/*! This is the main method that gets called every time the module is updated.  Provide an appropriate description.
*/
void FanInStates::UpdateState(uint64_t CurrentSimNanos)
{
    const auto data = scene->getMujocoData();
    JointArrayStateMsgPayload jointStatesOut;  //!< local copy of message buffer

    // always zero the output message buffers before assigning values
    jointStatesOut = this->jointStatesOutMsg.zeroMsgPayload;

    for (int i = 0; i < 8; ++i) {
        int idx = 6+i;
        jointStatesOut.thetas[i] = data->qpos[idx+1];
        jointStatesOut.thetaDots[i] = data->qvel[idx];
    }

    // write to the output messages
    this->jointStatesOutMsg.write(&jointStatesOut, this->moduleID, CurrentSimNanos);
    // Write the C-wrapped output message
    JointArrayStateMsg_C_write(&jointStatesOut, &this->jointStatesOutMsgC, this->moduleID, CurrentSimNanos);
}
