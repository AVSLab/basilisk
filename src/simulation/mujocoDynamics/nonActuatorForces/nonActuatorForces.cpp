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


#include "simulation/mujocoDynamics/nonActuatorForces/nonActuatorForces.h"
#include <iostream>
#include <cstring>
#include <mujoco/mujoco.h>

/*! Initialize C-wrapped output messages */
void
NonActuatorForces::SelfInit(){
    MJNonActuatorForcesMsg_C_init(&this->forcesOutMsgC);
}

/*! This is the constructor for the module class.  It sets default variable
    values and initializes the various parts of the model */
NonActuatorForces::NonActuatorForces()
{
}

/*! This method is used to reset the module and checks that required input messages are connect.
*/
void NonActuatorForces::Reset(uint64_t CurrentSimNanos)
{
    if (!scene) {
        bskLogger.bskLog(BSK_ERROR, "NonActuatorForces: scene pointer not set!");
    }

    const auto model = scene->getMujocoModel();
    // extract the DOF dimensions
    this->nDOF = model -> nv;
    this->nbase = (model->nv >= 6 ? 6 : 0);
    this->nj = std::max(0, this->nDOF - this->nbase);

    if (this->nbase + this->nj > 6 + MAX_EFF_CNT) {
        bskLogger.bskLog(BSK_ERROR, "NonActuatorForces: number of DOF (%d) exceeds message capacity (%d)!", this->nbase+this->nj, 6+MAX_EFF_CNT);
    }
}


/*! This is the main method that gets called every time the module is updated.  Provide an appropriate description.
*/
void NonActuatorForces::UpdateState(uint64_t CurrentSimNanos)
{
    auto data = scene->getMujocoData();

    MJNonActuatorForcesMsgPayload forcesOut;  //!< local copy of message buffer

    // always zero the output message buffers before assigning values
    forcesOut = this->forcesOutMsg.zeroMsgPayload;

    // check if there are any constraints currently active
    const bool constraints_current = (data->nefc > 0);

    // Extract the non-actuator forces on the base
    if (this->nbase >= 6) {
        // free-floating base
        for (int i = 0; i < 3; i++) {
            forcesOut.baseTransForces[i] = -data->qfrc_bias[i];
            forcesOut.baseTransForces[i] += data->qfrc_applied[i];
            forcesOut.baseTransForces[i] += data->qfrc_passive[i];

            forcesOut.baseRotForces[i] = -data->qfrc_bias[i + 3];
            forcesOut.baseRotForces[i] += data->qfrc_applied[i + 3];
            forcesOut.baseRotForces[i] += data->qfrc_passive[i + 3];

            if (constraints_current) {
                forcesOut.baseTransForces[i] += data->qfrc_constraint[i];
                forcesOut.baseRotForces[i] += data->qfrc_constraint[i + 3];
            }

        }

    } else {
        // fixed base
        for (int i = 0; i < 3; i++) {
            forcesOut.baseTransForces[i] = 0.0;
            forcesOut.baseRotForces[i] = 0.0;
        }
    }

    // Extract the non-actuator forces on the joints
    for (int i = 0; i < this->nj; i++) {
        forcesOut.jointForces[i] = -data->qfrc_bias[i + this->nbase];
        forcesOut.jointForces[i] += data->qfrc_applied[i + this->nbase];
        forcesOut.jointForces[i] += data->qfrc_passive[i + this->nbase];
        if (constraints_current)
            forcesOut.jointForces[i] += data->qfrc_constraint[i + this->nbase];
    }

    // write to the output messages
    this->forcesOutMsg.write(&forcesOut, this->moduleID, CurrentSimNanos);
    // also write to the C-wrapped output message
    MJNonActuatorForcesMsg_C_write(&forcesOut, &this->forcesOutMsgC, this->moduleID, CurrentSimNanos);
}
