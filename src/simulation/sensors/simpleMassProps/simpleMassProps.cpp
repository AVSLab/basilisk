/*
 ISC License

 Copyright (c) 2021, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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
#include "simulation/sensors/simpleMassProps/simpleMassProps.h"
#include <iostream>
#include "architecture/utilities/linearAlgebra.h"

/*! This is the constructor for the module class. */
SimpleMassProps::SimpleMassProps()
{
    return;
}

/*! Module Destructor.  */
SimpleMassProps::~SimpleMassProps()
{
    return;
}


/*! This method is used to reset the module.

 */
void SimpleMassProps::Reset(uint64_t CurrentSimNanos)
{
    // check if input message is linked
    if (!this->scMassPropsInMsg.isLinked())
    {
        bskLogger.bskLog(BSK_ERROR, "simpleMassProps.scMassPropsInMsg is not linked.");
    }

    // zero the incoming message buffer
    this->scMassPropsMsgBuffer = this->scMassPropsInMsg.zeroMsgPayload;

    // call the UpdateState function. This make sure the vehicleConfig message is populated with the correct values upon the initialization of the simulation.
    // Some FSW modules like mrpFeedback require this, as they only set the necessary mas properties on Reset and not throughout the sim.
    UpdateState(CurrentSimNanos);
}

/*! This method reads the spacecraft mass properties state input message
 */
void SimpleMassProps::readInputMessages()
{
    // read the incoming power message and transfer it to the data buffer
    this->scMassPropsMsgBuffer = this->scMassPropsInMsg();

    return;
}

/*! This method writes the vehicle configuration output message.

 @param CurrentClock The clock time associated with the model call
 */
void SimpleMassProps::writeOutputMessages(uint64_t CurrentClock)
{
    // write the output message
    this->vehicleConfigOutMsg.write(&this->vehicleConfigMsgBuffer, this->moduleID, CurrentClock);

    return;
}

/*! This method transfers the spacecraft mass propertiies information to a FSW format
 */
void SimpleMassProps::computeMassProperties()
{
    // copy the mass value
    this->vehicleConfigMsgBuffer.massSC = this->scMassPropsMsgBuffer.massSC;

    // copy the inertia value
    for(uint64_t i = 0; i < 3; i++){
        for (uint64_t j = 0; j < 3; j++) {
            this->vehicleConfigMsgBuffer.ISCPntB_B[3*i + j] = this->scMassPropsMsgBuffer.ISC_PntB_B[i][j];
        }
    }

    // transfer the center of mass
    v3Copy(this->scMassPropsMsgBuffer.c_B, this->vehicleConfigMsgBuffer.CoM_B);

    return;
}


/*! This is the main method that gets called every time the module is updated. It reads the simulation message, transfers its contents and writes to an output FSW message.

 */
void SimpleMassProps::UpdateState(uint64_t CurrentSimNanos)
{
    readInputMessages();
    computeMassProperties();
    writeOutputMessages(CurrentSimNanos);

    return;
}
