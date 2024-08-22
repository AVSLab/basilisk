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
#include "fswAlgorithms/effectorInterfaces/forceInertialToBody/forceInertialToBody.h"
#include <iostream>
#include <cstring>
#include "architecture/utilities/rigidBodyKinematics.h"
#include "architecture/utilities/linearAlgebra.h"

/*! This is the constructor for the module class.  It sets default variable
    values and initializes the various parts of the model */
ForceInertialToBody::ForceInertialToBody()
{
}

/*! Module Destructor.  */
ForceInertialToBody::~ForceInertialToBody()
{
    return;
}


/*! This method is used to reset the module.
    @return void
 */
void ForceInertialToBody::Reset(uint64_t CurrentSimNanos)
{
    /* Check that required input messages are connected */
    if (!this->dataForceInertialInMsg.isLinked()) {
        bskLogger.bskLog(BSK_INFORMATION, "dataForceInertialInMsg is not linked");
    }
    if (!this->dataNavAttInMsg.isLinked()) {
        bskLogger.bskLog(BSK_INFORMATION, "dataForceInertialInMsg is not linked");
    }
}


/*! This is the main method that gets called every time the module is updated.  Provide an appropriate description.
    @return void
 */
void ForceInertialToBody::UpdateState(uint64_t CurrentSimNanos)
{
    CmdForceBodyMsgPayload cmdForceBodyMsgBuffer;               /*!< local output message copy */
    CmdForceInertialMsgPayload cmdForceInertialMsgBuffer;        /*!< local copy of input message1 */
    NavAttMsgPayload navAttMsgBuffer;                             /*!< local copy of input message1 */
    double force_inertial[3];
    double sigma_BN[3];
    double dcm_BN[3][3];
    double force_body[3];

    // always zero the output buffer first
    cmdForceBodyMsgBuffer = this->ForceBodyMsg.zeroMsgPayload;
    v3SetZero(force_body);

    /*! - Read the optional input messages */
    if (this->dataForceInertialInMsg.isLinked()) {
        cmdForceInertialMsgBuffer = this->dataForceInertialInMsg();
        v3Copy(cmdForceInertialMsgBuffer.forceRequestInertial, force_inertial);
    }

    if (this->dataNavAttInMsg.isLinked()) {
        navAttMsgBuffer = this->dataNavAttInMsg();
        v3Copy(navAttMsgBuffer.sigma_BN, sigma_BN);
    }

    /*! - rotate force_inertial using sigma_BN */
    MRP2C(sigma_BN, dcm_BN);
    m33tMultV3(dcm_BN, force_inertial, force_body);
    /*! - store the output message */
    v3Copy(force_body, cmdForceBodyMsgBuffer.forceRequestBody);

    /*! - write the module output message */
    this->ForceBodyMsg.write(&cmdForceBodyMsgBuffer, this->moduleID, CurrentSimNanos);

    /* this logging statement is not typically required.  It is done here to see in the
     quick-start guide which module is being executed */
    bskLogger.bskLog(BSK_INFORMATION, "C++ Module ID %lld ran Update at %fs", this->moduleID, (double) CurrentSimNanos/(1e9));

}
