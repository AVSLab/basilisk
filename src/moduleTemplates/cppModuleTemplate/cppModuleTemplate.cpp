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
#include "moduleTemplates/cppModuleTemplate/cppModuleTemplate.h"
#include "architecture/utilities/linearAlgebra.h"

/*! This is the constructor for the module class.  It sets default variable
    values and initializes the various parts of the model */
CppModuleTemplate::CppModuleTemplate()
{
}

/*! Module Destructor.  */
CppModuleTemplate::~CppModuleTemplate()
{
    return;
}


/*! This method is used to reset the module.

 */
void CppModuleTemplate::Reset(uint64_t CurrentSimNanos)
{
    /*! Reset the runtime counter; preserve the user-configured sampleConfigVector. */
    this->updateCounter = 0.0;
    bskLogger.bskLog(BSK_INFORMATION, "Variable updateCounter set to %f in reset.",this->updateCounter);

    /* zero output message on reset */
    CModuleTemplateMsgPayload outMsgBuffer={};       /*!< local output message copy */
    this->dataOutMsg.write(&outMsgBuffer, this->moduleID, CurrentSimNanos);
}


void CppModuleTemplate::UpdateState(uint64_t CurrentSimNanos)
{
    // Zero the output buffer each update to avoid publishing uninitialized fields.
    CModuleTemplateMsgPayload outMsgBuffer = this->dataOutMsg.zeroMsgPayload;
    CModuleTemplateMsgPayload inMsgBuffer;        /*!< local copy of input message */
    double inputVector[3];                       /*!< [-] sample input vector */

    // Use a zero vector when the optional input is not connected.
    v3SetZero(inputVector);

    /*! - Read the optional input messages */
    if (this->dataInMsg.isLinked()) {
        inMsgBuffer = this->dataInMsg();
        v3Copy(inMsgBuffer.dataVector, inputVector);
    }

    // Sample math: copy the input vector and add the counter to its first component.
    v3Copy(inputVector, outMsgBuffer.dataVector);
    this->updateCounter += 1.0;  // [-]
    outMsgBuffer.dataVector[0] += this->updateCounter;

    /*! - Write the module output message */
    this->dataOutMsg.write(&outMsgBuffer, this->moduleID, CurrentSimNanos);

    /* this logging statement is not typically required.  It is done here to see in the
     quick-start guide which module is being executed */
    bskLogger.bskLog(BSK_INFORMATION, "C++ Module ID %lld ran Update at %fs", this->moduleID, (double) CurrentSimNanos/(1e9));

}

void CppModuleTemplate::setUpdateCounter(double value)
{
    // check that value is in acceptable range
    if (value > 0) {
        this->updateCounter = value;
    } else {
        bskLogger.bskError("CppModuleTemplate: updateCounter variable must be strictly positive, you tried to set %f", value);
    }
}

void CppModuleTemplate::setSampleConfigVector(std::array<double, 3> value)
{
    // check that value is in acceptable range
    for (size_t i = 0; i < value.size(); i++) {
        if (value[i] <= 0.0) {
            bskLogger.bskError("CppModuleTemplate: sampleConfigVector components must be strictly positive");
        }
    }
    this->sampleConfigVector = value;
}
