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
#include <iostream>
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
    /*! - reset any required variables */
    this->dummy = 0.0;
    bskLogger.bskLog(BSK_INFORMATION, "Variable dummy set to %f in reset.",this->dummy);

    /* zero output message on reset */
    CModuleTemplateMsgPayload outMsgBuffer={};       /*!< local output message copy */
    this->dataOutMsg.write(&outMsgBuffer, this->moduleID, CurrentSimNanos);
}


/*! This is the main method that gets called every time the module is updated.  Provide an appropriate description.

 */
void CppModuleTemplate::UpdateState(uint64_t CurrentSimNanos)
{
    double Lr[3];                                   /*!< [unit] variable description */
    CModuleTemplateMsgPayload outMsgBuffer;       /*!< local output message copy */
    CModuleTemplateMsgPayload inMsgBuffer;        /*!< local copy of input message */
    double  inputVector[3];

    // always zero the output buffer first
    outMsgBuffer = this->dataOutMsg.zeroMsgPayload;
    v3SetZero(inputVector);

    /*! - Read the optional input messages */
    if (this->dataInMsg.isLinked()) {
        inMsgBuffer = this->dataInMsg();
        v3Copy(inMsgBuffer.dataVector, inputVector);
    }

    /*! - Add the module specific code */
    v3Copy(inputVector, Lr);
    this->dummy += 1.0;
    Lr[0] += this->dummy;

    /*! - store the output message */
    v3Copy(Lr, outMsgBuffer.dataVector);

    /*! - write the module output message */
    this->dataOutMsg.write(&outMsgBuffer, this->moduleID, CurrentSimNanos);

    /* this logging statement is not typically required.  It is done here to see in the
     quick-start guide which module is being executed */
    bskLogger.bskLog(BSK_INFORMATION, "C++ Module ID %lld ran Update at %fs", this->moduleID, (double) CurrentSimNanos/(1e9));

}

void CppModuleTemplate::setDummy(double value)
{
    // check that value is in acceptable range
    if (value > 0) {
        this->dummy = value;
    } else {
        bskLogger.bskLog(BSK_ERROR, "CppModuleTemplate: dummy variable must be strictly positive, you tried to set %f", value);
    }
}

void CppModuleTemplate::setDumVector(std::array<double, 3> value)
{
    // check that value is in acceptable range
    for (int i = 0; i < 3; i++) {
        if (value[i] <= 0.0) {
            bskLogger.bskLog(BSK_ERROR, "CppModuleTemplate: dumVariable variables must be strictly positive");
            return;
        }
    }
    this->dumVector = value;
}
