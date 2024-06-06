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

#ifndef CPP_MODULE_TEMPLATE_H
#define CPP_MODULE_TEMPLATE_H

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/msgPayloadDefC/CModuleTemplateMsgPayload.h"
#include "architecture/utilities/bskLogging.h"
#include "architecture/messaging/messaging.h"

#include <array>

/*! @brief basic Basilisk C++ module class */
class CppModuleTemplate: public SysModel {
public:
    CppModuleTemplate();
    ~CppModuleTemplate();

    void Reset(uint64_t CurrentSimNanos);
    void UpdateState(uint64_t CurrentSimNanos);

    Message<CModuleTemplateMsgPayload> dataOutMsg;     //!< attitude navigation output msg
    ReadFunctor<CModuleTemplateMsgPayload> dataInMsg;  //!< translation navigation output msg

    BSKLogger bskLogger;                               //!< BSK Logging

    /** setter for `dummy` property */
    void setDummy(double value);
    /** getter for `dummy` property */
    double getDummy() const {return this->dummy;}
    /** setter for `dumVector` property */
    void setDumVector(std::array<double, 3> value);
    /** getter for `dumVector` property */
    std::array<double, 3> getDumVector() const {return this->dumVector;}

private:

    double dummy = {};                                 //!< [units] sample module variable declaration
    std::array<double, 3> dumVector = {};              //!< [units] sample vector variable

};


#endif
