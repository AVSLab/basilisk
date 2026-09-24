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
#include <stdint.h>

/*! @brief basic Basilisk C++ module class */
class CppModuleTemplate: public SysModel {
public:
    CppModuleTemplate();
    ~CppModuleTemplate() override;

    /*! @brief Reset the counter and publish a zero output payload.
     *  @param CurrentSimNanos [ns] Time at which the module is reset.
     *  @note The sampleConfigVector configuration is preserved.
     */
    void Reset(uint64_t CurrentSimNanos) override;
    /*! @brief Add the update counter to the first component of the optional input vector.
     *  @param CurrentSimNanos [ns] Time at which the module is updated.
     */
    void UpdateState(uint64_t CurrentSimNanos) override;

    Message<CModuleTemplateMsgPayload> dataOutMsg;     //!< attitude navigation output msg
    ReadFunctor<CModuleTemplateMsgPayload> dataInMsg;  //!< translation navigation output msg

    BSKLogger bskLogger;                               //!< BSK Logging

    /*! @brief Demonstrate scalar assignment and validation using the runtime counter.
     *  @param value [-] Positive counter value.
     *  @note Reset() always clears the counter, including after this setter is used.
     */
    void setUpdateCounter(double value);
    /*! @brief Read the runtime counter, for example for variable logging.
     *  @return [-] Current counter value.
     */
    double getUpdateCounter() const {return this->updateCounter;}
    /*! @brief Set the sample configuration vector used for variable logging.
     *  @param value [-] Sample vector with positive components.
     *  @note Reset() preserves this vector. The vector is unused by the output calculation.
     */
    void setSampleConfigVector(std::array<double, 3> value);
    /*! @brief Read the sample configuration vector.
     *  @return [-] Current sample vector.
     */
    std::array<double, 3> getSampleConfigVector() const {return this->sampleConfigVector;}

private:

    double updateCounter = {};                       //!< [-] Runtime counter; Reset clears it and UpdateState increments it.
    std::array<double, 3> sampleConfigVector = {};     //!< [-] Sample configuration for logging; retained by Reset and unused by UpdateState.

};


#endif
