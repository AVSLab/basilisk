/*
 ISC License

 Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

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

#ifndef _EPHEM_DIFFERENCE_WITH_UNCERTAINTY_H_
#define _EPHEM_DIFFERENCE_WITH_UNCERTAINTY_H_

#include <stdint.h>
#include <Eigen/Dense>
#include "architecture/utilities/avsEigenSupport.h"
#include "architecture/messaging/messaging.h"

#include "architecture/msgPayloadDefC/EphemerisMsgPayload.h"
#include "architecture/msgPayloadDefC/NavTransMsgPayload.h"
#include "architecture/msgPayloadDefCpp/FilterMsgPayload.h"

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/utilities/bskLogging.h"

/*! @brief This module computes the difference between two ephemeris messages, and outputs the relative states into a
 * navigation and filter message */
class EphemDifferenceWithUncertainty: public SysModel {
public:
    EphemDifferenceWithUncertainty();
    ~EphemDifferenceWithUncertainty();

    void UpdateState(uint64_t currentSimNanos) override;
    void Reset(uint64_t currentSimNanos) override;

    ReadFunctor<EphemerisMsgPayload> ephemBaseInMsg;
    ReadFunctor<EphemerisMsgPayload> ephemSecondaryInMsg;
    Message<NavTransMsgPayload> navTransOutMsg;
    Message<FilterMsgPayload> filterOutMsg;

    BSKLogger bskLogger;
};

#endif
