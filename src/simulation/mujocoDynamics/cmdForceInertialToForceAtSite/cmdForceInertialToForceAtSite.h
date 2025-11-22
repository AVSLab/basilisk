/*
 ISC License

 Copyright (c) 2025, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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
#ifndef CMD_FORCE_INERTIAL_TO_FORCE_AT_SITE_H
#define CMD_FORCE_INERTIAL_TO_FORCE_AT_SITE_H

#include "architecture/utilities/bskLogging.h"
#include "architecture/messaging/messaging.h"
#include "architecture/_GeneralModuleFiles/sys_model.h"

#include "architecture/msgPayloadDefC/CmdForceInertialMsgPayload.h"
#include "architecture/msgPayloadDefC/ForceAtSiteMsgPayload.h"
#include "architecture/msgPayloadDefC/SCStatesMsgPayload.h"
#include "architecture/msgPayloadDefC/NavAttMsgPayload.h"

/**
 * @brief Convert a commanded force in the inertial frame into a force at a site frame.
 *
 * Computes a force vector expressed in the site frame S from an input force expressed in the
 * inertial frame N. Uses either the site attitude message (preferred) or the site state message
 * to obtain orientation S relative to N. The output message contains the force expressed in S.
 */
class CmdForceInertialToForceAtSite : public SysModel
{
public:
    /** @name Framework interface @{ */

    /**
     * @brief Advance the model and publish outputs.
     * Reads forceRequestInertial in N, builds C_SN from sigma_BN, computes F_S, and writes forceOutMsg.
     *
     * @param CurrentSimNanos Current simulation time in nanoseconds.
     */
    void UpdateState(uint64_t CurrentSimNanos) override;

    /**
     * @brief Validate input links.
     * @param CurrentSimNanos Current simulation time in nanoseconds.
     */
    void Reset(uint64_t CurrentSimNanos) override;

    /** @} */

public:
    /** @name I/O Messages @{ */

    /// Commanded force in N
    ReadFunctor<CmdForceInertialMsgPayload> cmdForceInertialInMsg;

    /// Site state providing sigma_BN as a fallback orientation source
    ReadFunctor<SCStatesMsgPayload> siteFrameStateInMsg;

    /// Site attitude providing sigma_BN as the primary orientation source
    ReadFunctor<NavAttMsgPayload> siteAttInMsg;

    /// Output force expressed in S
    Message<ForceAtSiteMsgPayload> forceOutMsg;

    /** @} */

    /// Logger
    BSKLogger bskLogger;
};

#endif
