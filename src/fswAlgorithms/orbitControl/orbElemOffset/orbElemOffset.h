/*
 ISC License

 Copyright (c) 2025, Autonomous Vehicle Systems Lab

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

#ifndef ORB_ELEM_OFFSET_H
#define ORB_ELEM_OFFSET_H

#include <cstdint>

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/messaging/messaging.h"
#include "architecture/msgPayloadDefC/ClassicElementsMsgPayload.h"
#include "architecture/utilities/bskLogging.h"

/*!
 * @brief Classic orbital elements combiner with optional mean anomaly offset handling.
 *
 * This module reads two ClassicElementsMsgPayload inputs:
 *
 *  - mainElementsInMsg: nominal or "base" classical orbital elements
 *  - offsetElementsInMsg: element offsets to apply to the base elements
 *
 * For most fields the module performs a simple element wise addition
 *
 * \code
 *   out.a      = main.a      + offset.a
 *   out.e      = main.e      + offset.e
 *   out.i      = main.i      + offset.i
 *   out.Omega  = main.Omega  + offset.Omega
 *   out.omega  = main.omega  + offset.omega
 * \endcode
 *
 * The treatment of the true anomaly field f depends on the flag useMeanAnomalyOffset
 *
 * - If useMeanAnomalyOffset is false (default) then the offsetElementsInMsg.f
 *   field is interpreted as a true anomaly increment and the output is
 *
 *   \code
 *   out.f = main.f + offset.f
 *   \endcode
 *
 * - If useMeanAnomalyOffset is true then offsetElementsInMsg.f is interpreted
 *   as a mean anomaly offset \f$\Delta M\f$ in radians The module then
 *
 *   1 Converts the main elements (main.f main.e) to mean anomaly M_main
 *   2 Forms the final eccentricity as e_out = main.e + offset.e
 *   3 Forms the final mean anomaly M_out = M_main + offset.f
 *   4 Converts (M_out e_out) back to a true anomaly f_out
 *
 *   and sets out.f = f_out
 *
 * All other elements in ClassicElementsMsgPayload are left to zero.
 *
 * The output payload is written to elementsOutMsg.
 */
class OrbElemOffset : public SysModel
{
public:
    /** Default constructor */
    OrbElemOffset() = default;

    /*!
     * @brief Reset the internal state and validate input message connections.
     *
     * This method checks that both mainElementsInMsg and offsetElementsInMsg
     * are connected If either is not linked an error is logged through
     * the module logger
     *
     * @param CurrentSimNanos Current simulation time in nanoseconds (unused)
     */
    void Reset(uint64_t CurrentSimNanos) override;

    /*!
     * @brief Compute the combined classical orbital elements and write them out.
     *
     * The module reads both input messages, constructs the output classical
     * elements according to the rules described in the class documentation,
     * and writes the result to elementsOutMsg
     *
     * @param CurrentSimNanos Current simulation time in nanoseconds
     */
    void UpdateState(uint64_t CurrentSimNanos) override;

public:
    /*!
     * @brief Toggle interpretation of offsetElementsInMsg.f as a mean anomaly offset.
     *
     * - false: offsetElementsInMsg.f is treated as a true anomaly increment
     * - true: offsetElementsInMsg.f is treated as a mean anomaly offset \f$\Delta M\f$
     */
    bool useMeanAnomalyOffset = false;

    //! Nominal or "main" classical elements input message
    ReadFunctor<ClassicElementsMsgPayload> mainElementsInMsg;

    //! Classical elements offset input message
    ReadFunctor<ClassicElementsMsgPayload> offsetElementsInMsg;

    //! Output classical elements message for the combined elements
    Message<ClassicElementsMsgPayload> elementsOutMsg;

    BSKLogger bskLogger;   //!< BSK Logging
};

#endif /* ORB_ELEM_OFFSET_H */
