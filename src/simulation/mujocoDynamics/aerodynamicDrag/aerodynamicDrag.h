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
#ifndef AERODYNAMIC_DRAG_H
#define AERODYNAMIC_DRAG_H

#include "architecture/utilities/bskLogging.h"
#include "architecture/messaging/messaging.h"
#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "simulation/mujocoDynamics/_GeneralModuleFiles/MJScene.h"

#include "architecture/msgPayloadDefC/DragGeometryMsgPayload.h"
#include "architecture/msgPayloadDefC/AtmoPropsMsgPayload.h"
#include "architecture/msgPayloadDefC/ForceAtSiteMsgPayload.h"
#include "architecture/msgPayloadDefC/TorqueAtSiteMsgPayload.h"

/**
 * @brief Aerodynamic drag model that computes force and torque at a MuJoCo site.
 *
 * Updates a force and a torque to represent quasi-steady aerodynamic drag acting
 * at a site fixed on a body. Force direction is opposite the relative flow.
 * Magnitude follows \f$ F = \tfrac{1}{2}\rho v^2 C_D A \f$.
 * Torque is \f$ \tau = r_{CP/S} \times F \f$ in the site frame.
 *
 * The force and torque output in `forceOutMsg` and `torqueOutMsg` are given
 * in the site reference frame whose state is given in `referenceFrameStateInMsg`.
 */
class AerodynamicDrag : public SysModel
{
public:
    /**
     * @brief Bind this model to an MJBody target.
     *
     * If this method is used, only the `dragGeometryInMsg` and
     * `atmoDensInMsg` must be linked.
     *
     * @param body MuJoCo body that will receive the drag force and torque at it's center of mass.
     * @return Reference to the actuator on which the forceand torque is applied.
     */
    MJForceTorqueActuator& applyTo(MJBody& body);

    /**
     * @brief Bind this model to an MJSite target.
     *
     * If this method is used, only the `dragGeometryInMsg` and
     * `atmoDensInMsg` must be linked.
     *
     * @param site MuJoCo site where the force application point is defined.
     * @return Reference to the actuator on which the forceand torque is applied.
     */
    MJForceTorqueActuator& applyTo(MJSite& site);

    /**
     * @brief Bind this model to an existing force/torque actuator.
     *
     * If this method is used, the `dragGeometryInMsg`,
     * `atmoDensInMsg`, and `referenceFrameStateInMsg` must be linked.
     *
     * @param actuator Precreated actuator that will receive the computed wrench.
     * @return Reference to the same actuator.
     *
     * @note You
     */
    MJForceTorqueActuator& applyTo(MJForceTorqueActuator& actuator);


    /**
     * @brief Advance the model and publish outputs.
     * @param CurrentSimNanos Current simulation time in nanoseconds.
     */
    void UpdateState(uint64_t CurrentSimNanos) override;

    /**
     * @brief Validate all input messages are linked.
     * @param CurrentSimNanos Current simulation time in nanoseconds.
     */
    void Reset(uint64_t CurrentSimNanos) override;


public:

    /** @brief Geometry input. */
    ReadFunctor<DragGeometryMsgPayload> dragGeometryInMsg;

    /** @brief Atmospheric properties input. */
    ReadFunctor<AtmoPropsMsgPayload> atmoDensInMsg;

    /** @brief State of the reference frame . */
    ReadFunctor<SCStatesMsgPayload> referenceFrameStateInMsg;

    /** @brief Output force at the site in the site reference frame. */
    Message<ForceAtSiteMsgPayload> forceOutMsg;

    /** @brief Output torque in the site reference frame.*/
    Message<TorqueAtSiteMsgPayload> torqueOutMsg;


    /** @brief Logger */
    BSKLogger bskLogger;
};


#endif
