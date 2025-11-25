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
#include "aerodynamicDrag.h"

#include <Eigen/Dense>

void
AerodynamicDrag::UpdateState(uint64_t CurrentSimNanos)
{
    const double density = this->atmoDensInMsg().neutralDensity;

    const auto geomPayload = this->dragGeometryInMsg();
    const double cd = geomPayload.dragCoeff;
    const double area = geomPayload.projectedArea;
    const Eigen::Map<const Eigen::Vector3d> r_CP_s(geomPayload.r_CP_S);

    // 'S' denotes the 'site' reference frame
    const auto siteStatePayload = this->referenceFrameStateInMsg();
    Eigen::MRPd sigma_SN;
    sigma_SN = Eigen::Map<const Eigen::Vector3d>(siteStatePayload.sigma_BN);
    const Eigen::Matrix3d dcm_SN = sigma_SN.toRotationMatrix().transpose();

    // inertial velocity of the center of the S frame, expressed in the S frame
    const auto v_S = dcm_SN*Eigen::Map<const Eigen::Vector3d>(siteStatePayload.v_BN_N);
    const auto v = v_S.norm();

    if (v < 1e-10)
    {
        this->forceOutMsg.write(&this->forceOutMsg.zeroMsgPayload, this->moduleID, CurrentSimNanos);
        this->torqueOutMsg.write(&this->torqueOutMsg.zeroMsgPayload, this->moduleID, CurrentSimNanos);
        return;
    }

    const Eigen::Vector3d force_S = -0.5 * cd * area * density * v * v_S;
    const Eigen::Vector3d torque_S = r_CP_s.cross(force_S);

    ForceAtSiteMsgPayload forcePayload = this->forceOutMsg.zeroMsgPayload;
    TorqueAtSiteMsgPayload torquePayload = this->torqueOutMsg.zeroMsgPayload;
    std::copy_n(force_S.data(), 3, forcePayload.force_S);
    std::copy_n(torque_S.data(), 3, torquePayload.torque_S);

    this->forceOutMsg.write(&forcePayload, this->moduleID, CurrentSimNanos);
    this->torqueOutMsg.write(&torquePayload, this->moduleID, CurrentSimNanos);}

void AerodynamicDrag::Reset(uint64_t /*CurrentSimNanos*/)
{
    if (!this->atmoDensInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR,
            "AerodynamicDrag: atmoDensInMsg is not linked. "
            "Call atmoDensInMsg.subscribeTo(<AtmoPropsMsg>) before simulation.");
    }

    if (!this->dragGeometryInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR,
            "AerodynamicDrag: dragGeometryInMsg is not linked. "
            "Call dragGeometryInMsg.subscribeTo(<DragGeometryMsg>) before simulation.");
    }

    if (!this->referenceFrameStateInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR,
            "AerodynamicDrag: referenceFrameStateInMsg is not linked. "
            "Call referenceFrameStateInMsg.subscribeTo(<SiteStateMsg>) before simulation.");
    }
}


MJForceTorqueActuator&
AerodynamicDrag::applyTo(MJBody& body)
{
    return applyTo(body.getCenterOfMass());
}

MJForceTorqueActuator&
AerodynamicDrag::applyTo(MJSite& site)
{
    this->referenceFrameStateInMsg.subscribeTo( &site.stateOutMsg );
    const auto actuatorName = ModelTag + "_aerodynamicDrag_" + site.getName();
    auto& actuator = site.getBody().getSpec().getScene().addForceTorqueActuator(actuatorName, site);
    return applyTo(actuator);
}

MJForceTorqueActuator&
AerodynamicDrag::applyTo(MJForceTorqueActuator& actuator)
{
    actuator.forceInMsg.subscribeTo( &this->forceOutMsg );
    actuator.torqueInMsg.subscribeTo( &this->torqueOutMsg );
    return actuator;
}
