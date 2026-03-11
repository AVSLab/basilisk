/*
 ISC License

 Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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
#include "cannonballDrag.h"

#include "architecture/utilities/avsEigenMRP.h"

void
CannonballDrag::SelfInit()
{
    ForceAtSiteMsg_C_init(&this->forceOutMsgC);
    TorqueAtSiteMsg_C_init(&this->torqueOutMsgC);
}

void
CannonballDrag::UpdateState(uint64_t CurrentSimNanos)
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

    // velocity used for drag: inertial or atmosphere-relative
    Eigen::Vector3d vRel_N = Eigen::Map<const Eigen::Vector3d>(siteStatePayload.v_BN_N);
    if (this->useAtmosphereRelativeVelocity) {
        const Eigen::Map<const Eigen::Vector3d> r_N(siteStatePayload.r_BN_N);
        vRel_N -= this->planetOmega_N.cross(r_N);
    }
    const auto v_S = dcm_SN * vRel_N;
    const auto v = v_S.norm();

    const Eigen::Vector3d force_S = -0.5 * cd * area * v * density * v_S;
    const Eigen::Vector3d torque_S = r_CP_s.cross(force_S);

    ForceAtSiteMsgPayload forcePayload = this->forceOutMsg.zeroMsgPayload;
    TorqueAtSiteMsgPayload torquePayload = this->torqueOutMsg.zeroMsgPayload;
    std::copy_n(force_S.data(), 3, forcePayload.force_S);
    std::copy_n(torque_S.data(), 3, torquePayload.torque_S);

    this->forceOutMsg.write(&forcePayload, this->moduleID, CurrentSimNanos);
    this->torqueOutMsg.write(&torquePayload, this->moduleID, CurrentSimNanos);

    ForceAtSiteMsg_C_write(&forcePayload, &this->forceOutMsgC, this->moduleID, CurrentSimNanos);
    TorqueAtSiteMsg_C_write(&torquePayload, &this->torqueOutMsgC, this->moduleID, CurrentSimNanos);
}

/*!
 * @brief Enables or disables the use of atmosphere-relative velocity for drag computation.
 * When enabled, the drag force is computed using v_rel = v_sc - (omega_planet x r_sc).
 * @param useRelVel  true to use atmosphere-relative velocity, false to use inertial velocity
 */
void CannonballDrag::setUseAtmosphereRelativeVelocity(bool useRelVel)
{
    this->useAtmosphereRelativeVelocity = useRelVel;
}

/*!
 * @brief Returns whether atmosphere-relative velocity is used in drag computation.
 * @return true if atmosphere-relative velocity is enabled
 */
bool CannonballDrag::getUseAtmosphereRelativeVelocity() const
{
    return this->useAtmosphereRelativeVelocity;
}

/*!
 * @brief Sets the planetary rotation vector expressed in the inertial frame.
 * Used to compute the atmosphere velocity when useAtmosphereRelativeVelocity is enabled.
 * For Earth, the default is taken from OMEGA_EARTH in astroConstants.h: [0, 0, 7.2921159e-5] rad/s.
 * @param omega  Planetary rotation vector [rad/s] in inertial frame N
 */
void CannonballDrag::setPlanetOmega_N(const Eigen::Vector3d& omega)
{
    this->planetOmega_N = omega;
}

/*!
 * @brief Returns the planetary rotation vector expressed in the inertial frame.
 * @return omega_planet [rad/s] in inertial frame N
 */
Eigen::Vector3d CannonballDrag::getPlanetOmega_N() const
{
    return this->planetOmega_N;
}

void CannonballDrag::Reset(uint64_t /*CurrentSimNanos*/)
{
    if (!this->atmoDensInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR,
            "CannonballDrag: atmoDensInMsg is not linked. "
            "Call atmoDensInMsg.subscribeTo(<AtmoPropsMsg>) before simulation.");
    }

    if (!this->dragGeometryInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR,
            "CannonballDrag: dragGeometryInMsg is not linked. "
            "Call dragGeometryInMsg.subscribeTo(<DragGeometryMsg>) before simulation.");
    }

    if (!this->referenceFrameStateInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR,
            "CannonballDrag: referenceFrameStateInMsg is not linked. "
            "Call referenceFrameStateInMsg.subscribeTo(<SiteStateMsg>) before simulation.");
    }
}
