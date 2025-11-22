#include "cmdForceInertialToForceAtSite.h"

#include <Eigen/Dense>
#include "architecture/utilities/avsEigenMRP.h"

void
CmdForceInertialToForceAtSite::UpdateState(uint64_t CurrentSimNanos)
{
    // 'S' denotes the 'site' reference frame
    Eigen::MRPd sigma_SN;
    if (this->siteAttInMsg.isLinked())
    {
        sigma_SN = Eigen::Map<const Eigen::Vector3d>(this->siteAttInMsg().sigma_BN);
    }
    else
    {
        sigma_SN = Eigen::Map<const Eigen::Vector3d>(this->siteFrameStateInMsg().sigma_BN);
    }
    const Eigen::Matrix3d dcm_SN = sigma_SN.toRotationMatrix().transpose();

    Eigen::Map<const Eigen::Vector3d> force_N{this->cmdForceInertialInMsg().forceRequestInertial};
    const Eigen::Vector3d force_S = dcm_SN*force_N;

    ForceAtSiteMsgPayload forcePayload = this->forceOutMsg.zeroMsgPayload;
    std::copy_n(force_S.data(), 3, forcePayload.force_S);
    this->forceOutMsg.write(&forcePayload, this->moduleID, CurrentSimNanos);
}

void CmdForceInertialToForceAtSite::Reset(uint64_t /*CurrentSimNanos*/)
{
    if (!this->cmdForceInertialInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR,
            "CmdForceInertialToForceAtSite: cmdForceInertialInMsg is not linked. "
            "Call cmdForceInertialInMsg.subscribeTo(<CmdForceInertialMsg>) before simulation.");
    }

    if (!this->siteFrameStateInMsg.isLinked() && !this->siteAttInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR,
            "CmdForceInertialToForceAtSite: siteFrameStateInMsg and siteAttInMsg are not linked. At least one must be linked."
            "Call siteFrameStateInMsg.subscribeTo(<SCStatesMsg>) or siteAttInMsg.subscribeTo(<NavAttMsg>) before simulation.");
    }

}
