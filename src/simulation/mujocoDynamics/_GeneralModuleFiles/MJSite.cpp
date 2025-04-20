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

#include "MJSite.h"
#include "MJBody.h"
#include "MJScene.h"
#include "MJSpec.h"

#include <Eigen/Geometry>

#include <sstream>

MJSite::MJSite(mjsSite* site, MJBody& body) : MJObject(site), body(body) {}

void MJSite::setPositionRelativeToBody(const Eigen::Vector3d& position)
{
    // Copy new position in the mjSpec
    std::copy_n(position.data(), 3, mjsObject->pos);

    // If id exists, then this site has already been configured with
    // an mjModel. Thus, update the position in said mjModel
    if (this->id.has_value())
    {
        auto model = this->body.getSpec().getMujocoModel();
        std::copy_n(position.data(), 3, model->site_pos + 3 * this->getId());

        // However, this process will make the kinematics stale
        this->body.getSpec().getScene().markKinematicsAsStale();
    }
}

void MJSite::setAttitudeRelativeToBody(const Eigen::MRPd& attitude)
{
    auto mat = attitude.toRotationMatrix();
    auto quat = Eigen::Quaterniond(mat);
    auto quatVec = Eigen::Vector4d{quat.w(), quat.x(), quat.y(), quat.z()};

    // Copy new quat in the mjSpec
    std::copy_n(quatVec.data(), 4, mjsObject->quat);

    // If id exists, then this site has already been configured with
    // an mjModel. Thus, update the quat in said mjModel
    if (this->id.has_value())
    {
        auto model = this->body.getSpec().getMujocoModel();
        std::copy_n(quatVec.data(), 4, model->site_quat + 4 * this->getId());

        // However, this process will make the kinematics stale
        this->body.getSpec().getScene().markKinematicsAsStale();
    }
}

void MJSite::writeFwdKinematicsMessage(mjModel* model, mjData* data, uint64_t CurrentSimNanos)
{
    SCStatesMsgPayload payload;

    std::copy_n(data->site_xpos + 3 * this->getId(), 3, payload.r_BN_N);
    Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> rot{data->site_xmat +
                                                                 9 * this->getId()};
    Eigen::Map<Eigen::MRPd> mrpd{payload.sigma_BN};
    mrpd = rot;

    double res[6];
    mj_objectVelocity(model, data, mjOBJ_SITE, static_cast<int>(this->getId()), res, 0);

    // TODO: Double check this is right
    std::copy_n(res, 3, payload.omega_BN_B);
    std::copy_n(res + 3, 3, payload.v_BN_N);

    this->stateOutMsg.write(&payload, this->body.getSpec().getScene().moduleID, CurrentSimNanos);
}
