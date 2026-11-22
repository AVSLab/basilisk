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

#include "NBodyGravity.h"

#include <algorithm>
#include <optional>
#include <vector>

#include "architecture/utilities/avsEigenSupport.h"
#include "architecture/utilities/rigidBodyKinematics.h"
#include "architecture/utilities/stateExtrapolation.h"
#include "simulation/dynamics/_GeneralModuleFiles/gravityEffector.h"

namespace {
std::shared_ptr<GravityModel> getGravityModel(const GravitySource& source)
{
    return source.gravBody ? source.gravBody->gravityModel : source.model;
}

bool isCentral(const GravitySource& source)
{
    return source.gravBody ? source.gravBody->isCentralBody : source.isCentralBody;
}

ReadFunctor<SpicePlanetStateMsgPayload>* stateReader(GravitySource& source)
{
    if (source.stateInMsg.isLinked()) {
        return &source.stateInMsg;
    }
    return source.gravBody && source.gravBody->planetBodyInMsg.isLinked() ? &source.gravBody->planetBodyInMsg : nullptr;
}

struct SourcePose
{
    std::shared_ptr<GravityModel> model;
    bool central;
    Eigen::Vector3d position = Eigen::Vector3d::Zero(); // [m] in the ephemeris frame
    Eigen::Matrix3d dcm_PN = Eigen::Matrix3d::Identity();

    Eigen::Vector3d acceleration(const Eigen::Vector3d& displacement_N) const
    {
        return dcm_PN.transpose() * model->computeField(dcm_PN * displacement_N);
    }
};

SourcePose sampleSource(GravitySource& source, std::optional<uint64_t> taskTick)
{
    SourcePose pose{ getGravityModel(source), isCentral(source) };
    auto* reader = stateReader(source);
    if (!reader) {
        return pose;
    }
    auto sample = (*reader)();
    const double dt = taskTick ? MJScene::getDynamicsTimeOffset(*taskTick, reader->timeWritten()) : 0.0; // [s]
    pose.position = Eigen::Vector3d(sample.PositionVector);
    pose.dcm_PN = c2DArray2EigenMatrix3d(sample.J20002Pfix);
    if (pose.dcm_PN.isZero()) { // Position-only ephemerides are inertially aligned.
        pose.dcm_PN.setIdentity();
    }
    if (dt != 0.0) {
        pose.position += dt * Eigen::Vector3d(sample.VelocityVector);
        const Eigen::Matrix3d dcm_NP = pose.dcm_PN.transpose();
        const auto spin = planetSpin(dcm_NP, c2DArray2EigenMatrix3d(sample.J20002Pfix_dot).transpose());
        pose.dcm_PN = advanceDcm(dcm_NP, spin, dt).transpose();
    }
    return pose;
}

std::vector<SourcePose> sampleSources(std::map<std::string, GravitySource>& sources, std::optional<uint64_t> taskTick)
{
    std::vector<SourcePose> poses;
    poses.reserve(sources.size());
    for (auto& [_, source] : sources) {
        poses.push_back(sampleSource(source, taskTick));
    }
    return poses;
}

Eigen::Vector3d targetAcceleration(const std::vector<SourcePose>& sources, const SCStatesMsgPayload& target)
{
    const auto central =
        std::find_if(sources.begin(), sources.end(), [](const SourcePose& source) { return source.central; });
    const Eigen::Vector3d origin = central == sources.end() ? Eigen::Vector3d::Zero() : central->position;
    const Eigen::Vector3d targetPosition(target.r_BN_N);
    Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
    for (const auto& source : sources) {
        const Eigen::Vector3d sourceToOrigin = origin - source.position;
        acceleration += source.acceleration(sourceToOrigin + targetPosition);
        if (central != sources.end() && !source.central) {
            // Subtract this source's acceleration on the moving central body at the same stage.
            acceleration -= source.acceleration(sourceToOrigin);
        }
    }
    return acceleration;
}
} // namespace

void NBodyGravity::Reset(uint64_t CurrentSimNanos [[maybe_unused]])
{
    std::string errorPrefix = "In NBodyGravity '" + ModelTag + "': ";
    for (auto&& [name, target] : targets) {
        if (!target.centerOfMassStateInMsg.isLinked()) {
            bskLogger.bskError(
                "%sTarget '%s' centerOfMassStateInMsg is not linked!", errorPrefix.c_str(), name.c_str());
        }

        if (!target.massPropertiesInMsg.isLinked()) {
            bskLogger.bskError("%sTarget '%s' massPropertiesInMsg is not linked!", errorPrefix.c_str(), name.c_str());
        }
    }

    bool foundCentralSource = false;

    for (auto&& [name, source] : sources) {
        if (sources.size() > 1 && !stateReader(source)) {
            bskLogger.bskError("%sSource '%s' has no ephemeris input but there are multiple gravity sources!",
                               errorPrefix.c_str(),
                               name.c_str());
        }

        auto gravityModel = getGravityModel(source);
        if (!gravityModel) {
            bskLogger.bskError("%sSource '%s' has no gravity model!", errorPrefix.c_str(), name.c_str());
        }

        auto error = source.gravBody ? gravityModel->initializeParameters(*source.gravBody)
                                     : gravityModel->initializeParameters();
        if (error) {
            bskLogger.bskError("%sWhile initializing source '%s' gravity model: %s",
                               errorPrefix.c_str(),
                               name.c_str(),
                               error->c_str());
        }

        if (gravityModel->dependsOnOrientation() && !stateReader(source)) {
            bskLogger.bskLog(BSK_WARNING,
                             "%sSource '%s' uses an orientation-dependent gravity model, but no "
                             "ephemeris input is connected. The body will be treated as non-rotating.",
                             errorPrefix.c_str(),
                             name.c_str());
        }

        if (isCentral(source)) {
            if (foundCentralSource) {
                bskLogger.bskError("%sMore than one central body!", errorPrefix.c_str());
            }
            foundCentralSource = true;
        }
    }
}

void NBodyGravity::UpdateState(uint64_t CurrentSimNanos)
{
    const auto poses = sampleSources(sources, CurrentSimNanos);
    for (auto&& [_, target] : targets) {
        auto targetStatePayload = target.centerOfMassStateInMsg();
        Eigen::Matrix3d dcm_BN = std::invoke([&]() {
            double dcm_BN[3][3];
            MRP2C(targetStatePayload.sigma_BN, dcm_BN);
            return cArray2EigenMatrix3d(*dcm_BN);
        });

        auto gravity_N = targetAcceleration(poses, targetStatePayload);
        auto gravity_B = dcm_BN * gravity_N;

        auto massPropPayload = target.massPropertiesInMsg();
        Eigen::Vector3d force_B = massPropPayload.massSC * gravity_B;

        ForceAtSiteMsgPayload forcePayload;
        std::copy_n(force_B.data(), 3, forcePayload.force_S);
        target.massFixedForceOutMsg.write(&forcePayload, moduleID, CurrentSimNanos);
    }
}

GravitySource& NBodyGravity::addGravitySource(std::string name,
                                              std::shared_ptr<GravityModel> gravityModel,
                                              bool isCentralBody)
{
    auto [source, actuallyEmplaced] =
        this->sources.emplace(name, GravitySource{ gravityModel, {}, isCentralBody, nullptr });

    if (!actuallyEmplaced) {
        bskLogger.bskError("Cannot use repeated source name: %s", name.c_str());
    }

    return source->second;
}

GravitySource& NBodyGravity::addGravitySource(std::string name, std::shared_ptr<GravBodyData> gravBody)
{
    if (!gravBody) {
        bskLogger.bskError("Cannot add source '%s' from a null GravBodyData.", name.c_str());
    }

    auto [source, actuallyEmplaced] =
        this->sources.emplace(name, GravitySource{ nullptr, {}, false, std::move(gravBody) });

    if (!actuallyEmplaced) {
        bskLogger.bskError("Cannot use repeated source name: %s", name.c_str());
    }

    return source->second;
}

GravityTarget& NBodyGravity::addGravityTarget(std::string name)
{
    // Obscure addition strategy required so that we build in place the
    // GravityTarget. This avoids copying the `Message<...>` in GravityTarget,
    // which proved to cause weird bugs (multiple messages pointing to the same
    // address in memory for their payload).
    auto [target, actuallyEmplaced] =
        this->targets.emplace(std::piecewise_construct, std::forward_as_tuple(name), std::forward_as_tuple());

    if (!actuallyEmplaced) {
        bskLogger.bskError("Cannot use repeated target name: %s", name.c_str());
    }

    return target->second;
}

GravityTarget& NBodyGravity::addGravityTarget(std::string name, MJSite& site)
{
    auto& target = addGravityTarget(name);

    /* Tie the state outMsg of the site to the inMsg of the gravity target */
    target.centerOfMassStateInMsg.subscribeTo(&site.stateOutMsg);

    /* Create a new actuator on the site, then tie the force outMsg of
    the gravity target to the actuator force inMsg */
    std::string actuatorName = ModelTag + "_gravity_target_at_" + name;
    auto& actuator = site.getBody().getSpec().getScene().addForceActuator(actuatorName, site.getName());
    actuator.forceInMsg.subscribeTo(&target.massFixedForceOutMsg);
    return target;
}

GravityTarget& NBodyGravity::addGravityTarget(std::string name, MJBody& body)
{
    auto& target = addGravityTarget(std::move(name), body.getCenterOfMass());
    target.massPropertiesInMsg.subscribeTo(&body.massPropertiesOutMsg);
    return target;
}

Eigen::Vector3d NBodyGravity::computeAccelerationFromSource(GravitySource& source, Eigen::Vector3d r_J2000)
{
    const auto pose = sampleSource(source, std::nullopt);
    return pose.acceleration(r_J2000 - pose.position);
}

Eigen::Vector3d NBodyGravity::computeAccelerationOnTarget(GravityTarget& target)
{
    return targetAcceleration(sampleSources(sources, std::nullopt), target.centerOfMassStateInMsg());
}
