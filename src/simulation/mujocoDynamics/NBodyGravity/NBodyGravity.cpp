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

#include "architecture/utilities/rigidBodyKinematics.h"
#include "simulation/dynamics/_GeneralModuleFiles/gravityEffector.h"

namespace {
std::shared_ptr<GravityModel>
getGravityModel(const GravitySource& source)
{
    return source.gravBody ? source.gravBody->gravityModel : source.model;
}

bool
isCentral(const GravitySource& source)
{
    return source.gravBody ? source.gravBody->isCentralBody : source.isCentralBody;
}

bool
isStateLinked(GravitySource& source)
{
    return source.stateInMsg.isLinked()
           || (source.gravBody && source.gravBody->planetBodyInMsg.isLinked());
}

SpicePlanetStateMsgPayload
readState(GravitySource& source, BSKLogger& bskLogger)
{
    if (source.stateInMsg.isLinked())
    {
        return source.stateInMsg();
    }
    if (source.gravBody && source.gravBody->planetBodyInMsg.isLinked())
    {
        return source.gravBody->planetBodyInMsg();
    }
    bskLogger.bskError("Cannot read an unlinked NBodyGravity source ephemeris message.");
}
} // namespace

void
NBodyGravity::Reset(uint64_t CurrentSimNanos)
{
    std::string errorPrefix = "In NBodyGravity '" + ModelTag + "': ";
    for (auto&& [name, target] : targets)
    {
        if (!target.centerOfMassStateInMsg.isLinked())
        {
            bskLogger.bskError("%sTarget '%s' centerOfMassStateInMsg is not linked!", errorPrefix.c_str(), name.c_str());
        }

        if (!target.massPropertiesInMsg.isLinked())
        {
            bskLogger.bskError("%sTarget '%s' massPropertiesInMsg is not linked!", errorPrefix.c_str(), name.c_str());
        }
    }

    bool foundCentralSource = false;

    for (auto&& [name, source] : sources)
    {
        if (sources.size() > 1 && !isStateLinked(source))
        {
            bskLogger.bskError(
                "%sSource '%s' has no ephemeris input but there are multiple gravity sources!",
                errorPrefix.c_str(),
                name.c_str());
        }

        auto gravityModel = getGravityModel(source);
        if (!gravityModel)
        {
            bskLogger.bskError("%sSource '%s' has no gravity model!", errorPrefix.c_str(), name.c_str());
        }

        auto error = source.gravBody
            ? gravityModel->initializeParameters(*source.gravBody)
            : gravityModel->initializeParameters();
        if (error)
        {
            bskLogger.bskError(
                "%sWhile initializing source '%s' gravity model: %s",
                errorPrefix.c_str(),
                name.c_str(),
                error->c_str());
        }

        if (gravityModel->dependsOnOrientation() && !isStateLinked(source))
        {
            bskLogger.bskLog(
                BSK_WARNING,
                "%sSource '%s' uses an orientation-dependent gravity model, but no "
                "ephemeris input is connected. The body will be treated as non-rotating.",
                errorPrefix.c_str(),
                name.c_str());
        }

        if (isCentral(source))
        {
            if (foundCentralSource)
            {
                bskLogger.bskError("%sMore than one central body!", errorPrefix.c_str());
            }
            foundCentralSource = true;
        }
    }
}

void
NBodyGravity::UpdateState(uint64_t CurrentSimNanos)
{
    for (auto&& [_, target] : targets)
    {
        auto targetStatePayload = target.centerOfMassStateInMsg();
        Eigen::Matrix3d dcm_BN = std::invoke([&](){
            double dcm_BN[3][3];
            MRP2C(targetStatePayload.sigma_BN, dcm_BN);
            return cArray2EigenMatrix3d(*dcm_BN);
        });

        auto gravity_N = computeAccelerationOnTarget(target);
        auto gravity_B = dcm_BN * gravity_N;

        auto massPropPayload = target.massPropertiesInMsg();
        Eigen::Vector3d force_B = massPropPayload.massSC * gravity_B;

        ForceAtSiteMsgPayload forcePayload;
        std::copy_n(force_B.data(), 3, forcePayload.force_S);
        target.massFixedForceOutMsg.write(&forcePayload, moduleID, CurrentSimNanos);
    }
}

GravitySource&
NBodyGravity::addGravitySource(std::string name, std::shared_ptr<GravityModel> gravityModel, bool isCentralBody)
{
    auto[source, actuallyEmplaced] =
        this->sources.emplace(name, GravitySource{gravityModel, {}, isCentralBody, nullptr});

    if (!actuallyEmplaced)
    {
        bskLogger.bskError("Cannot use repeated source name: %s", name.c_str());
    }

    return source->second;
}

GravitySource&
NBodyGravity::addGravitySource(std::string name, std::shared_ptr<GravBodyData> gravBody)
{
    if (!gravBody)
    {
        bskLogger.bskError("Cannot add source '%s' from a null GravBodyData.", name.c_str());
    }

    auto[source, actuallyEmplaced] =
        this->sources.emplace(name, GravitySource{nullptr, {}, false, std::move(gravBody)});

    if (!actuallyEmplaced)
    {
        bskLogger.bskError("Cannot use repeated source name: %s", name.c_str());
    }

    return source->second;
}

GravityTarget&
NBodyGravity::addGravityTarget(std::string name)
{
    // Obscure addition strategy required so that we build in place the
    // GravityTarget. This avoids copying the `Message<...>` in GravityTarget,
    // which proved to cause weird bugs (multiple messages pointing to the same
    // address in memory for their payload).
    auto[target, actuallyEmplaced] = this->targets.emplace(
        std::piecewise_construct,
        std::forward_as_tuple(name),
        std::forward_as_tuple()
    );

    if (!actuallyEmplaced)
    {
        bskLogger.bskError("Cannot use repeated target name: %s", name.c_str());
    }

    return target->second;
}

GravityTarget&
NBodyGravity::addGravityTarget(std::string name, MJSite& site)
{
    auto& target = addGravityTarget(name);

    /* Tie the state outMsg of the site to the inMsg of the gravity target */
    target.centerOfMassStateInMsg.subscribeTo( &site.stateOutMsg );

    /* Create a new actuator on the site, then tie the force outMsg of
    the gravity target to the actuator force inMsg */
    std::string actuatorName = ModelTag + "_gravity_target_at_" + name;
    auto& actuator = site.getBody().getSpec().getScene().addForceActuator(actuatorName, site.getName());
    actuator.forceInMsg.subscribeTo( &target.massFixedForceOutMsg );
    return target;
}

GravityTarget&
NBodyGravity::addGravityTarget(std::string name, MJBody& body)
{
    auto& target = addGravityTarget(std::move(name), body.getCenterOfMass());
    target.massPropertiesInMsg.subscribeTo( &body.massPropertiesOutMsg );
    return target;
}

Eigen::Vector3d
NBodyGravity::computeAccelerationFromSource(GravitySource& source, Eigen::Vector3d r_J2000)
{
    // Orientation and position of the gravity source in J2000
    Eigen::Matrix3d dcm_sourceFixedJ2000 = Eigen::Matrix3d::Identity();
    Eigen::Vector3d r_sourceJ2000 = Eigen::Vector3d::Zero();
    if (isStateLinked(source))
    {
        auto spicePayload = readState(source, bskLogger);
        dcm_sourceFixedJ2000 = c2DArray2EigenMatrix3d(spicePayload.J20002Pfix);
        r_sourceJ2000 = cArray2EigenVector3d(spicePayload.PositionVector);
    }

    // Transform position in J2000 reference frame to source-fixed reference frame
    Eigen::Vector3d r_sourceFix = dcm_sourceFixedJ2000 * (r_J2000 - r_sourceJ2000);

    // Compute the gravity acceleration on the source-fixed reference frame
    Eigen::Vector3d grav_sourceFix = getGravityModel(source)->computeField(r_sourceFix);

    // Convert gravity to J2000 reference frame
    return dcm_sourceFixedJ2000.transpose() * grav_sourceFix;
}

Eigen::Vector3d
NBodyGravity::computeAccelerationOnTarget(GravityTarget& target)
{
    /* List of points and frames of interest in this function:

        - SpiceRef: Spice messages report body positions and orientations
            w.r.t this frame. Typically, this is the J2000 frame centered
            at the solar system barycenter, but might be different if
            `zeroBase` or `referenceBase` were updated in `SpiceInterface`
        - N: inertial reference frame of the simulation. If there is a
            central body, then the center of this reference frame is the
            center of the central body. Otherwise, the center coincides
            with J2000. The orientation always coincides with J2000.
        - target: a position corresponding to the target
    */

    auto targetStatePayload = target.centerOfMassStateInMsg();

    /* Position of the target w.r.t the inertial center of the simulation,
    given in the inertial reference frame of the simulation.
    This might be body-centered if this simulation has a central body. */
    Eigen::Vector3d r_targetN_N = cArray2EigenVector3d(targetStatePayload.r_BN_N);

    /* Position of the central body w.r.t the Spice reference frame.
    The following loop will update this IF there is a central body whose state
    inMsg is connected, otherwise assume central source is at center of integration. */
    Eigen::Vector3d r_centralSourceSpiceRef_N = Eigen::Vector3d::Zero();
    bool centralSourceExists = false;

    for (auto&& [_, source] : sources)
    {
        if (!isCentral(source)) continue;

        centralSourceExists = true;

        if (!isStateLinked(source)) break;

        auto spicePayload = readState(source, bskLogger);
        r_centralSourceSpiceRef_N = cArray2EigenVector3d(spicePayload.PositionVector);
    }

    /* Position of the target w.r.t the Spice center and reference frame.
    If there is a central body, and this central body does not correspond to
    the SpiceRef center, then this is the position of the target w.r.t the central
    body (r_targetN_N) + the position of the central w.r.t SpiceRef (r_centralSourceSpiceRef_N). */
    Eigen::Vector3d r_targetSpiceRef_N = r_targetN_N + r_centralSourceSpiceRef_N;

    // Gravity acceleration acting on the target in the inertial reference frame
    Eigen::Vector3d grav_N = Eigen::Vector3d::Zero();

    // Total acceleration is the sum of accelerations produced by each source
    // But also taking into account the relative acceleration of the central body
    for (auto&& [_, source] : sources)
    {
        grav_N += computeAccelerationFromSource(source, r_targetSpiceRef_N);

        // If there is a central body, and 'source' is not this one,
        // then we need to take into account the acceleration of the central
        // body due to the effect of this other source.
        if (centralSourceExists && !isCentral(source))
        {
            grav_N -= computeAccelerationFromSource(source, r_centralSourceSpiceRef_N);
        }
    }

    return grav_N;
}
