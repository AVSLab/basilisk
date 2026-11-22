/*
 ISC License

 Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#include <algorithm>
#include <functional>
#include <optional>

#include <Eigen/Geometry>

#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/macroDefinitions.h"
#include "architecture/utilities/stateExtrapolation.h"
#include "gravityEffector.h"

GravBodyData::GravBodyData()
    : gravityModel{ std::make_shared<PointMassGravityModel>() }
    , localPlanet{ std::invoke([]() {
        SpicePlanetStateMsgPayload payload{};
        m33SetIdentity(payload.J20002Pfix); // Initialize rotation matrix to identity
        return payload;
    }) }
{
}

void GravBodyData::initBody(int64_t moduleID [[maybe_unused]])
{
    std::optional<std::string> errorMessage;
    if (this->gravityModel) {
        this->gravityModel->bskLogger = &bskLogger;
        errorMessage = this->gravityModel->initializeParameters(*this);
    } else {
        errorMessage = "Gravity model is null";
    }

    if (errorMessage) {
        std::string fullMsg = "Error initializating body '" + this->planetName + "'. " + errorMessage.value();
        this->bskLogger.bskError("%s", fullMsg.c_str());
    }
}

double GravBodyData::ephemerisOffset(double stageTimeSeconds) const
{
    // Subtract whole seconds first to retain the fractional message epoch on long simulations.
    constexpr uint64_t nanosPerSecond = 1000000000; // [ns/s]
    return (stageTimeSeconds - static_cast<double>(this->timeWritten / nanosPerSecond)) -
           static_cast<double>(this->timeWritten % nanosPerSecond) * NANO2SEC;
}

double GravBodyData::ephemerisOffset(uint64_t timeNanos) const
{
    // Subtract integer epochs before conversion. A distant ephemeris still has a finite offset.
    if (timeNanos >= this->timeWritten) {
        return static_cast<double>(timeNanos - this->timeWritten) * NANO2SEC;
    }
    return -static_cast<double>(this->timeWritten - timeNanos) * NANO2SEC;
}

Eigen::Vector3d GravBodyData::positionAtOffset(double dt) const
{
    return Eigen::Vector3d(this->localPlanet.PositionVector) + Eigen::Vector3d(this->localPlanet.VelocityVector) * dt;
}

void GravBodyData::computePlanetOrientation(double dt,
                                            Eigen::Matrix3d& dcmAdvanced_NPfix,
                                            Eigen::Matrix3d& dcmAdvanced_NPfix_dot) const
{
    // The message holds [PN], the rotation from the inertial to the planet-fixed frame. The orientation is advanced
    // in its transpose [NP].
    using RowMajorMatrix3d = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>;
    Eigen::Matrix3d dcm_NPfix = Eigen::Map<const RowMajorMatrix3d>(&this->localPlanet.J20002Pfix[0][0]).transpose();
    if (dcm_NPfix.isZero()) { // Sanity check for connected messages that do not initialize J20002Pfix
        dcm_NPfix = Eigen::Matrix3d::Identity();
    }
    const Eigen::Matrix3d dcm_NPfix_dot =
        Eigen::Map<const RowMajorMatrix3d>(&this->localPlanet.J20002Pfix_dot[0][0]).transpose();

    // Advance the planet orientation by the time since the message was written as a rotation
    // about the planet angular velocity, see advanceDcm().
    const PlanetSpin spin = planetSpin(dcm_NPfix, dcm_NPfix_dot);
    dcmAdvanced_NPfix = advanceDcm(dcm_NPfix, spin, dt);
    dcmAdvanced_NPfix_dot = advanceDcmDot(spin, dcmAdvanced_NPfix, dcm_NPfix_dot);
}

Eigen::Vector3d GravBodyData::computeGravityInertial(Eigen::Vector3d r_I, uint64_t simTimeNanos)
{
    return this->gravityAtOffset(r_I, this->ephemerisOffset(simTimeNanos), this->standaloneProperties);
}

Eigen::Vector3d GravBodyData::gravityAtOffset(const Eigen::Vector3d& r_I, double dt, const PropertyHandles& properties)
{
    Eigen::Matrix3d dcmAdvanced_NPfix, dcmAdvanced_NPfix_dot;
    this->computePlanetOrientation(dt, dcmAdvanced_NPfix, dcmAdvanced_NPfix_dot);

    // store the current planet orientation and rates as [PN] and [PN_dot], as documented for the properties
    if (properties.J20002Pfix) {
        *properties.J20002Pfix = dcmAdvanced_NPfix.transpose();
        *properties.J20002Pfix_dot = dcmAdvanced_NPfix_dot.transpose();
    }

    // Compute position in the body-fixed reference frame and compute the gravity
    Eigen::Vector3d r_Pfix = dcmAdvanced_NPfix.transpose() * r_I;
    Eigen::Vector3d grav_Pfix = this->gravityModel->computeField(r_Pfix);

    return dcmAdvanced_NPfix * grav_Pfix;
}

double GravBodyData::computePotentialEnergyInertial(const Eigen::Vector3d& r_I, uint64_t simTimeNanos) const
{
    return this->potentialAtOffset(r_I, this->ephemerisOffset(simTimeNanos));
}

double GravBodyData::potentialAtOffset(const Eigen::Vector3d& r_I, double dt) const
{
    Eigen::Matrix3d dcm_NPfix, dcm_NPfix_dot;
    this->computePlanetOrientation(dt, dcm_NPfix, dcm_NPfix_dot);
    return this->gravityModel->computePotentialEnergy(dcm_NPfix.transpose() * r_I);
}

void GravBodyData::loadEphemeris()
{
    if (this->planetBodyInMsg.isLinked()) {
        if (!this->planetBodyInMsg.isWritten()) {
            this->ephemerisReady = false;
            return;
        }
        this->localPlanet = this->planetBodyInMsg();
        this->timeWritten = this->planetBodyInMsg.timeWritten();
    } else {
        // use default zero planet state information
        this->localPlanet = this->planetBodyInMsg.zeroMsgPayload;
        this->timeWritten = 0;
    }
    this->ephemerisReady = true;
}

void GravBodyData::registerProperties(DynParamManager& statesIn)
{
    this->standaloneProperties = this->createProperties(statesIn);
}

GravBodyData::PropertyHandles GravBodyData::createProperties(DynParamManager& statesIn)
{
    if (this->planetName == "") {
        auto errorMessage = "You must specify a planetary body name in GravBodyData";
        this->bskLogger.bskError("%s", errorMessage);
        throw std::invalid_argument(errorMessage);
    }

    PropertyHandles properties;
    Eigen::Vector3d stateInit = Eigen::Vector3d::Zero();
    properties.r_PN_N = statesIn.createProperty(this->planetName + ".r_PN_N", stateInit);
    properties.v_PN_N = statesIn.createProperty(this->planetName + ".v_PN_N", stateInit);

    Eigen::MatrixXd muInit = Eigen::MatrixXd::Zero(1, 1);
    properties.muPlanet = statesIn.createProperty(this->planetName + ".mu", muInit);

    properties.J20002Pfix =
        statesIn.createProperty(this->planetName + ".J20002Pfix", c2DArray2EigenMatrix3d(this->localPlanet.J20002Pfix));
    properties.J20002Pfix_dot = statesIn.createProperty(this->planetName + ".J20002Pfix_dot",
                                                        c2DArray2EigenMatrix3d(this->localPlanet.J20002Pfix_dot));
    return properties;
}

void GravityEffector::Reset(uint64_t currentSimNanos [[maybe_unused]])
{
    // Initializes the bodies
    for (auto&& body : this->gravBodies) {
        body->initBody(this->moduleID);

        // Warn if a body's gravity field depends on its orientation (e.g. it has
        // tesseral/sectoral spherical-harmonic terms, or a polyhedral shape) but
        // no planet-orientation message is connected. Without it the body is
        // treated as non-rotating, which makes the orientation-dependent terms
        // produce spurious orbital drift (see issue #1352). Note: a message that
        // is linked but only supplies position (leaving J20002Pfix zero) triggers
        // the same identity fallback at runtime; at reset time only message
        // linkage can be checked.
        if (body->gravityModel && body->gravityModel->dependsOnOrientation() && !body->planetBodyInMsg.isLinked()) {
            this->bskLogger.bskLog(BSK_WARNING,
                                   "Gravity body '%s' uses an orientation-dependent gravity model "
                                   "(e.g. spherical-harmonic tesseral/sectoral terms of order >= 1, "
                                   "or a polyhedral shape model), but no planet-orientation message "
                                   "is connected to planetBodyInMsg. The body will be treated as "
                                   "non-rotating, which produces spurious orbital drift. Connect a "
                                   "planet ephemeris/orientation message, e.g. via "
                                   "gravBodyFactory.createSpiceInterface().",
                                   body->planetName.c_str());
        }
    }
    this->loadEphemerides(true);
}

void GravityEffector::UpdateState(uint64_t currentSimNanos)
{
    this->loadEphemerides(false);
    this->writeOutputMessages(currentSimNanos);
}

bool GravityEffector::ephemeridesReady() const
{
    return this->bodiesResolved && std::all_of(this->gravBodies.begin(), this->gravBodies.end(), [](const auto& body) {
               return body->ephemerisReady;
           });
}

void GravityEffector::loadEphemerides(bool atReset)
{
    this->bodiesResolved = false;
    this->originReady = true;
    // Updates the central body
    this->centralBody.reset();
    for (auto&& body : this->gravBodies) {
        if (atReset && !body->planetBodyInMsg.isLinked()) {
            body->ephemerisReady = true; // Keep a manually configured initial planet state.
        } else {
            body->loadEphemeris();
        }

        if (!body->isCentralBody)
            continue;

        if (this->centralBody) // A centralBody was already set
        {
            auto errorMessage = "Specified two central bodies at the same time";
            this->bskLogger.bskError("%s", errorMessage);
            throw std::invalid_argument(errorMessage);
        } else {
            this->centralBody = body;
            this->originReady = body->ephemerisReady;
        }
    }
    this->bodiesResolved = true;
}

void GravityEffector::writeOutputMessages(uint64_t currentSimNanos)
{
    if (this->centralBodyOutMsg.isLinked() && this->centralBody) {
        auto& input = this->centralBody->planetBodyInMsg;
        if (input.isLinked()) {
            if (input.isWritten()) {
                auto planet = input();
                this->centralBodyOutMsg.write(&planet, this->moduleID, input.timeWritten());
            }
        } else {
            this->centralBodyOutMsg.write(&this->centralBody->localPlanet, this->moduleID, currentSimNanos);
        }
    }
}

void GravityEffector::registerProperties(DynParamManager& statesIn)
{
    static const Eigen::Vector3d zeroVector3d = Eigen::Vector3d::Zero();

    this->inertialFrame = &statesIn.inertialFrame;
    this->gravProperty = statesIn.createProperty(this->vehicleGravityPropName, zeroVector3d);
    this->inertialPositionProperty = statesIn.createProperty(this->inertialPositionPropName, zeroVector3d);
    this->inertialVelocityProperty = statesIn.createProperty(this->inertialVelocityPropName, zeroVector3d);

    // Spacecraft can share a body's ephemeris, but each writes planet properties in its own state manager.
    this->bodyProperties.clear();
    for (const auto& body : this->gravBodies) {
        this->bodyProperties.emplace(body, body->createProperties(statesIn));
    }
}

void GravityEffector::linkInStates(DynParamManager& statesIn)
{
    this->timeCorr = statesIn.getPropertyReference(this->systemTimeCorrPropName);
}

double GravityEffector::currentEphemerisOffset(const GravBodyData& body) const
{
    const auto& exactEpoch = this->inertialFrame->exactStageTimeNanos;
    return exactEpoch ? body.ephemerisOffset(*exactEpoch) : body.ephemerisOffset((*this->timeCorr)(1));
}

void GravityEffector::computeGravityField(Eigen::Vector3d r_cF_N, Eigen::Vector3d rDot_cF_N [[maybe_unused]])
{
    Eigen::Vector3d r_cN_N;                           // position of s/c CoM wrt N
    Eigen::Vector3d r_CN_N = Eigen::Vector3d::Zero(); // inertial position of central body if there is one. Big C is
                                                      // central body. Little c is CoM of s/c

    if (this->centralBody) // If there is a central body
    {
        r_CN_N = this->centralBody->positionAtOffset(this->currentEphemerisOffset(*this->centralBody));
        r_cN_N = r_cF_N + r_CN_N; // shift s/c to be wrt inertial frame origin
                                  // if it isn't already
    } else {
        r_cN_N = r_cF_N;
    }

    // acceleration of CoM of s/c wrt Frame in which it is stored/integrated in spacecraft
    Eigen::Vector3d rDotDot_cF_N = Eigen::Vector3d::Zero();

    for (auto&& body : this->gravBodies) {
        // position of Planet being queried wrt N
        const auto& properties = this->bodyProperties.at(body);
        const double dt = this->currentEphemerisOffset(*body); // [s]
        Eigen::Vector3d r_PN_N = body->positionAtOffset(dt);
        Eigen::Vector3d r_cP_N = r_cN_N - r_PN_N; // position of s/c CoM wrt planet in N

        if (this->centralBody && !body->isCentralBody) // If there is a central body, and its not
                                                       // 'body'
        {
            // Subtract accel of central body due to other bodies to get
            // RELATIVE accel of s/c. See Vallado on "Three-body and n-body
            // Equations"
            rDotDot_cF_N += body->gravityAtOffset(r_PN_N - r_CN_N, dt, properties);
        }
        // acceleration of c wrt N in N, due to P
        rDotDot_cF_N += body->gravityAtOffset(r_cP_N, dt, properties);

        // store planet states in the state engine parameters
        *properties.r_PN_N = r_PN_N;
        *properties.v_PN_N = cArray2EigenVector3d(body->localPlanet.VelocityVector);
        (*properties.muPlanet)(0, 0) = body->mu;
    }

    *this->gravProperty = rDotDot_cF_N;
}

void GravityEffector::updateInertialPosAndVel(Eigen::Vector3d r_BF_N, Eigen::Vector3d rDot_BF_N)
{
    const double dt = this->centralBody ? this->currentEphemerisOffset(*this->centralBody) : 0.0; // [s]
    this->updateInertialPosAndVelAtOffset(r_BF_N, rDot_BF_N, dt);
}

void GravityEffector::updateInertialPosAndVel(Eigen::Vector3d r_BF_N,
                                              Eigen::Vector3d rDot_BF_N,
                                              uint64_t outputTimeNanos)
{
    const double dt = this->centralBody ? this->centralBody->ephemerisOffset(outputTimeNanos) : 0.0; // [s]
    this->updateInertialPosAndVelAtOffset(r_BF_N, rDot_BF_N, dt);
}

void GravityEffector::updateInertialPosAndVelAtOffset(const Eigen::Vector3d& r_BF_N,
                                                      const Eigen::Vector3d& rDot_BF_N,
                                                      double dt)
{
    if (this->centralBody) {
        this->inertialFrame->originPosition = this->centralBody->positionAtOffset(dt);
        this->inertialFrame->originVelocity = cArray2EigenVector3d(this->centralBody->localPlanet.VelocityVector);
    } else {
        this->inertialFrame->originPosition.setZero();
        this->inertialFrame->originVelocity.setZero();
    }
    *this->inertialPositionProperty = this->inertialFrame->originPosition + r_BF_N;
    *this->inertialVelocityProperty = this->inertialFrame->originVelocity + rDot_BF_N;
}

void GravityEffector::updateEnergyContributions(Eigen::Vector3d r_cF_N,
                                                double& orbPotEnergyContr,
                                                uint64_t simTimeNanos)
{
    if (!this->ephemeridesReady()) {
        this->bskLogger.bskError(
            "GravityEffector: orbital energy requires a sample from every linked planet ephemeris.");
    }
    this->accumulatePotential(r_cF_N, orbPotEnergyContr, simTimeNanos);
}

void GravityEffector::updateEnergyContributionsAtTime(Eigen::Vector3d r_cF_N, double& orbPotEnergyContr, double time)
{
    this->accumulatePotential(r_cF_N, orbPotEnergyContr, time);
}

template<typename Time>
void GravityEffector::accumulatePotential(Eigen::Vector3d r_cF_N, double& orbPotEnergyContr, Time time)
{
    Eigen::Vector3d r_CN_N = Eigen::Vector3d::Zero(); // C is central body. position of C wrt N in N
    Eigen::Vector3d r_cN_N;                           // position c wrt N in N

    if (this->centralBody) { // Evaluates true if there is a central body, false
                             // otherwise
        r_CN_N = this->centralBody->positionAtOffset(this->centralBody->ephemerisOffset(time));
        r_cN_N = r_cF_N + r_CN_N; // shift s/c to be wrt inertial frame origin
                                  // if it isn't already
    } else {
        r_cN_N = r_cF_N;
    }

    for (auto&& body : this->gravBodies) {
        // P is planet being queried. position of planet wrt N in N
        const double dt = body->ephemerisOffset(time); // [s]
        Eigen::Vector3d r_PN_N = body->positionAtOffset(dt);
        // c is s/c CoM. position of c wrt P in N
        Eigen::Vector3d r_cP_N = r_cN_N - r_PN_N;

        if (this->centralBody && !body->isCentralBody) // If there is a central body and it's not
                                                       // 'body'
        {
            // potential of central body w/in current planet field. leads to
            // relative potential energy solution
            orbPotEnergyContr += body->potentialAtOffset(r_PN_N - r_CN_N, dt);
        }
        orbPotEnergyContr += body->potentialAtOffset(r_cP_N, dt);
    }
}

void GravityEffector::setGravBodies(std::vector<std::shared_ptr<GravBodyData>> gravBodies)
{
    this->gravBodies = std::move(gravBodies);
}

void GravityEffector::addGravBody(std::shared_ptr<GravBodyData> gravBody)
{
    this->gravBodies.push_back(gravBody);
}
