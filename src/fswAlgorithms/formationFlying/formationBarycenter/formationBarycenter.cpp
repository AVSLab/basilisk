/*
 ISC License

 Copyright (c) 2021, Autonomous Vehicle Systems Lab, University of Colorado Boulder

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

#include "fswAlgorithms/formationFlying/formationBarycenter/formationBarycenter.h"
#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/orbitalMotion.h"
#include <cmath>
#include <cstddef>
#include <limits>
#include <math.h>

namespace {
/**
 * @brief Construct the orthonormal equinoctial plane axes for one common representation.
 * @param tiltSine Dimensionless inclination coordinate, tan(i/2)^factor sin(Omega).
 * @param tiltCosine Dimensionless inclination coordinate, tan(i/2)^factor cos(Omega).
 * @param factor Common retrograde factor, +1 or -1.
 * @param firstAxis_N First dimensionless plane axis in the inertial frame.
 * @param secondAxis_N Second dimensionless plane axis in the inertial frame.
 */
void equinoctialPlane(double tiltSine, double tiltCosine, double factor, double firstAxis_N[3], double secondAxis_N[3])
{
    const double denominator = 1.0 + tiltSine * tiltSine + tiltCosine * tiltCosine;
    firstAxis_N[0] = (1.0 - tiltSine * tiltSine + tiltCosine * tiltCosine) / denominator;
    firstAxis_N[1] = 2.0 * tiltSine * tiltCosine / denominator;
    firstAxis_N[2] = -2.0 * factor * tiltSine / denominator;
    secondAxis_N[0] = 2.0 * factor * tiltSine * tiltCosine / denominator;
    secondAxis_N[1] = factor * (1.0 + tiltSine * tiltSine - tiltCosine * tiltCosine) / denominator;
    secondAxis_N[2] = 2.0 * tiltCosine / denominator;
}
}

/*! This is the constructor for the module class.  It sets default variable
    values and initializes the various parts of the model */
FormationBarycenter::FormationBarycenter()
{
    this->useOrbitalElements = false;
    this->orbitalElementType = CLASSICAL;
    this->mu = 0;
}

/*! Module Destructor */
FormationBarycenter::~FormationBarycenter() {}

/*! This method self initializes the C-wrapped output message.
 */
void FormationBarycenter::SelfInit()
{
    NavTransMsg_C_init(&this->transOutMsgC);
}

/*! This method is used to reset the module and checks that required input messages are connected.
 */
void FormationBarycenter::Reset(uint64_t CurrentSimNanos [[maybe_unused]])
{
    // check that required input messages are connected
    if (this->scNavInMsgs.size() == 0 || this->scPayloadInMsgs.size() == 0) {
        bskLogger.bskError(
            "FormationBarycenter module must have at least one spacecraft added through `addSpacecraftToModel`");
    }

    // check if the gravitational parameter is set if using orbital elements averaging
    if (this->mu == 0 && this->useOrbitalElements) {
        bskLogger.bskError(
            "FormationBarycenter module requires defining a gravitational parameter if using orbital elements.");
    }
    if (this->useOrbitalElements && this->orbitalElementType != CLASSICAL) {
        this->validateEquinoctialConfiguration();
    }
}

/*! Adds a scNav and scPayload messages name to the vector of names to be subscribed to.
 */
void FormationBarycenter::addSpacecraftToModel(Message<NavTransMsgPayload>* tmpScNavMsg,
                                               Message<VehicleConfigMsgPayload>* tmpScPayloadMsg)
{
    this->scNavInMsgs.push_back(tmpScNavMsg->addSubscriber());
    this->scPayloadInMsgs.push_back(tmpScPayloadMsg->addSubscriber());
}

/*! Reads the input messages
 */
void FormationBarycenter::ReadInputMessages()
{

    NavTransMsgPayload scNavMsg;
    VehicleConfigMsgPayload scPayloadMsg;

    // clear out the vector of spacecraft navigation and mass messages.  This is created freshly below.
    this->scNavBuffer.clear();
    this->scPayloadBuffer.clear();

    // read in the spacecraft state messages
    for (long unsigned int c = 0; c < this->scNavInMsgs.size(); c++) {
        scNavMsg = this->scNavInMsgs.at(c)();
        scPayloadMsg = this->scPayloadInMsgs.at(c)();
        this->scNavBuffer.push_back(scNavMsg);
        this->scPayloadBuffer.push_back(scPayloadMsg);
    }
}

/*! Does the barycenter calculations
 */
void FormationBarycenter::computeBaricenter()
{
    // create temporarary variables
    double barycenter[]{ 0, 0, 0 };
    double barycenterVelocity[]{ 0, 0, 0 };
    double totalMass{ 0 };

    // check which averaging to use
    if (!this->useOrbitalElements) {
        // compute the cartesian barycenter
        for (long unsigned int c = 0; c < this->scNavInMsgs.size(); c++) {
            for (int n = 0; n < 3; n++) {
                barycenter[n] += this->scPayloadBuffer.at(c).massSC * this->scNavBuffer.at(c).r_BN_N[n];
                barycenterVelocity[n] += this->scPayloadBuffer.at(c).massSC * this->scNavBuffer.at(c).v_BN_N[n];
            }
            totalMass += this->scPayloadBuffer.at(c).massSC;
        }

        for (int n = 0; n < 3; n++) {
            barycenter[n] /= totalMass;
            barycenterVelocity[n] /= totalMass;
        }
    } else if (this->orbitalElementType != CLASSICAL) {
        this->computeEquinoctialBarycenter(barycenter, barycenterVelocity);
    } else {
        ClassicElements orbitElements = {}; // zero the orbit elements first
        ClassicElements tempElements;
        double OmegaSineSum = 0;
        double OmegaCosineSum = 0;
        double omegaSineSum = 0;
        double omegaCosineSum = 0;
        double fSineSum = 0;
        double fCosineSum = 0;

        // compute the orbital elements barycenter
        for (long unsigned int c = 0; c < this->scNavInMsgs.size(); c++) {
            // convert the position and velocity vectors into orbital elements
            rv2elem(this->mu, this->scNavBuffer.at(c).r_BN_N, this->scNavBuffer.at(c).v_BN_N, &tempElements);

            orbitElements.a += this->scPayloadBuffer.at(c).massSC * tempElements.a;
            orbitElements.e += this->scPayloadBuffer.at(c).massSC * tempElements.e;
            orbitElements.i += this->scPayloadBuffer.at(c).massSC * tempElements.i;

            OmegaSineSum += this->scPayloadBuffer.at(c).massSC * sin(tempElements.Omega);
            OmegaCosineSum += this->scPayloadBuffer.at(c).massSC * cos(tempElements.Omega);
            omegaSineSum += this->scPayloadBuffer.at(c).massSC * sin(tempElements.omega);
            omegaCosineSum += this->scPayloadBuffer.at(c).massSC * cos(tempElements.omega);
            fSineSum += this->scPayloadBuffer.at(c).massSC * sin(tempElements.f);
            fCosineSum += this->scPayloadBuffer.at(c).massSC * cos(tempElements.f);

            totalMass += this->scPayloadBuffer.at(c).massSC;
        }

        orbitElements.a /= totalMass;
        orbitElements.e /= totalMass;
        orbitElements.i /= totalMass;
        orbitElements.Omega = atan2(OmegaSineSum, OmegaCosineSum);
        orbitElements.omega = atan2(omegaSineSum, omegaCosineSum);
        orbitElements.f = atan2(fSineSum, fCosineSum);

        // convert orbital elements into position and velocity vectors
        elem2rv(this->mu, &orbitElements, barycenter, barycenterVelocity);
    }

    // save the information to the output buffer
    for (int n = 0; n < 3; n++) {
        this->transOutBuffer.r_BN_N[n] = barycenter[n];
        this->transOutBuffer.v_BN_N[n] = barycenterVelocity[n];
    }
}

void FormationBarycenter::validateEquinoctialConfiguration()
{
    if (this->orbitalElementType != EQUINOCTIAL && this->orbitalElementType != RETROGRADE_EQUINOCTIAL) {
        this->bskLogger.bskError("FormationBarycenter orbitalElementType is not a supported choice.");
    }
    if (!std::isfinite(this->mu) || this->mu <= 0.0) {
        this->bskLogger.bskError(
            "FormationBarycenter equinoctial averaging requires a finite positive gravitational parameter.");
    }
}

void FormationBarycenter::computeEquinoctialBarycenter(double position_N[3], double velocity_N[3])
{
    this->validateEquinoctialConfiguration();
    const double factor = this->orbitalElementType == EQUINOCTIAL ? 1.0 : -1.0;
    const double roundoff = 8.0 * std::numeric_limits<double>::epsilon();
    double totalMass = 0.0; // [kg]
    for (const auto& payload : this->scPayloadBuffer) {
        if (!std::isfinite(payload.massSC) || payload.massSC < 0.0) {
            this->bskLogger.bskError(
                "FormationBarycenter equinoctial averaging requires finite nonnegative spacecraft mass.");
        }
        totalMass += payload.massSC;
    }
    if (!std::isfinite(totalMass) || totalMass <= 0.0) {
        this->bskLogger.bskError("FormationBarycenter equinoctial averaging requires finite positive total mass.");
    }

    double semimajorAxis = 0.0; // [m]
    double shapeSine = 0.0, shapeCosine = 0.0;
    double tiltSine = 0.0, tiltCosine = 0.0;
    double longitudeSine = 0.0, longitudeCosine = 0.0;
    for (std::size_t index = 0; index < this->scNavBuffer.size(); ++index) {
        auto& navigation = this->scNavBuffer[index];
        const double weight = this->scPayloadBuffer[index].massSC / totalMass;
        if (weight == 0.0) {
            continue;
        }
        for (int component = 0; component < 3; ++component) {
            if (!std::isfinite(navigation.r_BN_N[component]) || !std::isfinite(navigation.v_BN_N[component])) {
                this->bskLogger.bskError(
                    "FormationBarycenter equinoctial averaging requires finite position and velocity.");
            }
        }
        const double radius = v3Norm(navigation.r_BN_N); // [m]
        if (!std::isfinite(radius) || radius <= 0.0) {
            this->bskLogger.bskError(
                "FormationBarycenter equinoctial averaging requires a nonzero finite orbital radius.");
        }
        const double speedScale = std::sqrt(this->mu / radius); // [m/s]
        if (!std::isfinite(speedScale) || speedScale <= 0.0) {
            this->bskLogger.bskError(
                "FormationBarycenter equinoctial averaging requires a representable elliptic state.");
        }
        double radial_N[3], scaledVelocity_N[3], momentum_N[3], normal_N[3], eccentricity_N[3];
        v3Scale(1.0 / radius, navigation.r_BN_N, radial_N);
        v3Scale(1.0 / speedScale, navigation.v_BN_N, scaledVelocity_N);
        const double speedSquared = v3Dot(scaledVelocity_N, scaledVelocity_N);
        const double binding = 2.0 - speedSquared;
        v3Cross(radial_N, scaledVelocity_N, momentum_N);
        const double momentum = v3Norm(momentum_N);
        if (!std::isfinite(binding) || binding <= roundoff * std::fmax(2.0, speedSquared) || !std::isfinite(momentum) ||
            momentum <= roundoff * std::sqrt(speedSquared)) {
            this->bskLogger.bskError("FormationBarycenter equinoctial averaging supports only resolvable nonradial "
                                     "elliptic states (e < 1).");
        }
        const double memberAxis = radius / binding; // [m]
        v3Scale(1.0 / momentum, momentum_N, normal_N);
        const double transverse = std::hypot(normal_N[0], normal_N[1]);
        const double pole = factor * normal_N[2];
        if (pole < 0.0 && transverse <= roundoff) {
            this->bskLogger.bskError(
                "FormationBarycenter selected equinoctial representation is singular at the opposite equatorial pole.");
        }
        // Rationalize the denominator near the excluded pole to avoid subtracting nearly equal numbers.
        const double tiltScale = pole >= 0.0 ? 1.0 / (1.0 + pole) : (1.0 - pole) / (transverse * transverse);
        const double memberTiltSine = normal_N[0] * tiltScale;
        const double memberTiltCosine = -normal_N[1] * tiltScale;
        double firstAxis_N[3], secondAxis_N[3];
        equinoctialPlane(memberTiltSine, memberTiltCosine, factor, firstAxis_N, secondAxis_N);
        v3Cross(scaledVelocity_N, momentum_N, eccentricity_N);
        v3Subtract(eccentricity_N, radial_N, eccentricity_N);
        const double memberShapeSine = v3Dot(eccentricity_N, secondAxis_N);
        const double memberShapeCosine = v3Dot(eccentricity_N, firstAxis_N);
        const double eccentricity = std::hypot(memberShapeSine, memberShapeCosine);
        if (!std::isfinite(memberAxis) || !std::isfinite(eccentricity) || eccentricity >= 1.0) {
            this->bskLogger.bskError(
                "FormationBarycenter equinoctial averaging requires representable elliptic elements (e < 1).");
        }
        const double trueLongitude =
            std::atan2(v3Dot(navigation.r_BN_N, secondAxis_N), v3Dot(navigation.r_BN_N, firstAxis_N)); // [rad]
        const double ellipticFactor = std::sqrt((1.0 - eccentricity) * (1.0 + eccentricity));
        const double eccentricLongitude =
            trueLongitude +
            2.0 * std::atan2(memberShapeSine * std::cos(trueLongitude) - memberShapeCosine * std::sin(trueLongitude),
                             1.0 + ellipticFactor + memberShapeCosine * std::cos(trueLongitude) +
                                 memberShapeSine * std::sin(trueLongitude)); // [rad]
        const double meanLongitude = eccentricLongitude - memberShapeCosine * std::sin(eccentricLongitude) +
                                     memberShapeSine * std::cos(eccentricLongitude); // [rad]
        semimajorAxis += weight * memberAxis;
        shapeSine += weight * memberShapeSine;
        shapeCosine += weight * memberShapeCosine;
        tiltSine += weight * memberTiltSine;
        tiltCosine += weight * memberTiltCosine;
        longitudeSine += weight * std::sin(meanLongitude);
        longitudeCosine += weight * std::cos(meanLongitude);
    }
    if (std::hypot(longitudeSine, longitudeCosine) <= 64.0 * std::numeric_limits<double>::epsilon()) {
        this->bskLogger.bskError(
            "FormationBarycenter mean longitude is undefined because the weighted directions cancel.");
    }
    const double eccentricity = std::hypot(shapeSine, shapeCosine);
    if (!std::isfinite(semimajorAxis) || semimajorAxis <= 0.0 || !std::isfinite(eccentricity) || eccentricity >= 1.0) {
        this->bskLogger.bskError("FormationBarycenter averaged equinoctial elements are not a representable ellipse.");
    }
    const double meanLongitude = std::atan2(longitudeSine, longitudeCosine);                          // [rad]
    const double periapsisLongitude = eccentricity == 0.0 ? 0.0 : std::atan2(shapeSine, shapeCosine); // [rad]
    const double eccentricLongitude =
        M2E(meanLongitude - periapsisLongitude, eccentricity) + periapsisLongitude; // [rad]
    const double cosine = std::cos(eccentricLongitude);
    const double sine = std::sin(eccentricLongitude);
    const double beta = 1.0 / (1.0 + std::sqrt((1.0 - eccentricity) * (1.0 + eccentricity)));
    const double crossTerm = beta * shapeSine * shapeCosine;
    const double firstScale = 1.0 - beta * shapeSine * shapeSine;
    const double secondScale = 1.0 - beta * shapeCosine * shapeCosine;
    const double longitudeRate = (std::sqrt(this->mu / semimajorAxis) / semimajorAxis) /
                                 (1.0 - shapeCosine * cosine - shapeSine * sine); // [rad/s]
    double firstAxis_N[3], secondAxis_N[3];
    equinoctialPlane(tiltSine, tiltCosine, factor, firstAxis_N, secondAxis_N);
    for (int component = 0; component < 3; ++component) {
        position_N[component] =
            semimajorAxis * ((firstScale * cosine + crossTerm * sine - shapeCosine) * firstAxis_N[component] +
                             (secondScale * sine + crossTerm * cosine - shapeSine) * secondAxis_N[component]);
        velocity_N[component] = semimajorAxis * longitudeRate *
                                ((-firstScale * sine + crossTerm * cosine) * firstAxis_N[component] +
                                 (secondScale * cosine - crossTerm * sine) * secondAxis_N[component]);
        if (!std::isfinite(position_N[component]) || !std::isfinite(velocity_N[component])) {
            this->bskLogger.bskError(
                "FormationBarycenter equinoctial reference is not representable as finite position and velocity.");
        }
    }
}

/*! writes the output messages
 */
void FormationBarycenter::WriteOutputMessage(uint64_t CurrentClock)
{
    // write C++ output message
    this->transOutMsg.write(&this->transOutBuffer, this->moduleID, CurrentClock);

    // write C output message
    NavTransMsg_C_write(&this->transOutBuffer, &this->transOutMsgC, this->moduleID, CurrentClock);
}

/*! This is the main method that gets called every time the module is updated.
 */
void FormationBarycenter::UpdateState(uint64_t CurrentSimNanos)
{
    this->ReadInputMessages();
    this->transOutBuffer = this->transOutMsg.zeroMsgPayload; // zero the output message buffer
    this->computeBaricenter();
    this->WriteOutputMessage(CurrentSimNanos);
}
