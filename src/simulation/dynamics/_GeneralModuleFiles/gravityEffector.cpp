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

#include <functional>

#include "gravityEffector.h"
#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/macroDefinitions.h"


GravBodyData::GravBodyData()
    : gravityModel{std::make_shared<PointMassGravityModel>()},
      localPlanet{std::invoke([]() {
          SpicePlanetStateMsgPayload payload{};
          m33SetIdentity(payload.J20002Pfix); // Initialize rotation matrix to identity
          return payload;
      })}
{
}

void GravBodyData::initBody(int64_t moduleID)
{
    std::optional<std::string> errorMessage;
    if (this->gravityModel) {
        this->gravityModel->bskLogger = &bskLogger;
        errorMessage = this->gravityModel->initializeParameters(*this);
    }
    else {
        errorMessage = "Gravity model is null";
    }

    if (errorMessage) {
        std::string fullMsg =
            "Error initializating body '" + this->planetName + "'. " + errorMessage.value();
        this->bskLogger.bskLog(BSK_ERROR, fullMsg.c_str());
    }
}

Eigen::Vector3d GravBodyData::computeGravityInertial(Eigen::Vector3d r_I, uint64_t simTimeNanos)
{
    double dt = diffNanoToSec(simTimeNanos, this->timeWritten);
    Eigen::Matrix3d dcm_PfixN = c2DArray2EigenMatrix3d(this->localPlanet.J20002Pfix).transpose();
    if (dcm_PfixN
            .isZero()) { // Sanity check for connected messages that do not initialize J20002Pfix
        dcm_PfixN = Eigen::Matrix3d::Identity();
    }

    Eigen::Matrix3d dcm_PfixN_dot =
        c2DArray2EigenMatrix3d(this->localPlanet.J20002Pfix_dot).transpose();
    dcm_PfixN += dcm_PfixN_dot * dt;

    // store the current planet orientation and rates
    *this->J20002Pfix = dcm_PfixN;
    *this->J20002Pfix_dot = dcm_PfixN_dot;

    // Compute position in the body-fixed reference frame and compute the gravity
    Eigen::Matrix3d dcm_NPfix = dcm_PfixN.transpose();
    Eigen::Vector3d r_Pfix = dcm_NPfix * r_I;
    Eigen::Vector3d grav_Pfix = this->gravityModel->computeField(r_Pfix);

    return dcm_PfixN * grav_Pfix;
}

void GravBodyData::loadEphemeris()
{
    if (this->planetBodyInMsg.isLinked()) {
        this->localPlanet = this->planetBodyInMsg();
        this->timeWritten = this->planetBodyInMsg.timeWritten();
    }
    else {
        // use default zero planet state information
        this->localPlanet = this->planetBodyInMsg.zeroMsgPayload;
        this->timeWritten = 0;
    }
}

void GravBodyData::registerProperties(DynParamManager& statesIn)
{
    if (this->planetName == "") {
        auto errorMessage = "You must specify a planetary body name in GravBodyData";
        this->bskLogger.bskLog(BSK_ERROR, errorMessage);
        throw std::invalid_argument(errorMessage);
    }

    Eigen::Vector3d stateInit = Eigen::Vector3d::Zero();
    this->r_PN_N = statesIn.createProperty(this->planetName + ".r_PN_N", stateInit);
    this->v_PN_N = statesIn.createProperty(this->planetName + ".v_PN_N", stateInit);

    Eigen::MatrixXd muInit = Eigen::MatrixXd::Zero(1, 1);
    this->muPlanet = statesIn.createProperty(this->planetName + ".mu", muInit);

    this->J20002Pfix = statesIn.createProperty(
        this->planetName + ".J20002Pfix", c2DArray2EigenMatrix3d(this->localPlanet.J20002Pfix));
    this->J20002Pfix_dot =
        statesIn.createProperty(this->planetName + ".J20002Pfix_dot",
                                c2DArray2EigenMatrix3d(this->localPlanet.J20002Pfix_dot));
}

void GravityEffector::Reset(uint64_t currentSimNanos)
{
    // Initializes the bodies
    for (auto&& body : this->gravBodies) {
        body->initBody(this->moduleID);
    }
}

void GravityEffector::UpdateState(uint64_t currentSimNanos)
{
    // Updates the central body
    this->centralBody.reset();
    for (auto&& body : this->gravBodies) {
        body->loadEphemeris();

        if (!body->isCentralBody) continue;

        if (this->centralBody) // A centralBody was already set
        {
            auto errorMessage = "Specified two central bodies at the same time";
            this->bskLogger.bskLog(BSK_ERROR, errorMessage);
            throw std::invalid_argument(errorMessage);
        }
        else {
            this->centralBody = body;
        }
    }

    this->writeOutputMessages(currentSimNanos);
}

void GravityEffector::writeOutputMessages(uint64_t currentSimNanos)
{
    if (this->centralBodyOutMsg.isLinked() && this->centralBody) {
        this->centralBodyOutMsg.write(&this->centralBody->localPlanet, this->moduleID,
                                      currentSimNanos);
    }
}

void GravityEffector::prependSpacecraftNameToStates()
{
    this->inertialPositionPropName =
        this->nameOfSpacecraftAttachedTo + this->inertialPositionPropName;
    this->inertialVelocityPropName =
        this->nameOfSpacecraftAttachedTo + this->inertialVelocityPropName;
    this->vehicleGravityPropName = this->nameOfSpacecraftAttachedTo + this->vehicleGravityPropName;
}

void GravityEffector::registerProperties(DynParamManager& statesIn)
{
    static const Eigen::Vector3d zeroVector3d = Eigen::Vector3d::Zero();

    this->gravProperty = statesIn.createProperty(this->vehicleGravityPropName, zeroVector3d);
    this->inertialPositionProperty =
        statesIn.createProperty(this->inertialPositionPropName, zeroVector3d);
    this->inertialVelocityProperty =
        statesIn.createProperty(this->inertialVelocityPropName, zeroVector3d);

    // register planet position and velocity state vectors as parameters in the
    // state engine
    for (auto&& body : this->gravBodies) {
        body->registerProperties(statesIn);
    }
}

void GravityEffector::linkInStates(DynParamManager& statesIn)
{
    this->timeCorr = statesIn.getPropertyReference(this->systemTimeCorrPropName);
}

void GravityEffector::computeGravityField(Eigen::Vector3d r_cF_N, Eigen::Vector3d rDot_cF_N)
{
    uint64_t systemClock = (uint64_t)this->timeCorr->data()[0];
    Eigen::Vector3d r_cN_N; // position of s/c CoM wrt N
    Eigen::Vector3d r_CN_N; // inertial position of central body if there is one. Big C is
                            // central body. Little c is CoM of s/c

    if (this->centralBody) // If there is a central body
    {
        r_CN_N = getEulerSteppedGravBodyPosition(this->centralBody);
        r_cN_N = r_cF_N + r_CN_N; // shift s/c to be wrt inertial frame origin
                                  // if it isn't already
    }
    else {
        r_cN_N = r_cF_N;
    }

    // acceleration of CoM of s/c wrt Frame in which it is stored/integrated in spacecraft
    Eigen::Vector3d rDotDot_cF_N = Eigen::Vector3d::Zero();

    for (auto&& body : this->gravBodies) {
        // position of Planet being queried wrt N
        Eigen::Vector3d r_PN_N = getEulerSteppedGravBodyPosition(body);
        Eigen::Vector3d r_cP_N = r_cN_N - r_PN_N; // position of s/c CoM wrt planet in N

        if (this->centralBody && !body->isCentralBody) // If there is a central body, and its not
                                                       // 'body'
        {
            // Subtract accel of central body due to other bodies to get
            // RELATIVE accel of s/c. See Vallado on "Three-body and n-body
            // Equations"
            rDotDot_cF_N += body->computeGravityInertial(r_PN_N - r_CN_N, systemClock);
        }
        // acceleration of c wrt N in N, due to P
        rDotDot_cF_N += body->computeGravityInertial(r_cP_N, systemClock);

        // store planet states in the state engine parameters
        *(body->r_PN_N) = r_PN_N;
        *(body->v_PN_N) = cArray2EigenVector3d(body->localPlanet.VelocityVector);
        (*(body->muPlanet))(0, 0) = body->mu;
    }

    *this->gravProperty = rDotDot_cF_N;
}

void GravityEffector::updateInertialPosAndVel(Eigen::Vector3d r_BF_N, Eigen::Vector3d rDot_BF_N)
{
    // Here we add the central body inertial position and velocities to the
    // central-body-relative position and velicities which are propogated
    // relative to the central body
    if (this->centralBody) // If there is a central body
    {
        Eigen::Vector3d r_CN_N = getEulerSteppedGravBodyPosition(this->centralBody);
        *this->inertialPositionProperty = r_CN_N + r_BF_N;
        *this->inertialVelocityProperty =
            cArray2EigenMatrixXd(this->centralBody->localPlanet.VelocityVector, 3, 1) + rDot_BF_N;
    }
    else {
        *this->inertialPositionProperty = r_BF_N;
        *this->inertialVelocityProperty = rDot_BF_N;
    }
}

Eigen::Vector3d
GravityEffector::getEulerSteppedGravBodyPosition(std::shared_ptr<GravBodyData> bodyData)
{
    uint64_t systemClock = (uint64_t)this->timeCorr->data()[0];
    double dt = diffNanoToSec(systemClock, bodyData->timeWritten);
    Eigen::Vector3d r_PN_N = cArray2EigenVector3d(bodyData->localPlanet.PositionVector);
    r_PN_N += cArray2EigenVector3d(bodyData->localPlanet.VelocityVector) * dt;
    return r_PN_N;
}

void GravityEffector::updateEnergyContributions(Eigen::Vector3d r_cF_N, double& orbPotEnergyContr)
{
    Eigen::Vector3d r_CN_N; // C is central body. position of C wrt N in N
    Eigen::Vector3d r_cN_N; // position c wrt N in N

    if (this->centralBody) { // Evaluates true if there is a central body, false
                             // otherwise
        r_CN_N = getEulerSteppedGravBodyPosition(this->centralBody);
        r_cN_N = r_cF_N + r_CN_N; // shift s/c to be wrt inertial frame origin
                                  // if it isn't already
    }
    else {
        r_cN_N = r_cF_N;
    }

    for (auto&& body : this->gravBodies) {
        // P is planet being queried. position of planet wrt N in N
        Eigen::Vector3d r_PN_N = getEulerSteppedGravBodyPosition(body);
        // c is s/c CoM. position of c wrt P in N
        Eigen::Vector3d r_cP_N = r_cN_N - r_PN_N;

        if (this->centralBody && !body->isCentralBody) // If there is a central body and it's not
                                                       // 'body'
        {
            // potential of central body w/in current planet field. leads to
            // relative potential energy solution
            orbPotEnergyContr += body->gravityModel->computePotentialEnergy(r_PN_N - r_CN_N);
        }
        orbPotEnergyContr = body->gravityModel->computePotentialEnergy(
            r_cP_N); // Potential w/in current planet field
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
