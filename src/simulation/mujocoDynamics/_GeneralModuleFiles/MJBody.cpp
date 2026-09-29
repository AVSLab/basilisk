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

#include "MJBody.h"
#include "MJScene.h"
#include "MJSpec.h"

#include <cmath>
#include <stdexcept>
#include <type_traits>
#include <unordered_map>

#include <iostream>

namespace
{
    /**
     * Returns true if the first three scalar joints are translational
     * and along the main axis ([1,0,0], [0,1,0], [0,0,1]).
    */
    bool areJoints3DTranslation(std::list<MJScalarJoint>& joints)
    {
        if (joints.size() < 3) return false;

        Eigen::Index idx = 0;
        for (auto&& joint : joints)
        {
            if (idx == 3) break;
            if (joint.isHinge()) return false;
            if (std::fabs(joint.getAxis()[idx] - 1) > 1e-10) return false;
            ++idx;
        }

        return true;
    }
}

MJBody::MJBody(mjsBody* body, MJSpec& spec)
    : MJObject(body), spec(spec)
{

    // SITES
    for (auto child = mjs_firstChild(body, mjOBJ_SITE, 0); child; child = mjs_nextChild(body, child, 0))
    {

        auto mjssite = mjs_asSite(child);
        assert(mjssite != NULL);
        this->sites.emplace_back(mjssite, *this);
    }

    if (!this->hasSite(this->name + "_com")) {
        this->addSite(this->name + "_com", Eigen::Vector3d::Zero());
    }

    if (!this->hasSite(this->name + "_origin")) {
        this->addSite(this->name + "_origin", Eigen::Vector3d::Zero());
    }

    // JOINTS
    for (auto child = mjs_firstChild(body, mjOBJ_JOINT, 0); child; child = mjs_nextChild(body, child, 0))
    {

        auto mjsjoint = mjs_asJoint(child);
        assert(mjsjoint != NULL);

        switch (mjsjoint->type)
        {
        case mjJNT_HINGE:
        case mjJNT_SLIDE:
            this->orderedJoints.push_back(&this->scalarJoints.emplace_back(mjsjoint, *this));
            break;
        case mjJNT_BALL:
            this->ballJoint.emplace(mjsjoint, *this);
            this->orderedJoints.push_back(&this->ballJoint.value());
            break;
        case mjJNT_FREE:
            this->freeJoint.emplace(mjsjoint, *this);
            this->orderedJoints.push_back(&this->freeJoint.value());
            break;
        default:
            throw std::runtime_error("Unknown joint type."); // should not happen unless MuJoCo adds new joint
        }
    }

}

void
MJBody::configure(mjModel* mujocoModel)
{
    MJObject::configure(mujocoModel);
    for (auto& joint : this->scalarJoints) {
        joint.configure(mujocoModel);
    }
    if (this->ballJoint) {
        this->ballJoint->configure(mujocoModel);
    }
    if (this->freeJoint) {
        this->freeJoint->configure(mujocoModel);
    }
    for (auto& site : this->sites) {
        site.configure(mujocoModel);
    }
    auto& com = this->getCenterOfMass();
    const Eigen::Vector3d position(mujocoModel->body_ipos + 3 * this->getId());
    com.commitPositionRelativeToBody(position, mujocoModel);
    // A site compiled at the body origin ignores site_pos until it is recompiled
    // with a nonzero offset. Request that update once for an offset center of mass.
    if (mujocoModel->site_sameframe[com.getId()] == mjSAMEFRAME_BODY && position.norm() > 1e-9) { // [m]
        this->getSpec().markAsNeedingToRecompileModel();
    }
}

MJSite& MJBody::getSite(const std::string& name)
{
    auto sitePtr = std::find_if(std::begin(sites), std::end(sites), [&](auto&& obj) {
        return obj.getName() == name;
    });

    if (sitePtr == std::end(sites)) {
        this->getSpec().getScene().bskLogger.bskError("Unknown site '%s' in body '%s'", name.c_str(), this->name.c_str());
    }

    return *sitePtr;
}

MJScalarJoint& MJBody::getScalarJoint(const std::string& name)
{
    auto jointPtr = std::find_if(std::begin(scalarJoints), std::end(scalarJoints), [&](auto&& obj) {
        return obj.getName() == name;
    });

    if (jointPtr != std::end(scalarJoints)) return *jointPtr;

    this->getSpec().getScene().bskLogger.bskError("Unknown scalar joint '%s' in body '%s'", name.c_str(), this->getName().c_str());
}

MJBallJoint& MJBody::getBallJoint()
{
    if (!this->ballJoint.has_value()) {
        this->getSpec().getScene().bskLogger.bskError("Tried to get a ball joint for a body without ball joints: %s", name.c_str());
    }
    return this->ballJoint.value();
}

MJFreeJoint & MJBody::getFreeJoint()
{
    if (!this->freeJoint.has_value()) {
        this->getSpec().getScene().bskLogger.bskError("Tried to get a free joint for a body without free joints: %s", name.c_str());
    }
    return this->freeJoint.value();
}

void MJBody::setPosition(const Eigen::Vector3d& position)
{
    if (this->freeJoint.has_value()) {
        this->freeJoint.value().setPosition(position);
    } else if (areJoints3DTranslation(scalarJoints))
    {
        Eigen::Index idx = 0;
        for (auto&& joint : scalarJoints)
        {
            if (idx == 3) break;
            joint.setPosition(position[idx]);
            ++idx;
        }
    } else {
        this->getSpec().getScene().bskLogger.bskError("Tried to set position in a body with no 'free' joint or no 3D translational joints: %s", name.c_str());
    }
}

void MJBody::setVelocity(const Eigen::Vector3d& velocity)
{
    if (this->freeJoint.has_value()) {
        this->freeJoint.value().setVelocity(velocity);
    } else if (areJoints3DTranslation(scalarJoints))
    {
        Eigen::Index idx = 0;
        for (auto&& joint : scalarJoints)
        {
            if (idx == 3) break;
            joint.setVelocity(velocity[idx]);
            ++idx;
        }
    } else {
        this->getSpec().getScene().bskLogger.bskError("Tried to set velocity in a body with no 'free' joint or no 3D translational joints: %s", name.c_str());
    }
}

void MJBody::setAttitude(const Eigen::MRPd& attitude)
{
    if (!this->freeJoint.has_value()) {
        this->getSpec().getScene().bskLogger.bskError("Tried to set attitude in non-free body: %s", name.c_str());
    }
    this->freeJoint.value().setAttitude(attitude);
}

void MJBody::setAttitudeRate(const Eigen::Vector3d& attitudeRate)
{
    if (!this->freeJoint.has_value()) {
        this->getSpec().getScene().bskLogger.bskError("Tried to set attitude rate in non-free body: %s", name.c_str());
    }
    this->freeJoint.value().setAttitudeRate(attitudeRate);
}

void MJBody::writeFwdKinematicsMessages(mjModel* m, mjData* d, uint64_t CurrentSimNanos)
{
    for (auto&& site : this->sites) {
        site.writeFwdKinematicsMessage(m, d, CurrentSimNanos);
    }
}

void MJBody::writeStateDependentOutputMessages(uint64_t CurrentSimNanos)
{
    SCMassPropsMsgPayload massPropertiesOutMsgPayload;

    massPropertiesOutMsgPayload.massSC = this->getMass();
    this->massPropertiesOutMsg.write(&massPropertiesOutMsgPayload,
                                     this->getSpec().getScene().moduleID,
                                     CurrentSimNanos);

    for (auto&& joint : this->scalarJoints) {
        joint.writeJointStateMessage(CurrentSimNanos);
    }
}

void
MJBody::registerJointPositionStates(DynParamRegisterer registerer, bool highOrderAttitude)
{
    for (MJJoint* joint : this->orderedJoints) {
        joint->registerPositionStates(registerer, highOrderAttitude);
    }
}

void
MJBody::registerJointVelocityStates(DynParamRegisterer registerer)
{
    for (MJJoint* joint : this->orderedJoints) {
        joint->registerVelocityStates(registerer);
    }
}

void
MJBody::setJointPositionDerivativesFromMujoco(const mjData* data)
{
    for (MJJoint* joint : this->orderedJoints) {
        joint->setPositionDerivativeFromMujoco(data);
    }
}

void
MJBody::validateJointStateLayout(const double* qposBase, const double* qvelBase) const
{
    for (const MJJoint* joint : this->orderedJoints) {
        joint->validateStateLayout(qposBase, qvelBase);
    }
}

double MJBody::getMass()
{
    // This body's mass lives at its body id in the scene's bulk mass state.
    return this->getSpec().getScene().getMassState()->stateView()(static_cast<Eigen::Index>(this->getId()));
}

void
MJBody::applyPrevalidatedMass(mjModel* model, double newMass) noexcept
{
    const double oldMass = model->body_mass[this->getId()];
    constexpr double massEpsilon = 10.0 * std::numeric_limits<double>::epsilon();
    const double diff = std::abs(oldMass - newMass);
    if (diff > massEpsilon) {
        // Compute the ratio before overwriting body_mass (else it would be 1.0).
        // Shape is fixed, so inertia scales linearly with mass.
        const double massRatio = newMass / oldMass;

        // Update the mass in the mjModel AND mjsBody
        model->body_mass[this->getId()] = newMass;
        this->mjsObject->mass = newMass;

        // Update the inertia in the mjModel AND mjsBody
        for (size_t i = 0; i < 3; i++) {
            model->body_inertia[3 * this->getId() + i] *= massRatio;
            this->mjsObject->inertia[i] = model->body_inertia[3 * this->getId() + i];
        }

        this->getSpec().getScene().markMujocoModelConstAsStale();
        this->getSpec().getScene().markKinematicsAsStale();
    }
}

void MJBody::updateMassPropsDerivative()
{
    if (this->derivativeMassPropertiesInMsg.isLinked()) {
        auto deriv = this->derivativeMassPropertiesInMsg();
        // Write into this body's entry of the bulk mass state derivative.
        this->getSpec().getScene().getMassState()->derivativeView()(static_cast<Eigen::Index>(this->getId())) =
          deriv.massSC;
    }
}

void MJBody::updateConstrainedEqualityJoints()
{
    for (auto&& joint : this->scalarJoints) {
        joint.updateConstrainedEquality();
    }
}

void MJBody::addSite(std::string name, const Eigen::Vector3d& position, const Eigen::MRPd& attitude)
{

    if (this->hasSite(name)) {
        this->getSpec().getScene().bskLogger.bskError("Tried to create site '%s' twice for body '%s'", name.c_str(), this->name.c_str());
    }

    spec.markAsNeedingToRecompileModel();
    auto mjssite = mjs_addSite(this->mjsObject, 0);
    MJBasilisk::detail::setSpecObjectName(mjssite, name);

    auto& site = this->sites.emplace_back(mjssite, *this);

    site.setPositionRelativeToBody(position);
    site.setAttitudeRelativeToBody(attitude);
}

bool MJBody::hasSite(const std::string& name) const
{
    return std::find_if(std::begin(sites), std::end(sites), [&](auto&& obj) {
               return obj.getName() == name;
           }) != std::end(sites);
}
