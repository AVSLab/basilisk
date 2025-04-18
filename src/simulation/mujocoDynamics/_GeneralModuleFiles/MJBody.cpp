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

#include <stdexcept>
#include <type_traits>
#include <unordered_map>

#include <iostream>

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
            this->scalarJoints.emplace_back(mjsjoint, *this);
            break;
        case mjJNT_BALL:
            this->ballJoint.emplace(mjsjoint, *this);
            break;
        case mjJNT_FREE:
            this->freeJoint.emplace(mjsjoint, *this);
            break;
        default:
            throw std::runtime_error("Unknown joint type."); // should not happen unless MuJoCo adds new joint
        }
    }

}

void MJBody::configure(const mjModel* mujocoModel)
{
    MJObject::configure(mujocoModel);

    for (auto&& joint : this->scalarJoints) {
        joint.configure(mujocoModel);
    }
    if (this->ballJoint.has_value()) {
        this->ballJoint->configure(mujocoModel);
    }
    if (this->freeJoint.has_value()) {
        this->freeJoint->configure(mujocoModel);
    }

    for (auto&& site : this->sites) {
        site.configure(mujocoModel);
    }

    // Update the position of the center of mass
    auto& com = this->getCenterOfMass();
    auto bodyId = this->getId();
    auto siteId = com.getId();
    std::copy_n(mujocoModel->body_ipos + 3 * bodyId, 3, mujocoModel->site_pos + 3 * siteId);

    // Update the mass property states
    if (!this->massState) {
        // Should not happen
        this->getSpec().getScene().logAndThrow("Tried to configure MJBody before massState was created.");
    }

    this->massState->setState(Eigen::Matrix<double, 1, 1>{mujocoModel->body_mass[this->getId()]});
}

MJSite& MJBody::getSite(const std::string& name)
{
    auto sitePtr = std::find_if(std::begin(sites), std::end(sites), [&](auto&& obj) {
        return obj.getName() == name;
    });

    if (sitePtr == std::end(sites)) {
        this->getSpec().getScene().logAndThrow("Unknown site '" + name + "' in body '" + this->name + "'");
    }

    return *sitePtr;
}

MJScalarJoint& MJBody::getScalarJoint(const std::string& name)
{
    auto jointPtr = std::find_if(std::begin(scalarJoints), std::end(scalarJoints), [&](auto&& obj) {
        return obj.getName() == name;
    });

    if (jointPtr != std::end(scalarJoints)) return *jointPtr;

    this->getSpec().getScene().logAndThrow("Unknown scalar joint '" + name + "' in body '" + this->getName() + "'");
}

MJBallJoint& MJBody::getBallJoint()
{
    if (!this->ballJoint.has_value()) {
        this->getSpec().getScene().logAndThrow<std::runtime_error>("Tried to get a ball joint for a body without ball joints: " +
                                                    name);
    }
    return this->ballJoint.value();
}


MJFreeJoint & MJBody::getFreeJoint()
{
    if (!this->freeJoint.has_value()) {
        this->getSpec().getScene().logAndThrow<std::runtime_error>("Tried to get a free joint for a body without free joints: " +
                                                    name);
    }
    return this->freeJoint.value();
}

void MJBody::setPosition(const Eigen::Vector3d& position)
{
    if (!this->freeJoint.has_value()) {
        this->getSpec().getScene().logAndThrow<std::runtime_error>("Tried to set position in non-free body " +
                                                    name);
    }
    this->freeJoint.value().setPosition(position);
}

void MJBody::setVelocity(const Eigen::Vector3d& velocity)
{
    if (!this->freeJoint.has_value()) {
        this->getSpec().getScene().logAndThrow<std::runtime_error>("Tried to set velocity in non-free body " +
                                                    name);
    }
    this->freeJoint.value().setVelocity(velocity);
}

void MJBody::setAttitude(const Eigen::MRPd& attitude)
{
    if (!this->freeJoint.has_value()) {
        this->getSpec().getScene().logAndThrow<std::runtime_error>("Tried to set attitude in non-free body " +
                                                    name);
    }
    this->freeJoint.value().setAttitude(attitude);
}

void MJBody::setAttitudeRate(const Eigen::Vector3d& attitudeRate)
{
    if (!this->freeJoint.has_value()) {
        this->getSpec().getScene().logAndThrow<std::runtime_error>("Tried to set attitude rate in non-free body " +
                                                    name);
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

    massPropertiesOutMsgPayload.massSC = this->massState->getState()(0);
    this->massPropertiesOutMsg.write(&massPropertiesOutMsgPayload,
                                     this->getSpec().getScene().moduleID,
                                     CurrentSimNanos);

    for (auto&& joint : this->scalarJoints) {
        joint.writeJointStateMessage(CurrentSimNanos);
    }
}

void MJBody::registerStates(DynParamRegisterer paramManager)
{
    this->massState = paramManager.registerState(1, 1, "mass");
}

void MJBody::updateMujocoModelFromMassProps()
{
    auto m = spec.getMujocoModel();

    double newMass = this->massState->getState()(0, 0);
    auto diff = abs(m->body_mass[this->getId()] - newMass);
    if (diff > 10 * std::numeric_limits<double>::epsilon()) {

        // Update the mass in the mjModel AND mjsBody
        m->body_mass[this->getId()] = newMass;
        this->mjsObject->mass = newMass;

        // Update the inertia in the mjModel AND mjsBody
        for (size_t i = 0; i < 3; i++) {
            m->body_inertia[3 * this->getId() + i] *= newMass / m->body_mass[this->getId()];
            this->mjsObject->inertia[i] = m->body_inertia[3 * this->getId() + i];
        }

        this->getSpec().getScene().markMujocoModelConstAsStale();
        this->getSpec().getScene().markKinematicsAsStale();
    }
}

void MJBody::updateMassPropsDerivative()
{
    if (this->derivativeMassPropertiesInMsg.isLinked()) {
        auto deriv = this->derivativeMassPropertiesInMsg();
        this->massState->setDerivative(Eigen::Matrix<double, 1, 1>{deriv.massSC});
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
        this->getSpec().getScene().logAndThrow("Tried to create site " + name + " twice for body " +
                                     this->name);
    }

    auto mjssite = mjs_addSite(this->mjsObject, 0);
    mjs_setString(mjssite->name, name.c_str());

    auto& site = this->sites.emplace_back(mjssite, *this);

    spec.markAsNeedingToRecompileModel(); // Any updates to the 'structure' (i.e. new elements), we need to recompile

    site.setPositionRelativeToBody(position);
    site.setAttitudeRelativeToBody(attitude);
}

bool MJBody::hasSite(const std::string& name) const
{
    return std::find_if(std::begin(sites), std::end(sites), [&](auto&& obj) {
               return obj.getName() == name;
           }) != std::end(sites);
}
