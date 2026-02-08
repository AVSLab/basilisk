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

#ifndef MJBODY_H
#define MJBODY_H

#include <mujoco/mujoco.h>

#include <Eigen/Dense>

#include <string>
#include <vector>
#include <list>

#include "architecture/messaging/messaging.h"
#include "architecture/utilities/avsEigenSupport.h"
#include "simulation/dynamics/_GeneralModuleFiles/dynParamManager.h"
#include "architecture/msgPayloadDefC/SCMassPropsMsgPayload.h"

#include "MJJoint.h"
#include "MJObject.h"
#include "MJSite.h"
#include "StatefulSysModel.h"

/// @cond
/**
 * @brief Specialization of getObjectTypeName for mjsBody.
 *
 * Provides a human-readable name for the MuJoCo body type.
 *
 * @return A string view representing the body type.
 */
template <>
constexpr std::string_view MJBasilisk::detail::getObjectTypeName<mjsBody>()
{
    return "body";
}
/// @endcond

class MJScene;
class MJSpec;

/**
 * @brief Represents a MuJoCo body object.
 *
 * The `MJBody` class manages a body in the MuJoCo simulation, including its joints, sites,
 * and inertial properties.
 */
class MJBody : public MJObject<mjsBody>
{
public:
    /**
     * @brief Constructs an `MJBody` with a given MuJoCo body object and the spec where it's created.
     *
     * @param mjsBody Pointer to the MuJoCo body.
     * @param spec Reference to the MJSpec associated with the body.
     */
    MJBody(mjsBody* mjsBody, MJSpec& spec);

    // Delete copy and move constructors and assignment operators
    MJBody(const MJBody&) = delete;
    MJBody(MJBody&&) = delete;
    MJBody& operator=(const MJBody&) = delete;
    MJBody& operator=(MJBody&&) = delete;

    /**
     * @brief Configures the body within a given MuJoCo model.
     *
     * @param model Pointer to the MuJoCo model used for configuration.
     */
    void configure(const mjModel* model);

    /**
     * @brief Adds a site to the body with a specified name, position, and attitude.
     *
     * @param name The name of the site.
     * @param position The position of the site relative to the body.
     * @param attitude The orientation of the site as Modified Rodrigues Parameters (MRP).
     */
    void addSite(std::string name,
                 const Eigen::Vector3d& position,
                 const Eigen::MRPd& attitude = {0., 0., 0.});

    /**
     * @brief Checks if the body has a site with a given name.
     *
     * @param name The name of the site to search for.
     * @return True if the site exists, false otherwise.
     */
    bool hasSite(const std::string& name) const;

    /**
     * @brief Retrieves a site by name.
     *
     * @param name The name of the site to retrieve.
     * @return A reference to the `MJSite` with the specified name.
     * @throw std::invalid_argument if the site does not exist.
     */
    MJSite& getSite(const std::string& name);

    /**
     * @brief Retrieves the center of mass site of the object.
     *
     * This site is located at the center of mass of the body (ignoring
     * any child bodies). The frame orientation is the same as the origin
     * site frame orientation.
     *
     * @return A reference to the center of mass site.
     */
    MJSite& getCenterOfMass() { return getSite(this->name + "_com"); }

    /**
     * @brief Retrieves the origin site.
     *
     * This site corresponds to the origin (main, base) reference frame
     * of the body. All other sites' positions, as well as the center
     * of mass, are given with respect to this reference frame.
     *
     * @return A reference to the origin site.
     */
    MJSite& getOrigin() { return getSite(this->name + "_origin"); }

    /**
     * @brief Retrieves the associated MJSpec object.
     *
     * @return A reference to the `MJSpec` associated with this body.
     */
    MJSpec& getSpec() { return spec; }

    /**
     * @brief Retrieves a scalar joint by name.
     *
     * @param name The name of the scalar joint.
     * @return A reference to the `MJScalarJoint`.
     * @throw std::invalid_argument if the joint does not exist.
     * @throw std::invalid_argument if the joint with that name is not a scalar joint.
     */
    MJScalarJoint& getScalarJoint(const std::string& name);

    /**
     * @brief Checks whether the body has a ball joint.
     *
     * @return True if the body has a ball joint.
     */
    bool hasBallJoint() { return this->ballJoint.has_value(); }

    /**
     * @brief Retrieves the ball joint of this body, if it exists.
     *
     * @return A reference to the `MJBallJoint`.
     * @throw std::runtime_error if the joint does not have a ball joint.
     */
    MJBallJoint& getBallJoint();

    /**
     * @brief Checks whether this is a free body.
     *
     * 'Free' bodies are those that have a 'free' joint, and thus
     * they can move freely in the environment. For these bodies,
     * we can set the position, velocity, attitude, and angular rate.
     * Usually, 'free' bodies are those without parent bodies in
     * the simulation ('top-level' bodies).
     *
     * @return True if the body has a free joint.
     */
    bool isFree() { return this->freeJoint.has_value(); }

    /**
     * @brief Retrieves the free joint of this body, if it exists.
     *
     * @return A reference to the `MJBallJoint`.
     * @throw std::runtime_error if the joint does not have a free joint.
     */
    MJFreeJoint& getFreeJoint();

    /**
     * @brief Sets the position of the body.
     *
     * This method may only be called for 'free' bodies. Free
     * bodies are those that have a 'free' joint.
     *
     * @param position The desired 3D position vector.
     * @throw std::runtime_error if this body is not free (has no 'free' joint).
     */
    void setPosition(const Eigen::Vector3d& position);

    /**
     * @brief Sets the translational velocity of the body.
     *
     * This method may only be called for 'free' bodies. Free
     * bodies are those that have a 'free' joint.
     *
     * @param velocity The desired 3D velocity vector.
     * @throw std::runtime_error if this body is not free (has no 'free' joint).
     */
    void setVelocity(const Eigen::Vector3d& velocity);

    /**
     * @brief Sets the orientation (attitude) of the body.
     *
     * This method may only be called for 'free' bodies. Free
     * bodies are those that have a 'free' joint.
     *
     * @param attitude The orientation represented as Modified Rodrigues Parameters (MRP).
     * @throw std::runtime_error if this body is not free (has no 'free' joint).
     */
    void setAttitude(const Eigen::MRPd& attitude);

    /**
     * @brief Sets the attitude rate (angular velocity) of the body.
     *
     * This method may only be called for 'free' bodies. Free
     * bodies are those that have a 'free' joint.
     *
     * @param attitudeRate The desired 3D attitude rate vector.
     * @throw std::runtime_error if this body is not free (has no 'free' joint).
     */
    void setAttitudeRate(const Eigen::Vector3d& attitudeRate);

    /**
     * @brief Writes forward kinematics messages based on the current state.
     *
     * Writes the state (position, attitude, etc.) out messages for all
     * sites in this body.
     *
     * @param m Pointer to the MuJoCo model.
     * @param d Pointer to the MuJoCo data.
     * @param CurrentSimNanos The current simulation time in nanoseconds.
     */
    void writeFwdKinematicsMessages(mjModel* m, mjData* d, uint64_t CurrentSimNanos);

    /**
     * @brief Writes body-mass-properties and joint-state messages.
     *
     * @param CurrentSimNanos The current simulation time in nanoseconds.
     */
    void writeStateDependentOutputMessages(uint64_t CurrentSimNanos);

    /**
     * @brief Registers the body's states with a dynamic parameter registerer.
     *
     * Currently, only the mass of the body is considered a parameter.
     *
     * @param paramManager The dynamic parameter registerer.
     */
    void registerStates(DynParamRegisterer paramManager);

    /**
     * @brief Updates the MuJoCo model from the mass properties of the body.
     *
     * The mass of the body is a state, which may change in time (for example,
     * when a thruster burns fuel). This change in mass must be relayed back
     * to MuJoCo. Calling this method will update the `mjModel` with the
     * mass value stored in the mass state of this body.
     *
     * Note that the inertia is also updated (scaled by the change in mass).
     * The center of mass remains constant.
     *
     * This method will also mark the kinematics and mujoco model 'const' as stale.
     */
    void updateMujocoModelFromMassProps();

    /**
     * @brief Updates the derivative of the mass state with the information
     * from the connected `derivativeMassPropertiesInMsg`.
     *
     * If no such message is connected, then the derivative is assumed to be zero.
     */
    void updateMassPropsDerivative();

    /**
     * @brief Updates all constrained equality joints associated with the body.
     *
     * Calls `MJScalarJoint::updateConstrainedEquality` for all scalar joints in
     * this body.
     */
    void updateConstrainedEqualityJoints();

public:
    Message<SCMassPropsMsgPayload> massPropertiesOutMsg; ///< Message to output body mass properties.

    ReadFunctor<SCMassPropsMsgPayload> derivativeMassPropertiesInMsg; ///< Functor to read mass properties derivatives.

protected:
    MJSpec& spec; ///< Reference to the spec where this body is defined.

    std::list<MJSite> sites; ///< List of sites associated with the body.

    std::optional<MJFreeJoint> freeJoint; ///< Optional free joint associated with the body.
    std::optional<MJBallJoint> ballJoint; ///< Optional ball joint associated with the body.
    std::list<MJScalarJoint> scalarJoints; ///< List of scalar joints associated with the body.

    StateData* massState; ///< State data representing the mass of the body.
};

#endif
