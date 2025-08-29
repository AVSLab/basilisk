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

#ifndef MJJOINT_H
#define MJJOINT_H

#include <mujoco/mujoco.h>

#include "architecture/messaging/messaging.h"
#include "architecture/msgPayloadDefC/ScalarJointStateMsgPayload.h"
#include "architecture/utilities/avsEigenSupport.h"
#include "simulation/dynamics/_GeneralModuleFiles/dynParamManager.h"

#include "MJEquality.h"
#include "MJObject.h"

class MJBody;

/// @cond
/**
 * @brief Specialization of getObjectTypeName for mjsJoint.
 *
 * Provides a human-readable name for the MuJoCo joint type.
 *
 * @return A string view representing the joint type.
 */
template <>
constexpr std::string_view MJBasilisk::detail::getObjectTypeName<mjsJoint>()
{
    return "joint";
}
/// @endcond

/**
 * @brief Represents a MuJoCo joint.
 *
 * The `MJJoint` class provides basic functionality to handle joints in MuJoCo.
 * This is a base class for more specific joint types.
 */
class MJJoint : public MJObject<mjsJoint>
{
public:
    /**
     * @brief Constructs an `MJJoint` with a given joint and the body
     * it's attached to.
     *
     * @param joint Pointer to the MuJoCo joint.
     * @param body Reference to the body the joint is associated with.
     */
    MJJoint(mjsJoint* joint, MJBody& body) : MJObject(joint), body(body) {}

    // Delete copy and move constructors and assignment operators
    MJJoint(const MJJoint&) = delete;
    MJJoint(MJJoint&&) = delete;
    MJJoint& operator=(const MJJoint&) = delete;
    MJJoint& operator=(MJJoint&&) = delete;

    /**
     * @brief Virtual destructor for MJJoint.
     */
    virtual ~MJJoint() {}

    /**
     * @brief Configures the joint within a given MuJoCo model.
     *
     * @param m Pointer to the MuJoCo model used for configuration.
     */
    void configure(const mjModel* m);

protected:
    /**
     * @brief Checks if the joint has been properly initialized.
     *
     * Throws an exception if initialization has not been completed.
     */
    void checkInitialized();

protected:
    MJBody& body; ///< Reference to the body the joint is attached to.

    std::optional<size_t> qposAdr; ///< Address for position in the state vector.
    std::optional<size_t> qvelAdr; ///< Address for velocity in the state vector.
};

/**
 * @brief Represents a scalar joint in MuJoCo.
 *
 * The `MJScalarJoint` class provides additional functionality for single
 * degree-of-freedom joints, both linear and angular. The position and velocity
 * of this joint can be set.
 *
 * If `constrainedStateInMsg` is linked, the value in this message will be read
 * and applied to an `MJSingleJointEquality` such that the joint is constrained
 * to hold a specific state. This can be used to enforce joints to follow a
 * specific path, while letting the MuJoCo engine figure out the necessary
 * force/torque to produce this path.
 */
class MJScalarJoint : public MJJoint
{
public:
    /**
     * @brief Constructs an `MJScalarJoint` with a given joint and body.
     *
     * @param joint Pointer to the MuJoCo joint.
     * @param body Reference to the body the joint is associated with.
     */
    MJScalarJoint(mjsJoint* joint, MJBody& body);

    /**
     * @brief Sets the position of the joint.
     *
     * For revolute joints, this is the angle (in radians) of the joint with respect
     * to its zero-pose.
     *
     * For linear joints, this is the displacement (in meters) of the joint with
     * respect to its zero-pose.
     *
     * @param value The desired position value.
     */
    void setPosition(double value);

    /**
     * @brief Sets the velocity of the joint.
     *
     * For revolute joints, this is the angular rate (in radians per second).
     *
     * For linear joints, this is the displacement rate (in meters per second).
     *
     * @param value The desired velocity value.
     */
    void setVelocity(double value);

    /**
     * @brief Returns the equality constraint object associated with this scalar joint.
     *
     * The returned MJSingleJointEquality can be used to enforce a specific state
     * for the joint within the MuJoCo simulation. This is typically used when the
     * joint is constrained to follow a particular position or velocity, as specified
     * by an input message or control logic.
     *
     * @return The MJSingleJointEquality object for this joint.
     */
    MJSingleJointEquality getConstrainedEquality();

    /**
     * @brief Configures the scalar joint within the given MuJoCo model.
     *
     * @param model Pointer to the MuJoCo model.
     */
    void configure(const mjModel* model);

    /**
     * @brief Updates the constrained equality associated with the joint.
     *
     * If `constrainedStateInMsg` is connected, then activates the equality
     * constraint of this joint and sets its 'offset' to the value in the message.
     */
    void updateConstrainedEquality();

    /**
     * @brief Writes the joint state message based on the current simulation state.
     *
     * @param CurrentSimNanos The current simulation time in nanoseconds.
     */
    void writeJointStateMessage(uint64_t CurrentSimNanos);

public:
    Message<ScalarJointStateMsgPayload> stateOutMsg; ///< Message to output joint position state.
    Message<ScalarJointStateMsgPayload> stateDotOutMsg; ///< Message to output joint velocity state.

    ReadFunctor<ScalarJointStateMsgPayload> constrainedStateInMsg; ///< Functor to read constrained state input.

protected:
    /** An equality used to enforce a specific state for the joint. */
    MJSingleJointEquality constrainedEquality;
};

/**
 * @brief Represents a ball joint in MuJoCo.
 *
 * Not fully supported.
 */
class MJBallJoint : public MJJoint
{
public:
    /** Use constructor from MJJoint */
    using MJJoint::MJJoint;
};

/**
 * @brief Represents a free joint in MuJoCo.
 *
 * The `MJFreeJoint` class provides additional functionality for joints with three
 * translational and three rotational degrees of freedom, including setting position,
 * velocity, attitude, and attitude rate.
 */
class MJFreeJoint : public MJJoint
{
public:
    /** Use constructor from MJJoint */
    using MJJoint::MJJoint;

    /**
     * @brief Sets the position of the free joint.
     *
     * @param position The desired 3D position vector with respect to the inertial frame.
     */
    void setPosition(const Eigen::Vector3d& position);

    /**
     * @brief Sets the translational velocity of the free joint.
     *
     * @param velocity The desired 3D velocity vector with respect to the inertial frame.
     */
    void setVelocity(const Eigen::Vector3d& velocity);

    /**
     * @brief Sets the attitude (orientation) of the free joint.
     *
     * @param attitude The orientation represented as Modified Rodrigues Parameters (MRP) with respect to the inertial frame.
     */
    void setAttitude(const Eigen::MRPd& attitude);

    /**
     * @brief Sets the attitude rate (angular velocity) of the free joint.
     *
     * @param attitudeRate The desired 3D attitude rate vector with respect to the inertial frame.
     * @todo Verify if this matches the expected attitude rate conventions with Basilisk.
     */
    void setAttitudeRate(const Eigen::Vector3d& attitudeRate);
};

#endif
