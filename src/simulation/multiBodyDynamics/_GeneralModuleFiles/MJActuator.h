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

#ifndef MJACTUATOR_H
#define MJACTUATOR_H

#include <map>
#include <memory>
#include <vector>

#include <mujoco/mujoco.h>
#include "architecture/messaging/messaging.h"
#include "architecture/msgPayloadDefC/ForceAtSiteMsgPayload.h"
#include "architecture/msgPayloadDefC/SingleActuatorMsgPayload.h"
#include "architecture/msgPayloadDefC/TorqueAtSiteMsgPayload.h"
#include "MJObject.h"

/// @cond
/**
 * @brief Specialization of getObjectTypeName for mjsActuator.
 *
 * Provides a human-readable name for the MuJoCo actuator type.
 *
 * @return A string view representing the actuator type.
 */
template <>
constexpr std::string_view MJBasilisk::detail::getObjectTypeName<mjsActuator>()
{
    return "actuator";
}
/// @endcond

/**
 * @brief A class representing a MuJoCo actuator object.
 *
 * MuJoCo actuators, and thus also `MJActuatorObject`, are single-input
 * single-output. This means that the force/torque produced by the
 * actuator is determined by a single degree of freedom. However, in
 * simulation we usually want to control force and torque along multiple
 * dimensions. Thus, we use `MJActuator`, which is a collection of
 * `MJActuatorObject`.
 *
 * This class wraps a MuJoCo actuator object and provides functionality to
 * update its control value.
 */
class MJActuatorObject : public MJObject<mjsActuator>
{
public:
    /** Use the same constructor as MJObject */
    using MJObject<mjsActuator>::MJObject;

    /**
     * @brief Updates the control value of the actuator in the simulation data.
     *
     * @param data Pointer to the MuJoCo simulation data.
     * @param value The control value to set.
     */
    void updateCtrl(mjData* data, double value);
};

/**
 * @brief Base class representing a MuJoCo actuator.
 *
 * This class represents a general actuator composed of one or more
 * `MJActuatorObject` instances. It provides methods for configuring the
 * actuator and updating its control values.
 *
 * `MJActuatorObject` and MuJoCo actuators are single-input single-output,
 * which means that they can be controlled with only one degree of freedom.
 * For convenience, subclasses of `MJActuator` represent a useful collection
 * of these 1-dof actuators. See `MJSingleActuator`, `MJForceActuator`,
 * `MJTorqueActuator`, and `MJForceTorqueActuator`.
 */
class MJActuator
{
public:
    /**
     * @brief Constructs an MJActuator with a specified name and actuator objects.
     *
     * @param name The name of the actuator.
     * @param subActuators A vector of `MJActuatorObject` representing
     * the individual actuator parts.
     */
    MJActuator(std::string name, std::vector<MJActuatorObject>&& subActuators)
        : name(std::move(name)), subActuators(subActuators) {}

    // Delete copy and move constructors and assignment operators
    MJActuator(const MJActuator&) = delete;
    MJActuator(MJActuator&&) = delete;
    MJActuator& operator=(const MJActuator&) = delete;
    MJActuator& operator=(MJActuator&&) = delete;

    /**
     * @brief Virtual destructor for MJActuator.
     */
    virtual ~MJActuator() {}

    /**
     * @brief Gets the name of the actuator.
     *
     * @return The name of the actuator.
     */
    const std::string& getName() { return name; }

    /**
     * @brief Configures the actuator within a given MuJoCo model.
     *
     * @param model Pointer to the MuJoCo model used for configuration.
     */
    void configure(const mjModel* model);

    /**
     * @brief Updates the control value of the actuator(s) in the MuJoCo data.
     *
     * @param data Pointer to the MuJoCo simulation data.
     */
    void updateCtrl(mjData* data);

    /**
     * @brief Reads control messages for the actuator.
     *
     * This pure virtual function must be implemented by derived classes
     * to read specific control messages.
     *
     * @return A vector of control values.
     */
    virtual std::vector<double> readControlMessages() = 0;

protected:
    std::string name; ///< The name of the actuator.
    std::vector<MJActuatorObject> subActuators; ///< The individual actuator objects.
};

/**
 * @brief Class representing a 1-degree-of-freedom actuator.
 *
 * This actuator type has 1 control degree of freedom. This can be useful
 * to represent forces applied to linear joints, torques applied to revolute
 * joints, or forces/torques applied along a fixed direction.
 *
 * This class is designed to handle one `SingleActuatorMsgPayload` message to
 * control the force / torque magnitude applied.
 */
class MJSingleActuator : public MJActuator
{
public:
    /** Use the same constructor as MJActuator */
    using MJActuator::MJActuator;

    /**
     * @brief Reads control messages for a single actuator.
     *
     * @return A vector containing the control value.
     */
    std::vector<double> readControlMessages() override;

    ReadFunctor<SingleActuatorMsgPayload> actuatorInMsg; ///< Functor to read actuator control messages.
};

/**
 * @brief Class representing an actuator that can exert an arbitrary force vector.
 *
 * This actuator has 3 degrees of freedom. To achieve this, it's composed of
 * three 1-degree-of-freedom `MJActuatorObject`, each
 * applying a force along perpendicular directions.
 *
 * This class is designed to handle one `ForceAtSiteMsgPayload` message to
 * control the force vector applied.
 */
class MJForceActuator : public MJActuator
{
public:
    /** Use the same constructor as MJActuator */
    using MJActuator::MJActuator;

    /**
     * @brief Reads force control messages for the actuator.
     *
     * @return A vector containing the force control values.
     */
    std::vector<double> readControlMessages() override;

    ReadFunctor<ForceAtSiteMsgPayload> forceInMsg; ///< Functor to read force control messages.
};

/**
 * @brief Class representing an actuator that can exert an arbitrary torque vector.
 *
 * This actuator has 3 degrees of freedom. To achieve this, it's composed of
 * three 1-degree-of-freedom `MJActuatorObject`, each
 * applying a torque along perpendicular directions.
 *
 * This class is designed to handle one `TorqueAtSiteMsgPayload` message to
 * control the torque vector applied.
 */
class MJTorqueActuator : public MJActuator
{
public:
    /** Use the same constructor as MJActuator */
    using MJActuator::MJActuator;

    /**
     * @brief Reads torque control messages for the actuator.
     *
     * @return A vector containing the torque control values.
     */
    std::vector<double> readControlMessages() override;

    ReadFunctor<TorqueAtSiteMsgPayload> torqueInMsg; ///< Functor to read torque control messages.
};

/**
 * @brief Class representing an actuator that can exert an arbitrary force and torque vectors.
 *
 * This actuator has 6 degrees of freedom. To achieve this, it's composed of
 * size 1-degree-of-freedom `MJActuatorObject`, each
 * applying a force and torque along perpendicular directions.
 *
 * This class is designed to handle one `ForceAtSiteMsgPayload` and one `TorqueAtSiteMsgPayload`
 * messages to control the force and torque vectors applied.
 */
class MJForceTorqueActuator : public MJActuator
{
public:
    /** Use the same constructor as MJActuator */
    using MJActuator::MJActuator;

    /**
     * @brief Reads force and torque control messages for the actuator.
     *
     * @return A vector containing the force and torque control values.
     */
    std::vector<double> readControlMessages() override;

    ReadFunctor<ForceAtSiteMsgPayload> forceInMsg; ///< Functor to read force control messages.
    ReadFunctor<TorqueAtSiteMsgPayload> torqueInMsg; ///< Functor to read torque control messages.
};

#endif
