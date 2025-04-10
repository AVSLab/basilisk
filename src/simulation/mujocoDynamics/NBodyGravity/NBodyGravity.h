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

#ifndef N_BODY_GRAVITY_FORCE
#define N_BODY_GRAVITY_FORCE

#include <memory>
#include <map>
#include <string>

#include <Eigen/Dense>

#include "architecture/messaging/messaging.h"
#include "architecture/msgPayloadDefC/SpicePlanetStateMsgPayload.h"
#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/utilities/bskLogging.h"
#include "simulation/dynamics/_GeneralModuleFiles/gravityModel.h"

#include "simulation/mujocoDynamics/_GeneralModuleFiles/MJScene.h"

/**
 * @brief Represents a gravity source in an N-body simulation.
 *
 * A gravity source is a celestial body or point mass that generates a gravitational field.
 */
struct GravitySource
{
    std::shared_ptr<GravityModel> model; ///< The gravity model associated with the source.
    ReadFunctor<SpicePlanetStateMsgPayload> stateInMsg; ///< Input message providing the state of the source.
    bool isCentralBody; ///< Flag indicating whether this source is the central body.
};

/**
 * @brief Represents a gravity target in an N-body simulation.
 *
 * A gravity target is a body or site subject to gravitational forces.
 */
struct GravityTarget
{
    ReadFunctor<SCMassPropsMsgPayload> massPropertiesInMsg; ///< Input message for the target's mass properties.
    ReadFunctor<SCStatesMsgPayload> centerOfMassStateInMsg; ///< Input message for the target's state.
    Message<ForceAtSiteMsgPayload> massFixedForceOutMsg; ///< Output message for the computed force.
};

/**
 * @brief Simulates N-body gravitational interactions.
 *
 * The `NBodyGravity` class manages gravitational forces between multiple
 * sources and targets, computing the resultant forces on the targets
 * due to the gravitational fields of the sources.
 */
class NBodyGravity : public SysModel
{
public:
    /**
     * @brief Resets the N-body gravity simulation.
     *
     * Ensures all sources and targets input messages are linked,
     * that at most one central source is defined, and initializes
     * the source gravity model parameters.
     *
     * @param CurrentSimNanos The current simulation time in nanoseconds.
     * @throw std::runtime_error If there are configuration errors.
     */
    void Reset(uint64_t CurrentSimNanos);

    /**
     * @brief Updates the gravitational force of all targets.
     *
     * @param CurrentSimNanos The current simulation time in nanoseconds.
     */
    void UpdateState(uint64_t CurrentSimNanos);

    /**
     * @brief Adds a new gravity source to the simulation.
     *
     * A gravity source is a celestial body or point mass that generates a gravitational field.
     *
     * Remember to link the `stateInMsg` of the returned source.
     *
     * @param name The name of the gravity source.
     * @param gravityModel The gravity model associated with the source.
     * @param isCentralBody Indicates if this is the central body (only one per simulation).
     * @return A reference to the newly added `GravitySource`.
     * @throw std::invalid_argument If the source name is duplicated.
     */
    GravitySource& addGravitySource(std::string name, std::shared_ptr<GravityModel> gravityModel, bool isCentralBody);

    /**
     * @brief Adds a new gravity target to the simulation.
     *
     * A gravity target is a body or site subject to gravitational forces.
     *
     * Remember to link the `massPropertiesInMsg` and `centerOfMassStateInMsg`
     * messages of the returned target.
     *
     * @param name The name of the gravity target.
     * @return A reference to the newly added `GravityTarget`.
     * @throw std::invalid_argument If the target name is duplicated.
     */
    GravityTarget& addGravityTarget(std::string name);

    /**
     * @brief Adds a gravity target tied to a specific site.
     *
     * The gravity field is computed at the site's position/orientation,
     * and the gravity force/torque is applied at a newly created
     * actuator on this site.
     *
     * Remember to tie the `massPropertiesInMsg` message of the returned target.
     *
     * @param name The name of the gravity target.
     * @param site The site associated with the target.
     * @return A reference to the newly added `GravityTarget`.
     */
    GravityTarget& addGravityTarget(std::string name, MJSite& site);

    /**
     * @brief Adds a gravity target tied to a specific body.
     *
     * The gravity field is computed at the body's center mass,
     * and the gravity force/torque is applied at a newly created
     * actuator on this site.
     *
     * @param name The name of the gravity target.
     * @param body The body associated with the target.
     * @return A reference to the newly added `GravityTarget`.
     */
    GravityTarget& addGravityTarget(std::string name, MJBody& body);

    /**
     * @brief Retrieves a gravity source by name.
     *
     * @param name The name of the gravity source.
     * @return A reference to the `GravitySource`.
     * @throw std::out_of_range If the source does not exist.
     */
    GravitySource& getGravitySource(std::string name) { return sources.at(name); }

    /**
     * @brief Retrieves a gravity target by name.
     *
     * @param name The name of the gravity target.
     * @return A reference to the `GravityTarget`.
     * @throw std::out_of_range If the target does not exist.
     */
    GravityTarget& getGravityTarget(std::string name) { return targets.at(name); }

    /**
     * @brief Computes the gravitational acceleration from a source at a position.
     *
     * @param source The gravity source.
     * @param r_J2000 The position vector in the J2000 frame.
     * @return The gravitational acceleration vector.
     */
    Eigen::Vector3d computeAccelerationFromSource(GravitySource& source, Eigen::Vector3d r_J2000);

    /**
     * @brief Computes the total gravitational acceleration acting on a target.
     *
     * @param target The gravity target.
     * @return The gravitational acceleration vector.
     */
    Eigen::Vector3d computeAccelerationOnTarget(GravityTarget& target);

public:
    BSKLogger bskLogger; ///< BSK Logging

protected:
    // NOTE: choosing map over unordered_map so that computations
    // are run in a consistent order

    std::map<std::string, GravitySource> sources; ///< Map of gravity sources by name.
    std::map<std::string, GravityTarget> targets; ///< Map of gravity targets by name.
};

#endif
