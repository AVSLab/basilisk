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

#ifndef MJSPEC_H
#define MJSPEC_H

#include <mujoco/mujoco.h>

#include <list>
#include <memory>
#include <type_traits>

#include "MJActuator.h"
#include "MJBody.h"
#include "MJUtils.h"

namespace Eigen
{
using Vector6d = Eigen::Matrix<double, 6, 1>;
}

class MJScene;

/**
 * @brief A wrapper for a MuJoCo `mjSpec`, which represents a high-level model specification.
 *
 * The `MJSpec` class manages a pair of MuJoCo models and data objects, for which it supports
 * recompilation.
 *
 * This class keeps tracks of Basilisk wrappers for the MuJoCo bodies, actuators, and
 * equalities of an `MJScene`.
 */
class MJSpec
{
public:
    /**
     * @brief Guard to temporarily disable model recompilation.
     *
     * Disables automatic recompilation of the MuJoCo model during its lifetime.
     * Can be used when one wants to take one or several actions that would normally
     * trigger a model recompilation, but for efficiency recompilation is supressed
     * until a later moment.
     */
    struct NoRecompileGuard {
        /**
         * @brief Constructs the guard and disables recompilation.
         *
         * @param spec Reference to the `MJSpec` object.
         */
        NoRecompileGuard(MJSpec& spec)
            : spec(spec), prevShouldRecompileWhenAsked(spec.shouldRecompileWhenAsked)
        {
            spec.shouldRecompileWhenAsked = false;
        }

        /**
         * @brief Restores the previous recompilation flag state.
         */
        ~NoRecompileGuard() { spec.shouldRecompileWhenAsked = prevShouldRecompileWhenAsked; }

        MJSpec& spec; ///< Reference to the `MJSpec` object.
        bool prevShouldRecompileWhenAsked; ///< Previous state of recompilation flag.
    };

    /**
     * @brief Constructs an `MJSpec` object.
     *
     * @param scene Reference to the `MJScene` managing this spec.
     * @param xmlString The XML string defining the MuJoCo model.
     * @param files Optional list of additional files to load.
     */
    MJSpec(MJScene& scene, std::string xmlString, const std::vector<std::string>& files = {});

    // Delete copy and move operators
    MJSpec(const MJSpec&) = delete;
    MJSpec(MJSpec&&) = delete;
    MJSpec& operator=(const MJSpec&) = delete;
    MJSpec& operator=(MJSpec&&) = delete;

    /**
     * @brief Retrieves the scene that this model specification represents.
     *
     * @return Reference to the `MJScene`.
     */
    MJScene& getScene() { return this->scene; }

    /**
     * @brief Declares that the `mjModel` should recompiled because
     * we performed an action that rendered the current `mjModel` stale.
     */
    void markAsNeedingToRecompileModel() { this->shouldRecompile = true; }

    /**
     * @brief Retrieves the MuJoCo model (`mjModel` object).
     *
     * @return Pointer to the MuJoCo model.
     */
    mjModel* getMujocoModel();

    /**
     * @brief Retrieves the MuJoCo data (`mjData` object).
     *
     * @return Pointer to the MuJoCo data.
     */
    mjData* getMujocoData();

    /**
     * @brief Retrieves the raw MuJoCo model specification (`mjSpec` object).
     *
     * @return Pointer to the MuJoCo specification.
     */
    mjSpec* getMujocoSpec() { return this->spec.get(); }

    /**
     * @brief Recompiles the model if needed.
     *
     * @return True if recompilation occurred, false otherwise.
     */
    bool recompileIfNeeded();

    /**
     * @brief Configures all bodies, actuators, and equalities with
     * the current model.
     */
    void configure();

    /**
     * @brief Retrieves the list of all bodies in the simulation.
     *
     * @return Reference to the list of `MJBody` objects.
     */
    std::list<MJBody>& getBodies() { return bodies; }

    /**
     * @brief Retrieves the list of all actuators in the simulation.
     *
     * @return Reference to the vector of unique pointers to `MJActuator` objects.
     */
    std::vector<std::unique_ptr<MJActuator>>& getActuators() { return actuators; }

    /**
     * @brief Retrieves the list of equalities in the simulation.
     *
     * Note that this not include constraint equalities automatically
     * created for all scalar joints in the simulation.
     *
     * @return Reference to the list of `MJEquality` objects.
     */
    std::list<MJEquality>& getEqualities() { return equalities; }

    /**
     * @brief Checks if an actuator exists with the given name.
     *
     * @param name The name of the actuator.
     * @return True if the actuator exists, false otherwise.
     */
    bool hasActuator(const std::string& name);

    /**
     * @brief Retrieves an actuator by name and casts it to the specified type.
     *
     * @tparam T The expected type of the actuator.
     * @param name The name of the actuator.
     * @return Reference to the actuator of type `T`.
     * @throws std::invalid_argument If the actuator does not exist or has a different type.
     */
    template <typename T> T& getActuator(const std::string& name);

    /**
     * @brief Adds a single-input actuator acting on a joint (`MJSingleActuator`).
     *
     * This actuator will impart a force and/or torque at the given joint.
     *
     * @param name The name of the actuator.
     * @param joint The joint to attach the actuator.
     * @return Reference to the created `MJSingleActuator`.
     */
    MJSingleActuator& addJointSingleActuator(const std::string& name,
                                        const std::string& joint);

    /**
     * @brief Adds a single-input actuator (`MJSingleActuator`).
     *
     * This actuator will impart a force and/or torque at the given site.
     *
     * The `gear` argument defines how the single input of the actuator
     * (the control value of this actuator is a single value) maps into
     * a force and/or torque. For example:
     *
     *  - For `gear={1,0,0,0,0,0}` and `ctrl=10`, the actuator will impart
     *    10 N of force along the X-axis of the `site` reference frame.
     *  - For `gear={1,0,0,0,1,0}` and `ctrl=10`, the actuator will impart
     *    10 N of force along the X-axis of the `site` reference frame and
     *    10 Nm of torque along the Y-axis of the same reference frame.
     *
     * @param name The name of the actuator.
     * @param site The site to attach the actuator.
     * @param gear The gear parameters for the actuator.
     * @return Reference to the created `MJSingleActuator`.
     */
    MJSingleActuator& addSingleActuator(const std::string& name,
                                        const std::string& site,
                                        const Eigen::Vector6d& gear);

    /**
     * @brief Adds a composite actuator to the specification.
     *
     * Composite actuators are those composed by more than one
     * single actuator.
     *
     * @tparam T The type of composite actuator to add. Supported types
     *   are `MJForceActuator`, `MJTorqueActuator`, `MJForceTorqueActuator`.
     * @param name The name of the actuator.
     * @param site The site to attach the actuator.
     * @return Reference to the created actuator of type `T`.
     */
    template <typename T> T& addCompositeActuator(const std::string& name, const std::string& site);

protected:
    std::unique_ptr<mjSpec, MJBasilisk::detail::mjSpecDeleter> spec; ///< MuJoCo spec.
    std::unique_ptr<mjModel, MJBasilisk::detail::mjModelDeleter> model; ///< MuJoCo model.
    std::unique_ptr<mjData, MJBasilisk::detail::mjDataDeleter> data; ///< MuJoCo data.
    std::unique_ptr<mjVFS, MJBasilisk::detail::mjVFSDeleter> virtualFileSystem; ///< MuJoCo virtual file system.

    MJScene& scene; ///< Reference to the associated scene.
    std::vector<std::unique_ptr<MJActuator>> actuators; ///< List of actuators.
    std::list<MJBody> bodies; ///< List of bodies.
    std::list<MJEquality> equalities; ///< List of equalities.

    /**
     * A number of operations on the mjSpec (like adding elements)
     * will require that we recompile the mjModel. If this flag is
     * true when getMujocoModel()/getMujocoData() is called, then
     * they will be recompiled before returning. However, also read
     * the docstring for shouldRecompileWhenAsked.
     */
    bool shouldRecompile = false;

    /**
     * If we are going to make a series of operations that should
     * trigger a recompile, but we want to avoid having to recompile
     * until they are all done, then we can switch this flag, which
     * will prevent any recompilation when calling
     * getMujocoModel()/getMujocoData().
     */
    bool shouldRecompileWhenAsked = true;

protected:
    void loadBodies(); ///< Load bodies in spec into MJBody
    void loadActuators(); ///< Load actuators in spec into MJActuator
    void loadEqualities(); ///< Load equalities in spec into MJEquality

protected:
    /**
     * Creates an actuator of type `T`, where `T` must be one of: `MJForceActuator`,
     * `MJTorqueActuator`, or `MJForceTorqueActuator`.
     *
     * The actuator will be composed of a collection of the `MJActuatorObject`
     * given in `existingActuators`, if they are available. Otherwise, new `MJActuatorObject`
     * will be created at the given site.
     *
     * @tparam T The type of actuator to add. Supported types
     *   are `MJForceActuator`, `MJTorqueActuator`, `MJForceTorqueActuator`.
     * @param name The name of the actuator.
     * @param siteHint The site to attach the actuator.
     * @param existingActuators Colleciton of existing `MJActuatorObject` to use.
     * @return Reference to the created actuator of type `T`.
    */
    template <typename T>
    std::unique_ptr<T>
    createActuator(const std::string& name,
                   const std::string& siteHint,
                   std::unordered_map<std::string, MJActuatorObject>& existingActuators);
};

// Implementation of templated methods

template <typename T> T& MJSpec::getActuator(const std::string& name)
{
    auto actuatorPtr = std::find_if(std::begin(actuators), std::end(actuators), [&](auto&& obj) {
        return obj->getName() == name;
    });

    if (actuatorPtr == std::end(actuators))
        MJBasilisk::detail::logAndThrow("Tried to get actuator '" + name +
                                        "' but it does not exist.");

    auto casted = dynamic_cast<T*>(actuatorPtr->get());
    if (!casted)
        MJBasilisk::detail::logAndThrow("Tried to get actuator '" + name +
                                        "', but it is not of the expected type.");

    return *casted;
}

template<class> constexpr bool dependent_false = false;

template <typename T>
std::unique_ptr<T>
MJSpec::createActuator(const std::string& name,
                       const std::string& siteHint,
                       std::unordered_map<std::string, MJActuatorObject>& existingActuators)
{
        // Composite actuator types require multiple mujoco actuators
    // with different gears
    std::vector<std::pair<std::string, size_t>> subactuatorNameAndGears;
    if constexpr (std::is_same_v<T, MJForceActuator>) {
        subactuatorNameAndGears = {
            {name + "_fx", 0},
            {name + "_fy", 1},
            {name + "_fz", 2},
        };
    } else if constexpr (std::is_same_v<T, MJTorqueActuator>) {
        subactuatorNameAndGears = {
            {name + "_mx", 3},
            {name + "_my", 4},
            {name + "_mz", 5},
        };
    } else if constexpr (std::is_same_v<T, MJForceTorqueActuator>) {
        subactuatorNameAndGears = {
            {name + "_fx", 0},
            {name + "_fy", 1},
            {name + "_fz", 2},
            {name + "_mx", 3},
            {name + "_my", 4},
            {name + "_mz", 5},
        };
    } else {
        // I'd like to do static_assert(false, ...) but not all compilers support it
        static_assert(dependent_false<T>, "Unknown actuator type");
    }
    std::vector<MJActuatorObject> subActuatorObjects;

    for (auto&& [subActuatorName, gearInd] : subactuatorNameAndGears) {
                 // The object already exists! We can just reuse it
        if (existingActuators.count(subActuatorName)) {
            subActuatorObjects.emplace_back(std::move(existingActuators.at(subActuatorName)));
            existingActuators.erase(subActuatorName);
        }
        // We need to generate a new subactuator name, but we can only do so if
        // siteHint was given
        else if (siteHint.empty()) {
            MJBasilisk::detail::logAndThrow("Tried to autogenerate mujoco actuator for actuator " +
                                            name + ", but no 'site' was provided.");
        }
        // We need to create the mjs element with the given hint and gear
        else {
            auto newMjsActuator = mjs_addActuator(this->spec.get(), 0);
            newMjsActuator->trntype = mjTRN_SITE;
            mjs_setString(newMjsActuator->name, subActuatorName.c_str());
            mjs_setString(newMjsActuator->target, siteHint.c_str());
            newMjsActuator->gear[0] = 0;
            newMjsActuator->gear[gearInd] = 1;
            newMjsActuator->dyntype = mjDYN_NONE;
            newMjsActuator->gaintype = mjGAIN_FIXED;
            newMjsActuator->biastype = mjBIAS_NONE;
            newMjsActuator->gainprm[0] = 1;
            subActuatorObjects.emplace_back(newMjsActuator);
        }
    }
    return std::make_unique<T>(name, std::move(subActuatorObjects));
}

template <typename T>
inline T& MJSpec::addCompositeActuator(const std::string& name, const std::string& site)
{
    if ((this->hasActuator(name)))
        MJBasilisk::detail::logAndThrow("Tried to add actuator with name '" + name +
                                        "' but one already exists with that name.");

    std::unordered_map<std::string, MJActuatorObject> existingActuators;
    this->actuators.emplace_back(createActuator<T>(name, site, existingActuators));

    return getActuator<T>(name);
}

#endif
