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

#ifndef MJSCENE_H
#define MJSCENE_H

#include <memory>
#include <stdexcept>
#include <mujoco/mujoco.h>

#include "MJSpec.h"
#include "MJUtils.h"
#include "architecture/_GeneralModuleFiles/sys_model_task.h"
#include "architecture/msgPayloadDefCpp/MJSceneStateMsgPayload.h"
#include "MJQPosStateData.h"
#include "simulation/dynamics/_GeneralModuleFiles/dynamicObject.h"

/**
 * @brief Represents a dynamic object that solves multi-body dynamics through MuJoCo.
 *
 * The `MJScene` class defines a multi-body system where bodies are connected
 * by joints, allowing forces and torques to act at specific points or joints.
 * The state of the system is advanced in time through integration, with forward
 * kinematics transforming joint states into body positions and orientations.
 */
class MJScene : public DynamicObject
{
public:
    /**
     * @brief Constructs an MJScene object using an XML string.
     *
     * For a detailed reference on the XML format, see:
     * https://mujoco.readthedocs.io/en/stable/XMLreference.html
     *
     * @param xmlString The XML string defining the MuJoCo model.
     * @param files Optional additional files for the model.
     */
    MJScene(std::string xmlString, const std::vector<std::string>& files = {});

    /**
     * @brief Creates an MJScene object from a file.
     *
     * The file's content are read, then passed to the constructor
     * of this class.
     *
     * @param fileName The name of the file containing the MuJoCo XML.
     * @return An MJScene object.
     */
    static MJScene fromFile(const std::string& fileName);

    // Deleted copy and move operations
    MJScene(const MJScene&) = delete;
    MJScene(MJScene&&) = delete;
    MJScene& operator=(const MJScene&) = delete;
    MJScene& operator=(MJScene&&) = delete;

    /**
     * @brief Retrieves a body by name.
     *
     * @param name The name of the body.
     * @return A reference to the `MJBody`.
     * @throw std::invalid_argument If the body does not exist.
     */
    MJBody& getBody(const std::string& name);

    /**
     * @brief Retrieves a site by name.
     *
     * @param name The name of the site.
     * @return A reference to the `MJSite`.
     * @throw std::invalid_argument If the site does not exist.
     */
    MJSite& getSite(const std::string& name);

    /**
     * @brief Retrieves an equality constraint by name.
     *
     * @param name The name of the equality constraint.
     * @return A reference to the `MJEquality`.
     * @throw std::invalid_argument If the equality does not exist.
     */
    MJEquality& getEquality(const std::string& name);

    /**
     * @brief Retrieves a single-input actuator by name.
     *
     * See the documentation on `MJSingleActuator` for details.
     *
     * @param name The name of the actuator.
     * @return A reference to the `MJSingleActuator`.
     */
    MJSingleActuator& getSingleActuator(const std::string& name);

    /**
     * @brief Retrieves a force actuator by name.
     *
     * See the documentation on `MJForceActuator` for details.
     *
     * @param name The name of the actuator.
     * @return A reference to the `MJForceActuator`.
     */
    MJForceActuator& getForceActuator(const std::string& name);

    /**
     * @brief Retrieves a torque actuator by name.
     *
     * See the documentation on `MJTorqueActuator` for details.
     *
     * @param name The name of the actuator.
     * @return A reference to the `MJTorqueActuator`.
     */
    MJTorqueActuator& getTorqueActuator(const std::string& name);

    /**
     * @brief Retrieves a force-torque actuator by name.
     *
     * See the documentation on `MJForceTorqueActuator` for details.
     *
     * @param name The name of the actuator.
     * @return A reference to the `MJForceTorqueActuator`.
     */
    MJForceTorqueActuator& getForceTorqueActuator(const std::string& name);

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
     * @brief Adds a single-input actuator acting on a joint (`MJSingleActuator`).
     *
     * This actuator will impart a force and/or torque at the given joint.
     *
     * @param name The name of the actuator.
     * @param joint The joint to attach the actuator.
     * @return Reference to the created `MJSingleActuator`.
     */
    MJSingleActuator& addJointSingleActuator(const std::string& name,
                                        const MJJoint& joint);

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
                                        const MJSite& site,
                                        const Eigen::Vector6d& gear);

    /**
     * @brief Adds a force actuator to the scene.
     *
     * This actuator can impart an arbitrary force vector at
     * the given site.
     *
     * This actuator takes three input controls: the force
     * magnitude along each axis of the site's reference frame
     * (`[f_x, f_y, f_z]`).
     *
     * @param name The name of the actuator.
     * @param site The site associated with the actuator.
     * @return A reference to the newly added `MJForceActuator`.
     */
    MJForceActuator& addForceActuator(const std::string& name, const std::string& site);

    /**
     * @brief Adds a force actuator to the scene.
     *
     * This actuator can impart an arbitrary force vector at
     * the given site.
     *
     * This actuator takes three input controls: the force
     * magnitude along each axis of the site's reference frame
     * (`[f_x, f_y, f_z]`).
     *
     * @param name The name of the actuator.
     * @param site The site associated with the actuator.
     * @return A reference to the newly added `MJForceActuator`.
     */
    MJForceActuator& addForceActuator(const std::string& name, const MJSite& site);

    /**
     * @brief Adds a torque actuator to the scene.
     *
     * This actuator can impart an arbitrary torque vector at
     * the given site.
     *
     * This actuator takes three input controls: the torque
     * magnitude along each axis of the site's reference frame
     * (`[t_x, t_y, t_z]`).
     *
     * @param name The name of the actuator.
     * @param site The site associated with the actuator.
     * @return A reference to the newly added `MJTorqueActuator`.
     */
    MJTorqueActuator& addTorqueActuator(const std::string& name, const std::string& site);

    /**
     * @brief Adds a torque actuator to the scene.
     *
     * This actuator can impart an arbitrary torque vector at
     * the given site.
     *
     * This actuator takes three input controls: the torque
     * magnitude along each axis of the site's reference frame
     * (`[t_x, t_y, t_z]`).
     *
     * @param name The name of the actuator.
     * @param site The site associated with the actuator.
     * @return A reference to the newly added `MJTorqueActuator`.
     */
    MJTorqueActuator& addTorqueActuator(const std::string& name, const MJSite& site);

    /**
     * @brief Adds a force-torque actuator to the scene.
     *
     * This actuator can impart an arbitrary force and torque
     * vectors at the given site.
     *
     * This actuator takes six input controls: the force and
     * torque magnitude along each axis of the site's reference
     * frame (`[f_x, f_y, f_z, t_x, t_y, t_z]`).
     *
     * @param name The name of the actuator.
     * @param site The site associated with the actuator.
     * @return A reference to the newly added `MJForceTorqueActuator`.
     */
    MJForceTorqueActuator& addForceTorqueActuator(const std::string& name, const std::string& site);

    /**
     * @brief Adds a force-torque actuator to the scene.
     *
     * This actuator can impart an arbitrary force and torque
     * vectors at the given site.
     *
     * This actuator takes six input controls: the force and
     * torque magnitude along each axis of the site's reference
     * frame (`[f_x, f_y, f_z, t_x, t_y, t_z]`).
     *
     * @param name The name of the actuator.
     * @param site The site associated with the actuator.
     * @return A reference to the newly added `MJForceTorqueActuator`.
     */
    MJForceTorqueActuator& addForceTorqueActuator(const std::string& name, const MJSite& site);

    /**
     * @brief Adds a model to the dynamics task.
     *
     * The dynamics task of this dynamical object is called once
     * per integrator sub-step (so, it will be called 4 times if
     * using the RK4 integrator, for example). Models in the dynamics
     * task typically compute and apply forces and torques on the system
     * that depend on the simulation time or the state of the multi-body.
     *
     * @warning `SysModel` added to the dynamics task should be memoryless.
     * That is, any output message computed should depend strictly on
     * input messages and the current time/states. It should save values
     * internally with the expectation of using them in further calls
     * to `UpdateState`. The reason for this is that `UpdateState` may
     * be called at arbitrary simulation times, and not necessarily in
     * order or with consistent time steps.
     *
     * @param model The system model to add.
     * @param priority The priority of the model in the task list.
     */
    void AddModelToDynamicsTask(SysModel* model, int32_t priority = -1);

    /**
     * @brief Adds forward kinematics to the dynamics task.
     *
     * The state of the multi-body is given by the state of its joints.
     * However, we are usually interested in positions/velocities of
     * sites. Forward kinematics is the process through which joint
     * states are transformed to site position/velocities. A forward
     * dynamics calculation is done by a special 'forward-kinematics'
     * model.
     *
     * The dynamics task of an `MJScene` comes preloaded with a 'forward-kinematics'
     * model at priority `MJScene::FWD_KINEMATICS_PRIORITY` (100). This high
     * tolerance means that, usually, whatever joint state is present at the
     * start of the dynamics task, it will be transformed into site position/velocity
     * such that subsequent models can query this information.
     *
     * Advance usage of this class might update the joint state of the
     * multi-body midway though its dynamics task. For example, this can
     * be used to simulate joints with prescribed states (not implemented
     * at the moment). In this case, one would have to add an additional
     * forward-kinematics models such that this updated joint state
     * is transformed into site position/velocity information and thus
     * subsequent models query the correct information.
     *
     * A safe practice is to always add forward-kinematics models after
     * updating the joint state.
     *
     * @param priority The priority of the forward kinematics in the task list.
     */
    void AddFwdKinematicsToDynamicsTask(int32_t priority);

    /**
     * @brief Calls `SelfInit` on all system models in the dynamics task.
     */
    void SelfInit() override;

    /**
     * @brief Calls `Reset` on all system models in the dynamics task and
     * calls `initializeDynamics`.
     *
     * @param CurrentSimNanos The current simulation time in nanoseconds.
     */
    void Reset(uint64_t CurrentSimNanos) override;

    /**
     * @brief Registers the dynamic states and recompiles the MuJoCo model.
     */
    void initializeDynamics() override;

    /**
     * @brief Integrates the dynamics up to the given time and writes output messages.
     *
     * @param callTime The current simulation time.
     */
    void UpdateState(uint64_t callTime) override;

    /**
     * @brief Computes the equations of motion for the system.
     *
     * This evaluates the dynamics task for the given time, which should
     * update the control values of actuators, the positions of constrained
     * joints... These information is translated, through MuJoCo, into
     * first derivatives of the joint states and the mass of bodies.
     *
     * @param t The current simulation time in seconds.
     * @param timeStep The integration time step.
     */
    void equationsOfMotion(double t, double timeStep) override;

    /**
     * @brief Prepares the system before actual integration.
     *
     * @param callTimeNanos The current simulation time.
     */
    void preIntegration(uint64_t callTimeNanos) override;

    /**
     * @brief Finalizes the system after integration.
     *
     * This performs forward kinematics on the final integrated
     * states. It may also perform an extra call to the equations
     * of motion if the `extraEoMCall` flag is `true`. This
     * can be useful if users want to record simulation values
     * that are computed during the dynamics task depending
     * on the final state.
     *
     * @param callTimeNanos The current simulation time.
     */
    void postIntegration(uint64_t callTimeNanos) override;

    /**
     * @brief Marks the model as needing recompilation.
     *
     * This should be called everytime one performs an action that
     * renders the existing MuJoCo model stale, such as when adding/deleting
     * bodies/sites/actuators...
     */
    void markAsNeedingToRecompileModel() { spec.markAsNeedingToRecompileModel(); }

    /**
     * @brief Checks if MuJoCo model constants are stale.
     *
     * MuJoCo model constants are values that are computed once
     * and cached on the MuJoCo model from other 'constant' values
     * input by the user. In MuJoCo, the mass of bodies is a 'constant',
     * thus updating it requires us to recompute the dependent 'constants'.
     *
     * @return True if the model constants are stale.
     */
    bool areMujocoModelConstStale() { return mjModelConstStale; }

    /**
     * @brief Marks MuJoCo model constants as stale.
     *
     * Should be called everytime the mass of a body is updated.
     *
     * See `areMujocoModelConstStale`.
     */
    void markMujocoModelConstAsStale() { mjModelConstStale = true; }

    /**
     * @brief Checks if forward kinematics are stale.
     *
     * Kinematics (the position/velocity of bodies and sites) are
     * stale when the joint state of the muli-body changes (since
     * the former depends on the latter).
     *
     * If kinematics are stale, this means that the current values
     * in the site position/velocity are not up to date with the
     * current joint states.
     *
     * @return True if forward kinematics are stale.
     */
    bool areKinematicsStale() { return forwardKinematicsStale; }

    /**
     * @brief Marks forward kinematics as stale.
     *
     * Should be called every time the position or velocity
     * of a joint is updated.
     */
    void markKinematicsAsStale() { forwardKinematicsStale = true; }

    /**
     * @brief Writes forward kinematics output messages.
     *
     * Writes the site state output messages with the current
     * values stored in the MuJoCo data.
     *
     * @warning This does NOT update the MuJoCo data itself, so
     * the written messages may still be stale! If you want to
     * forward the kinematics AND write the messages, call `MJFwdKinematics::fwdKinematics`
     *
     * @param CurrentSimNanos The current simulation time in nanoseconds.
     */
    void writeFwdKinematicsMessages(uint64_t CurrentSimNanos);

    /**
     * @brief Saves the current MuJoCo model to a file.
     *
     * @param filename The name of the file to save the model.
     */
    void saveToFile(std::string filename);

    /**
     * @brief Retrieves the MuJoCo model.
     *
     * @return Pointer to the MuJoCo model.
     */
    mjModel* getMujocoModel() { return this->spec.getMujocoModel(); }

    /**
     * @brief Retrieves the MuJoCo data.
     *
     * @return Pointer to the MuJoCo data.
     */
    mjData* getMujocoData() { return this->spec.getMujocoData(); }

    /**
     * @brief Retrieves the position state data.
     *
     * @return Pointer to `MJQPosStateData`.
     * @throw std::runtime_error If the state has not been initialized.
     */
    MJQPosStateData* getQposState();

    /**
     * @brief Retrieves the velocity state data.
     *
     * @return Pointer to `StateData`.
     * @throw std::runtime_error If the state has not been initialized.
     */
    StateData* getQvelState();

    /**
     * @brief Retrieves the actuation state data.
     *
     * @return Pointer to `StateData`.
     * @throw std::runtime_error If the state has not been initialized.
     */
    StateData* getActState();

    /**
     * @brief Prints MuJoCo model debug information to a file.
     *
     * @param path The file path for debug output.
     */
    void printMujocoModelDebugInfo(const std::string& path);

    /**
     * @brief Logs an error message on the BKLogger and throws an exception.
     *
     * @tparam T The type of the exception to throw.
     * @param error The error message.
     */
    template <typename T = std::invalid_argument>
    [[noreturn]] void logAndThrow (const std::string& error);

public:
    static const int FWD_KINEMATICS_PRIORITY = 10000; ///< Priority for default forward kinematics model.

    /** @brief Flag to run the equations of motion after integration.
     *
     * If `true`, calls `equationsOfMotion` once after integration has
     * finished, thus using the appropriate states. This won't impact
     * the dynamics of the object. However, it will re-compute any
     * messages output by the dynamics task with the final, proper,
     * state, which can be useful if users want to record messages
     * output by the models in this task. It will also recompute the
     * derivative of the states, which again can be useful if recording
     * accelerations.
     */
    bool extraEoMCall = false;

    Message<MJSceneStateMsgPayload> stateOutMsg; ///< Message with all the the scene's position, velocity, and actuators states.

protected:
    /**
     * @brief Updates MuJoCo structs from the Basilisk `StateData` objects.
     */
    void updateMujocoArraysFromStates();

    /**
     * @brief Writes the values of the position, velocity, and actuators states to the `stateOutMsg` messages.
     *
     * @param CurrentSimNanos The current simulation time in nanoseconds.
     */
    void writeOutputStateMessages(uint64_t CurrentSimNanos);

protected:
    MJSpec spec; ///< `MJSpec` (MuJoCo model specification wrapper) associated with this scene.
    bool mjModelConstStale = false; ///< Flag indicating stale model constants.
    bool forwardKinematicsStale = true; ///< Flag indicating stale forward kinematics.

    SysModelTask dynamicsTask; ///< Task managing models involved in the dynamics of this scene.
    std::vector<std::unique_ptr<SysModel>> ownedSysModel; ///< System models that should be cleared on this scene destruction.

    MJQPosStateData* qposState; ///< Position state data.
    StateData* qvelState; ///< Velocity state data.
    StateData* actState; ///< Actuator state data.
};

template <typename T>
inline void MJScene::logAndThrow (const std::string& error)
{
    MJBasilisk::detail::logAndThrow<T>(error, &this->bskLogger);
}

#endif
