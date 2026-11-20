/*
 ISC License

 Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#ifndef GENERAL_SINGLE_BODY_STATE_EFFECTOR_H
#define GENERAL_SINGLE_BODY_STATE_EFFECTOR_H

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/messaging/messaging.h"
#include "architecture/msgPayloadDefC/ArrayMotorForceMsgPayload.h"
#include "architecture/msgPayloadDefC/ArrayMotorTorqueMsgPayload.h"
#include "architecture/msgPayloadDefC/HingedRigidBodyMsgPayload.h"
#include "architecture/msgPayloadDefC/LinearTranslationRigidBodyMsgPayload.h"
#include "architecture/msgPayloadDefC/SCStatesMsgPayload.h"
#include "architecture/utilities/avsEigenMRP.h"
#include "architecture/utilities/avsEigenSupport.h"
#include "architecture/utilities/bskLogging.h"
#include "simulation/dynamics/_GeneralModuleFiles/dynParamManager.h"
#include "simulation/dynamics/_GeneralModuleFiles/stateData.h"
#include "simulation/dynamics/_GeneralModuleFiles/stateEffector.h"
#include <Eigen/Dense>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

class GeneralSingleBodyStateEffector;

/**
 * @brief Configuration and runtime state for one sequential joint degree of freedom.
 * @note Configuration bounds are checked by the owning effector during state registration and Reset().
 */
struct DOF
{
  public:
    DOF() = default;
    // Preserve DOF identity: copying or assigning ownership would invalidate the owning effector.
    DOF(const DOF&) = delete;
    DOF& operator=(const DOF&) = delete;

    /** Enumeration indicating the joint DOF type */
    enum class Type
    {
        ROTATION,
        TRANSLATION
    };

    /**
     * @brief Set the joint motion axis.
     * @param axis_G Finite unit vector expressed in this joint's frame G [-].
     */
    void setDOFAxis(Eigen::Vector3d axis_G) { this->axis_G = axis_G; };
    /**
     * @brief Set the initial joint position used during state registration.
     * @param betaInit Finite rotation angle [rad] or translation displacement [m].
     */
    void setBetaInit(double betaInit) { this->betaInit = betaInit; };
    /**
     * @brief Set the initial joint rate used during state registration.
     * @param betaDotInit Finite angular rate [rad/s] or translation rate [m/s].
     */
    void setBetaDotInit(double betaDotInit) { this->betaDotInit = betaDotInit; };
    /**
     * @brief Set the joint spring constant.
     * @param k Finite nonnegative stiffness [N*m/rad] for rotation or [N/m] for translation.
     */
    void setSpringConstantK(double k) { this->k = k; };
    /**
     * @brief Set the joint damping constant.
     * @param c Finite nonnegative damping [N*m*s/rad] for rotation or [N*s/m] for translation.
     */
    void setDampingConstantC(double c) { this->c = c; };

    /**
     * @brief Get the current joint motion type.
     * @return Type assigned when the DOF is added to an effector; rotation for a new DOF.
     */
    Type getDOFType() const { return this->type; };
    /**
     * @brief Get the configured joint motion axis.
     * @return Axis expressed in this joint's frame G [-].
     */
    Eigen::Vector3d getDOFAxis() const { return this->axis_G; };
    /**
     * @brief Get the configured initial joint position.
     * @return Rotation angle [rad] or translation displacement [m].
     */
    double getBetaInit() const { return this->betaInit; };
    /**
     * @brief Get the configured initial joint rate.
     * @return Angular rate [rad/s] or translation rate [m/s].
     */
    double getBetaDotInit() const { return this->betaDotInit; };
    /**
     * @brief Get the joint spring constant.
     * @return Stiffness [N*m/rad] for rotation or [N/m] for translation.
     */
    double getSpringConstantK() const { return this->k; };
    /**
     * @brief Get the joint damping constant.
     * @return Damping [N*m*s/rad] for rotation or [N*s/m] for translation.
     */
    double getDampingConstantC() const { return this->c; };

  private:
    friend class GeneralSingleBodyStateEffector;

    const GeneralSingleBodyStateEffector* owner = nullptr; //!< Non-owning pointer to the effector using this DOF

    Type type{ Type::ROTATION };             //!< [-] DOF type
    Eigen::Vector3d axis_G{ 1.0, 0.0, 0.0 }; //!< [-] DOF axis
    double betaInit{};                       //!< [rad] or [m] Initial DOF state
    double betaDotInit{};                    //!< [rad/s] or [m/s] Initial DOF rate
    double k{};                              //!< [N*m/rad] or [N/m] DOF spring constant
    double c{};                              //!< [N*m*s/rad] or [N*s/m] DOF damping constant

    uint32_t index{};    //!< [-] DOF index
    double beta{};       //!< [rad] or [m] DOF state
    double betaDot{};    //!< [rad/s] or [m/s] DOF rate
    double betaRef{};    //!< [rad] or [m] DOF reference state
    double betaDotRef{}; //!< [rad/s] or [m/s] DOF reference rate
    Eigen::Vector3d r_GB_B{
        Eigen::Vector3d::Zero()
    }; //!< [m] Position vector of joint frame G relative to the hub frame B
    Eigen::Vector3d omega_GB_B{
        Eigen::Vector3d::Zero()
    }; //!< [rad/s] Angular velocity vector of the joint body frame G relative to the hub frame B
    Eigen::Matrix3d dcm_GB{
        Eigen::Matrix3d::Identity()
    }; //!< [-] Initial effector body frame G attitude relative to the hub frame B
    double u{}; //!< [N * m] Joint DOF torque
    double f{}; //!< [N] Joint DOF force
};

/**
 * @brief Rigid body state effector with up to three rotational and three translational DOFs.
 * @note Setters store configuration; validation occurs during state registration and Reset().
 */
class GeneralSingleBodyStateEffector
    : public StateEffector
    , public SysModel
{
  public:
    GeneralSingleBodyStateEffector();  //!< Class constructor
    ~GeneralSingleBodyStateEffector(); //!< Class destructor

    /**
     * @brief Set the component mass.
     * @param mass Finite, strictly positive mass [kg].
     */
    void setMass(const double mass) { this->mass = mass; };
    /**
     * @brief Set the component inertia about its center of mass Gc.
     * @param IPntGc_G Finite symmetric inertia tensor expressed in frame G [kg*m^2].
     * @note Principal moments must be nonnegative, satisfy the triangle inequalities, and have at most one zero.
     */
    void setIPntGc_G(const Eigen::Matrix3d IPntGc_G) { this->IPntGc_G = IPntGc_G; };
    /**
     * @brief Set the component center-of-mass offset.
     * @param r_GcG_G Finite position of Gc relative to body origin G, expressed in frame G [m].
     */
    void setR_GcG_G(const Eigen::Vector3d r_GcG_G) { this->r_GcG_G = r_GcG_G; };
    /**
     * @brief Set the joint mounting position on the hub.
     * @param r_G0B_B Finite position of mounting origin G0 relative to hub origin B, expressed in frame B [m].
     */
    void setR_G0B_B(Eigen::Vector3d r_G0B_B) { this->r_G0B_B = r_G0B_B; };
    /**
     * @brief Set the joint mounting orientation.
     * @param dcm_G0B Proper rotation matrix mapping frame B components into the undeflected mounting frame G0 [-].
     */
    void setDCM_G0B(Eigen::Matrix3d dcm_G0B) { this->dcm_G0B = dcm_G0B; };

    /**
     * @brief Get the component mass.
     * @return Configured mass [kg].
     */
    double getMass() const { return this->mass; };
    /**
     * @brief Get the component inertia about its center of mass Gc.
     * @return Inertia tensor expressed in frame G [kg*m^2].
     */
    const Eigen::Matrix3d getIPntGc_G() const { return this->IPntGc_G; };
    /**
     * @brief Get the component center-of-mass offset.
     * @return Position of Gc relative to body origin G, expressed in frame G [m].
     */
    const Eigen::Vector3d getR_GcG_G() const { return this->r_GcG_G; };
    /**
     * @brief Get the joint mounting position on the hub.
     * @return Position of mounting origin G0 relative to hub origin B, expressed in frame B [m].
     */
    Eigen::Vector3d getR_G0B_B() const { return this->r_G0B_B; };
    /**
     * @brief Get the joint mounting orientation.
     * @return Rotation matrix mapping frame B components into the undeflected mounting frame G0 [-].
     */
    Eigen::Matrix3d getDCM_G0B() const { return this->dcm_G0B; };

    /**
     * @brief Add a sequential rotational degree of freedom before topology is frozen.
     * @param newDOF Non-null DOF that does not currently belong to an effector.
     * @note At most three rotational DOFs are supported.
     */
    void addRotDOF(std::shared_ptr<DOF> newDOF);
    /**
     * @brief Add a sequential translational degree of freedom before topology is frozen.
     * @param newDOF Non-null DOF that does not currently belong to an effector.
     * @note At most three translational DOFs are supported.
     */
    void addTransDOF(std::shared_ptr<DOF> newDOF);
    /**
     * @brief Get a DOF from the configured sequence.
     * @param index Zero-based index in the combined rotational/translational addition order.
     * @return Shared pointer to the DOF at the requested index.
     */
    std::shared_ptr<DOF> getDOF(uint64_t index) { return this->jointDOFList.at(index); };
    /** @brief Release all DOFs and their message ports before topology is frozen. */
    void clearDOFs();

    /**
     * @brief Validate configuration while preserving live joint states and cached commands/references.
     * @param currentSimNanos Current simulation time [ns].
     */
    void Reset(uint64_t currentSimNanos) override;
    /**
     * @brief Validate configuration and register joint states with their configured initial values.
     * @param statesIn Parent spacecraft state manager.
     * @note Spacecraft reset calls this method to restore initial joint states without clearing cached commands.
     */
    void registerStates(DynParamManager& statesIn) override;
    /**
     * @brief Link the hub attitude/rate states and inertial position/velocity properties.
     * @param states Parent spacecraft state manager containing the configured state and property names.
     */
    void linkInStates(DynParamManager& states) override;
    /**
     * @brief Read linked, written inputs and publish current output states.
     * @param currentSimNanos Current simulation time and output message timestamp [ns].
     * @note Requires task scheduling to consume inputs; unlinked or unwritten inputs preserve cached values.
     */
    void UpdateState(uint64_t currentSimNanos) override;
    /**
     * @brief Compute inertial configuration from the accepted hub state and current joint kinematics.
     * @note Requires linked hub states and refreshed joint kinematics from updateEffectorMassProps().
     */
    void computeGeneralBodyInertialStates();
    /**
     * @brief Publish joint states and refresh subscribed inertial configuration output.
     * @param currentSimNanos Output message timestamp [ns].
     */
    void writeOutputStateMessages(uint64_t currentSimNanos) override;
    /**
     * @brief Refresh joint kinematics and effector mass properties from registered states.
     * @param integTime Current integration time [s].
     */
    void updateEffectorMassProps(double integTime) override;
    /**
     * @brief Compute the effector contributions to the hub backsubstitution equations.
     * @param integTime Current integration time [s].
     * @param[out] backSubContr Effector backsubstitution matrices and forcing vectors.
     * @param sigma_BN Hub frame B attitude relative to inertial frame N as MRPs [-].
     * @param omega_BN_B Hub angular velocity relative to frame N, expressed in frame B [rad/s].
     * @param g_N Gravitational acceleration expressed in frame N [m/s^2].
     */
    void updateContributions(double integTime,
                             BackSubMatrices& backSubContr,
                             Eigen::MRPd sigma_BN,
                             Eigen::Vector3d omega_BN_B,
                             Eigen::Vector3d g_N) override;
    /**
     * @brief Compute joint state derivatives using the current backsubstitution coefficients.
     * @param integTime Current integration time [s].
     * @param rDDot_BN_N Hub origin B inertial acceleration expressed in frame N [m/s^2].
     * @param omegaDot_BN_B Hub inertial angular acceleration expressed in frame B [rad/s^2].
     * @param sigma_BN Hub frame B attitude relative to inertial frame N as MRPs [-].
     */
    void computeDerivatives(double integTime,
                            Eigen::Vector3d rDDot_BN_N,
                            Eigen::Vector3d omegaDot_BN_B,
                            Eigen::MRPd sigma_BN) override;
    /**
     * @brief Compute the effector energy and angular momentum contributions.
     * @param integTime Current integration time [s].
     * @param[out] rotAngMomPntCContr_B Angular momentum contribution about hub point B, in frame B [kg*m^2/s].
     * @param[out] rotEnergyContr Kinetic energy relative to hub point B plus spring potential energy [J].
     * @param omega_BN_B Hub angular velocity relative to frame N, expressed in frame B [rad/s].
     * @note The spacecraft shifts the summed contributions to its system center of mass.
     */
    void updateEnergyMomContributions(double integTime,
                                      Eigen::Vector3d& rotAngMomPntCContr_B,
                                      double& rotEnergyContr,
                                      Eigen::Vector3d omega_BN_B) override;

    std::vector<ReadFunctor<HingedRigidBodyMsgPayload>>
        spinningBodyRefInMsg; //!< Input messages for rotational DOF reference angles [rad] and rates [rad/s]
    std::vector<ReadFunctor<ArrayMotorTorqueMsgPayload>>
        motorTorqueInMsg; //!< Input message for rotational degree of freedom motor torque
    std::vector<ReadFunctor<LinearTranslationRigidBodyMsgPayload>>
        translatingBodyRefInMsgs; //!< Input messages for translational DOF reference displacements [m] and rates [m/s]
    std::vector<ReadFunctor<ArrayMotorForceMsgPayload>>
        motorForceInMsg; //!< Input message for translational degree of freedom motor force
    std::vector<Message<HingedRigidBodyMsgPayload>*>
        spinningBodyOutMsgs; //!< Output message for rotational degree of freedom scalar states
    std::vector<Message<LinearTranslationRigidBodyMsgPayload>*>
        translatingBodyOutMsgs; //!< Output message for translational degree of freedom scalar states
    Message<SCStatesMsgPayload> generalSingleBodyConfigLogOutMsg; //!< Output message for the effector inertial states

  private:
    // Runtime dimensions use inline storage for the module's maximum of six DOFs.
    using DOFVector = Eigen::Matrix<double, Eigen::Dynamic, 1, 0, 6, 1>;
    using DOFMatrix = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, 0, 6, 6>;
    using DOFMatrix3 = Eigen::Matrix<double, Eigen::Dynamic, 3, 0, 6, 3>;
    using DOFMapping = Eigen::Matrix<double, 6, Eigen::Dynamic, 0, 6, 6>;

    double mass{};                                           //!< [kg] General body mass
    Eigen::Matrix3d IPntGc_G{ Eigen::Matrix3d::Identity() }; //!< [kg * m^2] Effector inertia
    Eigen::Vector3d r_GcG_G{
        Eigen::Vector3d::Zero()
    }; //!< [m] Position vector of the effector center of mass point Gc relative to its body frame origin point G
    Eigen::Vector3d r_G0B_B{
        Eigen::Vector3d::Zero()
    }; //!< [m] Initial position vector of the effector body frame origin point G relative to the hub frame origin point B
    Eigen::Matrix3d dcm_G0B{
        Eigen::Matrix3d::Identity()
    }; //!< [-] Initial effector body frame G attitude relative to the hub frame B
    std::vector<std::shared_ptr<DOF>> jointDOFList; //!< [-] Effector sequential DOF list

    uint32_t numDOF{};                   //!< [-] Total number of effector degrees of freedom
    uint32_t numRotDOF{};                //!< [-] Total number of effector rotational degrees of freedom
    uint32_t numTransDOF{};              //!< [-] Total number of effector translational degrees of freedom
    std::vector<double> betaInitList;    //!< [rad] or [m] per entry, initial state vector
    std::vector<double> betaDotInitList; //!< [rad/s] or [m/s] per entry, initial state vector rate

    DOFVector beta;                    //!< [rad] or [m] per entry, effector state vector
    DOFVector betaDot;                 //!< [rad/s] or [m/s] per entry, effector state vector rate
    StateData* betaState = nullptr;    //!< [rad] or [m] per entry, state data container for joint positions
    StateData* betaDotState = nullptr; //!< [rad/s] or [m/s] per entry, state data container for joint rates

    const StateData* hubSigmaState = nullptr; //!< Hub attitude state, read live when publishing inertial outputs
    const StateData* hubOmegaState = nullptr; //!< Hub angular-rate state, read live when publishing inertial outputs
    Eigen::MatrixXd* inertialPositionProperty = nullptr; //!< [m] Identifier for the hub inertial position
    Eigen::MatrixXd* inertialVelocityProperty = nullptr; //!< [m/s] Identifier for the hub inertial velocity

    DOFMapping TMat;      //!< [-] Effector mapping vectrix
    DOFMapping TPrimeMat; //!< [1/s] Effector mapping vectrix rate
    DOFMapping UMat;      //!< Kinematics storage matrix; upper three rows [m], lower three rows [rad]
    Eigen::Matrix<double, 6, 1> vVec{ Eigen::Matrix<double, 6, 1>::Zero() }; //!< Upper entries [m/s^2], lower [rad/s^2]
    Eigen::Vector3d r_GB_B{
        Eigen::Vector3d::Zero()
    }; //!< [m] Position vector of the effector body frame origin point G relative to the hub frame origin point B
    Eigen::Vector3d r_GcN_N{
        Eigen::Vector3d::Zero()
    }; //!< [m] Position vector of the effector center of mass point Gc relative to the inertial frame origin point N
    Eigen::Vector3d v_GcN_N{
        Eigen::Vector3d::Zero()
    }; //!< [m/s] Velocity vector of the effector center of mass point Gc relative to the inertial frame origin point N
    Eigen::MRPd sigma_GN{
        Eigen::MRPd::Identity()
    }; //!< [-] MRP attitude of the effector body frame G relative to the inertial frame N
    Eigen::Vector3d omega_GN_G{
        Eigen::Vector3d::Zero()
    }; //!< [rad/s] Angular velocity vector of the effector body frame G relative to the inertial frame N

    DOFMatrix3 ABeta; //!< Backsubstitution A; rotational rows [rad/m], translational rows [-]
    DOFMatrix3 BBeta; //!< Backsubstitution B; rotational rows [-], translational rows [m/rad]
    DOFVector CBeta;  //!< Backsubstitution C; rotational entries [rad/s^2], translational entries [m/s^2]

    Eigen::Matrix3d dcm_BN{
        Eigen::Matrix3d::Identity()
    }; //!< [-] Hub body frame B attitude relative to the inertial frame N
    Eigen::Vector3d omega_BN_B{
        Eigen::Vector3d::Zero()
    }; //!< [rad/s] Angular velocity vector of the hub body frame B relative to the inertial frame N

    static uint64_t effectorID;
    BSKLogger bskLogger;

    std::string nameOfBetaState;    //!< [-] String for the effector state vector name
    std::string nameOfBetaDotState; //!< [-] String for the effector state vector rate name

    /** @brief Validate configured parameters and initial joint inertia without changing integrated states. */
    void validateConfiguration();
    /**
     * @brief Check numerical rank after coordinate scaling and invert the joint mass matrix.
     * @param jointMassMatrix Joint inertia matrix to validate and invert.
     * @return Joint mass matrix inverse in the original coordinate scales.
     */
    DOFMatrix computeJointMassMatrixInverse(const DOFMatrix& jointMassMatrix);
    /** Internal method for setting up a new sequential degree of freedom */
    void addDOF(std::shared_ptr<DOF> newDOF);
    /** @brief Release DOFs and message ports, including during destruction of a frozen effector. */
    void clearDOFsInternal();
    /** Internal method to compute all joint attitude properties */
    void computeAttitudeProperties();
    /** Internal method to compute all joint position properties */
    void computePositionProperties();
    /** Internal method to compute all joint angular velocity properties */
    void computeAngularVelocityProperties();
};

#endif /* GENERAL_SINGLE_BODY_STATE_EFFECTOR_H */
