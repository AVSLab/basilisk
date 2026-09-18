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

#ifndef DUAL_HINGED_RIGID_BODY_STATE_EFFECTOR_H
#define DUAL_HINGED_RIGID_BODY_STATE_EFFECTOR_H

#include "simulation/dynamics/_GeneralModuleFiles/stateEffector.h"
#include "simulation/dynamics/_GeneralModuleFiles/dynamicEffector.h"
#include "simulation/dynamics/_GeneralModuleFiles/stateData.h"
#include "architecture/_GeneralModuleFiles/sys_model.h"
#include <Eigen/Dense>
#include <cstdint>
#include <optional>
#include <string>
#ifndef SWIG
#include "simulation/dynamics/_GeneralModuleFiles/effectorName.h"
#endif
#include <vector>
#include "simulation/dynamics/_GeneralModuleFiles/dynParamManager.h"
#include "architecture/utilities/avsEigenMRP.h"
#include "architecture/utilities/avsEigenSupport.h"
#include "architecture/utilities/bskLogging.h"

#include "architecture/msgPayloadDefC/ArrayMotorTorqueMsgPayload.h"
#include "architecture/msgPayloadDefC/SCStatesMsgPayload.h"
#include "architecture/msgPayloadDefC/HingedRigidBodyMsgPayload.h"
#include "architecture/messaging/messaging.h"

/*! @brief dual hinged rigid body state effector */
class DualHingedRigidBodyStateEffector final : public StateEffector, public SysModel {
public:
    /** @brief Set the explicit name used by the dynamics manager.
     * @param value Exact custom name, including names matching the automatic pattern.
     * @note Manager-local names cannot change after registration.
     */
    void setNameOfTheta1State(const std::string& value);
    /** @brief Get the constructor name or the resolved name after registration.
     * @return Current state or property name.
     */
    const std::string& getNameOfTheta1State() const { return this->nameOfTheta1State; }
    /** @brief Set the explicit name used by the dynamics manager.
     * @param value Exact custom name, including names matching the automatic pattern.
     * @note Manager-local names cannot change after registration.
     */
    void setNameOfTheta1DotState(const std::string& value);
    /** @brief Get the constructor name or the resolved name after registration.
     * @return Current state or property name.
     */
    const std::string& getNameOfTheta1DotState() const { return this->nameOfTheta1DotState; }
    /** @brief Set the explicit name used by the dynamics manager.
     * @param value Exact custom name, including names matching the automatic pattern.
     * @note Manager-local names cannot change after registration.
     */
    void setNameOfTheta2State(const std::string& value);
    /** @brief Get the constructor name or the resolved name after registration.
     * @return Current state or property name.
     */
    const std::string& getNameOfTheta2State() const { return this->nameOfTheta2State; }
    /** @brief Set the explicit name used by the dynamics manager.
     * @param value Exact custom name, including names matching the automatic pattern.
     * @note Manager-local names cannot change after registration.
     */
    void setNameOfTheta2DotState(const std::string& value);
    /** @brief Get the constructor name or the resolved name after registration.
     * @return Current state or property name.
     */
    const std::string& getNameOfTheta2DotState() const { return this->nameOfTheta2DotState; }
    /** @brief Set the explicit name used by the dynamics manager.
     * @param value Exact custom name, including names matching the automatic pattern.
     * @note Manager-local names cannot change after registration.
     */
    void setNameOfInertialPositionProperty1(const std::string& value);
    /** @brief Get the constructor name or the resolved name after registration.
     * @return Current state or property name.
     */
    const std::string& getNameOfInertialPositionProperty1() const { return this->nameOfInertialPositionProperty1; }
    /** @brief Set the explicit name used by the dynamics manager.
     * @param value Exact custom name, including names matching the automatic pattern.
     * @note Manager-local names cannot change after registration.
     */
    void setNameOfInertialVelocityProperty1(const std::string& value);
    /** @brief Get the constructor name or the resolved name after registration.
     * @return Current state or property name.
     */
    const std::string& getNameOfInertialVelocityProperty1() const { return this->nameOfInertialVelocityProperty1; }
    /** @brief Set the explicit name used by the dynamics manager.
     * @param value Exact custom name, including names matching the automatic pattern.
     * @note Manager-local names cannot change after registration.
     */
    void setNameOfInertialAttitudeProperty1(const std::string& value);
    /** @brief Get the constructor name or the resolved name after registration.
     * @return Current state or property name.
     */
    const std::string& getNameOfInertialAttitudeProperty1() const { return this->nameOfInertialAttitudeProperty1; }
    /** @brief Set the explicit name used by the dynamics manager.
     * @param value Exact custom name, including names matching the automatic pattern.
     * @note Manager-local names cannot change after registration.
     */
    void setNameOfInertialAngVelocityProperty1(const std::string& value);
    /** @brief Get the constructor name or the resolved name after registration.
     * @return Current state or property name.
     */
    const std::string& getNameOfInertialAngVelocityProperty1() const { return this->nameOfInertialAngVelocityProperty1; }
    /** @brief Set the explicit name used by the dynamics manager.
     * @param value Exact custom name, including names matching the automatic pattern.
     * @note Manager-local names cannot change after registration.
     */
    void setNameOfInertialPositionProperty2(const std::string& value);
    /** @brief Get the constructor name or the resolved name after registration.
     * @return Current state or property name.
     */
    const std::string& getNameOfInertialPositionProperty2() const { return this->nameOfInertialPositionProperty2; }
    /** @brief Set the explicit name used by the dynamics manager.
     * @param value Exact custom name, including names matching the automatic pattern.
     * @note Manager-local names cannot change after registration.
     */
    void setNameOfInertialVelocityProperty2(const std::string& value);
    /** @brief Get the constructor name or the resolved name after registration.
     * @return Current state or property name.
     */
    const std::string& getNameOfInertialVelocityProperty2() const { return this->nameOfInertialVelocityProperty2; }
    /** @brief Set the explicit name used by the dynamics manager.
     * @param value Exact custom name, including names matching the automatic pattern.
     * @note Manager-local names cannot change after registration.
     */
    void setNameOfInertialAttitudeProperty2(const std::string& value);
    /** @brief Get the constructor name or the resolved name after registration.
     * @return Current state or property name.
     */
    const std::string& getNameOfInertialAttitudeProperty2() const { return this->nameOfInertialAttitudeProperty2; }
    /** @brief Set the explicit name used by the dynamics manager.
     * @param value Exact custom name, including names matching the automatic pattern.
     * @note Manager-local names cannot change after registration.
     */
    void setNameOfInertialAngVelocityProperty2(const std::string& value);
    /** @brief Get the constructor name or the resolved name after registration.
     * @return Current state or property name.
     */
    const std::string& getNameOfInertialAngVelocityProperty2() const { return this->nameOfInertialAngVelocityProperty2; }

private:
    std::string nameOfTheta1State; //!< Current state or property name.
    std::optional<std::string> customNameOfTheta1State; //!< Explicit override.
    std::string nameOfTheta1DotState; //!< Current state or property name.
    std::optional<std::string> customNameOfTheta1DotState; //!< Explicit override.
    std::string nameOfTheta2State; //!< Current state or property name.
    std::optional<std::string> customNameOfTheta2State; //!< Explicit override.
    std::string nameOfTheta2DotState; //!< Current state or property name.
    std::optional<std::string> customNameOfTheta2DotState; //!< Explicit override.
    std::string nameOfInertialPositionProperty1; //!< Current state or property name.
    std::optional<std::string> customNameOfInertialPositionProperty1; //!< Explicit override.
    std::string nameOfInertialVelocityProperty1; //!< Current state or property name.
    std::optional<std::string> customNameOfInertialVelocityProperty1; //!< Explicit override.
    std::string nameOfInertialAttitudeProperty1; //!< Current state or property name.
    std::optional<std::string> customNameOfInertialAttitudeProperty1; //!< Explicit override.
    std::string nameOfInertialAngVelocityProperty1; //!< Current state or property name.
    std::optional<std::string> customNameOfInertialAngVelocityProperty1; //!< Explicit override.
    std::string nameOfInertialPositionProperty2; //!< Current state or property name.
    std::optional<std::string> customNameOfInertialPositionProperty2; //!< Explicit override.
    std::string nameOfInertialVelocityProperty2; //!< Current state or property name.
    std::optional<std::string> customNameOfInertialVelocityProperty2; //!< Explicit override.
    std::string nameOfInertialAttitudeProperty2; //!< Current state or property name.
    std::optional<std::string> customNameOfInertialAttitudeProperty2; //!< Explicit override.
    std::string nameOfInertialAngVelocityProperty2; //!< Current state or property name.
    std::optional<std::string> customNameOfInertialAngVelocityProperty2; //!< Explicit override.
    bool effectorNamesResolved = false; //!< Protect registered manager-local names.
    /** @brief Track explicit assignments and reject changes after registration.
     * @param currentName Visible name to update.
     * @param customName Explicit override metadata.
     * @param value Exact custom name.
     */
    void setCustomName(std::string& currentName, std::optional<std::string>& customName, const std::string& value);
    /** @brief Apply the manager's resolved names before registering data.
     * @param manager Manager holding this effector's declaration.
     */
    void applyResolvedNames(DynParamManager& manager);

public:

    DualHingedRigidBodyStateEffector();
    ~DualHingedRigidBodyStateEffector();
    void addDynamicEffector(DynamicEffector *newDynamicEffector, int segment) override;  //!< Method for adding attached dynamic effector
    void registerProperties(DynParamManager& states) override;       //!< Method for registering the panel inertial properties
    void registerStates(DynParamManager& statesIn) override;     //!< class method
    void linkInStates(DynParamManager& states) override;         //!< class method
    void updateEffectorMassProps(double integTime) override;     //!< class method
    void updateContributions(double integTime, BackSubMatrices & backSubContr, Eigen::MRPd sigma_BN, Eigen::Vector3d omega_BN_B, Eigen::Vector3d g_N) override;  //!< Back-sub contributions
    void updateEnergyMomContributions(double integTime, Eigen::Vector3d & rotAngMomPntCContr_B,
                                              double & rotEnergyContr, Eigen::Vector3d omega_BN_B) override;  //!< Energy and momentum calculations
    void computeDerivatives(double integTime, Eigen::Vector3d rDDot_BN_N, Eigen::Vector3d omegaDot_BN_B, Eigen::MRPd sigma_BN) override;  //!< Method for each stateEffector to calculate derivatives
    void Reset(uint64_t CurrentSimNanos) override;
    void UpdateState(uint64_t CurrentSimNanos) override;
    void writeOutputStateMessages(uint64_t CurrentClock) override;

private:
    void computePanelInertialStates();
    void prependSpacecraftNameToStates() override; //!< class method used for multiple spacecraft

public:
    double mass1;                     //!< [kg] mass of 1st hinged rigid body
    double mass2;                     //!< [kg] mass of 2nd hinged rigid body
    double d1;                        //!< [m] distance from hinge point H1 to hinged rigid body center of mass S1
    double d2;                        //!< [m] distance from hinge point H2 to hinged rigid body center of mass S2
    double l1;                        //!< [m] distance from hinge point H1 to hinged point H2
    double k1;                        //!< [N-m/rad] torsional spring constant of hinge
    double k2;                        //!< [N-m/rad] torsional spring constant of hinge
    double c1;                        //!< [N-m-s/rad] rotational damping coefficient of hinge
    double c2;                        //!< [N-m-s/rad] rotational damping coefficient of hinge
    double theta1Init;                //!< [rad] Initial hinged rigid body angle for first panel
    double theta1DotInit;             //!< [rad/s] Initial hinged rigid body angle rate for first panel
    double theta2Init;                //!< [rad] Initial hinged rigid body angle for second panel
    double theta2DotInit;             //!< [rad/s] Initial hinged rigid body angle rate for second panel
    Eigen::Matrix3d IPntS1_S1;        //!< [kg-m^2] Inertia of hinged rigid body about point S in S frame components
    Eigen::Matrix3d IPntS2_S2;        //!< [kg-m^2] Inertia of hinged rigid body about point S in S frame components
    Eigen::Vector3d r_H1B_B;          //!< [m] vector pointing from body frame origin to Hinge location
    Eigen::Matrix3d dcm_H1B;          //!< [-] DCM from body frame to hinge frame
    double thetaH2S1;                 //!< [-] theta offset of H2 frame with respect to S1 frame
    BSKLogger bskLogger;                      //!< BSK Logging
    ReadFunctor<ArrayMotorTorqueMsgPayload> motorTorqueInMsg; //!< (optional) motor torque input message
    std::vector<Message<HingedRigidBodyMsgPayload>*> dualHingedRigidBodyOutMsgs; //!< state output message vector for all panels
    std::vector<Message<SCStatesMsgPayload>*> dualHingedRigidBodyConfigLogOutMsgs; //!< panel state config log message vector for all panels
    std::vector<DynamicEffector*> dynEffectors;           //!< Vector of dynamic effectors attached
    std::vector<int> dynEffectorSegments;                 //!< Segment index for each attached dynamic effector

private:
    static uint64_t effectorID;        //!< [] ID number of this panel
    Eigen::Vector3d r_H1P_P;          //!< [m] vector pointing from primary body frame P origin to Hinge 1 location.  If a single spacecraft body is modeled than P is the same as B
    Eigen::Vector3d r_H2P_P;          //!< [m] vector pointing from primary body frame P origin to Hinge 2 location
    Eigen::Matrix3d dcm_H1P;          //!< DCM from primary body frame to hinge 1 frame
    double u1;                        //!< [N-m] motor torques on panel 1
    double u2;                        //!< [N-m] motor torques on panel 2
    Eigen::Matrix3d rTildeH1B_B;      //!< [-] Tilde matrix of rHB_B
    Eigen::Matrix3d dcm_S1P;          //!< [-] DCM from primary body to S1 frame
    Eigen::Matrix3d dcm_S2P;          //!< [-] DCM from primary body to S2 frame
    Eigen::Vector3d omega_PN_S1;      //!< [rad/s] omega_PN in S1 frame components
    Eigen::Vector3d omega_PN_S2;      //!< [rad/s] omega_PN in S2 frame components
    Eigen::Vector3d sHat11_P;         //!< [-] unit direction vector for the first axis of the S frame
    Eigen::Vector3d sHat12_P;         //!< [-] unit direction vector for the second axis of the S frame
    Eigen::Vector3d sHat13_P;         //!< [-] unit direction vector for the third axis of the S frame
    Eigen::Vector3d sHat21_P;         //!< [-] unit direction vector for the first axis of the S frame
    Eigen::Vector3d sHat22_P;         //!< [-] unit direction vector for the second axis of the S frame
    Eigen::Vector3d sHat23_P;         //!< [-] unit direction vector for the third axis of the S frame
    Eigen::Vector3d r_S1P_P;          //!< [-] Vector pointing from body origin to CoM of hinged rigid body in P frame comp
    Eigen::Vector3d r_S2P_P;          //!< [-] Vector pointing from body origin to CoM of hinged rigid body in P frame comp
    Eigen::Matrix3d rTildeS1P_P;      //!< [-] Tilde matrix of rSP_P
    Eigen::Matrix3d rTildeS2P_P;      //!< [-] Tilde matrix of rSP_P
    Eigen::Vector3d rPrimeS1P_P;      //!< [m/s] Body time derivative of rSP_P
    Eigen::Vector3d rPrimeS2P_P;      //!< [m/s] Body time derivative of rSBP_P
    Eigen::Matrix3d rPrimeTildeS1P_P; //!< [-] Tilde matrix of rPrime_SP_P
    Eigen::Matrix3d rPrimeTildeS2P_P; //!< [-] Tilde matrix of rPrime_SP_P
    Eigen::Matrix3d IS1PrimePntS1_P;  //!< [kg-m^2/s] time body derivative IPntS in primary body frame components
    Eigen::Matrix3d IS2PrimePntS2_P;  //!< [kg-m^2/s] time body derivative IPntS in primary body frame components
    Eigen::Vector3d omega_PNLoc_P;    //!< [rad/s] local copy of omegaPN
    Eigen::Matrix3d omegaTildePNLoc_P;//!< [-] tilde matrix of omegaPN
    double theta1;                    //!< [rad] hinged rigid body angle
    double theta1Dot;                 //!< [rad/s] hinged rigid body angle rate
    double theta2;                    //!< [rad] hinged rigid body angle
    double theta2Dot;                 //!< [rad/s] hinged rigid body angle rate
    Eigen::Matrix2d matrixADHRB;      //!< [-] term needed for Backsubstitution
    Eigen::Matrix2d matrixEDHRB;      //!< [-] term needed for Backsubstitution
    Eigen::MatrixXd matrixFDHRB;
    Eigen::MatrixXd matrixGDHRB;
    Eigen::Vector2d vectorVDHRB;
    StateData *theta1State;           //!< [-] state manager of theta for hinged rigid body
    StateData *theta1DotState;        //!< [-] state manager of thetaDot for hinged rigid body
    StateData *theta2State;           //!< [-] state manager of theta for hinged rigid body
    StateData *theta2DotState;        //!< [-] state manager of thetaDot for hinged rigid body
    std::vector<Eigen::Vector3d> r_SN_N;      //!< [m] position vector of hinge CM S relative to inertial frame
    std::vector<Eigen::Vector3d> v_SN_N;      //!< [m/s] inertial velocity vector of S relative to inertial frame
    std::vector<Eigen::MatrixXd*> r_HN_N;     //!< [m] position vector of hinge point H relative to inertial frame
    std::vector<Eigen::MatrixXd*> v_HN_N;     //!< [m/s] inertial velocity vector of H relative to inertial frame
    std::vector<Eigen::MatrixXd*> sigma_SN;   //!< MRP attitude of panel frame S relative to inertial frame
    std::vector<Eigen::MatrixXd*> omega_SN_S; //!< [rad/s] inertial panel frame angular velocity vector
    Eigen::MRPd sigma_BN{0.0, 0.0, 0.0};        //!< Hub/Inertial attitude represented by MRP of body relative to inertial frame
    Eigen::Vector3d omega_BN_B{0.0, 0.0, 0.0};  //!< Hub/Inertial angular velocity vector in B frame components
    StateData *v_BN_NState;           //!< Hub/Inertial velocity vector in inertial frame components
    StateData* hubSigmaState = nullptr;  //!< hub attitude state, read live for the published kinematics
    Eigen::MatrixXd *inertialPositionProperty;  //!< [m] r_N inertial position relative to system spice zeroBase/refBase
    Eigen::MatrixXd *inertialVelocityProperty;  //!< [m] v_N inertial velocity relative to system spice zeroBase/refBase
    Eigen::MatrixXd *g_N;             //!< [m/s^2] Gravitational acceleration in N frame components

    /** @brief Assign the selected body's property names to an attached effector.
     * @param effector Dynamic effector receiving the names.
     * @param segment One-based body index.
     */
    void assignStateParamNames(DynamicEffector* effector, int segment);


    void validateConfiguration(); //!< Validate panel masses and the configured hinge-frame DCM
#ifndef SWIG
public:
    /** @brief Bind dependent effectors after all body properties are registered.
     * @param manager Manager containing the resolved states and properties.
     */
    void bindAttachedDynamicEffectors(DynParamManager& manager) override;
protected:
    /** @brief Declare the names sharing this effector's automatic index.
     * @return State and property declarations, including every body.
     */
    EffectorNameGroup describeEffectorNames() const override;
#endif
};


#endif /* DUAL_STATE_EFFECTOR_H */
