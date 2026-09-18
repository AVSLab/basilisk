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

#ifndef HINGED_RIGID_BODY_STATE_EFFECTOR_H
#define HINGED_RIGID_BODY_STATE_EFFECTOR_H

#include <Eigen/Dense>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>
#include "simulation/dynamics/_GeneralModuleFiles/dynParamManager.h"
#ifndef SWIG
#include "simulation/dynamics/_GeneralModuleFiles/effectorName.h"
#endif
#include "simulation/dynamics/_GeneralModuleFiles/stateEffector.h"
#include "simulation/dynamics/_GeneralModuleFiles/dynamicEffector.h"
#include "simulation/dynamics/_GeneralModuleFiles/stateData.h"
#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/utilities/avsEigenMRP.h"

#include "architecture/msgPayloadDefC/ArrayMotorTorqueMsgPayload.h"
#include "architecture/msgPayloadDefC/SCStatesMsgPayload.h"
#include "architecture/msgPayloadDefC/HingedRigidBodyMsgPayload.h"
#include "architecture/messaging/messaging.h"

#include "architecture/utilities/bskLogging.h"

/*! @brief hinged rigid body state effector class */
class HingedRigidBodyStateEffector final : public StateEffector, public SysModel {
public:
    double mass;                     //!< [kg] mass of hinged rigid body
    double d;                        //!< [m] distance from hinge point H to hinged rigid body center of mass S
    double k;                        //!< [N-m/rad] torsional spring constant of hinge
    double c;                        //!< [N-m-s/rad] rotational damping coefficient of hinge
    double thetaInit;                //!< [rad] Initial hinged rigid body angle
    double thetaDotInit;             //!< [rad/s] Initial hinged rigid body angle rate
    double thetaRef;                  //!< [rad] hinged rigid body reference angle
    double thetaDotRef;               //!< [rad/s] hinged rigid body reference angle rate
    /** @brief Set the explicit theta name; Python exposes this as nameOfThetaState.
     * @param value Exact custom name, including names that match an automatic name.
     * @note Manager-local names cannot change after registration.
     */
    void setNameOfThetaState(const std::string& value);
    /** @brief Get the current theta name.
     * @return Legacy name before preparation, or the resolved name after registration.
     */
    const std::string& getNameOfThetaState() const { return this->nameOfThetaState; }
    /** @brief Set the explicit thetaDot name; Python exposes this as nameOfThetaDotState.
     * @param value Exact custom name, including names that match an automatic name.
     * @note Manager-local names cannot change after registration.
     */
    void setNameOfThetaDotState(const std::string& value);
    /** @brief Get the current thetaDot name.
     * @return Legacy name before preparation, or the resolved name after registration.
     */
    const std::string& getNameOfThetaDotState() const { return this->nameOfThetaDotState; }
    /** @brief Set the explicit position name; Python exposes this as nameOfInertialPositionProperty.
     * @param value Exact custom name, including names that match an automatic name.
     * @note Manager-local names cannot change after registration.
     */
    void setNameOfInertialPositionProperty(const std::string& value);
    /** @brief Get the current position name.
     * @return Legacy name before preparation, or the resolved name after registration.
     */
    const std::string& getNameOfInertialPositionProperty() const { return this->nameOfInertialPositionProperty; }
    /** @brief Set the explicit velocity name; Python exposes this as nameOfInertialVelocityProperty.
     * @param value Exact custom name, including names that match an automatic name.
     * @note Manager-local names cannot change after registration.
     */
    void setNameOfInertialVelocityProperty(const std::string& value);
    /** @brief Get the current velocity name.
     * @return Legacy name before preparation, or the resolved name after registration.
     */
    const std::string& getNameOfInertialVelocityProperty() const { return this->nameOfInertialVelocityProperty; }
    /** @brief Set the explicit attitude name; Python exposes this as nameOfInertialAttitudeProperty.
     * @param value Exact custom name, including names that match an automatic name.
     * @note Manager-local names cannot change after registration.
     */
    void setNameOfInertialAttitudeProperty(const std::string& value);
    /** @brief Get the current attitude name.
     * @return Legacy name before preparation, or the resolved name after registration.
     */
    const std::string& getNameOfInertialAttitudeProperty() const { return this->nameOfInertialAttitudeProperty; }
    /** @brief Set the explicit angularVelocity name; Python exposes this as nameOfInertialAngVelocityProperty.
     * @param value Exact custom name, including names that match an automatic name.
     * @note Manager-local names cannot change after registration.
     */
    void setNameOfInertialAngVelocityProperty(const std::string& value);
    /** @brief Get the current angularVelocity name.
     * @return Legacy name before preparation, or the resolved name after registration.
     */
    const std::string& getNameOfInertialAngVelocityProperty() const { return this->nameOfInertialAngVelocityProperty; }
    Eigen::Matrix3d IPntS_S;         //!< [kg-m^2] Inertia of hinged rigid body about point S in S frame components
    Eigen::Vector3d r_HB_B;          //!< [m] vector pointing from body frame origin to Hinge location
    Eigen::Matrix3d dcm_HB;          //!< DCM from body frame to hinge frame
    Message<HingedRigidBodyMsgPayload> hingedRigidBodyOutMsg; //!< state output message name
    ReadFunctor<ArrayMotorTorqueMsgPayload> motorTorqueInMsg; //!< (optional) motor torque input message name
    ReadFunctor<HingedRigidBodyMsgPayload> hingedRigidBodyRefMsg; //!< (optional) rigid body reference input message name
    Message<SCStatesMsgPayload> hingedRigidBodyConfigLogOutMsg; //!< panel state config log message name
    HingedRigidBodyMsgPayload HRBoutputStates;  //!< instance of messaging system message struct
    BSKLogger bskLogger;                      //!< BSK Logging
    std::vector<DynamicEffector*> dynEffectors;  //!< Vector of dynamic effectors attached

private:
    std::string nameOfThetaState; //!< Current theta name.
    std::optional<std::string> customThetaState; //!< Explicit override, independent of automatic names.
    std::string nameOfThetaDotState; //!< Current thetaDot name.
    std::optional<std::string> customThetaDotState; //!< Explicit override, independent of automatic names.
    std::string nameOfInertialPositionProperty; //!< Current position name.
    std::optional<std::string> customInertialPositionProperty; //!< Explicit override, independent of automatic names.
    std::string nameOfInertialVelocityProperty; //!< Current velocity name.
    std::optional<std::string> customInertialVelocityProperty; //!< Explicit override, independent of automatic names.
    std::string nameOfInertialAttitudeProperty; //!< Current attitude name.
    std::optional<std::string> customInertialAttitudeProperty; //!< Explicit override, independent of automatic names.
    std::string nameOfInertialAngVelocityProperty; //!< Current angularVelocity name.
    std::optional<std::string> customInertialAngVelocityProperty; //!< Explicit angular-velocity property override.
    bool effectorNamesResolved = false; //!< Prevent changes to registered manager-local names.
    /** @brief Record an explicit name assignment, enforcing resolved-name immutability.
     * @param currentName Currently visible name to update.
     * @param customName Metadata identifying an explicit override.
     * @param value Exact custom name.
     */
    void setCustomName(std::string& currentName, std::optional<std::string>& customName, const std::string& value);
    /** @brief Apply resolved names before registering panel data.
     * @param manager Dynamics manager holding this panel's declaration.
     */
    void applyResolvedNames(DynParamManager& manager);

    static uint64_t effectorID;        //!< [] ID number of this panel
    double u;                        //!< [N-m] optional motor torque

    /** @brief Assign current inertial property names to one attachment.
     * @param effector Dynamic effector attached to this panel.
     */
    void assignStateParamNames(DynamicEffector* effector);

    // Terms needed for Backsubstitution
    Eigen::Vector3d aTheta;         //!< term needed for Backsubstitution
    Eigen::Vector3d bTheta;         //!< term needed for Backsubstitution
    double cTheta;                  //!< term needed for Backsubstitution

    // Vector quantities
    Eigen::Vector3d r_HP_P;          //!< [m] vector pointing from primary body frame P origin to Hinge location.  If a single spacecraft body is modeled than P is the same as B
    Eigen::Vector3d omega_PN_S;      //!< [rad/s] omega_BN in S frame components
    Eigen::Vector3d sHat1_P;         //!< unit direction vector for the first axis of the S frame
    Eigen::Vector3d sHat2_P;         //!< unit direction vector for the second axis of the S frame
    Eigen::Vector3d sHat3_P;         //!< unit direction vector for the third axis of the S frame
    Eigen::Vector3d r_SP_P;          //!< Vector pointing from B to CoM of hinged rigid body in B frame components
    Eigen::Vector3d rPrime_SP_P;     //!< [m/s] Body time derivative of rSB_B
    Eigen::Vector3d omegaLoc_PN_P;  //!< [rad/s] local copy of omegaBN
    Eigen::Vector3d omega_BN_B{0.0, 0.0, 0.0};  //!< Hub/Inertial angular velocity vector in B frame components

    // Matrix quantities
    Eigen::Matrix3d dcm_HP;          //!< DCM from primary body frame to hinge frame
    Eigen::Matrix3d dcm_SH;          //!< DCM from hinge to hinged rigid body frame, S
    Eigen::Matrix3d dcm_SP;          //!< DCM from body to S frame
    Eigen::Matrix3d rTilde_HP_P;     //!< Tilde matrix of rHB_B
    Eigen::Matrix3d rTilde_SP_P;     //!< Tilde matrix of rSB_B
    Eigen::Matrix3d rPrimeTilde_SP_P;  //!< Tilde matrix of rPrime_SB_B
    Eigen::Matrix3d ISPrimePntS_P;   //!< [kg-m^2/s] time body derivative IPntS in body frame components
    Eigen::Matrix3d omegaTildeLoc_PN_P; //!< tilde matrix of omegaBN
    Eigen::MRPd sigma_BN{0.0, 0.0, 0.0};       //!< Hub/Inertial attitude represented by MRP

    // Hinged rigid body properties
    Eigen::Vector3d r_SN_N;          //!< [m] position vector of hinge CM S relative to inertial frame N
    Eigen::Vector3d v_SN_N;          //!< [m/s] inertial velocity vector of S relative to inertial frame N

    Eigen::MatrixXd* r_HN_N;     //!< [m] position vector of hinge point H relative to inertial frame
    Eigen::MatrixXd* v_HN_N;     //!< [m/s] inertial velocity vector of H relative to inertial frame

    Eigen::MatrixXd* sigma_SN;        //!< MRP attitude of panel frame S relative to inertial frame
    Eigen::MatrixXd* omega_SN_S;      //!< [rad/s] inertial panel frame angular velocity vector

    // Hub properties
    StateData* hubSigmaState = nullptr;  //!< hub attitude state, read live for the published kinematics
    Eigen::MatrixXd* inertialPositionProperty;  //!< [m] r_N inertial position relative to system spice zeroBase/refBase
    Eigen::MatrixXd* inertialVelocityProperty;  //!< [m] v_N inertial velocity relative to system spice zeroBase/refBase

    Eigen::MatrixXd* c_B;            //!< [m] Vector from point B to CoM of s/c in B frame components
    Eigen::MatrixXd* cPrime_B;       //!< [m/s] Body time derivative of vector c_B in B frame components

    // States
    double theta;                    //!< [rad] hinged rigid body angle
    double thetaDot;                 //!< [rad/s] hinged rigid body angle rate
    StateData* thetaState;           //!< state manager of theta for hinged rigid body
    StateData* thetaDotState;        //!< state manager of thetaDot for hinged rigid body

public:
    HingedRigidBodyStateEffector();  //!< Constructor
    ~HingedRigidBodyStateEffector();  //!< Destructor
    void Reset(uint64_t CurrentSimNanos) override;
    void writeOutputStateMessages(uint64_t CurrentClock) override;
	void UpdateState(uint64_t CurrentSimNanos) override;
    void registerStates(DynParamManager& statesIn) override;  //!< Method for registering the HRB states
    void linkInStates(DynParamManager& states) override;  //!< Method for getting access to other states
    void addDynamicEffector(DynamicEffector *newDynamicEffector, int segment = 1) override;  //!< Method for adding attached dynamic effector
    void registerProperties(DynParamManager& states) override;       //!< Method for registering the HRB inertial properties
    void updateContributions(double integTime, BackSubMatrices & backSubContr, Eigen::MRPd sigma_BN, Eigen::Vector3d omega_BN_B, Eigen::Vector3d g_N) override;  //!< Method for back-sub contributions
    void computeDerivatives(double integTime, Eigen::Vector3d rDDot_BN_N, Eigen::Vector3d omegaDot_BN_B, Eigen::MRPd sigma_BN) override;  //!< Method for HRB to compute its derivatives
    void updateEffectorMassProps(double integTime) override;  //!< Method for giving the s/c the HRB mass props and prop rates
    void updateEnergyMomContributions(double integTime, Eigen::Vector3d & rotAngMomPntCContr_B, double & rotEnergyContr, Eigen::Vector3d omega_BN_B) override; //!< Computing energy and momentum for HRBs
    void calcForceTorqueOnBody(double integTime, Eigen::Vector3d omega_BN_B) override;  //!< Force and torque on s/c due to HRBs
    void prependSpacecraftNameToStates() override; //!< class method

#ifndef SWIG
    /** @brief Bind attached effectors after the panel properties have been registered.
     * @param manager Dynamics manager containing this panel's properties.
     */
    void bindAttachedDynamicEffectors(DynParamManager& manager) override;

protected:
    /** @brief Declare the panel's two state names and four inertial property names.
     * @return Group with a shared automatic index and independently tracked custom names.
     */
    EffectorNameGroup describeEffectorNames() const override;
#endif

private:
    void validateConfiguration(); //!< Validate mass and the configured hinge-frame DCM
    void computePanelInertialStates();
};


#endif /* STATE_EFFECTOR_H */
