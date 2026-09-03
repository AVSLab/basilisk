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

#ifndef N_HINGED_RIGID_BODY_STATE_EFFECTOR_H
#define N_HINGED_RIGID_BODY_STATE_EFFECTOR_H

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/utilities/avsEigenMRP.h"
#include "architecture/utilities/bskLogging.h"
#include "simulation/dynamics/_GeneralModuleFiles/dynamicEffector.h"
#include "simulation/dynamics/_GeneralModuleFiles/stateData.h"
#include "simulation/dynamics/_GeneralModuleFiles/stateEffector.h"
#include <Eigen/Dense>
#include <cstdint>
#include <string>
#include <vector>

#include "architecture/messaging/messaging.h"
#include "architecture/msgPayloadDefC/HingedRigidBodyMsgPayload.h"
#include "architecture/msgPayloadDefC/SCStatesMsgPayload.h"

/*! Struct containing all the panel variables. All members are public by default so they can be changed by methods of
 the N_hingedRigidBodyStateEffector class. */
struct HingedPanel {
    double mass = 1.0;               //!< [kg] mass of hinged panel
    double d = 1.0;                  //!< [m] distance from hinge point to hinged rigid body center of mass
    double k = 1.0;                  //!< [N-m/rad] torsional spring constant of hinge
    double c = 0.0;                  //!< [N-m-s/rad] rotational damping coefficient of hinge
    double thetaInit = 0.0;          //!< [rad] Initial hinged rigid body angle
    double thetaDotInit = 0.0;       //!< [rad/s] Initial hinged rigid body angle rate
    Eigen::Matrix3d IPntS_S;         //!< [kg-m^2] Inertia of hinged rigid body about point S in S frame components
    double theta = 0.0;              //!< [rad] hinged rigid body angle
    double theta_0 = 0.0;            //!< [rad] hinged rigid body rest angle
    double thetaDot = 0.0;           //!< [rad/s] hinged rigid body angle rate
    Eigen::Matrix3d dcm_SB;          //!< DCM from body to S frame
    Eigen::Vector3d omega_BN_S;      //!< [rad/s] omega_BN in S frame components
    Eigen::Vector3d omega_SB_B;      //!< [rad/s] omega_SB in B frame components
    Eigen::Vector3d sHat1_B;         //!< unit direction vector for the first axis of the S frame
    Eigen::Vector3d sHat2_B;         //!< unit direction vector for the second axis of the S frame
    Eigen::Vector3d sHat3_B;         //!< unit direction vector for the third axis of the S frame
    Eigen::Vector3d r_HB_B;          //!< Vector pointing from B to this panel's hinge in B frame components
    Eigen::Vector3d rPrime_HB_B;     //!< [m/s] Body time derivative of this panel's r_HB_B
    Eigen::Vector3d r_SB_B;          //!< Vector pointing from B to CoM of hinged rigid body in B frame components
    Eigen::Matrix3d rTilde_SB_B;     //!< Tilde matrix of rSB_B
    Eigen::Vector3d rPrime_SB_B;     //!< [m/s] Body time derivative of rSB_B
    Eigen::Matrix3d rPrimeTilde_SB_B;//!< Tilde matrix of rPrime_SB_B
    Eigen::Matrix3d ISPrimePntS_B;   //!< [kg-m^2/s] time body derivative IPntS in body frame components

    Eigen::Vector3d r_ScN_N;    //!< [m] position vector of the panel CoM S relative to the inertial frame
    Eigen::Vector3d v_ScN_N;    //!< [m/s] inertial velocity vector of S relative to the inertial frame

    std::string nameOfInertialPositionProperty;    //!< identifier for the inertial position property
    std::string nameOfInertialVelocityProperty;    //!< identifier for the inertial velocity property
    std::string nameOfInertialAttitudeProperty;    //!< identifier for the inertial attitude property
    std::string nameOfInertialAngVelocityProperty; //!< identifier for the inertial angular velocity property

    Eigen::MatrixXd* r_HN_N = nullptr;     //!< [m] position vector of the panel hinge H relative to the inertial frame
    Eigen::MatrixXd* v_HN_N = nullptr;     //!< [m/s] inertial velocity vector of H relative to the inertial frame
    Eigen::MatrixXd* sigma_SN = nullptr;   //!< MRP attitude of panel frame S relative to the inertial frame
    Eigen::MatrixXd* omega_SN_S = nullptr; //!< [rad/s] inertial panel frame angular velocity vector

    std::vector<DynamicEffector*> dynEffectors; //!< Vector of dynamic effectors attached to this panel
    Eigen::Vector3d extForce_B;                 //!< [N] attached effector force on this panel in B frame components
    Eigen::Vector3d extTorquePntH_B; //!< [N-m] attached effector torque on this panel about H in B frame components

    /**
     * @brief Assign this panel's state-engine property names to an attached effector.
     * @tparam Type Pointer type for an effector that accepts inertial property names.
     * @param effector Effector that receives the panel's inertial property names.
     */
    template<typename Type>
    void assignStateParamNames(Type effector)
    {
        effector->setPropName_inertialPosition(this->nameOfInertialPositionProperty);
        effector->setPropName_inertialVelocity(this->nameOfInertialVelocityProperty);
        effector->setPropName_inertialAttitude(this->nameOfInertialAttitudeProperty);
        effector->setPropName_inertialAngVelocity(this->nameOfInertialAngVelocityProperty);
    };
};

/*! @brief NHingedRigidBodyStateEffector class */
class NHingedRigidBodyStateEffector final : public StateEffector, public SysModel {
public:
    std::string nameOfThetaState;    //!< Identifier for the theta state data container
    std::string nameOfThetaDotState; //!< Identifier for the thetaDot state data container
    Eigen::Vector3d r_HB_B;          //!< [m] vector pointing from body frame origin to the first Hinge location
    Eigen::Matrix3d rTilde_HB_B;     //!< Tilde matrix of rHB_B
    Eigen::Matrix3d dcm_HB;          //!< DCM from body frame to hinge frame
    void addHingedPanel(HingedPanel NewPanel); //!< class method
    BSKLogger bskLogger;                      //!< BSK Logging
    std::vector<Message<HingedRigidBodyMsgPayload>*> nHingedRigidBodyOutMsgs;   //!< panel state output messages
    std::vector<Message<SCStatesMsgPayload>*> nHingedRigidBodyConfigLogOutMsgs; //!< panel config log messages

private:
    double totalMass;                //!< [kg] Total mass of effector
    StateData* thetaState;           //!< state manager of theta for hinged rigid body
    StateData* thetaDotState;        //!< state manager of thetaDot for hinged rigid body
    std::vector<HingedPanel> PanelVec; //!< vector containing all the info on the different panels
    Eigen::MatrixXd matrixADHRB;    //!< [-] term needed for back substitution
    Eigen::MatrixXd matrixEDHRB;    //!< [-] term needed for back substitution
    Eigen::MatrixX3d matrixFDHRB;   //!< [-] term needed for back substitution
    Eigen::MatrixX3d matrixGDHRB;   //!< [-] term needed for back substitution
    Eigen::VectorXd vectorVDHRB;    //!< [-] term needed for back substitution
    Eigen::Vector3d omegaLoc_BN_B;  //!< [rad/s] local copy of omegaBN
    Eigen::MRPd sigma_BN{0.0, 0.0, 0.0};       //!< Hub attitude relative to the inertial frame
    StateData* hubSigmaState = nullptr;        //!< hub attitude state, read live for the published kinematics
    Eigen::MatrixXd* inertialPositionProperty = nullptr; //!< [m] r_N position relative to system spice zeroBase
    Eigen::MatrixXd* inertialVelocityProperty = nullptr; //!< [m/s] v_N velocity relative to system spice zeroBase
    Eigen::MatrixXd* g_N;           //!< [m/s^2] Gravitational acceleration in N frame components
    std::string propertyNameIndex;  //!< effector identifier used to name the per panel properties
    bool hasAttachedEffectors = false; //!< true once any panel carries a dynamic effector
    static uint64_t effectorID;        //!< [] ID number of this panel

public:
    NHingedRigidBodyStateEffector();  //!< Constructor
    ~NHingedRigidBodyStateEffector();  //!< Destructor
    double HeaviFunc(double cond); //!< Heaviside function used for matrix contributions
    void writeOutputStateMessages(uint64_t CurrentClock) override;
    void UpdateState(uint64_t CurrentSimNanos) override;
    void registerStates(DynParamManager& statesIn) override; //!< Method for registering the HRB states
    void registerProperties(DynParamManager& states) override; //!< Method for registering the panel properties
    void addDynamicEffector(DynamicEffector* newDynamicEffector, int segment) override; //!< Attach an effector
    void linkInStates(DynParamManager& states) override;     //!< Method for getting access to other states
    void updateEffectorMassProps(double integTime) override; //!< Method for stateEffector to give mass contributions
    void updateContributions(double integTime,
                             BackSubMatrices& backSubContr,
                             Eigen::MRPd sigma_BN,
                             Eigen::Vector3d omega_BN_B,
                             Eigen::Vector3d g_N) override; //!< Back-sub contributions
    void updateEnergyMomContributions(double integTime,
                                      Eigen::Vector3d& rotAngMomPntCContr_B,
                                      double& rotEnergyContr,
                                      Eigen::Vector3d omega_BN_B) override; //!< Energy and momentum calculations
    void computeDerivatives(double integTime,
                            Eigen::Vector3d rDDot_BN_N,
                            Eigen::Vector3d omegaDot_BN_B,
                            Eigen::MRPd sigma_BN) override; //!< Method for computing the effector derivatives

private:
    void checkPanelUniformity();       //!< Method for rejecting a panel chain the EOMs cannot represent
    void computePanelInertialStates(); //!< Method for computing the panel states relative to the inertial frame
};


#endif /* N_HINGED_RIGID_BODY_STATE_EFFECTOR_H */
