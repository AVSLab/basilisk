/*
 ISC License

 Copyright (c) 2022, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#ifndef SPINNING_BODY_ONE_DOF_STATE_EFFECTOR_H
#define SPINNING_BODY_ONE_DOF_STATE_EFFECTOR_H

#include <Eigen/Dense>
#include "simulation/dynamics/_GeneralModuleFiles/stateEffector.h"
#include "simulation/dynamics/_GeneralModuleFiles/stateData.h"
#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/utilities/avsEigenMRP.h"

#include "architecture/msgPayloadDefC/ArrayMotorTorqueMsgPayload.h"
#include "architecture/msgPayloadDefC/ArrayEffectorLockMsgPayload.h"
#include "architecture/msgPayloadDefC/SCStatesMsgPayload.h"
#include "architecture/msgPayloadDefC/HingedRigidBodyMsgPayload.h"
#include "architecture/messaging/messaging.h"

#include "architecture/utilities/bskLogging.h"

/*! @brief spinning body state effector class */
class SpinningBodyOneDOFStateEffector: public StateEffector, public SysModel {

public:
    double mass = 1.0;                                               //!< [kg] mass of spinning body
    double k = 0.0;                                                  //!< [N-m/rad] torsional spring constant
    double c = 0.0;                                                  //!< [N-m-s/rad] rotational damping coefficient
    double thetaInit = 0.0;                                          //!< [rad] initial spinning body angle
    double thetaDotInit = 0.0;                                       //!< [rad/s] initial spinning body angle rate
    std::string nameOfThetaState;                                    //!< -- identifier for the theta state data container
    std::string nameOfThetaDotState;                                 //!< -- identifier for the thetaDot state data container
    Eigen::Vector3d r_SB_B{0.0, 0.0, 0.0};                  //!< [m] vector pointing from body frame B origin to spinning frame S origin in B frame components
    Eigen::Vector3d r_ScS_S{0.0, 0.0, 0.0};                 //!< [m] vector pointing from spinning frame S origin to point Sc (center of mass of the spinner) in S frame components
    Eigen::Vector3d sHat_S{1.0, 0.0, 0.0};                  //!< -- spinning axis in S frame components.
    Eigen::Matrix3d IPntSc_S;                                        //!< [kg-m^2] Inertia of spinning body about point Sc in S frame components
    Eigen::Matrix3d dcm_S0B;                                         //!< -- DCM from the body frame to the S0 frame (S frame for theta=0)
    Message<HingedRigidBodyMsgPayload> spinningBodyOutMsg;           //!< state output message
    Message<SCStatesMsgPayload> spinningBodyConfigLogOutMsg;         //!< spinning body state config log message
    ReadFunctor<ArrayMotorTorqueMsgPayload> motorTorqueInMsg;        //!< -- (optional) motor torque input message
    ReadFunctor<ArrayEffectorLockMsgPayload> motorLockInMsg;         //!< -- (optional) motor lock flag input message
    ReadFunctor<HingedRigidBodyMsgPayload> spinningBodyRefInMsg;     //!< -- (optional) spinning body reference input message name

    SpinningBodyOneDOFStateEffector();  //!< -- Contructor
    ~SpinningBodyOneDOFStateEffector() override; //!< -- Destructor
    void Reset(uint64_t CurrentClock) override;  //!< -- Method for reset
    void writeOutputStateMessages(uint64_t CurrentClock) override;   //!< -- Method for writing the output messages
    void UpdateState(uint64_t CurrentSimNanos) override;             //!< -- Method for updating information
    void registerStates(DynParamManager& statesIn) override;         //!< -- Method for registering the SB states
    void linkInStates(DynParamManager& states) override;             //!< -- Method for getting access to other states
    void updateContributions(double integTime,
                             BackSubMatrices& backSubContr, Eigen::Vector3d sigma_BN,
                             Eigen::Vector3d omega_BN_B, Eigen::Vector3d g_N) override;   //!< -- Method for back-substitution contributions
    void computeDerivatives(double integTime, Eigen::Vector3d rDDot_BN_N,
                            Eigen::Vector3d omegaDot_BN_B, Eigen::Vector3d sigma_BN) override;                         //!< -- Method for SB to compute its derivatives
    void updateEffectorMassProps(double integTime) override;         //!< -- Method for giving the s/c the HRB mass props and prop rates
    void updateEnergyMomContributions(double integTime, Eigen::Vector3d& rotAngMomPntCContr_B,
                                      double& rotEnergyContr, Eigen::Vector3d omega_BN_B) override;         //!< -- Method for computing energy and momentum for SBs
    void prependSpacecraftNameToStates() override;                   //!< Method used for multiple spacecraft
    void computeSpinningBodyInertialStates();               //!< Method for computing the SB's states

private:
    static uint64_t effectorID;         //!< [] ID number of this panel
    double u = 0.0;                     //!< [N-m] optional motor torque
    int lockFlag = 0;                   //!< [] flag for locking the rotation axis
    double thetaRef = 0.0;              //!< [rad] spinning body reference angle
    double thetaDotRef = 0.0;           //!< [rad] spinning body reference angle rate

    // Terms needed for back substitution
    Eigen::Vector3d aTheta{0.0, 0.0, 0.0};             //!< -- rDDot_BN term for back substitution
    Eigen::Vector3d bTheta{0.0, 0.0, 0.0};             //!< -- omegaDot_BN term for back substitution
    double cTheta = 0.0;                                        //!< -- scalar term for back substitution
    double mTheta = 0.0;                                        //!< -- auxiliary term for back substitution

    // Vector quantities
    Eigen::Vector3d sHat_B{1.0, 0.0, 0.0};             //!< -- spinning axis in B frame components
    Eigen::Vector3d r_ScS_B{0.0, 0.0, 0.0};            //!< [m] vector pointing from spinning frame S origin to point Sc in B frame components
    Eigen::Vector3d r_ScB_B{0.0, 0.0, 0.0};            //!< [m] vector pointing from body frame B origin to point Sc in B frame components.
    Eigen::Vector3d rPrime_ScS_B{0.0, 0.0, 0.0};       //!< [m/s] body frame time derivative of r_ScS_B
    Eigen::Vector3d rPrime_ScB_B{0.0, 0.0, 0.0};       //!< [m/s] body frame time derivative of r_ScB_B
    Eigen::Vector3d rDot_ScB_B{0.0, 0.0, 0.0};         //!< [m/s] inertial frame time derivative of r_ScB_B
    Eigen::Vector3d omega_SB_B{0.0, 0.0, 0.0};         //!< [rad/s] angular velocity of the S frame wrt the B frame in B frame components.
    Eigen::Vector3d omega_BN_B{0.0, 0.0, 0.0};         //!< [rad/s] angular velocity of the B frame wrt the N frame in B frame components.
    Eigen::Vector3d omega_SN_B{0.0, 0.0, 0.0};         //!< [rad/s] angular velocity of the S frame wrt the N frame in B frame components.
    Eigen::MRPd sigma_BN{0.0, 0.0, 0.0};               //!< -- body frame attitude wrt to the N frame in MRPs

    // Matrix quantities
    Eigen::Matrix3d rTilde_ScB_B;       //!< [m] tilde matrix of r_ScB_B
    Eigen::Matrix3d omegaTilde_SB_B;    //!< [rad/s] tilde matrix of omega_SB_B
    Eigen::Matrix3d omegaTilde_BN_B;    //!< [rad/s] tilde matrix of omega_BN_B
    Eigen::Matrix3d dcm_BS;             //!< -- DCM from spinner frame to body frame
    Eigen::Matrix3d dcm_BN;             //!< -- DCM from inertial frame to body frame
    Eigen::Matrix3d IPntSc_B;           //!< [kg-m^2] inertia of spinning body about point Sc in B frame components

    // Spinning body properties
    Eigen::Vector3d r_ScN_N{0.0, 0.0, 0.0};            //!< [m] position vector of spinning body center of mass Sc relative to the inertial frame origin N
    Eigen::Vector3d v_ScN_N{0.0, 0.0, 0.0};            //!< [m/s] inertial velocity vector of Sc relative to inertial frame
    Eigen::Vector3d sigma_SN{0.0, 0.0, 0.0};           //!< -- MRP attitude of frame S relative to inertial frame
    Eigen::Vector3d omega_SN_S{0.0, 0.0, 0.0};         //!< [rad/s] inertial spinning body frame angular velocity vector

    // States
    double theta = 0.0;                           //!< [rad] spinning body angle
    double thetaDot = 0.0;                        //!< [rad/s] spinning body angle rate
    Eigen::MatrixXd* inertialPositionProperty = nullptr;  //!< [m] r_N inertial position relative to system spice zeroBase/refBase
    Eigen::MatrixXd* inertialVelocityProperty = nullptr;  //!< [m] v_N inertial velocity relative to system spice zeroBase/refBase
    StateData *thetaState = nullptr;              //!< -- state manager of theta for spinning body
    StateData *thetaDotState = nullptr;           //!< -- state manager of thetaDot for spinning body
};


#endif /* SPINNING_BODY_ONE_DOF_STATE_EFFECTOR_H */
