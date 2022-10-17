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

#ifndef SPINNING_BODY_TWO_DOF_STATE_EFFECTOR_H
#define SPINNING_BODY_TWO_DOF_STATE_EFFECTOR_H

#include <Eigen/Dense>
#include "simulation/dynamics/_GeneralModuleFiles/stateEffector.h"
#include "simulation/dynamics/_GeneralModuleFiles/stateData.h"
#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/utilities/avsEigenMRP.h"

#include "architecture/msgPayloadDefC/ArrayMotorTorqueMsgPayload.h"
#include "architecture/msgPayloadDefC/SCStatesMsgPayload.h"
#include "architecture/msgPayloadDefC/SpinningBodyTwoDOFMsgPayload.h"
#include "architecture/messaging/messaging.h"

#include "architecture/utilities/bskLogging.h"

/*! @brief spinning body state effector class */
class SpinningBodyTwoDOFStateEffector: public StateEffector, public SysModel {
public:
    double mass1;                                                //!< [kg] mass of spinning body
    double mass2;                                                //!< [kg] mass of spinning body
    double theta1Init;                                           //!< [rad] initial spinning body angle
    double theta1DotInit;                                        //!< [rad/s] initial spinning body angle rate
    double theta2Init;                                           //!< [rad] initial spinning body angle
    double theta2DotInit;                                        //!< [rad/s] initial spinning body angle rate
    std::string nameOfTheta1State;                               //!< -- identifier for the theta state data container
    std::string nameOfTheta1DotState;                            //!< -- identifier for the thetaDot state data container
    std::string nameOfTheta2State;                               //!< -- identifier for the theta state data container
    std::string nameOfTheta2DotState;                            //!< -- identifier for the thetaDot state data container
    Eigen::Vector3d r_S1B_B;                                     //!< [m] vector pointing from body frame B origin to spinning frame S origin in B frame components
    Eigen::Vector3d r_S2S1_S1;                                     //!< [m] vector pointing from body frame B origin to spinning frame S origin in B frame components
    Eigen::Vector3d r_Sc1S1_S1;                                    //!< [m] vector pointing from spinning frame S origin to point Sc (center of mass of the spinner) in S frame components
    Eigen::Vector3d r_Sc2S2_S2;                                    //!< [m] vector pointing from spinning frame S origin to point Sc (center of mass of the spinner) in S frame components
    Eigen::Vector3d s1Hat_S1;                                     //!< -- spinning axis in S frame components.
    Eigen::Vector3d s2Hat_S2;                                     //!< -- spinning axis in S frame components.
    Eigen::Matrix3d IPntSc1_S1;                                   //!< [kg-m^2] Inertia of spinning body about point Sc in S frame components
    Eigen::Matrix3d IPntSc2_S2;                                   //!< [kg-m^2] Inertia of spinning body about point Sc in S frame components
    Eigen::Matrix3d dcm_S10B;                                    //!< -- DCM from the body frame to the S0 frame (S frame for theta=0)
    Eigen::Matrix3d dcm_S20S1;                                    //!< -- DCM from the body frame to the S0 frame (S frame for theta=0)
    Message<SpinningBodyTwoDOFMsgPayload> spinningBodyOutMsg;         //!< state output message
    std::vector<Message<SCStatesMsgPayload>> spinningBodyConfigLogOutMsg;    //!< spinning body state config log message
    ReadFunctor<ArrayMotorTorqueMsgPayload> motorTorqueInMsg; //!< -- (optional) motor torque input message name
    BSKLogger bskLogger;                                        //!< -- BSK Logging

private:
    static uint64_t effectorID;         //!< [] ID number of this panel
    double u1;                           //!< [N-m] optional motor torque
    double u2;

    // Terms needed for back substitution
    Eigen::Matrix<double, 2, 3> ATheta;             //!< -- rDDot_BN term for back substitution
    Eigen::Matrix<double, 2, 3> BTheta;             //!< -- omegaDot_BN term for back substitution
    Eigen::Vector2d CTheta;                      //!< -- scalar term for back substitution

    // Vector quantities
    Eigen::Vector3d s1Hat_B;             //!< -- spinning axis in B frame components
    Eigen::Vector3d s2Hat_B;             //!< -- spinning axis in B frame components
    Eigen::Vector3d r_Sc1S1_B;            //!< [m] vector pointing from spinning frame S origin to point Sc in B frame components
    Eigen::Vector3d r_Sc1B_B;            //!< [m] vector pointing from body frame B origin to point Sc in B frame components.
    Eigen::Vector3d r_Sc2S2_B;            //!< [m] vector pointing from spinning frame S origin to point Sc in B frame components
    Eigen::Vector3d r_S2S1_B;            //!< [m] vector pointing from spinning frame S origin to point Sc in B frame components
    Eigen::Vector3d r_Sc2B_B;            //!< [m] vector pointing from body frame B origin to point Sc in B frame components.
    Eigen::Vector3d rPrime_Sc1S1_B;       //!< [m/s] body frame time derivative of r_ScS_B
    Eigen::Vector3d rPrime_Sc2S2_B;       //!< [m/s] body frame time derivative of r_ScS_B
    Eigen::Vector3d rPrime_S2S1_B;       //!< [m/s] body frame time derivative of r_ScS_B
    Eigen::Vector3d rPrime_Sc1B_B;       //!< [m/s] body frame time derivative of r_ScB_B
    Eigen::Vector3d rPrime_Sc2B_B;       //!< [m/s] body frame time derivative of r_ScB_B
    Eigen::Vector3d rDot_Sc1B_B;         //!< [m/s] inertial frame time derivative of r_ScB_B
    Eigen::Vector3d rDot_Sc2B_B;         //!< [m/s] inertial frame time derivative of r_ScB_B
    Eigen::Vector3d omega_S1B_B;         //!< [rad/s] angular velocity of the S frame wrt the B frame in B frame components.
    Eigen::Vector3d omega_S2S1_B;         //!< [rad/s] angular velocity of the S frame wrt the B frame in B frame components.
    Eigen::Vector3d omega_S2B_B;         //!< [rad/s] angular velocity of the S frame wrt the B frame in B frame components.
    Eigen::Vector3d omega_BN_B;         //!< [rad/s] angular velocity of the B frame wrt the N frame in B frame components.
    Eigen::Vector3d omega_S1N_B;         //!< [rad/s] angular velocity of the S frame wrt the N frame in B frame components.
    Eigen::Vector3d omega_S2N_B;         //!< [rad/s] angular velocity of the S frame wrt the N frame in B frame components.
    Eigen::MRPd sigma_BN;               //!< -- body frame attitude wrt to the N frame in MRPs

    // Matrix quantities
    Eigen::Matrix3d rTilde_Sc1B_B;       //!< [m] tilde matrix of r_ScB_B
    Eigen::Matrix3d rTilde_Sc2B_B;       //!< [m] tilde matrix of r_ScB_B
    Eigen::Matrix3d omegaTilde_S1B_B;    //!< [rad/s] tilde matrix of omega_SB_B
    Eigen::Matrix3d omegaTilde_S2B_B;    //!< [rad/s] tilde matrix of omega_SB_B
    Eigen::Matrix3d omegaTilde_BN_B;    //!< [rad/s] tilde matrix of omega_BN_B
    Eigen::Matrix3d dcm_BS1;             //!< -- DCM from spinner frame to body frame
    Eigen::Matrix3d dcm_BS2;             //!< -- DCM from spinner frame to body frame
    Eigen::Matrix3d dcm_BN;             //!< -- DCM from inertial frame to body frame
    Eigen::Matrix3d IPntSc1_B;           //!< [kg-m^2] inertia of spinning body about point Sc in B frame components
    Eigen::Matrix3d IPntSc2_B;           //!< [kg-m^2] inertia of spinning body about point Sc in B frame components
    Eigen::Matrix3d IPrimePntSc1_B;           //!< [kg-m^2] inertia of spinning body about point Sc in B frame components
    Eigen::Matrix3d IPrimePntSc2_B;           //!< [kg-m^2] inertia of spinning body about point Sc in B frame components

    // Spinning body properties
    Eigen::Vector3d r_Sc1N_N;            //!< [m] position vector of spinning body center of mass Sc relative to the inertial frame origin N
    Eigen::Vector3d r_Sc2N_N;            //!< [m] position vector of spinning body center of mass Sc relative to the inertial frame origin N
    Eigen::Vector3d v_Sc1N_N;            //!< [m/s] inertial velocity vector of Sc relative to inertial frame
    Eigen::Vector3d v_Sc2N_N;            //!< [m/s] inertial velocity vector of Sc relative to inertial frame
    Eigen::Vector3d sigma_S1N;           //!< -- MRP attitude of frame S relative to inertial frame
    Eigen::Vector3d sigma_S2N;           //!< -- MRP attitude of frame S relative to inertial frame
    Eigen::Vector3d omega_S1N_S;         //!< [rad/s] inertial spinning body frame angular velocity vector
    Eigen::Vector3d omega_S2N_S;         //!< [rad/s] inertial spinning body frame angular velocity vector

    // States
    double theta1;                       //!< [rad] spinning body angle
    double theta1Dot;                    //!< [rad/s] spinning body angle rate
    double theta2;                       //!< [rad] spinning body angle
    double theta2Dot;                    //!< [rad/s] spinning body angle rate
    StateData *hubSigma;                //!< hub/inertial attitude represented by MRP
    StateData *hubOmega;                //!< hub/inertial angular velocity vector in B frame components
    StateData *hubPosition;             //!< hub/inertial position vector in inertial frame components
    StateData *hubVelocity;             //!< hub/inertial velocity vector in inertial frame components
    StateData *theta1State;              //!< -- state manager of theta for spinning body
    StateData *theta1DotState;           //!< -- state manager of thetaDot for spinning body
    StateData* theta2State;              //!< -- state manager of theta for spinning body
    StateData* theta2DotState;           //!< -- state manager of thetaDot for spinning body
    Eigen::MatrixXd *c_B;               //!< [m] vector from point B to CoM of s/c in B frame components
    Eigen::MatrixXd *cPrime_B;          //!< [m/s] body time derivative of vector c_B in B frame components

public:
    SpinningBodyTwoDOFStateEffector();    //!< -- Contructor
    ~SpinningBodyTwoDOFStateEffector();   //!< -- Destructor
    void Reset(uint64_t CurrentClock);                   //!< -- Method for reset
    void writeOutputStateMessages(uint64_t CurrentClock);   //!< -- Method for writing the output messages
	void UpdateState(uint64_t CurrentSimNanos);             //!< -- Method for updating information
    void registerStates(DynParamManager& statesIn);         //!< -- Method for registering the SB states
    void linkInStates(DynParamManager& states);             //!< -- Method for getting access to other states
    void updateContributions(double integTime, BackSubMatrices & backSubContr, Eigen::Vector3d sigma_BN, Eigen::Vector3d omega_BN_B, Eigen::Vector3d g_N);  //!< -- Method for back-substitution contributions
    void computeDerivatives(double integTime, Eigen::Vector3d rDDot_BN_N, Eigen::Vector3d omegaDot_BN_B, Eigen::Vector3d sigma_BN);                         //!< -- Method for SB to compute its derivatives
    void updateEffectorMassProps(double integTime);         //!< -- Method for giving the s/c the HRB mass props and prop rates
    void updateEnergyMomContributions(double integTime, Eigen::Vector3d & rotAngMomPntCContr_B, double & rotEnergyContr, Eigen::Vector3d omega_BN_B);       //!< -- Method for computing energy and momentum for SBs
    void prependSpacecraftNameToStates();                   //!< Method used for multiple spacecraft
    void computeSpinningBodyInertialStates();               //!< Method for computing the SB's states

private:

};


#endif /* SPINNING_BODY_2DOF_STATE_EFFECTOR_H */
