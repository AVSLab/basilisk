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
#include "architecture/msgPayloadDefC/ArrayEffectorLockMsgPayload.h"
#include "architecture/msgPayloadDefC/SCStatesMsgPayload.h"
#include "architecture/msgPayloadDefC/HingedRigidBodyMsgPayload.h"
#include "architecture/messaging/messaging.h"

#include "architecture/utilities/bskLogging.h"

/*! @brief spinning body state effector class */
class SpinningBodyTwoDOFStateEffector: public StateEffector, public SysModel {
public:
    double mass1 = 0.0;                 //!< [kg] mass of lower spinning body (can be 0)
    double mass2 = 1.0;                 //!< [kg] mass of upper spinning body
    double k1 = 0.0;                    //!< [N-m/rad] torsional spring constant for first rotation axis
    double k2 = 0.0;                    //!< [N-m/rad] torsional spring constant for second rotation axis
    double c1 = 0.0;                    //!< [N-m-s/rad] rotational damping coefficient for first rotation axis
    double c2 = 0.0;                    //!< [N-m-s/rad] rotational damping coefficient for second rotation axis
    double theta1Init = 0.0;            //!< [rad] initial first axis angle
    double theta1DotInit = 0.0;         //!< [rad/s] initial first axis angle rate
    double theta2Init = 0.0;            //!< [rad] initial second axis angle
    double theta2DotInit = 0.0;         //!< [rad/s] initial second axis angle rate
    std::string nameOfTheta1State;      //!< -- identifier for the theta1 state data container
    std::string nameOfTheta1DotState;   //!< -- identifier for the thetaDot1 state data container
    std::string nameOfTheta2State;      //!< -- identifier for the theta2 state data container
    std::string nameOfTheta2DotState;   //!< -- identifier for the thetaDot2 state data container
    Eigen::Vector3d r_S1B_B{0.0,0.0,0.0};            //!< [m] vector pointing from body frame B origin to lower spinning frame S1 origin in B frame components
    Eigen::Vector3d r_S2S1_S1{0.0,0.0,0.0};          //!< [m] vector pointing from lower spinning frame S1 origin to upper spinning frame S2 origin in S1 frame components
    Eigen::Vector3d r_Sc1S1_S1{0.0,0.0,0.0};         //!< [m] vector pointing from lower spinning frame S1 origin to point Sc1 (center of mass of the lower spinner) in S1 frame components
    Eigen::Vector3d r_Sc2S2_S2{0.0,0.0,0.0};         //!< [m] vector pointing from upper spinning frame S2 origin to point Sc2 (center of mass of the upper spinner) in S2 frame components
    Eigen::Vector3d s1Hat_S1{1.0,0.0,0.0};           //!< -- first spinning axis in S1 frame components.
    Eigen::Vector3d s2Hat_S2{1.0,0.0,0.0};           //!< -- second spinning axis in S2 frame components.
    Eigen::Matrix3d IS1PntSc1_S1;       //!< [kg-m^2] Inertia of lower spinning body about point Sc1 in S1 frame components
    Eigen::Matrix3d IS2PntSc2_S2;       //!< [kg-m^2] Inertia of upper spinning body about point Sc2 in S2 frame components
    Eigen::Matrix3d dcm_S10B;           //!< -- DCM from the body frame to the S10 frame (S1 frame for theta1=0)
    Eigen::Matrix3d dcm_S20S1;          //!< -- DCM from the S1 frame to the S20 frame (S2 frame for theta2=0)
    std::vector<Message<HingedRigidBodyMsgPayload>*> spinningBodyOutMsgs {new Message<HingedRigidBodyMsgPayload>,
            new Message<HingedRigidBodyMsgPayload>};       //!< vector of state output messages
    std::vector<Message<SCStatesMsgPayload>*> spinningBodyConfigLogOutMsgs {new Message<SCStatesMsgPayload>,
            new Message<SCStatesMsgPayload>};     //!< vector of spinning body state config log messages
    ReadFunctor<ArrayMotorTorqueMsgPayload> motorTorqueInMsg;                   //!< -- (optional) motor torque input message name
    ReadFunctor<ArrayEffectorLockMsgPayload> motorLockInMsg;                    //!< -- (optional) motor lock input message name
    std::vector<ReadFunctor<HingedRigidBodyMsgPayload>> spinningBodyRefInMsgs {ReadFunctor<HingedRigidBodyMsgPayload>(),
            ReadFunctor<HingedRigidBodyMsgPayload>()};    //!< (optional) vector of spinning body reference input messages

    SpinningBodyTwoDOFStateEffector();      //!< -- Contructor
    ~SpinningBodyTwoDOFStateEffector();     //!< -- Destructor
    void Reset(uint64_t CurrentClock);      //!< -- Method for reset
    void writeOutputStateMessages(uint64_t CurrentClock);   //!< -- Method for writing the output messages
    void UpdateState(uint64_t CurrentSimNanos);             //!< -- Method for updating information
    void registerStates(DynParamManager& statesIn);         //!< -- Method for registering the SB states
    void linkInStates(DynParamManager& states);             //!< -- Method for getting access to other states
    void updateContributions(double integTime,
                             BackSubMatrices& backSubContr,
                             Eigen::Vector3d sigma_BN,
                             Eigen::Vector3d omega_BN_B,
                             Eigen::Vector3d g_N);  //!< -- Method for back-substitution contributions
    void computeDerivatives(double integTime,
                            Eigen::Vector3d rDDot_BN_N,
                            Eigen::Vector3d omegaDot_BN_B,
                            Eigen::Vector3d sigma_BN);                         //!< -- Method for SB to compute its derivatives
    void updateEffectorMassProps(double integTime);         //!< -- Method for giving the s/c the HRB mass props and prop rates
    void updateEnergyMomContributions(double integTime,
                                      Eigen::Vector3d& rotAngMomPntCContr_B,
                                      double& rotEnergyContr,
                                      Eigen::Vector3d omega_BN_B);       //!< -- Method for computing energy and momentum for SBs
    void prependSpacecraftNameToStates();                   //!< Method used for multiple spacecraft
    void computeSpinningBodyInertialStates();               //!< Method for computing the SB's states

private:
    static uint64_t effectorID;     //!< [] ID number of this panel
    double u1 = 0.0;                //!< [N-m] optional motor torque for first axis
    double u2 = 0.0;                //!< [N-m] optional motor torque for second axis
    int lockFlag1 = 0;              //!< [] flag for locking the first rotation axis
    int lockFlag2 = 0;              //!< [] flag for locking the second rotation axis
    double theta1Ref = 0.0;         //!< [rad] spinning body reference angle
    double theta1DotRef = 0.0;      //!< [rad] spinning body reference angle rate
    double theta2Ref = 0.0;         //!< [rad] spinning body reference angle
    double theta2DotRef = 0.0;      //!< [rad] spinning body reference angle rate
    double mass = 1.0;              //!< [kg] mass of the spinner system

    // Terms needed for back substitution
    Eigen::Matrix<double, 2, 3> ATheta;     //!< -- rDDot_BN term for back substitution
    Eigen::Matrix<double, 2, 3> BTheta;     //!< -- omegaDot_BN term for back substitution
    Eigen::Vector2d CTheta{0.0,0.0};                 //!< -- scalar term for back substitution

    // Vector quantities
    Eigen::Vector3d s1Hat_B{1.0,0.0,0.0};            //!< -- first spinning axis in B frame components
    Eigen::Vector3d s2Hat_B{1.0,0.0,0.0};            //!< -- second spinning axis in B frame components
    Eigen::Vector3d r_Sc1S1_B{0.0,0.0,0.0};          //!< [m] vector pointing from lower spinning frame S1 origin to point Sc1 in B frame components
    Eigen::Vector3d r_Sc1B_B{0.0,0.0,0.0};           //!< [m] vector pointing from body frame B origin to point Sc1 in B frame components.
    Eigen::Vector3d r_Sc2S2_B{0.0,0.0,0.0};          //!< [m] vector pointing from upper spinning frame S2 origin to point Sc2 in B frame components
    Eigen::Vector3d r_S2S1_B{0.0,0.0,0.0};           //!< [m] vector pointing from lower spinning frame S1 origin to upper spinning frame S2 origin in B frame components
    Eigen::Vector3d r_Sc2S1_B{0.0,0.0,0.0};          //!< [m] vector pointing from lower spinning frame S1 origin to point Sc2 in B frame components
    Eigen::Vector3d r_Sc2B_B{0.0,0.0,0.0};           //!< [m] vector pointing from body frame B origin to point Sc2 in B frame components.
    Eigen::Vector3d r_ScB_B{0.0,0.0,0.0};            //!< [m] vector pointing from body frame B origin to point Sc (center of mass of the spinner system) in B frame components.
    Eigen::Vector3d rPrime_Sc1S1_B{0.0,0.0,0.0};     //!< [m/s] body frame time derivative of r_Sc1S1_B
    Eigen::Vector3d rPrime_Sc2S2_B{0.0,0.0,0.0};     //!< [m/s] body frame time derivative of r_Sc2S2_B
    Eigen::Vector3d rPrime_S2S1_B{0.0,0.0,0.0};      //!< [m/s] body frame time derivative of r_S2S1_B
    Eigen::Vector3d rPrime_Sc1B_B{0.0,0.0,0.0};      //!< [m/s] body frame time derivative of r_Sc1B_B
    Eigen::Vector3d rPrime_Sc2B_B{0.0,0.0,0.0};      //!< [m/s] body frame time derivative of r_Sc2B_B
    Eigen::Vector3d rPrime_Sc2S1_B{0.0,0.0,0.0};     //!< [m/s] body frame time derivative of r_Sc2S1_B
    Eigen::Vector3d rPrime_ScB_B{0.0,0.0,0.0};       //!< [m/s] body frame time derivative of r_ScB_B
    Eigen::Vector3d rDot_Sc1B_B{0.0,0.0,0.0};        //!< [m/s] inertial frame time derivative of r_Sc1B_B
    Eigen::Vector3d rDot_Sc2B_B{0.0,0.0,0.0};        //!< [m/s] inertial frame time derivative of r_Sc2B_B
    Eigen::Vector3d omega_S1B_B{0.0,0.0,0.0};        //!< [rad/s] angular velocity of the S1 frame wrt the B frame in B frame components
    Eigen::Vector3d omega_S2S1_B{0.0,0.0,0.0};       //!< [rad/s] angular velocity of the S2 frame wrt the S1 frame in B frame components
    Eigen::Vector3d omega_S2B_B{0.0,0.0,0.0};        //!< [rad/s] angular velocity of the S2 frame wrt the B frame in B frame components
    Eigen::Vector3d omega_BN_B{0.0,0.0,0.0};         //!< [rad/s] angular velocity of the B frame wrt the N frame in B frame components
    Eigen::Vector3d omega_S1N_B{0.0,0.0,0.0};        //!< [rad/s] angular velocity of the S1 frame wrt the N frame in B frame components
    Eigen::Vector3d omega_S2N_B{0.0,0.0,0.0};        //!< [rad/s] angular velocity of the S2 frame wrt the N frame in B frame components
    Eigen::MRPd sigma_BN{0.0,0.0,0.0};               //!< -- body frame attitude wrt to the N frame in MRPs

    // Matrix quantities
    Eigen::Matrix3d rTilde_Sc1B_B;      //!< [m] tilde matrix of r_Sc1B_B
    Eigen::Matrix3d rTilde_Sc2B_B;      //!< [m] tilde matrix of r_Sc2B_B
    Eigen::Matrix3d omegaTilde_S1B_B;   //!< [rad/s] tilde matrix of omega_S1B_B
    Eigen::Matrix3d omegaTilde_S2B_B;   //!< [rad/s] tilde matrix of omega_S2B_B
    Eigen::Matrix3d omegaTilde_BN_B;    //!< [rad/s] tilde matrix of omega_BN_B
    Eigen::Matrix3d dcm_BS1;            //!< -- DCM from lower spinner frame to body frame
    Eigen::Matrix3d dcm_BS2;            //!< -- DCM from upper spinner frame to body frame
    Eigen::Matrix3d dcm_BN;             //!< -- DCM from inertial frame to body frame
    Eigen::Matrix3d IS1PntSc1_B;        //!< [kg-m^2] inertia of lower spinning body about point Sc1 in B frame components
    Eigen::Matrix3d IS2PntSc2_B;        //!< [kg-m^2] inertia of upper spinning body about point Sc2 in B frame components
    Eigen::Matrix3d IPrimeS1PntSc1_B;   //!< [kg-m^2] body frame time derivative of inertia of inertia of lower spinning body about point Sc1 in B frame components
    Eigen::Matrix3d IPrimeS2PntSc2_B;   //!< [kg-m^2] body frame time derivative of inertia of inertia of upper spinning body about point Sc2 in B frame components

    // Spinning body properties
    Eigen::Vector3d r_Sc1N_N{0.0,0.0,0.0};           //!< [m] position vector of lower spinning body center of mass Sc1 relative to the inertial frame origin N
    Eigen::Vector3d r_Sc2N_N{0.0,0.0,0.0};           //!< [m] position vector of upper spinning body center of mass Sc2 relative to the inertial frame origin N
    Eigen::Vector3d v_Sc1N_N{0.0,0.0,0.0};           //!< [m/s] inertial velocity vector of Sc1 relative to inertial frame
    Eigen::Vector3d v_Sc2N_N{0.0,0.0,0.0};           //!< [m/s] inertial velocity vector of Sc2 relative to inertial frame
    Eigen::Vector3d sigma_S1N{0.0,0.0,0.0};          //!< -- MRP attitude of frame S1 relative to inertial frame
    Eigen::Vector3d sigma_S2N{0.0,0.0,0.0};          //!< -- MRP attitude of frame S2 relative to inertial frame
    Eigen::Vector3d omega_S1N_S1{0.0,0.0,0.0};       //!< [rad/s] inertial lower spinning body frame angular velocity vector
    Eigen::Vector3d omega_S2N_S2{0.0,0.0,0.0};       //!< [rad/s] inertial upper spinning body frame angular velocity vector

    // States
    double theta1 = 0.0;                //!< [rad] first axis angle
    double theta1Dot = 0.0;             //!< [rad/s] first axis angle rate
    double theta2 = 0.0;                //!< [rad] second axis angle
    double theta2Dot = 0.0;             //!< [rad/s] second axis angle rate
    Eigen::MatrixXd* inertialPositionProperty = nullptr;    //!< [m] r_N inertial position relative to system spice zeroBase/refBase
    Eigen::MatrixXd* inertialVelocityProperty = nullptr;    //!< [m] v_N inertial velocity relative to system spice zeroBase/refBase
    StateData *theta1State = nullptr;    //!< -- state manager of theta1 for spinning body
    StateData *theta1DotState = nullptr; //!< -- state manager of theta1Dot for spinning body
    StateData* theta2State = nullptr;    //!< -- state manager of theta2 for spinning body
    StateData* theta2DotState = nullptr; //!< -- state manager of theta2Dot for spinning body
};

#endif /* SPINNING_BODY_TWO_DOF_STATE_EFFECTOR_H */
