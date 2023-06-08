/*
 ISC License

 Copyright (c) 2023, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#ifndef PRESCRIBED_MOTION_STATE_EFFECTOR_H
#define PRESCRIBED_MOTION_STATE_EFFECTOR_H

#include "simulation/dynamics/_GeneralModuleFiles/stateEffector.h"
#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "simulation/dynamics/_GeneralModuleFiles/stateData.h"
#include "architecture/messaging/messaging.h"
#include "architecture/msgPayloadDefC/SCStatesMsgPayload.h"
#include "architecture/msgPayloadDefC/PrescribedMotionMsgPayload.h"
#include "architecture/utilities/avsEigenSupport.h"
#include "architecture/utilities/avsEigenMRP.h"

/*! @brief prescribed motion state effector class */
class PrescribedMotionStateEffector: public StateEffector, public SysModel {
public:
    PrescribedMotionStateEffector();
    ~PrescribedMotionStateEffector();
    void Reset(uint64_t CurrentClock) override;                      //!< Method for reset
    void writeOutputStateMessages(uint64_t CurrentClock) override;   //!< Method for writing the output messages
	void UpdateState(uint64_t CurrentSimNanos) override;             //!< Method for updating the effector states
    void registerStates(DynParamManager& statesIn) override;         //!< Method for registering the effector's states
    void linkInStates(DynParamManager& states) override;             //!< Method for giving the effector access to hub states
    void updateContributions(double integTime,
                             BackSubMatrices & backSubContr,
                             Eigen::Vector3d sigma_BN,
                             Eigen::Vector3d omega_BN_B,
                             Eigen::Vector3d g_N) override; //!< Method for computing the effector's back-substitution contributions
    void computeDerivatives(double integTime,
                            Eigen::Vector3d rDDot_BN_N,
                            Eigen::Vector3d omegaDot_BN_B,
                            Eigen::Vector3d sigma_BN) override; //!< Method for effector to compute its state derivatives
    void updateEffectorMassProps(double integTime) override; //!< Method for calculating the effector mass props and prop rates
    void updateEnergyMomContributions(double integTime,
                                      Eigen::Vector3d & rotAngMomPntCContr_B,
                                      double & rotEnergyContr,
                                      Eigen::Vector3d omega_BN_B) override; //!< Method for computing the energy and momentum of the effector
    void computePrescribedMotionInertialStates(); //!< Method for computing the effector's states relative to the inertial frame

    double mass;                                        //!< [kg] Effector mass
    Eigen::Matrix3d IPntFc_F;                           //!< [kg-m^2] Inertia of the effector about its center of mass in F frame components
    Eigen::Vector3d r_MB_B;                             //!< [m] Position of point M relative to point B in B frame components
    Eigen::Vector3d r_FcF_F;                            //!< [m] Position of the effector center of mass relative to point F in F frame components
    Eigen::Vector3d omega_MB_B;                         //!< [rad/s] Angular velocity of frame M with respect to frame B in B frame components
    Eigen::Vector3d omegaPrime_MB_B;                    //!< [rad/s^2] B frame time derivative of omega_MB_B in B frame components
    Eigen::MRPd sigma_MB;                               //!< MRP attitude of frame M relative to frame B

    // Prescribed parameters
    Eigen::Vector3d r_FM_M;                             //!< [m] Position of point F relative to point M in M frame components
    Eigen::Vector3d rPrime_FM_M;                        //!< [m/s] B frame time derivative of r_FM_M in M frame components
    Eigen::Vector3d rPrimePrime_FM_M;                   //!< [m/s^2] B frame time derivative of rPrime_FM_M in M frame components
    Eigen::Vector3d omega_FM_F;                         //!< [rad/s] Angular velocity of frame F relative to frame M in F frame components
    Eigen::Vector3d omegaPrime_FM_F;                    //!< [rad/s^2] B frame time derivative of omega_FM_F in F frame components
    Eigen::MRPd sigma_FM;                               //!< MRP attitude of frame F relative to frame M

    ReadFunctor<PrescribedMotionMsgPayload> prescribedMotionInMsg;      //!< Input message for the effector's prescribed states
    Message<PrescribedMotionMsgPayload> prescribedMotionOutMsg;         //!< Output message for the effector's prescribed states
    Message<SCStatesMsgPayload> prescribedMotionConfigLogOutMsg;        //!< Output config log message for the effector's states

private:
    static uint64_t effectorID;                                         //!< ID number of this panel

    // Given quantities from user in python
    Eigen::Matrix3d IPntFc_B;                           //!< [kg-m^2] Inertia of the effector about its center of mass in B frame components
    Eigen::Vector3d r_FB_B;                             //!< [m] Position of point F relative to point B in B frame components
    Eigen::Vector3d r_FcF_B;                            //!< [m] Position of the effector center of mass relative to point F in B frame components

    // Prescribed parameters in body frame components
    Eigen::Vector3d r_FM_B;                             //!< [m] Position of point F relative to point M in B frame components
    Eigen::Vector3d rPrime_FM_B;                        //!< [m/s] B frame time derivative of position r_FM_B in B frame components
    Eigen::Vector3d rPrimePrime_FM_B;                   //!< [m/s^2] B frame time derivative of rPrime_FM_B in B frame components
    Eigen::Vector3d omega_FM_B;                         //!< [rad/s] Angular velocity of F frame relative to the M frame in B frame components
    Eigen::Vector3d omegaPrime_FM_B;                    //!< [rad/s^2] B frame time derivative of omega_FB_B in B frame components
    Eigen::Vector3d omega_FB_B;                         //!< [rad/s] Angular velocity of frame F relative to frame B in B frame components
    Eigen::Vector3d omegaPrime_FB_B;                    //!< [rad/s^2] B frame time derivative of omega_FB_B in B frame components

    // Other vector quantities
    Eigen::Vector3d r_FcM_B;                            //!< [m] Position of frame F center of mass relative to point M in B frame components
    Eigen::Vector3d r_FcB_B;                            //!< [m] Position of frame F center of mass relative to point B in B frame components
    Eigen::Vector3d rPrime_FcM_B;                       //!< [m/s] B frame time derivative of r_FcM_B in B frame components
    Eigen::Vector3d rPrime_FcB_B;                       //!< [m/s] B frame time derivative of r_FcB_B in B frame components
    Eigen::Vector3d rPrimePrime_FcB_B;                  //!< [m/s^2] B frame time derivative of rPrime_FcB_B in B frame components
    Eigen::Vector3d omega_BN_B;                         //!< [rad/s] Angular velocity of frame B relative to the inertial frame in B frame components
    Eigen::Vector3d omega_FN_B;                         //!< [rad/s] Angular velocity of frame F relative to the inertial frame in B frame components
    Eigen::Vector3d rDot_FcB_B;                         //!< [m/s] Inertial time derivative of r_FcB_B in B frame components
    Eigen::MRPd sigma_BN;                               //!< MRP attitude of frame B relative to the inertial frame

    // DCMs
    Eigen::Matrix3d dcm_BN;                             //!< DCM from inertial frame to frame B
    Eigen::Matrix3d dcm_BM;                             //!< DCM from frame M to frame B
    Eigen::Matrix3d dcm_FM;                             //!< DCM from frame M to frame F
    Eigen::Matrix3d dcm_BF;                             //!< DCM from frame F to frame B

    // Other matrix quantities
    Eigen::Matrix3d rTilde_FcB_B;                       //!< [m] Tilde cross product matrix of r_FcB_B
    Eigen::Matrix3d omegaTilde_BN_B;                    //!< [rad/s] Tilde cross product matrix of omega_BN_B
    Eigen::Matrix3d omegaTilde_FB_B;                    //!< [rad/s] Tilde cross product matrix of omega_FB_B

    // Effector properties relative to the inertial frame
    Eigen::Vector3d r_FcN_N;                            //!< [m] Position of frame F center of mass relative to the inertial frame in inertial frame components
    Eigen::Vector3d v_FcN_N;                            //!< [m/s] Inertial velocity of frame F center of mass relative to the inertial frame in inertial frame components
    Eigen::Vector3d sigma_FN;                           //!< MRP attitude of frame F relative to the inertial frame
    Eigen::Vector3d omega_FN_F;                         //!< [rad/s] Angular velocity of frame F relative to the inertial frame in F frame components

    // Hub states
    StateData *hubSigma;                                //!< Hub attitude relative to the inertial frame represented by MRP
    StateData *hubOmega;                                //!< [rad/s] Hub angular velocity in B frame components relative to the inertial frame
    Eigen::MatrixXd* inertialPositionProperty;          //!< [m] r_N Inertial position relative to system spice zeroBase/refBase
    Eigen::MatrixXd* inertialVelocityProperty;          //!< [m] v_N Inertial velocity relative to system spice zeroBase/refBase
};

#endif /* PRESCRIBED_MOTION_STATE_EFFECTOR_H */
