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

#ifndef SPINNING_BODY_STATE_EFFECTOR_H
#define SPINNING_BODY_STATE_EFFECTOR_H

#include <Eigen/Dense>
#include "simulation/dynamics/_GeneralModuleFiles/stateEffector.h"
#include "simulation/dynamics/_GeneralModuleFiles/stateData.h"
#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/utilities/avsEigenMRP.h"

#include "architecture/msgPayloadDefC/ArrayMotorTorqueMsgPayload.h"
#include "architecture/msgPayloadDefC/SCStatesMsgPayload.h"
#include "architecture/msgPayloadDefC/SpinningBodyMsgPayload.h"
#include "architecture/messaging/messaging.h"

#include "architecture/utilities/bskLogging.h"

/*! @brief hinged rigid body state effector class */
class SpinningBodyStateEffector: public StateEffector, public SysModel {
public:
    double mass;                    //!< [kg] mass of hinged rigid body
    double thetaInit;               //!< [rad] Initial hinged rigid body angle
    double thetaDotInit;               //!< [rad/s] Initial hinged rigid body angle rate
    std::string nameOfThetaState;   //!< -- Identifier for the theta state data container
    std::string nameOfThetaDotState;   //!< -- Identifier for the thetaDot state data container
    Eigen::Matrix3d IPntSc_S;       //!< [kg-m^2] Inertia of hinged rigid body about point Sc in S frame components
    Eigen::Vector3d r_SB_B;         //!< [m] vector pointing from body frame B origin to spinning frame S origin in B frame components.
    Eigen::Vector3d r_ScS_S;        //!< [m] vector pointing from spinning frame origin to point Sc (center of mass of the spinner) in S frame components
    Eigen::Vector3d sHat_S;               //!< spinning axis in B or S frame components.
    Eigen::Matrix3d dcm_S0B;        //!< -- DCM from the body frame to the S frame for theta=0
    Message<SpinningBodyMsgPayload> spinningBodyOutMsg; //!< -- state output message name
    Message<SCStatesMsgPayload> spinningBodyConfigLogOutMsg; //!< panel state config log message name
    SpinningBodyMsgPayload SBoutputStates;  //!< instance of messaging system message struct
    BSKLogger bskLogger;                       //!< -- BSK Logging

private:
    static uint64_t effectorID;         //!< [] ID number of this panel
    double theta;                       //!< [rad] hinged rigid body angle
    double thetaDot;                       //!< [rad/s] hinged rigid body angle rate
    double cTheta;                      //!< -- term needed for back substitution
    double dTheta;                      //!< -- term needed for back substitution
    double u;                           //!< [N-m] optional motor torque
    Eigen::Matrix3d dcm_BS;         //!< -- DCM from spinner frame to body frame
    Eigen::Vector3d prv_S0S;             
    Eigen::Vector3d sHat_B;
    Eigen::Vector3d r_ScS_B;            //!< [m] vector pointing from spinning frame origin to point Sc (center of mass of the spinner) in B frame components
    Eigen::Vector3d r_ScB_B;            //!< [m] vector pointing from spinning frame S origin to the center of mass of the spinning body, in S frame components.
    Eigen::Matrix3d rTilde_ScB_B;       //!< [m] tilde matrix of r_ScB_B
    Eigen::Matrix3d rTilde_ScS_B;       //!< [m] tilde matrix of r_ScS_B
    Eigen::Matrix3d IPntSc_B;           //!< [kg-m^2] Inertia of hinged rigid body about point Sc in B frame components
    Eigen::Vector3d omega_SB_B;         //!< [rad/s] angular velocity of the S frame wrt the B frame in B frame components.
    Eigen::Vector3d omega_SN_B;         //!< [rad/s] angular velocity of the S frame wrt the inertial frame in B frame components.
    Eigen::Vector3d rPrime_ScS_B;       //!< [m/s] body frame time derivative of r_ScS_B
    Eigen::Vector3d rPrime_ScB_B;       //!< [m/s] body frame time derivative of r_ScB_B
    Eigen::Matrix3d rPrimeTilde_ScB_B;  //!< [m/s] tilde matrix of rPrime_ScB_B
    Eigen::Vector3d rDot_ScS_B;
    Eigen::Vector3d rDot_SB_B;
    Eigen::Vector3d rDot_ScB_B;
    Eigen::Matrix3d omegaTilde_SB_B;    //!< [rad/s] tilde matrix of omega_SB_B
    Eigen::Matrix3d omegaTilde_SN_B;    //!< [rad/s] tilde matrix of omega_SN_B
    Eigen::Vector3d r_ScN_N;             //!< [m] position vector of hinge CM S relative to inertial frame
    Eigen::Vector3d v_ScN_N;             //!< [m/s] inertial velocity vector of S relative to inertial frame
    Eigen::Vector3d sigma_SN;           //!< -- MRP attitude of frame S relative to inertial frame
    Eigen::Vector3d omega_SN_S;         //!< [rad/s] inertial panel frame angular velocity vector
    Eigen::Vector3d aTheta;             //!< -- term needed for back substitution
    Eigen::Vector3d bTheta;             //!< -- term needed for back substitution
    Eigen::MRPd sigma_BN;
    Eigen::Matrix3d dcm_BN;
    Eigen::Vector3d omega_BN_B;
    Eigen::Matrix3d omegaTilde_BN_B;    //!< [rad/s] tilde matrix of omega_BN_B

    StateData *hubSigma;                //!< Hub/Inertial attitude represented by MRP
    StateData *hubOmega;              //!< Hub/Inertial angular velocity vector in B frame components
    StateData *hubPosition;                //!< Hub/Inertial position vector in inertial frame components
    StateData *hubVelocity;                  //!< Hub/Inertial velocity vector in inertial frame components
    StateData *thetaState;              //!< -- state manager of theta for hinged rigid body
    StateData *thetaDotState;              //!< -- state manager of thetaDot for hinged rigid body
    Eigen::MatrixXd *c_B;               //!< [m] Vector from point B to CoM of s/c in B frame components

    Eigen::MatrixXd *cPrime_B;          //!< [m/s] Body time derivative of vector c_B in B frame components

public:
    SpinningBodyStateEffector();  //!< -- Contructor
    ~SpinningBodyStateEffector();  //!< -- Destructor
    void writeOutputStateMessages(uint64_t CurrentClock);
	void UpdateState(uint64_t CurrentSimNanos);
    void registerStates(DynParamManager& statesIn);  //!< -- Method for registering the HRB states
    void linkInStates(DynParamManager& states);  //!< -- Method for getting access to other states
    void updateContributions(double integTime, BackSubMatrices & backSubContr, Eigen::Vector3d sigma_BN, Eigen::Vector3d omega_BN_B, Eigen::Vector3d g_N);  //!< -- Method for back-sub contributions
    void computeDerivatives(double integTime, Eigen::Vector3d rDDot_BN_N, Eigen::Vector3d omegaDot_BN_B, Eigen::Vector3d sigma_BN);  //!< -- Method for HRB to compute its derivatives
    void updateEffectorMassProps(double integTime);  //!< -- Method for giving the s/c the HRB mass props and prop rates
    void updateEnergyMomContributions(double integTime, Eigen::Vector3d & rotAngMomPntCContr_B, double & rotEnergyContr, Eigen::Vector3d omega_BN_B); //!< -- Computing energy and momentum for HRBs
    void calcForceTorqueOnBody(double integTime, Eigen::Vector3d omega_BN_B);  //!< -- Force and torque on s/c due to SBs
    void prependSpacecraftNameToStates(); //!< class method
    void computeSpinningBodyInertialStates();

private:

};


#endif /* STATE_EFFECTOR_H */
