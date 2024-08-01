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
#include "simulation/dynamics/_GeneralModuleFiles/stateEffector.h"
#include "simulation/dynamics/_GeneralModuleFiles/stateData.h"
#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/utilities/avsEigenMRP.h"

#include "architecture/msgPayloadDefC/ArrayMotorTorqueMsgPayload.h"
#include "architecture/msgPayloadDefC/SCStatesMsgPayload.h"
#include "architecture/msgPayloadDefC/HingedRigidBodyMsgPayload.h"
#include "architecture/messaging/messaging.h"

#include "architecture/utilities/bskLogging.h"

/*! @brief hinged rigid body state effector class */
class HingedRigidBodyStateEffector : public StateEffector, public SysModel {
public:
    double mass;                     //!< [kg] mass of hinged rigid body
    double d;                        //!< [m] distance from hinge point H to hinged rigid body center of mass S
    double k;                        //!< [N-m/rad] torsional spring constant of hinge
    double c;                        //!< [N-m-s/rad] rotational damping coefficient of hinge
    double thetaInit;                //!< [rad] Initial hinged rigid body angle
    double thetaDotInit;             //!< [rad/s] Initial hinged rigid body angle rate
    double thetaRef;                  //!< [rad] hinged rigid body reference angle
    double thetaDotRef;               //!< [rad/s] hinged rigid body reference angle rate
    std::string nameOfThetaState;    //!< -- Identifier for the theta state data container
    std::string nameOfThetaDotState; //!< -- Identifier for the thetaDot state data container
    Eigen::Matrix3d IPntS_S;         //!< [kg-m^2] Inertia of hinged rigid body about point S in S frame components
    Eigen::Vector3d r_HB_B;          //!< [m] vector pointing from body frame origin to Hinge location
    Eigen::Matrix3d dcm_HB;          //!< -- DCM from body frame to hinge frame
    Message<HingedRigidBodyMsgPayload> hingedRigidBodyOutMsg; //!< -- state output message name
    ReadFunctor<ArrayMotorTorqueMsgPayload> motorTorqueInMsg; //!< -- (optional) motor torque input message name
    ReadFunctor<HingedRigidBodyMsgPayload> hingedRigidBodyRefMsg; //!< -- (optional) rigid body reference input message name
    Message<SCStatesMsgPayload> hingedRigidBodyConfigLogOutMsg; //!< panel state config log message name
    HingedRigidBodyMsgPayload HRBoutputStates;  //!< instance of messaging system message struct
    BSKLogger bskLogger;                      //!< -- BSK Logging

private:
    static uint64_t effectorID;        //!< [] ID number of this panel
    double theta;                    //!< [rad] hinged rigid body angle
    double thetaDot;                 //!< [rad/s] hinged rigid body angle rate
    double cTheta;                  //!< -- term needed for back substitution
    double u;                        //!< [N-m] optional motor torque
    Eigen::Vector3d r_HP_P;          //!< [m] vector pointing from primary body frame P origin to Hinge location.  If a single spacecraft body is modeled than P is the same as B
    Eigen::Matrix3d dcm_HP;          //!< -- DCM from primary body frame to hinge frame
    Eigen::Vector3d aTheta;         //!< -- term needed for back substitution
    Eigen::Vector3d bTheta;         //!< -- term needed for back substitution
    Eigen::Matrix3d rTilde_HP_P;     //!< -- Tilde matrix of rHB_B
    Eigen::Matrix3d dcm_SH;          //!< -- DCM from hinge to hinged rigid body frame, S
    Eigen::Matrix3d dcm_SP;          //!< -- DCM from body to S frame
    Eigen::Vector3d omega_PN_S;       //!< [rad/s] omega_BN in S frame components
    Eigen::Vector3d sHat1_P;         //!< -- unit direction vector for the first axis of the S frame
    Eigen::Vector3d sHat2_P;         //!< -- unit direction vector for the second axis of the S frame
    Eigen::Vector3d sHat3_P;         //!< -- unit direction vector for the third axis of the S frame
    Eigen::Vector3d r_SP_P;          //!< -- Vector pointing from B to CoM of hinged rigid body in B frame components
    Eigen::Matrix3d rTilde_SP_P;     //!< -- Tilde matrix of rSB_B
    Eigen::Vector3d rPrime_SP_P;     //!< [m/s] Body time derivative of rSB_B
    Eigen::Matrix3d rPrimeTilde_SP_P;  //!< -- Tilde matrix of rPrime_SB_B
    Eigen::Matrix3d ISPrimePntS_P;  //!< [kg-m^2/s] time body derivative IPntS in body frame components
    Eigen::Vector3d omegaLoc_PN_P;  //!< [rad/s] local copy of omegaBN
    Eigen::Matrix3d omegaTildeLoc_PN_P; //!< -- tilde matrix of omegaBN
    Eigen::Vector3d r_SN_N;          //!< [m] position vector of hinge CM S relative to inertial frame
    Eigen::Vector3d v_SN_N;          //!< [m/s] inertial velocity vector of S relative to inertial frame
    Eigen::Vector3d sigma_SN;        //!< -- MRP attitude of panel frame S relative to inertial frame
    Eigen::Vector3d omega_SN_S;      //!< [rad/s] inertial panel frame angular velocity vector
    Eigen::MRPd sigma_BN{0.0, 0.0, 0.0};         //!< Hub/Inertial attitude represented by MRP
    Eigen::Vector3d omega_BN_B{0.0, 0.0, 0.0};  //!< Hub/Inertial angular velocity vector in B frame components
    StateData *thetaState;           //!< -- state manager of theta for hinged rigid body
    Eigen::MatrixXd *inertialPositionProperty;  //!< [m] r_N inertial position relative to system spice zeroBase/refBase
    Eigen::MatrixXd *inertialVelocityProperty;  //!< [m] v_N inertial velocity relative to system spice zeroBase/refBase

    StateData *thetaDotState;        //!< -- state manager of thetaDot for hinged rigid body
    Eigen::MatrixXd *c_B;            //!< [m] Vector from point B to CoM of s/c in B frame components

    Eigen::MatrixXd *cPrime_B;       //!< [m/s] Body time derivative of vector c_B in B frame components

public:
    HingedRigidBodyStateEffector();  //!< -- Contructor
    ~HingedRigidBodyStateEffector();  //!< -- Destructor
    void writeOutputStateMessages(uint64_t CurrentClock);
	void UpdateState(uint64_t CurrentSimNanos);
    void registerStates(DynParamManager& statesIn);  //!< -- Method for registering the HRB states
    void linkInStates(DynParamManager& states);  //!< -- Method for getting access to other states
    void updateContributions(double integTime, BackSubMatrices & backSubContr, Eigen::Vector3d sigma_BN, Eigen::Vector3d omega_BN_B, Eigen::Vector3d g_N);  //!< -- Method for back-sub contributions
    void computeDerivatives(double integTime, Eigen::Vector3d rDDot_BN_N, Eigen::Vector3d omegaDot_BN_B, Eigen::Vector3d sigma_BN);  //!< -- Method for HRB to compute its derivatives
    void updateEffectorMassProps(double integTime);  //!< -- Method for giving the s/c the HRB mass props and prop rates
    void updateEnergyMomContributions(double integTime, Eigen::Vector3d & rotAngMomPntCContr_B, double & rotEnergyContr, Eigen::Vector3d omega_BN_B); //!< -- Computing energy and momentum for HRBs
    void calcForceTorqueOnBody(double integTime, Eigen::Vector3d omega_BN_B);  //!< -- Force and torque on s/c due to HRBs
    void prependSpacecraftNameToStates(); //!< class method

private:
    void computePanelInertialStates();
};


#endif /* STATE_EFFECTOR_H */
