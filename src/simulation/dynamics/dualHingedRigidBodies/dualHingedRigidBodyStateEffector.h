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
#include "simulation/dynamics/_GeneralModuleFiles/stateData.h"
#include "architecture/_GeneralModuleFiles/sys_model.h"
#include <Eigen/Dense>
#include "architecture/utilities/avsEigenMRP.h"
#include "architecture/utilities/avsEigenSupport.h"
#include "architecture/utilities/bskLogging.h"

#include "architecture/msgPayloadDefC/ArrayMotorTorqueMsgPayload.h"
#include "architecture/msgPayloadDefC/SCStatesMsgPayload.h"
#include "architecture/msgPayloadDefC/HingedRigidBodyMsgPayload.h"
#include "architecture/messaging/messaging.h"




/*! @brief dual hinged rigid body state effector */
class DualHingedRigidBodyStateEffector : public StateEffector, public SysModel {
public:
    DualHingedRigidBodyStateEffector();
    ~DualHingedRigidBodyStateEffector();
    void registerStates(DynParamManager& statesIn);     //!< class method
    void linkInStates(DynParamManager& states);         //!< class method
    void updateEffectorMassProps(double integTime);     //!< class method
    void updateContributions(double integTime, BackSubMatrices & backSubContr, Eigen::Vector3d sigma_BN, Eigen::Vector3d omega_BN_B, Eigen::Vector3d g_N);  //!< -- Back-sub contributions
    void updateEnergyMomContributions(double integTime, Eigen::Vector3d & rotAngMomPntCContr_B,
                                              double & rotEnergyContr, Eigen::Vector3d omega_BN_B);  //!< -- Energy and momentum calculations
    void computeDerivatives(double integTime, Eigen::Vector3d rDDot_BN_N, Eigen::Vector3d omegaDot_BN_B, Eigen::Vector3d sigma_BN);  //!< -- Method for each stateEffector to calculate derivatives
    void Reset(uint64_t CurrentSimNanos);
    void UpdateState(uint64_t CurrentSimNanos);
    void writeOutputStateMessages(uint64_t CurrentClock);

private:
    void computePanelInertialStates();
    void prependSpacecraftNameToStates(); //!< class method

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
    std::string nameOfTheta1State;    //!< [-] Identifier for the theta state data container
    std::string nameOfTheta1DotState; //!< [-] Identifier for the thetaDot state data container
    std::string nameOfTheta2State;    //!< [-] Identifier for the theta state data container
    std::string nameOfTheta2DotState; //!< [-] Identifier for the thetaDot state data container
    BSKLogger bskLogger;                      //!< -- BSK Logging
    ReadFunctor<ArrayMotorTorqueMsgPayload> motorTorqueInMsg; //!< -- (optional) motor torque input message
    std::vector<Message<HingedRigidBodyMsgPayload>*> dualHingedRigidBodyOutMsgs; //!< -- state output message vector for all panels
    std::vector<Message<SCStatesMsgPayload>*> dualHingedRigidBodyConfigLogOutMsgs; //!< panel state config log message vector for all panels

private:
    static uint64_t effectorID;        //!< [] ID number of this panel
    Eigen::Vector3d r_H1P_P;          //!< [m] vector pointing from primary body frame P origin to Hinge 1 location.  If a single spacecraft body is modeled than P is the same as B
    Eigen::Matrix3d dcm_H1P;          //!< -- DCM from primary body frame to hinge 1 frame
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
    Eigen::Matrix2d matrixADHRB;      //!< [-] term needed for back substitution
    Eigen::Matrix2d matrixEDHRB;      //!< [-] term needed for back substitution
    Eigen::MatrixXd matrixFDHRB;
    Eigen::MatrixXd matrixGDHRB;
    Eigen::Vector2d vectorVDHRB;
    StateData *theta1State;           //!< [-] state manager of theta for hinged rigid body
    StateData *theta1DotState;        //!< [-] state manager of thetaDot for hinged rigid body
    StateData *theta2State;           //!< [-] state manager of theta for hinged rigid body
    StateData *theta2DotState;        //!< [-] state manager of thetaDot for hinged rigid body
    Eigen::Vector3d r_SN_N[2];        //!< [m] position vector of hinge CM S relative to inertial frame
    Eigen::Vector3d v_SN_N[2];        //!< [m/s] inertial velocity vector of S relative to inertial frame
    Eigen::Vector3d sigma_SN[2];      //!< -- MRP attitude of panel frame S relative to inertial frame
    Eigen::Vector3d omega_SN_S[2];    //!< [rad/s] inertial panel frame angular velocity vector
    Eigen::MRPd sigma_BN{0.0, 0.0, 0.0};        //!< Hub/Inertial attitude represented by MRP of body relative to inertial frame
    Eigen::Vector3d omega_BN_B{0.0, 0.0, 0.0};  //!< Hub/Inertial angular velocity vector in B frame components
    StateData *v_BN_NState;           //!< Hub/Inertial velocity vector in inertial frame components
    Eigen::MatrixXd *inertialPositionProperty;  //!< [m] r_N inertial position relative to system spice zeroBase/refBase
    Eigen::MatrixXd *inertialVelocityProperty;  //!< [m] v_N inertial velocity relative to system spice zeroBase/refBase
    Eigen::MatrixXd *g_N;             //!< [m/s^2] Gravitational acceleration in N frame components

};


#endif /* DUAL_STATE_EFFECTOR_H */
