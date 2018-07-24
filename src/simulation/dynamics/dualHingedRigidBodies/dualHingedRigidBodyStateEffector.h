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

#include "../_GeneralModuleFiles/stateEffector.h"
#include "../_GeneralModuleFiles/stateData.h"
#include "_GeneralModuleFiles/sys_model.h"
#include <Eigen/Dense>
#include "../simulation/utilities/avsEigenMRP.h"
#include "../simulation/utilities/avsEigenSupport.h"

/*! @brief Class to represent a solar array of two panels. The first panel is hinged on a single axis to the spacecraft body.
            The second panel is hinged to the first panel by a parallel axis on the opposite end of the first panel from the spacecraft body.)
 The module
 [PDF Description](Basilisk-DUALHINGEDRIGIDBODYSTATEEFFECTOR-20180102.pdf)
 contains further information on this module's function,
 how to run it, as well as testing.
 */

class DualHingedRigidBodyStateEffector : public StateEffector, public SysModel {
public:
    DualHingedRigidBodyStateEffector();
    ~DualHingedRigidBodyStateEffector();
    void registerStates(DynParamManager& statesIn);
    void linkInStates(DynParamManager& states);
    void updateEffectorMassProps(double integTime);
    void updateContributions(double integTime, BackSubMatrices & backSubContr, Eigen::Vector3d sigma_BN, Eigen::Vector3d omega_BN_B, Eigen::Vector3d g_N);  //!< -- Back-sub contributions
    void updateEnergyMomContributions(double integTime, Eigen::Vector3d & rotAngMomPntCContr_B,
                                              double & rotEnergyContr, Eigen::Vector3d omega_BN_B);  //!< -- Energy and momentum calculations
    void computeDerivatives(double integTime, Eigen::Vector3d rDDot_BN_N, Eigen::Vector3d omegaDot_BN_B, Eigen::Vector3d sigma_BN);  //!< -- Method for each stateEffector to calculate derivatives

public:
    double mass1;                     //!< [kg] mass of 1st hinged rigid body
    double mass2;                     //!< [kg] mass of 2nd hinged rigid body
    double d1;                        //!< [m] distance from hinge point to hinged rigid body center of mass
    double d2;                        //!< [m] distance from hinge point to hinged rigid body center of mass
    double l1;                        //!< [m] distance from hinge point to hinged rigid body center of mass
    double l2;                        //!< [m] distance from hinge point to hinged rigid body center of mass
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
    Eigen::Vector3d rH1B_B;           //!< [m] vector pointing from body frame origin to Hinge location
    Eigen::Matrix3d dcmH1B;           //!< [-] DCM from body frame to hinge frame
    double thetaH2S1;                 //!< [-] theta offset of H2 frame with respect to S1 frame
    std::string nameOfTheta1State;    //!< [-] Identifier for the theta state data container
    std::string nameOfTheta1DotState; //!< [-] Identifier for the thetaDot state data container
    std::string nameOfTheta2State;    //!< [-] Identifier for the theta state data container
    std::string nameOfTheta2DotState; //!< [-] Identifier for the thetaDot state data container
    Eigen::MatrixXd *g_N;             //!< [m/s^2] Gravitational acceleration in N frame components

private:
    Eigen::Matrix3d rTildeH1B_B;      //!< [-] Tilde matrix of rHB_B
    Eigen::Matrix3d dcmS1B;           //!< [-] DCM from body to S1 frame
    Eigen::Matrix3d dcmS2B;           //!< [-] DCM from body to S2 frame
    Eigen::Vector3d omegaBN_S1;       //!< [rad/s] omega_BN in S frame components
    Eigen::Vector3d omegaBN_S2;       //!< [rad/s] omega_BN in S frame components
    Eigen::Vector3d sHat11_B;         //!< [-] unit direction vector for the first axis of the S frame
    Eigen::Vector3d sHat12_B;         //!< [-] unit direction vector for the second axis of the S frame
    Eigen::Vector3d sHat13_B;         //!< [-] unit direction vector for the third axis of the S frame
    Eigen::Vector3d sHat21_B;         //!< [-] unit direction vector for the first axis of the S frame
    Eigen::Vector3d sHat22_B;         //!< [-] unit direction vector for the second axis of the S frame
    Eigen::Vector3d sHat23_B;         //!< [-] unit direction vector for the third axis of the S frame
    Eigen::Vector3d rS1B_B;           //!< [-] Vector pointing from body origin to CoM of hinged rigid body in B frame comp
    Eigen::Vector3d rS2B_B;           //!< [-] Vector pointing from body origin to CoM of hinged rigid body in B frame comp
    Eigen::Matrix3d rTildeS1B_B;      //!< [-] Tilde matrix of rSB_B
    Eigen::Matrix3d rTildeS2B_B;      //!< [-] Tilde matrix of rSB_B
    Eigen::Vector3d rPrimeS1B_B;      //!< [m/s] Body time derivative of rSB_B
    Eigen::Vector3d rPrimeS2B_B;      //!< [m/s] Body time derivative of rSB_B
    Eigen::Matrix3d rPrimeTildeS1B_B; //!< [-] Tilde matrix of rPrime_SB_B
    Eigen::Matrix3d rPrimeTildeS2B_B; //!< [-] Tilde matrix of rPrime_SB_B
    Eigen::Matrix3d IS1PrimePntS1_B;  //!< [kg-m^2/s] time body derivative IPntS in body frame components
    Eigen::Matrix3d IS2PrimePntS2_B;  //!< [kg-m^2/s] time body derivative IPntS in body frame components
    Eigen::Vector3d omegaBNLoc_B;     //!< [rad/s] local copy of omegaBN
    Eigen::Matrix3d omegaTildeBNLoc_B;//!< [-] tilde matrix of omegaBN
    double theta1;                    //!< [rad] hinged rigid body angle
    double theta1Dot;                 //!< [rad/s] hinged rigid body angle rate
    double theta2;                    //!< [rad] hinged rigid body angle
    double theta2Dot;                 //!< [rad/s] hinged rigid body angle rate
    Eigen::Matrix2d matrixADHRB;      //!< [-] term needed for back substitution
    Eigen::Matrix2d matrixEDHRB;      //!< [-] term needed for back substitution
    Eigen::MatrixXd matrixFDHRB;
    Eigen::MatrixXd matrixGDHRB;
    Eigen::Vector2d vectorVDHRB;
    StateData *hubSigma;              //!< [-] state manager access to the hubs MRP state
    StateData *hubOmega;              //!< [-] state manager access to the hubs omegaBN_B state
    StateData *hubVelocity;           //!< [-] state manager access to the hubs rDotBN_N state
    StateData *theta1State;           //!< [-] state manager of theta for hinged rigid body
    StateData *theta1DotState;        //!< [-] state manager of thetaDot for hinged rigid body
    StateData *theta2State;           //!< [-] state manager of theta for hinged rigid body
    StateData *theta2DotState;        //!< [-] state manager of thetaDot for hinged rigid body

};

#endif /* DUAL_STATE_EFFECTOR_H */
