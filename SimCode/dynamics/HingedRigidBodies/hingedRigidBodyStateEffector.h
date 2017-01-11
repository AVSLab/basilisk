/*
 ISC License

 Copyright (c) 2016-2017, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#include "../_GeneralModuleFiles/stateEffector.h"
#include "../_GeneralModuleFiles/stateData.h"
#include "_GeneralModuleFiles/sys_model.h"
#include <Eigen/Dense>
#include "../SimCode/utilities/avsEigenMRP.h"

/*! @brief Abstract class that is used to implement an effector impacting a dynamic body 
           that does not itself maintain a state or represent a changing component of
           the body (for example: gravity, thrusters, solar radiation pressure, etc.)
 */

class HingedRigidBodyStateEffector : public StateEffector, public SysModel {
public:
    HingedRigidBodyStateEffector();
    ~HingedRigidBodyStateEffector();
    void registerStates(DynParamManager& statesIn);
    void linkInStates(DynParamManager& states);
    void updateContributions(double integTime, Eigen::Matrix3d & matrixAcontr, Eigen::Matrix3d & matrixBcontr, Eigen::Matrix3d & matrixCcontr, Eigen::Matrix3d & matrixDcontr, Eigen::Vector3d & vecTranscontr, Eigen::Vector3d & vecRotcontr);
    void computeDerivatives(double integTime);
    void updateEffectorMassProps(double integTime);
    void updateEnergyMomContributions(double integTime, Eigen::Vector3d & rotAngMomPntCContr_B, double & rotEnergyContr);

public:
    double mass;                    //!< [kg] mass of hinged rigid body
    double d;                       //!< [m] distance from hinge point to hinged rigid body center of mass
    double k;                       //!< [N-m/rad] torsional spring constant of hinge
    double c;                       //!< [N-m-s/rad] rotational damping coefficient of hinge
    Eigen::MatrixXd *g_N;           //!< [m/s^2] Gravitational acceleration in N frame components
    Eigen::Matrix3d IPntS_S;        //!< [kg-m^2] Inertia of hinged rigid body about point S in S frame components
    Eigen::Matrix3d dcm_HB;         //!< [-] DCM from body frame to hinge frame
    std::string nameOfThetaState;   //!< [-] Identifier for the theta state data container
    std::string nameOfThetaDotState; //!< [-] Identifier for the thetaDot state data container
    Eigen::Vector3d r_HB_B;          //!< [m] vector pointing from body frame origin to Hinge location

private:
    Eigen::Matrix3d rTildeHB_B;     //!< [-] Tilde matrix of rHB_B
    Eigen::Matrix3d dcm_SH;         //!< [-] DCM from hinge to hinged rigid body frame, S
    Eigen::Matrix3d dcm_SB;         //!< [-] DCM from body to S frame
    Eigen::Vector3d omegaBN_S;      //!< [rad/s] omega_BN in S frame components
    Eigen::Vector3d sHat1_B;        //!< [-] unit direction vector for the first axis of the S frame
    Eigen::Vector3d sHat2_B;        //!< [-] unit direction vector for the second axis of the S frame
    Eigen::Vector3d sHat3_B;        //!< [-] unit direction vector for the third axis of the S frame
    Eigen::Vector3d rSB_B;          //!< [-] Vector pointing from body origin to CoM of hinged rigid body in B frame comp
    Eigen::Matrix3d rTildeSB_B;     //!< [-] Tilde matrix of rSB_B
    Eigen::Vector3d rPrimeSB_B;     //!< [m/s] Body time derivative of rSB_B
    Eigen::Matrix3d rPrimeTildeSB_B;//!< [-] Tilde matrix of rPrime_SB_B
    Eigen::Matrix3d ISPrimePntS_B;  //!< [kg-m^2/s] time body derivative IPntS in body frame components
    Eigen::Vector3d omegaBNLoc_B;   //!< [rad/s] local copy of omegaBN
    Eigen::Matrix3d omegaTildeBNLoc_B; //!< [-] tilde matrix of omegaBN
    double theta;                   //!< [rad] hinged rigid body angle
    double thetaDot;                //!< [rad/s] hinged rigid body angle rate
    double a_theta;                 //!< [-] term needed for back substitution
    StateData *hubSigma;            //!< [-] state manager access to the hubs MRP state
    StateData *hubOmega;            //!< [-] state manager access to the hubs omegaBN_B state
    StateData *hubVelocity;         //!< [-] state manager access to the hubs rDotBN_N state
    StateData *thetaState;          //!< [-] state manager of theta for hinged rigid body
    StateData *thetaDotState;       //!< [-] state manager of thetaDot for hinged rigid body
};

#endif /* STATE_EFFECTOR_H */
