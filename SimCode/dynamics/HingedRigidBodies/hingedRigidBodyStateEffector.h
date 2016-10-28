/*
 Copyright (c) 2016, Autonomous Vehicle Systems Lab, Univeristy of Colorado at Boulder
 
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
#include "../_GeneralModuleFiles/sys_model.h"
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
    void updateEffectorMassPropRates(double integTime);

public:
    double mass;               //!< kg, mass of hinged rigid body
    double d;                  //!< m, distance from hinge point to hinged rigid body center of mass
    double k;                  //!< N-m/rad, torsional spring constant of hinge
    double c;                  //!< N-m-s/rad, rotational damping coefficient of hinge
    Eigen::Matrix3d IPntS_S;         //!< kg-m^2, Inertia of hinged rigid body about point S in S frame components
    Eigen::Vector3d rHB_B;          //!< m, vector pointing from body frame origin to Hinge location
    Eigen::Matrix3d HB;              //!< DCM from body frame to hinge frame
private:
    Eigen::Matrix3d rTildeHB_B;  //!< Tilde matrix of rHB_B
    Eigen::Matrix3d SH;           //!< DCM from hinge to hinged rigid body frame, S
    Eigen::Matrix3d SB;           //!< DCM from body to S frame
    Eigen::Vector3d omegaBN_S;      //!< omega_BN in S frame components
    Eigen::Vector3d sHat1_B;         //!< unit direction vector for the first axis of the S frame
    Eigen::Vector3d sHat2_B;         //!< unit direction vector for the second axis of the S frame
    Eigen::Vector3d sHat3_B;         //!< unit direction vector for the third axis of the S frame
    Eigen::Vector3d rSB_B;          //!< Vector pointing from body origin to CoM of hinged rigid body in B frame comp
    Eigen::Matrix3d rTildeSB_B;  //!< Tilde matrix of r_SB_B
    Eigen::Vector3d rPrimeSB_B;     //!< Body time derivative of r_SB_B
    Eigen::Matrix3d rPrimeTildeSB_B;//!< Tilde matrix of rPrime_SB_B
    Eigen::Matrix3d ISPrimePntS_B;
    Eigen::Vector3d omegaBNLoc_B;
    Eigen::Matrix3d omegaTildeBNLoc_B;
    double theta;              //!< rad, hinged rigid body angle
    double thetaDot;           //!< rad/s, hinged rigid body angle rate
    double a_theta;
    StateData *hubSigma;
    StateData *hubOmega;
    StateData *hubVelocity;
    StateData *thetaState;
    StateData *thetaDotState;
    
};

#endif /* STATE_EFFECTOR_H */
