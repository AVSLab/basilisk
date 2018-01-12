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

#ifndef HUB_EFFECTOR_H
#define HUB_EFFECTOR_H

#include <Eigen/Dense>
#include "../_GeneralModuleFiles/stateEffector.h"
#include "../_GeneralModuleFiles/stateData.h"
#include "../simulation/utilities/avsEigenMRP.h"

/*! @brief This class is an instantiation of the stateEffector abstract class and is for the hub of the s/c. The hub
 has 4 states: r_BN_N, rDot_BN_N, sigma_BN and omega_BN_B. The hub utilizes the back-substitution method for calculating
 its derivatives using contributions from stateEffectors and dynEffectors. */
class HubEffector : public StateEffector {
public:
    double mHub;                         //!< [kg] mass of the hub
    bool useTranslation;                 //!< -- Whether the s/c has translational states
    bool useRotation;                    //!< -- Whether the s/c has rotational states
    uint64_t MRPSwitchCount;             //!< -- Count on times we've shadowed
    std::string nameOfHubPosition;       //!< -- Identifier for hub position states
    std::string nameOfHubVelocity;       //!< -- Identifier for hub velocity states
    std::string nameOfHubSigma;          //!< -- Identifier for hub sigmaBN states
    std::string nameOfHubOmega;          //!< -- Identifier for hub omegaBN_B states
    Eigen::Vector3d r_BcB_B;             //!< [m] vector from point B to CoM of hub in B frame components
    Eigen::Matrix3d IHubPntBc_B;         //!< [kg m^2] Inertia of hub about point Bc in B frame components
    Eigen::MatrixXd *m_SC;               //!< [kg] spacecrafts total mass
    Eigen::MatrixXd *mDot_SC;            //!< [kg] Time derivative of spacecrafts total mass
    Eigen::MatrixXd *ISCPntB_B;          //!< [kg m^2] Inertia of s/c about point B in B frame components
    Eigen::MatrixXd *c_B;                //!< [m] Vector from point B to CoM of s/c in B frame components
    Eigen::MatrixXd *cPrime_B;           //!< [m] Body time derivative of c_B
    Eigen::MatrixXd *ISCPntBPrime_B;     //!< [m] Body time derivative of ISCPntB_B
    Eigen::MatrixXd *g_N;                //!< [m/s^2] Gravitational acceleration in N frame components
    Eigen::Matrix3d matrixA;             //!< -- Back-Substitution matrix A
    Eigen::Matrix3d matrixB;             //!< -- Back-Substitution matrix B
    Eigen::Matrix3d matrixC;             //!< -- Back-Substitution matrix C
    Eigen::Matrix3d matrixD;             //!< -- Back-Substitution matrix D
    Eigen::Vector3d vecTrans;            //!< -- Back-Substitution translation vector
    Eigen::Vector3d vecRot;              //!< -- Back-Substitution rotation vector
    Eigen::Vector3d sumForceExternal_N;  //!< [N] Sum of forces given in the inertial frame
    Eigen::Vector3d sumForceExternal_B;  //!< [N] Sum of forces given in the body frame
    Eigen::Vector3d sumTorquePntB_B;     //!< [N-m] Total torque about point B in B frame components
    Eigen::Vector3d r_CN_NInit;          //!< [m] Initial position of the spacecraft wrt to base
    Eigen::Vector3d v_CN_NInit;          //!< [m/s Initial velocity of the spacecraft wrt base
    Eigen::Vector3d sigma_BNInit;        //!< -- Initial attitude of the spacecraft wrt base
    Eigen::Vector3d omega_BN_BInit;      //!< [r/s] Initial attitude rate of the spacecraf wrt base

public:
    HubEffector();                       //!< -- Contructor
    ~HubEffector();                      //!< -- Destructor
    void linkInStates(DynParamManager& statesIn);  //!< -- Method to give the hub access to states
    void registerStates(DynParamManager& states);  //!< -- Method for the hub to register some states
    void updateEffectorMassProps(double integTime);  //!< -- Method for the hub to update its mass props for the s/c
    void computeDerivatives(double integTime);  //!< -- Method for the hub to compute it's derivatives
    void updateEnergyMomContributions(double integTime, Eigen::Vector3d &rotAngMomPntCContr_B,
                                      double & rotEnergyContr); //!< -- Add contributions to energy and momentum
    void modifyStates(double integTime); //!< -- Method to switch MRPs

private:
    StateData *posState;                 //!< [-] State data container for hub position
    StateData *velocityState;            //!< [-] State data container for hub velocity
    StateData *sigmaState;               //!< [-] State data container for hub sigma_BN
    StateData *omegaState;               //!< [-] State data container for hub omegaBN_B
};

#endif /* HUB_EFFECTOR_H */
