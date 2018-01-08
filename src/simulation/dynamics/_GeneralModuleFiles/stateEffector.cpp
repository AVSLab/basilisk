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

#include "stateEffector.h"

/*! This is the constructor, just setting the variables to zero */
StateEffector::StateEffector()
{
    // - Set effector mass props to zero
    effProps.IEffPntB_B.fill(0.0);
    effProps.IEffPrimePntB_B.fill(0.0);
    effProps.rEff_CB_B.fill(0.0);
    effProps.rEffPrime_CB_B.fill(0.0);
    effProps.mEff = 0.0;
    effProps.mEffDot = 0.0;
    return;
}

/*! This is the destructor, nothing to report here */
StateEffector::~StateEffector()
{
    return;
}

/*! This method is for the state effector to provide its contributions of mass and mass rates to the dynamicObject. This
 allows for the dynamicObject to have access to the total mass, and inerita, mass and inertia rates*/
void StateEffector::updateEffectorMassProps(double integTime)
{
    return;
}

/*! This method is strictly for the back-substituion method for computing the dynamics of the spacecraft. The back-sub
 method first computes rDDot_BN_N and omegaDot_BN_B for the spacecraft using these contributions from the state 
 effectors. Then computeDerivatives is called to compute the stateEffectors derivatives using rDDot_BN_N omegaDot_BN_B*/
void StateEffector::updateContributions(double integTime, Eigen::Matrix3d & matrixAcontr, Eigen::Matrix3d
                                        & matrixBcontr, Eigen::Matrix3d & matrixCcontr, Eigen::Matrix3d & matrixDcontr,
                                        Eigen::Vector3d & vecTranscontr, Eigen::Vector3d & vecRotcontr)
{
    return;
}

/*! This method allows for an individual stateEffector to add its energy and momentum calculations to the dynamicObject.
 The analytical devlopement of these contributions can be seen in 
 Basilisk/simulation/dynamics/_Documentation/Basilisk-EnergyAndMomentum-20161219.pdf*/
void StateEffector::updateEnergyMomContributions(double integTime, Eigen::Vector3d & rotAngMomPntCContr_B,
                                                 double & rotEnergyContr)
{
    return;
}

/*! This method allows for an individual stateEffector to modify their states after integration*/
void StateEffector::modifyStates(double integTime)
{
    return;
}
