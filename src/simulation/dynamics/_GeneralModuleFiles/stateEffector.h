/*
 ISC License

 Copyright (c) 2016-2018, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#ifndef STATE_EFFECTOR_H
#define STATE_EFFECTOR_H

#include <Eigen/Dense>
#include "dynParamManager.h"

/*! @brief Abstract class that is used to implement an effector attached to the dynamicObject that has a state that
 needs to be integrated. For example: reaction wheels, flexing solar panels, fuel slosh etc */
typedef struct {
    double mEff;                           //!< [kg] Mass of the effector
    double mEffDot;					   //!< [kg/s] Time derivate of mEff
    Eigen::Matrix3d IEffPntB_B;            //!< [kg m^2] Inertia of effector relative to point B in B frame components
    Eigen::Vector3d rEff_CB_B;             //!< [m] Center of mass of effector with respect to point B in B frame comp
    Eigen::Vector3d rEffPrime_CB_B;        //!< [m/s] Time derivative with respect to the body of rEff_CB_B
    Eigen::Matrix3d IEffPrimePntB_B;       //!< [kg m^2/s] Time derivative with respect to the body of IEffPntB_B
}EffectorMassProps;

class StateEffector {
public:
    EffectorMassProps effProps;            //!< -- stateEffectors instantiation of effector mass props
    Eigen::Vector3d forceOnBody_B;         //!< [N] Force that the state effector applies to the s/c
    Eigen::Vector3d torqueOnBodyPntB_B;    //!< [N] Torque that the state effector applies to the body about point B
    Eigen::Vector3d torqueOnBodyPntC_B;    //!< [N] Torque that the state effector applies to the body about point B
    
public:
    StateEffector();                       //!< -- Contructor
    virtual ~StateEffector();              //!< -- Destructor
    virtual void updateEffectorMassProps(double integTime);  //!< -- Method for stateEffector to give mass contributions
    virtual void updateContributions(double integTime, Eigen::Matrix3d & matrixAcontr, Eigen::Matrix3d & matrixBcontr,
                                     Eigen::Matrix3d & matrixCcontr, Eigen::Matrix3d & matrixDcontr, Eigen::Vector3d
                                     & vecTranscontr, Eigen::Vector3d & vecRotcontr);  //!< -- Back-sub contributions
    virtual void updateEnergyMomContributions(double integTime, Eigen::Vector3d & rotAngMomPntCContr_B,
                                              double & rotEnergyContr);  //!< -- Energy and momentum calculations
    virtual void modifyStates(double integTime); //!< -- Modify state values after integration
    virtual void calcForceTorqueOnBody(double integTime);  //!< -- Force and torque on s/c due to stateEffector
    virtual void writeOutputStateMessages(uint64_t integTimeNanos); //!< -- Write State Messages after integration
    virtual void registerStates(DynParamManager& states) = 0;  //!< -- Method for stateEffectors to register states
    virtual void linkInStates(DynParamManager& states) = 0;  //!< -- Method for stateEffectors to get other states
    virtual void computeDerivatives(double integTime)=0;  //!< -- Method for each stateEffector to calculate derivatives
};

#endif /* STATE_EFFECTOR_H */
