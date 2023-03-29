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

#ifndef SPACECRAFT_PLUS_H
#define SPACECRAFT_PLUS_H

#include <vector>
#include <stdint.h>
#include "simulation/dynamics/_GeneralModuleFiles/dynParamManager.h"
#include "simulation/dynamics/_GeneralModuleFiles/stateEffector.h"
#include "simulation/dynamics/_GeneralModuleFiles/dynamicEffector.h"
#include "simulation/dynamics/_GeneralModuleFiles/gravityEffector.h"
#include "simulation/dynamics/_GeneralModuleFiles/dynamicObject.h"
#include "simulation/dynamics/_GeneralModuleFiles/stateVecIntegrator.h"
#include "architecture/_GeneralModuleFiles/sys_model.h"

#include "architecture/msgPayloadDefC/SCStatesMsgPayload.h"
#include "architecture/msgPayloadDefC/SCMassPropsMsgPayload.h"
#include "architecture/msgPayloadDefC/AttRefMsgPayload.h"
#include "architecture/msgPayloadDefC/TransRefMsgPayload.h"

#include "../_GeneralModuleFiles/hubEffector.h"
#include "architecture/utilities/bskLogging.h"
#include "architecture/messaging/messaging.h"



/*! @brief spacecraft dynamic effector */
class Spacecraft : public DynamicObject{
public:
    uint64_t simTimePrevious;            //!< -- Previous simulation time
    std::string sysTimePropertyName;     //!< -- Name of the system time property
    ReadFunctor<AttRefMsgPayload> attRefInMsg; //!< -- (optional) reference attitude input message name
    ReadFunctor<TransRefMsgPayload> transRefInMsg; //!< -- (optional) reference translation input message name
    double totOrbEnergy;                 //!< [J] Total orbital kinetic energy
    double totRotEnergy;                 //!< [J] Total rotational energy
    double rotEnergyContr;               //!< [J] Contribution of stateEffector to total rotational energy
    double orbPotentialEnergyContr;      //!< [J] Contribution of stateEffector to total rotational energy
    double currTimeStep;                 //!< [s] Time after integration, used for dvAccum calculation
    double timePrevious;                 //!< [s] Time before integration, used for dvAccum calculation
    BackSubMatrices backSubContributions;//!< class variable
    Eigen::Vector3d sumForceExternal_N;  //!< [N] Sum of forces given in the inertial frame
    Eigen::Vector3d sumForceExternal_B;  //!< [N] Sum of forces given in the body frame
    Eigen::Vector3d sumTorquePntB_B;     //!< [N-m] Total torque about point B in B frame components
    
    Eigen::Vector3d dvAccum_CN_B;           //!< [m/s] Accumulated delta-v of center of mass relative to inertial frame in body frame coordinates
    Eigen::Vector3d dvAccum_BN_B;        //!< [m/s] accumulated delta-v of body frame relative to inertial frame in body frame coordinates
    Eigen::Vector3d nonConservativeAccelpntB_B;//!< [m/s/s] Current spacecraft body acceleration in the B frame
    Eigen::Vector3d omegaDot_BN_B;       //!< [rad/s/s] angular acceleration of body wrt to N in body frame
    Eigen::Vector3d totOrbAngMomPntN_N;  //!< [kg m^2/s] Total orbital angular momentum about N in N frame compenents
    Eigen::Vector3d totRotAngMomPntC_N;  //!< [kg m^2/s] Total rotational angular momentum about C in N frame compenents
    Eigen::Vector3d rotAngMomPntCContr_B;  //!< [kg m^2/s] Contribution of stateEffector to total rotational angular mom.
    HubEffector hub;                     //!< -- The spacecraft plus needs access to the spacecraft hub
    GravityEffector gravField;           //!< -- Gravity effector for gravitational field experienced by spacecraft
    std::vector<StateEffector*> states;               //!< -- Vector of state effectors attached to dynObject
    std::vector<DynamicEffector*> dynEffectors;       //!< -- Vector of dynamic effectors attached to dynObject
    BSKLogger bskLogger;                      //!< -- BSK Logging
    Message<SCStatesMsgPayload> scStateOutMsg;      //!< spacecraft state output message
    Message<SCMassPropsMsgPayload> scMassOutMsg;    //!< spacecraft mass properties output message

public:
    Spacecraft();                    //!< -- Constructor
    ~Spacecraft();                   //!< -- Destructor
    void initializeDynamics();           //!< -- This method initializes all of the dynamics and variables for the s/c
    void computeEnergyMomentum(double time);  //!< -- This method computes the total energy and momentum of the s/c
    void updateSCMassProps(double time);  //!< -- This method computes the total mass properties of the s/c
    void calcForceTorqueFromStateEffectors(double time, Eigen::Vector3d omega_BN_B);  //!< -- This method computes the force and torque from the stateEffectors
    void Reset(uint64_t CurrentSimNanos);
	void writeOutputStateMessages(uint64_t clockTime); //!< -- Method to write all of the class output messages
    void UpdateState(uint64_t CurrentSimNanos);  //!< -- Runtime hook back into Basilisk arch
    void linkInStates(DynParamManager& statesIn);  //!< Method to get access to the hub's states
    void equationsOfMotion(double integTimeSeconds, double timeStep);    //!< -- This method computes the equations of motion for the whole system
    void integrateState(double time);       //!< -- This method steps the state forward one step in time
    void addStateEffector(StateEffector *newSateEffector);  //!< -- Attaches a stateEffector to the system
    void addDynamicEffector(DynamicEffector *newDynamicEffector);  //!< -- Attaches a dynamicEffector
    void preIntegration(double callTime);       //!< -- method to perform pre-integration steps
    void postIntegration(double callTime);      //!< -- method to perform post-integration steps

private:
    StateData *hubR_N;                          //!< -- State data accesss to inertial position for the hub
    StateData *hubV_N;                          //!< -- State data access to inertial velocity for the hub
    StateData *hubOmega_BN_B;                   //!< -- State data access to the attitude rate of the hub
    StateData *hubSigma;                        //!< -- State data access to sigmaBN for the hub
    StateData *hubGravVelocity;                 //!< -- State data access to the gravity-accumulated DV on the Body frame
    StateData *BcGravVelocity;                  //!< -- State data access to the gravity-accumulated DV on point Bc
    Eigen::MatrixXd *inertialPositionProperty;  //!< [m] r_N inertial position relative to system spice zeroBase/refBase
    Eigen::MatrixXd *inertialVelocityProperty;  //!< [m] v_N inertial velocity relative to system spice zeroBase/refBase


    Eigen::MatrixXd *m_SC;               //!< [kg] spacecrafts total mass
    Eigen::MatrixXd *mDot_SC;            //!< [kg/s] Time derivative of spacecrafts total mass
    Eigen::MatrixXd *ISCPntB_B;          //!< [kg m^2] Inertia of s/c about point B in B frame components
    Eigen::MatrixXd *c_B;                //!< [m] Vector from point B to CoM of s/c in B frame components
    Eigen::MatrixXd *cPrime_B;           //!< [m/s] Body time derivative of c_B
    Eigen::MatrixXd *cDot_B;             //!< [m/s] Inertial time derivative of c_B
    Eigen::MatrixXd *ISCPntBPrime_B;     //!< [kg m^2/s] Body time derivative of ISCPntB_B
    Eigen::MatrixXd *g_N;                //!< [m/s^2] Gravitational acceleration in N frame components
    Eigen::MatrixXd *sysTime;            //!< [s] System time

    double localTimeStep;                //!< [s] integration time step
    double timeBefore;                   //!< [s] prior time value
    Eigen::Vector3d oldOmega_BN_B;       //!< [r/s] prior angular rate of B wrt N in the Body frame

private:
    void readOptionalRefMsg();                  //!< -- Read the optional attitude or translational reference input message and set the reference states
};


#endif /* SPACECRAFT_PLUS_H */
