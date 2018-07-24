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
#include "../_GeneralModuleFiles/dynParamManager.h"
#include "../_GeneralModuleFiles/stateEffector.h"
#include "../_GeneralModuleFiles/dynamicEffector.h"
#include "../_GeneralModuleFiles/gravityEffector.h"
#include "../_GeneralModuleFiles/dynamicObject.h"
#include "../_GeneralModuleFiles/stateVecIntegrator.h"
#include "_GeneralModuleFiles/sys_model.h"
#include "simMessages/scPlusStatesSimMsg.h"
#include "simMessages/scPlusMassPropsSimMsg.h"
#include "hubEffector.h"

/*! @brief This is an instantiation of the dynamicObject abstract class that is a spacecraft with stateEffectors and
 dynamicEffectors attached to it. The spacecraftPlus allows for both translation and
 rotation. stateEffectors such as RWs, flexible solar panel, fuel slosh etc can be added to the spacecraft by attaching 
 stateEffectors. dynamicEffectors such as thrusters, external force and torque, SRP etc can be added to the spacecraft 
 by attaching dynamicEffectors. This class performs all of this interaction between stateEffectors, dynamicEffectors and
  the hub.

 The module
 [PDF Description](Basilisk-SPACECRAFTPLUS-20170808.pdf)
 contains further information on this module's function,
 how to run it, as well as testing.

 */
class SpacecraftPlus : public DynamicObject{
public:
    uint64_t simTimePrevious;            //!< -- Previous simulation time
    uint64_t numOutMsgBuffers;           //!< -- Number of output message buffers for I/O
    std::string sysTimePropertyName;     //!< -- Name of the system time property
    std::string scStateOutMsgName;       //!< -- Name of the state output message
    std::string scMassStateOutMsgName;   //!< -- Name of the state output message
    double totOrbEnergy;                 //!< [J] Total orbital kinetic energy
    double totRotEnergy;                 //!< [J] Total rotational energy
    double rotEnergyContr;               //!< [J] Contribution of stateEffector to total rotational energy
    double orbPotentialEnergyContr;      //!< [J] Contribution of stateEffector to total rotational energy
    double currTimeStep;                 //!< [s] Time after integration, used for dvAccum calculation
    double timePrevious;                 //!< [s] Time before integration, used for dvAccum calculation
    BackSubMatrices backSubContributions;
    Eigen::Vector3d sumForceExternal_N;  //!< [N] Sum of forces given in the inertial frame
    Eigen::Vector3d sumForceExternal_B;  //!< [N] Sum of forces given in the body frame
    Eigen::Vector3d sumTorquePntB_B;     //!< [N-m] Total torque about point B in B frame components
    Eigen::MatrixXd *m_SC;               //!< [kg] spacecrafts total mass
    Eigen::MatrixXd *mDot_SC;            //!< [kg/s] Time derivative of spacecrafts total mass
    Eigen::MatrixXd *ISCPntB_B;          //!< [kg m^2] Inertia of s/c about point B in B frame components
    Eigen::MatrixXd *c_B;                //!< [m] Vector from point B to CoM of s/c in B frame components
    Eigen::MatrixXd *cPrime_B;           //!< [m/s] Body time derivative of c_B
    Eigen::MatrixXd *cDot_B;             //!< [m/s] Inertial time derivative of c_B
    Eigen::MatrixXd *ISCPntBPrime_B;     //!< [kg m^2/s] Body time derivative of ISCPntB_B
    Eigen::MatrixXd *g_N;                //!< [m/s^2] Gravitational acceleration in N frame components
    Eigen::MatrixXd *sysTime;            //!< [s] System time
    Eigen::Vector3d dvAccum_B;           //!< [m/s] Accumulated delta-v of center of mass relative to inertial frame in body frame coordinates
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
    
public:
    SpacecraftPlus();                    //!< -- Constructor
    ~SpacecraftPlus();                   //!< -- Destructor
    void initializeDynamics();           //!< -- This method initializes all of the dynamics and variables for the s/c
    void computeEnergyMomentum(double time);  //!< -- This method computes the total energy and momentum of the s/c
    void updateSCMassProps(double time);  //!< -- This method computes the total mass properties of the s/c
    void calcForceTorqueFromStateEffectors(double time, Eigen::Vector3d omega_BN_B);  //!< -- This method computes the force and torque from the stateEffectors
    void SelfInit();                     //!< -- Lets spacecraft plus create its own msgs
    void CrossInit();                    //!< -- Hook to tie s/c plus back into provided msgs
	void writeOutputStateMessages(uint64_t clockTime); //!< -- Method to write all of the class output messages
    void UpdateState(uint64_t CurrentSimNanos);  //!< -- Runtime hook back into Basilisk arch
    void linkInStates(DynParamManager& statesIn);  //!< Method to get access to the hub's states
    void equationsOfMotion(double integTimeSeconds);    //!< -- This method computes the equations of motion for the whole system
    void integrateState(double time);       //!< -- This method steps the state forward one step in time
    void addStateEffector(StateEffector *newSateEffector);  //!< -- Attaches a stateEffector to the system
    void addDynamicEffector(DynamicEffector *newDynamicEffector);  //!< -- Attaches a dynamicEffector

private:
    StateData *hubR_N;                          //!< -- State data accesss to inertial position for the hub
    StateData *hubV_N;                          //!< -- State data access to inertial velocity for the hub
    StateData *hubOmega_BN_B;                   //!< -- State data access to the attitude rate of the hub
    StateData *hubSigma;                        //!< -- State data access to sigmaBN for the hub
    Eigen::MatrixXd *inertialPositionProperty;  //!< [m] r_N inertial position relative to system spice zeroBase/refBase
    Eigen::MatrixXd *inertialVelocityProperty;  //!< [m] v_N inertial velocity relative to system spice zeroBase/refBase
    int64_t scStateOutMsgId;                    //!< -- Message ID for the outgoing spacecraft state
    int64_t scMassStateOutMsgId;                //!< -- Message ID for the outgoing spacecraft mass state
};

#endif /* SPACECRAFT_PLUS_H */
