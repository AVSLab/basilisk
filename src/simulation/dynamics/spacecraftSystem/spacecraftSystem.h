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

#ifndef SPACECRAFT_DYNAMICS_H
#define SPACECRAFT_DYNAMICS_H

#include <vector>
#include <stdint.h>
#include "../_GeneralModuleFiles/dynParamManager.h"
#include "../_GeneralModuleFiles/stateEffector.h"
#include "../_GeneralModuleFiles/dynamicEffector.h"
#include "../_GeneralModuleFiles/gravityEffector.h"
#include "../_GeneralModuleFiles/dynamicObject.h"
#include "../_GeneralModuleFiles/stateVecIntegrator.h"
#include "../_GeneralModuleFiles/sys_model.h"
#include "../_GeneralModuleFiles/hubEffector.h"

#include "architecture/msgPayloadDefC/SCStatesMsgPayload.h"
#include "architecture/msgPayloadDefC/SCMassPropsMsgPayload.h"
#include "architecture/msgPayloadDefC/SCEnergyMomentumMsgPayload.h"
#include "architecture/messaging/messaging.h"

#include "architecture/utilities/bskLogging.h"


/*! @brief docking data structure */
struct DockingData {
    Eigen::Vector3d r_DB_B;  //!< variable
    Eigen::Matrix3d dcm_DB;  //!< variable
    Eigen::Vector3d r_DP_P;  //!< variable
    Eigen::Matrix3d dcm_DP;  //!< variable
    std::string portName;    //!< variable
    DockingData()
    {
        this->r_DB_B.setZero();
        this->r_DP_P.setZero();
        this->dcm_DB.setIdentity();
        this->dcm_DP.setIdentity();
        portName = "";
    }
};

/*! @brief spacecraft dynamic effector */
class SpacecraftUnit {
public:
    friend class SpacecraftSystem;

    bool docked;                         //!< class variable
    std::string spacecraftName;          //!< Name of the spacecraft so that multiple spacecraft can be distinguished
    Message<SCStatesMsgPayload> scStateOutMsg;       //!< Name of the state output message
    Message<SCMassPropsMsgPayload> scMassStateOutMsg;   //!< Name of the state output message
    Message<SCEnergyMomentumMsgPayload> scEnergyMomentumOutMsg;   //!< Name of the state output message

    double totOrbEnergy;                 //!< [J] Total orbital kinetic energy
    double totRotEnergy;                 //!< [J] Total rotational energy

    double rotEnergyContr;               //!< [J] Contribution of stateEffector to total rotational energy
    double orbPotentialEnergyContr;      //!< [J] Contribution of stateEffector to total rotational energy
    Eigen::Vector3d totOrbAngMomPntN_N;  //!< [kg m^2/s] Total orbital angular momentum about N in N frame compenents
    Eigen::Vector3d totRotAngMomPntC_N;  //!< [kg m^2/s] Total rotational angular momentum about C in N frame compenents
    Eigen::Vector3d rotAngMomPntCContr_B;  //!< [kg m^2/s] Contribution of stateEffector to total rotational angular mom.

    BackSubMatrices backSubMatricesContributions; //!< class variable

    Eigen::Vector3d sumForceExternal_N;  //!< [N] Sum of forces given in the inertial frame
    Eigen::Vector3d sumForceExternal_B;  //!< [N] Sum of forces given in the body frame
    Eigen::Vector3d sumTorquePntB_B;     //!< [N-m] Total torque about point B in B frame components

    Eigen::Vector3d oldV_CN_N;           //!< class variable
    Eigen::Vector3d oldV_BN_N;           //!< class variable
    Eigen::Vector3d oldOmega_BN_B;       //!< class variable

    Eigen::Vector3d dvAccum_CN_B;        //!< [m/s] Accumulated delta-v of center of mass relative to inertial frame in body frame coordinates
    Eigen::Vector3d dvAccum_BN_B;        //!< [m/s] accumulated delta-v of body frame relative to inertial frame in body frame coordinates
    Eigen::Vector3d nonConservativeAccelpntB_B;//!< [m/s/s] Current spacecraft body acceleration in the B frame
    Eigen::Vector3d omegaDot_BN_B;       //!< [rad/s/s] angular acceleration of body wrt to N in body frame



    HubEffector hub;                     //!< class variable
    GravityEffector gravField;           //!< Gravity effector for gravitational field experienced by spacecraft
    std::vector<StateEffector*> states;               //!< Vector of state effectors attached to dynObject
    std::vector<DynamicEffector*> dynEffectors;       //!< Vector of dynamic effectors attached to dynObject
    std::vector<DockingData*> dockingPoints;    //!< class variable


    Eigen::MatrixXd *inertialPositionProperty;  //!< [m] r_N inertial position relative to system spice zeroBase/refBase
    Eigen::MatrixXd *inertialVelocityProperty;  //!< [m] v_N inertial velocity relative to system spice zeroBase/refBase

    BSKLogger bskLogger;                      //!< BSK Logging

public:
    SpacecraftUnit();
    ~SpacecraftUnit();

    void addStateEffector(StateEffector *newStateEffector);  //!< Attaches a stateEffector to the system
    void addDynamicEffector(DynamicEffector *newDynamicEffector);  //!< Attaches a dynamicEffector
    void addDockingPort(DockingData *newDockingPort);  //!< Attaches a dynamicEffector

    void SelfInitSC(int64_t moduleID);                     //!< Lets spacecraft plus create its own msgs
    void ResetSC(uint64_t CurrentSimNanos);

    void writeOutputMessagesSC(uint64_t clockTime, int64_t moduleID); //!< Method to write all of the class output messages
    void linkInStatesSC(DynParamManager& statesIn);  //!< Method to get access to the hub's states
    void initializeDynamicsSC(DynParamManager& statesIn); //!< class method

private:
    template <typename Type>
    void assignStateParamNames(Type effector) {
        /* assign the state engine names for the parent rigid body states */
        effector->setStateNameOfPosition(this->hub.nameOfHubPosition);
        effector->setStateNameOfVelocity(this->hub.nameOfHubVelocity);
        effector->setStateNameOfSigma(this->hub.nameOfHubSigma);
        effector->setStateNameOfOmega(this->hub.nameOfHubOmega);

        /* assign the state engine names for the parent rigid property values */
        effector->setPropName_m_SC(this->propName_m_SC);
        effector->setPropName_mDot_SC(this->propName_mDot_SC);
        effector->setPropName_centerOfMassSC(this->propName_centerOfMassSC);
        effector->setPropName_inertiaSC(this->propName_inertiaSC);
        effector->setPropName_inertiaPrimeSC(this->propName_inertiaPrimeSC);
        effector->setPropName_centerOfMassPrimeSC(this->propName_centerOfMassPrimeSC);
        effector->setPropName_centerOfMassDotSC(this->propName_centerOfMassDotSC);
        effector->setPropName_inertialPosition(this->gravField.inertialPositionPropName);
        effector->setPropName_inertialVelocity(this->gravField.inertialVelocityPropName);
        effector->setPropName_vehicleGravity(this->gravField.vehicleGravityPropName);
    };

    Eigen::MatrixXd *m_SC;               //!< [kg] spacecrafts total mass
    Eigen::MatrixXd *mDot_SC;            //!< [kg/s] Time derivative of spacecrafts total mass
    Eigen::MatrixXd *ISCPntB_B;          //!< [kg m^2] Inertia of s/c about point B in B frame components
    Eigen::MatrixXd *c_B;                //!< [m] Vector from point B to CoM of s/c in B frame components
    Eigen::MatrixXd *cPrime_B;           //!< [m/s] Body time derivative of c_B
    Eigen::MatrixXd *cDot_B;             //!< [m/s] Inertial time derivative of c_B
    Eigen::MatrixXd *ISCPntBPrime_B;     //!< [kg m^2/s] Body time derivative of ISCPntB_B

    Eigen::MatrixXd *g_N;                //!< [m/s^2] Gravitational acceleration in N frame components
    StateData *hubR_N;                   //!< State data accesss to inertial position for the hub
    StateData *hubV_N;                   //!< State data access to inertial velocity for the hub
    StateData *hubOmega_BN_B;            //!< State data access to the attitude rate of the hub
    StateData *hubSigma;                 //!< State data access to sigmaBN for the hub
    StateData *hubGravVelocity;          //!< State data access to the gravity-accumulated DV on the Body frame
    StateData *BcGravVelocity;           //!< State data access to the gravity-accumulated DV on point Bc

    std::string propName_m_SC;                  //!< property name of m_SC
    std::string propName_mDot_SC;               //!< property name of mDot_SC
    std::string propName_centerOfMassSC;        //!< property name of centerOfMassSC
    std::string propName_inertiaSC;             //!< property name of inertiaSC
    std::string propName_inertiaPrimeSC;        //!< property name of inertiaPrimeSC
    std::string propName_centerOfMassPrimeSC;   //!< property name of centerOfMassPrimeSC
    std::string propName_centerOfMassDotSC;     //!< property name of centerOfMassDotSC

};


/*! @brief spacecraft dynamic effector */
class SpacecraftSystem : public DynamicObject{
public:

    uint64_t numOutMsgBuffers;           //!< Number of output message buffers for I/O
    std::string sysTimePropertyName;     //!< Name of the system time property
    SpacecraftUnit primaryCentralSpacecraft;   //!< Primary spacecraft in which other spacecraft can attach/detach to/from
    std::vector<SpacecraftUnit*> spacecraftDockedToPrimary; //!< vector of spacecraft currently docked with primary spacecraft
    std::vector<SpacecraftUnit*> unDockedSpacecraft; //!< vector of spacecraft currently detached from all other spacecraft
    int numberOfSCAttachedToPrimary;          //!< class variable
    BSKLogger bskLogger;                      //!< BSK Logging

public:
    SpacecraftSystem();                    //!< Constructor
    ~SpacecraftSystem();                   //!< Destructor
    void initializeDynamics();           //!< This method initializes all of the dynamics and variables for the s/c
    void computeEnergyMomentum(double time);  //!< This method computes the total energy and momentum of the s/c
    void computeEnergyMomentumSC(double time, SpacecraftUnit& spacecraft);  //!< This method computes the total energy and momentum of the s/c
    void computeEnergyMomentumSystem(double time);  //!< This method computes the total energy and momentum of the s/c
    void updateSpacecraftMassProps(double time, SpacecraftUnit& spacecraft);  //!< This method computes the total mass properties of the s/c
    void updateSystemMassProps(double time);  //!< This method computes the total mass properties of the s/c
    void initializeSCPosVelocity(SpacecraftUnit& spacecraft); //!< class method
    void Reset(uint64_t CurrentSimNanos);
    void writeOutputMessages(uint64_t clockTime); //!< Method to write all of the class output messages
    void UpdateState(uint64_t CurrentSimNanos);  //!< Runtime hook back into Basilisk arch
    void equationsOfMotion(double integTimeSeconds, double timeStep);    //!< This method computes the equations of motion for the whole system
    void equationsOfMotionSC(double integTimeSeconds, double timeStep, SpacecraftUnit& spacecraft);    //!< This method computes the equations of motion for the whole system
    void equationsOfMotionSystem(double integTimeSeconds, double timeStep);    //!< This method computes the equations of motion for the whole system
    void findPriorStateInformation(SpacecraftUnit& spacecraft);  //!< class method
    void calculateDeltaVandAcceleration(SpacecraftUnit& spacecraft, double localTimeStep); //!< class method
    void attachSpacecraftToPrimary(SpacecraftUnit *newSpacecraft, std::string dockingPortNameOfNewSpacecraft, std::string dockingToPortName);  //!< Attaches a spacecraft to the primary spacecraft chain
    void addSpacecraftUndocked(SpacecraftUnit *newSpacecraft);  //!< Attaches a spacecraft to the primary spacecraft chain
    void determineAttachedSCStates();  //!< class method
    void preIntegration(uint64_t callTime) final;  //!< pre-integration steps
    void postIntegration(uint64_t callTime) final;  //!< post-integration steps

private:
    Eigen::MatrixXd *sysTime;            //!< [s] System time

};


#endif /* SPACECRAFT_DYNAMICS_H */
