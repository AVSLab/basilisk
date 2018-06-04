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

#include "spacecraftDynamics.h"
#include "simFswInterfaceMessages/macroDefinitions.h"
#include "../_GeneralModuleFiles/svIntegratorRK4.h"
#include "utilities/avsEigenSupport.h"
#include "utilities/avsEigenMRP.h"
#include <iostream>

Spacecraft::Spacecraft()
{
    // - Set default names
    this->spacecraftName = "spacecraft";
    this->scStateOutMsgName = "inertial_state_output";
    this->scMassStateOutMsgName = "mass_state_output";
    this->numOutMsgBuffers = 2;

    // - Set values to either zero or default values
    this->scStateOutMsgId = -1;
    this->dvAccum_B.setZero();
    this->dvAccum_BN_B.setZero();

    return;
}

Spacecraft::~Spacecraft()
{
    return;
}

void Spacecraft::addStateEffector(StateEffector *newStateEffector)
{
    this->states.push_back(newStateEffector);

    // Give the stateEffector the name of the spacecraft it is attached to
    newStateEffector->nameOfSpacecraftAttachedTo = this->spacecraftName;

    return;
}

void Spacecraft::addDynamicEffector(DynamicEffector *newDynamicEffector)
{
    this->dynEffectors.push_back(newDynamicEffector);

    return;
}

void Spacecraft::writeOutputMessages(uint64_t clockTime)
{
    return;
}
void Spacecraft::linkInStates(DynParamManager& statesIn)
{
    return;
}

/*! This is the constructor, setting variables to default values */
SpacecraftDynamics::SpacecraftDynamics()
{
    // - Set default names
    this->sysTimePropertyName = "systemTime";

    // - Set values to either zero or default values
    this->currTimeStep = 0.0;
    this->timePrevious = 0.0;
    this->simTimePrevious = 0;
    this->numOutMsgBuffers = 2;

    // - Set integrator as RK4 by default
    this->integrator = new svIntegratorRK4(this);

    return;
}

/*! This is the destructor, nothing to report here */
SpacecraftDynamics::~SpacecraftDynamics()
{
    return;
}

/*! This method creates the messages for s/c output data and initializes the gravity field*/
void SpacecraftDynamics::SelfInit()
{
    // - Call this for the primary spacecraft
    // - Create the message for the spacecraft state
    this->primaryCentralSpacecraft.scStateOutMsgName = this->primaryCentralSpacecraft.spacecraftName + "_" + this->primaryCentralSpacecraft.scStateOutMsgName;
    this->primaryCentralSpacecraft.scMassStateOutMsgName = this->primaryCentralSpacecraft.spacecraftName + "_" + this->primaryCentralSpacecraft.scMassStateOutMsgName;
    this->primaryCentralSpacecraft.scStateOutMsgId = SystemMessaging::GetInstance()->CreateNewMessage(this->primaryCentralSpacecraft.scStateOutMsgName,
                                                                             sizeof(SCStatesSimMsg),
                                                                             this->primaryCentralSpacecraft.numOutMsgBuffers,
                                                                             "SCPlusStatesSimMsg", this->moduleID);
    // - Create the message for the spacecraft mass state
    this->primaryCentralSpacecraft.scMassStateOutMsgId = SystemMessaging::GetInstance()->CreateNewMessage(this->primaryCentralSpacecraft.scMassStateOutMsgName,
                                                                                 sizeof(SCMassPropsSimMsg),
                                                                                 this->primaryCentralSpacecraft.numOutMsgBuffers,
                                                                                 "SCPlusMassPropsSimMsg", this->moduleID);
    // - Call the gravity fields selfInit method
    this->primaryCentralSpacecraft.gravField.SelfInit();

    return;
}

/*! This method is used to cross link the messages and to initialize the dynamics */
void SpacecraftDynamics::CrossInit()
{
    // - Call gravity field cross initialization for all spacecraft
    this->primaryCentralSpacecraft.gravField.CrossInit();
    
    // - Call method for initializing the dynamics of spacecraftPlus
    this->initializeDynamics();

    return;
}

void Spacecraft::addDockingPort(DockingData *newDockingPort)
{
    this->dockingPoints.push_back(newDockingPort);

    return;
}

/*! This method attaches a stateEffector to the dynamicObject */
void SpacecraftDynamics::addSpacecraftUndocked(Spacecraft *newSpacecraft)
{
    this->unDockedSpacecraft.push_back(newSpacecraft);

    return;
}

/*! This method attaches a stateEffector to the dynamicObject */
void SpacecraftDynamics::attachSpacecraftToPrimary(Spacecraft *newSpacecraft, std::string dockingPortNameOfNewSpacecraft, std::string dockingToPortName)
{
    // Create chain of docked spacecraft
    std::vector<DockingData*>::iterator dockingItPrimary;
    int checkDock = 0;
    for(dockingItPrimary = this->primaryCentralSpacecraft.dockingPoints.begin(); dockingItPrimary != this->primaryCentralSpacecraft.dockingPoints.end(); dockingItPrimary++)
    {
        std::vector<DockingData*>::iterator dockingIt;
        if (dockingToPortName == (*dockingItPrimary)->portName) {
            for(dockingIt = newSpacecraft->dockingPoints.begin(); dockingIt != newSpacecraft->dockingPoints.end(); dockingIt++)
            {
                if (dockingPortNameOfNewSpacecraft == (*dockingIt)->portName) {
                    (*dockingIt)->r_DP_P = (*dockingItPrimary)->r_DP_P;
                    (*dockingIt)->dcm_DP = (*dockingItPrimary)->dcm_DP;
                    newSpacecraft->hub.dcm_BP = (*dockingIt)->dcm_DB.transpose()*(*dockingIt)->dcm_DP;
                    newSpacecraft->hub.r_BP_P = (*dockingIt)->r_DP_P - newSpacecraft->hub.dcm_BP.transpose()*(*dockingIt)->r_DB_B;
                    checkDock += 1;
                } else {
                    std::cerr << __FILE__ <<": Port name given is not the same port name held in the newSpacecraft";
                    std::cerr << "  Quitting."<<std::endl;
                }

            }
        }
    }

    if (checkDock < 1) {
        std::vector<Spacecraft*>::iterator spacecraftConnectedIt;
        for(spacecraftConnectedIt = this->spacecraftDockedToPrimary.begin(); spacecraftConnectedIt != this->spacecraftDockedToPrimary.end(); spacecraftConnectedIt++)
        {
            std::vector<DockingData*>::iterator dockingItConnected;
            for(dockingItConnected = (*spacecraftConnectedIt)->dockingPoints.begin(); dockingItConnected != (*spacecraftConnectedIt)->dockingPoints.end(); dockingItConnected++)
            {
                std::vector<DockingData*>::iterator dockingIt;
                if (dockingToPortName == (*dockingItConnected)->portName) {
                    for(dockingIt = newSpacecraft->dockingPoints.begin(); dockingIt != newSpacecraft->dockingPoints.end(); dockingIt++)
                    {
                        if (dockingPortNameOfNewSpacecraft == (*dockingIt)->portName) {
                            (*dockingIt)->r_DP_P = (*dockingItConnected)->r_DP_P;
                            (*dockingIt)->dcm_DP = (*dockingItConnected)->dcm_DP;
                            newSpacecraft->hub.dcm_BP = (*dockingIt)->dcm_DB.transpose()*(*dockingIt)->dcm_DP;
                            newSpacecraft->hub.r_BP_P = (*dockingIt)->r_DP_P - newSpacecraft->hub.dcm_BP.transpose()*(*dockingIt)->r_DB_B;
                            checkDock += 1;
                        } else {
                            std::cerr << __FILE__ <<": Port name given is not the same port name held in the newSpacecraft";
                            std::cerr << "  Quitting."<<std::endl;
                        }

                    }
                }

            }

        }
    }

    if (checkDock < 1) {
        std::cerr << __FILE__ <<": The new spacecraft did not get attached due to naming problems with the ports";
        std::cerr << "  Quitting."<<std::endl;
    }

    this->spacecraftDockedToPrimary.push_back(newSpacecraft);

    return;
}

/*! This is the method where the messages of the state of vehicle are written */
void SpacecraftDynamics::writeOutputMessages(uint64_t clockTime)
{
    // - Write output messages for each spacecraft
    // - Populate state output message
    SCStatesSimMsg stateOut;
    eigenMatrixXd2CArray(*this->primaryCentralSpacecraft.inertialPositionProperty, stateOut.r_BN_N);
    eigenMatrixXd2CArray(*this->primaryCentralSpacecraft.inertialVelocityProperty, stateOut.v_BN_N);
    Eigen::MRPd sigmaLocal_BN;
    sigmaLocal_BN = (Eigen::Vector3d) this->primaryCentralSpacecraft.hubSigma->getState();
    Eigen::Matrix3d dcm_NB = sigmaLocal_BN.toRotationMatrix();
    Eigen::Vector3d rLocal_CN_N = (*this->primaryCentralSpacecraft.inertialPositionProperty) + dcm_NB*(*this->primaryCentralSpacecraft.c_B);
    Eigen::Vector3d vLocal_CN_N = (*this->primaryCentralSpacecraft.inertialVelocityProperty) + dcm_NB*(*this->primaryCentralSpacecraft.cDot_B);
    eigenVector3d2CArray(rLocal_CN_N, stateOut.r_CN_N);
    eigenVector3d2CArray(vLocal_CN_N, stateOut.v_CN_N);
    eigenMatrixXd2CArray(this->primaryCentralSpacecraft.hubSigma->getState(), stateOut.sigma_BN);
    eigenMatrixXd2CArray(this->primaryCentralSpacecraft.hubOmega_BN_B->getState(), stateOut.omega_BN_B);
    eigenMatrixXd2CArray(this->primaryCentralSpacecraft.dvAccum_B, stateOut.TotalAccumDVBdy);
    stateOut.MRPSwitchCount = this->primaryCentralSpacecraft.hub.MRPSwitchCount;
    eigenMatrixXd2CArray(this->primaryCentralSpacecraft.dvAccum_BN_B, stateOut.TotalAccumDV_BN_B);
    eigenVector3d2CArray(this->primaryCentralSpacecraft.nonConservativeAccelpntB_B, stateOut.nonConservativeAccelpntB_B);
    eigenVector3d2CArray(this->primaryCentralSpacecraft.omegaDot_BN_B, stateOut.omegaDot_BN_B);
    SystemMessaging::GetInstance()->WriteMessage(this->primaryCentralSpacecraft.scStateOutMsgId, clockTime, sizeof(SCStatesSimMsg),
                                                 reinterpret_cast<uint8_t*> (&stateOut), this->moduleID);

    // - Populate mass state output message
    SCMassPropsSimMsg massStateOut;
    massStateOut.massSC = (*this->primaryCentralSpacecraft.m_SC)(0,0);
    eigenMatrixXd2CArray(*this->primaryCentralSpacecraft.c_B, massStateOut.c_B);
    eigenMatrixXd2CArray(*this->primaryCentralSpacecraft.ISCPntB_B, (double *)massStateOut.ISC_PntB_B);
    SystemMessaging::GetInstance()->WriteMessage(this->primaryCentralSpacecraft.scMassStateOutMsgId, clockTime, sizeof(SCMassPropsSimMsg),
                                                 reinterpret_cast<uint8_t*> (&massStateOut), this->moduleID);

    return;
}

/*! This method is a part of sysModel and is used to integrate the state and update the state in the messaging system */
void SpacecraftDynamics::UpdateState(uint64_t CurrentSimNanos)
{
    // - Convert current time to seconds
    double newTime = CurrentSimNanos*NANO2SEC;

    // - Get access to the spice bodies
    this->primaryCentralSpacecraft.gravField.UpdateState(CurrentSimNanos);

    // - Integrate the state forward in time
    this->integrateState(newTime);
    // - Update the inertial position of each spacecraft
    Eigen::Vector3d rLocal_BN_N = this->primaryCentralSpacecraft.hubR_N->getState();
    Eigen::Vector3d vLocal_BN_N = this->primaryCentralSpacecraft.hubV_N->getState();
    this->primaryCentralSpacecraft.gravField.updateInertialPosAndVel(rLocal_BN_N, vLocal_BN_N);

    // - Write the state of the vehicle into messages
    this->writeOutputMessages(CurrentSimNanos);
    this->simTimePrevious = CurrentSimNanos;

    return;
}

/*! This method allows the spacecraftDynamics to have access to the current state of the hub for MRP switching, writing 
 messages, and calculating energy and momentum */
void SpacecraftDynamics::linkInStates(DynParamManager& statesIn)
{
    // - Get access to all spacecraft hub states
    this->primaryCentralSpacecraft.hubR_N = statesIn.getStateObject(this->primaryCentralSpacecraft.hub.nameOfHubPosition);
    this->primaryCentralSpacecraft.hubV_N = statesIn.getStateObject(this->primaryCentralSpacecraft.hub.nameOfHubVelocity);
    this->primaryCentralSpacecraft.hubSigma = statesIn.getStateObject(this->primaryCentralSpacecraft.hub.nameOfHubSigma);   /* Need sigmaBN for MRP switching */
    this->primaryCentralSpacecraft.hubOmega_BN_B = statesIn.getStateObject(this->primaryCentralSpacecraft.hub.nameOfHubOmega);

    // - Get access to the hubs position and velocity in the property manager
    this->primaryCentralSpacecraft.inertialPositionProperty = statesIn.getPropertyReference(this->primaryCentralSpacecraft.gravField.inertialPositionPropName);
    this->primaryCentralSpacecraft.inertialVelocityProperty = statesIn.getPropertyReference(this->primaryCentralSpacecraft.gravField.inertialVelocityPropName);
    this->primaryCentralSpacecraft.g_N = statesIn.getPropertyReference(this->primaryCentralSpacecraft.gravField.vehicleGravityPropName);

    return;
}

/*! This method is used to initialize the simulation by registering all of the states, linking the dynamicEffectors,
 stateEffectors, and the hub, initialize gravity, and initialize the sim with the initial conditions specified in python
 for the simulation */
void SpacecraftDynamics::initializeDynamics()
{
    // - SpacecraftPlus initiates all of the spaceCraft mass properties
    Eigen::MatrixXd initM_SC(1,1);
    Eigen::MatrixXd initMDot_SC(1,1);
    Eigen::MatrixXd initC_B(3,1);
    Eigen::MatrixXd initISCPntB_B(3,3);
    Eigen::MatrixXd initCPrime_B(3,1);
    Eigen::MatrixXd initCDot_B(3,1);
    Eigen::MatrixXd initISCPntBPrime_B(3,3);
    Eigen::MatrixXd systemTime(2,1);
    systemTime.setZero();
    // - Create the properties
    std::string tmpName;
    tmpName = this->primaryCentralSpacecraft.spacecraftName + "_" + "m_SC";
    this->primaryCentralSpacecraft.m_SC = this->dynManager.createProperty(tmpName, initM_SC);
    tmpName = this->primaryCentralSpacecraft.spacecraftName + "_" + "mDot_SC";
    this->primaryCentralSpacecraft.mDot_SC = this->dynManager.createProperty(tmpName, initMDot_SC);
    tmpName = this->primaryCentralSpacecraft.spacecraftName + "_" + "centerOfMassSC";
    this->primaryCentralSpacecraft.c_B = this->dynManager.createProperty(tmpName, initC_B);
    tmpName = this->primaryCentralSpacecraft.spacecraftName + "_" + "inertiaSC";
    this->primaryCentralSpacecraft.ISCPntB_B = this->dynManager.createProperty(tmpName, initISCPntB_B);
    tmpName = this->primaryCentralSpacecraft.spacecraftName + "_" + "inertiaPrimeSC";
    this->primaryCentralSpacecraft.ISCPntBPrime_B = this->dynManager.createProperty(tmpName, initISCPntBPrime_B);
    tmpName = this->primaryCentralSpacecraft.spacecraftName + "_" + "centerOfMassPrimeSC";
    this->primaryCentralSpacecraft.cPrime_B = this->dynManager.createProperty(tmpName, initCPrime_B);
    tmpName = this->primaryCentralSpacecraft.spacecraftName + "_" + "centerOfMassDotSC";
    this->primaryCentralSpacecraft.cDot_B = this->dynManager.createProperty(tmpName, initCDot_B);
    this->sysTime = this->dynManager.createProperty(this->sysTimePropertyName, systemTime);

    // - Give name of all spacecraft to attached hubEffector
    this->primaryCentralSpacecraft.hub.nameOfSpacecraftAttachedTo = this->primaryCentralSpacecraft.spacecraftName;
    // - Give name of all spacecraft to attached hubEffector
    this->primaryCentralSpacecraft.gravField.nameOfSpacecraftAttachedTo = this->primaryCentralSpacecraft.spacecraftName;
    
    // - Before er'body registers their properties, we need to prepend their state names with the spacecraft
    this->primaryCentralSpacecraft.hub.prependSpacecraftNameToStates();
    this->primaryCentralSpacecraft.gravField.prependSpacecraftNameToStates();
    std::vector<StateEffector*>::iterator stateIt;
    for(stateIt = this->primaryCentralSpacecraft.states.begin(); stateIt != this->primaryCentralSpacecraft.states.end(); stateIt++)
    {
        (*stateIt)->prependSpacecraftNameToStates();
    }
    

    // - Register the gravity properties with the dynManager, 'erbody wants g_N!
    this->primaryCentralSpacecraft.gravField.registerProperties(this->dynManager);

    // - Register the hub states
    this->primaryCentralSpacecraft.hub.registerStates(this->dynManager);

    // - Loop through stateEffectors to register their states
    for(stateIt = this->primaryCentralSpacecraft.states.begin(); stateIt != this->primaryCentralSpacecraft.states.end(); stateIt++)
    {
        (*stateIt)->registerStates(this->dynManager);
    }
    
    // - Link in states for the spaceCraftPlus, gravity and the hub
    this->linkInStates(this->dynManager);
    this->primaryCentralSpacecraft.gravField.linkInStates(this->dynManager);
    this->primaryCentralSpacecraft.hub.linkInStates(this->dynManager);

    // - Update the mass properties of the spacecraft to retrieve c_B and cDot_B to update r_BN_N and v_BN_N
    this->updateSystemMassProps(0.0);

    // - Edit r_BN_N and v_BN_N to take into account that point B and point C are not coincident
    // - Pulling the state from the hub at this time gives us r_CN_N
    Eigen::Vector3d rInit_BN_N = this->primaryCentralSpacecraft.hubR_N->getState();
    Eigen::MRPd sigma_BN;
    sigma_BN = (Eigen::Vector3d) this->primaryCentralSpacecraft.hubSigma->getState();
    Eigen::Matrix3d dcm_NB = sigma_BN.toRotationMatrix();
    // - Substract off the center mass to leave r_BN_N
    rInit_BN_N -= dcm_NB*(*this->primaryCentralSpacecraft.c_B);
    // - Subtract off cDot_B to get v_BN_N
    Eigen::Vector3d vInit_BN_N = this->primaryCentralSpacecraft.hubV_N->getState();
    vInit_BN_N -= dcm_NB*(*this->primaryCentralSpacecraft.cDot_B);
    // - Finally set the translational states r_BN_N and v_BN_N with the corrections
    this->primaryCentralSpacecraft.hubR_N->setState(rInit_BN_N);
    this->primaryCentralSpacecraft.hubV_N->setState(vInit_BN_N);

    // - Loop through the stateEffectros to link in the states needed
    for(stateIt = this->primaryCentralSpacecraft.states.begin(); stateIt != this->primaryCentralSpacecraft.states.end(); stateIt++)
    {
        (*stateIt)->linkInStates(this->dynManager);
    }

    // - Loop through the dynamicEffectors to link in the states needed
    std::vector<DynamicEffector*>::iterator dynIt;
    for(dynIt = this->primaryCentralSpacecraft.dynEffectors.begin(); dynIt != this->primaryCentralSpacecraft.dynEffectors.end(); dynIt++)
    {
        (*dynIt)->linkInStates(this->dynManager);
    }

    // - Call all stateEffectors in each spacecraft to give them body frame information
    std::vector<Spacecraft*>::iterator spacecraftIt;
    for(spacecraftIt = this->spacecraftDockedToPrimary.begin(); spacecraftIt != this->spacecraftDockedToPrimary.end(); spacecraftIt++)
    {
        for(stateIt = (*spacecraftIt)->states.begin(); stateIt != (*spacecraftIt)->states.end(); stateIt++)
        {
            // Obviously need to change this
            Eigen::Vector3d vectorPlaceHolder;
            Eigen::Matrix3d MatrixPlaceHolder;
            (*stateIt)->receiveMotherSpacecraftData(vectorPlaceHolder, MatrixPlaceHolder);
        }
    }

    // - Call equations of motion at time zero
    this->equationsOfMotion(0.0);

    return;
}

/*! This method is used to update the mass properties of the entire spacecraft using contributions from stateEffectors */
void SpacecraftDynamics::updateSystemMassProps(double time)
{
    // - Zero the properties which will get populated in this method
    (*this->primaryCentralSpacecraft.m_SC).setZero();
    (*this->primaryCentralSpacecraft.mDot_SC).setZero();
    (*this->primaryCentralSpacecraft.c_B).setZero();
    (*this->primaryCentralSpacecraft.ISCPntB_B).setZero();
    (*this->primaryCentralSpacecraft.cPrime_B).setZero();
    (*this->primaryCentralSpacecraft.cDot_B).setZero();
    (*this->primaryCentralSpacecraft.ISCPntBPrime_B).setZero();

    // Add in hubs mass props to the spacecraft mass props
    this->primaryCentralSpacecraft.hub.updateEffectorMassProps(time);
    (*this->primaryCentralSpacecraft.m_SC)(0,0) += this->primaryCentralSpacecraft.hub.effProps.mEff;
    (*this->primaryCentralSpacecraft.ISCPntB_B) += this->primaryCentralSpacecraft.hub.effProps.IEffPntB_B;
    (*this->primaryCentralSpacecraft.c_B) += this->primaryCentralSpacecraft.hub.effProps.mEff*this->primaryCentralSpacecraft.hub.effProps.rEff_CB_B;

    // - Loop through state effectors to get mass props
    std::vector<StateEffector*>::iterator it;
    for(it = this->primaryCentralSpacecraft.states.begin(); it != this->primaryCentralSpacecraft.states.end(); it++)
    {
        (*it)->updateEffectorMassProps(time);
        // - Add in effectors mass props into mass props of spacecraft
        (*this->primaryCentralSpacecraft.m_SC)(0,0) += (*it)->effProps.mEff;
        (*this->primaryCentralSpacecraft.mDot_SC)(0,0) += (*it)->effProps.mEffDot;
        (*this->primaryCentralSpacecraft.ISCPntB_B) += (*it)->effProps.IEffPntB_B;
        (*this->primaryCentralSpacecraft.c_B) += (*it)->effProps.mEff*(*it)->effProps.rEff_CB_B;
        (*this->primaryCentralSpacecraft.ISCPntBPrime_B) += (*it)->effProps.IEffPrimePntB_B;
        (*this->primaryCentralSpacecraft.cPrime_B) += (*it)->effProps.mEff*(*it)->effProps.rEffPrime_CB_B;
        // For high fidelity mass depletion, this is left out: += (*it)->effProps.mEffDot*(*it)->effProps.rEff_CB_B
    }

    // Divide c_B and cPrime_B by the total mass of the spaceCraft to finalize c_B and cPrime_B
    (*this->primaryCentralSpacecraft.c_B) = (*this->primaryCentralSpacecraft.c_B)/(*this->primaryCentralSpacecraft.m_SC)(0,0);
    (*this->primaryCentralSpacecraft.cPrime_B) = (*this->primaryCentralSpacecraft.cPrime_B)/(*this->primaryCentralSpacecraft.m_SC)(0,0)
    - (*this->primaryCentralSpacecraft.mDot_SC)(0,0)*(*this->primaryCentralSpacecraft.c_B)/(*this->primaryCentralSpacecraft.m_SC)(0,0)/(*this->primaryCentralSpacecraft.m_SC)(0,0);
    Eigen::Vector3d omegaLocal_BN_B = this->primaryCentralSpacecraft.hubOmega_BN_B->getState();
    Eigen::Vector3d cLocal_B = (*this->primaryCentralSpacecraft.c_B);
    (*this->primaryCentralSpacecraft.cDot_B) = (*this->primaryCentralSpacecraft.cPrime_B) + omegaLocal_BN_B.cross(cLocal_B);

    return;
}

/*! This method is solving Xdot = F(X,t) for the system. The hub needs to calculate its derivatives, along with all of 
 the stateEffectors. The hub also has gravity and dynamicEffectors acting on it and these relationships are controlled 
 in this method. At the end of this method all of the states will have their corresponding state derivatives set in the 
 dynParam Manager thus solving for Xdot*/
void SpacecraftDynamics::equationsOfMotion(double integTimeSeconds)
{
    // - Update time to the current time
    uint64_t integTimeNanos = this->simTimePrevious + (integTimeSeconds-this->timePrevious)/NANO2SEC;
    (*this->sysTime) << integTimeNanos, integTimeSeconds;

    // - Zero all Matrices and vectors for back-sub and the dynamics
    this->primaryCentralSpacecraft.hub.hubBackSubMatrices.matrixA.setZero();
    this->primaryCentralSpacecraft.hub.hubBackSubMatrices.matrixB.setZero();
    this->primaryCentralSpacecraft.hub.hubBackSubMatrices.matrixC.setZero();
    this->primaryCentralSpacecraft.hub.hubBackSubMatrices.matrixD.setZero();
    this->primaryCentralSpacecraft.hub.hubBackSubMatrices.vecTrans.setZero();
    this->primaryCentralSpacecraft.hub.hubBackSubMatrices.vecRot.setZero();
    this->primaryCentralSpacecraft.sumForceExternal_B.setZero();
    this->primaryCentralSpacecraft.sumForceExternal_N.setZero();
    this->primaryCentralSpacecraft.sumTorquePntB_B.setZero();

    // - Update the mass properties of the spacecraft
    this->updateSystemMassProps(integTimeSeconds);

    // - This is where gravity is computed (gravity needs to know c_B to calculated gravity about r_CN_N)
    Eigen::MRPd sigmaBNLoc;
    Eigen::Matrix3d dcm_NB;
    Eigen::Vector3d cLocal_N;

    sigmaBNLoc = (Eigen::Vector3d) this->primaryCentralSpacecraft.hubSigma->getState();
    dcm_NB = sigmaBNLoc.toRotationMatrix();
    cLocal_N = dcm_NB*(*this->primaryCentralSpacecraft.c_B);
    Eigen::Vector3d rLocal_CN_N = this->primaryCentralSpacecraft.hubR_N->getState() + dcm_NB*(*this->primaryCentralSpacecraft.c_B);
    Eigen::Vector3d vLocal_CN_N = this->primaryCentralSpacecraft.hubV_N->getState() + dcm_NB*(*this->primaryCentralSpacecraft.cDot_B);

    this->primaryCentralSpacecraft.gravField.computeGravityField(rLocal_CN_N, vLocal_CN_N);

    // - Loop through dynEffectors to compute force and torque on the s/c
    std::vector<DynamicEffector*>::iterator dynIt;
    for(dynIt = this->primaryCentralSpacecraft.dynEffectors.begin(); dynIt != this->primaryCentralSpacecraft.dynEffectors.end(); dynIt++)
    {
        // - Compute the force and torque contributions from the dynamicEffectors
        (*dynIt)->computeForceTorque(integTimeSeconds);
        this->primaryCentralSpacecraft.sumForceExternal_N += (*dynIt)->forceExternal_N;
        this->primaryCentralSpacecraft.sumForceExternal_B += (*dynIt)->forceExternal_B;
        this->primaryCentralSpacecraft.sumTorquePntB_B += (*dynIt)->torqueExternalPntB_B;
    }

    // - Loop through state effectors to get contributions for back-substitution
    std::vector<StateEffector*>::iterator it;
    for(it = this->primaryCentralSpacecraft.states.begin(); it != this->primaryCentralSpacecraft.states.end(); it++)
    {
        /* - Set the contribution matrices to zero (just in case a stateEffector += on the matrix or the stateEffector
         doesn't have a contribution for a matrix and doesn't set the matrix to zero */
        this->primaryCentralSpacecraft.backSubMatricesContributions.matrixA.setZero();
        this->primaryCentralSpacecraft.backSubMatricesContributions.matrixB.setZero();
        this->primaryCentralSpacecraft.backSubMatricesContributions.matrixC.setZero();
        this->primaryCentralSpacecraft.backSubMatricesContributions.matrixD.setZero();
        this->primaryCentralSpacecraft.backSubMatricesContributions.vecTrans.setZero();
        this->primaryCentralSpacecraft.backSubMatricesContributions.vecRot.setZero();

        // - Call the update contributions method for the stateEffectors and add in contributions to the hub matrices
        (*it)->updateContributions(integTimeSeconds, this->primaryCentralSpacecraft.backSubMatricesContributions);
        this->primaryCentralSpacecraft.hub.hubBackSubMatrices.matrixA += this->primaryCentralSpacecraft.backSubMatricesContributions.matrixA;
        this->primaryCentralSpacecraft.hub.hubBackSubMatrices.matrixB += this->primaryCentralSpacecraft.backSubMatricesContributions.matrixB;
        this->primaryCentralSpacecraft.hub.hubBackSubMatrices.matrixC += this->primaryCentralSpacecraft.backSubMatricesContributions.matrixC;
        this->primaryCentralSpacecraft.hub.hubBackSubMatrices.matrixD += this->primaryCentralSpacecraft.backSubMatricesContributions.matrixD;
        this->primaryCentralSpacecraft.hub.hubBackSubMatrices.vecTrans += this->primaryCentralSpacecraft.backSubMatricesContributions.vecTrans;
        this->primaryCentralSpacecraft.hub.hubBackSubMatrices.vecRot += this->primaryCentralSpacecraft.backSubMatricesContributions.vecRot;
    }

    // - Finish the math that is needed
    Eigen::Vector3d cLocal_B;
    Eigen::Vector3d cPrimeLocal_B;
    cLocal_B = *this->primaryCentralSpacecraft.c_B;
    cPrimeLocal_B = *this->primaryCentralSpacecraft.cPrime_B;

    Eigen::Matrix3d intermediateMatrix;
    Eigen::Vector3d intermediateVector;
    Eigen::Vector3d omegaLocalBN_B = this->primaryCentralSpacecraft.hubOmega_BN_B->getState();
    this->primaryCentralSpacecraft.hub.hubBackSubMatrices.matrixA += (*this->primaryCentralSpacecraft.m_SC)(0,0)*intermediateMatrix.Identity();
    intermediateMatrix = eigenTilde((*this->primaryCentralSpacecraft.c_B));  // make c_B skew symmetric matrix
    this->primaryCentralSpacecraft.hub.hubBackSubMatrices.matrixB += -(*this->primaryCentralSpacecraft.m_SC)(0,0)*intermediateMatrix;
    this->primaryCentralSpacecraft.hub.hubBackSubMatrices.matrixC += (*this->primaryCentralSpacecraft.m_SC)(0,0)*intermediateMatrix;
    this->primaryCentralSpacecraft.hub.hubBackSubMatrices.matrixD += *this->primaryCentralSpacecraft.ISCPntB_B;
    this->primaryCentralSpacecraft.hub.hubBackSubMatrices.vecTrans += -2.0*(*this->primaryCentralSpacecraft.m_SC)(0, 0)*omegaLocalBN_B.cross(cPrimeLocal_B)
    - (*this->primaryCentralSpacecraft.m_SC)(0, 0)*omegaLocalBN_B.cross(omegaLocalBN_B.cross(cLocal_B))
    - 2.0*(*this->primaryCentralSpacecraft.mDot_SC)(0,0)*(cPrimeLocal_B+omegaLocalBN_B.cross(cLocal_B));
    intermediateVector = *this->primaryCentralSpacecraft.ISCPntB_B*omegaLocalBN_B;
    this->primaryCentralSpacecraft.hub.hubBackSubMatrices.vecRot += -omegaLocalBN_B.cross(intermediateVector) - *this->primaryCentralSpacecraft.ISCPntBPrime_B*omegaLocalBN_B;

    // - Map external force_N to the body frame
    Eigen::Vector3d sumForceExternalMappedToB;
    sumForceExternalMappedToB = dcm_NB.transpose()*this->primaryCentralSpacecraft.sumForceExternal_N;

    // - Edit both v_trans and v_rot with gravity and external force and torque
    Eigen::Vector3d gLocal_N = *this->primaryCentralSpacecraft.g_N;

    // -  Make additional contributions to the matrices from the hub

    // - Need to find force of gravity on the spacecraft
    Eigen::Vector3d gravityForce_N;
    gravityForce_N = (*this->primaryCentralSpacecraft.m_SC)(0,0)*gLocal_N;

    Eigen::Vector3d gravityForce_B;
    gravityForce_B = dcm_NB.transpose()*gravityForce_N;
    this->primaryCentralSpacecraft.hub.hubBackSubMatrices.vecTrans += gravityForce_B + sumForceExternalMappedToB + this->primaryCentralSpacecraft.sumForceExternal_B;
    this->primaryCentralSpacecraft.hub.hubBackSubMatrices.vecRot += cLocal_B.cross(gravityForce_B) + this->primaryCentralSpacecraft.sumTorquePntB_B;

    // - Compute the derivatives of the hub states before looping through stateEffectors
    this->primaryCentralSpacecraft.hub.computeDerivatives(integTimeSeconds);

    // - Loop through state effectors for compute derivatives
    for(it = this->primaryCentralSpacecraft.states.begin(); it != this->primaryCentralSpacecraft.states.end(); it++)
    {
        (*it)->computeDerivatives(integTimeSeconds);
    }

    return;
}

/*! This method is used to integrate the state forward in time, switch MRPs, calculate energy and momentum, and 
 calculate the accumulated deltaV */
void SpacecraftDynamics::integrateState(double integrateToThisTime)
{
    // - Find the time step
    double localTimeStep = integrateToThisTime - timePrevious;

    // - Find v_CN_N before integration for accumulated DV
    Eigen::Vector3d oldV_BN_N = this->primaryCentralSpacecraft.hubV_N->getState();  // - V_BN_N before integration
    Eigen::Vector3d oldV_CN_N;  // - V_CN_N before integration
    Eigen::Vector3d oldC_B;     // - Center of mass offset before integration
    Eigen::Vector3d oldOmega_BN_B;  // - angular rate of B wrt N in the Body frame
    Eigen::MRPd oldSigma_BN;    // - Sigma_BN before integration
    // - Get the angular rate, oldOmega_BN_B from the dyn manager
    oldOmega_BN_B = this->primaryCentralSpacecraft.hubOmega_BN_B->getState();
    // - Get center of mass, v_BN_N and dcm_NB from the dyn manager
    oldSigma_BN = (Eigen::Vector3d) this->primaryCentralSpacecraft.hubSigma->getState();
    // - Finally find v_CN_N
    Eigen::Matrix3d oldDcm_NB = oldSigma_BN.toRotationMatrix(); // - dcm_NB before integration
    oldV_CN_N = oldV_BN_N + oldDcm_NB*(*this->primaryCentralSpacecraft.cDot_B);


    // - Integrate the state from the last time (timeBefore) to the integrateToThisTime
    double timeBefore = integrateToThisTime - localTimeStep;
    this->integrator->integrate(timeBefore, localTimeStep);
    this->timePrevious = integrateToThisTime;     // - copy the current time into previous time for next integrate state call

    // - Call hubs modify states to allow for switching of MRPs
    this->primaryCentralSpacecraft.hub.modifyStates(integrateToThisTime);

    // - Loop over stateEffectors to call modifyStates
    std::vector<StateEffector*>::iterator it;
    for(it = this->primaryCentralSpacecraft.states.begin(); it != this->primaryCentralSpacecraft.states.end(); it++)
    {
        // - Call energy and momentum calulations for stateEffectors
        (*it)->modifyStates(integrateToThisTime);
    }

    // - Call mass properties to get current info on the mass props of the spacecraft
    this->updateSystemMassProps(integrateToThisTime);

    // - Find v_CN_N after the integration for accumulated DV
    Eigen::Vector3d newV_BN_N = this->primaryCentralSpacecraft.hubV_N->getState(); // - V_BN_N after integration
    Eigen::Vector3d newV_CN_N;  // - V_CN_N after integration
    Eigen::MRPd newSigma_BN;    // - Sigma_BN after integration
    // - Get center of mass, v_BN_N and dcm_NB
    Eigen::Vector3d sigmaBNLoc;
    sigmaBNLoc = (Eigen::Vector3d) this->primaryCentralSpacecraft.hubSigma->getState();
    newSigma_BN = sigmaBNLoc;
    Eigen::Matrix3d newDcm_NB = newSigma_BN.toRotationMatrix();  // - dcm_NB after integration
    newV_CN_N = newV_BN_N + newDcm_NB*(*this->primaryCentralSpacecraft.cDot_B);

    // - Find change in velocity
    Eigen::Vector3d dV_N; // dV of the center of mass in the intertial frame
    Eigen::Vector3d dV_B_N; //dV of the body frame in the inertial frame
    Eigen::Vector3d dV_B_B; //dV of the body frame in the body frame
    dV_N = newV_CN_N - oldV_CN_N;
    dV_B_N = newV_BN_N - oldV_BN_N;
    // - Subtract out gravity
    Eigen::Vector3d gLocal_N;
    gLocal_N = *this->primaryCentralSpacecraft.g_N;
    dV_N -= gLocal_N*localTimeStep;
    dV_B_N -= gLocal_N*localTimeStep;
    dV_B_B = newDcm_NB.transpose()*dV_B_N;

    // - Find accumulated DV of the center of mass in the body frame
    this->primaryCentralSpacecraft.dvAccum_B += newDcm_NB.transpose()*dV_N;

    // - Find the accumulated DV of the body frame in the body frame
    this->primaryCentralSpacecraft.dvAccum_BN_B += dV_B_B;

    // - non-conservative acceleration of the body frame in the body frame
    this->primaryCentralSpacecraft.nonConservativeAccelpntB_B = dV_B_B/localTimeStep;

    // - angular acceleration in the body frame
    Eigen::Vector3d newOmega_BN_B;
    newOmega_BN_B = this->primaryCentralSpacecraft.hubOmega_BN_B->getState();
    this->primaryCentralSpacecraft.omegaDot_BN_B = (newOmega_BN_B - oldOmega_BN_B)/localTimeStep; //angular acceleration of B wrt N in the Body frame

    // - Compute Energy and Momentum
    this->computeEnergyMomentum(integrateToThisTime);

    return;
}

/*! This method is used to find the total energy and momentum of the spacecraft. It finds the total orbital energy,
 total orbital angular momentum, total rotational energy and total rotational angular momentum. These values are used 
 for validation purposes. */
void SpacecraftDynamics::computeEnergyMomentum(double time)
{
    // - Grab values from state Manager
    Eigen::Vector3d rLocal_BN_N = this->primaryCentralSpacecraft.hubR_N->getState();
    Eigen::Vector3d rDotLocal_BN_N = this->primaryCentralSpacecraft.hubV_N->getState();
    Eigen::MRPd sigmaLocal_BN;
    sigmaLocal_BN = (Eigen::Vector3d ) this->primaryCentralSpacecraft.hubSigma->getState();

    // - Find DCM's
    Eigen::Matrix3d dcmLocal_NB = sigmaLocal_BN.toRotationMatrix();
    Eigen::Matrix3d dcmLocal_BN = dcmLocal_NB.transpose();

    // - Convert from inertial frame to body frame
    Eigen::Vector3d rBNLocal_B;
    Eigen::Vector3d rDotBNLocal_B;
    rBNLocal_B = dcmLocal_BN*rLocal_BN_N;
    rDotBNLocal_B = dcmLocal_BN*rDotLocal_BN_N;

    // - zero necessarry variables
    Eigen::Vector3d totOrbAngMomPntN_B;
    Eigen::Vector3d totRotAngMomPntC_B;
    totOrbAngMomPntN_B.setZero();
    totRotAngMomPntC_B.setZero();
    this->primaryCentralSpacecraft.totOrbAngMomPntN_N.setZero();
    this->primaryCentralSpacecraft.totRotAngMomPntC_N.setZero();
    this->primaryCentralSpacecraft.rotAngMomPntCContr_B.setZero();
    this->primaryCentralSpacecraft.totOrbEnergy = 0.0;
    this->primaryCentralSpacecraft.totRotEnergy = 0.0;
    this->primaryCentralSpacecraft.rotEnergyContr = 0.0;

    // - Get the hubs contribution
    this->primaryCentralSpacecraft.hub.updateEnergyMomContributions(time, this->primaryCentralSpacecraft.rotAngMomPntCContr_B, this->primaryCentralSpacecraft.rotEnergyContr);
    totRotAngMomPntC_B += this->primaryCentralSpacecraft.rotAngMomPntCContr_B;
    this->primaryCentralSpacecraft.totRotEnergy += this->primaryCentralSpacecraft.rotEnergyContr;

    // - Loop over stateEffectors to get their contributions to energy and momentum
    std::vector<StateEffector*>::iterator it;
    for(it = this->primaryCentralSpacecraft.states.begin(); it != this->primaryCentralSpacecraft.states.end(); it++)
    {
        // - Set the matrices to zero
        this->primaryCentralSpacecraft.rotAngMomPntCContr_B.setZero();
        this->primaryCentralSpacecraft.rotEnergyContr = 0.0;

        // - Call energy and momentum calulations for stateEffectors
        (*it)->updateEnergyMomContributions(time, this->primaryCentralSpacecraft.rotAngMomPntCContr_B, this->primaryCentralSpacecraft.rotEnergyContr);
        totRotAngMomPntC_B += this->primaryCentralSpacecraft.rotAngMomPntCContr_B;
        this->primaryCentralSpacecraft.totRotEnergy += this->primaryCentralSpacecraft.rotEnergyContr;
    }

    // - Get cDot_B from manager
    Eigen::Vector3d cDotLocal_B = (*this->primaryCentralSpacecraft.cDot_B);

    // - Add in orbital kinetic energy into the total orbital energy calculations
    this->primaryCentralSpacecraft.totOrbEnergy += 1.0/2.0*(*this->primaryCentralSpacecraft.m_SC)(0,0)*(rDotBNLocal_B.dot(rDotBNLocal_B) + 2.0*rDotBNLocal_B.dot(cDotLocal_B)
                                                      + cDotLocal_B.dot(cDotLocal_B));

    // - Call gravity effector and add in its potential contributions to the total orbital energy calculations
    this->primaryCentralSpacecraft.orbPotentialEnergyContr = 0.0;
    Eigen::Vector3d rLocal_CN_N = this->primaryCentralSpacecraft.hubR_N->getState() + dcmLocal_NB*(*this->primaryCentralSpacecraft.c_B);
    this->primaryCentralSpacecraft.gravField.updateEnergyContributions(rLocal_CN_N, this->primaryCentralSpacecraft.orbPotentialEnergyContr);
    this->primaryCentralSpacecraft.totOrbEnergy += (*this->primaryCentralSpacecraft.m_SC)(0,0)*this->primaryCentralSpacecraft.orbPotentialEnergyContr;

    // - Find total rotational energy
    this->primaryCentralSpacecraft.totRotEnergy += -1.0/2.0*(*this->primaryCentralSpacecraft.m_SC)(0,0)*cDotLocal_B.dot(cDotLocal_B);

    // - Find orbital angular momentum for the spacecraft
    Eigen::Vector3d rCN_N;
    Eigen::Vector3d rDotCN_N;
    rCN_N = rLocal_BN_N + dcmLocal_NB*(*this->primaryCentralSpacecraft.c_B);
    rDotCN_N = rDotLocal_BN_N + dcmLocal_NB*cDotLocal_B;
    this->primaryCentralSpacecraft.totOrbAngMomPntN_N = (*this->primaryCentralSpacecraft.m_SC)(0,0)*(rCN_N.cross(rDotCN_N));

    // - Find rotational angular momentum for the spacecraft
    totRotAngMomPntC_B += -(*this->primaryCentralSpacecraft.m_SC)(0,0)*(Eigen::Vector3d (*this->primaryCentralSpacecraft.c_B)).cross(cDotLocal_B);
    this->primaryCentralSpacecraft.totRotAngMomPntC_N = dcmLocal_NB*totRotAngMomPntC_B;

    return;
}
