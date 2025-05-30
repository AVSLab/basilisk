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

#include "spacecraftSystem.h"
#include "architecture/utilities/macroDefinitions.h"
#include "../_GeneralModuleFiles/svIntegratorRK4.h"
#include "architecture/utilities/avsEigenSupport.h"
#include "architecture/utilities/avsEigenMRP.h"
#include "../../../architecture/utilities/rigidBodyKinematics.h"
#include <iostream>

SpacecraftUnit::SpacecraftUnit()
{
    // - Set default names
    this->spacecraftName = "spacecraft";

    // - Set values to either zero or default values
    this->dvAccum_CN_B.setZero();
    this->dvAccum_BN_B.setZero();

    return;
}

SpacecraftUnit::~SpacecraftUnit()
{
    return;
}

void SpacecraftUnit::addStateEffector(StateEffector *newStateEffector)
{
    this->assignStateParamNames<StateEffector *>(newStateEffector);

    this->states.push_back(newStateEffector);

    // Give the stateEffector the name of the spacecraft it is attached to
    newStateEffector->nameOfSpacecraftAttachedTo = this->spacecraftName;
}

void SpacecraftUnit::addDynamicEffector(DynamicEffector *newDynamicEffector)
{
    this->assignStateParamNames<DynamicEffector *>(newDynamicEffector);

    this->dynEffectors.push_back(newDynamicEffector);
}

void SpacecraftUnit::addDockingPort(DockingData *newDockingPort)
{
    this->dockingPoints.push_back(newDockingPort);

    return;
}

void SpacecraftUnit::SelfInitSC(int64_t moduleID)
{
}


/*! This method is used to reset the module.

 */
void SpacecraftUnit::ResetSC(uint64_t CurrentSimNanos)
{
    this->gravField.Reset(CurrentSimNanos);
}

void SpacecraftUnit::writeOutputMessagesSC(uint64_t clockTime, int64_t moduleID)
{
    // - Write output messages for each spacecraft
    // - Populate state output message
    SCStatesMsgPayload stateOut;
    stateOut = this->scStateOutMsg.zeroMsgPayload;
    eigenMatrixXd2CArray(*this->inertialPositionProperty, stateOut.r_BN_N);
    eigenMatrixXd2CArray(*this->inertialVelocityProperty, stateOut.v_BN_N);
    Eigen::MRPd sigmaLocal_BN;
    sigmaLocal_BN = (Eigen::Vector3d) this->hubSigma->getState();
    Eigen::Matrix3d dcm_NB = sigmaLocal_BN.toRotationMatrix();
    Eigen::Vector3d rLocal_CN_N = (*this->inertialPositionProperty) + dcm_NB*(*this->c_B);
    Eigen::Vector3d vLocal_CN_N = (*this->inertialVelocityProperty) + dcm_NB*(*this->cDot_B);
    eigenVector3d2CArray(rLocal_CN_N, stateOut.r_CN_N);
    eigenVector3d2CArray(vLocal_CN_N, stateOut.v_CN_N);
    eigenMatrixXd2CArray(this->hubSigma->getState(), stateOut.sigma_BN);
    eigenMatrixXd2CArray(this->hubOmega_BN_B->getState(), stateOut.omega_BN_B);
    eigenMatrixXd2CArray(this->dvAccum_CN_B, stateOut.TotalAccumDVBdy);
    stateOut.MRPSwitchCount = this->hub.MRPSwitchCount;
    eigenMatrixXd2CArray(this->dvAccum_BN_B, stateOut.TotalAccumDV_BN_B);
    eigenVector3d2CArray(this->nonConservativeAccelpntB_B, stateOut.nonConservativeAccelpntB_B);
    eigenVector3d2CArray(this->omegaDot_BN_B, stateOut.omegaDot_BN_B);
    this->scStateOutMsg.write(&stateOut, moduleID, clockTime);

    // - Populate mass state output message
    SCMassPropsMsgPayload massStateOut;
    massStateOut = this->scMassStateOutMsg.zeroMsgPayload;
    massStateOut.massSC = (*this->m_SC)(0,0);
    eigenMatrixXd2CArray(*this->c_B, massStateOut.c_B);
    eigenMatrixXd2CArray(*this->ISCPntB_B, (double *)massStateOut.ISC_PntB_B);
    this->scMassStateOutMsg.write(&massStateOut, moduleID, clockTime);

    // - Populate energy momentum output message
    SCEnergyMomentumMsgPayload energyMomentumOut;
    energyMomentumOut = this->scEnergyMomentumOutMsg.zeroMsgPayload;
    energyMomentumOut.spacecraftRotEnergy = this->totRotEnergy;
    energyMomentumOut.spacecraftOrbEnergy = this->totOrbEnergy;
    eigenVector3d2CArray(this->totRotAngMomPntC_N, energyMomentumOut.spacecraftRotAngMomPntC_N);
    eigenVector3d2CArray(this->totOrbAngMomPntN_N, energyMomentumOut.spacecraftOrbAngMomPntN_N);
    this->scEnergyMomentumOutMsg.write(&energyMomentumOut, moduleID, clockTime);
    return;
}
void SpacecraftUnit::linkInStatesSC(DynParamManager& statesIn)
{
    // - Get access to all spacecraft hub states
    this->hubR_N = statesIn.getStateObject(this->hub.nameOfHubPosition);
    this->hubV_N = statesIn.getStateObject(this->hub.nameOfHubVelocity);
    this->hubSigma = statesIn.getStateObject(this->hub.nameOfHubSigma);   /* Need sigmaBN for MRP switching */
    this->hubOmega_BN_B = statesIn.getStateObject(this->hub.nameOfHubOmega);
    this->hubGravVelocity = statesIn.getStateObject(this->hub.nameOfHubGravVelocity);
    this->BcGravVelocity = statesIn.getStateObject(this->hub.nameOfBcGravVelocity);

    // - Get access to the hubs position and velocity in the property manager
    this->inertialPositionProperty = statesIn.getPropertyReference(this->gravField.inertialPositionPropName);
    this->inertialVelocityProperty = statesIn.getPropertyReference(this->gravField.inertialVelocityPropName);
    this->g_N = statesIn.getPropertyReference(this->gravField.vehicleGravityPropName);

    return;
}

void SpacecraftUnit::initializeDynamicsSC(DynParamManager& statesIn)
{
    // - Spacecraft() initiates all of the spaceCraft mass properties
    Eigen::MatrixXd initM_SC(1,1);
    Eigen::MatrixXd initMDot_SC(1,1);
    Eigen::MatrixXd initC_B(3,1);
    Eigen::MatrixXd initISCPntB_B(3,3);
    Eigen::MatrixXd initCPrime_B(3,1);
    Eigen::MatrixXd initCDot_B(3,1);
    Eigen::MatrixXd initISCPntBPrime_B(3,3);

    // - Create the properties
    this->propName_m_SC = this->spacecraftName + "m_SC";
    this->m_SC = statesIn.createProperty(this->propName_m_SC, initM_SC);
    this->propName_mDot_SC = this->spacecraftName + "mDot_SC";
    this->mDot_SC = statesIn.createProperty(this->propName_mDot_SC, initMDot_SC);
    this->propName_centerOfMassSC = this->spacecraftName + "centerOfMassSC";
    this->c_B = statesIn.createProperty(this->propName_centerOfMassSC, initC_B);
    this->propName_inertiaSC = this->spacecraftName + "inertiaSC";
    this->ISCPntB_B = statesIn.createProperty(this->propName_inertiaSC, initISCPntB_B);
    this->propName_inertiaPrimeSC = this->spacecraftName + "inertiaPrimeSC";
    this->ISCPntBPrime_B = statesIn.createProperty(this->propName_inertiaPrimeSC, initISCPntBPrime_B);
    this->propName_centerOfMassPrimeSC = this->spacecraftName + "centerOfMassPrimeSC";
    this->cPrime_B = statesIn.createProperty(this->propName_centerOfMassPrimeSC, initCPrime_B);
    this->propName_centerOfMassDotSC = this->spacecraftName + "centerOfMassDotSC";
    this->cDot_B = statesIn.createProperty(this->propName_centerOfMassDotSC, initCDot_B);

    // - Give name of all spacecraft to attached hubEffector
    this->hub.nameOfSpacecraftAttachedTo = this->spacecraftName;
    // - Give name of all spacecraft to attached hubEffector
    this->gravField.nameOfSpacecraftAttachedTo = this->spacecraftName;

    // - Before er'body registers their properties, we need to prepend their state names with the spacecraft
    this->hub.prependSpacecraftNameToStates();
    this->gravField.prependSpacecraftNameToStates();
    std::vector<StateEffector*>::iterator stateIt;
    for(stateIt = this->states.begin(); stateIt != this->states.end(); stateIt++)
    {
        (*stateIt)->prependSpacecraftNameToStates();
    }

    // - Register the gravity properties with the dynManager, 'erbody wants g_N!
    this->gravField.registerProperties(statesIn);

    // - Register the hub states
    this->hub.registerStates(statesIn);

    // - Loop through stateEffectors to register their states
    for(stateIt = this->states.begin(); stateIt != this->states.end(); stateIt++)
    {
        (*stateIt)->registerStates(statesIn);
    }

    // - Link in states for the spacecraft, gravity and the hub
    this->linkInStatesSC(statesIn);
    this->gravField.linkInStates(statesIn);
    this->hub.linkInStates(statesIn);

    // - Loop through the stateEffectors to link in the states needed
    for(stateIt = this->states.begin(); stateIt != this->states.end(); stateIt++)
    {
        (*stateIt)->linkInStates(statesIn);
    }

    // - Loop through the dynamicEffectors to link in the states needed
    std::vector<DynamicEffector*>::iterator dynIt;
    for(dynIt = this->dynEffectors.begin(); dynIt != this->dynEffectors.end(); dynIt++)
    {
        (*dynIt)->linkInStates(statesIn);
    }

    return;
}

/*! This is the constructor, setting variables to default values */
SpacecraftSystem::SpacecraftSystem()
{
    // - Set default names
    this->sysTimePropertyName = "systemTime";

    // - Set values to either zero or default values
    this->numOutMsgBuffers = 2;

    // - Set integrator as RK4 by default
    this->integrator = new svIntegratorRK4(this);
    this->numberOfSCAttachedToPrimary = 0;

    return;
}

/*! This is the destructor, nothing to report here */
SpacecraftSystem::~SpacecraftSystem()
{
    return;
}

/*! This method adds a spacecraft to the simulation as a free floating spacecraft */
void SpacecraftSystem::addSpacecraftUndocked(SpacecraftUnit *newSpacecraft)
{
    this->unDockedSpacecraft.push_back(newSpacecraft);

    return;
}

/*! This method attaches a spacecraft to the chain of spacecraft attached to the primary spacecraft */
void SpacecraftSystem::attachSpacecraftToPrimary(SpacecraftUnit *newSpacecraft, std::string dockingPortNameOfNewSpacecraft, std::string dockingToPortName)
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
                    break;
                } else {
                    std::cerr << __FILE__ <<": Port name given is not the same port name held in the newSpacecraft";
                    std::cerr << "  Quitting."<<std::endl;
                }

            }
        }
    }

    if (checkDock < 1) {
        std::vector<SpacecraftUnit*>::iterator spacecraftConnectedIt;
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
                            break;
                        } else {
                            std::cerr << __FILE__ <<": Port name given is not the same port name held in the newSpacecraft";
                            std::cerr << "  Quitting."<<std::endl;
                        }

                    }
                }

            }

        }
    }

    if (checkDock != 1) {
        std::cerr << __FILE__ <<": The new spacecraft did not get attached due to naming problems with the ports";
        std::cerr << "  Quitting."<<std::endl;
    } else {
        this->numberOfSCAttachedToPrimary += 1;
        newSpacecraft->docked = true;
        this->primaryCentralSpacecraft.docked = true;
    }

    this->spacecraftDockedToPrimary.push_back(newSpacecraft);

    return;
}


/*! This method is used to reset the module.

 */
void SpacecraftSystem::Reset(uint64_t CurrentSimNanos)
{
    this->primaryCentralSpacecraft.ResetSC(CurrentSimNanos);

    // - Call this for all of the connected spacecraft
    std::vector<SpacecraftUnit*>::iterator spacecraftConnectedIt;
    for(spacecraftConnectedIt = this->spacecraftDockedToPrimary.begin(); spacecraftConnectedIt != this->spacecraftDockedToPrimary.end(); spacecraftConnectedIt++)
    {
        (*spacecraftConnectedIt)->ResetSC(CurrentSimNanos);
    }

    // - Call this for all of the unconnected spacecraft
    std::vector<SpacecraftUnit*>::iterator spacecraftUnConnectedIt;
    for(spacecraftUnConnectedIt = this->unDockedSpacecraft.begin(); spacecraftUnConnectedIt != this->unDockedSpacecraft.end(); spacecraftUnConnectedIt++)
    {
        (*spacecraftUnConnectedIt)->ResetSC(CurrentSimNanos);
    }

    // - Call method for initializing the dynamics of spacecraft
    this->initializeDynamics();

    this->timeBefore = CurrentSimNanos * NANO2SEC;
    this->timeBeforeNanos = CurrentSimNanos;
}

/*! This is the method where the messages of the state of vehicle are written */
void SpacecraftSystem::writeOutputMessages(uint64_t clockTime)
{
    // - Call writeOutputMessages for primary spacecraft
    this->primaryCentralSpacecraft.writeOutputMessagesSC(clockTime, this->moduleID);

    // - Call this for all of the connected spacecraft
    std::vector<SpacecraftUnit*>::iterator spacecraftConnectedIt;
    for(spacecraftConnectedIt = this->spacecraftDockedToPrimary.begin(); spacecraftConnectedIt != this->spacecraftDockedToPrimary.end(); spacecraftConnectedIt++)
    {
        (*spacecraftConnectedIt)->writeOutputMessagesSC(clockTime, this->moduleID);
    }

    // - Call this for all of the unconnected spacecraft
    std::vector<SpacecraftUnit*>::iterator spacecraftUnConnectedIt;
    for(spacecraftUnConnectedIt = this->unDockedSpacecraft.begin(); spacecraftUnConnectedIt != this->unDockedSpacecraft.end(); spacecraftUnConnectedIt++)
    {
        (*spacecraftUnConnectedIt)->writeOutputMessagesSC(clockTime, this->moduleID);
    }

    return;
}

/*! This method is a part of sysModel and is used to integrate the state and update the state in the messaging system */
void SpacecraftSystem::UpdateState(uint64_t CurrentSimNanos)
{

    // - Get access to the spice bodies
    this->primaryCentralSpacecraft.gravField.UpdateState(CurrentSimNanos);

    // - Call this for all of the connected spacecraft
    std::vector<SpacecraftUnit*>::iterator spacecraftConnectedIt;
    for(spacecraftConnectedIt = this->spacecraftDockedToPrimary.begin(); spacecraftConnectedIt != this->spacecraftDockedToPrimary.end(); spacecraftConnectedIt++)
    {
        (*spacecraftConnectedIt)->gravField.UpdateState(CurrentSimNanos);
    }

    // - Call this for all of the unconnected spacecraft
    std::vector<SpacecraftUnit*>::iterator spacecraftUnConnectedIt;
    for(spacecraftUnConnectedIt = this->unDockedSpacecraft.begin(); spacecraftUnConnectedIt != this->unDockedSpacecraft.end(); spacecraftUnConnectedIt++)
    {
        (*spacecraftUnConnectedIt)->gravField.UpdateState(CurrentSimNanos);
    }

    // - Integrate the state forward in time
    this->integrateState(CurrentSimNanos);

    // - Update the inertial position of each spacecraft
    Eigen::Vector3d rLocal_BN_N = this->primaryCentralSpacecraft.hubR_N->getState();
    Eigen::Vector3d vLocal_BN_N = this->primaryCentralSpacecraft.hubV_N->getState();
    this->primaryCentralSpacecraft.gravField.updateInertialPosAndVel(rLocal_BN_N, vLocal_BN_N);

    // - Same thing for all of the connected spacecraft
    for(spacecraftConnectedIt = this->spacecraftDockedToPrimary.begin(); spacecraftConnectedIt != this->spacecraftDockedToPrimary.end(); spacecraftConnectedIt++)
    {
        rLocal_BN_N = (*spacecraftConnectedIt)->hubR_N->getState();
        vLocal_BN_N = (*spacecraftConnectedIt)->hubV_N->getState();
        (*spacecraftConnectedIt)->gravField.updateInertialPosAndVel(rLocal_BN_N, vLocal_BN_N);
    }

    // - Same thing for all of the connected spacecraft
    for(spacecraftUnConnectedIt = this->unDockedSpacecraft.begin(); spacecraftUnConnectedIt != this->unDockedSpacecraft.end(); spacecraftUnConnectedIt++)
    {
        rLocal_BN_N = (*spacecraftUnConnectedIt)->hubR_N->getState();
        vLocal_BN_N = (*spacecraftUnConnectedIt)->hubV_N->getState();
        (*spacecraftUnConnectedIt)->gravField.updateInertialPosAndVel(rLocal_BN_N, vLocal_BN_N);
    }

    // - Write the state of the vehicle into messages
    this->writeOutputMessages(CurrentSimNanos);

    return;
}

/*! This method is used to initialize the simulation by registering all of the states, linking the dynamicEffectors,
 stateEffectors, and the hub, initialize gravity, and initialize the sim with the initial conditions specified in python
 for the simulation */
void SpacecraftSystem::initializeDynamics()
{
    Eigen::MatrixXd systemTime(2,1);
    systemTime.setZero();
    this->sysTime = this->dynManager.createProperty(this->sysTimePropertyName, systemTime);

    // - Call initializeDynamics for primary spacecraft
    this->primaryCentralSpacecraft.initializeDynamicsSC(this->dynManager);

    // - Call this for all of the connected spacecraft
    std::vector<SpacecraftUnit*>::iterator spacecraftConnectedIt;
    for(spacecraftConnectedIt = this->spacecraftDockedToPrimary.begin(); spacecraftConnectedIt != this->spacecraftDockedToPrimary.end(); spacecraftConnectedIt++)
    {
        (*spacecraftConnectedIt)->initializeDynamicsSC(this->dynManager);
    }

    // - Call this for all of the unconnected spacecraft
    std::vector<SpacecraftUnit*>::iterator spacecraftUnConnectedIt;
    for(spacecraftUnConnectedIt = this->unDockedSpacecraft.begin(); spacecraftUnConnectedIt != this->unDockedSpacecraft.end(); spacecraftUnConnectedIt++)
    {
        (*spacecraftUnConnectedIt)->initializeDynamicsSC(this->dynManager);
    }

    // - Update the mass properties of the spacecraft to retrieve c_B and cDot_B to update r_BN_N and v_BN_N
    this->updateSystemMassProps(0.0);

    // - Call mass props for all the rest of the spacecraft
    for(spacecraftUnConnectedIt = this->unDockedSpacecraft.begin(); spacecraftUnConnectedIt != this->unDockedSpacecraft.end(); spacecraftUnConnectedIt++)
    {
        this->updateSpacecraftMassProps(0.0, (*(*spacecraftUnConnectedIt)));
    }

    // - Edit r_BN_N and v_BN_N to take into account that point B and point C are not coincident
    // - Pulling the state from the hub at this time gives us r_CN_N
    this->initializeSCPosVelocity(this->primaryCentralSpacecraft);
    // - determine the initial hub states for spacecraft connected to the primary spacecraft
    this->determineAttachedSCStates();

    // - Initialize this for all other spacecraft
    // - Call mass props for all the rest of the spacecraft
    for(spacecraftUnConnectedIt = this->unDockedSpacecraft.begin(); spacecraftUnConnectedIt != this->unDockedSpacecraft.end(); spacecraftUnConnectedIt++)
    {
        this->initializeSCPosVelocity((*(*spacecraftUnConnectedIt)));
    }

    // - Call all stateEffectors in each spacecraft to give them body frame information
    std::vector<SpacecraftUnit*>::iterator spacecraftIt;
    for(spacecraftIt = this->spacecraftDockedToPrimary.begin(); spacecraftIt != this->spacecraftDockedToPrimary.end(); spacecraftIt++)
    {
        std::vector<StateEffector*>::iterator stateIt;
        for(stateIt = (*spacecraftIt)->states.begin(); stateIt != (*spacecraftIt)->states.end(); stateIt++)
        {
            (*stateIt)->receiveMotherSpacecraftData((*spacecraftIt)->hub.r_BP_P, (*spacecraftIt)->hub.dcm_BP);
        }
    }

    // - Call equations of motion at time zero
    this->equationsOfMotion(0.0, 1.0);

    return;
}

/*! This method is used to update the mass properties of the entire spacecraft using contributions from stateEffectors */
void SpacecraftSystem::updateSpacecraftMassProps(double time, SpacecraftUnit& spacecraft)
{
    // - Zero the properties which will get populated in this method
    (*spacecraft.m_SC).setZero();
    (*spacecraft.mDot_SC).setZero();
    (*spacecraft.c_B).setZero();
    (*spacecraft.ISCPntB_B).setZero();
    (*spacecraft.cPrime_B).setZero();
    (*spacecraft.cDot_B).setZero();
    (*spacecraft.ISCPntBPrime_B).setZero();

    // Add in hubs mass props to the spacecraft mass props
    spacecraft.hub.updateEffectorMassProps(time);
    (*spacecraft.m_SC)(0,0) += spacecraft.hub.effProps.mEff;
    (*spacecraft.ISCPntB_B) += spacecraft.hub.effProps.IEffPntB_B;
    (*spacecraft.c_B) += spacecraft.hub.effProps.mEff*spacecraft.hub.effProps.rEff_CB_B;

    // - Loop through state effectors to get mass props
    std::vector<StateEffector*>::iterator it;
    for(it = spacecraft.states.begin(); it != spacecraft.states.end(); it++)
    {
        (*it)->updateEffectorMassProps(time);
        // - Add in effectors mass props into mass props of spacecraft
        (*spacecraft.m_SC)(0,0) += (*it)->effProps.mEff;
        (*spacecraft.mDot_SC)(0,0) += (*it)->effProps.mEffDot;
        (*spacecraft.ISCPntB_B) += (*it)->effProps.IEffPntB_B;
        (*spacecraft.c_B) += (*it)->effProps.mEff*(*it)->effProps.rEff_CB_B;
        (*spacecraft.ISCPntBPrime_B) += (*it)->effProps.IEffPrimePntB_B;
        (*spacecraft.cPrime_B) += (*it)->effProps.mEff*(*it)->effProps.rEffPrime_CB_B;
        // For high fidelity mass depletion, this is left out: += (*it)->effProps.mEffDot*(*it)->effProps.rEff_CB_B
    }

    // Divide c_B and cPrime_B by the total mass of the spaceCraft to finalize c_B and cPrime_B
    (*spacecraft.c_B) = (*spacecraft.c_B)/(*spacecraft.m_SC)(0,0);
    (*spacecraft.cPrime_B) = (*spacecraft.cPrime_B)/(*spacecraft.m_SC)(0,0)
    - (*spacecraft.mDot_SC)(0,0)*(*spacecraft.c_B)/(*spacecraft.m_SC)(0,0)/(*spacecraft.m_SC)(0,0);
    Eigen::Vector3d omegaLocal_BN_B = spacecraft.hubOmega_BN_B->getState();
    Eigen::Vector3d cLocal_B = (*spacecraft.c_B);
    (*spacecraft.cDot_B) = (*spacecraft.cPrime_B) + omegaLocal_BN_B.cross(cLocal_B);

    return;
}

void SpacecraftSystem::updateSystemMassProps(double time)
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
        // - Add in effectors mass props into mass props of this->primaryCentralSpacecraft
        (*this->primaryCentralSpacecraft.m_SC)(0,0) += (*it)->effProps.mEff;
        (*this->primaryCentralSpacecraft.mDot_SC)(0,0) += (*it)->effProps.mEffDot;
        (*this->primaryCentralSpacecraft.ISCPntB_B) += (*it)->effProps.IEffPntB_B;
        (*this->primaryCentralSpacecraft.c_B) += (*it)->effProps.mEff*(*it)->effProps.rEff_CB_B;
        (*this->primaryCentralSpacecraft.ISCPntBPrime_B) += (*it)->effProps.IEffPrimePntB_B;
        (*this->primaryCentralSpacecraft.cPrime_B) += (*it)->effProps.mEff*(*it)->effProps.rEffPrime_CB_B;
        // For high fidelity mass depletion, this is left out: += (*it)->effProps.mEffDot*(*it)->effProps.rEff_CB_B
    }

    // - Call this for all of the connected spacecraft
    std::vector<SpacecraftUnit*>::iterator spacecraftConnectedIt;
    for(spacecraftConnectedIt = this->spacecraftDockedToPrimary.begin(); spacecraftConnectedIt != this->spacecraftDockedToPrimary.end(); spacecraftConnectedIt++)
    {
        // Add in hubs mass props to the spacecraft mass props
        (*spacecraftConnectedIt)->hub.updateEffectorMassProps(time);
        (*this->primaryCentralSpacecraft.m_SC)(0,0) += (*spacecraftConnectedIt)->hub.effProps.mEff;
        (*this->primaryCentralSpacecraft.ISCPntB_B) += (*spacecraftConnectedIt)->hub.effProps.IEffPntB_B;
        (*this->primaryCentralSpacecraft.c_B) += (*spacecraftConnectedIt)->hub.effProps.mEff*(*spacecraftConnectedIt)->hub.effProps.rEff_CB_B;

        // - Loop through state effectors to get mass props
        std::vector<StateEffector*>::iterator it;
        for(it = (*spacecraftConnectedIt)->states.begin(); it != (*spacecraftConnectedIt)->states.end(); it++)
        {
            (*it)->updateEffectorMassProps(time);
            // - Add in effectors mass props into mass props of this->primaryCentralSpacecraft
            (*this->primaryCentralSpacecraft.m_SC)(0,0) += (*it)->effProps.mEff;
            (*this->primaryCentralSpacecraft.mDot_SC)(0,0) += (*it)->effProps.mEffDot;
            (*this->primaryCentralSpacecraft.ISCPntB_B) += (*it)->effProps.IEffPntB_B;
            (*this->primaryCentralSpacecraft.c_B) += (*it)->effProps.mEff*(*it)->effProps.rEff_CB_B;
            (*this->primaryCentralSpacecraft.ISCPntBPrime_B) += (*it)->effProps.IEffPrimePntB_B;
            (*this->primaryCentralSpacecraft.cPrime_B) += (*it)->effProps.mEff*(*it)->effProps.rEffPrime_CB_B;
            // For high fidelity mass depletion, this is left out: += (*it)->effProps.mEffDot*(*it)->effProps.rEff_CB_B
        }
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

void SpacecraftSystem::initializeSCPosVelocity(SpacecraftUnit &spacecraft)
{
    Eigen::Vector3d rInit_BN_N = spacecraft.hubR_N->getState();
    Eigen::MRPd sigma_BN;
    sigma_BN = (Eigen::Vector3d) spacecraft.hubSigma->getState();
    Eigen::Matrix3d dcm_NB = sigma_BN.toRotationMatrix();
    // - Substract off the center mass to leave r_BN_N
    rInit_BN_N -= dcm_NB*(*spacecraft.c_B);
    // - Subtract off cDot_B to get v_BN_N
    Eigen::Vector3d vInit_BN_N = spacecraft.hubV_N->getState();
    vInit_BN_N -= dcm_NB*(*spacecraft.cDot_B);
    // - Finally set the translational states r_BN_N and v_BN_N with the corrections
    spacecraft.hubR_N->setState(rInit_BN_N);
    spacecraft.hubV_N->setState(vInit_BN_N);

    return;
}

/*! This method is solving Xdot = F(X,t) for the system. The hub needs to calculate its derivatives, along with all of
 the stateEffectors. The hub also has gravity and dynamicEffectors acting on it and these relationships are controlled
 in this method. At the end of this method all of the states will have their corresponding state derivatives set in the
 dynParam Manager thus solving for Xdot*/
void SpacecraftSystem::equationsOfMotion(double integTimeSeconds, double timeStep)
{
    // - Update time to the current time
    uint64_t integTimeNanos = secToNano(integTimeSeconds);
    (*this->sysTime) << (double)integTimeNanos, integTimeSeconds;

    this->equationsOfMotionSystem(integTimeSeconds, timeStep);
    // Call this for all unconnected spacecraft:
    // - Call this for all of the unconnected spacecraft
    std::vector<SpacecraftUnit*>::iterator spacecraftUnConnectedIt;
    for(spacecraftUnConnectedIt = this->unDockedSpacecraft.begin(); spacecraftUnConnectedIt != this->unDockedSpacecraft.end(); spacecraftUnConnectedIt++)
    {
        this->equationsOfMotionSC(integTimeSeconds, timeStep, (*(*spacecraftUnConnectedIt)));
    }

    return;
}

void SpacecraftSystem::equationsOfMotionSC(double integTimeSeconds, double timeStep, SpacecraftUnit& spacecraft)
{
    // - Zero all Matrices and vectors for back-sub and the dynamics
    spacecraft.hub.hubBackSubMatrices.matrixA.setZero();
    spacecraft.hub.hubBackSubMatrices.matrixB.setZero();
    spacecraft.hub.hubBackSubMatrices.matrixC.setZero();
    spacecraft.hub.hubBackSubMatrices.matrixD.setZero();
    spacecraft.hub.hubBackSubMatrices.vecTrans.setZero();
    spacecraft.hub.hubBackSubMatrices.vecRot.setZero();
    spacecraft.sumForceExternal_B.setZero();
    spacecraft.sumForceExternal_N.setZero();
    spacecraft.sumTorquePntB_B.setZero();

    // - Update the mass properties of the spacecraft
    this->updateSpacecraftMassProps(integTimeSeconds, spacecraft);

    // - This is where gravity is computed (gravity needs to know c_B to calculated gravity about r_CN_N)
    Eigen::MRPd sigmaBNLoc;
    Eigen::Matrix3d dcm_NB;
    Eigen::Vector3d cLocal_N;

    sigmaBNLoc = (Eigen::Vector3d) spacecraft.hubSigma->getState();
    dcm_NB = sigmaBNLoc.toRotationMatrix();
    cLocal_N = dcm_NB*(*spacecraft.c_B);
    Eigen::Vector3d rLocal_CN_N = spacecraft.hubR_N->getState() + dcm_NB*(*spacecraft.c_B);
    Eigen::Vector3d vLocal_CN_N = spacecraft.hubV_N->getState() + dcm_NB*(*spacecraft.cDot_B);

    spacecraft.gravField.computeGravityField(rLocal_CN_N, vLocal_CN_N);

    // - Loop through dynEffectors to compute force and torque on the s/c
    std::vector<DynamicEffector*>::iterator dynIt;
    for(dynIt = spacecraft.dynEffectors.begin(); dynIt != spacecraft.dynEffectors.end(); dynIt++)
    {
        // - Compute the force and torque contributions from the dynamicEffectors
        (*dynIt)->computeForceTorque(integTimeSeconds, timeStep);
        spacecraft.sumForceExternal_N += (*dynIt)->forceExternal_N;
        spacecraft.sumForceExternal_B += (*dynIt)->forceExternal_B;
        spacecraft.sumTorquePntB_B += (*dynIt)->torqueExternalPntB_B;
    }

    // - Loop through state effectors to get contributions for back-substitution
    std::vector<StateEffector*>::iterator it;
    for(it = spacecraft.states.begin(); it != spacecraft.states.end(); it++)
    {
        /* - Set the contribution matrices to zero (just in case a stateEffector += on the matrix or the stateEffector
         doesn't have a contribution for a matrix and doesn't set the matrix to zero */
        spacecraft.backSubMatricesContributions.matrixA.setZero();
        spacecraft.backSubMatricesContributions.matrixB.setZero();
        spacecraft.backSubMatricesContributions.matrixC.setZero();
        spacecraft.backSubMatricesContributions.matrixD.setZero();
        spacecraft.backSubMatricesContributions.vecTrans.setZero();
        spacecraft.backSubMatricesContributions.vecRot.setZero();

        // - Call the update contributions method for the stateEffectors and add in contributions to the hub matrices
        (*it)->updateContributions(integTimeSeconds, spacecraft.backSubMatricesContributions, spacecraft.hubSigma->getState(), spacecraft.hubOmega_BN_B->getState(), *spacecraft.g_N);
        spacecraft.hub.hubBackSubMatrices.matrixA += spacecraft.backSubMatricesContributions.matrixA;
        spacecraft.hub.hubBackSubMatrices.matrixB += spacecraft.backSubMatricesContributions.matrixB;
        spacecraft.hub.hubBackSubMatrices.matrixC += spacecraft.backSubMatricesContributions.matrixC;
        spacecraft.hub.hubBackSubMatrices.matrixD += spacecraft.backSubMatricesContributions.matrixD;
        spacecraft.hub.hubBackSubMatrices.vecTrans += spacecraft.backSubMatricesContributions.vecTrans;
        spacecraft.hub.hubBackSubMatrices.vecRot += spacecraft.backSubMatricesContributions.vecRot;
    }

    // - Finish the math that is needed
    Eigen::Vector3d cLocal_B;
    Eigen::Vector3d cPrimeLocal_B;
    cLocal_B = *spacecraft.c_B;
    cPrimeLocal_B = *spacecraft.cPrime_B;

    Eigen::Matrix3d intermediateMatrix;
    Eigen::Vector3d intermediateVector;
    Eigen::Vector3d omegaLocalBN_B = spacecraft.hubOmega_BN_B->getState();
    spacecraft.hub.hubBackSubMatrices.matrixA += (*spacecraft.m_SC)(0,0)*intermediateMatrix.Identity();
    intermediateMatrix = eigenTilde((*spacecraft.c_B));  // make c_B skew symmetric matrix
    spacecraft.hub.hubBackSubMatrices.matrixB += -(*spacecraft.m_SC)(0,0)*intermediateMatrix;
    spacecraft.hub.hubBackSubMatrices.matrixC += (*spacecraft.m_SC)(0,0)*intermediateMatrix;
    spacecraft.hub.hubBackSubMatrices.matrixD += *spacecraft.ISCPntB_B;
    spacecraft.hub.hubBackSubMatrices.vecTrans += -2.0*(*spacecraft.m_SC)(0, 0)*omegaLocalBN_B.cross(cPrimeLocal_B)
    - (*spacecraft.m_SC)(0, 0)*omegaLocalBN_B.cross(omegaLocalBN_B.cross(cLocal_B))
    - 2.0*(*spacecraft.mDot_SC)(0,0)*(cPrimeLocal_B+omegaLocalBN_B.cross(cLocal_B));
    intermediateVector = *spacecraft.ISCPntB_B*omegaLocalBN_B;
    spacecraft.hub.hubBackSubMatrices.vecRot += -omegaLocalBN_B.cross(intermediateVector) - *spacecraft.ISCPntBPrime_B*omegaLocalBN_B;

    // - Map external force_N to the body frame
    Eigen::Vector3d sumForceExternalMappedToB;
    sumForceExternalMappedToB = dcm_NB.transpose()*spacecraft.sumForceExternal_N;

    // - Edit both v_trans and v_rot with gravity and external force and torque
    Eigen::Vector3d gLocal_N = *spacecraft.g_N;

    // -  Make additional contributions to the matrices from the hub

    // - Need to find force of gravity on the spacecraft
    Eigen::Vector3d gravityForce_N;
    gravityForce_N = (*spacecraft.m_SC)(0,0)*gLocal_N;

    Eigen::Vector3d gravityForce_B;
    gravityForce_B = dcm_NB.transpose()*gravityForce_N;
    spacecraft.hub.hubBackSubMatrices.vecTrans += gravityForce_B + sumForceExternalMappedToB + spacecraft.sumForceExternal_B;
    spacecraft.hub.hubBackSubMatrices.vecRot += cLocal_B.cross(gravityForce_B) + spacecraft.sumTorquePntB_B;

    // - Compute the derivatives of the hub states before looping through stateEffectors
    spacecraft.hub.computeDerivatives(integTimeSeconds, spacecraft.hubV_N->getStateDeriv(), spacecraft.hubOmega_BN_B->getStateDeriv(), spacecraft.hubSigma->getState());

    // - Loop through state effectors for compute derivatives
    for(it = spacecraft.states.begin(); it != spacecraft.states.end(); it++)
    {
        (*it)->computeDerivatives(integTimeSeconds, spacecraft.hubV_N->getStateDeriv(), spacecraft.hubOmega_BN_B->getStateDeriv(), spacecraft.hubSigma->getState());
    }

    return;
}

void SpacecraftSystem::equationsOfMotionSystem(double integTimeSeconds, double timeStep)
{
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
        (*dynIt)->computeForceTorque(integTimeSeconds, timeStep);
        this->primaryCentralSpacecraft.sumForceExternal_N += (*dynIt)->forceExternal_N;
        this->primaryCentralSpacecraft.sumForceExternal_B += (*dynIt)->forceExternal_B;
        this->primaryCentralSpacecraft.sumTorquePntB_B += (*dynIt)->torqueExternalPntB_B;
    }

    // - Call this for all of the connected spacecraft
    std::vector<SpacecraftUnit*>::iterator spacecraftConnectedIt;
    for(spacecraftConnectedIt = this->spacecraftDockedToPrimary.begin(); spacecraftConnectedIt != this->spacecraftDockedToPrimary.end(); spacecraftConnectedIt++)
    {
        // - Loop through dynEffectors to compute force and torque on the s/c
        for(dynIt = (*spacecraftConnectedIt)->dynEffectors.begin(); dynIt != (*spacecraftConnectedIt)->dynEffectors.end(); dynIt++)
        {
            // - Compute the force and torque contributions from the dynamicEffectors
            (*dynIt)->computeForceTorque(integTimeSeconds, timeStep);
            this->primaryCentralSpacecraft.sumForceExternal_N += (*dynIt)->forceExternal_N;
            this->primaryCentralSpacecraft.sumForceExternal_B += (*dynIt)->forceExternal_B;
            this->primaryCentralSpacecraft.sumTorquePntB_B += (*dynIt)->torqueExternalPntB_B;
        }
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
        (*it)->updateContributions(integTimeSeconds, this->primaryCentralSpacecraft.backSubMatricesContributions, this->primaryCentralSpacecraft.hubSigma->getState(), this->primaryCentralSpacecraft.hubOmega_BN_B->getState(), *this->primaryCentralSpacecraft.g_N);
        this->primaryCentralSpacecraft.hub.hubBackSubMatrices.matrixA += this->primaryCentralSpacecraft.backSubMatricesContributions.matrixA;
        this->primaryCentralSpacecraft.hub.hubBackSubMatrices.matrixB += this->primaryCentralSpacecraft.backSubMatricesContributions.matrixB;
        this->primaryCentralSpacecraft.hub.hubBackSubMatrices.matrixC += this->primaryCentralSpacecraft.backSubMatricesContributions.matrixC;
        this->primaryCentralSpacecraft.hub.hubBackSubMatrices.matrixD += this->primaryCentralSpacecraft.backSubMatricesContributions.matrixD;
        this->primaryCentralSpacecraft.hub.hubBackSubMatrices.vecTrans += this->primaryCentralSpacecraft.backSubMatricesContributions.vecTrans;
        this->primaryCentralSpacecraft.hub.hubBackSubMatrices.vecRot += this->primaryCentralSpacecraft.backSubMatricesContributions.vecRot;
    }

    // - Call this for all of the connected spacecraft
    for(spacecraftConnectedIt = this->spacecraftDockedToPrimary.begin(); spacecraftConnectedIt != this->spacecraftDockedToPrimary.end(); spacecraftConnectedIt++)
    {
        for(it = (*spacecraftConnectedIt)->states.begin(); it != (*spacecraftConnectedIt)->states.end(); it++)
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
            (*it)->updateContributions(integTimeSeconds, this->primaryCentralSpacecraft.backSubMatricesContributions, this->primaryCentralSpacecraft.hubSigma->getState(), this->primaryCentralSpacecraft.hubOmega_BN_B->getState(), *this->primaryCentralSpacecraft.g_N);
            this->primaryCentralSpacecraft.hub.hubBackSubMatrices.matrixA += this->primaryCentralSpacecraft.backSubMatricesContributions.matrixA;
            this->primaryCentralSpacecraft.hub.hubBackSubMatrices.matrixB += this->primaryCentralSpacecraft.backSubMatricesContributions.matrixB;
            this->primaryCentralSpacecraft.hub.hubBackSubMatrices.matrixC += this->primaryCentralSpacecraft.backSubMatricesContributions.matrixC;
            this->primaryCentralSpacecraft.hub.hubBackSubMatrices.matrixD += this->primaryCentralSpacecraft.backSubMatricesContributions.matrixD;
            this->primaryCentralSpacecraft.hub.hubBackSubMatrices.vecTrans += this->primaryCentralSpacecraft.backSubMatricesContributions.vecTrans;
            this->primaryCentralSpacecraft.hub.hubBackSubMatrices.vecRot += this->primaryCentralSpacecraft.backSubMatricesContributions.vecRot;
        }
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

    // - Need to find force of gravity on the this->primaryCentralSpacecraft
    Eigen::Vector3d gravityForce_N;
    gravityForce_N = (*this->primaryCentralSpacecraft.m_SC)(0,0)*gLocal_N;

    Eigen::Vector3d gravityForce_B;
    gravityForce_B = dcm_NB.transpose()*gravityForce_N;
    this->primaryCentralSpacecraft.hub.hubBackSubMatrices.vecTrans += gravityForce_B + sumForceExternalMappedToB + this->primaryCentralSpacecraft.sumForceExternal_B;
    this->primaryCentralSpacecraft.hub.hubBackSubMatrices.vecRot += cLocal_B.cross(gravityForce_B) + this->primaryCentralSpacecraft.sumTorquePntB_B;

    // - Compute the derivatives of the hub states before looping through stateEffectors
    this->primaryCentralSpacecraft.hub.computeDerivatives(integTimeSeconds, this->primaryCentralSpacecraft.hubV_N->getStateDeriv(), this->primaryCentralSpacecraft.hubOmega_BN_B->getStateDeriv(), this->primaryCentralSpacecraft.hubSigma->getState());

    // - Need to figure out how to pass accelerations of rBDDot and omegaDot of the primary hub to the other hubs stateEffectors

    // - Loop through state effectors for compute derivatives
    for(it = this->primaryCentralSpacecraft.states.begin(); it != this->primaryCentralSpacecraft.states.end(); it++)
    {
        (*it)->computeDerivatives(integTimeSeconds, this->primaryCentralSpacecraft.hubV_N->getStateDeriv(), this->primaryCentralSpacecraft.hubOmega_BN_B->getStateDeriv(), this->primaryCentralSpacecraft.hubSigma->getState());
    }

    // - Call this for all of the connected spacecraft
    for(spacecraftConnectedIt = this->spacecraftDockedToPrimary.begin(); spacecraftConnectedIt != this->spacecraftDockedToPrimary.end(); spacecraftConnectedIt++)
    {
        // - Loop through state effectors for compute derivatives
        for(it = (*spacecraftConnectedIt)->states.begin(); it != (*spacecraftConnectedIt)->states.end(); it++)
        {
            (*it)->computeDerivatives(integTimeSeconds, this->primaryCentralSpacecraft.hubV_N->getStateDeriv(), this->primaryCentralSpacecraft.hubOmega_BN_B->getStateDeriv(), this->primaryCentralSpacecraft.hubSigma->getState());
        }
    }

    return;
}

void SpacecraftSystem::findPriorStateInformation(SpacecraftUnit &spacecraft)
{
    // - Find v_CN_N before integration for accumulated DV
    spacecraft.oldV_BN_N = spacecraft.hubV_N->getState();  // - V_BN_N before integration
    Eigen::Vector3d oldC_B;     // - Center of mass offset before integration
    Eigen::Vector3d oldOmega_BN_B;  // - angular rate of B wrt N in the Body frame
    Eigen::MRPd oldSigma_BN;    // - Sigma_BN before integration
    // - Get the angular rate, oldOmega_BN_B from the dyn manager
    oldOmega_BN_B = spacecraft.hubOmega_BN_B->getState();
    // - Get center of mass, v_BN_N and dcm_NB from the dyn manager
    oldSigma_BN = (Eigen::Vector3d) spacecraft.hubSigma->getState();
    // - Finally find v_CN_N
    Eigen::Matrix3d oldDcm_NB = oldSigma_BN.toRotationMatrix(); // - dcm_NB before integration
    spacecraft.oldV_CN_N = spacecraft.oldV_BN_N + oldDcm_NB*(*spacecraft.cDot_B);
    spacecraft.hub.matchGravitytoVelocityState(spacecraft.oldV_CN_N);

    return;
}

void SpacecraftSystem::determineAttachedSCStates()
{
    // Pull out primary spacecraft states
    Eigen::MRPd sigmaPNLoc;
    sigmaPNLoc = (Eigen::Vector3d) this->primaryCentralSpacecraft.hubSigma->getState();
    Eigen::Vector3d omegaPNLoc_P = this->primaryCentralSpacecraft.hubOmega_BN_B->getState();
    Eigen::Vector3d r_PNLocal_N = this->primaryCentralSpacecraft.hubR_N->getState();
    Eigen::Vector3d r_PNDotLocal_N = this->primaryCentralSpacecraft.hubV_N->getState();
    Eigen::Matrix3d dcm_NP;
    dcm_NP = sigmaPNLoc.toRotationMatrix();

    // Loop over connected spacecraft to edit the hub states
    // - Call this for all of the connected spacecraft
    std::vector<SpacecraftUnit*>::iterator spacecraftConnectedIt;
    for(spacecraftConnectedIt = this->spacecraftDockedToPrimary.begin(); spacecraftConnectedIt != this->spacecraftDockedToPrimary.end(); spacecraftConnectedIt++)
    {
        Eigen::Matrix3d dcm_BN = (*spacecraftConnectedIt)->hub.dcm_BP*dcm_NP.transpose();
        double dcm_BNArray[9];
        double sigma_BNArray[3];
        eigenMatrix3d2CArray(dcm_BN, dcm_BNArray);
        C2MRP(RECAST3X3 dcm_BNArray, sigma_BNArray);
        Eigen::Vector3d sigmaBNLoc;
        sigmaBNLoc = cArray2EigenVector3d(sigma_BNArray);
        // Set the MRP for the hub
        (*spacecraftConnectedIt)->hubSigma->setState(sigmaBNLoc);

        // Now omega
        Eigen::Vector3d omegaBNLocal_B = (*spacecraftConnectedIt)->hub.dcm_BP*omegaPNLoc_P;
        (*spacecraftConnectedIt)->hubOmega_BN_B->setState(omegaBNLocal_B);

        // Now the translation states
        Eigen::Vector3d r_BNLocal_N = r_PNLocal_N + dcm_NP*(*spacecraftConnectedIt)->hub.r_BP_P;
        (*spacecraftConnectedIt)->hubR_N->setState(r_BNLocal_N);
        Eigen::Vector3d r_BNDotLocal_N = r_PNDotLocal_N + dcm_NP*omegaPNLoc_P.cross((*spacecraftConnectedIt)->hub.r_BP_P);
        (*spacecraftConnectedIt)->hubV_N->setState(r_BNDotLocal_N);
    }

    return;
}

void SpacecraftSystem::calculateDeltaVandAcceleration(SpacecraftUnit &spacecraft, double localTimeStep)
{
    // - Find v_CN_N after the integration for accumulated DV
    Eigen::Vector3d newV_BN_N = spacecraft.hubV_N->getState(); // - V_BN_N after integration
    Eigen::Vector3d newV_CN_N;  // - V_CN_N after integration
    Eigen::MRPd newSigma_BN;    // - Sigma_BN after integration
    // - Get center of mass, v_BN_N and dcm_NB
    Eigen::Vector3d sigmaBNLoc;
    sigmaBNLoc = (Eigen::Vector3d) spacecraft.hubSigma->getState();
    newSigma_BN = sigmaBNLoc;
    Eigen::Matrix3d newDcm_NB = newSigma_BN.toRotationMatrix();  // - dcm_NB after integration
    newV_CN_N = newV_BN_N + newDcm_NB*(*spacecraft.cDot_B);

    // - Find accumulated DV of the center of mass in the body frame
    spacecraft.dvAccum_CN_B += newDcm_NB.transpose()*(newV_CN_N -
                                                    spacecraft.BcGravVelocity->getState());

    // - Find the accumulated DV of the body frame in the body frame
    spacecraft.dvAccum_BN_B += newDcm_NB.transpose()*(newV_BN_N -
                                                    spacecraft.hubGravVelocity->getState());

    // - non-conservative acceleration of the body frame in the body frame
    spacecraft.nonConservativeAccelpntB_B = (newDcm_NB.transpose()*(newV_BN_N -
                                                                   spacecraft.hubGravVelocity->getState()))/localTimeStep;

    // - angular acceleration in the body frame
    Eigen::Vector3d newOmega_BN_B;
    newOmega_BN_B = spacecraft.hubOmega_BN_B->getState();
    spacecraft.omegaDot_BN_B = (newOmega_BN_B - spacecraft.oldOmega_BN_B)/localTimeStep; //angular acceleration of B wrt N in the Body frame

    return;
}

/*! This method is used to find the total energy and momentum of the spacecraft. It finds the total orbital energy,
 total orbital angular momentum, total rotational energy and total rotational angular momentum. These values are used
 for validation purposes. */
void SpacecraftSystem::computeEnergyMomentum(double time)
{
    this->computeEnergyMomentumSystem(time);

    // - Call this for all of the unconnected spacecraft
    std::vector<SpacecraftUnit*>::iterator spacecraftUnConnectedIt;
    for(spacecraftUnConnectedIt = this->unDockedSpacecraft.begin(); spacecraftUnConnectedIt != this->unDockedSpacecraft.end(); spacecraftUnConnectedIt++)
    {
        this->computeEnergyMomentumSC(time, (*(*spacecraftUnConnectedIt)));
    }

    return;
}

void SpacecraftSystem::computeEnergyMomentumSC(double time, SpacecraftUnit &spacecraft)
{
    // - Grab values from state Manager
    Eigen::Vector3d rLocal_BN_N = spacecraft.hubR_N->getState();
    Eigen::Vector3d rDotLocal_BN_N = spacecraft.hubV_N->getState();
    Eigen::MRPd sigmaLocal_BN;
    sigmaLocal_BN = (Eigen::Vector3d ) spacecraft.hubSigma->getState();

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
    spacecraft.totOrbAngMomPntN_N.setZero();
    spacecraft.totRotAngMomPntC_N.setZero();
    spacecraft.rotAngMomPntCContr_B.setZero();
    spacecraft.totOrbEnergy = 0.0;
    spacecraft.totRotEnergy = 0.0;
    spacecraft.rotEnergyContr = 0.0;

    // - Get the hubs contribution
    spacecraft.hub.updateEnergyMomContributions(time, spacecraft.rotAngMomPntCContr_B, spacecraft.rotEnergyContr, spacecraft.hubOmega_BN_B->getState());
    totRotAngMomPntC_B += spacecraft.rotAngMomPntCContr_B;
    spacecraft.totRotEnergy += spacecraft.rotEnergyContr;

    // - Loop over stateEffectors to get their contributions to energy and momentum
    std::vector<StateEffector*>::iterator it;
    for(it = spacecraft.states.begin(); it != spacecraft.states.end(); it++)
    {
        // - Set the matrices to zero
        spacecraft.rotAngMomPntCContr_B.setZero();
        spacecraft.rotEnergyContr = 0.0;

        // - Call energy and momentum calulations for stateEffectors
        (*it)->updateEnergyMomContributions(time, spacecraft.rotAngMomPntCContr_B, spacecraft.rotEnergyContr, spacecraft.hubOmega_BN_B->getState());
        totRotAngMomPntC_B += spacecraft.rotAngMomPntCContr_B;
        spacecraft.totRotEnergy += spacecraft.rotEnergyContr;
    }

    // - Get cDot_B from manager
    Eigen::Vector3d cDotLocal_B = (*spacecraft.cDot_B);

    // - Add in orbital kinetic energy into the total orbital energy calculations
    spacecraft.totOrbEnergy += 1.0/2.0*(*spacecraft.m_SC)(0,0)*(rDotBNLocal_B.dot(rDotBNLocal_B) + 2.0*rDotBNLocal_B.dot(cDotLocal_B)
                                                                                                        + cDotLocal_B.dot(cDotLocal_B));

    // - Call gravity effector and add in its potential contributions to the total orbital energy calculations
    spacecraft.orbPotentialEnergyContr = 0.0;
    Eigen::Vector3d rLocal_CN_N = spacecraft.hubR_N->getState() + dcmLocal_NB*(*spacecraft.c_B);
    spacecraft.gravField.updateEnergyContributions(rLocal_CN_N, spacecraft.orbPotentialEnergyContr);
    spacecraft.totOrbEnergy += (*spacecraft.m_SC)(0,0)*spacecraft.orbPotentialEnergyContr;

    // - Find total rotational energy
    spacecraft.totRotEnergy += -1.0/2.0*(*spacecraft.m_SC)(0,0)*cDotLocal_B.dot(cDotLocal_B);

    // - Find orbital angular momentum for the spacecraft
    Eigen::Vector3d rCN_N;
    Eigen::Vector3d rDotCN_N;
    rCN_N = rLocal_BN_N + dcmLocal_NB*(*spacecraft.c_B);
    rDotCN_N = rDotLocal_BN_N + dcmLocal_NB*cDotLocal_B;
    spacecraft.totOrbAngMomPntN_N = (*spacecraft.m_SC)(0,0)*(rCN_N.cross(rDotCN_N));

    // - Find rotational angular momentum for the spacecraft
    totRotAngMomPntC_B += -(*spacecraft.m_SC)(0,0)*(Eigen::Vector3d (*spacecraft.c_B)).cross(cDotLocal_B);
    spacecraft.totRotAngMomPntC_N = dcmLocal_NB*totRotAngMomPntC_B;

    return;
}

void SpacecraftSystem::computeEnergyMomentumSystem(double time)
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
    this->primaryCentralSpacecraft.hub.updateEnergyMomContributions(time, this->primaryCentralSpacecraft.rotAngMomPntCContr_B, this->primaryCentralSpacecraft.rotEnergyContr, this->primaryCentralSpacecraft.hubOmega_BN_B->getState());
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
        (*it)->updateEnergyMomContributions(time, this->primaryCentralSpacecraft.rotAngMomPntCContr_B, this->primaryCentralSpacecraft.rotEnergyContr, this->primaryCentralSpacecraft.hubOmega_BN_B->getState());
        totRotAngMomPntC_B += this->primaryCentralSpacecraft.rotAngMomPntCContr_B;
        this->primaryCentralSpacecraft.totRotEnergy += this->primaryCentralSpacecraft.rotEnergyContr;
    }

    // - Get all of the attached hubs and stateEffectors contributions
    std::vector<SpacecraftUnit*>::iterator spacecraftConnectedIt;
    for(spacecraftConnectedIt = this->spacecraftDockedToPrimary.begin(); spacecraftConnectedIt != this->spacecraftDockedToPrimary.end(); spacecraftConnectedIt++)
    {
        // - Get the hubs contribution
        (*spacecraftConnectedIt)->hub.updateEnergyMomContributions(time, this->primaryCentralSpacecraft.rotAngMomPntCContr_B, this->primaryCentralSpacecraft.rotEnergyContr, this->primaryCentralSpacecraft.hubOmega_BN_B->getState());
        totRotAngMomPntC_B += this->primaryCentralSpacecraft.rotAngMomPntCContr_B;
        this->primaryCentralSpacecraft.totRotEnergy += this->primaryCentralSpacecraft.rotEnergyContr;

        // - Loop over stateEffectors to get their contributions to energy and momentum
        std::vector<StateEffector*>::iterator it;
        for(it = (*spacecraftConnectedIt)->states.begin(); it != (*spacecraftConnectedIt)->states.end(); it++)
        {
            // - Set the matrices to zero
            this->primaryCentralSpacecraft.rotAngMomPntCContr_B.setZero();
            this->primaryCentralSpacecraft.rotEnergyContr = 0.0;

            // - Call energy and momentum calulations for stateEffectors
            (*it)->updateEnergyMomContributions(time, this->primaryCentralSpacecraft.rotAngMomPntCContr_B, this->primaryCentralSpacecraft.rotEnergyContr, this->primaryCentralSpacecraft.hubOmega_BN_B->getState());
            totRotAngMomPntC_B += this->primaryCentralSpacecraft.rotAngMomPntCContr_B;
            this->primaryCentralSpacecraft.totRotEnergy += this->primaryCentralSpacecraft.rotEnergyContr;
        }
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

/*! Prepare for integration process, not currently implemented in SpacecraftSystem
 @param integrateToThisTimeNanos Time to integrate to
 */
void SpacecraftSystem::preIntegration(uint64_t integrateToThisTimeNanos) {

    // - Find the time step
    this->timeStep = diffNanoToSec(integrateToThisTimeNanos, this->timeBeforeNanos);

    this->findPriorStateInformation(this->primaryCentralSpacecraft);

    // - Call this for all of the unconnected spacecraft
    std::vector<SpacecraftUnit*>::iterator spacecraftUnConnectedIt;
    for(spacecraftUnConnectedIt = this->unDockedSpacecraft.begin(); spacecraftUnConnectedIt != this->unDockedSpacecraft.end(); spacecraftUnConnectedIt++)
    {
        this->findPriorStateInformation((*(*spacecraftUnConnectedIt)));
    }

}

/*! Perform post-integration steps, not currently implemented in SpacecraftSystem
 @param integrateToThisTimeNanos Time to integrate to
 */
void SpacecraftSystem::postIntegration(uint64_t integrateToThisTimeNanos) {
    std::vector<SpacecraftUnit*>::iterator spacecraftUnConnectedIt;
    this->timeBeforeNanos = integrateToThisTimeNanos; // - copy the current time into previous time for next integrate state call
    this->timeBefore = integrateToThisTimeNanos*NANO2SEC;     // - copy the current time into previous time for next integrate state call
    double integrateToThisTime = integrateToThisTimeNanos*NANO2SEC; // - convert to seconds

    // - Calculate the states of the attached spacecraft from the primary spacecraft
    this->determineAttachedSCStates();

    // - Call hubs modify states to allow for switching of MRPs
    this->primaryCentralSpacecraft.hub.modifyStates(integrateToThisTime);

    // - Just in case the MRPs of the attached hubs need to be switched
    std::vector<SpacecraftUnit*>::iterator spacecraftConnectedIt;
    for(spacecraftConnectedIt = this->spacecraftDockedToPrimary.begin(); spacecraftConnectedIt != this->spacecraftDockedToPrimary.end(); spacecraftConnectedIt++)
    {
        (*spacecraftConnectedIt)->hub.modifyStates(integrateToThisTime);
    }

    // - Just in case the MRPs of the unattached hubs need to be switched
    for(spacecraftUnConnectedIt = this->unDockedSpacecraft.begin(); spacecraftUnConnectedIt != this->unDockedSpacecraft.end(); spacecraftUnConnectedIt++)
    {
        (*spacecraftUnConnectedIt)->hub.modifyStates(integrateToThisTime);
    }

    // - Loop over stateEffectors to call modifyStates
    std::vector<StateEffector*>::iterator it;
    for(it = this->primaryCentralSpacecraft.states.begin(); it != this->primaryCentralSpacecraft.states.end(); it++)
    {
        // - Call energy and momentum calulations for stateEffectors
        (*it)->modifyStates(integrateToThisTime);
    }

    // - Call this for all of the connected spacecraft
    for(spacecraftConnectedIt = this->spacecraftDockedToPrimary.begin(); spacecraftConnectedIt != this->spacecraftDockedToPrimary.end(); spacecraftConnectedIt++)
    {
        for(it = (*spacecraftConnectedIt)->states.begin(); it != (*spacecraftConnectedIt)->states.end(); it++)
        {
            // - Call energy and momentum calulations for stateEffectors
            (*it)->modifyStates(integrateToThisTime);
        }
    }

    // - Call this for all of the unconnected spacecraft
    for(spacecraftUnConnectedIt = this->unDockedSpacecraft.begin(); spacecraftUnConnectedIt != this->unDockedSpacecraft.end(); spacecraftUnConnectedIt++)
    {
        for(it = (*spacecraftUnConnectedIt)->states.begin(); it != (*spacecraftUnConnectedIt)->states.end(); it++)
        {
            // - Call energy and momentum calulations for stateEffectors
            (*it)->modifyStates(integrateToThisTime);
        }
    }

    // - Call mass properties to get current info on the mass props of the spacecraft
    this->updateSystemMassProps(integrateToThisTime);

    // - Call mass props for all the rest of the spacecraft
    for(spacecraftUnConnectedIt = this->unDockedSpacecraft.begin(); spacecraftUnConnectedIt != this->unDockedSpacecraft.end(); spacecraftUnConnectedIt++)
    {
        this->updateSpacecraftMassProps(integrateToThisTime, (*(*spacecraftUnConnectedIt)));
    }

    this->calculateDeltaVandAcceleration(this->primaryCentralSpacecraft, this->timeStep);

    // - Call for the rest of the spacecraft
    for(spacecraftUnConnectedIt = this->unDockedSpacecraft.begin(); spacecraftUnConnectedIt != this->unDockedSpacecraft.end(); spacecraftUnConnectedIt++)
    {
        this->calculateDeltaVandAcceleration((*(*spacecraftUnConnectedIt)), this->timeStep);
    }

    // - Compute Energy and Momentum
    this->computeEnergyMomentum(integrateToThisTime);

}
