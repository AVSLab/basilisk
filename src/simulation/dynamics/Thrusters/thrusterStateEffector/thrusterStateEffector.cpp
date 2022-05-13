/*
 ISC License

 Copyright (c) 2022, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#include <cstring>
#include <iostream>
#include <cmath>

#include "thrusterStateEffector.h"
#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/astroConstants.h"
#include "architecture/utilities/macroDefinitions.h"
#include "architecture/utilities/avsEigenSupport.h"
#include <cstring>
#include <iostream>
#include <cmath>

/*! The Constructor.*/
ThrusterStateEffector::ThrusterStateEffector()
{
    // - zero the mass props and mass prop rates contributions
    this->effProps.mEff = 0.0;
    this->effProps.rEff_CB_B.fill(0.0);
    this->effProps.IEffPntB_B.fill(0.0);
    this->effProps.rEffPrime_CB_B.fill(0.0);
    this->effProps.IEffPrimePntB_B.fill(0.0);

    CallCounts = 0;
    this->mDotTotal = 0.0;
    this->kappaInit = 0.00;
    this->kappaDotInit = 0.0;
    this->nameOfKappaState = "thrusterKappa" + std::to_string(this->effectorID);
    this->nameOfKappaDotState = "thrusterKappaDot" + std::to_string(this->effectorID);
    this->effectorID++;

    return;
}

uint64_t ThrusterStateEffector::effectorID = 1;

/*! The destructor. */
ThrusterStateEffector::~ThrusterStateEffector()
{
    for (long unsigned int c=0; c<this->thrusterOutMsgs.size(); c++) {
        free(this->thrusterOutMsgs.at(c));
    }

    this->effectorID = 1;    /* reset the panel ID*/

    return;
}

/*! This method is here to write the output message structure into the specified
 message.
 @param CurrentClock The current time used for time-stamping the message
 @return void
 */
void ThrusterStateEffector::writeOutputStateMessages(uint64_t CurrentClock)
{
    int idx = 0;
    std::vector<THRSimConfigMsgPayload>::iterator it;

    THROutputMsgPayload tmpThruster;
    for (it = this->thrusterData.begin(); it != this->thrusterData.end(); ++it)
    {
        tmpThruster = this->thrusterOutMsgs[idx]->zeroMsgPayload;
        eigenVector3d2CArray(it->thrLoc_B, tmpThruster.thrusterLocation);
        eigenVector3d2CArray(it->thrDir_B, tmpThruster.thrusterDirection);
        tmpThruster.maxThrust = it->MaxThrust;
        tmpThruster.thrustFactor = it->ThrustOps.ThrustFactor;
        tmpThruster.thrustForce = v3Norm(it->ThrustOps.opThrustForce_B);
        v3Copy(it->ThrustOps.opThrustForce_B, tmpThruster.thrustForce_B);
        v3Copy(it->ThrustOps.opThrustTorquePntB_B, tmpThruster.thrustTorquePntB_B);
        
        this->thrusterOutMsgs[idx]->write(&tmpThruster, this->moduleID, CurrentClock);

        idx ++;
    }
}


/*! This method is used to read the incoming command message and set the
 associated command structure for operating the thrusters.
 @return void
 */
bool ThrusterStateEffector::ReadInputs()
{
    
    std::vector<double>::iterator CmdIt;
    uint64_t i;
    bool dataGood;
    
    if (this->cmdsInMsg.isLinked()) {
        //! - read the incoming command array
        this->incomingCmdBuffer = this->cmdsInMsg();
        dataGood = this->cmdsInMsg.isWritten();

        //! - Check if message has already been read, if stale return
        if(this->prevCommandTime==this->cmdsInMsg.timeWritten() || !dataGood) {
            return(false);
        }
        this->prevCommandTime = this->cmdsInMsg.timeWritten();
    } else {
        this->incomingCmdBuffer = this->cmdsInMsg.zeroMsgPayload;
        this->prevCommandTime = 0;
    }

    //! - Set the NewThrustCmds vector.  Using the data() method for raw speed
    double *CmdPtr;
    for(i=0, CmdPtr = NewThrustCmds.data(); i < this->thrusterData.size();
        CmdPtr++, i++)
    {
        *CmdPtr = this->incomingCmdBuffer.OnTimeRequest[i];
    }
    return(true);
    
}

/*! This method is used to link the states to the thrusters
 @return void
 @param states The states to link
 */
void ThrusterStateEffector::linkInStates(DynParamManager& states){
    this->hubSigma = states.getStateObject("hubSigma");
	this->hubOmega = states.getStateObject("hubOmega");
}

void ThrusterStateEffector::addThruster(THRSimConfigMsgPayload *newThruster)
{
    this->thrusterData.push_back(*newThruster);

    /* create corresponding output message */
    Message<THROutputMsgPayload> *msg;
    msg = new Message<THROutputMsgPayload>;
    this->thrusterOutMsgs.push_back(msg);
}

/*! This method is the main cyclical call for the scheduled part of the thruster
 dynamics model.  It reads the current commands array and sets the thruster
 configuration data based on that incoming command set.  Note that the main
 dynamical method (ComputeDynamics()) is not called here and is intended to be
 called from the dynamics plant in the system
 @return void
 @param CurrentSimNanos The current simulation time in nanoseconds
 */
void ThrusterStateEffector::UpdateState(uint64_t CurrentSimNanos)
{
    //! - Read the inputs and then call ConfigureThrustRequests to set up dynamics
    if(this->ReadInputs())
    {
        //this->ConfigureThrustRequests(this->prevCommandTime*1.0E-9);
    }
    this->writeOutputStateMessages(CurrentSimNanos);
}

/*! This method allows the HRB state effector to register its states: theta and thetaDot with the dyn param manager */
void ThrusterStateEffector::registerStates(DynParamManager& states)
{

    return;
}

/*! This method allows the HRB state effector to provide its contributions to the mass props and mass prop rates of the
 spacecraft */
void ThrusterStateEffector::updateEffectorMassProps(double integTime)
{
   

    return;
}

/*! This method allows the HRB state effector to give its contributions to the matrices needed for the back-sub
 method */
void ThrusterStateEffector::updateContributions(double integTime, BackSubMatrices& backSubContr, Eigen::Vector3d sigma_BN, Eigen::Vector3d omega_BN_B, Eigen::Vector3d g_N)
{

    return;
}

/*! This method is used to find the derivatives for the HRB stateEffector: thetaDDot and the kinematic derivative */
void ThrusterStateEffector::computeDerivatives(double integTime, Eigen::Vector3d rDDot_BN_N, Eigen::Vector3d omegaDot_BN_B, Eigen::Vector3d sigma_BN)
{
    
    return;
}

/*! This method is for calculating the contributions of the HRB state effector to the energy and momentum of the s/c */
void ThrusterStateEffector::updateEnergyMomContributions(double integTime, Eigen::Vector3d& rotAngMomPntCContr_B,
    double& rotEnergyContr, Eigen::Vector3d omega_BN_B)
{

    return;
}

void ThrusterStateEffector::calcForceTorqueOnBody(double integTime, Eigen::Vector3d omega_BN_B)
{

    return;
}
