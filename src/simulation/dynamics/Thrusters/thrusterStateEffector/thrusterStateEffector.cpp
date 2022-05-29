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
    this->prevFireTime = 0.0;
    this->prevCommandTime = 0xFFFFFFFFFFFFFFFF;
    this->kappaInit = 0.0;
    this->nameOfKappaState = "thrusterKappa" + std::to_string(this->effectorID);
    this->effectorID++;

    return;
}

uint64_t ThrusterStateEffector::effectorID = 1;

/*! The destructor. */
ThrusterStateEffector::~ThrusterStateEffector()
{
    // Free memory to avoid errors
    for (long unsigned int c=0; c<this->thrusterOutMsgs.size(); c++) {
        free(this->thrusterOutMsgs.at(c));
    }

    this->effectorID = 1;    /* reset the panel ID*/

    return;
}

/*! This method is used to reset the module.
 @return void
 */
void ThrusterStateEffector::Reset(uint64_t CurrentSimNanos)
{
    //! - Clear out any currently firing thrusters and re-init cmd array
    this->NewThrustCmds.clear();
    this->NewThrustCmds.insert(this->NewThrustCmds.begin(), this->thrusterData.size(), 0.0);

    return;
}

/*! This method is used to read the incoming command message and set the
 associated command structure for operating the thrusters.
 @return void
 */
bool ThrusterStateEffector::ReadInputs()
{
    // Initialize local variables
    std::vector<double>::iterator CmdIt;
    uint64_t i;
    bool dataGood;
    
    //Check if the message has been linked
    if (this->cmdsInMsg.isLinked()) {
        //! - Read the incoming command array
        this->incomingCmdBuffer = this->cmdsInMsg();
        dataGood = this->cmdsInMsg.isWritten();

        //! - Check if message has already been read, if so then stale return
        if(this->prevCommandTime == this->cmdsInMsg.timeWritten() || !dataGood) {
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

        idx++;
    }
}

/*! This method is used to read the new commands vector and set the thruster
 firings appropriately.  It assumes that the ReadInputs method has already been
 run successfully.  It honors all previous thruster firings if they are still
 active.  Note that for unit testing purposes you can insert firings directly
 into NewThrustCmds.
 @return void
 @param currentTime The current simulation time converted to a double
 */
void ThrusterStateEffector::ConfigureThrustRequests(uint64_t currentTime)
{
    std::vector<THRSimConfigMsgPayload>::iterator it;
    std::vector<double>::iterator CmdIt;
    //! - Iterate through the list of thruster commands that we read in.
    for (CmdIt = NewThrustCmds.begin(), it = this->thrusterData.begin();
        it != this->thrusterData.end(); it++, CmdIt++)
    {
        if (*CmdIt >= it->MinOnTime) /// - Check to see if we have met minimum for each thruster
        {
            //! - For each case where we are above the minimum firing request, reset the thruster
            it->ThrustOps.ThrustOnCmd = *CmdIt;
            it->ThrustOps.fireCounter += it->ThrustOps.ThrustFactor > 0.0
                ? 0 : 1;
        }
        else
        {
            //! - Will ensure that thruster shuts down once this cmd expires
            it->ThrustOps.ThrustOnCmd = it->ThrustOps.ThrustFactor > 1E-5
                ? *CmdIt : 0.0;
        }
        it->ThrustOps.ThrusterEndTime = this->prevCommandTime + it->ThrustOps.ThrustOnCmd;
        //! After we have assigned the firing to the internal thruster, zero the command request.
        *CmdIt = 0.0;
    }

}

void ThrusterStateEffector::addThruster(THRSimConfigMsgPayload* newThruster)
{
    this->thrusterData.push_back(*newThruster);

    /* create corresponding output message */
    Message<THROutputMsgPayload>* msg;
    msg = new Message<THROutputMsgPayload>;
    this->thrusterOutMsgs.push_back(msg);
}

/*! This method is used to link the states to the thrusters
 @return void
 @param states The states to link
 */
void ThrusterStateEffector::linkInStates(DynParamManager& states){
    this->hubSigma = states.getStateObject("hubSigma");
	this->hubOmega = states.getStateObject("hubOmega");
}

/*! This method allows the thruster state effector to register its state kappa with the dyn param manager */
void ThrusterStateEffector::registerStates(DynParamManager& states)
{
    // - Register the states associated with thruster - kappa
    this->kappaState = states.registerState(this->thrusterData.size(), 1, this->nameOfKappaState);
    Eigen::MatrixXd kappaInitMatrix(this->thrusterData.size(), 1);
    // Loop through all thrusters to initialize each state variable
    for (uint64_t i = 0; i < this->thrusterData.size(); i++) {
        kappaInitMatrix(i, 0) = this->kappaInit;
    }  
    this->kappaState->setState(kappaInitMatrix);

    return;
}

/*! This method is used to find the derivatives for the thruster stateEffector */
void ThrusterStateEffector::computeDerivatives(double integTime, Eigen::Vector3d rDDot_BN_N, Eigen::Vector3d omegaDot_BN_B, Eigen::Vector3d sigma_BN)
{
    std::vector<THRSimConfigMsgPayload>::iterator it;
    THROperationMsgPayload* ops;
    uint64_t i;
    double dt = 0.0;
    double zeta = 1.0;

    dt = integTime - prevFireTime;

    // - Compute Derivatives
    Eigen::MatrixXd kappaDot(this->thrusterData.size(), 1);

    // Loop through all thrusters to initialize each state variable
    for (it = this->thrusterData.begin(), i = 0; it != this->thrusterData.end(); it++, i++)
    {
        // Grab the thruster operations payload
        ops = &it->ThrustOps;

        //! - For each thruster check if the end time is greater than the current time, and if so thrust
        if ((ops->ThrusterEndTime - integTime) > 0.0 && ops->ThrustOnCmd > 0.0) {
            kappaDot(i, 0) = (1.0 - this->kappaState->state(i, 0)) / zeta;
        }
        else {
            kappaDot(i, 0) = -this->kappaState->state(i, 0) / zeta;
        }

        // Save the state to thruster ops
        ops->ThrustFactor = this->kappaState->state(i, 0);
    }
    this->kappaState->setDerivative(kappaDot);
   
    return;
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
    if (this->ReadInputs())
    {
        this->ConfigureThrustRequests(this->prevCommandTime);
    }
    this->writeOutputStateMessages(CurrentSimNanos);
}
