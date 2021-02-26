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

#include <cstring>
#include <iostream>
#include <cmath>

#include "thrusterDynamicEffector.h"
#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/astroConstants.h"
#include "architecture/utilities/macroDefinitions.h"
#include "architecture/utilities/avsEigenSupport.h"
#include <cstring>
#include <iostream>
#include <cmath>

/*! The Constructor.*/
ThrusterDynamicEffector::ThrusterDynamicEffector()
: stepsInRamp(30)
, prevFireTime(0.0)
, prevCommandTime(0xFFFFFFFFFFFFFFFF)
{
    CallCounts = 0;
    forceExternal_B.fill(0.0);
    torqueExternalPntB_B.fill(0.0);
    forceExternal_N.fill(0.0);
    this->stateDerivContribution.resize(1);
    this->stateDerivContribution.setZero();
    this->mDotTotal = 0.0;
    return;
}

/*! The destructor. */
ThrusterDynamicEffector::~ThrusterDynamicEffector()
{
    for (int c=0; c<this->thrusterOutMsgs.size(); c++) {
        free(this->thrusterOutMsgs.at(c));
    }
    return;
}


/*! This method is used to reset the module.
 @return void
 */
void ThrusterDynamicEffector::Reset(uint64_t CurrentSimNanos)
{
    //! - Clear out any currently firing thrusters and re-init cmd array
    NewThrustCmds.clear();
    NewThrustCmds.insert(this->NewThrustCmds.begin(), this->thrusterData.size(), 0.0);
    mDotTotal = 0.0;

    // check if input message has not been included
    if (!this->cmdsInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "thrusterDynamicEffector.cmdsInMsg was not linked.");
    }


    return;
}

/*! This method is here to write the output message structure into the specified
 message.
 @param CurrentClock The current time used for time-stamping the message
 @return void
 */
void ThrusterDynamicEffector::writeOutputMessages(uint64_t CurrentClock)
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
bool ThrusterDynamicEffector::ReadInputs()
{
    
    std::vector<double>::iterator CmdIt;
    uint64_t i;
    bool dataGood;

    //! - read the incoming command array
    this->incomingCmdBuffer = this->cmdsInMsg();
    dataGood = this->cmdsInMsg.isWritten();

    //! - Check if message has already been read, if stale return
    if(prevCommandTime==this->cmdsInMsg.timeWritten() || !dataGood) {
        return(false);
    }
    prevCommandTime = this->cmdsInMsg.timeWritten();
    
    //! - Set the NewThrustCmds vector.  Using the data() method for raw speed
    double *CmdPtr;
    for(i=0, CmdPtr = NewThrustCmds.data(); i < this->thrusterData.size();
        CmdPtr++, i++)
    {
        *CmdPtr = this->incomingCmdBuffer.OnTimeRequest[i];
    }
    return(true);
    
}

/*! This method is used to read the new commands vector and set the thruster
 firings appropriately.  It assumes that the ReadInputs method has already been
 run successfully.  It honors all previous thruster firings if they are still
 active.  Note that for unit testing purposes you can insert firings directly
 into NewThrustCmds.
 @return void
 @param currentTime The current simulation time converted to a double
 */
void ThrusterDynamicEffector::ConfigureThrustRequests(double currentTime)
{
    std::vector<THRSimConfigMsgPayload>::iterator it;
    std::vector<double>::iterator CmdIt;
    std::vector<THRTimePairMsgPayload>::iterator PairIt;
    //! - Iterate through the list of thruster commands that we read in.
    for(CmdIt = NewThrustCmds.begin(), it = this->thrusterData.begin();
        it != this->thrusterData.end(); it++, CmdIt++)
    {
        if(*CmdIt >= it->MinOnTime) /// - Check to see if we have met minimum for each thruster
        {
            //! - For each case where we are above the minimum firing request, reset the thruster
            it->ThrustOps.ThrustOnCmd = *CmdIt;
            it->ThrustOps.fireCounter += it->ThrustOps.ThrustFactor > 0.0
            ? 0 : 1;
        }
        else
        {
            //! - Will ensure that thruster shuts down once this cmd expires
            it->ThrustOps.ThrustOnCmd = it->ThrustOps.ThrustFactor > 0.0
            ? *CmdIt : 0.0;
        }
        it->ThrustOps.ThrusterStartTime = currentTime;
        it->ThrustOps.PreviousIterTime = currentTime;
        it->ThrustOps.ThrustOnRampTime = 0.0;
        it->ThrustOps.ThrustOnSteadyTime = 0.0;
        it->ThrustOps.ThrustOffRampTime = 0.0;
        //! After we have assigned the firing to the internal thruster, zero the command request.
        *CmdIt = 0.0;
    }
    
}

/*! This method is used to link the states to the thrusters
 @return void
 @param states The states to link
 */
void ThrusterDynamicEffector::linkInStates(DynParamManager& states){
    this->hubSigma = states.getStateObject("hubSigma");
	this->hubOmega = states.getStateObject("hubOmega");
}

/*! This method computes the Forces on Torque on the Spacecraft Body.
 @return void
 @param integTime Integration time
 */
void ThrusterDynamicEffector::computeForceTorque(double integTime){
    
    std::vector<THRSimConfigMsgPayload>::iterator it;
    THROperationMsgPayload *ops;
    Eigen::Vector3d SingleThrusterForce;
    Eigen::Vector3d SingleThrusterTorque;
    Eigen::Vector3d CoMRelPos;
	Eigen::Vector3d omegaLocal_BN_B;
	Eigen::Matrix3d BMj;
	Eigen::Matrix3d	axesWeightMatrix;
	Eigen::Vector3d BM1, BM2, BM3;
    double tmpThrustMag = 0;
    double dt = 0.0;
	double mDotNozzle;
    
    //! - Zero out the structure force/torque for the thruster set
    // MassProps are missing, so setting CoM to zero momentarily
    CoMRelPos.setZero();
    this->forceExternal_B.setZero();
    this->forceExternal_N.setZero();
    this->torqueExternalPntB_B.setZero();
    dt = integTime - prevFireTime;

	omegaLocal_BN_B = hubOmega->getState();
	axesWeightMatrix << 2, 0, 0, 0, 1, 0, 0, 0, 1;
    
    //! - Iterate through all of the thrusters to aggregate the force/torque in the system
    for(it = this->thrusterData.begin(); it != this->thrusterData.end(); it++)
    {
        ops = &it->ThrustOps;
        //! - For each thruster see if the on-time is still valid and if so, call ComputeThrusterFire()
        if((ops->ThrustOnCmd + ops->ThrusterStartTime  - integTime) >= -dt*10E-10 &&
           ops->ThrustOnCmd > 0.0)
        {
            ComputeThrusterFire(&(*it), integTime);
        }
        //! - If we are not actively firing, continue shutdown process for active thrusters
        else if(ops->ThrustFactor > 0.0)
        {
            ComputeThrusterShut(&(*it), integTime);
        }
        //! - For each thruster, aggregate the current thrust direction into composite body force
        tmpThrustMag = it->MaxThrust*ops->ThrustFactor;
        // Apply dispersion to magnitude
        tmpThrustMag *= (1. + it->thrusterMagDisp);
        SingleThrusterForce = it->thrDir_B*tmpThrustMag;
        this->forceExternal_B = SingleThrusterForce + forceExternal_B;
        
        //! - Compute the point B relative torque and aggregate into the composite body torque
        SingleThrusterTorque = it->thrLoc_B.cross(SingleThrusterForce);
        this->torqueExternalPntB_B = SingleThrusterTorque + torqueExternalPntB_B;

		if (!it->updateOnly) {
			//! - Add the mass depletion force contribution
			mDotNozzle = 0.0;
			if (it->steadyIsp * ops->IspFactor > 0.0)
			{
				mDotNozzle = it->MaxThrust*ops->ThrustFactor / (EARTH_GRAV *
					it->steadyIsp * ops->IspFactor);
			}
			this->forceExternal_B += 2 * mDotNozzle*omegaLocal_BN_B.cross(it->thrLoc_B);

			//! - Add the mass depletion torque contribution
			BM1 = it->thrDir_B;
			BM2 << -BM1(1), BM1(0), BM1(2);
			BM3 = BM1.cross(BM2);
			BMj.col(0) = BM1;
			BMj.col(1) = BM2;
			BMj.col(2) = BM3;
			this->torqueExternalPntB_B += mDotNozzle * (eigenTilde(it->thrDir_B)*eigenTilde(it->thrDir_B).transpose()
				+ it->areaNozzle / (4 * M_PI) * BMj*axesWeightMatrix*BMj.transpose())*omegaLocal_BN_B;

		}
        // - Save force and torque values for messages
        eigenVector3d2CArray(SingleThrusterForce, it->ThrustOps.opThrustForce_B);
        eigenVector3d2CArray(SingleThrusterTorque, it->ThrustOps.opThrustTorquePntB_B);
    }
    //! - Once all thrusters have been checked, update time-related variables for next evaluation
    prevFireTime = integTime;
}

void ThrusterDynamicEffector::addThruster(THRSimConfigMsgPayload *newThruster)
{
    this->thrusterData.push_back(*newThruster);

    /* create corresponding output message */
    Message<THROutputMsgPayload> *msg;
    msg = new Message<THROutputMsgPayload>;
    this->thrusterOutMsgs.push_back(msg);
}


void ThrusterDynamicEffector::computeStateContribution(double integTime){
    
    std::vector<THRSimConfigMsgPayload>::iterator it;
    THROperationMsgPayload *ops;
    double mDotSingle=0.0;
    this->mDotTotal = 0.0;
	this->stateDerivContribution.setZero();
    //! - Iterate through all of the thrusters to aggregate the force/torque in the system
    for(it = this->thrusterData.begin(); it != this->thrusterData.end(); it++)
    {
        ops = &it->ThrustOps;
        mDotSingle = 0.0;
        if(it->steadyIsp * ops->IspFactor > 0.0)
        {
            mDotSingle = it->MaxThrust*ops->ThrustFactor/(EARTH_GRAV *
                                                          it->steadyIsp * ops->IspFactor);
        }
        this->mDotTotal += mDotSingle;
        

        
    }
    this->stateDerivContribution(0) = this->mDotTotal;

    return;
}


/*! This method is used to get the current force for a thruster firing.  It uses
 the configuration data associated with a given thruster and the current clock
 time to determine what state and force the thruster should be in.
 @return void
 @param CurrentThruster Pointer to the configuration data for a given thruster
 @param currentTime The current simulation clock time converted to a double
 */
void ThrusterDynamicEffector::ComputeThrusterFire(THRSimConfigMsgPayload *CurrentThruster,
                                                  double currentTime)
{
    std::vector<THRTimePairMsgPayload>::iterator it;
    THROperationMsgPayload *ops = &(CurrentThruster->ThrustOps);
    //! - Set the current ramp time for the thruster firing
    if(ops->ThrustOnRampTime == 0.0 &&
       CurrentThruster->ThrusterOnRamp.size() > 0)
    {
        ops->ThrustOnRampTime = thrFactorToTime(CurrentThruster,
                                                &(CurrentThruster->ThrusterOnRamp));
    }
    double LocalOnRamp = (currentTime - ops->PreviousIterTime) +
    ops->ThrustOnRampTime;
    LocalOnRamp = LocalOnRamp >= 0.0 ? LocalOnRamp : 0.0;
    double prevValidThrFactor = 0.0;
    double prevValidIspFactor = 0.0;
    double prevValidDelta = 0.0;
    
    //! - Iterate through the on-ramp for the thruster data to find where we are in ramp
    for(it = CurrentThruster->ThrusterOnRamp.begin();
        it != CurrentThruster->ThrusterOnRamp.end(); it++)
    {
        //! - If the current on-time is less than the ramp delta, set that ramp thrust factor
        if(LocalOnRamp < it->TimeDelta)
        {
            ops->ThrustFactor = (it->ThrustFactor - prevValidThrFactor)/
            (it->TimeDelta - prevValidDelta) *
            (LocalOnRamp - prevValidDelta) + prevValidThrFactor;
            ops->IspFactor = (it->IspFactor - prevValidIspFactor)/
            (it->TimeDelta - prevValidDelta) *
            (LocalOnRamp - prevValidDelta) + prevValidIspFactor;
            ops->ThrustOnRampTime = LocalOnRamp;
            ops->totalOnTime += (currentTime - ops->PreviousIterTime);
            ops->PreviousIterTime = currentTime;
            return;
        }
        prevValidThrFactor = it->ThrustFactor;
        prevValidIspFactor = it->IspFactor;
        prevValidDelta = it->TimeDelta;
    }
    //! - If we did not find the current time in the on-ramp, then we are at steady-state
    
    ops->ThrustOnSteadyTime += (currentTime - ops->PreviousIterTime);
    ops->totalOnTime += (currentTime - ops->PreviousIterTime);
    ops->PreviousIterTime = currentTime;
    ops->ThrustFactor = ops->IspFactor = 1.0;
    ops->ThrustOffRampTime = 0.0;
}


/*! This method is used to go through the process of shutting down a thruster
 once it has been commanded off.  It uses the configuration data associated with
 a given thruster and the current clock time to turn off the thruster according
 to the ramp profile.
 @return void
 @param CurrentThruster Pointer to the configuration data for a given thruster
 @param currentTime The current simulation clock time converted to a double
 */
void ThrusterDynamicEffector::ComputeThrusterShut(THRSimConfigMsgPayload *CurrentThruster,
                                                  double currentTime)
{
    std::vector<THRTimePairMsgPayload>::iterator it;
    THROperationMsgPayload *ops = &(CurrentThruster->ThrustOps);
    
    //! - Set the current off-ramp time based on the previous clock time and now
    if(ops->ThrustOffRampTime == 0.0 &&
       CurrentThruster->ThrusterOffRamp.size() > 0)
    {
        ops->ThrustOffRampTime = thrFactorToTime(CurrentThruster,
                                                 &(CurrentThruster->ThrusterOffRamp));
    }
    double LocalOffRamp = (currentTime - ops->PreviousIterTime) +
    ops->ThrustOffRampTime;
    LocalOffRamp = LocalOffRamp >= 0.0 ? LocalOffRamp : 0.0;
    double prevValidThrFactor = 1.0;
    double prevValidIspFactor = 1.0;
    double prevValidDelta = 0.0;
    //! - Iterate through the off-ramp to find the place where we are in the shutdown ramp
    for(it = CurrentThruster->ThrusterOffRamp.begin();
        it != CurrentThruster->ThrusterOffRamp.end(); it++)
    {
        //! - Once we find the location in the off-ramp, set that thrust factor to current
        if(LocalOffRamp < it->TimeDelta)
        {
            ops->ThrustFactor = (it->ThrustFactor - prevValidThrFactor)/
            (it->TimeDelta - prevValidDelta) *
            (LocalOffRamp - prevValidDelta) + prevValidThrFactor;
            ops->IspFactor = (it->IspFactor - prevValidIspFactor)/
            (it->TimeDelta - prevValidDelta) *
            (LocalOffRamp - prevValidDelta) + prevValidIspFactor;
            ops->ThrustOffRampTime = LocalOffRamp;
            ops->PreviousIterTime = currentTime;
            return;
        }
        prevValidThrFactor = it->ThrustFactor;
        prevValidIspFactor = it->IspFactor;
        prevValidDelta = it->TimeDelta;
    }
    //! - If we did not find the location in the off-ramp, we've reached the end state and zero thrust
    ops->ThrustFactor = ops->IspFactor = 0.0;
    ops->ThrustOnRampTime = 0.0;
}

/*! This method finds the location in the time in the specified ramp that
 corresponds to the current thruster thrust factor.  It is designed to
 initialize the ramp-up and ramp-down effects to the appropriate point in
 their respective ramps based on the initial force
 @return double The time in the ramp associated with the thrust factor
 @param thrData The data for the thruster that we are currently firing
 @param thrRamp This just allows us to avoid switching to figure out which ramp
 */
double ThrusterDynamicEffector::thrFactorToTime(THRSimConfigMsgPayload *thrData,
                                                std::vector<THRTimePairMsgPayload> *thrRamp)
{
    std::vector<THRTimePairMsgPayload>::iterator it;
    //! - Grab the last element in the ramp and determine if it goes up or down
    it = thrRamp->end();
    it--;
    double rampTime = it->TimeDelta;
    double rampDirection = std::copysign(1.0,
                                         it->ThrustFactor - thrData->ThrustOps.ThrustFactor);
    
    //! - Initialize the time computation functiosn based on ramp direction
    double prevValidThrFactor = rampDirection < 0 ? 1.0 : 0.0;
    double prevValidDelta = 0.0;
    for(it=thrRamp->begin(); it!=thrRamp->end(); it++)
    {
        //! - Determine if we haven't reached the right place in the ramp
        bool pointCheck = rampDirection > 0 ?
        it->ThrustFactor <= thrData->ThrustOps.ThrustFactor :
        it->ThrustFactor >= thrData->ThrustOps.ThrustFactor;
        //! - If we have not located the location in the ramp, continue
        if(pointCheck)
        {
            prevValidThrFactor = it->ThrustFactor;
            prevValidDelta = it->TimeDelta;
            continue;
        }
        
        //! - Linearly interpolate between the points, check for numerical garbage, and return clean interpolation
        rampTime = (it->TimeDelta - prevValidDelta)/(it->ThrustFactor -
                                                     prevValidThrFactor) * (thrData->ThrustOps.ThrustFactor -
                                                                            prevValidThrFactor) + prevValidDelta;
        rampTime = rampTime < 0.0 ? 0.0 : rampTime;
        break;
    }
    
    return(rampTime);
}


/*! This method is the main cyclical call for the scheduled part of the thruster
 dynamics model.  It reads the current commands array and sets the thruster
 configuration data based on that incoming command set.  Note that the main
 dynamical method (ComputeDynamics()) is not called here and is intended to be
 called from the dynamics plant in the system
 @return void
 @param CurrentSimNanos The current simulation time in nanoseconds
 */
void ThrusterDynamicEffector::UpdateState(uint64_t CurrentSimNanos)
{
    //! - Read the inputs and then call ConfigureThrustRequests to set up dynamics
    if(this->ReadInputs())
    {
        this->ConfigureThrustRequests(prevCommandTime*1.0E-9);
    }
    this->writeOutputMessages(CurrentSimNanos);
}
