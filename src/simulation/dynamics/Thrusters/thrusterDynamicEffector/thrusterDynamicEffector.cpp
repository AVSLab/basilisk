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

#include <iostream>

#include "thrusterDynamicEffector.h"
#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/astroConstants.h"
#include "architecture/utilities/avsEigenSupport.h"

/*! The Constructor.*/
ThrusterDynamicEffector::ThrusterDynamicEffector()
: stepsInRamp(30)
, mDotTotal(0.0)
, fuelMass(-1.0)
, prevFireTime(0.0)
, prevCommandTime(0xFFFFFFFFFFFFFFFF)
{
    CallCounts = 0;
    forceExternal_B.fill(0.0);
    torqueExternalPntB_B.fill(0.0);
    forceExternal_N.fill(0.0);
    this->stateDerivContribution.resize(1);
    this->stateDerivContribution.setZero();
    return;
}

/*! The destructor. */
ThrusterDynamicEffector::~ThrusterDynamicEffector()
{
    for (long unsigned int c=0; c<this->thrusterOutMsgs.size(); c++) {
        free(this->thrusterOutMsgs.at(c));
    }
    return;
}


/*! This method is used to reset the module.

 */
void ThrusterDynamicEffector::Reset(uint64_t CurrentSimNanos)
{
    //! Clear out any currently firing thrusters and re-init cmd array
    NewThrustCmds.clear();
    NewThrustCmds.insert(this->NewThrustCmds.begin(), this->thrusterData.size(), 0.0);
    mDotTotal = 0.0;

    return;
}

/*! This method is here to write the output message structure into the specified
 message.
 @param CurrentClock The current time used for time-stamping the message

 */
void ThrusterDynamicEffector::writeOutputMessages(uint64_t CurrentClock)
{
    int idx = 0;
    std::vector<std::shared_ptr<THRSimConfig>>::iterator itp;
    std::shared_ptr<THRSimConfig> it;

    THROutputMsgPayload tmpThruster;
    for (itp = this->thrusterData.begin(); itp != this->thrusterData.end(); ++itp)
    {
        it = *itp;
        tmpThruster = this->thrusterOutMsgs[idx]->zeroMsgPayload;
        eigenVector3d2CArray(it->thrLoc_B, tmpThruster.thrusterLocation);
        eigenVector3d2CArray(it->thrDir_B, tmpThruster.thrusterDirection);
        tmpThruster.maxThrust = it->MaxThrust;
        tmpThruster.thrustFactor = it->ThrustOps.ThrustFactor;
        tmpThruster.thrustBlowDownFactor = it->ThrustOps.thrustBlowDownFactor;
        tmpThruster.ispBlowDownFactor = it->ThrustOps.ispBlowDownFactor;
        tmpThruster.thrustForce = v3Norm(it->ThrustOps.opThrustForce_B);
        v3Copy(it->ThrustOps.opThrustForce_B, tmpThruster.thrustForce_B);
        v3Copy(it->ThrustOps.opThrustTorquePntB_B, tmpThruster.thrustTorquePntB_B);

        this->thrusterOutMsgs[idx]->write(&tmpThruster, this->moduleID, CurrentClock);

        idx ++;
    }
}


/*! This method is used to read the incoming command message and set the
 associated command structure for operating the thrusters.

 */
bool ThrusterDynamicEffector::ReadInputs()
{
    uint64_t i;
    bool dataGood;

    if (this->cmdsInMsg.isLinked()) {
        // read the incoming command array
        this->incomingCmdBuffer = this->cmdsInMsg();
        dataGood = this->cmdsInMsg.isWritten();

        // Check if message has already been read, if stale return
        if(this->prevCommandTime==this->cmdsInMsg.timeWritten() || !dataGood) {
            return(false);
        }
        this->prevCommandTime = this->cmdsInMsg.timeWritten();
    } else {
        this->incomingCmdBuffer = this->cmdsInMsg.zeroMsgPayload;
        this->prevCommandTime = 0;
    }

    // Set the NewThrustCmds vector.  Using the data() method for raw speed
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

 @param currentTime The current simulation time converted to a double
 */
void ThrusterDynamicEffector::ConfigureThrustRequests(double currentTime)
{
    std::vector<double>::iterator CmdIt;
    size_t THIter = 0;
    // Iterate through the list of thruster commands that we read in.
    for(CmdIt = NewThrustCmds.begin(); CmdIt != NewThrustCmds.end(); CmdIt++)
    {
        if(*CmdIt >= this->thrusterData[THIter]->MinOnTime) // Check to see if we have met minimum for each thruster
        {
            // For each case where we are above the minimum firing request, reset the thruster
            this->thrusterData[THIter]->ThrustOps.ThrustOnCmd = *CmdIt;
            this->thrusterData[THIter]->ThrustOps.fireCounter += this->thrusterData[THIter]->ThrustOps.ThrustFactor > 0.0
            ? 0 : 1;
        }
        else
        {
            // Will ensure that thruster shuts down once this cmd expires
            this->thrusterData[THIter]->ThrustOps.ThrustOnCmd = this->thrusterData[THIter]->ThrustOps.ThrustFactor > 0.0
            ? *CmdIt : 0.0;
        }
        this->thrusterData[THIter]->ThrustOps.ThrusterStartTime = currentTime;
        this->thrusterData[THIter]->ThrustOps.PreviousIterTime = currentTime;
        this->thrusterData[THIter]->ThrustOps.ThrustOnRampTime = 0.0;
        this->thrusterData[THIter]->ThrustOps.ThrustOnSteadyTime = 0.0;
        this->thrusterData[THIter]->ThrustOps.ThrustOffRampTime = 0.0;
        // After we have assigned the firing to the internal thruster, zero the command request.
        *CmdIt = 0.0;
        THIter++;
    }

}

/*! This method is used to update the location and orientation of the thrusters
* at every UpdateState call when the thrusters are attached to a body other than
* the hub.

 */
void ThrusterDynamicEffector::UpdateThrusterProperties()
{
    // Save hub variables
    Eigen::Vector3d r_BN_N = (Eigen::Vector3d)*this->inertialPositionProperty;
    Eigen::Vector3d omega_BN_B = this->hubOmega->getState();
    Eigen::MRPd sigma_BN;
    sigma_BN = (Eigen::Vector3d)this->hubSigma->getState();
    Eigen::Matrix3d dcm_BN = (sigma_BN.toRotationMatrix()).transpose();

    // Define the variables related to which body the thruster is attached to. The F frame represents the platform body where the thruster attaches to
    Eigen::MRPd sigma_FN;
    Eigen::Matrix3d dcm_FN;
    Eigen::Vector3d omega_FN_F;
    Eigen::Vector3d r_FN_N;

    // Define the relative variables between the attached body and the hub
    Eigen::Matrix3d dcm_BF;

    // Loop through all thrusters
    std::vector<ReadFunctor<SCStatesMsgPayload>>::iterator it;
    int index;
    for (it = this->attachedBodyInMsgs.begin(), index = 0; it != this->attachedBodyInMsgs.end(); it++, index++)
    {
        // Check if the message is linked, and if so do the conversion
        if (it->isLinked() && it->isWritten())
        {
            // Save to buffer
            this->attachedBodyBuffer = this->attachedBodyInMsgs.at(index)();

            // Grab attached body variables
            sigma_FN = cArray2EigenMRPd(attachedBodyBuffer.sigma_BN);
            omega_FN_F = cArray2EigenVector3d(attachedBodyBuffer.omega_BN_B);
            r_FN_N = cArray2EigenVector3d(attachedBodyBuffer.r_BN_N);

            // Compute the DCM between the attached body and the hub
            dcm_FN = (sigma_FN.toRotationMatrix()).transpose();
            dcm_BF = dcm_BN * dcm_FN.transpose();

            // Populate the relative state structure
            this->bodyToHubInfo.at(index).r_FB_B = dcm_BN * (r_FN_N - r_BN_N);
            this->bodyToHubInfo.at(index).dcm_BF = dcm_BF;
            this->bodyToHubInfo.at(index).omega_FB_B = dcm_BF * omega_FN_F - omega_BN_B;
        }
    }
}

/*! This method is used to link the states to the thrusters

 @param states The states to link
 */
void ThrusterDynamicEffector::linkInStates(DynParamManager& states){
    this->hubSigma = states.getStateObject(this->stateNameOfSigma);
    this->hubOmega = states.getStateObject(this->stateNameOfOmega);

    this->inertialPositionProperty = states.getPropertyReference(this->propName_inertialPosition);

    for(const auto& thrusterConfig : this->thrusterData) {
        if (this->fuelMass < 0.0 &&
            (!thrusterConfig->thrBlowDownCoeff.empty() || !thrusterConfig->ispBlowDownCoeff.empty())) {
            bskLogger.bskLog(BSK_WARNING,"ThrusterDynamicEffector: blow down coefficients have been "
                                          "specified, but no fuel tank is attached.");
        }
    }
}

/*! This method computes the Forces on Torque on the Spacecraft Body.

 @param integTime Integration time
 @param timeStep Current integration time step used
 */
void ThrusterDynamicEffector::computeForceTorque(double integTime, double timeStep)
{
    // Save omega_BN_B
    Eigen::Vector3d omegaLocal_BN_B = this->hubOmega->getState();

    // Force and torque variables
    Eigen::Vector3d SingleThrusterForce;
    Eigen::Vector3d SingleThrusterTorque;
    double tmpThrustMag = 0;

    // Define the relative variables between the attached body and the hub
    Eigen::Vector3d thrustDirection_B;
    Eigen::Vector3d thrustLocation_B;

    // Expelled momentum variables
    Eigen::Matrix3d BMj;
    Eigen::Matrix3d	axesWeightMatrix;
    Eigen::Vector3d BM1, BM2, BM3;
    double mDotNozzle;

    // Zero out the structure force/torque for the thruster set
    this->forceExternal_B.setZero();
    this->forceExternal_N.setZero();
    this->torqueExternalPntB_B.setZero();
    double dt = integTime - prevFireTime;

	axesWeightMatrix << 2, 0, 0, 0, 1, 0, 0, 0, 1;

    // Loop variables
    std::shared_ptr<THRSimConfig> it;
    THROperation* ops;

    // Iterate through all of the thrusters to aggregate the force/torque in the system
    int index;
    for(index = 0; index < this->thrusterData.size(); ++index)
    {
        it = this->thrusterData[index];
        ops = &it->ThrustOps;

        // Compute the thruster properties wrt the hub (note that B refers to the F frame when extracting from the thruster info)
        thrustDirection_B = this->bodyToHubInfo.at(index).dcm_BF * it->thrDir_B;
        thrustLocation_B = this->bodyToHubInfo.at(index).r_FB_B + this->bodyToHubInfo.at(index).dcm_BF * it->thrLoc_B;

        // If the connected fuel tank is subject to blow down effects, update them here
        if (this->fuelMass >= 0.0 && (!it->thrBlowDownCoeff.empty() || !it->ispBlowDownCoeff.empty())) {
            this->computeBlowDownDecay(it);
        }

        // For each thruster see if the on-time is still valid and if so, call ComputeThrusterFire()
        if((ops->ThrustOnCmd + ops->ThrusterStartTime  - integTime) >= -dt*10E-10 &&
           ops->ThrustOnCmd > 0.0)
        {
            ComputeThrusterFire(it, integTime);
        }
        // If we are not actively firing, continue shutdown process for active thrusters
        else if(ops->ThrustFactor > 0.0)
        {
            ComputeThrusterShut(it, integTime);
        }

        // For each thruster, aggregate the current thrust direction into composite body force
        tmpThrustMag = it->MaxThrust * ops->ThrustFactor * ops->thrustBlowDownFactor;

        // Apply dispersion to magnitude
        tmpThrustMag *= (1. + it->thrusterMagDisp);
        SingleThrusterForce = tmpThrustMag * thrustDirection_B;
        this->forceExternal_B += SingleThrusterForce;

        // Compute the point B relative torque and aggregate into the composite body torque
        SingleThrusterTorque = thrustLocation_B.cross(SingleThrusterForce) + ops->ThrustFactor *
                               ops->thrustBlowDownFactor * it->MaxSwirlTorque * thrustDirection_B;
        this->torqueExternalPntB_B += SingleThrusterTorque;

		if (!it->updateOnly) {
			// Add the mass depletion force contribution
			mDotNozzle = 0.0;
			if (it->steadyIsp * ops->IspFactor * ops->ispBlowDownFactor > 0.0)
			{
				mDotNozzle = tmpThrustMag / (EARTH_GRAV * it->steadyIsp * ops->IspFactor * ops->ispBlowDownFactor);
			}
			this->forceExternal_B += 2 * mDotNozzle * (this->bodyToHubInfo.at(index).omega_FB_B +
                                     omegaLocal_BN_B).cross(thrustLocation_B);

			// Add the mass depletion torque contribution
			BM1 = thrustDirection_B;
			BM2 << -BM1(1), BM1(0), BM1(2);
			BM3 = BM1.cross(BM2);
			BMj.col(0) = BM1;
			BMj.col(1) = BM2;
			BMj.col(2) = BM3;
			this->torqueExternalPntB_B += mDotNozzle * (eigenTilde(thrustDirection_B) *
                                          eigenTilde(thrustDirection_B).transpose() + it->areaNozzle / (4 * M_PI) *
                                          BMj * axesWeightMatrix * BMj.transpose()) *
                                          (this->bodyToHubInfo.at(index).omega_FB_B + omegaLocal_BN_B);

		}
        // Save force and torque values for messages
        eigenVector3d2CArray(SingleThrusterForce, it->ThrustOps.opThrustForce_B);
        eigenVector3d2CArray(SingleThrusterTorque, it->ThrustOps.opThrustTorquePntB_B);
    }
    // Once all thrusters have been checked, update time-related variables for next evaluation
    prevFireTime = integTime;
}

/*! This method adds new thruster(s) to the thruster set.

 @param newThruster thruster sim config(s)
 */
void ThrusterDynamicEffector::addThruster(std::shared_ptr<THRSimConfig> newThruster)
{
    this->thrusterData.push_back(newThruster);

    // Create corresponding output message
    Message<THROutputMsgPayload>* msg;
    msg = new Message<THROutputMsgPayload>;
    this->thrusterOutMsgs.push_back(msg);

    // Push back an empty message
    ReadFunctor<SCStatesMsgPayload> emptyReadFunctor;
    this->attachedBodyInMsgs.push_back(emptyReadFunctor);

    // Add space for the conversion from body to hub and populate it with default values
    BodyToHubInfo attachedBodyToHub;
    attachedBodyToHub.dcm_BF.setIdentity();
    attachedBodyToHub.r_FB_B.setZero();
    attachedBodyToHub.omega_FB_B.setZero();
    this->bodyToHubInfo.push_back(attachedBodyToHub);
}

/*! This method adds new thruster(s) to the thruster set connected to a different body than the hub.

 @param newThruster thruster sim config(s)
 @param bodyStateMsg body states to which thruster(s) are attached
 */
void ThrusterDynamicEffector::addThruster(std::shared_ptr<THRSimConfig> newThruster, Message<SCStatesMsgPayload>* bodyStateMsg)
{
    this->thrusterData.push_back(newThruster);

    // Create corresponding output message
    Message<THROutputMsgPayload>* msg;
    msg = new Message<THROutputMsgPayload>;
    this->thrusterOutMsgs.push_back(msg);

    // Save the incoming body message
    this->attachedBodyInMsgs.push_back(bodyStateMsg->addSubscriber());

    // Add space for the conversion from body to hub and populate it with default values
    BodyToHubInfo attachedBodyToHub;
    attachedBodyToHub.dcm_BF.setIdentity();
    attachedBodyToHub.r_FB_B.setZero();
    attachedBodyToHub.omega_FB_B.setZero();
    this->bodyToHubInfo.push_back(attachedBodyToHub);

    return;
}


/*! This method is used to update the blow down effects to the thrust and/or Isp
* at every computeForceTorque call when the thrusters are attached to a fuel
* tank subject to blow down effects.

 */
void ThrusterDynamicEffector::computeBlowDownDecay(std::shared_ptr<THRSimConfig> currentThruster)
{
    THROperation *ops = &(currentThruster->ThrustOps);

    if (!currentThruster->thrBlowDownCoeff.empty()) {
        double thrustBlowDown = 0.0;
        double thrOrder = 1.0;
        for(auto thrCoeff = currentThruster->thrBlowDownCoeff.rbegin(); thrCoeff !=
                                               currentThruster->thrBlowDownCoeff.rend(); thrCoeff++) {
            thrustBlowDown += *thrCoeff * thrOrder;
            thrOrder *= fuelMass; // Fuel mass assigned in fuel tank's updateEffectorMassProps method
        }
        ops->thrustBlowDownFactor = std::clamp(thrustBlowDown / currentThruster->MaxThrust, double (0.0), double (1.0));
    }

    if (!currentThruster->ispBlowDownCoeff.empty()) {
        double ispBlowDown = 0.0;
        double ispOrder = 1.0;
        for (auto ispCoeff = currentThruster->ispBlowDownCoeff.rbegin(); ispCoeff !=
                                                currentThruster->ispBlowDownCoeff.rend(); ispCoeff++) {
            ispBlowDown += *ispCoeff * ispOrder;
            ispOrder *= fuelMass; // Fuel mass assigned in fuel tank's updateEffectorMassProps method
        }
        ops->ispBlowDownFactor = std::clamp(ispBlowDown / currentThruster->steadyIsp, double (0.0), double (1.0));
    }
}

/*! This method computes contributions to the fuel mass depletion. */
void ThrusterDynamicEffector::computeStateContribution(double integTime){

    std::vector<std::shared_ptr<THRSimConfig>>::iterator itp;
    std::shared_ptr<THRSimConfig> it;
    THROperation *ops;
    double mDotSingle=0.0;
    this->mDotTotal = 0.0;
	this->stateDerivContribution.setZero();
    // Iterate through all of the thrusters to aggregate the force/torque in the system
    for(itp = this->thrusterData.begin(); itp != this->thrusterData.end(); itp++)
    {
        it = *itp;
        ops = &it->ThrustOps;
        mDotSingle = 0.0;
        if(it->steadyIsp * ops->IspFactor * ops->ispBlowDownFactor > 0.0)
        {
            mDotSingle = it->MaxThrust * ops->ThrustFactor * ops->thrustBlowDownFactor / (EARTH_GRAV * it->steadyIsp * ops->IspFactor * ops->ispBlowDownFactor);
        }
        this->mDotTotal += mDotSingle;
    }
    this->stateDerivContribution(0) = this->mDotTotal;

    return;
}


/*! This method is used to get the current force for a thruster firing.  It uses
 the configuration data associated with a given thruster and the current clock
 time to determine what state and force the thruster should be in.

 @param CurrentThruster Pointer to the configuration data for a given thruster
 @param currentTime The current simulation clock time converted to a double
 */
void ThrusterDynamicEffector::ComputeThrusterFire(std::shared_ptr<THRSimConfig> CurrentThruster,
                                                  double currentTime)
{
    std::vector<THRTimePair>::iterator it;
    THROperation *ops = &(CurrentThruster->ThrustOps);
    // Set the current ramp time for the thruster firing
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

    // Iterate through the on-ramp for the thruster data to find where we are in ramp
    for(it = CurrentThruster->ThrusterOnRamp.begin();
        it != CurrentThruster->ThrusterOnRamp.end(); it++)
    {
        // If the current on-time is less than the ramp delta, set that ramp thrust factor
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
    // If we did not find the current time in the on-ramp, then we are at steady-state

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

 @param CurrentThruster Pointer to the configuration data for a given thruster
 @param currentTime The current simulation clock time converted to a double
 */
void ThrusterDynamicEffector::ComputeThrusterShut(std::shared_ptr<THRSimConfig> CurrentThruster,
                                                  double currentTime)
{
    std::vector<THRTimePair>::iterator it;
    THROperation *ops = &(CurrentThruster->ThrustOps);

    // Set the current off-ramp time based on the previous clock time and now
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
    // Iterate through the off-ramp to find the place where we are in the shutdown ramp
    for(it = CurrentThruster->ThrusterOffRamp.begin();
        it != CurrentThruster->ThrusterOffRamp.end(); it++)
    {
        // Once we find the location in the off-ramp, set that thrust factor to current
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
    // If we did not find the location in the off-ramp, we've reached the end state and zero thrust
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
double ThrusterDynamicEffector::thrFactorToTime(std::shared_ptr<THRSimConfig> thrData,
                                                std::vector<THRTimePair> *thrRamp)
{
    std::vector<THRTimePair>::iterator it;
    // Grab the last element in the ramp and determine if it goes up or down
    it = thrRamp->end();
    it--;
    double rampTime = it->TimeDelta;
    double rampDirection = std::copysign(1.0,
                                         it->ThrustFactor - thrData->ThrustOps.ThrustFactor);

    // Initialize the time computation functiosn based on ramp direction
    double prevValidThrFactor = rampDirection < 0 ? 1.0 : 0.0;
    double prevValidDelta = 0.0;
    for(it=thrRamp->begin(); it!=thrRamp->end(); it++)
    {
        // Determine if we haven't reached the right place in the ramp
        bool pointCheck = rampDirection > 0 ?
        it->ThrustFactor <= thrData->ThrustOps.ThrustFactor :
        it->ThrustFactor >= thrData->ThrustOps.ThrustFactor;
        // If we have not located the location in the ramp, continue
        if(pointCheck)
        {
            prevValidThrFactor = it->ThrustFactor;
            prevValidDelta = it->TimeDelta;
            continue;
        }

        // Linearly interpolate between the points, check for numerical garbage, and return clean interpolation
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

 @param CurrentSimNanos The current simulation time in nanoseconds
 */
void ThrusterDynamicEffector::UpdateState(uint64_t CurrentSimNanos)
{
    // Read the inputs and then call ConfigureThrustRequests to set up dynamics
    if(this->ReadInputs())
    {
        this->ConfigureThrustRequests(this->prevCommandTime*1.0E-9);
    }
    this->UpdateThrusterProperties();
    this->writeOutputMessages(CurrentSimNanos);
}
