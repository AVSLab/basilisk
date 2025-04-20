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

#include <iostream>

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

    // initialize the state derivative contribution for mass rate
    this->stateDerivContribution.resize(1);
    this->stateDerivContribution.setZero();

    // initialize internal variables
    CallCounts = 0;
    this->prevCommandTime = -1.0;  // initialize to a negative number to allow an onTime command at t=0
    this->mDotTotal = 0.0;
    this->nameOfKappaState = "kappaState" + std::to_string(this->effectorID);
    this->effectorID++;

    // clear all vectors
    this->thrusterData.clear();
    this->thrusterOutMsgs.clear();
    this->NewThrustCmds.clear();

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

 */
void ThrusterStateEffector::Reset(uint64_t CurrentSimNanos)
{
    // Clear out any currently firing thrusters and re-init cmd array
    this->NewThrustCmds.clear();
    this->NewThrustCmds.insert(this->NewThrustCmds.begin(), this->thrusterData.size(), 0.0);

    // Reset the mas flow value
    this->mDotTotal = 0.0;

    return;
}

/*! This method is used to read the incoming command message and set the
 associated command structure for operating the thrusters.

 */
bool ThrusterStateEffector::ReadInputs()
{
    // Initialize local variables
    uint64_t i;
    bool dataGood;

    // Check if the message has been linked
    if (this->cmdsInMsg.isLinked()) {
        //! - Read the incoming command array
        this->incomingCmdBuffer = this->cmdsInMsg();
        dataGood = this->cmdsInMsg.isWritten();

        //! - Check if message has already been read, if so then stale return
        if(abs(this->prevCommandTime - this->cmdsInMsg.timeWritten() * NANO2SEC) < 1E-9 || !dataGood) {
            return(false);
        }
        this->prevCommandTime = this->cmdsInMsg.timeWritten() * NANO2SEC;
    } else {
        this->incomingCmdBuffer = this->cmdsInMsg.zeroMsgPayload;
        this->prevCommandTime = 0.0;
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

/*! This method is here to write the output message structure into the specified
 message.
 @param CurrentClock The current time used for time-stamping the message

 */
void ThrusterStateEffector::writeOutputStateMessages(uint64_t CurrentClock)
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

 */
void ThrusterStateEffector::ConfigureThrustRequests()
{
    std::vector<double>::iterator CmdIt;
    size_t THIter = 0;
    //! - Iterate through the list of thruster commands that we read in.
    for(CmdIt = NewThrustCmds.begin(); CmdIt != NewThrustCmds.end(); CmdIt++)
    {
        if (*CmdIt >= this->thrusterData[THIter]->MinOnTime) /// - Check to see if we have met minimum for each thruster
        {
            //! - For each case where we are above the minimum firing request, reset the thruster
            this->thrusterData[THIter]->ThrustOps.ThrustOnCmd = *CmdIt;
            this->thrusterData[THIter]->ThrustOps.fireCounter += this->thrusterData[THIter]->ThrustOps.ThrustFactor > 0.0
                ? 0 : 1;
        }
        else
        {
            //! - Will ensure that thruster shuts down once this cmd expires
            this->thrusterData[THIter]->ThrustOps.ThrustOnCmd = this->thrusterData[THIter]->ThrustOps.ThrustFactor > 1E-5
                ? *CmdIt : 0.0;
        }
        this->thrusterData[THIter]->ThrustOps.ThrusterEndTime = this->prevCommandTime + this->thrusterData[THIter]->ThrustOps.ThrustOnCmd;
        //! After we have assigned the firing to the internal thruster, zero the command request.
        *CmdIt = 0.0;
        THIter++;
    }

    return;
}

/*! This method is used to update the location and orientation of the thrusters
* at every UpdateState call when the thrusters are attached to a body other than
* the hub.

 */
void ThrusterStateEffector::UpdateThrusterProperties()
{
    // Save hub variables
    Eigen::Vector3d r_BN_N = (Eigen::Vector3d)*this->inertialPositionProperty;
    Eigen::Vector3d omega_BN_B = this->hubOmega->getState();
    Eigen::MRPd sigma_BN;
    sigma_BN = (Eigen::Vector3d) this->hubSigma->getState();
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

void ThrusterStateEffector::addThruster(std::shared_ptr<THRSimConfig> newThruster)
{
    this->thrusterData.push_back(newThruster);

    // Create corresponding output message
    Message<THROutputMsgPayload>* msg;
    msg = new Message<THROutputMsgPayload>;
    this->thrusterOutMsgs.push_back(msg);

    // Set the initial condition
    double state = 0.0;
    this->kappaInit.push_back(state);

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

void ThrusterStateEffector::addThruster(std::shared_ptr<THRSimConfig> newThruster, Message<SCStatesMsgPayload>* bodyStateMsg)
{
    this->thrusterData.push_back(newThruster);

    // Create corresponding output message
    Message<THROutputMsgPayload>* msg;
    msg = new Message<THROutputMsgPayload>;
    this->thrusterOutMsgs.push_back(msg);

    // Set the initial condition
    double state = 0.0;
    this->kappaInit.push_back(state);

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

/*! This method is used to link the states to the thrusters

 @param states The states to link
 */
void ThrusterStateEffector::linkInStates(DynParamManager& states){
    this->hubSigma = states.getStateObject(this->stateNameOfSigma);
    this->hubOmega = states.getStateObject(this->stateNameOfOmega);
    this->inertialPositionProperty = states.getPropertyReference(this->nameOfSpacecraftAttachedTo + this->propName_inertialPosition);
}

/*! This method allows the thruster state effector to register its state kappa with the dyn param manager */
void ThrusterStateEffector::registerStates(DynParamManager& states)
{
    // - Register the states associated with thruster - kappa
    this->kappaState = states.registerState((uint32_t) this->thrusterData.size(), 1, this->nameOfKappaState);
    Eigen::MatrixXd kappaInitMatrix(this->thrusterData.size(), 1);
    // Loop through all thrusters to initialize each state variable
    for (uint64_t i = 0; i < this->thrusterData.size(); i++) {
        // Make sure that the thruster state is between 0 and 1
        if (this->kappaInit[i] < 0.0 || this->kappaInit[i] > 1.0) {
            bskLogger.bskLog(BSK_ERROR, "thrusterStateEffector: the initial condition for the thrust factor must be between 0 and 1. Setting it to 0.");
            this->kappaInit[i] = 0.0;
        }
        kappaInitMatrix(i, 0) = this->kappaInit[i];
    }
    this->kappaState->setState(kappaInitMatrix);

    return;
}

/*! This method is used to find the derivatives for the thruster stateEffector */
void ThrusterStateEffector::computeDerivatives(double integTime, Eigen::Vector3d rDDot_BN_N, Eigen::Vector3d omegaDot_BN_B, Eigen::Vector3d sigma_BN)
{
    std::vector<std::shared_ptr<THRSimConfig>>::iterator itp;
    std::shared_ptr<THRSimConfig> it;
    THROperation* ops;
    uint64_t i;

    // - Compute Derivatives
    Eigen::MatrixXd kappaDot(this->thrusterData.size(), 1);

    // Loop through all thrusters to initialize each state variable
    for (itp = this->thrusterData.begin(), i = 0; itp != this->thrusterData.end(); itp++, i++)
    {
        it = *itp;
        // Grab the thruster operations payload
        ops = &it->ThrustOps;

        //! - For each thruster check if the end time is greater than the current time, and if so thrust
        if ((ops->ThrusterEndTime - integTime) >= 0.0 && ops->ThrustOnCmd > 0.0) {
            kappaDot(i, 0) = (1.0 - this->kappaState->state(i, 0)) * it->cutoffFrequency;
        }
        else {
            kappaDot(i, 0) = -this->kappaState->state(i, 0) * it->cutoffFrequency;
        }

        // Save the state to thruster ops
        ops->ThrustFactor = this->kappaState->state(i, 0);
    }
    this->kappaState->setDerivative(kappaDot);

    return;
}

void ThrusterStateEffector::calcForceTorqueOnBody(double integTime, Eigen::Vector3d omega_BN_B)
{
    // Save omega_BN_B
    Eigen::Vector3d omegaLocal_BN_B = omega_BN_B;

    // Force and torque variables
    Eigen::Vector3d SingleThrusterForce;
    Eigen::Vector3d SingleThrusterTorque;
    double tmpThrustMag = 0;

    // Auxiliary variables to convert direction and location from F to B
    Eigen::Vector3d thrustDirection_B;
    Eigen::Vector3d thrustLocation_B;

    // Expelled momentum variables
    Eigen::Matrix3d BMj;
    Eigen::Matrix3d	axesWeightMatrix;
    Eigen::Vector3d BM1, BM2, BM3;
    double mDotNozzle;

    //! - Zero out the structure force/torque for the thruster set
    // MassProps are missing, so setting CoM to zero momentarily
    this->forceOnBody_B.setZero();
    this->torqueOnBodyPntB_B.setZero();
    this->torqueOnBodyPntC_B.setZero();

    axesWeightMatrix << 2, 0, 0, 0, 1, 0, 0, 0, 1;

    // Loop variables
    std::shared_ptr<THRSimConfig> it;
    THROperation* ops;

    //! - Iterate through all of the thrusters to aggregate the force/torque in the system
    int index;
    for(index = 0; index < this->thrusterData.size(); ++index)
    {
        // Save the thruster ops information
        it = this->thrusterData[index];
        ops = &it->ThrustOps;

        // Compute the thruster properties wrt the hub (note that B refers to the F frame when extracting from the thruster info)
        thrustDirection_B = this->bodyToHubInfo.at(index).dcm_BF * it->thrDir_B;
        thrustLocation_B = this->bodyToHubInfo.at(index).r_FB_B + this->bodyToHubInfo.at(index).dcm_BF * it->thrLoc_B;

        //! - For each thruster, aggregate the current thrust direction into composite body force
        tmpThrustMag = it->MaxThrust * ops->ThrustFactor;
        // Apply dispersion to magnitude
        tmpThrustMag *= (1. + it->thrusterMagDisp);
        SingleThrusterForce = tmpThrustMag * thrustDirection_B;
        this->forceOnBody_B += SingleThrusterForce;

        //! - Compute the point B relative torque and aggregate into the composite body torque
        SingleThrusterTorque = thrustLocation_B.cross(SingleThrusterForce) + ops->ThrustFactor * it->MaxSwirlTorque * thrustDirection_B;
        this->torqueOnBodyPntB_B += SingleThrusterTorque;

        if (!it->updateOnly) {
            //! - Add the mass depletion force contribution
            mDotNozzle = 0.0;
            if (it->steadyIsp * ops->ThrustFactor > 0.0)
            {
                mDotNozzle = it->MaxThrust * ops->ThrustFactor / (EARTH_GRAV * it->steadyIsp);
            }
            this->forceOnBody_B += 2 * mDotNozzle * (this->bodyToHubInfo.at(index).omega_FB_B + omegaLocal_BN_B).cross(thrustLocation_B);

            //! - Add the mass depletion torque contribution
            BM1 = thrustDirection_B;
            BM2 << -BM1(1), BM1(0), BM1(2);
            BM3 = BM1.cross(BM2);
            BMj.col(0) = BM1;
            BMj.col(1) = BM2;
            BMj.col(2) = BM3;
            this->torqueOnBodyPntB_B += mDotNozzle * (eigenTilde(thrustDirection_B) * eigenTilde(thrustDirection_B).transpose()
                + it->areaNozzle / (4 * M_PI) * BMj * axesWeightMatrix * BMj.transpose()) * (this->bodyToHubInfo.at(index).omega_FB_B + omegaLocal_BN_B);

        }
        // - Save force and torque values for messages
        eigenVector3d2CArray(SingleThrusterForce, it->ThrustOps.opThrustForce_B);
        eigenVector3d2CArray(SingleThrusterTorque, it->ThrustOps.opThrustTorquePntB_B);
    }

    return;
}

void ThrusterStateEffector::updateContributions(double integTime, BackSubMatrices& backSubContr, Eigen::Vector3d sigma_BN, Eigen::Vector3d omega_BN_B, Eigen::Vector3d g_N)
{
    // Define the translational and rotational contributions from the computed force and torque
    backSubContr.vecTrans = this->forceOnBody_B;
    backSubContr.vecRot = this->torqueOnBodyPntB_B;

}

/*! This is the method for the thruster effector to add its contributions to the mass props and mass prop rates of the vehicle */
void ThrusterStateEffector::updateEffectorMassProps(double integTime) {

    std::vector<std::shared_ptr<THRSimConfig>>::iterator itp;
    std::shared_ptr<THRSimConfig> it;
    THROperation* ops;
    double mDotSingle = 0.0;
    this->mDotTotal = 0.0;
    this->stateDerivContribution.setZero();
    //! - Iterate through all of the thrusters to aggregate the mass flow rate in the system
    for (itp = this->thrusterData.begin(); itp != this->thrusterData.end(); itp++)
    {
        it = *itp;
        ops = &it->ThrustOps;
        mDotSingle = 0.0;
        if (it->steadyIsp * ops->ThrustFactor > 0.0)
        {
            mDotSingle = it->MaxThrust * ops->ThrustFactor / (EARTH_GRAV * it->steadyIsp);
        }
        this->mDotTotal += mDotSingle;
    }
    this->stateDerivContribution(0) = this->mDotTotal;

    return;

}

/*! This method is the main cyclical call for the scheduled part of the thruster
 dynamics model.  It reads the current commands array and sets the thruster
 configuration data based on that incoming command set.  Note that the main
 dynamical method (ComputeDynamics()) is not called here and is intended to be
 called from the dynamics plant in the system

 @param CurrentSimNanos The current simulation time in nanoseconds
 */
void ThrusterStateEffector::UpdateState(uint64_t CurrentSimNanos)
{
    //! - Read the inputs and then call ConfigureThrustRequests to set up dynamics
    if (this->ReadInputs())
    {
        this->ConfigureThrustRequests();
    }
    this->UpdateThrusterProperties();
    this->writeOutputStateMessages(CurrentSimNanos);
}
