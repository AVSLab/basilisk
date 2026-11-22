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

#include "architecture/messaging/ownedMessage.h"
#include "architecture/utilities/astroConstants.h"
#include "architecture/utilities/avsEigenSupport.h"
#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/macroDefinitions.h"
#include "thrusterStateEffector.h"
#include <cmath>

namespace {
// The nozzle load reported in telemetry excludes the separate expelled-momentum terms applied to the hub.
struct NozzleLoad
{
    Eigen::Vector3d direction_B;
    Eigen::Vector3d location_B;
    Eigen::Vector3d force_B;
    Eigen::Vector3d torquePntB_B;
};

NozzleLoad nozzleLoad(const THRSimConfig& nozzle, const BodyToHubInfo& mount, double factor)
{
    NozzleLoad load;
    load.direction_B = mount.dcm_BF * nozzle.thrDir_B;
    load.location_B = mount.r_FB_B + mount.dcm_BF * nozzle.thrLoc_B;
    load.force_B = nozzle.MaxThrust * factor * (1.0 + nozzle.thrusterMagDisp) * load.direction_B;
    load.torquePntB_B = load.location_B.cross(load.force_B) + factor * nozzle.MaxSwirlTorque * load.direction_B;
    return load;
}
}

/*! The Constructor.*/
ThrusterStateEffector::ThrusterStateEffector()
{
    // - zero the mass props and mass prop rates contributions
    this->effProps.mEff = 0.0;
    this->effProps.rEff_CB_B.fill(0.0);
    this->effProps.IEffPntB_B.fill(0.0);
    this->effProps.rEffPrime_CB_B.fill(0.0);
    this->effProps.IEffPrimePntB_B.fill(0.0);
    this->r_PcP_P.setZero();

    // initialize the state derivative contribution for mass rate
    this->stateDerivContribution.resize(1);
    this->stateDerivContribution.setZero();

    // initialize internal variables
    CallCounts = 0;
    this->prevCommandTime = -1.0; // initialize to a negative number to allow an onTime command at t=0
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
ThrusterStateEffector::~ThrusterStateEffector() = default;

/*! This method is used to reset the module.
 *
 * @param[in] CurrentSimNanos [ns] Current simulation time.
 */
void ThrusterStateEffector::Reset(uint64_t CurrentSimNanos [[maybe_unused]])
{
    this->validateConfiguration();
    // Clear out any currently firing thrusters and re-init cmd array
    this->NewThrustCmds.clear();
    this->NewThrustCmds.insert(this->NewThrustCmds.begin(), this->thrusterData.size(), 0.0);

    // Reset the mas flow value
    this->mDotTotal = 0.0;

    return;
}

/*! @brief Validate configuration dimensions before initialization or command processing. */
void ThrusterStateEffector::validateConfiguration()
{
    this->validateRegisteredCount();
    if (this->cmdsInMsg.isLinked() && this->thrusterData.size() > MAX_EFF_CNT) {
        this->bskLogger.bskError(
            "ThrusterStateEffector: a linked cmdsInMsg supports at most MAX_EFF_CNT (%d) thrusters.", MAX_EFF_CNT);
    }
    if (this->kappaInit.size() != this->thrusterData.size()) {
        this->bskLogger.bskError("ThrusterStateEffector: kappaInit must contain one entry per thruster.");
    }
    for (double kappa : this->kappaInit) {
        if (!std::isfinite(kappa) || kappa < 0.0 || kappa > 1.0) {
            this->bskLogger.bskError("ThrusterStateEffector: kappaInit entries must be finite and between 0 and 1.");
        }
    }
}

/*! @brief Reject device-count changes without relying on a live parent state pointer. */
void ThrusterStateEffector::validateRegisteredCount()
{
    if (this->registeredThrusterCount && this->thrusterData.size() != *this->registeredThrusterCount) {
        this->bskLogger.bskError("ThrusterStateEffector: thruster count cannot change after state registration. "
                                 "Configure all thrusters before InitializeSimulation().");
    }
}

/*! @brief Read the incoming command message and populate the thruster command vector.
 * @return Whether a new command is available, or zero commands were supplied for an unlinked input.
 */
bool ThrusterStateEffector::ReadInputs()
{
    this->validateConfiguration();
    this->NewThrustCmds.resize(this->thrusterData.size(), 0.0); // [s]
    // Initialize local variables
    uint64_t i;
    bool dataGood;

    // Check if the message has been linked
    if (this->cmdsInMsg.isLinked()) {
        //! - Read the incoming command array
        this->incomingCmdBuffer = this->cmdsInMsg();
        dataGood = this->cmdsInMsg.isWritten();

        //! - Check if message has already been read, if so then stale return
        if (abs(this->prevCommandTime - static_cast<double>(this->cmdsInMsg.timeWritten()) * NANO2SEC) < 1E-9 ||
            !dataGood) {
            return (false);
        }
        this->prevCommandTime = static_cast<double>(this->cmdsInMsg.timeWritten()) * NANO2SEC;
    } else {
        this->incomingCmdBuffer = this->cmdsInMsg.zeroMsgPayload;
        this->prevCommandTime = 0.0;
        this->NewThrustCmds.assign(this->thrusterData.size(), 0.0); // [s]
        return true;
    }

    // Set the NewThrustCmds vector.  Using the data() method for raw speed
    double* CmdPtr;
    for (i = 0, CmdPtr = NewThrustCmds.data(); i < this->thrusterData.size(); CmdPtr++, i++) {
        *CmdPtr = this->incomingCmdBuffer.OnTimeRequest[i];
    }
    return (true);
}

/*! @brief Publish nozzle telemetry at the accepted spacecraft tick.
 * @param CurrentClock Scheduled publication tick for a standalone effector [ns].
 */
void ThrusterStateEffector::writeOutputStateMessages(uint64_t CurrentClock)
{
    this->validateRegisteredCount();
    // The effector can run before the spacecraft. A rejected step clears the frame epoch until retry succeeds.
    this->outputTimeNanos =
        this->inertialFrame ? this->inertialFrame->acceptedTimeNanos.value_or(this->outputTimeNanos) : CurrentClock;
    const auto epoch = this->inertialOutputEpoch(CurrentClock);
    if (this->kappaState && epoch) {
        // Sampling for publication must not change the mounts used by the dynamics between scheduled updates.
        this->sampleMounts(this->acceptedMounts, epoch);
    }
    size_t idx = 0;
    std::vector<std::shared_ptr<THRSimConfig>>::iterator itp;
    std::shared_ptr<THRSimConfig> it;

    THROutputMsgPayload tmpThruster;
    for (itp = this->thrusterData.begin(); itp != this->thrusterData.end(); ++itp) {
        it = *itp;
        tmpThruster = this->thrusterOutMsgs[idx]->zeroMsgPayload;
        eigenVector3d2CArray(it->thrLoc_B, tmpThruster.thrusterLocation);
        eigenVector3d2CArray(it->thrDir_B, tmpThruster.thrusterDirection);
        tmpThruster.maxThrust = it->MaxThrust;
        if (this->kappaState) {
            // The integrator may have accepted or restored a state different from its last derivative sample.
            tmpThruster.thrustFactor = this->kappaState->stateView()(static_cast<Eigen::Index>(idx), 0);
            const auto load = nozzleLoad(*it, this->acceptedMounts[idx], tmpThruster.thrustFactor);
            eigenVector3d2CArray(load.force_B, tmpThruster.thrustForce_B);
            eigenVector3d2CArray(load.torquePntB_B, tmpThruster.thrustTorquePntB_B);
        } else {
            tmpThruster.thrustFactor = it->ThrustOps.ThrustFactor;
            v3Copy(it->ThrustOps.opThrustForce_B, tmpThruster.thrustForce_B);
            v3Copy(it->ThrustOps.opThrustTorquePntB_B, tmpThruster.thrustTorquePntB_B);
        }
        tmpThruster.thrustForce = v3Norm(tmpThruster.thrustForce_B);

        this->thrusterOutMsgs[idx]->write(&tmpThruster, this->moduleID, this->outputTimeNanos);

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
    this->validateRegisteredCount();
    if (this->NewThrustCmds.size() > this->thrusterData.size()) {
        this->bskLogger.bskError("ThrusterStateEffector: NewThrustCmds must not exceed the thruster count.");
    }
    std::vector<double>::iterator CmdIt;
    size_t THIter = 0;
    //! - Iterate through the list of thruster commands that we read in.
    for (CmdIt = NewThrustCmds.begin(); CmdIt != NewThrustCmds.end(); CmdIt++) {
        if (*CmdIt >= this->thrusterData[THIter]->MinOnTime) /// - Check to see if we have met minimum for each thruster
        {
            //! - For each case where we are above the minimum firing request, reset the thruster
            this->thrusterData[THIter]->ThrustOps.ThrustOnCmd = *CmdIt;
            this->thrusterData[THIter]->ThrustOps.fireCounter +=
                this->thrusterData[THIter]->ThrustOps.ThrustFactor > 0.0 ? 0u : 1u;
        } else {
            //! - Will ensure that thruster shuts down once this cmd expires
            this->thrusterData[THIter]->ThrustOps.ThrustOnCmd =
                this->thrusterData[THIter]->ThrustOps.ThrustFactor > 1E-5 ? *CmdIt : 0.0;
        }
        this->thrusterData[THIter]->ThrustOps.ThrusterEndTime =
            this->prevCommandTime + this->thrusterData[THIter]->ThrustOps.ThrustOnCmd;
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
    this->validateRegisteredCount();
    if (this->inertialFrame && (!this->inertialFrame->acceptedTimeNanos || !this->inertialFrame->originReady)) {
        return; // Keep the sampled mount geometry while the inertial hub position is unavailable.
    }
    this->sampleMounts(this->bodyToHubInfo);
}

void ThrusterStateEffector::sampleMounts(std::vector<BodyToHubInfo>& mounts, std::optional<uint64_t> epoch)
{
    if (!this->hubSigma) {
        return;
    }
    // Save hub variables
    Eigen::Vector3d r_BN_N = (Eigen::Vector3d) * this->inertialPositionProperty;
    Eigen::Vector3d omega_BN_B = this->hubOmega->stateView();
    Eigen::MRPd sigma_BN(this->hubSigma->stateView().data());
    Eigen::Matrix3d dcm_BN = (sigma_BN.toRotationMatrix()).transpose();

    // Define the variables related to which body the thruster is attached to. The F frame represents the platform body
    // where the thruster attaches to
    Eigen::MRPd sigma_FN;
    Eigen::Matrix3d dcm_FN;
    Eigen::Vector3d omega_FN_F;
    Eigen::Vector3d r_FN_N;

    // Define the relative variables between the attached body and the hub
    Eigen::Matrix3d dcm_BF;

    // Loop through all thrusters
    std::vector<ReadFunctor<SCStatesMsgPayload>>::iterator it;
    size_t index;
    for (it = this->attachedBodyInMsgs.begin(), index = 0; it != this->attachedBodyInMsgs.end(); it++, index++) {
        // For telemetry, pair inertial poses from the same tick or retain the previous relative transform.
        if (it->isLinked() && it->isWritten() && (!epoch || it->timeWritten() == *epoch)) {
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
            mounts.at(index).r_FB_B = dcm_BN * (r_FN_N - r_BN_N);
            mounts.at(index).dcm_BF = dcm_BF;
            mounts.at(index).omega_FB_B = dcm_BF * omega_FN_F - omega_BN_B;
        }
    }
}

/*! @brief Add a thruster.
 *
 * @param[in] newThruster Thruster configuration to add.
 * @note Thrusters can only be added before state registration.
 */
void ThrusterStateEffector::addThruster(std::shared_ptr<THRSimConfig> newThruster)
{
    this->requireMutableTopology("ThrusterStateEffector::addThruster");
    this->thrusterData.push_back(newThruster);

    // Create corresponding output message
    addOwnedMessage(this->ownedThrusterOutMsgs, this->thrusterOutMsgs);

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

/*! @brief Add a thruster.
 *
 * @param[in] newThruster Thruster configuration to add.
 * @param[in] bodyStateMsg State message for the body carrying the thruster.
 * @note Thrusters can only be added before state registration.
 */
void ThrusterStateEffector::addThruster(std::shared_ptr<THRSimConfig> newThruster,
                                        Message<SCStatesMsgPayload>* bodyStateMsg)
{
    this->addThruster(newThruster, bodyStateMsg->addSubscriber());
}

void ThrusterStateEffector::addThruster(std::shared_ptr<THRSimConfig> newThruster,
                                        ReadFunctor<SCStatesMsgPayload> bodyStateReader)
{
    this->requireMutableTopology("ThrusterStateEffector::addThruster");
    this->thrusterData.push_back(newThruster);

    // Create corresponding output message
    addOwnedMessage(this->ownedThrusterOutMsgs, this->thrusterOutMsgs);

    // Set the initial condition
    double state = 0.0;
    this->kappaInit.push_back(state);

    // Save the incoming body message
    this->attachedBodyInMsgs.push_back(bodyStateReader);

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
void ThrusterStateEffector::linkInStates(DynParamManager& states)
{
    this->hubSigma = states.getStateObject(this->stateNameOfSigma);
    this->hubOmega = states.getStateObject(this->stateNameOfOmega);
    this->inertialPositionProperty = states.getPropertyReference(this->propName_inertialPosition);
}

/*! This method allows the thruster state effector to register its state kappa with the dyn param manager
 *
 * @param[in,out] states Dynamic parameter manager used to register states or properties.
 */
void ThrusterStateEffector::registerStates(DynParamManager& states)
{
    this->validateConfiguration();
    this->NewThrustCmds.resize(this->thrusterData.size(), 0.0); // [s]
    if (this->thrusterData.empty()) {
        this->registeredThrusterCount = 0;
        return;
    }
    // - Register the states associated with thruster - kappa
    this->kappaState = states.registerState((uint32_t)this->thrusterData.size(), 1, this->nameOfKappaState);
    Eigen::MatrixXd kappaInitMatrix(this->thrusterData.size(), 1);
    // Loop through all thrusters to initialize each state variable
    for (Eigen::Index i = 0; i < kappaInitMatrix.rows(); i++) {
        const size_t thrusterIndex = static_cast<size_t>(i);
        kappaInitMatrix(i, 0) = this->kappaInit[thrusterIndex];
    }
    this->kappaState->setState(kappaInitMatrix);
    this->acceptedMounts = this->bodyToHubInfo;
    this->registeredThrusterCount = this->thrusterData.size();

    return;
}

/*! This method is used to find the derivatives for the thruster stateEffector
 *
 * @param[in] integTime [s] Current integration time.
 * @param[in] rDDot_BN_N [m/s^2] Hub translational acceleration expressed in inertial-frame components.
 * @param[in] omegaDot_BN_B [rad/s^2] Hub angular acceleration expressed in body-frame components.
 * @param[in] sigma_BN Hub attitude relative to the inertial frame.
 */
void ThrusterStateEffector::computeDerivatives(double integTime,
                                               Eigen::Vector3d rDDot_BN_N [[maybe_unused]],
                                               Eigen::Vector3d omegaDot_BN_B [[maybe_unused]],
                                               Eigen::MRPd sigma_BN [[maybe_unused]])
{
    this->validateRegisteredCount();
    if (this->thrusterData.empty()) {
        return;
    }
    std::vector<std::shared_ptr<THRSimConfig>>::iterator itp;
    std::shared_ptr<THRSimConfig> it;
    THROperation* ops;
    Eigen::Index i;

    // - Compute Derivatives
    Eigen::MatrixXd kappaDot(this->thrusterData.size(), 1);
    const auto kappa = this->kappaState->stateView();

    // Loop through all thrusters to initialize each state variable
    for (itp = this->thrusterData.begin(), i = 0; itp != this->thrusterData.end(); itp++, i++) {
        it = *itp;
        // Grab the thruster operations payload
        ops = &it->ThrustOps;

        //! - For each thruster check if the end time is greater than the current time, and if so thrust
        if ((ops->ThrusterEndTime - integTime) >= 0.0 && ops->ThrustOnCmd > 0.0) {
            kappaDot(i, 0) = (1.0 - kappa(i, 0)) * it->cutoffFrequency;
        } else {
            kappaDot(i, 0) = -kappa(i, 0) * it->cutoffFrequency;
        }

        // Save the state to thruster ops
        ops->ThrustFactor = kappa(i, 0);
    }
    this->kappaState->setDerivative(kappaDot);

    return;
}

/*! @brief Calculate the force and torque exerted on the attached body.
 *
 * @param[in] integTime [s] Current integration time.
 * @param[in] omega_BN_B [rad/s] Hub angular velocity expressed in body-frame components.
 */
void ThrusterStateEffector::calcForceTorqueOnBody(double integTime [[maybe_unused]], Eigen::Vector3d omega_BN_B)
{
    this->validateRegisteredCount();
    // Save omega_BN_B
    Eigen::Vector3d omegaLocal_BN_B = omega_BN_B;

    // Expelled momentum variables
    Eigen::Matrix3d BMj;
    Eigen::Matrix3d axesWeightMatrix;
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
    size_t index;
    for (index = 0; index < this->thrusterData.size(); ++index) {
        // Save the thruster ops information
        it = this->thrusterData[index];
        ops = &it->ThrustOps;

        const auto load = nozzleLoad(*it, this->bodyToHubInfo[index], ops->ThrustFactor);
        this->forceOnBody_B += load.force_B;
        this->torqueOnBodyPntB_B += load.torquePntB_B;
        eigenVector3d2CArray(load.force_B, ops->opThrustForce_B);
        eigenVector3d2CArray(load.torquePntB_B, ops->opThrustTorquePntB_B);

        if (!it->updateOnly) {
            const auto& thrustDirection_B = load.direction_B;
            const auto& thrustLocation_B = load.location_B;
            //! - Add the mass depletion force contribution
            mDotNozzle = 0.0;
            if (it->steadyIsp * ops->ThrustFactor > 0.0) {
                mDotNozzle = it->MaxThrust * ops->ThrustFactor / (EARTH_GRAV * it->steadyIsp);
            }
            this->forceOnBody_B +=
                2 * mDotNozzle * (this->bodyToHubInfo.at(index).omega_FB_B + omegaLocal_BN_B).cross(thrustLocation_B);

            //! - Add the mass depletion torque contribution
            BM1 = thrustDirection_B;
            BM2 << -BM1(1), BM1(0), BM1(2);
            BM3 = BM1.cross(BM2);
            BMj.col(0) = BM1;
            BMj.col(1) = BM2;
            BMj.col(2) = BM3;
            this->torqueOnBodyPntB_B += mDotNozzle *
                                        (eigenTilde(thrustDirection_B) * eigenTilde(thrustDirection_B).transpose() +
                                         it->areaNozzle / (4 * M_PI) * BMj * axesWeightMatrix * BMj.transpose()) *
                                        (this->bodyToHubInfo.at(index).omega_FB_B + omegaLocal_BN_B);
        }
    }

    return;
}

/*! @brief Update the effector Backsubstitution contributions.
 *
 * @param[in] integTime [s] Current integration time.
 * @param[in,out] backSubContr Backsubstitution contributions.
 * @param[in] sigma_BN Hub attitude relative to the inertial frame.
 * @param[in] omega_BN_B [rad/s] Hub angular velocity expressed in body-frame components.
 * @param[in] g_N [m/s^2] Gravitational acceleration expressed in inertial-frame components.
 */
void ThrusterStateEffector::updateContributions(double integTime [[maybe_unused]],
                                                BackSubMatrices& backSubContr,
                                                Eigen::MRPd sigma_BN [[maybe_unused]],
                                                Eigen::Vector3d omega_BN_B [[maybe_unused]],
                                                Eigen::Vector3d g_N [[maybe_unused]])
{
    // Define the translational and rotational contributions from the computed force and torque
    backSubContr.vecTrans = this->forceOnBody_B;
    backSubContr.vecRot = this->torqueOnBodyPntB_B;
}

/*! This is the method for the thruster effector to add its contributions to the mass props and mass prop rates of the
 * vehicle
 *
 * @param[in] integTime [s] Current integration time.
 */
void ThrusterStateEffector::updateEffectorMassProps(double integTime [[maybe_unused]])
{
    this->validateRegisteredCount();

    std::vector<std::shared_ptr<THRSimConfig>>::iterator itp;
    std::shared_ptr<THRSimConfig> it;
    THROperation* ops;
    double mDotSingle = 0.0;
    this->mDotTotal = 0.0;
    this->stateDerivContribution.setZero();
    //! - Iterate through all of the thrusters to aggregate the mass flow rate in the system
    for (itp = this->thrusterData.begin(); itp != this->thrusterData.end(); itp++) {
        it = *itp;
        ops = &it->ThrustOps;
        mDotSingle = 0.0;
        if (it->steadyIsp * ops->ThrustFactor > 0.0) {
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
    if (this->ReadInputs()) {
        this->ConfigureThrustRequests();
    }
    this->UpdateThrusterProperties();
    this->writeOutputStateMessages(CurrentSimNanos);
}
