/*
 ISC License

 Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#include "simulation/deviceInterface/jointArrayRefProfiler/jointArrayRefProfiler.h"

void JointArrayRefProfiler::Reset(uint64_t CurrentSimNanos)
{
    // check that required input messages are connected
    if (this->jointStatesInMsgs.empty()) {
        bskLogger.bskError("JointArrayRefProfiler.jointStatesInMsgs vector is empty.");
    } else {
        if (this->jointStatesInMsgs.size() != static_cast<std::size_t>(this->numHingedJoints)) {
            bskLogger.bskError("JointArrayRefProfiler.jointStatesInMsgs size does not match numHingedJoints.");
        }
        for (std::size_t i = 0; i < this->jointStatesInMsgs.size(); ++i) {
            if (!this->jointStatesInMsgs[i].isLinked()) {
                bskLogger.bskError("JointArrayRefProfiler.jointStatesInMsgs[%zu] was not linked.", i);
            }
        }
    }
    if (this->jointStateDotsInMsgs.empty()) {
        bskLogger.bskError("JointArrayRefProfiler.jointStateDotsInMsgs vector is empty.");
    } else {
        if (this->jointStateDotsInMsgs.size() != static_cast<std::size_t>(this->numHingedJoints)) {
            bskLogger.bskError("JointArrayRefProfiler.jointStateDotsInMsgs size does not match numHingedJoints.");
        }
        for (std::size_t i = 0; i < this->jointStateDotsInMsgs.size(); ++i) {
            if (!this->jointStateDotsInMsgs[i].isLinked()) {
                bskLogger.bskError("JointArrayRefProfiler.jointStateDotsInMsgs[%zu] was not linked.", i);
            }
        }
    }
    if (!this->desJointStatesInMsg.isLinked()) {
        bskLogger.bskError("JointArrayRefProfiler.desJointStatesInMsg was not linked.");
    }

    // check that the profile type is set and valid
    if (this->profileType.empty()) {
        this->bskLogger.bskError("JointArrayRefProfiler.profileType is not set.");
    }
    const std::vector<std::string> validProfileTypes = {"lowPass", "linear", "cubic", "quintic"};
    if (std::find(validProfileTypes.begin(), validProfileTypes.end(), this->profileType) == validProfileTypes.end()) {
        this->bskLogger.bskError("JointArrayRefProfiler.profileType is not valid.");
    }

    // check that wc and filterDt are positive if the profile type is low pass
    if (this->profileType == "lowPass") {
        if (this->wc <= 0.0) {
            this->bskLogger.bskError("JointArrayRefProfiler.wc must be positive.");
        }
        if (this->filterDt <= 0.0) {
            this->bskLogger.bskError("JointArrayRefProfiler.filterDt must be positive.");
        }
    }

    // check that profile duration is positive if the profile type is not low pass
    if (this->profileType != "lowPass") {
        if (this->profileDuration <= 0.0) {
            this->bskLogger.bskError("JointArrayRefProfiler.profileDuration must be positive.");
        }
    }

    this->profileStartTime = 0;  // [ns]
    this->profileStartTimeSet = false;
    this->refJointAngles = Eigen::VectorXd::Zero(this->numHingedJoints);
    this->refJointRates = Eigen::VectorXd::Zero(this->numHingedJoints);
    this->refJointAccels = Eigen::VectorXd::Zero(this->numHingedJoints);
}

void JointArrayRefProfiler::UpdateState(uint64_t CurrentSimNanos)
{
    if (!this->desJointStatesInMsg.isWritten()) {
        this->bskLogger.bskError("JointArrayRefProfiler called before desired joint states message was written.");
    }

    JointArrayStateMsgPayload desJointStatesIn = this->desJointStatesInMsg();
    if (desJointStatesIn.states.size() != static_cast<std::size_t>(this->numHingedJoints)) {
        this->bskLogger.bskError("JointArrayRefProfiler.desJointStatesInMsg.states size does not match numHingedJoints.");
    }
    if (desJointStatesIn.stateDots.size() != static_cast<std::size_t>(this->numHingedJoints)) {
        this->bskLogger.bskError("JointArrayRefProfiler.desJointStatesInMsg.stateDots size does not match numHingedJoints.");
    }
    if (desJointStatesIn.stateDDots.size() != static_cast<std::size_t>(this->numHingedJoints)) {
        this->bskLogger.bskError("JointArrayRefProfiler.desJointStatesInMsg.stateDDots size does not match numHingedJoints.");
    }

    bool newDesiredStateMsg = false;

    if (!this->profileStartTimeSet) {
        newDesiredStateMsg = true;
    } else {
        newDesiredStateMsg =
            desJointStatesIn.states != this->prevDesJointStates.states ||
            desJointStatesIn.stateDots != this->prevDesJointStates.stateDots ||
            desJointStatesIn.stateDDots != this->prevDesJointStates.stateDDots;
    }

    if (newDesiredStateMsg) {
        this->profileStartTime = CurrentSimNanos;  // [ns]
        this->profileStartTimeSet = true;
        this->startJointAngles = Eigen::VectorXd::Zero(this->numHingedJoints);
        this->startJointRates = Eigen::VectorXd::Zero(this->numHingedJoints);
        this->prevDesJointStates = desJointStatesIn;
        for (std::size_t i = 0; i < this->jointStatesInMsgs.size(); ++i) {
            ScalarJointStateMsgPayload jointStateIn = this->jointStatesInMsgs[i]();
            ScalarJointStateMsgPayload jointStateDotIn = this->jointStateDotsInMsgs[i]();
            this->startJointAngles[i] = jointStateIn.state;
            this->startJointRates[i] = jointStateDotIn.state;
        }

        if (this->profileType == "lowPass") {
            // for low pass filter, initialize the reference states to the current states when a new command is received
            this->refJointAngles = this->startJointAngles;
            this->refJointRates = this->startJointRates;
            this->refJointAccels = Eigen::VectorXd::Zero(this->numHingedJoints);
        }
    }

    // compute the current reference states based on the profile type
    if (this->profileType == "lowPass" && !newDesiredStateMsg) {
        this->computeLowPassFilter(CurrentSimNanos, desJointStatesIn);
    }
    else if (this->profileType == "linear") {
        this->computeLinearProfile(CurrentSimNanos, desJointStatesIn);
    }
    else if (this->profileType == "cubic") {
        this->computeCubicProfile(CurrentSimNanos, desJointStatesIn);
    }
    else if (this->profileType == "quintic") {
        this->computeQuinticProfile(CurrentSimNanos, desJointStatesIn);
    }

    // write the current reference states to the output message
    JointArrayStateMsgPayload desJointStatesOut = this->desJointStatesOutMsg.zeroMsgPayload;
    desJointStatesOut.states.resize(static_cast<std::size_t>(this->numHingedJoints), 0.0);
    desJointStatesOut.stateDots.resize(static_cast<std::size_t>(this->numHingedJoints), 0.0);
    desJointStatesOut.stateDDots.resize(static_cast<std::size_t>(this->numHingedJoints), 0.0);
    for (int i = 0; i < this->numHingedJoints; ++i) {
        desJointStatesOut.states[static_cast<std::size_t>(i)] = this->refJointAngles[i];
        desJointStatesOut.stateDots[static_cast<std::size_t>(i)] = this->refJointRates[i];
        desJointStatesOut.stateDDots[static_cast<std::size_t>(i)] = this->refJointAccels[i];
    }
    this->desJointStatesOutMsg.write(&desJointStatesOut, this->moduleID, CurrentSimNanos);
}

void JointArrayRefProfiler::computeLowPassFilter(uint64_t CurrentSimNanos, const JointArrayStateMsgPayload& desJointStatesIn)
{
    double beta = 1.0 - std::exp(-this->wc * this->filterDt);

    Eigen::VectorXd prevRefJointAngles = this->refJointAngles;
    Eigen::VectorXd prevRefJointRates = this->refJointRates;
    for (int i = 0; i < this->numHingedJoints; ++i) {
        const double thetaCmd = desJointStatesIn.states[i];
        this->refJointAngles[i] = prevRefJointAngles[i] + beta * (thetaCmd - prevRefJointAngles[i]);
        this->refJointRates[i] = (this->refJointAngles[i] - prevRefJointAngles[i]) / this->filterDt;
        this->refJointAccels[i] = (this->refJointRates[i] - prevRefJointRates[i]) / this->filterDt;
    }
}

void JointArrayRefProfiler::computeLinearProfile(uint64_t CurrentSimNanos, const JointArrayStateMsgPayload& desJointStatesIn)
{
    double timeSec = CurrentSimNanos * NANO2SEC; // convert current time to seconds
    double tau = timeSec - (this->profileStartTime * NANO2SEC); // time since profile start in seconds

    this->refJointAngles.setZero(this->numHingedJoints);
    this->refJointRates.setZero(this->numHingedJoints);
    this->refJointAccels.setZero(this->numHingedJoints);
    if (tau >= this->profileDuration) {
        for (int i = 0; i < this->numHingedJoints; ++i) {
            this->refJointAngles[i] = desJointStatesIn.states[i];
            this->refJointRates[i] = 0.0;
            this->refJointAccels[i] = 0.0;
        }
        return;
    }

    for (int i = 0; i < this->numHingedJoints; ++i) {
        double theta0 = this->startJointAngles[i];
        double thetaf = desJointStatesIn.states[i];
        double T = this->profileDuration;

        // compute reference angle, rate, and acceleration
        this->refJointAngles[i] = theta0 + (thetaf - theta0) * tau / T;
        this->refJointRates[i] = (thetaf - theta0) / T;
        this->refJointAccels[i] = 0.0;
    }
}

void JointArrayRefProfiler::computeCubicProfile(uint64_t CurrentSimNanos, const JointArrayStateMsgPayload& desJointStatesIn)
{
    double timeSec = CurrentSimNanos * NANO2SEC; // convert current time to seconds
    double tau = timeSec - (this->profileStartTime * NANO2SEC); // time since profile start in seconds

    this->refJointAngles.setZero(this->numHingedJoints);
    this->refJointRates.setZero(this->numHingedJoints);
    this->refJointAccels.setZero(this->numHingedJoints);
    if (tau >= this->profileDuration) {
        for (int i = 0; i < this->numHingedJoints; ++i) {
            this->refJointAngles[i] = desJointStatesIn.states[i];
            this->refJointRates[i] = 0.0;
            this->refJointAccels[i] = 0.0;
        }
        return;
    }

    for (int i = 0; i < this->numHingedJoints; ++i) {
        double theta0 = this->startJointAngles[i];
        double thetaf = desJointStatesIn.states[i];
        double thetaDot0 = this->startJointRates[i];
        double T = this->profileDuration;

        // cubic profile coefficients
        double a0 = theta0;
        double a1 = thetaDot0;
        double a2 = (3 * (thetaf - theta0)) / (T * T) - (2 * thetaDot0) / T;
        double a3 = -(2 * (thetaf - theta0)) / (T * T * T) + (thetaDot0) / (T * T);

        // compute reference angle, rate, and acceleration
        this->refJointAngles[i] = a0 + a1 * tau + a2 * tau * tau + a3 * tau * tau * tau;
        this->refJointRates[i] = a1 + 2.0 * a2 * tau + 3.0 * a3 * tau * tau;
        this->refJointAccels[i] = 2.0 * a2 + 6.0 * a3 * tau;
    }
}

void JointArrayRefProfiler::computeQuinticProfile(uint64_t CurrentSimNanos, const JointArrayStateMsgPayload& desJointStatesIn)
{
    double timeSec = CurrentSimNanos * NANO2SEC; // convert current time to seconds
    double tau = timeSec - (this->profileStartTime * NANO2SEC); // time since profile start in seconds

    this->refJointAngles.setZero(this->numHingedJoints);
    this->refJointRates.setZero(this->numHingedJoints);
    this->refJointAccels.setZero(this->numHingedJoints);
    if (tau >= this->profileDuration) {
        for (int i = 0; i < this->numHingedJoints; ++i) {
            this->refJointAngles[i] = desJointStatesIn.states[i];
            this->refJointRates[i] = 0.0;
            this->refJointAccels[i] = 0.0;
        }
        return;
    }

    for (int i = 0; i < this->numHingedJoints; ++i) {
        double theta0 = this->startJointAngles[i];
        double thetaf = desJointStatesIn.states[i];
        double thetaDot0 = this->startJointRates[i];
        double deltaTheta = thetaf - theta0;
        double T = this->profileDuration;

        // quintic profile coefficients
        double a0 = theta0;
        double a1 = thetaDot0;
        double a2 = 0.0;
        double a3 = (20 * deltaTheta - 12 * thetaDot0 * T) / (2 * T * T * T);
        double a4 = (-30 * deltaTheta + 16 * thetaDot0 * T) / (2 * T * T * T * T);
        double a5 = (12 * deltaTheta - 6 * thetaDot0 * T) / (2 * T * T * T * T * T);

        // compute reference angle, rate, and acceleration
        this->refJointAngles[i] = a0 + a1 * tau + a2 * tau * tau + a3 * tau * tau * tau + a4 * tau * tau * tau * tau + a5 * tau * tau * tau * tau * tau;
        this->refJointRates[i] = a1 + 2.0 * a2 * tau + 3.0 * a3 * tau * tau + 4.0 * a4 * tau * tau * tau + 5.0 * a5 * tau * tau * tau * tau;
        this->refJointAccels[i] = 2.0 * a2 + 6.0 * a3 * tau + 12.0 * a4 * tau * tau + 20.0 * a5 * tau * tau * tau;
    }
}

void JointArrayRefProfiler::setProfileType(std::string profileType)
{
    this->profileType = profileType;
}

void JointArrayRefProfiler::setProfileDuration(double profileDuration)
{
    this->profileDuration = profileDuration;
}

void JointArrayRefProfiler::setWc(double wc)
{
    this->wc = wc;
}

void JointArrayRefProfiler::setFilterDt(double filterDt)
{
    this->filterDt = filterDt;
}

void JointArrayRefProfiler::addHingedJoint()
{
    // increase the number of hinged joints by 1
    this->numHingedJoints++;

    // add a new input message reader for the new hinged joint
    this->jointStatesInMsgs.push_back(ReadFunctor<ScalarJointStateMsgPayload>());
    this->jointStateDotsInMsgs.push_back(ReadFunctor<ScalarJointStateMsgPayload>());
}
