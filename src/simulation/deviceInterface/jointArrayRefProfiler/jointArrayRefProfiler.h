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

#ifndef JOINT_ARRAY_REF_PROFILER_H
#define JOINT_ARRAY_REF_PROFILER_H

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/messaging/messaging.h"
#include "architecture/msgPayloadDefC/ScalarJointStateMsgPayload.h"
#include "architecture/msgPayloadDefCpp/JointArrayStateMsgPayload.h"
#include "architecture/utilities/bskLogging.h"
#include "architecture/utilities/macroDefinitions.h"
#include "architecture/utilities/avsEigenSupport.h"
#include <Eigen/Dense>

/*! @brief This module converts instantaneous reference angle changes for an array of joints to smooth profiles.
 */
class JointArrayRefProfiler : public SysModel {
public:
    JointArrayRefProfiler() = default; //!< This is the constructor for the module class.
    ~JointArrayRefProfiler() = default; //!< This is the destructor for the module class.

    /*!
    * This method is used to reset the module and checks that required input messages are connected.
    */
    void Reset(uint64_t CurrentSimNanos);

    /*!
    * This is the main method that gets called every time the module is updated.
    * It determines the current joint reference states based on the profile shape and duration.
    * */
    void UpdateState(uint64_t CurrentSimNanos);

public:
    std::vector<ReadFunctor<ScalarJointStateMsgPayload>> jointStatesInMsgs;     //!< vector of current joint state input messages
    std::vector<ReadFunctor<ScalarJointStateMsgPayload>> jointStateDotsInMsgs;  //!< vector of current joint state-derivative input messages
    ReadFunctor<JointArrayStateMsgPayload> desJointStatesInMsg;                 //!< Desired joint-array state input message

    Message<JointArrayStateMsgPayload> desJointStatesOutMsg;                  //!< Joint-array reference output message

    BSKLogger bskLogger;                                                        //!< BSK Logging
    /** setter for the `profileType` property */
    void setProfileType(std::string profileType);
    /** getter for the `profileType` property */
    std::string getProfileType() const {return this->profileType;}
    /** setter for the `profileDuration` property */
    void setProfileDuration(double profileDuration);
    /** getter for the `profileDuration` property */
    double getProfileDuration() const {return this->profileDuration;}
    /** setter for the `wc` property */
    void setWc(double wc);
    /** getter for the `wc` property */
    double getWc() const {return this->wc;}
    /** setter for the `filterDt` property */
    void setFilterDt(double filterDt);
    /** getter for the `filterDt` property */
    double getFilterDt() const {return this->filterDt;}
    /** method for adding a new hinged joint to the system */
    void addHingedJoint();

private:
    std::string profileType; //!< [-] Reference profile type
    double profileDuration = -1.0;  //!< [s] Reference profile duration, used for all profile types except "lowPass"
    double wc = -1.0;               //!< [rad/s] low pass filter cutoff frequency, only used if profileType is "lowPass"
    double filterDt = -1.0;         //!< [s] low pass filter time step, only used if profileType is "lowPass"
    int numHingedJoints = 0;  //!< number of hinged joints in the system
    uint64_t profileStartTime = 0;  //!< [ns] simulation time at which the current profile started
    bool profileStartTimeSet = false;  //!< flag indicating whether the profile start time has been set
    Eigen::VectorXd startJointAngles;  //!< [rad] joint angles at the start of the current reference profile
    Eigen::VectorXd startJointRates;  //!< [rad/s] joint angle rates at the start of the current reference profile
    Eigen::VectorXd refJointAngles;  //!< [rad] current profiled joint angles
    Eigen::VectorXd refJointRates;  //!< [rad/s] current profiled joint angle rates
    Eigen::VectorXd refJointAccels;  //!< [rad/s^2] current profiled joint angle accelerations
    JointArrayStateMsgPayload prevDesJointStates; //!< previous desired joint states, used to detect changes in the desired state command

    /*! Method for computing the low pass filtered joint reference profile
        * @param CurrentSimNanos current simulation time in nanoseconds
        * @param desJointStatesIn desired joint states input message payload
        * */
    void computeLowPassFilter(uint64_t CurrentSimNanos, const JointArrayStateMsgPayload& desJointStatesIn);

    /*! Method for computing the linear joint reference profile
        * @param CurrentSimNanos current simulation time in nanoseconds
        * @param desJointStatesIn desired joint states input message payload
        * */
    void computeLinearProfile(uint64_t CurrentSimNanos, const JointArrayStateMsgPayload& desJointStatesIn);

    /*! Method for computing the cubic joint reference profile
        * @param CurrentSimNanos current simulation time in nanoseconds
        * @param desJointStatesIn desired joint states input message payload
        * */
    void computeCubicProfile(uint64_t CurrentSimNanos, const JointArrayStateMsgPayload& desJointStatesIn);

    /*! Method for computing the quintic joint reference profile
        * @param CurrentSimNanos current simulation time in nanoseconds
        * @param desJointStatesIn desired joint states input message payload
        * */
    void computeQuinticProfile(uint64_t CurrentSimNanos, const JointArrayStateMsgPayload& desJointStatesIn);

};

#endif
