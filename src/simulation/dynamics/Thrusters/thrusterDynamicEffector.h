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


#ifndef THRUSTER_DYNAMIC_EFFECTOR_H
#define THRUSTER_DYNAMIC_EFFECTOR_H

#include "../_GeneralModuleFiles/dynamicEffector.h"
#include "../_GeneralModuleFiles/stateData.h"
#include "_GeneralModuleFiles/sys_model.h"
#include "simMessages/thrTimePairSimMsg.h"
#include "simMessages/thrOperationSimMsg.h"
#include "simMessages/thrConfigSimMsg.h"
#include "simMessages/thrOutputSimMsg.h"
#include "../../simFswInterfaceMessages/thrArrayOnTimeCmdIntMsg.h"
#include "utilities/bskLogging.h"
#include <Eigen/Dense>
#include <vector>



/*! @brief thruster dynamic effector class */
class ThrusterDynamicEffector: public SysModel, public DynamicEffector {
public:
    ThrusterDynamicEffector();
    ~ThrusterDynamicEffector();
    void linkInStates(DynParamManager& states);
    void computeForceTorque(double integTime);
    void computeStateContribution(double integTime);
    void SelfInit();
    void CrossInit();
    //! Add a new thruster to the thruster set
    void addThruster(THRConfigSimMsg *newThruster);
    void UpdateState(uint64_t CurrentSimNanos);
    void writeOutputMessages(uint64_t CurrentClock);
    bool ReadInputs();
    void ConfigureThrustRequests(double currentTime);
    void ComputeThrusterFire(THRConfigSimMsg *CurrentThruster,
                             double currentTime);
    void ComputeThrusterShut(THRConfigSimMsg *CurrentThruster,
                             double currentTime);
    

public:
    int stepsInRamp;                               //!< class variable
    std::vector<THRConfigSimMsg> thrusterData;      //!< -- Thruster information
    std::string InputCmds;                         //!< -- message used to read command inputs
    std::string inputProperties;                   //!< [-] The mass properties of the spacecraft
    uint64_t thrusterOutMsgNameBufferCount;        //!< -- Count on number of buffers to output
    std::vector<std::string> thrusterOutMsgNames;  //!< -- Message name for all thruster data
    std::vector<double> NewThrustCmds;             //!< -- Incoming thrust commands
    double mDotTotal;                              //!< kg/s Current mass flow rate of thrusters
    double prevFireTime;                           //!< s  Previous thruster firing time
	double thrFactorToTime(THRConfigSimMsg *thrData,
		std::vector<THRTimePairSimMsg> *thrRamp);
	StateData *hubSigma;                           //!< class variable
    StateData *hubOmega;                           //!< class varaible
    BSKLogger bskLogger;                      //!< -- BSK Logging

private:
    //    bool bdyFrmReady;                         //!< [-] Flag indicating that the body frame is ready
    std::vector<uint64_t> thrusterOutMsgIds;        //!< -- Message ID of each thruster
    std::vector<THROutputSimMsg> thrusterOutBuffer;//!< -- Message buffer for thruster data
    int64_t CmdsInMsgID;                            //!< -- Message ID for incoming data
    //int64_t propsInID;                            //!< [-] The ID associated with the mss props msg
    THRArrayOnTimeCmdIntMsg IncomingCmdBuffer;     //!< -- One-time allocation for savings
    uint64_t prevCommandTime;                       //!< -- Time for previous valid thruster firing

};


#endif /* THRUSTER_DYNAMIC_EFFECTOR_H */
