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

#ifndef THR_ON_TIME_TO_FORCE_H
#define THR_ON_TIME_TO_FORCE_H

#include <cstdint>
#include <vector>

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/messaging/messaging.h"
#include "architecture/msgPayloadDefC/SingleActuatorMsgPayload.h"
#include "architecture/msgPayloadDefC/THRArrayOnTimeCmdMsgPayload.h"
#include "architecture/utilities/bskLogging.h"
#include "architecture/utilities/avsEigenSupport.h"

/*! @brief Converts thruster on-time commands into per-thruster force messages.
 */
class ThrOnTimeToForce: public SysModel {
public:
    ThrOnTimeToForce() = default;                  //!< This is the constructor for the module class.
    ~ThrOnTimeToForce() = default;                 //!< This is the destructor for the module class.

    /*!
    * This method is used to reset the module and checks internal consistency.
    */
    void Reset(uint64_t CurrentSimNanos);

    /*!
    * This is the main method that gets called every time the module is updated.
    * It decrements firing timers and writes one force command per thruster.
    * */
    void UpdateState(uint64_t CurrentSimNanos);

public:
    ReadFunctor<THRArrayOnTimeCmdMsgPayload> onTimeInMsg;  //!< input on-time command array

    std::vector<Message<SingleActuatorMsgPayload>*> thrusterForceOutMsgs;  //!< vector of thruster force output messages

    BSKLogger bskLogger;              //!< BSK Logging

    /** setter for `thrMag` property */
    void setThrMag(const std::vector<double>&);
    /** getter for `thrMag` property */
    std::vector<double> getThrMag() const {return this->thrMag;}
    /** method for adding a new thruster force output channel */
    void addThruster();

private:
    int numThr = 0;                            //!< number of thrusters
    std::vector<double> thrMag;               //!< [N] fixed force magnitude for each thruster
    std::vector<double> firingTimeRemaining;      //!< [s] remaining commanded firing time for each thruster
    uint64_t prevCommandWriteNanos;                //!< [ns] last processed command message time stamp
    double previousUpdateTimeSec;                  //!< [s] previous module update time
};


#endif
