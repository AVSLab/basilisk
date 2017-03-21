/*
 ISC License

 Copyright (c) 2016-2017, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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
#include "sensors/star_tracker/star_tracker.h"
#include "architecture/messaging/system_messaging.h"
#include "utilities/rigidBodyKinematics.h"
#include "utilities/linearAlgebra.h"
#include "simFswInterfaceMessages/macroDefinitions.h"
#include <iostream>
#include <cstring>

StarTracker::StarTracker()
{
    CallCounts = 0;
    this->messagesLinked = false;
    this->inputTimeID = -1;
    this->inputStateID = -1;
    this->inputTimeMessage = "spice_time_output_data";
    this->inputStateMessage = "inertial_state_output";
    this->outputStateMessage = "star_tracker_state";
    this->OutputBufferCount = 2;
    m33SetIdentity(RECAST3X3 this->dcm_CS);
    return;
}

StarTracker::~StarTracker()
{
    return;
}

bool StarTracker::LinkMessages()
{
    inputTimeID = SystemMessaging::GetInstance()->subscribeToMessage(
        inputTimeMessage, sizeof(SpiceTimeSimMsg), moduleID);
    inputStateID = SystemMessaging::GetInstance()->subscribeToMessage(
        inputStateMessage, sizeof(SCPlusStatesSimMsg), moduleID);
    
    
    return(inputTimeID >=0 && inputStateID >= 0);
}

void StarTracker::SelfInit()
{
    //! Begin method steps
    uint64_t numStates = 3;
    outputStateID = SystemMessaging::GetInstance()->
        CreateNewMessage(outputStateMessage, sizeof(STSensorIntMsg),
        OutputBufferCount, "STSensorIntMsg", moduleID);
    
    AMatrix.clear();
    AMatrix.insert(AMatrix.begin(), numStates*numStates, 0.0);
    mSetIdentity(AMatrix.data(), numStates, numStates);
    for(uint32_t i=0; i<3; i++)
    {
		AMatrix.data()[i * 3 + i] = 1.0;
    }
    //! - Alert the user if the noise matrix was not the right size.  That'd be bad.
    if(PMatrix.size() != numStates*numStates)
    {
        std::cerr << __FILE__ <<": Your process noise matrix (PMatrix) is not 3*3.";
        std::cerr << "  You should fix that.  Popping zeros onto end"<<std::endl;
        PMatrix.insert(PMatrix.begin()+PMatrix.size(), numStates*numStates - PMatrix.size(),
                       0.0);
    }
    errorModel.setNoiseMatrix(PMatrix);
    errorModel.setRNGSeed(RNGSeed);
    errorModel.setUpperBounds(walkBounds);
}

void StarTracker::CrossInit()
{
    messagesLinked = LinkMessages();
}

void StarTracker::readInputMessages()
{
    SingleMessageHeader localHeader;
    
    if(!this->messagesLinked)
    {
        this->messagesLinked = LinkMessages();
    }
    
    memset(&this->timeState, 0x0, sizeof(SpiceTimeSimMsg));
    memset(&this->scState, 0x0, sizeof(SCPlusStatesSimMsg));
    if(inputStateID >= 0)
    {
        SystemMessaging::GetInstance()->ReadMessage(inputStateID, &localHeader,
                                                    sizeof(SCPlusStatesSimMsg), reinterpret_cast<uint8_t*>(&scState), moduleID);
    }
    if(inputTimeID >= 0)
    {
        SystemMessaging::GetInstance()->ReadMessage(inputTimeID, &localHeader,
                                                    sizeof(SpiceTimeSimMsg), reinterpret_cast<uint8_t*>(&timeState), moduleID);
        this->envTimeClock = localHeader.WriteClockNanos;
    }
}

void StarTracker::computeSensorErrors()
{
    this->errorModel.setPropMatrix(AMatrix);
    this->errorModel.computeNextState();
    this->navErrors = this->errorModel.getCurrentState();
}

void StarTracker::applySensorErrors()
{
    double sigmaSensed[3];
    PRV2MRP(&(navErrors.data()[0]), this->mrpErrors);
    addMRP(scState.sigma_BN, this->mrpErrors, sigmaSensed);
    computeQuaternion(sigmaSensed, &this->sensedValues);
    this->sensedValues.timeTag = this->sensorTimeTag;
}

void StarTracker::computeQuaternion(double *sigma, STSensorIntMsg *sensorValues)
{
    double dcm_BN[3][3];            /* dcm, inertial to body frame */
    double dcm_SN[3][3];            /* dcm, inertial to structure frame */
    double dcm_CN[3][3];            /* dcm, inertial to case frame */
    MRP2C(sigma, dcm_BN);
    m33tMultM33(scState.dcm_BS, dcm_BN, dcm_SN);
    m33MultM33(RECAST3X3 dcm_CS, dcm_SN, dcm_CN);
    C2EP(dcm_CN, sensorValues->qInrtl2Case);
}

void StarTracker::computeSensorTimeTag(uint64_t CurrentSimNanos)
{
    this->sensorTimeTag = this->timeState.J2000Current;
    this->sensorTimeTag += (CurrentSimNanos - this->envTimeClock)*1.0E-9;
}

void StarTracker::computeTrueOutput()
{
    this->trueValues.timeTag = this->sensorTimeTag;
    computeQuaternion(this->scState.sigma_BN, &this->trueValues);
}


void StarTracker::writeOutputMessages(uint64_t CurrentSimNanos)
{
    SystemMessaging::GetInstance()->WriteMessage(outputStateID, CurrentSimNanos,
                                                 sizeof(STSensorIntMsg), reinterpret_cast<uint8_t *>(&this->sensedValues), moduleID);
}

void StarTracker::UpdateState(uint64_t CurrentSimNanos)
{
    readInputMessages();
    computeSensorTimeTag(CurrentSimNanos);
    computeSensorErrors();
    computeTrueOutput();
    applySensorErrors();
    writeOutputMessages(CurrentSimNanos);
}
