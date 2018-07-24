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
#include "sensors/star_tracker/star_tracker.h"
#include "architecture/messaging/system_messaging.h"
#include "utilities/rigidBodyKinematics.h"
#include "utilities/linearAlgebra.h"
#include "utilities/bsk_Print.h"
#include "simFswInterfaceMessages/macroDefinitions.h"
#include <iostream>
#include <cstring>
#include "utilities/avsEigenSupport.h"
#include "utilities/gauss_markov.h"

StarTracker::StarTracker()
{
    this->CallCounts = 0;
    this->messagesLinked = false;
    this->inputStateID = -1;
    this->inputStateMessage = "inertial_state_output";
    this->outputStateMessage = "star_tracker_state";
    this->OutputBufferCount = 2;
    this->sensorTimeTag = 0;
    m33SetIdentity(RECAST3X3 this->dcm_CB);
    this->errorModel = GaussMarkov(3);
    this->PMatrix.fill(0.0);
    this->AMatrix.fill(0.0);
    this->walkBounds.fill(0.0);
    return;
}

StarTracker::~StarTracker()
{
    return;
}

bool StarTracker::LinkMessages()
{
    this->inputStateID = SystemMessaging::GetInstance()->subscribeToMessage(
        this->inputStateMessage, sizeof(SCPlusStatesSimMsg), this->moduleID);
    
    
    return(inputStateID >= 0);
}

void StarTracker::SelfInit()
{
    //! Begin method steps
    uint64_t numStates = 3;
    this->outputStateID = SystemMessaging::GetInstance()->
        CreateNewMessage(this->outputStateMessage, sizeof(STSensorIntMsg),
        OutputBufferCount, "STSensorIntMsg", this->moduleID);
    
    this->AMatrix.setIdentity(numStates, numStates);

    //! - Alert the user if the noise matrix was not the right size.  That'd be bad.
    if(this->PMatrix.size() != numStates*numStates)
    {
        BSK_PRINT(MSG_ERROR, "Your process noise matrix (PMatrix) is not 3*3. Quitting.");
        return;
    }
    if(this->walkBounds.size() != numStates){
        BSK_PRINT(MSG_ERROR, "Your walkbounds is not size 3. Quitting");
        return;
    }
    this->errorModel.setNoiseMatrix(this->PMatrix);
    this->errorModel.setRNGSeed(this->RNGSeed);
    this->errorModel.setUpperBounds(this->walkBounds);
}

void StarTracker::CrossInit()
{
    messagesLinked = this->LinkMessages();
}

void StarTracker::readInputMessages()
{
    SingleMessageHeader localHeader;
    
    if(!this->messagesLinked)
    {
        this->messagesLinked = LinkMessages();
    }
    
    memset(&this->scState, 0x0, sizeof(SCPlusStatesSimMsg));
    if(this->inputStateID >= 0)
    {
        SystemMessaging::GetInstance()->ReadMessage(this->inputStateID, &localHeader,
                                                    sizeof(SCPlusStatesSimMsg), reinterpret_cast<uint8_t*>(&scState), this->moduleID);
        this->sensorTimeTag = localHeader.WriteClockNanos;
    }
}

void StarTracker::computeSensorErrors()
{
    this->errorModel.setPropMatrix(this->AMatrix);
    this->errorModel.computeNextState();
    this->navErrors = this->errorModel.getCurrentState();
}

void StarTracker::applySensorErrors()
{
    double sigmaSensed[3];
    PRV2MRP(&(this->navErrors.data()[0]), this->mrpErrors);
    addMRP(this->scState.sigma_BN, this->mrpErrors, sigmaSensed);
    this->computeQuaternion(sigmaSensed, &this->sensedValues);
    this->sensedValues.timeTag = this->sensorTimeTag;
}

void StarTracker::computeQuaternion(double *sigma, STSensorIntMsg *sensorValues)
{
    double dcm_BN[3][3];            /* dcm, inertial to body frame */
    double dcm_CN[3][3];            /* dcm, inertial to case frame */
    MRP2C(sigma, dcm_BN);
    m33MultM33(RECAST3X3 this->dcm_CB, dcm_BN, dcm_CN);
    C2EP(dcm_CN, sensorValues->qInrtl2Case);
}

void StarTracker::computeTrueOutput()
{
    this->trueValues.timeTag = this->sensorTimeTag;
    this->computeQuaternion(this->scState.sigma_BN, &this->trueValues);
}


void StarTracker::writeOutputMessages(uint64_t CurrentSimNanos)
{
    SystemMessaging::GetInstance()->WriteMessage(this->outputStateID, CurrentSimNanos,
                                                 sizeof(STSensorIntMsg), reinterpret_cast<uint8_t *>(&this->sensedValues), this->moduleID);
}

void StarTracker::UpdateState(uint64_t CurrentSimNanos)
{
    this->readInputMessages();
    this->computeSensorErrors();
    this->computeTrueOutput();
    this->applySensorErrors();
    this->writeOutputMessages(CurrentSimNanos);
}
