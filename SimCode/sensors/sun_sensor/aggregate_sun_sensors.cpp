/*
Copyright (c) 2016, Autonomous Vehicle Systems Lab, Univeristy of Colorado at Boulder

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
#include "sensors/sun_sensor/aggregate_sun_sensors.h"
#include "architecture/messaging/system_messaging.h"
#include "../ADCSAlgorithms/sensorInterfaces/CSSSensorData/cssComm.h"
#include "utilities/rigidBodyKinematics.h"
#include "utilities/linearAlgebra.h"
#include <math.h>
#include <iostream>
#include <cstring>
#include <random>

AggregateSunSensors::AggregateSunSensors()
{
    CallCounts = 0;
	inputCSSIDs.clear();
	inputCSSMsgs.clear();
	outputAggMsg = "";
	outputDataID = -1;
    outputBufferCount = 2;
	outputBuffer = NULL;
    
    return;
}

AggregateSunSensors::~AggregateSunSensors()
{
	if (outputBuffer != NULL)
	{
		delete [] outputBuffer;
	}
    return;
}

void AggregateSunSensors::SelfInit()
{
	outputDataID = SystemMessaging::GetInstance()->CreateNewMessage(outputAggMsg,
		MAX_NUM_CSS_SENSORS*sizeof(CSSRawOutputData), outputBufferCount, "CSSRawOutputData", moduleID);

}

void AggregateSunSensors::CrossInit()
{
	std::vector<std::string>::iterator it;
	CSSRawOutputData storageInit;
	memset(&storageInit, 0x0, sizeof(CSSRawOutputData));
	for (it = inputCSSMsgs.begin(); it != inputCSSMsgs.end(); it++)
	{
		int64_t localID = SystemMessaging::GetInstance()->subscribeToMessage(*it,
			sizeof(CSSRawOutputData), moduleID);
		inputCSSIDs.push_back(localID);
		cssStorage.push_back(storageInit);
	}
	outputBuffer = new CSSRawOutputData[inputCSSMsgs.size()];
}


void AggregateSunSensors::ReadInputs()
{
	bool messageRead;
	std::vector<int64_t>::iterator it;
	std::vector<CSSRawOutputData>::iterator dataIt;
	CSSRawOutputData messageOut;
	SingleMessageHeader messHead;
	for (it = inputCSSIDs.begin(), dataIt=cssStorage.begin(); 
	    it != inputCSSIDs.end(); it++, dataIt++)
	{
		messageRead = SystemMessaging::GetInstance()->ReadMessage(*it, &messHead, sizeof(CSSRawOutputData),
			reinterpret_cast<uint8_t*> (&messageOut));
		if (messageRead && messHead.WriteSize >= sizeof(CSSRawOutputData))
		{
			memcpy(&(*dataIt), &messHead, sizeof(CSSRawOutputData));
		}
	}
}

void AggregateSunSensors::WriteOutputs(uint64_t Clock)
{
	std::vector<CSSRawOutputData>::iterator it;
	for (it = cssStorage.begin(); it != cssStorage.end(); it++)
	{
		memcpy(&(outputBuffer[it - cssStorage.begin()]), &(*it), sizeof(CSSRawOutputData));
	}
	SystemMessaging::GetInstance()->WriteMessage(outputDataID, Clock,
		cssStorage.size()*sizeof(CSSRawOutputData), reinterpret_cast<uint8_t*> (outputBuffer), moduleID);
}

void AggregateSunSensors::UpdateState(uint64_t CurrentSimNanos)
{
    ReadInputs();
    WriteOutputs(CurrentSimNanos);
}
