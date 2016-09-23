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

#include "attDetermination/SunlineUKF/sunlineUKF.h"
#include "attDetermination/_GeneralModuleFiles/UKFUtilities.h"
#include "SimCode/utilities/linearAlgebra.h"
#include "ADCSUtilities/ADCSAlgorithmMacros.h"
#include "vehicleConfigData/vehicleConfigData.h"
#include <string.h>
#include <math.h>

/*! This method initializes the ConfigData for theCSS WLS estimator.
 It checks to ensure that the inputs are sane and then creates the
 output message
 @return void
 @param ConfigData The configuration data associated with the CSS WLS estimator
 */
void SelfInit_sunlineUKF(SunlineUKFConfig *ConfigData, uint64_t moduleID)
{

    int i;
    double tempMatrix[SKF_N_STATES*SKF_N_STATES];
    
    mSetZero(ConfigData->cssNHat_B, MAX_NUM_CSS_SENSORS, 3);
    
    /*! Begin method steps */
    /*! - Create output message for module */
	ConfigData->outputStateID = CreateNewMessage(ConfigData->outputNavStateName, 
		sizeof(NavAttOut), "NavAttOut", moduleID);

    ConfigData->timeTag = 0.0;
    ConfigData->dt = 0.0;
	ConfigData->numStates = SKF_N_STATES;
    ConfigData->countHalfSPs = SKF_N_STATES;
	ConfigData->numObs = MAX_N_CSS_MEAS;
	vSetZero(ConfigData->obs, ConfigData->numObs);
	ConfigData->lambdaVal = ConfigData->alpha*ConfigData->alpha*
		(ConfigData->numStates + ConfigData->kappa) - ConfigData->numStates;
	ConfigData->gamma = sqrt(ConfigData->numStates + ConfigData->lambdaVal);

	vSetZero(ConfigData->wM, ConfigData->countHalfSPs * 2 + 1);
	vSetZero(ConfigData->wC, ConfigData->countHalfSPs * 2 + 1);
	mSetZero(ConfigData->sBar, ConfigData->numStates, ConfigData->numStates);
	mSetZero(ConfigData->SP, ConfigData->countHalfSPs * 2 + 1, 
		ConfigData->numStates);
	mSetZero(ConfigData->sQnoise, ConfigData->numStates, ConfigData->numStates);
	ConfigData->wM[0] = ConfigData->lambdaVal / (ConfigData->numStates + 
		ConfigData->lambdaVal);
	ConfigData->wC[0] = ConfigData->lambdaVal / (ConfigData->numStates + 
		ConfigData->lambdaVal) + (1 - ConfigData->alpha*ConfigData->alpha 
		+ ConfigData->beta);
	for (i = 1; i<ConfigData->countHalfSPs * 2 + 1; i++)
	{
		ConfigData->wM[i] = 1.0 / 2.0*1.0 / (ConfigData->numStates + 
			ConfigData->lambdaVal);
		ConfigData->wC[i] = ConfigData->wM[i];
	}
	mCopy(ConfigData->covar, ConfigData->numStates, ConfigData->numStates, 
		ConfigData->sBar);
	ukfCholDecomp(ConfigData->sBar, ConfigData->numStates,
		ConfigData->numStates, tempMatrix);
    mCopy(tempMatrix, ConfigData->numStates, ConfigData->numStates,
        ConfigData->sBar);
	ukfCholDecomp(ConfigData->qNoise, ConfigData->numStates,
		ConfigData->numStates, ConfigData->sQnoise);
	mTranspose(ConfigData->sQnoise, ConfigData->numStates,
		ConfigData->numStates, ConfigData->sQnoise);
    
}

/*! This method performs the second stage of initialization for the CSS sensor
 interface.  It's primary function is to link the input messages that were
 created elsewhere.
 @return void
 @param ConfigData The configuration data associated with the CSS interface
 */
void CrossInit_sunlineUKF(SunlineUKFConfig *ConfigData, uint64_t moduleID)
{
    
    /*! - Loop over the number of sensors and find IDs for each one */
    ConfigData->inputCSSDataID = subscribeToMessage(ConfigData->inputCSSDataName,
        sizeof(CSSOutputData), moduleID);
    ConfigData->inputPropsID = subscribeToMessage(ConfigData->inputPropsName,
        sizeof(vehicleConfigData), moduleID);
    ConfigData->inputCSSConID = subscribeToMessage(ConfigData->inputCSSConfigName,
                                                   sizeof(CSSConstConfig), moduleID);
    
    
}

/*! This method resets the sunline attitude filter to an initial state and
 initializes the internal estimation matrices.
 @return void
 @param ConfigData The configuration data associated with the CSS estimator
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void Reset_sunlineUKF(SunlineUKFConfig *ConfigData, uint64_t callTime,
                      uint64_t moduleID)
{
    
    int32_t i;
    vehicleConfigData localConfigData;
    CSSConstConfig localCSSConfig;
    uint64_t writeTime;
    uint32_t writeSize;
    
    memset(&localConfigData, 0x0 ,sizeof(vehicleConfigData));
    memset(&localCSSConfig, 0x0, sizeof(CSSConstConfig));
    
    ReadMessage(ConfigData->inputPropsID, &writeTime, &writeSize,
                sizeof(vehicleConfigData), &localConfigData, moduleID);
    ReadMessage(ConfigData->inputCSSConID, &writeTime, &writeSize,
                sizeof(CSSConstConfig), &localCSSConfig, moduleID);
    for(i=0; i<localCSSConfig.nCSS; i = i+1)
    {
         m33MultV3(RECAST3X3 localConfigData.BS, localCSSConfig.cssVals[i].nHat_S,
             &(ConfigData->cssNHat_B[i*3]));
    }
    ConfigData->numCSSTotal = localCSSConfig.nCSS;

	memset(&(ConfigData->outputSunline), 0x0, sizeof(NavAttOut));

    return;
}

/*! This method takes the parsed CSS sensor data and outputs an estimate of the
 sun vector in the ADCS body frame
 @return void
 @param ConfigData The configuration data associated with the CSS estimator
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void Update_sunlineUKF(SunlineUKFConfig *ConfigData, uint64_t callTime,
    uint64_t moduleID)
{
    double newTimeTag;
    uint64_t ClockTime;
    uint32_t ReadSize;
    
    /*! Begin method steps*/
    /*! - Read the input parsed CSS sensor data message*/
    ClockTime = 0;
    ReadSize = 0;
    memset(&(ConfigData->rawSensorData), 0x0, sizeof(CSSOutputData));
    ReadMessage(ConfigData->inputCSSDataID, &ClockTime, &ReadSize,
        sizeof(CSSOutputData), (void*) (&(ConfigData->rawSensorData)), moduleID);
    newTimeTag = ClockTime * NANO2SEC;
    if(newTimeTag >= ConfigData->timeTag && ReadSize > 0)
    {
        sunlineUKFTimeUpdate(ConfigData, newTimeTag);
        sunlineUKFMeasUpdate(ConfigData, newTimeTag);
    }
    
    newTimeTag = callTime*NANO2SEC;
    if(newTimeTag > ConfigData->timeTag)
    {
        sunlineUKFTimeUpdate(ConfigData, newTimeTag);
    }
	v3Copy(ConfigData->state, ConfigData->outputSunline.vehSunPntBdy);
    v3Normalize(ConfigData->outputSunline.vehSunPntBdy,
        ConfigData->outputSunline.vehSunPntBdy);
	WriteMessage(ConfigData->outputStateID, callTime, sizeof(NavAttOut), 
		&(ConfigData->outputSunline), moduleID);
    return;
}

/*! This method propagates a sunline state vector forward in time.  Note 
    that the calling parameter is updated in place to save on data copies.
	@return void
	@param stateInOut The state that is propagated
*/
void sunlineStateProp(double *stateInOut, double dt)
{

    double propagatedVel[3];
    double initialMag;
    v3Scale(dt, &(stateInOut[3]), propagatedVel);
    initialMag = v3Norm(stateInOut);
    v3Add(stateInOut, propagatedVel, stateInOut);
    v3Normalize(stateInOut, stateInOut);
    v3Scale(initialMag, stateInOut, stateInOut);

	return;
}

/*! This method performs the time update for the sunline kalman filter.
     It propagates the sigma points forward in time and then gets the current 
	 covariance and state estimates.
	 @return void
     @param ConfigData The configuration data associated with the CSS estimator
     @param updateTime The time that we need to fix the filter to (seconds)
*/
void sunlineUKFTimeUpdate(SunlineUKFConfig *ConfigData, double updateTime)
{
	int i, Index;
	double xBar[SKF_N_STATES], sBarT[SKF_N_STATES*SKF_N_STATES];
	double xComp[SKF_N_STATES], AT[(2 * SKF_N_STATES + SKF_N_STATES)*SKF_N_STATES];
	double aRow[SKF_N_STATES], rAT[SKF_N_STATES*SKF_N_STATES], xErr[SKF_N_STATES]; 
	double sBarUp[SKF_N_STATES*SKF_N_STATES];
	double *spPtr;

	mTranspose(ConfigData->sBar, ConfigData->numStates, ConfigData->numStates,
		sBarT);
	
	ConfigData->dt = updateTime - ConfigData->timeTag;
	vCopy(ConfigData->state, ConfigData->numStates,
		&(ConfigData->SP[0 * ConfigData->numStates + 0]));

	sunlineStateProp(&(ConfigData->SP[0 * ConfigData->numStates + 0]),
        ConfigData->dt);
	vCopy(&(ConfigData->SP[0 * ConfigData->numStates + 0]),
		ConfigData->numStates, xBar);
	vScale(ConfigData->wM[0], xBar, ConfigData->numStates, xBar);

	for (i = 0; i<ConfigData->countHalfSPs; i++)
	{
		Index = i + 1;
		spPtr = &(ConfigData->SP[Index*ConfigData->numStates]);
		vCopy(&sBarT[i*ConfigData->numStates], ConfigData->numStates, spPtr);
		vScale(ConfigData->gamma, spPtr, ConfigData->numStates, spPtr);
		vAdd(spPtr, ConfigData->numStates, ConfigData->state, spPtr);
		sunlineStateProp(spPtr, ConfigData->dt);
		vScale(ConfigData->wM[Index], spPtr, ConfigData->numStates, xComp);
		vAdd(xComp, ConfigData->numStates, xBar, xBar);
		
		Index = i + 1 + ConfigData->countHalfSPs;
        spPtr = &(ConfigData->SP[Index*ConfigData->numStates]);
        vCopy(&sBarT[i*ConfigData->numStates], ConfigData->numStates, spPtr);
        vScale(-ConfigData->gamma, spPtr, ConfigData->numStates, spPtr);
        vAdd(spPtr, ConfigData->numStates, ConfigData->state, spPtr);
        sunlineStateProp(spPtr, ConfigData->dt);
        vScale(ConfigData->wM[Index], spPtr, ConfigData->numStates, xComp);
        vAdd(xComp, ConfigData->numStates, xBar, xBar);
	}
    mSetZero(AT, (2 * ConfigData->countHalfSPs + ConfigData->numStates),
        ConfigData->countHalfSPs);
	
	for (i = 0; i<2 * ConfigData->countHalfSPs; i++)
	{
		
        vScale(-1.0, xBar, ConfigData->numStates, aRow);
        vAdd(aRow, ConfigData->numStates,
             &(ConfigData->SP[(i+1)*ConfigData->numStates]), aRow);
        vScale(sqrt(ConfigData->wC[i+1]), aRow, ConfigData->numStates, aRow);
		memcpy((void *)&AT[i*ConfigData->numStates], (void *)aRow,
			ConfigData->numStates*sizeof(double));
	}
	memcpy(&AT[2 * ConfigData->countHalfSPs*ConfigData->numStates],
		ConfigData->sQnoise, ConfigData->numStates*ConfigData->numStates
        *sizeof(double));
    ukfQRDJustR(AT, 2 * ConfigData->countHalfSPs + ConfigData->numStates,
                ConfigData->countHalfSPs, rAT);
	
    mCopy(rAT, ConfigData->numStates, ConfigData->numStates, sBarT);
    mTranspose(sBarT, ConfigData->numStates, ConfigData->numStates,
        ConfigData->sBar);
    vScale(-1.0, xBar, ConfigData->numStates, xErr);
    vAdd(xErr, ConfigData->numStates, &ConfigData->SP[0], xErr);
    ukfCholDownDate(ConfigData->sBar, xErr, ConfigData->wC[0],
        ConfigData->numStates, sBarUp);
    mCopy(sBarUp, ConfigData->numStates, ConfigData->numStates, ConfigData->sBar);

    mTranspose(ConfigData->sBar, ConfigData->numStates, ConfigData->numStates,
        ConfigData->covar);
	mMultM(ConfigData->sBar, ConfigData->numStates, ConfigData->numStates,
        ConfigData->covar, ConfigData->numStates, ConfigData->numStates,
           ConfigData->covar);
    vCopy(&(ConfigData->SP[0]), ConfigData->numStates, ConfigData->state );
	
	ConfigData->timeTag = updateTime;
}

/*! This method computes what the expected measurement vector is for each CSS 
    that is present on the spacecraft.  All data is transacted from the main 
    data structure for the model because there are many variables that would 
    have to be updated otherwise.
 @return void
 @param ConfigData The configuration data associated with the CSS estimator

 */
void sunlineUKFMeasModel(SunlineUKFConfig *ConfigData)
{
    uint32_t i, j, obsCounter;
    double sensorNormal[3];
    obsCounter = 0;
    for(i=0; i<ConfigData->numCSSTotal; i++)
    {
        if(ConfigData->rawSensorData.CosValue[i] > ConfigData->sensorUseThresh)
        {
            v3Copy(&(ConfigData->cssNHat_B[i*3]), sensorNormal);
            ConfigData->obs[obsCounter] = ConfigData->rawSensorData.CosValue[i];
            for(j=0; j<ConfigData->countHalfSPs*2+1; j++)
            {
                ConfigData->yMeas[obsCounter*(ConfigData->countHalfSPs*2+1) + j] =
                    v3Dot(&(ConfigData->SP[j*SKF_N_STATES]), sensorNormal);
            }
            obsCounter++;
        }
    }
    mTranspose(ConfigData->yMeas, obsCounter, ConfigData->countHalfSPs*2+1,
        ConfigData->yMeas);
    ConfigData->numObs = obsCounter;
    
}

/*! This method performs the measurement update for the sunline kalman filter.
 It applies the observations in the obs vectors to the current state estimate and 
 updates the state/covariance with that information.
 @return void
 @param ConfigData The configuration data associated with the CSS estimator
 @param updateTime The time that we need to fix the filter to (seconds)
 */
void sunlineUKFMeasUpdate(SunlineUKFConfig *ConfigData, double updateTime)
{
    uint32_t i;
    double yBar[MAX_N_CSS_MEAS], syInv[MAX_N_CSS_MEAS*MAX_N_CSS_MEAS];
    double kMat[SKF_N_STATES*MAX_N_CSS_MEAS];
    double xHat[SKF_N_STATES], sBarT[SKF_N_STATES*SKF_N_STATES], tempYVec[MAX_N_CSS_MEAS];
    double AT[(2 * SKF_N_STATES + MAX_N_CSS_MEAS)*MAX_N_CSS_MEAS], qChol[MAX_N_CSS_MEAS*MAX_N_CSS_MEAS];
    double rAT[MAX_N_CSS_MEAS*MAX_N_CSS_MEAS], syT[MAX_N_CSS_MEAS*MAX_N_CSS_MEAS];
    double sy[MAX_N_CSS_MEAS*MAX_N_CSS_MEAS];
    double updMat[MAX_N_CSS_MEAS*MAX_N_CSS_MEAS], pXY[SKF_N_STATES*MAX_N_CSS_MEAS];
    sunlineUKFMeasModel(ConfigData);
    vSetZero(yBar, ConfigData->numObs);
    for(i=0; i<ConfigData->countHalfSPs*2+1; i++)
    {
        vCopy(&(ConfigData->yMeas[i*ConfigData->numObs]), ConfigData->numObs,
              tempYVec);
        vScale(ConfigData->wM[i], tempYVec, ConfigData->numObs, tempYVec);
        vAdd(yBar, ConfigData->numObs, tempYVec, yBar);
    }
    mSetZero(AT, ConfigData->countHalfSPs*2+ConfigData->numObs,
        ConfigData->numObs);
    for(i=0; i<ConfigData->countHalfSPs*2; i++)
    {
        vScale(-1.0, yBar, ConfigData->numObs, tempYVec);
        vAdd(tempYVec, ConfigData->numObs,
             &(ConfigData->yMeas[(i+1)*ConfigData->numObs]), tempYVec);
        vScale(sqrt(ConfigData->wC[i+1]), tempYVec, ConfigData->numObs, tempYVec);
        memcpy(&(AT[i*ConfigData->numObs]), tempYVec,
               ConfigData->numObs*sizeof(double));
    }
    mSetZero(ConfigData->qObs, ConfigData->numCSSTotal, ConfigData->numCSSTotal);
    mSetIdentity(ConfigData->qObs, ConfigData->numObs, ConfigData->numObs);
    mScale(ConfigData->qObsVal, ConfigData->qObs, ConfigData->numObs,
           ConfigData->numObs, ConfigData->qObs);
    ukfCholDecomp(ConfigData->qObs, ConfigData->numObs, ConfigData->numObs, qChol);
    memcpy(&(AT[2*ConfigData->countHalfSPs*ConfigData->numObs]),
           qChol, ConfigData->numObs*ConfigData->numObs*sizeof(double));
    ukfQRDJustR(AT, 2*ConfigData->countHalfSPs+ConfigData->numObs,
                ConfigData->numObs, rAT);
    mCopy(rAT, ConfigData->numObs, ConfigData->numObs, syT);
    mTranspose(syT, ConfigData->numObs, ConfigData->numObs, sy);
    vScale(-1.0, yBar, ConfigData->numObs, tempYVec);
    vAdd(tempYVec, ConfigData->numObs, &(ConfigData->yMeas[0]), tempYVec);
    ukfCholDownDate(sy, tempYVec, ConfigData->wC[0],
                    ConfigData->numObs, updMat);
    mCopy(updMat, ConfigData->numObs, ConfigData->numObs, sy);
    mTranspose(sy, ConfigData->numObs, ConfigData->numObs, syT);

    mSetZero(pXY, ConfigData->numStates, ConfigData->numObs);

    for(i=0; i<2*ConfigData->countHalfSPs+1; i++)
    {
        vScale(-1.0, yBar, ConfigData->numObs, tempYVec);
        vAdd(tempYVec, ConfigData->numObs,
             &(ConfigData->yMeas[i*ConfigData->numObs]), tempYVec);
        vSubtract(&(ConfigData->SP[i*ConfigData->numStates]), ConfigData->numStates,
                  &(ConfigData->SP[0]), xHat);
        vScale(ConfigData->wC[i], xHat, ConfigData->numStates, xHat);
        mMultM(xHat, ConfigData->numStates, 1, tempYVec, 1, ConfigData->numObs,
            kMat);
        mAdd(pXY, ConfigData->numStates, ConfigData->numObs, kMat, pXY);
    }

    ukfUInv(syT, ConfigData->numObs, ConfigData->numObs, syInv);
    
    mMultM(pXY, ConfigData->numStates, ConfigData->numObs, syInv,
           ConfigData->numObs, ConfigData->numObs, kMat);
    ukfLInv(sy, ConfigData->numObs, ConfigData->numObs, syInv);
    mMultM(kMat, ConfigData->numStates, ConfigData->numObs, syInv,
           ConfigData->numObs, ConfigData->numObs, kMat);
    vSubtract(ConfigData->obs, ConfigData->numObs, yBar, tempYVec);
    mMultM(kMat, ConfigData->numStates, ConfigData->numObs, tempYVec,
        ConfigData->numObs, 1, xHat);
    vAdd(ConfigData->state, ConfigData->numStates, xHat, ConfigData->state);
    mMultM(kMat, ConfigData->numStates, ConfigData->numObs, sy,
           ConfigData->numObs, ConfigData->numObs, pXY);
    mTranspose(pXY, ConfigData->numStates, ConfigData->numObs, pXY);
    for(i=0; i<ConfigData->numObs; i++)
    {
        vCopy(&(pXY[i*ConfigData->numStates]), ConfigData->numStates, tempYVec);
        ukfCholDownDate(ConfigData->sBar, tempYVec, -1.0, ConfigData->numStates, sBarT);
        mCopy(sBarT, ConfigData->numStates, ConfigData->numStates,
            ConfigData->sBar);
    }
    
}