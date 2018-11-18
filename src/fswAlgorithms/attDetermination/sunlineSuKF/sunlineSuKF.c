/*
 ISC License

 Copyright (c) 2016-2018, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#include "attDetermination/sunlineSuKF/sunlineSuKF.h"
#include "attDetermination/_GeneralModuleFiles/ukfUtilities.h"
#include "simulation/utilities/linearAlgebra.h"
#include "simulation/utilities/rigidBodyKinematics.h"
#include "simFswInterfaceMessages/macroDefinitions.h"
#include <string.h>
#include <math.h>

/*! This method initializes the ConfigData for theCSS WLS estimator.
 It checks to ensure that the inputs are sane and then creates the
 output message
 @return void
 @param ConfigData The configuration data associated with the CSS WLS estimator
 */
void SelfInit_sunlineSuKF(SunlineSuKFConfig *ConfigData, uint64_t moduleID)
{
    
    mSetZero(ConfigData->cssNHat_B, MAX_NUM_CSS_SENSORS, 3);
    /*! Begin method steps */
    /*! - Create output message for module */
	ConfigData->navStateOutMsgId = CreateNewMessage(ConfigData->navStateOutMsgName,
		sizeof(NavAttIntMsg), "NavAttIntMsg", moduleID);
    /*! - Create filter states output message which is mostly for debug*/
    ConfigData->filtDataOutMsgId = CreateNewMessage(ConfigData->filtDataOutMsgName,
        sizeof(SunlineFilterFswMsg), "SunlineFilterFswMsg", moduleID);
    
}

/*! This method performs the second stage of initialization for the CSS sensor
 interface.  It's primary function is to link the input messages that were
 created elsewhere.
 @return void
 @param ConfigData The configuration data associated with the CSS interface
 */
void CrossInit_sunlineSuKF(SunlineSuKFConfig *ConfigData, uint64_t moduleID)
{
    /*! Begin method steps */
    /*! - Find the message ID for the coarse sun sensor data message */
    ConfigData->cssDataInMsgId = subscribeToMessage(ConfigData->cssDataInMsgName,
        sizeof(CSSArraySensorIntMsg), moduleID);
    /*! - Find the message ID for the coarse sun sensor configuration message */
    ConfigData->cssConfigInMsgId = subscribeToMessage(ConfigData->cssConfigInMsgName,
                                                   sizeof(CSSConfigFswMsg), moduleID);
    
}

/*! This method resets the sunline attitude filter to an initial state and
 initializes the internal estimation matrices.
 @return void
 @param ConfigData The configuration data associated with the CSS estimator
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void Reset_sunlineSuKF(SunlineSuKFConfig *ConfigData, uint64_t callTime,
                      uint64_t moduleID)
{
    
    int32_t i;
    CSSConfigFswMsg cssConfigInBuffer;
    uint64_t writeTime;
    uint32_t writeSize;
    double tempMatrix[SKF_N_STATES_SWITCH*SKF_N_STATES_SWITCH];
    
    /*! Begin method steps*/
    /*! - Zero the local configuration data structures and outputs */
    memset(&cssConfigInBuffer, 0x0, sizeof(CSSConfigFswMsg));
    memset(&(ConfigData->outputSunline), 0x0, sizeof(NavAttIntMsg));
    
    /*! - Read in mass properties and coarse sun sensor configuration information.*/
    ReadMessage(ConfigData->cssConfigInMsgId, &writeTime, &writeSize,
                sizeof(CSSConfigFswMsg  ), &cssConfigInBuffer, moduleID);
    
    /*! - For each coarse sun sensor, convert the configuration data over from structure to body*/
    for(i=0; i<cssConfigInBuffer.nCSS; i = i+1)
    {
        v3Copy(cssConfigInBuffer.cssVals[i].nHat_B, &(ConfigData->cssNHat_B[i*3]));
        ConfigData->CBias[i] = cssConfigInBuffer.cssVals[i].CBias;
    }
    /*! - Save the count of sun sensors for later use */
    ConfigData->numCSSTotal = cssConfigInBuffer.nCSS;
    
    /*! - Initialize filter parameters to max values */
    ConfigData->timeTag = callTime*NANO2SEC;
    ConfigData->dt = 0.0;
    ConfigData->numStates = SKF_N_STATES_SWITCH;
    ConfigData->countHalfSPs = SKF_N_STATES_SWITCH;
    ConfigData->numObs = MAX_N_CSS_MEAS;
    
    /*! Initalize the filter to use b_1 of the body frame to make frame*/
    v3Set(1, 0, 0, ConfigData->bVec_B);
    ConfigData->switchTresh = 0.866;
    
    /*! - Ensure that all internal filter matrices are zeroed*/
    vSetZero(ConfigData->obs, ConfigData->numObs);
    vSetZero(ConfigData->wM, ConfigData->countHalfSPs * 2 + 1);
    vSetZero(ConfigData->wC, ConfigData->countHalfSPs * 2 + 1);
    mSetZero(ConfigData->sBar, ConfigData->numStates, ConfigData->numStates);
    mSetZero(ConfigData->SP, ConfigData->countHalfSPs * 2 + 1,
             ConfigData->numStates);
    mSetZero(ConfigData->sQnoise, ConfigData->numStates, ConfigData->numStates);
    
    /*! - Set lambda/gamma to standard value for unscented kalman filters */
    ConfigData->lambdaVal = ConfigData->alpha*ConfigData->alpha*
    (ConfigData->numStates + ConfigData->kappa) - ConfigData->numStates;
    ConfigData->gamma = sqrt(ConfigData->numStates + ConfigData->lambdaVal);
    
    
    /*! - Set the wM/wC vectors to standard values for unscented kalman filters*/
    ConfigData->wM[0] = ConfigData->lambdaVal / (ConfigData->numStates +
                                                 ConfigData->lambdaVal);
    ConfigData->wC[0] = ConfigData->lambdaVal / (ConfigData->numStates +
                                                 ConfigData->lambdaVal) + (1 - ConfigData->alpha*ConfigData->alpha + ConfigData->beta);
    for (i = 1; i<ConfigData->countHalfSPs * 2 + 1; i++)
    {
        ConfigData->wM[i] = 1.0 / 2.0*1.0 / (ConfigData->numStates +
                                             ConfigData->lambdaVal);
        ConfigData->wC[i] = ConfigData->wM[i];
    }
    
    /*! - User a cholesky decomposition to obtain the sBar and sQnoise matrices for use in 
          filter at runtime*/
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
    
    return;
}

/*! This method takes the parsed CSS sensor data and outputs an estimate of the
 sun vector in the ADCS body frame
 @return void
 @param ConfigData The configuration data associated with the CSS estimator
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void Update_sunlineSuKF(SunlineSuKFConfig *ConfigData, uint64_t callTime,
    uint64_t moduleID)
{
    double newTimeTag;
    double yBar[MAX_N_CSS_MEAS];
    double tempYVec[MAX_N_CSS_MEAS];
    double sunheading_hat[3];
    double states_BN[SKF_N_STATES_SWITCH];
    int i;
    uint64_t ClockTime;
    uint32_t ReadSize;
    SunlineFilterFswMsg sunlineDataOutBuffer;
    
    /*! Begin method steps*/
    /*! - Read the input parsed CSS sensor data message*/
    ClockTime = 0;
    ReadSize = 0;
    memset(&(ConfigData->cssSensorInBuffer), 0x0, sizeof(CSSArraySensorIntMsg));
    ReadMessage(ConfigData->cssDataInMsgId, &ClockTime, &ReadSize,
        sizeof(CSSArraySensorIntMsg), (void*) (&(ConfigData->cssSensorInBuffer)), moduleID);
    
    v3Normalize(&ConfigData->state[0], sunheading_hat);
    
    
    /*! - Check for switching frames */
    if (v3Dot(ConfigData->bVec_B, sunheading_hat) > ConfigData->switchTresh)
    {
        sunlineSuKFSwitch(ConfigData->bVec_B, ConfigData->state, ConfigData->covar);
    }
    
    /*! - If the time tag from the measured data is new compared to previous step, 
          propagate and update the filter*/
    newTimeTag = ClockTime * NANO2SEC;
    if(newTimeTag >= ConfigData->timeTag && ReadSize > 0)
    {
        sunlineSuKFTimeUpdate(ConfigData, newTimeTag);
        sunlineSuKFMeasUpdate(ConfigData, newTimeTag);
    }
    
    /*! - If current clock time is further ahead than the measured time, then
          propagate to this current time-step*/
    newTimeTag = callTime*NANO2SEC;
    if(newTimeTag > ConfigData->timeTag)
    {
        sunlineSuKFTimeUpdate(ConfigData, newTimeTag);
    }
    
    /*! - Compute Post Fit Residuals, first get Y (eq 22) using the states post fit*/
    sunlineSuKFMeasModel(ConfigData);
    
    /*! - Compute the value for the yBar parameter (equation 23)*/
    vSetZero(yBar, ConfigData->numObs);
    for(i=0; i<ConfigData->countHalfSPs*2+1; i++)
    {
        vCopy(&(ConfigData->yMeas[i*ConfigData->numObs]), ConfigData->numObs,
              tempYVec);
        vScale(ConfigData->wM[i], tempYVec, ConfigData->numObs, tempYVec);
        vAdd(yBar, ConfigData->numObs, tempYVec, yBar);
    }
    
    /*! - The post fits are y- ybar*/
    mSubtract(ConfigData->obs, MAX_N_CSS_MEAS, 1, yBar, ConfigData->postFits);
    
    /*! - Write the sunline estimate into the copy of the navigation message structure*/
	v3Copy(ConfigData->state, ConfigData->outputSunline.vehSunPntBdy);
    v3Normalize(ConfigData->outputSunline.vehSunPntBdy,
        ConfigData->outputSunline.vehSunPntBdy);
    ConfigData->outputSunline.timeTag = ConfigData->timeTag;
	WriteMessage(ConfigData->navStateOutMsgId, callTime, sizeof(NavAttIntMsg),
		&(ConfigData->outputSunline), moduleID);
    
    /* Switch the rates back to omega_BN instead of oemga_SB */
    vCopy(ConfigData->state, SKF_N_STATES_SWITCH, states_BN);
    vScale(-1, &(states_BN[3]), 2, &(states_BN[3]));
    
    /*! - Populate the filter states output buffer and write the output message*/
    sunlineDataOutBuffer.timeTag = ConfigData->timeTag;
    sunlineDataOutBuffer.numObs = ConfigData->numObs;
    memmove(sunlineDataOutBuffer.covar, ConfigData->covar,
            SKF_N_STATES_SWITCH*SKF_N_STATES_SWITCH*sizeof(double));
    memmove(sunlineDataOutBuffer.state, states_BN, SKF_N_STATES_SWITCH*sizeof(double));
    memmove(sunlineDataOutBuffer.postFitRes, ConfigData->postFits, MAX_N_CSS_MEAS*sizeof(double));
    WriteMessage(ConfigData->filtDataOutMsgId, callTime, sizeof(SunlineFilterFswMsg),
                 &sunlineDataOutBuffer, moduleID);
    
    return;
}

/*! This method propagates a sunline state vector forward in time.  Note 
    that the calling parameter is updated in place to save on data copies.
	@return void
	@param stateInOut The state that is propagated
*/
void sunlineStateProp(double *stateInOut, double *b_Vec, double dt)
{

    double propagatedVel[SKF_N_STATES_HALF];
    double omegaCrossd[SKF_N_STATES_HALF];
    double omega_BN_S[SKF_N_STATES_HALF] = {0, -stateInOut[3], -stateInOut[4]};
    double omega_BN_B[SKF_N_STATES_HALF];
    double dcm_BS[SKF_N_STATES_HALF][SKF_N_STATES_HALF];

    mSetZero(dcm_BS, SKF_N_STATES_HALF, SKF_N_STATES_HALF);

    sunlineSuKFComputeDCM_BS(stateInOut, b_Vec, &dcm_BS[0][0]);
    mMultV(dcm_BS, SKF_N_STATES_HALF, SKF_N_STATES_HALF, omega_BN_S, omega_BN_B);
    /* Set local variables to zero*/
    vSetZero(propagatedVel, SKF_N_STATES_HALF);
    
    /*! Begin state update steps */
    /*! Take omega cross d*/
    v3Cross(omega_BN_B, stateInOut, omegaCrossd);
    
    /*! - Multiply omega cross d by dt and add to state to propagate */
    v3Scale(-dt, omegaCrossd, propagatedVel);
    v3Add(stateInOut, propagatedVel, stateInOut);
    
	return;
}

/*! This method performs the time update for the sunline kalman filter.
     It propagates the sigma points forward in time and then gets the current 
	 covariance and state estimates.
	 @return void
     @param ConfigData The configuration data associated with the CSS estimator
     @param updateTime The time that we need to fix the filter to (seconds)
*/
void sunlineSuKFTimeUpdate(SunlineSuKFConfig *ConfigData, double updateTime)
{
	int i, Index;
	double sBarT[SKF_N_STATES_SWITCH*SKF_N_STATES_SWITCH];
	double xComp[SKF_N_STATES_SWITCH], AT[(2 * SKF_N_STATES_SWITCH + SKF_N_STATES_SWITCH)*SKF_N_STATES_SWITCH];
	double aRow[SKF_N_STATES_SWITCH], rAT[SKF_N_STATES_SWITCH*SKF_N_STATES_SWITCH], xErr[SKF_N_STATES_SWITCH]; 
	double sBarUp[SKF_N_STATES_SWITCH*SKF_N_STATES_SWITCH];
	double *spPtr;
	/*! Begin method steps*/
	ConfigData->dt = updateTime - ConfigData->timeTag;
    
    /*! - Copy over the current state estimate into the 0th Sigma point and propagate by dt*/
	vCopy(ConfigData->state, ConfigData->numStates,
		&(ConfigData->SP[0 * ConfigData->numStates + 0]));
	sunlineStateProp(&(ConfigData->SP[0 * ConfigData->numStates + 0]), ConfigData->bVec_B, ConfigData->dt);
    /*! - Scale that Sigma point by the appopriate scaling factor (Wm[0])*/
	vScale(ConfigData->wM[0], &(ConfigData->SP[0 * ConfigData->numStates + 0]),
        ConfigData->numStates, ConfigData->xBar);
    /*! - Get the transpose of the sBar matrix because it is easier to extract Rows vs columns*/
    mTranspose(ConfigData->sBar, ConfigData->numStates, ConfigData->numStates,
               sBarT);
    /*! - For each Sigma point, apply sBar-based error, propagate forward, and scale by Wm just like 0th.
          Note that we perform +/- sigma points simultaneously in loop to save loop values.*/
	for (i = 0; i<ConfigData->countHalfSPs; i++)
	{
		Index = i + 1;
		spPtr = &(ConfigData->SP[Index*ConfigData->numStates]);
		vCopy(&sBarT[i*ConfigData->numStates], ConfigData->numStates, spPtr);
		vScale(ConfigData->gamma, spPtr, ConfigData->numStates, spPtr);
		vAdd(spPtr, ConfigData->numStates, ConfigData->state, spPtr);
		sunlineStateProp(spPtr, ConfigData->bVec_B, ConfigData->dt);
		vScale(ConfigData->wM[Index], spPtr, ConfigData->numStates, xComp);
		vAdd(xComp, ConfigData->numStates, ConfigData->xBar, ConfigData->xBar);
		
		Index = i + 1 + ConfigData->countHalfSPs;
        spPtr = &(ConfigData->SP[Index*ConfigData->numStates]);
        vCopy(&sBarT[i*ConfigData->numStates], ConfigData->numStates, spPtr);
        vScale(-ConfigData->gamma, spPtr, ConfigData->numStates, spPtr);
        vAdd(spPtr, ConfigData->numStates, ConfigData->state, spPtr);
        sunlineStateProp(spPtr, ConfigData->bVec_B, ConfigData->dt);
        vScale(ConfigData->wM[Index], spPtr, ConfigData->numStates, xComp);
        vAdd(xComp, ConfigData->numStates, ConfigData->xBar, ConfigData->xBar);
	}
    /*! - Zero the AT matrix prior to assembly*/
    mSetZero(AT, (2 * ConfigData->countHalfSPs + ConfigData->numStates),
        ConfigData->countHalfSPs);
	/*! - Assemble the AT matrix.  Note that this matrix is the internals of 
          the qr decomposition call in the source design documentation.  It is 
          the inside of equation 20 in that document*/
	for (i = 0; i<2 * ConfigData->countHalfSPs; i++)
	{
		
        vScale(-1.0, ConfigData->xBar, ConfigData->numStates, aRow);
        vAdd(aRow, ConfigData->numStates,
             &(ConfigData->SP[(i+1)*ConfigData->numStates]), aRow);
        vScale(sqrt(ConfigData->wC[i+1]), aRow, ConfigData->numStates, aRow);
		memcpy((void *)&AT[i*ConfigData->numStates], (void *)aRow,
			ConfigData->numStates*sizeof(double));
	}
    /*! - Pop the sQNoise matrix on to the end of AT prior to getting QR decomposition*/
	memcpy(&AT[2 * ConfigData->countHalfSPs*ConfigData->numStates],
		ConfigData->sQnoise, ConfigData->numStates*ConfigData->numStates
        *sizeof(double));
    /*! - QR decomposition (only R computed!) of the AT matrix provides the new sBar matrix*/
    ukfQRDJustR(AT, 2 * ConfigData->countHalfSPs + ConfigData->numStates,
                ConfigData->countHalfSPs, rAT);
    mCopy(rAT, ConfigData->numStates, ConfigData->numStates, sBarT);
    mTranspose(sBarT, ConfigData->numStates, ConfigData->numStates,
        ConfigData->sBar);
    
    /*! - Shift the sBar matrix over by the xBar vector using the appropriate weight 
          like in equation 21 in design document.*/
    vScale(-1.0, ConfigData->xBar, ConfigData->numStates, xErr);
    vAdd(xErr, ConfigData->numStates, &ConfigData->SP[0], xErr);
    ukfCholDownDate(ConfigData->sBar, xErr, ConfigData->wC[0],
        ConfigData->numStates, sBarUp);
    
    /*! - Save current sBar matrix, covariance, and state estimate off for further use*/
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
void sunlineSuKFMeasModel(SunlineSuKFConfig *ConfigData)
{
    uint32_t i, j, obsCounter;
    double sensorNormal[3];
    /* Begin method steps */
    obsCounter = 0;
    /*! - Loop over all available coarse sun sensors and only use ones that meet validity threshold*/
    for(i=0; i<ConfigData->numCSSTotal; i++)
    {
        if(ConfigData->cssSensorInBuffer.CosValue[i] > ConfigData->sensorUseThresh)
        {
            /*! - For each valid measurement, copy observation value and compute expected obs value 
                  on a per sigma-point basis.*/
            v3Scale(ConfigData->CBias[i], &(ConfigData->cssNHat_B[i*3]), sensorNormal);
            ConfigData->obs[obsCounter] = ConfigData->cssSensorInBuffer.CosValue[i];
            for(j=0; j<ConfigData->countHalfSPs*2+1; j++)
            {
                ConfigData->yMeas[obsCounter*(ConfigData->countHalfSPs*2+1) + j] =
                    v3Dot(&(ConfigData->SP[j*SKF_N_STATES_SWITCH]), sensorNormal);
            }
            obsCounter++;
        }
    }
    /*! - yMeas matrix was set backwards deliberately so we need to transpose it through*/
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
void sunlineSuKFMeasUpdate(SunlineSuKFConfig *ConfigData, double updateTime)
{
    uint32_t i;
    double yBar[MAX_N_CSS_MEAS], syInv[MAX_N_CSS_MEAS*MAX_N_CSS_MEAS];
    double kMat[SKF_N_STATES_SWITCH*MAX_N_CSS_MEAS];
    double xHat[SKF_N_STATES_SWITCH], sBarT[SKF_N_STATES_SWITCH*SKF_N_STATES_SWITCH], tempYVec[MAX_N_CSS_MEAS];
    double AT[(2 * SKF_N_STATES_SWITCH + MAX_N_CSS_MEAS)*MAX_N_CSS_MEAS], qChol[MAX_N_CSS_MEAS*MAX_N_CSS_MEAS];
    double rAT[MAX_N_CSS_MEAS*MAX_N_CSS_MEAS], syT[MAX_N_CSS_MEAS*MAX_N_CSS_MEAS];
    double sy[MAX_N_CSS_MEAS*MAX_N_CSS_MEAS];
    double updMat[MAX_N_CSS_MEAS*MAX_N_CSS_MEAS], pXY[SKF_N_STATES_SWITCH*MAX_N_CSS_MEAS];
    
    /*! Begin method steps*/
    
    /*! - Compute the valid observations and the measurement model for all observations*/
    sunlineSuKFMeasModel(ConfigData);
    
    /*! - Compute the value for the yBar parameter (note that this is equation 23 in the 
          time update section of the reference document*/
    vSetZero(yBar, ConfigData->numObs);
    for(i=0; i<ConfigData->countHalfSPs*2+1; i++)
    {
        vCopy(&(ConfigData->yMeas[i*ConfigData->numObs]), ConfigData->numObs,
              tempYVec);
        vScale(ConfigData->wM[i], tempYVec, ConfigData->numObs, tempYVec);
        vAdd(yBar, ConfigData->numObs, tempYVec, yBar);
    }
    
    /*! - Populate the matrix that we perform the QR decomposition on in the measurement 
          update section of the code.  This is based on the differenence between the yBar 
          parameter and the calculated measurement models.  Equation 24 in driving doc. */
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
    
    /*! - This is the square-root of the Rk matrix which we treat as the Cholesky
        decomposition of the observation variance matrix constructed for our number 
        of observations*/
    mSetZero(ConfigData->qObs, ConfigData->numCSSTotal, ConfigData->numCSSTotal);
    mSetIdentity(ConfigData->qObs, ConfigData->numObs, ConfigData->numObs);
    mScale(ConfigData->qObsVal, ConfigData->qObs, ConfigData->numObs,
           ConfigData->numObs, ConfigData->qObs);
    ukfCholDecomp(ConfigData->qObs, ConfigData->numObs, ConfigData->numObs, qChol);
    memcpy(&(AT[2*ConfigData->countHalfSPs*ConfigData->numObs]),
           qChol, ConfigData->numObs*ConfigData->numObs*sizeof(double));
    /*! - Perform QR decomposition (only R again) of the above matrix to obtain the 
          current Sy matrix*/
    ukfQRDJustR(AT, 2*ConfigData->countHalfSPs+ConfigData->numObs,
                ConfigData->numObs, rAT);
    mCopy(rAT, ConfigData->numObs, ConfigData->numObs, syT);
    mTranspose(syT, ConfigData->numObs, ConfigData->numObs, sy);
    /*! - Shift the matrix over by the difference between the 0th SP-based measurement 
          model and the yBar matrix (cholesky down-date again)*/
    vScale(-1.0, yBar, ConfigData->numObs, tempYVec);
    vAdd(tempYVec, ConfigData->numObs, &(ConfigData->yMeas[0]), tempYVec);
    ukfCholDownDate(sy, tempYVec, ConfigData->wC[0],
                    ConfigData->numObs, updMat);
    /*! - Shifted matrix represents the Sy matrix */
    mCopy(updMat, ConfigData->numObs, ConfigData->numObs, sy);
    mTranspose(sy, ConfigData->numObs, ConfigData->numObs, syT);

    /*! - Construct the Pxy matrix (equation 26) which multiplies the Sigma-point cloud 
          by the measurement model cloud (weighted) to get the total Pxy matrix*/
    mSetZero(pXY, ConfigData->numStates, ConfigData->numObs);
    for(i=0; i<2*ConfigData->countHalfSPs+1; i++)
    {
        vScale(-1.0, yBar, ConfigData->numObs, tempYVec);
        vAdd(tempYVec, ConfigData->numObs,
             &(ConfigData->yMeas[i*ConfigData->numObs]), tempYVec);
        vSubtract(&(ConfigData->SP[i*ConfigData->numStates]), ConfigData->numStates,
                  ConfigData->xBar, xHat);
        vScale(ConfigData->wC[i], xHat, ConfigData->numStates, xHat);
        mMultM(xHat, ConfigData->numStates, 1, tempYVec, 1, ConfigData->numObs,
            kMat);
        mAdd(pXY, ConfigData->numStates, ConfigData->numObs, kMat, pXY);
    }

    /*! - Then we need to invert the SyT*Sy matrix to get the Kalman gain factor.  Since
          The Sy matrix is lower triangular, we can do a back-sub inversion instead of 
          a full matrix inversion.  That is the ukfUInv and ukfLInv calls below.  Once that 
          multiplication is done (equation 27), we have the Kalman Gain.*/
    ukfUInv(syT, ConfigData->numObs, ConfigData->numObs, syInv);
    
    mMultM(pXY, ConfigData->numStates, ConfigData->numObs, syInv,
           ConfigData->numObs, ConfigData->numObs, kMat);
    ukfLInv(sy, ConfigData->numObs, ConfigData->numObs, syInv);
    mMultM(kMat, ConfigData->numStates, ConfigData->numObs, syInv,
           ConfigData->numObs, ConfigData->numObs, kMat);
    
    
    /*! - Difference the yBar and the observations to get the observed error and 
          multiply by the Kalman Gain to get the state update.  Add the state update 
          to the state to get the updated state value (equation 27).*/
    vSubtract(ConfigData->obs, ConfigData->numObs, yBar, tempYVec);
    mMultM(kMat, ConfigData->numStates, ConfigData->numObs, tempYVec,
        ConfigData->numObs, 1, xHat);
    vAdd(ConfigData->state, ConfigData->numStates, xHat, ConfigData->state);
    /*! - Compute the updated matrix U from equation 28.  Note that I then transpose it 
         so that I can extract "columns" from adjacent memory*/
    mMultM(kMat, ConfigData->numStates, ConfigData->numObs, sy,
           ConfigData->numObs, ConfigData->numObs, pXY);
    mTranspose(pXY, ConfigData->numStates, ConfigData->numObs, pXY);
    /*! - For each column in the update matrix, perform a cholesky down-date on it to 
          get the total shifted S matrix (called sBar in internal parameters*/
    for(i=0; i<ConfigData->numObs; i++)
    {
        vCopy(&(pXY[i*ConfigData->numStates]), ConfigData->numStates, tempYVec);
        ukfCholDownDate(ConfigData->sBar, tempYVec, -1.0, ConfigData->numStates, sBarT);
        mCopy(sBarT, ConfigData->numStates, ConfigData->numStates,
            ConfigData->sBar);
    }
    /*! - Compute equivalent covariance based on updated sBar matrix*/
    mTranspose(ConfigData->sBar, ConfigData->numStates, ConfigData->numStates,
               ConfigData->covar);
    mMultM(ConfigData->sBar, ConfigData->numStates, ConfigData->numStates,
           ConfigData->covar, ConfigData->numStates, ConfigData->numStates,
           ConfigData->covar);
}


/*! This method computes the dcms necessary for the switch between the two frames.
 It the switches the states and the covariance, and sets s2 to be the new, different vector of the body frame.
 @return void
 @param covarBar The time updated covariance
 @param hObs The H matrix filled with the observations
 @param s2_B Pointer to the second frame vector
 @param states Pointer to the states
 @param covar Pointer to the covariance
 */

void sunlineSuKFSwitch(double *bVec_B, double *states, double *covar)
{
    double dcm_BSold[SKF_N_STATES_HALF][SKF_N_STATES_HALF];
    double dcm_BSnew_T[SKF_N_STATES_HALF][SKF_N_STATES_HALF];
    double dcm_SnewSold[SKF_N_STATES_HALF][SKF_N_STATES_HALF];
    double switchMatP[SKF_N_STATES_SWITCH][SKF_N_STATES_SWITCH];
    double switchMat[SKF_N_STATES_SWITCH][SKF_N_STATES_SWITCH];
    
    double sun_heading_norm[SKF_N_STATES_HALF];
    double b1[SKF_N_STATES_HALF];
    double b2[SKF_N_STATES_HALF];
    
    /*!  Set the body frame vectors*/
    v3Set(1, 0, 0, b1);
    v3Set(0, 1, 0, b2);
    v3Normalize(&(states[0]), sun_heading_norm);
    
    /*! Populate the dcm_BS with the "old" S-frame*/
    sunlineSuKFComputeDCM_BS(sun_heading_norm, bVec_B, &dcm_BSold[0][0]);
    
    if (v3IsEqual(bVec_B, b1, 1e-10))
    {
        sunlineSuKFComputeDCM_BS(sun_heading_norm, b2, &dcm_BSnew_T[0][0]);
        v3Copy(b2, bVec_B);
    }
    else
    {
        sunlineSuKFComputeDCM_BS(sun_heading_norm, b1, &dcm_BSnew_T[0][0]);
        v3Copy(b1, bVec_B);
    }
    
    mTranspose(dcm_BSnew_T, SKF_N_STATES_HALF, SKF_N_STATES_HALF, dcm_BSnew_T);
    mMultM(dcm_BSnew_T, 3, 3, dcm_BSold, 3, 3, dcm_SnewSold);
    
    mSetIdentity(switchMat, SKF_N_STATES_SWITCH, SKF_N_STATES_SWITCH);
    mSetSubMatrix(&dcm_SnewSold[1][1], 1, 2, &switchMat, SKF_N_STATES_SWITCH, SKF_N_STATES_SWITCH, 3, 3);
    mSetSubMatrix(&dcm_SnewSold[2][1], 1, 2, &switchMat, SKF_N_STATES_SWITCH, SKF_N_STATES_SWITCH, 4, 3);
    
    mMultV(switchMat, SKF_N_STATES_SWITCH, SKF_N_STATES_SWITCH, states, states);
    mMultM(switchMat, SKF_N_STATES_SWITCH, SKF_N_STATES_SWITCH, covar, SKF_N_STATES_SWITCH, SKF_N_STATES_SWITCH, switchMatP);
    mTranspose(switchMat, SKF_N_STATES_SWITCH, SKF_N_STATES_SWITCH, switchMat);
    mMultM(switchMatP, SKF_N_STATES_SWITCH, SKF_N_STATES_SWITCH, switchMat, SKF_N_STATES_SWITCH, SKF_N_STATES_SWITCH, covar);
    return;
}


void sunlineSuKFComputeDCM_BS(double sunheading[SKF_N_STATES_HALF], double bVec[SKF_N_STATES_HALF], double *dcm){
    double s1_B[SKF_N_STATES_HALF];
    double s2_B[SKF_N_STATES_HALF];
    double s3_B[SKF_N_STATES_HALF];
    
    mSetZero(dcm, SKF_N_STATES_HALF, SKF_N_STATES_HALF);
    v3SetZero(s2_B);
    v3SetZero(s3_B);
    
    v3Normalize(sunheading, s1_B);
    v3Cross(sunheading, bVec, s2_B);
    if (v3Norm(s2_B) < 1E-5){
        mSetIdentity(dcm, SKF_N_STATES_HALF, SKF_N_STATES_HALF);
    }
    else{
    v3Normalize(s2_B, s2_B);
    /*! Populate the dcm_BS with the "new" S-frame*/
    mSetSubMatrix(s1_B, 1, SKF_N_STATES_HALF, dcm, SKF_N_STATES_HALF, SKF_N_STATES_HALF, 0, 0);
    mSetSubMatrix(&(s2_B), 1, SKF_N_STATES_HALF, dcm, SKF_N_STATES_HALF, SKF_N_STATES_HALF, 1, 0);
    v3Cross(sunheading, s2_B, s3_B);
    v3Normalize(s3_B, s3_B);
    mSetSubMatrix(&(s3_B), 1, SKF_N_STATES_HALF, dcm, SKF_N_STATES_HALF, SKF_N_STATES_HALF, 2, 0);
    mTranspose(dcm, SKF_N_STATES_HALF, SKF_N_STATES_HALF, dcm);
    }
    
}
