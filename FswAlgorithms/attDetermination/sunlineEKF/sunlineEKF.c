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

#include "attDetermination/sunlineEKF/sunlineEKF.h"
#include "attDetermination/_GeneralModuleFiles/ukfUtilities.h"
#include "SimCode/utilities/linearAlgebra.h"
#include "SimCode/utilities/rigidBodyKinematics.h"
#include "simFswInterfaceMessages/macroDefinitions.h"
#include "vehicleConfigData/vehicleConfigData.h"
#include <string.h>
#include <math.h>

/*! This method initializes the ConfigData for theCSS WLS estimator.
 It checks to ensure that the inputs are sane and then creates the
 output message
 @return void
 @param ConfigData The configuration data associated with the CSS WLS estimator
 */
void SelfInit_sunlineEKF(sunlineEKFConfig *ConfigData, uint64_t moduleID)
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
void CrossInit_sunlineEKF(sunlineEKFConfig *ConfigData, uint64_t moduleID)
{
    /*! Begin method steps */
    /*! - Find the message ID for the coarse sun sensor data message */
    ConfigData->cssDataInMsgId = subscribeToMessage(ConfigData->cssDataInMsgName,
        sizeof(CSSArraySensorIntMsg), moduleID);
    /*! - Find the message ID for the vehicle mass properties configuration message */
    ConfigData->massPropsInMsgId = subscribeToMessage(ConfigData->massPropsInMsgName,
        sizeof(VehicleConfigFswMsg), moduleID);
    /*! - Find the message ID for the coarse sun sensor configuration message */
    ConfigData->cssConfInMsgId = subscribeToMessage(ConfigData->cssConfInMsgName,
                                                   sizeof(CSSConstConfig), moduleID);
    
    
}

/*! This method resets the sunline attitude filter to an initial state and
 initializes the internal estimation matrices.
 @return void
 @param ConfigData The configuration data associated with the CSS estimator
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void Reset_sunlineEKF(sunlineEKFConfig *ConfigData, uint64_t callTime,
                      uint64_t moduleID)
{
    
    int32_t i;
    VehicleConfigFswMsg massPropsInBuffer;
    CSSConstConfig cssConfigInBuffer;
    uint64_t writeTime;
    uint32_t writeSize;
    
    /*! Begin method steps*/
    /*! - Zero the local configuration data structures and outputs */
    memset(&massPropsInBuffer, 0x0 ,sizeof(VehicleConfigFswMsg));
    memset(&cssConfigInBuffer, 0x0, sizeof(CSSConstConfig));
    memset(&(ConfigData->outputSunline), 0x0, sizeof(NavAttIntMsg));
    
    /*! - Read in mass properties and coarse sun sensor configuration information.*/
    ReadMessage(ConfigData->massPropsInMsgId, &writeTime, &writeSize,
                sizeof(VehicleConfigFswMsg), &massPropsInBuffer, moduleID);
    ReadMessage(ConfigData->cssConfInMsgId, &writeTime, &writeSize,
                sizeof(CSSConstConfig), &cssConfigInBuffer, moduleID);
    
    /*! - For each coarse sun sensor, convert the configuration data over from structure to body*/
    for(i=0; i<cssConfigInBuffer.nCSS; i = i+1)
    {
        m33MultV3(RECAST3X3 massPropsInBuffer.dcm_BS, cssConfigInBuffer.cssVals[i].nHat_S,
                  ConfigData->cssNHat_B[i]);
    }
    /*! - Save the count of sun sensors for later use */
    ConfigData->numCSSTotal = cssConfigInBuffer.nCSS;
    
    /*! - Initialize filter parameters to max values */
    ConfigData->timeTag = callTime*NANO2SEC;
    ConfigData->dt = 0.0;
    ConfigData->numStates = SKF_N_STATES;
    ConfigData->numObs = MAX_N_CSS_MEAS;
    
    /*! - Ensure that all internal filter matrices are zeroed*/
    vSetZero(ConfigData->obs, ConfigData->numObs);
    vSetZero(ConfigData->x, ConfigData->numStates);
    mSetZero(ConfigData->covarBar, ConfigData->numStates, ConfigData->numStates);
    mSetZero(ConfigData->covar, ConfigData->numStates, ConfigData->numStates);
    
    mSetIdentity(ConfigData->stateTransition, ConfigData->numStates, ConfigData->numStates);
    mSetZero(ConfigData->dynMat, ConfigData->numStates, ConfigData->numStates);
    mSetZero(ConfigData->measMat, ConfigData->numStates, ConfigData->numStates);
    
    mSetZero(ConfigData->procNoise, ConfigData->numStates, ConfigData->numStates);
    mSetZero(ConfigData->measNoise, ConfigData->numStates, ConfigData->numStates);
    
    return;
}

/*! This method takes the parsed CSS sensor data and outputs an estimate of the
 sun vector in the ADCS body frame
 @return void
 @param ConfigData The configuration data associated with the CSS estimator
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void Update_sunlineEKF(sunlineEKFConfig *ConfigData, uint64_t callTime,
    uint64_t moduleID)
{
    double newTimeTag;
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
    
    /*! - If the time tag from the measured data is new compared to previous step, 
          propagate and update the filter*/
    newTimeTag = ClockTime * NANO2SEC;
    if(newTimeTag >= ConfigData->timeTag && ReadSize > 0)
    {
        sunlineEKFTimeUpdate(ConfigData, newTimeTag);
        sunlineEKFMeasUpdate(ConfigData, newTimeTag);
    }
    
    /*! - If current clock time is further ahead than the measured time, then
          propagate to this current time-step*/
    newTimeTag = callTime*NANO2SEC;
    if(newTimeTag > ConfigData->timeTag)
    {
        sunlineEKFTimeUpdate(ConfigData, newTimeTag);
    }
    
    /*! - Write the sunline estimate into the copy of the navigation message structure*/
	v3Copy(ConfigData->states, ConfigData->outputSunline.vehSunPntBdy);
    v3Normalize(ConfigData->outputSunline.vehSunPntBdy,
        ConfigData->outputSunline.vehSunPntBdy);
    ConfigData->outputSunline.timeTag = ConfigData->timeTag;
	WriteMessage(ConfigData->navStateOutMsgId, callTime, sizeof(NavAttIntMsg),
		&(ConfigData->outputSunline), moduleID);
    
    /*! - Populate the filter states output buffer and write the output message*/
    sunlineDataOutBuffer.timeTag = ConfigData->timeTag;
    sunlineDataOutBuffer.numObs = ConfigData->numObs;
    memmove(sunlineDataOutBuffer.covar, ConfigData->covar,
            SKF_N_STATES*SKF_N_STATES*sizeof(double));
    memmove(sunlineDataOutBuffer.state, ConfigData->states, SKF_N_STATES*sizeof(double));
    WriteMessage(ConfigData->filtDataOutMsgId, callTime, sizeof(SunlineFilterFswMsg),
                 &sunlineDataOutBuffer, moduleID);
    
    return;
}

/*! This method propagates a sunline state vector forward in time.  Note 
    that the calling parameter is updated in place to save on data copies.
	@return void
	@param stateInOut The state that is propagated
*/
void sunlineStateSTMProp(double *stateInOut, double (*STMin)[6][6], double (*A)[6][6], double dt)
{

    double propagatedVel[3];
    double pointUnit[3];
    double unitComp;
    double propagatedSTM[6][6];
    double deltatASTM[6][6];
    
    /*! Begin state update steps */
    /*! - Unitize the current estimate to find direction to restrict motion*/
    v3Normalize(stateInOut, pointUnit);
    unitComp = v3Dot(&(stateInOut[3]), pointUnit);
    v3Scale(unitComp, pointUnit, pointUnit);
    /*! - Subtract out rotation in the sunline axis because that is not observable 
          for coarse sun sensors*/
    v3Subtract(&(stateInOut[3]), pointUnit, &(stateInOut[3]));
    v3Scale(dt, &(stateInOut[3]), propagatedVel);
    v3Add(stateInOut, propagatedVel, stateInOut);
    
    /*! Begin STM propagation step */
    
    m66MultM66((*A), (*STMin), deltatASTM);
    m66Scale(dt, deltatASTM, deltatASTM);
    m66Add((*STMin), deltatASTM, propagatedSTM);

	return;
}

/*! This method computes the dynamics matrix, which is the derivative of the
 dynamics F by the state X, evaluated at the reference state. It takes in the
 configure data and updates this A matrix
 @return void
 @param ConfigData The configuration data associated with the estimator
 */

void sunlineDynMatrix(double *states, double (*A)[6][6])
{
    double dovernorm2d[3], dddot, ddtnorm2[3][3];
    double I3[3][3], d2I3[3][3];
    double douterddot[3][3], douterd[3][3], neg2dddot[3][3];
    double secondterm[3][3], firstterm[3][3];
    double normd2;
    double dFdd[3][3], dFdddot[3][3];
    
    /* dFdd */
    mSetIdentity(I3, 3, 3);
    dddot = v3Dot(&(states[3]), &(states[0]));
    normd2 = v3Norm(&(states[0]))*v3Norm(&(states[0]));
                  
    mScale(normd2, I3, 3, 3, d2I3);
    
    v3OuterProduct(&(states[0]), &(states[3]), douterddot);
    m33Scale(-2, douterddot, neg2dddot);
    m33Subtract(d2I3, neg2dddot, secondterm);
    m33Scale(dddot/(normd2*normd2), secondterm, secondterm);
                    
    v3Scale(1/normd2, &(states[0]), dovernorm2d);
    v3OuterProduct(&(states[3]), dovernorm2d, firstterm);
             
    m33Add(firstterm, secondterm, dFdd);
    m33Scale(-1, dFdd, dFdd);

    /* Populate the first 3x3 matrix of the dynamics matrix*/
    m66Set33Matrix(0, 0, dFdd, (*A));
           
    /* dFdddot */
    v3OuterProduct(&(states[0]), &(states[0]), douterd);
    m33Scale(-1/normd2, douterd, ddtnorm2);
    
    m33Add(I3, ddtnorm2, dFdddot);
             
    /* Populate the second 3x3 matrix */
    m66Set33Matrix(0, 3, dFdddot, (*A));

    return;
}

/*! This method performs the time update for the sunline kalman filter.
     It propagates the sigma points forward in time and then gets the current 
	 covariance and state estimates.
	 @return void
     @param ConfigData The configuration data associated with the CSS estimator
     @param updateTime The time that we need to fix the filter to (seconds)
*/
void sunlineEKFTimeUpdate(sunlineEKFConfig *ConfigData, double updateTime)
{
    double stmT[6][6], covPhiT[6][6], qGammaT[3][6], gammaQGammaT[6][6];
    
	/*! Begin method steps*/
	ConfigData->dt = updateTime - ConfigData->timeTag;
    
    /*! - Propagate the previous reference states and STM to the current time */
    sunlineDynMatrix(ConfigData->states, &(ConfigData->dynMat));
    sunlineStateSTMProp(ConfigData->states, &(ConfigData->stateTransition), &(ConfigData->dynMat), ConfigData->dt);

    /* xbar = Phi*x */
    m66MultV6(ConfigData->stateTransition, ConfigData->x, ConfigData->xBar);
    
    /*! - Update the covariance */
    /*Pbar = Phi*P*Phi^T + Gamma*Q*Gamma^T*/
    m66Transpose(ConfigData->stateTransition, stmT);
    m66MultM66(ConfigData->covar, stmT, covPhiT);
    m66MultM66(ConfigData->stateTransition, stmT, ConfigData->covarBar);
    
    /*Compute Gamma and add gammaQGamma^T to Pbar*/
    double Gamma[6][3]={{ConfigData->dt/4,0,0},{0,ConfigData->dt/4,0},{0,0,ConfigData->dt/4},{ConfigData->dt,0,0},{0,ConfigData->dt,0},{0,0,ConfigData->dt}};
    
    mMultMt(ConfigData->procNoise, 3, 3, Gamma, 6, 3, qGammaT);
    mMultM(Gamma, 6, 3, qGammaT, 3, 6, gammaQGammaT);
    m66Add(ConfigData->covarBar, gammaQGammaT, ConfigData->covarBar);
    
    
	ConfigData->timeTag = updateTime;
}




/*! This method computes what the expected measurement vector is for each CSS
 that is present on the spacecraft.  All data is transacted from the main
 data structure for the model because there are many variables that would
 have to be updated otherwise.
 @return void
 @param ConfigData The configuration data associated with the CSS estimator
 
 */

void sunlineEKFMeasModel(sunlineEKFConfig *ConfigData)
{
    uint32_t i, obsCounter;
//    uint32_t j;
//    double sensorNormal[3];
    /* Begin method steps */
    obsCounter = 0;
    /*! - Loop over all available coarse sun sensors and only use ones that meet validity threshold*/
    for(i=0; i<ConfigData->numCSSTotal; i++)
    {
        if(ConfigData->cssSensorInBuffer.CosValue[i] > ConfigData->sensorUseThresh)
        {
//            /*! - For each valid measurement, copy observation value and compute expected obs value on a per sigma-point basis.*/
//            v3Copy(&(ConfigData->cssNHat_B[i*3]), sensorNormal);
//            ConfigData->obs[obsCounter] = ConfigData->cssSensorInBuffer.CosValue[i];
//            for(j=0; j<ConfigData->countHalfSPs*2+1; j++)
//            {
//                ConfigData->yMeas[obsCounter*(ConfigData->countHalfSPs*2+1) + j] =
//                    v3Dot(&(ConfigData->SP[j*SKF_N_STATES]), sensorNormal);
//            }
            obsCounter++;
        }
    }
    /*! - yMeas matrix was set backwards deliberately so we need to transpose it through*/
//    mTranspose(ConfigData->yMeas, obsCounter, ConfigData->countHalfSPs*2+1,
//        ConfigData->yMeas);
    ConfigData->numObs = obsCounter;
    
}

/*! This method performs the measurement update for the sunline kalman filter.
 It applies the observations in the obs vectors to the current state estimate and 
 updates the state/covariance with that information.
 @return void
 @param ConfigData The configuration data associated with the CSS estimator
 @param updateTime The time that we need to fix the filter to (seconds)
 */
void sunlineEKFMeasUpdate(sunlineEKFConfig *ConfigData, double updateTime)
{
    uint32_t i;
    double yBar[MAX_N_CSS_MEAS], syInv[MAX_N_CSS_MEAS*MAX_N_CSS_MEAS];
    double kMat[SKF_N_STATES*MAX_N_CSS_MEAS];
    double xHat[SKF_N_STATES], sBarT[SKF_N_STATES*SKF_N_STATES], tempYVec[MAX_N_CSS_MEAS];
    double AT[(2 * SKF_N_STATES + MAX_N_CSS_MEAS)*MAX_N_CSS_MEAS], qChol[MAX_N_CSS_MEAS*MAX_N_CSS_MEAS];
    double rAT[MAX_N_CSS_MEAS*MAX_N_CSS_MEAS], syT[MAX_N_CSS_MEAS*MAX_N_CSS_MEAS];
    double sy[MAX_N_CSS_MEAS*MAX_N_CSS_MEAS];
    double updMat[MAX_N_CSS_MEAS*MAX_N_CSS_MEAS], pXY[SKF_N_STATES*MAX_N_CSS_MEAS];
    
    /*! Begin method steps*/
    
    /*! - Compute the valid observations and the measurement model for all observations*/
    sunlineEKFMeasModel(ConfigData);
    
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
