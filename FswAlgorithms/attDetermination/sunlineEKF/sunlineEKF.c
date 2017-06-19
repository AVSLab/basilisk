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
        sunlineTimeUpdate(ConfigData, newTimeTag);
        sunlineMeasUpdate(ConfigData, newTimeTag);
    }
    
    /*! - If current clock time is further ahead than the measured time, then
          propagate to this current time-step*/
    newTimeTag = callTime*NANO2SEC;
    if(newTimeTag > ConfigData->timeTag)
    {
        sunlineTimeUpdate(ConfigData, newTimeTag);
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

/*! This method performs the time update for the sunline kalman filter.
     It propagates the sigma points forward in time and then gets the current 
	 covariance and state estimates.
	 @return void
     @param ConfigData The configuration data associated with the CSS estimator
     @param updateTime The time that we need to fix the filter to (seconds)
*/
void sunlineTimeUpdate(sunlineEKFConfig *ConfigData, double updateTime)
{
    double stmT[SKF_N_STATES][SKF_N_STATES], covPhiT[SKF_N_STATES][SKF_N_STATES], qGammaT[SKF_N_STATES/2][SKF_N_STATES], gammaQGammaT[SKF_N_STATES][SKF_N_STATES];
    
	/*! Begin method steps*/
	ConfigData->dt = updateTime - ConfigData->timeTag;
    
    /*! - Propagate the previous reference states and STM to the current time */
    sunlineDynMatrix(ConfigData->states, &(ConfigData->dynMat));
    sunlineStateSTMProp(ConfigData->dynMat, ConfigData->dt, ConfigData->states, &(ConfigData->stateTransition));

    /* xbar = Phi*x */
    m66MultV6(ConfigData->stateTransition, ConfigData->x, ConfigData->xBar);
    
    /*! - Update the covariance */
    /*Pbar = Phi*P*Phi^T + Gamma*Q*Gamma^T*/
    m66Transpose(ConfigData->stateTransition, stmT);
    m66MultM66(ConfigData->covar, stmT, covPhiT);
    m66MultM66(ConfigData->stateTransition, stmT, ConfigData->covarBar);
    
    /*Compute Gamma and add gammaQGamma^T to Pbar*/
    double Gamma[6][3]={{ConfigData->dt/4,0,0},{0,ConfigData->dt/4,0},{0,0,ConfigData->dt/4},{ConfigData->dt,0,0},{0,ConfigData->dt,0},{0,0,ConfigData->dt}};
    
    mMultMt(ConfigData->procNoise, SKF_N_STATES/2, SKF_N_STATES/2, Gamma, SKF_N_STATES, SKF_N_STATES/2, qGammaT);
    mMultM(Gamma, SKF_N_STATES, SKF_N_STATES/2, qGammaT, SKF_N_STATES/2, SKF_N_STATES, gammaQGammaT);
    m66Add(ConfigData->covarBar, gammaQGammaT, ConfigData->covarBar);
    
    
	ConfigData->timeTag = updateTime;
}


/*! This method propagates a sunline state vector forward in time.  Note
 that the calling parameter is updated in place to save on data copies.
	@return void
	@param stateInOut The state that is propagated
 */
void sunlineStateSTMProp(double dynMat[SKF_N_STATES][SKF_N_STATES], double dt, double *stateInOut, double (*stateTransition)[SKF_N_STATES][SKF_N_STATES])
{
    
    double propagatedVel[3], ddotminusdtilde[3];
    double pointUnit[3];
    double unitComp;
    double deltatASTM[6][6];
    
    /*! Begin state update steps */
    /*! - Unitize the current estimate to find direction to restrict motion*/
    v3Normalize(stateInOut, pointUnit);
    unitComp = v3Dot(&(stateInOut[3]), pointUnit);
    v3Scale(unitComp, pointUnit, pointUnit);
    /*! - Subtract out rotation in the sunline axis because that is not observable
     for coarse sun sensors*/
    v3Subtract(&(stateInOut[3]), pointUnit, ddotminusdtilde);
    v3Scale(dt, ddotminusdtilde, propagatedVel);
    v3Add(stateInOut, propagatedVel, stateInOut);
    
    /*! Begin STM propagation step */
    
    m66MultM66(dynMat, *stateTransition, deltatASTM);
    m66Scale(dt, deltatASTM, deltatASTM);
    m66Add(*stateTransition, deltatASTM, *stateTransition);
    
    return;
}

/*! This method computes the dynamics matrix, which is the derivative of the
 dynamics F by the state X, evaluated at the reference state. It takes in the
 configure data and updates this A matrix
 @return void
 @param ConfigData The configuration data associated with the estimator
 */

void sunlineDynMatrix(double states[SKF_N_STATES], double (*dynMat)[SKF_N_STATES][SKF_N_STATES])
{
    double dddot, ddtnorm2[3][3];
    double I3[3][3], d2I3[3][3];
    double douterddot[3][3], douterd[3][3], neg2dd[3][3];
    double secondterm[3][3], firstterm[3][3];
    double normd2;
    double dFdd[3][3], dFdddot[3][3];
    
    /* dFdd */
    mSetIdentity(I3, 3, 3);
    dddot = v3Dot(&(states[0]), &(states[3]));
    normd2 = v3Norm(&(states[0]))*v3Norm(&(states[0]));
    
    mScale(normd2, I3, 3, 3, d2I3);
    
    v3OuterProduct(&(states[0]), &(states[3]), douterddot);
    v3OuterProduct(&(states[0]), &(states[0]), douterd);
    
    m33Scale(-2.0, douterd, neg2dd);
    m33Add(d2I3, neg2dd, secondterm);
    m33Scale(dddot/(normd2*normd2), secondterm, secondterm);
    
    m33Scale(1.0/normd2, douterddot, firstterm);
    
    m33Add(firstterm, secondterm, dFdd);
    m33Scale(-1.0, dFdd, dFdd);
    
    /* Populate the first 3x3 matrix of the dynamics matrix*/
    m66Set33Matrix(0, 0, dFdd, (*dynMat));
    
    /* dFdddot */
    m33Scale(-1.0/normd2, douterd, ddtnorm2);
    m33Add(I3, ddtnorm2, dFdddot);
    
    /* Populate the second 3x3 matrix */
    m66Set33Matrix(0, 1, dFdddot, (*dynMat));
    
    return;
}


/*! This method performs the measurement update for the sunline kalman filter.
 It applies the observations in the obs vectors to the current state estimate and
 updates the state/covariance with that information.
 @return void
 @param ConfigData The configuration data associated with the CSS estimator
 @param updateTime The time that we need to fix the filter to (seconds)
 */
void sunlineMeasUpdate(sunlineEKFConfig *ConfigData, double updateTime)
{
    uint32_t i;
    double yObs[MAX_N_CSS_MEAS],  hObs[MAX_N_CSS_MEAS][SKF_N_STATES];
    double noise[MAX_N_CSS_MEAS][MAX_N_CSS_MEAS];
    
    /*! Begin method steps*/
    /*! - Compute the valid observations and the measurement model for all observations*/
    sunlineHMatrixYMeas(ConfigData->states, ConfigData->numCSSTotal, ConfigData->cssSensorInBuffer.CosValue, ConfigData->sensorUseThresh, ConfigData->cssNHat_B, ConfigData->obs, ConfigData->yMeas, ConfigData->numObs, &(ConfigData->measMat));
    
    /*! - Compute the value for the yBar parameter (note that this is equation 23 in the
     time update section of the reference document*/
    vSetZero(yObs, ConfigData->numObs);
    mSetZero(hObs, ConfigData->numObs, ConfigData->numStates);
    mSetZero(noise, ConfigData->numObs, ConfigData->numObs);
    
    vCopy(ConfigData->yMeas, ConfigData->numObs, yObs);
    
    for(i=0; i<ConfigData->numObs+1; i++)
    {
        vCopy(ConfigData->measMat[i], SKF_N_STATES, hObs[i]);
        vCopy(ConfigData->measNoise[i], ConfigData->numObs, noise[i]);
    }
    
    /*! - Compute the Kalman Gain. */
    sunlineKalmanGain(ConfigData->covarBar, hObs, ConfigData->measNoise, ConfigData->numObs, &(ConfigData->kalmanGain));
    
    /*! - Compute the update with a CKF */
    sunlineCKFUpdate(ConfigData->xBar, ConfigData->kalmanGain, ConfigData->covarBar, noise, ConfigData->numObs, yObs, hObs, &(ConfigData->x), &(ConfigData->covar));
    /*! - Compute the update with a EKF, notice the reference state is added as an argument because it is changed by the filter update */
    sunlineEKFUpdate(ConfigData->xBar, ConfigData->kalmanGain, ConfigData->covarBar, noise, ConfigData->numObs, yObs, hObs, &(ConfigData->states), &(ConfigData->x), &(ConfigData->covar));
    
}

/*! This method computes the updated states, state error, and covariance for
 the filter in the case that we are using a CKF. This is done in the case 
 where the covariance is too large and needs to be brought down in order to 
 assure filter convergence.
 @return void
 @param ConfigData The configuration data associated with the CSS estimator
 
 */

void sunlineCKFUpdate(double xBar[SKF_N_STATES], double kalmanGain[SKF_N_STATES][MAX_N_CSS_MEAS], double covarBar[SKF_N_STATES][SKF_N_STATES], double noiseMat[MAX_N_CSS_MEAS][MAX_N_CSS_MEAS], int numObs, double yObs[MAX_N_CSS_MEAS], double hObs[MAX_N_CSS_MEAS][SKF_N_STATES], double (*x)[SKF_N_STATES], double (*covar)[SKF_N_STATES][SKF_N_STATES])
{
    double measMatx[numObs], innov[numObs], kInnov[SKF_N_STATES];
    double eye[SKF_N_STATES][SKF_N_STATES], kH[SKF_N_STATES][SKF_N_STATES];
    double eyeKalH[SKF_N_STATES][SKF_N_STATES], eyeKalHT[SKF_N_STATES][SKF_N_STATES];
    double eyeKalHCovarBar[SKF_N_STATES][SKF_N_STATES], kalR[SKF_N_STATES][numObs];
    double kalT[MAX_N_CSS_MEAS][SKF_N_STATES], kalRKalT[SKF_N_STATES][SKF_N_STATES];
    
    /*! - Compute innovation, multiply it my Kalman Gain, and add it to xBar*/
    mMultM(hObs, numObs, SKF_N_STATES, xBar, SKF_N_STATES, 1, measMatx);
    vSubtract(yObs, numObs, measMatx, innov);
    mMultM(kalmanGain, SKF_N_STATES, numObs, innov, numObs, 1, kInnov);
    vAdd(xBar, SKF_N_STATES, kInnov, x);
    
    /*! - Compute new covariance with Joseph's method*/
    mMultM(kalmanGain, SKF_N_STATES, numObs, hObs, numObs, SKF_N_STATES, kH);
    mSetIdentity(eye, SKF_N_STATES, SKF_N_STATES);
    mSubtract(eye, SKF_N_STATES, SKF_N_STATES, kH, eyeKalH);
    mTranspose(eyeKalH, SKF_N_STATES, SKF_N_STATES, eyeKalHT);
    mMultM(eyeKalH, SKF_N_STATES, SKF_N_STATES, covarBar, SKF_N_STATES, SKF_N_STATES, eyeKalHCovarBar);
    mMultM(eyeKalHCovarBar, SKF_N_STATES, SKF_N_STATES, eyeKalHT, SKF_N_STATES, SKF_N_STATES, covar);
    
    /* Add noise to the covariance*/
    mMultM(kalmanGain, SKF_N_STATES, numObs, noiseMat, numObs, numObs, kalR);
    mTranspose(kalmanGain, SKF_N_STATES, numObs, kalT);
    mMultM(kalR, SKF_N_STATES, numObs, kalT, numObs, SKF_N_STATES, kalRKalT);
    mAdd(covar, SKF_N_STATES, SKF_N_STATES, kalRKalT, covar);
    
    
}

/*! This method computes the updated states, state error, and covariance for
 the filter in the case that we are using a EKF. This is done in the case
 where the covariance is small enough.
 @return void
 @param ConfigData The configuration data associated with the CSS estimator
 
 */

void sunlineEKFUpdate(double xBar[SKF_N_STATES], double kalmanGain[SKF_N_STATES][MAX_N_CSS_MEAS], double covarBar[SKF_N_STATES][SKF_N_STATES], double noiseMat[MAX_N_CSS_MEAS][MAX_N_CSS_MEAS], int numObs, double yObs[MAX_N_CSS_MEAS], double hObs[MAX_N_CSS_MEAS][SKF_N_STATES], double (*states)[SKF_N_STATES], double (*x)[SKF_N_STATES], double (*covar)[SKF_N_STATES][SKF_N_STATES])
{

    double eye[SKF_N_STATES][SKF_N_STATES], kH[SKF_N_STATES][SKF_N_STATES];
    double eyeKalH[SKF_N_STATES][SKF_N_STATES], eyeKalHT[SKF_N_STATES][SKF_N_STATES];
    double eyeKalHCovarBar[SKF_N_STATES][SKF_N_STATES], kalR[SKF_N_STATES][numObs];
    double kalT[MAX_N_CSS_MEAS][SKF_N_STATES], kalRKalT[SKF_N_STATES][SKF_N_STATES];
    
    /*! - Update the state error*/
    mMultM(kalmanGain, SKF_N_STATES, numObs, yObs, numObs, 1, x);

    /*! - Change the reference state*/
    mAdd(states, SKF_N_STATES, 1, x, states);
    
    /*! - Compute new covariance with Joseph's method*/
    mMultM(kalmanGain, SKF_N_STATES, numObs, hObs, numObs, SKF_N_STATES, kH);
    mSetIdentity(eye, SKF_N_STATES, SKF_N_STATES);
    mSubtract(eye, SKF_N_STATES, SKF_N_STATES, kH, eyeKalH);
    mTranspose(eyeKalH, SKF_N_STATES, SKF_N_STATES, eyeKalHT);
    mMultM(eyeKalH, SKF_N_STATES, SKF_N_STATES, covarBar, SKF_N_STATES, SKF_N_STATES, eyeKalHCovarBar);
    mMultM(eyeKalHCovarBar, SKF_N_STATES, SKF_N_STATES, eyeKalHT, SKF_N_STATES, SKF_N_STATES, covar);
    
    /* Add noise to the covariance*/
    mMultM(kalmanGain, SKF_N_STATES, numObs, noiseMat, numObs, numObs, kalR);
    mTranspose(kalmanGain, SKF_N_STATES, numObs, kalT);
    mMultM(kalR, SKF_N_STATES, numObs, kalT, numObs, SKF_N_STATES, kalRKalT);
    mAdd(covar, SKF_N_STATES, SKF_N_STATES, kalRKalT, covar);
    
}

/*! This method computes the H matrix, defined by dGdX. As well as computing the 
 innovation, difference between the measurements and the expected measurements.
 This methods modifies the numObs, measMat, and yMeas. 
 @return void
 @param ConfigData The configuration data associated with the CSS estimator
 
 */

void sunlineHMatrixYMeas(double states[SKF_N_STATES], int numCSS, double cssSensorCos[MAX_N_CSS_MEAS], double sensorUseThresh, double cssNHat_B[MAX_NUM_CSS_SENSORS][3], double *obs, double *yMeas, int *numObs, double (*measMat)[MAX_N_CSS_MEAS][SKF_N_STATES])
{
    uint32_t i, obsCounter;
    double sensorNormal[3];
    
    v3SetZero(sensorNormal);

    /* Begin method steps */
    obsCounter = 0;
    /*! - Loop over all available coarse sun sensors and only use ones that meet validity threshold*/
    for(i=0; i<numCSS; i++)
    {
        if(cssSensorCos[i] > sensorUseThresh)
        {
            /*! - For each valid measurement, copy observation value and compute expected obs value and fill out H matrix.*/
            v3Copy(cssNHat_B[i], sensorNormal);
            
            *(obs+obsCounter) = cssSensorCos[i];
            *(yMeas+obsCounter) = cssSensorCos[i];// - v3Dot(&(states[0]), sensorNormal);
            
            v3Copy(cssNHat_B[i], *(measMat+obsCounter)[0]);
            obsCounter++;
        }
    }
    *numObs = obsCounter;
}



/*! This method computes the Kalman gain given the measurements.
 @return void
 @param ConfigData The configuration data associated with the CSS estimator
 
 */

void sunlineKalmanGain(double covarBar[SKF_N_STATES][SKF_N_STATES], double hObs[MAX_N_CSS_MEAS][SKF_N_STATES], double measNoise[MAX_N_CSS_MEAS][MAX_N_CSS_MEAS], int numObs, double (*kalmanGain)[SKF_N_STATES][MAX_N_CSS_MEAS])
{
    double hObsT[SKF_N_STATES][MAX_N_CSS_MEAS];
    double covHT[SKF_N_STATES][MAX_N_CSS_MEAS];
    double hCovar[MAX_N_CSS_MEAS][SKF_N_STATES], hCovarHT[MAX_N_CSS_MEAS][MAX_N_CSS_MEAS], hCovarHTinv[MAX_N_CSS_MEAS][MAX_N_CSS_MEAS];
    double rMat[MAX_N_CSS_MEAS][MAX_N_CSS_MEAS];
    
    /* Setting all local variables to zero */
    mSetZero(hObsT, SKF_N_STATES, MAX_N_CSS_MEAS);
    mSetZero(covHT, SKF_N_STATES, MAX_N_CSS_MEAS);
    mSetZero(hCovar, MAX_N_CSS_MEAS, SKF_N_STATES);
    mSetZero(hCovarHT, MAX_N_CSS_MEAS, MAX_N_CSS_MEAS);
    mSetZero(hCovarHTinv, MAX_N_CSS_MEAS, MAX_N_CSS_MEAS);
    mSetZero(rMat, MAX_N_CSS_MEAS, MAX_N_CSS_MEAS);
    
    /* Begin method steps */
    mTranspose(hObs, numObs, SKF_N_STATES, hObsT);
    
    mMultM(covarBar, SKF_N_STATES, SKF_N_STATES, hObsT, SKF_N_STATES, numObs, covHT);
    mMultM(hObs, numObs, SKF_N_STATES, covarBar, SKF_N_STATES, SKF_N_STATES, hCovar);
    mMultM(hCovar, numObs, SKF_N_STATES, hObsT, SKF_N_STATES, numObs, hCovarHT);
    
    mCopy(measNoise, numObs, numObs, rMat);
    
    /*! - Add measurement noise */
    mAdd(hCovarHT, numObs, numObs, rMat, hCovarHT);
    
    /*! - Invert the previous matrix */
    mInverse(hCovarHT, numObs, hCovarHTinv);
    
    /*! - Compute the Kalman Gain */
    mMultM(covHT, SKF_N_STATES, numObs, hCovarHTinv, numObs, numObs, kalmanGain);
    
}


