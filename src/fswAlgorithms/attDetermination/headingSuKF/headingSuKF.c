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

#include "attDetermination/headingSuKF/headingSuKF.h"
#include "attDetermination/_GeneralModuleFiles/ukfUtilities.h"
#include "simulation/utilities/linearAlgebra.h"
#include "simulation/utilities/rigidBodyKinematics.h"
#include "simFswInterfaceMessages/macroDefinitions.h"
#include <string.h>
#include <math.h>

/*! This method initializes the ConfigData for the heading estimator.
 It checks to ensure that the inputs are sane and then creates the
 output message
 @return void
 @param ConfigData The configuration data associated with the heading estimator
 */
void SelfInit_headingSuKF(HeadingSuKFConfig *ConfigData, uint64_t moduleID)
{
        /*! Begin method steps */
    /*! - Create output message for module */
	ConfigData->opnavDataOutMsgId = CreateNewMessage(ConfigData->opnavOutMsgName,
		sizeof(OpnavFswMsg), "OpnavFswMsg", moduleID);
    /*! - Create filter states output message which is mostly for debug*/
    ConfigData->filtDataOutMsgId = CreateNewMessage(ConfigData->filtDataOutMsgName,
        sizeof(HeadingFilterFswMsg), "HeadingFilterFswMsg", moduleID);
    
}

/*! This method performs the second stage of initialization for the heading filter.  It's primary function is to link the input messages that were
 created elsewhere.
 @return void
 @param ConfigData The configuration data associated with the heading filter
 */
void CrossInit_headingSuKF(HeadingSuKFConfig *ConfigData, uint64_t moduleID)
{
    /*! Begin method steps */
    /*! - Find the message ID for the coarse sun sensor data message */
    ConfigData->opnavDataInMsgId = subscribeToMessage(ConfigData->opnavDataInMsgName,
        sizeof(OpnavFswMsg), moduleID);
    
}

/*! This method resets the heading attitude filter to an initial state and
 initializes the internal estimation matrices.
 @return void
 @param ConfigData The configuration data associated with the heading estimator
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void Reset_headingSuKF(HeadingSuKFConfig *ConfigData, uint64_t callTime,
                      uint64_t moduleID)
{
    
    int32_t i;
    double tempMatrix[HEAD_N_STATES_SWITCH*HEAD_N_STATES_SWITCH];
    
    /*! Begin method steps*/
    /*! - Zero the local configuration data structures and outputs */
    memset(&(ConfigData->outputHeading), 0x0, sizeof(NavAttIntMsg));

    
    /*! - Initialize filter parameters to max values */
    ConfigData->timeTag = callTime*NANO2SEC;
    ConfigData->dt = 0.0;
    ConfigData->numStates = HEAD_N_STATES_SWITCH;
    ConfigData->countHalfSPs = HEAD_N_STATES_SWITCH;
    
    /*! Initalize the filter to use b_1 of the body frame to make frame*/
    v3Set(1, 0, 0, ConfigData->bVec_B);
    ConfigData->switchTresh = 0.866;
    
    /*! - Ensure that all internal filter matrices are zeroed*/
    vSetZero(ConfigData->obs, OPNAV_MEAS);
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
    
    vCopy(ConfigData->stateInit, ConfigData->numStates, ConfigData->state);
    
    /*! - User a cholesky decomposition to obtain the sBar and sQnoise matrices for use in 
          filter at runtime*/
    mCopy(ConfigData->covarInit, ConfigData->numStates, ConfigData->numStates,
          ConfigData->covar);
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

/*! This method takes the parsed heading sensor data and outputs an estimate of the
 sun vector in the ADCS body frame
 @return void
 @param ConfigData The configuration data associated with the heading estimator
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void Update_headingSuKF(HeadingSuKFConfig *ConfigData, uint64_t callTime,
    uint64_t moduleID)
{
    double newTimeTag;
    double yBar[OPNAV_MEAS];
    double tempYVec[OPNAV_MEAS];
    double heading_hat[3];
    double states_BN[HEAD_N_STATES_SWITCH];
    int i;
    uint64_t ClockTime;
    uint32_t ReadSize;
    HeadingFilterFswMsg headingDataOutBuffer;
    OpnavFswMsg opnavOutputBuffer;
    
    /*! Begin method steps*/
    /*! - Read the input parsed heading sensor data message*/
    ClockTime = 0;
    ReadSize = 0;
    memset(&(ConfigData->opnavInBuffer), 0x0, sizeof(OpnavFswMsg));
    v3SetZero(ConfigData->obs);
    v3SetZero(ConfigData->postFits);
    ReadMessage(ConfigData->opnavDataInMsgId, &ClockTime, &ReadSize,
        sizeof(OpnavFswMsg), (void*) (&(ConfigData->opnavInBuffer)), moduleID);
    
    v3Normalize(&ConfigData->state[0], heading_hat);
    
    
    /*! - Check for switching frames */
    if (v3Dot(ConfigData->bVec_B, heading_hat) > ConfigData->switchTresh)
    {
        headingSuKFSwitch(ConfigData->bVec_B, ConfigData->state, ConfigData->covar);
    }
    
    /*! - If the time tag from the measured data is new compared to previous step, 
          propagate and update the filter*/
    newTimeTag = ClockTime * NANO2SEC;
    if(newTimeTag >= ConfigData->timeTag && ReadSize > 0)
    {
        headingSuKFTimeUpdate(ConfigData, newTimeTag);
        headingSuKFMeasUpdate(ConfigData, newTimeTag);
    }
    
    /*! - If current clock time is further ahead than the measured time, then
          propagate to this current time-step*/
    newTimeTag = callTime*NANO2SEC;
    if(newTimeTag > ConfigData->timeTag)
    {
        headingSuKFTimeUpdate(ConfigData, newTimeTag);
    }
    
    /*! - Compute the value for the yBar parameter (equation 23)*/
    vSetZero(yBar, OPNAV_MEAS);
    for(i=0; i<ConfigData->countHalfSPs*2+1; i++)
    {
        vCopy(&(ConfigData->yMeas[i*OPNAV_MEAS]), OPNAV_MEAS,
              tempYVec);
        vScale(ConfigData->wM[i], tempYVec, OPNAV_MEAS, tempYVec);
        vAdd(yBar, OPNAV_MEAS, tempYVec, yBar);
    }
    
    /*! - The post fits are y - ybar if a measurement was read, if observations are zero,
     do not compute post fit residuals*/
    if(!v3IsZero(ConfigData->obs, 1E-10)){
        mSubtract(ConfigData->obs, OPNAV_MEAS, 1, yBar, ConfigData->postFits);}
    
    /* Switch the rates back to omega_BN instead of oemga_SB */
    vCopy(ConfigData->state, HEAD_N_STATES_SWITCH, states_BN);
    vScale(-1, &(states_BN[3]), 2, &(states_BN[3]));
    
    /*! - Populate the filter states output buffer and write the output message*/
    headingDataOutBuffer.timeTag = ConfigData->timeTag;
    mCopy(ConfigData->covar, HEAD_N_STATES_SWITCH, HEAD_N_STATES_SWITCH, headingDataOutBuffer.covar);
    vCopy(states_BN, HEAD_N_STATES_SWITCH, headingDataOutBuffer.state);
    v3Copy(ConfigData->postFits, headingDataOutBuffer.postFitRes);
    WriteMessage(ConfigData->filtDataOutMsgId, callTime, sizeof(HeadingFilterFswMsg),
                 &headingDataOutBuffer, moduleID);
    
    /*! - Write the heading estimate into the copy of the OpNav message structure*/
    opnavOutputBuffer.timeTag = ConfigData->timeTag;
    m33Copy(RECAST3X3 ConfigData->covar, RECAST3X3 opnavOutputBuffer.covar);
    v3Copy(&states_BN[0], opnavOutputBuffer.rel_pos);
    WriteMessage(ConfigData->opnavDataOutMsgId, callTime, sizeof(OpnavFswMsg),
                 &opnavOutputBuffer, moduleID);
    
    return;
}

/*! This method propagates a heading state vector forward in time.  Note
    that the calling parameter is updated in place to save on data copies.
	@return void
	@param stateInOut The state that is propagated
*/
void headingStateProp(double *stateInOut, double *b_Vec, double dt)
{

    double propagatedVel[HEAD_N_STATES];
    double omegaCrossd[HEAD_N_STATES];
    double omega_BN_S[HEAD_N_STATES] = {0, -stateInOut[3], -stateInOut[4]};
    double omega_BN_B[HEAD_N_STATES];
    double dcm_BS[HEAD_N_STATES][HEAD_N_STATES];

    mSetZero(dcm_BS, HEAD_N_STATES, HEAD_N_STATES);

    headingSuKFComputeDCM_BS(stateInOut, b_Vec, &dcm_BS[0][0]);
    mMultV(dcm_BS, HEAD_N_STATES, HEAD_N_STATES, omega_BN_S, omega_BN_B);
    /* Set local variables to zero*/
    vSetZero(propagatedVel, HEAD_N_STATES);
    
    /*! Begin state update steps */
    /*! Take omega cross d*/
    v3Cross(omega_BN_B, stateInOut, omegaCrossd);
    
    /*! - Multiply omega cross d by dt and add to state to propagate */
    v3Scale(-dt, omegaCrossd, propagatedVel);
    v3Add(stateInOut, propagatedVel, stateInOut);
    
	return;
}

/*! This method performs the time update for the heading kalman filter.
     It propagates the sigma points forward in time and then gets the current 
	 covariance and state estimates.
	 @return void
     @param ConfigData The configuration data associated with the heading estimator
     @param updateTime The time that we need to fix the filter to (seconds)
*/
void headingSuKFTimeUpdate(HeadingSuKFConfig *ConfigData, double updateTime)
{
	int i, Index;
	double sBarT[HEAD_N_STATES_SWITCH*HEAD_N_STATES_SWITCH];
	double xComp[HEAD_N_STATES_SWITCH], AT[(2 * HEAD_N_STATES_SWITCH + HEAD_N_STATES_SWITCH)*HEAD_N_STATES_SWITCH];
	double aRow[HEAD_N_STATES_SWITCH], rAT[HEAD_N_STATES_SWITCH*HEAD_N_STATES_SWITCH], xErr[HEAD_N_STATES_SWITCH];
	double sBarUp[HEAD_N_STATES_SWITCH*HEAD_N_STATES_SWITCH];
	double *spPtr;
	/*! Begin method steps*/
	ConfigData->dt = updateTime - ConfigData->timeTag;
    
    /*! - Copy over the current state estimate into the 0th Sigma point and propagate by dt*/
	vCopy(ConfigData->state, ConfigData->numStates,
		&(ConfigData->SP[0 * ConfigData->numStates + 0]));
	headingStateProp(&(ConfigData->SP[0 * ConfigData->numStates + 0]), ConfigData->bVec_B, ConfigData->dt);
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
		headingStateProp(spPtr, ConfigData->bVec_B, ConfigData->dt);
		vScale(ConfigData->wM[Index], spPtr, ConfigData->numStates, xComp);
		vAdd(xComp, ConfigData->numStates, ConfigData->xBar, ConfigData->xBar);
		
		Index = i + 1 + ConfigData->countHalfSPs;
        spPtr = &(ConfigData->SP[Index*ConfigData->numStates]);
        vCopy(&sBarT[i*ConfigData->numStates], ConfigData->numStates, spPtr);
        vScale(-ConfigData->gamma, spPtr, ConfigData->numStates, spPtr);
        vAdd(spPtr, ConfigData->numStates, ConfigData->state, spPtr);
        headingStateProp(spPtr, ConfigData->bVec_B, ConfigData->dt);
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

/*! This method computes what the expected measurement vector is for each opnave measurement.  All data is transacted from the main
    data structure for the model because there are many variables that would 
    have to be updated otherwise.
 @return void
 @param ConfigData The configuration data associated with the heading estimator

 */
void headingSuKFMeasModel(HeadingSuKFConfig *ConfigData)
{
    /* Begin method steps */
    /*! - Loop over sigma points */
    int j;
    int i;
    v3Copy(ConfigData->opnavInBuffer.rel_pos, ConfigData->obs);
    for(j=0; j<ConfigData->countHalfSPs*2+1; j++)
    {
        for(i=0; i<3; i++)
        ConfigData->yMeas[i*(ConfigData->countHalfSPs*2+1) + j] =
            ConfigData->SP[i + j*HEAD_N_STATES_SWITCH];
    }
    
    /*! - yMeas matrix was set backwards deliberately so we need to transpose it through*/
    mTranspose(ConfigData->yMeas, OPNAV_MEAS, ConfigData->countHalfSPs*2+1,
        ConfigData->yMeas);
    
}

/*! This method performs the measurement update for the heading kalman filter.
 It applies the observations in the obs vectors to the current state estimate and 
 updates the state/covariance with that information.
 @return void
 @param ConfigData The configuration data associated with the heading estimator
 @param updateTime The time that we need to fix the filter to (seconds)
 */
void headingSuKFMeasUpdate(HeadingSuKFConfig *ConfigData, double updateTime)
{
    uint32_t i;
    double yBar[OPNAV_MEAS], syInv[OPNAV_MEAS*OPNAV_MEAS];
    double kMat[HEAD_N_STATES_SWITCH*OPNAV_MEAS];
    double xHat[HEAD_N_STATES_SWITCH], sBarT[HEAD_N_STATES_SWITCH*HEAD_N_STATES_SWITCH], tempYVec[OPNAV_MEAS];
    double AT[(2 * HEAD_N_STATES_SWITCH + OPNAV_MEAS)*OPNAV_MEAS], qChol[OPNAV_MEAS*OPNAV_MEAS];
    double rAT[OPNAV_MEAS*OPNAV_MEAS], syT[OPNAV_MEAS*OPNAV_MEAS];
    double sy[OPNAV_MEAS*OPNAV_MEAS];
    double updMat[OPNAV_MEAS*OPNAV_MEAS], pXY[HEAD_N_STATES_SWITCH*OPNAV_MEAS];
    
    /*! Begin method steps*/
    
    /*! - Compute the valid observations and the measurement model for all observations*/
    headingSuKFMeasModel(ConfigData);
    
    /*! - Compute the value for the yBar parameter (note that this is equation 23 in the 
          time update section of the reference document*/
    vSetZero(yBar, OPNAV_MEAS);
    for(i=0; i<ConfigData->countHalfSPs*2+1; i++)
    {
        vCopy(&(ConfigData->yMeas[i*OPNAV_MEAS]), OPNAV_MEAS,
              tempYVec);
        vScale(ConfigData->wM[i], tempYVec, OPNAV_MEAS, tempYVec);
        vAdd(yBar, OPNAV_MEAS, tempYVec, yBar);
    }
    
    /*! - Populate the matrix that we perform the QR decomposition on in the measurement 
          update section of the code.  This is based on the differenence between the yBar 
          parameter and the calculated measurement models.  Equation 24 in driving doc. */
    mSetZero(AT, ConfigData->countHalfSPs*2+OPNAV_MEAS,
        OPNAV_MEAS);
    for(i=0; i<ConfigData->countHalfSPs*2; i++)
    {
        vScale(-1.0, yBar, OPNAV_MEAS, tempYVec);
        vAdd(tempYVec, OPNAV_MEAS,
             &(ConfigData->yMeas[(i+1)*OPNAV_MEAS]), tempYVec);
        vScale(sqrt(ConfigData->wC[i+1]), tempYVec, OPNAV_MEAS, tempYVec);
        memcpy(&(AT[i*OPNAV_MEAS]), tempYVec,
               OPNAV_MEAS*sizeof(double));
    }
    
    /*! - This is the square-root of the Rk matrix which we treat as the Cholesky
        decomposition of the observation variance matrix constructed for our number 
        of observations*/
    mSetIdentity(ConfigData->qObs, OPNAV_MEAS, OPNAV_MEAS);
    mScale(ConfigData->qObsVal, ConfigData->qObs, OPNAV_MEAS,
           OPNAV_MEAS, ConfigData->qObs);
    ukfCholDecomp(ConfigData->qObs, OPNAV_MEAS, OPNAV_MEAS, qChol);
    memcpy(&(AT[2*ConfigData->countHalfSPs*OPNAV_MEAS]),
           qChol, OPNAV_MEAS*OPNAV_MEAS*sizeof(double));
    /*! - Perform QR decomposition (only R again) of the above matrix to obtain the 
          current Sy matrix*/
    ukfQRDJustR(AT, 2*ConfigData->countHalfSPs+OPNAV_MEAS,
                OPNAV_MEAS, rAT);
    mCopy(rAT, OPNAV_MEAS, OPNAV_MEAS, syT);
    mTranspose(syT, OPNAV_MEAS, OPNAV_MEAS, sy);
    /*! - Shift the matrix over by the difference between the 0th SP-based measurement 
          model and the yBar matrix (cholesky down-date again)*/
    vScale(-1.0, yBar, OPNAV_MEAS, tempYVec);
    vAdd(tempYVec, OPNAV_MEAS, &(ConfigData->yMeas[0]), tempYVec);
    ukfCholDownDate(sy, tempYVec, ConfigData->wC[0],
                    OPNAV_MEAS, updMat);
    /*! - Shifted matrix represents the Sy matrix */
    mCopy(updMat, OPNAV_MEAS, OPNAV_MEAS, sy);
    mTranspose(sy, OPNAV_MEAS, OPNAV_MEAS, syT);

    /*! - Construct the Pxy matrix (equation 26) which multiplies the Sigma-point cloud 
          by the measurement model cloud (weighted) to get the total Pxy matrix*/
    mSetZero(pXY, ConfigData->numStates, OPNAV_MEAS);
    for(i=0; i<2*ConfigData->countHalfSPs+1; i++)
    {
        vScale(-1.0, yBar, OPNAV_MEAS, tempYVec);
        vAdd(tempYVec, OPNAV_MEAS,
             &(ConfigData->yMeas[i*OPNAV_MEAS]), tempYVec);
        vSubtract(&(ConfigData->SP[i*ConfigData->numStates]), ConfigData->numStates,
                  ConfigData->xBar, xHat);
        vScale(ConfigData->wC[i], xHat, ConfigData->numStates, xHat);
        mMultM(xHat, ConfigData->numStates, 1, tempYVec, 1, OPNAV_MEAS,
            kMat);
        mAdd(pXY, ConfigData->numStates, OPNAV_MEAS, kMat, pXY);
    }

    /*! - Then we need to invert the SyT*Sy matrix to get the Kalman gain factor.  Since
          The Sy matrix is lower triangular, we can do a back-sub inversion instead of 
          a full matrix inversion.  That is the ukfUInv and ukfLInv calls below.  Once that 
          multiplication is done (equation 27), we have the Kalman Gain.*/
    ukfUInv(syT, OPNAV_MEAS, OPNAV_MEAS, syInv);
    
    mMultM(pXY, ConfigData->numStates, OPNAV_MEAS, syInv,
           OPNAV_MEAS, OPNAV_MEAS, kMat);
    ukfLInv(sy, OPNAV_MEAS, OPNAV_MEAS, syInv);
    mMultM(kMat, ConfigData->numStates, OPNAV_MEAS, syInv,
           OPNAV_MEAS, OPNAV_MEAS, kMat);
    
    
    /*! - Difference the yBar and the observations to get the observed error and 
          multiply by the Kalman Gain to get the state update.  Add the state update 
          to the state to get the updated state value (equation 27).*/
    vSubtract(ConfigData->obs, OPNAV_MEAS, yBar, tempYVec);
    mMultM(kMat, ConfigData->numStates, OPNAV_MEAS, tempYVec,
        OPNAV_MEAS, 1, xHat);
    vAdd(ConfigData->state, ConfigData->numStates, xHat, ConfigData->state);
    /*! - Compute the updated matrix U from equation 28.  Note that I then transpose it 
         so that I can extract "columns" from adjacent memory*/
    mMultM(kMat, ConfigData->numStates, OPNAV_MEAS, sy,
           OPNAV_MEAS, OPNAV_MEAS, pXY);
    mTranspose(pXY, ConfigData->numStates, OPNAV_MEAS, pXY);
    /*! - For each column in the update matrix, perform a cholesky down-date on it to 
          get the total shifted S matrix (called sBar in internal parameters*/
    for(i=0; i<OPNAV_MEAS; i++)
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

void headingSuKFSwitch(double *bVec_B, double *states, double *covar)
{
    double dcm_BSold[HEAD_N_STATES][HEAD_N_STATES];
    double dcm_BSnew_T[HEAD_N_STATES][HEAD_N_STATES];
    double dcm_SnewSold[HEAD_N_STATES][HEAD_N_STATES];
    double switchMatP[HEAD_N_STATES_SWITCH][HEAD_N_STATES_SWITCH];
    double switchMat[HEAD_N_STATES_SWITCH][HEAD_N_STATES_SWITCH];
    
    double sun_heading_norm[HEAD_N_STATES];
    double b1[HEAD_N_STATES];
    double b2[HEAD_N_STATES];
    
    /*!  Set the body frame vectors*/
    v3Set(1, 0, 0, b1);
    v3Set(0, 1, 0, b2);
    v3Normalize(&(states[0]), sun_heading_norm);
    
    /*! Populate the dcm_BS with the "old" S-frame*/
    headingSuKFComputeDCM_BS(sun_heading_norm, bVec_B, &dcm_BSold[0][0]);
    
    if (v3IsEqual(bVec_B, b1, 1e-10))
    {
        headingSuKFComputeDCM_BS(sun_heading_norm, b2, &dcm_BSnew_T[0][0]);
        v3Copy(b2, bVec_B);
    }
    else
    {
        headingSuKFComputeDCM_BS(sun_heading_norm, b1, &dcm_BSnew_T[0][0]);
        v3Copy(b1, bVec_B);
    }
    
    mTranspose(dcm_BSnew_T, HEAD_N_STATES, HEAD_N_STATES, dcm_BSnew_T);
    mMultM(dcm_BSnew_T, 3, 3, dcm_BSold, 3, 3, dcm_SnewSold);
    
    mSetIdentity(switchMat, HEAD_N_STATES_SWITCH, HEAD_N_STATES_SWITCH);
    mSetSubMatrix(&dcm_SnewSold[1][1], 1, 2, &switchMat, HEAD_N_STATES_SWITCH, HEAD_N_STATES_SWITCH, 3, 3);
    mSetSubMatrix(&dcm_SnewSold[2][1], 1, 2, &switchMat, HEAD_N_STATES_SWITCH, HEAD_N_STATES_SWITCH, 4, 3);
    
    mMultV(switchMat, HEAD_N_STATES_SWITCH, HEAD_N_STATES_SWITCH, states, states);
    mMultM(switchMat, HEAD_N_STATES_SWITCH, HEAD_N_STATES_SWITCH, covar, HEAD_N_STATES_SWITCH, HEAD_N_STATES_SWITCH, switchMatP);
    mTranspose(switchMat, HEAD_N_STATES_SWITCH, HEAD_N_STATES_SWITCH, switchMat);
    mMultM(switchMatP, HEAD_N_STATES_SWITCH, HEAD_N_STATES_SWITCH, switchMat, HEAD_N_STATES_SWITCH, HEAD_N_STATES_SWITCH, covar);
    return;
}


void headingSuKFComputeDCM_BS(double sunheading[HEAD_N_STATES], double bVec[HEAD_N_STATES], double *dcm){
    double s1_B[HEAD_N_STATES];
    double s2_B[HEAD_N_STATES];
    double s3_B[HEAD_N_STATES];
    
    mSetZero(dcm, HEAD_N_STATES, HEAD_N_STATES);
    v3SetZero(s2_B);
    v3SetZero(s3_B);
    
    v3Normalize(sunheading, s1_B);
    v3Cross(sunheading, bVec, s2_B);
    if (v3Norm(s2_B) < 1E-5){
        mSetIdentity(dcm, HEAD_N_STATES, HEAD_N_STATES);
    }
    else{
    v3Normalize(s2_B, s2_B);
    /*! Populate the dcm_BS with the "new" S-frame*/
    mSetSubMatrix(s1_B, 1, HEAD_N_STATES, dcm, HEAD_N_STATES, HEAD_N_STATES, 0, 0);
    mSetSubMatrix(&(s2_B), 1, HEAD_N_STATES, dcm, HEAD_N_STATES, HEAD_N_STATES, 1, 0);
    v3Cross(sunheading, s2_B, s3_B);
    v3Normalize(s3_B, s3_B);
    mSetSubMatrix(&(s3_B), 1, HEAD_N_STATES, dcm, HEAD_N_STATES, HEAD_N_STATES, 2, 0);
    mTranspose(dcm, HEAD_N_STATES, HEAD_N_STATES, dcm);
    }
    
}
