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

#include "attDetermination/InertialUKF/inertialUKF.h"
#include "attDetermination/_GeneralModuleFiles/ukfUtilities.h"
#include "SimCode/utilities/linearAlgebra.h"
#include "SimCode/utilities/rigidBodyKinematics.h"
#include "SimFswInterfaceMessages/macroDefinitions.h"
#include <string.h>
#include <math.h>

/*! This method initializes the ConfigData for theCSS WLS estimator.
 It checks to ensure that the inputs are sane and then creates the
 output message
 @return void
 @param ConfigData The configuration data associated with the CSS WLS estimator
 */
void SelfInit_inertialUKF(InertialUKFConfig *ConfigData, uint64_t moduleID)
{
    
    /*! Begin method steps */
    /*! - Create output message for module */
	ConfigData->navStateOutMsgId = CreateNewMessage(ConfigData->navStateOutMsgName,
		sizeof(NavAttMessage), "NavAttMessage", moduleID);
    /*! - Create filter states output message which is mostly for debug*/
    ConfigData->filtDataOutMsgId = CreateNewMessage(ConfigData->filtDataOutMsgName,
        sizeof(InertialFilterMessage), "InertialFilterMessage", moduleID);
    
}

/*! This method performs the second stage of initialization for the CSS sensor
 interface.  It's primary function is to link the input messages that were
 created elsewhere.
 @return void
 @param ConfigData The configuration data associated with the CSS interface
 */
void CrossInit_inertialUKF(InertialUKFConfig *ConfigData, uint64_t moduleID)
{
    /*! Begin method steps */
    /*! - Find the message ID for the coarse sun sensor data message */
    ConfigData->stDataInMsgId = subscribeToMessage(ConfigData->stDataInMsgName, sizeof(STAttMessage), moduleID);
    /*! - Find the message ID for the vehicle mass properties configuration message */
    ConfigData->massPropsInMsgId = subscribeToMessage(ConfigData->massPropsInMsgName,
        sizeof(VehicleConfigMessage), moduleID);
    ConfigData->rwParamsInMsgID = subscribeToMessage(ConfigData->rwParamsInMsgName,
                                                     sizeof(RWArrayConfigFswMsg), moduleID);
    ConfigData->rwSpeedsInMsgID = subscribeToMessage(ConfigData->rwSpeedsInMsgName,
        sizeof(RWSpeedMessage), moduleID);
    
    
}

/*! This method resets the inertial inertial filter to an initial state and
 initializes the internal estimation matrices.
 @return void
 @param ConfigData The configuration data associated with the CSS estimator
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void Reset_inertialUKF(InertialUKFConfig *ConfigData, uint64_t callTime,
                      uint64_t moduleID)
{
    
    int32_t i;
    VehicleConfigMessage massPropsInBuffer;
    uint64_t writeTime;
    uint32_t writeSize;
    double tempMatrix[AKF_N_STATES*AKF_N_STATES];
    
    /*! Begin method steps*/
    /*! - Zero the local configuration data structures and outputs */
    memset(&massPropsInBuffer, 0x0 ,sizeof(VehicleConfigMessage));
    memset(&(ConfigData->outputInertial), 0x0, sizeof(NavAttMessage));
    
    /*! - Read in mass properties and coarse sun sensor configuration information.*/
    ReadMessage(ConfigData->massPropsInMsgId, &writeTime, &writeSize,
                sizeof(VehicleConfigMessage), &massPropsInBuffer, moduleID);
    /*! - Read static RW config data message and store it in module variables */
    ReadMessage(ConfigData->rwParamsInMsgID, &writeTime, &writeSize,
                sizeof(RWArrayConfigFswMsg), &(ConfigData->rwConfigParams), moduleID);
    ReadMessage(ConfigData->massPropsInMsgId, &writeTime, &writeSize,
        sizeof(VehicleConfigMessage), &(ConfigData->localConfigData), moduleID);
    
    /*! - Initialize filter parameters to max values */
    ConfigData->timeTag = callTime*NANO2SEC;
    ConfigData->dt = 0.0;
    ConfigData->numStates = AKF_N_STATES;
    ConfigData->countHalfSPs = AKF_N_STATES;
    ConfigData->numObs = 3;
    ConfigData->firstPassComplete = 0;
    
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
void Update_inertialUKF(InertialUKFConfig *ConfigData, uint64_t callTime,
    uint64_t moduleID)
{
    double newTimeTag;
    uint64_t ClockTime;
    uint32_t ReadSize;
    uint32_t otherSize;
    InertialFilterMessage inertialDataOutBuffer;
    
    /*! Begin method steps*/
    /*! - Read the input parsed CSS sensor data message*/
    ClockTime = 0;
    ReadSize = 0;
    memset(&(ConfigData->stSensorIn), 0x0, sizeof(STAttMessage));
    ReadMessage(ConfigData->stDataInMsgId, &ClockTime, &ReadSize,
        sizeof(STAttMessage), (void*) (&(ConfigData->stSensorIn)), moduleID);
    /*! - If the time tag from the measured data is new compared to previous step, 
          propagate and update the filter*/
    newTimeTag = ClockTime * NANO2SEC;
    
    if (v3Norm(ConfigData->state) > ConfigData->switchMag) //Little extra margin
    {
        MRPswitch(ConfigData->state, ConfigData->switchMag, ConfigData->state);
    }
    
    ReadMessage(ConfigData->rwSpeedsInMsgID, &ClockTime, &otherSize,
        sizeof(RWSpeedMessage), &(ConfigData->rwSpeeds), moduleID);
    if(ConfigData->firstPassComplete == 0)
    {
        memcpy(&(ConfigData->rwSpeedPrev), &(ConfigData->rwSpeeds), sizeof(RWSpeedMessage));
        ConfigData->firstPassComplete = 1;
    }
    
    ReadMessage(ConfigData->massPropsInMsgId, &ClockTime, &otherSize,
                sizeof(VehicleConfigMessage), &(ConfigData->localConfigData), moduleID);
    m33Inverse(RECAST3X3 ConfigData->localConfigData.ISCPntB_B, ConfigData->IInv);
    
    if(newTimeTag >= ConfigData->timeTag && ReadSize > 0)
    {
        inertialUKFTimeUpdate(ConfigData, newTimeTag);
        inertialUKFMeasUpdate(ConfigData, newTimeTag);
    }
    
    /*! - If current clock time is further ahead than the measured time, then
          propagate to this current time-step*/
    newTimeTag = callTime*NANO2SEC;
    if(newTimeTag > ConfigData->timeTag)
    {
        inertialUKFTimeUpdate(ConfigData, newTimeTag);
    }
    
    /*! - Write the inertial estimate into the copy of the navigation message structure*/
	v3Copy(ConfigData->state, ConfigData->outputInertial.sigma_BN);
    v3Copy(&(ConfigData->state[3]), ConfigData->outputInertial.omega_BN_B);
    ConfigData->outputInertial.timeTag = ConfigData->timeTag;
	WriteMessage(ConfigData->navStateOutMsgId, callTime, sizeof(NavAttMessage),
		&(ConfigData->outputInertial), moduleID);
    
    /*! - Populate the filter states output buffer and write the output message*/
    inertialDataOutBuffer.timeTag = ConfigData->timeTag;
    inertialDataOutBuffer.numObs = ConfigData->numObs;
    memmove(inertialDataOutBuffer.covar, ConfigData->covar,
            AKF_N_STATES*AKF_N_STATES*sizeof(double));
    memmove(inertialDataOutBuffer.state, ConfigData->state, AKF_N_STATES*sizeof(double));
    WriteMessage(ConfigData->filtDataOutMsgId, callTime, sizeof(InertialFilterMessage),
                 &inertialDataOutBuffer, moduleID);
    
    memcpy(&(ConfigData->rwSpeedPrev), &(ConfigData->rwSpeeds), sizeof(RWSpeedMessage));
    
    return;
}

/*! This method propagates a inertial state vector forward in time.  Note 
    that the calling parameter is updated in place to save on data copies.
	@return void
	@param stateInOut The state that is propagated
*/
void inertialStateProp(InertialUKFConfig *ConfigData, double *stateInOut, double dt)
{

    double qDot[3];
    double BMatrix[3][3];
    double torqueTotal[3];
    double wheelAccel;
    double torqueSingle[3];
    double angAccelTotal[3];
    int i;
    
    BmatMRP(stateInOut, BMatrix);
    m33Scale(0.25, BMatrix, BMatrix);
    m33MultV3(BMatrix, &(stateInOut[3]), qDot);
    v3Scale(dt, qDot, qDot);
    v3Add(stateInOut, qDot, stateInOut);
    
    v3SetZero(torqueTotal);
    for(i=0; i<ConfigData->rwConfigParams.numRW; i++)
    {
        wheelAccel = ConfigData->rwSpeeds.wheelSpeeds[i]-
            ConfigData->rwSpeedPrev.wheelSpeeds[i];
        wheelAccel /= dt/ConfigData->rwConfigParams.JsList[i];
        v3Scale(wheelAccel, &(ConfigData->rwConfigParams.GsMatrix_B[i*3]), torqueSingle);
        v3Subtract(torqueTotal, torqueSingle, torqueTotal);
    }
    m33MultV3(ConfigData->IInv, torqueTotal, angAccelTotal);
    v3Scale(dt, angAccelTotal, angAccelTotal);
    v3Add(&(stateInOut[3]), angAccelTotal, &(stateInOut[3]));
	return;
}

/*! This method performs the time update for the inertial kalman filter.
     It propagates the sigma points forward in time and then gets the current 
	 covariance and state estimates.
	 @return void
     @param ConfigData The configuration data associated with the CSS estimator
     @param updateTime The time that we need to fix the filter to (seconds)
*/
void inertialUKFTimeUpdate(InertialUKFConfig *ConfigData, double updateTime)
{
	int i, Index;
	double sBarT[AKF_N_STATES*AKF_N_STATES];
	double xComp[AKF_N_STATES], AT[(2 * AKF_N_STATES + AKF_N_STATES)*AKF_N_STATES];
	double aRow[AKF_N_STATES], rAT[AKF_N_STATES*AKF_N_STATES], xErr[AKF_N_STATES]; 
	double sBarUp[AKF_N_STATES*AKF_N_STATES];
	double *spPtr;
	/*! Begin method steps*/
	ConfigData->dt = updateTime - ConfigData->timeTag;
    
    /*! - Copy over the current state estimate into the 0th Sigma point and propagate by dt*/
	vCopy(ConfigData->state, ConfigData->numStates,
		&(ConfigData->SP[0 * ConfigData->numStates + 0]));
	inertialStateProp(ConfigData, &(ConfigData->SP[0 * ConfigData->numStates + 0]),
        ConfigData->dt);
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
		inertialStateProp(ConfigData, spPtr, ConfigData->dt);
		vScale(ConfigData->wM[Index], spPtr, ConfigData->numStates, xComp);
		vAdd(xComp, ConfigData->numStates, ConfigData->xBar, ConfigData->xBar);
		
		Index = i + 1 + ConfigData->countHalfSPs;
        spPtr = &(ConfigData->SP[Index*ConfigData->numStates]);
        vCopy(&sBarT[i*ConfigData->numStates], ConfigData->numStates, spPtr);
        vScale(-ConfigData->gamma, spPtr, ConfigData->numStates, spPtr);
        vAdd(spPtr, ConfigData->numStates, ConfigData->state, spPtr);
        inertialStateProp(ConfigData, spPtr, ConfigData->dt);
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
void inertialUKFMeasModel(InertialUKFConfig *ConfigData)
{
    double quatTranspose[4];
    double quatMeas[4];
    double EPSum[4];
    double mrpSum[3];
    int i;
    
    MRP2EP(ConfigData->state, quatTranspose);
    v3Scale(-1.0, &(quatTranspose[1]), &(quatTranspose[1]));
    MRP2EP(ConfigData->stSensorIn.MRP_BdyInrtl, quatMeas);
    addEP(quatTranspose, quatMeas, EPSum);
    EP2MRP(EPSum, mrpSum);
    if (v3Norm(mrpSum) > 1.0)
    {
        MRPshadow(ConfigData->stSensorIn.MRP_BdyInrtl,
                  ConfigData->stSensorIn.MRP_BdyInrtl);
    }
    
    for(i=0; i<ConfigData->countHalfSPs*2+1; i++)
    {
        v3Copy(&(ConfigData->SP[i*AKF_N_STATES]), &(ConfigData->yMeas[i*3]));
    }
    
    v3Copy(ConfigData->stSensorIn.MRP_BdyInrtl, ConfigData->obs);
    ConfigData->numObs = 3;
    
}

/*! This method performs the measurement update for the inertial kalman filter.
 It applies the observations in the obs vectors to the current state estimate and 
 updates the state/covariance with that information.
 @return void
 @param ConfigData The configuration data associated with the CSS estimator
 @param updateTime The time that we need to fix the filter to (seconds)
 */
void inertialUKFMeasUpdate(InertialUKFConfig *ConfigData, double updateTime)
{
    uint32_t i;
    double yBar[3], syInv[3*3];
    double kMat[AKF_N_STATES*3];
    double xHat[AKF_N_STATES], sBarT[AKF_N_STATES*AKF_N_STATES], tempYVec[3];
    double AT[(2 * AKF_N_STATES + 3)*3], qChol[3*3];
    double rAT[3*3], syT[3*3];
    double sy[3*3];
    double updMat[3*3], pXY[AKF_N_STATES*3];
    
    /*! Begin method steps*/
    
    /*! - Compute the valid observations and the measurement model for all observations*/
    inertialUKFMeasModel(ConfigData);
    
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
        vCopy(&(pXY[i*ConfigData->numStates]), ConfigData->numStates, xHat);
        ukfCholDownDate(ConfigData->sBar, xHat, -1.0, ConfigData->numStates, sBarT);
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
