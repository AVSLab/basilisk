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

#include "attDetermination/InertialUKF/inertialUKF.h"
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
void SelfInit_inertialUKF(InertialUKFConfig *ConfigData, uint64_t moduleID)
{
    
    /*! Begin method steps */
    /*! - Create output message for module */
	ConfigData->navStateOutMsgId = CreateNewMessage(ConfigData->navStateOutMsgName,
		sizeof(NavAttIntMsg), "NavAttIntMsg", moduleID);
    /*! - Create filter states output message which is mostly for debug*/
    ConfigData->filtDataOutMsgId = CreateNewMessage(ConfigData->filtDataOutMsgName,
        sizeof(InertialFilterFswMsg), "InertialFilterFswMsg", moduleID);
    
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
    int i;
    for (i = 0; i < ConfigData->STDatasStruct.numST; i++)
    {
        ConfigData->STDatasStruct.STMessages[i].stInMsgID = subscribeToMessage(ConfigData->STDatasStruct.STMessages[i].stInMsgName, sizeof(STAttFswMsg), moduleID);
    }
	ConfigData->massPropsInMsgId = subscribeToMessage(ConfigData->massPropsInMsgName, 
		sizeof(VehicleConfigFswMsg), moduleID);
    /*! - Find the message ID for the vehicle mass properties configuration message */
    ConfigData->rwParamsInMsgID = subscribeToMessage(ConfigData->rwParamsInMsgName,
                                                     sizeof(RWArrayConfigFswMsg), moduleID);
    ConfigData->rwSpeedsInMsgID = subscribeToMessage(ConfigData->rwSpeedsInMsgName,
        sizeof(RWSpeedIntMsg), moduleID);
    
    ConfigData->gyrBuffInMsgID = subscribeToMessage(ConfigData->gyrBuffInMsgName,
                                                   sizeof(AccDataFswMsg), moduleID);
    
    
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
    uint64_t writeTime;
    uint32_t writeSize;
    double tempMatrix[AKF_N_STATES*AKF_N_STATES];
    
    /*! Begin method steps*/
    /*! - Zero the local configuration data structures and outputs */
    memset(&(ConfigData->outputInertial), 0x0, sizeof(NavAttIntMsg));
    
    /*! - Read static RW config data message and store it in module variables */
    ReadMessage(ConfigData->rwParamsInMsgID, &writeTime, &writeSize,
                sizeof(RWArrayConfigFswMsg), &(ConfigData->rwConfigParams), moduleID);
    ReadMessage(ConfigData->massPropsInMsgId, &writeTime, &writeSize,
        sizeof(VehicleConfigFswMsg), &(ConfigData->localConfigData), moduleID);
    
    /*! - Initialize filter parameters to max values */
    ConfigData->timeTag = callTime*NANO2SEC;
    ConfigData->dt = 0.0;
    ConfigData->numStates = AKF_N_STATES;
    ConfigData->countHalfSPs = AKF_N_STATES;
    ConfigData->numObs = 3;
    ConfigData->firstPassComplete = 0;
    ConfigData->speedDt = 0.0;
    ConfigData->timeWheelPrev = 0;
    ConfigData->badUpdate = 0;
    
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
    ConfigData->badUpdate += ukfCholDecomp(ConfigData->sBar, ConfigData->numStates,
                  ConfigData->numStates, tempMatrix);
    if (ConfigData->badUpdate<0){
        printf("bad update occured in UKF");
        return;}
    ConfigData->badUpdate += ukfCholDecomp(ConfigData->qNoise, ConfigData->numStates,
                  ConfigData->numStates, ConfigData->sQnoise);
    if (ConfigData->badUpdate<0){
        printf("bad update occured in UKF");
        return;}

    mCopy(tempMatrix, ConfigData->numStates, ConfigData->numStates,
          ConfigData->sBar);
    mTranspose(ConfigData->sQnoise, ConfigData->numStates,
               ConfigData->numStates, ConfigData->sQnoise);
    
    v3Copy(ConfigData->state, ConfigData->sigma_BNOut);
    v3Copy(&(ConfigData->state[3]), ConfigData->omega_BN_BOut);
    ConfigData->timeTagOut = ConfigData->timeTag;
    

    return;
}

/*! This method reads in the messages from all availabel star trackers and orders them
 @return void
 @param ConfigData The configuration data associated with the CSS estimator
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void Read_STMessages(InertialUKFConfig *ConfigData, uint64_t moduleID)
{
    uint64_t ClockTime; /* [ns] Read time for the message*/
    uint32_t ReadSize;  /* [-] Non-zero size indicates we received ST msg*/
    int bufferSTIndice; /* Local ST message to copy and organize  */
    int i;
    int j;
    
    for (i = 0; i < ConfigData->STDatasStruct.numST; i++)
    {
        /*! Begin method steps*/
        /*! - Read the input parsed CSS sensor data message*/
        ClockTime = 0;
        ReadSize = 0;
        memset(&(ConfigData->stSensorIn[i]), 0x0, sizeof(STAttFswMsg));
        ReadMessage(ConfigData->STDatasStruct.STMessages[i].stInMsgID, &ClockTime, &ReadSize,
                    sizeof(STAttFswMsg), (void*) (&(ConfigData->stSensorIn[i])), moduleID);
        
        ConfigData->ClockTimeST[i] = ClockTime;
        ConfigData->ReadSizeST[i] = ReadSize;
        
        /*! - If the time tag from the measured data is new compared to previous step,
         propagate and update the filter*/
        for (j=0; j<i; j++)
            {
                ConfigData->stSensorOrder[j+1] = i;
                if (ConfigData->stSensorIn[i].timeTag < ConfigData->stSensorIn[j].timeTag )
                {
                    bufferSTIndice = j+1;
                    ConfigData->stSensorOrder[j+1] =  ConfigData->stSensorOrder[i];
                    ConfigData->stSensorOrder[i] = bufferSTIndice;
                }
            }
    }
    

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
    double newTimeTag;  /* [s] Local Time-tag variable*/
    uint64_t ClockTime; /* [ns] Read time for the message*/
    uint32_t ReadSize;  /* [-] Non-zero size indicates we received ST msg*/
    uint32_t otherSize; /* [-] Size of messages that are assumed to be good*/
    int32_t trackerValid; /* [-] Indicates whether the star tracker was valid*/
    double sigma_BNSum[3]; /* [-] Local MRP for propagated state*/
    InertialFilterFswMsg inertialDataOutBuffer; /* [-] Output filter info*/
    AccDataFswMsg gyrBuffer; /* [-] Buffer of IMU messages for gyro prop*/
    int i;
    
    // Reset update check to zero
    ConfigData->badUpdate = 0;
    
    if (v3Norm(ConfigData->state) > ConfigData->switchMag) //Little extra margin
    {
        MRPswitch(ConfigData->state, ConfigData->switchMag, ConfigData->state);
    }
    
    memset(&gyrBuffer, 0x0, sizeof(AccDataFswMsg));
    ReadMessage(ConfigData->gyrBuffInMsgID, &ClockTime, &otherSize,
                sizeof(AccDataFswMsg), &gyrBuffer, moduleID);
    ReadMessage(ConfigData->rwSpeedsInMsgID, &ClockTime, &otherSize,
                sizeof(RWSpeedIntMsg), &(ConfigData->rwSpeeds), moduleID);
    Read_STMessages(ConfigData, moduleID);
    
    if(ConfigData->firstPassComplete == 0)
    {
        memcpy(&(ConfigData->rwSpeedPrev), &(ConfigData->rwSpeeds), sizeof(RWSpeedIntMsg));
        ConfigData->timeWheelPrev = ClockTime;
        ConfigData->timeTag = ConfigData->stSensorIn[0].timeTag > ClockTime ?
        ConfigData->stSensorIn[0].timeTag*NANO2SEC : ClockTime*NANO2SEC;
        
        ConfigData->firstPassComplete = 1;
    }
    
    ConfigData->speedDt = (ClockTime - ConfigData->timeWheelPrev)*NANO2SEC;
    ConfigData->timeWheelPrev = ClockTime;
    
    ReadMessage(ConfigData->massPropsInMsgId, &ClockTime, &otherSize,
                sizeof(VehicleConfigFswMsg), &(ConfigData->localConfigData), moduleID);
    m33Inverse(RECAST3X3 ConfigData->localConfigData.ISCPntB_B, ConfigData->IInv);
    
    for (i = 0; i < ConfigData->STDatasStruct.numST; i++)
    {
        newTimeTag = ConfigData->stSensorIn[ConfigData->stSensorOrder[i]].timeTag * NANO2SEC;
        ClockTime = ConfigData->ClockTimeST[ConfigData->stSensorOrder[i]];
        ReadSize =  ConfigData->ReadSizeST[ConfigData->stSensorOrder[i]];
        
        /*! - If the star tracker has provided a new message compared to last time,
              update the filter to the new measurement*/
        trackerValid = 0;
        if(newTimeTag >= ConfigData->timeTag && ReadSize > 0)
        {
            inertialUKFTimeUpdate(ConfigData, newTimeTag);
            inertialUKFMeasUpdate(ConfigData, newTimeTag, ConfigData->stSensorOrder[i]);
            trackerValid = 1;
        }
        
    }
    /*! - If current clock time is further ahead than the measured time, then
     propagate to this current time-step*/
    newTimeTag = callTime*NANO2SEC;
    if(newTimeTag > ConfigData->timeTag)
    {
        /*! - If we have gotten  a masurement this frame, propagate to caller time.
         This will extrapolate the state blindy, which is not ideal so the
         latency between the ST meas and the current time is hopefully small.*/
        if(trackerValid)
        {
            inertialUKFTimeUpdate(ConfigData, newTimeTag);
            v3Copy(ConfigData->state, ConfigData->sigma_BNOut);
            v3Copy(&(ConfigData->state[3]), ConfigData->omega_BN_BOut);
            ConfigData->timeTagOut = ConfigData->timeTag;
        }
        /*! - If no star tracker measurement was available, propagate the state
         on the gyro measurements received since the last ST update.  Note
         that the rate estimate is just smoothed gyro data in this case*/
        else
        {
            /*! - Assemble the aggregrate rotation from the gyro buffer*/
            inertialUKFAggGyrData(ConfigData, ConfigData->timeTagOut,
                                  newTimeTag, &gyrBuffer);
            /*! - Propagate the attitude quaternion with the aggregate rotation*/
            addMRP(ConfigData->sigma_BNOut, ConfigData->aggSigma_b2b1,
                   sigma_BNSum);
            /*! - Switch the MRPs if necessary*/
            if (v3Norm(sigma_BNSum) > ConfigData->switchMag) //Little extra margin
            {
                MRPswitch(sigma_BNSum, ConfigData->switchMag, sigma_BNSum);
            }
            v3Copy(sigma_BNSum, ConfigData->sigma_BNOut);
            /*! - Rate estimate in this case is simply the low-pass filtered
             gyro data.  This is likely much noisier than the time-update
             solution*/
            for(i=0; i<3; i++)
            {
                ConfigData->omega_BN_BOut[i] =
                ConfigData->gyroFilt[i].currentState;
            }
            ConfigData->timeTagOut = newTimeTag;
            
        }
    }
    else
    {
        /*! - If we are already at callTime just copy the states over without
         change*/
        v3Copy(ConfigData->state, ConfigData->sigma_BNOut);
        v3Copy(&(ConfigData->state[3]), ConfigData->omega_BN_BOut);
        ConfigData->timeTagOut = ConfigData->timeTag;
    }
    
    /*! - Write the inertial estimate into the copy of the navigation message structure*/
    v3Copy(ConfigData->sigma_BNOut, ConfigData->outputInertial.sigma_BN);
    v3Copy(ConfigData->omega_BN_BOut, ConfigData->outputInertial.omega_BN_B);
    ConfigData->outputInertial.timeTag = ConfigData->timeTagOut;
	
	WriteMessage(ConfigData->navStateOutMsgId, callTime, sizeof(NavAttIntMsg),
		&(ConfigData->outputInertial), moduleID);
    
    /*! - Populate the filter states output buffer and write the output message*/
    inertialDataOutBuffer.timeTag = ConfigData->timeTag;
    inertialDataOutBuffer.numObs = ConfigData->numObs;
    memmove(inertialDataOutBuffer.covar, ConfigData->covar,
            AKF_N_STATES*AKF_N_STATES*sizeof(double));
    memmove(inertialDataOutBuffer.state, ConfigData->state, AKF_N_STATES*sizeof(double));
    WriteMessage(ConfigData->filtDataOutMsgId, callTime, sizeof(InertialFilterFswMsg),
                 &inertialDataOutBuffer, moduleID);
    
    memcpy(&(ConfigData->rwSpeedPrev), &(ConfigData->rwSpeeds), sizeof(RWSpeedIntMsg));
    
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
    
    /*! Begin method steps*/
    /*! - Convert the state derivative (body rate) to sigmaDot and propagate 
          the attitude MRPs*/
    BmatMRP(stateInOut, BMatrix);
    m33Scale(0.25, BMatrix, BMatrix);
    m33MultV3(BMatrix, &(stateInOut[3]), qDot);
    v3Scale(dt, qDot, qDot);
    v3Add(stateInOut, qDot, stateInOut);
    
    /*! - Assemble the total torque from the reaction wheels to get the forcing 
          function from any wheels present*/
    v3SetZero(torqueTotal);
    for(i=0; i<ConfigData->rwConfigParams.numRW; i++)
    {
        if(ConfigData->speedDt == 0.0)
        {
            continue;
        }
        wheelAccel = ConfigData->rwSpeeds.wheelSpeeds[i]-
            ConfigData->rwSpeedPrev.wheelSpeeds[i];
        wheelAccel /= ConfigData->speedDt/ConfigData->rwConfigParams.JsList[i];
        v3Scale(wheelAccel, &(ConfigData->rwConfigParams.GsMatrix_B[i*3]), torqueSingle);
        v3Subtract(torqueTotal, torqueSingle, torqueTotal);
    }
    /*! - Get the angular acceleration and propagate the state forward (euler prop)*/
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
	int i, Index, k;
	double sBarT[AKF_N_STATES*AKF_N_STATES];
	double xComp[AKF_N_STATES], AT[(2 * AKF_N_STATES + AKF_N_STATES)*AKF_N_STATES];
	double aRow[AKF_N_STATES], rAT[AKF_N_STATES*AKF_N_STATES], xErr[AKF_N_STATES]; 
	double sBarUp[AKF_N_STATES*AKF_N_STATES];
	double *spPtr;
    double procNoise[AKF_N_STATES*AKF_N_STATES];
    
	/*! Begin method steps*/
	ConfigData->dt = updateTime - ConfigData->timeTag;
    
    mCopy(ConfigData->sQnoise, AKF_N_STATES, AKF_N_STATES, procNoise);
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
        if (ConfigData->wC[i+1]<0){
            ConfigData->badUpdate += 1;
            printf("bad update occured in UKF");
            return;
        }
        else{
        vScale(sqrt(ConfigData->wC[i+1]), aRow, ConfigData->numStates, aRow);
		memcpy((void *)&AT[i*ConfigData->numStates], (void *)aRow,
			ConfigData->numStates*sizeof(double));
        }
	}
//    ! - Scale sQNoise matrix depending on the dt
    for (k=0;k<3;k++){
        procNoise[k*AKF_N_STATES+k] *= ConfigData->dt*ConfigData->dt/2;
        procNoise[(k+3)*AKF_N_STATES+(k+3)] *= ConfigData->dt;
    }
    /*! - Pop the sQNoise matrix on to the end of AT prior to getting QR decomposition*/
	memcpy(&AT[2 * ConfigData->countHalfSPs*ConfigData->numStates],
		procNoise, ConfigData->numStates*ConfigData->numStates
        *sizeof(double));
    /*! - QR decomposition (only R computed!) of the AT matrix provides the new sBar matrix*/
    ConfigData->badUpdate += ukfQRDJustR(AT, 2 * ConfigData->countHalfSPs + ConfigData->numStates,
                ConfigData->countHalfSPs, rAT);
    if (ConfigData->badUpdate<0){
        printf("bad update occured in UKF");
        return;}
    mCopy(rAT, ConfigData->numStates, ConfigData->numStates, sBarT);
    mTranspose(sBarT, ConfigData->numStates, ConfigData->numStates,
        ConfigData->sBar);
    
    /*! - Shift the sBar matrix over by the xBar vector using the appropriate weight 
          like in equation 21 in design document.*/
    vScale(-1.0, ConfigData->xBar, ConfigData->numStates, xErr);
    vAdd(xErr, ConfigData->numStates, &ConfigData->SP[0], xErr);
    ConfigData->badUpdate += ukfCholDownDate(ConfigData->sBar, xErr, ConfigData->wC[0],
        ConfigData->numStates, sBarUp);
    if (ConfigData->badUpdate<0){
        printf("bad update occured in UKF");
        return;}

    
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
void inertialUKFMeasModel(InertialUKFConfig *ConfigData, int currentST)
{
    double quatTranspose[4];
    double quatMeas[4];
    double EPSum[4];
    double mrpSum[3];
    int i;
    
    /*! This math seems more difficult than it should be, but there is a method.
        The input MRP may or may not be in the same "shadow" set as the state est.
        So, if they are different in terms of light/shadow, you have to get them 
        to the same representation otherwise your residuals will show 360 degree 
        errors.  Which is not ideal.  So that's why it is so blessed complicated.  
        The measurement is shadowed into the same representation as the state*/
    /*! Begin method steps*/
    MRP2EP(ConfigData->state, quatTranspose);
    v3Scale(-1.0, &(quatTranspose[1]), &(quatTranspose[1]));
    MRP2EP(ConfigData->stSensorIn[currentST].MRP_BdyInrtl, quatMeas);
    addEP(quatTranspose, quatMeas, EPSum);
    EP2MRP(EPSum, mrpSum);
    if (v3Norm(mrpSum) > 1.0)
    {
        MRPshadow(ConfigData->stSensorIn[currentST].MRP_BdyInrtl,
                  ConfigData->stSensorIn[currentST].MRP_BdyInrtl);
    }
    
    /*! - The measurement model is the same as the states since the star tracker 
          measures the inertial attitude directly.*/
    for(i=0; i<ConfigData->countHalfSPs*2+1; i++)
    {
        v3Copy(&(ConfigData->SP[i*AKF_N_STATES]), &(ConfigData->yMeas[i*3]));
    }
    
    v3Copy(ConfigData->stSensorIn[currentST].MRP_BdyInrtl, ConfigData->obs);
    ConfigData->numObs = 3;
    
}

/*! This method aggregates the input gyro data into a combined total quaternion 
    rotation to push the state forward by.  This information is stored in the 
    main data structure for use in the propagation routines.
 @return void
 @param ConfigData The configuration data associated with the CSS estimator
 @param propTime The time that we need to fix the filter to (seconds)
 @param gyrData The gyro measurements that we are going to accumulate forward into time
 */
void inertialUKFAggGyrData(InertialUKFConfig *ConfigData, double prevTime,
    double propTime, AccDataFswMsg *gyrData)
{
    uint32_t minFutInd;  /* [-] Index in buffer that is the oldest new meas*/
    int i, j;
    double minFutTime;   /* [s] smallest future measurement time-tag*/
    double measTime;     /* [s] measurement time*/
    /*! Note that the math here is tortured to avoid the issues of adding
          PRVs together.  That is numerically problematic, so we convert to 
          euler parameters (quaternions) and add those*/
    double ep_BpropB0[4], ep_B1B0[4], epTemp[4], omeg_BN_B[3], prvTemp[3];
    double dt;
    
    /*! Begin method steps*/
    minFutInd = 0;
    minFutTime = propTime;
    /*! - Loop through the entire gyro buffer to find the first index that is 
          in the future compared to prevTime*/
    for(i=0; i<MAX_ACC_BUF_PKT; i++)
    {
        measTime = gyrData->accPkts[i].measTime*NANO2SEC;
        if(measTime > prevTime && measTime < minFutTime)
        {
            minFutInd = i;
            minFutTime = measTime;
        }
    }
    /*! - Initialize the propagated euler parameters and time*/
    v4SetZero(ep_BpropB0);
    ep_BpropB0[0] = 1.0;
    i=0;
    measTime = prevTime;
    /*! - Loop through buffer for all valid measurements to assemble the 
          composite rotation since the previous time*/
    while(minFutTime <= propTime && minFutTime > prevTime
        && i<MAX_ACC_BUF_PKT)
    {
        dt = minFutTime - measTime;
        /*! - Treat rates scaled by dt as a PRV (small angle approximation)*/
        v3Copy(gyrData->accPkts[minFutInd].gyro_B, omeg_BN_B);
        v3Scale(dt, omeg_BN_B, prvTemp);
        
        /*! - Convert the PRV to euler parameters and add that delta-rotation 
              to the running sum (ep_BpropB0)*/
        PRV2EP(prvTemp, ep_B1B0);
        v4Copy(ep_BpropB0, epTemp);
        addEP(epTemp, ep_B1B0, ep_BpropB0);
        ConfigData->gyrAggTimeTag = minFutTime;
        i++;
        /*! - Prepare for the next measurement and set time-tags for termination*/
        measTime = minFutTime;
        /*% operator used because gyro buffer is a ring-buffer and this operator 
            wraps the index back to zero when we overflow.*/
        minFutInd = (minFutInd + 1)%MAX_ACC_BUF_PKT;
        minFutTime = gyrData->accPkts[minFutInd].measTime*NANO2SEC;
        /*! - Apply low-pass filter to gyro measurements to get smoothed body rate*/
        for(j=0; j<3; j++)
        {
            lowPassFilterSignal(omeg_BN_B[j], &(ConfigData->gyroFilt[j]));
        }
    }
    /*! - Saved the measurement count and convert the euler parameters to MRP 
          as that is our filter representation*/
    ConfigData->numUsedGyros = i;
    EP2MRP(ep_BpropB0, ConfigData->aggSigma_b2b1);
    return;
}

/*! This method performs the measurement update for the inertial kalman filter.
 It applies the observations in the obs vectors to the current state estimate and 
 updates the state/covariance with that information.
 @return void
 @param ConfigData The configuration data associated with the CSS estimator
 @param updateTime The time that we need to fix the filter to (seconds)
 */
void inertialUKFMeasUpdate(InertialUKFConfig *ConfigData, double updateTime, int currentST)
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
    inertialUKFMeasModel(ConfigData, currentST);
    
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
        if (ConfigData->wC[i+1]<0){
            ConfigData->badUpdate += 1;
            printf("bad update occured in UKF");
            return;
        }
        vScale(sqrt(ConfigData->wC[i+1]), tempYVec, ConfigData->numObs, tempYVec);
        memcpy(&(AT[i*ConfigData->numObs]), tempYVec,
               ConfigData->numObs*sizeof(double));
    }
    
    /*! - This is the square-root of the Rk matrix which we treat as the Cholesky
        decomposition of the observation variance matrix constructed for our number 
        of observations*/
    ConfigData->badUpdate += ukfCholDecomp(ConfigData->STDatasStruct.STMessages[currentST].noise, ConfigData->numObs, ConfigData->numObs, qChol);
    if (ConfigData->badUpdate<0){
        printf("bad update occured in UKF");
        return;}
    memcpy(&(AT[2*ConfigData->countHalfSPs*ConfigData->numObs]),
           qChol, ConfigData->numObs*ConfigData->numObs*sizeof(double));
    /*! - Perform QR decomposition (only R again) of the above matrix to obtain the 
          current Sy matrix*/
    ConfigData->badUpdate += ukfQRDJustR(AT, 2*ConfigData->countHalfSPs+ConfigData->numObs,
                ConfigData->numObs, rAT);
    if (ConfigData->badUpdate<0){
        printf("bad update occured in UKF");
        return;}

    mCopy(rAT, ConfigData->numObs, ConfigData->numObs, syT);
    mTranspose(syT, ConfigData->numObs, ConfigData->numObs, sy);
    /*! - Shift the matrix over by the difference between the 0th SP-based measurement 
          model and the yBar matrix (cholesky down-date again)*/
    vScale(-1.0, yBar, ConfigData->numObs, tempYVec);
    vAdd(tempYVec, ConfigData->numObs, &(ConfigData->yMeas[0]), tempYVec);
    ConfigData->badUpdate += ukfCholDownDate(sy, tempYVec, ConfigData->wC[0],
                    ConfigData->numObs, updMat);
    if (ConfigData->badUpdate<0){
        printf("bad update occured in UKF");
        return;}

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
        ConfigData->badUpdate += ukfCholDownDate(ConfigData->sBar, xHat, -1.0, ConfigData->numStates, sBarT);
        if (ConfigData->badUpdate<0){
            printf("bad update occured in UKF");
            return;}
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
