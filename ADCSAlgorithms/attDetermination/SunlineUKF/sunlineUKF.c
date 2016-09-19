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
    
    /*! Begin method steps */
    /*! - Create output message for module */
	ConfigData->outputStateID = CreateNewMessage(ConfigData->outputNavStateName, 
		sizeof(NavAttOut), "NavAttOut", moduleID);
	int i;

	ConfigData->numStates = SKF_N_STATES;
	ConfigData->numObs = MAX_N_CSS_MEAS;
	vSetZero(ConfigData->state, ConfigData->numStates);
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
		ConfigData->wC[i] = ConfigData->wC[i];
	}
	mCopy(ConfigData->covar, ConfigData->numStates, ConfigData->numStates, 
		ConfigData->sBar);
	ukfCholDecomp(ConfigData->sBar, ConfigData->numStates,
		ConfigData->numStates, ConfigData->sBar);
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
    int32_t i;
    vehicleConfigData localConfigData;
    uint64_t writeTime;
    uint32_t writeSize;
    /*! - Loop over the number of sensors and find IDs for each one */
    ConfigData->inputCSSDataID = subscribeToMessage(ConfigData->inputCSSDataName,
        MAX_NUM_CSS_SENSORS*sizeof(CSSOutputData), moduleID);
    ConfigData->inputPropsID = subscribeToMessage(ConfigData->inputPropsName,
        sizeof(vehicleConfigData), moduleID);
    ReadMessage(ConfigData->inputPropsID, &writeTime, &writeSize,
                sizeof(vehicleConfigData), &localConfigData, moduleID);
    /*for(i=0; i<MAX_NUM_CSS_SENSORS; i = i+1)
    {
        m33MultV3(RECAST3X3 localConfigData.BS, ConfigData->CSSData[i].nHatStr,
                  ConfigData->CSSData[i].nHatBdy);
    }*/
    
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
    
    return;
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
	return;
}

/*! This method propagates a sunline state vector forward in time.  Note 
    that the calling parameter is updated in place to save on data copies.
	@return void
	@param stateInOut The state that is propagated
*/
void sunlineStateProp(double *stateInOut)
{
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
	double xComp[SKF_N_STATES], AT[(2 * SKF_N_STATES + 1)*SKF_N_STATES];
	double aRow[SKF_N_STATES], rAT[SKF_N_STATES*SKF_N_STATES], xErr[SKF_N_STATES]; 
	double xErrT[SKF_N_STATES], SPErrMat[SKF_N_STATES*SKF_N_STATES]; 
	double sBarUp[SKF_N_STATES*SKF_N_STATES];
	double *spPtr;

	mTranspose(ConfigData->sBar, ConfigData->numStates, ConfigData->numStates,
		sBarT);
	
	ConfigData->dt = updateTime - ConfigData->TimeTag;
	vCopy(ConfigData->state, ConfigData->numStates,
		&(ConfigData->SP[0 * ConfigData->numStates + 0]));

	sunlineStateProp(&(ConfigData->SP[0 * ConfigData->numStates + 0]));
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
		sunlineStateProp(spPtr);
		vCopy(spPtr, ConfigData->numStates, xComp);
		vScale(ConfigData->wM[Index], xComp, ConfigData->numStates, xComp);
		vAdd(xComp, ConfigData->numStates, xBar, xBar);
		
		/*Index = i + 1 + this->CountHalfSPs;
		this->SP[Index].MatOps_VecSet(&SbarT.vec_vals[i*this->NumStates]);
		this->SP[Index].MatOps_scale(-this->gamma);
		this->SP[Index].MatOps_add(this->SP[Index], state);
		this->StateProp(this->SP[Index]);
		Xcomp = this->SP[Index];
		Xcomp.MatOps_scale(this->Wm.vec_vals[Index]);
		Xbar.MatOps_add(Xbar, Xcomp);*/
	}
	/*AT.MatOps_init(2 * this->CountHalfSPs + this->NumStates, this->NumStates);
	for (i = 0; i<2 * this->CountHalfSPs; i++)
	{
		ARow = Xbar;
		ARow.MatOps_scale(-1.0);
		ARow.MatOps_add(ARow, SP[i + 1]);
		ARow.MatOps_scale(sqrt(this->Wc.vec_vals[i + 1]));
		memcpy((void *)&AT.vec_vals[i*this->NumStates], (void *)ARow.vec_vals,
			this->NumStates*sizeof(double));
	}
	memcpy(&AT.vec_vals[2 * this->CountHalfSPs*this->NumStates],
		this->SQnoise.vec_vals, this->NumStates*this->NumStates*sizeof(double));
	RAT.MatOps_QRD_JustR(AT);
	memcpy(SbarT.vec_vals, RAT.vec_vals, this->NumStates*this->NumStates
		*sizeof(double));
	this->Sbar.MatOps_transpose(SbarT);
	Xerr = Xbar;
	Xerr.MatOps_scale(-1.0);
	Xerr.MatOps_add(Xerr, SP[0]);
	XerrT.MatOps_transpose(Xerr);
	SPErrMat.MatOps_mult(Xerr, XerrT);
	SPErrMat.MatOps_scale(this->Wc.vec_vals[0]);
	SBarUp.MatOps_mult(this->Sbar, SbarT);
	SBarUp.MatOps_add(SBarUp, SPErrMat);
	this->Sbar.MatOps_CholDecomp(SBarUp);
	SbarT.MatOps_transpose(this->Sbar);
	this->SP[0] = Xbar;
	for (i = 0; i<this->CountHalfSPs; i++)
	{
		Index = i + 1;
		this->SP[Index].MatOps_VecSet(&SbarT.vec_vals[i*SbarT.dim_array[0]]);
		this->SP[Index].MatOps_scale(gamma);
		this->SP[Index].MatOps_add(this->SP[Index], Xbar);
		Index = i + 1 + this->CountHalfSPs;
		this->SP[Index].MatOps_VecSet(&SbarT.vec_vals[i*SbarT.dim_array[0]]);
		this->SP[Index].MatOps_scale(-gamma);
		this->SP[Index].MatOps_add(this->SP[Index], Xbar);
	}
	Covar.MatOps_transpose(Sbar);
	Covar.MatOps_mult(Sbar, Covar);
	state = SP[0];
	TimeTag = UpdateTime;*/
}
