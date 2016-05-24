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

#include "attDetermination/CSSEst/cssWlsEst.h"
#include "SimCode/utilities/linearAlgebra.h"
#include "ADCSUtilities/ADCSAlgorithmMacros.h"
#include <string.h>

/*! This method initializes the ConfigData for theCSS WLS estimator.
 It checks to ensure that the inputs are sane and then creates the
 output message
 @return void
 @param ConfigData The configuration data associated with the CSS WLS estimator
 */
void SelfInit_cssWlsEst(CSSWLSConfig *ConfigData, uint64_t moduleID)
{
    
    /*! Begin method steps */
    /*! - Create output message for module */
    ConfigData->OutputMsgID = CreateNewMessage(ConfigData->OutputDataName,
        sizeof(CSSWlsEstOut), "CSSWlsEstOut", moduleID);
    
}

/*! This method performs the second stage of initialization for the CSS sensor
 interface.  It's primary function is to link the input messages that were
 created elsewhere.
 @return void
 @param ConfigData The configuration data associated with the CSS interface
 */
void CrossInit_cssWlsEst(CSSWLSConfig *ConfigData, uint64_t moduleID)
{
    /*! - Loop over the number of sensors and find IDs for each one */
    ConfigData->InputMsgID = subscribeToMessage(ConfigData->InputDataName,
        MAX_NUM_CSS_SENSORS*sizeof(CSSOutputData), moduleID);
    
}

/*! This method computes a least squares fit with the given parameters.  It
 treats the inputs as though they were double dimensioned arrays but they
 are all singly dimensioned for ease of use
 @return success indicator (1 for good, 0 for fail)
 @param numActiveCss The count on input measurements
 @param H The predicted pointing vector for each measurement
 @param W the weighting matrix for the set of measurements
 @param y the observation vector for the valid sensors
 @param x The output least squares fit for the observations
 */
int computeWlsmn(int numActiveCss, double *H, double *W,
                 double *y, double x[3])
{
    double m22[2*2];
    double m32[3*2];
    int status = 0;
    double  m33[3*3];
    double  m33_2[3*3];
    double  m3N[3*MAX_NUM_CSS_SENSORS];
    double  m3N_2[3*MAX_NUM_CSS_SENSORS];
    uint32_t i;
    
    /*! Begin method steps */
    /*! - If we only have one sensor, output best guess (cone of possiblities)*/
    if(numActiveCss == 1) {
        /* Here's a guess.  Do with it what you will. */
        for(i = 0; i < 3; i=i+1) {
            x[i] = H[0*MAX_NUM_CSS_SENSORS+i] * y[0];
        }
    } else if(numActiveCss == 2) { /*! - If we have two, then do a 2x2 fit */
        
        /*!   -# Find minimum norm solution */
        mMultMt(H, 2, 3, H, 2, 3, m22);
        status = m22Inverse(RECAST2x2 m22, RECAST2x2 m22);
        mtMultM(H, 2, 3, m22, 2, 2, m32);
        /*!   -# Multiply the Ht(HHt)^-1 by the observation vector to get fit*/
        mMultV(m32, 3, 2, y, x);
    } else if(numActiveCss > 2) {/*! - If we have more than 2, do true LSQ fit*/
        /*!    -# Use the weights to compute (HtWH)^-1HW*/
        mtMultM(H, numActiveCss, 3, W, numActiveCss, numActiveCss, m3N);
        mMultM(m3N, 3, numActiveCss, H, numActiveCss, 3, m33);
        status = m33Inverse(RECAST3X3 m33, RECAST3X3 m33_2);
        mMultMt(m33_2, 3, 3, H, numActiveCss, 3, m3N);
        mMultM(m3N, 3, numActiveCss, W, numActiveCss, numActiveCss, m3N_2);
        /*!    -# Multiply the LSQ matrix by the obs vector for best fit*/
        mMultV(m3N_2, 3, numActiveCss, y, x);
    }
    
    return(status);
}

/*! This method takes the parsed CSS sensor data and outputs an estimate of the
 sun vector in the ADCS body frame
 @return void
 @param ConfigData The configuration data associated with the CSS estimator
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void Update_cssWlsEst(CSSWLSConfig *ConfigData, uint64_t callTime,
    uint64_t moduleID)
{
    
    uint64_t ClockTime;
    uint32_t ReadSize;
    CSSOutputData InputBuffer[MAX_NUM_CSS_SENSORS];
    double H[MAX_NUM_CSS_SENSORS*3];
    double y[MAX_NUM_CSS_SENSORS];
    double W[MAX_NUM_CSS_SENSORS*MAX_NUM_CSS_SENSORS];
    uint32_t i;
    uint32_t status;
    
    /*! Begin method steps*/
    /*! - Read the input parsed CSS sensor data message*/
    memset(InputBuffer, 0x0, MAX_NUM_CSS_SENSORS*sizeof(CSSOutputData));
    ReadMessage(ConfigData->InputMsgID, &ClockTime, &ReadSize,
                MAX_NUM_CSS_SENSORS*sizeof(CSSOutputData),
                (void*) (InputBuffer), moduleID);
    
    /*! - Zero the observed active CSS count*/
    ConfigData->numActiveCss = 0;
    
    /*! - Loop over the maximum number of sensors to check for good measurements
     -# Isolate if measurement is good
     - Set body vector for this measurement
     - Get measurement value into observation vector
     - Set inverse noise matrix
     - increase the number of valid observations
     -# Otherwise just continue
     */
    for(i=0; i<MAX_NUM_CSS_SENSORS; i = i+1)
    {
        if(InputBuffer[i].CosValue > ConfigData->SensorUseThresh)
        {
            v3Scale(ConfigData->CSSData[i].CBias,
                ConfigData->CSSData[i].nHatBdy, &H[ConfigData->numActiveCss*3]);
            y[ConfigData->numActiveCss] = InputBuffer[i].CosValue;
            ConfigData->numActiveCss = ConfigData->numActiveCss + 1;
            
        }
    }
    
    if(ConfigData->numActiveCss == 0) /*! - If there is no sun, just quit*/
    {
        return;
    }
    /*! - Configuration option to weight the measurements, otherwise set
     weighting matrix to identity*/
    if(ConfigData->UseWeights > 0)
    {
        mDiag(y, ConfigData->numActiveCss, W);
    }
    else
    {
        mSetIdentity(W, ConfigData->numActiveCss, ConfigData->numActiveCss);
    }
    
    /*! - Get least squares fit for sun pointing vector*/
    status = computeWlsmn(ConfigData->numActiveCss, H, W, y,
                          ConfigData->OutputData.sHatBdy);
    v3Normalize(ConfigData->OutputData.sHatBdy, ConfigData->OutputData.sHatBdy);
    WriteMessage(ConfigData->OutputMsgID, callTime, sizeof(CSSWlsEstOut),
                 &(ConfigData->OutputData), moduleID);
    return;
}
