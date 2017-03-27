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

#include "transDetermination/dvAccumulation/dvAccumulation.h"
#include "simFswInterfaceMessages/macroDefinitions.h"
#include "utilities/linearAlgebra.h"
#include <string.h>
#include <stdlib.h>

/*! This method initializes the ConfigData for the nav aggregation algorithm.  
    It initializes the output message in the messaging system.
 @return void
 @param ConfigData The configuration data associated with the Nav aggregation interface
 */
void SelfInit_dvAccumulation(DVAccumulationData *ConfigData, uint64_t moduleID)
{
    ConfigData->outputNavMsgID = CreateNewMessage(ConfigData->outputNavName,
        sizeof(NavTransIntMsg), "NavTransIntMsg", moduleID);
}

/*! This method performs the second stage of initialization for the nav aggregration 
    interface.  For each configured message, it subscribes to the target message 
    and saves the ID.
 @return void
 @param ConfigData The configuration data associated with the aggregate nav interface
 */
void CrossInit_dvAccumulation(DVAccumulationData *ConfigData, uint64_t moduleID)
{
        ConfigData->accPktInMsgID= subscribeToMessage(
            ConfigData->accPktInMsgName, sizeof(AccDataFswMsg), moduleID);
}

/*////////////////////////////////////////////////////Experimenting QuickSort START////////////////*/
void swap(AccPktDataFswMsg *p, AccPktDataFswMsg *q){
    AccPktDataFswMsg t;
    t=*p;
    *p=*q;
    *q=t;
}
int partition(AccPktDataFswMsg *A, int start, int end){
    int i;
    uint64_t pivot=A[end].measTime;
    int partitionIndex=start;
    for(i=start; i<end; i++){
        if(A[i].measTime<=pivot){
            swap(&(A[i]), &(A[partitionIndex]));
            partitionIndex++;
        }
    }
    swap(&(A[partitionIndex]), &(A[end]));
    return partitionIndex;
}
void QuickSort(AccPktDataFswMsg *A, int start, int end){
    if(start<end){
        int partitionIndex=partition(A, start, end);
        QuickSort(A, start, partitionIndex-1);
        QuickSort(A, partitionIndex+1, end);
    }
}
/*////////////////////////////////////////////////////Experimenting QuickSort END////////////////*/


/*! This method takes the navigation message snippets created by the various
    navigation components in the FSW and aggregates them into a single complete
    navigation message.
 @return void
 @param ConfigData The configuration data associated with the aggregate nav module
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void Update_dvAccumulation(DVAccumulationData *ConfigData, uint64_t callTime, uint64_t moduleID)
{
    uint64_t writeTime;
    uint32_t writeSize;
//    uint64_t measTime[MAX_ACC_BUF_PKT];
//    double accel_pltf[MAX_ACC_BUF_PKT];
//    double DV[MAX_ACC_BUF_PKT];
//    double sum[MAX_ACC_BUF_PKT];
//    double dvMean[MAX_ACC_BUF_PKT];
    double dt;
    double frameDVPlt[3];
    double frameDVBdy[3];
    double frameDVStr[3];
    AccDataFswMsg inputAccData;
    int i;
    
    ReadMessage(ConfigData->accPktInMsgID, &writeTime, &writeSize,
                sizeof(AccDataFswMsg), &inputAccData, moduleID);
   
    /* stacks data in time order*/
    QuickSort(&(inputAccData.accPkts[0]), 0, MAX_ACC_BUF_PKT-1); //measTime is the array we want to sort. We're sorting the time calculated for each measurement taken from the accelerometer in order in terms of time.
    
    for(i=0; i<MAX_ACC_BUF_PKT; i++)
    {
        if(inputAccData.accPkts[i].measTime > ConfigData->previousTime)
        {
            dt = (inputAccData.accPkts[i].measTime - ConfigData->previousTime)*NANO2SEC;
            v3Scale(dt, inputAccData.accPkts[i].accel_Pltf, frameDVPlt);
            m33MultV3(RECAST3X3 ConfigData->dcm_SPltf, frameDVPlt, frameDVStr);
            v3Copy(frameDVStr, frameDVBdy);
            v3Add(ConfigData->outputData.vehAccumDV, frameDVBdy,
                ConfigData->outputData.vehAccumDV);
            ConfigData->previousTime = inputAccData.accPkts[i].measTime;
        }
    }
    
    WriteMessage(ConfigData->outputNavMsgID, callTime, sizeof(NavTransIntMsg),
                 &ConfigData->outputData, moduleID);
    
    //for(int k=0; k<XXX; k=k+measTime){ // the XXX indicates that I'm not sure of what the maximum end limit for the loop. When does the measurement stop? ... Also here, I'm assuming measTime is the time of each frame interval (of accel when DVs being off-pulsed)
    //    for(int j=0; j<ConfigData->msgCount; j++){ //
    //        uint32_t a=0; //a is the initial start time.. but I need it to continue going.. MUST FIND OUT HOW!!!
    //        uint32_t b=0.5;
            /*calculate the DV, integrate, and find the mean*/
    //        for(int i=a; i<=b; i+=(b-a)/6){
    //            DV[i]=(measTime[i])*(accel_pltf[i]);
    //            sum[i]+=DV[i]*(b-a)/6;
    //            dvMean[i]=sum[i]/(b-a);
    //        }
/* cutoff burn & stop accel if reaches beyond threshold*/
   //     }
   // }
    
/*obtain acceleration measurements data & figures out the time and size*/
/*! Begin method steps */
/*! - Iterate through all of the input messages and archive their nav data*/
//    for(int i=0; i<ConfigData->msgCount; i=i+1)
//    {
//        ReadMessage(ConfigData->outputNavMsgID, &writeTime, &ConfigData->msgCount,
//                    sizeof(NavTransIntMsg), &(ConfigData->outputData), moduleID);
//    }
    
    return;

}
