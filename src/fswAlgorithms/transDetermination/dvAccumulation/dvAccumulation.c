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

#include "transDetermination/dvAccumulation/dvAccumulation.h"
#include "simFswInterfaceMessages/macroDefinitions.h"
#include "utilities/linearAlgebra.h"
#include "simulation/utilities/bsk_Print.h"
#include <string.h>
#include <stdlib.h>

/*! This method initializes the configData for the nav aggregation algorithm.
    It initializes the output message in the messaging system.
 @return void
 @param configData The configuration data associated with the Nav aggregation interface
 */
void SelfInit_dvAccumulation(DVAccumulationData *configData, int64_t moduleID)
{
    configData->bskPrint = _BSKPrint();
    configData->outputNavMsgID = CreateNewMessage(configData->outputNavName,
        sizeof(NavTransIntMsg), "NavTransIntMsg", moduleID);
}

/*! This method performs the second stage of initialization for the nav aggregration
    interface.  For each configured message, it subscribes to the target message
    and saves the ID.
 @return void
 @param configData The configuration data associated with the aggregate nav interface
 */
void CrossInit_dvAccumulation(DVAccumulationData *configData, int64_t moduleID)
{
        configData->accPktInMsgID= subscribeToMessage(
            configData->accPktInMsgName, sizeof(AccDataFswMsg), moduleID);
}

void Reset_dvAccumulation(DVAccumulationData *configData, uint64_t callTime,
                          int64_t moduleID)
{
    /*! - Configure accumulator to reset itself*/
    AccDataFswMsg inputAccData;
    uint64_t timeOfMsgWritten;
    uint32_t sizeOfMsgWritten;
    int i;

    /*! - zero the input message container and read in the accelerometer data message */
    memset(&inputAccData, 0x0, sizeof(AccDataFswMsg));
    ReadMessage(configData->accPktInMsgID, &timeOfMsgWritten, &sizeOfMsgWritten,
                sizeof(AccDataFswMsg), (void *) &inputAccData, moduleID);

    /*! - stacks data in time order*/
    dvAccumulation_QuickSort(&(inputAccData.accPkts[0]), 0, MAX_ACC_BUF_PKT-1);

    /*! - reset accumulated DV vector to zero */
    v3SetZero(configData->vehAccumDV_B);

    /*! - reset previous time value to zero */
    configData->previousTime = 0;

    /* - reset initialization flag */
    configData->dvInitialized = 0;

    /*! - If we find valid timestamp, ensure that no "older" meas get ingested*/
    for(i=(MAX_ACC_BUF_PKT-1); i>=0; i--)
    {
        if(inputAccData.accPkts[i].measTime > 0)
        {
            /* store the newest time tag found as the previous time tag */
            configData->previousTime = inputAccData.accPkts[i].measTime;
            break;
        }
    }
}

/*////////////////////////////////////////////////////Experimenting QuickSort START////////////////*/
void dvAccumulation_swap(AccPktDataFswMsg *p, AccPktDataFswMsg *q){
    AccPktDataFswMsg t;
    t=*p;
    *p=*q;
    *q=t;
}
int dvAccumulation_partition(AccPktDataFswMsg *A, int start, int end){
    int i;
    uint64_t pivot=A[end].measTime;
    int partitionIndex=start;
    for(i=start; i<end; i++){
        if(A[i].measTime<=pivot){
            dvAccumulation_swap(&(A[i]), &(A[partitionIndex]));
            partitionIndex++;
        }
    }
    dvAccumulation_swap(&(A[partitionIndex]), &(A[end]));
    return partitionIndex;
}

/*! Sort the AccPktDataFswMsg by the measTime with an iterative quickSort.
  @return void
  @param A --> Array to be sorted,
  @param start  --> Starting index,
  @param end  --> Ending index */
void dvAccumulation_QuickSort (AccPktDataFswMsg *A, int start, int end)
{
    /*! - Create an auxiliary stack array. This contains indicies. */
    int stack[MAX_ACC_BUF_PKT];
    if((end-start + 1) > MAX_ACC_BUF_PKT)
    {
        _printMessage(A->bskPrint, MSG_ERROR, "Stack insufficiently sized for quick-sort somehow");
    }

    /*! - initialize the index of the top of the stack */
    int top = -1;

    /*! - push initial values of l and h to stack */
    stack[ ++top ] = start;
    stack[ ++top ] = end;

    /*! - Keep popping from stack while is not empty */
    while ( top >= 0 )
    {
        /* Pop h and l */
        end = stack[ top-- ];
        start = stack[ top-- ];

        /*! - Set pivot element at its correct position in sorted array */
        int partitionIndex = dvAccumulation_partition( A, start, end );

        /*! - If there are elements on left side of pivot, then push left side to stack */
        if ( partitionIndex-1 > start )
        {
            stack[ ++top ] = start;
            stack[ ++top ] = partitionIndex - 1;
        }

        /*! - If there are elements on right side of pivot, then push right side to stack */
        if ( partitionIndex+1 < end )
        {
            stack[ ++top ] = partitionIndex + 1;
            stack[ ++top ] = end;
        }
    }
}
/*////////////////////////////////////////////////////Experimenting QuickSort END////////////////*/


/*! This method takes the navigation message snippets created by the various
    navigation components in the FSW and aggregates them into a single complete
    navigation message.
 @return void
 @param configData The configuration data associated with the aggregate nav module
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void Update_dvAccumulation(DVAccumulationData *configData, uint64_t callTime, int64_t moduleID)
{
    uint64_t timeOfMsgWritten;
    uint32_t sizeOfMsgWritten;
    int i;
    double dt;
    double frameDV_B[3];            /* [m/s] The DV of an integrated acc measurement */
    AccDataFswMsg inputAccData;     /* [-] Input message container */
    NavTransIntMsg outputData;      /* [-] The local storage of the outgoing message data */

    /*! - zero input and output message container */
    memset(&inputAccData, 0x0, sizeof(AccDataFswMsg));
    memset(&outputData, 0x0, sizeof(NavTransIntMsg));

    /*! - read accelerometer input message */
    ReadMessage(configData->accPktInMsgID, &timeOfMsgWritten, &sizeOfMsgWritten,
                sizeof(AccDataFswMsg), &inputAccData, moduleID);

    /*! - stack data in time order */

    dvAccumulation_QuickSort(&(inputAccData.accPkts[0]), 0, MAX_ACC_BUF_PKT-1); /* measTime is the array we want to sort. We're sorting the time calculated for each measurement taken from the accelerometer in order in terms of time. */

    /*! - Ensure that the computed dt doesn't get huge.*/
    if(configData->dvInitialized == 0)
    {
        for(i=0; i<MAX_ACC_BUF_PKT; i++)
        {
            if(inputAccData.accPkts[i].measTime > configData->previousTime)
            {
                configData->previousTime = inputAccData.accPkts[i].measTime;
                configData->dvInitialized = 1;
                break;
            }
        }
    }

    /*! - process new accelerometer data to accumulate Delta_v */
    for(i=0; i<MAX_ACC_BUF_PKT; i++)
    {
        /*! - see if data is newer than last data time stamp */
        if(inputAccData.accPkts[i].measTime > configData->previousTime)
        {
            dt = (inputAccData.accPkts[i].measTime - configData->previousTime)*NANO2SEC;
            v3Scale(dt, inputAccData.accPkts[i].accel_B, frameDV_B);
            v3Add(configData->vehAccumDV_B, frameDV_B, configData->vehAccumDV_B);
            configData->previousTime = inputAccData.accPkts[i].measTime;
        }
    }

    /*! - Create output message */

    outputData.timeTag = configData->previousTime;
    v3Copy(configData->vehAccumDV_B, outputData.vehAccumDV);

    /*! - write accumulated Dv message */
    WriteMessage(configData->outputNavMsgID, callTime, sizeof(NavTransIntMsg),
                 &outputData, moduleID);

    return;

}
