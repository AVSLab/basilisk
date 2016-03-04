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

#ifndef ST_INERTIAL_UKF_hh
#define ST_INERTIAL_UKF_hh

#include <stdint.h>
#include <vector>
#include "SimCode/Utilities/sys_model.h"
#include "../_GeneralModuleFiles/unscent_kalfilt.h"

typedef struct {
	double TimeTag;          // s  Current Seconds elapsed since start
	double NEDOutputQuat[4]; // -- Current NED2Bdy Quaternion est
}AttOutputStruct;


class  STInertialUKF : public SysModel, public UnscentKalFilt {
public:
   STInertialUKF();
   ~STInertialUKF();
   void UpdateState(uint64_t callTime);
   void SelfInit();
   void CrossInit();
   void StateProp(MatrixOperations &StateCurr);
   void ComputeMeasModel();
   void CheckForUpdate(double TimeLatest);
   void DetermineConvergence(double CovarNorm);
       
public:
   
   bool ReInitFilter;       // -- Command to reset filter
   double QNoiseInit[6*6]; // -- Qnoise matrix to init with
   double CovarInit[6*6];  // -- Covariance matrix to start out with
   double QStObs[3*3];   // -- observation noise matrix
   double ConvThresh;       // -- Required level of covariance convergence
   int ConvTestCounter;     // -- Number of sequential passes that must satisfy
   double HTolerance;       // s  Level of agreement required for time-tagging
   
   int ConvCounter;         // -- Current counter of Convergence Values
   bool FilterConverged;    // -- Indicator that filter has converged

   double MRP_BdyInrtl_Init[4];       // -- Initialization value for modified rodrigues parameters
   double w_BdyInrtl_Bdy[3];        // -- Initial body rate estimate to seed filter with
   double LastStTime;       // -- Last Accelerometer time-tag

   double CovarEst[3*3];   // -- Covariance estimate output from filter

   std::string stInputName; // -- Input message name for star tracker data
   uint64_t stInputID;  // -- Input port ID
   std::string wheelSpeedsName; // -- Name for reaction wheel speed measurement
   uint64_t wheelSpeedsID; // -- Input message for the wheel speeds
   std::string InertialUKFStateName; // -- Output sampling port name
   uint32_t InertialUKFStateID; // -- Output port ID for NED value
};


#endif
