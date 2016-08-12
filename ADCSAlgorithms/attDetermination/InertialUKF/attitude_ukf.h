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
#include "SimCode/_GeneralModuleFiles/sys_model.h"
#include "../_GeneralModuleFiles/unscent_kalfilt.h"
#include "sensorInterfaces/STSensorData/stComm.h"
#include "../_GeneralModuleFiles/navStateOut.h"
#include "effectorInterfaces/_GeneralModuleFiles/rwSpeedData.h"
typedef struct {
    double gsAxBdy[3];      //! [-] Spin axis for RWA in body frame
    double Js;              //! [kgm2] Spin axis inertia for the RWA
} RWConfigElement;

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
    MatrixOperations ComputeSPVector(
                                     MatrixOperations SPError,
                                     MatrixOperations StateIn);
    MatrixOperations ComputeObsError(
                                     MatrixOperations SPError,
                                     MatrixOperations StateIn);
    void appendRWInformation(RWConfigElement *newElement) {rwData.push_back(*newElement);}
    
    
public:
    
    bool ReInitFilter;       // -- Command to reset filter
    bool initToMeas;         // -- Command to initialize to measurement
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
    double MRPPrevious[3];
    double IInv[9];          //!< [-] Inverse of the spacecraft inertia tensor
    STOutputData stMeas;     //!< [-] Current star tracker measurement
    double LastStTime;       // -- Last Accelerometer time-tag
    
    double CovarEst[6*6];   // -- Covariance estimate output from filter
    NavStateOut localOutput; //! -- Current output state estimate
    RWSpeedData currentSpeeds; //! [-] Current estimate of the wheel speeds
    RWSpeedData previousSpeeds; //! [-] Previous set of wheel speeds
    std::vector<RWConfigElement> rwData; //! [-] Vector of reaction wheel configuration data
    
    
    std::string stInputName; // -- Input message name for star tracker data
    uint64_t stInputID;  // -- Input port ID
    std::string wheelSpeedsName; // -- Name for reaction wheel speed measurement
    uint64_t wheelSpeedsID; // -- Input message for the wheel speeds
    std::string InertialUKFStateName; // -- Output sampling port name
    uint32_t InertialUKFStateID; // -- Output port ID for NED value
    std::string inputRWSpeeds; //! [-] Input RWA speeds message name
    uint32_t inputSpeedsID; //! [-] Input port ID for RWA speeds
    std::string inputVehicleConfigDataName; //! [-] Input vehicle configuration name
    uint32_t inputVehicleConfigDataID; //! [-] Input vehicle configuration ID
};


#endif
