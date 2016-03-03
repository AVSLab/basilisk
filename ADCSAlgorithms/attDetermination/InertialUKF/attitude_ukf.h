
#ifndef ATTITUDE_UKF_hh
#define ATTITUDE_UKF_hh

#include <stdint.h>
#include <vector>
#include "SimCode/utilities/sys_model.h"
#include "../_GeneralModuleFiles/unscent_kalfilt.h"
#include "../_GeneralModuleFiles/matrix_operations.h"

typedef struct {
	double TimeTag;                     // -- Time-stamp of data
	double AccumulatedMeas[3];          // -- Measurement data
}ImuMeasEntry;

typedef struct {
   double TimeTag;          // s Time-tag associated with quaternion
   double Qstate[4];        // -- Quaternion state estimate
}UKFQState;

typedef struct {
	double TimeTag;            // s Time-tag associated with filtered Quaternion
	double QFilter[4];         // -- Quaternion estimated by filter
	uint64_t UpdateCounter;    // -- Counter for filter updates
	double SBarMat[3][3];      // -- Quasi covariance output
	double CovarMat[3][3];     // -- Covariance matrix output
	double StateValues[3];     // -- MRP values for current state
}UKFAttFiltData;

class  AttitudeUKF : public SysModel, public UnscentKalFilt {
public:
   AttitudeUKF();
   ~AttitudeUKF();
   void GNC_alg();
   void GNC_config();
   void GNC_alg_init();
   virtual void StateProp(MatrixOperations &StateCurr);
   virtual void ComputeMeasModel();
   virtual MatrixOperations ComputeSPVector(
       MatrixOperations SPError,
        MatrixOperations StateIn);
    virtual MatrixOperations ComputeObsError(
       MatrixOperations SPError,
        MatrixOperations StateIn);
   void PropagateOutput(std::vector<ImuMeasEntry> &SortedDRs);
   std::vector<ImuMeasEntry> SortDRs( 
      std::vector<ImuMeasEntry> DRBuffer);
   void AdvanceState(
      MatrixOperations &QCurrent,
      MatrixOperations DRAdvance);
   void CheckForUpdate(
      UKFQState TestState);
   void UpdateStateToMeasurement();
   void DetermineConvergence(
      double CovarNorm);
   void ApplyFilterState();
       
//   bool WriteCheckpoint();
//   bool DisplayCheckpoint();
//   bool ResetFromCheckpoint();

public:
   //RingBuffer<UKFQState> QBuffer; // -- Ring buffer of quaternions
   //NEDOutputStruct PrevNEDEst; // -- Previous Tracker measurement
   int    RingLength;       // -- Redundant but needed for Trick
   double SmallAngThresh;   // r  Definition of "small"
   bool ReInitFilter;       // -- Command to reset filter
   double QNoiseInit[3][3]; // -- Qnoise matrix to init with
   double CovarInit[3][3];  // -- Covariance matrix to start out with
   double QStrObs[3][3];    // -- observation noise matrix
   double T_Imu2Str[3][3];  // -- Conversion matrix from IMu chass to veh str
   double ConvThresh;       // -- Required level of covariance convergence
   int ConvTestCounter;     // -- Number of sequential passes that must satisfy
   
   double DRLast[3];        // r  Last snapped angular rotation
   double StateScalar;      // -- Scalar value to use in quat filter
   double NewScalar;        // -- New scalar to update with
   double Q_B1B2[4];        // -- Transformation from Bdy 1 to Bdy 2
   double CurrGyroBias[3];  // r/s Current gyro bias estimate
   int ConvCounter;         // -- Current counter of Convergence Values
   bool FilterConverged;    // -- Indicator that filter has converged

   double QTimeTag;         // s  Nav time associated with quaternion/omega
   double Q_ECI2Bdy[4];     // -- Current estimate of the Quaternion
   double Q_Filter[4];      // -- Quaternion estimate from filter
   double QVecFilt[3];      // -- Filtered quaternion vector estimate
   double CovarEst[3][3];   // -- Covariance estimate output from filter
   uint64_t UpdateCounter;  // -- Counter for number of measurement updates in filter

   std::string InputDRBufferName; // -- Input DR buffer name
   std::string InputNEDFiltName;  // -- Input NED buffer name
   std::string InputGyroBiasName; // -- Input Gyro bias estimate
   std::string OutputAttEstName;  // -- Output Attitude estimate
   std::string OutputFiltDatName; // -- Output filtered data SP name
   uint32_t InputDRBufferID; // -- Input DR buffer name
   uint32_t InputNEDFiltID;  // -- Input NED buffer name
   uint32_t InputGyroBiasID; // -- Input gyro bias ID
   uint32_t OutputAttEstID;  // -- Output Attitude estimate
   uint32_t OutputFiltDatID; // -- Output filtered data SP ID

   //GenericCheckpoint checkpoint; // -- Generic checkpoint routine for restarts
   //char *checkpointFile;    // -- (Path and) file name for checkpoint to read/write
   //char *checkpointName;    // -- User-defined name for a given checkpoint
};


#endif
