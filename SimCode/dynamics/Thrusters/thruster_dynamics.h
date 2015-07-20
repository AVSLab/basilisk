
#ifndef ORB_ELEM_CONVERT_H
#define ORB_ELEM_CONVERT_H

#include <vector>
#include "utilities/sys_model.h"
#include "utilities/dyn_effector.h"

typedef struct {
   double ThrustFactor;                 // -- Percentage of max thrust
   double TimeDelta;                    // s  Time delta from start of event
}ThrusterTimePair;

typedef struct {
   double ThrustFactor;                // -- Current Thrust Percentage
   double ThrustOnRampTime;             // s  Time thruster has been on for
   double ThrustOnSteadyTime;           // s  Time thruster has been on steady
   double ThrustOffRampTime;            // s  Time thruster has been turning off
   double ThrusterStartTime;            // s  Time thruster has been executing total
   double ThrustOnCmd;                  // s  Time Thruster was requested
   double PreviousIterTime;             // s  Previous thruster int time
}ThrusterOperationData;

typedef struct {
   std::vector<double> ThrusterLocation;// m Location of thruster in structural
   std::vector<double> ThrusterDirection; // -- Unit vector of thruster pointing
   std::vector<ThrusterTimePair> ThrusterOnRamp;  // -- Percentage of max thrust for ramp up
   std::vector<ThrusterTimePair> ThrusterOffRamp; // -- Percentage of max thrust for ramp down
   double MaxThrust;                    // N  Steady state thrust of thruster
   double MinOnTime;                    // s  Minimum allowable on-time
   ThrusterOperationData ThrustOps;     // -- Thruster operating data
}ThrusterConfigData;

typedef struct {
   double OnTimeRequest;                // s Requested on-time for thruster
}ThrustCmdStruct;

class ThrusterDynamics: public SysModel, public DynEffector {
public:
   ThrusterDynamics();
   ~ThrusterDynamics();
 
   void SelfInit();
   void CrossInit();
   void AddThruster(ThrusterConfigData *NewThruster) {ThrusterData.push_back(*NewThruster);}
   void UpdateState(uint64_t CurrentSimNanos);
   void WriteOutputMessages(uint64_t CurrentClock);
   void ReadInputs();
   void ConfigureThrustRequests(double CurrentTime);
   void ComputeDynamics(MassPropsData *Props, OutputStateData *Bstate, 
      double CurrentTime);
   void ComputeThrusterFire(ThrusterConfigData *CurrentThruster, 
      double CurrentTime);
   void ComputeThrusterShut(ThrusterConfigData *CurrentThruster, 
      double CurrentTime);
       
public:
   std::vector<ThrusterConfigData> ThrusterData;  // -- Thruster information
   std::string InputCmds;            // -- message used to read command inputs
   std::string OutputDataString;     // -- port to use for output data
   uint64_t OutputBufferCount;       // -- Count on number of buffers to output
   std::vector<double> NewThrustCmds;// -- Incoming thrust commands
   double StrForce[3];               // N  Computed force in str for thrusters
   double StrTorque[3];              // Nm Computed torque in str for thrusters

private:
   int64_t CmdsInMsgID;               // -- MEssage ID for incoming data
   int64_t StateOutMsgID;             // -- Message ID for outgoing data
   ThrustCmdStruct *IncomingCmdBuffer;// -- One-time allocation for savings
};

#endif
