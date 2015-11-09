
#ifndef THRUSTER_DYNAMICS_H
#define THRUSTER_DYNAMICS_H

#include <vector>
#include "utilities/sys_model.h"
#include "utilities/dyn_effector.h"

/*! \addtogroup SimModelGroup
 * @{
 */

//! @brief Container for time/value pairs for ramp on and ramp off profiles.
/*! This structure is used to build up the thruster on and thruster off ramps.
 It pairs the time-delta from on command and thrust factor (percentage/100.0)
 for the entire ramp.*/
typedef struct {
    double ThrustFactor;                 //!< -- Percentage of max thrust
    double IspFactor;                    //!< s  fraction specific impulse 
    double TimeDelta;                    //!< s  Time delta from start of event
}ThrusterTimePair;

//! @brief Container for current operational data of a given thruster
/*! This structure is used to determine the current state of a given thruster.
 It defines where in the cycle the thruster is and how much longer it should be
 on for.  It is intended to have the previous firing remain resident for logging*/
typedef struct {
    double ThrustFactor;                 //!< -- Current Thrust Percentage
    double IspFactor;                    //!< -- Current fractional ISP
    double ThrustOnRampTime;             //!< s  Time thruster has been on for
    double ThrustOnSteadyTime;           //!< s  Time thruster has been on steady
    double ThrustOffRampTime;            //!< s  Time thruster has been turning off
    double ThrusterStartTime;            //!< s  Time thruster has been executing total
    double ThrustOnCmd;                  //!< s  Time Thruster was requested
    double PreviousIterTime;             //!< s  Previous thruster int time
    uint64_t fireCounter;                //!< (-) Number of times thruster fired
}ThrusterOperationData;

//! @brief Container for overall thruster configuration data for single thruster
/*! This structure is used to define the overall configuration of an entire
 thruster.  It holds the current operational data for the thruster, the
 ramp/max/min configuration data, and the physical location/orientation data for
 a thruster.*/
typedef struct {
    std::vector<double> ThrusterLocation;          //!< m Location of thruster in structural
    std::vector<double> ThrusterDirection;         //!< -- Unit vector of thruster pointing
    std::vector<ThrusterTimePair> ThrusterOnRamp;  //!< -- Percentage of max thrust for ramp up
    std::vector<ThrusterTimePair> ThrusterOffRamp; //!< -- Percentage of max thrust for ramp down
    double MaxThrust;                              //!< N  Steady state thrust of thruster
    double steadyIsp;                              //!< s  Steady state specific impulse of thruster
    double MinOnTime;                              //!< s  Minimum allowable on-time
    ThrusterOperationData ThrustOps;               //!< -- Thruster operating data
}ThrusterConfigData;

//! @brief Input container for thruster firing requests.
/*! This structure is used for the array of thruster commands.  It is pretty
 sparse, but it is included as a structure for growth and for clear I/O
 definitions.*/
typedef struct {
    double OnTimeRequest;                //!< s Requested on-time for thruster
}ThrustCmdStruct;

//! @brief Thruster dynamics class used to provide thruster effects on body
/*! This class is used to hold and operate a set of thrusters that are located
 on the spacecraft.  It contains all of the configuration data for the thruster
 set, reads an array of on-time requests (double precision in seconds).  It is
 intended to be attached to the dynamics plant in the system using the
 DynEffector interface and as such, does not directly write the current force
 or torque into the messaging system.  The nominal interface to dynamics are the
 BodyForce and BodyTorque arrays that are provided by the DynEffector base class.
 There is technically double inheritance here, but both the DynEffector and
 SysModel classes are abstract base classes so there is no risk of diamond.*/
class ThrusterDynamics: public SysModel, public DynEffector {
public:
    ThrusterDynamics();
    ~ThrusterDynamics();
    
    void SelfInit();
    void CrossInit();
    //! Add a new thruster to the thruster set
    void AddThruster(ThrusterConfigData *NewThruster) {ThrusterData.push_back(*NewThruster);}
    void UpdateState(uint64_t CurrentSimNanos);
    void WriteOutputMessages(uint64_t CurrentClock);
    bool ReadInputs();
    void ConfigureThrustRequests(double CurrentTime);
    void ComputeDynamics(MassPropsData *Props, OutputStateData *Bstate,
                         double CurrentTime);
    void ComputeThrusterFire(ThrusterConfigData *CurrentThruster,
                             double CurrentTime);
    void ComputeThrusterShut(ThrusterConfigData *CurrentThruster,
                             double CurrentTime);
    void updateMassProperties(double CurrentTime);
    double thrFactorToTime(ThrusterConfigData *thrData,
        std::vector<ThrusterTimePair> *thrRamp);
    
public:
    std::vector<ThrusterConfigData> ThrusterData;  //!< -- Thruster information
    std::string InputCmds;                         //!< -- message used to read command inputs
    std::string OutputDataString;                  //!< -- port to use for output data
    uint64_t OutputBufferCount;                    //!< -- Count on number of buffers to output
    std::vector<double> NewThrustCmds;             //!< -- Incoming thrust commands
    double StrForce[3];                            //!< N  Computed force in str for thrusters
    double StrTorque[3];                           //!< Nm Computed torque in str for thrusters
    double mDotTotal;                              //!< kg/s Current mass flow rate of thrusters
    double prevFireTime;                           //!< s  Previous thruster firing time
    
private:
    int64_t CmdsInMsgID;                           //!< -- MEssage ID for incoming data
    int64_t StateOutMsgID;                         //!< -- Message ID for outgoing data
    ThrustCmdStruct *IncomingCmdBuffer;            //!< -- One-time allocation for savings
    uint64_t prevCommandTime;                      //!< -- Time for previous valid thruster firing
};

/*! @} */

#endif
