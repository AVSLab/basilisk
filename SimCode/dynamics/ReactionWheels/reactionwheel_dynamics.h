
#ifndef REACTIONWHEEL_DYNAMICS_H
#define REACTIONWHEEL_DYNAMICS_H

#include <vector>
#include "utilities/sys_model.h"
#include "utilities/dyn_effector.h"
#include "../ADCSAlgorithms/effectorInterfaces/errorConversion/vehEffectorOut.h"

/*! \addtogroup SimModelGroup
 * @{
 */

//! @brief Container for overall thruster configuration data for single thruster
/*! This structure is used to define the overall configuration of an entire
 thruster.  It holds the current operational data for the thruster, the
 ramp/max/min configuration data, and the physical location/orientation data for
 a thruster.*/
typedef struct {
	std::vector<double> ReactionWheelLocation;          //!< m Location of thruster in structural
	std::vector<double> ReactionWheelDirection;         //!< -- Unit vector of thruster pointing
	double MaxTorque;                              //!< N  Steady state thrust of thruster
	double currentTorque;                          //!< Nm Current reaction wheel motor torque
	double rwOmega;                                //!< r/s Current Reaction wheel angular velocity
	double wheelAngle;                             //!< r The current angle of the reaction wheel rwt zero
	double Js;                                     //!< kgm2 The inertia value about the RW spin axis
}ReactionWheelConfigData;

//! @brief Input container for thruster firing requests.
/*! This structure is used for the array of thruster commands.  It is pretty
 sparse, but it is included as a structure for growth and for clear I/O
 definitions.*/
typedef struct {
 double TorqueRequest;                //!< s Requested on-time for thruster
}RWCmdStruct;

typedef struct {
	double wheelSpeeds[MAX_NUM_EFFECTORS];                //!< r/s The current angular velocity of the wheel
}RWOutputData;

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
class ReactionWheelDynamics: public SysModel, public DynEffector {
public:
    ReactionWheelDynamics();
    ~ReactionWheelDynamics();
    
    void SelfInit();
    void CrossInit();
    //! Add a new thruster to the thruster set
    void AddReactionWheel(ReactionWheelConfigData *NewRW) {ReactionWheelData.push_back(*NewRW);}
    void UpdateState(uint64_t CurrentSimNanos);
    void WriteOutputMessages(uint64_t CurrentClock);
    void ReadInputs();
    void ConfigureRWRequests(double CurrentTime);
    void ComputeDynamics(MassPropsData *Props, OutputStateData *Bstate,
                         double CurrentTime);

public:
    std::vector<ReactionWheelConfigData> ReactionWheelData;  //!< -- Thruster information
    std::string InputCmds;                         //!< -- message used to read command inputs
    std::string OutputDataString;                  //!< -- port to use for output data
    uint64_t OutputBufferCount;                    //!< -- Count on number of buffers to output
    std::vector<RWCmdStruct> NewRWCmds;                 //!< -- Incoming thrust commands
    double StrForce[3];                            //!< N  Computed force in str for thrusters
    double StrTorque[3];                           //!< Nm Computed torque in str for thrusters
	RWOutputData outputStates;  //!< (-) Output data from the reaction wheels
    
private:
    int64_t CmdsInMsgID;                           //!< -- MEssage ID for incoming data
    int64_t StateOutMsgID;                         //!< -- Message ID for outgoing data
    RWCmdStruct *IncomingCmdBuffer;            //!< -- One-time allocation for savings
    uint64_t prevCommandTime;                      //!< -- Time for previous valid thruster firing
};

/*! @} */

#endif
