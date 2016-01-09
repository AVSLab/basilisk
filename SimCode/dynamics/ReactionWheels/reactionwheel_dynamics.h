
#ifndef REACTIONWHEEL_DYNAMICS_H
#define REACTIONWHEEL_DYNAMICS_H

#include <vector>
#include "utilities/sys_model.h"
#include "utilities/dyn_effector.h"

/*! \addtogroup SimModelGroup
 * @{
 */

//! @brief Container for overall RW configuration data for single RW
/*! This structure is used to define the overall configuration of an entire
 RW.  It holds the current operational data for the RW, the
 ramp/max/min configuration data, and the physical location/orientation data for
 a RW.*/
typedef struct {
 std::vector<double> r_S; //!< m, position vector of the RW relative to the spacecraft structural frame
 std::vector<double> gsHat_S; //!< spin axis unit vector in structural frame
 std::vector<double> gtHat0_S; //!< initial torque axis unit vector in structural frame
 std::vector<double> ggHat0_S; //!< initial gimbal axis unit vector in structural frame
 double u_max; //!< N-m, Max torque
 double u_f; //!< N-m, Coulomb friction torque magnitude
}ReactionWheelConfigData;

//! @brief Input container for thruster firing requests.
/*! This structure is used for the array of RW commands.  It is pretty
 sparse, but it is included as a structure for growth and for clear I/O
 definitions.*/
typedef struct {
 double u_c; //!< N-m, torque command for RW
}RWCmdStruct;

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
//    void ComputeThrusterFire(ThrusterConfigData *CurrentThruster,
//                             double CurrentTime);
//    void ComputeThrusterShut(ThrusterConfigData *CurrentThruster,
//                             double CurrentTime);

public:
    std::vector<ReactionWheelConfigData> ReactionWheelData;  //!< -- Thruster information
    std::string InputCmds;                         //!< -- message used to read command inputs
    std::string OutputDataString;                  //!< -- port to use for output data
    uint64_t OutputBufferCount;                    //!< -- Count on number of buffers to output
    std::vector<RWCmdStruct> NewRWCmds;                 //!< -- Incoming thrust commands
    double F_S[3];                            //!< N  Computed force in str
    double tau_S[3];                           //!< N-m Computed torque in str
    
private:
    int64_t CmdsInMsgID;                           //!< -- MEssage ID for incoming data
    int64_t StateOutMsgID;                         //!< -- Message ID for outgoing data
    RWCmdStruct *IncomingCmdBuffer;            //!< -- One-time allocation for savings
    uint64_t prevCommandTime;                      //!< -- Time for previous valid thruster firing
};

/*! @} */

#endif
