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

#ifndef REACTIONWHEEL_DYNAMICS_H
#define REACTIONWHEEL_DYNAMICS_H

#include <vector>
#include "_GeneralModuleFiles/sys_model.h"
#include "_GeneralModuleFiles/dyn_effector.h"
#include "../ADCSAlgorithms/effectorInterfaces/errorConversion/vehEffectorOut.h"
#include "../ADCSAlgorithms/effectorInterfaces/_GeneralModuleFiles/rwSpeedData.h"
#include "utilities/simMacros.h"

/*! \addtogroup SimModelGroup
 * @{
 */
typedef struct {
    double wheelPositions[MAX_EFF_CNT];                
}RWConfigOutputData;

//! @brief Container for overall RW configuration data for single RW
/*! This structure is used to define the overall configuration of an entire
 RW.  It holds the current operational data for the RW, the
 ramp/max/min configuration data, and the physical location/orientation data for
 a RW.*/
typedef struct {
    std::string typeName;      //!< [], string containing the RW type name
    double r_S[3];             //!< m, position vector of the RW relative to the spacecraft structural frame
    double gsHat_S[3];         //!< spin axis unit vector in structural frame
    double gtHat0_S[3];        //!< initial torque axis unit vector in structural frame
    double ggHat0_S[3];        //!< initial gimbal axis unit vector in structural frame
    double r_B[3];             //!< m, position vector of the RW relative to the spacecraft body frame
    double gsHat_B[3];         //!< spin axis unit vector in body frame
    double gtHat0_B[3];        //!< initial torque axis unit vector in body frame
    double ggHat0_B[3];        //!< initial gimbal axis unit vector in body frame
    double theta;              //!< rad, wheel angle
    double u_current;          //!< N-m, current motor torque
    double u_max;              //!< N-m, Max torque
    double u_min;              //!< N-m, Min torque
    double u_f;                //!< N-m, Coulomb friction torque magnitude
    double Omega;              //!< rad/s, wheel speed
    double Omega_max;          //!< rad/s, max wheel speed
    double Js;                 //!< kg-m^2, spin axis gsHat rotor moment of inertia
    double Jt;                 //!< kg-m^2, gtHat axis rotor moment of inertia
    double Jg;                 //!< kg-m^2, ggHat axis rotor moment of inertia
    double U_s;                //!< kg-m, static imbalance
    double U_d;                //!< kg-m^2, dynamic imbalance
    double mass;               //!< kg, reaction wheel rotor mass
    double F_B[3];             //!< N, single RW force with simple jitter model
    double tau_B[3];           //!< N-m, single RW torque with simple jitter model
    bool usingRWJitter;        //!< flag for using imbalance torques
    double linearFrictionRatio;//!< [%] ratio relative to max speed value up to which the friction behaves linearly
}ReactionWheelConfigData;

//! @brief Input container for thruster firing requests.
/*! This structure is used for the array of RW commands.  It is pretty
 sparse, but it is included as a structure for growth and for clear I/O
 definitions.*/
typedef struct {
    double u_cmd; //!< N-m, torque command for RW
}RWCmdStruct;

//! @brief Thruster dynamics class used to provide thruster effects on body
/*! This class is used to hold and operate a set of thrusters that are located
 on the spacecraft.  It contains all of the configuration data for the thruster
 set, reads an array of on-time requests (double precision in seconds).  It is
 intended to be attached to the dynamics plant in the system using the
 DynEffector interface and as such, does not directly write the current force
 or torque into the messaging system.  The nominal interface to dynamics are the
 dynEffectorForce and dynEffectorTorque arrays that are provided by the 
 DynEffector base class.
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
    std::vector<ReactionWheelConfigData> ReactionWheelData;     //!< -- RW information
    std::string InputCmds;                                      //!< -- message used to read command inputs
    std::string OutputDataString;                               //!< -- port to use for output data
    std::string inputVehProps;                                  //!< [-] Input mass properties of vehicle
    uint64_t OutputBufferCount;                                 //!< -- Count on number of buffers to output
    std::vector<RWCmdStruct> NewRWCmds;                         //!< -- Incoming attitude commands
	RWSpeedData outputStates;                                   //!< (-) Output data from the reaction wheels
    double sumF_B[3];                                           //!< N  Computed jitter force in body frame
    double sumTau_B[3];                                         //!< N-m Computed jitter torque in body frame
    
private:
    std::vector<std::string> rwOutMsgNames;                     //!< -- vector with the message names of each RW
    std::vector<uint64_t> rwOutMsgIds;                          //!< -- vector with the ID of each RW
    int64_t CmdsInMsgID;                                        //!< -- Message ID for incoming data
    int64_t StateOutMsgID;                                      //!< -- Message ID for outgoing data
    RWCmdStruct *IncomingCmdBuffer;                             //!< -- One-time allocation for savings
    uint64_t prevCommandTime;                                   //!< -- Time for previous valid thruster firing
};

/*! @} */

#endif
