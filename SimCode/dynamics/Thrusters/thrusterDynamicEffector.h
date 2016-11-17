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


#ifndef THRUSTER_DYNAMIC_EFFECTOR_H
#define THRUSTER_DYNAMIC_EFFECTOR_H

#include "../_GeneralModuleFiles/dynamicEffector.h"
#include "../_GeneralModuleFiles/stateData.h"
#include "_GeneralModuleFiles/sys_model.h"
#include <Eigen/Dense>
#include <vector>



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
    double totalOnTime;                  //!< s  Total amount of time thruster has fired
    uint64_t fireCounter;                //!< (-) Number of times thruster fired
}ThrusterOperationData;

//! @brief Container for overall thruster configuration data for single thruster
/*! This structure is used to define the overall configuration of an entire
 thruster.  It holds the current operational data for the thruster, the
 ramp/max/min configuration data, and the physical location/orientation data for
 a thruster.*/
typedef struct {
    std::string typeName;                           //!< [], string containing the thruster type name
    Eigen::Vector3d inputThrLoc_S;                        //!< m Location of thruster in structural
    Eigen::Vector3d inputThrDir_S;                        //!< -- Unit vector of thruster pointing
    Eigen::Vector3d thrLoc_B;                             //!< [m] Thruster location expressed in body
    Eigen::Vector3d thrDir_B;                             //!< [-] Thruster direction unit vector in body
    std::vector<ThrusterTimePair> ThrusterOnRamp;   //!< -- Percentage of max thrust for ramp up
    std::vector<ThrusterTimePair> ThrusterOffRamp;  //!< -- Percentage of max thrust for ramp down
    double MaxThrust;                               //!< N  Steady state thrust of thruster
    double steadyIsp;                               //!< s  Steady state specific impulse of thruster
    double MinOnTime;                               //!< s  Minimum allowable on-time
    ThrusterOperationData ThrustOps;                //!< -- Thruster operating data
    double thrusterMagDisp;                         //!< -- Percentage of magnitude dispersion
    std::vector<double> thrusterDirectionDisp;      //!< -- Unit vector of dispersed thruster pointing
}ThrusterConfigData;

/*! This structure is used in the messaging system to communicate what the
 state of the vehicle is currently.*/
typedef struct {
    Eigen::Vector3d thrusterLocation;                     //!< m  Current position vector (inertial)
    Eigen::Vector3d thrusterDirection;                    //!< -- Unit vector of thruster pointing
    double maxThrust;                               //!< N  Steady state thrust of thruster
    double thrustFactor;                            //!< -- Current Thrust Percentage
}ThrusterOutputData;

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
 dynEffectorForce and dynEffectorTorque arrays that are provided by the DynEffector base class.
 There is technically double inheritance here, but both the DynEffector and
 SysModel classes are abstract base classes so there is no risk of diamond.*/
class ThrusterDynamicEffector: public SysModel, public DynamicEffector {
public:
    ThrusterDynamicEffector();
    ~ThrusterDynamicEffector();
    void linkInStates(DynParamManager& states);
    void computeBodyForceTorque(double integTime);
    
    void SelfInit();
    void CrossInit();
    //! Add a new thruster to the thruster set
    void AddThruster(ThrusterConfigData *NewThruster) {ThrusterData.push_back(*NewThruster);}
    void UpdateState(uint64_t CurrentSimNanos);
    void WriteOutputMessages(uint64_t CurrentClock);
    bool ReadInputs();
    void ConfigureThrustRequests(double currentTime);
    void ComputeThrusterFire(ThrusterConfigData *CurrentThruster,
                             double currentTime);
    void ComputeThrusterShut(ThrusterConfigData *CurrentThruster,
                             double currentTime);
    void updateMassProperties(double currentTime);
    

public:
    int stepsInRamp;
    std::vector<ThrusterConfigData> ThrusterData;  //!< -- Thruster information
    std::string InputCmds;                         //!< -- message used to read command inputs
    std::string inputProperties;                   //!< [-] The mass properties of the spacecraft
    std::string inputBSName;                       //!< [-] Structure to body dynamic property
    uint64_t thrusterOutMsgNameBufferCount;        //!< -- Count on number of buffers to output
    std::vector<std::string> thrusterOutMsgNames;                //!< -- Message name for all thruster data
    std::vector<double> NewThrustCmds;             //!< -- Incoming thrust commands
    double mDotTotal;                              //!< kg/s Current mass flow rate of thrusters
    double prevFireTime;                           //!< s  Previous thruster firing time
    double thrFactorToTime(ThrusterConfigData *thrData,
                           std::vector<ThrusterTimePair> *thrRamp);
    StateData *hubSigma;
    Eigen::Vector3d forceExternal_B;      //! [-] External force applied by this effector
    Eigen::Vector3d torqueExternalPntB_B; //! [-] External torque applied by this effector
    
private:
    //    bool bdyFrmReady;                              //!< [-] Flag indicating that the body frame is ready
    Eigen::MatrixXd *dcm_BS;               //!< [kg] spacecrafts total mass
    std::vector<uint64_t> thrusterOutMsgIds;                      //!< -- Message ID of each thruster
    std::vector<ThrusterOutputData> thrusterOutBuffer; //!< -- Message buffer for thruster data
    int64_t CmdsInMsgID;                            //!< -- Message ID for incoming data
    //int64_t propsInID;                              //!< [-] The ID associated with the mss props msg
    ThrustCmdStruct *IncomingCmdBuffer;             //!< -- One-time allocation for savings
    uint64_t prevCommandTime;                       //!< -- Time for previous valid thruster firing

};

#endif /* THRUSTER_DYNAMIC_EFFECTOR_H */
