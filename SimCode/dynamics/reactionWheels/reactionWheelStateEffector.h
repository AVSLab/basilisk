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


#ifndef REACTIONWHEELSTATEEFFECTOR_H
#define REACTIONWHEELSTATEEFFECTOR_H

#include "../_GeneralModuleFiles/stateEffector.h"
#include "../_GeneralModuleFiles/dynParamManager.h"
#include "../_GeneralModuleFiles/dynamicEffector.h"
#include "../_GeneralModuleFiles/dynObject2.h"
#include <Eigen/Dense>
#include "utilities/simMacros.h"
#include "_GeneralModuleFiles/sys_model.h"
#include "_GeneralModuleFiles/dyn_effector.h"
#include "../ADCSAlgorithms/effectorInterfaces/errorConversion/vehEffectorOut.h"
#include "../ADCSAlgorithms/effectorInterfaces/_GeneralModuleFiles/rwSpeedData.h"

/*! @brief Abstract class that is used to implement an effector impacting a dynamic body 
           that does not itself maintain a state or represent a changing component of
           the body (for example: gravity, thrusters, solar radiation pressure, etc.)
 */

typedef struct {
	double wheelPositions[MAX_EFF_CNT];
}RWConfigOutputData;

typedef struct {
	double u_cmd; //!< N-m, torque command for RW
}RWCmdStruct;

enum RWModels { BalancedWheels, JitterSimple, JitterFullyCoupled };

typedef struct {
	std::string typeName;      //!< [], string containing the RW type name
	Eigen::Vector3d rWB_S;		//!< m, position vector of the RW relative to the spacecraft structural frame
	Eigen::Vector3d gsHat_S;	//!< spin axis unit vector in structural frame
	Eigen::Vector3d gtHat0_S;	//!< initial torque axis unit vector in structural
	Eigen::Vector3d ggHat0_S;	//!< initial gimbal axis unit vector in structural frame
	Eigen::Vector3d rWB_B;		//!< m, position vector of the RW relative to the spacecraft body frame
	Eigen::Vector3d gsHat_B;	//!< spin axis unit vector in body frame
	Eigen::Vector3d gtHat0_B;	//!< initial torque axis unit vector in body frame
	Eigen::Vector3d ggHat0_B;	//!< initial gimbal axis unit vector in body frame
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
	Eigen::Vector3d F_B;             //!< N, single RW force with simple jitter model
	Eigen::Vector3d tau_B;           //!< N-m, single RW torque with simple jitter model
	double linearFrictionRatio;//!< [%] ratio relative to max speed value up to which the friction behaves linearly
	RWModels RWModel;
}ReactionWheelConfigData;


class ReactionWheelStateEffector:  public SysModel, public StateEffector {
public:
    EffectorMassProps effProps;
	int numRW;
	int numRWJitter;
    
public:
    ReactionWheelStateEffector();
	~ReactionWheelStateEffector();
	void registerStates(DynParamManager& states);
	void linkInStates(DynParamManager& states);
//    virtual void updateBackSubstitution(double integTime)=0;
	void updateContributions(double integTime, Eigen::Matrix3d & matrixAcontr, Eigen::Matrix3d & matrixBcontr, Eigen::Matrix3d & matrixCcontr, Eigen::Matrix3d & matrixDcontr, Eigen::Vector3d & vecTranscontr, Eigen::Vector3d & vecRotcontr);
    void computeDerivatives(double integTime);
    void updateEffectorMassProps(double integTime);
    void updateEffectorMassPropRates(double integTime);

public:


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
	Eigen::Vector3d sumF_B;                                           //!< N  Computed jitter force in body frame
	Eigen::Vector3d sumTau_B;                                         //!< N-m Computed jitter torque in body frame
    std::string nameOfReactionWheelOmegasState;
    std::string nameOfReactionWheelThetasState;

private:
	std::vector<std::string> rwOutMsgNames;                     //!< -- vector with the message names of each RW
	std::vector<uint64_t> rwOutMsgIds;                          //!< -- vector with the ID of each RW
	int64_t CmdsInMsgID;                                        //!< -- Message ID for incoming data
	int64_t StateOutMsgID;                                      //!< -- Message ID for outgoing data
	RWCmdStruct *IncomingCmdBuffer;                             //!< -- One-time allocation for savings
	uint64_t prevCommandTime;                                   //!< -- Time for previous valid thruster firing

	StateData *hubSigma;
	StateData *hubOmega;
	StateData *OmegasState;
	StateData *thetasState;

};

#endif /* STATE_EFFECTOR_H */
