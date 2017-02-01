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


#ifndef REACTIONWHEELSTATEEFFECTOR_H
#define REACTIONWHEELSTATEEFFECTOR_H

#include "../_GeneralModuleFiles/stateEffector.h"
#include "../_GeneralModuleFiles/dynParamManager.h"
#include "../_GeneralModuleFiles/dynamicEffector.h"
#include "../_GeneralModuleFiles/dynamicObject.h"
#include <Eigen/Dense>
#include "utilities/simMacros.h"
#include "_GeneralModuleFiles/sys_model.h"
#include "../ADCSAlgorithms/effectorInterfaces/_GeneralModuleFiles/vehEffectorOut.h"
#include "../ADCSAlgorithms/effectorInterfaces/_GeneralModuleFiles/rwSpeedData.h"
#include "../SimCode/utilities/avsEigenMRP.h"
#include "../SimCode/utilities/avsEigenSupport.h"

/*! @brief Abstract class that is used to implement an effector impacting a dynamic body 
           that does not itself maintain a state or represent a changing component of
           the body (for example: gravity, thrusters, solar radiation pressure, etc.)
 */

typedef struct {
	double wheelPositions[MAX_EFF_CNT];
}RWConfigOutputData;

typedef struct {
	double u_cmd; //!< [N-m], torque command for RW
}RWCmdStruct;

enum RWModels { BalancedWheels, JitterSimple, JitterFullyCoupled };

typedef struct {
	std::string typeName;      //!< [-], string containing the RW type name
	Eigen::Vector3d rWB_S;		//!< [m], position vector of the RW relative to the spacecraft structural frame
	Eigen::Vector3d gsHat_S;	//!< [-] spin axis unit vector in structural frame
	Eigen::Vector3d w2Hat0_S;	//!< [-] initial torque axis unit vector in structural
	Eigen::Vector3d w3Hat0_S;	//!< [-] initial gimbal axis unit vector in structural frame
	Eigen::Vector3d rWB_B;		//!< [m], position vector of the RW relative to the spacecraft body frame
	Eigen::Vector3d gsHat_B;	//!< [-] spin axis unit vector in body frame
	Eigen::Vector3d w2Hat0_B;	//!< [-] initial torque axis unit vector in body frame
	Eigen::Vector3d w3Hat0_B;	//!< [-] initial gimbal axis unit vector in body frame
    double mass;               //!< [kg], reaction wheel rotor mass
	double theta;              //!< [rad], wheel angle
	double Omega;              //!< [rad/s], wheel speed
    double Js;                 //!< [kg-m^2], spin axis gsHat rotor moment of inertia
    double Jt;                 //!< [kg-m^2], gtHat axis rotor moment of inertia
    double Jg;                 //!< [kg-m^2], ggHat axis rotor moment of inertia
    double U_s;                //!< [kg-m], static imbalance
    double U_d;                //!< [kg-m^2], dynamic imbalance
    double d;                	//!< [m], wheel center of mass offset from wheel frame origin
    double J13;                	//!< [kg-m^2], x-z inertia of wheel about wheel center in wheel frame (imbalance)
	double u_current;          //!< [N-m], current motor torque
	double u_max;              //!< [N-m], Max torque
	double u_min;              //!< [N-m], Min torque
	double u_f;                //!< [N-m], Coulomb friction torque magnitude
	double Omega_max;          //!< [rad/s], max wheel speed
	double linearFrictionRatio;//!< [%] ratio relative to max speed value up to which the friction behaves linearly
	RWModels RWModel; //!< [-], Type of imbalance model to use
	Eigen::Vector3d aOmega; //!< [-], parameter used in coupled jitter back substitution
	Eigen::Vector3d bOmega; //!< [-], parameter used in coupled jitter back substitution
	double cOmega; //!< [-], parameter used in coupled jitter back substitution
	Eigen::Matrix3d IRWPntWc_B;
	Eigen::Matrix3d IPrimeRWPntWc_B;
	Eigen::Vector3d rWcB_B;
	Eigen::Matrix3d rTildeWcB_B;
	Eigen::Vector3d rPrimeWcB_B;
	Eigen::Vector3d w2Hat_B;
	Eigen::Vector3d w3Hat_B;
}ReactionWheelConfigData;


class ReactionWheelStateEffector:  public SysModel, public StateEffector {
public:
    ReactionWheelStateEffector();
	~ReactionWheelStateEffector();
	void registerStates(DynParamManager& states);
	void linkInStates(DynParamManager& states);
	void updateContributions(double integTime, Eigen::Matrix3d & matrixAcontr, Eigen::Matrix3d & matrixBcontr, Eigen::Matrix3d & matrixCcontr, Eigen::Matrix3d & matrixDcontr, Eigen::Vector3d & vecTranscontr, Eigen::Vector3d & vecRotcontr);
    void computeDerivatives(double integTime);
    void updateEffectorMassProps(double integTime);
    void updateEnergyMomContributions(double integTime, Eigen::Vector3d & rotAngMomPntCContr_B, double & rotEnergyContr);
	void SelfInit();
	void CrossInit();
	void AddReactionWheel(ReactionWheelConfigData *NewRW) {ReactionWheelData.push_back(*NewRW);}
	void UpdateState(uint64_t CurrentSimNanos);
	void WriteOutputMessages(uint64_t CurrentClock);
	void ReadInputs();
	void ConfigureRWRequests(double CurrentTime);
    
public:
	std::vector<ReactionWheelConfigData> ReactionWheelData;     //!< -- RW information
    Eigen::MatrixXd *g_N;           //!< [m/s^2] Gravitational acceleration in N frame components
	std::string InputCmds;                                      //!< -- message used to read command inputs
	std::string OutputDataString;                               //!< -- port to use for output data
    uint64_t OutputBufferCount;                                 //!< -- Count on number of buffers to output
	std::vector<RWCmdStruct> NewRWCmds;                         //!< -- Incoming attitude commands
	RWSpeedData outputStates;                                   //!< (-) Output data from the reaction wheels
    std::string nameOfReactionWheelOmegasState;
    std::string nameOfReactionWheelThetasState;
	int numRW;
	int numRWJitter;

private:
	std::vector<std::string> rwOutMsgNames;                     //!< -- vector with the message names of each RW
	std::vector<uint64_t> rwOutMsgIds;                          //!< -- vector with the ID of each RW
	int64_t CmdsInMsgID;                                        //!< -- Message ID for incoming data
	int64_t StateOutMsgID;                                      //!< -- Message ID for outgoing data
	RWCmdStruct *IncomingCmdBuffer;                             //!< -- One-time allocation for savings
	uint64_t prevCommandTime;                                   //!< -- Time for previous valid thruster firing

	StateData *hubSigma;
	StateData *hubOmega;
	StateData *hubVelocity;
	StateData *OmegasState;
	StateData *thetasState;

};

#endif /* STATE_EFFECTOR_H */
