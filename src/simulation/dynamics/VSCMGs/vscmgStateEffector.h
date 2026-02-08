/*
 ISC License

 Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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


#ifndef VSCMGSTATEEFFECTOR_H
#define VSCMGSTATEEFFECTOR_H

#include "../_GeneralModuleFiles/stateEffector.h"
#include "../_GeneralModuleFiles/dynParamManager.h"
#include "../_GeneralModuleFiles/dynamicEffector.h"
#include "../_GeneralModuleFiles/dynamicObject.h"
#include <Eigen/Dense>
#include "architecture/_GeneralModuleFiles/sys_model.h"

#include "architecture/msgPayloadDefC/VSCMGSpeedMsgPayload.h"
#include "architecture/msgPayloadDefC/VSCMGCmdMsgPayload.h"
#include "architecture/msgPayloadDefC/VSCMGArrayTorqueMsgPayload.h"
#include "architecture/msgPayloadDefCpp/VSCMGConfigMsgPayload.h"
#include "architecture/messaging/messaging.h"

#include "architecture/utilities/macroDefinitions.h"
#include "architecture/utilities/bskLogging.h"
#include "architecture/utilities/avsEigenMRP.h"
#include "architecture/utilities/avsEigenSupport.h"



/*! @brief VSCMG state effector class */
class VSCMGStateEffector:  public SysModel, public StateEffector {
public:
    VSCMGStateEffector();
	~VSCMGStateEffector();
	void registerStates(DynParamManager& states);
	void linkInStates(DynParamManager& states);
    void updateEffectorMassProps(double integTime);
    void Reset(uint64_t CurrentSimNanos);
    void AddVSCMG(VSCMGConfigMsgPayload *NewVSCMG);
	void UpdateState(uint64_t CurrentSimNanos);
	void WriteOutputMessages(uint64_t CurrentClock);
	void ReadInputs();
	void ConfigureVSCMGRequests(double CurrentTime);
    void updateContributions(double integTime, BackSubMatrices & backSubContr, Eigen::Vector3d sigma_BN, Eigen::Vector3d omega_BN_B, Eigen::Vector3d g_N);  //!< [-] Back-sub contributions
    void updateEnergyMomContributions(double integTime, Eigen::Vector3d & rotAngMomPntCContr_B,
                                              double & rotEnergyContr, Eigen::Vector3d omega_BN_B);  //!< [-] Energy and momentum calculations
    void computeDerivatives(double integTime, Eigen::Vector3d rDDot_BN_N, Eigen::Vector3d omegaDot_BN_B, Eigen::Vector3d sigma_BN);  //!< [-] Method for each stateEffector to calculate derivatives

public:
	std::vector<VSCMGConfigMsgPayload> VSCMGData; //!< [-] VSCMG data structure
    Eigen::MatrixXd *g_N; 						//!< [m/s^2] Gravitational acceleration in N frame components

    ReadFunctor<VSCMGArrayTorqueMsgPayload> cmdsInMsg;  //!< [-] motor torque command input message
	Message<VSCMGSpeedMsgPayload> speedOutMsg; 	        //!< [-] VSCMG speed output message
    std::vector<Message<VSCMGConfigMsgPayload>*> vscmgOutMsgs;   //!< [-] vector of VSCMG output messages

    std::vector<VSCMGCmdMsgPayload> newVSCMGCmds; 	//!< [-] Incoming torque commands
	VSCMGSpeedMsgPayload outputStates; 				//!< [-] Output data from the VSCMGs
    std::string nameOfVSCMGOmegasState;         //!< [-] identifier for the Omegas state data container
    std::string nameOfVSCMGThetasState;         //!< [-] identifier for the Thetas state data container
	std::string nameOfVSCMGGammasState;         //!< [-] identifier for the Gammas state data container
	std::string nameOfVSCMGGammaDotsState;      //!< [-] identifier for the GammaDots state data container
	int numVSCMG;                               //!< [-] number of VSCMGs
	int numVSCMGJitter;                         //!< [-] number of VSCMGs with jitter
  BSKLogger bskLogger;                      //!< [-] BSK Logging

private:
    VSCMGArrayTorqueMsgPayload incomingCmdBuffer; 	//!< [-] One-time allocation for savings
	uint64_t prevCommandTime;                  	//!< [-] Time for previous valid thruster firing

	StateData *hubOmega;                        //!< [rad/s] angular velocity of the B frame wrt the N frame in B frame components
	StateData *OmegasState;                     //!< [rad/s] RW spin state
	StateData *thetasState;                     //!< [rad] RW angle
	StateData *gammasState;                     //!< [rad] CMG gimbal angle
	StateData *gammaDotsState;                  //!< [rad/s] CMG gimbal angle rate

};


#endif /* STATE_EFFECTOR_H */
