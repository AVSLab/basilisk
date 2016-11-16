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


#include "reactionWheelStateEffector.h"
#include "architecture/messaging/system_messaging.h"
#include "utilities/linearAlgebra.h"
#include <cstring>
#include <iostream>
#include <cmath>

ReactionWheelStateEffector::ReactionWheelStateEffector()
{
	CallCounts = 0;
	InputCmds = "reactionwheel_cmds";
	OutputDataString = "reactionwheel_output_states";
	OutputBufferCount = 2;
	CmdsInMsgID = -1;
	StateOutMsgID = -1;
	IncomingCmdBuffer = NULL;
	prevCommandTime = 0xFFFFFFFFFFFFFFFF;
	this->sumF_B.setZero();
	this->sumTau_B.setZero();

    effProps.IEffPntB_B.fill(0.0);
    effProps.rCB_B.fill(0.0);
    effProps.mEff = 0.0;
	effProps.IEffPrimePntB_B.fill(0.0);
	effProps.rPrimeCB_B.fill(0.0);

    this->nameOfReactionWheelOmegasState = "reactionWheelOmegas";
    this->nameOfReactionWheelThetasState = "reactionWheelThetas";
    
    return;
}


ReactionWheelStateEffector::~ReactionWheelStateEffector()
{
    return;
}

void ReactionWheelStateEffector::linkInStates(DynParamManager& statesIn)
{
	this->hubSigma = statesIn.getStateObject("hubSigma");
	this->hubOmega = statesIn.getStateObject("hubOmega");
}

void ReactionWheelStateEffector::registerStates(DynParamManager& states)
{
	this->numRWJitter = 0;
	this->numRW = 0;
	std::vector<ReactionWheelConfigData>::iterator RWIt;

	for(RWIt=ReactionWheelData.begin(); RWIt!=ReactionWheelData.end(); RWIt++) {
		if (RWIt->RWModel == JitterSimple || RWIt->RWModel == JitterFullyCoupled) {
			this->numRWJitter++;
		}
		this->numRW++;
	}

	this->OmegasState = states.registerState(this->numRW, 1, this->nameOfReactionWheelOmegasState);

	if (numRWJitter > 0) {
		this->thetasState = states.registerState(this->numRWJitter, 1, this->nameOfReactionWheelThetasState);
	}

}

void ReactionWheelStateEffector::updateEffectorMassProps(double integTime)
{
	return;
}

void ReactionWheelStateEffector::updateContributions(double integTime, Eigen::Matrix3d & matrixAcontr, Eigen::Matrix3d & matrixBcontr, Eigen::Matrix3d & matrixCcontr, Eigen::Matrix3d & matrixDcontr, Eigen::Vector3d & vecTranscontr, Eigen::Vector3d & vecRotcontr)
{
	Eigen::Vector3d omegaBNLoc_B;
	Eigen::MatrixXd OmegasLoc;
	int RWi = 0;
	std::vector<ReactionWheelConfigData>::iterator RWIt;

	omegaBNLoc_B = hubOmega->getState();
	OmegasLoc = OmegasState->getState();

	matrixAcontr.setZero();
	matrixBcontr.setZero();
	matrixCcontr.setZero();
	vecTranscontr.setZero();

	for(RWIt=ReactionWheelData.begin(); RWIt!=ReactionWheelData.end(); RWIt++)
	{
		matrixDcontr += RWIt->Js * RWIt->gsHat_B * RWIt->gsHat_B.transpose();
		vecRotcontr -= RWIt->gsHat_B * RWIt->u_current + RWIt->Js*OmegasLoc(RWi,1)*omegaBNLoc_B.cross(RWIt->gsHat_B);
		RWi++;
	}
	return;
}

void ReactionWheelStateEffector::computeDerivatives(double integTime)
{
	Eigen::MatrixXd OmegasDot(this->numRW,1);
	Eigen::Vector3d omegaDotBNLoc_B;
	int RWi = 0;
	std::vector<ReactionWheelConfigData>::iterator RWIt;

	//! Grab necessarry values from manager
	omegaDotBNLoc_B = this->hubOmega->getStateDeriv();

	//! - Compute Derivatives
	for(RWIt=ReactionWheelData.begin(); RWIt!=ReactionWheelData.end(); RWIt++)
	{
		OmegasDot(RWi,1) = RWIt->u_current/RWIt->Js - RWIt->gsHat_B.transpose()*omegaDotBNLoc_B;
		RWi++;
	}

	OmegasState->setDerivative(OmegasDot);
}






/*! This method is used to clear out the current RW states and make sure
 that the overall model is ready
 @return void
 */
void ReactionWheelStateEffector::SelfInit()
{
	SystemMessaging *messageSys = SystemMessaging::GetInstance();
	RWCmdStruct RWCmdInitializer;
	RWCmdInitializer.u_cmd = 0.0;

	//! Begin method steps
	//! - Clear out any currently firing RWs and re-init cmd array
	NewRWCmds.clear();
	NewRWCmds.insert(NewRWCmds.begin(), ReactionWheelData.size(), RWCmdInitializer );
	//! - Clear out the incoming command buffer and resize to max RWs
	if(IncomingCmdBuffer != NULL)
	{
		delete [] IncomingCmdBuffer;
	}
	IncomingCmdBuffer = new RWCmdStruct[ReactionWheelData.size()];

	// Reserve a message ID for each reaction wheel config output message
	uint64_t tmpWheeltMsgId;
	std::string tmpWheelMsgName;
	std::vector<ReactionWheelConfigData>::iterator it;
	for (it = ReactionWheelData.begin(); it != ReactionWheelData.end(); it++)
	{
		tmpWheelMsgName = "rw_" + std::to_string(it - ReactionWheelData.begin()) + "_data";
		tmpWheeltMsgId = messageSys->CreateNewMessage(tmpWheelMsgName, sizeof(ReactionWheelConfigData), OutputBufferCount, "ReactionWheelConfigData", moduleID);
		this->rwOutMsgNames.push_back(tmpWheelMsgName);
		this->rwOutMsgIds.push_back(tmpWheeltMsgId);
	}

	StateOutMsgID = messageSys->CreateNewMessage(OutputDataString, sizeof(RWSpeedData),
												 OutputBufferCount, "RWSpeedData", moduleID);
}

/*! This method is used to connect the input command message to the RWs.
 It sets the message ID based on what it finds for the input string.  If the
 message is not successfully linked, it will warn the user.
 @return void
 */
void ReactionWheelStateEffector::CrossInit()
{

	int64_t propsID;
	SingleMessageHeader localHeader;
	MassPropsData localProps;
	Eigen::Matrix3d T_str2Bdy;

	// copy T_str2bdy into eigen
	for (int i=0; i<3; i++) {
		for (int j=0; j<3; j++) {
			T_str2Bdy(i,j) = localProps.T_str2Bdy[3*i+j];
		}
	}


	//! Begin method steps
	//! - Find the message ID associated with the InputCmds string.
	//! - Warn the user if the message is not successfully linked.
	CmdsInMsgID = SystemMessaging::GetInstance()->subscribeToMessage(InputCmds,
																	 sizeof(RWCmdStruct)*MAX_EFF_CNT, moduleID);
	if(CmdsInMsgID < 0)
	{
		std::cerr << "WARNING: Did not find a valid message with name: ";
		std::cerr << InputCmds << "  :" << std::endl<< __FILE__ << std::endl;
	}

	propsID = SystemMessaging::GetInstance()->subscribeToMessage(rwVehPropsInMsgName,
																 sizeof(MassPropsData), moduleID);
	SystemMessaging::GetInstance()->ReadMessage(propsID, &localHeader,
												sizeof(MassPropsData), reinterpret_cast<uint8_t*>(&localProps));
	std::vector<ReactionWheelConfigData>::iterator it;
	for (it = ReactionWheelData.begin(); it != ReactionWheelData.end(); it++)
	{
		if (it->gsHat_S.norm() > 0.01) {
			it->gsHat_B = T_str2Bdy * it->gsHat_S;
		} else {
			std::cerr <<
			"Error: gsHat_S not properly initialized.  Don't set gsHat_B directly in python.";
		}
		if (it->RWModel == JitterSimple || it->RWModel == JitterFullyCoupled) {
			if (it->gtHat0_S.norm() > 0.01) {
				it->gtHat0_B = T_str2Bdy * it->gtHat0_S;
			} else {
				std::cerr << "Error: gtHat0_S not properly initialized.  Don't set gtHat0_B directly in python.";
			}
			if (it->ggHat0_S.norm() > 0.01) {
				it->ggHat0_B = T_str2Bdy * it->ggHat0_S;
			} else {
				std::cerr << "Error: ggHat0_S not properly initialized.  Don't set ggHat0_S directly in python.";
			}
		}
		it->rWB_B = T_str2Bdy * it->rWB_S;
	}
}

/*! This method is here to write the output message structure into the specified
 message.  It is currently blank but we will certainly have an output message
 soon.  If it is already here, bludgeon whoever added it and didn't fix the
 comment.
 @param CurrentClock The current time used for time-stamping the message
 @return void
 */
void ReactionWheelStateEffector::WriteOutputMessages(uint64_t CurrentClock)
{
	SystemMessaging *messageSys = SystemMessaging::GetInstance();
	ReactionWheelConfigData tmpRW;
	std::vector<ReactionWheelConfigData>::iterator it;
	for (it = ReactionWheelData.begin(); it != ReactionWheelData.end(); it++)
	{
		outputStates.wheelSpeeds[it - ReactionWheelData.begin()] = it->Omega;

		tmpRW.rWB_S = it->rWB_S;
		tmpRW.gsHat_S = it->gsHat_S;
		tmpRW.gtHat0_S = it->gtHat0_S;
		tmpRW.ggHat0_S = it->ggHat0_S;
		tmpRW.theta = it->theta;
		tmpRW.u_current = it->u_current;
		tmpRW.u_max = it->u_max;
		tmpRW.u_min = it->u_min;
		tmpRW.u_f = it->u_f;
		tmpRW.Omega = it->Omega;
		tmpRW.Omega_max = it->Omega_max;
		tmpRW.Js = it->Js;
		tmpRW.U_s = it->U_s;
		tmpRW.U_d = it->U_d;
		tmpRW.RWModel = it->RWModel;
		// Write out config data for eachreaction wheel
		messageSys->WriteMessage(this->rwOutMsgIds.at(it - ReactionWheelData.begin()),
								 CurrentClock,
								 sizeof(ReactionWheelConfigData),
								 reinterpret_cast<uint8_t*> (&tmpRW),
								 moduleID);
	}

	// Write this message once for all reaction wheels
	messageSys->WriteMessage(StateOutMsgID, CurrentClock,
							 sizeof(RWSpeedData), reinterpret_cast<uint8_t*> (&outputStates), moduleID);
}

/*! This method is used to read the incoming command message and set the
 associated command structure for operating the RWs.
 @return void
 */
void ReactionWheelStateEffector::ReadInputs()
{
//
	std::vector<double>::iterator CmdIt;
	uint64_t i;
	//! Begin method steps
	//! - If the input message ID is invalid, return without touching states
	if(CmdsInMsgID < 0)
	{
		return;
	}

	//! - Zero the command buffer and read the incoming command array
	SingleMessageHeader LocalHeader;
	memset(IncomingCmdBuffer, 0x0, ReactionWheelData.size()*sizeof(RWCmdStruct));
	SystemMessaging::GetInstance()->ReadMessage(CmdsInMsgID, &LocalHeader,
												ReactionWheelData.size()*sizeof(RWCmdStruct),
												reinterpret_cast<uint8_t*> (IncomingCmdBuffer), moduleID);

	//! - Check if message has already been read, if stale return
	//    if(prevCommandTime==LocalHeader.WriteClockNanos) {
	//        return;
	//    }
	prevCommandTime = LocalHeader.WriteClockNanos;

	//! - Set the NewRWCmds vector.  Using the data() method for raw speed
	RWCmdStruct *CmdPtr;
	for(i=0, CmdPtr = NewRWCmds.data(); i<ReactionWheelData.size();
		CmdPtr++, i++)
	{
		CmdPtr->u_cmd = IncomingCmdBuffer[i].u_cmd;
	}

}

///*! This method is used to read the new commands vector and set the RW
// firings appropriately.  It assumes that the ReadInputs method has already been
// run successfully.
// @return void
// @param CurrentTime The current simulation time converted to a double
// */
void ReactionWheelStateEffector::ConfigureRWRequests(double CurrentTime)
{
	//! Begin method steps
	std::vector<RWCmdStruct>::iterator CmdIt;
	int RWIter = 0;
	double u_s;
	double cosTheta;
	double sinTheta;
	Eigen::Vector3d gtHat_B;
	Eigen::Vector3d temp1;
	Eigen::Vector3d temp2;
	double omegaCritical;

	// zero the sum vectors of RW jitter torque and force
	this->sumTau_B.setZero();
	this->sumF_B.setZero();

	// loop through commands
	for(CmdIt=NewRWCmds.begin(); CmdIt!=NewRWCmds.end(); CmdIt++)
	{
		// saturation
		if (this->ReactionWheelData[RWIter].u_max > 0) {
			if(CmdIt->u_cmd > this->ReactionWheelData[RWIter].u_max) {
				CmdIt->u_cmd = this->ReactionWheelData[RWIter].u_max;
			} else if(CmdIt->u_cmd < -this->ReactionWheelData[RWIter].u_max) {
				CmdIt->u_cmd = -this->ReactionWheelData[RWIter].u_max;
			}
		}

		// minimum torque
		if( std::abs(CmdIt->u_cmd) < this->ReactionWheelData[RWIter].u_min) {
			CmdIt->u_cmd = 0.0;
		}

		// Coulomb friction
		if (this->ReactionWheelData[RWIter].linearFrictionRatio > 0.0) {
			omegaCritical = this->ReactionWheelData[RWIter].Omega_max * this->ReactionWheelData[RWIter].linearFrictionRatio;
		} else {
			omegaCritical = 0.0;
		}
		if(this->ReactionWheelData[RWIter].Omega > omegaCritical) {
			u_s = CmdIt->u_cmd - this->ReactionWheelData[RWIter].u_f;
		} else if(this->ReactionWheelData[RWIter].Omega < -omegaCritical) {
			u_s = CmdIt->u_cmd + this->ReactionWheelData[RWIter].u_f;
		} else {
			if (this->ReactionWheelData[RWIter].linearFrictionRatio > 0) {
				u_s = CmdIt->u_cmd - this->ReactionWheelData[RWIter].u_f*this->ReactionWheelData[RWIter].Omega/omegaCritical;
			} else {
				u_s = CmdIt->u_cmd;
			}
		}

		this->ReactionWheelData[RWIter].u_current = u_s; // save actual torque for reaction wheel motor

		// zero previous RW jitter torque/force vector
		this->ReactionWheelData[RWIter].tau_B.setZero();
		this->ReactionWheelData[RWIter].F_B.setZero();
		// imbalance torque
		if (this->ReactionWheelData[RWIter].RWModel == JitterSimple) {

			cosTheta = cos(this->ReactionWheelData[RWIter].theta);
			sinTheta = sin(this->ReactionWheelData[RWIter].theta);

			temp1 = cosTheta * this->ReactionWheelData[RWIter].gtHat0_B;
			temp2 = sinTheta * this->ReactionWheelData[RWIter].ggHat0_B;
			gtHat_B = temp1 + temp2; // current gtHat axis vector represented in body frame

			/* Fs = Us * Omega^2 */ // calculate static imbalance force
			this->ReactionWheelData[RWIter].F_B = this->ReactionWheelData[RWIter].U_s * pow(this->ReactionWheelData[RWIter].Omega,2) * gtHat_B;
			this->sumF_B = this->ReactionWheelData[RWIter].F_B + this->sumF_B;

			/* tau_s = cross(r_B,Fs) */ // calculate static imbalance torque
			this->ReactionWheelData[RWIter].tau_B = this->ReactionWheelData[RWIter].rWB_B.cross(this->ReactionWheelData[RWIter].F_B);
			/* tau_d = Ud * Omega^2 */ // calculate dynamic imbalance torque
			temp2 = this->ReactionWheelData[RWIter].U_d*pow(this->ReactionWheelData[RWIter].Omega,2) * gtHat_B;
			// add in dynamic imbalance torque
			this->ReactionWheelData[RWIter].tau_B = this->ReactionWheelData[RWIter].tau_B + temp2;
			this->sumTau_B = this->ReactionWheelData[RWIter].tau_B + this->sumTau_B;

		} else if (this->ReactionWheelData[RWIter].RWModel == JitterFullyCoupled) {

		}

		RWIter++;

	}
}

/*! This method is used to compute all the dynamical effects for the RW set.
 It is an inherited method from the DynEffector class and is designed to be called
 by the dynamics plant for the simulation.
 @return void
 @param Props Current mass properties of the vehicle (using center of mass and str2bdy transformation
 @param Bstate Current state of the vehicle (not used by RW dynamics)
 @param CurrentTime Current simulation time converted to double precision
 */
void ReactionWheelStateEffector::ComputeDynamics(MassPropsData *Props,
											OutputStateData *Bstate,
											double CurrentTime)
{}

/*! This method is the main cyclical call for the scheduled part of the RW
 dynamics model.  It reads the current commands array and sets the RW
 configuration data based on that incoming command set.  Note that the main
 dynamical method (ComputeDynamics()) is not called here and is intended to be
 called from the dynamics plant in the system
 @return void
 @param CurrentSimNanos The current simulation time in nanoseconds
 */
void ReactionWheelStateEffector::UpdateState(uint64_t CurrentSimNanos)
{
	//! Begin method steps
	//! - Read the inputs and then call ConfigureRWRequests to set up dynamics
	ReadInputs();
	ConfigureRWRequests(CurrentSimNanos*NANO2SEC);
	WriteOutputMessages(CurrentSimNanos);
//
}

