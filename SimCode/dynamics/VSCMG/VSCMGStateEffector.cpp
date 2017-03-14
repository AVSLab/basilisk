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


#include "VSCMGStateEffector.h"
#include "architecture/messaging/system_messaging.h"
#include <cstring>
#include <iostream>
#include <cmath>

VSCMGStateEffector::VSCMGStateEffector()
{
	CallCounts = 0;
	InputCmds = "vscmg_cmds";
	OutputDataString = "vscmg_output_states";
	OutputBufferCount = 2;
	CmdsInMsgID = -1;
	StateOutMsgID = -1;
	prevCommandTime = 0xFFFFFFFFFFFFFFFF;

    effProps.mEff = 0.0;
    effProps.IEffPntB_B.setZero();
    effProps.rEff_CB_B.setZero();
	effProps.IEffPrimePntB_B.setZero();
	effProps.rEffPrime_CB_B.setZero();

    this->nameOfVSCMGOmegasState = "VSCMGOmegas";
    this->nameOfVSCMGThetasState = "VSCMGThetas";
    
    return;
}


VSCMGStateEffector::~VSCMGStateEffector()
{
    return;
}

void VSCMGStateEffector::linkInStates(DynParamManager& statesIn)
{
	//! - Get access to the hubs sigma, omegaBN_B and velocity needed for dynamic coupling
	this->hubSigma = statesIn.getStateObject("hubSigma");
	this->hubOmega = statesIn.getStateObject("hubOmega");
	this->hubVelocity = statesIn.getStateObject("hubVelocity");
    this->g_N = statesIn.getPropertyReference("g_N");

	return;
}

void VSCMGStateEffector::registerStates(DynParamManager& states)
{
    //! - Find number of RWs and number of RWs with jitter
    this->numRWJitter = 0;
    this->numRW = 0;
    std::vector<VSCMGConfigSimMsg>::iterator RWIt;
    //! zero the RW Omega and theta values (is there I should do this?)
    Eigen::MatrixXd omegasForInit(this->ReactionWheelData.size(),1);

    for(RWIt=ReactionWheelData.begin(); RWIt!=ReactionWheelData.end(); RWIt++) {
        if (RWIt->RWModel == JitterSimple || RWIt->RWModel == JitterFullyCoupled) {
            this->numRWJitter++;
        }
        omegasForInit(RWIt - this->ReactionWheelData.begin(), 0) = RWIt->Omega;
        this->numRW++;
    }
    
	this->OmegasState = states.registerState(this->numRW, 1, this->nameOfVSCMGOmegasState);

	if (numRWJitter > 0) {
		this->thetasState = states.registerState(this->numRWJitter, 1, this->nameOfVSCMGThetasState);
	}

    this->OmegasState->setState(omegasForInit);
    if (this->numRWJitter > 0) {
        Eigen::MatrixXd thetasForZeroing(this->numRWJitter,1);
        thetasForZeroing.setZero();
        this->thetasState->setState(thetasForZeroing);
    }

    return;
}

void VSCMGStateEffector::updateEffectorMassProps(double integTime)
{
    // - Zero the mass props information because these will be accumulated during this call
    this->effProps.mEff = 0.;
    this->effProps.rEff_CB_B.setZero();
    this->effProps.IEffPntB_B.setZero();
    this->effProps.rEffPrime_CB_B.setZero();
    this->effProps.IEffPrimePntB_B.setZero();
    
    int thetaCount = 0;
    std::vector<VSCMGConfigSimMsg>::iterator RWIt;
	for(RWIt=ReactionWheelData.begin(); RWIt!=ReactionWheelData.end(); RWIt++)
	{
		RWIt->Omega = this->OmegasState->getState()(RWIt - ReactionWheelData.begin(), 0);
		if (RWIt->RWModel == JitterFullyCoupled) {
			RWIt->theta = this->thetasState->getState()(thetaCount, 0);
			Eigen::Matrix3d dcm_WW0 = eigenM1(RWIt->theta);
			Eigen::Matrix3d dcm_BW0;
			dcm_BW0.col(0) = RWIt->gsHat_B;
			dcm_BW0.col(1) = RWIt->w2Hat0_B;
			dcm_BW0.col(2) = RWIt->w3Hat0_B;
			Eigen::Matrix3d dcm_BW = dcm_BW0 * dcm_WW0.transpose();
			RWIt->w2Hat_B = dcm_BW.col(1);
			RWIt->w3Hat_B = dcm_BW.col(2);

			//! wheel inertia tensor about wheel center of mass represented in B frame
			Eigen::Matrix3d IRWPntWc_W;
			IRWPntWc_W << RWIt->Js, 0., RWIt->J13, \
								0., RWIt->Jt, 0., \
								RWIt->J13, 0., RWIt->Jg;
			RWIt->IRWPntWc_B = dcm_BW * IRWPntWc_W * dcm_BW.transpose();

			//! wheel inertia tensor body frame derivative about wheel center of mass represented in B frame
			Eigen::Matrix3d IPrimeRWPntWc_W;
			IPrimeRWPntWc_W << 0., -RWIt->J13, 0., \
								-RWIt->J13, 0., 0., \
								0., 0., 0.;
			IPrimeRWPntWc_W *= RWIt->Omega;
			RWIt->IPrimeRWPntWc_B = dcm_BW * IPrimeRWPntWc_W * dcm_BW.transpose();

			//! wheel center of mass location
			RWIt->rWcB_B = RWIt->rWB_B + RWIt->d*RWIt->w2Hat_B;
			RWIt->rTildeWcB_B = eigenTilde(RWIt->rWcB_B);
			RWIt->rPrimeWcB_B = RWIt->d*RWIt->Omega*RWIt->w3Hat_B;
			Eigen::Matrix3d rPrimeTildeWcB_B = eigenTilde(RWIt->rPrimeWcB_B);

			//! - Give the mass of the reaction wheel to the effProps mass
			this->effProps.mEff += RWIt->mass;
			this->effProps.rEff_CB_B += RWIt->mass*RWIt->rWcB_B;
			this->effProps.IEffPntB_B += RWIt->IRWPntWc_B + RWIt->mass*RWIt->rTildeWcB_B*RWIt->rTildeWcB_B.transpose();
			this->effProps.rEffPrime_CB_B += RWIt->mass*RWIt->rPrimeWcB_B;
			this->effProps.IEffPrimePntB_B += RWIt->IPrimeRWPntWc_B + RWIt->mass*rPrimeTildeWcB_B*RWIt->rTildeWcB_B.transpose() + RWIt->mass*RWIt->rTildeWcB_B*rPrimeTildeWcB_B.transpose();
            thetaCount++;
		} else if (RWIt->RWModel == JitterSimple) {
			RWIt->theta = this->thetasState->getState()(thetaCount, 0);
			Eigen::Matrix3d dcm_WW0 = eigenM1(RWIt->theta);
			Eigen::Matrix3d dcm_BW0;
			dcm_BW0.col(0) = RWIt->gsHat_B;
			dcm_BW0.col(1) = RWIt->w2Hat0_B;
			dcm_BW0.col(2) = RWIt->w3Hat0_B;
			Eigen::Matrix3d dcm_BW = dcm_BW0 * dcm_WW0.transpose();
			RWIt->w2Hat_B = dcm_BW.col(1);
			RWIt->w3Hat_B = dcm_BW.col(2);
			thetaCount++;
		}
	}

    // - Need to divide out the total mass of the reaction wheels from rCB_B and rPrimeCB_B
    if (this->effProps.mEff > 0) {
        this->effProps.rEff_CB_B /= this->effProps.mEff;
        this->effProps.rEffPrime_CB_B /= this->effProps.mEff;
    }

	return;
}

void VSCMGStateEffector::updateContributions(double integTime, Eigen::Matrix3d & matrixAcontr, Eigen::Matrix3d & matrixBcontr, Eigen::Matrix3d & matrixCcontr, Eigen::Matrix3d & matrixDcontr, Eigen::Vector3d & vecTranscontr, Eigen::Vector3d & vecRotcontr)
{
	Eigen::Vector3d omegaLoc_BN_B;
	Eigen::Vector3d tempF;
	double omegas;
	double omegaw2;
	double omegaw3;
	double dSquared;
	double OmegaSquared;
    Eigen::MRPd sigmaBNLocal;
    Eigen::Matrix3d dcm_BN;                        /* direction cosine matrix from N to B */
    Eigen::Matrix3d dcm_NB;                        /* direction cosine matrix from B to N */
    Eigen::Vector3d gravityTorquePntW_B;          /* torque of gravity on HRB about Pnt H */
    Eigen::Vector3d gLocal_N;                          /* gravitational acceleration in N frame */
    Eigen::Vector3d g_B;                          /* gravitational acceleration in B frame */
    gLocal_N = *this->g_N;

    //! - Find dcm_BN
    sigmaBNLocal = (Eigen::Vector3d )this->hubSigma->getState();
    dcm_NB = sigmaBNLocal.toRotationMatrix();
    dcm_BN = dcm_NB.transpose();
    //! - Map gravity to body frame
    g_B = dcm_BN*gLocal_N;

	omegaLoc_BN_B = this->hubOmega->getState();

    std::vector<VSCMGConfigSimMsg>::iterator RWIt;
	for(RWIt=ReactionWheelData.begin(); RWIt!=ReactionWheelData.end(); RWIt++)
	{
		OmegaSquared = RWIt->Omega * RWIt->Omega;

		if (RWIt->RWModel == BalancedWheels || RWIt->RWModel == JitterSimple) {
			matrixDcontr -= RWIt->Js * RWIt->gsHat_B * RWIt->gsHat_B.transpose();
			vecRotcontr -= RWIt->gsHat_B * RWIt->u_current + RWIt->Js*RWIt->Omega*omegaLoc_BN_B.cross(RWIt->gsHat_B);

			//! imbalance torque (simplified external)
			if (RWIt->RWModel == JitterSimple) {
				/* Fs = Us * Omega^2 */ // static imbalance force
				tempF = RWIt->U_s * OmegaSquared * RWIt->w2Hat_B;
				vecTranscontr += tempF;

				//! add in dynamic imbalance torque
				/* tau_s = cross(r_B,Fs) */ // static imbalance torque
				/* tau_d = Ud * Omega^2 */ // dynamic imbalance torque
				vecRotcontr += ( RWIt->rWB_B.cross(tempF) ) + ( RWIt->U_d*OmegaSquared * RWIt->w2Hat_B );
			}
        } else if (RWIt->RWModel == JitterFullyCoupled) {

			omegas = RWIt->gsHat_B.transpose()*omegaLoc_BN_B;
			omegaw2 = RWIt->w2Hat_B.transpose()*omegaLoc_BN_B;
			omegaw3 = RWIt->w3Hat_B.transpose()*omegaLoc_BN_B;

            gravityTorquePntW_B = RWIt->d*RWIt->w2Hat_B.cross(RWIt->mass*g_B);

			dSquared = RWIt->d * RWIt->d;

			RWIt->aOmega = -RWIt->mass*RWIt->d/(RWIt->Js + RWIt->mass*dSquared) * RWIt->w3Hat_B;
			RWIt->bOmega = -1.0/(RWIt->Js + RWIt->mass*dSquared)*((RWIt->Js+RWIt->mass*dSquared)*RWIt->gsHat_B + RWIt->J13*RWIt->w3Hat_B + RWIt->mass*RWIt->d*RWIt->rWB_B.cross(RWIt->w3Hat_B));
			RWIt->cOmega = 1.0/(RWIt->Js + RWIt->mass*dSquared)*(omegaw2*omegaw3*(-RWIt->mass*dSquared)-RWIt->J13*omegaw2*omegas-RWIt->mass*RWIt->d*RWIt->w3Hat_B.transpose()*omegaLoc_BN_B.cross(omegaLoc_BN_B.cross(RWIt->rWB_B))+RWIt->u_current + RWIt->gsHat_B.dot(gravityTorquePntW_B));

			matrixAcontr += RWIt->mass * RWIt->d * RWIt->w3Hat_B * RWIt->aOmega.transpose();
			matrixBcontr += RWIt->mass * RWIt->d * RWIt->w3Hat_B * RWIt->bOmega.transpose();
			matrixCcontr += (RWIt->IRWPntWc_B*RWIt->gsHat_B + RWIt->mass*RWIt->d*RWIt->rWcB_B.cross(RWIt->w3Hat_B))*RWIt->aOmega.transpose();
			matrixDcontr += (RWIt->IRWPntWc_B*RWIt->gsHat_B + RWIt->mass*RWIt->d*RWIt->rWcB_B.cross(RWIt->w3Hat_B))*RWIt->bOmega.transpose();
			vecTranscontr += RWIt->mass*RWIt->d*(OmegaSquared*RWIt->w2Hat_B - RWIt->cOmega*RWIt->w3Hat_B);
			vecRotcontr += RWIt->mass*RWIt->d*OmegaSquared*RWIt->rWcB_B.cross(RWIt->w2Hat_B) - RWIt->IPrimeRWPntWc_B*RWIt->Omega*RWIt->gsHat_B - omegaLoc_BN_B.cross(RWIt->IRWPntWc_B*RWIt->Omega*RWIt->gsHat_B+RWIt->mass*RWIt->rWcB_B.cross(RWIt->rPrimeWcB_B)) - (RWIt->IRWPntWc_B*RWIt->gsHat_B+RWIt->mass*RWIt->d*RWIt->rWcB_B.cross(RWIt->w3Hat_B))*RWIt->cOmega;
		}
	}
	return;
}

void VSCMGStateEffector::computeDerivatives(double integTime)
{
	Eigen::MatrixXd OmegasDot(this->numRW,1);
    Eigen::MatrixXd thetasDot(this->numRWJitter,1);
	Eigen::Vector3d omegaDotBNLoc_B;
	Eigen::MRPd sigmaBNLocal;
	Eigen::Matrix3d dcm_BN;                        /* direction cosine matrix from N to B */
	Eigen::Matrix3d dcm_NB;                        /* direction cosine matrix from B to N */
	Eigen::Vector3d rDDotBNLoc_N;                 /* second time derivative of rBN in N frame */
	Eigen::Vector3d rDDotBNLoc_B;                 /* second time derivative of rBN in B frame */
	int RWi = 0;
    int thetaCount = 0;
	std::vector<VSCMGConfigSimMsg>::iterator RWIt;

	//! Grab necessarry values from manager
	omegaDotBNLoc_B = this->hubOmega->getStateDeriv();
	rDDotBNLoc_N = this->hubVelocity->getStateDeriv();
	sigmaBNLocal = (Eigen::Vector3d )this->hubSigma->getState();
	dcm_NB = sigmaBNLocal.toRotationMatrix();
	dcm_BN = dcm_NB.transpose();
	rDDotBNLoc_B = dcm_BN*rDDotBNLoc_N;

	//! - Compute Derivatives
	for(RWIt=ReactionWheelData.begin(); RWIt!=ReactionWheelData.end(); RWIt++)
	{
        if(RWIt->RWModel == JitterFullyCoupled || RWIt->RWModel == JitterSimple) {
            // - Set trivial kinemetic derivative
            thetasDot(thetaCount,0) = RWIt->Omega;
            thetaCount++;
        }
		if (RWIt->RWModel == BalancedWheels || RWIt->RWModel == JitterSimple) {
			OmegasDot(RWi,0) = RWIt->u_current/RWIt->Js - RWIt->gsHat_B.transpose()*omegaDotBNLoc_B;
        } else if(RWIt->RWModel == JitterFullyCoupled) {
			OmegasDot(RWi,0) = RWIt->aOmega.dot(rDDotBNLoc_B) + RWIt->bOmega.dot(omegaDotBNLoc_B) + RWIt->cOmega;
		}
		RWi++;
	}

	OmegasState->setDerivative(OmegasDot);
    if (this->numRWJitter > 0) {
        thetasState->setDerivative(thetasDot);
    }
}

void VSCMGStateEffector::updateEnergyMomContributions(double integTime, Eigen::Vector3d & rotAngMomPntCContr_B, double & rotEnergyContr)
{
	Eigen::MRPd sigmaBNLocal;
	Eigen::Matrix3d dcm_BN;                        /* direction cosine matrix from N to B */
	Eigen::Matrix3d dcm_NB;                        /* direction cosine matrix from B to N */
	Eigen::Vector3d omegaLoc_BN_B = hubOmega->getState();

    //! - Compute energy and momentum contribution of each wheel
    rotAngMomPntCContr_B.setZero();
    std::vector<VSCMGConfigSimMsg>::iterator RWIt;
    for(RWIt=ReactionWheelData.begin(); RWIt!=ReactionWheelData.end(); RWIt++)
    {
		if (RWIt->RWModel == BalancedWheels || RWIt->RWModel == JitterSimple) {
			rotAngMomPntCContr_B += RWIt->Js*RWIt->gsHat_B*RWIt->Omega;
			rotEnergyContr += 1.0/2.0*RWIt->Js*RWIt->Omega*RWIt->Omega;
		} else if (RWIt->RWModel == JitterFullyCoupled) {
			Eigen::Vector3d omega_WN_B = omegaLoc_BN_B + RWIt->Omega*RWIt->gsHat_B;
			Eigen::Vector3d r_WcB_B = RWIt->rWcB_B;
			Eigen::Vector3d rDot_WcB_B = RWIt->d*RWIt->Omega*RWIt->w3Hat_B + omegaLoc_BN_B.cross(RWIt->rWcB_B);

			rotEnergyContr += 0.5*omega_WN_B.transpose()*RWIt->IRWPntWc_B*omega_WN_B + 0.5*RWIt->mass*rDot_WcB_B.dot(rDot_WcB_B);
			rotAngMomPntCContr_B += RWIt->IRWPntWc_B*omega_WN_B + RWIt->mass*r_WcB_B.cross(rDot_WcB_B);
		}
    }

    return;
}

/*! This method is used to clear out the current RW states and make sure
 that the overall model is ready
 @return void
 */
void VSCMGStateEffector::SelfInit()
{
	SystemMessaging *messageSys = SystemMessaging::GetInstance();
	VSCMGCmdSimMsg RWCmdInitializer;
	RWCmdInitializer.u_s_cmd = 0.0;

	//! Begin method steps
	//! - Clear out any currently firing RWs and re-init cmd array
	NewRWCmds.clear();
	NewRWCmds.insert(NewRWCmds.begin(), ReactionWheelData.size(), RWCmdInitializer );

	// Reserve a message ID for each reaction wheel config output message
	uint64_t tmpWheeltMsgId;
	std::string tmpWheelMsgName;
	std::vector<VSCMGConfigSimMsg>::iterator it;
	for (it = ReactionWheelData.begin(); it != ReactionWheelData.end(); it++)
	{
		tmpWheelMsgName = "rw_bla" + std::to_string(it - ReactionWheelData.begin()) + "_data";
		tmpWheeltMsgId = messageSys->CreateNewMessage(tmpWheelMsgName, sizeof(VSCMGConfigSimMsg), OutputBufferCount, "VSCMGConfigSimMsg", moduleID);
		this->rwOutMsgNames.push_back(tmpWheelMsgName);
		this->rwOutMsgIds.push_back(tmpWheeltMsgId);
	}

	StateOutMsgID = messageSys->CreateNewMessage(OutputDataString, sizeof(RWSpeedIntMsg),
												 OutputBufferCount, "RWSpeedIntMsg", moduleID);

    return;
}

/*! This method is used to connect the input command message to the RWs.
 It sets the message ID based on what it finds for the input string.  If the
 message is not successfully linked, it will warn the user.
 @return void
 */
void VSCMGStateEffector::CrossInit()
{
    //! massProps doesn't exist anymore, hardcode structure to body for now (NEEDS TO CHANGE)
    Eigen::Matrix3d dcm_BS;             /* structure to body frame DCM */
    dcm_BS.setIdentity();

	//! Begin method steps
	//! - Find the message ID associated with the InputCmds string.
	//! - Warn the user if the message is not successfully linked.
	CmdsInMsgID = SystemMessaging::GetInstance()->subscribeToMessage(InputCmds,
                                                                     sizeof(RWArrayTorqueIntMsg),
																	 moduleID);
	if(CmdsInMsgID < 0)
	{
		std::cerr << "WARNING: Did not find a valid message with name: ";
		std::cerr << InputCmds << "  :" << std::endl<< __FILE__ << std::endl;
	}

	std::vector<VSCMGConfigSimMsg>::iterator it;
	for (it = ReactionWheelData.begin(); it != ReactionWheelData.end(); it++)
	{
		if (it->gsHat_S.norm() > 0.01) {
			it->gsHat_B = dcm_BS * it->gsHat_S;
		} else {
			std::cerr <<
			"Error: gsHat_S not properly initialized.  Don't set gsHat_B directly in python.";
		}
		if (it->RWModel == JitterSimple || it->RWModel == JitterFullyCoupled) {
			if (it->w2Hat0_S.norm() > 0.01) {
				it->w2Hat0_B = dcm_BS * it->w2Hat0_S;
			} else {
				std::cerr << "Error: gtHat0_S not properly initialized.  Don't set gtHat0_B directly in python.";
			}
			if (it->w3Hat0_S.norm() > 0.01) {
				it->w3Hat0_B = dcm_BS * it->w3Hat0_S;
			} else {
				std::cerr << "Error: ggHat0_S not properly initialized.  Don't set ggHat0_S directly in python.";
			}
		}

		//! Define CoM offset d and off-diagonal inertia J13 if using fully coupled model
		if (it->RWModel == JitterFullyCoupled) {
			it->d = it->U_s/it->mass; //!< determine CoM offset from static imbalance parameter
			it->J13 = it->U_d; //!< off-diagonal inertia is equal to dynamic imbalance parameter
		}

		it->rWB_B = dcm_BS * it->rWB_S;
	}
}

/*! This method is here to write the output message structure into the specified
 message.  It is currently blank but we will certainly have an output message
 soon.  If it is already here, bludgeon whoever added it and didn't fix the
 comment.
 @param CurrentClock The current time used for time-stamping the message
 @return void
 */
void VSCMGStateEffector::WriteOutputMessages(uint64_t CurrentClock)
{
	SystemMessaging *messageSys = SystemMessaging::GetInstance();
	VSCMGConfigSimMsg tmpRW;
	std::vector<VSCMGConfigSimMsg>::iterator it;
	for (it = ReactionWheelData.begin(); it != ReactionWheelData.end(); it++)
	{
        if (numRWJitter > 0) {
            double thetaCurrent = this->thetasState->getState()(it - ReactionWheelData.begin(), 0);
            it->theta = thetaCurrent;
        }
        double omegaCurrent = this->OmegasState->getState()(it - ReactionWheelData.begin(), 0);
        it->Omega = omegaCurrent;
		outputStates.wheelSpeeds[it - ReactionWheelData.begin()] = it->Omega;

		tmpRW.rWB_S = it->rWB_S;
		tmpRW.gsHat_S = it->gsHat_S;
		tmpRW.w2Hat0_S = it->w2Hat0_S;
		tmpRW.w3Hat0_S = it->w3Hat0_S;
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
								 sizeof(VSCMGConfigSimMsg),
								 reinterpret_cast<uint8_t*> (&tmpRW),
								 moduleID);
	}

	// Write this message once for all reaction wheels
	messageSys->WriteMessage(StateOutMsgID, CurrentClock,
							 sizeof(RWSpeedIntMsg), reinterpret_cast<uint8_t*> (&outputStates), moduleID);
}

/*! This method is used to read the incoming command message and set the
 associated command structure for operating the RWs.
 @return void
 */
void VSCMGStateEffector::ReadInputs()
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
	memset(IncomingCmdBuffer.motorTorque, 0x0, sizeof(RWArrayTorqueIntMsg));
	SystemMessaging::GetInstance()->ReadMessage(CmdsInMsgID, &LocalHeader,
												sizeof(RWArrayTorqueIntMsg),
												reinterpret_cast<uint8_t*> (&IncomingCmdBuffer), moduleID);

	//! - Check if message has already been read, if stale return
	//    if(prevCommandTime==LocalHeader.WriteClockNanos) {
	//        return;
	//    }
	prevCommandTime = LocalHeader.WriteClockNanos;

	//! - Set the NewRWCmds vector.  Using the data() method for raw speed
	VSCMGCmdSimMsg *CmdPtr;
	for(i=0, CmdPtr = NewRWCmds.data(); i<ReactionWheelData.size();
		CmdPtr++, i++)
	{
		CmdPtr->u_s_cmd = IncomingCmdBuffer.motorTorque[i];
	}

}

///*! This method is used to read the new commands vector and set the RW
// firings appropriately.  It assumes that the ReadInputs method has already been
// run successfully.
// @return void
// @param CurrentTime The current simulation time converted to a double
// */
void VSCMGStateEffector::ConfigureRWRequests(double CurrentTime)
{
	//! Begin method steps
	std::vector<VSCMGCmdSimMsg>::iterator CmdIt;
	int RWIter = 0;
	double u_s;
	double omegaCritical;

	// loop through commands
	for(CmdIt=NewRWCmds.begin(); CmdIt!=NewRWCmds.end(); CmdIt++)
	{
		// saturation
		if (this->ReactionWheelData[RWIter].u_max > 0) {
			if(CmdIt->u_s_cmd > this->ReactionWheelData[RWIter].u_max) {
				CmdIt->u_s_cmd = this->ReactionWheelData[RWIter].u_max;
			} else if(CmdIt->u_s_cmd < -this->ReactionWheelData[RWIter].u_max) {
				CmdIt->u_s_cmd = -this->ReactionWheelData[RWIter].u_max;
			}
		}

		// minimum torque
		if( std::abs(CmdIt->u_s_cmd) < this->ReactionWheelData[RWIter].u_min) {
			CmdIt->u_s_cmd = 0.0;
		}

		// Coulomb friction
		if (this->ReactionWheelData[RWIter].linearFrictionRatio > 0.0) {
			omegaCritical = this->ReactionWheelData[RWIter].Omega_max * this->ReactionWheelData[RWIter].linearFrictionRatio;
		} else {
			omegaCritical = 0.0;
		}
		if(this->ReactionWheelData[RWIter].Omega > omegaCritical) {
			u_s = CmdIt->u_s_cmd - this->ReactionWheelData[RWIter].u_f;
		} else if(this->ReactionWheelData[RWIter].Omega < -omegaCritical) {
			u_s = CmdIt->u_s_cmd + this->ReactionWheelData[RWIter].u_f;
		} else {
			if (this->ReactionWheelData[RWIter].linearFrictionRatio > 0) {
				u_s = CmdIt->u_s_cmd - this->ReactionWheelData[RWIter].u_f*this->ReactionWheelData[RWIter].Omega/omegaCritical;
			} else {
				u_s = CmdIt->u_s_cmd;
			}
		}

		this->ReactionWheelData[RWIter].u_current = u_s; // save actual torque for reaction wheel motor

		RWIter++;

	}
}

/*! This method is the main cyclical call for the scheduled part of the RW
 dynamics model.  It reads the current commands array and sets the RW
 configuration data based on that incoming command set.  Note that the main
 dynamical method (ComputeDynamics()) is not called here and is intended to be
 called from the dynamics plant in the system
 @return void
 @param CurrentSimNanos The current simulation time in nanoseconds
 */
void VSCMGStateEffector::UpdateState(uint64_t CurrentSimNanos)
{
	//! Begin method steps
	//! - Read the inputs and then call ConfigureRWRequests to set up dynamics
	ReadInputs();
	ConfigureRWRequests(CurrentSimNanos*NANO2SEC);
	WriteOutputMessages(CurrentSimNanos);
//
}

