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
	this->nameOfVSCMGGammasState = "VSCMGGammas";
	this->nameOfVSCMGGammaDotsState = "VSCMGGammaDots";
    
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
    //! - Find number of VSCMGs and number of VSCMGs with jitter
    this->numVSCMGJitter = 0;
    this->numVSCMG = 0;
    std::vector<VSCMGConfigSimMsg>::iterator vscmgIt;
    //! zero the Omega and theta values
    Eigen::MatrixXd omegasForInit(this->VSCMGData.size(),1);
	Eigen::MatrixXd gammasForInit(this->VSCMGData.size(),1);
	Eigen::MatrixXd gammaDotsForInit(this->VSCMGData.size(),1);

    for(vscmgIt=VSCMGData.begin(); vscmgIt!=VSCMGData.end(); vscmgIt++) {
        if (vscmgIt->VSCMGModel == JitterSimple || vscmgIt->VSCMGModel == JitterFullyCoupled) {
            this->numVSCMGJitter++;
        }
        omegasForInit(vscmgIt - this->VSCMGData.begin(), 0) = vscmgIt->Omega;
		gammasForInit(vscmgIt - this->VSCMGData.begin(), 0) = vscmgIt->gamma;
		gammaDotsForInit(vscmgIt - this->VSCMGData.begin(), 0) = vscmgIt->gammaDot;
        this->numVSCMG++;
    }
    
	this->OmegasState = states.registerState(this->numVSCMG, 1, this->nameOfVSCMGOmegasState);
	this->gammasState = states.registerState(this->numVSCMG, 1, this->nameOfVSCMGGammasState);
	this->gammaDotsState = states.registerState(this->numVSCMG, 1, this->nameOfVSCMGGammaDotsState);

    this->OmegasState->setState(omegasForInit);
	this->gammasState->setState(gammasForInit);
	this->gammaDotsState->setState(gammaDotsForInit);

	if (numVSCMGJitter > 0) {
		this->thetasState = states.registerState(this->numVSCMGJitter, 1, this->nameOfVSCMGThetasState);
        Eigen::MatrixXd thetasForZeroing(this->numVSCMGJitter,1);
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
    std::vector<VSCMGConfigSimMsg>::iterator vscmgIt;
	for(vscmgIt=VSCMGData.begin(); vscmgIt!=VSCMGData.end(); vscmgIt++)
	{
		vscmgIt->Omega = this->OmegasState->getState()(vscmgIt - VSCMGData.begin(), 0);
		if (vscmgIt->VSCMGModel == JitterFullyCoupled) {
			vscmgIt->theta = this->thetasState->getState()(thetaCount, 0);
			Eigen::Matrix3d dcm_WW0 = eigenM1(vscmgIt->theta);
			Eigen::Matrix3d dcm_BW0;
			dcm_BW0.col(0) = vscmgIt->gsHat_B;
			dcm_BW0.col(1) = vscmgIt->w2Hat0_B;
			dcm_BW0.col(2) = vscmgIt->w3Hat0_B;
			Eigen::Matrix3d dcm_BW = dcm_BW0 * dcm_WW0.transpose();
			vscmgIt->w2Hat_B = dcm_BW.col(1);
			vscmgIt->w3Hat_B = dcm_BW.col(2);

			//! wheel inertia tensor about wheel center of mass represented in B frame
			Eigen::Matrix3d IRWPntWc_W;
			IRWPntWc_W << vscmgIt->Js, 0., vscmgIt->J13, \
								0., vscmgIt->Jt, 0., \
								vscmgIt->J13, 0., vscmgIt->Jg;
			vscmgIt->IRWPntWc_B = dcm_BW * IRWPntWc_W * dcm_BW.transpose();

			//! wheel inertia tensor body frame derivative about wheel center of mass represented in B frame
			Eigen::Matrix3d IPrimeRWPntWc_W;
			IPrimeRWPntWc_W << 0., -vscmgIt->J13, 0., \
								-vscmgIt->J13, 0., 0., \
								0., 0., 0.;
			IPrimeRWPntWc_W *= vscmgIt->Omega;
			vscmgIt->IPrimeRWPntWc_B = dcm_BW * IPrimeRWPntWc_W * dcm_BW.transpose();

			//! wheel center of mass location
			vscmgIt->rWcB_B = vscmgIt->rWB_B + vscmgIt->d*vscmgIt->w2Hat_B;
			vscmgIt->rTildeWcB_B = eigenTilde(vscmgIt->rWcB_B);
			vscmgIt->rPrimeWcB_B = vscmgIt->d*vscmgIt->Omega*vscmgIt->w3Hat_B;
			Eigen::Matrix3d rPrimeTildeWcB_B = eigenTilde(vscmgIt->rPrimeWcB_B);

			//! - Give the mass of the reaction wheel to the effProps mass
			this->effProps.mEff += vscmgIt->mass;
			this->effProps.rEff_CB_B += vscmgIt->mass*vscmgIt->rWcB_B;
			this->effProps.IEffPntB_B += vscmgIt->IRWPntWc_B + vscmgIt->mass*vscmgIt->rTildeWcB_B*vscmgIt->rTildeWcB_B.transpose();
			this->effProps.rEffPrime_CB_B += vscmgIt->mass*vscmgIt->rPrimeWcB_B;
			this->effProps.IEffPrimePntB_B += vscmgIt->IPrimeRWPntWc_B + vscmgIt->mass*rPrimeTildeWcB_B*vscmgIt->rTildeWcB_B.transpose() + vscmgIt->mass*vscmgIt->rTildeWcB_B*rPrimeTildeWcB_B.transpose();
            thetaCount++;
		} else if (vscmgIt->VSCMGModel == JitterSimple) {
			vscmgIt->theta = this->thetasState->getState()(thetaCount, 0);
			Eigen::Matrix3d dcm_WW0 = eigenM1(vscmgIt->theta);
			Eigen::Matrix3d dcm_BW0;
			dcm_BW0.col(0) = vscmgIt->gsHat_B;
			dcm_BW0.col(1) = vscmgIt->w2Hat0_B;
			dcm_BW0.col(2) = vscmgIt->w3Hat0_B;
			Eigen::Matrix3d dcm_BW = dcm_BW0 * dcm_WW0.transpose();
			vscmgIt->w2Hat_B = dcm_BW.col(1);
			vscmgIt->w3Hat_B = dcm_BW.col(2);
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

    std::vector<VSCMGConfigSimMsg>::iterator vscmgIt;
	for(vscmgIt=VSCMGData.begin(); vscmgIt!=VSCMGData.end(); vscmgIt++)
	{
		OmegaSquared = vscmgIt->Omega * vscmgIt->Omega;

		if (vscmgIt->VSCMGModel == BalancedWheels || vscmgIt->VSCMGModel == JitterSimple) {
			matrixDcontr -= vscmgIt->Js * vscmgIt->gsHat_B * vscmgIt->gsHat_B.transpose();
			vecRotcontr -= vscmgIt->gsHat_B * vscmgIt->u_current + vscmgIt->Js*vscmgIt->Omega*omegaLoc_BN_B.cross(vscmgIt->gsHat_B);

			//! imbalance torque (simplified external)
			if (vscmgIt->VSCMGModel == JitterSimple) {
				/* Fs = Us * Omega^2 */ // static imbalance force
				tempF = vscmgIt->U_s * OmegaSquared * vscmgIt->w2Hat_B;
				vecTranscontr += tempF;

				//! add in dynamic imbalance torque
				/* tau_s = cross(r_B,Fs) */ // static imbalance torque
				/* tau_d = Ud * Omega^2 */ // dynamic imbalance torque
				vecRotcontr += ( vscmgIt->rWB_B.cross(tempF) ) + ( vscmgIt->U_d*OmegaSquared * vscmgIt->w2Hat_B );
			}
        } else if (vscmgIt->VSCMGModel == JitterFullyCoupled) {

			omegas = vscmgIt->gsHat_B.transpose()*omegaLoc_BN_B;
			omegaw2 = vscmgIt->w2Hat_B.transpose()*omegaLoc_BN_B;
			omegaw3 = vscmgIt->w3Hat_B.transpose()*omegaLoc_BN_B;

            gravityTorquePntW_B = vscmgIt->d*vscmgIt->w2Hat_B.cross(vscmgIt->mass*g_B);

			dSquared = vscmgIt->d * vscmgIt->d;

			vscmgIt->aOmega = -vscmgIt->mass*vscmgIt->d/(vscmgIt->Js + vscmgIt->mass*dSquared) * vscmgIt->w3Hat_B;
			vscmgIt->bOmega = -1.0/(vscmgIt->Js + vscmgIt->mass*dSquared)*((vscmgIt->Js+vscmgIt->mass*dSquared)*vscmgIt->gsHat_B + vscmgIt->J13*vscmgIt->w3Hat_B + vscmgIt->mass*vscmgIt->d*vscmgIt->rWB_B.cross(vscmgIt->w3Hat_B));
			vscmgIt->cOmega = 1.0/(vscmgIt->Js + vscmgIt->mass*dSquared)*(omegaw2*omegaw3*(-vscmgIt->mass*dSquared)-vscmgIt->J13*omegaw2*omegas-vscmgIt->mass*vscmgIt->d*vscmgIt->w3Hat_B.transpose()*omegaLoc_BN_B.cross(omegaLoc_BN_B.cross(vscmgIt->rWB_B))+vscmgIt->u_current + vscmgIt->gsHat_B.dot(gravityTorquePntW_B));

			matrixAcontr += vscmgIt->mass * vscmgIt->d * vscmgIt->w3Hat_B * vscmgIt->aOmega.transpose();
			matrixBcontr += vscmgIt->mass * vscmgIt->d * vscmgIt->w3Hat_B * vscmgIt->bOmega.transpose();
			matrixCcontr += (vscmgIt->IRWPntWc_B*vscmgIt->gsHat_B + vscmgIt->mass*vscmgIt->d*vscmgIt->rWcB_B.cross(vscmgIt->w3Hat_B))*vscmgIt->aOmega.transpose();
			matrixDcontr += (vscmgIt->IRWPntWc_B*vscmgIt->gsHat_B + vscmgIt->mass*vscmgIt->d*vscmgIt->rWcB_B.cross(vscmgIt->w3Hat_B))*vscmgIt->bOmega.transpose();
			vecTranscontr += vscmgIt->mass*vscmgIt->d*(OmegaSquared*vscmgIt->w2Hat_B - vscmgIt->cOmega*vscmgIt->w3Hat_B);
			vecRotcontr += vscmgIt->mass*vscmgIt->d*OmegaSquared*vscmgIt->rWcB_B.cross(vscmgIt->w2Hat_B) - vscmgIt->IPrimeRWPntWc_B*vscmgIt->Omega*vscmgIt->gsHat_B - omegaLoc_BN_B.cross(vscmgIt->IRWPntWc_B*vscmgIt->Omega*vscmgIt->gsHat_B+vscmgIt->mass*vscmgIt->rWcB_B.cross(vscmgIt->rPrimeWcB_B)) - (vscmgIt->IRWPntWc_B*vscmgIt->gsHat_B+vscmgIt->mass*vscmgIt->d*vscmgIt->rWcB_B.cross(vscmgIt->w3Hat_B))*vscmgIt->cOmega;
		}
	}
	return;
}

void VSCMGStateEffector::computeDerivatives(double integTime)
{
	Eigen::MatrixXd OmegasDot(this->numVSCMG,1);
    Eigen::MatrixXd thetasDot(this->numVSCMGJitter,1);
	Eigen::MatrixXd gammasDot(this->numVSCMG,1);
	Eigen::MatrixXd gammaDotsDot(this->numVSCMG,1);
	Eigen::Vector3d omegaDotBNLoc_B;
	Eigen::MRPd sigmaBNLocal;
	Eigen::Matrix3d dcm_BN;                        /* direction cosine matrix from N to B */
	Eigen::Matrix3d dcm_NB;                        /* direction cosine matrix from B to N */
	Eigen::Vector3d rDDotBNLoc_N;                 /* second time derivative of rBN in N frame */
	Eigen::Vector3d rDDotBNLoc_B;                 /* second time derivative of rBN in B frame */
	int VSCMGi = 0;
    int thetaCount = 0;
	std::vector<VSCMGConfigSimMsg>::iterator vscmgIt;

	//! Grab necessarry values from manager
	omegaDotBNLoc_B = this->hubOmega->getStateDeriv();
	rDDotBNLoc_N = this->hubVelocity->getStateDeriv();
	sigmaBNLocal = (Eigen::Vector3d )this->hubSigma->getState();
	dcm_NB = sigmaBNLocal.toRotationMatrix();
	dcm_BN = dcm_NB.transpose();
	rDDotBNLoc_B = dcm_BN*rDDotBNLoc_N;

	//! - Compute Derivatives
	for(vscmgIt=VSCMGData.begin(); vscmgIt!=VSCMGData.end(); vscmgIt++)
	{
		gammasDot(VSCMGi,0) = vscmgIt->gammaDot;
        if(vscmgIt->VSCMGModel == JitterFullyCoupled || vscmgIt->VSCMGModel == JitterSimple) {
            // - Set trivial kinemetic derivative
            thetasDot(thetaCount,0) = vscmgIt->Omega;
            thetaCount++;
        }
		if (vscmgIt->VSCMGModel == BalancedWheels || vscmgIt->VSCMGModel == JitterSimple) {
			OmegasDot(VSCMGi,0) = vscmgIt->u_current/vscmgIt->Js - vscmgIt->gsHat_B.transpose()*omegaDotBNLoc_B;
			gammaDotsDot(VSCMGi,0) = 1.0;
        } else if(vscmgIt->VSCMGModel == JitterFullyCoupled) {
			OmegasDot(VSCMGi,0) = vscmgIt->aOmega.dot(rDDotBNLoc_B) + vscmgIt->bOmega.dot(omegaDotBNLoc_B) + vscmgIt->cOmega;
			gammaDotsDot(VSCMGi,0) = 1.0;
		}
		VSCMGi++;
	}

	OmegasState->setDerivative(OmegasDot);
	gammasState->setDerivative(gammasDot);
	gammaDotsState->setDerivative(gammaDotsDot);
    if (this->numVSCMGJitter > 0) {
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
    std::vector<VSCMGConfigSimMsg>::iterator vscmgIt;
    for(vscmgIt=VSCMGData.begin(); vscmgIt!=VSCMGData.end(); vscmgIt++)
    {
		if (vscmgIt->VSCMGModel == BalancedWheels || vscmgIt->VSCMGModel == JitterSimple) {
			rotAngMomPntCContr_B += vscmgIt->Js*vscmgIt->gsHat_B*vscmgIt->Omega;
			rotEnergyContr += 1.0/2.0*vscmgIt->Js*vscmgIt->Omega*vscmgIt->Omega;
		} else if (vscmgIt->VSCMGModel == JitterFullyCoupled) {
			Eigen::Vector3d omega_WN_B = omegaLoc_BN_B + vscmgIt->Omega*vscmgIt->gsHat_B;
			Eigen::Vector3d r_WcB_B = vscmgIt->rWcB_B;
			Eigen::Vector3d rDot_WcB_B = vscmgIt->d*vscmgIt->Omega*vscmgIt->w3Hat_B + omegaLoc_BN_B.cross(vscmgIt->rWcB_B);

			rotEnergyContr += 0.5*omega_WN_B.transpose()*vscmgIt->IRWPntWc_B*omega_WN_B + 0.5*vscmgIt->mass*rDot_WcB_B.dot(rDot_WcB_B);
			rotAngMomPntCContr_B += vscmgIt->IRWPntWc_B*omega_WN_B + vscmgIt->mass*r_WcB_B.cross(rDot_WcB_B);
		}
    }

    return;
}

/*! This method is used to clear out the current VSCMG states and make sure
 that the overall model is ready
 @return void
 */
void VSCMGStateEffector::SelfInit()
{
	SystemMessaging *messageSys = SystemMessaging::GetInstance();
	VSCMGCmdSimMsg VSCMGCmdInitializer;
	VSCMGCmdInitializer.u_s_cmd = 0.0;

	//! Begin method steps
	//! - Clear out any currently firing VSCMGs and re-init cmd array
	NewVSCMGCmds.clear();
	NewVSCMGCmds.insert(NewVSCMGCmds.begin(), VSCMGData.size(), VSCMGCmdInitializer );

	// Reserve a message ID for each reaction wheel config output message
	uint64_t tmpWheeltMsgId;
	std::string tmpWheelMsgName;
	std::vector<VSCMGConfigSimMsg>::iterator it;
	for (it = VSCMGData.begin(); it != VSCMGData.end(); it++)
	{
		tmpWheelMsgName = "vscmg_bla" + std::to_string(it - VSCMGData.begin()) + "_data";
		tmpWheeltMsgId = messageSys->CreateNewMessage(tmpWheelMsgName, sizeof(VSCMGConfigSimMsg), OutputBufferCount, "VSCMGConfigSimMsg", moduleID);
		this->vscmgOutMsgNames.push_back(tmpWheelMsgName);
		this->vscmgOutMsgIds.push_back(tmpWheeltMsgId);
	}

	StateOutMsgID = messageSys->CreateNewMessage(OutputDataString, sizeof(VSCMGSpeedIntMsg),
												 OutputBufferCount, "VSCMGSpeedIntMsg", moduleID);

    return;
}

/*! This method is used to connect the input command message to the VSCMGs.
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
                                                                     sizeof(VSCMGArrayTorqueIntMsg),
																	 moduleID);
	if(CmdsInMsgID < 0)
	{
		std::cerr << "WARNING: Did not find a valid message with name: ";
		std::cerr << InputCmds << "  :" << std::endl<< __FILE__ << std::endl;
	}

	std::vector<VSCMGConfigSimMsg>::iterator it;
	for (it = VSCMGData.begin(); it != VSCMGData.end(); it++)
	{
		if (it->gsHat_S.norm() > 0.01) {
			it->gsHat_B = dcm_BS * it->gsHat_S;
		} else {
			std::cerr <<
			"Error: gsHat_S not properly initialized.  Don't set gsHat_B directly in python.";
		}
		if (it->VSCMGModel == JitterSimple || it->VSCMGModel == JitterFullyCoupled) {
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
		if (it->VSCMGModel == JitterFullyCoupled) {
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
	VSCMGConfigSimMsg tmpVSCMG;
	std::vector<VSCMGConfigSimMsg>::iterator it;
	for (it = VSCMGData.begin(); it != VSCMGData.end(); it++)
	{
        if (numVSCMGJitter > 0) {
            double thetaCurrent = this->thetasState->getState()(it - VSCMGData.begin(), 0);
            it->theta = thetaCurrent;
        }
        double omegaCurrent = this->OmegasState->getState()(it - VSCMGData.begin(), 0);
        it->Omega = omegaCurrent;
		outputStates.wheelSpeeds[it - VSCMGData.begin()] = it->Omega;
		double gammaCurrent = this->gammasState->getState()(it - VSCMGData.begin(), 0);
		it->gamma = gammaCurrent;
		outputStates.gimbalAngles[it - VSCMGData.begin()] = it->gamma;
		double gammaDotCurrent = this->gammaDotsState->getState()(it - VSCMGData.begin(), 0);
		it->gammaDot = gammaDotCurrent;
		outputStates.gimbalRates[it - VSCMGData.begin()] = it->gammaDot;

		tmpVSCMG.rWB_S = it->rWB_S;
		tmpVSCMG.gsHat_S = it->gsHat_S;
		tmpVSCMG.w2Hat0_S = it->w2Hat0_S;
		tmpVSCMG.w3Hat0_S = it->w3Hat0_S;
		tmpVSCMG.theta = it->theta;
		tmpVSCMG.u_current = it->u_current;
		tmpVSCMG.u_max = it->u_max;
		tmpVSCMG.u_min = it->u_min;
		tmpVSCMG.u_f = it->u_f;
		tmpVSCMG.Omega = it->Omega;
		tmpVSCMG.Omega_max = it->Omega_max;
		tmpVSCMG.gamma = it->gamma;
		tmpVSCMG.gammaDot = it->gammaDot;
		tmpVSCMG.Js = it->Js;
		tmpVSCMG.U_s = it->U_s;
		tmpVSCMG.U_d = it->U_d;
		tmpVSCMG.VSCMGModel = it->VSCMGModel;
		// Write out config data for eachreaction wheel
		messageSys->WriteMessage(this->vscmgOutMsgIds.at(it - VSCMGData.begin()),
								 CurrentClock,
								 sizeof(VSCMGConfigSimMsg),
								 reinterpret_cast<uint8_t*> (&tmpVSCMG),
								 moduleID);
	}

	// Write this message once for all reaction wheels
	messageSys->WriteMessage(StateOutMsgID, CurrentClock,
							 sizeof(VSCMGSpeedIntMsg), reinterpret_cast<uint8_t*> (&outputStates), moduleID);
}

/*! This method is used to read the incoming command message and set the
 associated command structure for operating the VSCMGs.
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
	memset(IncomingCmdBuffer.wheelTorque, 0x0, sizeof(VSCMGArrayTorqueIntMsg));
	SystemMessaging::GetInstance()->ReadMessage(CmdsInMsgID, &LocalHeader,
												sizeof(VSCMGArrayTorqueIntMsg),
												reinterpret_cast<uint8_t*> (&IncomingCmdBuffer), moduleID);

	//! - Check if message has already been read, if stale return
	//    if(prevCommandTime==LocalHeader.WriteClockNanos) {
	//        return;
	//    }
	prevCommandTime = LocalHeader.WriteClockNanos;

	//! - Set the NewVSCMGCmds vector.  Using the data() method for raw speed
	VSCMGCmdSimMsg *CmdPtr;
	for(i=0, CmdPtr = NewVSCMGCmds.data(); i<VSCMGData.size();
		CmdPtr++, i++)
	{
		CmdPtr->u_s_cmd = IncomingCmdBuffer.wheelTorque[i];
	}

}

///*! This method is used to read the new commands vector and set the VSCMG
// torque commands appropriately.  It assumes that the ReadInputs method has
// already been run successfully.
// @return void
// @param CurrentTime The current simulation time converted to a double
// */
void VSCMGStateEffector::ConfigureVSCMGRequests(double CurrentTime)
{
	//! Begin method steps
	std::vector<VSCMGCmdSimMsg>::iterator CmdIt;
	int vscmgIter = 0;
	double u_s;
	double omegaCritical;

	// loop through commands
	for(CmdIt=NewVSCMGCmds.begin(); CmdIt!=NewVSCMGCmds.end(); CmdIt++)
	{
		// saturation
		if (this->VSCMGData[vscmgIter].u_max > 0) {
			if(CmdIt->u_s_cmd > this->VSCMGData[vscmgIter].u_max) {
				CmdIt->u_s_cmd = this->VSCMGData[vscmgIter].u_max;
			} else if(CmdIt->u_s_cmd < -this->VSCMGData[vscmgIter].u_max) {
				CmdIt->u_s_cmd = -this->VSCMGData[vscmgIter].u_max;
			}
		}

		// minimum torque
		if( std::abs(CmdIt->u_s_cmd) < this->VSCMGData[vscmgIter].u_min) {
			CmdIt->u_s_cmd = 0.0;
		}

		// Coulomb friction
		if (this->VSCMGData[vscmgIter].linearFrictionRatio > 0.0) {
			omegaCritical = this->VSCMGData[vscmgIter].Omega_max * this->VSCMGData[vscmgIter].linearFrictionRatio;
		} else {
			omegaCritical = 0.0;
		}
		if(this->VSCMGData[vscmgIter].Omega > omegaCritical) {
			u_s = CmdIt->u_s_cmd - this->VSCMGData[vscmgIter].u_f;
		} else if(this->VSCMGData[vscmgIter].Omega < -omegaCritical) {
			u_s = CmdIt->u_s_cmd + this->VSCMGData[vscmgIter].u_f;
		} else {
			if (this->VSCMGData[vscmgIter].linearFrictionRatio > 0) {
				u_s = CmdIt->u_s_cmd - this->VSCMGData[vscmgIter].u_f*this->VSCMGData[vscmgIter].Omega/omegaCritical;
			} else {
				u_s = CmdIt->u_s_cmd;
			}
		}

		this->VSCMGData[vscmgIter].u_current = u_s; // save actual torque for reaction wheel motor

		vscmgIter++;

	}
}

/*! This method is the main cyclical call for the scheduled part of the VSCMG
 dynamics model.  It reads the current commands array and sets the VSCMG
 configuration data based on that incoming command set.  Note that the main
 dynamical method (ComputeDynamics()) is not called here and is intended to be
 called from the dynamics plant in the system
 @return void
 @param CurrentSimNanos The current simulation time in nanoseconds
 */
void VSCMGStateEffector::UpdateState(uint64_t CurrentSimNanos)
{
	//! Begin method steps
	//! - Read the inputs and then call ConfigureVSCMGRequests to set up dynamics
	ReadInputs();
	ConfigureVSCMGRequests(CurrentSimNanos*NANO2SEC);
	WriteOutputMessages(CurrentSimNanos);
//
}

