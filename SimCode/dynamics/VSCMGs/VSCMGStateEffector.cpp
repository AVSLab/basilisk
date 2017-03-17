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
	this->CallCounts = 0;
	this->InputCmds = "vscmg_cmds";
	this->OutputDataString = "vscmg_output_states";
	this->OutputBufferCount = 2;
	this->CmdsInMsgID = -1;
	this->StateOutMsgID = -1;
	this->prevCommandTime = 0xFFFFFFFFFFFFFFFF;

    this->effProps.mEff = 0.0;
    this->effProps.IEffPntB_B.setZero();
    this->effProps.rEff_CB_B.setZero();
	this->effProps.IEffPrimePntB_B.setZero();
	this->effProps.rEffPrime_CB_B.setZero();

    this->nameOfVSCMGOmegasState = "VSCMGOmegas";
    this->nameOfVSCMGThetasState = "VSCMGThetas";
	this->nameOfVSCMGGammasState = "VSCMGGammas";
	this->nameOfVSCMGGammaDotsState = "VSCMGGammaDots";

	std::vector<VSCMGConfigSimMsg>::iterator vscmgIt;
	for(vscmgIt=VSCMGData.begin(); vscmgIt!=VSCMGData.end(); vscmgIt++) {
		vscmgIt->theta = 0.0;
		vscmgIt->Omega = 0.0;
		vscmgIt->gamma = 0.0;
		vscmgIt->gammaDot = 0.0;
	}

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
    //! zero the Omega and theta values
    Eigen::MatrixXd omegasForInit(this->VSCMGData.size(),1);
	Eigen::MatrixXd gammasForInit(this->VSCMGData.size(),1);
	Eigen::MatrixXd gammaDotsForInit(this->VSCMGData.size(),1);

	std::vector<VSCMGConfigSimMsg>::iterator vscmgIt;
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
		vscmgIt->gamma = this->gammasState->getState()(vscmgIt - VSCMGData.begin(), 0);
		vscmgIt->gammaDot = this->gammaDotsState->getState()(vscmgIt - VSCMGData.begin(), 0);
		if (vscmgIt->VSCMGModel == JitterFullyCoupled || vscmgIt->VSCMGModel == JitterSimple) {
			vscmgIt->theta = this->thetasState->getState()(thetaCount, 0);
			thetaCount++;
		}

		if (vscmgIt->VSCMGModel == BalancedWheels || vscmgIt->VSCMGModel == JitterSimple) {

			Eigen::Matrix3d dcm_GG0 = eigenM3(vscmgIt->gamma);
			Eigen::Matrix3d dcm_BG0;
			dcm_BG0.col(0) = vscmgIt->gsHat0_B;
			dcm_BG0.col(1) = vscmgIt->gtHat0_B;
			dcm_BG0.col(2) = vscmgIt->ggHat_B;
			Eigen::Matrix3d dcm_BG = dcm_BG0 * dcm_GG0.transpose();
			vscmgIt->gsHat_B = dcm_BG.col(0);
			vscmgIt->gtHat_B = dcm_BG.col(1);

			if (vscmgIt->VSCMGModel == JitterSimple) {
				Eigen::Matrix3d dcm_WG = eigenM1(vscmgIt->theta);
				Eigen::Matrix3d dcm_WG0 = dcm_WG * dcm_GG0;
				Eigen::Matrix3d dcm_BW = dcm_BG0 * dcm_WG0.transpose();
				vscmgIt->w2Hat_B = dcm_BW.col(1);
				vscmgIt->w3Hat_B = dcm_BW.col(2);
			}

			//! wheel inertia tensor about wheel center of mass represented in B frame
			Eigen::Matrix3d IRWPntWc_W;
			IRWPntWc_W << vscmgIt->IW1, 0., 0., \
							0., vscmgIt->IW2, 0., \
							0., 0., vscmgIt->IW3;
			vscmgIt->IRWPntWc_B = dcm_BG * IRWPntWc_W * dcm_BG.transpose();

			//! wheel inertia tensor body frame derivative about wheel center of mass represented in B frame
			Eigen::Matrix3d IPrimeRWPntWc_W;
			IPrimeRWPntWc_W(0,0) = 0.0;
			IPrimeRWPntWc_W(0,1) = vscmgIt->gammaDot*(vscmgIt->IW1-vscmgIt->IW2);
			IPrimeRWPntWc_W(0,2) = 0.0;
			IPrimeRWPntWc_W(1,0) = IPrimeRWPntWc_W(0,1);
			IPrimeRWPntWc_W(1,1) = 0.0;
			IPrimeRWPntWc_W(1,2) = 0.0;
			IPrimeRWPntWc_W(2,0) = 0.0;
			IPrimeRWPntWc_W(2,1) = 0.0;
			IPrimeRWPntWc_W(2,2) = 0.0;
			vscmgIt->IPrimeRWPntWc_B = dcm_BG * IPrimeRWPntWc_W * dcm_BG.transpose();

			//! gimbal inertia tensor about wheel center of mass represented in B frame
			Eigen::Matrix3d IGIMPntGc_G;
			IGIMPntGc_G << vscmgIt->IG1, 0., 0., \
							0., vscmgIt->IG2, 0., \
							0., 0., vscmgIt->IG3;
			vscmgIt->IGIMPntGc_B = dcm_BG * IGIMPntGc_G * dcm_BG.transpose();

			//! gimbal inertia tensor body frame derivative about gimbal center of mass represented in B frame
			Eigen::Matrix3d IPrimeGIMPntGc_G;
			IPrimeGIMPntGc_G << 0., vscmgIt->gammaDot*(vscmgIt->IG1-vscmgIt->IG2), 0., \
							vscmgIt->gammaDot*(vscmgIt->IG1-vscmgIt->IG2), 0., 0., \
							0., 0., 0.;
			vscmgIt->IPrimeGIMPntGc_B = dcm_BG * IPrimeGIMPntGc_G * dcm_BG.transpose();

			//! wheel center of mass location
			vscmgIt->rWcB_B = vscmgIt->rWB_B;
			vscmgIt->rTildeWcB_B = eigenTilde(vscmgIt->rWcB_B);
			vscmgIt->rPrimeWcB_B.setZero();
			Eigen::Matrix3d rPrimeTildeWcB_B = eigenTilde(vscmgIt->rPrimeWcB_B);

			//! - Give the mass of the VSCMG to the effProps mass
			this->effProps.mEff += vscmgIt->massV;
			this->effProps.rEff_CB_B += vscmgIt->massV*vscmgIt->rWB_B;
			this->effProps.IEffPntB_B += vscmgIt->IRWPntWc_B + vscmgIt->IGIMPntGc_B + vscmgIt->massV*vscmgIt->rTildeWcB_B*vscmgIt->rTildeWcB_B.transpose();
			this->effProps.rEffPrime_CB_B += vscmgIt->massV*vscmgIt->rPrimeWcB_B;
			this->effProps.IEffPrimePntB_B += vscmgIt->IPrimeRWPntWc_B + vscmgIt->IPrimeGIMPntGc_B;

		} else if (vscmgIt->VSCMGModel == JitterFullyCoupled) {
//			Eigen::Matrix3d dcm_WG = eigenM1(vscmgIt->theta);
//			Eigen::Matrix3d dcm_WG0 = dcm_WG * dcm_GG0;
//			Eigen::Matrix3d dcm_BW = dcm_BG0 * dcm_WG0.transpose();
//			vscmgIt->w2Hat_B = dcm_BW.col(1);
//			vscmgIt->w3Hat_B = dcm_BW.col(2);

//			//! gimbal inertia tensor about wheel center of mass represented in B frame
//			Eigen::Matrix3d IGIMPntGc_G;
//			IGIMPntGc_G << vscmgIt->IG1, vscmgIt->IG12, vscmgIt->IG13, \
//			vscmgIt->IG12, vscmgIt->IG2, vscmgIt->IG23, \
//			vscmgIt->IG13, vscmgIt->IG23, vscmgIt->IG3;
//			vscmgIt->IGIMPntGc_B = dcm_BG * IGIMPntGc_G * dcm_BG.transpose();

//			//! gimbal inertia tensor body frame derivative about gimbal center of mass represented in B frame
//			Eigen::Matrix3d IPrimeGIMPntGc_G;
//			IPrimeGIMPntGc_G << -2*vscmgIt->IG12, (vscmgIt->IG1-vscmgIt->IG2), -vscmgIt->IG23, \
//			(vscmgIt->IG1-vscmgIt->IG2), 2*vscmgIt->IG12, vscmgIt->IG13, \
//			-vscmgIt->IG23, vscmgIt->IG13, 0.;
//			vscmgIt->IPrimeGIMPntGc_B = dcm_BG * IPrimeGIMPntGc_G * dcm_BG.transpose();

		}

	}

    // - Need to divide out the total mass of the VSCMGs from rCB_B and rPrimeCB_B
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
	double omegat;
	double omegag;
	double dSquared;
	double OmegaSquared;
	Eigen::Matrix3d omegaTilde;
    Eigen::MRPd sigmaBNLocal;
    Eigen::Matrix3d dcm_BN;                        /* direction cosine matrix from N to B */
    Eigen::Matrix3d dcm_NB;                        /* direction cosine matrix from B to N */
    Eigen::Vector3d gravityTorquePntW_B;          /* torque of gravity on HRB about Pnt H */
    Eigen::Vector3d gLocal_N;                          /* gravitational acceleration in N frame */
    Eigen::Vector3d g_B;                          /* gravitational acceleration in B frame */
	Eigen::Vector3d omega_WB_B;
    gLocal_N = *this->g_N;
	Eigen::Vector3d vecRotcontr_temp;
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
		omegaTilde = eigenTilde(omegaLoc_BN_B);
		omega_WB_B = vscmgIt->gammaDot*vscmgIt->ggHat_B+vscmgIt->Omega*vscmgIt->gsHat_B;
		omegas = vscmgIt->gsHat_B.transpose()*omegaLoc_BN_B;
		omegat = vscmgIt->gtHat_B.transpose()*omegaLoc_BN_B;
		omegag = vscmgIt->ggHat_B.transpose()*omegaLoc_BN_B;

		if (vscmgIt->VSCMGModel == BalancedWheels || vscmgIt->VSCMGModel == JitterSimple) {
			matrixDcontr -= vscmgIt->IV3 * vscmgIt->ggHat_B * vscmgIt->ggHat_B.transpose() + vscmgIt->IW1 * vscmgIt->gsHat_B * vscmgIt->gsHat_B.transpose();
			vecRotcontr_temp = (vscmgIt->u_s_current-vscmgIt->IW1*omegat*vscmgIt->gammaDot)*vscmgIt->gsHat_B + vscmgIt->IW1*vscmgIt->Omega*vscmgIt->gammaDot*vscmgIt->gtHat_B + (vscmgIt->u_g_current+(vscmgIt->IV1-vscmgIt->IV2)*omegas*omegat+vscmgIt->IW1*vscmgIt->Omega*omegat)*vscmgIt->ggHat_B
							+ omegaTilde*vscmgIt->IGIMPntGc_B*vscmgIt->gammaDot*vscmgIt->ggHat_B + omegaTilde*vscmgIt->IRWPntWc_B*omega_WB_B;
			vecRotcontr -= vecRotcontr_temp;

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

//            gravityTorquePntW_B = vscmgIt->d*vscmgIt->w2Hat_B.cross(vscmgIt->mass*g_B);
//
//			dSquared = vscmgIt->d * vscmgIt->d;
//
//			vscmgIt->aOmega = -vscmgIt->mass*vscmgIt->d/(vscmgIt->IW1 + vscmgIt->mass*dSquared) * vscmgIt->w3Hat_B;
//			vscmgIt->bOmega = -1.0/(vscmgIt->IW1 + vscmgIt->mass*dSquared)*((vscmgIt->IW1+vscmgIt->mass*dSquared)*vscmgIt->gsHat_B + vscmgIt->IW13*vscmgIt->w3Hat_B + vscmgIt->mass*vscmgIt->d*vscmgIt->rWB_B.cross(vscmgIt->w3Hat_B));
//			vscmgIt->cOmega = 1.0/(vscmgIt->IW1 + vscmgIt->mass*dSquared)*(omegaw2*omegaw3*(-vscmgIt->mass*dSquared)-vscmgIt->IW13*omegaw2*omegas-vscmgIt->mass*vscmgIt->d*vscmgIt->w3Hat_B.transpose()*omegaLoc_BN_B.cross(omegaLoc_BN_B.cross(vscmgIt->rWB_B))+vscmgIt->u_s_current + vscmgIt->gsHat_B.dot(gravityTorquePntW_B));
//
//			matrixAcontr += vscmgIt->mass * vscmgIt->d * vscmgIt->w3Hat_B * vscmgIt->aOmega.transpose();
//			matrixBcontr += vscmgIt->mass * vscmgIt->d * vscmgIt->w3Hat_B * vscmgIt->bOmega.transpose();
//			matrixCcontr += (vscmgIt->IRWPntWc_B*vscmgIt->gsHat_B + vscmgIt->mass*vscmgIt->d*vscmgIt->rWcB_B.cross(vscmgIt->w3Hat_B))*vscmgIt->aOmega.transpose();
//			matrixDcontr += (vscmgIt->IRWPntWc_B*vscmgIt->gsHat_B + vscmgIt->mass*vscmgIt->d*vscmgIt->rWcB_B.cross(vscmgIt->w3Hat_B))*vscmgIt->bOmega.transpose();
//			vecTranscontr += vscmgIt->mass*vscmgIt->d*(OmegaSquared*vscmgIt->w2Hat_B - vscmgIt->cOmega*vscmgIt->w3Hat_B);
//			vecRotcontr += vscmgIt->mass*vscmgIt->d*OmegaSquared*vscmgIt->rWcB_B.cross(vscmgIt->w2Hat_B) - vscmgIt->IPrimeRWPntWc_B*vscmgIt->Omega*vscmgIt->gsHat_B - omegaLoc_BN_B.cross(vscmgIt->IRWPntWc_B*vscmgIt->Omega*vscmgIt->gsHat_B+vscmgIt->mass*vscmgIt->rWcB_B.cross(vscmgIt->rPrimeWcB_B)) - (vscmgIt->IRWPntWc_B*vscmgIt->gsHat_B+vscmgIt->mass*vscmgIt->d*vscmgIt->rWcB_B.cross(vscmgIt->w3Hat_B))*vscmgIt->cOmega;
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
	Eigen::Vector3d omegaLoc_BN_B;
	Eigen::MRPd sigmaBNLocal;
	Eigen::Matrix3d dcm_BN;                        /* direction cosine matrix from N to B */
	Eigen::Matrix3d dcm_NB;                        /* direction cosine matrix from B to N */
	Eigen::Vector3d rDDotBNLoc_N;                 /* second time derivative of rBN in N frame */
	Eigen::Vector3d rDDotBNLoc_B;                 /* second time derivative of rBN in B frame */
	double omegas;
	double omegat;
	double omegag;
	int VSCMGi = 0;
    int thetaCount = 0;
	std::vector<VSCMGConfigSimMsg>::iterator vscmgIt;

	//! Grab necessarry values from manager
	omegaDotBNLoc_B = this->hubOmega->getStateDeriv();
	omegaLoc_BN_B = this->hubOmega->getState();
	rDDotBNLoc_N = this->hubVelocity->getStateDeriv();
	sigmaBNLocal = (Eigen::Vector3d )this->hubSigma->getState();
	dcm_NB = sigmaBNLocal.toRotationMatrix();
	dcm_BN = dcm_NB.transpose();
	rDDotBNLoc_B = dcm_BN*rDDotBNLoc_N;

	//! - Compute Derivatives
	for(vscmgIt=VSCMGData.begin(); vscmgIt!=VSCMGData.end(); vscmgIt++)
	{
		omegas = vscmgIt->gsHat_B.transpose()*omegaLoc_BN_B;
		omegat = vscmgIt->gtHat_B.transpose()*omegaLoc_BN_B;
		omegag = vscmgIt->ggHat_B.transpose()*omegaLoc_BN_B;

		gammasDot(VSCMGi,0) = vscmgIt->gammaDot;
        if(vscmgIt->VSCMGModel == JitterFullyCoupled || vscmgIt->VSCMGModel == JitterSimple) {
            // - Set trivial kinemetic derivative
            thetasDot(thetaCount,0) = vscmgIt->Omega;
            thetaCount++;
        }
		if (vscmgIt->VSCMGModel == BalancedWheels || vscmgIt->VSCMGModel == JitterSimple) {
			OmegasDot(VSCMGi,0) =  - omegat*vscmgIt->gammaDot - vscmgIt->gsHat_B.transpose()*omegaDotBNLoc_B + vscmgIt->u_s_current/vscmgIt->IW1;
			gammaDotsDot(VSCMGi,0) = 1/vscmgIt->IV3*(vscmgIt->u_g_current+(vscmgIt->IV1-vscmgIt->IV2)*omegas*omegat+vscmgIt->IW1*vscmgIt->Omega*omegat-vscmgIt->IV3*vscmgIt->ggHat_B.transpose()*omegaDotBNLoc_B);
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
		Eigen::Vector3d omega_GN_B = omegaLoc_BN_B + vscmgIt->gammaDot*vscmgIt->ggHat_B;
		Eigen::Vector3d omega_WN_B = omega_GN_B + vscmgIt->Omega*vscmgIt->gsHat_B;
		if (vscmgIt->VSCMGModel == BalancedWheels || vscmgIt->VSCMGModel == JitterSimple) {
			vscmgIt->rWcB_B = vscmgIt->rWB_B;
			Eigen::Vector3d rDotWcB_B = omegaLoc_BN_B.cross(vscmgIt->rWcB_B);
			rotAngMomPntCContr_B += vscmgIt->IRWPntWc_B*omega_WN_B + vscmgIt->IGIMPntGc_B*omega_GN_B + vscmgIt->massV*vscmgIt->rWcB_B.cross(rDotWcB_B);
			rotEnergyContr += 1.0/2.0*omega_WN_B.dot(vscmgIt->IRWPntWc_B*omega_WN_B) + 1.0/2.0*omega_GN_B.dot(vscmgIt->IGIMPntGc_B*omega_GN_B) + 1.0/2.0*vscmgIt->massV*rDotWcB_B.dot(rDotWcB_B);
		} else if (vscmgIt->VSCMGModel == JitterFullyCoupled) {
//			Eigen::Vector3d r_WcB_B = vscmgIt->rWcB_B;
//			Eigen::Vector3d rDot_WcB_B = vscmgIt->d*vscmgIt->Omega*vscmgIt->w3Hat_B + omegaLoc_BN_B.cross(vscmgIt->rWcB_B);
//			rotAngMomPntCContr_B += vscmgIt->IRWPntWc_B*omega_WN_B + vscmgIt->mass*r_WcB_B.cross(rDot_WcB_B);
//			rotEnergyContr += 0.5*omega_WN_B.transpose()*vscmgIt->IRWPntWc_B*omega_WN_B + 0.5*vscmgIt->mass*rDot_WcB_B.dot(rDot_WcB_B);
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
	VSCMGCmdInitializer.u_g_cmd = 0.0;

	//! Begin method steps
	//! - Clear out any currently firing VSCMGs and re-init cmd array
	NewVSCMGCmds.clear();
	NewVSCMGCmds.insert(NewVSCMGCmds.begin(), VSCMGData.size(), VSCMGCmdInitializer );

	// Reserve a message ID for each VSCMG config output message
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
		if (it->gsHat0_S.norm() > 0.01) {
			it->gsHat0_B = dcm_BS * it->gsHat0_S;
		} else {
			std::cerr <<
			"Error: gsHat0_S not properly initialized.  Don't set gsHat_B directly in python.";
		}
		if (it->gtHat0_S.norm() > 0.01) {
			it->gtHat0_B = dcm_BS * it->gtHat0_S;
		} else {
			std::cerr <<
			"Error: gtHat0_S not properly initialized.  Don't set gtHat_B directly in python.";
		}
		if (it->ggHat_S.norm() > 0.01) {
			it->ggHat_B = dcm_BS * it->ggHat_S;
		} else {
			std::cerr <<
			"Error: ggHat_S not properly initialized.  Don't set ggHat_B directly in python.";
		}

		it->w2Hat0_B = it->gtHat0_B;
		it->w3Hat0_B = it->ggHat_B;

		//! Define CoM offset d and off-diagonal inertia IW13 if using fully coupled model
		if (it->VSCMGModel == JitterFullyCoupled) {
			it->d = it->U_s/it->massW; //!< determine CoM offset from static imbalance parameter
			it->IW13 = it->U_d; //!< off-diagonal inertia is equal to dynamic imbalance parameter
		}
		if (it->VSCMGModel == BalancedWheels || it->VSCMGModel == JitterSimple) {
			it->IV1 = it->IW1 + it->IG1;
			it->IV2 = it->IW2 + it->IG2;
			it->IV3 = it->IW3 + it->IG3;
			it->IG12 = 0.;
			it->IG13 = 0.;
			it->IG23 = 0.;
			it->IW13 = 0.;
		}

		it->rWB_B = dcm_BS * it->rWB_S;
		it->massV = it->massG + it->massW;
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
		tmpVSCMG.gsHat0_S = it->gsHat0_S;
		tmpVSCMG.u_s_current = it->u_s_current;
		tmpVSCMG.u_s_max = it->u_s_max;
		tmpVSCMG.u_s_min = it->u_s_min;
		tmpVSCMG.u_s_f = it->u_s_f;
		tmpVSCMG.u_g_current = it->u_g_current;
		tmpVSCMG.u_g_max = it->u_g_max;
		tmpVSCMG.u_g_min = it->u_g_min;
		tmpVSCMG.u_g_f = it->u_g_f;
		tmpVSCMG.theta = it->theta;
		tmpVSCMG.Omega = it->Omega;
		tmpVSCMG.Omega_max = it->Omega_max;
		tmpVSCMG.gamma = it->gamma;
		tmpVSCMG.gammaDot = it->gammaDot;
		tmpVSCMG.gammaDot_max = it->gammaDot_max;
		tmpVSCMG.IW1 = it->IW1;
		tmpVSCMG.U_s = it->U_s;
		tmpVSCMG.U_d = it->U_d;
		tmpVSCMG.VSCMGModel = it->VSCMGModel;
		// Write out config data for each VSCMG
		messageSys->WriteMessage(this->vscmgOutMsgIds.at(it - VSCMGData.begin()),
								 CurrentClock,
								 sizeof(VSCMGConfigSimMsg),
								 reinterpret_cast<uint8_t*> (&tmpVSCMG),
								 moduleID);
	}

	// Write this message once for all VSCMGs
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
	memset(IncomingCmdBuffer.wheelTorque, 0x0, sizeof(VSCMGArrayTorqueIntMsg)/2);
	memset(IncomingCmdBuffer.gimbalTorque, 0x0, sizeof(VSCMGArrayTorqueIntMsg)/2);
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
		CmdPtr->u_g_cmd = IncomingCmdBuffer.gimbalTorque[i];
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
	int vscmgIt = 0;
	double u_s;
	double u_g;
	double omegaCritical;
	double gammaDotCritical;

	// loop through commands
	for(CmdIt=NewVSCMGCmds.begin(); CmdIt!=NewVSCMGCmds.end(); CmdIt++)
	{
		//! wheel torque saturation
		//! set u_s_max to less than zero to disable saturation
		if (this->VSCMGData[vscmgIt].u_s_max > 0.0) {
			if(CmdIt->u_s_cmd > this->VSCMGData[vscmgIt].u_s_max) {
				CmdIt->u_s_cmd = this->VSCMGData[vscmgIt].u_s_max;
			} else if(CmdIt->u_s_cmd < -this->VSCMGData[vscmgIt].u_s_max) {
				CmdIt->u_s_cmd = -this->VSCMGData[vscmgIt].u_s_max;
			}
		}

		//! gimbal torque saturation
		//! set u_g_max to less than zero to disable saturation
		if (this->VSCMGData[vscmgIt].u_g_max > 0.0) {
			if(CmdIt->u_g_cmd > this->VSCMGData[vscmgIt].u_g_max) {
				CmdIt->u_g_cmd = this->VSCMGData[vscmgIt].u_g_max;
			} else if(CmdIt->u_g_cmd < -this->VSCMGData[vscmgIt].u_g_max) {
				CmdIt->u_g_cmd = -this->VSCMGData[vscmgIt].u_g_max;
			}
		}

		//! minimum wheel torque
		//! set u_s_min to less than zero to disable minimum torque
		if( std::abs(CmdIt->u_s_cmd) < this->VSCMGData[vscmgIt].u_s_min) {
			CmdIt->u_s_cmd = 0.0;
		}

		//! minimum gimbal torque
		//! set u_g_min to less than zero to disable minimum torque
		if( std::abs(CmdIt->u_g_cmd) < this->VSCMGData[vscmgIt].u_g_min) {
			CmdIt->u_g_cmd = 0.0;
		}

		//! wheel Coulomb friction
		//! set wheelLinearFrictionRatio to less than zero to disable linear friction
		//! set u_s_f to zero to disable all friction
		if (this->VSCMGData[vscmgIt].wheelLinearFrictionRatio > 0.0) {
			omegaCritical = this->VSCMGData[vscmgIt].Omega_max * this->VSCMGData[vscmgIt].wheelLinearFrictionRatio;
		} else {
			omegaCritical = 0.0;
		}
		if(this->VSCMGData[vscmgIt].Omega > omegaCritical) {
			u_s = CmdIt->u_s_cmd - this->VSCMGData[vscmgIt].u_s_f;
		} else if(this->VSCMGData[vscmgIt].Omega < -omegaCritical) {
			u_s = CmdIt->u_s_cmd + this->VSCMGData[vscmgIt].u_s_f;
		} else {
			if (this->VSCMGData[vscmgIt].wheelLinearFrictionRatio > 0) {
				u_s = CmdIt->u_s_cmd - this->VSCMGData[vscmgIt].u_s_f*this->VSCMGData[vscmgIt].Omega/omegaCritical;
			} else {
				u_s = CmdIt->u_s_cmd;
			}
		}

		//! gimbal Coulomb friction
		//! set gimbalLinearFrictionRatio to less than zero to disable linear friction
		//! set u_g_f to zero to disable friction
		if (this->VSCMGData[vscmgIt].gimbalLinearFrictionRatio > 0.0) {
			gammaDotCritical = this->VSCMGData[vscmgIt].gammaDot_max * this->VSCMGData[vscmgIt].wheelLinearFrictionRatio;
		} else {
			gammaDotCritical = 0.0;
		}
		if(this->VSCMGData[vscmgIt].gammaDot > gammaDotCritical) {
			u_g = CmdIt->u_g_cmd - this->VSCMGData[vscmgIt].u_g_f;
		} else if(this->VSCMGData[vscmgIt].gammaDot < -gammaDotCritical) {
			u_g = CmdIt->u_g_cmd + this->VSCMGData[vscmgIt].u_g_f;
		} else {
			if (this->VSCMGData[vscmgIt].gimbalLinearFrictionRatio > 0) {
				u_g = CmdIt->u_g_cmd - this->VSCMGData[vscmgIt].u_g_f*this->VSCMGData[vscmgIt].gammaDot/gammaDotCritical;
			} else {
				u_g = CmdIt->u_g_cmd;
			}
		}

		this->VSCMGData[vscmgIt].u_s_current = u_s; // save actual torque for wheel motor
		this->VSCMGData[vscmgIt].u_g_current = u_g; // save actual torque for wheel motor

		vscmgIt++;

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

