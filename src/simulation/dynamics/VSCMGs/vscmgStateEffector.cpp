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


#include "vscmgStateEffector.h"
#include "architecture/messaging/system_messaging.h"
#include <cstring>
#include <iostream>
#include <cmath>
#include "utilities/bsk_Print.h"

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

	std::vector<VSCMGConfigSimMsg>::iterator it;
	for(it=VSCMGData.begin(); it!=VSCMGData.end(); it++) {
		it->theta = 0.0;
		it->Omega = 0.0;
		it->gamma = 0.0;
		it->gammaDot = 0.0;
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

	std::vector<VSCMGConfigSimMsg>::iterator it;
    for(it=VSCMGData.begin(); it!=VSCMGData.end(); it++) {
        if (it->VSCMGModel == vscmgJitterSimple || it->VSCMGModel == vscmgJitterFullyCoupled) {
            this->numVSCMGJitter++;
        }
        omegasForInit(it - this->VSCMGData.begin(), 0) = it->Omega;
		gammasForInit(it - this->VSCMGData.begin(), 0) = it->gamma;
		gammaDotsForInit(it - this->VSCMGData.begin(), 0) = it->gammaDot;
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
    std::vector<VSCMGConfigSimMsg>::iterator it;
	for(it=VSCMGData.begin(); it!=VSCMGData.end(); it++)
	{
		it->Omega = this->OmegasState->getState()(it - VSCMGData.begin(), 0);
		it->gamma = this->gammasState->getState()(it - VSCMGData.begin(), 0);
		it->gammaDot = this->gammaDotsState->getState()(it - VSCMGData.begin(), 0);
		if (it->VSCMGModel == vscmgJitterFullyCoupled || it->VSCMGModel == vscmgJitterSimple) {
			it->theta = this->thetasState->getState()(thetaCount, 0);
			thetaCount++;
		}

		Eigen::Matrix3d dcm_GG0 = eigenM3(it->gamma);
		Eigen::Matrix3d dcm_BG0;
		dcm_BG0.col(0) = it->gsHat0_B;
		dcm_BG0.col(1) = it->gtHat0_B;
		dcm_BG0.col(2) = it->ggHat_B;
		Eigen::Matrix3d dcm_BG = dcm_BG0 * dcm_GG0.transpose();
		it->gsHat_B = dcm_BG.col(0);
		it->gtHat_B = dcm_BG.col(1);

		Eigen::Matrix3d dcm_WG = eigenM1(it->theta);
		Eigen::Matrix3d dcm_WG0 = dcm_WG * dcm_GG0;
		Eigen::Matrix3d dcm_BW = dcm_BG0 * dcm_WG0.transpose();
		it->w2Hat_B = dcm_BW.col(1);
		it->w3Hat_B = dcm_BW.col(2);

		if (it->VSCMGModel == vscmgBalancedWheels || it->VSCMGModel == vscmgJitterSimple) {

			//! gimbal inertia tensor about wheel center of mass represented in B frame
			Eigen::Matrix3d IGIMPntGc_G;
			IGIMPntGc_G << it->IG1, 0., 0., \
							0., it->IG2, 0., \
							0., 0., it->IG3;
			it->IGPntGc_B = dcm_BG * IGIMPntGc_G * dcm_BG.transpose();

			//! gimbal inertia tensor body frame derivative about gimbal center of mass represented in B frame
			Eigen::Matrix3d IPrimeGIMPntGc_G;
			IPrimeGIMPntGc_G << 0., it->gammaDot*(it->IG1-it->IG2), 0., \
								it->gammaDot*(it->IG1-it->IG2), 0., 0., \
								0., 0., 0.;
			it->IPrimeGPntGc_B = dcm_BG * IPrimeGIMPntGc_G * dcm_BG.transpose();

			//! wheel inertia tensor about wheel center of mass represented in B frame
			Eigen::Matrix3d IRWPntWc_W;
			IRWPntWc_W << it->IW1, 0., 0., \
							0., it->IW2, 0., \
							0., 0., it->IW3;
			it->IWPntWc_B = dcm_BG * IRWPntWc_W * dcm_BG.transpose();

			//! wheel inertia tensor body frame derivative about wheel center of mass represented in B frame
			Eigen::Matrix3d IPrimeRWPntWc_W;
			IPrimeRWPntWc_W(0,0) = 0.0;
			IPrimeRWPntWc_W(0,1) = it->gammaDot*(it->IW1-it->IW2);
			IPrimeRWPntWc_W(0,2) = 0.0;
			IPrimeRWPntWc_W(1,0) = IPrimeRWPntWc_W(0,1);
			IPrimeRWPntWc_W(1,1) = 0.0;
			IPrimeRWPntWc_W(1,2) = 0.0;
			IPrimeRWPntWc_W(2,0) = 0.0;
			IPrimeRWPntWc_W(2,1) = 0.0;
			IPrimeRWPntWc_W(2,2) = 0.0;
			it->IPrimeWPntWc_B = dcm_BG * IPrimeRWPntWc_W * dcm_BG.transpose();

			//! VSCMG center of mass location
			//! Note that points W, G, Wc, Gc are coincident
			it->rWcB_B = it->rGB_B;
			it->rTildeWcB_B = eigenTilde(it->rWcB_B);
			it->rPrimeWcB_B.setZero();

			//! - Give the mass of the VSCMG to the effProps mass
			this->effProps.mEff += it->massV;
			this->effProps.rEff_CB_B += it->massV*it->rGB_B;
			this->effProps.IEffPntB_B += it->IWPntWc_B + it->IGPntGc_B + it->massV*it->rTildeWcB_B*it->rTildeWcB_B.transpose();
			this->effProps.rEffPrime_CB_B += it->massV*it->rPrimeWcB_B;
			this->effProps.IEffPrimePntB_B += it->IPrimeWPntWc_B + it->IPrimeGPntGc_B;

		} else if (it->VSCMGModel == vscmgJitterFullyCoupled) {

			it->rGcG_B = dcm_BG * it->rGcG_G;

			//! gimbal inertia tensor about wheel center of mass represented in B frame
			Eigen::Matrix3d IGIMPntGc_G;
			IGIMPntGc_G << it->IG1, it->IG12, it->IG13, \
							it->IG12, it->IG2, it->IG23, \
							it->IG13, it->IG23, it->IG3;
			it->IGPntGc_B = dcm_BG * IGIMPntGc_G * dcm_BG.transpose();

			//! gimbal inertia tensor body frame derivative about gimbal center of mass represented in B frame
			Eigen::Matrix3d IPrimeGIMPntGc_G;
			IPrimeGIMPntGc_G << -2*it->IG12, (it->IG1-it->IG2), -it->IG23, \
								(it->IG1-it->IG2), 2*it->IG12, it->IG13, \
								-it->IG23, it->IG13, 0.;
			IPrimeGIMPntGc_G *= it->gammaDot;
			it->IPrimeGPntGc_B = dcm_BG * IPrimeGIMPntGc_G * dcm_BG.transpose();

			//! wheel inertia tensor about wheel center of mass represented in B frame
			Eigen::Matrix3d IRWPntWc_W;
			IRWPntWc_W << it->IW1, 0., it->IW13, \
							0., it->IW2, 0., \
							it->IW13, 0., it->IW3;
			it->IWPntWc_B = dcm_BW * IRWPntWc_W * dcm_BW.transpose();

			//! wheel inertia tensor body frame derivative about wheel center of mass represented in B frame
			double Ia = it->IW1 - it->IW2;
			double Ib = it->IW3 - it->IW1;
			double Ic = it->IW2 - it->IW3;
			Eigen::Matrix3d IPrimeRWPntWc_W;
			IPrimeRWPntWc_W(0,0) = 2.0*it->gammaDot*it->IW13*sin(it->theta);
			IPrimeRWPntWc_W(0,1) = it->gammaDot*Ia*cos(it->theta)-it->IW13*it->Omega;
			IPrimeRWPntWc_W(0,2) = it->gammaDot*Ib*sin(it->theta);
			IPrimeRWPntWc_W(1,0) = IPrimeRWPntWc_W(0,1);
			IPrimeRWPntWc_W(1,1) = 0.0;
			IPrimeRWPntWc_W(1,2) = it->gammaDot*it->IW13*cos(it->theta)+Ic*it->Omega;
			IPrimeRWPntWc_W(2,0) = IPrimeRWPntWc_W(0,2);
			IPrimeRWPntWc_W(2,1) = IPrimeRWPntWc_W(1,2);
			IPrimeRWPntWc_W(2,2) = -2.0*it->IW13*it->gammaDot*sin(it->theta);
			it->IPrimeWPntWc_B = dcm_BW * IPrimeRWPntWc_W * dcm_BW.transpose();

			//! VSCMG center of mass location
			it->rGcB_B = it->rGB_B + it->rGcG_B;
			it->rTildeGcB_B = eigenTilde(it->rGcB_B);
			it->rPrimeGcB_B = it->gammaDot*it->ggHat_B.cross(it->rGcG_B);
			it->rPrimeTildeGcB_B = eigenTilde(it->rPrimeGcB_B);
			it->rWcG_B = it->L*it->ggHat_B + it->l*it->gsHat_B + it->d*it->w2Hat_B;
			it->rWcB_B = it->rGB_B + it->rWcG_B;
			it->rTildeWcB_B = eigenTilde(it->rWcB_B);
			it->rPrimeWcB_B = it->d*it->Omega*it->w3Hat_B - it->d*it->gammaDot*cos(it->theta)*it->gsHat_B + it->l*it->gammaDot*it->gtHat_B;
			it->rPrimeTildeWcB_B = eigenTilde(it->rPrimeWcB_B);

			//! - Give the mass of the VSCMG to the effProps mass
			this->effProps.mEff += it->massV;
			this->effProps.rEff_CB_B += it->massG*it->rGcB_B + it->massW*it->rWcB_B;
			this->effProps.IEffPntB_B += it->IWPntWc_B + it->IGPntGc_B + it->massG*it->rTildeGcB_B*it->rTildeGcB_B.transpose() + it->massW*it->rTildeWcB_B*it->rTildeWcB_B.transpose();
			this->effProps.rEffPrime_CB_B += it->massG*it->rPrimeGcB_B + it->massW*it->rPrimeWcB_B;
			this->effProps.IEffPrimePntB_B += it->IPrimeGPntGc_B + it->massG*it->rPrimeTildeGcB_B*it->rTildeGcB_B.transpose() + it->massG*it->rTildeGcB_B*it->rPrimeTildeGcB_B.transpose()
											+ it->IPrimeWPntWc_B + it->massW*it->rPrimeTildeWcB_B*it->rTildeWcB_B.transpose() + it->massW*it->rTildeWcB_B*it->rPrimeTildeWcB_B.transpose();
		}
	}

    // - Need to divide out the total mass of the VSCMGs from rCB_B and rPrimeCB_B
    if (this->effProps.mEff > 0) {
        this->effProps.rEff_CB_B /= this->effProps.mEff;
        this->effProps.rEffPrime_CB_B /= this->effProps.mEff;
    }

	return;
}

void VSCMGStateEffector::updateContributions(double integTime, BackSubMatrices & backSubContr, Eigen::Vector3d sigma_BN, Eigen::Vector3d omega_BN_B, Eigen::Vector3d g_N)
{
	Eigen::Vector3d omegaLoc_BN_B;
	Eigen::Vector3d tempF;
	double omegas;
	double omegat;
	double omegag;
	double OmegaSquared;
	Eigen::Matrix3d omegaTilde;
    Eigen::MRPd sigmaBNLocal;
    Eigen::Matrix3d dcm_BN;                        /* direction cosine matrix from N to B */
    Eigen::Matrix3d dcm_NB;                        /* direction cosine matrix from B to N */
    Eigen::Vector3d gravityTorquePntW_B;          /* torque of gravity on HRB about Pnt H */
	Eigen::Vector3d gravityTorquePntG_B;          /* torque of gravity on HRB about Pnt H */
    Eigen::Vector3d gLocal_N;                          /* gravitational acceleration in N frame */
    Eigen::Vector3d g_B;                          /* gravitational acceleration in B frame */
	Eigen::Vector3d omega_WB_B;
    gLocal_N = *this->g_N;

	Eigen::Vector3d ur;
	Eigen::Vector3d vr;
	Eigen::Vector3d kr;
	Eigen::Vector3d uomega;
	Eigen::Vector3d vomega;
	Eigen::Vector3d komega;


    //! - Find dcm_BN
    sigmaBNLocal = (Eigen::Vector3d )this->hubSigma->getState();
    dcm_NB = sigmaBNLocal.toRotationMatrix();
    dcm_BN = dcm_NB.transpose();
    //! - Map gravity to body frame
    g_B = dcm_BN*gLocal_N;

	omegaLoc_BN_B = this->hubOmega->getState();

    std::vector<VSCMGConfigSimMsg>::iterator it;
	for(it=VSCMGData.begin(); it!=VSCMGData.end(); it++)
	{
		OmegaSquared = it->Omega * it->Omega;
		omegaTilde = eigenTilde(omegaLoc_BN_B);
		omega_WB_B = it->gammaDot*it->ggHat_B+it->Omega*it->gsHat_B;
		omegas = it->gsHat_B.transpose()*omegaLoc_BN_B;
		omegat = it->gtHat_B.transpose()*omegaLoc_BN_B;
		omegag = it->ggHat_B.transpose()*omegaLoc_BN_B;

		if (it->VSCMGModel == vscmgBalancedWheels || it->VSCMGModel == vscmgJitterSimple) {
			backSubContr.matrixD -= it->IV3 * it->ggHat_B * it->ggHat_B.transpose() + it->IW1 * it->gsHat_B * it->gsHat_B.transpose();
			backSubContr.vecRot -= (it->u_s_current-it->IW1*omegat*it->gammaDot)*it->gsHat_B + it->IW1*it->Omega*it->gammaDot*it->gtHat_B + (it->u_g_current+(it->IV1-it->IV2)*omegas*omegat+it->IW1*it->Omega*omegat)*it->ggHat_B
			+ omegaTilde*it->IGPntGc_B*it->gammaDot*it->ggHat_B + omegaTilde*it->IWPntWc_B*omega_WB_B;
			if (it->VSCMGModel == vscmgJitterSimple) {
				/* static imbalance force: Fs = Us * Omega^2 */
				tempF = it->U_s * OmegaSquared * it->w2Hat_B;
				backSubContr.vecTrans += tempF;
				/* static imbalance torque: tau_s = cross(r_B,Fs), dynamic imbalance torque: tau_d = Ud * Omega^2 */
				backSubContr.vecRot += it->rGB_B.cross(tempF) + it->U_d*OmegaSquared*it->w2Hat_B;
			}
        } else if (it->VSCMGModel == vscmgJitterFullyCoupled) {

            gravityTorquePntW_B = it->d*it->w2Hat_B.cross(it->massW*g_B);
			gravityTorquePntG_B = it->rGcG_B.cross(it->massG*g_B);
			it->gravityTorqueWheel_s = it->gsHat_B.dot(gravityTorquePntW_B);
			it->gravityTorqueGimbal_g = it->ggHat_B.dot(gravityTorquePntG_B);


			double dSquared = it->d * it->d;
			double gammaDotSquared = it->gammaDot * it->gammaDot;
			Eigen::Matrix3d omegaTildeSquared = omegaTilde*omegaTilde;
			Eigen::Matrix3d ggHatTilde_B = eigenTilde(it->ggHat_B);
			Eigen::Matrix3d w2HatTilde_B = eigenTilde(it->w2Hat_B);
			Eigen::Vector3d omega_GN_B = omegaLoc_BN_B + it->gammaDot*it->ggHat_B;
			Eigen::Vector3d omega_WN_B = omega_GN_B + it->Omega*it->gsHat_B;

			Eigen::Vector3d rVcG_B = 1.0/it->massV*(it->massG*it->rGcG_B + it->massW*it->rWcG_B);
			Eigen::Vector3d rVcB_B = rVcG_B + it->rGB_B;
			Eigen::Matrix3d rTildeVcG_B = eigenTilde(rVcG_B);
			Eigen::Matrix3d rTildeVcB_B = eigenTilde(rVcG_B + it->rGB_B);
			Eigen::Vector3d rGcVc_B = it->rGcG_B - rVcG_B;
			Eigen::Vector3d rWcVc_B = it->rWcG_B - rVcG_B;
			Eigen::Matrix3d rTildeGcVc_B = eigenTilde(rGcVc_B);
			Eigen::Matrix3d rTildeWcVc_B = eigenTilde(rWcVc_B);
			Eigen::Matrix3d IVPntVc_B = it->IGPntGc_B + it->massG*rTildeGcVc_B*rTildeGcVc_B.transpose()
										+ it->IWPntWc_B + it->massW*rTildeWcVc_B*rTildeWcVc_B.transpose();
			Eigen::Vector3d rPrimeGcG_B = it->rPrimeGcB_B;
			Eigen::Vector3d rPrimeWcG_B = it->rPrimeWcB_B;
			Eigen::Vector3d rPrimeVcG_B = it->rhoG*rPrimeGcG_B + it->rhoW*rPrimeWcG_B;
			Eigen::Vector3d rPrimeVcB_B = rPrimeVcG_B;
			Eigen::Vector3d rPrimeGcVc_B = rPrimeGcG_B - rPrimeVcG_B;
			Eigen::Vector3d rPrimeWcVc_B = rPrimeWcG_B - rPrimeVcG_B;

			Eigen::Matrix3d P = it->massW*it->rhoG*rTildeWcVc_B - it->massG*it->rhoW*rTildeGcVc_B + it->massW*rTildeVcG_B;
			Eigen::Matrix3d Q = it->massG*it->rhoW*rTildeGcVc_B - it->massW*it->rhoG*rTildeWcVc_B + it->massG*rTildeVcG_B;

			it->egamma = it->ggHat_B.dot(it->IGPntGc_B*it->ggHat_B+it->IWPntWc_B*it->ggHat_B+P*(it->l*it->gtHat_B-it->d*cos(it->theta)*it->gsHat_B)+Q*ggHatTilde_B*it->rGcG_B);
			it->agamma = 1.0/it->egamma*it->massV*rTildeVcG_B*it->ggHat_B;
			it->bgamma = -1.0/it->egamma*(IVPntVc_B.transpose()*it->ggHat_B-it->massV*rTildeVcB_B*rTildeVcG_B*it->ggHat_B);
			it->cgamma = -1.0/it->egamma*(it->ggHat_B.dot(it->IWPntWc_B*it->gsHat_B) + it->d*it->ggHat_B.dot(P*it->w3Hat_B));
			it->dgamma = -1.0/it->egamma*it->ggHat_B.dot( it->gammaDot*Q*ggHatTilde_B*rPrimeGcG_B + P*((2.0*it->d*it->gammaDot*it->Omega*sin(it->theta)-it->l*gammaDotSquared)*it->gsHat_B-it->d*gammaDotSquared*cos(it->theta)*it->gtHat_B-it->d*OmegaSquared*it->w2Hat_B)
							+ it->IPrimeGPntGc_B*omega_GN_B + omegaTilde*it->IGPntGc_B*omega_GN_B + it->IWPntWc_B*it->Omega*it->gammaDot*it->gtHat_B + it->IPrimeWPntWc_B*omega_WN_B + omegaTilde*it->IWPntWc_B*omega_WN_B
							+ it->massG*rTildeGcVc_B*(2.0*omegaTilde*rPrimeGcVc_B+omegaTilde*omegaTilde*rGcVc_B) + it->massW*rTildeWcVc_B*(2.0*omegaTilde*rPrimeWcVc_B+omegaTildeSquared*rWcVc_B)
							+ it->massV*rTildeVcG_B*(2.0*omegaTilde*rPrimeVcB_B+omegaTildeSquared*rVcB_B) )
							+ 1.0/it->egamma*(it->u_g_current + it->gravityTorqueGimbal_g);

			it->eOmega = it->IW1 + it->massW*dSquared;
			it->aOmega = -1.0/it->eOmega*it->massW*it->d*it->w3Hat_B;
			it->bOmega = -1.0/it->eOmega*(it->IWPntWc_B.transpose()*it->gsHat_B - it->massW*it->d*it->rTildeWcB_B*w2HatTilde_B*it->gsHat_B);
			it->cOmega = -1.0/it->eOmega*(it->IW13*cos(it->theta)-it->massW*it->d*it->l*sin(it->theta));
			it->dOmega = -1.0/it->eOmega*(it->gsHat_B.dot(it->IPrimeWPntWc_B*omega_WN_B)
										  + it->gsHat_B.transpose()*omegaTilde*it->IWPntWc_B*omega_WN_B
										  + it->massW*it->d*it->gsHat_B.transpose()*w2HatTilde_B*(2.0*it->rPrimeTildeWcB_B.transpose()*omegaLoc_BN_B+omegaTilde*omegaTilde*it->rWcB_B))
										  + (1.0/it->eOmega)*(it->IW13*sin(it->theta)*it->Omega*it->gammaDot - it->massW*dSquared*gammaDotSquared*cos(it->theta)*sin(it->theta) + it->u_s_current + it->gravityTorqueWheel_s);

			it->p = (it->aOmega+it->cOmega*it->agamma)/(1.0-it->cOmega*it->cgamma);
			it->q = (it->bOmega+it->cOmega*it->bgamma)/(1.0-it->cOmega*it->cgamma);
			it->s = (it->dOmega+it->cOmega*it->dgamma)/(1.0-it->cOmega*it->cgamma);

			ur = it->massG*ggHatTilde_B*it->rGcG_B - it->massW*it->d*cos(it->theta)*it->gsHat_B + it->massW*it->l*it->gtHat_B;
			vr = it->massW*it->d*it->w3Hat_B;
			kr = it->massG*it->gammaDot*ggHatTilde_B*it->rPrimeGcB_B + it->massW*((2.0*it->d*it->gammaDot*it->Omega*sin(it->theta)-it->l*gammaDotSquared)*it->gsHat_B-it->d*gammaDotSquared*cos(it->theta)*it->gtHat_B-it->d*OmegaSquared*it->w2Hat_B);

			uomega = it->IGPntGc_B*it->ggHat_B + it->massG*it->rTildeGcB_B*ggHatTilde_B*it->rGcG_B + it->IWPntWc_B*it->ggHat_B + it->massW*it->rTildeWcB_B*(it->l*it->gtHat_B-it->d*cos(it->theta)*it->gsHat_B);
			vomega = it->IWPntWc_B*it->gsHat_B + it->massW*it->d*it->rTildeWcB_B*it->w3Hat_B;
			komega = it->IPrimeGPntGc_B*it->gammaDot*it->ggHat_B + omegaTilde*it->IGPntGc_B*it->gammaDot*it->ggHat_B + it->massG*omegaTilde*it->rTildeGcB_B*it->rPrimeGcB_B + it->massG*it->gammaDot*it->rTildeGcB_B*ggHatTilde_B*rPrimeGcG_B
						+ it->IWPntWc_B*it->Omega*it->gammaDot*it->gtHat_B + it->IPrimeWPntWc_B*omega_WB_B + omegaTilde*it->IWPntWc_B*omega_WB_B + it->massW*omegaTilde*it->rTildeWcB_B*it->rPrimeWcB_B
						+ it->massW*it->rTildeWcB_B*((2.0*it->d*it->gammaDot*it->Omega*sin(it->theta)-it->l*gammaDotSquared)*it->gsHat_B-it->d*gammaDotSquared*cos(it->theta)*it->gtHat_B-it->d*OmegaSquared*it->w2Hat_B);

			backSubContr.matrixA += ur*it->agamma.transpose() + (vr+ur*it->cgamma)*it->p.transpose();
			backSubContr.matrixB += ur*it->bgamma.transpose() + (vr+ur*it->cgamma)*it->q.transpose();
			backSubContr.matrixC += uomega*it->agamma.transpose() + (vomega+uomega*it->cgamma)*it->p.transpose();
			backSubContr.matrixD += uomega*it->bgamma.transpose() + (vomega+uomega*it->cgamma)*it->q.transpose();
			backSubContr.vecTrans -= kr + ur*it->dgamma + (vr+ur*it->cgamma)*it->s;
			backSubContr.vecRot -= komega + uomega*it->dgamma + (vomega+uomega*it->cgamma)*it->s;
		}
	}
	return;
}

void VSCMGStateEffector::computeDerivatives(double integTime, Eigen::Vector3d rDDot_BN_N, Eigen::Vector3d omegaDot_BN_B, Eigen::Vector3d sigma_BN)
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
	std::vector<VSCMGConfigSimMsg>::iterator it;

	//! Grab necessarry values from manager
	omegaDotBNLoc_B = this->hubOmega->getStateDeriv();
	omegaLoc_BN_B = this->hubOmega->getState();
	rDDotBNLoc_N = this->hubVelocity->getStateDeriv();
	sigmaBNLocal = (Eigen::Vector3d )this->hubSigma->getState();
	dcm_NB = sigmaBNLocal.toRotationMatrix();
	dcm_BN = dcm_NB.transpose();
	rDDotBNLoc_B = dcm_BN*rDDotBNLoc_N;

	//! - Compute Derivatives
	for(it=VSCMGData.begin(); it!=VSCMGData.end(); it++)
	{
		omegas = it->gsHat_B.transpose()*omegaLoc_BN_B;
		omegat = it->gtHat_B.transpose()*omegaLoc_BN_B;
		omegag = it->ggHat_B.transpose()*omegaLoc_BN_B;

		gammasDot(VSCMGi,0) = it->gammaDot;
        if(it->VSCMGModel == vscmgJitterFullyCoupled || it->VSCMGModel == vscmgJitterSimple) {
            // - Set trivial kinemetic derivative
            thetasDot(thetaCount,0) = it->Omega;
            thetaCount++;
        }
		if (it->VSCMGModel == vscmgBalancedWheels || it->VSCMGModel == vscmgJitterSimple) {
			OmegasDot(VSCMGi,0) = - omegat*it->gammaDot - it->gsHat_B.transpose()*omegaDotBNLoc_B + (it->u_s_current+it->gravityTorqueWheel_s)/it->IW1;
			gammaDotsDot(VSCMGi,0) = 1/it->IV3*((it->u_g_current+it->gravityTorqueGimbal_g)+(it->IV1-it->IV2)*omegas*omegat+it->IW1*it->Omega*omegat-it->IV3*it->ggHat_B.transpose()*omegaDotBNLoc_B);
        } else if(it->VSCMGModel == vscmgJitterFullyCoupled) {
			OmegasDot(VSCMGi,0) = it->p.dot(rDDotBNLoc_B) + it->q.dot(omegaDotBNLoc_B) + it->s;
			gammaDotsDot(VSCMGi,0) = it->agamma.dot(rDDotBNLoc_B) + it->bgamma.dot(omegaDotBNLoc_B) + it->cgamma*OmegasDot(VSCMGi,0) + it->dgamma;
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

void VSCMGStateEffector::updateEnergyMomContributions(double integTime, Eigen::Vector3d & rotAngMomPntCContr_B,
                                                      double & rotEnergyContr, Eigen::Vector3d omega_BN_B)
{
	Eigen::MRPd sigmaBNLocal;
	Eigen::Matrix3d dcm_BN;                        /* direction cosine matrix from N to B */
	Eigen::Matrix3d dcm_NB;                        /* direction cosine matrix from B to N */
	Eigen::Vector3d omegaLoc_BN_B = hubOmega->getState();

    //! - Compute energy and momentum contribution of each wheel
    rotAngMomPntCContr_B.setZero();
    std::vector<VSCMGConfigSimMsg>::iterator it;
    for(it=VSCMGData.begin(); it!=VSCMGData.end(); it++)
    {
		Eigen::Vector3d omega_GN_B = omegaLoc_BN_B + it->gammaDot*it->ggHat_B;
		Eigen::Vector3d omega_WN_B = omega_GN_B + it->Omega*it->gsHat_B;
		if (it->VSCMGModel == vscmgBalancedWheels || it->VSCMGModel == vscmgJitterSimple) {
			it->rWcB_B = it->rGB_B;
			Eigen::Vector3d rDotWcB_B = omegaLoc_BN_B.cross(it->rWcB_B);
			rotAngMomPntCContr_B += it->IWPntWc_B*omega_WN_B + it->IGPntGc_B*omega_GN_B + it->massV*it->rWcB_B.cross(rDotWcB_B);
			rotEnergyContr += 1.0/2.0*omega_WN_B.dot(it->IWPntWc_B*omega_WN_B) + 1.0/2.0*omega_GN_B.dot(it->IGPntGc_B*omega_GN_B) + 1.0/2.0*it->massV*rDotWcB_B.dot(rDotWcB_B);
		} else if (it->VSCMGModel == vscmgJitterFullyCoupled) {
			Eigen::Vector3d rDotWcB_B = it->rPrimeWcB_B + omegaLoc_BN_B.cross(it->rWcB_B);
			Eigen::Vector3d rDotGcB_B = it->rPrimeGcB_B + omegaLoc_BN_B.cross(it->rGcB_B);
			rotAngMomPntCContr_B += it->IWPntWc_B*omega_WN_B + it->massW*it->rWcB_B.cross(rDotWcB_B);
			rotAngMomPntCContr_B += it->IGPntGc_B*omega_GN_B + it->massG*it->rGcB_B.cross(rDotGcB_B);
			rotEnergyContr += 1.0/2.0*omega_WN_B.transpose()*it->IWPntWc_B*omega_WN_B + 1.0/2.0*it->massW*rDotWcB_B.dot(rDotWcB_B);
			rotEnergyContr += 1.0/2.0*omega_GN_B.transpose()*it->IGPntGc_B*omega_GN_B + 1.0/2.0*it->massG*rDotGcB_B.dot(rDotGcB_B);
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
	//! Begin method steps
	//! - Find the message ID associated with the InputCmds string.
	//! - Warn the user if the message is not successfully linked.
	CmdsInMsgID = SystemMessaging::GetInstance()->subscribeToMessage(InputCmds,
                                                                     sizeof(VSCMGArrayTorqueIntMsg),
																	 moduleID);
	if(CmdsInMsgID < 0)
	{
        BSK_PRINT(MSG_WARNING, "Did not find a valid message with name: %s", InputCmds.c_str());
	}

	std::vector<VSCMGConfigSimMsg>::iterator it;
	for (it = VSCMGData.begin(); it != VSCMGData.end(); it++)
	{
		it->w2Hat0_B = it->gtHat0_B;
		it->w3Hat0_B = it->ggHat_B;

		//! Define CoM offset d and off-diagonal inertia IW13 if using fully coupled model
		if (it->VSCMGModel == vscmgJitterFullyCoupled) {
			it->d = it->U_s/it->massW; //!< determine CoM offset from static imbalance parameter
			it->IW13 = it->U_d; //!< off-diagonal inertia is equal to dynamic imbalance parameter
		}
		if (it->VSCMGModel == vscmgBalancedWheels || it->VSCMGModel == vscmgJitterSimple) {
			it->IV1 = it->IW1 + it->IG1;
			it->IV2 = it->IW2 + it->IG2;
			it->IV3 = it->IW3 + it->IG3;
			it->IG12 = 0.;
			it->IG13 = 0.;
			it->IG23 = 0.;
			it->IW13 = 0.;
		}

		Eigen::Matrix3d dcm_GG0 = eigenM3(it->gamma);
		Eigen::Matrix3d dcm_BG0;
		dcm_BG0.col(0) = it->gsHat0_B;
		dcm_BG0.col(1) = it->gtHat0_B;
		dcm_BG0.col(2) = it->ggHat_B;
		Eigen::Matrix3d dcm_BG = dcm_BG0 * dcm_GG0.transpose();
		it->gsHat_B = dcm_BG.col(0);
		it->gtHat_B = dcm_BG.col(1);

		it->rGcG_B = dcm_BG * it->rGcG_G;
		it->massV = it->massG + it->massW;
		it->rhoG = it->massG/it->massV;
		it->rhoW = it->massW/it->massV;
	}
    return;
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
	int it = 0;
	double u_s;
	double u_g;
	double omegaCritical;
	double gammaDotCritical;

	// loop through commands
	for(CmdIt=NewVSCMGCmds.begin(); CmdIt!=NewVSCMGCmds.end(); CmdIt++)
	{
		//! wheel torque saturation
		//! set u_s_max to less than zero to disable saturation
		if (this->VSCMGData[it].u_s_max > 0.0) {
			if(CmdIt->u_s_cmd > this->VSCMGData[it].u_s_max) {
				CmdIt->u_s_cmd = this->VSCMGData[it].u_s_max;
			} else if(CmdIt->u_s_cmd < -this->VSCMGData[it].u_s_max) {
				CmdIt->u_s_cmd = -this->VSCMGData[it].u_s_max;
			}
		}

		//! gimbal torque saturation
		//! set u_g_max to less than zero to disable saturation
		if (this->VSCMGData[it].u_g_max > 0.0) {
			if(CmdIt->u_g_cmd > this->VSCMGData[it].u_g_max) {
				CmdIt->u_g_cmd = this->VSCMGData[it].u_g_max;
			} else if(CmdIt->u_g_cmd < -this->VSCMGData[it].u_g_max) {
				CmdIt->u_g_cmd = -this->VSCMGData[it].u_g_max;
			}
		}

		//! minimum wheel torque
		//! set u_s_min to less than zero to disable minimum torque
		if( std::abs(CmdIt->u_s_cmd) < this->VSCMGData[it].u_s_min) {
			CmdIt->u_s_cmd = 0.0;
		}

		//! minimum gimbal torque
		//! set u_g_min to less than zero to disable minimum torque
		if( std::abs(CmdIt->u_g_cmd) < this->VSCMGData[it].u_g_min) {
			CmdIt->u_g_cmd = 0.0;
		}

        // Speed saturation
        if (std::abs(this->VSCMGData[it].Omega)>= this->VSCMGData[it].Omega_max) {
            CmdIt->u_s_cmd = 0.0;
            printf("Omega_max = %f\n", this->VSCMGData[it].Omega_max);
        }

		//! wheel Coulomb friction
		//! set wheelLinearFrictionRatio to less than zero to disable linear friction
		//! set u_s_f to zero to disable all friction
		if (this->VSCMGData[it].wheelLinearFrictionRatio > 0.0) {
			omegaCritical = this->VSCMGData[it].Omega_max * this->VSCMGData[it].wheelLinearFrictionRatio;
		} else {
			omegaCritical = 0.0;
		}
		if(this->VSCMGData[it].Omega > omegaCritical) {
			u_s = CmdIt->u_s_cmd - this->VSCMGData[it].u_s_f;
		} else if(this->VSCMGData[it].Omega < -omegaCritical) {
			u_s = CmdIt->u_s_cmd + this->VSCMGData[it].u_s_f;
		} else {
			if (this->VSCMGData[it].wheelLinearFrictionRatio > 0) {
				u_s = CmdIt->u_s_cmd - this->VSCMGData[it].u_s_f*this->VSCMGData[it].Omega/omegaCritical;
			} else {
				u_s = CmdIt->u_s_cmd;
			}
		}

		//! gimbal Coulomb friction
		//! set gimbalLinearFrictionRatio to less than zero to disable linear friction
		//! set u_g_f to zero to disable friction
		if (this->VSCMGData[it].gimbalLinearFrictionRatio > 0.0) {
			gammaDotCritical = this->VSCMGData[it].gammaDot_max * this->VSCMGData[it].wheelLinearFrictionRatio;
		} else {
			gammaDotCritical = 0.0;
		}
		if(this->VSCMGData[it].gammaDot > gammaDotCritical) {
			u_g = CmdIt->u_g_cmd - this->VSCMGData[it].u_g_f;
		} else if(this->VSCMGData[it].gammaDot < -gammaDotCritical) {
			u_g = CmdIt->u_g_cmd + this->VSCMGData[it].u_g_f;
		} else {
			if (this->VSCMGData[it].gimbalLinearFrictionRatio > 0) {
				u_g = CmdIt->u_g_cmd - this->VSCMGData[it].u_g_f*this->VSCMGData[it].gammaDot/gammaDotCritical;
			} else {
				u_g = CmdIt->u_g_cmd;
			}
		}

		this->VSCMGData[it].u_s_current = u_s; // save actual torque for wheel motor
		this->VSCMGData[it].u_g_current = u_g; // save actual torque for wheel motor

		it++;

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
