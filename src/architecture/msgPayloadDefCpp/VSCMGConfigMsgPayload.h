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

#ifndef SIM_VSCMG_CONFIG_MESSAGE_H
#define SIM_VSCMG_CONFIG_MESSAGE_H

#include <Eigen/Dense>
#include <vector>


/*! @brief enumeration definiting the types of VSCMG modes */
enum VSCMGModels { vscmgBalancedWheels, vscmgJitterSimple, vscmgJitterFullyCoupled };


/*! @brief Structure used to define the individual VSCMG configuration data message*/
typedef struct
//@cond DOXYGEN_IGNORE
VSCMGConfigMsgPayload
//@endcond
{
	VSCMGModels VSCMGModel;     //!< [-], type of imbalance model to use
	Eigen::Vector3d rGB_B;		//!< [m], vector pointing from body frame B origin to VSCMG frame G origin in B frame components
	Eigen::Vector3d gsHat0_B;   //!< [-] first axis of the G frame in B frame components (G frame for gamma=0)]
	Eigen::Vector3d gsHat_B;	//!< [-] first axis of the G frame in B frame components
	Eigen::Vector3d gtHat0_B;   //!< [-] second axis of the G frame in B frame components (G frame for gamma=0)]
	Eigen::Vector3d gtHat_B;    //!< [-] second axis of the G frame in B frame components
	Eigen::Vector3d ggHat_B;    //!< [-] third axis of the G frame in B frame components
	Eigen::Vector3d w2Hat0_B;	//!< [-] second axis of the W frame in B frame components (W frame for theta=0)
	Eigen::Vector3d w2Hat_B;    //!< [-] second axis of the W frame in B frame components
    Eigen::Vector3d w3Hat0_B;	//!< [-] third axis of the W frame in B frame components (W frame for theta=0)
	Eigen::Vector3d w3Hat_B;    //!< [-] third axis of the W frame in B frame components
    double massV;               //!< [kg] mass of the VSCMG system
	double massG;               //!< [kg] mass of the gimbal
	double massW;               //!< [kg] mass of the wheel
    double theta;               //!< [rad] wheel angle
    double Omega;               //!< [rad/s] wheel speed
	double gamma;               //!< [rad] gimbal angle
	double gammaDot;            //!< [rad/s] gimbal rate
    double IW1;                 //!< [kg-m^2] inertia of the wheel about the wheel spin axis in W frame components
    double IW2;                 //!< [kg-m^2] inertia of the wheel about the wheel second axis in W frame components
    double IW3;                 //!< [kg-m^2] inertia of the wheel about the wheel third axis in W frame components
	double IW13;                //!< [kg-m^2] inertia coupling between wheel first and third axes in W frame components
	double IG1;              	//!< [kg-m^2] inertia of the gimbal about the gimbal first axis in G frame components
	double IG2;          		//!< [kg-m^2] inertia of the gimbal about the gimbal second axis in G frame components
	double IG3;                 //!< [kg-m^2] inertia of the gimbal about the gimbal third axis in G frame components
	double IG12;             	//!< [kg-m^2] inertia coupling between gimbal first and second axes in G frame components
	double IG13;         		//!< [kg-m^2] inertia coupling between gimbal first and third axes in G frame components
	double IG23;                //!< [kg-m^2] inertia coupling between gimbal second and third axes in G frame components
	double IV1;              	//!< [kg-m^2] inertia of the VSCMG about the VSCMG first axis in G frame components
	double IV2;          		//!< [kg-m^2] inertia of the VSCMG about the VSCMG second axis in G frame components
	double IV3;                 //!< [kg-m^2] inertia of the VSCMG about the VSCMG third axis in G frame components
	double rhoG;                //!< [-] ratio of the gimbal mass to the VSCMG mass
	double rhoW;                //!< [-] ratio of the wheel mass to the VSCMG mass
    double U_s;                 //!< [kg-m] static imbalance
    double U_d;                 //!< [kg-m^2] dynamic imbalance
    double d;                	//!< [m] offset of point Wc (center of mass of the wheel) from wheel frame W origin
	double l;                   //!< [m] offset of wheel third axis from gimbal third axis
	double L;                   //!< [m] offset of the wheel first axis from the gimbal first axis
    double u_s_current;         //!< [N-m] current wheel motor torque
    double u_s_max = -1.0;      //!< [N-m] Max wheel torque
    double u_s_min;             //!< [N-m] Min wheel torque
    double u_s_f;               //!< [N-m] Coulomb friction wheel torque magnitude
    double Omega_max = -1.0;    //!< [rad/s] max wheel speed
	double wheelLinearFrictionRatio;//!< [%] ratio relative to max wheel speed value up to which the friction behaves linearly
	double u_g_current;         //!< [N-m] current gimbal motor torque
	double u_g_max = -1.0;      //!< [N-m] Max gimbal torque
	double u_g_min;             //!< [N-m] Min gimbal torque
	double u_g_f;               //!< [N-m] Coulomb friction gimbal torque magnitude
	double gammaDot_max;        //!< [rad/s] max gimbal rate
	double gimbalLinearFrictionRatio;//!< [%] ratio relative to max gimbal rate value up to which the friction behaves linearly

	Eigen::Matrix3d IGPntGc_B;  //!< [kg-m^2] inertia of the gimbal about point Gc (center of mass of the gimbal) in B frame components
    Eigen::Matrix3d IWPntWc_B;  //!< [kg-m^2] inertia of the wheel about point Wc (center of mass of the wheel) in B frame components
	Eigen::Matrix3d IPrimeGPntGc_B;     //!< [kg-m^2] body frame time derivative of IGPntGc_B
	Eigen::Matrix3d IPrimeWPntWc_B;     //!< [kg-m^2] body frame time derivative of IWPntWc_B
	Eigen::Vector3d rGcG_G;     //!< [m] vector pointing from VSCMG frame G origin to point Gc (center of mass of the gimbal) in G frame components
	Eigen::Vector3d rGcG_B;     //!< [m] vector pointing from VSCMG frame G origin to point Gc (center of mass of the gimbal) in B frame components
	Eigen::Vector3d rGcB_B;     //!< [m] vector pointing from body frame B origin to point Gc (center of mass of the gimbal) in B frame components
    Eigen::Vector3d rWcB_B;     //!< [m] vector pointing from body frame B origin to point Wc (center of mass of the wheel) in B frame components
	Eigen::Vector3d rWcG_B;     //!< [m] vector pointing from VSCMG frame G origin to point Wc (center of mass of the wheel) in B frame components
	Eigen::Matrix3d rTildeGcB_B;        //!< [m] tilde matrix of rGcB_B
    Eigen::Matrix3d rTildeWcB_B;        //!< [m] tilde matrix of rWcB_B
	Eigen::Vector3d rPrimeGcB_B;        //!< [m/s] body frame time derivative of rGcB_B
    Eigen::Vector3d rPrimeWcB_B;        //!< [m/s] body frame time derivative of rWcB_B
	Eigen::Matrix3d rPrimeTildeGcB_B;   //!< [m/s] tilde matrix of body frame time derivative of rGcB_B
	Eigen::Matrix3d rPrimeTildeWcB_B;   //!< [m/s] tilde matrix of body frame time derivative of rWcB_B

	Eigen::Vector3d aOmega; //!< [1/m] parameter used in coupled jitter back substitution
	Eigen::Vector3d bOmega; //!< [-] parameter used in coupled jitter back substitution
	double cOmega;          //!< [-] parameter used in coupled jitter back substitution
	double dOmega;          //!< [rad/s^2] parameter used in coupled jitter back substitution
	double eOmega;          //!< [kg-m^2] parameter used in coupled jitter back substitution
	Eigen::Vector3d agamma; //!< [1/m] parameter used in coupled jitter back substitution
	Eigen::Vector3d bgamma; //!< [-] parameter used in coupled jitter back substitution
	double cgamma;          //!< [-] parameter used in coupled jitter back substitution
	double dgamma;          //!< [rad/s^2] parameter used in coupled jitter back substitution
	double egamma;          //!< [kg-m^2] parameter used in coupled jitter back substitution
	Eigen::Vector3d p;      //!< [1/m] parameter used in coupled jitter back substitution
	Eigen::Vector3d q;      //!< [-] parameter used in coupled jitter back substitution
	double s;               //!< [rad/s^2] parameter used in coupled jitter back substitution

	double gravityTorqueWheel_s;    //!< [N-m] gravitational torque on the wheel about wheel spin axis
	double gravityTorqueGimbal_g;   //!< [N-m] gravitational torque on the gimbal about gimbal third axis
}VSCMGConfigMsgPayload;




#endif
