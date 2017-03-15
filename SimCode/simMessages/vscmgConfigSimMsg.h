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

#ifndef SIM_VSCMG_CONFIG_MESSAGE_H
#define SIM_VSCMG_CONFIG_MESSAGE_H

/*! @brief enumeration definiting the types of VSCMG modes */ 
enum VSCMGModels { BalancedWheels, JitterSimple, JitterFullyCoupled };


/*! @brief Structure used to define the individual VSCMG configuration data message*/
typedef struct {
    std::string typeName;      //!< [-], string containing the VSCMG type name
    Eigen::Vector3d rWB_S;		//!< [m], position vector of the VSCMG relative to the spacecraft structural frame
    Eigen::Vector3d gsHat_S;	//!< [-] spin axis unit vector in structural frame
    Eigen::Vector3d w2Hat0_S;	//!< [-] initial torque axis unit vector in structural
    Eigen::Vector3d w3Hat0_S;	//!< [-] initial gimbal axis unit vector in structural frame
    Eigen::Vector3d rWB_B;		//!< [m], position vector of the VSCMG relative to the spacecraft body frame
    Eigen::Vector3d gsHat_B;	//!< [-] spin axis unit vector in body frame
    Eigen::Vector3d w2Hat0_B;	//!< [-] initial torque axis unit vector in body frame
    Eigen::Vector3d w3Hat0_B;	//!< [-] initial gimbal axis unit vector in body frame
    double mass;               //!< [kg], wheel rotor mass
    double theta;              //!< [rad], wheel angle
    double Omega;              //!< [rad/s], wheel speed
	double gamma;              //!< [s], gimbal angle
	double gammaDot;              //!< [rad/s], gimbal rate
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
    VSCMGModels VSCMGModel; //!< [-], Type of imbalance model to use
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
}VSCMGConfigSimMsg;




#endif
