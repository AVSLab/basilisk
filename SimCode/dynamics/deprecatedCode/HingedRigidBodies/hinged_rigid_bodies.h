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

#ifndef HINGED_RIGID_BODIES_H
#define HINGED_RIGID_BODIES_H

#include <vector>
#include "_GeneralModuleFiles/sys_model.h"
#include "_GeneralModuleFiles/dyn_effector.h"
#include "../ADCSAlgorithms/effectorInterfaces/errorConversion/vehEffectorOut.h"

//! @brief Container for hinged rigid body information for hinged rigid body dynamics*/
typedef struct {
    double mass;               //!< kg, mass of hinged rigid body
    double IPntS_S[9];         //!< kg-m^2, Inertia of hinged rigid body about point S in S frame components
    double d;                  //!< m, distance from hinge point to hinged rigid body center of mass
    double k;                  //!< N-m/rad, torsional spring constant of hinge
    double c;                  //!< N-m-s/rad, rotational damping coefficient of hinge
    double r_HB_B[3];          //!< m, vector pointing from body frame origin to Hinge location
    double rTilde_HB_B[3][3];  //!< Tilde matrix of r_HB_B
    double HB[9];              //!< DCM from body frame to hinge frame
    double SH[3][3];           //!< DCM from hinge to hinged rigid body frame, S
    double SB[3][3];           //!< DCM from body to S frame
    double omega_BN_S[3];      //!< omega_BN in S frame components
    double sHat1_B[3];         //!< unit direction vector for the first axis of the S frame
    double sHat2_B[3];         //!< unit direction vector for the second axis of the S frame
    double sHat3_B[3];         //!< unit direction vector for the third axis of the S frame
    double r_SB_B[3];          //!< Vector pointing from body origin to CoM of hinged rigid body in B frame comp
    double rTilde_SB_B[3][3];  //!< Tilde matrix of r_SB_B
    double rPrime_SB_B[3];     //!< Body time derivative of r_SB_B
    double rPrimeTilde_SB_B[3][3];//!< Tilde matrix of rPrime_SB_B
    double theta;              //!< rad, hinged rigid body angle
    double thetaDot;           //!< rad/s, hinged rigid body angle rate
}HingedRigidBodyConfigData;

class HingedRigidBodies {
public:
    HingedRigidBodies();
    ~HingedRigidBodies();

    //! Add a new hinged rigid body to hinged rigid body set
    void addHingedRigidBody(HingedRigidBodyConfigData *newHRB) {hingedRigidBodyData.push_back(*newHRB);}

public:
    std::vector<HingedRigidBodyConfigData> hingedRigidBodyData;
    
private:
};

/*! @} */

#endif
