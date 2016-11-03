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

#ifndef FUEL_TANK_H
#define FUEL_TANK_H

#include <vector>
#include "_GeneralModuleFiles/sys_model.h"
#include "_GeneralModuleFiles/dyn_effector.h"
#include "../ADCSAlgorithms/effectorInterfaces/errorConversion/vehEffectorOut.h"

//! @brief Container for Fuel Tank information for fuel slosh dynamics*/
typedef struct {
    double massFT;             //!< kg, total mass of the fuel tank
    double r_TB_B[3];          //!< m, position vector from B point to center of tank, T
}FuelTankConfigData;

typedef struct {
    double massFSP;            //!< kg, mass of fuel slosh particle
    double r_PT_B[3];          //!< m, postion vector from its tank center to slosh equilibrium, P, in body frame
    double r_PB_B[3];          //!< m, position vector from B point to slosh equilibrium, P, in body frame
    double r_PcB_B[3];         //!< m, position vector from B to center of mass of slosh mass, in body frame
    double rTilde_PcB_B[3][3]; //!< tilde matrix of r_Pc_B_B
    double rPrime_PcB_B[3];    //!< m/s, body derivative of r_PcB_B, in body frame
    double rPrimeTilde_PcB_B[3][3]; //! - tilde matrix of rPrime_PcB_B
    double pHat_B[3];          //!< slosh direction unit vector, in body frame
    double k;                  //!< N/m, linear spring constant for fuel slosh
    double c;                  //!< N-s/m, linear damping term for fuel slosh
    double rho;                //!< m, fuel slosh displacement from equilibrium
    double rhoDot;             //!< m/s, time derivative of rho;
}FuelSloshParticleConfigData;

class FuelTank {
public:
    FuelTank();
    ~FuelTank();

    void addFuelSloshParticle(FuelSloshParticleConfigData *newFSP) {fuelSloshParticlesData.push_back(*newFSP);}

public:
    std::vector<FuelSloshParticleConfigData> fuelSloshParticlesData;
    FuelTankConfigData fuelTankData;
    
private:
};

/*! @} */

#endif
