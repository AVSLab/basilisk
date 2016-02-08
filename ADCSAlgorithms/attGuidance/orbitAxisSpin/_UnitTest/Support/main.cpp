//
//  main.cpp
//  UnitTests
//
//  Created by Mar Cols Margenet on 28/01/16.
//  Copyright © 2016 Mar Cols Margenet. All rights reserved.
//

#include <iostream>
#include <fstream>
#include <limits>
#include <iomanip>

typedef std::numeric_limits< double > dbl;
using namespace std;
extern "C" {
#include "linearAlgebra.h"
#include "rigidBodyKinematics.h"
#include "astroCnt.h"
#include "orbitMotion.h"
}

/* <!-  Function to print a vector[3] on a file */
void printV3 (string fileName, string varName, double v3[3]);
void printM33 (string fileName, string varName, double M33[3][3]);
void testHill();
void testVel();


int main(int argc, const char * argv[]) {
    
    testHill();
    testVel();
    
    return 0;
}

void testVel()
{
    string fileName = "unitTestOutput.txt";
    string varName;
    /* <!- Velocity Pointing Test */
    /* <!-  Output          */
    double sigma_VN[3];
    double omega_VN_N[3];
    double domega_VN_N[3];
    
    /* <!-  Input           */
    double r_BN_N[3]; // Rs
    double v_BN_N[3]; // Vs
    double celBdyPositonVector[3]; // Rp
    double celBdyVelocityVector[3]; // Vp
    v3Set(500., 500, 1000., r_BN_N);
    v3Set(-500., -500., 0., celBdyPositonVector);
    v3Set(10., 10., 0., v_BN_N);
    v3SetZero(celBdyVelocityVector);
    
    /* <!-  Module Variables */
    double mu = MU_EARTH;
    
    /* <!-  Beforehand Computations    */
    double r[3], v[3], h[3];
    v3Subtract(r_BN_N, celBdyPositonVector, r);
    v3Subtract(v_BN_N, celBdyVelocityVector, v);
    v3Cross(r, v, h);
    
    double i_r[3], i_theta[3], i_h[3];
    v3Normalize(r, i_r);
    v3Normalize(h, i_h);
    v3Cross(i_h, i_r, i_theta);
    
    double hm, rm, drdt, dfdt, ddfdt2;
    hm = v3Norm(h);
    rm = v3Norm(r);
    drdt = v3Dot(v, i_r);
    dfdt = hm / (rm * rm);
    ddfdt2 = - 2.0 * drdt / rm * dfdt;
    
    /* <!- Hill Frame */
    double HN[3][3];
    double sigma_HN[3], omega_HN_N[3], domega_HN_N[3];
    v3Copy(i_r, HN[0]);
    v3Copy(i_theta, HN[1]);
    v3Copy(i_h, HN[2]);
    C2MRP(HN, sigma_HN);
    v3Scale(dfdt, HN[2], omega_HN_N);
    v3Scale(ddfdt2, HN[2], domega_HN_N);
    
    /* <!-  Begin Method    */
    double VN[3][3];
    double i_v[3], i_n[3];
    v3Normalize(v, i_v);
    v3Cross(i_v, i_h, i_n);
    v3Copy(i_n, VN[0]);
    v3Copy(i_v, VN[1]);
    v3Copy(i_h, VN[2]);
    C2MRP(VN, sigma_VN);
    
    /* <!- Compute Orbit Elements */
    /* compute the eccentricity vector */
    double temp3[3], v3[3];
    v3Cross(v, h, temp3);
    v3Scale(-mu, i_r, v3);
    v3Add(temp3, v3, temp3);
    double i_e[3];
    double e;
    e = v3Norm(temp3) / mu;
    v3Scale(1/ (e * mu), temp3, i_e);
    /* compute true anomaly */
    double f;
    v3Cross(i_e, i_r, v3);
    f = atan2(v3Dot(v3, i_h), v3Dot(i_e, i_r));
    
    /* <!- Compute Velocity Frame Rates */
    double dBdt, ddBdt2;
    double denom, temp;
    denom = 1 + e*e + 2*e*cos(f);
    temp = e*(e + cos(f))/denom;
    dBdt = dfdt * temp;
    ddBdt2 = ddfdt2 * temp + dfdt*dfdt * e*(e*e-1)*sin(f) / (denom*denom);
    
    double omega_VH_N[3], domega_VH_N[3];
    v3Scale(-1*dBdt, VN[2], omega_VH_N);
    v3Scale(-1*ddBdt2, VN[2], domega_VH_N);
    v3Add(omega_VH_N, omega_HN_N, omega_VN_N);
    v3Add(domega_VH_N, domega_HN_N, domega_VN_N);
    
    /* <!-  Print Results */
    varName = "sigma_VN[3]";
    printV3(fileName, varName, sigma_VN);
    varName = "omega_VN_N[3]";
    printV3(fileName, varName, omega_VN_N);
    varName = "domega_VN_N[3]";
    printV3(fileName, varName, domega_VN_N);
    
    /* <!-  Velocity Spinning Test  */
    /* <!-  Output              */
    double sigma_VsN[3];
    double omega_VsN_N[3];
    double domega_VsN_N[3];
    /* <!-  Input              */
    double omega_spin = M_PI/8;
    double phi_spin_0 = M_PI/4;
    int    o_spin = 0;
    
    /* <!-  Begin Method        */
    // <!-  Being Method    /
    int o1, o2;
    o1 = o_spin;
    o2 = o1 + 1;
    if ( o2 > 2 ) { o2 = 0; }
    
    double omega_VsV_N[3];
    v3Scale(omega_spin, VN[o1], omega_VsV_N);
    v3Add(omega_VN_N, omega_VsV_N, omega_VsN_N);
    double domega_VsV_N[3];
    v3Cross(VN[2], VN[o1], v3);
    double orbitRate;
    orbitRate = dfdt - dBdt;
    v3Scale(orbitRate * omega_spin, v3, domega_VsV_N);
    v3Add(domega_VN_N, domega_VsV_N, domega_VsN_N);
    
    double sigma_VsV[3], VsV[3][3];
    double phi_spin = phi_spin_0;
    double dt = 0.5;
    double localTime[3];
    
    for (int i = 0; i < 3; i++)
    {
        Mi(phi_spin, o1+1, VsV);
        C2MRP(VsV, sigma_VsV);
        v3Add(sigma_VsV, sigma_VN, sigma_VsN);
        phi_spin += dt * omega_spin;
        localTime[i] = dt*i;
        varName = "sigma_RN[3] = ";
        printV3(fileName, varName, sigma_VsN);
    }
    varName = "t = ";
    printV3(fileName, varName, localTime);
    
    /* <!-  Print Results */
    varName = "sigma_VsN[3]";
    printV3(fileName, varName, sigma_VsN);
    varName = "omega_VsN_N[3]";
    printV3(fileName, varName, omega_VsN_N);
    varName = "domega_VsN_N[3]";
    printV3(fileName, varName, domega_VsN_N);
    
}

void testHill()
{
    string fileName = "unitTestOutput.txt";
    string varName;
    /* <!- Hill Pointing Test */
    /* <!-  Output          */
    double sigma_HN[3];
    double omega_HN_N[3];
    double domega_HN_N[3];
    
    v3SetZero(sigma_HN);
    v3SetZero(omega_HN_N);
    v3SetZero(domega_HN_N);
    
    /* <!-  Input           */
    double r_BN_N[3]; // Rs
    double v_BN_N[3]; // Vs
    double celBdyPositonVector[3]; // Rp
    double celBdyVelocityVector[3]; // Vp
    v3Set(500., 500, 1000., r_BN_N);
    v3Set(-500., -500., 0., celBdyPositonVector);
    v3Set(0., 20., 0., v_BN_N);
    v3SetZero(celBdyVelocityVector);
    
    /* <!-  Begin Method    */
    double r[3], v[3], h[3];
    v3Subtract(r_BN_N, celBdyPositonVector, r);
    v3Subtract(v_BN_N, celBdyVelocityVector, v);
    v3Cross(r, v, h);
    
    double i_r[3], i_theta[3], i_h[3];
    v3Normalize(r, i_r);
    v3Normalize(h, i_h);
    v3Cross(i_h, i_r, i_theta);
    
    double HN[3][3];
    v3Copy(i_r, HN[0]);
    v3Copy(i_theta, HN[1]);
    v3Copy(i_h, HN[2]);
    
    double hm, rm, drdt, dfdt, ddfdt2;
    hm = v3Norm(h);
    rm = v3Norm(r);
    drdt = v3Dot(v, i_r);
    dfdt = hm / (rm * rm);
    ddfdt2 = - 2.0 * drdt / rm * dfdt;
    
    C2MRP(HN, sigma_HN);
    v3Scale(dfdt, HN[2], omega_HN_N);
    v3Scale(ddfdt2, HN[2], domega_HN_N);
    
    /* <!-  Print Results */
    varName = "sigma_R0N[3]";
    printV3(fileName, varName, sigma_HN);
    varName = "omega_R0N_N[3]";
    printV3(fileName, varName, omega_HN_N);
    varName = "domega_R0N_N[3]";
    printV3(fileName, varName, domega_HN_N);
    
    /* <!-  Hill Spinning Test */
    // <!- Output    /
    double sigma_HsN[3];
    double omega_HsN_N[3];
    double domega_HsN_N[3];
    
    // <!- Input    /
    double omega_spin = M_PI/8;
    double phi_spin_0 = M_PI/4;
    int    o_spin = 0;
    
    // <!-  Being Method    /
    int o1, o2;
    o1 = o_spin;
    o2 = o1 + 1;
    if ( o2 > 2 ) { o2 = 0; }
    
    double omega_HsH_N[3];
    v3Scale(omega_spin, HN[o1], omega_HsH_N);
    v3Add(omega_HN_N, omega_HsH_N, omega_HsN_N);
    double domega_HsH_N[3], v3[3];
    v3Cross(HN[2], HN[o1], v3);
    v3Scale(dfdt * omega_spin, v3, domega_HsH_N);
    v3Add(domega_HN_N, domega_HsH_N, domega_HsN_N);
    
    double sigma_HsH[3], HsH[3][3];
    double phi_spin = phi_spin_0;
    double dt = 0.5;
    double localTime[3];
    
    for (int i = 0; i < 3; i++)
    {
        Mi(phi_spin, o1+1, HsH);
        C2MRP(HsH, sigma_HsH);
        v3Add(sigma_HsH, sigma_HN, sigma_HsN);
        phi_spin += dt * omega_spin;
        localTime[i] = dt*i;
        varName = "sigma_RN[3] = ";
        printV3(fileName, varName, sigma_HsN);
    }
    varName = "t = ";
    printV3(fileName, varName, localTime);
    
    /*<!-  Print Results    */
    varName = "omega_RN_N[3]";
    printV3(fileName, varName, omega_HsN_N);
    varName = "domega_RN_N[3]";
    printV3(fileName, varName, domega_HsN_N);
}

void printV3 (string fileName, string varName, double v3[3])
{
    ofstream outputFile;
    outputFile.open(fileName, std::ios_base::app);
    outputFile << std::fixed << std::setprecision(12);
    cout.precision(dbl::max_digits10);
    if (outputFile.is_open())
    {
        outputFile << varName <<" = [ ";
        cout << varName <<" = [ ";
        for (int i=0; i < 3; i++)
        {
            outputFile << fixed << v3[i] << " ";
            cout << fixed << v3[i] << " ";
        }
        outputFile << "] \n \n";
        cout << "] \n \n";
    }  else {
        cout << "Unable to open file.\n";
        return;
    }
}

/* <!-  Function to print a matrix[3] on a file */
void printM33 (string fileName, string varName, double M33[3][3])
{
    ofstream outputFile;
    outputFile.open(fileName, std::ios_base::app);
    cout.precision(dbl::max_digits10);
    if (outputFile.is_open())
    {
        outputFile << varName <<" = \n [\n ";
        cout << varName <<" = \n [\n ";
        for (int i = 0; i < 3; i++)
        {
            outputFile << "\t"<< fixed << M33[0][i] << " " << fixed << M33[1][i] << " " << fixed << M33[2][i] << "\n ";
            cout << "\t"<< fixed << M33[0][i] << " " << fixed << M33[1][i] << " " << fixed << M33[2][i] << "\n ";
        }
        outputFile << "] \n\n";
        cout << "] \n\n";
    }  else {
        cout << "Unable to open file.\n";
        return;
    }
}

