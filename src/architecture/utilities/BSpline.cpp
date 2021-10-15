/*
 ISC License

 Copyright (c) 2021, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#include "BSpline.h"
#include <architecture/utilities/avsEigenSupport.h>
#include <architecture/utilities/linearAlgebra.h>
#include "architecture/utilities/rigidBodyKinematics.h"
#include <iostream>
#include <cstring>
#include <math.h>

/*!  */
InputDataSet::InputDataSet()
{
    return;
}

/*!  */
InputDataSet::InputDataSet(Eigen::VectorXd X1, Eigen::VectorXd X2, Eigen::VectorXd X3)
{
    this->X1 = X1;
    this->X2 = X2;
    this->X3 = X3;
    this->XDot_0_flag = false;
    this->XDot_N_flag = false;
    this->XDDot_0_flag = false;
    this->XDDot_N_flag = false;
    this->T_flag = false;
    return;
}

/*!  */
InputDataSet::~InputDataSet()
{
    return;
}

/*!  */
void InputDataSet::setXDot_0(Eigen::Vector3d XDot_0) {this->XDot_0 = XDot_0; this->XDot_0_flag = true; return;}

/*!  */
void InputDataSet::setXDot_N(Eigen::Vector3d XDot_N) {this->XDot_N = XDot_N; this->XDot_N_flag = true; return;}

/*!  */
void InputDataSet::setXDDot_0(Eigen::Vector3d XDDot_0) {this->XDDot_0 = XDDot_0; this->XDDot_0_flag = true; return;}

/*!  */
void InputDataSet::setXDDot_N(Eigen::Vector3d XDDot_N) {this->XDDot_N = XDDot_N; this->XDDot_N_flag = true; return;}

/*!  */
void InputDataSet::setT(Eigen::VectorXd T) {this->T = T; this->T_flag = true; return;}


/*!  */
OutputDataSet::OutputDataSet()
{
    return;
}

/*!  */
OutputDataSet::~OutputDataSet()
{
    return;
}

/*! */
void interpolate(InputDataSet Input, int Num, double avgXDot, int P, OutputDataSet *Output)
{
    int N = Input.X1.size() - 1;

    Eigen::VectorXd T(N+1);
    if (Input.T_flag == true) {
        T = Input.T;
    }
    else {
        T[0] = 0;
        for (int n = 1; n < N+1; n++) {
            T[n] = T[n-1] + pow( (pow(Input.X1[n]-Input.X1[n-1], 2) + pow(Input.X2[n]-Input.X2[n-1], 2) + pow(Input.X3[n]-Input.X3[n-1], 2)), 0.5 );
        }
    }

    double Ttot = T[N];

    // build uk vector
    Eigen::VectorXd uk(N+1);
    for (int n = 0; n < N+1; n++) {
        uk[n] = T[n] / Ttot;
    }

    int K = 0;  // number of endpoint derivatives
    if (Input.XDot_0_flag == true) {K += 1;}
    if (Input.XDot_N_flag == true) {K += 1;}
    if (Input.XDDot_0_flag == true) {K += 1;}
    if (Input.XDDot_N_flag == true) {K += 1;}

    int M = N + P + K + 1;

    // build knot vector U
    Eigen::VectorXd U(M+1);
    double u;
    for (int p = 0; p < P+1; p++) {
        U[p] = 0;
    }

    for (int j = 0; j < M-2*P-1; j++) {
        u = 0.0;
        for (int i = j; i < j+P; i++) {
            if (i >= uk.size()) {
                u += uk[N] / P;
            }
            else {
                u += uk[i] / P;
            }
        U[P+j+1] = u;
        }
    }

    for (int p = 0; p < P+1; p++) {
        U[M-P+p] = 1;   // should be 1
    }

    // build stiffness matrix A
    Eigen::MatrixXd A(N+K+1,N+K+1);
    // populate A with zeros
    for (int n = 0; n < N+K+1; n++) {
        for (int m = 0; m < N+K+1; m++) {
            A(n,m) = 0;
        }
    }
    // build vectors Q1, Q2, Q3
    Eigen::VectorXd Q1(N+K+1), Q2(N+K+1), Q3(N+K+1);
    int n = -1;
    if (Input.XDot_0_flag == true) {
        n += 1;
        A(n,0) = -1;
        A(n,1) =  1;
        Q1[n] = U[P+1] / P * Input.XDot_0[0] * Ttot;
        Q2[n] = U[P+1] / P * Input.XDot_0[1] * Ttot;
        Q3[n] = U[P+1] / P * Input.XDot_0[2] * Ttot;
    }
    if (Input.XDDot_0_flag == true) {
        n += 1;
        A(n,0) = U[P+2];
        A(n,1) = -(U[P+1] + U[P+2]);
        A(n,2) = U[P+1];
        Q1[n] = ( pow(U[P+1],2) * U[P+2] / (P*(P-1)) ) * Input.XDDot_0[0] * pow(Ttot,2);
        Q2[n] = ( pow(U[P+1],2) * U[P+2] / (P*(P-1)) ) * Input.XDDot_0[1] * pow(Ttot,2);
        Q3[n] = ( pow(U[P+1],2) * U[P+2] / (P*(P-1)) ) * Input.XDDot_0[2] * pow(Ttot,2);
    }
    n += 1;
    int m = -1;
    int n0 = n;
    Eigen::VectorXd NN(N+K+1), NN1(N+K+1), NN2(N+K+1);
    for (n = n0; n < N+n0+1; n++) {
        m += 1;
        basisFunction(uk[m], U, N+K+1, P, &NN[0], &NN1[0], &NN2[0]);
        for (int b = 0; b < N+K+1; b++) {
            A(n,b) = NN[b];
        }
        Q1[n] = Input.X1[m];
        Q2[n] = Input.X2[m];
        Q3[n] = Input.X3[m];
    }
    n = N+n0;
    if (Input.XDDot_N_flag == true) {
        n += 1;
        A(n,N+K-2) = 1 - U[M-P-1];
        A(n,N+K-1) = -(2 - U[M-P-1] - U[M-P-2]);
        A(n,N+K) = 1 - U[M-P-2];
        Q1[n] = ( pow((1-U[M-P-1]),2) * (1-U[M-P-2]) / (P*(P-1)) ) * Input.XDDot_N[0] * pow(Ttot,2);
        Q2[n] = ( pow((1-U[M-P-1]),2) * (1-U[M-P-2]) / (P*(P-1)) ) * Input.XDDot_N[1] * pow(Ttot,2);
        Q3[n] = ( pow((1-U[M-P-1]),2) * (1-U[M-P-2]) / (P*(P-1)) ) * Input.XDDot_N[2] * pow(Ttot,2);
    }
    if (Input.XDot_N_flag == true) {
        n += 1;
        A(n,N+K-1) = -1;
        A(n,N+K) = 1;
        Q1[n] = (1-U[M-P-1]) / P * Input.XDot_N[0] * Ttot;
        Q2[n] = (1-U[M-P-1]) / P * Input.XDot_N[1] * Ttot;
        Q3[n] = (1-U[M-P-1]) / P * Input.XDot_N[2] * Ttot;
    }
    
    Eigen::MatrixXd B = A.inverse();
    Eigen::VectorXd C1 = B * Q1;
    Eigen::VectorXd C2 = B * Q2;
    Eigen::VectorXd C3 = B * Q3;

    double dt = 1.0 / (Num - 1);
    double t = 0;
    Output->T.resize(Num);
    Output->X1.resize(Num);
    Output->X2.resize(Num);
    Output->X3.resize(Num);
    Output->XD1.resize(Num);
    Output->XD2.resize(Num);
    Output->XD3.resize(Num);
    Output->XDD1.resize(Num);
    Output->XDD2.resize(Num);    
    Output->XDD3.resize(Num);
    for (int i = 0; i < Num; i++) {
        basisFunction(t, U, N+K+1, P, &NN[0], &NN1[0], &NN2[0]);
        Output->T[i] = t * Ttot;
        Output->X1[i] = NN.dot(C1);
        Output->X2[i] = NN.dot(C2);
        Output->X3[i] = NN.dot(C3);
        Output->XD1[i]  = NN1.dot(C1) / Ttot;
        Output->XD2[i]  = NN1.dot(C2) / Ttot;
        Output->XD3[i]  = NN1.dot(C3) / Ttot;
        Output->XDD1[i] = NN2.dot(C1) / pow(Ttot,2);
        Output->XDD2[i] = NN2.dot(C2) / pow(Ttot,2);
        Output->XDD3[i] = NN2.dot(C3) / pow(Ttot,2);
        t += dt;
    }
    // override last point to avoid errors
    Output->T[Num-1] = Ttot;
    Output->X1[Num-1] = Input.X1[N];
    Output->X2[Num-1] = Input.X2[N];
    Output->X3[Num-1] = Input.X3[N];
    if (Input.XDot_N_flag == true) {
        Output->XD1[Num-1] = Input.XDot_N[0];
        Output->XD2[Num-1] = Input.XDot_N[1];
        Output->XD3[Num-1] = Input.XDot_N[2];
    }
    else {
        Output->XD1[Num-1] = ( (Output->X1[Num-3]-Output->X1[Num-1])*0.5 - (Output->X1[Num-2]-Output->X1[Num-1])*2 ) / (dt * Ttot);
        Output->XD2[Num-1] = ( (Output->X2[Num-3]-Output->X2[Num-1])*0.5 - (Output->X2[Num-2]-Output->X2[Num-1])*2 ) / (dt * Ttot);
        Output->XD3[Num-1] = ( (Output->X3[Num-3]-Output->X3[Num-1])*0.5 - (Output->X3[Num-2]-Output->X3[Num-1])*2 ) / (dt * Ttot);      
    }
    if (Input.XDDot_N_flag == true) {
        Output->XDD1[Num-1] = Input.XDDot_N[0];
        Output->XDD2[Num-1] = Input.XDDot_N[1];
        Output->XDD3[Num-1] = Input.XDDot_N[2];
    }
    else {
        Output->XDD1[Num-1] = ( (Output->XD1[Num-3]-Output->XD1[Num-1])*0.5 - (Output->XD1[Num-2]-Output->XD1[Num-1])*2 ) / (dt * Ttot);
        Output->XDD2[Num-1] = ( (Output->XD2[Num-3]-Output->XD2[Num-1])*0.5 - (Output->XD2[Num-2]-Output->XD2[Num-1])*2 ) / (dt * Ttot);
        Output->XDD3[Num-1] = ( (Output->XD3[Num-3]-Output->XD3[Num-1])*0.5 - (Output->XD3[Num-2]-Output->XD3[Num-1])*2 ) / (dt * Ttot);      
    }

    return;
}

/*! */
void basisFunction(double t, Eigen::VectorXd U, int I, int P, double *NN, double *NN1, double *NN2)
{
    Eigen::MatrixXd N(U.size()-1, P+1);
    Eigen::MatrixXd N1(U.size()-1, P+1);
    Eigen::MatrixXd N2(U.size()-1, P+1);

    /* populate matrices with zeros */
    for (int i = 0; i < U.size()-1; i++) {
        for (int p = 0; p < P+1; p++) {
            N(i,p)  = 0;
            N1(i,p) = 0;
            N2(i,p) = 0;
        }
    }
    /* zero order */
    for (int i = 0; i < U.size()-1; i++) {
        if ( (t >= U(i)) && (t <= U(i+1)) ) {
            N(i,0) = 1;
        }
    }
    /* higher order */
    for (int p = 1; p < P+1; p++) {
        for (int i = 0; i < U.size()-P-1; i++) {
            if (U[i+p]-U[i] != 0) {
                N(i,p)  += (t-U[i]) / (U[i+p]-U[i]) * N(i,p-1);
                N1(i,p) += p / (U[i+p]-U[i]) * N(i,p-1);
                N2(i,p) += p / (U[i+p]-U[i]) * N1(i,p-1);
            }
            if (U[i+p+1]-U[i+1] != 0) {
                N(i,p)  += (U[i+p+1]-t) / (U[i+p+1]-U[i+1]) * N(i+1,p-1);
                N1(i,p) -= p / (U[i+p+1]-U[i+1]) * N(i+1,p-1);
                N2(i,p) -= p / (U[i+p+1]-U[i+1]) * N1(i+1,p-1);
            }
        }
    }
    for (int i = 0; i < I; i++) {
        *(NN+i)  = N(i,P);
        *(NN1+i) = N1(i,P);
        *(NN2+i) = N2(i,P);
    }

    return;
}