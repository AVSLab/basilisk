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

// Checking comment

#include "BSpline.h"
#include <architecture/utilities/avsEigenSupport.h>
#include <iostream>
#include <cstring>
#include <math.h>

/*! This constructor initializes an Input structure for BSpline interpolation */
InputDataSet::InputDataSet()
{
    return;
}

/*! The constructor requires 3 N-dimensional vectors containing the coordinates of the waypoints */
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
    this->AvgXDot_flag = false;
    this->W_flag = false;
    this->LS_Dot = false;
    uint64_t N1 = (uint64_t) X1.size();
    uint64_t N2 = (uint64_t) X2.size();
    uint64_t N3 = (uint64_t) X3.size();

    if ((N1 != N2) || (N1 != N3) || (N2 != N3)) {
        std::cout << "Error in BSpline.InputDataSet: \n the Input coordinate vectors X1, X2, X3 have different sizes. \n";
    }

    return;
}

/*! Generic destructor */
InputDataSet::~InputDataSet()
{
    return;
}

/*! Set the first derivative of the starting point (optional) */
void InputDataSet::setXDot_0(Eigen::Vector3d XDot_0) {this->XDot_0 = XDot_0; this->XDot_0_flag = true; return;}

/*! Set the first derivative of the last point (optional) */
void InputDataSet::setXDot_N(Eigen::Vector3d XDot_N) {this->XDot_N = XDot_N; this->XDot_N_flag = true; return;}

/*! Set the second derivative of the starting point (optional) */
void InputDataSet::setXDDot_0(Eigen::Vector3d XDDot_0) {this->XDDot_0 = XDDot_0; this->XDDot_0_flag = true; return;}

/*! Set the second derivative of the last point (optional) */
void InputDataSet::setXDDot_N(Eigen::Vector3d XDDot_N) {this->XDDot_N = XDDot_N; this->XDDot_N_flag = true; return;}

/*! Set the time tags for each waypoint (optional). Cannot be imposed together with avg velocity norm below */
void InputDataSet::setT(Eigen::VectorXd T) {this->T = T; this->T_flag = true; this->AvgXDot_flag = false; return;}

/*! Set the average velocity norm (optional). Cannot be imposed together with time tag vector above */
void InputDataSet::setAvgXDot(double AvgXDot) {this->AvgXDot = AvgXDot; this->AvgXDot_flag = true; this->T_flag = false; return;}

/*! Set the weights for each waypoint. Weights are used in the LS approximation */
void InputDataSet::setW(Eigen::VectorXd W) {this->W = W; this->W_flag = true; return;}

/*! Set LS_dot to true which means first derivative LS approximation occurs (optional) */
void InputDataSet::setLS_Dot() {this->LS_Dot = true; return;}


/*! This constructor initializes an Output structure for BSpline interpolation */
OutputDataSet::OutputDataSet()
{
    return;
}

/*! Generic destructor */
OutputDataSet::~OutputDataSet()
{
    return;
}

/*! This method returns x, xDot and xDDot at the desired input time T */
void OutputDataSet::getData(double T, double x[3], double xDot[3], double xDDot[3])
{
    int N = (int) this->T.size() - 1;
    double Ttot = this->T[N];

    // if T < Ttot calculalte the values
    if (T <= Ttot) {
        double t = T / Ttot;
        int Q = (int) this->C1.size();
        Eigen::VectorXd NN(Q), NN1(Q), NN2(Q);
        basisFunction(t, this->U, Q, this->P, &NN[0], &NN1[0], &NN2[0]);
        x[0] = NN.dot(this->C1);
        x[1] = NN.dot(this->C2);
        x[2] = NN.dot(this->C3);
        xDot[0] = NN1.dot(this->C1) / Ttot;
        xDot[1] = NN1.dot(this->C2) / Ttot;
        xDot[2] = NN1.dot(this->C3) / Ttot;
        xDDot[0] = NN2.dot(this->C1) / pow(Ttot,2);
        xDDot[1] = NN2.dot(this->C2) / pow(Ttot,2);
        xDDot[2] = NN2.dot(this->C3) / pow(Ttot,2);
    }
    // if t > Ttot return final value with zero derivatives
    else {
        x[0] = this->X1[N];
        x[1] = this->X2[N];
        x[2] = this->X3[N];
        xDot[0] = 0;
        xDot[1] = 0;
        xDot[2] = 0;
        xDDot[0] = 0;
        xDDot[1] = 0;
        xDDot[2] = 0;
    }
}

/*! This method returns single coordinates of x, xDot and xDDot at the desired input time T. */
/*! It is designed to be accessible from Python */
double OutputDataSet::getStates(double T, int derivative, int index)
{
    int N = (int) this->T.size()-1;
    double Ttot = this->T[N];

    // if T < Ttot calculalte the values
    if (T <= Ttot) {
        double t = T / Ttot;
        int Q = (int) this->C1.size();
        Eigen::VectorXd NN(Q), NN1(Q), NN2(Q);
        basisFunction(t, this->U, Q, this->P, &NN[0], &NN1[0], &NN2[0]);

        switch (derivative) {
            case 0 :
                switch(index) {
                    case 0 :
                        return NN.dot(this->C1);
                    case 1 :
                        return NN.dot(this->C2);
                    case 2 :
                        return NN.dot(this->C3);
                    default :
                        std::cout << "Error in Output.getStates: invalid index \n";
                        return 1000;
                }
            case 1 :
                switch(index) {
                    case 0 :
                        return NN1.dot(this->C1) / Ttot;
                    case 1 :
                        return NN1.dot(this->C2) / Ttot;
                    case 2 :
                        return NN1.dot(this->C3) / Ttot;
                    default :
                        std::cout << "Error in Output.getStates: invalid index \n";
                        return 1000;
                }
            case 2 :
                switch(index) {
                    case 0 :
                        return NN2.dot(this->C1) / pow(Ttot,2);
                    case 1 :
                        return NN2.dot(this->C2) / pow(Ttot,2);
                    case 2 :
                        return NN2.dot(this->C3) / pow(Ttot,2);
                    default :
                        std::cout << "Error in Output.getStates: invalid index \n";
                        return 1000;
                }
            default :
                std::cout << "Error in Output.getStates: invalid derivative \n";
                return 1000;
        }
    }
    // if t > Ttot return final value with zero derivatives
    else {
        switch (derivative) {
            case 0 :
                switch(index) {
                    case 0 :
                        return this->X1[N];
                    case 1 :
                        return this->X2[N];
                    case 2 :
                        return this->X3[N];
                    default :
                        std::cout << "Error in Output.getStates: invalid index \n";
                        return 1000;
                }
            case 1 :
                switch(index) {
                    case 0 :
                        return 0;
                    case 1 :
                        return 0;
                    case 2 :
                        return 0;
                    default :
                        std::cout << "Error in Output.getStates: invalid index \n";
                        return 1000;
                }
            case 2 :
                switch(index) {
                    case 0 :
                        return 0;
                    case 1 :
                        return 0;
                    case 2 :
                        return 0;
                    default :
                        std::cout << "Error in Output.getStates: invalid index \n";
                        return 1000;
                }
            default :
                std::cout << "Error in Output.getStates: invalid derivative \n";
                return 1000;
        }
    }
}

/*! This function takes the Input structure, performs the BSpline interpolation and outputs the result into Output structure */
void interpolate(InputDataSet Input, int Num, int P, OutputDataSet *Output)
{
    Output->P = P;
    
    std::cout << "interpolation has begun";
    
    // q = number of waypoints - 1
    int q = (int) Input.X1.size() - 1;
    
    // T = time tags; if not specified, it is computed from a cartesian distance assuming a constant velocity norm on average
    Eigen::VectorXd T(q+1);
    double S = 0;
    if (Input.T_flag == true) {
        T = Input.T;
    }
    else {
        T[0] = 0;
        for (int n = 1; n < q+1; n++) {
            T[n] = T[n-1] + pow( (pow(Input.X1[n]-Input.X1[n-1], 2) + pow(Input.X2[n]-Input.X2[n-1], 2) + pow(Input.X3[n]-Input.X3[n-1], 2)), 0.5 );
            S += T[n] - T[n-1];
        }
    }
    if (Input.AvgXDot_flag == true) {
        for (int n = 0; n < q+1; n++) {
            T[n] = T[n] / T[q] * S / Input.AvgXDot;
        }
    }
    
    double Ttot = T[q];

    // build uk vector: normalized waypoint time tags
    Eigen::VectorXd uk(q+1);
    for (int n = 0; n < q+1; n++) {
        uk[n] = T[n] / Ttot;
    }

    // K = number of endpoint derivatives
    int K = 0;
    if (Input.XDot_0_flag == true) {K += 1;}
    if (Input.XDot_N_flag == true) {K += 1;}
    if (Input.XDDot_0_flag == true) {K += 1;}
    if (Input.XDDot_N_flag == true) {K += 1;}
    
    // The maximum polynomial order is N + K. If a higher order is requested, print a BSK_ERROR
    if (P > q + K) {
        std::cout << "Error in BSpline.interpolate: \n the desired polynomial order P is too high. Mass matrix A will be singular. \n" ;
    }

    int M = q + P + K + 1;

    // build knot vector U of size M + 1
    Eigen::VectorXd U(M+1);
    double u;
    for (int p = 0; p < P+1; p++) {
        U[p] = 0;
    }
    for (int j = 0; j < M-2*P-1; j++) {
        u = 0.0;
        for (int i = j; i < j+P; i++) {
            if (i >= uk.size()) {
                u += uk[q] / P;
            }
            else {
                u += uk[i] / P;
            }
        U[P+j+1] = u;
        }
    }
    for (int p = 0; p < P+1; p++) {
        U[M-P+p] = 1;
    }

    // build stiffness matrix A of size (N+K+1)x(N+K+1)
    Eigen::MatrixXd A(q+K+1,q+K+1);
    // build vectors Q1, Q2, Q3 (left hand side of linear system)
    Eigen::VectorXd Q1(q+K+1), Q2(q+K+1), Q3(q+K+1);
    // populate A with zeros
    for (int n = 0; n < q+K+1; n++) {
        for (int m = 0; m < q+K+1; m++) {
            A(n,m) = 0;
        }
    }
    int n = -1;
    // constrain first derivative at starting point
    if (Input.XDot_0_flag == true) {
        n += 1;
        A(n,0) = -1;
        A(n,1) =  1;
        Q1[n] = U[P+1] / P * Input.XDot_0[0] * Ttot;
        Q2[n] = U[P+1] / P * Input.XDot_0[1] * Ttot;
        Q3[n] = U[P+1] / P * Input.XDot_0[2] * Ttot;
    }
    // constrain second derivative at starting point
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
    Eigen::VectorXd NN(q+K+1), NN1(q+K+1), NN2(q+K+1);
    // constrain waypoints
    for (n = n0; n < q+n0+1; n++) {
        m += 1;
        basisFunction(uk[m], U, q+K+1, P, &NN[0], &NN1[0], &NN2[0]);
        for (int b = 0; b < q+K+1; b++) {
            A(n,b) = NN[b];
        }
        Q1[n] = Input.X1[m];
        Q2[n] = Input.X2[m];
        Q3[n] = Input.X3[m];
    }
    n = q+n0;
    // constrain second derivative at final point
    if (Input.XDDot_N_flag == true) {
        n += 1;
        A(n,q+K-2) = 1 - U[M-P-1];
        A(n,q+K-1) = -(2 - U[M-P-1] - U[M-P-2]);
        A(n,q+K) = 1 - U[M-P-2];
        Q1[n] = ( pow((1-U[M-P-1]),2) * (1-U[M-P-2]) / (P*(P-1)) ) * Input.XDDot_N[0] * pow(Ttot,2);
        Q2[n] = ( pow((1-U[M-P-1]),2) * (1-U[M-P-2]) / (P*(P-1)) ) * Input.XDDot_N[1] * pow(Ttot,2);
        Q3[n] = ( pow((1-U[M-P-1]),2) * (1-U[M-P-2]) / (P*(P-1)) ) * Input.XDDot_N[2] * pow(Ttot,2);
    }
    // constrain first derivative at final point
    if (Input.XDot_N_flag == true) {
        n += 1;
        A(n,q+K-1) = -1;
        A(n,q+K) = 1;
        Q1[n] = (1-U[M-P-1]) / P * Input.XDot_N[0] * Ttot;
        Q2[n] = (1-U[M-P-1]) / P * Input.XDot_N[1] * Ttot;
        Q3[n] = (1-U[M-P-1]) / P * Input.XDot_N[2] * Ttot;
    }

    // solve linear systems
    Eigen::MatrixXd B = A.inverse();
    Eigen::VectorXd C1 = B * Q1;
    Eigen::VectorXd C2 = B * Q2;
    Eigen::VectorXd C3 = B * Q3;

    Output->U = U;
    Output->C1 = C1;
    Output->C2 = C2;
    Output->C3 = C3;

    double dt = 1.0 / (Num - 1);
    double t = 0;
    // store the interpolated trajectory information into Output structure
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
        basisFunction(t, U, q+K+1, P, &NN[0], &NN1[0], &NN2[0]);
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

    return;
}

/*! This function takes the Input structure, performs the BSpline LS approximation and outputs the result into Output structure */
void approximate(InputDataSet Input, int Num, int n, int P, OutputDataSet *Output)
{
    
    std::cout << "Approximate starts"<<std::endl;
    
    Output->P = P;
    
    // q = number of waypoints - 1
    int q = (int) Input.X1.size() - 1;
    
    
    
//    for (int a = 0; a < q+1; a++) {
//        std::cout<<Input.X1[a]<<","<<Input.X2[a]<<","<<Input.X3[a]<<std::endl;
//    }
    

    //std::cout << "The value of q is "<<q<<std::endl;
        
    // T = time tags; if not specified, it is computed from a cartesian distance assuming a constant velocity norm on average
    Eigen::VectorXd T(q+1);
    double S = 0;
    if (Input.T_flag == true) {
        T = Input.T;
        for (int a = 1; a < q+1; a++) {
            T[a] = T[a-1] + pow( (pow(Input.X1[a]-Input.X1[a-1], 2) + pow(Input.X2[a]-Input.X2[a-1], 2) + pow(Input.X3[a]-Input.X3[a-1], 2)), 0.5 );
            S += T[a] - T[a-1];
        }
        T = Input.T;
    }
    else {
        T[0] = 0;
        for (int a = 1; a < q+1; a++) {
            T[a] = T[a-1] + pow( (pow(Input.X1[a]-Input.X1[a-1], 2) + pow(Input.X2[a]-Input.X2[a-1], 2) + pow(Input.X3[a]-Input.X3[a-1], 2)), 0.5 );
            S += T[a] - T[a-1];
            
        }
    if (Input.AvgXDot_flag == true) {
        for (int b = 0; b < q+1; b++) {
            T[b] = T[b] / T[q] * S / Input.AvgXDot;
        }
    }
        }
    
    // Need to access the timestaps for the waypoints for the final graph
    //if (Input.T_flag == false) {
    Output->T_way_calc = T;
    //}
    
    double Ttot = T[q];
    
    
    for (int a=0; a<q+1;a++) {
        std::cout<<T[a]<<std::endl;
    }
    

    // build uk vector: normalized waypoint time tags
    Eigen::VectorXd uk(q+1);
    for (int c = 0; c < q+1; c++) {
        uk[c] = T[c] / Ttot;
    }
    
    
    // The maximum polynomial order is N + K. If a higher order is requested, print a BSK_ERROR
    if (P > n) {
        std::cout << "Error in BSpline.approximate: \n the desired polynomial order P can't be higher than the number of control points n. \n" ;
    }

    // build knot vector U of size n + P + 2
    Eigen::VectorXd U(n+P+2);
    double d, alpha;
    int i;
    d = ((double)(q+1)) / ((double)abs(n-P+1));
    for (int p = 0; p < P+1; p++) {
        U[p] = 0;
    }
    for (int j = 1; j < n-P+1; j++) {
        i = int(j*d);
        alpha = j*d - i;
        U[P+j] = (1-alpha)*uk[i-1] + alpha*uk[i];
        if (j==1 & U[P+1] < uk[1]) {
            U[P+1] = uk[1];
        }
        if (j == n-P & U[n] > uk[q-1]) {
            U[n] = uk[q-1];
        }
        
    }
    
    for (int p = 0; p < P+1; p++) {
        U[n+p+1] = 1; // Got rid of the n here which fixed the error, now we are matching but next results are nan
    }
    
    
    // Calculate 1st Derivatives:
    
    Eigen::VectorXd X1_primehat(q-1);
    Eigen::VectorXd X2_primehat(q-1);
    Eigen::VectorXd X3_primehat(q-1);

    int index = 0;
    
    // Calculate X Prime Hat Derivatives
    
    //Central Finite Derivatives
    for (int k = 1;k<q-1;k++) {
        X1_primehat[index] = (uk[k+1]-uk[k])/(uk[k+1]-uk[k-1])*(Input.X1[k]-Input.X1[k-1])/(uk[k]-uk[k-1])+(uk[k]-uk[k-1])/(uk[k+1]-uk[k-1])*(Input.X1[k+1]-Input.X1[k])/(uk[k+1]-uk[k]);
        X2_primehat[index] =(uk[k+1]-uk[k])/(uk[k+1]-uk[k-1])*(Input.X2[k]-Input.X2[k-1])/(uk[k]-uk[k-1])+(uk[k]-uk[k-1])/(uk[k+1]-uk[k-1])*(Input.X2[k+1]-Input.X2[k])/(uk[k+1]-uk[k]);
        X3_primehat[index] =(uk[k+1]-uk[k])/(uk[k+1]-uk[k-1])*(Input.X3[k]-Input.X3[k-1])/(uk[k]-uk[k-1])+(uk[k]-uk[k-1])/(uk[k+1]-uk[k-1])*(Input.X3[k+1]-Input.X3[k])/(uk[k+1]-uk[k]);
        index++;
    }
    
    
    // Calculate X Prime Derivatives
    Eigen::VectorXd X1_prime(q+1),X2_prime(q+1),X3_prime(q+1);
    
    // Set Initial Derivative
    X1_prime[0] = Input.XDot_0[0];
    X2_prime[0] = Input.XDot_0[1];
    X3_prime[0] = Input.XDot_0[2];
    
    // Set Final Derivative
    X1_prime[q] = Input.XDot_N[0];
    X2_prime[q] = Input.XDot_N[1];
    X3_prime[q] = Input.XDot_N[2];
    

    for (int k = 1;k<q;k++) {
        double a = double(X1_primehat[k]);
        double b = double(X2_primehat[k]);
        double c = double(X3_primehat[k]);
        double mag = pow(a,2.0)+pow(b,2.0)+pow(c,2.0);
        mag = pow(mag,0.5);
        mag = abs((mag));
        double temp;
        if (mag != 0) {
            std::cout<<"Non zero mag"<<std::endl;
            temp =0.03*Ttot/mag;
        }
        else {
            std::cout<<"zero mag"<<std::endl;
            temp = 0;
        }
        X1_prime[k] = X1_primehat[k]*temp;
        X2_prime[k] = X2_primehat[k]*temp;
        X3_prime[k] = X3_primehat[k]*temp;
    }
    
    
    // K = number of endpoint derivatives
    int K = 0;
    if (Input.XDot_0_flag == true) {K += 1;}
    if (Input.XDot_N_flag == true) {K += 1;}
    if (Input.XDDot_0_flag == true) {K += 1;}
    if (Input.XDDot_N_flag == true) {K += 1;}

    // build stiffness matrix MD of size (K+2)x(K+2)
    Eigen::MatrixXd MD(K+2,K+2);
    // build vectors T1, T2, T3 (left hand side of linear system)
    Eigen::VectorXd T1(K+2), T2(K+2), T3(K+2);
    // populate MD with zeros
    for (int e = 0; e < K+2; e++) {
        for (int m = 0; m < K+2; m++) {
            MD(e,m) = 0;
        }
    }
    Eigen::VectorXd NN(n+1), NN1(n+1), NN2(n+1);
    basisFunction(uk[0], U, n+1, P, &NN[0], &NN1[0], &NN2[0]);
     i = 0;
    MD(0,0) = NN[0];
    T1[0] = Input.X1[0];
    T2[0] = Input.X2[0];
    T3[0] = Input.X3[0];
    // constrain first derivative at starting point
    if (Input.XDot_0_flag == true) {
        i += 1;
        MD(i,0) = NN1[0];
        MD(i,1) = NN1[1];
        T1[i] = Input.XDot_0[0] * Ttot;
        T2[i] = Input.XDot_0[1] * Ttot;
        T3[i] = Input.XDot_0[2] * Ttot;
    }
    // constrain second derivative at starting point
    if (Input.XDDot_0_flag == true) {
        i += 1;
        MD(i,0) = NN2[0];
        MD(i,1) = NN2[1];
        MD(i,2) = NN2[2];
        T1[i] = Input.XDDot_0[0] * pow(Ttot,2);
        T2[i] = Input.XDDot_0[1] * pow(Ttot,2);
        T3[i] = Input.XDDot_0[2] * pow(Ttot,2);
    }
    basisFunction(uk[q], U, n+1, P, &NN[0], &NN1[0], &NN2[0]);
    // constrain second derivative at ending point
    if (Input.XDDot_N_flag == true) {
        i += 1;
        MD(i,K-1) = NN2[n-2];
        MD(i,K)   = NN2[n-1];
        MD(i,K+1) = NN2[n];
        T1[K-1] = Input.XDDot_N[0] * pow(Ttot,2);
        T2[K]   = Input.XDDot_N[1] * pow(Ttot,2);
        T3[K+1] = Input.XDDot_N[2] * pow(Ttot,2);
    }
    // constrain first derivative at ending point
    if (Input.XDot_N_flag == true) {
        i += 1;
        MD(i,K)   = NN1[n-1];
        MD(i,K+1) = NN1[n];
        T1[i] = Input.XDot_N[0] * Ttot;
        T2[i] = Input.XDot_N[1] * Ttot;
        T3[i] = Input.XDot_N[2] * Ttot;
    }
    i += 1;
    MD(i,K+1) = NN[n];
//    std::cout<<"NN[n] "<<NN[n]<<std::endl;
    T1[i] = Input.X1[q];
    T2[i] = Input.X2[q];
    T3[i] = Input.X3[q];
    
    // solve linear systems
    Eigen::MatrixXd B = MD.inverse();
    
    Eigen::VectorXd C1_1 = B * T1;
    Eigen::VectorXd C2_1 = B * T2;
    Eigen::VectorXd C3_1 = B * T3;
    
    // populate Rk vectors with the base points for LS minimization
    Eigen::VectorXd rhok1(2*q-2), rhok2(2*q-2), rhok3(2*q-2);
    for (int c = 1; c < q; c++) {
        basisFunction(uk[c], U, n+1, P, &NN[0], &NN1[0], &NN2[0]);
        rhok1[c-1] = Input.X1[c] - NN[0]*C1_1[0] - NN[n]*C1_1[K+1];
        rhok1[c+q-2] = X1_prime[c] - NN1[0]*C1_1[0] - NN1[n]*C1_1[K+1];
        //std::cout<<"The value of c "<<c<<" and X1_prime[c] is "<<X1_prime[c]<<std::endl;
        rhok2[c-1] = Input.X2[c] - NN[0]*C2_1[0] - NN[n]*C2_1[K+1];
        rhok2[c+q-2] = X2_prime[c] - NN1[0]*C2_1[0] - NN1[n]*C2_1[K+1];
        //std::cout<<"The value of c "<<c<<" and X2_prime[c] is "<<X2_prime[c]<<std::endl;
        rhok3[c-1] = Input.X3[c] - NN[0]*C3_1[0] - NN[n]*C3_1[K+1];
        rhok3[c+q-2] = X3_prime[c] - NN1[0]*C3_1[0] - NN1[n]*C3_1[K+1];
        //std::cout<<"The value of c "<<c<<" and X3_prime[c] is "<<X3_prime[c]<<std::endl;
        if (Input.XDot_0_flag == true) {
            //std::cout<<"The initial value is rhok1[c-1] "<<rhok1[c-1]<<std::endl;
            rhok1[c-1] -= NN[1]*C1_1[1];
            //std::cout<<"The new value is rhok1[c-1]"<<rhok1[c-1]<<std::endl;
            //std::cout<<"The initial value is rhok1[c+q+2] "<<rhok2[c+q-2]<<std::endl;
            rhok1[c+q-2] -= NN1[1]*C1_1[1];
            //std::cout<<"The new value is rhok1[c+q-2] "<<rhok2[c+q-2]<<std::endl;
            //std::cout<<"The new value is rhok2[c-1] "<<rhok2[c-1]<<std::endl;
            rhok2[c-1] -= NN[1]*C2_1[1];
            //std::cout<<"The new value is rhok2[c-1] "<<rhok2[c-1]<<std::endl;
            //std::cout<<"The initial value is rhok2[c+q-2]"<<rhok2[c+q-2]<<std::endl;
            rhok2[c+q-2] -= NN1[1]*C2_1[1];
            //std::cout<<"The new value is rhok2[c+q-2]"<<rhok2[c+q-2]<<std::endl;
            //std::cout<<"The initial value is rhok3[c-1]"<<rhok3[c-1]<<std::endl;
            rhok3[c-1] -= NN[1]*C3_1[1];
            //std::cout<<"The new value is rhok3[c-1]"<<rhok3[c-1]<<std::endl;
            //std::cout<<"The initial value is rhok3[c+q-2]"<<rhok3[c+q-2]<<std::endl;
            rhok3[c+q-2] -= NN1[1]*C3_1[1];
            //std::cout<<"The initial value is rhok3[c+q-2]"<<rhok3[c+q-2]<<std::endl;
        }
        if (Input.XDDot_0_flag == true) {
            rhok1[c-1] -= NN[2]*C1_1[2];
            rhok1[c+q-2] -= NN1[2]*C1_1[2];
            rhok2[c-1] -= NN[2]*C2_1[2];
            rhok2[c+q-2] -= NN1[2]*C2_1[2];
            rhok3[c-1] -= NN[2]*C3_1[2];
            rhok3[c+q-2] -= NN1[2]*C3_1[2];
        }
        if (Input.XDDot_N_flag == true) {
            rhok1[c-1] -= NN[n-2]*C1_1[K-1];
            rhok1[c+q-2] -= NN1[n-2]*C1_1[K-1];
            rhok2[c-1] -= NN[n-2]*C2_1[K-1];
            rhok2[c+q-2] -= NN1[n-2]*C2_1[K-1];
            rhok3[c-1] -= NN[n-2]*C3_1[K-1];
            rhok3[c+q-2] -= NN1[n-2]*C3_1[K-1];
        }
        if (Input.XDot_N_flag == true) {
            //std::cout<<"The initial value is rhok1[c-1] "<<rhok1[c-1]<<std::endl;
            rhok1[c-1] -= NN[n-1]*C1_1[K];
            //std::cout<<"The new value is rhok1[c-1] "<<rhok1[c-1]<<std::endl;
            //std::cout<<"The initial value is rhok1[c+q-2] "<<rhok1[c+q-2]<<std::endl;
            rhok1[c+q-2]-= NN1[n-1]*C1_1[K];
            //std::cout<<"The new value is rhok1[c+q-2] "<<rhok1[c+q-2]<<std::endl;
            //std::cout<<"The initial value is rhok2[c-1]"<<rhok2[c-1]<<std::endl;
            rhok2[c-1] -= NN[n-1]*C2_1[K];
            //std::cout<<"The new value is rhok2[c-1]"<<rhok2[c-1]<<std::endl;
            //std::cout<<"The initial value is rhok2[c+q-2]"<<rhok2[c+q-2]<<std::endl;
            rhok2[c+q-2]-= NN1[n-1]*C2_1[K];
            //std::cout<<"The final value is rhok2[c+q-2]"<<rhok2[c+q-2]<<std::endl;
            //std::cout<<"The initial value is rhok3[c-1]"<<rhok3[c-1]<<std::endl;
            rhok3[c-1] -= NN[n-1]*C3_1[K];
            //std::cout<<"The final value is rhok3[c-1]"<<rhok3[c-1]<<std::endl;
            //std::cout<<"The initial value is rhok3[c+q-2]"<<rhok3[c+q-2]<<std::endl;
            rhok3[c+q-2]-= NN1[n-1]*C3_1[K];
            //std::cout<<"The final value is rhok3[c+q-2]"<<rhok3[c+q-2]<<std::endl;
        }
    }
    
//    std::cout<<"Printing rhok vectors"<<std::endl;
//
//    for ( int y = 0; y < 2*q-2; y++) {
//        std::cout<<rhok1[y]<<","<<rhok2[y]<<","<<rhok3[y]<<std::endl;
//    }
    
        // Split code based on whether LS approximation is done with first derivative constraints or not
        
        Eigen::MatrixXd ND(q-1,n-K-1);
        Eigen::VectorXd rho1(n-K-1), rho2(n-K-1), rho3(n-K-1);
        Eigen::MatrixXd NWN(n-K-1,n-K-1), NWN_inv(n-K-1,n-K-1);
        
        
        // LS approximation without first derivative constraint
        if (Input.LS_Dot == false){
        
            std::cout<<"Not constraining first derivative"<<std::endl;

        // populate LS matrix ND
        for (int d = 0; d < q-1; d++) {
            basisFunction(uk[1+d], U, n+1, P, &NN[0], &NN1[0], &NN2[0]);
            int k = 1;
            if (Input.XDot_0_flag == true) {k += 1;}
            if (Input.XDDot_0_flag == true) {k += 1;}
            for (int e = 0; e < n-K-1; e++) {
                ND(d,e) = NN[k+e];
//                std::cout<<"ND print condition 1"<<std::endl;
//                std::cout<<ND(d,e)<<std::endl;
            }
        }

        // populate weight matrix W
        Eigen::MatrixXd W(q-1,q-1);
        for (int f = 0; f < q-1; f++) {
            for (int m = 0; m < q-1; m++) {
                if (f == m) {
                    if (Input.W_flag) {
                        W(f,m) = Input.W[f+1];
                    }
                    else {
                        W(f,m) = 1;
                    }
                }
                else {
                    W(f,m) = 0;
                }
            }
        }

        B = ND.transpose() * W;
            
        Eigen::VectorXd rhok1_short(q-1),rhok2_short(q-1),rhok3_short(q-1);
            
//        std::cout<<"Rhok short"<<std::endl;
        for (int c = 0; c <q; c++) {
            rhok1_short[c] = rhok1[c];
//            std::cout<<rhok1_short[c]<<std::endl;
            rhok2_short[c] = rhok2[c];
//            std::cout<<rhok2_short[c]<<std::endl;
            rhok3_short[c] = rhok3[c];
//            std::cout<<rhok3_short[c]<<std::endl;
            }
        
        rho1 = B * rhok1_short;
        rho2 = B * rhok2_short;
        rho3 = B * rhok3_short;
            
        NWN = B * ND;
        NWN_inv = NWN.inverse();
    }
    
        
        // LS approximation with first derivative constraint
    else {
        
//        std::cout<<"Entered else condition successfully"<<std::endl;
//        std::cout<<Input.X1Dot_des[0]<<std::endl;
//        std::cout<<Input.X2Dot_des[0]<<std::endl;
//        std::cout<<Input.X3Dot_des[0]<<std::endl;
//        std::cout<<"Passed this phase"<<std::endl;
//        std::cout<<q<<std::endl;
        
            
//        for (i = 0;i <q-1;i++) {
//                rhok1D[i] = rhok1[i];
//                rhok2D[i] = rhok2[i];
//                rhok3D[i] = rhok3[i];
//            }
//        //Change these to C3_1,C2_1 along with C1_1
//        for (int c = q-1; c < 2*q-2; c++) {
//            basisFunction(uk[c], U, n+1, P, &NN[0], &NN1[0], &NN2[0]);
//            rhok1D[c-1] = X1_prime[c] - NN1[0]*C1_1[0] - NN1[n]*C1_1[K+1];
//            rhok2D[c-1] = X2_prime[c] - NN1[0]*C2_1[0] - NN1[n]*C2_1[K+1];
//            rhok3D[c-1] = X3_prime[c] - NN1[0]*C3_1[0] - NN1[n]*C3_1[K+1];
//            if (Input.XDot_0_flag == true) {
//                rhok1D[c-1] -= NN1[1]*C1_1[1];
//                rhok2D[c-1] -= NN1[1]*C2_1[1];
//                rhok3D[c-1] -= NN1[1]*C3_1[1];
//            }
//            if (Input.XDDot_0_flag == true) {
//                rhok1D[c-1] -= NN1[2]*C1_1[2];
//                rhok2D[c-1] -= NN1[2]*C2_1[2];
//                rhok3D[c-1] -= NN1[2]*C3_1[2];
//            }
//            if (Input.XDDot_N_flag == true) {
//                rhok1D[c-1] -= NN1[n-2]*C1_1[K-1];
//                rhok2D[c-1] -= NN1[n-2]*C2_1[K-1];
//                rhok3D[c-1] -= NN1[n-2]*C3_1[K-1];
//            }
//            if (Input.XDot_N_flag == true) {
//                rhok1D[c-1] -= NN1[n-1]*C1_1[K];
//                rhok2D[c-1] -= NN1[n-1]*C2_1[K];
//                rhok3D[c-1] -= NN1[n-1]*C3_1[K];
//            }
//        }
        
        // new ND matrix is twice as large, will have to superimpose the previous ND matrix on this one
        Eigen::MatrixXd ND_2(2*q-2,n-K-1);
        
        // populate LS matrix ND
        for (int d = 0; d < q-1; d++) {
            basisFunction(uk[1+d], U, n+1, P, &NN[0], &NN1[0], &NN2[0]);
            int k = 1;
            if (Input.XDot_0_flag == true) {k += 1;}
            if (Input.XDDot_0_flag == true) {k += 1;}
            for (int e = 0; e < n-K-1; e++) {
                ND(d,e) = NN[k+e];
                //std::cout<<"ND print condition 2"<<std::endl;
                //std::cout<<ND(d,e)<<std::endl;
            }
        }
        
        // populate LS matrix ND_2  with ND matrix in top half
        for (int d = 0; d < q-1; d++) {
            basisFunction(uk[1+d], U, n+1, P, &NN[0], &NN1[0], &NN2[0]);
            for (int e = 0; e < n-K-1; e++) {
                ND_2(d,e) = ND(d,e);
            }
        }
        
        // calculate bottom half
        for (int d = q-1; d < 2*q-2; d++) {
            int k = 1;
            if (Input.XDot_0_flag == true) {k += 1;}
            if (Input.XDDot_0_flag == true) {k += 1;}
            for (int e = 0; e < n-K-1; e++) {
                //std::cout<<"ND_2 matrix terms"<<std::endl;
                ND_2(d,e) = NN1[k+e];
                //std::cout<<ND_2(d,e)<<std::endl;
            }
        }
        
  
        
        
        // populate weight matrix W_2
        Eigen::MatrixXd W_2(2*q-2,2*q-2);
        for (int f = 0; f < 2*q-2; f++) {
            for (int m = 0; m < 2*q-2; m++) {
                if (f == m) {
                    if (Input.W_flag) {
                        W_2(f,m) = Input.W[f+1];
                    }
                    else {
                        W_2(f,m) = 1;
                    }
                }
                else {
                    W_2(f,m) = 0;
                }
            }
        }
        
        
        B = ND_2.transpose() * W_2;
        
        
        rho1 = B * rhok1;
        
        rho2 = B * rhok2;
        
        rho3 = B * rhok3;
        
        NWN = B * ND_2;
                
        NWN_inv = NWN.inverse();
    }
    // compute LS values R for the control points
    Eigen::VectorXd C1_2 = NWN_inv * rho1;
    Eigen::VectorXd C2_2 = NWN_inv * rho2;
    Eigen::VectorXd C3_2 = NWN_inv * rho3;
    
    // build control point vectors C
    Eigen::VectorXd C1(n+1), C2(n+1), C3(n+1);
    int g = 0;
    C1[g] = C1_1[g];  C2[g] = C2_1[g];  C3[g] = C3_1[g];
    if (Input.XDot_0_flag == true) {
        g += 1;
        C1[g] = C1_1[g];  C2[g] = C2_1[g];  C3[g] = C3_1[g];
    }
    if (Input.XDDot_0_flag == true) {
        g += 1;
        C1[g] = C1_1[g];  C2[g] = C2_1[g];  C3[g] = C3_1[g];
    }
    for (int h = 0; h < n-K-1; h++) {
        C1[g+h+1] = C1_2[h];  C2[g+h+1] = C2_2[h];  C3[g+h+1] = C3_2[h];
    }
    if (Input.XDDot_N_flag == true) {
        g += 1;
        C1[n-K-1+g] = C1_1[g];  C2[n-K-1+g] = C2_1[g];  C3[n-K-1+g] = C3_1[g];
    }
    if (Input.XDot_N_flag == true) {
        g += 1;
        C1[n-K-1+g] = C1_1[g];  C2[n-K-1+g] = C2_1[g];  C3[n-K-1+g] = C3_1[g];
    }
    g += 1;
    C1[n-K-1+g] = C1_1[g];  C2[n-K-1+g] = C2_1[g];  C3[n-K-1+g] = C3_1[g];

    Output->U = U;
    Output->C1 = C1;
    Output->C2 = C2;
    Output->C3 = C3;
    
    for (int a=0;a<q;a++){
        X1_prime[a] = X1_prime[a]/Ttot;
        X2_prime[a] = X2_prime[a]/Ttot;
        X3_prime[a] = X3_prime[a]/Ttot;
    }
        
    
    Output->X1_prime = X1_prime;
    Output->X2_prime = X2_prime;
    Output->X3_prime = X3_prime;

    double dt = 1.0 / (Num - 1);
    double t = 0;
    // store the interpolated trajectory information into Output structure
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
        basisFunction(t, U, n+1, P, &NN[0], &NN1[0], &NN2[0]);
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
    std::cout<<"Approximate done"<<std::endl;
    return;
}


/*! This function calculates the basis functions NN of order P, and derivatives NN1, NN2, for a given time t and knot vector U */
void basisFunction(double t, Eigen::VectorXd U, int I, int P, double *NN, double *NN1, double *NN2)
{
    Eigen::MatrixXd N(I, P+1);
    Eigen::MatrixXd N1(I, P+1);
    Eigen::MatrixXd N2(I, P+1);
    /* populate matrices with zeros */
    for (int i = 0; i < I; i++) {
        for (int p = 0; p < P+1; p++) {
            N(i,p)  = 0;
            N1(i,p) = 0;
            N2(i,p) = 0;
        }
    }
    /* zero order */
    for (int i = 0; i < I; i++) {
        if ( (t >= U(i)) && (t < U(i+1)) ) {
            N(i,0) = 1;
        }
    }
    if (abs(t-1.0) < 1e-5) {
        N(I-1,0) = 1;
    }
    /* higher order - De Boor formula */
    for (int p = 1; p < P+1; p++) {
        for (int i = 0; i < I; i++) {
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
    // output result
    for (int i = 0; i < I; i++) {
        *(NN+i)  = N(i,P);
        *(NN1+i) = N1(i,P);
        *(NN2+i) = N2(i,P);
    }

    return;
    
}
