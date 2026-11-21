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
#include <architecture/utilities/bskLogging.h>
#include <cmath>
#include <iostream>

namespace {
Eigen::VectorXd waypointTimes(const InputDataSet& input)
{
    const auto count = input.X1.size();
    if (count < 2) {
        BSKLogger().bskError("BSpline: a fit requires at least two waypoints.");
    }
    if (input.X2.size() != count || input.X3.size() != count || (input.T_flag && input.T.size() != count)) {
        BSKLogger().bskError("BSpline: coordinates and supplied times must have one entry per waypoint.");
    }

    Eigen::VectorXd times(count);
    if (input.T_flag) {
        times = input.T;
    } else {
        times[0] = 0.0; // [s]
        for (Eigen::Index i = 1; i < count; ++i) {
            const Eigen::Vector3d step(
                input.X1[i] - input.X1[i - 1], input.X2[i] - input.X2[i - 1], input.X3[i] - input.X3[i - 1]);
            times[i] = times[i - 1] + step.norm();
        }
        if (input.AvgXDot_flag) {
            if (!std::isfinite(input.AvgXDot) || input.AvgXDot <= 0.0) {
                BSKLogger().bskError("BSpline: average waypoint speed must be finite and positive.");
            }
            times /= input.AvgXDot;
        }
    }
    if (!times.allFinite() || times[0] != 0.0 ||
        (times.tail(count - 1).array() <= times.head(count - 1).array()).any()) {
        BSKLogger().bskError("BSpline: waypoint times must start at zero and be finite and strictly increasing.");
    }
    return times;
}

Eigen::Index lastSampleIndex(const Eigen::VectorXd& times)
{
    if (times.size() == 0) {
        BSKLogger().bskError("BSpline: a spline query requires fitted samples.");
    }
    return times.size() - 1;
}
} // namespace

/*! This constructor initializes an Input structure for BSpline interpolation */
InputDataSet::InputDataSet()
{
    return;
}

/*! The constructor requires 3 N-dimensional vectors containing the coordinates of the waypoints */
InputDataSet::InputDataSet(const Eigen::VectorXd& X1, const Eigen::VectorXd& X2, const Eigen::VectorXd& X3)
{
    this->X1 = X1;
    this->X2 = X2;
    this->X3 = X3;

    uint64_t N1 = (uint64_t)X1.size();
    uint64_t N2 = (uint64_t)X2.size();
    uint64_t N3 = (uint64_t)X3.size();

    if ((N1 != N2) || (N1 != N3) || (N2 != N3)) {
        std::cout
            << "Error in BSpline.InputDataSet: \n the Input coordinate vectors X1, X2, X3 have different sizes. \n";
    }

    return;
}

/*! Generic destructor */
InputDataSet::~InputDataSet()
{
    return;
}

/*! Set the first derivative of the starting point (optional) */
void InputDataSet::setXDot_0(Eigen::Vector3d XDot_0)
{
    this->XDot_0 = XDot_0;
    this->XDot_0_flag = true;
    return;
}

/*! Set the first derivative of the last point (optional) */
void InputDataSet::setXDot_N(Eigen::Vector3d XDot_N)
{
    this->XDot_N = XDot_N;
    this->XDot_N_flag = true;
    return;
}

/*! Set the second derivative of the starting point (optional) */
void InputDataSet::setXDDot_0(Eigen::Vector3d XDDot_0)
{
    this->XDDot_0 = XDDot_0;
    this->XDDot_0_flag = true;
    return;
}

/*! Set the second derivative of the last point (optional) */
void InputDataSet::setXDDot_N(Eigen::Vector3d XDDot_N)
{
    this->XDDot_N = XDDot_N;
    this->XDDot_N_flag = true;
    return;
}

/*! Set the time tags for each waypoint (optional). Cannot be imposed together with avg velocity norm below */
void InputDataSet::setT(const Eigen::VectorXd& T)
{
    this->T = T;
    this->T_flag = true;
    this->AvgXDot_flag = false;
    return;
}

/*! Set the average velocity norm (optional). Cannot be imposed together with time tag vector above */
void InputDataSet::setAvgXDot(double AvgXDot)
{
    this->AvgXDot = AvgXDot;
    this->AvgXDot_flag = true;
    this->T_flag = false;
    return;
}

/*! Set the weights for each waypoint. Weights are used in the LS approximation */
void InputDataSet::setW(const Eigen::VectorXd& W)
{
    this->W = W;
    this->W_flag = true;
    return;
}

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
    const auto N = lastSampleIndex(this->T);
    double Ttot = this->T[N];

    // if T < Ttot calculalte the values
    if (T <= Ttot) {
        double t = T / Ttot;
        int Q = (int)this->C1.size();
        Eigen::VectorXd NN(Q), NN1(Q), NN2(Q);
        basisFunction(t, this->U, Q, this->P, &NN[0], &NN1[0], &NN2[0]);
        x[0] = NN.dot(this->C1);
        x[1] = NN.dot(this->C2);
        x[2] = NN.dot(this->C3);
        xDot[0] = NN1.dot(this->C1) / Ttot;
        xDot[1] = NN1.dot(this->C2) / Ttot;
        xDot[2] = NN1.dot(this->C3) / Ttot;
        xDDot[0] = NN2.dot(this->C1) / pow(Ttot, 2);
        xDDot[1] = NN2.dot(this->C2) / pow(Ttot, 2);
        xDDot[2] = NN2.dot(this->C3) / pow(Ttot, 2);
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
    const auto N = lastSampleIndex(this->T);
    double Ttot = this->T[N];

    // if T < Ttot calculalte the values
    if (T <= Ttot) {
        double t = T / Ttot;
        int Q = (int)this->C1.size();
        Eigen::VectorXd NN(Q), NN1(Q), NN2(Q);
        basisFunction(t, this->U, Q, this->P, &NN[0], &NN1[0], &NN2[0]);

        switch (derivative) {
            case 0:
                switch (index) {
                    case 0:
                        return NN.dot(this->C1);
                    case 1:
                        return NN.dot(this->C2);
                    case 2:
                        return NN.dot(this->C3);
                    default:
                        std::cout << "Error in Output.getStates: invalid index \n";
                        return 1000;
                }
            case 1:
                switch (index) {
                    case 0:
                        return NN1.dot(this->C1) / Ttot;
                    case 1:
                        return NN1.dot(this->C2) / Ttot;
                    case 2:
                        return NN1.dot(this->C3) / Ttot;
                    default:
                        std::cout << "Error in Output.getStates: invalid index \n";
                        return 1000;
                }
            case 2:
                switch (index) {
                    case 0:
                        return NN2.dot(this->C1) / pow(Ttot, 2);
                    case 1:
                        return NN2.dot(this->C2) / pow(Ttot, 2);
                    case 2:
                        return NN2.dot(this->C3) / pow(Ttot, 2);
                    default:
                        std::cout << "Error in Output.getStates: invalid index \n";
                        return 1000;
                }
            default:
                std::cout << "Error in Output.getStates: invalid derivative \n";
                return 1000;
        }
    }
    // if t > Ttot return final value with zero derivatives
    else {
        switch (derivative) {
            case 0:
                switch (index) {
                    case 0:
                        return this->X1[N];
                    case 1:
                        return this->X2[N];
                    case 2:
                        return this->X3[N];
                    default:
                        std::cout << "Error in Output.getStates: invalid index \n";
                        return 1000;
                }
            case 1:
                switch (index) {
                    case 0:
                        return 0;
                    case 1:
                        return 0;
                    case 2:
                        return 0;
                    default:
                        std::cout << "Error in Output.getStates: invalid index \n";
                        return 1000;
                }
            case 2:
                switch (index) {
                    case 0:
                        return 0;
                    case 1:
                        return 0;
                    case 2:
                        return 0;
                    default:
                        std::cout << "Error in Output.getStates: invalid index \n";
                        return 1000;
                }
            default:
                std::cout << "Error in Output.getStates: invalid derivative \n";
                return 1000;
        }
    }
}

/*! This function takes the Input structure, performs the BSpline interpolation and outputs the result into Output
 * structure */
void interpolate(const InputDataSet& Input, int Num, int P, OutputDataSet* Output)
{
    const Eigen::VectorXd T = waypointTimes(Input);
    Output->P = P;

    // N = number of waypoints - 1
    int N = (int)Input.X1.size() - 1;

    const double Ttot = T[N];
    const Eigen::VectorXd uk = T / Ttot;

    // K = number of endpoint derivatives
    int K = 0;
    if (Input.XDot_0_flag == true) {
        K += 1;
    }
    if (Input.XDot_N_flag == true) {
        K += 1;
    }
    if (Input.XDDot_0_flag == true) {
        K += 1;
    }
    if (Input.XDDot_N_flag == true) {
        K += 1;
    }

    // The maximum polynomial order is N + K. If a higher order is requested, print a BSK_ERROR
    if (P > N + K) {
        std::cout << "Error in BSpline.interpolate: \n the desired polynomial order P is too high. Mass matrix A will "
                     "be singular. \n";
    }

    int M = N + P + K + 1;

    // build knot vector U of size M + 1
    Eigen::VectorXd U(M + 1);
    double u;
    for (int p = 0; p < P + 1; p++) {
        U[p] = 0;
    }
    for (int j = 0; j < M - 2 * P - 1; j++) {
        u = 0.0;
        for (int i = j; i < j + P; i++) {
            if (i >= uk.size()) {
                u += uk[N] / P;
            } else {
                u += uk[i] / P;
            }
            U[P + j + 1] = u;
        }
    }
    for (int p = 0; p < P + 1; p++) {
        U[M - P + p] = 1;
    }

    // build stiffness matrix A of size (N+K+1)x(N+K+1)
    Eigen::MatrixXd A(N + K + 1, N + K + 1);
    // build vectors Q1, Q2, Q3 (left hand side of linear system)
    Eigen::VectorXd Q1(N + K + 1), Q2(N + K + 1), Q3(N + K + 1);
    // populate A with zeros
    for (int n = 0; n < N + K + 1; n++) {
        for (int m = 0; m < N + K + 1; m++) {
            A(n, m) = 0;
        }
    }
    int n = -1;
    // constrain first derivative at starting point
    if (Input.XDot_0_flag == true) {
        n += 1;
        A(n, 0) = -1;
        A(n, 1) = 1;
        Q1[n] = U[P + 1] / P * Input.XDot_0[0] * Ttot;
        Q2[n] = U[P + 1] / P * Input.XDot_0[1] * Ttot;
        Q3[n] = U[P + 1] / P * Input.XDot_0[2] * Ttot;
    }
    // constrain second derivative at starting point
    if (Input.XDDot_0_flag == true) {
        n += 1;
        A(n, 0) = U[P + 2];
        A(n, 1) = -(U[P + 1] + U[P + 2]);
        A(n, 2) = U[P + 1];
        Q1[n] = (pow(U[P + 1], 2) * U[P + 2] / (P * (P - 1))) * Input.XDDot_0[0] * pow(Ttot, 2);
        Q2[n] = (pow(U[P + 1], 2) * U[P + 2] / (P * (P - 1))) * Input.XDDot_0[1] * pow(Ttot, 2);
        Q3[n] = (pow(U[P + 1], 2) * U[P + 2] / (P * (P - 1))) * Input.XDDot_0[2] * pow(Ttot, 2);
    }
    n += 1;
    int m = -1;
    int n0 = n;
    Eigen::VectorXd NN(N + K + 1), NN1(N + K + 1), NN2(N + K + 1);
    // constrain waypoints
    for (n = n0; n < N + n0 + 1; n++) {
        m += 1;
        basisFunction(uk[m], U, N + K + 1, P, &NN[0], &NN1[0], &NN2[0]);
        for (int b = 0; b < N + K + 1; b++) {
            A(n, b) = NN[b];
        }
        Q1[n] = Input.X1[m];
        Q2[n] = Input.X2[m];
        Q3[n] = Input.X3[m];
    }
    n = N + n0;
    // constrain second derivative at final point
    if (Input.XDDot_N_flag == true) {
        n += 1;
        A(n, N + K - 2) = 1 - U[M - P - 1];
        A(n, N + K - 1) = -(2 - U[M - P - 1] - U[M - P - 2]);
        A(n, N + K) = 1 - U[M - P - 2];
        Q1[n] = (pow((1 - U[M - P - 1]), 2) * (1 - U[M - P - 2]) / (P * (P - 1))) * Input.XDDot_N[0] * pow(Ttot, 2);
        Q2[n] = (pow((1 - U[M - P - 1]), 2) * (1 - U[M - P - 2]) / (P * (P - 1))) * Input.XDDot_N[1] * pow(Ttot, 2);
        Q3[n] = (pow((1 - U[M - P - 1]), 2) * (1 - U[M - P - 2]) / (P * (P - 1))) * Input.XDDot_N[2] * pow(Ttot, 2);
    }
    // constrain first derivative at final point
    if (Input.XDot_N_flag == true) {
        n += 1;
        A(n, N + K - 1) = -1;
        A(n, N + K) = 1;
        Q1[n] = (1 - U[M - P - 1]) / P * Input.XDot_N[0] * Ttot;
        Q2[n] = (1 - U[M - P - 1]) / P * Input.XDot_N[1] * Ttot;
        Q3[n] = (1 - U[M - P - 1]) / P * Input.XDot_N[2] * Ttot;
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
        basisFunction(t, U, N + K + 1, P, &NN[0], &NN1[0], &NN2[0]);
        Output->T[i] = t * Ttot;
        Output->X1[i] = NN.dot(C1);
        Output->X2[i] = NN.dot(C2);
        Output->X3[i] = NN.dot(C3);
        Output->XD1[i] = NN1.dot(C1) / Ttot;
        Output->XD2[i] = NN1.dot(C2) / Ttot;
        Output->XD3[i] = NN1.dot(C3) / Ttot;
        Output->XDD1[i] = NN2.dot(C1) / pow(Ttot, 2);
        Output->XDD2[i] = NN2.dot(C2) / pow(Ttot, 2);
        Output->XDD3[i] = NN2.dot(C3) / pow(Ttot, 2);
        t += dt;
    }

    return;
}

/*! This function takes the Input structure, performs the BSpline LS approximation and outputs the result into Output
 * structure */
void approximate(const InputDataSet& Input, int Num, int Q, int P, OutputDataSet* Output)
{
    const Eigen::VectorXd T = waypointTimes(Input);
    if (Input.W_flag && Input.W.size() != Input.X1.size()) {
        BSKLogger().bskError("BSpline: supplied spline weights must have one entry per waypoint.");
    }
    if ((Input.XDDot_0_flag && !Input.XDot_0_flag) || (Input.XDDot_N_flag && !Input.XDot_N_flag)) {
        BSKLogger().bskError(
            "BSpline: approximation requires velocity at each endpoint with a prescribed acceleration.");
    }
    const int startDerivatives = Input.XDot_0_flag + Input.XDDot_0_flag;
    const int endDerivatives = Input.XDot_N_flag + Input.XDDot_N_flag;
    const int K = startDerivatives + endDerivatives;
    const int N = static_cast<int>(T.size()) - 1;
    // Q is the last control-point index. Reserve one point for each endpoint position and derivative.
    if (Q < K + 1) {
        BSKLogger().bskError("BSpline: approximation needs at least %d control points for the endpoint constraints.",
                             K + 2);
    }
    if (Q - K > N) {
        BSKLogger().bskError("BSpline: approximation has more free control points than interior waypoints.");
    }
    if (P > Q) {
        BSKLogger().bskError("BSpline: approximation degree must be less than the number of control points.");
    }
    if (P < startDerivatives || P < endDerivatives) {
        BSKLogger().bskError("BSpline: approximation degree must be nonnegative and support the endpoint derivatives.");
    }
    const double Ttot = T[N];
    const Eigen::VectorXd uk = T / Ttot;

    // build knot vector U of size Q + P + 2
    Eigen::VectorXd U(Q + P + 2);
    const double d = static_cast<double>(N + 1) / (Q - P + 1);
    for (int p = 0; p < P + 1; p++) {
        U[p] = 0;
    }
    for (int j = 1; j < Q - P + 1; j++) {
        const int knotIndex = static_cast<int>(j * d);
        if (knotIndex < 1 || knotIndex > N) {
            BSKLogger().bskError("BSpline: approximation interior knots must be bracketed by waypoint times.");
        }
        const double alpha = j * d - knotIndex;
        U[P + j] = (1 - alpha) * uk[knotIndex - 1] + alpha * uk[knotIndex];
        if (!(U[P + j - 1] < U[P + j] && U[P + j] < 1.0)) {
            BSKLogger().bskError("BSpline: approximation interior knots must increase strictly between zero and one.");
        }
    }
    for (int p = 0; p < P + 1; p++) {
        U[Q + p + 1] = 1;
    }

    // build stiffness matrix MD of size (K+2)x(K+2)
    Eigen::MatrixXd MD(K + 2, K + 2);
    // build vectors T1, T2, T3 (left hand side of linear system)
    Eigen::VectorXd T1(K + 2), T2(K + 2), T3(K + 2);
    // populate MD with zeros
    for (int n = 0; n < K + 2; n++) {
        for (int m = 0; m < K + 2; m++) {
            MD(n, m) = 0;
        }
    }
    Eigen::VectorXd NN(Q + 1), NN1(Q + 1), NN2(Q + 1);
    basisFunction(uk[0], U, Q + 1, P, &NN[0], &NN1[0], &NN2[0]);
    int constraintIndex = 0;
    MD(0, 0) = NN[0];
    T1[0] = Input.X1[0];
    T2[0] = Input.X2[0];
    T3[0] = Input.X3[0];
    // constrain first derivative at starting point
    if (Input.XDot_0_flag == true) {
        constraintIndex += 1;
        MD(constraintIndex, 0) = NN1[0];
        MD(constraintIndex, 1) = NN1[1];
        T1[constraintIndex] = Input.XDot_0[0] * Ttot;
        T2[constraintIndex] = Input.XDot_0[1] * Ttot;
        T3[constraintIndex] = Input.XDot_0[2] * Ttot;
    }
    // constrain second derivative at starting point
    if (Input.XDDot_0_flag == true) {
        constraintIndex += 1;
        MD(constraintIndex, 0) = NN2[0];
        MD(constraintIndex, 1) = NN2[1];
        MD(constraintIndex, 2) = NN2[2];
        T1[constraintIndex] = Input.XDDot_0[0] * pow(Ttot, 2);
        T2[constraintIndex] = Input.XDDot_0[1] * pow(Ttot, 2);
        T3[constraintIndex] = Input.XDDot_0[2] * pow(Ttot, 2);
    }
    basisFunction(uk[N], U, Q + 1, P, &NN[0], &NN1[0], &NN2[0]);
    // constrain second derivative at ending point
    if (Input.XDDot_N_flag == true) {
        constraintIndex += 1;
        MD(constraintIndex, K - 1) = NN2[Q - 2];
        MD(constraintIndex, K) = NN2[Q - 1];
        MD(constraintIndex, K + 1) = NN2[Q];
        T1[constraintIndex] = Input.XDDot_N[0] * pow(Ttot, 2);
        T2[constraintIndex] = Input.XDDot_N[1] * pow(Ttot, 2);
        T3[constraintIndex] = Input.XDDot_N[2] * pow(Ttot, 2);
    }
    // constrain first derivative at ending point
    if (Input.XDot_N_flag == true) {
        constraintIndex += 1;
        MD(constraintIndex, K) = NN1[Q - 1];
        MD(constraintIndex, K + 1) = NN1[Q];
        T1[constraintIndex] = Input.XDot_N[0] * Ttot;
        T2[constraintIndex] = Input.XDot_N[1] * Ttot;
        T3[constraintIndex] = Input.XDot_N[2] * Ttot;
    }
    constraintIndex += 1;
    MD(constraintIndex, K + 1) = NN[Q];
    T1[constraintIndex] = Input.X1[N];
    T2[constraintIndex] = Input.X2[N];
    T3[constraintIndex] = Input.X3[N];

    // solve linear systems
    Eigen::MatrixXd B = MD.inverse();
    Eigen::VectorXd C1_1 = B * T1;
    Eigen::VectorXd C2_1 = B * T2;
    Eigen::VectorXd C3_1 = B * T3;

    // populate Rk vectors with the base points for LS minimization
    Eigen::MatrixXd residuals(N - 1, 3);
    auto Rk1 = residuals.col(0), Rk2 = residuals.col(1), Rk3 = residuals.col(2);
    for (int n = 1; n < N; n++) {
        if (Input.W_flag && Input.W[n] == 0.0) {
            residuals.row(n - 1).setZero();
            continue;
        }
        basisFunction(uk[n], U, Q + 1, P, &NN[0], &NN1[0], &NN2[0]);
        Rk1[n - 1] = Input.X1[n] - NN[0] * C1_1[0] - NN[Q] * C1_1[K + 1];
        Rk2[n - 1] = Input.X2[n] - NN[0] * C2_1[0] - NN[Q] * C2_1[K + 1];
        Rk3[n - 1] = Input.X3[n] - NN[0] * C3_1[0] - NN[Q] * C3_1[K + 1];
        if (Input.XDot_0_flag == true) {
            Rk1[n - 1] -= NN[1] * C1_1[1];
            Rk2[n - 1] -= NN[1] * C2_1[1];
            Rk3[n - 1] -= NN[1] * C3_1[1];
        }
        if (Input.XDDot_0_flag == true) {
            Rk1[n - 1] -= NN[2] * C1_1[2];
            Rk2[n - 1] -= NN[2] * C2_1[2];
            Rk3[n - 1] -= NN[2] * C3_1[2];
        }
        if (Input.XDDot_N_flag == true) {
            Rk1[n - 1] -= NN[Q - 2] * C1_1[K - 1];
            Rk2[n - 1] -= NN[Q - 2] * C2_1[K - 1];
            Rk3[n - 1] -= NN[Q - 2] * C3_1[K - 1];
        }
        if (Input.XDot_N_flag == true) {
            Rk1[n - 1] -= NN[Q - 1] * C1_1[K];
            Rk2[n - 1] -= NN[Q - 1] * C2_1[K];
            Rk3[n - 1] -= NN[Q - 1] * C3_1[K];
        }
    }

    // populate LS matrix ND
    Eigen::MatrixXd ND(N - 1, Q - K - 1);
    for (int n = 0; n < N - 1; n++) {
        basisFunction(uk[1 + n], U, Q + 1, P, &NN[0], &NN1[0], &NN2[0]);
        int k = 1;
        if (Input.XDot_0_flag == true) {
            k += 1;
        }
        if (Input.XDDot_0_flag == true) {
            k += 1;
        }
        for (int b = 0; b < Q - K - 1; b++) {
            ND(n, b) = NN[k + b];
        }
        if (Input.W_flag) {
            const double weight = Input.W[n + 1];
            if (!std::isfinite(weight) || weight < 0.0) {
                BSKLogger().bskError("BSpline: interior waypoint weights must be finite and nonnegative.");
            }
            const double scale = std::sqrt(weight);
            ND.row(n) *= scale;
            residuals.row(n) *= scale;
        }
    }

    Eigen::MatrixXd freeControlPoints(ND.cols(), 3);
    if (ND.cols() > 0) {
        const auto qr = ND.colPivHouseholderQr();
        if (qr.rank() < ND.cols()) {
            BSKLogger().bskError(
                "BSpline: the waypoint fit is rank deficient and cannot determine all free control points.");
        }
        freeControlPoints = qr.solve(residuals);
    }

    // build control point vectors C
    Eigen::VectorXd C1(Q + 1), C2(Q + 1), C3(Q + 1);
    constraintIndex = 0;
    C1[constraintIndex] = C1_1[constraintIndex];
    C2[constraintIndex] = C2_1[constraintIndex];
    C3[constraintIndex] = C3_1[constraintIndex];
    if (Input.XDot_0_flag == true) {
        constraintIndex += 1;
        C1[constraintIndex] = C1_1[constraintIndex];
        C2[constraintIndex] = C2_1[constraintIndex];
        C3[constraintIndex] = C3_1[constraintIndex];
    }
    if (Input.XDDot_0_flag == true) {
        constraintIndex += 1;
        C1[constraintIndex] = C1_1[constraintIndex];
        C2[constraintIndex] = C2_1[constraintIndex];
        C3[constraintIndex] = C3_1[constraintIndex];
    }
    for (int q = 0; q < Q - K - 1; q++) {
        C1[constraintIndex + q + 1] = freeControlPoints(q, 0);
        C2[constraintIndex + q + 1] = freeControlPoints(q, 1);
        C3[constraintIndex + q + 1] = freeControlPoints(q, 2);
    }
    if (Input.XDDot_N_flag == true) {
        constraintIndex += 1;
        C1[Q - K - 1 + constraintIndex] = C1_1[constraintIndex];
        C2[Q - K - 1 + constraintIndex] = C2_1[constraintIndex];
        C3[Q - K - 1 + constraintIndex] = C3_1[constraintIndex];
    }
    if (Input.XDot_N_flag == true) {
        constraintIndex += 1;
        C1[Q - K - 1 + constraintIndex] = C1_1[constraintIndex];
        C2[Q - K - 1 + constraintIndex] = C2_1[constraintIndex];
        C3[Q - K - 1 + constraintIndex] = C3_1[constraintIndex];
    }
    constraintIndex += 1;
    C1[Q - K - 1 + constraintIndex] = C1_1[constraintIndex];
    C2[Q - K - 1 + constraintIndex] = C2_1[constraintIndex];
    C3[Q - K - 1 + constraintIndex] = C3_1[constraintIndex];

    Output->P = P;
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
        basisFunction(t, U, Q + 1, P, &NN[0], &NN1[0], &NN2[0]);
        Output->T[i] = t * Ttot;
        Output->X1[i] = NN.dot(C1);
        Output->X2[i] = NN.dot(C2);
        Output->X3[i] = NN.dot(C3);
        Output->XD1[i] = NN1.dot(C1) / Ttot;
        Output->XD2[i] = NN1.dot(C2) / Ttot;
        Output->XD3[i] = NN1.dot(C3) / Ttot;
        Output->XDD1[i] = NN2.dot(C1) / pow(Ttot, 2);
        Output->XDD2[i] = NN2.dot(C2) / pow(Ttot, 2);
        Output->XDD3[i] = NN2.dot(C3) / pow(Ttot, 2);
        t += dt;
    }

    return;
}

/*! This function calculates the basis functions NN of order P, and derivatives NN1, NN2, for a given time t and knot
 * vector U */
void basisFunction(double t, const Eigen::VectorXd& U, int I, int P, double* NN, double* NN1, double* NN2)
{
    Eigen::MatrixXd N(I, P + 1);
    Eigen::MatrixXd N1(I, P + 1);
    Eigen::MatrixXd N2(I, P + 1);
    /* populate matrices with zeros */
    for (int i = 0; i < I; i++) {
        for (int p = 0; p < P + 1; p++) {
            N(i, p) = 0;
            N1(i, p) = 0;
            N2(i, p) = 0;
        }
    }
    /* zero order */
    for (int i = 0; i < I; i++) {
        if ((t >= U(i)) && (t < U(i + 1))) {
            N(i, 0) = 1;
        }
    }
    // Close the right endpoint, including accumulated sampling roundoff above one.
    if (t >= 1.0 && abs(t - 1.0) < 1e-5) {
        N(I - 1, 0) = 1;
    }
    /* higher order - De Boor formula */
    for (int p = 1; p < P + 1; p++) {
        for (int i = 0; i < I; i++) {
            if (U[i + p] - U[i] != 0) {
                N(i, p) += (t - U[i]) / (U[i + p] - U[i]) * N(i, p - 1);
                N1(i, p) += p / (U[i + p] - U[i]) * N(i, p - 1);
                N2(i, p) += p / (U[i + p] - U[i]) * N1(i, p - 1);
            }
            if (U[i + p + 1] - U[i + 1] != 0) {
                N(i, p) += (U[i + p + 1] - t) / (U[i + p + 1] - U[i + 1]) * N(i + 1, p - 1);
                N1(i, p) -= p / (U[i + p + 1] - U[i + 1]) * N(i + 1, p - 1);
                N2(i, p) -= p / (U[i + p + 1] - U[i + 1]) * N1(i + 1, p - 1);
            }
        }
    }
    // output result
    for (int i = 0; i < I; i++) {
        *(NN + i) = N(i, P);
        *(NN1 + i) = N1(i, P);
        *(NN2 + i) = N2(i, P);
    }

    return;
}
