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


#include <Eigen/Dense>
#include "architecture/utilities/avsEigenSupport.h"
#include "architecture/utilities/macroDefinitions.h"


//! @brief The KeplerianOrbit class represents an elliptical orbit and provides a coherent set of
//! common outputs such as position and velocity, orbital period, semi-parameter, etc. It uses the
//! utility orbitalMotion to do orbital element to position and velocity conversion.

class InputDataSet {
public:
    InputDataSet();
    InputDataSet(Eigen::VectorXd X1, Eigen::VectorXd X2, Eigen::VectorXd X3);
    ~InputDataSet();

    void setXDot_0(Eigen::Vector3d XDot_0);
    void setXDot_N(Eigen::Vector3d XDot_N);
    void setXDDot_0(Eigen::Vector3d XDDot_0);
    void setXDDot_N(Eigen::Vector3d XDDot_N);
    void setT(Eigen::VectorXd T);
    
    Eigen::VectorXd T;
    Eigen::VectorXd X1;
    Eigen::VectorXd X2;
    Eigen::VectorXd X3;
    Eigen::Vector3d XDot_0;
    Eigen::Vector3d XDot_N;
    Eigen::Vector3d XDDot_0;
    Eigen::Vector3d XDDot_N;
    bool T_flag;
    bool XDot_0_flag;
    bool XDot_N_flag;
    bool XDDot_0_flag;
    bool XDDot_N_flag;
};

class OutputDataSet {
public:
    OutputDataSet();
    ~OutputDataSet();
    
    Eigen::VectorXd T;
    Eigen::VectorXd X1;
    Eigen::VectorXd X2;
    Eigen::VectorXd X3;
    Eigen::VectorXd XD1;
    Eigen::VectorXd XD2;
    Eigen::VectorXd XD3;
    Eigen::VectorXd XDD1;
    Eigen::VectorXd XDD2;
    Eigen::VectorXd XDD3;
};

void interpolate(InputDataSet Input, int N, double avgXDot, int P, OutputDataSet *Output);

void basisFunction(double t, Eigen::VectorXd U, int I, int P, double *NN, double *NN1, double *NN2);