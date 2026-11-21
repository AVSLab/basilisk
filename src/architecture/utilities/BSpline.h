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

#include "architecture/utilities/avsEigenSupport.h"
#include "architecture/utilities/macroDefinitions.h"
#include <Eigen/Dense>

//! The InputDataSet class contains the information about the points that must be interpolated.
//! It is used as a data structure to initialize the inputs that are passed to the interpolating function.

class InputDataSet
{
  public:
    InputDataSet();
    InputDataSet(const Eigen::VectorXd& X1, const Eigen::VectorXd& X2, const Eigen::VectorXd& X3);
    ~InputDataSet();

    void setXDot_0(Eigen::Vector3d XDot_0);
    void setXDot_N(Eigen::Vector3d XDot_N);
    void setXDDot_0(Eigen::Vector3d XDDot_0);
    void setXDDot_N(Eigen::Vector3d XDDot_N);
    void setT(const Eigen::VectorXd& T);
    void setW(const Eigen::VectorXd& W);
    void setAvgXDot(double AvgXDot);

    double AvgXDot = 0.0;                              //!< [coordinate units/s] desired average velocity norm
    Eigen::VectorXd T;                                 //!< [s] Waypoint times, finite and strictly increasing from zero
    Eigen::VectorXd W;                                 //!< Waypoint weights, finite and nonnegative for interior points
    Eigen::VectorXd X1;                                //!< coordinate #1 of the waypoints
    Eigen::VectorXd X2;                                //!< coordinate #2 of the waypoints
    Eigen::VectorXd X3;                                //!< coordinate #3 of the waypoints
    Eigen::Vector3d XDot_0 = Eigen::Vector3d::Zero();  //!< First derivative at the starting point
    Eigen::Vector3d XDot_N = Eigen::Vector3d::Zero();  //!< First derivative at the final point
    Eigen::Vector3d XDDot_0 = Eigen::Vector3d::Zero(); //!< Second derivative at the starting point
    Eigen::Vector3d XDDot_N = Eigen::Vector3d::Zero(); //!< Second derivative at the final point

    bool T_flag = false;       //!< indicates that time tags have been specified; if true, AvgXDot_flag is false
    bool AvgXDot_flag = false; //!< indicates that avg velocity norm has been specified; if true, T_flag is false
    bool W_flag = false;       //!< indicates that weight vector has been specified
    bool XDot_0_flag = false;  //!< indicates that first derivative at starting point has been specified
    bool XDot_N_flag = false;  //!< indicates that first derivative at final point has been specified
    bool XDDot_0_flag = false; //!< indicates that second derivative at starting point has been specified
    bool XDDot_N_flag = false; //!< indicates that second derivative at final point has been specified
};

//! @brief The OutputDataSet class is used as a data structure to contain the interpolated function and its first- and
//! second-order derivatives, together with the time-tag vector T.
class OutputDataSet
{
  public:
    OutputDataSet();
    ~OutputDataSet();
    void getData(double t, double x[3], double xDot[3], double xDDot[3]);
    double getStates(double t, int derivative, int index);

    Eigen::VectorXd T;    //!< time tags for each point of the interpolated trajectory
    Eigen::VectorXd X1;   //!< coordinate #1 of the interpolated trajectory
    Eigen::VectorXd X2;   //!< coordinate #2 of the interpolated trajectory
    Eigen::VectorXd X3;   //!< coordinate #3 of the interpolated trajectory
    Eigen::VectorXd XD1;  //!< first derivative of coordinate #1 of the interpolated trajectory
    Eigen::VectorXd XD2;  //!< first derivative of coordinate #2 of the interpolated trajectory
    Eigen::VectorXd XD3;  //!< first derivative of coordinate #3 of the interpolated trajectory
    Eigen::VectorXd XDD1; //!< second derivative of coordinate #1 of the interpolated trajectory
    Eigen::VectorXd XDD2; //!< second derivative of coordinate #2 of the interpolated trajectory
    Eigen::VectorXd XDD3; //!< second derivative of coordinate #3 of the interpolated trajectory

    int P;              //!< polynomial degree of the BSpline
    Eigen::VectorXd U;  //!< knot vector of the BSpline
    Eigen::VectorXd C1; //!< coordinate #1 of the control points
    Eigen::VectorXd C2; //!< coordinate #2 of the control points
    Eigen::VectorXd C3; //!< coordinate #3 of the control points
};

void interpolate(const InputDataSet& Input, int Num, int P, OutputDataSet* Output);

/*! @brief Fit a least-squares spline to the supplied waypoints.
 * @param Input Waypoints and optional constraints. Each endpoint acceleration requires velocity at the same endpoint.
 * @param Num Number of output samples.
 * @param Q Last control-point index. Each free control point requires at least one interior waypoint.
 * @param P Polynomial degree. The resulting Q - P interior knots must lie strictly between the endpoints.
 * @param Output Fitted spline and sampled states.
 */
void approximate(const InputDataSet& Input, int Num, int Q, int P, OutputDataSet* Output);

void basisFunction(double t, const Eigen::VectorXd& U, int I, int P, double* NN, double* NN1, double* NN2);
