/*
 ISC License

 Copyright (c) 2024, University of Colorado at Boulder

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

#include "dynamicModels.h"

DynamicsModel::DynamicsModel() = default;

DynamicsModel::~DynamicsModel() = default;

/*! Call the propagation function for the dynamics
   @param std::array<double, 2> interval : time interval
   @param StateVector state : initial state
   @param double dt: time step
   @return StateVector propagatedState
*/
StateVector DynamicsModel::propagate(const std::array<double, 2> interval, const StateVector& state, const double dt) const {
    double t_0 = interval[0];
    double t_f = interval[1];
    double t = t_0;
    StateVector X(state);

    /*! Propagate to t_final with an RK4 integrator */
    double N = ceil((t_f-t_0)/dt);
    for (int c=0; c < N; c++) {
        double step = std::min(dt,t_f-t);
        X = this->rk4(this->propagator, X, t, step);
        t = t + step;
    }

    return X;
}

/*! Set the propagation function for the dynamics
   @param std::function<const StateVector(const StateVector&)>& dynamicsPropagator
*/
void DynamicsModel::setDynamics(const std::function<const StateVector(const double, const StateVector&)>& dynamicsPropagator){
    this->propagator = dynamicsPropagator;
}

/*! Call the dynamics matrix containing the partials of the dynamics with respect to the state
   @param std::function<const Eigen::MatrixXd(const StateVector&)>& dynamicsMatrixCalculator
*/
Eigen::MatrixXd DynamicsModel::computeDynamicsMatrix(const double time, const StateVector& state) const {
    return this->dynamicsMatrix(time, state);
}

/*! Set the dynamics matrix containing the partials of the dynamics with respect to the state
   @param std::function<const Eigen::MatrixXd(const StateVector&)>& dynamicsMatrixCalculator
*/
void DynamicsModel::setDynamicsMatrix(const std::function<const Eigen::MatrixXd(const double, const StateVector&)>&
        dynamicsMatrixCalculator){
    this->dynamicsMatrix = dynamicsMatrixCalculator;
}

/*! Runge-Kutta 4 (RK4) function for state propagation
    @param ODEfunction function handle that includes the equations of motion
    @param X0 initial state
    @param t0 initial time
    @param dt time step
    @return Eigen::VectorXd
*/
    StateVector DynamicsModel::rk4(std::function<const StateVector(const double, const StateVector&)> ODEfunction,
            const StateVector& X0,
            double t0,
            double dt) {
    double h = dt;

    StateVector k1 = ODEfunction(t0, X0);
    StateVector k2 = ODEfunction(t0 + h/2., X0.add(k1.scale(h/2.)));
    StateVector k3 = ODEfunction(t0 + h/2., X0.add(k2.scale(h/2.)));
    StateVector k4 = ODEfunction(t0 + h, X0.add(k3.scale(h)));

    return X0.add(k1.scale(h/6.).add(k2.scale(h/3.)).add(k3.scale(h/3.)).add(k4.scale(h/6.)));
}
