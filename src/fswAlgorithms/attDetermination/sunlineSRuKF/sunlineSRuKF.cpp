/*
 ISC License

 Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

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

#include "sunlineSRuKF.h"

SunlineSRuKF::SunlineSRuKF() = default;

SunlineSRuKF::~SunlineSRuKF() = default;

/*! Initialize C-wrapped output messages */
void SunlineSRuKF::SelfInit(){
    NavAttMsg_C_init(&this->navAttOutMsgC);
}

/*! Reset the sunline filter to an initial state and
 initializes the internal estimation matrices.
 @return void
 @param CurrentSimNanos The clock time at which the function was called (nanoseconds)
 */
void SunlineSRuKF::customReset() {
    /*! - Check if the required messages have been connected */
    assert(this->cssDataInMsg.isLinked());
    assert(this->cssConfigInMsg.isLinked());
    assert(this->navAttInMsg.isLinked());

    /*! read in CSS configuration message */
    this->cssConfigInputBuffer = this->cssConfigInMsg();
}

/*! Normalize the updated sunline estimate
 @return void
 @param CurrentSimNanos The clock time at which the function was called (nanoseconds)
 */
void SunlineSRuKF::customFinalizeUpdate() {
    this->state.head(3).normalize();
}

/*! Read the message containing the measurement data.
 It updates class variables relating to measurement data including validity and time tags.
 @return void
 */
void SunlineSRuKF::writeOutputMessages(uint64_t CurrentSimNanos) {

}

/*! Read the message containing the measurement data.
 * It updates class variables relating to measurement data including validity and time tags.
 @return void
 */
void SunlineSRuKF::readFilterMeasurements() {

}

/*! Integrate the equations of motion of two body point mass gravity using Runge-Kutta 4 (RK4)
    @param interval integration interval
    @param X0 initial state
    @param dt time step
    @return Eigen::VectorXd
*/
Eigen::VectorXd SunlineSRuKF::propagate(std::array<double, 2> interval, const Eigen::VectorXd& X0, double dt){
    double t_0 = interval[0];
    double t_f = interval[1];
    double t = t_0;
    Eigen::VectorXd X = X0;

    std::function<Eigen::VectorXd(double, Eigen::VectorXd)> stateDerivative = [](double t, Eigen::VectorXd state)
    {
        Eigen::VectorXd XDot(state.size());
        /*! Implement propagation with rate derivatives set to zero */
        Eigen::Vector3d sHat  = state.segment(0, 3);
        Eigen::Vector3d omega = state.segment(3, 3);
        XDot.segment(0,3) = sHat.cross(omega);
        XDot.segment(3,3).setZero();

        return XDot;
    };

    /*! Propagate to t_final with an RK4 integrator */
    double N = ceil((t_f-t_0)/dt);
    for (int c=0; c < N; c++) {
        double step = std::min(dt,t_f-t);
        X = this->rk4(stateDerivative, X, t, step);
        t = t + step;
    }

    return X;
}

/*! Set the CSS measurement noise
    @param double cssMeasurementNoise
    @return void
    */
void SunlineSRuKF::setCssMeasurementNoiseStd(const double cssMeasurementNoiseStd) {
    this->cssMeasNoiseStd = cssMeasurementNoiseStd;
}

/*! Set the gyro measurement noise
    @param double gyroMeasurementNoise
    @return void
    */
void SunlineSRuKF::setGyroMeasurementNoiseStd(const double gyroMeasurementNoiseStd) {
    this->gyroMeasNoiseStd = gyroMeasurementNoiseStd;
}

/*! Set the filter measurement noise scale factor if desirable
    @param double measurementNoiseScale
    @return void
    */
void SunlineSRuKF::setMeasurementNoiseScale(const double measurementNoiseScale) {
    this->measNoiseScaling = measurementNoiseScale;
}

/*! Get the CSS measurement noise
    @param double cssMeasurementNoise
    @return void
    */
double SunlineSRuKF::getCssMeasurementNoiseStd() const {
    return this->cssMeasNoiseStd;
}

/*! Get the gyro measurement noise
    @param double gyroMeasurementNoise
    @return void
    */
double SunlineSRuKF::getGyroMeasurementNoiseStd() const {
    return this->gyroMeasNoiseStd;
}

/*! Get the filter measurement noise scale factor
    @return double measNoiseScaling
    */
double SunlineSRuKF::getMeasurementNoiseScale() const {
    return this->measNoiseScaling;
}
