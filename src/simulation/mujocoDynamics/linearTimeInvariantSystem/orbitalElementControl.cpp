/*
 ISC License

 Copyright (c) 2025, Autonomous Vehicle Systems Lab

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

#include "orbitalElementControl.h"

#include <cmath>

#include "architecture/utilities/orbitalMotion.h"

namespace {

double angleWarp(double angle)
{
    const double width = 2.0 * M_PI;
    double adjusted = angle;
    while (adjusted > M_PI) {
        adjusted -= width;
    }
    while (adjusted < -M_PI) {
        adjusted += width;
    }
    return adjusted;
}

/**
 * @brief Compute classical Gauss control matrix B (6x3) for given elements.
 *
 * See: Schaub and Junkins, Analytical Mechanics of Space Systems.
 *
 * Rows: da, de, di, dOmega, domega, dM
 * Cols: f_R, f_T, f_N in Hill or LVLH frame.
 */
Eigen::Matrix<double, 6, 3> calculateClassicalBMatrix(double mu, const ClassicElements &oe_cl, BSKLogger& bskLogger)
{
    Eigen::Matrix<double, 6, 3> B;
    B.setZero();
    constexpr double eps = 1e-12;

    const double a     = oe_cl.a;
    const double e     = oe_cl.e;
    const double i     = oe_cl.i;
    const double omega = oe_cl.omega;
    const double f     = oe_cl.f;

    const double theta = omega + f;
    const double sinI  = std::sin(i);
    if (std::abs(sinI) <= eps) {
        bskLogger.bskLog(
            BSK_ERROR,
            "OrbitalElementControl::calculateClassicalBMatrix: singular inclination detected (sin(i) near zero).");
    }
    const double eta   = std::sqrt(1.0 - e * e);
    const double b     = a * eta;
    const double n     = std::sqrt(mu / std::pow(a, 3));
    const double h     = n * a * b;
    if (std::abs(h) <= eps) {
        bskLogger.bskLog(
            BSK_ERROR,
            "OrbitalElementControl::calculateClassicalBMatrix: singular orbit detected (angular momentum h near zero).");
    }
    if (std::abs(e) <= eps || std::abs(h * e) <= eps) {
        bskLogger.bskLog(
            BSK_ERROR,
            "OrbitalElementControl::calculateClassicalBMatrix: singular eccentricity detected (h*e near zero).");
    }
    const double p     = a * (1.0 - e * e);
    const double denom = (1.0 + e * std::cos(f));
    if (std::abs(denom) <= eps) {
        bskLogger.bskLog(
            BSK_ERROR,
            "OrbitalElementControl::calculateClassicalBMatrix: invalid geometry (1 + e*cos(f) near zero).");
    }
    const double r     = p / denom;

    // B matrix (rows: da, de, di, dOmega, domega, dM; cols: f_R, f_T, f_N)
    B(0, 0) = 2.0 * std::pow(a, 2) * e * std::sin(f) / h / a;
    B(0, 1) = 2.0 * std::pow(a, 2) * p / (h * r) / a;
    B(0, 2) = 0.0;

    B(1, 0) =  p * std::sin(f) / h;
    B(1, 1) = ((p + r) * std::cos(f) + r * e) / h;
    B(1, 2) = 0.0;

    B(2, 0) = 0.0;
    B(2, 1) = 0.0;
    B(2, 2) = r * std::cos(theta) / h;

    B(3, 0) = 0.0;
    B(3, 1) = 0.0;
    B(3, 2) = r * std::sin(theta) / (h * sinI);

    B(4, 0) = -p * std::cos(f) / (h * e);
    B(4, 1) = (p + r) * std::sin(f) / (h * e);
    B(4, 2) = -r * std::sin(theta) * std::cos(i) / (h * sinI);

    B(5, 0) =  eta * (p * std::cos(f) - 2.0 * r * e) / (h * e);
    B(5, 1) = -eta * (p + r) * std::sin(f) / (h * e);
    B(5, 2) = 0.0;

    return B;
}

bool isMatrixSPD(const Eigen::MatrixXd& mat)
{
    if ((mat - mat.transpose()).norm() > 1e-12) {
        return false;
    }

    const double minEigP = mat.selfadjointView<Eigen::Lower>().eigenvalues().minCoeff();
    if (minEigP <= 0.0) {
        return false;
    }

    return true;
}

}  // anonymous namespace

void
OrbitalElementControl::Reset(uint64_t CurrentSimNanos)
{
    // Base class checks A/B/C/D consistency.
    LinearTimeInvariantSystem::Reset(CurrentSimNanos);

    if (this->mu <= 0.0) {
        bskLogger.bskLog(
            BSK_ERROR,
            "OrbitalElementControl::Reset: mu must be set to a positive value.");
    }

    if (!this->targetOEInMsg.isLinked()) {
        bskLogger.bskLog(
            BSK_ERROR,
            "OrbitalElementControl::Reset: targetOEInMsg was not connected.");
    }
    if (!this->currentOEInMsg.isLinked()) {
        bskLogger.bskLog(
            BSK_ERROR,
            "OrbitalElementControl::Reset: currentOEInMsg was not connected.");
    }
}

void
OrbitalElementControl::setProportionalGain(const Eigen::MatrixXd& K)
{
    const size_t K_rows = static_cast<size_t>(K.rows());
    const size_t K_cols = static_cast<size_t>(K.cols());

    if (K_rows != 6 || K_cols != 6) {
        bskLogger.bskLog(
            BSK_ERROR,
            (
                std::string("OrbitalElementControl::setProportionalGain: gain has inconsistent dimensions. ") +
                "Expected size [6x6], but received [" +
                    std::to_string(K_rows) + " x " + std::to_string(K_cols) +
                "]."
            ).c_str()
        );
    }

    if (!isMatrixSPD(K))
    {
        bskLogger.bskLog(BSK_WARNING, "OrbitalElementControl::setProportionalGain: gain is not symmetric positive definite");
    }

    this->setD(K);
}

void
OrbitalElementControl::setIntegralGain(const Eigen::MatrixXd& K)
{
    const size_t K_rows = static_cast<size_t>(K.rows());
    const size_t K_cols = static_cast<size_t>(K.cols());

    if (K_rows != 6 || K_cols != 6) {
        bskLogger.bskLog(
            BSK_ERROR,
            (
                std::string("OrbitalElementControl::setIntegralGain: gain K has inconsistent dimensions. ") +
                "Expected size [6x6], but received [" +
                    std::to_string(K_rows) + " x " + std::to_string(K_cols) +
                "]."
            ).c_str()
        );
    }

    if (!isMatrixSPD(K))
    {
        bskLogger.bskLog(BSK_WARNING, "OrbitalElementControl::setIntegralGain: gain is not symmetric positive definite");
    }

    this->setB(Eigen::MatrixXd::Identity(6, 6));
    this->setC(K);
}

Eigen::VectorXd
OrbitalElementControl::readInput(uint64_t /*CurrentSimNanos*/)
{
    const size_t n = this->getInputSize();
    Eigen::VectorXd result = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(n));

    const ClassicElementsMsgPayload& targetOE  = this->targetOEInMsg();
    const ClassicElementsMsgPayload& currentOE = this->currentOEInMsg();

    if (targetOE.a == 0.0) {
        this->bskLogger.bskLog(
            BSK_ERROR,
            "OrbitalElementControl::readInput: targetOE.a is zero; target message may be unwritten.");
    }

    // Relative OE error: [da/a, de, di, dOmega, domega, dM]
    result(0) = (currentOE.a - targetOE.a) / targetOE.a;
    result(1) = currentOE.e - targetOE.e;
    result(2) = angleWarp(currentOE.i     - targetOE.i);
    result(3) = angleWarp(currentOE.Omega - targetOE.Omega);
    result(4) = angleWarp(currentOE.omega - targetOE.omega);

    const double currentE = f2E(currentOE.f, currentOE.e);
    const double currentM = E2M(currentE, currentOE.e);
    const double targetE  = f2E(targetOE.f,  targetOE.e);
    const double targetM  = E2M(targetE,  targetOE.e);

    result(5) = angleWarp(currentM - targetM);

    return result;
}

void
OrbitalElementControl::writeOutput(uint64_t CurrentSimNanos, const Eigen::VectorXd &y)
{
    // Copy current classical elements into ClassicElements struct.
    const ClassicElementsMsgPayload& inputElement = this->currentOEInMsg();
    ClassicElements elements;
    elements.a     = inputElement.a;
    elements.e     = inputElement.e;
    elements.i     = inputElement.i;
    elements.Omega = inputElement.Omega;
    elements.omega = inputElement.omega;
    elements.f     = inputElement.f;

    // Compute Gauss control matrix B (6x3) that maps force_LVLH to d(oe).
    Eigen::Matrix<double, 6, 3> B = calculateClassicalBMatrix(this->mu, elements, this->bskLogger);
    Eigen::Vector3d force_LVLH = -B.transpose() * y;

    // Current inertial position and velocity from elements.
    double r_N_arr[3];
    double v_N_arr[3];
    elem2rv(this->mu, &elements, r_N_arr, v_N_arr);

    auto r_N  = Eigen::Map<Eigen::Vector3d>(r_N_arr);
    auto v_N  = Eigen::Map<Eigen::Vector3d>(v_N_arr);

    // Build LVLH DCM H_N with rows R, T, N.
    Eigen::Vector3d ir   = r_N.normalized();
    Eigen::Vector3d hVec = r_N.cross(v_N);
    Eigen::Vector3d ih   = hVec.normalized();
    Eigen::Vector3d it   = ih.cross(ir);

    Eigen::Matrix3d H_N;
    H_N.row(0) = ir.transpose();  // radial
    H_N.row(1) = it.transpose();  // along track
    H_N.row(2) = ih.transpose();  // cross track

    // Transform LVLH force to inertial frame.
    Eigen::Vector3d force_N = H_N.transpose() * force_LVLH;

    CmdForceInertialMsgPayload payload;
    payload.forceRequestInertial[0] = force_N(0);
    payload.forceRequestInertial[1] = force_N(1);
    payload.forceRequestInertial[2] = force_N(2);

    this->forceOutMsg.write(&payload, moduleID, CurrentSimNanos);
}
