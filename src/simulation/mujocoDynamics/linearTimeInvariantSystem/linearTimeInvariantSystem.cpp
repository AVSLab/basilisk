/*
 ISC License

 Copyright (c) 2025, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#include "linearTimeInvariantSystem.h"
#include <string>

void LinearTimeInvariantSystem::registerStates(DynParamRegisterer registerer)
{
    const size_t stateSize = this->getStateSize();
    if (stateSize > 0) {
        xState = registerer.registerState(stateSize, 1, "x");
        xState->setState(Eigen::VectorXd::Constant(stateSize, 0.0)); // default to x0 = 0
    }
}

void LinearTimeInvariantSystem::UpdateState(uint64_t CurrentSimNanos)
{
    const size_t stateSize = this->getStateSize();
    const size_t inputSize = this->getInputSize();
    const size_t outputSize = this->getOutputSize();

    Eigen::VectorXd u;
    if (inputSize > 0) {
        u = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(inputSize));
        u = readInput(CurrentSimNanos);

        if (static_cast<size_t>(u.size()) != inputSize) {
            bskLogger.bskLog(
                BSK_ERROR,
                "LinearTimeInvariantSystem::UpdateState: readInput returned vector with incorrect size.");
        }
    }

    Eigen::VectorXd y = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(outputSize));

    if (xState && stateSize > 0) {
        Eigen::VectorXd x = xState->getState();

        if (static_cast<size_t>(x.size()) != stateSize) {
            bskLogger.bskLog(
                BSK_ERROR,
                "LinearTimeInvariantSystem::UpdateState: stored state has incorrect size.");
        }

        Eigen::VectorXd dx = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(stateSize));
        if (A.cols() > 0) {
            dx += A * x;
        }
        if (B.cols() > 0 && inputSize > 0) {
            dx += B * u;
        }
        xState->setDerivative(dx);

        if (C.cols() > 0) {
            y += C * x;
        }
    }

    if (D.size() > 0 && inputSize > 0) {
        y += D * u;
    }

    writeOutput(CurrentSimNanos, y);
}

void LinearTimeInvariantSystem::Reset(uint64_t /*CurrentSimNanos*/)
{
    const size_t stateSize  = this->getStateSize();
    const size_t inputSize  = this->getInputSize();
    const size_t outputSize = this->getOutputSize();

    // =======================
    // Matrix A (state x state)
    // =======================
    if (A.size() > 0) {
        const size_t A_rows = static_cast<size_t>(A.rows());
        const size_t A_cols = static_cast<size_t>(A.cols());

        if (A_rows != stateSize || A_cols != stateSize) {
            bskLogger.bskLog(
                BSK_ERROR,
                (
                    std::string("LinearTimeInvariantSystem::Reset: matrix A has inconsistent dimensions. ") +
                    "Expected size [" +
                        std::to_string(stateSize) + " x " + std::to_string(stateSize) +
                    "] based on getStateSize(), but received [" +
                        std::to_string(A_rows) + " x " + std::to_string(A_cols) +
                    "]."
                ).c_str()
            );
        }
    }

    // =======================
    // Matrix B (state x input)
    // =======================
    if (B.size() > 0) {
        const size_t B_rows = static_cast<size_t>(B.rows());
        const size_t B_cols = static_cast<size_t>(B.cols());

        if (B_rows != stateSize || B_cols != inputSize) {
            bskLogger.bskLog(
                BSK_ERROR,
                (
                    std::string("LinearTimeInvariantSystem::Reset: matrix B has inconsistent dimensions. ") +
                    "Expected size [" +
                        std::to_string(stateSize) + " x " + std::to_string(inputSize) +
                    "] based on getStateSize() and getInputSize(), but received [" +
                        std::to_string(B_rows) + " x " + std::to_string(B_cols) +
                    "]."
                ).c_str()
            );
        }
    }

    // ==========================
    // Matrix C (output x state)
    // ==========================
    if (C.size() > 0) {
        const size_t C_rows = static_cast<size_t>(C.rows());
        const size_t C_cols = static_cast<size_t>(C.cols());

        if (C_rows != outputSize || C_cols != stateSize) {
            bskLogger.bskLog(
                BSK_ERROR,
                (
                    std::string("LinearTimeInvariantSystem::Reset: matrix C has inconsistent dimensions. ") +
                    "Expected size [" +
                        std::to_string(outputSize) + " x " + std::to_string(stateSize) +
                    "] based on getOutputSize() and getStateSize(), but received [" +
                        std::to_string(C_rows) + " x " + std::to_string(C_cols) +
                    "]."
                ).c_str()
            );
        }
    }

    // =========================
    // Matrix D (output x input)
    // =========================
    if (D.size() > 0) {
        const size_t D_rows = static_cast<size_t>(D.rows());
        const size_t D_cols = static_cast<size_t>(D.cols());

        if (D_rows != outputSize || D_cols != inputSize) {
            bskLogger.bskLog(
                BSK_ERROR,
                (
                    std::string("LinearTimeInvariantSystem::Reset: matrix D has inconsistent dimensions. ") +
                    "Expected size [" +
                        std::to_string(outputSize) + " x " + std::to_string(inputSize) +
                    "] based on getOutputSize() and getInputSize(), but received [" +
                        std::to_string(D_rows) + " x " + std::to_string(D_cols) +
                    "]."
                ).c_str()
            );
        }
    }
}

const Eigen::MatrixXd &LinearTimeInvariantSystem::getA() const
{
    return A;
}

void LinearTimeInvariantSystem::setA(const Eigen::MatrixXd &Ain)
{
    A = Ain;
}

const Eigen::MatrixXd &LinearTimeInvariantSystem::getB() const
{
    return B;
}

void LinearTimeInvariantSystem::setB(const Eigen::MatrixXd &Bin)
{
    B = Bin;
}

const Eigen::MatrixXd &LinearTimeInvariantSystem::getC() const
{
    return C;
}

void LinearTimeInvariantSystem::setC(const Eigen::MatrixXd &Cin)
{
    C = Cin;
}

const Eigen::MatrixXd &LinearTimeInvariantSystem::getD() const
{
    return D;
}

void LinearTimeInvariantSystem::setD(const Eigen::MatrixXd &Din)
{
    D = Din;
}

void LinearTimeInvariantSystem::configureSecondOrder(double wn, double zeta, double k)
{
    if (wn <= 0.0) {
        bskLogger.bskLog(
            BSK_ERROR,
            "configureSecondOrder: natural frequency wn must be positive.");
        return;
    }
    if (zeta < 0.0) {
        bskLogger.bskLog(
            BSK_ERROR,
            "configureSecondOrder: damping ratio zeta must be non negative.");
        return;
    }

    A.resize(2, 2);
    B.resize(2, 1);
    C.resize(1, 2);
    D.resize(1, 1);

    const double wn2 = wn * wn;

    A << 0.0 ,    1.0,
         -wn2,   -2.0 * zeta * wn;

    B << 0.0,
         k * wn2;

    C << 1.0, 0.0;

    D << 0.0;
}

void LinearTimeInvariantSystem::configureSecondOrder(const Eigen::VectorXd &wn,
                                                     const Eigen::VectorXd &zeta,
                                                     const Eigen::VectorXd &k)
{
    const Eigen::Index n = wn.size();

    if (n <= 0) {
        bskLogger.bskLog(
            BSK_ERROR,
            "LinearTimeInvariantSystem::configureSecondOrder(MIMO): input vectors must be non empty."
        );
        return;
    }

    if (zeta.size() != n || k.size() != n) {
        bskLogger.bskLog(
            BSK_ERROR,
            (
                std::string("LinearTimeInvariantSystem::configureSecondOrder(MIMO): size mismatch. ") +
                "Expected wn, zeta, k to all have length " + std::to_string(static_cast<long long>(n)) + ". "
                "Received lengths wn=" + std::to_string(static_cast<long long>(wn.size())) +
                ", zeta=" + std::to_string(static_cast<long long>(zeta.size())) +
                ", k=" + std::to_string(static_cast<long long>(k.size())) + "."
            ).c_str()
        );
        return;
    }

    // Validate entries
    for (Eigen::Index i = 0; i < n; ++i) {
        const double wn_i   = wn(i);
        const double zeta_i = zeta(i);

        if (wn_i <= 0.0) {
            bskLogger.bskLog(
                BSK_ERROR,
                (
                    std::string("LinearTimeInvariantSystem::configureSecondOrder(MIMO): wn(") +
                    std::to_string(static_cast<long long>(i)) +
                    ") must be positive, but is " + std::to_string(wn_i) + "."
                ).c_str()
            );
            return;
        }

        if (zeta_i < 0.0) {
            bskLogger.bskLog(
                BSK_ERROR,
                (
                    std::string("LinearTimeInvariantSystem::configureSecondOrder(MIMO): zeta(") +
                    std::to_string(static_cast<long long>(i)) +
                    ") must be non negative, but is " + std::to_string(zeta_i) + "."
                ).c_str()
            );
            return;
        }
    }

    // Allocate matrices for MIMO second-order system: n channels, each 2 states
    const Eigen::Index stateSize  = 2 * n;
    const Eigen::Index inputSize  = n;
    const Eigen::Index outputSize = n;

    A.setZero(stateSize, stateSize);
    B.setZero(stateSize, inputSize);
    C.setZero(outputSize, stateSize);
    D.setZero(outputSize, inputSize);

    for (Eigen::Index i = 0; i < n; ++i) {
        const double wn_i   = wn(i);
        const double zeta_i = zeta(i);
        const double k_i    = k(i);

        const double wn2 = wn_i * wn_i;

        const Eigen::Index rowPos = 2 * i;     // position state index
        const Eigen::Index rowVel = 2 * i + 1; // velocity state index

        // A block for channel i:
        // [  0           1     ]
        // [ -wn_i^2  -2*zeta_i*wn_i ]
        A(rowPos, rowVel) = 1.0;
        A(rowVel, rowPos) = -wn2;
        A(rowVel, rowVel) = -2.0 * zeta_i * wn_i;

        // B column i: [0, k_i * wn_i^2]^T in rows (rowPos, rowVel)
        B(rowVel, i) = k_i * wn2;

        // C row i: [1 0] on the position state of channel i
        C(i, rowPos) = 1.0;

        // D(i, i) remains zero (direct feedthrough = 0)
    }
}

Eigen::VectorXd
LinearTimeInvariantSystem::getX()
{
    if (!xState) {
        bskLogger.bskLog(
            BSK_ERROR,
            "LinearTimeInvariantSystem::getX: state has not been registered.");
        return Eigen::VectorXd();
    }
    return xState->getState();
}

void LinearTimeInvariantSystem::setX(const Eigen::VectorXd &xin)
{
    if (!xState) {
        bskLogger.bskLog(
            BSK_ERROR,
            "LinearTimeInvariantSystem::setX: state has not been registered.");
        return;
    }

    const size_t stateSize = this->getStateSize();
    if (static_cast<size_t>(xin.size()) != stateSize) {
        bskLogger.bskLog(
            BSK_ERROR,
            "LinearTimeInvariantSystem::setX: input state vector has incorrect size.");
        return;
    }

    xState->setState(xin);
}

size_t LinearTimeInvariantSystem::getInputSize() const
{
    if (B.cols() > 0) {
        return static_cast<size_t>(B.cols());
    }
    if (D.cols() > 0) {
        return static_cast<size_t>(D.cols());
    }
    return 0;
}

size_t LinearTimeInvariantSystem::getStateSize() const
{
    if (A.cols() > 0) {
        return static_cast<size_t>(A.cols());
    }
    if (B.rows() > 0) {
        return static_cast<size_t>(B.rows());
    }
    if (C.cols() > 0) {
        return static_cast<size_t>(C.cols());
    }
    return 0;
}

size_t LinearTimeInvariantSystem::getOutputSize() const
{
    if (C.rows() > 0) {
        return static_cast<size_t>(C.rows());
    }
    if (D.rows() > 0) {
        return static_cast<size_t>(D.rows());
    }
    return 0;
}
