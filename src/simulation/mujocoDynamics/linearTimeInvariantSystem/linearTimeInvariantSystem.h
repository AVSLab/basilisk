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
#ifndef LINEAR_TIME_INVARIANT_SYSTEM_H
#define LINEAR_TIME_INVARIANT_SYSTEM_H

#include <cstdint>
#include <cstddef>

#include <Eigen/Dense>

#include "simulation/mujocoDynamics/_GeneralModuleFiles/StatefulSysModel.h"
#include "architecture/utilities/bskLogging.h"

/**
 * @brief Linear time invariant system of the form
 *        x_dot = A x + B u, y = C x + D u.
 *
 * Subclasses must implement the input and output mapping methods.
 */
class LinearTimeInvariantSystem : public StatefulSysModel {
public:
    /** @brief Default constructor. */
    LinearTimeInvariantSystem() = default;

    /** @name Framework interface @{ */

    /**
     * @brief Register the state vector with the dynamics framework.
     *
     * If the state dimension is greater than zero, this function allocates
     * a state block of size getStateSize() and initializes it to zero.
     *
     * @param registerer State registration helper provided by the framework.
     */
    void registerStates(DynParamRegisterer registerer) override;

    /**
     * @brief Compute the state derivative and system output at the current time.
     *
     * This method reads the current input vector using readInput, computes
     * the state derivative
     * x_dot = A x + B u and the output y = C x + D u, and writes the output
     * using writeOutput. If no state is registered, only the direct feedthrough
     * term y = D u is evaluated if applicable.
     *
     * @param CurrentSimNanos Current simulation time in nanoseconds.
     */
    void UpdateState(uint64_t CurrentSimNanos) override;

    /**
     * @brief Perform consistency checks and one time initialization.
     *
     * This method validates that the dimensions of the system matrices A, B,
     * C, and D are consistent with getStateSize, getInputSize, and
     * getOutputSize. Any inconsistency is reported through the logger.
     *
     * @param CurrentSimNanos Current simulation time in nanoseconds.
     */
    virtual void Reset(uint64_t CurrentSimNanos) override;

    /** @} */

    /** @name Parameter getters and setters
     *  Accessors for the system matrices A, B, C, and D.
     *  @{ */

    /**
     * @brief Get the state matrix A.
     *
     * @return Constant reference to the A matrix.
     */
    const Eigen::MatrixXd &getA() const;

    /**
     * @brief Set the state matrix A.
     *
     * This matrix defines the homogeneous term in the state equation
     * x_dot = A x + B u.
     *
     * @param Ain New A matrix.
     */
    void setA(const Eigen::MatrixXd &Ain);

    /**
     * @brief Get the input matrix B.
     *
     * @return Constant reference to the B matrix.
     */
    const Eigen::MatrixXd &getB() const;

    /**
     * @brief Set the input matrix B.
     *
     * This matrix defines the contribution of the input vector u to the
     * state equation x_dot = A x + B u.
     *
     * @param Bin New B matrix.
     */
    void setB(const Eigen::MatrixXd &Bin);

    /**
     * @brief Get the output matrix C.
     *
     * @return Constant reference to the C matrix.
     */
    const Eigen::MatrixXd &getC() const;

    /**
     * @brief Set the output matrix C.
     *
     * This matrix maps the state vector x to the output equation
     * y = C x + D u.
     *
     * @param Cin New C matrix.
     */
    void setC(const Eigen::MatrixXd &Cin);

    /**
     * @brief Get the feedthrough matrix D.
     *
     * @return Constant reference to the D matrix.
     */
    const Eigen::MatrixXd &getD() const;

    /**
     * @brief Set the feedthrough matrix D.
     *
     * This matrix maps the input vector u directly to the output equation
     * y = C x + D u.
     *
     * @param Din New D matrix.
     */
    void setD(const Eigen::MatrixXd &Din);

    /**
     * @brief Configure the A, B, C, D matrices as a standard SISO second order model.
     *
     * The resulting transfer function is:
     *   G(s) = k * wn^2 / (s^2 + 2*zeta*wn*s + wn^2)
     *
     * State space realization:
     *   x = [position, velocity]^T
     *   x_dot = [   0          1    ] x + [0          ] u
     *           [ -wn^2  -2*zeta*wn ]     [k * wn^2]
     *   y = [1 0] x
     *
     * @param wn   Natural frequency in rad/s. Must be positive.
     * @param zeta Damping ratio (dimensionless). Must be non negative.
     * @param k    Static gain. Default is unity.
     */
    void configureSecondOrder(double wn, double zeta, double k = 1.0);

    /**
     * @brief Configure A, B, C, D as a decoupled MIMO second order model.
     *
     * Each element of the input vectors defines one independent SISO channel:
     *
     *   G_i(s) = k_i * wn_i^2 / (s^2 + 2*zeta_i*wn_i*s + wn_i^2)
     *
     * The full state vector is
     *   x = [pos_0, vel_0, pos_1, vel_1, ..., pos_{n-1}, vel_{n-1}]^T
     *
     * For n channels, the resulting dimensions are:
     *   A: (2n x 2n), B: (2n x n), C: (n x 2n), D: (n x n).
     *
     * @param wn   Vector of natural frequencies in rad/s. Each wn(i) must be positive.
     * @param zeta Vector of damping ratios (dimensionless). Each zeta(i) must be non negative.
     * @param k    Vector of static gains. Must have the same length as wn and zeta.
     */
    void configureSecondOrder(const Eigen::VectorXd &wn,
                              const Eigen::VectorXd &zeta,
                              const Eigen::VectorXd &k);

    /** @} */

    /** @name State access
     *  Accessors for the internal state vector.
     *  @{ */

    /**
     * @brief Get the current state vector x.
     *
     * If the state has not been registered yet, an empty vector is returned
     * and an error is logged.
     *
     * @return Copy of the current state vector.
     */
    Eigen::VectorXd getX();

    /**
     * @brief Set the current state vector x.
     *
     * If the state has been registered and the supplied vector has the same
     * dimension as the stored state, the state is updated. Otherwise an
     * error is logged and the call is ignored.
     *
     * @param xin New state vector.
     */
    void setX(const Eigen::VectorXd &xin);

    /** @} */

    /**
     * @brief Get the dimension of the input vector u.
     *
     * The size is inferred from the B or D matrices when they are set.
     *
     * @return Size of the input vector.
     */
    virtual size_t getInputSize() const;

    /**
     * @brief Get the dimension of the state vector x.
     *
     * The size is inferred from the A, B, or C matrices when they are set.
     *
     * @return Size of the state vector.
     */
    virtual size_t getStateSize() const;

    /**
     * @brief Get the dimension of the output vector y.
     *
     * The size is inferred from the C or D matrices when they are set.
     *
     * @return Size of the output vector.
     */
    virtual size_t getOutputSize() const;

    /**
     * @brief Logger used to report errors and informational messages.
     */
    BSKLogger bskLogger;

protected:
    /**
     * @brief Read the current input vector u from the simulation.
     *
     * Subclasses must implement this method to construct the input vector
     * of size getInputSize() from the relevant input messages at the given
     * simulation time.
     *
     * @param CurrentSimNanos Current simulation time in nanoseconds.
     *
     * @return Input vector u.
     */
    virtual Eigen::VectorXd readInput(uint64_t CurrentSimNanos) = 0;

    /**
     * @brief Write the current output vector y to the simulation.
     *
     * Subclasses must implement this method to map the output vector
     * of size getOutputSize() to the appropriate output messages at the
     * given simulation time.
     *
     * @param CurrentSimNanos Current simulation time in nanoseconds.
     * @param y Output vector to write.
     */
    virtual void writeOutput(uint64_t CurrentSimNanos,
                             const Eigen::VectorXd &y) = 0;

private:
    /**
     * @brief Pointer to the registered state data for the state vector x.
     *
     * This pointer is set during registerStates and is used to read and
     * update the state and its derivative.
     */
    StateData *xState = nullptr;

    /**
     * @brief State matrix A.
     *
     * Defines the homogeneous dynamics x_dot = A x + B u.
     */
    Eigen::MatrixXd A;

    /**
     * @brief Input matrix B.
     *
     * Defines the input contribution to the dynamics x_dot = A x + B u.
     */
    Eigen::MatrixXd B;

    /**
     * @brief Output matrix C.
     *
     * Maps the state vector to the output y = C x + D u.
     */
    Eigen::MatrixXd C;

    /**
     * @brief Feedthrough matrix D.
     *
     * Maps the input vector directly to the output y = C x + D u.
     */
    Eigen::MatrixXd D;
};

#endif /* LINEAR_TIME_INVARIANT_SYSTEM_H */
